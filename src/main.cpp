/*
 * MICROMOUSE - ESP32-S3 - independent wall following + Mahony filter
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "Config.h"
#include "Globals.h"
#include "IMU.h"
#include "Sensors.h"
#include "Motor.h"
#include "WallAvoidance.h"
#include "Web.h"
#include <math.h>

namespace
{
const int SENSOR_L = 0;
const int SENSOR_FL = 1;
const int SENSOR_FR = 2;
const int SENSOR_R = 3;
const int FRONT_STOP_SENSOR_LEFT = SENSOR_L;
const int FRONT_STOP_SENSOR_RIGHT = SENSOR_R;

const int CONTROL_PERIOD_MS = 10;
const float CONTROL_DT_SECONDS = CONTROL_PERIOD_MS / 1000.0f;
const int SENSOR_DELTA_PWM_GAIN = 0;
const int FRONT_STOP_BRAKE_RAMP_MULTIPLIER = 3;
const uint32_t MAIN_CONTROL_STACK_SIZE = 8192;
const uint32_t CONTROL_TASK_WDT_TIMEOUT_SECONDS = 2;
const uint32_t CONTROL_LOOP_SOFT_TIMEOUT_MS = 50;
const int IR_STEER_FILTER_KEEP = 4;
const int IR_STEER_FILTER_TOTAL = 5;
const uint32_t MAZE_POST_BACKUP_SETTLE_MS = 180;
const uint32_t MAZE_CELL_DECISION_WAIT_MS = 3000;
const uint32_t MAZE_CENTER_SETTLE_MS = 500;
const uint32_t POINT_TURN_SETTLE_MS = 180;
const uint32_t POINT_TURN_TOTAL_TIMEOUT_MS = 3000;

long lastSpeedPulseL = 0;
long lastSpeedPulseR = 0;

void changeState(RunState newState);
void resetRunProfileAndPidMemory();
int sideCloseDirection(int sensorIndex, int sideRef, const RuntimeParams &cfg);
int computeHeadingHoldSteer(const RuntimeParams &cfg, int limit);
void startBackDrive(long targetPulses);
bool updateBackDrive(const RuntimeParams &cfg);
bool updatePointTurnCore(const RuntimeParams &cfg);
bool updatePointTurn(const RuntimeParams &cfg);

enum MazePhase
{
  MAZE_DECIDE,
  MAZE_SETTLE_AFTER_BACKUP,
  MAZE_DECIDE_AFTER_BACKUP,
  MAZE_WAIT_AT_CELL,
  MAZE_CENTER_AFTER_TURN,
  MAZE_TURNING,
  MAZE_RUN_BEFORE_TURN,
  MAZE_RUN_TO_WALL,
  MAZE_BRAKE_AT_CELL,
  MAZE_BRAKE_AT_WALL,
  MAZE_BACK_UP_UNTIL_TURN,
  MAZE_BACK_UP_AFTER_DEAD_END
};

enum PointTurnPhase
{
  POINT_TURN_COARSE,
  POINT_TURN_SETTLE,
  POINT_TURN_FINE
};

enum CellDrivePhase
{
  CELL_DRIVE_RUN,
  CELL_DRIVE_BRAKE,
  CELL_DRIVE_SETTLE,
  CELL_DRIVE_FINE,
  CELL_DRIVE_HEADING_FINE
};

struct MazeWalls
{
  bool left;
  bool front;
  bool right;
};

MazePhase mazePhase = MAZE_DECIDE;
float mazeTurnStartYaw = 0.0f;
float mazeTurnDegrees = 0.0f;
uint32_t mazeTurnStartMs = 0;
uint32_t mazeSettleStartMs = 0;
long mazeLateTurnStartPulse = 0;
bool mazeHasQueuedTurn = false;
float mazeQueuedTurnDegrees = 0.0f;
PointTurnPhase pointTurnPhase = POINT_TURN_COARSE;
uint32_t pointTurnSettleStartMs = 0;
CellDrivePhase cellDrivePhase = CELL_DRIVE_RUN;
uint32_t cellDrivePhaseStartMs = 0;
int cellDriveFineAttempts = 0;
CellDrivePhase backDrivePhase = CELL_DRIVE_RUN;
uint32_t backDrivePhaseStartMs = 0;
int backDriveFineAttempts = 0;
long backDriveTargetPulses = 0;
bool pointTurnPostBackupEnabled = false;
bool pointTurnPostBackupRunning = false;

long absPulse(long value)
{
  return value < 0 ? -value : value;
}

long averageTravelPulse()
{
  PulseSnapshot pulses = getPulseSnapshot();
  return (absPulse(pulses.left) + absPulse(pulses.right)) / 2;
}

float mazeTurnAngleDeg(const RuntimeParams &cfg)
{
  return constrain(cfg.maze_turn_angle_deg, 45.0f, 90.0f);
}

float mazeTurnRightDeg(const RuntimeParams &cfg)
{
  return -mazeTurnAngleDeg(cfg);
}

float mazeTurnLeftDeg(const RuntimeParams &cfg)
{
  return mazeTurnAngleDeg(cfg);
}

bool isMazeRunState(RunState state)
{
  return state == MAZE_RIGHT_HAND || state == MAZE_RIGHT_HAND_CELL;
}

bool isMazeCellRunState()
{
  return carState == MAZE_RIGHT_HAND_CELL;
}

bool isPointTurnTestState(RunState state)
{
  return state == TEST_TURN_L || state == TEST_TURN_R;
}

bool isBackDriveTestState(RunState state)
{
  return state == TEST_BACK_ONE_CELL;
}

int sensorValueByIndex(const IrSnapshot &ir, int sensorIndex)
{
  switch (sensorIndex)
  {
  case SENSOR_L:
    return ir.left;
  case SENSOR_FL:
    return ir.frontLeft;
  case SENSOR_FR:
    return ir.frontRight;
  case SENSOR_R:
    return ir.right;
  default:
    return 0;
  }
}

bool isSensorCalibrated(int sensorIndex, const RuntimeParams &cfg)
{
  int margin = max(50, cfg.ir_deadband / 2);
  return max_IR[sensorIndex] < 4095 - margin;
}

bool isMazeSensorWallCalibrated(int sensorIndex, const RuntimeParams &cfg)
{
  int margin = max(50, cfg.ir_deadband / 2);
  return max_IR[sensorIndex] < 4095 - margin &&
         min_IR[sensorIndex] > margin;
}

int frontCorrectionStart(int sensorIndex, const RuntimeParams &cfg)
{
  if (!isSensorCalibrated(sensorIndex, cfg))
    return 4095;
  return constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
}

bool isFrontWallDetected(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  int leftThreshold = max(0, frontCorrectionStart(FRONT_STOP_SENSOR_LEFT, cfg) -
                                 cfg.front_stop_early_margin);
  int rightThreshold = max(0, frontCorrectionStart(FRONT_STOP_SENSOR_RIGHT, cfg) -
                                  cfg.front_stop_early_margin);

  bool frontLeftWarn = isSensorCalibrated(FRONT_STOP_SENSOR_LEFT, cfg) &&
                       sensorValueByIndex(ir, FRONT_STOP_SENSOR_LEFT) >
                           leftThreshold;
  bool frontRightWarn = isSensorCalibrated(FRONT_STOP_SENSOR_RIGHT, cfg) &&
                        sensorValueByIndex(ir, FRONT_STOP_SENSOR_RIGHT) >
                            rightThreshold;

  return frontLeftWarn && frontRightWarn;
}

int computeWheelSpeedPid(float targetSpeed, float actualSpeed, float kp, float ki,
                         float kd, float &integral, float &prevErr,
                         float &dFilter)
{
  float error = targetSpeed - actualSpeed;
  integral = constrain(integral + error, -120.0f, 120.0f);

  float derivative = error - prevErr;
  dFilter = 0.7f * dFilter + 0.3f * derivative;
  prevErr = error;

  return (int)(kp * error + ki * integral + kd * dFilter);
}

int keepMotorRunningPwm(int pwm, const RuntimeParams &cfg)
{
  return constrain(pwm, cfg.Turn_Min, cfg.Turn_Max);
}

int sideReferenceByIndex(int sensorIndex)
{
  if (sensorIndex == SENSOR_FL)
    return side_ref_FL;
  if (sensorIndex == SENSOR_FR)
    return side_ref_FR;
  return (sensorIndex == SENSOR_L) ? side_ref_L : side_ref_R;
}

bool isMazeSensorReady(int sensorIndex, const RuntimeParams &cfg)
{
  return isMazeSensorWallCalibrated(sensorIndex, cfg) ||
         sideReferenceByIndex(sensorIndex) < 4095;
}

bool isMazeWallBySensor(int sensorIndex, const IrSnapshot &ir,
                        const RuntimeParams &cfg)
{
  int value = sensorValueByIndex(ir, sensorIndex);
  int deadband = constrain(cfg.ir_deadband / 4, 10, 60);

  if (isMazeSensorWallCalibrated(sensorIndex, cfg))
  {
    int wallRef = constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
    int emptyRef = constrain(min_IR[sensorIndex] - cfg.offset_lower, 0, 4095);
    int diff = wallRef - emptyRef;
    if (abs(diff) <= deadband)
      return false;

    int threshold = (wallRef + emptyRef) / 2;
    return (diff > 0) ? (value >= threshold) : (value <= threshold);
  }

  int sideRef = sideReferenceByIndex(sensorIndex);
  if (sideRef < 4095)
  {
    int direction = sideCloseDirection(sensorIndex, sideRef, cfg);
    return (direction > 0) ? (value >= sideRef + deadband)
                           : (value <= sideRef - deadband);
  }

  return false;
}

bool isMazeFrontWallBySensor(int sensorIndex, const IrSnapshot &ir,
                             const RuntimeParams &cfg)
{
  int value = sensorValueByIndex(ir, sensorIndex);
  int deadband = constrain(cfg.ir_deadband / 4, 10, 60);

  if (isMazeSensorWallCalibrated(sensorIndex, cfg))
  {
    int wallRef = constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
    int emptyRef = constrain(min_IR[sensorIndex] - cfg.offset_lower, 0, 4095);
    int diff = wallRef - emptyRef;
    if (abs(diff) <= deadband)
      return false;

    int margin = constrain(cfg.front_stop_early_margin, 0, 4095);
    int threshold = (diff > 0) ? (wallRef - margin) : (wallRef + margin);
    threshold = constrain(threshold, min(wallRef, emptyRef),
                          max(wallRef, emptyRef));
    return (diff > 0) ? (value >= threshold) : (value <= threshold);
  }

  return isMazeWallBySensor(sensorIndex, ir, cfg);
}

bool isMazeSideOpenForTurn(int sensorIndex, const IrSnapshot &ir,
                           const RuntimeParams &cfg)
{
  if (!isMazeSensorWallCalibrated(sensorIndex, cfg))
    return false;

  int value = sensorValueByIndex(ir, sensorIndex);
  int wallRef = constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
  int emptyRef = constrain(min_IR[sensorIndex] - cfg.offset_lower, 0, 4095);
  int span = abs(wallRef - emptyRef);
  int openDistance = max(span / 2, max(30, cfg.ir_deadband / 2));

  if (span <= openDistance)
    return false;

  return (wallRef > emptyRef) ? (value <= wallRef - openDistance)
                              : (value >= wallRef + openDistance);
}

MazeWalls readMazeWalls(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  MazeWalls walls = {};
  walls.front = isMazeFrontWallBySensor(SENSOR_L, ir, cfg) ||
                isMazeFrontWallBySensor(SENSOR_R, ir, cfg);
  walls.left = isMazeWallBySensor(SENSOR_FL, ir, cfg);
  walls.right = isMazeWallBySensor(SENSOR_FR, ir, cfg);

  return walls;
}

void resetMazeRunner()
{
  mazePhase = MAZE_RUN_TO_WALL;
  mazeTurnStartYaw = continuousYaw;
  mazeTurnDegrees = 0.0f;
  mazeTurnStartMs = 0;
  mazeSettleStartMs = 0;
  mazeLateTurnStartPulse = 0;
  mazeHasQueuedTurn = false;
  mazeQueuedTurnDegrees = 0.0f;
  pointTurnPhase = POINT_TURN_COARSE;
  pointTurnSettleStartMs = 0;
  pointTurnPostBackupEnabled = false;
  pointTurnPostBackupRunning = false;
  backDriveTargetPulses = 0;
  target_yaw = continuousYaw;
  isTurningTask = false;
}

void queueMazeTurn(float turnDegrees)
{
  if (mazeHasQueuedTurn)
    return;

  mazeQueuedTurnDegrees = turnDegrees;
  mazeHasQueuedTurn = true;
}

void resetCellDriveController()
{
  cellDrivePhase = CELL_DRIVE_RUN;
  cellDrivePhaseStartMs = 0;
  cellDriveFineAttempts = 0;
}

void resetBackDriveController()
{
  backDrivePhase = CELL_DRIVE_RUN;
  backDrivePhaseStartMs = 0;
  backDriveFineAttempts = 0;
  backDriveTargetPulses = 0;
}

void beginPointTurn(float turnDegrees, bool allowPostBackup = true)
{
  mazeTurnDegrees = turnDegrees;
  mazeTurnStartYaw = continuousYaw;
  target_yaw = mazeTurnStartYaw + turnDegrees;
  mazeTurnStartMs = millis();
  pointTurnPhase = POINT_TURN_COARSE;
  pointTurnSettleStartMs = 0;
  pointTurnPostBackupEnabled = allowPostBackup;
  pointTurnPostBackupRunning = false;
  isTurningTask = true;
}

void startMazeTurn(float turnDegrees)
{
  mazePhase = MAZE_TURNING;
  beginPointTurn(turnDegrees);
}

void startQueuedMazeTurn()
{
  float turnDegrees = mazeQueuedTurnDegrees;
  mazeHasQueuedTurn = false;
  mazeQueuedTurnDegrees = 0.0f;
  startMazeTurn(turnDegrees);
}

void startMazeCenterAfterTurn()
{
  brakeMotors();
  resetMotorsAndPID();
  resetRunProfileAndPidMemory();
  mazePhase = MAZE_CENTER_AFTER_TURN;
  mazeSettleStartMs = millis();
  isTurningTask = false;
}

void startMazeLateTurn(float turnDegrees, const RuntimeParams &cfg)
{
  int latePulses = max(0, cfg.maze_turn_late_pulses);
  if (latePulses <= 0)
  {
    startMazeTurn(turnDegrees);
    return;
  }

  mazePhase = MAZE_RUN_BEFORE_TURN;
  mazeTurnDegrees = turnDegrees;
  mazeLateTurnStartPulse = averageTravelPulse();
  isTurningTask = false;
}

void startMazeDeadEndBackup()
{
  resetMotorsAndPID();
  resetRunProfileAndPidMemory();
  mazeHasQueuedTurn = false;
  mazeQueuedTurnDegrees = 0.0f;
  mazePhase = isMazeCellRunState() ? MAZE_BACK_UP_UNTIL_TURN
                                   : MAZE_BACK_UP_AFTER_DEAD_END;
  isTurningTask = false;
}

void startMazeCellWait()
{
  brakeMotors();
  resetMotorsAndPID();
  resetRunProfileAndPidMemory();
  target_yaw = continuousYaw;
  mazeSettleStartMs = millis();
  mazePhase = MAZE_WAIT_AT_CELL;
  isTurningTask = false;
}

void startMazePostBackupDecision()
{
  brakeMotors();
  resetMotorsAndPID();
  resetRunProfileAndPidMemory();
  target_yaw = continuousYaw;
  mazePhase = MAZE_SETTLE_AFTER_BACKUP;
  mazeSettleStartMs = millis();
  isTurningTask = false;
}

void startMazeStraightRun()
{
  resetMotorsAndPID();
  resetRunProfileAndPidMemory();
  mazePhase = MAZE_RUN_TO_WALL;
  isTurningTask = false;
}

int sideWallCorrectionStart(int sensorIndex, const RuntimeParams &cfg)
{
  int sideRef = sideReferenceByIndex(sensorIndex);
  if (sideRef < 4095)
    return sideRef;
  if (max_IR[sensorIndex] >= 4095 - max(10, cfg.offset_upper))
    return 4095;
  return constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
}

int sideErrorDeadband(const RuntimeParams &cfg)
{
  return constrain(cfg.ir_deadband / 8, 6, 24);
}

int sideCorrectionKick(const RuntimeParams &cfg)
{
  return constrain(cfg.Turn_Min / 4, 5, 12);
}

int sideSteerLimit(const RuntimeParams &cfg)
{
  int maxLimit = max(4, cfg.Turn_Max - cfg.Turn_Min);
  return constrain(cfg.wall_steer_limit, 4, maxLimit);
}

int mazeCenterDeadband(const RuntimeParams &cfg)
{
  return constrain(cfg.ir_deadband / 4, 18, 70);
}

int mazeCenterSteerLimit(const RuntimeParams &cfg)
{
  int softLimit = max(4, cfg.wall_steer_limit / 2);
  return constrain(softLimit, 4, sideSteerLimit(cfg));
}

int mazeIrSteerRateLimit(const RuntimeParams &cfg)
{
  return constrain(max(1, cfg.ramp_rate / 4), 1, 4);
}

int driveSteerLimit(const RuntimeParams &cfg)
{
  int maxLimit = max(10, cfg.Turn_Max - cfg.Turn_Min);
  int baseLimit = max(cfg.wall_steer_limit, (cfg.Turn_Max - cfg.Turn_Min) / 3);
  return constrain(baseLimit, 10, maxLimit);
}

int computeHeadingHoldSteer(const RuntimeParams &cfg, int limit)
{
  float yawError = continuousYaw - target_yaw;
  return constrain((int)(yawError * cfg.k_gyro), -limit, limit);
}

bool updatePointTurnCore(const RuntimeParams &cfg)
{
  float yawRemainingSigned = target_yaw - continuousYaw;
  float yawRemaining = fabsf(yawRemainingSigned);
  float coarseTolerance = max(2.0f, cfg.Turn_Err);
  float fineTolerance = constrain(max(0.6f, cfg.Turn_Err), 0.6f, 1.2f);
  debugTotalSteer = (int)yawRemainingSigned;
  debugSteerIR = 0;

  if (pointTurnPhase == POINT_TURN_SETTLE)
  {
    brakeMotors();
    if (millis() - pointTurnSettleStartMs < POINT_TURN_SETTLE_MS)
      return false;

    if (yawRemaining <= fineTolerance)
    {
      target_yaw = mazeTurnStartYaw + mazeTurnDegrees;
      pointTurnPhase = POINT_TURN_COARSE;
      pointTurnSettleStartMs = 0;
      isTurningTask = false;
      return true;
    }

    pointTurnPhase = POINT_TURN_FINE;
  }

  float activeTolerance =
      (pointTurnPhase == POINT_TURN_FINE) ? fineTolerance : coarseTolerance;
  if (yawRemaining <= activeTolerance)
  {
    brakeMotors();
    pointTurnPhase = POINT_TURN_SETTLE;
    pointTurnSettleStartMs = millis();
    return false;
  }

  if (millis() - mazeTurnStartMs > POINT_TURN_TOTAL_TIMEOUT_MS)
  {
    changeState(IDLE);
    return false;
  }

  int turnMin = cfg.Turn_Min;
  int turnMax = cfg.Turn_Max;
  if (pointTurnPhase == POINT_TURN_FINE)
  {
    turnMin = constrain(max(12, cfg.Turn_Min - 10), 10, cfg.Turn_Min);
    turnMax = constrain(max(turnMin, cfg.Turn_Min + 8), turnMin, cfg.Turn_Max);
  }

  int turnPwm =
      constrain((int)(yawRemaining * cfg.k_gyro), turnMin, turnMax);
  int targetPwm_L = (yawRemainingSigned < 0.0f) ? turnPwm : -turnPwm;
  int targetPwm_R = (yawRemainingSigned < 0.0f) ? -turnPwm : turnPwm;
  int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
  int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);
  setLeftMotor(newPwm_L);
  setRightMotor(newPwm_R);
  return false;
}

bool updatePointTurn(const RuntimeParams &cfg)
{
  if (pointTurnPostBackupRunning)
  {
    if (updateBackDrive(cfg))
    {
      pointTurnPostBackupRunning = false;
      return true;
    }
    return false;
  }

  if (!updatePointTurnCore(cfg))
    return false;

  if (pointTurnPostBackupEnabled && cfg.point_turn_backup_pulses > 0)
  {
    pointTurnPostBackupEnabled = false;
    pointTurnPostBackupRunning = true;
    startBackDrive(cfg.point_turn_backup_pulses);
    return false;
  }

  pointTurnPostBackupEnabled = false;
  return true;
}

int sideCloseDirection(int sensorIndex, int sideRef, const RuntimeParams &cfg)
{
  int deadband = sideErrorDeadband(cfg);
  int wallRaw = -1;
  int emptyRaw = -1;

  if (max_IR[sensorIndex] < 4095 - max(10, cfg.offset_upper))
  {
    wallRaw = constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
  }
  if (min_IR[sensorIndex] > max(10, cfg.offset_lower))
  {
    emptyRaw = constrain(min_IR[sensorIndex] - cfg.offset_lower, 0, 4095);
  }

  if (wallRaw >= 0 && abs(wallRaw - sideRef) > deadband)
  {
    return (wallRaw > sideRef) ? 1 : -1;
  }
  if (emptyRaw >= 0 && abs(emptyRaw - sideRef) > deadband)
  {
    return (sideRef > emptyRaw) ? 1 : -1;
  }
  return -1;
}

int sideCloseError(int sensorIndex, const IrSnapshot &ir, const RuntimeParams &cfg)
{
  int sideRef = sideWallCorrectionStart(sensorIndex, cfg);
  if (sideRef >= 4095)
    return 0;

  int sideValue = sensorValueByIndex(ir, sensorIndex);
  int direction = sideCloseDirection(sensorIndex, sideRef, cfg);
  int error = (direction > 0) ? (sideValue - sideRef) : (sideRef - sideValue);

  return (error > sideErrorDeadband(cfg)) ? error : 0;
}

int correctFromSideWalls(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  int leftError = sideCloseError(SENSOR_FL, ir, cfg);
  int rightError = sideCloseError(SENSOR_FR, ir, cfg);
  debugSideErrorL = leftError;
  debugSideErrorR = rightError;
  if (leftError == 0 && rightError == 0)
    return 0;

  int rawSteer = leftError - rightError;
  int steer = (int)(rawSteer * cfg.k_ir);
  if (steer > 0)
    steer = max(steer, sideCorrectionKick(cfg));
  else if (steer < 0)
    steer = min(steer, -sideCorrectionKick(cfg));

  int limit = sideSteerLimit(cfg);
  steer = constrain(steer, -limit, limit);
  debugSteerIR = steer;
  return steer;
}

int signedSideCenterError(int sensorIndex, const IrSnapshot &ir,
                          const RuntimeParams &cfg)
{
  int sideRef = sideReferenceByIndex(sensorIndex);
  if (sideRef >= 4095)
    return 0;

  int sideValue = sensorValueByIndex(ir, sensorIndex);
  int direction = sideCloseDirection(sensorIndex, sideRef, cfg);
  int error = (direction > 0) ? (sideValue - sideRef)
                              : (sideRef - sideValue);
  int deadband = mazeCenterDeadband(cfg);

  return (abs(error) > deadband) ? error : 0;
}

int computeMazeCenteringSteer(const IrSnapshot &ir, const MazeWalls &walls,
                              const RuntimeParams &cfg)
{
  int leftError = walls.left ? signedSideCenterError(SENSOR_FL, ir, cfg) : 0;
  int rightError = walls.right ? signedSideCenterError(SENSOR_FR, ir, cfg) : 0;

  debugSideErrorL = leftError;
  debugSideErrorR = rightError;

  if (leftError == 0 && rightError == 0)
    return 0;

  int rawSteer = leftError - rightError;
  float gainScale = (walls.left && walls.right) ? 0.65f : 0.45f;
  int steer = (int)(rawSteer * cfg.k_ir * gainScale);
  if (abs(steer) <= 1)
    return 0;

  int limit = mazeCenterSteerLimit(cfg);
  return constrain(steer, -limit, limit);
}

int oneCellStopTolerance(const RuntimeParams &cfg)
{
  return constrain(cfg.pulses_per_cell / 90, 8, 18);
}

int oneCellBrakeLeadPulses(const RuntimeParams &cfg)
{
  return constrain(cfg.pulses_per_cell / 8, 80, 180);
}

int oneCellFinePwm(const RuntimeParams &cfg)
{
  return constrain(max(18, cfg.Turn_Min - 8), 15, cfg.Turn_Max);
}

float oneCellHeadingTolerance(const RuntimeParams &cfg)
{
  return constrain(max(0.8f, cfg.Turn_Err), 0.8f, 1.5f);
}

void startOneCellHeadingCorrection()
{
  float headingError = target_yaw - continuousYaw;
  beginPointTurn(headingError, false);
  cellDrivePhase = CELL_DRIVE_HEADING_FINE;
}

int computeEncoderBalanceSteer(long travelL, long travelR,
                               const RuntimeParams &cfg)
{
  int limit = max(3, mazeCenterSteerLimit(cfg) / 2);
  return constrain((int)((travelR - travelL) * 0.06f), -limit, limit);
}

bool brakeOneCellDrive(const RuntimeParams &cfg)
{
  int brakeRamp = max(1, cfg.ramp_rate * FRONT_STOP_BRAKE_RAMP_MULTIPLIER);
  int newPwm_L = applyRateLimit(0, pwmL, brakeRamp);
  int newPwm_R = applyRateLimit(0, pwmR, brakeRamp);
  setLeftMotor(newPwm_L);
  setRightMotor(newPwm_R);
  return abs(newPwm_L) <= 3 && abs(newPwm_R) <= 3;
}

int distanceStopTolerance(long targetPulses)
{
  int target = (targetPulses > 5000) ? 5000 : (int)targetPulses;
  target = max(1, target);
  return constrain(target / 90, 6, 18);
}

int distanceBrakeLeadPulses(long targetPulses)
{
  int target = (targetPulses > 5000) ? 5000 : (int)targetPulses;
  target = max(1, target);
  return constrain(target / 8, 20, 180);
}

void startBackDrive(long targetPulses)
{
  resetMotorsAndPID();
  resetRunProfileAndPidMemory();
  resetBackDriveController();
  backDriveTargetPulses = (targetPulses < 0) ? -targetPulses : targetPulses;
}

bool updateBackDrive(const RuntimeParams &cfg)
{
  if (backDriveTargetPulses <= 0)
  {
    resetBackDriveController();
    return true;
  }

  PulseSnapshot pulses = getPulseSnapshot();
  long currentPulseL = pulses.left;
  long currentPulseR = pulses.right;
  long travelL = absPulse(currentPulseL);
  long travelR = absPulse(currentPulseR);
  long travelAvg = (travelL + travelR) / 2;
  long remaining = backDriveTargetPulses - travelAvg;
  int stopTolerance = distanceStopTolerance(backDriveTargetPulses);

  if (backDrivePhase == CELL_DRIVE_HEADING_FINE)
  {
    if (updatePointTurnCore(cfg))
    {
      resetBackDriveController();
      return true;
    }
    return false;
  }

  if (backDrivePhase == CELL_DRIVE_BRAKE)
  {
    debugTotalSteer = (int)remaining;
    if (brakeOneCellDrive(cfg))
    {
      backDrivePhase = CELL_DRIVE_SETTLE;
      backDrivePhaseStartMs = millis();
    }
    return false;
  }

  if (backDrivePhase == CELL_DRIVE_SETTLE)
  {
    brakeMotors();
    debugTotalSteer = (int)remaining;
    if (millis() - backDrivePhaseStartMs < 140)
      return false;

    pulses = getPulseSnapshot();
    travelAvg = (absPulse(pulses.left) + absPulse(pulses.right)) / 2;
    remaining = backDriveTargetPulses - travelAvg;
    if (abs(remaining) <= stopTolerance || backDriveFineAttempts >= 3)
    {
      float headingError = target_yaw - continuousYaw;
      if (fabsf(headingError) > oneCellHeadingTolerance(cfg))
      {
        bool wasPostBackupRunning = pointTurnPostBackupRunning;
        beginPointTurn(headingError, false);
        pointTurnPostBackupRunning = wasPostBackupRunning;
        backDrivePhase = CELL_DRIVE_HEADING_FINE;
        return false;
      }

      resetBackDriveController();
      return true;
    }

    backDrivePhase = CELL_DRIVE_FINE;
    backDrivePhaseStartMs = millis();
    backDriveFineAttempts++;
    return false;
  }

  if (backDrivePhase == CELL_DRIVE_FINE)
  {
    if (abs(remaining) <= stopTolerance ||
        millis() - backDrivePhaseStartMs > 500)
    {
      backDrivePhase = CELL_DRIVE_BRAKE;
      return false;
    }

    int direction = (remaining > 0) ? -1 : 1;
    int finePwm = oneCellFinePwm(cfg) * direction;
    setLeftMotor(applyRateLimit(finePwm, pwmL, max(1, cfg.ramp_rate / 2)));
    setRightMotor(applyRateLimit(finePwm, pwmR, max(1, cfg.ramp_rate / 2)));
    debugSteerIR = 0;
    debugTotalSteer = (int)remaining;
    return false;
  }

  int brakeLead = distanceBrakeLeadPulses(backDriveTargetPulses);
  if (remaining <= brakeLead)
  {
    backDrivePhase = CELL_DRIVE_BRAKE;
    return false;
  }

  float speedL = (float)absPulse(currentPulseL - lastSpeedPulseL);
  float speedR = (float)absPulse(currentPulseR - lastSpeedPulseR);
  lastSpeedPulseL = currentPulseL;
  lastSpeedPulseR = currentPulseR;

  float decelScale =
      constrain((float)(remaining - brakeLead) / (float)brakeLead, 0.0f, 1.0f);
  float targetVel = cfg.min_vel + (cfg.max_vel - cfg.min_vel) * decelScale;
  if (virtualVel > targetVel)
    virtualVel = max(targetVel, virtualVel - cfg.accel_rate * 2.0f);
  else
    virtualVel = min(targetVel, virtualVel + cfg.accel_rate);
  virtualVel = constrain(virtualVel, cfg.min_vel, cfg.max_vel);
  virtualPos = travelAvg;

  int pidTrimL = computeWheelSpeedPid(virtualVel, speedL, cfg.Kp_L,
                                      cfg.Ki_L, cfg.Kd_L, integralL,
                                      prevErrL, dFilterL);
  int pidTrimR = computeWheelSpeedPid(virtualVel, speedR, cfg.Kp_R,
                                      cfg.Ki_R, cfg.Kd_R, integralR,
                                      prevErrR, dFilterR);

  float pwmScale =
      constrain((float)(remaining - brakeLead) / (float)(brakeLead * 2),
                0.0f, 1.0f);
  int baseCommand =
      cfg.Turn_Min + (int)((cfg.base_pwm - cfg.Turn_Min) * pwmScale);
  int basePwmL = keepMotorRunningPwm(baseCommand + pidTrimL, cfg);
  int basePwmR = keepMotorRunningPwm(baseCommand + pidTrimR, cfg);
  int steerLimit = driveSteerLimit(cfg);
  int gyroSteer = computeHeadingHoldSteer(cfg, steerLimit);
  int encoderSteer = -computeEncoderBalanceSteer(travelL, travelR, cfg);
  int totalSteer = constrain(gyroSteer + encoderSteer,
                             -steerLimit, steerLimit);
  debugSteerIR = 0;
  debugTotalSteer = totalSteer;

  int targetPwm_L =
      constrain(-basePwmL + totalSteer - cfg.wheel_trim_L,
                -cfg.Turn_Max, -cfg.Turn_Min);
  int targetPwm_R =
      constrain(-basePwmR - totalSteer - cfg.wheel_trim_R,
                -cfg.Turn_Max, -cfg.Turn_Min);

  int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
  int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);
  setLeftMotor(newPwm_L);
  setRightMotor(newPwm_R);
  return false;
}

bool updateOneCellDrive(const IrSnapshot &ir, const RuntimeParams &cfg,
                        int &filteredIrSteer, int boostPwmL, int boostPwmR,
                        bool mazeMode, const MazeWalls *walls)
{
  PulseSnapshot pulses = getPulseSnapshot();
  long currentPulseL = pulses.left;
  long currentPulseR = pulses.right;
  long travelL = absPulse(currentPulseL);
  long travelR = absPulse(currentPulseR);
  long travelAvg = (travelL + travelR) / 2;
  long remaining = (long)cfg.pulses_per_cell - travelAvg;
  int stopTolerance = oneCellStopTolerance(cfg);

  if (cellDrivePhase == CELL_DRIVE_HEADING_FINE)
  {
    if (updatePointTurnCore(cfg))
    {
      resetCellDriveController();
      return true;
    }
    return false;
  }

  if (cellDrivePhase == CELL_DRIVE_BRAKE)
  {
    debugTotalSteer = (int)remaining;
    if (brakeOneCellDrive(cfg))
    {
      cellDrivePhase = CELL_DRIVE_SETTLE;
      cellDrivePhaseStartMs = millis();
    }
    return false;
  }

  if (cellDrivePhase == CELL_DRIVE_SETTLE)
  {
    brakeMotors();
    debugTotalSteer = (int)remaining;
    if (millis() - cellDrivePhaseStartMs < 140)
      return false;

    pulses = getPulseSnapshot();
    travelAvg = (absPulse(pulses.left) + absPulse(pulses.right)) / 2;
    remaining = (long)cfg.pulses_per_cell - travelAvg;
    if (abs(remaining) <= stopTolerance || cellDriveFineAttempts >= 3)
    {
      float headingError = fabsf(target_yaw - continuousYaw);
      if (headingError > oneCellHeadingTolerance(cfg))
      {
        filteredIrSteer = 0;
        startOneCellHeadingCorrection();
        return false;
      }

      resetCellDriveController();
      return true;
    }

    cellDrivePhase = CELL_DRIVE_FINE;
    cellDrivePhaseStartMs = millis();
    cellDriveFineAttempts++;
    return false;
  }

  if (cellDrivePhase == CELL_DRIVE_FINE)
  {
    if (abs(remaining) <= stopTolerance ||
        millis() - cellDrivePhaseStartMs > 500)
    {
      cellDrivePhase = CELL_DRIVE_BRAKE;
      return false;
    }

    int direction = (remaining > 0) ? 1 : -1;
    int finePwm = oneCellFinePwm(cfg) * direction;
    setLeftMotor(applyRateLimit(finePwm, pwmL, max(1, cfg.ramp_rate / 2)));
    setRightMotor(applyRateLimit(finePwm, pwmR, max(1, cfg.ramp_rate / 2)));
    debugSteerIR = 0;
    debugTotalSteer = (int)remaining;
    return false;
  }

  int brakeLead = oneCellBrakeLeadPulses(cfg);
  if (remaining <= brakeLead)
  {
    cellDrivePhase = CELL_DRIVE_BRAKE;
    return false;
  }

  WallAvoidanceResult wallAvoidance = computeStraightWallAvoidance(ir, cfg);
  int targetSteerIR = 0;
  if (mazeMode && walls != nullptr)
  {
    int centerSteer = computeMazeCenteringSteer(ir, *walls, cfg);
    targetSteerIR =
        (centerSteer != 0 || !wallAvoidance.active)
            ? centerSteer
            : wallAvoidance.steer;
    filteredIrSteer = applyRateLimit(targetSteerIR, filteredIrSteer,
                                     mazeIrSteerRateLimit(cfg));
  }
  else
  {
    int sideSteer = correctFromSideWalls(ir, cfg);
    targetSteerIR = wallAvoidance.active ? wallAvoidance.steer : sideSteer;
    filteredIrSteer =
        (filteredIrSteer * IR_STEER_FILTER_KEEP + targetSteerIR) /
        IR_STEER_FILTER_TOTAL;
  }
  if (targetSteerIR == 0 && abs(filteredIrSteer) <= 1)
    filteredIrSteer = 0;
  debugSteerIR = filteredIrSteer;

  float speedL = (float)absPulse(currentPulseL - lastSpeedPulseL);
  float speedR = (float)absPulse(currentPulseR - lastSpeedPulseR);
  lastSpeedPulseL = currentPulseL;
  lastSpeedPulseR = currentPulseR;

  float decelScale =
      constrain((float)(remaining - brakeLead) / (float)brakeLead, 0.0f, 1.0f);
  float targetVel = cfg.min_vel + (cfg.max_vel - cfg.min_vel) * decelScale;
  if (virtualVel > targetVel)
    virtualVel = max(targetVel, virtualVel - cfg.accel_rate * 2.0f);
  else
    virtualVel = min(targetVel, virtualVel + cfg.accel_rate);
  virtualVel = constrain(virtualVel, cfg.min_vel, cfg.max_vel);
  virtualPos = travelAvg;

  int pidTrimL = computeWheelSpeedPid(virtualVel, speedL, cfg.Kp_L,
                                      cfg.Ki_L, cfg.Kd_L, integralL,
                                      prevErrL, dFilterL);
  int pidTrimR = computeWheelSpeedPid(virtualVel, speedR, cfg.Kp_R,
                                      cfg.Ki_R, cfg.Kd_R, integralR,
                                      prevErrR, dFilterR);

  float pwmScale =
      constrain((float)(remaining - brakeLead) / (float)(brakeLead * 2),
                0.0f, 1.0f);
  int baseCommand =
      cfg.Turn_Min + (int)((cfg.base_pwm - cfg.Turn_Min) * pwmScale);
  int basePwmL = keepMotorRunningPwm(baseCommand + pidTrimL, cfg);
  int basePwmR = keepMotorRunningPwm(baseCommand + pidTrimR, cfg);
  int steerLimit = driveSteerLimit(cfg);
  int gyroSteer = computeHeadingHoldSteer(cfg, steerLimit);
  int encoderSteer = computeEncoderBalanceSteer(travelL, travelR, cfg);
  int totalSteer = constrain(gyroSteer + filteredIrSteer + encoderSteer,
                             -steerLimit, steerLimit);
  debugTotalSteer = totalSteer;

  int targetPwm_L =
      keepMotorRunningPwm(basePwmL + totalSteer + cfg.wheel_trim_L, cfg);
  int targetPwm_R =
      keepMotorRunningPwm(basePwmR - totalSteer + cfg.wheel_trim_R, cfg);

  targetPwm_L = constrain(targetPwm_L + boostPwmL, 0, cfg.Turn_Max);
  targetPwm_R = constrain(targetPwm_R + boostPwmR, 0, cfg.Turn_Max);

  int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
  int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);
  setLeftMotor(newPwm_L);
  setRightMotor(newPwm_R);
  return false;
}

void resetWheelPidMemory()
{
  PulseSnapshot pulses = getPulseSnapshot();
  integralL = integralR = prevErrL = prevErrR = dFilterL = dFilterR = 0.0f;
  integralT = prevErrT = dFilterT = 0.0f;
  lastSpeedPulseL = pulses.left;
  lastSpeedPulseR = pulses.right;
}

void resetRunProfileAndPidMemory()
{
  virtualPos = 0.0f;
  virtualVel = 0.0f;
  resetCellDriveController();
  resetBackDriveController();
  resetWheelPidMemory();
}

bool applyPendingRuntimeParamsAtSafePoint()
{
  if (carState != IDLE)
    return false;

  RuntimeParams params;
  if (!consumePendingRuntimeParams(params))
    return false;

  applyActiveRuntimeParams(params);
  resetRunProfileAndPidMemory();
  return true;
}

void setupControlTaskWatchdog()
{
  esp_task_wdt_init(CONTROL_TASK_WDT_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(nullptr);
}

void feedControlTaskWatchdog()
{
  esp_task_wdt_reset();
}

void finishControlLoop(uint32_t loopStartMs)
{
  uint32_t elapsedMs = millis() - loopStartMs;
  controlTaskLastLoopMs = elapsedMs;

  if (elapsedMs > CONTROL_LOOP_SOFT_TIMEOUT_MS)
  {
    controlTaskOverrunCount++;
    changeState(IDLE);
  }

  feedControlTaskWatchdog();
}

void changeState(RunState newState)
{
  if (carState == newState)
  {
    if (newState == IDLE)
      applyPendingRuntimeParamsAtSafePoint();
    return;
  }

  if (carState == IDLE)
    applyPendingRuntimeParamsAtSafePoint();

  resetMotorsAndPID();
  resetRunProfileAndPidMemory();

  carState = newState;
  stateStartTime = millis();

  switch (newState)
  {
  case PID_RUN:
  case PID_RUN_ONE_CELL:
  case MAZE_RIGHT_HAND:
  case MAZE_RIGHT_HAND_CELL:
    resetWheelPidMemory();
    virtualVel = 0.0f;
    virtualPos = 0.0f;
    target_yaw = continuousYaw;
    isRunning = true;
    if (isMazeRunState(newState))
    {
      resetMazeRunner();
      if (newState == MAZE_RIGHT_HAND_CELL)
        pixels.setPixelColor(0, pixels.Color(0, 160, 255));
      else
        pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    }
    else
    {
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    }
    break;
  case TEST_L:
  case TEST_R:
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(255, 0, 255));
    break;
  case TEST_TURN_L:
  case TEST_TURN_R:
    resetWheelPidMemory();
    target_yaw = continuousYaw;
    beginPointTurn((newState == TEST_TURN_R) ? mazeTurnRightDeg(captureActiveRuntimeParams())
                                             : mazeTurnLeftDeg(captureActiveRuntimeParams()));
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(255, 128, 0));
    break;
  case TEST_BACK_ONE_CELL:
  {
    RuntimeParams cfg = captureActiveRuntimeParams();
    target_yaw = continuousYaw;
    startBackDrive(cfg.pulses_per_cell);
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(128, 128, 255));
    break;
  }
  case IDLE:
  default:
    isRunning = false;
    resetMazeRunner();
    applyPendingRuntimeParamsAtSafePoint();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    break;
  }
  pixels.show();
}

void mainControlTask(void *pvParameters)
{
  (void)pvParameters;

  static int prev_ir_FL = 0;
  static int prev_ir_FR = 0;
  static int filteredIrSteer = 0;
  TickType_t lastWakeTime = xTaskGetTickCount();

  setupControlTaskWatchdog();

  for (;;)
  {
    uint32_t loopStartMs = millis();
    int boostPwmL = 0;
    int boostPwmR = 0;

    applyPendingRuntimeParamsAtSafePoint();

    if (stateChangeRequested)
    {
      changeState(requestedState);
      stateChangeRequested = false;
    }

    RuntimeParams cfg = captureActiveRuntimeParams();

    IrSnapshot rawIr;
    readIR_TDM(&rawIr);
    IrSnapshot ir = getIrSnapshot();

    if (!isRunning)
    {
      debugSideErrorL = 0;
      debugSideErrorR = 0;
      debugSteerIR = 0;
      debugTotalSteer = 0;
      filteredIrSteer = 0;
    }

    bool frontWallStop =
        (isRunning && !isMazeRunState(carState) &&
         !isPointTurnTestState(carState) &&
         !isBackDriveTestState(carState) &&
         isFrontWallDetected(rawIr, cfg));
    if (frontWallStop)
    {
      int brakeRamp = max(1, cfg.ramp_rate * FRONT_STOP_BRAKE_RAMP_MULTIPLIER);
      int newPwm_L = applyRateLimit(0, pwmL, brakeRamp);
      int newPwm_R = applyRateLimit(0, pwmR, brakeRamp);
      setLeftMotor(newPwm_L);
      setRightMotor(newPwm_R);

      if (abs(newPwm_L) <= 3 && abs(newPwm_R) <= 3)
      {
        changeState(IDLE);
      }
    }

    if (!frontWallStop)
    {
      if (isRunning)
      {
        int deltaFL = ir.frontLeft - prev_ir_FL;
        int deltaFR = ir.frontRight - prev_ir_FR;
        boostPwmL = (deltaFL > 0) ? (SENSOR_DELTA_PWM_GAIN * deltaFL) : 0;
        boostPwmR = (deltaFR > 0) ? (SENSOR_DELTA_PWM_GAIN * deltaFR) : 0;
      }
      prev_ir_FL = ir.frontLeft;
      prev_ir_FR = ir.frontRight;

      updateIMU(CONTROL_DT_SECONDS);

      if (isRunning)
      {
        switch (carState)
        {
        case TEST_L:
          setLeftMotor(constrain(cfg.base_pwm + boostPwmL, 0, cfg.Turn_Max));
          setRightMotor(constrain(boostPwmR, 0, cfg.Turn_Max));
          break;

        case TEST_R:
          setRightMotor(constrain(cfg.base_pwm + boostPwmR, 0, cfg.Turn_Max));
          setLeftMotor(constrain(boostPwmL, 0, cfg.Turn_Max));
          break;

        case TEST_TURN_L:
        case TEST_TURN_R:
          if (updatePointTurn(cfg))
          {
            changeState(IDLE);
          }
          break;

        case TEST_BACK_ONE_CELL:
          if (updateBackDrive(cfg))
          {
            changeState(IDLE);
          }
          break;

        case PID_RUN:
        case PID_RUN_ONE_CELL:
        {
          if (carState == PID_RUN_ONE_CELL)
          {
            if (updateOneCellDrive(ir, cfg, filteredIrSteer, boostPwmL,
                                   boostPwmR, false, nullptr))
            {
              changeState(IDLE);
            }
            break;
          }

          PulseSnapshot pulses = getPulseSnapshot();
          long currentPulseL = pulses.left;
          long currentPulseR = pulses.right;
          long travelL = absPulse(currentPulseL);
          long travelR = absPulse(currentPulseR);
          long travelAvg = (travelL + travelR) / 2;

          WallAvoidanceResult wallAvoidance =
              computeStraightWallAvoidance(ir, cfg);
          int sideSteer = correctFromSideWalls(ir, cfg);
          int steerIR = wallAvoidance.active ? wallAvoidance.steer : sideSteer;
          debugSteerIR = steerIR;
          filteredIrSteer =
              (filteredIrSteer * IR_STEER_FILTER_KEEP + steerIR) /
              IR_STEER_FILTER_TOTAL;
          if (steerIR == 0 && abs(filteredIrSteer) <= 1)
            filteredIrSteer = 0;

          float speedL = (float)absPulse(currentPulseL - lastSpeedPulseL);
          float speedR = (float)absPulse(currentPulseR - lastSpeedPulseR);
          lastSpeedPulseL = currentPulseL;
          lastSpeedPulseR = currentPulseR;

          virtualVel = constrain(virtualVel + cfg.accel_rate, cfg.min_vel,
                                 cfg.max_vel);
          virtualPos = travelAvg;

          int pidTrimL = computeWheelSpeedPid(virtualVel, speedL, cfg.Kp_L,
                                              cfg.Ki_L, cfg.Kd_L, integralL,
                                              prevErrL, dFilterL);
          int pidTrimR = computeWheelSpeedPid(virtualVel, speedR, cfg.Kp_R,
                                              cfg.Ki_R, cfg.Kd_R, integralR,
                                              prevErrR, dFilterR);
          int basePwmL = keepMotorRunningPwm(cfg.base_pwm + pidTrimL, cfg);
          int basePwmR = keepMotorRunningPwm(cfg.base_pwm + pidTrimR, cfg);
          int steerLimit = driveSteerLimit(cfg);
          int gyroSteer = computeHeadingHoldSteer(cfg, steerLimit);
          int totalSteer = constrain(gyroSteer + filteredIrSteer,
                                     -steerLimit, steerLimit);
          debugTotalSteer = totalSteer;

          int targetPwm_L =
              keepMotorRunningPwm(basePwmL + totalSteer + cfg.wheel_trim_L, cfg);
          int targetPwm_R =
              keepMotorRunningPwm(basePwmR - totalSteer + cfg.wheel_trim_R, cfg);

          targetPwm_L = constrain(targetPwm_L + boostPwmL, 0, cfg.Turn_Max);
          targetPwm_R = constrain(targetPwm_R + boostPwmR, 0, cfg.Turn_Max);

          int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
          int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);

          setLeftMotor(newPwm_L);
          setRightMotor(newPwm_R);
          break;
        }

        case MAZE_RIGHT_HAND:
        case MAZE_RIGHT_HAND_CELL:
        {
          bool cellMode = isMazeCellRunState();

          if (mazePhase == MAZE_DECIDE)
          {
            MazeWalls walls = readMazeWalls(rawIr, cfg);
            debugSideErrorL = walls.left ? 1 : 0;
            debugSideErrorR = walls.right ? 1 : 0;
            debugSteerIR = walls.front ? 1 : 0;
            debugTotalSteer = 0;
            filteredIrSteer = 0;
            brakeMotors();
            if (!walls.right)
            {
              if (!cellMode && !walls.front)
                startMazeLateTurn(mazeTurnRightDeg(cfg), cfg);
              else
                startMazeTurn(mazeTurnRightDeg(cfg));
            }
            else if (!walls.front)
            {
              startMazeStraightRun();
            }
            else if (!walls.left)
            {
              startMazeTurn(mazeTurnLeftDeg(cfg));
            }
            else
            {
              startMazeDeadEndBackup();
            }
            break;
          }

          if (mazePhase == MAZE_SETTLE_AFTER_BACKUP)
          {
            brakeMotors();
            debugSteerIR = 0;
            debugTotalSteer =
                (int)(MAZE_POST_BACKUP_SETTLE_MS -
                      (millis() - mazeSettleStartMs));

            if (millis() - mazeSettleStartMs >= MAZE_POST_BACKUP_SETTLE_MS)
            {
              mazePhase = MAZE_DECIDE_AFTER_BACKUP;
            }
            break;
          }

          if (mazePhase == MAZE_DECIDE_AFTER_BACKUP)
          {
            MazeWalls walls = readMazeWalls(rawIr, cfg);
            bool rightOpen = isMazeSideOpenForTurn(SENSOR_FR, rawIr, cfg);
            bool leftOpen = isMazeSideOpenForTurn(SENSOR_FL, rawIr, cfg);
            debugSideErrorL = walls.left ? 1 : 0;
            debugSideErrorR = walls.right ? 1 : 0;
            debugSteerIR = walls.front ? 1 : 0;
            debugTotalSteer = 0;
            filteredIrSteer = 0;
            brakeMotors();

            if (cellMode)
            {
              if (!walls.right)
              {
                startMazeTurn(mazeTurnRightDeg(cfg));
              }
              else if (!walls.front)
              {
                startMazeStraightRun();
              }
              else if (!walls.left)
              {
                startMazeTurn(mazeTurnLeftDeg(cfg));
              }
              else
              {
                startMazeDeadEndBackup();
              }
            }
            else if (rightOpen)
            {
              startMazeTurn(mazeTurnRightDeg(cfg));
            }
            else if (leftOpen)
            {
              startMazeTurn(mazeTurnLeftDeg(cfg));
            }
            else
            {
              startMazeDeadEndBackup();
            }
            break;
          }

          if (mazePhase == MAZE_WAIT_AT_CELL)
          {
            brakeMotors();
            debugSteerIR = 0;
            uint32_t waitElapsedMs = millis() - mazeSettleStartMs;
            debugTotalSteer =
                (int)((waitElapsedMs >= MAZE_CELL_DECISION_WAIT_MS)
                          ? 0
                          : (MAZE_CELL_DECISION_WAIT_MS - waitElapsedMs));

            if (waitElapsedMs >= MAZE_CELL_DECISION_WAIT_MS)
            {
              if (mazeHasQueuedTurn)
                startQueuedMazeTurn();
              else
                mazePhase = MAZE_DECIDE;
            }
            break;
          }

          if (mazePhase == MAZE_CENTER_AFTER_TURN)
          {
            brakeMotors();
            debugSteerIR = 0;
            uint32_t centerElapsedMs = millis() - mazeSettleStartMs;
            debugTotalSteer =
                (int)((centerElapsedMs >= MAZE_CENTER_SETTLE_MS)
                          ? 0
                          : (MAZE_CENTER_SETTLE_MS - centerElapsedMs));

            if (centerElapsedMs >= MAZE_CENTER_SETTLE_MS)
            {
              startMazeStraightRun();
            }
            break;
          }

          if (mazePhase == MAZE_TURNING)
          {
            if (updatePointTurn(cfg))
            {
              filteredIrSteer = 0;
              if (cellMode)
                startMazeCenterAfterTurn();
              else
                startMazeStraightRun();
            }
            break;
          }

          if (mazePhase == MAZE_BRAKE_AT_WALL)
          {
            int brakeRamp =
                max(1, cfg.ramp_rate * FRONT_STOP_BRAKE_RAMP_MULTIPLIER);
            int newPwm_L = applyRateLimit(0, pwmL, brakeRamp);
            int newPwm_R = applyRateLimit(0, pwmR, brakeRamp);
            setLeftMotor(newPwm_L);
            setRightMotor(newPwm_R);

            if (abs(newPwm_L) <= 3 && abs(newPwm_R) <= 3)
            {
              if (cellMode)
              {
                startMazeCellWait();
              }
              else
              {
                brakeMotors();
                resetMotorsAndPID();
                resetRunProfileAndPidMemory();
                target_yaw = continuousYaw;
                mazePhase = MAZE_DECIDE;
              }
            }
            break;
          }

          if (mazePhase == MAZE_BRAKE_AT_CELL)
          {
            int brakeRamp =
                max(1, cfg.ramp_rate * FRONT_STOP_BRAKE_RAMP_MULTIPLIER);
            int newPwm_L = applyRateLimit(0, pwmL, brakeRamp);
            int newPwm_R = applyRateLimit(0, pwmR, brakeRamp);
            setLeftMotor(newPwm_L);
            setRightMotor(newPwm_R);

            if (abs(newPwm_L) <= 3 && abs(newPwm_R) <= 3)
            {
              filteredIrSteer = 0;
              startMazeCellWait();
            }
            break;
          }

          if (mazePhase == MAZE_BACK_UP_UNTIL_TURN)
          {
            bool rightOpen = isMazeSideOpenForTurn(SENSOR_FR, rawIr, cfg);
            bool leftOpen = isMazeSideOpenForTurn(SENSOR_FL, rawIr, cfg);

            debugSideErrorL = leftOpen ? 1 : 0;
            debugSideErrorR = rightOpen ? 1 : 0;
            debugSteerIR = 0;
            debugTotalSteer = (int)averageTravelPulse();

            if (rightOpen || leftOpen)
            {
              float turnDegrees = rightOpen ? mazeTurnRightDeg(cfg)
                                            : mazeTurnLeftDeg(cfg);
              brakeMotors();
              resetMotorsAndPID();
              resetRunProfileAndPidMemory();
              filteredIrSteer = 0;
              target_yaw = continuousYaw;
              startMazeTurn(turnDegrees);
              break;
            }

            int backupPwm =
                constrain(cfg.maze_dead_end_backup_pwm, 0, cfg.Turn_Max);
            int targetPwm_L = -backupPwm;
            int targetPwm_R = -backupPwm;
            int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
            int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);
            setLeftMotor(newPwm_L);
            setRightMotor(newPwm_R);
            break;
          }

          if (mazePhase == MAZE_BACK_UP_AFTER_DEAD_END)
          {
            PulseSnapshot pulses = getPulseSnapshot();
            long travelAvg =
                (absPulse(pulses.left) + absPulse(pulses.right)) / 2;

            debugSteerIR = 0;
            debugTotalSteer =
                (int)(cfg.maze_dead_end_backup_pulses - travelAvg);

            if (travelAvg >= cfg.maze_dead_end_backup_pulses)
            {
              startMazePostBackupDecision();
              break;
            }

            int backupPwm =
                constrain(cfg.maze_dead_end_backup_pwm, 0, cfg.Turn_Max);
            int targetPwm_L = -backupPwm;
            int targetPwm_R = -backupPwm;
            int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
            int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);
            setLeftMotor(newPwm_L);
            setRightMotor(newPwm_R);
            break;
          }

          MazeWalls walls = readMazeWalls(rawIr, cfg);
          debugSideErrorL = walls.left ? 1 : 0;
          debugSideErrorR = walls.right ? 1 : 0;

          PulseSnapshot pulses = getPulseSnapshot();
          long currentPulseL = pulses.left;
          long currentPulseR = pulses.right;
          long travelL = absPulse(currentPulseL);
          long travelR = absPulse(currentPulseR);
          long travelAvg = (travelL + travelR) / 2;

          if (cellMode && mazePhase == MAZE_RUN_TO_WALL && !mazeHasQueuedTurn)
          {
            if (isMazeSideOpenForTurn(SENSOR_FR, rawIr, cfg))
            {
              queueMazeTurn(mazeTurnRightDeg(cfg));
            }
            else if (isMazeSideOpenForTurn(SENSOR_FL, rawIr, cfg))
            {
              queueMazeTurn(mazeTurnLeftDeg(cfg));
            }
          }

          if (walls.front)
          {
            debugSteerIR = 1;
            filteredIrSteer = 0;
            mazePhase = MAZE_BRAKE_AT_WALL;
            break;
          }

          if (cellMode && mazePhase == MAZE_RUN_TO_WALL)
          {
            if (updateOneCellDrive(ir, cfg, filteredIrSteer, boostPwmL,
                                   boostPwmR, true, &walls))
            {
              filteredIrSteer = 0;
              startMazeCellWait();
            }
            break;
          }

          if (!cellMode && mazePhase == MAZE_RUN_TO_WALL &&
              isMazeSideOpenForTurn(SENSOR_FR, rawIr, cfg))
          {
            startMazeLateTurn(mazeTurnRightDeg(cfg), cfg);
            break;
          }

          if (mazePhase == MAZE_RUN_BEFORE_TURN)
          {
            long lateTravel = travelAvg - mazeLateTurnStartPulse;
            if (lateTravel < 0)
              lateTravel = 0;

            debugTotalSteer = (int)(cfg.maze_turn_late_pulses - lateTravel);
            if (lateTravel >= cfg.maze_turn_late_pulses)
            {
              startMazeTurn(mazeTurnDegrees);
              break;
            }
          }

          WallAvoidanceResult wallAvoidance =
              computeStraightWallAvoidance(ir, cfg);
          int centerSteer = computeMazeCenteringSteer(ir, walls, cfg);
          int targetSteerIR =
              (centerSteer != 0 || !wallAvoidance.active)
                  ? centerSteer
                  : wallAvoidance.steer;
          filteredIrSteer = applyRateLimit(targetSteerIR, filteredIrSteer,
                                           mazeIrSteerRateLimit(cfg));
          if (targetSteerIR == 0 && abs(filteredIrSteer) <= 1)
            filteredIrSteer = 0;
          debugSteerIR = filteredIrSteer;

          float speedL = (float)absPulse(currentPulseL - lastSpeedPulseL);
          float speedR = (float)absPulse(currentPulseR - lastSpeedPulseR);
          lastSpeedPulseL = currentPulseL;
          lastSpeedPulseR = currentPulseR;

          virtualVel = constrain(virtualVel + cfg.accel_rate, cfg.min_vel,
                                 cfg.max_vel);
          virtualPos = travelAvg;

          int pidTrimL = computeWheelSpeedPid(virtualVel, speedL, cfg.Kp_L,
                                              cfg.Ki_L, cfg.Kd_L, integralL,
                                              prevErrL, dFilterL);
          int pidTrimR = computeWheelSpeedPid(virtualVel, speedR, cfg.Kp_R,
                                              cfg.Ki_R, cfg.Kd_R, integralR,
                                              prevErrR, dFilterR);
          int basePwmL = keepMotorRunningPwm(cfg.base_pwm + pidTrimL, cfg);
          int basePwmR = keepMotorRunningPwm(cfg.base_pwm + pidTrimR, cfg);
          int steerLimit = driveSteerLimit(cfg);
          int gyroSteer = computeHeadingHoldSteer(cfg, steerLimit);
          int totalSteer = constrain(gyroSteer + filteredIrSteer,
                                     -steerLimit, steerLimit);
          debugTotalSteer = totalSteer;

          int targetPwm_L =
              keepMotorRunningPwm(basePwmL + totalSteer + cfg.wheel_trim_L, cfg);
          int targetPwm_R =
              keepMotorRunningPwm(basePwmR - totalSteer + cfg.wheel_trim_R, cfg);

          targetPwm_L = constrain(targetPwm_L + boostPwmL, 0, cfg.Turn_Max);
          targetPwm_R = constrain(targetPwm_R + boostPwmR, 0, cfg.Turn_Max);

          int newPwm_L = applyRateLimit(targetPwm_L, pwmL, cfg.ramp_rate);
          int newPwm_R = applyRateLimit(targetPwm_R, pwmR, cfg.ramp_rate);

          setLeftMotor(newPwm_L);
          setRightMotor(newPwm_R);
          break;
        }

        default:
          changeState(IDLE);
          break;
        }
      }
    }

    finishControlLoop(loopStartMs);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

} // namespace

void setup()
{
  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(10);
  pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  pixels.show();

  initSensors();
  initMotors();
  initIMU();
  setupWebServer();

  xTaskCreatePinnedToCore(
      mainControlTask,
      "MainControlTask",
      MAIN_CONTROL_STACK_SIZE,
      nullptr,
      configMAX_PRIORITIES - 1,
      nullptr,
      1);

  changeState(IDLE);
}

void loop()
{
  vTaskDelete(nullptr);
}
