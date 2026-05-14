/*
 * MICROMOUSE - ESP32-S3 - independent wall following + Mahony filter
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "Config.h"
#include "Globals.h"
#include "Sensors.h"
#include "Motor.h"
#include "WallAvoidance.h"
#include "Web.h"

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
const int FRONT_STOP_EARLY_MARGIN = 1000;
const int FRONT_STOP_BRAKE_RAMP_MULTIPLIER = 3;
const int FRONT_STOP_CONFIRM_LOOPS = 2;
const uint32_t MAIN_CONTROL_STACK_SIZE = 8192;
const uint32_t CONTROL_TASK_WDT_TIMEOUT_SECONDS = 2;
const uint32_t CONTROL_LOOP_SOFT_TIMEOUT_MS = 50;
const int IR_STEER_FILTER_KEEP = 3;
const int IR_STEER_FILTER_TOTAL = 4;

long lastSpeedPulseL = 0;
long lastSpeedPulseR = 0;

void changeState(RunState newState);

long absPulse(long value)
{
  return value < 0 ? -value : value;
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

int frontCorrectionStart(int sensorIndex, const RuntimeParams &cfg)
{
  if (!isSensorCalibrated(sensorIndex, cfg))
    return 4095;
  return constrain(max_IR[sensorIndex] + cfg.offset_upper, 0, 4095);
}

int frontStopThreshold(int sensorIndex, const RuntimeParams &cfg)
{
  if (!isSensorCalibrated(sensorIndex, cfg))
    return 4095;

  int correctionStart = frontCorrectionStart(sensorIndex, cfg);
  int minimumUsableThreshold = max(30, cfg.ir_deadband / 2);
  if (correctionStart <= minimumUsableThreshold)
    return 4095;

  int earlyMargin = min(FRONT_STOP_EARLY_MARGIN,
                        max(20, correctionStart / 3));
  return constrain(correctionStart - earlyMargin, minimumUsableThreshold, 4094);
}

bool isFrontWallDetectedByPair(const IrSnapshot &ir, const RuntimeParams &cfg,
                               int leftSensor, int rightSensor)
{
  int leftThreshold = frontStopThreshold(leftSensor, cfg);
  int rightThreshold = frontStopThreshold(rightSensor, cfg);

  bool frontLeftWarn = isSensorCalibrated(leftSensor, cfg) &&
                       sensorValueByIndex(ir, leftSensor) >
                           leftThreshold;
  bool frontRightWarn = isSensorCalibrated(rightSensor, cfg) &&
                        sensorValueByIndex(ir, rightSensor) >
                            rightThreshold;

  return frontLeftWarn && frontRightWarn;
}

bool isFrontWallDetected(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  return isFrontWallDetectedByPair(ir, cfg, FRONT_STOP_SENSOR_LEFT,
                                   FRONT_STOP_SENSOR_RIGHT);
}

bool isOneCellFrontWallDetected(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  return isFrontWallDetectedByPair(ir, cfg, SENSOR_L, SENSOR_R);
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
  return (sensorIndex == SENSOR_L) ? side_ref_L : side_ref_R;
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
  return max(20, cfg.ir_deadband / 2);
}

int sideErrorReleaseDeadband(const RuntimeParams &cfg)
{
  return max(8, sideErrorDeadband(cfg) / 2);
}

int sideCorrectionKick(const RuntimeParams &cfg)
{
  return constrain(cfg.Turn_Min / 3, 6, 15);
}

int sideSteerLimit(const RuntimeParams &cfg)
{
  return constrain((cfg.Turn_Max - cfg.Turn_Min) / 3, 6, 22);
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
  return 1;
}

int sideCloseError(int sensorIndex, const IrSnapshot &ir, const RuntimeParams &cfg)
{
  static bool leftActive = false;
  static bool rightActive = false;
  bool *active = (sensorIndex == SENSOR_L) ? &leftActive : &rightActive;

  int sideRef = sideWallCorrectionStart(sensorIndex, cfg);
  if (sideRef >= 4095) {
    *active = false;
    return 0;
  }

  int sideValue = sensorValueByIndex(ir, sensorIndex);
  int direction = sideCloseDirection(sensorIndex, sideRef, cfg);
  int error = (direction > 0) ? (sideValue - sideRef) : (sideRef - sideValue);
  error = max(0, error);

  if (error > sideErrorDeadband(cfg)) {
    *active = true;
  } else if (error < sideErrorReleaseDeadband(cfg)) {
    *active = false;
  }

  return *active ? error : 0;
}

int correctFromSideWalls(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  int leftError = sideCloseError(SENSOR_L, ir, cfg);
  int rightError = sideCloseError(SENSOR_R, ir, cfg);
  if (leftError == 0 && rightError == 0)
    return 0;

  int rawSteer = leftError - rightError;
  int steer = (int)(rawSteer * cfg.k_ir);

  return constrain(steer, -sideSteerLimit(cfg), sideSteerLimit(cfg));
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
    resetWheelPidMemory();
    virtualVel = 0.0f;
    virtualPos = 0.0f;
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    break;
  case TEST_L:
  case TEST_R:
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(255, 0, 255));
    break;
  case IDLE:
  default:
    isRunning = false;
    applyPendingRuntimeParamsAtSafePoint();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    break;
  }
  pixels.show();
}

void mainControlTask(void *pvParameters)
{
  (void)pvParameters;

  static int filteredSteerIR = 0;
  static int frontWallStopConfirmCount = 0;
  static bool frontWallStopLatched = false;
  TickType_t lastWakeTime = xTaskGetTickCount();

  setupControlTaskWatchdog();

  for (;;)
  {
    uint32_t loopStartMs = millis();

    applyPendingRuntimeParamsAtSafePoint();

    if (stateChangeRequested)
    {
      changeState(requestedState);
      stateChangeRequested = false;
      frontWallStopConfirmCount = 0;
      frontWallStopLatched = false;
    }

    RuntimeParams cfg = captureActiveRuntimeParams();
    if (!isRunning)
    {
      filteredSteerIR = 0;
      frontWallStopConfirmCount = 0;
      frontWallStopLatched = false;
    }

    IrSnapshot rawIr;
    readIR_TDM(&rawIr);
    IrSnapshot ir = getIrSnapshot();

    bool frontWallSeen =
        isRunning &&
        ((carState == PID_RUN_ONE_CELL) ? isOneCellFrontWallDetected(rawIr, cfg)
                                        : isFrontWallDetected(rawIr, cfg));
    if (frontWallSeen)
    {
      if (frontWallStopConfirmCount < FRONT_STOP_CONFIRM_LOOPS)
        frontWallStopConfirmCount++;
      if (frontWallStopConfirmCount >= FRONT_STOP_CONFIRM_LOOPS)
        frontWallStopLatched = true;
    }
    else if (!frontWallStopLatched)
    {
      frontWallStopConfirmCount = 0;
    }

    bool frontWallStop = isRunning && frontWallStopLatched;
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
        switch (carState)
        {
        case TEST_L:
          setLeftMotor(constrain(cfg.base_pwm, 0, cfg.Turn_Max));
          setRightMotor(0);
          break;

        case TEST_R:
          setRightMotor(constrain(cfg.base_pwm, 0, cfg.Turn_Max));
          setLeftMotor(0);
          break;

        case PID_RUN:
        case PID_RUN_ONE_CELL:
        {
          PulseSnapshot pulses = getPulseSnapshot();
          long currentPulseL = pulses.left;
          long currentPulseR = pulses.right;
          long travelL = absPulse(currentPulseL);
          long travelR = absPulse(currentPulseR);
          long travelAvg = (travelL + travelR) / 2;
          if (carState == PID_RUN_ONE_CELL &&
              travelAvg >= cfg.pulses_per_cell)
          {
            changeState(IDLE);
            break;
          }

          int steerIR = 0;
          if (carState == PID_RUN)
          {
            WallAvoidanceResult wallAvoidance =
                computeStraightWallAvoidance(ir, cfg);
            steerIR = wallAvoidance.active ? wallAvoidance.steer
                                           : correctFromSideWalls(ir, cfg);
          }
          else
          {
            filteredSteerIR = 0;
          }

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
          int steerLimit = max(0, cfg.Turn_Max - cfg.Turn_Min);
          filteredSteerIR =
              (filteredSteerIR * IR_STEER_FILTER_KEEP + steerIR) /
              IR_STEER_FILTER_TOTAL;
          if (steerIR == 0 && abs(filteredSteerIR) <= 1)
            filteredSteerIR = 0;

          int totalSteer = constrain(filteredSteerIR,
                                     -steerLimit, steerLimit);

          int targetPwm_L = keepMotorRunningPwm(basePwmL + totalSteer, cfg);
          int targetPwm_R = keepMotorRunningPwm(basePwmR - totalSteer, cfg);

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
