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
const int SENSOR_DELTA_PWM_GAIN = 2;
const int FRONT_STOP_EARLY_MARGIN = 1000;
const int FRONT_STOP_BRAKE_RAMP_MULTIPLIER = 3;
const uint32_t MAIN_CONTROL_STACK_SIZE = 8192;
const uint32_t CONTROL_TASK_WDT_TIMEOUT_SECONDS = 2;
const uint32_t CONTROL_LOOP_SOFT_TIMEOUT_MS = 50;

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

bool isFrontWallDetected(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  int leftThreshold = max(0, frontCorrectionStart(FRONT_STOP_SENSOR_LEFT, cfg) -
                                 FRONT_STOP_EARLY_MARGIN);
  int rightThreshold = max(0, frontCorrectionStart(FRONT_STOP_SENSOR_RIGHT, cfg) -
                                  FRONT_STOP_EARLY_MARGIN);

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

int sideErrorDeadband()
{
  return 0;
}

int sideCorrectionKick(const RuntimeParams &cfg)
{
  return constrain(cfg.Turn_Min, 25, 45);
}

int sideCloseDirection(int sensorIndex, int sideRef, const RuntimeParams &cfg)
{
  int deadband = sideErrorDeadband();
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
  int sideRef = sideWallCorrectionStart(sensorIndex, cfg);
  if (sideRef >= 4095)
    return 0;

  int sideValue = sensorValueByIndex(ir, sensorIndex);
  int direction = sideCloseDirection(sensorIndex, sideRef, cfg);
  int error = (direction > 0) ? (sideValue - sideRef) : (sideRef - sideValue);

  return (error > sideErrorDeadband()) ? error : 0;
}

int correctFromSideWalls(const IrSnapshot &ir, const RuntimeParams &cfg)
{
  int leftError = sideCloseError(SENSOR_L, ir, cfg);
  int rightError = sideCloseError(SENSOR_R, ir, cfg);
  if (leftError == 0 && rightError == 0)
    return 0;

  int rawSteer = leftError - rightError;
  int steer = (int)(rawSteer * cfg.k_ir);
  if (steer > 0)
    steer = max(steer, sideCorrectionKick(cfg));
  else if (steer < 0)
    steer = min(steer, -sideCorrectionKick(cfg));

  return constrain(steer, -(cfg.Turn_Max - cfg.Turn_Min),
                   cfg.Turn_Max - cfg.Turn_Min);
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
    resetWheelPidMemory();
    virtualVel = 0.0f;
    virtualPos = 0.0f;
    target_yaw = continuousYaw;
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

  static int prev_ir_FL = 0;
  static int prev_ir_FR = 0;
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

    bool frontWallStop = (isRunning && isFrontWallDetected(rawIr, cfg));
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

        case PID_RUN:
        {
          PulseSnapshot pulses = getPulseSnapshot();
          long currentPulseL = pulses.left;
          long currentPulseR = pulses.right;
          long travelL = absPulse(currentPulseL);
          long travelR = absPulse(currentPulseR);
          WallAvoidanceResult wallAvoidance =
              computeStraightWallAvoidance(ir, cfg);
          int steerIR = wallAvoidance.active ? wallAvoidance.steer
                                             : correctFromSideWalls(ir, cfg);

          if (steerIR != 0)
          {
            resetWheelPidMemory();

            int minSteerPower = wallAvoidance.active ? wallAvoidanceKick(cfg)
                                                     : sideCorrectionKick(cfg);
            int steerPower = constrain(abs(steerIR), minSteerPower,
                                       cfg.Turn_Max);
            int outsidePwm = cfg.Turn_Max;
            int insidePwm = constrain(cfg.base_pwm - steerPower, 0, cfg.Turn_Max);
            int targetPwm_L = (steerIR > 0) ? outsidePwm : insidePwm;
            int targetPwm_R = (steerIR > 0) ? insidePwm : outsidePwm;

            setLeftMotor(targetPwm_L);
            setRightMotor(targetPwm_R);
            break;
          }

          float speedL = (float)absPulse(currentPulseL - lastSpeedPulseL);
          float speedR = (float)absPulse(currentPulseR - lastSpeedPulseR);
          lastSpeedPulseL = currentPulseL;
          lastSpeedPulseR = currentPulseR;

          virtualVel = constrain(virtualVel + cfg.accel_rate, cfg.min_vel,
                                 cfg.max_vel);
          virtualPos = (travelL + travelR) * 0.5f;

          int pidTrimL = computeWheelSpeedPid(virtualVel, speedL, cfg.Kp_L,
                                              cfg.Ki_L, cfg.Kd_L, integralL,
                                              prevErrL, dFilterL);
          int pidTrimR = computeWheelSpeedPid(virtualVel, speedR, cfg.Kp_R,
                                              cfg.Ki_R, cfg.Kd_R, integralR,
                                              prevErrR, dFilterR);
          int basePwmL = keepMotorRunningPwm(cfg.base_pwm + pidTrimL, cfg);
          int basePwmR = keepMotorRunningPwm(cfg.base_pwm + pidTrimR, cfg);
          float yawError = continuousYaw - target_yaw;
          int totalSteer = (int)(yawError * cfg.k_gyro);

          int targetPwm_L = keepMotorRunningPwm(basePwmL + totalSteer, cfg);
          int targetPwm_R = keepMotorRunningPwm(basePwmR - totalSteer, cfg);

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
