/*
 * ================================================================
 * MICROMOUSE — ESP32-S3 — BÁM TƯỜNG ĐỘC LẬP + MAHONY FILTER
 * ================================================================
 */

#include <Arduino.h>
#include "Config.h"
#include "Globals.h"
#include "IMU.h"
#include "Sensors.h"
#include "Motor.h"
#include "Web.h"

long lastSpeedPulseL = 0;
long lastSpeedPulseR = 0;

long absPulse(long value)
{
  return value < 0 ? -value : value;
}

bool isFrontSensorReady(int sensorIndex)
{
  int frontMargin = max(50, ir_deadband / 2);
  return max_IR[sensorIndex] < 4095 - frontMargin;
}

bool isSideSensorClose(int sensorIndex)
{
  int sideRef = (sensorIndex == 0) ? side_ref_L : side_ref_R;
  int sideValue = (sensorIndex == 0) ? ir_L : ir_R;
  if (sideRef >= 4095)
    return false;
  return sideValue > sideRef + max(15, ir_deadband / 4);
}

int frontCorrectionStart(int sensorIndex)
{
  if (!isFrontSensorReady(sensorIndex))
    return 4095;
  return constrain(max_IR[sensorIndex] + offset_upper, 0, 4095);
}

int frontEmergencyStart(int sensorIndex)
{
  int emergencyMargin = max(120, offset_upper);
  if (!isFrontSensorReady(sensorIndex))
    return 4095;
  return constrain(max_IR[sensorIndex] + emergencyMargin, 0, 4095);
}

bool isFrontWallDetected()
{
  bool flWarn = isFrontSensorReady(0) && ir_FL > frontCorrectionStart(0);
  bool frWarn = isFrontSensorReady(3) && ir_FR > frontCorrectionStart(3);

  return flWarn && frWarn;
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

int keepMotorRunningPwm(int pwm)
{
  return constrain(pwm, Turn_Min, Turn_Max);
}

int sideWallCorrectionStart(int sensorIndex)
{
  int sideRef = (sensorIndex == 0) ? side_ref_L : side_ref_R;
  if (sideRef < 4095)
    return sideRef;
  if (max_IR[sensorIndex] >= 4095 - max(10, offset_upper))
    return 4095;
  return constrain(max_IR[sensorIndex] + offset_upper, 0, 4095);
}

int sideErrorDeadband()
{
  return 0;
}

int sideCorrectionKick()
{
  return constrain(Turn_Min, 25, 45);
}

int sideCloseDirection(int sensorIndex, int sideRef)
{
  int deadband = sideErrorDeadband();
  int wallRaw = -1;
  int emptyRaw = -1;

  if (max_IR[sensorIndex] < 4095 - max(10, offset_upper))
  {
    wallRaw = constrain(max_IR[sensorIndex] + offset_upper, 0, 4095);
  }
  if (min_IR[sensorIndex] > max(10, offset_lower))
  {
    emptyRaw = constrain(min_IR[sensorIndex] - offset_lower, 0, 4095);
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

int sideCloseError(int sensorIndex)
{
  int sideRef = sideWallCorrectionStart(sensorIndex);
  if (sideRef >= 4095)
    return 0;

  int sideValue = (sensorIndex == 0) ? ir_L : ir_R;
  int direction = sideCloseDirection(sensorIndex, sideRef);
  int error = (direction > 0) ? (sideValue - sideRef) : (sideRef - sideValue);

  return (error > sideErrorDeadband()) ? error : 0;
}

int correctFromSideWalls()
{
  int leftError = sideCloseError(1);
  int rightError = sideCloseError(2);
  if (leftError == 0 && rightError == 0)
    return 0;

  int rawSteer = leftError - rightError; // Duong: ne tuong trai, am: ne tuong phai
  int steer = (int)(rawSteer * k_ir);
  if (steer > 0)
    steer = max(steer, sideCorrectionKick());
  else if (steer < 0)
    steer = min(steer, -sideCorrectionKick());

  return constrain(steer, -(Turn_Max - Turn_Min), Turn_Max - Turn_Min);
}

// Biến lưu thời gian chạy task
TickType_t xLastWakeTime;
const TickType_t xFrequency = 10 / portTICK_PERIOD_MS; // Chu kỳ 10ms (100Hz)

// ==============================================================================
// HÀM CHUYỂN TRẠNG THÁI (TRÁNH RÒ RỈ TRẠNG THÁI)
// ==============================================================================
void changeState(RunState newState)
{
  if (carState == newState)
    return;

  // Dọn dẹp PWM và PID cũ trước khi sang state mới
  resetMotorsAndPID();

  carState = newState;
  stateStartTime = millis();

  // Xử lý một số cấu hình riêng theo trạng thái
  switch (newState)
  {
  case PID_RUN:
    lastSpeedPulseL = pulseL;
    lastSpeedPulseR = pulseR;
    virtualVel = 0.0f;
    virtualPos = 0.0f;
    target_yaw = continuousYaw; // Lưu góc hiện tại làm mục tiêu
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // XANH LÁ: Đang chạy PID
    break;
  case TEST_L:
  case TEST_R:
    isRunning = true;
    pixels.setPixelColor(0, pixels.Color(255, 0, 255)); // TÍM: Test Motor
    break;
  case IDLE:
  default:
    isRunning = false;
    pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // ĐỎ: Dừng
    break;
  }
  pixels.show();
}

// ==============================================================================
// TASK ĐIỀU KHIỂN CHÍNH (CORE 1 - 100Hz)
// ==============================================================================
void mainControlTask(void *pvParameters)
{
  static int prev_ir_FL = 0;
  static int prev_ir_FR = 0;
  const int kDeltaPwm = 2;
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    int boostPwmL = 0;
    int boostPwmR = 0;
    unsigned long now = millis();
    float dt_imu = 0.01f; // Chạy chuẩn 10ms -> dt = 0.01s

    if (stateChangeRequested)
    {
      changeState(requestedState);
      stateChangeRequested = false;
    }

    // 1. ĐỌC CẢM BIẾN IR
    readIR_TDM();

    if (isRunning)
    {
      int deltaFL = ir_FL - prev_ir_FL; // Cam bien so 1 (FL)
      int deltaFR = ir_FR - prev_ir_FR; // Cam bien so 2 (FR)
      boostPwmL = (deltaFL > 0) ? (kDeltaPwm * deltaFL) : 0;
      boostPwmR = (deltaFR > 0) ? (kDeltaPwm * deltaFR) : 0;
    }
    prev_ir_FL = ir_FL;
    prev_ir_FR = ir_FR;

    // 2. CẬP NHẬT IMU (MAHONY)
    updateIMU(dt_imu);

    // 3. ĐIỀU KHIỂN THEO TRẠNG THÁI
    if (isRunning)
    {
      switch (carState)
      {
      case WAIT_1S:
        // Trạng thái đệm
        if (now - stateStartTime >= 1000)
        {
          changeState(PID_RUN);
        }
        break;

      case TEST_L:
        setLeftMotor(constrain(base_pwm + boostPwmL, 0, Turn_Max));
        setRightMotor(constrain(boostPwmR, 0, Turn_Max));
        break;

      case TEST_R:
        setRightMotor(constrain(base_pwm + boostPwmR, 0, Turn_Max));
        setLeftMotor(constrain(boostPwmL, 0, Turn_Max));
        break;

      case PID_RUN:
      {
        if (isFrontWallDetected())
        {
          changeState(IDLE);
          break;
        }

        long currentPulseL = pulseL;
        long currentPulseR = pulseR;
        long travelL = absPulse(currentPulseL);
        long travelR = absPulse(currentPulseR);
        long targetPulseL = pulses_per_cell;
        long targetPulseR = max(1L, (pulses_per_cell * 18L) / 20L);
        int steerIR = correctFromSideWalls();

        if (steerIR != 0)
        {
          lastSpeedPulseL = currentPulseL;
          lastSpeedPulseR = currentPulseR;
          integralL = integralR = prevErrL = prevErrR = dFilterL = dFilterR = 0;

          int steerPower = constrain(abs(steerIR), sideCorrectionKick(), Turn_Max);
          int outsidePwm = Turn_Max;
          int insidePwm = constrain(base_pwm - steerPower, 0, Turn_Max);
          int targetPwm_L = (steerIR > 0) ? outsidePwm : insidePwm;
          int targetPwm_R = (steerIR > 0) ? insidePwm : outsidePwm;

          setLeftMotor(targetPwm_L);
          setRightMotor(targetPwm_R);
          break;
        }

        bool leftDone = travelL >= targetPulseL;
        bool rightDone = travelR >= targetPulseR;

        if (leftDone && rightDone)
        {
          changeState(IDLE);
          break;
        }

        float speedL = (float)absPulse(currentPulseL - lastSpeedPulseL);
        float speedR = (float)absPulse(currentPulseR - lastSpeedPulseR);
        lastSpeedPulseL = currentPulseL;
        lastSpeedPulseR = currentPulseR;

        virtualVel = constrain(virtualVel + accel_rate, min_vel, max_vel);
        virtualPos = (travelL + travelR) * 0.5f;

        int pidTrimL = computeWheelSpeedPid(virtualVel, speedL, Kp_L, Ki_L,
                                            Kd_L, integralL, prevErrL,
                                            dFilterL);
        int pidTrimR = computeWheelSpeedPid(virtualVel, speedR, Kp_R, Ki_R,
                                            Kd_R, integralR, prevErrR,
                                            dFilterR);
        int basePwmL = keepMotorRunningPwm(base_pwm + pidTrimL);
        int basePwmR = keepMotorRunningPwm(base_pwm + pidTrimR);
        // A. Tính toán bù lái từ Gyro (MPU6050)
        float yawError = continuousYaw - target_yaw;
        int steerGyro = (int)(yawError * k_gyro);

        // B. L/R chi sua lech tuong ben. FL/FR chi dung de dung khi gap tuong truoc.
#if 0
          static bool hasLeftWall = false;
          static bool hasRightWall = false;

          // Hysteresis: Quá ngưỡng tường (max_IR) thì tính là có tường. Dưới ngưỡng trống (min_IR) thì báo mất tường.
          if (ir_FL > max_IR[1]) hasLeftWall = true;
          else if (ir_FL < min_IR[1]) hasLeftWall = false;

          if (ir_FR > max_IR[2]) hasRightWall = true;
          else if (ir_FR < min_IR[2]) hasRightWall = false;
          
          int oldSteerIR = 0;

          // Chỉ bám tường khi góc MPU không vẹo quá 3 độ
          if (abs(yawError) <= 3.0f) {
            if (hasLeftWall && hasRightWall) {
              int diff = ir_FL - ir_FR;
              if (abs(diff) > ir_deadband) {
                steerIR = (diff > 0) ? (5 + (diff - ir_deadband) / 30)
                                     : -(5 + (-diff - ir_deadband) / 30);
              }
            } else if (hasLeftWall && !hasRightWall) {
              int diffL = ir_FL - max_IR[1];
              if (abs(diffL) > ir_deadband) {
                steerIR = (diffL > 0) ? (5 + (diffL - ir_deadband) / 30)
                                      : -(5 + (-diffL - ir_deadband) / 30);
              }
            } else if (!hasLeftWall && hasRightWall) {
              int diffR = ir_FR - max_IR[2];
              if (abs(diffR) > ir_deadband) {
                steerIR = (diffR > 0) ? -(5 + (diffR - ir_deadband) / 30)
                                      : (5 + (-diffR - ir_deadband) / 30);
              }
            }
          }

          // C. Tổng hợp và Ramping PWM
#endif
        int totalSteer = steerGyro;
        if (steerIR != 0)
        {
          int gyroLimit = max(1, abs(steerIR) / 2);
          totalSteer = steerIR + constrain(steerGyro, -gyroLimit, gyroLimit);
        }

        int targetPwm_L = keepMotorRunningPwm(basePwmL + totalSteer);
        int targetPwm_R = keepMotorRunningPwm(basePwmR - totalSteer);

        targetPwm_L = constrain(targetPwm_L + boostPwmL, 0, Turn_Max);
        targetPwm_R = constrain(targetPwm_R + boostPwmR, 0, Turn_Max);

        int newPwm_L = leftDone ? 0 : applyRateLimit(targetPwm_L, pwmL);
        int newPwm_R = rightDone ? 0 : applyRateLimit(targetPwm_R, pwmR);

        setLeftMotor(newPwm_L);
        setRightMotor(newPwm_R);
        break;
      }

      default:
        changeState(IDLE);
        break;
      }
    }

    // Delay cứng đúng 10ms (tính từ thời điểm xLastWakeTime)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==============================================================================
// SETUP (Hàm loop mặc định sẽ bị bỏ trống)
// ==============================================================================
void setup()
{
  Serial.begin(115200);

  // 1. Khởi tạo LED RGB
  pixels.begin();
  pixels.setBrightness(10);
  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // VÀNG: Đang khởi tạo
  pixels.show();

  // 2. Khởi tạo các module con
  initSensors();
  initMotors();
  initIMU();

  // 3. Khởi tạo Web Server (Chạy ở Core 0)
  setupWebServer();

  // 4. Khởi tạo Task Điều khiển chính (Chạy ở Core 1, Ưu tiên Cao nhất)
  xTaskCreatePinnedToCore(
      mainControlTask,          // Hàm task
      "MainControlTask",        // Tên task
      8192,                     // Kích thước stack
      NULL,                     // Tham số truyền vào
      configMAX_PRIORITIES - 1, // Mức ưu tiên cao nhất
      NULL,                     // Handle
      1                         // Chạy trên Core 1
  );

  changeState(IDLE); // Chuyển màu Đỏ (hoàn tất)
}

void loop()
{
  // FreeRTOS đã lo toàn bộ, loop() không làm gì cả.
  vTaskDelete(NULL);
}
