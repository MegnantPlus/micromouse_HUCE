#include "IMU.h"
#include "Config.h"
#include "Globals.h"
#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;
float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
float prevRawAngleZ = 0;
int rotations = 0;

#define twoKp (2.0f * 1.5f)
#define twoKi (0.0f)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
volatile bool imuRezeroActive = false;

const uint32_t GYRO_Z_AUTO_BIAS_STILL_MS = 1200;
const float GYRO_Z_AUTO_BIAS_MAX_STILL_DPS = 1.0f;
const float GYRO_Z_AUTO_BIAS_ALPHA = 0.01f;
const float GYRO_Z_AUTO_BIAS_MAX_STEP_DPS = 0.003f;
const long GYRO_Z_AUTO_BIAS_MAX_ENCODER_DELTA = 2;
long autoBiasStartPulseL = 0;
long autoBiasStartPulseR = 0;
uint32_t autoBiasStillStartMs = 0;

int16_t readMpuWord() {
  int high = Wire.read();
  int low = Wire.read();
  return (int16_t)((high << 8) | low);
}

void updateGyroZAutoBias(float gyroZRawDps) {
  uint32_t nowMs = millis();
  long currentPulseL = pulseL;
  long currentPulseR = pulseR;
  bool idleState = carState == IDLE && !isRunning && !isTurningTask;
  bool motorsStopped = pwmL == 0 && pwmR == 0;
  float biasError = gyroZRawDps - GyroErrorZ;
  bool gyroLooksStill = fabsf(biasError) <= GYRO_Z_AUTO_BIAS_MAX_STILL_DPS;

  if (!idleState) {
    autoBiasStillStartMs = 0;
    debugGyroZAutoBiasStillMs = 0;
    debugGyroZAutoBiasActive = 0;
    debugGyroZAutoBiasReason = 2;
    return;
  }

  if (!motorsStopped) {
    autoBiasStillStartMs = 0;
    debugGyroZAutoBiasStillMs = 0;
    debugGyroZAutoBiasActive = 0;
    debugGyroZAutoBiasReason = 3;
    return;
  }

  if (!gyroLooksStill) {
    autoBiasStillStartMs = 0;
    debugGyroZAutoBiasStillMs = 0;
    debugGyroZAutoBiasActive = 0;
    debugGyroZAutoBiasReason = 5;
    return;
  }

  if (autoBiasStillStartMs == 0) {
      autoBiasStillStartMs = nowMs;
    autoBiasStartPulseL = currentPulseL;
    autoBiasStartPulseR = currentPulseR;
  }

  bool encodersStill =
      labs(currentPulseL - autoBiasStartPulseL) <=
          GYRO_Z_AUTO_BIAS_MAX_ENCODER_DELTA &&
      labs(currentPulseR - autoBiasStartPulseR) <=
          GYRO_Z_AUTO_BIAS_MAX_ENCODER_DELTA;

  if (!encodersStill) {
    autoBiasStillStartMs = 0;
    debugGyroZAutoBiasStillMs = 0;
    debugGyroZAutoBiasActive = 0;
    debugGyroZAutoBiasReason = 4;
    return;
  }

  debugGyroZAutoBiasStillMs = nowMs - autoBiasStillStartMs;

  if (debugGyroZAutoBiasStillMs >= GYRO_Z_AUTO_BIAS_STILL_MS) {
    float step = constrain(biasError * GYRO_Z_AUTO_BIAS_ALPHA,
                           -GYRO_Z_AUTO_BIAS_MAX_STEP_DPS,
                           GYRO_Z_AUTO_BIAS_MAX_STEP_DPS);
    GyroErrorZ += step;
    debugGyroZAutoBiasActive = 1;
    debugGyroZAutoBiasReason = 0;
  } else {
    debugGyroZAutoBiasActive = 0;
    debugGyroZAutoBiasReason = 1;
  }
}

void writeRegister(uint8_t reg, uint8_t data) {
  if (i2cMutex != NULL) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                  float dt) {
  float recipNorm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    integralFBx += twoKi * halfex * dt;
    integralFBy += twoKi * halfey * dt;
    integralFBz += twoKi * halfez * dt;
    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void calibrateMPU(int duration_ms) {
  long num_samples = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  unsigned long start_time = millis();

  while (millis() - start_time < duration_ms) {
    if (i2cMutex != NULL) xSemaphoreTake(i2cMutex, portMAX_DELAY);
    Wire.beginTransmission((uint8_t)MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_ADDR, (size_t)6, true);
    sum_gx += (float)readMpuWord() / 65.5f;
    sum_gy += (float)readMpuWord() / 65.5f;
    sum_gz += (float)readMpuWord() / 65.5f;
    if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);
    
    num_samples++;
    delay(2);
  }
  GyroErrorX = sum_gx / num_samples;
  GyroErrorY = sum_gy / num_samples;
  GyroErrorZ = sum_gz / num_samples;
  debugGyroZBiasDps = GyroErrorZ;
  debugGyroZRawDps = GyroErrorZ;
  debugGyroZCorrectedDps = 0.0f;
  debugYawDriftDpm = 0.0f;
  debugGyroZAutoBiasActive = 0;
  debugGyroZAutoBiasStillMs = 0;
  debugGyroZAutoBiasReason = 1;
  autoBiasStillStartMs = 0;
  autoBiasStartPulseL = pulseL;
  autoBiasStartPulseR = pulseR;
  
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  integralFBx = integralFBy = integralFBz = prevRawAngleZ = 0;
  continuousYaw = 0;
  rotations = 0;
}

bool rezeroIMU(int duration_ms) {
  if (imuRezeroActive)
    return false;

  imuRezeroActive = true;
  debugGyroZAutoBiasActive = 0;
  debugGyroZAutoBiasStillMs = 0;
  debugGyroZAutoBiasReason = 1;

  calibrateMPU(constrain(duration_ms, 300, 3000));
  target_yaw = continuousYaw;

  imuRezeroActive = false;
  return true;
}

void initIMU() {
  if (i2cMutex == NULL) {
    i2cMutex = xSemaphoreCreateMutex();
  }
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  writeRegister(0x6B, 0x00);
  writeRegister(0x1A, 0x04);
  writeRegister(0x1B, 0x08);
  writeRegister(0x1C, 0x08);
  calibrateMPU(3000); // Tĩnh trong 3s
}

void updateIMU(float dt) {
  if (imuRezeroActive)
    return;

  if (i2cMutex != NULL) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (size_t)14, true);
  
  float accX = (float)readMpuWord() / 8192.0f;
  float accY = (float)readMpuWord() / 8192.0f;
  float accZ = (float)readMpuWord() / 8192.0f;
  Wire.read(); Wire.read(); // Bỏ qua nhiệt độ
  float gyroXRawDps = (float)readMpuWord() / 65.5f;
  float gyroYRawDps = (float)readMpuWord() / 65.5f;
  float gyroZRawDps = (float)readMpuWord() / 65.5f;
  if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);

  if (i2cMutex != NULL) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(0x41);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (size_t)2, true);
  int16_t tempRaw = readMpuWord();
  if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);

  updateGyroZAutoBias(gyroZRawDps);
  float gyroZCorrectedDps = gyroZRawDps - GyroErrorZ;
  float gyroX_rad = (gyroXRawDps - GyroErrorX) * (PI / 180.0f);
  float gyroY_rad = (gyroYRawDps - GyroErrorY) * (PI / 180.0f);
  float gyroZ_rad = gyroZCorrectedDps * (PI / 180.0f);

  debugGyroZRawDps = gyroZRawDps;
  debugGyroZCorrectedDps = gyroZCorrectedDps;
  debugGyroZBiasDps = GyroErrorZ;
  debugMpuTempC = ((float)tempRaw / 340.0f) + 36.53f;
  debugYawDriftDpm = gyroZCorrectedDps * 60.0f;

  MahonyUpdate(gyroX_rad, gyroY_rad, gyroZ_rad, accX, accY, accZ, dt);
  
  float rawAngleZ = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (180.0f / PI);
  float deltaYaw = rawAngleZ - prevRawAngleZ;

  if (deltaYaw < -180.0f)
    rotations++;
  else if (deltaYaw > 180.0f)
    rotations--;

  prevRawAngleZ = rawAngleZ;
  continuousYaw = rawAngleZ + (rotations * 360.0f);
}
