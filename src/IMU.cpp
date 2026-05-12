#include "IMU.h"
#include "Config.h"
#include "Globals.h"
#include <Wire.h>

const int MPU_ADDR = 0x68;
float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
float prevRawAngleZ = 0;
int rotations = 0;

#define twoKp (2.0f * 1.5f)
#define twoKi (0.0f)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void writeRegister(uint8_t reg, uint8_t data) {
  if (i2cMutex != NULL) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU_ADDR);
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
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    sum_gx += (float)(Wire.read() << 8 | Wire.read()) / 65.5f;
    sum_gy += (float)(Wire.read() << 8 | Wire.read()) / 65.5f;
    sum_gz += (float)(Wire.read() << 8 | Wire.read()) / 65.5f;
    if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);
    
    num_samples++;
    delay(2);
  }
  GyroErrorX = sum_gx / num_samples;
  GyroErrorY = sum_gy / num_samples;
  GyroErrorZ = sum_gz / num_samples;
  
  q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
  integralFBx = integralFBy = integralFBz = prevRawAngleZ = 0;
  continuousYaw = 0;
  rotations = 0;
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
  if (i2cMutex != NULL) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  float accX = (float)(Wire.read() << 8 | Wire.read()) / 8192.0f;
  float accY = (float)(Wire.read() << 8 | Wire.read()) / 8192.0f;
  float accZ = (float)(Wire.read() << 8 | Wire.read()) / 8192.0f;
  Wire.read(); Wire.read(); // Bỏ qua nhiệt độ
  float gyroX_rad = (((float)(Wire.read() << 8 | Wire.read()) / 65.5f) - GyroErrorX) * (PI / 180.0f);
  float gyroY_rad = (((float)(Wire.read() << 8 | Wire.read()) / 65.5f) - GyroErrorY) * (PI / 180.0f);
  float gyroZ_rad = (((float)(Wire.read() << 8 | Wire.read()) / 65.5f) - GyroErrorZ) * (PI / 180.0f);
  if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);

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
