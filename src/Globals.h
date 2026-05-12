#pragma once

#include "Config.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// ---------------------------------------------------------
// BIẾN IR & HIỆU CHUẨN
// ---------------------------------------------------------
extern volatile int ir_L, ir_FL, ir_FR, ir_R;
extern int base_L, base_FL, base_FR, base_R;

// ---------------------------------------------------------
// BIẾN TOÀN CỤC WEB / PID
// ---------------------------------------------------------
extern float Kp_L, Ki_L, Kd_L;
extern float Kp_R, Ki_R, Kd_R;
extern float Kp_T, Ki_T, Kd_T;

extern int Turn_Max;
extern int Turn_Min;
extern float Turn_Err;

extern float accel_rate;
extern float max_vel;
extern float min_vel;

extern int ramp_rate;
extern float k_gyro;
extern float k_ir;
extern int pulses_per_cell;
extern int side_ref_L;
extern int side_ref_R;

// --- BIẾN NGƯỠNG & ĐIỀU KHIỂN BĂM XUNG ---
extern int max_IR[4];
extern int min_IR[4];
extern int offset_upper;
extern int offset_lower;
extern int ir_deadband;
extern int base_pwm;

// ---------------------------------------------------------
// BIẾN ĐIỀU KHIỂN & TRẠNG THÁI
// ---------------------------------------------------------
extern volatile long pulseL, pulseR;
extern int pwmL, pwmR;

extern float virtualPos, virtualVel;
extern float integralL, integralR, prevErrL, prevErrR, dFilterL, dFilterR;
extern float integralT, prevErrT, dFilterT;

extern volatile RunState carState;
extern volatile unsigned long stateStartTime;

extern volatile RunState requestedState;
extern volatile bool stateChangeRequested;

extern volatile bool isRunning;
extern volatile bool isTurningTask;

// ---------------------------------------------------------
// ĐỐI TƯỢNG PHẦN CỨNG (LED, MPU)
// ---------------------------------------------------------
extern Adafruit_NeoPixel pixels;

extern volatile float continuousYaw;
extern float target_yaw;

// FreeRTOS Mutex để bảo vệ I2C (nếu cần)
extern SemaphoreHandle_t i2cMutex;
