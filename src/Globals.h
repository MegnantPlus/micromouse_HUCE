#pragma once

#include "Config.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// ---------------------------------------------------------
// BIẾN IR & HIỆU CHUẨN
// ---------------------------------------------------------
extern volatile int ir_L, ir_FL, ir_FR, ir_R;
extern volatile int raw_ir_L, raw_ir_FL, raw_ir_FR, raw_ir_R;
extern int base_L, base_FL, base_FR, base_R;

struct IrSnapshot {
	int left;
	int frontLeft;
	int frontRight;
	int right;
};

struct RuntimeParams {
	float Kp_L;
	float Ki_L;
	float Kd_L;
	float Kp_R;
	float Ki_R;
	float Kd_R;
	float Kp_T;
	float Ki_T;
	float Kd_T;
	int Turn_Max;
	int Turn_Min;
	float Turn_Err;
	float accel_rate;
	float max_vel;
	float min_vel;
	int ramp_rate;
	float k_gyro;
	float k_ir;
	int wall_steer_limit;
	int wheel_trim_L;
	int wheel_trim_R;
	int pulses_per_cell;
	int front_stop_early_margin;
	int maze_dead_end_backup_pulses;
	int maze_dead_end_backup_pwm;
	int side_ref_L;
	int side_ref_FL;
	int side_ref_FR;
	int side_ref_R;
	int offset_upper;
	int offset_lower;
	int ir_deadband;
	int base_pwm;
};

// Sensor numbering: use 1..4 in code (user-facing). Map to internal 0-based arrays with SIDX(x).
#define S_L 1
#define S_FL 2
#define S_FR 3
#define S_R 4
#define SIDX(s) ((s) - 1)

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
extern int wall_steer_limit;
extern int wheel_trim_L;
extern int wheel_trim_R;
extern int pulses_per_cell;
extern int front_stop_early_margin;
extern int maze_dead_end_backup_pulses;
extern int maze_dead_end_backup_pwm;
extern int side_ref_L;
extern int side_ref_FL;
extern int side_ref_FR;
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
extern volatile uint32_t controlTaskLastLoopMs;
extern volatile uint32_t controlTaskOverrunCount;
extern volatile int debugSideErrorL;
extern volatile int debugSideErrorR;
extern volatile int debugSteerIR;
extern volatile int debugTotalSteer;

RuntimeParams captureActiveRuntimeParams();
void applyActiveRuntimeParams(const RuntimeParams &params);
void queuePendingRuntimeParams(const RuntimeParams &params);
bool consumePendingRuntimeParams(RuntimeParams &params);

// ---------------------------------------------------------
// ĐỐI TƯỢNG PHẦN CỨNG (LED, MPU)
// ---------------------------------------------------------
extern Adafruit_NeoPixel pixels;

extern volatile float continuousYaw;
extern float target_yaw;

// FreeRTOS Mutex để bảo vệ I2C (nếu cần)
extern SemaphoreHandle_t i2cMutex;
