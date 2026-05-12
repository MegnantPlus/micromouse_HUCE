#include "Globals.h"

// ---------------------------------------------------------
// BIẾN IR & HIỆU CHUẨN (Dùng volatile vì được ghi trong task và đọc từ web)
// ---------------------------------------------------------
volatile int ir_L = 0, ir_FL = 0, ir_FR = 0, ir_R = 0;
int base_L = 0, base_FL = 0, base_FR = 0, base_R = 0; // Giá trị tường chuẩn

// --- BIẾN NGƯỠNG & ĐIỀU KHIỂN BĂM XUNG ---
int max_IR[4] = {4095, 4095, 4095, 4095};
int min_IR[4] = {0, 0, 0, 0};
int offset_upper = 500;
int offset_lower = 700;
int ir_deadband = 150;
int base_pwm = 80; // Tốc độ rùa bò an toàn

// ---------------------------------------------------------
// BIẾN TOÀN CỤC WEB / PID
// ---------------------------------------------------------
float Kp_L = 5.025, Ki_L = 0.0, Kd_L = 15.1;
float Kp_R = 5.0, Ki_R = 0.0, Kd_R = 15.1;

float Kp_T = 4.82, Ki_T = 0.1, Kd_T = 1.0;
int Turn_Max = 100;
int Turn_Min = 35;
float Turn_Err = 0.5;

float accel_rate = 0.2;
float max_vel = 15.0;
float min_vel = 5.0;

int ramp_rate = 10; // Gioi han tang toc (slew-rate limiter)
float k_gyro = 6.0;
float k_ir = 0.1; // Hệ số bám tường IR
int pulses_per_cell = 980;
int side_ref_L = 4095;
int side_ref_R = 4095;

// ---------------------------------------------------------
// BIẾN ĐIỀU KHIỂN & TRẠNG THÁI
// ---------------------------------------------------------
volatile long pulseL = 0, pulseR = 0;
int pwmL = 0, pwmR = 0;

float virtualPos = 0.0f, virtualVel = 0.0f;
float integralL = 0, integralR = 0, prevErrL = 0, prevErrR = 0, dFilterL = 0,
      dFilterR = 0;
float integralT = 0, prevErrT = 0, dFilterT = 0;

volatile RunState carState = IDLE;
volatile unsigned long stateStartTime = 0;

volatile RunState requestedState = IDLE;
volatile bool stateChangeRequested = false;

volatile bool isRunning = false;
volatile bool isTurningTask = false;

// ---------------------------------------------------------
// ĐỐI TƯỢNG PHẦN CỨNG (LED, MPU)
// ---------------------------------------------------------
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile float continuousYaw = 0.0;
float target_yaw = 0.0;

SemaphoreHandle_t i2cMutex = NULL;
