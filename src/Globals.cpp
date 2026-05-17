#include "Globals.h"

namespace {
portMUX_TYPE runtimeParamsMux = portMUX_INITIALIZER_UNLOCKED;
RuntimeParams pendingRuntimeParams;
bool hasPendingRuntimeParams = false;

void copyRuntimeParamsFromGlobals(RuntimeParams &params) {
  params.Kp_L = Kp_L;
  params.Ki_L = Ki_L;
  params.Kd_L = Kd_L;
  params.Kp_R = Kp_R;
  params.Ki_R = Ki_R;
  params.Kd_R = Kd_R;
  params.Kp_T = Kp_T;
  params.Ki_T = Ki_T;
  params.Kd_T = Kd_T;
  params.Turn_Max = Turn_Max;
  params.Turn_Min = Turn_Min;
  params.Turn_Err = Turn_Err;
  params.accel_rate = accel_rate;
  params.max_vel = max_vel;
  params.min_vel = min_vel;
  params.ramp_rate = ramp_rate;
  params.k_gyro = k_gyro;
  params.k_ir = k_ir;
  params.wall_steer_limit = wall_steer_limit;
  params.wheel_trim_L = wheel_trim_L;
  params.wheel_trim_R = wheel_trim_R;
  params.pulses_per_cell = pulses_per_cell;
  params.front_stop_early_margin = front_stop_early_margin;
  params.maze_dead_end_backup_pulses = maze_dead_end_backup_pulses;
  params.maze_dead_end_backup_pwm = maze_dead_end_backup_pwm;
  params.maze_turn_late_pulses = maze_turn_late_pulses;
  params.maze_turn_angle_deg = maze_turn_angle_deg;
  params.point_turn_backup_pulses = point_turn_backup_pulses;
  params.side_ref_L = side_ref_L;
  params.side_ref_FL = side_ref_FL;
  params.side_ref_FR = side_ref_FR;
  params.side_ref_R = side_ref_R;
  params.offset_upper = offset_upper;
  params.offset_lower = offset_lower;
  params.ir_deadband = ir_deadband;
  params.base_pwm = base_pwm;
}

void copyRuntimeParamsToGlobals(const RuntimeParams &params) {
  Kp_L = params.Kp_L;
  Ki_L = params.Ki_L;
  Kd_L = params.Kd_L;
  Kp_R = params.Kp_R;
  Ki_R = params.Ki_R;
  Kd_R = params.Kd_R;
  Kp_T = params.Kp_T;
  Ki_T = params.Ki_T;
  Kd_T = params.Kd_T;
  Turn_Max = params.Turn_Max;
  Turn_Min = params.Turn_Min;
  Turn_Err = params.Turn_Err;
  accel_rate = params.accel_rate;
  max_vel = params.max_vel;
  min_vel = params.min_vel;
  ramp_rate = params.ramp_rate;
  k_gyro = params.k_gyro;
  k_ir = params.k_ir;
  wall_steer_limit = params.wall_steer_limit;
  wheel_trim_L = params.wheel_trim_L;
  wheel_trim_R = params.wheel_trim_R;
  pulses_per_cell = params.pulses_per_cell;
  front_stop_early_margin = params.front_stop_early_margin;
  maze_dead_end_backup_pulses = params.maze_dead_end_backup_pulses;
  maze_dead_end_backup_pwm = params.maze_dead_end_backup_pwm;
  maze_turn_late_pulses = params.maze_turn_late_pulses;
  maze_turn_angle_deg = params.maze_turn_angle_deg;
  point_turn_backup_pulses = params.point_turn_backup_pulses;
  side_ref_L = params.side_ref_L;
  side_ref_FL = params.side_ref_FL;
  side_ref_FR = params.side_ref_FR;
  side_ref_R = params.side_ref_R;
  offset_upper = params.offset_upper;
  offset_lower = params.offset_lower;
  ir_deadband = params.ir_deadband;
  base_pwm = params.base_pwm;
}
}

// ---------------------------------------------------------
// BIẾN IR & HIỆU CHUẨN (Dùng volatile vì được ghi trong task và đọc từ web)
// ---------------------------------------------------------
volatile int ir_L = 0, ir_FL = 0, ir_FR = 0, ir_R = 0;
volatile int raw_ir_L = 0, raw_ir_FL = 0, raw_ir_FR = 0, raw_ir_R = 0;
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
int wall_steer_limit = 22;
int wheel_trim_L = 0;
int wheel_trim_R = 0;
float k_ir = 0.1; // Hệ số bám tường IR
int pulses_per_cell = 980;
int front_stop_early_margin = 800;
int maze_dead_end_backup_pulses = 980;
int maze_dead_end_backup_pwm = 55;
int maze_turn_late_pulses = 120;
float maze_turn_angle_deg = 78.0f;
int point_turn_backup_pulses = 80;
int side_ref_L = 4095;
int side_ref_FL = 4095;
int side_ref_FR = 4095;
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
volatile uint32_t controlTaskLastLoopMs = 0;
volatile uint32_t controlTaskOverrunCount = 0;
volatile int debugSideErrorL = 0;
volatile int debugSideErrorR = 0;
volatile int debugSteerIR = 0;
volatile int debugTotalSteer = 0;

uint8_t maze_wall_map[MAZE_GRID_W][MAZE_GRID_H] = {};
uint8_t maze_known_map[MAZE_GRID_W][MAZE_GRID_H] = {};
uint8_t maze_path_map[MAZE_GRID_W][MAZE_GRID_H] = {};
uint8_t maze_visit_count[MAZE_GRID_W][MAZE_GRID_H] = {};
uint16_t maze_dist_map[MAZE_GRID_W][MAZE_GRID_H] = {};
int maze_start_x = 0;
int maze_start_y = 0;
int maze_start_heading = MAZE_PATH_N;
int maze_goal_x = MAZE_GRID_W / 2;
int maze_goal_y = MAZE_GRID_H / 2;
volatile int debugMazeX = -1;
volatile int debugMazeY = -1;
volatile int debugMazeHeading = MAZE_PATH_N;
volatile int debugMazeNextDir = MAZE_PATH_EMPTY;
volatile int debugMapSenseSource = 0;
volatile int debugMapLeftDir = MAZE_PATH_W;
volatile int debugMapFrontDir = MAZE_PATH_N;
volatile int debugMapRightDir = MAZE_PATH_E;
volatile int debugMapLeftKnown = 0;
volatile int debugMapFrontKnown = 0;
volatile int debugMapRightKnown = 0;
volatile int debugMapLeftWall = 0;
volatile int debugMapFrontWall = 0;
volatile int debugMapRightWall = 0;

void resetMazeMaps() {
  for (int y = 0; y < MAZE_GRID_H; y++) {
    for (int x = 0; x < MAZE_GRID_W; x++) {
      maze_wall_map[x][y] = 0;
      maze_known_map[x][y] = 0;
      maze_path_map[x][y] = MAZE_PATH_EMPTY;
      maze_visit_count[x][y] = 0;
      maze_dist_map[x][y] = MAZE_FLOOD_UNREACHABLE;
    }
  }
  for (int x = 0; x < MAZE_GRID_W; x++) {
    maze_wall_map[x][0] |= MAZE_WALL_S;
    maze_known_map[x][0] |= MAZE_WALL_S;
    maze_wall_map[x][MAZE_GRID_H - 1] |= MAZE_WALL_N;
    maze_known_map[x][MAZE_GRID_H - 1] |= MAZE_WALL_N;
  }
  for (int y = 0; y < MAZE_GRID_H; y++) {
    maze_wall_map[0][y] |= MAZE_WALL_W;
    maze_known_map[0][y] |= MAZE_WALL_W;
    maze_wall_map[MAZE_GRID_W - 1][y] |= MAZE_WALL_E;
    maze_known_map[MAZE_GRID_W - 1][y] |= MAZE_WALL_E;
  }
  maze_start_x = 0;
  maze_start_y = 0;
  maze_start_heading = MAZE_PATH_N;
  maze_goal_x = MAZE_GRID_W / 2;
  maze_goal_y = MAZE_GRID_H / 2;
  debugMazeX = -1;
  debugMazeY = -1;
  debugMazeHeading = MAZE_PATH_N;
  debugMazeNextDir = MAZE_PATH_EMPTY;
  debugMapSenseSource = 0;
  debugMapLeftDir = MAZE_PATH_W;
  debugMapFrontDir = MAZE_PATH_N;
  debugMapRightDir = MAZE_PATH_E;
  debugMapLeftKnown = 0;
  debugMapFrontKnown = 0;
  debugMapRightKnown = 0;
  debugMapLeftWall = 0;
  debugMapFrontWall = 0;
  debugMapRightWall = 0;
}

RuntimeParams captureActiveRuntimeParams() {
  RuntimeParams params;

  portENTER_CRITICAL(&runtimeParamsMux);
  copyRuntimeParamsFromGlobals(params);
  portEXIT_CRITICAL(&runtimeParamsMux);

  return params;
}

void applyActiveRuntimeParams(const RuntimeParams &params) {
  portENTER_CRITICAL(&runtimeParamsMux);
  copyRuntimeParamsToGlobals(params);
  portEXIT_CRITICAL(&runtimeParamsMux);
}

void queuePendingRuntimeParams(const RuntimeParams &params) {
  portENTER_CRITICAL(&runtimeParamsMux);
  pendingRuntimeParams = params;
  hasPendingRuntimeParams = true;
  portEXIT_CRITICAL(&runtimeParamsMux);
}

bool consumePendingRuntimeParams(RuntimeParams &params) {
  bool hasParams;

  portENTER_CRITICAL(&runtimeParamsMux);
  hasParams = hasPendingRuntimeParams;
  if (hasParams) {
    params = pendingRuntimeParams;
    hasPendingRuntimeParams = false;
  }
  portEXIT_CRITICAL(&runtimeParamsMux);

  return hasParams;
}

// ---------------------------------------------------------
// ĐỐI TƯỢNG PHẦN CỨNG (LED, MPU)
// ---------------------------------------------------------
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile float continuousYaw = 0.0;
float target_yaw = 0.0;
volatile float debugGyroZRawDps = 0.0f;
volatile float debugGyroZCorrectedDps = 0.0f;
volatile float debugGyroZBiasDps = 0.0f;
volatile float debugMpuTempC = 0.0f;
volatile float debugYawDriftDpm = 0.0f;
volatile int debugGyroZAutoBiasActive = 0;
volatile uint32_t debugGyroZAutoBiasStillMs = 0;
volatile int debugGyroZAutoBiasReason = 1;

SemaphoreHandle_t i2cMutex = NULL;
