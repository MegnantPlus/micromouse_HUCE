#pragma once

#include <stdint.h>

#define LED_PIN 48
#define NUMPIXELS 1

#define PIN_SDA 8
#define PIN_SCL 9

#define ENC_L_C1 18
#define ENC_L_C2 5
#define ENC_R_C1 7
#define ENC_R_C2 6

#define MOT_L_PWM 10
#define MOT_L_DIR 11
#define MOT_R_PWM 12
#define MOT_R_DIR 13

#define IR_RX_L 1
#define IR_RX_FL 2
#define IR_RX_FR 3
#define IR_RX_R 4

#define IR_TX_L 14
#define IR_TX_FL 15
#define IR_TX_FR 16
#define IR_TX_R 17

constexpr int FRONT_STOP_EARLY_MARGIN = 800;
constexpr unsigned long FRONT_STOP_DECEL_MS = 60;
constexpr int FRONT_STOP_BRAKE_RAMP_MULTIPLIER = 3;

constexpr bool AUTO_START = true;

constexpr int MAZE_GRID_W = 16;
constexpr int MAZE_GRID_H = 16;
constexpr int MAZE_GRID_CELLS = MAZE_GRID_W * MAZE_GRID_H;

constexpr uint8_t MAZE_WALL_W = 0x01;
constexpr uint8_t MAZE_WALL_E = 0x02;
constexpr uint8_t MAZE_WALL_S = 0x04;
constexpr uint8_t MAZE_WALL_N = 0x08;

constexpr uint8_t MAZE_PATH_N = 0;
constexpr uint8_t MAZE_PATH_E = 1;
constexpr uint8_t MAZE_PATH_S = 2;
constexpr uint8_t MAZE_PATH_W = 3;
constexpr uint8_t MAZE_PATH_STOP = 4;
constexpr uint8_t MAZE_PATH_EMPTY = 255;

enum RunState {
  IDLE,
  WAIT_1S,
  BLIND_START,
  PID_RUN,
  DECEL_STOP,
  TEST_L,
  TEST_R,
  TEST_TURN_L,
  TEST_TURN_R,
  TEST_BACK_ONE_CELL,
  PID_RUN_ONE_CELL,
  MAZE_RIGHT_HAND,
  MAZE_RIGHT_HAND_CELL,
  MAZE_MATRIX_RUN
};
