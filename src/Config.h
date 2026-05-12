#pragma once

// --- CẤU HÌNH LED RGB ---
#define LED_PIN 48
#define NUMPIXELS 1

// ---------------------------------------------------------
// PINOUT MỚI (ADC1 CHUYÊN DỤNG CHO MẮT THU)
// ---------------------------------------------------------
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

// MẮT THU (RX) - Bắt buộc dùng ADC1
#define IR_RX_L 1  // Trái
#define IR_RX_FL 2 // Trước Trái
#define IR_RX_FR 3 // Trước Phải
#define IR_RX_R 4  // Phải

// MẮT PHÁT (TX) - Kích Mosfet kênh N
#define IR_TX_L 14
#define IR_TX_FL 15
#define IR_TX_FR 16
#define IR_TX_R 17

// Enum trạng thái xe
enum RunState { IDLE, WAIT_1S, BLIND_START, PID_RUN, TEST_L, TEST_R };
