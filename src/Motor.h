#pragma once

#include <Arduino.h>

struct PulseSnapshot {
  long left;
  long right;
};

/**
 * @brief Khởi tạo các chân IO và Ngắt cho động cơ và Encoder.
 */
void initMotors();

/**
 * @brief Xóa hoàn toàn trạng thái của Motor, Encoder và PID. 
 * Chống hiện tượng chớp tắt hoặc giật cục khi đổi State.
 */
void resetMotorsAndPID();

/**
 * @brief Điều khiển Motor Trái. Dương là đi tới, âm là đi lùi.
 */
void setLeftMotor(int pwm);

/**
 * @brief Điều khiển Motor Phải. Dương là đi tới, âm là đi lùi.
 */
void setRightMotor(int pwm);

/**
 * @brief Hãm phanh tức thời cả 2 động cơ.
 */
void brakeMotors();

/**
 * @brief Giới hạn tốc độ thay đổi xung (Slew-rate limiter / Ramping).
 * Chống tụt áp do dòng khởi động quá cao.
 */
int applyRateLimit(int pwmNew, int pwmPrev);
int applyRateLimit(int pwmNew, int pwmPrev, int rateLimit);
PulseSnapshot getPulseSnapshot();
