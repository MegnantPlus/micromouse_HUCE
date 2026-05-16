#pragma once

#include <Arduino.h>

/**
 * @brief Khởi tạo cảm biến MPU6050, thiết lập cấu hình và gọi hàm hiệu chuẩn.
 */
void initIMU();

/**
 * @brief Đọc dữ liệu MPU6050, đưa qua bộ lọc Mahony để cập nhật góc continuousYaw.
 * @param dt Thời gian delta time kể từ lần gọi trước (giây).
 */
void updateIMU(float dt);

/**
 * @brief Lay lai bias gyro va reset yaw ve 0. Chi goi khi xe dung yen.
 * @param duration_ms Thoi gian lay mau bias.
 * @return false neu dang co lan rezero khac chay.
 */
bool rezeroIMU(int duration_ms);
