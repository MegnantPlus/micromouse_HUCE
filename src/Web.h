#pragma once

#include <Arduino.h>

/**
 * @brief Khởi tạo Captive Portal và Web Server.
 * Chạy trên Core 0, không ảnh hưởng đến vòng lặp điều khiển PID trên Core 1.
 */
void setupWebServer();
