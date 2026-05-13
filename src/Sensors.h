#pragma once

#include <Arduino.h>
#include "Globals.h"

/**
 * @brief Khởi tạo các chân IO cho cảm biến IR.
 */
void initSensors();

/**
 * @brief Đọc giá trị từ 4 cảm biến hồng ngoại (IR) luân phiên.
 * Hàm này dùng cơ chế quét nhanh, không dùng delay(10) gây nghẽn vòng lặp.
 */
void readIR_TDM(IrSnapshot *rawOut = nullptr);

/**
 * @brief Lay snapshot 4 mat IR trong mot critical section.
 */
IrSnapshot getIrSnapshot();
