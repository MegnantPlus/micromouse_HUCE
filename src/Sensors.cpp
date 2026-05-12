#include "Sensors.h"
#include "Config.h"
#include "Globals.h"

// --- THUẬT TOÁN LỌC TRUNG VỊ (MEDIAN FILTER) ---
int getMedian(int arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (arr[i] > arr[j]) {
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
      }
    }
  }
  return arr[n / 2];
}

// --- HÀM ĐỌC 1 MẮT ---
int readSingleEye(int tx_pin, int rx_pin) {
  int samples[3]; // Lấy 3 mẫu để chạy nhanh hơn trong task 10ms

  digitalWrite(tx_pin, HIGH); // Bật LED phát

  // Nạp tụ điện ký sinh
  delayMicroseconds(350);

  for (int i = 0; i < 3; i++) {
    samples[i] = analogRead(rx_pin);
    if (i < 2) delayMicroseconds(200);
  }

  digitalWrite(tx_pin, LOW); // Tắt LED phát

  return getMedian(samples, 3);
}

void initSensors() {
  pinMode(IR_TX_L, OUTPUT);
  pinMode(IR_TX_FL, OUTPUT);
  pinMode(IR_TX_FR, OUTPUT);
  pinMode(IR_TX_R, OUTPUT);
  analogReadResolution(12);
}

// ---------------------------------------------------------
// THUẬT TOÁN ĐỌC LED 4 MẮT SIÊU NHANH (~3ms tổng)
// ---------------------------------------------------------
void readIR_TDM() {
  const float alpha = 0.3f; // Hệ số EMA lọc nhiễu lai

  // Quét luân phiên cực nhanh, không làm nghẽn vòng lặp FreeRTOS
  int raw_L = readSingleEye(IR_TX_L, IR_RX_L);
  ir_L = (int)(alpha * raw_L + (1.0f - alpha) * ir_L);
  delayMicroseconds(100); // Ngắt nhiễu quang học nhỏ
  
  int raw_FL = readSingleEye(IR_TX_FL, IR_RX_FL);
  ir_FL = (int)(alpha * raw_FL + (1.0f - alpha) * ir_FL);
  delayMicroseconds(100);
  
  int raw_FR = readSingleEye(IR_TX_FR, IR_RX_FR);
  ir_FR = (int)(alpha * raw_FR + (1.0f - alpha) * ir_FR);
  delayMicroseconds(100);
  
  int raw_R = readSingleEye(IR_TX_R, IR_RX_R);
  ir_R = (int)(alpha * raw_R + (1.0f - alpha) * ir_R);
}