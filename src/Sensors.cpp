#include "Sensors.h"
#include "Config.h"
#include "Globals.h"

namespace {
portMUX_TYPE irMux = portMUX_INITIALIZER_UNLOCKED;

void publishIrSnapshot(const IrSnapshot &snapshot) {
  portENTER_CRITICAL(&irMux);
  ir_L = snapshot.left;
  ir_FL = snapshot.frontLeft;
  ir_FR = snapshot.frontRight;
  ir_R = snapshot.right;
  portEXIT_CRITICAL(&irMux);
}
}

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

int readSingleEye(int tx_pin, int rx_pin) {
  int samples[3];

  digitalWrite(tx_pin, HIGH);
  delayMicroseconds(350);

  for (int i = 0; i < 3; i++) {
    samples[i] = analogRead(rx_pin);
    if (i < 2) delayMicroseconds(200);
  }

  digitalWrite(tx_pin, LOW);
  return getMedian(samples, 3);
}

void initSensors() {
  pinMode(IR_TX_L, OUTPUT);
  pinMode(IR_TX_FL, OUTPUT);
  pinMode(IR_TX_FR, OUTPUT);
  pinMode(IR_TX_R, OUTPUT);
  analogReadResolution(12);
}

IrSnapshot getIrSnapshot() {
  IrSnapshot snapshot;

  portENTER_CRITICAL(&irMux);
  snapshot.left = ir_L;
  snapshot.frontLeft = ir_FL;
  snapshot.frontRight = ir_FR;
  snapshot.right = ir_R;
  portEXIT_CRITICAL(&irMux);

  return snapshot;
}

void readIR_TDM(IrSnapshot *rawOut) {
  const float alpha = 0.3f;
  IrSnapshot previous = getIrSnapshot();
  IrSnapshot next;
  IrSnapshot raw;

  raw.left = readSingleEye(IR_TX_L, IR_RX_L);
  next.left = (int)(alpha * raw.left + (1.0f - alpha) * previous.left);
  delayMicroseconds(100);

  raw.frontLeft = readSingleEye(IR_TX_FL, IR_RX_FL);
  next.frontLeft = (int)(alpha * raw.frontLeft + (1.0f - alpha) * previous.frontLeft);
  delayMicroseconds(100);

  raw.frontRight = readSingleEye(IR_TX_FR, IR_RX_FR);
  next.frontRight = (int)(alpha * raw.frontRight + (1.0f - alpha) * previous.frontRight);
  delayMicroseconds(100);

  raw.right = readSingleEye(IR_TX_R, IR_RX_R);
  next.right = (int)(alpha * raw.right + (1.0f - alpha) * previous.right);

  if (rawOut != nullptr) {
    *rawOut = raw;
  }

  publishIrSnapshot(next);
}
