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

  digitalWrite(tx_pin, LOW);
  delayMicroseconds(80);

  for (int i = 0; i < 3; i++) {
    int ambient = analogRead(rx_pin);

    digitalWrite(tx_pin, HIGH);
    delayMicroseconds(220);
    int reflected = analogRead(rx_pin);
    digitalWrite(tx_pin, LOW);

    samples[i] = max(0, reflected - ambient);
    if (i < 2) delayMicroseconds(120);
  }

  return getMedian(samples, 3);
}

void initSensors() {
  pinMode(IR_TX_L, OUTPUT);
  pinMode(IR_TX_FL, OUTPUT);
  pinMode(IR_TX_FR, OUTPUT);
  pinMode(IR_TX_R, OUTPUT);

  digitalWrite(IR_TX_L, LOW);
  digitalWrite(IR_TX_FL, LOW);
  digitalWrite(IR_TX_FR, LOW);
  digitalWrite(IR_TX_R, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(IR_RX_L, ADC_11db);
  analogSetPinAttenuation(IR_RX_FL, ADC_11db);
  analogSetPinAttenuation(IR_RX_FR, ADC_11db);
  analogSetPinAttenuation(IR_RX_R, ADC_11db);
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
  const float alpha = 0.2f;
  const int maxStepPerCycle = 140;
  const int noiseDeadband = 6;
  IrSnapshot previous = getIrSnapshot();
  IrSnapshot next;
  IrSnapshot raw;

  raw.left = readSingleEye(IR_TX_L, IR_RX_L);
  int limitedLeft = previous.left + constrain(raw.left - previous.left, -maxStepPerCycle, maxStepPerCycle);
  next.left = (int)(alpha * limitedLeft + (1.0f - alpha) * previous.left);
  if (abs(next.left - previous.left) <= noiseDeadband) next.left = previous.left;
  delayMicroseconds(100);

  raw.frontLeft = readSingleEye(IR_TX_FL, IR_RX_FL);
  int limitedFrontLeft = previous.frontLeft + constrain(raw.frontLeft - previous.frontLeft, -maxStepPerCycle, maxStepPerCycle);
  next.frontLeft = (int)(alpha * limitedFrontLeft + (1.0f - alpha) * previous.frontLeft);
  if (abs(next.frontLeft - previous.frontLeft) <= noiseDeadband) next.frontLeft = previous.frontLeft;
  delayMicroseconds(100);

  raw.frontRight = readSingleEye(IR_TX_FR, IR_RX_FR);
  int limitedFrontRight = previous.frontRight + constrain(raw.frontRight - previous.frontRight, -maxStepPerCycle, maxStepPerCycle);
  next.frontRight = (int)(alpha * limitedFrontRight + (1.0f - alpha) * previous.frontRight);
  if (abs(next.frontRight - previous.frontRight) <= noiseDeadband) next.frontRight = previous.frontRight;
  delayMicroseconds(100);

  raw.right = readSingleEye(IR_TX_R, IR_RX_R);
  int limitedRight = previous.right + constrain(raw.right - previous.right, -maxStepPerCycle, maxStepPerCycle);
  next.right = (int)(alpha * limitedRight + (1.0f - alpha) * previous.right);
  if (abs(next.right - previous.right) <= noiseDeadband) next.right = previous.right;

  if (rawOut != nullptr) {
    *rawOut = raw;
  }

  raw_ir_L = raw.left;
  raw_ir_FL = raw.frontLeft;
  raw_ir_FR = raw.frontRight;
  raw_ir_R = raw.right;

  publishIrSnapshot(next);
}
