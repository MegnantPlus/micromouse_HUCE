#include "Motor.h"
#include "Config.h"
#include "Globals.h"

portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isr_Left() {
  portENTER_CRITICAL_ISR(&motorMux);
  if (digitalRead(ENC_L_C2)) pulseL++;
  else pulseL--;
  portEXIT_CRITICAL_ISR(&motorMux);
}

void IRAM_ATTR isr_Right() {
  portENTER_CRITICAL_ISR(&motorMux);
  if (digitalRead(ENC_R_C2)) pulseR++;
  else pulseR--;
  portEXIT_CRITICAL_ISR(&motorMux);
}

void initMotors() {
  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isr_Left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isr_Right, RISING);

  pinMode(MOT_L_DIR, OUTPUT);
  pinMode(MOT_L_PWM, OUTPUT);
  pinMode(MOT_R_DIR, OUTPUT);
  pinMode(MOT_R_PWM, OUTPUT);
  brakeMotors();
}

void resetMotorsAndPID() {
  brakeMotors();
  portENTER_CRITICAL(&motorMux);
  pulseL = 0;
  pulseR = 0;
  portEXIT_CRITICAL(&motorMux);
  virtualPos = 0.0f;
  virtualVel = 0.0f;
  integralL = integralR = prevErrL = prevErrR = dFilterL = dFilterR = 0;
  integralT = prevErrT = dFilterT = 0;
}

void setLeftMotor(int pwm) {
  if (pwm >= 0) {
    digitalWrite(MOT_L_DIR, LOW);
    analogWrite(MOT_L_PWM, pwm);
  } else {
    digitalWrite(MOT_L_DIR, HIGH);
    analogWrite(MOT_L_PWM, 255 + pwm);
  }
  pwmL = pwm;
}

void setRightMotor(int pwm) {
  if (pwm >= 0) {
    digitalWrite(MOT_R_DIR, LOW);
    analogWrite(MOT_R_PWM, pwm);
  } else {
    digitalWrite(MOT_R_DIR, HIGH);
    analogWrite(MOT_R_PWM, 255 + pwm);
  }
  pwmR = pwm;
}

void brakeMotors() {
  digitalWrite(MOT_L_DIR, LOW);
  analogWrite(MOT_L_PWM, 0);
  digitalWrite(MOT_R_DIR, LOW);
  analogWrite(MOT_R_PWM, 0);
  pwmL = 0;
  pwmR = 0;
}

int applyRateLimit(int pwmNew, int pwmPrev) {
  int delta = pwmNew - pwmPrev;
  if (delta > ramp_rate) return pwmPrev + ramp_rate;
  if (delta < -ramp_rate) return pwmPrev - ramp_rate;
  return pwmNew;
}
