#include "pid.h"

#ifndef OMNI_WHEEL_H
#define OMNI_WHEEL_H

#include <Arduino.h>

const int WHEEL_COUNT = 3;

// #define TEST
// #define MOTOR_TEST
// #define NOENCODER

// following math radian rotation
// FrontRight, FrontLeft, BackLeft, BackRight

double setXspeed = 0.0;
double setYspeed = 0.0;
double setRspeed = 0.0;

double motorXspeed = 0.0;
double motorYspeed = 0.0;
double motorRspeed = 0.0;

PID pidX = PID(0.5, 0.0, 0.0);
PID pidY = pidX;
PID pidR = pidX;

const double MAX_ACCELERATION = 200.0;

const double STARTUP_DURATION = 0.1; // necessary for encoder interrupts to stabilize
double currentStartupDuration = 0.0;

// const double FIXED_DELTA = 0.01; // to stabilize
// double elapsedDuration = 0.0;

// y > 0 - forward
// y < 0 - backward
// x > 0 - right
// x < 0 - left
void setMotorSpeeds(double x, double y, double rotation) {
  const int MINIMUM_REQUIRED_PWM = 0;
  for (int i = 0; i < WHEEL_COUNT; i++) {
    const double *OMNI_WHEEL_DIRECTION = OMNI_WHEEL_DIRECTIONS[i];
    double speed = OMNI_WHEEL_DIRECTION[0] * x + OMNI_WHEEL_DIRECTION[1] * y + rotation;
    int pwm = constrain((int)abs(speed), 0, 255);
    digitalWrite(WHEEL_PIN_DIRS[i], speed > 0.0 ? WHEEL_PINSTATE_FORWARDS[i] : !WHEEL_PINSTATE_FORWARDS[i]);
    analogWrite(WHEEL_PIN_PWMS[i], pwm < MINIMUM_REQUIRED_PWM ? 0 : pwm);
  }
}
void setSpeeds(double x, double y, double rotation) {
  setXspeed = x;
  setYspeed = y;
  setRspeed = rotation;
}
#define ISR_encoder(n) void ISR_encoder ## n() { \
  encoderTurn ## n += digitalRead(ENCODER_DT ## n) ? ENCODER_DIR ## n : -ENCODER_DIR ## n; \
}
#define attach_encoder(n) attachInterrupt(digitalPinToInterrupt(ENCODER_CLK ## n), ISR_encoder ## n, RISING)
#define pinmode_encoder(n) pinMode(ENCODER_CLK ## n, INPUT_PULLUP); pinMode(ENCODER_DT ## n, INPUT)
void wheelReset() {
  setXspeed = setYspeed = setRspeed = 0.0;
  motorXspeed = motorYspeed = motorRspeed = 0.0;
  setMotorSpeeds(0.0, 0.0, 0.0);
}
void omniWheelBegin() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(WHEEL_PIN_DIRS[i], OUTPUT);
    pinMode(WHEEL_PIN_PWMS[i], OUTPUT);
  }
}
double moveToward(double start, double target, double max) {
  double diff = target - start;
  if (abs(diff) < max) return target;
  return start + (diff < 0.0 ? -max : max);
}
void wheelUpdate(double delta) {
  if (currentStartupDuration < STARTUP_DURATION) {
    currentStartupDuration += delta;
    wheelReset();
    return;
  }

  double maxAcc = MAX_ACCELERATION * delta;
  motorXspeed = moveToward(motorXspeed, setXspeed, maxAcc);
  motorYspeed = moveToward(motorYspeed, setYspeed, maxAcc);
  motorRspeed = moveToward(motorRspeed, setRspeed, maxAcc);

  #ifndef TEST
  setMotorSpeeds(motorXspeed, motorYspeed, motorRspeed);
  #else
  #ifdef MOTOR_TEST
  static int wheelToRotate = 0;
  static double duration = 0.0;
  const double DURATION = 1.0;
  duration += delta;
  if (duration >= DURATION) {
    duration = 0.0;
    wheelToRotate += 1;
    if (wheelToRotate > 3) wheelToRotate = 0;
  }
  for (int i = 0; i < 3; i++) {
    digitalWrite(WHEEL_PIN_DIRS[i], WHEEL_PINSTATE_FORWARDS[i]);
    analogWrite(WHEEL_PIN_PWMS[i], wheelToRotate == i ? 100 : 0);
  }
  #endif
  #endif
}

#endif