#include "pid.h"

#ifndef OMNI_WHEEL_H
#define OMNI_WHEEL_H

#include <Arduino.h>

const int WHEEL_COUNT = 3;

// #define TEST
// #define MOTOR_TEST
// #define NOENCODER

// assumes configuration where encoders are directed at
// 0 - 120*
// 1 - 240*
// 2 - 0*
// degrees
const int ENCODER_CLK0 = 19, ENCODER_CLK1 = 20, ENCODER_CLK2 = 21;
const int ENCODER_DT0 = 44, ENCODER_DT1 = 42, ENCODER_DT2 = 40;
const int ENCODER_DIR0 = -1, ENCODER_DIR1 = -1, ENCODER_DIR2 = -1;

const double MAX_ACCELERATION = 200.0;

// following math radian rotation
// FrontRight, FrontLeft, BackLeft, BackRight
const bool WHEEL_PINSTATE_FORWARDS[WHEEL_COUNT] = { 1, 0, 0 };
const int WHEEL_PIN_DIRS[WHEEL_COUNT] = { 39, 37, 47 };
const int WHEEL_PIN_PWMS[WHEEL_COUNT] = { 2, 4, 3 };

const double OMNI_WHEEL_RADIANS[WHEEL_COUNT] = { PI, PI + PI * 2.0 / 3.0, PI * 1.0 / 3.0 };
const double OMNI_WHEEL_DIRECTIONS[WHEEL_COUNT][2] = {
  { cos(OMNI_WHEEL_RADIANS[0]), sin(OMNI_WHEEL_RADIANS[0]) },
  { cos(OMNI_WHEEL_RADIANS[1]), sin(OMNI_WHEEL_RADIANS[1]) },
  { cos(OMNI_WHEEL_RADIANS[2]), sin(OMNI_WHEEL_RADIANS[2]) }
};

volatile int64_t encoderTurn0 = 0, encoderTurn1 = 0, encoderTurn2 = 0;
int64_t prevTurn0 = 0, prevTurn1 = 0, prevTurn2 = 0;

double setXspeed = 0.0;
double setYspeed = 0.0;
double setRspeed = 0.0;

double motorXspeed = 0.0;
double motorYspeed = 0.0;
double motorRspeed = 0.0;

PID pidX = PID(0.4, 0.0, 0.01);
PID pidY = pidX;
PID pidR = pidX;

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
ISR_encoder(0)
ISR_encoder(1)
ISR_encoder(2)
void wheelReset() {
  encoderTurn0 = encoderTurn1 = encoderTurn2 = 0;
  prevTurn0 = prevTurn1 = prevTurn2 = 0;
  setXspeed = setYspeed = setRspeed = 0.0;
  motorXspeed = motorYspeed = motorRspeed = 0.0;
}
void omniWheelBegin() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(WHEEL_PIN_DIRS[i], OUTPUT);
    pinMode(WHEEL_PIN_PWMS[i], OUTPUT);
  }
  pinmode_encoder(0);
  pinmode_encoder(1);
  pinmode_encoder(2);

  attach_encoder(0);
  attach_encoder(1);
  attach_encoder(2);
}
void wheelUpdate(double delta) {
  if (currentStartupDuration < STARTUP_DURATION) {
    currentStartupDuration += delta;
    wheelReset();
    return;
  }
  // if (elapsedDuration < FIXED_DELTA) {
  //   elapsedDuration += delta;
  //   return;
  // }
  // delta = elapsedDuration;
  // elapsedDuration -= FIXED_DELTA;

  #ifndef NOENCODER

  int64_t diff0 = encoderTurn0 - prevTurn0;
  int64_t diff1 = encoderTurn1 - prevTurn1;
  int64_t diff2 = encoderTurn2 - prevTurn2;
  prevTurn0 += diff0;
  prevTurn1 += diff1;
  prevTurn2 += diff2;

  double dt0 = (double)diff0;
  double dt1 = (double)diff1;
  double dt2 = (double)diff2;

  double dx = (2.0 * dt2 - dt0 - dt1) / 3.0;
  double dy = (dt0 - dt1) / sqrt(3.0);
  double dr = (dt0 + dt1 + dt2) / 3.0;

  double vx = dx / delta;
  double vy = dy / delta;
  double vr = dr / delta;

  double ax = constrain(pidX.compute(delta, setXspeed, vx), -MAX_ACCELERATION, MAX_ACCELERATION);
  double ay = constrain(pidY.compute(delta, setYspeed, vy), -MAX_ACCELERATION, MAX_ACCELERATION);
  double ar = constrain(pidR.compute(delta, setRspeed, vr), -MAX_ACCELERATION, MAX_ACCELERATION);

  motorXspeed = constrain(motorXspeed + ax * delta, -256.0, 256.0);
  motorYspeed = constrain(motorYspeed + ay * delta, -256.0, 256.0);
  motorRspeed = constrain(motorRspeed + ar * delta, -256.0, 256.0);

  #else
  motorXspeed = setXspeed;
  motorYspeed = setYspeed;
  motorRspeed = setRspeed;
  #endif

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
  #ifndef NOENCODER
  printf(
    "set speed x %4.0f y %4.0f r %4.0f, speed x %4.0f y %4.0f rot %4.0f, mspeed x %7.2f y %7.2f rot %7.2f\n",
    setXspeed * delta, setYspeed * delta, setRspeed * delta,
    dx, dy, dr, motorXspeed, motorYspeed, motorRspeed
  );
  #endif
  #endif
}

#endif