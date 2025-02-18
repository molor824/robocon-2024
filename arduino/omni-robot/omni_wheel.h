#include "pid.h"

#ifndef OMNI_WHEEL_H
#define OMNI_WHEEL_H

#include <Arduino.h>

const int WHEEL_COUNT = 4;

// #define TEST
// #define NOENCODER

// following math radian rotation
// FrontRight, FrontLeft, BackLeft, BackRight
const bool WHEEL_PINSTATE_FORWARDS[WHEEL_COUNT] = { 1, 0, 0, 0 };
const int WHEEL_PIN_DIRS[WHEEL_COUNT] = { 39, 45, 37, 47 };
const int WHEEL_PIN_PWMS[WHEEL_COUNT] = { 2, 5, 4, 3 };

// Left Right Front encoders
// L direction 0, -1
// R direction 0, 1
// F direction -1, 0
// encoder pin placement
// R - middle
// F - closer to arduino
// L - further to arduino
const int ENCODER_L_CLK = 19;
const int ENCODER_R_CLK = 20;
const int ENCODER_F_CLK = 21;
const int ENCODER_L_DT = 44;
const int ENCODER_R_DT = 42;
const int ENCODER_F_DT = 40;
const int ENCODER_L_TURN = -1;
const int ENCODER_R_TURN = -1;
const int ENCODER_F_TURN = -1;

const double OMNI_WHEEL_RADIANS[WHEEL_COUNT] = { PI * 0.25, PI * 0.75, PI * 1.25, PI * 1.75 };
const double OMNI_WHEEL_DIRECTIONS[WHEEL_COUNT][2] = {
  { cos(OMNI_WHEEL_RADIANS[0]), sin(OMNI_WHEEL_RADIANS[0]) },
  { cos(OMNI_WHEEL_RADIANS[1]), sin(OMNI_WHEEL_RADIANS[1]) },
  { cos(OMNI_WHEEL_RADIANS[2]), sin(OMNI_WHEEL_RADIANS[2]) },
  { cos(OMNI_WHEEL_RADIANS[3]), sin(OMNI_WHEEL_RADIANS[3]) },
};

volatile int64_t encoderLturns = 0;
volatile int64_t encoderRturns = 0;
volatile int64_t encoderFturns = 0;

double setXspeed = 0.0;
double setYspeed = 0.0;
double setRspeed = 0.0;

PID pidX = PID(1.0, 0.005, 0.1);

PID pidY = pidX;
PID pidRot = pidX;

// y > 0 - forward
// y < 0 - backward
// x > 0 - right
// x < 0 - left
void setMotorSpeeds(double x, double y, double rotation) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    const double *OMNI_WHEEL_DIRECTION = OMNI_WHEEL_DIRECTIONS[i];
    double speed = OMNI_WHEEL_DIRECTION[0] * x + OMNI_WHEEL_DIRECTION[1] * y + rotation;
    digitalWrite(WHEEL_PIN_DIRS[i], speed > 0.0 ? WHEEL_PINSTATE_FORWARDS[i] : !WHEEL_PINSTATE_FORWARDS[i]);
    analogWrite(WHEEL_PIN_PWMS[i], constrain(abs((int)speed), 0, 255));
  }
}
void setSpeeds(double x, double y, double rotation) {
  setXspeed = x;
  setYspeed = y;
  setRspeed = rotation;
}
void ISR_encoderL() {
  encoderLturns += digitalRead(ENCODER_L_DT) ? ENCODER_L_TURN : -ENCODER_L_TURN;
}
void ISR_encoderR() {
  encoderRturns += digitalRead(ENCODER_R_DT) ? ENCODER_R_TURN : -ENCODER_R_TURN;
}
void ISR_encoderF() {
  encoderFturns += digitalRead(ENCODER_F_DT) ? ENCODER_F_TURN : -ENCODER_F_TURN;
}
void omniWheelBegin() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(WHEEL_PIN_DIRS[i], OUTPUT);
    pinMode(WHEEL_PIN_PWMS[i], OUTPUT);
  }
  pinMode(ENCODER_L_CLK, INPUT_PULLUP);
  pinMode(ENCODER_R_CLK, INPUT_PULLUP);
  pinMode(ENCODER_F_CLK, INPUT_PULLUP);
  pinMode(ENCODER_L_DT, INPUT);
  pinMode(ENCODER_R_DT, INPUT);
  pinMode(ENCODER_F_DT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_CLK), ISR_encoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_CLK), ISR_encoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_F_CLK), ISR_encoderF, RISING);
}
void wheelUpdate(double delta) {
  static int64_t previousLturns = 0;
  static int64_t previousRturns = 0;
  static int64_t previousFturns = 0;
  
  static double motorXspeed = 0.0;
  static double motorYspeed = 0.0;
  static double motorRspeed = 0.0;

  #ifndef NOENCODER

  int64_t diffL = encoderLturns - previousLturns;
  int64_t diffR = encoderRturns - previousRturns;
  int64_t diffF = encoderFturns - previousFturns;

  previousLturns += diffL;
  previousRturns += diffR;
  previousFturns += diffF;

  double dEl = ((double)diffL) / delta;
  double dEr = ((double)diffR) / delta;
  double dEf = ((double)diffF) / delta;

  double drot = (dEr + dEl) / 2.0;
  double dy = ((dEr - drot) + (drot - dEl)) / 2.0;
  double dx = drot - dEf;

  double outX = pidX.compute(delta, setXspeed, dx);
  double outY = pidX.compute(delta, setYspeed, dy);
  double outR = pidX.compute(delta, setRspeed, drot);
  motorXspeed += outX * delta;
  motorYspeed += outY * delta;
  motorRspeed += outR * delta;

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
    if (wheelToRotate > 4) wheelToRotate = 0;
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(WHEEL_PIN_DIRS[i], WHEEL_PINSTATE_FORWARDS[i]);
    analogWrite(WHEEL_PIN_PWMS[i], wheelToRotate == i ? 100 : 0);
  }
  #endif
  printf(
    "delta %f encoder r %lld l %lld f %lld, speed dx %f dy %f drot %f, mspeed dx %f dy %f drot %f\n",
    delta,
    encoderRturns, encoderLturns, encoderFturns,
    dx, dy, drot, motorXspeed, motorYspeed, motorRspeed
  );
  #endif
}

#endif