#include "pid.h"

#ifndef OMNI_WHEEL_H
#define OMNI_WHEEL_H

#include <Arduino.h>

const int WHEEL_COUNT = 4;

// following math radian rotation
// FrontRight, FrontLeft, BackLeft, BackRight
const bool WHEEL_PINSTATE_FORWARDS[WHEEL_COUNT] = { 0, 1, 1, 1 };
const int WHEEL_PIN_DIRS[WHEEL_COUNT] = { 39, 45, 37, 47 };
const int WHEEL_PIN_PWMS[WHEEL_COUNT] = { 2, 5, 4, 3 };

// A, B, C encoders
// A - right side
// B - left side
// C - back side
const int ENCODER_L_CLK = 1;
const int ENCODER_R_CLK = 2;
const int ENCODER_F_CLK = 3;
const int ENCODER_L_DT = 4;
const int ENCODER_R_DT = 5;
const int ENCODER_F_DT = 6;
const int ENCODER_L_STATE = HIGH;
const int ENCODER_R_STATE = HIGH;
const int ENCODER_F_STATE = HIGH;

const double OMNI_WHEEL_RADIANS[WHEEL_COUNT] = { PI * 0.25, PI * 0.75, PI * 1.25, PI * 1.75 };
const double OMNI_WHEEL_DIRECTIONS[WHEEL_COUNT][2] = {
  { -sinf(OMNI_WHEEL_RADIANS[0]), cosf(OMNI_WHEEL_RADIANS[0]) },
  { -sinf(OMNI_WHEEL_RADIANS[1]), cosf(OMNI_WHEEL_RADIANS[1]) },
  { -sinf(OMNI_WHEEL_RADIANS[2]), cosf(OMNI_WHEEL_RADIANS[2]) },
  { -sinf(OMNI_WHEEL_RADIANS[3]), cosf(OMNI_WHEEL_RADIANS[3]) },
};

volatile int64_t encoderLturns = 0;
volatile int64_t encoderRturns = 0;
volatile int64_t encoderFturns = 0;

int64_t previousLturns = 0;
int64_t previousRturns = 0;
int64_t previousFturns = 0;

double setXspeed = 0.0;
double setYspeed = 0.0;
double setRspeed = 0.0;

double setXposition = 0.0;
double setYposition = 0.0;
double setRotation = 0.0;

double measuredXposition = 0.0;
double measuredYposition = 0.0;
double measuredRotation = 0.0;

PID pidX = PID(1.0, 0.005, 0.1);
PID pidY = PID(1.0, 0.005, 0.1);
PID pidRot = PID(1.0, 0.005, 0.1);

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
  encoderLturns += !digitalRead(ENCODER_L_DT) == !ENCODER_L_STATE ? 1 : -1;
}
void ISR_encoderR() {
  encoderRturns += !digitalRead(ENCODER_R_DT) == !ENCODER_R_STATE ? 1 : -1;
}
void ISR_encoderF() {
  encoderFturns += !digitalRead(ENCODER_F_DT) == !ENCODER_F_STATE ? 1 : -1;
}
void omniWheelBegin() {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(WHEEL_PIN_DIRS[i], OUTPUT);
    pinMode(WHEEL_PIN_PWMS[i], OUTPUT);
  }
  pinMode(ENCODER_L_CLK, OUTPUT);
  pinMode(ENCODER_R_CLK, OUTPUT);
  pinMode(ENCODER_F_CLK, OUTPUT);
  pinMode(ENCODER_L_DT, OUTPUT);
  pinMode(ENCODER_R_DT, OUTPUT);
  pinMode(ENCODER_F_DT, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_CLK), ISR_encoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_CLK), ISR_encoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_F_CLK), ISR_encoderF, RISING);
}
void wheelUpdate(double delta) {
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
  double dy = (dEr - dEl) / 2.0;
  double dx = -(dEf - drot);

  double cosdr = cos(drot);
  double sindr = sin(drot);

  measuredXposition += dx * cosdr - dy * sindr;
  measuredYposition += dx * sindr + dy * cosdr;
  measuredRotation += drot;

  setXposition += setXspeed * delta;
  setYposition += setYspeed * delta;
  setRotation += setRspeed * delta;

  double xVel = pidX.compute(delta, setXposition, measuredXposition);
  double yVel = pidY.compute(delta, setYposition, measuredYposition);
  double rotVel = pidRot.compute(delta, setRotation, measuredRotation);

  setMotorSpeeds(xVel, yVel, rotVel);
}

#endif