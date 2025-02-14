#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

const int STEPS_PER_REV = 200;
const int STEPPER_PIN_DIR = 2;
const int STEPPER_PIN_STEP = 5;
const int STEPPER_PINSTATE_FORWARD = HIGH;

double stepDuration = 0.0;
double stepSpeed = 200.0;

void stepperBegin() {
  pinMode(STEPPER_PIN_DIR, OUTPUT);
  pinMode(STEPPER_PIN_STEP, OUTPUT);
}

void stepperUpdate(double delta) {
  stepDuration += delta * stepSpeed;
  if (stepDuration > 1.0) stepDuration -= 1.0;
  digitalWrite(STEPPER_PIN_DIR, stepSpeed > 0.0 ? STEPPER_PINSTATE_FORWARD : !STEPPER_PINSTATE_FORWARD);
  digitalWrite(STEPPER_PIN_STEP, stepDuration > 0.5);
}

#endif