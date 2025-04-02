#ifndef CYLINDERS_H
#define CYLINDERS_H

#include <Arduino.h>

const int EXTEND_PIN = 51;
const int THROW_PIN = 53;
const int CATCH_PIN = 49;
const double MAX_THROW = 0.5;

double throwDuration = 0.0;
bool cylinderExtended = false;
bool cylinderCaught = false;

void cylinderBegin() {
  pinMode(THROW_PIN, OUTPUT);
  pinMode(EXTEND_PIN, OUTPUT);
  pinMode(CATCH_PIN, OUTPUT);
  digitalWrite(THROW_PIN, LOW);
  digitalWrite(EXTEND_PIN, LOW);
  digitalWrite(CATCH_PIN, LOW);
}
void cylinderExtend() {
  if (cylinderExtended) {
    cylinderExtended = false;
    digitalWrite(EXTEND_PIN, LOW);
  } else {
    cylinderExtended = true;
    digitalWrite(EXTEND_PIN, HIGH);
  }
}
void cylinderCatch() {
  if (cylinderCaught) {
    cylinderCaught = false;
    digitalWrite(CATCH_PIN, LOW);
  } else {
    cylinderCaught = true;
    digitalWrite(CATCH_PIN, HIGH);
  }
}
void cylinderThrow() {
  if (throwDuration > 0.0) return;
  throwDuration = MAX_THROW;
}
void cylinderUpdate(double delta) {
  if (throwDuration <= 0.0) {
    digitalWrite(THROW_PIN, LOW);
  } else {
    digitalWrite(THROW_PIN, HIGH);
    throwDuration -= delta;
  }
}

#endif