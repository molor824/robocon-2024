#ifndef BALL_DROP_H
#define BALL_DROP_H

#include <Arduino.h>

const double DROP_DURATION = 0.5;
const double GRAB_DURATION = 0.2;
const double SEND_DURATION = 0.4;

const int THROW_PIN_DIR = 4;
const int THROW_PIN_PWM = 5;
const int THROW_PINSTATE_DROP = HIGH;

double currentDuration = 0.0;
bool send = false;

enum {
  STATE_DROP,
  STATE_GRAB,
  STATE_SLEEP
} currentState = STATE_SLEEP;

bool tryGrab() {
  if (currentState == STATE_SLEEP) {
    currentState = STATE_DROP;
    currentDuration = 0.0;
    send = false;
    digitalWrite(THROW_PIN_DIR, THROW_PINSTATE_DROP);
    digitalWrite(THROW_PIN_PWM, HIGH);
    return true;
  }
  return false;
}
bool trySend() {
  if (currentState == STATE_SLEEP) {
    currentState = STATE_DROP;
    currentDuration = 0.0;
    send = true;
    digitalWrite(THROW_PIN_DIR, THROW_PINSTATE_DROP);
    digitalWrite(THROW_PIN_PWM, HIGH);
  }
  return false;
}
void ballDropUpdate(double delta) {
  if (currentState == STATE_SLEEP) return;

  currentDuration += delta;
  switch (currentState) {
    case STATE_DROP:
      if (currentDuration > DROP_DURATION) {
        currentDuration -= DROP_DURATION;
        currentState = STATE_GRAB;
        digitalWrite(THROW_PIN_DIR, !THROW_PINSTATE_DROP);
      }
      break;
    case STATE_GRAB:
      int duration = send ? SEND_DURATION : GRAB_DURATION;
      if (currentDuration > duration) {
        currentDuration = 0.0;
        currentState = STATE_SLEEP;
        digitalWrite(THROW_PIN_PWM, LOW);
      }
      break;
  }
}
void ballDropBegin() {
  pinMode(THROW_PIN_DIR, OUTPUT);
  pinMode(THROW_PIN_PWM, OUTPUT);
}

#endif