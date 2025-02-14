#include "omni_wheel.h"
#include "ball_drop.h"
#include "stepper.h"

const double MAX_SPEED = 1.25;
const double SLOW_MAX_SPEED = 4;

const double ROTATE_SPEED = 200.0;

enum CONTROLS : uint16_t {
  KEY_SPEED_CHANGE = 1 << 0,
  KEY_GRAB = 1 << 1,
  KEY_SEND = 1 << 2,
  KEY_ROTATE_LEFT = 1 << 3,
  KEY_ROTATE_RIGHT = 1 << 4
};

struct SerialIn {
  int16_t directionX;
  int16_t directionY;
  int16_t rotation;
  uint16_t inputs;
};

unsigned long previousElapsed;
void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  Serial2.begin(115200);

  omniWheelBegin();
  ballDropBegin();
  stepperBegin();

  previousElapsed = micros();
}

int previousInput = 0;

void loop() {
  unsigned long elapsed = micros();
  unsigned long iDelta = elapsed - previousElapsed;
  if (iDelta < 0) { // compensate for overflows
    iDelta = (0xFFFFFFFF - previousElapsed) + elapsed + 1;
  }
  previousElapsed = elapsed;

  double delta = (double)iDelta / 1000000.0;

  ballDropUpdate(delta);
  stepperUpdate(delta);
  wheelUpdate(delta);

  while (Serial2.available() >= sizeof(SerialIn)) {
    SerialIn input;
    Serial2.readBytes((uint8_t*)&input, sizeof(SerialIn));

    int inputs = input.inputs;
    
    bool speedChange = inputs & KEY_SPEED_CHANGE;
    bool grab = inputs & KEY_GRAB;
    bool send = inputs & KEY_SEND;
    bool rotateLeft = inputs & KEY_ROTATE_LEFT;
    bool rotateRight = inputs & KEY_ROTATE_RIGHT;

    double speedMultiplier = speedChange ? SLOW_MAX_SPEED : MAX_SPEED;
    double xSpeed = (double)input.directionX / 512.0 * speedMultiplier;
    double ySpeed = (double)input.directionY / 512.0 * speedMultiplier;
    double rSpeed = (double)input.rotation / 512.0 * speedMultiplier;

    setSpeeds(xSpeed, ySpeed, rSpeed);

    if (rotateLeft) stepSpeed = ROTATE_SPEED;
    else if (rotateRight) stepSpeed = -ROTATE_SPEED;

    bool previousGrab = previousInput & KEY_GRAB;
    bool previousSend = previousInput & KEY_SEND;

    if (grab && !previousGrab) tryGrab();
    if (send && !previousSend) trySend();

    previousInput = inputs;
  }
}
