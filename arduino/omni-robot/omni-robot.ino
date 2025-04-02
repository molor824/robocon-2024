#include <LibPrintf.h>
#include "omni_wheel.h"
#include "cylinders.h"

const double MAX_SPEED = 300.0;
const double SLOW_MAX_SPEED = 100.0;
const double ROTATION_MULTIPLIER = 0.7;

bool slowMode = true;

enum CONTROLS : uint16_t {
  KEY_SPEED_CHANGE = 1 << 0,
  KEY_EXTEND = 1 << 1,
  KEY_THROW = 1 << 2,
  KEY_CATCH = 1 << 3,
  KEY_RESET = 1 << 4,
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
  SerialUSB.begin(500000);
  Serial2.begin(500000);
  printf_init(SerialUSB);

  omniWheelBegin();
  cylinderBegin();

  previousElapsed = micros();
}

int prevInputs = 0;

void loop() {
  unsigned long elapsed = micros();
  unsigned long iDelta = elapsed - previousElapsed;
  if (iDelta < 0) { // compensate for overflows
    iDelta = (0xFFFFFFFF - previousElapsed) + elapsed + 1;
  }
  previousElapsed = elapsed;

  double delta = (double)iDelta / 1000000.0;

  wheelUpdate(delta);
  cylinderUpdate(delta);

  while (Serial2.available() >= sizeof(SerialIn)) {
    SerialIn input;
    Serial2.readBytes((uint8_t*)&input, sizeof(SerialIn));

    int inputs = input.inputs;

    bool extend = inputs & KEY_EXTEND;
    bool prevExtend = prevInputs & KEY_EXTEND;

    bool keyThrow = inputs & KEY_THROW;
    bool prevThrow = prevInputs & KEY_THROW;

    bool keyCatch = inputs & KEY_CATCH;
    bool prevCatch = prevInputs & KEY_CATCH;

    bool speedChange = inputs & KEY_SPEED_CHANGE;
    bool prevSpeedChange = prevInputs & KEY_SPEED_CHANGE;

    bool reset = inputs & KEY_RESET;
    bool prevReset = prevInputs & KEY_RESET;

    if (extend && !prevExtend) cylinderExtend();
    if (keyThrow && !prevThrow) cylinderThrow();
    if (keyCatch && !prevCatch) cylinderCatch();
    if (speedChange && !prevSpeedChange) slowMode = !slowMode;
    if (reset && !prevReset) wheelReset();
    prevInputs = inputs;

    double speedMultiplier = slowMode ? SLOW_MAX_SPEED : MAX_SPEED;
    double xSpeed = (double)constrain(input.directionX, -512, 512) / 512.0 * speedMultiplier;
    double ySpeed = (double)constrain(input.directionY, -512, 512) / -512.0 * speedMultiplier;
    double rSpeed = (double)constrain(input.rotation, -512, 512) / -512.0 * speedMultiplier * ROTATION_MULTIPLIER;

    setSpeeds(xSpeed, ySpeed, rSpeed);
  }
}
