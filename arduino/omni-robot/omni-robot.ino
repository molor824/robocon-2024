#include <LibPrintf.h>
#include "omni_wheel.h"
#include "cylinders.h"

const double MAX_SPEED = 400.0;
const double SLOW_MAX_SPEED = 300.0;

enum CONTROLS : uint16_t {
  KEY_SPEED_CHANGE = 1 << 0,
  KEY_EXTEND = 1 << 1,
  KEY_THROW = 1 << 2,
  KEY_CATCH = 1 << 3,
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
    
    bool speedChange = inputs & KEY_SPEED_CHANGE;

    double speedMultiplier = speedChange ? SLOW_MAX_SPEED : MAX_SPEED;
    double xSpeed = (double)input.directionX / 512.0 * speedMultiplier;
    double ySpeed = (double)input.directionY / -512.0 * speedMultiplier;
    double rSpeed = (double)input.rotation / 512.0 * speedMultiplier;

    setSpeeds(xSpeed, ySpeed, rSpeed);
    printf("x: %f, y: %f, r: %f, mul: %f\n", xSpeed, ySpeed, rSpeed, speedMultiplier);

    bool extend = inputs & KEY_EXTEND;
    bool prevExtend = prevInputs & KEY_EXTEND;

    bool keyThrow = inputs & KEY_THROW;
    bool prevThrow = prevInputs & KEY_THROW;

    bool keyCatch = inputs & KEY_CATCH;
    bool prevCatch = prevInputs & KEY_CATCH;

    if (extend && !prevExtend) cylinderExtend();
    if (keyThrow && !prevThrow) cylinderThrow();
    if (keyCatch && !prevCatch) cylinderCatch();

    prevInputs = inputs;
  }
}
