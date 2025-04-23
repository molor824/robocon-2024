#include <Bluepad32.h>
#include "movement.h"
#include "cylinders.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

const double TIMEOUT_DURATION = 0.5;
double inactiveDuration = 0.0;

unsigned long lastElapsed;

const int16_t CENTER_X = 0;
const int16_t CENTER_Y = 0;

bool keySpeedChange = false, keyBrake = false;
bool prevSpeedChange = false;
bool keyExtend = false, prevExtend = false;
bool keyCatch = false, prevCatch = true;
bool keyThrow = false, prevThrow = false;
bool fastMode = false;

double directionX = 0.0, directionY = 0.0, rotation = 0.0;

// Arduino setup function. Runs in CPU 1
void setup() {
  lastElapsed = micros();

  Serial.begin(115200);
  Serial2.begin(500000);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  BP32.enableVirtualDevice(false);
  delay(1);
}
void onInputUpdate() {
  if (keySpeedChange && !prevSpeedChange) Movement::toggleFastMode();
  Movement::setDirections(directionX, directionY, rotation);
  Movement::setBrake(keyBrake);

  if (keyExtend && !prevExtend) Cylinders::toggleExtend();
  if (keyCatch && !prevCatch) Cylinders::toggleCatch();
  if (keyThrow && !prevThrow) Cylinders::startThrow();

  prevSpeedChange = keySpeedChange;
  prevExtend = keyExtend;
  prevThrow = keyThrow;
  prevCatch = keyCatch;
}
struct SerialOutput {
  int16_t omni[3];
  uint8_t cylinder;
};
// Arduino loop function. Runs in CPU 1.
void loop() {
  unsigned long elapsed = micros();
  unsigned long iDelta = elapsed - lastElapsed;
  lastElapsed = elapsed;
  double delta = (double)iDelta / 1000000.0;

  inactiveDuration += delta;

  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool updated = BP32.update();
  if (updated) {
    for (auto controller : myControllers) {
      if (controller && controller->isConnected() && controller->hasData() && controller->isGamepad()) {
        uint8_t dpad = controller->dpad();

        directionX = (double)(controller->axisX() - CENTER_X) / 512.0;
        directionY = (double)(controller->axisY() - CENTER_Y) / -512.0;
        rotation = (double)(controller->axisRX() - CENTER_X) / -512.0;
        keySpeedChange = controller->r1();
        keyBrake = controller->l1();
        keyThrow = controller->a();
        keyExtend = controller->b();
        keyCatch = controller->x();
        inactiveDuration = 0.0;
        onInputUpdate();
        break;
      }
    }
  }
  if (inactiveDuration >= TIMEOUT_DURATION) {
    directionX = directionY = rotation = 0.0;
    keySpeedChange = keyBrake = false;
    onInputUpdate();
  }

  Movement::loop(delta);
  Cylinders::loop(delta);

  if (Serial2.available() > 0) {
    Serial2.read();

    SerialOutput out;
    Movement::serial(out.omni);
    out.cylinder = Cylinders::serial();

    Serial2.write((uint8_t*)&out, sizeof(SerialOutput));
  }
}
