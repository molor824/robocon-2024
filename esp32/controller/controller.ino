#include <Bluepad32.h>

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
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
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

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(500000);
    Serial2.begin(500000);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    // BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);
}

enum CONTROLS : uint16_t {
  KEY_SPEED_CHANGE = 1 << 0,
  KEY_EXTEND = 1 << 1,
  KEY_THROW = 1 << 2,
  KEY_CATCH = 1 << 3,
  KEY_RESET = 1 << 4,
};
struct SerialOut {
  int16_t directionX;
  int16_t directionY;
  int16_t rotation;
  uint16_t inputs;
};

const int16_t CENTER_X = 4;
const int16_t CENTER_Y = 4;

// Arduino loop function. Runs in CPU 1.
void loop() {

    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool updated = BP32.update();
    if (updated) {
      for (auto controller : myControllers) {
        if (controller && controller->isConnected() && controller->hasData() && controller->isGamepad()) {
          uint8_t dpad = controller->dpad();
          SerialOut out;
          out.directionX = controller->axisX() - CENTER_X;
          out.directionY = controller->axisY() - CENTER_Y;
          out.rotation = controller->axisRX() - CENTER_X;
          out.inputs = (controller->r1() ? KEY_SPEED_CHANGE : 0)
            | (controller->b() ? KEY_EXTEND : 0)
            | (controller->a() ? KEY_THROW : 0)
            | (controller->x() ? KEY_CATCH : 0)
            | (controller->y() ? KEY_RESET : 0);
          Serial2.write((uint8_t*)&out, sizeof(SerialOut));
          break;
        }
      }
    }
}
