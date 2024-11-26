// rightFront, leftFront, leftBack, rightBack
const bool WHEEL_FORWARD[] = { 0, 1, 0, 0 };
const int WHEEL_DIRS[] = { 37, 45, 39, 47 };
const int WHEEL_PWMS[] = { 4, 5, 2, 3 };
const float CALIB_FACTORS[] = { 1.1, 1.0, 0.9, 1.0 };
const int GRAB_PINS[] = { 53, 51 }; // relay
const int GRAB_DIR = 43;
const int GRAB_PWM = 7;
const int GRAB_FORWARD = 0;
const int GRAB_SPEED = 60;
const int GRAB_DURATION = 1000;
const int THROW_PINS[] = { 49, 34 };  // relay
const int THROW_DIR = 41;
const int THROW_PWM = 6;
const int THROW_FORWARD = 0;

float directionX = 0, directionY = 0, rotation = 0;
bool keyGrab = false, keyThrow = false;
// const float GRAB_START_TIME = 0.1;
// const float GRAB_STOP_TIME = 1.0;
// const float MAX_GRAB_TIME = 1.2;
// float grabMotorTime = 0;
// float throwMotorTime = 0;
// uint32_t lastElapsed = 0;

// y > 0 - forward
// y < 0 - backward
// x > 0 - right
// x < 0 - left
float* setWheelSpeeds(float x, float y, float rotation, float speeds[4]) {
  speeds[0] = y - x - rotation;
  speeds[1] = y + x + rotation;
  speeds[2] = y - x + rotation;
  speeds[3] = y + x - rotation;
  // float maximum = speeds[0];
  // for (int i = 1; i < 4; i++)
  //   if (maximum < speeds[i]) maximum = speeds[i];
  // for (int i = 0; i < 4; i++)
  //   speeds[i] /= maximum;
  return speeds;
}
float absf(float a) {
  if (a < 0.0) return -a;
  return a;
}
void setMotorPins(float speed[4], float maxSpeed = 4) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(WHEEL_DIRS[i], speed[i] > 0.0f ? WHEEL_FORWARD[i] : !WHEEL_FORWARD[i]);
    analogWrite(WHEEL_PWMS[i], (int)absf(speed[i] / maxSpeed * 255.0f));
  }
}

enum CONTROLS : uint16_t {
  KEY_GRAB = 1,
  KEY_THROW = 2,
};
struct SerialIn {
  int16_t directionX;
  int16_t directionY;
  int16_t rotation;
  uint16_t controls;
};

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  Serial2.begin(115200);

  // lastElapsed = millis();
}

// #define TEST_MOTOR

void throwBall() {
  digitalWrite(THROW_PWM, HIGH);
  digitalWrite(THROW_DIR, THROW_FORWARD);
  delay(1000);
  for (int i = 0; i < 2; i++)
    digitalWrite(THROW_PINS[i], HIGH);
}
void stopThrowBall() {
  digitalWrite(THROW_PWM, LOW);
  for (int i = 0; i < 2; i++)
    digitalWrite(THROW_PINS[i], LOW);
}
void grabBall() {
  for (int i = 0; i < 2; i++)
    digitalWrite(GRAB_PINS[i], HIGH);
  delay(100);
  digitalWrite(GRAB_DIR, GRAB_FORWARD);
  analogWrite(GRAB_PWM, GRAB_SPEED);
  delay(GRAB_DURATION);
  analogWrite(GRAB_PWM, 0);
  for (int i = 0; i < 2; i++)
    digitalWrite(GRAB_PINS[i], LOW);
  delay(100);
  digitalWrite(GRAB_DIR, !GRAB_FORWARD);
  analogWrite(GRAB_PWM, GRAB_SPEED);
  delay(GRAB_DURATION);
  digitalWrite(GRAB_PWM, 0);
}
void loop() {
  // uint32_t deltaI = millis() - lastElapsed;
  // lastElapsed += deltaI;
  // float delta = (float)deltaI / 1000.0;

  while (Serial2.available() >= sizeof(SerialIn)) {
    SerialIn input;
    Serial2.readBytes((uint8_t*)&input, sizeof(SerialIn));
    directionX = (float)input.directionX / 512.0f;
    directionY = (float)input.directionY / -512.0f;
    rotation = (float)input.rotation / 512.0f;
    bool currentKeyGrab = (input.controls & KEY_GRAB) != 0;
    bool currentKeyThrow = (input.controls & KEY_THROW) != 0;

    SerialUSB.println(input.controls);

    if (!keyThrow && currentKeyThrow) {
      throwBall();
    } else if (keyThrow && !currentKeyThrow) {
      stopThrowBall();
    }
    if (!keyGrab && currentKeyGrab) {
      grabBall();
    }

    keyGrab = currentKeyGrab;
    keyThrow = currentKeyThrow;
  }

  // grab code (WIP)
  // if (keyGrab) {
  //   grabMotorTime += delta;
  // } else {
  //   grabMotorTime -= delta;
  // }
  // grabMotorTime = min(max(grabMotorTime, 0), MAX_GRAB_TIME);
  // bool inGrabTime = grabMotorTime >= GRAB_START_TIME && grabMotorTime <= GRAB_STOP_TIME;
  // analogWrite(GRAB_PWM, inGrabTime ? GRAB_SPEED : 0);
  // digitalWrite(GRAB_DIR, keyGrab ? GRAB_FORWARD : !GRAB_FORWARD);
  // for (int i = 0; i < 2; i++)
  //   digitalWrite(GRAB_PINS[i], inGrabTime && !keyGrab);
  
  #ifndef TEST_MOTOR
  float speeds[4];
  setMotorPins(setWheelSpeeds(directionX, directionY, rotation, speeds), 6);
  #else
  for (int i = 0; i < 4; i++) {
    float speeds[4] = {0, 0, 0, 0};
    speeds[i] = 1;
    setMotorPins(speeds, 4);
    delay(1000);
    speeds[i] = -1;
    setMotorPins(speeds, 4);
    delay(1000);
  }
  #endif
}
