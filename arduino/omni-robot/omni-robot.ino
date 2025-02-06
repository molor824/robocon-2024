const int WHEEL_COUNT = 4;

// following math radian rotation
// FrontRight, FrontLeft, BackLeft, BackRight
const bool WHEEL_FORWARD[WHEEL_COUNT] = { 0, 1, 1, 1 };
const int WHEEL_DIRS[WHEEL_COUNT] = { 39, 45, 37, 47 };
const int WHEEL_PWMS[WHEEL_COUNT] = { 2, 5, 4, 3 };

const float MAX_SPEED = 1.25;
const float SLOW_MAX_SPEED = 4;
bool fastMode = true;

const float OMNI_WHEEL_RADIANS[WHEEL_COUNT] = { PI * 0.25, PI * 0.75, PI * 1.25, PI * 1.75 };
const float OMNI_WHEEL_DIRECTIONS[WHEEL_COUNT][2] = {
  { -sinf(OMNI_WHEEL_RADIANS[0]), cosf(OMNI_WHEEL_RADIANS[0]) },
  { -sinf(OMNI_WHEEL_RADIANS[1]), cosf(OMNI_WHEEL_RADIANS[1]) },
  { -sinf(OMNI_WHEEL_RADIANS[2]), cosf(OMNI_WHEEL_RADIANS[2]) },
  { -sinf(OMNI_WHEEL_RADIANS[3]), cosf(OMNI_WHEEL_RADIANS[3]) },
};

// y > 0 - forward
// y < 0 - backward
// x > 0 - right
// x < 0 - left
float* setWheelSpeeds(float x, float y, float rotation, float speeds[WHEEL_COUNT]) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    const float *OMNI_WHEEL_DIRECTION = OMNI_WHEEL_DIRECTIONS[i];
    speeds[i] = OMNI_WHEEL_DIRECTION[0] * x + OMNI_WHEEL_DIRECTION[1] * y + rotation;
  }
}
float absf(float a) {
  if (a < 0.0) return -a;
  return a;
}
void setMotorPins(float speeds[WHEEL_COUNT], float maxSpeed = 4) {
  for (int i = 0; i < WHEEL_COUNT; i++) {
    digitalWrite(WHEEL_DIRS[i], speeds[i] > 0.0f ? WHEEL_FORWARD[i] : !WHEEL_FORWARD[i]);
    analogWrite(WHEEL_PWMS[i], (int)absf(speeds[i] / maxSpeed * 255.0f));
  }
}

enum CONTROLS : uint16_t {
  KEY_SPEED_CHANGE = 1,
};
struct SerialIn {
  int16_t directionX;
  int16_t directionY;
  int16_t rotation;
  uint16_t inputs;
};
// #define TEST_MOTOR
// #define TEST_GRAB
// #define TEST_THROW

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  Serial2.begin(115200);

  for (int i = 0; i < WHEEL_COUNT; i++) {
    pinMode(WHEEL_DIRS[i], OUTPUT);
    pinMode(WHEEL_PWMS[i], OUTPUT);
  }
}


void loop() {
  while (Serial2.available() >= sizeof(SerialIn)) {
    SerialIn input;
    Serial2.readBytes((uint8_t*)&input, sizeof(SerialIn));
    
    bool speedChange = (input.inputs & KEY_SPEED_CHANGE) != 0;
    float speeds[4];
    setWheelSpeeds((float)input.directionX / 512.0f, (float)input.directionY / -512.0f, (float)input.rotation / 512.0f, speeds);
    setMotorPins(speeds, speedChange ? MAX_SPEED : SLOW_MAX_SPEED);
  }
}
