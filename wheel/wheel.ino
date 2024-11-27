// rightFront, leftFront, leftBack, rightBack
const bool WHEEL_FORWARD[] = { 0, 1, 1, 1 };
const int WHEEL_DIRS[] = { 39, 45, 37, 47 };
const int WHEEL_PWMS[] = { 2, 5, 4, 3 };

const int GRAB_PIN = 53; // relay
const int GRAB_DIR = 43;
const int GRAB_PWM = 6;
const int GRAB_FORWARD = 0;
const int GRAB_SPEED = 80;
const float GRAB_MOTOR_START = 0.2;
const float MAX_GRAB_TIME = 1.0;
float grabMotorTime = 0;
bool grab = false;

const float HALF_LIFT_TIME = 2.5;
const float LIFT_MOTOR_END = 2.4;
const float LIFT_MOTOR_START = 0.1;
float liftMotorTime = HALF_LIFT_TIME * 2;

const int THROW_PIN = 51;  // relay
const int THROW_DIR = 41;
const int THROW_PWM = 7;
const int THROW_FORWARD = 0;
const int THROW_SPEED = 230;
const float MAX_THROW_TIME = 2.5;
const float THROW_RELAY_TIME = 2.0;
float throwMotorTime = 0;

const int ENCODER_A[] = {21, 20, 19, 18};
const int ENCODER_B[] = {38, 40, 42, 44};
const int ENCODER_Z[] = {46, 48, 50, 52};

uint32_t lastElapsed = 0;
float directionX = 0, directionY = 0, rotation = 0;
bool keyGrab = false, keyThrow = false, keyLift = false, keySpeed = false;

const float MAX_SPEED = 1.25;
const float SLOW_MAX_SPEED = 4;
bool fastMode = true;

// y > 0 - forward
// y < 0 - backward
// x > 0 - right
// x < 0 - left
float* setWheelSpeeds(float x, float y, float rotation, float speeds[4]) {
  speeds[0] = y + x - rotation;
  speeds[1] = y - x + rotation;
  speeds[2] = y + x + rotation;
  speeds[3] = y - x - rotation;
  float maximum = 1.0;
  for (int i = 0; i < 4; i++)
    maximum = max(maximum, speeds[i]);
  for (int i = 0; i < 4; i++)
    speeds[i] /= maximum;
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
  KEY_LIFT = 4,
  KEY_SPEED_CHANGE = 8,
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

  for (int i = 0; i < 4; i++) {
    pinMode(WHEEL_DIRS[i], OUTPUT);
    pinMode(WHEEL_PWMS[i], OUTPUT);
  }
  pinMode(THROW_PIN, OUTPUT);
  pinMode(GRAB_PIN, OUTPUT);
  pinMode(THROW_PWM, OUTPUT);
  pinMode(THROW_DIR, OUTPUT);
  pinMode(GRAB_PWM, OUTPUT);
  pinMode(GRAB_DIR, OUTPUT);

  digitalWrite(THROW_DIR, THROW_FORWARD);

  lastElapsed = millis();
}


void loop() {
  uint32_t deltaI = millis() - lastElapsed;
  lastElapsed += deltaI;
  float delta = (float)deltaI / 1000.0;

  while (Serial2.available() >= sizeof(SerialIn)) {
    SerialIn input;
    Serial2.readBytes((uint8_t*)&input, sizeof(SerialIn));
    directionX = (float)input.directionX / 512.0f;
    directionY = (float)input.directionY / -512.0f;
    rotation = (float)input.rotation / 512.0f;
    bool currentKeyGrab = (input.inputs & KEY_GRAB) != 0;
    bool currentKeyThrow = (input.inputs & KEY_THROW) != 0;
    bool currentKeyLift = (input.inputs & KEY_LIFT) != 0;
    bool currentKeySpeed = (input.inputs & KEY_SPEED_CHANGE) != 0;

    if (!keyGrab && currentKeyGrab) {
      grab = !grab;
    }
    if (!keyLift && currentKeyLift && liftMotorTime == HALF_LIFT_TIME * 2) {
      liftMotorTime = 0;
    }
    if (!keySpeed && currentKeySpeed) {
      fastMode = !fastMode;
    }

    keyGrab = currentKeyGrab;
    keyThrow = currentKeyThrow;
    keyLift = currentKeyLift;
    keySpeed = currentKeySpeed;
  }

  // if (grab) {
    liftMotorTime += delta;
    liftMotorTime = min(max(liftMotorTime, 0), HALF_LIFT_TIME * 2);
    digitalWrite(GRAB_PIN, liftMotorTime < HALF_LIFT_TIME && liftMotorTime > 0 ? HIGH : LOW);
    digitalWrite(GRAB_DIR, liftMotorTime <= HALF_LIFT_TIME ? GRAB_FORWARD : !GRAB_FORWARD);
    analogWrite(GRAB_PWM, (liftMotorTime >= LIFT_MOTOR_START && liftMotorTime < LIFT_MOTOR_END)
      || (liftMotorTime >= HALF_LIFT_TIME && liftMotorTime < (HALF_LIFT_TIME + LIFT_MOTOR_END)) ? GRAB_SPEED : 0);
  // }
  // if (liftMotorTime <= 0) {
  //   grabMotorTime += grab ? delta : -delta;
  //   grabMotorTime = min(max(grabMotorTime, 0), MAX_GRAB_TIME);
  //   digitalWrite(GRAB_PIN, grabMotorTime > 0);
  //   digitalWrite(GRAB_DIR, grab ? GRAB_FORWARD : !GRAB_FORWARD);
  //   analogWrite(GRAB_PWM, grabMotorTime >= GRAB_MOTOR_START && grabMotorTime < MAX_GRAB_TIME ? GRAB_SPEED : 0);
  // }
  
  throwMotorTime += keyThrow ? delta : -delta;
  throwMotorTime = min(max(throwMotorTime, 0), MAX_THROW_TIME);
  int throwMotorPWM = throwMotorTime > 0 && throwMotorTime < MAX_THROW_TIME && keyThrow ? THROW_SPEED : 0;
  bool throwRelay = throwMotorTime >= THROW_RELAY_TIME && keyThrow;
  analogWrite(THROW_PWM, throwMotorPWM);
  digitalWrite(THROW_PIN, throwRelay);

  float speeds[4];
  setMotorPins(setWheelSpeeds(directionX, directionY, rotation, speeds), fastMode ? MAX_SPEED : SLOW_MAX_SPEED);
}
