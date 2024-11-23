// rightFront, leftFront, leftBack, rightBack
const bool FORWARD_STATES[] = { 1, 1, 1, 1 };
const int DIRECTION_PINS[] = { 22, 24, 26, 28 };
const int PWM_PINS[] = { 2, 3, 4, 5 };
float directionX = 0, directionY = 0, rotation = 0;

// y > 0 - forward
// y < 0 - backward
// x > 0 - right
// x < 0 - left
float* setWheelSpeeds(float x, float y, float rotation, float speeds[4]) {
  speeds[0] = y - x + rotation;
  speeds[1] = y + x - rotation;
  speeds[2] = y - x - rotation;
  speeds[3] = y + x + rotation;
  return speeds;
}
void setMotorPins(float speed[4], float maxSpeed = 4) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(DIRECTION_PINS[i], speed[i] > 0 == FORWARD_STATES[i]);
    analogWrite(PWM_PINS[i], (int)(speed[i] / maxSpeed * 255));
  }
}

struct WheelSerialIn {
  int16_t directionX;
  int16_t directionY;
  int16_t rotation;
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  while (Serial1.available() >= sizeof(WheelSerialIn)) {
    WheelSerialIn input;
    Serial1.readBytes((uint8_t*)&input, sizeof(WheelSerialIn));
    directionX = (float)input.directionX / 512.0f;
    directionY = (float)input.directionY / 512.0f;
    rotation = (float)input.rotation / 512.0f;
    Serial.print(input.directionX);
    Serial.print(" ");
    Serial.print(input.directionY);
    Serial.print(" ");
    Serial.print(input.rotation);
    Serial.println();
  }
  float speeds[4];
  setMotorPins(setWheelSpeeds(directionX, directionY, rotation, speeds));
}
