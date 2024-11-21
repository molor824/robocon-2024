// rightFront, leftFront, leftBack, rightBack
const bool FORWARD_STATES[] = { 1, 1, 1, 1 };
const int DIRECTION_PINS[] = { 22, 24, 26, 28 };
const int PWM_PINS[] = { 2, 3, 4, 5 };

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  float speeds[4];
  setWheelSpeeds(0.5, 0.5, 0, speeds);
  for (int i = 0; i < 4; i++) Serial.println(speeds[i]);
}

void loop() {
  // put your main code here, to run repeatedly:
  float speeds[4];
  setMotorPins(setWheelSpeeds(0, 1, 0, speeds), 2);
  delay(200);
  setMotorPins(setWheelSpeeds(1, 0, 0, speeds), 2);
  delay(200);
  setMotorPins(setWheelSpeeds(0, -1, 0, speeds), 2);
  delay(200);
  setMotorPins(setWheelSpeeds(-1, 0, 0, speeds), 2);
  delay(200);
}
