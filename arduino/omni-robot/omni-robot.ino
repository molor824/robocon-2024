const bool WHEEL_FORWARDS[] = { 1, 0, 0 };
const int WHEEL_PIN_DIRS[] = { 39, 37, 47 };
const int WHEEL_PIN_PWMS[] = { 2, 4, 3 };

void setMotorSpeed(int16_t speed, int index) {
  bool forward = WHEEL_FORWARDS[index];
  digitalWrite(WHEEL_PIN_DIRS[index], speed >= 0 ? forward : !forward);
  analogWrite(WHEEL_PIN_PWMS[index], constrain(abs(speed), 0, 255));
}
void setup() {
  Serial2.begin(500000);
}
void loop() {
  while (Serial.available() > 0) {
    int16_t omniSpeeds[3];
    Serial.readBytes((uint8_t*)omniSpeeds, sizeof(omniSpeeds));
    for (int i = 0; i < 3; i++) {
      setMotorSpeed(omniSpeeds[i], i);
    }
  }
}
