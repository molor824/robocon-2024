const int EN_PIN = 8;
const int DIR_PIN = 5;
const int PULSE_PIN = 2;

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);

  SerialUSB.begin(115200);
}

const float MAX_SPEED = 200.0;
const float SENSITIVITY = 1.5;
const float ACCEL_SENSITIVITY = 4.0;

long lastElapsed = micros();

long targetStep = 200;
long nextTargetStep = 0;

long stepCount = 0;
float stepSpeed = 0.0;
float stepValue = 0.0;

float switchDuration = 3.0;
float switchValue = 0.0;

void stepperUpdate(float delta) {
  digitalWrite(DIR_PIN, stepSpeed >= 0.0);
  digitalWrite(PULSE_PIN, stepValue < 0.5);
  stepValue += delta * fabsf(stepSpeed);
  if (stepValue > 1.0) {
    stepCount += stepSpeed >= 0.0 ? 1 : -1;
    stepValue = 0.0;
  }
}
#define MIN(a, b) ((a) < (b) ? (a) : (b))
void loop() {
  long elapsed = micros();
  long lDelta = elapsed - lastElapsed;
  lastElapsed = elapsed;

  float delta = (float)lDelta / 1000000.0;

  long requiredStep = targetStep - stepCount;
  float requiredSpeed = (float)requiredStep * SENSITIVITY;
  float requiredAcceleration = (requiredSpeed - stepSpeed) * ACCEL_SENSITIVITY;
  stepSpeed += requiredAcceleration * delta;
  stepSpeed = MIN(stepSpeed, MAX_SPEED);

  stepperUpdate(delta);

  switchValue += delta;
  if (switchValue > switchDuration) {
    switchValue -= switchDuration;
    long temp = nextTargetStep;
    nextTargetStep = targetStep;
    targetStep = temp;
  }
}
