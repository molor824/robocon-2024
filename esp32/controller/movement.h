#include "HardwareSerial.h"
#pragma once

#include "vector2.h"

namespace Movement {
  const double MAX_VERTICAL_ACCEL = 400.0;
  const double MAX_HORIZONTAL_ACCEL = 200.0;
  const double MAX_ROTATION_ACCEL = 800.0;

  const double FAST_SPEED = 200.0;
  const double SLOW_SPEED = 100.0;
  const double ROTATION_SPEED = 40.0;

  Vector2 currentVelocity = {};
  double currentRotation = 0.0;
  Vector2 direction = {};
  double rotation = 0.0;

  double omni0 = 0.0, omni1 = 0.0, omni2 = 0.0;
  bool fastMode = false;
  bool brake = false;

  void setOmniSpeed(Vector2 velocity, double rotation) {
    const double SQRT_3_HALF = sqrt(3.0) * 0.5;

    double vx = velocity.x;
    double vy = velocity.y;
    double vr = rotation;

    omni0 = -vx + vr;
    omni1 = vx * 0.5 - vy * SQRT_3_HALF + vr;
    omni2 = vx * 0.5 + vy * SQRT_3_HALF + vr;
  }
  void setBrake(bool _brake) {
    brake = _brake;
  }
  void toggleFastMode() {
    fastMode = !fastMode;
  }
  void setDirections(double directionX, double directionY, double _rotation) {
    direction = Vector2(
      directionX,
      directionY
    );
    rotation = _rotation;
  }
  double moveToward(double current, double target, double max) {
    double diff = target - current;
    if (abs(diff) <= max) return target;
    return current + (diff < 0.0 ? -max : max);
  }
  void loop(double delta) {
    if (brake) {
      omni0 = omni1 = omni2 = 0.0;
      currentVelocity = {};
      currentRotation = 0.0;
      return;
    }
    double speed = fastMode ? FAST_SPEED : SLOW_SPEED;
    Vector2 targetVelocity = direction.mul(speed);
    double targetRotation = rotation * ROTATION_SPEED;

    currentRotation = moveToward(currentRotation, targetRotation, MAX_ROTATION_ACCEL * delta);
    currentVelocity.x = moveToward(currentVelocity.x, targetVelocity.x, MAX_HORIZONTAL_ACCEL * delta);
    currentVelocity.y = moveToward(currentVelocity.y, targetVelocity.y, MAX_VERTICAL_ACCEL * delta);
    // currentRotation -= (currentVelocity.x - prevVelocity.x) * COUNTER_ROTATION_MULTIPLIER;

    setOmniSpeed(currentVelocity, currentRotation);
  }
  void serial(int16_t speeds[3]) {
    speeds[0] = constrain(round(omni0), -255, 255);
    speeds[1] = constrain(round(omni1), -255, 255);
    speeds[2] = constrain(round(omni2), -255, 255);
  }
};