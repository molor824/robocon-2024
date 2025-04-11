#pragma once

#include "vector3.h"

namespace Movement {
  const double MAX_ACCEL = 400.0;
  const double FAST_SPEED = 200.0;
  const double SLOW_SPEED = 100.0;
  const double ROTATION_MULTIPLIER = 0.6;

  Vector3 currentVelocity = {};
  Vector3 velocity = {};

  double omni0 = 0.0, omni1 = 0.0, omni2 = 0.0;

  void setOmniSpeed(Vector3 velocity) {
    const double SQRT_3_HALF = sqrt(3.0) * 0.5;

    double vx = velocity.x;
    double vy = velocity.y;
    double vr = velocity.z;

    omni0 = -vx + vr;
    omni1 = vx * 0.5 - vy * SQRT_3_HALF + vr;
    omni2 = vx * 0.5 + vy * SQRT_3_HALF + vr;
  }
  void brake() {
    currentVelocity = {};
    omni0 = omni1 = omni2 = 0.0;
  }
  void setVelocities(double directionX, double directionY, double rotation, bool fastMode) {
    double speed = fastMode ? FAST_SPEED : SLOW_SPEED;
    velocity = Vector3(
      directionX * speed,
      directionY * speed,
      rotation * speed * ROTATION_MULTIPLIER
    );
  }
  void loop(double delta) {
    double acceleration = MAX_ACCEL * delta;
    currentVelocity = currentVelocity.moveToward(velocity, acceleration);

    setOmniSpeed(currentVelocity);
  }
  void sendSerial() {
    int16_t omniSpeeds[3] = {
      constrain(round(omni0), -255, 255),
      constrain(round(omni1), -255, 255),
      constrain(round(omni2), -255, 255),
    };

    Serial2.write((uint8_t*)omniSpeeds, sizeof(omniSpeeds));
  }
};