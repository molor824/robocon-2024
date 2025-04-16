#pragma once

namespace Cylinders {
  const double MAX_THROW_DURATION = 0.5;

  double throwDuration = 0.0;

  bool extendPin = false;
  bool throwPin = false;
  bool catchPin = false;

  void toggleExtend() {
    extendPin = !extendPin;
  }
  void toggleCatch() {
    catchPin = !catchPin;
  }
  void startThrow() {
    throwDuration = MAX_THROW_DURATION;
  }
  void loop(double delta) {
    throwPin = throwDuration > 0.0;
    if (throwDuration > 0) throwDuration -= delta;
  }
  uint8_t serial() {
    return (extendPin ? 1 : 0) | (throwPin ? 1 << 1 : 0) | (catchPin ? 1 << 2 : 0);
  }
}