#pragma once

struct Vector2 {
  double x, y;
  Vector2() : x(0.0), y(0.0) {}
  Vector2(double x, double y) : x(x), y(y) {}
  double lengthSqr() const {
    return x * x + y * y;
  }
  double length() const {
    return sqrt(lengthSqr());
  }
  Vector2 add(Vector2 other) const {
    return Vector2(x + other.x, y + other.y);
  }
  Vector2 sub(Vector2 other) const {
    return Vector2(x - other.x, y - other.y);
  }
  Vector2 mul(double scalar) const {
    return Vector2(x * scalar, y * scalar);
  }
  Vector2 moveToward(Vector2 target, double max) const {
    Vector2 diff = target.sub(*this);
    double length = diff.length();
    if (length <= max) return target;
    return add(diff.mul(max / length));
  }
  bool almostZero(double difference = 0.000001) const {
    return abs(x) <= difference && abs(y) <= difference;
  }
};