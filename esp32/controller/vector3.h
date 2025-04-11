#pragma once

struct Vector3 {
  double x, y, z;
  Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
  double lengthSqr() const {
    return x * x + y * y + z * z;
  }
  double length() const {
    return sqrt(lengthSqr());
  }
  Vector3 add(Vector3 other) const {
    return Vector3(x + other.x, y + other.y, z + other.z);
  }
  Vector3 sub(Vector3 other) const {
    return Vector3(x - other.x, y - other.y, z - other.z);
  }
  Vector3 mul(double scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
  }
  Vector3 moveToward(Vector3 target, double max) const {
    Vector3 diff = target.sub(*this);
    double length = diff.length();
    if (length <= max) return target;
    return add(diff.mul(max / length));
  }
};