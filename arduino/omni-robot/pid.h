#ifndef PID_H
#define PID_H

struct PID {
  double kp, ki, kd;
  double prevError;
  double integral;

  PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), prevError(0.0), integral(0.0) {}

  double compute(double delta, double setPoint, double measuredValue) {
    double error = setPoint - measuredValue;
    integral += error * delta;
    double derivative = (error - prevError) / delta;
    prevError = error;

    return kp * error + ki * integral + kd * derivative;
  }
};

#endif