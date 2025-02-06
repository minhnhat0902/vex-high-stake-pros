#include "pid.hpp"

PID::PID(double kP, double kI, double kD, double epsilon)
    : KP(kP), KI(kI), KD(kD), EPSILON(epsilon) {}

double PID::update(double error) {
  // Reset the integral if the sign of the error changes.
  if ((error < 0) != (last_error < 0)) {
    integral = 0;
  } else {
    integral += error;
  }

  double derivative = error - last_error;
  last_error = error;
  return KP * error + KI * integral + KD * derivative;
}
