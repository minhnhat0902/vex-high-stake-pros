/// @file pid.hpp
///
/// @brief Contains the PID class for autonomous control.

#include "main.h"

class PID {
  /// @brief The maximum error tolerance of the PID controller.
  const double EPSILON;

  /// @brief The proportional coefficient of the PID controller.
  const double KP;

  /// @brief The integral coefficient of the PID controller.
  const double KI;

  /// @brief The derivative coefficient of the PID controller.
  const double KD;

  /// @brief The integral of the PID controller.
  double integral = 0;

  /// @brief The last error of the PID controller.
  double last_error = 0;

 public:
  /// @brief Initializes a PID controller.
  ///
  /// @param kP The proportional coefficient.
  /// @param kI The integral coefficient.
  /// @param kD The derivative coefficient.
  /// @param epsilon The maximum error tolerance.
  ///
  /// # Example:
  ///
  /// ```
  /// PID pid_motor(
  ///   0.1, // kP
  ///   0.01, // kI
  ///   20, // kD
  ///   0.1, // epsilon
  /// );
  /// ```
  PID(double kP, double kI, double kD, double epsilon);

  /// @brief Updates the PID controller.
  ///
  /// @param error The error of the PID controller.
  /// @return The output of the PID controller.
  double update(double error);
};