#include "main.h"

class Drivetrain {
  static constexpr double EPSILON = 0.1;

  static constexpr double KP = 0.100;
  static constexpr double KI = 0.000;
  static constexpr double KD = 0.000;

  const pros::MotorGroup left_motors;
  const pros::MotorGroup right_motors;
  const double track_width;
  const double inv_wheel_circ;
  const double gear_ratio;

  double average_left_position();
  double average_right_position();

 public:
  /// @brief Constructs a Drivetrain object.
  ///
  /// @param left_motor_ports A list of port numbers for the left side motors.
  /// Port numbers range from 1 to 21, inclusive, or from -21 to -1 for reversed
  /// motors.
  /// @param right_motor_ports A list of port numbers for the right side motors.
  /// Port numbers range from 1 to 21, inclusive, or from -21 to -1 for reversed
  /// motors.
  /// @param track_width The distance between the center of the left wheels to
  /// the center of the right wheels.
  /// @param wheel_circ The circumference of the wheels in mm.
  /// @param gear_ratio The gear ratio of the drivetrain; the number of output
  /// gear teeth divided by number of input gear teeth.
  ///
  /// # Example:
  ///
  /// ```
  /// Drivetrain drivetrain(
  ///   {-1, -2}, // Left motors are connected at port 1, 2 and reversed
  ///   {3, 4},   // Right motors are connected at port 3, 4
  ///   350,      // Track width is 350mm
  ///   260,      // Wheel circumference is 260mm
  ///   36 / 60   // Gear ratio is 36:60
  /// );
  /// ```
  Drivetrain(std::initializer_list<std::int8_t> left_motor_ports,
             std::initializer_list<std::int8_t> right_motor_ports,
             double track_width, double wheel_circ, double gear_ratio);

  /// @brief Moves the drivetrain with the given distance in mm.
  ///
  /// @param distance The distance to move in mm.
  void move(double distance);

  /// @brief Rotates the drivetrain with the given angle in degrees.
  ///
  /// @param angle The angle to rotate in degrees. A positive angle rotates the
  /// drivetrain counter-clockwise.
  void rotate(double angle);

  /// @brief Moves the robot with the given distance in mm using PID control.
  ///
  /// @param distance The distance to move in mm.
  void move_pid(double distance);

  /// @brief Rotates the robot with the given angle in degrees using PID
  /// control.
  ///
  /// @param angle The angle to rotate in degrees. A positive angle rotates the
  /// drivetrain counter-clockwise.
  void rotate_pid(double angle);
};
