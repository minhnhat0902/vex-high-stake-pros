#include "drivetrain.hpp"

const double INV_360 = 1.0 / 360;
const double PI = 3.14159265358979324;

Drivetrain::Drivetrain(std::initializer_list<std::int8_t> left_motor_ports,
                       std::initializer_list<std::int8_t> right_motor_ports,
                       double track_width, double wheel_circ, double gear_ratio)
    : left_motors(left_motor_ports),
      right_motors(right_motor_ports),
      track_width(track_width),
      inv_wheel_circ(1 / wheel_circ),
      gear_ratio(gear_ratio) {}

double Drivetrain::average_left_position() {
  return (left_motors.get_position(0) + left_motors.get_position(1)) / 2;
}

double Drivetrain::average_right_position() {
  return (right_motors.get_position(0) + right_motors.get_position(1)) / 2;
}

void Drivetrain::move(double distance) {
  double spin_distance = distance * inv_wheel_circ * 360 * gear_ratio;
  left_motors.move_relative(spin_distance, 140);
  right_motors.move_relative(spin_distance, 140);

  pros::delay(3000);
}

void Drivetrain::rotate(double angle) {
  double spin_distance =
      angle * INV_360 * track_width * PI * inv_wheel_circ * gear_ratio;

  left_motors.move_relative(-spin_distance, 140);
  right_motors.move_relative(spin_distance, 140);

  pros::delay(3000);
}

void Drivetrain::move_pid(double distance) {
  left_motors.tare_position();
  right_motors.tare_position();

  const double set_point = distance * inv_wheel_circ * 360 * gear_ratio;

  double left_error = set_point - average_left_position();
  double right_error = set_point - average_right_position();
  double last_left_error = 0;
  double last_right_error = 0;
  double left_integral = 0;
  double right_integral = 0;
  double left_derivative = 0;
  double right_derivative = 0;

  while (abs(last_left_error) > Drivetrain::EPSILON ||
         abs(last_right_error) > Drivetrain::EPSILON) {
    left_motors.move_velocity(Drivetrain::KP * left_error +
                              Drivetrain::KI * left_integral +
                              Drivetrain::KD * left_derivative);
    right_motors.move_velocity(Drivetrain::KP * right_error +
                               Drivetrain::KI * right_integral +
                               Drivetrain::KD * right_derivative);

    pros::delay(20);

    last_left_error = left_error;
    last_right_error = right_error;
    left_error = set_point - average_left_position();
    right_error = set_point - average_right_position();
    left_integral += left_error;
    right_integral += right_error;
    left_derivative = (left_error - last_left_error) * 50;
    right_derivative = (right_error - last_right_error) * 50;
  }

  left_motors.move_velocity(0);
  right_motors.move_velocity(0);
}

void Drivetrain::rotate_pid(double angle) {
  left_motors.tare_position();
  right_motors.tare_position();

  const double set_point =
      angle * INV_360 * track_width * PI * inv_wheel_circ * gear_ratio;

  double left_error = average_left_position() - set_point;
  double right_error = set_point - average_right_position();
  double last_left_error = 0;
  double last_right_error = 0;
  double left_integral = 0;
  double right_integral = 0;
  double left_derivative = 0;
  double right_derivative = 0;

  while (abs(left_error) > Drivetrain::EPSILON ||
         abs(right_error) > Drivetrain::EPSILON) {
    left_motors.move_velocity(Drivetrain::KP * left_error +
                              Drivetrain::KI * left_integral +
                              Drivetrain::KD * left_derivative);
    right_motors.move_velocity(Drivetrain::KP * right_error +
                               Drivetrain::KI * right_integral +
                               Drivetrain::KD * right_derivative);

    pros::delay(20);

    last_left_error = left_error;
    last_right_error = right_error;
    left_error = average_left_position() - set_point;
    right_error = set_point - average_right_position();
    left_integral += left_error;
    right_integral += right_error;
    left_derivative = (left_error - last_left_error) * 50;
    right_derivative = (right_error - last_right_error) * 50;
  }

  left_motors.move_velocity(0);
  right_motors.move_velocity(0);
}

// FIXME: Position-based PID does not guarantee that the velocity ratio between
// the left and right motors is correct is maintained approximately constant
void Drivetrain::move_curve_pid(double curvature_radius, double arc_angle) {
  left_motors.tare_position();
  right_motors.tare_position();

  // TODO: Factor in the wheel base length in the radius calculation using the
  // Pythagorean theorem
  const double left_radius = curvature_radius - track_width / 2;
  const double right_radius = curvature_radius + track_width / 2;

  const double left_set_point =
      arc_angle * left_radius * PI * 2 * inv_wheel_circ * gear_ratio;
  const double right_set_point =
      arc_angle * right_radius * PI * 2 * inv_wheel_circ * gear_ratio;

  double left_error = left_set_point - average_left_position();
  double right_error = right_set_point - average_right_position();
  double last_left_error = 0;
  double last_right_error = 0;
  double left_integral = 0;
  double right_integral = 0;
  double left_derivative = 0;
  double right_derivative = 0;

  while (abs(left_error) > Drivetrain::EPSILON ||
         abs(right_error) > Drivetrain::EPSILON) {
    left_motors.move_velocity(Drivetrain::KP * left_error +
                              Drivetrain::KI * left_integral +
                              Drivetrain::KD * left_derivative);
    right_motors.move_velocity(Drivetrain::KP * right_error +
                               Drivetrain::KI * right_integral +
                               Drivetrain::KD * right_derivative);

    pros::delay(10);

    last_left_error = left_error;
    last_right_error = right_error;
    left_error = left_set_point - average_left_position();
    right_error = right_set_point - average_right_position();
    left_integral += left_error;
    right_integral += right_error;
    left_derivative = (left_error - last_left_error) * 100;
    right_derivative = (right_error - last_right_error) * 100;
  }

  left_motors.move_velocity(0);
  right_motors.move_velocity(0);
}