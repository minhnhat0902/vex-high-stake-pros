#include "main.h"

#include "drivetrain.hpp"

/// @brief Track width in mm.
const double TRACK_WIDTH = 357.0;

/// @brief Wheel circumference in mm.
const double WHEEL_CIRC = 262.0;

/// @brief Number of output gear teeth divided by number of input gear teeth.
const double GEAR_RATIO = 36 / 60;

/// @brief Left motors port numbers, with the negative sign to reverse them.
const std::initializer_list<std::int8_t> LEFT_MOTORS_PORT = {-1, -2};

/// @brief Right motors port numbers.
const std::initializer_list<std::int8_t> RIGHT_MOTORS_PORT = {3, 4};

/// @brief Piston three-wire port letter.
const std::int8_t PISTON_PORT = 'a';

/// @brief Bumper three-wire port letter.
const std::int8_t BUMPER_PORT = 'b';

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() { pros::screen::set_eraser(0xffffff); }

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  Drivetrain drivetrain(LEFT_MOTORS_PORT, RIGHT_MOTORS_PORT, TRACK_WIDTH,
                        WHEEL_CIRC, GEAR_RATIO);

  drivetrain.move_pid(600);
  drivetrain.rotate_pid(90);
  drivetrain.move_pid(-600);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
}