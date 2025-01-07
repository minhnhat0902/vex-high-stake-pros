#include "main.h"


/// @brief Left motors port numbers, with the negative sign to reverse them.
const std::initializer_list<std::int8_t> LEFT_MOTORS_PORT = {-1, -2};

/// @brief Right motors port numbers.
const std::initializer_list<std::int8_t> RIGHT_MOTORS_PORT = {3, 4};

/// @brief Piston three-wire port letter.
const std::int8_t PISTON_PORT = 'a';

/// @brief Bumper three-wire port letter.
const std::int8_t BUMPER_PORT = 'b';

const std::int8_t CONVEYOR_PORT = 1;
const double CONVEYOR_SPEED_PERCENT = 75.0;

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
void autonomous() {}

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
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  pros::MotorGroup left_motors(LEFT_MOTORS_PORT);
  pros::MotorGroup right_motors(RIGHT_MOTORS_PORT);
  pros::Motor conveyor(CONVEYOR_PORT);
  pros::ADIDigitalOut piston(PISTON_PORT);
  pros::ADIDigitalIn bumper(BUMPER_PORT);

  bool retracted = false;

  while (true) {
    // Arcade control scheme
    left_motors.move(
        master.get_analog(ANALOG_LEFT_Y));  // Sets left motor voltage
    right_motors.move(
        master.get_analog(ANALOG_RIGHT_Y));  // Sets right motor voltage

    pros::screen::erase();
    pros::screen::set_pen(0x000000);

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      conveyor.move_velocity(2 * CONVEYOR_SPEED_PERCENT);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      conveyor.move_velocity(-2 * CONVEYOR_SPEED_PERCENT);
    } else {
      conveyor.move_velocity(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      if (retracted) {
        piston.set_value(0);
        retracted = false;
        pros::delay(200);
      } else {
        piston.set_value(1);
        retracted = true;
        pros::delay(200);
      }
    }

    if (bumper.get_value()) {
      pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Clicked!");
      // piston.set_value(false);
      // pros::delay(1000);
      // piston.set_value(true);
    } else {
      pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Not clicked!");
    }

    pros::delay(20);  // Run for 20 ms then update
  }
}