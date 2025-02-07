#include "main.h"

#include <atomic>

#include "csv.hpp"
#include "drivetrain.hpp"
#include "lemlib-tarball/api.hpp"  // IWYU pragma: keep
#include "lemlib/api.hpp"          // IWYU pragma: keep
#include "pid.hpp"
#include "selector.hpp"

/// @brief Track width in mm.
const double TRACK_WIDTH = 357.0;

/// @brief Wheel circumference in mm.
const double WHEEL_CIRC = 262.0;

/// @brief Number of output gear teeth divided by number of input gear teeth.
const double GEAR_RATIO = 36.0 / 60.0;

/// @brief Left motors port numbers, with the negative sign to reverse them.
const std::initializer_list<std::int8_t> LEFT_MOTORS_PORT = {-1, -2};

/// @brief Right motors port numbers.
const std::initializer_list<std::int8_t> RIGHT_MOTORS_PORT = {3, 4};

/// @brief Intake motor port number.
const std::int8_t INTAKE_PORT = -6;

/// @brief Conveyor motor port number.
const std::int8_t CONVEYOR_PORT = -7;

/// @brief Ladybrown motor port number.
const std::int8_t LADYBROWN_PORT = 9;

/// @brief Port number for the vision sensor.
const uint8_t VISION_PORT = 8;

/// @brief Port number for the inertial sensor
const uint8_t INERTIAL_PORT = 13;

/// @brief Potentiometer three-wire port letter.
const std::int8_t POTENTIOMETER_PORT = 'a';

/// @brief Piston three-wire port letter.
const std::int8_t PISTON_PORT = 'b';

/// @brief The speed of the conveyor as a percentage of its max speed (200 rpm).
const double CONVEYOR_SPEED_PERCENT = 100;

/// @brief The speed of the conveyor when transporting a wrong color donut as a
/// percentage of its max speed (200 rpm).
const double CONVEYOR_TRANSPORT_SPEED_PERCENT = 75;

/// @brief The speed of the ladybrown as a percentage of its max speed (200
/// rpm).
const double LADYBROWN_SPEED_PERCENT = 75;

/// @brief Vision sensor signature ID for the red donut.
const uint32_t RED_SIG_ID = 1;

/// @brief Vision sensor signature ID for the blue donut.
const uint32_t BLUE_SIG_ID = 2;

/// @brief Distance in degrees for the conveyor to move in reverse for ejecting
/// a wrong color donut after the donut reaches the top of the conveyor.
const double CONVEYOR_EJECT_DISTANCE = 100;

/// @brief Minimum screen coverage for the donut to be detected using the vision
/// sensor.
const int MIN_SCREEN_COVERAGE = 160;

/// @brief The left edge limit for the vision sensor to consider a donut in
/// view. If the left edge of the donut is less than this limit, it is
/// considered leaving the screen.
const int LEFT_EDGE_LIMIT = 80;

/// @brief Gear ratio for the conveyor motor.
const double CONVEYOR_GEAR_RATIO = 12.0 / 18.0;

/// @brief Number of teeth in the conveyor output sprocket.
const int CONVEYOR_OUT_TEETH = 12;

/// @brief Number of links in the conveyor.
const int CONVEYOR_LINKS = 74;

/// @brief Offset in links between the first and second conveyor hooks.
const int CONVEYOR_HOOK_2_OFFSET_LINKS = 38;

/// @brief Number of motor degrees for the conveyor to make a full revolution.
const int CONVEYOR_REVOLUTION_DEGREES = double(CONVEYOR_LINKS) /
                                        double(CONVEYOR_OUT_TEETH) * 360.0 *
                                        CONVEYOR_GEAR_RATIO;

/// @brief Offset in degrees between the first and second conveyor hook.
const int CONVEYOR_HOOK_2_OFFSET_DEGREES =
    double(CONVEYOR_HOOK_2_OFFSET_LINKS) / double(CONVEYOR_OUT_TEETH) * 360.0 *
    CONVEYOR_GEAR_RATIO;

/// @brief Distance in conveyor motor degrees needed to rotate a hook from the
/// top of the conveyor to the vision sensor.
const int DISTANCE_VISION_TO_TOP = 360;

/// @brief The error tolerance for hook positioning during vision sensor
/// detection and transportation, in degrees of the conveyor motor.
const int CONVEYOR_TOLERANCE = 50;

/// @brief The distance in conveyor motor degrees for the conveyor to move
/// forward for a cooldown after a wrong color donut has been ejected.
const int CONVEYOR_COOLDOWN_DISTANCE = 200;

/// @brief Position of the ladybrown at the top of the conveyor for pickup, in
/// potentiometer units.
const int LADYBROWN_PICKUP_POSITION = 1880;

/// @brief Acceptable error in the ladybrown position.
const int LADYBROWN_EPSILON = 20;

/// @brief The proportional coefficient of the ladybrown PID controller.
const double LADYBROWN_KP = 0.05;

/// @brief The integral coefficient of the ladybrown PID controller.
const double LADYBROWN_KI = 0.0;

/// @brief The derivative coefficient of the ladybrown PID controller.
const double LADYBROWN_KD = 0.0;

/// @brief Enum for the colors of the donuts.
enum class DONUT_COLOR { RED, BLUE };

/// @brief Enum for motor spinning states.
enum class SPIN_STATE {
  /// @brief The motor is idle.
  STOP,
  /// @brief The motor is spinning forward.
  FORWARD,
  /// @brief The motor is spinning in reverse.
  REVERSE
};

/// @brief Enum for donut color sorting states.
enum class SORTING_STATE {
  /// @brief No donuts of the wrong color are detected.
  NOT_DETECTED,
  /// @brief A donut of the wrong color is detected.
  IN_VIEW,
  /// @brief A donut of the wrong color is moving from the vision sensor's
  /// detection range to the top of the conveyor.
  TRANSPORTING,
  /// @brief A donut of the wrong color is being ejected from the conveyor.
  EJECTING,
  /// @brief A donut is ejected from the conveyor, and the conveyor is moving
  /// forward for a cooldown distance.
  RECOVERING
};

// Initializing the controller, motors and sensors.
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors(LEFT_MOTORS_PORT);
pros::MotorGroup right_motors(RIGHT_MOTORS_PORT);
pros::Motor intake(INTAKE_PORT);
pros::Motor conveyor(CONVEYOR_PORT);
pros::Motor ladybrown(LADYBROWN_PORT);
pros::Vision vision_sensor(VISION_PORT);
pros::Imu imu(INERTIAL_PORT);
pros::adi::DigitalOut piston(PISTON_PORT);
pros::adi::AnalogIn potentiometer(POTENTIOMETER_PORT);

// The current time in milliseconds.
uint32_t now;

// Importing the tarball asset
ASSET(test_txt);  // '.' replaced with "_" to make c++ happy
// Create a decoder for the tarball
lemlib_tarball::Decoder decoder(test_txt);

/**
 * Spins the conveyor and intake motors at a specified speed for a given
 * duration.
 *
 * @param speed The speed at which to spin the motors, in RPM.
 * @param duration The time to spin the motors, in milliseconds.
 */
void motorSpin(int speed, int duration) {
  conveyor.move_velocity(speed) && intake.move_velocity(speed);
  pros::delay(duration);
  conveyor.move_velocity(0) && intake.move_velocity(0);
}

// Drivetrain settings.
lemlib::Drivetrain drivetrain(
    &left_motors,                // left motor group
    &right_motors,               // right motor group
    14.05,                       // 10 inch track width
    lemlib::Omniwheel::OLD_325,  // using old 3.25" omnis
    333.33,                      // drivetrain rpm is 333.33
    2  // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// Lateral motion controller.
lemlib::ControllerSettings linearController(
    10,   // proportional gain (kP)
    0,    // integral gain (kI)
    3,    // derivative gain (kD)
    3,    // anti windup
    1,    // small error range, in inches
    100,  // small error range timeout, in milliseconds
    3,    // large error range, in inches
    500,  // large error range timeout, in milliseconds
    20    // maximum acceleration (slew)
);

// Angular motion controller.
lemlib::ControllerSettings angularController(
    2,    // proportional gain (kP)
    0,    // integral gain (kI)
    10,   // derivative gain (kD)
    3,    // anti windup
    1,    // small error range, in degrees
    100,  // small error range timeout, in milliseconds
    3,    // large error range, in degrees
    500,  // large error range timeout, in milliseconds
    0     // maximum acceleration (slew)
);

// Sensors for odometry.
lemlib::OdomSensors sensors(
    nullptr,  // vertical tracking wheel, which we don't have
    nullptr,  // vertical tracking wheel 2, which we don't have
    nullptr,  // horizontal tracking wheel, which we don't have
    nullptr,  // horizontal tracking wheel 2, which we don't have
    &imu      // inertial sensor
);

// Input curve for throttle input during driver control.
lemlib::ExpoDriveCurve throttleCurve(
    3,     // joystick deadband out of 127
    10,    // minimum output where drivetrain will move out of 127
    1.019  // expo curve gain
);

// Input curve for steer input during driver control.
lemlib::ExpoDriveCurve steerCurve(
    3,     // joystick deadband out of 127
    10,    // minimum output where drivetrain will move out of 127
    1.019  // expo curve gain
);

// Create the chassis.
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

// CONVEYOR VARIABLES ----------------------------------------------------- //
// The state of the conveyor motor.
std::atomic<SPIN_STATE> conveyor_state = SPIN_STATE::STOP;
// The target position for the conveyor motor to stop at.
int conveyor_stop_target = 0;

// COLOR SORTING VARIABLES ------------------------------------------------ //
// The state of the donut sorting process.
std::atomic<SORTING_STATE> sorting_state = SORTING_STATE::NOT_DETECTED;
// The color of the donuts being scored, defaulting to red.
std::atomic<DONUT_COLOR> scoring_color = DONUT_COLOR::RED;
// The currently visible donut.
pros::vision_object_s_t visible_donut;
// Color signature for the red donut.
auto RED_SIG = pros::Vision::signature_from_utility(
    RED_SIG_ID, 3797, 11657, 7727, -2281, -1609, -1945, 1.5, 0);
// Color signature for the blue donut.
auto BLUE_SIG = pros::Vision::signature_from_utility(
    BLUE_SIG_ID, -4477, -3539, -4008, 51, 4449, 2250, 2.6, 0);

// TESTING VARIABLES -------------------------------------------------------- //
// Whether the robot is in testing mode.
std::atomic<bool> is_testing = false;
// Testing program selector.
Selector program_selector(controller,
                          {Key{"Program", {"Straight", "Turn"}, 0}});
// Straight program selector.
Selector straight_selector(controller,
                           {Key{"Distance",
                                {"12", "24", "36", "48", "60", "72", "-72",
                                 "-60", "-48", "-36", "-24", "-12"},
                                0}});
// Turn program selector.
Selector turn_selector(controller,
                       {Key{"Angle",
                            {"15", "30", "45", "90", "105", "120", "-120",
                             "-105", "-90", "-45", "-30", "-15"},
                            0}});

/**
 * Runs when the center button on the LCD emulator display is pressed.
 */
void on_center_button() { is_testing = !is_testing; }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();  // Initialize brain screen
  pros::lcd::register_btn1_cb(on_center_button);
  chassis.calibrate();  // Calibrate sensors

  now = pros::millis();

  // Thread to for brain screen and position logging
  pros::Task screenTask([&]() {
    while (true) {
      // Print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);          // X
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);          // Y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);  // Heading
      pros::lcd::print(3, "Conveyor: %d",
                       conveyor.get_raw_position(&now));  // Conveyor
      // Log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // Delay to save resources
      pros::delay(50);
    }
  });

  // Thread to for color sorting
  pros::Task sortingTask([&]() {
    while (true) {
      // Start the donut color sorting process if the conveyor is spinning.
      if (conveyor_state.load() == SPIN_STATE::FORWARD) {
        switch (sorting_state.load()) {
          case SORTING_STATE::NOT_DETECTED:
            // Check for donuts of the wrong color using the vision sensor.
            visible_donut = vision_sensor.get_by_sig(
                0, scoring_color.load() == DONUT_COLOR::RED ? BLUE_SIG_ID
                                                            : RED_SIG_ID);

            // Avoid false positive detections by checking if the donut cover
            // enough of the vision sensor's field of view.
            if (visible_donut.width >= MIN_SCREEN_COVERAGE) {
              // Slow the conveyor down to avoid uncertainty in hook ejecting
              // position.
              conveyor.move_velocity(2 * CONVEYOR_TRANSPORT_SPEED_PERCENT);

              // The donut is now in view.
              sorting_state = SORTING_STATE::IN_VIEW;
            }
            break;

          case SORTING_STATE::IN_VIEW:
            // Check for donuts of the wrong color using the vision sensor.
            visible_donut = vision_sensor.get_by_sig(
                0, scoring_color.load() == DONUT_COLOR::RED ? BLUE_SIG_ID
                                                            : RED_SIG_ID);

            // Check if the donut is leaving the vision sensor's field of view.
            if (visible_donut.left_coord > LEFT_EDGE_LIMIT &&
                visible_donut.left_coord < MIN_SCREEN_COVERAGE) {
              // The donut is now transporting to the top of the conveyor.
              sorting_state = SORTING_STATE::TRANSPORTING;

              int conveyor_position = conveyor.get_raw_position(&now);
              // // The number of revolutions the conveyor has made.
              // int conveyor_revs = conveyor_position /
              // CONVEYOR_REVOLUTION_DEGREES; The offset in degrees of the
              // conveyor motor from the last complete revolution.
              int conveyor_offset =
                  conveyor_position % CONVEYOR_REVOLUTION_DEGREES;

              // Correct for negative conveyor position.
              if (conveyor_offset < 0) {
                conveyor_offset += CONVEYOR_REVOLUTION_DEGREES;
              }

              // Set the target position for the conveyor to stop at based on
              // which hook the donut is on.
              if (conveyor_offset < CONVEYOR_TOLERANCE) {
                // The detected hook is the first hook and it has passed the
                // standard detection position.
                conveyor_stop_target = conveyor_position - conveyor_offset +
                                       DISTANCE_VISION_TO_TOP -
                                       CONVEYOR_TOLERANCE;
              } else if (conveyor_offset >
                         CONVEYOR_REVOLUTION_DEGREES - CONVEYOR_TOLERANCE) {
                // The detected hook is the first hook and it has not passed
                // the standard detection position.
                conveyor_stop_target = conveyor_position - conveyor_offset +
                                       CONVEYOR_REVOLUTION_DEGREES +
                                       DISTANCE_VISION_TO_TOP -
                                       CONVEYOR_TOLERANCE;
              } else {
                // The detected hook is the second hook.
                conveyor_stop_target = conveyor_position - conveyor_offset +
                                       CONVEYOR_HOOK_2_OFFSET_DEGREES +
                                       DISTANCE_VISION_TO_TOP -
                                       CONVEYOR_TOLERANCE;
              }
            }
            break;

          case SORTING_STATE::TRANSPORTING:
            // If the conveyor has reached the top, start ejecting the donut
            // by moving the conveyor in reverse.
            if (conveyor.get_raw_position(&now) >= conveyor_stop_target) {
              sorting_state = SORTING_STATE::EJECTING;
              conveyor.move_velocity(-2 * CONVEYOR_SPEED_PERCENT);
              // Offset the target position by the reversing distance.
              conveyor_stop_target -=
                  (CONVEYOR_EJECT_DISTANCE - 2 * CONVEYOR_TOLERANCE);
            }
            break;

          case SORTING_STATE::EJECTING:
            // If the conveyor has reversed to target position, reset the
            // sorting state and move the conveyor forward.
            if (conveyor.get_raw_position(&now) <= conveyor_stop_target) {
              sorting_state = SORTING_STATE::RECOVERING;
              conveyor.move_velocity(2 * CONVEYOR_SPEED_PERCENT);
              conveyor_stop_target +=
                  (CONVEYOR_COOLDOWN_DISTANCE - 2 * CONVEYOR_TOLERANCE);
            }
            break;

          case SORTING_STATE::RECOVERING:
            // If the conveyor has reversed to target position, reset the
            // sorting state and move the conveyor forward.
            if (conveyor.get_raw_position(&now) >= conveyor_stop_target) {
              sorting_state = SORTING_STATE::NOT_DETECTED;
              conveyor_stop_target = 0;
            }
            break;
        }
      }
      pros::delay(20);
    }
  });
}

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
  // Set the stopping point for Lady Brown
  while (potentiometer.get_value() > 700) {
    ladybrown.move_velocity(0);
  }
  // Set the brake mode for Lady Brown
  ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Setting pose manually
  chassis.setPose({-57, -14, 325});

  chassis.follow(decoder["path1"], 15, 2000, false);
  chassis.follow(decoder["path2"], 15, 2000);
  chassis.follow(decoder["path3"], 15, 2000);
  chassis.follow(decoder["path4"], 15, 2000, false);

  // Alliance wall stake
  motorSpin(200, 2000);

  // chassis.follow(test_txt, 15, 4000);
  //  Follow set path
  while (true) {
    lemlib::Pose pose = chassis.getPose();
    if (pose.x >= -30 && pose.y >= -18) {
      // TODO: Use PID to set to the angle
      // moveToAngle(23.94);
      intake.move_velocity(200) && conveyor.move_velocity(200);
      pros::delay(2000);
      intake.move_velocity(0) && conveyor.move_velocity(0);
    }
    // Update motors
    // Delay to save resources
    pros::delay(10);
  }
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
  // The number of main control loop iterations since the start of the program.
  int frame_counter = 0;

  // PISTON VARIABLES ------------------------------------------------------- //
  // Whether the piston is retracted.
  bool retracted = false;

  // LADYBROWN VARIABLES ---------------------------------------------------- //
  // Whether the ladybrown is currently snapping to pickup position at the top
  // of the conveyor.
  bool ladybrown_snapping = false;
  // PID controller for the ladybrown.
  PID ladybrown_pid(LADYBROWN_KP, LADYBROWN_KI, LADYBROWN_KD);

  // TESTING VARIABLES ------------------------------------------------------ //
  // Whether a testing program has been selected.
  bool has_selected_program = false;

  // INITIALIZATION --------------------------------------------------------- //
  // Set the color signatures for the vision sensor.
  vision_sensor.set_signature(RED_SIG_ID, &RED_SIG);
  vision_sensor.set_signature(BLUE_SIG_ID, &BLUE_SIG);

  // Set the brake mode for the ladybrown.
  ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  conveyor.tare_position();   // Reset the conveyor position.
  potentiometer.calibrate();  // Calibrate the potentiometer.

  // MAIN CONTROL LOOP ------------------------------------------------------ //
  while (true) {
    if (is_testing.load()) {
      if (!has_selected_program) {
        // Select a testing program on controller A button.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
          has_selected_program = true;

          pros::delay(200);

          // Skip this iteration.
          frame_counter++;
          pros::delay(50);
          continue;
        }
        
        program_selector.update();
        program_selector.display();
      } else {
        // Go back to the program selector on controller B button.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
          has_selected_program = false;

          pros::delay(200);

          // Skip this iteration.
          frame_counter++;
          pros::delay(50);
          continue;
        }

        switch (program_selector.get_current_key()->index) {
          case 0:
            // Run the straight program on controller A button.
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
              Key* straight_key = straight_selector.get_current_key();
              float distance =
                  std::stof(straight_key->values[straight_key->index]);
              chassis.setPose(0, 0, 0);
              chassis.moveToPoint(0, distance, 100000,
                                  {forwards : distance > 0}, false);

              pros::delay(200);
            }

            straight_selector.update();
            straight_selector.display();
            break;
          case 1:
            // Run the turn program on controller A button.
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
              Key* turn_key = turn_selector.get_current_key();
              chassis.setPose(0, 0, 0);
              chassis.turnToHeading(
                  std::stof(turn_key->values[turn_key->index]), 10000, {},
                  false);

              pros::delay(200);
            }

            turn_selector.update();
            turn_selector.display();
            break;
        }
      }

      frame_counter++;
      pros::delay(50);  // Run for 50 ms then update
      continue;
    }

    // Tank control scheme.
    left_motors.move(int(
        pow(double(controller.get_analog(ANALOG_LEFT_Y)) / 127, 3.0) * 127));
    right_motors.move(int(
        pow(double(controller.get_analog(ANALOG_RIGHT_Y)) / 127, 3.0) * 127));

    // Ladybrown control with the controller L buttons.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      ladybrown.move_velocity(2 * LADYBROWN_SPEED_PERCENT);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      ladybrown.move_velocity(-2 * LADYBROWN_SPEED_PERCENT);
    } else {
      ladybrown.move_velocity(0);
    }

    // Simultaneous conveyor and intake control with the controller R buttons.
    if (sorting_state.load() == SORTING_STATE::NOT_DETECTED) {
      if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move_velocity(2 * CONVEYOR_SPEED_PERCENT);
        conveyor.move_velocity(2 * CONVEYOR_SPEED_PERCENT);

        conveyor_state = SPIN_STATE::FORWARD;
      } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake.move_velocity(-2 * CONVEYOR_SPEED_PERCENT);
        conveyor.move_velocity(-2 * CONVEYOR_SPEED_PERCENT);

        conveyor_state = SPIN_STATE::REVERSE;
      } else {
        intake.move_velocity(0);
        conveyor.move_velocity(0);

        conveyor_state = SPIN_STATE::STOP;
      }
    }

    // Toggle piston state with the controller B button.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
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

    // Toggle donut scoring color with the controller A button.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      if (scoring_color == DONUT_COLOR::RED) {
        scoring_color = DONUT_COLOR::BLUE;
        pros::delay(200);
      } else {
        scoring_color = DONUT_COLOR::RED;
        pros::delay(200);
      }
    }

    // Ladybrown snapping with the controller X button.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      ladybrown_snapping = true;
      pros::delay(200);
    }

    // Ladybrown snapping with PID.
    if (ladybrown_snapping) {
      int ladybrown_error =
          LADYBROWN_PICKUP_POSITION - potentiometer.get_value_calibrated();

      // If the ladybrown is within the acceptable error, stop the motor.
      if (std::abs(ladybrown_error) < LADYBROWN_EPSILON) {
        ladybrown_snapping = false;
        ladybrown.move_velocity(0);
      } else {  // Otherwise, use PID for the ladybrown's velocity.
        ladybrown.move(ladybrown_pid.update(ladybrown_error));
      }
    }

    // Print the potentiometer value to the controller every 10 frames because
    // of the slow refresh rate of the controller screen.
    if (!(frame_counter % 10)) {
      // controller.print(0, 0, "Score: %s",
      //                  scoring_color == DONUT_COLOR::RED ? "RED " : "BLUE");
      controller.print(0, 0, "%d", potentiometer.get_value());
    }

    frame_counter++;
    pros::delay(20);  // Run for 20 ms then update
  }
}