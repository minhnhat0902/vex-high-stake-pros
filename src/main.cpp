#include "main.h"

#include "drivetrain.hpp"
#include "lemlib-tarball/api.hpp"  // IWYU pragma: keep
#include "lemlib/api.hpp"          // IWYU pragma: keep

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

/// @brief The speed of the ladybrown as a percentage of its max speed (200
/// rpm).
const double LADYBROWN_SPEED_PERCENT = 75;

/// @brief Vision sensor signature ID for the red donut.
const uint32_t RED_SIG_ID = 1;

/// @brief Vision sensor signature ID for the blue donut.
const uint32_t BLUE_SIG_ID = 2;

/// @brief Delay in ms for the conveyor to stop after a donut of the wrong color
/// is detected.
const uint32_t CONVEYOR_DELAY = 350;

/// @brief Delay in ms for the conveyor to start after ejecting a donut of the
/// wrong color.
const uint16_t CONVEYOR_HALT = 200;

/// @brief Distance in degrees for the conveyor to move after a donut of the
/// wrong color is detected.
const double CONVEYOR_STOP_DISTANCE = 770;

/// @brief Minimum screen coverage for the donut to be detected using the vision
/// sensor.
const int MIN_SCREEN_COVERAGE = 100;

/// @brief Gear ratio for the conveyor motor.
const double CONVEYOR_GEAR_RATIO = 12.0 / 18.0;

/// @brief Number of teeth in the conveyor output sprocket.
const int CONVEYOR_OUT_TEETH = 12;

/// @brief Number of links in the conveyor.
const int CONVEYOR_LINKS = 74;

/// @brief Offset in links between the first and second conveyor hooks.
const int CONVEYOR_HOOK_2_OFFSET = 31;

/// @brief Number of motor degrees for the conveyor to make a full revolution.
const double CONVEYOR_REVOLUTION_DEGREES = double(CONVEYOR_LINKS) /
                                           double(CONVEYOR_OUT_TEETH) * 360.0 *
                                           CONVEYOR_GEAR_RATIO;

/// @brief Offset in degrees between the first and second conveyor hook.
const double CONVEYOR_HOOK_2_OFFSET_DEGREES = double(CONVEYOR_HOOK_2_OFFSET) /
                                              double(CONVEYOR_OUT_TEETH) *
                                              360.0 * CONVEYOR_GEAR_RATIO;

/// @brief Distance in conveyor motor degrees between the vision sensor and the
/// top of the conveyor.
const double DISTANCE_VISION_TO_TOP = 520;

/// @brief Position of the ladybrown at the top of the conveyor for pickup, in
/// potentiometer units.
const int LADYBROWN_PICKUP_POSITION = 1060;

/// @brief Acceptable error in the ladybrown position.
const int LADYBROWN_EPSILON = 20;

/// @brief The proportional coefficient of the ladybrown PID controller.
const double LADYBROWN_K_P = 0.1;

/// @brief Enum for the colors of the donuts.
enum class DONUT_COLOR { RED, BLUE };

/// @brief Enum for motor spinning states.
enum class SPIN_STATE { STOP, FORWARD, REVERSE };

/// @brief Enum for donut color sorting states.
enum class SORTING_STATE { NOT_DETECTED, STANDBY, STOP };

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

// Drivetrain settings
lemlib::Drivetrain drivetrain(
    &left_motors,                // left motor group
    &right_motors,               // right motor group
    14.05,                       // 10 inch track width
    lemlib::Omniwheel::OLD_325,  // using old 3.25" omnis
    333.33,                      // drivetrain rpm is 333.33
    2  // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// Lateral motion controller
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

// Angular motion controller
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

// sensors for odometry
lemlib::OdomSensors sensors(
    nullptr,  // vertical tracking wheel, which we don't have
    nullptr,  // vertical tracking wheel 2, which we don't have
    nullptr,  // horizontal tracking wheel, which we don't have
    nullptr,  // horizontal tracking wheel 2, which we don't have
    &imu      // inertial sensor
);

// Input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(
    3,     // joystick deadband out of 127
    10,    // minimum output where drivetrain will move out of 127
    1.019  // expo curve gain
);

// Input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(
    3,     // joystick deadband out of 127
    10,    // minimum output where drivetrain will move out of 127
    1.019  // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();  // Initialize brain screen
  chassis.calibrate();      // Calibrate sensors

  // Thread to for brain screen and position logging
  pros::Task screenTask([&]() {
    while (true) {
      // Print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);          // X
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);          // Y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);  // Heading
      // Log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // Delay to save resources
      pros::delay(50);
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
  int frame_counter = 0;

  bool retracted = false;
  SORTING_STATE sorting_state = SORTING_STATE::NOT_DETECTED;
  SPIN_STATE conveyor_state = SPIN_STATE::STOP;

  auto RED_SIG = pros::Vision::signature_from_utility(
      RED_SIG_ID, 3797, 11657, 7727, -2281, -1609, -1945, 1.5, 0);
  auto BLUE_SIG = pros::Vision::signature_from_utility(
      BLUE_SIG_ID, -4477, -3539, -4008, 51, 4449, 2250, 2.6, 0);

  vision_sensor.set_signature(RED_SIG_ID, &RED_SIG);
  vision_sensor.set_signature(BLUE_SIG_ID, &BLUE_SIG);

  DONUT_COLOR scoring_color = DONUT_COLOR::BLUE;
  pros::vision_object_s_t visible_donut;
  double conveyor_stop_target = 0;

  bool ladybrown_snapping = false;

  conveyor.tare_position();
  ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  potentiometer.calibrate();

  while (true) {
    // Arcade control scheme
    left_motors.move(
        int(pow(double(controller.get_analog(ANALOG_LEFT_Y)) / 127, 3.0) *
            127));  // Sets left motor voltage
    right_motors.move(
        int(pow(double(controller.get_analog(ANALOG_RIGHT_Y)) / 127, 3.0) *
            127));  // Sets right motor voltage

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      ladybrown.move_velocity(2 * LADYBROWN_SPEED_PERCENT);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      ladybrown.move_velocity(-2 * LADYBROWN_SPEED_PERCENT);
    } else {
      ladybrown.move_velocity(0);
    }

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

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      if (scoring_color == DONUT_COLOR::RED) {
        scoring_color = DONUT_COLOR::BLUE;
        pros::delay(200);
      } else {
        scoring_color = DONUT_COLOR::RED;
        pros::delay(200);
      }
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      ladybrown_snapping = true;
      pros::delay(200);
    }

    if (conveyor_state == SPIN_STATE::FORWARD) {
      switch (sorting_state) {
        case SORTING_STATE::NOT_DETECTED:
          visible_donut = vision_sensor.get_by_sig(
              0, scoring_color == DONUT_COLOR::RED ? BLUE_SIG_ID : RED_SIG_ID);

          if (visible_donut.height >= MIN_SCREEN_COVERAGE) {
            sorting_state = SORTING_STATE::STANDBY;
            double conveyor_position = conveyor.get_position();
            int conveyor_revs =
                int(conveyor_position / CONVEYOR_REVOLUTION_DEGREES);
            double conveyor_offset =
                conveyor_position - conveyor_revs * CONVEYOR_REVOLUTION_DEGREES;
            double hook_offset = conveyor_offset + DISTANCE_VISION_TO_TOP;
            double closest_offset = std::abs(hook_offset);

            int detected_hook = 1;

            if (std::abs(hook_offset + CONVEYOR_HOOK_2_OFFSET_DEGREES) >
                closest_offset) {
              detected_hook = 2;
            }

            conveyor_stop_target =
                (conveyor_revs + 1) * CONVEYOR_REVOLUTION_DEGREES;

            if (detected_hook == 2) {
              conveyor_stop_target += CONVEYOR_HOOK_2_OFFSET_DEGREES;
            }
          }
          break;

        case SORTING_STATE::STANDBY:
          if (conveyor.get_position() >= conveyor_stop_target) {
            conveyor.move_velocity(0);
            sorting_state = SORTING_STATE::STOP;
            pros::delay(CONVEYOR_HALT);
          }
          break;

        default:
          conveyor.move_velocity(2 * CONVEYOR_SPEED_PERCENT);
      }
    } else {
      sorting_state = SORTING_STATE::NOT_DETECTED;
      conveyor_stop_target = 0;
    }

    if (ladybrown_snapping) {
      int ladybrown_error =
          potentiometer.get_value_calibrated() - LADYBROWN_PICKUP_POSITION;
      if (std::abs(ladybrown_error) < LADYBROWN_EPSILON) {
        ladybrown_snapping = false;
        ladybrown.move_velocity(0);
      } else {
        ladybrown.move_velocity(-2 * LADYBROWN_SPEED_PERCENT * ladybrown_error *
                                LADYBROWN_K_P);
      }
    }

    if (!(frame_counter % 10)) {
      // controller.print(0, 0, "Score: %s",
      //              scoring_color == DONUT_COLOR::RED ? "RED " : "BLUE");
      controller.print(0, 0, "%d", potentiometer.get_value());
    }

    frame_counter++;
    pros::delay(20);  // Run for 20 ms then update
  }
}