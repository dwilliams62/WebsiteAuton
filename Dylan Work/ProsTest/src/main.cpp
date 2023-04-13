#include "main.h"
// #include "lemlib/api.hpp"

#include "okapi/api.hpp"
using namespace okapi;

pros::Controller Controller1(pros::E_CONTROLLER_MASTER);
pros::Motor lmotor1 (7, pros::E_MOTOR_GEAR_BLUE, false,pros:: E_MOTOR_ENCODER_DEGREES);
pros::Motor lmotor2 (8, pros::E_MOTOR_GEAR_BLUE, false,pros:: E_MOTOR_ENCODER_DEGREES);
pros::Motor lmotor3 (9, pros::E_MOTOR_GEAR_BLUE, false,pros:: E_MOTOR_ENCODER_DEGREES);
pros::Motor lmotor4 (10, pros::E_MOTOR_GEAR_BLUE, false,pros:: E_MOTOR_ENCODER_DEGREES);
pros::Motor rmotor1 (2, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rmotor2 (3, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rmotor3 (4, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rmotor4 (6, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftMotorGroup ({lmotor1, lmotor2, lmotor3, lmotor4});
pros::Motor_Group rightMotorGroup ({rmotor1, rmotor2, rmotor3, rmotor4});

// lemlib::Drivetrain_t drivetrain {
//     &leftMotorGroup, // left drivetrain motors
//     &rightMotorGroup, // right drivetrain motors
//     12, // track width
//     3.25, // wheel diameter
//     360 // wheel rpm
// };

okapi::MotorGroup leftMotors({7,8,9,10});
okapi::MotorGroup rightMotors({2,3,4,6});

std::shared_ptr<OdomChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors(rightMotors, leftMotors) // left motor is 1, right motor is 2 (reversed)
    // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{5.2_in, 12_in}, imev5GreenTPR})
	.withMaxVoltage(0.25)
	.withGains({0.0007, 0.0, 0.0}, {0.0007, 0.000, 0.0})
    // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
    .withOdometry()
    .buildOdometry();


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
	// set the state to zero
	chassis->setState({0_in, 0_in, 0_deg});

	//chassis->moveDistance({24_in});
	chassis->driveToPoint({-1_in, -25_in});
	// chassis->driveToPoint({-57_in, -1_in});
	// chassis->driveToPoint({-64_in, 23_in});
	// // turn approximately 45 degrees to end up at 90 degrees
	// chassis->turnToAngle(90_deg);
	// // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
	// chassis->turnToPoint({5_ft, 0_ft});
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

double controlCurve(double conP){
  //Slow curve
  return (exp(-14.6/10)+exp((fabs(conP)-100)/10)*(1-exp(-14.6/10)))*conP;
}

void opcontrol() {
	while (true) {
		double leftDrive = (Controller1.get_analog(ANALOG_RIGHT_Y) - Controller1.get_analog(ANALOG_RIGHT_X));
    	double rightDrive = (Controller1.get_analog(ANALOG_RIGHT_Y) + Controller1.get_analog(ANALOG_RIGHT_X));
		leftMotorGroup.move(controlCurve(leftDrive));
		rightMotorGroup.move(controlCurve(rightDrive));

		pros::delay(10);
	}
}
