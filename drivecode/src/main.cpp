#include "main.h"
#include "functions.h"
#include "motors.h"

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

	//Sets everything to zero in case I messed up
	left_mg.tare_position(); 
	right_mg.tare_position(); 

	inertial.reset(); 
	while (inertial.is_calibrating())
	{
		pros::delay(20);
	}
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
	//drivePID(distance, velocity, waitTime)
	//REMEMBER THAT ONE VEX TILE IS 24 INCHES! 
	turnPID(90, 1, 7000);  
	//driveAmount(-15, 100);
	//driveAmount(-150, 30);
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

	while (true) {


		int leftLog = 0;
		int rightLog = 0;
		//Gets speed from left and right joysticks
		int leftSpeed = master.get_analog(ANALOG_LEFT_Y);  
		int rightSpeed = master.get_analog(ANALOG_RIGHT_Y);

		pros::lcd::print(0, "%d", rightSpeed);

		leftSpeed < 0 ? leftLog = (leftSpeed * leftSpeed) / -127 : leftLog = (leftSpeed * leftSpeed) / 127;
		rightSpeed < 0 ? rightLog = (rightSpeed * rightSpeed) / -127 : rightLog = (rightSpeed * rightSpeed) / 127;

		if (fabs(leftLog) > 1)
		{
			left_mg.move(leftLog);
		}	
		else if (fabs(leftLog) < 1 && fabs(rightLog) > 1) //Otherwise we would brake at the same time. 
		{
			left_mg.move_velocity(0);
			left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else
		{
			left_mg.move_velocity(0);
			right_mg.move_velocity(0); 
			left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}

		if (fabs(rightLog) > 1)
		{
			right_mg.move(rightLog);
		}
		
		else if (fabs(rightLog) < 1 && fabs(leftLog) > 1)
		{
			right_mg.move_velocity(0); 
			right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		else
		{
			left_mg.move_velocity(0);
			right_mg.move_velocity(0);
			left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}
		
		pros::delay(20);                               // Run for 20 ms then update to prevent CPU Death
	}
}