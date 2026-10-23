#include "main.h"
#include "functions.h"
#include "motors.h"
#include "lemlib/api.hpp"
#include "routes/vex/turnleft2pin.h"
#include "routes/recf/2pinrollerright.h"
#include "routes/recf/3pinright.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */



 int auton = 1; 
void on_center_button() {
	auton++; 
	if (auton == 1)
	{
		pros::lcd::print(4, "2 Pin Roller Right Side Selected");
	}
	else if (auton == 2)
	{
		pros::lcd::print(4, "3 Pin Right Side Selected"); 
	}
	else if (auton == 3)
	{
		pros::lcd::print(4, "Skills Auto (TURN RIGHT FIRST) Selected"); 
	}
	else
	{
		auton = 0; 
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
	pros::lcd::set_text(1, "Hello, World!");
	pros::lcd::register_btn1_cb(on_center_button);

	//Sets everything to zero in case I messed up
	left_mg.tare_position(); 
	right_mg.tare_position(); 

	cascade_left.tare_position();
	cascade_right.tare_position(); 

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
	//True makes it clamp. 
	
	//Turn Left First
	if (auton == 1)
	{
		twopinrollerright(); 
	}
	//Turn Right First
	else if (auton == 2)
	{
		threepinright(); 
	}
	//Skills Auton - MUST FLIP TO BE AUTON 2
	else if (auton == 3)
	{
		clamp_piston.set_value(true);
		drivePID(1, 1, 200);
		pros::delay(200);

		cascade_left.move_relative(250, 600);
		cascade_right.move_relative(250, 600);
		drivePID(8, 1, 500);
		driveAmount(-24, 600, 1000);
		drivePID(7.5, 1.3, 500);
		driveAmount(-24, 600, 1000);
		drivePID(18.5, 1, 1300);

		// Turns to first goal

		turnPID(90, 1, 1000);

		cascade_left.move_relative(550, 600);
		cascade_right.move_relative(550, 600);

		pros::lcd::print(7, "This worked!");
		drivePID(18, 1, 1000); // ERROR HERE!

		cascade_left.move_relative(-650, 600);
		cascade_right.move_relative(-650, 600);

		pros::delay(200);
		drivePID(-3, 1, 250); // Correction
		pros::delay(200);
		clamp_piston.set_value(false); // PLOP!

		drivePID(-14.3, 1, 1000);
		turnPID(36.5, 1, 1500); // Turn to next pin

		// 2nd number distslowdown
		driveAndClampPID(33.5, 850, 200, 0.75, 6500);
		cascade_left.move_relative(850, 600);
		cascade_right.move_relative(850, 600);

		drivePID(5, 1, 600);

		turnPID(182, 1, 1000);
		drivePID(25, 0.85, 1500);

		cascade_left.move_relative(-950, 600);
		cascade_right.move_relative(-950, 600);

		clamp_piston.set_value(false);
		drivePID(-7, 1, 500);

		clamp_piston.set_value(false);
		drivePID(-12, 1, 1200);
		turnPID(-45, 1, 1000); 
		drivePID(27, 1, 1800); 
	}
	
	//Clamp_error set in degrees
	//driveAndClampPID(10, 30, 1, 1000); 
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
	bool piston_val = true;
	while (true) {

	
		int leftLog = 0;
		int rightLog = 0;
		//Gets speed from left and right joysticks
		int leftSpeed = master.get_analog(ANALOG_LEFT_Y);  
		int rightSpeed = master.get_analog(ANALOG_RIGHT_Y);
		double cascade_pos = (cascade_left.get_position() + cascade_right.get_position()) / 2; 

		//pros::lcd::print(0, "%d", rightSpeed);
		//pros::lcd::print(1, "%d", piston_val);
		
		//pros::lcd::print(1, "%d", cascade_pos); 

	//Faster Acceleration Setting
	if (cascade_pos > 100)
	{
		//We are using x^3/127^2
		leftLog = (leftSpeed * leftSpeed * leftSpeed) / 16129;
		 rightLog = (rightSpeed * rightSpeed * rightSpeed) / 16129;

		if (fabs(leftSpeed) > 110)
		{
			leftSpeed < 0 ? leftSpeed = -110 : leftSpeed = 110; 
		}
		if (fabs(rightSpeed) > 110)
		{
			rightSpeed < 0 ? rightSpeed = -110 : rightSpeed = 110;
		}

		if (fabs(leftLog) > 0.5)
		{
			left_mg.move(leftLog);
		}
		else if (fabs(leftLog) < 0.5 && fabs(rightLog) > 0.5) // Otherwise we would brake at the same time.
		{
			left_mg.move_velocity(0);
			left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else
		{
			left_mg.move_velocity(0);
			right_mg.move_velocity(0);
			left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}

		if (fabs(rightLog) > 0.5)
		{
			right_mg.move(rightLog);
		}

		else if (fabs(rightLog) < 0.5 && fabs(leftLog) > 0.5)
		{
			right_mg.move_velocity(0);
			right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
		else
		{
			left_mg.move_velocity(0);
			right_mg.move_velocity(0);
			left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		}
	}
	else
	{
		leftSpeed < 0 ? leftLog = (leftSpeed * leftSpeed) / -127 : leftLog = (leftSpeed * leftSpeed) / 127;
		rightSpeed < 0 ? rightLog = (rightSpeed * rightSpeed) / -127 : rightLog = (rightSpeed * rightSpeed) / 127;

		if (fabs(leftLog) > 0.5)
		{
			left_mg.move(leftLog);
		}
		else if (fabs(leftLog) < 0.5 && fabs(rightLog) > 0.5) // Otherwise we would brake at the same time.
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

		if (fabs(rightLog) > 0.5)
		{
			right_mg.move(rightLog);
		}

		else if (fabs(rightLog) < 0.5 && fabs(leftLog) > 0.5)
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
	}
		
		
		if (master.get_digital(DIGITAL_R1))
		{
			cascade_left.move_velocity(600);
			cascade_right.move_velocity(600);
		}
		else if (master.get_digital(DIGITAL_R2) && cascade_pos > 0)
		{
			cascade_left.move_velocity(-600);
			cascade_right.move_velocity(-600);
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			cascade_left.move_velocity(-600);
			cascade_right.move_velocity(-600);
		}
		else
		{
			cascade_left.move_velocity(0);
			cascade_right.move_velocity(0);
			cascade_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			cascade_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		}

		if (master.get_digital_new_press(DIGITAL_A))
		{
			piston_val = !piston_val; 

			//Macro
			if (piston_val)
			{
				clamp_piston.set_value(true); 
				cascade_left.move_velocity(600);
				cascade_right.move_velocity(600);
				pros::delay(130);

				cascade_left.move_velocity(0);
				cascade_right.move_velocity(0);
				cascade_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
				cascade_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			}
		}
		if (piston_val)
		{
			clamp_piston.set_value(true);
		}
		else
		{
			clamp_piston.set_value(false); 
		}

		
		pros::delay(20);                               // Run for 20 ms then update to prevent CPU Death
	}
}