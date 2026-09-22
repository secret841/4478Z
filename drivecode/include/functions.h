#include "main.h"
#include "motors.h"
#include "api.h" 

#pragma once
const float wheel_radius = 3.25; 
const float aryans_constant = 1.270;

double degreesToInches(double degrees, double wheel_radius)
{
    double inches = degrees * ((2 * M_PI) / (360)) * wheel_radius;
    return inches; 
}

double inchesToDegrees(double inches, double wheel_radius)
{
    double degrees = inches * (360 / (2 * M_PI * wheel_radius)); 
    return degrees;
}

void driveAmount(double inches, double velocit)
{
    int degrees = inchesToDegrees(inches, aryans_constant); 
    pros::lcd::print(0, "%d", degrees);
    
    left_mg.move_relative(degrees, velocit);
    right_mg.move_relative(degrees, velocit);

   double current = left_mg.get_position(); 
    double desired_position = left_mg.get_position() + degrees;

    double error = desired_position - left_mg.get_position();
     while (fabs(error) > 2)
    {
        pros::lcd::print(1, "%f", error); 
        current = left_mg.get_position();
        error = desired_position - current; 
        pros::delay(20); 
    }
   
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    return; 
/*
    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);*/
}

double kP = 0.155, kI = 0, kD = 0.1; 
double min_power = 6; 
double max_power = 110;

void drivePID(double desired_inches, double velocit, int waitTime)
{
    int currTime = 0; 
    double slew = 5; 
    double power = 0; 
    int integral = 0;
    double derivative = 0;  
    
    double desired_degrees = inchesToDegrees(desired_inches, aryans_constant); 
    double curr_pos = (left_mg.get_position() + right_mg.get_position()) / 2;
    double prev_pos = curr_pos;

    double desired_position = curr_pos + desired_degrees;

    double error = desired_position - curr_pos; 

    pros::lcd::print(3, "Degrees: %f", desired_degrees);
    while (fabs(error) > 1 && currTime < waitTime)
    {
        curr_pos = (left_mg.get_position() + right_mg.get_position()) / 2;
        pros::lcd::print(6, "Derivative : %f", derivative); 
        pros::lcd::print(4, "Error: %f", error); 
        pros::lcd::print(5, "Power: %f", power); 
        
       error = desired_position - curr_pos; 
       derivative = curr_pos - prev_pos; 
       integral += error; 

        if (integral > 5000)
        {
            integral = 0; 
        }

       power = ((error * kP) + (integral * kI) + (derivative * kD)) * velocit;  
       
       //Ensures that we at least move with the min_power; at least 2/127 or whatever it is.

       if (fabs(power) > slew)
       {
         power > 0 ? power = slew : power = -slew; 
       }
       else if (fabs(power) > max_power)
       {
        power > 0 ? power = max_power : power = -max_power; 
       }
       if (fabs(power) < min_power)
       {
            power > 0 ? power = min_power: power = -min_power; 
       }
        left_mg.move(power);
        right_mg.move(power);

       prev_pos = curr_pos;
       currTime += 20; 
       slew += 8;

       pros::delay(20); 
    }
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
   

    return; 
}

double tKP = 0.75, tKI = 0, tKD = 0; 
void turnPID(double desiredAngle, double velocit, int waitTime)
{
    double derivative = 0; 
    double integral = 0; 
    double power = 0; 
    int currWait = 0; 
    double currAngle = inertial.get_rotation(); 

    //Bounds currentAngle at [0, 360]
    while (currAngle > 360)
    {
        currAngle -= 360; 
    }
    while (currAngle < 0)
    {
        currAngle += 360;
    }
    double error = desiredAngle - currAngle; 

    while (fabs(error) > 0.1 && currWait < waitTime)
    {
        pros::lcd::print(2, "error: %f", error); 
        pros::lcd::print(3, "power: %f", power); 
        currAngle = inertial.get_rotation(); 
        error = desiredAngle - currAngle;

        power = ((tKP * error) + (tKI * integral) + (tKD * derivative)) * velocit; 

        if (power > 0)
        {
            left_mg.move(power); 
            right_mg.move(-power); 
        }
        else
        {
            left_mg.move(-power); 
            right_mg.move(power); 
        }
        currWait += 20;
        pros::delay(20); 
    }

    left_mg.move_velocity(0);
    right_mg.move_velocity(0); 

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 
}