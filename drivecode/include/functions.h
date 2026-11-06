#include "main.h"
#include "motors.h"
#include "api.h"
#include "lemlib/api.hpp"

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

void driveAmount(double inches, double velocit, int waitTime)
{
    int degrees = inchesToDegrees(inches, aryans_constant); 
    //pros::lcd::print(0, "%d", degrees);
    
    left_mg.move_relative(degrees, velocit);
    right_mg.move_relative(degrees, velocit);

   double current = left_mg.get_position(); 
    double desired_position = left_mg.get_position() + degrees;

    double error = desired_position - left_mg.get_position();
    int currWait = 0; 
     while (fabs(error) > 2 && currWait < waitTime)
    {
        pros::lcd::print(1, "%f", error); 
        current = left_mg.get_position();
        error = desired_position - current; 
        currWait += 20; 
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
double max_power = 115;

void drivePID(double desired_inches, double velocita, int waitTime)
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

    //pros::lcd::print(3, "Degrees: %f", desired_degrees);
    while (fabs(error) > 1 && currTime < waitTime)
    {
        curr_pos = (left_mg.get_position() + right_mg.get_position()) / 2;
       // pros::lcd::print(6, "currTime: %i", currTime); 
        //pros::lcd::print(4, "Error: %f", error); 
        //pros::lcd::print(5, "Power: %f", power); 
        
       error = desired_position - curr_pos; 
       derivative = curr_pos - prev_pos; 

       power = ((error * kP) + (integral * kI) + (derivative * kD)) * velocita;  
       
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

        if (currTime > waitTime)
        {
            return; 
        }

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

void drivePIDUncapped(double desired_inches, double velocita, int waitTime)
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
       // pros::lcd::print(6, "currTime: %i", currTime); 
       // pros::lcd::print(4, "Error: %f", error); 
       // pros::lcd::print(5, "Power: %f", power); 
        
       error = desired_position - curr_pos; 
       derivative = curr_pos - prev_pos; 

       power = ((error * kP) + (integral * kI) + (derivative * kD)) * velocita;  
       
       //Ensures that we at least move with the min_power; at least 2/127 or whatever it is.

       if (fabs(power) > max_power)
       {
        power > 0 ? power = max_power : power = -max_power; 
       }
       if (fabs(power) < min_power)
       {
            power > 0 ? power = min_power: power = -min_power; 
       }
        left_mg.move(power);
        right_mg.move(power);

        if (currTime > waitTime)
        {
            return; 
        }

       prev_pos = curr_pos;
       currTime += 20; 

       pros::delay(20); 
    }
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
   

    return; 
}

double min_clamp_power = 8; 
void driveAndClampPID(double desired_inches, double distSlowDown, double clamp_error, double velocit, int waitTime)
{
    bool firstTime = true; 
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
        //pros::lcd::print(6, "Derivative : %f", derivative);
        //pros::lcd::print(4, "Error: %f", error);
        //pros::lcd::print(5, "Power: %f", power);

        error = desired_position - curr_pos;
        derivative = curr_pos - prev_pos;
        integral += error;

        if (integral > 5000)
        {
            integral = 0;
        }

        power = ((error * kP) + (integral * kI) + (derivative * kD)) * velocit;

        // Ensures that we at least move with the min_power; at least 2/127 or whatever it is.

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
            power > 0 ? power = min_power : power = -min_power;
        }

        //Since we're so close - start clamping
        if (fabs(error) < clamp_error && firstTime)
        {
            clamp_piston.set_value(true); 
            firstTime = false;
        }

        //Goes slow once we get real close
        if (fabs(error) < distSlowDown)
        {
            power > 0 ? power = min_clamp_power : power = -min_clamp_power; 
        }
        left_mg.move(power);
        right_mg.move(power);

        prev_pos = curr_pos;
        currTime += 20;
        slew += 8;

        pros::delay(20);
    }
    clamp_piston.set_value(false);
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    return;
}

void driveAndSlowPID(double desired_inches, double distSlowDown, double velocit, int waitTime)
{
    double min_move_power = 15; 
    bool firstTime = true;
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

    //pros::lcd::print(3, "Degrees: %f", desired_degrees);
    while (fabs(error) > 1 && currTime < waitTime)
    {
        curr_pos = (left_mg.get_position() + right_mg.get_position()) / 2;
        // pros::lcd::print(6, "Derivative : %f", derivative);
        // pros::lcd::print(4, "Error: %f", error);
        // pros::lcd::print(5, "Power: %f", power);

        error = desired_position - curr_pos;
        derivative = curr_pos - prev_pos;
        integral += error;

        if (integral > 5000)
        {
            integral = 0;
        }

        power = ((error * kP) + (integral * kI) + (derivative * kD)) * velocit;

        // Ensures that we at least move with the min_power; at least 2/127 or whatever it is.

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
            power > 0 ? power = min_power : power = -min_power;
        }


        // Goes slow once we get real close
        if (fabs(error) < distSlowDown)
        {
            power > 0 ? power = min_move_power : power = -min_move_power;
        }
        left_mg.move(power);
        right_mg.move(power);

        prev_pos = curr_pos;
        currTime += 20;
        slew += 8;

        pros::delay(20);
    }
    clamp_piston.set_value(true);
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    return;
}

double tKP = 1.1, tKI = 0, tKD = 0.35; 
double min_turn_power = 16; 
int turnPID(double desiredAngle, double velocit, int waitTime)
{
    double derivative = 0; 
    
    double integral = 0; 
    double power = 0; 
    int currWait = 0; 
    double currAngle = inertial.get_rotation();
    double prevAngle = inertial.get_rotation();

    double error = desiredAngle - currAngle; 
    double prevError = error; 

    double firstError = desiredAngle - currAngle; 

    double random = 90; 

    while (fabs(error) > 0.25 && currWait < waitTime)
    {
        //pros::lcd::print(6, "currWait: %i", currWait);
        currAngle = inertial.get_heading(); 
        error = desiredAngle - currAngle;

        //Readjusts error
        while (error > 180)
        {
            error -= 360;
            pros::delay(10);  
        }

        while (error < -180)
        {
            error += 360; 
            pros::delay(10); 
        }

        derivative =  prevError - error; 


        power = ((tKP * error) + (tKI * integral) + (tKD * derivative)) * velocit; 

        if (fabs(power) < min_turn_power)
        {
            power > 0 ? power = min_turn_power : power = -min_turn_power; 
        }

        left_mg.move(power); 
        right_mg.move(-power); 
    
        prevAngle = currAngle;
        prevError = error;

        pros::lcd::set_text(4, "Error: " + std::to_string(error));
        currWait += 20;
        pros::delay(20); 

    }

    left_mg.move_velocity(0);
    right_mg.move_velocity(0); 

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE); 

    return random; 
}

double newtKP = 2.7, newtKI = 0, newtKD = 0.75;
double newTurnMin = 38; 
int newTurnPID(double heading, double velocit, int waitTime)
{
    int currWait = 0;
    double error = heading - inertial.get_rotation();
    pros::lcd::set_text(2, "Original Error " + std::to_string(error));
    double derivative = 0;
    double prevError = 0;

    while (fabs(error) >= 0.1 && currWait <= waitTime)
    {
        // master.print(0, 0, "Theta: %f", Inertial.get_rotation());
        error = heading - inertial.get_rotation();
        double power = (error * newtKP) + (derivative * newtKD);

        if (fabs(power) < newTurnMin)
        {
            power >= 0 ? power = newTurnMin : power = -newTurnMin; 
        }
        left_mg.move_velocity(power * velocit);
        right_mg.move_velocity(-power * velocit);

        derivative = prevError - error;
        prevError = error;

        // Keeps robot from self-destructing
        currWait += 20;
        pros::delay(20);
        pros::lcd::set_text(1, "Current Error: " + std::to_string(error));
        pros::lcd::set_text(4, "Current Power: " + std::to_string(power));
    }

    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    

    return 0; 
}

void moveCascade(double desired_inches, double velocit, int waitTime)
{
    int currTime = 0;
    double slew = 5;
    double power = 0;
    int integral = 0;
    double derivative = 0;

    double desired_degrees = inchesToDegrees(desired_inches, aryans_constant);
    double curr_pos = (cascade_left.get_position() + cascade_right.get_position()) /2.0; 
    double prev_pos = curr_pos;

    double desired_position = curr_pos + desired_degrees;

    double error = desired_position - curr_pos;

    pros::lcd::print(3, "Degrees: %f", desired_degrees);
    while (fabs(error) > 1 && currTime < waitTime)
    {
        curr_pos = (cascade_left.get_position() + cascade_right.get_position());
        //pros::lcd::print(6, "Derivative : %f", derivative);
        //pros::lcd::print(4, "Error: %f", error);
        //pros::lcd::print(5, "Power: %f", power);

        error = desired_position - curr_pos;
        derivative = curr_pos - prev_pos;
        integral += error;

        if (integral > 5000)
        {
            integral = 0;
        }

        power = ((error * kP) + (integral * kI) + (derivative * kD)) * velocit;

        // Ensures that we at least move with the min_power; at least 2/127 or whatever it is.

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
            power > 0 ? power = min_power : power = -min_power;
        }
        cascade_left.move(power);
        cascade_right.move(power); 

        prev_pos = curr_pos;
        currTime += 20;
        slew += 8;

        pros::delay(20);
    }

    cascade_left.move_velocity(0); 
    cascade_right.move_velocity(0); 

    cascade_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    cascade_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  

    return;
}

void cascadeDown(int waitTime)
{
    int currTime = 0; 
    double cascade_position = cascade_left.get_position(); 

    while (currTime < waitTime && cascade_position > 2)
    {

    }
    return; 
}
    