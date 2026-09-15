#include "main.h"
#include "api.h" 
#include "main.cpp"

#pragma once
const float wheel_radius = 3.25; 

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

double kP = 0, kI = 0, kD = 0; 
void drivePID(int desired_inches)
{
    int integral = 0;
    double derivative;  
    double prev_pos; 
    double desired_degrees = inchesToDegrees(desired_inches, wheel_radius); 
    double curr_pos = (left_mg.get_position() + right_mg.get_position()) / 2;
    double desired_position = curr_pos + desired_degrees;

    double error = desired_position - curr_pos; 

    while (fabs(error) > 1)
    {
       error = desired_position - curr_pos; 
       integral += error; 

        if (integral > 5000)
        {
            integral = 0; 
        }

       double power = (error * kP) + (integral * kI) + (derivative * kD);     
       left_mg.move(power); 
       right_mg.move(power); 
    }
}