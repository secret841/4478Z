#include "main.h"
#include "motors.h"
#include "api.h"
#include "functions.h"
#include "lemlib/api.hpp"

#pragma once


void twopinrollerright()
{
    //Comment out all instances of clamping for now
    clamp_piston.set_value(true);
    pros::delay(100); 

    cascade_left.move_relative(450, 600);
    cascade_right.move_relative(450, 600);

    
    drivePID(-22, 1, 900); //Back up, clear two pins
    drivePID(7, 1, 500); //Get back in position

    turnPID(90, 1, 1100); //Turn towards goal
    driveAndSlowPID(22.5, 250, 1, 1000); 

    cascade_left.move_relative(-350, 600);
    cascade_right.move_relative(-350, 600);

    drivePID(-2, 1, 300); 
    clamp_piston.set_value(false); //PLOP!

    drivePID(-28.5, 1, 1000); //Back out of goal
    turnPID(0, 1, 1000);
    
    driveAndClampPID(18, 100, 20, 1, 1000); //Clamp at match loader here
    cascade_left.move_relative(120, 600); //Lift up cascade slightly
    cascade_right.move_relative(120, 600); 

    drivePID(-15, 1, 1000); //Back out of matchloader

    cascade_left.move_relative(700, 600);
    cascade_right.move_relative(700, 600);

    turnPID(90, 1, 1000);
    driveAndSlowPID(31, 250, 1, 1000);

    cascade_left.move_relative(-550, 600);
    cascade_right.move_relative(-550, 600);

    drivePID(-2, 1, 300);

    clamp_piston.set_value(false); //PLOP

    drivePIDUncapped(-15, 1.5, 600); //Maneuvering to the toggle
    turnPID(45, 1.5, 500); 
    drivePIDUncapped(16, 1.5, 600); 
    turnPID(90, 1.5, 500); 
    drivePID(22, 1.5, 600); 
    turnPID(180, 1.5, 600);//Face the back to toggle

    driveAmount(-18, 600, 500); //Smash toggle
    driveAmount(6, 600, 400); 
    driveAmount(-18, 500, 400);  
    driveAmount(10, 600, 400); 
}