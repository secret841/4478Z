#include "main.h"
#include "motors.h"
#include "api.h"
#include "functions.h"
#include "lemlib/api.hpp"

#pragma once

void skills()
{
//Prototype code
    //DO LATER
    clamp_piston.set_value(true);
    pros::delay(100);

    cascade_left.move_relative(850, 600);
    cascade_right.move_relative(850, 600);

    drivePID(-22, 1.2, 750); // Back up, clear two pins
    drivePID(7.3, 1, 450);   // Get back in position

    // chassis.turnToHeading(90, 1000); //
    turnPID(90, 1, 1000); // Turn towards goal
    driveAndSlowPID(22.5, 250, 1, 900);

    cascade_left.move_relative(-950, 600);
    cascade_right.move_relative(-950, 600);

    drivePID(3, 1, 300);
    pros::delay(400); 
    clamp_piston.set_value(false); // PLOP!

    turnPID(90, 1, 1000); 

    drivePID(-46, 0.85, 2250); // Back out of goal via a slam
    drivePID(5, 1, 600); 
    turnPID(2, 1, 1000);

    driveAndClampPID(22, 100, 20, 0.75, 1250); // Clamp at match loader here
    cascade_left.move_relative(220, 600);  // Lift up cascade slightly
    cascade_right.move_relative(220, 600);

    drivePID(-13.8, 1, 1000); // Back out of matchloader

    cascade_left.move_relative(1050, 600);
    cascade_right.move_relative(1050, 600);

    turnPID(90, 1, 1100);
    driveAndSlowPID(31, 350, 1, 1000);

    cascade_left.move_relative(-750, 600);
    cascade_right.move_relative(-750, 600);

    drivePID(2, 1, 300);

    clamp_piston.set_value(false); // PLOP

    drivePID(-4, 1, 500); 
    turnPID(90, 1, 1000); 

    drivePID(-24.5, 1, 1250); // Back out of goal again

    cascade_left.move_relative(-300, 600); // Put cascade down
    cascade_right.move_relative(-300, 600);
    turnPID(0, 0.9, 1200);

    driveAndClampPID(27.5, 100, 20, 0.75, 1100); // Clamp at match loader here

    cascade_left.move_relative(120, 600); // Lift up cascade slightly
    cascade_right.move_relative(120, 600);

    drivePID(-14, 1, 700); // Back out of matchloader

    cascade_left.move_relative(1400, 600); // Lift up cascade slightly
    cascade_right.move_relative(1400, 600);
    turnPID(90, 1, 950); // Turn towards goal again

    // pros::delay(28000);
    driveAndSlowPID(32, 450, 1.3, 1050);

    cascade_left.move_relative(-950, 520);
    cascade_right.move_relative(-950, 520);

    drivePID(8, 1, 250);

    clamp_piston.set_value(false); // PLOP

    pros::delay(300);

    cascade_left.move_relative(-1250, 520);
    cascade_right.move_relative(-1250, 520);

    drivePID(-4, 0.8, 500);
    turnPID(90, 1, 1000); // Realigns off of goal the goal
    
   

    pros::delay(150); 
    drivePID(-46, 1, 2200); //Back out of goal, goes for #4
    drivePID(8, 1, 600); 

    cascade_left.move_relative(-450, 600);
    cascade_right.move_relative(-450, 600);
    turnPID(4, 1, 1500);

    pros::delay(200); 
    driveAndClampPID(27.5, 100, 20, 0.8, 1200); // Clamp at match loader here

    cascade_left.move_relative(620, 600); // Lift up cascade slightly
    cascade_right.move_relative(620, 600);
    
    
    drivePID(-14, 1, 700); // Back out of matchloader
    turnPID(90, 1, 1200); //Turn to goal

    pros::delay(200);

    cascade_left.move_relative(3500, 600); // Lift up cascade slightly
    cascade_right.move_relative(3500, 600);
    pros::delay(2200);

    driveAndSlowPID(32, 450, 0.85, 1250); //Go for 4th pin

    cascade_left.move_relative(-2550, 600);
    cascade_right.move_relative(-2550, 600);


    drivePID(8, 1, 450);

    pros::delay(500); 
    clamp_piston.set_value(false); // PLOP

    pros::delay(300);

    drivePID(-15, 0.8, 500);
}