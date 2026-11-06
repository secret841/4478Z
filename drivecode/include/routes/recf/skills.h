#include "main.h"
#include "motors.h"
#include "api.h"
#include "functions.h"
#include "lemlib/api.hpp"

#pragma once

void skills()
{
    // Comment out all instances of clamping for now
    clamp_piston.set_value(true);
    pros::delay(100);

    cascade_left.move_relative(850, 600);
    cascade_right.move_relative(850, 600);

    drivePID(-22, 1.2, 850); // Back up, clear two pins
    drivePID(9.3, 1, 550);   // Get back in position

    pros::delay(100); 
    // chassis.turnToHeading(90, 1000); //
    turnPID(90, 1, 1100); // Turn towards goal
    driveAndSlowPID(22.5, 250, 1, 1200);

    cascade_left.move_relative(-950, 600);
    cascade_right.move_relative(-950, 600);

    drivePID(4, 1, 200);
    clamp_piston.set_value(false); // PLOP!

    drivePID(-27, 1, 1050); // Back out of goal
    turnPID(0, 1, 1200);

    driveAndClampPID(18, 100, 20, 0.9, 950); // Clamp at match loader here
    cascade_left.move_relative(220, 600);  // Lift up cascade slightly
    cascade_right.move_relative(220, 600);

    drivePID(-13.8, 1, 1000); // Back out of matchloader

    cascade_left.move_relative(850, 600);
    cascade_right.move_relative(850, 600);

    turnPID(90, 1, 1200);
    pros::delay(200); 
    driveAndSlowPID(31, 350, 1, 1000);

    cascade_left.move_relative(-650, 600);
    cascade_right.move_relative(-650, 600);

    drivePID(-2, 1, 300);

    clamp_piston.set_value(false); // PLOP

    drivePID(-27, 1, 950); // Back out of goal again

    cascade_left.move_relative(-300, 600); // Put cascade down
    cascade_right.move_relative(-300, 600);
    turnPID(-0.5, 0.9, 1100);

    pros::delay(200); 
    driveAndClampPID(23.5, 100, 20, 0.9, 900); // Clamp at match loader here

    cascade_left.move_relative(120, 600); // Lift up cascade slightly
    cascade_right.move_relative(120, 600);

    drivePID(-15, 1, 800); // Back out of matchloader

    cascade_left.move_relative(1500, 600); // Lift up cascade slightly
    cascade_right.move_relative(1500, 600);
    turnPID(90, 1, 1050); // Turn towards goal again

    pros::delay(200);
    driveAndSlowPID(32, 450, 0.95, 1250);

    cascade_left.move_relative(-950, 520);
    cascade_right.move_relative(-950, 520);

    drivePID(8, 1, 450);

    clamp_piston.set_value(false); // PLOP

    pros::delay(300);

    drivePID(-5, 0.8, 500);
    turnPID(90, 1, 500); 

    pros::delay(50); 
    drivePID(-23, 1, 1000); //Back out of goal, goes for #4

    cascade_left.move_relative(-1250, 520);
    cascade_right.move_relative(-1250, 520);
    turnPID(0, 1, 1500);

    pros::delay(200); 
    driveAndClampPID(23.5, 100, 20, 0.9, 900); // Clamp at match loader here

    cascade_left.move_relative(120, 600); // Lift up cascade slightly
    cascade_right.move_relative(120, 600);

    drivePID(-14, 1, 700); // Back out of matchloader
    turnPID(90, 1, 1200); //Turn to goal

    pros::delay(200);

    cascade_left.move_relative(2400, 600); // Lift up cascade slightly
    cascade_right.move_relative(2400, 600);
    pros::delay(500);

    driveAndSlowPID(32, 450, 0.95, 1250); //Go for 4th pin

    cascade_left.move_relative(-1450, 520);
    cascade_right.move_relative(-1450, 520);

    drivePID(8, 1, 450);

    clamp_piston.set_value(false); // PLOP

    pros::delay(300);

    drivePID(-15, 0.8, 500);
}