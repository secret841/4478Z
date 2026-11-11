#include "main.h"
#include "motors.h"
#include "api.h"
#include "functions.h"
#include "lemlib/api.hpp"

#pragma once

int current_cascadePos = 0;
void threepinright()
{
    //chassis.setPose(0, 0, 0); 

    // Comment out all instances of clamping for now
    clamp_piston.set_value(true);
    pros::delay(100);

    cascade_left.move_relative(850, 600);
    cascade_right.move_relative(850, 600);

    drivePID(-22, 1.2, 750); // Back up, clear two pins
    drivePID(7.3, 1, 450);   // Get back in position

    //chassis.turnToHeading(90, 1000); //
    turnPID(90, 1, 1000); // Turn towards goal
    driveAndSlowPID(22.5, 250, 1, 900);

    cascade_left.move_relative(-950, 600);
    cascade_right.move_relative(-950, 600);

    drivePID(3, 1, 400);
    clamp_piston.set_value(false); // PLOP!

    drivePID(-28, 1, 1050); // Back out of goal
    turnPID(0, 1, 1000);

    driveAndClampPID(18, 100, 20, 1, 950); // Clamp at match loader here
    cascade_left.move_relative(220, 600);  // Lift up cascade slightly
    cascade_right.move_relative(220, 600);

    drivePID(-13.8, 1, 1000); // Back out of matchloader

    cascade_left.move_relative(1050, 600);
    cascade_right.move_relative(1050, 600);

    turnPID(90, 1, 1100);
    driveAndSlowPID(31, 350, 1, 1000);

    cascade_left.move_relative(-750, 600);
    cascade_right.move_relative(-750, 600);

    turnPID(90, 1, 250); 
    drivePID(5, 1, 400);

    clamp_piston.set_value(false); // PLOP

    drivePID(-27, 1, 950); // Back out of goal again

    cascade_left.move_relative(-300, 600); // Put cascade down
    cascade_right.move_relative(-300, 600);
    turnPID(0, 1, 1000);

    driveAndClampPID(23.5, 100, 20, 1, 900); // Clamp at match loader here

   
    cascade_left.move_relative(120, 600);    // Lift up cascade slightly
    cascade_right.move_relative(120, 600);

    drivePID(-14, 1, 700); // Back out of matchloader

    cascade_left.move_relative(1500, 600); // Lift up cascade slightly
    cascade_right.move_relative(1500, 600);
    turnPID(90, 1, 950); // Turn towards goal again

    //pros::delay(28000);
    driveAndSlowPID(32, 450, 1.3, 1050);

    cascade_left.move_relative(-950, 520);
    cascade_right.move_relative(-950, 520);

    current_cascadePos = cascade_left.get_position(); //Tracks cascade position to get rid of error

    drivePID(8, 1, 250);

    current_cascadePos = cascade_left.get_position();
    clamp_piston.set_value(false); // PLOP

    pros::delay(300);

    cascade_left.move_relative(-1250, 600);
    cascade_right.move_relative(-1250, 600);

    current_cascadePos = cascade_left.get_position();
    drivePID(-8, 0.8, 500);
}