#include "main.h"
#include "motors.h"
#include "api.h"
#include "functions.h"
#include "lemlib/api.hpp"

#pragma once

void threepinleft()
{
     // Comment out all instances of clamping for now
    clamp_piston.set_value(true);
    pros::delay(100);

    cascade_left.move_relative(650, 600);
    cascade_right.move_relative(650, 600);

    drivePID(-22, 1.2, 800); // Back up, clear two pins
    drivePID(7.3, 1, 450);   // Get back in position

    turnPID(-90, 1, 900); // Turn towards goal
    driveAndSlowPID(22.5, 350, 1, 900);

    cascade_left.move_relative(-550, 600);
    cascade_right.move_relative(-550, 600);

    drivePID(-2, 1, 100);
    clamp_piston.set_value(false); // PLOP!

    drivePID(-26.5, 1.2, 950); // Back out of goal
    turnPID(0, 1.2, 900);

    driveAndClampPID(18, 100, 20, 1, 900); // Clamp at match loader here
    cascade_left.move_relative(220, 600);   // Lift up cascade slightly
    cascade_right.move_relative(220, 600);

    drivePID(-15, 1, 900); // Back out of matchloader

    cascade_left.move_relative(700, 600);
    cascade_right.move_relative(700, 600);

    turnPID(-90, 1, 1000);
    driveAndSlowPID(31, 350, 1, 1100);

    cascade_left.move_relative(-550, 600);
    cascade_right.move_relative(-550, 600);

    drivePID(-2, 1, 300);

    clamp_piston.set_value(false); // PLOP

    drivePID(-29.5, 1, 1000); // Back out of goal again

    cascade_left.move_relative(-300, 600); // Put cascade down
    cascade_right.move_relative(-300, 600);
    turnPID(-1, 1, 1000);

    driveAndClampPID(24, 100, 20, 1, 900); // Clamp at match loader here
    cascade_left.move_relative(120, 600);   // Lift up cascade slightly
    cascade_right.move_relative(120, 600);

    drivePID(-15.8, 1, 800); // Back out of matchloader

    cascade_left.move_relative(1450, 600); // Lift up cascade slightly
    cascade_right.move_relative(1450, 600);
    turnPID(-90, 1, 800); //Turn towards goal again
    
    driveAndSlowPID(32.5, 450, 1.25, 800);


    cascade_left.move_relative(-950, 520);
    cascade_right.move_relative(-950, 520);

    drivePID(-2, 1, 200);

    clamp_piston.set_value(false); // PLOP

    pros::delay(300); 

    drivePID(-8, 0.8, 500); 
}