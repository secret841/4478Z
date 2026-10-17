#include "main.h"
#include "motors.h"
#include "api.h"
#include "functions.h"
#include "lemlib/api.hpp"


#pragma once

void turnleft2pin()
{
    clamp_piston.set_value(true);
    drivePID(1, 1, 200);
    pros::delay(200);

    cascade_left.move_relative(250, 600);
    cascade_right.move_relative(250, 600);
    drivePID(-11, 1, 1000);
    drivePID(5.5, 1, 300);
    drivePID(-11, 1, 1000);
    drivePID(18, 1, 1000);

    // Turns to first goal

    turnPID(-90, 1, 1000);

    cascade_left.move_relative(550, 600);
    cascade_right.move_relative(550, 600);

    pros::lcd::print(7, "This worked!");
    drivePID(18, 0.8, 1000); // ERROR HERE!

    cascade_left.move_relative(-650, 600);
    cascade_right.move_relative(-650, 600);

    pros::delay(200);
    drivePID(-3, 1, 250); // Correction
    pros::delay(200);
    clamp_piston.set_value(false); // PLOP!

    drivePID(-14.3, 1, 1000);
    cascade_left.move_relative(150, 600);
    cascade_right.move_relative(150, 600);
    turnPID(-36.5, 1, 1500); // Turn to next pin

    // 2nd number distslowdown
    driveAndClampPID(32, 650, 200, 0.75, 2500);
    cascade_left.move_relative(850, 600);
    cascade_right.move_relative(850, 600);

    drivePID(5, 1, 600);

    turnPID(-182, 1, 1000);
    drivePID(23, 0.75, 1500);

    cascade_left.move_relative(-950, 600);
    cascade_right.move_relative(-950, 600);

    clamp_piston.set_value(false);
    drivePID(-7, 1, 500);
}