#include "main.h"
#include "api.h"

#pragma once

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Front to back ports
pros::MotorGroup left_mg({10, -6, -9}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({3, -1, 2}, pros::MotorGearset::blue);

pros::Imu inertial(5); 