#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"

#pragma once

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Front to back ports
pros::MotorGroup right_mg({8, -4, 5}, pros::MotorGearset::blue);
pros::MotorGroup left_mg({9, -10, -6}, pros::MotorGearset::blue);
  
pros::Imu inertial(20); 