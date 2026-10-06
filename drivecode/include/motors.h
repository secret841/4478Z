#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"

#pragma once

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Front to back ports

//It was 2, -3, 7 before
pros::MotorGroup right_mg({2, -3, 7}, pros::MotorGearset::blue);
pros::MotorGroup left_mg({9, -10, -20}, pros::MotorGearset::blue);

pros::Motor cascade1(4, pros::MotorGearset::blue); 
pros::Motor cascade2(-5, pros::MotorGearset::blue); 
  
pros::Imu inertial(20); 