#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"

#pragma once

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Front to back ports

//It was 2, -3, 7 before
pros::MotorGroup right_mg({4, -6}, pros::MotorGearset::blue);
pros::MotorGroup left_mg({1, -3}, pros::MotorGearset::blue);


pros::MotorGroup cascade_left({12, 16}, pros::MotorGearset::blue);
pros::MotorGroup cascade_right({-7, -8}, pros::MotorGearset::blue); 

pros::ADIDigitalOut clamp_piston('A'); 

pros::Motor cascade1(4, pros::MotorGearset::blue); 
pros::Motor cascade2(-5, pros::MotorGearset::blue); 
  
pros::Imu inertial(20); 