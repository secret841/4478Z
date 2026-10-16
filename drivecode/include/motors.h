#include "main.h"
#include "api.h"
#include "lemlib/api.hpp"

#pragma once

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Front to back ports

//It was 2, -3, 7 before
pros::MotorGroup left_mg({-14, 15}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({16, -17}, pros::MotorGearset::blue);

//Left has 7, 2: Right has 10, 9
pros::MotorGroup cascade_left({-5, 6}, pros::MotorGearset::blue);
pros::MotorGroup cascade_right({-7, 9}, pros::MotorGearset::blue); 

pros::ADIDigitalOut clamp_piston('D'); 

//pros::Motor cascade1(4, pros::MotorGearset::blue); 
//pros::Motor cascade2(-5, pros::MotorGearset::blue); 
  
pros::Imu inertial(13); 