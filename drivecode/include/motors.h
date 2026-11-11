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
pros::Imu inertial(11);

//Change later
lemlib::Drivetrain drivetrain(&left_mg,                   // left motor group
                              &right_mg,                  // right motor group
                              11.625,                     // 11.625 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450,                        // drivetrain rpm is 360
                              2                           // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr,                  // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr,                  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial                 // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              3,   // derivative gain (kD)
                                              3,   // anti windup
                                              1,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.4,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              10,  // derivative gain (kD)
                                              0,   // anti windup
                                              0,   // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0,   // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

//pros::Motor cascade1(4, pros::MotorGearset::blue); 
//pros::Motor cascade2(-5, pros::MotorGearset::blue); 
  
