#include "main.h"
#include "externs.hpp"
#include"lemlib/api.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, 12, -13}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({1, -15, 16}, pros::MotorGearset::blue);
pros::Motor intake(18, pros::MotorGearset::blue);
pros::Motor ws(20, pros::MotorGearset::red);

pros::Rotation wsr(21);
// pros::Rotation odomRot(-7);

pros::Rotation verticalRota(-17);
pros::Rotation horizontalRota(19);

pros::ADIDigitalOut clampPistons ('H');
pros::ADIDigitalOut boink ('G') ;

pros::Optical color (9);
//trackwidth 29cm = ~11.417in
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.5, lemlib::Omniwheel::NEW_325, 450, 2);




//odom
pros::Imu imu(14);
// // pros::Rotation rv(7); //vert
// pros::Rotation rh(-6); //horiz set to negative if reverse

lemlib::TrackingWheel htw(&horizontalRota, lemlib::Omniwheel::NEW_275, -3); // third value is tracking center offset
lemlib::TrackingWheel vtw(&verticalRota, lemlib::Omniwheel::NEW_2, -0.25);

lemlib::OdomSensors sensors(&vtw, // vsertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &htw, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//                             &odomRot, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );

// lateral PID controller
lemlib::ControllerSettings lateral_controller(4.4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.75, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0.01, // integral gain (kI)
                                              15.25, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(10, // joystick deadband out of 127
                                     15, // minimum output where drivetrain will move out of 127
                                     1.03 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(10, // joystick deadband out of 127
                                  15, // minimum output where drivetrain will move out of 127
                                  1.03 // expo curve gain
);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
						&throttle_curve,
						&steer_curve // odometry sensors
);