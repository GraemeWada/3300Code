#include "main.h"
#include "lemlib/api.hpp"
#include "externs.hpp"

intakeState inState = intakeState::STOP;
alliance color = alliance::RED;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left({-2, -12, 13}, pros::MotorGearset::blue);
pros::MotorGroup right({6, -7, 9}, pros::MotorGearset::blue);

pros::Motor intake(-10);
pros::Motor ws(16);

pros::Optical optical(1);//update port when run

pros::Rotation wsr(21);

pros::ADIDigitalOut clampPistons('G');
pros::ADIDigitalOut descore('B');
pros::ADIDigitalOut pto('H');

pros::Rotation h(15);
pros::Rotation v(5);
pros::Imu imu(14);

// pros::Distance rightSensor(21);
// pros::Distance leftSensor(2);
// pros::Distance upSensor(16);

lemlib::TrackingWheel htw(&h, lemlib::Omniwheel::NEW_275, -4.61);
lemlib::TrackingWheel vtw(&v, 2.000f, -0.433);

lemlib::Drivetrain drivetrain(&left, &right, 11.5, lemlib::Omniwheel::NEW_325, 450, 2);

// lateral PID controller 7 0 35
lemlib::ControllerSettings lateral_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              .2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller 2.65 0 21 kU = 8.5, pU = 0.478 scalar = 0.7, 15
lemlib::ControllerSettings angular_controller(0.7 * 8.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15 * 8.5 * 0.478, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vtw, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &htw, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(10, // joystick deadband out of 127
                                     25, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(10, // joystick deadband out of 127
                                  25, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
                        &throttle_curve,
                        &steer_curve // odometry sensors
);