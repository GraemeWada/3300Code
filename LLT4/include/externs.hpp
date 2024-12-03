#include "lemlib/api.hpp"
extern pros::Controller controller;
extern pros::MotorGroup left_mg;    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
extern pros::MotorGroup right_mg;
extern pros::Motor intake;
extern pros::Motor ws;

extern pros::Rotation wsr;
// extern pros::Rotation odomRot;
extern pros::Rotation verticalRota;
extern pros::Rotation horizontalRota;

extern pros::ADIDigitalOut clampPistons;

extern pros::ADIDigitalOut boink;
//trackwidth 29cm = ~11.417in
extern lemlib::Drivetrain drivetrain;



//odom
extern pros::Imu imu;
// // pros::Rotation rv(7); //vert
// pros::Rotation rh(-6); //horiz set to negative if reverse

extern lemlib::TrackingWheel htw; // third value is tracking center offset
extern lemlib::TrackingWheel vtw;

extern pros::Optical color;
// lemlib::TrackingWheel vtw(&rv, 3.25, 5.25);

extern lemlib::OdomSensors sensors;

// lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//                             &odomRot, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );

// lateral PID controller


// input curve for throttle input during driver control
extern lemlib::ExpoDriveCurve throttle_curve;

// input curve for steer input during driver control
extern lemlib::ExpoDriveCurve steer_curve;


extern lemlib::Chassis chassis;