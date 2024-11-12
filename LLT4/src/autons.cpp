#include "main.h"
#include "lemlib/api.hpp"



void blueNegtive(){
    chassis.setPose(55,47,270);
    chassis.moveToPoint(20, 47, 1500);
    chassis.waitUntil(20);
    intake.move_voltage(12000);
    pros::delay(600);
    intake.move_voltage(0);
    chassis.moveToPoint(20, 17, 2000, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(250);
    
    chassis.moveToPose(3, 40, 270, 3000, {.forwards = true});
    chassis.waitUntil(5);
     
    intake.move_voltage(12000);
    chassis.moveToPoint(15, 35, 1000, {.forwards = false});
    

  
}

void blue_test()
{
    //grab ring
    chassis.setPose(55,47,270);
    chassis.moveToPoint(20, 47, 1500);
    chassis.waitUntil(20);
    intake.move_voltage(12000);
    pros::delay(700);
    intake.move_voltage(0);
    //grab goal
    chassis.moveToPoint(20, 17, 2000, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(250);

    intake.move_voltage(12000);
    chassis.moveToPose(3, 40, 270, 3000, {.forwards = true});
    chassis.waitUntil(18);
    pros::delay(5000);
    intake.move_voltage(0);
}