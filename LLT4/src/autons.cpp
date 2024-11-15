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
void LiftPID(float desiredAngle, float kP, float kD, float settleError = 250){
    static float prevError = 0;
    static float error = 0;
    static float voltScalar = 1.5;
    //desired angle in centidegrees (0-36,000)
    //settle error 250 cdeg
    //idle = 0, primed = 26, scored = 140
    float derivative = 0;
    prevError = error;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    pros::lcd::set_text(1, std::to_string(error));
    pros::lcd::set_text(4, std::to_string(angle));
    


    derivative = error - prevError;

    float power = (error * kP + derivative * kD) * voltScalar;
    pros::lcd::set_text(3, std::to_string(power));

    pros::lcd::set_text(5, std::to_string(fabs(error)));
    if(fabs(error) <= settleError){
        ws.brake();
        return;
    }

    if(power > 12000){
        power = 12000;
    } else if( power < -12000){
        power = -12000;
    }
    ws.move_voltage(power);
}

void blueNegtive2()
{
    // //grab ring
    // chassis.setPose(55,47,270);
    // chassis.moveToPoint(20, 47, 1500);
    // chassis.waitUntil(20);
    // intake.move_voltage(12000);
    // pros::delay(700);
    // intake.move_voltage(0);
    // //grab goal
    // chassis.moveToPoint(20, 17, 2000, {.forwards = false});
    // chassis.waitUntilDone();
    // clampPistons.set_value(true);
    // pros::delay(250);

    // intake.move_voltage(12000);
    // chassis.moveToPose(3, 40, 270, 3000, {.forwards = true});
    // chassis.waitUntil(18);
    // pros::delay(5000);
    // intake.move_voltage(0);

    // chassis.moveToPose(24, 36, 270, 1750, {.forwards = false});
    // chassis.moveToPoint(24,26, 1000, {.forwards = false});
    // chassis.waitUntilDone();
    // clampPistons.set_value(true);
    // intake.move_voltage(12000);
    // chassis.moveToPoint(23.5, 47, 1000, {.forwards = true});
    chassis.setPose(55,32, 180);
    chassis.moveToPoint(55,0 , 1000, {.earlyExitRange = 25});
    chassis.turnToHeading(270, 1000, {.earlyExitRange = 240});
    chassis.waitUntilDone();
    chassis.moveToPoint(chassis.getPose().x+13, chassis.getPose().y, 750, {.forwards = false, .maxSpeed = 50});
    clampPistons.set_value(true);
    chassis.waitUntilDone();
    intake.move_voltage( 12000);
    pros::delay(200);
    chassis.moveToPoint(55, 0, 1000);
    chassis.moveToPoint(20, 26, 2000, {.forwards = false, .maxSpeed = 75});
    clampPistons.set_value(false);
    intake.move_voltage(0);
    chassis.waitUntilDone();

    clampPistons.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(24, 50, 1000,{}, false);
    intake.move_voltage(12000);

    chassis.moveToPoint(4,43, 1000);

    chassis.moveToPoint(24, 0, 3000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    clampPistons.set_value(false);





 
    





}