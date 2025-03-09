#include "lemlib/api.hpp"
#include "main.h"
#include "autons.hpp"
#include "externs.hpp"

float error(float desiredAngle, float settleError = 50){
    static float error = 0;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    if(fabs(error) <= settleError){
        error = 0;
    }
    return error;
}

void safeAWP(){
int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        for (int i = 0; i < 600; i++){
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(error(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(error(2400)));
                break;
                case 2:
                ws.move_voltage(-liftpid.update(error(15500)));
                break;
                case 3:
                ws.move_voltage(-liftpid.update(error(21000)));
                break;
                case 4:
                ws.move_voltage(-liftpid.update(error(5000)));
            }
            pros::delay(25);
        }
    });

    chassis.setPose(54, 16, 180);
    chassis.moveToPoint(54, -1, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(61.5, 0, 1000, {.forwards = false, .maxSpeed = 55});
    chassis.waitUntilDone();
    intake.move_voltage(12000);
    pros::delay(300);
    intake.move_voltage(-12000);
    pros::delay(100);
    intake.move_voltage(0);
    chassis.moveToPoint(40, 0, 1000, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.moveToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(100);
    clampPistons.set_value(true);
    pros::delay(300);
    intake.move_voltage(12000);
    chassis.cancelAllMotions();
    chassis.moveToPoint(24, 42, 1500);
    pros::delay(500);
    chassis.waitUntilDone();
    chassis.moveToPose(12, 12, 45, 3400, {.forwards = false, .maxSpeed = 80});
    pros::delay(750);
    liftState = 2; liftpid.reset();
    intake.move_voltage(0);
    clampPistons.set_value(false);
    chassis.waitUntilDone();
    liftState = 1;
    pros::delay(500);
    
}

void goalRushSAWP(){
    chassis.setPose(57, -27, 70);
    chassis.moveToPoint(10, -44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(23, -39, 1000);
    chassis.waitUntilDone();
    intake.move_voltage(12000);
    pros::delay(750);
    clampPistons.set_value(false);
    chassis.turnToHeading(208, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(12, -58, 1000, {.maxSpeed = 80});
    pros::delay(700);
    intake.move_voltage(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(24, -24, 1500, {.forwards = false, .maxSpeed = 85});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(300);
    intake.move_voltage(12000);
    pros::delay(700);
    intake.move_voltage(0);
    clampPistons.set_value(false);
    chassis.moveToPose(12, -12, 315, 3000, {.maxSpeed = 70});
}

void safeSigAWP(){
int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        for (int i = 0; i < 600; i++){
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(error(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(error(2400)));
                break;
                case 2:
                ws.move_voltage(-liftpid.update(error(15500)));
                break;
                case 3:
                ws.move_voltage(-liftpid.update(error(21000)));
                break;
                case 4:
                ws.move_voltage(-liftpid.update(error(5000)));
            }
            pros::delay(25);
        }
    });

    chassis.setPose(54, 16, 180);
    chassis.moveToPoint(54, -1, 1500, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.moveToPoint(61.5, 0, 1000, {.forwards = false, .maxSpeed = 65});
    chassis.waitUntilDone();
    intake.move_voltage(12000);
    pros::delay(250);
    intake.move_voltage(-12000);
    pros::delay(100);
    intake.move_voltage(0);
    chassis.moveToPoint(40, 0, 1000, {.minSpeed = 80, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.moveToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    // pros::delay(100);
    clampPistons.set_value(true);
    pros::delay(150);
    intake.move_voltage(12000);
    chassis.cancelAllMotions();
    chassis.moveToPoint(24, 42, 1500);
    pros::delay(200);
    chassis.waitUntilDone();
    chassis.moveToPoint(9, 42, 1500);
    intake.move_voltage(12000);
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.moveToPoint(24,50,1000,{.forwards =false, .minSpeed=50, .earlyExitRange = 5});
    chassis.waitUntilDone();
    clampPistons.set_value(false);

    chassis.moveToPoint(40,0, 1600,{.forwards=false,.minSpeed = 80, .earlyExitRange = 10});
    chassis.waitUntilDone();
    intake.move_voltage(0);

    chassis.moveToPoint(22,-24, 3000,{.forwards=false,.maxSpeed=80});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(300);
    intake.move_voltage(12000);
    chassis.moveToPoint(23, -48, 1500);
    intake.move_voltage(12000);
    chassis.waitUntilDone();
         chassis.moveToPoint(15, -15, 1500,{.maxSpeed=70});
    chassis.waitUntilDone();

        clampPistons.set_value(false);








    
}