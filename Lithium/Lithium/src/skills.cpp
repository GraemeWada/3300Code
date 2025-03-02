#include "lemlib/api.hpp"
#include "main.h"
#include "skills.hpp"
#include "mcl.hpp"
#include "externs.hpp"

void computePossibleLocation(double hOffset = 5.9, double vOffset = 3.5){
    double distance = rightSensor.get(); //get distance to wall
    distance /= 25.4; //converts to inches

    double originalHeading = chassis.getPose().theta;
    double heading = chassis.getPose().theta + 90; //PATH heading of distsensor
    heading = (heading * -1) + 90; //Unit Circle heading of distsensor
    heading *= (3.1415926535/180); // to radians
    // pros::lcd::set_text(6, std::to_string(distance) + ", " + std::to_string(heading));

    double xDist = distance * cos(heading + 3.1415926535); //reverse heading to calculate X shift
    xDist += hOffset * cos(heading + 3.1415926535); //horizontal offset of distance sensor
    xDist += vOffset * cos(heading + (3.1415926535/2)); //vertical offset of distance sensor
    double yDist = distance * sin(heading + 3.1415926535); //reverse heading to calculate Y shift
    yDist += hOffset * sin(heading + 3.1415926535); //horizontal offset of distance sensor
    yDist += vOffset * sin(heading + (3.1415926535/2)); //vertical offset of distance sensor
    
    double leftX = -70+xDist;//x and y values for the resulting square
    double rightX = 70+xDist;
    double topY = 70+yDist;
    double bottomY = -70+yDist;
    pros::lcd::set_text(4, std::to_string(leftX) + ", " + std::to_string(rightX));
    pros::lcd::set_text(5, std::to_string(topY) + ", " + std::to_string(bottomY));

    double currentX = chassis.getPose().x;
    double currentY = chassis.getPose().y;

    double distances[4] = {fabs(currentY - topY), fabs(currentY - bottomY), fabs(currentX - leftX), fabs(currentX - rightX)}; //TBLR
    double minDist = distances[0];
    int wall = 0;
    for (int i = 0; i < 4; i++){
        if(distances[i] < minDist){
            minDist = distances[i];
            wall = i;
        }
    }
    pros::lcd::set_text(6, std::to_string(distances[0]) + ", " + std::to_string(distances[1]));
    pros::lcd::set_text(7, std::to_string(distances[2]) + ", " + std::to_string(distances[3]));
    double xCoordinate;
    double yCoordinate;
    switch(wall){
        case 0: //top
            xCoordinate = currentX;
            yCoordinate = (topY + currentY) / 2;
            break;
        case 1: //bottom
            xCoordinate = currentX;
            yCoordinate = (bottomY + currentY) / 2;
            break;
        case 2: //left
            xCoordinate = (leftX + currentX) / 2;
            yCoordinate = currentY;
            break;
        case 3: //right
            xCoordinate = (rightX + currentX) / 2;
            yCoordinate = currentY;
            break;
    }
    if(fabs(xCoordinate) <= 70 && fabs(yCoordinate <= 70)){
        chassis.setPose(xCoordinate, yCoordinate, originalHeading);
        pros::lcd::set_text(3, std::to_string(wall) + ", X: "+std::to_string(xCoordinate) + ", Y: "+std::to_string(yCoordinate));
    }
}

int Intake(bool forward = true){
    if(forward){
        inState = intakeState::IN;
        return 1;
    } else {
        inState = intakeState::OUT;
        return -1;
    }
}

int Stop(){
    inState = intakeState::STOP;
    return 0;
}

float CLE(float desiredAngle, float settleError = 50){
    static float error = 0;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    if(fabs(error) <= settleError){
        error = 0;
    }
    return error;
}

// void skills(){
//     //stall detection
//     int intakeState = 0;
//     bool detectStalls = true;

//     //lift PID loop
//     //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
//     int liftState = 0;
//     ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     wsr.reset();
//     lemlib::PID liftpid(3, 0, 3);
//     liftpid.reset();
//     pros::Task lift_task([&]() {
//         while (true) {
//             if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 50 && detectStalls){
//                 pros::delay(300);
//                 if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 50 && detectStalls){
//                     intake.move_voltage(intakeState * -1 * 12000);
//                     pros::delay(200);
//                     intake.move_voltage(intakeState * 12000);
//                 }
               
//             }
            
//             switch(liftState){
//                 case 0:
//                 ws.move_voltage(-liftpid.update(CLE(700, 10)));
//                 break;
//                 case 1:
//                 ws.move_voltage(-liftpid.update(CLE(2400)));
//                 break;
//                 case 2:
//                 ws.move_voltage(-liftpid.update(CLE(15500)));
//                 break;
//                 case 3:
//                 ws.move_voltage(-liftpid.update(CLE(21000)));
//                 break;
//                 case 4:
//                 ws.move_voltage(-liftpid.update(CLE(5000)));
//             }
//             pros::delay(25);
//         }
//     });

//     chassis.setPose(-60.5, 0, 90);

//     //Part one

//     //score alliance stake
//     //Move intake forward
//     intakeState = Intake();
//     pros::delay(300);
//     //move intake back slightly to not get stuck on wall stake
//     intakeState = Intake(false);
//     pros::delay(100);

//     intakeState = Stop(); //Stop intake
//     //grab goal
//     chassis.moveToPoint(-48, -24, 2000, {.forwards = false});
//     chassis.waitUntilDone();
//     clampPistons.set_value(true);
//     pros::delay(300);
//     intakeState = Intake();
//     //grab rings 1 and 2
//     chassis.moveToPoint(-24, -24, 2000);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(24, -48, 2000);
//     chassis.waitUntilDone();
//     pros::delay(300);
//     //load ring 3
//     chassis.moveToPoint(48, -60, 2000);
//     pros::delay(500);
//     liftState = 1; liftpid.reset(); detectStalls = false;
//     chassis.waitUntilDone();
//     pros::delay(800);
//     intakeState = Stop();
//     pros::delay(100);
//     //double wall stake
//     chassis.moveToPoint(0, -42, 2000, {.forwards = false});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(180, 1000);
//     liftState = 4; liftpid.reset(); detectStalls = true;
//     pros::delay(100);
//     intakeState = Intake();
//     chassis.moveToPoint(0, -70, 1000, {.maxSpeed = 80});
//     pros::delay(300);
//     intakeState = Stop();
//     chassis.waitUntil(20);
//     liftState = 3; liftpid.reset();
//     pros::delay(500);
// }

// void skills2(){
//     //stall detection
//     int intakeState = 0;
//     bool detectStalls = true;
//     //lift PID loop
//     //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
//     int liftState = 0;
//     ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     wsr.reset();
//     lemlib::PID liftpid(3, 0, 3);
//     liftpid.reset();
//     pros::Task lift_task([&]() {
//         while (true) {
//             if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                 pros::delay(200);
//                 if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                     intake.move_voltage(intakeState * -1 * 12000);
//                     pros::delay(200);
//                     intake.move_voltage(intakeState * 12000);
//                 }
//             }
            
//             switch(liftState){
//                 case 0:
//                 ws.move_voltage(-liftpid.update(CLE(700, 10)));
//                 break;
//                 case 1:
//                 ws.move_voltage(-liftpid.update(CLE(2400)));
//                 break;
//                 case 2:
//                 ws.move_voltage(-liftpid.update(CLE(18000)));
//                 break;
//                 case 3:
//                 ws.move_voltage(-liftpid.update(CLE(21000)));
//                 break;
//                 case 4:
//                 ws.move_voltage(-liftpid.update(CLE(5000)));
//             }
//             pros::delay(25);
//         }
//     });

//     chassis.setPose(-60.5, 0, 90);

//     //Part one

//     //score alliance stake
//     intakeState = Intake();
//     pros::delay(300);
//     intakeState = Intake(false);
//     pros::delay(100);
//     intakeState = Stop();
//     chassis.moveToPoint(-55,0,1000, {.minSpeed = 80, .earlyExitRange = 1});
//     //grab goal
//     chassis.moveToPoint(-48, -24, 1500, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     pros::delay(100);    
//     clampPistons.set_value(true);
    
//     //ring 1
    
//     chassis.moveToPoint(-24,-24, 1500, {.minSpeed = 100, .earlyExitRange = 2});
//     pros::delay(300);
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     chassis.moveToPoint(-4, -40, 1000, {.minSpeed = 85, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(24, -48, 1500, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     //Wall Stake Ring
//     chassis.moveToPoint(0, -56, 1500, {.maxSpeed = 70});
//     pros::delay(800);
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.waitUntilDone();
//     chassis.turnToHeading(180, 1000);
//     chassis.moveToPoint(0, -75, 5000, {.maxSpeed = 60});
//     pros::delay(500);
//     intakeState = Stop();
//     liftState = 2; detectStalls = true; liftpid.reset();
//     pros::delay(1000);
//     chassis.cancelMotion();
//     chassis.moveToPoint(0, -48, 1000, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntil(10);
//     liftState = 0; liftpid.reset();

//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, -46, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-60, -48, 1500, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(100);
//     //ring 6
//     chassis.moveToPose(-24, -57, 90, 2500);
//     chassis.waitUntilDone();
//     pros::delay(200);
//     //corner
//     chassis.moveToPoint(-65, -65, 1500, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     pros::delay(250);
//     intakeState = Stop();  

//     //part 2

//     //goal
//     chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
//     chassis.turnToHeading(180, 800);
//     chassis.moveToPoint(-50, 24, 1500, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     // pros::delay(100);
//     clampPistons.set_value(true);
    
//     //ring 1
//     chassis.moveToPoint(-24,24, 5000, {.maxSpeed = 90});
//     pros::delay(300);
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     chassis.moveToPoint(-7, 43, 5000, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(27, 54, 5000, {.maxSpeed = 85});
//     chassis.waitUntilDone();
//     //Wall Stake Ring
//     chassis.moveToPoint(-6, 59, 5000, {.maxSpeed = 90});
//     pros::delay(800);
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.waitUntilDone();
//     chassis.turnToHeading(0, 5000);
//     chassis.moveToPoint(-5.5, 75, 5000, {.maxSpeed = 60});
//     pros::delay(500);
//     intakeState = Stop();
//     liftState = 2; detectStalls = true; liftpid.reset();
//     pros::delay(1000);
//     chassis.cancelMotion();
//     chassis.moveToPoint(-6, 48, 1000, {.forwards = false});
//     chassis.waitUntil(10);
//     liftState = 0; liftpid.reset();
//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-60, 48, 1500, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(100);
//     //ring 6
//     chassis.moveToPose(-24, 62, 90, 2500);
//     chassis.waitUntilDone();
//     pros::delay(200);
//     //corner
//     chassis.moveToPoint(-65, 65, 1500, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     pros::delay(250);
//     intakeState = Stop();

//     //part 3

//     computePossibleLocation();
//     chassis.moveToPoint(0, 48, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(25, 29, 1500, {.maxSpeed = 100});
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     // pros::delay(50);
//     intakeState = Stop();

//     chassis.moveToPoint(55, -1, 4500, {.forwards = false, .maxSpeed = 65});
//     chassis.waitUntilDone();
//     pros::delay(100);
//     clampPistons.set_value(true);
//     pros::delay(100);
//     chassis.moveToPoint(47, 12, 1000, {.minSpeed = 80, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(48, 60, 2000, {.maxSpeed = 100});
//     pros::delay(250);
//     computePossibleLocation();
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //back up
//     chassis.moveToPose(30, 52, 90, 2000, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     //grab next
//     chassis.moveToPoint(60, 52, 1500, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     computePossibleLocation();
//     // pros::delay(50);
//     //back up
//     chassis.moveToPose(22, 22, 45, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //under ladder
//     // chassis.turnToHeading(225, 600);
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(0, 0, 1500, {.maxSpeed = 100});
//     // chassis.waitUntilDone();
//     // //next ring
//     // chassis.turnToHeading(135, 600);
//     // chassis.waitUntilDone();
//     // chassis.moveToPoint(30, -24, 1500, {.maxSpeed = 100});



//     //move to ring 5
//     chassis.moveToPoint(51, 0, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 7});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(24, -16, 1500, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     pros::delay(750);

//     chassis.turnToHeading(315, 700);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(67, -67, 1500, {.forwards = false, .maxSpeed = 90});
//     pros::delay(250);
//     clampPistons.set_value(false);
//     intakeState = Stop();
//     chassis.waitUntilDone();
//     computePossibleLocation();
//     pros::delay(100);
    
//     //blue goal in corner
//     chassis.moveToPoint(28, -35, 1200, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(66, 20, 2000, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     chassis.moveToPoint(67, 67, 1000);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(48, 48, 1000, {.forwards = false});
// }

void skills3(){
    //use skills path "autonSkills.txt" on path.jerryio.com

    //see sections 5 and 6 of lemlib docs
    //https://lemlib.readthedocs.io/en/stable/tutorials/5_angular_motion.html

    //stall detection
    int intakeState = 0;
    bool detectStalls = true;
    //lift PID loop
    //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
    int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        while (true) {
            if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
                pros::delay(200);
                if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
                    inState = intakeState::OUT;
                    pros::delay(200);
                    inState = intakeState::IN;
                }
            }
            
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(CLE(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(CLE(2400)));
                break;
                case 2:
                ws.move_voltage(-liftpid.update(CLE(18000)));
                break;
                case 3:
                ws.move_voltage(-liftpid.update(CLE(21000)));
                break;
                case 4:
                ws.move_voltage(-liftpid.update(CLE(5000)));
            }
            pros::delay(25);
        }
    });

    //set start pose
    chassis.setPose(-60.5, 0, 90);

    //Part one

    //score alliance stake
    intakeState = Intake();
    pros::delay(300);
    intakeState = Intake(false);
    pros::delay(100);
    intakeState = Stop();
    //move away from wall stake slightly
    chassis.moveToPoint(-55,0,1000, {.minSpeed = 80, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //grab goal
    //always add chassis.waitUntilDone() after a movement
    chassis.moveToPoint(-48, -24, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(100);    
    //clamp goal
    clampPistons.set_value(true);
    
    //ring 1
    chassis.moveToPoint(-24,-24, 1500, {.minSpeed = 100, .earlyExitRange = 2});
    pros::delay(300);
    //intake 300ms after starting movement
    intakeState = Intake();
    chassis.waitUntilDone();
    //ring 2
    //drive around ladder
    chassis.moveToPoint(-4, -40, 1000, {.minSpeed = 85, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //go to ring
    chassis.moveToPoint(24, -48, 1500, {.maxSpeed = 100});
    chassis.waitUntilDone();
    //Wall Stake Ring
    chassis.moveToPoint(0, -56, 1500, {.maxSpeed = 70});
    pros::delay(500);
    //reset liftpid when switching liftstate
    //don't detect stalls when loading wall stakes
    liftState = 1; detectStalls = false; liftpid.reset();
    chassis.waitUntilDone();
    //turn to wall stake
    chassis.turnToHeading(180, 1000);
    //move to wall stake
    chassis.moveToPoint(0, -75, 5000, {.maxSpeed = 60});
    pros::delay(400);
    intakeState = Stop();

    //score wall stake
    liftState = 2; detectStalls = true; liftpid.reset();
    pros::delay(1000);
    //wait until scored and then cancel motion to 0, -75
    chassis.cancelMotion();
    //move away from the wall stake
    chassis.moveToPoint(0, -48, 1000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(10);
    liftState = 0; liftpid.reset();

    //ring 3
    intakeState = Intake();
    chassis.moveToPoint(-24, -46, 1500, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 4, 5
    chassis.moveToPoint(-60, -48, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(300);
    //ring 6
    //swings to a pose driving parallel to wall to maximize chance of picking up ring
    chassis.moveToPose(-24, -57, 90, 2500,{.maxSpeed=90});
    chassis.waitUntilDone();
    pros::delay(350);
    //corner
    chassis.moveToPoint(-65, -65, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    //drop goal and reverse intake to make sure it doesn't get stuck on the hooks
    intakeState = Intake(false);
    clampPistons.set_value(false);
    pros::delay(250);
    //stop intake
    intakeState = Stop();  

    //part 2

    //move to center
    chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    //turn around
    chassis.turnToHeading(180, 800);
    //move to goal
    chassis.moveToPoint(-50, 24, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    // pros::delay(100);
    clampPistons.set_value(true);
    
    //ring 1
    chassis.moveToPoint(-24,24, 5000, {.maxSpeed = 90});
    pros::delay(300);
    intakeState = Intake();
    chassis.waitUntilDone();
    //ring 2
    //drive around ladder
    chassis.moveToPoint(-7, 43, 5000, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //go to ring
    chassis.moveToPoint(27, 54, 5000, {.maxSpeed = 85});
    chassis.waitUntilDone();
    //Wall Stake Ring
    //grab ring
    chassis.moveToPoint(-6, 59, 5000, {.maxSpeed = 90});
    pros::delay(600);
    //stop stall detection, set load state
    liftState = 1; detectStalls = false; liftpid.reset();
    chassis.waitUntilDone();
    //turn to wall stake
    chassis.turnToHeading(0, 5000);
    //move to wall stake
    chassis.moveToPoint(-5.5, 75, 5000, {.maxSpeed = 60});
    pros::delay(400);
    intakeState = Stop();
    //set scored state
    liftState = 2; detectStalls = true; //liftpid.reset();
    pros::delay(1000);
    //cancel move to wall stake
    chassis.cancelMotion();
    //move away from wall stake
    chassis.moveToPoint(-6, 48, 1000, {.forwards = false});
    chassis.waitUntil(10);
    //set idle state
    liftState = 0; liftpid.reset();
    //ring 3
    intakeState = Intake();
    chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 4, 5
    chassis.moveToPoint(-60, 48, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(300);
    //ring 6
    chassis.moveToPose(-24, 62, 90, 2500,{.maxSpeed=70});
    chassis.waitUntilDone();
    pros::delay(350);
    //corner
    chassis.moveToPoint(-65, 65, 1500, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    intakeState = Intake(false);
    clampPistons.set_value(false);
    pros::delay(250);
    intakeState = Stop();

    //part 3
    //distance sensor reset
    computePossibleLocation();
    //move to point near ladder
    chassis.moveToPoint(0, 48, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //move towards ring
    chassis.moveToPoint(25, 29, 1500, {.maxSpeed = 100});
    //intake until done motion to grab and hold ring
    intakeState = Intake();
    chassis.waitUntilDone();
    // pros::delay(50);
    intakeState = Stop();
    //move to goal
    chassis.moveToPoint(55, -1, 3500, {.forwards = false, .maxSpeed = 65});
    chassis.waitUntilDone();
    pros::delay(100);
    //clamp goal
    clampPistons.set_value(true);
    pros::delay(100);
    //drive around blue ring goal
    chassis.moveToPoint(47, 12, 1000, {.minSpeed = 80, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //move to ring cluster and pick up 2 rings
    chassis.moveToPoint(45, 64, 1600, {.maxSpeed = 100});
    pros::delay(250);
    //distance sensor reset while driving to wall stake
    computePossibleLocation();
    intakeState = Intake();
    chassis.waitUntilDone();
    //back up and turn towards next ring
    chassis.moveToPose(30, 52, 90, 2000, {.forwards = false, .minSpeed=80, .earlyExitRange=5});
    chassis.waitUntilDone();
    //grab next
    chassis.moveToPoint(62, 47, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    // pros::delay(50);
    //back up to pose 22, 22 facing 45 degrees
    chassis.moveToPose(22, 22, 45, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
    chassis.waitUntilDone();

    //move to ring 5
    //drive around ladder
    chassis.moveToPoint(51, 0, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 7});
    chassis.waitUntilDone();
    //grab ring
    chassis.moveToPoint(24, -16, 1500, {.maxSpeed = 100});
    chassis.waitUntilDone();
    pros::delay(750);
    //turn towards corner
    chassis.turnToHeading(315, 700);
    chassis.waitUntilDone();
    
    //Go to center ring
    // chassis.moveToPoint(3, -3, 1000, {.maxSpeed = 90});
    // chassis.waitUntilDone();
    //push to corner
    chassis.moveToPoint(67, -67, 1500, {.forwards = false, .maxSpeed = 90});
    pros::delay(250);
    //unclamp before movement ends to goal wedge
    clampPistons.set_value(false);
    //stop intake
    intakeState = Stop();
    chassis.waitUntilDone();

    //distance reset
    computePossibleLocation();
    pros::delay(100);
    
    //blue goal in corner
    //drive around close blue goal
    chassis.moveToPoint(28, -35, 1200, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //drive to  far blue goal
    chassis.moveToPose(60, 20, 220, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();
    //push blue goal into corner
    chassis.moveToPoint(67, 67, 1000, {.forwards = false});
    chassis.waitUntilDone();
    //move back
    chassis.moveToPoint(48, 48, 1000);
}
/*

// 53 CENTER AUTON
// */
// void skills4(){
//     //use skills path "autonSkills.txt" on path.jerryio.com

//     //see sections 5 and 6 of lemlib docs
//     //https://lemlib.readthedocs.io/en/stable/tutorials/5_angular_motion.html

//     //stall detection
//     int intakeState = 0;
//     bool detectStalls = true;
//     //lift PID loop
//     //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
//     int liftState = 0;
//     ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     wsr.reset();
//     lemlib::PID liftpid(3, 0, 3);
//     liftpid.reset();
//     pros::Task lift_task([&]() {
//         while (true) {
//             if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                 pros::delay(200);
//                 if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                     inState = intakeState::OUT;
//                     pros::delay(200);
//                     inState = intakeState::IN;
//                 }
//             }
            
//             switch(liftState){
//                 case 0:
//                 ws.move_voltage(-liftpid.update(CLE(700, 10)));
//                 break;
//                 case 1:
//                 ws.move_voltage(-liftpid.update(CLE(2400)));
//                 break;
//                 case 2:
//                 ws.move_voltage(-liftpid.update(CLE(18000)));
//                 break;
//                 case 3:
//                 ws.move_voltage(-liftpid.update(CLE(21000)));
//                 break;
//                 case 4:
//                 ws.move_voltage(-liftpid.update(CLE(5000)));
//             }
//             pros::delay(25);
//         }
//     });

//     //set start pose
//     chassis.setPose(-60.5, 0, 90);

//     //Part one

//     //score alliance stake
//     intakeState = Intake();
//     pros::delay(300);
//     intakeState = Intake(false);
//     pros::delay(100);
//     intakeState = Stop();
//     //move away from wall stake slightly
//     chassis.moveToPoint(-55,0,1000, {.minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //grab goal
//     //always add chassis.waitUntilDone() after a movement
//     chassis.moveToPoint(-48, -24, 1500, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     pros::delay(100);    
//     //clamp goal
//     clampPistons.set_value(true);
    
//     //ring 1
//     chassis.moveToPoint(-24,-24, 1500, {.minSpeed = 100, .earlyExitRange = 2});
//     pros::delay(300);
//     //intake 300ms after starting movement
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     //drive around ladder
//     chassis.moveToPoint(-4, -40, 1000, {.minSpeed = 85, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //go to ring
//     chassis.moveToPoint(24, -48, 1500, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     //Wall Stake Ring
//     chassis.moveToPoint(0, -56, 1500, {.maxSpeed = 70});
//     pros::delay(500);
//     //reset liftpid when switching liftstate
//     //don't detect stalls when loading wall stakes
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.waitUntilDone();
//     //turn to wall stake
//     chassis.turnToHeading(180, 1000);
//     //move to wall stake
//     chassis.moveToPoint(0, -75, 5000, {.maxSpeed = 60});
//     pros::delay(400);
//     intakeState = Stop();

//     //score wall stake
//     liftState = 2; detectStalls = true; liftpid.reset();
//     pros::delay(1000);
//     //wait until scored and then cancel motion to 0, -75
//     chassis.cancelMotion();
//     //move away from the wall stake
//     chassis.moveToPoint(0, -48, 1000, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntil(10);
//     liftState = 0; liftpid.reset();

//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, -46, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-60, -48, 1500, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(300);
//     //ring 6
//     //swings to a pose driving parallel to wall to maximize chance of picking up ring
//     chassis.moveToPose(-24, -57, 90, 2500,{.maxSpeed=90});
//     chassis.waitUntilDone();
//     pros::delay(350);
//     //corner
//     chassis.moveToPoint(-65, -65, 1300, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     //drop goal and reverse intake to make sure it doesn't get stuck on the hooks
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     // pros::delay(250);
//     //stop intake
//     intakeState = Stop();  

//     //part 2

//     //move to center
//     chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
//     //turn around
//     chassis.turnToHeading(180, 800);
//     //move to goal
//     chassis.moveToPoint(-50, 24, 1500, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     // pros::delay(100);
//     clampPistons.set_value(true);
    
//     //ring 1
//     chassis.moveToPoint(-24,24, 5000, {.maxSpeed = 90});
//     pros::delay(300);
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     //drive around ladder
//     chassis.moveToPoint(-7, 43, 5000, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //go to ring
//     chassis.moveToPoint(27, 54, 5000, {.maxSpeed = 85});
//     chassis.waitUntilDone();
//     //Wall Stake Ring
//     //grab ring
//     chassis.moveToPoint(-6, 57, 5000, {.maxSpeed = 90});
//     pros::delay(600);
//     //stop stall detection, set load state
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.waitUntilDone();
//     //turn to wall stake
//     chassis.turnToHeading(0, 5000);
//     //move to wall stake
//     chassis.moveToPoint(-5.5, 75, 5000, {.maxSpeed = 60});
//     pros::delay(400);
//     intakeState = Stop();
//     //set scored state
//     liftState = 2; detectStalls = true; //liftpid.reset();
//     pros::delay(1000);
//     //cancel move to wall stake
//     chassis.cancelMotion();
//     //move away from wall stake
//     chassis.moveToPoint(-6, 48, 1000, {.forwards = false});
//     chassis.waitUntil(10);
//     //set idle state
//     liftState = 0; liftpid.reset();
//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-62, 48, 1500, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     pros::delay(600);
//     //ring 6
//     chassis.moveToPose(-24, 62, 90, 2500,{.maxSpeed=90});
//     chassis.waitUntilDone();
//     pros::delay(350);
//     //corner
//     chassis.moveToPoint(-65, 65, 1300, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     // pros::delay(250);
//     intakeState = Stop();

//     //part 3
//     //distance sensor reset
//     computePossibleLocation();
//     //move to point near ladder
//     chassis.moveToPoint(0, 48, 1700, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //move towards ring
//     chassis.moveToPoint(25, 29, 1500, {.maxSpeed = 100});
//     //intake until done motion to grab and hold ring
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     // pros::delay(50);
//     intakeState = Stop();
//     //move to goal
//     chassis.moveToPoint(55, -1, 3500, {.forwards = false, .maxSpeed = 65});
//     chassis.waitUntilDone();
//     pros::delay(100);
//     //clamp goal
//     clampPistons.set_value(true);
//     pros::delay(100);
//     //drive around blue ring goal
//     chassis.moveToPoint(47, 12, 1000, {.minSpeed = 80, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //move to ring cluster and pick up 2 rings
//     chassis.moveToPoint(45, 64, 1600, {.maxSpeed = 100});
//     pros::delay(250);
//     //distance sensor reset while driving to wall stake
//     computePossibleLocation();
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //back up and turn towards next ring
//     chassis.moveToPose(30, 52, 90, 2000, {.forwards = false, .minSpeed=80, .earlyExitRange=5});
//     chassis.waitUntilDone();
//     //grab next
//     chassis.moveToPoint(62, 47, 1500, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     // pros::delay(50);
//     //back up to pose 22, 22 facing 45 degrees
//     chassis.moveToPose(22, 22, 45, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();

//     //move to ring 5
//     //drive around ladder
//     chassis.moveToPoint(51, 0, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 7});
//     chassis.waitUntilDone();
//     //grab ring
//     chassis.moveToPoint(24, -16, 1200, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     pros::delay(750);
//     //turn towards corner
//     chassis.turnToHeading(315, 700);
//     chassis.waitUntilDone();
    
//     //Go to center ring
//     chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
//     pros::delay(800);
//     //push to corner
//     chassis.moveToPoint(67, -67, 1500, {.forwards = false, .maxSpeed = 90});
//     pros::delay(750);
//     //unclamp before movement ends to goal wedge
//     clampPistons.set_value(false);
//     //stop intake
//     intakeState = Stop();
//     chassis.waitUntilDone();

//     //distance reset
//     computePossibleLocation();
//     pros::delay(100);
    
//     //blue goal in corner
//     //drive around close blue goal
//     chassis.moveToPoint(28, -35, 1200, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //drive to  far blue goal
//     chassis.moveToPose(60, 20, 220, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
//     chassis.waitUntilDone();
//     //push blue goal into corner
//     chassis.moveToPoint(67, 67, 1000, {.forwards = false});
//     chassis.waitUntilDone();
//     //move back
//     chassis.moveToPoint(48, 48, 1000);
// }

// void skills5(){
//     //use skills path "autonSkills.txt" on path.jerryio.com

//     //see sections 5 and 6 of lemlib docs
//     //https://lemlib.readthedocs.io/en/stable/tutorials/5_angular_motion.html

//     //stall detection
//     int intakeState = 0;
//     bool detectStalls = true;
//     //lift PID loop
//     //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
//     int liftState = 0;
//     ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     wsr.reset();
//     lemlib::PID liftpid(3, 0, 3);
//     liftpid.reset();
//     pros::Task lift_task([&]() {
//         while (true) {
//             if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                 pros::delay(200);
//                 if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                     inState = intakeState::OUT;
//                     pros::delay(200);
//                     inState = intakeState::IN;
//                 }
//             }
            
//             switch(liftState){
//                 case 0:
//                 ws.move_voltage(-liftpid.update(CLE(700, 10)));
//                 break;
//                 case 1:
//                 ws.move_voltage(-liftpid.update(CLE(2400)));
//                 break;
//                 case 2:
//                 ws.move_voltage(-liftpid.update(CLE(18000)));
//                 break;
//                 case 3:
//                 ws.move_voltage(-liftpid.update(CLE(21000)));
//                 break;
//                 case 4:
//                 ws.move_voltage(-liftpid.update(CLE(5000)));
//             }
//             pros::delay(25);
//         }
//     });

//     //set start pose
//     chassis.setPose(-60, 0, 90);

// //Part one

//     //score alliance stake
//     intakeState = Intake();
//     pros::delay(300);
//     intakeState = Intake(false);
//     pros::delay(100);
//     intakeState = Stop();
//     //move away from allian stake slightly
//     chassis.moveToPoint(-55,0,1000, {.minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //grab goal
//     //always add chassis.waitUntilDone() after a movement
//     chassis.moveToPoint(-48, -24, 1000, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     //clamp goal
//     clampPistons.set_value(true);
//     pros::delay(100);    
    
//     //ring 1
//     chassis.moveToPoint(-24,-24, 1500, {.minSpeed = 100, .earlyExitRange = 2});
//     pros::delay(300);
//     //intake 300ms after starting movement
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     //drive around ladder
//     chassis.moveToPoint(-4, -40, 1000, {.minSpeed = 85, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //go to ring
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.moveToPoint(24, -48, 1200, {.maxSpeed = 100}); //1500 to 1200
//     chassis.waitUntilDone();


//     //Wall Stake Ring
//     chassis.moveToPoint(0, -37, 1500, {.maxSpeed = 70}); //57 to 42, go perp to wall stake instead of par
//     chassis.waitUntilDone();


//     // pros::delay(500);
//     //reset liftpid when switching liftstate
//     //don't detect stalls when loading wall stakes
//     chassis.waitUntilDone();
//     //turn to wall stake
//     chassis.turnToHeading(180, 500); //decreased from 1s to 0.5x
//     //move to wall stake
//     chassis.moveToPoint(0, -75, 5000, {.maxSpeed = 60});
//     pros::delay(1500);
//     // intakeState = Stop();

//     //score wall stake
//     liftState = 2; detectStalls = true; liftpid.reset();
//     pros::delay(700);
//     //wait until scored and then cancel motion to 0, -75
//     chassis.cancelMotion();
//     //move away from the wall stake
//     chassis.moveToPoint(0, -48, 1000, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntil(10);
//     liftState = 0; liftpid.reset();

//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, -46, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-62, -48, 1200, {.maxSpeed = 70}); //-48 to -47
//     chassis.waitUntilDone();
// //HARD SET AGAIN CUZ IDK WHY IT NOT WORK AHAHAHHAHAHHAHAHAHHAHH
//     // chassis.setPose(-62, -48, chassis.getPose().theta); //-48 to -47
//     // pros::delay(300);
// //ADD 4 TO X AXIS AS IT IS CONSISTENTLY OFF BY 3 FOR SOME REASON
//     // chassis.setPose(chassis.getPose().x+4,chassis.getPose().y,chassis.getPose().theta);

//     //ring 6
//     //swings to a pose driving parallel to wall to maximize chance of picking up ring
//     chassis.moveToPose(-20, -62, 90, 1500,{.maxSpeed=110}); //2500 to 1500
//     chassis.waitUntilDone();
//     pros::delay(350);
//     //corner
//     chassis.moveToPoint(-65, -65, 700, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     //drop goal and reverse intake to make sure it doesn't get stuck on the hooks
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     // pros::delay(250);
//     //stop intake
//     intakeState = Stop();  

// //part 2

//     //move to center
//     chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
// //HARDSET AGAIN WTFREAK IS GOING ONNNNNNNNNNN AHHHHHHASFHDFHOSDIHOFD
//     // chassis.setPose(-48, 0, chassis.getPose().theta);
//     //turn around
//     chassis.turnToHeading(180, 800);
//     //move to goal
//     chassis.moveToPoint(-53, 24, 1500, {.forwards = false, .maxSpeed = 80}); //-50 to -53
//     chassis.waitUntilDone();
//     // pros::delay(100);
//     clampPistons.set_value(true);
    
//     //ring 1
//     chassis.moveToPoint(-24,24, 1500, {.maxSpeed = 90});
//     pros::delay(300);
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     //drive around ladder
//     chassis.moveToPoint(-7, 43, 1000, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //go to ring
//     chassis.moveToPoint(27, 54, 1200, {.maxSpeed = 85});
//     chassis.waitUntilDone();
//     //Wall Stake Ring
//     //grab ring
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.moveToPoint(0, 42, 1500, {.maxSpeed = 90}); //57 to 42, go perp to wall stake instead of par
//     pros::delay(600);
//     //stop stall detection, set load state
//     chassis.waitUntilDone();
//     //turn to wall stake
//     chassis.turnToHeading(0, 500);
//     //move to wall stake
//     chassis.moveToPoint(-0, 75, 1500, {.maxSpeed = 60}); //1000 to 1500

//     pros::delay(1500);
//     intakeState = Stop();
//     //set scored state
//     liftState = 2; detectStalls = true; //liftpid.reset();
//     pros::delay(700);
//     //cancel move to wall stake
//     chassis.cancelMotion();
//     //move away from wall stake
//     chassis.moveToPoint(-6, 48, 1000, {.forwards = false});
//     chassis.waitUntil(10);
//     //set idle state
//     liftState = 0; liftpid.reset();
//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-64, 48, 1000, {.maxSpeed = 70}); //48 to 47 1500 to 1000
//     chassis.waitUntilDone();
//     pros::delay(600);
//     //ring 6
//     chassis.moveToPose(-20, 64, 90, 1500,{.maxSpeed=90}); //24 to 18
//     chassis.waitUntilDone();
//     pros::delay(350);
//     //compute
//     computePossibleLocation();
//     //corner
//     chassis.moveToPoint(-65, 65, 700, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     // pros::delay(250);
//     intakeState = Stop();

// //part 3
//     //distance sensor reset
//     computePossibleLocation();
//     //move to point near ladder
//     chassis.moveToPoint(-5, 48, 1700, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();
// //HARD SET 4TH TIME YAY!!!!!!!!11!!!
//     chassis.setPose(0, 48, chassis.getPose().theta);
//     //move towards ring
//     chassis.moveToPoint(25, 29, 1500, {.maxSpeed = 100});
//     //intake until done motion to grab and hold ring
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     // pros::delay(50);
//     intakeState = Stop();
//     //move to goal
//     chassis.moveToPoint(40, 0, 3000, {.forwards = false, .maxSpeed = 65}); //Middle goal
//     chassis.waitUntilDone();
//     pros::delay(100);
//     //clamp goal
//     clampPistons.set_value(true);
//     pros::delay(100);

//     //ring 2 (Bottom right of ladder)
//     //turn to next ring on top  right of ladder
//     chassis.turnToHeading(200,500);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(24, -16, 1000, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     //turn towards center
//     chassis.turnToHeading(315, 500);
//     chassis.waitUntilDone();
    
//     //Ring 3 (Center)
//     chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
//     //move to ring cluster and pick up 2 rings
//     //Ring 4
//     chassis.moveToPoint(44, 4, 1600, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     //Ring 5
//     chassis.moveToPoint(45, 64, 500, {.maxSpeed = 100});
//     pros::delay(250);
//     //distance sensor reset while driving to wall stake
//     computePossibleLocation();
//     chassis.waitUntilDone();
//     //back up and turn towards next ring
//     chassis.moveToPose(30, 52, 90, 2000, {.forwards = false, .minSpeed=80, .earlyExitRange=5});
//     chassis.waitUntilDone();
//     //Ring 6 (Top right right)
//     chassis.moveToPoint(62, 47, 1200, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     //Turn back to corner
//     chassis.turnToHeading(120,500);
//     chassis.waitUntilDone();
//     //Corner
//     chassis.moveToPoint(65,65,1000,{.forwards=false, .minSpeed=80});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);

//     // pros::delay(50);
//     //Go around blue mogoal
//     chassis.moveToPoint(43,22,1000,{.forwards=false, .minSpeed=80});
//     chassis.waitUntilDone();
//     //back up to pose 22, 22 facing 45 degrees
//     chassis.moveToPose(22, 22, 45, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();

//     //move to ring 5
//     //drive around ladder
//     chassis.moveToPoint(51, 0, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 7});
//     chassis.waitUntilDone();
//     pros::delay(750);
//     pros::delay(800);
//     //push to corner
//     chassis.moveToPoint(67, -67, 1500, {.forwards = false, .maxSpeed = 90});
//     pros::delay(750);
//     //unclamp before movement ends to goal wedge
//     clampPistons.set_value(false);
//     //stop intake
//     intakeState = Stop();
//     chassis.waitUntilDone();

//     //distance reset
//     computePossibleLocation();
//     pros::delay(100);
    
//     //blue goal in corner
//     //drive around close blue goal
//     chassis.moveToPoint(28, -35, 1200, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //drive to  far blue goal
//     chassis.moveToPose(60, 20, 220, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
//     chassis.waitUntilDone();
//     //lift ladybrown
//     liftState = 2; detectStalls = true; //liftpid.reset();
//     //push blue goal into corner
//     chassis.moveToPoint(67, 67, 1000, {.forwards = false});
//     chassis.waitUntilDone();
//     //move back
//     chassis.moveToPoint(0, 0, 5000, {.forwards=false,.minSpeed=127});
    
// }


// /*
//  61 AUTON HOPE COPE NOPE
//  2 Per wall (4)
//  1 per alliance (2)

// */
// void skills6(){
//     //use skills path "autonSkills.txt" on path.jerryio.com

//     //see sections 5 and 6 of lemlib docs
//     //https://lemlib.readthedocs.io/en/stable/tutorials/5_angular_motion.html

//     //stall detection
//     int intakeState = 0;
//     bool detectStalls = true;
//     //lift PID loop
//     //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
//     int liftState = 0;
//     ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     wsr.reset();
//     lemlib::PID liftpid(3, 0, 3);
//     liftpid.reset();
//     pros::Task lift_task([&]() {
//         while (true) {
//             if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                 pros::delay(200);
//                 if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                     inState = intakeState::OUT;
//                     pros::delay(200);
//                     inState = intakeState::IN;
//                 }
//             }
            
//             switch(liftState){
//                 case 0:
//                 ws.move_voltage(-liftpid.update(CLE(700, 10)));
//                 break;
//                 case 1:
//                 ws.move_voltage(-liftpid.update(CLE(2400)));
//                 break;
//                 case 2:
//                 ws.move_voltage(-liftpid.update(CLE(18000)));
//                 break;
//                 case 3:
//                 ws.move_voltage(-liftpid.update(CLE(21000)));
//                 break;
//                 case 4:
//                 ws.move_voltage(-liftpid.update(CLE(5000)));
//             }
//             pros::delay(25);
//         }
//     });

//     //set start pose
//     chassis.setPose(-60, 0, 90);

// //Part one

//     //score alliance stake
//     intakeState = Intake();
//     pros::delay(200);
//     intakeState = Intake(false);
//     pros::delay(50);
//     intakeState = Stop();
//     //move away from allian stake slightly
//     chassis.moveToPoint(-55,0,1000, {.minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //grab goal
//     //always add chassis.waitUntilDone() after a movement
//     chassis.moveToPoint(-48, -24, 1000, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     //clamp goal
//     clampPistons.set_value(true);
//     pros::delay(100);    
    
//     //ring 1
//     chassis.moveToPoint(-24,-24, 1500, {.minSpeed = 100, .earlyExitRange = 2});
//     pros::delay(300);
//     //intake 300ms after starting movement
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     //drive around ladder
//     chassis.moveToPoint(-4, -40, 1000, {.minSpeed = 85, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //go to ring
//     //ladybrown
//     chassis.moveToPoint(24, -48, 1200, {.maxSpeed = 100}); //1500 to 1200
//     chassis.waitUntilDone();
    
//     //Lift to slightly above so to intake the other ring for double wallstake cope
//     //ring (WALL 1)
//     //far right bottom ring
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.moveToPoint(24, -59, 1200, {.maxSpeed = 100}); //1500 to 1200
//     chassis.waitUntilDone();
    
    
//     //Wall Stake Ring
//     liftState = 4;
//     chassis.moveToPoint(0, -42, 1500, {.maxSpeed = 70}); //57 to 42, go perp to wall stake instead of par
//     // pros::delay(500);
//     //reset liftpid when switching liftstate
//     //don't detect stalls when loading wall stakes
    
//     chassis.waitUntilDone();
//     //turn to wall stake
//     chassis.turnToHeading(180, 500); //decreased from 1s to 0.5x
//     //move to wall stake
//     chassis.moveToPoint(0, -75, 5000, {.maxSpeed = 60});
//     pros::delay(1500);
//     intakeState = Stop();

//     //score wall stake
//     liftState = 2; detectStalls = true; liftpid.reset();
//     pros::delay(700);
//     //wait until scored and then cancel motion to 0, -75
//     chassis.cancelMotion();
//     //Load ring 2 into lb
    
//     liftState=1; detectStalls = false; liftpid.reset();
    
//     //move away from the wall stake
//     chassis.moveToPoint(0, -48, 1000, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntil(10);
//     liftState = 0; liftpid.reset();

//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, -46, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-60, -48, 1500, {.maxSpeed = 70}); //-48 to -47
//     chassis.waitUntilDone();
//     pros::delay(300);
// //ADD 3 TO X AXIS AS IT IS CONSISTENTLY OFF BY 3 FOR SOME REASON
//     chassis.setPose(chassis.getPose().x+3,chassis.getPose().y,chassis.getPose().theta);

//     //ring 6
//     //swings to a pose driving parallel to wall to maximize chance of picking up ring
//     chassis.moveToPose(-24, -60, 90, 1500,{.maxSpeed=100}); //2500 to 1500
//     chassis.waitUntilDone();
//     pros::delay(350);
//     //corner
//     chassis.moveToPoint(-65, -65, 700, {.forwards = false, .maxSpeed = 80});
//     chassis.waitUntilDone();
//     //drop goal and reverse intake to make sure it doesn't get stuck on the hooks
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     // pros::delay(250);
//     //stop intake
//     intakeState = Stop();  

// //part 2

//     //move to center
//     chassis.moveToPoint(-48, 0, 2000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
//     //turn around
//     chassis.turnToHeading(180, 800);
//     //move to goal
//     chassis.moveToPoint(-53, 24, 1500, {.forwards = false, .maxSpeed = 80}); //-50 to -53
//     chassis.waitUntilDone();
//     // pros::delay(100);
//     clampPistons.set_value(true);
    
//     //ring 1
//     chassis.moveToPoint(-24,24, 1500, {.maxSpeed = 90});
//     pros::delay(300);
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     //ring 2
//     //drive around ladder
//     chassis.moveToPoint(-7, 43, 1000, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //go to ring
//     chassis.moveToPoint(27, 54, 1200, {.maxSpeed = 85});
//     chassis.waitUntilDone();
//     //Wall Stake Ring
//     //grab ring
//     chassis.moveToPoint(-6, 42, 1500, {.maxSpeed = 90}); //57 to 42, go perp to wall stake instead of par
//     pros::delay(600);
//     //stop stall detection, set load state
//     liftState = 1; detectStalls = false; liftpid.reset();
//     chassis.waitUntilDone();
//     //turn to wall stake
//     chassis.turnToHeading(0, 500);
//     //move to wall stake
//     chassis.moveToPoint(-5.5, 75, 1500, {.maxSpeed = 60}); //1000 to 1500

//     pros::delay(1500);
//     intakeState = Stop();
//     //set scored state
//     liftState = 2; detectStalls = true; //liftpid.reset();
//     pros::delay(700);
//     //cancel move to wall stake
//     chassis.cancelMotion();
//     //move away from wall stake
//     chassis.moveToPoint(-6, 48, 1000, {.forwards = false});
//     chassis.waitUntil(10);
//     //set idle state
//     liftState = 0; liftpid.reset();
//     //ring 3
//     intakeState = Intake();
//     chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //ring 4, 5
//     chassis.moveToPoint(-64, 48, 1000, {.maxSpeed = 70}); //48 to 47 1500 to 1000
//     chassis.waitUntilDone();
//     pros::delay(600);
//     //ring 6
//     chassis.moveToPose(-24, 60, 90, 1500,{.maxSpeed=90}); //24 to 18
//     chassis.waitUntilDone();
//     pros::delay(350);
//     //compute
//     computePossibleLocation();
//     //corner
//     chassis.moveToPoint(-65, 65, 700, {.forwards = false, .maxSpeed = 90});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);
//     // pros::delay(250);
//     intakeState = Stop();

// //part 3
//     //distance sensor reset
//     computePossibleLocation();
//     //move to point near ladder
//     chassis.moveToPoint(0, 48, 1700, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();
//     //move towards ring
//     chassis.moveToPoint(25, 29, 1500, {.maxSpeed = 100});
//     //intake until done motion to grab and hold ring
//     intakeState = Intake();
//     chassis.waitUntilDone();
//     // pros::delay(50);
//     intakeState = Stop();
//     //move to goal
//     chassis.moveToPoint(50, 0, 3000, {.forwards = false, .maxSpeed = 65}); //Middle goal
//     chassis.waitUntilDone();
//     pros::delay(100);
//     //clamp goal
//     clampPistons.set_value(true);
//     pros::delay(100);

//     //ring 2 (Bottom right of ladder)
//     //turn to next ring on top  right of ladder
//     chassis.turnToHeading(200,500);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(24, -16, 1000, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     //turn towards center
//     chassis.turnToHeading(315, 500);
//     chassis.waitUntilDone();
    
//     //Ring 3 (Center)
//     chassis.moveToPoint(0, 0, 1000, {.maxSpeed = 90});
//     chassis.waitUntilDone();
//     //move to ring cluster and pick up 2 rings
//     //Ring 4
//     chassis.moveToPoint(44, 4, 1600, {.maxSpeed = 100});
//     chassis.waitUntilDone();
//     //Ring 5
//     chassis.moveToPoint(45, 64, 500, {.maxSpeed = 100});
//     pros::delay(250);
//     //distance sensor reset while driving to wall stake
//     computePossibleLocation();
//     chassis.waitUntilDone();
//     //back up and turn towards next ring
//     chassis.moveToPose(30, 52, 90, 2000, {.forwards = false, .minSpeed=80, .earlyExitRange=5});
//     chassis.waitUntilDone();
//     //Ring 6 (Top right right)
//     chassis.moveToPoint(62, 47, 1200, {.maxSpeed = 70});
//     chassis.waitUntilDone();
//     //Turn back to corner
//     chassis.turnToHeading(120,500);
//     chassis.waitUntilDone();
//     //Corner
//     chassis.moveToPoint(65,65,1000,{.forwards=false, .minSpeed=80});
//     chassis.waitUntilDone();
//     intakeState = Intake(false);
//     clampPistons.set_value(false);

//     // pros::delay(50);
//     //Go around blue mogoal
//     chassis.moveToPoint(43,22,1000,{.forwards=false, .minSpeed=80});
//     chassis.waitUntilDone();
//     //back up to pose 22, 22 facing 45 degrees
//     chassis.moveToPose(22, 22, 45, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 2});
//     chassis.waitUntilDone();

//     //move to ring 5
//     //drive around ladder
//     chassis.moveToPoint(51, 0, 1500, {.maxSpeed = 100, .minSpeed = 75, .earlyExitRange = 7});
//     chassis.waitUntilDone();
//     pros::delay(750);
//     pros::delay(800);
//     //push to corner
//     chassis.moveToPoint(67, -67, 1500, {.forwards = false, .maxSpeed = 90});
//     pros::delay(750);
//     //unclamp before movement ends to goal wedge
//     clampPistons.set_value(false);
//     //stop intake
//     intakeState = Stop();
//     chassis.waitUntilDone();

//     //distance reset
//     computePossibleLocation();
//     pros::delay(100);
    
//     //blue goal in corner
//     //drive around close blue goal
//     chassis.moveToPoint(28, -35, 1200, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 1});
//     chassis.waitUntilDone();
//     //drive to  far blue goal
//     chassis.moveToPose(60, 20, 220, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
//     chassis.waitUntilDone();
//     //lift ladybrown
//     liftState = 2; detectStalls = true; //liftpid.reset();
//     //push blue goal into corner
//     chassis.moveToPoint(67, 67, 1000, {.forwards = false});
//     chassis.waitUntilDone();
//     //move back
//     chassis.moveToPoint(0, 0, 5000, {.forwards=false,.minSpeed=127});
    
// }
// void skills7(){
// //stall detection
//     int intakeState = 0;
//     bool detectStalls = true;
//     //lift PID loop
//     //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
//     int liftState = 0;
//     ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//     wsr.reset();
//     lemlib::PID liftpid(3, 0, 3);
//     liftpid.reset();
//     pros::Task lift_task([&]() {
//         while (true) {
//             if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                 pros::delay(200);
//                 if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
//                     inState = intakeState::OUT;
//                     pros::delay(200);
//                     inState = intakeState::IN;
//                 }
//             }
            
//             switch(liftState){
//                 case 0:
//                 ws.move_voltage(-liftpid.update(CLE(700, 10)));
//                 break;
//                 case 1:
//                 ws.move_voltage(-liftpid.update(CLE(2400)));
//                 break;
//                 case 2:
//                 ws.move_voltage(-liftpid.update(CLE(18000)));
//                 break;
//                 case 3:
//                 ws.move_voltage(-liftpid.update(CLE(21000)));
//                 break;
//                 case 4:
//                 ws.move_voltage(-liftpid.update(CLE(5000)));
//             }
//             pros::delay(25);
//         }
//     });

//     //set start pose
//     chassis.setPose(-60, 0, 90);

//     //Part one

//     //score alliance stake
//     intakeState = Intake();
//     pros::delay(300);
//     intakeState = Intake(false);
//     pros::delay(100);
//     intakeState = Stop();
//     //move away from wall stake slightly
//     chassis.moveToPoint(-55,0,1000, {.minSpeed = 100, .earlyExitRange = 3});
//     chassis.waitUntilDone();

//     //get goal
//     chassis.moveToPoint(-48,-24,1500,{.forwards=false});
//     chassis.waitUntilDone();
//     clampPistons.set_value(true);
//     pros::delay(100);

//     //get 1st ring
//     intakeState = Intake();
//     chassis.moveToPoint(-24,-24,1500,{.minSpeed=70,.earlyExitRange=3});

//     //go to 2nd and 3rd ring first goal
//     chassis.moveToPoint(0,-45,1000,{.minSpeed=90, .earlyExitRange=2});
//     chassis.waitUntilDone();
//     computePossibleLocation();
//     chassis.moveToPoint(24,-46,1500);
//     chassis.waitUntilDone();
//     //get wall stake ring #1
//     chassis.moveToPoint(45,-60,1000);
//     chassis.waitUntilDone();
//     liftState=1; intakeState=0;
//     Stop();
//     //move to infront wall stake
//     chassis.moveToPoint(0,-45,1500);
//     chassis.turnToHeading(180,1000);
//     liftState=4;
//     //move to wallstake and pick up the ring
//     chassis.moveToPoint(0,-62,1500,{.maxSpeed=80});
//     chassis.waitUntilDone();
//     //score first ring and pick up second
//     liftState=2;
//     pros::delay(70);
//     intakeState=Intake();
//     pros::delay(200);
//     intakeState=Stop();
//     //load second ring
//     pros::delay(200);
//     liftState=1;
//     Intake();
//     pros::delay(200);
//     Stop();
//     liftState=2;
//     pros::delay(400);
    


    

    


// }

void skills8(){
    //stall detection
    int intakeState = 0;
    bool detectStalls = true;
    //lift PID loop
    //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
    int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        while (true) {
            if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
                pros::delay(200);
                if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
                    inState = intakeState::OUT;
                    pros::delay(200);
                    inState = intakeState::IN;
                }
            }
            
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(CLE(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(CLE(2700)));
                break;
                case 2:
                ws.move_voltage(-liftpid.update(CLE(19500)));
                break;
                case 3:
                ws.move_voltage(-liftpid.update(CLE(21000)));
                break;
                case 4:
                ws.move_voltage(-liftpid.update(CLE(6000)));
            }
            pros::delay(25);
        }
    });

    //set start pose
    chassis.setPose(-60, 0, 90);

    //Part one
 
    //score alliance stake
    intakeState = Intake();
    pros::delay(300);
    intakeState = Intake(false);
    pros::delay(100);
    intakeState = Stop();
    //move away from wall stake slightly
    chassis.moveToPoint(-55,0,1000, {.minSpeed = 100, .earlyExitRange = 3});
    chassis.waitUntilDone();
 
    //get goal
    chassis.moveToPoint(-48,-24,1500,{.forwards=false});
    chassis.waitUntil(26);
    clampPistons.set_value(true);
    // pros::delay(100);
 
    //get 1st ring
    intakeState = Intake();
    chassis.cancelMotion();
    chassis.moveToPoint(-24, -24, 1500, {.minSpeed = 80, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //drive around ladder
    chassis.moveToPoint(0, -38, 1500, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 2
    chassis.moveToPoint(24, -49, 1500, {.maxSpeed = 70, .minSpeed = 65, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //move to ring 3
    chassis.moveToPoint(48, -60, 2500, {.maxSpeed = 55});
    pros::delay(800);
    liftState = 1; detectStalls = false; liftpid.reset();
    chassis.waitUntilDone();
    
    //move to wall stake
    chassis.moveToPoint(1, -43, 1500, {.forwards = false, .maxSpeed = 90});
    pros::delay(700);
    intakeState = Stop();
    //wiggle intake
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    //partial lift
    pros::delay(300);
    liftState = 4; detectStalls = true; liftpid.reset();
    chassis.waitUntilDone();
    //turn to wall stake
    chassis.turnToHeading(180, 1000);
    intakeState = Intake();
    chassis.waitUntilDone();
    //move into wall stake ring
    chassis.moveToPoint(1, -72, 3000, {.maxSpeed = 50});
    pros::delay(800);
    //hold ring
    intakeState = Stop();
    //score
    pros::delay(600);
    liftState = 2; liftpid.reset();
    pros::delay(800);
    //load next
    liftState = 1;
    chassis.cancelMotion();
    pros::delay(200);
    intakeState = Intake();
    pros::delay(600);
    //wiggle
    chassis.moveToPoint(1, -72, 3000, {.maxSpeed = 60});
    intakeState = Stop();
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    // pros::delay(100);
    // intakeState = Intake();
    // pros::delay(100);
    // intakeState = Stop();
    //score
    pros::delay(100);
    liftState = 2; liftpid.reset();
    pros::delay(800);
    chassis.cancelMotion();
    //move away from wall stake
    chassis.moveToPoint(1, -48, 1500, {.forwards = false});
    chassis.waitUntilDone();
    liftState = 0; liftpid.reset();
    chassis.turnToHeading(270, 500);
    chassis.waitUntilDone();
    //next 3 rings
    chassis.moveToPoint(-62, -48, 1500, {.maxSpeed = 70});
    intakeState = Intake();
    chassis.waitUntilDone();
    pros::delay(100);
    //ring 6
    chassis.moveToPose(-26, -60, 90, 3000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-64, -64, 1000, {.forwards = false});
    pros::delay(400);
    intakeState = Stop();
    chassis.waitUntilDone();
    clampPistons.set_value(false);
    intakeState = Intake(false);

    //part 2

    //move to part 2 goal
    chassis.moveToPoint(-48, -12, 2000, {.minSpeed = 90, .earlyExitRange = 1});
    pros::delay(100);
    intakeState = Stop();
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 400);
    chassis.waitUntilDone();
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    // return;
    intakeState = Intake();

    //ring 1
    chassis.moveToPoint(-24, 24, 1500, {.maxSpeed = 90, .minSpeed = 80, .earlyExitRange = 2});
    chassis.waitUntilDone();
   //drive around ladder
    chassis.moveToPoint(0, 38, 1500, {.maxSpeed = 80, .minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //ring 2
    chassis.moveToPoint(24, 49, 1500, {.maxSpeed = 70, .minSpeed = 65, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //move to ring 3
    chassis.moveToPoint(48, 60, 2500, {.maxSpeed = 55});
    pros::delay(800);
    liftState = 1; detectStalls = false; liftpid.reset();
    chassis.waitUntilDone();

    //move in front of wall stake
    chassis.moveToPoint(0, 43, 1500, {.forwards = false, .maxSpeed = 90});
    pros::delay(500);
    intakeState = Stop();
    //wiggle intake
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    //partial lift
    pros::delay(300);
    liftState = 4; detectStalls = true; liftpid.reset();
    chassis.waitUntilDone();
    //turn to wall stake
    chassis.turnToHeading(0, 1000);
    intakeState = Intake();
    chassis.waitUntilDone();
    //move into wall stake ring
    chassis.moveToPoint(0, 72, 3000, {.maxSpeed = 50});
    pros::delay(800);
    //hold ring
    intakeState = Stop();
    //score
    pros::delay(600);
    liftState = 2; liftpid.reset();
    pros::delay(800);
    //load next
    liftState = 1;
    chassis.cancelMotion();
    pros::delay(200);
    intakeState = Intake();
    pros::delay(600);
    //wiggle
    chassis.moveToPoint(0, 72, 3000, {.maxSpeed = 60});
    intakeState = Stop();
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    pros::delay(100);
    intakeState = Intake();
    pros::delay(100);
    intakeState = Stop();
    pros::delay(100);
    liftState = 2; liftpid.reset();
    pros::delay(800);
    chassis.cancelMotion();
}

void skills9(){
    //stall detection
    int intakeState = 0;
    bool detectStalls = true;
    //lift PID loop
    //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
    int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        while (true) {
            if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
                pros::delay(200);
                if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 1 && detectStalls){
                    inState = intakeState::OUT;
                    pros::delay(200);
                    inState = intakeState::IN;
                }
            }
            
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(CLE(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(CLE(2700)));
                break;
                case 2:
                // {
                    // float power = -10000;
                    // if(-liftpid.update(CLE(21000)) > power){
                    //     power = -liftpid.update(CLE(21000));
                    // }
                    ws.move_voltage(-liftpid.update(CLE(19500)));
                // }
                break;
                case 3:
                {
                    float power = -8000;
                    if(-liftpid.update(CLE(21000)) > power){
                        power = -liftpid.update(CLE(21000));
                    }
                    ws.move_voltage(power);
                }
                break;
                case 4:
                ws.move_voltage(-liftpid.update(CLE(6000)));
            }
            pros::delay(25);
        }
    });

    //set start pose
    chassis.setPose(-60, 0, 90);

    //SPLIT ONE
 
    //score alliance stake
    intakeState = Intake();
    pros::delay(300);
    intakeState = Intake(false);
    pros::delay(100);
    intakeState = Stop();
    //move away from wall stake slightly
    chassis.moveToPoint(-55,0,3000, {.minSpeed = 100, .earlyExitRange = 3});
    chassis.waitUntilDone();
 
    //get goal
    chassis.moveToPoint(-48,-24,3000,{.forwards=false});
    chassis.waitUntil(25);
    clampPistons.set_value(true);
    // pros::delay(100);
 
    //get 1st ring
    intakeState = Intake();
    chassis.cancelMotion();
    chassis.moveToPoint(-24, -24, 3000, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //drive around ladder
    chassis.moveToPoint(0, -38, 3000, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 2});
    //ring 2
    chassis.moveToPoint(24, -48, 3000, {.maxSpeed = 80});
    pros::delay(200);
    liftState = 1; detectStalls = false; liftpid.reset();
    
    
    chassis.waitUntilDone();
    //move to wall stake
    chassis.moveToPoint(0, -42, 3000, {.forwards = false, .maxSpeed = 70});
    pros::delay(700);
    intakeState = Stop();
    // pros::delay(100);
    // intakeState = Intake();
    chassis.waitUntilDone();
    // intakeState = Stop();
    pros::delay(50);
    liftState = 4; detectStalls = true; liftpid.reset();
    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();
    
    
    //drive to wall stake
    chassis.moveToPoint(0, -65, 3000, {.maxSpeed = 60});
    intakeState = Intake();
    //score
    pros::delay(900);
    liftState = 2; liftpid.reset();
    
    pros::delay(600);
    chassis.cancelMotion();
    pros::delay(200);
    
    chassis.moveToPoint(0, -48, 3000, {.forwards = false, .minSpeed = 90, .earlyExitRange = 2});
    pros::delay(50);
    liftState = 0; liftpid.reset();
    chassis.waitUntilDone();
    //next 3
    chassis.moveToPoint(-62, -44, 2000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    //last ring
    chassis.moveToPose(-30, -58, 90, 2000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(100);
    //corner 
    chassis.moveToPoint(-64, -64, 1000, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(false);
    intakeState = Intake(false);

    //SPLIT TWO

    //alliance stake ring
    liftState = 1; liftpid.reset(); detectStalls = false;
    chassis.moveToPoint(43, -46, 2500, {.maxSpeed = 100});
    pros::delay(500);
    // computePossibleLocation(); computePossibleLocation(); computePossibleLocation();
    intakeState = Intake();
    chassis.waitUntilDone();
    
    chassis.turnToHeading(215, 300);
    pros::delay(300);
    chassis.waitUntilDone();
    
    //grab goal
    chassis.moveToPoint(59, -24, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 1});
    
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    chassis.turnToHeading(340, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    
    
    chassis.waitUntilDone();
    pros::delay(200);
    clampPistons.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(70, -70, 1750, {.forwards = false, .minSpeed = 110});
    pros::delay(100);
    intakeState = Stop();
    chassis.waitUntilDone();
    
    //move to goal
    chassis.moveToPoint(48, -30, 1500,{.minSpeed=90,.earlyExitRange=4});
    chassis.waitUntilDone();
    // computePossibleLocation(); computePossibleLocation(); computePossibleLocation();
    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();
    //alliance stake
    chassis.moveToPoint(48, 3, 1200, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    chassis.moveToPoint(70, 0, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    //alignment
    chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 3000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    liftState = 3; liftpid.reset(); detectStalls = true;
    pros::delay(800);
    //backsup
    chassis.moveToPoint(48,0,3000,{.forwards = false,.minSpeed = 70});
    chassis.waitUntilDone();
    liftState=0; liftpid.reset();

    //drive to first ring for second goal (top right ladder)
    intakeState = Intake();
    chassis.moveToPoint(24,-24,3000);
    chassis.waitUntilDone();
    
    chassis.turnToHeading(315,500,{.maxSpeed=100,.minSpeed=70,.earlyExitRange=10});
    chassis.waitUntilDone();
    pros::delay(500);
    intakeState = Stop();
    
    
    //ladder ring
    chassis.moveToPose(-3,3,315,1500,{.lead = 0.05, .maxSpeed = 100, .minSpeed = 70,.earlyExitRange=3});
    chassis.waitUntil(20);
    intakeState = Intake();
    chassis.waitUntilDone();
    
    //ring after ladder (bottom left ladder)
    chassis.moveToPose(-24,24,315, 1500, {.lead = 0.05,.maxSpeed = 100,.minSpeed=80,.earlyExitRange=2});
    chassis.waitUntil(5);
    intakeState = Stop();


    chassis.waitUntil(20);  
    intakeState = Intake();
    chassis.waitUntilDone();
    //corner ring in cluster
    chassis.moveToPoint(-45, 49, 1500, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //left ring
    chassis.moveToPoint(-60, 48, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    //right ring
    chassis.turnToHeading(45, 300);
    chassis.waitUntilDone();
    chassis.moveToPoint(-42, 66, 1500, {.maxSpeed = 80});
    chassis.waitUntilDone();
    chassis.turnToHeading(120, 500);
    chassis.waitUntilDone();
    clampPistons.set_value(false);
    intakeState = Stop();
    pros::delay(50);
    chassis.moveToPoint(-64,70,1000,{.forwards=false});
    chassis.waitUntilDone();
    clampPistons.set_value(false);
    intakeState = Intake(false);
    pros::delay(100);
    intakeState = Stop();

    //SPLIT THREE

    //grab ring for next wall stake
    chassis.moveToPoint(-17, 45, 1500, {.maxSpeed = 100});
    pros::delay(100);
    liftState = 1; liftpid.reset(); detectStalls = false;
    intakeState = Intake();
    chassis.waitUntilDone();
    
    //grab goal
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false, .maxSpeed = 80});
    pros::delay(600);
    intakeState = Stop();
    pros::delay(100);
    // intakeState = Intake();
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    
    //move in front of wall stake
    chassis.moveToPoint(4, 42, 1500, {.maxSpeed = 100});
    // intakeState = Stop();
    liftState = 4; liftpid.reset(); detectStalls = true;
    chassis.waitUntilDone();
    //turn to wall stake
    chassis.turnToHeading(0, 500);
    chassis.waitUntilDone();
    //drive to wall stake
    chassis.moveToPoint(5, 70, 3000, {.maxSpeed = 60});
    intakeState = Intake();
    //score
    pros::delay(900);
    liftState = 2; liftpid.reset();
    pros::delay(600);
    chassis.cancelMotion();
    pros::delay(200);
    //move away from wall stake
    chassis.moveToPoint(5, 48, 3000, {.forwards = false, .minSpeed = 90, .earlyExitRange = 2});
    pros::delay(50);
    liftState = 0; liftpid.reset();
    chassis.waitUntilDone();
    //move to ring closest
    chassis.moveToPoint(24, 48, 1000, {.maxSpeed = 100, .minSpeed = 80, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //move to ring on right 
    chassis.moveToPoint(24, 24, 1000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    //center to right of ring cluster
    chassis.turnToHeading(45, 300);
    chassis.waitUntilDone();
    chassis.moveToPose(65, 42, 90, 3000, {.maxSpeed = 110, .minSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(500);
    //left of ring cluster
    chassis.moveToPose(52, 30, 0, 2500, {.forwards = false, .maxSpeed = 80, .minSpeed = 60});
    chassis.waitUntilDone();
    chassis.moveToPose(28, 63, 270, 2500, {.maxSpeed = 100, .minSpeed = 90, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(17, 63, 1000, {.minSpeed = 100, .earlyExitRange = 1});
    chassis.waitUntilDone();
    //corner
    chassis.turnToPoint(chassis.getPose().x + 48, chassis.getPose().y+10, 400, {.forwards = false});
    chassis.waitUntilDone();
    intakeState = Intake(false);
    clampPistons.set_value(false);
    pros::delay(100);
    chassis.moveToPoint(chassis.getPose().x + 48, chassis.getPose().y+10, 1500, {.forwards = false, .minSpeed = 110});
    
    chassis.waitUntilDone();
    intakeState = Stop();
    //climb
    chassis.moveToPoint(39, 33, 1000, {.minSpeed = 90});
    chassis.waitUntilDone();
    liftState = 2; liftpid.reset();
    chassis.turnToHeading(45, 300);
    chassis.waitUntilDone();
    chassis.moveToPoint(3, -3, 1700, {.forwards = false, .maxSpeed = 80, .minSpeed = 70});
    chassis.waitUntilDone();
    liftState = 4; liftpid.reset();
    left.move_voltage(3000);
    right.move_voltage(3000);
    pros::delay(100);
    left.move_voltage(0);
    right.move_voltage(0);
    pros::delay(5000);
}