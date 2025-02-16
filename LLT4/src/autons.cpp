#include "main.h"
#include <atomic>

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
    ws.move_voltage(-power);
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
    intake.move_voltage(-12000);
    pros::delay(200);
    chassis.moveToPoint(55, 0, 1000);
    chassis.moveToPoint(20, 26, 2000, {.forwards = false, .maxSpeed = 75});
    clampPistons.set_value(false);
    intake.move_voltage(0);
    chassis.waitUntilDone();

    clampPistons.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(24, 50, 1000,{}, false);
    intake.move_voltage(-12000);

    chassis.moveToPoint(4,43, 1000);

    chassis.moveToPoint(24, 0, 3000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    clampPistons.set_value(false);

}


/**************************************/
/*                                    */
/*                                    */
/*                                    */
/*             NEW AUTONS             */
/*                                    */
/*                                    */
/*                                    */
/***************************************/


void blueNegative6ring()
{
    
    chassis.setPose(54, 24, 90);
    chassis.moveToPoint(23, 24, 1000, {.forwards = false});
    chassis.waitUntil(24);
    //clamp
    clampPistons.set_value(true);
    pros::delay(300);
    intake.move_voltage(-12000);
    //ring 1
    chassis.moveToPoint(24, 47, 2000);
    chassis.waitUntilDone();
    
    pros::delay(500);
    chassis.moveToPoint(9, 43, 1500); //ring left
    chassis.waitUntilDone();
    pros::delay(700);
    chassis.moveToPoint(-24, 47, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(9,50,1500); //ring right
    chassis.waitUntilDone();
    //middle ring
    chassis.moveToPoint(46, 0, 3000);
    chassis.waitUntilDone();
    //bottom pos ring
    chassis.moveToPoint(63, -59, 2000);
    chassis.waitUntilDone();
    //pos corner sweep
    chassis.moveToPoint(62, -54, 500,{.forwards=false});
    boink.set_value(true);
    chassis.waitUntilDone();

    chassis.turnToHeading(340,1000);
    chassis.waitUntilDone();
    boink.set_value(false);
    clampPistons.set_value(false);
    pros :: delay(200);
    chassis.moveToPoint(28, -6, 5000);



}

void blueNegativeSafe(){
    chassis.setPose(54, 24, 90);
    chassis.moveToPoint(23, 24, 2000, {.forwards = false});
    chassis.waitUntil(24);
    clampPistons.set_value(true);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::delay(500);
    // intake.move_voltage(12000);
    chassis.moveToPoint(24, 47, 2000);
    chassis.waitUntilDone();
    // intake.move_voltage(-12000);
    pros::delay(500);
    chassis.moveToPoint(8, 41, 2000);
    chassis.waitUntilDone();
    pros::delay(700);
    // intake.move_voltage(0);
    // chassis.moveToPose(39,39, 45, 10000, {.forwards = false,.maxSpeed=60});
    // chassis.waitUntilDone();
    // intake.move_voltage(-12000);
    // clampPistons.set_value(false);
    chassis.moveToPoint(24, 47, 2000, {.forwards = false});
    chassis.waitUntilDone();
    // intake.move_voltage(0);
    chassis.moveToPoint(6, 49, 1000);
    chassis.moveToPoint(24,3,1000,{.maxSpeed=100});
}


void bluePositiveSafe(){
    chassis.setPose(54, -24, 90);
    chassis.moveToPoint(24, -24, 5000, {.forwards=false, .maxSpeed=70});
    chassis.waitUntil(24);
    clampPistons.set_value(true);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::delay(200);
    chassis.moveToPoint(24,-48,5000);
    chassis.waitUntilDone();
    pros::delay(500);
    intake.move_voltage(0);
    clampPistons.set_value(false);
    chassis.moveToPoint(24,0,5000);
}

void bluePositiveSafe2(){
    chassis.setPose(60, -24, 0);
    chassis.moveToPoint(61, 0, 5000);
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(70, 0, 1000, {.forwards=false});
    pros::delay(50);
    intake.move_voltage(-12000);
    pros::delay(300);
    intake.move_voltage(12000);
    pros::delay(100);
    intake.move_voltage(0);

    chassis.moveToPoint(60, -24, 5000, {.forwards=false});
    chassis.waitUntilDone();
    
    chassis.moveToPoint(24,-24,5000, {.forwards=false, .maxSpeed=70});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(500);
    intake.move_voltage(-12000);
    
    chassis.moveToPoint(24,-48,5000);
    chassis.waitUntilDone();
    pros::delay(1000);
    intake.move_voltage(0);
    chassis.moveToPoint(24,5,5000, {.maxSpeed = 50});
}

void blueElimsWS(){

    std::atomic<int> i(0);

    pros::Task screenTask([&]() {
        color.set_led_pwm(100);
           int redMin = 9;
                int redMax = 18;
                int blueMin = 190;
                int blueMax = 250;
                bool blue =true;      
            while (true) {
         
                            
            static float liftP = 3;
                    static float liftD = 1;
                    switch(i){
                            case 0:
                            LiftPID(0, liftP, liftD);
                            break;
                            case 1:
                            LiftPID(2500, liftP, liftD);
                            break;
                            case 2:
                            LiftPID(12000, liftP, liftD);
                            case 3:
                            LiftPID(14500, liftP, liftD);
                            pros::delay(20);
                        }
                    
                
            
                if(!blue){
                    if(color.get_hue()>blueMin&& color.get_hue()<blueMax){
                        pros::delay(60);
                        intake.move_voltage(12000);
                        pros::delay(80);
                        intake.move_voltage(-12000);

                    }
                }
                else{
                    if(color.get_hue()>redMin&& color.get_hue()<redMax){
                        pros::delay(60);
                        intake.move_voltage(12000);
                        pros::delay(80);
                        intake.move_voltage(-12000);

                    }
                }

                
            pros::delay(10);
        }
    });
   

    chassis.setPose(54, 23.5, 90 );

    chassis.moveToPoint(30, 23.5, 3000, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(200);

    chassis.moveToPoint(7.5, 38, 3000, {.maxSpeed=90});
    intake.move_voltage(-12000);
    chassis.waitUntilDone();
    pros::delay(200);
    intake.move_voltage(0);

    chassis.moveToPoint(22,45.5,2000, {.minSpeed=60,.earlyExitRange = 5.5});
    intake.move_voltage(-12000);
    chassis.waitUntilDone();
    pros::delay(200);
    intake.move_voltage(0);

    chassis.moveToPoint(8.5,50,2000, {.maxSpeed=100, .minSpeed = 45, .earlyExitRange = 5});
    intake.move_voltage(-12000);
    chassis.waitUntilDone();
    pros::delay(200);
    i=1;
    chassis.moveToPose(6.4,60,330, 3000,{.maxSpeed= 85});
    i=2;
    chassis.waitUntilDone();
    intake.move_voltage(4000);
    pros::delay(70);
    i=3;
    intake.move_voltage(0);    
    pros::delay(500);
    i=0;
    chassis.turnToPoint(0,48,1000, {.minSpeed=40, .earlyExitRange=30});
    chassis.moveToPose(47,0,180,6000);
    chassis.waitUntil(45);
    intake.move_voltage(-12000);
    chassis.moveToPoint(47,10,750, {.forwards=false,.minSpeed = 70, .earlyExitRange=4});
    chassis.moveToPoint(47,-10,1000);
    clampPistons.set_value(false);
    chassis.waitUntilDone();
    intake.move_voltage(0);
    chassis.moveToPose(57,0,0, 1000, {.minSpeed=75});
    chassis.turnToHeading(270, 800);
    chassis.moveToPoint(61,0,800);
    chassis.waitUntilDone();
    intake.move_voltage(-12000);
    screenTask.remove();
};
void intakeUntilBlue(){
            color.set_led_pwm(100);
            int redMin = 9;
            int redMax = 18;
            int blueMin = 190;
            int blueMax = 250;
            bool blue =true;      
                
            while (!(color.get_hue()>blueMin&& color.get_hue()<blueMax)) {
                intake.move_voltage(-12000);
                pros::delay(10);
            }
}

void blueFullSoloAWP(){
   chassis.setPose(53.5,31,180);

    chassis.moveToPoint(49,38,1000,{.forwards = false, .maxSpeed = 75});//push alliance off the line
    
    //score alliacne stake with preload
    chassis.moveToPoint(49,-1,2000);
    chassis.turnToHeading(270,1500,{.maxSpeed=85});
    chassis.moveToPoint(62,-1,750,{.forwards = false});
    chassis.waitUntilDone();
    intake.move_voltage(-12000);
    pros::delay(400);
    intake.move_voltage(0);
    //get the goal

    chassis.moveToPoint(40,0,1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(23.5,23.5,1000,{.forwards =false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(200);

    //score rings from stack on our side
    chassis.moveToPoint(23.5,47,1000);
    intake.move_voltage(-12000);

    //score rings from stack in middle
    chassis.moveToPoint(2, 42, 1300, {.maxSpeed=80});
    // pros::delay(500);
    // chassis.moveToPoint(17,47,1500,{.forwards=false});

    // chassis.moveToPoint(9,50,2000,{.maxSpeed=80});
    // pros::delay(500);
    //touch ladder
    chassis.moveToPoint(16,11,2000);
}





    


   


/**************************************/
/*                                    */
/*                                    */
/*                                    */
/*             RED AUTONS             */
/*                                    */
/*                                    */
/*                                    */
/***************************************/
void redNegativeSafe(){
    // chassis.setPose(-54, 24, 270);
    // chassis.moveToPoint(-23, 24, 1500, {.forwards = false});
    // chassis.waitUntil(24);
    // clampPistons.set_value(true);
    // pros::delay(500);
    // intake.move_voltage(-12000);
    // pros::delay(500);
    // // intake.move_voltage(12000);
    // chassis.moveToPoint(-24, 47, 1500);
    // chassis.waitUntilDone();
    // // intake.move_voltage(-12000);
    // pros::delay(500);
    // chassis.moveToPoint(-12, 43, 1500);
    // chassis.waitUntilDone();
    // pros::delay(700);
    // // intake.move_voltage(0);
    // // chassis.moveToPose(39,39, 45, 10000, {.forwards = false,.maxSpeed=60});
    // // chassis.waitUntilDone();
    // // intake.move_voltage(-12000);
    // // clampPistons.set_value(false);
    // chassis.moveToPoint(-24, 47, 1000, {.forwards = false});
    // chassis.waitUntilDone();
    // // intake.move_voltage(0);
    // chassis.moveToPoint(-10, 50, 1500);
    // pros::delay(500);
    // chassis.moveToPoint(-24,3,1500,{.maxSpeed=50});
    chassis.setPose(-54, 24, 270);
    chassis.moveToPoint(-23, 24, 2000, {.forwards = false});
    chassis.waitUntil(24);
    clampPistons.set_value(true);
    pros::delay(250);
    intake.move_voltage(-12000);
    pros::delay(200);  
    // intake.move_voltage(12000);
    chassis.moveToPoint(-24, 47, 2000);
    chassis.waitUntilDone();
    // intake.move_voltage(-12000);
    pros::delay(200);
    //-8
    chassis.moveToPoint(-7.5, 41, 2000);
    chassis.waitUntilDone();
    pros::delay(150);
    // intake.move_voltage(0);
    // chassis.moveToPose(39,39, 45, 10000, {.forwards = false,.maxSpeed=60});
    // chassis.waitUntilDone();
    // intake.move_voltage(-12000);
    // clampPistons.set_value(false);
    chassis.moveToPoint(-24, 47, 2000, {.forwards = false, .earlyExitRange = 5});
  
    // intake.move_voltage(0);
    //new
    //-9,49
    chassis.moveToPoint(-8, 52, 1000);
     pros::Task screenTask([&]() {
        color.set_led_pwm(100);
            while (true) {
                int redMin = 9;
                int redMax = 18;
                int blueMin = 190;
                int blueMax = 250;
                while(true){
                    
                        
                            if(color.get_hue()>blueMin&& color.get_hue()<blueMax){
                                pros::delay(75);
                                intake.move_voltage(12000);
                                pros::delay(80);
                                intake.move_voltage(-12000);
                                // console.print("saw blue");
                            }

                            
                       
                pros::delay(10);
        }
    }});
    chassis.moveToPose(-48,0, 180, 2500);
    chassis.moveToPoint(-48,-18, 1000);
    intake.move_voltage(-12000);
    


}

void redPositiveSafe(){
    chassis.setPose(-54, -24, 270);
    chassis.moveToPoint(-24, -24, 5000, {.forwards=false, .maxSpeed=70});
    chassis.waitUntil(24);
    clampPistons.set_value(true);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::delay(200);
    chassis.moveToPoint(-24,-48,5000);
    chassis.waitUntilDone();
    pros::delay(500);
    intake.move_voltage(0);
    clampPistons.set_value(false);
    chassis.moveToPoint(-24,0,5000);
}

void redPositiveSafe2(){
    chassis.setPose(-60, -24, 0);
    chassis.moveToPoint(-61, 0, 5000);
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-70, 0, 1000, {.forwards=false});
    pros::delay(50);
    intake.move_voltage(-12000);
    pros::delay(300);
    intake.move_voltage(12000);
    pros::delay(100);
    intake.move_voltage(0);

    chassis.moveToPoint(-60, -24, 5000, {.forwards=false});
    chassis.waitUntilDone();
    
    chassis.moveToPoint(-24,-24,5000, {.forwards=false, .maxSpeed=70});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(500);
    intake.move_voltage(-12000);
    
    chassis.moveToPoint(-24,-48,5000);
    chassis.waitUntilDone();
    pros::delay(1000);
    intake.move_voltage(0);
    clampPistons.set_value(false);
    chassis.moveToPoint(-24,5,5000);
}

void redElim(){
       chassis.setPose(-54, 24, 270);
    chassis.moveToPoint(-23, 24, 5000, {.forwards = false});
    chassis.waitUntil(24);
    clampPistons.set_value(true);
    pros::delay(500);
    intake.move_voltage(-12000);
    pros::delay(500);
    // intake.move_voltage(12000);
    chassis.moveToPoint(-24, 47, 5000);
    chassis.waitUntilDone();
    // intake.move_voltage(-12000);
    pros::delay(500);
    chassis.moveToPoint(-12, 43, 5000);
    chassis.waitUntilDone();
    pros::delay(700);
    // intake.move_voltage(0);
    // chassis.moveToPose(39,39, 45, 10000, {.forwards = false,.maxSpeed=60});
    // chassis.waitUntilDone();
    // intake.move_voltage(-12000);
    // clampPistons.set_value(false);
    chassis.moveToPoint(-24, 47, 5000, {.forwards = false});
    chassis.waitUntilDone();
    // intake.move_voltage(0);
    chassis.moveToPoint(-10, 50, 5000);
    pros::delay(500);
    chassis.moveToPoint(-24, 47, 5000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(54, 60, 5000);
    chassis.waitUntilDone();
}



/**************************************/
/*                                    */
/*                                    */
/*                                    */
/*               Skills               */
/*                                    */
/*                                    */
/*                                    */
/***************************************/

void skills2(){
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //alliance wall stake
    chassis.setPose(-62,0,90);
    intake.move_voltage(-12000);
    pros::delay(500);
    intake.move_voltage(12000);
    pros::delay(100);
    intake.move_voltage(0);

    //move forward
    chassis.moveToPoint(-48, 0, 1000);
    chassis.waitUntilDone();

    //turn to goal
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();

    //pick up goal
    chassis.moveToPoint(-48,20,1500,{.forwards=false});
    chassis.waitUntilDone();

    //clamp
    pros::delay(400);
    clampPistons.set_value(true);
    pros::delay(200);
    intake.move_voltage(-12000);

    //ring 1
    chassis.moveToPoint(-24,24,1500);
    chassis.waitUntilDone();
    pros::delay(300);

    //ring near wall stake
    chassis.moveToPoint(0,60,2000, {.maxSpeed=80});
    chassis.waitUntilDone();
    pros::delay(300);

    //line of 3 rings
    chassis.moveToPoint(-24,48,1000);
    chassis.waitUntilDone();
    pros::delay(200);

    chassis.moveToPoint(-48,48,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(100);

    chassis.moveToPoint(-60,48,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(100);

    //last ring
    chassis.moveToPoint(-46,60,1000);
    chassis.waitUntilDone();
    pros::delay(100);
    
    //corner
    chassis.moveToPose(-64,64,135,1000,{.forwards=false}); //
    chassis.waitUntilDone();
    intake.move_voltage(12000);
    pros::delay(200);
    clampPistons.set_value(false);
    pros::delay(200);
    intake.move_voltage(0);

    //Quadrant 2

    //move outwards
    chassis.moveToPoint(-48,0,3000,{.maxSpeed=80});
    chassis.waitUntilDone();
    //turn to goal
    chassis.turnToHeading(0,1500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-50,-22,1000,{.forwards=false}); //
    chassis.waitUntilDone();
    //clamp goal
    pros::delay(400);
    clampPistons.set_value(true);
    pros::delay(200);
    intake.move_voltage(-12000);

    //ring 1
    chassis.moveToPoint(-24,-24,1500);
    chassis.waitUntilDone();
    pros::delay(300);

    //ring near wall stake
    chassis.moveToPoint(0,-60,2000, {.maxSpeed=80});
    chassis.waitUntilDone();
    pros::delay(300);

    //line of 3 rings
    chassis.moveToPoint(-24,-48,1000);
    chassis.waitUntilDone();
    pros::delay(200);

    chassis.moveToPoint(-48,-48,2000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(100);

    chassis.moveToPoint(-60,-48,1000,{.maxSpeed=60});
    chassis.waitUntilDone();
    pros::delay(100);

    //last ring
    chassis.moveToPoint(-46,-60,1000);
    chassis.waitUntilDone();
    pros::delay(300);
    
    //corner
    chassis.moveToPose(-64,-64,45,1000,{.forwards=false});
    chassis.waitUntilDone();
    intake.move_voltage(12000);
    pros::delay(200);
    clampPistons.set_value(false);
    pros::delay(200);
    intake.move_voltage(0);

    // Quadrant 3

    //move to center
    chassis.moveToPoint(0, -48, 2000);
    chassis.waitUntilDone();
    //pick up ring
    intake.move_voltage(-12000);
    chassis.moveToPoint(27,-19,1000);//
    chassis.waitUntilDone();
    pros::delay(200);
    intake.move_voltage(0);
    //go to goal
    chassis.turnToHeading(225,1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(50,4,1500,{.forwards=false});
    chassis.waitUntilDone();
    pros::delay(400);
    clampPistons.set_value(true);
    pros::delay(200);
    intake.move_voltage(-12000);
    //rings
    chassis.moveToPoint(24,-48,1000);
    chassis.waitUntilDone();
    pros::delay(300);

    chassis.moveToPoint(48,-44,1000);//
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.turnToHeading(180,1000);
    chassis.waitUntilDone();

    chassis.moveToPoint(48,-60,1000);
    chassis.waitUntilDone();
    pros::delay(300);
    //corner
    chassis.turnToHeading(300,1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(64,-64,1000,{.forwards=false});
    chassis.waitUntilDone();
    intake.move_voltage(12000);
    pros::delay(100);
    clampPistons.set_value(false);
    pros::delay(300);
    intake.move_voltage(0);

    // Quadrant 4
    chassis.moveToPoint(29,-23,1500);
    chassis.waitUntilDone();
    //goal
    chassis.moveToPoint(60,24,1500,{.forwards=false});
    chassis.waitUntilDone();
    pros::delay(400);
    clampPistons.set_value(true);
    pros::delay(200);
    //corner
    chassis.moveToPoint(66,66,2000,{.forwards=false});
    chassis.waitUntilDone();
    clampPistons.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(53,46,2000);
}


ASSET(topLeftSkills_txt)
void skillsPPtest(){
    chassis.setPose(-60, 0, 90);
    intake.move_voltage(-12000);
    pros::delay(500);
    intake.move_voltage(0);

    chassis.moveToPoint(-50,20, 3000, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(300);

    intake.move_voltage(-12000);
    chassis.follow(topLeftSkills_txt, 10, 10000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-56,56,2000, {.forwards=false,.maxSpeed=60});

}



void test (){
    chassis.setPose(-59, 0, 90);
    chassis.moveToPoint(-32,0,1000);
    chassis.moveToPoint(-59, 0, 1000);
    intake.move_voltage(-12000);
    pros::delay(500);
    intake.move_voltage(0);


}