#include "main.h"
#include "lemlib/api.hpp"
#include "externs.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // computePossibleLocation();
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {clampPistons.set_value(false);}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
     pros::Task intakeWithSort([&](){
        int degsToReverse = 170;
        int inRotationStart = 0;
        static int redMin = 9;
        static int redMax = 18;
        static int blueMin = 190;
        static int blueMax = 250;
        bool hasSeenRejection = false;
        optical.set_led_pwm(100);

        while(true){
            if(inState == intakeState::IN){
                intake.move_voltage(12000);
                //colour sortign

                if(color == alliance::RED){
                    if(optical.get_hue()>blueMin&& optical.get_hue()<blueMax && !hasSeenRejection ){
                        intake.tare_position();
                        inRotationStart = intake.get_position();
                        hasSeenRejection = true;

                        pros::lcd::print(4, "Proximty %ld", optical.get_proximity()); // heading



                    }
                    if(hasSeenRejection && optical.get_proximity()<=210){
                        intake.move_voltage(-12000);
                        pros::delay(90);
                        hasSeenRejection = false;
                    }
                }
                else if(color == alliance::BLUE){
                    
                    if(optical.get_hue()>redMin&& optical.get_hue()<redMax && !hasSeenRejection ){
                        intake.tare_position();
                        inRotationStart = intake.get_position();
                        hasSeenRejection = true;

                        pros::lcd::print(4, "seen red"); // heading



                    }
                    if(hasSeenRejection && optical.get_proximity()<=210){
                        intake.move_voltage(-12000);
                        pros::delay(90);
                        hasSeenRejection = false;
                    }
                }





            } else if(inState == intakeState::OUT){
                intake.move_voltage(-12000);
            } else {
                intake.move_voltage(0);
                intake.brake();
                intake.tare_position();
                
            }
            pros::delay(5);
        }

    });
    skills9();
    // testMCL();
    // chassis.setPose(0,0,0);
    // chassis.turnToHeading(180, 1000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 1000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0,1000);
    // chassis.waitUntilDone();
    
    // chassis.moveToPoint(0,24,5000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0,5000,{.forwards = false});
    // chassis.waitUntilDone();


    // //PID TUNE AUTON
    // chassis.setPose(-60, 0, 90);
    // chassis.moveToPoint(-36, 0, 5000);
    // chassis.waitUntilDone();
    // //turn right
    // chassis.turnToHeading(180, 4000);
    // chassis.waitUntilDone();
    // //move forwards
    // chassis.moveToPoint(-36, -48, 5000);
    // chassis.waitUntilDone();
    // //move forwards when turning
    // chassis.turnToHeading(90, 1000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(48, -48, 5000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(315, 4000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0,5000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-48, 48, 5000);
    // chassis.waitUntilDone();

    // chassis.setPose(0,0,0);
    // chassis.turnToHeading(180, 100000);

}

float computeLiftError(float desiredAngle, float settleError = 50){
    static float error = 0;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    if(fabs(error) <= settleError){
        error = 0;
    }
    return error;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // chassis.setPose(-60, 0, 90);
    // testMCL();
	// loop forever
    double time = pros::millis();
	bool clamp = false;
	bool descoring = false;
    int i = 0;
	// ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // wsr.reset();

    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    bool manuealLift = false;
    int liftLastUsedTime = 0;
    int startTime=0;
    static int maxTime = 300;

    bool allowPTO = false;
    bool PTO=false;
    double ptoTime = 85000;

    while (true) {
        
        if(controller.get_digital(DIGITAL_A)){
            controller.rumble("-");
            PTO = !PTO;
            pto.set_value(PTO);
        }

        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if(controller.get_digital(DIGITAL_R1)){
    	    intake.move_voltage(12000);
        } else if (controller.get_digital(DIGITAL_R2)){
            intake.move_voltage(-12000);
        } else {
            intake.brake();
		}

		if(controller.get_digital_new_press(DIGITAL_L1)){
			clamp = !clamp;
			clampPistons.set_value(clamp);
		}
		if(controller.get_digital_new_press(DIGITAL_UP)){
			descoring = !descoring;
			descore.set_value(descoring);
		}

        if(controller.get_digital_new_press(DIGITAL_L2)){
            liftpid.reset();
            i++;
            if(i == 5){
                i = 2;
            } else if( i > 2 ){
                i = 0;
            }
        }

        if(controller.get_digital_new_press(DIGITAL_DOWN)){
            i = 3;
        }
        if(controller.get_digital_new_press(DIGITAL_RIGHT)){
            i = 4;
        }

        // switch(i){
        //     case 0:
        //     ws.move_voltage(-liftpid.update(computeLiftError(700, 10)));
        //     break;
        //     case 1:
        //     ws.move_voltage(-liftpid.update(computeLiftError(2400)));
        //     break;
        //     case 2:
        //     ws.move_voltage(-liftpid.update(computeLiftError(15500)));
        //     break;
        //     case 3:
        //     ws.move_voltage(-liftpid.update(computeLiftError(21000)));
        //     break;
        //     case 4:
        //     ws.move_voltage(-liftpid.update(computeLiftError(5000)));
        // }

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}