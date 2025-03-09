#include "main.h"
#include "lemlib/api.hpp"
#include "externs.hpp"
#include "mcl.cpp"

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
    MonteCarloLocalization mcl(chassis, 14);
    //right dist sensor
    mcl.addDistanceSensor(21, 5.9, 3.125, -90);
    //front dist sensor
    mcl.addDistanceSensor(16, -5.9, 3.125, 90);
    //left dist sensor
    mcl.addDistanceSensor(2, 4.5, 6.5, 0);
    pros::Task mcl_task([&]() {
        pros::delay(20);
    });
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

    while (true) {

        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if(controller.get_digital(DIGITAL_R1)){
    	    // intake.move_voltage(12000);
            inState = intakeState::IN;
        } else if (controller.get_digital(DIGITAL_R2)){
            // intake.move_voltage(-12000);
            inState = intakeState::OUT;
        } else {
            // intake.brake();
            inState = intakeState::STOP;
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

        switch(i){
            case 0:
            ws.move_voltage(-liftpid.update(computeLiftError(700, 10)));
            break;
            case 1:
            ws.move_voltage(-liftpid.update(computeLiftError(2600, 100)));
            break;
            case 2:
            ws.move_voltage(-liftpid.update(computeLiftError(19500)));
            break;
            case 3:
            ws.move_voltage(-liftpid.update(computeLiftError(21000)));
            break;
            case 4:
            ws.move_voltage(-liftpid.update(computeLiftError(6500)));
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            chassis.setPose(0,0,90);
            chassis.moveToPoint(chassis.getPose().x-7, chassis.getPose().y, 2000, {.forwards = false, .maxSpeed = 60});
            chassis.waitUntilDone();
            i = 3;
        }

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}