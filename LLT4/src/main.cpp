#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
//Auto selector LIB
#include "robodash/api.h"

//github 
//https://github.com/unwieldycat/robodash/blob/main/src/main.cpp

rd::Selector selector({
    //Blue or Red indicates team, + or - indicates corner side
    //S or R indicates safe or rush
    //#pt indicates points
    //#/3 indicates number of local WP tasks
    //#/4 indicates number of sig WP tasks
    {"Blue- S 5pt 2/3 1/4", &blueNegativeSafe},
    {"Blue+ S 4pt 1/3 1/4", &bluePositiveSafe},
    {"Blue+ S 6pt 2/3 1/4", &bluePositiveSafe2},
    {"Red- S 5pt 2/3 1/4", &redNegativeSafe},
    {"Red+ S 4pt 1/3 1/4", &redPositiveSafe},
    {"Red+ S 6pt 2/3 1/4", &redPositiveSafe2},
    {"Red Elims", &redElim},
    {"Skills", &skills2},
    {"skills Corners", &cornerSkills}
    {"Test Pure Pursuit Skills", &skillsPPtest}
});
rd::Console console;


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
    chassis.calibrate();

    selector.focus();

    // pros::Task screenTask([&]() {
    //     while (true) {
    //         // print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    //         // log position telemetry
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         // delay to save resources
    //         pros::delay(50);
    // }
    // });
	// while (true){
    //     pros::lcd::print(1, "Rotation Sensor H: %i", verticalRota.get_position());
	// 	pros::lcd::print(0, "Rotation Sensor V: %i", horizontalRota.get_position());
    //     pros::delay(10); // delay to save resources. DO NOT REMOVE
	// }
    
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
    // turn to face heading 90 with a very long timeout
    // blueNegtive();    // chassis.follow(path1_txt, 10, 4000);
    wsr.reset_position();
    selector.run_auton();
}

//copied from lemlib
float driveCurve(float input, float deadband, float minOutput, float curve) {
    // return 0 if input is within deadzone
    if (fabs(input) <= deadband) return 0;
    // g is the output of g(x) as defined in the Desmos graph
    const float g = fabs(input) - deadband;
    // g127 is the output of g(127) as defined in the Desmos graph
    const float g127 = 127 - deadband;
    // i is the output of i(x) as defined in the Desmos graph
    const float i = pow(curve, g - 127) * g * lemlib::sgn(input);
    // i127 is the output of i(127) as defined in the Desmos graph
    const float i127 = pow(curve, g127 - 127) * g127;
    return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * lemlib::sgn(input);
}

/** 
 * @brief
 * Arcade drive control
 * will function like normal drive code until max change is exeeded
 * after which accel of drive will be limited
 * 
 * @param left left stick data
 * @param right right stick control
 * @param changeRate maximum voltage change rate (per update cycle)
 * @param maxChange maximum change with no limiting or smoothing
 * @param deadBand drive curve setting
 * @param minOutput drive curve setting
 * @param curve drive curve setting

**/
void antiTipDrive(int left, int right, double changeRate, int maxChange, float deadband, float minOutput, float Rcurve, float Lcurve){
    static int prevLeftPower = 0;
    static int prevRightPower = 0;
    float leftC = driveCurve(left, deadband, minOutput,Lcurve);
    float rightC= driveCurve(right, deadband, minOutput,Rcurve);

    float leftPower = ( leftC + rightC) * 12000 / 127;
    float rightPower = (rightC - leftC) * 12000 / 127;


    int leftChange  = leftPower - prevLeftPower;
    int rightChange = rightPower - prevRightPower;

    if(abs(leftChange) >= maxChange){
         if (leftChange > 0) {
            leftPower = prevLeftPower + changeRate;  // accel limit
        } else {
            leftPower = prevLeftPower - changeRate;  // deccel limit
        }
    }

    if(abs(rightChange) >= maxChange){
         if (rightChange > 0) {
            rightPower = prevRightPower + changeRate;  // accel limit
        } else {
            rightPower = prevRightPower - changeRate;  // deccel limit
        }
    }

    //Update motor voltages
    left_mg.move_voltage(leftPower);
    right_mg.move_voltage(rightPower);

    //Update prev values
    prevLeftPower = leftPower;
    prevRightPower = rightPower;
    

}




/** 
 * @brief
 * Used to control the postion of the lady brown mechnisam on the robot with a PID control loop
 * 
 * @param desiredAngle the target angle in centidegrees
 * @param kP Proportional gain for PID 
 * @param kI Integral gain for the PID
 * @param kD Derivative gain for the PID 
 * @param settleError (Optional) Threshold for acceptable error to consider the target reached. Defaults to 250.
**/
void liftPID(float desiredAngle, float kP, float kD, float settleError = 250){
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
    
    // pros::lcd::set_text(1, std::to_string(error));
    // pros::lcd::set_text(4, std::to_string(angle));
    std::cout << error;
    std::cout << angle;



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
    pros::Task screenTask([&]() {
        color.set_led_pwm(100);
            while (true) {
                int redMin = 9;
                int redMax = 18;
                int blueMin = 190;
                int blueMax = 250;
                bool blue =true;
                while(true){
                    if(controller.get_digital(DIGITAL_R1)){
                        intake.move_voltage(-12000);
                        if(!blue){
                            if(color.get_hue()>blueMin&& color.get_hue()<blueMax){
                                pros::delay(75);
                                intake.move_voltage(12000);
                                pros::delay(80);
                                intake.move_voltage(-12000);
                                console.print("saw blue");

                            }
                        }
                        else{
                            if(color.get_hue()>redMin&& color.get_hue()<redMax){
                                pros::delay(75);
                                intake.move_voltage(12000);
                                pros::delay(80);
                                intake.move_voltage(-12000);
                                console.print("saw blue");

                            }
                        }

                    } else if (controller.get_digital(DIGITAL_R2)){
                        intake.move_voltage(12000);
                    } else {
                        intake.brake();
                    }
                    if(controller.get_digital_new_press(DIGITAL_DOWN)){
                        blue = !blue;
                    }

                }
                pros::delay(10);
        }
    });
    static float liftP = 3;
    static float liftD = 1;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    int i = 0;
    bool clamp = false;
    bool boinked = false;
    intake.set_voltage_limit(12000);
    wsr.set_position(0);
    
    // controller
    // loop to continuously update motors
    while (true) {
        if(controller.get_digital_new_press(DIGITAL_L1)){
            clamp = !clamp;
            clampPistons.set_value(clamp);
        }
        if(controller.get_digital_new_press(DIGITAL_RIGHT)){
            boinked = !boinked;
            boink.set_value(boinked);
        }

        if(controller.get_digital_new_press(DIGITAL_L2)){
            i++;
            if( i > 2 ){
                i = 0;
            }
        }
        
        switch(i){
                case 0:
                liftPID(0, liftP, liftD);
                break;
                case 1:
                liftPID(2500, liftP, liftD);
                break;
                case 2:
                liftPID(14500, liftP, liftD);
            }
            // pros::lcd::set_text(2, std::to_string(i));

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        antiTipDrive(rightX, leftY, 500, 900, 15, 20, 1.05, 1.09);
        // delay to save resources
        pros::delay(10);
    }
}