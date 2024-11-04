#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
//Auto selector LIB
#include "robodash/api.h"
//github 
//https://github.com/unwieldycat/robodash/blob/main/src/main.cpp


pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-2, -1, -4}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({3, 14, 15}, pros::MotorGearset::blue);
pros::Motor intake(10, pros::MotorGearset::blue);
pros::Motor ws(11, pros::MotorGearset::red);

pros::Rotation wsr(19);

pros::ADIDigitalOut clampPistons ('H');
//trackwidth 29cm = ~11.417in
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.417, lemlib::Omniwheel::NEW_325, 450, 2);

/**
 * @brief Lady brown mech states
 * 
 * @details
 * Prime: Load state  
 * Prelim: Initial state before scoring  
 * Score: Scoring state
 */
enum LBPos {
    LB_Prelim,  
    LB_Prime,   
    LB_Score    
};

//odom
pros::Imu imu(20);
// // pros::Rotation rv(7); //vert
// pros::Rotation rh(-6); //horiz set to negative if reverse

// lemlib::TrackingWheel htw(&rh, lemlib::Omniwheel::NEW_275, 2.125); // third value is tracking center offset
// lemlib::TrackingWheel vtw(&rv, 3.25, 5.25);

// lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//                             &htw, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(4.4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.75, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.4, // proportional gain (kP)
                                              0.01, // integral gain (kI)
                                              15.25, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(10, // joystick deadband out of 127
                                     15, // minimum output where drivetrain will move out of 127
                                     1.03 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(10, // joystick deadband out of 127
                                  15, // minimum output where drivetrain will move out of 127
                                  1.03 // expo curve gain
);


lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
						&throttle_curve,
						&steer_curve // odometry sensors
);

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
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
    chassis.calibrate();
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
    //     }
    // });
	// while (true){
    //     pros::lcd::print(1, "Rotation Sensor H: %i", rh.get_position());
	// 	//pros::lcd::print(0, "Rotation Sensor V: %i", rh.get_position());
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
ASSET(path1_txt);

void autonomous() {
	chassis.setPose(0,0,0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
    // chassis.follow(path1_txt, 10, 4000);

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
void antiTipDrive(int left, int right, double changeRate, int maxChange, float deadband, float minOutput, float curve){
    static double initTime = pros::millis();
    static int prevLeftPower = 0;
    static int prevRightPower = 0;

    int leftPower = (driveCurve(left, deadband, minOutput,curve) + driveCurve(right, deadband, minOutput,curve)) * 12000 / 127;
    int rightPower = (driveCurve(right, deadband, minOutput,curve) - driveCurve(left, deadband, minOutput,curve)) * 12000 / 127;


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


void LBController(LBPos pos){

}

float prevError = 0;
float error = 0;
float voltScalar = 1.5;
float integral = 0;

void liftPID(float desiredAngle, float kP, float kI, float kD, float settleError = 250, float integralMax = 36000){
    //desired angle in centidegrees (0-36,000)
    //settle error 250 cdeg
    //idle = 0, primed = 26, scored = 140
    float derivative = 0;
    prevError = error;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    pros::lcd::set_text(1, std::to_string(error));
    pros::lcd::set_text(4, std::to_string(angle));
    

    integral += error;

    if(integral >= 36000){
        integral = 0;
    }

    derivative = error - prevError;

    float power = (error * kP + integral * kI + derivative * kD) * voltScalar;
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
float liftP = 3;
float liftI = 0;
float liftD = 1;

void opcontrol() {
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    bool clamp = false;
    intake.set_voltage_limit(12000);
    int i = 0;
    wsr.reset_position();
    
    // controller
    // loop to continuously update motors
    while (true) {
        if(controller.get_digital_new_press(DIGITAL_DOWN)){
            if(!clamp){
                clamp = true;
                clampPistons.set_value(true);
            } else {
                clamp = false;
                clampPistons.set_value(false);
            }
        }

        if(controller.get_digital(DIGITAL_R2)){
            intake.move_voltage(12000);
        } else if (controller.get_digital(DIGITAL_R1)){
            intake.move_voltage(-12000);
        } else {
            intake.brake();
        }
        // if(controller.get_digital(DIGITAL_L2)){
        //     ws.move_voltage(12000);
        // } else if (controller.get_digital(DIGITAL_L1)){
        //     ws.move_voltage(-12000);
        // } else {
        //     ws.brake();
        // }

        if(controller.get_digital_new_press(DIGITAL_X)){
            i++;
            if( i > 2 ){
                i = 0;
            }
        }
        
        switch(i){
                case 0:
                liftPID(0, liftP, liftI, liftD, 250);
                break;
                case 1:
                liftPID(2600, liftP, liftI, liftD, 250);
                break;
                case 2:
                liftPID(14500, liftP, liftI, liftD, 250);
            }
            pros::lcd::set_text(2, std::to_string(i));

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        antiTipDrive(rightX, leftY, 500, 900, 15, 20, 1.03);
        // delay to save resources
        pros::delay(10);
    }
}