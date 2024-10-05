#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-13, -12, 11}, pros::MotorGearset::blue);    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({16, 15, -14}, pros::MotorGearset::blue);
    pros::Motor intake(17, pros::MotorGearset::blue);

pros::ADIDigitalOut clampPistons ('H');
//trackwidth 29cm = ~11.417in
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.417, lemlib::Omniwheel::NEW_325, 450, 2);


//odom
// pros::Imu imu(19);
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
                            nullptr // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
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
    //chassis.turnToHeading(90, 100000);
    chassis.follow(path1_txt, 4000, 15);

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

        bool clamp = false;
        intake.set_voltage_limit(12000);
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
            // get joystick positions
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            // move the chassis with curvature drive
            chassis.arcade(leftY, rightX);
            // delay to save resources
            pros::delay(10);
        }
    }