#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ------------------------------------------------------------ //
// Robot Configuration                                          //
// ------------------------------------------------------------ //
pros::v5::MotorGears drivetrainMotorsRatio = pros::MotorGearset::blue; // 600 rpm
double maxRPM = 200.0;
double aux_speed = 200.0;
int midWheelTrackWidth = 11;
int driveTrainRpm = 200;
int horizontalDrift = 2;

// ports definition
signed char frontRightUpMotorPort = 18;
signed char frontRightDownMotorPort = -17;
signed char backRightUpMotorPort = 20;
signed char backRightDownMotorPort = -19;

signed char frontLeftUpMotorPort = 13;
signed char frontLeftDownMotorPort = -14;
signed char backLeftUpMotorPort = 11;
signed char backLeftDownMotorPort = -12;
signed char upperBackFlexWheelPort = 8;

signed char inertialSensorPort = 16;

signed char intakeRollerPort = 9;
signed char roller1MotorPort = 1;
signed char bazookaMotorPort = 2;
signed char roller2MotorPort = 10;

signed char rotationSensorPort = 15;

pros::MotorGroup leftMotorsGroup = pros::MotorGroup({
        frontLeftUpMotorPort, 
        frontLeftDownMotorPort, 
        backLeftUpMotorPort, 
        backLeftDownMotorPort
    }, drivetrainMotorsRatio);

pros::MotorGroup rightMotorsGroup = pros::MotorGroup({
        frontRightUpMotorPort, 
        frontRightDownMotorPort, 
        backRightUpMotorPort, 
        backRightDownMotorPort
    }, drivetrainMotorsRatio);;

pros::adi::DigitalOut pistonBazookaMech =  pros::adi::DigitalOut('H');
pros::adi::DigitalOut pistonLoaderMech = pros::adi::DigitalOut('G');

pros::controller_digital_e_t bazookaPistonMechButton = pros::E_CONTROLLER_DIGITAL_R1;
pros::controller_digital_e_t loaderPistonMechButton = pros::E_CONTROLLER_DIGITAL_L1;
pros::controller_digital_e_t intakeToBackRollerButton = pros::E_CONTROLLER_DIGITAL_A;
pros::controller_digital_e_t intakeToBazookaRollerButton = pros::E_CONTROLLER_DIGITAL_R2;
pros::controller_digital_e_t ejectButton = pros::E_CONTROLLER_DIGITAL_L2;

// Inertial Sensor
pros::Imu imu(inertialSensorPort);

// vertical tracking wheel encoder
pros::Rotation verticalEnc(rotationSensorPort);

pros::Motor roller1Motor(roller1MotorPort, pros::MotorGearset::green);
pros::Motor intakeMotorFront(intakeRollerPort, pros::MotorGearset::green);
pros::Motor bazookaMotor(bazookaMotorPort, pros::MotorGearset::green);
pros::Motor roller2Motor(roller2MotorPort, pros::MotorGearset::green);
pros::Motor upperBackFlexWheelMotor(upperBackFlexWheelPort, pros::MotorGearset::green);

// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotorsGroup,
                        &rightMotorsGroup, 
                        midWheelTrackWidth,
                        lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                        driveTrainRpm,
                        horizontalDrift // horizontal drift is 2 if using tracking wheels if not use 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in degrees
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in degrees
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                    10, // minimum output where drivetrain will move out of 127
                                    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                10, // minimum output where drivetrain will move out of 127
                                1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();     // calibrate sensorss

    pros::Task screenTask([&]()
                          {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        } });

}

void disabled() {}

void competition_initialize() {

}


ASSET(example_txt);

void autonomous() { 
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 10, 999999);
}


void opcontrol() {
    // Piston state tracking
    bool piston_state = false;
    bool last_d_pressed = false;
    bool last_x_pressed = false;

    while (true) {

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 

        chassis.arcade(rightX, leftY);  

        if (controller.get_digital(intakeToBackRollerButton)) {
            roller1Motor.move_velocity(aux_speed);
            intakeMotorFront.move_velocity(-aux_speed);
            bazookaMotor.move_velocity(-aux_speed);
            roller2Motor.move_velocity(aux_speed);
            upperBackFlexWheelMotor.move_velocity(aux_speed);
        } 
        else if (controller.get_digital(intakeToBazookaRollerButton)) {
            roller1Motor.move_velocity(aux_speed);
            intakeMotorFront.move_velocity(-aux_speed);
            bazookaMotor.move_velocity(-aux_speed);
            roller2Motor.move_velocity(-aux_speed);
        } 
		else if (controller.get_digital(ejectButton)) {
            roller1Motor.move_velocity(-aux_speed);
            intakeMotorFront.move_velocity(aux_speed);
            bazookaMotor.move_velocity(aux_speed);
            roller2Motor.move_velocity(aux_speed);
        } 
        else {
            roller1Motor.move_velocity(0);
            intakeMotorFront.move_velocity(0);
            bazookaMotor.move_velocity(0);
            roller2Motor.move_velocity(0);
            upperBackFlexWheelMotor.move_velocity(0);
        }
        
        if (controller.get_digital(loaderPistonMechButton)) {
            if (!last_d_pressed) {
                piston_state = !piston_state;
                pistonLoaderMech.set_value(piston_state);
                last_d_pressed = true;
            }
        } else {
            last_d_pressed = false;
        }
        
        if (controller.get_digital(bazookaPistonMechButton)) {
            if (!last_x_pressed) {
                piston_state = !piston_state;
                pistonBazookaMech.set_value(piston_state);
                last_x_pressed = true;
            }
        } else {
            last_x_pressed = false;
        }
        
        // delay to save resources
        pros::delay(10);
    }
}