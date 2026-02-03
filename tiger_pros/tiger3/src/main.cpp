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
pros::v5::MotorGears drivetrainMotorsRatio = pros::MotorGearset::blue;;
double maxRPM = 200.0;
double aux_speed = 200.0;
int midWheelTrackWidth = 11;
int driveTrainRpm = 200;
int horizontalDrift = 2;

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

signed char intakeRoller = 8;
signed char roller1AndRoller2Motor = 1;
signed char bazookaMotor = 2;
signed char roller3Motor = 10;

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
pros::controller_digital_e_t loaderPistonMechButton = pros::E_CONTROLLER_DIGITAL_DOWN;
pros::controller_digital_e_t intakeToBackRollerButton = pros::E_CONTROLLER_DIGITAL_L1;
pros::controller_digital_e_t intakeToBazookaRollerButton = pros::E_CONTROLLER_DIGITAL_L2;
pros::controller_digital_e_t intakeOnlyButton = pros::E_CONTROLLER_DIGITAL_R2;
pros::controller_digital_e_t ejectButton = pros::E_CONTROLLER_DIGITAL_B;

// Inertial Sensor
pros::Imu imu(inertialSensorPort);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    // Robot configuration
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {

}

/**
 * Runs during auto
 */
void autonomous() { 
    /* 
    // CHANGE THIS BASED ON ALLIANCE:
    // true = blue alliance, false = red alliance
    bool isBlue = false;
    
    if (isBlue == true) {
        // BLUE ALLIANCE - Bottom left start
        chassis.setPose(-58.701, -48.425, 269.029);
        
        // forward(57.413, 30)
        chassis.moveToPoint(-58.701, -48.425 + 57.413, 3000);
        
        // turnTo(222.466, 30)
        chassis.turnToHeading(222.466, 2000);
        
        // forward(21.857, 30)
        chassis.moveToPoint(-58.701 - 15, -48.425 + 57.413 - 15, 3000);
        
        // turnTo(318.24, 30)
        chassis.turnToHeading(318.24, 2000);
        
        // forward(20.516, 30)
        chassis.moveToPoint(-58.701 - 15 + 14, -48.425 + 57.413 - 15 + 14, 3000);
        
        // turnTo(269.029, 30)
        chassis.turnToHeading(269.029, 2000);
        
        // backward(32.252, 30)
        chassis.moveToPoint(-58.701 - 15 + 14 - 32.252, -48.425 + 57.413 - 15 + 14, 3000, {.forwards = false});
        
        // turnTo(0, 30)
        chassis.turnToHeading(0, 2000);
        
    } else {
        // RED ALLIANCE - Bottom right start (mirrored)
        chassis.setPose(58.701, -48.425, 360 - 269.029);
        
        // forward(57.413, 30) - mirrored
        chassis.moveToPoint(58.701, -48.425 + 57.413, 3000);
        
        // turnTo(222.466, 30) - mirrored
        chassis.turnToHeading(360 - 222.466, 2000);
        
        // forward(21.857, 30) - mirrored
        chassis.moveToPoint(58.701 + 15, -48.425 + 57.413 - 15, 3000);
        
        // turnTo(318.24, 30) - mirrored
        chassis.turnToHeading(360 - 318.24, 2000);
        
        // forward(20.516, 30) - mirrored
        chassis.moveToPoint(58.701 + 15 - 14, -48.425 + 57.413 - 15 + 14, 3000);
        
        // turnTo(269.029, 30) - mirrored
        chassis.turnToHeading(360 - 269.029, 2000);
        
        // backward(32.252, 30) - mirrored
        chassis.moveToPoint(58.701 + 15 - 14 + 32.252, -48.425 + 57.413 - 15 + 14, 3000, {.forwards = false});
        
        // turnTo(0, 30)
        chassis.turnToHeading(0, 2000);
    }*/
    //tuning pid 
    //chassis.setPose(0, 0, 0);
    //chassis.moveToPoint(0, 15, 999999);
    
    /*
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(180, 999999);
    */
   
   

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // vertical tracking wheel encoder. Rotation sensor, port 23, reversed
    pros::Rotation verticalEnc(15);

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
    
    chassis.calibrate(); // calibrate sensors

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

    // Piston state tracking
    bool piston_state = false;
    bool last_d_pressed = false;
    bool last_x_pressed = false;

    pros::Motor topChainMotor(roller1AndRoller2Motor, pros::MotorGearset::green);
    pros::Motor intakeMotorFront(intakeRoller, pros::MotorGearset::green);
    pros::Motor intakeMotor(bazookaMotor, pros::MotorGearset::green);
    pros::Motor upperRollerMotor(roller3Motor, pros::MotorGearset::green);
    pros::Motor upperBackFlexWheelMotor(upperBackFlexWheelPort, pros::MotorGearset::green);
    


    while (true) {

        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 

        chassis.arcade(rightX, leftY);  

        if (controller.get_digital(intakeToBackRollerButton)) {
            topChainMotor.move_velocity(aux_speed);
            intakeMotorFront.move_velocity(0);
            intakeMotor.move_velocity(-aux_speed);
            upperRollerMotor.move_velocity(aux_speed);
            upperBackFlexWheelMotor.move_velocity(aux_speed);
        } 
        else if (controller.get_digital(intakeToBazookaRollerButton)) {
            topChainMotor.move_velocity(aux_speed);
            intakeMotorFront.move_velocity(-aux_speed);
            intakeMotor.move_velocity(-aux_speed);
            upperRollerMotor.move_velocity(-aux_speed);
        } 
		else if (controller.get_digital(ejectButton)) {
            topChainMotor.move_velocity(-aux_speed);
            intakeMotorFront.move_velocity(aux_speed);
            intakeMotor.move_velocity(aux_speed);
            upperRollerMotor.move_velocity(aux_speed);
        } 
        else if (controller.get_digital(intakeOnlyButton)) {
            topChainMotor.move_velocity(0);
            intakeMotorFront.move_velocity(-aux_speed);
            intakeMotor.move_velocity(0);
            upperRollerMotor.move_velocity(0);
        }
        else {
            topChainMotor.move_velocity(0);
            intakeMotorFront.move_velocity(0);
            intakeMotor.move_velocity(0);
            upperRollerMotor.move_velocity(0);
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
//kp=12.,kd+36
//angular kp2.35,kd20