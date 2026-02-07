#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({11,-12,13,-14},
                            pros::MotorGearset::blue); // left motor group - ports 13, 14 (reversed), 11, 12 (reversed)
pros::MotorGroup rightMotors({-17,18,-19,20}, 
                             pros::MotorGearset::blue); // right motor group - ports 18, 17 (reversed), 20, 19 (reversed)

// Inertial Sensor on port 3
pros::Imu imu(16);

// tiger-shark intake motors
pros::Motor topChainMotor(-5, pros::MotorGearset::green);       // Port 1, normal
pros::Motor intakeMotor(-6, pros::MotorGearset::green);        // Port 16, normal



// PNEUMATIC PISTONS
pros::adi::DigitalOut pis1('H');  // Piston 1 on port H 
pros::adi::DigitalOut pis2('G');  // Piston 2 on port G 

// tracking wheels

// vertical tracking wheel encoder. Rotation sensor, port 23, reversed
pros::Rotation verticalEnc(15);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.125);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.15, //  mid wheels in inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                               350, // drivetrain rpm is 450
                              2 // horizontal drift is 2 if using tracking wheels if not use 8

);

// lateral motion controller
lemlib::ControllerSettings linearController(1.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors( &vertical, // vertical tracking wheel
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
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
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
ASSET(example_txt);

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
    //chassis.moveToPoint(2, 0, 2000);
    //chassis.moveToPoint(0, 2, 2000);
    //chassis.turnToHeading(180, 1000);
    
    /*
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(180, 999999);
    */

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // Piston state tracking
    bool piston_state = false;
    bool last_d_pressed = false;
    bool last_x_pressed = false;
    
    // Auxiliary motor speed (RPM)
    int aux_speed = 200; // Adjust this value as needed

    while (true) {

       int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   
int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 
chassis.arcade(rightX, leftY);  

       /*/ // AUXILIARY MOTOR CONTROL
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            // Button A - INTAKE MODE (pull rings in)
            topChainMotor.move_velocity(aux_speed);       // Forward
            intakeMotor.move_velocity(-aux_speed);        // Reverse
            
            upperOutakeMotor.move_velocity(aux_speed);    // Forward
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            // Button B - OUTTAKE MODE (push rings out)
            topChainMotor.move_velocity(-aux_speed);      // Reverse
            intakeMotor.move_velocity(aux_speed);         // Forward
          
            upperOutakeMotor.move_velocity(-aux_speed);   // Reverse
        } 
        else {
            // No button - STOP all auxiliary motors
            topChainMotor.move_velocity(0);
            intakeMotor.move_velocity(0);
            
            upperOutakeMotor.move_velocity(0);
        }
        
        // PISTON TOGGLE LOGIC
        
        // Button Down - Toggle pis2
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            if (!last_d_pressed) {
                piston_state = !piston_state;
                pis2.set_value(piston_state);
                last_d_pressed = true;
            }
        } else {
            last_d_pressed = false;
        }
        
        // Button X - Toggle pis1
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (!last_x_pressed) {
                piston_state = !piston_state;
                pis1.set_value(piston_state);
                last_x_pressed = true;
            }
        } else {
            last_x_pressed = false;
        }*/
        
        // delay to save resources
        pros::delay(10);
    }
}

/*motor groups 

pros::MotorGroup bigname({3,4,5});
pros::MotorGroup bignamel({9,10,11});

*/ 
/* motor
 pros::Motor intakone(12);
*/ 

//pneumatic/*pros::adi::Pneumatics myname('prot B',false); */