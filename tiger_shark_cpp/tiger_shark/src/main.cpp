/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:                                                                 */
/*    Created:      1/23/2026, 11:04:35 AM                                    */
/*    Description:  Tiger Shark VEX V5 Project                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller(primary);

// ------------------------------------------------------------ //
// TigerShark Configuration - SELECT FIRST BEFORE UPLOADING     //
const int currentTigerIndex = 0;                                //
// ------------------------------------------------------------ //


class TigerShark {
  public:
  vex::gearSetting drivetrainMotorsRatio;
  double maxRPM;
  double aux_speed;

  vex::motor frontRightUpMotor;
  vex::motor frontRightDownMotor;
  vex::motor backRightUpMotor;
  vex::motor backRightDownMotor;

  vex::motor frontLeftUpMotor;
  vex::motor frontLeftDownMotor;
  vex::motor backLeftUpMotor;
  vex::motor backLeftDownMotor;

  vex::motor intakeAndRoller1AndRoller2Motor;
  vex::motor bazookaMotor;
  vex::motor roller3Motor;

  vex::motor_group leftGears;
  vex::motor_group rightGears;

  vex::digital_out* pistonBazookaMech;
  vex::digital_out* pistonLoaderMech;


  void initShark(vex::gearSetting motorRatio){

    frontRightUpMotor = motor(PORT18, motorRatio, false);
    frontRightDownMotor = motor(PORT17, motorRatio, true);
    backRightUpMotor = motor(PORT20, motorRatio, false);
    backRightDownMotor = motor(PORT19, motorRatio, true);

    frontLeftUpMotor = motor(PORT13, motorRatio, false);
    frontLeftDownMotor = motor(PORT14, motorRatio, true);
    backLeftUpMotor = motor(PORT11, motorRatio, false);
    backLeftDownMotor = motor(PORT12, motorRatio, true);

    intakeAndRoller1AndRoller2Motor = motor(PORT1, ratio18_1, false);
    bazookaMotor = motor(PORT2, ratio18_1, true);
    roller3Motor = motor(PORT10, ratio18_1, false);

    // Motor Groups
    leftGears = motor_group(frontRightUpMotor, frontRightDownMotor, backRightUpMotor, backRightDownMotor);
    rightGears = motor_group(frontLeftUpMotor, frontLeftDownMotor, backLeftUpMotor, backLeftDownMotor);

    // Pneumatics (3-Wire Ports)
    pistonBazookaMech = new digital_out(Brain.ThreeWirePort.H);
    pistonLoaderMech = new digital_out(Brain.ThreeWirePort.G);
  }
};

TigerShark tigerShark[4]{};

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton() {
  // All activities that occur before the competition starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("Pre-Autonomous");
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous() {
  // Autonomous code here
  wait(20, msec);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol() {
  TigerShark& currentTiger = tigerShark[currentTigerIndex];
  Brain.Screen.clearScreen();
  Brain.Screen.print("Driver Control");

  bool piston_loader_state = false;
  bool piston_bazooka_state = false;
  bool last_x_pressed = false;
  bool last_d_pressed = false;

  // User control code here
  while (1) {
    // DRIVE TRAIN CONTROL

    // Axis 3 is Forward/Backward, Axis 1 is Turning
    double forwardBackward = (Controller1.Axis3.position() / 100.0) * currentTiger.maxRPM;
    double turning = (Controller1.Axis1.position() / 100.0) * currentTiger.maxRPM;

    // Calculate motor speeds (arcade drive)
    double rightSpeed = forwardBackward + turning;
    double leftSpeed = forwardBackward - turning;

    // Spin motors
    currentTiger.rightGears.spin(forward, rightSpeed, rpm);
    currentTiger.leftGears.spin(forward, leftSpeed, rpm);

    // --- PISTON TOGGLE LOGIC ---
    // Button Down toggles piston2
    if (Controller1.ButtonDown.pressing()) {
      if (!last_d_pressed) {
        piston_loader_state = !piston_loader_state;
        currentTiger.pistonLoaderMech->set(piston_loader_state);
        last_d_pressed = true;
      }
    } else {
      last_d_pressed = false;
    }

    // Button X toggles piston
    if (Controller1.ButtonX.pressing()) {
      if (!last_x_pressed) {
        piston_bazooka_state = !piston_bazooka_state;
        currentTiger.pistonBazookaMech->set(piston_bazooka_state);
        last_x_pressed = true;
      }
    } else {
      last_x_pressed = false;
    }

    // *** AUXILIARY MOTOR CONTROL ***
    if (Controller1.ButtonA.pressing()) {
      currentTiger.roller3Motor.spin(reverse, currentTiger.aux_speed, rpm);
      currentTiger.intakeAndRoller1AndRoller2Motor.spin(reverse, currentTiger.aux_speed, rpm);
      currentTiger.bazookaMotor.spin(reverse, currentTiger.aux_speed, rpm);
    } else if (Controller1.ButtonB.pressing()) {
      currentTiger.roller3Motor.spin(forward, currentTiger.aux_speed, rpm);
      currentTiger.intakeAndRoller1AndRoller2Motor.spin(forward, currentTiger.aux_speed, rpm);
      currentTiger.bazookaMotor.spin(forward, currentTiger.aux_speed, rpm);
    } else if (Controller1.ButtonY.pressing()) {
      currentTiger.roller3Motor.spin(reverse, currentTiger.aux_speed, rpm);
      currentTiger.intakeAndRoller1AndRoller2Motor.spin(forward, currentTiger.aux_speed, rpm);
      currentTiger.bazookaMotor.spin(forward, currentTiger.aux_speed, rpm);
    } else {
      currentTiger.roller3Motor.stop();
      currentTiger.intakeAndRoller1AndRoller2Motor.stop();
      currentTiger.bazookaMotor.stop();
    }

    // Display motor diagnostics
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Right Speed: %.2f RPM", currentTiger.rightGears.velocity(rpm));
    Brain.Screen.setCursor(1, 20);

    displayDiagnostics(currentTiger);

    wait(100, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // fixed config for each tiger
  // tiger 1 config (Blue and White)
  tigerShark[0].drivetrainMotorsRatio = ratio18_1;
  tigerShark[0].maxRPM = 600.0;
  tigerShark[0].aux_speed = 200.0;

  // tiger 2 config (Black and Gold)
  tigerShark[1].drivetrainMotorsRatio = ratio18_1;
  tigerShark[1].maxRPM = 600.0;
  tigerShark[1].aux_speed = 200.0;

  // tiger 3 config (Red and White)
  tigerShark[2].drivetrainMotorsRatio = ratio6_1;
  tigerShark[2].maxRPM = 600.0;
  tigerShark[2].aux_speed = 200.0;

  // tiger 4 config (Blue and Purple)
  tigerShark[3].drivetrainMotorsRatio = ratio6_1;
  tigerShark[3].maxRPM = 600.0;
  tigerShark[3].aux_speed = 200.0;


  tigerShark[currentTigerIndex].initShark(tigerShark[currentTigerIndex].drivetrainMotorsRatio);


  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

void displayDiagnostics(TigerShark& currentTiger) {
int row = 2;
    
    // Right side motors
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FR Up Temp: %.1fC", currentTiger.frontRightUpMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FR Up Current: %.2fA", currentTiger.frontRightUpMotor.current(amp));
    
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FR Down Temp: %.1fC", currentTiger.frontRightDownMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FR Down Current: %.2fA", currentTiger.frontRightDownMotor.current(amp));
    
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BR Up Temp: %.1fC", currentTiger.backRightUpMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BR Up Current: %.2fA", currentTiger.backRightUpMotor.current(amp));
    
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BR Down Temp: %.1fC", currentTiger.backRightDownMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BR Down Current: %.2fA", currentTiger.backRightDownMotor.current(amp));

    // Left side motors
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FL Up Temp: %.1fC", currentTiger.frontLeftUpMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FL Up Current: %.2fA", currentTiger.frontLeftUpMotor.current(amp));
    
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FL Down Temp: %.1fC", currentTiger.frontLeftDownMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("FL Down Current: %.2fA", currentTiger.frontLeftDownMotor.current(amp));
    
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BL Up Temp: %.1fC", currentTiger.backLeftUpMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BL Up Current: %.2fA", currentTiger.backLeftUpMotor.current(amp));
    
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BL Down Temp: %.1fC", currentTiger.backLeftDownMotor.temperature(celsius));
    Brain.Screen.setCursor(row++, 1);
    Brain.Screen.print("BL Down Current: %.2fA", currentTiger.backLeftDownMotor.current(amp));
}