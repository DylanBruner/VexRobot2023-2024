/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2140C                                                     */
/*    Created:      9/18/2023, 7:31:18 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "better_motor_group.h"
#include "auton_selector.h"

using namespace vex;

brain       Brain;
controller  Controller1;

const int LEFT_FRONT = PORT7;
const int LEFT_MIDDLE = PORT21;
const int LEFT_BACK = PORT10;
const int RIGHT_FRONT = PORT8;
const int RIGHT_MIDDLE = PORT18;
const int RIGHT_BACK = PORT19;
const int CATA_PORT = PORT6;
const int WINCH_PORT = PORT4;

// Drive motors
int32_t rightPorts[] = {RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK};//20
int32_t leftPorts[] = {LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK};
// all blue motors
motor leftFront = motor(LEFT_FRONT, gearSetting::ratio6_1);
motor leftMiddle = motor(LEFT_MIDDLE, gearSetting::ratio6_1);
motor leftBack = motor(LEFT_BACK, gearSetting::ratio6_1);
motor rightFront = motor(RIGHT_FRONT, gearSetting::ratio6_1);
motor rightMiddle = motor(RIGHT_MIDDLE, gearSetting::ratio6_1);
motor rightBack = motor(RIGHT_BACK, gearSetting::ratio6_1);
BetterMotorGroup RightMotors(rightPorts, 3);
BetterMotorGroup LeftMotors(leftPorts, 3);

// Other motors
motor CataMotor(CATA_PORT);
motor WinchMotor(WINCH_PORT, gearSetting::ratio36_1);
limit CataSwitch(Brain.ThreeWirePort.A);
digital_out BackArm(Brain.ThreeWirePort.H);

// Auton variables
const bool debug = true;

// modes & states
const int DM_STRAIGHT = 0;
const int DM_TURN = 1;
const int DISABLED = 2;

// config
const double kP = 0.05; // for distance

// Straight drive config
const double lrKp = 4; // for straight drive (for some unknown reason this doesn't work unless i use division)
const double lrMax = 1; // max change in left/right power (decreases faster side)

// runtime
double drivePower = 8;
int leftTarget = 0;
int rightTarget = 0;
int driveMode = DM_STRAIGHT;
// End of auton variables

bool disableCataSwitch = false; // Does what it says

void onCataSwitchPress(){
    if (disableCataSwitch) return;
    CataMotor.stop();
}

void onCataControllerButtonPress(){
    CataMotor.setStopping(hold);
    CataMotor.spin(fwd, 12, volt);
}

void onCataControllerButtonRelease(){
    CataMotor.stop();
}

// Auton Code =================================================================================
void stopDrive(){
    driveMode = DISABLED;
    leftFront.stop();
    leftBack.stop();
    leftMiddle.stop();
    rightFront.stop();
    rightBack.stop();
    rightMiddle.stop();


}

void resetTracking(){
    leftFront.resetPosition();
    leftBack.resetPosition();
    leftMiddle.resetPosition();
    rightFront.resetPosition();
    rightBack.resetPosition();
    rightMiddle.resetPosition();
}

// this needs code added to it that makes sure the values stay near each other
void _spinLeft(directionType dir, int volts){
    leftFront.spin(dir, volts, volt);
    leftMiddle.spin(dir, volts, volt);
    leftBack.spin(dir, volts, volt);
}

void _spinRight(directionType dir, int volts){
    rightFront.spin(dir, volts, volt);
    rightMiddle.spin(dir, volts, volt);
    rightBack.spin(dir, volts, volt);
}

// Main drive task
int autonDriveTask(){
    while (true){
        task::sleep(10);
        if (driveMode == DISABLED) {continue;}

        int l_error = leftFront.position(degrees) - leftTarget;
        int r_error = rightFront.position(degrees) - rightTarget;
        // lr should be the difference between the two
        int lr_error = l_error - r_error;

        int l_pos = leftFront.position(degrees);
        int r_pos = rightFront.position(degrees);

        if (debug){
            Brain.Screen.printAt(1, 20, "Left Pos: %d    ", l_pos);
            Brain.Screen.printAt(1, 40, "Right Pos: %d   ", r_pos);

            Brain.Screen.printAt(1, 60, "Left Error: %d    ", l_error);
            Brain.Screen.printAt(1, 80, "Right Error: %d   ", r_error);
            Brain.Screen.printAt(1, 100, "LR Error: %d   ", lr_error);
        }

        double l_out = l_error * kP;
        double r_out = r_error * kP;

        if (debug){
            Brain.Screen.printAt(1, 120, "LR Out: %d                   ", lr_error);
            Brain.Screen.printAt(1, 140, "LR Out Post: %d                   ", lr_error / lrKp);
        }

        int lr_out = lr_error / lrKp;
        // limit lr_out to -lrMax to lrMax
        if (lr_out > lrMax) lr_out = lrMax;
        if (lr_out < -lrMax) lr_out = -lrMax;


        // limit to -drivePower to drivePower
        if (l_out > drivePower) l_out = drivePower;
        if (l_out < -drivePower) l_out = -drivePower;
        if (r_out > drivePower) r_out = drivePower;
        if (r_out < -drivePower) r_out = -drivePower;

        // figure out which side is ahead and slow it down
        if (driveMode == DM_STRAIGHT){
            if (l_error > r_error){
                // add or subtract based on the direction
                if (lr_out > 0) l_out += lr_out;
                else l_out -= lr_out;
                if (debug) Brain.Screen.printAt(1, 160, "Left is ahead   ");
            } else if (r_error > l_error){
                // add or subtract based on the direction
                if (lr_out > 0) r_out += lr_out;
                else r_out -= lr_out;
                if (debug) Brain.Screen.printAt(1, 160, "Right is ahead   ");
            }
        }

        _spinLeft(fwd, -l_out);
        _spinRight(fwd, -r_out);
    }
}

// returns true if the velocity of leftFront or rightFront is not 0
bool isDriving(){
    return leftFront.velocity(pct) != 0 || rightFront.velocity(pct) != 0;
}

void driveAsync(int left, int right, int power){
    if (left == right) driveMode = DM_STRAIGHT;
    else driveMode = DM_TURN;

    leftTarget = leftBack.position(degrees) + left;
    rightTarget = leftBack.position(degrees) + right;
    drivePower = power;
}

// Auto-stops when velocity drops to 0
bool drive(int left, int right, int power){
    driveAsync(left, right, power);
    task::sleep(1000);
    while (isDriving()){
        task::sleep(100);
    }
    stopDrive();
    return true;
}

bool drive(int left, int right){
    return drive(left, right, drivePower);
}

// Autons ===============================
void pushInAuton(){
    // Controller1.rumble("........");
    // drive(700, 700, 12);
    driveAsync(700, 700, 7);
    // wait(1000, msec);
    task::sleep(1000);
    stopDrive();
    task::sleep(1000);
    // Controller1.rumble(".");
    drive(500, 500, -5);
}

void descoreAuton(){
    drive(-50000, 50000, 12);
}

void pushInAndDescore(){
    // drive(1400, 1400, 7);
    // drive(800, 0, 7);
    // drive(100, 100, 7);
    // BackArm.set(true);

    drive(-300, -300, 3); // line up with the line parallel to the goal
    drive(-200, 200); // turn to be parallel with the corner thingy
    resetTracking();
    drive(-390, -390);
    drive(300, -300);
    BackArm.set(true);
    resetTracking();
    drive(-200, -350);
    drive(-300, 300, 12);
}

// End of auton code ==========================================================================

// Driver Code ================================================================================
void driver(){
    stopDrive(); // Disable auton drive task
    CataSwitch.pressed(onCataSwitchPress);
    Controller1.ButtonR1.pressed(onCataControllerButtonPress);
    Controller1.ButtonR1.released(onCataControllerButtonRelease);
    
    while(true){
        RightMotors.spin(fwd, Controller1.Axis2.position(), pct);
        LeftMotors.spin(fwd, Controller1.Axis3.position(), pct);
        
        // Cata
        if (Controller1.ButtonY.pressing()){
            CataMotor.spin(reverse, 12, volt);
        } else if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonB.pressing()){
            CataMotor.stop();
        }

        // For the start of the match, launching pre-loads
        if (Controller1.ButtonB.pressing()){
            disableCataSwitch = true;
            CataMotor.spin(fwd, 12, volt);
        } else {disableCataSwitch = false;}

        // Winch
        if (Controller1.ButtonDown.pressing() && Controller1.ButtonRight.pressing()){
            WinchMotor.spin(fwd, 12, volt);
            CataMotor.setStopping(coast);
        } else {WinchMotor.stop();}

        // Back Arm
        if (Controller1.ButtonL1.pressing()){
            BackArm.set(true);
        } else {BackArm.set(false);}
    }
}
// End of driver code =========================================================================

int main() {
    task dt(autonDriveTask);

    // Configure some other stuff
    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
    CataMotor.setStopping(hold);

    driver();
    // pushInAndDescore();

    // AutonSelector selector = AutonSelector();
    // selector.setCompetitionMode(true);
    // selector.setDriver(driver);
    // selector.addAuton(driver, "Driver Only");
    // selector.addAuton(pushInAuton, "Push In");
    // selector.addAuton(descoreAuton, "Descorer");
    // selector.run(&Controller1, &Brain);
}