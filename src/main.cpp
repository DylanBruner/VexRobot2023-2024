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
#include "odometry.h"

using namespace vex;

brain       Brain;
controller  Controller1;

const int LEFT_FRONT = PORT7; // 7
const int LEFT_MIDDLE = PORT21; // 21
const int LEFT_BACK = PORT10;
const int RIGHT_FRONT = PORT8; // 8
const int RIGHT_MIDDLE = PORT18;
const int RIGHT_BACK = PORT19;
const int CATA_PORT = PORT6; // 6
const int WINCH_PORT = PORT4;

const int L_ENCODER = PORT15;
const int R_ENCODER = PORT14;
const int CATA_ENCODER = PORT14;
const int CATA_BALL_DISTANCE = PORT13;

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
rotation CataEncoder(CATA_ENCODER);
vex::distance CataBallDistance(CATA_BALL_DISTANCE);


motor WinchMotor(WINCH_PORT, gearSetting::ratio36_1);
limit CataSwitch(Brain.ThreeWirePort.A);
digital_out BackArm(Brain.ThreeWirePort.E);
digital_out RightArm(Brain.ThreeWirePort.D);
digital_out LeftArm(Brain.ThreeWirePort.H);

// Auton variables
const bool DEBUG = true;
const int TILE_CONST = 700;

// modes & states
const int DM_STRAIGHT = 0;
const int DM_TURN = 1;
const int DISABLED = 2;

const int CATA_MANUAL = 0;
const int CATA_SEMI_AUTO = 1;
const int CATA_AUTO = 2;

// config
const double kP = 0.05; // for distance

// Straight drive config
const double lrKp = 4; // for straight drive (for some unknown reason this doesn't work unless i use division)
const double lrMax = 1; // max change in left/right power (decreases faster side)

// runtime
double drivePower = 8;
double leftTarget = 0;
double rightTarget = 0;
int driveMode = DM_STRAIGHT;
int cataMode = CATA_AUTO;
bool cataStopped = false;
// End of auton variables

bool disableCataSwitch = false; // Does what it says
int lastBatterPercentage = 0;

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

        double l_error = leftFront.position(degrees) - leftTarget;
        double r_error = rightFront.position(degrees) - rightTarget;
        // lr should be the difference between the two
        double lr_error = l_error - r_error;

        double l_pos = leftFront.position(degrees);
        double r_pos = rightFront.position(degrees);

        if (DEBUG){
            Brain.Screen.printAt(1, 20, "Left Pos: %d    ", l_pos);
            Brain.Screen.printAt(1, 40, "Right Pos: %d   ", r_pos);

            Brain.Screen.printAt(1, 60, "Left Error: %d    ", l_error);
            Brain.Screen.printAt(1, 80, "Right Error: %d   ", r_error);
            Brain.Screen.printAt(1, 100, "LR Error: %d   ", lr_error);
        }

        double l_out = l_error * kP;
        double r_out = r_error * kP;

        if (DEBUG){
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
                if (DEBUG) Brain.Screen.printAt(1, 160, "Left is ahead   ");
            } else if (r_error > l_error){
                // add or subtract based on the direction
                if (lr_out > 0) r_out += lr_out;
                else r_out -= lr_out;
                if (DEBUG) Brain.Screen.printAt(1, 160, "Right is ahead   ");
            }
        }

        _spinLeft(fwd, -l_out);
        _spinRight(fwd, -r_out);
    }
}

int cataTask(){
    CataEncoder.resetPosition();
    while (true){
        task::sleep(10);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Cata: %f", CataEncoder.position(degrees));
        // print out the cataball distance
        Brain.Screen.newLine();
        Brain.Screen.print("Cata Ball: %f", CataBallDistance.objectDistance(inches));

        if (cataMode == CATA_SEMI_AUTO){
            if (CataEncoder.position(degrees) > 100 && !cataStopped){
                CataMotor.stop();
                cataStopped = true;
            } else if (cataStopped && CataEncoder.position(degrees) < 90){
                cataStopped = false;
            }
        } else if (cataMode == CATA_AUTO){
            if (CataEncoder.position(degrees) > 100 && CataBallDistance.objectDistance(inches) < 6 && !cataStopped){
                CataMotor.stop();
                cataStopped = true;
                Controller1.rumble(".....");
            } else if (CataBallDistance.objectDistance(inches) > 6){
                CataMotor.spin(fwd, 12, volt);
                cataStopped = false;
            } else if (CataEncoder.position(degrees) < 90){
                CataMotor.spin(fwd, 12, volt);
            }
        }
    }

}

// returns true if the velocity of leftFront or rightFront is not 0
bool isDriving(){
    return leftFront.velocity(pct) != 0 || rightFront.velocity(pct) != 0;
}

void driveAsync(double left, double right, double power){
    if (left == right) driveMode = DM_STRAIGHT;
    else driveMode = DM_TURN;

    leftTarget = leftBack.position(degrees) + (left * (driveMode == DM_STRAIGHT ? TILE_CONST : 1));
    rightTarget = leftBack.position(degrees) + (right * (driveMode == DM_STRAIGHT ? TILE_CONST : 1));
    drivePower = power;
}

// Auto-stops when velocity drops to 0
bool drive(double left, double right, double power){
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
void pushInTouchPole(){
    drive(2.5, 2.5, 5); // push the ball in the goal
    drive(-0.3, -0.3, 5); // back up
    drive(-400, 0, 5); // turn to be parallel with the non-driverbox sides
    BackArm.set(true);
    drive(-2.2, -2.2, 5); // drive back into the middle corner thingy
    drive(200, -200, 12); // turn more into the pole
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

void justGoForward(){
    drive(5, 5, 9);
    drive(-0.75, -0.75, 9);
}

void winpointAuton(){

}

// End of auton code ==========================================================================

// Driver Code ================================================================================
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

void onButtonUpPressed(){
    LeftArm.set(true);
    RightArm.set(true);
}

void onButtonDownPressed(){
    LeftArm.set(false);
    RightArm.set(false);
}

void driver(){
    stopDrive(); // Disable auton drive task
    CataSwitch.pressed(onCataSwitchPress);
    Controller1.ButtonR1.pressed(onCataControllerButtonPress);
    Controller1.ButtonR1.released(onCataControllerButtonRelease);
    Controller1.ButtonUp.pressed(onButtonUpPressed);
    Controller1.ButtonDown.pressed(onButtonDownPressed);

    
    while(true){
        if (lastBatterPercentage != Brain.Battery.capacity(percent)){
            lastBatterPercentage = Brain.Battery.capacity(percent);
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Battery: %d%%", lastBatterPercentage);
        }
        
        RightMotors.spin(fwd, Controller1.Axis2.position(), pct);
        LeftMotors.spin(fwd, Controller1.Axis3.position(), pct);
        
        // Cata
        if (Controller1.ButtonY.pressing()){
            CataMotor.spin(reverse, 12, volt);
        } else if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonB.pressing() && !cataMode == CATA_AUTO){
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

void motorTester(){
    driveMode = DISABLED; // disable the drive task

    int lastMotor = -1;
    int currentMotor = 0; // 0-5

    motor* testing = nullptr;
    // set testing to the motor that is currently being tested, 0-2 = left, 3-5 = right
    while (true){
        if (lastMotor != currentMotor){
            Controller1.rumble(".");
            if (testing != nullptr) testing->stop();

            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            if (currentMotor == 0) {testing = &leftFront; Controller1.Screen.print("Left Front");}
            else if (currentMotor == 1) {testing = &leftMiddle; Controller1.Screen.print("Left Middle");}
            else if (currentMotor == 2) {testing = &leftBack; Controller1.Screen.print("Left Back");}
            else if (currentMotor == 3) {testing = &rightFront; Controller1.Screen.print("Right Front");}
            else if (currentMotor == 4) {testing = &rightMiddle; Controller1.Screen.print("Right Middle");}
            else if (currentMotor == 5) {testing = &rightBack; Controller1.Screen.print("Right Back");}

            Controller1.Screen.newLine();
            Controller1.Screen.print("Temp: %d", testing->temperature(vex::temperatureUnits::fahrenheit));
            lastMotor = currentMotor;
        }

        if (Controller1.ButtonA.pressing()){
            wait(500, msec);
            currentMotor++;
            if (currentMotor > 5) currentMotor = 0;
        }

        if (testing != nullptr) testing->spin(fwd, Controller1.Axis3.position(), pct);
    }
}

int main() {
    task dt(autonDriveTask);
    // task ct(cataTask);

    if (Controller1.ButtonA.pressing()){}

    // Configure some other stuff
    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
    CataMotor.setStopping(hold);

    // driver();
    winpointAuton();

    // AutonSelector selector = AutonSelector();
    // selector.setCompetitionMode(true);
    // selector.setDriver(driver);
    // selector.addAuton(driver, "Driver Only");
    // selector.addAuton(motorTester, "Motor Tester");
    // selector.addAuton(justGoForward, "Go Forward...");
    // selector.addAuton(pushInTouchPole, "Push In & Pole");
    // selector.run(&Controller1, &Brain);

    // Brain.Screen.clearScreen();

    // double t = 1;
    // leftFront.setMaxTorque(t, pct);
    // leftMiddle.setMaxTorque(t, pct);
    // leftBack.setMaxTorque(t, pct);
    // rightFront.setMaxTorque(t, pct);
    // rightMiddle.setMaxTorque(t, pct);
    // rightBack.setMaxTorque(t, pct);

    // rotation leftTracker(L_ENCODER);
    // rotation rightTracker(R_ENCODER);
    
    // Odometry odometry = Odometry(3.5, 3.5, 0.0, &leftTracker, &rightTracker, &rightBack);


    // while (true){
    //     vex::task::sleep(10);
    // 
    //     Brain.Screen.setCursor(1, 1);
    //     Brain.Screen.print("Left: %f            ", tracker.position(degrees));
    // }

    // while (true){
    //     vex::task::sleep(10);

    //     // draw the heading on the brain screen
    //     Brain.Screen.setCursor(1, 1);
    //     double error = (0) - odometry.getHeading();
    //     Brain.Screen.print("Heading: %f               ",         odometry.getHeading());
    //     Brain.Screen.newLine();
    //     Brain.Screen.print("Error: %f               ", error);

    //     if (fabs(error) > 10){
    //         Controller1.rumble(".");
    //         if (error < 0){
    //             LeftMotors.spin(fwd, -fmax(fabs(error / 4), 5), pct);
    //             RightMotors.spin(fwd, fmax(fabs(error / 4), 5), pct);
    //         } else {
    //             LeftMotors.spin(fwd, fmax(fabs(error / 4), 5), pct);
    //             RightMotors.spin(fwd, -fmax(fabs(error / 4), 5), pct);
    //         }
    //     } else {
    //         LeftMotors.stop();
    //         RightMotors.stop();
    //     }
    // }
}