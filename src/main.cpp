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
#include <math.h>

using namespace vex;

brain       Brain;
controller  Controller1;

const int LEFT_FRONT = PORT7; // 7
const int LEFT_MIDDLE = PORT21; // 21
const int LEFT_BACK = PORT10;
const int RIGHT_FRONT = PORT8; // 8
const int RIGHT_MIDDLE = PORT19;
const int RIGHT_BACK = PORT18;
const int CATA_PORT = PORT6; // 6
const int WINCH_PORT = PORT4;

const int R_ENCODER = PORT14;
const int CATA_ENCODER = PORT17;
const int CATA_BALL_DISTANCE = PORT3;

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

// Sensors
rotation CataEncoder(CATA_ENCODER, true);
vex::distance CataBallDistance(CATA_BALL_DISTANCE);
inertial Inertial(PORT13);

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
const int DM_DISABLED = 2;

const int CATA_MANUAL = 0;
const int CATA_SEMI_AUTO = 1;
const int CATA_AUTO = 2;

// config
const double slewStep = 0.2; // voltage change per step
const double kP = 0.05; // for distance

const double turnKP = 0.075;
const double turnKI = 0.0002;
const double turnKD = 0.1;

// Straight drive config
const double lrKp = 0.05; // for straight drive (for some unknown reason this doesn't work unless i use division)
const double lrMax = 0.5; // max change in left/right power (decreases faster side)

// runtime
double targetPower = 0;
double drivePower = 8;
double leftStart = 0;
double rightStart = 0;
double leftTarget = 0;
double rightTarget = 0;
double lastOut = 0;
double lastError = 0;
double targetTurn = 0;
double integral = 0;
int driveMode = DM_STRAIGHT;
int cataMode = CATA_SEMI_AUTO;
bool cataStopped = false;
bool shootingBall = false;
// End of auton variables

bool disableCataSwitch = false; // Does what it says
int lastBatterPercentage = 0;

bool disableCataTask = false;
bool skillsCataThing = false;

// Math utils =================================================================================
int sign(double num){
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
}

// Auton Code =================================================================================
void stopDrive(){
    driveMode = DM_DISABLED;
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
void _spinLeft(directionType dir, double volts){
    leftFront.spin(dir, volts, volt);
    leftMiddle.spin(dir, volts, volt);
    leftBack.spin(dir, volts, volt);
}

void _spinRight(directionType dir, double volts){
    rightFront.spin(dir, volts, volt);
    rightMiddle.spin(dir, volts, volt);
    rightBack.spin(dir, volts, volt);
}

// Main drive task
int autonDriveTask(){
    while (true){
        task::sleep(10);
        if (driveMode == DM_DISABLED) {
            Brain.Screen.setCursor(5, 1);
            Brain.Screen.print("Heading: %f   ", Inertial.rotation());
            continue;
        }

        LeftMotors.setStopping(hold);
        RightMotors.setStopping(hold);

        double l_out = 0;
        double r_out = 0;

        if (driveMode == DM_STRAIGHT){
            double l_error = leftFront.position(degrees) - leftTarget;
            double r_error = rightFront.position(degrees) - rightTarget;
            // lr should be the difference between the two
            double lr_error = l_error - r_error;

            double l_pos = leftFront.position(degrees);
            double r_pos = rightFront.position(degrees);

            l_out = l_error * kP;
            r_out = r_error * kP;

            if ((fabs(l_error) + fabs(r_error)) / 2 < 600){ // if we're close to the target, slow down
                drivePower *= 0.95;
            } else {
                drivePower += (targetPower - drivePower) / 10;
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

            lastOut = (fabs(l_out) + fabs(r_out)) / 2;

            // Straight'ness correction
            if (Inertial.rotation(degrees) > targetTurn){
                l_out *= 0.9;
            } else if (Inertial.rotation(degrees) < targetTurn){
                r_out *= 0.9;
            }

            // _spinLeft(fwd, -l_out);
            // _spinRight(fwd, -r_out);
        } else if (DM_TURN){
            double error = targetTurn - Inertial.rotation(degrees);
            if (fabs(error) < 1 && leftFront.velocity(pct) < 1 && rightFront.velocity(pct) < 1) {
                stopDrive(); // also sets the drive to disabled
            }

            double derivative = (error - lastError) * turnKD;
            lastError = error;
            integral += error;
            double proportional = error * turnKP;

            if (fabs(error) > 22 * (180 / M_PI)) integral = 0;
            if (integral > 100) integral = 100;
            if (-integral > 100) integral = -100;

            double out = proportional + derivative + (integral * turnKI);

            drivePower = targetPower;

            if (out > drivePower) out = drivePower;
            if (out < -drivePower) out = -drivePower;

            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("Error: %f   ", error);

            Brain.Screen.newLine();
            Brain.Screen.print("Target: %f   ", targetTurn);
            Brain.Screen.newLine();
            Brain.Screen.print("Out: %f   ", out);
            Brain.Screen.newLine();
            Brain.Screen.print("Heading: %f   ", Inertial.rotation());



            l_out = -out;
            r_out = out;
        }

        _spinLeft(fwd, -l_out);
        _spinRight(fwd, -r_out);
    }
}

// Runs in the background, manages the cata
// does things like full and semi-auto
int cataTask(){
    CataEncoder.resetPosition();
    while (true){
        task::sleep(10);
        if(disableCataTask) continue;

        // Brain.Screen.setCursor(1, 1);
        // Brain.Screen.print("Cata: %f", CataEncoder.position(degrees));
        // Brain.Screen.newLine();
        // Brain.Screen.print("Cata Ball: %f", CataBallDistance.objectDistance(inches));

        if (cataMode == CATA_SEMI_AUTO){
            if (CataEncoder.position(degrees) > 85 && !cataStopped){
                CataMotor.stop();
                cataStopped = true;
            } else if (cataStopped && CataEncoder.position(degrees) < 85){
                cataStopped = false;
            }
        } else if (cataMode == CATA_AUTO){
            if (CataEncoder.position(degrees) > 85 && !cataStopped && !shootingBall){
                CataMotor.stop();
                cataStopped = true;
            } else if (cataStopped && CataBallDistance.objectDistance(inches) < 3){
                cataStopped = false;
                shootingBall = true;
                Controller1.rumble(".");
            } else if (!cataStopped || shootingBall) {
                CataMotor.spin(fwd, 12, volt);
            }

            if (CataEncoder.position(degrees) < 5){
                shootingBall = false;
                Controller1.rumble("_");
            }
        }
    }
}

// returns true if the velocity of leftFront or rightFront is not 0
bool isDriving(){
    return fabs(leftFront.velocity(pct)) != 0 || fabs(rightFront.velocity(pct)) != 0; 
}

void driveAsync(double left, double right, double power){
    drivePower = power * 0.4; // 10% power
    targetPower = power;
    targetTurn = Inertial.rotation();
    driveMode = DM_STRAIGHT;

    leftStart = leftBack.position(degrees);
    rightStart = rightBack.position(degrees);

    leftTarget = leftFront.position(degrees) + (left * (driveMode == DM_STRAIGHT ? TILE_CONST : 1));
    rightTarget = rightFront.position(degrees) + (right * (driveMode == DM_STRAIGHT ? TILE_CONST : 1));
}

void turnAsync(double degrees, double power){
    integral = 0;
    driveMode = DM_TURN;
    drivePower = power * 0.10;
    targetPower = power;
    targetTurn = Inertial.rotation() + degrees;
}

void turn(double degrees, double power){
    turnAsync(degrees, power);
    task::sleep(1000);
    while (driveMode != DM_DISABLED){
        task::sleep(100);
    }
    stopDrive();
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
    double startTime = Brain.timer(timeUnits::msec);

    // if (true) return;
    // Push the ball in the goal
    drive(2, 2, 9);

    drive(-0.3, -0.3, 4); // back up a tad in case we've gone under the goal
    drive(-100, 100, 7);
    resetTracking();
    drive(0.4, 0.4, 4);
    resetTracking();
        
    // Get the ball out of the corner
    BackArm.set(true);
    drive(-0.45, -0.45, 8);
    drive(-200, -600, 4);
    BackArm.set(false);

    // drive back to the bar after pushing the ball out of the corner
    resetTracking();
    drive(-600, -900, 8);
    resetTracking();
    drive(-700, 700, 5);
    resetTracking();
    drive(1.65, 1.65, 8);
    drive(0.2, 0.2, 4);

    double timeElapsed = Brain.timer(timeUnits::msec) - startTime;
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Time Elapsed: %f", timeElapsed);
}

void nearSideAuton(){
    // push the pre-load then smack it into the goal
    drive(1, 1, 6);
    drive(250, -290, 6);
    resetTracking();
    LeftArm.set(true);
    wait(1, sec);
    LeftArm.set(false);

    // get the ball out of the corner
    BackArm.set(true);
    wait(0.5, sec);
    drive(-300, 300, 6);
    resetTracking();
    BackArm.set(false);
    drive(650, -650, 8);
    resetTracking();
    drive(1, 1, 9);
    resetTracking();
    drive(-140, 140, 9);
    resetTracking();
    drive(1.6, 1.6, 9);
    drive(0.3, 0.3, 5);


    // drive(-1.3, -1.3, 7);
    // drive(75, -75, 6);
    // resetTracking();
    // drive(-0.8, -0.8, 7);
    // drive(170, -170, 5);
    // resetTracking();
    // drive(-0.6, -0.6, 9);
    // BackArm.set(false);


    // drive(0.35, 0.35, 5); // get a little away from the corner
    // drive(350, -350, 5); // turn to go touch the thingy
    // resetTracking();

    // drive(600, 960, 7); // should be pointed straight at the bar
    // resetTracking();

    // drive(1.8, 1.8, 7); // drive to the bar
    // drive(0.35, 0.35, 2.4);

    // Get the ball out of the corner
    // resetTracking();
    // drive(0.75, 0.75, 7);
    // resetTracking();
    // drive(275, -275, 7);
    // BackArm.set(true);
    // resetTracking();
    // drive(-0.14, -0.14, 7);
    // resetTracking();
    // wait(0.5, sec);
    // drive(-300, 300, 9);
    // resetTracking();
    // BackArm.set(false);

    // // line up with the bar
    // drive(355, -355, 9);
    // resetTracking();
    // drive(0.9, 0.9, 9);
    // resetTracking();
    // drive(-140, 140, 9);
    // resetTracking();

    // drive(1.4, 1.4, 9);
}


bool disableRebound = false;
void skills_onCataPressed(){
    CataMotor.stop();
    wait(2000, msec);
    if (disableRebound) return;
    CataMotor.spin(fwd, 12, volt);
}

void skills_onCataRelease(){
    CataMotor.stop();
    // CataMotor.spinFor(-150, deg, 100, velocityUnits::pct);
    CataMotor.spin(fwd, 12, volt);
}

int holdCataDown(){
    while (true){
        task::sleep(10);
        if (CataEncoder.position(deg) < 70){
            CataMotor.spin(fwd, 12, volt);
        } else {
            CataMotor.stop();
        }
    }
}

void skillsAuton(){
    driveMode = DM_DISABLED;
    disableCataTask = true;
    CataMotor.spin(fwd, 12, volt);
    _spinLeft(reverse, 1.5);
    _spinRight(reverse, 1.5);
    wait(44.5*1000, msec);
    Controller1.rumble("...");
    wait(3*1000, msec);
    _spinLeft(fwd, 0);
    _spinRight(fwd, 0);

    disableRebound = true;
    // CataSwitch.pressed(skills_onCataPressed);
    task t(holdCataDown);
    resetTracking();
    drive(350, 0, 5);
    resetTracking();
    drive(4.75, 4.75, 11);
    drive(-300, 300, 9);
    resetTracking();
    drive(0.5, 0.5, 11);
    drive(-160, 160, 9);
    resetTracking();
    drive(2.5, 2.5, 11);
    drive(350, -350, 9);
    LeftArm.set(true);
    RightArm.set(true);
    resetTracking();
    drive(100, -100, 9);
    resetTracking();
    drive(1, 1, 12);
    drive(-1, -1, 12);
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
    // CataSwitch.pressed(onCataSwitchPress);
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
        } else if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonB.pressing() && !(cataMode == CATA_AUTO)){
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
    driveMode = DM_DISABLED; // disable the drive task

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
    task ct(cataTask);

    // Conf igure some other stuff
    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
    CataMotor.setStopping(hold); 


    // AutonSelector selector = AutonSelector();
    // selector.setCompetitionMode(true);
    // selector.setDriver(driver);
    // selector.addAuton(winpointAuton, "Winpoint Auton");
    // selector.addAuton(skillsAuton, "Skills Auton");
    // selector.addAuton(nearSideAuton, "Near Side Auton");
    // selector.run(&Controller1, &Brain);
    Inertial.calibrate();
    while (Inertial.isCalibrating()){
        task::sleep(10);
    }
    Inertial.resetHeading();
    Inertial.resetRotation();
    wait(1, sec);
    turn(-90, 5);
}