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
#include "config.h"
#include <math.h>

using namespace vex;

brain       Brain;
controller  Controller1;

// Drive motors
int32_t rightPorts[] = {RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK};
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

// Sensors
inertial Inertial(INERTIAL_PORT);

// lift top = D
// power lift down = E
// power up = G

digital_out FrontArms(Brain.ThreeWirePort.F);
digital_out BackArm(Brain.ThreeWirePort.E);

digital_out TopLiftPiston(Brain.ThreeWirePort.D);
digital_out PowerUpPiston(Brain.ThreeWirePort.G);
digital_out PowerDownPiston(Brain.ThreeWirePort.H); //

// runtime
double targetPower = 0;
double drivePower = 8;
double leftTarget = 0;
double rightTarget = 0;
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

            l_out = l_error * driveKP;
            r_out = r_error * driveKP;

            if ((fabs(l_error) + fabs(r_error)) / 2 < 600){ // if we're close to the target, slow down
                drivePower *= 0.95;
            } else {
                drivePower += (targetPower - drivePower) / 10;
            }
            
            // Straight'ness correction
            if (Inertial.rotation(degrees) > targetTurn){
                l_out *= 0.9;
            } else if (Inertial.rotation(degrees) < targetTurn){
                r_out *= 0.9;
            }
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

bool isDriving(){
    return fabs(leftFront.velocity(pct)) != 0 || fabs(rightFront.velocity(pct)) != 0; 
}

void driveAsync(double left, double right, double power){
    drivePower = power * 0.4; // 10% power
    targetPower = power;
    targetTurn = Inertial.rotation();
    driveMode = DM_STRAIGHT;

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
void dummyAuton(){} // does nothing

void justGoForward(){
    drive(5, 5, 9);
    drive(-0.75, -0.75, 9);
}

void nearSideWinpoint(){
    BackArm.set(true);
    
    // get the ball out of the corner
    drive(-0.15, -0.15, 8);
    drive(-0.5, 0.5, 4);
    BackArm.set(false);
    drive(0.15, 0.15, 5);

    // go touch the bar
    drive(-0.38, 0.38, 4);
    drive(-0.6, -0.6, 6);
    drive(-0.19, 0.19, 6);
    LeftMotors.setStopping(coast);
    RightMotors.setStopping(coast);
    // drive(-1, -1, 6);
    driveMode = DM_DISABLED; // disable the drive task
    _spinLeft(reverse, 4);
    _spinRight(reverse, 4);
    wait(1, seconds);
    stopDrive();

    wait(5, seconds);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
}

void farSideWinpoint(){
    // push the ball into the goal and flatten out
    drive(-2, -2, 9);
    drive(0.3, 0.3, 5);
    drive(-0.2, 0.2, 6);

    // go get the ball out of the corner
    drive(-0.5, -0.5, 6);
    drive(0.5, 0.5, 6);
    drive(0.5, -0.5, 6);
    drive(0.5, 0.5, 6);
    BackArm.set(true);
    drive(0.27, -0.27, 6);
    drive(-0.45, -0.45, 6);
    drive(-0.3, 0.3, 6);

    BackArm.set(false);
    drive(0.5, 0.5, 6);
    drive(0.7, -0.7, 6);
}

// End of auton code ==========================================================================

// Driver Code ================================================================================
void driver(){
    stopDrive(); // Disable auton drive task

    Controller1.ButtonX.pressed([](){
        PowerDownPiston.set(true); // this is inverted
        PowerUpPiston.set(true);
    });

    Controller1.ButtonB.pressed([](){
        PowerDownPiston.set(true); // this is inverted
        PowerUpPiston.set(false);
    });

    Controller1.ButtonY.pressed([](){
        PowerDownPiston.set(false); // this is inverted
        PowerUpPiston.set(false);
    });
    
    while(true){
        if (lastBatterPercentage != Brain.Battery.capacity(percent)){
            lastBatterPercentage = Brain.Battery.capacity(percent);
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Battery: %d%%", lastBatterPercentage);
        }

        // Side Arms ==========
        if (Controller1.ButtonUp.pressing()) {
            FrontArms.set(true);
        } else if (Controller1.ButtonDown.pressing()) {
            FrontArms.set(false);
        }

        // Back Arm ==========
        if (Controller1.ButtonLeft.pressing()) {
            BackArm.set(true);
        } else if (Controller1.ButtonRight.pressing()) {
            BackArm.set(false);
        }

        RightMotors.spin(fwd, Controller1.Axis2.position(), pct);
        LeftMotors.spin(fwd, Controller1.Axis3.position(), pct);        
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

    // Conf igure some other stuff
    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);

    Inertial.calibrate();
    // while (Inertial.isCalibrating()){
        // task::sleep(10);
    // }

    // nearSideWinpoint();
    farSideWinpoint();

    // AutonSelector selector = AutonSelector();
    // selector.setCompetitionMode(true);
    // selector.setDriver(driver);
    // selector.addAuton(driver, "Driver Control");
    // selector.addAuton(dummyAuton, "No Auton");
    // selector.addAuton(justGoForward, "Drive Forward");
    // selector.run(&Controller1, &Brain);
}