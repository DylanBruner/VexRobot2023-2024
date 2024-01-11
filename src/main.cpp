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

motor flywheelMotor = motor(FLYWHEEL_PORT, gearSetting::ratio6_1, true);
motor intakeMotor = motor(INTAKE_PORT, gearSetting::ratio6_1, true);

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

            double l_pos = leftFront.position(degrees);
            double r_pos = rightFront.position(degrees);

            // l_out = l_error * driveKP;
            // r_out = r_error * driveKP;
            // drive power needs to be calculated into the equation
            l_out = (l_error * driveKP) + (drivePower * 0.5);
            r_out = (r_error * driveKP) + (drivePower * 0.5);

            if (l_out > drivePower) l_out = drivePower;
            if (l_out < -drivePower) l_out = -drivePower;
            if (r_out > drivePower) r_out = drivePower;
            if (r_out < -drivePower) r_out = -drivePower;

            drivePower = targetPower;
            if ((fabs(l_error) + fabs(r_error)) / 2 < 600){ // if we're close to the target, slow down
                drivePower *= 0.95;
            } else {
                drivePower += (targetPower - drivePower) / 10;
            }
            
            // Straight'ness correction
            // if (Inertial.rotation(degrees) > targetTurn){
            //     l_out *= 0.9;
            // } else if (Inertial.rotation(degrees) < targetTurn){
            //     r_out *= 0.9;
            // }
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
    return fabs(leftFront.velocity(pct)) > 5 || fabs(rightFront.velocity(pct)) > 5; 
}

void driveAsync(double left, double right, double power){
    drivePower = power * 0.4; // 10% power
    // drivePower = power;
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
    // Intake a ball
    drive(0.24, 0.24, 5);
    intakeMotor.setVelocity(100, pct);
    intakeMotor.spinFor(-600, degrees, false);
    wait(0.2, seconds);


    // Get the ball out of the corner
    drive(-0.275, 0.275, 8);
    drive(0.5, 0.5, 8);
    drive(0.5, -0.5, 8);
    BackArm.set(true);
    drive(-0.25, 0.25, 20);
    BackArm.set(false);

    drive(-0.4, 0.4, 8);
    drive(-0.45, -0.45, 4.5);

    drive(0.82, -0.82, 6);

    drive(1.15, 1.15, 8);
    intakeMotor.spinFor(1000, degrees, true);

    drive(-0.85, -0.85, 8);
    drive(-1, 1, 7);

    drive(-1, -1, 7);
    driveMode = DM_DISABLED;
    LeftMotors.setStopping(coast);
    RightMotors.setStopping(coast);
    _spinLeft(reverse, 2);
    _spinRight(reverse, 2);
    wait(4, seconds);
    _spinLeft(fwd, 2);
    _spinRight(fwd, 2);
    wait(0.1, seconds);
    stopDrive();

    // drive(0.3, -0.3, 6);
    // drive(0.15, 0.15, 6);
    // BackArm.set(true);
    // drive(-0.2, 0.2, 6);

    // drive(0, 0.5, 7);
    // drive(1.05, 1.05, 5);
    // drive(0.3, -0.3, 7);


    // // Push the ball into the goal
    // drive(-2, -2, 7);

    // // Flatten out by using the goal
    // drive(0.2, 0.2, 5);
    // drive(0.1, -0.18, 6);
    // drive(-0.3, -0.3, 6);

    // // Go get the ball out of the corner
    // drive(0.1, 0.6, 6);
    // drive(0.9, 0.9, 6);
    // BackArm.set(true);
    // drive(-0.85, 0.85, 6);

    // // Go touch the bar
    // BackArm.set(false);
    // drive(-0.3, 0.3, 6);
    // drive(-1.25, -1.25, 2);
    // drive(-0.175, 0.175, 6);

    // drive(-0.8, -0.8, 7);
    // driveMode = DM_DISABLED;
    // _spinLeft(reverse, 2);
    // _spinRight(reverse, 2);
    // wait(4, seconds);


    // BackArm.set(true);
    
    // // get the ball out of the corner
    // drive(-0.15, -0.15, 8);
    // drive(-0.5, 0.5, 4);
    // BackArm.set(false);
    // drive(0.15, 0.15, 5);

    // // go touch the bar
    // drive(-0.38, 0.38, 4);
    // drive(-0.6, -0.6, 6);
    // drive(-0.19, 0.19, 6);
    // LeftMotors.setStopping(coast);
    // RightMotors.setStopping(coast);
    // drive(-1, -1, 6);
    // driveMode = DM_DISABLED; // disable the drive task
    // _spinLeft(reverse, 4);
    // _spinRight(reverse, 4);
    // wait(1, seconds);
    // stopDrive();

    // wait(3, seconds);
    // LeftMotors.setStopping(brake);
    // RightMotors.setStopping(brake);

    // _spinLeft(fwd, 4);
    // _spinRight(fwd, 4);

    // wait(2, seconds);
    // stopDrive();
}

void farSideWinpoint(){
    // push the ball into the goal and flatten out
    drive(-2, -2, 6.5);
    drive(0.3, 0.3, 5);
    drive(-0.15, 0.15, 6);

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
    drive(0.72, -0.72, 6);
    drive(-0.7, -0.7, 6);
    drive(0, -0.6, 6);
    drive(-0.7, -0.7, 6);
    LeftMotors.setStopping(coast);
    RightMotors.setStopping(coast);
    driveMode = DM_DISABLED; // disable the drive task
    _spinLeft(reverse, 4);
    _spinRight(reverse, 4);
    wait(0.7, seconds);
    stopDrive();
    wait(5, seconds);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
}

void skillsAuton(){
    PowerDownPiston.set(true); // this is inverted
    PowerUpPiston.set(true);
    TopLiftPiston.set(true);
    flywheelMotor.spin(fwd, 100, pct);
    wait(60, seconds);
}

// End of auton code ==========================================================================

// Driver Code ================================================================================
bool liftUp = false;
bool upperLift = false;
bool backArm = false;
bool frontArms = false;

bool flywheelSpinning = false;

void driver(){
    stopDrive(); // Disable auton drive task

    Controller1.ButtonUp.pressed([](){
        liftUp = !liftUp;
        PowerDownPiston.set(true); // this is inverted
        PowerUpPiston.set(liftUp);
    });

    Controller1.ButtonDown.pressed([](){
        PowerDownPiston.set(false);
        PowerUpPiston.set(false);
        TopLiftPiston.set(false);
        liftUp = false;
    });

    Controller1.ButtonLeft.pressed([](){
        upperLift = !upperLift;
        TopLiftPiston.set(upperLift);
    });

    Controller1.ButtonB.pressed([](){
        backArm = !backArm;
        BackArm.set(backArm);
    });

    Controller1.ButtonY.pressed([](){
        frontArms = !frontArms;
        FrontArms.set(frontArms); 
    });

    Controller1.ButtonL1.pressed([](){
        flywheelSpinning = !flywheelSpinning;
        if (flywheelSpinning){
            flywheelMotor.spin(fwd, 100, pct);
        } else {
            flywheelMotor.stop();
        }
    });

    Controller1.ButtonL2.pressed([](){
        flywheelSpinning = !flywheelSpinning;
        if (flywheelSpinning){
            flywheelMotor.spin(reverse, 100, pct);
        } else {
            flywheelMotor.stop();
        }
    });

    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);

    while(true){
        if (lastBatterPercentage != Brain.Battery.capacity(percent)){
            lastBatterPercentage = Brain.Battery.capacity(percent);
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Battery: %d%%", lastBatterPercentage);
        }

        if (Controller1.ButtonR1.pressing()){
            intakeMotor.spin(reverse, 100, pct);
        } else if (Controller1.ButtonR2.pressing()){
            intakeMotor.spin(fwd, 100, pct);
        } else {
            intakeMotor.stop();
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

    // Inertial.calibrate();
    // while (Inertial.isCalibrating()){
        // task::sleep(10);
    // }

    double start = Brain.timer(timeUnits::msec);
    farSideWinpoint();
    // driver();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Time: %f", Brain.timer(timeUnits::msec) - start);

    // AutonSelector selector = AutonSelector();
    // selector.setCompetitionMode(true);
    // selector.setDriver(driver);
    // selector.addAuton(driver, "Driver Control");
    // selector.addAuton(dummyAuton, "No Auton");
    // selector.addAuton(nearSideWinpoint, "NearSide WP");
    // selector.addAuton(farSideWinpoint, "FarSide WP");
    // selector.addAuton(skillsAuton, "Skills Auton");
    // selector.addAuton(justGoForward, "Drive Forward");
    // selector.run(&Controller1, &Brain);
}