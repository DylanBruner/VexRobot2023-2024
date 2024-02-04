/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2140C                                                     */
/*    Created:      9/18/2023, 7:31:18 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "auton/auton_selector.h"
#include "robocontrol.h"
#include "auton/autons/autons.h"
#include "driver/opcontrol.h"
#include "driver/keybindmanager.h"
#include "config.h"
#include <math.h>

using namespace vex;


void dummyAuton(){} // does nothing

int main() {
    task dt(autonDriveTask);

    // Configure some other stuff
    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
    setFrontArms(false);

    if (!leftFront.installed() || !leftMiddle.installed() || !leftBack.installed() || 
        !rightFront.installed() || !rightMiddle.installed() || !rightBack.installed() ||
        !flywheelMotor.installed() || !intakeMotor.installed()){

        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("Warning a motor is not connected!");
        string motorsNotConnected = "";
        if (!leftFront.installed()) motorsNotConnected += "Left Front, ";
        if (!leftMiddle.installed()) motorsNotConnected += "Left Middle, ";
        if (!leftBack.installed()) motorsNotConnected += "Left Back, ";
        if (!rightFront.installed()) motorsNotConnected += "Right Front, ";
        if (!rightMiddle.installed()) motorsNotConnected += "Right Middle, ";
        if (!rightBack.installed()) motorsNotConnected += "Right Back, ";
        if (!flywheelMotor.installed()) motorsNotConnected += "Flywheel, ";
        if (!intakeMotor.installed()) motorsNotConnected += "Intake, ";
        Controller1.Screen.newLine();
        Controller1.Screen.print(motorsNotConnected.c_str());
        Controller1.Screen.newLine();
        Controller1.Screen.print("Press A to continue");
        while (!Controller1.ButtonA.pressing()){
            task::sleep(10);
            Controller1.rumble(".");
        }
    }

    AutonSelector selector = AutonSelector();
    selector.setCompetitionMode(true);
    selector.setDriver(driver);
    selector.addAuton(driver, "Driver Control");
    selector.addAuton(dummyAuton, "No Auton");
    selector.addAuton(nearSideWinpoint, "NearSide WP");
    selector.addAuton(nearSidePoints, "NearSide Points");
    selector.addAuton(farSideWinpoint, "FarSide WP");
    selector.addAuton(skillsAuton, "Skills Auton");
    selector.addAuton(justGoForward, "Drive Forward");
    selector.addAuton(goBackwardsAuton, "JIC Backwards");
    selector.run(&Controller1, &Brain);
}