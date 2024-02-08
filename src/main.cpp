#include "vex.h"
#include "control/robocontrol.h"
#include "auton/auton_selector.h"
#include "auton/autons/autons.h"
#include "driver/opcontrol.h"
#include "driver/keybindmanager.h"
#include "safety/motormon.h"
#include "config.h"
#include <math.h>

using namespace vex;


int main() {
    task dt(autonDriveTask);

    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
    intakeMotor.setStopping(hold);
    setFrontArms(false);

    MotorMonitor monitor = MotorMonitor();
    monitor.start();

    AutonSelector()
        .setDriver(driver)
        .addAuton(driver, "Driver Control")
        .addAuton([]{}, "No Auton")
        .addAuton(nearSideWinpoint, "NearSide WP")
        .addAuton(nearSidePoints, "NearSide Points")
        .addAuton(farSideWinpoint, "FarSide WP")
        .addAuton(skillsAuton, "Skills Auton")
        .addAuton(justGoForward, "Drive Forward")
        .addAuton(goBackwardsAuton, "JIC Backwards")
        .run();
}