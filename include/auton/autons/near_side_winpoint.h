#include "vex.h"
#include "config.h"
#include "auton/auton.h"

#pragma once

void nearSideWinpoint(){
    // intakeMotor.spinFor(-200, degrees, 100, velocityUnits::pct); // drop the intake
    // setFrontArms(false);

    // Intake a ball
    drive(0.5, 0.5, 8);

    intakeMotor.setVelocity(100, pct);
    intakeMotor.spinFor(-500, degrees, false);
    wait(0.2, seconds);

    // get the ball out of the corner
    drive(0.25, -0.25, 7);
    drive(-0.22, -0.22, 7);
    BackArm.set(true);
    drive(-0.25, 0.25, 7);
    BackArm.set(false);

    // go out-take the ball into the goal
    drive(0.6, -0.6, 6);
    drive(0.75, 0.75, 6);
    drive(-0.05, 0.15, 6);

    drive(1, 1, 6);
    intakeMotor.spinFor(1800, degrees, false);
    wait(0.5, seconds);


    // go turn around to touch the bar
    drive(-1.2, -1.2, 6);
    // drive(1, -1, 6);
    drive(-1, 1, 6);
    drive(-1, -1, 6);

    driveMode = DM_DISABLED; // disable the drive task
    _spinLeft(reverse, 2.5);
    _spinRight(reverse, 2.5);
    wait(0.5, seconds);
    while ((leftFront.efficiency() + leftMiddle.efficiency() + leftBack.efficiency()) / 3 > 0.15){
        task::sleep(10);
    }
    Controller1.rumble("...");
    stopDrive();
    drive(0, 0.2, 6);
}