#include "vex.h"
#include "config.h"
#include "control/robocontrol.h"

#pragma once

void skillsAuton(){
    intakeMotor.spinFor(200, degrees, 100, velocityUnits::pct); // drop the intake

    setFrontArms(false);

    // Intake a ball
    drive(0.5, 0.5, 8);

    intakeMotor.setVelocity(100, pct);
    intakeMotor.setStopping(hold);
    intakeMotor.spinFor(-800, degrees, true);
    wait(0.2, seconds);

    intakeMotor.spin(reverse, 12, voltageUnits::volt);

    // turn to shoot
    drive(0.35, -0.35, 6);
    drive(-0.7, -0.7, 5);
    flywheelMotor.spin(reverse, 11.5, volt);
    drive(0.07, -0.03, 6);

    wait(33, seconds); // 33 seconds
    Controller1.rumble(".......");
    flywheelMotor.stop();
    intakeMotor.stop();

    drive(0.5, 0.8, 6);
    drive(1.5, 1.5, 6);

    drive(-0.24, 0.24, 6);
    drive(1, 1, 6);
    // setFrontArms(true);
    drive(-0.5, 0.5, 6);
    intakeMotor.spinFor(1800, degrees, false);
    drive(0.5, 0.5, 12);
    drive(-1, -1, 12);
    drive(1.5, 1.5, 12);

    // move the ball we hit
    drive(-1.32, -1.32, 6);
    BackArm.set(true);
    drive(-0.8, 0.8, 7);
    drive(-0.55, 0, 6);
    drive(-0.85, 0.85, 6);
    drive(0.5, 0.5, 6);
    BackArm.set(false);

    // align a lil bit
    setFrontArms(false);
    drive(1, 1, 6);
    drive(-0.5, -0.5, 6);
    setFrontArms(true);

    // go over the bar
    setFrontArms(true);
    // drive(-26, -26, 12);
    driveMode = DM_DISABLED; // disable the drive task
    _spinLeft(reverse, 12);
    _spinRight(reverse, 12);
    wait(4, seconds);
    stopDrive();
    resetTracking();

    drive(1, 1, 7);
    drive(1, -1, 6);
    drive(5, 5, 12);

    drive(-1, -1, 12);
    drive(-0.35, 0, 6);
    drive(1, 1, 6);


    drive(-1, -1, 6);
    drive(-0.33, 0.33, 12);
    drive(0.5, 0.5, 12);
    drive(0.5, -0.5, 6);
    drive(1, 1, 12);
    drive(-1, -1, 12);
}