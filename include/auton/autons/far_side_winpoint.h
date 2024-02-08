#include "vex.h"
#include "config.h"
#include "control/robocontrol.h"

#pragma once

void farSideWinpoint(){
    // Intake a ball
    drive(0.5, 0.5, 5);
    intakeMotor.setVelocity(100, pct);
    intakeMotor.spinFor(-800, degrees, false);
    wait(0.2, seconds);
    drive(-0.4, -0.4, 6);

    // knock the ball out of the corner
    drive(0.25, -0.25, 7);
    drive(1, 1, 7);
    BackArm.set(true);
    drive(-0.5, 0.5, 7);
    drive(-0.1, -0.1, 6);
    drive(-0.2, 0.2, 7);
    BackArm.set(false);

    // go put the ball in the front of the goal
    drive(0.85, 0.85, 6);
    drive(0.48, -0.48, 6);
    drive(1.8, 1.8, 6);
    drive(0.5, -0.5, 6);
    setFrontArms(true);
    drive(-0.5, -0.5, 6);
    intakeMotor.setVelocity(100, pct);
    intakeMotor.spinFor(1800, degrees, false);
    wait(0.5, seconds);
    drive(1, 1, 6);
    drive(-0.5, -0.5, 6);

    drive(1, 1, 12);

    drive(-0.5, -0.5, 6);
    drive(1, 1, 12);

    drive(-0.5, -0.5, 6);

    setFrontArms(false);

    drive(-0.5, 0.5, 6);
}