#include "vex.h"
#include "config.h"
#include "control/robocontrol.h"

#pragma once

void farSideWinpoint(){

    // grab the middle-middle ball
    drive(2.5, 2.5, 12);
    intakeMotor.spin(reverse, 100, pct);
    drive(0.5, 0.5, 12);
    wait(100, msec);
    intakeMotor.stop();
    drive(0.54, -0.54, 12);
    drive(0.2, 0.2, 12);
    intakeMotor.setVelocity(100, pct);
    intakeMotor.spinFor(1800, degrees, false);
    wait(50, msec);
    drive(1, 1, 24);

    // grab the middle back ball
    drive(-0.5, -0.5, 12);
    drive(1, -1, 12);
    drive(-0.5, -0.5, 12); // drive backwards to straight up a bit
    drive(1, 1, 12);
    drive(0.08, -0.08, 12);
    drive(0.5, 0.5, 12);

    intakeMotor.spinFor(-800, degrees);
    drive(-0.5, -0.5, 12);
    drive(0.9, -0.9, 12);
    drive(0.7, 0.7, 12);
    intakeMotor.spinFor(1800, degrees, false);
    wait(100, msec);
    drive(1, 1, 24);
    
    // go get the ball out of the corner
    drive(-1.4, -1.4, 12);
    drive(0.4, -0.4, 12);
    drive(2.8, 2.8, 12);
    drive(-0.09, 0.8, 12);
    wait(200, msec);
    setRightArm(true);
    drive(-0.3, 0.3, 12);
    drive(0.3, 0.3, 12);
}