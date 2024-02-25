#include "vex.h"
#include "config.h"
#include "control/robocontrol.h"

#pragma once

void farSidePoints(){
    // intake the ball
    drive(0.5, 0.5, 8);
    intakeMotor.setVelocity(100, pct);
    intakeMotor.spinFor(-500, degrees, false);
    wait(0.2, seconds);

    // put in the front of the goal
    drive(-0.15, 0.15, 12);
    drive(1.5, 1.5, 12);
    drive(0.2, -0.2, 12);
    drive(0.6, 0.6, 12);
    drive(0.5, -0.5, 12);
    intakeMotor.spinFor(1800, degrees, 80, velocityUnits::pct, false);
    wait(500, msec);

    // go grab the middle ball
    drive(-0.58, 0.58, 12);
    drive(0.3, 0.3, 6);
    intakeMotor.spinFor(-1500, degrees, 100, velocityUnits::pct, false);
    drive(0.3, 0.3, 12);
    
    // put it in front of the goal
    drive(0.6, -0.6, 12);
    intakeMotor.spinFor(1800, degrees, 80, velocityUnits::pct, true);
    drive(-0.47, 0.47, 12);
    drive(0.5, 0.5, 12);

    // go grab another ball
    drive(-0.5, 0.5, 12);
    drive(0.7, 0.7, 12);
    intakeMotor.spinFor(-700, degrees, 100, velocityUnits::pct, false);
    drive(0.3, 0.3, 12);
    intakeMotor.spinFor(-200, degrees, 100, velocityUnits::pct, false);
    drive(1, -1, 12);
    drive(0.5, 0.5, 12);
    intakeMotor.spinFor(1800, degrees, 80, velocityUnits::pct, true);
    setFrontArms(true);
    wait(200, msec);
    drive(1, 1, 24);
    drive(-1, -1, 12);
    drive(1, 1, 12);
}