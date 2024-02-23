#include "vex.h"
#include "config.h"
#include "auton/auton.h"
#include "control/robocontrol.h"

using namespace vex;

void goofyAuton(){
    double lastLeft = leftFront.position(rotationUnits::deg);
    double lastRight = rightFront.position(rotationUnits::deg);

    LeftMotors.setStopping(coast);
    RightMotors.setStopping(coast);

    while (true){
        double leftDiff = leftFront.position(rotationUnits::deg) - lastLeft;
        double rightDiff = rightFront.position(rotationUnits::deg) - lastRight;

        wait(10, msec);

        if (abs(leftDiff) > 0.2 || abs(rightDiff) > 0.2){
            Controller1.rumble(".");
            driveAsync(-500, 500, 12);
            liftPowerUp();
            wait(5, seconds);
            stopDrive();
            powerLiftDown();
            wait(1, seconds);

            lastLeft = leftFront.position(rotationUnits::deg);
            lastRight = rightFront.position(rotationUnits::deg);
        }
    }
}