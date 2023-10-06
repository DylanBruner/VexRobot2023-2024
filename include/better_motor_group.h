#include "vex.h"

#pragma once

using namespace vex;

class BetterMotorGroup {
    public:
        motor* motors[10];

        /*
            Create a "BetterMotorGroup"
            This is used so we can group more than two motors together,
            this is needed because vex only supports motor groups of two motors.
        */
        BetterMotorGroup(int32_t ports[], int numMotors) {
            for (int i = 0; i < numMotors; i++) {
                motors[i] = new motor(ports[i]);
            }
        }

        void spin(directionType dir, double velocity, velocityUnits units);
        void spin(directionType dir, double velocity, percentUnits units);
        void spinFor(directionType dir, double rotation, rotationUnits units, double velocity, velocityUnits units2, bool waitForCompletion);
        void stop(brakeType mode);
        void stop();
        void resetPosition();
        void setReversed(bool reverse);
        void setVelocity(double velocity, velocityUnits units);
        void setStopping(brakeType mode);
};