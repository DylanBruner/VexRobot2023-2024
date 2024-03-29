#include "vex.h"

#pragma once

using namespace vex;

class BetterMotorGroup {
    public:
        motor* motors[10];

        BetterMotorGroup(int32_t ports[], int numMotors) {
            for (int i = 0; i < numMotors; i++) {
                motors[i] = new motor(ports[i]);
            }
        }

        void spin(directionType dir, double velocity, velocityUnits units);
        void spin(directionType dir, double velocity, percentUnits units);
        void spin(directionType dir, double velocity, voltageUnits units);
        void spinFor(directionType dir, double rotation, rotationUnits units, double velocity, velocityUnits units2, bool waitForCompletion);
        void stop(brakeType mode);
        void stop();
        void resetPosition();
        void setReversed(bool reverse);
        void setVelocity(double velocity, velocityUnits units);
        void setStopping(brakeType mode);
        void setMaxTorque(double torque, percentUnits units);
        void setMaxTorque(double torque, torqueUnits units);
};