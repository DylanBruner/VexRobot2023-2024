#include "better_motor_group.h"

void BetterMotorGroup::spin(directionType dir, double velocity, velocityUnits units) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->spin(dir, velocity, units);
        }
    }
}

void BetterMotorGroup::spin(directionType dir, double velocity, percentUnits units) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->spin(dir, velocity, units);
        }
    }
}

void BetterMotorGroup::spin(directionType dir, double velocity, voltageUnits units) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->spin(dir, velocity, units);
        }
    }
}

void BetterMotorGroup::spinFor(directionType dir, double rotation, rotationUnits units, double velocity, velocityUnits units2, bool waitForCompletion = false) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            if (i == 3){
                motors[i]->spinFor(dir, rotation, units, velocity, units2, false);
            } else {
                motors[i]->spinFor(dir, rotation, units, velocity, units2, waitForCompletion);
            }
        }
    }
}

void BetterMotorGroup::stop(brakeType mode) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->stop(mode);
        }
    }
}

void BetterMotorGroup::stop() {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->stop();
        }
    }
}

void BetterMotorGroup::resetPosition() {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->resetPosition();
        }
    }
}

void BetterMotorGroup::setReversed(bool reverse) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->setReversed(reverse);
        }
    }
}

void BetterMotorGroup::setStopping(brakeType mode) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->setStopping(mode);
        }
    }
}

void BetterMotorGroup::setVelocity(double velocity, velocityUnits units) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->setVelocity(velocity, units);
        }
    }
}

void BetterMotorGroup::setMaxTorque(double torque, percentUnits units) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->setMaxTorque(torque, units);
        }
    }
}

void BetterMotorGroup::setMaxTorque(double torque, torqueUnits units) {
    for (int i = 0; i < 10; i++) {
        if (motors[i] != NULL) {
            motors[i]->setMaxTorque(torque, units);
        }
    }
}