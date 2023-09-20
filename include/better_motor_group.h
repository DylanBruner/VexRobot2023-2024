#include "vex.h"

using namespace vex;

class BetterMotorGroup {
    public:
        motor* motors[10];

        // BetterMotorGroup constructor takes in a array of int's (motor ports)
        BetterMotorGroup(int32_t ports [], int numMotors) {
            // Resize the motor array to the number of motors
            for (int i = 0; i < numMotors; i++) {
                motors[i] = new motor(ports[i]);
            }
        }

        void spin(directionType dir, double velocity, velocityUnits units) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->spin(dir, velocity, units);
                }
            }
        }

        void spin(directionType dir, double velocity, percentUnits units) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->spin(dir, velocity, units);
                }
            }
        }

        void spinFor(directionType dir, double rotation, rotationUnits units, double velocity, velocityUnits units2, bool waitForCompletion = false) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->spinFor(dir, rotation, units, velocity, units2, waitForCompletion);
                }
            }
        }

        void stop(brakeType mode) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->stop(mode);
                }
            }
        }

        void stop() {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->stop();
                }
            }
        }

        void resetPosition() {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->resetPosition();
                }
            }
        }

        void setReversed(bool reverse) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->setReversed(reverse);
                }
            }
        }

        void setStopping(brakeType mode) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->setStopping(mode);
                }
            }
        }

        void setVelocity(double velocity, velocityUnits units) {
            for (int i = 0; i < 10; i++) {
                if (motors[i] != NULL) {
                    motors[i]->setVelocity(velocity, units);
                }
            }
        }
};