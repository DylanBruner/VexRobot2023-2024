#include "vex.h"

using namespace vex;

// Config stuffs ========================
const int ACCEL_STEP  = 8;
const int DECCEL_STEP = 200;
const double DEG_TO_DISTANCE = 1; // in the name...

// Drive TODO: Figure out real values to put here
const double driveKP = 0.5; // Error magnitude
// const double driveKI = 0.5;
const double driveKD = 0.5; // Decreases output if increasing rapidly

// Not-Config stuffs ====================

class PidController {
    public:
        PidController(motor lf, motor lm, motor lb, motor rf, motor rm, motor rb);

        void driveAsync(double distance, double max);
        void driveAsync(double distance);
        void setStopping(brakeType mode);
        void stopAuton();

    private:
        motor* leftFront;
        motor* leftMiddle;
        motor* leftBack;
        motor* rightFront;
        motor* rightMiddle;
        motor* rightBack;
        motor *motors[] = {leftFront, leftMiddle, leftBack, 
                           rightFront, rightMiddle, rightBack};

        double maxSpeed = 100;

        // for pid
        double startLeft = 0;
        double startRight = 0;

        int driveTarget = 0;
        int turnTarget = 0;
        int driveMode = 0; //1=drive, 2=turn, 0=nothing
        
        int slew(int speed, int lastSpeed);
        int driveTask();
        int left_drive(int vel);
        int right_drive(int vel);
}