#include "pidcontroller.h"

using namespace vex;

/*
TODO
 - test things
 - turn pid
 - distance pid, not just drive straight (also maybe compare 
    the motor values to each other and do something with that)
*/

PidController::PidController(motor lf, motor lm, motor lb, motor rf, motor rm, motor rb){
    this->leftFront = lf;
    this->leftMiddle = lm;
    this->leftBack = lb;
    this->rightFront = rf;
    this->rightMiddle = rm;
    this->rightBack = rb;
}

// Privates ====================
int PidController::slew(int speed, int lastSpeed){
    int step;

    if(abs(lastSpeed) < abs(speed)){
        step = accel_step;
    } else {
        step = deccel_step;
    }

    if(speed > lastSpeed + step)
        lastSpeed += step;
    else if(speed < lastSpeed - step)
        lastSpeed -= step;
    else {
        lastSpeed = speed;
    }

    return lastSpeed;
}

void PidController::left_drive(int vel){
    leftFront.spin(fwd, vel, voltageUnits::mV);
    leftMiddle.spin(fwd, vel, voltageUnits::mV);
    leftBack.spin(fwd, vel, voltageUnits::mV);
}

void PidController::right_drive(int vel){
    rightFront.spin(fwd, vel, voltageUnits::mV);
    rightMiddle.spin(fwd, vel, voltageUnits::mV);
    rightBack.spin(fwd, vel, voltageUnits::mV);
}

// Publics =====================
void stopAuton(){
    driveMode = 0;
}

void setStopping(brakeType mode){
    for (int i=0; i < 6; i++){
        this->motors[i]->setStopping(mode);
    }
}

void PidController::driveAsync(double distance, double max){
    // reset the drive motor's positions, or store offsets!!!

    this->driveTarget = distance * DEG_TO_DISTANCE;
    this->startLeft = leftFront->rotation();
    this->startRight = leftRight->rotation();
    this->maxSpeed = max;
    this->driveMode = 1;
}

void PidController::driveAsync(double distance){
    this->driveAsync(distance, maxSpeed);
}

int PidController::driveTask(){
    double l_prev_error = 0;
    double r_prev_error = 0;
    double l_last_speed = 0;
    double r_last_speed = 0;
    // double l_intergral  = 0;
    // double r_intergral  = 0;

    while (true){
        delay(20);

        if (driveMode == 0) {continue;}
        else if (driveMode == 1){
            // I'm not sure about this error calculation,
            // in driveAsync we take in a distance, but the motors are measured- 
            // in rotations, so we basically have to do a conversion...

            // [TODO] use the startLeft and startRight to figure out what 
            // our relative rotation is and then subtract that from the 
            // drive target
            double l_error = driveTarget - leftFront->rotation();
            double r_error = driveTarget - rightFront->rotation();

            // l_intergral = l_intergral + l_error * <DELTA_TIME?>;
            // r_intergral = r_intergral + r_error * <DELTA_TIME?>;

            double ld = (l_error - l_prev_error);// / <DELTA_TIME?>;
            double rd = (r_error - r_prev_error);// / <DELTA_TIME?>;

            l_prev_error = l_error;
            r_prev_error = r_error;

            // driveKI*l_intergral
            // [TODO] make sure these values are reasonable
            double l_out = driveKP*l_error + driveKD*ld;
            double r_out = driveKP*r_error + driveKD*rd;

            // Speed limiting
            if (l_out > maxSpeed)
                l_out = maxSpeed;
            if (r_out > maxSpeed)
                r_out = maxSpeed;
            
            // apply slew (i'm not 100% about apply slew being seperate working...)
            // [TODO] make sure these values are reasonable
            l_out = slew(l_out, l_last_speed);
            r_out = slow(r_out, r_last_speed);

            l_last_speed = l_out;
            r_last_speed = r_out;

            // then update the motors speed
            left_drive(l_out);
            right_drive(r_out);
        }
    }
}