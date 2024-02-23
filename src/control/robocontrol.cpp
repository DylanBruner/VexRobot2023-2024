#include "control/robocontrol.h"

void setFrontArms(bool state){
    setLeftArm(state);
    setRightArm(state);
}

void liftPowerUp(){
    LiftUpPiston.set(true);
    LiftPowerDownPiston.set(true);
}

void idleLiftDown(){
    LiftUpPiston.set(false);
}

void powerLiftDown(){
    LiftUpPiston.set(false);
    LiftPowerDownPiston.set(false);
}

void setLeftArm(bool state){
    FrontLeftArmDown.set(state);
    FrontLeftArmUp.set(!state);
}

void setRightArm(bool state){
    FrontRightArmDown.set(state);
    FrontRightArmUp.set(!state);
}