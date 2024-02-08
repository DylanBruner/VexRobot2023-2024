#include "robocontrol.h"

void setFrontArms(bool state){
    FrontArmsOne.set(state);
    FrontArmsTwo.set(!state);
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