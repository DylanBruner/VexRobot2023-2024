#include "vex.h"
#include "config.h"
#include "robocontrol.h"

#pragma once

void goBackwardsAuton(){
    drive(-2, -2, 12);
    drive(1, 1, 6);
}