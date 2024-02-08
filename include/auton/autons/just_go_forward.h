#include "vex.h"
#include "config.h"
#include "control/robocontrol.h"

#pragma once

void justGoForward(){
    drive(5, 5, 9);
    drive(-0.75, -0.75, 9);
}