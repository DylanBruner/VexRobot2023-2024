#include "auton.h"
#include "vex.h"
#include <functional>

Auton::Auton(vex::controller* c, vex::brain* b, vex::motor* cataMotor, vex::limit* cataSwitch) {
    Controller1 = c;
    Brain = b;
    CataMotor = cataMotor;
    CataSwitch = cataSwitch;
}

void Auton::leftAuton() {
}