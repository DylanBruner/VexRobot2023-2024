#pragma once
#include <functional>
#include "vex.h"

class Auton {
    public:
        Auton(vex::controller* c, vex::brain* b, vex::motor* cataMotor, vex::limit* cataSwitch);
        void leftAuton();

    private:
        vex::controller* Controller1;
        vex::brain* Brain;
        vex::motor* CataMotor;
        vex::limit* CataSwitch;
};