/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2140C                                                     */
/*    Created:      9/18/2023, 7:31:18 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
brain       Brain;
controller  Controller1;
motor       RightMotor1(PORT4);
motor       RightMotor2(PORT5);
motor       RightMotor3(PORT6);
motor       LeftMotor1(PORT7, true);
motor       LeftMotor2(PORT8, true);
motor       LeftMotor3(PORT9, true);
motor       CataMotor(PORT10);
limit       CataSwitch(Brain.ThreeWirePort.A);

// define your global instances of motors and other devices here
bool switchPressed = false;

// create a task that handles catapult logic 
int cataTask() {
    while (true){
        if (Controller1.ButtonA.pressing()){
            if (switchPressed && !CataSwitch.pressing())
                switchPressed = false;
            CataMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        } else {
            CataMotor.stop(vex::brakeType::hold);
        }

        if (CataSwitch.pressing() && !switchPressed){
            switchPressed = true;
            CataMotor.stop(vex::brakeType::hold);
            task::sleep(1000);
        }
    }
    return(0);
}

int main() {
    // run the cata task
    vex::task t(cataTask);

    while(1) {
        int left = Controller1.Axis3.position();
        int right = Controller1.Axis2.position();

        LeftMotor1.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);
        LeftMotor2.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);
        LeftMotor3.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);

        RightMotor1.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
        RightMotor2.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
        RightMotor3.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    }
}