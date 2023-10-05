/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       2140C                                                     */
/*    Created:      9/18/2023, 7:31:18 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "better_motor_group.h"
#include "autonselector.h"
#include "auton.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
brain       Brain;
controller  Controller1;

// Drive motors
int32_t rightPorts[] = {PORT20, PORT17, PORT9};//20
int32_t leftPorts[] = {PORT6, PORT12, PORT11};
BetterMotorGroup RightMotors(rightPorts, 3);
BetterMotorGroup LeftMotors(leftPorts, 3);

motor CataMotor(PORT5);
motor WinchMotor(PORT3);
limit CataSwitch(Brain.ThreeWirePort.A);

bool switchPressed = false;

// cata logic task cause i dont feel like writing real code
int cataTask() {
    while (true){
        if (Controller1.ButtonA.pressing()){
            if (switchPressed && !CataSwitch.pressing())
                switchPressed = false;
            CataMotor.spin(fwd, 100, pct);
        } else {
            CataMotor.stop(hold);
        }

        if (CataSwitch.pressing() && !switchPressed){
            switchPressed = true;
            CataMotor.stop(hold);
            task::sleep(1000);
        }
    }
    return 0;
}

void driver(){
    // Start the cata task
    task t(cataTask);

    while(1) {
        RightMotors.spin(fwd, Controller1.Axis2.position(), pct);
        LeftMotors.spin(fwd, Controller1.Axis3.position(), pct);
    }
}

int main() {
    // Configure some other stuff
    LeftMotors.setReversed(true);
    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);
    
    AutonSelector selector = AutonSelector();
    // AutonSelector selector();
    selector.setCompetitionMode(false);
    selector.setupDevices(&Controller1, &Brain, &CataMotor, &CataSwitch);
    selector.setDriver(&driver);
    selector.addAuton("Left Auton", &Auton::leftAuton);
    selector.addAuton("Left Auton1", &Auton::leftAuton);
    selector.addAuton("Left Auton2", &Auton::leftAuton);
    selector.addAuton("Left Auton3", &Auton::leftAuton);
    selector.addAuton("Left Auton4", &Auton::leftAuton);
    
    selector.run();
}