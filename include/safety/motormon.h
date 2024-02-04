#include "vex.h"
#include "config.h"
#include "robocontrol.h"
#include "driver/keybindmanager.h"
#include "util.h"
#include <string.h>
#include <map>

#pragma once


using namespace vex;

class NamedMotor {

    public:
        motor* m;
        std::string name;
        
        NamedMotor(motor* m, std::string name){
            this->m = m;
            this->name = name;
        }
};

class MotorMonitor {
    public:
        void start(){
            task t(internalTaskWrapper, this);
        }
    
    private:
        NamedMotor motors[8] = {
            NamedMotor(&leftFront, "Left Front"),
            NamedMotor(&leftMiddle, "Left Middle"),
            NamedMotor(&leftBack, "Left Back"),
            NamedMotor(&rightFront, "Right Front"),
            NamedMotor(&rightMiddle, "Right Middle"),
            NamedMotor(&rightBack, "Right Back"),
            NamedMotor(&flywheelMotor, "Flywheel"),
            NamedMotor(&intakeMotor, "Intake")
        };

        int internalTask(){
            bool cleared = false;
            while (!cleared){
                std::string motorsNotConnected = "";
                for (int i = 0; i < 8; i++){
                    if (!motors[i].m->installed()){
                        motorsNotConnected += motors[i].name + ", ";
                    }
                }
                if (motorsNotConnected != ""){
                    Controller1.Screen.clearScreen();
                    Controller1.Screen.setCursor(1, 1);
                    Controller1.Screen.print("Warning a motor is not connected!");
                    Controller1.Screen.newLine();
                    Controller1.Screen.print(motorsNotConnected.c_str());
                    Controller1.Screen.newLine();
                    Controller1.Screen.print("Press X to continue");
                    while (!Controller1.ButtonX.pressing()){
                        task::sleep(10);
                        Controller1.rumble("-");
                    }
                    cleared = true;
                }
                task::sleep(100);
            }
            return 0;
        }

        static int internalTaskWrapper(void* instance){
            return static_cast<MotorMonitor*>(instance)->internalTask();
        }
};