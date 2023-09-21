#include "autonselector.h"
#include "auton.h"
#include "vex.h"
#include <string>

using namespace vex;

extern const controller* Controller1;

AutonSelector::AutonSelector() {}
AutonSelector::AutonSelector(string names[], void(Auton::*callback[])()) {
    for (int i = 0; i < 10; i++) {
        this->names[i] = names[i];
        this->callback[i] = callback[i];
    }
}

void AutonSelector::setCompetitionMode(bool competitionMode) {
    this->competitionMode = competitionMode;
}

void AutonSelector::setupDevices(controller* c, brain* b, motor* cataMotor, limit* cataSwitch) {
    Controller1 = c;
    Brain = b;
    CataMotor = cataMotor;
    CataSwitch = cataSwitch;
}

void AutonSelector::setDriver(void(*driver)()) {
    this->driver = driver;
}

void AutonSelector::addAuton(string name, void(Auton::*callback)()) {
    for (int i = 0; i < 10; i++) {
        if (names[i] == "") {
            names[i] = name;
            this->callback[i] = callback;
            return;
        }
    }
}

void AutonSelector::run(bool competitionMode){
    int page = 0;
    int selected = 0;
    bool draw = true;

    int lastPress = 0;
    const int delay = 120;

    // Three items / page
    // infinate number of pages, use mod to get the page & item

    while (true){
        if (draw){
            draw = false;
            Controller1->Screen.clearScreen();
            Controller1->Screen.setCursor(1, 1);

            page = selected / 3;

            // Draw the page
            for (int i = 0; i < 3; i++){
                int item = (page * 3) + i;
                if (names[item] != ""){
                    if (item == selected){
                        Controller1->Screen.print("-> ");
                    } else {
                        Controller1->Screen.print("   ");
                    }
                    string name = names[item];
                    Controller1->Screen.print(&name[0]);
                    Controller1->Screen.newLine();
                }
            }
        }

        // Check for button presses
        if (Controller1->ButtonUp.pressing() && lastPress + delay < Brain->Timer.time(msec)){
            lastPress = Brain->Timer.time(msec);
            if (selected > 0){
                selected--;
                draw = true;
            }
        } else if (Controller1->ButtonDown.pressing() && lastPress + delay < Brain->Timer.time(msec)){
            lastPress = Brain->Timer.time(msec);
            if (names[(page * 3) + selected + 1] != ""){
                selected++;
                draw = true;
            }
        } else if (Controller1->ButtonA.pressing()){
            Controller1->Screen.clearScreen();
            // Auton func
            (Auton(this->Controller1, this->Brain, this->CataMotor, this->CataSwitch).*callback[(page * 3) + selected])();
            // Driver func
            this->driver();
            return;
        }
    }
}

void AutonSelector::run(){
    AutonSelector::run(false);
}