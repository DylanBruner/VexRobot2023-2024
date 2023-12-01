#include "auton_selector.h"

AutonSelector::AutonSelector(){};

void AutonSelector::addAuton(void (*auton)(), string name) {
    autons[autonCount] = auton;
    names[autonCount] = name;
    autonCount++;
}

void AutonSelector::setCompetitionMode(bool competition) {
    this->competition = competition;
}

void AutonSelector::setDriver(void (*driver)()) {
    this->driver = driver;
}

void AutonSelector::run(vex::controller* controller, vex::brain* Brain) {
    int page = 0;
    int selected = 0;
    bool draw = true;

    int lastPress = 0;
    const int delay = 120; // How long a button must be held before it repeats

    // Three items / page
    // infinate number of pages, use mod to get the page & item

    while (true){
        if (draw){
            draw = false;
            controller->Screen.clearScreen();
            controller->Screen.setCursor(1, 1);

            page = selected / 3;

            // Draw the page
            for (int i = 0; i < 3; i++){
                int item = (page * 3) + i;
                if (names[item] != ""){
                    if (item == selected){ // Show a arrow before the selected option
                        controller->Screen.print("-> ");
                    } else {
                        controller->Screen.print("   ");
                    }
                    string name = names[item];
                    controller->Screen.print(&name[0]);
                    controller->Screen.newLine();
                }
            }
        }

        // Check if the buttons have been pressed with a <delay> second cooldown
        if (controller->ButtonUp.pressing() && lastPress + delay < Brain->Timer.time(vex::msec)){
            lastPress = Brain->Timer.time(vex::msec);
            if (selected > 0){
                selected--;
                draw = true;
            }
        } else if (controller->ButtonDown.pressing() && lastPress + delay < Brain->Timer.time(vex::msec)){
            lastPress = Brain->Timer.time(vex::msec);
            if (names[(page * 3) + selected + 1] != ""){
                selected++;
                draw = true;
            }
        } else if (controller->ButtonA.pressing()){
            controller->Screen.clearScreen();

            if (this->competition){
                vex::competition comp = vex::competition();
                comp.drivercontrol(driver);
                comp.autonomous(autons[(page * 3) + selected]);
            } else {
                autons[(page * 3) + selected]();
            }

            return;
        } else if (controller->ButtonB.pressing()){
            controller->rumble("....");
            controller->Screen.clearScreen();
            controller->Screen.print("Running prgm #%f", (page * 3) + selected);
            wait(1, vex::seconds);
            autons[(page * 3) + selected]();
            driver();
        }
    }
}