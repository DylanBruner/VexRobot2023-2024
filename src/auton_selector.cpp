#include "auton/auton_selector.h"

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
            controller->Screen.print("> ");
            controller->Screen.print(names[selected].c_str());
            for (int i = 1; i < 3; i++){
                controller->Screen.setCursor(i + 1, 1);
                controller->Screen.print(names[(selected + i) % autonCount].c_str());
            }
        }

        // Check if the buttons have been pressed with a <delay> second cooldown
        if (controller->ButtonUp.pressing() && lastPress + delay < Brain->Timer.time(vex::msec)){
            selected--;
            if (selected < 0){
                selected = autonCount - 1;
            }
            draw = true;
            lastPress = Brain->Timer.time(vex::msec);
        } else if (controller->ButtonDown.pressing() && lastPress + delay < Brain->Timer.time(vex::msec)){
            selected++;
            if (selected >= autonCount){
                selected = 0;
            }
            draw = true;
            lastPress = Brain->Timer.time(vex::msec);
        } else if (controller->ButtonA.pressing()){
            vex::competition comp = vex::competition();
            comp.drivercontrol(driver);
            comp.autonomous(autons[selected]);
            controller->Screen.clearScreen();
            controller->Screen.setCursor(1, 1);
            controller->Screen.print("Competition Mode");
            controller->Screen.setCursor(2, 1);
            controller->Screen.print(names[selected].c_str());
            return;
        } else if (controller->ButtonB.pressing()){
            controller->rumble("...");
            double start = Brain->Timer.time(vex::sec);
            this->autons[selected]();
            double end = Brain->Timer.time(vex::sec);
            controller->Screen.clearScreen();
            controller->Screen.setCursor(1, 1);
            controller->Screen.print("Auton Complete");
            controller->Screen.setCursor(2, 1);
            controller->Screen.print("Time: ");
            controller->Screen.print(end - start);
            controller->Screen.print("s");
            return;
        }
    }
}