#include "auton/auton_selector.h"


AutonSelector::AutonSelector(){};

AutonSelector& AutonSelector::addAuton(void (*auton)(), string name) {
    autons[autonCount] = auton;
    names[autonCount] = name;
    autonCount++;
    return *this;
}

AutonSelector& AutonSelector::setDriver(void (*driver)()) {
    this->driver = driver;
    return *this;
}

void AutonSelector::run() {
    int selected = 0;
    bool draw = true;

    int lastPress = 0;
    const int repeatDelay = 120;

    while (true){
        if (draw){
            draw = false;
            Controller1.Screen.clearScreen();

            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print(("> " + names[selected]).c_str());
            for (int i = 1; i < 3; i++){
                Controller1.Screen.setCursor(i + 1, 1);
                Controller1.Screen.print(names[(selected + i) % autonCount].c_str());
            }
        }

        // Check if the buttons have been pressed with a <delay> second cooldown
        if (Controller1.ButtonUp.pressing() && lastPress + repeatDelay < Brain.Timer.time(vex::msec)){
            selected--;
            if (selected < 0){
                selected = autonCount - 1;
            }
            draw = true;
            lastPress = Brain.Timer.time(vex::msec);
        } else if (Controller1.ButtonDown.pressing() && lastPress + repeatDelay < Brain.Timer.time(vex::msec)){
            selected++;
            if (selected >= autonCount){
                selected = 0;
            }
            draw = true;
            lastPress = Brain.Timer.time(vex::msec);
        } else if (Controller1.ButtonA.pressing()){
            vex::competition comp = vex::competition();
            comp.drivercontrol(driver);
            comp.autonomous(autons[selected]);

            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Competition Mode");
            Controller1.Screen.setCursor(2, 1);
            Controller1.Screen.print(names[selected].c_str());
            return;
        } else if (Controller1.ButtonB.pressing()){
            Controller1.rumble("...");
            double start = Brain.Timer.time(vex::sec);
            this->autons[selected]();
            double end = Brain.Timer.time(vex::sec);

            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Auton Complete");
            Controller1.Screen.setCursor(2, 1);
            Controller1.Screen.print("Time: %fs", end - start);
            return;
        }
    }
}