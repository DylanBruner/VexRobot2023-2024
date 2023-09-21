#include "vex.h"
#include <string>
#include "auton.h"

using namespace std;

class AutonSelector {
    public:
        AutonSelector();
        AutonSelector(string names[], void(Auton::*callback[])());
        void addAuton(string name, void(Auton::*func)());
        void setupDevices(vex::controller* c, vex::brain* b, vex::motor* cataMotor, vex::limit* cataSwitch);
        void run(bool competitionMode);
        void run();

    private:
        string names[10];
        void(Auton::*callback[10])();

        // Devices cause idfk how to share devices between files
        vex::controller* Controller1;
        vex::brain* Brain;
        vex::motor* CataMotor;
        vex::limit* CataSwitch;
};