#include "vex.h"
#include <string>
#include "auton.h"

using namespace std;

class AutonSelector {
    public:
        AutonSelector();
        AutonSelector(string names[], void(Auton::*callback[])());
        void addAuton(string name, void(Auton::*func)());
        void setDriver(void(*driver)());
        void setCompetitionMode(bool competitionMode);
        void setupDevices(vex::controller* c, vex::brain* b, vex::motor* cataMotor, vex::limit* cataSwitch);
        void run(bool competitionMode);
        void run();

    private:
        bool competitionMode = false;

        string names[20];
        void(Auton::*callback[20])();
        void(*driver)();

        // Devices cause idfk how to share devices between files
        vex::controller* Controller1;
        vex::brain* Brain;
        vex::motor* CataMotor;
        vex::limit* CataSwitch;
};