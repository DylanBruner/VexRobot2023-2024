#include <string>
#include "vex.h"

using namespace std;

class AutonSelector {
    public:
        AutonSelector();
        void run(vex::controller* controller, vex::brain* Brain);
        AutonSelector& addAuton(void (*auton)(), string name);
        AutonSelector& setCompetitionMode(bool competition);
        AutonSelector& setDriver(void (*driver)());
    
    private:
        int autonCount = 0;
        bool competition = true;
        void (*autons[10])();
        string names[10];
        void (*driver)();
};