#include <string>
#include "vex.h"

using namespace std;

class AutonSelector {
    public:
        AutonSelector();
        void run(vex::controller* controller, vex::brain* Brain);
        void addAuton(void (*auton)(), string name);
        void setCompetitionMode(bool competition);
        void setDriver(void (*driver)());
    
    private:
        int autonCount = 0;
        bool competition = false;
        void (*autons[10])();
        string names[10];
        void (*driver)();
};