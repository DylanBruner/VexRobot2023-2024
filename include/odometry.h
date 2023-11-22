#include "vex.h"

class Odometry {
    public:
        Odometry(double tl, double tr, double tb, vex::rotation* left, vex::rotation* right, vex::motor* middle);
        double getHeading();
        double getX();
        double getY();
    
    private:
        double tl, tr, tb;
        vex::rotation* left;
        vex::rotation* right;
        vex::motor* middle;
};