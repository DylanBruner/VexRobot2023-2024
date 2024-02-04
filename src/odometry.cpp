#include "auton/odometry.h"
#include <math.h>

Odometry::Odometry(double tl, double tr, double tb, vex::rotation* left, vex::rotation* right, vex::motor* middle){
    this->tl = tl;
    this->tr = tr;
    this->tb = tb;
    this->left = left;
    this->right = right;
    this->middle = middle;

    this->left->resetPosition();
    this->right->resetPosition();
}

double Odometry::getHeading(){
    // (this->tl + this->tr)
    return (( ( this->left->position(vex::deg) - this->right->position(vex::deg) ) /  (this->tl + this->tr)) * 180 / M_PI) / 45;
}