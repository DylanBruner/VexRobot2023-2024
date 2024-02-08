#include <vex.h>
#include "config.h"

#pragma once

using namespace vex;

extern double targetPower;
extern double drivePower;
extern double leftTarget;
extern double rightTarget;
extern double lastError;
extern double targetTurn;
extern double integral;
extern int driveMode;

void stopDrive();

void resetTracking();

// this needs code added to it that makes sure the values stay near each other
void _spinLeft(directionType dir, double volts);

void _spinRight(directionType dir, double volts);

bool isDriving();


extern double startErrorLeft;
extern double startErrorRight;
extern double lastSpeed;

// Main drive task
int autonDriveTask();


void driveAsync(double left, double right, double power);

void turnAsync(double left, double right, double power);

void turn(double left, double right, double power);

// Auto-stops when velocity drops to 0
bool drive(double left, double right, double power);
bool drive(int left, int right);