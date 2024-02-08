#include "vex.h"
#include "config.h"
#include "robocontrol.h"
#include "driver/keybindmanager.h"
#include "util.h"
#include <cstdlib>

#pragma once

extern bool liftUp;
extern bool backArm;
extern bool frontArms;
extern bool flywheelSpinning;

// variables for auto torque =======
extern int drivePowerMode; // 0 = auto, 1 = 100% power

const int efficiencySampleLength = 5;
extern double efficiencySamples[efficiencySampleLength];
extern int efficiencySampleIndex;
// =================================

int getDrivePower();
double getTemp();

extern int lastBatteryPercentage ;
extern int lastDriveMode;
extern int lastOutputPower;
extern int lastTemp;

void screenTick();
int driveHelper();
void driver();