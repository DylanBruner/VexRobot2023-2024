#include <vex.h>

#include "better_motor_group.h"

#pragma once

// Devices ====================================================================
const int LEFT_FRONT = vex::PORT20;
const int LEFT_MIDDLE = vex::PORT19;
const int LEFT_BACK = vex::PORT18;
const int RIGHT_FRONT = vex::PORT12;
const int RIGHT_MIDDLE = vex::PORT13;
const int RIGHT_BACK = vex::PORT14;

const int INTAKE_PORT = vex::PORT15;
const int FLYWHEEL_PORT = vex::PORT17;

const int INERTIAL_PORT = vex::PORT16;


// Constants ==================================================================
const bool DEBUG = true;
const int TILE_CONST = 700;
const int TURN_CONST = 3.99;

const int DRIVE_SCALE_FACTOR = 3; // odd number!

// modes & states
const int DM_STRAIGHT = 0;
const int DM_TURN = 1;
const int DM_DISABLED = 2;
const int DM_DRIVER = 3;

const int CATA_MANUAL = 0;
const int CATA_SEMI_AUTO = 1;
const int CATA_AUTO = 2;

// config
// const double driveKP = 0.05; // for distance
const double driveKP = 0.15; // for distance

const double turnKP = 0.075;
const double turnKI = 0.0002;
const double turnKD = 0.1;

// =============================================================================
// =============================================================================
// =============================== [ELECTRONICS] ===============================
// =============================================================================
// =============================================================================

brain       Brain;
controller  Controller1;

// Drive motors
int32_t rightPorts[] = {RIGHT_FRONT, RIGHT_MIDDLE, RIGHT_BACK};
int32_t leftPorts[] = {LEFT_FRONT, LEFT_MIDDLE, LEFT_BACK};

motor leftFront = motor(LEFT_FRONT, gearSetting::ratio6_1);
motor leftMiddle = motor(LEFT_MIDDLE, gearSetting::ratio6_1);
motor leftBack = motor(LEFT_BACK, gearSetting::ratio6_1);
motor rightFront = motor(RIGHT_FRONT, gearSetting::ratio6_1);
motor rightMiddle = motor(RIGHT_MIDDLE, gearSetting::ratio6_1);
motor rightBack = motor(RIGHT_BACK, gearSetting::ratio6_1);
BetterMotorGroup RightMotors(rightPorts, 3);
BetterMotorGroup LeftMotors(leftPorts, 3);

motor flywheelMotor = motor(FLYWHEEL_PORT, gearSetting::ratio6_1, true);
motor intakeMotor = motor(INTAKE_PORT, gearSetting::ratio6_1, true);

// Sensors
inertial Inertial(INERTIAL_PORT);

// Three wire ports
digital_out FrontArmsOne(Brain.ThreeWirePort.F);
digital_out BackArm(Brain.ThreeWirePort.E);

digital_out FrontArmsTwo(Brain.ThreeWirePort.D);
digital_out LiftPowerDownPiston(Brain.ThreeWirePort.G);
digital_out LiftUpPiston(Brain.ThreeWirePort.H);