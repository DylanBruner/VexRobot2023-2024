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

extern brain       Brain;
extern controller  Controller1;

// Drive motors
extern int32_t rightPorts[];
extern int32_t leftPorts[];

extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightFront;
extern motor rightMiddle;
extern motor rightBack;
extern BetterMotorGroup RightMotors;
extern BetterMotorGroup LeftMotors;

extern motor flywheelMotor;
extern motor intakeMotor;

// Sensors
extern inertial Inertial;

// Three wire ports
extern digital_out FrontArmsOne;
extern digital_out BackArm;

extern digital_out FrontArmsTwo;
extern digital_out LiftPowerDownPiston;
extern digital_out LiftUpPiston;