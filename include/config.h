#include <vex.h>

// Devices ====================================================================
const int LEFT_FRONT = vex::PORT20;
const int LEFT_MIDDLE = vex::PORT19;
const int LEFT_BACK = vex::PORT18;
const int RIGHT_FRONT = vex::PORT12;
const int RIGHT_MIDDLE = vex::PORT13;
const int RIGHT_BACK = vex::PORT14;

const int INTAKE_PORT = vex::PORT15;

const int INERTIAL_PORT = vex::PORT16;


// Constants ==================================================================
const bool DEBUG = true;
const int TILE_CONST = 700;

// modes & states
const int DM_STRAIGHT = 0;
const int DM_TURN = 1;
const int DM_DISABLED = 2;

const int CATA_MANUAL = 0;
const int CATA_SEMI_AUTO = 1;
const int CATA_AUTO = 2;

// config
const double driveKP = 0.05; // for distance
const double turnKP = 0.075;
const double turnKI = 0.0002;
const double turnKD = 0.1;