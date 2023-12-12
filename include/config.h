#include <vex.h>

// Devices ====================================================================
const int LEFT_FRONT = vex::PORT7;
const int LEFT_MIDDLE = vex::PORT21;
const int LEFT_BACK = vex::PORT10;
const int RIGHT_FRONT = vex::PORT8;
const int RIGHT_MIDDLE = vex::PORT19;
const int RIGHT_BACK = vex::PORT18;
const int CATA_PORT = vex::PORT6;
const int WINCH_PORT = vex::PORT4;

const int R_ENCODER = vex::PORT14;
const int CATA_ENCODER = vex::PORT17;
const int CATA_BALL_DISTANCE = vex::PORT3;


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