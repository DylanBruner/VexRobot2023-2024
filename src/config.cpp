#include "config.h"

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
digital_out BackArm(Brain.ThreeWirePort.C); // 
digital_out FrontLeftArmDown(Brain.ThreeWirePort.G); //
digital_out FrontLeftArmUp(Brain.ThreeWirePort.H); //
digital_out FrontRightArmDown(Brain.ThreeWirePort.D); // 
digital_out FrontRightArmUp(Brain.ThreeWirePort.B); //

digital_out LiftPowerDownPiston(Brain.ThreeWirePort.E);
digital_out LiftUpPiston(Brain.ThreeWirePort.F);