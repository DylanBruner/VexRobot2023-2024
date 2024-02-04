#include "vex.h"
#include "config.h"
#include "robocontrol.h"
#include "driver/keybindmanager.h"
#include "util.h"

#pragma once

int lastBatteryPercentage = 0;

bool liftUp = false;
bool backArm = false;
bool frontArms = false;

bool flywheelSpinning = false;

void driver(){
    stopDrive(); // Disable auton drive task

    // Lift Up / Idle Lift Down ==============================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        liftUp = !liftUp;
        if (liftUp){
            liftPowerUp();
        } else {
            idleLiftDown();
        }
    }).addCondition(Controller1.ButtonB, true).setName("Lift Up"));

    // Lift Power Down =======================================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        powerLiftDown();
    }).addCondition(Controller1.ButtonY, true).setName("Lift Down"));

    // Back Arm ==============================================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        backArm = !backArm;
        BackArm.set(backArm);
    }).addCondition(Controller1.ButtonA, true).setName("Back Arm"));

    // Front Arms ============================================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        frontArms = !frontArms;
        setFrontArms(frontArms);
    }).addCondition(Controller1.ButtonDown, true).setName("Front Arms"));

    // Flywheel Forward ======================================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        flywheelSpinning = !flywheelSpinning;
        if (flywheelSpinning){
            flywheelMotor.spin(fwd, 100, pct);
        } else {
            flywheelMotor.stop();
        }
    }).addCondition(Controller1.ButtonL1, true).setName("Flywheel Forward"));

    // Flywheel Reverse ======================================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        flywheelSpinning = !flywheelSpinning;
        if (flywheelSpinning){
            flywheelMotor.spin(reverse, 100, pct);
        } else {
            flywheelMotor.stop();
        }
    }).addCondition(Controller1.ButtonL2, true).setName("Flywheel Reverse"));

    // Intake ================================================================
    keybindManager.registerKeybinding(KeyBinding().onPressing([](){
        intakeMotor.spin(reverse, 100, pct);
    }).addCondition(Controller1.ButtonR1, true).setName("Intake Reverse"));

    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        intakeMotor.spin(fwd, 100, pct);
    }).addCondition(Controller1.ButtonR2, true).setName("Intake Forward"));

    keybindManager.registerKeybinding(KeyBinding().onPressing([](){
        intakeMotor.stop();
    }).addCondition(Controller1.ButtonR2, false).addCondition(Controller1.ButtonR1, false).setName("Intake Stop"));

    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);

    while(true){
        if (lastBatteryPercentage != Brain.Battery.capacity(percent)){
            lastBatteryPercentage = Brain.Battery.capacity(percent);
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Battery: %d%%", lastBatteryPercentage);
        }

        RightMotors.spin(fwd, scaleDrive(Controller1.Axis2.position(), DRIVE_SCALE_FACTOR), pct);
        LeftMotors.spin(fwd, scaleDrive(Controller1.Axis3.position(), DRIVE_SCALE_FACTOR), pct);        
    }
}