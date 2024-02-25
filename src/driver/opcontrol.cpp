#include "driver/opcontrol.h"

bool liftUp = false;
bool backArm = false;
bool frontArms = false;
bool flywheelSpinning = false;

// variables for auto torque =======
int drivePowerMode = 0; // 0 = auto, 1 = 100% power

double efficiencySamples[efficiencySampleLength];
int efficiencySampleIndex = 0;
// =================================

int getDrivePower(){
    if (drivePowerMode == 1) return 100;

    double avgEfficiency = 0;
    for (int i = 0; i < efficiencySampleLength; i++){
        avgEfficiency += efficiencySamples[i];
    }
    avgEfficiency /= efficiencySampleLength;

    return avgEfficiency <= 5 ? 100 : 75;
}

int getActiveMotors(){
    return leftFront.installed() + leftMiddle.installed() + leftBack.installed() + 
           rightFront.installed() + rightMiddle.installed() + rightBack.installed();
}

double getTemp(){
    return (leftFront.temperature(celsius) + leftMiddle.temperature(celsius) + leftBack.temperature(celsius) + 
            rightFront.temperature(celsius) + rightMiddle.temperature(celsius) + rightBack.temperature(celsius)) / getActiveMotors();
}

int lastBatteryPercentage = 0;
int lastDriveMode = -1;
int lastOutputPower = 0;
int lastTemp = 0;
void screenTick(){
    if (driveMode != DM_DRIVER) return;

    if (lastBatteryPercentage != Brain.Battery.capacity() || lastDriveMode != drivePowerMode || lastOutputPower != getDrivePower() || lastTemp != getTemp()){
        lastBatteryPercentage = Brain.Battery.capacity();
        lastDriveMode = drivePowerMode;
        lastOutputPower = getDrivePower();
        lastTemp = getTemp();

        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("Battery: %d%% | [%.0fC]    ", Brain.Battery.capacity(), Brain.Battery.temperature(celsius));
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("D: %s (%d%%) [%.0fC]    ", drivePowerMode == 0 ? "Auto" : "Full", getDrivePower(), getTemp());
    }
}

int driveHelper(){
    Controller1.Screen.clearScreen();
    
    while (true){
        task::sleep(250);
        if (driveMode != DM_DRIVER) continue;

        screenTick();

        // make sure the motors are actually trying to move
        int activeMotors = (leftFront.current() > 0.1) + (leftMiddle.current() > 0.1) + (leftBack.current() > 0.1) +
                           (rightFront.current() > 0.1) + (rightMiddle.current() > 0.1) + (rightBack.current() > 0.1);

        if (activeMotors == 0) continue;

        double efficiency = (leftFront.efficiency() + leftMiddle.efficiency() + leftBack.efficiency() + 
                             rightFront.efficiency() + rightMiddle.efficiency() + rightBack.efficiency()) / activeMotors;
            
        efficiencySamples[efficiencySampleIndex] = efficiency;
        efficiencySampleIndex = (efficiencySampleIndex + 1) % efficiencySampleLength;
    }
}

void driver(){
    stopDrive(); // Disable auton drive task
    task t(driveHelper); // Start driver control task

    intakeMotor.spinFor(200, degrees, 100, velocityUnits::pct); // drop the intake

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

    // Turbo Mode ===========================================================
    keybindManager.registerKeybinding(KeyBinding().onPressed([](){
        drivePowerMode = !drivePowerMode;
        if (drivePowerMode == 1) Controller1.rumble("-");
        else Controller1.rumble(".");
    }).addCondition(Controller1.ButtonRight, true).setName("Turbo Mode"));

    LeftMotors.setStopping(brake);
    RightMotors.setStopping(brake);

    Brain.Timer.reset();

    while(true){
        driveMode = DM_DRIVER;

        if (Brain.Timer.time(timeUnits::msec) > 45000){
            drivePowerMode = 1;
        }

        int power = getDrivePower();
        LeftMotors.setMaxTorque(power, percent);
        RightMotors.setMaxTorque(power, percent);

        if (std::abs(Controller1.Axis2.position()) >= 2){
            RightMotors.spin(fwd, scaleDrive(Controller1.Axis2.position(), DRIVE_SCALE_FACTOR) / 100 * 12, volt);
        } else {
            RightMotors.stop(brake);
        }

        if (std::abs(Controller1.Axis3.position()) >= 2){
            LeftMotors.spin(fwd, scaleDrive(Controller1.Axis3.position(), DRIVE_SCALE_FACTOR) / 100 * 12, volt);
        } else {
            LeftMotors.stop(brake);
        }
    }
}