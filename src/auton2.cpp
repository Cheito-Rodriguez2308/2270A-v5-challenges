#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "api.h"
#include "pros/motors.h"

namespace vex {
  class brain;
}

// Brain should be defined by default



// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration


using namespace vex;

competition Competition;

// ----------------- Motors (your ports) -----------------
// Left: 11, 4, 20  (ONLY 20 reversed)
motor Lf(PORT11, gearSetting::ratio18_1, false);
motor Lm(PORT4,  gearSetting::ratio18_1, false);
motor Lb(PORT20, gearSetting::ratio18_1, true);   // inverted

// Right: 1, 19, 10 (ONLY 10 reversed)
motor Rf(PORT1,  gearSetting::ratio18_1, false);
motor Rm(PORT19, gearSetting::ratio18_1, false);
motor Rb(PORT10, gearSetting::ratio18_1, true);   // inverted

motor_group leftDrive(Lf, Lm, Lb);
motor_group rightDrive(Rf, Rm, Rb);

// Intake + conveyor
motor intake(PORT6,  gearSetting::ratio18_1, false);
motor conveyor(PORT7, gearSetting::ratio18_1, false);

motor_group intakeGroup(intake);
motor_group conveyorGroup(conveyor);

// ----------------- Pistons (3-wire ports A–H) -----------------
// CHANGE A/B to the real letters on your Brain
digital_out pistonArm(Brain.ThreeWirePort.A);      // piston brazo
digital_out pistonDescore(Brain.ThreeWirePort.B);  // piston descoring

// ----------------- Helpers -----------------
static void driveTime(int leftPct, int rightPct, int ms) {
  leftDrive.spin(fwd, leftPct, pct);
  rightDrive.spin(fwd, rightPct, pct);
  wait(ms, msec);
  leftDrive.stop(brakeType::brake);
  rightDrive.stop(brakeType::brake);
}

static void intakeTime(int intakePct, int conveyorPct, int ms) {
  intakeGroup.spin(fwd, intakePct, pct);
  conveyorGroup.spin(fwd, conveyorPct, pct);
  wait(ms, msec);
  intakeGroup.stop(brakeType::brake);
  conveyorGroup.stop(brakeType::brake);
}

// ----------------- Autonomous routine -----------------
void auton_tank_basic() {
  leftDrive.setStopping(brakeType::brake);
  rightDrive.setStopping(brakeType::brake);
  intakeGroup.setStopping(brakeType::brake);
  conveyorGroup.setStopping(brakeType::brake);

  // pistons default
  pistonArm.set(false);
  pistonDescore.set(false);

  //-----------------------start----------------------------
  // Drive forward while intaking
  intakeTime(100, 100, 300);
  driveTime(60, 60, 9A00);

  // Turn and intake
  intakeTime(80, 80, 200);
  driveTime(50, -50, 450);

  // Example piston arm pulse
  pistonArm.set(true);
  wait(200, msec);
  pistonArm.set(false);

  // Drive and conveyor
  intakeTime(60, 100, 300);
  driveTime(55, 55, 500);

  // Example piston descoring pulse
  pistonDescore.set(true);
  wait(200, msec);
  pistonDescore.set(false);

  // hold at end
  leftDrive.stop(brakeType::hold);
  rightDrive.stop(brakeType::hold);
}

// ----------------- VEXcode structure -----------------
void pre_auton() {
  
}

void autonomous() {
  auton_tank_basic();
}

void usercontrol() {
  while (true) {
    wait(20, msec);
  }
}

int main() {
  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}
