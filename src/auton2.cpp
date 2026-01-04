#include "main.h" 
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "api.h"
#include "pros/motors.h"
//===========================================================================
// Hay que poner los puertos actualizados
// motor group para simplificar el codigo
//===========================================================================
pros::MotorGroup leftDrive({11, 4, 20});

pros::MotorGroup rightDrive({1, 19, 10});

pros::MotorGroup intakeGroup({6});

pros::MotorGroup conveyorGroup({7});

static void driveTime(int leftPct, int rightPct, int ms) {
  leftDrive.move(leftPct);
  rightDrive.move(rightPct);
  pros::delay(ms);
  leftDrive.brake();
  rightDrive.brake();
}

static void intakeTime(int intakePct, int conveyorPct, int ms) {
  intakeGroup.move(intakePct);
  conveyorGroup.move(conveyorPct);
  pros::delay(ms);
  intakeGroup.brake();
  conveyorGroup.brake();
}

// ============================================================================
// Grupo de motores (y, x, z )
// y = leftDrive (velocidad)
// x = rightDrive (velocidad)
// z = tiempo en ms (tiempo que se mantiene la velocidad)
// ============================================================================

// ============================================================================
// Grupo de motores intakeTime (a, b, z)
// a = intakeGroup (velocidad: positivo=intake, negativo=outtake)
// b = conveyorGroup (velocidad: positivo=forward, negativo=reverse)
// z = tiempo en ms (tiempo que se mantiene la velocidad)
// ============================================================================

void auton_tank_basic() {
  // Drive forward while intaking
  intakeTime(100, 100, 300);  // intake and conveyor at 100%, for 300ms
  driveTime(60,  60, 900);
  
  // Turn and intake
  intakeTime(80, 80, 200);
  driveTime(50, -50, 450);
  
  // Drive and conveyor
  intakeTime(60, 100, 300);
  driveTime(55,  55, 500);

  // Stop everything
  intakeGroup.brake();
  conveyorGroup.brake();
  leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
