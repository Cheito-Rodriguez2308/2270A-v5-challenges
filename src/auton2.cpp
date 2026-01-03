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
pros::MotorGroup leftDrive({
  pros::Motor(11, pros::E_MOTOR_GEARSET_18, false),
  pros::Motor(4,  pros::E_MOTOR_GEARSET_18, false),
  pros::Motor(20, pros::E_MOTOR_GEARSET_18, false)
});

pros::MotorGroup rightDrive({
  pros::Motor(1,  pros::E_MOTOR_GEARSET_18, false),
  pros::Motor(19, pros::E_MOTOR_GEARSET_18, true),   //RM invertido
  pros::Motor(10, pros::E_MOTOR_GEARSET_18, false)
});

static void driveTime(int leftPct, int rightPct, int ms) {
  leftDrive.move(leftPct);
  rightDrive.move(rightPct);
  pros::delay(ms);
  leftDrive.brake();
  rightDrive.brake();
}

// ============================================================================
// Grupo de motores (y, x, z )
// y = leftDrive (velocidad)
// x = rightDrive (velocidad)
// z = tiempo en ms (tiempo que se mantiene la velocidad)
// ============================================================================

void auton_tank_basic() {
  driveTime(60,  60, 900);
  driveTime(50, -50, 450);
  driveTime(55,  55, 500);

  // Hold al final
  leftDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  rightDrive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
  leftDrive.brake();
  rightDrive.brake();
}
