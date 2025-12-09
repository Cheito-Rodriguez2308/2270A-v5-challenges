#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "drive.hpp"
#include "motion.hpp"
#include "pros/apix.h"

// ======================================================
// MODULO: auton (implementacion simple)
//   - Usa solo dos primitivas de movimiento:
//       drive_mm_pid(dist_mm)
//       turn_imu_deg(delta_deg)
//   - No recalibra sensores.
//   - No usa Ramsete ni funciones duplicadas.
// ======================================================

// Seleccion por defecto: auton del lado derecho
AutonId g_auton_selected = AutonId::Right;

// Nombre legible para cada auton (se usa en el HUD del brain)
const char* auton_name(AutonId id) {
  switch (id) {
    case AutonId::Right:  return "RIGHT";
    case AutonId::Left:   return "LEFT";
  }
  return "UNKNOWN";
}

// ------------------------------------------------------
// Declaraciones de etapas
// ------------------------------------------------------

// RIGHT
static void auton_right_stage_preload(
  double FWD1_MM,
  double FWD2_MM,
  int    convPower,
  int    intakeFwd
);

static void auton_right_stage_platform(
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int    convPower
);

static void auton_right_stage_second_cycle(
  double FWD4_MM,
  double FWD5_MM,
  int    convPower,
  int    intakeFwd
);

// LEFT
static void auton_left_stage_preload(
  double FWD1_MM,
  double FWD2_MM,
  int    convPower,
  int    intakeFwd
);

static void auton_left_stage_platform(
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int    convPower
);

static void auton_left_stage_second_cycle(
  double FWD4_MM,
  double FWD5_MM,
  int    convPower,
  int    intakeFwd
);

// ------------------------------------------------------
// Despachador principal
// ------------------------------------------------------
void autonomous_routine() {
  switch (g_auton_selected) {
    case AutonId::Right:
      auton_right();
      break;
    case AutonId::Left:
      auton_left();
      break;
    default:
      auton_right();
      break;
  }
}

// ======================================================
// AUTON RIGHT
// ======================================================

void auton_right() {
  const uint32_t t0 = pros::millis();

  drive.set_brake(pros::E_MOTOR_BRAKE_HOLD);

  const double FWD1_MM   = 20.0 * 25.4;
  const double FWD2_MM   = 9.5  * 25.4;
  const double REV_1_MM  = 31.0 * 25.4;
  const double BACK_MM   = 17.0 * 25.4;
  const double FWD7_MM   = 8.5  * 25.4;

  const double FWD4_MM   = 7.0  * 25.4;
  const double FWD5_MM   = 15.0 * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  auton_right_stage_preload(
    FWD1_MM,
    FWD2_MM,
    convPower,
    intakeFwd
  );

  auton_right_stage_platform(
    REV_1_MM,
    BACK_MM,
    FWD7_MM,
    convPower
  );

  auton_right_stage_second_cycle(
    FWD4_MM,
    FWD5_MM,
    convPower,
    intakeFwd
  );

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) {
    pros::delay(15000 - elapsed);
  }

  intake.move(0);
  conveyor.move(0);
  drive.set_percent(0, 0);
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);
}

// ETAPA 1: salida de start y uso del preload
static void auton_right_stage_preload(
  double FWD1_MM,
  double FWD2_MM,
  int    convPower,
  int    intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  // Adelante 20"
  drive_mm_pid(FWD1_MM);
  pros::delay(60);

  // Giro 50° izquierda
  turn_imu_deg(+50.0);
  pros::delay(100);

  conveyor.move(0);
  pros::delay(60);

  // Adelante 9.5"
  drive_mm_pid(FWD2_MM);
  pros::delay(80);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);
  pros::delay(80);
}

// ETAPA 2: giro a plataforma y puntuacion con piston
static void auton_right_stage_platform(
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int    convPower
) {
  // Reversa 31.0"
  drive_mm_pid(-REV_1_MM);

  // Giro 90° izquierda
  turn_imu_deg(+90.0);

  piston_1.set_value(true);
  pros::delay(300);

  // Adelante 8.5"
  drive_mm_pid(FWD7_MM);

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);

  // Reversa 17"
  drive_mm_pid(-BACK_MM);
}

// ETAPA 3: segundo ciclo / limpieza
static void auton_right_stage_second_cycle(
  double FWD4_MM,
  double FWD5_MM,
  int    convPower,
  int    intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  conveyor.move(0);
  intake.move(0);

  // Adelante 7"
  drive_mm_pid(FWD4_MM);

  // Giro 70° derecha
  turn_imu_deg(-70.0);

  // Adelante 15"
  drive_mm_pid(FWD5_MM);
}

// ======================================================
// AUTON LEFT
// ======================================================

void auton_left() {
  const uint32_t t0 = pros::millis();

  drive.set_brake(pros::E_MOTOR_BRAKE_HOLD);

  const double FWD1_MM   = 20.0 * 25.4;
  const double FWD2_MM   = 9.5  * 25.4;
  const double REV_1_MM  = 31.0 * 25.4;
  const double BACK_MM   = 17.0 * 25.4;
  const double FWD7_MM   = 8.5  * 25.4;

  const double FWD4_MM   = 7.0  * 25.4;
  const double FWD5_MM   = 15.0 * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  auton_left_stage_preload(
    FWD1_MM,
    FWD2_MM,
    convPower,
    intakeFwd
  );

  auton_left_stage_platform(
    REV_1_MM,
    BACK_MM,
    FWD7_MM,
    convPower
  );

  auton_left_stage_second_cycle(
    FWD4_MM,
    FWD5_MM,
    convPower,
    intakeFwd
  );

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) {
    pros::delay(15000 - elapsed);
  }

  intake.move(0);
  conveyor.move(0);
  drive.set_percent(0, 0);
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);
}

// ETAPA 1: salida de start y uso del preload
static void auton_left_stage_preload(
  double FWD1_MM,
  double FWD2_MM,
  int    convPower,
  int    intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  // Adelante 20"
  drive_mm_pid(FWD1_MM);
  pros::delay(60);

  // Giro 50° derecha
  turn_imu_deg(-50.0);
  pros::delay(100);

  conveyor.move(0);
  pros::delay(60);

  // Adelante 9.5"
  drive_mm_pid(FWD2_MM);
  pros::delay(80);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);
  pros::delay(80);
}

// ETAPA 2: giro a plataforma y puntuacion con piston
static void auton_left_stage_platform(
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int    convPower
) {
  // Reversa 31.0"
  drive_mm_pid(-REV_1_MM);

  // Giro 90° derecha
  turn_imu_deg(-90.0);
  pros::delay(120);

  piston_1.set_value(true);
  pros::delay(320);

  // Adelante 8.5"
  drive_mm_pid(FWD7_MM);
  pros::delay(100);

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(120);

  // Reversa 17"
  drive_mm_pid(-BACK_MM);
  pros::delay(100);
}

// ETAPA 3: segundo ciclo / limpieza
static void auton_left_stage_second_cycle(
  double FWD4_MM,
  double FWD5_MM,
  int    convPower,
  int    intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(500);
  pros::delay(2000);

  // Adelante 7"
  drive_mm_pid(FWD4_MM);

  // Giro 70° izquierda
  turn_imu_deg(+70.0);

  // Adelante 15"
  drive_mm_pid(FWD5_MM);
}
