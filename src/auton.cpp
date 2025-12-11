#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "drive.hpp"
#include "motion.hpp"
#include "pros/apix.h"
#include <algorithm>
#include <cmath>

// ======================================================
// Selección de autónomo
// ======================================================

AutonId g_auton_selected = AutonId::Right;

const char* auton_name(AutonId id) {
  switch (id) {
    case AutonId::Right: return "RIGHT";
    case AutonId::Left:  return "LEFT";
  }
  return "UNKNOWN";
}

// ======================================================
// Helpers internos
// ======================================================

namespace {
  
  constexpr double AUTON_IDEAL_VOLTAGE = 12.6;

  double auton_get_voltage() {
    return pros::battery::get_voltage() / 1000.0;
  }

  double auton_voltage_comp() {
    double volt = auton_get_voltage();
    if (volt <= 0.0) return 1.0;
    double comp = AUTON_IDEAL_VOLTAGE / volt;
    if (comp > 1.10) comp = 1.10;
    if (comp < 0.95) comp = 0.95;
    return comp;
  }

} // namespace

// ======================================================
// Despachador principal
// ======================================================

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
//   Versión C++ del auton estable en Python
// ======================================================

void auton_right() {
  const double comp = auton_voltage_comp();
  const uint32_t t0 = pros::millis();

  auto autopct = [comp](int p) {
    return clamp_pct(static_cast<int>(p * comp));
  };

  Drive drive;
  drive.set_brake(pros::E_MOTOR_BRAKE_HOLD);

  const int drive_base = cfg.AUTO_DRIVE_PCT;
  const int turn_base  = cfg.AUTO_TURN_PCT;

  const int drive_pct  = autopct(drive_base);
  const int turn_fast  = autopct(turn_base);
  const int turn_slow  = std::max(10, static_cast<int>(turn_fast * 0.60));

  // Distancias en mm (igual que Python)
  const double FWD1_MM = 15.0  * 25.4;
  const double FWD2_MM = 21.9  * 25.4;
  const double FWD3_MM = 10.0  * 25.4;
  const double BACK_MM = 18.0  * 25.4;
  const double FWD7_MM = 6.0   * 25.4;
  const double FWD4_MM = 12.0  * 25.4;
  const double FWD5_MM = 18.0  * 25.4;
  const double FWD6_MM = 13.0  * 25.4;

  const int convPower  = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd  = cfg.INTAKE_FWD_PCT * 127 / 100;

  // 1) Adelante 15"
  drive_straight_mm(
    FWD1_MM,
    drive_pct,
    0.40,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(100);

  // 2) Giro ~43° derecha
  //    Positivo = izquierda, negativo = derecha
  turn_imu_deg_2stage(
    -43.0,
    turn_fast,
    turn_slow,
    0.92,
    120
  );
  pros::delay(100);

  // 4) Adelante ~21.9"
  drive_straight_mm(
    FWD2_MM,
    drive_pct,
    0.30,
    120.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(80);

  // 5) Giro ~42° derecha
  turn_imu_deg_2stage(
    -42.0,
    turn_fast,
    turn_slow,
    0.92,
    120
  );
  pros::delay(120);

  // 6) Piston principal ON
  piston_1.set_value(true);
  pros::delay(320);

  // 7) Conveyor REVERSE y adelante 10"
  conveyor.move(-convPower);
  drive_straight_mm(
    FWD3_MM,
    autopct(45),
    0.30,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(80);

  // 8) Espera 2000 ms para que caigan los anillos
  pros::delay(2000);

  // 9) Adelante 6"
  drive_straight_mm(
    FWD7_MM,
    drive_pct,
    0.40,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  // 10) Piston OFF
  piston_1.set_value(false);

  // 11) Reversa 18"
  drive_straight_mm(
    -BACK_MM,
    drive_pct,
    0.30,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(200);

  // 13) Conveyor + Intake REVERSE (limpiar)
  conveyor.move(-convPower);
  intake.move(-intakeFwd);

  // 12) Espera 2 s mientras limpia
  pros::delay(2000);

  // 14) Adelante 12"
  drive_straight_mm(
    FWD4_MM,
    drive_pct,
    0.30,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  // 15) Giro ~62° derecha
  turn_imu_deg_2stage(
    -62.0,
    turn_fast,
    turn_slow,
    0.92,
    120
  );
  pros::delay(120);

  // 16) Adelante 18" con conveyor REVERSE
  drive_straight_mm(
    FWD5_MM,
    drive_pct,
    0.30,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  // 17) Espera 2 s con conveyor REVERSE
  pros::delay(2000);

  // 18) Conveyor FORWARD y adelante 13"
  conveyor.move(convPower);
  drive_straight_mm(
    FWD6_MM,
    autopct(45),
    0.30,
    80.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  // Fin de auton, respetar 15 s
  uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) {
    pros::delay(15000 - elapsed);
  }

  intake.move(0);
  conveyor.move(0);
  drive.set_percent(0, 0);
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);
}

// ======================================================
// AUTON LEFT
//   Versión espejo simple de RIGHT
//   Puedes tunear distancias y ángulos en práctica
// ======================================================

void auton_left() {
  const double comp = auton_voltage_comp();
  const uint32_t t0 = pros::millis();

  auto autopct = [comp](int p) {
    return clamp_pct(static_cast<int>(p * comp));
  };

  Drive drive;
  drive.set_brake(pros::E_MOTOR_BRAKE_HOLD);

  const int drive_base = cfg.AUTO_DRIVE_PCT;
  const int turn_base  = cfg.AUTO_TURN_PCT;

  const int drive_pct  = autopct(drive_base);
  const int turn_fast  = autopct(turn_base);
  const int turn_slow  = std::max(10, static_cast<int>(turn_fast * 0.60));

  // Misma geometría por ahora
  const double FWD1_MM = 15.0  * 25.4;
  const double FWD2_MM = 21.9  * 25.4;
  const double FWD3_MM = 10.0  * 25.4;
  const double BACK_MM = 18.0  * 25.4;
  const double FWD7_MM = 6.0   * 25.4;
  const double FWD4_MM = 12.0  * 25.4;
  const double FWD5_MM = 18.0  * 25.4;
  const double FWD6_MM = 13.0  * 25.4;

  const int convPower  = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd  = cfg.INTAKE_FWD_PCT * 127 / 100;

  // 1) Adelante 15"
  drive_straight_mm(
    FWD1_MM,
    drive_pct,
    0.40,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(100);

  // Giros izquierdo en vez de derecho
  turn_imu_deg_2stage(
    +43.0,
    turn_fast,
    turn_slow,
    0.92,
    120
  );
  pros::delay(100);

  drive_straight_mm(
    FWD2_MM,
    drive_pct,
    0.30,
    120.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(80);

  turn_imu_deg_2stage(
    +42.0,
    turn_fast,
    turn_slow,
    0.92,
    120
  );
  pros::delay(120);

  piston_1.set_value(true);
  pros::delay(320);

  conveyor.move(-convPower);
  drive_straight_mm(
    FWD3_MM,
    autopct(45),
    0.30,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(80);

  pros::delay(2000);

  drive_straight_mm(
    FWD7_MM,
    drive_pct,
    0.40,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  piston_1.set_value(false);

  drive_straight_mm(
    -BACK_MM,
    drive_pct,
    0.30,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(200);

  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2000);

  drive_straight_mm(
    FWD4_MM,
    drive_pct,
    0.30,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  turn_imu_deg_2stage(
    +62.0,
    turn_fast,
    turn_slow,
    0.92,
    120
  );
  pros::delay(120);

  drive_straight_mm(
    FWD5_MM,
    drive_pct,
    0.30,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  pros::delay(2000);

  conveyor.move(convPower);
  drive_straight_mm(
    FWD6_MM,
    autopct(45),
    0.30,
    80.0,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) {
    pros::delay(15000 - elapsed);
  }

  intake.move(0);
  conveyor.move(0);
  drive.set_percent(0, 0);
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);
}
