#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "pros/apix.h"
#include "pros/motors.h"
#include <algorithm>
#include <cmath>

// ======================================================
// MODULO: auton (implementacion)
// Nueva version usando SOLO:
// - drive_straight_mm (Rotation + IMU, motion.cpp)
// - turn_imu_deg_2stage (IMU, motion.cpp)
// - Tramos largos: slow_down_mm = 200 a 260
// - Tramos cortos: slow_down_mm = 120 a 160
//
// COMBO COAST + BRAKE:
// - Durante el tramo: COAST (se siente humano, sin “clavar”)
// - Al final: pulso corto de BRAKE para asentar sin quedarte en HOLD
//
// NOTA IMPORTANTE
// Este auton asume que drive_straight_mm fue extendida a:
// drive_straight_mm(dist_mm, base_pct, kP_heading, slow_down_mm, end_brake,
//                   soft_settle_ms, brake_pulse_ms)
// Si tu motion.hpp aun no tiene esos 2 parametros extra,
// tienes que añadirlos o usar el wrapper que te pongo abajo.
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

namespace {

  constexpr double AUTON_IDEAL_VOLTAGE = 12.6;

  // Slowdowns recomendados
  constexpr double SLOW_LONG_MM  = 240.0; // 200-260
  constexpr double SLOW_LONG2_MM = 260.0; // para reversa larga
  constexpr double SLOW_MED_MM   = 220.0; // 200-260
  constexpr double SLOW_SHORT_MM = 140.0; // 120-160
  constexpr double SLOW_TINY_MM  = 130.0; // 120-160

  // Settles
  constexpr int SOFT_SETTLE_MS = 80;
  constexpr int BRAKE_PULSE_MS = 60;

  double auton_get_voltage() {
    return pros::battery::get_voltage() / 1000.0;
  }

  double auton_voltage_comp() {
    const double volt = auton_get_voltage();
    if (volt <= 0.0) return 1.0;

    double comp = AUTON_IDEAL_VOLTAGE / volt;
    if (comp > 1.20) comp = 1.20;
    if (comp < 0.90) comp = 0.90;
    return comp;
  }

  int auton_clamp_pct(int v) {
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
  }

  int autopct(double comp, int p) {
    return auton_clamp_pct(static_cast<int>(p * comp));
  }

  void set_drive_brake(pros::motor_brake_mode_e mode) {
    lf.set_brake_mode(mode);
    lm.set_brake_mode(mode);
    lb.set_brake_mode(mode);
    rf.set_brake_mode(mode);
    rm.set_brake_mode(mode);
    rb.set_brake_mode(mode);
  }

  void brake_pulse(int ms = 70) {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    lf.brake(); lm.brake(); lb.brake();
    rf.brake(); rm.brake(); rb.brake();
    pros::delay(ms);
    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
  }

} // namespace

// RIGHT
static void auton_right_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
);

static void auton_right_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
);

static void auton_right_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
);

// LEFT
static void auton_left_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
);

static void auton_left_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
);

static void auton_left_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
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
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);

  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

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
    comp, drivePct, turnFast, turnSlow,
    FWD1_MM, FWD2_MM, convPower, intakeFwd
  );

  auton_right_stage_platform(
    comp, drivePct, turnFast, turnSlow,
    REV_1_MM, BACK_MM, FWD7_MM, convPower
  );

  auton_right_stage_second_cycle(
    comp, drivePct, turnFast, turnSlow,
    FWD4_MM, FWD5_MM, convPower, intakeFwd
  );

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  intake.move(0);
  conveyor.move(0);
  brake_pulse(60);
}

// ETAPA 1: salida de start y uso del preload
static void auton_right_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  // Adelante 20" (largo)
  drive_straight_mm(
    FWD1_MM,
    drivePct,
    0.40,
    SLOW_LONG_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(50);

  // Giro 50° izquierda
  turn_imu_deg_2stage(
    +50.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  conveyor.move(0);
  pros::delay(60);

  // Adelante 9.5" (corto)
  drive_straight_mm(
    FWD2_MM,
    drivePct,
    0.30,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(70);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);
  pros::delay(80);

  (void)comp;
}

// ETAPA 2: giro a plataforma y puntuacion con piston
static void auton_right_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
) {
  // Reversa 31.0" (largo)
  drive_straight_mm(
    -REV_1_MM,
    drivePct,
    0.30,
    SLOW_LONG2_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(70);

  // Giro 90° izquierda
  turn_imu_deg_2stage(
    +90.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(70);
  pros::delay(60);

  piston_1.set_value(true);
  pros::delay(300);

  // Adelante 8.5" (corto)
  drive_straight_mm(
    FWD7_MM,
    autopct(comp, cfg.AUTO_DRIVE_PCT),
    0.40,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST
  );

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(60);

  // Reversa 17" (largo medio)
  drive_straight_mm(
    -BACK_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
}

// ETAPA 3: segundo ciclo / limpieza
static void auton_right_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  conveyor.move(0);
  intake.move(0);
  pros::delay(60);

  // Adelante 7" (corto)
  drive_straight_mm(
    FWD4_MM,
    drivePct,
    0.30,
    SLOW_TINY_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(60);

  // Giro 70° derecha
  turn_imu_deg_2stage(
    -70.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  // Adelante 15" (largo)
  drive_straight_mm(
    FWD5_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST
  );

  (void)comp;
}

// ======================================================
// AUTON LEFT
// ======================================================
void auton_left() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);

  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

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
    comp, drivePct, turnFast, turnSlow,
    FWD1_MM, FWD2_MM, convPower, intakeFwd
  );

  auton_left_stage_platform(
    comp, drivePct, turnFast, turnSlow,
    REV_1_MM, BACK_MM, FWD7_MM, convPower
  );

  auton_left_stage_second_cycle(
    comp, drivePct, turnFast, turnSlow,
    FWD4_MM, FWD5_MM, convPower, intakeFwd
  );

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  intake.move(0);
  conveyor.move(0);
  brake_pulse(60);
}

// ETAPA 1: salida de start y uso del preload
static void auton_left_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  // Adelante 20" (largo)
  drive_straight_mm(
    FWD1_MM,
    drivePct,
    0.40,
    SLOW_LONG_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(50);

  // Giro 50° derecha
  turn_imu_deg_2stage(
    -50.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  conveyor.move(0);
  pros::delay(60);

  // Adelante 9.5" (corto)
  drive_straight_mm(
    FWD2_MM,
    drivePct,
    0.30,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(70);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);
  pros::delay(80);

  (void)comp;
}

// ETAPA 2: giro a plataforma y puntuacion con piston
static void auton_left_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
) {
  // Reversa 31.0" (largo)
  drive_straight_mm(
    -REV_1_MM,
    drivePct,
    0.30,
    SLOW_LONG2_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(70);

  // Giro 90° derecha
  turn_imu_deg_2stage(
    -90.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(70);
  pros::delay(60);

  piston_1.set_value(true);
  pros::delay(320);

  // Adelante 8.5" (corto)
  drive_straight_mm(
    FWD7_MM,
    autopct(comp, cfg.AUTO_DRIVE_PCT),
    0.40,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(50);

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(120);

  // Reversa 17" (largo medio)
  drive_straight_mm(
    -BACK_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(60);
}

// ETAPA 3: segundo ciclo / limpieza
static void auton_left_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  conveyor.move(0);
  intake.move(0);
  pros::delay(60);

  // Adelante 7" (corto)
  drive_straight_mm(
    FWD4_MM,
    drivePct,
    0.30,
    SLOW_TINY_MM,
    pros::E_MOTOR_BRAKE_COAST
  );
  pros::delay(60);

  // Giro 70° izquierda
  turn_imu_deg_2stage(
    +70.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  // Adelante 15" (largo)
  drive_straight_mm(
    FWD5_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST
  );

  (void)comp;
}
