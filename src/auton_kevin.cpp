#include "auton.hpp"
#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "pros/motors.h"

namespace {

// ---------------- Device mapping ----------------
static pros::Motor &Intake   = intake;
static pros::Motor &Conveyor = conveyor;

static pros::adi::DigitalOut &PistonA = piston_1;

// ---------------- Units helpers ----------------
constexpr double IN_TO_MM = 25.4;

inline int clamp_pct(int v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

inline int mm_from_in(double inches) {
  const double mm = inches * IN_TO_MM;
  return static_cast<int>(mm + (mm >= 0 ? 0.5 : -0.5));
}

// ---------------- Voltage compensation ----------------
inline double kevin_voltage_comp() {
  const double volt = pros::battery::get_voltage() / 1000.0;
  if (volt <= 0.01) return 1.0;

  constexpr double IDEAL = 12.6;
  double comp = IDEAL / volt;

  if (comp > 1.10) comp = 1.10;
  if (comp < 0.95) comp = 0.95;
  return comp;
}

inline int autopct(int pct, double comp) {
  return clamp_pct(static_cast<int>(pct * comp));
}

inline void stop_mechs() {
  Intake.brake();
  Conveyor.brake();
}

inline uint32_t now_ms() {
  return pros::millis();
}

} // namespace

namespace auton_kevin {

static Mode g_mode = Mode::SKILLS;

void set_mode(Mode m) { g_mode = m; }
Mode get_mode() { return g_mode; }

void autonomous_routine_kevin() {
  switch (g_mode) {
    case Mode::MATCH_IZQ:  routine_match_izq();  break;
    case Mode::MATCH_DER:  routine_match_der(); break;
    case Mode::SKILLS: routine_skills(); break;
    default:               routine_match_der();  break;
  }
}

void routine_match_izq() {
  const double comp = kevin_voltage_comp();
  const uint32_t t0 = now_ms();

  const int drive_base = 50;
  const int turn_base  = 35;

  // Use your new 2-stage defaults, still scaled by comp
  const int turn_fast  = autopct((int)(turn_base * 0.85), comp);
  const int turn_slow  = autopct((int)(turn_base * 0.60), comp);

  const int FWD1_MM = mm_from_in(29.5);
  const int FWD2_MM = mm_from_in(9.5);
  const int FWD3_MM = mm_from_in(1.0);
  const int BACK_MM = mm_from_in(25.5);
  const int FWD7_MM = mm_from_in(6.0);
  const int FWD4_MM = mm_from_in(18.0);
  const int FWD5_MM = mm_from_in(18.0);
  const int FWD6_MM = mm_from_in(34.0);

  PistonA.set_value(false);

  drive_straight_mm(FWD1_MM,
                    autopct(drive_base, comp),
                    0.42,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(100);

  turn_imu_deg_2stage(-90,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(120);

  PistonA.set_value(true);
  pros::delay(320);

  drive_straight_mm(FWD2_MM,
                    autopct(75, comp),
                    0.30,
                    110.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(40);

  drive_straight_mm(FWD3_MM,
                    autopct(drive_base, comp),
                    0.30,
                    110.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);

  Conveyor.move(-127);
  pros::delay(920);

  Conveyor.move(0);
  PistonA.set_value(false);

  drive_straight_mm(-BACK_MM,
                    autopct(drive_base, comp),
                    0.30,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(80);

  Conveyor.move(-127);
  Intake.move(127);
  pros::delay(2000);

  Conveyor.move(0);
  Intake.move(0);

  drive_straight_mm(FWD5_MM,
                    autopct(drive_base, comp),
                    0.30,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  turn_imu_deg_2stage(-88,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(120);

  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  stop_mechs();
}

void routine_match_der() {
  const double comp = kevin_voltage_comp();
  const uint32_t t0 = now_ms();

  const int drive_base = 50;
  const int turn_base  = 35;

  // Use your new 2-stage defaults, still scaled by comp
  const int turn_fast  = autopct((int)(turn_base * 0.85), comp);
  const int turn_slow  = autopct((int)(turn_base * 0.60), comp);

  const int FWD1_MM = mm_from_in(29.8);
  const int FWD2_MM = mm_from_in(9.5);
  const int FWD3_MM = mm_from_in(1.0);
  const int BACK_MM = mm_from_in(25.5);
  const int FWD7_MM = mm_from_in(6.0);
  const int FWD4_MM = mm_from_in(30.0);
  const int FWD5_MM = mm_from_in(18.0);
  const int FWD6_MM = mm_from_in(34.0);

  PistonA.set_value(false);

  drive_straight_mm(FWD1_MM,
                    autopct(drive_base, comp),
                    0.40,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(100);

  turn_imu_deg_2stage(90,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(120);

  PistonA.set_value(true);
  pros::delay(320);

  drive_straight_mm(FWD2_MM,
                    autopct(85, comp),
                    0.30,
                    110.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(40);

  drive_straight_mm(FWD3_MM,
                    autopct(drive_base, comp),
                    0.30,
                    110.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);

  Conveyor.move(-127);
  pros::delay(920);

  Conveyor.move(0);
  PistonA.set_value(false);

  drive_straight_mm(-BACK_MM,
                    autopct(drive_base, comp),
                    0.30,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(80);

  Conveyor.move(-127);
  Intake.move(127);
  pros::delay(2000);

  Conveyor.move(0);
  Intake.move(0);

  drive_straight_mm(FWD5_MM,
                    autopct(drive_base, comp),
                    0.30,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  turn_imu_deg_2stage(135,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(120);

  drive_straight_mm(FWD4_MM,
                    autopct(drive_base, comp),
                    0.30,
                    160.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  PistonA.set_value(true);
  Conveyor.move(-127);

  drive_straight_mm(FWD3_MM,
                    autopct(drive_base, comp),
                    0.30,
                    90.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(920);

  PistonA.set_value(false);

  Conveyor.move(127);
  pros::delay(1000);

  drive_straight_mm(FWD7_MM,
                    autopct(drive_base, comp),
                    0.30,
                    90.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(920);

  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  stop_mechs();
}

void routine_skills() {
  const double comp = kevin_voltage_comp();
  const uint32_t t0 = now_ms();

  const int drive_base = 50;
  const int drive_fast = 60;   // long cross-field legs
  const int drive_slow = 38;   // goal contact legs

  const int turn_base  = 35;

  // Use your new 2-stage defaults.
  // We still scale by battery comp, but keep the same signature.
  const int turn_fast  = autopct((int)(turn_base * 0.85), comp);
  const int turn_slow  = autopct((int)(turn_base * 0.55), comp);

  // =========================
  // RIGHT START SKILLS PATH
  // =========================

  const int D_FWD_TO_CENTER_MM     = mm_from_in(25.0);
  const int D_FWD_PISTON_MM        = mm_from_in(4.0);
  const int D_BUMP_CENTER_MM       = mm_from_in(3.0);

  const int D_BACK_FROM_CENTER_MM  = mm_from_in(47.5);

  const int D_FWD_TO_RLOADER_MM    = mm_from_in(6.0);
  const int D_BACK_FROM_RLOADER_MM = mm_from_in(28.0);

  const int D_FWD_TO_LONG_MM       = mm_from_in(34.0);
  const int D_BUMP_LONG_MM         = mm_from_in(10.0);

  const int D_CROSS_1_MM           = mm_from_in(96.0);
  const int D_CROSS_2_MM           = mm_from_in(36.0);

  const int D_FWD_TO_LLOADER_MM    = mm_from_in(10.0);
  const int D_BACK_FROM_LLOADER_MM = mm_from_in(28.0);
  const int D_FWD_FROM_LLOADER_MM  = mm_from_in(10.0);

  const int D_FWD_TO_PARK_MM       = mm_from_in(18.0);

  // Angles
  const double A_TURN_TO_CENTER    = -90.0;
  const double A_TURN_TO_RLOADER   = -115.0;
  const double A_TURN_TO_LONG      =  90.0;
  const double A_TURN_TO_LLOADER   = -90.0;
  const double A_TURN_TO_PARK      =  0.0;

  PistonA.set_value(false);

  Conveyor.move(0);
  Intake.move(0);

  // -------------------------
  // 1) Forward, turn, take low center
  // -------------------------
  drive_straight_mm(D_FWD_TO_CENTER_MM,
                    autopct(drive_base, comp),
                    0.30,
                    140.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(200);

  // First turn, with new signature
  turn_imu_deg_2stage(35,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  // Conveyor forward right after first turn
  Conveyor.move(127);
  pros::delay(80);

  PistonA.set_value(true);

  drive_straight_mm(D_FWD_PISTON_MM,
                    autopct(35, comp),
                    0.30,
                    90.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(220);

  PistonA.set_value(false);

  turn_imu_deg_2stage(A_TURN_TO_CENTER,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  drive_straight_mm(D_BUMP_CENTER_MM,
                    autopct(drive_slow, comp),
                    0.30,
                    90.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  Conveyor.move(-127);
  pros::delay(650);
  Conveyor.move(0);

  // -------------------------
  // 2) Backwards, turn, right loader
  // -------------------------
  drive_straight_mm(-D_BACK_FROM_CENTER_MM,
                    autopct(drive_base, comp),
                    0.30,
                    160.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  turn_imu_deg_2stage(A_TURN_TO_RLOADER,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  PistonA.set_value(true);
  pros::delay(120);

  drive_straight_mm(D_FWD_TO_RLOADER_MM,
                    autopct(drive_slow, comp),
                    0.30,
                    90.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  Conveyor.move(-127);
  pros::delay(900);
  Conveyor.move(0);

  // -------------------------
  // 3) Back, long goal
  // -------------------------
  PistonA.set_value(false);
  pros::delay(60);

  drive_straight_mm(-D_BACK_FROM_RLOADER_MM,
                    autopct(drive_base, comp),
                    0.30,
                    160.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(700);
  Conveyor.move(0);
  Intake.move(0);

  drive_straight_mm(D_BUMP_LONG_MM,
                    autopct(drive_slow, comp),
                    0.30,
                    120.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  turn_imu_deg_2stage(A_TURN_TO_LONG,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  // -------------------------
  // 4) Cross, left loader
  // -------------------------
  drive_straight_mm(D_CROSS_1_MM,
                    autopct(drive_fast, comp),
                    0.30,
                    220.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  turn_imu_deg_2stage(A_TURN_TO_LLOADER,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  PistonA.set_value(true);
  pros::delay(120);

  drive_straight_mm(D_FWD_TO_LLOADER_MM,
                    autopct(drive_slow, comp),
                    0.30,
                    90.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(120);

  Conveyor.move(127);
  pros::delay(900);
  Conveyor.move(0);

  // -------------------------
  // 5) Back, collect, park
  // -------------------------
  drive_straight_mm(-D_BACK_FROM_LLOADER_MM,
                    autopct(drive_base, comp),
                    0.30,
                    160.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(1200);

  drive_straight_mm(D_FWD_FROM_LLOADER_MM,
                    autopct(drive_base, comp),
                    0.30,
                    120.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  turn_imu_deg_2stage(-90,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  drive_straight_mm(D_CROSS_2_MM,
                    autopct(drive_fast, comp),
                    0.30,
                    180.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  turn_imu_deg_2stage(A_TURN_TO_PARK,
                      turn_fast,
                      turn_slow,
                      0.88,
                      160,
                      1.0,
                      1400);
  pros::delay(180);

  Conveyor.move(0);
  Intake.move(0);
  PistonA.set_value(false);
  pros::delay(80);

  drive_straight_mm(D_FWD_TO_PARK_MM,
                    autopct(drive_fast, comp),
                    0.30,
                    120.0,
                    pros::E_MOTOR_BRAKE_HOLD,
                    80,
                    60);
  pros::delay(180);

  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 60000) pros::delay(60000 - elapsed);

  stop_mechs();
}

} // namespace auton_kevin
