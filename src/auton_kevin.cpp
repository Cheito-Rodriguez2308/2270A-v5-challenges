// src/auton_kevin.cpp
#include "auton_kevin.hpp"

#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"

namespace {

// ---------------- Device mapping ----------------
static pros::Motor &Intake   = intake;
static pros::Motor &Conveyor = conveyor;

static pros::adi::DigitalOut &PistonA = piston_1;
static pros::adi::DigitalOut &PistonB = piston_2;

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

static Mode g_mode = Mode::MATCH_IZQ;

void set_mode(Mode m) { g_mode = m; }
Mode get_mode() { return g_mode; }

void autonomous_routine_kevin() {
  switch (g_mode) {
    case Mode::MATCH_IZQ:  routine_match_izq();  break;
    case Mode::SKILLS_IZQ: routine_skills_izq(); break;
    default:               routine_match_izq();  break;
  }
}

void routine_match_izq() {
  const double comp = kevin_voltage_comp();
  const uint32_t t0 = now_ms();

  const int drive_base = 50;
  const int turn_base  = 35;

  const int FWD1_MM = mm_from_in(15.0);
  const int FWD2_MM = mm_from_in(21.8);
  const int FWD3_MM = mm_from_in(10.0);
  const int BACK_MM = mm_from_in(18.0);
  const int FWD7_MM = mm_from_in(6.0);
  const int FWD4_MM = mm_from_in(12.0);
  const int FWD5_MM = mm_from_in(18.0);
  const int FWD6_MM = mm_from_in(13.0);

  PistonA.set_value(false);
  PistonB.set_value(false);

  drive_straight_mm(FWD1_MM, autopct(drive_base, comp), 0.40, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(100);

  turn_imu_deg_2stage(-43,
                      autopct(turn_base, comp),
                      autopct(static_cast<int>(turn_base * 0.60), comp),
                      0.92,
                      120);
  pros::delay(100);

  drive_straight_mm(FWD2_MM, autopct(drive_base, comp), 0.30, 120, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(80);

  turn_imu_deg_2stage(-42,
                      autopct(turn_base, comp),
                      autopct(static_cast<int>(turn_base * 0.60), comp),
                      0.92,
                      120);
  pros::delay(120);

  PistonA.set_value(true);
  pros::delay(320);

  Conveyor.move(-127);
  drive_straight_mm(FWD3_MM, autopct(45, comp), 0.30, 90, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(80);

  pros::delay(2000);

  drive_straight_mm(FWD7_MM, autopct(drive_base, comp), 0.40, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(120);

  drive_straight_mm(-BACK_MM, autopct(drive_base, comp), 0.30, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(200);

  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(2000);

  PistonA.set_value(false);

  drive_straight_mm(FWD4_MM, autopct(drive_base, comp), 0.30, 90, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(120);

  turn_imu_deg_2stage(-62,
                      autopct(turn_base, comp),
                      autopct(static_cast<int>(turn_base * 0.60), comp),
                      0.92,
                      120);
  pros::delay(120);

  drive_straight_mm(FWD5_MM, autopct(drive_base, comp), 0.30, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(120);

  pros::delay(2000);

  Conveyor.move(127);
  drive_straight_mm(FWD6_MM, autopct(45, comp), 0.30, 80, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(120);

  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  stop_mechs();
}

void routine_skills_izq() {
  const double comp = kevin_voltage_comp();
  const uint32_t t0 = now_ms();

  const int drive_base = 50;
  const int turn_base  = 35;

  const int turn_fast = autopct(turn_base, comp);
  const int turn_slow = autopct(static_cast<int>(turn_base * 0.60), comp);

  const int FWD2_MM = mm_from_in(21.9);
  const int FWD3_MM = mm_from_in(10.0);
  const int BACK_MM = mm_from_in(18.0);
  const int FWD4_MM = mm_from_in(12.0);
  const int FWD5_MM = mm_from_in(10.0);
  const int FWD6_MM = mm_from_in(43.0);
  const int FWD7_MM = mm_from_in(10.0);
  const int FWD8_MM = mm_from_in(13.0);

  PistonA.set_value(false);
  PistonB.set_value(false);

  drive_straight_mm(FWD2_MM, autopct(drive_base, comp), 0.30, 120, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(140);

  turn_imu_deg_2stage(-42, turn_fast, turn_slow, 0.92, 120);
  pros::delay(160);

  PistonA.set_value(true);
  pros::delay(200);

  Conveyor.move(-127);
  drive_straight_mm(FWD3_MM, autopct(45, comp), 0.30, 90, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(140);

  pros::delay(1000);

  PistonA.set_value(false);
  pros::delay(100);

  drive_straight_mm(-BACK_MM, autopct(drive_base, comp), 0.30, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(220);

  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(2000);

  drive_straight_mm(FWD4_MM, autopct(drive_base, comp), 0.30, 90, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(140);

  turn_imu_deg_2stage(-42, turn_fast, turn_slow, 0.92, 120);
  pros::delay(160);

  drive_straight_mm(FWD5_MM, autopct(drive_base, comp), 0.30, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(140);

  turn_imu_deg_2stage(42, turn_fast, turn_slow, 0.92, 120);
  pros::delay(160);

  drive_straight_mm(FWD6_MM, autopct(45, comp), 0.30, 80, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(220);

  turn_imu_deg_2stage(42, turn_fast, turn_slow, 0.92, 120);
  pros::delay(160);

  drive_straight_mm(FWD7_MM, autopct(drive_base, comp), 0.30, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(140);

  turn_imu_deg_2stage(-42, turn_fast, turn_slow, 0.92, 120);
  pros::delay(160);

  PistonA.set_value(true);
  pros::delay(200);

  drive_straight_mm(FWD8_MM, autopct(drive_base, comp), 0.30, 110, pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(140);

  Conveyor.move(-127);
  pros::delay(2000);

  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  stop_mechs();
}

} // namespace auton_kevin
