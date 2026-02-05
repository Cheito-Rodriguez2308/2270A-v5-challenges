#include "auton.hpp"
#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "pros/motors.h"
#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"

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

namespace auton_kevin {
// ------------------------------------------------------------
// LEFT SIDE MATCH AUTON
// ------------------------------------------------------------

/**
 * routine_match_izq
 *
 * Left side match autonomous.
 *
 * High level flow
 *  0) Init pneumatics
 *  1) Drive to first objective
 *  2) Turn to face goal
 *  3) Fire piston and tap in
 *  4) Score cycle with conveyor and intake
 *  5) End within 15s hard cap
 *
 * Notes
 *  - drive_straight_mm uses: Rotation distance + IMU heading PID + FF + velocity PID
 *  - turn_imu_deg_2stage uses IMU heading for a smooth 2 stage turn
 *  - autopct(...) scales with battery voltage compensation
 */

 void routine_match_izq() {
  // --------------------------------------------------------------------------
  // Timing and voltage normalization
  // --------------------------------------------------------------------------
  const double comp = kevin_voltage_comp();   // battery sag compensation factor
  const uint32_t t0 = now_ms();               // routine start time

  // --------------------------------------------------------------------------
  // Base drive and turn aggressiveness
  // --------------------------------------------------------------------------
  const int drive_base = 50;                  // baseline straight speed (percent)
  const int turn_base  = 35;                  // baseline turn speed (percent)

  // Two stage turn speeds (fast then slow), scaled by voltage comp
  const int turn_fast  = autopct((int)(turn_base * 0.85), comp);
  const int turn_slow  = autopct((int)(turn_base * 0.60), comp);

  // --------------------------------------------------------------------------
  // Heading PID for straight driving
  // Units (omega PID)
  //  - kP_h: (rad/s) per rad
  //  - kI_h: (rad/s) per (rad*s)
  //  - kD_h: (rad/s) per (rad/s) using measured yaw rate damping
  // --------------------------------------------------------------------------
  const double kP_h = 4.0;
  const double kI_h = 0.0;
  const double kD_h = 0.4;

  // --------------------------------------------------------------------------
  // Distances (mm). Derived from inch measurements for readability.
  // --------------------------------------------------------------------------
  const int FWD1_MM = mm_from_in(29.5);       // opening drive
  const int FWD2_MM = mm_from_in(9.0);        // approach after piston
  const int FWD3_MM = mm_from_in(1.0);        // small tap forward
  const int BACK_MM = mm_from_in(26.5);       // retreat to intake cycle
  const int FWD5_MM = mm_from_in(10.0);       // small forward/back micro cycle

  // Unused in this routine, keep declarations if you plan to re-enable later
  const int FWD4_MM = mm_from_in(18.0);
  const int FWD6_MM = mm_from_in(34.0);
  const int FWD7_MM = mm_from_in(6.0);
  (void)FWD4_MM; (void)FWD6_MM; (void)FWD7_MM;

  // --------------------------------------------------------------------------
  // Pre-start state
  // --------------------------------------------------------------------------
  PistonA.set_value(false);                   // piston retracted
  Conveyor.move(0);
  Intake.move(0);

  // ==========================================================================
  // 1) Opening drive to first objective
  // ==========================================================================
  drive_straight_mm(
    FWD1_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    140.0,                                  // slow-down window (mm)
    pros::E_MOTOR_BRAKE_HOLD,                // stop solid at target
    80,                                      // soft settle (ms)
    60                                       // brake pulse (ms)
  );
  pros::delay(100);

  // ==========================================================================
  // 2) Face target
  // ==========================================================================
  turn_imu_deg_2stage(
    -90,                                     // left turn
    turn_fast,
    turn_slow,
    0.88,                                    // stage split
    160,                                     // settle between stages
    1.0,                                     // tolerance
    1400                                     // timeout
  );
  pros::delay(120);

  // ==========================================================================
  // 3) Piston action and short approach
  // ==========================================================================
  PistonA.set_value(true);
  pros::delay(320);

  drive_straight_mm(
    FWD2_MM,
    autopct(75, comp),                       // faster punch-in
    kP_h, kI_h, kD_h,
    110.0,
    pros::E_MOTOR_BRAKE_BRAKE,
    80,
    60
  );
  pros::delay(40);

  drive_straight_mm(
    FWD3_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    110.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );

  // Score pulse with conveyor
  Conveyor.move(-127);
  pros::delay(500);
  Conveyor.move(0);

  PistonA.set_value(false);

  // ==========================================================================
  // 4) Retreat and intake cycle
  // ==========================================================================
  drive_straight_mm(
    -BACK_MM,
    autopct(75, comp),
    kP_h, kI_h, kD_h,
    140.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(80);

  // Intake + conveyor cycle
  Conveyor.move(-127);
  Intake.move(127);
  pros::delay(2000);
  Conveyor.move(0);
  Intake.move(0);

  // ==========================================================================
  // 5) Micro adjust cycle (forward then back)
  // ==========================================================================
  drive_straight_mm(
    FWD5_MM,
    autopct(75, comp),
    kP_h, kI_h, kD_h,
    140.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(120);

  drive_straight_mm(
    -FWD5_MM,
    autopct(75, comp),
    kP_h, kI_h, kD_h,
    140.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(120);

  // --------------------------------------------------------------------------
  // Match timing guard. Ensure we do not exceed 15 seconds.
  // --------------------------------------------------------------------------
  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  // Stop all non-drive mechanisms
  stop_mechs();
}
}