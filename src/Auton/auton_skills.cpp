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
/**
 * routine_skills
 *
 * 60 second skills autonomous.
 *
 * High level flow (right start path)
 *  0) Init pneumatics and mechanisms
 *  1) Take low center goal
 *  2) Back out and score right loader
 *  3) Back out, hit long goal, then cross field
 *  4) Score left loader
 *  5) Collect and park
 *
 * Notes
 *  - drive_straight_mm uses: Rotation distance + IMU heading PID + FF + velocity PID
 *  - turn_imu_deg_2stage uses IMU heading for a smooth 2 stage turn
 *  - autopct(...) scales with battery voltage compensation
 *  - All distances are in millimeters
 *  - All angles are IMU degrees
 */

void routine_skills() {
  // --------------------------------------------------------------------------
  // Timing and voltage normalization
  // --------------------------------------------------------------------------
  const double comp = kevin_voltage_comp();   // battery sag compensation factor
  const uint32_t t0 = now_ms();               // routine start time

  // --------------------------------------------------------------------------
  // Drive speed presets
  // --------------------------------------------------------------------------
  const int drive_base = 50;                  // baseline travel
  const int drive_fast = 60;                  // long cross-field legs
  const int drive_slow = 38;                  // goal contact legs

  // --------------------------------------------------------------------------
  // Turn presets (two stage), scaled by battery voltage compensation
  // --------------------------------------------------------------------------
  const int turn_base  = 35;
  const int turn_fast  = autopct((int)(turn_base * 0.85), comp);
  const int turn_slow  = autopct((int)(turn_base * 0.55), comp);

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
  const int D_FWD_TO_CENTER_MM     = mm_from_in(25.0);
  const int D_FWD_PISTON_MM        = mm_from_in(4.0);
  const int D_BUMP_CENTER_MM       = mm_from_in(3.0);

  const int D_BACK_FROM_CENTER_MM  = mm_from_in(47.5);

  const int D_FWD_TO_RLOADER_MM    = mm_from_in(6.0);
  const int D_BACK_FROM_RLOADER_MM = mm_from_in(28.0);

  const int D_BUMP_LONG_MM         = mm_from_in(10.0);

  const int D_CROSS_1_MM           = mm_from_in(96.0);
  const int D_CROSS_2_MM           = mm_from_in(36.0);

  const int D_FWD_TO_LLOADER_MM    = mm_from_in(10.0);
  const int D_BACK_FROM_LLOADER_MM = mm_from_in(28.0);
  const int D_FWD_FROM_LLOADER_MM  = mm_from_in(10.0);

  const int D_FWD_TO_PARK_MM       = mm_from_in(18.0);

  // Unused distance in current script, keep if you later re-enable that leg
  const int D_FWD_TO_LONG_MM       = mm_from_in(34.0);
  (void)D_FWD_TO_LONG_MM;

  // --------------------------------------------------------------------------
  // Turn angles (deg). Positive left, negative right by your conventions.
  // --------------------------------------------------------------------------
  const double A_TURN_TO_CENTER    = -90.0;
  const double A_TURN_TO_RLOADER   = -115.0;
  const double A_TURN_TO_LONG      =  90.0;
  const double A_TURN_TO_LLOADER   = -90.0;
  const double A_TURN_TO_PARK      =  0.0;

  // --------------------------------------------------------------------------
  // Pre-start state
  // --------------------------------------------------------------------------
  PistonA.set_value(false);                   // piston retracted
  Conveyor.move(0);
  Intake.move(0);

  // ==========================================================================
  // 1) Take low center goal
  // ==========================================================================
  drive_straight_mm(
    D_FWD_TO_CENTER_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    140.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(200);

  // Small alignment turn (script specific)
  turn_imu_deg_2stage(
    35,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  // Pre-spin conveyor for immediate scoring response
  Conveyor.move(127);
  pros::delay(80);
  Conveyor.move(0);

  // Piston tap
  PistonA.set_value(true);

  drive_straight_mm(
    D_FWD_PISTON_MM,
    autopct(35, comp),
    kP_h, kI_h, kD_h,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(220);

  PistonA.set_value(false);

  // Turn to face center goal
  turn_imu_deg_2stage(
    A_TURN_TO_CENTER,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  // Bump into center goal
  drive_straight_mm(
    D_BUMP_CENTER_MM,
    autopct(drive_slow, comp),
    kP_h, kI_h, kD_h,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(120);

  // Score outtake
  Conveyor.move(-127);
  pros::delay(650);
  Conveyor.move(0);

  // ==========================================================================
  // 2) Back out and score right loader
  // ==========================================================================
  drive_straight_mm(
    -D_BACK_FROM_CENTER_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    160.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  turn_imu_deg_2stage(
    A_TURN_TO_RLOADER,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  PistonA.set_value(true);
  pros::delay(120);

  drive_straight_mm(
    D_FWD_TO_RLOADER_MM,
    autopct(drive_slow, comp),
    kP_h, kI_h, kD_h,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(120);

  Conveyor.move(-127);
  pros::delay(900);
  Conveyor.move(0);

  // ==========================================================================
  // 3) Back out, hit long goal, then set up for cross
  // ==========================================================================
  PistonA.set_value(false);
  pros::delay(60);

  drive_straight_mm(
    -D_BACK_FROM_RLOADER_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    160.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  // Reverse intake to clear and prep
  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(700);
  Conveyor.move(0);
  Intake.move(0);

  // Bump long goal
  drive_straight_mm(
    D_BUMP_LONG_MM,
    autopct(drive_slow, comp),
    kP_h, kI_h, kD_h,
    120.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(120);

  // Turn to long goal orientation
  turn_imu_deg_2stage(
    A_TURN_TO_LONG,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  // ==========================================================================
  // 4) Cross field, score left loader
  // ==========================================================================
  drive_straight_mm(
    D_CROSS_1_MM,
    autopct(drive_fast, comp),
    kP_h, kI_h, kD_h,
    220.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  turn_imu_deg_2stage(
    A_TURN_TO_LLOADER,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  PistonA.set_value(true);
  pros::delay(120);

  drive_straight_mm(
    D_FWD_TO_LLOADER_MM,
    autopct(drive_slow, comp),
    kP_h, kI_h, kD_h,
    90.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(120);

  Conveyor.move(127);
  pros::delay(900);
  Conveyor.move(0);

  // ==========================================================================
  // 5) Back, collect, cross back, then park
  // ==========================================================================
  drive_straight_mm(
    -D_BACK_FROM_LLOADER_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    160.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  // Collection window
  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(1200);

  drive_straight_mm(
    D_FWD_FROM_LLOADER_MM,
    autopct(drive_base, comp),
    kP_h, kI_h, kD_h,
    120.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  // Re-orient for return cross
  turn_imu_deg_2stage(
    -90,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  drive_straight_mm(
    D_CROSS_2_MM,
    autopct(drive_fast, comp),
    kP_h, kI_h, kD_h,
    180.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  // Final orientation for park
  turn_imu_deg_2stage(
    A_TURN_TO_PARK,
    turn_fast,
    turn_slow,
    0.88,
    160,
    1.0,
    1400
  );
  pros::delay(180);

  // Mechanisms off before park
  Conveyor.move(0);
  Intake.move(0);
  PistonA.set_value(false);
  pros::delay(80);

  // Park
  drive_straight_mm(
    D_FWD_TO_PARK_MM,
    autopct(drive_fast, comp),
    kP_h, kI_h, kD_h,
    120.0,
    pros::E_MOTOR_BRAKE_HOLD,
    80,
    60
  );
  pros::delay(180);

  // --------------------------------------------------------------------------
  // Skills timing guard. Ensure we fill the full 60 seconds if needed.
  // --------------------------------------------------------------------------
  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 60000) pros::delay(60000 - elapsed);

  // Stop all non-drive mechanisms
  stop_mechs();
}
 // namespace auton_kevin
}