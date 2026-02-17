// auton_skills_ramsete.cpp
#include "auton.hpp"
#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "odom.hpp"
#include <cmath>

// ---------------- Device mapping ----------------
static pros::Motor& Intake   = intake;
static pros::Motor& Conveyor = conveyor;
static pros::adi::DigitalOut& PistonA = piston_1;

// ---------------- Units helpers ----------------
constexpr double IN_TO_MM = 25.4;

static inline int clamp_pct_local(int v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

static inline int mm_from_in(double inches) {
  const double mm = inches * IN_TO_MM;
  return static_cast<int>(mm + (mm >= 0 ? 0.5 : -0.5));
}

// ---------------- Voltage compensation ----------------
static inline double kevin_voltage_comp() {
  const double volt = pros::battery::get_voltage() / 1000.0;
  if (volt <= 0.01) return 1.0;

  constexpr double IDEAL = 12.6;
  double comp = IDEAL / volt;

  if (comp > 1.10) comp = 1.10;
  if (comp < 0.95) comp = 0.95;
  return comp;
}

static inline int autopct(int pct, double comp) {
  return clamp_pct_local(static_cast<int>(pct * comp));
}

static inline void stop_mechs() {
  Intake.brake();
  Conveyor.brake();
}

static inline uint32_t now_ms() { return pros::millis(); }

// ============================================================================
// RAMSETE planning helpers
// Coordinate system matches your odom:
// - heading 0 deg points to +Y
// - x grows to the right when facing +Y
// - forward increases y
// ============================================================================
static inline double deg2rad(double d) { return d * (PI / 180.0); }

struct PlanPoseMM {
  double x_mm;
  double y_mm;
  double heading_deg;  // desired end heading, absolute
};

static inline void plan_advance_mm(PlanPoseMM& p, double dist_mm) {
  const double th = deg2rad(p.heading_deg);
  p.x_mm += dist_mm * std::sin(th);
  p.y_mm += dist_mm * std::cos(th);
}

static inline void plan_turn_deg(PlanPoseMM& p, double delta_deg) {
  p.heading_deg += delta_deg;
  while (p.heading_deg >= 360.0) p.heading_deg -= 360.0;
  while (p.heading_deg < 0.0)    p.heading_deg += 360.0;
}

namespace auton_kevin {

/**
 * routine_skills_ramsete
 *
 * Converts your skills routine into RAMSETE-based point-to-pose moves.
 * Strategy:
 * - Keep your mechanism timing and actions.
 * - Replace each drive_straight_mm segment with drive_to_pose_ramsete_mm using
 *   an internally-tracked absolute plan pose (x,y,heading).
 * - Keep turn_imu_deg_2stage for “pure turns” to reduce risk. After each turn,
 *   the plan heading updates to match the intended absolute heading.
 *
 * Important:
 * - This assumes your odometry is running and reasonably stable.
 * - Start pose is set to (0,0,0). If your real start is different, change it.
 */
void routine_skills_ramsete() {
  // --------------------------------------------------------------------------
  // Timing and voltage normalization
  // --------------------------------------------------------------------------
  const double comp = kevin_voltage_comp();
  const uint32_t t0 = now_ms();

  // --------------------------------------------------------------------------
  // Drive speed presets
  // --------------------------------------------------------------------------
  const int drive_base = 50;
  const int drive_fast = 60;
  const int drive_slow = 38;

  // --------------------------------------------------------------------------
  // Turn presets (two stage)
  // --------------------------------------------------------------------------
  const int turn_base  = 35;
  const int turn_fast  = autopct((int)(turn_base * 0.85), comp);
  const int turn_slow  = autopct((int)(turn_base * 0.55), comp);

  // --------------------------------------------------------------------------
  // RAMSETE knobs for this demo
  // --------------------------------------------------------------------------
  const double a_max_mps2 = 1.2;

  const double pos_tol_mm_fast = 25.0;
  const double pos_tol_mm_mid  = 20.0;
  const double pos_tol_mm_prec = 15.0;

  const double ang_tol_deg_fast = 3.0;
  const double ang_tol_deg_prec = 2.0;

  // --------------------------------------------------------------------------
  // Distances (mm)
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

  // --------------------------------------------------------------------------
  // Turn angles (deg), relative
  // --------------------------------------------------------------------------
  const double A_TURN_ALIGN_1       =  35.0;
  const double A_TURN_TO_CENTER     = -90.0;
  const double A_TURN_TO_RLOADER    = -115.0;
  const double A_TURN_TO_LONG       =  90.0;
  const double A_TURN_TO_LLOADER    = -90.0;
  const double A_TURN_TO_PARK       =   0.0;   // absolute heading target later

  // --------------------------------------------------------------------------
  // Pre-start state and pose reset
  // --------------------------------------------------------------------------
  PistonA.set_value(false);
  Conveyor.move(0);
  Intake.move(0);

  // Reset odometry start pose: (0,0) facing +Y
  odom.reset(0.0, 0.0, 0.0);
  pros::delay(30);

  // Internal planned absolute pose in mm and degrees
  PlanPoseMM plan{0.0, 0.0, 0.0};

  // Helper lambda to execute RAMSETE drive to current plan pose
  auto go_plan = [&](int base_pct,
                     double pos_tol_mm,
                     double ang_tol_deg,
                     int timeout_ms,
                     pros::motor_brake_mode_e end_brake) {
    drive_to_pose_ramsete_mm(
              plan.x_mm,
              plan.y_mm,
              plan.heading_deg,
              base_pct,
              a_max_mps2,
              pos_tol_mm,
              ang_tol_deg,
              timeout_ms,
              end_brake
    );
  };

  // ========================================================================
  // 1) Take low center goal
  // ========================================================================
  plan_advance_mm(plan, D_FWD_TO_CENTER_MM);
  go_plan(
    autopct(drive_base, comp),
    pos_tol_mm_mid,
    ang_tol_deg_fast,
    3200,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(200);

  // Small alignment turn
  turn_imu_deg_2stage(A_TURN_ALIGN_1, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
  pros::delay(180);
  plan_turn_deg(plan, A_TURN_ALIGN_1);

  // Pre-spin conveyor
  Conveyor.move(127);
  pros::delay(80);
  Conveyor.move(0);

  // Piston tap forward
  PistonA.set_value(true);
  plan_advance_mm(plan, D_FWD_PISTON_MM);
  go_plan(
    autopct(35, comp),
    pos_tol_mm_prec,
    ang_tol_deg_prec,
    1800,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(220);

  PistonA.set_value(false);

  // Turn to face center goal
  turn_imu_deg_2stage(A_TURN_TO_CENTER, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
  pros::delay(180);
  plan_turn_deg(plan, A_TURN_TO_CENTER);

  // Bump into center goal
  plan_advance_mm(plan, D_BUMP_CENTER_MM);
  go_plan(
    autopct(drive_slow, comp),
    pos_tol_mm_prec,
    ang_tol_deg_prec,
    1400,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  Conveyor.move(-127);
  pros::delay(650);
  Conveyor.move(0);

  // ========================================================================
  // 2) Back out and score right loader
  // ========================================================================
  plan_advance_mm(plan, -D_BACK_FROM_CENTER_MM);
  go_plan(
    autopct(drive_base, comp),
    pos_tol_mm_mid,
    ang_tol_deg_fast,
    4200,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  turn_imu_deg_2stage(A_TURN_TO_RLOADER, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
  pros::delay(180);
  plan_turn_deg(plan, A_TURN_TO_RLOADER);

  PistonA.set_value(true);
  pros::delay(120);

  plan_advance_mm(plan, D_FWD_TO_RLOADER_MM);
  go_plan(
    autopct(drive_slow, comp),
    pos_tol_mm_prec,
    ang_tol_deg_prec,
    1600,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  Conveyor.move(-127);
  pros::delay(900);
  Conveyor.move(0);

  // ========================================================================
  // 3) Back out, hit long goal, then set up for cross
  // ========================================================================
  PistonA.set_value(false);
  pros::delay(60);

  plan_advance_mm(plan, -D_BACK_FROM_RLOADER_MM);
  go_plan(
    autopct(drive_base, comp),
    pos_tol_mm_mid,
    ang_tol_deg_fast,
    3000,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(700);
  Conveyor.move(0);
  Intake.move(0);

  plan_advance_mm(plan, D_BUMP_LONG_MM);
  go_plan(
    autopct(drive_slow, comp),
    pos_tol_mm_prec,
    ang_tol_deg_prec,
    1800,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  turn_imu_deg_2stage(A_TURN_TO_LONG, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
  pros::delay(180);
  plan_turn_deg(plan, A_TURN_TO_LONG);

  // ========================================================================
  // 4) Cross field, score left loader
  // ========================================================================
  plan_advance_mm(plan, D_CROSS_1_MM);
  go_plan(
    autopct(drive_fast, comp),
    pos_tol_mm_fast,
    ang_tol_deg_fast,
    6000,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  turn_imu_deg_2stage(A_TURN_TO_LLOADER, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
  pros::delay(180);
  plan_turn_deg(plan, A_TURN_TO_LLOADER);

  PistonA.set_value(true);
  pros::delay(120);

  plan_advance_mm(plan, D_FWD_TO_LLOADER_MM);
  go_plan(
    autopct(drive_slow, comp),
    pos_tol_mm_prec,
    ang_tol_deg_prec,
    2200,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(120);

  Conveyor.move(127);
  pros::delay(900);
  Conveyor.move(0);

  // ========================================================================
  // 5) Back, collect, cross back, then park
  // ========================================================================
  plan_advance_mm(plan, -D_BACK_FROM_LLOADER_MM);
  go_plan(
    autopct(drive_base, comp),
    pos_tol_mm_mid,
    ang_tol_deg_fast,
    3200,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  Conveyor.move(-127);
  Intake.move(-127);
  pros::delay(1200);

  plan_advance_mm(plan, D_FWD_FROM_LLOADER_MM);
  go_plan(
    autopct(drive_base, comp),
    pos_tol_mm_mid,
    ang_tol_deg_fast,
    2400,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  // Re-orient for return cross
  turn_imu_deg_2stage(-90, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
  pros::delay(180);
  plan_turn_deg(plan, -90.0);

  plan_advance_mm(plan, D_CROSS_2_MM);
  go_plan(
    autopct(drive_fast, comp),
    pos_tol_mm_fast,
    ang_tol_deg_fast,
    3800,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  // Final orientation for park.
  // Your old code uses turn_imu_deg_2stage(A_TURN_TO_PARK) where A_TURN_TO_PARK = 0,
  // which is ambiguous as relative. Keep the behavior as "turn to absolute 0 deg".
  // If your turn function is relative, replace this with (0 - current_heading).
  {
    const double cur = imu_main.get_heading();
    double err = 0.0 - cur;
    while (err > 180.0) err -= 360.0;
    while (err < -180.0) err += 360.0;
    turn_imu_deg_2stage(err, turn_fast, turn_slow, 0.88, 160, 1.0, 1400);
    pros::delay(180);
    plan.heading_deg = 0.0;
  }

  Conveyor.move(0);
  Intake.move(0);
  PistonA.set_value(false);
  pros::delay(80);

  plan_advance_mm(plan, D_FWD_TO_PARK_MM);
  go_plan(
    autopct(drive_fast, comp),
    pos_tol_mm_mid,
    ang_tol_deg_fast,
    2600,
    pros::E_MOTOR_BRAKE_HOLD
  );
  pros::delay(180);

  // --------------------------------------------------------------------------
  // Skills timing guard
  // --------------------------------------------------------------------------
  const uint32_t elapsed = now_ms() - t0;
  if (elapsed < 60000) pros::delay(60000 - elapsed);

  stop_mechs();
}

}  // namespace auton_kevin
