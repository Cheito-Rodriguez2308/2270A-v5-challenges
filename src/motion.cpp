#include "motion.hpp"
#include "devices.hpp"
#include "config.hpp"
#include "api.h"

#include <algorithm>
#include <cmath>

/**
 * \file motion.cpp
 *
 * \brief Motion primitives implementation.
 *
 * \par Main idea
 *   - Rotation sensor measures distance reliably
 *   - IMU holds heading during straight drives
 *   - RPM slew smooths acceleration and braking
 *   - Brake pulse seats the robot, then COAST keeps it feeling natural
 *
 * \par Critical tuning knobs
 *   - slow_down_mm controls how early you start slowing down
 *   - kP_heading controls drift correction strength
 *   - brake_pulse_ms controls how hard the stop feels
 *   - soft_settle_ms controls how much the robot "relaxes" after the stop
 */

// ============================================================================
//   _____           _
//  |  __ \         (_)
//  | |  | | _____  ___  ___ ___
//  | |  | |/ _ \ \/ / |/ __/ _ \
//  | |__| |  __/>  <| | (_|  __/
//  |_____/ \___/_/\_\_|\___\___|
//
//  motion.cpp  (Rotation + IMU)  "worlds-style stable"
// ============================================================================
//
// What was added vs your current file
//  - Dynamic heading kP + angular deadband by phase (far, mid, near)
//  - Cubic decel profile near target (ratio^3) for less overshoot
//  - Turn speed profile + RPM slew for less zig-zag
//  - Small safety timeouts inside turn loops
//
// What stayed the same
//  - Same motion.hpp signatures
//  - Rotation distance primary with motor fallback
//  - Brake pulse then coast settle, end_brake from caller
// ============================================================================

// ============================================================================
//   Geometry and drivetrain constants
// ============================================================================

// Motor gearset 18 max rpm approximation.
static constexpr int MAX_RPM_18 = 200;

// ============================================================================
//   Internal helpers
// ============================================================================

static inline int clamp_i(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline double clamp_d(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/**
 * \brief Percent command to rpm for a 200 rpm gearset.
 */
static int pct_to_rpm(int pct) {
  pct = clamp_i(pct, -100, 100);
  return (pct * MAX_RPM_18) / 100;
}

/**
 * \brief Ensures enough rpm to overcome friction.
 */
static int apply_min_rpm(int rpm, double remaining_mm) {
  if (rpm == 0) return 0;

  const int sign = (rpm > 0) ? 1 : -1;
  int mag = std::abs(rpm);

  const int min_far  = 60;
  const int min_near = 30;
  const int min_rpm  = (remaining_mm > 180.0) ? min_far : min_near;

  mag = std::max(mag, min_rpm);
  return sign * mag;
}

/**
 * \brief Slew helper. Limits rpm delta per 10 ms.
 */
static int slew_to(int target, int current, int step) {
  const int d = target - current;
  if (std::abs(d) <= step) return target;
  return current + ((d > 0) ? step : -step);
}

/**
 * \brief Set brake mode on all drive motors.
 */
static void set_drive_brake(pros::motor_brake_mode_e mode) {
  lf.set_brake_mode(mode);
  lm.set_brake_mode(mode);
  lb.set_brake_mode(mode);
  rf.set_brake_mode(mode);
  rm.set_brake_mode(mode);
  rb.set_brake_mode(mode);
}

/**
 * \brief Output RPM to drivetrain (left, right).
 */
static void set_drive_rpm(int l_rpm, int r_rpm) {
  lf.move_velocity(l_rpm);
  lm.move_velocity(l_rpm);
  lb.move_velocity(l_rpm);

  rf.move_velocity(r_rpm);
  rm.move_velocity(r_rpm);
  rb.move_velocity(r_rpm);
}

/**
 * \brief Normalize heading to [0, 360).
 */
static double norm360(double a) {
  while (a >= 360.0) a -= 360.0;
  while (a < 0.0) a += 360.0;
  return a;
}

// ============================================================================
//   Reset
// ============================================================================

void reset_drive_positions() {
  lf.tare_position();
  lm.tare_position();
  lb.tare_position();
  rf.tare_position();
  rm.tare_position();
  rb.tare_position();
  rot_main.reset_position();
}

// ============================================================================
//   Conversion functions
// ============================================================================

double rot_deg_to_mm(double deg) {
  const double rev_rot = deg / 360.0;
  const double rev_wheel = rev_rot * TRACKING_GEAR_RATIO;
  return rev_wheel * TRACKING_WHEEL_CIRC_MM * TRACKING_SCALE;
}

double mm_to_rot_deg(double mm) {
  const double rev_wheel = mm / (TRACKING_WHEEL_CIRC_MM * TRACKING_SCALE);
  const double rev_rot = rev_wheel / TRACKING_GEAR_RATIO;
  return rev_rot * 360.0;
}

// ============================================================================
//   Angle helper
// ============================================================================

double angle_error(double target, double current) {
  double err = target - current;
  while (err > 180) err -= 360;
  while (err < -180) err += 360;
  return err;
}

// ============================================================================
//   Drive straight
// ============================================================================

  void drive_straight_mm(double dist_mm,
                        int base_pct,
                        double kP_heading,
                        double slow_down_mm,
                        pros::motor_brake_mode_e end_brake,
                        int soft_settle_ms,
                        int brake_pulse_ms)
{
  reset_drive_positions();

  const int dir = (dist_mm >= 0) ? 1 : -1;
  const double target_mm = std::abs(dist_mm);

  // During motion. Always COAST for smoother feel.
  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  // Reference heading for heading hold.
  const double start_heading = imu_main.get_heading();

  // Timeout. Prevents hanging on stalls.
  const uint32_t t0 = pros::millis();
  const double est_speed_mm_s = std::max(200.0, (std::abs(base_pct) / 100.0) * 600.0);
  const uint32_t timeout_ms =
    static_cast<uint32_t>(std::max(2000.0, (target_mm / est_speed_mm_s + 1.2) * 1000.0));

  // RPM slew state.
  int left_rpm_cmd  = 0;
  int right_rpm_cmd = 0;
  const int RPM_SLEW_PER_10MS = 18;

  while (true) {
    const uint32_t now = pros::millis();
    if (now - t0 > timeout_ms) break;

    // Rotation based distance.
    const double rot_deg = std::abs(rot_main.get_position()) / 100.0;
    const double rot_mm  = rot_deg_to_mm(rot_deg);

    // Motor fallback distance for rare rotation glitches.
    const double mot_deg = std::abs(rot_main.get_position());
    const double mot_mm  = (mot_deg / 360.0) * DRIVE_WHEEL_CIRC_MM;

    // const double d_mm = (rot_mm < 1.0 && mot_mm > 3.0) ? mot_mm : rot_mm;
    const double d_mm = rot_mm;
    if (d_mm >= target_mm) break;

    const double remaining = target_mm - d_mm;
    pros::lcd::print(1, "Target: %.2f", target_mm);
    pros::lcd::print(2, "Driven: %.2f", d_mm);
    pros::lcd::print(3, "Remaining: %.2f", remaining);
    if (remaining <= 2.0) break;

    // Phase based tuning (worlds style).
    const double phase = (target_mm > 1.0) ? (remaining / target_mm) : 0.0;

    double tol_deg = 2.0;
    double kp_use  = kP_heading;

    if (phase > 0.60) {
      tol_deg = 4.0;
      kp_use  = kP_heading * 0.60;
    } else if (phase > 0.30) {
      tol_deg = 2.0;
      kp_use  = kP_heading;
    } else {
      tol_deg = 1.0;
      kp_use  = kP_heading * 1.20;
    }

    // Heading correction with deadband.
    const double heading = imu_main.get_heading();
    double err = angle_error(start_heading, heading);
    if (std::abs(err) < tol_deg) err = 0.0;

    double trim = kp_use * err;
    trim = clamp_d(trim, -15.0, 15.0);

    // Cubic slowdown profile.
    int pct = base_pct;
    if (slow_down_mm > 0.0 && remaining < slow_down_mm) {
      double ratio = clamp_d(remaining / slow_down_mm, 0.0, 1.0);
      ratio = ratio * ratio * ratio;
      pct = std::max(35, static_cast<int>(std::lround(base_pct * ratio)));
    }

    int left_pct  = static_cast<int>(std::lround(pct - trim));
    int right_pct = static_cast<int>(std::lround(pct + trim));

    left_pct  = clamp_i(left_pct,  -100, 100);
    right_pct = clamp_i(right_pct, -100, 100);

    // Convert to rpm and apply minimum rpm.
    const int left_target_rpm  = apply_min_rpm(pct_to_rpm(dir * left_pct),  remaining);
    const int right_target_rpm = apply_min_rpm(pct_to_rpm(dir * right_pct), remaining);

    // Slew to targets.
    left_rpm_cmd  = slew_to(left_target_rpm,  left_rpm_cmd,  RPM_SLEW_PER_10MS);
    right_rpm_cmd = slew_to(right_target_rpm, right_rpm_cmd, RPM_SLEW_PER_10MS);

    set_drive_rpm(left_rpm_cmd, right_rpm_cmd);
    pros::delay(10);
  }

  // Ramp down to zero.
  for (int i = 0; i < 8; i++) {
    left_rpm_cmd  = slew_to(0, left_rpm_cmd,  RPM_SLEW_PER_10MS);
    right_rpm_cmd = slew_to(0, right_rpm_cmd, RPM_SLEW_PER_10MS);
    set_drive_rpm(left_rpm_cmd, right_rpm_cmd);
    pros::delay(10);
  }

  // Brake pulse. Seats the robot without HOLD.
  set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
  set_drive_rpm(0, 0);

  if (brake_pulse_ms > 0) pros::delay(brake_pulse_ms);

  // Coast settle.
  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
  if (soft_settle_ms > 0) pros::delay(soft_settle_ms);

  // Final brake mode for the caller.
  set_drive_brake(end_brake);
  if (end_brake != pros::E_MOTOR_BRAKE_COAST) {
    lf.brake(); lm.brake(); lb.brake();
    rf.brake(); rm.brake(); rb.brake();
  }
}

// ============================================================================
//   Turn in two stages
// ============================================================================
//
// Goal
//  - No zig-zag near the end
//  - Speed falls as |error| falls
//  - RPM slew keeps the turn smooth
//
// Tuning
//  - kV controls how fast command rises with error
//  - min_pct prevents stall
//  - stop tolerances decide when each stage ends
// ============================================================================

static void turn_to_heading_profile(double target,
                                   int max_pct,
                                   int min_pct,
                                   double stop_tol_deg,
                                   uint32_t timeout_ms)
{
  max_pct = clamp_i(max_pct, 10, 100);
  min_pct = clamp_i(min_pct,  6, max_pct);

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const uint32_t t0 = pros::millis();

  int l_cmd = 0;
  int r_cmd = 0;
  const int RPM_SLEW_PER_10MS = 20;

  while (true) {
    if (pros::millis() - t0 > timeout_ms) break;

    const double cur = imu_main.get_heading();
    const double err = angle_error(target, cur);
    if (std::abs(err) <= stop_tol_deg) break;

    // Speed profile.
    const double kV = 1.2;
    double cmd_pct = kV * std::abs(err);
    cmd_pct = clamp_d(cmd_pct, static_cast<double>(min_pct), static_cast<double>(max_pct));

    const int sign = (err > 0.0) ? 1 : -1;
    const int rpm = pct_to_rpm(sign * static_cast<int>(std::lround(cmd_pct)));

    const int l_target =  rpm;
    const int r_target = -rpm;

    l_cmd = slew_to(l_target, l_cmd, RPM_SLEW_PER_10MS);
    r_cmd = slew_to(r_target, r_cmd, RPM_SLEW_PER_10MS);

    set_drive_rpm(l_cmd, r_cmd);
    pros::delay(10);
  }

  // Small settle stop.
  set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
  set_drive_rpm(0, 0);
  pros::delay(60);
  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
}

void turn_imu_deg_2stage(double deg_total,
                         int fast_pct,
                         int slow_pct,
                         double split,
                         int settle_ms)
{
  const double start = imu_main.get_heading();

  split = clamp_d(split, 0.50, 0.98);

  const double target_1 = norm360(start + deg_total * split);
  const double target_2 = norm360(start + deg_total);

  // Stage 1: faster, looser tol.
  turn_to_heading_profile(
    target_1,
    fast_pct,
    std::max(10, slow_pct),
    1.6,
    2400
  );

  if (settle_ms > 0) pros::delay(settle_ms);

  // Stage 2: slower, tighter tol.
  turn_to_heading_profile(
    target_2,
    std::max(12, slow_pct),
    8,
    1.0,
    2400
  );

  // Final stop feel.
  lf.brake(); lm.brake(); lb.brake();
  rf.brake(); rm.brake(); rb.brake();
}