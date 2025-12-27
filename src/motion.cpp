#include "motion.hpp"
#include "devices.hpp"
#include "api.h"

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
//   Geometry and drivetrain constants
// ============================================================================

// Wheel diameter 3.25 in. Circumference in mm.
// 3.25 * 25.4 = 82.55 mm. 82.55 * pi = 259.3 mm.
static constexpr double WHEEL_TRAVEL_MM = 259.3;

// Rotation to wheel ratio.
static constexpr double GEAR_RATIO_ROT_TO_WHEEL = 64.0 / 36.0;

// Motor gearset 18 max rpm approximation.
static constexpr int MAX_RPM_18 = 200;

// ============================================================================
//   Reset
// ============================================================================

/**
 * \brief Zero the drive motor encoders and rotation sensor.
 */
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

/**
 * \brief Rotation degrees to millimeters traveled.
 */
double rot_deg_to_mm(double deg) {
  const double rev_rot   = deg / 360.0;
  const double rev_wheel = rev_rot * GEAR_RATIO_ROT_TO_WHEEL;
  return rev_wheel * WHEEL_TRAVEL_MM;
}

/**
 * \brief Millimeters traveled to Rotation degrees.
 */
double mm_to_rot_deg(double mm) {
  const double rev_wheel = mm / WHEEL_TRAVEL_MM;
  const double rev_rot   = rev_wheel / GEAR_RATIO_ROT_TO_WHEEL;
  return rev_rot * 360.0;
}

// ============================================================================
//   Angle helper
// ============================================================================

/**
 * \brief Signed shortest angle error in degrees in [-180, 180].
 */
double angle_error(double target, double current) {
  double err = target - current;
  while (err > 180) err -= 360;
  while (err < -180) err += 360;
  return err;
}

// ============================================================================
//   Internal helpers
// ============================================================================

/**
 * \brief Percent command to rpm for a 200 rpm gearset.
 */
static int pct_to_rpm(int pct) {
  pct = std::clamp(pct, -100, 100);
  return (pct * MAX_RPM_18) / 100;
}

/**
 * \brief Ensures enough rpm to overcome friction.
 *
 * \details Uses a higher minimum when far from target and a lower minimum
 * near the end to avoid overshoot.
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

// ============================================================================
//   Drive straight
// ============================================================================

/**
 * \brief Drive straight using Rotation for distance and IMU for heading.
 */
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
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  // Reference heading for heading hold.
  const double start_heading = imu_main.get_heading();

  // Timeout. Prevents hanging on stalls.
  const uint32_t t0 = pros::millis();
  const double est_speed_mm_s = std::max(180.0, (std::abs(base_pct) / 100.0) * 350.0);
  const uint32_t timeout_ms =
    static_cast<uint32_t>(std::max(2000.0, (target_mm / est_speed_mm_s + 1.0) * 1000.0));

  // RPM slew state. Smooth start and stop.
  int left_rpm_cmd  = 0;
  int right_rpm_cmd = 0;
  const int RPM_SLEW_PER_10MS = 18;

  /**
   * \brief Slew helper. Limits rpm delta per 10 ms.
   */
  auto slew_to = [&](int target, int current) {
    const int d = target - current;
    if (std::abs(d) <= RPM_SLEW_PER_10MS) return target;
    return current + ((d > 0) ? RPM_SLEW_PER_10MS : -RPM_SLEW_PER_10MS);
  };

  while (true) {
    const uint32_t now = pros::millis();
    if (now - t0 > timeout_ms) break;

    // Rotation based distance.
    const double rot_deg = std::abs(rot_main.get_position()) / 100.0;
    const double rot_mm  = rot_deg_to_mm(rot_deg);

    // Motor fallback distance for rare rotation glitches.
    const double mot_deg = std::abs(lf.get_position());
    const double mot_mm  = (mot_deg / 360.0) * WHEEL_TRAVEL_MM;

    const double d_mm = (rot_mm < 1.0 && mot_mm > 3.0) ? mot_mm : rot_mm;

    const double remaining = target_mm - d_mm;
    if (remaining <= 2.0) break;

    // Heading correction.
    const double heading = imu_main.get_heading();
    const double err = angle_error(start_heading, heading);

    double trim = kP_heading * err;
    trim = std::clamp(trim, -15.0, 15.0);

    // Slowdown profile.
    int pct = base_pct;
    if (slow_down_mm > 0.0 && remaining < slow_down_mm) {
      pct = std::max(35, static_cast<int>(std::round(base_pct * (remaining / slow_down_mm))));
    }

    int left_pct  = static_cast<int>(std::round(pct - trim));
    int right_pct = static_cast<int>(std::round(pct + trim));

    left_pct  = std::clamp(left_pct,  -100, 100);
    right_pct = std::clamp(right_pct, -100, 100);

    // Convert to rpm and apply minimum rpm.
    const int left_target_rpm  = apply_min_rpm(pct_to_rpm(dir * left_pct),  remaining);
    const int right_target_rpm = apply_min_rpm(pct_to_rpm(dir * right_pct), remaining);

    // Slew to targets.
    left_rpm_cmd  = slew_to(left_target_rpm,  left_rpm_cmd);
    right_rpm_cmd = slew_to(right_target_rpm, right_rpm_cmd);

    // Output.
    lf.move_velocity(left_rpm_cmd);
    lm.move_velocity(left_rpm_cmd);
    lb.move_velocity(left_rpm_cmd);

    rf.move_velocity(right_rpm_cmd);
    rm.move_velocity(right_rpm_cmd);
    rb.move_velocity(right_rpm_cmd);

    pros::delay(10);
  }

  // Ramp down to zero.
  for (int i = 0; i < 8; i++) {
    left_rpm_cmd  = slew_to(0, left_rpm_cmd);
    right_rpm_cmd = slew_to(0, right_rpm_cmd);

    lf.move_velocity(left_rpm_cmd);
    lm.move_velocity(left_rpm_cmd);
    lb.move_velocity(left_rpm_cmd);

    rf.move_velocity(right_rpm_cmd);
    rm.move_velocity(right_rpm_cmd);
    rb.move_velocity(right_rpm_cmd);

    pros::delay(10);
  }

  // Brake pulse. Seats the robot without HOLD.
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  lf.move_velocity(0);
  lm.move_velocity(0);
  lb.move_velocity(0);
  rf.move_velocity(0);
  rm.move_velocity(0);
  rb.move_velocity(0);

  if (brake_pulse_ms > 0) {
    pros::delay(brake_pulse_ms);
  }

  // Coast settle. Prevents harsh stop feel.
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  if (soft_settle_ms > 0) {
    pros::delay(soft_settle_ms);
  }

  // Final brake mode for the caller.
  lf.set_brake_mode(end_brake);
  lm.set_brake_mode(end_brake);
  lb.set_brake_mode(end_brake);
  rf.set_brake_mode(end_brake);
  rm.set_brake_mode(end_brake);
  rb.set_brake_mode(end_brake);

  // Only apply an explicit brake when final mode is not COAST.
  if (end_brake != pros::E_MOTOR_BRAKE_COAST) {
    lf.brake();
    lm.brake();
    lb.brake();
    rf.brake();
    rm.brake();
    rb.brake();
  }
}

// ============================================================================
//   Turn in two stages
// ============================================================================

/**
 * \brief Turn using IMU. Stage 1 does most of the angle. Stage 2 finishes slowly.
 */
void turn_imu_deg_2stage(double deg_total,
                         int fast_pct,
                         int slow_pct,
                         double split,
                         int settle_ms)
{
  const double start = imu_main.get_heading();
  double target_1 = start + deg_total * split;
  double target_2 = start + deg_total;

  /**
   * \brief Normalize heading to [0, 360).
   */
  auto norm = [&](double a) {
    while (a >= 360) a -= 360;
    while (a < 0) a += 360;
    return a;
  };

  target_1 = norm(target_1);
  target_2 = norm(target_2);

  // Stage 1.
  while (true) {
    const double cur = imu_main.get_heading();
    const double err = angle_error(target_1, cur);
    if (std::abs(err) < 1.5) break;

    const int sign = (err > 0) ? 1 : -1;
    const int cmd_rpm = pct_to_rpm(sign * fast_pct);

    lf.move_velocity(cmd_rpm);
    lm.move_velocity(cmd_rpm);
    lb.move_velocity(cmd_rpm);

    rf.move_velocity(-cmd_rpm);
    rm.move_velocity(-cmd_rpm);
    rb.move_velocity(-cmd_rpm);

    pros::delay(10);
  }

  if (settle_ms > 0) {
    pros::delay(settle_ms);
  }

  // Stage 2.
  while (true) {
    const double cur = imu_main.get_heading();
    const double err = angle_error(target_2, cur);
    if (std::abs(err) < 1.0) break;

    const int sign = (err > 0) ? 1 : -1;
    const int cmd_rpm = pct_to_rpm(sign * slow_pct);

    lf.move_velocity(cmd_rpm);
    lm.move_velocity(cmd_rpm);
    lb.move_velocity(cmd_rpm);

    rf.move_velocity(-cmd_rpm);
    rm.move_velocity(-cmd_rpm);
    rb.move_velocity(-cmd_rpm);

    pros::delay(10);
  }

  // Stop.
  lf.brake();
  lm.brake();
  lb.brake();
  rf.brake();
  rm.brake();
  rb.brake();
}
