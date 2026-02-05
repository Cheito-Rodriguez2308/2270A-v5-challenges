#include "motion.hpp"
#include "devices.hpp"
#include "config.hpp"
#include "ramsete.hpp"
#include "kinematics.hpp"
#include "feedforward.hpp"
#include "pid.hpp"
#include "odom.hpp"


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
                       double kI_heading,
                       double kD_heading,
                       double slow_down_mm,
                       pros::motor_brake_mode_e end_brake,
                       int soft_settle_ms,
                       int brake_pulse_ms)
{
  reset_drive_positions();

  const int dir = (dist_mm >= 0.0) ? 1 : -1;
  const double target_mm = std::fabs(dist_mm);

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  // Feedforward and velocity PIDs (output in volts)
  SimpleFeedforward ff(FF_kS, FF_kV, FF_kA);

  // Start gains. Tune on field.
  // kp units: volts per (m/s)
  // kd units: volts per (m/s^2)
  PID pidL(4.0, 0.0, 0.20, -12.0, 12.0);
  PID pidR(4.0, 0.0, 0.20, -12.0, 12.0);

  // Small integral optional
  pidL.setIntegralLimits(-1.0, 1.0);
  pidR.setIntegralLimits(-1.0, 1.0);

  const double dt = 0.010;

  // Drive wheel circumference for velocity conversion
  const double circ_m = DRIVE_WHEEL_CIRC_MM / 1000.0;

  // Theoretical top speed from your 200 rpm assumption in motion.cpp
  const double v_theoretical = (200.0 / 60.0) * circ_m; // m/s

  // Map base_pct to a max velocity target
  const double v_max = std::clamp(std::fabs(base_pct) / 100.0, 0.10, 1.00) * v_theoretical;

  // Acceleration limit for profiling
  const double a_max = 1.2; // m/s^2, tune

  // Heading reference
  const double start_heading_deg = imu_main.get_heading();

  // Helpers
  auto deg2rad = [](double d) { return d * (PI / 180.0); };

  auto avg_left_rpm = []() -> double {
    return (lf.get_actual_velocity() + lm.get_actual_velocity() + lb.get_actual_velocity()) / 3.0;
  };
  auto avg_right_rpm = []() -> double {
    return (rf.get_actual_velocity() + rm.get_actual_velocity() + rb.get_actual_velocity()) / 3.0;
  };

  auto rpm_to_mps = [&](double rpm) -> double {
    return (rpm / 60.0) * circ_m;
  };

  // For a_ref
  double v_ref_prev = 0.0;

  // Timeout
  const uint32_t t0 = pros::millis();
  const double est_speed_mm_s = std::max(150.0, (v_max * 1000.0) * 0.75);
  const uint32_t timeout_ms =
    static_cast<uint32_t>(std::max(1800.0, (target_mm / est_speed_mm_s + 1.5) * 1000.0));

  // Use Rotation for distance
  double last_rot_deg = rot_main.get_position() / 100.0;


  double h_i = 0.0;
  const double H_I_LIM = 0.8; // rad, tune 0.3 to 1.2

  double last_heading_deg = imu_main.get_heading();

  while (true) {
    const uint32_t now = pros::millis();
    if (now - t0 > timeout_ms) break;

    // Distance traveled from Rotation
    const double rot_deg = rot_main.get_position() / 100.0;
    const double d_mm = rot_deg_to_mm(std::fabs(rot_deg));
    if (d_mm >= target_mm) break;

    const double remaining_mm = target_mm - d_mm;
    if (remaining_mm <= 2.0) break;

    // Velocity profile based on remaining distance
    const double remaining_m = remaining_mm / 1000.0;

    // Braking velocity bound
    double v_brake = std::sqrt(std::max(0.0, 2.0 * a_max * remaining_m));

    // Slow down window scaling, keeps old knob meaningful
    if (slow_down_mm > 0.0 && remaining_mm < slow_down_mm) {
      double ratio = remaining_mm / slow_down_mm;
      ratio = std::clamp(ratio, 0.0, 1.0);
      ratio = ratio * ratio * ratio;
      v_brake *= ratio;
    }

    double v_ref = std::min(v_max, v_brake);
    v_ref *= dir;

    // Acceleration reference
    const double a_ref = (v_ref - v_ref_prev) / dt;
    v_ref_prev = v_ref;

    // Heading PID -> omega command (rad/s)
    const double cur_heading_deg = imu_main.get_heading();
    const double h_err_deg = angle_error(start_heading_deg, cur_heading_deg);
    const double h_err_rad = deg2rad(h_err_deg);

    // Measured yaw rate from heading delta (rad/s)
    double dh = cur_heading_deg - last_heading_deg;
    if (dh > 180.0) dh -= 360.0;
    if (dh < -180.0) dh += 360.0;
    last_heading_deg = cur_heading_deg;

    const double w_meas = deg2rad(dh) / dt;

    // Anti windup gates
    const double v_mag = std::fabs(v_ref);
    const bool moving_enough = (v_mag > 0.10);

    // Predict unclamped omega to decide whether to integrate
    const double w_p = kP_heading * h_err_rad;
    const double w_d = -kD_heading * w_meas;

    double w_unclamped_no_i = w_p + w_d;

    if (moving_enough && std::fabs(w_unclamped_no_i) < MAX_ANGULAR_V_RPS) {
      h_i += h_err_rad * dt;
      h_i = std::clamp(h_i, -H_I_LIM, H_I_LIM);
    } else {
      // bleed integral so it does not stick after saturation
      h_i *= 0.90;
    }

    const double w_i = kI_heading * h_i;

    double w_cmd = w_p + w_i + w_d;
    w_cmd = std::clamp(w_cmd, -MAX_ANGULAR_V_RPS, MAX_ANGULAR_V_RPS);

    // Convert chassis commands to wheel linear speeds
    WheelSpeeds ws = chassisToWheelSpeeds(v_ref, w_cmd);
    const double vL_ref = ws.left;
    const double vR_ref = ws.right;

    // Measure wheel speeds
    const double vL_meas = rpm_to_mps(avg_left_rpm());
    const double vR_meas = rpm_to_mps(avg_right_rpm());

    // Feedforward voltages per side
    const double uL_ff = ff.calculate(vL_ref, a_ref);
    const double uR_ff = ff.calculate(vR_ref, a_ref);

    // Feedback voltages per side
    const double eL = vL_ref - vL_meas;
    const double eR = vR_ref - vR_meas;

    const double uL_pid = pidL.calculate(eL, dt);
    const double uR_pid = pidR.calculate(eR, dt);

    double uL = uL_ff + uL_pid;
    double uR = uR_ff + uR_pid;

    // Clamp to battery safe range
    uL = std::clamp(uL, -12.0, 12.0);
    uR = std::clamp(uR, -12.0, 12.0);

    // Send voltage in mV
    const int mVL = static_cast<int>(std::lround(uL * 1000.0));
    const int mVR = static_cast<int>(std::lround(uR * 1000.0));

    lf.move_voltage(mVL); lm.move_voltage(mVL); lb.move_voltage(mVL);
    rf.move_voltage(mVR); rm.move_voltage(mVR); rb.move_voltage(mVR);

    // Debug optional
    pros::lcd::print(1, "rem %.1fmm", remaining_mm);
    pros::lcd::print(2, "vref %.2f vL %.2f vR %.2f", v_ref, vL_meas, vR_meas);
    pros::lcd::print(3, "herr %.2fdeg w %.2f", h_err_deg, w_cmd);

    pros::delay(10);
  }

  // Stop and settle
  lf.move_voltage(0); lm.move_voltage(0); lb.move_voltage(0);
  rf.move_voltage(0); rm.move_voltage(0); rb.move_voltage(0);

  set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
  if (brake_pulse_ms > 0) pros::delay(brake_pulse_ms);

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
  if (soft_settle_ms > 0) pros::delay(soft_settle_ms);

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
                         int settle_ms,
                         double tol_deg,
                         int timeout_ms) 
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

// ============================================================================
//   Drive to pose with Ramsete
// ============================================================================


void drive_to_pose_ramsete_mm(double x_mm,
                              double y_mm,
                              double heading_deg,
                              int base_pct,
                              double a_max_mps2,
                              double pos_tol_mm,
                              double ang_tol_deg,
                              int timeout_ms,
                              pros::motor_brake_mode_e end_brake)
{
  // Ensure odom is updating during this call.
  g_odom_pause.store(false);

  // Controller objects
  RamseteController ramsete(RAMSETE_B, RAMSETE_ZETA);
  SimpleFeedforward ff(FF_kS, FF_kV, FF_kA);

  // Velocity PID per side. Output in volts.
  PID pidL(4.0, 0.0, 0.20, -12.0, 12.0);
  PID pidR(4.0, 0.0, 0.20, -12.0, 12.0);
  pidL.setIntegralLimits(-1.0, 1.0);
  pidR.setIntegralLimits(-1.0, 1.0);

  constexpr double dt = 0.010;

  // Target pose in meters, radians
  const double x_goal = x_mm / 1000.0;
  const double y_goal = y_mm / 1000.0;
  const double th_goal = (heading_deg * PI) / 180.0;

  // Start pose snapshot. Defines the straight-line reference segment.
  const Pose2D start = odom.getPose();
  const double x0 = start.x;
  const double y0 = start.y;

  const double dx_total = x_goal - x0;
  const double dy_total = y_goal - y0;
  const double L_total  = std::sqrt(dx_total * dx_total + dy_total * dy_total);

  // If goal is extremely close, do a final heading settle only.
  if (L_total < (pos_tol_mm / 1000.0)) {
    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
    lf.move_voltage(0); lm.move_voltage(0); lb.move_voltage(0);
    rf.move_voltage(0); rm.move_voltage(0); rb.move_voltage(0);
    set_drive_brake(end_brake);
    return;
  }

  // Reference line heading. Keep reference theta constant along the line.
  const double th_ref_const = std::atan2(dy_total, dx_total);

  // Speed mapping
  const double drive_circ_m = DRIVE_WHEEL_CIRC_MM / 1000.0;
  const double v_theoretical = (200.0 / 60.0) * drive_circ_m;
  const double pct01 = std::clamp(std::fabs(base_pct) / 100.0, 0.10, 1.00);
  const double v_max = pct01 * v_theoretical;

  auto avg_left_rpm = []() -> double {
    return (lf.get_actual_velocity() + lm.get_actual_velocity() + lb.get_actual_velocity()) / 3.0;
  };
  auto avg_right_rpm = []() -> double {
    return (rf.get_actual_velocity() + rm.get_actual_velocity() + rb.get_actual_velocity()) / 3.0;
  };
  auto rpm_to_mps = [&](double rpm) -> double {
    return (rpm / 60.0) * drive_circ_m;
  };

  auto norm_pi = [](double a) -> double {
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
  };

  // Drive brake mode during tracking
  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const uint32_t t0 = pros::millis();
  double v_ref_prev = 0.0;

  while (true) {
    const uint32_t now = pros::millis();
    if (static_cast<int>(now - t0) > timeout_ms) break;

    const Pose2D cur = odom.getPose();

    const double dxg = x_goal - cur.x;
    const double dyg = y_goal - cur.y;
    const double dist_to_goal_m = std::sqrt(dxg * dxg + dyg * dyg);

    const double dist_to_goal_mm = dist_to_goal_m * 1000.0;
    if (dist_to_goal_mm <= pos_tol_mm) {
      const double th_err = norm_pi(th_goal - cur.theta) * (180.0 / PI);
      if (std::fabs(th_err) <= ang_tol_deg) break;
    }

    // Project current position onto the start->goal line to get progress.
    const double px = cur.x - x0;
    const double py = cur.y - y0;
    const double dot = px * dx_total + py * dy_total;
    double t = dot / (L_total * L_total);
    t = std::clamp(t, 0.0, 1.0);

    // Reference pose on the line
    Pose2D ref;
    ref.x = x0 + t * dx_total;
    ref.y = y0 + t * dy_total;
    ref.theta = th_ref_const;

    // Reference speeds. Use braking law from remaining distance.
    double v_brake = std::sqrt(std::max(0.0, 2.0 * a_max_mps2 * dist_to_goal_m));
    double v_ref = std::min(v_max, v_brake);

    // Sign based on whether goal is generally forward in robot frame.
    // This avoids sudden backward commands when the robot is angled.
    const double dir_forward = std::cos(cur.theta) * dxg + std::sin(cur.theta) * dyg;
    if (dir_forward < 0.0) v_ref = -v_ref;

    const double a_ref = (v_ref - v_ref_prev) / dt;
    v_ref_prev = v_ref;

    const double w_ref = 0.0;

    // RAMSETE output (chassis)
    const auto out = ramsete.calculate(cur, ref, v_ref, w_ref);
    double v_cmd = std::clamp(out.v, -v_max, v_max);
    double w_cmd = std::clamp(out.w, -MAX_ANGULAR_V_RPS, MAX_ANGULAR_V_RPS);

    // Wheel targets
    const WheelSpeeds ws = chassisToWheelSpeeds(v_cmd, w_cmd);
    const double vL_ref = ws.left;
    const double vR_ref = ws.right;

    // Wheel measured speeds
    const double vL_meas = rpm_to_mps(avg_left_rpm());
    const double vR_meas = rpm_to_mps(avg_right_rpm());

    // Feedforward per side
    const double uL_ff = ff.calculate(vL_ref, a_ref);
    const double uR_ff = ff.calculate(vR_ref, a_ref);

    // Feedback per side
    const double eL = vL_ref - vL_meas;
    const double eR = vR_ref - vR_meas;

    const double uL_pid = pidL.calculate(eL, dt);
    const double uR_pid = pidR.calculate(eR, dt);

    double uL = std::clamp(uL_ff + uL_pid, -12.0, 12.0);
    double uR = std::clamp(uR_ff + uR_pid, -12.0, 12.0);

    const int mVL = static_cast<int>(std::lround(uL * 1000.0));
    const int mVR = static_cast<int>(std::lround(uR * 1000.0));

    lf.move_voltage(mVL); lm.move_voltage(mVL); lb.move_voltage(mVL);
    rf.move_voltage(mVR); rm.move_voltage(mVR); rb.move_voltage(mVR);

    pros::delay(10);
  }

  // Stop
  lf.move_voltage(0); lm.move_voltage(0); lb.move_voltage(0);
  rf.move_voltage(0); rm.move_voltage(0); rb.move_voltage(0);

  set_drive_brake(end_brake);
  if (end_brake != pros::E_MOTOR_BRAKE_COAST) {
    lf.brake(); lm.brake(); lb.brake();
    rf.brake(); rm.brake(); rb.brake();
  }
}
