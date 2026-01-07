#include "drive.hpp"
#include "config.hpp"
#include "api.h"


/**
 * \file drive.cpp
 *
 * \brief Implementation of Drive primitives.
 *
 * \par Critical warning
 *   - kWheelTravelMm is set for 3.75 inch wheels in this file.
 *   - Your robot uses 3.25 inch wheels.
 *   - If you use Drive::deg_to_mm for auton distances, it will underdrive.
 *
 * \par Fix for 3.25 inch wheels
 *   - Diameter 3.25 in -> 82.55 mm
 *   - Circumference = pi * d = 259.3 mm
 *   - Set kWheelTravelMm to ~259.3
 */

namespace {
  constexpr double kPi            = 3.1415926535;

  // Wheel travel per 1 revolution in mm.
  // 3.25 in diameter => ~259.3 mm
  constexpr double kWheelTravelMm = DRIVE_WHEEL_CIRC_MM;

  // Track width in mm. Must match center to center distance of left vs right wheels.
  constexpr double kTrackWidthMm  = 320.0;

  /**
   * \brief Clamp percent command to the safe range [-100, 100].
   */
  int clamp_percent(int v) {
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
  }

  /**
   * \brief Convert percent [-100, 100] to motor power [-127, 127].
   */
  int pct_to_power(int pct) {
    pct = clamp_percent(pct);
    return static_cast<int>(pct * 127 / 100);
  }
}

Drive drive;

/**
 * \brief Reset integrated encoders on all 6 drive motors.
 */
void Drive::reset_encoders() {
  lf.tare_position();
  lm.tare_position();
  lb.tare_position();
  rf.tare_position();
  rm.tare_position();
  rb.tare_position();
}

/**
 * \brief Get left reference motor position in degrees.
 */
double Drive::front_left_deg() const {
  return lf.get_position();
}

/**
 * \brief Get right reference motor position in degrees.
 */
double Drive::front_right_deg() const {
  return rf.get_position();
}

/**
 * \brief Set brake mode on all 6 drive motors.
 */
void Drive::set_brake(pros::motor_brake_mode_e_t mode) {
  lf.set_brake_mode(mode);
  lm.set_brake_mode(mode);
  lb.set_brake_mode(mode);
  rf.set_brake_mode(mode);
  rm.set_brake_mode(mode);
  rb.set_brake_mode(mode);
}

/**
 * \brief Apply open loop tank commands to the chassis.
 */
void Drive::set_percent(int left_pct, int right_pct) {
  const int left_power  = pct_to_power(left_pct);
  const int right_power = pct_to_power(right_pct);

  lf.move(left_power);
  lm.move(left_power);
  lb.move(left_power);

  rf.move(right_power);
  rm.move(right_power);
  rb.move(right_power);
}

/**
 * \brief Convert motor degrees to millimeters using kWheelTravelMm.
 */
double Drive::deg_to_mm(double deg) {
  return (deg / 360.0) * kWheelTravelMm;
}

/**
 * \brief Convert millimeters to motor degrees using kWheelTravelMm.
 */
double Drive::mm_to_deg(double mm) {
  return (mm / kWheelTravelMm) * 360.0;
}

/**
 * \brief Drive straight using encoder feedback and left-right trim.
 *
 * \details
 *   - Uses absolute distance traveled on each side
 *   - Trim pushes the faster side down
 *   - Slowdown scales base power inside the last slow_down_mm window
 *   - Stall detection exits if motion stops
 */
void Drive::drive_straight_mm(
    double dist_mm,
    int base_pct,
    double kp_mm,
    double slow_down_mm,
    pros::motor_brake_mode_e_t end_brake)
{
  const int dir = (dist_mm >= 0.0) ? 1 : -1;
  const double target_mm = std::fabs(dist_mm);

  // Safety: keep these conservative. Increase only after testing.
  const int TIMEOUT_MS      = 1400;
  const int STALL_WINDOW_MS = 150;
  const double STALL_MIN_MM = 2.0;

  reset_encoders();
  set_brake(end_brake);

  const int start_time = pros::millis();
  int last_check_time  = start_time;
  double last_avg_mm   = 0.0;

  while (true) {
    const int now = pros::millis();

    const double Lmm = std::fabs(deg_to_mm(front_left_deg()));
    const double Rmm = std::fabs(deg_to_mm(front_right_deg()));
    const double avg_mm = 0.5 * (Lmm + Rmm);

    const double remaining = target_mm - avg_mm;
    if (remaining <= 2.0) {
      break;
    }

    // Stall detection
    if (now - last_check_time >= STALL_WINDOW_MS) {
      const double delta_mm = std::fabs(avg_mm - last_avg_mm);
      if (delta_mm < STALL_MIN_MM) {
        break;
      }
      last_avg_mm     = avg_mm;
      last_check_time = now;
    }

    // Timeout
    if (now - start_time >= TIMEOUT_MS) {
      break;
    }

    // Trim based on left-right distance mismatch
    const double diff_mm = Lmm - Rmm;
    const double trim = kp_mm * diff_mm;

    // Slowdown near target
    int pct = base_pct;
    if (remaining < slow_down_mm && slow_down_mm > 0.0) {
      const double scale = remaining / slow_down_mm;
      const double safe_scale = std::max(0.1, scale);
      const int scaled = static_cast<int>(base_pct * safe_scale);
      pct = std::max(25, scaled);
    }

    const int left_cmd  = clamp_percent(static_cast<int>((pct - trim) * dir));
    const int right_cmd = clamp_percent(static_cast<int>((pct + trim) * dir));

    set_percent(left_cmd, right_cmd);
    pros::delay(10);
  }

  set_percent(0, 0);
}

/**
 * \brief Turn right by converting degrees to arc length and running two stages.
 */
void Drive::turn_right_deg(double deg_total, int fast_pct, int slow_pct) {
  if (deg_total <= 0.0) return;

  const double arc_mm  = kPi * kTrackWidthMm * (deg_total / 360.0);
  const double fast_mm = arc_mm * 0.92;
  const double slow_mm = arc_mm - fast_mm;

  const int TIMEOUT_FAST_MS   = 1000;
  const int TIMEOUT_SLOW_MS   = 800;
  const int STALL_WINDOW_MS   = 150;
  const double STALL_MIN_MM   = 1.5;

  reset_encoders();
  int start_time = pros::millis();
  int last_check_time = start_time;
  double last_avg_mm = 0.0;

  // Stage 1: fast
  while (true) {
    const int now = pros::millis();

    const double Lmm = std::fabs(deg_to_mm(front_left_deg()));
    const double Rmm = std::fabs(deg_to_mm(front_right_deg()));
    const double avg_mm = 0.5 * (Lmm + Rmm);
    if (avg_mm >= fast_mm) break;

    if (now - last_check_time >= STALL_WINDOW_MS) {
      const double delta_mm = std::fabs(avg_mm - last_avg_mm);
      if (delta_mm < STALL_MIN_MM) {
        break;
      }
      last_avg_mm = avg_mm;
      last_check_time = now;
    }

    if (now - start_time >= TIMEOUT_FAST_MS) {
      break;
    }

    set_percent( fast_pct, -fast_pct);
    pros::delay(10);
  }

  set_percent(0, 0);
  pros::delay(120);

  // Stage 2: slow
  reset_encoders();
  start_time      = pros::millis();
  last_check_time = start_time;
  last_avg_mm     = 0.0;

  while (true) {
    const int now = pros::millis();

    const double Lmm = std::fabs(deg_to_mm(front_left_deg()));
    const double Rmm = std::fabs(deg_to_mm(front_right_deg()));
    const double avg_mm = 0.5 * (Lmm + Rmm);
    if (avg_mm >= slow_mm) break;

    if (now - last_check_time >= STALL_WINDOW_MS) {
      const double delta_mm = std::fabs(avg_mm - last_avg_mm);
      if (delta_mm < STALL_MIN_MM) {
        break;
      }
      last_avg_mm = avg_mm;
      last_check_time = now;
    }

    if (now - start_time >= TIMEOUT_SLOW_MS) {
      break;
    }

    set_percent( slow_pct, -slow_pct);
    pros::delay(10);
  }

  set_percent(0, 0);
}

/**
 * \brief Turn left by converting degrees to arc length and running two stages.
 */
void Drive::turn_left_deg(double deg_total, int fast_pct, int slow_pct) {
  if (deg_total <= 0.0) return;

  const double arc_mm  = kPi * kTrackWidthMm * (deg_total / 360.0);
  const double fast_mm = arc_mm * 0.92;
  const double slow_mm = arc_mm - fast_mm;

  const int TIMEOUT_FAST_MS   = 1000;
  const int TIMEOUT_SLOW_MS   = 800;
  const int STALL_WINDOW_MS   = 150;
  const double STALL_MIN_MM   = 1.5;

  reset_encoders();
  int start_time = pros::millis();
  int last_check_time = start_time;
  double last_avg_mm = 0.0;

  // Stage 1: fast
  while (true) {
    const int now = pros::millis();

    const double Lmm = std::fabs(deg_to_mm(front_left_deg()));
    const double Rmm = std::fabs(deg_to_mm(front_right_deg()));
    const double avg_mm = 0.5 * (Lmm + Rmm);
    if (avg_mm >= fast_mm) break;

    if (now - last_check_time >= STALL_WINDOW_MS) {
      const double delta_mm = std::fabs(avg_mm - last_avg_mm);
      if (delta_mm < STALL_MIN_MM) {
        break;
      }
      last_avg_mm = avg_mm;
      last_check_time = now;
    }

    if (now - start_time >= TIMEOUT_FAST_MS) {
      break;
    }

    set_percent(-fast_pct,  fast_pct);
    pros::delay(10);
  }

  set_percent(0, 0);
  pros::delay(120);

  // Stage 2: slow
  reset_encoders();
  start_time      = pros::millis();
  last_check_time = start_time;
  last_avg_mm     = 0.0;

  while (true) {
    const int now = pros::millis();

    const double Lmm = std::fabs(deg_to_mm(front_left_deg()));
    const double Rmm = std::fabs(deg_to_mm(front_right_deg()));
    const double avg_mm = 0.5 * (Lmm + Rmm);
    if (avg_mm >= slow_mm) break;

    if (now - last_check_time >= STALL_WINDOW_MS) {
      const double delta_mm = std::fabs(avg_mm - last_avg_mm);
      if (delta_mm < STALL_MIN_MM) {
        break;
      }
      last_avg_mm = avg_mm;
      last_check_time = now;
    }

    if (now - start_time >= TIMEOUT_SLOW_MS) {
      break;
    }

    set_percent(-slow_pct,  slow_pct);
    pros::delay(10);
  }

  set_percent(0, 0);
}
