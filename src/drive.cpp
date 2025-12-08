#include "drive.hpp"
#include <cmath>
#include <algorithm>

namespace {
  constexpr double kPi            = 3.1415926535;
  constexpr double kWheelTravelMm = 299.0;   // rueda 3.75
  constexpr double kTrackWidthMm  = 320.0;

  int clamp_percent(int v) {
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
  }

  int pct_to_power(int pct) {
    pct = clamp_percent(pct);
    return static_cast<int>(pct * 127 / 100);
  }
}

Drive drive;

void Drive::reset_encoders() {
  lf.tare_position();
  lm.tare_position();
  lb.tare_position();
  rf.tare_position();
  rm.tare_position();
  rb.tare_position();
}

double Drive::front_left_deg() const {
  return lf.get_position();
}

double Drive::front_right_deg() const {
  return rf.get_position();
}

void Drive::set_brake(pros::motor_brake_mode_e_t mode) {
  lf.set_brake_mode(mode);
  lm.set_brake_mode(mode);
  lb.set_brake_mode(mode);
  rf.set_brake_mode(mode);
  rm.set_brake_mode(mode);
  rb.set_brake_mode(mode);
}

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

double Drive::deg_to_mm(double deg) {
  return (deg / 360.0) * kWheelTravelMm;
}

double Drive::mm_to_deg(double mm) {
  return (mm / kWheelTravelMm) * 360.0;
}

void Drive::drive_straight_mm(
    double dist_mm,
    int base_pct,
    double kp_mm,
    double slow_down_mm,
    pros::motor_brake_mode_e_t end_brake)
{
  const int dir = (dist_mm >= 0.0) ? 1 : -1;
  const double target_mm = std::fabs(dist_mm);

  const int TIMEOUT_MS      = 1400;
  const int STALL_WINDOW_MS = 150;
  const double STALL_MIN_MM = 2.0;

  reset_encoders();
  set_brake(end_brake);

  const int start_time = pros::millis();
  int last_check_time  = start_time;

  double last_avg_mm = 0.0;

  while (true) {
    const int now = pros::millis();

    const double Lmm = std::fabs(deg_to_mm(front_left_deg()));
    const double Rmm = std::fabs(deg_to_mm(front_right_deg()));
    const double avg_mm = 0.5 * (Lmm + Rmm);

    const double remaining = target_mm - avg_mm;
    if (remaining <= 2.0) {
      break;
    }

    if (now - last_check_time >= STALL_WINDOW_MS) {
      const double delta_mm = std::fabs(avg_mm - last_avg_mm);
      if (delta_mm < STALL_MIN_MM) {
        break;
      }
      last_avg_mm    = avg_mm;
      last_check_time = now;
    }

    if (now - start_time >= TIMEOUT_MS) {
      break;
    }

    const double diff_mm = Lmm - Rmm;
    const double trim = kp_mm * diff_mm;

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

    const int left_cmd  =  fast_pct;
    const int right_cmd = -fast_pct;
    set_percent(left_cmd, right_cmd);
    pros::delay(10);
  }
  set_percent(0, 0);
  pros::delay(120);

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

    const int left_cmd  =  slow_pct;
    const int right_cmd = -slow_pct;
    set_percent(left_cmd, right_cmd);
    pros::delay(10);
  }
  set_percent(0, 0);
}

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

    const int left_cmd  = -fast_pct;
    const int right_cmd =  fast_pct;
    set_percent(left_cmd, right_cmd);
    pros::delay(10);
  }
  set_percent(0, 0);
  pros::delay(120);

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

    const int left_cmd  = -slow_pct;
    const int right_cmd =  slow_pct;
    set_percent(left_cmd, right_cmd);
    pros::delay(10);
  }
  set_percent(0, 0);
}
