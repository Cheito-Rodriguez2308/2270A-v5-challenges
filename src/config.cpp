#include "config.hpp"
#include "api.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>

// ======================================================
// TUNING GLOBAL: Cfg cfg
// ======================================================

const Cfg cfg{
  75,   // DRIVE_MAX_PCT
  60,   // TURN_MAX_PCT
  95,   // TURBO_DRIVE_MAX_PCT
  80,   // TURBO_TURN_MAX_PCT
  55,   // AUTO_DRIVE_PCT
  35,   // AUTO_TURN_PCT
  1,    // DEADBAND_FWD
  3,    // DEADBAND_TURN
  2.2,  // SENSITIVITY_SOFT
  2.5,  // SENSITIVITY_TURBO
  8.0,  // SLEW_PCT_PER_20MS
  0.0,  // SLEW_TURN_PER_20MS
  6.0,  // SLEW_PCT_PER_20MS_TURBO
  0.0,  // SLEW_TURN_PER_20MS_TURBO
  10,   // MIN_TURN_START
  4,    // PIVOT_FWD_DEADBAND
  100,  // INTAKE_FWD_PCT
  100,  // INTAKE_REV_PCT
  100,  // CONV_PCT
  false,// PISTON_DEFAULT
  false // LEFT_PREV
};

const AutonCfg autonCfg{
  80,   // DRIVE_FAST_PCT
  45,   // DRIVE_PRECISE_PCT
  70,   // TURN_FAST_PCT
  35,   // TURN_PRECISE_PCT

  24.0 * 25.4,  // TILE_MM
  20.0 * 25.4,  // TO_LOADER_MM
  36.0 * 25.4,  // TO_NEAR_LONG_MM
  10.0 * 25.4,  // ALONG_LONG_STEP_MM
  18.0 * 25.4,  // TO_CENTER_GOAL_MM

  1500, // FEED_TIME_MS
  400   // INTAKE_SPINUP_MS
};

// Helpers

int clamp_pct(int v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

int slew_step(int target, int current, int step) {
  if (step <= 0) return target;
  const int d = target - current;
  if (std::abs(d) <= step) {
    return target;
  }
  return current + (d > 0 ? step : -step);
}

double AnalogInputScaling(double x, double sens) {
  const double z = 127.0 * x;
  const double a = std::exp(-std::fabs(sens) / 10.0);
  const double b = std::exp((std::fabs(z) - 127.0) / 10.0);
  return (a + b * (1.0 - a)) * z / 127.0;
}

double shape_axis(double raw, double sens, double deadband) {
  if (std::fabs(raw) < deadband) {
    return 0.0;
  }
  return AnalogInputScaling(raw, sens);
}

int apply_slew(int target, int& state, int step) {
  if (step <= 0) {
    state = target;
    return target;
  }
  state = slew_step(target, state, step);
  return state;
}

double get_voltage() {
  return pros::battery::get_voltage() / 1000.0;
}

double voltage_comp() {
  constexpr double IDEAL_VOLTAGE = 12.6;
  const double v = get_voltage();
  if (v <= 0.0) return 1.0;

  double c = IDEAL_VOLTAGE / v;
  if (c > 1.20) c = 1.20;
  if (c < 0.90) c = 0.90;
  return c;
}
