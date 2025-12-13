#include "config.hpp"
#include "api.h"

// ============================================================================
//   _______          _
//  |__   __|        (_)
//     | |_   _ _ __  _ _ __   __ _
//     | | | | | '_ \| | '_ \ / _` |
//     | | |_| | | | | | | | | (_| |
//     |_|\__,_|_| |_|_|_| |_|\__, |
//                             __/ |
//                            |___/
//
// ============================================================================

/**
 * \brief Global robot tuning.
 *
 * \details Edit these values during tuning sessions.
 * Keep them stable during competition, unless you are fixing a real issue.
 */
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

// ============================================================================
//    _         _               _____  __
//   / \  _   _| |_ ___  _ __  / ____|/ _|
//  / _ \| | | | __/ _ \| '_ \| |    | |_
// / ___ \ |_| | || (_) | | | | |____|  _|
//_/   \_\__,_|\__\___/|_| |_|\_____|_|
//
// ============================================================================

/**
 * \brief High level auton tuning.
 *
 * \details Distances are written as inches converted to mm.
 * Keep this consistent across all auton routines.
 */
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

// ============================================================================
//   _   _      _
//  | | | | ___| |_ __   ___ _ __ ___
//  | |_| |/ _ \ | '_ \ / _ \ '__/ __|
//  |  _  |  __/ | |_) |  __/ |  \__ \
//  |_| |_|\___|_| .__/ \___|_|  |___/
//               |_|
//
// ============================================================================

/**
 * \brief Clamp a percent command to [-100, 100].
 */
int clamp_pct(int v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

/**
 * \brief Slew a value toward a target with a max step.
 *
 * \note If `step <= 0`, the target is returned immediately.
 */
int slew_step(int target, int current, int step) {
  if (step <= 0) return target;

  const int d = target - current;
  if (std::abs(d) <= step) return target;

  return current + ((d > 0) ? step : -step);
}

/**
 * \brief Nonlinear scaling for analog input.
 *
 * \details This implements a curve where the mid-range is softened while
 * still allowing full output near stick extremes.
 */
double AnalogInputScaling(double x, double sens) {
  const double z = 127.0 * x;
  const double a = std::exp(-std::fabs(sens) / 10.0);
  const double b = std::exp((std::fabs(z) - 127.0) / 10.0);
  return (a + b * (1.0 - a)) * z / 127.0;
}

/**
 * \brief Apply deadband and curve shaping to an axis.
 */
double shape_axis(double raw, double sens, double deadband) {
  if (std::fabs(raw) < deadband) return 0.0;
  return AnalogInputScaling(raw, sens);
}

/**
 * \brief Apply slew to a target while updating the stored state.
 */
int apply_slew(int target, int& state, int step) {
  if (step <= 0) {
    state = target;
    return target;
  }
  state = slew_step(target, state, step);
  return state;
}

/**
 * \brief Get battery voltage in volts.
 */
double get_voltage() {
  return pros::battery::get_voltage() / 1000.0;
}

/**
 * \brief Battery voltage compensation factor around an ideal voltage.
 *
 * \details Output is clamped to avoid overdriving motors at low voltage.
 */
double voltage_comp() {
  constexpr double IDEAL_VOLTAGE = 12.6;

  const double v = get_voltage();
  if (v <= 0.0) return 1.0;

  double c = IDEAL_VOLTAGE / v;
  if (c > 1.20) c = 1.20;
  if (c < 0.90) c = 0.90;
  return c;
}
