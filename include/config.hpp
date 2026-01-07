#pragma once

/**
 * \file config.hpp
 *
 * \brief Central tuning and constants.
 *
 * \details This module exists to keep all tuning in one place:
 * - Driver limits, curves, deadbands, slew
 * - Auton high level speeds and distances
 * - Geometry and control constants for advanced models
 * - Small utility helpers used across modules
 *
 * \par Instructions
 * 1. Edit values in `config.cpp` only.
 * 2. Do not hardcode tuning numbers in other files.
 * 3. Start tuning in this order:
 *    - Deadbands
 *    - Driver max pct
 *    - Slew (accel feel)
 *    - Auton drive and turn pct
 *
 * \note If you add a new tuning value, add it to `Cfg` or `AutonCfg`,
 * then initialize it in `config.cpp`.
 */

// ============================================================================
//    _____             __ _
//   / ____|           / _(_)
//  | |     ___  _ __ | |_ _  __ _
//  | |    / _ \| '_ \|  _| |/ _` |
//  | |___| (_) | | | | | | | (_| |
//   \_____\___/|_| |_|_| |_|\__, |
//                            __/ |
//                           |___/
//
// ============================================================================

/**
 * \struct Cfg
 *
 * \brief Driver and subsystem tuning.
 *
 * \details Treat this as "global knobs" for the whole robot.
 * Anything touched often during tuning belongs here.
 */
struct Cfg {
  //> Driver normal speed caps
  int DRIVE_MAX_PCT;
  int TURN_MAX_PCT;

  //> Driver turbo caps
  int TURBO_DRIVE_MAX_PCT;
  int TURBO_TURN_MAX_PCT;

  // precision mode caps
  int PREC_DRIVE_MAX_PCT;
  int PREC_TURN_MAX_PCT;
  
  //> Auton base caps
  int AUTO_DRIVE_PCT;
  int AUTO_TURN_PCT;

  //> Deadbands for analog sticks (raw units mapped to your shaping)
  double DEADBAND_FWD;
  double DEADBAND_TURN;

  //> Sensitivity shaping parameters
  double SENSITIVITY_SOFT;
  double SENSITIVITY_TURBO;

  //> Slew limiting. Percent change per 20ms tick
  double SLEW_PCT_PER_20MS;
  double SLEW_TURN_PER_20MS;
  double SLEW_PCT_PER_20MS_TURBO;
  double SLEW_TURN_PER_20MS_TURBO;

  //> Pivot behavior
  int MIN_TURN_START;
  int PIVOT_FWD_DEADBAND;

  //> Subsystems power caps
  int INTAKE_FWD_PCT;
  int INTAKE_REV_PCT;
  int CONV_PCT;

  //> Initial states
  bool PISTON_DEFAULT;
  bool LEFT_PREV;
};

//> Global tuning object
extern const Cfg cfg;

// ============================================================================
//      _         _               _____  __
//     / \  _   _| |_ ___  _ __  / ____|/ _|
//    / _ \| | | | __/ _ \| '_ \| |    | |_
//   / ___ \ |_| | || (_) | | | | |____|  _|
//  /_/   \_\__,_|\__\___/|_| |_|\_____|_|
//
// ============================================================================

/**
 * \struct AutonCfg
 *
 * \brief High level autonomous parameters.
 *
 * \details Use this for coarse auton tuning. Distances are in millimeters.
 */
struct AutonCfg {
  //> Drive and turn presets for fast vs precise actions
  int DRIVE_FAST_PCT;
  int DRIVE_PRECISE_PCT;
  int TURN_FAST_PCT;
  int TURN_PRECISE_PCT;

  //> Field distances in mm (tiles and common travel segments)
  double TILE_MM;
  double TO_LOADER_MM;
  double TO_NEAR_LONG_MM;
  double ALONG_LONG_STEP_MM;
  double TO_CENTER_GOAL_MM;

  //> Timing constants in ms
  int FEED_TIME_MS;
  int INTAKE_SPINUP_MS;
};

//> Global auton config object
extern const AutonCfg autonCfg;

// ============================================================================
//    ____                _              _
//   / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___
//  | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
//  | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
//   \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/
//
// ============================================================================

//> Tracking wheel geometry (mm, m)
// 2.75 in * 25.4 = 69.85 mm
constexpr double WHEEL_DIAM_MM  = 69.85;
constexpr double WHEEL_CIRC_MM  = WHEEL_DIAM_MM * 3.1415926535;
constexpr double WHEEL_CIRC_M   = WHEEL_CIRC_MM / 1000.0;

//> Track width between drive wheel centers (mm)
constexpr double TRACK_WIDTH_MM = 320.0;

//> Odometry update period
constexpr int    ODOM_PERIOD_MS = 10;
constexpr double ODOM_PERIOD_S  = ODOM_PERIOD_MS / 1000.0;

//> Motion limits for advanced control models
constexpr double MAX_LINEAR_V_MPS  = 1.5;
constexpr double MAX_ANGULAR_V_RPS = 6.0;

//> Default PID constants (only used if a module chooses to)
constexpr double DRIVE_KP = 1.0;
constexpr double DRIVE_KI = 0.0;
constexpr double DRIVE_KD = 0.1;

//> Feedforward model: volts = kS + kV * v + kA * a
constexpr double FF_kS = 0.2;
constexpr double FF_kV = 2.0;
constexpr double FF_kA = 0.3;

//> Ramsete controller parameters
constexpr double RAMSETE_B    = 2.0;
constexpr double RAMSETE_ZETA = 0.7;

// ============================================================================
//    _   _      _
//   | | | | ___| |_ __   ___ _ __ ___
//   | |_| |/ _ \ | '_ \ / _ \ '__/ __|
//   |  _  |  __/ | |_) |  __/ |  \__ \
//   |_| |_|\___|_| .__/ \___|_|  |___/
//                |_|
//
// ============================================================================

/**
 * \brief Clamp a percent command to [-100, 100].
 *
 * \param v Input percent
 * \return Clamped percent
 */
int clamp_pct(int v);

/**
 * \brief Slew a value toward a target with a max step.
 *
 * \param target Desired value
 * \param current Current value
 * \param step Maximum delta allowed this call
 * \return New value after slew step
 */
int slew_step(int target, int current, int step);

/**
 * \brief Nonlinear scaling for analog input.
 *
 * \param x Input in controller units normalized to [-1, 1]
 * \param sens Sensitivity parameter
 * \return Scaled output in normalized units
 */
double AnalogInputScaling(double x, double sens);

/**
 * \brief Apply deadband and curve shaping to an axis.
 *
 * \param raw Raw input
 * \param sens Sensitivity parameter
 * \param deadband Deadband threshold
 * \return Shaped axis value
 */
double shape_axis(double raw, double sens, double deadband);

/**
 * \brief Apply slew to a target and update the stored state.
 *
 * \param target Desired value
 * \param state Persistent state to be updated
 * \param step Maximum delta allowed this call
 * \return New state value
 */
int apply_slew(int target, int& state, int step);

/**
 * \brief Get battery voltage in volts.
 */
double get_voltage();

/**
 * \brief Battery voltage compensation factor around an ideal voltage.
 *
 * \details Useful to maintain similar feel when battery sags.
 * Output clamped to a safe range.
 */
double voltage_comp();
