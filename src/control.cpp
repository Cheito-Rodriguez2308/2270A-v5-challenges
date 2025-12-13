#include "control.hpp"
#include "api.h"

/**
 * \file control.cpp
 *
 * \brief Driver control implementation.
 *
 * \par Design goals
 *   - Predictable stick feel using shaping plus deadband
 *   - Optional turbo without changing code
 *   - Precision hold for alignment
 *   - Slew limits for smoother acceleration
 *   - Simple toggles for pistons and turbo
 *
 * \par Notes
 *   - drive.set_brake(COAST) gives a more human feel in teleop
 *   - voltage_comp() affects only HUD here. Auton uses its own compensation
 */

// ============================================================================
//   Driver state
// ============================================================================

/**
 * \struct DriverState
 *
 * \brief Stores controller and mechanism state across loop iterations.
 *
 * \details
 *   - turbo_on toggles drive limits and sensitivity
 *   - piston_state persists piston_1 toggle
 *   - slew values store ramped commands for forward and turn
 *   - prev button states implement rising edge toggles
 */
struct DriverState {
  bool turbo_on;
  bool piston_state;
  bool piston_state_2;

  int f_slew;
  int t_slew;

  int f_target;
  int t_target;

  int f_pct;
  int t_pct;

  bool a_prev;
  bool left_prev;
  bool right_prev;

  DriverState()
    : turbo_on(false),
      piston_state(cfg.PISTON_DEFAULT),
      piston_state_2(false),
      f_slew(0),
      t_slew(0),
      f_target(0),
      t_target(0),
      f_pct(0),
      t_pct(0),
      a_prev(false),
      left_prev(false),
      right_prev(false) {}
};

namespace {

  /**
   * \brief Arcade mix to left and right tank commands.
   *
   * \param f_pct Forward command in percent.
   * \param t_pct Turn command in percent.
   * \param left_pct Output left percent.
   * \param right_pct Output right percent.
   */
  void mix_arcade(int f_pct, int t_pct, int& left_pct, int& right_pct) {
    left_pct  = clamp_pct(f_pct + t_pct);
    right_pct = clamp_pct(f_pct - t_pct);
  }

} // namespace

// ============================================================================
//   Match pre setup
// ============================================================================

/**
 * \brief Set safe default states before match.
 */
void pre_auton() {
  piston_1.set_value(cfg.PISTON_DEFAULT);
}

// ============================================================================
//   Teleop loop
// ============================================================================

/**
 * \brief Driver control loop. Reads controller, shapes inputs, drives chassis, runs subsystems.
 *
 * \details
 *   Loop timing is fixed to LOOP_MS. Uses pros::millis timing.
 */
void driver_control_loop() {
  // COAST feels natural and prevents sudden lockups on release.
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);

  DriverState s;

  // Apply initial mechanism states.
  piston_1.set_value(s.piston_state);
  piston_2.set_value(s.piston_state_2);

  // Convert deadbands from percent integer to [-1, 1] scale.
  const double deadbandFwd  = cfg.DEADBAND_FWD  / 100.0;
  const double deadbandTurn = cfg.DEADBAND_TURN / 100.0;

  // Slew steps in percent per 20ms.
  const int slewDriveNormal = static_cast<int>(cfg.SLEW_PCT_PER_20MS);
  const int slewTurnNormal  = static_cast<int>(cfg.SLEW_TURN_PER_20MS);
  const int slewDriveTurbo  = static_cast<int>(cfg.SLEW_PCT_PER_20MS_TURBO);
  const int slewTurnTurbo   = static_cast<int>(cfg.SLEW_TURN_PER_20MS_TURBO);

  // Precision mode scaling.
  constexpr double PRECISION_DRIVE_SCALE = 0.40;
  constexpr double PRECISION_TURN_SCALE  = 0.45;

  // Main loop period.
  constexpr int LOOP_MS = 20;

  // HUD caching to avoid constant LCD writes.
  int lastTurbo     = -1;
  int lastVoltage10 = -1;
  int lastComp100   = -1;

  pros::lcd::initialize();
  pros::lcd::set_text(0, "Turbo OFF");
  pros::lcd::set_text(1, "Ready");

  while (true) {
    const uint32_t loop_start = pros::millis();

    // ------------------------------------------------------
    // Read sticks
    // ------------------------------------------------------
    const double f_raw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  / 127.0;
    const double t_raw = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    // Precision hold.
    const bool precision_on = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    // ------------------------------------------------------
    // Shape inputs
    // ------------------------------------------------------
    const double sens = s.turbo_on ? cfg.SENSITIVITY_TURBO : cfg.SENSITIVITY_SOFT;

    const double f_shaped = shape_axis(f_raw, sens, deadbandFwd);
    const double t_shaped = shape_axis(t_raw, sens, deadbandTurn);

    s.f_target = static_cast<int>(f_shaped * 100.0);
    s.t_target = static_cast<int>(t_shaped * 100.0);

    // ------------------------------------------------------
    // Slew and limits
    // ------------------------------------------------------
    int drive_limit;
    int turn_limit;

    if (s.turbo_on) {
      s.f_slew = apply_slew(s.f_target, s.f_slew, slewDriveTurbo);
      s.t_slew = (slewTurnTurbo == 0) ? s.t_target : apply_slew(s.t_target, s.t_slew, slewTurnTurbo);

      drive_limit = cfg.TURBO_DRIVE_MAX_PCT;
      turn_limit  = cfg.TURBO_TURN_MAX_PCT;
    } else {
      s.f_slew = apply_slew(s.f_target, s.f_slew, slewDriveNormal);
      s.t_slew = (slewTurnNormal == 0) ? s.t_target : apply_slew(s.t_target, s.t_slew, slewTurnNormal);

      drive_limit = cfg.DRIVE_MAX_PCT;
      turn_limit  = cfg.TURN_MAX_PCT;
    }

    if (precision_on) {
      drive_limit = static_cast<int>(drive_limit * PRECISION_DRIVE_SCALE);
      turn_limit  = static_cast<int>(turn_limit  * PRECISION_TURN_SCALE);

      if (drive_limit < 10) drive_limit = 10;
      if (turn_limit  < 10) turn_limit  = 10;
    }

    s.f_pct = static_cast<int>(s.f_slew * drive_limit / 100);
    s.t_pct = static_cast<int>(s.t_slew * turn_limit  / 100);

    // ------------------------------------------------------
    // Drive output
    // ------------------------------------------------------
    int left_pct;
    int right_pct;
    mix_arcade(s.f_pct, s.t_pct, left_pct, right_pct);

    // Pivot helper for easier in place turns.
    if (std::abs(s.f_pct) <= cfg.PIVOT_FWD_DEADBAND && std::abs(s.t_pct) > 0) {
      const int mag = std::max(std::abs(s.t_pct), cfg.MIN_TURN_START);
      const int left_p  = s.t_pct > 0 ?  mag : -mag;
      const int right_p = s.t_pct > 0 ? -mag :  mag;
      drive.set_percent(left_p, right_p);
    } else {
      drive.set_percent(left_pct, right_pct);
    }

    // ------------------------------------------------------
    // Intake control
    // ------------------------------------------------------
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(cfg.INTAKE_FWD_PCT * 127 / 100);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(-cfg.INTAKE_REV_PCT * 127 / 100);
    } else {
      intake.move(0);
      intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    // ------------------------------------------------------
    // Conveyor control
    // ------------------------------------------------------
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      conveyor.move(cfg.CONV_PCT * 127 / 100);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      conveyor.move(-cfg.CONV_PCT * 127 / 100);
    } else {
      conveyor.move(0);
      conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    // ------------------------------------------------------
    // Piston 1 toggle on A
    // ------------------------------------------------------
    const bool a_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    if (a_now && !s.a_prev) {
      s.piston_state = !s.piston_state;
      piston_1.set_value(s.piston_state);

      master.rumble(".");
      master.clear_line(1);
      master.set_text(1, 0, s.piston_state ? "Piston 1 ON" : "Piston 1 OFF");
    }
    s.a_prev = a_now;

    // ------------------------------------------------------
    // Piston 2 hold on LEFT
    // ------------------------------------------------------
    const bool left_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    piston_2.set_value(left_now);
    s.left_prev = left_now;

    // ------------------------------------------------------
    // Turbo toggle on RIGHT
    // ------------------------------------------------------
    const bool right_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (right_now && !s.right_prev) {
      s.turbo_on = !s.turbo_on;

      master.rumble(s.turbo_on ? "." : "..");
      master.clear_line(0);
      master.set_text(0, 0, s.turbo_on ? "Turbo ON " : "Turbo OFF");
    }
    s.right_prev = right_now;

    // ------------------------------------------------------
    // HUD update
    // ------------------------------------------------------
    const double v = get_voltage();
    const double c = voltage_comp();
    const int turboFlag  = s.turbo_on ? 1 : 0;
    const int voltage10  = static_cast<int>(v * 10.0);
    const int comp100    = static_cast<int>(c * 100.0);

    if (lastTurbo != turboFlag || voltage10 != lastVoltage10 || comp100 != lastComp100) {
      char buf0[32];
      char buf1[32];
      std::snprintf(buf0, sizeof(buf0), "Turbo: %s", s.turbo_on ? "ON" : "OFF");
      std::snprintf(buf1, sizeof(buf1), "V=%.1f C=%.2f", v, c);
      pros::lcd::set_text(0, buf0);
      pros::lcd::set_text(1, buf1);
      lastTurbo     = turboFlag;
      lastVoltage10 = voltage10;
      lastComp100   = comp100;
    }

    // ------------------------------------------------------
    // Loop timing
    // ------------------------------------------------------
    const uint32_t elapsed = pros::millis() - loop_start;
    if (elapsed < static_cast<uint32_t>(LOOP_MS)) {
      pros::delay(LOOP_MS - elapsed);
    }
  }
}
