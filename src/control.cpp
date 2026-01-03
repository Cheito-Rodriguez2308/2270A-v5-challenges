#include "control.hpp"
#include "api.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

/**
 * \file control.cpp
 *
 * \brief Driver control implementation (BASE = your 1st code)
 *
 * Changes imported from your 2nd code:
 *  - 3 drive modes (Normal/Precision/Turbo) on D-pad (UP / RIGHT / DOWN)
 *  - 3 piston toggles (Y / B / X)
 *  - Controller HUD + match countdown rumble
 *
 * Everything else follows your 1st code style/flow (shaping, slew, pivot helper, intake, conveyor, brain LCD).
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
 *   - mode: 0 normal, 1 precision, 2 turbo
 *   - p1/p2/p3: piston states persist across loop
 *   - slew values store ramped commands for forward and turn
 *   - prev button states implement rising edge toggles
 */
struct DriverState {
  // Modes: 0 normal, 1 precision, 2 turbo
  int mode;

  // Pistons
  bool p1_state; // piston_1
  bool p2_state; // piston_2
  bool p3_state; // piston_3

  int f_slew;
  int t_slew;

  int f_target;
  int t_target;

  int f_pct;
  int t_pct;

  // D-pad rising edge
  bool up_prev;
  bool right_prev;
  bool down_prev;

  // Piston rising edge
  bool y_prev;
  bool b_prev;
  bool x_prev;

  DriverState()
    : mode(0),
      p1_state(cfg.PISTON_DEFAULT),
      p2_state(false),
      p3_state(false),
      f_slew(0),
      t_slew(0),
      f_target(0),
      t_target(0),
      f_pct(0),
      t_pct(0),
      up_prev(false),
      right_prev(false),
      down_prev(false),
      y_prev(false),
      b_prev(false),
      x_prev(false) {}
};

namespace {

  /**
   * \brief Arcade mix to left and right tank commands.
   */
  void mix_arcade(int f_pct, int t_pct, int& left_pct, int& right_pct) {
    left_pct  = clamp_pct(f_pct + t_pct);
    right_pct = clamp_pct(f_pct - t_pct);
  }

  // ---------------- Controller HUD + Match Timer ----------------

  static constexpr double DRIVER_TIME_S   = 105.0; // driver period (change if you want)
  static constexpr int    HUD_REFRESH_MS  = 200;

  static int clamp_int(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
  }

  static const char* mode_tag(int mode) {
    if (mode == 2) return "[T]";
    if (mode == 1) return "[P]";
    return "[N]";
  }

  static void controller_hud_print(pros::Controller& ctl,
                                   int mode,
                                   int drive_limit,
                                   int turn_limit,
                                   int speed_pct,
                                   int turn_pct,
                                   double remaining_s,
                                   bool p1,
                                   bool p2,
                                   bool p3) {
    int bat = pros::battery::get_capacity();

    char timeLabel[16];
    if (remaining_s <= 30) std::snprintf(timeLabel, sizeof(timeLabel), "END:%02d", (int)remaining_s);
    else                   std::snprintf(timeLabel, sizeof(timeLabel), "%02d", (int)remaining_s);

    char line0[32];
    char line1[32];

    std::snprintf(line0, sizeof(line0), "%s S%02d B%02d %s",
                  mode_tag(mode),
                  clamp_int(speed_pct, 0, 99),
                  clamp_int(bat, 0, 99),
                  timeLabel);

    std::snprintf(line1, sizeof(line1), "D%02d T%02d P%d%d%d Tr%02d",
                  clamp_int(drive_limit, 0, 99),
                  clamp_int(turn_limit, 0, 99),
                  p1 ? 1 : 0,
                  p2 ? 1 : 0,
                  p3 ? 1 : 0,
                  clamp_int(turn_pct, 0, 99));

    ctl.print(0, 0, "%-15s", line0);
    ctl.print(1, 0, "%-15s", line1);
  }

  static void match_rumble(pros::Controller& ctl, double remaining_s) {
    static bool rumble_60_done = false;
    static bool rumble_30_done = false;
    static bool rumble_15_done = false;
    static double prev_remaining = DRIVER_TIME_S;

    static uint32_t last_pulse_ms = 0;

    auto crossed = [&](double thr) -> bool {
      return (prev_remaining > thr) && (remaining_s <= thr);
    };

    if (!rumble_60_done && crossed(60)) { ctl.rumble(".");  rumble_60_done = true; }
    if (!rumble_30_done && crossed(30)) { ctl.rumble(".."); rumble_30_done = true; }
    if (!rumble_15_done && crossed(15)) { ctl.rumble("..."); rumble_15_done = true; }

    // pulse between 10 and 5 seconds remaining
    if (remaining_s <= 10 && remaining_s >= 5) {
      uint32_t now = pros::millis();
      if (now - last_pulse_ms > 1000) {
        ctl.rumble("...");
        last_pulse_ms = now;
      }
    }

    prev_remaining = remaining_s;
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
  piston_2.set_value(false);
  piston_3.set_value(false);
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
  piston_1.set_value(s.p1_state);
  piston_2.set_value(s.p2_state);
  piston_3.set_value(s.p3_state);

  // Convert deadbands from percent integer to [-1, 1] scale.
  const double deadbandFwd  = cfg.DEADBAND_FWD  / 100.0;
  const double deadbandTurn = cfg.DEADBAND_TURN / 100.0;

  // Slew steps in percent per 20ms.
  const int slewDriveNormal = static_cast<int>(cfg.SLEW_PCT_PER_20MS);
  const int slewTurnNormal  = static_cast<int>(cfg.SLEW_TURN_PER_20MS);
  const int slewDriveTurbo  = static_cast<int>(cfg.SLEW_PCT_PER_20MS_TURBO);
  const int slewTurnTurbo   = static_cast<int>(cfg.SLEW_TURN_MAX_PCT_TURBO ? cfg.SLEW_TURN_PER_20MS_TURBO : cfg.SLEW_TURN_PER_20MS_TURBO);
  // ^ keep as turbo slew variable; if you don't have SLEW_TURN_MAX_PCT_TURBO ignore, it's harmless if removed.
  // If your cfg only has SLEW_TURN_PER_20MS_TURBO, you can simplify to:
  // const int slewTurnTurbo = static_cast<int>(cfg.SLEW_TURN_PER_20MS_TURBO);

  // Main loop period.
  constexpr int LOOP_MS = 20;

  // HUD caching to avoid constant LCD writes.
  int lastMode     = -1;
  int lastVoltage10 = -1;
  int lastComp100   = -1;

  pros::lcd::initialize();
  pros::lcd::set_text(0, "Mode: [N]");
  pros::lcd::set_text(1, "Ready");

  // Match timer + controller HUD gating
  const uint32_t match_start_ms = pros::millis();
  uint32_t last_hud_ms = 0;

  // Prevent HUD from instantly overwriting a short message
  uint32_t hud_pause_until_ms = 0;

  // small startup rumble
  master.rumble(".");

  while (true) {
    const uint32_t loop_start = pros::millis();

    // ------------------------------------------------------
    // Mode selection (UP=Normal, RIGHT=Precision toggle, DOWN=Turbo toggle)
    // ------------------------------------------------------
    const bool up_now    = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    const bool right_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    const bool down_now  = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

    if (up_now && !s.up_prev) {
      s.mode = 0; // normal
      master.rumble(".");
      hud_pause_until_ms = pros::millis() + 250;
    }
    if (right_now && !s.right_prev) {
      s.mode = (s.mode != 1) ? 1 : 0; // precision toggle
      master.rumble(s.mode == 1 ? ".." : ".");
      hud_pause_until_ms = pros::millis() + 250;
    }
    if (down_now && !s.down_prev) {
      s.mode = (s.mode != 2) ? 2 : 0; // turbo toggle
      master.rumble(s.mode == 2 ? "..." : ".");
      hud_pause_until_ms = pros::millis() + 250;
    }

    s.up_prev    = up_now;
    s.right_prev = right_now;
    s.down_prev  = down_now;

    // ------------------------------------------------------
    // Read sticks
    // ------------------------------------------------------
    const double f_raw = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)  / 127.0;
    const double t_raw = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

    // ------------------------------------------------------
    // Mode parameters (limits + sensitivity + slew)
    // ------------------------------------------------------
    int drive_limit = cfg.DRIVE_MAX_PCT;
    int turn_limit  = cfg.TURN_MAX_PCT;
    double sens     = cfg.SENSITIVITY_SOFT;
    int slewDrive   = slewDriveNormal;
    int slewTurn    = slewTurnNormal;

    if (s.mode == 2) { // turbo
      drive_limit = cfg.TURBO_DRIVE_MAX_PCT;
      turn_limit  = cfg.TURBO_TURN_MAX_PCT;
      sens        = cfg.SENSITIVITY_TURBO;
      slewDrive   = slewDriveTurbo;
      slewTurn    = slewTurnTurbo;
    } else if (s.mode == 1) { // precision
      drive_limit = cfg.PREC_DRIVE_MAX_PCT;
      turn_limit  = cfg.PREC_TURN_MAX_PCT;
      sens        = cfg.SENSITIVITY_SOFT;
      slewDrive   = slewDriveNormal;
      slewTurn    = slewTurnNormal;
    }

    // ------------------------------------------------------
    // Shape inputs
    // ------------------------------------------------------
    const double f_shaped = shape_axis(f_raw, sens, deadbandFwd);
    const double t_shaped = shape_axis(t_raw, sens, deadbandTurn);

    s.f_target = static_cast<int>(f_shaped * 100.0);
    s.t_target = static_cast<int>(t_shaped * 100.0);

    // ------------------------------------------------------
    // Slew and limits
    // ------------------------------------------------------
    s.f_slew = apply_slew(s.f_target, s.f_slew, slewDrive);
    s.t_slew = (slewTurn == 0) ? s.t_target : apply_slew(s.t_target, s.t_slew, slewTurn);

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
    // Pistons toggles (Y/B/X)
    // ------------------------------------------------------
    const bool y_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    if (y_now && !s.y_prev) {
      s.p1_state = !s.p1_state;
      piston_1.set_value(s.p1_state);
      master.rumble(".");
      master.clear_line(1);
      master.set_text(1, 0, s.p1_state ? "Piston1 ON" : "Piston1 OFF");
      hud_pause_until_ms = pros::millis() + 350;
    }
    s.y_prev = y_now;

    const bool b_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (b_now && !s.b_prev) {
      s.p2_state = !s.p2_state;
      piston_2.set_value(s.p2_state);
      master.rumble("..");
      master.clear_line(1);
      master.set_text(1, 0, s.p2_state ? "Piston2 ON" : "Piston2 OFF");
      hud_pause_until_ms = pros::millis() + 350;
    }
    s.b_prev = b_now;

    const bool x_now = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (x_now && !s.x_prev) {
      s.p3_state = !s.p3_state;
      piston_3.set_value(s.p3_state);
      master.rumble(s.p3_state ? "..." : ".");
      master.clear_line(1);
      master.set_text(1, 0, s.p3_state ? "Piston3 ON" : "Piston3 OFF");
      hud_pause_until_ms = pros::millis() + 350;
    }
    s.x_prev = x_now;

    // ------------------------------------------------------
    // Brain LCD HUD update (mode + voltage/comp)
    // ------------------------------------------------------
    const double v = get_voltage();
    const double c = voltage_comp();
    const int modeFlag   = s.mode; // 0/1/2
    const int voltage10  = static_cast<int>(v * 10.0);
    const int comp100    = static_cast<int>(c * 100.0);

    if (lastMode != modeFlag || voltage10 != lastVoltage10 || comp100 != lastComp100) {
      char buf0[32];
      char buf1[32];
      std::snprintf(buf0, sizeof(buf0), "Mode: %s", mode_tag(s.mode));
      std::snprintf(buf1, sizeof(buf1), "V=%.1f C=%.2f", v, c);
      pros::lcd::set_text(0, buf0);
      pros::lcd::set_text(1, buf1);
      lastMode      = modeFlag;
      lastVoltage10 = voltage10;
      lastComp100   = comp100;
    }

    // ------------------------------------------------------
    // Controller HUD + Match countdown rumble
    // ------------------------------------------------------
    const uint32_t now_ms = pros::millis();

    const double elapsed_s = (now_ms - match_start_ms) / 1000.0;
    double remaining_s = DRIVER_TIME_S - elapsed_s;
    if (remaining_s < 0) remaining_s = 0;

    match_rumble(master, remaining_s);

    if (now_ms >= hud_pause_until_ms && (now_ms - last_hud_ms) >= (uint32_t)HUD_REFRESH_MS) {
      const int speed_pct = std::abs(s.f_pct);
      const int turn_pct  = std::abs(s.t_pct);

      controller_hud_print(
        master,
        s.mode,
        drive_limit,
        turn_limit,
        speed_pct,
        turn_pct,
        remaining_s,
        s.p1_state,
        s.p2_state,
        s.p3_state
      );

      last_hud_ms = now_ms;
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
