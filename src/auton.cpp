#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "api.h"
#include "pros/motors.h"

#include <algorithm>
#include <cmath>

/**
 * \file auton.cpp
 *
 * \brief Autonomous routines using only motion.cpp primitives.
 *
 * Core motion primitives used
 *   - drive_straight_mm (Rotation + IMU)
 *   - turn_imu_deg_2stage (IMU)
 *
 * Slowdown policy
 *   - Long segments use slow_down_mm in [200..260]
 *   - Short segments use slow_down_mm in [120..160]
 *
 * Stop feel policy
 *   - During segment. COAST
 *   - End of segment. drive_straight_mm does a brake pulse then returns to COAST
 *   - Extra brake_pulse is used only after turns, and at the end of auton
 *
 * Important requirement
 *   Your motion.hpp drive_straight_mm signature must include:
 *   (soft_settle_ms, brake_pulse_ms)
 *
 * ============================================================================
 *  Skills 60 route (your request)
 * ============================================================================
 *
 * Start: LEFT side of blocks
 * Path:
 *   1) Lower Center Goal
 *   2) Left Loader
 *   3) Left Long Goal
 *   4) Right Loader
 *   5) Right Long Goal
 *   6) Upper Center Goal
 *
 * NOTE
 *   Distances below are starter numbers. You must tune per field and robot.
 */

// ============================================================================
//   Field ASCII (driver mental map)
// ============================================================================
//
//   (Top)
//   +----------------------------------------------------------+
//   |                   [  UPPER CENTER GOAL  ]                |
//   |                                                          |
//   |   [LEFT LONG]                              [RIGHT LONG]  |
//   |                                                          |
//   |   [LEFT LOADER]                          [RIGHT LOADER]  |
//   |                                                          |
//   |                   [  LOWER CENTER GOAL  ]                |
//   |                                                          |
//   |  [ LEFT BLOCKS START ]                      [ RIGHT START] |
//   +----------------------------------------------------------+
//   (Bottom)
//
// Conventions used in this auton
//   - Heading is IMU heading at each segment start (your drive_straight_mm does that inside).
//   - We turn, brake_pulse, then drive.
//   - Conveyor directions follow your existing code:
//       +convPower  = feed/intake in
//       -convPower  = score out

// ============================================================================
//   Global auton selector
// ============================================================================

AutonId g_auton_selected = AutonId::Right;

const char* auton_name(AutonId id) {
  switch (id) {
    case AutonId::Right:      return "RIGHT";
    case AutonId::Left:       return "LEFT";
    case AutonId::Skills60:   return "SKILLS_60";
  }
  return "UNKNOWN";
}

namespace {

  // ========================================================================
  //   Voltage compensation
  // ========================================================================

  constexpr double AUTON_IDEAL_VOLTAGE = 12.6;

  double auton_get_voltage() {
    return pros::battery::get_voltage() / 1000.0;
  }

  double auton_voltage_comp() {
    const double volt = auton_get_voltage();
    if (volt <= 0.0) return 1.0;

    double comp = AUTON_IDEAL_VOLTAGE / volt;
    if (comp > 1.20) comp = 1.20;
    if (comp < 0.90) comp = 0.90;
    return comp;
  }

  int auton_clamp_pct(int v) {
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
  }

  int autopct(double comp, int p) {
    return auton_clamp_pct(static_cast<int>(p * comp));
  }

  // ========================================================================
  //   Drive brake helpers
  // ========================================================================

  void set_drive_brake(pros::motor_brake_mode_e mode) {
    lf.set_brake_mode(mode);
    lm.set_brake_mode(mode);
    lb.set_brake_mode(mode);
    rf.set_brake_mode(mode);
    rm.set_brake_mode(mode);
    rb.set_brake_mode(mode);
  }

  void brake_pulse(int ms = 70) {
    set_drive_brake(pros::E_MOTOR_BRAKE_BRAKE);
    lf.brake(); lm.brake(); lb.brake();
    rf.brake(); rm.brake(); rb.brake();
    pros::delay(ms);
    set_drive_brake(pros::E_MOTOR_BRAKE_COAST);
  }

  // ========================================================================
  //   Standard tuning constants
  // ========================================================================

  constexpr double SLOW_LONG_MM  = 240.0;
  constexpr double SLOW_LONG2_MM = 260.0;
  constexpr double SLOW_MED_MM   = 220.0;
  constexpr double SLOW_SHORT_MM = 140.0;
  constexpr double SLOW_TINY_MM  = 130.0;

  constexpr int SOFT_SETTLE_MS = 80;
  constexpr int BRAKE_PULSE_MS = 60;

  // ========================================================================
  //   Skills timing
  // ========================================================================

  constexpr uint32_t SKILLS_MS = 60000;
  constexpr uint32_t END_RESERVE_MS = 250;

  bool time_ok(uint32_t t0) {
    const uint32_t used = pros::millis() - t0;
    return (used + END_RESERVE_MS) < SKILLS_MS;
  }

  // ========================================================================
  //   Mechanism helpers
  // ========================================================================

  void stop_mechs() {
    intake.move(0);
    conveyor.move(0);
  }

  void score_out(int convPower, int ms) {
    conveyor.move(-convPower);
    pros::delay(ms);
    conveyor.move(0);
  }

  void load_in(int convPower, int intakeFwd, int ms) {
    conveyor.move(+convPower);
    intake.move(intakeFwd);
    pros::delay(ms);
    conveyor.move(0);
    intake.move(0);
  }

  // ========================================================================
  //   "Worlds feel" wrappers (inspired by your friend)
  //   Still uses YOUR existing motion.cpp primitives.
  //
  //   What we mimic:
  //     - Consistent segment kP by length
  //     - Consistent delays and brake pulses
  //     - Conservative slowDown usage to reduce overshoot
  // ========================================================================

  void drive_seg(double mm, int drivePct, double kP, double slowDownMm) {
    drive_straight_mm(
      mm,
      drivePct,
      kP,
      slowDownMm,
      pros::E_MOTOR_BRAKE_COAST,
      SOFT_SETTLE_MS,
      BRAKE_PULSE_MS
    );
    pros::delay(40);
  }

  void drive_long(double mm, int drivePct)  { drive_seg(mm, drivePct, 0.40, SLOW_LONG2_MM); }
  void drive_med(double mm, int drivePct)   { drive_seg(mm, drivePct, 0.35, SLOW_MED_MM); }
  void drive_short(double mm, int drivePct) { drive_seg(mm, drivePct, 0.30, SLOW_SHORT_MM); }
  void drive_tiny(double mm, int drivePct)  { drive_seg(mm, drivePct, 0.30, SLOW_TINY_MM); }

  void turn_smooth(double deg, int turnFast, int turnSlow) {
    turn_imu_deg_2stage(
      deg,
      turnFast,
      turnSlow,
      0.92,
      120
    );
    brake_pulse(55);
    pros::delay(45);
  }

} // namespace

// ============================================================================
//   Stage declarations
// ============================================================================

// Existing 15s
static void auton_right_stage_preload(double comp,int drivePct,int turnFast,int turnSlow,double FWD1_MM,double FWD2_MM,int convPower,int intakeFwd);
static void auton_right_stage_platform(double comp,int drivePct,int turnFast,int turnSlow,double REV_1_MM,double BACK_MM,double FWD7_MM,int convPower);
static void auton_right_stage_second_cycle(double comp,int drivePct,int turnFast,int turnSlow,double FWD4_MM,double FWD5_MM,int convPower,int intakeFwd);

static void auton_left_stage_preload(double comp,int drivePct,int turnFast,int turnSlow,double FWD1_MM,double FWD2_MM,int convPower,int intakeFwd);
static void auton_left_stage_platform(double comp,int drivePct,int turnFast,int turnSlow,double REV_1_MM,double BACK_MM,double FWD7_MM,int convPower);
static void auton_left_stage_second_cycle(double comp,int drivePct,int turnFast,int turnSlow,double FWD4_MM,double FWD5_MM,int convPower,int intakeFwd);

// Skills 60
static void skills_stage_1_lower_center(uint32_t t0,int drivePct,int turnFast,int turnSlow,int convPower);
static void skills_stage_2_left_loader(uint32_t t0,int drivePct,int turnFast,int turnSlow,int convPower,int intakeFwd);
static void skills_stage_3_left_long(uint32_t t0,int drivePct,int turnFast,int turnSlow,int convPower);
static void skills_stage_4_right_loader(uint32_t t0,int drivePct,int turnFast,int turnSlow,int convPower,int intakeFwd);
static void skills_stage_5_right_long(uint32_t t0,int drivePct,int turnFast,int turnSlow,int convPower);
static void skills_stage_6_upper_center(uint32_t t0,int drivePct,int turnFast,int turnSlow,int convPower);

// ============================================================================
//   Dispatcher
// ============================================================================

void autonomous_routine() {
  switch (g_auton_selected) {
    case AutonId::Right:    auton_right(); break;
    case AutonId::Left:     auton_left();  break;
    case AutonId::Skills60: auton_skills_60_left(); break;
    default:                auton_left();  break;
  }
}

// ============================================================================
//   AUTON SKILLS 60
// ============================================================================

void auton_skills_60_left() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);
  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  // Brief: start with mechanisms safe
  stop_mechs();
  pros::delay(40);

  skills_stage_1_lower_center(t0, drivePct, turnFast, turnSlow, convPower);
  skills_stage_2_left_loader (t0, drivePct, turnFast, turnSlow, convPower, intakeFwd);
  skills_stage_3_left_long   (t0, drivePct, turnFast, turnSlow, convPower);
  skills_stage_4_right_loader(t0, drivePct, turnFast, turnSlow, convPower, intakeFwd);
  skills_stage_5_right_long  (t0, drivePct, turnFast, turnSlow, convPower);
  skills_stage_6_upper_center(t0, drivePct, turnFast, turnSlow, convPower);

  // Hold to 60s end
  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < SKILLS_MS) pros::delay(SKILLS_MS - elapsed);

  stop_mechs();
  brake_pulse(70);
}

// ============================================================================
//   SKILLS STAGES (BRIEFS + STARTER NUMBERS)
// ============================================================================

static void skills_stage_1_lower_center(
  uint32_t t0,
  int drivePct,
  int turnFast,
  int turnSlow,
  int convPower
) {
  if (!time_ok(t0)) return;

  /**
   * Brief
   *   From left blocks, go to lower center goal and dump.
   * Notes
   *   - Add a small bias turn if you hit the wall.
   */

  const double BIAS_TURN_DEG          = +10.0;
  const double FWD_TO_LOWER_CENTER_MM = 44.0 * 25.4;

  turn_smooth(BIAS_TURN_DEG, turnFast, turnSlow);
  if (!time_ok(t0)) return;

  drive_long(FWD_TO_LOWER_CENTER_MM, drivePct);
  if (!time_ok(t0)) return;

  score_out(convPower, 650);
  pros::delay(60);
}

static void skills_stage_2_left_loader(
  uint32_t t0,
  int drivePct,
  int turnFast,
  int turnSlow,
  int convPower,
  int intakeFwd
) {
  if (!time_ok(t0)) return;

  /**
   * Brief
   *   Leave lower center and intake from left loader.
   */

  const double BACK_OFF_MM            = 10.0 * 25.4;
  const double DRIVE_TO_LEFT_LANE_MM  = 24.0 * 25.4;
  const double TURN_TO_LEFT_LOADER_DEG = -90.0;
  const double APPROACH_LOADER_MM     = 9.0  * 25.4;
  const int    LOAD_MS                = 950;
  const double LEAVE_MM               = 6.0  * 25.4;

  drive_short(-BACK_OFF_MM, drivePct);
  if (!time_ok(t0)) return;

  drive_med(DRIVE_TO_LEFT_LANE_MM, drivePct);
  if (!time_ok(t0)) return;

  turn_smooth(TURN_TO_LEFT_LOADER_DEG, turnFast, turnSlow);
  if (!time_ok(t0)) return;

  drive_tiny(APPROACH_LOADER_MM, drivePct);
  if (!time_ok(t0)) return;

  load_in(convPower, intakeFwd, LOAD_MS);
  if (!time_ok(t0)) return;

  drive_tiny(-LEAVE_MM, drivePct);
  pros::delay(40);
}

static void skills_stage_3_left_long(
  uint32_t t0,
  int drivePct,
  int turnFast,
  int turnSlow,
  int convPower
) {
  if (!time_ok(t0)) return;

  /**
   * Brief
   *   Go from left loader to left long goal and dump.
   */

  const double DRIVE_TO_LEFT_LONG_MM  = 34.0 * 25.4;
  const double TURN_TO_LONG_DEG       = +90.0;
  const double APPROACH_LONG_MM       = 12.0 * 25.4;
  const int    SCORE_MS               = 850;

  drive_long(DRIVE_TO_LEFT_LONG_MM, drivePct);
  if (!time_ok(t0)) return;

  turn_smooth(TURN_TO_LONG_DEG, turnFast, turnSlow);
  if (!time_ok(t0)) return;

  drive_short(APPROACH_LONG_MM, drivePct);
  if (!time_ok(t0)) return;

  score_out(convPower, SCORE_MS);
  pros::delay(60);
}

static void skills_stage_4_right_loader(
  uint32_t t0,
  int drivePct,
  int turnFast,
  int turnSlow,
  int convPower,
  int intakeFwd
) {
  if (!time_ok(t0)) return;

  /**
   * Brief
   *   Cross field to right loader and intake.
   */

  const double BACK_OFF_MM      = 10.0 * 25.4;
  const double TURN_TO_CROSS_DEG = +90.0;
  const double CROSS_FIELD_MM   = 78.0 * 25.4;
  const double APPROACH_MM      = 9.0  * 25.4;
  const int    LOAD_MS          = 950;
  const double LEAVE_MM         = 6.0  * 25.4;

  drive_short(-BACK_OFF_MM, drivePct);
  if (!time_ok(t0)) return;

  turn_smooth(TURN_TO_CROSS_DEG, turnFast, turnSlow);
  if (!time_ok(t0)) return;

  drive_long(CROSS_FIELD_MM, drivePct);
  if (!time_ok(t0)) return;

  drive_tiny(APPROACH_MM, drivePct);
  if (!time_ok(t0)) return;

  load_in(convPower, intakeFwd, LOAD_MS);
  if (!time_ok(t0)) return;

  drive_tiny(-LEAVE_MM, drivePct);
  pros::delay(40);
}

static void skills_stage_5_right_long(
  uint32_t t0,
  int drivePct,
  int turnFast,
  int turnSlow,
  int convPower
) {
  if (!time_ok(t0)) return;

  /**
   * Brief
   *   Go from right loader to right long goal and dump.
   */

  const double DRIVE_TO_RIGHT_LONG_MM = 34.0 * 25.4;
  const double TURN_TO_LONG_DEG       = -90.0;
  const double APPROACH_LONG_MM       = 12.0 * 25.4;
  const int    SCORE_MS               = 850;

  drive_long(DRIVE_TO_RIGHT_LONG_MM, drivePct);
  if (!time_ok(t0)) return;

  turn_smooth(TURN_TO_LONG_DEG, turnFast, turnSlow);
  if (!time_ok(t0)) return;

  drive_short(APPROACH_LONG_MM, drivePct);
  if (!time_ok(t0)) return;

  score_out(convPower, SCORE_MS);
  pros::delay(60);
}

static void skills_stage_6_upper_center(
  uint32_t t0,
  int drivePct,
  int turnFast,
  int turnSlow,
  int convPower
) {
  if (!time_ok(t0)) return;

  /**
   * Brief
   *   Go to upper center goal and dump last.
   */

  const double BACK_OFF_MM             = 10.0 * 25.4;
  const double DRIVE_TO_UPPER_CENTER_MM = 40.0 * 25.4;
  const double TURN_TO_CENTER_DEG      = -90.0;
  const double APPROACH_CENTER_MM      = 14.0 * 25.4;
  const int    SCORE_MS                = 900;

  drive_short(-BACK_OFF_MM, drivePct);
  if (!time_ok(t0)) return;

  drive_long(DRIVE_TO_UPPER_CENTER_MM, drivePct);
  if (!time_ok(t0)) return;

  turn_smooth(TURN_TO_CENTER_DEG, turnFast, turnSlow);
  if (!time_ok(t0)) return;

  drive_short(APPROACH_CENTER_MM, drivePct);
  if (!time_ok(t0)) return;

  score_out(convPower, SCORE_MS);
  pros::delay(60);
}

// ============================================================================
//   AUTON RIGHT (UNCHANGED FROM YOUR TEMPLATE)
// ============================================================================

void auton_right() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);

  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

  const double FWD1_MM   = 40.0 * 25.4;
  const double FWD2_MM   = 9.5  * 25.4;
  const double REV_1_MM  = 38.0 * 25.4;
  const double BACK_MM   = 17.0 * 25.4;
  const double FWD7_MM   = 8.5  * 25.4;

  const double FWD4_MM   = 7.0  * 25.4;
  const double FWD5_MM   = 15.0 * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  auton_right_stage_preload(comp, drivePct, turnFast, turnSlow, FWD1_MM, FWD2_MM, convPower, intakeFwd);
  auton_right_stage_platform(comp, drivePct, turnFast, turnSlow, REV_1_MM, BACK_MM, FWD7_MM, convPower);
  auton_right_stage_second_cycle(comp, drivePct, turnFast, turnSlow, FWD4_MM, FWD5_MM, convPower, intakeFwd);

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  stop_mechs();
  brake_pulse(60);
}

static void auton_right_stage_preload(
  double comp,int drivePct,int turnFast,int turnSlow,double FWD1_MM,double FWD2_MM,int convPower,int intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  drive_straight_mm(FWD1_MM, drivePct, 0.40, SLOW_LONG_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(50);

  turn_imu_deg_2stage(+45.0, turnFast, turnSlow, 0.92, 120);
  brake_pulse(60);
  pros::delay(70);

  conveyor.move(0);
  pros::delay(60);

  drive_straight_mm(FWD2_MM, drivePct, 0.30, SLOW_SHORT_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(70);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  stop_mechs();
  pros::delay(80);

  (void)comp;
}

static void auton_right_stage_platform(
  double comp,int drivePct,int turnFast,int turnSlow,double REV_1_MM,double BACK_MM,double FWD7_MM,int convPower
) {
  drive_straight_mm(-REV_1_MM, drivePct, 0.30, SLOW_LONG2_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(70);

  turn_imu_deg_2stage(+90.0, turnFast, turnSlow, 0.92, 120);
  brake_pulse(70);
  pros::delay(60);

  piston_1.set_value(true);
  pros::delay(300);

  drive_straight_mm(FWD7_MM, autopct(comp, cfg.AUTO_DRIVE_PCT), 0.40, SLOW_SHORT_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(60);

  drive_straight_mm(-BACK_MM, drivePct, 0.30, SLOW_MED_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
}

static void auton_right_stage_second_cycle(
  double comp,int drivePct,int turnFast,int turnSlow,double FWD4_MM,double FWD5_MM,int convPower,int intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  stop_mechs();
  pros::delay(60);

  drive_straight_mm(FWD4_MM, drivePct, 0.30, SLOW_TINY_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(60);

  turn_imu_deg_2stage(-70.0, turnFast, turnSlow, 0.92, 120);
  brake_pulse(60);
  pros::delay(70);

  drive_straight_mm(FWD5_MM, drivePct, 0.30, SLOW_MED_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);

  (void)comp;
}

// ============================================================================
//   AUTON LEFT (UNCHANGED FROM YOUR TEMPLATE)
// ============================================================================

void auton_left() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);

  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

  const double FWD1_MM   = 20.0 * 25.4;
  const double FWD2_MM   = 9.5  * 25.4;
  const double REV_1_MM  = 31.0 * 25.4;
  const double BACK_MM   = 17.0 * 25.4;
  const double FWD7_MM   = 8.5  * 25.4;

  const double FWD4_MM   = 7.0  * 25.4;
  const double FWD5_MM   = 15.0 * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  auton_left_stage_preload(comp, drivePct, turnFast, turnSlow, FWD1_MM, FWD2_MM, convPower, intakeFwd);
  auton_left_stage_platform(comp, drivePct, turnFast, turnSlow, REV_1_MM, BACK_MM, FWD7_MM, convPower);
  auton_left_stage_second_cycle(comp, drivePct, turnFast, turnSlow, FWD4_MM, FWD5_MM, convPower, intakeFwd);

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  stop_mechs();
  brake_pulse(60);
}

static void auton_left_stage_preload(
  double comp,int drivePct,int turnFast,int turnSlow,double FWD1_MM,double FWD2_MM,int convPower,int intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  drive_straight_mm(FWD1_MM, drivePct, 0.40, SLOW_LONG_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(50);

  turn_imu_deg_2stage(-50.0, turnFast, turnSlow, 0.92, 120);
  brake_pulse(60);
  pros::delay(70);

  conveyor.move(0);
  pros::delay(60);

  drive_straight_mm(FWD2_MM, drivePct, 0.30, SLOW_SHORT_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(70);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  stop_mechs();
  pros::delay(80);

  (void)comp;
}

static void auton_left_stage_platform(
  double comp,int drivePct,int turnFast,int turnSlow,double REV_1_MM,double BACK_MM,double FWD7_MM,int convPower
) {
  drive_straight_mm(-REV_1_MM, drivePct, 0.30, SLOW_LONG2_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(70);

  turn_imu_deg_2stage(-90.0, turnFast, turnSlow, 0.92, 120);
  brake_pulse(70);
  pros::delay(60);

  piston_1.set_value(true);
  pros::delay(320);

  drive_straight_mm(FWD7_MM, autopct(comp, cfg.AUTO_DRIVE_PCT), 0.40, SLOW_SHORT_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(50);

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(120);

  drive_straight_mm(-BACK_MM, drivePct, 0.30, SLOW_MED_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(60);

  (void)comp;
}

static void auton_left_stage_second_cycle(
  double comp,int drivePct,int turnFast,int turnSlow,double FWD4_MM,double FWD5_MM,int convPower,int intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  stop_mechs();
  pros::delay(60);

  drive_straight_mm(FWD4_MM, drivePct, 0.30, SLOW_TINY_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);
  pros::delay(60);

  turn_imu_deg_2stage(+70.0, turnFast, turnSlow, 0.92, 120);
  brake_pulse(60);
  pros::delay(70);

  drive_straight_mm(FWD5_MM, drivePct, 0.30, SLOW_MED_MM, pros::E_MOTOR_BRAKE_COAST, SOFT_SETTLE_MS, BRAKE_PULSE_MS);

  (void)comp;
}