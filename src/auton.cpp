#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "motion.hpp"
#include "api.h"
#include "pros/motors.h"

/**
 * \file auton.cpp
 *
 * \brief Autonomous routines using only motion.cpp primitives.
 *
 * \par Core motion primitives used
 *   - drive_straight_mm (Rotation + IMU)
 *   - turn_imu_deg_2stage (IMU)
 *
 * \par Slowdown policy
 *   - Long segments use slow_down_mm in [200..260]
 *   - Short segments use slow_down_mm in [120..160]
 *
 * \par Stop feel policy
 *   - During segment. COAST
 *   - End of segment. drive_straight_mm does a brake pulse then returns to COAST
 *   - Extra brake_pulse is used only after turns, and at the end of auton
 *
 * \par Important requirement
 *   Your motion.hpp drive_straight_mm signature must include:
 *   (soft_settle_ms, brake_pulse_ms)
 */

 // ============================================================================
//   Global auton selector
// ============================================================================

AutonId g_auton_selected = AutonId::Right;

const char* auton_name(AutonId id) {
  switch (id) {
    case AutonId::Right:  return "RIGHT";
    case AutonId::Left:   return "LEFT";
  }
  return "UNKNOWN";
}

namespace {

  // ========================================================================
  //   Voltage compensation
  // ========================================================================

  constexpr double AUTON_IDEAL_VOLTAGE = 12.6;

  /**
   * \brief Read battery voltage in volts.
   */
  double auton_get_voltage() {
    return pros::battery::get_voltage() / 1000.0;
  }

  /**
   * \brief Compute voltage compensation multiplier.
   *
   * \returns Comp factor clamped to [0.90..1.20]
   */
  double auton_voltage_comp() {
    const double volt = auton_get_voltage();
    if (volt <= 0.0) return 1.0;

    double comp = AUTON_IDEAL_VOLTAGE / volt;
    if (comp > 1.20) comp = 1.20;
    if (comp < 0.90) comp = 0.90;
    return comp;
  }

  /**
   * \brief Clamp percent command to [-100..100].
   */
  int auton_clamp_pct(int v) {
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
  }

  /**
   * \brief Apply voltage comp to a percent command.
   */
  int autopct(double comp, int p) {
    return auton_clamp_pct(static_cast<int>(p * comp));
  }

  // ========================================================================
  //   Drive brake helpers
  // ========================================================================

  /**
   * \brief Set brake mode on all drive motors.
   */
  void set_drive_brake(pros::motor_brake_mode_e mode) {
    lf.set_brake_mode(mode);
    lm.set_brake_mode(mode);
    lb.set_brake_mode(mode);
    rf.set_brake_mode(mode);
    rm.set_brake_mode(mode);
    rb.set_brake_mode(mode);
  }

  /**
   * \brief Short BRAKE pulse to settle after a turn or at the end.
   */
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

  // Slowdowns
  constexpr double SLOW_LONG_MM  = 240.0;
  constexpr double SLOW_LONG2_MM = 260.0;
  constexpr double SLOW_MED_MM   = 220.0;
  constexpr double SLOW_SHORT_MM = 140.0;
  constexpr double SLOW_TINY_MM  = 130.0;

  // Motion settling passed into drive_straight_mm
  constexpr int SOFT_SETTLE_MS = 80;
  constexpr int BRAKE_PULSE_MS = 60;

} // namespace

// ============================================================================
//   Stage declarations
// ============================================================================

// RIGHT
static void auton_right_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
);

static void auton_right_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
);

static void auton_right_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
);

// LEFT
static void auton_left_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
);

static void auton_left_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
);

static void auton_left_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
);

// ============================================================================
//   Dispatcher
// ============================================================================

/**
 * \brief Main auton dispatcher.
 */
void autonomous_routine() {
  switch (g_auton_selected) {
    case AutonId::Right:
      auton_right();
      break;
    case AutonId::Left:
      auton_left();
      break;
    default:
      auton_right();
      break;
  }
}

// ============================================================================
//   AUTON RIGHT
// ============================================================================

/**
 * \brief Right side auton routine.
 */
void auton_right() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);

  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

  // Distances (inches -> mm)
  const double FWD1_MM   = 40.0 * 25.4;
  const double FWD2_MM   = 9.5  * 25.4;
  const double REV_1_MM  = 38.0 * 25.4;
  const double BACK_MM   = 17.0 * 25.4;
  const double FWD7_MM   = 8.5  * 25.4;

  const double FWD4_MM   = 7.0  * 25.4;
  const double FWD5_MM   = 15.0 * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  auton_right_stage_preload(
    comp, drivePct, turnFast, turnSlow,
    FWD1_MM, FWD2_MM, convPower, intakeFwd
  );

  auton_right_stage_platform(
    comp, drivePct, turnFast, turnSlow,
    REV_1_MM, BACK_MM, FWD7_MM, convPower
  );

  auton_right_stage_second_cycle(
    comp, drivePct, turnFast, turnSlow,
    FWD4_MM, FWD5_MM, convPower, intakeFwd
  );

  // Hold until 15s ends
  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  intake.move(0);
  conveyor.move(0);
  brake_pulse(60);
}

/**
 * \brief Stage 1. Leave start and score preload.
 */
static void auton_right_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  // Forward 40 in. Long segment.
  drive_straight_mm(
    FWD1_MM,
    drivePct,
    0.40,
    SLOW_LONG_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(50);

  // Turn 45 deg left.
  turn_imu_deg_2stage(
    -45.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  conveyor.move(0);
  pros::delay(60);

  // Forward 9.5 in. Short segment.
  drive_straight_mm(
    FWD2_MM,
    drivePct,
    0.30,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(70);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);
  pros::delay(80);

  (void)comp;
}

/**
 * \brief Stage 2. Go to platform and score with piston.
 */
static void auton_right_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
) {
  // Reverse 38 in. Long segment.
  drive_straight_mm(
    -REV_1_MM,
    drivePct,
    0.30,
    SLOW_LONG2_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(70);

  // Turn 90 deg left.
  turn_imu_deg_2stage(
    -90.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(70);
  pros::delay(60);

  piston_1.set_value(true);
  pros::delay(300);

  // Forward 8.5 in. Short segment.
  drive_straight_mm(
    FWD7_MM,
    autopct(comp, cfg.AUTO_DRIVE_PCT),
    0.40,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(60);

  // Reverse 17 in. Medium segment.
  drive_straight_mm(
    -BACK_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
}

/**
 * \brief Stage 3. Second cycle or cleanup.
 */
static void auton_right_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  conveyor.move(0);
  intake.move(0);
  pros::delay(60);

  // Forward 7 in. Tiny segment.
  drive_straight_mm(
    FWD4_MM,
    drivePct,
    0.30,
    SLOW_TINY_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(60);

  // Turn 70 deg left.
  turn_imu_deg_2stage(
    70.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  // Forward 15 in. Medium segment.
  drive_straight_mm(
    FWD5_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );

  (void)comp;
}

// ============================================================================
//   AUTON LEFT
// ============================================================================

/**
 * \brief Left side auton routine.
 */
void auton_left() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  set_drive_brake(pros::E_MOTOR_BRAKE_COAST);

  const int drivePct = autopct(comp, cfg.AUTO_DRIVE_PCT);

  const int turnFast = autopct(comp, cfg.AUTO_TURN_PCT);
  const int turnSlow = std::max(10, static_cast<int>(turnFast * 0.60));

  // Distances (inches -> mm)
  const double FWD1_MM   = 30.0 * 25.4;
  const double FWD2_MM   = 9.5  * 25.4;
  const double REV_1_MM  = 31.0 * 25.4;
  const double BACK_MM   = 17.0 * 25.4;
  const double FWD7_MM   = 8.5  * 25.4;

  const double FWD4_MM   = 7.0  * 25.4;
  const double FWD5_MM   = 15.0 * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  auton_left_stage_preload(
    comp, drivePct, turnFast, turnSlow,
    FWD1_MM, FWD2_MM, convPower, intakeFwd
  );

  auton_left_stage_platform(
    comp, drivePct, turnFast, turnSlow,
    REV_1_MM, BACK_MM, FWD7_MM, convPower
  );

  auton_left_stage_second_cycle(
    comp, drivePct, turnFast, turnSlow,
    FWD4_MM, FWD5_MM, convPower, intakeFwd
  );

  // Hold until 15s ends
  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) pros::delay(15000 - elapsed);

  intake.move(0);
  conveyor.move(0);
  brake_pulse(60);
}

/**
 * \brief Stage 1. Leave start and score preload.
 */
static void auton_left_stage_preload(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD1_MM,
  double FWD2_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  pros::delay(80);

  // Forward 30 in. Long segment.
  drive_straight_mm(
    FWD1_MM,
    drivePct,
    0.40,
    SLOW_LONG_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(50);

  // Turn 45 deg left.
  turn_imu_deg_2stage(
    45.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  conveyor.move(0);
  pros::delay(60);

  // Forward 9.5 in. Short segment.
  drive_straight_mm(
    FWD2_MM,
    drivePct,
    0.30,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(70);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);
  pros::delay(80);

  (void)comp;
}

/**
 * \brief Stage 2. Go to platform and score with piston.
 */
static void auton_left_stage_platform(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double REV_1_MM,
  double BACK_MM,
  double FWD7_MM,
  int convPower
) {
  // Reverse 31 in. Long segment.
  drive_straight_mm(
    -REV_1_MM,
    drivePct,
    0.30,
    SLOW_LONG2_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(70);

  // Turn 90 deg left.
  turn_imu_deg_2stage(
    90.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(70);
  pros::delay(60);

  piston_1.set_value(true);
  pros::delay(320);

  // Forward 8.5 in. Short segment.
  drive_straight_mm(
    FWD7_MM,
    autopct(comp, cfg.AUTO_DRIVE_PCT),
    0.40,
    SLOW_SHORT_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(50);

  conveyor.move(-convPower);
  pros::delay(2000);

  piston_1.set_value(false);
  pros::delay(120);

  // Reverse 17 in. Medium segment.
  drive_straight_mm(
    -BACK_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(60);

  (void)comp;
}

/**
 * \brief Stage 3. Second cycle or cleanup.
 */
static void auton_left_stage_second_cycle(
  double comp,
  int drivePct,
  int turnFast,
  int turnSlow,
  double FWD4_MM,
  double FWD5_MM,
  int convPower,
  int intakeFwd
) {
  conveyor.move(-convPower);
  intake.move(-intakeFwd);
  pros::delay(2500);

  conveyor.move(0);
  intake.move(0);
  pros::delay(60);

  // Forward 7 in. Tiny segment.
  drive_straight_mm(
    FWD4_MM,
    drivePct,
    0.30,
    SLOW_TINY_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );
  pros::delay(60);

  // Turn 70 deg left.
  turn_imu_deg_2stage(
    +70.0,
    turnFast,
    turnSlow,
    0.92,
    120
  );
  brake_pulse(60);
  pros::delay(70);

  // Forward 15 in. Medium segment.
  drive_straight_mm(
    FWD5_MM,
    drivePct,
    0.30,
    SLOW_MED_MM,
    pros::E_MOTOR_BRAKE_COAST,
    SOFT_SETTLE_MS,
    BRAKE_PULSE_MS
  );

  (void)comp;
}
