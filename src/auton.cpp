#include "auton.hpp"
#include "config.hpp"
#include "devices.hpp"
#include "drive.hpp"
#include "motion.hpp"
#include "odom.hpp"
#include "pros/apix.h"

AutonId g_auton_selected = AutonId::Right;

const char* auton_name(AutonId id) {
  switch (id) {
    case AutonId::Right:  return "RIGHT";
    case AutonId::Left:   return "LEFT";
  }
  return "UNKNOWN";
}

namespace {

  int auton_clamp_pct(int v) {
    if (v > 100) return 100;
    if (v < -100) return -100;
    return v;
  }

  constexpr double AUTON_IDEAL_VOLTAGE = 12.6;
  constexpr double RAD2DEG = 180.0 / 3.1415926535;

  double auton_get_voltage() {
    return pros::battery::get_voltage() / 1000.0;
  }

  double auton_voltage_comp() {
    double volt = auton_get_voltage();
    if (volt <= 0.0) return 1.0;
    double comp = AUTON_IDEAL_VOLTAGE / volt;
    if (comp > 1.20) comp = 1.20;
    if (comp < 0.90) comp = 0.90;
    return comp;
  }
}

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

void auton_right() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  drive.set_brake(pros::E_MOTOR_BRAKE_HOLD);

  auto autopct = [comp](int p) {
    return auton_clamp_pct(static_cast<int>(p * comp));
  };

  const int drivePct = autopct(cfg.AUTO_DRIVE_PCT);

  const double FWD1_MM = 20.0 * 25.4;
  const double FWD2_MM = 9.5  * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  conveyor.move(-convPower);
  pros::delay(80);

  drive_mm_pid(FWD1_MM);
  pros::delay(60);

  turn_imu_deg(+50.0);
  pros::delay(100);

  conveyor.move(0);
  pros::delay(60);

  drive_mm_pid(FWD2_MM);
  pros::delay(80);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) {
    pros::delay(15000 - elapsed);
  }

  intake.move(0);
  conveyor.move(0);
  drive.set_percent(0, 0);
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);
}

void auton_left() {
  const double   comp = auton_voltage_comp();
  const uint32_t t0   = pros::millis();

  drive.set_brake(pros::E_MOTOR_BRAKE_HOLD);

  auto autopct = [comp](int p) {
    return auton_clamp_pct(static_cast<int>(p * comp));
  };

  const int drivePct = autopct(cfg.AUTO_DRIVE_PCT);

  const double FWD1_MM = 20.0 * 25.4;
  const double FWD2_MM = 9.5  * 25.4;

  const int convPower = cfg.CONV_PCT       * 127 / 100;
  const int intakeFwd = cfg.INTAKE_FWD_PCT * 127 / 100;

  conveyor.move(-convPower);
  pros::delay(80);

  drive_to_mm(FWD1_MM, drivePct, 2000);
  pros::delay(60);

  turn_imu_deg(-50.0);
  pros::delay(100);

  conveyor.move(0);
  pros::delay(60);

  drive_to_mm(FWD2_MM, drivePct, 1500);
  pros::delay(80);

  conveyor.move(convPower);
  intake.move(intakeFwd);
  pros::delay(1000);

  conveyor.move(0);
  intake.move(0);

  const uint32_t elapsed = pros::millis() - t0;
  if (elapsed < 15000) {
    pros::delay(15000 - elapsed);
  }

  intake.move(0);
  conveyor.move(0);
  drive.set_percent(0, 0);
  drive.set_brake(pros::E_MOTOR_BRAKE_COAST);
}
