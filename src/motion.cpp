#include "motion.hpp"
#include "devices.hpp"
#include <cmath>
#include <algorithm>

// Ruedas 3.25" = ~259.3 mm
static constexpr double WHEEL_TRAVEL_MM = 259.3;

// Relación 64:36 entre rotation y rueda
static constexpr double GEAR_RATIO_ROT_TO_WHEEL = 64.0 / 36.0;

// gearset 18: max aprox 200 rpm
static constexpr int MAX_RPM_18 = 200;

// -----------------------------
// Reset de posiciones
// -----------------------------
void reset_drive_positions() {
  lf.tare_position();
  lm.tare_position();
  lb.tare_position();
  rf.tare_position();
  rm.tare_position();
  rb.tare_position();
  rot_main.reset_position();
}

// -----------------------------
// Conversion rotation -> mm
// -----------------------------
double rot_deg_to_mm(double deg) {
  const double rev_rot   = deg / 360.0;
  const double rev_wheel = rev_rot * GEAR_RATIO_ROT_TO_WHEEL;
  return rev_wheel * WHEEL_TRAVEL_MM;
}

double mm_to_rot_deg(double mm) {
  const double rev_wheel = mm / WHEEL_TRAVEL_MM;
  const double rev_rot   = rev_wheel / GEAR_RATIO_ROT_TO_WHEEL;
  return rev_rot * 360.0;
}

// -----------------------------
// Error de ángulo [-180, 180]
// -----------------------------
double angle_error(double target, double current) {
  double err = target - current;
  while (err > 180) err -= 360;
  while (err < -180) err += 360;
  return err;
}

// -----------------------------
// Helpers internos: pct -> rpm, mínimo rpm por fricción
// -----------------------------
static int pct_to_rpm(int pct) {
  pct = std::clamp(pct, -100, 100);
  return (pct * MAX_RPM_18) / 100;
}

static int apply_min_rpm(int rpm, double remaining_mm) {
  if (rpm == 0) return 0;

  const int sign = (rpm > 0) ? 1 : -1;
  int mag = std::abs(rpm);

  const int min_far  = 60;
  const int min_near = 30;
  const int min_rpm  = (remaining_mm > 180.0) ? min_far : min_near;

  mag = std::max(mag, min_rpm);
  return sign * mag;
}

// ====================================================
// DRIVE RECTO SUAVE (Rotation + IMU)
// ====================================================
void drive_straight_mm(double dist_mm,
                       int base_pct,
                       double kP_heading,
                       double slow_down_mm,
                       pros::motor_brake_mode_e end_brake,
                       int soft_settle_ms,
                       int brake_pulse_ms)
{
  reset_drive_positions();

  const int dir = (dist_mm >= 0) ? 1 : -1;
  const double target_mm = std::abs(dist_mm);

  // Durante movimiento: COAST para sensación más humana
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  const double start_heading = imu_main.get_heading();

  // Timeout simple
  const uint32_t t0 = pros::millis();
  const double est_speed_mm_s = std::max(180.0, (std::abs(base_pct) / 100.0) * 350.0);
  const uint32_t timeout_ms =
    (uint32_t)std::max(2000.0, (target_mm / est_speed_mm_s + 1.0) * 1000.0);

  // Slew de RPM para suavizar aceleración y frenado
  int left_rpm_cmd  = 0;
  int right_rpm_cmd = 0;
  const int RPM_SLEW_PER_10MS = 18;  // 12-25

  auto slew_to = [&](int target, int current) {
    const int d = target - current;
    if (std::abs(d) <= RPM_SLEW_PER_10MS) return target;
    return current + ((d > 0) ? RPM_SLEW_PER_10MS : -RPM_SLEW_PER_10MS);
  };

  while (true) {
    const uint32_t now = pros::millis();
    if (now - t0 > timeout_ms) break;

    const double rot_deg = std::abs(rot_main.get_position());
    const double rot_mm  = rot_deg_to_mm(rot_deg);

    const double mot_deg = std::abs(lf.get_position());
    const double mot_mm  = (mot_deg / 360.0) * WHEEL_TRAVEL_MM;

    const double d_mm = (rot_mm < 1.0 && mot_mm > 3.0) ? mot_mm : rot_mm;

    const double remaining = target_mm - d_mm;
    if (remaining <= 2.0) break;

    const double heading = imu_main.get_heading();
    const double err = angle_error(start_heading, heading);

    double trim = kP_heading * err;
    trim = std::clamp(trim, -15.0, 15.0);

    int pct = base_pct;
    if (slow_down_mm > 0.0 && remaining < slow_down_mm) {
      pct = std::max(35, (int)std::round(base_pct * (remaining / slow_down_mm)));
    }

    int left_pct  = (int)std::round(pct - trim);
    int right_pct = (int)std::round(pct + trim);

    left_pct  = std::clamp(left_pct,  -100, 100);
    right_pct = std::clamp(right_pct, -100, 100);

    const int left_target_rpm  = apply_min_rpm(pct_to_rpm(dir * left_pct),  remaining);
    const int right_target_rpm = apply_min_rpm(pct_to_rpm(dir * right_pct), remaining);

    left_rpm_cmd  = slew_to(left_target_rpm,  left_rpm_cmd);
    right_rpm_cmd = slew_to(right_target_rpm, right_rpm_cmd);

    lf.move_velocity(left_rpm_cmd);
    lm.move_velocity(left_rpm_cmd);
    lb.move_velocity(left_rpm_cmd);

    rf.move_velocity(right_rpm_cmd);
    rm.move_velocity(right_rpm_cmd);
    rb.move_velocity(right_rpm_cmd);

    pros::delay(10);
  }

  // Ramp down suave
  for (int i = 0; i < 8; i++) {
    left_rpm_cmd  = slew_to(0, left_rpm_cmd);
    right_rpm_cmd = slew_to(0, right_rpm_cmd);

    lf.move_velocity(left_rpm_cmd);
    lm.move_velocity(left_rpm_cmd);
    lb.move_velocity(left_rpm_cmd);

    rf.move_velocity(right_rpm_cmd);
    rm.move_velocity(right_rpm_cmd);
    rb.move_velocity(right_rpm_cmd);

    pros::delay(10);
  }

  // Pulse BRAKE para parar limpio
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  lf.move_velocity(0);
  lm.move_velocity(0);
  lb.move_velocity(0);
  rf.move_velocity(0);
  rm.move_velocity(0);
  rb.move_velocity(0);

  if (brake_pulse_ms > 0) pros::delay(brake_pulse_ms);

  // Luego COAST para sensación natural
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  if (soft_settle_ms > 0) pros::delay(soft_settle_ms);

  // Estado final según caller
  lf.set_brake_mode(end_brake);
  lm.set_brake_mode(end_brake);
  lb.set_brake_mode(end_brake);
  rf.set_brake_mode(end_brake);
  rm.set_brake_mode(end_brake);
  rb.set_brake_mode(end_brake);

  if (end_brake != pros::E_MOTOR_BRAKE_COAST) {
    lf.brake();
    lm.brake();
    lb.brake();
    rf.brake();
    rm.brake();
    rb.brake();
  }
}

// ====================================================
// GIRO EN 2 ETAPAS (IMU)
// ====================================================
void turn_imu_deg_2stage(double deg_total,
                         int fast_pct,
                         int slow_pct,
                         double split,
                         int settle_ms)
{
  const double start = imu_main.get_heading();
  double target_1 = start + deg_total * split;
  double target_2 = start + deg_total;

  auto norm = [&](double a){
    while (a >= 360) a -= 360;
    while (a < 0) a += 360;
    return a;
  };

  target_1 = norm(target_1);
  target_2 = norm(target_2);

  // Etapa 1
  while (true) {
    const double cur = imu_main.get_heading();
    const double err = angle_error(target_1, cur);
    if (std::abs(err) < 1.5) break;

    const int sign = (err > 0) ? 1 : -1;
    const int cmd_rpm = pct_to_rpm(sign * fast_pct);

    lf.move_velocity(cmd_rpm);
    lm.move_velocity(cmd_rpm);
    lb.move_velocity(cmd_rpm);

    rf.move_velocity(-cmd_rpm);
    rm.move_velocity(-cmd_rpm);
    rb.move_velocity(-cmd_rpm);

    pros::delay(10);
  }

  if (settle_ms > 0) pros::delay(settle_ms);

  // Etapa 2
  while (true) {
    const double cur = imu_main.get_heading();
    const double err = angle_error(target_2, cur);
    if (std::abs(err) < 1.0) break;

    const int sign = (err > 0) ? 1 : -1;
    const int cmd_rpm = pct_to_rpm(sign * slow_pct);

    lf.move_velocity(cmd_rpm);
    lm.move_velocity(cmd_rpm);
    lb.move_velocity(cmd_rpm);

    rf.move_velocity(-cmd_rpm);
    rm.move_velocity(-cmd_rpm);
    rb.move_velocity(-cmd_rpm);

    pros::delay(10);
  }

  // Stop final
  lf.brake();
  lm.brake();
  lb.brake();
  rf.brake();
  rm.brake();
  rb.brake();
}
