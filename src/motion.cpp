#include "motion.hpp"
#include "devices.hpp"
#include <cmath>
#include <algorithm>

// Ruedas 3.25" = ~259.3 mm
static constexpr double WHEEL_TRAVEL_MM = 259.3;

// Relación 60:36 entre rotation y rueda
static constexpr double GEAR_RATIO_ROT_TO_WHEEL = 64.0 / 36.0;

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
  rot_main.reset();
}

// -----------------------------
// Conversion rotation -> mm
// -----------------------------
double rot_deg_to_mm(double deg) {
  double rev_rot = deg / 360.0;
  double rev_wheel = rev_rot * GEAR_RATIO_ROT_TO_WHEEL;
  return rev_wheel * WHEEL_TRAVEL_MM;
}

double mm_to_rot_deg(double mm) {
  double rev_wheel = mm / WHEEL_TRAVEL_MM;
  double rev_rot = rev_wheel / GEAR_RATIO_ROT_TO_WHEEL;
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

// ====================================================
// DRIVE RECTO EXACTO COMO PYTHON
// ====================================================

static int pct_to_rpm(int pct) {
  pct = std::clamp(pct, -100, 100);
  // gearset 18: max aprox 200 rpm
  return static_cast<int>(pct * 200 / 100);
}

static int apply_min_rpm(int rpm) {
  if (rpm == 0) return 0;
  int s = (rpm > 0) ? 1 : -1;
  int m = std::abs(rpm);
  m = std::max(m, 70);   // prueba 70–100
  return s * m;
}

void drive_straight_mm(double dist_mm,
                       int base_pct,
                       double kP_heading,
                       double slow_down_mm,
                       pros::motor_brake_mode_e end_brake)
{
  reset_drive_positions();

  int direction = (dist_mm >= 0) ? 1 : -1;
  double target_mm = std::abs(dist_mm);

  // Heading inicial
  double start_heading = imu_main.get_heading();

  // Timer de seguridad
  uint32_t start_time = pros::millis();
  double est_speed = std::max(150.0, base_pct * 2.2); 
  double max_time_ms = std::max(2000.0, (target_mm / est_speed + 1.0) * 1000.0);

  while (true) {
    double dt = pros::millis() - start_time;
    if (dt > max_time_ms) break;

    double rot_deg = std::abs(rot_main.get_position());
    double rot_mm = rot_deg_to_mm(rot_deg);

    // Fallback motor si rotation no se mueve
    double mot_deg = std::abs(lf.get_position());
    double mot_mm = (mot_deg / 360.0) * WHEEL_TRAVEL_MM;

    double d_mm = (rot_mm < 1.0 && mot_mm > 3.0) ? mot_mm : rot_mm;

    double remaining = target_mm - d_mm;
    if (remaining <= 2.0) break;

    // Corrección de heading
    double heading = imu_main.get_heading();
    double err = angle_error(start_heading, heading);

    double trim = kP_heading * err;
    trim = std::clamp(trim, -15.0, 15.0);

    // Slowdown
    int pct = base_pct;
    if (remaining < slow_down_mm) {
      pct = std::max(35, static_cast<int>(base_pct * (remaining / slow_down_mm)));
    }

    int left_cmd  = pct - trim;
    int right_cmd = pct + trim;

    left_cmd  = std::clamp(left_cmd, -100, 100);
    right_cmd = std::clamp(right_cmd, -100, 100);

    // Aplicar velocidad (RPM-Based)
    auto pct_to_rpm = [](int pct) {
      pct = std::clamp(pct, -100, 100);
      // gearset 18: max aprox 200 rpm
      return static_cast<int>(pct * 200 / 100);
    };

    // un mínimo para romper fricción en recta
    auto apply_min = [](int rpm) {
      if (rpm == 0) return 0;
      int sign = (rpm > 0) ? 1 : -1;
      int mag = std::abs(rpm);
      mag = std::max(mag, 60); // prueba 60–90
      return sign * mag;
    };

    int left_rpm  = apply_min_rpm(pct_to_rpm(direction * left_cmd));
    int right_rpm = apply_min_rpm(pct_to_rpm(direction * right_cmd));

    lf.move_velocity(left_rpm);
    lm.move_velocity(left_rpm);
    lb.move_velocity(left_rpm);

    rf.move_velocity(right_rpm);
    rm.move_velocity(right_rpm);
    rb.move_velocity(right_rpm);

    pros::delay(10);
  } 

  // Frenar
  lf.set_brake_mode(end_brake);
  lm.set_brake_mode(end_brake);
  lb.set_brake_mode(end_brake);
  rf.set_brake_mode(end_brake);
  rm.set_brake_mode(end_brake);
  rb.set_brake_mode(end_brake);

  lf.brake();
  lm.brake();
  lb.brake();
  rf.brake();
  rm.brake();
  rb.brake();
}

// ====================================================
// GIRO EN 2 ETAPAS EXACTO COMO PYTHON
// ====================================================
void turn_imu_deg_2stage(double deg_total,
                         int fast_pct,
                         int slow_pct,
                         double split,
                         int settle_ms)
{
  double start = imu_main.get_heading();
  double target_1 = start + deg_total * split;
  double target_2 = start + deg_total;

  // Normalizar
  auto norm = [&](double a){
    while (a >= 360) a -= 360;
    while (a < 0) a += 360;
    return a;
  };

  target_1 = norm(target_1);
  target_2 = norm(target_2);

  // Etapa 1
  while (true) {
    double cur = imu_main.get_heading();
    double err = angle_error(target_1, cur);
    if (std::abs(err) < 1.5) break;

    int cmd = (err > 0 ? fast_pct : -fast_pct);

    lf.move_velocity(cmd);
    lm.move_velocity(cmd);
    lb.move_velocity(cmd);

    rf.move_velocity(-cmd);
    rm.move_velocity(-cmd);
    rb.move_velocity(-cmd);

    pros::delay(10);
  }

  pros::delay(settle_ms);

  // Etapa 2
  while (true) {
    double cur = imu_main.get_heading();
    double err = angle_error(target_2, cur);
    if (std::abs(err) < 1.0) break;

    int cmd = (err > 0 ? slow_pct : -slow_pct);

    lf.move_velocity(cmd);
    lm.move_velocity(cmd);
    lb.move_velocity(cmd);

    rf.move_velocity(-cmd);
    rm.move_velocity(-cmd);
    rb.move_velocity(-cmd);

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
