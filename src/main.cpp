#include "motion.hpp"
#include "PID.hpp"
#include "odom.hpp"
#include "drive.hpp"
#include "devices.hpp"
#include <cmath>

extern Odometry odom;

static double sgn(double x) {
  return (x >= 0.0) ? 1.0 : -1.0;
}

// ===============================
// RECTAS BASADAS EN ODOMETRIA
// ===============================
//
// - Usa odom.getTotalDistance() en METROS.
// - Convierte a mm internamente.
// - Usa PID sobre distancia restante.
// - Controla los seis motores de drive en paralelo.
//
void drive_mm_pid(double mm_target) {
  double dir = sgn(mm_target);
  double mm  = std::fabs(mm_target);

  // Distancia inicial en METROS desde odometria
  const double start_distance_m = odom.getTotalDistance();
  double traveled_mm = 0.0;

  // PID sobre distancia restante (mm)
  PID pid(0.25, 0.0, 0.05, -200.0, 200.0);

  // Brake mode consistente para drive
  lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // Timeout de seguridad por si algo se tranca
  const int TIMEOUT_MS = 3000;
  const uint32_t t_start = pros::millis();

  while (traveled_mm < mm) {
    uint32_t now = pros::millis();
    if (now - t_start >= static_cast<uint32_t>(TIMEOUT_MS)) {
      break;
    }

    double current_m = odom.getTotalDistance();
    traveled_mm = (current_m - start_distance_m) * 1000.0;  // metros a mm

    double remaining_mm = mm - traveled_mm;
    if (remaining_mm < 5.0) {
      break;
    }

    // dt fijo de 10 ms
    double output = pid.calculate(remaining_mm, 10.0 / 1000.0);
    int cmd = static_cast<int>(dir * output);

    lf.move_velocity(cmd);
    lm.move_velocity(cmd);
    lb.move_velocity(cmd);
    rf.move_velocity(cmd);
    rm.move_velocity(cmd);
    rb.move_velocity(cmd);

    pros::delay(10);
  }

  lf.move(0);
  lm.move(0);
  lb.move(0);
  rf.move(0);
  rm.move(0);
  rb.move(0);
}

// ===============================
// GIROS BASADOS EN IMU
// ===============================

static double wrap_deg(double a) {
  while (a >= 180.0) a -= 360.0;
  while (a <  -180.0) a += 360.0;
  return a;
}

// Giro relativo en grados usando heading del IMU.
// Usa PID sobre error de heading.
// No recalibra el IMU.
void turn_imu_deg(double delta_deg) {
  // Heading actual 0..360
  double start = imu_main.get_heading();
  double target = start + delta_deg;

  // Envuelve a 0..360
  if (target >= 360.0) target -= 360.0;
  if (target <   0.0)  target += 360.0;

  // PID sobre error en grados
  PID pid(2.0, 0.0, 0.25, -100.0, 100.0);

  const int TIMEOUT_MS = 2500;
  const uint32_t t_start = pros::millis();

  while (true) {
    uint32_t now = pros::millis();
    if (now - t_start >= static_cast<uint32_t>(TIMEOUT_MS)) {
      break;
    }

    double current = imu_main.get_heading();
    double error = wrap_deg(target - current);

    // Ventana de error aceptable
    if (std::fabs(error) < 1.5) {
      break;
    }

    double output = pid.calculate(error, 10.0 / 1000.0);
    int cmd = static_cast<int>(output);

    // Lado izquierdo y derecho en sentidos opuestos
    lf.move_velocity(cmd);
    lm.move_velocity(cmd);
    lb.move_velocity(cmd);
    rf.move_velocity(-cmd);
    rm.move_velocity(-cmd);
    rb.move_velocity(-cmd);

    pros::delay(10);
  }

  lf.move(0);
  lm.move(0);
  lb.move(0);
  rf.move(0);
  rm.move(0);
  rb.move(0);
}
