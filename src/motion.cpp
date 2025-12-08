#include "motion.hpp"
#include "pid.hpp"
#include "odom.hpp"
#include "drive.hpp"
#include "devices.hpp"
#include <cmath>

extern Odometry odom;

static constexpr double RAD2DEG = 180.0 / 3.1415926535;
static constexpr double M_TO_MM = 1000.0;

// PID distancia en mm
static PID pid_dist(
  0.8,
  0.00,
  0.06,
  -100.0,
   100.0
);

// PID heading hold en grados
static PID pid_heading_hold(
  0.80,
  0.00,
  0.00,
  -40.0,
   40.0
);

// PID giro absoluto en grados
static PID pid_turn(
  1.0,
  0.00,
  0.10,
  -100.0,
   100.0
);

void drive_to_mm(double target_mm, double max_pct, double timeout_ms) {
  pid_dist.reset();
  pid_heading_hold.reset();

  Pose2D pose0 = odom.getPose();
  double theta0_rad = pose0.theta;
  double theta0_deg = theta0_rad * RAD2DEG;

  double s0_m = odom.getTotalDistance();

  uint32_t t_start   = pros::millis();
  uint32_t last_time = t_start;

  while (true) {
    uint32_t now = pros::millis();
    double dt = (now - last_time) / 1000.0;
    if (dt <= 0.0) dt = 0.01;
    last_time = now;

    double s_m  = odom.getTotalDistance();
    double ds_m = s_m - s0_m;
    double forward_mm = ds_m * M_TO_MM;

    double error_dist_mm = target_mm - forward_mm;

    if (std::fabs(error_dist_mm) < 5.0 || (now - t_start) > timeout_ms) {
      break;
    }

    double u_dist = pid_dist.calculate(error_dist_mm, dt);
    if (u_dist >  max_pct) u_dist =  max_pct;
    if (u_dist < -max_pct) u_dist = -max_pct;

    Pose2D pose = odom.getPose();
    double heading_deg = pose.theta * RAD2DEG;

    double error_heading_deg = theta0_deg - heading_deg;
    while (error_heading_deg >  180.0) error_heading_deg -= 360.0;
    while (error_heading_deg < -180.0) error_heading_deg += 360.0;

    double u_head = pid_heading_hold.calculate(error_heading_deg, dt);

    double left  = -(u_dist + u_head);
    double right = -(u_dist - u_head);

    drive.set_percent(static_cast<int>(left),
                      static_cast<int>(right));

    pros::delay(10);
  }

  drive.set_percent(0, 0);
}

void turn_to_deg(double target_deg, double max_pct, double timeout_ms) {
  pid_turn.reset();

  uint32_t t_start   = pros::millis();
  uint32_t last_time = t_start;

  while (true) {
    uint32_t now = pros::millis();
    double dt = (now - last_time) / 1000.0;
    if (dt <= 0.0) dt = 0.01;
    last_time = now;

    Pose2D pose = odom.getPose();
    double heading_deg = pose.theta * RAD2DEG;

    double error_deg = target_deg - heading_deg;
    while (error_deg >  180.0) error_deg -= 360.0;
    while (error_deg < -180.0) error_deg += 360.0;

    if ((std::fabs(error_deg) < 1.0 && (now - t_start) > 200) ||
        (now - t_start) > timeout_ms) {
      break;
    }

    double u = pid_turn.calculate(error_deg, dt);

    if (u >  max_pct) u =  max_pct;
    if (u < -max_pct) u = -max_pct;

    drive.set_percent(static_cast<int>(u),
                      static_cast<int>(-u));

    pros::delay(10);
  }

  drive.set_percent(0, 0);
}

static double sgn(double x) {
  return (x >= 0.0) ? 1.0 : -1.0;
}

void drive_mm_pid(double mm_target) {
  double dir = sgn(mm_target);
  double mm  = std::fabs(mm_target);

  const double start_distance_m = odom.getTotalDistance();
  double traveled_mm = 0.0;

  PID pid(0.25, 0.0, 0.05, -200.0, 200.0);

  lf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rf.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rb.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (traveled_mm < mm) {
    double current_m = odom.getTotalDistance();
    traveled_mm = (current_m - start_distance_m) * 1000.0;

    double remaining_mm = mm - traveled_mm;

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

static double wrap_deg(double a) {
  while (a >= 180.0) a -= 360.0;
  while (a <  -180.0) a += 360.0;
  return a;
}

void turn_imu_deg(double delta_deg) {
  double start = imu_main.get_heading();
  double target = start + delta_deg;

  if (target >= 360.0) target -= 360.0;
  if (target <   0.0)  target += 360.0;

  PID pid(2.0, 0.0, 0.25, -100.0, 100.0);

  while (true) {
    double current = imu_main.get_heading();
    double error = wrap_deg(target - current);

    if (std::fabs(error) < 1.5) {
      break;
    }

    double output = pid.calculate(error, 10.0 / 1000.0);
    int cmd = static_cast<int>(output);

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
