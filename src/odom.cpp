#include "odom.hpp"
#include "devices.hpp"
#include "config.hpp"
#include <cmath>

static constexpr double DEG2RAD = 3.1415926535 / 180.0;

Odometry::Odometry()
  : pose_{0.0, 0.0, 0.0},
    last_rot_deg_(0.0),
    total_distance_m_(0.0) {}

void Odometry::reset(double x, double y, double theta_rad) {
  pose_.x     = x;
  pose_.y     = y;
  pose_.theta = theta_rad;

  rot_main.reset_position();

  last_rot_deg_     = rot_main.get_position();
  total_distance_m_ = 0.0;
}

void Odometry::update() {
  double heading_deg = imu_main.get_heading();
  double heading_rad = heading_deg * DEG2RAD;

  double current_rot_deg = rot_main.get_position();
  double delta_deg       = current_rot_deg - last_rot_deg_;

  const double sensor_gear_teeth = 64.0;
  const double wheel_gear_teeth  = 36.0;
  delta_deg *= (sensor_gear_teeth / wheel_gear_teeth);

  double delta_s_m = (delta_deg / 360.0) * WHEEL_CIRC_MM;

  total_distance_m_ += delta_s_m;

  pose_.x     += delta_s_m * std::cos(heading_rad);
  pose_.y     += delta_s_m * std::sin(heading_rad);
  pose_.theta  = heading_rad;

  last_rot_deg_ = current_rot_deg;
}

Pose2D Odometry::getPose() const {
  return pose_;
}

double Odometry::getTotalDistance() {
  return total_distance_m_;
}

Odometry odom;

void odom_task_fn() {
  while (true) {
    odom.update();
    pros::delay(10);
  }
}
