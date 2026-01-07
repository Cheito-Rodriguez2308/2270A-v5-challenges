#include "odom.hpp"
#include "devices.hpp"
#include "config.hpp"
#include "api.h"

/**
 * \file odom.cpp
 *
 * \brief Odometry implementation.
 *
 * \par Notes on sensor units
 *   - This implementation stores last_rot_deg_ in degrees
 *   - rot_main.get_position is treated as degrees in this file
 *
 * \warning If your Rotation is configured as centidegrees, fix the conversion.
 * Use one convention and stick to it across motion.cpp and odom.cpp.
 */

std::atomic<bool> g_odom_pause{false};

static constexpr double DEG2RAD = PI / 180.0;

// Gear ratio between Rotation and wheel.
static constexpr double ROT_TO_WHEEL_RATIO = 1.0; // 1:1

// ============================================================================
//   Construction
// ============================================================================
double Odometry::rot_deg() {
  // Rotation get_position returns centidegrees in PROS.
  return rot_main.get_position() / 100.0;
}

double Odometry::heading_deg_unwrapped(double h,
                                       double& last_h,
                                       double& cont_h) {
  // Unwrap h in [0, 360) into a continuous angle.
  double d = h - last_h;
  if (d > 180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  cont_h += d;
  last_h = h;
  return cont_h;
}

Odometry::Odometry()
  : pose_{0.0, 0.0, 0.0},
    last_rot_deg_(0.0),
    last_heading_deg_(0.0),
    theta_cont_deg_(0.0),
    total_distance_m_(0.0) {}

// ============================================================================
//   Reset
// ============================================================================

void Odometry::reset(double x, double y, double theta_rad) {
  pose_.x     = x;
  pose_.y     = y;
  pose_.theta = theta_rad;

  rot_main.reset_position();

  last_rot_deg_ = 0.0;

  const double h = imu_main.get_heading();
  last_heading_deg_ = h;
  theta_cont_deg_   = h;

  total_distance_m_ = 0.0;
}

// ============================================================================
//   Update step
// ============================================================================

void Odometry::update() {
  // Read sensors
  const double h_wrap = imu_main.get_heading();
  const double h_unwrap_deg =
    heading_deg_unwrapped(h_wrap, last_heading_deg_, theta_cont_deg_);
  const double theta_rad = h_unwrap_deg * DEG2RAD;

  const double cur_rot_deg = rot_deg();
  double delta_deg = cur_rot_deg - last_rot_deg_;

  // Apply tracking ratio (1:1) and scale
  delta_deg *= TRACKING_GEAR_RATIO;

  // Degrees -> meters
  double delta_s_m =
    (delta_deg / 360.0) * (TRACKING_WHEEL_CIRC_MM / 1000.0) * TRACKING_SCALE;

  // Glitch reject
  if (std::fabs(delta_s_m) > ODOM_MAX_STEP_M) {
    last_rot_deg_ = cur_rot_deg;
    pose_.theta   = theta_rad;
    return;
  }

  total_distance_m_ += delta_s_m;

  // Axis convention
  // theta = 0 points to +Y
  // Forward integrates into Y
  pose_.y     += delta_s_m * std::cos(theta_rad);
  pose_.x     += delta_s_m * std::sin(theta_rad);
  pose_.theta  = theta_rad;

  last_rot_deg_ = cur_rot_deg;
}

// ============================================================================
//   Accessors
// ============================================================================

Pose2D Odometry::getPose() const {
  return pose_;
}

double Odometry::getTotalDistance() {
  return total_distance_m_;
}

// ============================================================================
//   Global instance and task
// ============================================================================

Odometry odom;

void odom_task_fn() {
  while (true) {
    if (!g_odom_pause.load()) {
      odom.update();
    }
    pros::delay(10);
  }
}

void odom_zero() {
  odom.reset(0.0, 0.0, 0.0);
}

void odom_print_debug(int lcd_line) {
  const Pose2D p = odom.getPose();
  pros::lcd::print(lcd_line, "x %.3f y %.3f th %.1f",
                   p.x, p.y, p.theta * 180.0 / 3.1415926535);
}
