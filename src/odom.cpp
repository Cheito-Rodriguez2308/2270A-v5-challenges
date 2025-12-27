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

static constexpr double PI = 3.1415926535;
static constexpr double DEG2RAD = PI / 180.0;

// Gear ratio between Rotation and wheel.
static constexpr double SENSOR_GEAR_TEETH = 64.0;
static constexpr double WHEEL_GEAR_TEETH  = 36.0;
static constexpr double ROT_TO_WHEEL_RATIO = (SENSOR_GEAR_TEETH / WHEEL_GEAR_TEETH);

// ============================================================================
//   Construction
// ============================================================================

Odometry::Odometry()
  : pose_{0.0, 0.0, 0.0},
    last_rot_deg_(0.0),
    total_distance_m_(0.0) {}

// ============================================================================
//   Reset
// ============================================================================

void Odometry::reset(double x, double y, double theta_rad) {
  pose_.x     = x;
  pose_.y     = y;
  pose_.theta = theta_rad;

  rot_main.reset_position();

  // Store last rotation in degrees.
  // If get_position returns centidegrees, change to: rot_main.get_position() / 100.0
  last_rot_deg_ = (rot_main.get_position() / 100.0);

  total_distance_m_ = 0.0;
}

// ============================================================================
//   Update step
// ============================================================================

void Odometry::update() {
  // IMU heading in radians.
  const double heading_deg = imu_main.get_heading();
  const double heading_rad = heading_deg * DEG2RAD;

  // Rotation delta in degrees.
  const double current_rot_deg = rot_main.get_position() / 100.0;
  double delta_deg = current_rot_deg - last_rot_deg_;

  // Convert sensor rotation to wheel rotation using gear ratio.
  delta_deg *= ROT_TO_WHEEL_RATIO;

  // Convert wheel rotation degrees to linear distance in meters.
  const double delta_s_m =
    (delta_deg / 360.0) * (WHEEL_CIRC_MM / 1000.0);

  total_distance_m_ += delta_s_m;

  // Integrate forward distance along current heading.
  pose_.x     += delta_s_m * std::cos(heading_rad);
  pose_.y     += delta_s_m * std::sin(heading_rad);
  pose_.theta  = heading_rad;

  last_rot_deg_ = current_rot_deg;
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
