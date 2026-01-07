#pragma once

#include "api.h"

/**
 * \file odom.hpp
 *
 * \brief Minimal odometry using 1 Rotation sensor for distance and 1 IMU for heading.
 *
 * \par What this module does
 *   - Integrates forward distance from rot_main into (x, y)
 *   - Uses imu_main heading as the robot orientation
 *   - Tracks total distance traveled in meters
 *
 * \par What this module does not do
 *   - No lateral tracking wheel. No true arc integration from wheel deltas
 *   - No drift correction, no field reset using vision or GPS
 *
 * \par Coordinate system
 *   - x, y in meters
 *   - theta in radians
 *   - Heading comes from IMU get_heading (degrees 0..360)
 *
 * \par Requisites
 *   - devices.hpp provides imu_main and rot_main
 *   - config.hpp provides WHEEL_CIRC_MM
 *
 * \par Tuning and correctness checklist
 *   1. Rotation units. PROS Rotation get_position returns centidegrees by default.
 *      Use a single convention everywhere. Either degrees or centidegrees.
 *   2. Gear ratio. If rot_main is on the 64:36 gearing, apply it once, always.
 *   3. IMU wrap. imu_main.get_heading wraps 0..360. Use rad conversion only.
 *   4. Units. This file uses meters for x, y, total distance.
 *
 * \par Thread control
 *   - g_odom_pause lets you pause odom updates during auton actions if you want.
 */

extern std::atomic<bool> g_odom_pause;

// ============================================================================
//   Pose container
// ============================================================================

/**
 * \struct Pose2D
 *
 * \brief Simple 2D pose.
 */
struct Pose2D {
  double x;     // meters
  double y;     // meters
  double theta; // radians
};

// ============================================================================
//   Odometry class
// ============================================================================

/**
 * \class Odometry
 *
 * \brief Integrates distance from Rotation and heading from IMU into a pose.
 */
class Odometry {
public:
  /**
   * \brief Construct odometry with pose = 0 and distance = 0.
   */
  Odometry();

  /**
   * \brief Reset pose and internal state.
   *
   * \param x Initial x in meters.
   * \param y Initial y in meters.
   * \param theta_rad Initial theta in radians.
   */
  void reset(double x, double y, double theta_rad);

  /**
   * \brief Update pose using latest sensor readings.
   *
   * \note Call at fixed period, recommended 10 ms.
   */
  void update();

  /**
   * \brief Get current pose snapshot.
   */
  Pose2D getPose() const;

  /**
   * \brief Total integrated distance traveled in meters.
   */
  double getTotalDistance();

private:
  Pose2D pose_;

  // Rotation last reading in degrees (Rotation returns centidegrees by default).
  double last_rot_deg_;

  // Heading unwrap state (degrees).
  double last_heading_deg_;
  double theta_cont_deg_;

  // Total traveled distance in meters.
  double total_distance_m_;

  // Helpers
  static double rot_deg();
  static double heading_deg_unwrapped(double heading_deg,
                                      double& last_heading_deg,
                                      double& theta_cont_deg);
};

extern Odometry odom;

/**
 * \brief Task function for running odometry continuously.
 */
extern void odom_task_fn();
  // Debug helpers
  void odom_zero();
  void odom_print_debug(int lcd_line = 1);

