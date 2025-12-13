#pragma once

#include "config.hpp"

/**
 * \file kinematics.hpp
 *
 * \brief Differential drive kinematics helpers.
 *
 * \par What this file does
 *   Converts chassis velocities (v, w) into left and right wheel linear speeds.
 *
 * \par Units
 *   - v_mps in meters per second
 *   - w_rps in radians per second
 *   - TRACK_WIDTH_MM is in millimeters. Convert to meters inside the function.
 *
 * \par Common mistake
 *   Using TRACK_WIDTH_MM directly without converting mm to meters.
 *   That makes wheel speeds 1000x too large.
 */

/**
 * \struct WheelSpeeds
 *
 * \brief Wheel linear speeds for left and right sides.
 */
struct WheelSpeeds {
  double left;   // m/s
  double right;  // m/s
};

/**
 * \brief Convert chassis velocities to wheel linear speeds.
 *
 * \param v_mps Chassis linear velocity (m/s)
 * \param w_rps Chassis angular velocity (rad/s)
 * \returns WheelSpeeds in m/s
 */
inline WheelSpeeds chassisToWheelSpeeds(double v_mps, double w_rps) {
  const double track_m = TRACK_WIDTH_MM / 1000.0;
  const double half_track = track_m / 2.0;

  WheelSpeeds ws;
  ws.left  = v_mps - w_rps * half_track;
  ws.right = v_mps + w_rps * half_track;
  return ws;
}
