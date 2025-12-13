#pragma once

#include "odom.hpp"
#include "config.hpp"
#include "api.h"

/**
 * \file ramsete.hpp
 *
 * \brief RAMSETE controller for differential drive trajectory tracking.
 *
 * \par What this controller does
 *   Converts pose error (current vs reference) into chassis commands:
 *   - v: linear velocity command (m/s)
 *   - w: angular velocity command (rad/s)
 *
 * \par Inputs you must provide
 *   - current pose (x, y in meters, theta in radians)
 *   - reference pose (x, y in meters, theta in radians)
 *   - v_ref and w_ref from your trajectory at the same time index
 *
 * \par Outputs
 *   - v_cmd, w_cmd. Feed to kinematics (wheel speeds), then to motor control.
 *
 * \par Units. Must be consistent
 *   - x, y in meters
 *   - theta in radians
 *   - v_ref in m/s
 *   - w_ref in rad/s
 *
 * \par Tuning
 *   - b controls aggressiveness with lateral error and curvature
 *   - zeta controls damping. Higher reduces oscillation but can feel sluggish
 *
 * \par Recommended starting values
 *   - b = 2.0
 *   - zeta = 0.7
 *
 * \par Common mistakes
 *   - Passing degrees instead of radians for theta
 *   - Mixing meters with millimeters or inches
 *   - Using TRACK_WIDTH_MM directly in meters math elsewhere
 */

/**
 * \class RamseteController
 *
 * \brief Stateless RAMSETE controller with adjustable b and zeta.
 */
class RamseteController {
public:
  /**
   * \struct Output
   *
   * \brief Chassis velocity command output.
   */
  struct Output {
    double v;  // linear velocity command (m/s)
    double w;  // angular velocity command (rad/s)
  };

  /**
   * \brief Construct controller with parameters.
   */
  RamseteController(double b = RAMSETE_B,
                    double zeta = RAMSETE_ZETA)
    : b_(b), zeta_(zeta) {}

  /**
   * \brief Calculate velocity commands from pose error.
   *
   * \param current Current pose (meters, radians)
   * \param reference Desired pose (meters, radians)
   * \param v_ref Reference linear velocity (m/s)
   * \param w_ref Reference angular velocity (rad/s)
   * \returns Output {v_cmd, w_cmd}
   */
  Output calculate(const Pose2D& current,
                   const Pose2D& reference,
                   double v_ref,
                   double w_ref) const {
    // World-frame pose error
    const double dx     = reference.x - current.x;
    const double dy     = reference.y - current.y;
    const double dtheta = normalizeAngle(reference.theta - current.theta);

    // Transform position error into robot frame
    const double cos_theta = std::cos(current.theta);
    const double sin_theta = std::sin(current.theta);

    const double x_error_robot =  cos_theta * dx + sin_theta * dy;
    const double y_error_robot = -sin_theta * dx + cos_theta * dy;

    // Gain term from RAMSETE paper
    const double v_ref_abs = v_ref;
    const double w_ref_abs = w_ref;

    const double k =
      2.0 * zeta_ *
      std::sqrt(w_ref_abs * w_ref_abs + b_ * v_ref_abs * v_ref_abs);

    // sinc(dtheta) with small-angle protection
    double sinc_term;
    const double eps = 1e-6;
    if (std::fabs(dtheta) < eps) {
      sinc_term = 1.0;
    } else {
      sinc_term = std::sin(dtheta) / dtheta;
    }

    // Chassis command
    const double v_cmd = v_ref * std::cos(dtheta) + k * x_error_robot;
    const double w_cmd = w_ref + b_ * v_ref * sinc_term * y_error_robot + k * dtheta;

    return Output{v_cmd, w_cmd};
  }

  /**
   * \brief Update parameters at runtime.
   */
  void setParams(double b, double zeta) {
    b_ = b;
    zeta_ = zeta;
  }

private:
  double b_;
  double zeta_;

  /**
   * \brief Normalize angle to [-pi, +pi].
   */
  static double normalizeAngle(double angle) {
    const double pi = 3.1415926535;
    while (angle >  pi) angle -= 2.0 * pi;
    while (angle < -pi) angle += 2.0 * pi;
    return angle;
  }
};
