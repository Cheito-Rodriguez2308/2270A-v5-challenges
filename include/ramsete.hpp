#pragma once

#include "odom.hpp"
#include "config.hpp"
#include <cmath>

class RamseteController {
public:
  struct Output {
    double v;
    double w;
  };

  RamseteController(double b    = RAMSETE_B,
                    double zeta = RAMSETE_ZETA)
    : b_(b), zeta_(zeta) {}

  Output calculate(const Pose2D& current,
                   const Pose2D& reference,
                   double v_ref,
                   double w_ref) const {
    double dx     = reference.x - current.x;
    double dy     = reference.y - current.y;
    double dtheta = normalizeAngle(reference.theta - current.theta);

    double cos_theta = std::cos(current.theta);
    double sin_theta = std::sin(current.theta);

    double x_error_robot =  cos_theta * dx + sin_theta * dy;
    double y_error_robot = -sin_theta * dx + cos_theta * dy;

    double v_ref_abs = v_ref;
    double w_ref_abs = w_ref;
    double k = 2.0 * zeta_ *
               std::sqrt(w_ref_abs * w_ref_abs +
                         b_ * v_ref_abs * v_ref_abs);

    double sinc_term;
    const double eps = 1e-6;
    if (std::fabs(dtheta) < eps) {
      sinc_term = 1.0;
    } else {
      sinc_term = std::sin(dtheta) / dtheta;
    }

    double v_cmd = v_ref * std::cos(dtheta) + k * x_error_robot;
    double w_cmd = w_ref + b_ * v_ref * sinc_term * y_error_robot + k * dtheta;

    Output out{v_cmd, w_cmd};
    return out;
  }

  void setParams(double b, double zeta) {
    b_    = b;
    zeta_ = zeta;
  }

private:
  double b_;
  double zeta_;

  static double normalizeAngle(double angle) {
    const double pi = 3.1415926535;
    while (angle >  pi) angle -= 2.0 * pi;
    while (angle < -pi) angle += 2.0 * pi;
    return angle;
  }
};
