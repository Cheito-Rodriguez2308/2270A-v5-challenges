#pragma once

#include "api.h"
#include <algorithm>
#include <cmath>

// MODULO: PID reutilizable

class PID {
public:
  PID(double kp, double ki, double kd,
      double min_output = -1e9,
      double max_output =  1e9)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      min_output_(min_output),
      max_output_(max_output),
      min_integral_(-1e9),
      max_integral_( 1e9),
      integral_(0.0),
      prev_error_(0.0),
      first_run_(true) {}

  void reset() {
    integral_   = 0.0;
    prev_error_ = 0.0;
    first_run_  = true;
  }

  double calculate(double error, double dt) {
    if (dt <= 0.0) {
      return 0.0;
    }

    double p_term = kp_ * error;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, min_integral_, max_integral_);
    double i_term = ki_ * integral_;

    double d_term = 0.0;
    if (!first_run_) {
      double derivative = (error - prev_error_) / dt;
      d_term = kd_ * derivative;
    } else {
      first_run_ = false;
    }

    prev_error_ = error;

    double output = p_term + i_term + d_term;
    output = std::clamp(output, min_output_, max_output_);
    return output;
  }

  void setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setOutputLimits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
  }

  void setIntegralLimits(double min_integral, double max_integral) {
    min_integral_ = min_integral;
    max_integral_ = max_integral;
  }

private:
  double kp_;
  double ki_;
  double kd_;

  double min_output_;
  double max_output_;

  double min_integral_;
  double max_integral_;

  double integral_;
  double prev_error_;
  bool   first_run_;
};
