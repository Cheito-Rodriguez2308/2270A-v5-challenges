#pragma once

#include "api.h"

/**
 * \file pid.hpp
 *
 * \brief Reusable PID controller.
 *
 * \par What this PID does
 *   - P term. kp * error
 *   - I term. Integrates error over time, clamped by integral limits
 *   - D term. Uses derivative of error, disabled on first call after reset
 *   - Output clamping. Clamps final output to min_output..max_output
 *
 * \par Units
 *   - error in your chosen units
 *   - dt in seconds
 *   - output in your chosen command units
 *
 * \par Tuning workflow
 *   1. Set ki = 0, kd = 0
 *   2. Increase kp until response is fast but stable
 *   3. Add kd to reduce overshoot and oscillation
 *   4. Add small ki to remove steady state error
 *   5. Set integral limits to prevent windup
 *   6. Set output limits to match your actuator range
 *
 * \par Common mistakes
 *   - Passing dt in milliseconds instead of seconds
 *   - Forgetting to reset() between separate moves
 *   - Allowing integral to wind up when actuator saturates
 */

/**
 * \class PID
 *
 * \brief Simple PID controller with output and integral clamping.
 */
class PID {
public:
  /**
   * \brief Construct PID with gains and output limits.
   */
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

  /**
   * \brief Reset internal state.
   *
   * \details
   *   - Clears integral
   *   - Resets prev_error
   *   - Disables derivative term for next calculate() call
   */
  void reset() {
    integral_   = 0.0;
    prev_error_ = 0.0;
    first_run_  = true;
  }

  /**
   * \brief Compute PID output.
   *
   * \param error Current error value
   * \param dt Time step in seconds
   * \returns Clamped control output
   */
  double calculate(double error, double dt) {
    if (dt <= 0.0) {
      return 0.0;
    }

    // Proportional
    const double p_term = kp_ * error;

    // Integral with clamping
    integral_ += error * dt;
    integral_ = std::clamp(integral_, min_integral_, max_integral_);
    const double i_term = ki_ * integral_;

    // Derivative
    double d_term = 0.0;
    if (!first_run_) {
      const double derivative = (error - prev_error_) / dt;
      d_term = kd_ * derivative;
    } else {
      first_run_ = false;
    }

    prev_error_ = error;

    // Output clamp
    double output = p_term + i_term + d_term;
    output = std::clamp(output, min_output_, max_output_);
    return output;
  }

  /**
   * \brief Update gains at runtime.
   */
  void setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  /**
   * \brief Set output clamp limits.
   */
  void setOutputLimits(double min_output, double max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
  }

  /**
   * \brief Set integral clamp limits.
   *
   * \details Use to prevent integral windup.
   */
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
