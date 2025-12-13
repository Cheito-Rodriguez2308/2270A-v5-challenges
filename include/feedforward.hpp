#pragma once

#include "config.hpp"
#include "api.h"

/**
 * \file feedforward.hpp
 *
 * \brief Minimal feedforward model for drivetrain velocity control.
 *
 * \par Model
 *   V = kS * sign(v) + kV * v + kA * a
 *
 * \par Units
 *   - velocity_mps in meters per second
 *   - accel_mps2 in meters per second squared
 *   - output in volts (or a voltage-like command)
 *
 * \par Tuning workflow
 *   1. Start with kA = 0
 *   2. Tune kS. Increase until drivetrain starts moving from rest with a small v command
 *   3. Tune kV. Adjust until steady speed matches target across multiple speeds
 *   4. Tune kA. Add until acceleration tracking improves without overshoot
 *
 * \par Notes
 *   - This struct does not clamp output. Clamp where you apply motor commands.
 *   - Uses FF_kS, FF_kV, FF_kA from config.hpp by default.
 */

/**
 * \struct SimpleFeedforward
 *
 * \brief Stores kS, kV, kA and provides calculate().
 */
struct SimpleFeedforward {
  double kS;
  double kV;
  double kA;

  /**
   * \brief Construct with optional custom gains.
   */
  SimpleFeedforward(double ks = FF_kS,
                    double kv = FF_kV,
                    double ka = FF_kA)
    : kS(ks), kV(kv), kA(ka) {}

  /**
   * \brief Compute feedforward output.
   *
   * \param velocity_mps Desired velocity in m/s
   * \param accel_mps2 Desired acceleration in m/s^2
   * \returns Voltage-like command from feedforward model
   */
  double calculate(double velocity_mps, double accel_mps2) const {
    // Dead zone. Avoid commanding kS when near zero.
    if (std::fabs(velocity_mps) < 1e-4 && std::fabs(accel_mps2) < 1e-4) {
      return 0.0;
    }

    const double sign_v = (velocity_mps >= 0.0) ? 1.0 : -1.0;
    return kS * sign_v + kV * velocity_mps + kA * accel_mps2;
  }

  /**
   * \brief Update gains at runtime.
   */
  void setGains(double ks, double kv, double ka) {
    kS = ks;
    kV = kv;
    kA = ka;
  }
};
