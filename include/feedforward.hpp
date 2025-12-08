#pragma once
#include "config.hpp"
#include <cmath>

struct SimpleFeedforward {
  double kS;
  double kV;
  double kA;

  SimpleFeedforward(double ks = FF_kS,
                    double kv = FF_kV,
                    double ka = FF_kA)
    : kS(ks), kV(kv), kA(ka) {}

  double calculate(double velocity_mps, double accel_mps2) const {
    if (std::fabs(velocity_mps) < 1e-4 && std::fabs(accel_mps2) < 1e-4) {
      return 0.0;
    }

    double sign_v = (velocity_mps >= 0.0) ? 1.0 : -1.0;
    return kS * sign_v + kV * velocity_mps + kA * accel_mps2;
  }

  void setGains(double ks, double kv, double ka) {
    kS = ks;
    kV = kv;
    kA = ka;
  }
};
