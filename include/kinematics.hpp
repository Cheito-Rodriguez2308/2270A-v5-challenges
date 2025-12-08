#pragma once

#include "config.hpp"

struct WheelSpeeds {
  double left;
  double right;
};

inline WheelSpeeds chassisToWheelSpeeds(double v_mps, double w_rps) {
  double half_track = TRACK_WIDTH_M / 2.0;
  WheelSpeeds ws;
  ws.left  = v_mps - w_rps * half_track;
  ws.right = v_mps + w_rps * half_track;
  return ws;
}
