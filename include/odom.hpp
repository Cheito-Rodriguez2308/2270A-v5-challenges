#pragma once

#include "pros/apix.h"

// MODULO: Odometry con Rotation e Inertial

struct Pose2D {
  double x;
  double y;
  double theta;
};

class Odometry {
public:
  Odometry();

  void reset(double x, double y, double theta_rad);
  void update();

  Pose2D getPose() const;
  double getTotalDistance();

private:
  Pose2D pose_;

  double last_rot_deg_;
  double total_distance_m_;
};

extern Odometry odom;
extern void odom_task_fn();
