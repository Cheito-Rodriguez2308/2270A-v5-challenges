#pragma once

#include "api.h"
#include "devices.hpp"

// MODULO: drive
// Encapsula el chasis y expone metodos de movimiento

class Drive {
public:
  void reset_encoders();

  double front_left_deg() const;
  double front_right_deg() const;

  void set_brake(pros::motor_brake_mode_e_t mode);

  // Potencia por lado en porcentaje [-100, 100]
  void set_percent(int left_pct, int right_pct);

  static double deg_to_mm(double deg);
  static double mm_to_deg(double mm);

  void drive_straight_mm(
    double dist_mm,
    int base_pct,
    double kP_mm,
    double slow_down_mm,
    pros::motor_brake_mode_e_t end_brake
  );

  void turn_right_deg(double deg_total, int fast_pct, int slow_pct);
  void turn_left_deg(double deg_total, int fast_pct, int slow_pct);
};

extern Drive drive;
