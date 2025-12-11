#pragma once
#include "pros/apix.h"

// Reset de motores
void reset_drive_positions();

// Conversiones mm <-> rotation
double rot_deg_to_mm(double deg);
double mm_to_rot_deg(double mm);

// Error de ángulo [-180, 180]
double angle_error(double target, double current);

// Drive recto usando rotation + IMU
void drive_straight_mm(double dist_mm,
                       int base_pct = 50,
                       double kP_heading = 0.5,
                       double slow_down_mm = 120.0,
                       pros::motor_brake_mode_e end_brake = pros::E_MOTOR_BRAKE_BRAKE);

// Giro en dos etapas usando IMU
void turn_imu_deg_2stage(double deg_total,
                         int fast_pct = 35,
                         int slow_pct = 22,
                         double split = 0.92,
                         int settle_ms = 120);
