#pragma once

#include "api.h"

// Movimiento recto usando Odometry y PID
void drive_to_mm(double target_mm,
                 double max_pct = 80.0,
                 double timeout_ms = 3000);

// Giro a angulo absoluto con Odometry
void turn_to_deg(double target_deg,
                 double max_pct = 80.0,
                 double timeout_ms = 2500);

// Helpers sencillos
void drive_mm_pid(double mm_target);
void turn_imu_deg(double delta_deg);
