#pragma once

#include "motion.hpp"

// Test simple para probar odometria y giros
inline void test_autonomous() {
  // Avanza 24 pulgadas
  drive_straight_mm(24.0 * 25.4);

  pros::delay(300);

  // Giro 90 grados
  turn_imu_deg_2stage(90.0);
}
