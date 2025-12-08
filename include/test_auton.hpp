#pragma once

#include "motion.hpp"

// Test simple para probar odometria y giros
inline void test_autonomous() {
  // Avanza 24 pulgadas
  drive_mm_pid(24.0 * 25.4);

  pros::delay(300);

  // Giro 90 grados
  turn_imu_deg(90.0);
}
