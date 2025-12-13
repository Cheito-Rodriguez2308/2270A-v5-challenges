#pragma once

#include "motion.hpp"

/**
 * \file test_auton.hpp
 *
 * \brief Minimal motion test routine.
 *
 * \par Purpose
 *   - Validate drive_straight_mm and turn_imu_deg_2stage quickly
 *   - Useful after tuning kP_heading, slow_down_mm, and turn speeds
 *
 * \par Usage
 *   Call test_autonomous() from opcontrol() or autonomous() during debugging.
 *
 * \warning
 *   This routine moves the robot. Use on a safe field space.
 */

/**
 * \brief Simple autonomous test.
 *
 * \details
 *   - Drive forward 24 inches
 *   - Pause
 *   - Turn 90 degrees
 */
inline void test_autonomous() {
  drive_straight_mm(24.0 * 25.4);

  pros::delay(300);

  turn_imu_deg_2stage(90.0);
}
