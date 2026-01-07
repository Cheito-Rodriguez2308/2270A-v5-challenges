#pragma once
#include "pros/apix.h"

/**
 * \file motion.hpp
 *
 * \brief Motion primitives used by auton.
 *
 * \par What this module does
 *   - Converts tracking Rotation degrees to linear millimeters
 *   - Drives straight using Rotation for distance and IMU for heading hold
 *   - Turns using IMU in two stages for accuracy
 *
 * \par Your robot assumptions
 *   - Drive motors are configured in devices.cpp
 *   - rot_main is geared 1:1 relative to the wheel
 *   - Wheel diameter is 2.75 in
 *
 * \par Tuning map
 *   - drive_straight_mm:
 *     - base_pct: overall speed. Lower for accuracy
 *     - kP_heading: heading correction. Higher fixes drift, too high oscillates
 *     - slow_down_mm: braking distance. Larger = smoother stop, less overshoot
 *     - soft_settle_ms: coast settle time after brake pulse
 *     - brake_pulse_ms: short brake pulse to stop clean without HOLD
 *     - end_brake: final brake mode after settling
 *   - turn_imu_deg_2stage:
 *     - fast_pct: stage 1 speed
 *     - slow_pct: stage 2 speed
 *     - split: how much of the angle is done in stage 1
 *     - settle_ms: pause between stages to reduce overshoot
 */

// ============================================================================
//   Conversions and helpers
// ============================================================================

/**
 * \brief Reset drive motors and tracking rotation to 0.
 */
void reset_drive_positions();

/**
 * \brief Convert Rotation sensor degrees to millimeters traveled by the wheel.
 *
 * \param deg Rotation sensor degrees.
 * \return Linear distance in millimeters.
 */
double rot_deg_to_mm(double deg);

/**
 * \brief Convert millimeters traveled to Rotation sensor degrees.
 *
 * \param mm Linear distance in millimeters.
 * \return Rotation sensor degrees.
 */
double mm_to_rot_deg(double mm);

/**
 * \brief Compute shortest signed angle error in degrees in range [-180, 180].
 *
 * \param target Target heading in degrees [0, 360).
 * \param current Current heading in degrees [0, 360).
 * \return Signed error in degrees.
 */
double angle_error(double target, double current);

// ============================================================================
//   Motion primitives
// ============================================================================

/**
 * \brief Drive straight for a given distance using Rotation for distance and IMU for heading.
 *
 * \param dist_mm Signed distance in millimeters. Positive forward. Negative reverse.
 * \param base_pct Base speed in percent.
 * \param kP_heading Heading correction gain using IMU error.
 * \param slow_down_mm Distance window where speed scales down.
 * \param end_brake Final brake mode after settling.
 * \param soft_settle_ms Coast settle time after brake pulse.
 * \param brake_pulse_ms Brake pulse time to seat the robot.
 */
void drive_straight_mm(double dist_mm,
                       int base_pct = 50,
                       double kP_heading = 0.5,
                       double slow_down_mm = 140.0,
                       pros::motor_brake_mode_e end_brake = pros::E_MOTOR_BRAKE_BRAKE,
                       int soft_settle_ms = 80,
                       int brake_pulse_ms = 60);

/**
 * \brief Turn using IMU in two stages for better final accuracy.
 *
 * \param deg_total Signed turn in degrees. Positive left. Negative right.
 * \param fast_pct Stage 1 speed percent.
 * \param slow_pct Stage 2 speed percent.
 * \param split Fraction of angle for stage 1.
 * \param settle_ms Pause between stages.
 */
void turn_imu_deg_2stage(double deg_total,
                         int fast_pct = 35,
                         int slow_pct = 22,
                         double split = 0.92,
                         int settle_ms = 120);
