#pragma once

#include "api.h"
#include "devices.hpp"

/**
 * \file drive.hpp
 *
 * \brief Drive wrapper for tank chassis control.
 *
 * \par Purpose
 *   - Centralize all drive related actions in one class
 *   - Avoid repeating motor group code across the project
 *   - Expose simple primitives used by auton and driver control
 *
 * \par What to tune
 *   - This module is older in your codebase. Your newer auton uses motion.cpp
 *   - If you still use this drive module:
 *       - kWheelTravelMm in drive.cpp must match your wheel diameter
 *       - TIMEOUT and stall constants inside drive_straight_mm and turns
 *       - kp_mm for straight correction
 *
 * \par Notes
 *   - set_percent expects values in percent [-100, 100]
 *   - brake mode affects stopping feel. COAST feels more human. BRAKE/HOLD lock.
 */

class Drive {
public:
  /**
   * \brief Reset integrated motor encoders for all 6 drive motors.
   *
   * \note This resets motor positions only. It does not reset IMU or rotation sensor.
   */
  void reset_encoders();

  /**
   * \brief Read left reference motor position in degrees.
   *
   * \return Motor encoder position in degrees.
   */
  double front_left_deg() const;

  /**
   * \brief Read right reference motor position in degrees.
   *
   * \return Motor encoder position in degrees.
   */
  double front_right_deg() const;

  /**
   * \brief Set brake mode for all 6 drive motors.
   *
   * \param mode pros brake mode, COAST, BRAKE, or HOLD.
   */
  void set_brake(pros::motor_brake_mode_e_t mode);

  /**
   * \brief Apply open loop tank power to the chassis.
   *
   * \param left_pct  Left side power in percent [-100, 100]
   * \param right_pct Right side power in percent [-100, 100]
   *
   * \note Internally converts percent to motor power [-127, 127].
   */
  void set_percent(int left_pct, int right_pct);

  /**
   * \brief Convert motor degrees to linear millimeters using wheel travel constant.
   */
  static double deg_to_mm(double deg);

  /**
   * \brief Convert linear millimeters to motor degrees using wheel travel constant.
   */
  static double mm_to_deg(double mm);

  /**
   * \brief Drive straight for a target distance using motor encoders.
   *
   * \details
   *   - Uses front left and front right motor encoders as distance reference
   *   - Applies trim based on left minus right distance
   *   - Slows down near the target to reduce overshoot
   *   - Includes stall detection and timeout
   *
   * \param dist_mm      Signed distance in mm. Positive forward, negative reverse.
   * \param base_pct     Base power in percent.
   * \param kP_mm        Correction gain for left right error in mm.
   * \param slow_down_mm Distance window in mm for slowdown.
   * \param end_brake    Brake mode used during the move.
   */
  void drive_straight_mm(
    double dist_mm,
    int base_pct,
    double kP_mm,
    double slow_down_mm,
    pros::motor_brake_mode_e_t end_brake
  );

  /**
   * \brief Turn right by an angle using encoder based arc length, two stage.
   *
   * \param deg_total Positive degrees to turn right.
   * \param fast_pct  Power for the first stage.
   * \param slow_pct  Power for the second stage.
   */
  void turn_right_deg(double deg_total, int fast_pct, int slow_pct);

  /**
   * \brief Turn left by an angle using encoder based arc length, two stage.
   *
   * \param deg_total Positive degrees to turn left.
   * \param fast_pct  Power for the first stage.
   * \param slow_pct  Power for the second stage.
   */
  void turn_left_deg(double deg_total, int fast_pct, int slow_pct);
};

extern Drive drive;
