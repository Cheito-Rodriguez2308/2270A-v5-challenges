#pragma once

#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "drive.hpp"

/**
 * \file control.hpp
 *
 * \brief Driver control and match flow entry points.
 *
 * \par Purpose
 *   - pre_auton() sets safe initial states before match start
 *   - autonomous_routine() is called by PROS during auton period
 *   - driver_control_loop() runs the full teleop loop
 *
 * \par What to tune
 *   - cfg.DRIVE_MAX_PCT, cfg.TURN_MAX_PCT
 *   - cfg.TURBO_* limits
 *   - cfg.SENSITIVITY_* (axis shaping)
 *   - cfg.SLEW_* (ramp rate)
 *   - cfg.MIN_TURN_START and cfg.PIVOT_FWD_DEADBAND (pivot feel)
 *
 * \par Controls
 *   - Left Stick (Y axis): forward and reverse
 *   - Right Stick (X axis): turn
 *   - RIGHT: precision hold (scales limits down)
 *   - DOWN: turbo toggle
 *   - A: piston_1 toggle
 *   - B: piston_2 hold
 *   - Y: piston_3 toggle
 *   - L1 L2: intake fwd and rev
 *   - R1 R2: conveyor fwd and rev
 */

// ------------------------------------------------------
// Match flow
// ------------------------------------------------------

/**
 * \brief Pre auton setup. Called before autonomous and driver control.
 */
void pre_auton();

/**
 * \brief Auton dispatcher. Defined in auton.cpp.
 */
void autonomous_routine();

/**
 * \brief Main teleop loop. Blocks forever.
 */
void driver_control_loop();
