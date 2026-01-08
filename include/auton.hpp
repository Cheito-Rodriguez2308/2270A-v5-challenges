#pragma once

/**
 * \file auton.hpp
 *
 * \brief Autonomous interface and selector.
 *
 * \par What lives here
 *   - AutonId enum
 *   - Global auton selector variable
 *   - Public auton entry points used by PROS competition template
 *
 * \par Usage
 *   1. Set g_auton_selected somewhere in your UI or pre_auton
 *   2. Call autonomous_routine() from PROS autonomous()
 */

/**
 * \enum AutonId
 *
 * \brief Identifiers for autonomous routines.
 */
enum class AutonId : int {
  Right,
  Left
};

// Selected auton (default set in auton.cpp)
extern AutonId g_auton_selected;

/**
 * \brief Human readable name for HUD and debugging.
 */
const char* auton_name(AutonId id);

/**
 * \brief Main dispatcher for autonomous.
 */
void autonomous_routine();

/**
 * \brief Right side auton routine.
 */
void auton_right();

/**
 * \brief Left side auton routine.
 */
void auton_left();

/**
 * \brief Skills 60 sec autonomous
*/
void auton_skills_60_left();
