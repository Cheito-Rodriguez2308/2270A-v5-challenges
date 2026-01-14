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
  Left,
  Auton_Kevin_IZQ,
  Auton_Kevin_SKILLS_IZQ
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
 * auton_kevin
 *
 * Drop-in replacement for your auton.cpp interface.
 *
 * main.cpp change:
 *   - replace autonomous_routine() with autonomous_routine_kevin()
 */

namespace auton_kevin {

// Which routine to run inside autonomous_routine_kevin()
enum class Mode {
  MATCH_IZQ,        // from auton_izq
  SKILLS_IZQ        // from auton_skills_izq
};

// Set from GUI or from competition_initialize()
void set_mode(Mode m);
Mode get_mode();

// This is the function name you will call from PROS main.cpp
void autonomous_routine_kevin();

// Optional direct entry points
void routine_match_izq();
void routine_skills_izq();

} // namespace auton_kevin
