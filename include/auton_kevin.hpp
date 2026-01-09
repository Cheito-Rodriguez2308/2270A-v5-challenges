#pragma once

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
