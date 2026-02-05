#include "auton.hpp"

namespace auton_kevin {

static Mode g_mode = Mode::MATCH_IZQ;

void set_mode(Mode m) { g_mode = m; }
Mode get_mode() { return g_mode; }

void autonomous_routine_kevin() {
  switch (g_mode) {
    case Mode::MATCH_IZQ: routine_match_izq(); break;
    case Mode::MATCH_DER: routine_match_der(); break;
    case Mode::SKILLS:    routine_skills();    break;
    default:              routine_match_izq(); break;
  }
}

} // namespace auton_kevin
