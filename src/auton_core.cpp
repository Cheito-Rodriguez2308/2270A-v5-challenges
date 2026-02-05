#include "auton.hpp"

// Default auton selection used by GUI and competition_initialize
AutonId g_auton_selected = AutonId::Auton_Kevin_DER;

const char* auton_name(AutonId id) {
  switch (id) {
    case AutonId::Right:             return "Right";
    case AutonId::Left:              return "Left";
    case AutonId::Auton_Kevin_IZQ:   return "Kevin Match IZQ";
    case AutonId::Auton_Kevin_DER:   return "Kevin Match DER";
    case AutonId::Auton_Kevin_SKILLS:return "Kevin Skills";
    default:                         return "Unknown";
  }
}

// If your code still calls autonomous_routine(), keep a bridge.
// If you only use autonomous_routine_kevin(), you can remove this.
void autonomous_routine() {
  // Map AutonId into Kevin Mode
  using auton_kevin::Mode;

  switch (g_auton_selected) {
    case AutonId::Auton_Kevin_IZQ:
      auton_kevin::set_mode(Mode::MATCH_IZQ);
      break;

    case AutonId::Auton_Kevin_DER:
      auton_kevin::set_mode(Mode::MATCH_DER);
      break;

    case AutonId::Auton_Kevin_SKILLS:
      auton_kevin::set_mode(Mode::SKILLS);
      break;

    // Legacy
    case AutonId::Right:
      auton_right();
      return;

    case AutonId::Left:
      auton_left();
      return;

    default:
      auton_kevin::set_mode(Mode::MATCH_DER);
      break;
  }

  auton_kevin::autonomous_routine_kevin();
}

// Optional. Keep these if main.cpp or other code calls them.
// If you do not have implementations, at least provide safe stubs.
void auton_right() {
  // TODO: implement or redirect
  auton_kevin::set_mode(auton_kevin::Mode::MATCH_DER);
  auton_kevin::autonomous_routine_kevin();
}

void auton_left() {
  // TODO: implement or redirect
  auton_kevin::set_mode(auton_kevin::Mode::MATCH_IZQ);
  auton_kevin::autonomous_routine_kevin();
}
