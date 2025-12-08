#pragma once

// Identificadores de autonomos
enum class AutonId : int {
  Right,
  Left
};

extern AutonId g_auton_selected;

const char* auton_name(AutonId id);

void autonomous_routine();

void auton_right();
void auton_left();
