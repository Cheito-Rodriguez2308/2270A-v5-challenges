#pragma once

#include "api.h"
#include "config.hpp"
#include "devices.hpp"
#include "drive.hpp"

// TUNING DRIVER CONTROL
// - Usa cfg.* para cambiar feeling sin tocar logica

void pre_auton();

// Despachador de auton (definido en auton.cpp)
void autonomous_routine();

// Bucle principal de teleoperado
void driver_control_loop();
