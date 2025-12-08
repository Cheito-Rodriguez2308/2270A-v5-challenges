#pragma once

#include "api.h"

// MODULO: devices
// - Declara controlador, motores, actuadores y sensores
// - Puertos definidos en devices.cpp

// Controlador principal
extern pros::Controller master;

// Motores de drive
extern pros::Motor lf;
extern pros::Motor lm;
extern pros::Motor lb;

extern pros::Motor rf;
extern pros::Motor rm;
extern pros::Motor rb;

// Subsistemas
extern pros::Motor intake;
extern pros::Motor conveyor;
extern pros::adi::DigitalOut piston_1;
extern pros::adi::DigitalOut piston_2;

// Sensores
extern pros::Imu      imu_main;
extern pros::Rotation rot_main;

// Config hardware
void configure_motors();

// Random seed
void initialize_random_seed();
