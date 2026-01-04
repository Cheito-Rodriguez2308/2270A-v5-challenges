#pragma once

#include "api.h"

/**
 * \file devices.hpp
 *
 * \brief Central device declarations for the robot.
 *
 * \details This module owns the global objects that represent the hardware:
 * - Controller
 * - Drive motors
 * - Subsystem motors
 * - Pneumatics
 * - Sensors
 *
 * \par Requisites
 * - PROS kernel and headers available through `api.h`.
 *
 * \par Instructions
 * 1. Include this header anywhere you need hardware access.
 * 2. Call `configure_motors()` once inside `initialize()`.
 * 3. Call `initialize_random_seed()` once inside `initialize()`.
 *
 * \note Ports and object instantiation live in `devices.cpp`.
 */

// ============================================================================
//   _____           _
//  |  __ \         (_)
//  | |  | | _____  ___  ___ ___
//  | |  | |/ _ \ \/ / |/ __/ _ \
//  | |__| |  __/>  <| | (_|  __/
//  |_____/ \___/_/\_\_|\___\___|
//
// ============================================================================

//> Main controller (driver station)
extern pros::Controller master;

//> Drive motors
extern pros::Motor lf;
extern pros::Motor lm;
extern pros::Motor lb;

extern pros::Motor rf;
extern pros::Motor rm;
extern pros::Motor rb;

//> Subsystems
extern pros::Motor intake;
extern pros::Motor conveyor;

extern pros::adi::DigitalOut piston_1;
extern pros::adi::DigitalOut piston_2;
extern pros::adi::DigitalOut piston_3;

//> Sensors
extern pros::Imu      imu_main;
extern pros::Rotation rot_main;

// ============================================================================
//   ______                _   _
//  |  ____|              | | (_)
//  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
//  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
//  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
//  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//
// ============================================================================

/**
 * \brief Configure motor gearsets, reversals, and encoder units.
 *
 * \details Call this once at program start.
 *
 * \note This does not start or calibrate sensors.
 */
void configure_motors();

/**
 * \brief Seed the C RNG using battery and time.
 *
 * \details Uses `pros::millis()`, battery voltage, and battery current.
 * This helps break deterministic randomness across boots.
 *
 * \note Call this once inside `initialize()`.
 */
void initialize_random_seed();
