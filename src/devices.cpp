#include "devices.hpp"
#include "api.h"


// ============================================================================
//   _____           _              _   _       _   _
//  |  __ \         (_)            | | (_)     | | (_)
//  | |  | | _____  ___  ___ ___   | |_ _  __ _| |_ _  ___  _ __  ___
//  | |  | |/ _ \ \/ / |/ __/ _ \  | __| |/ _` | __| |/ _ \| '_ \/ __|
//  | |__| |  __/>  <| | (_|  __/  | |_| | (_| | |_| | (_) | | | \__ \
//  |_____/ \___/_/\_\_|\___\___|   \__|_|\__,_|\__|_|\___/|_| |_|___/
//
// ============================================================================

//> Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//> Drive motors
pros::Motor lf(3);
pros::Motor lm(4);
pros::Motor lb(20);

pros::Motor rf(1);
pros::Motor rm(2);
pros::Motor rb(10);

//> Subsystems
pros::Motor intake(6); // outtake
pros::Motor conveyor(7);

pros::ADIDigitalOut piston_1('A');
pros::ADIDigitalOut piston_2('B');
pros::ADIDigitalOut piston_3('C');
pros::ADIDigitalOut piston_4('D');

//> Sensors
pros::Imu      imu_main(9);
pros::Rotation rot_main(8);

// ============================================================================
//   __  __       _                 _____            __ _
//  |  \/  |     | |               / ____|          / _(_)
//  | \  / | ___ | |_ ___  _ __   | |     ___  _ __| |_ _  __ _
//  | |\/| |/ _ \| __/ _ \| '__|  | |    / _ \| '__|  _| |/ _` |
//  | |  | | (_) | || (_) | |     | |___| (_) | |  | | | | (_| |
//  |_|  |_|\___/ \__\___/|_|      \_____\___/|_|  |_| |_|\__, |
//                                                         __/ |
//                                                        |___/
//
// ============================================================================

/**
 * \brief Configure all motors with gearing, reversal, and encoder units.
 *
 * \details Standardizes motor configuration so other modules assume consistent
 * units and direction. This prevents "magic config" scattered across code.
 */
void configure_motors() {
  // Left front
  lf.set_gearing(pros::E_MOTOR_GEARSET_18);
  lf.set_reversed(true);
  lf.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Left mid
  lm.set_gearing(pros::E_MOTOR_GEARSET_18);
  lm.set_reversed(false);
  lm.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Left back
  lb.set_gearing(pros::E_MOTOR_GEARSET_18);
  lb.set_reversed(true);
  lb.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Right front
  rf.set_gearing(pros::E_MOTOR_GEARSET_18);
  rf.set_reversed(false);
  rf.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Right mid
  rm.set_gearing(pros::E_MOTOR_GEARSET_18);
  rm.set_reversed(true);
  rm.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Right back
  rb.set_gearing(pros::E_MOTOR_GEARSET_18);
  rb.set_reversed(false);
  rb.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Intake
  intake.set_gearing(pros::E_MOTOR_GEARSET_18);
  intake.set_reversed(false);
  intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Conveyor
  conveyor.set_gearing(pros::E_MOTOR_GEARSET_18);
  conveyor.set_reversed(false);
  conveyor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

// ============================================================================
//   _____                 _                 _____               _
//  |  __ \               | |               / ____|             | |
//  | |__) |__ _ _ __   __| | ___  _ __ ___| (___   ___  ___  __| |
//  |  _  // _` | '_ \ / _` |/ _ \| '__/ __|\___ \ / _ \/ _ \/ _` |
//  | | \ \ (_| | | | | (_| | (_) | |  \__ \____) |  __/  __/ (_| |
//  |_|  \_\__,_|_| |_|\__,_|\___/|_|  |___/_____/ \___|\___|\__,_|
//
// ============================================================================

/**
 * \brief Seed the standard C RNG using battery and time.
 *
 * \details This is useful if you use `rand()` anywhere for non-critical random
 * behavior. The seed is built from:
 * - system time in ms
 * - battery voltage in mV
 * - battery current in mA
 *
 * \note Call once at boot. Seeding repeatedly reduces randomness quality.
 */
void initialize_random_seed() {
  const int system_time = pros::millis();
  const int battery_mv  = pros::battery::get_voltage();
  const int battery_ma  = pros::battery::get_current();

  const int seed = system_time + battery_mv + battery_ma;
  std::srand(seed);
}
