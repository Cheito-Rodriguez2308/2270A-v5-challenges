#include "devices.hpp"
#include "pros/motors.h"
#include <cstdlib>

// Controlador
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motores de drive
pros::Motor lf(11);
pros::Motor lm(4);
pros::Motor lb(20);

pros::Motor rf(1);
pros::Motor rm(19);
pros::Motor rb(10);

// Subsistemas
pros::Motor intake(6);
pros::Motor conveyor(7);
pros::adi::DigitalOut piston_1('A');
pros::adi::DigitalOut piston_2('B');

// Sensores
pros::Imu      imu_main(9);
pros::Rotation rot_main(8);

// Configura gearset, sentido y encoder
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
  intake.set_reversed(true);
  intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

  // Conveyor
  conveyor.set_gearing(pros::E_MOTOR_GEARSET_18);
  conveyor.set_reversed(false);
  conveyor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

// Random seed con bateria y tiempo
void initialize_random_seed() {
  int system_time = pros::millis();
  int battery_mv  = pros::battery::get_voltage();
  int battery_ma  = pros::battery::get_current();

  int seed = system_time + battery_mv + battery_ma;
  std::srand(seed);
}
