#include "main.h"
#include "devices.hpp"
#include "config.hpp"
#include "control.hpp"
#include "drive.hpp"
#include "auton.hpp"
#include "motion.hpp"
#include "odom.hpp"
#include "test_auton.hpp"

// MODULO: main
// - Conecta PROS con tus modulos
// - Llama configure_motors e initialize_random_seed
// - Delega autonomous y opcontrol

// Referencia global de odometria
extern Odometry odom;

// Calibracion centralizada de IMU y odometria
void calibrate_imu_and_odom() {
  imu_main.reset();
  while (imu_main.is_calibrating()) {
    pros::delay(20);
  }

  // Referencia de odometria en (0, 0, 0 rad)
  odom.reset(0.0, 0.0, 0.0);
}

// Tarea de seguridad - mientras mantienes Y, frena el drive
void safety_task() {
  while (true) {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      lf.brake();
      lm.brake();
      lb.brake();
      rf.brake();
      rm.brake();
      rb.brake();
    }
    pros::delay(50);
  }
}

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Init...");

  configure_motors();
  initialize_random_seed();

  // Calibrar IMU una sola vez al arrancar
  static bool imu_ready = false;
  if (!imu_ready) {
    imu_main.reset();                 // calibra una vez al arrancar el programa
    while (imu_main.is_calibrating()) {
      pros::delay(20);
    }
    imu_ready = true;
  }
  // Iniciar tareas
  pros::Task safetyTask(safety_task, "Safety Task");
  pros::Task odomTask(odom_task_fn, "Odom Task");

  pros::lcd::set_text(0, "Init OK");
}

void disabled() {
}

void competition_initialize() {
  // Loop de seleccion mientras el robot esta deshabilitado
  while (pros::competition::is_disabled() &&
         !pros::competition::is_autonomous()) {

    // DOWN -> recalibrar IMU y odometria
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::lcd::print(0, "Recalibrando IMU...");
      calibrate_imu_and_odom();
      pros::lcd::print(0, "IMU listo");
    }

    // D pad RIGHT -> auton derecho
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      g_auton_selected = AutonId::Right;
    }

    // D pad LEFT -> auton izquierdo
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      g_auton_selected = AutonId::Left;
    }

    // Mostrar seleccion
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "Auton: %s", auton_name(g_auton_selected));

    master.clear_line(0);
    master.set_text(0, 0, auton_name(g_auton_selected));

    pros::delay(20);
  }
}

void autonomous() {
  autonomous_routine();
}

void opcontrol() {
  // Loop principal del driver
  driver_control_loop();
  // Si quieres probar trayectorias o tests, puedes llamar:
  // test_autonomous();
}
