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
  g_odom_pause.store(true);

  // Detener drive por seguridad
  lf.move(0); lm.move(0); lb.move(0);
  rf.move(0); rm.move(0); rb.move(0);

  imu_main.reset();
  while (imu_main.is_calibrating()) {
    pros::delay(20);
  }

  odom.reset(0.0, 0.0, 0.0);

  g_odom_pause.store(false);
}

// Tarea para recalibrar IMU con boton DOWN
void imu_button_task() {
  while (true) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::lcd::print(0, "Recalibrando IMU...");
      calibrate_imu_and_odom();
      pros::lcd::print(0, "IMU listo");
    }
    pros::delay(20);
  }
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

  pros::Task imuBtnTask(imu_button_task, "IMU Button");

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
