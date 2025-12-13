#include "main.h"
#include "devices.hpp"
#include "config.hpp"
#include "control.hpp"
#include "drive.hpp"
#include "auton.hpp"
#include "motion.hpp"
#include "odom.hpp"
#include "test_auton.hpp"

/**
 * \file main.cpp
 *
 * \brief PROS entry points. Wires modules together.
 *
 * \par Responsibilities
 *   - Initialize LCD
 *   - Configure motors and random seed
 *   - Start background tasks (odometry, safety, IMU button)
 *   - Provide competition hooks: initialize, disabled, comp init, auton, opcontrol
 *
 * \par Startup flow
 *   1. initialize()
 *      - configure_motors()
 *      - initialize_random_seed()
 *      - start tasks
 *   2. competition_initialize()
 *      - while disabled: select auton and optional IMU recalibration
 *   3. autonomous()
 *      - autonomous_routine()
 *   4. opcontrol()
 *      - driver_control_loop()
 *
 * \par IMU calibration policy
 *   - Recalibration is manual via controller DOWN
 *   - Odometry pauses during calibration to avoid pose corruption
 *
 * \par Safety policy
 *   - Holding Y brakes the drive motors
 *   - Intended as an emergency stop while testing
 */

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

/**
 * \brief Global odometry instance (defined in odom.cpp).
 */
extern Odometry odom;

// -----------------------------------------------------------------------------
// IMU + Odom calibration
// -----------------------------------------------------------------------------

/**
 * \brief Calibrate IMU and reset odometry pose.
 *
 * \details
 *   - Pauses odometry task updates
 *   - Stops drive motors for safety
 *   - Resets IMU, waits until calibration ends
 *   - Resets odometry pose to origin
 *   - Resumes odometry task updates
 */
void calibrate_imu_and_odom() {
  g_odom_pause.store(true);

  // Stop drive motors for safety
  lf.move(0); lm.move(0); lb.move(0);
  rf.move(0); rm.move(0); rb.move(0);

  imu_main.reset();
  while (imu_main.is_calibrating()) {
    pros::delay(20);
  }

  // Reset odometry pose. Units: meters, radians
  odom.reset(0.0, 0.0, 0.0);

  g_odom_pause.store(false);
}

// -----------------------------------------------------------------------------
// Background tasks
// -----------------------------------------------------------------------------

/**
 * \brief Task. Manual IMU recalibration on DOWN new press.
 *
 * \details
 *   - Prints status on LCD
 *   - Calls calibrate_imu_and_odom()
 */
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

/**
 * \brief Task. Safety brake while holding Y.
 *
 * \details
 *   - Brakes all drive motors while Y is held
 *   - Useful during bench tests and debugging
 */
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

// -----------------------------------------------------------------------------
// PROS hooks
// -----------------------------------------------------------------------------

/**
 * \brief PROS initialize hook. Runs once at boot.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Init...");

  configure_motors();
  initialize_random_seed();

  // Tasks live for the program lifetime
  pros::Task imuBtnTask(imu_button_task, "IMU Button");
  pros::Task safetyTask(safety_task, "Safety Task");
  pros::Task odomTask(odom_task_fn, "Odom Task");

  pros::lcd::set_text(0, "Init OK");
}

/**
 * \brief PROS disabled hook.
 */
void disabled() {
}

/**
 * \brief PROS competition initialize hook.
 *
 * \details
 *   Runs while disabled before match start.
 *   - DOWN recalibrates IMU and odometry
 *   - RIGHT selects Right auton
 *   - LEFT selects Left auton
 *   - Prints selection on LCD and controller
 */
void competition_initialize() {
  while (pros::competition::is_disabled() &&
         !pros::competition::is_autonomous()) {

    // Manual recalibration
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::lcd::print(0, "Recalibrando IMU...");
      calibrate_imu_and_odom();
      pros::lcd::print(0, "IMU listo");
    }

    // Auton selection
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      g_auton_selected = AutonId::Right;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      g_auton_selected = AutonId::Left;
    }

    // Display selection
    pros::lcd::clear_line(0);
    pros::lcd::print(0, "Auton: %s", auton_name(g_auton_selected));

    master.clear_line(0);
    master.set_text(0, 0, auton_name(g_auton_selected));

    pros::delay(20);
  }
}

/**
 * \brief PROS autonomous hook.
 */
void autonomous() {
  autonomous_routine();
}

/**
 * \brief PROS operator control hook.
 */
void opcontrol() {
  driver_control_loop();

  // For quick tests, call test_autonomous() instead of driver loop,
  // or temporarily comment the driver loop and run tests here.
  // test_autonomous();
}
