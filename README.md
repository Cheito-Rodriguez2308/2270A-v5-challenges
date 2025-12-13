# Rust-Eze, Lightning McQueen
VEX V5 Robotics Competition. Pushback 2025.

This repository contains the full PROS C++ codebase used by Rust-Eze for driver control and autonomous.

## Goals
### Match autonomous, 15 seconds
- Target: score at least 3 blocks into the center and long goal.
- Focus: repeatable scoring, low risk, fast setup.

### Autonomous Skills
- Target: build toward a 60 second routine.
- Focus: consistency first, then speed.

---

## Hardware
### Drivetrain
- Type: 6 motor drivetrain, VEX V5 green motors.
- Wheel diameter: 3.25 in.
- Motor gearset: 18:1.

### Sensors
- Inertial sensor (IMU): Port 9.
- Rotation sensor: Port 8.
- Tracking wheels: none.

### Pneumatics
- Piston 1: ADI port A.
- Piston 2: ADI port B.

---

## Software architecture
This codebase is modular. Each module isolates one job.

### `devices.(hpp|cpp)`
Defines all motors, sensors, controller, and pneumatics in one place.
- Update ports here if wiring changes.
- Runs motor configuration in `configure_motors()`.
- Seeds randomness in `initialize_random_seed()`.

### `config.(hpp|cpp)`
All tuning lives here.
- Driver limits, deadbands, sensitivity, slew.
- Autonomous base percents.
- Subsystem percents.
- Voltage compensation helpers.

### `control.(hpp|cpp)`
Driver control logic.
- Arcade mixing.
- Turbo toggle.
- Precision mode on hold.
- Pistons.
- Subsystems.
- LCD HUD.

### `motion.(hpp|cpp)`
Autonomous motion primitives.
- `drive_straight_mm`: Rotation sensor distance with IMU heading hold.
- `turn_imu_deg_2stage`: Two-stage IMU turn for fast approach and clean finish.
- Uses slew on RPM and a soft settle plus brake pulse to stop clean.

### `auton.(hpp|cpp)`
Full match autonomous routines.
- Uses only motion primitives.
- Implements Right and Left autonomous.
- Applies voltage compensation to reduce battery-related drift.

### `odom.(hpp|cpp)`
Lightweight odometry.
- Uses Rotation sensor for distance and IMU for heading.
- Tracks pose `(x, y, theta)`.
- Runs as a task.
- Current status: enabled for telemetry and future expansion.

### `main.cpp`
Entry point.
- Initializes LCD.
- Calls `configure_motors()` and `initialize_random_seed()`.
- Starts tasks:
  - Odometry task
  - Safety brake task
  - IMU recalibration button task
- Implements competition selection:
  - D-pad RIGHT selects Right auton.
  - D-pad LEFT selects Left auton.
  - D-pad DOWN recalibrates IMU and resets odometry.

---

## Controls
### Drive
- Forward and reverse: Left stick Y-axis.
- Turn: Right stick X-axis.

### Modes
- Precision mode: hold `X`.
- Turbo toggle: D-pad `RIGHT`.

### Intake
From `control.cpp`:
- Intake forward: `L1`
- Intake reverse: `L2`

### Conveyor
From `control.cpp`:
- Conveyor forward: `R1`
- Conveyor reverse: `R2`

### Pneumatics
- Piston 1 toggle: `A`
- Piston 2 hold: D-pad `LEFT`

### Safety
- Hold `Y` to brake the drivetrain.

### Recalibration
- D-pad `DOWN` recalibrates IMU and resets odometry.
- Available in `competition_initialize()` and via the background button task.

---

## Autonomous design choices
### What runs the auton
- Straight driving uses:
  - Rotation sensor distance.
  - IMU heading hold.
- Turning uses:
  - IMU only, two stages.

### Why this approach
- Fewer moving parts than full path following.
- Easier to tune under time pressure.
- Higher reliability for match auton.

### Battery compensation
Auton drive and turn percents are scaled to reduce variation as voltage drops.

---

## What we are not using yet
These files exist for advanced control, but they are not driving the robot right now.

- `pid.hpp`
- `feedforward.hpp`
- `kinematics.hpp`
- `ramsete.hpp`

Current autonomous does not call them. The robot uses the motion primitives in `motion.cpp`.

---

## Build and deploy
### Requirements
- PROS toolchain installed.
- Project opened in your editor of choice.

### Typical workflow
- Build.
- Upload to brain.
- Test in small segments.
- Retune only from `config.cpp` unless you are changing motion logic.

---

## Tuning policy
### Safe to change often
- `cfg.AUTO_DRIVE_PCT`
- `cfg.AUTO_TURN_PCT`
- `cfg.DEADBAND_*`
- `cfg.SENSITIVITY_*`
- `cfg.SLEW_*`
- Intake and conveyor percents

### Change only with testing time
- Motion gains like heading `kP_heading`.
- Slow-down distances.
- Any constants in `motion.cpp`.

---

## Testing plan
Goal: maximum consistency.

Suggested routine each test day:
1. Recalibrate IMU on the field.
2. Run Right auton 5 times. Record success count and failure mode.
3. Run Left auton 5 times. Record success count and failure mode.
4. Adjust one variable at a time.
5. Lock settings for the week once results stabilize.

Metrics to track:
- Blocks scored in 15 seconds.
- Time to first score.
- Turn error and drift.
- Any stalls or overshoot.

---

## Credits
- Code: Jose Julian Rodriguez Muñoz (main programmer)
- Mentors and teachers: Carmen Noble, Luisette Gonzalez

---
`