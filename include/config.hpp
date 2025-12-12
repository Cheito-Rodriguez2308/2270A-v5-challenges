#pragma once

// ======================================================
// MODULO: config
// Responsable de:
// - Centralizar TODOS los valores de tuning del robot
// - Evitar "magic numbers" regados por el codigo
// - Dar una guia clara de que tocar y que no
// ======================================================

struct Cfg {
  // Driver velocidad normal
  int DRIVE_MAX_PCT;
  int TURN_MAX_PCT;

  // Driver turbo
  int TURBO_DRIVE_MAX_PCT;
  int TURBO_TURN_MAX_PCT;

  // Autonomo
  int AUTO_DRIVE_PCT;
  int AUTO_TURN_PCT;

  // Deadbands
  int DEADBAND_FWD;
  int DEADBAND_TURN;

  // Curvas de sensibilidad
  double SENSITIVITY_SOFT;
  double SENSITIVITY_TURBO;

  // Slew rate
  double SLEW_PCT_PER_20MS;
  double SLEW_TURN_PER_20MS;
  double SLEW_PCT_PER_20MS_TURBO;
  double SLEW_TURN_PER_20MS_TURBO;

  // Pivot
  int MIN_TURN_START;
  int PIVOT_FWD_DEADBAND;

  // Subsistemas
  int INTAKE_FWD_PCT;
  int INTAKE_REV_PCT;
  int CONV_PCT;

  // Estado inicial
  bool PISTON_DEFAULT;
  bool LEFT_PREV;
};

// Config global
extern const Cfg cfg;

// Config de auton de mas alto nivel
struct AutonCfg {
  int DRIVE_FAST_PCT;
  int DRIVE_PRECISE_PCT;
  int TURN_FAST_PCT;
  int TURN_PRECISE_PCT;

  double TILE_MM;
  double TO_LOADER_MM;
  double TO_NEAR_LONG_MM;
  double ALONG_LONG_STEP_MM;
  double TO_CENTER_GOAL_MM;

  int FEED_TIME_MS;
  int INTAKE_SPINUP_MS;
};

extern const AutonCfg autonCfg;

// Constantes geometricas para modelos avanzados

// Diametro de la rueda de tracking en milimetros
constexpr double WHEEL_DIAM_MM  = 82.55;
constexpr double WHEEL_CIRC_MM  = WHEEL_DIAM_MM * 3.1415926535;
constexpr double WHEEL_CIRC_M = WHEEL_CIRC_MM / 1000.0;

// Ancho entre centros de ruedas en milimetros
constexpr double TRACK_WIDTH_MM = 320.0;

// Periodo de odometria
constexpr int    ODOM_PERIOD_MS = 10;
constexpr double ODOM_PERIOD_S  = ODOM_PERIOD_MS / 1000.0;

// Limites de velocidad
constexpr double MAX_LINEAR_V_MPS  = 1.5;
constexpr double MAX_ANGULAR_V_RPS = 6.0;

// PID por defecto (si los usas en otro sitio)
constexpr double DRIVE_KP = 1.0;
constexpr double DRIVE_KI = 0.0;
constexpr double DRIVE_KD = 0.1;

// Feedforward voltios = kS + kV v + kA a
constexpr double FF_kS = 0.2;
constexpr double FF_kV = 2.0;
constexpr double FF_kA = 0.3;

// Parametros Ramsete
constexpr double RAMSETE_B    = 2.0;
constexpr double RAMSETE_ZETA = 0.7;

// Helpers generales

int clamp_pct(int v);
int slew_step(int target, int current, int step);
double AnalogInputScaling(double x, double sens);
double shape_axis(double raw, double sens, double deadband);
int apply_slew(int target, int& state, int step);

double get_voltage();
double voltage_comp();
