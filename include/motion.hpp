#pragma once

#include "api.h"

// ======================================================
// MODULO: motion
//
// PROPOSITO
//   Proveer primitivas simples y robustas para auton:
//     - drive_mm_pid: avanzar recto una distancia en mm.
//     - turn_imu_deg: girar un angulo especifico en grados.
//
// DISEÑO
//   - Rectas usan distancia total de odometria (Rotation).
//   - Giros usan heading absoluto del IMU.
//   - Ninguna otra funcion modifica sensores.
// ======================================================

// Movimiento recto en mm usando odometria.
// mm_target  -> distancia objetivo en milimetros (positivo adelante).
void drive_mm_pid(double mm_target);

// Giro relativo en grados usando IMU.
// delta_deg > 0  -> giro a la izquierda.
// delta_deg < 0  -> giro a la derecha.
void turn_imu_deg(double delta_deg);
