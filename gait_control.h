/*
 * GAIT_CONTROL.H - Ispravan tripod gait s lokalnim koordinatnim sustavom
 *
 * KLJUČNI KONCEPT — KOORDINATNI SUSTAV:
 * ======================================
 * IK u ovom robotu računa LOKALNE koordinate od prikvačišta svake noge:
 *   x = udaljenost od prikvačišta ravno van (uvijek ~GAIT_NEU_X)
 *   y = lijevo/desno od osi noge
 *   z = gore/dolje (negativno = prema tlu)
 *
 * Noge su montirane pod kutovima od tijela:
 *   Noga 0: +60°,  Noga 1: -60°
 *   Noga 2: +90°,  Noga 3: -90°
 *   Noga 4: +120°, Noga 5: -120°
 *
 * "Korak naprijed" (globalni +X) = pomak u Y osi u lokalnom sustavu noge.
 * Projekcija: local_y = -sin(leg_angle) * step_length
 *
 * Primjer:
 *   Noga 0 (+60°): forward local_y = -sin(60°)*40 = -34.6mm  → θ1=-12°
 *   Noga 1 (-60°): forward local_y = -sin(-60°)*40 = +34.6mm → θ1=+12°
 *   Coxa se pomiče jer θ1 = atan2(y, x) i y ≠ 0!
 */

#ifndef GAIT_CONTROL_H
#define GAIT_CONTROL_H

#include <math.h>
#include "inverse_kinematics.h"
#include "servo_control.h"

// ==================== GAIT PARAMETRI ====================
const float GAIT_NEU_X      = 160.0f;  // Udaljenost stopala od prikvačišta (mm)
const float GAIT_NEU_Z      = -70.0f;  // Visina tijela (mm)

const float STEP_LENGTH     = 45.0f;   // Duljina koraka (mm)
const float STEP_HEIGHT     = 35.0f;   // Visina podizanja noge (mm)
const float TURN_STEP       = 40.0f;   // Pomak za okretanje (mm)

const int   GAIT_STEPS      = 30;      // Koraka interpolacije po fazi
const int   GAIT_STEP_DELAY = 20;      // ms po koraku (20×12 = 240ms po fazi)

// ==================== KUTOVI MONTAŽE NOGU ====================
const float LEG_MOUNT_ANGLE[6] = {
   60.0f,   // Noga 0: prednja lijeva
  -60.0f,   // Noga 1: prednja desna
   90.0f,   // Noga 2: srednja lijeva
  -90.0f,   // Noga 3: srednja desna
  120.0f,   // Noga 4: zadnja lijeva
  -120.0f   // Noga 5: zadnja desna
};

const int GROUP_A[3] = {0, 2, 4};
const int GROUP_B[3] = {1, 3, 5};

// Praćenje lokalnih Y pozicija stopala
float legLocalY[6] = {0, 0, 0, 0, 0, 0};

// ==================== SMJERNI VEKTORI PO NOZI ====================

// Koliko se lokalni Y pomiče kada tijelo ide naprijed za 1mm
// = -sin(leg_angle)  [projekcija globalnog (1,0) na lokalnu normalnu os]
float legForwardDir(int leg) {
  return -sinf(LEG_MOUNT_ANGLE[leg] * (float)M_PI / 180.0f);
}

// Koliko se lokalni Y pomiče za okretanje
// Lijeve noge (pozitivni kut montaže) → +Y za lijevo
// Desne noge (negativni kut montaže)  → -Y za lijevo
// = sign(leg_angle) * direction
float legTurnDir(int leg, int direction) {
  return (LEG_MOUNT_ANGLE[leg] > 0 ? 1.0f : -1.0f) * direction;
}

// ==================== POSTAVLJANJE JEDNE NOGE (bez delay) ====================
void setLegRaw(int leg, float x, float y, float z) {
  JointAngles ang = calculateIK(x, y, z);
  if (!ang.valid) return;
  setServoRaw(LEG_TO_SERVO[leg][0], ang.theta1);
  setServoRaw(LEG_TO_SERVO[leg][1], ang.theta2);
  setServoRaw(LEG_TO_SERVO[leg][2], ang.theta3);
}

// ==================== SIMULTANO POMICANJE DVIJE GRUPE ====================
void moveBothGroups(
    const int* swingGroup,  int swingCount,
    const int* stanceGroup, int stanceCount,
    float swingTargY[],    // ciljni lokalni Y za svaku swing nogu
    float stanceTargY[])   // ciljni lokalni Y za svaku stance nogu
{
  float swingStartY[3], stanceStartY[3];
  for (int i = 0; i < swingCount;  i++) swingStartY[i]  = legLocalY[swingGroup[i]];
  for (int i = 0; i < stanceCount; i++) stanceStartY[i] = legLocalY[stanceGroup[i]];

  for (int s = 0; s <= GAIT_STEPS; s++) {
    float t = (float)s / GAIT_STEPS;

    // Swing — sinusni luk
    for (int i = 0; i < swingCount; i++) {
      int leg = swingGroup[i];
      float y = swingStartY[i] + (swingTargY[i] - swingStartY[i]) * t;
      float z = GAIT_NEU_Z + STEP_HEIGHT * sinf(t * (float)M_PI);
      setLegRaw(leg, GAIT_NEU_X, y, z);
    }

    // Stance — klizi po tlu
    for (int i = 0; i < stanceCount; i++) {
      int leg = stanceGroup[i];
      float y = stanceStartY[i] + (stanceTargY[i] - stanceStartY[i]) * t;
      setLegRaw(leg, GAIT_NEU_X, y, GAIT_NEU_Z);
    }

    delay(GAIT_STEP_DELAY);
  }

  // Ažuriraj praćenje
  for (int i = 0; i < swingCount;  i++) legLocalY[swingGroup[i]]  = swingTargY[i];
  for (int i = 0; i < stanceCount; i++) legLocalY[stanceGroup[i]] = stanceTargY[i];
}

// ==================== TRIPOD GAIT NAPRIJED ====================
void tripodGaitForward() {
  // Faza 1: A swing naprijed, B stance gura
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  =  legForwardDir(GROUP_A[i]) * STEP_LENGTH;
      stanceTargY[i] = -legForwardDir(GROUP_B[i]) * STEP_LENGTH;
    }
    moveBothGroups(GROUP_A, 3, GROUP_B, 3, swingTargY, stanceTargY);
  }
  // Faza 2: B swing naprijed, A stance gura
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  =  legForwardDir(GROUP_B[i]) * STEP_LENGTH;
      stanceTargY[i] = -legForwardDir(GROUP_A[i]) * STEP_LENGTH;
    }
    moveBothGroups(GROUP_B, 3, GROUP_A, 3, swingTargY, stanceTargY);
  }
}

// ==================== TRIPOD GAIT NAZAD ====================
void tripodGaitBackward() {
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  = -legForwardDir(GROUP_A[i]) * STEP_LENGTH;
      stanceTargY[i] =  legForwardDir(GROUP_B[i]) * STEP_LENGTH;
    }
    moveBothGroups(GROUP_A, 3, GROUP_B, 3, swingTargY, stanceTargY);
  }
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  = -legForwardDir(GROUP_B[i]) * STEP_LENGTH;
      stanceTargY[i] =  legForwardDir(GROUP_A[i]) * STEP_LENGTH;
    }
    moveBothGroups(GROUP_B, 3, GROUP_A, 3, swingTargY, stanceTargY);
  }
}

// ==================== OKRETANJE LIJEVO ====================
void tripodGaitTurnLeft() {
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  =  legTurnDir(GROUP_A[i], +1) * TURN_STEP;
      stanceTargY[i] = -legTurnDir(GROUP_B[i], +1) * TURN_STEP;
    }
    moveBothGroups(GROUP_A, 3, GROUP_B, 3, swingTargY, stanceTargY);
  }
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  =  legTurnDir(GROUP_B[i], +1) * TURN_STEP;
      stanceTargY[i] = -legTurnDir(GROUP_A[i], +1) * TURN_STEP;
    }
    moveBothGroups(GROUP_B, 3, GROUP_A, 3, swingTargY, stanceTargY);
  }
}

// ==================== OKRETANJE DESNO ====================
void tripodGaitTurnRight() {
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  =  legTurnDir(GROUP_A[i], -1) * TURN_STEP;
      stanceTargY[i] = -legTurnDir(GROUP_B[i], -1) * TURN_STEP;
    }
    moveBothGroups(GROUP_A, 3, GROUP_B, 3, swingTargY, stanceTargY);
  }
  {
    float swingTargY[3], stanceTargY[3];
    for (int i = 0; i < 3; i++) {
      swingTargY[i]  =  legTurnDir(GROUP_B[i], -1) * TURN_STEP;
      stanceTargY[i] = -legTurnDir(GROUP_A[i], -1) * TURN_STEP;
    }
    moveBothGroups(GROUP_B, 3, GROUP_A, 3, swingTargY, stanceTargY);
  }
}

// ==================== RESET ====================
void resetGaitState() {
  for (int i = 0; i < 6; i++) legLocalY[i] = 0.0f;
}

#endif // GAIT_CONTROL_H