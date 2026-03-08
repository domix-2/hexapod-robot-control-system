#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Arduino.h>
#include <math.h>

// ==================== KONSTANTE GEOMETRIJE NOGE ====================
const float COXA_LENGTH = 50.0;     
const float FEMUR_LENGTH = 84.0;    
const float TIBIA_LENGTH = 99.918;  
// Maksimalni doseg noge
const float MAX_REACH = FEMUR_LENGTH + TIBIA_LENGTH; 

// ==================== STRUKTURA ZA ZGLOBNE KUTOVE ====================
struct JointAngles {
  float theta1;  // Kut kuka (coxa) - horizontalna rotacija [°]
  float theta2;  // Kut bedra (femur) - vertikalna rotacija [°]  
  float theta3;  // Kut noge (tibia) - vertikalna rotacija [°]
  bool valid;    // Je li rješenje validno
};

// ==================== PROTOTIPI FUNKCIJA ====================
bool isPositionReachable(float x, float y, float z);
bool validateJointAngles(JointAngles angles);

// ==================== FUNKCIJE INVERZNE KINEMATIKE ====================

/*
 * Izračun inverzne kinematike za željenu poziciju stopala
 */
JointAngles calculateIK(float x, float y, float z) {
  JointAngles angles;
  angles.valid = false;

  // Provjera je li pozicija dosegnuta
  if (!isPositionReachable(x, y, z)) {
    return angles;
  }

  // Theta1 - kut kuka (horizontalni)
  angles.theta1 = atan2(y, x) * (180.0 / PI);

  // Horizontalna udaljenost od coxa zgloba do stopala
  float h = sqrt(x * x + y * y);

  // Udaljenost od femur zgloba do stopala
  float r = h - COXA_LENGTH;

  // Duljina između femur i tibia zglobova do stopala
  float L = sqrt(r * r + z * z);

  // Kut u trokut femur-tibia-stopalo
  float alpha = acos((FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - L * L) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));

  // Kut u trokut femur-r-stopalo
  float beta = acos((FEMUR_LENGTH * FEMUR_LENGTH + L * L - TIBIA_LENGTH * TIBIA_LENGTH) / (2 * FEMUR_LENGTH * L));

  // Kut nagiba
  float gamma = atan2(z, r);

  // Theta2 - kut bedra
  angles.theta2 = (beta + gamma) * (180.0 / PI) ;

  // Theta3 - kut noge
  angles.theta3 = 90 - (alpha * (180.0 / PI)) ;

  // Validacija kutova
  angles.valid = validateJointAngles(angles);

  return angles;
}

/*
 * Provjera je li pozicija dosegnuta
 */
bool isPositionReachable(float x, float y, float z) {
  float h = sqrt(x * x + y * y);
  float r = h - COXA_LENGTH;
  
  if (r < 0) return false;
  
  float L = sqrt(r * r + z * z);
  
  return (L <= MAX_REACH && L > 0);
}

/*
 * Ispis kutova na serijski monitor (za debugging)
 */
void printJointAngles(JointAngles angles, int legNum) {
  // Serial.print("Noga ");  // Comment out for no serial
  // Serial.print(legNum);
  // Serial.print(": ");
  
  if (!angles.valid) {
    // Serial.println("NEVALIDAN POLOŽAJ");
    return;
  }
  
  // Serial.print("θ1(kuk)=");
  // Serial.print(angles.theta1, 2);
  // Serial.print("° | θ2(bedro)=");
  // Serial.print(angles.theta2, 2);
  // Serial.print("° | θ3(noga)=");
  // Serial.print(angles.theta3, 2);
  // Serial.println("°");
}

/*
 * Validacija kutova - provjera su li unutar fizičkih granica servo motora
 */
bool validateJointAngles(JointAngles angles) {
  // Wider, more realistic limits for MG996R servos
  const float THETA1_MIN = -90.0;
  const float THETA1_MAX = 90.0;
  
  const float THETA2_MIN = -90.0;   // Allow more downward bend
  const float THETA2_MAX = 90.0;
  
  const float THETA3_MIN = -150.0;  // Allow deeper knee bend
  const float THETA3_MAX = 30.0;    // Allow some extension

  if (angles.theta1 < THETA1_MIN || angles.theta1 > THETA1_MAX) return false;
  if (angles.theta2 < THETA2_MIN || angles.theta2 > THETA2_MAX) return false;
  if (angles.theta3 < THETA3_MIN || angles.theta3 > THETA3_MAX) return false;

  return true;
}

#endif // INVERSE_KINEMATICS_H