
#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "inverse_kinematics.h"

#define PCA9685_RIGHT_ADDR 0x41
#define PCA9685_LEFT_ADDR  0x40
#define SERVO_FREQ 50

const int DEFAULT_TRANSITION_TIME = 300;
const int TRANSITION_STEPS        = 20;
const float SAFETY_MARGIN_DEG     = 5.0;

float currentServoPosition[18] = {0};

struct ServoCalibration {
  int   min_us;
  int   neutral_us;
  int   max_us;
  int   direction;
  float min_angle_deg;
  float max_angle_deg;
};

const ServoCalibration SERVO_CAL[18] = {
  // COXA (0-5)
  {1400, 1800, 2200,  1, -60,  60},  // noga 0 FR
  { 900, 1200, 1400,  1, -60,  60},  // noga 1 MR
  { 900, 1200, 1500,  1, -60,  60},  // noga 2 RR
  {1500, 1800, 2100, -1, -60,  60},  // noga 3 RL
  {1500, 1780, 2000, -1, -60,  60},  // noga 4 ML
  { 900, 1200, 1500, -1, -60,  60},  // noga 5 FL

  // FEMUR (6-11)
  {1100, 1550, 1750, -1, -90,  30},  // noga 0 FR
  { 800, 1400, 2000, -1, -90,  30},  // noga 1 MR
  { 800, 1250, 2000, -1, -90,  30},  // noga 2 RR
  {1000, 1730, 2100,  1, -90,  30},  // noga 3 RL
  {1000, 1700, 2100,  1, -90,  30},  // noga 4 ML
  {1000, 1730, 2100,  1, -90,  30},  // noga 5 FL

  // TIBIA (12-17)
  {1500, 1700, 2100,  1, -140, 30},  // noga 0 FR
  {1400, 1700, 2000,  1, -140, 30},  // noga 1 MR
  {1500, 1700, 1900,  1, -140, 30},  // noga 2 RR
  { 900, 1300, 1500, -1, -140, 30},  // noga 3 RL
  { 900, 1300, 1500, -1, -140, 30},  // noga 4 ML
  { 900, 1200, 1500, -1, -140, 30}   // noga 5 FL
};

const int LEG_TO_SERVO[6][3] = {
  {0,  6, 12},  // noga 0 FR
  {1,  7, 13},  // noga 1 MR
  {2,  8, 14},  // noga 2 RR
  {3,  9, 15},  // noga 3 RL
  {4, 10, 16},  // noga 4 ML
  {5, 11, 17}   // noga 5 FL
};

struct ServoMapping { int pca_side; int channel; };

const ServoMapping SERVO_MAP[18] = {
  {0,0},{0,1},{0,2},{1,0},{1,1},{1,2},  // COXA
  {0,3},{0,4},{0,5},{1,3},{1,4},{1,5},  // FEMUR
  {0,6},{0,7},{0,8},{1,6},{1,7},{1,8}   // TIBIA
};

Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(PCA9685_RIGHT_ADDR);
Adafruit_PWMServoDriver pwmLeft  = Adafruit_PWMServoDriver(PCA9685_LEFT_ADDR);

int usToPWM(int us) {
  return (int)((float)us / 20000.0f * 4096.0f);
}

/*
 * ISPRAVLJENA mapDegToUs — koristi stvarni kutni raspon servoa
 *
 * STARO: dijelilo uvijek s hardkodiranih 90 stupnjeva
 *   Femur maxDeg=30: ratio = 24/90 = 0.27 -> servo se pomakne 27% raspona
 *
 * NOVO: dijeli s kalibiranim minDeg/maxDeg
 *   Femur maxDeg=30: ratio = 24/30 = 0.80 -> servo se pomakne 80% raspona
 *   Rezultat: 3x veci fizicki pomak za isti IK kut
 */
int mapDegToUs(float deg, int minUs, int neutralUs, int maxUs,
               float minDeg, float maxDeg) {
  int us;
  if (deg < 0.0f) {
    float ratio = deg / minDeg;  // minDeg negativan, ratio 0..1
    us = (int)(neutralUs + ratio * (minUs - neutralUs));
  } else {
    float ratio = deg / maxDeg;  // maxDeg pozitivan, ratio 0..1
    us = (int)(neutralUs + ratio * (maxUs - neutralUs));
  }
  if (us < minUs) us = minUs;
  if (us > maxUs) us = maxUs;
  return us;
}

void setServoRaw(int servo_id, float angle_deg) {
  if (servo_id < 0 || servo_id >= 18) return;

  ServoCalibration cal = SERVO_CAL[servo_id];

  // Primijeni smjer montaze
  float adj = angle_deg * (float)cal.direction;

  // Ogranici na kalibrirani raspon (ne hardkodirano +/-90)
  if (adj < cal.min_angle_deg) adj = cal.min_angle_deg;
  if (adj > cal.max_angle_deg) adj = cal.max_angle_deg;

  // Pretvori u microsekunde koristeci stvarni kutni raspon
  int us = mapDegToUs(adj,
                      cal.min_us, cal.neutral_us, cal.max_us,
                      cal.min_angle_deg, cal.max_angle_deg);

  int pwm = usToPWM(us);

  ServoMapping map = SERVO_MAP[servo_id];
  Adafruit_PWMServoDriver* drv = (map.pca_side == 0) ? &pwmRight : &pwmLeft;
  drv->setPWM(map.channel, 0, pwm);

  currentServoPosition[servo_id] = angle_deg;
}

float easeInOutCubic(float t) {
  if (t < 0.5f) return 4.0f * t * t * t;
  float f = (2.0f * t) - 2.0f;
  return 0.5f * f * f * f + 1.0f;
}

void setServoSmooth(int servo_id, float target_deg,
                    int duration_ms = DEFAULT_TRANSITION_TIME) {
  if (servo_id < 0 || servo_id >= 18) return;
  float start_deg = currentServoPosition[servo_id];
  if (abs(target_deg - start_deg) < 1.0f) {
    setServoRaw(servo_id, target_deg);
    return;
  }
  int steps = TRANSITION_STEPS;
  int delay_ms = duration_ms / steps;
  for (int s = 0; s <= steps; s++) {
    float t = (float)s / (float)steps;
    setServoRaw(servo_id, start_deg + (target_deg - start_deg) * easeInOutCubic(t));
    delay(delay_ms);
  }
}

void setLegPosition(int leg, float x, float y, float z,
                    int duration_ms = DEFAULT_TRANSITION_TIME) {
  if (leg < 0 || leg >= 6) return;
  JointAngles ang = calculateIK(x, y, z);
  if (!ang.valid) return;

  int c = LEG_TO_SERVO[leg][0];
  int f = LEG_TO_SERVO[leg][1];
  int t = LEG_TO_SERVO[leg][2];
  int steps    = TRANSITION_STEPS;
  int delay_ms = duration_ms / steps;
  float sc = currentServoPosition[c];
  float sf = currentServoPosition[f];
  float st = currentServoPosition[t];

  for (int s = 0; s <= steps; s++) {
    float eased = easeInOutCubic((float)s / (float)steps);
    setServoRaw(c, sc + (ang.theta1 - sc) * eased);
    setServoRaw(f, sf + (ang.theta2 - sf) * eased);
    setServoRaw(t, st + (ang.theta3 - st) * eased);
    delay(delay_ms);
  }
}

void setAllServosNeutral() {
  for (int i = 0; i < 18; i++) {
    setServoRaw(i, 0.0f);
    delay(30);
  }
}

bool initServoControllers() {
  if (!pwmRight.begin() || !pwmLeft.begin()) return false;
  pwmRight.setPWMFreq(SERVO_FREQ);
  pwmLeft.setPWMFreq(SERVO_FREQ);
  delay(10);
  for (int i = 0; i < 18; i++) currentServoPosition[i] = 0.0f;
  return true;
}

#endif // SERVO_CONTROL_H
