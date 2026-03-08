// ==================== DEBUG ====================
#define DEBUG 1

#if DEBUG
  #define DPRINT(...)   Serial.print(__VA_ARGS__)
  #define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

#include <Wire.h>
#include <Preferences.h>
#include "servo_control.h"
#include "bluetooth_control.h"
#include "inverse_kinematics.h"
#include "komunikacija_Pi5.h"
#include "imu_control.h"
#include "gait_control.h"

// ==================== IMU STATUS ====================
bool imuAvailable = false;

// ==================== STATE ====================
enum RobotState {
  STATE_IDLE,
  STATE_WALKING_FORWARD,
  STATE_WALKING_BACKWARD,
  STATE_TURNING_LEFT,
  STATE_TURNING_RIGHT,
  STATE_STANDING
};

RobotState currentState  = STATE_IDLE;
RobotState previousState = STATE_IDLE;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 5000;

// ==================== POSITION VALIDATION ====================
bool validatePosition(float x, float y, float z, const char* desc) {
  JointAngles angles = calculateIK(x, y, z);

#if DEBUG
  DPRINT("  [VAL] ");
  DPRINT(desc);
  DPRINT(" (x="); DPRINT(x, 1);
  DPRINT(" y=");  DPRINT(y, 1);
  DPRINT(" z=");  DPRINT(z, 1);
  DPRINT(") -> ");

  if (angles.valid) {
    DPRINT("OK  t1="); DPRINT(angles.theta1, 1);
    DPRINT("  t2=");   DPRINT(angles.theta2, 1);
    DPRINT("  t3=");   DPRINT(angles.theta3, 1);
    DPRINTLN("");
  } else {
    DPRINTLN("INVALID !");
  }
#endif

  return angles.valid;
}

// ==================== GAIT VALIDATION ====================
void validateGaitPositions() {
#if DEBUG
  DPRINTLN("\n========================================");
  DPRINTLN("   GAIT POSITION VALIDATION");
  DPRINTLN("========================================");
  DPRINT("  GAIT_NEU_X  = "); DPRINTLN(GAIT_NEU_X);
  DPRINT("  GAIT_NEU_Z  = "); DPRINTLN(GAIT_NEU_Z);
  DPRINT("  STEP_LENGTH = "); DPRINTLN(STEP_LENGTH);
  DPRINT("  STEP_HEIGHT = "); DPRINTLN(STEP_HEIGHT);
  DPRINT("  TURN_STEP   = "); DPRINTLN(TURN_STEP);

  float fwdDir[6], turnDir[6];
  for (int i = 0; i < 6; i++) {
    fwdDir[i]  = -sinf(LEG_MOUNT_ANGLE[i] * (float)M_PI / 180.0f);
    turnDir[i] = (LEG_MOUNT_ANGLE[i] > 0) ? 1.0f : -1.0f;
  }

  DPRINTLN("\n[1] NEUTRALNA POZICIJA:");
  validatePosition(GAIT_NEU_X, 0.0f, GAIT_NEU_Z, "sve noge");

  DPRINTLN("\n[2] FORWARD swing vrh (noga gore):");
  bool fwdOK = true;
  for (int leg = 0; leg < 6; leg++) {
    char buf[20];
    snprintf(buf, sizeof(buf), "Leg %d fwd", leg);
    float y = fwdDir[leg] * STEP_LENGTH;
    float z = GAIT_NEU_Z + STEP_HEIGHT;
    if (!validatePosition(GAIT_NEU_X, y, z, buf)) fwdOK = false;
  }
  if (!fwdOK) DPRINTLN("  !! Smanji STEP_HEIGHT ili STEP_LENGTH !!");

  DPRINTLN("\n[3] FORWARD swing slijetanje:");
  for (int leg = 0; leg < 6; leg++) {
    char buf[20];
    snprintf(buf, sizeof(buf), "Noga %d land", leg);
    validatePosition(GAIT_NEU_X, fwdDir[leg] * STEP_LENGTH, GAIT_NEU_Z, buf);
  }

  DPRINTLN("\n[4] FORWARD stance kraj (noga straga):");
  for (int leg = 0; leg < 6; leg++) {
    char buf[20];
    snprintf(buf, sizeof(buf), "Noga %d stc", leg);
    validatePosition(GAIT_NEU_X, -fwdDir[leg] * STEP_LENGTH, GAIT_NEU_Z, buf);
  }

  DPRINTLN("\n[5] TURN swing vrh:");
  for (int leg = 0; leg < 6; leg++) {
    char buf[20];
    snprintf(buf, sizeof(buf), "Noga %d turn", leg);
    float y = turnDir[leg] * TURN_STEP;
    float z = GAIT_NEU_Z + STEP_HEIGHT;
    validatePosition(GAIT_NEU_X, y, z, buf);
  }

  DPRINTLN("\n========================================");
  DPRINTLN("  OK     = pozicija dosezna, kutovi ispisani");
  DPRINTLN("  INVALID = van dosega IK-a");
  DPRINTLN("  Noga struze -> povecaj STEP_HEIGHT");
  DPRINTLN("  INVALID     -> smanji STEP_HEIGHT/STEP_LENGTH");
  DPRINTLN("========================================\n");
#endif
}

// ==================== FORWARD DEKLARACIJE ====================
void processCommand(String cmd);
void moveToHomePosition();
void maintainStandingPosition();

// ==================== SETUP ====================
void setup() {
#if DEBUG
  Serial.begin(115200);
  delay(500);
  DPRINTLN("===== HEXAPOD INIT =====");
#endif

  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(200);
  DPRINTLN("I2C @ 100kHz");

  // IMU
  DPRINTLN("\n--- IMU Inicijalizacija ---");
  if (!initIMU()) {
    DPRINTLN("IMU nije dostupan - nastavljam bez njega");
    imuAvailable = false;
  } else {
    if (!loadGyroOffsets()) {
      DPRINTLN("Nema offseta - radim kalibraciju");
      calibrateGyro();
    }
    imuAvailable = true;
    DPRINTLN("IMU aktivan");
  }
  delay(200);

  // Servoi
  DPRINTLN("\n--- Servo Kontroleri ---");
  if (!initServoControllers()) {
    DPRINTLN("KRITICNO: Servoi ne rade!");
    while (true) { delay(1000); }
  }
  DPRINTLN("Servoi OK");
  delay(200);

  Wire.setClock(400000);
  DPRINTLN("I2C @ 400kHz");

  setAllServosNeutral();
  delay(500);

  // Bluetooth
  DPRINTLN("\n--- Bluetooth ---");
  if (!initBluetooth()) {
    DPRINTLN("Bluetooth nije inicijaliziran");
  } else {
    DPRINTLN("Bluetooth OK");
  }
  delay(100);

  // Raspberry Pi
  DPRINTLN("\n--- Raspberry Pi USB ---");
  initRPiComms();
  testRPiCommunication();

  // ==================== GAIT VALIDACIJA ====================
  // Rezultati se ispisuju ovdje na Serial Monitor
  // prije nego robot fizicki krene
  validateGaitPositions();

  moveToHomePosition();
  //setAllServosNeutral();
  delay(1000);

  currentState = STATE_IDLE;
  sendBluetoothStatus("READY");

  DPRINTLN("Setup complete - cekam naredbe");
  DPRINTLN("========================================\n");
}

// ==================== LOOP ====================
void loop() {

  // IMU update
  if (imuAvailable) {
    updateOrientation();
    if (isTilted(40.0)) {
      static unsigned long lastTiltWarning = 0;
      if (millis() - lastTiltWarning > 5000) {
        DPRINTLN("ROBOT JAKO NAGNUT!");
        lastTiltWarning = millis();
      }
    }
  }

  // Bluetooth
  if (checkBluetoothData()) {
    String cmd = getBluetoothCommand();
    if (cmd.length() > 0) {
      DPRINT("BT: "); DPRINTLN(cmd);
      processCommand(cmd);
      lastCommandTime = millis();
    }
  }

  // Geste (Raspberry Pi)
  int gesture = receiveGestureFromPi();
  if (gesture >= 0) {
    String cmd;
    switch (gesture) {
      case 0: cmd = "";          break;
      case 1: cmd = "FORWARD";   break;
      case 2: cmd = "BACKWARD";  break;
      case 3: cmd = "RIGHT";     break;
      case 4: cmd = "LEFT";      break;
      case 5: cmd = "STOP";      break;
      default: cmd = ""; break;
    }
    if (cmd != "") {
      DPRINT("Pi "); DPRINT(gesture); DPRINT(" -> "); DPRINTLN(cmd);
      processCommand(cmd);
      lastCommandTime = millis();
    }
  }

  // Kontinuirani hod
  switch (currentState) {
    case STATE_WALKING_FORWARD:   tripodGaitForward();   break;
    case STATE_WALKING_BACKWARD:  tripodGaitBackward();  break;
    case STATE_TURNING_LEFT:      tripodGaitTurnLeft();  break;
    case STATE_TURNING_RIGHT:     tripodGaitTurnRight(); break;
    case STATE_STANDING:
      maintainStandingPosition();
      currentState = STATE_IDLE;
      break;
    case STATE_IDLE:
      break;
  }

  // Timeout
  if (currentState != STATE_IDLE && currentState != STATE_STANDING &&
      millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    currentState = STATE_IDLE;
    sendBluetoothStatus("TIMEOUT_STOPPED");
    DPRINTLN("Timeout -> IDLE");
  }
}

// ==================== OBRADA KOMANDI ====================
void processCommand(String cmd) {
  DPRINT("CMD: "); DPRINTLN(cmd);

  if (cmd == "FORWARD") {
    currentState = STATE_WALKING_FORWARD;
    sendBluetoothConfirmation("FORWARD");
  } else if (cmd == "BACKWARD") {
    currentState = STATE_WALKING_BACKWARD;
    sendBluetoothConfirmation("BACKWARD");
  } else if (cmd == "LEFT") {
    currentState = STATE_TURNING_LEFT;
    sendBluetoothConfirmation("LEFT");
  } else if (cmd == "RIGHT") {
    currentState = STATE_TURNING_RIGHT;
    sendBluetoothConfirmation("RIGHT");
  } else if (cmd == "STOP") {
    currentState = STATE_IDLE;
    sendBluetoothConfirmation("STOP");
  } else if (cmd == "HOME") {
    currentState = STATE_STANDING;
    moveToHomePosition();
    sendBluetoothConfirmation("HOME");
  } else if (cmd == "STATUS") {
    sendBluetoothStatus("=== HEXAPOD STATUS ===");
    sendBluetoothStatus(btConnected ? "BT: OK" : "BT: ERROR");
    String stateStr = "STATE: ";
    switch(currentState) {
      case STATE_IDLE:             stateStr += "IDLE";     break;
      case STATE_WALKING_FORWARD:  stateStr += "FORWARD";  break;
      case STATE_WALKING_BACKWARD: stateStr += "BACKWARD"; break;
      case STATE_TURNING_LEFT:     stateStr += "LEFT";     break;
      case STATE_TURNING_RIGHT:    stateStr += "RIGHT";    break;
      case STATE_STANDING:         stateStr += "STANDING"; break;
    }
    sendBluetoothStatus(stateStr);
  } else {
    sendBluetoothStatus("UNKNOWN_CMD");
    DPRINTLN("Unknown command");
  }

  delay(50);
}

// ==================== POZICIJE ====================
void moveToHomePosition() {
  const float homeX = 160.0f;
  const float homeY = 0.0f;
  const float homeZ = -85.0f;
  const int   transTime = 400;

  DPRINTLN("Moving to home...");

  for (int leg = 0; leg < 6; leg++) {
    setLegPosition(leg, homeX, homeY, homeZ, transTime);
    delay(50);
  }

  resetGaitState();
}

void maintainStandingPosition() {
  delay(100);
}
