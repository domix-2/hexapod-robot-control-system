/*
 * KOMUNIKACIJA_PI5.H - Serijska komunikacija Arduino <-> Raspberry Pi 5 preko USB
 */

#ifndef KOMUNIKACIJA_PI5_H
#define KOMUNIKACIJA_PI5_H

#include <Arduino.h>

// ==================== KONFIGURACIJA ====================
#define RPI_BAUD         115200
#define RPI_TIMEOUT_MS   150

// Omogući debug ispise (zakomentiraj za finalni kod)
#ifndef DEBUG
  #define DEBUG 0
#endif

// ==================== INICIJALIZACIJA ====================
void initRPiComms() {
  Serial.setTimeout(RPI_TIMEOUT_MS);
  
  Serial.println("USB serijska komunikacija s Raspberry Pi 5 inicijalizirana");
  Serial.print("  Baud rate: "); Serial.println(RPI_BAUD);
  Serial.println("  Port: USB Virtual COM (ttyACM0 ili ttyUSB0 na Pi)");
  Serial.println("  Format poruka: F##C (npr. F01C, F02C, F05C)");
}

// ==================== TEST KOMUNIKACIJE ====================
bool testRPiCommunication() {
  Serial.println("Testiranje USB komunikacije s Raspberry Pi 5...");
  Serial.println("TEST");
  
  unsigned long start = millis();
  String response = "";

  while (millis() - start < 1500) {
    if (Serial.available()) {
      response = Serial.readStringUntil('\n');
      response.trim();
      Serial.print("← Primljeno: ");
      Serial.println(response);

      if (response.indexOf("OK") >= 0 || response.indexOf("hello") >= 0) {
        Serial.println("✓ Test uspješan! USB veza s Pi 5 radi.");
        return true;
      }
    }
    delay(10);
  }

  Serial.println("✗ Greška: Nema odgovora od Raspberry Pi 5");
  return false;
}

// ==================== PRIMANJE I OBRADA GESTE ====================
int receiveGestureFromPi() {
  if (!Serial.available()) {
    return -1;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  #if DEBUG
    Serial.print("[DEBUG] Primljena poruka: '");
    Serial.print(line);
    Serial.print("' (dužina: ");
    Serial.print(line.length());
    Serial.println(")");
  #endif

  // ISPRAVLJENO: Očekuje F##C format (4 znaka)
  if (line.length() < 4 || line[0] != 'F' || line[3] != 'C') {
    #if DEBUG
      Serial.println("[DEBUG] ✗ Nevažeći format (očekujem F##C)");
    #endif
    return -1;
  }

  char digit1 = line[1];
  char digit2 = line[2];
  
  if (!isdigit(digit1) || !isdigit(digit2)) {
    #if DEBUG
      Serial.println("[DEBUG] ✗ Znakovi nisu cifre");
    #endif
    return -1;
  }

  int fingerCount = (digit1 - '0') * 10 + (digit2 - '0');
  
  if (fingerCount < 0 || fingerCount > 5) {
    #if DEBUG
      Serial.print("[DEBUG] ✗ Broj izvan granica: ");
      Serial.println(fingerCount);
    #endif
    return -1;
  }

  #if DEBUG
    Serial.print("[DEBUG] ✓ Validna gesta: ");
    Serial.print(fingerCount);
    Serial.println(" prstiju");
  #endif

  // Potvrda natrag na Pi
  Serial.print("OK:");
  Serial.println(fingerCount);
  Serial.flush();

  return fingerCount;
}

// ==================== HEARTBEAT ====================
void sendHeartbeatToPi() {
  static unsigned long lastHb = 0;
  if (millis() - lastHb >= 5000) {
    Serial.println("HB");
    lastHb = millis();
    #if DEBUG
      Serial.println("[DEBUG] Heartbeat poslan");
    #endif
  }
}

#endif // KOMUNIKACIJA_PI5_H