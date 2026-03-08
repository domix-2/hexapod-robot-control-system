/*
 * IMU KALIBRACIJA - SAMO PREKO USB-a
 * 
 * Ovaj sketch:
 * - Inicijalizira I2C bus
 * - Inicijalizira LSM6DS3 (daisy-chain)
 * - Inicijalizira PCA9685 kontrolere (BEZ servo pokreta!)
 * - Kalibrira IMU gyro offset
 * - Sprema offset u EEPROM (opciono)
 * - NE POKREĆE servoe!
 */
#define DEBUG 1
#if DEBUG
  #define DPRINT(...)   Serial.print(__VA_ARGS__)
  #define DPRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DPRINT(...)
  #define DPRINTLN(...)
#endif

#include <Wire.h>
#include <Preferences.h>  // ESP32 NVS za spremanje offseta
#include "imu_control.h"

#define LED_PIN 2

Preferences prefs;
bool imuAvailable = false;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_PIN, OUTPUT);
  
  DPRINTLN("========================================");
  DPRINTLN("   IMU KALIBRACIJA - USB MODE");
  DPRINTLN("========================================");
  DPRINTLN();
  
  // Blink da pokažemo da smo živi
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // ==================== I2C INIT ====================
  DPRINTLN("Pokretanje I2C bus-a...");
  Wire.begin(21, 22);  // SDA=21, SCL=22 za ESP32
  Wire.setClock(100000);  // 100kHz za stabilnost
  delay(200);
  
  DPRINTLN("✓ I2C @ 100kHz\n");
  
  // ==================== I2C SCAN (OPCIONO) ====================
  DPRINTLN("Skeniram I2C bus...");
  scanI2C();
  DPRINTLN();
  
  // ==================== IMU INIT ====================
  DPRINTLN("Inicijalizacija LSM6DS3...");
  if (!initIMU()) {
    DPRINTLN("✗ LSM6DS3 nije pronađen!");
    DPRINTLN();
    DPRINTLN("Provjeri:");
    DPRINTLN("  - SDA/SCL pinove (21, 22)");
    DPRINTLN("  - Napajanje (3.3V)");
    DPRINTLN("  - I2C adresa (0x6A ili 0x6B)");
    DPRINTLN();
    
    // Treperi brzo zauvijek
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  imuAvailable = true;
  DPRINTLN("✓ LSM6DS3 inicijaliziran!\n");
  
  // ==================== KALIBRACIJA ====================
  DPRINTLN("========================================");
  DPRINTLN("PRIPREMA ZA KALIBRACIJU");
  DPRINTLN("========================================");
  DPRINTLN("VAŽNO:");
  DPRINTLN("  1. Postavi robota na RAVNU površinu");
  DPRINTLN("  2. NE DODIRUJ robota");
  DPRINTLN("  3. Čekaj 10 sekundi");
  DPRINTLN();
  DPRINTLN("Počinjem za 5 sekundi...");
  
  for (int i = 5; i > 0; i--) {
    DPRINT("  ");
    DPRINT(i);
    DPRINTLN("...");
    delay(1000);
  }
  
  DPRINTLN();
  
  // LED upaljeno tijekom kalibracije
  digitalWrite(LED_PIN, HIGH);
  
  // KALIBRACIJA
  calibrateGyro(); // <- poziva se iz imu_control.h
  
  digitalWrite(LED_PIN, LOW);
  
  // ==================== SPREMANJE U EEPROM ====================
  DPRINTLN();
  DPRINTLN("Spremanje gyro offseta u NVS (non-volatile storage)...");
  
  prefs.begin("imu", false);  // Namespace "imu", read-write mode
  prefs.putFloat("gyro_x", gyroOffsetX);
  prefs.putFloat("gyro_y", gyroOffsetY);
  prefs.putFloat("gyro_z", gyroOffsetZ);
  prefs.end();
  
  DPRINTLN("✓ Offset spremljen!");
  DPRINTLN();
  
  // ==================== POTVRDA ====================
  DPRINTLN("========================================");
  DPRINTLN("KALIBRACIJA ZAVRŠENA!");
  DPRINTLN("========================================");
  DPRINTLN();
  DPRINTLN("Gyro offseti:");
  DPRINT("  X: "); DPRINTLN(gyroOffsetX, 4);
  DPRINT("  Y: "); DPRINTLN(gyroOffsetY, 4);
  DPRINT("  Z: "); DPRINTLN(gyroOffsetZ, 4);
  DPRINTLN();
  DPRINTLN("Offseti su spremljeni u flash memoriju.");
  DPRINTLN("Možeš ih učitati u glavni program pomoću:");
  DPRINTLN("  prefs.getFloat(\"gyro_x\", 0.0);");
  DPRINTLN();
  DPRINTLN("========================================");
  DPRINTLN();
  
  // LED signal uspjeha - 3 duga trepera
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

// ==================== LOOP ====================
void loop() {
  // Update IMU i prikaži podatke
  if (imuAvailable) {
    updateOrientation();
    
    // Vizualni indikator nagiba
    if (isTilted(20.0)) {
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 200) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastBlink = millis();
      }
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    
    // Ispis svakih 2 sekunde
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
      DPRINTLN("---");
      DPRINT("Pitch: "); DPRINT(getPitch(), 1);
      DPRINT("°  Roll: "); DPRINT(getRoll(), 1);
      DPRINTLN("°");
      
      if (isTilted(20.0)) {
        DPRINTLN("⚠ NAGNUT!");
      }
      
      lastPrint = millis();
    }
  }
  
  delay(50);
}

// ==================== I2C SCANNER ====================
void scanI2C() {
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      DPRINT("  ✓ Uređaj na 0x");
      if (address < 16) DPRINT("0");
      DPRINT(address, HEX);
      
      // Identifikacija poznatih uređaja
      if (address == 0x40) DPRINT(" (PCA9685 Left)");
      if (address == 0x41) DPRINT(" (PCA9685 Right)");
      if (address == 0x6A || address == 0x6B) DPRINT(" (LSM6DS3)");
      
      DPRINTLN();
      nDevices++;
    }
    
    delay(5);
  }
  
  if (nDevices == 0) {
    DPRINTLN("  ✗ Nema I2C uređaja!");
  } else {
    DPRINT("  Pronađeno: ");
    DPRINT(nDevices);
    DPRINTLN(" uređaja");
  }
}