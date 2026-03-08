#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

#include <Adafruit_LSM6DS3.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

// ==================== KONFIGURACIJA ====================
#define IMU_UPDATE_INTERVAL 100 // Update svakih 20ms (50Hz)

// I2C PINOVI za ESP32
#ifndef I2C_SDA
  #define I2C_SDA 21
#endif
#ifndef I2C_SCL
  #define I2C_SCL 22
#endif

// ==================== GLOBALNE VARIJABLE ====================
Adafruit_LSM6DS3 lsm6ds3;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float pitch = 0, roll = 0;
float lastPitch = 0, lastRoll = 0;

unsigned long lastIMUUpdate = 0;

// ==================== INICIJALIZACIJA ====================
bool initIMU() {
  DPRINTLN("Inicijalizacija LSM6DS3 (Adafruit)...");
  
  // KRITIČNO: begin_I2C s Wire bus-om i adresom
  // Standardne adrese: 0x6A (SDO -> GND) ili 0x6B (SDO -> VCC)
  if (!lsm6ds3.begin_I2C(0x6A, &Wire)) {
    DPRINTLN("✗ LSM6DS3 nije pronađen na 0x6A!");
    DPRINTLN("  Pokušavam 0x6B...");
    
    if (!lsm6ds3.begin_I2C(0x6B, &Wire)) {
      DPRINTLN("✗ LSM6DS3 nije pronađen ni na 0x6B!");
      DPRINTLN("  Provjeri:");
      DPRINTLN("    - SDA/SCL pinove");
      DPRINTLN("    - Napajanje (3.3V)");
      DPRINTLN("    - SDO pin (GND za 0x6A, VCC za 0x6B)");
      return false;
    }
    
    DPRINTLN("✓ LSM6DS3 pronađen na 0x6B");
  } else {
    DPRINTLN("✓ LSM6DS3 pronađen na 0x6A");
  }
  
  // Postavi range-ove
  lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  DPRINTLN("  Accel range: ±4G");
  
  lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  DPRINTLN("  Gyro range: ±500 °/s");
  
  lsm6ds3.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3.setGyroDataRate(LSM6DS_RATE_104_HZ);
  DPRINTLN("  Data rate: 104 Hz");
  
  delay(100);
  
  DPRINTLN("✓ LSM6DS3 inicijaliziran!");
  
  return true;
}

// ==================== KALIBRACIJA ====================
void calibrateGyro() {
  DPRINTLN("========================================");
  DPRINTLN("KALIBRACIJA ŽIROSKOPA");
  DPRINTLN("DRŽITE ROBOTA POTPUNO MIRNO!");
  DPRINTLN("========================================");
  
  delay(2000);  // Daj vremena da se smiri
  
  const int samples = 200;  // 200 uzoraka umjesto 1200
  float sumX = 0, sumY = 0, sumZ = 0;
  
  DPRINT("Prikupljam uzorke: ");
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    lsm6ds3.getEvent(&accel, &gyro, &temp);
    
    sumX += gyro.gyro.x;
    sumY += gyro.gyro.y;
    sumZ += gyro.gyro.z;
    
    // Progress bar
    if (i % 20 == 0) {
      DPRINT(".");
    }
    
    delay(10);  // 10ms * 200 = 2 sekunde
    
    // KRITIČNO: Yield za watchdog
    yield();  // ← DODAJ!
  }
  
  DPRINTLN(" Gotovo!");
  
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  
  DPRINTLN("========================================");
  DPRINT("Gyro offset X: "); DPRINTLN(gyroOffsetX, 4);
  DPRINT("Gyro offset Y: "); DPRINTLN(gyroOffsetY, 4);
  DPRINT("Gyro offset Z: "); DPRINTLN(gyroOffsetZ, 4);
  DPRINTLN("========================================");
  DPRINTLN("✓ Kalibracija završena!");
  DPRINTLN();
}

// ==================== UPDATE ORIJENTACIJE ====================
void updateOrientation() {
  // Rate limiting - update samo svakih 20ms
  if (millis() - lastIMUUpdate < IMU_UPDATE_INTERVAL) {
    return;
  }
  lastIMUUpdate = millis();
  
  sensors_event_t accel, gyro, temp;
  lsm6ds3.getEvent(&accel, &gyro, &temp);
  
  // Oduzmi offset
  float gx = gyro.gyro.x - gyroOffsetX;
  float gy = gyro.gyro.y - gyroOffsetY;
  float gz = gyro.gyro.z - gyroOffsetZ;
  
  // dt = 20ms = 0.02s (50Hz)
  float dt = IMU_UPDATE_INTERVAL / 1000.0;
  
  // Pitch i Roll iz akcelerometra
  float accPitch = atan2(accel.acceleration.y, accel.acceleration.z) * 180.0 / PI;
  float accRoll  = atan2(-accel.acceleration.x, 
                         sqrt(accel.acceleration.y * accel.acceleration.y + 
                              accel.acceleration.z * accel.acceleration.z)) * 180.0 / PI;
  
  // Komplementarni filter (98% gyro + 2% accel)
  pitch = 0.98 * (lastPitch + gx * dt) + 0.02 * accPitch;
  roll  = 0.98 * (lastRoll  + gy * dt) + 0.02 * accRoll;
  
  lastPitch = pitch;
  lastRoll = roll;
  
  #if DEBUG
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 5000) {
      DPRINT("Pitch: "); 
      DPRINT(pitch, 1);
      DPRINT("°  Roll: "); 
      DPRINT(roll, 1);
      DPRINT("°  Temp: ");
      DPRINT(temp.temperature, 1);
      DPRINTLN("°C");
      lastPrint = millis();
    }
  #endif
}

// ==================== GET FUNKCIJE ====================
float getPitch() {
  return pitch;
}

float getRoll() {
  return roll;
}

bool isTilted(float threshold = 30.0) {
  return (abs(pitch) > threshold || abs(roll) > threshold);
}
// ==================== UČITAVANJE OFFSETA IZ NVS ====================
bool loadGyroOffsets() {
  Preferences prefs;
  prefs.begin("imu", true);  // Namespace "imu", read-only mode
  
  // Provjeri postoje li spremljeni offseti
  if (!prefs.isKey("gyro_x")) {
    Serial.println("⚠ Nema spremljenih gyro offseta u flash memoriji");
    Serial.println("  → Prvo pokreni 'imu.ino' za kalibraciju!");
    prefs.end();
    return false;
  }
  
  // Učitaj spremljene offsete
  gyroOffsetX = prefs.getFloat("gyro_x", 0.0);
  gyroOffsetY = prefs.getFloat("gyro_y", 0.0);
  gyroOffsetZ = prefs.getFloat("gyro_z", 0.0);
  prefs.end();
  
  Serial.println("✓ Gyro offseti učitani iz flash memorije:");
  Serial.print("  X: "); Serial.println(gyroOffsetX, 4);
  Serial.print("  Y: "); Serial.println(gyroOffsetY, 4);
  Serial.print("  Z: "); Serial.println(gyroOffsetZ, 4);
  
  return true;
}
#endif // IMU_CONTROL_H