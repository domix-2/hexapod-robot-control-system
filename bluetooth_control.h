/*
 * BLUETOOTH CONTROL - PRILAGOĐENO ZA TVOJU MIT APP INVENTOR APLIKACIJU
 * Prepoznaje jednoslovne komande: F=naprijed, B=nazad, L=lijevo, R=desno, S=stop, H=home
 */

#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <Arduino.h>
#include <HardwareSerial.h>

// ==================== KONFIGURACIJA ====================
#define BT_RX_PIN 8
#define BT_TX_PIN 9
#define BT_BAUD_RATE 9600

// ==================== GLOBALNE VARIJABLE ====================
HardwareSerial BTSerial(1);
String lastBTCommand = "";
bool btConnected = false;
unsigned long lastBTActivity = 0;

// ==================== MAPIRANJE KOMANDI ====================
String mapBluetoothCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.length() == 1) {
    char letter = cmd.charAt(0);
    switch(letter) {
      case 'F': return "FORWARD";
      case 'B': return "BACKWARD";
      case 'L': return "LEFT";
      case 'R': return "RIGHT";
      case 'S': return "STOP";
      case 'H': return "HOME";
      default: return "";
    }
  }
  
  // Ako je puna fraza (fallback za stare verzije app-a)
  if (cmd.indexOf("NAPRIJED") >= 0 || cmd.indexOf("NAPRIJED") >= 0) return "FORWARD";
  if (cmd.indexOf("NAZAD") >= 0) return "BACKWARD";
  if (cmd.indexOf("LIJEVO") >= 0) return "LEFT";
  if (cmd.indexOf("DESNO") >= 0) return "RIGHT";
  if (cmd.indexOf("STOP") >= 0) return "STOP";
  if (cmd.indexOf("HOME") >= 0 || cmd.indexOf("POČETNA") >= 0) return "HOME";

  return "";  // Nepoznato
}

// SLANJE PORUKA 

void sendBluetoothConfirmation(String message) {
  BTSerial.print("OK:");
  BTSerial.println(message);
}

void sendBluetoothStatus(String message) {
  BTSerial.print("STATUS:");
  BTSerial.println(message);
}

//  INICIJALIZACIJA 
bool initBluetooth() {
  BTSerial.begin(BT_BAUD_RATE, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);
  delay(100);

  if (!BTSerial) {
    return false;
  }

  btConnected = true;
  return true;
}

// ==================== PROVJERA I ČITANJE ====================
bool checkBluetoothData() {
  return BTSerial.available() > 0;
}

String getBluetoothCommand() {
  String received = "";
  
  while (BTSerial.available()) {
    char c = BTSerial.read();
    if (c == '\n' || c == '\r') break;
    received += c;
    delay(2);  // Mali delay da svi znakovi stignu
  }
  
  received.trim();
  
  if (received.length() == 0) {
    return "";
  }
  
  String command = mapBluetoothCommand(received);
  
  if (command.length() > 0) {
    lastBTCommand = command;
    lastBTActivity = millis();
  }
  
  return command;
}

#endif // BLUETOOTH_CONTROL_H