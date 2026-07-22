/*
  Test de communication I2C avec le LIDAR TF-Luna (adresse 0x10)
  -----------------------------------------------------------------
  Objectif : vérifier le câblage, puis lire distance / signal /
  température selon le protocole natif de la TF-Luna.

  Si ton XIAO ESP32S3 utilise des pins I2C spécifiques,
  décommente et adapte la ligne Wire.begin(SDA, SCL) ci-dessous.
*/

#include <Wire.h>

const uint8_t LIDAR_ADDR = 0x10;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Wire.begin(5, 6); // SDA, SCL -> décommente si besoin (ex: XIAO ESP32S3)
  Wire.begin();

  delay(500);
  Serial.println("=== Scan I2C ===");
  scanI2C();

  Serial.println();
  Serial.print("Test de ping sur 0x10... ");
  if (pingDevice(LIDAR_ADDR)) {
    Serial.println("OK, le LIDAR répond !");
  } else {
    Serial.println("Pas de réponse. Vérifie le câblage (SDA/SCL/GND/VCC).");
  }
}

void loop() {
  readTFLuna();
  delay(200); // la TF-Luna tourne par défaut à ~100Hz, pas besoin d'aller plus vite
}

void readTFLuna() {
  // On pointe sur le registre 0x00 (début du bloc de données)
  Wire.beginTransmission(LIDAR_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Erreur d'écriture du registre.");
    return;
  }

  // Puis on lit 6 octets d'un coup : distance, force du signal, température
  Wire.requestFrom(LIDAR_ADDR, (uint8_t)6);
  if (Wire.available() < 6) {
    Serial.println("Réponse incomplète du capteur.");
    return;
  }

  uint16_t distance   = Wire.read() | (Wire.read() << 8);      // en cm
  uint16_t strength   = Wire.read() | (Wire.read() << 8);      // force du signal
  int16_t  tempRaw    = Wire.read() | (Wire.read() << 8);      // température brute
  float temperature   = tempRaw / 8.0 - 256.0;                  // conversion en °C

  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.print(" cm | Signal : ");
  Serial.print(strength);
  Serial.print(" | Temp : ");
  Serial.print(temperature, 1);
  Serial.println(" °C");
}

bool pingDevice(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void scanI2C() {
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Périphérique trouvé à l'adresse 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println("Aucun périphérique I2C détecté.");
  }
}