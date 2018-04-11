/*
  LeddarVu8ArduinoEchoes.ino

  This sketch shows the readEchoes functionality of the LeddarVu8Arduino
  library.

  Created: 11 Apr. 2018
  by Fredrik Magnussen
*/

#include "LeddarVu8Arduino.h"

LeddarVu8Arduino leddarVu8;

uint8_t csPin = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("LeddarVu8Arduino library Testing");
  int8_t check = leddarVu8.begin(csPin);
  if (check <= 0) {
    Serial.println("LeddarVu8 initialized.");
  } else {
    Serial.print("Error: "); Serial.print(check);
    Serial.println(" - LeddarVu8 initialization failed.");
  }
}

void loop() {
  // Reading amplitudes and distances
  Serial.println("Reading amplitudes and distances: ");
  float distances[8];
  float amplitudes[8];
  int8_t check = leddarVu8.readEchoes(distances, amplitudes);
  if (check <= 0) {
    for (int i = 0; i < 8; i++) {
      Serial.print(distances[i], 4);
      Serial.print("\t");
      Serial.println(amplitudes[i]);
    }
  } else {
    Serial.print("Error: "); Serial.print(check);
  }
  delay(2000);
}
