/*
  LeddarVu8ArduinoInfo.ino

  This sketch shows you how to change the config parameters.

  Created: 12 Apr. 2018
  by Fredrik Magnussen
*/
#include "LeddarVu8Arduino.h"

LeddarVu8Arduino leddarVu8;

uint8_t csPin = 10;

void setup() {
  // setup
  Serial.begin(9600);
  Serial.println("LeddarVu8Arduino - Change config data");

  // Initialize Leddar device
  int8_t check = leddarVu8.begin(csPin);
  if (check <= 0) {
    Serial.println("LeddarVu8 initialized.");
  } else {
    Serial.print("Error: "); Serial.print(check);
    Serial.println(" - LeddarVu8 initialization failed.");
  }
  Serial.println();

  // Change configuration data
  leddarVu8.setAccumulationExponent(4);
  leddarVu8.setOversamplingExponent(2);
  leddarVu8.setBasePointSample(10);
}

void loop() {
  // empty
}
