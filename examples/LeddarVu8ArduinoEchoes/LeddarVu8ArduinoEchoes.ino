#include "LeddarVu8Arduino.h"

LeddarVu8Arduino leddarVu8;

uint8_t csPin = 10;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("LeddarVu8Arduino library Testing");

  //leddarVu8.begin(csPin);

  float distances[8];
  float amplitudes[8];
  leddarVu8.readEchoes(distances, amplitudes);
  for (int i = 0; i < 8; i++) {
    Serial.print(distances[i],4);
    Serial.print("\t");
    Serial.println(amplitudes[i]);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
