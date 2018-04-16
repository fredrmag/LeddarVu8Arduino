/*
  LeddarVu8ArduinoInfo.ino

  This sketch outputs the Device information and constants of the connected
  Leddar device.

  Created: 12 Apr. 2018
  by Fredrik Magnussen
*/
#include "LeddarVu8Arduino.h"

LeddarVu8Arduino leddarVu8;

uint8_t csPin = 10;

void setup() {
  // setup
  Serial.begin(9600);
  Serial.println("LeddarVu8Arduino Device info");

  // Initialize Leddar device
  int8_t check = leddarVu8.begin(csPin);
  if (check <= 0) {
    Serial.println("LeddarVu8 initialized.");
  } else {
    Serial.print("Error: "); Serial.print(check);
    Serial.println(" - LeddarVu8 initialization failed.");
  }
  Serial.println();

  char deviceName[32];
  uint8_t accumulationExponent;
  uint8_t oversamplingExponent;
  uint8_t basePointSample;
  uint32_t segmentEnabled;
  uint32_t referencePulseRate;
  float yaw;
  float pitch;
  float roll;
  float x;
  float y;
  float z;
  int8_t precision;
  uint8_t precisionEnabled;
  uint8_t saturationEnabled;
  uint8_t overshootManagementEnabled;
  int32_t sensitivity;
  uint8_t lightSourcePower;
  uint8_t autoLightSourcePowerEnabled;
  uint16_t autoFrameAverage;
  uint8_t autoDetectionsAverage;
  uint8_t objectDemergingEnabled;
  uint8_t staticNoiseRemovalEnabled;
  
  leddarVu8.getDeviceName(deviceName);
  accumulationExponent = leddarVu8.getAccumulationExponent();
  oversamplingExponent = leddarVu8.getOversamplingExponent();
  basePointSample = leddarVu8.getBasePointSample();
  segmentEnabled = leddarVu8.getSegmentEnabled();
  referencePulseRate = leddarVu8.getReferencePulseRate();
  yaw = leddarVu8.getYaw();
  pitch = leddarVu8.getPitch();
  roll = leddarVu8.getRoll();
  x = leddarVu8.getXPos();
  y = leddarVu8.getYPos();
  z = leddarVu8.getZPos();
  precision = leddarVu8.getPrecision();
  precisionEnabled = leddarVu8.getPrecisionEnabled();
  saturationEnabled = leddarVu8.getSaturationCompensationEnabled();
  overshootManagementEnabled = leddarVu8.getOvershootManagementEnabled();
  sensitivity = leddarVu8.getSensitivity();
  lightSourcePower = leddarVu8.getLightSourcePower();
  autoLightSourcePowerEnabled = leddarVu8.getAutoLightSourcePowerEnabled();
  autoFrameAverage = leddarVu8.getAutoFrameAverage();
  autoDetectionsAverage = leddarVu8.getAutoDetectionsAverage();
  objectDemergingEnabled = leddarVu8.getObjectDemergingEnabled();
  staticNoiseRemovalEnabled = leddarVu8.getStaticNoiseRemovalEnabled();
  
  Serial.println();
  Serial.println("Configuration data:");
  Serial.print("Device name: \t\t"); Serial.println(deviceName);
  
  Serial.print("Accumulation exponent: \t"); Serial.println(accumulationExponent);
  Serial.print("Oversampling exponent: \t"); Serial.println(oversamplingExponent);
  Serial.print("Base point sample: \t"); Serial.println(basePointSample);
  Serial.print("Segment enabled: \t"); Serial.println(segmentEnabled);
  Serial.print("Reference pulse rate: \t"); Serial.println(referencePulseRate);
  Serial.print("Yaw: \t\t\t"); Serial.println(yaw);
  Serial.print("Pitch: \t\t\t"); Serial.println(pitch);
  Serial.print("Roll: \t\t\t"); Serial.println(roll);
  Serial.print("X: \t\t\t"); Serial.println(x);
  Serial.print("Y: \t\t\t"); Serial.println(y);
  Serial.print("Z: \t\t\t"); Serial.println(z);
  Serial.print("Precision: \t\t"); Serial.println(precision);
  Serial.print("Precision enabled: \t"); Serial.println(precisionEnabled);
  Serial.print("Saturation enabled: \t"); Serial.println(saturationEnabled);
  Serial.print("Overshoot enabled: \t"); Serial.println(overshootManagementEnabled);
  Serial.print("Sensitivity: \t\t"); Serial.println(sensitivity);
  Serial.print("Light source power: \t"); Serial.println(lightSourcePower);
  Serial.print("Auto src power enabled:\t"); Serial.println(autoLightSourcePowerEnabled);
  Serial.print("Auto frame average: \t"); Serial.println(autoFrameAverage);
  Serial.print("Auto detec. average: \t"); Serial.println(autoDetectionsAverage);
  Serial.print("Object DEM enabled: \t"); Serial.println(objectDemergingEnabled);
  Serial.print("Noise removal enabled: \t"); Serial.println(staticNoiseRemovalEnabled);

}

void loop() {
  // empty
}
