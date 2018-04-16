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

  // Read the device information and print to screen
  char modulePartNumber[32];
  char softwarePartNumber[32];
  char moduleSerialNumber[32];
  char manufacturerName[32];
  char groupIdentifactionNumber[32];
  char buildDate[32];
  char firmwareVersion[32];
  char bootloaderVersion[32];
  char asicVersion[32];
  char fgpaVersion[32];

  uint16_t moduleType;
  uint8_t accExpMin;
  uint8_t accExpMax;
  uint8_t oversampExpMin;
  uint8_t oversampExpMax;
  uint8_t basePointSampleMin;
  uint8_t basePointSampleMax;
  uint16_t numberOfVerticalSegments;
  uint16_t numberOfHorizontalSegments;
  uint16_t numberOfReferenceSegments;
  uint32_t basePointSampleDistance;
  uint32_t referenceSegmentMask;
  uint16_t maxNumberOfSamples;
  uint32_t clockFrequency;
  uint8_t maxNumberOfDetectionsPerSegment;
  uint32_t distanceScale;
  uint8_t rawAmplitudeScaleBit;
  uint32_t rawAmplitudeScale;
  int16_t precisionMin;
  int16_t precisionMax;
  int32_t sensitivityMin;
  int32_t sensitivityMax;
  
  uint8_t currentLightSourcePowerCount;
  uint16_t autoFrameAverageMin;
  uint16_t autoFrameAverageMax;
  uint8_t autoLightSourcePowerPercentMin;
  uint8_t autoLightSourcePowerPercentMax;
  uint8_t autoDetectionsAverageMin;
  uint8_t autoDetectionsAverageMax;
  uint8_t staticNoiseCalibrationSource;
  uint32_t CPUloadScale;
  uint32_t TemperatureScale;
  
  leddarVu8.getModulePartNumber(modulePartNumber);
  leddarVu8.getSoftwarePartNumber(softwarePartNumber);
  leddarVu8.getModuleSerialNumber(moduleSerialNumber);
  leddarVu8.getManufacturerName(manufacturerName);
  leddarVu8.getGroupIdentificationNumber(groupIdentifactionNumber);
  leddarVu8.getBuildDate(buildDate);
  leddarVu8.getFirmwareVersion(firmwareVersion);
  leddarVu8.getBootloaderVersion(bootloaderVersion);
  leddarVu8.getAsicVersion(asicVersion);
  leddarVu8.getFgpaVersion(fgpaVersion);

  moduleType = leddarVu8.getModuleType();
  accExpMin = leddarVu8.getAccumulationExponentMin();
  accExpMax = leddarVu8.getAccumulationExponentMax();
  oversampExpMin = leddarVu8.getOversamplingExponentMin();
  oversampExpMax = leddarVu8.getOversamplingExponentMax();
  basePointSampleMin = leddarVu8.getBasePointSampleMin();
  basePointSampleMax = leddarVu8.getBasePointSampleMax();
  numberOfVerticalSegments = leddarVu8.getNumberOfVerticalSegments();
  numberOfHorizontalSegments = leddarVu8.getNumberOfHorizontalSegments();
  numberOfReferenceSegments = leddarVu8.getNumberOfReferenceSegments();
  basePointSampleDistance = leddarVu8.getBasePointSampleDistance();
  referenceSegmentMask = leddarVu8.getReferenceSegmentMask();
  maxNumberOfSamples = leddarVu8.getNumberOfSamplesMax();
  clockFrequency = leddarVu8.getClockFrequency();
  maxNumberOfDetectionsPerSegment = leddarVu8.getMaxNumberOfDetectionsPerSegment();
  distanceScale = leddarVu8.getDistanceScale();
  rawAmplitudeScaleBit = leddarVu8.getRawAmplitudeScaleBit();
  rawAmplitudeScale = leddarVu8.getRawAmplitudeScale();
  precisionMin = leddarVu8.getPrecisionMin();
  precisionMax = leddarVu8.getPrecisionMax();
  sensitivityMin = leddarVu8.getSensitivityMin();
  sensitivityMax = leddarVu8.getSensitivityMax();
  
  currentLightSourcePowerCount = leddarVu8.getCurrentLightSourcePowerCount();
  autoFrameAverageMin = leddarVu8.getAutoFrameAverageMin();
  autoFrameAverageMax = leddarVu8.getAutoFrameAverageMax();
  autoLightSourcePowerPercentMin = leddarVu8.getAutoLightSourcePowerPercentMin();
  autoLightSourcePowerPercentMax = leddarVu8.getAutoLightSourcePowerPercentMax();
  autoDetectionsAverageMin = leddarVu8.getAutoDetectionsAverageMin();
  autoDetectionsAverageMax = leddarVu8.getAutoDetectionsAverageMax();
  staticNoiseCalibrationSource = leddarVu8.getStaticNoiseCalibrationSource();
  CPUloadScale = leddarVu8.getCPUloadScale();
  TemperatureScale = leddarVu8.getTemperatureScale();
  
  Serial.println();
  Serial.println("Device information:");
  Serial.print("Module number: \t\t"); Serial.println(modulePartNumber);
  Serial.print("Software part number: \t"); Serial.println(softwarePartNumber);
  Serial.print("Module serial number: \t"); Serial.println(moduleSerialNumber);
  Serial.print("Manufacturer name: \t"); Serial.println(manufacturerName);
  Serial.print("Group id nr.: \t"); Serial.println(groupIdentifactionNumber);
  Serial.print("Build date: \t\t"); Serial.println(buildDate);
  Serial.print("Firmware version: \t"); Serial.println(firmwareVersion);
  Serial.print("Bootloader version: \t"); Serial.println(bootloaderVersion);
  Serial.print("ASIC version: \t\t"); Serial.println(asicVersion);
  Serial.print("FGPA version: \t\t"); Serial.println(fgpaVersion);
  Serial.print("Module type: \t\t"); Serial.print("0x0"); Serial.println(moduleType, HEX);
  Serial.println();
  Serial.println("Device constants:");
  Serial.print("Accumulation Exp. Min: \t\t"); Serial.println(accExpMin);
  Serial.print("Accumulation Exp. Max: \t\t"); Serial.println(accExpMax);
  Serial.print("Oversampling Exp. Min: \t\t"); Serial.println(oversampExpMin);
  Serial.print("Oversampling Exp. Max: \t\t"); Serial.println(oversampExpMax);
  Serial.print("Base Point Sample Min: \t\t"); Serial.println(basePointSampleMin);
  Serial.print("Base Point Sample Max: \t\t"); Serial.println(basePointSampleMax);
  Serial.print("Nr. of vert. segments: \t\t"); Serial.println(numberOfVerticalSegments);
  Serial.print("Nr. of hori. segments: \t\t"); Serial.println(numberOfHorizontalSegments);
  Serial.print("Nr. of hori. segments: \t\t"); Serial.println(numberOfHorizontalSegments);
  Serial.print("Nr. of ref. segments: \t\t");  Serial.println(numberOfReferenceSegments);
  Serial.print("Base point sample distance: \t");  Serial.println(basePointSampleDistance);
  Serial.print("Ref. Segment Mask.: \t\t");  Serial.println(referenceSegmentMask);
  Serial.print("Max number of samples: \t\t");  Serial.println(maxNumberOfSamples);
  Serial.print("Clock Frequency: \t\t");  Serial.println(clockFrequency);
  Serial.print("Max. nr. of detect. per segment:");  Serial.println(maxNumberOfDetectionsPerSegment);
  Serial.print("Distance scale: \t\t");  Serial.println(distanceScale);
  Serial.print("Raw amplitude scale bit: \t"); Serial.print("0x0"); Serial.println(rawAmplitudeScaleBit,HEX);
  Serial.print("Raw amplitude scale: \t\t");  Serial.println(rawAmplitudeScale);
  Serial.print("Precision min: \t\t\t");  Serial.println(precisionMin);
  Serial.print("Precision max: \t\t\t");  Serial.println(precisionMax);
  Serial.print("Sensitivity min: \t\t");  Serial.println(sensitivityMin);
  Serial.print("Sensitivity max: \t\t");  Serial.println(sensitivityMax);
  Serial.print("Cur, light source pwr. count: \t");  Serial.println(currentLightSourcePowerCount);
  Serial.print("Auto frame average min: \t");  Serial.println(autoFrameAverageMin);
  Serial.print("Auto frame average max: \t");  Serial.println(autoFrameAverageMax);
  Serial.print("Auto light src. pwr. % min:\t");  Serial.println(autoLightSourcePowerPercentMin);
  Serial.print("Auto light src. pwr. % max:\t");  Serial.println(autoLightSourcePowerPercentMax);
  Serial.print("Auto detections average min\t");  Serial.println(autoDetectionsAverageMin);
  Serial.print("Auto detections average max\t");  Serial.println(autoDetectionsAverageMax);
  Serial.print("Static noise calibration source\t");  Serial.println(staticNoiseCalibrationSource);
  Serial.print("CPU load scale:\t\t\t");  Serial.println(CPUloadScale);
  Serial.print("Temperature Scale: \t\t");  Serial.println(TemperatureScale);

}

void loop() {
  // empty
}
