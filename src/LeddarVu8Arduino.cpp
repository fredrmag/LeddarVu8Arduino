/* Arduino LeddarTechVu8 library
* Copyright (C) 2017 by PSS-GEO AS
*
* \file       LeddarVu8Arduino.cpp
* \brief      Arduino Library for the LeddarTech Vu8 SPI sensor
* \author     Fredrik Magnussen
* \since      March 2018
*
*/
#include "Arduino.h"
#include "LeddarVu8Arduino.h"
#include "SPIdriver.h"

//-----------------------------------------------------------------------------
/** Initialize the LeddarVu8 sensor
*  Modified from SdSpiCard from Arduino SdCard Library
* \param[in] csPin LeddarTechVu8 chip select pin.
*/
bool LeddarVu8Arduino::begin(uint8_t csPin){
  // Initialize spi
  spiBegin(csPin);
  spiSetSpiSettings(SPISettings(15000000, MSBFIRST, SPI_MODE0)); // 15 MHz
  // TODO: Check the device name to see if we are connected, then return bool value.
  return true;
}
/** Extract byte from a 32-bit number, given number and byte-place
* \param[in] number - number to extract byte from
* \param[in] place - byte place from right, starts at 0
* \return byte - corresponding byte for given number and byte place
*/
byte LeddarVu8Arduino::extractByte(uint32_t number, int place) {
  return (number >> (8 * place)) & 0xFF;
}
/** Send command to leddar sensor through SPI
* \param[in] opcode
* \param[in] address
* \param[in] dataSize
*
*/
void LeddarVu8Arduino::leddarCommand(uint8_t opcode, uint32_t address, size_t dataSize) {
  uint8_t bufSend[6];
  // form message
  bufSend[0] = opcode;
  bufSend[1] = extractByte(address, 2);
  bufSend[2] = extractByte(address, 1);
  bufSend[3] = extractByte(address, 0);
  bufSend[4] = extractByte(dataSize, 1);
  bufSend[5] = extractByte(dataSize, 0);
  // TODO: Send through SPI protocol instead of writing to Serial
#if (LEDDAR_DEBUG == 1)
  for (int j = 0; j < 6; j++) {
    Serial.print(bufSend[j], HEX);
  }
  Serial.println();
#endif
  // spiSend(bufSend,sizeof(bufSend));
}
/** Combine bytes with least significant byte coming first.
*   param[in]
*/
uint32_t LeddarVu8Arduino::combineBytes(uint8_t* buf,size_t n) {
  uint32_t output = 0;

  for (int k = 0; k < n; k++) {
    //Serial.println(buf[k],HEX);
    // Convert from byte to 32-bit int
    uint32_t tmp = buf[k];
    output = output | (uint32_t)(tmp << (k * 8));
    //Serial.println((uint32_t)(tmp << (k * 8)), HEX);
  }
  //Serial.println(output,HEX);
  return output;
}

/** Read the raw echo values from the Leddar Vu8 sensor, i.e., raw distances and
* amplitudes.
* param[out] rawEchoes -  array of 8 raw distances and 8 raw amplitude values
*/
void LeddarVu8Arduino::readRawEchoes(uint32_t* distances, uint32_t* amplitudes){
  size_t size = 96; // 8*12 bytes = 96
  leddarCommand(READ,LEDDARVU8_START_OF_DETECTION_LIST_ARRAYS,size);
  // delay 1 ms according to manual
  delay(1);

  uint8_t bufReceive[size];
  // Mock function to receive data until we get the sensor!
  // TODO: change to spiReceive!
  spiFakeReceive(bufReceive,size);
  // Parse the received buffer
  // The received buffer contains 8 segments that contains 12 bytes each
  for (int i = 0; i < 8; i++){
    uint8_t bufSegment[12];
    for (int j = i * 12; j < (i + 1) * 12; j++) {
      bufSegment[j - i * 12] = bufReceive[j];
    }
    // Extract the distance and amplitude from segment
    uint8_t bufDistance[4];
    for (int k = 0; k < 4; k++) {
      bufDistance[k] = bufSegment[k];
    }
    uint8_t bufAmplitude[4];
    int l = 0;
    for (int k = 4; k < 8; k++) {
      bufAmplitude[l] = bufSegment[k];
      l++;
    }

    // Combine the bytes to a Number
    distances[i] = combineBytes(bufDistance,4);
    amplitudes[i] = combineBytes(bufAmplitude,4);

    #if (LEDDAR_DEBUG == 1)
    for (int j = i * 12; j < (i + 1) * 12; j++) {
      Serial.print(bufReceive[j], HEX);
    }
    Serial.print("\t");
    Serial.print(distances[i], DEC);
    Serial.print("\t");
    Serial.println(amplitudes[i], DEC);
    #endif

  }
}
/** Read Echoes from the LeddarTech Vu8 sensor
*  \param[out] distances - scaled distance - in meters
*   \param[out] amplitudes - scaled amplitudes - in ??
*
*/
void LeddarVu8Arduino::readEchoes(float* distances, float* amplitudes){
  // Convert to scaled distance and amplitudes
  uint32_t rawDistance[8];
  uint32_t rawAmplitude[8];
  readRawEchoes(rawDistance,rawAmplitude);
  for (int i = 0; i<8; i++){
    distances[i] = float(rawDistance[i])/float(DISTANCE_SCALE);
    amplitudes[i] = float(rawAmplitude[i]);
  }
}

// Mock SPI function until we get the device
void LeddarVu8Arduino::spiFakeReceive(uint8_t* buf, size_t n){
  uint8_t fakeSPI[96] = {0x87, 0x30, 0x00, 0x00, 0x6A, 0xF1, 0x42, 0x03, 0x07, 0x00, 0x01, 0x00, 0x53, 0x5B, 0x00, 0x00, 0x99, 0x29, 0x04, 0x02, 0x06, 0x00, 0x01, 0x00, 0xC6, 0xAE, 0x00, 0x00, 0xB9, 0x91, 0xAA, 0x01, 0x05, 0x00, 0x01, 0x00, 0xC5, 0x97, 0x00, 0x00, 0xF9, 0xEA, 0x46, 0x05, 0x04, 0x00, 0x01, 0x00, 0x3F, 0xC7, 0x00, 0x00, 0x7F, 0x74, 0xA9, 0x03, 0x03, 0x00, 0x01, 0x00, 0xE9, 0xEE, 0x00, 0x00, 0x1D, 0x78, 0x8E, 0x00, 0x02, 0x00, 0x01, 0x00, 0xB2, 0x6C, 0x00, 0x00, 0x0E, 0x38, 0xD1, 0x00, 0x01, 0x00, 0x01, 0x00, 0x0F, 0x2F, 0x00, 0x00, 0x6D, 0xBD, 0x15, 0x02, 0x00, 0x00, 0x01, 0x00};
  for (size_t i = 0; i < n; i++) {
    buf[i] = fakeSPI[i];
  }
}

//-----------------------------------------------------------------------------
/** Make SPI ready for transfer
*
*/
void LeddarVu8Arduino::spiStart(){
  if (!m_spiActive) {
    spiActivate();
    spiSelect();
    m_spiActive = true;
  }
}
//-----------------------------------------------------------------------------
/** Finish the SPI transfer
*
*/
void LeddarVu8Arduino::spiStop() {
  if (m_spiActive) {
    spiUnselect();
    spiSend(0XFF);
    spiDeactivate();
    m_spiActive = false;
  }
}
//------------------------------------------------------------------------------
