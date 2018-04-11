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
#include "CRC16.h"

//-----------------------------------------------------------------------------
/** Initialize the LeddarVu8 sensor.
* \param[in] csPin - LeddarTechVu8 chip select pin.
* \return Error code, or 0 if OK.
*/
uint8_t LeddarVu8Arduino::begin(uint8_t csPin){
  // Initialize SPI - modified from the Arduino SD-card library.
  m_spiActive = false;
  spiBegin(csPin);
  spiSetSpiSettings(SPISettings(15000000, MSBFIRST, SPI_MODE0)); // 15 MHz
  spiStart();

  // Check if we are connected to the LeddarVu8 sensor. The module type address
  // returns 0x0D if we are connected to a LeddarVu8 sensor.

  // send command
  uint8_t *bufSend;
  size_t size = 2;
  bufSend = leddarCommand(READ, LEDDARVU8_MODULE_TYPE, size);

  delay(1);

  // Receive answer
  uint8_t bufReceive[size+2];
  spiReceive(bufReceive,size+2);
  spiStop();

// debug
#if (LEDDAR_DEBUG == 1)
  for (int j = 0; j < size+2; j++) {
    Serial.print(bufReceive[j], HEX);
  }
  Serial.println();
#endif

  uint16_t moduleType = combineBytes(bufReceive,2);
  if (moduleType != 0x0D){
    return ERROR_LEDDAR_NO_RESPONSE;
  }
  if(!checkCRC(bufSend,bufReceive,size)) {
    return ERROR_LEDDAR_CRC;
  }
  return 0;
}

/** Construct the full spi package and check the CRC16
* \param[in] bufSend - header sent to the LeddarVu8
* \param[in] bufReceive - data received from the LedderVu8, including the CRC16
* \param[in] n - size of data package received
* \param[out] boolean - true if CRC check is OK, false if fails
*
*/
bool LeddarVu8Arduino::checkCRC(uint8_t* bufSend, uint8_t* bufReceive,size_t size) {
  uint8_t spiPackage[6+size+2];
  // Build the full package
  // header
  for (int j = 0; j < 6; j++) {
    spiPackage[j] = bufSend[j];
  }
  // data
  uint16_t i = 0;
  for (int j = 6; j < 6 + size + 2; j++){
    spiPackage[j] = bufReceive[i];
    i++;
  }

#if (LEDDAR_DEBUG == 1)
  // Let the CRC16 calculate the expected CRC16
  // CRC16(spiPackage,6+size,false);
  for (int j = 0; j < 6 + size + 2; j++) {
    Serial.print(spiPackage[j], HEX);
  }
  Serial.println();
#endif

  // Check the CRC16
  if(!CRC16(spiPackage,6+size,true)){
    return false;
  }
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
* \return bufSend - data sent to the sensor
*
*/
uint8_t * LeddarVu8Arduino::leddarCommand(uint8_t opcode, uint32_t address, size_t dataSize) {
  // select card
  if (!m_spiActive) {
    spiStart();
  }

  static uint8_t bufSend[6];
  // form message
  bufSend[0] = opcode;
  bufSend[1] = extractByte(address, 2);
  bufSend[2] = extractByte(address, 1);
  bufSend[3] = extractByte(address, 0);
  bufSend[4] = extractByte(dataSize, 1);
  bufSend[5] = extractByte(dataSize, 0);

#if (LEDDAR_DEBUG == 1)
  for (int j = 0; j < 6; j++) {
    Serial.print(bufSend[j], HEX);
  }
  //Serial.println();
#endif
  spiSend(bufSend,sizeof(bufSend));
  return bufSend;
}
/** Converts an array of bytes with the least significant byte (LSB) coming
*   first, to a 32-bit integer.
*   \param[in] buf - array of bytes with LSB coming first.
*   \param[in] n - size of the buffer.
*   \return 32-bit integer with the result of the convertion.
*/
uint32_t LeddarVu8Arduino::combineBytes(uint8_t* buf,size_t n) {
  uint32_t output = 0;

  for (int k = 0; k < n; k++) {
    // Convert from byte to 32-bit int
    uint32_t tmp = buf[k];
    output = output | (uint32_t)(tmp << (k * 8));
  }

  return output;
}

/** Read the raw echo values from the Leddar Vu8 sensor, i.e., raw distances and
*   amplitudes.
*   \param[out] distances - array of 8 raw distances
*   \param[out] amplitudes - array of 8 raw amplitudes
*   \return 0 if no errors, and a nonzero value if an error occured.
*/
uint8_t LeddarVu8Arduino::readRawEchoes(uint32_t* distances, uint32_t* amplitudes){
  size_t size = 96; // 8*12 bytes = 96
  uint8_t *bufSend;
  bufSend = leddarCommand(READ,LEDDARVU8_START_OF_DETECTION_LIST_ARRAYS,size);
  // delay 1 ms according to manual
  delay(1);

  // Receive data
  uint8_t bufReceive[size+2];
  spiReceive(bufReceive,size+2);
  #if (LEDDAR_DEBUG == 1)
  for (int j = 0; j < size+2; j++) {
    Serial.print(bufReceive[j], HEX);
  }
  Serial.println();
  #endif
  spiStop();

  // Check the CRC16
  if(!checkCRC(bufSend,bufReceive,size)){
    return ERROR_LEDDAR_CRC;
  } else {
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

      // Combine the bytes to a 32-bit number
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
    return 0;
  }
}
/** Read Echoes from the LeddarTech Vu8 sensor
*   \param[out] distances - scaled distance - in meters
*   \param[out] amplitudes - scaled amplitudes - in ??
*   \return nonzero value if error, and 0 if no error
*
*/
uint8_t LeddarVu8Arduino::readEchoes(float* distances, float* amplitudes){
  // Convert values received from readRawEchoes to scaled distance and amplitudes
  uint32_t rawDistance[8];
  uint32_t rawAmplitude[8];
  uint8_t check = readRawEchoes(rawDistance,rawAmplitude);
  if(check >= 0){
    for (int i = 0; i<8; i++){
      distances[i] = float(rawDistance[i])/float(DISTANCE_SCALE);
      amplitudes[i] = float(rawAmplitude[i]);
    }
    return 0;
  } else { // CRC check failed, we return 0 in distance and amplitude
    for (int i = 0; i<8; i++){
      distances[i] = float(0);
      amplitudes[i] = float(0);
    }
    return check;
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
/** Initialize the SPI bus.
*
* \param[in] csPin - LeddarVu8 chip select pin.
*/
void LeddarVu8Arduino::spiBegin(uint8_t csPin){
  m_csPin = csPin;
  digitalWrite(csPin, HIGH);
  pinMode(csPin, OUTPUT);
  SPI.begin();
}
/** Save SPISettings.
*
* \param[in] spiSettings SPI speed, mode, and byte order.
*/
void LeddarVu8Arduino::spiSetSpiSettings(SPISettings spiSettings){
  m_spiSettings = spiSettings;
}
/** Activate SPI hardware.
*
*/
void LeddarVu8Arduino::spiActivate(){
  SPI.beginTransaction(m_spiSettings);
}
/** Deactivate SPI hardware.
*
*/
void LeddarVu8Arduino::spiDeactivate(){
  SPI.endTransaction();
}
/** Receive a byte.
*
* \return The byte.
*/
uint8_t LeddarVu8Arduino::spiReceive(){
  return SPI.transfer(0XFF);
}
/** Receive multiple bytes.
*
* \param[out] buf Buffer to receive the data.
* \param[in] n Number of bytes to receive.
*
* \return Zero for no error or nonzero error code.
*/
uint8_t LeddarVu8Arduino::spiReceive(uint8_t* buf, size_t n){
  for (size_t i = 0; i < n; i++) {
    buf[i] = SPI.transfer(0XFF);
  }
  return 0;
}
/** Send a byte.
*
* \param[in] data Byte to send
*/
void LeddarVu8Arduino::spiSend(uint8_t data){
  SPI.transfer(data);
}
/** Send multiple bytes.
*
* \param[in] buf Buffer for data to be sent.
* \param[in] n Number of bytes to send.
*/
void LeddarVu8Arduino::spiSend(const uint8_t* buf, size_t n){
  for (size_t i = 0; i < n; i++) {
    SPI.transfer(buf[i]);
  }
}
/** Set CS low.
*
*/
void LeddarVu8Arduino::spiSelect(){
  digitalWrite(m_csPin, LOW);
}
/** Set CS high.
*
*/
void LeddarVu8Arduino::spiUnselect(){
  digitalWrite(m_csPin, HIGH);
}
