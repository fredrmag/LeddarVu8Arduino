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
* \return nonzero value if error, and 0 if no error
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
  size_t size = 2;
  uint8_t buf[size];
  uint8_t check = readLeddarCommand(READ,LEDDARVU8_MODULE_TYPE,size,buf);

  uint16_t moduleType = combineBytes(buf,2);

  if (moduleType != 0x0D){
    return ERROR_LEDDAR_NO_RESPONSE;
  }
  return check;
}

/** Construct the full spi package and check the CRC16
* \param[in] bufSend - header sent to the LeddarVu8
* \param[in] bufReceive - data received from the LedderVu8, including the CRC16
* \param[in] n - size of data package received
* \return boolean - true if CRC check is OK, false if fails
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
  for (uint8_t j = 6; j < 6 + size + 2; j++){
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
/** Send read command to leddar sensor through SPI
*   \param[in] address
*   \param[in] dataSize
*   \return bufSend - data sent to the sensor
*
*/
uint8_t * LeddarVu8Arduino::readCommand(uint8_t opcode, uint32_t address, size_t dataSize) {
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
/** Send write command to leddar sensor through SPI
*   \param[in] opcode
*   \param[in] address
*   \param[in] dataSize
*
*/
void LeddarVu8Arduino::writeCommand(uint8_t opcode, uint32_t address, size_t dataSize) {
  // select card
  if (!m_spiActive) {
    spiStart();
  }

  uint8_t bufSend[8];
  // form message
  bufSend[0] = opcode;
  bufSend[1] = extractByte(address, 2);
  bufSend[2] = extractByte(address, 1);
  bufSend[3] = extractByte(address, 0);
  bufSend[4] = extractByte(dataSize, 1);
  bufSend[5] = extractByte(dataSize, 0);
  // Create the CRC in the two last bytes
  CRC16(bufSend,6,false);


#if (LEDDAR_DEBUG == 1)
  for (int j = 0; j < 8; j++) {
    Serial.print(bufSend[j], HEX);
  }
  Serial.println();
#endif
  spiSend(bufSend,sizeof(bufSend));
  spiStop();
  delay(1);
}
/** Send write command to leddar sensor with data through SPI
*   \param[in] opcode
*   \param[in] address
*   \param[in] dataSize
*
*/
void LeddarVu8Arduino::writeCommand(uint8_t opcode, uint32_t address, size_t dataSize, uint8_t* buf) {
  // select card
  if (!m_spiActive) {
    spiStart();
  }

  uint8_t bufSend[6+dataSize+2];
  // form message
  bufSend[0] = opcode;
  bufSend[1] = extractByte(address, 2);
  bufSend[2] = extractByte(address, 1);
  bufSend[3] = extractByte(address, 0);
  bufSend[4] = extractByte(dataSize, 1);
  bufSend[5] = extractByte(dataSize, 0);

  // Populate with data
  for (uint8_t i = 6; i < (6+dataSize); i++ ){
    bufSend[i] = buf[i-6];
  }

  // Create the CRC in the two last bytes
  CRC16(bufSend,6+dataSize,false);


#if (LEDDAR_DEBUG == 1)
  for (int j = 0; j < 6+dataSize+2; j++) {
    Serial.print(bufSend[j], HEX);
  }
  Serial.println();
#endif
  spiSend(bufSend,sizeof(bufSend));
  spiStop();
  delay(1);
}
/** Converts an array of bytes with the least significant byte (LSB) coming
*   first, to a 32-bit integer.
*   \param[in] buf - array of bytes with LSB coming first.
*   \param[in] n - size of the buffer.
*   \return 32-bit integer with the result of the convertion.
*/
uint32_t LeddarVu8Arduino::combineBytes(uint8_t* buf,size_t n) {
  uint32_t output = 0;

  for (uint8_t k = 0; k < n; k++) {
    // Convert from byte to 32-bit int
    uint32_t tmp = buf[k];
    output = output | (uint32_t)(tmp << (k * 8));
  }

  return output;
}
/** Send read command to leddar sensor through SPI, catch the response and
*   quality check with CRC.
*   \param[in] address
*   \param[in] dataSize
*   \param[out] buf - answer from the unit in length of dataSize.
*   \return nonzero if error.
*
*/
uint8_t LeddarVu8Arduino::readLeddarCommand(uint8_t opcode, uint32_t address, size_t dataSize, uint8_t* buf){
  size_t size = dataSize; // 8*12 bytes = 96
  uint8_t *bufSend;

  bufSend = readCommand(opcode, address, size);
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
  delay(1);
  // Check the CRC16
  if(!checkCRC(bufSend,bufReceive,size)){
    return ERROR_LEDDAR_CRC;
  } else {
    for (uint8_t i=0; i<size; i++){
      buf[i] = bufReceive[i];
    }
    return 0;
  }
}

/** Modify a parameter by writing data to the leddar device. The manual gives
*   the following recipee:
*   1. Disable the write protection
*   2. Poll the status register to get the ready state and write enabled flag
*      asserted.
*   3. Send the new parameter value.
*   4. Poll the status register to get the ready state
*   5. Send the write disabled command (write protection) to prevent any
*      unwanted parameter change.
*
*   \param[in] address
*   \param[in] dataSize
*   \param[in] buf - value to be set
*   \return nonzero if error.
*
*/
uint8_t LeddarVu8Arduino::writeLeddarCommand(uint32_t address, size_t dataSize, uint8_t* buf){
    // 1. Disable write protection
    writeCommand(WREN,0,0);
    // 2. Poll the status register
    uint8_t response = get8uint(RDSR,0);
    if (response == 2){
      // 3. Send the new parameter value
      writeCommand(WRITE, address, dataSize, buf);
    } else {
      return ERROR_LEDDAR_FAILED_ENABLE_WRITE_PROTECTION;
    }
    // 4. Poll the status register to get the ready state
    // wait for the device to be ready
    while (get8uint(RDSR,0) > 2){
      delay(300);
    }
    response = get8uint(RDSR,0);
    if (response == 2){
      // 5. Enable write protection
      writeCommand(WRDIS,0,0);
    } else {
      return ERROR_LEDDAR_FAILED_ENABLE_WRITE_PROTECTION;
    }
    // Check if write protection is enabled
    response = get8uint(RDSR,0);
    if(response > 1){
      return ERROR_LEDDAR_FAILED_ENABLE_WRITE_PROTECTION;
    }
    return 0;
}

/** Read the raw echo values from the Leddar Vu8 sensor, i.e., raw distances and
*   amplitudes.
*   \param[out] distances - array of 8 raw distances
*   \param[out] amplitudes - array of 8 raw amplitudes
*   \return 0 if no errors, and a nonzero value if an error occured.
*/
uint8_t LeddarVu8Arduino::readRawEchoes(uint32_t* distances, uint32_t* amplitudes){
  size_t size = 96; // 8*12 bytes = 96
  uint8_t buf[size];
  uint8_t check = readLeddarCommand(READ,LEDDARVU8_START_OF_DETECTION_LIST_ARRAYS,size,buf);

  // Check the CRC16
  if(check > 0){
    return check;
  } else {
    // Parse the received buffer
    // The received buffer contains 8 segments that contains 12 bytes each
    for (int i = 0; i < 8; i++){
      uint8_t bufSegment[12];
      for (int j = i * 12; j < (i + 1) * 12; j++) {
        bufSegment[j - i * 12] = buf[j];
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
        Serial.print(buf[j], HEX);
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
      amplitudes[i] = float(rawAmplitude[i])/float(AMPLITUDE_SCALE);
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
//----------------------------------------------------------------------------
// Configuration data bank R/W

/** Get the Device name as an ASCII string of the Leddar device.
*   \param[out] deviceName
*   \see LEDDARVU8_DEVICE_NAME
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getDeviceName(char* deviceName){
  return get32char(LEDDARVU8_DEVICE_NAME,deviceName);
}
/** Get Accumulation exponent: I.e. 3 = 2^3 = 8
*   \see LEDDARVU8_ACCUMMULATION_EXP
*   \return Accumulation exponent
*/
uint8_t LeddarVu8Arduino::getAccumulationExponent(){
  return get8uint(LEDDARVU8_ACCUMMULATION_EXP);
}
/** Set Accumulation exponent: I.e. 3 = 2^3 = 8
*   \see LEDDARVU8_ACCUMMULATION_EXP
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setAccumulationExponent(uint8_t AccumulationExponent){
  return set8uint(LEDDARVU8_ACCUMMULATION_EXP, AccumulationExponent);
}
/** Get Oversampling exponent: I.e. 3 = 2^3 = 8
*   \see LEDDARVU8_OVERSSAMPLING_EXP
*   \return Oversampling exponent
*/
uint8_t LeddarVu8Arduino::getOversamplingExponent(){
  return get8uint(LEDDARVU8_OVERSSAMPLING_EXP);
}
/** Set Oversampling exponent: I.e. 3 = 2^3 = 8
*   \see LEDDARVU8_OVERSSAMPLING_EXP
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setOversamplingExponent(uint8_t oversamplingExponent){
  return set8uint(LEDDARVU8_OVERSSAMPLING_EXP, oversamplingExponent);
}
/** Get base point samples.
*   Base point sample determines how long the device will keep listening to get
*   a detection. Thus, it affects the distance. If we listens for more samples,
*   we can receive samples that are farther away, thus increasing the distance.
*   \see LEDDARVU8_BASE_POINT_COUNT
*   \return Base point sample
*/
uint8_t LeddarVu8Arduino::getBasePointSample(){
  return get8uint(LEDDARVU8_BASE_POINT_COUNT);
}
/** Set Base point sample
*   \see LEDDARVU8_BASE_POINT_COUNT
*   \see getBasePointSample()
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setBasePointSample(uint8_t basePointSample){
  return set8uint(LEDDARVU8_BASE_POINT_COUNT,basePointSample);
}
/** Get bit field of segment enabled
*   \see LEDDARVU8_SEGMENT_ENABLE
*   \return Bit field of segment enabled
*/
uint32_t LeddarVu8Arduino::getSegmentEnabled(){
  return get32uint(LEDDARVU8_SEGMENT_ENABLE);
}
/** Set bit field of segment enabled
*   \see LEDDARVU8_SEGMENT_ENABLE
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setSegmentEnabled(uint32_t segmentEnabled){
  return set32uint(LEDDARVU8_SEGMENT_ENABLE, segmentEnabled);
}
/** Get acquisiton rate of the reference pulse
*   \see LEDDARVU8_REF_PULSE_RATE
*   \return Acquisiton rate of the reference pulse
*/
uint32_t LeddarVu8Arduino::getReferencePulseRate(){
  return get32uint(LEDDARVU8_REF_PULSE_RATE);
}
/** Set acquisiton rate of the reference pulse
*   \see LEDDARVU8_REF_PULSE_RATE
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setReferencePulseRate(uint32_t referencePulseRate){
  return set32uint(LEDDARVU8_REF_PULSE_RATE, referencePulseRate);
}
/** Get yaw angle of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_YAW
*   \return Yaw angle of the module
*/
float LeddarVu8Arduino::getYaw(){
  return getFloat(LEDDARVU8_YAW);
}
/** Set yaw angle of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_YAW
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setYaw(float yaw){
  return setFloat(LEDDARVU8_YAW, yaw);
}
/** Get pitch angle of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_PITCH
*   \return Pitch angle of the module
*/
float LeddarVu8Arduino::getPitch(){
  return getFloat(LEDDARVU8_PITCH);
}
/** Set pitch angle of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_PITCH
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setPitch(float pitch){
  return setFloat(LEDDARVU8_PITCH, pitch);
}
/** Get roll angle of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_ROLL
*   \return Roll angle of the module
*/
float LeddarVu8Arduino::getRoll(){
  return getFloat(LEDDARVU8_ROLL);
}
/** Set roll angle of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_ROLL
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setRoll(float roll){
  return setFloat(LEDDARVU8_ROLL, roll);
}
/** Get X-axis poistion of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_X
*   \return X-axis poistion of the module
*/
float LeddarVu8Arduino::getXPos(){
  return getFloat(LEDDARVU8_X);
}
/** Set X-axis poistion of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_X
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setXPos(float xPos){
  return setFloat(LEDDARVU8_X, xPos);
}
/** Get Y-axis poistion of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_Y
*   \return Y-axis poistion of the module
*/
float LeddarVu8Arduino::getYPos(){
  return getFloat(LEDDARVU8_Y);
}
/** Set Y-axis poistion of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_Y
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setYPos(float yPos){
  return setFloat(LEDDARVU8_Y, yPos);
}
/** Get Z-axis poistion of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_Z
*   \return Z-axis poistion of the module
*/
float LeddarVu8Arduino::getZPos(){
  return getFloat(LEDDARVU8_Z);
}
/** Set Z-axis poistion of the module
*   - Has no purpose at the moment. To be implemented in the future by
*     LeddarTech
*   \see LEDDARVU8_Z
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setZPos(float zPos){
  return setFloat(LEDDARVU8_Z, zPos);
}
/** Get Precision (smoothing)
*   Stabilizes the module measurements. The behavior of the smoothing algorithm
*   can be adjusted by a value from -16 to 16.
*   \see LEDDARVU8_PRECISION
*   \return Precision (smoothing)
*/
int8_t LeddarVu8Arduino::getPrecision(){
  return get8int(LEDDARVU8_PRECISION);
}
/** Set Precision (smoothing)
*   \see getPrecision()
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setPrecision(int8_t precision){
  return set8int(LEDDARVU8_PRECISION, precision);
}
/** Get Precision (smoothing) enabled.
*   \see LEDDARVU8_PRECISION_ENABLE
*   \return Precision (smoothing) enabled
*/
uint8_t LeddarVu8Arduino::getPrecisionEnabled(){
  return get8uint(LEDDARVU8_PRECISION_ENABLE);
}
/** Set Precision (smoothing) enabled.
*   \see LEDDARVU8_PRECISION_ENABLE
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setPrecisionEnabled(uint8_t precisionEnabled){
  return set8uint(LEDDARVU8_PRECISION_ENABLE, precisionEnabled);
}
/** Get saturation compensation enabled.
*   \see LEDDARVU8_SAT_COMP_ENABLE
*   \return Saturation compensation enabled.
*/
uint8_t LeddarVu8Arduino::getSaturationCompensationEnabled(){
  return get8uint(LEDDARVU8_SAT_COMP_ENABLE);
}
/** Set saturation compensation enabled.
*   \see LEDDARVU8_SAT_COMP_ENABLE
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setSaturationCompensationEnabled(uint8_t saturationCompensationEnabled){
  return set8uint(LEDDARVU8_SAT_COMP_ENABLE, saturationCompensationEnabled);
}
/** Get overshoot managment enabled.
*   \see LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE
*   \return Overshoot managment enabled.
*/
uint8_t LeddarVu8Arduino::getOvershootManagementEnabled(){
  return get8uint(LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE);
}
/** Set overshoot managment enabled.
*   \see LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setOvershootManagementEnabled(uint8_t overshootManagementEnabled){
  return set8uint(LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE, overshootManagementEnabled);
}
/** Get Sensitivity (detection threshold) setting expressed in a raw amplitude
*   scale.
*   \see LEDDARVU8_SENSITIVITY
*   \return Overshoot managment enabled.
*/
int32_t LeddarVu8Arduino::getSensitivity(){
  return get32int(LEDDARVU8_SENSITIVITY);
}
/** Set Sensitivity (detection threshold) setting expressed in a raw amplitude
*   scale.
*   \see LEDDARVU8_SENSITIVITY
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setSensitivity(int32_t sensitivity){
  return set32int(LEDDARVU8_SENSITIVITY, sensitivity);
}
/** Get light source power (0 to 100)
*   \see LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT
*   \return Light source power (0 to 100)
*/
uint8_t LeddarVu8Arduino::getLightSourcePower(){
  return get8uint(LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT);
}
/** Set light source power (0 to 100)
*   \see LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setLightSourcePower(uint8_t lightSourcePower){
  return set8uint(LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT, lightSourcePower);
}
/** Get auto light power source enabled
*   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
*   \return Auto light power source enabled
*/
uint8_t LeddarVu8Arduino::getAutoLightSourcePowerEnabled(){
  return get8uint(LEDDARVU8_LED_USER_AUTO_POWER_ENABLE);
}
/** Set auto light power source enabled
*   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setAutoLightSourcePowerEnabled(uint8_t autoLightSourcePowerEnabled){
  return set8uint(LEDDARVU8_LED_USER_AUTO_POWER_ENABLE, autoLightSourcePowerEnabled);
}
/** Get auto frame average: Changes the delay in number of measurements. This is
*   the responsivity of the auto light source power according to the number
*   of frames.
*   \see LEDDARVU8_LED_USER_AUTO_FRAME_AVG
*   \return Auto frame average
*/
uint16_t LeddarVu8Arduino::getAutoFrameAverage(){
  return get16uint(LEDDARVU8_LED_USER_AUTO_FRAME_AVG);
}
/** Set auto frame average
*   \see LEDDARVU8_LED_USER_AUTO_FRAME_AVG
*   \see getAutoFrameAverage()
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setAutoFrameAverage(uint16_t autoFrameAverage){
  return set16uint(LEDDARVU8_LED_USER_AUTO_FRAME_AVG, autoFrameAverage);
}
/** Get Auto detections average: Number of detections for saturation acceptance
*   (the number of detections that can be saturated to avoid decreasing the
*   light source power when using the automatic mode). This is the responsivity
*   of the auto light source power according ro the number of detections.
*   \see LEDDARVU8_LED_USER_AUTO_ECHO_AVG
*   \return Auto detections average
*/
uint8_t LeddarVu8Arduino::getAutoDetectionsAverage(){
  return get8uint(LEDDARVU8_LED_USER_AUTO_ECHO_AVG);
}
/** Set Auto detections average
*   \see LEDDARVU8_LED_USER_AUTO_ECHO_AVG
*   \see getAutoDetectionsAverage()
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setAutoDetectionsAverage(uint8_t autoDetectionsAverage){
  return set8uint(LEDDARVU8_LED_USER_AUTO_ECHO_AVG, autoDetectionsAverage);
}
/** Get object demerging (DEM) enabled
*   \see LEDDARVU8_DEM_ENABLE
*   \return Object demerging (DEM) enabled
*/
uint8_t LeddarVu8Arduino::getObjectDemergingEnabled(){
  return get8uint(LEDDARVU8_DEM_ENABLE);
}
/** Set object demerging (DEM) enabled
*   \see LEDDARVU8_DEM_ENABLE
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setObjectDemergingEnabled(uint8_t objectDemergingEnabled){
  return set8uint(LEDDARVU8_DEM_ENABLE, objectDemergingEnabled);
}
/** Get Static noise removal enabled
*   \see LEDDARVU8_ST_NOISE_REMOVAL
*   \return Static noise removal enabled
*/
uint8_t LeddarVu8Arduino::getStaticNoiseRemovalEnabled(){
  return get8uint(LEDDARVU8_ST_NOISE_REMOVAL);
}
/** Set Static noise removal enabled
*   \see LEDDARVU8_ST_NOISE_REMOVAL
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setStaticNoiseRemovalEnabled(uint8_t staticNoiseRemovalEnabled){
  return set8uint(LEDDARVU8_ST_NOISE_REMOVAL, staticNoiseRemovalEnabled);
}
//----------------------------------------------------------------------------
// Device information and constants - Read only

/** Get the module number of the Leddar device.
*   \param[out] moduleNumber
*   \see LEDDARVU8_MODEL_PART_NUMBER
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getModulePartNumber(char* modulePartNumber){
  return get32char(LEDDARVU8_MODEL_PART_NUMBER,modulePartNumber);
}
/** Get the software part number of the Leddar device.
*   \param[out] softwarePartNumber
*   \see LEDDARVU8_SOFTWARE_PART_NUMBER
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getSoftwarePartNumber(char* softwarePartNumber){
  return get32char(LEDDARVU8_SOFTWARE_PART_NUMBER,softwarePartNumber);
}
/** Get the module serial number of the Leddar device.
*   \param[out] moduleSerialNumber
*   \see LEDDARVU8_MODULE_SERIAL_NUMBER
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getModuleSerialNumber(char* moduleSerialNumber){
  return get32char(LEDDARVU8_MODULE_SERIAL_NUMBER,moduleSerialNumber);
}
/** Get the manufacturer name of the Leddar device.
*   \param[out] manufacturerName
*   \see LEDDARVU8_MANUFACTURER_NAME
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getManufacturerName(char* manufacturerName){
  return get32char(LEDDARVU8_MANUFACTURER_NAME,manufacturerName);
}
/** Get the group identification number as a ASCII of the Leddar device.
*   \param[out] groupIdentificationNumber
*   \see LEDDARVU8_GROUP_IDENTIFICATION_NUMBER
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getGroupIdentificationNumber(char* groupIdentificationNumber){
  return get32char(LEDDARVU8_GROUP_IDENTIFICATION_NUMBER,groupIdentificationNumber);
}
/** Get the build date of the Leddar device.
*   \param[out] buildDate
*   \see LEDDARVU8_BUILD_DATE
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getBuildDate(char* buildDate){
  return get32char(LEDDARVU8_BUILD_DATE,buildDate);
}
/** Get the Firmware version of the Leddar device.
*   \param[out] firmwareVersion
*   \see LEDDARVU8_FIRMWARE_VERSION
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getFirmwareVersion(char* firmwareVersion){
  return get32char(LEDDARVU8_FIRMWARE_VERSION,firmwareVersion);
}
/** Get the Bootloader version of the Leddar device.
*   \param[out] bootloaderVersion
*   \see LEDDARVU8_BOOTLOADER_VERSION
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getBootloaderVersion(char* bootloaderVersion){
  return get32char(LEDDARVU8_BOOTLOADER_VERSION,bootloaderVersion);
}
/** Get the ASIC version of the Leddar device.
*   \param[out] asicVersion
*   \see LEDDARVU8_ASIC_VERSION
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getAsicVersion(char* asicVersion){
  return get32char(LEDDARVU8_ASIC_VERSION,asicVersion);
}
/** Get the FGPA version of the Leddar device.
*   \param[out] fgpaVersion
*   \see LEDDARVU8_FGPA_VERSION
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::getFgpaVersion(char* fgpaVersion){
  return get32char(LEDDARVU8_FGPA_VERSION,fgpaVersion);
}
/** Get the Module type of the Leddar device.
*   \see LEDDARVU8_MODULE_TYPE
*   \return Module type
*/
uint16_t LeddarVu8Arduino::getModuleType(){
  return get16uint(LEDDARVU8_MODULE_TYPE);
}
/** Get the Accumulation exponent min of the Leddar device.
*   \see LEDDARVU8_ACCUMULATION_EXP_MIN
*   \return Accumulation Exponent min
*/
uint8_t LeddarVu8Arduino::getAccumulationExponentMin(){
  return get8uint(LEDDARVU8_ACCUMULATION_EXP_MIN);
}
/** Get the Accumulation exponent max of the Leddar device.
*   \see LEDDARVU8_ACCUMULATION_EXP_MAX
*   \return Accumulation Exponent max
*/
uint8_t LeddarVu8Arduino::getAccumulationExponentMax(){
  return get8uint(LEDDARVU8_ACCUMULATION_EXP_MAX);
}
/** Get the Oversampling exponent min of the Leddar device.
*   \see LEDDARVU8_OVERSAMPLING_EXP_MIN
*   \return Oversampling Exponent min
*/
uint8_t LeddarVu8Arduino::getOversamplingExponentMin(){
  return get8uint(LEDDARVU8_OVERSAMPLING_EXP_MIN);
}
/** Get the Oversampling exponent max of the Leddar device.
*   \see LEDDARVU8_OVERSAMPLING_EXP_MAX
*   \return Oversampling Exponent max
*/
uint8_t LeddarVu8Arduino::getOversamplingExponentMax(){
  return get8uint(LEDDARVU8_OVERSAMPLING_EXP_MAX);
}
/** Get the Base point sample min of the Leddar device.
*   \see LEDDARVU8_BASE_POINT_SAMPLE_MIN
*   \return Base Point Sample min
*/
uint8_t LeddarVu8Arduino::getBasePointSampleMin(){
  return get8uint(LEDDARVU8_BASE_POINT_SAMPLE_MIN);
}
/** Get the Base point sample max of the Leddar device.
*   \see LEDDARVU8_BASE_POINT_SAMPLE_MAX
*   \return Base Point Sample max
*/
uint8_t LeddarVu8Arduino::getBasePointSampleMax(){
  return get8uint(LEDDARVU8_BASE_POINT_SAMPLE_MAX);
}
/** Get number of vertical segments of the Leddar device.
*   \see LEDDARVU8_NUMBER_OF_VERTICAL_SEGMENTS
*   \return Number of vertical segments
*/
uint16_t LeddarVu8Arduino::getNumberOfVerticalSegments(){
  return get16uint(LEDDARVU8_NUMBER_OF_VERTICAL_SEGMENTS);
}
/** Get number of horizontal segments of the Leddar device.
*   \see LEDDARVU8_NUMBER_OF_HORIZONTAL_SEGMENTS
*   \return Number of horizontal segments
*/
uint16_t LeddarVu8Arduino::getNumberOfHorizontalSegments(){
  return get16uint(LEDDARVU8_NUMBER_OF_HORIZONTAL_SEGMENTS);
}
/** Get number of reference segments of the Leddar device.
*   \see LEDDARVU8_NUMBER_OF_REFERENCE_SEGMENTS
*   \return Reference segments
*/
uint16_t LeddarVu8Arduino::getNumberOfReferenceSegments(){
  return get16uint(LEDDARVU8_NUMBER_OF_REFERENCE_SEGMENTS);
}
/** Get Base Point sample distance of the Leddar device.
*   \see LEDDARVU8_BASE_POINT_SAMPLE_DISTANCE
*   \return Base point sample distance
*/
uint32_t LeddarVu8Arduino::getBasePointSampleDistance(){
  return get32uint(LEDDARVU8_BASE_POINT_SAMPLE_DISTANCE);
}
/** Get Reference segment mask of the Leddar device.
*   \see LEDDARVU8_REFERENCE_SEGMENT_MASK
*   \return Reference segment mask
*/
uint32_t LeddarVu8Arduino::getReferenceSegmentMask(){
  return get32uint(LEDDARVU8_REFERENCE_SEGMENT_MASK);
}
/** Get max number of samples of the Leddar device.
*   \see LEDDARVU8_NUMBER_OF_SAMPLE_MAX
*   \return Number of sample max
*/
uint16_t LeddarVu8Arduino::getNumberOfSamplesMax(){
  return get16uint(LEDDARVU8_NUMBER_OF_SAMPLE_MAX);
}
/** Get clock frequency of the Leddar device.
*   \see LEDDARVU8_CLOCK_FREQUENCY
*   \return Clock frequency
*/
uint32_t LeddarVu8Arduino::getClockFrequency(){
  return get32uint(LEDDARVU8_CLOCK_FREQUENCY);
}
/** Get maximum number of detections per segment of the Leddar device.
*   \see LEDDARVU8_MAX_NUMBER_OF_DETEC_PER_SEGMENT
*   \return Maximum number of detections per segment
*/
uint8_t LeddarVu8Arduino::getMaxNumberOfDetectionsPerSegment(){
  return get8uint(LEDDARVU8_MAX_NUMBER_OF_DETEC_PER_SEGMENT);
}
/** Get distance scale of the Leddar device.
*   \see LEDDARVU8_DISTANCE_SCALE
*   \return Distance scale
*/
uint32_t LeddarVu8Arduino::getDistanceScale(){
  return get32uint(LEDDARVU8_DISTANCE_SCALE);
}
/** Get raw amplitude scale bit of the Leddar device.
*   To get amplitude: (Raw amplitude) << (scale bit + 0x0D)
*   \see LEDDARVU8_RAW_AMPLITUDE_SCALE_BIT
*   \return Raw amplitude scale bit
*/
uint8_t LeddarVu8Arduino::getRawAmplitudeScaleBit(){
  return get8uint(LEDDARVU8_RAW_AMPLITUDE_SCALE_BIT);
}
/** Get raw amplitude scale of the Leddar device.
*   To get amplitude: (Raw amplitude) / (raw amplitude scale + 8192)
*   \see LEDDARVU8_RAW_AMPLITUDE_SCALE
*   \return Raw amplitude scale
*/
uint32_t LeddarVu8Arduino::getRawAmplitudeScale(){
  return get32uint(LEDDARVU8_RAW_AMPLITUDE_SCALE);
}
/** Get precision min of the Leddar device.
*   \see LEDDARVU8_PRECISION_MIN
*   \return Precision min
*/
int16_t LeddarVu8Arduino::getPrecisionMin(){
  return get16int(LEDDARVU8_PRECISION_MIN);
}
/** Get precision max of the Leddar device.
*   \see LEDDARVU8_PRECISION_MAX
*   \return Precision max
*/
int16_t LeddarVu8Arduino::getPrecisionMax(){
  return get16int(LEDDARVU8_PRECISION_MAX);
}
/** Get sensitivity min of the Leddar device.
*   \see LEDDARVU8_SENSITIVITY_MIN
*   \return Sensitivity min
*/
int32_t LeddarVu8Arduino::getSensitivityMin(){
  return get32int(LEDDARVU8_SENSITIVITY_MIN);
}
/** Get sensitivity max of the Leddar device.
*   \see LEDDARVU8_SENSITIVITY_MAX
*   \return Sensitivity max
*/
int32_t LeddarVu8Arduino::getSensitivityMax(){
  return get32int(LEDDARVU8_SENSITIVITY_MAX);
}
/** Get current light source power count of the Leddar device.
*   \see LEDDARVU8_CURRENT_LIGHT_SOURCE_PWR_COUNT
*   \return Current light source power count
*/
uint8_t LeddarVu8Arduino::getCurrentLightSourcePowerCount(){
  return get8uint(LEDDARVU8_CURRENT_LIGHT_SOURCE_PWR_COUNT);
}
/** Get auto frame average min of the Leddar device.
*   \see LEDDARVU8_AUTO_FRAME_AVG_MIN
*   \return Auto frame average min
*/
uint16_t LeddarVu8Arduino::getAutoFrameAverageMin(){
  return get16uint(LEDDARVU8_AUTO_FRAME_AVG_MIN);
}
/** Get auto frame average max of the Leddar device.
*   \see LEDDARVU8_AUTO_FRAME_AVG_MAX
*   \return Auto frame average max
*/
uint16_t LeddarVu8Arduino::getAutoFrameAverageMax(){
  return get16uint(LEDDARVU8_AUTO_FRAME_AVG_MAX);
}
/** Get auto light source power percent min of the Leddar device.
*   \see LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MIN
*   \return Auto light source power percent min
*/
uint8_t LeddarVu8Arduino::getAutoLightSourcePowerPercentMin(){
  return get8uint(LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MIN);
}
/** Get auto light source power percent max of the Leddar device.
*   \see LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MAX
*   \return Auto light source power percent max
*/
uint8_t LeddarVu8Arduino::getAutoLightSourcePowerPercentMax(){
  return get8uint(LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MAX);
}
/** Get auto detections average min of the Leddar device.
*   \see LEDDARVU8_AUTO_DETECTION_AVG_MIN
*   \return Auto detections average min
*/
uint8_t LeddarVu8Arduino::getAutoDetectionsAverageMin(){
  return get8uint(LEDDARVU8_AUTO_DETECTION_AVG_MIN);
}
/** Get auto detections average max of the Leddar device.
*   \see LEDDARVU8_AUTO_DETECTION_AVG_MAX
*   \return Auto detections average max
*/
uint8_t LeddarVu8Arduino::getAutoDetectionsAverageMax(){
  return get8uint(LEDDARVU8_AUTO_DETECTION_AVG_MAX);
}
/** Get static noise calibration source of the Leddar device.
*   0 = By end-user
*   1 = By factory
*   \see LEDDARVU8_STATIC_NOISE_CALIBRATION_SOURCE
*   \return Static noise calibration source
*/
uint8_t LeddarVu8Arduino::getStaticNoiseCalibrationSource(){
  return get8uint(LEDDARVU8_STATIC_NOISE_CALIBRATION_SOURCE);
}
/** Get CPU load scale of the Leddar device.
*   \see LEDDARVU8_CPU_LOAD_SCALE
*   \return CPU load scale
*/
uint32_t LeddarVu8Arduino::getCPUloadScale(){
  return get32uint(LEDDARVU8_CPU_LOAD_SCALE);
}
/** Get temperature scale of the Leddar device.
*   \see LEDDARVU8_TEMPERATURE_SCALE
*   \return Temperature scale
*/
uint32_t LeddarVu8Arduino::getTemperatureScale(){
  return get32uint(LEDDARVU8_TEMPERATURE_SCALE);
}
//-----------------------------------------------------------------------------
// Detection list bank: R only

/** Get Timestamp in ms since power up
*   \see LEDDARVU8_TIMESTAMP
*   \return Timestamp
*/
uint32_t LeddarVu8Arduino::getTimestamp(){
  return get32uint(LEDDARVU8_TIMESTAMP);
}
/** Get number of detections (N)
*   \see LEDDARVU8_NUMBER_OF_DETECTIONS
*   \return Number of detections (N)
*/
uint16_t LeddarVu8Arduino::getNumberOfDetections(){
  return get16uint(LEDDARVU8_NUMBER_OF_DETECTIONS);
}
/** Get current percentage of light source power
*   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
*   \return Current percentage of light source power
*/
uint16_t LeddarVu8Arduino::getCurrentPercentageOfLightSourcePower(){
  return get16uint(LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR);
}
/** Get acquisiton options
*   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
*   \return Acquisiton options
*/
uint32_t LeddarVu8Arduino::getAcquisitionOptions(){
  return get32uint(LEDDARVU8_ACQUISITION_OPTIONS);
}

//-----------------------------------------------------------------------------
// Transaction configuration bank - R/W

/** Get secure transaction enabled flags: <br />
*   1 = Enables the CRC calculation and validation on any transaction. This flag is enabled by default.<br />
*   0 = No CRC validation. The CRC field is still required in SPI protocol but can be set to any value.
*   \see LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS
*   \return Secure transaction enabled flags
*/
uint8_t LeddarVu8Arduino::getSecureTransactionEnabledFlags(){
  return get8uint(LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS);
}
/** Set secure transaction enabled flags:
*   \see LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS
*   \see getSecureTransactionEnabledFlags()
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::setSecureTransactionEnabledFlags(uint8_t secureTransactionEnabledFlags){
  return set8uint(LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS, secureTransactionEnabledFlags);
}
/** Get transaction mode: <br />
*   0 = Free run. The READY pin is asserted on each ready detection frame. The host must be able to read data on time. <br />
*   1 = Blocking read. On the READY pin assertion, host must read all data from traces or detections bank (data transaction control source configuration) to continue acquisition. <br />
*   2 = Partial blocking read. On the READY pin assertion, host can read all data from traces of the detection bank and the acquisition is still running. Possible loss of detection frames if the host reading data is very long.
*   \see LEDDARVU8_TRANSACTION_MODES
*   \return Transaction mode
*/
uint8_t LeddarVu8Arduino::getTransactionMode(){
  return get8uint(LEDDARVU8_TRANSACTION_MODE);
}
/** Set transaction mode
*   \see LEDDARVU8_TRANSACTION_MODE
*   \see getTransactionMode()
*   \return nonzero for error
*/
uint8_t LeddarVu8Arduino::setTransactionMode(uint8_t transactionMode){
  return set8uint(LEDDARVU8_TRANSACTION_MODE, transactionMode);
}
/** Get CRC of the last transaction
*   \see LEDDARVU8_CRC_OF_THE_LAST_TRANSACTION
*   \return CRC of last transaction
*/
uint16_t LeddarVu8Arduino::getCRClastTransaction(){
  return get16uint(LEDDARVU8_CRC_OF_THE_LAST_TRANSACTION);
}
/** Get bit-field information of last transaction: <br/>
*   All bits to 0: No transaction error <br />
*   Bit-0: Access right violation <br />
*   Bit-1: Invalid address <br />
*   Bit-2: Command not found <br />
*   Bit-3: Write disabled <br />
*   Bit-4: CRC failed <br />
*   Bit-5: Command execution error <br />
*   Bit-6: Invalid packet
*   \see LEDDARVU8_INFORMATION_LAST_TRANSACTION
*   \return Bit-field information of last transaction
*/
uint16_t LeddarVu8Arduino::getBitFieldInformationOfLastTransaction(){
  return get16uint(LEDDARVU8_INFORMATION_LAST_TRANSACTION);
}
/** Get data transaction control source: <br />
*    0 = On trace  <br />
*    1 = On detections  <br />
*    This register determines which data type will control the READY pin and manage the transaction mode.
*   \see LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE
*   \return Data transaction control source
*/
uint8_t LeddarVu8Arduino::getDataTransactionControlSource(){
  return get8uint(LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE);
}
/** Set data transaction control source <br />
*   \see LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE
*   \see getDataTransactionControlSource()
*   \return nonzero if error
*/
uint8_t LeddarVu8Arduino::setDataTransactionControlSource(uint8_t dataTransactionControlSource){
  return set8uint(LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE, dataTransactionControlSource);
}
//-----------------------------------------------------------------------------
// helper function get8uint
uint8_t LeddarVu8Arduino::get8uint(uint32_t address){
  size_t size = 1;
  uint8_t buf[size];

  readLeddarCommand(READ,address,size,buf);
  uint8_t bufInt = buf[0];

  #if (LEDDAR_DEBUG == 1)
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}
// helper function get8uint
uint8_t LeddarVu8Arduino::get8uint(uint8_t opcode, uint32_t address){
  size_t size = 1;
  uint8_t buf[size];
  readLeddarCommand(opcode,address,size,buf);
  uint8_t bufInt = buf[0];

  #if (LEDDAR_DEBUG == 1)
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}

// helper function get16uint
uint16_t LeddarVu8Arduino::get16uint(uint32_t address){
  size_t size = 2;
  uint8_t buf[size];
  readLeddarCommand(READ,address,size,buf);
  uint16_t bufInt = combineBytes(buf,2);

  #if (LEDDAR_DEBUG == 1)
      for (uint8_t j = 0; j < size; j++) {
        Serial.print(buf[j],HEX);
      }
      Serial.println();
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}

// helper function get32uint
uint32_t LeddarVu8Arduino::get32uint(uint32_t address){
  size_t size = 4;
  uint8_t buf[size];
  readLeddarCommand(READ,address,size,buf);
  uint32_t bufInt = combineBytes(buf,4);

  #if (LEDDAR_DEBUG == 1)
      for (uint8_t j = 0; j < size; j++) {
        Serial.print(buf[j],HEX);
      }
      Serial.println();
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}
// helper function get8int
int8_t LeddarVu8Arduino::get8int(uint32_t address){
  size_t size = 1;
  uint8_t buf[size];
  readLeddarCommand(READ,address,size,buf);
  int8_t bufInt = combineBytes(buf,size);

  #if (LEDDAR_DEBUG == 1)
      for (uint8_t j = 0; j < size; j++) {
        Serial.print(buf[j],HEX);
      }
      Serial.println();
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}

// helper function get16int
int16_t LeddarVu8Arduino::get16int(uint32_t address){
  size_t size = 2;
  uint8_t buf[size];
  readLeddarCommand(READ,address,size,buf);
  int16_t bufInt = combineBytes(buf,size);

  #if (LEDDAR_DEBUG == 1)
      for (uint8_t j = 0; j < size; j++) {
        Serial.print(buf[j],HEX);
      }
      Serial.println();
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}
// helper function get32int
int32_t LeddarVu8Arduino::get32int(uint32_t address){
  size_t size = 4;
  uint8_t buf[size];
  readLeddarCommand(READ,address,size,buf);
  int32_t bufInt = combineBytes(buf,size);

  #if (LEDDAR_DEBUG == 1)
      for (uint8_t j = 0; j < size; j++) {
        Serial.print(buf[j],HEX);
      }
      Serial.println();
      Serial.println(bufInt,HEX);
  #endif
  delay(1);
  return bufInt;
}

// helper function getFloat
float LeddarVu8Arduino::getFloat(uint32_t address){
  size_t size = 4;
  uint8_t buf[size];
  readLeddarCommand(READ,address,size,buf);
  leddarData.i = combineBytes(buf,size);

  #if (LEDDAR_DEBUG == 1)
      for (uint8_t j = 0; j < size; j++) {
        Serial.print(buf[j],HEX);
      }
      Serial.println();
      Serial.println(leddarData.i,HEX);
  #endif
  delay(1);
  return leddarData.f;
}

// helper function get32char
uint8_t LeddarVu8Arduino::get32char(uint32_t address, char* bufChar){
  size_t size = 32; // 32
  uint8_t buf[size];
  uint8_t check = readLeddarCommand(READ,address,size,buf);
#if (LEDDAR_DEBUG == 1)
    for (uint8_t j = 0; j < 32; j++) {
      Serial.print((char) buf[j]);
    }
    Serial.println();
#endif
  // Convert to char
  for (int j = 0; j < 32; j++) {
    bufChar[j] = buf[j];
  }
  delay(1);
  return check;
}

// helper function set8uint
uint8_t LeddarVu8Arduino::set8uint(uint32_t address, uint8_t value){
  size_t size = 1; // 1
  uint8_t *buf = (uint8_t *)&value;

  uint8_t check = writeLeddarCommand(address,size,buf);
  return check;
}
// helper function set16uint
uint8_t LeddarVu8Arduino::set16uint(uint32_t address, uint16_t value){
  size_t size = 2; // 1
  uint8_t *buf = (uint8_t *)&value;

  uint8_t check = writeLeddarCommand(address,size,buf);
  return check;
}
// helper function set32uint
uint8_t LeddarVu8Arduino::set32uint(uint32_t address, uint32_t value){
  size_t size = 4; // 1
  uint8_t *buf = (uint8_t *)&value;

  uint8_t check = writeLeddarCommand(address,size,buf);
  return check;
}

// helper function set8int
uint8_t LeddarVu8Arduino::set8int(uint32_t address, int8_t value){
  size_t size = 1; // 1
  uint8_t *buf = (uint8_t *)&value;

  uint8_t check = writeLeddarCommand(address,size,buf);
  return check;
}
// helper function set32int
uint8_t LeddarVu8Arduino::set32int(uint32_t address, int32_t value){
  size_t size = 4; // 1
  uint8_t *buf = (uint8_t *)&value;

  uint8_t check = writeLeddarCommand(address,size,buf);
  return check;
}

// helper function setFloat
uint8_t LeddarVu8Arduino::setFloat(uint32_t address, float value){
  size_t size = 4; // 1
  uint8_t *buf = (uint8_t *)&value;

  uint8_t check = writeLeddarCommand(address,size,buf);
  return check;
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
