/* Arduino LeddarTechVu8 library
*  Copyright (C) 2017 by PSS-GEO AS
*
*  \file       LeddarVu8Arduino.h
*  \brief      Arduino Library for the LeddarTech Vu8 SPI sensor
*  \author     Fredrik Magnussen
*  \since      March 2018
*
*/

// To prevent from including library twice
#ifndef LeddarVu8Arduino_h
#define LeddarVu8Arduino_h

#include <SPI.h>

#define LEDDAR_DEBUG 0

#define DISTANCE_SCALE 65536
#define AMPLITUDE_SCALE 8256

// SPI protocol - Leddar Vu8 constants - p.37-48 in Leddar Vu8 User guide
// Opcodes for SPI communication
#define READ        0x0B // Read data
#define WRITE       0x02 // Write data
#define RDSR        0x05 // Read status register
#define CE          0xC7 // Reset configuration
#define WREN        0x06 // Write enabled
#define WRDIS       0x04 // Write disabled
#define SOFTRST     0x99 // Software reset

// Memory banks for SPI communications
#define REGMAP_CFG_DATA     0x000000  // Configuration Data                 R/W
#define REGMAP_DEV_INFO     0x400000  // Device information and constants   Read only
#define REGMAP_DETECTIONS   0x500000  // Detection list                     Read only
#define REGMAP_TRN_CGF      0xFFFB00  // Transaction configuration          R/W

// REGMAP_CFG_DATA - R/W - 0x000000
#define LEDDARVU8_DEVICE_NAME                        0x00 | REGMAP_CFG_DATA // Module name as an ASCII string          char[32]
#define LEDDARVU8_ACCUMMULATION_EXP                  0x20 | REGMAP_CFG_DATA // Accumulation exponent: 3 => 2^3 = 8     uint8_t
#define LEDDARVU8_OVERSSAMPLING_EXP                  0x21 | REGMAP_CFG_DATA // Oversampling exponent: 3 => 2^3 = 8     uint8_t
#define LEDDARVU8_BASE_POINT_COUNT                   0x22 | REGMAP_CFG_DATA // Base point sample                       uint8_t
#define LEDDARVU8_SEGMENT_ENABLE                     0x23 | REGMAP_CFG_DATA // Bit field of segment enabled            uint32_t
#define LEDDARVU8_REF_PULSE_RATE                     0x27 | REGMAP_CFG_DATA // Aquisition rate of the reference pulse  uint32_t
#define LEDDARVU8_YAW                                0x2B | REGMAP_CFG_DATA // Yaw angle of the module                 float -> 4 bytes
#define LEDDARVU8_PITCH                              0x2F | REGMAP_CFG_DATA // Pitch angle of the module               float
#define LEDDARVU8_ROLL                               0x33 | REGMAP_CFG_DATA // Roll angle of the module                float
#define LEDDARVU8_X                                  0x37 | REGMAP_CFG_DATA // X-position of the module                float
#define LEDDARVU8_Y                                  0x3B | REGMAP_CFG_DATA // Y-position of the module                float
#define LEDDARVU8_Z                                  0x3F | REGMAP_CFG_DATA // Z-position of the module                float
#define LEDDARVU8_PRECISION                          0x43 | REGMAP_CFG_DATA // Precision (Smoothing):                  int8_t
// Stabilizes the module measurements. The behavior of the smoothing algorithm can be adjusted by a value ranging from -16 to 16.
#define LEDDARVU8_PRECISION_ENABLE                   0x44 | REGMAP_CFG_DATA // Precision enabled                       uint8_t
#define LEDDARVU8_SAT_COMP_ENABLE                    0x45 | REGMAP_CFG_DATA // Saturation compensation enabled         uint8_t
#define LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE        0x46 | REGMAP_CFG_DATA // Overshoot managment enabled             uint8_t
#define LEDDARVU8_SENSITIVITY                        0x47 | REGMAP_CFG_DATA // Sensitivity (detection threshold)       int32_t
// setting expressed in raw amplitude scale.
#define LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT     0x4B | REGMAP_CFG_DATA // Light source power (0 to 100)           uint8_t
#define LEDDARVU8_LED_USER_AUTO_POWER_ENABLE         0x4C | REGMAP_CFG_DATA // Auto light source power enabled         uint8_t
#define LEDDARVU8_LED_USER_AUTO_FRAME_AVG            0x4D | REGMAP_CFG_DATA // Auto frame average:                     uin16_t
// Changes the delay in the number of measurements.
// This is the responsivity of the auto light source power according to the number of frames.
#define LEDDARVU8_LED_USER_AUTO_ECHO_AVG             0x4F | REGMAP_CFG_DATA // Auto detections average:                uint8_t
// Number of detections for saturation acceptance (the number of detections can be saturated to avoid decreasing the light source power when using the automatic mode). This is the responsivity of the auto light source power according to the number of detections.
#define LEDDARVU8_DEM_ENABLE                         0x50 | REGMAP_CFG_DATA // Object demerging enabled                uint8_t
#define LEDDARVU8_ST_NOISE_REMOVAL                   0x51 | REGMAP_CFG_DATA // Static noise removal enabled            uint8_t

// REGMAP_DEV_INFO - R - 0x400000
#define LEDDARVU8_MODEL_PART_NUMBER                  0x0000 | REGMAP_DEV_INFO // Module part number as an ASCII string
#define LEDDARVU8_SOFTWARE_PART_NUMBER               0x0020 | REGMAP_DEV_INFO // Software part number as an ASCII string
#define LEDDARVU8_MODULE_SERIAL_NUMBER               0x0040 | REGMAP_DEV_INFO // Module serial number as an ASCII string
#define LEDDARVU8_MANUFACTURER_NAME                  0x0060 | REGMAP_DEV_INFO // Manufacturer name as an ASCII string
#define LEDDARVU8_GROUP_IDENTIFICATION_NUMBER        0x0080 | REGMAP_DEV_INFO // Group identification number as an ASCII string
#define LEDDARVU8_BUILD_DATE                         0x00A0 | REGMAP_DEV_INFO // Build date as an ASCII string
#define LEDDARVU8_FIRMWARE_VERSION                   0x00C0 | REGMAP_DEV_INFO // Firmware version as an ASCII string
#define LEDDARVU8_BOOTLOADER_VERSION                 0x00E0 | REGMAP_DEV_INFO // Bootloader version as an ASCII string
#define LEDDARVU8_ASIC_VERSION                       0x0100 | REGMAP_DEV_INFO // ASIC version as an ASCII string
#define LEDDARVU8_FGPA_VERSION                       0x0120 | REGMAP_DEV_INFO // FPGA version as an ASCII string
#define LEDDARVU8_MODULE_TYPE                        0x0140 | REGMAP_DEV_INFO // Module type - 0x00 Invalid device; 0x07 M16 Evaluation Kit; 0x08 IS16; 0x09 M16; 0x0A Leddar One; 0x0D Leddar Vu8;
#define LEDDARVU8_INTERNAL_USE1                      0x0142 | REGMAP_DEV_INFO // Internal Use
#define LEDDARVU8_ACCUMULATION_EXP_MIN               0x0146 | REGMAP_DEV_INFO // Accumulation exponent min
#define LEDDARVU8_ACCUMULATION_EXP_MAX               0x0147 | REGMAP_DEV_INFO // Accumulation exponent max
#define LEDDARVU8_OVERSAMPLING_EXP_MIN               0x0148 | REGMAP_DEV_INFO // Oversampling exponent min
#define LEDDARVU8_OVERSAMPLING_EXP_MAX               0x0149 | REGMAP_DEV_INFO // Oversampling exponent max
#define LEDDARVU8_BASE_POINT_SAMPLE_MIN              0x014A | REGMAP_DEV_INFO // Base point sample min
#define LEDDARVU8_BASE_POINT_SAMPLE_MAX              0x014B | REGMAP_DEV_INFO // Base poing sample max
#define LEDDARVU8_NUMBER_OF_VERTICAL_SEGMENTS        0x014C | REGMAP_DEV_INFO // Number of vertical segments
#define LEDDARVU8_NUMBER_OF_HORIZONTAL_SEGMENTS      0x014E | REGMAP_DEV_INFO // Number of horizontal segments
#define LEDDARVU8_NUMBER_OF_REFERENCE_SEGMENTS       0x0150 | REGMAP_DEV_INFO // Number of reference segments
#define LEDDARVU8_BASE_POINT_SAMPLE_DISTANCE         0x0152 | REGMAP_DEV_INFO // Base point sample distance
#define LEDDARVU8_REFERENCE_SEGMENT_MASK             0x0156 | REGMAP_DEV_INFO // Reference segment mask: bit-field mask indicates the postition of the reference segments
#define LEDDARVU8_NUMBER_OF_SAMPLE_MAX               0x015A | REGMAP_DEV_INFO // Number of sample max
#define LEDDARVU8_INTERNAL_USE2                      0x015C | REGMAP_DEV_INFO // Internal Use
#define LEDDARVU8_CLOCK_FREQUENCY                    0x015D | REGMAP_DEV_INFO // Clock frequency
#define LEDDARVU8_MAX_NUMBER_OF_DETEC_PER_SEGMENT    0x0161 | REGMAP_DEV_INFO // Maximum number of detections per segment
#define LEDDARVU8_DISTANCE_SCALE                     0x0162 | REGMAP_DEV_INFO // Distance scale
#define LEDDARVU8_RAW_AMPLITUDE_SCALE_BIT            0x0166 | REGMAP_DEV_INFO // Raw amplitude scale bit, to which 0xd must be added (amplitude scale given in bitshift). ie raw amplitude << (scale bit + 0x0d)
#define LEDDARVU8_RAW_AMPLITUDE_SCALE                0x0167 | REGMAP_DEV_INFO // Raw amplitude scale, to which the value 8192 must be added.
#define LEDDARVU8_PRECISION_MIN                      0x016B | REGMAP_DEV_INFO // Precision min.
#define LEDDARVU8_PRECISION_MAX                      0x016D | REGMAP_DEV_INFO // Precision max.
#define LEDDARVU8_SENSITIVITY_MIN                    0x016F | REGMAP_DEV_INFO // Sensitivity min.
#define LEDDARVU8_SENSITIVITY_MAX                    0x0173 | REGMAP_DEV_INFO // Sensitivity max.
#define LEDDARVU8_CURRENT_LIGHT_SOURCE_PWR_COUNT     0x0177 | REGMAP_DEV_INFO // Current light source power count (max 16)
#define LEDDARVU8_AUTO_FRAME_AVG_MIN                 0x0178 | REGMAP_DEV_INFO // Auto frame average min.
#define LEDDARVU8_AUTO_FRAME_AVG_MAX                 0x017A | REGMAP_DEV_INFO // Auto frame average max.
#define LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MIN       0x017C | REGMAP_DEV_INFO // Auto light source power percent min.
#define LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MAX       0x017D | REGMAP_DEV_INFO // Auto light source power percent max.
#define LEDDARVU8_AUTO_DETECTION_AVG_MIN             0x017E | REGMAP_DEV_INFO // Auto detections average min.
#define LEDDARVU8_AUTO_DETECTION_AVG_MAX             0x017F | REGMAP_DEV_INFO // Auto detections average max.
#define LEDDARVU8_STATIC_NOISE_CALIBRATION_SOURCE    0x0180 | REGMAP_DEV_INFO // Static noise calibration source:: 0 - By end-user; 1 - By factory
#define LEDDARVU8_CPU_LOAD_SCALE                     0x0181 | REGMAP_DEV_INFO // CPU load scale
#define LEDDARVU8_TEMPERATURE_SCALE                  0x0185 | REGMAP_DEV_INFO // Temperature scale

// REGMAP_DETECTIONS - R - 0x500000
#define LEDDARVU8_TIMESTAMP                          0x00 | REGMAP_DETECTIONS // Timestamp: in ms since power up
#define LEDDARVU8_NUMBER_OF_DETECTIONS                0x04 | REGMAP_DETECTIONS // Number of detections (N)
#define LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR           0x06 | REGMAP_DETECTIONS // Current percentage of light source power
#define LEDDARVU8_ACQUISITION_OPTIONS                0x08 | REGMAP_DETECTIONS // Acquisition options
#define LEDDARVU8_START_OF_DETECTION_LIST_ARRAYS     0x0C | REGMAP_DETECTIONS // Start of detection list array length: N*12 bytes

// REGMAP_TRN_CGF - R/W - 0xFFFB00
#define LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS   0x00 | REGMAP_TRN_CGF // Secure-transaction enabled flags:
//1 = Enables the CRC calculation and validation on any transaction. This flag is enabled by default.
//0 = No CRC validation. The CRC field is still required in SPI protocol but can be set to any value.
#define LEDDARVU8_TRANSACTION_MODE                  0x01 | REGMAP_TRN_CGF // Transaction modes:
// 0 = Free run. The READY pin is asserted on each ready detection frame. The host must be able to read data on time.
// 1 = Blocking read. On the READY pin assertion, host must read all data from traces or detections bank (data transaction control source configuration) to continue acquisition.
// 2 = Partial blocking read. On the READY pin assertion, host can read all data from traces of the detection bank and the acquisition is still running. Possible loss of detection frames if the host reading data is very long.
#define LEDDARVU8_CRC_OF_THE_LAST_TRANSACTION        0x02 | REGMAP_TRN_CGF // CRC of the last transaction
#define LEDDARVU8_INFORMATION_LAST_TRANSACTION       0x04 | REGMAP_TRN_CGF // Bit-field information of last transactions:
//All bits to 0: No transaction error
// Bit-0: Access right violation
// Bit-1: Invalid address
// Bit-2: Command not found
// Bit-3: Write disabled
// Bit-4: CRC failed
// Bit-5: Command execution error
// Bit-6: Invalid packet
#define LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE    0x06 | REGMAP_TRN_CGF // Data transaction control source:
// 0 = On trace
// 1 = On detections
// This register determines which data type will control the READY pin and manage the transaction mode.

// Error codes
#define ERROR_LEDDAR_CRC 1
#define ERROR_LEDDAR_NO_RESPONSE 2
#define ERROR_LEDDAR_BUSY 3
#define ERROR_LEDDAR_FAILED_ENABLE_WRITE_PROTECTION 4

/**
* \class LeddarVu8Arduino
* \brief LeddarVu8Arduino class
*/
class LeddarVu8Arduino {
public:
  /** Initialize the LeddarVu8 sensor.
  *   \param[in] csPin - LeddarTechVu8 chip select pin.
  *   \return nonzero value if error, and 0 if no error
  */
  uint8_t begin(uint8_t csPin);
  /** Extract byte from a 32-bit number, given number and byte-place
  *   \param[in] number - number to extract byte from
  *   \param[in] place - byte place from right, starts at 0
  *   \return byte - corresponding byte for given number and byte place
  */
  byte extractByte(uint32_t number, int place);
  /** Send read command to leddar sensor through SPI
  *   \param[in] address
  *   \param[in] dataSize
  *   \return bufSend - data sent to the sensor
  *
  */
  uint8_t * readCommand(uint8_t opcode, uint32_t address, size_t dataSize);
  /** Send read command to leddar sensor through SPI, catch the response and
  *   quality check with CRC.
  *   \param[in] address
  *   \param[in] dataSize
  *   \param[out] buf - answer from the unit in length of dataSize.
  *   \return nonzero if error.
  *
  */
  uint8_t readLeddarCommand(uint8_t opcode, uint32_t address, size_t dataSize, uint8_t* buf);
  void writeCommand(uint8_t opcode, uint32_t address, size_t dataSize);
  void writeCommand(uint8_t opcode, uint32_t address, size_t dataSize, uint8_t* buf);
  uint8_t writeLeddarCommand(uint32_t address, size_t dataSize, uint8_t* buf);
  /** Read the raw echo values from the Leddar Vu8 sensor, i.e., raw distances and
  *   amplitudes.
  *   \param[out] distances - array of 8 raw distances
  *   \param[out] amplitudes - array of 8 raw amplitudes
  *   \return 0 if no errors, and a nonzero value if an error occured.
  */
  uint8_t readRawEchoes(uint32_t* distances, uint32_t* amplitudes);
  /** Read Echoes from the LeddarTech Vu8 sensor
  *   \param[out] distances - scaled distance - in meters
  *   \param[out] amplitudes - scaled amplitudes - in ??
  *   \return nonzero value if error, and 0 if no error
  *
  */
  uint8_t readEchoes(float* distances, float* amplitudes);
  /** Construct the full spi package and check the CRC16
  * \param[in] bufSend - header sent to the LeddarVu8
  * \param[in] bufReceive - data received from the LedderVu8, including the CRC16
  * \param[in] n - size of data package received
  * \return boolean - true if CRC check is OK, false if fails
  *
  */
  bool checkCRC(uint8_t* bufSend, uint8_t* bufReceive,size_t size);
  /** Converts an array of bytes with the least significant byte (LSB) coming
  *   first, to a 32-bit integer.
  *   \param[in] buf - array of bytes with LSB coming first.
  *   \param[in] n - size of the buffer.
  *   \return 32-bit integer with the result of the convertion.
  */
  uint32_t combineBytes(uint8_t* buf,size_t n);
  //----------------------------------------------------------------------------
  // Configuration data bank R/W
  /** Get the Device name as an ASCII string of the Leddar device.
  *   \param[out] deviceName
  *   \see LEDDARVU8_DEVICE_NAME
  *   \return nonzero for error
  */
  uint8_t getDeviceName(char* deviceName);
  /** Get Accumulation exponent: I.e. 3 = 2^3 = 8
  *   \see LEDDARVU8_ACCUMMULATION_EXP
  *   \return Accumulation exponent
  */
  uint8_t getAccumulationExponent();
  /** Set Accumulation exponent: I.e. 3 = 2^3 = 8
  *   \see LEDDARVU8_ACCUMMULATION_EXP
  *   \return nonzero if error
  */
  uint8_t setAccumulationExponent(uint8_t AccumulationExponent);
  /** Get Oversampling exponent: I.e. 3 = 2^3 = 8
  *   \see LEDDARVU8_OVERSSAMPLING_EXP
  *   \return Oversampling exponent
  */
  uint8_t getOversamplingExponent();
  /** Set Oversampling exponent: I.e. 3 = 2^3 = 8
  *   \see LEDDARVU8_OVERSSAMPLING_EXP
  *   \return nonzero if error
  */
  uint8_t setOversamplingExponent(uint8_t oversamplingExponent);
  /** Get base point samples.
  *   Base point sample determines how long the device will keep listening to get
  *   a detection. Thus, it affects the distance. If we listens for more samples,
  *   we can receive samples that are farther away, thus increasing the distance.
  *   \see LEDDARVU8_BASE_POINT_COUNT
  *   \return Base point sample
  */
  uint8_t getBasePointSample();
  /** Set Base point sample
  *   \see LEDDARVU8_BASE_POINT_COUNT
  *   \see getBasePointSample()
  *   \return nonzero if error
  */
  uint8_t setBasePointSample(uint8_t basePointSample);
  /** Get bit field of segment enabled
  *   \see LEDDARVU8_SEGMENT_ENABLE
  *   \return Bit field of segment enabled
  */
  uint32_t getSegmentEnabled();
  /** Set bit field of segment enabled
  *   \see LEDDARVU8_SEGMENT_ENABLE
  *   \return nonzero if error
  */
  uint8_t setSegmentEnabled(uint32_t segmentEnabled);
  /** Get acquisiton rate of the reference pulse
  *   \see LEDDARVU8_REF_PULSE_RATE
  *   \return Acquisiton rate of the reference pulse
  */
  uint32_t getReferencePulseRate();
  /** Set acquisiton rate of the reference pulse
  *   \see LEDDARVU8_REF_PULSE_RATE
  *   \return nonzero if error
  */
  uint8_t setReferencePulseRate(uint32_t referencePulseRate);
  /** Get yaw angle of the module
  *   \see LEDDARVU8_YAW
  *   \return Yaw angle of the module
  */
  float getYaw();
  /** Set yaw angle of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_YAW
  *   \return nonzero if error
  */
  uint8_t setYaw(float yaw);
  /** Get pitch angle of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_PITCH
  *   \return Pitch angle of the module
  */
  float getPitch();
  /** Set pitch angle of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_PITCH
  *   \return nonzero if error
  */
  uint8_t setPitch(float pitch);
  /** Get roll angle of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_ROLL
  *   \return Roll angle of the module
  */
  float getRoll();
  /** Set roll angle of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_ROLL
  *   \return nonzero if error
  */
  uint8_t setRoll(float roll);
  /** Get X-axis poistion of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_X
  *   \return X-axis poistion of the module
  */
  float getXPos();
  /** Set X-axis poistion of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_X
  *   \return nonzero if error
  */
  uint8_t setXPos(float xPos);
  /** Get Y-axis poistion of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_Y
  *   \return Y-axis poistion of the module
  */
  float getYPos();
  /** Set Y-axis poistion of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_Y
  *   \return nonzero if error
  */
  uint8_t setYPos(float yPos);
  /** Get Z-axis poistion of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_Z
  *   \return Z-axis poistion of the module
  */
  float getZPos();
  /** Set Z-axis poistion of the module
  *   - Has no purpose at the moment. To be implemented in the future by
  *     LeddarTech
  *   \see LEDDARVU8_Z
  *   \return nonzero if error
  */
  uint8_t setZPos(float zPos);
  /** Get Precision (smoothing)
  *   Stabilizes the module measurements. The behavior of the smoothing algorithm
  *   can be adjusted by a value from -16 to 16.
  *   \see LEDDARVU8_PRECISION
  *   \return Precision (smoothing)
  */
  int8_t getPrecision();
  /** Set Precision (smoothing)
  *   \see getPrecision()
  *   \return nonzero if error
  */
  uint8_t setPrecision(int8_t precision);
  /** Get Precision (smoothing) enabled.
  *   \see LEDDARVU8_PRECISION_ENABLE
  *   \return Precision (smoothing) enabled
  */
  uint8_t getPrecisionEnabled();
  /** Set Precision (smoothing) enabled.
  *   \see LEDDARVU8_PRECISION_ENABLE
  *   \return nonzero if error
  */
  uint8_t setPrecisionEnabled(uint8_t precisionEnabled);
  /** Get saturation compensation enabled.
  *   \see LEDDARVU8_SAT_COMP_ENABLE
  *   \return Saturation compensation enabled.
  */
  uint8_t getSaturationCompensationEnabled();
  /** Set saturation compensation enabled.
  *   \see LEDDARVU8_SAT_COMP_ENABLE
  *   \return nonzero if error
  */
  uint8_t setSaturationCompensationEnabled(uint8_t saturationCompensationEnabled);
  /** Get overshoot managment enabled.
  *   \see LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE
  *   \return Overshoot managment enabled.
  */
  uint8_t getOvershootManagementEnabled();
  /** Set overshoot managment enabled.
  *   \see LEDDARVU8_OVERSHOOT_MANAGEMENT_ENABLE
  *   \return nonzero if error
  */
  uint8_t setOvershootManagementEnabled(uint8_t overshootManagementEnabled);
  /** Get Sensitivity (detection threshold) setting expressed in a raw amplitude
  *   scale.
  *   \see LEDDARVU8_SENSITIVITY
  *   \return Overshoot managment enabled.
  */
  int32_t getSensitivity();
  /** Set Sensitivity (detection threshold) setting expressed in a raw amplitude
  *   scale.
  *   \see LEDDARVU8_SENSITIVITY
  *   \return nonzero if error
  */
  uint8_t setSensitivity(int32_t sensitivity);
  /** Get light source power (0 to 100)
  *   \see LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT
  *   \return Light source power (0 to 100)
  */
  uint8_t getLightSourcePower();
  /** Set light source power (0 to 100)
  *   \see LEDDARVU8_LED_USER_CURRENT_POWER_PERCENT
  *   \return nonzero if error
  */
  uint8_t setLightSourcePower(uint8_t lightSourcePower);
  /** Get auto light power source enabled
  *   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
  *   \return Auto light power source enabled
  */
  uint8_t getAutoLightSourcePowerEnabled();
  /** Set auto light power source enabled
  *   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
  *   \return nonzero if error
  */
  uint8_t setAutoLightSourcePowerEnabled(uint8_t autoLightSourcePowerEnabled);
  /** Get auto frame average: Changes the delay in number of measurements. This is
  *   the responsivity of the auto light source power according to the number
  *   of frames.
  *   \see LEDDARVU8_LED_USER_AUTO_FRAME_AVG
  *   \return Auto frame average
  */
  uint16_t getAutoFrameAverage();
  /** Set auto frame average
  *   \see LEDDARVU8_LED_USER_AUTO_FRAME_AVG
  *   \see getAutoFrameAverage()
  *   \return nonzero if error
  */
  uint8_t setAutoFrameAverage(uint16_t autoFrameAverage);
  /** Get Auto detections average: Number of detections for saturation acceptance
  *   (the number of detections that can be saturated to avoid decreasing the
  *   light source power when using the automatic mode). This is the responsivity
  *   of the auto light source power according ro the number of detections.
  *   \see LEDDARVU8_LED_USER_AUTO_ECHO_AVG
  *   \return Auto detections average
  */
  uint8_t getAutoDetectionsAverage();
  /** Set Auto detections average
  *   \see LEDDARVU8_LED_USER_AUTO_ECHO_AVG
  *   \see getAutoDetectionsAverage()
  *   \return nonzero if error
  */
  uint8_t setAutoDetectionsAverage(uint8_t autoDetectionsAverage);
  /** Get object demerging (DEM) enabled
  *   \see LEDDARVU8_DEM_ENABLE
  *   \return Object demerging (DEM) enabled
  */
  uint8_t getObjectDemergingEnabled();
  /** Set object demerging (DEM) enabled
  *   \see LEDDARVU8_DEM_ENABLE
  *   \return nonzero if error
  */
  uint8_t setObjectDemergingEnabled(uint8_t objectDemergingEnabled);
  /** Get Static noise removal enabled
  *   \see LEDDARVU8_ST_NOISE_REMOVAL
  *   \return Static noise removal enabled
  */
  uint8_t getStaticNoiseRemovalEnabled();
  /** Set Static noise removal enabled
  *   \see LEDDARVU8_ST_NOISE_REMOVAL
  *   \return nonzero if error
  */
  uint8_t setStaticNoiseRemovalEnabled(uint8_t staticNoiseRemovalEnabled);
  //---------------------------------------------------------------------------
  // Device information and constants bank

  /** Get the module number of the Leddar device.
  *   \param[out] moduleNumber
  *   \see LEDDARVU8_MODEL_PART_NUMBER
  *   \return nonzero for error
  */
  uint8_t getModulePartNumber(char* modulePartNumber);
  /** Get the software part number of the Leddar device.
  *   \param[out] softwarePartNumber
  *   \see LEDDARVU8_SOFTWARE_PART_NUMBER
  *   \return nonzero for error
  */
  uint8_t getSoftwarePartNumber(char* softwarePartNumber);
  /** Get the module serial number of the Leddar device.
  *   \param[out] moduleSerialNumber
  *   \see LEDDARVU8_MODULE_SERIAL_NUMBER
  *   \return nonzero for error
  */
  uint8_t getModuleSerialNumber(char* moduleSerialNumber);
  /** Get the manufacturer name of the Leddar device.
  *   \param[out] manufacturerName
  *   \see LEDDARVU8_MANUFACTURER_NAME
  *   \return nonzero for error
  */
  uint8_t getManufacturerName(char* manufacturerName);
  /** Get the group identification number as a ASCII of the Leddar device.
  *   \param[out] groupIdentificationNumber
  *   \see LEDDARVU8_GROUP_IDENTIFICATION_NUMBER
  *   \return nonzero for error
  */
  uint8_t getGroupIdentificationNumber(char* groupIdentificationNumber);
  /** Get the build date of the Leddar device.
  *   \param[out] buildDate
  *   \see LEDDARVU8_BUILD_DATE
  *   \return nonzero for error
  */
  uint8_t getBuildDate(char* buildDate);
  /** Get the Firmware version of the Leddar device.
  *   \param[out] firmwareVersion
  *   \see LEDDARVU8_FIRMWARE_VERSION
  *   \return nonzero for error
  */
  uint8_t getFirmwareVersion(char* firmwareVersion);
  /** Get the Bootloader version of the Leddar device.
  *   \param[out] bootloaderVersion
  *   \see LEDDARVU8_BOOTLOADER_VERSION
  *   \return nonzero for error
  */
  uint8_t getBootloaderVersion(char* bootloaderVersion);
  /** Get the ASIC version of the Leddar device.
  *   \param[out] asicVersion
  *   \see LEDDARVU8_ASIC_VERSION
  *   \return nonzero for error
  */
  uint8_t getAsicVersion(char* asicVersion);
  /** Get the FGPA version of the Leddar device.
  *   \param[out] fgpaVersion
  *   \see LEDDARVU8_FGPA_VERSION
  *   \return nonzero for error
  */
  uint8_t getFgpaVersion(char* fgpaVersion);
  /** Get the Module type of the Leddar device.
  *   \see LEDDARVU8_MODULE_TYPE
  *   \return Module type
  */
  uint16_t getModuleType();
  /** Get the Accumulation exponent min of the Leddar device.
  *   \see LEDDARVU8_ACCUMULATION_EXP_MIN
  *   \return Accumulation Exponent min
  */
  uint8_t getAccumulationExponentMin();
  /** Get the Accumulation exponent max of the Leddar device.
  *   \see LEDDARVU8_ACCUMULATION_EXP_MAX
  *   \return Accumulation Exponent max
  */
  uint8_t getAccumulationExponentMax();
  /** Get the Oversampling exponent min of the Leddar device.
  *   \see LEDDARVU8_OVERSAMPLING_EXP_MIN
  *   \return Oversampling Exponent min
  */
  uint8_t getOversamplingExponentMin();
  /** Get the Oversampling exponent max of the Leddar device.
  *   \see LEDDARVU8_OVERSAMPLING_EXP_MAX
  *   \return Oversampling Exponent max
  */
  uint8_t getOversamplingExponentMax();
  /** Get the Base point sample min of the Leddar device.
  *   \see LEDDARVU8_BASE_POINT_SAMPLE_MIN
  *   \return Base Point Sample min
  */
  uint8_t getBasePointSampleMin();
  /** Get the Base point sample max of the Leddar device.
  *   \see LEDDARVU8_BASE_POINT_SAMPLE_MAX
  *   \return Base Point Sample max
  */
  uint8_t getBasePointSampleMax();
  /** Get number of vertical segments of the Leddar device.
  *   \see LEDDARVU8_NUMBER_OF_VERTICAL_SEGMENTS
  *   \return Number of vertical segments
  */
  uint16_t getNumberOfVerticalSegments();
  /** Get number of horizontal segments of the Leddar device.
  *   \see LEDDARVU8_NUMBER_OF_HORIZONTAL_SEGMENTS
  *   \return Number of horizontal segments
  */
  uint16_t getNumberOfHorizontalSegments();
  /** Get number of reference segments of the Leddar device.
  *   \see LEDDARVU8_NUMBER_OF_REFERENCE_SEGMENTS
  *   \return Reference segments
  */
  uint16_t getNumberOfReferenceSegments();
  /** Get Base Point sample distance of the Leddar device.
  *   \see LEDDARVU8_BASE_POINT_SAMPLE_DISTANCE
  *   \return Base point sample distance
  */
  uint32_t getBasePointSampleDistance();
  /** Get Reference segment mask of the Leddar device.
  *   \see LEDDARVU8_REFERENCE_SEGMENT_MASK
  *   \return Reference segment mask
  */
  uint32_t getReferenceSegmentMask();
  /** Get max number of samples of the Leddar device.
  *   \see LEDDARVU8_NUMBER_OF_SAMPLE_MAX
  *   \return Number of sample max
  */
  uint16_t getNumberOfSamplesMax();
  /** Get clock frequency of the Leddar device.
  *   \see LEDDARVU8_CLOCK_FREQUENCY
  *   \return Clock frequency
  */
  uint32_t getClockFrequency();
  /** Get maximum number of detections per segment of the Leddar device.
  *   \see LEDDARVU8_MAX_NUMBER_OF_DETEC_PER_SEGMENT
  *   \return Maximum number of detections per segment
  */
  uint8_t getMaxNumberOfDetectionsPerSegment();
  /** Get distance scale of the Leddar device.
  *   \see LEDDARVU8_DISTANCE_SCALE
  *   \return Distance scale
  */
  uint32_t getDistanceScale();
  /** Get raw amplitude scale bit of the Leddar device.
  *   To get amplitude: (Raw amplitude) << (scale bit + 0x0D)
  *   \see LEDDARVU8_RAW_AMPLITUDE_SCALE_BIT
  *   \return Raw amplitude scale bit
  */
  uint8_t getRawAmplitudeScaleBit();
  /** Get raw amplitude scale of the Leddar device.
  *   To get amplitude: (Raw amplitude) / (raw amplitude scale + 8192)
  *   \see LEDDARVU8_RAW_AMPLITUDE_SCALE
  *   \return Raw amplitude scale
  */
  uint32_t getRawAmplitudeScale();
  /** Get precision min of the Leddar device.
  *   \see LEDDARVU8_PRECISION_MIN
  *   \return Precision min
  */
  int16_t getPrecisionMin();
  /** Get precision max of the Leddar device.
  *   \see LEDDARVU8_PRECISION_MAX
  *   \return Precision max
  */
  int16_t getPrecisionMax();
  /** Get sensitivity min of the Leddar device.
  *   \see LEDDARVU8_SENSITIVITY_MIN
  *   \return Sensitivity min
  */
  int32_t getSensitivityMin();
  /** Get sensitivity max of the Leddar device.
  *   \see LEDDARVU8_SENSITIVITY_MAX
  *   \return Sensitivity max
  */
  int32_t getSensitivityMax();
  /** Get current light source power count of the Leddar device.
  *   \see LEDDARVU8_CURRENT_LIGHT_SOURCE_PWR_COUNT
  *   \return Current light source power count
  */
  uint8_t getCurrentLightSourcePowerCount();
  /** Get auto frame average min of the Leddar device.
  *   \see LEDDARVU8_AUTO_FRAME_AVG_MIN
  *   \return Auto frame average min
  */
  uint16_t getAutoFrameAverageMin();
  /** Get auto frame average max of the Leddar device.
  *   \see LEDDARVU8_AUTO_FRAME_AVG_MAX
  *   \return Auto frame average max
  */
  uint16_t getAutoFrameAverageMax();
  /** Get auto light source power percent min of the Leddar device.
  *   \see LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MIN
  *   \return Auto light source power percent min
  */
  uint8_t getAutoLightSourcePowerPercentMin();
  /** Get auto light source power percent max of the Leddar device.
  *   \see LEDDARVU8_AUTO_LIGHT_SRC_PWR_PRCNT_MAX
  *   \return Auto light source power percent max
  */
  uint8_t getAutoLightSourcePowerPercentMax();
  /** Get auto detections average min of the Leddar device.
  *   \see LEDDARVU8_AUTO_DETECTION_AVG_MIN
  *   \return Auto detections average min
  */
  uint8_t getAutoDetectionsAverageMin();
  /** Get auto detections average max of the Leddar device.
  *   \see LEDDARVU8_AUTO_DETECTION_AVG_MAX
  *   \return Auto detections average max
  */
  uint8_t getAutoDetectionsAverageMax();
  /** Get static noise calibration source of the Leddar device.
  *   0 = By end-user
  *   1 = By factory
  *   \see LEDDARVU8_STATIC_NOISE_CALIBRATION_SOURCE
  *   \return Static noise calibration source
  */
  uint8_t getStaticNoiseCalibrationSource();
  /** Get CPU load scale of the Leddar device.
  *   \see LEDDARVU8_CPU_LOAD_SCALE
  *   \return CPU load scale
  */
  uint32_t getCPUloadScale();
  /** Get temperature scale of the Leddar device.
  *   \see LEDDARVU8_TEMPERATURE_SCALE
  *   \return Temperature scale
  */
  uint32_t getTemperatureScale();
  //-----------------------------------------------------------------------------
  // Detection list bank: R only

  /** Get Timestamp in ms since power up
  *   \see LEDDARVU8_TIMESTAMP
  *   \return Timestamp
  */
  uint32_t getTimestamp();
  /** Get number of detections (N)
  *   \see LEDDARVU8_NUMBER_OF_DETECTIONS
  *   \return Number of detections (N)
  */
  uint16_t getNumberOfDetections();
  /** Get current percentage of light source power
  *   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
  *   \return Current percentage of light source power
  */
  uint16_t getCurrentPercentageOfLightSourcePower();
  /** Get acquisiton options
  *   \see LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR
  *   \return Acquisiton options
  */
  uint32_t getAcquisitionOptions();

  //-----------------------------------------------------------------------------
  // Transaction configuration bank - R/W

  /** Get secure transaction enabled flags: <br />
  *   1 = Enables the CRC calculation and validation on any transaction. This flag is enabled by default.<br />
  *   0 = No CRC validation. The CRC field is still required in SPI protocol but can be set to any value.
  *   \see LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS
  *   \return Secure transaction enabled flags
  */
  uint8_t getSecureTransactionEnabledFlags();
  /** Set secure transaction enabled flags:
  *   \see LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS
  *   \see getSecureTransactionEnabledFlags()
  *   \return nonzero for error
  */
  uint8_t setSecureTransactionEnabledFlags(uint8_t secureTransactionEnabledFlags);
  /** Get transaction mode: <br />
  *   0 = Free run. The READY pin is asserted on each ready detection frame. The host must be able to read data on time. <br />
  *   1 = Blocking read. On the READY pin assertion, host must read all data from traces or detections bank (data transaction control source configuration) to continue acquisition. <br />
  *   2 = Partial blocking read. On the READY pin assertion, host can read all data from traces of the detection bank and the acquisition is still running. Possible loss of detection frames if the host reading data is very long.
  *   \see LEDDARVU8_TRANSACTION_MODES
  *   \return Transaction mode
  */
  uint8_t getTransactionMode();
  /** Set transaction mode
  *   \see LEDDARVU8_TRANSACTION_MODE
  *   \see getTransactionMode()
  *   \return nonzero for error
  */
  uint8_t setTransactionMode(uint8_t transactionMode);
  /** Get CRC of the last transaction
  *   \see LEDDARVU8_CRC_OF_THE_LAST_TRANSACTION
  *   \return CRC of last transaction
  */
  uint16_t getCRClastTransaction();
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
  uint16_t getBitFieldInformationOfLastTransaction();
  /** Get data transaction control source: <br />
  *    0 = On trace  <br />
  *    1 = On detections  <br />
  *    This register determines which data type will control the READY pin and manage the transaction mode.
  *   \see LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE
  *   \return Data transaction control source
  */
  uint8_t getDataTransactionControlSource();
  /** Set data transaction control source <br />
  *   \see LEDDARVU8_DATA_TRANSACTION_CONTROL_SOURCE
  *   \see getDataTransactionControlSource()
  *   \return nonzero if error
  */
  uint8_t setDataTransactionControlSource(uint8_t dataTransactionControlSource);

  //--------------------------------------------------------------------------
  // SPI
  /** Make SPI ready for transfer
  *
  */
  void spiStart();
  /** Finish the SPI transfer
  *
  */
  void spiStop();
private:
  // helper function get8uint
  uint8_t get8uint(uint32_t address);
  // helper function get8uint
  uint8_t get8uint(uint8_t opcode, uint32_t address);
  // helper function get16uint
  uint16_t get16uint(uint32_t address);
  // helper function get32uint
  uint32_t get32uint(uint32_t address);

  // helper function get8int
  int8_t get8int(uint32_t address);
  // helper function get16int
  int16_t get16int(uint32_t address);
  // helper function get32uint
  int32_t get32int(uint32_t address);

  // helper function get32char
  uint8_t get32char(uint32_t address, char* bufChar);
  // helper function getFloat
  float getFloat(uint32_t address);

  // helper function set8uint
  uint8_t set8uint(uint32_t address, uint8_t value);
  // helper function set16uint
  uint8_t set16uint(uint32_t address, uint16_t value);
  // helper function set32uint
  uint8_t set32uint(uint32_t address, uint32_t value);

  // helper function set8int
  uint8_t set8int(uint32_t address, int8_t value);
  // helper function set32int
  uint8_t set32int(uint32_t address, int32_t value);

  // helper function setFloat
  uint8_t setFloat(uint32_t address, float value);

  // define a union for conversion between data types
  // http://www.cplusplus.com/doc/tutorial/other_data_types/
  union leddarData_t {
    uint8_t b[4];
    float f;
    uint32_t i;
  } leddarData;

  // SPI functions and variables
  SPISettings m_spiSettings;
  uint8_t 	m_csPin;
  bool    	m_spiActive;

  //--------------------------------------------------------------------------
  // These functions are taken from the Arduino SD card library
  /* Arduino SdCard Library
  * Copyright (C) 2016 by William Greiman
  * Modified by PSS-GEO, 2018
  */
  /** Initialize the SPI bus.
  *
  * \param[in] csPin - LeddarVu8 chip select pin.
  */
  void spiBegin(uint8_t csPin);
  /** Save SPISettings.
  *
  * \param[in] spiSettings SPI speed, mode, and byte order.
  */
  void spiSetSpiSettings(SPISettings settings);
  /** Activate SPI hardware.
  *
  */
  void spiActivate();
  /** Deactivate SPI hardware.
  *
  */
  void spiDeactivate();
  /** Receive a byte.
  *
  * \return The byte.
  */
  uint8_t spiReceive();
  /** Receive multiple bytes.
  *
  * \param[out] buf Buffer to receive the data.
  * \param[in] n Number of bytes to receive.
  *
  * \return Zero for no error or nonzero error code.
  */
  uint8_t spiReceive(uint8_t* buf, size_t n);
  /** Send a byte.
  *
  * \param[in] data Byte to send
  */
  void spiSend(uint8_t data);
  /** Send multiple bytes.
  *
  * \param[in] buf Buffer for data to be sent.
  * \param[in] n Number of bytes to send.
  */
  void spiSend(const uint8_t* buf, size_t n);
  /** Set CS low.
  *
  */
  void spiSelect();
  /** Set CS high.
  *
  */
  void spiUnselect();
};
#endif
