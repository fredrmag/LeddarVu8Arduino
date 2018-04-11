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
#define LEDDARVU8_FPGA_VERSION                       0x0120 | REGMAP_DEV_INFO // FPGA version as an ASCII string
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
#define LEDDARVU8_NUMBER_OF_DTECTIONS                0x04 | REGMAP_DETECTIONS // Number of detections (N)
#define LEDDARVU8_PERCENT_OF_LIGHT_SRC_PWR           0x06 | REGMAP_DETECTIONS // Current percentage of light source power
#define LEDDARVU8_ACQUISITION_OPTIONS                0x08 | REGMAP_DETECTIONS // Acquisition options
#define LEDDARVU8_START_OF_DETECTION_LIST_ARRAYS     0x0C | REGMAP_DETECTIONS // Start of detection list array length: N*12 bytes

// REGMAP_TRN_CGF - R/W - 0xFFFB00
#define LEDDARVU8_SECURE_TRANSACTION_ENABLED_FLAGS   0x00 | REGMAP_TRN_CGF // Secure-transaction enabled flags:
//1 = Enables the CRC calculation and validation on any transaction. This flag is enabled by default.
//0 = No CRC validation. The CRC field is still required in SPI protocol but can be set to any value.
#define LEDDARVU8_TRANSACTION_MODED                  0x01 | REGMAP_TRN_CGF // Transaction modes:
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


/**
* \class LeddarVu8Arduino
* \brief LeddarVu8Arduino class
*/
class LeddarVu8Arduino {
public:
  /** Initialize the LeddarVu8 sensor
  *  Modified from SdSpiCard from Arduino SdCard Library
  * \param[in] csPin LeddarTechVu8 chip select pin.
  */
  uint8_t begin(uint8_t csPin);
  /** Extract byte from a 32-bit number, given number and byte-place
  * \param[in] number - number to extract byte from
  * \param[in] place - byte place from right, starts at 0
  * \return byte - corresponding byte for given number and byte place
  */
  byte extractByte(uint32_t number, int place);
  /** Send command to leddar sensor through SPI
  * \param[in] opcode
  * \param[in] address
  * \param[in] dataSize
  *
  */
  uint8_t * leddarCommand(uint8_t opcode, uint32_t address, size_t dataSize);
  /** Read Raw Echoes from the LeddarTech Vu8 sensor
  * \param[out] distances - raw distances
  * \param[out] amplitudes - raw amplitudes
  *
  */
  uint8_t readRawEchoes(uint32_t* distances, uint32_t* amplitudes);
  /** Read Echoes from the LeddarTech Vu8 sensor
  *  \param[out] distances - scaled distance - in meters
  *  \param[out] amplitudes - scaled amplitudes - in ??
  *  \return 0 if no error, or a non-zero error code
  *
  */
  uint8_t readEchoes(float* distances, float* amplitudes);
  /** Construct the full spi package and check the CRC16
  * \param[in] bufSend - header sent to the LeddarVu8
  * \param[in] bufReceive - data received from the LedderVu8, including the CRC16
  * \param[in] n - size of data package received
  * \param[out] boolean - true if CRC check is OK, false if fails
  *
  */
  bool checkCRC(uint8_t* bufSend, uint8_t* bufReceive,size_t size);
  /** Combines the bytes in a buffer to a 32-bit number
  *  \param[in] buf - buffer with bytes
  *  \param[in] n - size of buffer
  *  \return combined bytes in uint32_t format
  */
  uint32_t combineBytes(uint8_t* buf,size_t n);

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
