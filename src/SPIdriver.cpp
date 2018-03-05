/* Arduino SdCard Library
 * Copyright (C) 2016 by William Greiman
 * Modified by Fredrik Magnussen, March 2, 2018
 *
 * This file is part of the Arduino SdSpiCard Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino SdSpiCard Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#include "SPIdriver.h"
#include <SPI.h>

/** Activate SPI hardware. */
void SPIDriver::activate() {
  SPI.beginTransaction(m_spiSettings);
}
/** Deactivate SPI hardware. */
void SPIDriver::deactivate() {
  SPI.endTransaction();
}
/** Initialize the SPI bus.
 *
 * \param[in] csPin SD card chip select pin.
 */
void SPIDriver::begin(uint8_t csPin) {
  m_csPin = csPin;
  digitalWrite(csPin, HIGH);
  pinMode(csPin, OUTPUT);
  SPI.begin();
}
/** Receive a byte.
 *
 * \return The byte.
 */
uint8_t SPIDriver::receive() {
  return SPI.transfer(0XFF);
}
/** Receive multiple bytes.
*
* \param[out] buf Buffer to receive the data.
* \param[in] n Number of bytes to receive.
*
* \return Zero for no error or nonzero error code.
*/
uint8_t SPIDriver::receive(uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    buf[i] = SPI.transfer(0XFF);
  }
  return 0;
}
/** Send a byte.
 *
 * \param[in] data Byte to send
 */
void SPIDriver::send(uint8_t data) {
  SPI.transfer(data);
}
/** Send multiple bytes.
 *
 * \param[in] buf Buffer for data to be sent.
 * \param[in] n Number of bytes to send.
 */
void SPIDriver::send(const uint8_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    SPI.transfer(buf[i]);
  }
}
/** Set CS low. */
void SPIDriver::select() {
  digitalWrite(m_csPin, LOW);
}
/** Save SPISettings.
 *
 * \param[in] spiSettings SPI speed, mode, and byte order.
 */
void SPIDriver::setSpiSettings(SPISettings spiSettings) {
  m_spiSettings = spiSettings;
}
/** Set CS high. */
void SPIDriver::unselect() {
  digitalWrite(m_csPin, HIGH);
}
