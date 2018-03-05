/* Arduino SdCard Library
* Copyright (C) 2016 by William Greiman
* Modified by Fredrik Magnusen, 2017
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
#ifndef SPIdriver_h
#define SPIdriver_h

#include <SPI.h>
/**
* \class SPIdriver
* \brief SPIdriver class. Modified from Arduino SdCard Library.
*/
class SPIdriver {
  public:
    /** Set SPI options for access to SD/SDHC cards.
    *
    */
    void activate();
    /** Initialize the SPI bus.
    *
    * \param[in] chipSelectPin SD card chip select pin.
    */
    void begin(uint8_t chipSelectPin);
    /**
    * End SPI transaction.
    */
    void deactivate();
    /** Receive a byte.
    *
    * \return The byte.
    */
    uint8_t receive();
    /** Receive multiple bytes.
    *
    * \param[out] buf Buffer to receive the data.
    * \param[in] n Number of bytes to receive.
    *
    * \return Zero for no error or nonzero error code.
    */
    uint8_t receive(uint8_t* buf, size_t n);
    /** Send a byte.
    *
    * \param[in] data Byte to send
    */
    void send(uint8_t data);
    /** Send multiple bytes.
    *
    * \param[in] buf Buffer for data to be sent.
    * \param[in] n Number of bytes to send.
    */
    void send(const uint8_t* buf, size_t n);
    /** Set CS low.
    *
    */
    void select();
    /** Save SPI settings.
    * \param[in] spiSettings SPI speed, mode, and bit order.
    */
    void setSpiSettings(SPISettings spiSettings);
    /** Set CS high.
    *
    */
    void unselect();
  private:
   SPISettings m_spiSettings;
   uint8_t m_csPin;
};
#endif  // SPIdriver_h
