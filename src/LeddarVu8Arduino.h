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
#include "SPIDriver.h"

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
    bool begin(uint8_t csPin);
    void spiStart();

  private:
    //---------------------------------------------------------------------------
     // functions defined in SPIDriver.h
     void spiBegin(uint8_t csPin) {
       m_spiDriver->begin(csPin);
     }
     void spiSetSpiSettings(SPISettings settings) {
       m_spiDriver->setSpiSettings(settings);
     }
     void spiActivate() {
       m_spiDriver->activate();
     }
     void spiDeactivate() {
       m_spiDriver->deactivate();
     }
     uint8_t spiReceive() {
       return m_spiDriver->receive();
     }
     uint8_t spiReceive(uint8_t* buf, size_t n) {
       return  m_spiDriver->receive(buf, n);
     }
     void spiSend(uint8_t data) {
        m_spiDriver->send(data);
     }
     void spiSend(const uint8_t* buf, size_t n) {
       m_spiDriver->send(buf, n);
     }
     void spiSelect() {
       m_spiDriver->select();
     }
     void spiUnselect() {
       m_spiDriver->unselect();
     }
    SPIDriver *m_spiDriver;
    bool    m_spiActive;
};


#endif
