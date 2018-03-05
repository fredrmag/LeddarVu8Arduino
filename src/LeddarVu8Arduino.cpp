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
bool LeddarVu8Arduino::begin(uint8_t csPin){
  // Initialize spi
  spiBegin(csPin);
  spiSetSpiSettings(SPISettings(15000000, MSBFIRST, SPI_MODE0)); // 15 MHz
  spiStart();
  // TODO: Check the device name to see if we are connected
}
//-----------------------------------------------------------------------------
void LeddarVu8Arduino::spiStart(){
  if (!m_spiActive) {
    spiActivate();
    spiSelect();
    m_spiActive = true;
  }
}
//-----------------------------------------------------------------------------
