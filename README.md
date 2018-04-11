# LeddarVu8Arduino class

## Introduction
Library for the [Leddar Vu8 LiDAR sensor](https://leddartech.com/modules/leddarvu/) (using SPI) made by LeddarTech. User guide can be downloaded [here](https://support.leddartech.com/file.php/181AGYWYHCZQY18023063C4C6/LeddarVu-and-Configurator-User-Guide.pdf).

Written by PSS-GEO AS.

## Examples
* LeddarVu8ArduinoEchoes - Example of readEchoes() function.

## TODO
- [x] Make mock library with very basic functionality.
- [x] Test on real sensor.
- [x] Cyclic redundancy verification.
- [ ] Add support for SPI write functions.
- [ ] Add support for other read functions when needed.

## Credit - 3rd party software used
* The SPI functions are modified from the Arduino SD-card library [SdFat](https://github.com/greiman/SdFat) written by William Greiman, and is under the [GNU General Public License](http://www.gnu.org/licenses/) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

* The CRC16 checking (CRC16.h) is taken from the [ArduinoLeddar](http://share.leddartech.com/ArduinoLeddar.zip) library written by LeddarTech, and is under the [MIT licence](https://opensource.org/licenses/mit-license.html).
