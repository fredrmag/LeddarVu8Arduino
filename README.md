# LeddarVu8Arduino class

## Introduction
Library for the [Leddar Vu8 LiDAR sensor](https://leddartech.com/modules/leddarvu/) (using SPI) made by LeddarTech. User guide can be downloaded [here](https://support.leddartech.com/file.php/181AGYWYHCZQY18023063C4C6/LeddarVu-and-Configurator-User-Guide.pdf).

## Examples
* LeddarVu8ArduinoEchoes - Example of the readEchoes() function.
* LeddarVu8ArdunioDeviceInformationAndConstants - Example of reading the device info and constants.
* LeddarVu8ArdunioConfigurationData - Example of reading the device configuration data.
* LeddarVu8ArduinoChangeConfig - Example of changing the configuration data.

## TODO
- [x] Make mock library with very basic functionality.
- [x] Test on real sensor.
- [x] Cyclic redundancy verification.
- [x] Add support for SPI write functions.
- [x] Add support for other read functions when needed.
- [ ] Add support in the examples of the use of the ready pin.
- [ ] Investigate if the error handling can be done better.

## Notes
### Future development to be done by LeddarTech
* The X, Y, Z, Yaw, Pitch and Roll of the module in the configuration data bank are for no purpose at the moment.
* Pin 6 & 8, external synchronisation (input) and receiver sampling output (output) respectively, are to be implented in the future by LeddarTech.

## Credit - 3rd party software used
* The SPI functions are modified from the Arduino SD-card library [SdFat](https://github.com/greiman/SdFat) written by William Greiman, and is under the [GNU General Public License](http://www.gnu.org/licenses/) as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
* The CRC16 checking (CRC16.h) is taken from the [ArduinoLeddar](http://share.leddartech.com/ArduinoLeddar.zip) library written by LeddarTech, and is under the [MIT licence](https://opensource.org/licenses/mit-license.html).
