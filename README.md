# MavESP8266

## Current Binary

Download the current version (MAVLink V2) from here: [ArduPilot fork](https://firmware.ardupilot.org/Tools/MAVESP8266/latest/)

Download the legacy version (MAVLink V1) from here: [Firmware version 1.1.1](http://www.grubba.com/mavesp8266/firmware-1.1.1.bin)

## ESP8266 WiFi Access Point and MavLink Bridge

[![Join the chat at https://gitter.im/dogmaphobic/mavesp8266](https://badges.gitter.im/dogmaphobic/mavesp8266.svg)](https://gitter.im/dogmaphobic/mavesp8266?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

This was developed using a [NodeMCU v2 Dev Kit](http://www.seeedstudio.com/depot/NodeMCU-v2-Lua-based-ESP8266-development-kit-p-2415.html) as it conveniently provides a secondary UART for debugging. It has been tested with the ESP-01 shipped with the [PixRacer](https://pixhawk.org/modules/pixracer) and it is stable at 921600 baud.

The build enviroment is based on [PlatformIO](http://platformio.org). Follow the instructions found here: http://platformio.org/#!/get-started (only tested on Mac OS) for installing it but skip the ```platform init``` step as this has already been done, modified and it is included in this repository. In summary:

```
brew install platformio
git clone --recursive https://github.com/markingle/mavesp8266.git
cd mavesp8266
platformio run
```

When you run ```platformio run``` for the first time, it will download the toolchains and all necessary libraries automatically.

Note: if build fails with `AttributeError: module 'enum' has no attribute 'IntFlag'`, try `pip3 uninstall -y enum34`

### Useful commands: ESP12e

* ```platformio run``` - process/build all targets
* ```platformio run -e esp12e``` - process/build just the ESP12e target (the NodeMcu v2, Adafruit HUZZAH, etc.)
* ```platformio run -e esp12e -t upload``` - build and upload firmware to embedded board
* ```platformio run -t clean``` - clean project (remove compiled files)

### Useful commands: ESP-01
* ```platformio run``` - process/build all targets
* ```platformio run -e esp01_1m``` - process/build just the ESP01 target with 1MB [Amazon ESP-01](https://www.amazon.com/gp/product/B01EA3UJJ4)
* ```platformio run -e esp01_1m -t upload``` - build and upload firmware to embedded board
* ```platformio run -t clean``` - clean project (remove compiled files)

### Useful commands: ESP32-WROOM (ESP32-WROOM-32UE)

* ```platformio run -e espwroom32 -t upload```
* ```platformio run -e espwroom32``` - process/build just the ESP32 target

### Useful commands: ESP32-S3 or ESP32-S3
* ```platformio run -e esp32-s3-devkitc-1 -t upload```
* ```platformio run -e esp32-s3-devkitc-1 ``` - process/build just the ESP32 target

### Useful commands: ESP32-S3-MINI or ESP32-S3-MINI [Mouser](https://www.mouser.com/ProductDetail/356-ESP32S3DEVKTM1N8)
* ```platformio run -e esp32-s3-devkitm-1 -t upload```
* ```platformio run -e esp32-s3-devkitm-1 ``` - process/build just the ESP32 target

### Useful commands: ESP32-C3-MINI
* ```platformio run -e esp32-c3-devkitm-1 -t upload```
* ```platformio run -e esp32-c3-devkitm-1 ``` - process/build just the ESP32 target

 resulting image(s) can be found in the directory ```.pioenvs``` created during the build process.

### Platform [Details](https://github.com/RealFlightSystems/mavesp8266/wiki/Espressif-WLAN-Mavlink-Router-and-Bridge-Development)

### MavLink Submodule - >>>> DON'T FORGET TO DO THIS!!!!!!!  <<<<<

The ```git clone --recursive``` above not only cloned the MavESP8266 repository but it also installed the dependent [MavLink](https://github.com/mavlink/c_library) sub-module. To upated the module (when needed), use the command:

```git submodule update --init```

### Wiring it up

User level (as well as wiring) instructions can be found here: https://pixhawk.org/peripherals/8266

* Resetting to Defaults: In case you change the parameters and get locked out of the module, all the parameters can be reset by bringing the GPIO02 pin low (Connect GPIO02 pin to GND pin). 

Get the ESP-01 adapter board here on [Amazon](https://www.amazon.com/gp/product/B07Q17XJ36/); commonly called an "ESP01 programmer", this one has a little rocker switch on the side (UART side for serial TTL debugging by AT commands, PROG for firmware programming) and a yellow pin header which allows you plugin the ESP01 module without any wires. It defaults to 115200 and enumerates under Ubuntu as a ch341-uart converter. 

### MavLink Protocol

The MavESP8266 handles its own set of parameters and commands. Look at the [PARAMETERS](PARAMETERS.md) page for more information.

### HTTP Protocol

There are some preliminary URLs that can be used for checking the WiFi Bridge status as well as updating firmware and changing parameters. [You can find it here.](HTTP.md)
