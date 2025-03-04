# Time machine clock

## Description

This is a small project to have a time machine clock like back to the future.

## How to

Build & flash esp32
* Set ESP32 in programming mode (GPIO0 to GND)
* Plug in FDTI
* `cd tools`
* `./docker_build.sh`
* `./docker_run.sh`
* `idf.py build`
* `idf.py -p /dev/ttyUSB0 flash`
* Power off & put ESP32 in boot mode (GPIO0 open)

## Pictures
