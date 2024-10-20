#!/bin/bash

# Install packages
sudo pip install asyncio adafruit-circuitpython-bmp3xx adafruit-circuitpython-bno055 board pigpio pytz pyserial websockets adafruit-circuitpython-ds3231 adafruit-circuitpython-tmp117 Flask

# Uninstall packages
sudo pip uninstall -y adafruit-circuitpython-busdevice adafruit-circuitpython-requests adafruit-circuitpython-typing adafruit-blinka numpy pyftdi pyserial pyusb rpi-ws281x sysv-ipc typing-extensions

# Reinstall packages
sudo pip install adafruit-circuitpython-busdevice adafruit-circuitpython-requests adafruit-circuitpython-typing adafruit-blinka numpy pyftdi pyserial pyusb rpi-ws281x sysv-ipc typing-extensions
