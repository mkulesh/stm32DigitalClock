# <img src="https://github.com/mkulesh/stm32DigitalClock/blob/master/images/stm32_image.png" align="center" height="48" width="48"> "A digital clock based on STM32F405 MCU"

This repository provides hardware layout and firmware for a radio-controlled digital clock based on [STM32F405RG](http://www.st.com/content/st_com/en/products/microcontrollers/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f4-series/stm32f405-415/stm32f405rg.html) MCU and [DCF77](https://de.wikipedia.org/wiki/DCF77) receiver:
![in operation](https://github.com/mkulesh/stm32DigitalClock/blob/master/images/in_operation.jpg)

## Hardware design
This clock consist of two boards:
- the first display board contains four 0.8 inch. single digit numerical displays, one 2x16 chars LCD-display, and four buttons that allow to adjust current time, three alarms and some settings:
![display board](https://github.com/mkulesh/stm32DigitalClock/blob/master/images/display_board1.jpg)

- the second board contains STM32F405 MCU with a lot of periphery: DC connector, voltage regulators, HSE and LSE crystals, JTAG programming connector, light sensor, temperature sensor, DCF77 receiver with ferrite core, SD card connector, DAC and audio amplifier for primary WAV-based alarm, outputs for audio speakers, one piezo buzzer for the secondary alarm, UART-USB bridge and USB connector (used for debugging purposes):
![mcu board](https://github.com/mkulesh/stm32DigitalClock/blob/master/images/mcu_board2.jpg)

PCB are developed in Eagle CAD (see directory pcb):
![mcu board layout](https://github.com/mkulesh/stm32DigitalClock/blob/master/images/mcu_board0.png)
![display board layout](https://github.com/mkulesh/stm32DigitalClock/blob/master/images/display_board0.png)

## Firmware
The firmware (see directory src) is written in C++ [System Workbench for STM32](http://www.st.com/en/development-tools/sw4stm32.html). It is based on HAL library, FatFS and the second object-oriented abstraction layer called *StmPlusPlus* that implements high-level access for all used hardware components. *StmPlusPlus* also contains WAV-streamer (16 bit stereo, 44kHz) and [DCF77](https://de.wikipedia.org/wiki/DCF77) receiver with a special windowed filter used to improce signal quality.

## There are some known problems in this project
- DCF77 receiver heeds some time a pair of hours to capture the time stamp. 
- Wrong position of the DC and USB connector: access to the connector is blocked by audio speakers
- Level of the input signal for the audio amplifier is too high. 

## License

This software is published under the *GNU General Public License, Version 3*

Copyright (C) 2016-2017 Mikhail Kulesh

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along with this program.

If not, see [www.gnu.org/licenses](http://www.gnu.org/licenses).
