# Servo 2
## 3-phase AC servomotor driver

This is a project of a servo inverter to drive 3-phase AC motor with/out encoder feedback

It's in alpha development so there are many bugs and things included in software but not really working/tested.

![CPU board](https://github.com/wiciu15/servo2/blob/main/PCB/servo_cpu/servo_cpu.png?raw=true)

This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
of the GNU General Public License version 3 as published by the Free Software Foundation.
This project is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
or indirectly by this software/hardware. The board is mains-powered. High voltage can kill you. Have caution when using high voltage devices

## Features

- Manual output voltage and frequency control
- U/f scalar motor control
- Open loop current control
- Field-oriented vector control (FOC) with encoder feedback for PMSM motors
- Encoders support:
  -  ABZ encoder with differential driver output, needs one manual rotation to index with a rotor properly
  -  Mitsubishi J2/J2s series, baud rate 2 500 000 bits/s
  -  Tamagawa T-Format encoders - tested with ABB BSM-series motor, baud rate 2 500 000 bits/s
  -  Panasonic minas series encoder - basic absolute position readout, support in early development
  -  DELTA NH4 series T+ T- encoder - very early development, not tested yet
- Braking chopper control
- under/overvoltage, overcurrent, short-circuit and other errors detection and handling
- Communication with PC using MODBUS over USB-CDC
- Parameter set, with non-volatile storage in external EEPROM
- OLED screen with basic user interface

## To-do in the future

- ramp generators and limiters for control loops
- position loop
- sensorless rotor angle aquisition
- fieldbus control (MODBUS RS485, CANopen)
- analog and digital I/O
- parametrization, actions and control using OLED screen and buttons

## Used third-party software
- I2C EEPROM driver by ControllersTech
- OLED driver https://github.com/afiskon/stm32-ssd1306
- stModbus https://github.com/wiciu15/stModbus/
- FreeRTOS
- STM32 HAL libraries



