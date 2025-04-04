# Servo 2
## 3-phase AC servomotor driver

This is a project of a servo inverter to drive 3-phase AC motor with/out encoder feedback

It's in alpha development so there are many bugs and things included in software but not really working/tested.

DEMO video: https://www.youtube.com/watch?v=Hu02lt5aCbM

CNC DEMO: https://www.youtube.com/watch?v=LXm4no8DIPE

![CPU board](https://github.com/wiciu15/servo2/blob/main/PCB/assembly.JPG?raw=true)

This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
of the GNU General Public License version 3 as published by the Free Software Foundation.
This project is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
or indirectly by this software/hardware. The board is mains-powered. High voltage can kill you. Have caution when using high voltage devices

## Features

- Manual voltage and frequency control
- U/f scalar motor control
- Open loop current control for test purposes
- Field-oriented vector control (FOC) for PMSM/BLDC motors with encoder feedback
- Encoders support:
  -  ABZ encoder with differential driver output, needs one manual rotation to index with a rotor properly
  -  Mitsubishi J2/J2s/J4/JE series, baud rate 2 500 000 bits/s, implementation is working but need some polish
  -  Tamagawa T-Format encoders - tested with ABB BSM-series motor, baud rate 2 500 000 bits/s, most robust implementation and testing at this moment
  -  Panasonic minas series encoder - support in early development, not much tested and buggy
  -  DELTA NH4 series T+ T- encoder - very early development
- Braking chopper control
- under/overvoltage, overcurrent, short-circuit and other errors detection and handling
- Communication with PC using MODBUS over USB-CDC
- simple CANopen node for fieldbus control
- Parameter set, with non-volatile storage in external EEPROM
- control and monitoring of drive using OLED screen and buttons
- ramp generators and limiters for current,speed and position control loops

## To-do in the future

- I2t thermal monitoring of motor
- sensorless rotor angle aquisition
- analog and digital I/O
- step/dir input
- current and speed loop gains auto-tuning
- motor parameters automatic measurement

## Used third-party software
- I2C EEPROM driver by ControllersTech
- OLED driver https://github.com/afiskon/stm32-ssd1306
- stModbus https://github.com/wiciu15/stModbus/
- CANopenNode https://github.com/CANopenNode/CANopenNode
- embedded printf implementation https://github.com/mpaland/printf
- FreeRTOS

## Hardware modification
MOdifications of rev.A of CPU board
- data/command line to OLED screen(PA15)
- bridge between tim1break input (PE15) and overcurrent interrupt(PA1)(break interrupt was not working i dont know why just bridged it to EXTI on other pin).
- OLED CLK is moved to PA5, PB3 is used as DEBUG_SWO.
- added filter to GND on FAULT line from power board to enchance noise rejection from EMI(i was getting random short-circuit errors).


