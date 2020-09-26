# STAG_GUI

## Introduction

Flashing this code on the STM32F769I-DISC1 board enables it to collect tactile data and send the data to a connected PC.
Using the Python GUI *STAG_GUI.py* from the *utils* folder visualizes the tactile frame.

## System requirements

If using the Makefile to compile the project, GNU Embedded Toolchain for Arm is required.
If using the Makefile to flash the code on the MCU (in terminal when inside the project folder: make flash), openocd is required.


## How to use

Use the buttons in the Python GUI to *Run* and *Pause* the application.

## Additional information

The correct drivers for the application need to be added in a directory called /Drivers. The project can also be created with the .ioc file in CubeMX, which will automatically add the neccessary drivers.


## Contact

Please email any questions or comments to [geigerf@student.ethz.ch](geigerf@student.ethz.ch).
