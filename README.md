# Embedded Spatial Mapping System

An embedded spatial mapping system built on the MSP432E401Y that uses a VL53L1X Time-of-Flight sensor and a 28BYJ-48 stepper motor to capture 360° distance measurements and reconstruct a 3D wire-frame view of an indoor environment.

## Overview

This project was developed for COMPENG 2DX3 (Microprocessor Systems). The system rotates a Time-of-Flight sensor through a full sweep, collects distance samples at fixed angular intervals, and transmits the data to a PC over UART for visualization in MATLAB.

The final implementation used a polling-based design for measurement and control. A single push button starts and stops scanning, status LEDs provide real-time feedback, and the scanned data is converted from polar coordinates into a 3D reconstruction.

## Features

* 360° planar distance scanning using the VL53L1X Time-of-Flight sensor
* Stepper motor control using a full-step sequence through a ULN2003 driver
* Polling-based acquisition and control in embedded C
* UART transmission to PC at 115200 baud
* MATLAB-based 3D wire-frame visualization
* Status LEDs for scan activity, measurement events, and UART transmission
* Mid-scan abort support with motor de-energization

## Hardware

* MSP432E401Y LaunchPad
* VL53L1X Time-of-Flight sensor
* 28BYJ-48 stepper motor
* ULN2003 stepper motor driver
* Push button input
* Jumper wires and breadboard

## Software / Tools

* Embedded C
* Keil µVision 5
* MATLAB
* I²C communication
* UART serial communication

## How It Works

1. The user presses the on-board button to begin scanning.
2. The microcontroller starts a scan sequence and turns on the scan status LED.
3. The VL53L1X sensor is polled for a fresh distance measurement.
4. After each sample, the firmware stores distance, angle, and displacement data.
5. The stepper motor advances to the next angular position.
6. Once a full sweep is complete, the data is sent to the PC over UART.
7. MATLAB reads the serial stream, converts the polar data to Cartesian coordinates, applies filtering, and plots a 3D wire-frame reconstruction.

## System Details

* 32 samples per 360° sweep
* 11.25° angular resolution
* Stepper motor driven through GPIO outputs PH0–PH3
* I²C communication with the VL53L1X on PB2/PB3
* UART communication to the PC at 115200 baud
* Polling used for button handling and sensor data acquisition

## Data Format

UART data is transmitted using lines in the format:

`DATA,<scan_idx>,<disp_cm>,<angle_hundredths>,<dist_mm>`

The MATLAB script reads these values, converts angle and distance units, and computes:

* `Y = R * cos(theta)`
* `Z = R * sin(theta)`
* `X = displacement`

## Repository Structure

* `firmware/` – embedded C source files for the MSP432E401Y
* `matlab/` – MATLAB script for serial parsing and 3D visualization
* `docs/` – report, project specification, and project images

## Key Learning Outcomes

* Embedded firmware development for hardware control
* Polling-based system design
* I²C sensor integration and UART communication
* Stepper motor sequencing and timing control
* Real-time data acquisition and PC-side visualization
* Converting raw sensor measurements into meaningful spatial data

## Notes

This project was built as an individual course project. The implementation prioritizes clarity, modularity, and reliable hardware-software integration for indoor spatial measurement.

## Images

Add photos or screenshots here:

* hardware setup
* wiring or block diagram
* MATLAB 3D output

## Author

Maryam Abbasi
