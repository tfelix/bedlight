# BedLIGHT

> Motion and MQTT controlled, multi LED Bedlight controller.

This is a motion sensitive LED Bedlight which can be controlled via a MQTT server. It can show various color schemes or solid colors. It uses an ESP32 and MPU-6050 with interrupt triggered data transfer in order to perform a motion analysis.

## Required Hardware

The original project was build with the following hardware:

* ESP32 Dev Board
* MPU-6050
* WS2812b LED Strip
* 5V Power Supply (should have at least 70W or 12A output current, but depends on the number of LED you want to drive)
* A few Resistors and Caps to buffer the power lines to the LED strips to prevent glitches due to voltage drops.

### Circuit

TBD

## Getting Started

After the hardware was assembled you need to copy the `credentials.h.example` to `credentials.h` and fill in your WiFi password as the controller does not support interactive setup (if someone wants to implement it, go for it).

This is an [Platform IO](https://platformio.org/) project. Compiling it by this IDE is therefore recommended.