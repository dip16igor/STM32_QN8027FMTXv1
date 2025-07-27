# STM32 QN8027 FM Transmitter

This project implements a control system for an FM transmitter based on the QN8027 chip using an STM32 microcontroller.

[Русская версия](README_RU.md)

## Hardware Features

- STM32 BluePill board as the main controller
- QN8027 FM transmitter module
- OLED Display (128x64) for user interface
- Rotary encoder with button for frequency and settings control
- I2C interface for communication with QN8027
- Real-time frequency and status display

## Features

- Frequency Range: 76-108 MHz
- Fine frequency tuning with encoder
- Digital display of current frequency
- Status monitoring of the transmitter
- Settings persistence in memory
- Real-time signal strength indication

## Display Information

The OLED display shows:
- Current FM frequency
- Signal strength
- Transmitter status
- Operation mode
- Error indicators (if any)

## Controls

- Rotary Encoder:
  - Rotate: Adjust frequency
  - Press: Switch between modes/confirm settings
- Settings adjustable:
  - Frequency
  - Transmission power
  - Other QN8027 parameters

## Technical Specifications

- Operating Voltage: 3.3V
- I2C Communication
- Frequency Resolution: 0.1 MHz
- Timer-based encoder reading
- Hardware debouncing

## Libraries Used

- Arduino core for STM32
- U8g2lib for OLED display
- HardwareTimer for precise timing
- Wire (I2C communication)

## Building and Flashing

The project uses PlatformIO as the build system. To build and flash:

1. Install PlatformIO
2. Open the project
3. Build using PlatformIO
4. Flash to STM32 board

## Pin Configuration

- PB6, PB7 - I2C (SCL, SDA) for QN8027
- Display Pins:
  - Specified in display configuration
- Encoder Pins:
  - Configured for timer-based reading

## Notes

- QN8027 initialization sequence is critical for proper operation
- All settings are checked for valid ranges
- Real-time frequency adjustment
- Error handling for I2C communication
