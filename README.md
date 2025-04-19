# Project Description
This project involves using an STM32F303RE Nucleo Board to interface with several components, such as:
- ESP WiFi Module for setting up a website and input AT commands from a serial terminal
- Pixy2 Camera which detects vector lines, barcodes, and intersections
- Makerdrive Motor Driver that connects to motors, and a combined photointerruptor module and velocimetric wheel to track velocity

## High-Level System Architecture
![High-Level System Architecture drawio](https://github.com/user-attachments/assets/eef70d12-ff19-4b28-8de5-2cff0da7fcde)

## Peripherals 
- UART DMA with Interrupts
- SPI with Interrupts
- TIMER 
- GPIO External Interrupts
- I2C

## Software Overview
- finite state machine logic to avoid race conditions
- structs for debugging and relevant variable access
- functions to make code more modular 
- macros to make code changes easier
- enums to make code more readable

## UART State Machine Logic
![UART State Machine drawio](https://github.com/user-attachments/assets/b936698d-a7e0-4a21-a1f5-3a4f9c68565f)


