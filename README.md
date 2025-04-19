# Project Description
This project involves using an STM32F303RE Nucleo Board to interface with several components:
- ESP WiFi Module for setting up a website and input AT commands from a serial terminal
- Pixy2 Camera for detecting vector lines, barcodes, and intersections
- Makerdrive Motor Driver that connects to motors, and a combined photointerruptor module and velocimetric wheel to track velocity

## High-Level System Architecture
![High-Level System Architecture drawio](https://github.com/user-attachments/assets/eef70d12-ff19-4b28-8de5-2cff0da7fcde)

## Peripherals 
- UART DMA with Interrupts
- SPI with Interrupts
- TIMER 
- GPIO External Interrupts
- I2C

## System Modes
The robot system can be in one of four modes:
- TEST mode (where the user can test the ESP WiFi module with AT Commands)
- TPIX mode (where the user can test Pixy Camera by sending HEX commands)
- TMOT mode (where the user can test motors with code-defined at commands)
- AUTO mode (where the user can set up a website and allow the robot to follow a line autonomously)

## UART Reception and Transmission State Machine Logic
![UART State Machine drawio](https://github.com/user-attachments/assets/b936698d-a7e0-4a21-a1f5-3a4f9c68565f)
1. UART RX receives data and processes AT Command (and immediately starts reception again)
2. UART TX gets triggered by a loaded data size and immediately starts transmission

## Website Setup and AUTO Mode State Transition Machine Logic
![ESP State Machine drawio](https://github.com/user-attachments/assets/3918f1a6-4c21-4383-9667-d6dc75230058)
1. UART1 TX sends AT Command to ESP RX
2. ESP RX receives AT Command and starts processing
3. ESP TX sends back a response
4. UART1 RX receives ESP response
5. Code checks for a valid response
6. If a valid response is received, then move on to the next state
7. Otherwise, remain in the same state and recurse back to step 1
8. Repeat step 1 for the next state until the website is successfully set up


