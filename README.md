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

## TEST Mode UARTs and ESP Dataflow
![UART Dataflow drawio](https://github.com/user-attachments/assets/d1a736f5-8fc5-4d41-a980-9415c7016510)
1. UART2 RX receives AT Command keyed in by the user on a serial terminal and processes it
2. UART1 TX gets triggered by a loaded data size and transmits to ESP WiFi Module
3. ESP RX receives data and starts processing
4. ESP TX sends back a response to UART1 RX 
5. UART1 RX receives data, processes it, and loads up UART2 TX data size
6. UART2 TX gets triggered by a loaded data size and starts tranmitting back to serial terminal
__Take note that the ESP WiFi Module sends back responses in chunks, hence UART1 RX must continuously be in reception mode to avoid missing data__

## Website Setup and AUTO Mode State Transition Machine Logic
![ESP State Machine drawio](https://github.com/user-attachments/assets/3918f1a6-4c21-4383-9667-d6dc75230058)
1. UART1 TX sends AT Command to ESP RX
2. ESP RX receives AT Command and starts processing
3. ESP TX sends back a response
4. UART1 RX receives ESP response
5. Code checks for a valid response
6. If a valid response is received, then move on to the next state
7. Otherwise, remain in the same state and recurse back to step 1
8. Repeat step 1 for the next state until serial terminal display "CIPSTATUS:2"
9. Let the user key in IP Address onto an external device to run website
10. Repeat step 1 until website is successfully setup


