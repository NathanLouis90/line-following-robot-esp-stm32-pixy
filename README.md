# line-following-robot-esp-stm32-pixy
This project involves using an STM32F303RE Nucleo Board to interface with several components, such as:
- ESP WiFi Module for setting up a website and input AT commands from a serial terminal
- Pixy2 Camera which detects vector lines, barcodes, and intersections
- Makerdrive Motor Driver that connects to motors, and a combined photointerruptor module and velocimetric wheel to track velocity

The peripherals used on the Nucleo Board are as follows:
- UART DMA with Interrupts
- SPI with Interrupts
- TIMER 
- GPIO External Interrupts
- I2C

The software uses finite state machine logic to avoid race conditions, structs for debugging and variable access, modular functions, etc.
