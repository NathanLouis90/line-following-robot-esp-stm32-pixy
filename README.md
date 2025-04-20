# Project Description
This project involves using an STM32F303RE Nucleo Board to interface with several components:
- ESP WiFi Module for setting up a website and input AT commands from a serial terminal
- Pixy2 Camera for detecting vector lines, barcodes, and intersections
- Makerdrive Motor Driver that connects to motors, and a combined photointerruptor module and velocimetric wheel to track velocity

## High-Level System Architecture
The following diagram defines the various elements that comprise the robot system, which performs communication, sensing, and navigation.

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
To avoid race conditions, UART TX can only perform transmission when it is in the idle state, as the UART RX is much faster than the UART TX; hence, a state sequence is implemented such that it does not lead to data loss or buffer overrun. Take note that UART TX is immediately triggered when UART RX receives data, and reception is always restarted after received data has been parsed properly. It is also possible for the user to implement timeouts to "delay" the transmission, however, this is not implemented here. 

![UART State Machine drawio](https://github.com/user-attachments/assets/b936698d-a7e0-4a21-a1f5-3a4f9c68565f)

There are two scenarios in which the UART can be in: either the UART received data properly, or it encountered an error. For the former, the process is as follows:
1. UART RX receives data and processes AT Command (and immediately starts reception again)
2. UART TX gets triggered by a loaded data size and immediately starts transmission

For error handling, the process is as follows:
1. UART RX encounters an error, and the software interrupt error callback is called
2. UART RX aborts reception, clears error flags, and restarts reception

## TEST Mode UARTs and ESP Dataflow
In this robot system, UART1 and UART2 are being utilized to interface user-input AT Commands between the serial terminal and ESP WiFi Module during TEST mode.

![UART Dataflow drawio](https://github.com/user-attachments/assets/d1a736f5-8fc5-4d41-a980-9415c7016510)

1. UART2 RX receives AT Command keyed in by the user on a serial terminal and processes it
2. UART1 TX gets triggered by a loaded data size and transmits to the ESP WiFi Module
3. ESP RX receives data and starts processing
4. ESP TX sends back a response to UART1 RX 
5. UART1 RX receives data, processes it, and loads up the UART2 TX data size
6. UART2 TX gets triggered by a loaded data size and starts transmitting back to the serial terminal

__Take note that the ESP WiFi Module sends back responses in chunks, hence UART1 RX must continuously be in reception mode to avoid missing reception of important data from ESP!!!__

## Website Setup and AUTO Mode State Transition Machine Logic
When the user keys in "AUTO" into the serial terminal, a process of setting up the website ensues. A back-and-forth process of transmission and reception occurs, where it waits and checks the responses and transitions to the next state if the response is valid. If not, it transitions back to its old state, where it restarts the process as defined in the old state.

![ESP State Machine drawio](https://github.com/user-attachments/assets/eea7eca6-e5c3-4d50-9be8-956e42bde6a5)

1. If UART1 TX has not transmitted, send AT Command via UART1 TX to ESP RX
2. ESP RX receives AT Command and starts processing 
3. ESP TX sends back a response
4. UART1 RX receives ESP response
5. If a valid response is received, then move on to the next state
6. Otherwise, return to the previous state and recurse back to step 1
7. Repeat steps 1 to 5 for the next state until the serial terminal displays "CIPSTATUS:2"
8. Let the user key in the IP Address on an external device to run the website
9. Repeat steps 1 to 5 until the website is successfully set up

## Website Communication State Machine Logic
Once the website is successfully set up, it will allow the user to toggle the LEDs or display the information parsed by the Pixy. CIPSEND is a command used to send data over a TCP connection. CIPCLOSE is a command used to close a connection.

![Website Communication State Machine drawio](https://github.com/user-attachments/assets/33845167-40ff-4a35-b497-2126a1ae71bb)

1. UART1 RX receives data when the user interacts with the website
2. If "GET /favicon" is received, respond with 'CIPCLOSE=CONNECTION_ID' to effectively manage spam requests initiated by the website's favicon request
3. If "GET /COMMAND_ID" (COMMAND_ID can be either "1", "2", or "3" as stated in the HTML code) is received, perform either toggling of LEDs or updating of the website of Pixy movement state, and transition to the next state
4. If ">" is received, send "CIPSEND=CONNECTION_ID, DATA_SIZE", where DATA_SIZE corresponds to the length of data you wish to transmit to the website (in the HTML code, it is 2), and transition to the next state
5. Send the corresponding command to the website to display ON or OFF for LEDs, or display the Pixy movement command state
6. If "SEND OK" is received, send "AT+CIPCLOSE=CONNECTION_ID" to finish handling the TCP request to allow the website to display changes on the website and transition to "WaitingWebsiteSend" state to parse future incoming data from the user

## Pixy SPI Interface State Machine Logic 
The SPI state machine logic works rather similarly to UART, however, a key difference is that data is handled synchronously (where SS is pulled low to trigger the CLK signal), hence error handling for frame or noise is not necessary for SPI. 

![Pixy State Machine drawio](https://github.com/user-attachments/assets/3ce625a3-bb5f-4a23-8383-5859cb04a115)

1. UART2 RX receives a hex command from the serial terminal keyed by the user
2. UART2 RX processes it and triggers Pixy to start transmitting via SPI TX IT
3. SPI TX CPLT Callback will be called when MOSI finishes transmitting, which will trigger MISO to start reception
4. SPI RX CPLT Callback will be called once data has been fully received by Pixy, allowing data to be parsed
5. The parsed data will be appended with a checksum suffix depending on the checksum equivalence, and trigger UART2 TX to begin transmitting back to the serial terminal

During AUTO mode, the UART2 RX will be idle, and the process will begin at step 2, where the "GET ALL" hex command is transmitted to the Pixy at specific intervals.

## Motor State Machine Logic
This state machine is straightforward, as it only involves the parsing of data received from the user at the serial terminal.

![Motor State Machine drawio](https://github.com/user-attachments/assets/8a545432-6315-4858-b5b6-da6625c54247)

1. UART2 RX receives an "at" command, and parses it by comparing it to other "at" commands
2. Depending on the "at" command, an action will be performed, and a corresponding response will be sent back to the serial terminal via UART2 TX
