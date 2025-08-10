# Project Description
This project involves using an STM32F303RE Nucleo Board to interface with several components:
- ESP WiFi Module for setting up a website and input AT commands from a serial terminal
- Pixy2 Camera for detecting vector lines, barcodes, and intersections
- Makerdrive Motor Driver that connects to motors, and a combined photointerruptor module and velocimetric wheel to track velocity

## System Hierarchy
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
On a high level, the ESP WiFi Module will act as a server, and the user who keyed in the IP address on their gadget will act as the client. Once the website is successfully set up, the user can toggle the LEDs or display the information parsed by the Pixy. The client will send a bunch of GET Requests (when they press on the website) to the ESP, which will respond for e.g, toggling of an LED. Moreover, the STM32 can send several AT Commands to the ESP Server. An example is CIPSEND, a command to send data over a TCP connection, or CIPCLOSE, a command to close a connection.

![Website Communication State Machine drawio](https://github.com/user-attachments/assets/33845167-40ff-4a35-b497-2126a1ae71bb)

1. UART1 RX receives data when the user interacts with the website
2. If "GET /favicon" is received, respond with 'CIPCLOSE=CONNECTION_ID' to effectively manage spam requests initiated by the website's favicon request
3. If "GET /COMMAND_ID" (COMMAND_ID can be either "1", "2", or "3" as stated in the HTML code) is received, perform either toggling of LEDs or updating of the website of Pixy movement state, and transition to the next state
4. If ">" is received, send "CIPSEND=CONNECTION_ID, DATA_SIZE", where DATA_SIZE corresponds to the length of data you wish to transmit to the website (in the HTML code, it is 2), and transition to the next state
5. Send the corresponding command to the website to display ON or OFF for LEDs, or display the Pixy movement command state
6. If "SEND OK" is received, send "AT+CIPCLOSE=CONNECTION_ID" to finish handling the TCP request to allow the website to display changes on the website and transition to "WaitingWebsiteSend" state to parse future incoming data from the user

## Pixy SPI Interface State Machine Logic 
The SPI state machine logic works rather similarly to UART; however, a key difference is that data is handled synchronously (where SS is pulled low to trigger the CLK signal), hence error handling for frame or noise is not necessary for SPI. 

![Pixy State Machine drawio](https://github.com/user-attachments/assets/3ce625a3-bb5f-4a23-8383-5859cb04a115)

1. UART2 RX receives a HEX command from the serial terminal keyed by the user
2. UART2 RX processes it and triggers Pixy to start transmitting via SPI TX IT
3. SPI TX CPLT Callback will be called when MOSI finishes transmitting, which will trigger MISO to start reception
4. SPI RX CPLT Callback will be called once data has been fully received by Pixy, allowing data to be parsed
5. The parsed data will be appended with a checksum suffix depending on the checksum equivalence, and trigger UART2 TX to begin transmitting back to the serial terminal

During AUTO mode, the UART2 RX will be idle, and the process will begin at step 2, where the "GET ALL" HEX command is transmitted to the Pixy at specific intervals.

## Motor State Machine Logic
This state machine is straightforward, as it only involves the parsing of data received from the user at the serial terminal.

![Motor State Machine drawio](https://github.com/user-attachments/assets/8a545432-6315-4858-b5b6-da6625c54247)

1. UART2 RX receives an "at" command, and parses it by comparing it to other "at" commands
2. Depending on the "at" command, an action will be performed, and a corresponding response will be sent back to the serial terminal via UART2 TX

During AUTO mode, similar to the Pixy logic, UART2 RX will be idle. However, the movement of the motors will now depend on what the Pixy camera detects and processes.

## PID Controller
The PID Controller prevents the robot from deviating from left to right when both wheels are set to the same velocity, since each motor and wheel is different. Below shows a crude example of how PID is applied in the project:

![PID Controller drawio](https://github.com/user-attachments/assets/5fd64306-6b22-4d53-90ab-6c417cf26175)

For PID, there are two primary concepts: the coefficient (constant), which is tuned and adjusted to achieve the desired outcome, such as preventing overshooting or steady-state error; and the error, which is derived from using sensors to measure the controlled variable.

PID consists of three components:
1. Proportional Control [kp * error]
2. Derivative Control [kd * (current_error - previous_error) / time_taken_between_these_errors)]
3. Integral Control [ki * time_taken_between_each_pid_control_applied]

Proportional Control corrects the current error by applying an output proportional to the error. For example, with an error of 100 m/s and a kp of 0.5, the controller outputs 50 m/s as a correction. As the error decreases, so does the correction, often leaving a small steady-state error because the output becomes zero when the error is zero.

Integral Control addresses this steady-state error by accumulating past errors and adjusting the output accordingly. However, excessive accumulation can lead to overshooting the setpoint or “integral windup,” causing large, sometimes sudden corrections.

Derivative Control predicts future error trends by responding to the rate of change of the error. This helps dampen rapid changes and reduces overshoot, improving stability and response.

## Areas for Improvement
### 1. UART AT Commands
Sending "AT+RST" or "AT+RESTORE" leads to errors akin to UART despite resetting and restarting the reception. A solution is to accumulate data (UART1 RX will start accumulating data) at the instance UART1 TX transmits the command to the ESP, set a boolean flag to start the timeout, which pauses the UART2 TX from transmitting straightaway. Then, check the timeout in the while loop to ascertain whether UART2 TX is ready to transmit to the serial terminal. The code is as follows:

```bash
if (ESP.started_accumulating) {
  if ( (uwTick - ESP.last_received_time) >= ESP.timeout_for_uart2_tx) {
    ESP.started_accumulating = false;
    uart2_tx.data_size = ESP.size_accumulated; // accumulated in the rx event callback from the esp

    ESP.size_accumulated = 0; // reset the size
  }
}
```

### 2. UART TX Dependency on UART RX
As mentioned above, UART TX will trigger immediately when UART RX receives data, but since there are some issues, such as race conditions and the fact that UART RX and UART TX operate at different clock cycles, it might be better to decouple UART RX and UART TX. One way is to set two new pointers for UART TX: __start__, which points to the beginning of transmission, and __end__, which points to the instance where a __'\0'__ is detected. UART TX will continuously scan the buffer for contents to transmit (at an interval), thus acting independently from UART RX. The code is as follows:

```bash
uart_tx.end = (uint8_t *) strchr((char*) uart_tx.start, '\0');
uart_tx.data_size = uart_tx.end - uart_tx.start;
uart_tx.end += 1; // for the null char
```

### 3. Simpler Pixy Line-Following Algorithm
The current line-following algorithm is quite complex. We can use a simpler technique that involves a tracking vector. 
The algorithm is as follows:

__Handle Barcodes -> Check for Vectors -> Orient Vectors to Point in Same Direction -> Find Next Vector to Follow -> Make Steering Decision__

When the Pixy sees a barcode, it will be made to perform a corresponding action that is tagged to the barcode and gets out of the function (e.g., the robot is made to move forward when it sees barcode 1).
Otherwise, it will proceed to check for vectors. If there is none, the robot will be made to stop. 
The vectors has to be oriented as the Pixy camera will randomly assign vectors, hence there must be an algorithm to ensure that the vectors are aligned in the same direction. You can refer to the following image for a better picture (no pun intended):

![Orientation Diagram drawio](https://github.com/user-attachments/assets/4f30ab07-7723-47dc-8376-ef79569a5674)

It then seeks for a vector that is the shortest in terms of horizontal width by quickly running through the vector list and then storing it into a variable.
Finally, it would make a steering decision based on the vector chosen (e.g., if the shortest vector lies on the left hand side of the Pixy view, it will be made to turn left).
The pseudocode is as such:

```bash
// 1. Handle Barcodes
if (Pixy.barcode_detected) {
  switch (Pixy.barcode_value) {
    case BarcodeForward:
      Pixy.movement = Forward;
      break;
    case BarcodeLeft:
      Pixy.movement = Left;
      break;
    case BarcodeRight:
      Pixy.movement = Right;
      break;
    case BarcodeStop:
      Pixy.movement = Stop;
      break;
    default:
      Pixy.barcode_detected = false;
      break;
  }

  if (Pixy.barcode_detected) {
    return; // get out because barcode is detected
  }
}

// 2. Check for vectors
if (Pixy.num_of_vectors == 0) {
  Pixy.movement = Stop;
  return;
}

// 3. Orient Vectors to Point in Same Direction
// set the reference vector
int ref_x1 = Pixy.vector[0].x1
int ref_y1 = Pixy.vector[0].y1

for (int i = 1; i < Pixy.num_of_vectors; ++i) {
  int x0 = Pixy.vector[i].x0
  int y0 = Pixy.vector[i].y0
  int x1 = Pixy.vector[i].x1
  int y1 = Pixy.vector[i].y1

  if (x1 == ref_x1 && y1 == ref_y1) {
    Pixy.vector[i].x0 = x1
    Pixy.vector[i].y0 = y1
    Pixy.vector[i].x1 = x0
    Pixy.vector[i].y1 = y0

    // recalculate angle
    Pixy.angle = calculate_angle(Pixy.vector[i].x0, Pixy.vector[i].x1, Pixy.vector[i].y0, Pixy.vector[i].y1);
  }
}

// 4. Find Next Vector to Follow
int smallest_x1 = INT16_MAX;
int index_of_smallest_x1 = -1;

// look for vectors that connect to the end of vector[0]
for (int i = 1; i < Pixy.num_of_vectors; i++) {
    if (Pixy.vector[i].x0 == Pixy.vector[0].x1 && 
        Pixy.vector[i].y0 == Pixy.vector[0].y1) {
        
        if (Pixy.vector[i].x1 < smallestX1) {
            smallest_x1 = Pixy.vector[i].x1;
            index_of_smallest_x1 = i;
        }
    }
}

// 5. Make Steering Decision
int8_t x_error = CAMERA_CENTER_X - Pixy.vector[0].x1;

if (abs(x_error) < THRESHOLD_X) {
    Pixy.movement = Forward;
} else if (x_error > THRESHOLD_X) {
    Pixy.movement = Left;
} else if (x_error < -THRESHOLD_X) {
    Pixy.movement = Right;
}
```

### 4. State Machines for Pixy
We can introduce two state machines to allow the Pixy to detect accurately, allowing the entire robotic system to move smoothly.
The first is a location state machine that determines the state in which the Pixy sees. So this could refer to lines, y-intersections, cross-intersections and more. The pseudocode is as follows:

```bash
switch (Pixy.location) {
  case Line:
    // check_for_lines();
  case YIntersection:
    // check_for_y_intersection();
  case CrossIntersection:
    // check_for_cross_intersection();
  ...
}
```

The second is a navigation state machine that determines the state in which the robot is currently moving. This can be forward, left or right. The pseudocode is as follows:

```bash
switch (Pixy.navigation) {
  case Forward:
    // move_forward();
  case RotateLeft:
    // rotate_left();
  case RotateRight:
    // rotate_right();
  ...
}
```

## Physical Robot

![image](https://github.com/user-attachments/assets/062bcdc3-357d-4031-be48-aa59b8dc0d5b)

![image](https://github.com/user-attachments/assets/5a8ab0e6-814e-4b69-8a84-a25da43eb843)

## Miscellaneous
There is an alternative way of collecting and parsing data from the ESP, which is known as the Ledger method, slightly different from the current direct buffer management implemented in my code. The ledger is a 2d array (index and contents) in which there is an rx pointer that reads data from ESP and immediately stores it into the ledger, and a tx pointer that transmits data in the ledger if there are any contents to transmit. The transmission can be immediately triggered as the current one, but only if the transmitter is not busy. The code for the struct is as follows:

```bash
typedef struct ledger_struct {
  uint8_t ledger[100][4096];
  uint8_t* rx_ptr;
  uint8_t* tx_ptr;
  uint8_t* threshold_ptr;
} Ledger_Entry_t;
```

Then include a function in the ESP state machine to transmit when the transmitter is ready. Note that the ledger can only contain 100 entries, hence, the threshold pointer can be used to prevent buffer overflow. 

## References
1. https://www.st.com/resource/en/reference_manual/rm0316-stm32f303xbcde-stm32f303x68-stm32f328x8-stm32f358xc-stm32f398xe-advanced-armbased-mcus-stmicroelectronics.pdf
2. https://www.datasheethub.com/espressif-esp8266-serial-esp-01-wifi-module/#google_vignette
3. https://arduinogetstarted.com/tutorials/arduino-http-request
4. https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:overview
5. https://www.youtube.com/watch?v=tFVAaUcOm4I
