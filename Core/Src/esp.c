/*
 * Filename:        esp.c
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 0.0  | Defined basic structs for uarts, error handling code
 * 		  as well as state machine logic that is based on a
 * 		  timeout system that handles transmission based on
 * 		  AT command received
 *
 * Version 1.0  | Enhanced the state machine logic by removing the
 * 		  timeout system and replaced with immediate tx of
 * 		  AT response from ESP WiFi module
 *
 * Version 2.0  | Added a function to allow changing of system
 * 		  mode in the event that the user would like to test
 * 		  other components such as pixy camera or motors
 *
 * Version 3.0  | Added the functions related to Auto Mode which
 * 		  includes setting up of website by sending a chain
 * 		  of AT commands
 *
 * Version 4.0  | Enhanced the state machine logic under AUTO mode
 * 		  to allow sending and processing of TCP requests
 * 		  and responses
 *
 * Description:
 * This file defines the two uarts used to communicate with the user
 * via a serial terminal and the ESP WiFi module which allows the
 * setup of a website server running on HTML code. It includes
 * several functionalities such as a state machine logic to handle
 * uart reception and processing of data to prevent race conditions,
 * error handling, as well as sending and receiving of TCP requests
 * and responses with a website which allows control of LEDs and
 * updating of movement commands based on pixy camera.
 * Below shows an abstract description of the communication flow:
 *  _______________________________________________
 * |   USER PC   |   UART2   |   UART1   |   ESP   |
 * |-------------|-----------|-----------|---------|
 * |     TX     --->  RX    --->  TX    ---> RX    |
 * |     RX     <---  TX    <---  RX    <--- TX    |
 * |_____________|___________|___________|_________|
 *
 * Take note that there exist a virtual bridge between UART2 and
 * USER PC which is the ST-Link Debugger
 *
 */

#include "esp.h"
#include "pixy.h"
#include "motor.h"

UART_Rx_t g_UART2_rx = { .huart = &huart2, .hdma_rx = &hdma_usart2_rx, .state =
		RxIdle, .data_size = 0, .hal_status = HAL_OK, .identifier = "2",
		.leftover_size =
		MAX_BUFFER_SIZE, };

UART_Tx_t g_UART2_tx = { .huart = &huart2, .hdma_tx = &hdma_usart2_tx, .state =
		TxIdle, .data_size = 0, .hal_status = HAL_OK, .identifier = "2",
		.rx_rewinded_state = RX_NOT_REWINDED, };

UART_Rx_t g_UART1_rx = { .huart = &huart1, .hdma_rx = &hdma_usart1_rx, .state =
		RxIdle, .data_size = 0, .hal_status = HAL_OK, .identifier = "1",
		.leftover_size = MAX_BUFFER_SIZE, };

UART_Tx_t g_UART1_tx = { .huart = &huart1, .hdma_tx = &hdma_usart1_tx, .state =
		TxIdle, .data_size = 0, .hal_status = HAL_OK, .identifier = "1",
		.rx_rewinded_state = RX_NOT_REWINDED };

ESP_t g_ESP =
		{ .system_mode = AutoMode, .system_message = { 0 },
				.at_commands = // refer to esp.h ESPState_t for more info on indexing
						{ "AT\r\n", "AT+CWMODE=1\r\n",
						  // Replace SSID and PWD with your own here hotspot settings
						  "AT+CWJAP=\"SSID\",\"PWD\"\r\n",
						  "AT+RST\r\n", "AT+CIFSR\r\n", "AT+CIPMUX=1\r\n",
						  "AT+CIPSERVER=1,80\r\n", "AT+CIPSTATUS\r\n",
						  "AT+CIPSEND=0,1917\r\n", "AT+CIPCLOSE=0\r\n",
						  "<html lang=\"en\"><head><meta charset=\"UTF-8\"><meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>g_ESP-01S Client</title></head><body><h3>Click on the Button to toggle the LED</h3><p>Blue LED State <span id=\"BlueLed\">____</span> </p><p>Red LED State <span id=\"RedLed\">____</span> </p><button id=\"1\" onclick=\"ledToggle(this.id)\">Blue LED</button><button id=\"2\" onclick=\"ledToggle(this.id)\">Red LED</button><p>Line Pattern Sheet <span id=\"LinePat\">____</span> </p><button id=\"update\" onclick=\"startUpdating()\">Update Continuously</button><button id=\"stop\" onclick=\"stopUpdating()\" disabled>Stop Update</button><script>let updateInterval;function ledToggle(clicked_id){var xhr=new XMLHttpRequest();xhr.open(\"GET\",clicked_id,true);xhr.send();xhr.onreadystatechange=function(){if(this.readyState==4&&this.status==200){if(this.responseText==\"10\"){document.getElementById(\"BlueLed\").innerHTML=\"OFF\";}else if(this.responseText==\"11\"){document.getElementById(\"BlueLed\").innerHTML=\"ON\";}else if(this.responseText==\"20\"){document.getElementById(\"RedLed\").innerHTML=\"OFF\";}else if(this.responseText==\"21\"){document.getElementById(\"RedLed\").innerHTML=\"ON\";}else if(this.responseText==\"30\"){document.getElementById(\"LinePat\").innerHTML=\"Feed\";}else if(this.responseText==\"31\"){document.getElementById(\"LinePat\").innerHTML=\"Left\";}else if(this.responseText==\"32\"){document.getElementById(\"LinePat\").innerHTML=\"Right\";}else if(this.responseText==\"33\"){document.getElementById(\"LinePat\").innerHTML=\"Stop\";}}};}function startUpdating(){document.getElementById(\"update\").disabled=true;document.getElementById(\"stop\").disabled=false;updateInterval=setInterval(()=>{ledToggle(\"3\");},3000);}function stopUpdating(){clearInterval(updateInterval);document.getElementById(\"update\").disabled=false;document.getElementById(\"stop\").disabled=true;}</script></body></html>", },
				.state = SendingAT, .tx_success = false, .ip_address = { 0 },
				.last_command_tick = 0, .timeout = 500,
				.website_fully_setup = false, .cipsend_sent = false,
				.send_html_page_once = false, .website_start_running = false,
				.html_state = WaitingWebsiteSend, .response = { 0 },
				.website_command = { 0 }, };

extern Pixy_t g_Pixy;
extern Motor_t g_Motor;

static const char *off_blue_led_response = "11";
static const char *on_blue_led_response = "10";
static const char *off_red_led_response = "21";
static const char *on_red_led_response = "20";

void start_uart_rx_to_idle(UART_Rx_t *uart_rx) {

	// Check the threshold
	if (uart_rx->buffer_start > uart_rx->buffer_threshold) {
		// Rewinding
		uart_rx->buffer_start = uart_rx->buffer_base;
		uart_rx->linked_tx->rx_rewinded_state = !RX_NOT_REWINDED; // Tell Tx that Rx has rewinded
	}

	uart_rx->hal_status = HAL_UARTEx_ReceiveToIdle_DMA(uart_rx->huart,
			uart_rx->buffer_start, (uint16_t) uart_rx->leftover_size);

	if (uart_rx->hal_status == HAL_OK) {
//			printf("UART%s HAL_UARTEx_ReceiveToIdleDMA started successfully!\n",
//					uart->UartIdentifier);
	} else {
		if (uart_rx->hal_status == HAL_BUSY) {
			// UART became busy after the check; retry later without error
//				printf("UART%s is busy, retrying...\n", uart->UartIdentifier);
		} else {
			// Handle other errors
			printf("UART%s HAL_UARTEx_ReceiveToIdleDMA failed! Error: %lu\n",
					uart_rx->identifier, uart_rx->hal_status);
			HAL_UART_AbortReceive(uart_rx->huart); // Reset DMA and UART Receiver
			uart_rx->state = RxError;
		}
	}
}
void start_uart_tx(UART_Tx_t *uart_tx) {
	uart_tx->hal_status = HAL_UART_Transmit_DMA(uart_tx->huart,
			uart_tx->buffer_start, uart_tx->data_size);
	if (uart_tx->hal_status == HAL_OK) {
//		printf("UART%s HAL_UART_Transmit_DMA started successfully!\n",
//				(char*) uart->UartIdentifier);
	} else {
//		printf("UART%s HAL_UART_Transmit_DMA failed! Error: %lu\n",
//				uart->UartIdentifier, uart->hal_status);
	}
}

void process_uart_rx_error(UART_Rx_t *uart_rx) {
// Get the error code
	uint32_t currentErrorCode = HAL_UART_GetError(uart_rx->huart);
//	printf("The error code is %u!\n", (uint8_t) currentErrorCode);

// Clear specific error flags based on error type
	if (currentErrorCode & HAL_UART_ERROR_NONE) {
//		printf("No error! :)\n");
		return;
	} else if (currentErrorCode & HAL_UART_ERROR_NE) {
		printf("Noisy!\n");
	} else if (currentErrorCode & HAL_UART_ERROR_ORE) {
		printf("Overrun error detected on UART%s!\n", uart_rx->identifier);
	} else if (currentErrorCode & HAL_UART_ERROR_FE) {
		printf("Frame Error!\n");
	} else if (currentErrorCode & HAL_UART_ERROR_DMA) {
		printf("DMA Transfer Error on UART%s!\n", uart_rx->identifier);
	}

//	HAL_UART_AbortReceive(uart->Huart); // reset DMA and UART Receiver
	HAL_UART_Abort(uart_rx->huart);
	HAL_UART_Abort(uart_rx->linked_tx->huart);
}

void esp_control(ESP_t *esp) { // to be run in the main loop
	// run UART state machines
	uart_state_machine(&g_UART1_rx, &g_UART1_tx);
	uart_state_machine(&g_UART2_rx, &g_UART2_tx);

	if (esp->system_mode == AutoMode) {
		process_esp_auto_mode(&g_UART1_tx);
	}
}

void process_uart_received_data(UART_Rx_t *uart_rx, ESP_t *esp, Pixy_t *pixy, Motor_t *motor) {
	*(uart_rx->buffer_start + uart_rx->data_size) = '\0'; // Change the last character to null
	if (uart_rx->huart->Instance == USART2) {

		// Check for any system mode changes
		check_system_mode_change(uart_rx, esp);

		// process at commands here
		if (strstr(esp->system_message, "test")) {
			g_UART1_tx.buffer_start = g_UART2_rx.buffer_start;
			g_UART1_tx.data_size = g_UART2_rx.data_size;
		}
		// disallow auto mode from sending at commands
		else if (strstr(esp->system_message, "auto")) {
			strcpy(esp->system_message,
					"Change to test mode to test AT commands!\r\n");
			g_UART2_tx.buffer_start = esp->system_message;
			g_UART2_tx.data_size = strlen(esp->system_message);
		}
		// tpix hex commands to be processed here such as AE C1 0E 00
		else if (strstr(esp->system_message, "tpix")) {
			pixy->input_packet_type = *(uart_rx->buffer_start + 2);
			printf("Response packet type: %02X\n", g_Pixy.input_packet_type);
			// trigger spi to start transmitting
			pixy->tx_ptr = uart_rx->buffer_start;
			pixy->tx_data_size = uart_rx->data_size;
			pixy->state = PixyTransmitting;
		}

		// regular user-define motor cmds to be processed here such as at+start
		else if (strstr(esp->system_message, "tmot")) {
			// first clear the contents of motor command
			memset(motor->command, 0, sizeof(motor->command));
			// start to copy contents from uart2rx into motor command buffer
			strcpy(motor->command, uart_rx->buffer_start);
			// change state to allow processing
			motor->state = MotorDataReceived;
		}

		// normal change-system messages to be processed here
		else {
			g_UART2_tx.buffer_start = esp->system_message;
			g_UART2_tx.data_size = strlen(esp->system_message);
		}
	} else if (uart_rx->huart->Instance == USART1) {
		g_UART2_tx.buffer_start = uart_rx->buffer_start;
		g_UART2_tx.data_size = uart_rx->data_size;
	}
	// Shift the Rx pointers to continuously receive data
	uart_rx->buffer_start += uart_rx->data_size; // Shift to the end of the message
	uart_rx->leftover_size -= uart_rx->data_size;

	uart_rx->data_size = 0;
}

void check_system_mode_change(UART_Rx_t *uart, ESP_t *esp) {
	memset(esp->system_message, 0, sizeof(esp->system_message));
	switch (esp->system_mode) {
	case TestMode:
		if (strstr(uart->buffer_start, "TEST")) {
			strcpy(esp->system_message, "Already in Test Mode!\r\n");
		} else if (strstr(uart->buffer_start, "AUTO")) {
			strcpy(esp->system_message, "Switching to Auto Mode!\r\n");
			esp->system_mode = AutoMode;
		} else if (strstr(uart->buffer_start, "TPIX")) {
			strcpy(esp->system_message, "Switching to Tpix Mode!\r\n");
			esp->system_mode = TpixMode;
		} else if (strstr(uart->buffer_start, "TMOT")) {
			strcpy(esp->system_message, "Switching to Tmot Mode!\r\n");
			esp->system_mode = TmotMode;
		} else { // normal AT commands
			strcpy(esp->system_message, "test");
		}
		break;
	case AutoMode:
		if (strstr(uart->buffer_start, "AUTO")) {
			strcpy(esp->system_message, "Already in Auto Mode!\r\n");
		} else if (strstr(uart->buffer_start, "TEST")) {
			strcpy(esp->system_message, "Switching to Test Mode!\r\n");
			esp->system_mode = TestMode;
		} else if (strstr(uart->buffer_start, "TPIX")) {
			strcpy(esp->system_message, "Switching to Tpix Mode!\r\n");
			esp->system_mode = TpixMode;
		} else if (strstr(uart->buffer_start, "TMOT")) {
			strcpy(esp->system_message, "Switching to Tmot Mode!\r\n");
			esp->system_mode = TmotMode;
		} else {
			strcpy(esp->system_message, "auto");
		}
		break;
	case TpixMode:
		if (strstr(uart->buffer_start, "TPIX")) {
			strcpy(esp->system_message, "Already in Tpix Mode!\r\n");
		} else if (strstr(uart->buffer_start, "TEST")) {
			strcpy(esp->system_message, "Switching to Test Mode!\r\n");
			esp->system_mode = TestMode;
		} else if (strstr(uart->buffer_start, "AUTO")) {
			strcpy(esp->system_message, "Switching to Auto Mode!\r\n");
			esp->system_mode = AutoMode;
		} else if (strstr(uart->buffer_start, "TMOT")) {
			strcpy(esp->system_message, "Switching to Tmot Mode!\r\n");
			esp->system_mode = TmotMode;
		} else {
			strcpy(esp->system_message, "tpix");
		}
		break;

	case TmotMode:
		if (strstr(uart->buffer_start, "TMOT")) {
			strcpy(esp->system_message, "Already in Tmot Mode!\r\n");
		} else if (strstr(uart->buffer_start, "TEST")) {
			strcpy(esp->system_message, "Switching to Test Mode!\r\n");
			esp->system_mode = TestMode;
		} else if (strstr(uart->buffer_start, "AUTO")) {
			strcpy(esp->system_message, "Switching to Auto Mode!\r\n");
			esp->system_mode = AutoMode;
		} else if (strstr(uart->buffer_start, "Tpix")) {
			strcpy(esp->system_message, "Switching to Tpix Mode!\r\n");
			esp->system_mode = TpixMode;
		} else {
			strcpy(esp->system_message, "tmot");
		}
		break;
	default:
		break;
	}

}

void reset_uart_rx_buffer(UART_Rx_t *uart_rx) {
	// reset the buffer pointers and variables
	uart_rx->buffer_start = uart_rx->buffer_base;
	uart_rx->buffer_end = uart_rx->buffer_base;
	uart_rx->leftover_size = MAX_BUFFER_SIZE;
	uart_rx->data_size = 0; // reset data size
	uart_rx->linked_tx->rx_rewinded_state = !(RX_NOT_REWINDED); // rewinded

	memset(uart_rx->buffer, 0, sizeof(uart_rx->buffer)); // clear the buffer contents
}

void process_esp_auto_mode(UART_Tx_t *uart_tx) { // will be using uart1 tx
	if ((uwTick - g_ESP.last_command_tick) >= g_ESP.timeout) {
		setup_website();

		if (g_ESP.website_fully_setup) {
			g_ESP.timeout = 10;
			switch (g_ESP.html_state) {
			case WaitingWebsiteSend:
				g_ESP.response_position = g_UART1_rx.buffer_start; // heavy race conditions

				g_ESP.connection_id = extract_connection_id(
						g_ESP.response_position);

				if (strstr(g_ESP.response_position, "GET /favicon")) {
					sprintf(g_ESP.website_command, "AT+CIPCLOSE=%d\r\n",
							g_ESP.connection_id);
					uart_tx->buffer_start = (uint8_t*) g_ESP.website_command;
					uart_tx->data_size = strlen(g_ESP.website_command);

					g_UART1_rx.data_size = 0;
				} else if (strstr(g_ESP.response_position, "GET /1")) {
					// the function below does transition to right arrow state
					toggle_led_and_respond(g_ESP.connection_id,
					BlueLED_GPIO_Port,
					BlueLED_Pin, off_blue_led_response, on_blue_led_response);
				} else if (strstr(g_ESP.response_position, "GET /2")) {
					toggle_led_and_respond(g_ESP.connection_id, RedLED_GPIO_Port,
					RedLED_Pin, off_red_led_response, on_red_led_response);
				} else if (strstr(g_ESP.response_position, "GET /3")) {
					update_website_movement_state(g_ESP.connection_id);
					g_ESP.html_state = RightArrow;
				}

				break;
			case RightArrow:

				if (strstr(g_ESP.response_position, ">")) {
					uart_tx->buffer_start = g_ESP.response;
					uart_tx->data_size = 2;
					g_ESP.html_state = SendOK;
					g_UART1_rx.data_size = 0;
				}
				break;
			case SendOK:
				if (strstr(g_ESP.response_position, "SEND OK")) {
					sprintf(g_ESP.website_command, "AT+CIPCLOSE=%d\r\n",
							g_ESP.connection_id);
					uart_tx->buffer_start = (uint8_t*) g_ESP.website_command;
					uart_tx->data_size = strlen(g_ESP.website_command);

					printf("LED Toggled. Response Sent: %s (ID: %d)\n",
							g_ESP.response, g_ESP.connection_id);

					g_UART1_rx.data_size = 0;
					g_ESP.html_state = WaitingWebsiteSend;
					reset_uart_rx_buffer(&g_UART1_rx); // clear the buffer
				}
				break;
			}
		}
		g_ESP.last_command_tick = uwTick;
	}
}

void setup_website(void) {
	switch (g_ESP.state) {
	case WaitForESPResponse: {
		if (strstr(g_ESP.response_position, "STATUS:2")) {
			LCD_Print2("IP Address: ", g_ESP.ip_address);
		}
		if (strstr(g_ESP.response_position, "CIFSR:STAIP")) {
			printf("Got IP!\n\n");
			parse_staip_response(g_ESP.response_position);
		}
		if (strstr(g_ESP.response_position, "OK")) {
			g_ESP.state = ++g_ESP.previous_state;
			g_ESP.tx_success = false;
			break;
		}

		g_ESP.state = g_ESP.previous_state;
		break;
	}
	case SendingAT:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 100;
			send_command_to_esp(AT, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = SendingAT;
		}
		break;
	case SetCWMode:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 100;
			send_command_to_esp(Cwmode, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = SetCWMode;
		}
		break;
	case ConnectWiFiCwjap:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 2500;
			send_command_to_esp(Cwjap, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = ConnectWiFiCwjap;
		}
		break;
	case ObtainIPCifsr:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 1000;
			send_command_to_esp(Cifsr, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = ObtainIPCifsr;
		}
		break;
	case EnableMultipleConnCipmux:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 100;
			send_command_to_esp(Cipmux, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = EnableMultipleConnCipmux;
		}
		break;
	case SetupCipserver:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 100;
			send_command_to_esp(Cipserver, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = SetupCipserver;
		}
		break;
	case StartTCPCipstatus:
		if (!g_ESP.tx_success) {
			g_ESP.timeout = 2000;
			send_command_to_esp(Cipstatus, &g_UART1_tx, &g_UART1_rx);
			g_ESP.state = WaitForESPResponse;
			g_ESP.previous_state = StartTCPCipstatus;
		}
		break;
	case WaitForTCPRequest:
		if (strstr(g_ESP.response_position, "GET / HTTP/1.1")) {
			g_ESP.timeout = 3000;
			g_ESP.state = SendWebsite;
		}
		break;
	case SendWebsite:
		if (!g_ESP.tx_success && !g_ESP.cipsend_sent) {
			send_command_to_esp(Cipsend0, &g_UART1_tx, &g_UART1_rx);
			g_ESP.cipsend_sent = true;
		}
		if (strstr(g_ESP.response_position, ">")) {
			if (!g_ESP.tx_success && !g_ESP.send_html_page_once) {
				send_command_to_esp(HTML, &g_UART1_tx, &g_UART1_rx);
				g_ESP.state = RunWebsite;
			}
			g_ESP.tx_success = false;
			g_ESP.timeout = 1000;
		}
		break;
	case RunWebsite:
		if (!g_ESP.website_start_running) {
			g_ESP.website_start_running = true;
			g_ESP.timeout = 500; // check every 500 ms for proper running
		} else if (g_ESP.website_start_running
				&& (strstr(g_ESP.response_position, "SEND OK"))) {
			send_command_to_esp(Cipclose0, &g_UART1_tx, &g_UART1_rx);
			g_ESP.website_fully_setup = true;
			g_ESP.tx_success = false;
			g_ESP.state = WebsiteSuccessfullySetup;
		}
		break;
	}
}

void uart_state_machine(UART_Rx_t *uart_rx, UART_Tx_t *uart_tx) {
	switch (uart_rx->state) {
	case RxIdle:
		break;
	case RxDataReceived:
		process_uart_received_data(uart_rx, &g_ESP, &g_Pixy, &g_Motor);
		restart_reception(uart_rx);
		break;
	case RxError:
		process_uart_rx_error(uart_rx);
		reset_uart_rx_buffer(uart_rx);
		while (uart_rx->hal_status != HAL_OK) {
			restart_reception(uart_rx);
		}
		break;
	}

	if (uart_tx->state == TxIdle && uart_tx->data_size > 0) {
		start_uart_tx(uart_tx);
		uart_tx->state = TxSendingData;
	}
}
