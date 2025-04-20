/*
 * Filename:        esp.h
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  | Added UART TX and UART RX structs. Added states for
 * 		  UART communication flow as well as states for AUTO
 * 		  mode phase transition. Added basic functions to allow
 * 		  sending, receiving and processing of AT commands
 * 		  from serial terminal.
 *
 * Version 2.0  | Added an ESP struct to track the state during AUTO
 * 		  mode and transmission success. Added more enums to
 * 		  track website state. Added helper functions to
 * 		  initialize and sending of AT commands during AUTO
 * 		  mode, toggling of LEDs and parsing of IP address.
 *
 * Version 3.0  | Added function prototypes to implement setting up of
 * 		  website as well as helper functions to make the code
 * 		  more modular
 *
 * Description:
 * This file describes the functions, macros, structs and enums necessary
 * to allow interfacing of serial terminal and ESP WiFi module. It allows
 * the user to send AT commands to setup a website or allow the STM32 to
 * do it on its own. The file also describe the various states the UART(s)
 * and ESP can occupy during TEST and AUTO mode.
 */

#ifndef ESP_TEST_AUTO_H
#define ESP_TEST_AUTO_H

/* START INCLUDES */
#include <stdio.h>
#include <string.h>
#include <main.h>
#include "stdbool.h"
#include "pixy.h"
#include "motor.h"
/* END INCLUDES */

/* START DEFINE MACROS */
#define MAX_BUFFER_SIZE 4096
#define ESP_RESPONSE_TIMEOUT 9
#define RX_NOT_REWINDED 0

// To check the system state on AT
#define SYSTEM_TEST "TEST\r\n"
#define SYSTEM_AUTO "AUTO\r\n"

// Time
#define ONE_SECOND 1000

/* END DEFINE MACROS */

/* START TYPEDEF */
typedef enum { // For Rx BufferState
	RxIdle, RxDataReceived, RxError,
} ReceiverState_t;

// For Transmitter
typedef enum {
	TxIdle, TxSendingData,
} TransmitterState_t;

typedef enum {
	TestMode, AutoMode, TpixMode, TmotMode,
} SystemMode_t;

typedef enum {
	AT = 0,
	Cwmode = 1,
	Cwjap = 2,
	Rst = 3,
	Cifsr = 4,
	Cipmux = 5,
	Cipserver = 6,
	Cipstatus = 7,
	Cipsend0 = 8,
	Cipclose0 = 9,
	HTML = 10,
} ATCommand_t;

typedef enum {
	SendingAT = 1,
	SetCWMode = 2,
	ConnectWiFiCwjap = 3,
	ObtainIPCifsr = 4,
	EnableMultipleConnCipmux = 5,
	SetupCipserver = 6,
	StartTCPCipstatus = 7,
	WaitForTCPRequest = 8,
	SendWebsite = 9,
	RunWebsite = 10,
	WaitForESPResponse = 11,
	WebsiteSuccessfullySetup = 100 // to prevent error incrementing so set to 100 if it get to 12
} ESPState_t;

typedef enum {
	WaitingWebsiteSend, RightArrow, SendOK,
} WebsiteState_t;

typedef struct __UART_Rx_struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_rx;
	uint8_t buffer[MAX_BUFFER_SIZE];
	uint16_t data_size; // string length
	volatile ReceiverState_t state; // get state of buffer in main function, can be 4 values
	char identifier[2]; // either uart1 or uart2

	// pointer variables for prepending or appending strings
	uint8_t *buffer_base; // actual base of the rx buffer
	uint8_t *buffer_threshold; // 75% of the buffer
	uint8_t *buffer_start; // ptr to start of data received, , will increase more than tx ptr
	uint8_t *buffer_end; // ptr to end of data received, will increase more than tx ptr

	// miscellaneous
	uint16_t leftover_size; // when data receives message, this will reduce
	HAL_StatusTypeDef hal_status; // status of hal, always assign to hal transmit or receive
	struct __UART_Tx_struct *linked_tx;	// tx that is connected to this rx

} UART_Rx_t;

typedef struct __UART_Tx_struct {
	UART_HandleTypeDef *huart;
	HAL_StatusTypeDef hal_status;
	uint16_t data_size; // will be the prefix or message size
	volatile TransmitterState_t state;
	DMA_HandleTypeDef *hdma_tx;

	// pointers
	uint8_t *buffer_base;
	uint8_t *buffer_start; // points to the start of data to be tx
	uint8_t *buffer_end; // points to the end of data to be tx

	char identifier[2]; // either uart1 or uart2
	struct __UART_Rx_struct *linked_rx; // tx that is connected to this rx

	// To handle rewinding
	uint8_t rx_rewinded_state;
} UART_Tx_t;

typedef struct __ESP_struct {
	SystemMode_t system_mode;
	char system_message[30];
	const char *at_commands[20];

	char ip_address[16];
	ESPState_t state, previous_state;
	bool tx_success; // indicate if transmission is a success
	bool website_fully_setup; // indicate whether website is running

	uint32_t last_command_tick; // updates everytime response from esp is check
	uint32_t timeout; // how long each auto phase transition takes place

	uint8_t *response_position;  // pointer to where the response starts

	bool cipsend_sent; // send cipsend when user starts website
	bool send_html_page_once; // indicate whether the html page has been sent yet
	bool website_start_running; // indicate whether or not to cipclose

	WebsiteState_t html_state;
	char website_command[20];
	uint8_t connection_id;
	uint8_t response[4]; // For response codes "10", "11", etc.
} ESP_t;

/* END TYPEDEF */

/* START FUNCTION PROTOTYPES */
// regular esp functions
// when the rx buffer receives something
void process_uart_received_data(UART_Rx_t *uart_rx, ESP_t *esp, Pixy_t *pixy, Motor_t *motor);
void start_uart_rx_to_idle(UART_Rx_t *uart_rx); // hal status checking rx data
void start_uart_tx(UART_Tx_t *uart_tx); // hal status checking tx of data
void process_uart_rx_error(UART_Rx_t *uart_rx); // when rx encounters error, execute this function
void esp_control(ESP_t *esp); // several state machines to control logic of ESP
void restart_reception(UART_Rx_t *uart_rx); // restart rx, state of rx state machine
void check_system_mode_change(UART_Rx_t *uart, ESP_t *esp); // check if user decides to change system state
void process_esp_auto_mode(UART_Tx_t *uart_tx); // process auto mode to start processing stuff like pixy updates, etc
void setup_website(void); // get the website up and running
void uart_state_machine(UART_Rx_t *uart_rx, UART_Tx_t *uart_tx); // state logic of uart rx and uart tx

// helper functions
void init_uart_rx_ptrs(UART_Rx_t *uart_rx);
void init_uart_tx_ptrs(UART_Tx_t *uart_tx);
void link_uarts(UART_Rx_t *uart_rx, UART_Tx_t *uart_tx);
void reset_uart_rx_buffer(UART_Rx_t *uart_rx);
void parse_staip_response(uint8_t *response);
// uart tx == uart1 tx, uart rx = uart1rx
void send_command_to_esp(uint8_t command_index, UART_Tx_t *uart_tx, UART_Rx_t *uart_rx);
uint8_t extract_connection_id(const char *buffer);
void toggle_led_and_respond(uint8_t connection_id, GPIO_TypeDef *gpio,
		uint16_t pin, const char *off_response, const char *on_response);
/* END FUNCTION PROTOTYPES */

/* START EXTERNAL VARIABLES */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
/* END EXTERNAL VARIABLES */

#endif
