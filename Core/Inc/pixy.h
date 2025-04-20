/*
 * Filename:        pixy.h
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  | Added the struct for pixy, basic function prototypes
 * 		  for pixy data, enum for pixy commands, suffixes for
 * 		  checksum
 *
 * Version 2.0  | Added the structs for vectors and barcode information.
 * 		  Added function for extracting features
 *
 * Version 3.0  | Added enums for pixy command movement state to
 * 		  display on website as well as indicators for AFC1
 * 		  response
 *
 * Version 4.0  | Added helper function prototypes for initialisation
 * 		  and reset of struct variables, as well as a line
 * 		  following algorithm function for decision-making
 *
 * Description:
 * This file defines structs, constants, macros for handling SPI
 * communication which allows proper interfacing between pixy camera,
 * STM32 and ESP WiFi module to allow robot to follow a line.
 */

#ifndef INC_PIXY_H_
#define INC_PIXY_H_

/* START INCLUDES */
#include "main.h"
#include "stdbool.h"
#include "stdio.h"
#include "math.h"
/* END INCLUDES */

/* START MACRO */
#define SPI_RECEIVE_SIZE 132
#define SPI_TRANSMIT_SIZE 6
#define	C_PASSED "\r\nC_Passed\r\n" // received checksum == calculated checksum
#define	C_FAILED "\r\nC_Failed\r\n" // received checksum != calculated checksum
#define CAMERA_CENTER_X 39 // to get error difference from center of camera horizontally
#define THRESHOLD_X 10 // should continue moving forward if x is within this number
#define CAMERA_CENTER_Y 25 // to get error difference from center of camera vertically
/* END MACRO */

/* START TYPEDEF */
typedef enum {
	PixyIdle, PixyTransmitting, PixyWaitingResponse, PixyDataReceived,
} PixyState_t;

typedef enum {
	Idle, Forward, Right, Left, Stop,
} PixyMoveCommandState_t;

typedef enum {
	AFC1NotFound = 0, AFC1ChecksumPassed = 1, AFC1ChecksumFailed = 2,
} AFC1Result;

typedef enum { // change according to what barcode you wanna use
	// barcode 6 = halt, barcode 7 = move forward, barcode 8 = move left, barcode 10 = move right
	BarcodeStop = 6,
	BarcodeForward = 7,
	BarcodeLeft = 8,
	BarcodeRight = 10,
	BarcodeNotDetected = 0, // no barcode detected
} BarcodeCommand;

typedef enum {
	VERSION = 0,
	LAMP_ON = 1,
	ALL_FEATURES = 2,
	VECTORS = 3,
	INTERSECTIONS = 4,
	BARCODES = 5,
	EXCEPT_BARCODES = 6,
	EXCEPT_INTERSECTIONS = 7,
	EXCEPT_VECTORS = 8,
	LAMP_OFF = 9,
} PixyCommand_t;

typedef struct __Vector_struct {
	uint8_t x0, y0, x1, y1, line_index, flag;
} Vector_t;

typedef struct __Barcode_struct {
	uint8_t x, y, flag, value;
} Barcode_t;

typedef struct __Pixy_struct {
	SPI_HandleTypeDef *hspi;
	uint8_t rx_buffer[SPI_RECEIVE_SIZE];
	uint8_t *rx_buffer_base;
	uint8_t *tx_ptr;
	uint16_t tx_data_size;

	uint8_t AEC1_commands[20][SPI_TRANSMIT_SIZE]; // where each command is 6 bytes long
	volatile PixyState_t state;
	uint8_t input_packet_type;

	GPIO_TypeDef *ssPort; // slave select port
	uint16_t ssPin; // slave select pin

	// for checksum, afc1 response
	bool checksum_valid;
	uint8_t response_length;
	char response_buffer[SPI_RECEIVE_SIZE];
	uint8_t response_packet_type;
	uint16_t payload_size;
	uint16_t checksum_received;
	uint16_t calculated_checksum;

	// for features
	uint8_t feature_buffer[SPI_RECEIVE_SIZE];
	Vector_t vector[4]; // should be enough to fit an intersection with 4 lines
	uint8_t num_of_vectors;
	uint8_t num_of_branches;
	Barcode_t barcode;
	bool barcode_detected;

	// to be sync with esp
	volatile PixyMoveCommandState_t move_command_state;

	// for auto mode
	uint32_t last_received_time;
	uint16_t timeout;
} Pixy_t;

/* END TYPEDEF */

/* START FUNCTION PROTOTYPES */
void pixy_control(Pixy_t *pixy); // pixy state machine logic
void process_pixy_data(Pixy_t *pixy); // whatever is received in spi rx buffer is processed here
AFC1Result find_afc1_response(Pixy_t *pixy);
void extract_features(void); // get features of what pixy sees
void line_following_algorithm(void); // make decision based on intersection
void update_website_movement_state(int connection_id); // give back response depending on what pixy sees

// helper functions
void init_pixy_struct(Pixy_t *pixy); // initialize some ptrs
// not being used currently
void reset_vector_contents(Pixy_t *pixy);
void reset_barcode_detection(Pixy_t *pixy);

/* END FUNCTION PROTOTYPES */

/* START EXTERN VARIABLES */
extern SPI_HandleTypeDef hspi2;
/* END EXTERN VARIABLES */

#endif /* INC_PIXY_H_ */
