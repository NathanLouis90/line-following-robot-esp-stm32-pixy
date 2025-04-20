/*
 * Filename:        pixy.c
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  | Defined the state logic for pixy camera, added
 * 		  relevant structs which included AEC1 commands, as well
 * 		  as a functions that parses data and check for AFC1
 * 		  response.
 *
 * Version 2.0  | Proper definition of a function to fully extract and
 * 		  parse raw SPI data into useful information
 *
 * Version 3.0  | Enhanced the logic for the parsing of raw SPI data
 *
 * Version 4.0  | Implemented line following algorithm to decide based
 * 		  on the data extracted
 *
 * Version 5.0  | Enhanced the line following algorithm as well as
 * 		  defined a function that interface with the ESP WiFi
 * 		  Module to state its intended movement for the user
 *
 * Description:
 * This file contains the pixy camera logic which interfaces with the
 * STM32 via SPI protocol with interrupts. It has a state machine
 * in which it receives data, parses it into useful information to
 * decide which line should the robot should follow or what decision
 * to make when it encounters an intersection as well as interfacin
 * with the website to communicate its intended movement.
 *
 */

#include "pixy.h"
#include "esp.h"
#include "motor.h"

// global structs
extern ESP_t g_ESP;
extern Motor_t g_Motor;
extern UART_Rx_t g_UART1_rx;
extern UART_Tx_t g_UART1_tx;
extern UART_Rx_t g_UART2_rx;
extern UART_Tx_t g_UART2_tx;

// left, right, forward, stop
static const char* left_response = "31";
static const char* right_response = "32";
static const char* forward_response = "30";
static const char* stop_response = "32";

Pixy_t g_Pixy = {
		.AEC1_commands[0] = { 0xAE, 0xC1, 0x0E, 0x00 }, // get pixy version
		.AEC1_commands[1] = { 0xAE, 0xC1, 0x16, 0x02, 0x01, 0x01 }, // turn on lamp
		.AEC1_commands[2] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x07 }, // get all features
		.AEC1_commands[3] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x01 }, // get all vectors
		.AEC1_commands[4] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x02 }, // get all intersections
		.AEC1_commands[5] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x03 }, // get all barcodes
		.AEC1_commands[6] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x04 }, // get all except barcodes
		.AEC1_commands[7] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x05 }, // get all except intersections
		.AEC1_commands[8] = { 0xAE, 0xC1, 0x30, 0x02, 0x01, 0x06 }, // get all except vectors
		.AEC1_commands[9] = { 0xAE, 0xC1, 0x16, 0x02, 0x00, 0x00 }, // turn off lamp
		.hspi = &hspi2, .rx_buffer = { 0 }, .state = PixyIdle, .ssPort =
		GPIOB, .ssPin = GPIO_PIN_12, .checksum_valid = false, .response_packet_type = 0,
		.payload_size = 0, .calculated_checksum = 0, .checksum_received = 0,
		.tx_data_size = 0, .response_buffer = { 0 }, .feature_buffer = { 0 },
		.num_of_vectors = 0, .move_command_state = Idle, .barcode_detected = false,
		.barcode.value = BarcodeNotDetected, .last_received_time = 0, .response_length =
				0, .timeout = 500, .num_of_branches = 0, };

void pixy_control(Pixy_t *pixy) { // to be run in main loop
	switch (pixy->state) {
	case PixyIdle:
		break;
	case PixyTransmitting:
		memset(pixy->rx_buffer, 0, sizeof(pixy->rx_buffer)); // reset before tx
		HAL_GPIO_WritePin(pixy->ssPort, pixy->ssPin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(pixy->hspi, pixy->tx_ptr, pixy->tx_data_size);
		pixy->state = PixyWaitingResponse;
		break;
	case PixyWaitingResponse:
		break;
	case PixyDataReceived:
		// reset before tx
		process_pixy_data(pixy);
		pixy->state = PixyIdle;
		break;
	}

	// for auto mode, will send the 'get all' cmd every timeout
	if (g_ESP.website_fully_setup && g_ESP.system_mode == AutoMode) {
		if (uwTick - g_Pixy.last_received_time >= pixy->timeout) {
			pixy->tx_ptr = pixy->AEC1_commands[2];
			pixy->tx_data_size = 6;
			pixy->state = PixyTransmitting;
			// update last received time
			pixy->last_received_time = uwTick;
		}
	}
}

void process_pixy_data(Pixy_t *pixy) {
	HAL_GPIO_WritePin(pixy->ssPort, pixy->ssPin, GPIO_PIN_SET); // pull ss to low to start clock

	// reset from previous check
	pixy->checksum_valid = false;
	pixy->response_length = 0;
	pixy->calculated_checksum = 0;
	pixy->checksum_received = 0;

	AFC1Result afc1Result = find_afc1_response(pixy); // find AF C1

	line_following_algorithm(); // make a decision based on data received

	if (afc1Result == AFC1ChecksumPassed) {
		// shift the buffer and start uart tx
		memcpy(pixy->response_buffer + pixy->response_length, C_PASSED,
				strlen(C_PASSED));

		extract_features(); // extract stuff like vectors, barcodes, interections

		// trigger uart tx
		g_UART2_tx.buffer_start = pixy->response_buffer;
		g_UART2_tx.data_size = pixy->response_length + strlen(C_PASSED);
	} else if (afc1Result == AFC1ChecksumFailed) {
		memcpy(pixy->response_buffer + pixy->response_length, C_FAILED,
				strlen(C_FAILED));

		// trigger uart tx
		g_UART2_tx.buffer_start = pixy->response_buffer;
		g_UART2_tx.data_size = pixy->response_length + strlen(C_FAILED);
	} else if (afc1Result == AFC1NotFound) {
		printf("AFC1 not found\n");
	}
}

AFC1Result find_afc1_response(Pixy_t *pixy) {
	for (uint16_t i = 0; i < SPI_RECEIVE_SIZE - 1; i++) {
		if (pixy->rx_buffer[i] == 0xAF && pixy->rx_buffer[i + 1] == 0xC1) {
			printf("Found AF C1 at index %d\n", i);
			pixy->response_packet_type = pixy->rx_buffer[i + 2];
			pixy->payload_size = pixy->rx_buffer[i + 3];
			pixy->response_length = pixy->payload_size + 6; // Sync (2) + Type (1) + Length (1) + Checksum (2)

			// extract received checksum
			pixy->checksum_received = (uint16_t) pixy->rx_buffer[i + 4]
					| ((uint16_t) pixy->rx_buffer[i + 5] << 8);

			// reset calculated checksum
			pixy->calculated_checksum = 0;

			// calculate checksum only over the payload bytes
			for (uint16_t j = i + 6; j < i + 6 + pixy->payload_size; j++) {
				if (j < SPI_RECEIVE_SIZE) { // Prevent buffer overrun
					pixy->calculated_checksum += pixy->rx_buffer[j];
				}
			}

			pixy->checksum_valid = (pixy->checksum_received
					== pixy->calculated_checksum);

			if (pixy->checksum_valid) {
				printf(
						"AFC1 Found and checksum passed! Received: 0x%04X, Calculated: 0x%04X\n",
						pixy->checksum_received, pixy->calculated_checksum);

				// copy data to response buffer and feature buffer
				memset(pixy->response_buffer, 0, sizeof(pixy->response_buffer));
				memset(pixy->feature_buffer, 0, sizeof(pixy->feature_buffer));

				// copy response including header
//				printf("Response bytes: ");
				for (int l = 0;
						l < pixy->response_length && (i + l) < SPI_RECEIVE_SIZE;
						l++) {
					pixy->response_buffer[l] = pixy->rx_buffer[i + l];
//					printf("%02X ", g_Pixy.response_buffer[l]);
				}
//				printf("\n");

				// copy just the payload to feature buffer
//				printf("Feature bytes: ");
				for (int m = 0;
						m < pixy->payload_size && (i + 6 + m) < SPI_RECEIVE_SIZE;
						m++) {
					pixy->feature_buffer[m] = pixy->rx_buffer[i + 6 + m];
//					printf("%02X ", g_Pixy.feature_buffer[m]);
				}
//				printf("\n");

				return AFC1ChecksumPassed;
			} else {
				printf(
						"AFC1 Found but checksum failed! Received: 0x%04X, Calculated: 0x%04X\n",
						pixy->checksum_received, pixy->calculated_checksum);

//				printf("Payload bytes: ");
//				for (uint16_t j = i + 6;
//						j < i + 6 + pixy->payload_size && j < SPI_RECEIVE_SIZE;
//						j++) {
//					printf("%02X ", pixy->rx_buffer[j]);
//				}
//				printf("\n");

				return AFC1ChecksumFailed;
			}
		}
	}

	// could not find any valid AF C1 pattern
	g_Pixy.checksum_valid = false;
	g_Pixy.response_length = 0;
	return AFC1NotFound;
}

void extract_features() {
//	if (g_Pixy.inputPacketType == 0x30) {
		// check if we have valid data to extract
		if (!g_Pixy.checksum_valid || g_Pixy.payload_size == 0) {
			printf("No valid data to extract features\n");
			return;
		}

		// start at base buffer
		uint16_t index = 0;
		uint16_t featureSize = 0;
		// reset our feature structs
		memset(g_Pixy.vector, 0, sizeof(g_Pixy.vector));
		memset(&g_Pixy.barcode, 0, sizeof(g_Pixy.barcode));
		g_Pixy.num_of_vectors = 0;
		g_Pixy.num_of_branches = 0;
		g_Pixy.barcode_detected = false;
		g_Pixy.barcode.value = BarcodeNotDetected;

		// start parsing features
		while (index < g_Pixy.payload_size) {
			featureSize = g_Pixy.feature_buffer[index + 1];
			switch (g_Pixy.feature_buffer[index]) {
			case 0x01: // vector block
				// find the number of vectors by checking the index after
				// then do some processing with the data
				if (featureSize == 0x06) {
					g_Pixy.num_of_vectors = 1;
				} else if (featureSize == 0x0C) {
					g_Pixy.num_of_vectors = 2;
				} else if (featureSize == 0x12) {
					g_Pixy.num_of_vectors = 3;
				} else if (featureSize == 0x18) {
					g_Pixy.num_of_vectors = 4;
				}
				// now put in the data into vector
				for (int i = 0, v = index; i < g_Pixy.num_of_vectors; ++i, v += 7) {
					g_Pixy.vector[i].x0 = g_Pixy.feature_buffer[v + 2];
					g_Pixy.vector[i].y0 = g_Pixy.feature_buffer[v + 3];
					g_Pixy.vector[i].x1 = g_Pixy.feature_buffer[v + 4];
					g_Pixy.vector[i].y1 = g_Pixy.feature_buffer[v + 5];
					g_Pixy.vector[i].flag = g_Pixy.feature_buffer[v + 6];
					g_Pixy.vector[i].line_index = g_Pixy.feature_buffer[v + 7];
				}
				break;
			case 0x02: // intersection
				g_Pixy.num_of_branches = g_Pixy.feature_buffer[index + 4];
				break;
			case 0x04: // barcode
				g_Pixy.barcode.x = g_Pixy.feature_buffer[index + 2];
				g_Pixy.barcode.y = g_Pixy.feature_buffer[index + 3];
				g_Pixy.barcode.flag = g_Pixy.feature_buffer[index + 4];
				g_Pixy.barcode.value = g_Pixy.feature_buffer[index + 5];
				g_Pixy.barcode_detected = true;
				break;
			}
			index += 2 + featureSize;
		}
//		printf("Number of vectors: %lu\nNumber of branches: %lu\n",
//				g_Pixy.num_of_vectors, g_Pixy.num_of_branches);
//		for (int i = 0; i < g_Pixy.num_of_vectors; i++) {
//			printf("Vector %lu: x0 = %lu, y0 = %lu, x1 = %lu, y1 = %lu\n",
//					g_Pixy.vector[i].lineIndex, g_Pixy.vector[i].x0,
//					g_Pixy.vector[i].y0, g_Pixy.vector[i].x1, g_Pixy.vector[i].y1);
//		}
//	} else {
//		printf("Packet type != Get Main/All. Ignore extracting features.\n");
//	}
}

void line_following_algorithm() {
//	if (g_Pixy.inputPacketType == 0x30) {
		// reset previous move command if needed
		PixyMoveCommandState_t prevMoveCmd = g_Pixy.move_command_state;

		// if got barcode
		if (g_Pixy.barcode_detected) {
			printf("Barcode detected with value: %d\n", g_Pixy.barcode.value);

			switch (g_Pixy.barcode.value) {
			case BarcodeForward:
				g_Pixy.move_command_state = Forward;
//				printf("Barcode command: Forward\n");
				break;
			case BarcodeLeft:
				g_Pixy.move_command_state = Left;
//				printf("Barcode command: Left\n");
				break;
			case BarcodeRight:
				g_Pixy.move_command_state = Right;
//				printf("Barcode command: Right\n");
				break;
			case BarcodeStop:
				g_Pixy.move_command_state = Stop;
//				printf("Barcode command: Stop\n");
				break;
			default:
//				printf("Unknown barcode value: %d, ignoring\n",
//						g_Pixy.barcode.value);
				g_Pixy.barcode_detected = false; // ignore barcode detected and proceed
				// break is intentionally left out to allow following lines to be processed
			}
			return; // do not need to do more processing
		}

		// check for intersection if no barcode
		if (!g_Pixy.barcode_detected && g_Pixy.num_of_branches > 0) {
			printf("Intersection detected with %d branches\n",
					g_Pixy.num_of_branches);

			if (g_Pixy.num_of_vectors == 3) { // detected a Y junction
				// We need to decide which branch to follow - left, right, or forward
				int16_t left_error = INT16_MAX;  // initialize with large values
				int16_t right_error = INT16_MAX;
				int16_t forward_error = INT16_MAX;
				// do remember that index is set negative
				int8_t left_index = -1;
				int8_t right_index = -1;
				int8_t forward_index = -1;

				// First, identify which vector is which (left, right, forward)
				// At a Y junction, we classify vectors by their endpoints' positions relative to center
				for (int i = 0; i < g_Pixy.num_of_vectors; i++) {
					// Calculate the angle of the vector relative to vertical
					float dx = g_Pixy.vector[i].x1 - g_Pixy.vector[i].x0;
					float dy = g_Pixy.vector[i].y1 - g_Pixy.vector[i].y0;
					float angle = atan2f(dx, dy) * (180.0f / M_PI); // angle in degrees

					// Classify based on angle
					// Note: Adjust these thresholds based on your camera orientation and mounting
//					printf("Vector %d angle: %.2f degrees\n", i, angle);

					if (angle < -30.0f) {  // Left branch
						left_index = i;
						left_error = abs(CAMERA_CENTER_X - g_Pixy.vector[i].x1);
					} else if (angle > 30.0f) {  // Right branch
						right_index = i;
						right_error = abs(CAMERA_CENTER_X - g_Pixy.vector[i].x1);
					} else {  // Forward branch (approximately straight)
						forward_index = i;
						forward_error = abs(CAMERA_CENTER_X - g_Pixy.vector[i].x1);
					}
				}

				// print identified branches and their errors
//				printf("Left branch (index %d): error = %d\n", leftIndex,
//						leftError);
//				printf("Right branch (index %d): error = %d\n", rightIndex,
//						rightError);
//				printf("Forward branch (index %d): error = %d\n", forwardIndex,
//						forwardError);

				// determine which direction has the smallest error
				if (left_index != -1 && left_error <= right_error
						&& left_error <= forward_error) {
					g_Pixy.move_command_state = Left;
					printf("Y-junction decision: Left (smallest error)\n");
				} else if (right_index != -1 && right_error <= left_error
						&& right_error <= forward_error) {
					g_Pixy.move_command_state = Right;
					printf("Y-junction decision: Right (smallest error)\n");
				} else if (forward_index != -1) {
					g_Pixy.move_command_state = Forward;
					printf("Y-junction decision: Forward (smallest error)\n");
				} else {
					// by default, just move forward
					g_Pixy.move_command_state = Forward;
					printf(
							"Couldn't make proper decision at Y-junction. Defaulting to Forward\n");
				}
			} else if (g_Pixy.num_of_vectors == 4) { // detected a cross intersection
				// similar for cross intersection
				int16_t left_error = INT16_MAX;
				int16_t right_error = INT16_MAX;
				int16_t forward_error = INT16_MAX;
				int8_t left_index = -1;
				int8_t right_index = -1;
				int8_t forward_index = -1;

				for (int i = 0; i < g_Pixy.num_of_vectors; i++) {
					float dx = g_Pixy.vector[i].x1 - g_Pixy.vector[i].x0;
					float dy = g_Pixy.vector[i].y1 - g_Pixy.vector[i].y0;
					float angle = atan2f(dx, dy) * (180.0f / M_PI);

//					printf("Vector %d angle: %.2f degrees\n", i, angle);

					// classify based on angle (adjust thresholds as needed)
					if (angle < -45.0f && angle > -135.0f) {  // Left branch
						left_index = i;
						left_error = abs(CAMERA_CENTER_X - g_Pixy.vector[i].x1);
					} else if (angle > 45.0f && angle < 135.0f) { // Right branch
						right_index = i;
						right_error = abs(CAMERA_CENTER_X - g_Pixy.vector[i].x1);
					} else if (angle > -45.0f && angle < 45.0f) { // Forward branch
						forward_index = i;
						forward_error = abs(CAMERA_CENTER_X - g_Pixy.vector[i].x1);
					}
					// the back branch (180 degrees) is ignored
				}

//				printf("Left branch (index %d): error = %d\n", leftIndex,
//						leftError);
//				printf("Right branch (index %d): error = %d\n", rightIndex,
//						rightError);
//				printf("Forward branch (index %d): error = %d\n", forwardIndex,
//						forwardError);

				// determine which direction has the smallest error
				if (left_index != -1 && left_error <= right_error
						&& left_error <= forward_error) {
					g_Pixy.move_command_state = Left;
					printf("Cross-junction decision: Left (smallest error)\n");
				} else if (right_index != -1 && right_error <= left_error
						&& right_error <= forward_error) {
					g_Pixy.move_command_state = Right;
					printf("Cross-junction decision: Right (smallest error)\n");
				} else if (forward_index != -1) {
					g_Pixy.move_command_state = Forward;
					printf(
							"Cross-junction decision: Forward (smallest error)\n");
				} else {
					// by default, just move forward
					g_Pixy.move_command_state = Forward;
					printf(
							"Couldn't make proper decision at cross junction, defaulting to Forward\n");
				}
			} else { // unexpected number of vectors at intersection
				g_Pixy.move_command_state = Forward;
				printf(
						"Unexpected number of vectors (%d) at intersection, defaulting to Forward\n",
						g_Pixy.num_of_vectors);
			}
		}

		// follow the line if no barcode or intersection!
		else if (!g_Pixy.barcode_detected && g_Pixy.num_of_vectors > 0) {
			int8_t xError = CAMERA_CENTER_X - g_Pixy.vector[0].x1;
			int8_t yError = CAMERA_CENTER_Y - g_Pixy.vector[0].y1;

			float angleDiff = 0.0f;
			if (xError != 0) { // cant divide by zero
				// currently angle diff is not really used
				angleDiff = atan2f((float) yError, (float) xError)
						* (180.0f / M_PI);
			}

//			printf(
//					"Line following: X Error: %d, Y Error: %d, Angle Difference: %.2f degrees\n",
//					xError, yError, angleDiff);

			// make decision based on error
			if (abs(xError) < THRESHOLD_X) {
				g_Pixy.move_command_state = Forward;
				printf("Line following decision: Forward\n");
			} else if (xError > THRESHOLD_X) {
				g_Pixy.move_command_state = Left;
				printf("Line following decision: Left\n");
			} else if (xError < -THRESHOLD_X) {
				g_Pixy.move_command_state = Right;
				printf("Line following decision: Right\n");
			}
		}

		// no valid features detected
		else if (!g_Pixy.barcode_detected && g_Pixy.num_of_vectors == 0) {
			printf("No line or barcode detected, stopping\n");
			g_Pixy.move_command_state = Stop;
		}

		// command has changed
		if (prevMoveCmd != g_Pixy.move_command_state) {
			printf("Move command changed from %d to %d\n", prevMoveCmd,
					g_Pixy.move_command_state);
		}
//	} else {
//		printf(
//				"Packet type != Get Main/All. Ignoring line following algorithm.\n");
//	}
}

void update_website_movement_state(int connection_id) {
	switch (g_Pixy.move_command_state) {
	case Forward:
		strcpy((char*) g_ESP.response, forward_response);
		break;
	case Left:
		strcpy((char*) g_ESP.response, left_response);
		break;
	case Right:
		strcpy((char*) g_ESP.response, right_response);
		break;
	case Stop:
		strcpy((char*) g_ESP.response, stop_response);
		break;
	}

	// possible race conditions
	sprintf(g_ESP.website_command, "AT+CIPSEND=%d,2\r\n", connection_id);
	g_UART1_tx.buffer_start = g_ESP.website_command;
	g_UART1_tx.data_size = strlen(g_ESP.website_command);
}
