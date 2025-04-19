/*
 * Filename:        helpers.c
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  |
 *
 * Version 2.0  |
 *
 * Version 3.0  |
 *
 * Description:
 *
 */

#include "esp.h"
#include "pixy.h"
#include "motor.h"

extern ESP_t g_ESP;
extern Pixy_t g_Pixy;
extern Motor_t g_Motor;
extern UART_Rx_t g_UART1_rx;
extern UART_Tx_t g_UART1_tx;
extern UART_Rx_t g_UART2_rx;
extern UART_Tx_t g_UART2_tx;
extern VelocimetricWheel_t g_left_VCW;
extern VelocimetricWheel_t g_right_VCW;
extern TestMotor_t g_test_motors;

/* START ESP HELPERS */
void init_uart_rx_ptrs(UART_Rx_t *uart_rx) {
	uart_rx->buffer_base = uart_rx->buffer;
	uart_rx->buffer_start = uart_rx->buffer_base;
	uart_rx->buffer_end = uart_rx->buffer_start + MAX_BUFFER_SIZE;
	uart_rx->buffer_threshold = uart_rx->buffer + (MAX_BUFFER_SIZE * 3 / 4);
}

void init_uart_tx_ptrs(UART_Tx_t *uart_tx) {
	uart_tx->buffer_base = uart_tx->linked_rx->buffer;
	uart_tx->buffer_start = uart_tx->linked_rx->buffer_start;
	uart_tx->buffer_end = uart_tx->buffer_start + MAX_BUFFER_SIZE;
}

void link_uarts(UART_Rx_t *uart_rx, UART_Tx_t *uart_tx) {
	uart_rx->linked_tx = uart_tx;
	uart_tx->linked_rx = uart_rx;
}

void restart_reception(UART_Rx_t *uart_rx) {
	// Restart the reception
	start_uart_rx_to_idle(uart_rx);
	uart_rx->state = RxIdle;
}

void parse_staip_response(uint8_t *response) {
	// find the first double quote (starting of the IP)
	const char *ip_start = strchr(response, '"');
	if (ip_start != NULL) {
		ip_start++;  // Move past the first '"'

		// find the second double quote (end of the IP)
		const char *ipEnd = strchr(ip_start, '"');
		if (ipEnd != NULL) {
			// calculate the length of the IP address
			size_t ip_length = ipEnd - ip_start;
			if (ip_length < sizeof(g_ESP.ip_address)) {
				// copy only the IP address into staIP
				strncpy(g_ESP.ip_address, ip_start, ip_length);
				g_ESP.ip_address[ip_length] = '\0';
			}
		}
	}
}

void send_command_to_esp(uint8_t command_index, UART_Tx_t *uart_tx,
		UART_Rx_t *uart_rx) {

	if (command_index
			< sizeof(g_ESP.at_commands) / sizeof(g_ESP.at_commands[0])) {
		const char *command = g_ESP.at_commands[command_index]; // Get the command

		printf("Command sent: %s\n", command);

		uart_tx->buffer_start = (uint8_t*) g_ESP.at_commands[command_index];
		uart_tx->data_size = strlen(g_ESP.at_commands[command_index]);

		g_ESP.response_position = uart_rx->buffer_start;

	} else {
		// Handle invalid command index
		printf("Invalid command index\n");
	}
}

void toggle_led_and_respond(uint8_t connection_id, GPIO_TypeDef *gpio,
		uint16_t pin, const char *off_response, const char *on_response) {
	// toggle led
	HAL_GPIO_TogglePin(gpio, pin);

	// prepare cipsend command
	sprintf(g_ESP.website_command, "AT+CIPSEND=%d,2\r\n", connection_id);
	g_UART1_tx.buffer_start = (uint8_t*) g_ESP.website_command;
	g_UART1_tx.data_size = strlen(g_ESP.website_command);

	// store rg_ESPonse for later sending
	if (HAL_GPIO_ReadPin(gpio, pin) == GPIO_PIN_SET) {
		strcpy((char*) g_ESP.response, on_response);
	} else {
		strcpy((char*) g_ESP.response, off_response);
	}

	// mark this part of buffer as processed
	g_UART1_rx.data_size = 0;
	g_ESP.html_state = RightArrow;
}

uint8_t extract_connection_id(const char *buffer) {
	char *connectionStr = strstr(buffer, "+IPD,");
	if (connectionStr) {
		return connectionStr[5] - '0';  // extract the connection ID
	}
	return 0; // Default connection ID
}
/* END ESP HELPERS */

/* START PIXY HELPERS */

void init_pixy_struct(Pixy_t *pixy) {
	pixy->rx_buffer_base = pixy->rx_buffer;
	pixy->tx_ptr = NULL;
	pixy->tx_data_size = 0;

	// for states
	pixy->state = PixyIdle;
	pixy->move_command_state = Stop; // stop at the start

	// reset detection flags
	pixy->barcode_detected = false;
	pixy->barcode.value = BarcodeNotDetected;

	// reset vector counts
	pixy->num_of_vectors = 0;
	pixy->num_of_branches = 0;
}

void reset_vector_contents(Pixy_t *pixy) {
	pixy->num_of_vectors = 0;
	pixy->num_of_branches = 0;

	for (int i = 0; i < 4; i++) {
		pixy->vector[i].x0 = 0;
		pixy->vector[i].y0 = 0;
		pixy->vector[i].x1 = 0;
		pixy->vector[i].y1 = 0;
		pixy->vector[i].line_index = 0;
		pixy->vector[i].flag = 0;
	}
}

void reset_barcode_detection(Pixy_t *pixy) {
	pixy->barcode_detected = false;
	pixy->barcode.value = BarcodeNotDetected;
	pixy->barcode.x = 0;
	pixy->barcode.y = 0;
	pixy->barcode.flag = 0;
}
/* END PIXY HELPERS */

/* START MOTOR HELPERS */
void update_pwm(Motor_t *motor) {
	// prevent ccr from exceeding more than arr value
	if (motor->left_CCR > ARR_VALUE) {
		motor->left_CCR = ARR_VALUE;
	}

	if (motor->right_CCR > ARR_VALUE) {
		motor->right_CCR = ARR_VALUE;
	}

	// update ccr value
	TIM3->CCR1 = motor->left_CCR;
	TIM3->CCR3 = motor->right_CCR;
}

void start_pwm(Motor_t *motor) {
	motor->left_hal_status = HAL_TIM_PWM_Start(motor->htim,
			motor->left_channel);
	motor->right_hal_status = HAL_TIM_PWM_Start(motor->htim,
			motor->right_channel);
}

void stop_pwm(Motor_t *motor) {
	motor->left_hal_status = HAL_TIM_PWM_Stop(motor->htim, motor->left_channel);
	motor->right_hal_status = HAL_TIM_PWM_Stop(motor->htim,
			motor->right_channel);
}

void reset_vcw_values(VelocimetricWheel_t *left, VelocimetricWheel_t *right) {
	left->fall_count = 0;
	left->rise_count = 0;

	right->fall_count = 0;
	right->rise_count = 0;
}

void obtain_ccr_values(Motor_t *motor) {
	// the formula obtained is plotted on an excel, extrapolated and using excel function
	// to derive the y = mx + c equation
	motor->left_CCR = (uint16_t) ((motor->scaled_input / LEFT_MOTOR_GRADIENT)
			+ LEFT_MOTOR_INTERCEPT) * PWM_SCALING_FACTOR;

	motor->right_CCR = (uint16_t) ((motor->scaled_input / RIGHT_MOTOR_GRADIENT)
			+ RIGHT_MOTOR_INTERCEPT) * PWM_SCALING_FACTOR;
}

float obtain_desired_velocity(uint16_t target_ccr, bool is_left_motor) {
	float scaled_input;

	if (is_left_motor) {
		// Left motor inverse formula: scaled_input = ((CCR + 25) / 790.5)² / 8 + 0.343/8
		float temp = (float) (target_ccr + 25) / 790.5f;
		scaled_input = (temp * temp) / 8.0f + 0.343f / 8.0f;
	} else {
		// Right motor inverse formula: scaled_input = ((CCR + 75) / 790.5)² / 8 + 0.375/8
		float temp = (float) (target_ccr + 75) / 790.5f;
		scaled_input = (temp * temp) / 8.0f + 0.375f / 8.0f;
	}

	return scaled_input;
}

// for obtaining gradient
void obtain_graph_formula(TestMotor_t *test, Motor_t *motor,
		VelocimetricWheel_t *left, VelocimetricWheel_t *right) {
	if (test->finish_testing_motors) {
		return; // testing already completed
	}

	uint16_t numCommands = sizeof(test->PWM_commands)
			/ sizeof(test->PWM_commands[0]);

	if (!test->left_motor_done) {
		// testing left motor
		uint16_t current_index = test->PWM_command_ptr - test->PWM_commands;

		if (current_index < numCommands) {
			if ((uwTick - test->last_tested_time) >= SAMPLE_TIME) {
				// apply PWM to left motor
				uint16_t pwm = test->PWM_commands[current_index];
				__HAL_TIM_SET_COMPARE(test->htim, test->left_channel, pwm);
				// stop right motor during left test
				__HAL_TIM_SET_COMPARE(test->htim, test->right_channel, 0);

				// calculate velocity
				float left_total_count = left->fall_count + left->rise_count;
				left->calculated_velocity = left_total_count
						/ (float) SAMPLE_TIME;
				test->left_result[current_index] = left->calculated_velocity;
				test->scaled_left_velocity_result[current_index] =
						left->calculated_velocity * (float) PWM_SCALING_FACTOR;

				// move to next command
				test->PWM_command_ptr++;
				test->last_tested_time = uwTick;
			}
		} else {
			// left motor tests done
			test->left_motor_done = true;
			test->PWM_command_ptr = test->PWM_commands; // Reset pointer for right motor
			test->last_tested_time = uwTick;
		}
	} else {
		// testing right motor
		uint16_t current_index = test->PWM_command_ptr - test->PWM_commands;

		if (current_index < numCommands) {
			if ((uwTick - test->last_tested_time) >= SAMPLE_TIME) {
				// apply PWM to right motor
				uint16_t pwm = test->PWM_commands[current_index];
				__HAL_TIM_SET_COMPARE(test->htim, test->right_channel, pwm);
				// stop left motor during right test
				__HAL_TIM_SET_COMPARE(test->htim, test->left_channel, 0);

				// calculate velocity
				float right_total_count = right->fall_count + right->rise_count;
				right->calculated_velocity = right_total_count
						/ (float) SAMPLE_TIME;
				test->right_result[current_index] = right->calculated_velocity;
				test->scaled_right_velocity_result[current_index] =
						right->calculated_velocity * (float) PWM_SCALING_FACTOR;

				test->PWM_command_ptr++;
				test->last_tested_time = uwTick;
			}
		} else {
			// all tests done
			test->finish_testing_motors = true;
			// stop both motors
			__HAL_TIM_SET_COMPARE(test->htim, test->left_channel, 0);
			__HAL_TIM_SET_COMPARE(test->htim, test->right_channel, 0);
		}
	}
}

void reset_pid_properties(PIDController_t *pid) {
	// proportional properties
	pid->error_left_motor = 0;
	pid->error_right_motor = 0;

	// derivative properties
	pid->derivative_left = 0;
	pid->derivative_right = 0;
	pid->prev_error_left = 0;
	pid->prev_error_right = 0;

	// integral properties
	pid->integral_left = 0;
	pid->integral_right = 0;
}
/* END MOTOR HELPERS */
