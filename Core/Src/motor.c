/*
 * Filename:        motor.c
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  | Definition of basic structs for motors, velocimetric
 * 		  wheels, as well as simple state machine logic for
 * 		  user input commands for motor
 *
 * Version 2.0  | Proper definition of processing of user inputs
 * 		  for motor commands
 *
 * Version 3.0  | Implemented PD logic to allow both motors to
 * 		  rotate relatively evenly
 *
 * Version 4.0  | Inclusion of integral term to define PID logic as
 * 		  well as proper definition of motors operating
 * 		  under AUTO mode
 *
 * Description:
 * This file contains motor control logic for the robot car. It defines
 * several functionalities such as processing of user inputs during
 * TMOT mode which allows control of motors, application of PID
 * control to allow the robot to move relatively forward as well as
 * allowing the robot to move according to what the pixy camera sees
 * during AUTO mode.
 *
 */

#include "pixy.h"
#include "esp.h"
#include "motor.h"
#include "main.h"
#include <math.h>

// view the main file to get accurate CCR values
Motor_t g_Motor = { .htim = &htim3, .left_CCR = 500, .right_CCR = 700,
		.left_channel = TIM_CHANNEL_1, .right_channel = TIM_CHANNEL_3,
		.motors_moving = false, .response = { 0 }, .state = MotorIdle,
		.command = { 0 }, .mvel_flag = false, .last_received_time = 0,
		.input_value = 0, .scaled_input = 0 };

VelocimetricWheel_t g_left_VCW = { .port = Left_VCW_GPIO_Port, .pin =
Left_VCW_Pin, .fall_count = 0, .rise_count = 0, };

VelocimetricWheel_t g_right_VCW = { .port = Right_VCW_GPIO_Port, .pin =
Right_VCW_Pin, .fall_count = 0, .rise_count = 0, };

TestMotor_t g_test_motors = { .PWM_commands = { 100, 200, 300, 400, 500, 600,
		700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800,
		1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900, 3000,
		3100, 3200, 3300, 3400, 3500 }, .htim = &htim3, .left_channel =
TIM_CHANNEL_1, .right_channel = TIM_CHANNEL_3, .left_motor_done = false,
		.finish_testing_motors = false, .left_result = { 0 }, .right_result = {
				0 }, .last_tested_time = 0,
		.scaled_left_velocity_result = { 0 }, .scaled_right_velocity_result = {
				0 }, };

PIDController_t g_PID_controller = { .desired_velocity = 0,
		.scaled_desired_velocity = 0, .error_left_motor = 0,
		.error_right_motor = 0, .last_received_time = 0, .prev_error_left = 0,
		.prev_error_right = 0, .delta_time = PID_SAMPLE_TIME / 1000.0f,
		.derivative_left = 0, .derivative_right = 0, .integral_left = 0,
		.integral_right = 0, .output_left = 0, .output_right = 0, };

extern ESP_t g_ESP;
extern Pixy_t g_Pixy;
extern UART_Rx_t g_UART1_rx;
extern UART_Tx_t g_UART1_tx;
extern UART_Rx_t g_UART2_rx;
extern UART_Tx_t g_UART2_tx;

void motor_control(Motor_t *motor, PIDController_t *pid,
		VelocimetricWheel_t *left, VelocimetricWheel_t *right) {
	switch (motor->state) {
	case MotorIdle:
		break;
	case MotorDataReceived:
		process_motor_data(motor, pid);
		motor->state = MotorIdle;
		break;
	}

	if (motor->motors_moving) {
		if (uwTick - motor->last_received_time >= SAMPLE_TIME) {
			uint16_t left_total_count = left->fall_count + left->rise_count;
			uint16_t right_total_count = right->fall_count + right->rise_count;

			// calculate velocity
			left->calculated_velocity = (float) left_total_count
					/ (float) SAMPLE_TIME;
			right->calculated_velocity = (float) right_total_count
					/ (float) SAMPLE_TIME;

			// scale the velocity
			left->scaled_velocity = left->calculated_velocity
					* PWM_SCALING_FACTOR;
			right->scaled_velocity = right->calculated_velocity
					* PWM_SCALING_FACTOR;

			// reset vcw counts
			reset_vcw_values(left, right);

			motor->last_received_time = uwTick;
		}
		if (motor->mvel_flag) {
			if (uwTick - pid->last_received_time >= PID_SAMPLE_TIME) {
				apply_pid(motor, pid, left, right);
				update_pwm(motor);

				pid->last_received_time = uwTick;
			}
		}
	}

	if (g_ESP.website_fully_setup && g_ESP.system_mode == AutoMode) {
		process_motor_in_auto_mode(motor, &g_Pixy);
	}
}

void process_motor_data(Motor_t *motor, PIDController_t *pid) {
	if (motor->command != NULL) {
		// clear the contents of response buffer
		memset(motor->response, 0, sizeof(motor->response));

		// start pwm generation - motor start moving
		if (strstr(motor->command, "at+start") != NULL) {
			printf("Starting motors!\n");
			start_pwm(motor);
			snprintf(motor->response, MOTOR_RESPONSE_LENGTH,
					"Left: %u\r\n, Right: %u\r\n", motor->left_CCR,
					motor->right_CCR);
			motor->motors_moving = true;

			motor->last_received_time = uwTick; // start counting to get velocity

			if (motor->mvel_flag) {
				pid->last_received_time = uwTick;
			}
		}
		// stop pwm generation - motor stop moving
		else if (strstr(motor->command, "at+stop") != NULL) {
			if (motor->motors_moving) {
				printf("Stopping motors!\n");
				stop_pwm(motor);
				strcpy(motor->response, "All Stopped\r\n");

				// reset vcw value and toggle the flag
				reset_vcw_values(&g_left_VCW, &g_right_VCW);
				// reset pid properties
				reset_pid_properties(pid);

				motor->motors_moving = false;
			} else {
				printf("Motors already stopped!\n");
				strcpy(motor->response, "Motors already stopped\r\n");
			}
		}
		// cancel mvel flag
		else if (strstr(motor->command, "at+cancel") != NULL) {
			if (motor->mvel_flag) {
				reset_vcw_values(&g_left_VCW, &g_right_VCW);
				reset_pid_properties(pid);
				motor->mvel_flag = false;
				strcpy(motor->response, "Stopping mvel\r\n");
			} else {
				strcpy(motor->response, "Mvel already disabled\r\n");
			}
		}
		// command to set the desired velocity so that the robot can move relatively forward
		else if (sscanf(motor->command, "at+mvel=%u", &motor->input_value)
				== 1) {
			motor->scaled_input = (float) motor->input_value
					/ (float) PWM_SCALING_FACTOR;
			obtain_ccr_values(motor);
			motor->mvel_flag = true;

			// capture the values for pid controller
			pid->desired_velocity = motor->input_value;
			pid->scaled_desired_velocity = motor->scaled_input;
			reset_pid_properties(pid);

			snprintf(motor->response, MOTOR_RESPONSE_LENGTH_V3,
					"Setting left to %u and right to %u\r\n", motor->left_CCR,
					motor->right_CCR);
		}
		// set the left CCR
		else if (sscanf(motor->command, "at+m1a=%u", &motor->left_CCR) == 1) {
			printf("Changing left motors\n");
			char response_temp[MOTOR_RESPONSE_LENGTH_V1];
			snprintf(response_temp, MOTOR_RESPONSE_LENGTH_V1, "Left: %u\r\n",
					motor->left_CCR);
			strcpy(motor->response, response_temp);

			// check for right motor command
			char *right_cmd = strstr(motor->command, "at+m2a=");
			if (right_cmd != NULL) {
				if (sscanf(right_cmd, "at+m2a=%u", &motor->right_CCR) == 1) {
					printf("Changing right motors too\n");
					// append right motor response
					snprintf(response_temp, MOTOR_RESPONSE_LENGTH_V1,
							"Right: %u\r\n", motor->right_CCR);
					strcat(motor->response, response_temp);
				}
			}
		}
		// set the right CCR
		else if (sscanf(motor->command, "at+m2a=%u", &motor->right_CCR) == 1) {
			printf("Changing right motors\n");
			snprintf(motor->response, MOTOR_RESPONSE_LENGTH_V1, "Right: %u\r\n",
					motor->right_CCR);
		}
		// print out left and right CCR values
		else if (strstr(motor->command, "at") != NULL) {
			snprintf(motor->response, MOTOR_RESPONSE_LENGTH,
					"Left: %u\r\n, Right: %u\r\n", motor->left_CCR,
					motor->right_CCR);
		}
		update_pwm(motor);

		// start to transmit data via uart2 tx
		g_UART2_tx.buffer_start = motor->response;
		g_UART2_tx.data_size = strlen(motor->response);
	} else {
		printf("Command buffer contains no data!\n");
	}
}

void apply_pid(Motor_t *motor, PIDController_t *pid, VelocimetricWheel_t *left,
		VelocimetricWheel_t *right) {
	// if error_left is less than 0 - motor needs to slow down
	// else - motor nees to speed up

	// calculate current errors (desired - actual)
	pid->error_left_motor = pid->scaled_desired_velocity
			- left->calculated_velocity;
	pid->error_right_motor = pid->scaled_desired_velocity
			- right->calculated_velocity;

	// calculate derivative terms
	pid->derivative_left = (pid->error_left_motor - pid->prev_error_left)
			/ pid->delta_time;
	pid->derivative_right = (pid->error_right_motor - pid->prev_error_right)
			/ pid->delta_time;

	// calculate integral terms with anti-windup
	pid->integral_left += pid->error_left_motor * pid->delta_time;
	pid->integral_right += pid->error_right_motor * pid->delta_time;

	// limit integral to prevent windup
	pid->integral_left =
			(pid->integral_left > 1000.0f) ? 1000.0f :
			(pid->integral_left < -1000.0f) ? -1000.0f : pid->integral_left;
	pid->integral_right =
			(pid->integral_right > 1000.0f) ? 1000.0f :
			(pid->integral_right < -1000.0f) ? -1000.0f : pid->integral_right;

	// calculate total output
	pid->output_left = (KP * pid->error_left_motor) + (KI * pid->integral_left)
			+ (KD * pid->derivative_left);
	pid->output_right = (KP * pid->error_right_motor)
			+ (KI * pid->integral_right) + (KD * pid->derivative_right);

	printf("Left output: %.5f, Right output: %.5f\n", pid->output_left,
			pid->output_right);

	// convert PID output to CCR adjustment - add to current CCR value
	int16_t left_adjustment = (int16_t) (pid->output_left * PWM_SCALING_FACTOR);
	int16_t right_adjustment =
			(int16_t) (pid->output_right * PWM_SCALING_FACTOR);

	// apply the adjustments
	int32_t new_left_ccr = (int32_t) (motor->left_CCR + left_adjustment);
	int32_t new_right_ccr = (int32_t) (motor->right_CCR + right_adjustment);

	// change the CCR values
	motor->left_CCR = (uint16_t) new_left_ccr;
	motor->right_CCR = (uint16_t) new_right_ccr;

	// update previous error for next PID calculation
	pid->prev_error_left = pid->error_left_motor;
	pid->prev_error_right = pid->error_right_motor;

//    printf("Adjusted Left CCR: %u, Right CCR: %u\n", motor->left_CCR,
//            motor->right_CCR);
}

void process_motor_in_auto_mode(Motor_t *motor, Pixy_t *pixy) {
	// if you wish to set mvel, remember to set it to false for all non-forward movement
	if (!motor->motors_moving) {
		motor->motors_moving = true;
		// set default speed - forward
		motor->left_CCR = STD_CCR_LEFT;
		motor->right_CCR = STD_CCR_RIGHT;
		/* set mvel flag here - use mvel to control the forward movement */
		start_pwm(motor);
	}

	// depending on what pixy sees - set corresponding CCR value
	switch (pixy->move_command_state) {
	case Idle:
		break;
	case Forward:
		motor->left_CCR = STD_CCR_LEFT;
		motor->right_CCR = STD_CCR_RIGHT;
		/* set mvel flag here - use mvel to control the forward movement */
		break;
	case Left:
		// robot should halt left motors and move right
		motor->left_CCR = 0;
		motor->right_CCR = STD_CCR_RIGHT;
		break;
	case Right:
		motor->left_CCR = STD_CCR_LEFT;
		motor->right_CCR = 0;
		break;
	case Stop:
		motor->left_CCR = 0;
		motor->right_CCR = 0;
		break;
	default:
		break;
	}

	update_pwm(motor);
}
