/*
 * Filename:        motor.h
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  | Added structs and enums for motors, velocimetric
 * 		  wheels states and functions for handling user inputs
 * 		  and setting PWM duty cycles
 *
 * Version 2.0  | Added helper functions for starting, stopping,
 * 		  and updating of PWM duty cycles as well as resetting
 * 		  velocimetric parameters
 *
 * Version 3.0  | Added PD controller struct and Test Motor struct to
 * 		  obtain the gradient for the motors
 *
 * Version 4.0  | Included integral to fully implement PID controller
 * 		  as well as constants and macros for PID. Added
 * 		  more helper functions to reset struct properties.
 *
 * Description:
 * This file defines structs, constants, macros for handling user input
 * commands under TMOT mode to allow testing of motors. It also define
 * the necessary function prototypes to allow interfacing of motors
 * and pixy camera to allow the robot to follow the line locomotively.
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

/* START INCLUDES */
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "stdbool.h"
#include "stm32f303xe.h"
/* END INCLUDES */

/* START MACRO */
#define LEFT_MOTOR_INTERCEPT   0.0011f
#define LEFT_MOTOR_GRADIENT    0.0057f // can be used as kp coefficient
#define RIGHT_MOTOR_INTERCEPT  0.002f
#define RIGHT_MOTOR_GRADIENT   0.0064f // can be used as kp coefficient
#define PWM_SCALING_FACTOR  100

#define ARR_VALUE 3599
// feel free to adjust depending on your hardware constraints
#define STD_CCR_LEFT 500
#define STD_CCR_RIGHT 470

// PID properties - feel free to change depending on your hardware properties
// increase KP for faster response, reduce KP for reduction in oscillations
// ensure KI is not too high if there are oscillations
// if there is steady-state error, increase KI
#define KP 1.0f
#define KI 0.2f
#define KD 0.05f

// sampling time for pid, and left and right VCW wheels
// reduce for more accuracy
#define SAMPLE_TIME 500 // sample time to capture rise and fall counts to calculate velocity
#define PID_SAMPLE_TIME 250

// motor lengths for responses
#define MOTOR_RESPONSE_LENGTH 35
#define MOTOR_RESPONSE_LENGTH_V1 16
#define MOTOR_RESPONSE_LENGTH_V2 18
#define MOTOR_RESPONSE_LENGTH_V3 39
/* END MACRO */

/* START TYPEDEF */
typedef enum {
	MotorIdle, MotorDataReceived,
} MotorState_t;

typedef enum {
	Advancing, SteeringLeft, SteeringRight,
} MotorDirection_t;

typedef struct __Motor_struct {
	// for easier use of hal functions later
	TIM_HandleTypeDef *htim;
	uint8_t left_channel;
	uint8_t right_channel;
	HAL_StatusTypeDef left_hal_status;
	HAL_StatusTypeDef right_hal_status;

	// for motor pwm config
	uint16_t left_CCR;
	uint16_t right_CCR;

	// to be sent to hterm
	char response[50];
	// to be processed from hterm
	char command[30];

	// state logic
	MotorState_t state;
	bool motors_moving;
	MotorDirection_t direction;

	// for mvel (used to make the robot move relatively straight)
	bool mvel_flag;
	uint16_t input_value; // user input value
	uint32_t last_received_time;
	float scaled_input;
} Motor_t;

typedef struct __VelocimetricWheel_struct {
	// gpio properties
	const GPIO_TypeDef *port;
	const uint16_t pin;

	// triggered by gpio external interrupts
	uint32_t rise_count;
	uint32_t fall_count;

	// velocity derived using total counts / sample time
	float calculated_velocity;
	float scaled_velocity;
} VelocimetricWheel_t;

typedef struct __TestMotor_struct { // this struct is purely for obtaining the gradient
	uint16_t PWM_commands[35];
	float left_result[35];
	float right_result[35];
	float scaled_left_velocity_result[35];
	float scaled_right_velocity_result[35];
	uint32_t last_tested_time;
	TIM_HandleTypeDef *htim;
	uint32_t left_channel;
	uint32_t right_channel;
	uint16_t *PWM_command_ptr;
	bool left_motor_done;
	bool finish_testing_motors;
} TestMotor_t;

typedef struct __PIDController_struct {
	uint32_t last_received_time;
	float desired_velocity;
	float scaled_desired_velocity;

	// for proportional
	float error_left_motor;
	float error_right_motor;

	// for derivative
	float prev_error_left;
	float prev_error_right;
	float derivative_left;
	float derivative_right;
	float delta_time;

	// for integral
	float integral_left;
	float integral_right;

	// all the constants multiplied by the errors
	float output_left;
	float output_right;
} PIDController_t;

/* END TYPEDEF */

/* START FUNCTION PROTOTYPES */
void motor_control(Motor_t *motor, PIDController_t *pid,
		VelocimetricWheel_t *left, VelocimetricWheel_t *right);
void process_motor_data(Motor_t *motor, PIDController_t *pid);
// proportional, derivative and integral
void apply_pid(Motor_t *motor, PIDController_t *pid, VelocimetricWheel_t *left,
		VelocimetricWheel_t *right);
void process_motor_in_auto_mode(Motor_t *motor, Pixy_t *pixy);

// helper functions
void start_pwm(Motor_t *motor);
void stop_pwm(Motor_t *motor);
void update_pwm(Motor_t *motor);
void reset_vcw_values(VelocimetricWheel_t *left, VelocimetricWheel_t *right);
void reset_pid_properties(PIDController_t *pid);
void obtain_ccr_values(Motor_t *motor);
void obtain_graph_formula(TestMotor_t *test, Motor_t *motor,
		VelocimetricWheel_t *left, VelocimetricWheel_t *right);
/* END FUNCTION PROTOTYPES */

/* START EXTERN VARIABLES */
extern TIM_HandleTypeDef htim3;
/* END EXTERN VARIABLES */

#endif /* INC_MOTOR_H_ */
