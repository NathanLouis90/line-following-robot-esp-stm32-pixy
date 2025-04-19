/*
 * Filename:        callbacks.c
 * Author:          Muhammad Adiel Firqin Bin Muhamad Subta
 *
 * Version History:
 * ---------------------------------------------------------------------
 * Version 1.0  | Added GPIO External Interrupt, UART Error, UART TxCplt
 *		  and UART Rx Event Callbacks
 *
 * Version 2.0  | Added SPI Callbacks for Rx and Tx
 *
 * Version 3.0  | Added lines of code for velocimetric wheels for 
 *		  velocity calculations
 *
 * Description:
 * This file defines the weak callbacks if an interrupt occurs
 * e.g. UART peripheral receives data.
 */

#include "main.h"
#include "esp.h"
#include "pixy.h"
#include "motor.h"

extern BLINKER_TypeDef greenBlinker;

extern ESP_t g_ESP;
extern Pixy_t g_Pixy;
extern Motor_t g_Motor;
extern UART_Rx_t g_UART1_rx;
extern UART_Tx_t g_UART1_tx;
extern UART_Rx_t g_UART2_rx;
extern UART_Tx_t g_UART2_tx;
extern VelocimetricWheel_t g_left_VCW;
extern VelocimetricWheel_t g_right_VCW;

/* START GREEN LED & MOTOR CALLBACKS */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case Blue_B1_Pin:
		switch (greenBlinker.delayValue) {
		case LONG_DELAY:
			greenBlinker.delayValue = MEDIUM_DELAY;
			break;
		case MEDIUM_DELAY:
			greenBlinker.delayValue = SHORT_DELAY;
			break;
		default:
			greenBlinker.delayValue = LONG_DELAY;
			break;
		}
		printf("The blink duration is %lu milliseconds.\n",
				greenBlinker.delayValue);
		break;
	case Left_VCW_Pin:
		if (HAL_GPIO_ReadPin(g_left_VCW.port, g_left_VCW.pin)) {
//			printf("Left VCW Rising!\n");
			g_left_VCW.rise_count++;
		} else {
//			printf("Left VCW Falling!\n");
			g_left_VCW.fall_count++;
		}
		break;

	case Right_VCW_Pin:
		if (HAL_GPIO_ReadPin(Right_VCW_GPIO_Port, Right_VCW_Pin)) {
//			printf("Right VCW Rising!\n");
			g_right_VCW.rise_count++;
		} else {
//			printf("Right VCW Falling!\n");
			g_right_VCW.fall_count++;
		}
		break;
	}
}
/* END GREEN LED & MOTOR CALLBACKS */

/* START ESP CALLBACKS */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		g_UART2_rx.state = RxError;
	} else if (huart->Instance == USART1) {
		g_UART1_rx.state = RxError;
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		g_UART2_tx.state = TxIdle;
		g_UART2_tx.data_size = 0;
	} else if (huart->Instance == USART1) {
		g_UART1_tx.state = TxIdle;
		g_UART1_tx.data_size = 0;
		g_ESP.tx_success = true;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART2) {
		g_UART2_rx.data_size = Size;
		g_UART2_rx.state = RxDataReceived;
	} else if (huart->Instance == USART1) {
		g_UART1_rx.data_size = Size;
		g_UART1_rx.state = RxDataReceived;
	}
}
/* END ESP CALLBACKS */

/* START PIXY CALLBACKS */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == g_Pixy.hspi->Instance) {
		g_Pixy.tx_data_size = 0;

		// now collect the data received by pixy
        HAL_SPI_Receive_IT(g_Pixy.hspi, g_Pixy.rx_buffer, SPI_RECEIVE_SIZE);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == g_Pixy.hspi->Instance) {
		g_Pixy.state = PixyDataReceived;
	}
}

/* END PIXY CALLBACKS */
