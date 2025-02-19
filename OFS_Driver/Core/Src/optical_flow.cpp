/*
 * optical_flow.c
 *
 *  Created on: Jan 30, 2023
 *      Author: hdoug
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "optical_flow.h"
#include "MSP_format.h"
#include "main.h"
#include <stdio.h>

uint8_t rx_data[18];
UART_HandleTypeDef huart2;


static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

// read data from sensor
void read_flow_data(){
	MX_USART2_UART_Init();
	HAL_UART_Receive_IT(&huart2, rx_data, 18);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	process_opflow(rx_data);
	read_flow_data();
}

void process_opflow(uint8_t *rx_data){
	printf("Values are:\n");
	for(int i=0;i<18;i++) {
	  printf("%X ", rx_data[i]);
	}
	msp_format_t msp_data;
	msp_parse_received_data(&msp_data, rx_data);

	if(msp_data.c_state == MSP_COMMAND_RECEIVED){
		opFlowUpdate(&msp_data);
	}
}

// INITIALIZATION

// CALIBRATION

// UPDATE FUNCTION
void opFlowUpdate(msp_format_t *raw_data)
{
	// need timing?
	raw_opflow_data_t *pkt = (raw_opflow_data_t *)raw_data->payload;
	printf("%X", pkt->quality);
	// display data on screen?
	// sets global state variable...
}
