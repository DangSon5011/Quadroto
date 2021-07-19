 /**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#define BUTTON_PD   255
#define DATA_XYZ 2
#define UPDOWN 15
#define BUTTON_START        55
#define BUTTON_STOP         66
#define BUTTON_HEAD_SET     13
#define BUTTON_HEAD_RESET   14
#define BUTTON_BACK_SET     15
#define BUTTON_BACK_RESET   16
#define BUTTON_LEFT_SET     17
#define BUTTON_LEFT_RESET   18
#define BUTTON_RIGHT_SET    19
#define BUTTON_RIGHT_RESET  20
#define BUTTON_UP_SET       21
#define BUTTON_UP_RESET     22
#define BUTTON_DOWN_SET     23
#define BUTTON_DOWN_RESET   24



unsigned char Button;
double U1, U2, U3, U4, offset_nghiengX, offset_nghiengY;

float gocdat_Roll, error_Roll, error_Roll_pre, pid_roll_P, pid_roll_I, pid_roll_D;
float gocdat_Pitch , error_Pitch,error_Pitch_pre, pid_pitch_P, pid_pitch_I, pid_pitch_D;
float gocdat_Yaw , error_Yaw,error_Yaw_pre, pid_yaw_P, pid_yaw_I, pid_yaw_D;

TIM_HandleTypeDef htim4;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void UART_READ_DATA(void)
{
	HAL_UART_Receive(&huart2,&Button,1,100);
	switch (Button)
	{
		case BUTTON_PD:
				HAL_TIM_Base_Stop_IT(&htim4);
				UART_READ_PD();
				HAL_TIM_Base_Start_IT(&htim4);
		 		break;
		case BUTTON_HEAD_SET:
				btn = btn_tien_click_down;
				offset_nghiengX = 2;
				break;
		case BUTTON_HEAD_RESET:
				btn = btn_tien_click_up;
				offset_nghiengX = 0;
				break;
		case BUTTON_BACK_SET:
				btn = btn_lui_click_down;
				offset_nghiengX = -2;
				break;
		case BUTTON_BACK_RESET:
				btn = btn_lui_click_up;
				offset_nghiengX = 0;
				break;
		case BUTTON_LEFT_SET:
				btn = btn_trai_click_down;
				offset_nghiengY = 2;
				break;
		case BUTTON_LEFT_RESET:
				btn = btn_trai_click_up;
				offset_nghiengY = 0;
				break;
		case BUTTON_RIGHT_SET:
				btn = btn_phai_click_down;
				offset_nghiengY = -2;
				break;
		case BUTTON_RIGHT_RESET:
				btn =  btn_phai_click_up;
				offset_nghiengY = 0;
				break;
		case BUTTON_UP_SET:
				U1 += UPDOWN;
				break;
		case BUTTON_UP_RESET:
				break;
		case BUTTON_DOWN_SET:
				U1 -= UPDOWN;
				break;
		case BUTTON_DOWN_RESET:
				break;
		case BUTTON_START:
				
				btn = btn_start;
				pid_pitch_I = pid_roll_I = pid_yaw_I = 0;
				U1 = 1000;
				break;
		case BUTTON_STOP:
		    btn = btn_stop;
				U1 = 0;
			
				break;
	}
	
}
void UART_READ_PD(void)
{
//	uint8_t buffer[12];
//	HAL_UART_Receive(&huart1,(uint8_t*)buffer,12,1000);
//	KP1 = buffer[0] + (float)buffer[1]/100;
//	KD1 = buffer[2] + (float)buffer[3]/100;
//	KP2 = buffer[4] + (float)buffer[5]/100;
//	KD2 = buffer[6] + (float)buffer[7]/100;
//	KP3 = buffer[8] + (float)buffer[9]/100;
//	KD3 = buffer[10] + (float)buffer[11]/100;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
