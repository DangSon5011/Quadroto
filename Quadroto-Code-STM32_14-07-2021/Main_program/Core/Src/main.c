/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU.h"
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
axis trucx, trucy, trucz; 

extern  double U1, U2, U3, U4 , offset_nghiengX, offset_nghiengY;
double b = 0.00009, d = 0.000006484;
double dt=0.025;
double T = 0.05;

double PWM[4];
double pwm1, pwm2, pwm3, pwm4;

int dem;
char buffer[100] = {0};
double gocread[800];

/*PID controller variables*/
extern float gocdat_Roll, error_Roll, error_Roll_pre, pid_roll_P, pid_roll_I, pid_roll_D;
extern float gocdat_Pitch , error_Pitch,error_Pitch_pre, pid_pitch_P, pid_pitch_I, pid_pitch_D;
extern float gocdat_Yaw , error_Yaw,error_Yaw_pre, pid_yaw_P, pid_yaw_I, pid_yaw_D, yaw_PID;

float roll_PID =0, pitch_PID = 0,yaw_PID = 0;

float roll_pre = 0, pitch_pre = 0;

volatile unsigned char btn;
float y_temp = 0;
//             Roll 1-3 (4, 0.01, 35);        Pitch 2-4(2.05, 0.015, 28);          YAW( P:5.2,I:0.5, D: 0.5)
volatile float KP1 = 1.35, KD1 = 35, KI1 = 0.015, KP2 = 1.35 , KD2 = 35, KI2 = 0.015 , KP3 =  3, KD3=0,KI3 = 0; 
//volatile float KP1 = 0, KD1 = 0, KI1 = 0.0, KP2 = 0 , KD2 = 0, KI2 = 00.0, KP3 = 0 , KD3=0,KI3 = 0; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Calculation_SetPoint(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim4.Instance)
	{

		LMS303DLHC_getdata(&trucx, &trucy,&trucz);
		L3GD20_getdata(&trucx, &trucy,&trucz);
		
		/* Complementary filter */
		trucx.gocreal = 0.98*(trucx.gocreal + trucx.goc_gryo) + 0.02*trucx.goc_acel;
		trucy.gocreal = 0.98*(trucy.gocreal + trucy.goc_gryo) + 0.02*trucy.goc_acel;
	
		
		/* Low- pass filter*/
		trucx.gocreal = (trucx.gocreal * dt + T * pitch_pre) / (dt + T);
		
		pitch_pre = trucx.gocreal;
		
		trucy.gocreal = (trucy.gocreal * dt + T * roll_pre ) / (dt + T);
		y_temp = trucy.gocreal * -1;
		roll_pre = trucy.gocreal;
		
		/*Calculation yaw angle from roll and pitch after filter*/
		yaw_cal(&trucx, &trucy, &trucz);
		
		/* Calculate Set point*/
		Calculation_SetPoint();
		
		
		/************* Calculate Roll PID controller ********************/
		
	
		error_Roll = gocdat_Roll - trucx.gocreal; 	// error
		
		pid_roll_P  =  KP1 * error_Roll;            // P part
		
		if ( error_Roll > -30 && error_Roll < 30)  
			 { pid_roll_I += KI1 * error_Roll; }			// I part
			 
		pid_roll_D  = (float)KD1 * (error_Roll - error_Roll_pre); // D part
		error_Roll_pre = error_Roll;
		
		if (pid_roll_I  >  100)
			 { pid_roll_I  =  100;}
		
		if (pid_roll_I  < -100)
			 {pid_roll_I  = -100;}
			 	 
		roll_PID  = pid_roll_P + pid_roll_I + pid_roll_D;   // Total 
			 
		if ( roll_PID  < -400) 
		   { roll_PID  = -400;}
		
		if ( roll_PID  >  400) 
		   { roll_PID =   400;}
	
		/******************************************************************/
			
			 
		/************** Calculate Pitch PID controller ********************/
			 
		error_Pitch = gocdat_Pitch - y_temp -2.2;    // error
			 
		pid_pitch_P =  KP2 * error_Pitch;		// P part
			 		
		if ( error_Pitch > -30 && error_Pitch < 30)  // I Part
			 { pid_pitch_I += KI2 * error_Pitch; }	

		pid_pitch_D = (float)KD2 * (error_Pitch - error_Pitch_pre);	 // D part	 
		error_Pitch_pre = error_Pitch;
			 
		if (pid_pitch_I >  100) 
			 { pid_pitch_I =  100;}
		
		if (pid_pitch_I < -100)
			 { pid_pitch_I = -100;}
			 
		pitch_PID = pid_pitch_P + pid_pitch_I + pid_pitch_D;  // Total
		
		if ( pitch_PID < -400) 
			 { pitch_PID = -400;}
		
		if ( pitch_PID >  400)
			 { pitch_PID =  400;}
			  
		/******************************************************************/ 
			 
			 
		/************** Calculate Pitch PID controller ********************/
		
		error_Yaw = gocdat_Yaw - trucz.gocreal;
			
		pid_yaw_P   =  KP3* error_Yaw;  // P part
		pid_yaw_I  += KI3 * error_Yaw * dt; // I Part
		pid_yaw_D   = (float)KD3 * (error_Yaw - error_Yaw_pre)/dt; // D part
    error_Yaw_pre = error_Yaw;
			 
		if (pid_yaw_I   >  50) 
			 {pid_yaw_I   =  50;}
		
		if (pid_yaw_I   < -50)  	
			 {pid_yaw_I   = -50;}
			 	
		yaw_PID		= pid_yaw_P + pid_yaw_I + pid_yaw_D;  // Total
		
		if ( yaw_PID   < -400) 
		   { yaw_PID  = -400;}
		
		if ( yaw_PID   >  400) 
			 { yaw_PID =   400;}
			 
		/******************************************************************/ 
			 
		/* Calculate PWM output*/
		if(U1 > 1500) 
			U1 = 1500;

		PWM[0] = U1 + pitch_PID  - yaw_PID ;	 
		PWM[1] = U1 + roll_PID + yaw_PID;	
		PWM[2] = U1 - pitch_PID  - yaw_PID ;
		PWM[3] = U1 - roll_PID + yaw_PID;
			 

	 for (int i=0;i<4;i++)
		 {
	     if (PWM[i] > 2000)
				 PWM[i] = 2000;
			 else if (PWM[i] < 1000)
				 PWM[i] = 1000;	 
		 }
		

		pwm1 = PWM[0] + offset_nghiengX;    
		pwm2 = PWM[1] + offset_nghiengY;
		pwm3 = PWM[2] - offset_nghiengX;
		pwm4 = PWM[3] - offset_nghiengY;
	
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_1,(uint16_t)pwm1);
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_2,(uint16_t)pwm2);
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_3,(uint16_t)pwm3);
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_4,(uint16_t)pwm4);	
		 
	}		
	
	/* Truyen thong 50ms/1 time: tang offset de lay phan thap phan*/
	if(dem++ > 2)
	{
		int size;
		long tmp_gocdatRoll = (long)1000*gocdat_Roll;
		long tmp_gocrealRoll = (long)1000*(trucy.gocreal+45);
		
		long tmp_gocdatPitch = (long)1000* gocdat_Pitch;
		long tmp_gocrealPitch = (long)1000*(trucx.gocreal+45);
		
		long tmp_gocdatYaw = (long) 1000*gocdat_Yaw;
		long tmp_gocrealYaw = (long)1000*(trucz.gocreal+45);
		
		size = sprintf((char*)buffer,"A,%ld,%ld,%ld,%ld,%ld,%ld\n",
										tmp_gocdatRoll,tmp_gocrealRoll, tmp_gocdatPitch, tmp_gocrealPitch, tmp_gocdatYaw, tmp_gocrealYaw);
													
		HAL_UART_Transmit(&huart2,(uint8_t*)buffer,size,100);
		dem=0;
	} 
} 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/*Initialize sensor*/
	LMS303DLHC_Init();
  L3GD20_Init();
	HAL_Delay(10);
	
	/* Get data from sensor*/
	LMS303DLHC_getdata(&trucx, &trucy,&trucz);
  L3GD20_getdata(&trucx, &trucy,&trucz);

	
	/* Start receiver data from bluetooth*/
	HAL_UART_Receive(&huart1, &Button, 1, 100);
	
	/*Inititalize PWM channel*/
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	
	/*Set PWM min*/
//	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_1, 2000);
//	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_2, 2000);
//	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_3, 2000);
//	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_4, 2000);
//	HAL_Delay(3000);
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_2, 1000);
		__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_4, 1000);
	HAL_Delay (2000);
	gocdat_Pitch = gocdat_Roll = 0;
		gocdat_Yaw = 150;
		HAL_TIM_Base_Start_IT(&htim4);
		
		

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		UART_READ_DATA();
		
		//HAL_UART_Transmit_IT
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Calculation_SetPoint(void)
{
	switch(btn)
	{
//		case btn_tien_click_down: 
//			gocdat_Pitch += 0.025f;
//		  if (gocdat_Pitch > 15)
//				 {gocdat_Pitch = 15;}
//			break;
//				 
//		case btn_tien_click_up:
//			gocdat_Pitch -= 0.025f;
//			if (gocdat_Pitch < 0)
//			   {gocdat_Pitch = 0;}
//			break;
//				 
//		case btn_lui_click_down:
//			gocdat_Pitch -= 0.025f;
//			if (gocdat_Pitch < -15)
//			   {gocdat_Pitch = -15;}
//			break;
//			
//		case btn_lui_click_up:
//			gocdat_Pitch += 0.025f;
//		  if (gocdat_Pitch > 0)
//				 {gocdat_Pitch = 0;}
//			break;
//				 
//		case btn_trai_click_down:
//			gocdat_Roll += 0.025f;
//		  if (gocdat_Roll > 15)
//				 {gocdat_Roll = 15;}
//			break;
//				 
//		case btn_trai_click_up:
//			gocdat_Roll -= 0.025f;
//		  if (gocdat_Roll < 0)
//				 {gocdat_Roll = 0;}
//			break;
//				 
//		case btn_phai_click_down:
//			gocdat_Roll -= 0.025f;
//		  if (gocdat_Roll < -15)
//				 {gocdat_Roll = -15;}
//			break;
//		
//		case btn_phai_click_up:
//			gocdat_Roll += 0.025f;
//		  if (gocdat_Roll > 0)
//				 {gocdat_Roll = 0;}
//			break;
//		
//		case btn_stop:

//			
//			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_1, 1000);
//			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_2, 1000);
// 			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_3, 1000);
//			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_4, 1000);
//		HAL_TIM_Base_Stop_IT(&htim4);
//		case btn_start:
//			U1 = 1000;
//			pitch_PID = roll_PID = 0;
//			HAL_TIM_Base_Start_IT(&htim4);
//			break;  
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
