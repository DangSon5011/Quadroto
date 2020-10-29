
#include "main.h"
#include <math.h>

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
//static void MX_TIM4_Init(void);
double Kalman(double newAngle, double newRate, double dt);

void L3GD20_Init(void);
void getdata_L3GD20(void);

double gocrate[3];//rate goc chuyen doi

void LMS303DLHC_getdata(void);
void LMS303DLHC_Init(void);
uint16_t DevAddress=0x32;

double ax,ay,az,gocx,gocy;
double RAD_TO_DO =57.2958;

double Q_angle = 0.001;; // Process noise variance for the accelerometer
double Q_bias = 0.003;; // Process noise variance for the gyro bias
double R_measure = 0.6; // Measurement noise variance - this is actually the variance of the measurement noise
double angle ,	anglex ; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
double bias = 0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    
double P[2][2]={{0,0},{0,0}}; // Error covariance matrix - This is a 2x2 matrix

double K[2]; // Kalman gain - This is a 2x1 matrix   | 1|
double y; // Angle difference - 1x1 matrix           | 2|
double S; // Estimate error




int main(void)
{
 
  HAL_Init();

  
  SystemClock_Config();

  
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
//  MX_TIM4_Init();
  HAL_Delay(100);
//	HAL_TIM_Base_Start_IT(&htim4);
	L3GD20_Init();
	LMS303DLHC_Init();
  while (1)
  {
		LMS303DLHC_getdata();
		getdata_L3GD20();
		HAL_Delay(10);
  }

}


void LMS303DLHC_Init(void)
	{ 
		// ham mac dinh khoi tao cho hi2c1, neu dùng ngoai vi i2c khac can thay the
		// DevAddress la bien toan cuc can khai bao truoc khi su dung
		uint8_t buffer_tx[2];
		for(int i=0;i<200;i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1,DevAddress,2,10)== HAL_OK)
			{
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
				break;
			}
	}
	//cau hinh che do lam viec
	buffer_tx[0]=0x20;
	buffer_tx[1]=0x57;// toc do  lay mau 100hz
  HAL_I2C_Master_Transmit(&hi2c1,DevAddress,buffer_tx,2,100);
		
	}
void LMS303DLHC_getdata(void)
	{  
		// ax,ay,az,gocx,gocy là bien toan cuc can phai duoc khai bao truoc khi su dung ham 
		// DevAddress la bien toan cuc can khai bao truoc khi su dung
		uint8_t buffer_tx[2];
		uint8_t buffer_rx[6];
		int16_t giatocx,giatocy,giatocz;
		buffer_tx[0]=0xA8;
		HAL_I2C_Master_Transmit(&hi2c1,DevAddress,buffer_tx,1,100);
		HAL_I2C_Master_Receive(&hi2c1,DevAddress,buffer_rx,6,100);
		giatocx=(buffer_rx[1]<<8)|buffer_rx[0];
		giatocy=(buffer_rx[3]<<8)|buffer_rx[2];
		giatocz=(buffer_rx[5]<<8)|buffer_rx[4];
		ax=((double)giatocx/1670.132 );
		ay=((double)giatocy/1670.132);
		az=((double)giatocz/1670.132)+0.3;
		gocx=(RAD_TO_DO*atan(ay/az));
		gocy=(RAD_TO_DO*atan(-ax/sqrt(ay*ay+az*az)));
		}
	
void L3GD20_Init(void)
	{  
		uint8_t buffer_tx[2];//dem truyen
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer_tx[0]=0x20;
		buffer_tx[1]=0x1f;// che do binh thuong, ODA=95hz, loc thong thap 32hz
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_Delay(10);
	//loc
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer_tx[0]=0x24;
		buffer_tx[1]=0x12;// , loc thong cao, thong thao lan 2
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_Delay(10);
	
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer_tx[0]=0x21;
		buffer_tx[1]=0x07;// , loc thong cao 0.05hz
		HAL_SPI_Transmit(&hspi1,buffer_tx,2,100);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	}
	
	
	
void getdata_L3GD20(void)
	{ 
		// gocrate[3] la bien toan cuc can khai bao truoc khi su dung
		uint8_t buffer1_tx[2];//dem truyen
		uint8_t buffer1_rx[6];//dem nhan
		int16_t gocx,gocy,gocz;//rate goc
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		buffer1_tx[0]=0x28|0xc0;// 0xC0 yeu cau doc nhieu byte
   
		HAL_SPI_Transmit(&hspi1,buffer1_tx,1,40);// gui dia chi thanh gi chua rate goc
		HAL_SPI_Receive(&hspi1,buffer1_rx,6,40);// doc rate goc
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		
		gocx=(buffer1_rx[1]<<8)|buffer1_rx[0];
		gocrate[0]=((double)gocx/32767)*250;//toc do goc truc x
		gocy=(buffer1_rx[3]<<8)|buffer1_rx[2];
		gocrate[1]=((double)gocy/32767)*250;// toc do goc truc y
		gocz=(buffer1_rx[5]<<8)|buffer1_rx[4];
		gocrate[2]=((double)gocz/32767)*250;//toc do goc truc z
	
		}
	double Kalman(double newAngle, double newRate, double dt)
	{
		    rate = newRate - bias;
        angle += dt * rate;
        
        // Update estimation error covariance - Project the error covariance ahead
        /* Step 2 */
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        /* Step 4 */
        S = P[0][0] + R_measure;   //(Pk-) +R
        /* Step 5 */
        K[0] = P[0][0] / S;        // K[0] = Kk = (Pk-)/ [(Pk-) +R]
        K[1] = P[1][0] / S;
        
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        /* Step 3 */
        y = newAngle - angle;      // Zk - (Xk-)  = y
        /* Step 6 */
        angle += K[0] * y;         // Xk = (Xk-) + Kk * [Zk - (Xk-)]
        bias += K[1] * y;
        
        // Calculate estimation error covariance - Update the error covariance
        /* Step 7 */
        P[0][0] -= K[0] * P[0][0];   // Pk.new = (Pk-) - Kk*(Pk-)
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];
        
        return angle;
	}
//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
//	{ 
//		if( htim->Instance== htim4.Instance)
//			{
//		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
//		 Kalman(gocx,gocrate[0],0.01);
//		}
//	}
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM4_Init(void)
//{

//  /* USER CODE BEGIN TIM4_Init 0 */

//  /* USER CODE END TIM4_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  /* USER CODE BEGIN TIM4_Init 1 */

//  /* USER CODE END TIM4_Init 1 */
//  htim4.Instance = TIM4;
//  htim4.Init.Prescaler = 1000;
//  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim4.Init.Period = 499;
//  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM4_Init 2 */

//  /* USER CODE END TIM4_Init 2 */

//}

///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
