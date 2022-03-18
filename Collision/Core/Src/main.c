/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include <sys/mman.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
extern enum IO; // { IN, OUT };
extern void config_gpio( const char port, const int pin_num,  const IO direction );
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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_StatusTypeDef ret;
  uint8_t buf[65536];
  #define Write_HM 0xAA
  #define Read_HM 0xAB
  int samples = 0x0F;
  /*Program the bootloader*/
  //PD0 = RTSN
  //PD1 = MFIO
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  0);
 // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  0);
  //HAL_Delay(50);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  0);
  HAL_Delay(6);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  0);
  HAL_Delay(4);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  1);
  HAL_Delay(50);


  //HAL_Delay(1000);
  buf[0] = 0x01;
  //buf[1] = 0x08;
  buf[1] = 0x00;
  buf[2] = 0x08;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
  HAL_Delay(2);

  printf("%d HAL bool\n", ret == HAL_OK);
  //if(ret == HAL_OK ){
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
  //}
  if(buf[0] != 0x00 || ret != HAL_OK ){
    printf("Error setting bootloader: code %x\n", buf[0]);
  }
  unsigned char byteF[254449];

  /*
  FILE *ptr;

  ptr = fopen("thealgo.bin","rb");  // r for read, b for binary
  if (!ptr)
     {
         printf("Error in reading file. Abort.\n");
         //return -3;
     }
  unsigned char byteF[254449];
  fread(&byteF,sizeof(byteF),1,ptr); // read 213408 bytes to our buffer

  for( int i = 0; i < 0x45; ++i){
	  printf("%x ", byteF[i]);
	  if(i%15){
		  printf("\n");
	  }
  }*/


  /*read mode*/
  buf[0] = 0x02;
  buf[1] = 0x00;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 2, 5000);
  printf("error code: %x application mode: %x\n", buf[0],buf[1]);


  /*setting page number*/
  uint8_t page_count = byteF[0x44];
  int page_size = 8192 + 16;
  buf[0] = 0x80;
  buf[1] = 0x02;
  buf[2] = 0x00;
  buf[3] = byteF[0x44];

  printf("%x pages\n", byteF[0x44]);
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 4, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
  if(buf[0] != 0x00 || ret != HAL_OK ){
    	  printf("Error setting page num: code %x\n", buf[0]);
   }

  /*initialization vector*/
  int byte_count = 0x32-0x28;
  buf[0] = 0x80;
  buf[1] = 0x00;
  for(int i = 0; i < byte_count; ++i){
	  buf[2+i] = byteF[0x28+i];
  }
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2 + byte_count, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
  if(buf[0] != 0x00 || ret != HAL_OK ){
      	  printf("Error setting page num: code %x\n", buf[0]);
   }
/*authentication bytes*/
  byte_count = 0x43-0x34;
	buf[0] = 0x80;
	buf[1] = 0x01;
	for(int i = 0; i < byte_count; ++i){
	  buf[2+i] = byteF[0x34+i];
	}
	ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2 + byte_count, 5000);
	HAL_Delay(2);
	ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
	if(buf[0] != 0x00 || ret != HAL_OK ){
			  printf("Error setting page num: code %x\n", buf[0]);
	   }

	/*erase application*/

	buf[0] = 0x80;
	buf[1] = 0x03;
	ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
	HAL_Delay(1400);
	ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
	if(buf[0] != 0x00 || ret != HAL_OK ){
			  printf("Error setting page num: code %x\n", buf[0]);
	   }
	int current = 0x4c;
	for(int i = 0; i < page_count; ++i){
		int count = 2;
		buf[0] = 0x8;
		buf[1] = 0x04;
		for(int j = current; j < (current + page_size); ++j){
			buf[count] = byteF[j];
			++count;
		}
		current = current + page_size;

		ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2 + page_size, 5000);
		HAL_Delay(340);
		ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
		if(buf[0] != 0x00 || ret != HAL_OK ){
					  printf("Error setting page %d : code %x\n", i, buf[0]);
			   }



	}







  //printf("buffer %x\n", byteF[0]);
//
//
//
  /*Go into application Mode*/
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  0);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  1);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  1);
  HAL_Delay(50);
  HAL_Delay(1000);


  config_gpio('D', 1, IN );
  /*set our mode to both raw and algorithm*/
  buf[0] = 0x02;
  buf[1] = 0x00;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 2, 5000);
  printf("error code: %x mode: %x\n", buf[0],buf[1]);

  buf[0] = 0xFF;
  buf[1] = 0x03;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 5, 5000);
  printf("error code: %x mode: %x %x %x %x\n", buf[0],buf[1], buf[2], buf[3],buf[4]);


  buf[0] = 0x10;
  buf[1] = 0x00;
  buf[2] = 0x03;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
  if(buf[0] != 0x00 || ret != HAL_OK ){
	  printf("Error setting mode: code %x\n", buf[0]);
  }

  /*Set FIFO threshold as almost full at 0x0F*/
  buf[0] = 0x10;
  buf[1] = 0x01;
  buf[2] = samples;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
  HAL_Delay(2);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
    if(buf[0] != 0x00 || ret != HAL_OK ){
  	  printf("Error setting FIFO threshold code: %x\n", buf[0]);
    }


  /*Enable the sensor*/
  buf[0] = 0x44;
  buf[1] = 0x03;
  buf[2] = 0x01;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
  HAL_Delay(40);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
    if(buf[0] != 0x00 || ret != HAL_OK ){
  	  printf("Error enabling sensor code: %x\n", buf[0]);
    }


  /*Enable the algorithm*/
  buf[0] = 0x52;
  buf[1] = 0x02;
  buf[2] = 0x01;
  ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 3, 5000);
  HAL_Delay(40);
  ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 1, 5000);
    if(buf[0] != 0x00 || ret != HAL_OK ){
  	  printf("Error Enabling Algorithm code: %x\n", buf[0]);
    }




  float heart_rate = 0;
  float SpO2 = 0;

  while (1)
      {
	    HAL_Delay(1000);
	    int error = 0;
	    /*read sensor hub status*/
	    buf[0] = 0x00;
	    buf[1] = 0x00;
	    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
	    if ( ret != HAL_OK ) {
	        printf("Error sensor write\r\n");
	        error = 1;
	        continue;
	    } else 
          {
	      	ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 2, 5000);
	      	if(buf[1] != 0x08)
              {
	      	    printf("Data bit not ready %x \n", buf[2]);
	      	    continue;
	      	    } // end if
	      	else if(buf[0] != 0x0)
              {
			        printf(" %x error \n", buf[1]);
					    continue;
				      } // end if
	  	  	/*read FIFO hub status*/
	  	  	buf[0] = 0x12;
	  	  	buf[1] = 0x00;
	  	  	ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
	  	  	if ( ret != HAL_OK ) {
	  	  	    printf("Error algorithm write\r\n");
	  	  	    error = 1;
	  	  	    continue;
	  	  	} else // Was able to write.
              {
	  	  	    ret = HAL_I2C_Master_Receive(&hi2c1, Read_HM, &buf[0], 2, 5000);
	  	  	    int sample_size = buf[1];
	  	  	    if(buf[0] != 0x0)
                  {
	  	  	        printf(" %x error \n", buf[1]);
	  	  	        continue;
	  	  	        } // end if
	  	  	    /* read the data */
					    buf[0] = 0x12;
					    buf[1] = 0x01;
					    ret = HAL_I2C_Master_Transmit(&hi2c1, Write_HM, &buf[0], 2, 5000);
					    ret = HAL_I2C_Master_Recieve(&hi2c1, Read_HM, &buf[0],2, 5000);
				      if ( ret != HAL_OK ) 
                  {
				          printf("Error algorithm read\r\n");
								  error = 1;
								  continue;
								  } 
              else 
                  {

								  int length_of_data = 1 + 18*sample_size;
								  ret = HAL_I2C_Master_Read(&hi2c1, Read_HM, &buf[0],length_of_data, 5000);
								  //ret = HAL_I2C_Master_Recieve(&hi2c1, Read_HM, &buf[0],2, 5000); //checks status
								  if(buf[0] != 0x0)
                      {
								  	  printf(" %x error \n", buf[1]);
								  	  continue;
								  	  } // end if
								  /*this gets us our data for heart_rate and SpO2*/
								  float viable = 0.0; //counts how many viable samples we have
								  for(int i = 13; i < length_of_data; i = i + 18)
                      {
								  		int temp_heart = (buf[i]<<8) + buf[i+1];
								  		temp_heart = temp_heart >> 1; //need to change to account for the 0.1
								  		int temp_SpO2 = (buf[i+3]<<8) + buf[i+4];
								  		temp_heart = temp_heart >> 1; //need to change to account for the 0.1
								  		int finger_status = buf[i+5];
								  		if(finger_status == 3)
                          {
								  			  ++viable;
								  			  heart_rate += temp_heart;
								  			  SpO2 += temp_SpO2;
								  	      } // end if
								      } // end for
								  heart_rate = heart_rate / viable;
								  SpO2 = SpO2 / viable; //average out our sample value

								  printf("heart: %f, SpO2: %f", heart_rate, SpO2);
								  } // end else
	  	  		  } // end else
	  	  	} // end else
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    } // end while
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
