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
//#include "ble_commands.h"
#include "ble.h"

#include <stdlib.h>

#include <stdint.h>
#include <stdio.h>

/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include LED driver */
#include "leds.h"
#include "timer.h"

/* include i2c driver*/
#include "i2c.h"
#include "lsm6dsl.h"
#include "math.h"

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();

  HAL_Delay(10);

  uint8_t nonDiscoverable = 0;

  /*leds_init();
	timer_init(TIM2);
	i2c_init();
	lsm6dsl_init();
	/*put lost detection algorithm here
	poll continuously the values of the output registers.

	// Loop forever
	int16_t prev_x = 0;
	int16_t prev_y = 0;
	int16_t prev_z = 0;
	for(;;) {
		int16_t x;
		int16_t y;
		int16_t z;
		lsm6dsl_read_xyz(&x,&y,&z);

		if(!(prev_x == 0 && prev_y == 0 && prev_z == 0)) {
			if (abs(x - prev_x) >= threshold || abs(y - prev_y) >= threshold || abs(z - prev_z) >= threshold) {  //it is moving
				lostFlag = 0;   //it is not lost
				startTimer = 0;   //stop the 1min timer since its not lost
				leds_set(0);   //reset leds to off whenever it switches from lost to not lost
			}
			else {  //it moved less than the threshold, so we say its lost
				startTimer = 1;
			}
		}
		prev_x = x;   //set prev to be equal to the current x
		prev_y = y;
		prev_z = z;

		//debugging print flags
		//printf("x: %d y: %d z: %d\n", x,y,z);
		//printf("lostFlag status %d\n", lostFlag);
	}*/



	while (1)
	  {
		  if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
			catchBLE();
		  }else{
			  HAL_Delay(1000);
			  // Send a string to the NORDIC UART service, remember to not include the newline
			  unsigned char test_str[] = "youlostit BLE test";
			  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str)-1, test_str);
		  }
		  // Wait for interrupt, only uncomment if low power is needed
		  //__WFI();
	  }
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
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
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


/*void TIM2_IRQHandler() {

	  // Check if the interrupt was caused by the update event
	if (TIM2->SR & TIM_SR_UIF) {
		//Clear the update interrupt flag
		TIM2->SR &= ~TIM_SR_UIF;
	}*/

	/*have a counter that counts up every time we enter interrupt when its lost.
	 * Enters interrupt 20 times per second (20hz), so counterup = 1200 means its been 1 min
	 */

	/*if(startTimer == 1) {
		counterup = counterup + 1;  //only start counting when the thing isn't moving
	}
	else {
		counterup = 0;
	}

	if (counterup >= 1200) {
		lostFlag = 1;   //it is lost
		if(preamble != 0) {
			uint8_t bitmask1 = preamble;
			//push the left 2 bits all the way to the right
			//so if we have 10011001 as preamble, then I push the leftmost 2 bits all the way to the right to get 00000010
			//then I call leds_set using those 2 bits
			bitmask1 >>= 6;
			leds_set(bitmask1);         //turn on the leds based on the 2 bits that I just pushed over
			//next I shift the preamble left by 2 bits so that I can repeat with the next 2 bits
			//so if I have preamble as 10011001, I end up with 01100100
			preamble <<= 2;        //left shift by 2 so i can read the next 2 bits
		}
		else if (ID != 0) {

			//repeat preamble algorithm with ID
			uint16_t bitmask2 = ID;
			bitmask2 >>= 14;       //push leftmost 2 bits all the way to the right
			leds_set(bitmask2);      //call leds_set on the 2 bits
			ID <<= 2;      //shift left by 2 bits to read the next 2 bits
		}
		else if (numMinutes != 0) {
			uint8_t minutesLost = numMinutes;
			//push the left 2 bits all the way to the right
			//so if we have 00001010 as minutes since lost, then I push the leftmost 2 bits all the way to the right to get 00000000
			//then I call leds_set using those 2 bits
			minutesLost >>= 6;
			leds_set(minutesLost);         //turn on the leds based on the 2 bits that I just pushed over
			//next I shift the numminutes over by 2 bits so that I can repeat with the next 2 bits
			//so if I have 00001010 as minutes since lost, I shift left to have 00101000
			numMinutes <<= 2;        //left shift by 2 so i can read the next 2 bits
		}
		else {    //reset preamble,ID, and minutes since lost when done blinking through all of them
			preamble = 0x99;    //preamble in hex
			ID = 7663;      //Phils ID
			numMinutes = (uint8_t)(floor(counterup/1200));
			//every 1200 counts is 1min, floor function to make sure its always an integer
		}
	}
}*/

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
