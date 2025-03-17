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
#include "lptimer.h"

/* include i2c driver*/
#include "i2c.h"
#include "lsm6dsl.h"
#include "math.h"


//volatile uint8_t preamble = 0x99;    //preamble in hex
//volatile uint16_t ID = 7663;      //Phils ID
volatile int counterup = 0;      //counter so that we can track when 1min has passed
volatile int threshold = 1500;     //threshold for accelerometer movement
volatile int lostFlag = 0;  //0 means not lost, 1 means lost
volatile int startTimer = 0;     //0 means the 1min lost timer is not on, 1 means the 1min lost timer is on
//volatile unsigned int numMinutes = 1;   //minutes since lost
volatile unsigned int numSeconds = 0;
volatile uint8_t sendFlag = 0;    //flag to see if you should send the tag message

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */


int _write(int file, char *ptr, int len) {
	//ITM_SendChar('H');
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void disable_bus() {
	  RCC->AHB1ENR = 0x00000000;
	  RCC->AHB2ENR = 0x00000000;
	  RCC->AHB3ENR = 0x00000000;
	  //RCC->APB1ENR1 = 0x00000000;
	  RCC->APB1ENR2 = 0x00000000;
	  RCC->APB2ENR = 0x00000000;
	  RCC->AHB1SMENR = 0x00000000;
	  RCC->AHB2SMENR = 0x00000000;
	  RCC->AHB3SMENR = 0x00000000;
	  //RCC->APB1SMENR1 = 0x00000000;
	  RCC->APB1SMENR2 = 0x00000000;
	  RCC->APB2SMENR = 0x00000000;
}

int main(void) {
    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    disable_bus();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_SPI3_Init();

    // Initialize LPTIM for the 60-second timer
    lptimer_init(LPTIM1);
    lptim_set_ms(5000);  // 5-second period

    // Initialize I2C and accelerometer
    i2c_init();
    lsm6dsl_init();

    // Initialize BLE module
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
    ble_init();
    HAL_Delay(100); // Wait for BLE module to stabilize

    // Set BLE to non-discoverable mode
    disconnectBLE(); 
    uint8_t nonDiscoverable = 0; // By default, be non-discoverable
    setDiscoverability(0);       // Make it non-discoverable
    standbyBle(); // Put BLE module in standby mode

    // Initialize variables for accelerometer data
    int16_t prev_x = 0;
    int16_t prev_y = 0;
    int16_t prev_z = 0;
    printf("End of inits\n");

    // Main loop
    while (1) {
        // Check if BLE interrupt is active
        if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port, BLE_INT_Pin)) {
            catchBLE();
        }

        // Read accelerometer data
        int16_t x, y, z;
        lsm6dsl_read_xyz(&x, &y, &z);

        // Check if the device is moving
        if (!(prev_x == 0 && prev_y == 0 && prev_z == 0)) {
            if (abs(x - prev_x) >= threshold || abs(y - prev_y) >= threshold || abs(z - prev_z) >= threshold) {
                // Device is moving
                if (lostFlag == 1) {
                    lostFlag = 0; // Switch back to "not lost"
                }
                disconnectBLE();    // Disconnect before setting discoverability to 0
                setDiscoverability(0); // Make it non-discoverable
                standbyBle();      // Put BLE in standby mode
                startTimer = 0;    // Stop the 60-second timer
                counterup = 0;     // Reset the lost timer
            } else {
                // Device is not moving (considered "lost")
                startTimer = 1; // Start the 60-second timer
            }
        }
        prev_x = x;
        prev_y = y;
        prev_z = z;

        // If the device is lost, set it to discoverable
        if (lostFlag) {
            setDiscoverability(1);
        }

        // Send a message if the sendFlag is set
        if (sendFlag) {
            unsigned char test_str[20] = "FMtag lost for";
            snprintf(test_str, 20, "FMtag lost for %ds", numSeconds);
            updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, sizeof(test_str) - 1, test_str);
            sendFlag = 0;
        }

        // Enter STOP2 mode to save power
        HAL_SuspendTick();
        HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
        HAL_ResumeTick();
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
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;    //100khz


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
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   //scale down APB1 to lowest possible while still maintaining functionality
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;   //scale down APB2 to lowest possible while still maintaining functionality

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

void LPTIM1_IRQHandler(void)
{
    // Check if the autoreload match interrupt flag is set
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        // Clear the autoreload match interrupt flag
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;

        // Handle the timer logic
        if (startTimer == 1) {
            counterup++;
        }
        if (counterup >= 12) {
            lostFlag = 1;
            if ((counterup % 2) == 0) {
                sendFlag = 1;
            }
            numSeconds = (unsigned int)(floor((counterup - 12) * 5));
        }
    }
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
