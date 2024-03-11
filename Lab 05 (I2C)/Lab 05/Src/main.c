/**
  *
  * Jaxon Parker
  * U1289670
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();               // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config();     // Configure the system clock

  // Enable GPIOB and GPIOC clocks in the RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Configure PB11 as alternate function mode, open-drain output type, and select I2C2_SDA
  GPIOB->MODER &= ~(3 << 22);   // Clear mode bits for PB11
  GPIOB->MODER |= (2 << 22);    // Set PB11 to alternate function mode
  GPIOB->OTYPER |= (1 << 11);   // Set PB11 to open-drain output type
  GPIOB->AFR[1] |= (4 << 12);   // Select alternate function I2C2_SDA for PB11

  // Configure PB13 as alternate function mode, open-drain output type, and select I2C2_SCL
  GPIOB->MODER &= ~(3 << 26);   // Clear mode bits for PB13
  GPIOB->MODER |= (2 << 26);    // Set PB13 to alternate function mode
  GPIOB->OTYPER |= (1 << 13);   // Set PB13 to open-drain output type
  GPIOB->AFR[1] |= (4 << 20);   // Select alternate function I2C2_SCL for PB13

  // Configure PB14 as output mode, push-pull output type, and initialize/set the pin high
  GPIOB->MODER |= (1 << 28);    // Set PB14 to output mode
  GPIOB->OTYPER &= ~(1 << 14);  // Set PB14 to push-pull output type
  GPIOB->BSRR |= (1 << 14);     // Set PB14 high initially

  // Configure PC0 as output mode, push-pull output type, and initialize/set the pin high
  GPIOC->MODER |= (1 << 0);     // Set PC0 to output mode
  GPIOC->OTYPER &= ~(1 << 0);   // Set PC0 to push-pull output type
  GPIOC->BSRR |= (1 << 0);      // Set PC0 high initially

  // Configure the LED pins to slow-speed, push-pull output mode without pull-up/down resistors
  // Blue LED (PC7)
  GPIOC->MODER |= (1 << 14);
  GPIOC->MODER &= ~(1 << 15);
  GPIOC->OTYPER &= ~(1 << 7);
  GPIOC->OSPEEDR &= ~(1 << 14);
  GPIOC->OSPEEDR &= ~(1 << 15);
  GPIOC->PUPDR &= ~(1 << 14);
  GPIOC->PUPDR &= ~(1 << 15);

  // Red LED (PC6)
  GPIOC->MODER |= (1 << 12);
  GPIOC->MODER &= ~(1 << 13);
  GPIOC->OTYPER &= ~(1 << 6);
  GPIOC->OSPEEDR &= ~(1 << 12);
  GPIOC->OSPEEDR &= ~(1 << 13);
  GPIOC->PUPDR &= ~(1 << 12); 
  GPIOC->PUPDR &= ~(1 << 13); 

  // Green LED (PC9)
  GPIOC->MODER |= (1 << 18);
  GPIOC->MODER &= ~(1 << 19);
  GPIOC->OTYPER &= ~(1 << 9);
  GPIOC->OSPEEDR &= ~(1 << 18);
  GPIOC->OSPEEDR &= ~(1 << 19);
  GPIOC->PUPDR &= ~(1 << 18); 
  GPIOC->PUPDR &= ~(1 << 19);

  // Orange LED (PC8) 
  GPIOC->MODER |= (1 << 16);
  GPIOC->MODER &= ~(1 << 17);
  GPIOC->OTYPER &= ~(1 << 8);
  GPIOC->OSPEEDR &= ~(1 << 16);
  GPIOC->OSPEEDR &= ~(1 << 17);
  GPIOC->PUPDR &= ~(1 << 16); 
  GPIOC->PUPDR &= ~(1 << 17);

  // User Button
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 

  GPIOA->MODER &= ~(3 << 0);      // Set as input
  GPIOA->OSPEEDR &= ~(3 << 0);    // Set as low speed
  GPIOA->PUPDR |= (1 << 0);       // Enable pull-down resistor

  // Initialize one pin logic high and the other to low
  GPIOC->ODR |= (1 << 7);     // Red High
  GPIOC->ODR &= ~(1 << 6);    // Blue Low
  GPIOC->ODR &= ~(1 << 9);    // Green Low
  GPIOC->ODR &= ~(1 << 8);    // Orange Low

  // Enable I2C2 peripheral in the RCC
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

  // Configure I2C2 parameters for 100 kHz standard-mode
  uint32_t timingValue = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);
  I2C2->TIMINGR = timingValue;

  // Enable the I2C2 peripheral using the PE bit in the CR1 register
  I2C2->CR1 |= I2C_CR1_PE;

  while (1)
  {
  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
