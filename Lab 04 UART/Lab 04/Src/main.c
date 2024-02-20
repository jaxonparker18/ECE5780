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

int main(void) {
  HAL_Init();               // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config();     // Configure the system clock

  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock for USART3 pins

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

  // USART3 configuration for PB10 (TX) and PB11 (RX)
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock

  // Set the Baud rate to 115200 bits/second
  uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
  uint32_t baud_rate = 115200;
  USART3->BRR = hclk_freq / baud_rate;

  // Enable transmitter and receiver hardware
  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;

  // Enable USART peripheral
  USART3->CR1 |= USART_CR1_UE;

  // Set PB10 and PB11 to alternate function mode
  GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
  GPIOB->MODER |= (GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1);

  // Program the correct alternate function number into the GPIO AFR registers
  GPIOB->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL10_Pos) | (0x04 << GPIO_AFRH_AFSEL11_Pos);

  // Initialize one pin logic high and the other to low
  GPIOC->ODR |= (1 << 7);   // Red High
  GPIOC->ODR &= ~(1 << 6);  // Blue Low
  GPIOC->ODR |= (1 << 9);   // Green high
  GPIOC->ODR &= ~(1 << 8);  // Orange Low

  while (1) {
    // Toggle red LED (PC6) with a delay of 400-600ms
    GPIOC->ODR ^= (1 << 6);
    HAL_Delay(500);

    // Transmit a character (for example, 'A') using the USART
    USART_TransmitChar('A');
  }
}

// Function to transmit a single character on USART3
void USART_TransmitChar(char c) {
  // Wait for the USART transmit data register to be empty
  while (!(USART3->ISR & USART_ISR_TXE));

  // Write the character into the transmit data register
  USART3->TDR = (uint16_t)c;
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
