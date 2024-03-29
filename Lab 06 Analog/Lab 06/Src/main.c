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
  SystemClock_Config();

  // -------------------------------------------------------------------------------------------------------------------------------
  // 6.1 Measuring a Potentiometer With the ADCx
  // -------------------------------------------------------------------------------------------------------------------------------

  // // Initialize LED pins (PC6 and PC7) to output
  // RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  // GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;

  // // Select PA0 as ADC input
  // RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // GPIOA->MODER |= GPIO_MODER_MODER0;

  // // Enable ADC1 in RCC peripheral
  // RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // // Configure ADC to 8-bit resolution, continuous conversion mode, software trigger
  // ADC1->CFGR1 |= ADC_CFGR1_CONT; // Continuous conversion mode
  // ADC1->CFGR1 &= ~ADC_CFGR1_RES; // 8-bit resolution

  // // Select PA0 as ADC input channel
  // ADC1->CHSELR |= ADC_CHSELR_CHSEL0;

  // // Perform ADC calibration
  // ADC1->CR |= ADC_CR_ADCAL;
  // while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to finish

  // // Enable ADC
  // ADC1->CR |= ADC_CR_ADEN;
  // while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait for ADC to be ready

  // // Start ADC conversion
  // ADC1->CR |= ADC_CR_ADSTART;

  // // Threshold values for LED activation
  // uint16_t thresholds[] = {500, 1500, 3000, 4000}; // Values for a 20k potentiometer

  // while (1) {
  //   // Read ADC data register
  //   uint16_t adc_value = ADC1->DR;

  //   // Adjust LEDs based on ADC value
  //   for (int i = 0; i < 4; i++) {
  //     if (adc_value >= thresholds[i]) {
  //       GPIOC->BSRR = (1 << (6 + i)); // Turn on LED
  //     } else {
  //       GPIOC->BSRR = (1 << (22 + i)); // Turn off LED
  //     }
  //   }
  // }

  // -------------------------------------------------------------------------------------------------------------------------------
  // 6.2 Generating Waveforms with the DAC
  // -------------------------------------------------------------------------------------------------------------------------------

  // Select PA4 as DAC output
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODER4;

  // Set PA4 to DAC output mode
  // GPIOA->MODER |= GPIO_MODER_MODER4_1;
  GPIOA->MODER |= (1<<8);
  GPIOA->MODER |= (1<<9);

  GPIOA->PUPDR  &= ~(1<<8);
  GPIOA->PUPDR &= ~(1<<9);

  // Enable DAC channel 1
  RCC->APB1ENR |= RCC_APB1ENR_DACEN;
  // DAC->CR |= DAC_CR_EN1;
  DAC->CR |= (1<<0);

  // Set DAC channel 1 to software trigger mode
  // DAC->CR |= DAC_CR_TEN1;
  // DAC->CR &= ~DAC_CR_TSEL1;
  DAC->CR |= (1<<3);
  DAC->CR |= (1<<4);
  DAC->CR |= (1<<5);

  // Sine Wave: 8-bit, 32 samples/cycle
  const uint8_t sine_wave[32] = {127,151,175,197,216,232,244,251,254,251,244,
  232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

  uint8_t index = 0;

  while (1) {
    // Write the next value from the wave-table to DAC channel 1 data register
    DAC->DHR8R1 = sine_wave[index];

    // Increment index 
    index = (index + 1);
    if(index == 32)
    {
      index = 0;
    }

    // 1ms delay
    HAL_Delay(1);
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
