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

  // Set up LEDs
  GPIOC->MODER = 0x55000;
  GPIOC->OTYPER = 0x00000000;
  GPIOC->OSPEEDR = 0x00000000;
  GPIOC->PUPDR = 0x00000000;

  // Configure PB11 as alternate function mode, open-drain output type, and select I2C2_SDA
  // GPIOB->MODER &= ~(3 << 22);   // Clear mode bits for PB11
  GPIOB->MODER |= (1 << 23);    // Set PB11 to alternate function mode
  GPIOB->OTYPER |= (1 << 11);   // Set PB11 to open-drain output type
  GPIOB->AFR[1] |= (4 << 12);   // Select alternate function I2C2_SDA for PB11

  // Configure PB13 as alternate function mode, open-drain output type, and select I2C2_SCL
  // GPIOB->MODER &= ~(3 << 26);   // Clear mode bits for PB13
  GPIOB->MODER |= (1 << 27);    // Set PB13 to alternate function mode
  GPIOB->OTYPER |= (1 << 13);   // Set PB13 to open-drain output type
  GPIOB->AFR[1] |= (1 << 22);   // Select alternate function I2C2_SCL for PB13
  GPIOB->AFR[1] |= (1 << 20);   // Select alternate function I2C2_SCL for PB13

  // Configure PB14 as output mode, push-pull output type, and initialize/set the pin high
  GPIOB->MODER |= (1 << 28);    // Set PB14 to output mode
  // GPIOB->OTYPER &= ~(1 << 14);  // Set PB14 to push-pull output type
  GPIOB->BSRR |= (1 << 14);     // Set PB14 high initially

  // Configure PC0 as output mode, push-pull output type, and initialize/set the pin high
  GPIOC->MODER |= 1;     // Set PC0 to output mode
  // GPIOC->OTYPER &= ~(1 << 0);   // Set PC0 to push-pull output type
  GPIOC->ODR |= 1;      // Set PC0 high initially

  // Enable I2C2 peripheral in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Configure I2C2 parameters for 100 kHz standard-mode
  uint32_t timingValue = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20); // PRESC | SCLL | SCLH | SDADEL | SCLDEL 
  I2C2->TIMINGR = timingValue;

  // Enable the I2C2 peripheral using the PE bit in the CR1 register
  I2C2->CR1 |= I2C_CR1_PE;

 

  // 5.4 --------------------------------------

  char failed = 'f';

  while(1) 
  {

    // Set the transaction parameters in the CR2 register
    // For write operation (sending register address)
    I2C2->CR2 |= (0x69 << 1); // Set slave address
    I2C2->CR2 |= (1 << 16); // Set number of bytes = 1
    I2C2->CR2 &= ~(1 << 10); // Set write operation

    // Set the START bit to begin the address frame
    I2C2->CR2 |= I2C_CR2_START;

    failed = 'f';

    // Wait until either TXIS or NACKF flags are set
    while(1) 
    { 
      // Checks if NACKF flag was set
      if((I2C2->ISR & I2C_ISR_NACKF) == (1 << 4)) 
      {
        failed = 't';
        break;
      }
      // Checks if TXIS flag was set
      if((I2C2->ISR & I2C_ISR_TXIS) == (1 << 1)) 
      {
        break;
      }
    }

    if(failed == 'f') 
    {
      // Write the address of the "WHO_AM_I" register into the I2C transmit register (TXDR)
      I2C2->TXDR |= 0x0F;

      // Wait until TC (Transfer Complete) flag is set
      while ((I2C2->ISR & I2C_ISR_TC) != (1 << 6))
      {
        // Waiting for TC to be set
      }

      GPIOC->ODR |= (1 << 9); // green LED
      HAL_Delay(3000);
    }
    // Check if NACKF flag is set (slave did not respond)
    else 
    {
      GPIOC->ODR |= (1 << 6); // red LED
    }

    // Reload CR2 register with the same parameters but set RD_WRN for read operation
    // I2C2->CR2 |= (0x69 << 1); // Set slave address
    // I2C2->CR2 |= (1 << 16); // Set number of bytes = 1
    I2C2->CR2 &= ~(1 << 10); // Set write operation
    I2C2->CR2 |= I2C_CR2_START; // Set START bit

    failed = 'f';

    // Wait until either RXNE or NACKF flags are set
    while(1) 
    { 
      // Checks if NACKF flag was set
      if((I2C2->ISR & I2C_ISR_NACKF) == (1 << 4)) 
      {
        failed = 't';
        break;
      }
      // Checks if RXNE flag was set
      if((I2C2->ISR & I2C_ISR_RXNE) == (1 << 2)) 
      {
        break;
      }
    }
    
    if(failed == 'f') 
    {
      // Wait until TC (Transfer Complete) flag is set
      while ((I2C2->ISR & I2C_ISR_TC) != (1 << 6))
      {
        // Waiting for TC to be set
      }

      // Check the contents of the RXDR register to see if it matches the expected value (0xD4)
      if (I2C2->RXDR == 0xD3) // I2C2->RXDR == 0x69
      {
        GPIOC->ODR |= (1 << 7); // blue LED
        // Transaction successful
      }
      else
      {
        GPIOC->ODR |= (1 << 8); // orange LED
        // Transaction failed
      }

      HAL_Delay(3000);
    }
    // Check if NACKF flag is set (slave did not respond)
    else 
    {
      GPIOC->ODR |= (1 << 6); // red LED
    }

    // Set the STOP bit in the CR2 register to release the I2C bus
    I2C2->CR2 |= I2C_CR2_STOP;

    GPIOC->ODR &= ~(1 << 7); // blue LED
    GPIOC->ODR &= ~(1 << 9); // green LED
    HAL_Delay(3000);
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
