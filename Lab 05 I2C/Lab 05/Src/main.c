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

  // Set up LEDs and C5
  GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12) | (1 << 0);    // Set PC9-PC6 and PC0 to output mode
  GPIOC->BSRR = (1 << 0);    // Set PC0 (CS) line high, selects I2C mode on gyro

  GPIOB->MODER  |= (1 << 23) | (1 << 27) | (1 << 28);  // Set PB11 & PB13 to AF Mode, PB14 to ouput
  GPIOB->OTYPER |= (1 << 11) | (1 << 13);              // Set PB11 & PB13 to open-drain output type
  GPIOB->PUPDR  |= (1 << 22) | (1 << 26);              // Set internal pull-up resistors on PB 11 & PB13
  GPIOB->AFR[1] = 0x00501000;                          // Set AF1 on PB11(I2C2_SDA) & AF5 on PB13(I2C2_SCL)
  GPIOB->BSRR = (1 << 14);                             // Set PB14 (address select) line high

  // Enable I2C2 peripheral in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Configure I2C2 parameters for 100 kHz standard-mode
  I2C2->TIMINGR = (1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);    // From table

  // Enable the I2C2 peripheral using the PE bit in the CR1 register
  I2C2->CR1 |= I2C_CR1_PE;

  // -------------------------------------------------------------------------------------------------------------------------------
  // 5.4 Reading the Register
  // -------------------------------------------------------------------------------------------------------------------------------

  // // Set the transaction parameters in the CR2 register
  // // For write operation (sending register address)
  // I2C2->CR2 = (0x69 << 1) | (1 << 16);

  // // Set the START bit to begin the address frame
  // I2C2->CR2 |= I2C_CR2_START;

  // // Wait until either TXIS or NACKF flags are set
  // while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Check if NACKF flag is set (slave did not respond)
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Write the address of the "WHO_AM_I" register into the I2C transmit register (TXDR)
  // I2C2->TXDR |= 0x0F;

  // // Wait until TC (Transfer Complete) flag is set
  // while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }
    
  // // Check if NACKF flag is set (slave did not respond)
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Reload CR2 register with the same parameters but set RD_WRN for read operation
  // I2C2->CR2 = (0x69 << 1) | (1 << 16) | I2C_CR2_RD_WRN | I2C_CR2_START;

  // // Wait until either RXNE or NACKF flags are set
  // while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Check if NACKF flag is set (slave did not respond)
  // if (I2C2->ISR & I2C_ISR_NACKF)
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Wait until TC (Transfer Complete) flag is set
  // while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // // Set the STOP bit in the CR2 register to release the I2C bus
  // I2C2->CR2 |= I2C_CR2_STOP;

  // // Check the contents of the RXDR register to see if it matches the expected value (0xD4)
  // if (I2C2->RXDR != 0xD3) // I2C2->RXDR == 0x69
  // {
  //   GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
  //   // Handle the error
  // }

  // GPIOC->BSRR |= (1 << (22)); // Clear PC6 to turn off the red LED

  // // Shows it made it through while and if statements
  // GPIOC->BSRR |= (1 << 7); // Set PC7 to turn on the blue LED

  // -------------------------------------------------------------------------------------------------------------------------------
  // 5.5 Initializing the Gyroscope
  // -------------------------------------------------------------------------------------------------------------------------------

  // Keep all other bits in CTRL_REG1 register as 0
  I2C2->TXDR = (1 << 0) | (1 << 1) | (1 << 7);

  // Write ctrlReg1Value to the CTRL_REG1 register of the gyroscope
  I2C2->CR2 = (0x69 << 1) | (1 << 16); // Addressing the gyroscope
  I2C2->TXDR = 0x20; // Register address of CTRL_REG1

  while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
  }

  I2C2->TXDR = 0x0B; //bit pattern to set Xen and Yen

  while (!(I2C2->ISR & (I2C_ISR_TC | I2C_ISR_NACKF)))
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle the error
  }

  if (I2C2->ISR & I2C_ISR_NACKF) 
  {
    GPIOC->BSRR |= (1 << 6); // Set PC6 high to turn on the red LED
    // Handle NACK error
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
