/**
  *
  *
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

//Timer 2 interrupt handler
void TIM2_IRQHandler(void)
{
  //Toggle green and orange LEDs
  GPIOC->ODR ^= GPIO_ODR_8;
  GPIOC->ODR ^= GPIO_ODR_9;
  //Clear update interrupt pending flag
  TIM2->SR &= ~TIM_SR_UIF;
}

//Initialize all four LEDs
void init_leds(void)
{
  //Initialize red LED, PC6
  GPIOC->MODER |= GPIO_MODER_MODER6_1; //Alternate function
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1); //No pull up or down
  GPIOC->AFR[0] &= 0xF0FFFFFF;

  //Initialize blue LED, PC7
  GPIOC->MODER |= GPIO_MODER_MODER7_1; //Alternate function
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1); //No pull up or down
  GPIOC->AFR[0] &= 0x0FFFFFFF;

  //Initialize orange LED, PC8
  GPIOC->MODER |= GPIO_MODER_MODER8_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR8_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1); //No pull up or down

  //Initialize green LED, PC9
  GPIOC->MODER |= GPIO_MODER_MODER9_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR9_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1); //No pull up or down

}

int main(void) 
{
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); //Configure the system clock

  //Enable clock to GPIOC for LEDS
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
  //Enable clock to GPIOA for button
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  
  //Initialize all LEDs
  init_leds(); 

  //Set green LED on and orange LED off
  GPIOC->BSRR |= GPIO_BSRR_BS_9;
  GPIOC->BSRR |= GPIO_BSRR_BR_8;

  //Enable clock to timer 2
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  //Set timer 2 frequency to 1 KHz by dividing by 7,999
  TIM2->PSC = 7999U;
  //Set ARR to count to 250
  TIM2->ARR = 250U;
  //Set edge-aligned mode
  TIM2->CR1 &= ~TIM_CR1_CMS;
  //Set upcounting mode
  TIM2->CR1 &= ~TIM_CR1_DIR;
  //Enable update event generation
  TIM2->CR1 &= ~TIM_CR1_UDIS;
  //Enable update interrupt
  TIM2->DIER |= TIM_DIER_UIE;
  //Enable timer 2
  TIM2->CR1 |= TIM_CR1_CEN;

  //Enable timer 2 interrupt handler in NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
  //Set priority to 1
  NVIC_SetPriority(TIM2_IRQn,1);

  //Enable clock to timer 3
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  //Set timer 3 frequency to 800 KHz by dividing by 10
  TIM3->PSC = 9U;
  //Set ARR to count to 1000
  TIM3->ARR = 1000U;
  //Set edge-aligned mode
  TIM3->CR1 &= ~TIM_CR1_CMS;
  //Set upcounting mode
  TIM3->CR1 &= ~TIM_CR1_DIR;
  //Enable update event generation
  TIM3->CR1 &= ~TIM_CR1_UDIS;
  //Configure channels 1 & 2 as outputs
  TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
  TIM3->CCMR1 &= ~TIM_CCMR1_CC2S;
  //Set output channel 1 to PWM mode 2
  TIM3->CCMR1 |= TIM_CCMR1_OC1M;
  //Set output channel 2 to PWM mode 1
  TIM3->CCMR1 &= ~TIM_CCMR1_OC2M_0;
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_1;
  TIM3->CCMR1 |= TIM_CCMR1_OC2M_2;
  //Enable output compare preload for both channels
  TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
  TIM3->CCMR1 |= TIM_CCMR1_OC2PE;
  //Set output enable for both channels
  TIM3->CCER |= TIM_CCER_CC1E;
  TIM3->CCER |= TIM_CCER_CC2E;
  //Enable timer 3
  TIM3->CR1 |= TIM_CR1_CEN;

  //To best demonstrate the change in the CCRx values on the LED brightness
  //  and the differences between PWM modes 1 & 2 I am configuring timer 1 
  //  in up-down count mode and the main loop will set the CCRx values of
  //  timer 3 to the count of timer 1, approximating a sine wave like in
  //  figure 3.5 in the lab handout
  
  //Enable clock to timer 1
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  //Set timer 1 frequency to 1 KHz by dividing by 8000
  TIM1->PSC = 7999U;
  //Set ARR to count to 1000
  TIM1->ARR = 1000U;
  //Set center aligned mode 3
  TIM1->CR1 |= TIM_CR1_CMS;
  //Set upcounting mode
  TIM2->CR1 &= ~TIM_CR1_DIR;
  //Enable timer 1
  TIM1->CR1 |= TIM_CR1_CEN;
  
  //Main loop
  while (1) 
  { 
    TIM3->CCR1 = TIM1->CNT;
    TIM3->CCR2 = TIM1->CNT;
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
