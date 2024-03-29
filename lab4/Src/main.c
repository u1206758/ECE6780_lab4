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


//Initialize all four LEDs
void init_leds(void)
{
  //Initialize red LED, PC6
  GPIOC->MODER |= GPIO_MODER_MODER6_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1); //No pull up or down

  //Initialize blue LED, PC7
  GPIOC->MODER |= GPIO_MODER_MODER7_0; //General purpose output
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7; // Push-pull
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_0; //Low speed
  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1); //No pull up or down

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

  //Set all LEDs off
  GPIOC->BSRR |= GPIO_BSRR_BR_6;
  GPIOC->BSRR |= GPIO_BSRR_BR_7;
  GPIOC->BSRR |= GPIO_BSRR_BR_8;
  GPIOC->BSRR |= GPIO_BSRR_BR_9;

}

//Global variables for USART3 received data
char rec_data;
uint8_t rec_data_flag;

//USART3 Interrupt Handler
void USART3_4_IRQHandler(void)
{
  rec_data = USART3->RDR;
  rec_data_flag = 1;
}

//Transmits one character over UART
void transmit_char(char c)
{
  //Wait until USART transmit register is empty
  while (!(USART3->ISR & USART_ISR_TC)) {}
  //Write character to transmit register
  USART3->TDR = c;
}

//Transmits error message over UART
void print_error(void)
{
  transmit_char('e');
  transmit_char('r');
  transmit_char('r');
  transmit_char('o');
  transmit_char('r');
  transmit_char('\n');
  transmit_char('\r');
}

//Transmits prompt message over UART
void print_prompt(void)
{
  transmit_char('C');
  transmit_char('M');
  transmit_char('D');
  transmit_char('?');
  transmit_char('\n');
  transmit_char('\r');
}

//Transmits input command back over UART
void print_command(char led, char action)
{
  switch (led)
  {
    case 'r':
      transmit_char('r');
      transmit_char('e');
      transmit_char('d');
      break;
    case 'g':
      transmit_char('g');
      transmit_char('r');
      transmit_char('e');
      transmit_char('e');
      transmit_char('n');
      break;
    case 'b':
      transmit_char('b');
      transmit_char('l');
      transmit_char('u');
      transmit_char('e');
      break;
    case 'o':
      transmit_char('o');
      transmit_char('r');
      transmit_char('a');
      transmit_char('n');
      transmit_char('g');
      transmit_char('e');
      break;
    default:
      print_error();
  }

  transmit_char(' ');

  switch (action)
  {
    case '0':
      transmit_char('o');
      transmit_char('f');
      transmit_char('f');      
      break;
    case '1':
      transmit_char('o');
      transmit_char('n');
      break;
    case '2':
      transmit_char('t');
      transmit_char('o');
      transmit_char('g');
      transmit_char('g');
      transmit_char('l');
      transmit_char('e');
      break;
    default:
      print_error();
  }
  transmit_char('\n');
  transmit_char('\r');
}

//Turns on input LED
void led_on(char led)
{
  switch (led)
  {
    case 'r':
      GPIOC->ODR |= GPIO_ODR_6;
      break;
    case 'g':
      GPIOC->ODR |= GPIO_ODR_9;
      break;
    case 'b':
      GPIOC->ODR |= GPIO_ODR_7;
      break;
    case 'o':
      GPIOC->ODR |= GPIO_ODR_8;
      break;
    default:
      print_error();
  }
}

//Turns off input LED
void led_off(char led)
{
  switch (led)
  {
    case 'r':
      GPIOC->ODR &= ~GPIO_ODR_6;
      break;
    case 'g':
      GPIOC->ODR &= ~GPIO_ODR_9;
      break;
    case 'b':
      GPIOC->ODR &= ~GPIO_ODR_7;
      break;
    case 'o':
      GPIOC->ODR &= ~GPIO_ODR_8;
      break;
    default:
      print_error();
  }
}

//Toggles input LED
void led_toggle(char led)
{
  switch (led)
  {
    case 'r':
      GPIOC->ODR ^= GPIO_ODR_6;
      break;
    case 'g':
      GPIOC->ODR ^= GPIO_ODR_9;
      break;
    case 'b':
      GPIOC->ODR ^= GPIO_ODR_7;
      break;
    case 'o':
      GPIOC->ODR ^= GPIO_ODR_8;
      break;
    default:
      print_error();
  }
}

int main(void) 
{
  HAL_Init(); // Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); //Configure the system clock

  //Enable clock to GPIOC for LEDS
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; 
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  //Initialize all LEDs
  init_leds(); 

  //Set pin PB10 for USART TX
  GPIOB->MODER |= GPIO_MODER_MODER10_1; //Alternate function
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT_10; // Push-pull
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR10_0; //Low speed
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR10_0; //Pull up
  //Set pin PB11 for USART RX
  GPIOB->MODER |= GPIO_MODER_MODER11_1; //Alternate function
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT_11; // Push-pull
  GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR11_0; //Low speed
  GPIOB->PUPDR |= GPIO_PUPDR_PUPDR11_0; //Pull up
  GPIOB->AFR[1] &= 0xFFFF44FF;
  GPIOB->AFR[1] |= 0x00004400;

  //Enable clock to USART3
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  //Set word length to 8 bits
  USART3->CR1 &= ~USART_CR1_M1;
  USART3->CR1 &= ~USART_CR1_M0;
  //Set baud rate to 115200 bits/sec
  USART3->BRR = 0x45;
  //Set stop bits to 1
  USART3->CR2 &= ~USART_CR2_STOP_0;
  USART3->CR2 &= ~USART_CR2_STOP_1;
  //Set no parity
  USART3->CR1 &= ~USART_CR1_PCE;
  //Enable transmitter and receiver
  USART3->CR1 |= USART_CR1_TE;
  USART3->CR1 |= USART_CR1_RE;
  //Enable receive register not empty interrupt
  USART3->CR1 |= USART_CR1_RXNEIE;
  //Enable and set USART interrupt priority in NVIC
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(USART3_4_IRQn, 2);
  //Enable USART peripheral
  USART3->CR1 |= USART_CR1_UE;
  
  volatile char led;
  volatile char action;
  HAL_Delay(100);
  print_prompt();

  //Main loop
  while (1) 
  { 
    if (rec_data_flag == 1)
    {
      led = rec_data;
      rec_data_flag = 0;
      if (led != 'r' && led != 'g' && led != 'b' && led != 'o')
      {
        print_error();
        print_prompt();
      }
      else
      {
        while (rec_data_flag == 0) {HAL_Delay(1);}
        action = rec_data;
        rec_data_flag = 0;
        switch (action)
        {
          case '0':
            led_off(led);
            print_command(led,action);
            break;
          case '1':
            led_on(led);
            print_command(led,action);
            break;
          case '2':
            led_toggle(led);
            print_command(led,action);
            break;
          default:
            print_error();
        }
        print_prompt();
      }
    }
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
