
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "rgb_out.h"
#include "m_driver_stm32f1xx.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void set_pwm(uint16_t level);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t default_fb[128*16] = {
  24,0,0,0,0,0,0,0,0,24,0,0,24,0,0,0,24,0,24,0,24,0,0,0,0,0,0,24,0,0,24,0,0,24,0,0,24,0,0,24,0,0,0,24,0,24,0,0,0,24,0,0,24,0,0,24,0,24,0,0,0,24,0,0,40,0,0,0,0,0,40,0,0,0,0,0,40,0,0,40,0,0,0,40,0,0,40,0,0,40,0,0,0,40,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,24,24,28,28,28,28,28,28,28,12,13,13,13,13,8,8,0,
  3,27,27,27,27,24,0,0,24,24,24,0,27,3,0,0,27,3,24,0,0,24,0,24,24,24,24,0,0,0,0,24,24,0,0,24,24,24,0,24,0,0,0,24,0,0,24,24,24,3,0,0,0,24,24,0,0,0,24,24,24,0,0,0,40,0,0,0,0,0,40,0,0,0,0,40,40,40,0,0,40,40,40,0,0,40,40,40,0,0,40,40,40,40,0,40,40,40,0,0,0,0,0,0,0,0,0,0,0,0,0,4,4,28,28,28,28,28,28,28,29,13,13,13,13,5,5,0,
  3,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,4,4,4,4,4,7,7,7,7,7,7,5,5,5,5,5,0,
  3,0,0,0,0,0,0,3,3,3,0,0,0,3,0,0,0,3,0,0,0,3,3,3,0,0,3,0,0,0,3,0,0,0,0,0,0,3,0,0,0,3,0,0,3,3,3,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,4,4,5,7,7,7,7,7,7,7,7,7,7,5,5,5,5,
  3,3,3,3,0,0,3,0,0,0,3,0,0,3,0,0,0,3,0,0,3,0,0,0,3,0,3,0,0,0,3,0,0,0,0,0,0,3,0,0,0,3,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,5,5,7,5,5,5,5,5,5,5,5,5,5,7,5,5,5,
  3,0,0,0,0,0,3,0,0,0,3,0,0,3,0,0,0,3,0,0,3,0,0,0,3,0,3,0,3,0,3,0,0,0,0,0,0,3,0,0,0,3,0,0,3,3,3,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,40,40,0,0,0,0,40,40,40,40,40,40,40,40,0,0,0,0,40,40,40,40,0,0,0,0,0,0,0,0,0,0,0,0,40,40,40,40,0,0,0,0,40,40,45,45,45,47,45,45,5,5,45,45,45,45,47,45,47,45,5,5,
  3,0,0,0,0,0,3,0,0,0,3,0,0,3,0,0,0,3,0,0,3,0,0,0,3,0,3,0,3,0,3,0,0,0,0,0,0,3,0,0,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,40,40,0,0,0,0,40,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,5,7,7,5,45,1,7,7,7,7,5,5,45,7,7,5,5,
  3,0,0,0,0,0,0,3,3,3,0,0,3,3,3,0,3,3,3,0,0,3,3,3,0,0,0,3,0,3,0,0,0,0,0,0,0,0,3,3,3,0,0,3,3,3,3,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,40,40,0,0,0,0,0,0,0,0,40,40,0,0,0,0,40,40,0,0,0,0,5,1,7,7,1,41,7,5,1,1,5,7,5,45,7,7,5,5,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,24,24,0,0,0,0,0,0,0,0,0,0,0,0,0,40,40,0,0,0,0,0,0,0,0,0,40,40,0,0,0,0,0,0,0,0,40,40,0,0,0,0,40,40,0,0,0,0,1,1,7,47,41,1,7,1,1,1,1,47,41,5,7,7,5,5,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,24,24,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,40,0,0,0,0,40,40,0,0,0,0,1,1,7,7,1,1,7,1,1,1,1,7,1,1,7,7,5,5,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,24,24,40,45,45,40,0,0,0,5,5,0,0,0,40,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,40,40,40,40,40,40,0,0,0,0,1,1,47,7,1,1,7,1,1,1,1,7,1,41,7,7,5,5,
  0,3,3,3,3,3,0,0,3,3,3,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,3,3,3,0,24,24,5,0,0,5,40,40,5,0,0,5,40,40,5,0,0,0,0,0,0,0,0,40,45,0,0,0,0,0,0,0,0,5,5,0,0,0,0,0,0,0,40,40,0,0,0,0,41,41,7,7,1,1,1,7,7,7,7,1,1,1,47,47,5,5,
  3,0,0,0,0,0,3,0,0,3,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,3,0,0,0,3,24,24,5,0,0,0,40,40,5,0,0,0,40,40,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,40,40,0,0,0,0,41,41,1,7,1,1,1,1,1,1,1,1,1,1,47,41,5,5,
  3,0,3,3,3,0,3,0,0,3,0,0,3,0,3,3,0,0,3,0,0,3,0,0,3,3,3,0,0,3,3,3,0,0,0,3,3,0,0,3,0,3,3,0,0,0,3,3,3,0,0,3,3,3,0,0,0,3,0,0,0,3,24,24,5,0,0,0,40,40,5,0,0,0,40,45,5,0,0,0,5,5,5,0,40,5,5,0,0,0,5,5,45,40,40,40,5,0,0,0,0,0,0,40,0,0,0,0,0,0,41,41,3,7,3,3,3,3,41,41,1,1,1,1,47,41,1,5,
  3,0,3,0,3,0,3,0,0,3,0,0,3,3,0,0,3,0,3,0,3,0,0,3,0,0,0,0,0,0,3,0,0,0,0,0,3,0,0,3,3,0,0,3,0,3,0,0,0,0,0,0,3,0,0,0,0,3,0,0,0,3,0,3,5,5,0,0,0,5,5,5,0,0,40,40,5,0,0,5,0,0,0,0,40,0,5,0,0,0,0,0,0,5,40,40,5,0,0,0,0,0,0,0,0,0,0,0,0,0,41,43,3,3,7,7,7,7,7,7,7,7,7,7,1,1,1,5,
  3,0,3,3,3,3,3,0,0,3,0,0,3,0,0,0,3,0,3,3,0,0,0,0,3,3,3,0,0,0,3,0,0,0,0,0,3,0,0,3,0,0,0,3,0,3,0,0,0,0,0,0,3,0,0,0,0,3,0,0,0,3,0,0,45,40,40,40,0,0,5,0,0,0,40,40,5,0,0,5,0,0,40,40,40,40,45,40,40,40,5,5,5,5,40,40,5,0,0,0,40,40,40,0,0,0,0,0,0,0,40,43,3,3,3,3,7,7,7,7,47,47,41,41,1,1,1,0
};
uint8_t *fb = default_fb;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  RGB_A(OUT_OFF);
  RGB_B(OUT_OFF);
  RGB_C(OUT_OFF);
  RGB_D(OUT_OFF);
  
  RGB_R1(OUT_OFF);
  RGB_G1(OUT_OFF);
  RGB_B1(OUT_OFF);

  RGB_R2(OUT_OFF);
  RGB_G2(OUT_OFF);
  RGB_B2(OUT_OFF);

  RGB_CLK(OUT_OFF);
  //RGB_OE(OUT_OFF);
  RGB_LAT(OUT_OFF);

  TEST(OUT_ON);

  HAL_Delay(1000);
  TEST(OUT_OFF);
  oe_pulse(128);
  uint8_t pulse = 1;
  
  
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


    /*for(uint8_t row = 0; row < NROWS/2; row++){
      for(uint8_t col = 0; col < NCOLS; col++){
        uint8_t pixel = fb[col + row * NCOLS];
        RGB_R1((pixel & 1) ? OUT_ON : OUT_OFF);
        RGB_G1((pixel & 2) ? OUT_ON : OUT_OFF);
        RGB_B1((pixel & 4) ? OUT_ON : OUT_OFF);
        RGB_R2((pixel & 8) ? OUT_ON : OUT_OFF);
        RGB_G2((pixel & 16) ? OUT_ON : OUT_OFF);
        RGB_B2((pixel & 32) ? OUT_ON : OUT_OFF);

        RGB_CLK(OUT_ON);
        RGB_CLK(OUT_OFF);
      }

      RGB_OE(OUT_OFF);
      RGB_A((row & 0x08) ? OUT_ON : OUT_OFF);
      RGB_B((row & 0x04) ? OUT_ON : OUT_OFF);
      RGB_C((row & 0x02) ? OUT_ON : OUT_OFF);
      RGB_D((row & 0x01) ? OUT_ON : OUT_OFF);
      RGB_LAT(OUT_OFF);
      RGB_LAT(OUT_ON);
      RGB_OE(OUT_ON);
      //HAL_TIM_PWM_Start(htim1,TIM_CHANNEL_1);
      for(uint32_t t = 0; t< 100; t++)
        asm("NOP");
    }*/
    //set_pwm(level);
    //TEST((level%4096 == 0)?OUT_OFF:OUT_ON);
    //if (level > 60000)
      //HAL_TIM_OnePulse_Start(&htim1, TIM_CHANNEL_1);
    //HAL_Delay(200);
    //level+=2048;
      //while(HAL_TIM_PWM_GetState(&htim1) == HAL_TIM_STATE_BUSY)
      //  asm("NOP");
      //TEST(OUT_ON);
      if (pulse){
        pulse = 0;
        oe_pulse_wait();
        TEST(OUT_ON);
        oe_pulse(16);
        oe_pulse_wait();
        TEST(OUT_OFF);
      }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2880-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 32768;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A_Pin|B_Pin|C_Pin|D_Pin 
                          |LAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R1_Pin|G1_Pin|B1_Pin|CLK_Pin 
                          |R2_Pin|G2_Pin|B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin D_Pin 
                           LAT_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D_Pin 
                          |LAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R1_Pin G1_Pin B1_Pin CLK_Pin 
                           R2_Pin G2_Pin B2_Pin */
  GPIO_InitStruct.Pin = R1_Pin|G1_Pin|B1_Pin|CLK_Pin 
                          |R2_Pin|G2_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void set_pwm(uint16_t level){
  TIM_OC_InitTypeDef sConfigOC;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = level;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_OnePulse_Start(&htim1, TIM_CHANNEL_1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
