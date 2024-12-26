/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LED.h"
#include "systick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LED leds[] = {LED1_PIN,LED2_PIN,LED3_PIN};

 
  // Systick_Init();
  while (1)
  {
    
  }
  /* USER CODE END 3 */
}
void TIM2_Main()
{
uint8_t dutyCycle = 0;
  uint8_t dir = 0;
  while (1)
  {
    if (dir == 0) 
    {
      dutyCycle += 1;
      if (dutyCycle >= 99)
      {
        dir = 1;
      }
      
    }else{
      dutyCycle -= 1;
      if (dutyCycle <= 1)
      {
        dir = 0;
      }
      
    }
    TIM2->CCR2 = dutyCycle;
    HAL_Delay(10);
  }
}

void TIM2_Init()
{
   // 开启时钟，配置GPIO A1工作模式
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  GPIOA->CRL |= GPIO_CRL_MODE1;
  GPIOA->CRL |= GPIO_CRL_CNF1_1;
  GPIOA->CRL |= GPIO_CRL_CNF1_0;
  // 定时器设置
  // 预分频值
  TIM2->PSC = 7199;
  // 重装载值,每隔10ms溢出一次
  TIM2->ARR = 99;
  //计数方向，给0 向上计数
  TIM2->CR1 &= ~TIM_CR1_DIR;
  //设置通道2的CCR值 （TIM2_CH2）,初始值多少都可以，占空比
  TIM2->CCR2 = 50;
  // 通道2的工作模式，00 输出模式 
  TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
  // 配置通道2为PWM1模式 110 (给CCMR1寄存器某一位设置0还是1，|= 是设置1 &=~是设置0，这里需要配置三位，右边是第0位)
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_2;
  TIM2->CCMR1 |= TIM_CCMR1_OC2M_1;
  TIM2->CCMR1 &= ~TIM_CCMR1_OC2M_0;
  // 配置CCER捕获/比较使能寄存器
  TIM2->CCER |= TIM_CCER_CC2E;

// 打开计时器
  TIM2->CR1 |= TIM_CR1_CEN;
  // 关闭计数器
  // TIM2->CR1 &= ~TIM_CR1_CEN;
}

void TIM6_Init()
{
    // 定时器使能 
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  //设置预分频值7199，做7200分频  得到10000Hz
  TIM4->PSC = 7199;
  // 设置自动重装载值9999，表示计数10000次产生一个UEV
  TIM4->ARR = 9999;
  // 更新中断使能
  TIM4->DIER |= TIM_DIER_UIE;
  // 配置NVIC
  NVIC_SetPriorityGrouping(3);
  NVIC_SetPriority(TIM4_IRQn,2);
  NVIC_EnableIRQ(TIM4_IRQn);

  // 开启定时器
  TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM6_IRQnHandler()
{
  // 清除中断标志
  TIM4->SR &= ~TIM_SR_UIF;
  HAL_GPIO_TogglePin(LED0_PIN_PORT,LED0_PIN);
  LED_Toogle(LED1_PIN);
}

// uint16_t sysCount = 0;
// // 系统定时器中断回调
// void HAL_IncTick2()
// {
//   sysCount++;
//   if (sysCount == 1000)
//   {
//     HAL_GPIO_TogglePin(LED0_PIN_PORT,LED0_PIN);
//     LED_Toogle(LED1_PIN);
//     sysCount = 0;
//   }
// }


void USART_DEMO()
{
    //中断DEMO
  // interruptDemo();


  // HAL_GPIO_WritePin(LED0_PIN_PORT,LED0_PIN,GPIO_PIN_RESET);
  
  // LED_On(LED1_PIN);
  // 1.配置时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();


  // 2.GPIO 工作模式 PA9 复用推挽mode 10 输出 11， 
  // // 配置MODE
  // GPIOA->CRH |= GPIO_CRH_MODE9; // 00 11 00 00
  // // 配置CNF 高位设置1 低位设置0
  // GPIOA->CRH |= GPIO_CRH_CNF9_1; //10 00 00 00
  // GPIOA->CRH &= ~GPIO_CRH_CNF9_0;//01 00 00 00
  // // PA10 浮空mode 01 输入 00 ,MODE10就是寄存器对应PIN
  // GPIOA->CRH &= ~GPIO_CRH_MODE10; //[00 11] [00 00] [00 00] 
  // GPIOA->CRH &= ~GPIO_CRH_MODE10_1;// 0010 0000 0000
  // GPIOA->CRH |= GPIO_CRH_MODE10_0;// 0001 0000 0000
  GPIO_InitTypeDef def;
  def.Mode = GPIO_MODE_AF_PP;
  def.Pin = GPIO_PIN_9;
  def.Pull = GPIO_NOPULL;
  def.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA,&def);
  GPIO_InitTypeDef def2;
  def2.Mode = GPIO_MODE_INPUT;
  def2.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA,&def2);
// 3.配置串口

  USART1->BRR = 0x271;
  USART1->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
  USART1->CR1 &= ~USART_CR1_M;
  USART1->CR1 &= ~USART_CR1_PCE;
  USART1->CR2 &= ~USART_CR2_STOP;



    //设置定时器HAL_Delay的优先级最高，否则由于优先级太低，中断卡死，比我们设置的中断高即可
  HAL_NVIC_SetPriority(SysTick_IRQn,2,0);

  // 开启中断使能
  USART1->CR1 |= USART_CR1_IDLEIE;
  USART1->CR1 |= USART_CR1_RXNEIE;
  NVIC_SetPriorityGrouping(3);
  NVIC_SetPriority(USART1_IRQn,3);
  NVIC_EnableIRQ(USART1_IRQn);
}


uint8_t buffer[100] = {};
uint8_t size = 0;

void USART1_IRQHandler()
{
  if (USART1->SR & USART_SR_RXNE)
  {
    // 接收完成一个字符
    buffer[size] = USART1->DR;
    size++;
  }else if (USART1->SR & USART_SR_IDLE)
  { 
    // 字符串整体接收完成
    // 清除IDLE
    USART1->DR;
    USART_SendString(buffer,size);
    size = 0;
  }
  
  
}
void USART_SendString(uint8_t *str, uint8_t *size)
{
  for (uint8_t i = 0; i < size; i++)
  {
    Usart_SendChar(str[i]);
  }
  
}
void Usart_SendChar(uint8_t ch)
{
  while ((USART1->SR & USART_SR_TXE) == 0)
  { 
  }
  USART1->DR = ch; 
  
}

uint8_t USART_ReceiveChar()
{
  while ((USART1->SR & USART_SR_RXNE) == 0) 
  {
  }
  return USART1->DR;
  
}
/// @brief 中断功能
void interruptDemo()
{
      // 按键是B0 灯是B12
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    
    //灯的初始化
    GPIO_InitTypeDef def;
    def.Mode = GPIO_MODE_OUTPUT_PP;
    def.Pin = GPIO_PIN_13;
    def.Pull = GPIO_NOPULL;
    def.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOC,&def);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
    
    //按键的初始化，按键一头连接VSS 一头连接PB0，需要设置PB0为低电平，配一个下拉输入
    
    // //设置为输入模式
    // GPIOB->CRL &= ~GPIO_CRL_MODE0;
    // //CNF 是 10 上下拉模式，具体上还是下，给一个低电平则为下拉模式
    // GPIOB->CRL |= GPIO_CRL_CNF0_1;
    // GPIOB->CRL &= ~GPIO_CRL_CNF0_0;
    // //给一个低电平则为下拉模式
    // GPIOB->ODR &= ~GPIO_ODR_ODR0;

    GPIO_InitTypeDef def2;
    def2.Mode = GPIO_MODE_IT_RISING;//外部中断上升沿模式
    def2.Pin = GPIO_PIN_0;
    def2.Pull = GPIO_PULLDOWN;//下拉输入 默认输出低电平 
    // def2.Speed = GPIO_SPEED_HIGH;//不需要设置
    HAL_GPIO_Init(GPIOB,&def2);
    // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);//可以不设置，下拉输入就是低电平了，不用设置

    //配置按键的外部中断寄存器，AFIO配置引脚的复用选择
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB;
    //配置EXTI
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->IMR |= EXTI_IMR_MR0;
    // //配置NVIC
    // NVIC_SetPriorityGrouping(3);//3 全部都是抢占优先级
    // NVIC_SetPriority(EXTI0_IRQn,3);
    // NVIC_EnableIRQ(EXTI0_IRQn);
    // // EXTI0_IRQHandler (配置回调函数，按下按键回调)
    HAL_NVIC_SetPriority(EXTI0_IRQn,3,0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
/// @brief 调用HAL_GPIO_EXTI_IRQHandler的回调
/// @param GPIO_Pin 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    HAL_Delay(20);
    //判断依然保持高电平 就翻转LED
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_SET)
    // if ((GPIOB->IDR & GPIO_IDR_IDR0) != 0)
    {
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    }else{
      
    }
  }
  
}

/// @brief 中断回调
/// HAL_GPIO_EXTI_IRQHandler 方法内自动清除中断请求
void EXTI0_IRQHandler(void)
{

   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    //清除中断挂起标志位
    // EXTI->PR |= EXTI_PR_PR0;
    // __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); 
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
