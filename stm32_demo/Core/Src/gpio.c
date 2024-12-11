/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
// 设置端口寄存器1 高电平
  HAL_GPIO_WritePin(GPIOB,LED1_PIN|LED2_PIN|LED3_PIN,GPIO_PIN_SET);
  GPIO_InitTypeDef def;
  def.Pin = LED1_PIN|LED2_PIN|LED3_PIN;
  def.Mode = GPIO_MODE_OUTPUT_PP;
  def.Pull = GPIO_NOPULL;
  def.Speed = GPIO_SPEED_HIGH;
  //初始化端口输出模式
  HAL_GPIO_Init(GPIOB,&def);

  HAL_GPIO_WritePin(LED0_PIN_PORT,LED0_PIN,GPIO_PIN_SET);
  GPIO_InitTypeDef def2;
  def2.Pin = LED0_PIN;
  def2.Mode = GPIO_MODE_OUTPUT_PP;
  def2.Pull = GPIO_NOPULL;
  def2.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(LED0_PIN_PORT,&def2);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
