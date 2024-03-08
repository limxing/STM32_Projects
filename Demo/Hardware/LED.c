#include "stm32f10x.h"

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef def;
	def.GPIO_Mode = GPIO_Mode_Out_PP;
	def.GPIO_Pin = GPIO_Pin_13;
	def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&def);
	
}

void LED_ON(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}

void LED_OFF(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}

