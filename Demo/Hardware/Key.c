#include "stm32f10x.h"                  // Device header
#include "Delay.h"


void KEY_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef def;
	def.GPIO_Mode = GPIO_Mode_IPU;
	def.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_10;
	def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&def);
}

uint8_t GET_Num(void)
{
	uint8_t num = 0;
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 0);
		Delay_ms(20);
		num = 1;
	}
	if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 0)
	{
		Delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 0);
		Delay_ms(20);
		num = 2;
	}
	return  num;
}
