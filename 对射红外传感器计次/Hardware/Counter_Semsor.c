#include "stm32f10x.h"                  // Device header

void CounterSemsor_Init(void)
{
	//开启引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//配置引脚
	GPIO_InitTypeDef def;
	def.GPIO_Mode = GPIO_Mode_IPU;
	def.GPIO_Pin = GPIO_Pin_14;
	def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&def);
	
//	配置AFIO外部中断引脚选择,选择14号引脚输入中断，对应EXTI 14号中断线路
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);
	
	//配置EXTI
	
	
}

