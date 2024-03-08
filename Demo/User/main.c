#include "stm32f10x.h"                  // Device header
#include "Delay.h"


int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef def;
	def.GPIO_Mode = GPIO_Mode_Out_PP;
	def.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&def);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1); //低电平 
	GPIO_SetBits(GPIOA,GPIO_Pin_0); // 高電平
	while(1)
	{
//		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
//		Delay_ms(500);
//		GPIO_SetBits(GPIOA,GPIO_Pin_1);
//		GPIO_SetBits(GPIOA,GPIO_Pin_0);
//		Delay_ms(500);
		
		
	}
}
