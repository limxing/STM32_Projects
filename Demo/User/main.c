#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include <stdio.h>
#include "LED.h"
#include "KEY.h"

#define ABC 666

typedef struct{
	int8_t a;
	int16_t b;
} ABCD;

void test(ABCD* abcd)
	{
		abcd->a = 10;
		abcd->b = 100;
		printf("a=%d b=%d",abcd->a,abcd->b);
	}

int main(void)
{
	KEY_Init();
	LED_Init();
	LED_OFF();
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef def;
	def.GPIO_Mode = GPIO_Mode_Out_PP;
	def.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&def);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1); //低电平 
	GPIO_SetBits(GPIOA,GPIO_Pin_0); // 高電平
//	板载发光二极管
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//	def.GPIO_Pin = GPIO_Pin_13;
//	GPIO_Init(GPIOC,&def);
//	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	uint8_t keyNum = 0;
	while(1)
	{
//		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
//		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
//		Delay_ms(500);
//		GPIO_SetBits(GPIOA,GPIO_Pin_1);
//		GPIO_SetBits(GPIOA,GPIO_Pin_0);
//		Delay_ms(500);
		
//		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//		Delay_ms(500);
//		GPIO_SetBits(GPIOC,GPIO_Pin_13);
//		Delay_ms(500);
		keyNum = GET_Num();
		if (keyNum == 1)
		{
			LED_ON();
		}
//		else
//		{
//			LED_OFF();
//		}
		
	}
}
