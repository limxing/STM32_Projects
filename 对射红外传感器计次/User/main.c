#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Counter_Semsor.h"
#include "Encoder.h"

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
//	CounterSemsor_Init();
//	Encoder_Init();

	
	
	while (1)
	{
		OLED_ShowString(1,1,"Count:");
//		OLED_ShowSignedNum(1,7,GetNum(),5);
		
	}
}
