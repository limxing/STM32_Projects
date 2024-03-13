#include "stm32f10x.h"    // Device header

int16_t encoderCount;

void Encoder_Init(void)
{
		//开启引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	//配置引脚
	GPIO_InitTypeDef def;
	def.GPIO_Mode = GPIO_Mode_IPU;
	def.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	def.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&def);
	
//	配置AFIO外部中断引脚选择,选择14号引脚输入中断，对应EXTI 14号中断线路
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	
	//配置EXTI
//	EXTI_DeInit;重置EXTI
//	EXTI_Init; 初始化EXTI
//	EXTI_SturctInit; 给def设置默认值
//	GenerateSWInterrupt;软件触发外部中断
//	EXTI_GetFlagStatus;获取指定的标志位是否是1（主程序调用）
//	EXTI_ClearFlag;清除是1的标志位（主程序调用）
//	EXTI_GetITStatus;获取中断标志位是否是1（中断函数调用，中断函数使用上两个也没问题）
//	EXTI_ClearITPendingBit;清除中断挂起标志位（中断函数调用）
	EXTI_InitTypeDef EXTI_Def;
	EXTI_Def.EXTI_Line = EXTI_Line0 | EXTI_Line1;
	EXTI_Def.EXTI_LineCmd = ENABLE;
	EXTI_Def.EXTI_Mode = EXTI_Mode_Interrupt;//中断模式还是事件模式
	EXTI_Def.EXTI_Trigger = EXTI_Trigger_Falling;//触发方式，上升沿 下降沿
	EXTI_Init(&EXTI_Def);
	
	//配置NVIC 内核外设
//	NVIC_PriorityGroupConfig 中断分组
//	NVIC_Init
//  配置中断分组，参数是分组 NVIC_PriorityGroup_(0-4)，全局参数，配置一次即可，可以放在主函数开始，相当于初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  配置决定中断的抢占和响应优先级，
	NVIC_InitTypeDef NVIC_Init_Def;
	NVIC_Init_Def.NVIC_IRQChannel = EXTI0_IRQn;//选择中断通道 选择对应单片机型号里面的，也要对应上述引脚线路
	NVIC_Init_Def.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init_Def.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，根据分组表中参数的范围配置NVIC_PriorityGroup_2 抢占0-3
	NVIC_Init_Def.NVIC_IRQChannelSubPriority = 1;//响应优先级 0-3
	NVIC_Init(&NVIC_Init_Def);
	
	
	NVIC_Init_Def.NVIC_IRQChannel = EXTI1_IRQn;//选择中断通道 选择对应单片机型号里面的，也要对应上述引脚线路
	NVIC_Init_Def.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init_Def.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级，根据分组表中参数的范围配置NVIC_PriorityGroup_2 抢占0-3
	NVIC_Init_Def.NVIC_IRQChannelSubPriority = 2;//响应优先级 0-3
	NVIC_Init(&NVIC_Init_Def);
}
 int16_t GetNum(void){

	return encoderCount;
}
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) == 0)
		{
		  encoderCount ++;
			//中断中不能做耗时操作，也不要操作显示屏
		}
		EXTI_ClearITPendingBit(EXTI_Line0); 
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) == SET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 0)
		{
		  encoderCount --;
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
