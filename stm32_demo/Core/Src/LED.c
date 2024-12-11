#include "LED.h"


void LED_On(LED led){
    HAL_GPIO_WritePin(GPIOB,led,GPIO_PIN_RESET);
}
void LED_Off(LED led)
{
    HAL_GPIO_WritePin(GPIOB,led,GPIO_PIN_SET);
}
void LED_Toogle(LED led)
{
    HAL_GPIO_TogglePin(GPIOB,led);
}
void LED_On_All(LED led[],uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        LED_On(led[i]);
    }
}
void LED_Off_All(LED led[],uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        LED_Off(led[i]);
    }
    
}
