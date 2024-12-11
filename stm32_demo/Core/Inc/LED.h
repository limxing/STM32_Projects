#ifndef __LED_H
#define __LED_H

#include "gpio.h"

#define LED uint16_t

void LED_On(LED led);
void LED_Off(LED led);
void LED_Toogle(LED led);
void LED_On_All(LED led[],uint8_t count);
void LED_Off_All(LED led[],uint8_t count);


#endif