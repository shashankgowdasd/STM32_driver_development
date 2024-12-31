/*
 * 001LedToggle.c
 *
 *  Created on: 18-Dec-2024
 *      Author: SHASHANK S D
 */


#include "stm32f401xx.h"

void Delay(void)
{
	// actually give the delay of ~200ms for 16 MHZ frequency
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void)
{
	GPIO_Handle_t gpioLED ;
	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_Config.GPIO_PinNumber = GPIO_PIN5;
	gpioLED.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLED.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
		Delay();
	}

	return 0;
}
