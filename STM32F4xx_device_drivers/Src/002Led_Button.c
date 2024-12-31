/*
 * 002Led_Button.c
 *
 *  Created on: 19-Dec-2024
 *      Author: SHASHANK S D
 */

#include "stm32f401xx.h"

#define BUTN_PRESSED 	ENABLE

void Delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLED,gpioButn;

	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_Config.GPIO_PinNumber = GPIO_PIN5;
	gpioLED.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLED.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	gpioButn.pGPIOx = GPIOC;
	gpioButn.GPIO_Config.GPIO_PinNumber = GPIO_PIN13;
	gpioButn.GPIO_Config.GPIO_PinMode = GPIO_MODE_IN;
	gpioButn.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//gpioButn.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP; 	// here this is not required because external pull up already their & no external pull down
	gpioButn.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioButn);

	while(1)
	{
		// here we are checking for the active low configuration, where button is pulled high before itself
		// so we are checking for the active low condition.
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN13) != BUTN_PRESSED)
		{
			Delay();			// delay for cancelling the debouncing effect
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
		}
	}


	return 0;
}
