/*
 * 003Butn_intp.c
 *
 *  Created on: 19-Dec-2024
 *      Author: SHASHANK S D
 */

#include <string.h>
#include "stm32f401xx.h"

#define BUTN_PRESSED 	ENABLE

void Delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLED,gpioButn;

	// for avoiding the garbage value collection
	memset(&gpioLED,0,sizeof(gpioLED));
	memset(&gpioButn,0,sizeof(gpioButn));

	// on board led
	gpioLED.pGPIOx = GPIOA;
	gpioLED.GPIO_Config.GPIO_PinNumber = GPIO_PIN5;
	gpioLED.GPIO_Config.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLED.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLED.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLED.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLED);

	// on board button
	gpioButn.pGPIOx = GPIOC;
	gpioButn.GPIO_Config.GPIO_PinNumber = GPIO_PIN13;
	gpioButn.GPIO_Config.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpioButn.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//gpioButn.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP; 	// here this is not required because external pull up already their & no external pull down
	gpioButn.GPIO_Config.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioButn);

	// IRQ Config
	GPIO_IRQPriorityConfig(IRQ_NO_EXT15_10, NVIC_IRQ_PR15);
	GPIO_IRQ_ITConfig(IRQ_NO_EXT15_10, ENABLE);


	while(1)
	{
	}


	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	Delay();    // for cancelling the debouncing effect
	GPIO_IRQHandling(GPIO_PIN13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN5);
}
