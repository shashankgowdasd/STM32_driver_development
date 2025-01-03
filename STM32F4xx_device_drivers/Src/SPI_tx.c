/*
 * SPI_tx.c
 *
 *  Created on: 24-Dec-2024
 *      Author: SHASHANK S D
 */

#include "stm32f401xx.h"

/* SPI1
 * PA4 -> SPI1_NSS
 * PA5 -> SPI1_SCLK
 * PA6 -> SPI1_MISO
 * PA7 -> SPI1_MOSI
 * Alternate function mode : 5
 */

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	// configure the SPI gpio pins
	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_Config.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_Config.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPIPins.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// NSS
	//SPIPins.GPIO_Config.GPIO_PinNumber = GPIO_PIN4;
	//GPIO_Init(&SPIPins);

	// SCLK
	SPIPins.GPIO_Config.GPIO_PinNumber = GPIO_PIN5;
	GPIO_Init(&SPIPins);

	// MISO
	//SPIPins.GPIO_Config.GPIO_PinNumber = GPIO_PIN6;
	//GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_Config.GPIO_PinNumber = GPIO_PIN7;
	GPIO_Init(&SPIPins);
}

void SPI1_init(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;
	SPI1Handle.SPI_Config.SPI_busConfig = SPI_BUS_CNFG_FD;
	SPI1Handle.SPI_Config.SPI_devMode = SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPI_Config.SPI_Speed = SPI_SCLK_SPPED_DIV2;
	SPI1Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI1Handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI1Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_CLK;
	SPI1Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;

	SPI_Init(&SPI1Handle);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioButn;

	gpioButn.pGPIOx = GPIOC;
	gpioButn.GPIO_Config.GPIO_PinNumber = GPIO_PIN13;
	gpioButn.GPIO_Config.GPIO_PinMode = GPIO_MODE_IN;
	gpioButn.GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//gpioButn.GPIO_Config.GPIO_PinOPType = GPIO_OP_TYPE_PP; 	// here this is not required because external pull up already their & no external pull down
	gpioButn.GPIO_Config.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	//GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioButn);
}

void Delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	char user_data[] = "Hello World!";

	GPIO_ButtonInit();

	// this function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	// configure the SPI1
	SPI1_init();

	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN13));

		Delay();

		// this makes NSS signal internally high and avoids MODF error
		//SPI_SSIConfig(SPI1,ENABLE);

		// enable the SPI peripheral
		SPI_PeripheralControl(SPI1,ENABLE);

		// first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI1, &dataLen,1);

		// Send data
		SPI_SendData(SPI1,(uint8_t *)user_data,strlen(user_data));

		// lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

		// disable SPI peripharal
		SPI_PeripheralControl(SPI1, DISABLE);
	}

	return 0;
}
