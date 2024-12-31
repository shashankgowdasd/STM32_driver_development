/*
 * stm32f401re_gpio_driver.c
 *
 *  Created on: 18-Dec-2024
 *      Author: SHASHANK S D
 */

#include "stm32f401re_gpio_driver.h"

/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp =0;

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);


	//	1. configure the mode of pin
	if(pGPIOHandle->GPIO_Config.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// here the multiply of 2 with the pin number will be setting the mode,because 2 bits for each reg
		temp = (pGPIOHandle->GPIO_Config.GPIO_PinMode << (2* pGPIOHandle->GPIO_Config.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);   // clearing the bit
		pGPIOHandle->pGPIOx->MODER |= temp;	// setting the bits
		temp = 0;
	}
	else
	{
		// interrupt mode
		if(pGPIOHandle->GPIO_Config.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. configure the falling edge strigger status register(FTSR)
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber); // clearing the bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);	// setting the bit
		}
		else if(pGPIOHandle->GPIO_Config.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber); // setting the bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);	// clearing the bit
		}
		else if(pGPIOHandle->GPIO_Config.GPIO_PinMode == GPIO_MODE_IT_FTRT)
		{
			// 1. configure both fallig and rising edge
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber); // clearing the bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);	// setting the bit
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_Config.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_Config.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);


		// 3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	}


	// 2. configure the speed
	temp =0;
	temp = (pGPIOHandle->GPIO_Config.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= (0x03 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. configure the output type
	temp =0;
	temp = (pGPIOHandle->GPIO_Config.GPIO_PinOPType  << pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= (0x1 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 4. configure the pullup and pulldown
	temp =0;
	temp = (pGPIOHandle->GPIO_Config.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_Config.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= (0x03 << pGPIOHandle->GPIO_Config.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 5. configure the alternate functionality
	temp =0;
	if(pGPIOHandle->GPIO_Config.GPIO_PinAltFunMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1,temp2;

		// as the alternate function mode is having the two array elements high/low
		// here temp1 is used for selecting AFRH/AFRL
		// here temp2 is used for selecting the bytes in the AFRH or AFRL

		temp1 = pGPIOHandle->GPIO_Config.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_Config.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_Config.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	// to get the particualr value of the IDR register right shifted and masked to 1, for getting the value of the bit
	value = pGPIOx->IDR >> PinNumber & 0x00000001;
	return value;
}

/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	// here we are return the entire port so we send IDR register itself
	value = pGPIOx->IDR;
	return value;
}

/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
	// write the directly the value to the port
	pGPIOx->ODR = Value;
}

/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <=31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)  // 32 to 63
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
		{
			// program ISER2 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 32));
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
			// program ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)  // 32 to 63
		{
			// program ISER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96) // 64 to 95
		{
			// program ISER2 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}
	}
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// 1. first lets find out the ipr reg
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amnt = (8 * iprx_section ) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx)  |= ( IRQPriority << shift_amnt );
}


/*****************************************************************************
* @fn 			- GPIO_PeriCLockControl
*
* @brief		- This function enable or disables peripheral closk for the given gpio
*
* @param1		- base address of the gpio peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// 1. clear the exti pc register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		// here we are clearing by setting the particular pin number of the external interrupt
		EXTI->PR |= (1<< PinNumber);
	}
}
