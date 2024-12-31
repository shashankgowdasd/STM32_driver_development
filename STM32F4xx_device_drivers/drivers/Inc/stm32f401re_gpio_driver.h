/*
 * stm32f401re_gpio_driver.h
 *
 *  Created on: 18-Dec-2024
 *      Author: SHASHANK S D
 */

#ifndef INC_STM32F401RE_GPIO_DRIVER_H_
#define INC_STM32F401RE_GPIO_DRIVER_H_

#include "stm32f401xx.h"

/*
 * GPIOx pin configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_pinConfig_t;


/*
 * this is a handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;    /* this holds the base address of the GPIO port to which the pin belongs NOTE: p for notify that it is pointer*/
    GPIO_pinConfig_t GPIO_Config;
}GPIO_Handle_t;


/*
 * GPIO input mode types configurations
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4			/* interrupt for falling edge */
#define GPIO_MODE_IT_RT		5			/* interrupt for rising edge */
#define GPIO_MODE_IT_FTRT   6        	/* interrupt for both edges */

/*
 * GPIO output pin type
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1


/*
 * GPIO pin possible output speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


/*
 *	GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2


/*
 * GPIO pin numbers
 */
#define GPIO_PIN0			0
#define GPIO_PIN1 			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15


/********************************* API for GPIO Driver ************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

/*
 * Init and deint
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F401RE_GPIO_DRIVER_H_ */
