/*
 * stm32f401xx.h
 *
 *  Created on: Dec 17, 2024
 *      Author: SHASHANK S D
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdio.h>
#include <stdint.h>


#define __vo      volatile

/****************************** processor specific details *********************************/

/*
 *  ARM Cortex Processor NVIC ISERx register Address for enabling the interrupt at the NVIC
 */

#define NVIC_ISER0		( (__vo uint32_t *)0xE000E100 )
#define NVIC_ISER1		( (__vo uint32_t *)0xE000E104 )
#define NVIC_ISER2		( (__vo uint32_t *)0xE000E108 )
#define NVIC_ISER3		( (__vo uint32_t *)0xE000E10C )


/*
 *  ARM Cortex Processor NVIC ICERx register Address for clearing the interrupt on NVIC
 */

#define NVIC_ICER0		( (__vo uint32_t *)0xE000E180 )
#define NVIC_ICER1		( (__vo uint32_t *)0xE000E184 )
#define NVIC_ICER2		( (__vo uint32_t *)0xE000E188 )
#define NVIC_ICER3		( (__vo uint32_t *)0xE000E18C )

/*
 * ARM Cortex Processor Priority Regsiter Address Calculation
 */
#define NVIC_PR_BASE_ADDR	((__vo uint32_t *)0xE000E400)

/*
 * NVIC IRQ Priority numbering for all priority levels
 */
#define NVIC_IRQ_PR0	0
#define NVIC_IRQ_PR1	1
#define NVIC_IRQ_PR2	2
#define NVIC_IRQ_PR3	3
#define NVIC_IRQ_PR4	4
#define NVIC_IRQ_PR5	5
#define NVIC_IRQ_PR6	6
#define NVIC_IRQ_PR7	7
#define NVIC_IRQ_PR8	8
#define NVIC_IRQ_PR9	9
#define NVIC_IRQ_PR10	10
#define NVIC_IRQ_PR11	11
#define NVIC_IRQ_PR12	12
#define NVIC_IRQ_PR13	13
#define NVIC_IRQ_PR14	14
#define NVIC_IRQ_PR15	15



#define NO_PR_BITS_IMPLEMENTED		4


/******************************************************************************************/


#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x0x200000U  // 64 KB
#define ROM					0x1FFF0000U  // system memory/ROM
#define SRAM				SRAM1_BASEADDR

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
 * address of peripherals which are hanging on the AHB1 bus
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/*
 * address of peripherals which are hanging on the APB1 bus
 */
#define I2C1_BASEADDR      (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR	   (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR	   (APB1PERIPH_BASE + 0x5C00)
#define USART2_BASEADDR	   (APB1PERIPH_BASE + 0x4400)
#define SPI2_BASEADDR	   (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR	   (APB1PERIPH_BASE + 0x3C00)

/*
 * address of peripherals which are hanging on the APB2 bus
 */
#define USART1_BASEADDR	   (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR    (APB2PERIPH_BASE + 0x1400)
#define SPI1_BASEADDR	   (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR	   (APB2PERIPH_BASE + 0x3400)
#define EXTI_BASEADDR	   (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR	   (APB2PERIPH_BASE + 0x3800)


/********************* RCC STructure definition ***********************************/

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t RESERVED8[2];
	__vo uint32_t DCKCF;
}RCC_RegDef_t;


/********************* EXTI peripheral Structure definition (APB2)***********************************/

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/********************* SYSCFG peripheral Structure definition (APB2)***********************************/

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;


/****************** *GPIO peripheral  register definition structures(AHB1) *********************/

typedef struct
{
	__vo uint32_t MODER;			 /* Address offset: 0x00 */
	__vo uint32_t OTYPER;            /* Address offset: 0x04 */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;


/******************* SPI peripheral structure definition(APB2) **************************/

typedef struct
{
	__vo uint8_t SPI_CR1;
	__vo uint8_t SPI_CR2;  // this is explicitly mentioned, no reg availble for 0x04
	__vo uint8_t SPI_SR;
	__vo uint8_t SPI_DR;
	__vo uint8_t SPI_CRCPR;
	__vo uint8_t SPI_RXCRCR;
	__vo uint8_t SPI_TXCRCR;
	__vo uint8_t SPI_I2SCFGR;
	__vo uint8_t SPI_I2SPR;
}SPI_RegDef_t;


/**************** peripheral definitions ******************************************/
#define GPIOA 		((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t *)GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)


/* SPI Macros */
#define SPI1		((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t *) SPI4_BASEADDR)

/*
 * GPIOx clock enable macros
 */

#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |= (1<<7))


/*
 * GPIOx clock disable macros
 */

#define GPIOA_PCLK_DI()    RCC->AHB1ENR &= ~(1<<0)
#define GPIOB_PCLK_DI()    RCC->AHB1ENR &= ~(1<<1)
#define GPIOC_PCLK_DI()    RCC->AHB1ENR &= ~(1<<2)
#define GPIOD_PCLK_DI()    RCC->AHB1ENR &= ~(1<<3)
#define GPIOE_PCLK_DI()    RCC->AHB1ENR &= ~(1<<4)
#define GPIOH_PCLK_DI()    RCC->AHB1ENR &= ~(1<<7)

/*
 * GPIOx reset macros
 */
#define GPIOA_REG_RESET()  do{ RCC->AHB1RSTR |= (1<<0); RCC->AHB1RSTR &= ~(1<<0); }while(0)
#define GPIOB_REG_RESET()  do{ RCC->AHB1RSTR |= (1<<1); RCC->AHB1RSTR &= ~(1<<0); }while(0)
#define GPIOC_REG_RESET()  do{ RCC->AHB1RSTR |= (1<<2); RCC->AHB1RSTR &= ~(1<<0); }while(0)
#define GPIOD_REG_RESET()  do{ RCC->AHB1RSTR |= (1<<3); RCC->AHB1RSTR &= ~(1<<0); }while(0)
#define GPIOE_REG_RESET()  do{ RCC->AHB1RSTR |= (1<<4); RCC->AHB1RSTR &= ~(1<<0); }while(0)
#define GPIOH_REG_RESET()  do{ RCC->AHB1RSTR |= (1<<7); RCC->AHB1RSTR &= ~(1<<0); }while(0)


/*
 * SPIx reset macros
 */
#define SPI1_REG_RESET()   do{ RCC->APB1RSTR |= (1<<12); RCC->APB1RSTR &= ~(1<<12);}while(0)
#define SPI2_REG_RESET()   do{ RCC->APB1RSTR |= (1<<15); RCC->APB1RSTR &= ~(1<<15);}while(0)
#define SPI3_REG_RESET()   do{ RCC->APB1RSTR |= (1<<14); RCC->APB1RSTR &= ~(1<<14);}while(0)
#define SPI4_REG_RESET()   do{ RCC->APB1RSTR |= (1<<13); RCC->APB1RSTR &= ~(1<<13);}while(0)

/**************** CLock enable macros for Comm. peripherals *********************/

/* I2C */
#define I2C1_PCLK_EN()     RCC->APB1ENR |= (1<<21)
#define I2C2_PCLK_EN()	   RCC->APB1ENR |= (1<<22)
#define I2C3_PCLK_EN()	   RCC->APB1ENR |= (1<<23)

/* SPI */
#define SPI1_PCLK_EN()	   RCC->APB2ENR |= (1<<12)
#define SPI2_PCLK_EN()	   RCC->APB1ENR |= (1<<15)
#define SPI3_PCLK_EN()	   RCC->APB1ENR |= (1<<14)
#define SPI4_PCLK_EN()	   RCC->APB2ENR |= (1<<13)

/* USART */
#define USART2_PCLK_EN()	   RCC->APB1ENR |= (1<<17)
#define USART6_PCLK_EN()	   RCC->APB2ENR |= (1<<5)
#define USART1_PCLK_EN()	   RCC->APB2ENR |= (1<<4)


/*
 * System config peripheral clock enable
 */
#define SYSCFG_PCLK_EN()	   RCC->APB2ENR |= (1<<14)


/**************** CLock disable macros for Comm. peripherals *********************/
/* I2C */
#define I2C1_PCLK_DI()     RCC->APB1ENR &= ~(1<<21)
#define I2C2_PCLK_DI()	   RCC->APB1ENR &= ~(1<<22)
#define I2C3_PCLK_DI()	   RCC->APB1ENR &= ~(1<<23)

/* SPI */
#define SPI1_PCLK_DI()	   RCC->APB2ENR &= ~(1<<12)
#define SPI2_PCLK_DI()	   RCC->APB1ENR &= ~(1<<15)
#define SPI3_PCLK_DI()	   RCC->APB1ENR &= ~(1<<14)
#define SPI4_PCLK_DI()	   RCC->APB2ENR &= ~(1<<13)

/* USART */
#define USART2_PCLK_DI()	   RCC->APB1ENR &= ~(1<<17)
#define USART6_PCLK_DI()	   RCC->APB2ENR &= ~(1<<5)
#define USART1_PCLK_DI()	   RCC->APB2ENR &= ~(1<<4)

/*
 * System config peripheral clock enable
 */
#define SYSCFG_CLK_DI()	   RCC->APB2ENR &= ~(1<<14)

#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOH) ? 5 : 0 )

/************************* IRQ Numbers ******************************************/

#define IRQ_NO_EXT0       		6
#define IRQ_NO_EXT1		  		7
#define IRQ_NO_EXT2				8
#define IRQ_NO_EXT3				9
#define IRQ_NO_EXT4				10
#define IRQ_NO_EXT9_5			23
#define IRQ_NO_EXT15_10			40



/***********************general macros ******************************************/
#define ENABLE   				1
#define DISABLE  				0
#define SET		 				ENABLE
#define RESET	 				DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLG_RESET				RESET
#define FLG_SET					SET


/*********************************************************************************
 *  Bit position definition of SPI peripheral
 *********************************************************************************/
/*
 * Bit position macros for SPI_CR1 reg
 */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15


/*
 * Bit position macros for SPI_CR2 reg
 */
#define SPI_CR2_RXDMAEN 		0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
 * Bot position macros for SPI_SR reg
 */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

#include <string.h>
#include "stm32f401re_spi_driver.h"
#include "stm32f401re_gpio_driver.h"

#endif /* INC_STM32F401XX_H_ */
