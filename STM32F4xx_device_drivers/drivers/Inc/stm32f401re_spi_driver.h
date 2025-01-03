/*
 * stm32f401re_spi_driver.h
 *
 *  Created on: 23-Dec-2024
 *      Author: SHASHANK S D
 */

#ifndef INC_STM32F401RE_SPI_DRIVER_H_
#define INC_STM32F401RE_SPI_DRIVER_H_

#include "stm32f401xx.h"

/*
 * SPI Configuration structure
 */

typedef struct
{
	uint8_t SPI_devMode;
	uint8_t SPI_busConfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;
}SPI_Config_t;

/*
 * SPI Handle structure
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;    // for accessing the SPI1,SPI2... peripheral
	SPI_Config_t SPI_Config;
}SPI_Handle_t;


/******************************* macros for SPI ****************************************/
/*
 * Mode of SPI
 */
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * SPI BusConfig
 */
#define SPI_BUS_CNFG_FD					1	  // BIDIMODE for HD or FD
#define SPI_BUS_CNFG_HD					2
/* here the below is commented bcz it is same as the Full duplex mode */
// #define SPI_BUS_CNFG_SIMPLEX_TXONLY		3     //TXONLY bit of SPI_CR1 reg
#define SPI_BUS_CNFG_SIMPLEX_RXONLY		3	  //RXONLY bit of SPI_CR1_reg


/*
 * SPI CLK SPEED (baudrate rate control)
 */
#define SPI_SCLK_SPPED_DIV2				0
#define SPI_SCLK_SPPED_DIV4				1
#define SPI_SCLK_SPPED_DIV8				2
#define SPI_SCLK_SPPED_DIV16			3
#define SPI_SCLK_SPPED_DIV32			4
#define SPI_SCLK_SPPED_DIV64			5
#define SPI_SCLK_SPPED_DIV128			6
#define SPI_SCLK_SPPED_DIV256			7


/*
 * SPI DFF(data frame format)
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1


/*
 * SPI CPOL
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1


/*
 * SPI CPHA
 */
#define SPI_CPHA_FIRST_CLK				0
#define SPI_CPHA_SECOND_CLK				1


/*
 * SPI SSM(Software Slave Management)
 */
#define SPI_SSM_EN 						1
#define SPI_SSM_DI						0

/*
 * SPI related status flag definitions
 */
#define SPI_TXE_FLAG				( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG				( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG				( 1 << SPI_SR_BSY )

/******************************* API supported by this driver***************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

/*
 * Init and deint
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */

/* Blocking mode API - using Polling method */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);


/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Other peripheral Control API's
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);


#endif /* INC_STM32F401RE_SPI_DRIVER_H_ */
