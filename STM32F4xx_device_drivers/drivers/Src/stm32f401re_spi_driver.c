/*
 * stm32f401re_spi_driver.c
 *
 *  Created on: 23-Dec-2024
 *      Author: SHASHANK S D
 */


#include "stm32f401xx.h"


/*
 * Peripheral Clock setup
 */

/*****************************************************************************
* @fn 			- SPI_PeriClockControl
*
* @brief		- This function enable or disables peripheral closk for the given SPI
*
* @param1		- base address of the spi peripheral
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

/*****************************************************************************
* @fn 			- SPIO_PeriCLockControl
*
* @brief		- This function will initiliaze the SPI peripheral
*
* @param1		- SPI handle for intializing the SPI
* @param2		- none
* @param3		- none
*
* @return		- none
*
* @Note			- none
*/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* 1. configure the SPI_CR1_reg */
	tempreg = pSPIHandle->SPI_Config.SPI_devMode << SPI_CR1_MSTR;

	/* 2. configure the bus config */
	if(pSPIHandle->SPI_Config.SPI_busConfig == SPI_BUS_CNFG_FD)
	{
		// BIDIMODE should be cleared - for Full duplex mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_busConfig == SPI_BUS_CNFG_HD)
	{
		// BIDIMODE should be set - for Half duplex mode
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_busConfig == SPI_BUS_CNFG_SIMPLEX_RXONLY)
	{
		// BIDIMODE should be cleared
		// SET the RXONLY bit for simplex mode
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	/* 3. Configure the spi serial clock speed (baud rate) */
	tempreg |= pSPIHandle->SPI_Config.SPI_Speed << SPI_CR1_BR;

	/* 4. COnfigure the DFF */
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/* 5. Configure the CPOL */
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/* 6. Configure the CPHA */
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;


	pSPIHandle->pSPIx->SPI_CR1 = tempreg;    // we can use the assignement operator becz of the fresh assignement

}


/*****************************************************************************
* @fn 			- SPI_DeInit
*
* @brief		- function for deinit the SPI peripheral
*
* @param1		- SPI register structure pointer for setting particular bit
* @param2		- none
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*
 * Get the status of the SPI
 */

/* Blocking mode API - using Polling method */

/*****************************************************************************
* @fn 			- Get_SPI_Status
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if((pSPIx->SPI_SR & FlagName))
	{
		return FLG_SET;
	}

	return FLG_RESET;
}





/*
 * Data send and receive
 */

/* Blocking mode API - using Polling method */

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
* @Note			- This is bloacking call, used with while loop
*/
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len)
{
	while(len > 0)
	{
		// 1. wait untill TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLG_RESET);

		// 2. check the DFF bit is CR1
        if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
        {
        	// 16 bit DFF
        	// load the DR with the tx buffer
        	pSPIx->SPI_DR = *(uint16_t *)pTxBuffer;
        	len--;
        	len--;
        	(uint16_t *)pTxBuffer++;
        }
        else
        {
        	// 8 bit DFF
        	// load the DR with the tx buffer
			pSPIx->SPI_DR = *pTxBuffer;
			len--;
			pTxBuffer++;
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len)
{
	while(len > 0)
	{
		// 1. wait untill RXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLG_RESET);

		// 2. check the DFF bit is CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF
			// load the data from DR to Rx buffer
			*(uint16_t *)pRxBuffer = pSPIx->SPI_DR;
			len--;
			len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			// 8 bit DFF
			// load the data from DR to Rx buffer
			*pRxBuffer = pSPIx->SPI_DR;
			len--;
			pRxBuffer++;
		}
	}
}


/*
 * IRQ Configuration and ISR Handling
 */


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
void SPI_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

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
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}


/*****************************************************************************
* @fn 			- SPI_PeriCLockControl
*
* @brief		- This function enable or disables SPI peripheral clock.
*
* @param1		- Register structure of the SPI
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}


/*****************************************************************************
* @fn 			- SPI_PeriCLockControl
*
* @brief		- This function enable or disables SPI peripheral clock.
*
* @param1		- Register structure of the SPI
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}



/*****************************************************************************
* @fn 			- SPI_PeriCLockControl
*
* @brief		- This function enable or disables SPI peripheral clock.
*
* @param1		- Register structure of the SPI
* @param2		- ENABLE or DISABLE
* @param3		-
*
* @return		- none
*
* @Note			- none
*/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
