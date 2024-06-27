/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Feb 23, 2024
 *      Author: capta
 */


#include <stm32f446xx_spi_driver.h>
#include <sys/_stdint.h>
#include<stddef.h>


 static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
 static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
 static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(EnorDi == DISABLE)
				{
					if(pSPIx == SPI1)
					{
						SPI1_PCLK_DI();
					}else if (pSPIx == SPI2)
					{
						SPI2_PCLK_DI();
					}else if (pSPIx == SPI3)
					{
						SPI3_PCLK_DI();
					}else if (pSPIx == SPI4)
					{
						SPI4_PCLK_DI();
					}
		}

		}
}



//INIT AND DE-INIT
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);
    uint32_t tempreg  =0;





    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD)
    {
    	tempreg &=~(1<< SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig== SPI_BUSCONFIG_HD)
    {
    	tempreg |= (1<< SPI_CR1_BIDIMODE);
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUSCONFIG_SIMPLEX_RXONLY )
        {
    	tempreg &=~(1<< SPI_CR1_BIDIMODE);
    	tempreg |= (1<< SPI_CR1_RXONLY);
        }
   tempreg |=  pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;


   tempreg |=  pSPIHandle->SPIConfig.SPI_DFF <<SPI_CR1_DFF;


   tempreg |=  pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

   tempreg |=  pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;



   pSPIHandle->pSPIx->SPI_CR1=  tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	                     if(pSPIx == SPI1)
						{
							SPI1_REG_RESET();
						}else if (pSPIx == SPI2)
						{
							SPI2_REG_RESET();
						}else if (pSPIx == SPI3)
						{
							SPI3_REG_RESET();
						}else if (pSPIx == SPI4)
						{
							SPI4_REG_RESET();
						}
}


 uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	 if(pSPIx->SPI_SR & FlagName)
	 {
		 return FLAG_SET;
	 }
	return FLAG_RESET;
}
//DATA SEND AND RECEIVE

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET);

		if(pSPIx->SPI_CR1 &(1 << SPI_CR1_DFF))
		{
			pSPIx->SPI_DR  = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET);

		if(pSPIx->SPI_CR1 &(1 << SPI_CR1_DFF))
		{
			 *((uint16_t *)pRxBuffer)= pSPIx->SPI_DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			 *(pRxBuffer) =pSPIx->SPI_DR;
			Len--;
			pRxBuffer++;
		}
	}
}


//IRQ CONFIGURATION


	void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
	{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber  < 64)
		{
			*NVIC_ISER1 |= (1 << IRQNumber  % 32);
		}

		else if(IRQNumber >= 64 && IRQNumber  < 96)
			{

			*NVIC_ISER2 |= (1 << IRQNumber  % 64);

			}

	}

	else
	{
		if(IRQNumber <= 31)
			{
				*NVIC_ICER0 |= (1 << IRQNumber);

			}
			else if(IRQNumber > 31 && IRQNumber  < 64)
			{
				*NVIC_ICER1 |= (1 << IRQNumber  % 32);
			}

			else if(IRQNumber >= 64 && IRQNumber  < 96)

				{

				*NVIC_ICER2 |= (1 << IRQNumber  % 64);

				}

	}
	}


	void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)

	{
	uint8_t iprx  = IRQNumber /4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift =  (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASEADDR  + iprx )  |= (IRQPriority << shift );

	}





void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{

		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);

		}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}

}
void SPI_SSICONFIG(SPI_RegDef_t *pSPIx,uint8_t EnorDi )
{
	if(EnorDi == ENABLE)
	{

		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);

		}
	else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}

}
void SPI_SSOECONFIG(SPI_RegDef_t *pSPIx,uint8_t EnorDi )
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


//IRQ  SPI

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len)
{


	uint8_t state  = pSPIHandle->TxState;
      if(state != SPI_BUSY_IN_TX)
      {
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
      }
      return state;

}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t state  = pSPIHandle->TxState;
      if(state != SPI_BUSY_IN_RX)
      {
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;
	pSPIHandle->RxState = SPI_BUSY_IN_RX;
	pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
      }
      return state;
}



void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	temp1  = pHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_TXEIE);
	//CHECK FOR TXE
	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);

	}


	//CHECK FOR RXNE
	temp1  = pHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
		{
			spi_rxne_interrupt_handle(pHandle);

		}
	//CHECK FOR OVR FLAG

	temp1  = pHandle->pSPIx->SPI_SR & ( 1 << SPI_SR_OVR);
		temp2 = pHandle->pSPIx->SPI_CR2 & ( 1 << SPI_CR2_ERRIE);

		if(temp1 && temp2)
			{
				spi_ovr_err_interrupt_handle(pHandle);

			}
}
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 &(1 << SPI_CR1_DFF))
			{
		pSPIHandle->pSPIx->SPI_DR  = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
				(uint16_t *)pSPIHandle->pTxBuffer++;
			}
			else
			{
				pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
	if(!  pSPIHandle->TxLen)
	{
		//if tx is 0 then close the application
		//THIS PREVENTS SETTING UP OF TXE FLAG
		SPI_CloseTransmission( pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);


	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 &( 1 << SPI_CR1_DFF))
				{
*((uint16_t *)pSPIHandle->pRxBuffer) = 	(uint16_t)pSPIHandle->pSPIx->SPI_DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer--;
			pSPIHandle->pRxBuffer--;
				}
				else
				{
					 *(pSPIHandle->pTxBuffer) = (uint8_t)pSPIHandle->pSPIx->SPI_DR;
					pSPIHandle->RxLen--;
					pSPIHandle->pRxBuffer--;
				}
		if(!  pSPIHandle->RxLen)
		{
			//if Rx is 0 then close the application
			//THIS PREVENTS SETTING UP OF RXNE FLAG
			SPI_CloseReception( pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);


		}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;

//CLEAR THE OVR FLAG
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
	temp  = pSPIHandle->pSPIx->SPI_DR;
	temp =  pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
//INFORM THE APPLICATION
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE);
			pSPIHandle->pTxBuffer = NULL;
			pSPIHandle->TxLen= 0;
			pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
				pSPIHandle->pRxBuffer = NULL;
				pSPIHandle->RxLen= 0;
				pSPIHandle->RxState = SPI_READY;
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	 //This is a weak implementation . the user application may override this function.
}

