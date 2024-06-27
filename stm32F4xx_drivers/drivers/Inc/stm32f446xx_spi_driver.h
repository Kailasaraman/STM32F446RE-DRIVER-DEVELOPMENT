/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: Feb 23, 2024
 *      Author: capta
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_



#include "stm32f446xx.h"

// SPI CONFIGURATION
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

//HANDLE STRUCTURE

typedef struct
{
	SPI_RegDef_t     *pSPIx;
	SPI_Config_t      SPIConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
		uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
		uint32_t 		TxLen;		/* !< To store Tx len > */
		uint32_t 		RxLen;		/* !< To store Tx len > */
		uint8_t 		TxState;	/* !< To store Tx state > */
		uint8_t 		RxState;	/* !< To store Rx state > */

}SPI_Handle_t;




//SPI MACROS

#define SPI_DeviceMode_Master         1
#define SPI_DeviceMode_Slave          0



#define SPI_BUSCONFIG_FD                     1
#define SPI_BUSCONFIG_HD                     2
#define SPI_BUSCONFIG_SIMPLEX_RXONLY         4


#define SPI_SCLK_SPEED_DIV2                  0
#define SPI_SCLK_SPEED_DIV4                  1
#define SPI_SCLK_SPEED_DIV8                  2
#define SPI_SCLK_SPEED_DIV16                 3
#define SPI_SCLK_SPEED_DIV32                 4
#define SPI_SCLK_SPEED_DIV64                 5
#define SPI_SCLK_SPEED_DIV128                6
#define SPI_SCLK_SPEED_DIV256                7



#define SPI_DFF_8BITS           0
#define SPI_DFF_16BITS          1





#define SPI_CPOL_HIGH           1
#define SPI_CPOL_LOW            0





#define SPI_CPHA_HIGH           1
#define SPI_CPHA_LOW            0





#define SPI_SSM_EN            1
#define SPI_SSM_DI            0


#define SPI_TXE_FLAG         (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG        (1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG        (1 << SPI_SR_BSY)


#define SPI_READY             0
#define SPI_BUSY_IN_RX        1
#define SPI_BUSY_IN_TX        2


//POSSIBLE SPI APPLICATION EVENTS
#define SPI_EVENT_TX_CMPLT        1
#define SPI_EVENT_RX_CMPLT        2
#define SPI_EVENT_OVR_ERR         3



//PERIPHERAL CLOCK
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);



//INIT AND DE-INIT
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);



//DATA SEND AND RECEIVE

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t Len);

//IRQ CONFIGURATION

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t *pHandle);

// SPI PERIPHERAL CONTROL
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi );
void SPI_SSICONFIG(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOECONFIG(SPI_RegDef_t *pSPIx,uint8_t EnorDi);


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//APPLICATION EVENT CALLBACK
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
