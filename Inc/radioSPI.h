#ifndef SPI_H
#define SPI_H


#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_spi.h"


void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

void    spiInit(uint32_t mode);
void    spiTransfer(void* buf, size_t count);
bool    isSpi_Ready();
uint8_t spiTransfer(uint8_t byte);
HAL_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef spiTransmit(uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef spiReceive(uint8_t *pRxData, uint16_t Size, uint32_t Timeout);


#endif // SPI_H
