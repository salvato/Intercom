#ifndef SPI_H
#define SPI_H

#include "stm32f4_discovery.h"

/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler


// To change...................
#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C
// ...................Up to here

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);

class SPI
{
public:
    SPI();
    void begin(uint32_t mode);
    void setBitOrder(uint8_t bitOrder);
    void setDataMode(uint8_t spiMode);
    void setClockDivider(uint8_t clockDivider);
    uint8_t transfer(uint8_t byte);
    void transfer(void* buf, size_t count);
    HAL_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef SPI_Transmit(uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
    HAL_StatusTypeDef SPI_Receive(uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
    bool is_SPI_Ready();

public:
    SPI_HandleTypeDef SpiHandle;
private:
    uint32_t GetPrescalerFromMaxFrequency(uint32_t MAX_SPI_Frequency);
};

#endif // SPI_H
