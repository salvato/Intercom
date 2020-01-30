#include "SPI.h"
#include "main.h" // for Error_Handler
#include "string.h"


void
HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    UNUSED(hspi);
    GPIO_InitTypeDef  GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    // Enable GPIO TX/RX clock
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();

    // SPI SCK GPIO pin configuration
    GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    // SPI MISO GPIO pin configuration
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Pin       = SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    // SPI MOSI GPIO pin configuration
    GPIO_InitStruct.Pin       = SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

    // Enable SPI clock
    SPIx_CLK_ENABLE();

    // NVIC for SPI
    HAL_NVIC_SetPriority(SPIx_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPIx_IRQn);
}


void
HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
    UNUSED(hspi);
    // Reset peripherals
    SPIx_FORCE_RESET();
    SPIx_RELEASE_RESET();
    // Disable peripherals and GPIO Clocks
    HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
    HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
    HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
    // Disable the NVIC for SPI
    HAL_NVIC_DisableIRQ(SPIx_IRQn);
}



SPI::SPI()
{
}


// In SPI, two specific features of the clock signal determine
// the start and end of data transmission:
// the clock polarity (CPOL) and clock phase (CPHA).
// CPOL refers to the idle state (either low or high) of the clock signal.
// To conserve power, devices will put the clock line at the idle state
// when not communicating with any slaves, and the two options available
// for this idle state are either low or high.
// CPHA refers to the edge of the clock signal upon which data is captured.
// Depending on the CPHA setting, data is captured either on the rising edge
// or the falling edge.
void
SPI::begin(uint32_t mode) {
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    memset(&SpiHandle, 0, sizeof(SpiHandle));
    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.BaudRatePrescaler = GetPrescalerFromMaxFrequency(3000000);
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.Mode              = mode;
    if(HAL_SPI_Init(&SpiHandle) != HAL_OK) {
        Error_Handler();
    }
}


uint32_t
SPI::GetPrescalerFromMaxFrequency(uint32_t MAX_SPI_Frequency) {
    // Prevent false input
    if (MAX_SPI_Frequency == 0) {
        return SPI_BAUDRATEPRESCALER_256;
    }
    // Calculate max SPI clock
    uint32_t APB_Frequency = HAL_RCC_GetPCLK2Freq();
    // Calculate prescaler value
    // Bits 5:3 in CR1 SPI registers are prescalers
    // 000 = 2, 001 = 4, 002 = 8, ..., 111 = 256
    for(uint8_t i=0; i<8; i++) {
        if(APB_Frequency/(1 << (i+1)) <= MAX_SPI_Frequency) {
            return (i<<3);
        }
    }
    // Use max prescaler possible
    return SPI_BAUDRATEPRESCALER_256;
}



void
SPI::setBitOrder(uint8_t bitOrder) {
    UNUSED(bitOrder);
}


void
SPI::setDataMode(uint8_t spiMode) {
    UNUSED(spiMode);
}


void
SPI::setClockDivider(uint8_t clockDivider) {
    UNUSED(clockDivider);
}


uint8_t
SPI::transfer(uint8_t byte) {
    uint8_t response;
    HAL_SPI_TransmitReceive(&SpiHandle, &byte, &response, 1, 2);
    return response;
}


void
SPI::transfer(void* buf, size_t count) {
    if(count==0) return;
    uint8_t* p = reinterpret_cast<uint8_t*>(buf);
    HAL_SPI_TransmitReceive(&SpiHandle, p, p, count, 2);
}


HAL_StatusTypeDef
SPI::SPI_Transmit(uint8_t *pTxData,
                  uint16_t Size,
                  uint32_t Timeout)
{
    return HAL_SPI_Transmit(&SpiHandle, pTxData, Size, Timeout);
}


HAL_StatusTypeDef
SPI::SPI_Receive(uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    return HAL_SPI_Receive(&SpiHandle, pRxData, Size, Timeout);
}


bool
SPI::is_SPI_Ready() {
    return SpiHandle.State == HAL_SPI_STATE_READY;
}
