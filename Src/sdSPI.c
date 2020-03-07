//          ____________________
//         /   | | | | | | | |  |
//        / |  | | | | | | | |  |
//        | |                   |
//        |                     |
//        |    1 2 3 4 5 6 7 8  |
//        | 9                   |
//        |                     |
//        |         SD          |
//        |        Card         |
//        |                     |
//        |                     |
//        |_____________________|

//        Mode    SDIO          SPI
//        ---------------------------------
//        Pin 1 - Data 3        ~Card Select
//        Pin 2 - Cmd I/O       MOSI
//        Pin 3 - Gnd
//        Pin 4 - Vdd (3.3V)
//        Pin 5 - Clock         SCLK
//        Pin 6 - Gnd
//        Pin 7 - Data 0        MISO
//        Pin 8 - Data 1        NC
//        Pin 9 - Data 2        NC

#include "sdSPI.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_spi.h"
#include "string.h"


// Value of Timeout when SPI communication fails
static uint32_t SpixTimeout = SD_SPIx_TIMEOUT_MAX;

SPI_HandleTypeDef hnucleo_Spi;


//  * @brief  Initializes SPI HAL
void
spi2Init() {
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    memset(&hnucleo_Spi, 0, sizeof(hnucleo_Spi));
    hnucleo_Spi.Instance = SD_SPIx;
    // SPI baudrate is set to 21.0 MHz (APB1/SPI_BaudRatePrescaler = 42/2 = 21.0 MHz)
    // - SD card SPI interface max baudrate is 25MHz for write/read
    hnucleo_Spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hnucleo_Spi.Init.Direction         = SPI_DIRECTION_2LINES;
    hnucleo_Spi.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hnucleo_Spi.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    hnucleo_Spi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    hnucleo_Spi.Init.CRCPolynomial     = 7;
    hnucleo_Spi.Init.DataSize          = SPI_DATASIZE_8BIT;
    hnucleo_Spi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hnucleo_Spi.Init.NSS               = SPI_NSS_SOFT;
    hnucleo_Spi.Init.TIMode            = SPI_TIMODE_DISABLED;
    hnucleo_Spi.Init.Mode              = SPI_MODE_MASTER;

    spi2GpioInit();
    HAL_SPI_Init(&hnucleo_Spi);
}


//  * @brief  Initializes SPI MSP.
void
spi2GpioInit() {
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*** Configure the GPIOs ***/
    SD_SPIx_SCK_GPIO_CLK_ENABLE();
    SD_SPIx_MISO_GPIO_CLK_ENABLE();
    SD_SPIx_MOSI_GPIO_CLK_ENABLE();

    /* Configure SPI SCK */
    GPIO_InitStruct.Pin       = SD_SPIx_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = SD_SPIx_SCK_AF;
    HAL_GPIO_Init(SD_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

    /* Configure SPI MOSI */
    GPIO_InitStruct.Pin       = SD_SPIx_MOSI_PIN;
    GPIO_InitStruct.Alternate = SD_SPIx_MOSI_AF;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    HAL_GPIO_Init(SD_SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

    /* Configure SPI MISO */
    GPIO_InitStruct.Pin       = SD_SPIx_MISO_PIN;
    GPIO_InitStruct.Alternate = SD_SPIx_MISO_AF;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    HAL_GPIO_Init(SD_SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

    /* Enable SPI clock */
    SD_SPIx_CLK_ENABLE();
}


void
spi2GpioDeInit() {
    // Reset peripherals
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    // Disable peripherals and GPIO Clocks
    HAL_GPIO_DeInit(SD_SPIx_SCK_GPIO_PORT, SD_SPIx_SCK_PIN);
    HAL_GPIO_DeInit(SD_SPIx_MISO_GPIO_PORT, SD_SPIx_MISO_PIN);
    HAL_GPIO_DeInit(SD_SPIx_MOSI_GPIO_PORT, SD_SPIx_MOSI_PIN);
    // Disable the NVIC for SPI
//    HAL_NVIC_DisableIRQ(SPIx_IRQn);
}



//  * @brief  SPI Write a byte to device
//  * @param  DataIn: value to be written
//  * @param  DataOut: value to be read
//  * @param  DataLegnth: length of data
void
spi2WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth) {
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) DataIn, DataOut, DataLegnth, SpixTimeout);
    /* Check the communication status */
    if(status != HAL_OK) {
        /* Execute user timeout callback */
        spi2Error();
    }
}


#ifndef FS_READNONLY
//  * @brief  SPI Write a byte to device.
//  * @param  Value: value to be written
void
spi2Write(uint8_t Value) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data;

    status = HAL_SPI_TransmitReceive(&hnucleo_Spi, (uint8_t*) &Value, &data, 1, SpixTimeout);

    /* Check the communication status */
    if(status != HAL_OK) {
        /* Execute user timeout callback */
        spi2Error();
    }
}
#endif


//  * @brief  SPI error treatment function.
void
spi2Error () {
    /* De-initialize the SPI communication BUS */
    HAL_SPI_DeInit(&hnucleo_Spi);
    /* Re-Initiaize the SPI communication BUS */
    spi2Init();
}
