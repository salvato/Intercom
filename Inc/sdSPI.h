#ifndef __SD_SPI_H
#define __SD_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>

#define __IO    volatile   

#define FS_READNONLY                            1

#define SD_SPIx                                 SPI2
#define SD_SPIx_CLK_ENABLE()                    __HAL_RCC_SPI2_CLK_ENABLE()

#define SD_SPIx_SCK_AF                          GPIO_AF5_SPI2
#define SD_SPIx_SCK_GPIO_PORT                   GPIOB
#define SD_SPIx_SCK_PIN                         GPIO_PIN_13
#define SD_SPIx_SCK_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_SPIx_SCK_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define SD_SPIx_MISO_AF                         GPIO_AF5_SPI2
#define SD_SPIx_MISO_GPIO_PORT                  GPIOC
#define SD_SPIx_MISO_PIN                        GPIO_PIN_2
#define SD_SPIx_MISO_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define SD_SPIx_MISO_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()

#define SD_SPIx_MOSI_AF                         GPIO_AF5_SPI2
#define SD_SPIx_MOSI_GPIO_PORT                  GPIOB
#define SD_SPIx_MOSI_PIN                        GPIO_PIN_15
#define SD_SPIx_MOSI_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define SD_SPIx_MOSI_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()

 // Maximum Timeout values for flags waiting loops. These timeouts are not based
 //   on accurate values, they just guarantee that the application will not remain
 //   stuck if the SPI communication is corrupted.
 //   You may modify these timeout values depending on CPU frequency and application
 //   conditions (interrupts routines ...).
 #define SD_SPIx_TIMEOUT_MAX                   1000


 void       spi2Init();
 void       spi2GpioInit();
 void       spi2DeInit();
 void       spi2GpioDeInit();
#ifndef FS_READNONLY
    void       spi2Write(uint8_t Value);
#endif
void       spi2WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
void       spi2Error();

#ifdef __cplusplus
}
#endif


#endif // __SD_SPI_H
