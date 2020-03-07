#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"

// FatFs includes component
#include "ff_gen_drv.h"
#include "sd_diskio.h"

//#include "ff.h"
//#include "usbh_def.h"


// Defines for the Audio playing process
#define WAVE_NAME         "0:AveMaria.wav"
#define AUDIO_BUFFER_SIZE 1024


//  * @brief  SD Control Interface pins
#define SD_CS_PIN                                 GPIO_PIN_8
#define SD_CS_GPIO_PORT                           GPIOB
#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()

//  * @brief  SD Control Lines management
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

#define SD_DUMMY_BYTE     0xFF


typedef enum {
    APPLICATION_IDLE = 0,
    APPLICATION_START,
    APPLICATION_RUNNING,

} MSC_ApplicationTypeDef;


typedef enum {
    BUFFER_OFFSET_NONE = 0,
    BUFFER_OFFSET_HALF,
    BUFFER_OFFSET_FULL,

} BUFFER_StateTypeDef;


typedef struct {
    uint32_t   ChunkID;       /* 0 */
    uint32_t   FileSize;      /* 4 */
    uint32_t   FileFormat;    /* 8 */
    uint32_t   SubChunk1ID;   /* 12 */
    uint32_t   SubChunk1Size; /* 16*/
    uint16_t   AudioFormat;   /* 20 */
    uint16_t   NbrChannels;   /* 22 */
    uint32_t   SampleRate;    /* 24 */

    uint32_t   ByteRate;      /* 28 */
    uint16_t   BlockAlign;    /* 32 */
    uint16_t   BitPerSample;  /* 34 */
    uint32_t   SubChunk2ID;   /* 36 */
    uint32_t   SubChunk2Size; /* 40 */

} WAVE_FormatTypeDef;


#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void);
void EXTI15_10_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hAdc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hAdc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void);
void BSP_AUDIO_OUT_TransferComplete_CallBack(void);
void BSP_AUDIO_IN_TransferComplete_CallBack(void);
void BSP_AUDIO_IN_HalfTransfer_CallBack(void);
void BSP_AUDIO_IN_Error_Callback(void);
void BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void I2S3_IRQHandler(void);
void I2S2_IRQHandler(void);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void TIM3_IRQHandler(void);
void TIM7_IRQHandler(void);

void _delay_3t(uint32_t cycles);
void delay_cycles(const int64_t cycles);
void delayMicroseconds(uint32_t us);

extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;
extern SPI_HandleTypeDef hnucleo_Spi;

#ifdef __cplusplus
}
#endif


#endif // __MAIN_H__
