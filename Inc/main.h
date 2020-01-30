#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"

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
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void I2S3_IRQHandler(void);
void I2S2_IRQHandler(void);

void _delay_3t(uint32_t cycles);
void delay_cycles(const int64_t cycles);
void delayMicroseconds(uint32_t us);

extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;

#ifdef __cplusplus
}
#endif


#endif // __MAIN_H__
