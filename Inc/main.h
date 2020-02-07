#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio.h"
#include "usbh_def.h"


#define REPEAT_ON        ((uint32_t)0x00) /* Replay Status in ON */
#define REPEAT_OFF       ((uint32_t)0x01) /* Replay Status in OFF */

/* Defines for the Audio playing process */
#define PAUSE_STATUS     ((uint32_t)0x00) /* Audio Player in Pause Status */
#define RESUME_STATUS    ((uint32_t)0x01) /* Audio Player in Resume Status */
#define IDLE_STATUS      ((uint32_t)0x02) /* Audio Player in Idle Status */

/* Defines for the Audio used commands */
#define CMD_PLAY           ((uint32_t)0x00)
#define CMD_RECORD         ((uint32_t)0x01)
#define CMD_STOP           ((uint32_t)0x02)

typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_START,
  APPLICATION_RUNNING,

} MSC_ApplicationTypeDef;


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
void OTG_FS_IRQHandler(void);
void COMMAND_AudioExecuteApplication(void);
void MSC_Application(void);

void _delay_3t(uint32_t cycles);
void delay_cycles(const int64_t cycles);
void delayMicroseconds(uint32_t us);

extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;

#ifdef __cplusplus
}
#endif


#endif // __MAIN_H__
