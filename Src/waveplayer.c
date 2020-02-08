/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/waveplayer.c
  * @author  MCD Application Team
  * @brief   I2S Audio player program.
  *                                           This is a version modified by G.S.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "main.h"
#include "ff.h"
#include "waveplayer.h"


extern __IO uint32_t RepeatState, PauseResumeStatus, PressCount;

static uint8_t Volume = 70;

extern __IO uint32_t CmdIndex; // Defined in main.c
extern MSC_ApplicationTypeDef AppliState; // Defined in main.c


void
WavePlayBack(uint32_t AudioFreq) {
    UINT bytesread = 0;
    RepeatState = REPEAT_ON;
    if(WavePlayerInit(AudioFreq) != 0) {
        Error_Handler();
    }
    Audio_Buffer = (uint8_t*)malloc(AUDIO_BUFFER_SIZE*sizeof(*Audio_Buffer));
    f_lseek(&FileRead, 0);
    f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
    AudioRemSize = WaveDataLength - bytesread;
    BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
    PauseResumeStatus = RESUME_STATUS;
    PressCount = 0;
    while((AudioRemSize != 0) && (AppliState != APPLICATION_IDLE)) {
        if(CmdIndex == CMD_PLAY) {
            if(PauseResumeStatus == PAUSE_STATUS) {
                WavePlayerPauseResume(PauseResumeStatus);
                PauseResumeStatus = IDLE_STATUS;
            }
            else if(PauseResumeStatus == RESUME_STATUS) {
                WavePlayerPauseResume(PauseResumeStatus);
                PauseResumeStatus = IDLE_STATUS;
            }
            bytesread = 0;
            if(buffer_offset == BUFFER_OFFSET_HALF) {
                f_read(&FileRead,
                       &Audio_Buffer[0],
                       AUDIO_BUFFER_SIZE/2,
                       (void *)&bytesread);
                buffer_offset = BUFFER_OFFSET_NONE;
            }

            if(buffer_offset == BUFFER_OFFSET_FULL) {
                f_read(&FileRead,
                       &Audio_Buffer[AUDIO_BUFFER_SIZE/2],
                       AUDIO_BUFFER_SIZE/2,
                       (void *)&bytesread);

                buffer_offset = BUFFER_OFFSET_NONE;
            }
            if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2)) {
                AudioRemSize -= bytesread;
            }
            else {
                AudioRemSize = 0;
            }
        }
        else {
            WavePlayerStop();
            f_close(&FileRead);
            AudioRemSize = 0;
            RepeatState = REPEAT_ON;
            break;
        }
    }
    RepeatState = REPEAT_ON;
    WavePlayerStop();
    f_close(&FileRead);
}


void
WavePlayerPauseResume(uint32_t wState) {
    if(wState == PAUSE_STATUS) {
        BSP_AUDIO_OUT_Pause();
    }
    else {
        BSP_AUDIO_OUT_Resume();
    }
}


void
WavePlayerStop(void) {
    free(Audio_Buffer);
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
}


int
WavePlayerInit(uint32_t AudioFreq) {
    return(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AudioFreq));
}


void
WavePlayerStart(void) {
    UINT bytesread = 0;
    char path[] = "0:/";
    char* wavefilename = NULL;
    WAVE_FormatTypeDef waveformat;

    if(f_opendir(&Directory, path) == FR_OK) {
            wavefilename = WAVE_NAME;
        if(f_open(&FileRead, wavefilename , FA_READ) != FR_OK) {
            BSP_LED_On(LED5);
            Error_Handler();
        }
        else {
            f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
            WaveDataLength = waveformat.FileSize;
            WavePlayBack(waveformat.SampleRate);
        }
    }
}


void
WavePlayer_CallBack(void) {
    if(AppliState != APPLICATION_IDLE) {
        RepeatState = REPEAT_ON;
        PauseResumeStatus = RESUME_STATUS;
        WaveDataLength =0;
        PressCount = 0;

        if(BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW) != AUDIO_OK) {
            Error_Handler();
        }
    }
} 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
