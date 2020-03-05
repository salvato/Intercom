#include "main.h"
#include "RF24.h"
#include "printf.h"
#include "string.h"
#include "stdlib.h"
#include "nRF24L01.h"
#include "ff_gen_drv.h"
#include "config.h"


//====================================================================
//
// The STM32F4Discovery board can supply ~100mA @ 3.3V and ~100mA @ 5V
//
// Timers on APB1 bus are architecturally limited to 84 MHz,
// as the bus is slower nominally 36-42 MHz across STM32 families.
//
// The APB2 bus is faster (84 MHz), and should permit a 168 MHz time-base.
// Refer to the clock tree diagram in the reference manual.
//
// MSP (MCU Support Package)
//====================================================================


//===============================================================
// Interrupt Priorities
//===============================================================
// PreemptPriority can be a value between 0 and 15
//      A lower priority value indicates a higher priority
// SubPriority can be a value between 0 and 15
//      A lower priority value indicates a higher priority.
//===============================================================
//#define AUDIO_OUT_IRQ_PREPRIO           0x0E
//#define AUDIO_IN_IRQ_PREPRIO            0x0D
  #define RADIO_IN_IRQ_PREPRIO            0x0F
  #define PUSH_BTN_IRQ_PREPRIO            0x0F
  #define ADC_DMA_IRQ_PREPRIO             0x02
  #define RELAY_PULSE_IRQ_PREPRIO         0x0F
//===============================================================


//===============================================================
#define MAX_CONNECTION_TIME  60000
#define MAX_WAIT_ACK_TIME    1000
#define MAX_NO_SIGNAL_TIME   5000
#define QUERY_INTERVAL       200
//===============================================================


//===============================================================
// State Machine for the USBH_USR_ApplicationState
//===============================================================
#define USBH_USR_FS_INIT    ((uint8_t)0x00)
#define USBH_USR_AUDIO      ((uint8_t)0x01)
//===============================================================


//===============================================================
// Commands
//===============================================================
typedef enum {
    connectRequest      = 0x10,
    connectionAccepted,
    connectionTimedOut,
    suspendCmd,
    suspendAck,
    checkBaseRequestCmd,
    wantConnectAck,
    openGateCmd,
    openGateAck,
    openCarGateCmd,
    openCarGateAck
} Commands;
//===============================================================


//===============================================================
// Local Functions
//===============================================================
static bool initSystem();
static void SystemClock_Config(void);
static void configPinInit();
static void configPinDeinit();
static void pushButtonInit(GPIO_TypeDef* Port, uint32_t Pin, IRQn_Type Irq);
static void relayGPIOInit(GPIO_TypeDef* Port, uint32_t Pin);
static void initLeds();
static void adcInit();
static void adcTIM2Init(void);
static void initBuffers(bool isBaseStation);
static void setRole(bool bPTX);
static void ledsOff();
static void connectBase();
static void connectRemote();
static void processBase();
static void processRemote();
static void processReceivedCommand(uint8_t command, uint8_t sourcePipe);
static void sendCommand(uint8_t command);
static bool prepareFileSystem();
static bool closeFileSystem();
static void startAlarm();
static bool updateAlarm();
static void stopAlarm();
static void relayTIM3Init();
static void relayPulse(uint32_t relayChannel, uint16_t msPulse);
//===============================================================


TIM_HandleTypeDef  Tim2Handle;
TIM_HandleTypeDef  Tim3Handle;
ADC_HandleTypeDef  hAdc;

char buf[255];

// nRF24 Radio Addresses
// Note: Addresses where the level shifts only one time (that is, 000FFFFFFF)
//       can often be detected in noise and can give a false detection,
//       which may give a raised Packet-Error-Rate.
//       Addresses as a continuation of the preamble (hi-low toggling)
//       raises the Packet-Error-Rate.
const uint8_t
pipes[6][5] = { // Note that NRF24L01(+) expects address LSB first
                  {0x70, 0xCD, 0xAB, 0xCD, 0xAB},
                  {0x71, 0xCD, 0xAB, 0xCD, 0xAB},
                  {0x72, 0xCD, 0xAB, 0xCD, 0xAB},
                  {0x73, 0xCD, 0xAB, 0xCD, 0xAB},
                  {0x74, 0xCD, 0xAB, 0xCD, 0xAB},
                  {0x75, 0xCD, 0xAB, 0xCD, 0xAB}
              };



// Data Buffers
uint8_t*  rxBuffer         = NULL;
uint8_t*  txBuffer         = NULL;
uint16_t* adcDataIn        = NULL;
uint8_t*  Audio_Buffer     = NULL;
uint16_t* Audio_Out_Buffer = NULL;
uint16_t* pdmDataIn        = NULL;
uint16_t* pcmDataOut       = NULL;



uint8_t       Volume       = 70; //% of Max
uint8_t       chunk2Write  = 0;
const uint8_t radioChannel = 2;


//===============================================================
// Disk and .wav File Variables
//===============================================================
char path[] = "0:/";
FIL File2Read;
DIR Directory;
FATFS DiskFatFs;          // File system object for disk logical drive
char DiskPath[4];         // Logical drive path
MSC_ApplicationTypeDef AppliState = APPLICATION_IDLE;


uint32_t           WaveDataLength;
__IO uint32_t      AudioRemSize;
WAVE_FormatTypeDef waveformat;
//===============================================================


__IO bool txOk;
__IO bool txFailed;
__IO bool rxDataReady;
__IO bool bRadioIrq;
__IO bool bReady2Send;
__IO bool bReady2Play;
__IO bool bBaseSleeping;
__IO bool bConnectionRequested;
__IO bool bConnectionAccepted;
__IO bool bConnectionWanted;
__IO bool bBaseDisconnected;
     bool bConnectionTimedOut;
     bool bBaseConnected;
__IO bool bSuspend;      // true if the Remote ask to suspend the connection
__IO bool bRadioDataAvailable;
__IO uint32_t startConnectTime;
__IO BUFFER_StateTypeDef buffer_offset;

__IO bool bSendOpenGate;
__IO bool bGateOpening;
__IO bool bSendOpenCarGate;
__IO bool bCarGateOpening;
     bool isBaseStation;


// HAL_NVIC_SystemReset();


int
main(void) {
    if(!initSystem())
        Error_Handler();

    // Ready for the endless loop...
    while(true) {
        if(isBaseStation) {
            connectBase(); // Base will be woken up by a button press
            processBase();
        }
        else {
            connectRemote(); // Remote will be woken up by receiving a command from Base
            processRemote();
        }
    }

}


bool
initSystem() {
    // System startup
    HAL_Init();
    SystemClock_Config();

    initLeds();
    configPinInit(); // Are we Base or Remote ?
    isBaseStation = HAL_GPIO_ReadPin(CONFIGURE_PORT, CONFIGURE_PIN);
    configPinDeinit(); // We don't need the pin anymore
    initBuffers(isBaseStation); // Initialize the needed memory buffers

    // Init Push Buttons and Relays
    PHONE_BUTTON_CLK_ENABLE();
    pushButtonInit(PHONE_BUTTON_GPIO_PORT, PHONE_BUTTON_GPIO_PIN, PHONE_BUTTON_IRQ);
    if(isBaseStation) {
        relayTIM3Init();
        GATE_RELAY_GPIO_CLK_ENABLE();
        relayGPIOInit(GATE_RELAY_GPIO_PORT, GATE_RELAY_GPIO_PIN);
        CAR_GATE_RELAY_GPIO_CLK_ENABLE();
        relayGPIOInit(CAR_GATE_RELAY_GPIO_PORT, CAR_GATE_RELAY_GPIO_PIN);
    }
    else {
        GATE_BUTTON_CLK_ENABLE();
        pushButtonInit(GATE_BUTTON_GPIO_PORT, GATE_BUTTON_GPIO_PIN, GATE_BUTTON_IRQ);
        CAR_GATE_BUTTON_CLK_ENABLE();
        pushButtonInit(CAR_GATE_BUTTON_GPIO_PORT, CAR_GATE_BUTTON_GPIO_PIN, CAR_GATE_BUTTON_IRQ);
    }

    // Init the Radio
    if(!rf24Init(radioChannel, RADIO_IN_IRQ_PREPRIO)) return false;
    rf24ClearInterrupts();// Avoid false interrupts from Radio
    rf24MaskIRQ(false, false, false);
    return true;
}


void
connectBase() {
    bool bTimeoutElapsed;
    uint32_t t0;
    setRole(PTX);
    rf24SetRetries(5, 15);
    rf24Flush_tx();
    rf24Flush_rx();

    BSP_LED_On(LED_GREEN);
    bRadioDataAvailable = false;
    do {
        bBaseConnected = false;
        bBaseSleeping  = true;
        t0 = HAL_GetTick()-QUERY_INTERVAL-1;
        while(bBaseSleeping) {
            if(HAL_GetTick()-t0 > QUERY_INTERVAL) {
                t0 = HAL_GetTick();
                txBuffer[0] = checkBaseRequestCmd;
                BSP_LED_On(LED_BLUE);
                rf24Enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
                rf24StartWrite();
                BSP_LED_Off(LED_BLUE);
            }
            if(bRadioDataAvailable) {
                bRadioDataAvailable = false;
                BSP_LED_On(LED_ORANGE);
                rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_ORANGE);
                if(rxBuffer[0] == wantConnectAck) {
                    bBaseSleeping  = false;
                    bBaseConnected = true;
                }
                if(rxBuffer[0] == openGateAck) {
                    relayPulse(GATE_RELAY_TIM_CHANNEL, 1500);
                }
                if(rxBuffer[0] == openCarGateAck) {
                    relayPulse(CAR_GATE_RELAY_TIM_CHANNEL, 1500);
                }
            }
        } // while(bBaseSleeping)

        // An awake event has been detected
        t0 = HAL_GetTick()-QUERY_INTERVAL-1;
        startConnectTime = HAL_GetTick();
        bTimeoutElapsed = false;
        while(!bBaseConnected && !bTimeoutElapsed) {
            if(HAL_GetTick()-t0 > QUERY_INTERVAL) {
                t0 = HAL_GetTick();
                txBuffer[0] = connectRequest;
                BSP_LED_On(LED_BLUE);
                rf24Enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
                rf24StartWrite();
                BSP_LED_Off(LED_BLUE);
            }
            if(bRadioDataAvailable) {
                bRadioDataAvailable = false;
                BSP_LED_On(LED_ORANGE); // Signal the packet's start reading
                rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_ORANGE); // Reading done
                if(rxBuffer[0] == connectionAccepted) {
                    bBaseConnected = true;
                    HAL_Delay(1);
                }
            }
            if(HAL_GetTick() < startConnectTime) // counter overflow
                startConnectTime = HAL_GetTick();
            bTimeoutElapsed = (HAL_GetTick()-startConnectTime) > MAX_CONNECTION_TIME;
        } // while(!bBaseConnected && !bTimeoutElapsed)

        if(!bBaseConnected) { // Connection timed out...
            // Probaly we should set the NOACK to avoid loosing the ACK message
            txBuffer[0] = connectionTimedOut;
            BSP_LED_On(LED_BLUE);
            rf24Enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
            rf24StartWrite();
            BSP_LED_Off(LED_BLUE);
            HAL_Delay(10);
        }

    } while(!bBaseConnected);

    ledsOff(); // Connection with the Remote established...
} // void connectBase()


void
connectRemote() {
    setRole(PRX);
    rf24SetRetries(5, 15);
    rf24Flush_tx();
    rf24Flush_rx();
    bRadioDataAvailable  = false;

    bool bRemoteConnected = false;
    bool bTimeoutElapsed  = false;
    BSP_LED_On(LED_GREEN);
    while(!bRemoteConnected) {
        bConnectionAccepted  = false;
        bConnectionRequested = false;
        bConnectionTimedOut  = false;
        bConnectionWanted    = false;
        while(!bRadioDataAvailable) {
        }
        bRadioDataAvailable  = false;

        BSP_LED_On(LED_ORANGE);
        rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
        BSP_LED_Off(LED_ORANGE);

        if(rxBuffer[0] == checkBaseRequestCmd) {
            if(bSendOpenGate) {
                bSendOpenGate = false;
                txBuffer[0] = openGateAck;
                BSP_LED_On(LED_BLUE);
                rf24WriteAckPayload(1, txBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE);
            }
            else if(bSendOpenCarGate) {
                bSendOpenCarGate = false;
                txBuffer[0] = openCarGateAck;
                BSP_LED_On(LED_BLUE);
                rf24WriteAckPayload(1, txBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE);
            }
            else if(bConnectionWanted) {
                bConnectionWanted = false;
                bRemoteConnected = true;
                bConnectionAccepted = true;
                txBuffer[0] = wantConnectAck;
                BSP_LED_On(LED_BLUE);
                rf24WriteAckPayload(1, txBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE);
                HAL_Delay(QUERY_INTERVAL+1); // Give time to the Base of receiving
                                             // the Ack and to convert into PRX mode
            }
        } // if(rxBuffer[0] == checkBaseRequestCmd)

        if(rxBuffer[0] == connectRequest) { // Connection Request received...
            bConnectionRequested = true;
            startAlarm();
            while(!bConnectionAccepted && !bConnectionTimedOut) {
                if(!updateAlarm()) {
                    bConnectionTimedOut = true;
                }
                if(bRadioDataAvailable) {
                    bRadioDataAvailable = false;
                    BSP_LED_On(LED_ORANGE);
                    rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
                    BSP_LED_Off(LED_ORANGE);
                    if(rxBuffer[0] == connectionTimedOut) {
                        bConnectionTimedOut = true;
                    }
                }
            } // while(!bConnectionAccepted && !bConnectionTimedOut)
            stopAlarm();

            if(!bConnectionTimedOut) {
                uint32_t startTimeout = HAL_GetTick();
                do { // Wait one more Connection Request...
                    if(bRadioDataAvailable) {
                        bRadioDataAvailable = false;
                        rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
                        if(rxBuffer[0] == connectRequest) { // Base is still asking for connection ?
                            txBuffer[0] = connectionAccepted;
                            BSP_LED_On(LED_BLUE);
                            rf24WriteAckPayload(1, txBuffer, MAX_PAYLOAD_SIZE);
                            BSP_LED_Off(LED_BLUE);
                            bRemoteConnected = true;
                            HAL_Delay(150); // Give time to send the Ack and to
                                            // convert the Base into PRX mode
                        }
                    }
                    if(HAL_GetTick() < startTimeout) // counter overflow
                        startTimeout = HAL_GetTick();
                    bTimeoutElapsed = (HAL_GetTick()-startTimeout) > MAX_WAIT_ACK_TIME;
                } while(!bRemoteConnected && !bTimeoutElapsed);
            } // if(!bConnectionTimedOut)

        } // if(rxBuffer[0] == connectRequest)

    } // while(!bRemoteConnected)

    // Connection with the Base established...
    BSP_LED_Off(LED_GREEN);
}


void
processBase() {
    uint8_t base;
    uint8_t offset;
    uint8_t pipeNum;
    bool bTimeoutElapsed;
    setRole(PRX); // Change role from PTX to PRX...
    rf24SetRetries(1, 0); // We have to be fast !!!
    rf24Flush_rx();
    rf24Flush_tx();

    adcInit();
    adcTIM2Init();
    // Enables ADC DMA requests and enables ADC peripheral
    HAL_ADC_Start_DMA(&hAdc, (uint32_t*)adcDataIn, 2*MAX_PAYLOAD_SIZE);
    chunk2Write = 0;
    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, DEFAULT_AUDIO_IN_FREQ);
    BSP_AUDIO_OUT_Play(Audio_Out_Buffer, 2*MAX_PAYLOAD_SIZE);
    // Enable ADC periodic sampling
    HAL_TIM_Base_Start(&Tim2Handle);

    // Set the initial status....
    bSuspend = false;
    bGateOpening = false;
    bCarGateOpening = false;
    bRadioDataAvailable = false;

    uint32_t startTimeout = HAL_GetTick();
    do { // Loop until disconnected or contact lost...

        if(bRadioDataAvailable) {
            bRadioDataAvailable = false;
            startTimeout = HAL_GetTick();
            rf24Available(&pipeNum);
            BSP_LED_On(LED_ORANGE);
            rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
            BSP_LED_Off(LED_ORANGE);
            if(pipeNum == 1) { // The packet contains Audio Data:
                // At first replay with our audio data...
                BSP_LED_On(LED_BLUE); // Signal the packet's start reading
                rf24WriteAckPayload(pipeNum, txBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE); // Reading done
                // Then place the data in the current Audio output chunk...
                base = chunk2Write*MAX_PAYLOAD_SIZE;
                for(uint8_t indx=0; indx<MAX_PAYLOAD_SIZE; indx++) {
                    offset = (base+indx) << 1;
                    Audio_Out_Buffer[offset]   = rxBuffer[indx] << 8; // 1st Stereo Channel
                    Audio_Out_Buffer[offset+1] = rxBuffer[indx] << 8; // 2nd Stereo Channel
                }
            }
            else { // The packet is a command
                HAL_TIM_Base_Stop(&Tim2Handle); // Avoid interference with Audio Data...
                BSP_AUDIO_OUT_Pause();
                processReceivedCommand(rxBuffer[0], pipeNum);
                BSP_AUDIO_OUT_Resume();
                HAL_TIM_Base_Start(&Tim2Handle); // Restore Audio Data sampling...
            } // We have done with the new data.

        } // if(bRadioDataAvailable)
        if(HAL_GetTick() < startTimeout) // counter overflow
            startTimeout = HAL_GetTick();
        bTimeoutElapsed = (HAL_GetTick()-startTimeout) > MAX_NO_SIGNAL_TIME;

    } while(!bSuspend && !bTimeoutElapsed);
    ledsOff();

    // Connection terminated...
    HAL_TIM_Base_Stop(&Tim2Handle);    // Stop ADC sampling...
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW); // Stop reproducing audio and power down the Codec
    HAL_Delay(300);
}


void
processRemote() {
    uint8_t base;
    uint8_t offset;
    setRole(PTX); // Change role from PRX to PTX...
    rf24SetRetries(1, 0); // We have to be fast !!!
    rf24Flush_tx();
    rf24Flush_rx();

    BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, DEFAULT_AUDIO_IN_FREQ);
    chunk2Write = 0;
    BSP_AUDIO_OUT_Play(Audio_Out_Buffer, 2*MAX_PAYLOAD_SIZE);
    BSP_AUDIO_IN_Record(pdmDataIn, INTERNAL_BUFF_SIZE);

    // Set the initial status...
    bSuspend          = false;
    bBaseDisconnected = false;
    bSendOpenGate     = false;
    bSendOpenCarGate  = false;
    bReady2Send       = false;
    bRadioIrq         = true; // To force sending as soon as we have data
    bool bTimeoutElapsed;
    uint32_t startTimeout = HAL_GetTick();
    do {
        if(bReady2Send && bRadioIrq) { // We will send data only when available and
            bReady2Send = false;       // the previous data were sent or lost !
            bRadioIrq = false;
            BSP_LED_On(LED_BLUE);
            rf24Enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
            rf24StartWrite();     // buffer prepared by the Microphone
            BSP_LED_Off(LED_BLUE); // Reading done
        }
        if(bRadioDataAvailable) { // It's an Acknowledge Packet containing Audio
            bRadioDataAvailable = false;
            BSP_LED_On(LED_ORANGE); // Signal the packet's start reading
            rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
            BSP_LED_Off(LED_ORANGE); // Reading done
            // Write data in the current Audio Buffer chunk
            base = chunk2Write*MAX_PAYLOAD_SIZE;
            for(uint8_t indx=0; indx<MAX_PAYLOAD_SIZE; indx++) {
                offset = (base+indx) << 1;
                Audio_Out_Buffer[offset]   = rxBuffer[indx] << 8; // 1st Stereo Channel
                Audio_Out_Buffer[offset+1] = rxBuffer[indx] << 8; // 2nd Stereo Channel
            }
            startTimeout = HAL_GetTick();
            // We have done with the new data...
        } // if(bRadioDataAvailable)

        if(bSuspend) {
            sendCommand(suspendCmd); // It also set bBaseDisconnected to true
        }

        if(bSendOpenGate) {
            sendCommand(openGateCmd);
            bSendOpenGate    = false;
        }

        if(bSendOpenCarGate) {
            sendCommand(openCarGateCmd);
            bSendOpenCarGate = false;
        }

        if(HAL_GetTick() < startTimeout) // counter overflow
            startTimeout = HAL_GetTick();
        bTimeoutElapsed = (HAL_GetTick()-startTimeout) > MAX_NO_SIGNAL_TIME;
    } while(!bBaseDisconnected && !bTimeoutElapsed);

    // Connection terminated...
    BSP_AUDIO_IN_Stop();
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW); // Stop reproducing audio and switch off the codec
    ledsOff();
    HAL_Delay(150);
}


void
processReceivedCommand(uint8_t command, uint8_t sourcePipe) {
    if(command == suspendCmd) {
        txBuffer[0] = suspendAck;
        bSuspend = true;
    }
    else if(command == openGateCmd) {
        txBuffer[0] = openGateAck;
        if(!bGateOpening)
            relayPulse(GATE_RELAY_TIM_CHANNEL, 1500);
        bGateOpening = true; // To avoid processing further equal commands
    }
    else if(command == openCarGateCmd) {
        txBuffer[0] = openCarGateAck;
        if(!bCarGateOpening)
            relayPulse(CAR_GATE_RELAY_TIM_CHANNEL, 1500);
        bCarGateOpening = true; // To avoid processing further equal commands
    }
    rf24WriteAckPayload(sourcePipe, txBuffer, MAX_PAYLOAD_SIZE);
    HAL_Delay(1); // Is this needed ???
}


void
sendCommand(uint8_t command) {
    uint8_t answer;
    BSP_AUDIO_IN_Stop(); // Ensure no other process can modify txBuffer
    rf24OpenWritingPipe(pipes[2]);
    rf24SetRetries(5, 15);
    rf24Flush_tx();     // Empty transmission queue
    uint32_t startTimeout = HAL_GetTick();
    uint32_t t0 = HAL_GetTick()-QUERY_INTERVAL-1; // Just to force the first send.
    bool bTimeout       = false;
    bBaseDisconnected   = false;
    bool bCommandDone   = false;
    ledsOff();
    txBuffer[0] = command;
    do {
        if(HAL_GetTick()-t0 > QUERY_INTERVAL) { // Send the command
            t0 = HAL_GetTick();
            BSP_LED_On(LED_BLUE);
            rf24Enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
            rf24StartWrite();
            BSP_LED_Off(LED_BLUE);
        }
        if(bRadioDataAvailable) { // Get the Ack
            bRadioDataAvailable = false;
            BSP_LED_On(LED_ORANGE); // Signal the packet's start reading
            rf24Read(rxBuffer, MAX_PAYLOAD_SIZE);
            BSP_LED_Off(LED_ORANGE); // Reading done
            answer = *rxBuffer;
            if(answer == suspendAck) {
                bBaseDisconnected = true;
                bCommandDone = true;
            }
            if(answer == openGateAck) {
                bCommandDone = true;
            }
            if(answer == openCarGateAck) {
                bCommandDone = true;
            }
        } // if(bRadioDataAvailable)

        if(HAL_GetTick() < startTimeout) // counter overflow
            startTimeout = HAL_GetTick();
        bTimeout = (HAL_GetTick() - startTimeout) > MAX_WAIT_ACK_TIME;
    } while(!bCommandDone && !bTimeout);

    if(!bBaseDisconnected) {
        rf24OpenWritingPipe(pipes[0]);
        rf24SetRetries(1, 0);
        BSP_AUDIO_IN_Record(pdmDataIn, INTERNAL_BUFF_SIZE); // Restart sending Audio Data
    }
    HAL_Delay(150);
}


void
startAlarm() {
    if(!prepareFileSystem())
        Error_Handler();
    buffer_offset = BUFFER_OFFSET_NONE;
    UINT bytesread = 0;
    WaveDataLength = 0;
    AudioRemSize = 0;
    if(f_open(&File2Read, WAVE_NAME , FA_READ) != FR_OK) {
        Error_Handler();
    }
    // Read the wav file header
    f_read (&File2Read, &waveformat, sizeof(waveformat), &bytesread);
    WaveDataLength = waveformat.FileSize;
    if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, waveformat.SampleRate) != AUDIO_OK) {
        Error_Handler();
    }
    Audio_Buffer = (uint8_t*)malloc(AUDIO_BUFFER_SIZE*sizeof(*Audio_Buffer));
    f_lseek(&File2Read, 0);
    f_read (&File2Read, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
    AudioRemSize = WaveDataLength - bytesread;
    BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
}


bool
updateAlarm() {
    UINT bytesread = 0;
    if(buffer_offset == BUFFER_OFFSET_HALF) {
        f_read(&File2Read,
               &Audio_Buffer[0],
               AUDIO_BUFFER_SIZE/2,
               &bytesread);
        buffer_offset = BUFFER_OFFSET_NONE;
    }
    if(buffer_offset == BUFFER_OFFSET_FULL) {
        f_read(&File2Read,
               &Audio_Buffer[AUDIO_BUFFER_SIZE/2],
               AUDIO_BUFFER_SIZE/2,
               &bytesread);
        buffer_offset = BUFFER_OFFSET_NONE;
    }
    if(AudioRemSize > (AUDIO_BUFFER_SIZE / 2)) {
        AudioRemSize -= bytesread;
    }
    else {
        AudioRemSize = 0;
        return false;
    }
    return true;
}


void
stopAlarm() {
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
    free(Audio_Buffer);
    f_close(&File2Read);
    closeFileSystem();
}


void
relayTIM3Init() {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef     sConfigOC;

  __HAL_RCC_TIM3_CLK_ENABLE();

  // Compute the prescaler value, to have TIM3Freq = 1KHz
  uint32_t uwPrescalerValue = (uint32_t)((SystemCoreClock) / 1000) - 1;

  Tim3Handle.Instance = TIM3;
  Tim3Handle.Init.Prescaler         = uwPrescalerValue;
  Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Tim3Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  Tim3Handle.Init.Period            = 300;
  Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  if(HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if(HAL_TIM_ConfigClockSource(&Tim3Handle, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  if(HAL_TIM_OC_Init(&Tim3Handle) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.Pulse      = 10; // will be overwritten
  sConfigOC.OCMode     = TIM_OCMODE_TIMING;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if(HAL_TIM_OC_ConfigChannel(&Tim3Handle, &sConfigOC, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM3_IRQn, RELAY_PULSE_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}


void
relayPulse(uint32_t relayChannel, uint16_t msPulse) {
    GPIO_TypeDef* port;
    uint32_t pin;
    if(relayChannel == GATE_RELAY_TIM_CHANNEL) {
        port = GATE_RELAY_GPIO_PORT;
        pin  = GATE_RELAY_GPIO_PIN;
    }
    if(relayChannel == CAR_GATE_RELAY_TIM_CHANNEL) {
        port = CAR_GATE_RELAY_GPIO_PORT;
        pin  = CAR_GATE_RELAY_GPIO_PIN;
    }
    if(relayChannel == FREE1_RELAY_TIM_CHANNEL) {
        port = FREE1_RELAY_GPIO_PORT;
        pin  = FREE1_RELAY_GPIO_PIN;
    }
//    if(relayChannel == FREE2_RELAY_TIM_CHANNEL) {
//        port = FREE2_RELAY_GPIO_PORT;
//        pin  = FREE2_RELAY_GPIO_PIN;
//    }

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); // will reset on interrupt service routine

    Tim3Handle.Instance->ARR = msPulse;
    __HAL_TIM_CLEAR_IT(&Tim3Handle, relayChannel);
    if(HAL_TIM_OC_Start_IT(&Tim3Handle, relayChannel) != HAL_OK) {
      Error_Handler();
    }
}


bool
prepareFileSystem() {
    AppliState = APPLICATION_IDLE;

    if(FATFS_LinkDriver(&SD_Driver, DiskPath) != 0)
        return false;
    // Initializes (mount) the File System
    if(f_mount(&DiskFatFs, (TCHAR const*)DiskPath, 0) != FR_OK)
        return false;
    if(f_opendir(&Directory, path) != FR_OK)
        return false;

    AppliState = APPLICATION_START;
    return true;
}


bool
closeFileSystem() {
    f_closedir(&Directory);
    f_mount(NULL, (TCHAR const*)"", 0);
    FATFS_UnLinkDriverEx(DiskPath, 0);
    AppliState = APPLICATION_IDLE;
    return true;
}


void
ledsOff() {
    BSP_LED_Off(LED_ORANGE);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_BLUE);
    BSP_LED_Off(LED_RED);
}


void
pushButtonInit(GPIO_TypeDef* Port, uint32_t Pin, IRQn_Type Irq) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin   = Pin;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(Port, &GPIO_InitStruct);

    // Enable and set Button EXTI Interrupt to the lowest priority
    HAL_NVIC_SetPriority(Irq, PUSH_BTN_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(Irq);
}


// TIM3
void
relayGPIOInit(GPIO_TypeDef* Port, uint32_t Pin) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin       = Pin;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
}



void
initBuffers(bool isBaseStation) {
    // Allocate common buffers... (The receive buffer and the Audio Out one)
    if(!rxBuffer)
        rxBuffer = (uint8_t*)malloc(MAX_PAYLOAD_SIZE*sizeof(*rxBuffer));
    if(!Audio_Out_Buffer)
        Audio_Out_Buffer = (uint16_t*)malloc(2*2*MAX_PAYLOAD_SIZE*sizeof(*Audio_Out_Buffer));
    memset(rxBuffer,         0, MAX_PAYLOAD_SIZE*sizeof(*rxBuffer));
    memset(Audio_Out_Buffer, 0, 2*2*MAX_PAYLOAD_SIZE*sizeof(*Audio_Out_Buffer));

    if(isBaseStation) {
        if(!adcDataIn)
            adcDataIn = (uint16_t*)malloc(2*MAX_PAYLOAD_SIZE*sizeof(*adcDataIn));
        if(!txBuffer)
            txBuffer  = (uint8_t *)malloc(2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
        memset(adcDataIn,  0, 2*MAX_PAYLOAD_SIZE*sizeof(*adcDataIn));
        memset(txBuffer,   0, 2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
    }
    else {
        if(!txBuffer)
            txBuffer = (uint8_t *)malloc(2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
        if(!pdmDataIn)
            pdmDataIn = (uint16_t*)malloc(INTERNAL_BUFF_SIZE*sizeof(*pdmDataIn));
        if(!pcmDataOut)
            pcmDataOut = (uint16_t*)malloc(2*PCM_OUT_SIZE*sizeof(*pcmDataOut));
        memset(txBuffer,   0, 2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
        memset(pdmDataIn,  0, INTERNAL_BUFF_SIZE*sizeof(*pdmDataIn));
        memset(pcmDataOut, 0, 2*PCM_OUT_SIZE*sizeof(*pcmDataOut));
    }
}


void
BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params) {
    // Ensure the clock is the same both for MIC input and DAC output
    BSP_AUDIO_IN_ClockConfig(hi2s, AudioFreq, Params);
}


void
setRole(bool bPTX) {
    if(bPTX) {
        rf24StopListening(); // Change role from PRX to PTX...
        rf24OpenWritingPipe(pipes[0]);
        for(uint8_t i=1; i<6; i++) {
            rf24OpenReadingPipe(i, pipes[i]);
        }
    }
    else {
        rf24OpenWritingPipe(pipes[1]);
        rf24OpenReadingPipe(1, pipes[0]);
        for(uint8_t i=2; i<6; i++) {
            rf24OpenReadingPipe(i, pipes[i]);
        }
        rf24StartListening();
    }
}


/// ADC MSP Initialization
void
HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
    GPIO_InitTypeDef          GPIO_InitStruct;
    static DMA_HandleTypeDef  hdma_adc;

    ADC1_CHANNEL_GPIO_CLK_ENABLE(); // Enable GPIO clock
    ADC1_CLK_ENABLE();  // ADC Periph clock enable
    ADC1_DMA_CLK_ENABLE();  // Enable DMA2 clock

    // Configure peripheral GPIO
    // ADC Channel GPIO pin configuration
    GPIO_InitStruct.Pin  = ADC1_CHANNEL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

    // Configure the DMA streams
    // Set the parameters to be configured
    hdma_adc.Instance = ADC1_DMA_STREAM;

    hdma_adc.Init.Channel             = ADC1_DMA_CHANNEL;
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode                = DMA_CIRCULAR;
    hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_adc.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adc.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_adc.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    hdma_adc.XferCpltCallback         = NULL;
    hdma_adc.XferHalfCpltCallback     = NULL;
    hdma_adc.XferM1HalfCpltCallback   = NULL;

    HAL_DMA_Init(&hdma_adc);

    // Associate the initialized DMA handle to the the ADC handle
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

    // Configure the NVIC for DMA
    // NVIC configuration for DMA transfer complete interrupt
    HAL_NVIC_SetPriority(ADC1_DMA_IRQn, ADC_DMA_IRQ_PREPRIO, 0);
    HAL_NVIC_EnableIRQ(ADC1_DMA_IRQn);
}


// The output register of the ADCs is a 16 bit register
// even when the resolution is <= 8 bit
void
adcInit() {
    hAdc.Instance = ADC1;

    hAdc.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
    hAdc.Init.Resolution            = ADC_RESOLUTION_12B;
    hAdc.Init.ScanConvMode          = DISABLE;
    hAdc.Init.ContinuousConvMode    = DISABLE;
    hAdc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hAdc.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hAdc.Init.DataAlign             = ADC_DATAALIGN_LEFT;
    hAdc.Init.NbrOfConversion       = 1;
    hAdc.Init.DMAContinuousRequests = ENABLE;
    hAdc.Init.EOCSelection          = DISABLE;
    if(HAL_ADC_Init(&hAdc) != HAL_OK) {
        Error_Handler();
    }
    // Configure ADC regular channel
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel      = ADC1_CHANNEL;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    sConfig.Offset       = 0;
    if(HAL_ADC_ConfigChannel(&hAdc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}


// Timer2 provides periodic triggers to start ADC conversion
void
adcTIM2Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    // Clock enable
    TIM2_CLK_ENABLE();
    // Time base configuration
    // Prescaler value to have a 400 KHz TIM2 counter clock
    const uint32_t counterClock = SystemCoreClock/100;// 1680000;
    uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock/2)/counterClock)-1;

    memset(&Tim2Handle, 0, sizeof(Tim2Handle));
    Tim2Handle.Instance = TIM2;
    Tim2Handle.Init.Period            = (counterClock/DEFAULT_AUDIO_IN_FREQ)-1;
    Tim2Handle.Init.Prescaler         = uwPrescalerValue;
    Tim2Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1; // tDTS=tCK_INT
    Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    Tim2Handle.Init.RepetitionCounter = 0;
    Tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&Tim2Handle) != HAL_OK) {
        Error_Handler();
    }
    memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&Tim2Handle, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&Tim2Handle, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}


// extern "C" {


void
EXTI15_10_IRQHandler(void) { // We received a radio interrupt...
    __HAL_GPIO_EXTI_CLEAR_IT(NRF24_IRQ_PIN);

    // Read & reset the IRQ status
    rf24WhatHappened(&txOk, &txFailed, &rxDataReady);

    if(rxDataReady) {
        bRadioDataAvailable = true;
    }

    if(txOk) { // TX_DS IRQ asserted when the ACK packet has been received.
        BSP_LED_Off(LED_RED); // Reset previous errors signal
    }

    if(txFailed) { // nRF24L01+ asserts the IRQ pin when MAX_RT is reached
                   // but the payload in TX FIFO is NOT removed!
        BSP_LED_On(LED_RED);
        rf24Flush_tx(); // Remove messages from the TX queue
    }
    bRadioIrq = true;
}


/// ADC Conversion_Complete callback
void
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hAdc) {
    UNUSED(hAdc);
    for(uint8_t i=0; i<MAX_PAYLOAD_SIZE; i++) {
        txBuffer[i] = (adcDataIn[i+MAX_PAYLOAD_SIZE] >> 8) & 0xFF;
    }
}


/// ADC Conversion_Half_Complete callback
void
HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hAdc) {
    UNUSED(hAdc);
    for(uint16_t i=0; i<MAX_PAYLOAD_SIZE; i++) {
        txBuffer[i] = (adcDataIn[i] >> 8) & 0xFF;
    }
}


/// ADC Error callback
void
HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    UNUSED(hadc);
    Error_Handler();
}


void
BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    if(bConnectionAccepted || bBaseConnected) {
        // Send the last received chunk to DAC
        uint8_t offset = chunk2Write*MAX_PAYLOAD_SIZE*2;
        BSP_AUDIO_OUT_ChangeBuffer(&Audio_Out_Buffer[offset], 2*MAX_PAYLOAD_SIZE);
        // Be ready for the next chunk
        chunk2Write = 1-chunk2Write;
        offset = chunk2Write*MAX_PAYLOAD_SIZE*2;
        memset(&Audio_Out_Buffer[offset], 0, 2*MAX_PAYLOAD_SIZE*sizeof(*Audio_Out_Buffer));
    }
    else {
        buffer_offset = BUFFER_OFFSET_FULL;
        BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE/2);
    }
}


void
BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    buffer_offset = BUFFER_OFFSET_HALF;
}


void
BSP_AUDIO_OUT_Error_CallBack(void) {
    /* Stop the program with an infinite loop */
    while (1)
    {}
    /* Could also generate a system reset to recover from the error */
    /* .... */
}


void
BSP_AUDIO_IN_TransferComplete_CallBack(void) {
    BSP_LED_On(LED_GREEN);
    BSP_AUDIO_IN_PDMToPCM(&pdmDataIn[INTERNAL_BUFF_SIZE/2], pcmDataOut);
    for(uint16_t i=0; i<PCM_OUT_SIZE; i++) {
        txBuffer[i+PCM_OUT_SIZE] = (pcmDataOut[i<<1] >> 8) & 0xFF;
    }
    bReady2Send = true;
    BSP_LED_Off(LED_GREEN);
}


void
BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
    BSP_AUDIO_IN_PDMToPCM(pdmDataIn, pcmDataOut);
    for(uint16_t i=0; i<PCM_OUT_SIZE; i++) {
        txBuffer[i] = (pcmDataOut[i<<1] >> 8) & 0xFF;
    }
}


void
BSP_AUDIO_IN_Error_Callback(void) {
    Error_Handler();
}


/// This function handles ADC DMA interrupt request.
void
DMA2_Stream0_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAdc.DMA_Handle);
}


/// SPI error callbacks
void
HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    UNUSED(hspi);
    uint32_t t0 = HAL_GetTick();
    BSP_LED_Off(LED4);
    BSP_LED_Off(LED5);
    BSP_LED_Off(LED6);
    while(HAL_GetTick()-t0 < 2000) {
        HAL_Delay(100);
        BSP_LED_Toggle(LED4);
        BSP_LED_Toggle(LED5);
        BSP_LED_Toggle(LED6);
    }
}


void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == PHONE_BUTTON_GPIO_PIN) {
        if(isBaseStation) {
            bBaseSleeping = false;
            startConnectTime = HAL_GetTick();
        }
        else {
            if(bConnectionRequested) {
                bConnectionRequested = false;
                bConnectionAccepted = true;
            }
            else if(bConnectionAccepted) {
                bSuspend = true;
            }
            else {
                bConnectionWanted = true;
            }
        }
    }
    if(GPIO_Pin == GATE_BUTTON_GPIO_PIN) {
        bSendOpenGate = true;
    }
    if(GPIO_Pin == CAR_GATE_BUTTON_GPIO_PIN) {
        bSendOpenCarGate = true;
    }
}


/// This function handles main I2S interrupt.
void
I2S3_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}


/// This function handles DMA Stream interrupt request.
void
I2S2_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}


void
HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t channel;
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_FLAG_CC1) != RESET) {
        port = FREE1_RELAY_GPIO_PORT;
        pin  = FREE1_RELAY_GPIO_PIN;
        channel = FREE1_RELAY_TIM_CHANNEL;
    }
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_FLAG_CC2) != RESET) {
        port = GATE_RELAY_GPIO_PORT;
        pin  = GATE_RELAY_GPIO_PIN;
        channel = GATE_RELAY_TIM_CHANNEL;
        bGateOpening = false;
    }
//    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_FLAG_CC3) != RESET) {
//        port = FREE2_RELAY_GPIO_PORT;
//        pin  = FREE2_RELAY_GPIO_PIN;
//        channel = FREE2_RELAY_TIM_CHANNEL;
//    }
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_FLAG_CC4) != RESET) {
        port = CAR_GATE_RELAY_GPIO_PORT;
        pin  = CAR_GATE_RELAY_GPIO_PIN;
        channel = CAR_GATE_RELAY_TIM_CHANNEL;
        bCarGateOpening = false;
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_TIM_OC_Stop_IT(htim, channel);
}


void
TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&Tim3Handle);
}


void
initLeds() {
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);
}


void
configPinInit() {
    GPIO_InitTypeDef GPIO_InitStructure;
    // Initialize CONFIGURE_PIN as Input with Pulldown
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStructure.Pin   = CONFIGURE_PIN;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    HAL_GPIO_Init(CONFIGURE_PORT, &GPIO_InitStructure);
}


void
configPinDeinit() {
    __HAL_RCC_GPIOC_CLK_DISABLE();
}


void
Error_Handler(void) {
    ledsOff();
    while(1) {
        HAL_Delay(80);
        BSP_LED_Toggle(LED3);
        HAL_Delay(80);
        BSP_LED_Toggle(LED4);
        HAL_Delay(80);
        BSP_LED_Toggle(LED5);
        HAL_Delay(80);
        BSP_LED_Toggle(LED6);
    }
}


/// System Clock Configuration
static void
SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    // Enable Power Control clock
    __HAL_RCC_PWR_CLK_ENABLE();
    // The voltage scaling allows optimizing the power consumption when the device is
    // clocked below the maximum system frequency, to update the voltage scaling value
    // regarding system frequency refer to product datasheet.
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    // Enable HSE Oscillator and activate PLL with HSE as source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
    // clocks dividers
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
    // on STM32F405x/407x/415x/417x Revision Z devices
    // prefetch is supported
    if (HAL_GetREVID() == 0x1001) {
        // Enable the Flash prefetch
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}


void
_delay_3t(uint32_t cycles) {
    asm __volatile__ (
                ".syntax unified\n"
                "1: \n"
                "	subs %[cyc],#1 \n"	/* 1Tck */
                "	bne 1b \n"		/* 2Tck */
                "	bx lr \n"
                ".syntax divided\n"
                : /* No output */
                : [cyc] "r" (cycles)
                : /* No memory */
                );
}


void
delay_cycles(const int64_t cycles) {
    if(cycles < 0) return;
    switch(cycles % 3) {
    default:
    case 0: break;
    case 1: asm __volatile__ ("nop"); break;
    case 2: asm __volatile__ ("nop\nnop"); break;
    }
    if(cycles > 3)
        _delay_3t((uint32_t)(cycles / 3));
    else /* same delay as the function call */
        asm __volatile__ ("nop\nnop\nnop\nnop\nnop\nnop\n");
}


void
delayMicroseconds(uint32_t us) {
    if(us == 0)
        return;
    delay_cycles(us * (SystemCoreClock/1000000)-16);
};


#ifdef  USE_FULL_ASSERT
void
assert_failed(uint8_t* file, uint32_t line) {
    sprintf(buf, "Wrong parameters value: file %s on line %d\r\n", file, (unsigned int)line);
    // Infinite loop
    while(1) {
    }
}
#endif
