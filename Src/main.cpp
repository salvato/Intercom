#include "main.h"
#include "RF24.h"
#include "printf.h"
#include "string.h"
#include "stdlib.h"
#include "nRF24L01.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio_dma.h"
#include "waveplayer.h"




//====================================================================
// The STM32F4Discovery board can supply ~100mA @ 3.3V and ~100mA @ 5V
//====================================================================



//------------------------------------------------------------------------------
//             Audio Driver Configuration parameters
//--------------------------------------------------------------------------------
// DEFAULT_AUDIO_IN_FREQ                 I2S_AUDIOFREQ_16K
// DEFAULT_AUDIO_IN_BIT_RESOLUTION       16
// DEFAULT_AUDIO_IN_CHANNEL_NBR          1 // Mono = 1, Stereo = 2
// DEFAULT_AUDIO_IN_VOLUME               64
// PDM buffer input size
//     INTERNAL_BUFF_SIZE                128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR
// PCM buffer output size
//     PCM_OUT_SIZE                      DEFAULT_AUDIO_IN_FREQ/1000
// CHANNEL_DEMUX_MASK                    0x55
//-------------------------------------------------------------------------------



//===============================================================
//                 Used Resources
//===============================================================
//  SPI3 (I2S3)         CS43L22     (Audio OUT)
//  DMA1 CH0 Stream7    CS43L22     (Audio OUT)
//  SPI2 (I2S2)         MP45DT02    (Audio IN)
//  DMA1 CH0 Stream3    MP45DT02    (Audio IN)
//  ADC1 CH1            STM32F4     (Adc IN)
//  DMA2 CH0 Stream0    STM32F4     (Adc IN)
//  TIM2                STM32F4     (Adc Trigger)
//  SPI1                NRF24L01    (Radio RxTx)
//===============================================================


//===============================================================
//                 CS43L22 Pinout (use I2S3==SPI3)
//===============================================================
// PA4      LRCK
// PA10     IRQ
// PB6      SCL
// PB9      SDA
// PC7      MCLK
// PC10     SCLK
// PC12     SDIN
// PD4      RESET
//===============================================================


//===============================================================
//                 MP45DT02 Microphone Pinout (use I2S2==SPI2)
//===============================================================
// PB10     CLK
// PC3      DOUT (MOSI)
//===============================================================


//===============================================================
//                 CONFIGURE_PIN
// MUST be connected to GND in the Slave Station
//===============================================================
// PC1      CONFIGURE_PIN
//===============================================================
#define CONFIGURE_PIN  GPIO_PIN_1
#define CONFIGURE_PORT GPIOC


//===============================================================
//                 nRF24L01+ (Use SPI1)
// nRF24L01+   STM32Fxxx     DESCRIPTION
//===============================================================
// (1) GND       GND         Ground
// (2) VCC       3.3V        3.3V
// (3) CE        PD8         RF activated pin
// (4) CSN       PD7         Chip select pin for SPI
// (5) SCK       PA5         SCK pin for SPI1
// (6) MOSI      PA7         MOSI pin for SPI1
// (7) MISO      PA6         MISO pin for SPI1
// (8) IRQ       PA10        Interrupt pin. Goes low when active.
//===============================================================
#define NRF24_CE_PORT     GPIOD
#define NRF24_CE_PIN      GPIO_PIN_8
#define NRF24_CSN_PORT    GPIOD
#define NRF24_CSN_PIN     GPIO_PIN_7
#define NRF24_IRQ_PORT    GPIOA
#define NRF24_IRQ_PIN     GPIO_PIN_10
#define NRF24_IRQ_CHAN    EXTI15_10_IRQn
#define PTX               true
#define PRX               !PTX


//===============================================================
//                 ADC
//===============================================================
// PA1      ADC1 Channel1 with DMA2 Channel0 Stream0
//===============================================================
#define ADC1_CHANNEL_GPIO_PORT          GPIOA
#define ADC1_CHANNEL_PIN                GPIO_PIN_1
#define ADC1_CHANNEL                    ADC_CHANNEL_1
#define ADC1_DMA_CHANNEL                DMA_CHANNEL_0
#define ADC1_DMA_STREAM                 DMA2_Stream0
#define ADC1_DMA_IRQn                   DMA2_Stream0_IRQn
#define ADC1_DMA_IRQHandler             DMA2_Stream0_IRQHandler
#define ADC1_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC1_DMA_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()
#define ADC1_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC1_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADC1_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()


//===============================================================
//                 TIM2 (ADC Sampling Clock)
//===============================================================
#define TIM2_CLK_ENABLE()               __HAL_RCC_TIM2_CLK_ENABLE()


//===============================================================
//                 LEDs by Colours
//===============================================================
#define LED_ORANGE  LED3
#define LED_GREEN   LED4
#define LED_RED     LED5
#define LED_BLUE    LED6


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


//===============================================================
// Local Functions
//===============================================================
static void SystemClock_Config(void);
static void InitConfigPin();
static void initLeds();
static void InitADC();
static void Init_TIM2_Adc(void);
static void initBuffers(bool isBaseStation);
static void setRole(bool bPTX);
static void ledsOff();
static void connectBase();
static void connectRemote();
static void processBase();
static void processRemote();


char buf[255];


TIM_HandleTypeDef  Tim2Handle;
ADC_HandleTypeDef  hAdc;
USBH_HandleTypeDef hUSB_Host; // USB Host handle
extern HCD_HandleTypeDef hhcd; // defined in usbh_conf.c


// nRF Addresses
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


RF24
rf24(NRF24_CE_PORT,  NRF24_CE_PIN,
     NRF24_CSN_PORT, NRF24_CSN_PIN,
     NRF24_IRQ_PORT, NRF24_IRQ_PIN, NRF24_IRQ_CHAN);

// Data Buffers
uint8_t*  rxBuffer         = NULL;
uint8_t*  txBuffer         = NULL;
uint16_t* adcDataIn        = NULL;
uint16_t* Audio_Out_Buffer = NULL;
uint16_t* pdmDataIn        = NULL;
uint16_t* pcmDataOut       = NULL;
uint16_t  offset;


uint32_t chunk = 0;
bool     isBaseStation;
uint8_t  status;


__IO bool tx_ok;
__IO bool tx_failed;
__IO bool rx_data_ready;
__IO bool bRadioIrq;
__IO bool bReady2Send;
__IO bool bReady2Play;
__IO bool bBaseSleeping;
__IO bool bConnectionRequested;
__IO bool bConnectionAccepted;
__IO bool bSuspend;      // true if the Remote ask to suspend the connection
__IO bool bRadioDataAvailable;
__IO uint32_t startConnectTime;


// Commands
typedef enum {
    connectRequest = 0x10,
    connectionAccepted,
    connectionTimedOut,
    suspendCmd,
    suspendAck
} Commands;


#define MAX_CONNECTION_TIME  15000
#define QUERY_INTERVAL       300

/* State Machine for the USBH_USR_ApplicationState */
#define USBH_USR_FS_INIT    ((uint8_t)0x00)
#define USBH_USR_AUDIO      ((uint8_t)0x01)

FATFS USBDISKFatFs;          /* File system object for USB disk logical drive */
char USBDISKPath[4];         /* USB Host logical drive path */

MSC_ApplicationTypeDef AppliState = APPLICATION_START;//APPLICATION_IDLE;
static uint8_t  USBH_USR_ApplicationState = USBH_USR_FS_INIT;

/* Re-play Wave file status on/off. Defined as external in waveplayer.c file */
__IO uint32_t RepeatState = REPEAT_ON;

/* Wave Player Pause/Resume Status. Defined as external in waveplayer.c file */
__IO uint32_t PauseResumeStatus = IDLE_STATUS;

/* Counter for User button presses. Defined as external in waveplayer.c file */
__IO uint32_t PressCount = 0;

__IO uint32_t CmdIndex = CMD_PLAY;

extern __IO BUFFER_StateTypeDef buffer_offset;


void
USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId) {
    UNUSED(pHost);
    switch (vId) {
        case HOST_USER_SELECT_CONFIGURATION:
            break;

        case HOST_USER_DISCONNECTION:
            WavePlayer_CallBack();
            AppliState = APPLICATION_IDLE;
            f_mount(NULL, (TCHAR const*)"", 0);
            break;

        case HOST_USER_CLASS_ACTIVE:
            AppliState = APPLICATION_START;
            break;

        case HOST_USER_CONNECTION:
            break;

        default:
            break;
    }
}



void
COMMAND_AudioExecuteApplication(void) {
    /* Execute the command switch the command index */
    switch (CmdIndex) {
        /* Start Playing from USB Flash memory */
        case CMD_PLAY:
            if (RepeatState == REPEAT_ON)
                WavePlayerStart();
            break;
        default:
            break;
    }
}


void
MSC_Application(void) {
    switch(USBH_USR_ApplicationState) {
        case USBH_USR_AUDIO:
            COMMAND_AudioExecuteApplication(); // Go to Audio menu
            USBH_USR_ApplicationState = USBH_USR_FS_INIT; // Set user initialization flag
            break;

        case USBH_USR_FS_INIT:
            // Initializes the File System
            if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) {
                Error_Handler();
            }
            USBH_USR_ApplicationState = USBH_USR_AUDIO; // Go to menu
            break;

        default:
            break;
    }
}


int
main(void) {
    const uint8_t Channel = 76;
    uint8_t Volume        = 70;   // % of Max

    // System startup
    HAL_Init();
    initLeds();
    SystemClock_Config();
    // Are we Base station or Remote ?
    InitConfigPin();
    isBaseStation = HAL_GPIO_ReadPin(CONFIGURE_PORT, CONFIGURE_PIN);
    // Initialize the corresponding things..
    initBuffers(isBaseStation);
    // Init Push Button(s)
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
    if(!rf24.begin(Channel, RADIO_IN_IRQ_PREPRIO)) Error_Handler();
    rf24.clearInterrupts();// Avoid false interrupts from Radio
    rf24.maskIRQ(false, false, false);

    while(1) {
        //=====================
        // Connection Phase....
        //=====================
        if(isBaseStation)
            connectBase();// We will be woken up by a button press
        else
            connectRemote();// We will be woken up by receiving a command
        //=======================
        // Connection Established
        //=======================
        bSuspend = false;
        if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO,
                              Volume,
                              DEFAULT_AUDIO_IN_FREQ) != AUDIO_OK)
            Error_Handler();
        //====================
        // Start Processing...
        //====================
        if(isBaseStation)
            processBase();
        else
            processRemote();
    } // while(1)
}

// The Remote stay receiving data until a "Connection Request" arrives from the Base.
// It then start playing an Alarm sound and wait for the user to pick up the phone.
// If the user does not interact within a given time the Remote stops the sound
// ad return waiting a new call.
// If the user interact within the given time it send a "Connectio Accepted" message
// and assumes to be connected with the Base.
void
connectRemote() {
    uint8_t pipe_num;
    bool bRemoteConnected = false;
    bConnectionAccepted   = false;
    bConnectionRequested  = false;
    setRole(PRX);
    rf24.flush_rx();
    rf24.setRetries(5, 15);//We don't need to bo very fast but reliable

    while(!bRemoteConnected) {
        BSP_LED_On(LED_RED);
        if(bRadioDataAvailable) {
            rf24.available(&pipe_num);
            bRadioDataAvailable = false;
            BSP_LED_On(LED_BLUE);
            rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
            BSP_LED_Off(LED_BLUE);
            if(rxBuffer[0] == connectRequest) {
                bConnectionRequested = true;
                buffer_offset = BUFFER_OFFSET_NONE;
                // Link the USB Host disk I/O
                if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0) {
                    USBH_Init(&hUSB_Host, USBH_UserProcess, 0);// Init Host Library
                    USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);// Add Supported Class
                    USBH_Start(&hUSB_Host);// Start Host Process
                }
                while(!bConnectionAccepted) {
                    switch(AppliState) {
                        case APPLICATION_START:
                            MSC_Application();
                            break;
                        case APPLICATION_IDLE:
                        default:
                            break;
                    }
                    USBH_Process(&hUSB_Host); // USBH_Background Process
                }
                rf24.flush_rx();
                bRadioDataAvailable = false;
                startConnectTime = HAL_GetTick();
                uint32_t elapsed = 0;
                do {
                    if(bRadioDataAvailable) {
                        bRadioDataAvailable = false;
                        rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                        if(rxBuffer[0] == connectRequest) { // Base is still asking for connection
                            txBuffer[0] = connectionAccepted;
                            BSP_LED_On(LED_ORANGE);
                            rf24.writeAckPayload(pipe_num, txBuffer, MAX_PAYLOAD_SIZE);
                            BSP_LED_Off(LED_ORANGE);
                            bRemoteConnected = true;
                            delayMicroseconds(6*250*15); // Give nRF24 time to transmit before
                                                         // changing role from PRX to PTX
                        }
                    }
                    elapsed = HAL_GetTick()-startConnectTime;
                } while(!bRemoteConnected || (elapsed < 5000));
            } // if(bRadioDataAvailable)
        }
        BSP_LED_Off(LED_RED);
    }
    BSP_LED_Off(LED_RED);
}


// The Base, once being woken up by a button press,
// send a "Connection Request" to the Remote and waits for
// a "Connection Accepted" message within a given time.
// If the "Connection Accepted" is received the Base assumes that
// a Remote is ready to talk otherwise the Base send a message
// when it gives up and returns sleeping.
void
connectBase() {
    setRole(PTX);
    rf24.flush_tx();
    // At first we don't need to be very fast but reliable
    rf24.setRetries(5, 15);
    bool bBaseConnected = false;
    bBaseSleeping       = true;

    do {
        // We will be woken up only by a button press
        while(bBaseSleeping) {
        }
        BSP_LED_Off(LED_ORANGE);
        // An external event has been detected: try to connect to a Remote !
        startConnectTime = HAL_GetTick();
        uint32_t t0 = startConnectTime-2000;
        uint32_t elapsed = 0;

        do {
            if(HAL_GetTick()-t0 > QUERY_INTERVAL) {
                t0 = HAL_GetTick();
                txBuffer[0] = connectRequest;
                BSP_LED_On(LED_BLUE);
                rf24.enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
                rf24.startWrite();
                BSP_LED_Off(LED_BLUE);
            }
            if(bRadioDataAvailable) {
                bRadioDataAvailable = false;
                BSP_LED_On(LED_ORANGE); // Signal the packet's start reading
                rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_ORANGE); // Reading done
                if(rxBuffer[0] == connectionAccepted) {
                    bBaseConnected = true;
                    delayMicroseconds(6*250*15); // Give nRF24 time to transmit and wait for ACK
                                                 // before changing role from PTX to PRX
                }
            }
            elapsed = HAL_GetTick()-startConnectTime;
        } while(!bBaseConnected && (elapsed < MAX_CONNECTION_TIME));

        ledsOff();
        if(!bBaseConnected) {
            txBuffer[0] = connectionTimedOut;
            BSP_LED_On(LED_BLUE);
            rf24.enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
            rf24.startWrite();
            BSP_LED_Off(LED_BLUE);
        }
    } while(!bBaseConnected);
}


void
processBase() {
    uint8_t pipe_num;
    setRole(PRX); // Change role from PTX to PRX...
    rf24.flush_rx();
    rf24.setRetries(1, 0); // We have to be fast !!!
    bSuspend = false;
    bRadioDataAvailable = false;
    InitADC();
    Init_TIM2_Adc();
    // Enables ADC DMA requests and enables ADC peripheral
    if(HAL_ADC_Start_DMA(&hAdc, (uint32_t*)adcDataIn, 2*MAX_PAYLOAD_SIZE) != HAL_OK) {
        Error_Handler();
    }
    chunk = 0;
    BSP_AUDIO_OUT_Play(Audio_Out_Buffer, 2*MAX_PAYLOAD_SIZE);
    rf24.maskIRQ(false, false, false);
    // Enable ADC periodic sampling
    if(HAL_TIM_Base_Start(&Tim2Handle) != HAL_OK) Error_Handler();
    BSP_LED_On(LED_GREEN);
    while(!bSuspend) {
        if(bRadioDataAvailable) {
            bRadioDataAvailable = false;
            rf24.available(&pipe_num);
            if(pipe_num == 2) {// The packet is a command
                BSP_LED_On(LED_BLUE); // Signal the packet's start reading
                rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE); // Reading done
                if(rxBuffer[0] == suspendCmd) {
                    txBuffer[0] = suspendAck;
                    bSuspend = true;
                    rf24.writeAckPayload(pipe_num, txBuffer, MAX_PAYLOAD_SIZE);
                    HAL_Delay(1000);
                }
            }
            else { // The packet contains Audio Data
                rf24.writeAckPayload(pipe_num, txBuffer, MAX_PAYLOAD_SIZE);
                // Then read the data...
                BSP_LED_On(LED_BLUE); // Signal the packet's start reading
                rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE); // Reading done
                // Write data in the current Audio out chunk...
                offset = chunk*MAX_PAYLOAD_SIZE;
                for(uint8_t indx=0; indx<MAX_PAYLOAD_SIZE; indx++) {
                    offset = (chunk*MAX_PAYLOAD_SIZE+indx) << 1;
                    Audio_Out_Buffer[offset]   = rxBuffer[indx] << 8; // 1st Stereo Channel
                    Audio_Out_Buffer[offset+1] = rxBuffer[indx] << 8; // 2nd Stereo Channel
                }
            }// We have done with the new data.
        } // if(bRadioDataAvailable)
    } // while(!bSuspend)
    HAL_TIM_Base_Stop(&Tim2Handle);
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW); // Stop reproducing audio
    ledsOff();
}


void
processRemote() {
    setRole(PTX); // Change role from PRX to PTX...
    rf24.flush_tx();
    bReady2Send  = false;
    rf24.setRetries(1, 0); // We have to be fast !!!
    BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
    BSP_AUDIO_IN_Record(pdmDataIn, INTERNAL_BUFF_SIZE);
    chunk = 0;
    BSP_AUDIO_OUT_Play(Audio_Out_Buffer, 2*sizeof(*Audio_Out_Buffer)*MAX_PAYLOAD_SIZE);
    rf24.maskIRQ(false, false, false);

    bRadioIrq = true; // To force the first sending when we are ready to send
    bSuspend = false;
    while(!bSuspend) {
        if(bReady2Send && bRadioIrq) { // We will send data only when avaialble and
            bReady2Send = 0;           // the previous data were sent or lost !
            bRadioIrq = false;
            BSP_LED_Off(LED_ORANGE);
            BSP_LED_Off(LED_BLUE);
            BSP_LED_On(LED_GREEN);// Signal the start sending...
            rf24.startWrite();
        }
        if(bRadioDataAvailable) {
            bRadioDataAvailable = false;
            BSP_LED_On(LED_BLUE); // Signal the packet's start reading
            rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
            BSP_LED_Off(LED_BLUE); // Reading done
            // Write data in the current chunk
            offset = chunk*MAX_PAYLOAD_SIZE;
            for(uint8_t indx=0; indx<MAX_PAYLOAD_SIZE; indx++) {
                offset = (chunk*MAX_PAYLOAD_SIZE+indx) << 1;
                Audio_Out_Buffer[offset]   = rxBuffer[indx] << 8; // 1st Stereo Channel
                Audio_Out_Buffer[offset+1] = rxBuffer[indx] << 8; // 2nd Stereo Channel
            }
            // We have done with the new data...

        }
    } // while(!bSuspend)
    ledsOff();
    BSP_AUDIO_IN_Stop();               // Stop sending Audio Data
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW); // Stop reproducing audio
    rf24.openWritingPipe(pipes[2]);
    rf24.flush_rx();
    BSP_LED_On(LED_BLUE);
    txBuffer[0] = suspendCmd;
    rf24.enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
    rf24.startWrite();
    BSP_LED_Off(LED_BLUE);
}


void
ledsOff() {
    BSP_LED_Off(LED_ORANGE);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_BLUE);
    BSP_LED_Off(LED_RED);
}


void
initBuffers(bool isBaseStation) {
    // Allocate common buffers... (The receive buffer and the Audio Out one)
    if(!rxBuffer)
        rxBuffer = (uint8_t*)malloc(MAX_PAYLOAD_SIZE*sizeof(*rxBuffer));
    if(!Audio_Out_Buffer)
        Audio_Out_Buffer = (uint16_t*)malloc(2*2*MAX_PAYLOAD_SIZE*sizeof(*Audio_Out_Buffer));
    memset(rxBuffer,           0, MAX_PAYLOAD_SIZE*sizeof(*rxBuffer));
    memset(Audio_Out_Buffer, 0, 2*2*MAX_PAYLOAD_SIZE*sizeof(*Audio_Out_Buffer));

    if(isBaseStation) {
        if(!adcDataIn)
            adcDataIn = (uint16_t*)malloc(2*MAX_PAYLOAD_SIZE*sizeof(*adcDataIn));
        if(!txBuffer)
            txBuffer  = (uint8_t *)malloc(MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
        memset(adcDataIn,  0, 2*MAX_PAYLOAD_SIZE*sizeof(*adcDataIn));
        memset(txBuffer,   0, 2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
    }
    else {
        if(!txBuffer)
            txBuffer = (uint8_t *)malloc(2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
        if(!pdmDataIn)
            pdmDataIn = (uint16_t*)malloc(INTERNAL_BUFF_SIZE*sizeof(*pdmDataIn));
        if(!pcmDataOut)
            pcmDataOut = (uint16_t*)malloc(PCM_OUT_SIZE*sizeof(*pcmDataOut));
        memset(txBuffer,   0, 2*MAX_PAYLOAD_SIZE*sizeof(*txBuffer));
        memset(pdmDataIn,  0, INTERNAL_BUFF_SIZE*sizeof(*pdmDataIn));
        memset(pcmDataOut, 0, PCM_OUT_SIZE*sizeof(*pcmDataOut));
    }
}


void
BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq, void *Params) {
    // Ensure the clock is the same for MIC input and DAC output
    BSP_AUDIO_IN_ClockConfig(hi2s, AudioFreq, Params);
}


void
setRole(bool bPTX) {
    if(bPTX) {
        rf24.stopListening(); // Change role from PRX to PTX...
        rf24.openWritingPipe(pipes[0]);
        for(uint8_t i=1; i<6; i++) {
            rf24.openReadingPipe(i, pipes[i]);
        }
    }
    else {
        rf24.openWritingPipe(pipes[1]);
        rf24.openReadingPipe(1, pipes[0]);
        for(uint8_t i=2; i<6; i++) {
            rf24.openReadingPipe(i, pipes[i]);
        }
        rf24.startListening();
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
    HAL_NVIC_SetPriority(ADC1_DMA_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(ADC1_DMA_IRQn);
}


// The output register of the ADCs is a 16 bit register
// even when the resolution is <= 8 bit
void
InitADC() {
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
Init_TIM2_Adc(void) {
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
OTG_FS_IRQHandler(void) {
  HAL_HCD_IRQHandler(&hhcd);
}


void
EXTI15_10_IRQHandler(void) { // We received a radio interrupt...
    // Read & reset the IRQ status
    rf24.whatHappened(&tx_ok, &tx_failed, &rx_data_ready);

    if(rx_data_ready) {
        bRadioDataAvailable = true;
    }

    if(!isBaseStation && tx_ok) { // TX_DS IRQ asserted when the ACK packet has been received.
        BSP_LED_Off(LED_RED); // Reset previous errors signal
        BSP_LED_On(LED_BLUE); // Signal a good transmission
    }

    if(!isBaseStation && tx_failed) {// nRF24L01+ asserts the IRQ pin when MAX_RT is reached
        // but the payload in TX FIFO is NOT removed!
        BSP_LED_On(LED_RED);
        BSP_LED_Off(LED_ORANGE);
        BSP_LED_Off(LED_BLUE);
    }
    bRadioIrq = true;
    __HAL_GPIO_EXTI_CLEAR_IT(NRF24_IRQ_PIN);
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
    if(bConnectionAccepted) {
        // Transfer last received chunk
        offset = chunk*MAX_PAYLOAD_SIZE*2;
        BSP_AUDIO_OUT_ChangeBuffer(&Audio_Out_Buffer[offset], 2*MAX_PAYLOAD_SIZE);
        // Prepare for next chunk
        chunk = 1-chunk;
        offset = chunk*MAX_PAYLOAD_SIZE*2;
        memset(&Audio_Out_Buffer[offset], 0, 2*MAX_PAYLOAD_SIZE*sizeof(*Audio_Out_Buffer));
    }
    else {
        buffer_offset = BUFFER_OFFSET_FULL;
        BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE/2);
    }
}


/*
void
BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
}
*/


void
BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    buffer_offset = BUFFER_OFFSET_HALF;
}


/**
* @brief  Manages the DMA FIFO error interrupt.
* @param  None
* @retval None
*/
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
    BSP_AUDIO_IN_PDMToPCM(&pdmDataIn[INTERNAL_BUFF_SIZE/2], pcmDataOut);
    for(uint16_t i=0; i<PCM_OUT_SIZE; i++) {
        txBuffer[i+PCM_OUT_SIZE] = (pcmDataOut[i<<1] >> 8) & 0xFF;
    }
    rf24.enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
    bReady2Send = true;
}


void
BSP_AUDIO_IN_HalfTransfer_CallBack(void) {
    BSP_AUDIO_IN_PDMToPCM((uint16_t*)&pdmDataIn[0], (uint16_t*)&pcmDataOut[0]);
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


/// This function handles External line 0 interrupt request.
void
EXTI0_IRQHandler(void) {
    if(isBaseStation)
        bBaseSleeping = false;
    else {
        if(bConnectionRequested) {
            bConnectionRequested = false;
            bConnectionAccepted = true;
            CmdIndex = CMD_STOP; // Ask to stop the Alarm Sound
        }
        else {
            bSuspend = true;
        }
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
}


/// This function handles External line 1 interrupt request.
void
EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
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
initLeds() {
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);
}


void
InitConfigPin() {
    GPIO_InitTypeDef GPIO_InitStructure;
    // Initialize CONFIGURE_PIN as Input with Pulldown
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStructure.Pin   = CONFIGURE_PIN;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    HAL_GPIO_Init(CONFIGURE_PORT, &GPIO_InitStructure);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void
NonFatalError_Handler(void) {// 2 sec of led blinking
    uint32_t t0 = HAL_GetTick();
    ledsOff();
    while(HAL_GetTick()-t0 < 1000) {
        HAL_Delay(100);
        BSP_LED_Toggle(LED3);
        BSP_LED_Toggle(LED4);
        BSP_LED_Toggle(LED5);
        BSP_LED_Toggle(LED6);
    }
    HAL_Delay(500);
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
