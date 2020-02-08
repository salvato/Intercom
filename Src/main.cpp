#include "main.h"
#include "RF24.h"
#include "printf.h"
#include "string.h"
#include "stdlib.h"
#include "nRF24L01.h"
#include "ff_gen_drv.h"
#include "usbh_diskio_dma.h"




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
//                 Push Buttons
//===============================================================
// PA0      Phone Push Button
// PA2      Gate Push Button
// PA3      Car Gate Push Button
//===============================================================
#define PHONE_BUTTON_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE();
#define PHONE_BUTTON_GPIO_PORT          GPIOA
#define PHONE_BUTTON_GPIO_PIN           GPIO_PIN_0
#define PHONE_BUTTON_IRQ                EXTI0_IRQn
//---------------------------------------------------------------
#define GATE_BUTTON_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE();
#define GATE_BUTTON_GPIO_PORT          GPIOA
#define GATE_BUTTON_GPIO_PIN           GPIO_PIN_2
#define GATE_BUTTON_IRQ                EXTI2_IRQn
//---------------------------------------------------------------
#define CAR_GATE_BUTTON_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE();
#define CAR_GATE_BUTTON_GPIO_PORT       GPIOA
#define CAR_GATE_BUTTON_GPIO_PIN        GPIO_PIN_3
#define CAR_GATE_BUTTON_IRQ             EXTI3_IRQn


//===============================================================
//                     Relays
//===============================================================
// PA2      Gate Relay
// PA3      Car Gate Relay
//===============================================================
#define GATE_RELAY_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE();
#define GATE_RELAY_GPIO_PORT            GPIOA
#define GATE_RELAY_GPIO_PIN             GPIO_PIN_2
//---------------------------------------------------------------
#define CAR_GATE_RELAY_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE();
#define CAR_GATE_RELAY_GPIO_PORT        GPIOA
#define CAR_GATE_RELAY_GPIO_PIN         GPIO_PIN_3


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


#define MAX_CONNECTION_TIME  60000
#define MAX_WAIT_ACK_TIME    5000
#define QUERY_INTERVAL       300

// State Machine for the USBH_USR_ApplicationState
#define USBH_USR_FS_INIT    ((uint8_t)0x00)
#define USBH_USR_AUDIO      ((uint8_t)0x01)


//===============================================================
// Local Functions
//===============================================================
static void SystemClock_Config(void);
static void InitConfigPin();
static void PushButton_Init(GPIO_TypeDef* Port, uint32_t Pin, IRQn_Type Irq);
static void Relay_Init(GPIO_TypeDef* Port, uint32_t Pin);
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
static void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId);
static bool prepareFileSystem();
static void startAlarm();
static bool updateAlarm();
static void stopAlarm();


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
uint8_t*  Audio_Buffer     = NULL;
uint16_t* Audio_Out_Buffer = NULL;
uint16_t* pdmDataIn        = NULL;
uint16_t* pcmDataOut       = NULL;
uint16_t  offset;


uint32_t chunk = 0;
bool     isBaseStation;
uint8_t  status;
uint8_t  Volume;

char path[] = "0:/";
FIL FileRead;
DIR Directory;

UINT bytesread;
uint32_t WaveDataLength;
__IO uint32_t AudioRemSize;
WAVE_FormatTypeDef waveformat;


__IO bool tx_ok;
__IO bool tx_failed;
__IO bool rx_data_ready;
__IO bool bRadioIrq;
__IO bool bReady2Send;
__IO bool bReady2Play;
__IO bool bBaseSleeping;

__IO bool bConnectionRequested;
__IO bool bConnectionAccepted;
     bool bConnectionTimedOut;
__IO bool bSuspend;      // true if the Remote ask to suspend the connection
__IO bool bRadioDataAvailable;
__IO uint32_t startConnectTime;
__IO BUFFER_StateTypeDef buffer_offset;



// Commands
typedef enum {
    connectRequest = 0x10,
    connectionAccepted,
    connectionTimedOut,
    suspendCmd,
    suspendAck,
    openGateCmd,
    openGateAck,
    openCarGateCmd,
    openCarGateAck
} Commands;


FATFS USBDISKFatFs;          // File system object for USB disk logical drive
char USBDISKPath[4];         // USB Host logical drive path

MSC_ApplicationTypeDef AppliState = APPLICATION_IDLE;
static uint8_t  USBH_USR_ApplicationState = USBH_USR_FS_INIT;


int
main(void) {
    const uint8_t Channel = 76;
    Volume = 70; // % of Max
    // System startup
    HAL_Init();
    initLeds();
    SystemClock_Config();
    // Are we Base or Remote ?
    InitConfigPin();
    isBaseStation = HAL_GPIO_ReadPin(CONFIGURE_PORT, CONFIGURE_PIN);
    // Initialize the corresponding things..
    if(!isBaseStation) { // Prepare Wave file to Play
        if(!prepareFileSystem())
            // Here we should provide an alternative way to produce the Alarm Sound !
            Error_Handler();
    }
    initBuffers(isBaseStation);

    // Init Push Buttons and Relays
    PHONE_BUTTON_CLK_ENABLE();
    PushButton_Init(PHONE_BUTTON_GPIO_PORT, PHONE_BUTTON_GPIO_PIN, PHONE_BUTTON_IRQ);
    if(isBaseStation) {
        GATE_RELAY_CLK_ENABLE();
        Relay_Init(GATE_RELAY_GPIO_PORT, GATE_RELAY_GPIO_PIN);
        CAR_GATE_RELAY_CLK_ENABLE();
        Relay_Init(CAR_GATE_RELAY_GPIO_PORT, CAR_GATE_RELAY_GPIO_PIN);
    }
    else {
        GATE_BUTTON_CLK_ENABLE();
        PushButton_Init(GATE_BUTTON_GPIO_PORT, GATE_BUTTON_GPIO_PIN, GATE_BUTTON_IRQ);
        CAR_GATE_BUTTON_CLK_ENABLE();
        PushButton_Init(CAR_GATE_BUTTON_GPIO_PORT, CAR_GATE_BUTTON_GPIO_PIN, CAR_GATE_BUTTON_IRQ);
    }
    if(!rf24.begin(Channel, RADIO_IN_IRQ_PREPRIO)) Error_Handler();
    rf24.clearInterrupts();// Avoid false interrupts from Radio
    rf24.maskIRQ(false, false, false);
    while(true) {
        if(isBaseStation) {
            connectBase();// We will be woken up by a button press
            processBase();
        }
        else {
            connectRemote();// We will be woken up by receiving a command
            processRemote();
        }
    }
}


void
USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId) {
    UNUSED(pHost);

    switch(vId) {
        case HOST_USER_SELECT_CONFIGURATION:
            break;
        case HOST_USER_DISCONNECTION:
            AppliState = APPLICATION_IDLE;
            f_mount(NULL, (TCHAR const*)"", 0);
            break;
        case HOST_USER_CLASS_ACTIVE:
            AppliState = APPLICATION_START;
            break;
        case HOST_USER_CONNECTION:
            break;
        case HOST_USER_CLASS_SELECTED:
            break;
        case HOST_USER_UNRECOVERED_ERROR:
            break;
        default: // No other cases at present !
            break;
    } //switch(vId)
}


bool
prepareFileSystem() {
    // Link the USB Host disk I/O
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) != 0) return false;
    if(USBH_Init(&hUSB_Host, USBH_UserProcess, 0) != USBH_OK) return false;
    if(USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS) != USBH_OK) return false;
    if(USBH_Start(&hUSB_Host) != USBH_OK) return false;

    USBH_USR_ApplicationState = USBH_USR_FS_INIT;
    while(AppliState != APPLICATION_START) {
        USBH_Process(&hUSB_Host); // USBH_Background Process
    }

    // Initializes (mount) the File System
    if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0 ) != FR_OK ) return false;
    if(f_opendir(&Directory, path) != FR_OK) return false;
    USBH_USR_ApplicationState = USBH_USR_AUDIO;
    return true;
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
    bool bBaseConnected;

    do {
        BSP_LED_On(LED_RED);
        bBaseConnected = false;
        bBaseSleeping = true;
        // We will be woken up only by a button press
        while(bBaseSleeping) {
        }

        // An external event has been detected: try to connect to a Remote !
        startConnectTime = HAL_GetTick();
        uint32_t t0      = startConnectTime-2000; // Just to start the first request.
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
                    HAL_Delay(50);
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
    BSP_LED_Off(LED_RED);
}


// The Remote stay receiving data until a "Connection Request" arrives from the Base.
// It then start playing an Alarm Sound and wait for the user to pick up the phone.
// If the user does not interact within a given time the Remote stops the sound
// ad return waiting a new call.
// If the user interact within the given time it send a "Connection Accepted" message
// and assumes to be connected with the Base.
void
connectRemote() {
    bool bRemoteConnected = false;
    bConnectionAccepted   = false;
    bConnectionRequested  = false;
    setRole(PRX);
    rf24.flush_rx();
    rf24.setRetries(5, 15);//We don't need to bo very fast but reliable

    while(!bRemoteConnected) {
        BSP_LED_On(LED_RED);
        while(!bRadioDataAvailable) {
        }
        bRadioDataAvailable  = false;
        bConnectionRequested = false;
        bConnectionTimedOut  = false;
        BSP_LED_On(LED_BLUE);
        rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
        BSP_LED_Off(LED_BLUE);
        if(rxBuffer[0] == connectRequest) {
            bConnectionRequested = true;
            startAlarm();
            while(!bConnectionAccepted && !bConnectionTimedOut) {
                if(!updateAlarm()) {
                    bConnectionTimedOut = true;
                }
                if(bRadioDataAvailable) {
                    bRadioDataAvailable = false;
                    rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                    if(rxBuffer[0] == connectionTimedOut) {
                        bConnectionTimedOut = true;
                    }
                }
                USBH_Process(&hUSB_Host); // USBH_Background Process
            }
            stopAlarm();

            if(!bConnectionTimedOut && bConnectionAccepted) {
                startConnectTime = HAL_GetTick();
                uint32_t elapsed = 0;
                do {
                    if(bRadioDataAvailable) {
                        bRadioDataAvailable = false;
                        rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                        if(rxBuffer[0] == connectRequest) { // Base is still asking for connection
                            txBuffer[0] = connectionAccepted;
                            BSP_LED_On(LED_ORANGE);
                            rf24.writeAckPayload(1, txBuffer, MAX_PAYLOAD_SIZE);
                            BSP_LED_Off(LED_ORANGE);
                            bRemoteConnected = true;
                            HAL_Delay(500); // Needed for unknown reasons...
                        }
                    }
                    elapsed = HAL_GetTick()-startConnectTime;
                } while(!bRemoteConnected && (elapsed < MAX_WAIT_ACK_TIME));
            }

        } // if(rxBuffer[0] == connectRequest)

    } // while(!bRemoteConnected)
    BSP_LED_Off(LED_RED);
}


void
processBase() {
    uint8_t pipe_num;
    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, DEFAULT_AUDIO_IN_FREQ);
    setRole(PRX); // Change role from PTX to PRX...
    rf24.flush_rx();
    rf24.setRetries(1, 0); // We have to be fast !!!
    bRadioDataAvailable = false;
    InitADC();
    Init_TIM2_Adc();
    // Enables ADC DMA requests and enables ADC peripheral
    if(HAL_ADC_Start_DMA(&hAdc, (uint32_t*)adcDataIn, 2*MAX_PAYLOAD_SIZE) != HAL_OK) {
        Error_Handler();
    }
    chunk = 0;
    rf24.maskIRQ(false, false, false);
    BSP_AUDIO_OUT_Play(Audio_Out_Buffer, 2*MAX_PAYLOAD_SIZE);
    // Enable ADC periodic sampling
    if(HAL_TIM_Base_Start(&Tim2Handle) != HAL_OK) Error_Handler();
    BSP_LED_On(LED_GREEN);
    bSuspend = false;
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
                    HAL_Delay(300);
                }
            }
            else { // The packet contains Audio Data: first send our audio data...
                rf24.writeAckPayload(pipe_num, txBuffer, MAX_PAYLOAD_SIZE);
                // then read the data...
                BSP_LED_On(LED_BLUE); // Signal the packet's start reading
                rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
                BSP_LED_Off(LED_BLUE); // Reading done
                // Save data in the current Audio out chunk...
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
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW); // Stop reproducing audio and power down the Codec
    ledsOff();
}


void
processRemote() {
    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, DEFAULT_AUDIO_IN_FREQ);
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
        if(bReady2Send && bRadioIrq) { // We will send data only when available and
            bReady2Send = false;       // the previous data were sent or lost !
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
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW); // Stop reproducing audio and switch off the codec
    rf24.openWritingPipe(pipes[2]);
    rf24.setRetries(5, 15);
    rf24.flush_rx();

    startConnectTime = HAL_GetTick();
    uint32_t t0      = startConnectTime-2000; // Just to start the first request.
    uint32_t elapsed = 0;
    bool bBaseDisConnected = false;
    do {
        if(HAL_GetTick()-t0 > QUERY_INTERVAL) {
            t0 = HAL_GetTick();
            BSP_LED_On(LED_BLUE);
            txBuffer[0] = suspendCmd;
            rf24.enqueue_payload(txBuffer, MAX_PAYLOAD_SIZE);
            rf24.startWrite();
            BSP_LED_Off(LED_BLUE);
        }
        if(bRadioDataAvailable) {
            bRadioDataAvailable = false;
            BSP_LED_On(LED_ORANGE); // Signal the packet's start reading
            rf24.read(rxBuffer, MAX_PAYLOAD_SIZE);
            BSP_LED_Off(LED_ORANGE); // Reading done
            if(rxBuffer[0] == suspendAck) {
                bBaseDisConnected = true;
                HAL_Delay(50);
            }
        }
        elapsed = HAL_GetTick()-startConnectTime;
    } while(!bBaseDisConnected && (elapsed < MAX_WAIT_ACK_TIME));
    ledsOff();
}


void
startAlarm() {
    buffer_offset = BUFFER_OFFSET_NONE;
    bytesread = 0;
    WaveDataLength = 0;
    AudioRemSize = 0;
    if(f_open(&FileRead, WAVE_NAME , FA_READ) != FR_OK) {
        // Here we should provide an alternative way to produce the Alarm Sound
        Error_Handler();
    }
    // Read the wav file header
    f_read (&FileRead, &waveformat, sizeof(waveformat), &bytesread);
    WaveDataLength = waveformat.FileSize;
    if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, waveformat.SampleRate) != AUDIO_OK) {
        Error_Handler();
    }
    Audio_Buffer = (uint8_t*)malloc(AUDIO_BUFFER_SIZE*sizeof(*Audio_Buffer));
    f_lseek(&FileRead, 0);
    f_read (&FileRead, &Audio_Buffer[0], AUDIO_BUFFER_SIZE, &bytesread);
    AudioRemSize = WaveDataLength - bytesread;
    BSP_AUDIO_OUT_Play((uint16_t*)&Audio_Buffer[0], AUDIO_BUFFER_SIZE);
}


bool
updateAlarm() {
    bytesread = 0;
    if(buffer_offset == BUFFER_OFFSET_HALF) {
        f_read(&FileRead,
               &Audio_Buffer[0],
               AUDIO_BUFFER_SIZE/2,
               &bytesread);
        buffer_offset = BUFFER_OFFSET_NONE;
    }
    if(buffer_offset == BUFFER_OFFSET_FULL) {
        f_read(&FileRead,
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
    f_close(&FileRead);
}


void
ledsOff() {
    BSP_LED_Off(LED_ORANGE);
    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_BLUE);
    BSP_LED_Off(LED_RED);
}


void
PushButton_Init(GPIO_TypeDef* Port, uint32_t Pin, IRQn_Type Irq) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin   = Pin;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
    HAL_GPIO_Init(Port, &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(Irq, 0x0F, 0);
    HAL_NVIC_EnableIRQ(Irq);
}



void
Relay_Init(GPIO_TypeDef* Port, uint32_t Pin) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin   = Pin;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(Port, &GPIO_InitStruct);
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
    // Ensure the clock is the same both for MIC input and DAC output
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
    if(isBaseStation) {
        startConnectTime = HAL_GetTick();
        bBaseSleeping = false;
    }
    else {
        if(bConnectionRequested) {
            bConnectionRequested = false;
            bConnectionAccepted = true;
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
