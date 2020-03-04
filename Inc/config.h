#pragma once

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

//        Mode    SDIO          SPI             GPIO
//        ---------------------------------------------
//        Pin 1 - Data 3        ~Card Select    PB8
//        Pin 2 - Cmd I/O       MOSI            PB15
//        Pin 3 - Gnd
//        Pin 4 - Vdd (3.3V)
//        Pin 5 - Clock         SCLK            PB13
//        Pin 6 - Gnd
//        Pin 7 - Data 0        MISO            PC2
//        Pin 8 - Data 1        NC
//        Pin 9 - Data 2        NC


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
//  TIM3                STM32F4     (Relay Pulse Generation)
//  TIM6                STM32F4     (Timeout Handling)
//  SPI1                NRF24L01    (Radio RxTx)
//  SDIO                STM32F4     (SD Card Reader)
//===============================================================


//===============================================================
//        SD Pinout (Share SPI2 with MP45DT02 Microphone)
//===============================================================
// PB8      ~Card Select
// PB15     MOSI
// PC2      MISO
// PB13     SCLK
//===============================================================


//===============================================================
//                 CS43L22 Pinout (use I2S3==SPI3)
//===================== ==========================================
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
// Relays (We use TIM3 to produce a long enough pulse)
//===============================================================
// PB4      Free1 Relay             TIM3 CH1
// PB5      Gate Relay              TIM3 CH2
// PB0      Free2 Relay             TIM3 CH3
// PB1      Car Gate Relay          TIM3 CH4
//===============================================================
#define FREE1_RELAY_TIM_CHANNEL             TIM_CHANNEL_1
#define FREE1_RELAY_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define FREE1_RELAY_GPIO_PORT               GPIOB
#define FREE1_RELAY_GPIO_PIN                GPIO_PIN_4
//---------------------------------------------------------------
#define GATE_RELAY_TIM_CHANNEL              TIM_CHANNEL_2
#define GATE_RELAY_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()
#define GATE_RELAY_GPIO_PORT                GPIOB
#define GATE_RELAY_GPIO_PIN                 GPIO_PIN_5
//------ Can't be used: Conflicting with the Phone Button ------
//#define FREE2_RELAY_TIM_CHANNEL             TIM_CHANNEL_3
//#define FREE2_RELAY_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
//#define FREE2_RELAY_GPIO_PORT               GPIOB
//#define FREE2_RELAY_GPIO_PIN                GPIO_PIN_0
//---------------------------------------------------------------
#define CAR_GATE_RELAY_TIM_CHANNEL          TIM_CHANNEL_4
#define CAR_GATE_RELAY_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define CAR_GATE_RELAY_GPIO_PORT            GPIOB
#define CAR_GATE_RELAY_GPIO_PIN             GPIO_PIN_1


//===============================================================
//                 LEDs by Colours
//===============================================================
#define LED_ORANGE  LED3
#define LED_GREEN   LED4
#define LED_RED     LED5
#define LED_BLUE    LED6

