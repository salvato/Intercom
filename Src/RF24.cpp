#include "main.h"
#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"



RF24::RF24(GPIO_TypeDef *_ceport, uint32_t _cepin,
           GPIO_TypeDef *_csport, uint32_t _cspin,
           GPIO_TypeDef *_irqport, uint32_t _irqpin, IRQn_Type _irqntype)
    : bBusy(false)
    , ce_port(_ceport)
    , csn_port(_csport)
    , irq_port(_irqport)
    , ce_pin(_cepin)
    , csn_pin(_cspin)
    , irq_pin(_irqpin)
    , irqn_type(_irqntype)
    , payload_size(MAX_PAYLOAD_SIZE)
    , dynamic_payloads_enabled(false)
    , addr_width(5)
    , csDelay(130)
    //,pipe0_reading_address(0)
{
    pipe0_reading_address[0] = 0;
}


void
RF24::enableGPIOClock(GPIO_TypeDef* port) {
    if(port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    else if(port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    else if(port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    else if(port == GPIOD)
        __HAL_RCC_GPIOD_CLK_ENABLE();
    else if(port == GPIOE)
        __HAL_RCC_GPIOE_CLK_ENABLE();
    else if(port == GPIOF)
        __HAL_RCC_GPIOF_CLK_ENABLE();
    else if(port == GPIOG)
        __HAL_RCC_GPIOG_CLK_ENABLE();
    else if(port == GPIOH)
        __HAL_RCC_GPIOH_CLK_ENABLE();
    else if(port == GPIOI)
        __HAL_RCC_GPIOI_CLK_ENABLE();
}


void
RF24::InitPins(uint32_t preempPriority) {
    GPIO_InitTypeDef GPIO_InitStructure;
    memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));

    enableGPIOClock(ce_port);
    GPIO_InitStructure.Pin   = ce_pin;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(ce_port, &GPIO_InitStructure);

    enableGPIOClock(csn_port);
    GPIO_InitStructure.Pin   = csn_pin;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(csn_port, &GPIO_InitStructure);

    if(irq_port) {
        enableGPIOClock(irq_port);
        GPIO_InitStructure.Pin   = irq_pin;
        GPIO_InitStructure.Pull  = GPIO_PULLUP;
        GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
        GPIO_InitStructure.Mode  = GPIO_MODE_IT_FALLING;
        HAL_GPIO_Init(irq_port, &GPIO_InitStructure);

        HAL_NVIC_SetPriority(irqn_type, preempPriority, 0);
    }
}


/****************************************************************************/

bool
RF24::begin(uint8_t channelNumber, uint32_t preempPriority) {
    uint8_t setup=0;
    // Initialize pins
    InitPins(preempPriority);
    // Initialize SPI interface
    spi.begin(SPI_MODE_MASTER);

    powerDown();
    ce(GPIO_PIN_RESET);
    csn(GPIO_PIN_SET);
    HAL_Delay(100); // To allow for nRF24L01+ powerup

    toggle_features();// Needed: See p46 of datasheet rev 2.0
    write_register(EN_AA, 0);
    write_register(DYNPD, 0);

    // Configure nRF24L01+ module
    setup  = _BV(CRCO)        | // 2 Bytes CRC
             _BV(EN_CRC)      | // CRC Enabled
             _BV(MASK_MAX_RT) | // No MAX_RT (Max Retry Reached) IRQ
             _BV(MASK_TX_DS)  | // No TX_DS (Data Sent)          IRQ
             _BV(MASK_RX_DR);   // No RX_DR (Data Ready)         IRQ
    setup &= ~_BV(PRIM_RX);     // PTX
    setup &= ~_BV(PWR_UP);      // POWER DOWN
    write_register(NRF_CONFIG, setup);
    if(read_register(NRF_CONFIG) != setup)
        return false;

    // In order to enable DPL the EN_DPL bit in the FEATURE register must be set.
    // In RX mode the DYNPD register has to be set.
    // A PTX that transmits to a PRX with DPL enabled must have the
    // DPL_P0 bit in DYNPD set
    // In order to enable Auto Acknowledgement with payload the EN_ACK_PAY bit
    // in the FEATURE register must be set.
    setup = _BV(EN_DPL)     | // Enables Dynamic Payload Length
            _BV(EN_ACK_PAY) | // Enables Payload with ACK
            _BV(EN_DYN_ACK);  // Enables the W_TX_PAYLOAD_NOACK command
    write_register(FEATURE, setup);
    if(read_register(FEATURE) != setup)
        return false;

    // Enable Enhanced ShockBurst Auto Acknowledgment on all pipes.
    setup = _BV(ENAA_P0) |
            _BV(ENAA_P1) |
            _BV(ENAA_P2) |
            _BV(ENAA_P3) |
            _BV(ENAA_P4) |
            _BV(ENAA_P5);
    write_register(EN_AA,  setup);
    if(read_register(EN_AA) != setup)
        Error_Handler();

    // Enable Dynamic Payload Length on all pipes
    setup = _BV(DPL_P0) |
            _BV(DPL_P1) |
            _BV(DPL_P2) |
            _BV(DPL_P3) |
            _BV(DPL_P4) |
            _BV(DPL_P5);
    write_register(DYNPD, setup);
    if(read_register(DYNPD) != setup)
        return false;
    dynamic_payloads_enabled = true;

    setRetries(1, 0); // 500 us and Single Retry

    // Reset value is MAX
    setPALevel(RF24_PA_MAX);

    // Get the RF setup to be sure the module is attached and responding
    setup = read_register(RF_SETUP);
    // if setup is 0 or 0xff then there was no response from module
    if(setup == 0 || setup == 0xff)
        return false;

    // Set the on Air Data Rate
    setDataRate(RF24_2MBPS);

    setChannel(channelNumber);

    // Flush FIFO buffers
    flush_rx();
    flush_tx();

    // Power up by default when begin() is called
    powerUp();

    // Clear Interrupt Request...
    clearInterrupts();
    // Enable hardware interrupts
    enableIRQ();

    return true;
}


void
RF24::clearInterrupts() {
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
}


void
RF24::enableIRQ() {
    __HAL_GPIO_EXTI_CLEAR_IT(irqn_type);
    HAL_NVIC_EnableIRQ(irqn_type);
}


void
RF24::disableIRQ() {
    HAL_NVIC_DisableIRQ(irqn_type);
}


void
RF24::csn(GPIO_PinState mode) {
    HAL_GPIO_WritePin(csn_port, csn_pin, mode);
	delayMicroseconds(csDelay);
}

/****************************************************************************/

void
RF24::ce(GPIO_PinState level) {
    HAL_GPIO_WritePin(ce_port, ce_pin,level);
}

/****************************************************************************/

void
RF24::beginTransaction() {
    csn(GPIO_PIN_RESET);
}

/****************************************************************************/

void
RF24::endTransaction() {
    csn(GPIO_PIN_SET);
}

/****************************************************************************/

uint8_t
RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t status;
    beginTransaction();
    status = spi.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
    while ( len-- ){
        *buf++ = spi.transfer(0xff);
    }
    endTransaction();
    return status;
}

/****************************************************************************/

uint8_t
RF24::read_register(uint8_t reg) {
    uint8_t result = 0;
    beginTransaction();
    spi.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
    result = spi.transfer(0xff);
    endTransaction();
    return result;
}

/****************************************************************************/

uint8_t
RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len) {
    uint8_t status;
    beginTransaction();
    status = spi.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
    while ( len-- )
        spi.transfer(*buf++);
    endTransaction();
    return status;
}

/****************************************************************************/

uint8_t
RF24::write_register(uint8_t reg, uint8_t value) {
    uint8_t status = 0;
    IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));
    beginTransaction();
    status = spi.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
    spi.transfer(value);
    endTransaction();
    return status;
}

/****************************************************************************/

uint8_t
RF24::enqueue_payload(const void* buf, uint8_t data_len) {
    uint8_t status = read_register(FIFO_STATUS);
    if(!(status & _BV(TX_EMPTY))) {
        return HAL_OK;
    }
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);
    data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    IF_SERIAL_DEBUG( printf("[Writing %u bytes %u blanks]\n",data_len,blank_len); );

    // Each command must be started wit a HIGH to LOW transition of CSN
    beginTransaction(); // csn(GPIO_PIN_RESET)
    status = spi.transfer(W_TX_PAYLOAD);
    spi.transfer((void *)current, data_len);
    while(blank_len--) {
        spi.transfer(0);
    }
    endTransaction(); // csn(GPIO_PIN_SET)
    return status;
}
/****************************************************************************/

uint8_t
RF24::write_payload(const void* buf, uint8_t data_len, const uint8_t writeType) {
    uint8_t status;
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);
    data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    IF_SERIAL_DEBUG( printf("[Writing %u bytes %u blanks]\n",data_len,blank_len); );

    beginTransaction(); // csn(GPIO_PIN_RESET)
    status = spi.transfer(writeType);
    while(data_len--) {
        spi.transfer(*current++);
    }
    while(blank_len--) {
        spi.transfer(0);
    }
    endTransaction(); // csn(GPIO_PIN_SET)
    return status;
}

/****************************************************************************/

uint8_t
RF24::read_payload(void* buf, uint8_t data_len) {
    uint8_t status;
    uint8_t* current = reinterpret_cast<uint8_t*>(buf);
    if(data_len > payload_size) data_len = payload_size;
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    IF_SERIAL_DEBUG( printf("[Reading %u bytes %u blanks]\n",data_len,blank_len); );
    beginTransaction();
    status = spi.transfer(R_RX_PAYLOAD);
    spi.transfer(current, data_len);
    if(blank_len)
        spi.transfer(current+data_len, blank_len);
    endTransaction();
    return status;
}

/****************************************************************************/

uint8_t
RF24::flush_rx(void) {
    return spiTrans(FLUSH_RX);
}

/****************************************************************************/

uint8_t
RF24::flush_tx(void) {
    return spiTrans(FLUSH_TX);
}

/****************************************************************************/

uint8_t
RF24::spiTrans(uint8_t cmd){
    uint8_t status;
    beginTransaction();
    status = spi.transfer(cmd);
    endTransaction();
    return status;
}

/****************************************************************************/

uint8_t
RF24::get_status(void) {
    return spiTrans(RF24_NOP);
}

/****************************************************************************/

char buffer[255];

/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
//struct __FILE {
//	int dummy;
//};
/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
//FILE __stdout;

//int fputc(int ch, FILE *f) {
//    /* Do your stuff here */
//    /* Send your custom byte */

//    //es: TM_USART_Putc(USART1, ch);

//    /* If everything is OK, you have to return character written */
//    return ch;
//    /* If character is not correct, you can return EOF (-1) to stop writing */
//    //return -1;
//}

//int
//__io_putchar(int ch) {
//    uint8_t c[1];
//    c[0] = ch & 0x00FF;
//    HAL_UART_Transmit(&huart2, &*c, 1, 10);
//    return ch;
//}

//int
//_write(int file,char *ptr, int len) {
//    int DataIdx;
//    for(DataIdx= 0; DataIdx< len; DataIdx++) {
//        __io_putchar(*ptr++);
//    }
//    return len;
//}
/*


    The way I got printf (and all other console-oriented stdio functions) to work was by creating custom implementations of the low-level I/O functions like _read() and _write().

    The GCC C library makes calls to the following functions to perform low-level I/O :

    int _read(int file, char *data, int len)
    int _write(int file, char *data, int len)
    int _close(int file)
    int _lseek(int file, int ptr, int dir)
    int _fstat(int file, struct stat *st)
    int _isatty(int file)

    These functions are implemented witihin the GCC C library as stub routines with "weak" linkage. If a declaration of any of the above functions appears in your own code, your substitute routine will override the declaration in the library and be used instead of the default (non-functional) routine.
 *
    Since I only wanted printf(), I only needed to populate the _write() function, which I did as follows.
    This is conventionally contained in syscalls.c.

#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

int
_write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

    // arbitrary timeout 1000
    HAL_StatusTypeDef status =
            HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

    // return # of bytes written - as best we can tell
    return (status == HAL_OK ? len : 0);
}



#include <stdio.h>
#include <stdarg.h>
int
printf_P(const char *format, ...) {
   va_list aptr;
   int ret;

   va_start(aptr, format);
   ret = vsprintf(buffer, format, aptr);
   va_end(aptr);

   return(ret);
}
 */

void
RF24::print_status(uint8_t status) {
    sprintf(buffer,
            PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
            status,
            (status & _BV(RX_DR))?1:0,
            (status & _BV(TX_DS))?1:0,
            (status & _BV(MAX_RT))?1:0,
            ((status >> RX_P_NO) & 0x07),
            (status & _BV(TX_FULL))?1:0
           );
}

/****************************************************************************/

void
RF24::print_observe_tx(uint8_t value) {
    sprintf(buffer,
            PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
            value,
            (value >> PLOS_CNT) & 0x0F,
            (value >> ARC_CNT) & 0x0F
            );
}

/****************************************************************************/

void
RF24::print_byte_register(const char* name, uint8_t reg, uint8_t qty) {
    int istart =0;
    istart += sprintf(buffer, PSTR(PRIPSTR"\t ="), name);
    while(qty--)
        istart += sprintf(buffer+istart, PSTR(" 0x%02x"), read_register(reg++));
    sprintf(buffer+istart, PSTR("\r\n"));
}

/****************************************************************************/

void
RF24::print_address_register(const char* name, uint8_t reg, uint8_t qty) {
    int istart = 0;
    istart += sprintf(buffer, PSTR(PRIPSTR"\t ="), name);
    while (qty--) {
        uint8_t buf[addr_width];
        read_register(reg++, buf, sizeof buf);
        istart += sprintf(buffer+istart, PSTR(" 0x"));
        uint8_t* bufptr = buf + sizeof buf;
        while(--bufptr >= buf)
            istart += sprintf(buffer+istart, PSTR("%02x"),*bufptr);
    }
    sprintf(buffer+istart, PSTR("\r\n"));
}

/****************************************************************************/

void
RF24::setChannel(uint8_t channel) {
    const uint8_t max_channel = 125;
    write_register(RF_CH,rf24_min(channel,max_channel));
}


uint8_t
RF24::getChannel() {
    return read_register(RF_CH);
}

/****************************************************************************/

void
RF24::setPayloadSize(uint8_t size) {
    payload_size = rf24_min(size, 32);
}

/****************************************************************************/

uint8_t
RF24::getPayloadSize(void) {
    return payload_size;
}

/****************************************************************************/

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
    rf24_datarate_e_str_0,
    rf24_datarate_e_str_1,
    rf24_datarate_e_str_2,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
    rf24_crclength_e_str_0,
    rf24_crclength_e_str_1,
    rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = {
    rf24_pa_dbm_e_str_0,
    rf24_pa_dbm_e_str_1,
    rf24_pa_dbm_e_str_2,
    rf24_pa_dbm_e_str_3,
};


void
RF24::printDetails(void) {
    uint8_t status = get_status();

    print_status(status);

    print_address_register(PSTR("RX_ADDR_P0-1"), RX_ADDR_P0, 2);
    print_byte_register(PSTR("RX_ADDR_P2-5"), RX_ADDR_P2, 4);
    print_address_register(PSTR("TX_ADDR\t"), TX_ADDR);

    print_byte_register(PSTR("RX_PW_P0-6"), RX_PW_P0, 6);
    print_byte_register(PSTR("EN_AA\t"), EN_AA);
    print_byte_register(PSTR("EN_RXADDR"), EN_RXADDR);
    print_byte_register(PSTR("RF_CH\t"), RF_CH);
    print_byte_register(PSTR("RF_SETUP"), RF_SETUP);
    print_byte_register(PSTR("CONFIG\t"), NRF_CONFIG);
    print_byte_register(PSTR("DYNPD/FEATURE"), DYNPD, 2);

    sprintf(buffer, PSTR("Data Rate\t = " PRIPSTR "\r\n"), pgm_read_ptr(&rf24_datarate_e_str_P[getDataRate()]));
    sprintf(buffer, PSTR("CRC Length\t = " PRIPSTR "\r\n"), pgm_read_ptr(&rf24_crclength_e_str_P[getCRCLength()]));
    sprintf(buffer, PSTR("PA Power\t = " PRIPSTR "\r\n"),  pgm_read_ptr(&rf24_pa_dbm_e_str_P[getPALevel()]));
}

/****************************************************************************/

bool
RF24::isChipConnected() {
  uint8_t setup = read_register(SETUP_AW);
  if(setup >= 1 && setup <= 3) {
    return true;
  }
  return false;
}

/****************************************************************************/

void
RF24::startListening(void) {
    powerUp();
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) | _BV(PRIM_RX));
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
    ce(GPIO_PIN_SET);
    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address[0] > 0) {
        write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
    }
    else {
        closeReadingPipe(0);
    }
    // Flush buffers
    //flush_rx();
    if(read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        flush_tx();
    }
    // Go!
    //delayMicroseconds(100);
}

/****************************************************************************/
static const
uint8_t child_pipe_enable[] PROGMEM = {
    ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};


void
RF24::stopListening(void) {
    ce(GPIO_PIN_RESET);
    delayMicroseconds(txDelay);//140
    if(read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        delayMicroseconds(txDelay); //140
        flush_tx();
    }
    write_register(NRF_CONFIG, (read_register(NRF_CONFIG)) & ~_BV(PRIM_RX) );
    write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0
}

/****************************************************************************/

void
RF24::powerDown(void) {
    ce(GPIO_PIN_RESET); // Guarantee CE is low on powerDown
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void
RF24::powerUp(void) {
    uint8_t cfg = read_register(NRF_CONFIG);
    // if not powered up then power up and wait for the radio to initialize
    if (!(cfg & _BV(PWR_UP))){
        write_register(NRF_CONFIG, cfg | _BV(PWR_UP));
        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CE is set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        HAL_Delay(5);
    }
}

/******************************************************************/

void
RF24::errNotify() {
	failureDetected = 1;
}

/******************************************************************/

//Similar to the previous write, clears the interrupt flags
bool
RF24::write( const void* buf, uint8_t len, const bool multicast ) {
    //Start Writing
    startFastWrite(buf, len, multicast);
    //Wait until complete or failed
    uint32_t timer = HAL_GetTick();
    while(!(get_status()  & (_BV(TX_DS) | _BV(MAX_RT)))) {
        if(HAL_GetTick() - timer > 95){
            errNotify();
            return 0;
        }
    }
    ce(GPIO_PIN_RESET);
    uint8_t status = write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
    //Max retries exceeded
    if(status & _BV(MAX_RT)){
        flush_tx(); //Only going to be 1 packet into the FIFO at a time using this method, so just flush
        return 0;
    }
    //TX OK 1 or 0
    return 1;
}

bool
RF24::write(const void* buf, uint8_t len){
    return write(buf, len, 0);
}

/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool
RF24::writeBlocking( const void* buf, uint8_t len, uint32_t timeout ) {
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //This way the FIFO will fill up and allow blocking until packets go through
    //The radio will auto-clear everything in the FIFO as long as CE remains high
    uint32_t timer = HAL_GetTick();							  //Get the time that the payload transmission started
    while( ( get_status()  & ( _BV(TX_FULL) ))) {		  //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout
        if( get_status() & _BV(MAX_RT)){					  //If MAX Retries have been reached
            reUseTX();										  //Set re-transmit and clear the MAX_RT interrupt flag
            if(HAL_GetTick() - timer > timeout){ return 0; }		  //If this payload has exceeded the user-defined timeout, exit and return 0
        }
        if(HAL_GetTick() - timer > (timeout+95) ){
            errNotify();
            return 0;
        }
    }
    //Start Writing
    startFastWrite(buf, len, 0);								  //Write the payload if a buffer is clear
    return 1;												  //Return 1 to indicate successful transmission
}

/****************************************************************************/

void
RF24::reUseTX(){
    write_register(NRF_STATUS,_BV(MAX_RT));			  //Clear max retry flag
    spiTrans(REUSE_TX_PL);
    ce(GPIO_PIN_RESET);										  //Re-Transfer packet
    ce(GPIO_PIN_SET);
}

/****************************************************************************/

bool
RF24::writeFast( const void* buf, uint8_t len, const bool multicast ) {
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //Return 0 so the user can control the retrys and set a timer or failure counter if required
    //The radio will auto-clear everything in the FIFO as long as CE remains high
    uint32_t timer = HAL_GetTick();

    while( ( get_status()  & ( _BV(TX_FULL) ))) {			  //Blocking only if FIFO is full. This will loop and block until TX is successful or fail
        if(get_status() & _BV(MAX_RT)){
            //reUseTX();										  //Set re-transmit
            write_register(NRF_STATUS, _BV(MAX_RT));			  //Clear max retry flag
            return 0;										  //Return 0. The previous payload has been retransmitted
        }
        if(HAL_GetTick() - timer > 95 ){
            errNotify();
            return 0;
        }
    }
    //Start Writing
    startFastWrite(buf,len,multicast);
    return 1;
}


bool
RF24::writeFast(const void* buf, uint8_t len) {
    return writeFast(buf, len, 0);
}

/****************************************************************************/

// Per the documentation, we want to set PTX Mode when not listening.
// Then all we do is write data and set CE high
// In this mode, if we can keep the FIFO buffers loaded,
// packets will transmit immediately (no 130us delay)
// Otherwise we enter Standby-II mode, which is still faster than standby mode
// Also, we remove the need to keep writing the config register over and over
// and delaying for 150 us each time if sending a stream of data
void
RF24::startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx) { //TMRh20
    write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    if(startTx) {
        ce(GPIO_PIN_SET);
	}
}

/****************************************************************************/

void
RF24::startWrite() {
    // Send the payload from FIFO
    while(bBusy){}
    bBusy = true;
    ce(GPIO_PIN_SET);
    delayMicroseconds(10);
    ce(GPIO_PIN_RESET);
    bBusy = false;
}

/****************************************************************************/

bool
RF24::rxFifoFull(){
	return read_register(FIFO_STATUS) & _BV(RX_FULL);
}

/****************************************************************************/

bool
RF24::txStandBy() {
    while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ) {
        if(get_status() & _BV(MAX_RT)){
            write_register(NRF_STATUS, _BV(MAX_RT));
            ce(GPIO_PIN_RESET);
            flush_tx(); // Non blocking, flush the data
            return 0;
        }
    }
    ce(GPIO_PIN_RESET); // Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

bool
RF24::txStandBy(uint32_t timeout, bool startTx) {
    if(startTx){
        stopListening();
        ce(GPIO_PIN_SET);
    }
    uint32_t start = HAL_GetTick();
    while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
        if( get_status() & _BV(MAX_RT)){
            write_register(NRF_STATUS,_BV(MAX_RT) );
            ce(GPIO_PIN_RESET);							  //Set re-transmit
            ce(GPIO_PIN_SET);
            if(HAL_GetTick() - start >= timeout){
                ce(GPIO_PIN_RESET); flush_tx(); return 0;
            }
        }
        if( HAL_GetTick() - start > (timeout+95)){
            errNotify();
            return 0;
        }
    }
    ce(GPIO_PIN_RESET);				   //Set STANDBY-I mode
    return 1;
}

/****************************************************************************/

void
RF24::maskIRQ(bool tx, bool fail, bool rx) {
	uint8_t config = read_register(NRF_CONFIG);
	/* clear the interrupt flags */
	config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	write_register(NRF_CONFIG, config);
}

/****************************************************************************/

uint8_t
RF24::getDynamicPayloadSize(void) {
    uint8_t result = 0;
    beginTransaction();
    spi.transfer( R_RX_PL_WID );
    result = spi.transfer(0xff);
    endTransaction();
    if(result > 32) {
        flush_rx();
        HAL_Delay(2);
        return 0;
    }
    return result;
}

/****************************************************************************/

bool
RF24::available(void) {
    return available(NULL);
}

/****************************************************************************/

bool
RF24::available(uint8_t* pipe_num) {
    if (!( read_register(FIFO_STATUS) & _BV(RX_EMPTY) )) {
        // If the caller wants the pipe number, include that
        if(pipe_num) {
            uint8_t status = get_status();
            *pipe_num = ( status >> RX_P_NO ) & 0x07;
        }
        return 1;
    }
    return 0;
}

/****************************************************************************/

void
RF24::read( void* buf, uint8_t len) {
    // Fetch the payload
    read_payload(buf, len);
    //Clear the two possible interrupt flags with one command
    //write_register(NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );
}

/****************************************************************************/

void
RF24::whatHappened(__IO bool* tx_ok, __IO bool* tx_fail, __IO bool* rx_ready) {
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    uint8_t status = write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
    // Report to the user what happened
    *tx_ok    = status & _BV(TX_DS);
    *tx_fail  = status & _BV(MAX_RT);
    *rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void
RF24::openWritingPipe(const uint8_t *address) {
    // Note that NRF24L01(+) expects address LSB first
    write_register(RX_ADDR_P0, address, addr_width);
    write_register(TX_ADDR, address, addr_width);
    write_register(RX_PW_P0, payload_size);
}

/****************************************************************************/

static const uint8_t
child_pipe[] PROGMEM = {
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};

static const uint8_t
child_payload_size[] PROGMEM = {
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

/****************************************************************************/

void
RF24::setAddressWidth(uint8_t a_width) {
    if(a_width -= 2){
        write_register(SETUP_AW,a_width%4);
        addr_width = (a_width%4) + 2;
    }
    else {
        write_register(SETUP_AW,0);
        addr_width = 2;
    }
}

/****************************************************************************/

void
RF24::openReadingPipe(uint8_t child, const uint8_t *address) {
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, address,addr_width);
    }
    if (child <= 6) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            write_register(pgm_read_byte(&child_pipe[child]), address, addr_width);
        }
        else {
            write_register(pgm_read_byte(&child_pipe[child]), address, 1);
        }
        write_register(pgm_read_byte(&child_payload_size[child]), payload_size);
        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
    }
}

/****************************************************************************/

void
RF24::closeReadingPipe( uint8_t pipe ) {
    write_register(EN_RXADDR,read_register(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void
RF24::toggle_features(void) {
    beginTransaction();
    spi.transfer(ACTIVATE);
    spi.transfer(0x73);
	endTransaction();
}

/****************************************************************************/

void
RF24::enableDynamicPayloads(void) {
    // Enable dynamic payload throughout the system
    //toggle_features();
    write_register(FEATURE, read_register(FEATURE) | _BV(EN_DPL) );
    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));
    dynamic_payloads_enabled = true;
}

/****************************************************************************/

void
RF24::disableDynamicPayloads(void) {
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads
    //toggle_features();
    write_register(FEATURE, 0);
    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    write_register(DYNPD, 0);
    dynamic_payloads_enabled = false;
}

/****************************************************************************/

void
RF24::enableAckPayload(void) {
    //
    // enable ack payload and dynamic payload features
    //
    //toggle_features();
    write_register(FEATURE, read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
    //
    // Enable dynamic payload on pipes 0 & 1
    //
    write_register(DYNPD, read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
    dynamic_payloads_enabled = true;
}

/****************************************************************************/

void
RF24::enableDynamicAck(void) {
    //
    // enable dynamic ack features
    //
    //toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DYN_ACK) );
    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
}

/****************************************************************************/

void
RF24::writeAckPayload(uint8_t pipe, const void* buf, uint8_t len) {
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    uint8_t data_len = rf24_min(len, 32);
    uint8_t status;
    beginTransaction();
    status = spi.transfer(W_ACK_PAYLOAD | (pipe & 0x07));
    if(status & _BV(TX_FULL)) {
        endTransaction();
        return;
    }
    while(data_len--)
        spi.transfer(*current++);
    endTransaction();
}

/****************************************************************************/

bool
RF24::isAckPayloadAvailable(void) {
    return ! (read_register(FIFO_STATUS) & _BV(RX_EMPTY));
}

/****************************************************************************/

void
RF24::setAutoAck(bool enable) {
    if(enable)
        write_register(EN_AA, 0x3F);
    else
        write_register(EN_AA, 0);
}

/****************************************************************************/

void
RF24::setAutoAck(uint8_t pipe, bool enable) {
    if(pipe <= 6) {
        uint8_t en_aa = read_register(EN_AA) ;
        if(enable) {
            en_aa |= _BV(pipe) ;
        }
        else {
            en_aa &= ~_BV(pipe) ;
        }
        write_register(EN_AA, en_aa) ;
    }
}

/****************************************************************************/

bool
RF24::testCarrier(void) {
  return (read_register(CD) & 1);
}

/****************************************************************************/

bool
RF24::testRPD(void) {
  return (read_register(RPD) & 1) ;
}

/****************************************************************************/

void
RF24::setPALevel(uint8_t level) {
    uint8_t setup = read_register(RF_SETUP) & 0xF8;
    if(level > 3) {  						// If invalid level, go to max PA
        level = (RF24_PA_MAX << 1) + 1;		// +1 to support the SI24R1 chip extra bit
    }
    else {
        level = (level << 1) + 1;	 		// Else set level as requested
    }
    setup |= level;
    write_register(RF_SETUP, setup) ;	// Write it to the chip
}

/****************************************************************************/

uint8_t
RF24::getPALevel(void) {
    return (read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

/****************************************************************************/

bool
RF24::setDataRate(rf24_datarate_e speed) {
    bool result = false;
    uint8_t setup = read_register(RF_SETUP) ;
    // HIGH and LOW '00' is 1Mbs - our default
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
    txDelay=250;
    if(speed == RF24_250KBPS) {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        setup |= _BV( RF_DR_LOW );
        txDelay=450;
    }
    else {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if(speed == RF24_2MBPS) {
            setup |= _BV(RF_DR_HIGH);
            txDelay=140;
        }
    }
    write_register(RF_SETUP, setup);
    // Verify our result
    if(read_register(RF_SETUP) == setup) {
        result = true;
    }
    return result;
}

/****************************************************************************/

rf24_datarate_e
RF24::getDataRate(void) {
    rf24_datarate_e result ;
    uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW)) {
        // '10' = 250KBPS
        result = RF24_250KBPS ;
    }
    else if (dr == _BV(RF_DR_HIGH)) {
        // '01' = 2MBPS
        result = RF24_2MBPS ;
    }
    else {
        // '00' = 1MBPS
        result = RF24_1MBPS ;
    }
    return result ;
}

/****************************************************************************/

void
RF24::setCRCLength(rf24_crclength_e length) {
    uint8_t config = read_register(NRF_CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;
    // switch uses RAM (evil!)
    if(length == RF24_CRC_DISABLED) {
        // Do nothing, we turned it off above.
    }
    else if(length == RF24_CRC_8) {
        config |= _BV(EN_CRC);
    }
    else {
        config |= _BV(EN_CRC);
        config |= _BV( CRCO );
    }
    write_register(NRF_CONFIG, config) ;
}

/****************************************************************************/

rf24_crclength_e
RF24::getCRCLength(void) {
    rf24_crclength_e result = RF24_CRC_DISABLED;
    uint8_t config = read_register(NRF_CONFIG) & (_BV(CRCO) | _BV(EN_CRC)) ;
    uint8_t AA = read_register(EN_AA);
    if(config & _BV(EN_CRC) || AA) {
        if(config & _BV(CRCO))
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }
    return result;
}

/****************************************************************************/

void
RF24::disableCRC(void) {
    uint8_t disable = read_register(NRF_CONFIG) & ~_BV(EN_CRC) ;
    write_register(NRF_CONFIG, disable) ;
}

/****************************************************************************/

void
RF24::setRetries(uint8_t delay, uint8_t count) {
    write_register(SETUP_RETR, (delay&0xf)<<ARD | (count&0xf)<<ARC);
}
