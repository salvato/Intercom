#include "main.h"
#include "config.h"
#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"


static void    csn(GPIO_PinState level);
static void    ce(GPIO_PinState level);
static uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);
static uint8_t read_register(uint8_t reg);
static uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);
static uint8_t write_register(uint8_t reg, uint8_t value);
static uint8_t read_payload(void* buf, uint8_t len);
static uint8_t write_payload(const void* buf, uint8_t len, const uint8_t writeType);
static uint8_t get_status(void);
static void    print_status(uint8_t status);
//static void    print_observe_tx(uint8_t value);
static void    print_byte_register(const char* name, uint8_t reg, uint8_t qty = 1);
static void    print_address_register(const char* name, uint8_t reg, uint8_t qty = 1);
static void    toggle_features(void);
static uint8_t spiTrans(uint8_t cmd);
static void    errNotify(void);
static void    beginTransaction();
static void    endTransaction();
static void    InitPins(uint32_t preempPriority);
static void    toggle_features(void);


static __IO     bool bBusy;
//static uint16_t spi_speed;                /**< SPI Bus Speed */
static uint8_t  payload_size;             /**< Fixed size of payloads */
static bool     dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
static uint8_t  pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
static uint8_t  addr_width;               /**< The address width to use - 3,4 or 5 bytes. */
static bool     failureDetected;
static uint32_t txDelay;
static uint32_t csDelay;


void
InitPins(uint32_t preempPriority) {
    GPIO_InitTypeDef GPIO_InitStructure;
    memset(&GPIO_InitStructure, 0, sizeof(GPIO_InitStructure));

    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = NRF24_CE_PIN;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(NRF24_CE_PORT, &GPIO_InitStructure);

    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStructure.Pin   = NRF24_CSN_PIN;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(NRF24_CSN_PORT, &GPIO_InitStructure);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin   = NRF24_IRQ_PIN;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(NRF24_IRQ_PORT, &GPIO_InitStructure);

    HAL_NVIC_SetPriority(NRF24_IRQ_CHAN, preempPriority, 0);
}


bool
rf24Init(uint8_t channelNumber, uint32_t preempPriority) {
    payload_size             = MAX_PAYLOAD_SIZE;
    dynamic_payloads_enabled = false;
    addr_width               = 5;
    csDelay                  = 130;
    pipe0_reading_address[0] = 0;
    uint8_t setup            = 0;
    bBusy                    = false;

    // Initialize pins
    InitPins(preempPriority);
    // Initialize SPI interface
    spiInit(SPI_MODE_MASTER);

    rf24PowerDown();
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
    write_register(NRF_CONFIG, &setup, 1);
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

    rf24SetRetries(1, 0); // 500 us and Single Retry

    // Reset value is MAX
    rf24SetPALevel(RF24_PA_MAX);

    // Get the RF setup to be sure the module is attached and responding
    setup = read_register(RF_SETUP);
    // if setup is 0 or 0xff then there was no response from module
    if(setup == 0 || setup == 0xff)
        return false;

    // Set the on Air Data Rate
    rf24SetDataRate(RF24_2MBPS);

    rf24SetChannel(channelNumber);

    // Flush FIFO buffers
    rf24Flush_rx();
    rf24Flush_tx();

    // Power up by default when begin() is called
    rf24PowerUp();

    // Clear Interrupt Request...
    rf24ClearInterrupts();
    // Enable hardware interrupts
    rf24EnableIRQ();

    return true;
}


void
rf24ClearInterrupts() {
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
}


void
rf24EnableIRQ() {
    __HAL_GPIO_EXTI_CLEAR_IT(NRF24_IRQ_CHAN);
    HAL_NVIC_EnableIRQ(NRF24_IRQ_CHAN);
}


void
rf24disableIRQ() {
    HAL_NVIC_DisableIRQ(NRF24_IRQ_CHAN);
}


void
csn(GPIO_PinState level) {
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, level);
	delayMicroseconds(csDelay);
}


void
ce(GPIO_PinState level) {
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN,level);
}


void
beginTransaction() {
    csn(GPIO_PIN_RESET);
}


void
endTransaction() {
    csn(GPIO_PIN_SET);
}


uint8_t
read_register(uint8_t reg, uint8_t* buf, uint8_t len) {
    uint8_t status;
    beginTransaction();
    status = spiTransfer( R_REGISTER | ( REGISTER_MASK & reg ) );
    while ( len-- ){
        *buf++ = spiTransfer(0xff);
    }
    endTransaction();
    return status;
}


uint8_t
read_register(uint8_t reg) {
    uint8_t result = 0;
    beginTransaction();
    spiTransfer( R_REGISTER | ( REGISTER_MASK & reg ) );
    result = spiTransfer(0xff);
    endTransaction();
    return result;
}


uint8_t
write_register(uint8_t reg, const uint8_t* buf, uint8_t len) {
    uint8_t status;
    beginTransaction();
    status = spiTransfer( W_REGISTER | ( REGISTER_MASK & reg ) );
    while ( len-- )
        spiTransfer(*buf++);
    endTransaction();
    return status;
}


uint8_t
write_register(uint8_t reg, uint8_t value) {
    uint8_t status = 0;
    IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));
    beginTransaction();
    status = spiTransfer( W_REGISTER | ( REGISTER_MASK & reg ) );
    spiTransfer(value);
    endTransaction();
    return status;
}


uint8_t
rf24Enqueue_payload(const void* buf, uint8_t data_len) {
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
    status = spiTransfer(W_TX_PAYLOAD);
    spiTransfer((void *)current, data_len);
    while(blank_len--) {
        spiTransfer(0);
    }
    endTransaction(); // csn(GPIO_PIN_SET)
    return status;
}


uint8_t
write_payload(const void* buf, uint8_t data_len, const uint8_t writeType) {
    uint8_t status;
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);
    data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    IF_SERIAL_DEBUG( printf("[Writing %u bytes %u blanks]\n",data_len,blank_len); );

    beginTransaction(); // csn(GPIO_PIN_RESET)
    status = spiTransfer(writeType);
    while(data_len--) {
        spiTransfer(*current++);
    }
    while(blank_len--) {
        spiTransfer(0);
    }
    endTransaction(); // csn(GPIO_PIN_SET)
    return status;
}


uint8_t
read_payload(void* buf, uint8_t data_len) {
    uint8_t status;
    uint8_t* current = reinterpret_cast<uint8_t*>(buf);
    if(data_len > payload_size) data_len = payload_size;
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    IF_SERIAL_DEBUG( printf("[Reading %u bytes %u blanks]\n",data_len,blank_len); );
    beginTransaction();
    status = spiTransfer(R_RX_PAYLOAD);
    spiTransfer(current, data_len);
    if(blank_len)
        spiTransfer(current+data_len, blank_len);
    endTransaction();
    return status;
}


uint8_t
rf24Flush_rx(void) {
    return spiTrans(FLUSH_RX);
}


uint8_t
rf24Flush_tx(void) {
    return spiTrans(FLUSH_TX);
}


uint8_t
spiTrans(uint8_t cmd){
    uint8_t status;
    beginTransaction();
    status = spiTransfer(cmd);
    endTransaction();
    return status;
}


uint8_t
get_status(void) {
    return spiTrans(RF24_NOP);
}


char buffer[255];


void
print_status(uint8_t status) {
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


//void
//print_observe_tx(uint8_t value) {
//    sprintf(buffer,
//            PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
//            value,
//            (value >> PLOS_CNT) & 0x0F,
//            (value >> ARC_CNT) & 0x0F
//            );
//}


void
print_byte_register(const char* name, uint8_t reg, uint8_t qty) {
    int istart =0;
    istart += sprintf(buffer, PSTR(PRIPSTR"\t ="), name);
    while(qty--)
        istart += sprintf(buffer+istart, PSTR(" 0x%02x"), read_register(reg++));
    sprintf(buffer+istart, PSTR("\r\n"));
}


void
print_address_register(const char* name, uint8_t reg, uint8_t qty) {
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


void
rf24SetChannel(uint8_t channel) {
    const uint8_t max_channel = 125;
    write_register(RF_CH,rf24_min(channel,max_channel));
}


uint8_t
rf24GetChannel() {
    return read_register(RF_CH);
}


void
rf24SetPayloadSize(uint8_t size) {
    payload_size = rf24_min(size, 32);
}


uint8_t
rf24GetPayloadSize(void) {
    return payload_size;
}


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
rf24PrintDetails(void) {
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

    sprintf(buffer, PSTR("Data Rate\t = " PRIPSTR "\r\n"), pgm_read_ptr(&rf24_datarate_e_str_P[rf24GetDataRate()]));
    sprintf(buffer, PSTR("CRC Length\t = " PRIPSTR "\r\n"), pgm_read_ptr(&rf24_crclength_e_str_P[rf24GetCRCLength()]));
    sprintf(buffer, PSTR("PA Power\t = " PRIPSTR "\r\n"),  pgm_read_ptr(&rf24_pa_dbm_e_str_P[rf24GetPALevel()]));
}


bool
rf24IsChipConnected() {
  uint8_t setup = read_register(SETUP_AW);
  if(setup >= 1 && setup <= 3) {
    return true;
  }
  return false;
}


void
rf24StartListening(void) {
    rf24PowerUp();
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) | _BV(PRIM_RX));
    write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
    ce(GPIO_PIN_SET);
    // Restore the pipe0 adddress, if exists
    if (pipe0_reading_address[0] > 0) {
        write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
    }
    else {
        rf24CloseReadingPipe(0);
    }
    // Flush buffers
    //flush_rx();
    if(read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        rf24Flush_tx();
    }
    // Go!
    //delayMicroseconds(100);
}


static const
uint8_t child_pipe_enable[] PROGMEM = {
    ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};


void
rf24StopListening(void) {
    ce(GPIO_PIN_RESET);
    delayMicroseconds(txDelay);//140
    if(read_register(FEATURE) & _BV(EN_ACK_PAY)) {
        delayMicroseconds(txDelay); //140
        rf24Flush_tx();
    }
    write_register(NRF_CONFIG, (read_register(NRF_CONFIG)) & ~_BV(PRIM_RX) );
    write_register(EN_RXADDR, read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0
}


void
rf24PowerDown(void) {
    ce(GPIO_PIN_RESET); // Guarantee CE is low on powerDown
    write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~_BV(PWR_UP));
}


//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void
rf24PowerUp(void) {
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


void
errNotify() {
	failureDetected = 1;
}


//Similar to the previous write, clears the interrupt flags
bool
rf24Write( const void* buf, uint8_t len, const bool multicast ) {
    //Start Writing
    rf24StartFastWrite(buf, len, multicast);
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
        rf24Flush_tx(); //Only going to be 1 packet into the FIFO at a time using this method, so just flush
        return 0;
    }
    //TX OK 1 or 0
    return 1;
}


bool
rf24Write(const void* buf, uint8_t len) {
    return rf24Write(buf, len, 0);
}


//For general use, the interrupt flags are not important to clear
bool
rf24WriteBlocking( const void* buf, uint8_t len, uint32_t timeout ) {
    //Block until the FIFO is NOT full.
    //Keep track of the MAX retries and set auto-retry if seeing failures
    //This way the FIFO will fill up and allow blocking until packets go through
    //The radio will auto-clear everything in the FIFO as long as CE remains high
    uint32_t timer = HAL_GetTick();							  //Get the time that the payload transmission started
    while( ( get_status()  & ( _BV(TX_FULL) ))) {		  //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout
        if( get_status() & _BV(MAX_RT)){					  //If MAX Retries have been reached
            rf24ReUseTX();										  //Set re-transmit and clear the MAX_RT interrupt flag
            if(HAL_GetTick() - timer > timeout){ return 0; }		  //If this payload has exceeded the user-defined timeout, exit and return 0
        }
        if(HAL_GetTick() - timer > (timeout+95) ){
            errNotify();
            return 0;
        }
    }
    //Start Writing
    rf24StartFastWrite(buf, len, 0); //Write the payload if a buffer is clear
    return 1; //Return 1 to indicate successful transmission
}


void
rf24ReUseTX(){
    write_register(NRF_STATUS,_BV(MAX_RT));			  //Clear max retry flag
    spiTrans(REUSE_TX_PL);
    ce(GPIO_PIN_RESET);										  //Re-Transfer packet
    ce(GPIO_PIN_SET);
}


bool
rf24WriteFast( const void* buf, uint8_t len, const bool multicast ) {
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
    rf24StartFastWrite(buf, len, multicast);
    return 1;
}


bool
rf24WriteFast(const void* buf, uint8_t len) {
    return rf24WriteFast(buf, len, 0);
}


// Per the documentation, we want to set PTX Mode when not listening.
// Then all we do is write data and set CE high
// In this mode, if we can keep the FIFO buffers loaded,
// packets will transmit immediately (no 130us delay)
// Otherwise we enter Standby-II mode, which is still faster than standby mode
// Also, we remove the need to keep writing the config register over and over
// and delaying for 150 us each time if sending a stream of data
void
rf24StartFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx) { //TMRh20
    write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    if(startTx) {
        ce(GPIO_PIN_SET);
	}
}


void
rf24StartWrite() {
    // Send the payload from FIFO
    while(bBusy){}
    bBusy = true;
    ce(GPIO_PIN_SET);
    delayMicroseconds(10);
    ce(GPIO_PIN_RESET);
    bBusy = false;
}


bool
rf24RxFifoFull(){
	return read_register(FIFO_STATUS) & _BV(RX_FULL);
}


bool
rf24TxStandBy() {
    while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ) {
        if(get_status() & _BV(MAX_RT)){
            write_register(NRF_STATUS, _BV(MAX_RT));
            ce(GPIO_PIN_RESET);
            rf24Flush_tx(); // Non blocking, flush the data
            return 0;
        }
    }
    ce(GPIO_PIN_RESET); // Set STANDBY-I mode
    return 1;
}


bool
rf24TxStandBy(uint32_t timeout, bool startTx) {
    if(startTx){
        rf24StopListening();
        ce(GPIO_PIN_SET);
    }
    uint32_t start = HAL_GetTick();
    while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
        if(get_status() & _BV(MAX_RT)) {
            write_register(NRF_STATUS,_BV(MAX_RT) );
            ce(GPIO_PIN_RESET);							  //Set re-transmit
            ce(GPIO_PIN_SET);
            if(HAL_GetTick() - start >= timeout) {
                ce(GPIO_PIN_RESET); rf24Flush_tx();
                return 0;
            }
        }
        if(HAL_GetTick() - start > (timeout+95)) {
            errNotify();
            return 0;
        }
    }
    ce(GPIO_PIN_RESET);				   //Set STANDBY-I mode
    return 1;
}


void
rf24MaskIRQ(bool tx, bool fail, bool rx) {
	uint8_t config = read_register(NRF_CONFIG);
	/* clear the interrupt flags */
	config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	write_register(NRF_CONFIG, config);
}


uint8_t
rf24GetDynamicPayloadSize(void) {
    uint8_t result = 0;
    beginTransaction();
    spiTransfer( R_RX_PL_WID );
    result = spiTransfer(0xff);
    endTransaction();
    if(result > 32) {
        rf24Flush_rx();
        HAL_Delay(2);
        return 0;
    }
    return result;
}


bool
rf24Available(void) {
    return rf24Available(NULL);
}


bool
rf24Available(uint8_t* pipe_num) {
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


void
rf24Read( void* buf, uint8_t len) {
    // Fetch the payload
    read_payload(buf, len);
    //Clear the two possible interrupt flags with one command
    //write_register(NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );
}

/****************************************************************************/

void
rf24WhatHappened(__IO bool* tx_ok, __IO bool* tx_fail, __IO bool* rx_ready) {
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    uint8_t status = write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
    // Report to the user what happened
    *tx_ok    = status & _BV(TX_DS);
    *tx_fail  = status & _BV(MAX_RT);
    *rx_ready = status & _BV(RX_DR);
}


void
rf24OpenWritingPipe(const uint8_t *address) {
    // Note that NRF24L01(+) expects address LSB first
    write_register(RX_ADDR_P0, address, addr_width);
    write_register(TX_ADDR, address, addr_width);
    write_register(RX_PW_P0, payload_size);
}


static const uint8_t
child_pipe[] PROGMEM = {
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};


static const uint8_t
child_payload_size[] PROGMEM = {
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};


void
rf24SetAddressWidth(uint8_t a_width) {
    if(a_width -= 2){
        write_register(SETUP_AW,a_width%4);
        addr_width = (a_width%4) + 2;
    }
    else {
        write_register(SETUP_AW,0);
        addr_width = 2;
    }
}


void
rf24OpenReadingPipe(uint8_t child, const uint8_t *address) {
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


void
rf24CloseReadingPipe( uint8_t pipe ) {
    write_register(EN_RXADDR,read_register(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}


void
toggle_features(void) {
    beginTransaction();
    spiTransfer(ACTIVATE);
    spiTransfer(0x73);
	endTransaction();
}


void
rf24EnableDynamicPayloads(void) {
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


void
rf24DisableDynamicPayloads(void) {
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


void
rf24EnableAckPayload(void) {
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


void
rf24EnableDynamicAck(void) {
    //
    // enable dynamic ack features
    //
    //toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DYN_ACK) );
    IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));
}


void
rf24WriteAckPayload(uint8_t pipe, const void* buf, uint8_t len) {
    const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

    uint8_t data_len = rf24_min(len, 32);
    uint8_t status;
    beginTransaction();
    status = spiTransfer(W_ACK_PAYLOAD | (pipe & 0x07));
    if(status & _BV(TX_FULL)) {
        endTransaction();
        return;
    }
    while(data_len--)
        spiTransfer(*current++);
    endTransaction();
}


bool
rf24IsAckPayloadAvailable(void) {
    return ! (read_register(FIFO_STATUS) & _BV(RX_EMPTY));
}


void
rf24SetAutoAck(bool enable) {
    if(enable)
        write_register(EN_AA, 0x3F);
    else
        write_register(EN_AA, 0);
}


void
rf24SetAutoAck(uint8_t pipe, bool enable) {
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


bool
rf24TestCarrier(void) {
  return (read_register(CD) & 1);
}


bool
rf24TestRPD(void) {
  return (read_register(RPD) & 1) ;
}


void
rf24SetPALevel(uint8_t level) {
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


uint8_t
rf24GetPALevel(void) {
    return (read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}


bool
rf24SetDataRate(rf24_datarate_e speed) {
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


rf24_datarate_e
rf24GetDataRate(void) {
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


void
rf24SetCRCLength(rf24_crclength_e length) {
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


rf24_crclength_e
rf24GetCRCLength(void) {
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


void
rf24DisableCRC(void) {
    uint8_t disable = read_register(NRF_CONFIG) & ~_BV(EN_CRC) ;
    write_register(NRF_CONFIG, disable) ;
}


void
rf24SetRetries(uint8_t delay, uint8_t count) {
    write_register(SETUP_RETR, (delay&0xf)<<ARD | (count&0xf)<<ARC);
}


bool
rf24IsValid() {
    return NRF24_CE_PIN != 0xff && NRF24_CSN_PIN != 0xff;
}
