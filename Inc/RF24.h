/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * @file RF24.h
 *
 * Class declaration for RF24 and helper enums
 */

#ifndef __RF24_H__
#define __RF24_H__

#include "RF24_config.h"

#define MAX_PAYLOAD_SIZE 32

typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

class RF24
{
private:
  SPI spi;

  __IO bool bBusy;
  GPIO_TypeDef* ce_port;
  GPIO_TypeDef* csn_port;
  GPIO_TypeDef* irq_port;
  uint32_t ce_pin;
  uint16_t csn_pin;
  uint16_t irq_pin;

  IRQn_Type irqn_type;

  uint16_t spi_speed; /**< SPI Bus Speed */
  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
  uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
  uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */
  
protected:
  void beginTransaction();
  void endTransaction();
  void enableGPIOClock(GPIO_TypeDef* port);
  void InitPins(uint32_t preempPriority);

public:/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */

  RF24(GPIO_TypeDef* _ceport, uint32_t _cepin,
       GPIO_TypeDef* _csport, uint32_t _cspin,
       GPIO_TypeDef *_irwqport, uint32_t _irqpin, IRQn_Type _irqntype);
  bool begin(uint8_t channelNumber, uint32_t preempPriority);
  bool isChipConnected();
  void startListening(void);
  void stopListening(void);
  bool available(void);
  void read( void* buf, uint8_t len );
  bool write( const void* buf, uint8_t len );
  void openWritingPipe(const uint8_t *address);
  void openReadingPipe(uint8_t number, const uint8_t *address);
  void printDetails(void);
  bool available(uint8_t* pipe_num);
  bool rxFifoFull();
  void powerDown(void);
  void powerUp(void) ;
  bool write( const void* buf, uint8_t len, const bool multicast );
  bool writeFast( const void* buf, uint8_t len );
  bool writeFast( const void* buf, uint8_t len, const bool multicast );
  bool writeBlocking( const void* buf, uint8_t len, uint32_t timeout );
  bool txStandBy();
  bool txStandBy(uint32_t timeout, bool startTx = 0);
  void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
  bool isAckPayloadAvailable(void);
  void whatHappened(volatile bool *tx_ok, volatile bool *tx_fail, volatile bool *rx_ready);
  void startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx = 1 );
  void startWrite();
  void reUseTX();
  uint8_t flush_tx(void);
  bool testCarrier(void);
  bool testRPD(void) ;
  bool isValid() { return ce_pin != 0xff && csn_pin != 0xff; }
  void closeReadingPipe( uint8_t pipe ) ;
  bool failureDetected;
  void setAddressWidth(uint8_t a_width);
  void setRetries(uint8_t delay, uint8_t count);
  void setChannel(uint8_t channel);
  uint8_t getChannel(void);
  void setPayloadSize(uint8_t size);
  uint8_t getPayloadSize(void);
  uint8_t getDynamicPayloadSize(void);
  void enableAckPayload(void);
  void enableDynamicPayloads(void);
  void disableDynamicPayloads(void);
  void enableDynamicAck();
  bool isPVariant(void) ;
  void setAutoAck(bool enable);
  void setAutoAck(uint8_t pipe, bool enable) ;
  void setPALevel ( uint8_t level );
  uint8_t getPALevel( void );
  bool setDataRate(rf24_datarate_e speed);
  rf24_datarate_e getDataRate( void ) ;
  void setCRCLength(rf24_crclength_e length);
  rf24_crclength_e getCRCLength(void);
  void disableCRC( void ) ;
  void maskIRQ(bool tx_ok,bool tx_fail,bool rx_ready);
  uint8_t flush_rx(void);
  uint8_t enqueue_payload(const void* buf, uint8_t data_len);

  uint32_t txDelay;
  uint32_t csDelay;

//private:
  void csn(GPIO_PinState mode);
  void ce(GPIO_PinState level);
  uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);
  uint8_t read_register(uint8_t reg);
  uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);
  uint8_t write_register(uint8_t reg, uint8_t value);
  uint8_t read_payload(void* buf, uint8_t len);
  uint8_t write_payload(const void* buf, uint8_t len, const uint8_t writeType);
  uint8_t get_status(void);
  void print_status(uint8_t status);
  void print_observe_tx(uint8_t value);
  void print_byte_register(const char* name, uint8_t reg, uint8_t qty = 1);
  void print_address_register(const char* name, uint8_t reg, uint8_t qty = 1);
  void toggle_features(void);
  uint8_t spiTrans(uint8_t cmd);
  void errNotify(void);
};

#endif // __RF24_H__
