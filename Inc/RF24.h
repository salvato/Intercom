#ifndef __RF24_H__
#define __RF24_H__

#include "RF24_config.h"

#define MAX_PAYLOAD_SIZE 32

typedef enum { RF24_PA_MIN = 0,
               RF24_PA_LOW,
               RF24_PA_HIGH,
               RF24_PA_MAX,
               RF24_PA_ERROR
             } rf24_pa_dbm_e ;

typedef enum { RF24_1MBPS = 0,
               RF24_2MBPS,
               RF24_250KBPS
             } rf24_datarate_e;

typedef enum { RF24_CRC_DISABLED = 0,
               RF24_CRC_8,
               RF24_CRC_16
             } rf24_crclength_e;

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */


    bool    rf24Init(uint8_t channelNumber, uint32_t preempPriority);
    void    rf24EnableIRQ();
    void    rf24disableIRQ();
    void    rf24ClearInterrupts();
    bool    rf24IsChipConnected();
    void    rf24StartListening(void);
    void    rf24StopListening(void);
    bool    rf24Available(void);
    void    rf24Read(void* buf, uint8_t len);
    bool    rf24Write(const void* buf, uint8_t len);
    void    rf24OpenWritingPipe(const uint8_t *address);
    void    rf24OpenReadingPipe(uint8_t number, const uint8_t *address);
    void    rf24PrintDetails(void);
    bool    rf24Available(uint8_t* pipe_num);
    bool    rf24RxFifoFull();
    void    rf24PowerDown(void);
    void    rf24PowerUp(void) ;
    bool    rf24Write(const void* buf, uint8_t len, const bool multicast);
    bool    rf24WriteFast(const void* buf, uint8_t len);
    bool    rf24WriteFast(const void* buf, uint8_t len, const bool multicast);
    bool    rf24WriteBlocking(const void* buf, uint8_t len, uint32_t timeout);
    bool    rf24TxStandBy();
    bool    rf24TxStandBy(uint32_t timeout, bool startTx = 0);
    void    rf24WriteAckPayload(uint8_t pipe, const void* buf, uint8_t len);
    bool    rf24IsAckPayloadAvailable(void);
    void    rf24WhatHappened(volatile bool *tx_ok, volatile bool *tx_fail, volatile bool *rx_ready);
    void    rf24StartFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx = 1 );
    void    rf24StartWrite();
    void    rf24ReUseTX();
    uint8_t rf24Flush_tx(void);
    bool    rf24TestCarrier(void);
    bool    rf24TestRPD(void) ;
    bool    rf24IsValid();
    void    rf24CloseReadingPipe( uint8_t pipe ) ;
    void    rf24SetAddressWidth(uint8_t a_width);
    void    rf24SetRetries(uint8_t delay, uint8_t count);
    void    rf24SetChannel(uint8_t channel);
    uint8_t rf24GetChannel(void);
    void    rf24SetPayloadSize(uint8_t size);
    uint8_t rf24GetPayloadSize(void);
    uint8_t rf24GetDynamicPayloadSize(void);
    void    rf24EnableAckPayload(void);
    void    rf24EnableDynamicPayloads(void);
    void    rf24DisableDynamicPayloads(void);
    void    rf24EnableDynamicAck();
    void    rf24SetAutoAck(bool enable);
    void    rf24SetAutoAck(uint8_t pipe, bool enable) ;
    void    rf24SetPALevel(uint8_t level);
    uint8_t rf24GetPALevel(void);
    bool    rf24SetDataRate(rf24_datarate_e speed);
    void    rf24SetCRCLength(rf24_crclength_e length);
    void    rf24DisableCRC(void) ;
    void    rf24MaskIRQ(bool tx_ok,bool tx_fail,bool rx_ready);
    uint8_t rf24Flush_rx(void);
    uint8_t rf24Enqueue_payload(const void* buf, uint8_t data_len);
    rf24_datarate_e  rf24GetDataRate(void) ;
    rf24_crclength_e rf24GetCRCLength(void);

#endif // __RF24_H__
