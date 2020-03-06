
/*
 Copyright (C)
  2011      J. Coliz <maniacbug@ymail.com>
  2015-2019 TMRh20
  2015      spaniakos <spaniakos@gmail.com>
  2015      nerdralph
  2015      zador-blood-stained
  2016      akatran
  2017-2019 Avamander <avamander@gmail.com>
  2019      IkpeohaGodson

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4_discovery.h"



#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

// RF modules support 10 Mhz SPI bus speed
const uint32_t RF24_SPI_SPEED = 10000000;

#include <radioSPI.h>


// Define _BV for non-Arduino platforms and for Arduino DUE
#define _BV(x) (1<<(x))

#define IF_SERIAL_DEBUG(x)


#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
typedef uint16_t prog_uint16_t;
#define PSTR(x) (x)

//#define printf_P printf

#define strlen_P strlen
#define PROGMEM
#define pgm_read_word(p) (*(p))
#define pgm_read_ptr(p) (*(p))
#define PRIPSTR "%s"


#endif // __RF24_CONFIG_H__
