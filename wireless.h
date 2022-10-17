//
//  wireless.h
//  Data_Teensy
//
//  Created by Ruedi Heimlicher on 25.01.2017.
//
//

#ifndef wireless_h
#define wireless_h

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

extern void wl_spi_init(void);
extern void spi_transfer_sync (uint8_t * dataout, uint8_t * datain, uint8_t len);
extern void spi_transmit_sync (uint8_t * dataout, uint8_t len) ;
extern uint8_t spi_fast_shift (uint8_t data);

#endif /* wireless_h */
