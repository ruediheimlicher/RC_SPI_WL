//
//  wireless.c
//  Data_Teensy
//
//  Created by Ruedi Heimlicher on 17.10.2022.
//
//  NRF24L01 Tutorial microcontroller.net
//

#include "wireless.h"
#include "wl_module.c"
#include "defines.h"


#define WL_SS       DDB2

#define WL_PIPE   1

void wl_spi_init()

// Initialize pins for spi communication
{
   
   SPI_PORT &= ~((1<<SPI_MOSI)|(1<<SPI_MISO)|(1<<SPI_SS)|(1<<SPI_CLK));
   // Define the following pins as output
   SPI_DDR |= ((1<<SPI_MOSI)|(1<<SPI_SS)|(1<<SPI_CLK));
   SPCR = ((1<<SPE)|
           (0<<SPIE)|
           (0<<DORD)|
           // 1:LSB first)
           (1<<MSTR)|
           (0<<SPR1)|(1<<SPR0)|
           (0<<CPOL)|(0<<CPHA));// 1:SCK hi when idle, 0: SCK lo when idle
   
   // 1:trailing edge sampling)
   // SPI Enable
   // SPI Interrupt Enable
   // Data Order (0:MSB first
   // Master/Slave select
   // SPI Clock Rate
   // Clock Polarity (0:SCK l
   // Clock Phase (0:leading
   
//   SPSR = (1<<SPI2X);// Double Clock Rate
   
   
   
   //Stromlaufplan für den Anschluß an einen ATmega8
}

void spi_transfer_sync (uint8_t * dataout, uint8_t * datain, uint8_t len)
// Shift full array through target device
{
   uint8_t i;
   for (i = 0; i < len; i++)
   {
      SPDR = dataout[i]; while((SPSR & (1<<SPIF))==0); datain[i] = SPDR;
   }
}
void spi_transmit_sync (uint8_t * dataout, uint8_t len)
// Shift full array to target device without receiving any byt e
{
   uint8_t i;
   for (i = 0; i < len; i++)
   {
      SPDR = dataout[i]; while((SPSR & (1<<SPIF))==0);
   }
}
uint8_t spi_fast_shift (uint8_t data)
// Clocks only one byte to target device and returns the recei ved one
{
   SPDR = data;
   while((SPSR & (1<<SPIF))==0); return SPDR;
}



