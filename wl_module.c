/*
	Copyright (c) 2011 by Ernst Buchmann
	
	Code based on the work of Stefan Engelke and Brennan Ball
	
 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 DEALINGS IN THE SOFTWARE.
 
 
 */

#include "wl_module.h"
#include "nRF24L01.h"
//#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Defines for setting the wl_module registers for transmitting or receiving mode
#define TX_POWERUP wl_module_config_register(CONFIG, wl_module_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) )
#define RX_POWERUP wl_module_config_register(CONFIG, wl_module_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) )



void wl_module_init()
// Initializes pins and interrupt to communicate with the wl_module
// Should be called in the early initializing phase at startup.
{

   // Define CSN and CE as Output and set them to default
   SPI_WL_DDR     |= (1<<SPI_WL_CE);	//Ausgang fuer WL-CE
   SPI_WL_PORT    |= (1<<SPI_WL_CE);	//HI
   SPI_WL_DDR     |= (1<<SPI_WL_CSN);	//Ausgang fuer WL-CSN
   SPI_WL_PORT    |= (1<<SPI_WL_CSN);	//HI
   
 
   // Interrupt auf INT1
   INTERRUPT_DDR &= ~(1<<INT1_PIN);
   INTERRUPT_PORT |= (1<<INT1_PIN);
   
   
   wl_module_CE_lo;
   wl_module_CSN_hi;
   
#if defined(__AVR_ATmega8__)
   // Initialize external interrupt 0 (PD2)
   MCUCR = ((1<<ISC11));	// Set external interupt on falling edge
   GICR  = ((1<<INT1));							// Activate INT1
#endif // __AVR_ATmega8__
   
#if defined(__AVR_ATmega88A__)
   EICRA = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));	// Set external interupt on falling edge for INT0 and INT1
   EIMSK  = ((0<<INT1)|(1<<INT0));							// Activate INT0
#endif // __AVR_ATmega88A__
   
#if defined(__AVR_ATmega168__)
   // Initialize external interrupt on port PD6 (PCINT22)
   DDRB &= ~(1<<PD6);
   PCMSK2 = (1<<PCINT22);
   PCICR  = (1<<PCIE2);
#endif // __AVR_ATmega168__
   
#if defined(__AVR_ATmega32U4__)
   // Initialize external interrupt on port PD0
   INTERRUPT_DDR &= ~(1<<INT0_PIN);
   INTERRUPT_PORT |= (1<<INT0_PIN);
   
   EICRA = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));
   // Set external interupt on falling edge for INT0 and INT1
   EIMSK  = ((0<<INT1)|(1<<INT0));
   
#endif // __AVR_ATmega32U4__
   
   
   
   // Initialize spi module
   wl_spi_init();
}


void wl_module_config()
// Sets the important registers in the wl-module and powers the module
// in receiving mode
{
   // Set RF channel
   wl_module_config_register(RF_CH,wl_module_CH);
   // Set data speed & Output Power configured in wl_module.h
   wl_module_config_register(RF_SETUP,wl_module_RF_SETUP);
   // Set length of incoming payload
   wl_module_config_register(RX_PW_P0, wl_module_PAYLOAD);
   
   // Start receiver
   PTX = 0;        // Start in receiving mode
   RX_POWERUP;     // Power up in receiving mode
   wl_module_CE_hi;     // Listening for pakets
}

extern void wl_module_rx_config()
// Sets the important registers in the wl-module and powers the module
// in receiving mode
{
   uint8_t data[5];
   // Set RF channel
   wl_module_config_register(RF_CH,wl_module_CH);
   // Set data speed & Output Power configured in wl_module.h
   wl_module_config_register(RF_SETUP,wl_module_RF_SETUP);
   //Enable all RX Data-Pipes
   wl_module_config_register(EN_RXADDR, EN_RXADDR_ERX_ALL);
   //Set RX_Address Pipe 0
   data[0]= data[1]= data[2]= data[3]= data[4]= RX_ADDR_P0_B0_DEFAULT_VAL;
   wl_module_set_rx_addr(data, 5, 0);
   
   //Set RX_Address Pipe 1
   data[0]= data[1]= data[2]= data[3]= data[4]= RX_ADDR_P1_B0_DEFAULT_VAL;
   wl_module_set_rx_addr(data, 5, 1);
   
   //Set RX_Address Pipe 2-5
   data[0]=RX_ADDR_P2_DEFAULT_VAL;
   wl_module_set_rx_addr(data, 1, 2);
   
   data[0]=RX_ADDR_P3_DEFAULT_VAL;
   wl_module_set_rx_addr(data, 1, 3);
   
   data[0]=RX_ADDR_P4_DEFAULT_VAL;
   wl_module_set_rx_addr(data, 1, 4);
   data[0]=RX_ADDR_P5_DEFAULT_VAL;
   wl_module_set_rx_addr(data, 1, 5);
   // Set length of incoming payload
   wl_module_config_register(RX_PW_P0, wl_module_PAYLOAD);
   wl_module_config_register(RX_PW_P1, wl_module_PAYLOAD);
   wl_module_config_register(RX_PW_P2, wl_module_PAYLOAD);
   wl_module_config_register(RX_PW_P3, wl_module_PAYLOAD);
   wl_module_config_register(RX_PW_P4, wl_module_PAYLOAD);
   wl_module_config_register(RX_PW_P5, wl_module_PAYLOAD);
   
   
   // Start receiver
   PTX = 0;        // Start in receiving mode
   RX_POWERUP;     // Power up in receiving mode
   wl_module_CE_hi;     // Listening for pakets
}

// Sets the wl-module as one of the six sender. Define for every sender a unique Number (wl_module_TX_NR_x)
// when you call this Function.
//  Each TX will get a TX-Address corresponding to the RX-Device.
// RX_Address_Pipe_0 must be the same as the TX-Address
extern void wl_module_tx_config(uint8_t tx_nr)
{
   uint8_t tx_addr[5];
   
   // Set RF channel
   wl_module_config_register(RF_CH,wl_module_CH);
   // Set data speed & Output Power configured in wl_module.h
   wl_module_config_register(RF_SETUP,wl_module_RF_SETUP);
   //Config the CONFIG Register (Mask IRQ, CRC, etc)
   wl_module_config_register(CONFIG, wl_module_CONFIG);
   // Set length of incoming payload
   //wl_module_config_register(RX_PW_P0, wl_module_PAYLOAD);
   
   wl_module_config_register(SETUP_RETR,(SETUP_RETR_ARD_750 | SETUP_RETR_ARC_15));
   
   //set the TX address for the pipe with the same number as the iteration
			switch(tx_nr)
			{
            case 0: //setup TX address as default RX address for pipe 0 (E7:E7:E7:E7:E7)
               tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P0_B0_DEFAULT_VAL;
               wl_module_set_TADDR(tx_addr);
               wl_module_set_RADDR(tx_addr);
               break;
            case 1: //setup TX address as default RX address for pipe 1 (C2:C2:C2:C2:C2)
               tx_addr[0] = tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
               wl_module_set_TADDR(tx_addr);
               wl_module_set_RADDR(tx_addr);
               break;
            case 2: //setup TX address as default RX address for pipe 2 (C2:C2:C2:C2:C3)
               tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
               tx_addr[0] = RX_ADDR_P2_DEFAULT_VAL;
               wl_module_set_TADDR(tx_addr);
               wl_module_set_RADDR(tx_addr);
               break;
            case 3: //setup TX address as default RX address for pipe 3 (C2:C2:C2:C2:C4)
               tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
               tx_addr[0] = RX_ADDR_P3_DEFAULT_VAL;
               wl_module_set_TADDR(tx_addr);
               wl_module_set_RADDR(tx_addr);
               break;
            case 4: //setup TX address as default RX address for pipe 4 (C2:C2:C2:C2:C5)
               tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
               tx_addr[0] = RX_ADDR_P4_DEFAULT_VAL;
               wl_module_set_TADDR(tx_addr);
               wl_module_set_RADDR(tx_addr);
               break;
            case 5: //setup TX address as default RX address for pipe 5 (C2:C2:C2:C2:C6)
               tx_addr[1] = tx_addr[2] = tx_addr[3] = tx_addr[4] = RX_ADDR_P1_B0_DEFAULT_VAL;
               tx_addr[0] = RX_ADDR_P5_DEFAULT_VAL;
               wl_module_set_TADDR(tx_addr);
               wl_module_set_RADDR(tx_addr);
               break;
         }
   
   PTX =0;
   TX_POWERUP;
   /*
    // Start receiver
    PTX = 0;        // Start in receiving mode
    RX_POWERUP;     // Power up in receiving mode
    wl_module_CE_hi;     // Listening for pakets
    */
}

//sets the TX address in the TX_ADDR register
//unsigned char * address is the actual address to be used.  It should be sized
//	according to the tx_addr length specified to the nrf24l01.
//unsigned int len is the length of the address.  Its value should be specified
//	according to the tx_addr length specified to the nrf24l01.
extern void wl_module_set_tx_addr(uint8_t * address, uint8_t len)
{
   wl_module_write_register(TX_ADDR, address, len);
}

//sets up the 24L01 as a transmitter
//this function takes the existing contents of the CONFIG register and simply
//  clears the PRIM_RX bit in the CONFIG register.
//note: if the read value of the CONFIG register already has the PRIM_RX bit cleared, this
//  function exits in order to not make an unecessary register write.
extern void wl_module_set_as_tx()
{
   unsigned char config;
   
   wl_module_read_register(CONFIG, &config, 1);
   
   if((config & CONFIG_PRIM_RX) == 0)
      return;
   
   config &= (~CONFIG_PRIM_RX);
   
   wl_module_write_register(CONFIG, &config, 1);
   
   wl_module_CE_lo;
}

//powers down the 24L01
//this function takes the existing contents of the CONFIG register and simply
//  clears the PWR_UP bit in the CONFIG register.
//note: if the read value of the CONFIG register already has the PWR_UP bit cleared, this
//  function exits in order to not make an unecessary register write.
extern void wl_module_power_down()
{
   unsigned char config;
   
   wl_module_read_register(CONFIG, &config, 1);
   
   if((config & CONFIG_PWR_UP) == 0)
      return;
   
   config &= (~CONFIG_PWR_UP);
   
   wl_module_write_register(CONFIG, &config, 1);
   
   wl_module_CE_lo;
}

//sets the RX address in the RX_ADDR register that is offset by rxpipenum
//unsigned char * address is the actual address to be used.  It should be sized
//	according to the rx_addr length that is being filled.
//unsigned int len is the length of the address.  Its value should be specified
//	according to the rx_addr length specified to the nrf24l01.
//unsigned char rxpipenum is the pipe number (zero to five) whose address is being
//	specified.  If an invalid address (greater than five) is supplied, the function
//  does nothing.
extern void wl_module_set_rx_addr(uint8_t * address, uint8_t len, uint8_t rxpipenum)
{
   if(rxpipenum > 5)
      return;
   
   wl_module_write_register(RX_ADDR_P0 + rxpipenum, address, len);
}

extern void wl_module_get_rx_addr(uint8_t *data, uint8_t rxpipenum, uint8_t len)
{
   
   if((rxpipenum > 5))
      return;
   
   wl_module_read_register(RX_ADDR_P0 + rxpipenum, data, len);
   
   
}

//sets the RX payload width on the pipe offset by rxpipenum
//unsigned char payloadwidth is the length of the payload for the pipe referenced in
//  rxpipenum.  It must be less than or equal to 32.  If an invalid payload width is
//  specified, the function does nothing.
//unsigned char rxpipenum is the pipe number (zero to five) whose address is being
//	specified.  If an invalid address (greater than five) is supplied, the function
//  does nothing.
extern void wl_module_set_rx_pw(unsigned char payloadwidth, unsigned char rxpipenum)
{
   if((rxpipenum > 5) || (payloadwidth > 32))
      return;
   
   wl_module_write_register(RX_PW_P0 + rxpipenum, &payloadwidth, 1);
}

//gets the RX payload width on the pipe offset by rxpipenum
//unsigned char rxpipenum is the pipe number (zero to five) whose address is being
//	specified.  If an invalid address (greater than five) is supplied, the function
//  does nothing.
extern uint8_t wl_module_get_rx_pw(uint8_t rxpipenum)
{
   unsigned char data;
   
   if((rxpipenum > 5))
      return 0;
   
   wl_module_read_register(RX_PW_P0 + rxpipenum, &data, 1);
   
   return data;
}

//returns the current pipe in the 24L01's STATUS register
extern uint8_t wl_module_get_rx_pipe()
{
   return wl_module_get_rx_pipe_from_status(wl_module_get_status());
}

extern uint8_t wl_module_get_rx_pipe_from_status(uint8_t status)
{
   return ((status & 0xE) >> 1);
}

void wl_module_set_RADDR(uint8_t * adr)
// Sets the receiving address
{
   wl_module_CE_lo;
   wl_module_write_register(RX_ADDR_P0,adr,5);
   wl_module_CE_hi;
}

void wl_module_set_TADDR(uint8_t * adr)
// Sets the transmitting address
{
   wl_module_write_register(TX_ADDR, adr,5);
}



extern uint8_t wl_module_data_ready()
// Checks if data is available for reading
{
   if (PTX) return 0;
   uint8_t status;
   // Read wl_module status
   wl_module_CSN_lo;                                // Pull down chip select
   status = spi_fast_shift(NOP);               // Read status register
   wl_module_CSN_hi;                                // Pull up chip select
   return status & (1<<RX_DR);
}

//returns true if TX_EMPTY bit in FIFO_STATUS register is set, false otherwise
extern uint8_t wl_module_fifo_tx_empty()
{
   uint8_t data;
   
   wl_module_read_register(FIFO_STATUS, &data, 1);
   
   
   return (data & FIFO_STATUS_TX_EMPTY);
}

//returns true if RX_EMPTY bit in FIFO_STATUS register is set, false otherwise
extern uint8_t wl_module_fifo_rx_empty()
{
   uint8_t data;
   
   wl_module_read_register(FIFO_STATUS, &data, 1);
   
   return (data & FIFO_STATUS_RX_EMPTY);
}

//returns the current RF channel in RF_CH register
extern uint8_t wl_module_get_rf_ch()
{
   uint8_t data;
   
   wl_module_read_register(RF_CH, &data, 1);
   
   return data;
}

//returns the current RF_SETUP Register
extern uint8_t wl_module_get_rf_setup()
{
   uint8_t data;
   
   wl_module_read_register(RF_SETUP, &data, 1);
   
   return data;
}

//returns the current PLOS_CNT value in OBSERVE_TX register
extern uint8_t wl_module_get_plos_cnt()
{
   uint8_t data;
   
   wl_module_read_register(OBSERVE_TX, &data, 1);
   
   return ((data & OBSERVE_TX_PLOS_CNT) >> 4);
}

//returns the current ARC_CNT value in OBSERVE_TX register
extern uint8_t wl_module_get_arc_cnt()
{
   uint8_t data;
   
   wl_module_read_register(OBSERVE_TX, &data, 1);
   
   return (data & OBSERVE_TX_ARC_CNT);
}

//return the value of the status register
extern uint8_t wl_module_get_status()
{
   return wl_module_get_one_byte(NOP);
}

extern uint8_t wl_module_get_rx_pipe_reading_status()
{
   uint8_t pipe;
   pipe = wl_module_get_one_byte(NOP);
   return ((pipe & 0x0E) >> 1);
}

extern uint8_t wl_module_get_one_byte(uint8_t command)
{
   uint8_t status;
   
   wl_module_CSN_lo;
   status = spi_fast_shift(command);
   wl_module_CSN_hi;
   
   return status;
   
}

extern uint8_t wl_module_get_data(uint8_t * data)
// Reads wl_module_PAYLOAD bytes into data array
{
   uint8_t status;
   wl_module_CSN_lo;                               // Pull down chip select
   status = spi_fast_shift( R_RX_PAYLOAD );            // Send cmd to read rx payload
   spi_transfer_sync(data,data,wl_module_PAYLOAD); // Read payload
   wl_module_CSN_hi;                               // Pull up chip select
   wl_module_config_register(STATUS,(1<<RX_DR));   // Reset status register
   return status;
}

void wl_module_config_register(uint8_t reg, uint8_t value)
// Clocks only one byte into the given wl-module register
{
   wl_module_CSN_lo;
   spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
   spi_fast_shift(value);
   wl_module_CSN_hi;
}

void wl_module_read_register(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the wl-module registers.
{
   wl_module_CSN_lo;
   spi_fast_shift(R_REGISTER | (REGISTER_MASK & reg));
   spi_transfer_sync(value,value,len);
   wl_module_CSN_hi;
}

void wl_module_write_register(uint8_t reg, uint8_t * value, uint8_t len)
// Writes an array of bytes into inte the wl-module registers.
{
   wl_module_CSN_lo;
   spi_fast_shift(W_REGISTER | (REGISTER_MASK & reg));
   spi_transmit_sync(value,len);
   wl_module_CSN_hi;
}


void wl_module_send(uint8_t * value, uint8_t len)
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
   //lcd_clr_line(3);
   //lcd_gotoxy(3,3);
   //lcd_putc('b');
   
   //lcd_puthex(PTX);
   //lcd_putc(' ');
   //return;
   while (PTX)
   {
      //lcd_putc('x');
   }
   
   // Wait until last paket is send
   //lcd_putc('A');
   wl_module_CE_lo;
   _delay_us(50);
   PTX = 1;                        // Set to transmitter mode
   TX_POWERUP;                     // Power up
   
   wl_module_CSN_lo;                    // Pull down chip select
   //lcd_putc('B');
   _delay_us(50);
   spi_fast_shift( FLUSH_TX );     // Write cmd to flush tx fifo
   //lcd_putc('C');
   wl_module_CSN_hi;                    // Pull up chip select
   //return;
   _delay_us(50);
   wl_module_CSN_lo;                    // Pull down chip select
   spi_fast_shift( W_TX_PAYLOAD ); // Write cmd to write payload
   spi_transmit_sync(value,len);   // Write payload
   wl_module_CSN_hi;                    // Pull up chip select
   
   wl_module_CE_hi;                     // Start transmission
   _delay_us(100);						// Gruenes Modul funktioniert nicht mit 10�s delay
   wl_module_CE_lo;
  // lcd_putc('D');
}
