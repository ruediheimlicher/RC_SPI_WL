/*
 * nRF24L01_Tutorial_Sender.c
 *
 * Created: 06.01.2012 20:15:04
 *  Author: Ernst Buchmann
 */ 

#ifndef F_CPU				//Define F_CPU if not done 
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "spi.h"
#include "wl_module.h"
#include "nRF24L01.h"

volatile uint8_t timercounter;

int main(void)
{
	uint8_t payload[wl_module_PAYLOAD];		//Array for Payload
	uint8_t maincounter =0;
	uint8_t k;
	
	wl_module_init();	//initialise nRF24L01+ Module
	_delay_ms(50);		//wait for nRF24L01+ Module
	sei();
	
	wl_module_tx_config(wl_module_TX_NR_0);		//Config Module
	
	//Timer aktivieren ATMEGA8
	#if defined(__AVR_ATmega8__)
	TCCR0 |= ( (1<<CS02) | (1<<CS00));		//Prescaler auf 1024 ATMEGA8
	TIMSK |= ( (1<<TOIE0));					//enable TOVF ATMEGA8
	#endif // __AVR_ATmega8__
	
	//Timer aktivieren ATMEGA88A
	#if defined(__AVR_ATmega88A__)
	TCCR0B |= ( (1<<CS02) | (1<<CS00));
	TIMSK0 |= ( (1<<TOIE0));
	#endif // __AVR_ATmega88A__
	
    while(1)
    {
		if (timercounter >= 30)			//30 entspricht ~1 Sekunde bei 8MHz
			{
				timercounter = 0;
			
				
				for (k=0; k<=wl_module_PAYLOAD-1; k++)
				{
					payload[k] = k;
				}
			
				payload[0] = maincounter;
				payload[1] = maincounter+1;				
			
				wl_module_send(payload,wl_module_PAYLOAD);
				
				maincounter++;
				if (maincounter >250)
				{
					maincounter = 0; 
				}
			}
    }
}

ISR(TIMER0_OVF_vect)
{
	timercounter++;
}


//Unterscheidung je nach verwendeten µC
#if defined(__AVR_ATmega8__)
ISR(INT0_vect)
#endif // __AVR_ATmega8__
#if defined(__AVR_ATmega88A__)
ISR(INT0_vect)
#endif // __AVR_ATmega88A__
#if defined(__AVR_ATmega168__)
ISR(PCINT2_vect) 
#endif // __AVR_ATmega168__  
// Interrupt handler 
{
    uint8_t status;   
    
        // Read wl_module status 
        wl_module_CSN_lo;                               // Pull down chip select
        status = spi_fast_shift(NOP);					// Read status register
        wl_module_CSN_hi;                               // Pull up chip select
		
		
		if (status & (1<<TX_DS))							// IRQ: Package has been sent
		{
			wl_module_config_register(STATUS, (1<<TX_DS));	//Clear Interrupt Bit
			PTX=0;
		}
		
		if (status & (1<<MAX_RT))							// IRQ: Package has not been sent, send again
		{
			wl_module_config_register(STATUS, (1<<MAX_RT));	// Clear Interrupt Bit
			wl_module_CE_hi;								// Start transmission
			_delay_us(10);								
			wl_module_CE_lo;
		}
		
		if (status & (1<<TX_FULL))							//TX_FIFO Full <-- this is not an IRQ
		{
			wl_module_CSN_lo;                               // Pull down chip select
			spi_fast_shift(FLUSH_TX);						// Flush TX-FIFO
			wl_module_CSN_hi;                               // Pull up chip select
		}
		
}