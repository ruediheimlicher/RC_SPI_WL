/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * General stdiodemo defines
 *
 * $Id: defines.h,v 1.1.2.2 2006/10/08 21:51:14 joerg_wunsch Exp $
 */

/* CPU frequency */
//#define F_CPU 8000000UL

#define LOOPLED_PORT	PORTB
#define LOOPLED_DDR	DDRB
#define LOOPLED_PIN	7

#define LOADDDR      DDRB
#define LOADPORT     PORTB
#define LOADPIN      0

#define ADCPORT      PORTC
#define ADCDDR       DDRC

#define PTPORT   PORTD
#define PTDDR   DDRD
#define PT_LOAD_PIN  7
#define PT_LO  PTPORT &= ~(1<<PT_LOAD_PIN)
#define PT_HI  PTPORT |= (1<<PT_LOAD_PIN)


//Oszi
#define OSZIPORT           PORTD
#define OSZIDDR            DDRD
#define PULSA              4
#define OSZIA_LO OSZIPORT &= ~(1<<PULSA)
#define OSZIA_HI OSZIPORT |= (1<<PULSA)
#define OSZIA_TOGG OSZIPORT ^= (1<<PULSA)

#define TEST_PIN           5

#define SPI_DDR			DDRB						// DDR fuer SPI
#define SPI_PORT        PORTB						// Port fuer SPI
#define SPI_PORTPIN     PINB						// Port-Pin fuer SPI

#define SPI_MISO           PB4
#define SPI_MOSI           PB3
#define SPI_CLK            PB5
#define SPI_SS             PB2

#define OSZIA_LO OSZIPORT &= ~(1<<PULSA)
#define OSZIA_HI OSZIPORT |= (1<<PULSA)
#define OSZIA_TOGG OSZIPORT ^= (1<<PULSA)

// Port fuer INT1
#define INTERRUPT_PORT   PORTD
#define INTERRUPT_DDR   DDRD

#define INT0_PIN        PD2
#define INT1_PIN        PD3

// Port fuer Chip select
#define SPI_WL_PORT     PORTC
#define SPI_WL_DDR      DDRC
#define SPI_WL_CE       PC0
#define SPI_WL_CSN      PC1

#define WL_ISR_RECV  7
#define WL_SEND_REQUEST 6
#define WL_NEXT_REQUEST 5

#define WL_DATA_PENDENT    4
#define WL_DATA_WAIT   3



// PWM-detect

#define PWMHI  1 // Bit fuer HI
#define PWM_DETECT_PORT    PORTB
#define PWM_DETECT_DDR     DDRB
#define PWM_DETECT_PIN     PINB
#define PWM_DETECT         7 // Pin fuer detect
#define PWM_DETECT_BIT     1 // bit fuer HI
