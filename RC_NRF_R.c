//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <avr/eeprom.h>
//#define F_CPU 4000000UL  // 4 MHz
#include <avr/delay.h>
#include "lcd.c"
#include "adc.c"

//#include "onewire.c"
//#include "ds18x20.c"
//#include "crc8.c"

//#include "conio.h"
#include "defines.h"
#include "wireless.c"
#include "wl_module.h" 
#include "nRF24L01.h"
//#include "RF24.h"

#define VREF 256
#define SPI_BUFSIZE  4


// von RC_PWM
#define SERVOMAX  2000
#define SERVOMIN  1000
#define TIMER0_STARTWERT   0x40

#define L_PORT 0 // index in servoportarray
#define LX_PIN  1
#define LX_PIN_HI  PORTB |= (1<<LX_PIN)
#define LX_PIN_LO  PORTB &= ~(1<<LX_PIN)


#define LY_PIN  2
#define LY_PIN_HI  PORTB |= (1<<LY_PIN)
#define LY_PIN_LO  PORTB &= ~(1<<LY_PIN)

#define R_PORT 1
#define RX_PIN  2
#define RX_PIN_HI  PORTC |= (1<<RX_PIN)
#define RX_PIN_LO  PORTC &= ~(1<<RX_PIN)

#define RY_PIN  3
#define RY_PIN_HI  PORTC |= (1<<RY_PIN)
#define RY_PIN_LO  PORTC &= ~(1<<RY_PIN)





#define LP_PIN  16
#define RP_PIN  17

#define LT_PIN   7
#define RT_PIN  4


static volatile uint8_t *servoportarray[] = { &PORTB, &PORTC, &PORTD };


volatile uint8_t servopinarray[SPI_BUFSIZE];

volatile uint8_t servowert = 0;
volatile uint8_t lastservowert = 0;

            
volatile uint8_t           timer0startwert=TIMER0_STARTWERT;
volatile uint16_t          Servo_ArrayInt[SPI_BUFSIZE] = {}; // unsigned Int




uint8_t impulscounter = 0;
uint16_t defaultwert = 800;

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;

uint16_t pwmpos=0;

#define PROGRAMM_DS	0
#define GRUPPE_DS		0xC0
//#define GRUPPE_DS	0xB0

#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4
#define SCLPIN		5

#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB



#define TASTE1		19
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		223
#define TASTE0		236
#define TASTER		248
#define TASTATURPORT PORTC
#define TASTATURPIN		3

#define MANUELL_PORT		PORTD
#define MANUELL_DDR		DDRD
#define MANUELL_PIN		PIND

#define MANUELL			7	// Bit 7 von Status 
#define MANUELLPIN		6	// Pin 6 von PORT D fuer Anzeige Manuell
#define MANUELLNEU		7	// Pin 7 von Status. Gesetzt wenn neue Schalterposition eingestellt
#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s


#define FOSC 1000000    /* oscillator-frequency in Hz */
#define BAUD 57600  //valid values:9600, 19200, 57600 kbits


volatile uint8_t					Programmstatus=0x00;
uint8_t Tastenwert=0;
uint8_t TastaturCount=0;
volatile uint16_t					Manuellcounter=0; // Countr fuer Timeout	
uint16_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;



volatile uint16_t	spiwaitcounter=0;


volatile uint32_t	loadcounter=0; // schaltet load fuer PowerBanl-Reset ein





//PWM-detector
volatile uint32_t	pwmhi=0; // dauer des hi
volatile uint32_t	pwmpuls=0; // periodendauer

volatile uint16_t	pwmhicounter=0;
volatile uint16_t	pwmpulscounter=0;

volatile uint8_t pwmstatus=0;



volatile uint8_t spi_status=0;

//Variablen WL
// MARK: WL Defs
volatile uint8_t wl_status=0;
volatile uint8_t PTX=0;
volatile uint8_t int0counter=0;
volatile uint8_t sendcounter=0;
volatile uint16_t	wlwaitcounter=0;
volatile uint16_t	firstruncounter=0x0F;
volatile uint8_t wl_spi_status;
char itoabuffer[20];
volatile uint8_t wl_data[wl_module_PAYLOAD] = {};



volatile uint8_t pipenummer = 1;

//volatile char text[] = {'*','M','a','s','t','e','r','*'};
char* text = "* Master *";

// ACD https://www.avrprogrammers.com/howto/attiny-comparator
// ACD https://www.avrprogrammers.com/howto/attiny-comparator
#define COMP_PORT PORTB
#define COMP_DDR DDRB

// Pins fuer Drive der RC
#define COMP_DRIVE_PIN_A  1
#define COMP_DRIVE_PIN_B  2

#define COMP_ADC_PORT PORTC
#define COMP_ADC_DDR DDRC

#define COMP_ADC_PIN_A  4
#define COMP_ADC_PIN_B  5

#define COMP_AIN_PORT   PORTD
#define COMP_AIN_DDR    DDRD
#define COMP_AIN0       6
#define COMP_AIN1       7


#define MULTIPLEX 1

volatile uint16_t captured_value;
volatile uint8_t captured;
volatile uint8_t overflow=0;
volatile uint8_t captcounter=0;
volatile uint16_t mittelwertA[4];
volatile uint16_t mittelwertB[4];
volatile uint8_t mposA=0;
volatile uint8_t mposB=0;
volatile uint8_t adckanal=0;

// end ACD

#pragma mark RC

struct PacketData 
{
   uint8_t lxAxisValue;
   uint8_t lyAxisValue;
   uint8_t rxAxisValue;
   uint8_t ryAxisValue;
   uint8_t lPotValue;  
   uint8_t rPotValue;    
   uint8_t switch1Value;
   uint8_t switch2Value;
   uint8_t switch3Value;
   uint8_t switch4Value;  
};

struct PacketData receiverData;


#define SIGNAL_TIMEOUT 500  // This is signal timeout in milli seconds. We will reset the data if no signal

const uint64_t pipeIn = 0xF9E8F0F0E1LL;
unsigned long lastRecvTime = 0;

volatile uint8_t wl_spi_status;



void delay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
//lcd_gotoxy(0,1);
//lcd_putint(Tastaturwert);
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}

void SPI_Master_init (void)
{
   
   SPCR |= (1<<MSTR);// Set as Master
   
   //  SPCR0 |= (1<<CPOL0)|(1<<CPHA0);
   
   /*
    SPI2X 	SPR1 	SPR0     SCK Frequency
    0       0        0     fosc/4
    0       0        1     fosc/16
    0       1        0     fosc/64
    0       1        1     fosc/128
    1       0        0     fosc/2
    1       0        1     fosc/8
    1       1        0     fosc/32
    1       1        1     fosc/64
    */
   
   //SPCR |= (1<<SPR0);               // div 16 SPI2X: div 8
   SPCR |= (1<<SPR1);               // div 64 SPI2X: div 32
   //SPCR |= (1<<SPR1) | (1<<SPR0);   // div 128 SPI2X: div 64
   //SPCR |= (1<<SPI2X0);
   
   SPCR |= (1<<SPE); // Enable SPI
   spi_status = SPSR;								//Status loeschen
   wl_spi_status = 0;
}

void deviceinit(void)
{
//	MANUELL_DDR |= (1<<MANUELLPIN);		//Pin 5 von PORT D als Ausgang fuer Manuell
	//MANUELL_PORT &= ~(1<<MANUELLPIN);

   LOADDDR |= (1<<LOADPIN);
   LOADPORT |= (1<<LOADPIN);
   
   PWM_DETECT_DDR &= ~(1<<PWM_DETECT);
   PWM_DETECT_PORT |= (1<<PWM_DETECT);
   
   LOOPLED_DDR |= (1<<LOOPLED_PIN);
	//PORTD &= ~(1<<CONTROL_B);
	//PORTD &= ~(1<<CONTROL_A);
   OSZIDDR |= (1<<PULSA);	//Pin 0 von  als Ausgang fuer OSZI
   OSZIPORT |= (1<<PULSA);		// HI
   
   ADCDDR &= ~(1<<PORTC2);
   ADCPORT &= ~(1<<PORTC2);
   ADCDDR &= ~(1<<PORTC3);
   ADCPORT &= ~(1<<PORTC3);
   ADCDDR &= ~(1<<PORTC4);
   ADCPORT &= ~(1<<PORTC4);
   ADCDDR &= ~(1<<PORTC5);
   ADCPORT &= ~(1<<PORTC5);

   DDRB  |= (1<<PB1);
   DDRB  |= (1<<PB2);

   
   PTDDR |= (1<<PT_LOAD_PIN); // Pin fuer Impuls-load von pT1000
   PTPORT |= (1<<PT_LOAD_PIN);// hi
	
 //  DDRB |= (1<<PORTB0);	//OC1A: Bit 1 von PORT B als Ausgang fuer PWM
//   PORTB |= (1<<PORTB0);	//LO


	DDRB |= (1<<PORTB1);	//OC1A: Bit 1 von PORT B als Ausgang fuer PWM
	PORTB &= ~(1<<PORTB1);	//LO
	

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);    //Pin  als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin  als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin  als Ausgang fuer LCD

#if defined(__AVR_ATmega8__)
   // Initialize external interrupt 0 (PD2)
   MCUCR = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));	// Set external interupt on falling edge
   GICR  = ((1<<INT1)|(0<<INT0));							// Activate INT1
   
   
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
   EIMSK  = ((1<<INT0));
   
#endif // __AVR_ATmega32U4__
   
}

char SPI_get_put_char(int cData)
{
   
   //Putchar -- Master
   /* Start transmission */
   SPDR = cData;
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)))
      ;
   /* Return data register */
   return SPDR;
}

void SPI_Init(void)
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   /*
   SPI_DDR |= (1<<2)|(1<<3)|(1<<5);
   SPI_DDR &= ~(1<<4);
   SPCR |= (1<<MSTR);
   SPCR |= (1<<SPR0)|(1<<SPR1);
   SPCR |= (1<<SPE);
   */
   
   
  
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   SPI_PORT |= (1<<SPI_MISO);
   
   
   SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_CLK)|(1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS);
   
   // Enable SPI, Master, set clock rate fck/16 
   SPCR =   (1<<SPE)|
            (1<<MSTR)|
   (1<<SPR0);//|
            //(1<<SPR1);
   
   /*
    Slave init
    // Set MISO output, all others input
    DDR_SPI = (1<<DD_MISO);
    // Enable SPI 
    SPCR = (1<<SPE);
    */
   
}
/*
void timer1_comp(void)
{
   // Set pin for driving resistor low.
   COMP_DDR |= (1<<COMP_DRIVE_PIN_A);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_A);
   COMP_DDR |= (1<<COMP_DRIVE_PIN_B);
   COMP_PORT &= ~(1<<COMP_DRIVE_PIN_B);
   
   // Disable the digital input buffers.
   //   DIDR = (1<<AIN1D) | (1<<AIN0D);
   if (MULTIPLEX)
   {
      // ADC-Eingaenge fuer Capt
      COMP_ADC_DDR &= ~(1<<COMP_ADC_PIN_A);
      COMP_ADC_PORT &= ~(1<<COMP_ADC_PIN_A);
      
      COMP_ADC_DDR &= ~(1<<COMP_ADC_PIN_B);
      COMP_ADC_PORT &= ~(1<<COMP_ADC_PIN_B);
      
      // AIN0, AIN1 Eingang
      COMP_AIN_DDR &= ~(1<<COMP_AIN0);
      COMP_AIN_DDR &= ~(1<<COMP_AIN1);
      
      
      SFIOR |= (1<<ACME);
      //ADMUX = 3;
   }
   
   
   //ADCSRA =0;//| = (1<<ADEN);                    // disable ADC if necessary
   ACSR =   (1<<ACIC) | (1<<ACIS1) | (1<<ACIS0);   // Comparator enabled, no bandgap, input capture.
   // Timer...
   TCCR1A = 0;
   TCCR1B =   (1<<CS10);                        // F_CPU / 1
   //TCCR1B =  (1<<ICES1);                      // Input capture on rising edge
   TCNT1 = 0;
   TIMSK1 |= (1<<TOIE1) | (1<<TICIE1);           // Timer interrupts on capture and overflow.
   sei();
}
*/

#pragma mark timer1
// Timer1 Servo

void timer1(void)
{
   
   // https://www.mikrocontroller.net/topic/83609
   
   int c=0;
     
   //TCCR1B |= (1<<CS11); // f/8
  TCCR1B |= (1<<CS11); // f
   TCNT1  = 0;                                          // reset Timer
   
                              // Impulsdauer
   OCR1B  = 0x500;            // Impulsdauer des Kanalimpulses
   
   TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
 //  TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
   OCR1A  = Servo_ArrayInt[(impulscounter & 0x07)]; // 
   
   
}

ISR(TIMER1_OVF_vect)
{
   PORTB ^= (1<<PB2);
   
}

// MARK: TIMER1_COMPA_vect
ISR(TIMER1_COMPA_vect) // ca 4 us
{
   OSZIA_TOGG;
   // 
   loadcounter++;
   
   if (impulscounter < SPI_BUFSIZE)
   {
      // Impulsdauer lesen
      OCR1A  = Servo_ArrayInt[(impulscounter)]; 

      // servowert setzen
      servowert = (uint8_t)servopinarray[impulscounter];// Pin & PORT>>4
      //uint8_t kanalport = (servowert & 0xF0)>>4;
      //uint8_t kanalpin = servowert & 0x0F;
      
      *servoportarray[(servowert & 0xF0)>>4] |= (1<<(servowert & 0x0F));
      
      if(impulscounter == 0) // Start Impulspaket. ersten impuls setzen
      {
         
      }
      else // vorherigen Impuls beenden
      {
         *servoportarray[(lastservowert & 0xF0)>>4] &= ~(1<<(lastservowert & 0x0F));
      }
      lastservowert = servowert;
      impulscounter++;
   }
   else 
   {
      // letzten Impuls beenden
      *servoportarray[(lastservowert & 0xF0)>>4] &= ~(1<<(lastservowert & 0x0F));
      impulscounter =0; // restart from beginning
      OCR1A = 15000;
   }
   
    
   TCNT1  = 0;
   if(OCR1A < 0xFFF0)
   {
      //OCR1A += 100;
   }
   else 
   {
      //OCR1A = 400;
   }
  

}


ISR(TIMER1_COMPB_vect) // ca 4 us
{
   //PORTB ^= (1<<PB2);
   // 
   loadcounter++;
   TCNT1  = 0;
   
   //OSZIA_HI;
 

}



/*
#pragma mark INT0 WL
ISR(INT0_vect)
{
   wl_spi_status |= (1<<WL_ISR_RECV);
   
}
*/


void timer2(void)
{
   DDRB |= (1<<PB2);
   
   TCCR2A = (1<<WGM21); // Wave Form Generation Mode 2: CTC, OC2A disconnected
   TCCR2B = (1<<CS20)  ; // prescaler = 256
   TIMSK2 = (1<<OCIE2A); // interrupt when Compare Match with OCR2A
   OCR2A = 83 ; // 10kHz

   
}

ISR(TIMER2_OVF_vect)
{
   //PORTB ^= (1<<PB2);
   
}


ISR(TIMER2_COMPA_vect) // ca 4 us
{
   //OSZIA_LO;
   PORTB ^= (1<<PB2);
   // 
   loadcounter++;

   
   //OSZIA_HI;
   // handle interrupt
}

#pragma mark INT1 WL
ISR(INT1_vect)
{
   
   wl_spi_status |= (1<<WL_ISR_RECV);
}

ISR (SPI_STC_vect)
{
  // SPDR = data;
}


/*
 // https://www.avrfreaks.net/forum/how-can-i-make-array-ports
 static volatile uint8_t *myports[] = { &PORTD, &PORTB, &PORTC };

 int main() {
     uint8_t i,j;
     for (i=0; i < 3; i++) {
       *(myports[i]) |= 0x10;  // set this bit in each port
     }
 }
 
 
 */

// MARK: main

int main (void)
{
   /* INITIALIZE */
   //	LCD_DDR |=(1<<LCD_RSDS_PIN);
   //	LCD_DDR |=(1<<LCD_ENABLE_PIN);
   //	LCD_DDR |=(1<<LCD_CLOCK_PIN);
   
   uint8_t pos = 0;
   for (pos=0;pos<8;pos++)
   {
      Servo_ArrayInt[pos] = defaultwert + pos*100;
   }
   Servo_ArrayInt[0] = 400;
   Servo_ArrayInt[1] = 600;
   Servo_ArrayInt[2] = 800;
   Servo_ArrayInt[3] = 1000;

   /*
   Servo_ArrayInt[4] = 400;
   Servo_ArrayInt[5] = 800;
   Servo_ArrayInt[6] = 1200;
   Servo_ArrayInt[7] = 1600;
   */
   
   servopinarray[0] = LX_PIN + (L_PORT<<4); // linker joystick
   servopinarray[1] = LY_PIN + (L_PORT<<4);
   servopinarray[2] = RX_PIN + (R_PORT<<4); // rechter joystick
   servopinarray[3] = RY_PIN + (R_PORT<<4);

 
   
   
   deviceinit();
   delay_ms(100);
//   SPI_Init();
//   SPI_Master_init();
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   lcd_puts("Guten Tag\0");
   delay_ms(200);
   
   lcd_cls();
   lcd_gotoxy(0,0);
   lcd_puts("READY\0");
   
   // DS1820 init-stuff begin
   // DS1820 init-stuff end
   
   volatile char incoming=0;
   size_t poscounter=0;
   
   // MARK: WL main
   uint8_t payload[wl_module_PAYLOAD];
   uint8_t maincounter =0;
   //Array for Payload
   
   wl_module_init();
   _delay_ms(50);
   
   sei();
   
   wl_module_rx_config();
   
   delay_ms(50);
   wl_module_CE_hi;
   //
   lcd_clr_line(0);
   
    // timer1_comp();
   initADC(2);
   
   uint8_t delaycount=10;
#pragma mark while
   uint8_t readstatus = wl_module_get_data((void*)&wl_data);
   //lcd_puts(" los");
   
   
   uint8_t eevar=13;
  timer1();
  // timer2();
   sei();
   while (1)
   {
      //  PORTC |= (1<<0);
      loopCount0 ++;
      //_delay_ms(2);
      //LOOPLED_PORT ^= (1<<LOOPLED_PIN);
      //incoming = SPDR;
      
      if (wl_spi_status & (1<<WL_ISR_RECV)) // in ISR gesetzt, etwas ist angekommen, Master fragt nach Daten
      {
         lcd_gotoxy(19,0);
         lcd_putc('$');
         if (int0counter < 0x2F)
         {
            int0counter++;
         }
         else
         {
            int0counter=0;
         }
         
         wl_spi_status &= ~(1<<WL_ISR_RECV);
         
         /*
         lcd_gotoxy(6,0);
         lcd_putc('i');
         lcd_puthex(int0counter);
         */
         
         // MARK: WL Loop
         /*
          lcd_gotoxy(0,1);
          lcd_puthex(wl_status & (1<<RX_DR));
          lcd_puthex(wl_status & (1<<TX_DS));
          lcd_puthex(wl_status & (1<<MAX_RT));
          lcd_puthex(wl_status & (1<<TX_FULL));
          */
         wl_status = wl_module_get_status();
         delay_ms(3);
         
         //lcd_gotoxy(18,0);
         

         //lcd_puthex(wl_status);
         
         
         pipenummer = wl_module_get_rx_pipe();
         
         delay_ms(3);
         
         if (pipenummer == WL_PIPE) // Request ist fuer uns, Data schicken
         {
 //           lcd_gotoxy(14,0);
 //          lcd_puts("p ok");
            //lcd_gotoxy(0,0);
//            delay_ms(2);
            //lcd_puts("          ");
            if (wl_status & (1<<RX_DR)) // IRQ: Package has been received
            {
               //OSZIA_LO;
               //lcd_gotoxy(16,2);
               //lcd_puts("RX+");
               //OSZIA_HI;
               
               uint8_t rec = wl_module_get_rx_pw(0);        //gets the RX payload width on the pipe
               
               //lcd_gotoxy(0,3);
               //lcd_puthex(rec);
               //lcd_putc(' ');
  //             delay_ms(3);
               uint8_t readstatus = wl_module_get_data((void*)&wl_data); // Reads wl_module_PAYLOAD bytes into data array
 //              delay_ms(3);
               wl_module_config_register(STATUS, (1<<RX_DR)); //Clear Interrupt Bit
  //             delay_ms(3);
               uint8_t i;
               
               //lcd_gotoxy(0,0);
               //lcd_puts("rs:");
               //lcd_puthex(readstatus); //
               /*
                // pi schreiben
                lcd_putc(' ');
                lcd_putint1(wl_data[0]);
                lcd_putc('.');
                for (i=2; i<4; i++)
                {
                lcd_putint1(wl_data[i]);
                }
                */
               //lcd_putc(' ');
               // Kontrolle: Data vom teensy
               uint16_t temperatur = (wl_data[11]<<8);
               temperatur |= wl_data[10];
               //lcd_putint12(temperatur);
               
               temperatur /=4; // *256/1024, unkalibriert
               //    lcd_putint2(temperatur/10);
               //    lcd_putc('.');
               //   lcd_putint1(temperatur%10);
               
               //lcd_put_tempbis99(temperatur);
               
               //            lcd_gotoxy(18,3);
               //            lcd_puthex(wl_data[9]);
               
               pwmpos = temperatur;
               OCR1A = temperatur;
               //OSZIA_HI;
               
               wl_spi_status |= (1<<WL_SEND_REQUEST);
               
               if (wl_spi_status & (1<<WL_SEND_REQUEST)) // senden starten
               {
                  wl_spi_status &= ~(1<<WL_SEND_REQUEST);
                  
                  // MARK: WL send
                  wl_module_tx_config(WL_PIPE); // neue Daten senden an Master auf pipe WL_PIPE
                  delay_ms(1);

                  //lcd_putc('b');
                  
                  
                  wl_module_send(payload,wl_module_PAYLOAD);
                  //lcd_putc('c');
                  
                  uint8_t tx_status = wl_module_get_status();
                  //delay_ms(3);
                  lcd_gotoxy(0,0);
                  lcd_putc('s');
                  //lcd_puthex(tx_status);
                  //lcd_putc(' ');
                  lcd_puthex(sendcounter);
                  
                  maincounter++;
                  PTX=0;
                  
                  wl_module_rx_config(); // empfangen wieder einstellen
                  //delay_ms(3);
               } // if
               
               
               
            }
            else
            {
               lcd_gotoxy(14,0);
               lcd_puts("p ex");
               
            }
            
         } // if pipenummer
         else
         {
            // lcd_puts("--");
            
         }
         
         
         
         if (wl_status & (1<<TX_DS)) // IRQ: Package has been sent
         {
            //OSZIA_LO; // 50 ms mit Anzeige, 140us ohne Anzeige
            sendcounter++;
            lcd_gotoxy(16,1);
            lcd_puts("TX+");
            wl_module_config_register(STATUS, (1<<TX_DS)); //Clear Interrupt Bit
            PTX=0;
            //OSZIA_HI;
         }
         else
         {
            //    lcd_gotoxy(16,1);
            //    lcd_puts("***");
         }
         
         
         if (wl_status & (1<<MAX_RT)) // IRQ: Package has not been sent, send again
         {
            lcd_gotoxy(16,1);
            lcd_puts("RT");
            
            wl_module_config_register(STATUS, (1<<MAX_RT)); // Clear Interrupt Bit
         }
         
         
         
      } // end ISR abarbeiten
      
      /*
       if (wl_spi_status & (1<<WL_SEND_REQUEST)) // senden starten
       {
       wl_spi_status &= ~(1<<WL_SEND_REQUEST);
       
       wl_module_tx_config(0); // neue Daten senden an device 0 (Master)
       
       //lcd_putc('b');
       
       wl_module_send(payload,wl_module_PAYLOAD);
       //lcd_putc('c');
       
       uint8_t tx_status = wl_module_get_status();
       
       lcd_gotoxy(0,1);
       //lcd_putc(' ');
       lcd_puthex(tx_status);
       lcd_putc(' ');
       lcd_puthex(sendcounter);
       
       maincounter++;
       PTX=0;
       
       wl_module_rx_config(); // empfangen wieder einstellen
       
       } // if
       */
      
      
      
      if (loopCount0 >=0xFE)
      {
         
         loopCount1++;
         
         if (loopCount1 >0xAF)
         {
            //OSZIA_TOGG;
            //LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         }
         
         if ((loopCount1 >0x02AF) )//&& (!(Programmstatus & (1<<MANUELL))))
         {
             /* 
            if(OCR1A < 1000)
            {
               OCR1A += 10;
            }
            else 
            {
               OCR1A = 400;
            }
*/
            lcd_gotoxy(0,0);
            //lcd_puts(" OCR1A ");
            //lcd_putint12(OCR1A);
            LOOPLED_PORT ^= (1<<LOOPLED_PIN);
            
            for (uint8_t pos = 0;pos < 4;pos++)
            {
               uint8_t tempservowert = (uint8_t)servopinarray[pos];// Pin & PORT>>4
               lcd_gotoxy(pos * 3,3);
               lcd_puthex(tempservowert);
               lcd_putc(' ');
               if(pos > 1)
               {
                //  *servoportarray[(tempservowert & 0xF0)>>4] |= (1<<tempservowert & 0x0F);
                //  _delay_ms(4);
                 // *servoportarray[(tempservowert & 0xF0)>>4] &= ~(1<<tempservowert & 0x0F);
               }
            }
            
            lcd_gotoxy(12,0);
            lcd_putc('p');
            lcd_puthex(pipenummer);
                     
/*
            lcd_gotoxy(0,2);
            
            uint8_t tempservowert = (uint8_t)servopinarray[2];
            lcd_puthex(tempservowert);
            lcd_putc(' ');
            lcd_puthex((tempservowert & 0xF0)>>4);
            lcd_putc(' ');
            lcd_puthex(tempservowert & 0x0F);
            lcd_putc(' ');
 */
            // Anzeige PWM
            /*
             lcd_gotoxy(0,0);
             lcd_puts("h ");
             lcd_putint16(pwmhi);
             //lcd_gotoxy(8,0);
             lcd_puts(" p ");
             lcd_putint16(pwmpuls);
             //OSZIA_LO;
             uint16_t pwm = pwmhi*100/pwmpuls; // 2us
             //OSZIA_HI;
             lcd_gotoxy(0,1);
             
             lcd_puts("m ");
             lcd_putint12(pwm);
             */
            
            
            // MARK: ADC Loop
            // Batterie lesen
            uint16_t adc3wert = readKanal(3);
            
            VREF_Quelle = ADC_REF_INTERNAL;
            uint8_t i=0;
            uint32_t temperatur2 = 0;
             
            
            
            
            //lcd_gotoxy(0,1);
            //_delay_us(300);
            
            
            // MARK: KTY
            // KTY
            
            
                
            uint8_t k;
            for (k=0; k<wl_module_PAYLOAD; k++)
            {
               //payload[k] = wl_module_PAYLOAD-k;
            }
            // Euler 2,71828182
            payload[0] = maincounter;
            payload[1] = 0;
            payload[2] = 2;
            payload[3] = 0;
            payload[4] = 7;
            payload[5] = 1;
            payload[6] = 8;
            payload[7] = 2;
            
            payload[9] = WL_PIPE;
            //            payload[10] = adc2wert & 0x00FF;
            //           payload[11] = (adc2wert & 0xFF00)>>8;
            payload[10] = temperatur2 & 0x00FF;
            payload[11] = (temperatur2 & 0xFF00)>>8;
            
            //            payload[12] = adc3wert & 0x00FF;
            //            payload[13] = (adc3wert & 0xFF00)>>8;
            //            payload[12] = (ktywert/KTY_FAKTOR) & 0x00FF;
            //            payload[13] = ((ktywert/KTY_FAKTOR) & 0xFF00)>>8;
            payload[12] = 0; //(ptwert) & 0x00FF;
            payload[13] = 0; //((ptwert) & 0xFF00)>>8;
            payload[14] = maincounter; // fortlaufender Wert, kontrolle ob hanging
            
            /*
             if (wl_spi_status & (1<<WL_SEND_REQUEST)) // senden starten
             {
             wl_spi_status &= ~(1<<WL_SEND_REQUEST);
             
             wl_module_tx_config(0); // neue Daten senden an device 0 (Master)
             
             //lcd_putc('b');
             
             wl_module_send(payload,wl_module_PAYLOAD);
             //lcd_putc('c');
             
             uint8_t tx_status = wl_module_get_status();
             
             lcd_gotoxy(0,1);
             //lcd_putc(' ');
             lcd_puthex(tx_status);
             lcd_putc(' ');
             lcd_puthex(sendcounter);
             
             maincounter++;
             PTX=0;
             
             wl_module_rx_config(); // empfangen wieder einstellen
             
             } // if
             */
            
            //lcd_gotoxy(0,3);
            
            //lcd_puthex(maincounter);
            
            if (maincounter >250)
               
            {
               maincounter = 0;
            }
            
            loopCount2++;
            
            //lcd_gotoxy(18,0);
            //lcd_puthex(loopCount2);
            
            loopCount1=0;
            //wl_status=0;
              
            // DS1820 loop-stuff end
            
            
            //lcd_putint(gTempdata[1]);
            //lcd_putint(gTempdata[2]);
            //delay_ms(1000);
         }
         
         loopCount0 =0;
      }
      
      // ***
      // ***
      
      /*
       if (!(PINB & (1<<PB0))) // Taste 0
       {
       lcd_gotoxy(10,1);
       lcd_puts("P0 Down\0");
       lcd_puthex(TastenStatus);
       
       if (! (TastenStatus & (1<<PB0))) //Taste 0 war nicht nicht gedrueckt
       {
       lcd_gotoxy(10,1);
       lcd_puts("P0 neu \0");
       TastenStatus |= (1<<PB0);
       delay_ms(1000);
       Tastencount=0;
       //lcd_gotoxy(3,1);
       //lcd_puts("P0 \0");
       //lcd_putint(Servoimpulsdauer);
       //delay_ms(800);
       SPDR = 'x';
       while(!(SPSR & (1<<SPIF)) && spiwaitcounter<0xFFF)
       {
       spiwaitcounter++;
       }
       spiwaitcounter=0;
       delay_ms(1000);
       //lcd_gotoxy(10,1);
       //lcd_puts("       \0");
       lcd_gotoxy(19,1);
       lcd_putc('+');
       }
       else
       {
       lcd_gotoxy(19,1);
       lcd_putc('$');
       //lcd_puts("*         *\0");
       
       Tastencount ++;
       if (Tastencount >= Tastenprellen)
       {
       Tastencount=0;
       TastenStatus &= ~(1<<PB0);
       lcd_gotoxy(10,1);
       lcd_puts("*         *\0");
       
       
       
       }
       }//	else
       
       } // Taste 0
       */
      
#pragma mark Tastatur
      /* ******************** */
      //		initADC(TASTATURPIN);
      //		Tastenwert=(readKanal(TASTATURPIN)>>2);
      
      //		lcd_gotoxy(3,1);
      //		lcd_putint(Tastenwert);
      //		Tastenwert=0;
      if (Tastenwert>5)
      {
         /*
          0:											1	2	3
          1:											4	5	6
          2:											7	8	9
          3:											x	0	y
          4: Schalterpos -
          5: Manuell ein
          6: Schalterpos +
          7:
          8:
          9:
          
          12: Manuell aus
          */
         
         TastaturCount++;
         if (TastaturCount>=200)
         {
            
            
            //lcd_gotoxy(17,1);
            //lcd_puts("T:  \0");
            //lcd_putint(Tastenwert);
            
            uint8_t Taste=Tastenwahl(Tastenwert);
            //Taste=0;
            //lcd_gotoxy(19,1);
            //lcd_putint1(Taste);
            //delay_ms(600);
            // lcd_clr_line(1);
            
            
            TastaturCount=0;
            Tastenwert=0x00;
            uint8_t i=0;
            uint8_t pos=0;
            //				lcd_gotoxy(18,1);
            //				lcd_putint2(Taste);
            continue;
            switch (Taste)
            {
               case 0:// Schalter auf Null-Position
               {
                  if (Programmstatus & (1<<MANUELL))
                  {
                     Manuellcounter=0;
                     Programmstatus |= (1<<MANUELLNEU);
                     /*
                      lcd_gotoxy(13,0);
                      lcd_puts("S\0");
                      lcd_gotoxy(19,0);
                      lcd_putint1(Schalterposition); // Schalterstellung
                      lcd_gotoxy(0,1);
                      lcd_puts("SI:\0");
                      lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
                      lcd_gotoxy(5,0);
                      lcd_puts("SP\0");
                      lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
                      */
                  }
                  
               }break;
                  
               case 1:	//
               {
                  if (Programmstatus & (1<<MANUELL))
                  {
                     uint8_t i=0;
                     lcd_gotoxy(0,0);
                     lcd_puts("Sens\0");
                     lcd_putc('1');
                     lcd_putc(' ');
                     /*
                      for (i=0;i<OW_ROMCODE_SIZE;i++)
                      {
                      lcd_puthex(gSensorIDs[0][i]);
                      if (i==3)
                      {
                      lcd_gotoxy(0,1);
                      }
                      lcd_putc(' ');
                      }
                      */
                     Manuellcounter=0;
                     
                  }
               }break;
                  
               case 2://
               {
                  
                  if (Programmstatus & (1<<MANUELL))
                  {
                     /*
                      uint8_t i=0;
                      lcd_gotoxy(0,0);
                      lcd_puts("Sens\0");
                      lcd_putc('1');
                      lcd_putc(' ');
                      for (i=0;i<OW_ROMCODE_SIZE;i++)
                      {
                      lcd_puthex(gSensorIDs[1][i]);
                      if (i==3)
                      {
                      lcd_gotoxy(0,1);
                      }
                      lcd_putc(' ');
                      }
                      */
                     Manuellcounter=0;
                     
                     
                  }
                  
               }break;
                  
               case 3: //	Uhr aus
               {
                  if (Programmstatus & (1<<MANUELL))
                  {
                     /*
                      uint8_t i=0;
                      lcd_gotoxy(0,0);
                      lcd_puts("Sens\0");
                      lcd_putc('1');
                      lcd_putc(' ');
                      for (i=0;i<OW_ROMCODE_SIZE;i++)
                      {
                      lcd_puthex(gSensorIDs[2][i]);
                      if (i==3)
                      {
                      lcd_gotoxy(0,1);
                      }
                      lcd_putc(' ');
                      }
                      */
                     Manuellcounter=0;
                     
                     
                  }
               }break;
                  
               case 4://
               { 
                  //DS18X20_read_scratchpad(&gSensorIDs[0][0], gScratchPad );
                  /*
                   uint8_t i=0;
                   lcd_gotoxy(0,0);
                   lcd_puts("Sens\0");
                   lcd_putc('0');
                   lcd_putc(' ');
                   for (i=0;i<OW_ROMCODE_SIZE;i++)
                   {
                   lcd_puthex(gScratchPad[i]);
                   if (i==3)
                   {
                   lcd_gotoxy(0,1);
                   }
                   lcd_putc(' ');
                   }
                   */
               }break;
                  
               case 5://
               { 
                  Programmstatus |= (1<<MANUELL);	// MANUELL ON
                  Manuellcounter=0;
                  MANUELL_PORT |= (1<<MANUELLPIN);
                  Programmstatus |= (1<<MANUELLNEU);
                  lcd_clr_line(1);
                  /*
                   lcd_gotoxy(13,0);
                   lcd_puts("S\0");
                   lcd_putint1(Schalterposition); // Schalterstellung
                   lcd_gotoxy(0,1);
                   lcd_puts("SP:\0");
                   lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
                   lcd_gotoxy(5,0);
                   lcd_puts("SI\0");
                   lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
                   */
               }break;
                  
               case 6://
               { 
                  //sensornummer=0xAF;
                  //Sensornummerlesen(0,&sensornummer);
                  //	lcd_gotoxy(0,0);
                  //	lcd_puts("Sens\0");
                  //	lcd_putc('1');
                  //	lcd_putc(' ');
                  //	lcd_puthex(sensornummer);
                  
               }break;
                  
               case 7:// Schalter rückwaerts
               {
                  /*
                   sensornummer=0x00;
                   Sensornummerlesen(0,&sensornummer);
                   lcd_gotoxy(0,0);
                   lcd_puts("Sens\0");
                   lcd_putc('0');
                   lcd_putc(' ');
                   lcd_puthex(sensornummer);
                   */
               }break;
                  
               case 8://
               {
                  /*
                   sensornummer=0x00;
                   Sensornummerlesen(1,&sensornummer);
                   lcd_gotoxy(0,0);
                   lcd_puts("Sens\0");
                   lcd_putc('1');
                   lcd_putc(' ');
                   lcd_puthex(sensornummer);
                   */
                  
               }break;
                  
               case 9:// Schalter vorwaerts
               {
                  /*
                   sensornummer=0x00;
                   Sensornummerlesen(2,&sensornummer);
                   lcd_gotoxy(0,0);
                   lcd_puts("Sens\0");
                   lcd_putc('2');
                   lcd_putc(' ');
                   lcd_puthex(sensornummer);
                   */
               }break;
                  
               case 10:// *
               { 
                  
               }break;
                  
               case 11://
               { 
                  
               }break;
                  
               case 12: // # Normalbetrieb einschalten
               {
                  Programmstatus &= ~(1<<MANUELL); // MANUELL OFF
                  Programmstatus &= ~(1<<MANUELLNEU);
                  MANUELL_PORT &= ~(1<<MANUELLPIN);
               }
                  
            }//switch Tastatur
            
            //				delay_ms(400);
            //				lcd_gotoxy(18,1);
            //				lcd_puts("  ");		// Tastenanzeige loeschen
            
         }//if TastaturCount	
         
      }
   }
   
   
   return 0;
}
