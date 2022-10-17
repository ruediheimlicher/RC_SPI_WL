/*
 *  adc.h
 *  TWI_Master
 *
 *  Created by Sysadmin on 12.11.07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */
struct adcwert16 {
  uint8_t wertH;
  uint8_t wertL;
  uint8_t wert8H;//obere 8 Bit
};
#define ADC_REF_POWER     (1<<REFS0)
#define ADC_REF_INTERNAL  ((1<<REFS1) | (1<<REFS0))
#define ADC_REF_EXTERNAL  (0)

// These prescaler values are for high speed mode, ADHSM = 1
#if F_CPU == 16000000L
#define ADC_PRESCALER ((1<<ADPS2) | (1<<ADPS1)) // f/64
#elif F_CPU == 8000000L
#define ADC_PRESCALER ((1<<ADPS2) | (1<<ADPS0)) // f/32
#elif F_CPU == 4000000L
#define ADC_PRESCALER ((1<<ADPS2))
#elif F_CPU == 2000000L
#define ADC_PRESCALER ((1<<ADPS1) | (1<<ADPS0))
#elif F_CPU == 1000000L
#define ADC_PRESCALER ((1<<ADPS1))
#else
#define ADC_PRESCALER ((1<<ADPS0))
#endif


extern struct adcwert16 ADCWert16;

volatile uint8_t VREF_Quelle = 0;
struct adcwert16 readKanal16Bit(uint8_t kanal);
void closeADC(void);
uint16_t readKanal(uint8_t derKanal);//Unsere Funktion zum ADC-Channel aus lesen

uint16_t readKanalOrig(uint8_t derKanal, uint8_t num); //Unsere Funktion zum ADC-Channel aus lesen
