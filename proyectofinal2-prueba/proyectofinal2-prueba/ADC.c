/*

 *  Author: yemo0
 */ 

// Configuraci?n para el ADC.

#include "ADC.h"

void initADC(void){
	ADMUX = 0;
	ADMUX |= (1 << REFS0);		// VREF = AVCC
	ADMUX &= ~(1 << REFS1);		// VREF = AVCC
	ADMUX |= (1 << ADLAR);		// JUSTIFICACION A LA DERECHA
	
	ADCSRA = 0;
	ADCSRA |= (1 << ADEN);		// ENCIENDER EL ADC
	ADCSRA |= (1 << ADIE);		// HABILITAR ISR ADC
	ADCSRA |= (1 << ADPS0);
	ADCSRA |= (1 << ADPS1);
	ADCSRA |= (1 << ADPS2);		// PRESCALES 128 -> 16M = 125kHz
	
	//Entradas para los potenciometros.
	
	DIDR0 |= (1 << ADC0D);		// DESABILITAR LA ENTRADA DIGITAL PC0
	DIDR0 |= (1 << ADC1D);		// DESABILITAR LA ENTRADA DIGITAL PC1
	DIDR0 |= (1 << ADC2D);		// DESABILITAR LA ENTRADA DIGITAL PC2
	DIDR0 |= (1 << ADC3D);		// DESABILITAR LA ENTRADA DIGITAL PC3
}

uint8_t ADC_CONVERT(uint8_t canal){
	ADMUX = (ADMUX & 0xF0)|canal;	// SELECCION DEL CANAL
	ADCSRA |= (1<<ADSC);			// INICIA EL ADC
	while((ADCSRA)&(1<<ADSC));		// FINALIZA LA CONVERSION
	return(ADCH);
}