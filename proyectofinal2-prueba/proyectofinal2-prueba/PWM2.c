/*
 * PWM2.c
 *
 * Created: 19/05/2025 04:05:31
 *  Author: Rodrigo Lara
 */ 


//Se llama a la libreria donde se declararon los prototipos de funci?n
#include "PWM2.h"

//Configuraci?n del mapeado del los servomotores
float map1(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}

void initPWM2A(uint8_t inverted, uint16_t prescaler){
	//Definimos que el pin PB3 es una saldia (OC2A)  - NO INVERTIDA
	DDRB |= (1 << DDB3); 
	
	//-->Definimos PWM modo FAST TOP = 0xFF (MAX)
	TCCR2A |= (1 << WGM21)|(1 << WGM20); //PWM MODO FAST
	
	if (inverted) {
		//-->Configurando OC2A como INVERTIDO
		TCCR2A |= (1 << COM2A1)|(1 << COM2A0);//PWM INVERTIDO
		} else	 {
			//-->Configurando OCBA como INVERTIDO
		TCCR2A |= (1 << COM2A1);//PWM NO INVERTIDO
	}

	if (prescaler == 1024) {
		//Seleccionamos el Prescaler de 1024
		TCCR2B |= (1 << CS22)|(1 << CS21)|(1 << CS20);
		} else {
		TCCR2B |= (1 << CS20);
	}
}

//--> Se configura el ciclo de trabajo del canal A // PB3
void updateDutyCA2(uint8_t duty){
	OCR2A = map1(duty,0,255,6,36);
}

