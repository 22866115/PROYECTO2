/*
 * PWM0.c
 *
 * Created: 19/05/2025 04:01:39
 *  Author: Rodrigo Lara
 */ 


//Se llama a la libreria donde se declararon los prototipos de funci?n
#include "PWM0.h"			

//Configuraci?n del mapeado del los servomotores
float map2(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}

void initPWM0FastA(uint8_t inverted, uint16_t prescaler){
	//Definimos que el pin PD6 es una saldia (OC0A)  - NO INVERTIDA
	DDRD |= (1<<DDD6);
	
	
	if(inverted){			
		//Configurando OC0A como INVERTIDO
		TCCR0A |= (1<<COM0A1)|(1<<COM0A0);
		}else{
		//Configurando OC0A como NO INVERTIDO
		TCCR0A |= (1<<COM0A1);
	}
	
	//Definimos PWM modo FAST TOP = 0xFF (MAX) --- Es igual para ambos TCCR0A y TCCR0B
	TCCR0A |= (1<<WGM01)|(1<<WGM00);
	
	//Seleccionamos el Prescaler de 1024
	TCCR0B |= (1<<CS02)|(1<<CS00);
}

//--> Se configura el ciclo de trabajo del canal A // PD6
void updateDutyCycleA(uint8_t duty){
	OCR0A = map2(duty,0,255,6,36);
}
