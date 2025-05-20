


//Se llama a la libreria donde se declararon los prototipos de funci?n
#include "PWM1.h"

//Configuraci?n del mapeado del los servomotores
float map(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}

void initPWM1A(uint8_t inverted, uint16_t prescaler, uint16_t top){
	//Definimos que el pin PB1 es una saldia (OC1A)  - NO INVERTIDA
	DDRB |= (1 << DDB1); 
	TCCR1A |= (1 << WGM11);				 //PWM MODO FAST ICR1
	TCCR1B |= (1 << WGM13)|(1 << WGM12); //PWM MODO FAST ICR1
	ICR1 = top;
	
	if (inverted) {
		TCCR1A |= (1 << COM1A1)|(1 << COM1A0);//PWM INVERTIDO
		} else	 {
		TCCR1A |= (1 << COM1A1);//PWM NO INVERTIDO
	}

	if (prescaler == 8) {
		//Se define un prescaler de 8
		TCCR1B |= (1 << CS11);
		} else {
		TCCR1B |= (1 << CS12);
	}
}

void initPWM1B(uint8_t inverted, uint16_t prescaler, uint16_t top){
	//Definimos que el pin PB2 es una saldia (OC1B)  - NO INVERTIDA
	DDRB |= (1 << DDB2); 
	TCCR1A |= (1 << WGM11);				 //PWM MODO FAST ICR1
	TCCR1B |= (1 << WGM13)|(1 << WGM12); //PWM MODO FAST ICR1
	ICR1 = top;
	
	if (inverted) {
		TCCR1A |= (1 << COM1B1)|(1 << COM1B0);//PWM INVERTIDO
		} else	 {
		TCCR1A |= (1 << COM1B1);//PWM NO INVERTIDO
	}

	if (prescaler == 8) {
		//Se define un prescaler de 8
		TCCR1B |= (1 << CS11);
		} else {
		TCCR1B |= (1 << CS12);
	}
}

//--> Se configura el ciclo de trabajo del canal A // PB1
void updateDutyCA1(uint8_t duty){   //--->900=0? 2800=90? 48000=180?
	OCR1A = map(duty,0,255,900,4800);
}

//--> Se configura el ciclo de trabajo del canal B // PB2
void updateDutyCB1(uint8_t duty){
	OCR1B = map(duty,0,255,900,4800);
}
//external interruption 