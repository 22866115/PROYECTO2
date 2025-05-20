
 //Universidad del Valle de Guatemala
 //Programacion de Microcontroladores
 //Autor: Rodrigo Javier Lara 
 //Proyecto: Proyecto 2
 

#define F_CPU 16000000UL	//Frecuencia de 16MHZ


//Librerias del sistema
#include <avr/io.h>			 //Liberia para los registros
#include <util/delay.h>		 //Libreria para los delays
#include <avr/interrupt.h>	 //Libreria para las interupciones
#include <stdint.h>			 //Libreria para enteros
#include <avr/eeprom.h>		//Libreria para la memoria eeprom


//Libreria propias

#include "PWM1.h"	//Libreria del TIMER1
#include "PWM2.h"		//Libreria del TIMER2
#include "PWM0.h"		//Libreria del TIMER0
#include "ADC.h"		//Libreria del ADC
#include "USART.h"	//Libreria de comunicacion adafruit

#include <avr/eeprom.h>

//Declaración de funciones
void INT0_ISR(void);		
void setup(void);			
void entradas_ADC(void);	
void INT_PCINT2(void);		

//Declaracion de motores servo para Eppromm 
void conf_servos(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);		

//Configuracion de vairiables
//Estados del sistema
uint8_t estado = 0;
uint8_t ADAFRUIT_IO = 1; // Bandera para identificar se se espera un canal o valor de ADAFRUIT
uint8_t CANAL_AD = 0; // Canal (a que enviar la se?al)
uint8_t ADA_Value = 0; // Valor transmitido desde adafruit
uint8_t UN = 0; // Valor transmitido desde adafruit
uint8_t DE = 0; // Valor transmitido desde adafruit
uint8_t CE = 0; // Valor transmitido desde adafruit

//Entradas del ADC
uint8_t DutyC1 = 0;
uint8_t DutyC2 = 0;
uint8_t DutyC3 = 0;
uint8_t DutyC4 = 0;

//Entradas del Los servos --> EEPROM
uint8_t servo1 = 0;
uint8_t servo2 = 0;
uint8_t servo3 = 0;
uint8_t servo4 = 0;

//Configuracion de variables USART 
volatile uint8_t servo1_MA = 0; // Valor de servo 1 cuando se conecta a ADAFRUIT/NUBE
volatile uint8_t servo2_MA = 0; // Valor de servo 2 cuando se conecta a ADAFRUIT/NUBE
volatile uint8_t servo3_MA = 0; // Valor de servo 3 cuando se conecta a ADAFRUIT/NUBE
volatile uint8_t servo4_MA = 0; // Valor de servo 4 cuando se conecta a ADAFRUIT/NUBE
volatile uint8_t RX_READ; // Valor de RX cuando se lee
volatile uint8_t RX_WRITE; // Valor de RX cuando se escribe

int main(void){
	cli();			//Se desactivan las interupcciones Globales
	
	//config de timers
	
	//timer1
	TCCR1A = 0;
	TCCR1B = 0;
	
	//timer2
	TCCR2A = 0;
	TCCR2B = 0;
	
	//timer0
	TCCR0A = 0;
	TCCR0B = 0;

	initADC();			//Configuraci?n del ADC
	initUART9600();		//Configuraci?n del USART
	
	//Configuracion del PWM//
	
	//--> TIMER1
	initPWM1A(no_invertido,8,39999);
	initPWM2A(no_invertido,1024);
	
	//--> TIMER 2
	initPWM1B(no_invertido,8,39999);
	
	//--> TIMER 0
	initPWM0FastA(no_invertido, 1024);

	//se configuran las interrupciones
	INT0_ISR();		//Funci?n de la interrupci?n INT0 -- PD2
	INT_PCINT2();	//Funci?n de la interrupci?n PCINT2 -- PD3 y PD5
	
	sei();		//Se activan las interupcciones Globales
	
	while (1) {							
		
		if (estado == 0){				//Estado 0 -- Modo manual 1 
										//-->Escribir EEPROM  -- Posici?n 1 y 2
			// -> LEDS					
			PORTB |= (1<<DDB4);		//Se enciende la led 1 - PB4
			PORTB &= ~(1<<DDB0);	//Se apaga la led 2 - PB0
			PORTD &= ~(1<<DDD7);	//Se apaga la led 3 - PD7
			
			entradas_ADC();			//Se llama a la funcion de los servos - manuales

		}
		else if (estado == 1){			//Estado 1 -- Modo manual 2
										//-->Escribir EEPROM  -- Posicion 3 y 4
			// -> LEDS
			PORTD |= (1<<DDD7);		//Se enciende la led 3 - PD7
			
			entradas_ADC();			//Se llama a la funcion de los servos - manuales

		}
				
		else if (estado == 2){		//Estado 2 -- Lectura de EEPROM Posicion 1 y 2
			// -> LEDS
			PORTB |= (1<<DDB0);		//Se enciende la led 2 - PB0
			PORTB &= ~(1<<DDB4);	//Se apaga la led 1 - PB4
			PORTD &= ~(1<<DDD7);	//Se apaga la led 3 - PD7
			
		}
		else if (estado == 3){		//Estado 3 -- Lectura de EEPROM Posicion 3 y 4
			// -> LEDS
			PORTD |= (1<<DDD7);		//Se enciende la led 3 - PD7
			PORTB &= ~(1<<DDB0);	//Se apaga la led 2 - PB0

		}
		else {						//Estado 4 -- Comunicaci?n ADAFRUIT_IO
			// -> LEDS
			PORTB |= (1<<DDB0);		//Se enciende la led 2 - PB0
			
			//--> Se llama a la funcion para configurar los servos
			conf_servos(servo1_MA, servo2_MA, servo3_MA, servo4_MA);

		}
	
	} // Ciclo while (bucle principal)
	
} //Ciclo Main


//Configuraciion de funciones

//-->Configuracion del boton de cambio de modo --- PD2
void INT0_ISR(void){
	EICRA = (1<<ISC01)|(1<<ISC00);			
	EIMSK = (1<<INT0);
	
	//Confifguraci?n de los puertos -- LEDS
	//-> Puertos de salida
	DDRB |= (1<<DDB4)|(1<<DDB0);			//Se configura PB4 y PB0 como salidas
	PORTB &= ~(1<<DDB4)|~(1<<DDB0);			//Se inicializa PB4 y PB0 en 0
	
	DDRD |= (1<<DDD7);						//Se configura PD7 como salida
	PORTB &= ~(1<<DDB7);					//Se inicializa PD7 en 0
	
	//->Puertos de entrada
	DDRD &= ~(1<<DDD2);		//PD2 como entrada --- Interupcion INT0
	PORTD |= (1<<DDD2);		//PD2 con pull up
}

//-->Configuracion de la interrupcion PCINT2 --- PD3 y PD5
void INT_PCINT2(void){
	//Configuraci?n puertos de entrada
	DDRD &= ~(1<<DDD3);			//->PD3 como entrada
	DDRD &= ~(1<<DDD5);			//->PD5 como entrada
	PORTD |= (1<<DDD3)|(1<<DDD5);		//PD3 y PD5 se inicializan en 0
	
	PCMSK2 |= (1<<DDD3)|(1<<DDD5);  //Configuraci?n de PCINT19 y PCINT21
	PCICR |= (1<<PCIE2);
}

//-->Configuraci?n de entradas generales
void setup (void){
	DDRC = 0;		// PUERTO C COMO ENTRADA
	
}

//-->Se define la funci?n de las entradas de ADC y modificaci?n de servomotores
void entradas_ADC(void){
	DutyC1 = ADC_CONVERT(7);	//Se lee el canal AD7 
	updateDutyCA1(DutyC1);		//Se envia los datos al servo 1 -- PWM1 Canal A
	ADMUX &= 0xF0;
	ADMUX |= (0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);	//Se multiplexea para ADC7
	ADCSRA |= (1<<ADSC);		//Se inicia nuevamente el ADC
	_delay_ms(10);
	
	
	DutyC2 = ADC_CONVERT(6);	//Se lee el canal AD6 
	updateDutyCA2(DutyC2);		//Se envia los datos al servo 2 -- PWM2 Canal A	
	ADMUX &= 0xF0;
	ADMUX |= (0<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0);	//Se multiplexea para ADC6
	ADCSRA |= (1<<ADSC);		//Se inicia nuevamente el ADC
	_delay_ms(10);
	
	
	DutyC3 = ADC_CONVERT(5);	//Se lee el canal AD5 
	updateDutyCB1(DutyC3);		//Se envia los datos al servo 3 -- PWM1 Canal B	
	ADMUX &= 0xF0;
	ADMUX |= (0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(1<<MUX0);	//Se multiplexea para ADC5
	ADCSRA |= (1<<ADSC);		//Se inicia nuevamente el ADC
	_delay_ms(10);
	
	
	DutyC4 = ADC_CONVERT(4);	//Se lee el canal AD4 
	updateDutyCycleA(DutyC4);	//Se envia los datos al servo 4 -- PWM0 Canal A	
	ADMUX &= 0xF0;
	ADMUX |= (0<<MUX3)|(1<<MUX2)|(0<<MUX1)|(0<<MUX0);	//Se multiplexea para ADC4
	ADCSRA |= (1<<ADSC);		//Se inicia nuevamente el ADC
	_delay_ms(10);
}

//--> Configuraci?n de la funci?n de EEPROM/SERVOMOTORES
void conf_servos(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4){
	updateDutyCA1(m1);		//Actualiza el servomotor 1
	_delay_ms(10);
	
	updateDutyCA2(m2);		//Actualiza el servomotor 2
	_delay_ms(10);
	
	updateDutyCB1(m3);		//Actualiza el servomotor 3
	_delay_ms(10);
	
	updateDutyCycleA(m4);	//Actualiza el servomotor 4
	_delay_ms(10);
	
}


//Configuraci?n Vectores de interrupci?n

//--> Vector de interrupcion Externo INT0
ISR(INT0_vect){
	estado++;		  //Se usa la variable "estado" como contador de los modos
	if(estado == 5){
		estado = 0;	  //Al llegar a 5 el contador, este se reinicia a 0--> Estado 0
	}
}

//--> Vector de interrupcion PCINT2
ISR(PCINT2_vect){
	if (estado == 0){				//Guarda la posici?n 1 y 2 en la EEPROM
		if((PIND & (1<<PIND3))==0){						//--> Posici?n 1	
			eeprom_write_byte((uint8_t*)0x00, DutyC1);
			eeprom_write_byte((uint8_t*)0x01, DutyC2);
			eeprom_write_byte((uint8_t*)0x02, DutyC3);
			eeprom_write_byte((uint8_t*)0x03, DutyC4);
		}  //eSPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
		
		if((PIND & (1<<PIND5))==0){						//--> Posici?n 2
			eeprom_write_byte((uint8_t*)0x04, DutyC1);
			eeprom_write_byte((uint8_t*)0x05, DutyC2);
			eeprom_write_byte((uint8_t*)0x06, DutyC3);
			eeprom_write_byte((uint8_t*)0x07, DutyC4);
		} //ESPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
	} 
	else if (estado == 1){				//Guarda la posici?n 3 y 4 en la EEPROM
		if((PIND & (1<<PIND3))==0){						//--> Posici?n 3
			eeprom_write_byte((uint8_t*)0x08, DutyC1);
			eeprom_write_byte((uint8_t*)0x09, DutyC2);
			eeprom_write_byte((uint8_t*)0x0A, DutyC3);
			eeprom_write_byte((uint8_t*)0x0B, DutyC4);
		} //eSPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
		
		if((PIND & (1<<PIND5))==0){						//--> Posici?n 4
			eeprom_write_byte((uint8_t*)0x0C, DutyC1);
			eeprom_write_byte((uint8_t*)0x0D, DutyC2);
			eeprom_write_byte((uint8_t*)0x0E, DutyC3);
			eeprom_write_byte((uint8_t*)0x0F, DutyC4);
		} //ESPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
		
	}
	else if (estado == 2){				//Realiza la posici?n 1 y 2 gurdada en la EEPROM
		if((PIND & (1<<PIND3))==0){						//--> Posici?n 1
			//_delay_ms(10);
			servo1 = eeprom_read_byte((uint8_t*)0x00);
			servo2 = eeprom_read_byte((uint8_t*)0x01);
			servo3 = eeprom_read_byte((uint8_t*)0x02);
			servo4 = eeprom_read_byte((uint8_t*)0x03);
			
			//-->Se llama a la funci?n para configurar los servos
			conf_servos(servo1, servo2, servo3, servo4); 
			
		} //ESPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
		
		if((PIND & (1<<PIND5))==0){						//--> Posici?n 2
			//_delay_ms(10);
			servo1 = eeprom_read_byte((uint8_t*)0x04);
			servo2 = eeprom_read_byte((uint8_t*)0x05);
			servo3 = eeprom_read_byte((uint8_t*)0x06);
			servo4 = eeprom_read_byte((uint8_t*)0x07);
			
			//-->Se llama a la funci?n para configurar los servos
			conf_servos(servo1, servo2, servo3, servo4);
		}//ESPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
	}
	else if (estado == 3){					//Realiza la posici?n 3 y 4 gurdada en la EEPROM
		if((PIND & (1<<PIND3))==0){							//--> Posici?n 3
			servo1 = eeprom_read_byte((uint8_t*)0x08);
			servo2 = eeprom_read_byte((uint8_t*)0x09);
			servo3 = eeprom_read_byte((uint8_t*)0x0A);
			servo4 = eeprom_read_byte((uint8_t*)0x0B);
			
			//-->Se llama a la funci?n para configurar los servos
			conf_servos(servo1, servo2, servo3, servo4);
		} //ESPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
		
		if((PIND & (1<<PIND5))==0){							//--> Posici?n 4
			servo1 = eeprom_read_byte((uint8_t*)0x0C);
			servo2 = eeprom_read_byte((uint8_t*)0x0D);
			servo3 = eeprom_read_byte((uint8_t*)0x0E);
			servo4 = eeprom_read_byte((uint8_t*)0x0F);
			
			//-->Se llama a la funci?n para configurar los servos
			conf_servos(servo1, servo2, servo3, servo4);
		}//ESPERAR 4 SEGUNDOS ANTES DE OTRA ACCI?N
	}
	else{
		//La interrupci?n no hace ninguna acci?n en este caso
	}
}

//--->Vector de interrupci?n ADC
ISR(ADC_vect){
	ADCSRA |= (1 << ADIF);	//LIMPIA LA BANDERA
}

//---->Vector de  interrupci?n de USART
ISR(USART_RX_vect){
	RX_READ = UDR0;			//Posici?n temporal para el valor leido
	switch(ADAFRUIT_IO){
		case 1:				// El primero es un char de 1 a 4
		CANAL_AD = RX_READ; //De 1 a 4
		ADAFRUIT_IO +=1;
		break;
		
		case 2:				//El 2do es Centenas
		CE = 100*(((uint8_t) RX_READ) - 48);
		ADAFRUIT_IO += 1;
		break;
		
		case 3:				//El 3ro es Decenas
		DE = 10*(((uint8_t) RX_READ) - 48);
		ADAFRUIT_IO += 1;
		break;
		
		case 4:				//El 4to es Unidades
		UN = ((uint8_t) RX_READ) - 48;
		ADA_Value = CE+DE+UN; //Suma de transmisiones
		switch (CANAL_AD)
		{ //Dependiendo de a que le estoy enviando el valor, modifica el valor del servo
			case '1':
			servo1_MA = ADA_Value;		//El valor de ADAFRUIT se almacena en el servo_1
			break;
			
			case '2':
			servo2_MA = ADA_Value;		//El valor de ADAFRUIT se almacena en el servo_2	
			break;
			
			case '3':
			servo3_MA = ADA_Value;		//El valor de ADAFRUIT se almacena en el servo_3
			break;
			
			case '4':
			servo4_MA = ADA_Value;		//El valor de ADAFRUIT se almacena en el servo_1
			break;
			
		}// Switch de los canales
		ADAFRUIT_IO = 1;		//Se establece en 1
		break;
		
	}//Switch --> Estados del Adafruit
}




