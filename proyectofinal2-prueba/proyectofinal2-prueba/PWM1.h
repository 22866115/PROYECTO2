

#ifndef PWM1_H_
#define PWM1_H_

//Se llaman a las librerias
#include <avr/io.h>
#include <stdint.h>

//Definimos etiquetas/Variables que se pueden usar mas adelante
#define invertido 1
#define no_invertido 0

void initPWM1A(uint8_t inverted, uint16_t prescaler, uint16_t top);	//Configuraci?n Canal A

void initPWM1B(uint8_t inverted, uint16_t prescaler, uint16_t top);	//Configuraci?n Canal B

void updateDutyCA1(uint8_t duty);	//Configurar ciclo de trabajo Canal A

void updateDutyCB1(uint8_t duty);	//Configurar ciclo de trabajo Canal B


//Declarar una funci?n de mapeo para convertir el valor del potenci?metro en el ciclo de trabajo del PWM
float map(float x, float in_min, float in_max, float out_min, float out_max);	

#endif /* PWM1_H_ */