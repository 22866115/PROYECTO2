/*
 * PWM2.h
 *
 * Created: 19/05/2025 04:05:41
 *  Author: Rodrigo Lara
 */ 



#ifndef PWM2_H_
#define PWM2_H_

//->Se definen las librerias
#include <avr/io.h>		//Liberia para LOS REGISTROS
#include <stdint.h>

//Definimos etiquetas/Variables que se pueden usar mas adelante
#define invertido 1
#define no_invertido 0

//Declarar una funci?n de mapeo para convertir el valor del potenci?metro en el ciclo de trabajo del PWM
float map1(float x, float in_min, float in_max, float out_min, float out_max);	

//Prototipo de funci?n para el Canal A
void initPWM2A(uint8_t inverted, uint16_t prescaler);

//Configurar ciclo de trabajo del Canal A
void updateDutyCA2(uint8_t duty);


#endif /* PWM2_H_ */