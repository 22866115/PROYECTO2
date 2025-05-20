/*
 * USART.h
 *
 * Created: 19/05/2025 04:07:20
 *  Author: Rodrigo Lara
 */ 



#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <stdint.h>

void initUART9600(void);
void writeUART(char character);
void writeString(const char *str);
void writeNumber(uint8_t number);
uint8_t getCommand(void);
uint8_t getServoNumber(void);
uint8_t getServoPosition(void);

#endif /* USART_H_ */