/*
 * ADC.h
 *
 * Created: 19/05/2025 04:00:20
 *  Author: Rodrigo Lara
 */ 




#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <stdint.h>

void initADC(void);

uint8_t ADC_CONVERT(uint8_t canal);


#endif /* ADC_H_ */