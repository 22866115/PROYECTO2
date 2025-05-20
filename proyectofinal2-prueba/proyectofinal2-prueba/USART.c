
/* 
 Universidad del Valle de Guatemala
 USART.c - 
 Autor: Rodrigo Lara
 
 ---------------
*/

#include "USART.h"
#include <avr/interrupt.h>

// Variables globales para comunicación
volatile uint8_t rx_buffer[4];
volatile uint8_t rx_index = 0;
volatile uint8_t command_ready = 0;

void initUART9600(void){
    // Configurar pines TX y RX
    DDRD &= ~(1<<DDD0);     // RX (PD0) como entrada
    DDRD |= (1<<DDD1);      // TX (PD1) como salida
    
    // Configurar registro A
    UCSR0A = (1<<U2X0);     // Modo FAST U2X0 = 1
    
    // Configurar registro B
    UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); // Interrupciones RX, habilitar RX y TX
    
    // Configurar registro C
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // 8 bits datos, no paridad, 1 bit stop
    
    // Baudrate = 9600
    UBRR0 = 207;
}

void writeUART(char character){
    while(!(UCSR0A & (1<<UDRE0))); // Esperar hasta que el buffer esté vacío
    UDR0 = character;
}

void writeString(const char *str){
    while(*str){
        writeUART(*str++);
    }
}

void writeNumber(uint8_t number){
    // Enviar número como string de 3 dígitos
    uint8_t hundreds = number / 100;
    uint8_t tens = (number % 100) / 10;
    uint8_t units = number % 10;
    
    writeUART(hundreds + '0');
    writeUART(tens + '0');
    writeUART(units + '0');
    writeUART('\n');
}

uint8_t getCommand(void){
    if(command_ready){
        command_ready = 0;
        return 1;
    }
    return 0;
}

uint8_t getServoNumber(void){
    return rx_buffer[0] - '0'; // Convertir char a número (1-5)
}

uint8_t getServoPosition(void){
    // Convertir los 3 dígitos a valor numérico (0-255)
    uint8_t hundreds = rx_buffer[1] - '0';
    uint8_t tens = rx_buffer[2] - '0';
    uint8_t units = rx_buffer[3] - '0';
    
    return (hundreds * 100) + (tens * 10) + units;
}

// Interrupción de recepción
ISR(USART_RX_vect){
    uint8_t received = UDR0;
    
    if(rx_index < 4 && received != '\n'){
        rx_buffer[rx_index++] = received;
    }
    else if(received == '\n'){
        if(rx_index == 4){ // Comando completo recibido
            command_ready = 1;
        }
        rx_index = 0; // Resetear para nuevo comando
    }
}