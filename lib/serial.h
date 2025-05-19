#ifndef SERIAL_H
#define SERIAL_H


#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include "registers.h"

#define BIT(x) (1UL << (x))


// Prototype function for USART1 (F411RE), USART2 (F429ZI) --> STM32F411RE UFQFPN48 package
// Calling this function will initialize the UART peripheral
// with the specified baud rate

void spin(volatile uint32_t count);
void uart_write_byte(struct uart *uart, uint8_t byte);
void uart_init(struct uart *uart, unsigned long baud);
void uart_write_buf(struct uart *uart, char *buf, int len);
int uart_read_ready(struct uart *uart);
uint8_t uart_read_byte(struct uart *uart) ;





#endif

