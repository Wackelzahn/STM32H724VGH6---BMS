#include "serial.h"


void spin(volatile uint32_t count) 
{
    while (count--) asm("nop");
}

void uart_write_byte(struct uart *uart, uint8_t byte) {
    uart->DR = byte;
    while ((uart->SR & BIT(7)) == 0) spin(1);
  }


void uart_init(struct uart *uart, unsigned long baud) 
    {
    
    // USART 2 not available in UFQFPN48 package
    // enable clock for UART
    if (uart == UART1) RCC->APB2ENR |= (1 << 4) ;
    if (uart == UART2) RCC->APB1ENR |= (1 << 17) ;
  

    if (uart == UART1) {            //PA9 = tx, PA10 = rx
   
      RCC->AHB1ENR |= BIT(0);         // enable clock for GPIOA

      // set PA9 (tx)
      // ----------------------------------------------
      // set PA9 to alternate function
      GPIOA->MODER &= ~(3U << 18);    // clear existing settings
      GPIOA->MODER |= (2U << 18);      // set to alternate function 
      // set PA9 to low speed
      GPIOA->OSPEEDR &= ~(3U << 18);  // clear existing settings
      // set PA9 to alternate function AF7
      GPIOA->AFRH &= ~(15U << 4);    // clear existing settings
      GPIOA->AFRH |= (7U << 4);      // set to alternate function AF7

      // set PA10 (rx)
      // ----------------------------------------------
      // set PA10 to alternate function
      GPIOA->MODER &= ~(3U << 20);    // clear existing settings
      GPIOA->MODER |= (2U << 20);     // set to alternate function
      // Set PA10 to low speed
      GPIOA->OSPEEDR &= ~(3U << 20);  // clear existing settings
      // set PA10 to alternate function AF7
      GPIOA->AFRH &= ~(15U << 8);      // clear existing settings
      GPIOA->AFRH |= (7U << 8);       // set to alternate function AF7


    uart->CR1 = 0;  // Disable UART
    uart->BRR = 16000000 / baud;  // Set baud rate
    uart->CR2 |= 0 << 12;  // 1 stop bit
    uart->CR1 = BIT(13) | BIT(5) | BIT(2) | BIT(3);  // Enable UART, RX_IRQ, RX, TX
    
    // Enable NVIC inerrupt for USART1
    NVIC->ISER[1] |= (1 << 5); // Set bit 5 in NVIC_ISER1
    
    }
    if (uart == UART2) {
      // tx = PD5, rx = PD6

      // enable clock for GPIOD
      RCC->AHB1ENR |= BIT(3);         // enable clock for GPIOD
      

     // set PD5 (tx)
      // ----------------------------------------------
      // set PD5 to alternate function
      GPIOD->MODER &= ~(3U << 10);    // clear existing settings
      GPIOD->MODER |= (2U << 10);      // set to alternate function
      // Set PD5 to low speed
      GPIOD->OSPEEDR &= ~(3U << 10);  // clear existing settings
      // set PD5 to alternate function AF7
      GPIOD->AFRL &= ~(15U << 20);    // clear existing settings
      GPIOD->AFRL |= (7U << 20);      // set to alternate function AF7
      

      // set PD6 (rx)
      // ----------------------------------------------
      // set PD6 to alternate function
      GPIOD->MODER &= ~(3U << 12);    // clear existing settings
      GPIOD->MODER |= (2U << 12);     // set to alternate function
      // Set PD6 to low speed
      GPIOD->OSPEEDR &= ~(3U << 12);  // clear existing settings
      // set PD6 to alternate function AF7
      GPIOD->AFRL &= ~(15U << 24);    // clear existing settings
      GPIOD->AFRL |= (7U << 24);      // set to alternate function AF7
       
      uart->CR1 = 0;  // Disable UART
      uart->BRR = 16000000 / baud;  // Set baud rate
      uart->CR2 |= 0 << 12;  // 1 stop bit
      uart->CR1 = BIT(13) | BIT(5) | BIT(2) | BIT(3);  // Enable UART, RX_IRQ, RX, TX
    
      // Enable NVIC inerrupt for USART2
      NVIC->ISER[1] |= (1 << 6); // Set bit 6 in NVIC_ISER1

    }

  }

void uart_write_buf(struct uart *uart, char *buf, int len) {
    while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
  }
  
int uart_read_ready(struct uart *uart) {
    return uart->SR & BIT(5);  // If RXNE bit is set, data is ready
  }
  
uint8_t uart_read_byte(struct uart *uart) {
    return (uint8_t) (uart->DR & 255);
  }
  