
// Created: 2025-09-05 16:00:00
// Author: M. Schneider
// File: src/SPI3_LTC6813.c
// -----------------------------------------------------
// Target Device: STM32H742VGH6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is the implementation file 
// for LTC6813-1 BAttery Management system
// -----------------------------------------------------


#include "SPI3_LTC6813.h"

// LTC6831 specific settings
#define LTC6831_SPI_MODE        3           // CPOL=1, CPHA=1
#define LTC6831_MAX_FREQ_HZ     1000000     // 1 MHz max SPI frequency

// SPI CR1 bits
#define SPI_CR1_SPE             (1 << 0)   // SPI Enable
#define SPI_CR1_MASRX           (1 << 8)   // Master automatic SUSP in Receive mode
#define SPI_CR1_CSTART          (1 << 9)   // Master transfer start
#define SPI_CR1_CSUSP           (1 << 10)  // Master SUSPend request
#define SPI_CR1_HDDIR           (1 << 11)  // Half-duplex direction
#define SPI_CR1_SSI             (1 << 12)  // Internal slave select

// SPI CFG1 bits
#define SPI_CFG1_DSIZE_8BIT     (7 << 0)   // Data size = 8 bits
#define SPI_CFG1_FTHLV_1DATA    (0 << 5)   // FIFO threshold = 1 data
#define SPI_CFG1_MBR_DIV16      (3 << 28)  // Baud rate = fPCLK/16
#define SPI_CFG1_MBR_DIV32      (4 << 28)  // Baud rate = fPCLK/32
#define SPI_CFG1_MBR_DIV64      (5 << 28)  // Baud rate = fPCLK/64
#define SPI_CFG1_MBR_DIV128     (6 << 28)  // Baud rate = fPCLK/128
#define SPI_CFG1_MBR_DIV256     (7 << 28)  // Baud rate = fPCLK/256

// SPI CFG2 bits
#define SPI_CFG2_MASTER         (1 << 22)  // Master mode
#define SPI_CFG2_COMM_FULL      (0 << 17)  // Full-duplex
#define SPI_CFG2_SSM            (1 << 26)  // Software slave management
#define SPI_CFG2_CPOL           (1 << 25)  // Clock polarity
#define SPI_CFG2_CPHA           (1 << 24)  // Clock phase
#define SPI_CFG2_LSBFRST        (0 << 23)  // MSB first
#define SPI_CFG2_SSOM           (1 << 30)  // SS output management
#define SPI_CFG2_SSOE           (1 << 29)  // SS output enable
#define SPI_CFG2_AFCNTR         (1 << 31)  // Alternate function control

// CS pin control macros
#define CS3_HIGH()   (GPIOA->BSRR = (1U << 15))
#define CS3_LOW()    (GPIOA->BSRR = (1U << (15 + 16)))


// Initialize SPI3 for communication with LTC6813
//  * Pin configuration (custom mapping):   PA15 - SPI3_NSS  (Output) - Chip Select
//                                          PB4  - SPI3_MISO (AF6) - Master In Slave Out
//                                          PC10 - SPI3_SCK  (AF6) - Serial Clock
//                                          PD6  - SPI3_MOSI (AF5) - Master Out Slave In

void SPI3_LTC6831_Init(void)
{
    // 1. Enable clocks for GPIOA, GPIOB, GPIOC, GPIOD and SPI3
    RCC->AHB4ENR |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);  // Enable GPIOA, B, C, D
    RCC->APB1LENR |= (1 << 15);     // Enable SPI3
    for(volatile int i = 0; i < 100; i++);  // Small delay for clock stabilization

    // 2. Reset SPI3 peripheral
    RCC->APB1LRSTR |= (1 << 15);   // Assert reset
    for(volatile int i = 0; i < 10; i++);
    RCC->APB1LRSTR &= ~(1 << 15);  // Release reset

    // 3. Configure GPIO pins for SPI3
    // PA15 (NSS) - GPIO Output
    GPIOA->MODER &= ~(3U << 30);     // Clear mode bits
    GPIOA->MODER |= (1U << 30);      // Output mode
    GPIOA->OSPEEDR |= (3U << 30);    // Set high speed
    GPIOA->BSRR = (1U << 15);        // Set CS high (inactive)

    // PB4 (MISO) - Alternate Function 6
    GPIOB->MODER &= ~(3U << 8);     // Clear mode bits
    GPIOB->MODER |= (2U << 8);      // Alternate function mode
    GPIOB->OSPEEDR |= (3U << 8);    // Set high speed
    GPIOB->PUPDR &= ~(3U << 8);     // No pull-up/pull-down
    GPIOB->AFR[0] &= ~(0xFU << 16); // Clear AF bits for PB4
    GPIOB->AFR[0] |= (6U << 16);    // Set AF6 for PB4

    // PC10 (SCK) - Alternate Function 6
    GPIOC->MODER &= ~(3U << 20);    // Clear mode bits
    GPIOC->MODER |= (2U << 20);     // Alternate function mode
    GPIOC->OSPEEDR |= (3U << 20);   // Set high speed
    GPIOC->PUPDR &= ~(3U << 20);    // No pull-up/pull-down
    GPIOC->AFR[1] &= ~(0xFU << 8);  // Clear AF bits for PC10
    GPIOC->AFR[1] |= (6U << 8);     // Set AF6 for PC10

    // PD6 (MOSI) - Alternate Function 5
    GPIOD->MODER &= ~(3U << 12);    // Clear mode bits
    GPIOD->MODER |= (2U << 12);     // Alternate function mode
    GPIOD->OSPEEDR |= (3U << 12);   // Set high speed
    GPIOD->PUPDR &= ~(3U << 12);    // No pull-up/pull-down
    GPIOD->AFR[0] &= ~(0xFU << 24); // Clear AF bits for PD6
    GPIOD->AFR[0] |= (5U << 24);    // Set AF5 for

    CS3_HIGH();  // Ensure CS is high
    
    // 4. Configure SPI3 for LTC6831
    // Disable SPI3 before configuration
    SPI3->CR1 &= ~SPI_CR1_SPE;

    // Configure SPI3_CR1
    SPI3->CR1 = 0;              // Clear register
    SPI3->CR1 |= SPI_CR1_SSI;   // Internal slave select high

    // Configure SPI3_CFG1
    SPI3->CFG1 = 0;  // Clear register
    SPI3->CFG1 |= SPI_CFG1_DSIZE_8BIT;      // 8-bit data size
    SPI3->CFG1 |= SPI_CFG1_FTHLV_1DATA;     // FIFO threshold = 1 data
    SPI3->CFG1 |= SPI_CFG1_MBR_DIV128;      // Set baud rate to ~781 kHz (100MHz/128)
    
    // Configure SPI3_CFG2
    SPI3->CFG2 = 0;  // Clear register
    SPI3->CFG2 |= SPI_CFG2_MASTER;          // Master mode
    SPI3->CFG2 |= SPI_CFG2_COMM_FULL;       // Full-duplex
    SPI3->CFG2 |= SPI_CFG2_SSM;             // Software slave management
    SPI3->CFG2 |= SPI_CFG2_CPOL;            // CPOL = 1 for Mode 3
    SPI3->CFG2 |= SPI_CFG2_CPHA;            // CPHA = 1 for Mode 3
    SPI3->CFG2 &= ~SPI_CFG2_LSBFRST;        // MSB first
    SPI3->CFG2 |= SPI_CFG2_AFCNTR;          // Enable alternate function control

    // Configure SPI3_CR2 (set TSIZE if needed for automatic EOT generation)
    SPI3->CR2 = 0;  // Clear register

    // Clear any pending flags
    SPI3->IFCR = 0xFFFFFFFF;
    
    // 5. Enable SPI3
    SPI3->CR1 |= SPI_CR1_SPE;

}

