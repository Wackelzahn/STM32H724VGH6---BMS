

// Created: 2025-09-05 16:00:00
// Author: M. Schneider
// File: src/SPI_Flash.c
// -----------------------------------------------------
// Target Device: STM32H742VGH6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is the implementation file 
// for the SPI Flash MX25L12833FM2I
// -----------------------------------------------------


#include "SPI_Flash.h"





// Simple delay function
void delay_us(uint32_t us)
{
    // Rough delay - adjust based on your system clock
    // Assuming 400MHz system clock
    volatile uint32_t count = us * 100U;
    while(count--);
}

// SPI byte transfer
uint8_t spi_transfer(uint8_t data)
{

        // CRITICAL: Reset TSIZE before each transfer
    //SPI4->CR1 &= ~(1U << 0);  // Disable SPI
    //SPI4->CR2 = 1U;           // Set TSIZE = 1
    //SPI4->CR1 |= (1U << 0);   // Re-enable SPI


    // Set transfer count to 1 byte in CR2 register
    // SPI4->CR2 = (SPI4->CR2 & ~0xFFFFU) | 1;

    // Always reset everything before transfer
    SPI4->CR1 &= ~((1U << 0) | (1 << 9));  // Disable & clear CSTART
    SPI4->IFCR = 0xFFFFFFFF;                        // Clear all flags
    SPI4->CR2 = 1;                                  // Reset TSIZE
    SPI4->CR1 |= (1U << 0);                       // Re-enable

    // Wait until TXE (Transmit buffer empty, bit 1) is set
    while (!(SPI4->SR & (1U << 1)));


    // Load data into TXDR
    *(volatile uint8_t*)&SPI4->TXDR = data;
 

    // Start the transaction
    SPI4->CR1 |= (1U << 9); // CSTART = 1

    // check for RXP package available
    while (!(SPI4->SR & (1u << 3)));

    //  Use 8-bit read instead of 32-bit
    uint8_t received_byte = *(volatile uint8_t*)&SPI4->RXDR;  // 8-bit read

    // Wait for end of transfer (EOT, bit 3)
    while (!(SPI4->SR & (1U << 3)));

    // Clear the EOT flag
    SPI4->IFCR |= (1U << 3);

    // clear TXTFL flag
    SPI4->IFCR |= (1U << 4);

    // Clear CSTART bit for next transfer
    SPI4->CR1 &= ~(1U << 9);

    return received_byte;
}

// Initialize GPIO and SPI4
flash_status_t mx25l_init(void)
{
 
    
    // Enable clocks
    RCC->AHB4ENR |= (1U << 4);    // GPIOE clock
    RCC->APB2ENR |= (1U << 13);   // SPI4 clock
    
  
    // Configure GPIO pins
    // PE2 (SCK), PE5 (MISO), PE6 (MOSI) - Alternate Function
    GPIOE->MODER &= ~((3U << 4) | (3U << 10) | (3U << 12));     // Clear mode bits
    GPIOE->MODER |= (2U << 4) | (2U << 10) | (2U << 12);        // Alternate function mode
    
    // PE4 (CS) - GPIO Output (DO NOT configure as SPI NSS alternate function!)
    GPIOE->MODER &= ~(3U << 8);   // Clear mode bits
    GPIOE->MODER |= (1U << 8);    // Output mode
    
    // Set speed to very high
    GPIOE->OSPEEDR |= (3U << 4) | (3U << 8) | (3U << 10) | (3U << 12);
    
    // No pull-up/pull-down for SPI pins, pull-up for CS
    GPIOE->PUPDR &= ~((3U << 4) | (3U << 8) | (3U << 10) | (3U << 12));
    GPIOE->PUPDR |= (1U << 8);    // Pull-up for CS
    
    // Set alternate function (AF5 for SPI4) - ONLY for SCK, MISO, MOSI (NOT CS!)
    GPIOE->AFR[0] &= ~((0xFU << 8) | (0xFU << 20) | (0xFU << 24));  // Clear AF bits
    GPIOE->AFR[0] |= (5U << 8) | (5U << 20) | (5U << 24);           // AF5
    // NOTE: PE4 (CS) is kept as GPIO, NOT configured for AF5!
    
    // Set CS HIGH and keep it stable
    CS_HIGH();
    
    // Disable SPI4 before configuration
    SPI4->CR1 &= ~(1U << 0) ; // SPE = 0
    
    // Configure SPI4 CR1 with SSI = 1 for software NSS
    SPI4->CR1 = 0;              // Clear all bits
    SPI4->CR1 = (1U << 12);     // SSI = 1 (Internal slave select)

    // Configure SPI4_CFG1
    SPI4->CFG1 = 0;  // Clear all bits
    SPI4->CFG1 |= (7U << 0);        // 8-bit data size (7+1=8)
    SPI4->CFG1 |= (0U << 5);        // FIFO threshold FTHLV: 1 byte
    SPI4->CFG1 |= (4U << 28);       // Baud rate: PCLK/256

    // Configure SPI4 CFG2 (master mode, Mode 0, enable SSM)
    SPI4->CFG2 =  (0U << 17)    // COMM = 0 (Full duplex)
                | (1U << 22)    // MASTER = 1 (Master mode)
                | (0U << 25)    // CPOL = 0 (low idle, Mode 0)
                | (0U << 24)    // CPHA = 0 (first clock, Mode 0)
                | (0U << 29)    // SSOE = 0 (Disable NSS output)
                | (1U << 26)    // SSM = 1 (Enable software slave management)
                | (1U << 31);   // AFCNTR = 1 (Alternate function control)

    // Set transfer size to 1 byte in CR2 (TSIZE[15:0])
    SPI4->CR2 = 1U; // TSIZE = 1 (single byte)

    // Clear any pending flags
    SPI4->IFCR = 0xFFFFFFFF;

    // Enable SPI4 (SPE = 1)
    SPI4->CR1 |= (1U << 0);

    // Short delay to stabilize
    delay_us(10U);
    CS_HIGH();
    return FLASH_OK;
}

// Keep CS low for entire command sequence
void read_flash_id_sequence(uint8_t* response)
{
    CS_LOW();  // Keep low for entire sequence
    
    // Transfer 1: Send command
    spi_transfer(0x9FU);  // Read Electronic Manufacturer ID & Device ID RDID
    
    // Transfers 2-4: Read response
    response[0] = spi_transfer(0x00);
    response[1] = spi_transfer(0x00); 

    
    CS_HIGH(); // Only raise CS at the end
}