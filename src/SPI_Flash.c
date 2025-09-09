

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




// Global variables for interrupt-driven operations
// Private variables (keep these static)
static volatile bool flash_operation_complete = true;
static volatile flash_operation_t flash_operation_type = FLASH_OP_NONE;




// Simple delay function
void delay_us(uint32_t us)
{
    // Rough delay - adjust based on your system clock
    // Assuming 400MHz system clock
    volatile uint32_t count = us * 100U;
    while(count--);
}

// Public accessor functions
bool is_flash_operation_complete(void)
{
    return flash_operation_complete;
}

flash_operation_t get_flash_operation_type(void)
{
    return flash_operation_type;
}

void set_flash_operation_complete(bool complete)
{
    flash_operation_complete = complete;
}

void set_flash_operation_type(flash_operation_t type)
{
    flash_operation_type = type;
}


// Initialize SPI4 in master mode, this function configures:
// PE2: SPI4_SCK  (AF5),PE5: SPI4_MISO (AF5), PE6: SPI4_MOSI (AF5), PE4: SPI4_NSS  (AF5) - but managed by software
// SPI Configuration:
// Master mode, 8-bit data frame, Software SS management (SSM=1, SSI=1),
// Clock polarity and phase: Mode 0 (CPOL=0, CPHA=0), MSB first, Baud rate: PCLK/256 (adjust as needed)
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

    // Enable interrupt
    __asm volatile ("cpsie i" ::: "memory"); // Enable global interrupts if not already done
    SPI4->IER |= (1U << 3);  // EOTIE = 1 Enable SPI4 peripheral interrupts
    
    // 4. Set NVIC priority for SPI4 (IRQ 84)
    uint32_t priority_reg_index = 84 / 4;        // = 21  IRQ 84 -> IPR[21], byte 0 (84/4 = 21, 84%4 = 0)
    uint32_t priority_byte_offset = (84 % 4) * 8; // = 0
    NVIC->IPR[priority_reg_index] = (NVIC->IPR[priority_reg_index] & ~(0xFFU << priority_byte_offset)) | 
                                    ((2U << 4) << priority_byte_offset);
    
    NVIC->ICPR[2] |= (1U << 20);  // Clear pending bit (84 -> ICPR[2] bit 20)
    NVIC->ISER[2] |= (1U << 20);  // Enable IRQ 84 (84 -> ISER[2] bit 20) in NVIC

    // Enable SPI4 (SPE = 1)
    SPI4->CR1 |= (1U << 0);

    // Short delay to stabilize
    delay_us(10U);
    CS_HIGH();
    return FLASH_OK;
}

// Keep CS low for entire command sequence
void read_flash_id_sequence(void)
{
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_READ_ID;
    CS_LOW();  // Keep low for entire sequence
    
    // Always reset everything before transfer
    SPI4->CR1 &= ~((1U << 0) | (1 << 9));  // Disable & clear CSTART
    SPI4->IFCR = 0xFFFFFFFF;               // Clear all flags
    SPI4->CR2 = 4;                         // Set TSIZE
    SPI4->CR1 |= (1U << 0);                // Re-enable

    // Wait until TXE (Transmit buffer empty, bit 1) is set
    while (!(SPI4->SR & (1U << 1)));

    // Load data into TXDR
    *(volatile uint8_t*)&SPI4->TXDR = 0x9F; // RDID command
    *(volatile uint8_t*)&SPI4->TXDR = 0x00; // Dummy byte
    *(volatile uint8_t*)&SPI4->TXDR = 0x00; // Dummy byte
    *(volatile uint8_t*)&SPI4->TXDR = 0x00; // Dummy byte
 
    // Start the transaction
    SPI4->CR1 |= (1U << 9); // CSTART = 1

}

// Read one word (4 bytes) from flash - simplified version
flash_status_t flash_read_word(uint32_t address)
{
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_READ_DATA;

    CS_LOW();
    
    // Reset SPI state
    SPI4->CR1 &= ~((1U << 0) | (1U << 9));
    SPI4->IFCR = 0xFFFFFFFF;
    SPI4->CR2 = 8;  // Command + 3 address bytes + 4 data bytes
    SPI4->CR1 |= (1U << 0);
    
    // Wait for TXP
    while (!(SPI4->SR & (1U << 1)));
    
    // Send read command and address
    *(volatile uint8_t*)&SPI4->TXDR = 0x03U;                    // READ command;
    *(volatile uint8_t*)&SPI4->TXDR = (address >> 16) & 0xFF;   // 24bit adress
    *(volatile uint8_t*)&SPI4->TXDR = (address >> 8) & 0xFF;
    *(volatile uint8_t*)&SPI4->TXDR = address & 0xFF;
    
    // Send 4 dummy bytes to clock out the word
    *(volatile uint8_t*)&SPI4->TXDR = 0x00;
    *(volatile uint8_t*)&SPI4->TXDR = 0x00;
    *(volatile uint8_t*)&SPI4->TXDR = 0x00;
    *(volatile uint8_t*)&SPI4->TXDR = 0x00;
    
    // Start transfer
    SPI4->CR1 |= (1U << 9);
    
    return FLASH_OK;
}