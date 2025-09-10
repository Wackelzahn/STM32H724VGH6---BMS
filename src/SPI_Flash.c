

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

typedef enum {
    FLASH_OP_NONE,
    FLASH_OP_WRITE_ENABLE,
    FLASH_OP_READ_STATUS,
    FLASH_OP_PAGE_PROGRAM,
    FLASH_OP_READ_DATA,
    FLASH_OP_SECTOR_ERASE,
    FLASH_OP_READ_ID
} flash_operation_t;

static volatile bool flash_operation_complete = true;
static volatile flash_operation_t flash_operation_type = FLASH_OP_NONE;
static volatile uint8_t flash_read_buffer[4];  // Buffer for read operations
static volatile uint8_t flash_status_reg = 0;


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

// Return the 32-bit data word from the read buffer
void return_data_word(uint8_t* data_buffer)
{
    
        data_buffer[0] = flash_read_buffer[0];
        data_buffer[1] = flash_read_buffer[1];
        data_buffer[2] = flash_read_buffer[2];
        data_buffer[3] = flash_read_buffer[3];
                      
    
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
flash_status_t read_flash_id_sequence(void)
{
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_READ_ID;

    // Clear buffer first
    flash_read_buffer[0] = 0;
    flash_read_buffer[1] = 0;
    flash_read_buffer[2] = 0;
    flash_read_buffer[3] = 0;

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

    return FLASH_OK;

}


// Wait for flash operation to complete
flash_status_t flash_wait_ready(void)
{
    uint8_t status = 0;
    volatile uint32_t timeout = 1000000;  // Large timeout for erase operations
    
    do {
        if ((flash_read_status(&status)) != FLASH_OK) {
            return FLASH_ERROR;
        }
        timeout--;
    } while ((status & 0x01U) && timeout > 0);  // Wait until WIP bit is cleared (Write In Progress)
    
    return (timeout > 0) ? FLASH_OK : FLASH_ERROR;
}

// Erase a 4KB sector
// @param address Any address within the sector to erase
flash_status_t flash_erase_sector(uint32_t address)
{
    // Enable write
    if (flash_write_enable() != FLASH_OK) return FLASH_ERROR;
    
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_SECTOR_ERASE;
    
    CS_LOW();
    
    // Reset SPI state
    SPI4->CR1 &= ~((1U << 0) | (1U << 9));
    SPI4->IFCR = 0xFFFFFFFF;
    SPI4->CR2 = 4;  // Command + 3 address bytes
    SPI4->CR1 |= (1U << 0);
    
    // Wait for TXP
    while (!(SPI4->SR & (1U << 1)));
    
    // Send sector erase command
    *(volatile uint8_t*)&SPI4->TXDR = 0x20U; // Sector Erase command
    
    // Send 24-bit address (MSB first)
    *(volatile uint8_t*)&SPI4->TXDR = (address >> 16) & 0xFF;
    *(volatile uint8_t*)&SPI4->TXDR = (address >> 8) & 0xFF;
    *(volatile uint8_t*)&SPI4->TXDR = address & 0xFF;
    
    // Start transfer
    SPI4->CR1 |= (1U << 9);
    
    // Wait for completion
    volatile uint32_t timeout = 100000;
    while (!flash_operation_complete && timeout--);
    
    if (timeout == 0) return FLASH_ERROR;
    
    // Wait for erase to complete (can take several hundred ms)
    return flash_wait_ready();
}

// Read flash status register
flash_status_t flash_read_status(uint8_t* status)
{
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_READ_STATUS;
    
    CS_LOW();
    
    // Reset SPI state
    SPI4->CR1 &= ~((1U << 0) | (1U << 9));
    SPI4->IFCR = 0xFFFFFFFF;
    SPI4->CR2 = 2;  // Command + 1 status byte
    SPI4->CR1 |= (1U << 0);
    
    // Wait for TXP
    while (!(SPI4->SR & (1U << 1)));
    
    // Send read status command
    *(volatile uint8_t*)&SPI4->TXDR = 0x05;  // FLASH_CMD_RDSR Read Status Register
    *(volatile uint8_t*)&SPI4->TXDR = 0x00;  // Dummy byte
    
    // Start transfer
    SPI4->CR1 |= (1U << 9);
    
      // Wait for completion
    volatile uint32_t timeout = 100000;
    while (!flash_operation_complete && timeout--);
    
    if (timeout > 0) {
        *status = flash_status_reg;  // <- USE THE PARAMETER HERE!
        return FLASH_OK;
    }

    return FLASH_OK;
}

// Send write enable command
flash_status_t flash_write_enable(void)
{
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_WRITE_ENABLE;
    
    CS_LOW();
    
    // Reset SPI state
    SPI4->CR1 &= ~((1U << 0) | (1U << 9));
    SPI4->IFCR = 0xFFFFFFFF;
    SPI4->CR2 = 1;  // 1 byte transfer
    SPI4->CR1 |= (1U << 0);
    
    // Wait for TXP
    while (!(SPI4->SR & (1U << 1)));
    
    // Send write enable command
    *(volatile uint8_t*)&SPI4->TXDR = 0x06; // FLASH_CMD_WREN Write Enable
    
    // Start transfer
    SPI4->CR1 |= (1U << 9);
    
    // Wait for completion
    volatile uint32_t timeout = 100000;
    while (!flash_operation_complete && timeout--);
    
    return (timeout > 0) ? FLASH_OK : FLASH_ERROR;
}

// @param address 24-bit address to write to
// @param data Pointer to data buffer
// @param length Number of bytes to write (max 256 bytes per page)
flash_status_t flash_write_page(uint32_t address, const uint8_t* data, uint16_t length)
{
    if (length > 256) return FLASH_ERROR;  // Page size limit
    
    // Enable write
    if (flash_write_enable() != FLASH_OK) return FLASH_ERROR;
    
    flash_operation_complete = false;
    flash_operation_type = FLASH_OP_PAGE_PROGRAM;
    
    CS_LOW();
    
    // Reset SPI state
    SPI4->CR1 &= ~((1U << 0) | (1U << 9));
    SPI4->IFCR = 0xFFFFFFFF;
    SPI4->CR2 = 4 + length;  // Command + 3 address bytes + data
    SPI4->CR1 |= (1U << 0);
    
    // Wait for TXP
    while (!(SPI4->SR & (1U << 1)));
    
    // Send page program command
    *(volatile uint8_t*)&SPI4->TXDR = 0x02U; // PAGE PROGRAM command
    
    // Send 24-bit address (MSB first)
    *(volatile uint8_t*)&SPI4->TXDR = (address >> 16) & 0xFF;
    *(volatile uint8_t*)&SPI4->TXDR = (address >> 8) & 0xFF;
    *(volatile uint8_t*)&SPI4->TXDR = address & 0xFF;
    
    // Send data bytes
    for (uint16_t i = 0; i < length; i++) {
        while (!(SPI4->SR & (1U << 1)));  // Wait for TXP
        *(volatile uint8_t*)&SPI4->TXDR = data[i];
    }
    
    // Start transfer
    SPI4->CR1 |= (1U << 9);
    
    // Wait for completion
    volatile uint32_t timeout = 100000;
    while (!flash_operation_complete && timeout--);
    
    if (timeout == 0) return FLASH_ERROR;
    
    // Wait for programming to complete
    return flash_wait_ready();
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



// --------------------------------------------------------------------------------------------------
// Interrupt handler
// SPI4 interrupt receive handler
//---------------------------------------------------------------------------------------------------

void SPI4_IRQHandler(void) {
  // check for EOT
  if ((SPI4->SR & (1U << 3))) {
    switch(flash_operation_type) {
      
      case FLASH_OP_READ_STATUS:
        {
          // Read received byte from RXDR
          *(volatile uint8_t*)&SPI4->RXDR;  // Discard command byte
          flash_status_reg = *(volatile uint8_t*)&SPI4->RXDR;  // Read status byte

          flash_read_buffer[0] = flash_status_reg;;  // 8-bit read
          flash_read_buffer[1] = 0;
          flash_read_buffer[2] = 0;
          flash_read_buffer[3] = 0;
          
        }
        break;
      
      case FLASH_OP_READ_ID:
        {
          // Read received byte from RXDR
          flash_read_buffer[0] = *(volatile uint8_t*)&SPI4->RXDR;  // 8-bit read -> trash
          flash_read_buffer[1] = *(volatile uint8_t*)&SPI4->RXDR;  // 8-bit read
          flash_read_buffer[2] = *(volatile uint8_t*)&SPI4->RXDR;  // 8-bit read
          flash_read_buffer[3] = *(volatile uint8_t*)&SPI4->RXDR;  // 8-bit read
          
        }
          break;

      case FLASH_OP_READ_DATA:
        {
          // Read data sequence
          *(volatile uint8_t*)&SPI4->RXDR;       // Command byte
          *(volatile uint8_t*)&SPI4->RXDR;       // Address byte
          *(volatile uint8_t*)&SPI4->RXDR;       // Address byte
          *(volatile uint8_t*)&SPI4->RXDR;       // Address byte
          flash_read_buffer[0] = *(volatile uint8_t*)&SPI4->RXDR; // Data byte
          flash_read_buffer[1] = *(volatile uint8_t*)&SPI4->RXDR; // Data byte
          flash_read_buffer[2] = *(volatile uint8_t*)&SPI4->RXDR; // Data byte
          flash_read_buffer[3] = *(volatile uint8_t*)&SPI4->RXDR; // Data byte
          
        }
        break;
        
        case FLASH_OP_SECTOR_ERASE:
        {
          // Read data sequence
          *(volatile uint8_t*)&SPI4->RXDR;       // Command byte -> flush
          *(volatile uint8_t*)&SPI4->RXDR;       // Address byte -> flush
          *(volatile uint8_t*)&SPI4->RXDR;       // Address byte -> flush
          *(volatile uint8_t*)&SPI4->RXDR;       // Address byte -> flush
        }
        break;

        case FLASH_OP_WRITE_ENABLE:
        {
          *(volatile uint8_t*)&SPI4->RXDR;       // Command byte -> flush
        }
        break;
        
        case FLASH_OP_PAGE_PROGRAM:
        {
          // Read data sequence

          while (SPI4->SR & (1U << 0)) {  // While RXP is set
                    *(volatile uint8_t*)&SPI4->RXDR; // Flush all data bytes
                }
        }

        default:
        // Clear any unexpected data
          while (SPI4->SR & (1U << 0)) {  // While RXP is set
                    *(volatile uint8_t*)&SPI4->RXDR;
                }
        break;
    }
    
    flash_operation_complete = true; 
    flash_operation_type = FLASH_OP_NONE;
   
    // Housekeeping
    SPI4->IFCR |= (1U << 3);  // Clear the EOT flag
    SPI4->IFCR |= (1U << 4);  // Clear TXTFL flag
    SPI4->CR1 &= ~(1U << 9);  // Clear CSTART bit for next transfer
    CS_HIGH();                // Only raise CS at the end
  }
}

