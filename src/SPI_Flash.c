

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


    // Set transfer size to 1 byte in CR2 (TSIZE[15:0])
    SPI4->CR2 = 1U; // TSIZE = 1 (single byte)

    // Pull NSS low (PE4) to select slave
    GPIOE->ODR &= ~(1U << 4);

    // Load data into TXDR
    //*(volatile uint8_t*)&SPI4->TXDR = data;
    SPI4->TXDR = data;
SPI4->TXDR = 0x55; // Test write
SPI4->TXDR = 0xFF; // Test write
    // Small delay to ensure TXDR is processed
    //for (volatile uint32_t i = 0; i < 10; i++) __asm volatile ("nop");

    // Start the transaction
    SPI4->CR1 |= (1U << 9); // CSTART = 1

    // Wait for receive data to be available (RXNE, bit 0)
    while (!(SPI4->SR & (1U << 0)));

    // Read received data
    uint8_t received = *(volatile uint8_t*)&SPI4->RXDR;

    // Wait for end of transfer (EOT, bit 3)
    while (!(SPI4->SR & (1U << 3)));

    // Clear the EOT flag
    SPI4->IFCR |= (1U << 3);

    // Pull NSS high (PE4) to deselect slave
    GPIOE->ODR |= (1U << 4);

    return received;
}

// Initialize GPIO and SPI4
flash_status_t mx25l_init(void)
{
    uint8_t id[3];
    
    // Enable clocks
    RCC->AHB4ENR |= (1U << 4);    // GPIOE clock
    RCC->APB2ENR |= (1U << 13);   // SPI4 clock
    
    // Also ensure SPI4 is taken out of reset (if needed)
    RCC->APB2RSTR |= (1U << 13);   // Reset SPI4
    delay_us(1U);
    RCC->APB2RSTR &= ~(1U << 13);  // Release SPI4 from reset
    
    // Longer delay to ensure clocks are stable
    delay_us(100U);
    
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
    
    // Configure SPI4 - OPTION 2: Aggressive MODF prevention
    // Make sure SPI is completely disabled first
    SPI4->CR1 = 0U;  // Clear all bits including SPE
    SPI4->CR2 = 0U;  // Clear CR2
    SPI4->CFG1 = 0U; // Clear CFG1
    SPI4->CFG2 = 0U; // Clear CFG2
    
// Configure SPI4 CFG1 (baud rate, data size, FIFO threshold)
    SPI4->CFG1 = 0x00000000;    // Clear all bits first
    SPI4->CFG1 = (3U << 28)     // MBR[2:0] = 011 (fPCLK/16)
                | (15U);         // DSIZE[4:0] = 01111 (8 bits)

    // Configure SPI4 CR1 with SSI = 1 for software NSS
    SPI4->CR1 = (1U << 12);    // SSI = 1 (Internal slave select)

    // Configure SPI4 CFG2 (master mode, Mode 0, enable SSM)
    SPI4->CFG2 = (1U << 22)    // MASTER = 1 (Master mode)
                | (0U << 25)    // CPOL = 0 (low idle, Mode 0)
                | (0U << 24)    // CPHA = 0 (first clock, Mode 0)
                | (0U << 29)    // SSOE = 0 (Disable NSS output)
                | (1U << 26);   // SSM = 1 (Enable software slave management)

    // Clear MODF flag after configuration using IFCR
    SPI4->IFCR = (1U << 5); // Write 1 to MODFC to clear MODF

    // Small delay to stabilize configuration
    for (volatile uint32_t i = 0; i < 100; i++) __asm volatile ("nop");

    // Enable SPI4 (SPE = 1)
    SPI4->CR1 |= (1U << 0);

    // Clear MODF flag after SPI enable using IFCR
    SPI4->IFCR = (1U << 5); // Write 1 to MODFC to clear MODF
    
    
    delay_us(100U);
    
    // Test communication by reading ID
    if (mx25l_read_id(id) == FLASH_OK)
    {
        // Expected ID: 0xC2, 0x20, 0x18
        if (id[0] == 0xC2U && id[1] == 0x20U && id[2] == 0x18U)
        {
            return FLASH_OK;
        }
    }
    
    return FLASH_ERROR;
}

// Read Flash ID
flash_status_t mx25l_read_id(uint8_t *id)
{
    CS_LOW();
    
    spi_transfer(MX25L_CMD_RDID);
    id[0] = spi_transfer(0x00U);
    id[1] = spi_transfer(0x00U);
    id[2] = spi_transfer(0x00U);
    
    CS_HIGH();
    
    return FLASH_OK;
}

// Enable write operations
flash_status_t mx25l_write_enable(void)
{
    CS_LOW();
    spi_transfer(MX25L_CMD_WREN);
    CS_HIGH();
    
    return FLASH_OK;
}

// Read status register
uint8_t mx25l_read_status(void)
{
    uint8_t status;
    
    CS_LOW();
    spi_transfer(MX25L_CMD_RDSR);
    status = spi_transfer(0x00U);
    CS_HIGH();
    
    return status;
}

// Wait for write operation to complete
flash_status_t mx25l_wait_ready(void)
{
    uint32_t timeout = 1000000U;
    
    while ((mx25l_read_status() & MX25L_SR_WIP) && timeout--)
    {
        delay_us(1U);
    }
    
    return (timeout > 0U) ? FLASH_OK : FLASH_TIMEOUT;
}

// Erase a 4KB sector
flash_status_t mx25l_sector_erase(uint32_t address)
{
    if (mx25l_write_enable() != FLASH_OK)
        return FLASH_ERROR;
    
    CS_LOW();
    spi_transfer(MX25L_CMD_SE);
    spi_transfer((address >> 16) & 0xFFU);
    spi_transfer((address >> 8) & 0xFFU);
    spi_transfer(address & 0xFFU);
    CS_HIGH();
    
    return mx25l_wait_ready();
}

// Program a page (up to 256 bytes)
flash_status_t mx25l_page_program(uint32_t address, uint8_t *data, uint16_t size)
{
    if (size > MX25L_PAGE_SIZE)
        size = MX25L_PAGE_SIZE;
    
    if (mx25l_write_enable() != FLASH_OK)
        return FLASH_ERROR;
    
    CS_LOW();
    spi_transfer(MX25L_CMD_PP);
    spi_transfer((address >> 16) & 0xFFU);
    spi_transfer((address >> 8) & 0xFFU);
    spi_transfer(address & 0xFFU);
    
    for (uint16_t i = 0U; i < size; i++)
    {
        spi_transfer(data[i]);
    }
    
    CS_HIGH();
    
    return mx25l_wait_ready();
}

// Read data from flash
flash_status_t mx25l_read_data(uint32_t address, uint8_t *buffer, uint32_t size)
{
    CS_LOW();
    spi_transfer(MX25L_CMD_READ);
    spi_transfer((address >> 16) & 0xFFU);
    spi_transfer((address >> 8) & 0xFFU);
    spi_transfer(address & 0xFFU);
    
    for (uint32_t i = 0U; i < size; i++)
    {
        buffer[i] = spi_transfer(0x00U);
    }
    
    CS_HIGH();
    
    return FLASH_OK;
}

// High-level function to write a variable
flash_status_t mx25l_write_variable(uint32_t address, void *data, uint32_t size)
{
    uint8_t *data_ptr = (uint8_t*)data;
    uint32_t remaining = size;
    uint32_t current_addr = address;
    
    // Calculate sectors to erase
    uint32_t start_sector = address / MX25L_SECTOR_SIZE;
    uint32_t end_sector = (address + size - 1U) / MX25L_SECTOR_SIZE;
    
    // Erase required sectors
    for (uint32_t sector = start_sector; sector <= end_sector; sector++)
    {
        if (mx25l_sector_erase(sector * MX25L_SECTOR_SIZE) != FLASH_OK)
        {
            return FLASH_ERROR;
        }
    }
    
    // Write data page by page
    while (remaining > 0U)
    {
        uint32_t page_offset = current_addr % MX25L_PAGE_SIZE;
        uint32_t write_size = MX25L_PAGE_SIZE - page_offset;
        
        if (write_size > remaining)
            write_size = remaining;
        
        if (mx25l_page_program(current_addr, data_ptr, (uint16_t)write_size) != FLASH_OK)
        {
            return FLASH_ERROR;
        }
        
        data_ptr += write_size;
        current_addr += write_size;
        remaining -= write_size;
    }
    
    return FLASH_OK;
}

// High-level function to read a variable
flash_status_t mx25l_read_variable(uint32_t address, void *buffer, uint32_t size)
{
    return mx25l_read_data(address, (uint8_t*)buffer, size);
}