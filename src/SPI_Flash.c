

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
static void delay_us(uint32_t us)
{
    // Rough delay - adjust based on your system clock
    // Assuming 400MHz system clock
    volatile uint32_t count = us * 100;
    while(count--);
}



// Initialize GPIO and SPI4
flash_status_t mx25l_init(void)
{
    uint8_t id[3];
    
    // Enable clocks
    RCC->AHB4ENR |= (1U << 4);    // GPIOE clock
    RCC->APB2ENR |= (1U << 13);   // SPI4 clock
    
    // Configure GPIO pins
    // PE2 (SCK), PE5 (MISO), PE6 (MOSI) - Alternate Function
    GPIOE->MODER &= ~((3U << 4) | (3U << 10) | (3U << 12));     // Clear mode bits
    GPIOE->MODER |= (2U << 4) | (2U << 10) | (2U << 12);        // Alternate function mode
    
    // PE4 (CS) - GPIO Output
    GPIOE->MODER &= ~(3U << 8);   // Clear mode bits
    GPIOE->MODER |= (1U << 8);    // Output mode
    
    // Set speed to very high
    GPIOE->OSPEEDR |= (3U << 4) | (3U << 8) | (3U << 10) | (3U << 12);
    
    // No pull-up/pull-down for SPI pins, pull-up for CS
    GPIOE->PUPDR &= ~((3U << 4) | (3U << 8) | (3U << 10) | (3U << 12));
    GPIOE->PUPDR |= (1 << 8);    // Pull-up for CS
    
    // Set alternate function (AF5 for SPI4)
    GPIOE->AFR[0] &= ~((0xFU << 8) | (0xFU << 20) | (0xFU << 24));  // Clear AF bits
    GPIOE->AFR[0] |= (5U << 8) | (5U << 20) | (5U << 24);           // AF5
    
    // Set CS high initially
    CS_HIGH();
    
    // Configure SPI4
    // Disable SPI first
    SPI4->CR1 &= ~(1U << 0);  // SPE = 0
    
    // CFG1: Data size = 8 bits, baudrate prescaler = 8
    SPI4->CFG1 = (7U << 0) |      // DSIZE = 7 (8 bits)
                (3U << 28);      // MBR = 3 (divide by 16)
    
    // CFG2: Master mode, full duplex, MSB first
    SPI4->CFG2 = (1U << 22) |     // MASTER = 1
                (0 << 17) |     // COMM = 0 (full duplex)
                (0 << 31);      // LSBFRST = 0 (MSB first)
    
    // CR1: Enable SPI
    SPI4->CR1 = (1U << 0);        // SPE = 1
    
    // Start communication
    SPI4->CR1 |= (1U << 9);       // CSTART = 1
    
    delay_us(100);
    
    // Test communication by reading ID
    if (mx25l_read_id(id) == FLASH_OK)
    {
        // Expected ID: 0xC2, 0x20, 0x18
        if (id[0] == 0xC2 && id[1] == 0x20 && id[2] == 0x18)
        {
            return FLASH_OK;
        }
    }
    
    return FLASH_ERROR;
}

// SPI byte transfer
static uint8_t spi_transfer(uint8_t data)
{
    // Wait for TX buffer empty
    while(!(SPI4->SR & (1U << 1))); // TXP bit
    
    // Send data
    *(volatile uint8_t*)&SPI4->TXDR = data;
    
    // Wait for RX buffer not empty
    while(!(SPI4->SR & (1U << 0))); // RXP bit
    
    // Read received data
    return *(volatile uint8_t*)&SPI4->RXDR;
}

// Read Flash ID
flash_status_t mx25l_read_id(uint8_t *id)
{
    CS_LOW();
    
    spi_transfer(MX25L_CMD_RDID);
    id[0] = spi_transfer(0x00);
    id[1] = spi_transfer(0x00);
    id[2] = spi_transfer(0x00);
    
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
    status = spi_transfer(0x00);
    CS_HIGH();
    
    return status;
}

// Wait for write operation to complete
flash_status_t mx25l_wait_ready(void)
{
    uint32_t timeout = 1000000;
    
    while ((mx25l_read_status() & MX25L_SR_WIP) && timeout--)
    {
        delay_us(1);
    }
    
    return (timeout > 0) ? FLASH_OK : FLASH_TIMEOUT;
}

// Erase a 4KB sector
flash_status_t mx25l_sector_erase(uint32_t address)
{
    if (mx25l_write_enable() != FLASH_OK)
        return FLASH_ERROR;
    
    CS_LOW();
    spi_transfer(MX25L_CMD_SE);
    spi_transfer((address >> 16) & 0xFF);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);
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
    spi_transfer((address >> 16) & 0xFF);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);
    
    for (uint16_t i = 0; i < size; i++)
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
    spi_transfer((address >> 16) & 0xFF);
    spi_transfer((address >> 8) & 0xFF);
    spi_transfer(address & 0xFF);
    
    for (uint32_t i = 0; i < size; i++)
    {
        buffer[i] = spi_transfer(0x00);
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
    uint32_t end_sector = (address + size - 1) / MX25L_SECTOR_SIZE;
    
    // Erase required sectors
    for (uint32_t sector = start_sector; sector <= end_sector; sector++)
    {
        if (mx25l_sector_erase(sector * MX25L_SECTOR_SIZE) != FLASH_OK)
        {
            return FLASH_ERROR;
        }
    }
    
    // Write data page by page
    while (remaining > 0)
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