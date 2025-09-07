

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
    // Set transfer size to 1 byte (required for STM32H7)
    SPI4->CR2 = 1U;
    
    // Start the transaction
    SPI4->CR1 |= (1U << 9);       // CSTART = 1
    
    // Wait for TX buffer empty
    while(!(SPI4->SR & (1U << 1))); // TXP bit
    
    // Send data
    *(volatile uint8_t*)&SPI4->TXDR = data;
    
    // Wait for RX buffer not empty
    while(!(SPI4->SR & (1U << 3))); // EOT bit - CHANGED FROM RXP TO EOT
    
    // Read received data
    uint8_t received = *(volatile uint8_t*)&SPI4->RXDR;
    
    // Wait for end of transfer
    while(!(SPI4->SR & (1U << 3))); // EOT bit
    
    // Clear the EOT flag
    SPI4->IFCR |= (1U << 3);
    
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
    
// Configure PE2, PE5, PE6 as alternate function (MODER = 10)
    GPIOE->MODER &= ~((3U << (2 * 2)) | (3U << (2 * 5)) | (3U << (2 * 6)));
    GPIOE->MODER |= (2U << (2 * 2)) | (2U << (2 * 5)) | (2U << (2 * 6));

    // Configure PE4 as output for software NSS (MODER = 01)
    GPIOE->MODER &= ~(3U << (2 * 4));
    GPIOE->MODER |= (1U << (2 * 4));

    // Set alternate function to AF5 for PE2, PE5, PE6 (AFR[0])
    GPIOE->AFR[0] &= ~((15U << (4 * 2)) | (15U << (4 * 5)) | (15U << (4 * 6)));
    GPIOE->AFR[0] |= (5U << (4 * 2)) | (5U << (4 * 5)) | (5U << (4 * 6));

    // Set push-pull output type
    GPIOE->OTYPER &= ~((1U << 2) | (1U << 4) | (1U << 5) | (1U << 6));

    // Set no pull-up/pull-down
    GPIOE->PUPDR &= ~((3U << (2 * 2)) | (3U << (2 * 4)) | (3U << (2 * 5)) | (3U << (2 * 6)));

    // Set high speed for SPI pins, medium for NSS
    GPIOE->OSPEEDR &= ~((3U << (2 * 2)) | (3U << (2 * 4)) | (3U << (2 * 5)) | (3U << (2 * 6)));
    GPIOE->OSPEEDR |= (2U << (2 * 2)) | (1U << (2 * 4)) | (2U << (2 * 5)) | (2U << (2 * 6));

    // Set NSS (PE4) high initially
    GPIOE->ODR |= (1U << 4);

    

// Configure SPI4 CFG1 (baud rate, data size, FIFO threshold)
    SPI4->CFG1 = (3U << 28)    // MBR[2:0] = 011 (fPCLK/16)
                | (0U << 10)    // UDRDET = 0 (no underrun detection)
                | (0U << 9)     // UDRCFG[1:0] = 00 (default)
                | (0U << 8)     // FTHLV[2:0] = 000 (1-byte threshold)
                | (15U << 1);   // DSIZE[4:0] = 01111 (8 bits)

    // Configure SPI4 CFG2 (master mode, Mode 0, disable hardware NSS)
    SPI4->CFG2 = (1U << 22)    // MASTER = 1 (Master mode)
                | (0U << 25)    // CPOL = 0 (low idle, Mode 0)
                | (0U << 24)    // CPHA = 0 (first clock, Mode 0)
                | (0U << 29)    // SSOE = 0 (Disable NSS output)
                | (0U << 26);   // SSM = 0 (Disable slave select mode)

 // Clear MODF flag after CFG2 configuration using IFCR
    SPI4->IFCR = (1U << 9); // Write 1 to MODFC to clear MODF

    // Configure SPI4 CR1 (basic enable)
    SPI4->CR1 = 0; // Reset value

    // Enable SPI4 (SPE = 1)
    SPI4->CR1 |= (1U << 0);

    // Clear MODF flag after SPI enable using IFCR
    SPI4->IFCR = (1U << 9); // Write 1 to MODFC to clear MODF
    
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