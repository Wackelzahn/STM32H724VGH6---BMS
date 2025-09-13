
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

// LTC6813 specific settings (supports up to 18 cells)
#define LTC6813_SPI_MODE        3           // CPOL=1, CPHA=1
#define LTC6813_MAX_FREQ_HZ     1000000     // 1 MHz max SPI frequency
#define LTC6813_MAX_CELLS       18          // Maximum number of cells

// SPI CR1 bits
#define SPI_CR1_SPE             (1U << 0)   // SPI Enable
#define SPI_CR1_MASRX           (1U << 8)   // Master automatic SUSP in Receive mode
#define SPI_CR1_CSTART          (1U << 9)   // Master transfer start
#define SPI_CR1_CSUSP           (1U << 10)  // Master SUSPend request
#define SPI_CR1_HDDIR           (1U << 11)  // Half-duplex direction
#define SPI_CR1_SSI             (1U << 12)  // Internal slave select

// SPI CFG1 bits
#define SPI_CFG1_DSIZE_8BIT     (7U << 0)   // Data size = 8 bits
#define SPI_CFG1_FTHLV_1DATA    (0U << 5)   // FIFO threshold = 1 data
#define SPI_CFG1_MBR_DIV2       (0U << 28)  // Baud rate = fPCLK/2
#define SPI_CFG1_MBR_DIV4       (1U << 28)  // Baud rate = fPCLK/4
#define SPI_CFG1_MBR_DIV8       (2U << 28)  // Baud rate = fPCLK/8
#define SPI_CFG1_MBR_DIV16      (3U << 28)  // Baud rate = fPCLK/16
#define SPI_CFG1_MBR_DIV32      (4U << 28)  // Baud rate = fPCLK/32
#define SPI_CFG1_MBR_DIV64      (5U << 28)  // Baud rate = fPCLK/64
#define SPI_CFG1_MBR_DIV128     (6U << 28)  // Baud rate = fPCLK/128
#define SPI_CFG1_MBR_DIV256     (7U << 28)  // Baud rate = fPCLK/256

// SPI CFG2 bits
#define SPI_CFG2_MASTER         (1U << 22)  // Master mode
#define SPI_CFG2_COMM_FULL      (0U << 17)  // Full-duplex
#define SPI_CFG2_SSM            (1U << 26)  // Software slave management
#define SPI_CFG2_CPOL           (1U << 25)  // Clock polarity
#define SPI_CFG2_CPHA           (1U << 24)  // Clock phase
#define SPI_CFG2_LSBFRST        (0U << 23)  // MSB first
#define SPI_CFG2_SSOM           (1U << 30)  // SS output management
#define SPI_CFG2_SSOE           (1U << 29)  // SS output enable
#define SPI_CFG2_AFCNTR         (1U << 31)  // Alternate function control

// CS pin control macros
#define CS3_HIGH()   (GPIOA->BSRR = (1U << 15))
#define CS3_LOW()    (GPIOA->BSRR = (1U << (15 + 16)))


// Initialize SPI3 for communication with LTC6813
//  * Pin configuration (custom mapping):   PA15 - SPI3_NSS  (Output) - Chip Select
//                                          PB4  - SPI3_MISO (AF6) - Master In Slave Out
//                                          PC10 - SPI3_SCK  (AF6) - Serial Clock
//                                          PD6  - SPI3_MOSI (AF5) - Master Out Slave In
// 
void SPI3_LTC6813_Init(void)
{
    // 1. Enable clocks for GPIOA, GPIOB, GPIOC, GPIOD and SPI3
    RCC->AHB4ENR |= (1U << 0) | (1U << 1) | (1U << 2) | (1U << 3);  // Enable GPIOA, B, C, D
    RCC->APB1LENR |= (1U << 15);     // Enable SPI3
    for(volatile int i = 0; i < 100; i++);  // Small delay for clock stabilization

    // 2. Reset SPI3 peripheral
    RCC->APB1LRSTR |= (1U << 15);   // Assert reset
    for(volatile int i = 0; i < 10; i++);
    RCC->APB1LRSTR &= ~(1U << 15);  // Release reset

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
    GPIOD->AFR[0] |= ((5U << 24));  // Set AF5 for PD6

    
    
    // 4. Configure SPI3 for LTC6831
    // Disable SPI3 before configuration
    SPI3->CR1 = 0; // Clear everything, SPE = 0

    // Configure SPI3_CR1
    SPI3->CR1 = (1U << 12);     // SSI = 1 (Internal slave select)

    // Configure SPI3_CFG1
    SPI3->CFG1 |= SPI_CFG1_DSIZE_8BIT;      // 8-bit data size
    SPI3->CFG1 |= SPI_CFG1_FTHLV_1DATA;     // FIFO threshold = 1 data
    SPI3->CFG1 |= SPI_CFG1_MBR_DIV256;      // Set baud rate to ~781 kHz (100MHz/128)
    //SPI3->CFG1 |= (3U << 8);  // FTHLV=4 (011b) - adjust to 4U<<8 for 8 if needed

    // Configure SPI3_CFG2

    
    SPI3->CFG2 = (0U << 17)    // COMM = 0 (Full duplex)
               | (1U << 22)    // MASTER = 1 (Master mode)
               | (1U << 25)    // CPOL = 1 (Mode 3)
               | (1U << 24)    // CPHA = 1 (Mode 3)
               | (0U << 29)    // SSOE = 0 
               | (1U << 26)    // SSM = 1 
               | (1U << 31);   // AFCNTR = 1



    // Configure SPI3_CR2 (set TSIZE if needed for automatic EOT generation)


    SPI3->CR2 = 12U ;  // set to 12 byte initially

    // Clear any pending flags
    SPI3->IFCR = 0xFFFFFFFF;
    
    // 5. Enable SPI3
    SPI3->CR1 |= (1U << 0);  // Enable SPI

    CS3_HIGH();  // Ensure CS is high

}

// Transmit and receive a single byte via SPI3
uint8_t SPI3_TransmitReceiveByte(uint8_t tx_data)
{
    // Wait until TXE flag is set (transmit buffer empty)
    while(!(SPI3->SR & (1 << 1)));
    
    // Send data (cast to uint8_t pointer for byte access)
    *((volatile uint8_t*)&SPI3->TXDR) = tx_data;
    
    // Start transfer
    SPI3->CR1 |= SPI_CR1_CSTART;
    
    // Wait until RXNE flag is set (receive buffer not empty)
    uint32_t timeout = 10000;
    while(!(SPI3->SR & (1 << 0)) && --timeout);
    if(timeout == 0) {
        // Timeout occurred
        return 0xFF;  // Return dummy value on timeout
    }
    
    // Read received data (cast to uint8_t pointer for byte access)
    return *((volatile uint8_t*)&SPI3->RXDR);
}


// Transmit multiple bytes to LTC6831
void SPI3_TransmitReceive(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t length)
{
    CS3_LOW();  // Assert CS
    
    for(uint16_t i = 0; i < length; i++) {
        uint8_t rx_byte = SPI3_TransmitReceiveByte(tx_buffer[i]);
        if(rx_buffer != NULL) {
            rx_buffer[i] = rx_byte;
        }
    }
    
    CS3_HIGH();  // Deassert CS
}


// Simple delay function (blocking)
void delay_ms(uint32_t ms)
{
    // This is a rough approximation, adjust based on your system clock
    for(uint32_t i = 0; i < ms; i++) {
        for(volatile uint32_t j = 0; j < 20000; j++);
    }
}

// Example LTC6831 command: Wake up the IC
void LTC6813_Wakeup(void)
{
    // Wake-up sequence (CS low for >300μs)
    CS3_LOW();
    delay_ms(1);  // Hold CS low for >300μs
    CS3_HIGH();
    delay_ms(1);  // Allow time to stabilize
}

// LTC6813 Command codes (supports 18 cells)
#define CMD_ADCV            0x0260  // Start cell voltage ADC conversion
#define CMD_ADCV_MODE_7K    CMD_ADCV  // 7kHz mode (fastest)
#define CMD_ADCV_MODE_26HZ  0x0360  // 26Hz mode (filtered)
#define CMD_ADCV_MODE_2K    0x0370  // 2kHz mode
#define CMD_RDCVA           0x0004  // Read cell voltage register group A (cells 1-3)
#define CMD_RDCVB           0x0006  // Read cell voltage register group B (cells 4-6)
#define CMD_RDCVC           0x0008  // Read cell voltage register group C (cells 7-9)
#define CMD_RDCVD           0x000A  // Read cell voltage register group D (cells 10-12)
#define CMD_RDCVE           0x0009  // Read cell voltage register group E (cells 13-15)
#define CMD_RDCVF           0x000B  // Read cell voltage register group F (cells 16-18)

// PEC (Packet Error Code) calculation table
static const uint16_t pec15Table[256] = {
    0x0000, 0xC599, 0xCEAB, 0x0B32, 0xD8CF, 0x1D56, 0x1664, 0xD3FD,
    0xF407, 0x319E, 0x3AAC, 0xFF35, 0x2CC8, 0xE951, 0xE263, 0x27FA,
    0xAD97, 0x680E, 0x633C, 0xA6A5, 0x7558, 0xB0C1, 0xBBF3, 0x7E6A,
    0x5990, 0x9C09, 0x973B, 0x52A2, 0x815F, 0x44C6, 0x4FF4, 0x8A6D,
    0x5B2E, 0x9EB7, 0x9585, 0x501C, 0x83E1, 0x4678, 0x4D4A, 0x88D3,
    0xAF29, 0x6AB0, 0x6182, 0xA41B, 0x77E6, 0xB27F, 0xB94D, 0x7CD4,
    0xF6B9, 0x3320, 0x3812, 0xFD8B, 0x2E76, 0xEBEF, 0xE0DD, 0x2544,
    0x02BE, 0xC727, 0xCC15, 0x098C, 0xDA71, 0x1FE8, 0x14DA, 0xD143,
    0xF3C5, 0x365C, 0x3D6E, 0xF8F7, 0x2B0A, 0xEE93, 0xE5A1, 0x2038,
    0x07C2, 0xC25B, 0xC969, 0x0CF0, 0xDF0D, 0x1A94, 0x11A6, 0xD43F,
    0x5E52, 0x9BCB, 0x90F9, 0x5560, 0x869D, 0x4304, 0x4836, 0x8DAF,
    0xAA55, 0x6FCC, 0x64FE, 0xA167, 0x729A, 0xB703, 0xBC31, 0x79A8,
    0xA8EB, 0x6D72, 0x6640, 0xA3D9, 0x7024, 0xB5BD, 0xBE8F, 0x7B16,
    0x5CEC, 0x9975, 0x9247, 0x57DE, 0x8423, 0x41BA, 0x4A88, 0x8F11,
    0x057C, 0xC0E5, 0xCBD7, 0x0E4E, 0xDDB3, 0x182A, 0x1318, 0xD681,
    0xF17B, 0x34E2, 0x3FD0, 0xFA49, 0x29B4, 0xEC2D, 0xE71F, 0x2286,
    0xA213, 0x678A, 0x6CB8, 0xA921, 0x7ADC, 0xBF45, 0xB477, 0x71EE,
    0x5614, 0x938D, 0x98BF, 0x5D26, 0x8EDB, 0x4B42, 0x4070, 0x85E9,
    0x0F84, 0xCA1D, 0xC12F, 0x04B6, 0xD74B, 0x12D2, 0x19E0, 0xDC79,
    0xFB83, 0x3E1A, 0x3528, 0xF0B1, 0x234C, 0xE6D5, 0xEDE7, 0x287E,
    0xF93D, 0x3CA4, 0x3796, 0xF20F, 0x21F2, 0xE46B, 0xEF59, 0x2AC0,
    0x0D3A, 0xC8A3, 0xC391, 0x0608, 0xD5F5, 0x106C, 0x1B5E, 0xDEC7,
    0x54AA, 0x9133, 0x9A01, 0x5F98, 0x8C65, 0x49FC, 0x42CE, 0x8757,
    0xA0AD, 0x6534, 0x6E06, 0xAB9F, 0x7862, 0xBDFB, 0xB6C9, 0x7350,
    0x51D6, 0x944F, 0x9F7D, 0x5AE4, 0x8919, 0x4C80, 0x47B2, 0x822B,
    0xA5D1, 0x6048, 0x6B7A, 0xAEE3, 0x7D1E, 0xB887, 0xB3B5, 0x762C,
    0xFC41, 0x39D8, 0x32EA, 0xF773, 0x248E, 0xE117, 0xEA25, 0x2FBC,
    0x0846, 0xCDDF, 0xC6ED, 0x0374, 0xD089, 0x1510, 0x1E22, 0xDBBB,
    0x0AF8, 0xCF61, 0xC453, 0x01CA, 0xD237, 0x17AE, 0x1C9C, 0xD905,
    0xFEFF, 0x3B66, 0x3054, 0xF5CD, 0x2630, 0xE3A9, 0xE89B, 0x2D02,
    0xA76F, 0x62F6, 0x69C4, 0xAC5D, 0x7FA0, 0xBA39, 0xB10B, 0x7492,
    0x5368, 0x96F1, 0x9DC3, 0x585A, 0x8BA7, 0x4E3E, 0x450C, 0x8095
};




// Calculate PEC15 for LTC6831 communication
uint16_t LTC6813_CalculatePEC(uint8_t *data, uint8_t len)
{
    uint16_t remainder = 16;  // PEC seed
    
    for(uint8_t i = 0; i < len; i++) {
        uint8_t address = (uint8_t)(((remainder >> 7) ^ data[i]) & 0xFF);
        remainder = (remainder << 8) ^ pec15Table[address];
    }
    
    return (remainder * 2);  // The PEC15 has a 0 in the LSB so multiply by 2
}


// Send command to LTC6831 with PEC
void LTC6813_SendCommand(uint16_t cmd)
{
    uint8_t cmd_data[4];
    uint16_t cmd_pec;
    
    // Prepare command bytes
    cmd_data[0] = (uint8_t)((cmd >> 8) & 0xFF);
    cmd_data[1] = (uint8_t)(cmd & 0xFF);
    
    // Calculate PEC for command
    cmd_pec = LTC6813_CalculatePEC(cmd_data, 2);
    cmd_data[2] = (uint8_t)((cmd_pec >> 8) & 0xFF);
    cmd_data[3] = (uint8_t)(cmd_pec & 0xFF);
    
    // Send command with PEC
    SPI3_TransmitReceive(cmd_data, NULL, 4);
}

// Read register group from LTC6831
uint8_t LTC6813_ReadRegisterGroup(uint16_t cmd, uint8_t *data)
{
    uint8_t rx_buffer[8];
    uint16_t received_pec, calculated_pec;
    
    // Send read command
    LTC6813_SendCommand(cmd);
    
    // Read 8 bytes (6 data bytes + 2 PEC bytes)
    CS3_LOW();
    for(uint8_t i = 0; i < 8; i++) {
        rx_buffer[i] = SPI3_TransmitReceiveByte(0xFF);  // Send dummy bytes to receive
    }
    CS3_HIGH();
    
    // Extract received PEC
    received_pec = (uint16_t)((rx_buffer[6] << 8) | rx_buffer[7]);
    
    // Calculate PEC for received data
    calculated_pec = LTC6813_CalculatePEC(rx_buffer, 6);
    
    // Copy data if PEC check passes
    if(received_pec == calculated_pec) {
        for(uint8_t i = 0; i < 6; i++) {
            data[i] = rx_buffer[i];
        }
        return 1;  // PEC check passed
    }
    
    return 0;  // PEC check failed
}

// Start cell voltage ADC conversion
void LTC6813_StartCellVoltageConversion(uint16_t mode)
{
    LTC6813_Wakeup();
    LTC6813_SendCommand(mode);
    
    // Wait for conversion to complete
    // 7kHz mode: ~200us per cell group
    // 26Hz mode: ~200ms per cell group  
    // 2kHz mode: ~1ms per cell group
    if(mode == CMD_ADCV_MODE_26HZ) {
        delay_ms(250);  // Conservative delay for filtered mode
    } else {
        delay_ms(3);    // Conservative delay for fast modes
    }
}


// Read all cell voltages from LTC6813 (18 cells)
uint8_t LTC6813_ReadAllCellVoltages(uint32_t *cell_voltages)
{
    uint8_t reg_data[6];
    uint8_t success = 1;
    
    // Start ADC conversion (using 7kHz mode for speed)
    LTC6813_StartCellVoltageConversion(CMD_ADCV_MODE_7K);
    
    // Read Cell Voltage Register Group A (Cells 1-3)
    if(LTC6813_ReadRegisterGroup(CMD_RDCVA, reg_data)) {
        cell_voltages[0] = reg_data[0] | (reg_data[1] << 8);
        cell_voltages[1] = reg_data[2] | (reg_data[3] << 8);
        cell_voltages[2] = reg_data[4] | (reg_data[5] << 8);
    } else {
        success = 0;
    }
    
    // Read Cell Voltage Register Group B (Cells 4-6)
    if(LTC6813_ReadRegisterGroup(CMD_RDCVB, reg_data)) {
        cell_voltages[3] = reg_data[0] | (reg_data[1] << 8);
        cell_voltages[4] = reg_data[2] | (reg_data[3] << 8);
        cell_voltages[5] = reg_data[4] | (reg_data[5] << 8);
    } else {
        success = 0;
    }
    
    // Read Cell Voltage Register Group C (Cells 7-9)
    if(LTC6813_ReadRegisterGroup(CMD_RDCVC, reg_data)) {
        cell_voltages[6] = reg_data[0] | (reg_data[1] << 8);
        cell_voltages[7] = reg_data[2] | (reg_data[3] << 8);
        cell_voltages[8] = reg_data[4] | (reg_data[5] << 8);
    } else {
        success = 0;
    }
    
    // Read Cell Voltage Register Group D (Cells 10-12)
    if(LTC6813_ReadRegisterGroup(CMD_RDCVD, reg_data)) {
        cell_voltages[9] = reg_data[0] | (reg_data[1] << 8);
        cell_voltages[10] = reg_data[2] | (reg_data[3] << 8);
        cell_voltages[11] = reg_data[4] | (reg_data[5] << 8);
    } else {
        success = 0;
    }
    
    // Read Cell Voltage Register Group E (Cells 13-15)
    if(LTC6813_ReadRegisterGroup(CMD_RDCVE, reg_data)) {
        cell_voltages[12] = reg_data[0] | (reg_data[1] << 8);
        cell_voltages[13] = reg_data[2] | (reg_data[3] << 8);
        cell_voltages[14] = reg_data[4] | (reg_data[5] << 8);
    } else {
        success = 0;
    }
    
    // Read Cell Voltage Register Group F (Cells 16-18)
    if(LTC6813_ReadRegisterGroup(CMD_RDCVF, reg_data)) {
        cell_voltages[15] = reg_data[0] | (reg_data[1] << 8);
        cell_voltages[16] = reg_data[2] | (reg_data[3] << 8);
        cell_voltages[17] = reg_data[4] | (reg_data[5] << 8);
    } else {
        success = 0;
    }
    
    return success;
}


// Convert raw voltage reading to millivolts
float LTC6813_ConvertToMillivolts(uint32_t raw_voltage)
{
    // LTC6813 returns voltage in 100uV units
    // So multiply by 0.1 to get millivolts
    return (float)raw_voltage * 0.1f;
}

// Even simpler - just try to read configuration register
bool LTC6813_SimpleTest(void)
{
    uint8_t cmd_data[4];
    uint8_t rx_data[8];
    uint16_t cmd_pec;
    
    // Wake up
    CS3_LOW();
    SPI3_TransmitReceiveByte(0xFF);
    CS3_HIGH();
    delay_ms(1);
    
    // Send RDCFGA command (0x0002) with PEC
    uint16_t cmd = 0x0002;  // Read Configuration Register A
    cmd_data[0] = (uint8_t)(cmd >> 8);
    cmd_data[1] = (uint8_t)(cmd & 0xFF);
    
    // Calculate PEC
    cmd_pec = LTC6813_CalculatePEC(cmd_data, 2);
    cmd_data[2] = (uint8_t)(cmd_pec >> 8);
    cmd_data[3] = (uint8_t)(cmd_pec & 0xFF);
    
    // Send command
    CS3_LOW();
    for(int i = 0; i < 4; i++) {
        SPI3_TransmitReceiveByte(cmd_data[i]);
    }
    CS3_HIGH();
    
    delay_ms(1);
    
    // Read response (6 data bytes + 2 PEC bytes)
    CS3_LOW();
    for(int i = 0; i < 8; i++) {
        rx_data[i] = SPI3_TransmitReceiveByte(0xFF);
    }
    CS3_HIGH();
    
    // Check if we got any non-zero response
    uint8_t got_response = 0;
    for(int i = 0; i < 8; i++) {
        if(rx_data[i] != 0x00 && rx_data[i] != 0xFF) {
            got_response = 1;
            break;
        }
    }
    
    return got_response;
}


// Test function 1: Send 4-byte command and wait for EOT
uint8_t test_command_transmit(uint16_t command)
{
    uint8_t cmd_data[4];
    uint16_t cmd_pec;
    
    // Prepare command bytes
    cmd_data[0] = (uint8_t)(command >> 8);
    cmd_data[1] = (uint8_t)(command & 0xFF);
    
    // Calculate PEC for command
    cmd_pec = LTC6813_CalculatePEC(cmd_data, 2);
    cmd_data[2] = (uint8_t)(cmd_pec >> 8);
    cmd_data[3] = (uint8_t)(cmd_pec & 0xFF);
    
    // Send 4-byte command
    CS3_LOW();
    
    // Set TSIZE for 4 bytes
    SPI3->CR2 = 4;
    
    // Start transfer
    SPI3->CR1 |= SPI_CR1_CSTART;
    
    // Send all 4 bytes
    for(int i = 0; i < 4; i++) {
        // Wait for TXP (TX buffer has space)
        while(!(SPI3->SR & (1 << 1)));
        
        // Write byte to TX FIFO
        *((volatile uint8_t*)&SPI3->TXDR) = cmd_data[i];
    }
    
    // Wait for EOT (End of Transfer) - bit 3
    while(!(SPI3->SR & (1 << 3)));
    
    // Clear EOT flag
    SPI3->IFCR = (1 << 3);
    
    CS3_HIGH();
    
    return 1;  // Command sent successfully
}

// Test function 2: Receive 8-byte register response
uint8_t test_register(uint16_t command)
{
    uint8_t rx_buffer[8];
    uint8_t cmd_data[4];
    uint16_t cmd_pec;

    // Small delay between command and read
    for(volatile int i = 0; i < 10000; i++);
    delay_ms(1000);

    // Wake-up sequence (CS low >240 µs)
    CS3_LOW();
    for(volatile int i = 0; i < 10000; i++);  // ~1 ms (adjust for your sysclk)
    CS3_HIGH();
    for(volatile int i = 0; i < 1000; i++);   // ~100 µs settle


    // Prepare command bytes
    cmd_data[0] = (uint8_t)(command >> 8);
    cmd_data[1] = (uint8_t)(command & 0xFF);
    
    // Calculate PEC for command
    cmd_pec = LTC6813_CalculatePEC(cmd_data, 2);
    cmd_data[2] = (uint8_t)(cmd_pec >> 8);
    cmd_data[3] = (uint8_t)(cmd_pec & 0xFF);


    

 CS3_LOW();
    
    // Reset SPI state
    SPI3->CR1 &= ~((1U << 0) | (1U << 9));
    SPI3->IFCR = 0xFFFFFFFF;
    SPI3->CR2 = 12;  // Command + 3 address bytes + 4 data bytes
    SPI3->CR1 |= (1U << 0);
    
    // Wait for TXP
    while (!(SPI3->SR & (1U << 1)));
    

       
    // Send bytes
    *(volatile uint8_t*)&SPI3->TXDR = cmd_data[0]; 
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = cmd_data[1]; 
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = cmd_data[2]; 
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = cmd_data[3]; 
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
     while (!(SPI3->SR & (1U << 1)));
    *(volatile uint8_t*)&SPI3->TXDR = 0xFF; // Dummy byte
      
    // Start the transaction
    SPI3->CR1 |= (1U << 9); // CSTART = 1  

    // Wait for EOT (End of Transfer)
    while(!(SPI3->SR & (1 << 3)));
        

    // Read received byte
    *(volatile uint8_t*)&SPI3->RXDR;  // Discard command byte
    *(volatile uint8_t*)&SPI3->RXDR;  // Discard command byte
    *(volatile uint8_t*)&SPI3->RXDR;  // Discard command byte
    *(volatile uint8_t*)&SPI3->RXDR;  // Discard command byte
    rx_buffer[0] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[1] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[2] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[3] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[4] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[5] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[6] = *((volatile uint8_t*)&SPI3->RXDR);
    rx_buffer[7] = *((volatile uint8_t*)&SPI3->RXDR);

    // Housekeeping
    SPI3->IFCR |= (1U << 3);  // Clear the EOT flag
    SPI3->IFCR |= (1U << 4);  // Clear TXTFL flag
    SPI3->CR1 &= ~(1U << 9);  // Clear CSTART bit for next transfer
    CS3_HIGH();                // Only raise CS at the end

    
    
  
    
    // Verify PEC
    uint16_t received_pec = (uint16_t)((rx_buffer[6] << 8) | rx_buffer[7]);
    uint16_t calculated_pec = LTC6813_CalculatePEC(rx_buffer, 6);
    
    if(received_pec == calculated_pec) {
        // Copy the 6 data bytes if PEC is good
        //for(int i = 0; i < 6; i++) {
        //    data_out[i] = rx_buffer[i];
       // }
        return 1;  // Success - PEC check passed
    }
    
    return 0;  // PEC check failed
}

