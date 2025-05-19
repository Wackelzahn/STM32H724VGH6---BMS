#include "I2C.h"


// Define I2C timing for 100 KHz with 16 MHz SYSCLK 
#define I2C_TIMING_100KHZ  0x00303D5B



uint32_t timeout = 10000;



/*   I2C1 Bare Metal Initialization Function
     This function initializes I2C1 for communication with a slave device.
     It configures the GPIO pins for I2C1_SCL (PA9) and I2C1_SDA (PA10), sets the clock speed,
     and enables the I2C1 peripheral.
     -----------------------------------------
     Parameters:  None
     Returns:     0=fail, 1=success    */


     bool I2C1_Init(void) {
        // 1. enable Clocks
        RCC->IOPENR |= (1U << 0); // GPIOA clock enable
        RCC->APBENR1 |= (1U << 21); // I2C1 clock enable
    
        // 2. configure GPIO pins PA9 (SCL) and PA10 (SDA) for I2C1
        GPIOA->MODER &= ~((3U << (9*2)) | (3U << (10*2)));  // Clear MODER9/10
        GPIOA->MODER |=  ((2U << (9*2)) | (2U << (10*2)));    // Set to Alternate Function
        GPIOA->AFR[1] &= ~((0xFU << ((9 - 8) * 4)) | (0xFU << ((10 - 8) * 4))); // Clear AF bits 
        GPIOA->AFR[1] |= ((6U << ((9 - 8) * 4)) | (6U << ((10 - 8) * 4)));      // Set AF6 for both pins 
        GPIOA->OTYPER |= ((1U << 9) | (1U << 10));            // Set OTYPER to open-drain for both pins
        GPIOA->PUPDR &= ~((3U << (9*2)) | (3U << (10*2)));  // Clear PUPDR9/10
        GPIOA->PUPDR |=  ((1U << (9*2)) | (1U << (10*2)));    // Set pull-up for both pins
        GPIOA->OSPEEDR |= ((3U << (9*2)) | (3U << (10*2))); // Set high speed for both pins
       
        // 3. configure I2C1
        I2C1->CR1 &= ~1U; // Disable I2C1 before configuring
    
        while (I2C1->CR1 & 1U); // Wait until I2C1 is disabled
    
        I2C1->TIMINGR = I2C_TIMING_100KHZ; // Timing register value for 100kHz, 16Mhz
    
        I2C1->CR1 = 0x00; // Reset Value, all features disabled
        I2C1->CR2 = 0x00; // 7-bit addressing mode
    
        I2C1->CR1 |= 1U; // Enable I2C1
       
        // 4. Wait until I2C1 is ready
        timeout = 10000;
        while (!(I2C1->CR1 & 1U)) {  // Fixed missing parenthesis
            if (--timeout == 0) {
                return false; // Timeout occurred
            }
        }
        
        return true; // Initialization successful
    }



/*   Write data to a specific register in an I2C slave device
     -----------------------------------------
     Parameters:    slave_addr: I2C slave address
                    reg_addr: Register address to write to
                    data: pointer to the data to be written
                    len: number of data bytes to write
     Returns:       1=success, 0=fail    */

     bool I2C1_Write(uint8_t slave_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len) {
    
        // Wait until I2C is not busy
        timeout = 10000;
        while (I2C1->ISR & I2C_ISR_BUSY) {
            if (--timeout == 0) return 0;
        }
    
        // Validate NBYTES (must fit in 8-bit field, 0-255)
        if (len + 1 > 255) {
            return 0; // NBYTES overflow
        }

        // Configure transfer: slave address, number of bytes to send (including register address),
        // generate start, enable autoend
        I2C1->CR2 = (slave_addr << 1) | ((uint32_t)(len + 1) << 16) |
                I2C_CR2_START | I2C_CR2_AUTOEND;
        
        // Wait until TXIS (transmit interrupt status) is set (transmitter register empty)
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_TXIS)) {  // wait for TXIS bit to be set (transmit register empty)
            if (--timeout == 0) return 0; // Timeout occurred
            if (I2C1->ISR & I2C_ISR_NACKF) {    // Check for NACK (meaning slave not responding)
                I2C1->ICR |= I2C_ICR_NACKCF;    // Clear NACK flag
                return 0; // Error occurred slave not responding
            }
        }
    
        // Send register address
        I2C1->TXDR = reg_addr;
        
        // Send data bytes
        for (uint16_t i = 0; i < len; i++) {
            // Wait until TXIS flag is set
            timeout = 10000;
            while (!(I2C1->ISR & I2C_ISR_TXIS)) {
                if (--timeout == 0) return 0;
                if (I2C1->ISR & I2C_ISR_NACKF) {    // Check for NACK (meaining slave not responding)
                    I2C1->ICR |= I2C_ICR_NACKCF;    // Clear NACK flag
                    return 0; // Error occurred slave not responding
                }
            }
            // Send data byte 
            I2C1->TXDR = data[i];
        }
    
        // Wait until STOPF flag is set (stop generated)
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_STOPF)) {
            if (--timeout == 0) return 0;
        }
    
        // Clear STOPF flag
        I2C1->ICR |= I2C_ICR_STOPCF;
        return true;  // Success
    }


/*
   Read data from a specific register in an I2C slave device
   -----------------------------------------
   Parameters:    slave_addr: I2C slave address
                  reg_addr: Register address to read from
                  data: pointer to the buffer to store the read data
                  nbytes: number of data bytes to read
   Returns:       1=success, 0=fail 
*/
bool I2C1_ReadReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    
    // Wait until I2C is not busy 
    timeout = 10000;
    while (I2C1->ISR & I2C_ISR_BUSY) {
        if (--timeout == 0) return 0;
    }

    // Validate NBYTES
    if (len > 255) {
        return 0; // NBYTES overflow
    }

    // Configure transfer: slave address, 1 byte to send (register address), generate start 
    I2C1->CR2 = (slave_addr << 1) | (1U << 16) | I2C_CR2_START;

    // Wait until TXIS flag is set (transmit register empty) 
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TXIS)) {
        if (--timeout == 0) return 0;
        if (I2C1->ISR & I2C_ISR_NACKF) {    // Check for NACK (meaning slave not responding)
            I2C1->ICR |= I2C_ICR_NACKCF;    // Clear NACK flag
            return 0;   // Error occurred slave not responding
        }
    }

    // Send register address
    I2C1->TXDR = reg_addr;

    // Wait until TC flag is set (transfer complete) 
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_TC)) {
        if (--timeout == 0) return 0;
    }
    
    // Configure for receive: slave address, number of bytes to read, generate start, enable autoend
    I2C1->CR2 = ((uint32_t)(slave_addr << 1)) | ((uint32_t)(len << 16)) | I2C_CR2_START | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN;

    // Read data bytes
    for (uint16_t i = 0; i < len; i++) {
        timeout = 10000;
        while (!(I2C1->ISR & I2C_ISR_RXNE)) {   // Wait until RXNE flag is set (receive register not empty)
            if (--timeout == 0) return 0;
            
            // Check for NACK (good practice to check during receive as well)
            if (I2C1->ISR & I2C_ISR_NACKF) {
                I2C1->ICR |= I2C_ICR_NACKCF;    // Clear NACK flag
                return 0;   // Error occurred slave not responding
            }
        }
        data[i] = (uint8_t)(I2C1->RXDR); // Read data byte - fixed: moved inside the loop
    }

    // Wait until STOPF flag is set (stop generated) 
    timeout = 10000;
    while (!(I2C1->ISR & I2C_ISR_STOPF)) {
        if (--timeout == 0) return 0;
    }
    
    I2C1->ICR |= I2C_ICR_STOPCF; // Clear STOPF flag
    return true;  // Success
}

