#ifndef I2C_H
#define I2C_H

#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include "registers.h"

// I2C bit definitions (add to your header if not present)
#define I2C_CR2_START   (1U << 13)
#define I2C_CR2_AUTOEND (1U << 25)
#define I2C_CR2_STOP    (1U << 14)
#define I2C_CR2_NACK    (1U << 4)
#define I2C_ISR_BUSY    (1U << 15)
#define I2C_CR2_NBYTES_Pos  (1U << 16)
#define I2C_CR2_RD_WRN  (1U << 10)
#define I2C_ISR_TXIS    (1U << 1)
#define I2C_ISR_RXNE    (1U << 2)
#define I2C_ISR_TC      (1U << 6)
#define I2C_ISR_STOPF   (1U << 5)
#define I2C_ISR_NACKF   (1U << 4)
#define I2C_ICR_STOPCF  (1U << 5)
#define I2C_ICR_NACKCF  (1U << 4)

bool I2C1_Init(void);
bool I2C1_Write(uint8_t slave_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);
bool I2C1_ReadReg(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);


#endif