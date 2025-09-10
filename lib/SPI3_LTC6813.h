#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include "registers.h"



// Function prototypes

void SPI3_LTC6831_Init(void);
uint8_t SPI3_TransmitReceiveByte(uint8_t tx_data);
void SPI3_TransmitReceive(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t length);
void delay_ms(uint32_t ms);
void LTC6831_Wakeup(void);
uint16_t LTC6831_CalculatePEC(uint8_t *data, uint8_t len);
void LTC6831_SendCommand(uint16_t cmd);
uint8_t LTC6831_ReadRegisterGroup(uint16_t cmd, uint8_t *data);
void LTC6831_StartCellVoltageConversion(uint16_t mode);



#endif