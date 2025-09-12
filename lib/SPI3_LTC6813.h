// Created: 2025-09-05 16:00:00
// Author: M. Schneider
// File: include/SPI3_LTC6813.h
// -----------------------------------------------------
// Target Device: STM32H742VGH6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is the header file 
// for LTC6813-1 Battery Management System
// -----------------------------------------------------


#ifndef SPI3_LTC6813_H
#define SPI3_LTC6813_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include "registers.h"



// Function prototypes

void SPI3_LTC6813_Init(void);
uint8_t SPI3_TransmitReceiveByte(uint8_t tx_data);
void SPI3_TransmitReceive(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t length);
void delay_ms(uint32_t ms);
void LTC6813_Wakeup(void);
uint16_t LTC6813_CalculatePEC(uint8_t *data, uint8_t len);
void LTC6831_SendCommand(uint16_t cmd);
uint8_t LTC6813_ReadRegisterGroup(uint16_t cmd, uint8_t *data);
void LTC6813_StartCellVoltageConversion(uint16_t mode);
uint8_t LTC6813_ReadAllCellVoltages(uint32_t *cell_voltages);
float LTC6813_ConvertToMillivolts(uint32_t raw_voltage);
bool LTC6813_SimpleTest(void);

uint8_t test_register(uint16_t command);
# endif // SPI3_LTC6813_H