#ifndef SHUNT_CAN_H
#define SHUNT_CAN_H

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

int32_t convert_0x0426_to_milliampere(uint8_t* data);
int32_t convert_0x0427_to_millivolts(uint8_t* data);
int32_t convert_0x0428_to_temperature (uint8_t* data);
int32_t convert_0x0429_to_microvolts (uint8_t* data);
float convert_0x042A_to_power (uint8_t* data);
uint16_t convert_0x042B_to_DieID (uint8_t* data);
uint16_t convert_0x042C_to_ManufacturerID (uint8_t* data);
float convert_0x042D_to_energy (uint8_t* data);
float convert_0x042E_to_charge (uint8_t* data);


#endif