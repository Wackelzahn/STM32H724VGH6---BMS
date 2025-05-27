#ifndef INA228_H
#define INA228_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include "I2C.h"


#define INA228_ADDR 0x40U


bool INA228_ReadRegister(uint8_t reg, uint8_t *data, uint8_t len);
bool INA228_WriteRegister(uint8_t reg, uint16_t value);

// Initialize INA228
// Set Shunt voltage Range to +/-40.96mV
// Set Shunt_Cal (0.000250 Ohm)
bool INA228_Init(void);


// Set Shunt Tempco
// The 16 bit register provides a resolution of 1ppm/°C/LSB
bool INA228_SetShuntTempco(uint16_t tempco);
// Read VBUS voltage
// VBUS result in mV
bool INA228_ReadVBUS(uint16_t *vbus);
// Read Temperature
// Temperature result in 0.01 degree Celsius
bool INA228_ReadTemp(int16_t *temp);
// Read Current
// Current result in mA
bool INA228_ReadCurr(int32_t *curr);
// Read Shunt voltage
// Shunt voltage result in uV
bool INA228_ReadShuntV(int32_t *shuntV);
// Read Power
// Power result in mW
bool INA228_ReadPower(uint32_t *powermW);
//Read Energy
// Energy result in Joule
bool INA228_ReadEnergy(uint64_t *energy);
// Read Manufacturer ID
bool INA228_ReadManufacturerID(uint16_t *manufID);
// Read Die ID
bool INA228_ReadDieID(uint16_t *dieID);
// Read charge in Coulombs
bool INA228_ReadCharge(int64_t *charge);



#endif