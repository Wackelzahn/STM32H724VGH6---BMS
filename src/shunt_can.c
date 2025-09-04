// Created: 2025-08-19 07:00:00
// Author: M. Schneider
// File: src/shunt_can.c
// -----------------------------------------------------
// Target Device: STM32H742VGH6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is the implementation file CAN bus interfacing 
// for the STM32G0B1 microcontroller.
// This source code is intended to create 
//      1. Initialization of the FDAN1 & FDCAN2 interface
//      2. Initialization of the GPIO
//      3. Setting up the Filter
//      4. FUNCTIONS FOR sending (receive is handled in ISR)
// -----------------------------------------------------

#include "shunt_can.h"

#define KEHRWERT_3200 5764607523034234ULL // 2^64 / 3200

int32_t convert_0x0426_to_milliampere(uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the current value from the first two bytes
    int32_t current_mA = data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);

     return current_mA; // Shunt current in milliamperes
}

int32_t convert_0x0427_to_millivolts(uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the voltage value from the first two bytes
    int32_t voltage_mV = data[1] | (data[0] << 8);

     return voltage_mV; // Bus Voltgae in millivolts
}

int32_t convert_0x0428_to_temperature (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the temperature value from the first two bytes
    int32_t temperature_C = data[1] | (data[0] << 8);

     return temperature_C;  // Die Temperature in degrees Celsius 1/100
}

int32_t convert_0x0429_to_microvolts (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the voltage value from the first four bytes
    int32_t voltage_uV = data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);

     return voltage_uV; // Shunt Voltage in microvolts
}

double convert_0x042A_to_milliwatts (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the power value from the first four bytes
    int32_t power = data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);

    double power_W = ((double)(power) / 3200) * 3.2; // Convert See Datasheet INA228 (Power[W] = CURRENT_LSB * POWER * 3.2 // Current_LSB = 1/3200 A

     return power_W; // Power in Watts
}

uint16_t convert_0x042B_to_DieID (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the Die ID value from the first four bytes
    uint16_t die_id = data[1] | (data[0] << 8);

     return die_id; // Die ID
}

uint16_t convert_0x042C_to_ManufacturerID (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the Die ID value from the first four bytes
    uint16_t Man_id = data[1] | (data[0] << 8);

     return Man_id; // Manufacturer ID
}

float convert_0x042D_to_energy (uint8_t* data) {

    if (data == NULL) {
        return 0.0f; // Handle null pointer
    }

    // Constructing the energy value from the 8-byte CAN array
    // Valuable information is 40bit. To construct a 32bit value for most efficient calculations
    // and safe processor resources, the values are shifted and combined accordingly.
    // by shifting the 40 valuable bist by 8 to the right we introduce a little error which is neglectable for our application
    // by shiftingh we divide the value by 256 which needs to be corrected later once we cast the uint32 result into float 
    uint32_t energy = ((uint32_t)data[6] | (data[5] << 8) | (data[4] << 16) | (data[3] << 24));

    // energy_J = (energy) / 3200) * 3.2 * 16; // Convert See Datasheet INA228   
    // avoid division for efficciency and avboud using int64 and float64
    // energy has already been divided by 256 by shifting above
    // we need to correct by multiplying with 256/3200 * 3.2 * 16 = 4.096
    // however, max conversion error of ~4 Joules is neglectable for our application        
 
    float energy_J = (float)energy * 4.096f;    // Multiply by 4.096

    return energy_J; // Energy in Joules
}

float convert_0x042E_to_charge (uint8_t* data) {

    if (data == NULL) {
        return 0.0f; // Handle null pointer
    }

    // Constructing the charge value from the 8-byte CAN array
    // Valuable information is 40bit. To construct a 32bit value for efficient calculations
    // the values are shifted and combined accordingly.
    // by shifting the 40 valuable bist by 8 to the right we introduce a little error which is neglectable for our application
    // by shiftingh we divide the value by 256 which needs to be corrected later once we cast the uint32 result into float 
    uint32_t charge = ((uint32_t)data[6] | (data[5] << 8) | (data[4] << 16) | (data[3] << 24));

    // charge_C = (charge) / 3200; // Convert See Datasheet INA228
    // avoid division for efficciency
    // charge has already been divided by 256 by shifting above
    // error max ~1 Coulomb is neglectable for our application
    // now only to correct by multiplying with 256/3200 = 0.08

    float charge_C = (float)charge * 0.08f;  // Multiply by (1/3200)*256 = 0.08

    // can charge be negative, so we need to handle that ??? to check!!

     return charge_C; // Charge in 
}

// End of file