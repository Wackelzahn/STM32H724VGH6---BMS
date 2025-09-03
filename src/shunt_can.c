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

double convert_0x042D_to_energy (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the energy value from the first eight bytes
    uint64_t energy = ((uint64_t)data[7] | ((uint64_t)data[6] << 8) | ((uint64_t)data[5] << 16) | ((uint64_t)data[4] << 24) |
                          ((uint64_t)data[3] << 32) | ((uint64_t)data[2] << 40) | ((uint64_t)data[1] << 48) | ((uint64_t)data[0] << 56));

    double energy_J = ((double)(energy) / 3200) * 3.2 * 16; // Convert See Datasheet INA228 (Energy[J] = CURRENT_LSB * ENERGY * 3.2 * 16 , Current_LSB = 1/3200 A

     return energy_J; // Energy in Joules
}

int64_t convert_0x042E_to_charge (uint8_t* data) {

    if (data == NULL) {
        return 0; // Handle null pointer
    }

    // Extract the charge value from the first eight bytes
    int64_t charge = ((uint64_t)data[7] | ((uint64_t)data[6] << 8) | ((uint64_t)data[5] << 16) | ((uint64_t)data[4] << 24) |
                          ((uint64_t)data[3] << 32) | ((uint64_t)data[2] << 40) | ((uint64_t)data[1] << 48) | ((uint64_t)data[0] << 56));

    int64_t charge_C = charge / 3200; // Convert See Datasheet INA228 (Charge[C] = CURRENT_LSB * CHARGE   , Current_LSB = 1/3200 A

     return charge_C; // Charge in 
}

// End of file