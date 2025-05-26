#include "INA228.h"


int32_t counter;



// Initialize the INA228
// Set Shunt voltage Range to +/-40.96mV
// Set Shunt_Cal (0.000250 Ohm)
// Set ADC sample averaging count to 16
bool INA228_Init(void) {
    // Set to bus voltage continuous mode
    
    I2C1_Init();
    uint16_t help = 0;

    // INA228 RESET
    if (INA228_WriteRegister(0x00, 0x8000)){  
    }
    else return false;   // failure

    // INA228 Set ADC sample averaging count to 16
    // Conversion time for temperature to 540μs, for shunt voltage to 4120μs and bus voltage to 540μs
    // Mode set to "continuous" bus, shunt and temperature measurement
    help = (0x2U << 0) | (0x4U << 3) | (0x7U << 6) | (0x4U << 9) | (0xFU << 12);
    if (INA228_WriteRegister(0x01, help)){    
    }
    else return false;   // failure
    
    // INA228 Set Shunt voltage Range to +/-40.96mV
    // equals max shunt current (250uOhm) at 163.84A
    // Enable Temperature Compensation 
    if (INA228_WriteRegister(0x00, 0b0000000000110000)){   
    }                                           
    else return false;   // failure
    
    // INA228 Set Shunt_Cal (0.000250 Ohm)
    // current LSB equals 163.84A / 2'19 = 312.5uA
    // 13107.2 x 10'6 x CURRENT_LSB x RSHUNT x 4 = 4096
    if (INA228_WriteRegister(0x02, 0x1000)){    
    }                                           
    else return false;   // failure   
    
    // INA228 Temperature coefficient
    // Set to 40 ppm/°C --> In the Range of appr. 25°C to 50°C
    // 0x0028U = 40 ppm/°C (according to Datasheet)
    if (INA228_SetShuntTempco(0x0028U)){    
    }                                           
    else return false;   // failure

    return true;
    }


bool INA228_ReadRegister(uint8_t reg, uint8_t *data, uint8_t len)
    {
            // Read the data from the register
            if (I2C1_ReadReg(INA228_ADDR, reg, data, len)) {
                return true;  // Success
            } else {
                return false; // Error occurred
            }
       
    }
    

bool INA228_WriteRegister(uint8_t reg, uint16_t value) 
    {
        uint8_t data[2];
        // Prepare the data to be sent
        data[0] = (value >> 8) & 0xFFU;   // High byte
        data[1] = value & 0xFFU;         // Low byte

    // Send the registe byte (register to be accessed)
    if (I2C1_Write(INA228_ADDR, reg, data, 2)) {
        return true;
    } else {
        return false;
    };
   
}


// Set Shunt Tempco
// The 16 bit register provides a resolution of 1ppm/°C/LSB
bool INA228_SetShuntTempco(uint16_t tempco) {
    // Set the shunt temperature compensation register
    if (INA228_WriteRegister(0x03, tempco)) {
        return true;
    } else {
        return false;
    }
}

// Read VBUS voltage
// VBUS = (VSHUNT * 0.00125) * 1310920 / 2^26
// 195.3125 μV/LSB
bool INA228_ReadVBUS(uint16_t *vbus) {
    uint8_t vbus_data[3];
    uint64_t product = 0;
    uint64_t factor = 204800;    // bingo!
    if (!(INA228_ReadRegister(0x05, vbus_data, 3))) return false;  
    uint32_t vbus_raw = ((uint32_t)vbus_data[0] << 12) | ((uint32_t)vbus_data[1] << 4) | (vbus_data[2] >> 4);
    
    product = factor * vbus_raw;   // be carefull. To cast to 64 bit during multiplication, facter needs to be 64 bit
    product = product >> 20;
    *vbus = (uint16_t)(product & 0xFFFF);

    return true;
}

// Read Current
// 312.5 μA/LSB
bool INA228_ReadCurr(int32_t *curr) {
    uint8_t curr_data[3];
    int32_t factor = 320;    // bingo!
    int32_t Current_mA = 0;
    if (!(INA228_ReadRegister(0x07, curr_data, 3))) return false;  
    uint32_t curr_raw = (curr_data[0] << 12) | (curr_data[1] << 4) | (curr_data[2] >> 4);
    // Sign extend if negative
    if (curr_raw & 0x80000U) curr_raw |= 0xFFF00000U;
    // Convert to mA
    Current_mA = (int32_t)curr_raw * factor; // typecast into signed int32_t 
    Current_mA = Current_mA >> 10;
    *curr = Current_mA;
    
    return true;
}

// Read Shunt voltage
// VSHUNT = (VSHUNT * LSB)  --> 78.125 nV/LSB when ADCRANGE = 1
bool INA228_ReadShuntV(int32_t *shuntV) {
    uint8_t shuntV_data[3];
    int32_t factor = 80;    // bingo!
    int64_t ShuntV_uV = 0;
    if (!(INA228_ReadRegister(0x04, shuntV_data, 3))) return false;  
    uint32_t shuntV_raw =  (shuntV_data[0] << 12) | (shuntV_data[1] << 4) | (shuntV_data[2] >> 4);
    // Sign extend if negative
    if (shuntV_raw & 0x80000U) shuntV_raw |= 0xFFF00000U;
    // Convert to uV
    ShuntV_uV = factor * (int32_t)shuntV_raw; // typecast into signed int32_t
    ShuntV_uV = ShuntV_uV >> 10;
    *shuntV = (int32_t)(ShuntV_uV);
    
    return true;
}

// Read Power
// Power register holds the value in [mW] for an current lsb of 0.0003125A
// The power value is unsigned!!!
bool INA228_ReadPower(uint32_t *powermW) {
    uint8_t power_data[3];
    if (!(INA228_ReadRegister(0x08, power_data, 3))) return false;  
    uint32_t power_raw = (power_data[0] << 16) | (power_data[1] << 8) | (power_data[2] << 0);
    *powermW = power_raw;
    
    return true;
}

// Read Energy
// Energy register holds the value in [Joule] for an current lsb of 0.0003125A
// The energy value is unsigned!!! 40bit
bool INA228_ReadEnergy(uint64_t *energy) {
    uint8_t energy_data[5];
    if (!(INA228_ReadRegister(0x09, energy_data, 3))) return false;  
    uint64_t energy_raw = ((uint64_t)energy_data[0] << 32) | ((uint64_t)energy_data[1] <<24) | ((uint64_t)energy_data[2] << 16) | ((uint64_t)energy_data[3] << 8) | ((uint64_t)energy_data[4] << 0);
    // energy_data is without explicitly casting, casted into int32. Here 
    // it is casted into int64_t. This is important to avoid overflow
    *energy = energy_raw;
    
    return true;
}

// Read Temperature
// returns signed int16_t in .xx [Celsius]
// 7.8125 m°C/LSB
bool INA228_ReadTemp(int16_t *temp) {  
    uint8_t temp_data[2];
    int32_t product = 0;
    int32_t factor = 800;    // bingo
    if (!(INA228_ReadRegister(0x06, temp_data, 2))) return false;  
    int16_t temp_raw = (temp_data[0] << 8) | (temp_data[1] << 0);
    
    product = factor * temp_raw;
    product = product >> 10;
    *temp = (int16_t)(product);
    
    return true;
}

// Read Manufacturer ID of the INA228
// Manufacturer ID is 0x5449
bool INA228_ReadManufacturerID(uint16_t *manufID) {
    uint8_t manufID_data[2];
    if (!(INA228_ReadRegister(0x3E, manufID_data, 2))) return false;  
    uint16_t manufID_raw = (manufID_data[0] << 8) | (manufID_data[1] << 0);
    *manufID = manufID_raw;
    
    return true;
}

// Read Device ID of the INA228
// Device ID is 0x228x -> last byte is revision ID
bool INA228_ReadDieID(uint16_t *devID) {
    uint8_t devID_data[2];
    if (!(INA228_ReadRegister(0x3F, devID_data, 2))) return false;  
    uint16_t devID_raw = (devID_data[0] << 8) | (devID_data[1] << 0);
    *devID = devID_raw;
    
    return true;
}


bool INA228_ReadCharge(int64_t *charge) {
    uint8_t charge_data[5];
    if (!(INA228_ReadRegister(0x0A, charge_data, 5))) return false;  
    int64_t charge_raw = ((int64_t)charge_data[0] << 32) | ((int64_t)charge_data[1] << 24) | ((int64_t)charge_data[2] << 16) | ((int64_t)charge_data[3] << 8) | ((int64_t)charge_data[4] << 0);
    *charge = charge_raw;
    
    return true;
}


