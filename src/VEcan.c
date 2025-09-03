#include "VEcan.h"
#include "can.h"


//variables for VE can for testing
uint16_t chargevoltage = 49100; //max charge voltage in mv
uint16_t chargecurrent = 30000; //max charge current in ma
uint16_t disvoltage = 42000; // max discharge voltage in mv
uint16_t discurrent = 30000; // max discharge current in ma
uint16_t SOH = 100; // SOH place holder

uint8_t splitter[2];




unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char bmsname[8] = {'C', 'F', 'P', ' ', 'B', 'M', 'S', ' '}; // Comprehensice Full Protection BMS
unsigned char bmsmanu[8] = {'C', '.', 'E', '.', ' ', 'E', 'N', 'G'};
unsigned char snPart1[8] = {'L', 'B', '0', 'A', '0', '1', '0', '0'};
unsigned char snPart2[8] = {'0', '2', '4', '5', '0', '0', '6', '8'};

int SOC =80;

double PackVoltage = 46.7;
float AvgTemperature = 20.5;
uint16_t currentact = 0;

CAN_TxBufferElement threefiftyone;
CAN_TxBufferElement threefiftyfive;
CAN_TxBufferElement threefiftysix;
CAN_TxBufferElement threefiftyA;
CAN_TxBufferElement threefiftyE;
CAN_TxBufferElement threefiftyF;
CAN_TxBufferElement threesixty;
CAN_TxBufferElement threeseventytwo;
CAN_TxBufferElement threeseventythree;
CAN_TxBufferElement threeseventyfour;
CAN_TxBufferElement threeseventyfive;
CAN_TxBufferElement threeseventysix;
CAN_TxBufferElement threeseventyseven;
CAN_TxBufferElement threeseventyeight;     // charged and discharged Energy
CAN_TxBufferElement threeseventynine;      // Installed total capacity
CAN_TxBufferElement threeeighty;           // snPart1
CAN_TxBufferElement threeeightyone;        // snPart2



void VECan_Init () {
    

    splitter[0] = (uint8_t)SOH & 0xff;
    splitter[1] = ((uint8_t)SOH >> 8);

    // Frame 0x380 Serial No. Part 1
    threeeighty.T0 = (0x380U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeeighty.T1 = 0; // No EFC, no FDF, no BRS, 
    threeeighty.T1 |= (8U << 16); // DLC = 8
    threeeighty.data[0] = 'L';
    threeeighty.data[1] = 'B';
    threeeighty.data[2] = '0';
    threeeighty.data[3] = snPart1[3];
    threeeighty.data[4] = snPart1[4];
    threeeighty.data[5] = snPart1[5];
    threeeighty.data[6] = snPart1[6];
    threeeighty.data[7] = snPart1[7];

    // Frame 0x381 Serial No. Part 2
    threeeightyone.T0 = (0x381U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeeightyone.T1 = 0; // No EFC, no FDF, no BRS, 
    threeeightyone.T1 |= (8U << 16); // DLC = 8
    threeeightyone.data[0] = snPart2[0];
    threeeightyone.data[1] = snPart2[1];
    threeeightyone.data[2] = snPart2[2];
    threeeightyone.data[3] = snPart2[3];
    threeeightyone.data[4] = snPart2[4];
    threeeightyone.data[5] = snPart2[5];
    threeeightyone.data[6] = snPart2[6];
    threeeightyone.data[7] = snPart2[7];

    // Frame 0x379 Installed total capacity
    threeseventynine.T0 = (0x379U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventynine.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventynine.T1 |= (8U << 16); // DLC = 8
    threeseventynine.data[0] = 0x30;        //  LSB of 0x0230 = 560d
    threeseventynine.data[1] = 0x02;        //  MSB

    // Frame 0x378 charged and discharged energy 
    threeseventyeight.T0 = (0x378U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventyeight.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventyeight.T1 |= (8U << 16); // DLC = 8
    threeseventyeight.data[0] = 0xA8;   // example 69kw/h -> 0x02A8 = 690
    threeseventyeight.data[1] = 0x02;
    threeseventyeight.data[2] = 0x00;
    threeseventyeight.data[3] = 0x00;
    threeseventyeight.data[4] = 0xB2;
    threeseventyeight.data[5] = 0x02;
    threeseventyeight.data[6] = 0x00;
    threeseventyeight.data[7] = 0x00;

    // Frame 0x377 Cell Name with Maximum Cell Temperature
    threeseventyseven.T0 = (0x377U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventyseven.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventyseven.T1 |= (8U << 16); // DLC = 8
    threeseventyseven.data[0] = '0';   // 0301 Cell 03 Bank 01
    threeseventyseven.data[1] = '3';
    threeseventyseven.data[2] = '0';
    threeseventyseven.data[3] = '1';
    threeseventyseven.data[4] = 0x00;
    threeseventyseven.data[5] = 0x00;
    threeseventyseven.data[6] = 0x00;
    threeseventyseven.data[7] = 0x00;
    
    // Frame 0x376 Cell Name with Minimum Cell Temperature
    threeseventysix.T0 = (0x376U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventysix.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventysix.T1 |= (8U << 16); // DLC = 8
    threeseventysix.data[0] = '1';   // 1601 Cell 03 Bank 01
    threeseventysix.data[1] = '6';
    threeseventysix.data[2] = '0';
    threeseventysix.data[3] = '1';
    threeseventysix.data[4] = 0x00;
    threeseventysix.data[5] = 0x00;
    threeseventysix.data[6] = 0x00;
    threeseventysix.data[7] = 0x00;

    // Frame 0x375 Cell Name with MAX Cell Voltage
    threeseventyfive.T0 = (0x375U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventyfive.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventyfive.T1 |= (8U << 16); // DLC = 8
    threeseventyfive.data[0] = '1';   // 1201 Cell 03 Bank 01
    threeseventyfive.data[1] = '2';
    threeseventyfive.data[2] = '0';
    threeseventyfive.data[3] = '1';
    threeseventyfive.data[4] = 0x00;
    threeseventyfive.data[5] = 0x00;
    threeseventyfive.data[6] = 0x00;
    threeseventyfive.data[7] = 0x00;

    // Frame 0x374 Cell Name with MIN Cell Voltage
    threeseventyfour.T0 = (0x374U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventyfour.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventyfour.T1 |= (8U << 16); // DLC = 8
    threeseventyfour.data[0] = '0';   // 0701 Cell 03 Bank 01
    threeseventyfour.data[1] = '7';
    threeseventyfour.data[2] = '0';
    threeseventyfour.data[3] = '1';
    threeseventyfour.data[4] = 0x00;
    threeseventyfour.data[5] = 0x00;
    threeseventyfour.data[6] = 0x00;
    threeseventyfour.data[7] = 0x00;

    // Frame 0x373 Cell Info: Voltage and Temperature
    threeseventythree.T0 = (0x373U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventythree.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventythree.T1 |= (8U << 16); // DLC = 8
    threeseventythree.data[0] = 0x01;   // LSB lowest cell voltage
    threeseventythree.data[1] = 0x0D;   // MSB
    threeseventythree.data[2] = 0x08;   // LSB highest cell voltage
    threeseventythree.data[3] = 0x0D;   // MSB
    threeseventythree.data[4] = 0x28;   // LSB lowest cell temperature
    threeseventythree.data[5] = 0x01;   // MSB
    threeseventythree.data[6] = 0x29;   // LSB highest cell temperature
    threeseventythree.data[7] = 0x01;   // MSB


    // Frame 0x372 Battery Bank Info
    threeseventytwo.T0 = (0x372U << 18) | (0x0U << 29) | (0x0U << 30); 
    threeseventytwo.T1 = 0; // No EFC, no FDF, no BRS, 
    threeseventytwo.T1 |= (8U << 16); // DLC = 8
    threeseventytwo.data[0] = 0x02;   // LSB no. Batteries online
    threeseventytwo.data[1] = 0x00;   // MSB
    threeseventytwo.data[2] = 0x01;   // LSB no. Batteries blocked charge
    threeseventytwo.data[3] = 0x00;   // MSB
    threeseventytwo.data[4] = 0x01;   // LSB no. Batteries blocked discharge
    threeseventytwo.data[5] = 0x00;   // MSB
    threeseventytwo.data[6] = 0x02;   // LSB no. Batteries offline
    threeseventytwo.data[7] = 0x00;   // MSB

    // Frame 0x360 Battery Bank Info
    threesixty.T0 = (0x360U << 18) | (0x0U << 29) | (0x0U << 30); 
    threesixty.T1 = 0; // No EFC, no FDF, no BRS, 
    threesixty.T1 |= (8U << 16); // DLC = 8
    threesixty.data[0] = 0x00;   // unknown

    // Frame 0x35F Battery Info Firmware & Capacity available
    threefiftyF.T0 = (0x35FU << 18) | (0x0U << 29) | (0x0U << 30); 
    threefiftyF.T1 = 0; // No EFC, no FDF, no BRS, 
    threefiftyF.T1 |= (8U << 16); // DLC = 8 
    threefiftyF.data[0] = 0x01;   // firmware version high
    threefiftyF.data[1] = 0x00;   // firmware version low
    threefiftyF.data[2] = 0x6E;   // hardware version high
    threefiftyF.data[3] = 0x01;   // hardware version low
    threefiftyF.data[4] = 0x32;   // LSB Battery capacity available
    threefiftyF.data[5] = 0x00;   // MSB
    threefiftyF.data[6] = 0x00;   // unknown
    threefiftyF.data[7] = 0x00;   // 

    // Frame 0x35E Manufacturer Info / Name
    threefiftyE.T0 = (0x35EU << 18) | (0x0U << 29) | (0x0U << 30); 
    threefiftyE.T1 = 0; // No EFC, no FDF, no BRS, 
    threefiftyE.T1 |= (8U << 16); // DLC = 8 
    threefiftyE.data[0] = 'C';   
    threefiftyE.data[1] = '.';   
    threefiftyE.data[2] = 'E';   
    threefiftyE.data[3] = '.';   
    threefiftyE.data[4] = ' ';   
    threefiftyE.data[5] = 'E';   
    threefiftyE.data[6] = 'N';   
    threefiftyE.data[7] = 'G';   

    // Frame 0x35A Alarms / Warnings
    threefiftyA.T0 = (0x35AU << 18) | (0x0U << 29) | (0x0U << 30); 
    threefiftyA.T1 = 0; // No EFC, no FDF, no BRS, 
    threefiftyA.T1 |= (8U << 16); // DLC = 8 
    threefiftyA.data[0] = 0x00;
    threefiftyA.data[1] = 0x00;  
    threefiftyA.data[2] = 0x00;    
    threefiftyA.data[3] = 0x00;   
    threefiftyA.data[4] = 0x00;  
    threefiftyA.data[5] = 0x00;    
    threefiftyA.data[6] = 0x00;    
    threefiftyA.data[7] = 0x00;    

    // Frame 0x356 Battery Status: Voltage, Current, Temperature
    threefiftysix.T0 = (0x356U << 18) | (0x0U << 29) | (0x0U << 30); 
    threefiftysix.T1 = 0; // No EFC, no FDF, no BRS, 
    threefiftysix.T1 |= (8U << 16); // DLC = 8 
    threefiftysix.data[0] = 0x8E;   // LSB voltage in Millivolt
    threefiftysix.data[1] = 0x14;   // MSB
    threefiftysix.data[2] = 0xF9;   // LSB (A/10) DeciAmp, signed!
    threefiftysix.data[3] = 0xFF;   // MSB
    threefiftysix.data[4] = 0xB4;   // LSB (T/10) DeciTemp, signed! 
    threefiftysix.data[5] = 0x00;   // MSB
    threefiftysix.data[6] = 0x00;    
    threefiftysix.data[7] = 0x00; 
    
    // Frame 0x355 Battery State Info SOC & SOH
    threefiftyfive.T0 = (0x355U << 18) | (0x0U << 29) | (0x0U << 30); 
    threefiftyfive.T1 = 0; // No EFC, no FDF, no BRS, 
    threefiftyfive.T1 |= (8U << 16); // DLC = 8 
    threefiftyfive.data[0] = 0x33;   // LSB SOC (%)
    threefiftyfive.data[1] = 0x00;   // MSB
    threefiftyfive.data[2] = (uint8_t)SOH & 0xff;   // LSB SOH (%)
    threefiftyfive.data[3] = ((uint8_t)SOH >> 8);   // MSB
  

    // Frame 0x351 DVCC: CVL, CCL, DCL, DVL
    threefiftyone.T0 = (0x351U << 18) | (0x0U << 29) | (0x0U << 30); 
    threefiftyone.T1 = 0; // No EFC, no FDF, no BRS, 
    threefiftyone.T1 |= (8U << 16); // DLC = 8 
    threefiftyone.data[0] = 0x38;   // LSB (V/10) DeciVolt CVL 
    threefiftyone.data[1] = 0x20;   // MSB
    threefiftyone.data[2] = 0xE8;   // LSB (A/10) DeciAmp CCL 
    threefiftyone.data[3] = 0x03;   // MSB
    threefiftyone.data[4] = 0xE8;   // LSB (A/10) DeciAmp DCL
    threefiftyone.data[5] = 0x03;   // MSB
    threefiftyone.data[6] = 0xC7;   // LSB (V/10) DeciVolt DVL
    threefiftyone.data[7] = 0x01;   // MSB




}

void VECan_send (void) {
    FDCAN2_transmit_message(&threeeighty);
    FDCAN2_transmit_message(&threeeightyone);
    FDCAN2_transmit_message(&threeseventynine);
    FDCAN2_transmit_message(&threeseventyeight);
    FDCAN2_transmit_message(&threeseventyseven);
    FDCAN2_transmit_message(&threeseventysix);
    FDCAN2_transmit_message(&threeseventyfive);
    FDCAN2_transmit_message(&threeseventyfour);
    FDCAN2_transmit_message(&threeseventythree);
    FDCAN2_transmit_message(&threeseventytwo);
    FDCAN2_transmit_message(&threesixty);
    FDCAN2_transmit_message(&threefiftyF);
    FDCAN2_transmit_message(&threefiftyE);
    FDCAN2_transmit_message(&threefiftyA);
    FDCAN2_transmit_message(&threefiftysix);
    FDCAN2_transmit_message(&threefiftyfive);
    FDCAN2_transmit_message(&threefiftyone);
}
