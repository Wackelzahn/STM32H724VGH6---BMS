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

CAN_TX_FRAME threefiftyone;
CAN_TX_FRAME threefiftyfive;
CAN_TX_FRAME threefiftysix;
CAN_TX_FRAME threefiftyA;
CAN_TX_FRAME threefiftyE;
CAN_TX_FRAME threefiftyF;
CAN_TX_FRAME threesixty;
CAN_TX_FRAME threeseventytwo;
CAN_TX_FRAME threeseventythree;
CAN_TX_FRAME threeseventyfour;
CAN_TX_FRAME threeseventyfive;
CAN_TX_FRAME threeseventysix;
CAN_TX_FRAME threeseventyseven;
CAN_TX_FRAME threeseventyeight;     // charged and discharged Energy
CAN_TX_FRAME threeseventynine;      // Installed total capacity
CAN_TX_FRAME threeeighty;           // snPart1
CAN_TX_FRAME threeeightyone;        // snPart2



void VECan_Init () {
    

    splitter[0] = (uint8_t)SOH & 0xff;
    splitter[1] = ((uint8_t)SOH >> 8);

    // Frame 0x380 Serial No. Part 1
    threeeighty.identifier = 0x0380;
    threeeighty.length = 0x0008U;
    threeeighty.data[0] = 'L';
    threeeighty.data[1] = 'B';
    threeeighty.data[2] = '0';
    threeeighty.data[3] = snPart1[3];
    threeeighty.data[4] = snPart1[4];
    threeeighty.data[5] = snPart1[5];
    threeeighty.data[6] = snPart1[6];
    threeeighty.data[7] = snPart1[7];

    // Frame 0x381 Serial No. Part 2
    threeeightyone.identifier = 0x0381;
    threeeightyone.length = 0x0008U;
    threeeightyone.data[0] = snPart2[0];
    threeeightyone.data[1] = snPart2[1];
    threeeightyone.data[2] = snPart2[2];
    threeeightyone.data[3] = snPart2[3];
    threeeightyone.data[4] = snPart2[4];
    threeeightyone.data[5] = snPart2[5];
    threeeightyone.data[6] = snPart2[6];
    threeeightyone.data[7] = snPart2[7];

    // Frame 0x379 Installed total capacity
    threeseventynine.identifier = 0x0379;
    threeseventynine.length = 0x0002U;
    threeseventynine.data[0] = 0x30;        //  LSB of 0x0230 = 560d
    threeseventynine.data[1] = 0x02;        //  MSB

    // Frame 0x378 charged and discharged energy 
    threeseventyeight.identifier = 0x0378;
    threeseventyeight.length = 0x0008U;
    threeseventyeight.data[0] = 0xA8;   // example 69kw/h -> 0x02A8 = 690
    threeseventyeight.data[1] = 0x02;
    threeseventyeight.data[2] = 0x00;
    threeseventyeight.data[3] = 0x00;
    threeseventyeight.data[4] = 0xB2;
    threeseventyeight.data[5] = 0x02;
    threeseventyeight.data[6] = 0x00;
    threeseventyeight.data[7] = 0x00;

    // Frame 0x377 Cell Name with Maximum Cell Temperature
    threeseventyseven.identifier = 0x0377;
    threeseventyseven.length = 0x0008U;
    threeseventyseven.data[0] = '0';   // 0301 Cell 03 Bank 01
    threeseventyseven.data[1] = '3';
    threeseventyseven.data[2] = '0';
    threeseventyseven.data[3] = '1';
    threeseventyseven.data[4] = 0x00;
    threeseventyseven.data[5] = 0x00;
    threeseventyseven.data[6] = 0x00;
    threeseventyseven.data[7] = 0x00;
    
    // Frame 0x376 Cell Name with Minimum Cell Temperature
    threeseventysix.identifier = 0x0376;
    threeseventysix.length = 0x0008U;
    threeseventysix.data[0] = '1';   // 1601 Cell 03 Bank 01
    threeseventysix.data[1] = '6';
    threeseventysix.data[2] = '0';
    threeseventysix.data[3] = '1';
    threeseventysix.data[4] = 0x00;
    threeseventysix.data[5] = 0x00;
    threeseventysix.data[6] = 0x00;
    threeseventysix.data[7] = 0x00;

    // Frame 0x375 Cell Name with MAX Cell Voltage
    threeseventyfive.identifier = 0x0375;
    threeseventyfive.length = 0x0008U;
    threeseventyfive.data[0] = '1';   // 1201 Cell 03 Bank 01
    threeseventyfive.data[1] = '2';
    threeseventyfive.data[2] = '0';
    threeseventyfive.data[3] = '1';
    threeseventyfive.data[4] = 0x00;
    threeseventyfive.data[5] = 0x00;
    threeseventyfive.data[6] = 0x00;
    threeseventyfive.data[7] = 0x00;

    // Frame 0x374 Cell Name with MIN Cell Voltage
    threeseventyfour.identifier = 0x0374;
    threeseventyfour.length = 0x0008U;
    threeseventyfour.data[0] = '0';   // 0701 Cell 03 Bank 01
    threeseventyfour.data[1] = '7';
    threeseventyfour.data[2] = '0';
    threeseventyfour.data[3] = '1';
    threeseventyfour.data[4] = 0x00;
    threeseventyfour.data[5] = 0x00;
    threeseventyfour.data[6] = 0x00;
    threeseventyfour.data[7] = 0x00;

    // Frame 0x373 Cell Info: Voltage and Temperature
    threeseventythree.identifier = 0x0373;
    threeseventythree.length = 0x0008U; 
    threeseventythree.data[0] = 0x01;   // LSB lowest cell voltage
    threeseventythree.data[1] = 0x0D;   // MSB
    threeseventythree.data[2] = 0x08;   // LSB highest cell voltage
    threeseventythree.data[3] = 0x0D;   // MSB
    threeseventythree.data[4] = 0x28;   // LSB lowest cell temperature
    threeseventythree.data[5] = 0x01;   // MSB
    threeseventythree.data[6] = 0x29;   // LSB highest cell temperature
    threeseventythree.data[7] = 0x01;   // MSB


    // Frame 0x372 Battery Bank Info
    threeseventytwo.identifier = 0x0372;
    threeseventytwo.length = 0x0008U; 
    threeseventytwo.data[0] = 0x02;   // LSB no. Batteries online
    threeseventytwo.data[1] = 0x00;   // MSB
    threeseventytwo.data[2] = 0x01;   // LSB no. Batteries blocked charge
    threeseventytwo.data[3] = 0x00;   // MSB
    threeseventytwo.data[4] = 0x01;   // LSB no. Batteries blocked discharge
    threeseventytwo.data[5] = 0x00;   // MSB
    threeseventytwo.data[6] = 0x02;   // LSB no. Batteries offline
    threeseventytwo.data[7] = 0x00;   // MSB

    // Frame 0x360 Battery Bank Info
    threesixty.identifier = 0x0360;
    threesixty.length = 0x0001U; 
    threesixty.data[0] = 0x00;   // unknown

    // Frame 0x35F Battery Info Firmware & Capacity available
    threefiftyF.identifier = 0x035F;
    threefiftyF.length = 0x0008U; 
    threefiftyF.data[0] = 0x01;   // firmware version high
    threefiftyF.data[1] = 0x00;   // firmware version low
    threefiftyF.data[2] = 0x6E;   // hardware version high
    threefiftyF.data[3] = 0x01;   // hardware version low
    threefiftyF.data[4] = 0x32;   // LSB Battery capacity available
    threefiftyF.data[5] = 0x00;   // MSB
    threefiftyF.data[6] = 0x00;   // unknown
    threefiftyF.data[7] = 0x00;   // 

    // Frame 0x35E Manufacturer Info / Name
    threefiftyE.identifier = 0x035E;
    threefiftyE.length = 0x0008U; 
    threefiftyE.data[0] = 'C';   
    threefiftyE.data[1] = '.';   
    threefiftyE.data[2] = 'E';   
    threefiftyE.data[3] = '.';   
    threefiftyE.data[4] = ' ';   
    threefiftyE.data[5] = 'E';   
    threefiftyE.data[6] = 'N';   
    threefiftyE.data[7] = 'G';   

    // Frame 0x35A Alarms / Warnings
    threefiftyA.identifier = 0x035A;
    threefiftyA.length = 0x0008U; 
    threefiftyA.data[0] = 0x00;
    threefiftyA.data[1] = 0x00;  
    threefiftyA.data[2] = 0x00;    
    threefiftyA.data[3] = 0x00;   
    threefiftyA.data[4] = 0x00;  
    threefiftyA.data[5] = 0x00;    
    threefiftyA.data[6] = 0x00;    
    threefiftyA.data[7] = 0x00;    

    // Frame 0x356 Battery Status: Voltage, Current, Temperature
    threefiftysix.identifier = 0x0356;
    threefiftysix.length = 0x0008U; 
    threefiftysix.data[0] = 0x8E;   // LSB voltage in Millivolt
    threefiftysix.data[1] = 0x14;   // MSB
    threefiftysix.data[2] = 0xF9;   // LSB (A/10) DeciAmp, signed!
    threefiftysix.data[3] = 0xFF;   // MSB
    threefiftysix.data[4] = 0xB4;   // LSB (T/10) DeciTemp, signed! 
    threefiftysix.data[5] = 0x00;   // MSB
    threefiftysix.data[6] = 0x00;    
    threefiftysix.data[7] = 0x00; 
    
    // Frame 0x355 Battery State Info SOC & SOH
    threefiftyfive.identifier = 0x0355;
    threefiftyfive.length = 0x0004U; 
    threefiftyfive.data[0] = 0x33;   // LSB SOC (%)
    threefiftyfive.data[1] = 0x00;   // MSB
    threefiftyfive.data[2] = (uint8_t)SOH & 0xff;   // LSB SOH (%)
    threefiftyfive.data[3] = ((uint8_t)SOH >> 8);   // MSB
  

    // Frame 0x351 DVCC: CVL, CCL, DCL, DVL
    threefiftyone.identifier = 0x0351;
    threefiftyone.length = 0x0008U; 
    threefiftyone.data[0] = 0x38;   // LSB (V/10) DeciVolt CVL 
    threefiftyone.data[1] = 0x02;   // MSB
    threefiftyone.data[2] = 0xE8;   // LSB (A/10) DeciAmp CCL 
    threefiftyone.data[3] = 0x03;   // MSB
    threefiftyone.data[4] = 0xE8;   // LSB (A/10) DeciAmp DCL
    threefiftyone.data[5] = 0x03;   // MSB
    threefiftyone.data[6] = 0xC7;   // LSB (V/10) DeciVolt DVL
    threefiftyone.data[7] = 0x01;   // MSB




}

void VECan_send () {
    Can_SendMessage(CAN1, &threeeighty);
    Can_SendMessage(CAN1, &threeeightyone);
    Can_SendMessage(CAN1, &threeseventynine);
    Can_SendMessage(CAN1, &threeseventyeight);
    Can_SendMessage(CAN1, &threeseventyseven);
    Can_SendMessage(CAN1, &threeseventysix);
    Can_SendMessage(CAN1, &threeseventyfive);
    Can_SendMessage(CAN1, &threeseventyfour);
    Can_SendMessage(CAN1, &threeseventythree);
    Can_SendMessage(CAN1, &threeseventytwo);
    Can_SendMessage(CAN1, &threesixty);
    Can_SendMessage(CAN1, &threefiftyF);
    Can_SendMessage(CAN1, &threefiftyE);
    Can_SendMessage(CAN1, &threefiftyA);
    Can_SendMessage(CAN1, &threefiftysix);
    Can_SendMessage(CAN1, &threefiftyfive);
    Can_SendMessage(CAN1, &threefiftyone);
}
