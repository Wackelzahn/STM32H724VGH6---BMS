

// Created: 2025-03-06 16:00:00
// Author: Michael Schneider
// -----------------------------------------------------
// Target Device: STM32G0B1KEU6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is a the program for the STM32G0B1 acting as a
// Sensor Node (SN) to collect data from the INA228 and
// send it via CAN bus to a Processing Unit (PU).
// 
// The program is using
//    - an accurate base tick (10ms) (SYSTICK IRQ)
//    - GPIO output
//    - SPI communication
//    - CAN communication
// -----------------------------------------------------


#include "startup.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "registers.h"
//#include "serial.h"
//#include "conversion.h"
#include "can.h"
#include "INA228.h"


//------------------------------------------------------------
// Global variables
//------------------------------------------------------------


uint32_t INA228_Power_mW; 
uint64_t INA228_Energy_Joule;
uint16_t INA228_vbus, INA228_DieID, INA228_ManufacturerID, test;
int32_t INA228_Current_mA, INA228_ShuntVltg;
int16_t INA228_Temp;
int64_t INA228_Charge_Coulombs;

uint8_t msg_counter = 0;

uint32_t previous_tec_value = 0; // Previous value of the transmit error counter 

// use volatile variables to ensure they are not optimized out by the compiler
// and to ensure they are always read from memory. Use for variables that are used
// in interrupt handlers or memory mapped registers. Or are modified by threads  / tasks.
volatile uint32_t one_sec_tick = 0;
volatile uint8_t rx_flag = 0;
volatile uint8_t rx_received[8];
volatile uint8_t SensorNodeStatus[8];       // Status of the Sensor Node
volatile uint8_t ProcessingUnitStatus[8];   // Status of the processing unit
volatile uint8_t ProcessingUnitCommand[8];  // Command from the processing unit to the sensor node
volatile bool CAN_TxError = false;          // Error flag for CAN communication
volatile uint8_t x=0;                       // Counter for testing how many interrupts were fired    
volatile uint8_t y=0;

CAN_RxBufferElement rx_temp;
CAN_TxBufferElement TxFrames[10]; // Array to hold Tx frames

#define FREQ 16000000  // PCLK, internal clock by default, 16 Mhz
#define BIT(x) (1UL << (x))


//------------------------------------------------------------
// Init Functions
//------------------------------------------------------------

static inline void init_clock(void) {
  RCC->CR |= (1U << 8);              // HSI16 clock enable
  while (!(RCC->CR & (1U << 10)));   // HSI16 clock ready flag
  RCC->APBENR2 |= (1U << 0);  // Enable SYSCFG clock
}

static inline void systick_init() {
  uint32_t ticks = 160000; // 10ms tick at 16MHz, 160,000 Cycles
  if ((ticks - 1) > 0xffffff) return;     // Systick timer is 24 bit
  SYST->RVR = ticks - 1;
  SYST->CVR = 0;
  SYST->CSR = BIT(0) | BIT(1) | BIT(2);   // Enable systick
  RCC->APBENR2 |= BIT(0);                 // Enable SYSCFG
}

static inline void PA2_out_init() {
  RCC->IOPENR |= 1 << 0;             // Enable GPIOA clock
  GPIOA->MODER |= 1 << 4;             // Set PA2 to output mode
  GPIOA->ODR &= ~(1U << 2);          // Clear PA2
  GPIOA->ODR |= (1U << 2);             // Set PA2
}

static inline void PB4_out_init() {
     // Enable GPIOB clock
    RCC->IOPENR |= (1U << 1);  // Set bit 1 for GPIOBEN
    // Configure PB4 as general purpose output mode
    GPIOB->MODER &= ~(3U << (4 * 2));     // Clear mode bits for pin 4 (2 bits per pin)
    GPIOB->MODER |= (1U << (4 * 2));      // Set as output (01) for pin 4
    // Configure PB4 as open drain
    GPIOB->OTYPER |= (1U << 4);  // Set open drain (1) for pin 4
    // Configure speed (optional, medium speed used here)
    GPIOB->OSPEEDR &= ~(3U << (4 * 2));   // Clear speed bits for pin 4
    GPIOB->OSPEEDR |= (1U << (4 * 2));    // Set as medium speed (01) for pin 4
    // No pull-up needed since LED circuit provides the pull to VCC
    GPIOB->PUPDR &= ~(3U << (4 * 2));     // Clear PUPD bits (00 = no pull-up/down)
    GPIOB->ODR &= ~(1U << 4);             // Clear PB4
    GPIOB->ODR |= (1U << 4);              // Set PB4
}

// -----------------------------------------------------------
// Read INA228 values and fill TxFrames function
// -----------------------------------------------------------

bool INA228_Read_Values (CAN_TxBufferElement *TxINAFrames) {
  // Read INA228 values and fill TxFrames
  
  if (INA228_ReadCurr(&INA228_Current_mA)) {
    TxINAFrames[0].data[0] = (uint8_t)(INA228_Current_mA >> 24);   // Store current in Tx buffer frame 0
    TxINAFrames[0].data[1] = (uint8_t)(INA228_Current_mA >> 16);
    TxINAFrames[0].data[2] = (uint8_t)(INA228_Current_mA >> 8);
    TxINAFrames[0].data[3] = (uint8_t)(INA228_Current_mA & 0xFF);
  } else {
    return false; // Error reading current
  }

  if (INA228_ReadVBUS(&INA228_vbus)) {
    TxINAFrames[1].data[0] = (uint8_t)(INA228_vbus >> 8);        // Store VBUS in Tx buffer
    TxINAFrames[1].data[1] = (uint8_t)(INA228_vbus & 0xFF);
  } else {
    return false; // Error reading VBUS
  }
    
  if (INA228_ReadTemp(&INA228_Temp)) {
    TxINAFrames[2].data[0] = (uint8_t)(INA228_Temp >> 8);        // Store temperature in Tx buffer
    TxINAFrames[2].data[1] = (uint8_t)(INA228_Temp & 0xFF);
  } else {
    return false; // Error reading temperature
  }

  if (INA228_ReadShuntV(&INA228_ShuntVltg)) {
    TxINAFrames[3].data[0] = (uint8_t)(INA228_ShuntVltg >> 24); // Store shunt voltage in Tx buffer
    TxINAFrames[3].data[1] = (uint8_t)(INA228_ShuntVltg >> 16);
    TxINAFrames[3].data[2] = (uint8_t)(INA228_ShuntVltg >> 8);
    TxINAFrames[3].data[3] = (uint8_t)(INA228_ShuntVltg & 0xFF);
  } else {
    return false; // Error reading shunt voltage
  }

  if (INA228_ReadPower(&INA228_Power_mW)) {
    TxINAFrames[4].data[0] = (uint8_t)(INA228_Power_mW >> 24); // Store power in Tx buffer
    TxINAFrames[4].data[1] = (uint8_t)(INA228_Power_mW >> 16);
    TxINAFrames[4].data[2] = (uint8_t)(INA228_Power_mW >> 8);
    TxINAFrames[4].data[3] = (uint8_t)(INA228_Power_mW & 0xFF);
  } else {
    return false; // Error reading power
  }

  if (INA228_ReadDieID(&INA228_DieID)) {
    TxINAFrames[5].data[0] = (uint8_t)(INA228_DieID >> 8); // Store die ID in Tx buffer
    TxINAFrames[5].data[1] = (uint8_t)(INA228_DieID & 0xFF);
  } else {
    return false; // Error reading die ID
  }

  if (INA228_ReadManufacturerID(&INA228_ManufacturerID)) {
    TxINAFrames[6].data[0] = (uint8_t)(INA228_ManufacturerID >> 8); // Store manufacturer ID in Tx buffer
    TxINAFrames[6].data[1] = (uint8_t)(INA228_ManufacturerID & 0xFF);
  } else {
    return false; // Error reading manufacturer ID
  }

  if (INA228_ReadEnergy(&INA228_Energy_Joule)) {
    TxINAFrames[7].data[0] = (uint8_t)(INA228_Energy_Joule >> 56); // Store Energy in Tx buffer
    TxINAFrames[7].data[1] = (uint8_t)(INA228_Energy_Joule >> 48);
    TxINAFrames[7].data[2] = (uint8_t)(INA228_Energy_Joule >> 40);
    TxINAFrames[7].data[3] = (uint8_t)(INA228_Energy_Joule >> 32);
    TxINAFrames[7].data[4] = (uint8_t)(INA228_Energy_Joule >> 24);
    TxINAFrames[7].data[5] = (uint8_t)(INA228_Energy_Joule >> 16);
    TxINAFrames[7].data[6] = (uint8_t)(INA228_Energy_Joule >> 8);
    TxINAFrames[7].data[7] = (uint8_t)(INA228_Energy_Joule & 0xFF);
  } else {
    return false; // Error reading power
  }

  if (INA228_ReadCharge(&INA228_Charge_Coulombs)) {
    TxINAFrames[8].data[0] = (uint8_t)(INA228_Charge_Coulombs >> 56); // Store charge in Tx buffer
    TxINAFrames[8].data[1] = (uint8_t)(INA228_Charge_Coulombs >> 48);
    TxINAFrames[8].data[2] = (uint8_t)(INA228_Charge_Coulombs >> 40);
    TxINAFrames[8].data[3] = (uint8_t)(INA228_Charge_Coulombs >> 32);
    TxINAFrames[8].data[4] = (uint8_t)(INA228_Charge_Coulombs >> 24);
    TxINAFrames[8].data[5] = (uint8_t)(INA228_Charge_Coulombs >> 16);
    TxINAFrames[8].data[6] = (uint8_t)(INA228_Charge_Coulombs >> 8);
    TxINAFrames[8].data[7] = (uint8_t)(INA228_Charge_Coulombs & 0xFF);
  } else {
    return false; // Error reading charge
  }

  return true;
}


//------------------------------------------------------------
// Main function
//------------------------------------------------------------

int main(void) {

  init_clock();           // Initialize system clock
  systick_init();         // 1s second (STM32G0 runs at 16MHz)
  PA2_out_init();         // Test output PA2
  PB4_out_init();         // Test output PB4
  INA228_Init();          // Initialize INA228
  Can_Init();             // Initialize CAN
  Init_FDCAN_INA228_Message(&TxFrames[0]);  // Initialize FDCAN messages with ID's starting from 0x426U
  // Element [0] is highest priority --> used for current [mA], etc...
  // Element [7] is lowest priority --> used for DieID

  while (1) {
    // Main loop

    // Check messages from Processing Unit
    if (rx_flag == 1) {
      rx_flag = 0;                        // Reset the flag after processing
      // Check ProcessingUnitCommand 
      if (ProcessingUnitCommand[0] == 0x01U) {
        // Reset TxErrorCode (Frame[9]) if command is received 
        for (uint8_t i = 0; i < 8; i++) {
          TxFrames[9].data[i] = 0x00U;    // Clear the TxErrorCode frame
        } 
      }
      if (ProcessingUnitCommand[0] == 0x02U) {
        // Reset the contents of the INA228 accumulation registers Energy and Charge
          INA228_WriteRegister(0x00, 0b0100000000000000);
          }
        }
      
    


    if (one_sec_tick >= 20) { // 0.2s second tick --> to be changed to 100ms
        one_sec_tick = 0;
        INA228_Read_Values(&TxFrames[0]); // Read INA228 data and fill TxFrames
        msg_counter = 0;                  // Reset msg_counter to 0
          for (uint8_t i = 0; i < 8; i++) {
              if (FDCAN2_Send_Std_CAN_Message(&TxFrames[i])) {
                msg_counter = i;          // Set msg_counter to the current frame index
                } else {
                  i = 8;                  // Exit loop if FIFO full (sending failed)
                  msg_counter++;          // Point to next message to be send by Interrupt Routine
                }
          }
          // send the remaining messages via Interrupt
          FDCAN2->IE |= BIT(9);           // Tx FIFO empty interrupt enable
          FDCAN2->IR = (1U << 9);         // Clear the TFE interrupt flag
          // Check for transmit errors
          if (CAN_TxError == 1) {
            GPIOB->ODR ^= (1U << 4);      // Toggle PB4
          } else  {
                GPIOB->ODR |= (1U << 4);  // SET open drain off PB4 --> LED off
                  }
    }
  }
}



//------------------------------------------------------------
// Interrupt Handlers
//------------------------------------------------------------


// SysTick interrupt handler
// This function is called every 10ms
void SysTick_IRQHandler(void) {
  GPIOA->ODR ^= (1 << 2);   // Toggle PA2 for testing purposes
  one_sec_tick++;           // Increment the tick counter
}


// FDCAN interrupt handler
// This function is called when a message is received
// Check if there is a message in FIFO0
void TIM16_FDCAN_IT0_IRQHandler(void) {
  uint32_t ir = FDCAN2->IR;

  // Received Interrupt routine
  if (ir & 0x1U) {                              // Check (RF0N) if there is a message in FIFO0
      // Get the FIFO status
      uint32_t rxf0s = FDCAN2->RXF0S;
      uint8_t get_index = (rxf0s >> 8) & 0x3U;  // Get index (F0GI)
      uint8_t fill_level = rxf0s & 0x7F;        // Get fill level (F0FL)
    
      // Only process if there are actually messages in the FIFO
      if (fill_level > 0) {
        CAN_RxBufferElement *ptr = &FDCAN2_RxFIFO0[get_index];
        rx_temp = FDCAN2_RxFIFO0[get_index];

      // Read the message ID
      uint32_t temp2 = ptr->R0;
      uint32_t MsgID = (temp2 >> 18) & 0x7FF;   // Read the message from FIFO0

      // Process based on message ID
      if (MsgID == 0x127) {
        memcpy((void*)ProcessingUnitStatus, ptr->data, 8);  // Read the message from FIFO0
      } else if (MsgID == 0x128) {
        memcpy((void*)ProcessingUnitCommand, ptr->data, 8); // Read the message from FIFO0
      }
   
      rx_flag = 1;                // Set the flag to indicate that a message has been received
      FDCAN2->RXF0A = get_index;  // Release the FIFO element
      FDCAN2->IR = 0x1U;          // Clear interrupt flag by writing 1 to it
      }
  }

  // Transmit interrupt routine
  if (ir & (1U << 9)) {                       // Check if Tx FIFO is empty (TXFE)
    x++;                                      // for testing to check how many TFEs were sent
    y = msg_counter; 
    if (msg_counter >= 10) {                  // All messages send!
      FDCAN2->IE &= ~BIT(9);                  // Tx FIFO empty interrupt disable
      FDCAN2->IR = (1U << 9);                 // Clear the TFE interrupt flag
      // If all messages are sent, check for transmit errors
      // and update the SensorNodeStatus with error codes if any
      uint32_t psr = FDCAN2->PSR;             // Read the PSR for transmit errors
      uint32_t ecr = FDCAN2->ECR;             // Read the ECR for transmit errors
      uint32_t lec = psr & 0x7U;              // Last error code
      uint32_t dlec = ((psr & 0x700U) >> 8);  // Data last error code
      uint32_t tec = ecr &0xFFU;              // Read the transmit error counter

      // Check for transmit error and error counter going up
      if ((lec != 0) || (dlec != 0b111) || tec > previous_tec_value) {  
          CAN_TxError = true;                 // Set error flag if there are transmit errors
          // write the error code to the SensorNodeStatus
          TxFrames[9].data[0] = (uint8_t)lec;           // Last error code
          TxFrames[9].data[1] = (uint8_t)dlec;          // Data last error code
          TxFrames[9].data[2] = (uint8_t)(tec >> 24);   // Transmit error counter
          TxFrames[9].data[3] = (uint8_t)(tec >> 16);
          TxFrames[9].data[4] = (uint8_t)(tec >> 8);
          TxFrames[9].data[5] = (uint8_t)(tec & 0xFF);
          TxFrames[9].data[6] = 0x1U;                   // Indicate, error currently existing
          TxFrames[9].data[7] = 0;                      // Reserved for future use
          // FDCAN2_Send_Std_CAN_Message(&TxFrames[0]); // Send HostStatus message with error codes
        } else {
            CAN_TxError = false;                        // Clear error flag if no transmit errors
            TxFrames[9].data[6] = 0x0U;                 // Indicate, no error currently existing
        }
      previous_tec_value = tec;         // Update the previous transmit error counter value

    } 
    else {                        // Send the next message
        for (y = y; y < 10; y++) {
          if (FDCAN2_Send_Std_CAN_Message(&TxFrames[msg_counter])) {    // send the next message package until FIFO is full again
            msg_counter++;        // Increment msg_counter if message was sent successfully
          }
        FDCAN2->IR = (1U << 9);   // Clear the TFE interrupt flag
        }
    }

  }
}

  



