

// Created: 2025-03-06 16:00:00
// Author: Michael Schneider
// -----------------------------------------------------
// Target Device: STM32G0B1KEU6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is a the program for the STM32G0B1 to collect 
// data from the INA228 and send it over CAN bus.
// 
// The program is using
//    - an accurate base tick (10ms) (SYSTICK IRQ)
//    - GPIO output
//    - SPI communication
//    - CAN communication
// -----------------------------------------------------
//


#include "startup.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "registers.h"
//#include "serial.h"
//#include "conversion.h"
#include "can.h"
//#include "VEcan.h"
#include "INA228.h"


//------------------------------------------------------------
// Global variables
//------------------------------------------------------------

uint8_t x=0;
uint32_t can_receive_FifO0 = 55;
uint32_t temp;

uint32_t Sprong1, Sprong2, Kaponk4, Power_mW, INA228_Power; 
uint64_t Kaponk, Energy_Joule;
uint64_t Kagong;
uint16_t vbus, vbus_mV, INA228_DieID, INA228_ManufacturerID;
int32_t Current_mA, Kaponk2, INA228_ShuntVltg, ShuntV_uV;
int16_t INA228_Temp, Temperature;

uint8_t zonk = 0;  

volatile uint8_t rx_flag = 0;
volatile uint8_t rx_received[8];
volatile uint8_t HostStatus[8];
volatile uint8_t HostCommand[8];
CAN_RxBufferElement rx_temp;
uint32_t one_sec_tick = 0;

#define FREQ 16000000  // PCLK, internal clock by default, 16 Mhz
#define BIT(x) (1UL << (x))


CAN_TxBufferElement Tx_Temperature = {
  .T0 = (0x426U << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x426, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {0, 0, 0, 0, 0, 0, 0, 0} // Data bytes
};

CAN_TxBufferElement Tx_ShuntVoltage = {
  .T0 = (0x427U << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x427, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {0, 0, 0, 0, 0, 0, 0, 0} // Data bytes
};

CAN_TxBufferElement Tx_VBus_Voltage = {
  .T0 = (0x428U << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x428, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {0, 0, 0, 0, 0, 0, 0, 0} // Data bytes
};

CAN_TxBufferElement Tx_Current = {
  .T0 = (0x429U << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x429, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {0, 0, 0, 0, 0, 0, 0, 0} // Data bytes
};

CAN_TxBufferElement Tx_Power = {
  .T0 = (0x42AU << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x42A, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {0, 0, 0, 0, 0, 0, 0, 0} // Data bytes
};

CAN_TxBufferElement Tx_DieID = {
  .T0 = (0x42BU << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x42B, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {0, 0, 0, 0, 0, 0, 0, 0} // Data bytes
};

//------------------------------------------------------------
// Init Functions
//------------------------------------------------------------

static inline void init_clock(void) {
  RCC->CR |= (1U << 8);              // HSI16 clock enable
  while (!(RCC->CR & (1U << 10)));   // HSI16 clock ready flag
  RCC->APBENR2 |= (1U << 0);  // Enable SYSCFG clock
}

static inline void systick_init() {
  uint32_t ticks = 16000; // 1ms tick at 16MHz, 16,000 Cycles
  if ((ticks - 1) > 0xffffff) return;    // Systick timer is 24 bit
  SYST->RVR = ticks - 1;
  SYST->CVR = 0;
  SYST->CSR = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APBENR2 |= BIT(0);               // Enable SYSCFG
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
    GPIOB->MODER &= ~(3U << (4 * 2));  // Clear mode bits for pin 4 (2 bits per pin)
    GPIOB->MODER |= (1U << (4 * 2));   // Set as output (01) for pin 4
    // Configure PB4 as open drain
    GPIOB->OTYPER |= (1U << 4);  // Set open drain (1) for pin 4
    // Configure speed (optional, medium speed used here)
    GPIOB->OSPEEDR &= ~(3U << (4 * 2));  // Clear speed bits for pin 4
    GPIOB->OSPEEDR |= (1U << (4 * 2));   // Set as medium speed (01) for pin 4
    // No pull-up needed since LED circuit provides the pull to VCC
    GPIOB->PUPDR &= ~(3U << (4 * 2));    // Clear PUPD bits (00 = no pull-up/down)
  GPIOB->ODR &= ~(1U << 4);           // Clear PB4
  GPIOB->ODR |= (1U << 4);            // Set PB4
}


//------------------------------------------------------------
// Main function
//------------------------------------------------------------

int main(void) {

  init_clock();             // Initialize system clock
  systick_init();   // 1s second (STM32G0 runs at 16MHz)
  PA2_out_init();           // Test output PA2
  PB4_out_init();           // Test output PB4
  if (INA228_Init()){
    x++;
  };                  // Initialize INA228 

  Can_Init();         // Initialize CAN

  if (FDCAN2_Send_Std_CAN_Message(&Tx_Temperature)) {
    x++;
  } else {
    x--;
  } 

  while (1) {


  if (INA228_ReadTemp(&INA228_Temp)) {
    Tx_Temperature.data[0] = (uint8_t)(INA228_Temp >> 8); // Store temperature in Tx buffer
    Tx_Temperature.data[1] = (uint8_t)(INA228_Temp & 0xFF);
    }

  if (INA228_ReadShuntV(&INA228_ShuntVltg)) {
    Tx_ShuntVoltage.data[0] = (uint8_t)(INA228_ShuntVltg >> 24); // Store shunt voltage in Tx buffer
    Tx_ShuntVoltage.data[1] = (uint8_t)(INA228_ShuntVltg >> 16);
    Tx_ShuntVoltage.data[2] = (uint8_t)(INA228_ShuntVltg >> 8);
    Tx_ShuntVoltage.data[3] = (uint8_t)(INA228_ShuntVltg & 0xFF);
    }


if (INA228_ReadDieID(&INA228_DieID)) {
    Tx_DieID.data[0] = (uint8_t)(INA228_DieID >> 8); // Store die ID in Tx buffer
    Tx_DieID.data[1] = (uint8_t)(INA228_DieID & 0xFF);
    }

if (INA228_ReadPower(&INA228_Power)) {
    Tx_Power.data[0] = (uint8_t)(INA228_Power >> 24); // Store power in Tx buffer
    Tx_Power.data[1] = (uint8_t)(INA228_Power >> 16);
    Tx_Power.data[2] = (uint8_t)(INA228_Power >> 8);
    Tx_Power.data[3] = (uint8_t)(INA228_Power & 0xFF);
    }
if (INA228_ReadVBUS(&vbus)) {
    Tx_VBus_Voltage.data[0] = (uint8_t)(vbus >> 8); // Store VBUS in Tx buffer
    Tx_VBus_Voltage.data[1] = (uint8_t)(vbus & 0xFF);
    }
if (INA228_ReadCurr(&Current_mA)) {
    Tx_Current.data[0] = (uint8_t)(Current_mA >> 24); // Store current in Tx buffer
    Tx_Current.data[1] = (uint8_t)(Current_mA >> 16);
    Tx_Current.data[2] = (uint8_t)(Current_mA >> 8);
    Tx_Current.data[3] = (uint8_t)(Current_mA & 0xFF);
    }



  if (rx_flag) {
    rx_flag = 0;        // check for message id and store the data
    // Do something with rx_received[8]
    }

  if (one_sec_tick >= 50) { // 1/2 second tick
      one_sec_tick = 0;
      if (zonk == 0) FDCAN2_Send_Std_CAN_Message(&Tx_Temperature);
      if (zonk == 1) FDCAN2_Send_Std_CAN_Message(&Tx_ShuntVoltage);
      if (zonk == 2) FDCAN2_Send_Std_CAN_Message(&Tx_VBus_Voltage);
      if (zonk == 3) FDCAN2_Send_Std_CAN_Message(&Tx_Current);
      if (zonk == 4) FDCAN2_Send_Std_CAN_Message(&Tx_Power);
      if (zonk == 5) FDCAN2_Send_Std_CAN_Message(&Tx_DieID);
      zonk++;
      if (zonk >= 6) zonk = 0; // Reset the flag
      // Do something every second
      // For example, send a message over CAN
      //FDCAN2_Send_Std_CAN_Message();
           
      GPIOB->ODR ^= (1 << 4); // Toggle PB4
      }
    

  }


}



//------------------------------------------------------------
// Interrupt Handlers
//------------------------------------------------------------


// SysTick interrupt handler
void SysTick_IRQHandler(void){
  // SysTick interrupt handler
  // This function is called every 1ms
  GPIOA->ODR ^= (1 << 2);  // Toggle PA2
  one_sec_tick++;         // Increment the tick counter
  
}

// FDCAN interrupt handler
  // This function is called when a message is received
  // Check if there is a message in FIFO0
void TIM16_FDCAN_IT0_IRQHandler(void) {
  uint32_t ir = FDCAN2->IR;

  if (ir & 0x1U) {  // Check (RF0N) if there is a message in FIFO0
    
      // Get the FIFO status
      uint32_t rxf0s = FDCAN2->RXF0S;
      uint8_t get_index = (rxf0s >> 8) & 0x3U;  // Get index (F0GI)
      uint8_t fill_level = rxf0s & 0x7F;        // Get fill level (F0FL)
    
      // Only process if there are actually messages in the FIFO
      if (fill_level > 0) {
        CAN_RxBufferElement *ptr = &FDCAN2_RxFIFO0[get_index];
   
      // Read the message ID
      uint32_t temp2 = ptr->R0;
      uint32_t MsgID = (temp2 >> 18) & 0x7FF; // Read the message from FIFO0

      // Process based on message ID
      if (MsgID == 0x127) {
        memcpy((void*)HostStatus, ptr->data, 8); // Read the message from FIFO0
      } else if (MsgID == 0x128) {
        memcpy((void*)HostCommand, ptr->data, 8); // Read the message from FIFO0
      }
   
    rx_flag = 1; // Set the flag to indicate that a message has been received
    FDCAN2->RXF0A = get_index; // Release the FIFO element
   
    FDCAN2->IR = 0x1U;      // Clear interrupt flag by writing 1 to it

    }
  // potential Transmit interrupt routine
  
}
}


