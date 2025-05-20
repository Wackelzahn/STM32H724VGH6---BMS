

// Created: 2025-03-06 16:00:00
// Author: Michael Schneider
// -----------------------------------------------------
// Target Device: STM32G0B1KEU6
// -----------------------------------------------------
// Programming Language: C, bare metal
//
// This is a test program for the STM32F411RE to explore
// the capabilities of the STM32F411RE microcontroller.
// The program is intended to create 
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

uint8_t x=0;
uint32_t can_receive_FifO0 = 55;
uint32_t temp;

uint32_t Sprong1, Sprong2, Kaponk4, Power_mW; 
uint64_t Kaponk, Energy_Joule;
uint64_t Kagong;
uint16_t vbus, vbus_mV, INA228_DieID, INA228_ManufacturerID;
int32_t Current_mA, Kaponk2, INA228_ShuntVltg, ShuntV_uV;
int16_t INA228_Temp, Temperature;

volatile uint8_t rx_flag = 0;
volatile uint8_t rx_received[8];
volatile uint8_t HostStatus[8];
volatile uint8_t HostCommand[8];
CAN_RxBufferElement rx_temp;
uint32_t one_sec_tick = 0;

#define FREQ 16000000  // PCLK, internal clock by default, 16 Mhz
#define BIT(x) (1UL << (x))


CAN_TxBufferElement Tx_Temperature = {
  .T0 = (0x427U << 18) | (0U << 29) | (0U << 30) | (0U << 31), // ID = 0x127, Data frame
  .T1 = (0x08U << 16) | (0U << 20) | (0U << 21) | (0U << 23), // 8 byte, classic CAN
  .data = {1, 2, 3, 4, 5, 6, 7, 8} // Data bytes
};





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


  // FDCAN2_Send_Std_CAN_Message();

  if (INA228_ReadTemp(&INA228_Temp)) {
    Temperature = INA228_Temp;
    Tx_Temperature.data[0] = (uint8_t)(Temperature >> 8); // Store temperature in Tx buffer
    Tx_Temperature.data[1] = (uint8_t)(Temperature & 0xFF);
    (void)Temperature;  // preventing compiler complained for not being used.
    }

  if (INA228_ReadShuntV(&INA228_ShuntVltg)) {
    ShuntV_uV = INA228_ShuntVltg;
    (void)ShuntV_uV;
    }

if (INA228_ReadDieID(&INA228_DieID)) {
    (void)INA228_DieID;
    }
if (INA228_ReadManufacturerID(&INA228_ManufacturerID)) {
    (void)INA228_ManufacturerID;
    }


  if (rx_flag) {
    rx_flag = 0;        // check for message id and store the data
    // Do something with rx_received[8]
    }

    if (one_sec_tick >= 1000) { // 1 second tick
      one_sec_tick = 0;
      // Do something every second
      // For example, send a message over CAN
      //FDCAN2_Send_Std_CAN_Message();
      FDCAN2_Send_Std_CAN_Message(&Tx_Temperature);
      GPIOB->ODR ^= (1 << 4); // Toggle PB4
    }

  }


}

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


