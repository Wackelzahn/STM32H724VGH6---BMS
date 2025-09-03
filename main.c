

// Created: 2025-08-18 16:00:00
// Author: Michael Schneider
// -----------------------------------------------------
// Target Device: STM32H742VGH6
// -----------------------------------------------------
// Programming Language: C, bloody pure bare metal (no CMSIS)
//
// This is a the program for the STM32H742 used for
// Controlling a BMS with CAN communication to a Shunt 
// sensor and a second CAN node to a host controller.
// further has LAN communication to a Processing Unit
// 
// The program is using
//    - an accurate base tick (10ms) (SYSTICK IRQ)
//    - GPIO output
//    - SPI communication
//    - CAN communication
//    - LAN communication
// -----------------------------------------------------


#include "startup.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "registers.h"
#include "can.h"
#include "rtc.h"
#include "VEcan.h"
#include "shunt_can.h"



//------------------------------------------------------------
// Global variables
//------------------------------------------------------------

uint8_t rx_message_426[8]; // buffer for received CAN message
uint8_t rx_message_427[8]; // buffer for received CAN message
uint8_t rx_message_428[8]; // buffer for received CAN message
uint8_t rx_message_429[8]; // buffer for received CAN message
uint8_t rx_message_42A[8]; // buffer for received CAN message
uint8_t rx_message_42B[8]; // buffer for received CAN message
uint8_t rx_message_42C[8]; // buffer for received CAN message
uint8_t rx_message_42D[8]; // buffer for received CAN message
uint8_t rx_message_42E[8]; // buffer for received CAN message

uint32_t fifo_full_counter = 0; // counter for FIFO full events
uint32_t fifo_message_lost_counter = 0; // counter for FIFO message lost events

// use volatile variables to ensure they are not optimized out by the compiler
// and to ensure they are always read from memory. Use for variables that are used
// in interrupt handlers or memory mapped registers. Or are modified by threads  / tasks.

volatile uint32_t tick = 0;
bool zonk = false; // for testing only
uint8_t seconds = 0; // seconds counter
uint8_t minutes = 0; // minutes counter
RTC_DateTime dt;

// Debug variables 
volatile int32_t current = 0;
volatile int32_t bus_voltage = 0;
volatile int32_t temperature = 0;
volatile int32_t shunt_voltage = 0;
volatile double power = 0;
volatile uint16_t die_id = 0;
volatile uint16_t manufacturer_id = 0;
volatile double energy = 0;
volatile int64_t charge = 0;

volatile uint32_t temp[16]; // for testing only


//------------------------------------------------------------
// Constants and definitions
//------------------------------------------------------------

#define SYSTEM_CLOCK_HZ 200000000 // 200MHz core clock
#define SYSTICK_FREQUENCY_HZ 100  // 100Hz = 10ms tick

#define BIT(x) (1UL << (x))

// bit definitions for GPIOC
#define RCC_AHB4ENR_GPIOCEN (1 << 2)

  // for testing only
  CAN_TxBufferElement tx_127;
  CAN_TxBufferElement tx_128;

//------------------------------------------------------------
// Init Functions
//------------------------------------------------------------

// Initialize the clock system to 200Mhz core clock
// and 100Mhz APB1, APB2, APB3 and AHB4 clock
// This is the default configuration for the STM32H742

void clock_init(void) {
  // 1. Set VOS2 power scaling  --> default is VOS3
  // PWR->D3CR = (PWR->D3CR & ~(3U << 14)) | (2U << 14);
  // for(volatile int i = 0; i < 100; i++);  // Small delay
  //while (!(PWR->D3CR & (1U << 13)));    // Wait until VOS is ready --> not working!! maybe HW fault
  
  // 2. Enable 16Mhz HSE
  RCC->CR |= (1U << 16); // Set HSEON bit
  while (!(RCC->CR & (1U << 17))); // Wait until HSE is ready

  // 3. Set PLL1 to 200Mhz Sysclock
  RCC->CR &= ~(1U << 24);                       // Disable PLL1 first
  while (RCC->CR & (1U << 25));                 // Wait until PLL1 is disabled
  RCC->PLLCKSELR = 0x0U;               // Clear PLL1 source bits and other dviders
  RCC->PLLCKSELR |= (0x2U << 0);                // Set to HSE as source for PLL
  RCC->PLLCKSELR |= (0x4U << 4);                // Set DIVM1 to 4 (HSE/4 = 4MHz)
  RCC->PLL1DIVR = ((100U-1U) << 0) |            // Set DIVN1 to 100
                  ((1U << 9) |                  // Set DIVP1 to 2
                   (1U << 16) | // Set DIVQ1 to 2 
                   (1U << 24)); // Set DIVR1 to 2
  RCC->PLLCFGR |= (2U << 2);    // Set PLL1 VCO Range to 4-8Mhz (PLL Input is 4MHz)
  RCC->PLLCFGR |= (1U << 16);   // Enable PLL1 p clock output
  RCC->CR |= (1U << 24);        // Enable PLL1
  while (!(RCC->CR & (1U << 25))); // Wait until PLL1 is ready
  
  // 3. Set flash latency (2 Wait States for 200Mhz VOS3)
  FLASH->ACR &= ~(0xFU << 0); // Clear latency bits
  FLASH->ACR |= (2U << 0); // Set latency to 2 wait states

  // 4. Configure bus prescaler
  RCC->D1CFGR &= ~(0x8U << 8); // D1CPRE = 1 (200MHz)
  RCC->D1CFGR &= ~(0x8U << 0); // HPRE = 1 (200MHz)

  // 5. Set APB3, APB1, APB2 and AHB4 prescaler to 2 (100Mhz each)
  RCC->D1CFGR |= (0x4U << 4); // Set APB3 prescaler to 2 (D1PPRE) 100MHz
  RCC->D2CFGR |= (0x4U << 4); // Set APB1 prescaler to 2 (D2PPRE1)
  RCC->D2CFGR |= (0x4U << 8); // Set APB2 prescaler to 2 (D2PPRE2)
  RCC->D3CFGR |= (0x4U << 4); // Set AHB4 prescaler to 2 (D3PPRE)

  // 6. Set system clock to PLL1 
  RCC->CFGR &= ~(0x3U << 0); // Clear SW bits
  RCC->CFGR |= (0x3U << 0); // Set SW to PLL1 
  while ((RCC->CFGR & (0x3U << 3)) != (0x3U << 3)); // Wait until SWS is PLL1

}

static inline void systick_init() {
  uint32_t reload_value = (2500000) - 1; // 100ms tick
  
  SYST->RVR = reload_value & 0x00FFFFFF;  // Set reload value for 10ms tick;
  SYST->CVR = 0;                          // Clear current value register
  SYST->CSR = (1U << 0) | (1U << 1); // | (1U << 2); // Enable counter, interrupt, processor
  
}

void gpio_init_PC0(void) {

  
  // Enable GPIOC clock
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
    
    // Configure PC0 as output
    GPIOC->MODER = (GPIOC->MODER & ~(3U << 0)) | (1U << 0);     // PC0 as output
    GPIOC->OTYPER |= (1U << 0);                                 // PC0 open-drain (open collector)
    GPIOC->OSPEEDR = (GPIOC->OSPEEDR & ~(3U << 0)) | (1U << 0); // PC0 medium speed (01)
}



// -----------------------------------------------------------
// Functions
// -----------------------------------------------------------

// CPACR register address for Cortex-M7
#define CPACR_REG   (*((volatile uint32_t*)0xE000ED88))

void enable_fpu(void) {
    // Enable CP10 and CP11 coprocessors (FPU)
    CPACR_REG |= (0xF << 20);  // Set bits 23:20 to 1111
    
    // Memory barriers to ensure FPU is enabled before use
    __asm volatile ("dsb");
    __asm volatile ("isb");
}

//------------------------------------------------------------
// Main function
//------------------------------------------------------------

int main(void) {
  
  enable_fpu(); // Enable FPU for floating point operations

  // for testing only
  // Standard ID 0x127, no RTR, no XTD, 
  tx_127.T0 = (0x127U << 18) | (0x0U << 29) | (0x0U << 30); 
  tx_127.T1 = 0; // No EFC, no FDF, no BRS, 
  tx_127.T1 |= (8U << 16); // DLC = 8
  tx_127.data[0] = 0xDE;
  tx_127.data[1] = 0xAD;
  tx_127.data[2] = 0xBE;    
  tx_127.data[3] = 0xFE;  
  tx_127.data[4] = 0xCA;
  tx_127.data[5] = 0x7E;  
  tx_127.data[6] = 0xB2;  
  tx_127.data[7] = 0x1E;

  tx_128.T0 = (0x128U << 18) | (0x0U << 29) | (0x0U << 30); 
  tx_128.T1 = 0; // No EFC, no FDF, no BRS, 
  tx_128.T1 |= (8U << 16); // DLC = 8
  tx_128.data[0] = 0xFF;
  tx_128.data[1] = 0xAD;
  tx_128.data[2] = 0xBE;    
  tx_128.data[3] = 0xFE;  
  tx_128.data[4] = 0xCA;
  tx_128.data[5] = 0xFF;  
  tx_128.data[6] = 0xB2;  
  tx_128.data[7] = 0x1E;

  clock_init();
  gpio_init_PC0();
  systick_init();
  // Enable interrupts globally
  __asm("cpsie i");
  FDCAN1_init(); // Initialize FDCAN1 interface
  FDCAN2_init(); // Initialize FDCAN2 interface
  VECan_Init (); // Initialize VE CAN messages
  VECan_send(); // Send VE CAN messages once at startup
  if (RTC_init()) zonk =1; // Initialize RTC interface
  dt = RTC_read_datetime(); // Read current date and time
    seconds = dt.seconds; // get current seconds
    minutes = dt.minutes; // get current minutes


  while (1) {
  //  if (fifo_full_counter > 0) {
      // Error handling for FIFO full
   //   void FDCAN1_IT0_IRQHandler(void); // Call the interrupt handler to clear the flag
  //  }
  //  if (fifo_message_lost_counter > 0) {
   //   // Error handling for message lost
   //   void FDCAN1_IT0_IRQHandler(void); // Call the interrupt handler to clear the flag
  //  }

}
}

//------------------------------------------------------------
// Interrupt Handlers
//------------------------------------------------------------

// SysTick interrupt handler
// This function is called every 100ms
void SysTick_IRQHandler(void) {
  
  tick++;                   // Increment the tick counter
  
  if (tick == 5) {          // 10 ticks = 1 second
    tick = 0;               // Reset the tick counter
    GPIOC->ODR ^= (1 << 0); // PC0 at 1Hz
    FDCAN1_transmit_message(&tx_127); // Transmit test message
    FDCAN1_transmit_message(&tx_128); // Transmit test message
    VECan_send (); // Send VE CAN messages every second
    current = convert_0x0426_to_milliampere(rx_message_426); // convert shunt message to current in mA
    bus_voltage = convert_0x0427_to_millivolts(rx_message_427); // convert shunt message to voltage in mV
    temperature = convert_0x0428_to_temperature(rx_message_428); // convert shunt message to temperature in 1/100 °C
    shunt_voltage = convert_0x0429_to_microvolts(rx_message_429); // convert shunt message to voltage in uV
    power = convert_0x042A_to_milliwatts(rx_message_42A); // convert shunt message to power in mW
    die_id = convert_0x042B_to_DieID(rx_message_42B); // convert shunt message to die ID
    manufacturer_id = convert_0x042C_to_ManufacturerID(rx_message_42C); // convert shunt message to manufacturer ID
    energy = convert_0x042D_to_energy(rx_message_42D); // convert shunt message to energy in uWh
    charge = convert_0x042E_to_charge(rx_message_42E); // convert shunt message to charge in uAh
  }
}
  

// FDCAN interrupt handler
// This function is called when a message is received
// Check if there is a message in FIFO0
void FDCAN1_IT0_IRQHandler(void) {

  uint32_t rxf0s;
  uint32_t fill_level;
  uint8_t get_index;
  uint32_t can_id;

  // Check if Message is lost
  if (FDCAN1->IR & BIT(3)) {
    fifo_message_lost_counter++;
    FDCAN1->IR |= BIT(3); // Clear the message lost flag
  }

  else if ((FDCAN1->IR & BIT(0))|(FDCAN1->IR & BIT(2))) { // New message in FIFO 0 or FIFO full
    if (FDCAN1->IR & BIT(2)) {
      fifo_full_counter++;
    }
    rxf0s = FDCAN1->RXF0S; // Read RX FIFO 0 status register
    fill_level = (FDCAN1->RXF0S & 0x7F); // Get fill level
    get_index = (rxf0s >> 8) & 0x3F; // Get index of the oldest message
     
    while (fill_level > 0) { // Process all messages in FIFO 0
      // Read message from RX FIFO 0
      volatile CAN_RxBufferElement *received_frame = (CAN_RxBufferElement*) (FDCAN_MESSAGE_RAM_BASE 
        + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_RX_FIFO0_offset + (get_index * sizeof(CAN_RxBufferElement)));
      can_id = (received_frame->R0 >> 18) & 0x7FF; // Get standard ID
      // avoid unused variable warning
      //(void)can_id;

      switch (can_id) {
        case 0x426:
          // Handle message with ID 0x426
          for (uint8_t i=0; i < 8; i++) {
            rx_message_426[i] = received_frame->data[i];
          }
          break;
        case 0x427:
          // Handle message with ID 0x427
          for (uint8_t i=0; i < 8; i++) {
            rx_message_427[i] = received_frame->data[i];
          }
          break;
        case 0x428:
          // Handle message with ID 0x428
          for (uint8_t i=0; i < 8; i++) {
            rx_message_428[i] = received_frame->data[i];
          }
          break;
        case 0x429:
          // Handle message with ID 0x429
          for (uint8_t i=0; i < 8; i++) {
            rx_message_429[i] = received_frame->data[i];
          }
          break;
        case 0x42A:
          // Handle message with ID 0x426
          for (uint8_t i=0; i < 8; i++) {
            rx_message_42A[i] = received_frame->data[i];
          }
          break;
        case 0x42B:
          // Handle message with ID 0x427
          for (uint8_t i=0; i < 8; i++) {
            rx_message_42B[i] = received_frame->data[i];
          }
          break;
        case 0x42C:
          // Handle message with ID 0x428
          for (uint8_t i=0; i < 8; i++) {
            rx_message_42C[i] = received_frame->data[i];
          }
          break;
        case 0x42D:
          // Handle message with ID 0x429
          for (uint8_t i=0; i < 8; i++) {
            rx_message_42D[i] = received_frame->data[i];
          }
          break;
        case 0x42E:
          // Handle message with ID 0x429
          for (uint8_t i=0; i < 8; i++) {
            rx_message_42E[i] = received_frame->data[i];
          }
          break;
          default:
          // Handle other messages
          // ...
          break;
      }
      // Acknowledge the message
      FDCAN1->RXF0A = get_index & 0x3F; // Acknowledge the message
      fill_level = (FDCAN1->RXF0S & 0x7F); // Get fill level


     
  }
      FDCAN1->IR |= BIT(0); // Clear the new FIFO 0 message flag
      FDCAN1->IR |= BIT(2); // Clear the FIFO 0 full flag
  }

  else {
    // Unknown interrupt, clear all flags
    FDCAN1->IR = 0xFFFFFFFF;
  }
 
}

