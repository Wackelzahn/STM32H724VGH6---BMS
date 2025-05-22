
// Created: 2025-03-19 07:00:00
// Author: M. Schneider
// -----------------------------------------------------
// Target Device: STM32G0B1KEU6
// -----------------------------------------------------
// Programming Language: C, pure bare metal (no CMSIS)
//
// This is the implementation file CAN bus interfacing 
// for the STM32G0B1 microcontroller.
// This source code is intended to create 
//    - the initialization of the FDAN2 interface
//      1. Initialization of the GPIO
//      2. Setting up the Filter
//      3. FUNCTIONS FOR sending (receive is handled in ISR)
// -----------------------------------------------------


#include "can.h"
#include "registers.h"

#define BIT(x) (1UL << (x)) // Bit banging
int32_t counter_loop = 0;   // counter for waiting loops


//  function for initializing the FDCAN2 Interface for STM32G0B1
//   CAN_RX on PB0, CAN_TX on PB1, Clock speed: 16MHz
//   Target bitrate: 500 Kbps
//  ------------------------------------
//  Parameters:  none
//  Returns:     1=success, 0=fail 
//  ------------------------------------

bool Can_Init(void) {
    // 1. enable Clocks
    RCC->IOPENR |= BIT(1);           // Enable clock for GPIOB

    // Configure PB0 (RX) and PB1 (TX) for FDCAN2
    // Clear mode bits first then set to alternate function (0b10)
    GPIOB->MODER &= ~(BIT(0) | BIT(1) | BIT(2) | BIT(3)); // Clear mode bits for PB0 and PB1
    GPIOB->MODER |= (BIT(1) | BIT(3));                    // Set alternate function mode for PB0 and PB1

    // Set pins to AF3 for FDCAN2 (PB0/PB1 on AF3 according to STM32G0B1 datasheet)
    GPIOB->AFR[0] &= ~(0xFU << (0*4) | 0xFU << (1*4));  // Clear AF for PB0 & PB1
    GPIOB->AFR[0] |= (0x3U << (0*4) | 0x3U << (1*4));   // Set AF3 for FDCAN2

    // Enable FDCAN clock
    RCC->APBENR1 |= BIT(12);         // Enable the clock for FDCAN2 (bit 12 in APBENR1)

    // Initialize FDCAN
    FDCAN2->CCCR |= BIT(0);             // Enter configuration mode (INIT bit)
    FDCAN2->CCCR |= BIT(1);             // Set CCE bit to allow configuration changes

    counter_loop = 0;
    while (!(FDCAN2->CCCR & BIT(0)))    // Check if INIT bit is set (in config mode)
    {                                   
        if (counter_loop > 1000) {
            return false;
        }
        counter_loop++;
    }

    // Configure for classic CAN operation, Normal mode, no FD operation,
    // no bit rate switching, no auto-retransmission, enable transmit pause
    // no protocol exception handling, can according ISO 11898-1
    FDCAN2->CCCR &= ~BIT(5);            // Disable bus monitoring
    FDCAN2->CCCR |= BIT(6);             // Automatic retransmission disabled
    FDCAN2->CCCR &= ~BIT(7);            // Normal operation mode    
    FDCAN2->CCCR &= ~BIT(8);            // FD Operation disabled
    FDCAN2->CCCR &= ~BIT(9);            // Bit Rate Switch disabled
    FDCAN2->CCCR &= ~BIT(12);            // Protocol Exception Handling disabled
    FDCAN2->CCCR |= BIT(14);            // Transmit Pause enabled
    FDCAN2->CCCR &= ~BIT(15);           // ISO 11898-1 mode disabled


    // Configure nominal bit timing for 500 Kbps with 16 MHz clock
    // Calculation:
    // Target bit rate = 500 Kbps
    // Prescaler = 2 -> 8 M2
    // NTS1 = 10 (11 tq), NTS2 = 3 (4 tq), SJW = 1 (2 tq)
    // Total bit time = 1 + 11 + 4 = 16 tq
    // Bit rate = 8 MHz / 16 = 500 Kbps
    FDCAN2->NBTP =  (3 << 25) |         // NTSEG2 (value - 1)
                    (1 << 16) |         // NTSEG1 (value - 1)
                    (10 << 8) |         // NBRP (value - 1)
                    (3 << 0);           // NSJW (value - 1)

    // Configure data bit rate (same as nominal for simplicity)
    // NSJW = 3 (register value) → DSJW = 3
    // NTSEG2 = 3 (register value) → DTSEG2 = 3
    // NTSEG1 = 10 (register value) → DTSEG1 = 10
    // NBRP = 1 (register value) → DBRP = 1
    FDCAN2->DBTP = (1U << 16) |          // DBRP register
                   (3U << 8) |           // DTSEG1 register
                   (10U << 4) |          // DTSEG2 register
                   (3U << 0);            // DSJW register
               

    // Standart filter configuration (1 Filter, accept only 0x127 ID to Rx FIFO 0)
    // SFID2 = 0x0127; // only ID1 accepted
    // SFID1 = 0x0127; // Filter ID 1
    // SFEC = 0b001; // If match, store in Rx FIFO0
    // SFT = 0b10; // Standard ID filter type (Filter and Mask)

    for (int i=0; i<212; i++) { // Clear FDCAN2 message RAM
        FDCAN2_StandartMessageIDFilter[i] = 0x00000000U;  // equal to FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset
    }

    // Set filter 0(1) to accept ID 0x127(0x0128)
    FDCAN2_StandartMessageIDFilter[0] = (0x127U << 0) |(0x127U << 16) |(0b001U << 27) |(0b10U << 30); 
    FDCAN2_StandartMessageIDFilter[1] = (0x128U << 0) |(0x128U << 16) |(0b001U << 27) |(0b10U << 30); 

    // Configure global filter settings
    // Set RRFE = 1 (reject extended frames), Set ANFE = 0b11 Reject non-matching frames
    // Set ANFE = 0b11 (reject non-matching frames), Set LSS = 0b00001 (1 standart filter)
    FDCAN2->RXGFC = (1U << 0) |     // Reject all "remote" frames with 29bit ID (RRFE = 1)
                    (1U << 1) |     // Reject all "remote" frames with 11bit ID (RRFE = 1)
                    (0b11 << 2) |   // Reject non-matching frames (ANFE = 0b11) 11bit ID
                    (0b11 << 4) |   // Reject non matching 11 bit ID's
                    (2U << 16);     // Set LSS = 0b00010 (two standard filter)
                    
    // Configure Interrupt for RX FIFO 0 receive new message
    FDCAN2->ILS &= BIT(0);          // Assign RF0N to IT0 (clear bit 0) 
    FDCAN2->IE |= BIT(0);           // Rx FIFO 0 new message interrupt enable
    FDCAN2->ILE |= BIT(0);          // Enable IT0 line to NVIC (to enable IT1 set to 1))

    NVIC->ISER[0] |= (1 << 21);     // Enable FDCAN2 interrupt in NVIC (IRQ21)
    NVIC->IPR[21] = (0 << 6);       // Set NVIC interrupt priority

    // Start FDCAN operation
    FDCAN2->CCCR &= ~BIT(4);
    FDCAN2->CCCR &= ~BIT(3);
    FDCAN2->CCCR &= ~BIT(2);         // Clear ASM
    FDCAN2->CCCR &= ~BIT(1);         // Clear CCE bit to allow normal operation
    FDCAN2->CCCR &= ~BIT(0);         // Clear INIT bit to start operation

    counter_loop = 0;
    while (FDCAN2->CCCR & BIT(0))    // Wait until INIT bit is cleared
    {                                   
        if (counter_loop > 1000) {
            return false;
        }
        counter_loop++;
    }
    
    return true;                     // Return success
}



//  function for sending a standart CAN frame through FDCAN2 Interface
//  ------------------------------------
//  Parameters:  TxFrame: pointer to the CAN_TxBufferElement structure
//               containing the message to be sent
//  Returns:     1=success, 0=fail 
//  ------------------------------------

bool FDCAN2_Send_Std_CAN_Message(CAN_TxBufferElement *TxFrame) {
    FDCAN2->TXBC &= ~BIT(24); // TX fifo operatioon mode

    uint32_t txfqs = FDCAN2->TXFQS; // Get Tx FIFO queue status
    uint32_t free_level = (txfqs &= 0x7U); // Get free level (TFFL) of Tx FIFO
    
    if (free_level == 0) { // Check if Tx FIFO is empty
        return false; // Tx FIFO is full, cannot send message
    }
    
    uint8_t put_index = (txfqs >> 16) & 0x3U; // Put index (TFQPI)
    
    FDCAN2_TxBuffer[put_index] = *TxFrame; // Copy the message to the Tx buffer

    if (put_index == 0) FDCAN2->TXBAR |= BIT(0); // Set Tx buffer request pending (TFRP) for buffer 0
    else if (put_index == 1) FDCAN2->TXBAR |= BIT(1); // Set Tx buffer request pending (TFRP) for buffer 1
    else if (put_index == 2) FDCAN2->TXBAR |= BIT(2); // Set Tx buffer request pending (TFRP) for buffer 2
    return true; // Message sent successfully --> only true to if check if Tx FIFO is empty
    // Note: The function does not check for Tx FIFO full condition
    // and assumes that the caller has already checked the FIFO status. ---->>> need to check!!
    // need to check error status in main!
}




