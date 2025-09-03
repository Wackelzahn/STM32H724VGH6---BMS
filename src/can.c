
// Created: 2025-08-19 07:00:00
// Author: M. Schneider
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


#include "can.h"
#include "registers.h"

#define BIT(x) (1UL << (x)) // Bit banging
uint32_t counter_loop = 0;   // counter for waiting loops


//   Functions for initializing the FDCAN1 & FDCAN2 Interface for STM32H742VGH6
//   FDCAN1_RX on PB5, FDCAN1_TX on PB6
//   FDCAN2_RX on PD0, FDCAN2_TX on PD1
//   Target bitrate: 500 Kbps
//  ------------------------------------
//  Parameters:  none
//  Returns:     1=success, 0=fail 
//  ------------------------------------



bool FDCAN1_init(void){
    
    // Enable FDCAN peripheral clock in APB1H
    RCC->APB1HENR |= (1 << 8);       // FDCAN clock enable (bit 8 in APB1HENR)
    RCC->D2CCIP1R &= ~(0x3U << 28);    // Clear bits -> (00) = clock source HSE (16MHz)
    
    // Enable GPIOB clock
    RCC->AHB4ENR |= (1 << 3); // Enable GPIOD clock

    // Configure FDCAN1 RX (PD0) and TX (PD1)
    GPIOD->MODER &= ~((3UL << 0)|(3UL << 2));         // Clear mode bits for PD0 & PD1
    GPIOD->MODER |= ((2UL << 0)|(2UL << 2));          // Set to alternate function mode
    GPIOD->AFR[0] &= ~((0xFUL << 0) | (0xFUL << 4));  // clear AF bits for PB5 & PB6
    GPIOD->AFR[0] |= (0x9U << 0) | (0x9U << 4);       // Set AF9 for FDCAN1

    // Enter initialization Mode
    FDCAN1->CCCR |= BIT(0); // Set INIT bit to enter initialization mode
    counter_loop = 0; // Reset counter for waiting loops
    while (!(FDCAN1->CCCR & BIT(0)))
    {
        if (counter_loop > 1000) {
            return false; // Initialization failed      
        }
        counter_loop++;
    }

    FDCAN1->CCCR |= BIT(1);     // Set CCE bit to enable configuration changes
    
    FDCAN1->CCCR &= ~BIT(8);    // FD Operation disabled, normal CAN mode
    FDCAN1->CCCR &= ~BIT(9);    // no bitrate switching
    FDCAN1->CCCR &= ~BIT(5);    // disable oring
    FDCAN1->CCCR |= BIT(6);     // disable automatic re-transmission
    FDCAN1->CCCR &= ~BIT(7);    // disable test mode
    FDCAN1->CCCR |= BIT(12);    // disable protocol exception handling
    FDCAN1->CCCR &= ~BIT(15);   // disable ISO 11898-1:2015 mode
    FDCAN1->CCCR |= BIT(14);    // enable transmit pause
    FDCAN1->CCCR &= ~BIT(2);    // normal can operation



    // Set the bit timing parameters (500kbps)
    // Prescaler (NBRP) = 25, Time segment 1 (NTSEG1) = 13, Time segment 2 (NTSEG2) = 2, SJW = 2
    // Bit rate = Fdcan_clk / (Prescaler * (1 + TSEG1 + TSEG2))
    // Assuming Fdcan_clk = 16 MHz HSE, Bit rate = 16MHz / (25 * (1 + 13 + 2)) = 500 kbps
    FDCAN1->NBTP =  (1UL << 0) |        // NTSEG2 = 1 (2 - 1)
                    (12UL << 8) |       // NTSEG1 = 12 (13 - 1)
                    (1UL << 16) |       // NBRP = 1 (2 - 1)
                    (1UL << 25);        // NSJW = 1 (2 - 1)

  
    // Clear FDCAN1 message RAM
    uint32_t *fdcan1_ram = (uint32_t *)FDCAN_MESSAGE_RAM_BASE;
    for (int i = 0; i < 634; i++) { // 2536 bytes = 634 words
    fdcan1_ram[i] = 0;
    } 

    // Set filter to accept messages
    // Filter Element 0: ID 0x426 -> RX FIFO 0
    FDCAN1_StandartMessageIDFilter[0] = (0x1U << 30) |      // SFT = 01 (Dual ID filter)
                                        (0x1U << 27) |      // SFEC = 001 (Store in RX FIFO 0)
                                        (0x426U << 16) |    // SFID1 = 0x42E (bits 26:16)
                                        (0x426U << 0);      // SFID2 = 0x42E (bits 10:0)
    // Filter Element 1: ID 0x427 -> RX FIFO 0  
    FDCAN1_StandartMessageIDFilter[1] = (0x1U << 30) |      // SFT = 01 (Dual ID filter)
                                        (0x1U << 27) |      // SFEC = 001 (Store in RX FIFO 0)
                                        (0x427U << 16) |    // SFID1 = 0x128 (bits 26:16)
                                        (0x427U << 0);      // SFID2 = 0x128 (bits 10:0)                                
     
   // Filter Element 1: ID 0x428 - 0x42E -> RX FIFO 0  
    FDCAN1_StandartMessageIDFilter[2] = (0x1U << 30) | (0x1U << 27) | (0x428U << 16) | (0x428U << 0);        
    FDCAN1_StandartMessageIDFilter[3] = (0x1U << 30) | (0x1U << 27) | (0x429U << 16) | (0x429U << 0);     
    FDCAN1_StandartMessageIDFilter[4] = (0x1U << 30) | (0x1U << 27) | (0x42AU << 16) | (0x42AU << 0);     
    FDCAN1_StandartMessageIDFilter[5] = (0x1U << 30) | (0x1U << 27) | (0x42BU << 16) | (0x42BU << 0);     
    FDCAN1_StandartMessageIDFilter[6] = (0x1U << 30) | (0x1U << 27) | (0x42CU << 16) | (0x42CU << 0);   
    FDCAN1_StandartMessageIDFilter[7] = (0x1U << 30) | (0x1U << 27) | (0x42DU << 16) | (0x42DU << 0);
    FDCAN1_StandartMessageIDFilter[8] = (0x1U << 30) | (0x1U << 27) | (0x42EU << 16) | (0x42EU << 0);    

    // Configure Global filter settings    
    // Reject remote frames, reject extended frames, reject non-matching frames                                
    FDCAN1->GFC = (0x1U << 0) |     // RRFE = 1 (Reject remote frames extended)
                  (0x1U << 1) |     // RRFS = 1 (Reject remote frames standart)
                  (0x3U << 2) |     // ANFE = 11 (Reject non-matching frames extended)
                  (0x3U <<4);       // ANFS = 00 (Reject non-matching frames standard)

    // Configure FDCAN1 message RAM
    // Configure the Element Size of the RX FIFO to 4 words (8 byte data)
    FDCAN1->RXESC &= ~(0x7U << 0);                          // F0DS = 0 (8 byte data) FIFO 1 -> maybe typo in the reference manual
    FDCAN1->RXESC &= ~(0x7U << 4);                          // F1DS = 0 (8 byte data) FIFO 0
    //Configure the Element Size of the TX Buffer to 4 words (8 byte data)
    FDCAN1->TXESC &= ~(0x7U << 0);                          // TBS = 0 (8 byte data)    

    // Configure Standard ID Filter location and size
    FDCAN1->SIDFC = (9 << 16) |                            // LSS = 28 filter elements
                    ((FDCAN_Std_Filter_RAM_offset/4) << 2); // FLESA = 0x0000/4 = 0x00
    // Configure Extended ID Filter location and size 
    FDCAN1->XIDFC = (8 << 16) |                             // LSS = 8 filter elements
                    ((FDCAN_Ext_Filter_RAM_offset/4) << 2); // FLESA = 0x0070/4 = 0x1C
    // Configure RX FIFO 0 location and size
    FDCAN1->RXF0C = (18 << 16) | (1U << 31) |                // F0S = 18 filter elements, Settin in Overwrite mode
                    ((FDCAN_RX_FIFO0_offset/4) << 2);       // F0SA = 0x00B0/4 = 0x2C
    // Configure RX FIFO 1 location and size
    FDCAN1->RXF1C = (18 << 16) | (1U << 31) |                // F1S = 18 filter elements, Settin in Overwrite mode
                    ((FDCAN_RX_FIFO1_offset/4) << 2);       // F1SA = 0x01D0/4 = 0x74
    // Configure RX Buffer location and size
    FDCAN1->RXBC =  ((FDCAN_RX_BUFFER_offset/4) << 2);      // F0SA = 0x02F0/4 = 0xBC
    // Configure TX Event FIFO location and size
    FDCAN1->TXEFC = (3 << 16) |                             // TEFS = 3 event FIFO elements
                    ((FDCAN_TX_EVENT_FIFO_offset/4) << 2);  // F0SA = 0x0890/4 = 0x224
    // Configure TX Buffer location and size
    FDCAN1->TXBC = (18 << 24) |  // TFQS = 18 (FIFO size)
                    (0 << 16) |   // NDTB = 0 (no dedicated buffers)
                    (0 << 30) |   // TFQM = 0 (FIFO mode)
                    ((FDCAN_TX_BUFFER_offset/4) << 2);


    // Configure Interrupt for RX FIFO 0 new message
    FDCAN1->IE |= BIT(0); // Enable RX FIFO 0 new message interrupt
    FDCAN1->ILS &= ~BIT(0); // Set RX FIFO 0 new message interrupt line 0
    FDCAN1->ILE |= BIT(0); // Enable interrupt line 0

    // Enable NVIC interrupt for FDCAN1
    NVIC->ISER[0] |= (1 << 19);        // FDCAN1_IT0_IRQn = 19,

    // Start FDCAN1 and exit initialization mode
    FDCAN1->CCCR &= ~BIT(4);
    FDCAN1->CCCR &= ~BIT(3);
    FDCAN1->CCCR &= ~BIT(2);
    FDCAN1->CCCR &= ~BIT(1); // Clear CCE bit to disable configuration changes
    FDCAN1->CCCR &= ~BIT(0); // Clear INIT bit to exit initialization mode

    counter_loop = 0; // Reset counter for waiting loops
    while (FDCAN1->CCCR & BIT(0))  // Wait until INIT bit is cleared
    {
        if (counter_loop > 1000) {
            return false; // Initialization failed      
        }
        counter_loop++;
    };

    return true; // Initialization successful
}

bool FDCAN2_init(void){
    
    // Enable FDCAN peripheral clock in APB1H
    RCC->APB1HENR |= (1 << 8);       // FDCAN clock enable (bit 9 in APB1HENR)
    RCC->D2CCIP1R &= ~(0x3U << 28);    // Clear bits -> (00) = clock source HSE (16MHz)
    
    // Enable GPIOB clock (for PB5/PB6 pins)
    RCC->AHB4ENR |= (1 << 1); // Enable GPIOB clock

    // Configure FDCAN2 RX (PB5) and TX (PB6)
    GPIOB->MODER &= ~((3UL << 10)|(3UL << 12));         // Clear mode bits for PB5 & PB6
    GPIOB->MODER |= ((2UL << 10)|(2UL << 12));          // Set to alternate function mode
    GPIOB->AFR[0] &= ~((0xFUL << 20) | (0xFUL << 24));  // clear AF bits for PB5 & PB6
    GPIOB->AFR[0] |= (0x9U << 20) | (0x9U << 24);       // Set AF9 for FDCAN2

    // Enter initialization Mode
    FDCAN2->CCCR |= BIT(0); // Set INIT bit to enter initialization mode
    counter_loop = 0; // Reset counter for waiting loops
    while (!(FDCAN2->CCCR & BIT(0)))
    {
        if (counter_loop > 1000) {
            return false; // Initialization failed      
        }
        counter_loop++;
    }

    FDCAN2->CCCR |= BIT(1);     // Set CCE bit to enable configuration changes
    
    FDCAN2->CCCR &= ~BIT(8);    // FD Operation disabled, normal CAN mode
    FDCAN2->CCCR &= ~BIT(9);    // no bitrate switching
    FDCAN2->CCCR &= ~BIT(5);    // disable oring
    FDCAN2->CCCR |= BIT(6);     // disable automatic re-transmission
    FDCAN2->CCCR &= ~BIT(7);    // disable test mode
    FDCAN2->CCCR |= BIT(12);    // disable protocol exception handling
    FDCAN2->CCCR &= ~BIT(15);   // disable ISO 11898-1:2015 mode
    FDCAN2->CCCR |= BIT(14);    // enable transmit pause
    FDCAN2->CCCR &= ~BIT(2);    // normal can operation

    // Set the bit timing parameters (500kbps) - same as FDCAN1
    // Prescaler (NBRP) = 2, Time segment 1 (NTSEG1) = 13, Time segment 2 (NTSEG2) = 2, SJW = 2
    // Bit rate = Fdcan_clk / (Prescaler * (1 + TSEG1 + TSEG2))
    // Assuming Fdcan_clk = 16 MHz HSE, Bit rate = 16MHz / (2 * (1 + 13 + 2)) = 500 kbps
    FDCAN2->NBTP =  (1UL << 0) |        // NTSEG2 = 1 (2 - 1)
                    (12UL << 8) |       // NTSEG1 = 12 (13 - 1)
                    (1UL << 16) |       // NBRP = 1 (2 - 1)
                    (1UL << 25);        // NSJW = 1 (2 - 1)

    // Clear FDCAN2 message RAM (uses different offset)
    uint32_t *fdcan2_ram = (uint32_t *)(FDCAN_MESSAGE_RAM_BASE + FDCAN2_MESSAGE_RAM_BASE_offset);
    for (int i = 0; i < 634; i++) { // 2536 bytes = 634 words
        fdcan2_ram[i] = 0;
    }              

    // Set filter to accept messages (different IDs for FDCAN2)
    // Filter Element 0: ID 0x500 -> RX FIFO 0
    FDCAN2_StandartMessageIDFilter[0] = (0x1U << 30) |      // SFT = 01 (Dual ID filter)
                                        (0x1U << 27) |      // SFEC = 001 (Store in RX FIFO 0)
                                        (0x500U << 16) |    // SFID1 = 0x500 (bits 26:16)
                                        (0x500U << 0);      // SFID2 = 0x500 (bits 10:0)
    // Filter Element 1: ID 0x501 -> RX FIFO 0  
    FDCAN2_StandartMessageIDFilter[1] = (0x1U << 30) |      // SFT = 01 (Dual ID filter)
                                        (0x1U << 27) |      // SFEC = 001 (Store in RX FIFO 0)
                                        (0x501U << 16) |    // SFID1 = 0x501 (bits 26:16)
                                        (0x501U << 0);      // SFID2 = 0x501 (bits 10:0)     

    // Filter Elements 2-8: ID 0x502 - 0x508 -> RX FIFO 0  
    FDCAN2_StandartMessageIDFilter[2] = (0x1U << 30) | (0x1U << 27) | (0x502U << 16) | (0x502U << 0);        
    FDCAN2_StandartMessageIDFilter[3] = (0x1U << 30) | (0x1U << 27) | (0x503U << 16) | (0x503U << 0);     
    FDCAN2_StandartMessageIDFilter[4] = (0x1U << 30) | (0x1U << 27) | (0x504U << 16) | (0x504U << 0);     
    FDCAN2_StandartMessageIDFilter[5] = (0x1U << 30) | (0x1U << 27) | (0x505U << 16) | (0x505U << 0);     
    FDCAN2_StandartMessageIDFilter[6] = (0x1U << 30) | (0x1U << 27) | (0x506U << 16) | (0x506U << 0);   
    FDCAN2_StandartMessageIDFilter[7] = (0x1U << 30) | (0x1U << 27) | (0x507U << 16) | (0x507U << 0);
    FDCAN2_StandartMessageIDFilter[8] = (0x1U << 30) | (0x1U << 27) | (0x508U << 16) | (0x508U << 0); 

    // Configure Global filter settings    
    // Reject remote frames, reject extended frames, reject non-matching frames                                
    FDCAN2->GFC = (0x1U << 0) |     // RRFE = 1 (Reject remote frames extended)
                  (0x1U << 1) |     // RRFS = 1 (Reject remote frames standart)
                  (0x3U << 2) |     // ANFE = 11 (Reject non-matching frames extended)
                  (0x3U <<4);       // ANFS = 11 (Reject non-matching frames standard)

    // Configure FDCAN2 message RAM
    // Configure the Element Size of the RX FIFO to 4 words (8 byte data)
    FDCAN2->RXESC &= ~(0x7U << 0);                          // F0DS = 0 (8 byte data) FIFO 0
    FDCAN2->RXESC &= ~(0x7U << 4);                          // F1DS = 0 (8 byte data) FIFO 1
    //Configure the Element Size of the TX Buffer to 4 words (8 byte data)
    FDCAN2->TXESC &= ~(0x7U << 0);                          // TBS = 0 (8 byte data)    

    // Calculate word offset for FDCAN2 message RAM base
    uint32_t fdcan2_word_offset = FDCAN2_MESSAGE_RAM_BASE_offset / 4;
    
    // Configure Standard ID Filter location and size
    FDCAN2->SIDFC = (9 << 16) |                                    // LSS = 9 filter elements
                    ((fdcan2_word_offset + FDCAN_Std_Filter_RAM_offset/4) << 2); 
    // Configure Extended ID Filter location and size 
    FDCAN2->XIDFC = (8 << 16) |                                     // LSS = 8 filter elements
                    ((fdcan2_word_offset + FDCAN_Ext_Filter_RAM_offset/4) << 2); 
    // Configure RX FIFO 0 location and size
    FDCAN2->RXF0C = (18 << 16) | (1U << 31) |                      // F0S = 18 elements, Overwrite mode
                    ((fdcan2_word_offset + FDCAN_RX_FIFO0_offset/4) << 2);       
    // Configure RX FIFO 1 location and size
    FDCAN2->RXF1C = (18 << 16) | (1U << 31) |                      // F1S = 18 elements, Overwrite mode
                    ((fdcan2_word_offset + FDCAN_RX_FIFO1_offset/4) << 2);       
    // Configure RX Buffer location and size
    FDCAN2->RXBC =  ((fdcan2_word_offset + FDCAN_RX_BUFFER_offset/4) << 2);      
    // Configure TX Event FIFO location and size
    FDCAN2->TXEFC = (3 << 16) |                                     // TEFS = 3 event FIFO elements
                    ((fdcan2_word_offset + FDCAN_TX_EVENT_FIFO_offset/4) << 2);  
    // Configure TX Buffer location and size
    FDCAN2->TXBC = (18 << 24) |  // TFQS = 18 (FIFO size)
                    (0 << 16) |   // NDTB = 0 (no dedicated buffers)
                    (0 << 30) |   // TFQM = 0 (FIFO mode)
                    ((fdcan2_word_offset + FDCAN_TX_BUFFER_offset/4) << 2);

    // Configure Interrupt for RX FIFO 0 new message
    FDCAN2->IE |= BIT(0); // Enable RX FIFO 0 new message interrupt
    FDCAN2->ILS &= ~BIT(0); // Set RX FIFO 0 new message interrupt line 0
    FDCAN2->ILE |= BIT(0); // Enable interrupt line 0

    // Enable NVIC interrupt for FDCAN2
    NVIC->ISER[0] |= (1 << 20);        // FDCAN2_IT0_IRQn = 20

    // Start FDCAN2 and exit initialization mode
    FDCAN2->CCCR &= ~BIT(4);
    FDCAN2->CCCR &= ~BIT(3);
    FDCAN2->CCCR &= ~BIT(2);
    FDCAN2->CCCR &= ~BIT(1); // Clear CCE bit to disable configuration changes
    FDCAN2->CCCR &= ~BIT(0); // Clear INIT bit to exit initialization mode

    counter_loop = 0; // Reset counter for waiting loops
    while (FDCAN2->CCCR & BIT(0))  // Wait until INIT bit is cleared
    {
        if (counter_loop > 1000) {
            return false; // Initialization failed      
        }
        counter_loop++;
    };

    return true; // Initialization successful
}







bool FDCAN1_transmit_message(CAN_TxBufferElement *tx_frame) {
    uint32_t txfqs = FDCAN1->TXFQS; // Read TX FIFO/queue status register
    uint32_t free_level = txfqs & 0x3F; // Get fill level
    uint32_t current_put_index = (txfqs >> 16) & 0x1F;  // TX FIFO Put Index (where to write next)
   

    // Check if there is space in the TX buffer
    if (free_level == 0) {    // Check if TX buffer is full (18 is the size of the TX buffer)
        return false;       // No space available
    }

    // Create a pointer to the TX buffer element in message RAM with put_index
    volatile CAN_TxBufferElement *tx_buffer = (CAN_TxBufferElement*) (FDCAN_MESSAGE_RAM_BASE 
        + FDCAN1_MESSAGE_RAM_BASE_offset + FDCAN_TX_BUFFER_offset + (current_put_index * sizeof(CAN_TxBufferElement)));

    // Copy the message to the TX buffer
    tx_buffer->T0 = tx_frame->T0;
    tx_buffer->T1 = tx_frame->T1;
    memcpy((void*)tx_buffer->data, tx_frame->data, 8);  // Copy 8 bytes safely

    // Request transmission
    FDCAN1->TXBAR |= (1U << current_put_index); // Set the corresponding bit in the Tx Buffer Add Request Register

    // Wait for put index to advance
    uint32_t timeout = 10000;
    uint32_t new_put_index;
    do {
        txfqs = FDCAN2->TXFQS;
        new_put_index = (txfqs >> 16) & 0x1F;
        timeout--;
    } while ((new_put_index == current_put_index) && (timeout > 0));    // Wait for put index to advance

    return (timeout > 0); // Message queued for transmission
}


// FDCAN2 transmit function
bool FDCAN2_transmit_message(CAN_TxBufferElement *tx_frame) {
    uint32_t txfqs = FDCAN2->TXFQS;
    uint32_t free_level = txfqs & 0x3F;
    uint32_t current_put_index = (txfqs >> 16) & 0x1F;

    if (free_level == 0) {
        return false;
    }

    // Create a pointer to the TX buffer element using your FDCAN2 macro
    volatile CAN_TxBufferElement *tx_buffer = &FDCAN2_TxBuffer[current_put_index];

    // Copy the message to the TX buffer
    tx_buffer->T0 = tx_frame->T0;
    tx_buffer->T1 = tx_frame->T1;

    // Copy data as 32-bit words (8 bytes = 2 words)
    ((uint32_t*)tx_buffer->data)[0] = ((uint32_t*)tx_frame->data)[0];
    ((uint32_t*)tx_buffer->data)[1] = ((uint32_t*)tx_frame->data)[1];

    // Request transmission
    FDCAN2->TXBAR |= (1U << current_put_index);

    // Wait for put index to advance
    uint32_t timeout = 10000;
    uint32_t new_put_index;
    do {
        txfqs = FDCAN2->TXFQS;
        new_put_index = (txfqs >> 16) & 0x1F;
        timeout--;
    } while ((new_put_index == current_put_index) && (timeout > 0));    // Wait for put index to advance
    
 
    return (timeout > 0); // Return remaining timeout (0 if timeout occurred)

}