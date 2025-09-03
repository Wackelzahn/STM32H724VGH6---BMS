#ifndef CAN_H
#define CAN_H

#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#include "registers.h"

// FDCAN Tx Buffer Element structure
typedef struct {
  uint32_t T0;
  uint32_t T1;
  uint8_t data[8];
} CAN_TxBufferElement;

// FDCAN Rx Buffer Element structure
typedef struct {
  uint32_t R0;
  uint32_t R1;
  uint8_t data[8];
} CAN_RxBufferElement;


// function declarations
bool FDCAN1_init(void);
bool FDCAN2_init(void);
bool FDCAN1_transmit_message(CAN_TxBufferElement *tx_frame);
bool FDCAN2_transmit_message(CAN_TxBufferElement *tx_frame);
// bool Init_FDCAN_INA228_Message(CAN_TxBufferElement *Frame);
// bool FDCAN2_Send_Std_CAN_Message(CAN_TxBufferElement *TxFrame);


#endif