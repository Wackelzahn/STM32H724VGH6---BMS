#ifndef VECAN_H
#define VECAN_H

#include <inttypes.h>
#include "can.h"

// Declare the CAN message structures as extern

extern CAN_TxBufferElement threefiftysix;


void VECan_Init ();
void VECan_send ();
void update_can_message_356(CAN_TxBufferElement* msg_356, 
                            volatile int32_t current_mA, 
                            volatile int32_t bus_voltage_mV, 
                            volatile int32_t temperature_centidegC);

#endif