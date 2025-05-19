#ifndef CAN_H
#define CAN_H

#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include "registers.h"



// function declarations
bool Can_Init(void);
void FDCAN2_Send_Std_CAN_Message(void);


#endif