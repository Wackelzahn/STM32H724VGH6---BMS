
#ifndef CONVERSION_H
#define CONVERSION_H

#include <inttypes.h>

// Prototype function for URAT Character transformation

// Calculate the length of a char array
int lenghtofarray(char *chararray);     

// Convert an integer to a char array, returns the length of the char array
int int2char(uint32_t num, char *buf);  


#endif