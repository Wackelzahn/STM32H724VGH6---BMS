

#include "conversion.h"


int lenghtofarray(char *chararray) 
  {
    int i = 0;
    while (chararray[i] != '\0') {
      i++;
    }
    return i;
  }

int int2char(uint32_t num, char *buf) 
  {
    int i = 0;
    do {
      buf[i++] = (char)(num % 10 + '0');
    } while ((num /= 10) > 0);
    buf[i] = '\0';
    int j = 0;
    char temp;
    for (j = 0; j < i / 2; j++) {
      temp = buf[j];
      buf[j] = buf[i - j - 1];
      buf[i - j - 1] = temp;
    }
    return i;
  }