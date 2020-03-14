#ifndef MESSAGES
#define MESSAGES

#include "mbed.h"

extern RawSerial pc;

typedef struct {
  uint8_t    printValue;
} mail_t;

void outputToTerminal();
extern void putMessage(uint8_t counter);
void listenToTerminal();

#endif