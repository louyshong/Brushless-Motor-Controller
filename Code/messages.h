#ifndef MESSAGES
#define MESSAGES

#include "mbed.h"
#include "string"
#include "motor.h"
#include "bitcoin.h"

extern RawSerial pc;

typedef struct {
  double    printValue;
} mail_t;

void outputToTerminal();
extern void putMessage(double counter);
void listenToTerminal();

#endif