#ifndef MESSAGES
#define MESSAGES

#include "mbed.h"
#include "string"
#include "motor.h"
#include "bitcoin.h"
#include "queue"
#include <regex>
#include "map"

extern RawSerial pc;

typedef struct {
  double    printValue;
  std::string message;
} mail_t;

// Threads
void outputToTerminal();
void listenToTerminal();

// Public Method
extern void putMessage(double counter);
extern void putMessage(std::string message);

// Variables that are updates by the decoder and accessible to the rest of the system
uint64_t newKey;
std::queue<std::string> tonesQ;
float maxSpeed; //set initial max speed to 100 rotations (600 segments) /s
double targetPosition;

// Mutex declarations
Mutex motorPower_mutex;
Mutex maxSpeed_mutex;
Mutex targetPosition_mutex;
Mutex newKey_mutex;
Mutex tonesQ_mutex;

#endif