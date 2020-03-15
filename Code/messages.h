#ifndef MESSAGES
#define MESSAGES

#include "mbed.h"
#include "string"
#include "motor.h"
#include "bitcoin.h"
#include "queue"
#include <regex>
#include "map"

#define PRINT_MESSAGE 0
#define BITCOIN_NONCE 1
#define UPDATED_KEY 2
#define MOTOR_STATUS 3

extern RawSerial pc;

typedef struct {
  uint8_t   type;
  uint8_t    number;
  float    dFloat;
  float    dFloat1;
  std::string message;
} mail_t;

// Threads
void outputToTerminal();
void decodeInputThread();

// Public Method
extern void putMessage(int type, uint8_t number);
extern void putMessage(int type, double number);
extern void putMessage(int type, std::string message);
extern void putMessage(int type, double targetVel, double actualVel);

// Variables that are updates by the decoder and accessible to the rest of the system
//uint64_t newKey;  
//std::queue<std::string> tonesQ;
//float maxSpeed; //set initial max speed to 100 rotations (600 segments) /s
//double targetPosition;


#endif