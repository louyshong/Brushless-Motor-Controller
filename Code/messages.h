#ifndef MESSAGES
#define MESSAGES

#include "mbed.h"
#include "string"
#include "motor.h"
#include "bitcoin.h"
#include "melodyTune.h"
#include "queue"
#include <regex>
#include "map"

#define PRINT_MESSAGE 0
#define BITCOIN_NONCE 1
#define UPDATED_KEY 2
#define MOTOR_STATUS 3
#define ERROR_STATUS 4
#define HASH_RATE 5

extern RawSerial pc;

typedef struct {
  uint8_t   type;
  uint8_t    number;
  float    dFloat;
  float    dFloat1;
  float    dFloat2;
  std::string message;
} mail_t;

// Threads
void outputToTerminal();
void decodeInputThread();

// Public Method
extern void putMessage(int type, uint8_t number);
extern void putMessage(int type, double number);
extern void putMessage(int type, std::string message);
extern void putMessage(int type, double posError, double velError);



#endif