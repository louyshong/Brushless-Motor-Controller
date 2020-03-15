#ifndef BITCOIN_MINER
#define BITCOIN_MINER

#include "mbed.h"
#include "SHA256.h"

void computeHash();

extern Mutex newKey_mutex;
extern volatile uint64_t newKey;


#endif
