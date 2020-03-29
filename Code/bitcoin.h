#ifndef BITCOIN_MINER
#define BITCOIN_MINER

#include "mbed.h"
#include "SHA256.h"

void computeHash();

extern Mutex newKey_mutex;
extern volatile uint64_t newKey;

const int const_hashRate = 5000;  // hashes to complete per-second

#endif
