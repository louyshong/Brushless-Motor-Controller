#ifndef BITCOIN_MINER
#define BITCOIN_MINER

#include "mbed.h"
#include "SHA256.h"

void computeHash();

extern Mutex newKey_mutex;
extern volatile uint64_t newKey;
extern Thread BitcoinThread;

const int CONST_HASH_RATE = 5000;  // hashes to complete per-second

#endif
