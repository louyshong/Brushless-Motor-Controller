#include "bitcoin.h"
#include "messages.h"

Mutex newKey_mutex;

Timer hashRateTimer;
int hashesCompleted = 0;

uint64_t totalCompleteHashes = 0;
Timer total_Hash_Rate_Timer;

//------------------- Bitcoin Mining ------------------------
void computeHash()
{
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)&sequence[48];
    uint64_t* nonce = (uint64_t*)&sequence[56];
    uint8_t hash[32];

    SHA256 hasher;
    
    hashRateTimer.start();
    total_Hash_Rate_Timer.start();

    while(true)
    {
        //complete 5000 hashes ASAP
        if(hashesCompleted < const_hashRate)
        {
            //assign new key
            newKey_mutex.lock();
            *key = newKey;
            newKey_mutex.unlock();

            hasher.computeHash(hash, sequence, 64);

            if(hash[0]==0 && hash[1]==0)
            {
                putMessage(BITCOIN_NONCE, (uint8_t)*nonce);
            }

            *nonce += 1;
            hashesCompleted ++;
            totalCompleteHashes ++;


        }
        else    // reset time and hashes complete if one second has passed
        {
            if(hashRateTimer.read_ms() >= 1000)
            {
                hashRateTimer.reset();
                hashesCompleted = 0;
            }
        }
        
        if(total_Hash_Rate_Timer.read() > 2.0)
        {
            double hashRate = totalCompleteHashes / (double)total_Hash_Rate_Timer.read();
            totalCompleteHashes = 0;
            //putMessage(HASH_RATE, hashRate);
            total_Hash_Rate_Timer.reset();
        }

    }
}

