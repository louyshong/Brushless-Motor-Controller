while (true) {
    BitcoinThread.signal_wait(0x1);
    //CONST_HASH_RATE = 5000
    for(int i = 0; i < CONST_HASH_RATE; i++) {
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        hasher.computeHash(hash, sequence, 64);
        *nonce +=1;

        if(hash[0] == 0 && hash[1] == 0) {
            putMessage(BITCOIN_NONCE, (uint8_t)*nonce);
        }
    }
}
