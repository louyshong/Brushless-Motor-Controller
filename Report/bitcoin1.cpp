while (true) {
    BitcoinThread.signal_wait(0x1);
    for(int i = 0; i < 5000; i++) {
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        sha256.computeHash(hash, sequence, 64);
        *nonce +=1;
    
        if(hash[0] == 0 && hash[1] == 0) {
            uint64_t numbernonce = *nonce;
            putMessage(numbernonce);
        }
    }
}