#include "mbed.h"
#include "SHA256.h"
#include "motor.h"
#include "bitcoin.h"
#include "messages.h"
#include "string"

//########################################################
//------------------- Define Threads ---------------------
//########################################################
Thread MessengerThread;
Thread TerminaListenerThread;
Thread BitcoinThread;
Thread MotorThread;


//########################################################
//-------------------- Main Method -----------------------
//########################################################
int main() {

    MessengerThread.start(callback(outputToTerminal));
    TerminaListenerThread.start(callback(listenToTerminal));
    BitcoinThread.start(callback(computeHash));
    MotorThread.start(callback(MotorMain));

}


