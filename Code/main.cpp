#include "mbed.h"
#include "SHA256.h"
#include "motor.h"
#include "bitcoin.h"
#include "messages.h"
#include "string"
#include "melodyTune.h"

//########################################################
//------------------- Define Threads ---------------------
//########################################################
Thread MessengerThread;
Thread TerminaListenerThread;
Thread BitcoinThread(osPriorityLow, 1024);
Thread motorCtrlT(osPriorityHigh, 1024);
//Thread motorCtrlT;
Thread TuneThread;

//########################################################
//-------------------- Main Method -----------------------
//########################################################
int main() 
{
    //start threads
    MessengerThread.start(callback(outputToTerminal));
    TerminaListenerThread.start(callback(decodeInputThread));
    BitcoinThread.start(callback(computeHash));
    motorCtrlT.start(callback(motorCtrlFn));
    TuneThread.start(callback(playTune_thread));

}


