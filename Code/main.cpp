#include "mbed.h"
#include "SHA256.h"
#include "motor.h"
#include "bitcoin.h"
#include "messages.h"
#include "string"
//#include "tune.h"

//########################################################
//------------------- Define Threads ---------------------
//########################################################
Thread MessengerThread;
Thread TerminaListenerThread;
Thread BitcoinThread;
Thread motorCtrlT(osPriorityHigh, 1024);
//Thread motorCtrlT;
//Thread TuneThread(osPriorityHigh, 1024);

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
    //TuneThread.start(callback(playTune_thread));

}


