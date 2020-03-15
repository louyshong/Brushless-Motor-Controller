#ifndef MELODY_TUNE
#define MELODY_TUNE

#include "mbed.h"
#include "motor.h"
#include "messages.h"
#include "queue"

void playTune_thread();

extern std::queue<std::string> tonesQ;
extern Mutex tonesQ_mutex;

#endif