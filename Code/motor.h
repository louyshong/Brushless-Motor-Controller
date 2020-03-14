#ifndef MOTOR
#define MOTOR

#include "mbed.h"
#include "messages.h"
#include "decode.h"


//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5
 
//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11
 
//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6          //0x08
#define L3Lpin D10           //0x10
#define L3Hpin D2          //0x20
 
#define PWMpin D9
 
//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0
 
//Test outputs
#define TP0pin D4
#define TP1pin D13
#define TP2pin A2

extern Thread motorCtrlT;
extern PwmOut MotorPWM;

extern int8_t intState;
extern int8_t intStateOld;
extern int8_t orState;
extern int64_t position;
extern int64_t positionOld;
extern int8_t direction;

//Main thread to instantiate
void motorCtrlFn();

#endif