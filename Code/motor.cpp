#include "motor.h"

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

//##########################################################################
//------------------------------- CONSTANTS --------------------------------
//##########################################################################

//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
int8_t lead = 2;  //2 for forwards, -2 for backwards
 
//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);
 
//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
 
DigitalOut TP1(TP1pin);
PwmOut MotorPWM(PWMpin);

Mutex maxSpeed_mutex;
Mutex motorPower_mutex;
Mutex targetPosition_mutex;

//##########################################################################
//------------------------------- VARIABLES --------------------------------
//##########################################################################

int8_t intState = 0;
int8_t intStateOld = 0;
int8_t orState = 0;
int64_t position = 0;
int64_t positionOld = 0;
int8_t direction = 0;
double motorPower = 0; //controller output

float maxSpeed = 600; //set initial max speed to 100 rotations (600 segments) /s
double targetPosition = 0;

int32_t PWM_PRD = 2500;


//##########################################################################
//------------------------------- FUNCTIONS --------------------------------
//##########################################################################


//------------------------- Initilisation Functions ------------------------
//these functions are called at the beginin to set the motor state and position

//Set a given drive state
void motorOut(int8_t driveState)
{  
    //Lookup the output byte from the drive state. 
    int8_t driveOut = driveTable[driveState & 0x07];  // 0x07 = 111b, therefore this extracts the first 3 bits (0-7)
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}
    
//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState()
{
    return stateMap[I1 + 2*I2 + 4*I3];
}
 
//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

void InitialiseMotor() 
{  
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD);
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    I1.rise(&UpdateMotorPosition);
    I1.fall(&UpdateMotorPosition);
    I2.rise(&UpdateMotorPosition);
    I2.fall(&UpdateMotorPosition);
    I3.rise(&UpdateMotorPosition);
    I3.fall(&UpdateMotorPosition);
}


//------------------------- Control/Update Functions ------------------------
//these function changes the speed and velociy of the motor depending on user input

//Poll the rotor state and set the motor outputs accordingly to spin the motor
void UpdateMotorPosition()
{
    intState = readRotorState();
    int8_t direction_detected = intState - intStateOld;

    //Update position
    if (direction_detected > 0) {
        position++; 
    }
    if (direction_detected < 0) {
        position--;
    }

    //Update direction
    if ((direction_detected != direction) && (abs(direction_detected) < 5)) {
        direction = (direction_detected > 0) - (direction_detected < 0); 
    }

    intStateOld = intState;
    
    //Call motorOut
    motorOut((intState - orState + lead + 6) % 6); //+6 to make sure the remainder is positive    
}

//called to un-freeze thread
void motorCtrlTick()
{
    motorCtrlT.signal_set(0x1);
}

// set the motor power
void UpdateMotorPower()
{
    float controllerOutput;

        //read motor power
        motorPower_mutex.lock();
        controllerOutput = motorPower;
        motorPower_mutex.unlock();

        //setting limit for pulse width of PWM
        if (abs(controllerOutput) > PWM_PRD) 
            MotorPWM.pulsewidth_us(PWM_PRD);
        else 
            MotorPWM.pulsewidth_us(abs(controllerOutput));
        

        //lead = 2 * ((controllerOutput > 0) - (controllerOutput < 0)); //lead set to 2*sgn(y_s)

        lead = (controllerOutput > 0) ? 2 : -2;
}


//##########################################################################
//-------------------------------- THREAD ----------------------------------
//##########################################################################

void motorCtrlFn()
{
    //start the motor
    InitialiseMotor(); 

    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);

    Timer velocityTimer;
    velocityTimer.start();

    Timer printTimer;
    printTimer.start();

    while (1) 
    {
        motorCtrlT.signal_wait(0x1); //wait for signal (every 100ms)

        //calculate velocity
        double velocity = (position - positionOld) * (1 / (double)velocityTimer.read()); //calculating time taken for each revolution (this seems wrong, need to ask about it) 

        velocityTimer.reset();

        float y_s = 25*(maxSpeed - velocity);

        //print velocity
        if(printTimer.read() > 2.5)
        {
            putMessage(MOTOR_STATUS, maxSpeed, (float)velocity);
            printTimer.reset();
        }
        

        UpdateMotorPower();
    }
}


