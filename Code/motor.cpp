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
 
//DigitalOut TP1(TP1pin);
PwmOut MotorPWM(PWMpin);

Mutex maxSpeed_mutex;
Mutex motorPower_mutex;
Mutex targetPosition_mutex;

float MAX_PWM = 1.0;

// Mutex power_mutex;
// float power_test = 0;

//##########################################################################
//------------------------------- VARIABLES --------------------------------
//##########################################################################

int8_t intState = 0;
int8_t intStateOld = -1;// set to -1 to get the motor moving
int8_t orState = 0;
int64_t position = 0;
int64_t positionOld = 0;
int8_t direction = 0;
float motorPower = 0; //controller output

// -------- Values from terminal input --------
float maxSpeed = 600; //set initial max speed to 100 rotations (600 segments) /s
double targetPosition = 0;

//------------- Control Variables ------------
// velocity error
float acc_v_error = 0.0;
//float MAX_V_ERROR = 880;
float MAX_V_ERROR = 1000;
float Kps = 0.014;  //scales error
float Kis = 0.0008; //dampens error
float VE = 0.0;

//distance error
float Kpr = 0.09;    //distance error constant
float Kdr = 0.09;   //dampens error
float DE = 0.0;

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


//------------------------- Control/Update Functions ------------------------
//these function changes the speed and velociy of the motor depending on user input

//Poll the rotor state and set the motor outputs accordingly to spin the motor
void UpdateMotorPosition()
{
    //TP1 = 1;
    intState = readRotorState();
    int8_t direction_detected = intState - intStateOld;

    //Update position
    if (direction_detected == 5) position--;
    else if (direction_detected == -5) position++;
    else position += (direction_detected);

    //Update direction
    if ((direction_detected != direction) && (abs(direction_detected) < 5)) {
        direction = (direction_detected > 0) - (direction_detected < 0); 
    }

    intStateOld = intState;
    
    //Call motorOut
    motorOut((intState - orState + lead + 6) % 6); //+6 to make sure the remainder is positive 
    //TP1 = 0;   
}

//called to un-freeze thread
void motorCtrlTick()
{
    motorCtrlT.signal_set(0x1);
}

// set the motor power
void UpdateMotorPower(float power)
{
    power = abs(power);
    if(power > 1.0) power = 1.0;

    MotorPWM.write(power);
}

void InitialiseMotor() 
{  
    MotorPWM.period_us(2500);
    MotorPWM.write(MAX_PWM);
    
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

    //get field in correct start position
    UpdateMotorPosition();
}

// calculate the error due to speed difference
float VelocityError(double currentSpeed)
{
    //update error term
    targetPosition_mutex.lock();
    maxSpeed_mutex.lock();
    float vError = (maxSpeed - currentSpeed);
    maxSpeed_mutex.unlock();
    targetPosition_mutex.unlock();

    //update integral error
    acc_v_error += vError * 10;
    if(acc_v_error > MAX_V_ERROR) acc_v_error = MAX_V_ERROR;
    if(acc_v_error < -MAX_V_ERROR) acc_v_error = -MAX_V_ERROR;

    //return error due to velocity delta
    //double errorVal = Kps*vError + Kis*acc_v_error;
    
    return Kps*vError + Kis*acc_v_error;
}

float DistanceError(double timePassed)
{
    // get current and previous distacne error
    targetPosition_mutex.lock();

    double posError = (targetPosition - position);
    double posErrorOld = targetPosition - positionOld;

    targetPosition_mutex.unlock();

    return (Kpr*posError) + (Kdr*(posError - posErrorOld)/(timePassed));
}


//##########################################################################
//-------------------------------- THREAD ----------------------------------
//##########################################################################

void motorCtrlFn()
{
    //start the motor
    InitialiseMotor(); 

    //start the ticker to execute the while loop once every 100ms
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);

    Timer velocityTimer;
    velocityTimer.start();

    Timer printTimer;
    printTimer.start();


    while (1) 
    {
        motorCtrlT.signal_wait(0x1); //wait for signal (every 100ms)

        // calculate velocity
        double timeElapsed = velocityTimer.read();
        velocityTimer.reset();
        double velocity = (position - positionOld) * (1 / timeElapsed);

        //find distance error
        DE = DistanceError(timeElapsed);

        // add non-linearity to prevent motor oscillation at small speeds (kills the error is within 0.5 rots)
        if(abs(targetPosition - position) < 3)
        {
            DE = 0;
        }

        //find velocity error
        VE = VelocityError(abs(velocity));
        if(VE < 0) VE = 0;

        //choose error to pass to motor
        float power_test = std::min(abs(VE), abs(DE));

        //print rotations and speed
        if(printTimer.read() > 1.0)
        {
            putMessage(SPEED_STATUS, (maxSpeed/6), (velocity/6));
            putMessage(DISTANCE_STATUS, (targetPosition/6), (position/6));

            printTimer.reset();
        }
        
        //update/reset variables
        lead = (DE >= 0) ? 2 : -2;

        //update/reset variables
        positionOld = position;
        UpdateMotorPower(power_test);

    }
}


