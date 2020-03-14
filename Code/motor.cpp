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
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};
 
//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed
 
//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards
 
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
inline int8_t readRotorState(){
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

int8_t intState = 0;
int8_t intStateOld = 0;
int8_t orState = 0;
int64_t position = 0;
int64_t positionOld = 0;
int8_t direction = 0;

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


void InitialiseMotor() 
{  
    const int32_t PWM_PRD = 2500;
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD);
    
    //Initialise the serial port
    //Serial pc(SERIAL_TX, SERIAL_RX);
    pc.printf("Hello\n\r");
    
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

//Thread motorCtrlT (osPriorityNormal,1024);

void motorCtrlTick()
{
    motorCtrlT.signal_set(0x1);
 }


void motorCtrlFn()
{
    //start the motor
    InitialiseMotor(); 

    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);

    Timer velocityTimer;
    velocityTimer.start();

    while (1) {
        motorCtrlT.signal_wait(0x1); //wait for signal (every 100ms)

        //calculate velocity
        double velocity = (position - positionOld) * (1 / (double)velocityTimer.read()); //calculating time taken for each revolution (this seems wrong, need to ask about it) 
 
        //incremement counter
        motorCtrlCounter++;
        
        //calculate y_r
        targetPosition_mutex.lock();
        double posError = targetPosition - position;
        double posErrorOld = targetPosition - positionOld;
        double y_r = (85*posError) + (15*(posError - posErrorOld)/((double)velocityTimer.read()));
        
        double vError = (((posError > 0) - (posError < 0))*(maxSpeed) - velocity);
        
        double intvError =+ vError/0.1;

        if(intvError > 880) intvError = 880;
        if(intvError < -880) intvError = -880;
        
        //calculate y_s
        maxSpeed_mutex.lock();
        double y_s= (15 * vError)+(15*intvError);
        
        if (motorCtrlCounter == 50) { //print measurements every 5s
            motorCtrlCounter = 0; //reset counter
            putMessageM(direction, (velocity/6), (maxSpeed/6), (targetPosition/6), (float(position)/6)); //output message
        }
        
        targetPosition_mutex.unlock();
        maxSpeed_mutex.unlock();
        
        //set motorPower
        motorPower_mutex.lock();
        if (velocity>=0){
            motorPower = std::min(y_s,y_r);
        }
        else{
            motorPower = std::max(y_s,y_r);
        }
        motorPower_mutex.unlock();
        
        velocityTimer.reset();
        positionOld = position;
}


