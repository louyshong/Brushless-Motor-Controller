#include "mbed.h"
#include "SHA256.h"
#include "string"
#include "queue"
#include "map"
#include <regex>
#include <cmath>

//Photointerrupter in pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder in pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive out pins   //Mask in out byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6           //0x08
#define L3Lpin D10          //0x10
#define L3Hpin D2           //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Test outputs
#define TP0pin D4
#define TP1pin D13
#define TP2pin A2

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

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Set a given drive state
void motorOut(int8_t driveState) {
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
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

//Mail 
typedef struct {
  uint64_t nonce; // nonce number of successful hash
} mail_t;

//Mail
typedef struct {
    int8_t direction;/* direction of movement */
    double velocity; /* velocity of movement */
    float max_speed; /* maximum speed set by user */
    double target_position; /* target position set by user */
    float current_position; /* current position */
} mail_t2;


Mail<mail_t, 16> mail_box;
Mail<mail_t2, 16> mail_box2;
Mail<uint8_t, 16> inCharQ;

//Threads
Thread thread1;
Thread thread2;
Thread thread3(osPriorityHigh, 1024);
Thread thread4;
Thread write2serialT;
Thread motorCtrlT(osPriorityHigh, 1024);

void putMessage(uint64_t foundnonce) {
     mail_t *mail = mail_box.alloc();
     mail->nonce = foundnonce;
     mail_box.put(mail);
}

void putMessageM (int8_t direction, double velocity, float max_speed, double target_position, float current_position) {
    mail_t2 *mail = mail_box2.alloc();
    mail->direction = direction;
    mail->velocity = velocity;
    mail->max_speed = max_speed;
    mail->target_position = target_position;
    mail->current_position = current_position;
    mail_box2.put(mail);
}


uint64_t newKey;
std::queue<std::string> tonesQ;

void serialISR() {
     uint8_t* newChar = inCharQ.alloc();
     *newChar = pc.getc();
     inCharQ.put(newChar);
}

void write2serial_thread (void) {
    while (true) {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            pc.printf("N%016llx\n\r", mail->nonce);
            mail_box.free(mail);
        }
    }
}


void write2serial_M (void) {
    while (true) {
        osEvent evt = mail_box2.get();
        if (evt.status == osEventMail) {
            mail_t2* mail = (mail_t2*)evt.value.p;
            printf("Direction: %d \n\r", mail->direction);
            printf("Max Speed: %f \n\r", mail->max_speed);
            printf("Current Velocity: %f \n\r", mail->velocity);
            printf("Target Position: %f \n\r", (mail->target_position));
            printf("Current Position: %f \n\r", (mail->current_position));
            printf("\n\r");
            mail_box2.free(mail);
        }
    }
}

//Mutex
Mutex motorPower_mutex;
Mutex maxSpeed_mutex;
Mutex targetPosition_mutex;
Mutex newKey_mutex;
Mutex tonesQ_mutex;

//Position and velocity measurement variables

static int8_t orState = 0;    //Rotor offset at motor state 0
int8_t intStateOld = 0;
int8_t direction = 0;
int64_t position = 0;
int64_t positionOld = 0;
float maxSpeed = 600; //set initial max speed to 100 rotations (600 segments) /s
double targetPosition = 0;
double motorPower = 0; //controller output

void decodeCommand_thread () {

    pc.attach(&serialISR);
    std::string command;
    
    while (true) {
        osEvent newEvent = inCharQ.get();
        uint8_t* newChar = (uint8_t*)newEvent.value.p;
        //Store the new character
        command += (char)*newChar;
        inCharQ.free(newChar);
        //Decode the command if it is complete
        if (command.back() == '\r') {
            
            pc.printf("Input: %s\n\r", command.c_str());
            
        
            //Decode maximum speed command
            if (std::regex_match(command, std::regex("V(\\d){1,3}(\\.\\d+)?\u000D"))) { 
                float speed;
                char firstChar;
                sscanf(command.c_str(), "%c %f", &firstChar, &speed);
                maxSpeed_mutex.lock();
                maxSpeed = (speed*6);//convert input to segments per second
                maxSpeed_mutex.unlock();
                //pc.printf("Max Speed: %f\n\r", max_speed);
            }
            
            //Decode target position command
            if (std::regex_match(command, std::regex("R(-)?(\\d){1,4}(\\.\\d+)?\u000D"))){
                float rotation; 
                char firstChar;
                sscanf(command.c_str(), "%c %f", &firstChar, &rotation);
                targetPosition_mutex.lock();
                targetPosition = position + (rotation*6);//convert input to segments
                targetPosition_mutex.unlock();
            }
            
            if(command[0] == 'K') {
                char firstChar;
                uint64_t key;
                sscanf(command.c_str(), "%c %llx", &firstChar, &key); //llx formatter since uint64 is unsigned long long 
                pc.printf("New Key: %llx\n\r", key);
                newKey_mutex.lock();
                newKey = key;
                newKey_mutex.unlock();
            }
            
            if(command[0] == 'T') {
                char firstChar;
                std::string tone; 
                
                tonesQ_mutex.lock(); 
                
                //Empty queue
                while (!tonesQ.empty()) {
                    tonesQ.pop();    
                }
                
                for(int i = 1; i < command.length(); i++) {
                    if(command[i] == 'A' || command[i] == 'B' || command[i] == 'C' || command[i] == 'D' || 
                       command[i] == 'E' || command[i] == 'F' || command[i] == 'G') {
                        tone = "";
                        tone += command[i];
                    }   
                    if(command[i] == '#' || command[i] == '^') {
                        tone += command[i];                    
                    }
                    if(command[i] == '1' || command[i] == '2' || command[i] == '3' || command[i] == '4' ||
                       command[i] == '4' || command[i] == '5' || command[i] == '6' || command[i] == '7' ||
                       command[i] == '8') {
                        tone += command[i]; 
                        pc.printf("Tone: %s\n\r", tone.c_str()); 
                        
                        tonesQ.push(tone);
                    
                    }
                }
                tonesQ_mutex.unlock();
            }
            
            command = ""; //Reset command
        } 
    }
}

int32_t tunepwm;

void playTune_thread () {
    
    float interval = 125;
    char duration; 
    std::string tone;

    
    //Initialise map of tones and corresponding frequencies
    std::map<std::string, int32_t> tonefreqmap = {{"A", (int32_t)(1000000 / 440)}, {"A#", (int32_t)(1000000 / 466.16)}, {"A^", (int32_t)(1000000 / 415.30)}, {"B", (int32_t)(1000000 / 493.88)}, 
                                                  {"B^", (int32_t)(1000000 / 466.16)}, {"C", (int32_t)(1000000 / 262.63)}, {"C#", (int32_t)(1000000 / 277.18)}, {"D", (int32_t)(1000000 / 293.66)}, 
                                                  {"D#", (int32_t)(1000000 / 311.13)}, {"D^", (int32_t)(1000000 / 277.18)}, {"E", (int32_t)(1000000 / 329.63)}, {"E#", (int32_t)(1000000 / 349.23)}, 
                                                  {"E^", (int32_t)(1000000 / 311.13)}, {"F", (int32_t)(1000000 / 349.23)}, {"F#", (int32_t)(1000000 / 369.99)}, {"F^", (int32_t)(1000000 / 329.63)}, 
                                                  {"G", (int32_t)(1000000 / 392)}, {"G#", (int32_t)(1000000 / 415.30)}, {"G^", (int32_t)(1000000 / 369.99)}};
    
    while (true) {
        //Check what tone needs to be played
        if(!tonesQ.empty()) {
            tonesQ_mutex.lock();
            tone = tonesQ.front();
            tonesQ.pop();
            tonesQ.push(tone); //Spec wants tune to be looped
            tonesQ_mutex.unlock();
            duration = tone.back();
            
            //Set new PWM period
            if(tonefreqmap.find(tone.substr(0, 1)) != tonefreqmap.end()) {
                tunepwm = tonefreqmap.find(tone.substr(0, 1)) -> second;
            } else {
                tunepwm = tonefreqmap.find(tone.substr(0)) -> second;
            }
            //pc.printf("Tone: %s\n\r", tone.c_str());   
            //pc.printf("New PWM Period is: %d\n\r", tunepwm);
            MotorPWM.period_us(tunepwm);
            //MotorPWM.pulsewidth_us(0.5*tunepwm);
            interval = ((int)duration - (int)'0') * 125; //Reset PWM period after tone duration has ended
        }
        
        ThisThread::sleep_for(interval);
    }
}

void bitcoinTick(){
    thread4.signal_set(0x1);
}

void bitcoin_thread () {
    Ticker bitcoinTicker;
    bitcoinTicker.attach(&bitcoinTick, 1);
    SHA256 sha256;

    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)&sequence[48];
    uint64_t* nonce = (uint64_t*)&sequence[56];
    uint8_t hash[32];
    
    int hashComplete = 0;
    
    while (true) {
        
        thread4.signal_wait(0x1);
        for(int i = 0; i < 5000; i++) {
            newKey_mutex.lock();
            *key = newKey;
            newKey_mutex.unlock();
            sha256.computeHash(hash, sequence, 64);
            *nonce +=1;
            hashComplete++;
        
            if(hash[0] == 0 && hash[1] == 0) {
                uint64_t numbernonce = *nonce;
                putMessage(numbernonce);
            }
        }
        pc.printf("Nonces per second: %d\n\r", hashComplete);
        hashComplete = 0;
    }
}

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}

int8_t intState; 



//---------------------------------------------------- MOTOR -------------------------------------------------------




//ISR Update Motor Position
void UpdateMotorPos() {
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

//Functions to Control Motor

int64_t motorCtrlCounter = 0;

void motorCtrlTick() {
    motorCtrlT.signal_set(0x1);
}

void motorCtrlFn() {

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
}
//------------------------------------------------------------------------------------------------------------------
    
//Main
int main() {
    thread1.start(callback(write2serial_thread));
    thread2.start(callback(decodeCommand_thread));
    thread3.start(callback(playTune_thread));
    thread4.start(callback(bitcoin_thread));
    write2serialT.start(callback(write2serial_M));
    motorCtrlT.start(callback(motorCtrlFn));
    
    int32_t PWM_PRD = (int32_t)(1000000 / 440);
    pc.printf("Initial PWM Period is: %d\n\r", PWM_PRD);
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.pulsewidth_us(PWM_PRD);
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    
    I1.rise(&UpdateMotorPos); 
    I1.fall(&UpdateMotorPos); 

    I2.rise(&UpdateMotorPos); 
    I2.fall(&UpdateMotorPos); 

    I3.rise(&UpdateMotorPos); 
    I3.fall(&UpdateMotorPos); 
    
    //Run once
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6);
    
    while (1) {

        float controllerOutput;

        //read motor power
        motorPower_mutex.lock();
        controllerOutput = motorPower;
        motorPower_mutex.unlock();

        //setting limit for pulse width of PWM
        if (abs(controllerOutput) > PWM_PRD) {
            MotorPWM.pulsewidth_us(PWM_PRD);
        }
        else {
            MotorPWM.pulsewidth_us(abs(controllerOutput));
        }

        lead = 2 * ((controllerOutput > 0) - (controllerOutput < 0)); //lead set to 2*sgn(y_s)

    }
}