#include "messages.h"

Mail<mail_t, 16> mail_box;
Mail<uint8_t, 8> inCharQ;
RawSerial pc(SERIAL_TX, SERIAL_RX);

//#######################################################################
//------------------------ Output Functions------------------------------
//#######################################################################

void outputToTerminal (void) {
    while(true)
    {
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) {
            mail_t *mail = (mail_t*)evt.value.p;
            pc.printf("\nNounce Found: %f \n"   , mail->printValue);
            
            mail_box.free(mail);
        }
    }
}

void putMessage(double counter)
{
   mail_t *mail = mail_box.alloc();
   mail->printValue = counter;
   mail_box.put(mail);
}


//#######################################################################
//------------------------- Input Functions------------------------------
//#######################################################################

// receive input data from terminal
void serialISR()
{
   uint8_t* newChar = inCharQ.alloc();
   *newChar = pc.getc();
   inCharQ.put(newChar);
}

bool decodeSpeedCommand(std::string input)
{
    //Decode maximum speed command
    if (std::regex_match(input, std::regex("V(\\d){1,3}(\\.\\d+)?\u000D"))) 
    { 
        float speed;
        char firstChar;
        sscanf(input.c_str(), "%c %f", &firstChar, &speed);
        maxSpeed_mutex.lock();
        maxSpeed = (speed*6);//convert input to segments per second
        maxSpeed_mutex.unlock();
        //pc.printf("Max Speed: %f\n\r", max_speed);
        return true;
    }
    return false;

}

bool decodeBitCoinKey(std::string input)
{
    if(command[0] == 'K') 
    {
        char firstChar;
        uint64_t key;
        sscanf(input.c_str(), "%c %llx", &firstChar, &key); //llx formatter since uint64 is unsigned long long 
        pc.printf("New Key: %llx\n\r", key);
        newKey_mutex.lock();
        newKey = key;
        newKey_mutex.unlock();
    }
    return false;
}

bool decodeTargetPositionCommand(std::string input)
{
    //Decode target position command
    if (std::regex_match(input, std::regex("R(-)?(\\d){1,4}(\\.\\d+)?\u000D")))
    {
        float rotation; 
        char firstChar;
        sscanf(input.c_str(), "%c %f", &firstChar, &rotation);
        targetPosition_mutex.lock();
        targetPosition = position + (rotation*6);//convert input to segments
        targetPosition_mutex.unlock();
    }
}

bool decodeMelodyCommand(std::string input)
{
    if(input[0] == 'T') 
    {
        char firstChar;
        std::string tone; 
        
        tonesQ_mutex.lock(); 
        
        //Empty queue
        while (!tonesQ.empty()) {
            tonesQ.pop();    
        }
        
        for(int i = 1; i < input.length(); i++) {
            if(input[i] == 'A' || input[i] == 'B' || input[i] == 'C' || input[i] == 'D' || 
                input[i] == 'E' || input[i] == 'F' || input[i] == 'G') {
                tone = "";
                tone += input[i];
            }   
            if(input[i] == '#' || input[i] == '^') {
                tone += input[i];                    
            }
            if(input[i] == '1' || input[i] == '2' || input[i] == '3' || input[i] == '4' ||
                input[i] == '4' || input[i] == '5' || input[i] == '6' || input[i] == '7' ||
                input[i] == '8') {
                tone += input[i]; 
                pc.printf("Tone: %s\n\r", tone.c_str()); 
                
                tonesQ.push(tone);
            
            }
        }
        tonesQ_mutex.unlock();
    }
}

void listenToTerminal()
{
    pc.attach(&serialISR);
    std::string command;
    
    while (true) 
    {
        //Store the new character
        osEvent newEvent = inCharQ.get();
        uint8_t* newChar = (uint8_t*)newEvent.value.p;
        command += (char)*newChar;
        inCharQ.free(newChar);


        //Decode the command if it is complete
        if (command.back() == '\r') 
        {
          //decode command  
            if(!decodeSpeedCommand(command))
                if(!decodeTargetPositionCommand(command))
                    if(!decodeBitCoinKey(command))  
                        decodeMelodyCommand(command);
            
            command = ""; //Reset command
        } 
    } 
}