#include "messages.h"

Mail<mail_t, 16> mail_box;
Mail<uint8_t, 8> inCharQ;
RawSerial pc(SERIAL_TX, SERIAL_RX);


volatile uint64_t newKey;

//#######################################################################
//------------------------ Output Functions------------------------------
//#######################################################################

void outputToTerminal (void) {
    while(true)
    {
        //check each mailbox
        osEvent evt = mail_box.get();
        if (evt.status == osEventMail) 
        {
            mail_t *mail = (mail_t*)evt.value.p;

            switch(mail->type)
            {
                case(PRINT_MESSAGE):
                    pc.printf("\n\r %s\r\n ", mail->message.c_str());
                    break;
                case(BITCOIN_NONCE):
                    pc.printf("\n\r Successful Nonce: %u\r\n ", mail->number);
                    break;
                case(HASH_RATE):
                    pc.printf("\n\r Hash Rate is: %f\n ", mail->number);
                    break;
                case(UPDATED_KEY):
                    pc.printf("\n\r New Key: %u\r\n ", mail->number);
                    break;
                case(DISTANCE_STATUS):
                    pc.printf("\n\r Target Rotation: %f\n ", mail->dFloat);
                    pc.printf("\r Current Rotation: %f\n ", mail->dFloat1);
                    break;
                case(SPEED_STATUS):
                    pc.printf("\r Target Speed: %f\n ", mail->dFloat);
                    pc.printf("\r Current Speed: %f\n ", mail->dFloat1);
                    break;
            }

            mail_box.free(mail);
        }


    }
}

void putMessage(int type, uint8_t number)
{
   mail_t *mail = mail_box.alloc();
   mail->type = type;
   mail->number = number;
   mail_box.put(mail);
}

void putMessage(int type, double number)
{
   mail_t *mail = mail_box.alloc();
   mail->type = type;
   mail->dFloat = number;
   mail_box.put(mail);
}

void putMessage(int type, std::string message)
{
    mail_t *mail = mail_box.alloc();
    mail->type = type;
    mail->message = message;
    mail_box.put(mail);
}

void putMessage(int type, double posError, double velError)
{
    mail_t *mail = mail_box.alloc();
    mail->type = type;
    mail->dFloat = posError;
    mail->dFloat1 = velError;
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

void decodeSpeedCommand(std::string input)
{
    //Decode maximum speed input
    if (std::regex_match(input, std::regex("V(\\d){1,3}(\\.\\d+)?\u000D"))) { 
        float speed;
        char firstChar;
        sscanf(input.c_str(), "%c %f", &firstChar, &speed);
        maxSpeed_mutex.lock();
        maxSpeed = (speed*6);//convert input to segments per second
        maxSpeed_mutex.unlock();
        putMessage(PRINT_MESSAGE, "Got Speed Command");
    }
}

void decodePositionCommand(std::string input)
{
    //Decode target position input
    if (std::regex_match(input, std::regex("R(-)?(\\d){1,4}(\\.\\d+)?\u000D")))
    {
        float rotation; 
        char firstChar;
        sscanf(input.c_str(), "%c %f", &firstChar, &rotation);
        targetPosition_mutex.lock();
        targetPosition = position + (rotation*6);//convert input to segments
        //std::string messageOut2 = "New Target Position: " + std::to_string(targetPosition);
        targetPosition_mutex.unlock();

        std::string messageOut = "Got Position Command: " + std::to_string(rotation);
        putMessage(PRINT_MESSAGE, messageOut);
        //putMessage(PRINT_MESSAGE, messageOut2);
    }
}

void decodeKeyCommand(std::string input)
{
    if(input[0] == 'K') 
    {
        char firstChar;
        uint64_t key;
        sscanf(input.c_str(), "%c %llx", &firstChar, &key); //llx formatter since uint64 is unsigned long long 
        //pc.printf("New Key: %llx\n\r", key);
        newKey_mutex.lock();
        newKey = key;
        newKey_mutex.unlock();
        putMessage(PRINT_MESSAGE, "Got Key Command");
    }
}


void decodeTuneCommand(std::string input)
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
        
        for(int i = 1; i < input.length(); i++) 
        {
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


void decodeInputThread () {

    pc.attach(&serialISR);
    std::string command;
    
    while (true) 
    {
        //Store the new character
        osEvent newEvent = inCharQ.get();
        uint8_t* newChar = (uint8_t*)newEvent.value.p;
        command += (char)*newChar;
        inCharQ.free(newChar);
        
        //Decode the input if it is complete
        if (command.back() == '\r') 
        {    
            decodeSpeedCommand(command);
            decodePositionCommand(command);
            decodeKeyCommand(command);
            decodeTuneCommand(command);
            putMessage(PRINT_MESSAGE, "Got Message: " + command);
            command = ""; //Reset input
        } 
    }
}