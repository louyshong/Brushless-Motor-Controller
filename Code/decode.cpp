#include "decode.h"

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