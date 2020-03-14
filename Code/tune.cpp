#include "tune.h"


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