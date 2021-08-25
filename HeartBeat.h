// open source interface for hearbeat

#ifndef HeartBeat_h
#define HeartBeat_h

#include "Arduino.h"

class HeartBeat
{
public:
HeartBeat();
void setOverSample(float value); 
float getOverSample();  
int getNumSamples();
int getNumOverSample();  
void setSamples(int index, float value); 
void setMean(); 
void setHeartValue(int index);
void setCountUntilMinute(int value);  
int getCountUntilMinute(); 
void setBeatsCounter(int value);
int getBeatsCounter();
void beatsIncrementer(); 
void reset();


private:
float overSample, mean; 
int num_overSample = 10; //How many light readings per sample 
int heartBeats = 0; 
float threshold = 15;
bool aboveThreshold = false;
bool belowThreshold = false; 
int num_samples = 20; //How many samples we take to calculate 'average' 
float samples[20] = { }; 
float heartValue; 
int beatsCounter = 0; 
int countUntilMinute = 0; 
};
#endif
