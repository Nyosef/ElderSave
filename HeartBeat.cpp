// Open source library for hearbeat calcualtions

#include "Arduino.h"
#include "HeartBeat.h"
HeartBeat::HeartBeat() {
}
void HeartBeat::setOverSample(float value){
	//set oversample value 
	overSample = value;
} 

float HeartBeat::getOverSample() {
	//Get oversample value
	return overSample;
}   

int HeartBeat::getNumSamples() {
	//get num_samples value
	return num_samples;
}  

int HeartBeat::getNumOverSample() {
	//get num_overSample value
	return num_overSample;
}  

void HeartBeat::setSamples(int index, float value){
	//Update samples array
	samples[index] = value;
} 

void HeartBeat::setMean(){
	//set mean value 
	int sum = 0; 
	for(int i = 0; i < num_samples; i++){
		sum += samples[i];  
	}  
	mean = sum / num_samples; //take the average 
}  

void HeartBeat::setHeartValue(int index){ 
	//set heart value
	setMean(); 
	heartValue = samples[index] - mean; 
} 

void HeartBeat::setCountUntilMinute(int value){
	//set countUntilMinute value
	countUntilMinute = value;
} 

int HeartBeat::getCountUntilMinute(){
	//get countUntilMinute value
	return countUntilMinute;
} 

void HeartBeat::setBeatsCounter(int value){
	//set beatsCounter value
	beatsCounter = value;
} 

int HeartBeat::getBeatsCounter(){
	//get beatsCounter value
	return beatsCounter;
}


void HeartBeat::beatsIncrementer(){
	//check thresholds to identify pulse amplitude
	//and increment beatsCounter
	if(heartValue > threshold)aboveThreshold = true;
	if(heartValue < threshold && aboveThreshold == true){
	  beatsCounter++;
	  aboveThreshold = false;
	} 
	if(heartValue < -2)belowThreshold = true; 
	if(heartValue > 0 && belowThreshold == true){
	  beatsCounter++;
	  belowThreshold = false;
	} 
} 

void HeartBeat::reset(){
	//reset parameters for beats per minute measuring 
	beatsCounter = 0;
	countUntilMinute = 0;
}
