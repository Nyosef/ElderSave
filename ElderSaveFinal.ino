/* ElderSave 
 *  
 * ElderSave is an all round walking/training device for the elderly, with the main purpose of allowing the elderly
 * to workout safely, and allow for immediate help in the case of need. The app tracks the body and alerts the surrounding
 * enviornment of the workout.
 *  
 * The circuit:
 * Input: Right Button, Left Button, Slide Switch, Accelerometer, Light Sensor
 * Output: NeoPixels in different colors, Tones
 * 
 * Video link: https://youtu.be/YYDFkgYiiH0
 * 
 * 
 * Blynk:
 * Input: GPS, Webhook, Slider, Styled Buttons
 * Output: Dispay
 * Virtual pins mapping:
 * - V0 = Emergency contact
 * - V1 = totalWalkingDistance
 * - V2 = setWalkingDistance
 * - V3 = startActivity
 * - V5 = GPS
 * - V6 = smsArrived
 * - V7 = restartActivity
 * 
 * Open source enviornments and code:
 *  Circuit Walker Sneakers: https://github.com/adafruit/Circuit_Walker_Sneakers/tree/master/Arduino_Adaptive_Step_Jerk_Test - allowing the pediometer to work
          * Based on IIRFilter.h  
 * Calculate distance between two gps coordinates  https://nathanrooy.github.io/posts/2016-09-07/haversine-with-python/
 * 
 *Calculate based Heartbeat: https://github.com/paulvangentcom/heartrate_analysis_Arduino
 
 * 
 * Created by:
 * Nir Yosef 305214371
 *  
 */



#define BLYNK_PRINT SerialUSB
//#define BLYNK_DEBUG SerialUSB

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <Adafruit_CircuitPlayground.h>
#include <math.h>
//#include <HeartBeat.h>
#include "HeartBeat.h"
#include "IIRFilter.h"

#define ROTATION_RATE   30 // for magic wheel
#define  INITIAL_JERK_AVG    0.10    // Initial average to be considered a step.
#define  ALPHA               0.125   // Weight for a weighted average of the jerk value.
#define  BETA                0.25    // Weight for a weighted standard deviation of the jerk value.
#define  GRAVITY             1.0     // Baseline accelerometer magnitude.
#define  SAMPLE_RATE_HZ      50.0    // How quickly to sample the accelerometer (50HZ).

// Coefficients for IIR Butterworth low pass filter with cutoff of 3.6667 hz.
#define FILTER_A       { 1.0, -2.08554010137, 1.54484341043, -0.394448994245 }
#define FILTER_B       { 0.00810678935123, 0.0243203680537, 0.0243203680537, 0.00810678935123 }

#define d2r (M_PI / 180.0)


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char auth[] = "SPunUFQu-if-G505M2k6jfjsIs42AJ6H";

char ssid[] = "Yosef";
char pass[] = "yosef2019";

#define EspSerial Serial1
#define ESP8266_BAUD 9600

ESP8266 wifi(&EspSerial);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t colors[] = {  // For rainbow wheel animation
  0x0245F9, 0x305AC9,0x7C97DE,0x435278,0x91A4D8,0xFF00FF,0x305AC9,0x00000,0x00000,0x00000
};

uint32_t emergencyColors[] = {  // For rainbow wheel animation
  0xFF0000, 0xFFFF00,0xFF0000,0xFFFF00,0xFF0000,0xFFFF00,0xFF0000,0xFFFF00,0xFF0000,0xFFFF00
};

const uint8_t spDANGER[]        PROGMEM = {0x2D,0xBF,0x21,0x92,0x59,0xB4,0x9F,0xA2,0x87,0x10,0x8E,0xDC,0x72,0xAB,0x5B,0x9D,0x62,0xA6,0x42,0x9E,0x9C,0xB8,0xB3,0x95,0x0D,0xAF,0x14,0x15,0xA5,0x47,0xDE,0x1D,0x7A,0x78,0x3A,0x49,0x65,0x55,0xD0,0x5E,0xAE,0x3A,0xB5,0x53,0x93,0x88,0x65,0xE2,0x00,0xEC,0x9A,0xEA,0x80,0x65,0x82,0xC7,0xD8,0x63,0x0A,0x9A,0x65,0x5D,0x53,0xC9,0x49,0x5C,0xE1,0x7D,0x2F,0x73,0x2F,0x47,0x59,0xC2,0xDE,0x9A,0x27,0x5F,0xF1,0x8B,0xDF,0xFF,0x03};


// Magic wheel vars
bool activateWheel = false;
int lock = 0;
int colorLocation;
int startIndex;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String emergencyContact;


// DEFINE MIN AND MAX HEARTBEAT HERE
// Vars indicating at what heartbeat level to enter emergency mode
bool emergencyMode = 0;
int minEmergencyHeartbeat = 50;
int maxEmeregencyHeaertbeat = 180;

double setWalkingDistance = 1.0;     // 
double remainingWalkingDistance = 0.0;      // How many kms left to run
double totalWalkingDistance = 0.0;     // How many kms already ran

double startTime = 0.0;         // Time when run started
double walkingTime = 0.0;       // How much time already ran
double calories = 0.0;  
int lastHeartBeat = 0;

bool startActivity = 0;              // Start run flag
bool restartActivity = 0;              // Reset run flag


// default origin coordinates
double latitude = 0;
double longitude = 0;

int gpsCount = 0;
long loopTimer = 0;

HeartBeat heartBeat;
int lightSensorValue;


// Opensource variables for the pedomitor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint32_t periodMS = int(1.0/SAMPLE_RATE_HZ*1000.0);     // Period of time (in milliseconds) between accelerometer samples.
IIRFilter<4,4> filter((float[])FILTER_A, (float[])FILTER_B);  // The IIR filter that will use the coefficients defined above.
uint32_t lastMS = 0;             // Time of last loop iteration.
uint32_t lastSampleMS = 0;       // Time of last accelerometer sample.
float lastTrough = GRAVITY;
float lastPeak = GRAVITY;
enum JerkState { ZERO, PEAK, TROUGH };
JerkState currentState = ZERO;
float jerkMean = INITIAL_JERK_AVG;
float jerkDev = jerkMean / 2.0;
int totalSteps = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool emergencyMailSent =0;
bool ReCnctFlag;
int ReCnctCount =0;

#define CLICKTHRESHHOLD 120

void setup() {
  Serial.begin(115200);
  CircuitPlayground.begin();
  SerialUSB.begin(9600);
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  Blynk.begin(auth, wifi, ssid, pass);
  
  Blynk.virtualWrite(V1, totalWalkingDistance);
  Blynk.virtualWrite(V2, setWalkingDistance);
  Blynk.virtualWrite(V3, startActivity);

  lastMS = millis(); // Time app started
}

void loop() {  

// Handling connection and reconnection settings in case of losing connection to Blynk

if (Blynk.connected()) {

    Blynk.run();
} else if (ReCnctFlag==0){

      ReCnctFlag = 1;  // Set reconnection Flag
    Serial.println("Starting reconnection timer in 30 seconds...");
      ReCnctFlag = 0;  // Reset reconnection Flag
      ReCnctCount++;  // Increment reconnection Counter
      Serial.print("Attempting reconnection #");
      Serial.println(ReCnctCount);
      wifi.setDHCP(1, 1, 1); //Enable dhcp in station mode and save in flash of esp8266
      Blynk.connect();  // Try to reconnect to the server
      if (Blynk.connectWiFi(ssid, pass)) {
        Blynk.connect();
      }
    }  // END Timer Function

  

 
  
  // Run every 1 second
  if((millis() - loopTimer) >  1000) {



    // Check elder heartbeat - if unnatural sends distress signal 
    // Sending out distress signals
    if(CircuitPlayground.leftButton() && startActivity == 0) {
      Blynk.disconnect();
      Serial.println("------------------Measuring heartbeat-------------------");
      CircuitPlayground.clearPixels();
      CircuitPlayground.setPixelColor(1, 0, 255, 0);
      lastHeartBeat = measureHeartbeat();

    if ((lastHeartBeat < minEmergencyHeartbeat) || (lastHeartBeat > maxEmeregencyHeaertbeat)){
      Serial.println("!!!!!!!!!!!!!!!!--------EMERGENCY MODE ACTIVATED---------!!!!!!!!!!!!!!!!");
      emergencyMode = 1;
      Serial.println("Emergency mode is now ");
      Serial.println(emergencyMode);

    for (int i =0; i<5 ; i++){
      CircuitPlayground.speaker.say(spDANGER);
    }
      

    }
      
      Serial.print("lastHeartBeat = "); Serial.println(lastHeartBeat);
      Blynk.connect();
      Serial.println("------------------Back Online-------------------");
      Blynk.run();

       if ((emergencyMode == 1) & (emergencyMailSent == 0))
       {                 

          Serial.println("sending emergency email to" + emergencyContact );
          Blynk.notify("Call Help!"); 
          Blynk.email("nyosef@gmail.com","Help Nir", "come to coordinates");
          emergencyMailSent = 1;
       }
    }
   
    // Check run end 
    remainingWalkingDistance = setWalkingDistance - totalWalkingDistance;
    
    if(remainingWalkingDistance <= 0 || restartActivity == 1) {
      endOfRun();
    }


    if(emergencyMode == 1){
              activateWheel = true;
              Wheel(activateWheel,emergencyColors);
     
    }

    

// Notifying on ativity start
    if(CircuitPlayground.slideSwitch() && startActivity == 1 && emergencyMode == 0){ 
    Serial.println("!!!!!!!!!!!Walk started!!!!!!!!!!!!");
      activateWheel = true;
      Wheel(activateWheel,colors);
          
    }
    else{

     if(emergencyMode == 0){ 
    Serial.println("Walk has not started yet");
    activateWheel = false;
     }
    }

    loopTimer = millis();  
  }




  // Count steps while running
  if (startActivity == 1) pedometer();


} // loop closing 



BLYNK_WRITE(V0) {
  emergencyContact = param.asStr();
  Serial.println(emergencyContact);
}



// Get set running distance from Blynk
BLYNK_WRITE(V2) {
  setWalkingDistance = param.asDouble();
}

// Get a start run command from Blynk and calculate times
BLYNK_WRITE(V3) {
  startActivity = param.asInt();

  // Start measuring run time
  if(startActivity == 1) startTime = millis(); // Start measuring run time

  // Measure time until run paused
  if(totalWalkingDistance > 0 && startActivity == 0) walkingTime += (millis() - startTime) / 1000; // time in seconds
}

// Get current GPS location and calculate running distance
BLYNK_WRITE(V5) {
  GpsParam gps(param);
  Serial.print("Got GPS update number "); Serial.println(gpsCount);
  Serial.print("lat: "); Serial.println(gps.getLat());
  Serial.print("lon: "); Serial.println(gps.getLon());

  
  // Calculate running distance while running
  if (gpsCount > 0 && startActivity == 1) {
    double segment = haversine(latitude, longitude, gps.getLat(), gps.getLon());
    totalWalkingDistance += segment;
    Serial.println("This should be showing");
    
      Serial.println("how much did you walk?");
      Serial.println(totalWalkingDistance);
  }

  latitude = gps.getLat();
  longitude = gps.getLon();
  Blynk.virtualWrite(V1, totalWalkingDistance);    // Update total running distance on Blynk

  gpsCount++;
}


// Get a reset run command from Blynk
BLYNK_WRITE(V7) {
  restartActivity = param.asInt();
}

// Send run data and reset them
void endOfRun() {
  walkingTime += (millis() - startTime) / 1000; // time in seconds
  Serial.println(" Time of Walk");
  Serial.println(walkingTime);
  
  calories = totalSteps * 0.04;
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V3, 0);
  Blynk.virtualWrite(V7, 0);
  totalSteps = 0;
  remainingWalkingDistance = 0.0;
  totalWalkingDistance = 0.0;
  startActivity = 0;
  restartActivity = 0;
  walkingTime = 0.0;
  calories = 0.0;
}



// Calculate haversine distance for linear distance in km
double haversine(double lat1, double long1, double lat2, double long2)
{
    double dlong = (long2 - long1) * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double a = pow(sin(dlat/2.0), 2) + cos(lat1*d2r) * cos(lat2*d2r) * pow(sin(dlong/2.0), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double d = 6367 * c;
    return d;
}


// Pedometer
void pedometer() {
  uint32_t currentMS = millis();
  uint32_t deltaBetweenLoopsMS = currentMS - lastMS;
  
  // Check if enough time has elapsed to take a new accelerometer reading.
  if (currentMS >= (lastSampleMS + periodMS)) {
    
    // Grab a new accelerometer reading and compute the magnitude of the acceleration vector
    CircuitPlayground.lis.read();
    float mag = sqrt(pow(CircuitPlayground.lis.x_g, 2.0) +
                     pow(CircuitPlayground.lis.y_g, 2.0) +
                     pow(CircuitPlayground.lis.z_g, 2.0));
    float filtered = filter.filter(mag); // Apply the low pass filter to the magnitude reading.
    
    // Compute the strength of a movement from up to down acceleration
    JerkState newState = ZERO;
    if (filtered < GRAVITY) newState = TROUGH;
    else if (filtered > GRAVITY) newState = PEAK;
    if (newState == currentState) {
      
      // Keep searching for the min trough value and maximum peak value.
      if (newState == TROUGH) lastTrough = min(lastTrough, filtered);
      else if (newState == PEAK) lastPeak = max(lastPeak, filtered);
    }
    else { // Changed state from peak to trough or vice/versa.  Compute the 'jerk' or distance
      if (newState == PEAK) {
        float jerk = lastPeak - lastTrough;
        
        // Check if the jerk value is within 4 standard deviations of the current jerk average.
        if (jerk > (jerkMean - 4.0*jerkDev)) {
          jerkDev = abs(jerkMean-jerk)*BETA + jerkDev*(1.0-BETA); // Update the jerk standard deviation
          jerkMean = jerk*ALPHA + jerkMean*(1.0-ALPHA); // Update the jerk average
          totalSteps++;
        }
        lastPeak = filtered;
        lastTrough = GRAVITY;
      }
    }
    currentState = newState; // Update for the next loop iteration detects changes.
    lastSampleMS = currentMS; // Update time of last sample.
  }
  lastMS = currentMS; // Update time of last frame. 
}


// Measures hearbeat and returns its value
int measureHeartbeat() { 
  delay(5000);
  bool continueLoop = true; 
  while(continueLoop == true){
    lightSensorValue = CircuitPlayground.lightSensor();
    for(int i = 0; i < heartBeat.getNumSamples(); i++) {
      heartBeat.setOverSample(heartBeat.getNumOverSample() * lightSensorValue); 
      float sampleValue = heartBeat.getOverSample() / heartBeat.getNumOverSample(); //find the average 
      heartBeat.setSamples(i, sampleValue);
      heartBeat.setHeartValue(i); 
      heartBeat.beatsIncrementer();
      delay(25); 
      heartBeat.setCountUntilMinute(heartBeat.getCountUntilMinute() + 25);
      if(heartBeat.getCountUntilMinute() >= 15000) continueLoop = false;
    }
  }
  return heartBeat.getBeatsCounter() * 4;
}


// Indicating lights for all surrounding participants that the elder person is walking safely and everythign is okay. 
// Helps with recognizing the elder person while walking during night

void Wheel(bool activateWheel, uint32_t *colors){  // Rainbow wheel animation
          if (activateWheel == true){
                    lock = 1;
                    CircuitPlayground.clearPixels();

                    
                  //setting the pixels
                  colorLocation = startIndex;
  
                  
                      for (int pixel=0; pixel<10; pixel++) {
                        CircuitPlayground.setPixelColor(pixel, colors[colorLocation]);
                        colorLocation++;
                        if (colorLocation > 9) colorLocation = 0;
                      }
                    
                  // Increment color location
                  startIndex++;
                      
                        // Reset to starting position
                        if (startIndex > 9)
                            { startIndex = 0;}
                      
                  delay(ROTATION_RATE);
          }
          
          else if(activateWheel == false){  // Smart Condition in order make it works :)
                  lock = lock - 1;
                  if (lock == 0){
                    CircuitPlayground.clearPixels();
                  }
          }
}
