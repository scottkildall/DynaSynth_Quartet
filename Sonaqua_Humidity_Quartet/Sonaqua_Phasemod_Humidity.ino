/*  
 *   Sonaqua_Phasemod_Humidity: Sketch for Dinacon
 *   
 *   by Scott Kildall
 *   www.kildall.com
 *   
 *   Uses the Mozzi Libraries
 *   
 *   Based on the Phasemod_Envelope example
 * -----------------------------------------  
 *   Sonaqua is copyright
 *   Scott Kildall, 2018, CC by-nc-sa.  
 * -----------------------------------------
 *   Mozzy is copyright
 *   Tim Barrass 2012, CC by-nc-sa.
 * -----------------------------------------
*/

//-- comment out to run with sensor
//#define SIMULATE_ON

#ifdef SIMULATE_ON
  #include "MSTimer.h"
#endif

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/cos8192_int8.h>
#include <tables/envelop2048_uint8.h>

#define CONTROL_RATE 512 // quite fast, keeps modulation smooth

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <COS8192_NUM_CELLS, AUDIO_RATE> aCarrier(COS8192_DATA);
Oscil <COS8192_NUM_CELLS, AUDIO_RATE> aModulator(COS8192_DATA);
Oscil <COS8192_NUM_CELLS, AUDIO_RATE> aModWidth(COS8192_DATA);
Oscil <COS8192_NUM_CELLS, CONTROL_RATE> kModFreq1(COS8192_DATA);
Oscil <COS8192_NUM_CELLS, CONTROL_RATE> kModFreq2(COS8192_DATA);
Oscil <ENVELOP2048_NUM_CELLS, AUDIO_RATE> aEnvelop(ENVELOP2048_DATA);

//-- simulated EC level
int simEC = 300;

//-- SONAQUA STUFF
#define humiditySensorPin (A1) 

//-- power led
#define greenLEDPin (12)
#define blueLEDPin (8)


#define switchPin (8)

// comment out for no serial debug (much faster)
#define SERIAL_DEBUG

int minEC = 0;
int maxEC = 1000; 
unsigned int curEC;

int humidityValue;
boolean bSwitchOn = false;
boolean bTurnOff = false;
boolean bTurnOn = false;

//--
float aModulatorBaseFreq = 135.0f;

#ifdef SIMULATE_ON
  //-- use this to simulate a switch on from a sequencer
  MSTimer switchTimer;
#endif


void setup() {
  randomSeed(A0);
  
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);
    Serial.println("Starting up: Sonaqua_Phasemod_Humidity");
  #endif

#ifdef SIMULATE_ON
  #ifdef SERIAL_DEBUG
    Serial.println("In Simulation mode");
    bSwitchOn = true;
    switchTimer.setTimer(5000);
  #endif
#endif

  // switchPin
   pinMode(switchPin, INPUT);
  
 // Flash LEDs
   pinMode(greenLEDPin,OUTPUT);      

#ifdef SIMULATE_ON
   pinMode(blueLEDPin,OUTPUT);               
#endif

  for(int i = 0; i < 6; i++ ) {
    digitalWrite(greenLEDPin,HIGH);

#ifdef SIMULATE_ON
    digitalWrite(blueLEDPin,HIGH);
#endif
    
    delay(100);
    digitalWrite(greenLEDPin,LOW);

#ifdef SIMULATE_ON
    digitalWrite(blueLEDPin,LOW);
#endif    
    delay(100);
  }
  
  digitalWrite(greenLEDPin,HIGH);

#ifdef SIMULATE_ON
  digitalWrite(blueLEDPin,bSwitchOn);
#endif
  //-- get initial value, we key off this in simulation mode

#ifdef SIMULATE_ON
  humidityValue = 500;
#else
  humidityValue = analogRead(humiditySensorPin);
#endif

#ifdef SERIAL_DEBUG
  Serial.print("initial humidity value = ");
  Serial.println(humidityValue);
#endif

  startMozzi(CONTROL_RATE);

  //-- initial values 
  aCarrier.setFreq(humidityValue);
  
  // kModFreq1 will cange a reverb of sorts
  kModFreq1.setFreq(.85f);

  // kModFreq2 will change the bass modulation — higher values seem to make it go slower
  kModFreq2.setFreq(0.1757f);

  //-- tis one is hard to figure out what it means exactly
  aModWidth.setFreq(0.1434f);

  //-- the lower the envelope, the longer the reverb/echo effcts
  aEnvelop.setFreq(8.0f);

  bSwitchOn = digitalRead(switchPin);
}


void updateControl() {
  
#ifdef SIMULATE_ON
  if( switchTimer.isExpired() ) {
      bSwitchOn = !bSwitchOn;
      Serial.print("switching to: " );
      Serial.println( bSwitchOn );
        
      if( bSwitchOn ) {
        bTurnOn = true;
        bTurnOff = false;
      }
      else {
        bTurnOn = false;
        bTurnOff = true;
      }
      
      digitalWrite(blueLEDPin,bSwitchOn);
      switchTimer.start();
    }
  
  // random number between 0-4
  humidityValue += (random(0,5)) - 2;
#else
//Serial.println(digitalRead(switchPin));
 boolean switched = digitalRead(switchPin);
 if( switched != bSwitchOn ) {
  //Serial.print("switching = ");

  bSwitchOn = !bSwitchOn;
  //Serial.println(bSwitchOn);
  
    if( bSwitchOn ) {
        bTurnOn = true;
        bTurnOff = false;
      }
      else {
        bTurnOn = false;
        bTurnOff = true;
      }
  }
  humidityValue = analogRead(humiditySensorPin);
//  Serial.print("h = ");
//  Serial.println(humidityValue);
  
#endif
//-- doing this below seems to screw up the timing
/*
#ifdef SERIAL_DEBUG
  Serial.print("humidity = ");
  Serial.println(humidityValue);
#endif
*/

  aModulator.setFreq(aModulatorBaseFreq + 0.4313f*kModFreq1.next() + kModFreq2.next());

   kModFreq1.setFreq(.85f + (float)(humidityValue+random(50))/300.f);
}


int updateAudio(){
  //-- always play through until we reach zero, then we turn off the beat
  if( bSwitchOn == false && bTurnOff == false ) {
    return 0;
  }
    
   aModulatorBaseFreq = (float)humidityValue/4.0f;

   //-- a short delay does a really crazy schmear of the sounds — the higher the delay, the more it sounds
   //-- like a unified sound as opposed to a series of distinct beats
   //-- the randomness adds a tinny sound at the end
   //-- note: this doesn't seem to affect the whiny sound that I don't like
    delayMicroseconds(humidityValue + 200 + random(400));

   aCarrier.setFreq((float)humidityValue/6.0f);

  int asig = aCarrier.phMod((int)aModulator.next()*(260u+aModWidth.next()));
  int audioValue = (asig*(byte)aEnvelop.next())>>8;

  //-- this will show us raw outputs
  //Serial.println( audioValue );
  
  if( bTurnOff == true && audioValue == 0 ) {
    bTurnOff = false;
    Serial.println("turning off");
  }
  else if( bTurnOn ) {
    if( audioValue == 0 ) { 
      bTurnOn = false;
     Serial.println("turning on");
    }
    else
      return 0;  
  }
  
  return audioValue;
}


void loop() {
  audioHook();
}


