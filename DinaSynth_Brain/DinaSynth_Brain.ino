/*  
 *   DinaSnyth_Brain
 *   
 *   by Scott Kildall
 *   www.kildall.com
 *   
 *   Sequencer for passive synths
 *   
 *   Designed for Arudino rather than Sonaqua boards
 *   
 *   
 */


#include "MSTimer.h"


#define NUM_SYNTHS (1)

int startSynthPin = 8;  

MSTimer synthTimer[NUM_SYNTHS];
boolean synthOn[NUM_SYNTHS];

int startTimerRange = 2000;
int endTimerRange = 10000;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up: DinaSnyth_Brain");

  Serial.print("Num synths = ");
  Serial.println(NUM_SYNTHS);

  for( int i = 0; i < NUM_SYNTHS; i++ ) {
    synthTimer[i].setTimer(random(startTimerRange,endTimerRange));
    synthOn[i] = true;

    pinMode(startSynthPin+i,OUTPUT);
    digitalWrite(startSynthPin+i,synthOn[i]);
  }
}

void loop() {
  for( int i = 0; i < NUM_SYNTHS; i++ ) {
    if( synthTimer[i].isExpired() ) {
        synthOn[i] = !synthOn[i];
        digitalWrite(startSynthPin+i, synthOn[i] );
        synthTimer[i].setTimer(random(startTimerRange,endTimerRange));

        Serial.print( "Switching synth: " );
        Serial.print( i+1 );
        Serial.print( " = " );
        Serial.println( synthOn[i] );
    }
  }
}
