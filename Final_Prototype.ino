/**
 * Example of OTAA device      
 * Authors: 
 *        Ivan Moreno
 *        Eduardo Contreras
 *  June 2019
 * 
 * This code is beerware; if you see me (or any other collaborator 
 * member) at the local, and you've found our code helpful, 
 * please buy us a round!
 * Distributed as-is; no warranty is given.
 */
#define DEBUG 0
#include "DHT.h"

DHT dht;

unsigned long previousMillisWhileInputs = 0;
int latchPin = 8;
int dataPin = 9;
int clockPin = 7;
int led = 10;
const int payload_size = 6;

byte payload[payload_size] = {0, 0, 0, 0, 0, 0};

void setup() {
Serial.begin(9600); 
while(!Serial);

  
  //More Inputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(dataPin, INPUT);
#if DEBUG
  Serial.println(String("More Inputs"));
#endif
  //More Inputs End

  //DHT11
#if TEMPSENSOR && DEBUG
  Serial.println("DHT11");
#endif
dht.setup(A0);
  //DHT11 End
}

void loop() {
  delay(1000);
  digitalWrite(led, HIGH);
  checkInputs();
  getDht11Inputs();
  Serial.write(payload, 6);
  delay(10000);
  digitalWrite(led, LOW);
}

void checkInputs(){
  int countTime = 0;
  for(int i = 0; i < 4; i++){
       payload[i] = 0; 
    }
  while(countTime <= 5){
    digitalWrite(latchPin,1);
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(latchPin,0);
          int i;
          int temp = 0;
          int pinState;
          byte myDataIn[4] = {0,0,0,0};
          byte test = 0;
        
          pinMode(clockPin, OUTPUT);
          pinMode(dataPin, INPUT);
        
          for (i=31; i>=0; i--)
          {
            digitalWrite(clockPin, 0);
            delayMicroseconds(0.2);
            temp = digitalRead(dataPin);
            if (temp) {
              pinState = 1;
              myDataIn[i/8] = myDataIn[i/8] | (1 << i%8); 
            }
            else {
              pinState = 0;
            }
            digitalWrite(clockPin, 1);
          }
          
    for(i = 0; i < 4; i++){
       if(myDataIn[i] > payload[i]){
        payload[i] = myDataIn[i];
      }
    }
    if(millis() - previousMillisWhileInputs > 1000){
      countTime++;
      previousMillisWhileInputs = millis();
    }
  }
}

void getDht11Inputs(){
  do{
    delay(dht.getMinimumSamplingPeriod());
    payload[payload_size-2] = dht.getTemperature();
    payload[payload_size-1] = dht.getHumidity();
  }while(dht.getStatusString() != "OK");
  
}
