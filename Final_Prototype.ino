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
#define DEBUG 0 // Treba biti 1 da bi radio program -_-

unsigned long previousMillisWhileInputs = 0;
int latchPin = 8;
int dataPin = 9;
int clockPin = 7;
int led = 10;
byte switchVar = 0;

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
  //DHT11 End
}

void loop() {
  delay(1000);
  digitalWrite(led, HIGH);
  switchVar = checkInputs();
  Serial.println(switchVar);
  delay(5000);
  digitalWrite(led, LOW);
}

byte checkInputs(){
  byte switchVarTemp = 0;
  int countTime = 0;
  
  while(countTime <= 5){
    if(millis() - previousMillisWhileInputs > 1000 || shiftIn(dataPin, clockPin) == 255){
      countTime++;
      previousMillisWhileInputs = millis();
    }
    digitalWrite(latchPin,1);
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(latchPin,0);
    byte tempSwitch = shiftIn(dataPin, clockPin);
    if(tempSwitch > switchVarTemp){
      #if DEBUG
      Serial.println(switchVarTemp, BIN);
      #endif
      switchVarTemp = tempSwitch;
    }
  }
  return switchVarTemp;
}

byte shiftIn(int myDataPin, int myClockPin) {

  int i;
  int temp = 0;
  int pinState;
  byte myDataIn = 0;

  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, INPUT);

  for (i=7; i>=0; i--)
  {
    digitalWrite(myClockPin, 0);
    delayMicroseconds(0.2);
    temp = digitalRead(myDataPin);
    if (temp) {
      pinState = 1;
      myDataIn = myDataIn | (1 << i);
    }
    else {
      pinState = 0;
    }
    digitalWrite(myClockPin, 1);
  }
  return myDataIn;
}
