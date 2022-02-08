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
#define DEBUG 1 // Treba biti 1 da bi radio program -_-
#define DHT11Pin A0
#define LED 1 // upali/ugasi led indikator
#if DEBUG
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
  #define beginSerial() Serial.begin(9600)
#else
  #define debug(x)
  #define debugln(x)
  #define beginSerial()
#endif
#include <lorawan.h>
#include "DHT.h"
#include <Wire.h>

DHT dht;

// OTAA credentials
const char devEui[] PROGMEM = {"70B3D57ED0049BC1"};
const char appEui[] PROGMEM = {"0000000000000000"};
const char appKey[] PROGMEM = {"77060A80845A12DE33C285991EC63105"};

unsigned long prevMillisLora;
unsigned long prevMillisInputs;
uint8_t wakeup_count = 3; //Change on two places
char outStr[200];  
byte payload[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//master|slave|slave|slave|temperature|humidity|heater|ventilator|vlaga|batteryStatus

byte recvStatus = 0;

bool dht11External = true;
//bool externalInterrupt = false;
bool statusChanged = true;

const PROGMEM sRFM_pins RFM_pins = {
  .CS = 6,
  .RST = 5,
  .DIO0 = 2,
  .DIO1 = 3,
  .DIO2 = 4,
  .DIO5 = A2
};

const int PROGMEM latchPin = 8;
const int PROGMEM dataPin = 9;
const int PROGMEM clockPin = 7;

const int PROGMEM inputsCtrl = A1;
#if LED
const int PROGMEM ledCtrl = A3;
#endif
byte tempDHT = 0;
byte humDHT = 0;

void setup() {
  beginSerial();
  
  //Watchdog
  ADCSRA = 0;
  PRR = //(1 << PRTWI) |
        (1 << PRTIM2) |
        (1 << PRTIM1) |
        //(1 << PRSPI) |
        //(1 << PRUSART0) |
        (1 << PRADC);
  //Watchdog End

  //LED
  #if LED
  pinMode(ledCtrl, OUTPUT);
  digitalWrite(ledCtrl, 1);
  #endif
  //LED End

  //I2C 
  Wire.begin(0);
  Wire.onReceive(receiveEvent);
  //I2C End
  
  //More Inputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(inputsCtrl, OUTPUT);
#if DEBUG
  debugln(String("More Inputs"));
#endif
  //More Inputs End

  //DHT11
#if TEMPSENSOR && DEBUG
  debugln("DHT11");
#endif
dht.setup(DHT11Pin);
  //DHT11 End
  
  //Lora Init
  initLoraWithJoin();
  // Join procedure End
}

void loop() {
  /*debug("batteryStatus: ");
  debugln(payload[9]);*/
  if(payload[9]){
    if(wakeup_count >= 3)//Change on two places
    {
      checkInputs();
      getInterfaceData();
      getDht11Inputs();
      lora.wakeUp();  
      debugln("Sending: ");    
      lora.sendUplink(payload, 10, 0, 1);
      recvStatus = lora.readData(outStr);
      if(recvStatus) {
        debugln(outStr);
      }
      lora.update();
      lora.sleep();
      wakeup_count = 0;
    }
    getBatteryInfo();
    wakeup_count++;
    goToSleep();
  }else{//On Main Power
    checkInputs();
    getInterfaceData();
    getDht11Inputs();
    if(statusChanged){
      debugln("Sending: ");
      lora.sendUplink(payload, 10, 0, 1);
      statusChanged = false;
    }

    recvStatus = lora.readData(outStr);
    if(recvStatus) {
      debugln(outStr);
    }
    lora.update();
    getBatteryInfo();
  }
}

void initLoraWithJoin(){
  //Lora Init
  if(!lora.init()){
    #if DEBUG
    debugln("RFM95 not detected");
    #endif
    delay(5000);
    return;
  }
  lora.setDeviceClass(CLASS_A);
  lora.setTxPower(A2,PA_BOOST_PIN);
  lora.setDataRate(SF9BW125);
  lora.setChannel(MULTI);
  char output1[16];
  for (byte k = 0; k < 16; k++) {
    output1[k] = pgm_read_byte_near(devEui + k);
  }
  lora.setDevEUI(output1);
  char output2[16];
  for (byte k = 0; k < 16; k++) {
    output2[k] = pgm_read_byte_near(appEui + k);
  }
  lora.setAppEUI(output2);
  char output3[32];
  for (byte k = 0; k < 32; k++) {
    output3[k] = pgm_read_byte_near(appKey + k);
  }
  lora.setAppKey(output3);
  //Lora Init End

  // Join procedure
  bool isJoined;
  do {
    #if DEBUG
    debugln("Joining...");
    #endif
    isJoined = lora.join();
    delay(5000);
  }while(!isJoined);
  #if DEBUG
  debugln("Joined to network");
  #endif
  // Join procedure End
  #if LED
  digitalWrite(ledCtrl, 0);
  delay(500);
  digitalWrite(ledCtrl, 1);
  delay(500);
  digitalWrite(ledCtrl, 0);
  delay(500);
  digitalWrite(ledCtrl, 1);
  #endif
}

void getBatteryInfo(){
  bool tempStatus = payload[9];
  Wire.requestFrom(3,1);//Address
  if(Wire.available())
  {
    debug("HERE");
    payload[9] = Wire.read();
  }else{
    payload[9] = 0;
  }
  if(payload[9] != tempStatus){
    statusChanged = true;
    debugln("Battery changed");
  }
  debugln(payload[9]);
}

void getInterfaceData(){
  Wire.beginTransmission(1);
  Wire.write(payload[9]); 
  if(Wire.endTransmission() != 0){
    dht11External = false;
    payload[6] = 0;
    payload[7] = 0;
    payload[8] = 0;
  }else{
    dht11External = true;
  }
}

void getDht11Inputs(){
  if(!dht11External){
    readSensor(); 
  }
  if((payload[4] - tempDHT >= 3 || payload[4] - tempDHT <= -3 || payload[5] - humDHT >= 5 || payload[5] - humDHT <= -5) && humDHT != 0){
    statusChanged = true;
    debugln("dht11 changed");
    payload[4] = tempDHT;
    payload[5] = humDHT;  
  }
}

void checkInputs(){ 
  digitalWrite(inputsCtrl, HIGH);
  delay(1000);
  int countTime = 0;
  byte tempPayload[4] = {0,0,0,0};
  int myDataIn[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  for(int x = 0; x < 1000; x++){
    digitalWrite(latchPin,1);
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(latchPin,0);
    
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, INPUT);     
    for (int i=31; i>=0; i--)
    {
      digitalWrite(clockPin, 0);
      delayMicroseconds(0.2);
      myDataIn[i] += digitalRead(dataPin);
      digitalWrite(clockPin, 1);
    }
    delay(1);
  }
   
  for(int i=31; i>=0; i--){
    if(myDataIn[i] > 20){
     tempPayload[i/8] = tempPayload[i/8] | (1 << i%8);
    }
  }
  digitalWrite(inputsCtrl, LOW);
  for(int i = 0; i < 4; i++){
    if(payload[i] != tempPayload[i]){
      payload[i] = tempPayload[i];
      statusChanged = true;
    }
  } 
  if(statusChanged){
    statusChanged = true;
  }else{
    statusChanged = false;
  }
}

void readSensor()
{
  // Make sure we don't poll the sensor too often
  // - Max sample rate DHT11 is 1 Hz   (duty cicle 1000 ms)
  // - Max sample rate DHT22 is 0.5 Hz (duty cicle 2000 ms)
  unsigned long startTime = millis();

  // Request sample

  digitalWrite(DHT11Pin, LOW); // Send start signal
  pinMode(DHT11Pin, OUTPUT);
  delay(18);

  pinMode(DHT11Pin, INPUT);
  digitalWrite(DHT11Pin, HIGH); // Switch bus to receive data

  // We're going to read 83 edges:
  // - First a FALLING, RISING, and FALLING edge for the start bit
  // - Then 40 bits: RISING and then a FALLING edge per bit
  // To keep our code simple, we accept any HIGH or LOW reading if it's max 85 usecs long

  uint16_t rawHumidity = 0;
  uint16_t rawTemperature = 0;
  uint16_t data = 0;

  for ( int8_t i = -3 ; i < 2 * 40; i++ ) {
    byte age;
    startTime = micros();

    do {
      age = (unsigned long)(micros() - startTime);
      if ( age > 90 ) {
        //error = ERROR_TIMEOUT;
        return;
      }
    }
    while ( digitalRead(DHT11Pin) == (i & 1) ? HIGH : LOW );

    if ( i >= 0 && (i & 1) ) {
      // Now we are being fed our 40 bits
      data <<= 1;

      // A zero max 30 usecs, a one at least 68 usecs.
      if ( age > 30 ) {
        data |= 1; // we got a one
      }
    }

    switch ( i ) {
      case 31:
        rawHumidity = data;
        break;
      case 63:
        rawTemperature = data;
        data = 0;
        break;
    }
  }

  if ( (byte)(((byte)rawHumidity) + (rawHumidity >> 8) + ((byte)rawTemperature) + (rawTemperature >> 8)) != data ) {
    //error = ERROR_CHECKSUM;
    return;
  }

  // Store readings

  humDHT = rawHumidity >> 8;
  tempDHT = rawTemperature >> 8;
}

ISR(WDT_vect){
  asm("wdr");
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;
}

void goToSleep() {

  asm("cli");
  
  uint8_t wdt_timeout = (1 << WDP3) | (1 << WDP0);
  asm("wdr");
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | wdt_timeout;

  SMCR |= (1 << SM1);
  SMCR |= (1 << SE);

  uint8_t mcucr_backup = MCUCR;
  MCUCR = (1 << BODS) | (1 << BODSE);
  MCUCR = (1 << BODS);

  asm("sei");
  asm("sleep");

  SMCR &= ~(1 << SE);

  MCUCR = mcucr_backup;
}

void receiveEvent(int howMany)
{
  int i = 0;
  byte x = 0;
  dht11External = true;
  debugln("YES SIR");
  while(Wire.available()){
    x = Wire.read();
    switch(i){
      case 0:
        payload[6] = x;
      break;
      case 1:
        payload[7] = x;
      break;
      case 2:
        payload[8] = x;
      break;
      case 3:
        tempDHT = x;
      break;
      case 4:
        humDHT = x;
      break;
      case 5:
        debugln("CHECK SIR");
        //payload[9] = x;
      break;
      default:
      debugln("Someting went wrong");
      break;
    }
    i++;
  }
  /*if(payload[9] == 0){
    externalInterrupt = true;
    debugln("SEND SIR");
  }*/
}
