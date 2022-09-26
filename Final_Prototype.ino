/**
 * Kontroler code      
 * Authors: 
 *        Joso Marich
 *  March 2022
 */
#define DEBUG 0 
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
#include <I2CAddress.h>
#include <SystemStatus.h>

DHT dht;

// OTAA credentials
const char devEui[] PROGMEM = {"4E0C04FA4C12B7C3"};
const char appEui[] PROGMEM = {"0000000000000000"};
const char appKey[] PROGMEM = {"552A242FCA876B931C02FD62C8C94B4B"};

unsigned long prevMillisLora;
unsigned long prevMillisInputs;
const uint8_t wakeUpCountConst = 3;
uint8_t wakeup_count = wakeUpCountConst; 
char outStr[255];  
byte payload[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//master|slave|slave|slave|temperature|humidity|heater|ventilator|vlaga|batteryStatus

bool manualMode = false;
byte recvStatus = 0;
const byte detectThresholdBattery = 50;
const byte detectThresholdMain = 50;
byte detectThreshold = detectThresholdMain;


bool dht11External = true;
//bool externalInterrupt = false;
bool statusChanged = true;
const byte PROGMEM SF9 = 0;
const byte PROGMEM SF12 = 1;
byte countOffline = 0;
byte looped = 2;
bool firstTime = true;


const PROGMEM sRFM_pins RFM_pins = {
  .CS = 6,
  .RST = 5,
  .DIO0 = 2,
  .DIO1 = 3,
  .DIO2 = 4
  //.DIO5 = 16
};

const PROGMEM byte latchPin = 8;
const PROGMEM byte dataPin = 9;
const PROGMEM byte clockPin = 7;

const PROGMEM byte inputsCtrl = A1;
#if LED
const PROGMEM byte ledCtrl = A3;
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
  Wire.begin(I2CAddress::Controller);
  Wire.setWireTimeout(3000, true);
  Wire.onReceive(receiveEvent);
  //I2C End
  
  //More Inputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  pinMode(inputsCtrl, OUTPUT);
  digitalWrite(inputsCtrl, LOW);
#if DEBUG
  //debugln(String("More Inputs"));
#endif
  //More Inputs End

  //DHT11
#if TEMPSENSOR && DEBUG
  //debugln("DHT11");
#endif
dht.setup(DHT11Pin);
  //DHT11 End
  
  //Lora Init
  initLoraWithJoin();
  // Join procedure End
}

void loop() {
  if(manualMode == true){
    checkInputs();
    Wire.beginTransmission(I2CAddress::Debugger);
    Wire.write(I2CAddress::Controller);
    Wire.write(payload[3]);
    Wire.endTransmission();
    delay(2000);
  }else{
    if(payload[9]){//On Battery
      if(wakeup_count == wakeUpCountConst){
          checkInputs();
          getInterfaceData();
          getDht11Inputs();
          lora.wakeUp();
      }else if(wakeup_count == wakeUpCountConst + 2){
        lora.sendUplink(payload, 11, 0, 1);
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
      
      if(wakeup_count < wakeUpCountConst){
        if(manualMode == false && payload[9] == true){
          goToSleep();
        }else{
          lora.wakeUp();    
        } 
      }
    
  }else{//On Main Power
      wakeup_count = 99;
      if(!firstTime){
        checkInputs();
        getInterfaceData();
        getDht11Inputs();
      }else{
        firstTime = false;
      }
      if(statusChanged && looped >= 2){
        debugln("Sending: ");
        lora.sendUplink(payload, 11, 0, 1);
        statusChanged = false;
        looped = 0;
      }
      recvStatus = lora.readData(outStr);
      if(recvStatus) {
        debugln(outStr);
      }
      lora.update();
      getBatteryInfo();
      if(looped > 3){
        looped = 2;
      }else{
        looped++;
      }
      
    }
  }
  
}

void initLoraWithJoin(){
  //Lora Init
  if(!lora.init()){
    debugln("RFM95 not detected");
    delay(5000);
    return;
  }
  lora.setDeviceClass(CLASS_A);
  //lora.setTxPower(16,PA_BOOST_PIN);
  lora.setDataRate(SF12BW125);
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
  bool isJoined = false;
  bool changeDataRate = false;
  byte tryDataRate = 0;
  do {
    debugln("Joining...");
    if(tryDataRate > 1){
      if(changeDataRate){
        changeDataRate = false;
        lora.setDataRate(SF9BW125);
      }else{
        changeDataRate = true;
        lora.setDataRate(SF12BW125);
      }  
    }
    delay(2500);
    isJoined = lora.join();
    delay(2500);
    tryDataRate++;
    delay(5000);
  }while(!isJoined);
  #if DEBUG
  //debugln("Joined to network");
  #endif
  // Join procedure End
  ledNotif(0);
  //lora.setDataRate(SF9BW125);
}

void getBatteryInfo(){
  bool tempStatus = payload[9];
  Wire.beginTransmission(I2CAddress::Ups);
  Wire.write(I2CAddress::Controller);
  if(Wire.endTransmission() != 0){
    countOffline++;
  }else{
    countOffline = 0;
    payload[9] = 0;
  }
  Wire.clearWireTimeoutFlag();
  if(countOffline > 3){
    payload[9] = 1;
  }
  if(payload[9] != tempStatus){
    statusChanged = true;
    debugln("Battery changed");
  }
  debugln(payload[9]);
  if(payload[9]){
    digitalWrite(ledCtrl, 0);
  }else{
    digitalWrite(ledCtrl, 1);
  }
        
}

void getInterfaceData(){
  pinMode(A5, OUTPUT);
  delay(50);
  pinMode(A5, INPUT);
  Wire.begin(I2CAddress::Controller);
  Wire.setWireTimeout(3000, true);
  Wire.onReceive(receiveEvent);
  Wire.beginTransmission(I2CAddress::Interface);
  Wire.write(I2CAddress::Controller);
  Wire.write(payload[9]); 
  if(Wire.endTransmission() != 0){
    dht11External = false;
    payload[6] = 0;
    payload[7] = 0;
    payload[8] = 0;
    if(payload[10] != 0){statusChanged = true;}
    payload[10] = 0;
  }else{
    dht11External = true;
    if(payload[10] != 1){statusChanged = true;}
    payload[10] = 1;
  }
  Wire.clearWireTimeoutFlag();
}

void getDht11Inputs(){
  if(!dht11External){
    readSensor(); 
  }
  if((payload[4] - tempDHT >= 3 || payload[4] - tempDHT <= -3 || payload[5] - humDHT >= 5 || payload[5] - humDHT <= -5) && humDHT != 0){
    statusChanged = true;
    //debugln("dht11 changed");
    payload[4] = tempDHT;
    payload[5] = humDHT;  
  }
}

void checkInputs(){ 
  if(payload[9]){
    ledNotif(1);
    detectThreshold = detectThresholdBattery;
  }else{
    detectThreshold = detectThresholdMain;
  }
  int countTime = 0;
  byte tempPayload[4] = {0,0,0,0};
  int myDataIn[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  digitalWrite(inputsCtrl, HIGH);
  for(int x = 0; x < 100; x++){
    
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
    digitalWrite(inputsCtrl, LOW);
    delay(20);
  }
  for(int i=31; i>=0; i--){
    if(myDataIn[i] > detectThreshold){
     tempPayload[i/8] = tempPayload[i/8] | (1 << i%8);
    }
  }
  delay(500);
  digitalWrite(latchPin, LOW);
  digitalWrite(clockPin, LOW);
  digitalWrite(inputsCtrl, LOW);
  for(int i = 0; i < 4; i++){
    if(payload[i] != tempPayload[i]){
      payload[i] = tempPayload[i];
      if(i != 3){
        if(payload[i] != 255 && payload[i] != 0){
          statusChanged = true;
        }
      }else{
        statusChanged = true;
      } 
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
  debugln("YES SIR");
  if(Wire.available()){
      switch(Wire.read()){
        case I2CAddress::Interface:
          receiveFromInterface();
          break;
        case I2CAddress::Ups:
          payload[9] = 0;
          break;
        case I2CAddress::Debugger:
          receiveFromDebugger();
          break;
        default:
          //debugln("Someting went wrong");
          ledNotif(-1);
          break; 
      }   
  }
  
  /*if(payload[9] == 0){
    externalInterrupt = true;
    debugln("SEND SIR");
  }*/
}

void receiveFromInterface(){
  dht11External = true;
  int i = 1;
  byte x = 0;
  while(Wire.available()){
    x = Wire.read();
    switch(i){
          case 1:
            if(payload[6] != x){statusChanged = true;}
            payload[6] = x;
          break;
          case 2:
            if(payload[7] != x){statusChanged = true;}
            payload[7] = x;
          break;
          case 3:
            if(payload[8] != x){statusChanged = true;}
            payload[8] = x;
          break;
          case 4:
            tempDHT = x;
          break;
          case 5:
            humDHT = x;
          break;
          case 6:
            //debugln("CHECK SIR");
            //payload[9] = x;
          break;
          default:
            //debugln("Someting went wrong");
            ledNotif(-1);
          break;
        }
        i++;
  }
}

void receiveFromDebugger(){
  int i = 1;
  byte x = 0;
  while(Wire.available()){
    x = Wire.read();
    switch(i){
      case 1:
        manualMode = x;
        break;
      case 2:
        setDataRate(x);
        break;
      case 3:
        detectThreshold = x;
        break;
      default:
        ledNotif(-1);
        break;
    }
    i++;
  }
}


void setDataRate (byte dataRate) {
  switch(dataRate){
    case SF9:
      lora.setDataRate(SF9BW125);
      break;
    case SF12:
      lora.setDataRate(SF12BW125);
      break;
    default:
      ledNotif(-1);
      break;
  }
}

void ledNotif(byte event){
  #if LED
  switch(event){
    case 0://Lora Connected
      digitalWrite(ledCtrl, 0);
      delay(500);
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0);
      delay(500);
      digitalWrite(ledCtrl, 1);
      break;
    case 1://On Battery
      digitalWrite(ledCtrl, 1);
      delay(250);
      digitalWrite(ledCtrl, 0);
      break;
    default://Error (SOS)
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      //
      digitalWrite(ledCtrl, 1);
      delay(1500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      digitalWrite(ledCtrl, 1);
      delay(1500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      digitalWrite(ledCtrl, 1);
      delay(1500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      //
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0); 
      delay(300);
      digitalWrite(ledCtrl, 1);
      delay(500);
      digitalWrite(ledCtrl, 0); 
      delay(3000);
      break;
  }
  #endif
}
