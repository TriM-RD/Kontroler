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
#define LED 0 // upali/ugasi led indikator
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
const char devEui[] PROGMEM = {"70B3D57ED004B6C6"};
const char appEui[] PROGMEM = {"0000000000000000"};
const char appKey[] PROGMEM = {"1A928AA42DED88203555BD5726C89D99"};

unsigned long prevMillisLora;
unsigned long prevMillisInputs;
bool statusChanged = true;
uint8_t wakeup_count = 3; //Change on two places

char outStr[100];  
byte payload[6] = {0, 0, 0, 0, 0, 0};

byte recvStatus = 0;

bool onBattery = true;

const PROGMEM sRFM_pins RFM_pins = {
  .CS = 6,
  .RST = 5,
  .DIO0 = 2,
  .DIO1 = 3,
  .DIO2 = 4
};

const int PROGMEM latchPin = 8;
const int PROGMEM dataPin = 9;
const int PROGMEM clockPin = 7;

const int PROGMEM inputsCtrl = A1;
#if LED
const int PROGMEM ledCtrl = A4;
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
  if(onBattery){
    if(wakeup_count >= 3)//Change on two places
    {
      checkInputs();
      getDht11Inputs();
      getInterfaceData();
      lora.wakeUp();  
      debugln("Sending: ");    
      lora.sendUplink(payload, 6, 0, 1);
      recvStatus = lora.readData(outStr);
      if(recvStatus) {
        debugln(outStr);
      }
      lora.update();
      lora.sleep();
      wakeup_count = 0;
    }
    wakeup_count++;
    goToSleep();
  }else{//On Main Power
    checkInputs();
    getDht11Inputs();
    getInterfaceData();
    if(statusChanged){
      debugln("Sending: ");
      lora.sendUplink(payload, 6, 0, 1);
      statusChanged = false;
    }

    recvStatus = lora.readData(outStr);
    if(recvStatus) {
      debugln(outStr);
    }
    lora.update();
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
  //lora.setTxPower(15,PA_BOOST_PIN);
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

void getInterfaceData(){
  Wire.beginTransmission(1);
  Wire.write("a"); 
  Wire.endTransmission();
}

void getDht11Inputs(){
    readSensor(); 
    if((payload[4] - tempDHT >= 3 || payload[4] - tempDHT <= -3 || payload[5] - humDHT >= 5 || payload[5] - humDHT <= -5) && humDHT != 0){
      statusChanged = true;
      payload[4] = tempDHT;
      payload[5] = humDHT;  
    }
    //debugln(tempDHT);
}

void checkInputs(){
  digitalWrite(inputsCtrl, HIGH);
  int countTime = 0;
  byte tempPayload[4] = {0,0,0,0};
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
       if(myDataIn[i] > tempPayload[i]){
        tempPayload[i] = myDataIn[i];
      }
    }
    if((unsigned long)(millis() - prevMillisInputs) >= 1000){
      countTime++;
      prevMillisInputs = millis();
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
  int x = Wire.read();       // recibe el último byte como número
  statusChanged = true;
  Serial.println("YES SIR");
  /*while(1 < Wire.available()) // hacemos loop por todos los bytes salvo el último
  {
    char c = Wire.read();    // recibe un byte como carácter
    Serial.print(c);         // imprime el carácter
  }
  int x = Wire.read();       // recibe el último byte como número
  Serial.println(x);         // imprime el número*/
}
