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
#if DEBUG
  #define debug(x) Serial.print(x)
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
  #define beginSerial() Serial.begin(9600)
#else
  #define debug(x)
  #define debugln(x)
  #define beginSerial()
#endif
#include <lorawan.h>

// OTAA credentials
const char devEui[] PROGMEM = {"70B3D57ED0049759"};
const char appEui[] PROGMEM = {"0000000000000000"};
const char appKey[] PROGMEM = {"682C6C2FC30EA122F8A375D36344EC61"};

unsigned long previousMillisWhileInputs = 0;
  //unsigned long previousMillis = 0;
bool statusChanged = true;
uint8_t wakeup_count = 3; //Change on two places

char outStr[255];  
byte payload[6] = {0, 0, 0, 0, 0, 0};

byte recvStatus = 0;

const PROGMEM sRFM_pins RFM_pins = {
  .CS = 6,
  .RST = 5,
  .DIO0 = 2,
  .DIO1 = 3,
  .DIO2 = 4,
  .DIO5 = A5,
};

const int PROGMEM latchPin = 8;
const int PROGMEM dataPin = 9;
const int PROGMEM clockPin = 7;

const int PROGMEM inputsCtrl = A1;
const int PROGMEM ledCtrl = A4;

byte tempDHT = 0;
byte humDHT = 0;

void setup() {
  delay(1000);
  beginSerial();
  //Watchdog
  ADCSRA = 0;

  PRR = (1 << PRTWI) |
        (1 << PRTIM2) |
        (1 << PRTIM1) |
        //(1 << PRSPI) |
        //(1 << PRUSART0) |
        (1 << PRADC);
  //Watchdog End

  //LED
  pinMode(ledCtrl, OUTPUT);
  digitalWrite(ledCtrl, 1);
  //LED End
  
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
readSensor();
  //DHT11 End
  
  //Lora Init
  initLoraWithJoin();
  // Join procedure End
}

void loop() {
    checkInputs();
    getDht11Inputs();
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
  lora.setTxPower(15,PA_BOOST_PIN);
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
  digitalWrite(ledCtrl, 0);
  delay(500);
  digitalWrite(ledCtrl, 1);
  delay(500);
  digitalWrite(ledCtrl, 0);
  delay(500);
  digitalWrite(ledCtrl, 1);
  delay(500);
  digitalWrite(ledCtrl, 0);
  delay(500);
  digitalWrite(ledCtrl, 1);
}

void getDht11Inputs(){
    int timeCount = 0;
    do{
      readSensor();
      timeCount++;
    }while(timeCount <= 5);   
    if((payload[4] - tempDHT >= 3 || payload[4] - tempDHT <= -3 || payload[5] - humDHT >= 5 || payload[5] - humDHT <= -5) && humDHT != 0){
      statusChanged = true;
      payload[4] = tempDHT;
      payload[5] = humDHT;  
    }
    debugln(tempDHT);
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
    if((unsigned long)(millis() - previousMillisWhileInputs) >= 2000){
      countTime++;
      previousMillisWhileInputs = millis();
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
  debugln("BeforeStart");
        debug(freeRam());
  delay(1000);
  // Make sure we don't poll the sensor too often
  // - Max sample rate DHT11 is 1 Hz   (duty cicle 1000 ms)
  // - Max sample rate DHT22 is 0.5 Hz (duty cicle 2000 ms)
  unsigned long startTime = millis();

  // Request sample

  digitalWrite(10, LOW); // Send start signal
  pinMode(10, OUTPUT);
  delay(18);

  pinMode(10, INPUT);
  digitalWrite(10, HIGH); // Switch bus to receive data

  // We're going to read 83 edges:
  // - First a FALLING, RISING, and FALLING edge for the start bit
  // - Then 40 bits: RISING and then a FALLING edge per bit
  // To keep our code simple, we accept any HIGH or LOW reading if it's max 85 usecs long

  //uint16_t rawHumidity = 0;
  uint16_t rawTemperature = 0;
  uint16_t data = 0;
  debugln("AfterStart");
        debug(freeRam());
  for ( int8_t i = -3 ; i < 2 * 40; i++ ) {
    byte age;
    startTime = micros();

    do {
      age = (unsigned long)(micros() - startTime);
      if ( age > 90 ) {
        debugln("AGE");
        debug(freeRam());
        //error = ERROR_TIMEOUT;
        debugln("TIMEOUT");
        return;
      }
    }
    while ( digitalRead(10) == (i & 1) ? HIGH : LOW );

    if ( i >= 0 && (i & 1) ) {
      // Now we are being fed our 40 bits
      data <<= 1;

      // A zero max 30 usecs, a one at least 68 usecs.
      if ( age > 30 ) {
        data |= 1; // we got a one
      }
    }

    switch ( i ) {
      /*case 31:
        rawHumidity = data;
        break;*/
      case 63:
        debugln("WORKING");
        debug(freeRam());
        rawTemperature = data;
        data = 0;
        break;
    }
  }

  // Store readings
  debugln(freeRam());
  humDHT = 0;
  tempDHT = rawTemperature >> 8;
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
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
