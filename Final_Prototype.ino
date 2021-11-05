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

#include <lorawan.h>

// OTAA credentials
const char devEui[] PROGMEM = {"70B3D57ED0047C31"};
const char appEui[] PROGMEM = {"0000000000000000"};
const char appKey[] PROGMEM = {"4AEFEF94C7B2291A41E0B634E9CEB977"};

unsigned long previousMillisWhileInputs = 0;
unsigned long previousMillis = 0;
uint8_t count = 0;

char myStr[5];
char outStr[255];
byte recvStatus = 0;

const sRFM_pins RFM_pins = {
  .CS = 6,
  .RST = 5,
  .DIO0 = 2,
  .DIO1 = 3,
  .DIO2 = 4,
  .DIO5 = -1,
};

uint8_t wakeup_count = 10; //Change on two places
uint8_t tx_buf[8]; // TX_BUF_SIZE

int latchPin = 8;
int dataPin = 9;
int clockPin = 7;
byte switchVar = 0;

void setup() {
  Serial.begin(9600);
/*#if DEBUG 
  while(!Serial);
#endif*/
  //Watchdog
  ADCSRA = 0;

  PRR = (1 << PRTWI) |
        (1 << PRTIM2) |
        (1 << PRTIM1) |
        //(1 << PRSPI) |
        //(1 << PRUSART0) |
        (1 << PRADC);
  //Watchdog End
  
  //More Inputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
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
  
  //Lora Init
  initLoraWithJoin();
  // Join procedure End
}

void loop() {
  delay(1000);
  if(wakeup_count >= 10)//Change on two places
  {
    switchVar = checkInputs();
    while(count <= 1){
      lora.wakeUp();
      if(millis() - previousMillis > 10000) {
        //lora.wakeUp();
        previousMillis = millis(); 
    
        sprintf(myStr, "%d", switchVar); 
    
        Serial.print("Sending: ");
        Serial.println(myStr);
        
        lora.sendUplink(myStr, strlen(myStr), 0, 1);
        count++;
      }
    
      recvStatus = lora.readData(outStr);
      if(recvStatus) {
        Serial.println(outStr);
      }
      
      // Check Lora RX
      lora.update();
    } 
    lora.sleep();
    wakeup_count = 0;
    count = 0;
  }
  wakeup_count++;
  delay(1000);
  goToSleep();
}

void initLoraWithJoin(){
  //Lora Init
  if(!lora.init()){
    #if DEBUG
    Serial.println("RFM95 not detected");
    #endif
    delay(5000);
    return;
  }
  lora.setDeviceClass(CLASS_A);
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
    Serial.println("Joining...");
    #endif
    isJoined = lora.join();
    delay(5000);
  }while(!isJoined);
  #if DEBUG
  Serial.println("Joined to network");
  #endif
  // Join procedure End
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
    /*#if DEBUG
      if( bitRead(switchVarTemp, 7) == 1){
        Serial.println(switchVarTemp, BIN);
        Serial.println(String("vrata"));
      }
    #endif*/
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
