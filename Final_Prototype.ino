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
#include <lorawan.h>

#define DEBUG 1

// OTAA credentials
const char *devEui = "70B3D57ED0047473";
const char *appEui = "0022000000002200";
const char *appKey = "F44FD48BBF11BC50D2FFB3BD34924263";

const unsigned long interval = 10000;    // 10 s interval to send message
unsigned long previousMillis = 0;  // will store last time message sent
unsigned int counter = 0;     // message counter

const int TX_BUF_SIZE = 8;
const uint8_t WAKEUP_CYCLES = 2;

char myStr[50];
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

uint8_t wakeup_count = WAKEUP_CYCLES;
uint8_t tx_buf[TX_BUF_SIZE];

int latchPin = 8;
int dataPin = 9;
int clockPin = 7;
byte switchVar1 = 0;

void setup() {
#if DEBUG
  Serial.begin(9600);
  while(!Serial);
#endif
  //Watchdog
  /*ADCSRA = 0;

  PRR = (1 << PRTWI) |
        (1 << PRTIM2) |
        (1 << PRTIM1) |
        //(1 << PRSPI) |
        //(1 << PRUSART0) |
        (1 << PRADC);*/
  //Watchdog End
  
  //More Inputs
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, INPUT);
  //More Inputs End
  
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
  lora.setDevEUI(devEui);
  lora.setAppEUI(appEui);
  lora.setAppKey(appKey);
  //Lora Init End

  // Join procedure
  bool isJoined;
  do {
    #if DEBUG
    Serial.println("Joining...");
    #endif
    isJoined = lora.join();
    delay(10000);
  }while(!isJoined);
  #if DEBUG
  Serial.println("Joined to network");
  #endif
  // Join procedure End
}

void loop() {
  // Check interval overflow
  #if DEBUG
  Serial.print("In Loop: ");
  #endif
  if(millis() - previousMillis > interval) {
    previousMillis = millis(); 

    sprintf(myStr, "%d", counter); 
    
    #if DEBUG
    Serial.print("Sending: ");
    Serial.println(myStr);
    #endif
    
    lora.sendUplink(myStr, strlen(myStr), 0, 1);
    counter++;
  }

  recvStatus = lora.readData(outStr);
  if(recvStatus) {
    #if DEBUG
    Serial.println(outStr);
    #endif
  }
  
  // Check Lora RX
  lora.update();
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
  
  uint8_t wdt_timeout = (1 << WDP3);
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
