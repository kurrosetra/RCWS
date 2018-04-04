#include "Arduino.h"
#include <avr/wdt.h>

#define DEBUG_MODE            0
#define DEBOUNCE_DELAY        200

#define WDT_TIMEOUT           WDTO_250MS //timeout of WDT = 250ms
#define WDT_RESET_TIME        100     //time to reset wdt in ms

#define SONY_PIN              12
#define FLIR_PIN              13
#define TRK_PIN               A0
#define STAB_PIN              A1
#define LRF_PWR_PIN           11
#define TRIGGER_PIN           10
#define VOLT_PIN              A3


const byte butPin[6]={SONY_PIN, FLIR_PIN, TRK_PIN, STAB_PIN, \
                     LRF_PWR_PIN, TRIGGER_PIN};
byte butVal=0;

void setup() {
  byte i=0;

  wdt_enable(WDT_TIMEOUT);

  Serial.begin(9600);
#if DEBUG_MODE
  int v=0;

  Serial.println(F("========================"));
  Serial.println(F("Button - RWS test suite!"));
  Serial.println(F("2017 02 27"));
  Serial.println(F("UNO Board"));
  Serial.println(F("========================"));
#endif

  for(i=0;i<6;i++){
    pinMode(butPin[i], INPUT_PULLUP);
  }
  pinMode(VOLT_PIN, INPUT);

#if DEBUG_MODE
  delayWdt(DEBOUNCE_DELAY);
  butVal= readBut();
  v = readVolt();

  Serial.print(F("init butVal= 0b"));
  Serial.println(butVal, BIN);
  Serial.print(F("volt= "));
  Serial.print(v);
  Serial.println('V');
#endif
}

void loop() {
  byte i= readBut();
  uint16_t voltVal=0;
  static uint32_t sendTimer=0, debounceTimer=0;

  wdt_reset();

  if(i != butVal){
    if(debounceTimer){
      if(millis() > debounceTimer){
        debounceTimer=0;
        if(i != butVal){
          butVal= i;
#if DEBUG_MODE
          Serial.print(F("butVal= 0b"));
          Serial.println(butVal, BIN);
#endif
        }
      }
    }else debounceTimer= millis() + DEBOUNCE_DELAY;
  }

  if(millis() > sendTimer){
    sendTimer= millis()+500;

    voltVal= readVolt();

    Serial.print(F("$BUT,"));
    Serial.print(butVal, HEX);
    Serial.print(',');
    Serial.print(voltVal);
    Serial.print('*');
  }
}


byte readBut()
{
  byte ret=0,i=0;

  for(i=0;i<6;i++){
    bitWrite(ret, i, !digitalRead(butPin[i]));
  }

  return ret;
}

uint16_t readVolt()
{
  int ret=0;

  ret= analogRead(VOLT_PIN);
  ret= constrain(map(ret, 0, 1024, 0, 5500), 0, 3000);

  return ret;
}

//delay with wdt enable
void delayWdt(unsigned int _delay)
{
  wdt_reset();
  if(_delay<=WDT_RESET_TIME){
    delay(_delay);
  }else{
    while(_delay>WDT_RESET_TIME){
      delay(WDT_RESET_TIME);
      wdt_reset();
      _delay-=WDT_RESET_TIME;
    }
    delay(_delay);
  }
  wdt_reset();
}
