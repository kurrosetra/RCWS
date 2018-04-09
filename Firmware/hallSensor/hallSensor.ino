#include "Arduino.h"
#include <avr/wdt.h>
#include <avr/io.h>

#ifndef PCICR
# define PCICR _SFR_MEM8(0x68)
# define PCIE0 0
# define PCIE1 1
# define PCIE2 2
#endif

#ifndef PCMSK1
# define PCMSK1 _SFR_MEM8(0x6C)
# define PCINT8 0
# define PCINT9 1
# define PCINT10 2
# define PCINT11 3
# define PCINT12 4
# define PCINT13 5
# define PCINT14 6

# define PCINT1_vect       _VECTOR(4)   /* Pin Change Interrupt Request 0 */
#endif

#define WDT_TIMEOUT           WDTO_250MS 	//timeout of WDT = 250ms
#define WDT_RESET_TIME        100     		//time to reset wdt in ms

const uint32_t UPDATE_RATE = 50;
const byte panFeedPin[3] = { 2, A5, A4 };
const byte tiltFeedPin[3] = { 3, A3, A2 };

const byte hallValue[6] = { 0b001, 0b011, 0b010, 0b110, 0b100, 0b101 };

volatile long panValue = 0;
volatile long tiltValue = 0;
volatile byte panPinVal = 0;
volatile byte tiltPinVal = 0;

byte sendCounter = 0;
String strIn = "";
const int strBufsize = 32;

void setup()
{
	wdt_enable(WDT_TIMEOUT);

	Serial.begin(250000);

	ioInit();
}

void loop()
{
	static uint32_t sendTimer = UPDATE_RATE;
	char c;
	bool strCompleted = 0;
	String tem;

	wdt_reset();

	if (millis() >= sendTimer) {
		sendTimer = millis() + UPDATE_RATE;

		Serial.print("$");
		Serial.print(sendCounter++);
		Serial.print(",");
		Serial.print(panValue);
		Serial.print(",");
		Serial.print(tiltValue);
		Serial.println("*");
	}

	if (Serial.available()) {
		c = Serial.read();
		if (c == '$')
			strIn = "";
		else if (c == '*') {
			if (strIn.length() < strBufsize)
				strCompleted = 1;
			else
				strIn = "";
		}
		strIn += c;
	}

	if (strCompleted) {
		if (strIn.indexOf(F("$RESET,")) >= 0) {
			c = strIn.charAt(7);
			if (c == '0') {
				panValue = 0;
				tiltValue = 0;
			}
			else if (c == '1')
				panValue = 0;
			else if (c == '2')
				tiltValue = 0;
		}

		strIn = "";
	}
}

void ioInit()
{
	byte a, pinVal = 0;

	for ( a = 0; a < 3; a++ ) {
		pinMode(panFeedPin[a], INPUT_PULLUP);
		pinMode(tiltFeedPin[a], INPUT_PULLUP);
	}

	//pan hall 1
	attachInterrupt(digitalPinToInterrupt(panFeedPin[0]), panEvent, CHANGE);
	//tilt hall 1
	attachInterrupt(digitalPinToInterrupt(tiltFeedPin[0]), tiltEvent, CHANGE);

	bitSet(PCICR, PCIE1);
	bitSet(PCMSK1, PCINT10);
	bitSet(PCMSK1, PCINT11);
	bitSet(PCMSK1, PCINT12);
	bitSet(PCMSK1, PCINT13);

	pinVal = 0;
	for ( a = 0; a < 3; a++ )
		bitWrite(pinVal, a, digitalRead(panFeedPin[a]));

	for ( a = 0; a < 6; a++ ) {
		if (pinVal == hallValue[a]) {
			panPinVal = a;
			break;
		}
	}

	pinVal = 0;
	for ( a = 0; a < 3; a++ )
		bitWrite(pinVal, a, digitalRead(tiltFeedPin[a]));

	for ( a = 0; a < 6; a++ ) {
		if (pinVal == hallValue[a]) {
			tiltPinVal = a;
			break;
		}
	}
}

void panEvent()
{
	byte a, reg = 0;

	for ( a = 0; a < 3; a++ )
		bitWrite(reg, a, digitalRead(panFeedPin[a]));

	if (reg != panPinVal) {
		if (reg > panPinVal) {
			if (panPinVal == 0)
				panValue--;
			else
				panValue++;
		}
		else if (reg < panPinVal) {
			if (reg == 0)
				panValue++;
			else
				panValue--;
		}

		panPinVal = reg;
	}
}

void tiltEvent()
{
	byte a, reg = 0;

	for ( a = 0; a < 3; a++ )
		bitWrite(reg, a, digitalRead(tiltFeedPin[a]));

	if (reg != tiltPinVal) {
		if (reg > tiltPinVal) {
			if (tiltPinVal == 0)
				tiltValue--;
			else
				tiltValue++;
		}
		else if (reg < tiltPinVal) {
			if (reg == 0)
				tiltValue++;
			else
				tiltValue--;
		}

		tiltPinVal = reg;
	}
}

ISR(PCINT1_vect)
{
	panEvent();
	tiltEvent();
}

/**
 * delay with wdt enable
 */
void delayWdt(unsigned int _delay)
{
	wdt_reset();
	if (_delay <= WDT_RESET_TIME) {
		delay(_delay);
	}
	else {
		while (_delay > WDT_RESET_TIME) {
			delay(WDT_RESET_TIME);
			wdt_reset();
			_delay -= WDT_RESET_TIME;
		}
		delay(_delay);
	}
	wdt_reset();
}
