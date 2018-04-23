#include "Arduino.h"
#include <avr/wdt.h>

#define WDT_TIMEOUT           WDTO_1S //timeout of WDT = 250ms
#define WDT_RESET_TIME        100     //time to reset wdt in ms

#include <Mouse.h>
#include <Keyboard.h>
#include <TM1637Display.h>
#include <Keypad.h>

/**
 * DEBUG DECLARATION
 */
#define DEBUG				1

#if DEBUG
# define DEBUG_JS			1
# define DEBUG_KEYPAD		1
# define DEBUG_PANEL		1
#endif	//#if DEBUG

/**
 * IO DEFINITION
 */
const byte jsAxisXPin = A3;
const byte jsAxisYPin = A2;
const byte jsRightPin = A1;
const byte jsLeftPin = A0;
const byte jsClearTargetPin = 15;
const byte dispClkPin = 10;
const byte dispDioPin = 16;
const byte resvPin = 14;

/**
 * JOYSTICK parameters
 */
const int jsRange = 12;				// output range of X or Y movement
const int jsResponseDelay = 50;		// response delay of the mouse, in ms
const int jsThreshold = 2;			// resting threshold
const int jsCenter = jsRange / 2;	// resting position value
bool jsLeftClickingFlag = 0;
bool jsRightClickingFlag = 0;
bool jsClearClickingFlag = 0;

/**
 * KEYPAD parameters
 */
const byte KP_ROWS = 4;
const byte KP_COLS = 4;
byte KP_ROWPINS[KP_ROWS] = { 2, 3, 4, 5 };
byte KP_COLPINS[KP_COLS] = { 6, 7, 8, 9 };
char hexaKeys[KP_ROWS][KP_COLS] = {
	{ '0', '1', '2', '3' },
	{ '4', '5', '6', '7' },
	{ '8', '9', '*', '#' },
	{ 'A', 'B', 'C', 'D' }
};
//initialize an instance of class NewKeypad
Keypad keypad = Keypad(makeKeymap(hexaKeys), KP_ROWPINS, KP_COLPINS, KP_ROWS, KP_COLS);

/**
 * DISPLAY parameters
 */
const uint32_t DISPLAY_REFRESH_TIMEOUT = 500;
uint16_t displayData = 0;
TM1637Display display(dispClkPin, dispDioPin);

/**
 * PANEL parameters
 */
byte RXLED = 17;  // The RX LED has a defined Arduino pin
// The TX LED was not so lucky, we'll need to use pre-defined
// macros (TXLED1, TXLED0) to control that.
// (We could use the same macros for the RX LED too -- RXLED1,
//  and RXLED0.)
String sPanel = "";
uint32_t txLedTimer = 0;

void setup()
{
	wdt_enable(WDT_TIMEOUT);

	dispInit();
	kpInit();
	jsInit();
	panelInit();
}

void loop()
{
	wdt_reset();

	dispHandler();
	jsHandler();
	kpHandler();
	panelHandler();
}

/**
 * JOYSTICK
 */
void jsInit()
{
	pinMode(jsAxisXPin, INPUT);
	pinMode(jsAxisYPin, INPUT);

	pinMode(jsLeftPin, INPUT_PULLUP);
	pinMode(jsRightPin, INPUT_PULLUP);
	pinMode(jsClearTargetPin, INPUT_PULLUP);

	//short delay to let outputs settle
	delayWdt(1000);
}

void jsHandler()
{
	static uint32_t jsResponseTimer = 0;
	int xReading = 0;
	int yReading = 0;

	if (millis() >= jsResponseTimer) {
		jsResponseTimer = millis() + jsResponseDelay;

		// read and scale the two axes:
		xReading = jsReadAxis(jsAxisXPin);
		yReading = jsReadAxis(jsAxisYPin);

		Mouse.move(xReading, yReading, 0);

		/**
		 * LEFT CLICK
		 */
		if (!jsLeftClickingFlag && digitalRead(jsLeftPin) == 0) {
			jsLeftClickingFlag = 1;
			// click the left button down
			Mouse.press(MOUSE_LEFT);
		}
		else if (jsLeftClickingFlag == 1 && digitalRead(jsLeftPin) == 1) {
			jsLeftClickingFlag = 0;
			// release the left button
			Mouse.release(MOUSE_LEFT);
		}

		/**
		 * RIGHT CLICK
		 */
		if (!jsRightClickingFlag && digitalRead(jsRightPin) == 0) {
			jsRightClickingFlag = 1;
			// click the Right button down
			Mouse.press(MOUSE_RIGHT);
		}
		else if (jsRightClickingFlag == 1 && digitalRead(jsRightPin) == 1) {
			jsRightClickingFlag = 0;
			// release the Right button
			Mouse.release(MOUSE_RIGHT);
		}

		/**
		 * CLEAR TARGET CLICK
		 */
		if (!jsClearClickingFlag && digitalRead(jsClearTargetPin) == 0) {
			jsClearClickingFlag = 1;
			//TODO [Apr 23, 2018, miftakur]:
			// send clear target to Jetson
//			Keyboard.println(F("CLEAR"));
		}
		else if (jsClearClickingFlag && digitalRead(jsClearClickingFlag) == 1)
			jsClearClickingFlag = 0;

	}	//(millis() >= jsResponseTimer)

}

int jsReadAxis(byte thisAxis)
{
	// read the analog input:
	int reading = analogRead(thisAxis);

	// map the reading from the analog input range to the output range:
	reading = map(reading, 0, 1023, 0, jsRange);

	// if the output reading is outside from the rest position threshold, use it:
	//  int distance = reading - center;
	int distance = jsCenter - reading;

	if (abs(distance) < jsThreshold)
		distance = 0;

	// return the distance for this axis:
	return distance;
}

/**
 * KEYPAD
 */
void kpInit()
{

}

void kpHandler()
{
	char keyPress = keypad.getKey();

	if (keyPress) {
#if DEBUG_KEYPAD
		Serial.println(keyPress);
#endif	//#if DEBUG_KEYPAD
		//TODO [Apr 23, 2018, miftakur]:
		//add keypad handler here
	}
}

/**
 * DISPLAY
 */
void dispInit()
{
	display.setBrightness(0x0F);
	display.showNumberDec(displayData);
}

void dispHandler()
{
	static uint16_t _prevDisplayData = 0;
	static uint32_t _changeDisplayTimer = 0;

	if (_changeDisplayTimer && millis() > _changeDisplayTimer) {
		_changeDisplayTimer = 0;

		if (displayData != _prevDisplayData) {
			display.showNumberDec(displayData);
			_prevDisplayData = displayData;
		}
	}

	if (_changeDisplayTimer == 0 && displayData != _prevDisplayData)
		_changeDisplayTimer = millis() + DISPLAY_REFRESH_TIMEOUT;
}

/**
 * PANEL
 */
void panelInit()
{
	Serial1.begin(115200);
	pinMode(RXLED, OUTPUT);

}

void panelHandler()
{
	char c;
	bool sCompleted = 0;
	byte awal, akhir;
	uint16_t lrfVal = 0;
	String tem;

	if (txLedTimer && millis() > txLedTimer) {
		txLedTimer = 0;
		TXLED0;
	}

	if (Serial1.available()) {
		digitalWrite(RXLED, !digitalRead(RXLED));
		c = Serial1.read();

		if (c == '$')
			sPanel = "";
		else if (c == '*')
			sCompleted = 1;

		sPanel += c;
	}

	if (sCompleted) {
		if (sPanel.indexOf(F("$LRF,")) >= 0) {
			awal = 5;
			akhir = sPanel.indexOf('*');
			tem = sPanel.substring(awal, akhir);

			lrfVal = tem.toInt();
			if (lrfVal < 10000 && lrfVal != displayData)
				displayData = lrfVal;
		}
	}
}

void panelSendNewRange(uint16_t range)
{
	Serial1.print(F("$DISP,"));
	Serial1.print(range);
	Serial1.println(F("*"));
	TXLED1;
	txLedTimer = millis() + 100;
}

/**
 * WDT
 */
//delay with wdt enable
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
