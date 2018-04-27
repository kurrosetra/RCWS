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
#define DEBUG				0

#if DEBUG
# define DEBUG_JS			0
# define DEBUG_KEYPAD		1
# define DEBUG_PANEL		0
#endif	//#if DEBUG

/**
 * IO DEFINITION
 */
const byte jsAxisXPin = A2;
const byte jsAxisYPin = A3;
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
char hexaKeys[KP_ROWS][KP_COLS] = { { '1', '4', '7', '*' }, { '2', '5', '8', '0' }, {
	'3',
	'6',
	'9',
	'#' }, { 'A', 'B', 'C', 'D' } };
bool inputNewValue = 0;
String temDisplayData;
//initialize an instance of class NewKeypad
Keypad keypad = Keypad(makeKeymap(hexaKeys), KP_ROWPINS, KP_COLPINS, KP_ROWS, KP_COLS);

/**
 * DISPLAY parameters
 */
const uint32_t DISPLAY_REFRESH_TIMEOUT = 100;
uint16_t displayData = 1000;
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
	String s;


	if (millis() >= jsResponseTimer) {
		jsResponseTimer = millis() + jsResponseDelay;

		// read and scale the two axes:
		xReading = jsReadAxis(jsAxisXPin, 0);
		yReading = jsReadAxis(jsAxisYPin, 1);

		Mouse.move(xReading, yReading, 0);

		/**
		 * LEFT CLICK
		 */
		if (!jsLeftClickingFlag && digitalRead(jsLeftPin) == 0) {
			jsLeftClickingFlag = 1;
			// click the left button down
			Mouse.press(MOUSE_LEFT);
			// set target in rectangular zone
			Keyboard.write('b');
		}
		else if (jsLeftClickingFlag == 1 && digitalRead(jsLeftPin) == 1) {
			jsLeftClickingFlag = 0;
			// release the left button
			Mouse.release(MOUSE_LEFT);
			// release rectangular zone
			Keyboard.write('b');
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
#if DEBUG_KEYPAD
			Keyboard.println(F("CLRTRK"));
#else
			// send clear target:
			// to Jetson
			//Keyboard.println(F("CLRTRK"));
			// or Panel
			s = F("$CLRTRK*");
			panelSendString(&s);
#endif	//#if DEBUG_KEYPAD

		}
		else if (jsClearClickingFlag && digitalRead(jsClearTargetPin) == 1)
			jsClearClickingFlag = 0;

	}	//(millis() >= jsResponseTimer)

}

int jsReadAxis(byte thisAxis, bool foldVal)
{
	int distance = 0;

	// read the analog input:
	int reading = analogRead(thisAxis);

	// map the reading from the analog input range to the output range:
	reading = map(reading, 0, 1023, 0, jsRange);

	// if the output reading is outside from the rest position threshold, use it:
	if (foldVal)
		distance = reading - jsCenter;
	else
		distance = jsCenter - reading;

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
	String tem;
	uint16_t temValue = 0;
	char keyPress = keypad.getKey();

	if (keyPress) {
#if DEBUG_KEYPAD
		Keyboard.write(keyPress);
#endif	//#if DEBUG_KEYPAD
		//add keypad handler here
		if (!inputNewValue) {
			switch (keyPress)
			{
			case 'A':
				displayData += 50;
				if (displayData >= 10000)
					displayData = 9999;
				break;
			case 'B':
				if (displayData > 50)
					displayData -= 50;
				else
					displayData = 0;
				break;
			case 'C':
				displayData = 1000;
				break;
			case 'D':
				panelSendNewRange(displayData);
				break;
			case '*':
				displayData = 0;
				temDisplayData = "";
				inputNewValue = 1;
				break;
			case '#':
				break;
			}
		}	//(!inputNewValue)
		else {
			if (keyPress >= '0' && keyPress <= '9') {
				temDisplayData += keyPress;
				if (temDisplayData.toInt() >= 10000)
					temDisplayData = "9999";
				displayData = temDisplayData.toInt();

			}	//(keyPress >= '0' && keyPress <= '9')
			else if (keyPress == 'D') {
				//send to Panel
				temValue = (uint16_t) temDisplayData.toInt();
				if (temValue >= 10000)
					temValue = 9999;
				displayData = temValue;
				panelSendNewRange(displayData);
				inputNewValue = 0;
			}
			else if (keyPress == '#') {
				// set to zero
				temDisplayData = "";
				displayData = 0;
			}
		}	// else (!inputNewValue)
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
	digitalWrite(RXLED, LOW);
	TXLED0;
}

void panelHandler()
{
	char c;
	bool sCompleted = 0;
	byte awal, akhir;
	uint16_t lrfVal = 0;
	String tem;

	if (txLedTimer && millis() >= txLedTimer) {
		txLedTimer = 0;
		TXLED0;
	}

	if (Serial1.available()) {
		c = Serial1.read();

		if (c == '$')
			sPanel = "";
		else if (c == '*')
			sCompleted = 1;

		sPanel += c;
	}

	if (sCompleted) {
		if (sPanel.indexOf(F("$LRF,")) >= 0) {
			digitalWrite(RXLED, !digitalRead(RXLED));
#if DEBUG_PANEL
			Keyboard.println(sPanel);
#endif	//#if DEBUG_PANEL

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

void panelSendString(String *s)
{
	Serial1.print(*s);
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
