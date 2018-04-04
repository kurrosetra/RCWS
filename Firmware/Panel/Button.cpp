// 
// 
// 

#include "Button.h"


byte ButtonClass::asci2byte(String s)
{
	char lsb = '0', msb = '0';
	byte ret = 0;

	if (s.length() > 1) {
		msb = s.charAt(0);
		lsb = s.charAt(1);
	}
	else lsb = s.charAt(0);

	ret = char2byte(msb) << 4 | (char2byte(lsb) & 0xF);


	return ret;
}

byte ButtonClass::char2byte(char c)
{
	byte i = 0;

	if (c >= '0' && c <= '9') i = c - '0';
	else if (c >= 'a' && c <= 'f') i = (c - 'a') + 10;
	else if (c >= 'A' && c <= 'F') i = (c - 'A') + 10;

	return i;
}

ButtonClass::ButtonClass(HardwareSerial & uart, uint32_t baud) :_uart(&uart)
{
	_baud = baud;
	_header = "$BUT";
}

ButtonClass::ButtonClass(HardwareSerial & uart, uint32_t baud, String header) :_uart(&uart)
{
	_baud = baud;
	_header = header;
}

void ButtonClass::init()
{
	_uart->begin(_baud);
	in.reserve(BUT_MAX_BUFSIZE);
}

bool ButtonClass::read()
{
	static byte _prevDataIn = 0;
	static uint16_t _prevVolVal = 0;
	bool ret = 0;
	char c;
	byte awal, akhir, b;
	uint16_t _v = 0;
	String s;

	if (_uart->available()) {
		c = _uart->read();
		if (c == _header.charAt(0)) {
			in = "";
			inCompleted = 0;
		}
		else if (c == BUT_CHAR_TERMINATOR) inCompleted = 1;

		in += c;
	}

	if (inCompleted) {
		//Serial.println(in);
		if (in.indexOf(_header) == 0) {
			akhir = in.indexOf(BUT_CHAR_SEPARATOR);

			//button command value
			awal = akhir + 1;
			akhir = in.indexOf(BUT_CHAR_SEPARATOR, awal);
			s = in.substring(awal, akhir);
			b = asci2byte(s);
			//dataIn = asci2byte(s);
			if (_prevDataIn != dataIn) {
				dataIn = _prevDataIn;
			}
			else {
				if (_prevDataIn != b) {
					_prevDataIn = b;
				}
			}

			//voltage value
			awal = akhir + 1;
			akhir = in.indexOf(BUT_CHAR_TERMINATOR, awal);
			s = in.substring(awal, akhir);
			_v = s.toInt();
			//volValue = s.toInt();
			if (_prevVolVal != volValue) {
				volValue = _prevVolVal;
			}
			else {
				if (_prevVolVal != _v) {
					_prevVolVal = _v;
				}
			}

			ret = 1;
		}
	}


	return ret;
}

byte ButtonClass::getCamera()
{
	byte i = CAMERA_NONE;

	if (bitRead(dataIn, BUT_POS_SONY)) i = CAMERA_SONY;
	else if (bitRead(dataIn, BUT_POS_FLIR)) i = CAMERA_THERMAL;

	return i;
}

byte ButtonClass::getMovementMode()
{
	byte i = MOVE_MANUAL;

	if (bitRead(dataIn, BUT_POS_TRACK)) i = MOVE_TRACK;
	else if (bitRead(dataIn, BUT_POS_STAB)) i = MOVE_STAB;

	return i;
}

bool ButtonClass::getLrfPower()
{
	return bitRead(dataIn, BUT_POS_LRF_PWR);
}

bool ButtonClass::getTriggerEnable()
{
	return bitRead(dataIn, BUT_POS_TRIGGER_EN);
}

uint16_t ButtonClass::getVoltage()
{
	return volValue;
}

