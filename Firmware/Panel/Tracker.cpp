// 
// 
// 

#include "Tracker.h"


#include "tracker.h"

TrackerClass::TrackerClass(HardwareSerial & uart, uint32_t baud) :_uart(&uart)
{
	_baud = baud;
	xMon = 960;
	yMon = 540;
}

void TrackerClass::init()
{
	_uart->begin(_baud);
}

bool TrackerClass::available()
{
	return _uart->available();
}

char TrackerClass::read()
{
	return _uart->read();
}

void TrackerClass::killApp()
{
	_uart->println(F("KILL"));
}

void TrackerClass::setLrfValue(uint16_t val)
{
	_uart->print(F("LRVAL,"));
	_uart->println(val);
}

void TrackerClass::setImuVal(double azVal, double elVal)
{
	_uart->print(F("AZVAL,"));
	_uart->println(azVal, 2);
	_uart->print(F("ELVAL,"));
	_uart->println(elVal, 2);
}

void TrackerClass::clearTrackId()
{
	byte a;

	for (a = 0; a < 10; a++) {
		_uart->print(F("RMTRK,"));
		_uart->println(a);
		_uart->flush();
		delay(1);
	}
}

void TrackerClass::clearTrackId(byte id)
{
	_uart->print(F("RMTRK,"));
	_uart->println(id);
	_uart->flush();
}

void TrackerClass::deactive()
{
	_uart->println(F("NOTRK"));
}

void TrackerClass::clearAllTrackId()
{
	_uart->println(F("CLTRK"));
}

void TrackerClass::write(char c)
{
	_uart->write(c);
}
