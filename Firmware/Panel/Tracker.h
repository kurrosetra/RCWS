// Tracker.h

#ifndef _TRACKER_h
#define _TRACKER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class TrackerClass
{
public:
	TrackerClass(HardwareSerial &uart, uint32_t baud = 115200);

	void init();
	bool available();
	char read();

	void killApp();
	void setLrfValue(uint16_t val);
	void setImuVal(double azVal, double elVal);
	void clearTrackId();
	void clearTrackId(byte id);
	void deactive();

private:
	HardwareSerial *_uart;
	uint32_t _baud;
	String str;

	uint16_t xMon, yMon;

};

#endif

