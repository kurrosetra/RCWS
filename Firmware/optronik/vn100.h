// vn100.h

#ifndef _VN100_h
#define _VN100_h

//_____ I N C L U D E S ________________________________________________________
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//_____ D E F I N I T I O N S __________________________________________________

//_____ M A C R O S ____________________________________________________________

//_____ D E C L A R A T I O N S ________________________________________________
class VN100
{
public:
	//functions
	VN100(HardwareSerial &uart, unsigned long baud);
	VN100(HardwareSerial &uart, unsigned long baud, String header);

	bool read();
	void getYpr(long* val);
	//variables
	long ypr[3];
private:
	//functions

	//variables
	HardwareSerial *_uart;
	unsigned long _baud;

	String _header;
	String in;
	bool inCompleted;
};

#endif

