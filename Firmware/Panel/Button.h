// Button.h

#ifndef _BUTTON_h
#define _BUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


#define BUT_CHAR_TERMINATOR			'*'
#define BUT_CHAR_SEPARATOR			','
#define BUT_MAX_BUFSIZE				32


class ButtonClass
{
protected:

	typedef enum
	{
		BUT_POS_SONY = 0,
		BUT_POS_FLIR = 1,
		BUT_POS_TRACK = 2,
		BUT_POS_STAB = 3,
		BUT_POS_LRF_PWR = 4,
		BUT_POS_TRIGGER_EN = 5
	}EButtonDataPos;


	byte asci2byte(String s);
	byte char2byte(char c);

	HardwareSerial *_uart;
	uint32_t _baud;

	String in;
	String _header;
	bool inCompleted;

	byte dataIn;
	uint16_t volValue;

public:
	ButtonClass(HardwareSerial &uart, uint32_t baud);
	ButtonClass(HardwareSerial &uart, uint32_t baud, String header);

	enum CAMERA_SWITCH
	{
		CAMERA_NONE = 0,
		CAMERA_SONY = 1,
		CAMERA_THERMAL = 2
	};

	enum MOVEMENT_STATE
	{
		MOVE_MANUAL = 0,
		MOVE_TRACK = 1,
		MOVE_STAB = 2
	};

	void init();
	bool read();

	byte getCamera();
	byte getMovementMode();
	bool getLrfPower();
	bool getTriggerEnable();
	uint16_t getVoltage();
};

#endif

