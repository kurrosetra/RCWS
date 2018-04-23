#include "Arduino.h"

#include <avr/wdt.h>

#define WDT_TIMEOUT           WDTO_1S //timeout of WDT = 2S
#define WDT_RESET_TIME        100     //time to reset wdt in ms

#include <SPI.h>
#include <mcp2515.h>
#include <Ingenia_SerialServoDrive.h>

/**
 * DEBUG DECLARATION
 */
#define DEBUG					1
#if DEBUG
# define BUS_DEBUG				1
# define MOVE_DEBUG				0
#if MOVE_DEBUG
#  define NOT_USE_PAN_MOTOR		0
#  define NOT_USE_TILT_MOTOR	1
#endif	//#if MOVE_DEBUG

#endif	//#if DEBUG

/**
 * IO DEFINITION
 */
#define BUS_CS_PIN				49
#define INDICATOR_PIN			13
#define TRIGGER_PIN				45
#define T_UP_LIMIT_PIN			10
#define T_DOWN_LIMIT_PIN		12
#define P_HOME_SWITCH_PIN		11

const uint32_t TRIGGER_TIMEOUT = 250;
bool triggerHOT = 0;
uint32_t triggerTimer = 0;

/**
 * MOTOR parameters
 */
#define USE_STEPPING_MODE		0
#define MOTOR_PAN_NODE			0x20
#define MOTOR_TILT_NODE			0x21

const uint32_t MOTOR_START_ENABLE_TIMEOUT = 3500;
const uint32_t MOTOR_UPDATE_TIMEOUT = 50;
const uint32_t MOTOR_MAX_SPEED_VAL = 29250;
const int32_t MOTOR_POS_REV_VAL = 2000L;

const uint16_t PAN_SPEED_HYST_VAL = 5;
const uint16_t TILT_SPEED_HYST_VAL = 25;

#if NOT_USE_PAN_MOTOR==0
Ingenia_SerialServoDrive motorPan(&Serial1, MOTOR_PAN_NODE);
#endif	//#if NOT_USE_PAN_MOTOR==0

#if NOT_USE_TILT_MOTOR==0
Ingenia_SerialServoDrive motorTilt(&Serial3, MOTOR_TILT_NODE);
#endif	//#if NOT_USE_TILT_MOTOR==0

bool motorEnable = 0;
struct nix_dIO
{
	byte ENABLE_PIN;
	byte HEALTH_PIN;
	byte FAULT_RESET_PIN;
	byte RESV_PIN;
};
nix_dIO panIO = { 33, 37, 35, 31 }, tiltIO = { 44, 48, 46, 42 };
long motorXcommand = 0, motorYcommand = 0;
int8_t panDriverMode, tiltDriverMode;
bool motorHoming = 0;

/**
 * BUS parameters
 */
const uint16_t BUS_MOTOR_ID = 0x300;
const byte BUS_MOTOR_SIZE = 5;
const uint16_t BUS_PANEL_ID = 0x310;
const byte BUS_PANEL_SIZE = 5;
const uint32_t BUS_JS_ID = 0x8CFDD633;
MCP2515 bus(BUS_CS_PIN);
struct can_frame recvMsg;
struct can_frame sendMsg;
bool brakeFree = 0;

void setup()
{
	wdt_enable(WDT_TIMEOUT);

#if DEBUG
	Serial.begin(230400);
	Serial.println(F("MotorControl - RCWS firmware"));
#endif

	ioInit();
	busInit();
	motorInit();
}

//void loop()
//{
//	wdt_reset();
//	busRecv();
//}

void loop()
{
	char c;

	wdt_reset();

	ioHandler();
	busHandler();
	motorHandler();

#if MOVE_DEBUG
	static long _spd = 0;
	bool change = 0;

	recvMsg.data[0] = 0b11;
#endif	//#if MOVE_DEBUG

	if (Serial.available()) {
		c = Serial.read();

		switch (c)
		{
#if MOVE_DEBUG
		case 'r':
#if NOT_USE_PAN_MOTOR==0
		Serial.print(F("posX= "));
		Serial.println((int32_t) motorPan.read(0x6064, 0));
//			Serial.print(F("motor speed= "));
//			Serial.print(motorPan.read(0x607f, 0));		// current speed
//			Serial.print(F(" / "));
//			Serial.println(motorPan.read(0x6080, 0));		// max speed
#endif	//#if NOT_USE_PAN_MOTOR==0
#if NOT_USE_TILT_MOTOR==0
		Serial.print(F("posY= "));
		Serial.println((int32_t) motorTilt.read(0x6064, 0));
#endif	//#if NOT_USE_TILT_MOTOR==0

		break;
		case '0':
		_spd = 0;
		change = 1;
		break;
		case 'H':
		Serial.print(F("do homing on current position ..."));
#if NOT_USE_PAN_MOTOR==0
		motorPan.doHoming(35);
		motorPan.setModeOfOperation(Ingenia_SerialServoDrive::DRIVE_MODE_PROFILE_POSITION);
#endif	//#if NOT_USE_PAN_MOTOR==0
#if NOT_USE_TILT_MOTOR==0
		motorTilt.doHoming(35);
		motorTilt.setModeOfOperation(Ingenia_SerialServoDrive::DRIVE_MODE_PROFILE_POSITION);
#endif	//#if NOT_USE_TILT_MOTOR==0

		Serial.println(F("done"));
		break;
		case 'B':
		Serial.print(F("back to starting point ..."));
#if NOT_USE_PAN_MOTOR==0
		motorPanSetToZero();
#endif	//#if NOT_USE_PAN_MOTOR==0

#if NOT_USE_TILT_MOTOR==0
		motorTiltSetToZero();
#endif	//#if NOT_USE_TILT_MOTOR==0
		Serial.println(F("done"));
		break;
		case 'd':
		_spd += 100;
		change = 1;
		break;
		case 'a':
		_spd -= 100;
		change = 1;
		break;
		case 'w':
		_spd += 500;
		change = 1;
		break;
		case 's':
		_spd -= 500;
		change = 1;
		break;
		case 'e':
		_spd += 2925;
		change = 1;
		break;
		case 'q':
		_spd -= 2925;
		change = 1;
		break;
		case 'c':
		_spd += 10;
		change = 1;
		break;
		case 'z':
		_spd -= 10;
		change = 1;
		break;
#endif	//#if MOVE_DEBUG
	}
}

#if MOVE_DEBUG
if (change) {
#if NOT_USE_PAN_MOTOR==0
	Serial.print(F("_spdX= "));

	motorXcommand = _spd;
	Serial.println(motorXcommand);
#endif	//#if NOT_USE_PAN_MOTOR==0

#if NOT_USE_TILT_MOTOR==0
	Serial.print(F("_spdY= "));

	motorYcommand = _spd * 5;
	Serial.println(motorYcommand);
#endif	//#if NOT_USE_TILT_MOTOR==0

}
#endif	//#if MOVE_DEBUG

}

////////
// IO //
////////
void ioInit()
{
#if DEBUG
Serial.print(F("io init ..."));
#endif
pinMode(INDICATOR_PIN, OUTPUT);
digitalWrite(INDICATOR_PIN, HIGH);

pinMode(panIO.ENABLE_PIN, OUTPUT);
digitalWrite(panIO.ENABLE_PIN, LOW);
pinMode(panIO.FAULT_RESET_PIN, OUTPUT);
digitalWrite(panIO.FAULT_RESET_PIN, LOW);
pinMode(panIO.HEALTH_PIN, INPUT_PULLUP);

pinMode(tiltIO.ENABLE_PIN, OUTPUT);
digitalWrite(tiltIO.ENABLE_PIN, LOW);
pinMode(tiltIO.FAULT_RESET_PIN, OUTPUT);
digitalWrite(tiltIO.FAULT_RESET_PIN, LOW);
pinMode(tiltIO.HEALTH_PIN, INPUT_PULLUP);

pinMode(TRIGGER_PIN, OUTPUT);
digitalWrite(TRIGGER_PIN, LOW);

#if DEBUG
Serial.println(F("done!"));
#endif
}

void ioHandler()
{
static uint32_t ledTimer = 0;

if (millis() > ledTimer && motorEnable) {
	ledTimer = millis() + 500;

	digitalWrite(INDICATOR_PIN, !digitalRead(INDICATOR_PIN));
}

if (triggerTimer && millis() >= triggerTimer) {
	digitalWrite(TRIGGER_PIN, LOW);
	triggerTimer = 0;
}

//	bool reset = 0;
//
//	if (digitalRead(panIO.HEALTH_PIN) == LOW) {
//		digitalWrite(panIO.FAULT_RESET_PIN, HIGH);
//		reset = 1;
//	}
//
//	if (digitalRead(tiltIO.HEALTH_PIN) == LOW) {
//		digitalWrite(tiltIO.FAULT_RESET_PIN, HIGH);
//		reset = 1;
//	}
//
//	if (reset) {
//		delayWdt(10);
//		digitalWrite(panIO.FAULT_RESET_PIN, LOW);
//		digitalWrite(tiltIO.FAULT_RESET_PIN, LOW);
//	}

}

/**
 * BUS section
 */
void busInit()
{
#if DEBUG
Serial.print(F("bus init... "));
#endif // BUS_DEBUG

SPI.begin();

bus.reset();
bus.setBitrate(CAN_250KBPS);
//	busSetFilter();
bus.setNormalMode();

sendMsg.can_id = BUS_MOTOR_ID;
sendMsg.can_dlc = BUS_MOTOR_SIZE;

//set init command header
recvMsg.data[0] = 0;

#if DEBUG
Serial.println(F("done!"));
#endif
}

void busSetFilter()
{
//RX Buffer 0
bus.setFilterMask(MCP2515::MASK0, 0, 0x3FF);
bus.setFilter(MCP2515::RXF0, 0, 0x310);		// RCWS panel
bus.setFilter(MCP2515::RXF1, 0, 0x000);

//RX Buffer 1
bus.setFilterMask(MCP2515::MASK1, 0, 0x3FF);
bus.setFilter(MCP2515::RXF2, 0, 0x000);
bus.setFilter(MCP2515::RXF3, 0, 0x000);
bus.setFilter(MCP2515::RXF4, 0, 0x000);
bus.setFilter(MCP2515::RXF5, 0, 0x000);
}

void busHandler()
{
busRecv();
busSend();
}

void busRecv()
{
uint32_t _id = 0;
int i = 0;
#if BUS_DEBUG
static byte debugCounter = 0;
#endif	//#if BUS_DEBUG

if (bus.readMessage(&recvMsg) == MCP2515::ERROR_OK) {
	_id = recvMsg.can_id;

	if (_id == BUS_PANEL_ID) {
		//brake update
		if (bitRead(recvMsg.data[0], 0) && bitRead(recvMsg.data[0], 1)) {
			if (brakeFree == 0) {
#if NOT_USE_PAN_MOTOR==0
				motorPan.enableMotor();
#endif	//#if NOT_USE_PAN_MOTOR==0
				delayWdt(10);
#if NOT_USE_TILT_MOTOR==0
				motorTilt.enableMotor();
#endif	//#if NOT_USE_TILT_MOTOR==0

			}
			brakeFree = 1;

			digitalWrite(panIO.ENABLE_PIN, HIGH);
			digitalWrite(tiltIO.ENABLE_PIN, HIGH);
		}
		else {
			if (brakeFree == 1) {
#if NOT_USE_PAN_MOTOR==0
				motorPan.disableMotor();
#endif	//#if NOT_USE_PAN_MOTOR==0
				delayWdt(10);
#if NOT_USE_TILT_MOTOR==0
				motorTilt.disableMotor();
#endif	//#if NOT_USE_TILT_MOTOR==0
			}
			brakeFree = 0;

			digitalWrite(panIO.ENABLE_PIN, LOW);
			digitalWrite(tiltIO.ENABLE_PIN, LOW);
		}

		//trigger update
		triggerHOT = bitRead(recvMsg.data[0], 2);
//			digitalWrite(TRIGGER_PIN, bitRead(recvMsg.data[0], 2));

		//motorXcommand update
		i = (int) recvMsg.data[2] << 8 | recvMsg.data[1];
		if (brakeFree)
			motorXcommand = i;
		else
			motorXcommand = 0;

		//check range
		if (abs(motorXcommand) > MOTOR_MAX_SPEED_VAL) {
			motorXcommand = MOTOR_MAX_SPEED_VAL;
			if (i < 0)
				motorXcommand = 0 - motorXcommand;
		}

		//motorYcommand update
		i = (int) recvMsg.data[4] << 8 | recvMsg.data[3];
		if (brakeFree)
			motorYcommand = i;
		else
			motorYcommand = 0;
		//convert to motor tilt range
		motorYcommand *= 10;

		//check range
		if (abs(motorYcommand) > MOTOR_MAX_SPEED_VAL * 5) {
			motorYcommand = MOTOR_MAX_SPEED_VAL * 5;
			if (i < 0)
				motorYcommand = 0 - motorYcommand;
		}

	}	//(_id == BUS_PANEL_ID)
	else if (_id == BUS_JS_ID) {

		if (bitRead(recvMsg.data[6],0) && bitRead(recvMsg.data[6], 2)) {
			if (triggerHOT) {
				digitalWrite(TRIGGER_PIN, HIGH);
				triggerTimer = millis() + TRIGGER_TIMEOUT;
			}
			else {
				digitalWrite(TRIGGER_PIN, LOW);
				triggerTimer = 0;
			}
		}
		else {
			digitalWrite(TRIGGER_PIN, LOW);
			triggerTimer = 0;
		}
	}	//(_id == BUS_JS_ID)

#if BUS_DEBUG
	debugCounter++;
	if (debugCounter > 20) {
		debugCounter = 0;
		Serial.print(F("trigger(H&S)= "));
		Serial.print(triggerHOT);
		Serial.print(F(" & "));
		if (_id == BUS_JS_ID) {
			Serial.print(bitRead(recvMsg.data[6], 2));
//			Serial.print(recvMsg.data[6], BIN);
		}
		Serial.print('\t');

		Serial.print(F("motorCMD= "));
		Serial.print(motorXcommand);
		Serial.print(F(" : "));
		Serial.println(motorYcommand);
	}
#endif	//#if BUS_DEBUG

}	//(bus.readMessage(&recvMsg) == MCP2515::ERROR_OK)
}

void busSend()
{
//	static uint32_t _busSendTimer = 250;
//
//	if (millis() > _busSendTimer) {
//		_busSendTimer = millis() + 50;
//
//
//	}	//(millis() > _busSendTimer)
}

/**
 * MOTOR section
 */
void motorInit()
{
#if DEBUG
Serial.print(F("motor init... "));
#endif // DEBUG

Serial1.begin(115200);
Serial3.begin(115200);

// Set Modes of operation to OPEN_LOOP_VECTOR
#if NOT_USE_PAN_MOTOR==0
motorPan.setModeOfOperation(Ingenia_SerialServoDrive::DRIVE_MODE_PROFILE_POSITION);
#endif	//#if NOT_USE_PAN_MOTOR==0

delayWdt(10);

#if NOT_USE_TILT_MOTOR==0
motorTilt.setModeOfOperation(Ingenia_SerialServoDrive::DRIVE_MODE_PROFILE_POSITION);
#endif	//#if NOT_USE_TILT_MOTOR

#if DEBUG
Serial.println(F("done!"));
#endif
}

void motorHandler()
{
static uint32_t _start_enable_motor = MOTOR_START_ENABLE_TIMEOUT;
static uint32_t _updateTimer = 0;

//wait to enable motors
if (millis() > _start_enable_motor && !motorEnable) {
	motorEnable = 1;

	//enabling all motor
#if NOT_USE_PAN_MOTOR==0
	motorPan.write(0x607F, 0, 0);
//		motorPan.enableMotor();
#endif	//#if NOT_USE_PAN_MOTOR==0

#if NOT_USE_TILT_MOTOR==0
	motorTilt.write(0x607F, 0, 0);
//		motorTilt.enableMotor();
#endif	//#if NOT_USE_TILT_MOTOR==0

#if DEBUG
	Serial.println(F("motors have been enabled!"));
#endif	//#if DEBUG
}

if (millis() > _updateTimer) {
	_updateTimer = millis() + MOTOR_UPDATE_TIMEOUT;

#if NOT_USE_PAN_MOTOR==0
	//PAN motor handler
	motorPanWrite(motorXcommand);

#endif	//#if NOT_USE_PAN_MOTOR==0

	delayWdt(1);

#if NOT_USE_TILT_MOTOR==0
	//TILT motor handler
	motorTiltWrite(motorYcommand);
#endif	//#if NOT_USE_TILT_MOTOR==0
}	//(millis() > _updateTimer)

}

#if NOT_USE_PAN_MOTOR==0
void motorPanWrite(long speed)
{
static uint32_t _prevSpeed = MOTOR_MAX_SPEED_VAL;
static byte stopCounter = 100;
uint32_t _spd = abs(speed);
int32_t _pos = 0;

if (motorEnable && brakeFree) {
	if (_spd > 0) {
		//set speed
		if (_prevSpeed != _spd)
			motorPan.write(0x607F, 0, _spd);

		if (speed < 0)
			_pos -= MOTOR_POS_REV_VAL;
		else
			_pos += MOTOR_POS_REV_VAL;

		//set position
		motorPan.setTargetPosition(_pos, true, true, false);
		stopCounter = 100;
	}
	else {
		if (_prevSpeed != 0 || stopCounter++ > 20) {
			stopCounter = 0;
			//halt
			motorPan.setTargetPosition(0, true, true, true);
			if (_prevSpeed != 0) {
				motorPan.disableMotor();
				delayWdt(10);
				motorPan.enableMotor();
			}
			//reset demand position
			_pos = motorPan.getActualPosition();
			motorPan.setTargetPosition(_pos);
		}
	}
	_prevSpeed = _spd;
}
}
#endif	//#if NOT_USE_PAN_MOTOR==0

#if NOT_USE_TILT_MOTOR==0
void motorTiltWrite(long speed)
{
static uint32_t _prevSpeed = MOTOR_MAX_SPEED_VAL * 5;
static byte stopCounter = 100;
uint32_t _spd = abs(speed);
int32_t _pos = 0;

if (motorEnable && brakeFree) {
	if (_spd > 0) {
		//set speed
		if (_prevSpeed != _spd)
			motorTilt.write(0x607F, 0, _spd);

		if (speed < 0)
			_pos -= MOTOR_POS_REV_VAL * 5;
		else
			_pos += MOTOR_POS_REV_VAL * 5;

		//set position
		motorTilt.setTargetPosition(_pos, true, true, false);
		stopCounter = 100;
	}
	else {
		if (_prevSpeed != 0 || stopCounter++ > 20) {
			stopCounter = 0;
			//halt
			motorTilt.setTargetPosition(0, true, true, true);
//				if (_prevSpeed != 0) {
//					motorTilt.disableMotor();
//					delayWdt(10);
//					motorPan.enableMotor();
//				}
			//reset demand position
			_pos = motorTilt.getActualPosition();
			motorTilt.setTargetPosition(_pos);
		}
	}
	_prevSpeed = _spd;
}
}
#endif	//#if NOT_USE_TILT_MOTOR==0

#if NOT_USE_PAN_MOTOR==0
void motorPanSetToZero()
{
long pos = 0, pos1 = 0, pos2 = 0;

//get current position
pos = motorPan.getActualPosition();
#if DEBUG
Serial.println();
Serial.print(F("current posX= "));
Serial.println(pos);
#endif	//#if DEBUG

pos1 = MOTOR_POS_REV_VAL * 66;
pos2 = abs(pos) % pos1;
if (pos2 > abs(pos1 / 2)) {
	pos2 = pos1 - pos2;

	if (pos >= 0)
		pos = pos2;
	else
		pos = 0 - pos2;
}
else {
	if (pos >= 0)
		pos = 0 - pos2;
	else
		pos = pos2;
}

#if DEBUG
Serial.print(pos1 / 2);
Serial.print(" ");
Serial.println(pos2);

Serial.print(F("target posX= "));
Serial.println(pos);

#endif	//#if DEBUG

//set speed
motorPan.write(0x607F, 0, MOTOR_MAX_SPEED_VAL);
//set position
motorPan.setTargetPosition(pos, true, true, false);

}
#endif	//#if NOT_USE_PAN_MOTOR==0

#if NOT_USE_TILT_MOTOR==0
void motorTiltSetToZero()
{
//set speed
motorTilt.write(0x607F, 0, MOTOR_MAX_SPEED_VAL);
//set position
motorTilt.setTargetPosition(0);
}
#endif	//#if NOT_USE_TILT_MOTOR==0

/**
 * WDT section
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
