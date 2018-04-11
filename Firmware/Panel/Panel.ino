#include "Arduino.h"

#include <avr/wdt.h>

#define WDT_TIMEOUT           WDTO_1S //timeout of WDT = 250ms
#define WDT_RESET_TIME        100     //time to reset wdt in ms

#include <SPI.h>
#include <mcp2515.h>
#include "Button.h"
#include "Tracker.h"
#include "PID.h"

#define DEBUG					1

#if DEBUG
#define BUTTON_DEBUG			1
#define BUS_DEBUG				1
#if BUS_DEBUG
#define BUS_JS_DEBUG			0
#define BUS_IMU_DEBUG			0
#endif // BUS_DEBUG
#define MOVE_DEBUG				1
#if MOVE_DEBUG
#define MOVE_MAN_DEBUG			1
#define MOVE_TRK_DEBUG			0
#if MOVE_TRK_DEBUG
#define TRACK_PID_DEBUG			1
#define TRACK_PIDX_DEBUG		1
#define TRACK_PIDY_DEBUG		0
#endif // MOVE_TRK_DEBUG
#define MOVE_STAB_DEBUG			0
#endif // MOVE_DEBUG

#endif // DEBUG

//IO definition
#define BUTTON_RXD_PIN			17
#define	MONITOR_EN_PIN			53
#define JS_EN_PIN				8
#define BUS_CS_PIN				9
#define INDICATOR_PIN			13

enum ZOOM_LEVEL
{
	ZOOM_0 = 0,
	ZOOM_1 = 1,
	ZOOM_2 = 2,
	ZOOM_3 = 3
};

enum joystick_zoom
{
	JS_ZOOM_NONE = 0,
	JS_ZOOM_IN = 1,
	JS_ZOOM_OUT = 2
};

struct joystick_frame
{
	bool deadman;
	byte zoom;bool trigger;bool aButton;
	int pan;
	int tilt;
};

//CAN parameter
const uint32_t BUS_JOYSTICK_ID = 0x8CFDD633;
const uint16_t BUS_MOTOR_ID = 0x300;
const uint16_t BUS_BUTTON_ID = 0x320;
const uint16_t BUS_OPT_ID = 0x330;
const uint16_t BUS_IMU_ID = 0x331;

const uint16_t BUS_PANEL_ID = 0x310;
const byte BUS_PANEL_SIZE = 5;
const uint16_t BUS_COMMAND_ID = 0x311;
const byte BUS_COMMAND_SIZE = 4;
MCP2515 bus(BUS_CS_PIN);
struct can_frame recvMsg;
struct can_frame sendMsg;
struct can_frame sendMotorMsg;
uint32_t busSendTimer = 0;
uint32_t busSendMotorTimer = 0;
double imuYPR[3];
uint16_t optLrfValue = 0;

//BUTTON parameter
ButtonClass button(Serial2, 9600);

//MOVEMENT parameter
byte movementState = ButtonClass::MOVE_MANUAL;
byte cameraState = ButtonClass::CAMERA_NONE;
byte optZoomLevel = ZOOM_0;
joystick_frame js;
double pidSetpoint = 0.0;
///TRACKER parameter
TrackerClass tracker(Serial3);
String trackerString = "";
const byte trackerMaxBufsize = 128;
byte trkIdMax = 0;
double trkXinput = 0.0;
double trkYinput = 0.0;
double trkXoutput = 0.0;
double trkYoutput = 0.0;
const double trkXkpid[3][3] = { { 20.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
const double trkYkpid[3][3] = { { 10.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 } };
PID trkXpid(&trkXinput, &trkXoutput, &pidSetpoint, trkXkpid[0][0], trkXkpid[0][1], trkXkpid[0][2],
DIRECT);
PID trkYpid(&trkYinput, &trkYoutput, &pidSetpoint, trkYkpid[0][0], trkYkpid[0][1], trkYkpid[0][2],
DIRECT);
///STAB parameter
int stbPosition[2];
double stbXinput = 0.0;
double stbYinput = 0.0;
double stbXoutput = 0.0;
double stbYoutput = 0.0;
const double stbXkpid[3] = { 20.0, 0.0, 0.0 };
const double stbYkpid[3] = { 10.0, 0.0, 0.0 };
PID stbXpid(&stbXinput, &stbXoutput, &pidSetpoint, stbXkpid[0], stbXkpid[1], stbXkpid[2], DIRECT);
PID stbYpid(&stbYinput, &stbYoutput, &pidSetpoint, stbYkpid[0], stbYkpid[1], stbYkpid[2], DIRECT);

//MOTOR parameter
const int motorXlimit = 29250;
const int motorYlimit = 14625;
//const int motorXlimit = 2925;
//const int motorYlimit = 1462;

int motorXspeed = 0;
int motorYspeed = 0;

void setup()
{
	wdt_enable(WDT_TIMEOUT);

#if DEBUG
	Serial.begin(230400);
	Serial.println(F("Panel - RCWS firmware"));
#endif // DEBUG

	ioInit();
	buttonInit();
	busInit();
	moveInit();
}

void loop()
{
	char c;

	wdt_reset();

#if DEBUG
	if (Serial.available()) {
		c = Serial.read();
		if (c == 't') {

		}
	}
#endif // DEBUG

	buttonHandler();
	busHandler();
	moveHandler();

}

////////
// IO //
////////
void ioInit()
{
	pinMode(INDICATOR_PIN, OUTPUT);
	pinMode(MONITOR_EN_PIN, OUTPUT);
	pinMode(JS_EN_PIN, OUTPUT);

	digitalWrite(INDICATOR_PIN, LOW);
	digitalWrite(MONITOR_EN_PIN, HIGH);
	digitalWrite(JS_EN_PIN, HIGH);
}

////////////
// BUTTON //
////////////
void buttonInit()
{
	pinMode(BUTTON_RXD_PIN, INPUT_PULLUP);
	button.init();
}

void buttonHandler()
{
	byte a;

	if (button.read()) {
		//movement change
		a = button.getMovementMode();
		if (movementState != a) {
			//end previous movementState
			if (movementState == ButtonClass::MOVE_TRACK)
				moveTrackEnd();
			else if (movementState == ButtonClass::MOVE_STAB)
				moveStabEnd();

			//init new movementState
			if (a == ButtonClass::MOVE_TRACK)
				moveTrackInit();
			else if (a == ButtonClass::MOVE_STAB)
				moveStabInit();

			//change movementState
			movementState = a;

			//update sendMsg
			sendMsg.data[2] = movementState;
		}
		//camera switch
		a = button.getCamera();
		if (cameraState != a) {
			cameraState = a;
			sendMsg.data[3] &= ~0b11;
			sendMsg.data[3] = cameraState;
		}
		//lrf power switch
		bitWrite(sendMsg.data[1], 1, button.getLrfPower());
		//trigger enable switch
		bitWrite(sendMotorMsg.data[0], 2, button.getTriggerEnable());
	}
}

/////////
// BUS //
/////////
void busInit()
{
#if BUS_DEBUG
	Serial.print(F("bus init... "));
#endif // BUS_DEBUG

	SPI.begin();

	bus.reset();
	bus.setBitrate(CAN_250KBPS);
	//busSetFilter();
	bus.setNormalMode();

	sendMsg.can_id = BUS_COMMAND_ID;
	sendMsg.can_dlc = BUS_COMMAND_SIZE;
	sendMotorMsg.can_id = BUS_PANEL_ID;
	sendMotorMsg.can_dlc = BUS_PANEL_SIZE;

	busSendTimer = millis() + 500;
	busSendMotorTimer = millis() + 250;
#if BUS_DEBUG
	Serial.println(F("done!"));
#endif // BUS_DEBUG
}

void busSetFilter()
{
	//RX Buffer 0
	bus.setFilterMask(MCP2515::MASK0, 0, 0x3FF);
	bus.setFilter(MCP2515::RXF0, 0, 0x111);		// Gunner main control
	bus.setFilter(MCP2515::RXF1, 0, 0x111);

	//RX Buffer 1
	bus.setFilterMask(MCP2515::MASK1, 0, 0x3FF);
	bus.setFilter(MCP2515::RXF2, 0, 0x220);		// Commander's button
	bus.setFilter(MCP2515::RXF3, 0, 0x201);		// Commander's main control
	bus.setFilter(MCP2515::RXF4, 0, 0x000);
	bus.setFilter(MCP2515::RXF5, 0, 0x000);
}

void busSend()
{
	if (millis() > busSendMotorTimer) {
		busSendMotorTimer = millis() + 50;

		if (js.deadman)
			sendMotorMsg.data[0] |= 0b11;
		else
			sendMotorMsg.data[0] &= ~0b11;
//		if (js.trigger)
//			sendMotorMsg.data[0] |= 0b100;

		sendMotorMsg.data[1] = byte(motorXspeed & 0xFF);
		sendMotorMsg.data[2] = byte(motorXspeed >> 8 & 0xFF);
		sendMotorMsg.data[3] = byte(motorYspeed & 0xFF);
		sendMotorMsg.data[4] = byte(motorYspeed >> 8 & 0xFF);

		bus.sendMessage(&sendMotorMsg);
	}

	if (millis() > busSendTimer) {
		busSendTimer = millis() + 200;

	}
}

void busRecv()
{
	byte a, b;
	int i;
	uint16_t _lrf = 0;
	uint32_t _id;

#if BUS_JS_DEBUG
	static uint32_t jsDispTimer = 0;
#endif // BUS_JS_DEBUG

	if (bus.readMessage(&recvMsg) == MCP2515::ERROR_OK) {
		_id = recvMsg.can_id;

		if (_id == BUS_JOYSTICK_ID) {
			js.deadman = bitRead(recvMsg.data[6], 0);
			js.trigger = bitRead(recvMsg.data[6], 2);
			js.aButton = bitRead(recvMsg.data[5], 2);
			if (bitRead(recvMsg.data[5], 4))
				js.zoom = JS_ZOOM_IN;
			else if (bitRead(recvMsg.data[5], 6))
				js.zoom = JS_ZOOM_OUT;
			else
				js.zoom = JS_ZOOM_NONE;
			if (js.deadman) {
				//joystick's pan command
				a = recvMsg.data[0] & 0xF;
				i = recvMsg.data[1];
				if (a == 0)
					js.pan = i;
				else if (a == 4)
					js.pan = 0 - i;
				else
					js.pan = 0;

				//joystick's tilt command
				a = recvMsg.data[2] & 0xF;
				i = recvMsg.data[3];
				if (a == 0)
					js.tilt = 0 - i;
				else if (a == 4)
					js.tilt = i;
				else
					js.tilt = 0;
//				if (a == 0)
//					js.tilt = i;
//				else if (a == 4)
//					js.tilt = 0 - i;
//				else
//					js.tilt = 0;
			}
			else {
				js.pan = 0;
				js.tilt = 0;
			}

#if BUS_JS_DEBUG
			if (millis() > jsDispTimer) {
				jsDispTimer = millis() + 100;

				Serial.print(F("JS (d t a z pan tilt): "));
				Serial.print(js.deadman); Serial.print(' ');
				Serial.print(js.trigger); Serial.print(' ');
				Serial.print(js.aButton); Serial.print(' ');
				Serial.print(js.zoom); Serial.print(' ');
				Serial.print(js.pan); Serial.print(' ');
				Serial.print(js.tilt); Serial.print(' ');
				Serial.println();
				Serial.println();
			}
#endif // BUS_JS_DEBUG
		}	//if (_id == BUS_JOYSTICK_ID)
		else if (_id == BUS_OPT_ID) {
			a = recvMsg.data[0];

			//cam selection:
			b = a & 0b11;
			if (cameraState != b) {
				if (b == ButtonClass::CAMERA_NONE && movementState != ButtonClass::MOVE_TRACK)
					trackerClear();
			}
			cameraState = b;

			//zoom state
			optZoomLevel = (a >> 2) & 0b11;
			if (optZoomLevel > 2)
				optZoomLevel = 2;

			_lrf = recvMsg.data[1] & 0x3F;
			_lrf = _lrf << 8;
			_lrf |= recvMsg.data[2];

			if (_lrf != optLrfValue) {
				optLrfValue = _lrf;
#if DEBUG
				Serial.println(F("==========="));
				Serial.print(F("new LRF val= "));
				Serial.println(optLrfValue);
				Serial.println(F("==========="));
#endif // DEBUG
				tracker.setLrfValue(optLrfValue);
			}
		}	//if (_id == BUS_OPT_ID)
		else if (_id == BUS_IMU_ID) {
			imuYPR[0] = int(recvMsg.data[1]) << 8 | recvMsg.data[0];
			imuYPR[1] = int(recvMsg.data[3]) << 8 | recvMsg.data[2];
			imuYPR[2] = int(recvMsg.data[5]) << 8 | recvMsg.data[4];
#if BUS_IMU_DEBUG
			if (imuCountDebug++ > 10) {
				imuCountDebug = 0;

				Serial.print(imuYPR[0]); Serial.print(' ');
				Serial.print(imuYPR[1]); Serial.print(' ');
				Serial.print(imuYPR[2]);
				Serial.println();
			}
#endif // BUS_IMU_DEBUG
		}

	}
}

void busHandler()
{
	busRecv();
	busSend();
}

/////////////
// TRACKER //
/////////////
void trackerInit()
{
	tracker.init();
	trackerString.reserve(trackerMaxBufsize);
}

void trackerHandler()
{
	char c;
	bool strCompleted = 0;
	static uint32_t imuUpdateTimer = 0;
	static uint32_t trackerValueTimer = 0;
	byte awal, akhir;
	String tem;
	int i;

	if (millis() > trackerValueTimer) {
		trkXinput = 0.0;
		trkYinput = 0.0;
	}

	if (tracker.available()) {
		c = tracker.read();

		if (c == '*')
			strCompleted = 1;
		else if (c == '$')
			trackerString = "";

		trackerString += c;
	}

	if (strCompleted) {
		trackerString.trim();

		trackerValueTimer = millis() + 1000;

		//$TRKUD,x,Xinput,Yinput*
		if (trackerString.indexOf("$TRKUD,") >= 0) {
			//#if TRACK_DEBUG && TRACK_PID_DEBUG==0
			//				Serial.println(trackerString);
			//#endif // TRACK_DEBUG

			//TRKID
			awal = 7;
			akhir = trackerString.indexOf(',', awal);
			tem = trackerString.substring(awal, akhir);
			i = tem.toInt();
			if (i > trkIdMax)
				trkIdMax = i;

			//PAN val
			awal = akhir + 1;
			akhir = trackerString.indexOf(',', awal);
			tem = trackerString.substring(awal, akhir);
			trkXinput = 0 - (double) tem.toInt();

			//TILT val
			awal = akhir + 1;
			akhir = trackerString.indexOf(',', awal);
			tem = trackerString.substring(awal, akhir);
			trkYinput = (double) tem.toInt();
//			#if TRACK_DEBUG && TRACK_PID_DEBUG==0
//							Serial.print(i); Serial.print(' ');
//							Serial.print(trkXinput); Serial.print(' ');
//							Serial.print(trkYinput); Serial.print(' ');
//							Serial.println();
//			#endif // TRACK_DEBUG
		}

		trackerString = "";
	}

	if (millis() > imuUpdateTimer) {
		imuUpdateTimer = millis() + 500;

		tracker.setImuVal((double) imuYPR[0] / 100, (double) imuYPR[1] / 100);
	}
}

void trackerClear()
{
#if TRACK_DEBUG && TRACK_PID_DEBUG==0
	Serial.print(F("trkIdMax= "));
	Serial.println(trkIdMax);
#endif // TRACK_DEBUG
	for ( byte i = 0; i <= trkIdMax; i++ )
		tracker.clearTrackId(0);

	trkIdMax = 0;
}

//////////
// MOVE //
//////////
void moveInit()
{
	moveManInit();

	trkXpid.SetSampleTime(50);
	trkXpid.SetOutputLimits((0 - motorXlimit), motorXlimit);
	trkXpid.SetMode(MANUAL);
	trkXpid.SetInputSensitivity(0, 0, 1);

	trkYpid.SetSampleTime(50);
	trkYpid.SetOutputLimits((0 - motorYlimit), motorYlimit);
	trkYpid.SetMode(MANUAL);
	trkXpid.SetInputSensitivity(0, 0, 1);

	stbXpid.SetSampleTime(50);
	stbXpid.SetOutputLimits((0 - motorXlimit), motorXlimit);
	stbXpid.SetMode(MANUAL);

	stbYpid.SetSampleTime(50);
	stbYpid.SetOutputLimits((0 - motorYlimit), motorYlimit);
	stbYpid.SetMode(MANUAL);
}

void moveHandler()
{
	if (movementState == ButtonClass::MOVE_MANUAL)
		moveManHandler();
	else if (movementState == ButtonClass::MOVE_TRACK)
		moveTrackHandler();
	else if (movementState == ButtonClass::MOVE_STAB)
		moveStabHandler();
}

void moveManInit()
{

}

void moveManHandler()
{
	moveManConversion(js.pan, js.tilt);

	//update sendMotorMsg
	bitWrite(sendMotorMsg.data[0], 0, js.deadman);
	bitWrite(sendMotorMsg.data[0], 1, js.deadman);
	sendMotorMsg.data[1] = motorXspeed & 0xFF;
	sendMotorMsg.data[2] = byte(motorXspeed >> 8);
	sendMotorMsg.data[3] = motorYspeed & 0xFF;
	sendMotorMsg.data[4] = byte(motorYspeed >> 8);

#if MOVE_MAN_DEBUG
	static uint32_t timer = 0;

	if (millis() > timer) {
		timer = millis() + 100;

		Serial.print(F("BTN: "));
		Serial.print(button.getTriggerEnable());
		Serial.print(bitRead(sendMotorMsg.data[0],2));
		Serial.print(' ');
		Serial.print(F("JS: "));
		Serial.print(js.pan);
		Serial.print(' ');
		Serial.println(js.tilt);
		Serial.print(F("mtr: "));
		Serial.print(motorXspeed);
		Serial.print(' ');
		Serial.println(motorYspeed);
		Serial.println();
	}
#endif // MOVE_MAN_DEBUG
}

void moveManConversion(int x, int y)
{
	const uint16_t jsXMid = 100;
	const uint16_t jsYMid = 100;
	const uint16_t jsXMax = 0xFA;
	const uint16_t jsYMax = 0xFA;

	const uint32_t speedXmid[3] = { 4000, 2000, 1000 };
	const uint32_t speedXmax[3] = { motorXlimit, 8000, 4000 };
	const uint32_t speedYmid[3] = { 2000, 1000, 500 };
	const uint32_t speedYmax[3] = { motorYlimit, 4000, 2000 };
	uint32_t speedXMid = speedXmid[optZoomLevel];
	uint32_t speedXMax = speedXmax[optZoomLevel];
	uint32_t speedYMid = speedYmid[optZoomLevel];
	uint32_t speedYMax = speedYmax[optZoomLevel];

	if (abs(x) < jsXMid)
		motorXspeed = constrain(map(abs(x), 0, jsXMid, 0, speedXMid), 0, speedXMid);
	else
		motorXspeed = constrain(map(abs(x), jsXMid, jsXMax, speedXMid, speedXMax), speedXMid,
				speedXMax);
	if (x < 0)
		motorXspeed = 0 - motorXspeed;

	if (abs(y) < jsYMid)
		motorYspeed = constrain(map(abs(y), 0, jsYMid, 0, speedYMid), 0, speedYMid);
	else
		motorYspeed = constrain(map(abs(y), jsYMid, jsYMax, speedYMid, speedYMax), speedYMid,
				speedYMax);
	if (y < 0)
		motorYspeed = 0 - motorYspeed;

}

void moveTrackInit()
{

	byte pidPointer = 0;

	pidPointer = optZoomLevel;
	if (pidPointer > 2)
		pidPointer = 2;

#if MOVE_TRK_DEBUG
	byte i=0;

	Serial.println(F("kpid XY:"));
	for (i = 0; i < 3; i++) {
		Serial.print(trkXkpid[pidPointer][i]);
		Serial.print(' ');
	}
	Serial.println();
	for (i = 0; i < 3; i++) {
		Serial.print(trkYkpid[pidPointer][i]);
		Serial.print(' ');
	}
	Serial.println();
#if TRACK_PIDX_DEBUG || TRACK_PIDY_DEBUG
	startPidTimer = millis();
#endif // TRACK_PIDX_DEBUG || TRACK_PIDY_DEBUG
#endif // MOVE_TRK_DEBUG

	trkXpid.SetTunings(trkXkpid[pidPointer][0], trkXkpid[pidPointer][1], trkXkpid[pidPointer][2]);
	trkYpid.SetTunings(trkYkpid[pidPointer][0], trkYkpid[pidPointer][1], trkYkpid[pidPointer][2]);

	trkXoutput = (double) motorXspeed;
	trkYoutput = (double) motorYspeed;

	trkXpid.SetMode(AUTOMATIC);
	trkYpid.SetMode(AUTOMATIC);
}

void moveTrackEnd()
{
	trkXpid.SetMode(MANUAL);
	trkYpid.SetMode(MANUAL);
}

void moveTrackHandler()
{
#if MOVE_TRK_DEBUG
	static uint32_t dispTimer = 0;
#endif // MOVE_TRK_DEBUG

	if (trkXpid.Compute()) {
		motorXspeed = (int) trkXoutput;

#if TRACK_PIDX_DEBUG
		if (millis() > dispTimer) {
			dispTimer = millis() + 100;

			Serial.print((millis() - startPidTimer));
			Serial.print(',');
			Serial.print(trkXinput); Serial.print(',');
			Serial.print(motorXspeed);
			Serial.println();
		}
#endif // TRACK_PIDX_DEBUG
	}

	if (trkYpid.Compute()) {
		motorYspeed = (int) trkYoutput;

#if TRACK_PIDY_DEBUG
		if (millis() > dispTimer) {
			dispTimer = millis() + 100;

			Serial.print((millis() - startPidTimer));
			Serial.print(',');
			Serial.print(trkYinput); Serial.print(',');
			Serial.print(motorYspeed);
			Serial.println();
		}
#endif // TRACK_PIDY_DEBUG
	}

#if MOVE_TRK_DEBUG
#if TRACK_PID_DEBUG==0
	if (millis()>dispTimer) {
		dispTimer = millis() + 500;

		Serial.println();
		Serial.print(F("dX:dY= "));
		Serial.print(trkXinput);
		Serial.print(' ');
		Serial.println(trkYinput);
		Serial.print(F("XYspeed= "));
		Serial.print(motorXspeed);
		Serial.print(' ');
		Serial.println(motorYspeed);
		Serial.println();
	}
#endif // TRACK_PID_DEBUG==0
#endif // MOVE_TRK_DEBUG

}

void moveStabInit()
{
	if (!(imuYPR[0] == 0 && imuYPR[1] == 0)) {
		stbPosition[0] = imuYPR[0];
		stbPosition[1] = imuYPR[1];

#if MOVE_STAB_DEBUG
		Serial.print(F("Az:El= "));
		Serial.print(stbPosition[0]); Serial.print(' ');
		Serial.println(stbPosition[1]);
#endif // MOVE_STAB_DEBUG

		stbXpid.SetTunings(stbXkpid[0], stbXkpid[1], stbXkpid[2]);
		stbYpid.SetTunings(stbYkpid[0], stbYkpid[1], stbYkpid[2]);

		stbXoutput = (double) motorXspeed;
		stbYoutput = (double) motorYspeed;

		stbXpid.SetMode(AUTOMATIC);
		stbYpid.SetMode(AUTOMATIC);
	}

}

void moveStabEnd()
{
	stbXpid.SetMode(MANUAL);
	stbYpid.SetMode(MANUAL);
}

void moveStabHandler()
{

#if STAB_DEBUG
	static byte dispTimer = 0;
#endif // STAB_DEBUG

	moveStabInput();

	if (stbXpid.Compute())
		motorXspeed = (int) stbXoutput;

	if (stbYpid.Compute())
		motorYspeed = (int) stbYoutput;

#if STAB_DEBUG
#if STAB_PIDX_DEBUG || STAB_PIDY_DEBUG
	if (millis() > dispTimer) {
		dispTimer = millis() + 100;

#if STAB_PIDX_DEBUG
		Serial.print(stbXinput); Serial.print(';');
		Serial.print(motorXspeed); Serial.print(';');
#endif // STAB_PIDX_DEBUG
#if STAB_PIDY_DEBUG
		Serial.print(stbYinput); Serial.print(';');
		Serial.print(motorYspeed); Serial.print(';');
#endif // STAB_PIDY_DEBUG
		Serial.println();
	}
#else
	if (millis()>dispTimer) {
		dispTimer = millis() + 500;
		Serial.println();
		Serial.print(F("dX:dY= "));
		Serial.print(stbXinput);
		Serial.print(' ');
		Serial.println(stbYinput);
		Serial.print(F("XYspeed= "));
		Serial.print(motorXspeed);
		Serial.print(' ');
		Serial.println(motorYspeed);
		Serial.println();
	}
#endif // STAB_PIDX_DEBUG
#endif // STAB_DEBUG
}

void moveStabInput()
{
	long i = 0;

	i = (long) imuYPR[0] - (long) stbPosition[0];
	if (i > 18000)
		i -= 36000;
	else if (i < -18000)
		i += 36000;
	stbXinput = (double) i / 100.0;

	i = imuYPR[1] - stbPosition[1];
	stbYinput = (double) i / 100.0;
}

/////////
// WDT //
/////////
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
