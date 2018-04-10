#include "Arduino.h"
#include <avr/wdt.h>
#include <SPI.h>
#include <mcp2515.h>
#include "libVisca.h"
//#include "vn100.h"

#define DEBUG					1
#if DEBUG

#define	BUS_DEBUG				0
#define CAM_DEBUG				0
#define LRF_DEBUG				0
#define IMU_DEBUG				0

#endif // DEBUG
#define USE_BOARD				"optronik-20171017"

#define WDT_TIMEOUT           WDTO_1S //timeout of WDT = 250ms
#define WDT_RESET_TIME        100     //time to reset wdt in ms

const byte CAM_ENABLE_PIN = 26;
const byte CAM_SELECT_PIN = 27;
const byte COMM_SONY_SEL_PIN = 24;
const byte COMM_THERMAL_SEL_PIN = 25;
const byte THERMAL_PWR_EN_PIN = 28;

#define THERMAL_PWR_ON    HIGH
#define THERMAL_PWR_OFF   LOW
#define CAM_SELECT_NONE       0
#define CAM_SELECT_SONY       1
#define CAM_SELECT_THERMAL    2
#define CAM_SELECT_DEFAULT    0xFF

const byte ulir_connect[6] = { 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF };
const byte ulir_zoom_1x[7] = { 0xF0, 0x01, 0x03, 0x02, 0x00, 0x05, 0xFF };
const byte ulir_zoom_2x[7] = { 0xF0, 0x01, 0x03, 0x02, 0x01, 0x06, 0xFF };
const byte ulir_zoom_3x[7] = { 0xF0, 0x01, 0x03, 0x02, 0x02, 0x07, 0xFF };
const byte ulir_zoom_4x[7] = { 0xF0, 0x01, 0x03, 0x02, 0x03, 0x08, 0xFF };

enum sonyFocus
{
	SONY_FOCUS_STOP,
	SONY_FOCUS_FAR,
	SONY_FOCUS_NEAR
};

byte camState = CAM_SELECT_DEFAULT;
byte zoomLevel = 0;

//LRF parameter
#define LRF_POWER_PIN			29
#define LRF_POWER_ON			HIGH
#define LRF_POWER_OFF			LOW
#define LRF_ENABLE_PIN			3

enum LrfStateValue
{
	LRF_DISABLE_state,
	LRF_ENABLE_state,
	LRF_READY_state,
	LRF_MEASURING_state,
	LRF_END_state
};

String lrfString;
byte lrfBufSize = 64;
byte lrfState = LRF_DISABLE_state;
uint16_t lrfValue = 0;
uint32_t lrfTimerEnable = 0;
uint32_t lrfTimerReady = 0;
uint32_t lrfTimerMeasuring = 0;
uint32_t lrfTimerEnd = 0;

//CAN parameter
const byte BUS_CS_PIN = 10;

const uint16_t BUS_MAIN_CMD1_ID = 0x310;
const uint16_t BUS_MAIN_CMD2_ID = 0x311;
const uint16_t BUS_BUTTON_ID = 0x320;
const uint16_t BUS_OPT_ID = 0x330;
const uint16_t BUS_IMU_ID = 0x331;
const byte BUS_OPT_SIZE = 3;
const byte BUS_IMU_SIZE = 6;
MCP2515 bus(BUS_CS_PIN);
struct can_frame sendOptMsg;
struct can_frame sendImuMsg;
struct can_frame recvMsg;

//SONY
LibVisca sony(Serial2);
VISCAInterface_t sonyIface;
VISCACamera_t sonyCamera;

// IMU
//VN100 imu(Serial3, 115200);

enum IMU_UPDATE_TIMEOUT
{
	imuStabTimeout = 50,
	imuNormalTimeout = 500
};
uint32_t imuSendTimer = imuNormalTimeout;

void setup()
{

#if DEBUG
	String s;
	int loc;

	Serial.begin(250000);
	Serial.println();
	Serial.println(F("optronik - FCS_GUNNER firmware"));
	s = __FILE__;
	loc = s.lastIndexOf('\\');
	s = s.substring(loc + 1);
	loc = s.indexOf('.');
	s = s.substring(0, loc);
	Serial.print(F("s/w: "));
	Serial.println(s);
	Serial.print(F("board: "));
	Serial.println(USE_BOARD);
#endif // DEBUG

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	imuInit();
	wdt_enable(WDT_TIMEOUT);

	camInit();
	busInit();
	lrfInit();
}

void loop()
{
	wdt_reset();

	busHandler();
	lrfHandler();
	imuHandler();
}

//----------- CAMERA -----------//
void camInit()
{
#if CAM_DEBUG
	char buf[128];
#endif	//#if CAM_DEBUG

#if CAM_DEBUG
	Serial.print(F("camera init ..."));
#endif // CAM_DEBUG

	//cam's video
	pinMode(CAM_ENABLE_PIN, OUTPUT);
	pinMode(CAM_SELECT_PIN, OUTPUT);

	//cam's comm
	pinMode(COMM_SONY_SEL_PIN, OUTPUT);
	pinMode(COMM_THERMAL_SEL_PIN, OUTPUT);

	camSelect(CAM_SELECT_NONE);

	sony.open_serial();
	sonyIface.broadcast = 0;
	sonyCamera.address = 1;
	//sony.clear(&sonyIface, &sonyCamera);
#if CAM_DEBUG
	Serial.println();
	if (sony.get_camera_info(&sonyIface, &sonyCamera) == VISCA_SUCCESS) {
		sprintf(buf, "vendor: 0x%04x\n model: 0x%04x\n ROM version: 0x%04x\n socket number: 0x%02x",
				sonyCamera.vendor, sonyCamera.model, sonyCamera.rom_version, sonyCamera.socket_num);
		Serial.println(buf);
	}
	else {
		Serial.println(F("error get camera info"));
	}
#endif // CAM_DEBUG

	camThermalPower(THERMAL_PWR_OFF);

#if CAM_DEBUG
	Serial.println(F("done!"));
#endif // CAM_DEBUG
}

bool camSelect(byte _cam)
{
	if (camState != _cam) {
		switch (_cam)
		{
		case CAM_SELECT_NONE:
#if CAM_DEBUG
			Serial.println(F("None CAM selected"));
#endif // CAM_DEBUG
			digitalWrite(CAM_ENABLE_PIN, HIGH);
			digitalWrite(CAM_SELECT_PIN, LOW);

			digitalWrite(COMM_SONY_SEL_PIN, LOW);
			digitalWrite(COMM_THERMAL_SEL_PIN, LOW);

			camThermalPower(THERMAL_PWR_OFF);

			bitWrite(sendOptMsg.data[0], 0, 0);
			bitWrite(sendOptMsg.data[0], 1, 0);
			break;
		case CAM_SELECT_SONY:
#if CAM_DEBUG
			Serial.println(F("CAM: Sony selected"));
#endif // CAM_DEBUG
			digitalWrite(CAM_ENABLE_PIN, LOW);
			digitalWrite(CAM_SELECT_PIN, LOW);

			digitalWrite(COMM_SONY_SEL_PIN, HIGH);
			digitalWrite(COMM_THERMAL_SEL_PIN, LOW);

			camThermalPower(THERMAL_PWR_OFF);
			delayWdt(10);
			camRxEmpty();

			bitWrite(sendOptMsg.data[0], 0, 1);
			bitWrite(sendOptMsg.data[0], 1, 0);
			break;
		case CAM_SELECT_THERMAL:
#if CAM_DEBUG
			Serial.println(F("CAM: Thermal selected"));
#endif // CAM_DEBUG
			digitalWrite(CAM_ENABLE_PIN, LOW);
			digitalWrite(CAM_SELECT_PIN, HIGH);

			digitalWrite(COMM_SONY_SEL_PIN, LOW);
			digitalWrite(COMM_THERMAL_SEL_PIN, HIGH);

			camThermalPower(THERMAL_PWR_ON);
			delayWdt(10);
			camRxEmpty();

			bitWrite(sendOptMsg.data[0], 0, 0);
			bitWrite(sendOptMsg.data[0], 1, 1);
			break;
		}
		camState = _cam;
		camZoom(zoomLevel = 1);
		return true;
	}
	else
		return false;
}

void camRxEmpty()
{
	while (Serial2.available())
		Serial2.read();
}

void camZoomAdd()
{
	zoomLevel++;
	if (zoomLevel > 4)
		zoomLevel = 4;
	else
		camZoom(zoomLevel);
}

void camZoomSubtract()
{
	zoomLevel--;
	if (zoomLevel < 1)
		zoomLevel = 1;
	else
		camZoom(zoomLevel);
}

void camZoom(byte zoom)
{
	static byte _prevZoom = 0;

	if (camState == CAM_SELECT_NONE)
		_prevZoom = 0;
	else {
		if (_prevZoom != zoom) {
			if (camState == CAM_SELECT_SONY)
				camSonyZoom(zoom);
			else if (camState == CAM_SELECT_THERMAL)
				camThermalZoom(zoom);

			_prevZoom = zoom;
			if (zoom == 1) {
				bitWrite(sendOptMsg.data[0], 2, 0);
				bitWrite(sendOptMsg.data[0], 3, 0);
			}
			else if (zoom == 2) {
				bitWrite(sendOptMsg.data[0], 2, 1);
				bitWrite(sendOptMsg.data[0], 3, 0);
			}
			else if (zoom == 3) {
				bitWrite(sendOptMsg.data[0], 2, 0);
				bitWrite(sendOptMsg.data[0], 3, 1);
			}
			else if (zoom == 4) {
				bitWrite(sendOptMsg.data[0], 2, 1);
				bitWrite(sendOptMsg.data[0], 3, 1);
			}
		}
	}
}

void camSonyZoom(byte zoom)
{
	if (zoom >= 1 && zoom <= 4)
		sony.set_focus_auto(&sonyIface, &sonyCamera, VISCA_FOCUS_AUTO_ON);

	switch (zoom)
	{
	case 1:
#if CAM_DEBUG
		Serial.println(F("SONY: zoom 1"));
#endif // CAM_DEBUG
		sony.set_zoom_value(&sonyIface, &sonyCamera, 0x2000);
		break;
	case 2:
#if CAM_DEBUG
		Serial.println(F("SONY: zoom 2"));
#endif // CAM_DEBUG
		sony.set_zoom_value(&sonyIface, &sonyCamera, 0x4000);
		break;
	case 3:
#if CAM_DEBUG
		Serial.println(F("SONY: zoom 3"));
#endif // CAM_DEBUG
		sony.set_zoom_value(&sonyIface, &sonyCamera, 0x6000);
		break;
	case 4:
#if CAM_DEBUG
		Serial.println(F("SONY: zoom 4"));
#endif // CAM_DEBUG
		sony.set_zoom_value(&sonyIface, &sonyCamera, 0x7AC0);
		//sony.set_zoom_value(&sonyIface, &sonyCamera, 0x7000);
		break;
	}
}

void camThermalPower(bool onf)
{
	digitalWrite(THERMAL_PWR_EN_PIN, onf);
#if CAM_DEBUG
	Serial.print(F("Thermal Power: "));
	Serial.println(digitalRead(THERMAL_PWR_EN_PIN));
#endif // CAM_DEBUG
}

void camThermalZoom(byte zoom)
{
	switch (zoom)
	{
	case 1:
#if CAM_DEBUG
		Serial.println(F("ULIR: zoom 1"));
#endif // CAM_DEBUG
		camThermalCmd(1);
		break;
	case 2:
#if CAM_DEBUG
		Serial.println(F("ULIR: zoom 2"));
#endif // CAM_DEBUG
		camThermalCmd(2);
		break;
	case 3:
#if CAM_DEBUG
		Serial.println(F("ULIR: zoom 3"));
#endif // CAM_DEBUG
		camThermalCmd(3);
		break;
	case 4:
#if CAM_DEBUG
		Serial.println(F("ULIR: zoom 4"));
#endif // CAM_DEBUG
		camThermalCmd(4);
		break;
	}
}

void camThermalCmd(byte zoom)
{
	byte buf[7], i = 0;

	switch (zoom)
	{
	case 1:
		memcpy(buf, ulir_zoom_1x, 7);
		break;
	case 2:
		memcpy(buf, ulir_zoom_2x, 7);
		break;
	case 3:
		memcpy(buf, ulir_zoom_3x, 7);
		break;
	case 4:
		memcpy(buf, ulir_zoom_4x, 7);
		break;
	}

	for ( i = 0; i < 7; i++ ) {
		Serial2.write(buf[i]);
	}
	Serial2.flush();
}

//----------- LRF -----------//
void lrfInit()
{
	Serial1.begin(19200);
	// LRF's PSU
	pinMode(LRF_POWER_PIN, OUTPUT);
	digitalWrite(LRF_POWER_PIN, LRF_POWER_OFF);
	// LRF's enable
	pinMode(LRF_ENABLE_PIN, OUTPUT);
	digitalWrite(LRF_ENABLE_PIN, LRF_POWER_OFF);

	lrfString.reserve(lrfBufSize);
	lrfString = "";

	lrfTimerReady = 0;
	lrfTimerMeasuring = 0;
	lrfTimerEnd = 0;
	lrfState = LRF_DISABLE_state;
}

void lrfHandler()
{
	bool lrfCompleted = 0;
	char c;
	String s;

#if LRF_DEBUG
	static uint32_t lrfDebugTimer = millis() + 500;

	if (millis() > lrfDebugTimer) {
		lrfDebugTimer = millis() + 200;
		Serial.print(F("lrf state: "));
		Serial.print(lrfState);
		Serial.print(F("\tlrfValue= "));
		Serial.println(lrfValue);

	}
#endif // LRF_DEBUG

	lrfStateTimeoutHandler();

	if (Serial1.available()) {
		c = Serial1.read();
		//Serial.write(c);
		lrfString += c;
		if (c == 0x1B)
			lrfString = "";
		else if (c == 0x0A)
			lrfCompleted = 1;
	}

	if (lrfCompleted) {
		lrfString.trim();
#if LRF_DEBUG
		Serial.println(lrfString);
#endif // LRF_DEBUG
		if (lrfString.indexOf("Jenoptik DLEM4k") >= 0) {
#if LRF_DEBUG
			s = "MW 1 5000";
#else
			s = "MW 1 5000";
#endif // LRF_DEBUG
			lrfSend(s);
			lrfTimerEnable = 0;
			lrfTimerReady = millis() + 3000;
			lrfState = LRF_READY_state;
			bitSet(sendOptMsg.data[1], 7);
		}
		else if (lrfString.indexOf("MW ") >= 0) {
			s = "DM 3 1 0 0";
			lrfSend(s);
			lrfTimerReady = 0;
			lrfTimerMeasuring = millis() + 10000;
			lrfState = LRF_MEASURING_state;
		}
		else if (lrfString.indexOf("DM ") >= 0) {
			//parse lrfString
			lrfValue = lrfFindValue(lrfString);
			sendOptMsg.data[2] = byte(lrfValue & 0xFF);
			sendOptMsg.data[1] &= 0xC0;
			sendOptMsg.data[1] |= byte((lrfValue >> 8) & 0x3F);
#if DEBUG
			Serial.println();
			Serial.print(F("LRF new value= "));
			Serial.print(lrfValue);
			Serial.println('m');
			Serial.println();
#endif // DEBUG
			lrfTimerMeasuring = 0;
			lrfTimerEnd = millis() + 20000;
			lrfState = LRF_END_state;
		}
		lrfString = "";
	}
}

void lrfStateTimeoutHandler()
{
	switch (lrfState)
	{
	case LRF_DISABLE_state:
		break;
	case LRF_ENABLE_state:
		if (lrfTimerEnable && millis() > lrfTimerEnable) {
			lrfTimerEnable = 0;
			lrfStateToDisable();
		}
		break;
	case LRF_READY_state:
		if (lrfTimerReady && millis() > lrfTimerReady) {
			lrfTimerReady = 0;
			lrfStateToDisable();
		}
		break;
	case LRF_MEASURING_state:
		if (lrfTimerMeasuring && millis() > lrfTimerMeasuring) {
			lrfTimerMeasuring = 0;
			lrfStateToDisable();
		}
		break;
	case LRF_END_state:
		if (lrfTimerEnd && millis() > lrfTimerEnd) {
			lrfTimerEnd = 0;
			lrfStateToDisable();
		}
		break;
	}
}

void lrfStateToDisable()
{
	lrfState = LRF_DISABLE_state;
	digitalWrite(LRF_ENABLE_PIN, LRF_POWER_OFF);
	delayWdt(10);
	digitalWrite(LRF_POWER_PIN, LRF_POWER_OFF);
	lrfValue = 0;
	sendOptMsg.data[1] = 0;
	sendOptMsg.data[2] = 0;
#if LRF_DEBUG
	Serial.println(F("lrf state: LRF_DISABLE_state"));
#endif // LRF_DEBUG
	lrfString = "";
}

uint16_t lrfFindValue(String s)
{
	const byte dataCount = 10;
	uint16_t ret = 0;
	uint16_t val[dataCount], strength = 0, strPoint = 1;
	float f;
	int i, awal, akhir;
	char c;
	String tem;

	for ( i = 0; i < dataCount; i++ )
		val[i] = 0;
	awal = s.indexOf("DM ") + 5;
	for ( i = 0; i < dataCount; i++ ) {
		akhir = s.indexOf(' ', awal);
		if (akhir > awal) {
			tem = s.substring(awal, akhir);
			f = tem.toFloat();
			if (((uint16_t) (f * 10.0f) % 10) >= 5)
				f = f + 1.0;
			val[i] = (uint16_t) f;
			awal = akhir + 1;
			c = s.charAt(awal);
			if (c == 0x03 || c == ' ')
				break;
		}
		else
			break;
	}

	//find strongest signal
	for ( i = 0; i < dataCount / 2; i++ ) {
		if (val[(2 * i) + 1] > strength) {
			strPoint = (2 * i) + 1;
			strength = val[strPoint];
		}
	}

	ret = val[strPoint - 1];

	return ret;
}

void lrfSend(String s)
{
	while (Serial1.available())
		Serial1.read();
#if LRF_DEBUG
	Serial.print(F("send \""));
	Serial.print(s);
	Serial.println(F("\" to LRF"));
#endif // LRF_DEBUG
	Serial1.write(0x1B);	// Esc
	Serial1.print(s);
	Serial1.write(' ');
	Serial1.write(0x0D);	// Cr
}

//----------- IMU -----------//
void imuInit()
{
}

void imuHandler()
{
//	byte i;
//	long _ypr[3];
//#if IMU_DEBUG
//	static uint16_t _imuCount = 0;
//#endif // IMU_DEBUG
//
//	if (imu.read()) {
//		imu.getYpr(_ypr);
//		for ( i = 0; i < 3; i++ )
//			_ypr[i] /= 10;
//
//		sendImuMsg.data[0] = _ypr[0] & 0xFF;
//		sendImuMsg.data[1] = (_ypr[0] >> 8) & 0xFF;
//		sendImuMsg.data[2] = _ypr[1] & 0xFF;
//		sendImuMsg.data[3] = (_ypr[1] >> 8) & 0xFF;
//		sendImuMsg.data[4] = _ypr[2] & 0xFF;
//		sendImuMsg.data[5] = (_ypr[2] >> 8) & 0xFF;
//
//		_imuCount++;
//	}
//
//#if IMU_DEBUG
//	if (_imuCount > 300) {
//		_imuCount = 0;
//
//		Serial.print(F("ypr: "));
//		Serial.print(_ypr[0]);
//		Serial.print(' ');
//		Serial.print(_ypr[1]);
//		Serial.print(' ');
//		Serial.print(_ypr[2]);
//		Serial.print('\t');
//		Serial.print(imuSendTimer);
//		Serial.println();
//
//	}
//#endif // IMU_DEBUG
}

//----------- BUS -----------//
void busInit()
{
	byte i;

#if BUS_DEBUG
	Serial.print(F("bus init ..."));
#endif // BUS_DEBUG
	SPI.begin();

	bus.reset();
	bus.setBitrate(CAN_500KBPS);
	//busSetFilter();
	bus.setNormalMode();

	sendOptMsg.can_id = BUS_OPT_ID;
	sendOptMsg.can_dlc = BUS_OPT_SIZE;
	for ( i = 0; i < BUS_OPT_SIZE; i++ )
		sendOptMsg.data[i] = 0;

	sendImuMsg.can_id = BUS_IMU_ID;
	sendImuMsg.can_dlc = BUS_IMU_SIZE;
	for ( i = 0; i < BUS_IMU_SIZE; i++ )
		sendImuMsg.data[i] = 0;
#if BUS_DEBUG
	Serial.println(F("done!"));
#endif // BUS_DEBUG
}

void busHandler()
{
	byte _cam_command = 0;
	byte _lrf_command = 0;
	static byte _prev_cam_command = 0;
	static byte _prev_lrf_command = 0;
	byte a, b;
//	byte focusSpeed;

	static uint32_t sendCamInfoTimer = millis() + 1000;
	static uint32_t sendImuTimer = millis() + 500;

	//receive command
	if (bus.readMessage(&recvMsg) == MCP2515::ERROR_OK) {
		//command from button
		if (recvMsg.can_id == BUS_BUTTON_ID) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

			_cam_command = recvMsg.data[1];
#if BUS_DEBUG
			Serial.print(F("button= 0b"));
			Serial.println(_cam_command, BIN);
#endif // BUS_DEBUG

			////////////TEST/////////////////////////////////////////////
			////sony test
			//bitSet(_cam_command, 0);
			////thermal test
			//bitSet(_cam_command, 1);
			/////////////////////////////////////////////////////////////

			//cam selection
			if (bitRead(_cam_command, 0))
				camSelect(CAM_SELECT_SONY);
			else if (bitRead(_cam_command, 1))
				camSelect(CAM_SELECT_THERMAL);
			else
				camSelect(CAM_SELECT_NONE);

			//zoom
			if (camState == CAM_SELECT_SONY || camState == CAM_SELECT_THERMAL) {
				if (bitRead(_cam_command, 2)) {
					if (!bitRead(_prev_cam_command, 2)) {
						camZoomAdd();
						Serial.print(F("Zoom: "));
						Serial.println(zoomLevel);
					}
					//if (camState == CAM_SELECT_SONY && zoomLevel >= 3)
					//	sony.set_stabilizer(&sonyIface, &sonyCamera, VISCA_CAM_STABILIZER_ON);
				}
				else if (bitRead(_cam_command, 3)) {
					if (!bitRead(_prev_cam_command, 3)) {
						camZoomSubtract();
						Serial.print(F("Zoom: "));
						Serial.println(zoomLevel);
					}
					//if (camState == CAM_SELECT_SONY && zoomLevel <= 2)
					//	sony.set_stabilizer(&sonyIface, &sonyCamera, VISCA_CAM_STABILIZER_OFF);
				}
			}

			//focus
			if (camState == CAM_SELECT_SONY) {
				a = _cam_command & 0b110000;
				b = _prev_cam_command & 0b110000;
				if (a != b) {
					a = a >> 4;
					if (a == SONY_FOCUS_FAR) {
						//if (zoomLevel == 1) focusSpeed = 7;
						//else if (zoomLevel == 2) focusSpeed = 3;
						//else if (zoomLevel == 3) focusSpeed = 1;
						//else focusSpeed = 0;
						//sony.set_focus_far_speed(&sonyIface, &sonyCamera, focusSpeed);
						sony.set_focus_auto(&sonyIface, &sonyCamera,
						VISCA_FOCUS_AUTO_OFF);
						sony.set_focus_far(&sonyIface, &sonyCamera);
					}
					else if (a == SONY_FOCUS_NEAR) {
						//if (zoomLevel == 1) focusSpeed = 7;
						//else if (zoomLevel == 2) focusSpeed = 3;
						//else if (zoomLevel == 3) focusSpeed = 1;
						//else focusSpeed = 0;
						//sony.set_focus_near_speed(&sonyIface, &sonyCamera, focusSpeed);
						sony.set_focus_auto(&sonyIface, &sonyCamera,
						VISCA_FOCUS_AUTO_OFF);
						sony.set_focus_near(&sonyIface, &sonyCamera);
					}
					else {
						sony.set_focus_stop(&sonyIface, &sonyCamera);
					}
				}
			}

			_prev_cam_command = _cam_command;
		}
		//command from mainControl
		else if (recvMsg.can_id == BUS_MAIN_CMD2_ID) {
			_lrf_command = recvMsg.data[1];

			//TODO:
			// add override handler

			//imu update according to stab movement
			if ((recvMsg.data[0] & 0b1100) && imuSendTimer == imuNormalTimeout) {
				Serial.println(F("STAB start"));
			}
			if (recvMsg.data[0] & 0b1100)
				imuSendTimer = imuStabTimeout;
			else
				imuSendTimer = imuNormalTimeout;

			//LRF start measuring command
			if (bitRead(_lrf_command, 1) && !bitRead(_prev_lrf_command, 1)) {
				if (lrfState == LRF_DISABLE_state) {
					digitalWrite(LRF_POWER_PIN, LRF_POWER_ON);
					delayWdt(10);
					digitalWrite(LRF_ENABLE_PIN, LRF_POWER_ON);
					delayWdt(100);
					lrfState = LRF_ENABLE_state;
					lrfTimerEnable = millis() + 5000;
					lrfString = "";
#if LRF_DEBUG
					Serial.println(F("lrf state: LRF_ENABLE_state"));
#endif // LRF_DEBUG
				}

			}

			_prev_lrf_command = _lrf_command;
		}
	}

	//send camera state
	if (millis() > sendCamInfoTimer) {
		sendCamInfoTimer = millis() + 300;

		bus.sendMessage(&sendOptMsg);
	}

	//send imu data
	if (millis() > sendImuTimer) {
		sendImuTimer = millis() + imuSendTimer;

		bus.sendMessage(&sendImuMsg);
	}

}

void busSetFilter()
{
	//RX Buffer 0
	bus.setFilterMask(MCP2515::MASK0, 0, 0x3FF);
	bus.setFilter(MCP2515::RXF0, 0, 0x101);		// Gunner main control
	bus.setFilter(MCP2515::RXF1, 0, 0x000);

	//RX Buffer 1
	bus.setFilterMask(MCP2515::MASK1, 0, 0x3FF);
	bus.setFilter(MCP2515::RXF2, 0, 0x220);		// Commander's button
	bus.setFilter(MCP2515::RXF3, 0, 0x201);		// Commander's main control
	bus.setFilter(MCP2515::RXF4, 0, 0x000);
	bus.setFilter(MCP2515::RXF5, 0, 0x000);
}

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
