#include "Arduino.h"
#include <avr/wdt.h>
#include <SPI.h>
#include <mcp2515.h>
#include "libVisca.h"
//#include "vn100.h"

#define DEBUG					1
#if DEBUG

#define	BUS_DEBUG				1
#define CAM_DEBUG				0
#define LRF_DEBUG				1
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
#define LRF_VALUE_VALID_TIMEOUT	10000
#define LRF_NEXT_REQ_TIMEOUT	2000

String lrfString;
byte lrfBufSize = 64;
uint16_t lrfVal = 0;
uint32_t lrfValValidTimer = 0;
uint32_t lrfNextStartTimer = 0;

//CAN parameter
const byte BUS_CS_PIN = 10;

const uint16_t BUS_MAIN_CMD1_ID = 0x310;
const uint16_t BUS_MAIN_CMD2_ID = 0x311;
const uint16_t BUS_BUTTON_ID = 0x320;
const uint16_t BUS_OPT_ID = 0x330;
const uint16_t BUS_IMU_ID = 0x331;
const uint32_t BUS_JOYSTICK_ID = 0x8CFDD633;
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

	Serial.begin(230400);
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
#if LRF_DEBUG
	Serial.print(F("LRF init ..."));
#endif	//#if LRF_DEBUG

	Serial1.begin(115200);

	// LRF's PSU
	pinMode(LRF_POWER_PIN, OUTPUT);
	lrfPower(LRF_POWER_OFF);

	lrfString.reserve(lrfBufSize);
	lrfString = "";
#if LRF_DEBUG
	Serial.println(F("done!"));
#endif	//#if LRF_DEBUG

}

void lrfPower(bool onf)
{
	digitalWrite(LRF_POWER_PIN, onf);
}

bool getLrfPower()
{
	return digitalRead(LRF_POWER_PIN);
}

void lrfHandler()
{
	static bool onCommand = 0;
	static bool lrfNewVal = 0;
	static uint32_t lrfSendBeat = 0;
	bool lrfCompleted = 0;
	char c;
	byte awal, akhir;
	String s;
	uint16_t _lrfTem = 0;

	if (getLrfPower() == LRF_POWER_OFF) {
		lrfSendBeat = 0;
		onCommand = 0;
	}
	else {
		if (lrfSendBeat == 0) {
			lrfSendBeat = millis() + 1000;
			lrfString = "";
			//flush rx data buffer
			while (Serial1.available())
				Serial1.read();
		}
		else {
			if (millis() > lrfSendBeat && !onCommand) {
				onCommand = 1;
				Serial1.println(F("\r\nON"));
#if LRF_DEBUG
				Serial.println(F("Send LRF ON command"));
#endif	//#if LRF_DEBUG

			}
		}

	}

	if (Serial1.available()) {
		c = Serial1.read();
#if LRF_DEBUG
		if (c != 0)
			Serial.write(c);
#endif	//#if LRF_DEBUG

		if (getLrfPower() == LRF_POWER_ON) {
			if (c == 0x0D)
				lrfString = "";
			else if (c == 'm')
				lrfCompleted = 1;

			lrfString += c;

			if (lrfCompleted) {
				if (lrfString.indexOf("D=") >= 0 && lrfString.indexOf('m') >= 0) {

#if LRF_DEBUG
					Serial.println(lrfString);
#endif	//#if LRF_DEBUG

					awal = lrfString.indexOf('=') + 1;
					if (lrfString.indexOf('.') >= 0)
						akhir = lrfString.indexOf('.');
					else
						akhir = lrfString.indexOf('m');

					s = lrfString.substring(awal, akhir);

					if (lrfNextStartTimer && !lrfNewVal) {
						_lrfTem = s.toInt();
						if (_lrfTem != lrfVal) {
							lrfVal = _lrfTem;
#if LRF_DEBUG
							Serial.print(F("new LRF val= "));
							Serial.println(lrfVal);
#endif	//#if LRF_DEBUG
							lrfNewVal = 1;
						}
					}

					lrfString = "";
				}
			}	//(lrfCompleted)
		}  // getLrfPower() == LRF_POWER_ON
		else {
			lrfString = "";
		}	// else getLrfPower() == LRF_POWER_ON
	}	//(Serial1.available())

	if (lrfNextStartTimer && millis() >= lrfNextStartTimer) {
		lrfNextStartTimer = 0;
		lrfNewVal = 0;
	}

	if (lrfValValidTimer && millis() >= lrfValValidTimer) {
		lrfValValidTimer = 0;
		lrfVal = 0;
	}

	//LSB
	sendOptMsg.data[2] = lrfVal & 0xFF;
	//MSB
	sendOptMsg.data[1] = (lrfVal >> 8);
	//bit 6 -> pointer
	bitClear(sendOptMsg.data[1], 6);
	//bit 7 -> power
	bitWrite(sendOptMsg.data[1], 7, getLrfPower());

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
	bus.setBitrate(CAN_250KBPS);
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

void busSetFilter()
{
	//RX Buffer 0
	bus.setFilterMask(MCP2515::MASK0, 0, 0x3FF);
	bus.setFilter(MCP2515::RXF0, 0, 0x311);  // panel
	bus.setFilter(MCP2515::RXF1, 0, 0x000);

	//RX Buffer 1
	bus.setFilterMask(MCP2515::MASK1, 0, 0x3FF);
	bus.setFilter(MCP2515::RXF2, 0, 0x220);  // Commander's button
	bus.setFilter(MCP2515::RXF3, 0, 0x201);  // Commander's main control
	bus.setFilter(MCP2515::RXF4, 0, 0x000);
	bus.setFilter(MCP2515::RXF5, 0, 0x000);
}

void busHandler()
{
	byte _cam_command = 0;
	byte _js_command = 0;
	static byte _prev_cam_command = 0;
	static byte _prev_js_command = 0;

	static uint32_t sendCamInfoTimer = millis() + 1000;
//	static uint32_t sendImuTimer = millis() + 500;

	//receive command
	if (bus.readMessage(&recvMsg) == MCP2515::ERROR_OK) {
		//command from panel
		if (recvMsg.can_id == BUS_MAIN_CMD2_ID) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

			/**
			 * CAMERA COMMAND
			 */
			_cam_command = recvMsg.data[3];
			if (_cam_command != _prev_cam_command) {
#if BUS_DEBUG
				Serial.print(F("cam= 0b "));
				Serial.println(_cam_command);
#endif	//#if BUS_DEBUG

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

				_prev_cam_command = _cam_command;
			}	//(_cam_command != _prev_cam_command)

			/**
			 * LRF POWER COMMAND
			 */
			lrfPower(bitRead(recvMsg.data[1], 1));
		}
		//(recvMsg.can_id == BUS_MAIN_CMD2_ID)
		else if (recvMsg.can_id == BUS_JOYSTICK_ID) {
			_js_command = recvMsg.data[5];

			/**
			 * ZOOM COMMAND
			 */
			if (camState == CAM_SELECT_SONY || camState == CAM_SELECT_THERMAL) {
				if (bitRead(_js_command, 4)) {
					if (!bitRead(_prev_js_command, 4)) {
						camZoomAdd();
#if BUS_DEBUG
						Serial.print(F("Zoom: "));
						Serial.println(zoomLevel);
#endif	//#if BUS_DEBUG
					}
				}
				else if (bitRead(_js_command, 6)) {
					if (!bitRead(_prev_js_command, 6)) {
						camZoomSubtract();
#if BUS_DEBUG
						Serial.print(F("Zoom: "));
						Serial.println(zoomLevel);
#endif	//#if BUS_DEBUG
					}
				}
			}

			/**
			 * LRF COMMAND
			 */
			if (getLrfPower() == LRF_POWER_ON && bitRead(recvMsg.data[5], 2)) {

				if (!lrfNextStartTimer) {
#if BUS_DEBUG
					Serial.println(F("LRF REQ DATA!"));
#endif	//#if BUS_DEBUG
					lrfNextStartTimer = millis() + LRF_NEXT_REQ_TIMEOUT;
					lrfValValidTimer = millis() + LRF_VALUE_VALID_TIMEOUT;
				}
			}

			_prev_js_command = _js_command;
		}	//recvMsg.can_id == BUS_JOYSTICK_ID

		//send camera state
		if (millis() > sendCamInfoTimer) {
			sendCamInfoTimer = millis() + 300;

			bus.sendMessage(&sendOptMsg);
		}

//		//send imu data
//		if (millis() > sendImuTimer) {
//			sendImuTimer = millis() + imuSendTimer;
//
//			bus.sendMessage(&sendImuMsg);
//		}
	}	//(bus.readMessage(&recvMsg) == MCP2515::ERROR_OK)
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
