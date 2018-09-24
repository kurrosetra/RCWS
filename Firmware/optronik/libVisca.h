// libVisca.h

#ifndef _LIBVISCA_h
#define _LIBVISCA_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//#define VISCA_DEBUG

/**********************/
/* Message formatting */
/**********************/

#define VISCA_COMMAND                    0x01
#define VISCA_INQUIRY                    0x09
#define VISCA_TERMINATOR                 0xFF

#define VISCA_CATEGORY_INTERFACE         0x00
#define VISCA_CATEGORY_CAMERA1           0x04
#define VISCA_CATEGORY_PAN_TILTER        0x06
#define VISCA_CATEGORY_CAMERA2           0x07

/* Known Vendor IDs */
#define VISCA_VENDOR_SONY    0x0020

/* Known Model IDs. The manual can be taken from
* http://www.sony.net/Products/ISP/docu_soft/index.html
*/
#define VISCA_MODEL_IX47x    0x0401          /* from FCB-IX47, FCB-IX470 instruction list */
#define VISCA_MODEL_EX47xL   0x0402          /* from FCB-EX47L, FCB-EX470L instruction list */
#define VISCA_MODEL_IX10     0x0404          /* FCB-IX10, FCB-IX10P instruction list */

#define VISCA_MODEL_EX780    0x0411          /* from EX780S(P) tech-manual */

#define VISCA_MODEL_EX480A   0x0412          /* from EX48A/EX480A tech-manual */
#define VISCA_MODEL_EX480AP  0x0413
#define VISCA_MODEL_EX48A    0x0414
#define VISCA_MODEL_EX48AP   0x0414
#define VISCA_MODEL_EX45M    0x041E
#define VISCA_MODEL_EX45MCE  0x041F

#define VISCA_MODEL_IX47A    0x0418          /* from IX47A tech-manual */
#define VISCA_MODEL_IX47AP   0x0419
#define VISCA_MODEL_IX45A    0x041A
#define VISCA_MODEL_IX45AP   0x041B

#define VISCA_MODEL_IX10A    0x041C          /* from IX10A tech-manual */
#define VISCA_MODEL_IX10AP   0x041D

#define VISCA_MODEL_EX780B   0x0420          /* from EX78/EX780 tech-manual */
#define VISCA_MODEL_EX780BP  0x0421
#define VISCA_MODEL_EX78B    0x0422
#define VISCA_MODEL_EX78BP   0x0423

#define VISCA_MODEL_EX480B   0x0424          /* from EX48/EX480 tech-manual */
#define VISCA_MODEL_EX480BP  0x0425
#define VISCA_MODEL_EX48B    0x0426
#define VISCA_MODEL_EX48BP   0x0427

#define VISCA_MODEL_EX980S   0x042E          /* from EX98/EX980 tech-manual */
#define VISCA_MODEL_EX980SP  0x042F
#define VISCA_MODEL_EX980    0x0430
#define VISCA_MODEL_EX980P   0x0431

#define VISCA_MODEL_H10      0x044A	     /* from H10 tech-manual */


/* Commands/inquiries codes */
#define VISCA_POWER                      0x00
#define VISCA_DEVICE_INFO                0x02
#define VISCA_KEYLOCK                    0x17
#define VISCA_ID                         0x22
#define VISCA_ZOOM                       0x07
#define   VISCA_ZOOM_STOP                  0x00
#define   VISCA_ZOOM_TELE                  0x02
#define   VISCA_ZOOM_WIDE                  0x03
#define   VISCA_ZOOM_TELE_SPEED            0x20
#define   VISCA_ZOOM_WIDE_SPEED            0x30
#define VISCA_ZOOM_VALUE                 0x47
#define VISCA_ZOOM_FOCUS_VALUE           0x47
#define VISCA_DZOOM                      0x06
#define   VISCA_DZOOM_OFF                  0x03
#define   VISCA_DZOOM_ON                   0x02
#define VISCA_DZOOM_LIMIT                0x26 /* implemented for H10 */
#define   VISCA_DZOOM_1X                   0x00
#define   VISCA_DZOOM_1_5X                 0x01
#define   VISCA_DZOOM_2X                   0x02
#define   VISCA_DZOOM_4X                   0x03
#define   VISCA_DZOOM_8X                   0x04
#define   VISCA_DZOOM_12X                  0x05
#define VISCA_DZOOM_MODE                 0x36
#define   VISCA_DZOOM_COMBINE              0x00
#define   VISCA_DZOOM_SEPARATE             0x01
#define VISCA_FOCUS                      0x08
#define   VISCA_FOCUS_STOP                 0x00
#define   VISCA_FOCUS_FAR                  0x02
#define   VISCA_FOCUS_NEAR                 0x03
#define   VISCA_FOCUS_FAR_SPEED            0x20
#define   VISCA_FOCUS_NEAR_SPEED           0x30
#define VISCA_FOCUS_VALUE                0x48
#define VISCA_FOCUS_AUTO                 0x38
#define   VISCA_FOCUS_AUTO_ON              0x02
#define   VISCA_FOCUS_AUTO_OFF             0x03
#define   VISCA_FOCUS_AUTO_MAN             0x10
#define VISCA_FOCUS_ONE_PUSH             0x18
#define   VISCA_FOCUS_ONE_PUSH_TRIG        0x01
#define   VISCA_FOCUS_ONE_PUSH_INF         0x02
#define VISCA_FOCUS_AUTO_SENSE           0x58
#define   VISCA_FOCUS_AUTO_SENSE_HIGH      0x02
#define   VISCA_FOCUS_AUTO_SENSE_LOW       0x03
#define VISCA_FOCUS_NEAR_LIMIT           0x28
#define VISCA_WB                         0x35
#define   VISCA_WB_AUTO                    0x00
#define   VISCA_WB_INDOOR                  0x01
#define   VISCA_WB_OUTDOOR                 0x02
#define   VISCA_WB_ONE_PUSH                0x03
#define   VISCA_WB_ATW                     0x04
#define   VISCA_WB_MANUAL                  0x05
#define VISCA_WB_TRIGGER                 0x10
#define   VISCA_WB_ONE_PUSH_TRIG           0x05
#define VISCA_RGAIN                      0x03
#define VISCA_RGAIN_VALUE                0x43
#define VISCA_BGAIN                      0x04
#define VISCA_BGAIN_VALUE                0x44
#define VISCA_AUTO_EXP                   0x39
#define   VISCA_AUTO_EXP_FULL_AUTO         0x00
#define   VISCA_AUTO_EXP_MANUAL            0x03
#define   VISCA_AUTO_EXP_SHUTTER_PRIORITY  0x0A
#define   VISCA_AUTO_EXP_IRIS_PRIORITY     0x0B
#define   VISCA_AUTO_EXP_GAIN_PRIORITY     0x0C
#define   VISCA_AUTO_EXP_BRIGHT            0x0D
#define   VISCA_AUTO_EXP_SHUTTER_AUTO      0x1A
#define   VISCA_AUTO_EXP_IRIS_AUTO         0x1B
#define   VISCA_AUTO_EXP_GAIN_AUTO         0x1C
#define VISCA_SLOW_SHUTTER               0x5A
#define   VISCA_SLOW_SHUTTER_AUTO          0x02
#define   VISCA_SLOW_SHUTTER_MANUAL        0x03
#define VISCA_SHUTTER                    0x0A
#define VISCA_SHUTTER_VALUE              0x4A
#define VISCA_IRIS                       0x0B
#define VISCA_IRIS_VALUE                 0x4B
#define VISCA_GAIN                       0x0C
#define VISCA_GAIN_VALUE                 0x4C
#define VISCA_BRIGHT                     0x0D
#define VISCA_BRIGHT_VALUE               0x4D
#define VISCA_EXP_COMP                   0x0E
#define VISCA_EXP_COMP_POWER             0x3E
#define VISCA_EXP_COMP_VALUE             0x4E
#define VISCA_BACKLIGHT_COMP             0x33
#define   VISCA_BACKLIGHT_COMP_ON		 0x02
#define   VISCA_BACKLIGHT_COMP_OFF		 0x03
#define VISCA_SPOT_AE                    0x59
#define   VISCA_SPOT_AE_ON               0x02
#define   VISCA_SPOT_AE_OFF              0x03
#define VISCA_SPOT_AE_POSITION           0x29
#define VISCA_APERTURE                   0x02
#define VISCA_APERTURE_VALUE             0x42
#define VISCA_ZERO_LUX                   0x01
#define VISCA_IR_LED                     0x31
#define VISCA_WIDE_MODE                  0x60
#define   VISCA_WIDE_MODE_OFF              0x00
#define   VISCA_WIDE_MODE_CINEMA           0x01
#define   VISCA_WIDE_MODE_16_9             0x02
#define VISCA_MIRROR                     0x61
#define VISCA_FREEZE                     0x62
#define   VISCA_FREEZE_ON                  0x02
#define   VISCA_FREEZE_OFF                 0x03
#define VISCA_PICTURE_EFFECT             0x63
#define   VISCA_PICTURE_EFFECT_OFF         0x00
#define   VISCA_PICTURE_EFFECT_PASTEL      0x01
#define   VISCA_PICTURE_EFFECT_NEGATIVE    0x02
#define   VISCA_PICTURE_EFFECT_SEPIA       0x03
#define   VISCA_PICTURE_EFFECT_BW          0x04
#define   VISCA_PICTURE_EFFECT_SOLARIZE    0x05
#define   VISCA_PICTURE_EFFECT_MOSAIC      0x06
#define   VISCA_PICTURE_EFFECT_SLIM        0x07
#define   VISCA_PICTURE_EFFECT_STRETCH     0x08
#define VISCA_DIGITAL_EFFECT             0x64
#define   VISCA_DIGITAL_EFFECT_OFF         0x00
#define   VISCA_DIGITAL_EFFECT_STILL       0x01
#define   VISCA_DIGITAL_EFFECT_FLASH       0x02
#define   VISCA_DIGITAL_EFFECT_LUMI        0x03
#define   VISCA_DIGITAL_EFFECT_TRAIL       0x04
#define VISCA_DIGITAL_EFFECT_LEVEL       0x65
#define VISCA_CAM_STABILIZER             0x34
#define   VISCA_CAM_STABILIZER_HOLD       0x00
#define   VISCA_CAM_STABILIZER_ON         0x02
#define   VISCA_CAM_STABILIZER_OFF        0x03
#define VISCA_MEMORY                     0x3F
#define   VISCA_MEMORY_RESET               0x00
#define   VISCA_MEMORY_SET                 0x01
#define   VISCA_MEMORY_RECALL              0x02
#define     VISCA_MEMORY_0                   0x00
#define     VISCA_MEMORY_1                   0x01
#define     VISCA_MEMORY_2                   0x02
#define     VISCA_MEMORY_3                   0x03
#define     VISCA_MEMORY_4                   0x04
#define     VISCA_MEMORY_5                   0x05
#define     VISCA_MEMORY_CUSTOM              0x7F
#define VISCA_DISPLAY                    0x15
#define   VISCA_DISPLAY_ON                  0x02
#define   VISCA_DISPLAY_OFF                 0x03
#define   VISCA_DISPLAY_TOGGLE              0x10
#define VISCA_DATE_TIME_SET              0x70
#define VISCA_DATE_DISPLAY               0x71
#define VISCA_TIME_DISPLAY               0x72
#define VISCA_TITLE_DISPLAY              0x74
#define   VISCA_TITLE_DISPLAY_CLEAR        0x00
#define   VISCA_TITLE_DISPLAY_ON           0x02
#define   VISCA_TITLE_DISPLAY_OFF          0x03
#define VISCA_TITLE_SET                  0x73
#define   VISCA_TITLE_SET_PARAMS           0x00
#define   VISCA_TITLE_SET_PART1            0x01
#define   VISCA_TITLE_SET_PART2            0x02
#define VISCA_IRRECEIVE                   0x08
#define   VISCA_IRRECEIVE_ON              0x02
#define   VISCA_IRRECEIVE_OFF             0x03
#define   VISCA_IRRECEIVE_ONOFF           0x10
#define VISCA_PT_DRIVE                     0x01
#define   VISCA_PT_DRIVE_HORIZ_LEFT        0x01
#define   VISCA_PT_DRIVE_HORIZ_RIGHT       0x02
#define   VISCA_PT_DRIVE_HORIZ_STOP        0x03
#define   VISCA_PT_DRIVE_VERT_UP           0x01
#define   VISCA_PT_DRIVE_VERT_DOWN         0x02
#define   VISCA_PT_DRIVE_VERT_STOP         0x03
#define VISCA_PT_ABSOLUTE_POSITION         0x02
#define VISCA_PT_RELATIVE_POSITION         0x03
#define VISCA_PT_HOME                      0x04
#define VISCA_PT_RESET                     0x05
#define VISCA_PT_LIMITSET                  0x07
#define   VISCA_PT_LIMITSET_SET            0x00
#define   VISCA_PT_LIMITSET_CLEAR          0x01
#define     VISCA_PT_LIMITSET_SET_UR       0x01
#define     VISCA_PT_LIMITSET_SET_DL       0x00
#define VISCA_PT_DATASCREEN                0x06
#define   VISCA_PT_DATASCREEN_ON           0x02
#define   VISCA_PT_DATASCREEN_OFF          0x03
#define   VISCA_PT_DATASCREEN_ONOFF        0x10

#define VISCA_PT_VIDEOSYSTEM_INQ           0x23
#define VISCA_PT_MODE_INQ                  0x10
#define VISCA_PT_MAXSPEED_INQ              0x11
#define VISCA_PT_POSITION_INQ              0x12
#define VISCA_PT_DATASCREEN_INQ            0x06


/**************************/
/* DIRECT REGISTER ACCESS */
/**************************/

#define VISCA_REGISTER_VALUE              0x24

#define VISCA_REGISTER_VISCA_BAUD          0x00
#define VISCA_REGISTER_BD9600               0x00
#define VISCA_REGISTER_BD19200              0x01
#define VISCA_REGISTER_BD38400              0x02

/* FCB-H10: Video Standard */
#define VISCA_REGISTER_VIDEO_SIGNAL        0x70

#define VISCA_REGISTER_VIDEO_1080I_60       0x01
#define VISCA_REGISTER_VIDEO_720P_60        0x02
#define VISCA_REGISTER_VIDEO_D1_CROP_60     0x03
#define VISCA_REGISTER_VIDEO_D1_SQ_60       0x04

#define VISCA_REGISTER_VIDEO_1080I_50       0x11
#define VISCA_REGISTER_VIDEO_720P_50        0x12
#define VISCA_REGISTER_VIDEO_D1_CROP_50     0x13
#define VISCA_REGISTER_VIDEO_D1_SQ_50       0x14


/*****************/
/* D30/D31 CODES */
/*****************/
#define VISCA_WIDE_CON_LENS		   0x26
#define   VISCA_WIDE_CON_LENS_SET          0x00

#define VISCA_AT_MODE                      0x01
#define   VISCA_AT_ONOFF                   0x10
#define VISCA_AT_AE                        0x02
#define VISCA_AT_AUTOZOOM                  0x03
#define VISCA_ATMD_FRAMEDISPLAY            0x04
#define VISCA_AT_FRAMEOFFSET               0x05
#define VISCA_ATMD_STARTSTOP               0x06
#define VISCA_AT_CHASE                     0x07
#define   VISCA_AT_CHASE_NEXT              0x10

#define VISCA_MD_MODE                      0x08
#define   VISCA_MD_ONOFF                   0x10
#define VISCA_MD_FRAME                     0x09
#define VISCA_MD_DETECT                    0x0A

#define VISCA_MD_ADJUST                    0x00
#define   VISCA_MD_ADJUST_YLEVEL           0x0B
#define   VISCA_MD_ADJUST_HUELEVEL         0x0C
#define   VISCA_MD_ADJUST_SIZE             0x0D
#define   VISCA_MD_ADJUST_DISPTIME         0x0F
#define   VISCA_MD_ADJUST_REFTIME          0x0B
#define   VISCA_MD_ADJUST_REFMODE          0x10

#define VISCA_AT_ENTRY                     0x15
#define VISCA_AT_LOSTINFO                  0x20
#define VISCA_MD_LOSTINFO                  0x21
#define VISCA_ATMD_LOSTINFO1               0x20
#define VISCA_ATMD_LOSTINFO2               0x07

#define VISCA_MD_MEASURE_MODE_1            0x27
#define VISCA_MD_MEASURE_MODE_2            0x28

#define VISCA_ATMD_MODE                    0x22
#define VISCA_AT_MODE_QUERY                0x23
#define VISCA_MD_MODE_QUERY                0x24
#define VISCA_MD_REFTIME_QUERY             0x11
#define VISCA_AT_POSITION                  0x20
#define VISCA_MD_POSITION                  0x21

/***************/
/* ERROR CODES */
/***************/

/* these two are defined by me, not by the specs. */
#define VISCA_SUCCESS                    0x00
#define VISCA_FAILURE                    0xFF

/* specs errors: */
#define VISCA_ERROR_MESSAGE_LENGTH       0x01
#define VISCA_ERROR_SYNTAX               0x02
#define VISCA_ERROR_CMD_BUFFER_FULL      0x03
#define VISCA_ERROR_CMD_CANCELLED        0x04
#define VISCA_ERROR_NO_SOCKET            0x05
#define VISCA_ERROR_CMD_NOT_EXECUTABLE   0x41

/* Generic definitions */
#define VISCA_ON                         0x02
#define VISCA_OFF                        0x03
#define VISCA_RESET                      0x00
#define VISCA_UP                         0x02
#define VISCA_DOWN                       0x03

/* response types */
#define VISCA_RESPONSE_CLEAR             0x40
#define VISCA_RESPONSE_ADDRESS           0x30
#define VISCA_RESPONSE_ACK               0x40
#define VISCA_RESPONSE_COMPLETED         0x50
#define VISCA_RESPONSE_ERROR             0x60

/* timeout in ms */
#define VISCA_SERIAL_WAIT				 100

/* size of the local packet buffer */
#define VISCA_INPUT_BUFFER_SIZE            32

/* This is the interface for the AVR platform.
*/
typedef struct _VISCA_interface
{
	//// RS232 data:
	//v24_port_t port_fd;

	// VISCA data:
	int address;
	int broadcast;

	// RS232 input buffer
	unsigned char ibuf[VISCA_INPUT_BUFFER_SIZE];
	int bytes;
	int type;
} VISCAInterface_t;


/* CAMERA STRUCTURE */
typedef struct _VISCA_camera
{
	// VISCA data:
	int address;

	// camera info:
	uint32_t vendor;
	uint32_t model;
	uint32_t rom_version;
	uint32_t socket_num;

} VISCACamera_t;


/* TITLE STRUCTURE */
typedef struct _VISCA_title
{
	uint32_t vposition;
	uint32_t hposition;
	uint32_t color;
	uint32_t blink;
	unsigned char title[20];

} VISCATitleData_t;

typedef struct _VISCA_packet
{
	unsigned char bytes[32];
	uint32_t length;
} VISCAPacket_t;

class LibVisca
{
public:
	//functions
	LibVisca(HardwareSerial &uart, unsigned long baud=9600);

	/* GENERAL FUNCTIONS */
	void open_serial();
	void close_serial();
	uint8_t set_address(VISCAInterface_t *iface, uint8_t *camera_num);
	uint8_t clear(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t get_camera_info(VISCAInterface_t *iface, VISCACamera_t *camera);

	/* COMMANDS */
	uint8_t set_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_keylock(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_camera_id(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t id);
	uint8_t set_zoom_tele(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_zoom_wide(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_zoom_stop(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_zoom_tele_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);
	uint8_t set_zoom_wide_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);
	uint8_t set_zoom_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t zoom);
	uint8_t set_zoom_and_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t zoom, uint32_t focus);
	uint8_t set_dzoom(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t power);
	uint8_t set_dzoom_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t limit);
	uint8_t set_dzoom_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t power);
	uint8_t set_focus_far(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_near(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_stop(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_far_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);
	uint8_t set_focus_near_speed(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t speed);
	uint8_t set_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t focus);
	uint8_t set_focus_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_focus_one_push(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_infinity(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_autosense_high(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_autosense_low(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_focus_near_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t limit);
	uint8_t set_whitebal_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t mode);
	uint8_t set_whitebal_one_push(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_rgain_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_rgain_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_rgain_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_rgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_bgain_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_bgain_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_bgain_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_bgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_shutter_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_shutter_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_shutter_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_shutter_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_iris_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_iris_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_iris_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_iris_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_gain_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_gain_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_gain_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_gain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_bright_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_bright_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_bright_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_bright_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_aperture_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_aperture_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_aperture_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_aperture_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_exp_comp_up(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_exp_comp_down(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_exp_comp_reset(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_exp_comp_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t value);
	uint8_t set_exp_comp_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_auto_exp_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);
	uint8_t set_slow_shutter_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_backlight_comp(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_zero_lux_shot(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_ir_led(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_wide_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);
	uint8_t set_mirror(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_freeze(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_picture_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);
	uint8_t set_digital_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);
	uint8_t set_digital_effect_level(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t level);
	uint8_t set_stabilizer(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t mode);
	uint8_t memory_set(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t channel);
	uint8_t memory_recall(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t channel);
	uint8_t memory_reset(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t channel);
	uint8_t set_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_date_time(VISCAInterface_t *iface, VISCACamera_t *camera, uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute);
	uint8_t set_date_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_time_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_title_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t power);
	uint8_t set_title_clear(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_title_params(VISCAInterface_t *iface, VISCACamera_t *camera, VISCATitleData_t *title);
	uint8_t set_title(VISCAInterface_t *iface, VISCACamera_t *camera, VISCATitleData_t *title);
	uint8_t set_irreceive_on(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_irreceive_off(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t set_irreceive_onoff(VISCAInterface_t *iface, VISCACamera_t *camera);
	/* INQUIRIES */
	uint8_t get_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_dzoom(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_dzoom_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *value);
	uint8_t get_zoom_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_focus_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_focus_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_focus_auto_sense(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_focus_near_limit(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_whitebal_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_rgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_bgain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_auto_exp_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_slow_shutter_auto(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_shutter_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_iris_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_gain_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_bright_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_exp_comp_power(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_exp_comp_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_backlight_comp(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_aperture_value(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_zero_lux_shot(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_ir_led(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_wide_mode(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_mirror(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_freeze(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_picture_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_digital_effect(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *mode);
	uint8_t get_digital_effect_level(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *value);
	uint8_t get_memory(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *channel);
	uint8_t get_display(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *power);
	uint8_t get_id(VISCAInterface_t *iface, VISCACamera_t *camera, uint16_t *id);
	uint8_t get_videosystem(VISCAInterface_t *iface, VISCACamera_t *camera, uint8_t *system);

	//variables

private:
	//functions
	void _append_byte(VISCAPacket_t *packet, uint8_t data);
	void _init_packet(VISCAPacket_t *packet);
	uint8_t _get_reply(VISCAInterface_t *iface, VISCACamera_t *camera);
	uint8_t _send_packet_with_reply(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet);
	uint8_t _write_packet_data(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet);
	uint8_t _send_packet(VISCAInterface_t *iface, VISCACamera_t *camera, VISCAPacket_t *packet);
	uint8_t _get_packet(VISCAInterface_t *iface);
	uint8_t _rx_clear();
	void _tx_flush();


	//variables
	HardwareSerial *_uart;
	unsigned long _baud;

};


#endif	//_LIBVISCA_h

