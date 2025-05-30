#ifndef hyrel_can_HEADER // prevent double dipping
#define hyrel_can_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    hyrel_can.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: include file for common public defines and functions of hyrel_can.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////
#include "main.h"

#define CAN_SYSTEM_DEVICE_ID    0x00

#ifdef BOOTLOADER
#define CAN_NUM_TX_TRANSFERS_PER_SLICE  1
#else
#define CAN_NUM_TX_TRANSFERS_PER_SLICE  3
#endif

#define MAX_TSR_TERR_ALLOWED 5
typedef enum {
	CAN_TRANSMIT_ERROR_MAILBOX_0 = 0,
	CAN_TRANSMIT_ERROR_MAILBOX_1,
	CAN_TRANSMIT_ERROR_MAILBOX_2,
	NUM_CAN_TRANSMIT_MAILBOXES,
} canTransmitMailbox_t;

#ifdef COMPILE_FOR_SYSTEM
#define NUM_CAN_HW_UNITS    2
#else
#define NUM_CAN_HW_UNITS    1
#endif


#define CAN_READ                    0b00        // 0x00
#define CAN_WRITE                   0b01        // 0x01
#define CAN_RETURN_DATA             0b10        // 0x02
#define CAN_WRITE_ACK               0b11        // 0x03

#define NO_PAGE                     ((byte)0)

#define CAN_BYTES_0                 ((byte)0)
#define CAN_BYTES_1                 ((byte)1)
#define CAN_BYTES_2                 ((byte)2)
#define CAN_BYTES_3                 ((byte)3)
#define CAN_BYTES_4                 ((byte)4)
#define CAN_BYTES_5                 ((byte)5)
#define CAN_BYTES_6                 ((byte)6)
#define CAN_BYTES_7                 ((byte)7)
#define CAN_BYTES_8                 ((byte)8)
#define CAN_BYTES_FF                ((byte)0xff)

// WARNING:  IF A NEW COMMAND IS ADDED, UPDATE "canMsgIdToNumBytes" in hyrel_can.c

#define CAN_MSG_DEVICE_POSITION             ((byte)0x00)
#define CAN_MSG_DEVICE_INFO                 ((byte)0x01)
#define CAN_MSG_FLASH_CONFIG                ((byte)0x02)
#define CAN_MSG_UNIQUE_ID                   ((byte)0x03)
#define CAN_MSG_PRE_DEFINED_ALIASES         ((byte)0x04)
#define CAN_MSG_USER_DEFINED_ALIASES        ((byte)0x05)
#define CAN_MSG_STATUS                      ((byte)0x06)
//#define CAN_MSG_HISTORY_RANGE_DEF         ((byte)0x07)    //NUKE DEPRECATE
//#define CAN_MSG_HISTORY_RANGES                ((byte)0x08)    //NUKE DEPRECATE
//#define CAN_MSG_HISTORY_COUNTERS          ((byte)0x09)    //NUKE DEPRECATE
#define CAN_MSG_PAGE_DEF                    ((byte)0x0a)
//#define CAN_MSG_TABLE_START_OFFSETS           ((byte)0x0b)    // not used in V1 -- tables are not loadable
//#define CAN_MSG_GLOBAL_STRUCT_INFO            ((byte)0x0c)    //NUKE DEPRECATE
#define CAN_MSG_DEVICE_INIT                 ((byte)0x0d)    // V1 - multi-page initialization based on deviceType

#define CAN_MSG_CONTROL_WORD                ((byte)0x10)
#define CAN_MSG_ADD_ALIAS                   ((byte)0x11)
#define CAN_MSG_REMOVE_ALIAS                ((byte)0x12)
#define CAN_MSG_AUTO_STATUS_CONTROL         ((byte)0x13)
//#define CAN_MSG_LED_CONTROL                   ((byte)0x14)    //NUKE DEPRECATE
#ifdef HYDRA_DIAGS
#define CAN_MSG_LED_OVERRIDE                ((byte)0x15)    //used by DIAGS
#endif //HYDRA_DIAGS
//#define CAN_MSG_EXTREMES                  ((byte)0x16)    // V0
#define CAN_MSG_EXTRUSION_TEMP_RANGES       ((byte)0x16)    // V1
//#define CAN_MSG_ERROR_REPORT_INTERVAL     ((byte)0x17)    //NUKE DEPRECATE
//#define CAN_MSG_SWITCH_DUTY                   ((byte)0x18)    // V0 only
//#define CAN_MSG_SWITCH_PWM                    ((byte)0x19)    // V0 only
//#define CAN_MSG_SWITCH_TEMP                   ((byte)0x1a)    // V0 only
#define CAN_MSG_KARL_FACTORS                ((byte)0x1b)
#define CAN_MSG_HSS_CONTROL                 ((byte)0x1c)    // V1
//#define CAN_MSG_HSS_FLAGS                 ((byte)0x1d)    // V1
#define CAN_MSG_MOTION_CONTROLS             ((byte)0x1e)

#define CAN_MSG_MOTOR_ENABLE                ((byte)0x20)    //NUKE DEPRICATED (USED IN V4, remove for V5) LEGACY
//#define CAN_MSG_MICROSTEPS                  ((byte)0x21)    // aliased to CAN_MSG_JET_INDEX - not used in V1
//#define CAN_MSG_JET_INDEX                   ((byte)0x21)    // aliased to CAN_MSG_MICROSTEPS - not used in V1
//#define CAN_MSG_CURRENT_BOOST             ((byte)0x22)    // not used in v1
#define CAN_MSG_FLOW_SCALE_FACTORS          ((byte)0x23)    // was CAN_MSG_fudge_factor
//#define CAN_MSG_EXTRUSION_STEPS_PER_NL        ((byte)0x24)    //NUKE DEPRICATED
//#define CAN_MSG_EXTRUSION_MAIN                ((byte)0x25)    // V0 only
#define CAN_MSG_EXTRUSION_UNPRIME           ((byte)0x26)
#define CAN_MSG_EXTRUSION_PRIME             ((byte)0x27)
//#define CAN_MSG_EXTRUSION_MANUAL          ((byte)0x28)    // V0 only
#define CAN_MSG_EXTRUSION_DWELL             ((byte)0x29)    //NUKE DEPRICATED
#define CAN_MSG_PRIME_AND_RUN_PARAMS        ((byte)0x2a)
//NUKE #define CAN_MSG_VELOCITY_SCALE_FACTOR          ((byte)0x2b)    //NUKE DEPRICATED
#define CAN_MSG_TOUCH_PROBE_OLD                ((byte)0x2c) // aliased with CAN_MSG_PICKNPLACE_CONTROLTOUCH PROBE -- BOO HISS LEGACY....
#define CAN_MSG_PICKNPLACE_CONTROL          ((byte)0x2c)
#define CAN_MSG_PICKNPLACE_DATA             ((byte)0x2d)
//#define CAN_MSG_CONTROL_VIA_PAGE            ((byte)0x2e)  // V1
#define CAN_MSG_TOUCH_PROBE                 ((byte)0x2f)


#define CAN_MSG_FILL_BUFFER                 ((byte)0x30)
#define CAN_MSG_ACCESS_BUFFER               ((byte)0x31)
//NUKE #define CAN_MSG_COPY_ADDR_TO_BUFFER          ((byte)0x32)
#define CAN_MSG_COPY_PAGE_TO_BUFFER         ((byte)0x33)
#define CAN_MSG_COPY_BUFFER_TO_PAGE         ((byte)0x34)
#define CAN_MSG_PAGE_CHECKSUM               ((byte)0x35)
//#define CAN_MSG_ACCESS_OPTION_BYTES           ((byte)0x36) //NUKE DEPRECATE
#define CAN_MSG_START_PRIMARY_PROGRAM       ((byte)0x37)
//#define CAN_MSG_READ_DEVICE_MEMORY            ((byte)0x38)    //NUKE DEPRECATE        //reads the device memory at location argument[0] for up to argument[1] words

#define CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL	((byte)0x3a)

#define CAN_MSG_EVENT_MESSAGE               ((byte)0x40)
#define CAN_MSG_LOOPBACK                    ((byte)0x41)
#define CAN_MSG_REPORT_ERROR                ((byte)0x42)
//#define CAN_MSG_AUTO_STATUS                   ((byte)0x43)    //NUKE DEPRECATE
#define CAN_MSG_PRIME_RUN                   ((byte)0x44)
#define CAN_MSG_UNPRIME                     ((byte)0x45)
#define CAN_MSG_PRIME                       ((byte)0x46)
#define CAN_MSG_RUN                         ((byte)0x47)
#define CAN_MSG_STOP                        ((byte)0x48)
// all 0x2a are aliased and must have the same number of data bytes
//#define CAN_MSG_V0_HEAD_CONTROL             ((byte)0x49)  // V0
//#define CAN_MSG_SET_LASER_POWER               ((byte)0x49)    // V0
//#define CAN_MSG_SPINDLE_CONTROL               ((byte)0x49)    // V0
// all 0x2a are aliased and must have the same number of data bytes
#define CAN_MSG_V1_HEAD_FUNCTION_CONTROL    ((byte)0x49)    // V1
#define CAN_MSG_HEAD_EXTRUDE_CONTROL        ((byte)0x49)    // V1
#define CAN_MSG_HEAD_LASER_CONTROL          ((byte)0x49)    // V1
#define CAN_MSG_HEAD_INKJET_CONTROL         ((byte)0x49)    // V1
#define CAN_MSG_HEAD_SPINDLE_CONTROL        ((byte)0x49)    // V1

#define CAN_MSG_SET_POWER_LEVEL_ONLY        ((byte)0x4a)    // for laser and inkjet raster modes
#define CAN_MSG_STEP_MOTOR                  ((byte)0x4b)
#define CAN_MSG_STRING                      ((byte)0x4c)
#define CAN_MSG_DISPLAY_CONTROL             ((byte)0x4d)

#ifdef HYDRA_DIAGS
#define CAN_MSG_DIAG_IO_INIT                ((byte)0x50)    // 8 bytes; page 0xff
#define CAN_MSG_DIAG_IO_WRITE               ((byte)0x51)    // 1 byte;  page 0xff
#define CAN_MSG_DIAG_IO_READ                ((byte)0x52)    // 1 byte;  page 0xff
#define CAN_MSG_DIAG_ADC_READ               ((byte)0x53)    // 2 bytes; page 0xff
#define CAN_MSG_DIAG_STEP_COUNTER           ((byte)0x54)    // 8 bytes; page 0
#endif

//#define CAN_MSG_fudge_factor              ((byte)0x23)
//#define CAN_MSG_Send_PRIMEandRun_Prams    ((byte)0x2a)
//#define CAN_Prime_Run                     ((byte)0x44)    // V0
//#define CAN_Unprime                           ((byte)0x45)    // V0
//#define CAN_Prime                         ((byte)0x46)    // V0
//#define CAN_Run                               ((byte)0x47)    // V0
//#define CAN_Stop                          ((byte)0x48)    // V0
//#define CAN_SetLaserPower                 ((byte)0x49)    // V0
//#define CAN_TurnOnSpindle                 ((byte)0x49)    // V0
//NUKE #define CAN_PICKNPLACE_CONTROL            ((byte)0x49)   // V0

// options (page value) for CAN_MSG_CONTROL_VIA_PAGE
//typedef enum {
	//PAGE_CONTROL___________     = 0,  // spare
	//PAGE_CONTROL_PRIME_RUN      = 1,
	//PAGE_CONTROL_UNPRIME        = 2,
	//PAGE_CONTROL_PRIME          = 3,
	//PAGE_CONTROL_RUN            = 4,
	//PAGE_CONTROL_STOP           = 5,
	//PAGE_CONTROL_STEP_MOTOR     = 6,
	//PAGE_CONTROL_MOTOR_ENABLE   = 7,
	//PAGE_CONTROL_MOTOR_DISABLE  = 8,
	//PAGE_CONTROL_INITIALIZE     = 9,  // V1 tell device to complete initialization process
//} page_control_t;


//NUKE #define HH_PAGE_MOTION_CONTROL_MANUAL 0
//NUKE #define HH_PAGE_MOTION_CONTROL_NORMAL 1

#define MAX_CAN_STRING_SIZE     65 // includes NULL char;

#define CLOSED_LOOP_MOTOR_CALIBRATE					1
#define CLOSED_LOOP_MOTOR_SAVE_CONFIG				2
#define CLOSED_LOOP_MOTOR_RESET_ALL					3
#define CLOSED_LOOP_MOTOR_RESTORE_DEFAULTS			4
#define CLOSED_LOOP_MOTOR_RESET_POSITION			5
#define CLOSED_LOOP_MOTOR_SIDE_STEP					6
#define CLOSED_LOOP_MOTOR_HOME_TO_LIMIT				7
#define CLOSED_LOOP_MOTOR_HOME_BY_FORCE				8
#define CLOSED_LOOP_LATHE_MODE						9
#define CLOSED_LOOP_SERVO_INIT_BY_TEACHING			10
#define CLOSED_LOOP_SERVO_INIT_BY_CURR_MIN_MAX		11
#define CLOSED_LOOP_SERVO_INIT_BY_FORCE				12
#define CLOSED_LOOP_SERVO_RUN						13
#define CLOSED_LOOP_SERVO_STOP						14

typedef enum {
	CAN_EVENT_DEVICE_ANNOUNCE           = 0,
	CAN_EVENT_BOOTLOADER_ANNOUNCE       = 1,
	CAN_EVENT_DEVICE_HEARTBEAT_0        = 2,
	CAN_EVENT_MANUAL_Z_MOVE             = 3,
	CAN_EVENT_PRIME_COMPLETE            = 4,
	CAN_EVENT_LIMIT_SWITCH_ON           = 5,
	CAN_EVENT_LIMIT_SWITCH_OFF          = 6,
	CAN_EVENT_PROBE_REPORT              = 7,
	CAN_EVENT_PNP_DOWN_REPORT           = 8,
	CAN_EVENT_PNP_UP_REPORT             = 9,
	CAN_EVENT_PNP_MOTION_COMPLETE       = 10,
	CAN_EVENT_PNP_MOTION_BLOCKED        = 11,
	CAN_EVENT_PROBE_ARMED               = 12,
	CAN_EVENT_PROBE_CONTACT_BEFORE_MOVE = 13,
	CAN_EVENT_CO2_WATCHDOG_EXPIRED      = 14,
	CAN_EVENT_CO2_PRIME_FINISHED_START_RUN = 15,
	CAN_EVENT_MOTION_LIMIT1_STATE_CHANGE= 16,
	CAN_EVENT_MOTION_LIMIT2_STATE_CHANGE= 17,
	CAN_EVENT_ENCODER_CALIBRATION_PASS	= 18,
	CAN_EVENT_ENCODER_CALIBRATION_FAIL	= 19,
	CAN_EVENT_ENCODER_CONFIG_SAVE_PASS	= 20,
	CAN_EVENT_ENCODER_CONFIG_SAVE_FAIL	= 21,
	CAN_EVENT_ENCODER_BAD_CALIBRATION	= 22,
	CAN_EVENT_ENCODER_CALIBRATING		= 23,
	CAN_EVENT_MOTION_HOMING_IN_PROCESS  = 24,
	CAN_EVENT_MOTION_HOMING_COMPLETE	= 25,
	CAN_EVENT_MOTION_JOGGING_COMPLETE	= 26,
	CAN_EVENT_MOTION_LATHE_COMPLETE		= 27,
	CAN_EVENT_DEVICE_HEARTBEAT_1        = 28,
	CAN_EVENT_DEVICE_HEARTBEAT_2        = 29,
	CAN_EVENT_DEVICE_HEARTBEAT_3        = 30,
	CAN_EVENT_DEVICE_HEARTBEAT_4        = 31,
	CAN_EVENT_DEVICE_HEARTBEAT_5        = 32,
	CAN_EVENT_DEVICE_HEARTBEAT_6        = 33,
	CAN_EVENT_DEVICE_HEARTBEAT_7        = 34,
	CAN_EVENT_DEVICE_HEARTBEAT_8        = 35,
	CAN_EVENT_DEVICE_HEARTBEAT_9        = 36,
	CAN_EVENT_DEVICE_WIPE_SAVED_SETTINGS = 37,
} canEventMsgType_t;

typedef enum {
	PAGE_DATA_TO_HOST               = 0,
	PAGE_DATA_TO_DEVICE             = 1,
	PAGE_DATA_COMMITTED             = 2,
	STARTING_BOOTLOADER             = 3,
	DEVICE_INFO_TO_HOST             = 4,
	FINISH_OPTION_BYTES             = 5,

	CLEARING_PAGES_PART1            = 10,       // CLEARING_PAGES_PARTx must remain sequential
	CLEARING_PAGES_PART2            = 11,
	CLEARING_PAGES_PART3            = 12,
	CLEARING_PAGES_PART4            = 13,

	DEVICE_CODE_CHECKSUM_PART1      = 20,       // DEVICE_CODE_CHECKSUM_PARTx must remain sequential
	DEVICE_CODE_CHECKSUM_PART2      = 21,
	DEVICE_CODE_CHECKSUM_PART3      = 22,
	DEVICE_CODE_CHECKSUM_PART4      = 23,
	DEVICE_CODE_CHECKSUM_PART5      = 24,
	DEVICE_CODE_CHECKSUM_PART6      = 25,

	TRANSFER_PAGE_PART1             = 30,       // TRANSFER_PAGE_PARTx must remain sequential
	TRANSFER_PAGE_PART2             = 31,
	TRANSFER_PAGE_PART3             = 32,
	TRANSFER_PAGE_PART4             = 33,
	TRANSFER_PAGE_PART5             = 34,
	TRANSFER_PAGE_PART6             = 35,

	DEVICE_INITIALIZATION           = 36,

	GUI_CMD_PROCESSED               = 37,		// cheating... really using this as an ACK, but since not being used, this was the easiest way to slip in to the code
} loopbackType;

typedef enum {
	STATUS_PAGE_HEATER_TEMP             = 0,
	STATUS_PAGE_MOTOR_TEMP              = 1,
	STATUS_PAGE_SWITCH_FAN              = 2,
	STATUS_PAGE_SWITCH_HEATER           = 3,
	STATUS_PAGE_LIMITS                  = 4,
	STATUS_PAGE_ADC_INFO_POSITION       = 5,
	STATUS_PAGE_ADC_INFO_RTD1           = 6,
	STATUS_PAGE_ADC_INFO_RTD2           = 7,
	STATUS_PAGE_ADC_INFO_RTD3           = 8,
	STATUS_PAGE_ADC_INFO_TEMP_SENSOR    = 9,
	STATUS_PAGE_ADC_RAW0_POSITION       = 10,
	STATUS_PAGE_ADC_RAW0_RTD1           = 11,
	STATUS_PAGE_ADC_RAW0_RTD2           = 12,
	STATUS_PAGE_ADC_RAW0_RTD3           = 13,
	STATUS_PAGE_ADC_RAW0_TEMP_SENSOR    = 14,
	STATUS_PAGE_ADC_RAW1_POSITION       = 15,
	STATUS_PAGE_ADC_RAW1_RTD1           = 16,
	STATUS_PAGE_ADC_RAW1_RTD2           = 17,
	STATUS_PAGE_ADC_RAW1_RTD3           = 18,
	STATUS_PAGE_ADC_RAW1_TEMP_SENSOR    = 19
} statusPageType_t;

typedef enum {
	GUI_CMD_DISPLAY_OFF             	= 0,
	GUI_CMD_DISPLAY_ON              	= 1,
	GUI_CMD_DISPLAY_INVERT          	= 2,
	GUI_CMD_DISPLAY_CLEAR           	= 3,
	GUI_CMD_DISPLAY_ROTATION        	= 4,
	GUI_CMD_SELECT_PANEL            	= 5,
	GUI_CMD_SELECT_FONT             	= 6,
	GUI_CMD_SET_PAGE                	= 7,
	GUI_CMD_SET_REFRESH_INTERVAL    	= 8,
	GUI_CMD_SELECT_THEME				= 9,
	GUI_CMD_FORCE_REDRAW				= 10,
	GUI_CMD_DISPLAY_BRIGHTNESS      	= 11,
	GUI_CMD_FAKE_LEDS_ENABLE        	= 12,

	GUI_CMD_SET_BORDER_WIDTH        	= 20,
	GUI_CMD_SET_COLOR_BORDER        	= 21,
	GUI_CMD_SET_COLOR_FILL          	= 22,
	GUI_CMD_SET_COLOR_FONT          	= 23,
	GUI_CMD_SET_COLOR_THEME_LABEL   	= 24,
	GUI_CMD_SET_COLOR_THEME_VALUE   	= 25,
	GUI_CMD_SET_COLOR_THEME_BG      	= 26,
	GUI_CMD_SET_CHROMA_KEY          	= 27,

	GUI_CMD_DRAW_PIXEL              	= 40,
	GUI_CMD_DRAW_PIXEL_W_COLOR      	= 41,
	GUI_CMD_DRAW_HORIZ_LINE         	= 42,
	GUI_CMD_DRAW_VERT_LINE          	= 43,
	GUI_CMD_DRAW_LINE               	= 44,
	GUI_CMD_DRAW_RECT               	= 45,
	GUI_CMD_FILL_RECT               	= 46,
	GUI_CMD_DRAW_FILL_RECT          	= 47,
	GUI_CMD_DRAW_CIRC               	= 48,
	GUI_CMD_FILL_CIRC               	= 49,
	GUI_CMD_DRAW_FILL_CIRC          	= 50,
	GUI_CMD_DRAW_CHAR               	= 51,
	GUI_CMD_DRAW_STRING             	= 52,
	GUI_CMD_DRAW_INDEXED_VARIABLE		= 53,

	GUI_CMD_START_NEW_SCREEN        	= 60,
	GUI_CMD_ADD_SCREEN_ENTRY        	= 61,

	GUI_CMD_IMAGE_REGION            	= 70,
	GUI_CMD_IMAGE_COLOR_16_BIT      	= 71,		// 4, 16-bit colors RGB565
	GUI_CMD_IMAGE_STR_COLOR_16_BIT  	= 72,		// 4, 16-bit colors RGB565
	GUI_CMD_IMAGE_STR_COLOR_8_BIT   	= 73,  		// 8, 8-bit colors RGB332
	GUI_CMD_IMAGE_STR_COLOR_4_BIT   	= 74,  		// 16, 4-bit colors RGB121
	GUI_CMD_IMAGE_STR_INDEX_8_BIT   	= 75, 		// 8, 8-bit color index into LUT
	GUI_CMD_IMAGE_STR_INDEX_4_BIT   	= 76, 		// 16, 4-bit color index into LUT
	GUI_CMD_IMAGE_STR_INDEX_1_BIT   	= 77,  		// 64, 1-bit colors (0)BG and (1)FG

	GUI_CMD_SET_PANEL_OFS_XY        	= 99,
} GUI_canCommand_t;

#define GUI_THEME_DEFAULT						0
#define GUI_THEME_BLACK_AND_BLUE_ON_WHITE		1
#define GUI_THEME_WHITE_AND_YELLOW_ON_BLACK		2


#define CAN_NUM_FILTERS                         14
#define NUM_PRE_DEFINED_ALIASES                 4
#define NUM_USER_DEFINED_ALIASES                8   // note, if this increases past 8, change  canbus.c are needed to
													// support more than 1 pg
													// also filters can at most support 9
#define ALIAS_PHYSICAL_FILTER_INDEX             0
#define ALIAS_BROADCAST_ALL_FILTER_INDEX        1
#define ALIAS_BROADCAST_TYPE_FILTER_INDEX       2
#define ALIAS_BROADCAST_YOKE_FILTER_INDEX       3
#define ALIAS_USER_DEFINED_START_FILTER_INDEX   4
#define ALIAS_IMMEDIATE_FILTER_INDEX            13

#define ALIAS_ALL_DEVICES                   0
#define ALIAS_ALL_YOKE1_DEVICES             10
#define ALIAS_ALL_YOKE2_DEVICES             20
#define ALIAS_ALL_YOKE3_DEVICES             30
#define ALIAS_ALL_YOKE4_DEVICES             40
#define ALIAS_ALL_CAN_AXIS_MOTORS			80
#define ALIAS_ALL_CAN_AXIS_MOTORS_X			81
#define ALIAS_ALL_CAN_AXIS_MOTORS_Y			82
#define ALIAS_ALL_CAN_AXIS_MOTORS_Z			83
#define ALIAS_ALL_CAN_AXIS_MOTORS_A			84
#define ALIAS_ALL_CAN_AXIS_MOTORS_B			85
#define ALIAS_ALL_CAN_AXIS_MOTORS_C			86

#define ALIAS_ALL_HOTBEDS                   90
#define ALIAS_ALL_EXTRUDERS                 100
#define ALIAS_UNUSED                        0xff

#define ALIAS_TOOL_NUMBER_MIN               50
#define ALIAS_TOOL_NUMBER_MAX               57

#define CAN_AXIS_MOTOR_MIN_DEVICE_POSITION		60
#define CAN_AXIS_MOTOR_MAX_DEVICE_POSITION		CAN_AXIS_MOTOR_MIN_DEVICE_POSITION + 0xe


#define FILTER_CONSTANT_VALUE           0x80000004
#define FILTER_MASK_CONSTANT_HEAD       0xE0040004
#define FILTER_MASK_CONSTANT_SYS        0xE0000004
#define FILTER_MASK_DEVICE              0x07F80000
#define FILTER_IMMEDIATE_BIT            0x00040000
#define FILTER_DEVICE_LSB_POSITION      19

extern uint32_t calculateChecksum8(uint8_t [], uint32_t);
extern uint32_t calculateChecksum16(uint16_t [], uint32_t);
extern uint32_t calculateChecksum32(uint32_t [], uint32_t);
extern void canInitFilter(CAN_TypeDef *, byte, byte, boolean, boolean);
extern boolean canIsValidAlias(byte device);
extern void canAddUserDefinedAlias(byte, byte []);
extern void canRemoveUserDefinedAlias(byte, byte []);
extern canStruct *canGetTxQueueNextInPtr(void);
extern byte WriteExtrusionCommandtoTxQue(byte, byte, uint16_t, uint16_t, uint16_t, uint16_t);//formats and sends canbus message
extern byte CanWriteSimplePacket(byte, byte, byte, byte, boolean);
extern byte canPackIntoTxQueueNoData(byte, byte, byte, byte, boolean);
extern byte canPackIntoTxQueue1x32(byte, byte, byte, byte, boolean, uint32_t);
extern byte canPackIntoTxQueue2x32(byte, byte, byte, byte, boolean, uint32_t, uint32_t);
extern byte canPackIntoTxQueue1x32_2x16(byte, byte, byte, byte, boolean, uint32_t, uint16_t, uint16_t);
extern byte canPackIntoTxQueue1x32_1x16(byte, byte, byte, byte, boolean, uint32_t, uint16_t);
extern byte canPackIntoTxQueue1x16(byte, byte, byte, byte, boolean, uint16_t);
extern byte canPackIntoTxQueue2x16(byte, byte, byte, byte, boolean, uint16_t, uint16_t);
extern byte canPackIntoTxQueue3x16(byte, byte, byte, byte, boolean, uint16_t, uint16_t, uint16_t);
extern byte canPackIntoTxQueue4x16(byte, byte, byte, byte, boolean, uint16_t, uint16_t, uint16_t, uint16_t);
extern byte canPackIntoTxQueue1x8(byte, byte, byte, byte, boolean, byte);
extern byte canPackIntoTxQueue2x8(byte, byte, byte, byte, boolean, byte, byte);
extern byte canPackIntoTxQueue8x8(byte, byte, byte, byte, boolean, byte []);
extern byte canDevicePackWriteIntoTxQueue2x32(byte, byte, uint32_t, uint32_t);
extern byte canDevicePackEventIntoTxQueue1x32(byte, uint32_t);

extern byte canAddToTxQueue(canSwStruct *);
extern byte canProcessTxQueue(void);
extern void canProcessTxQueueUntilEmpty(void);
extern byte canTransmit(CAN_TypeDef *, canHwStruct *);
extern void canGetMailboxData(CAN_FIFOMailBox_TypeDef *, canHwStruct *);
extern byte canReceive(CAN_TypeDef *, uint8_t, canHwStruct *);
extern byte canAddToImmediateRxQueue(void);
extern byte canAddToRxQueue(void);
extern void canUnpackCanHwStruct(canStruct *, canSwStruct *);
extern boolean canPrepNextRx(void);
extern void canSendString(byte device, char str[]);
extern int canSendStringToWorkingBufferOnDevice(byte device, char str[]);
extern int canSendAsciiHexStringAsBinaryToWorkingBufferOnDevice(byte device, char str[]);
extern boolean canReceiveString(canSwStruct *canRx, int *charsReceived, char *receiveString);
extern void canAddToRxQueueNoReturn(void);
extern void canProcessRxQueueNoReturn(void);
extern void canProcessTxQueueNoReturn(void);
extern void canProcessRxAndTxQueues(void);
extern void canFillOutCanStruct(canSwStruct *, canbusFormat_t);

////////////////////////////////////////////////////////////////////////////////
#endif // #ifndef hyrel_can_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE

