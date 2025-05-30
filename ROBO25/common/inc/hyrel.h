#ifndef hyrel_HEADER // prevent double dipping
#define hyrel_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:            hyrel.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose:         include file for common public defines and functions.
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#define CAN_500KBPS
//#define CAN_1000KBPS

#ifdef CAN_500KBPS
#define CANBUS_BIT_RATE 500000.0f
#elif defined(CAN_1000KBPS)
#define CANBUS_BIT_RATE 1000000.0f
#else
#define CANBUS_BIT_RATE 1.0f    // unknown
#warning "unknown canbus bit rate"
#endif

#define CHECKSUM_KEY 0x5555aaaa

typedef uint8_t byte;

typedef void (*PFUNC)(void);

typedef int16_t temp_t; // temperature (s10.5 format - 1/32 degree resolution)

typedef uint32_t pinType;

typedef enum {
	FALSE   = 0,
	TRUE    = 1
} boolean;

typedef enum {
   OFF     = 0,
   ON      = 1,
} offOn_t;

typedef enum {
	PASS  = 0,
	FAIL  = 1
} passFail_t;

typedef enum {
	LOW     = 0,
	HIGH    = 1,
	UNKNOWN_PIN_STATE = 2,
} pinStateValue_t;

typedef enum {
	DEASSERTED  = 0,
	ASSERTED    = 1,
} assertValue_t;

typedef enum {
	ACTIVE_LOW  = 0,
	ACTIVE_HIGH = 1,
	ACTIVE_DISABLED = 2,
} polarity_t;

typedef enum {
	SENSOR_OPEN     = 0,
	SENSOR_TRIPPED  = 1,
	SENSOR_CLOSED   = 1,         // make TRIPPED and CLOSED the same thing
	SENSOR_STATE_UNKNOWN
} sensorState_t;

typedef enum {
	CONTROL_OPEN_LOOP	= 0,
	CONTROL_BTT_PID		= 1,	//BTT - emulates big tree tech
	CONTROL_STP_PID		= 2,	//basic tracking steps (preferred for 405 heads; matches 3200 ustep/rev
	CONTROL_ANG_PID		= 3,	//target angle
	CONTROL_RPM_PID		= 4,	//target velocity
	CONTROL_EXP_PID		= 5,	//experimental
	CONTROL_SP0_PID		= 6,	//spare placeholder in flash
	CONTROL_SP1_PID		= 7,	//spare placeholder in flash
	CONTROL_NUM_PIDS
} motorControl_t;

typedef union {
	uint32_t u32;
	struct {
		unsigned encoder : 1;
		unsigned advance : 1;
		unsigned software : 1;
	} b;
} prediction_t;

typedef enum {
	SYSTEM_HYREL = 1,           // 30M, EngineSR
	SYSTEM_HYDRA = 2,           // 16A, EngineHR
	SYSTEM_MEGASONIC = 3,       // Megasonic machine (not a printer)
} system_t;

typedef enum {
	IN_FLOAT    = 0,
	IN_ANALOG   = 1,
	IN_PULLDOWN = 2,
	IN_PULLUP   = 3,
	OUT_0       = 4,
	OUT_1       = 5
} boostCtrl_t;

#define HH_MOTOR_DIRECTION_MASK 0x01
typedef enum {
	HH_MOTOR_REVERSE    = 0,
	HH_MOTOR_FORWARD    = 1,
} hhMotorDir_t;

typedef enum {
	MOTOR_DISABLED_STATE    = 0,
	MOTOR_ENABLED_STATE     = 1,
} motorEn_t;

#define WORKING_BUFFER_SIZE_IN_BYTES             1024
#define WORKING_BUFFER_SIZE_IN_WORDS             (WORKING_BUFFER_SIZE_IN_BYTES/4)

#define MAXINT  ((int)2147483647)   // 2^31-1 0x7fff_ffff
#define MININT  ((int)(-2147483647))  // -2^31  0x8000_0000

#define PI  3.141592654f
#define TWO_PI ((float)(2.0f * PI))                     // compiler will replace with actual answer
#define RADIANS_TO_DEGREES ((float)(360.0f / TWO_PI))   // compiler will replace with actual answer
#define DEGREES_TO_RADIANS ((float)(TWO_PI / 360.0f))   // compiler will replace with actual answer
#define TWO_PI_OVER_360    ((float)(TWO_PI / 360.0f))   // compiler will replace with actual answer
#define PI_OVER_TWO        ((float)(PI / 2.0f))         // compiler will replace with actual answer
#define MINUS_PI_OVER_TWO  ((float)(PI / -2.0f))        // compiler will replace with actual answer

#define HH_FORCE_FLAG_OPEN_LOOP				0x01
#define HH_FORCE_FLAG_MOTOR_DISABLED		0x02
#define HH_FORCE_CALIBRATION_INVALID		0x04

#define HH_COMM_WATCHDOG_START_VALUE 10
#define MOTION_TO_HH_CHECKIN_RATE 1
#define HH_NUM_HISTORY_COUNTERS 16
#define HH_SCALE_FACTOR_FRAC_BITS            10  // scale factors are resent in a fixed-point format with this many fractional bits
#define DEFAULT_LASER_COOLDOWN_TIME   30  // seconds of cooldown time for LASER

#define HH_U16_POWER_PCT_FRAC_BITS		15
#define HH_U8_POWER_PCT_FRAC_BITS		7

// legacy: for pre 4.003 medusa
typedef enum {
	TABLE_PAGE_INDEX            = 0,
	SOAPBOX_PAGE_INDEX          = 1,
	HH_NUM_SPECIAL_PAGES_V1,                            // will auto set to the correct number
	HISTORY0_PAGE_INDEX = HH_NUM_SPECIAL_PAGES_V1,      // will auto set to the correct number
	HISTORY1_PAGE_INDEX,                                // will auto set to the correct number
	HH_NUM_SPECIAL_PAGES_V0                             // will auto set to the correct number
} pageIndexType_t;

////////////////////////////////////////////////////////////////////////////////
// DEVICE INIT RELATED /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// using page field on the canbus to communicate to the head where the init data is
// destined.
//     the upper 3 bits specify an 'area' of interest (motor, switch0, adcs, etc)
//     the lower 5 bits are an index for specify values in that area

typedef enum {
	HX711_GAIN_DEFAULT 	= 0,
	HX711_GAIN_128_CH_A	= 1,
	HX711_GAIN_64_CH_A	= 2,
	HX711_GAIN_32_CB_B	= 3,
} hx711Gain_t;

typedef enum {
	HUMITURE_DEFAULT 		= 0,
	HUMITURE_DHT22_AM2302	= 1,
	HUMITURE_DHT11_AM2301	= 2,
} humiture_t;

typedef union {
	byte u8;
	struct {
		unsigned index : 5;
		unsigned area : 3;
	} b;
} devPageFields_t;

typedef enum {
	DEV_INIT_AREA_DEV       = 0,
	DEV_INIT_AREA_MTR,
	DEV_INIT_AREA_SW0,
	DEV_INIT_AREA_SW1,
	DEV_INIT_AREA_ADD,
// QQQQ ADD NEW AREA(S) HERE (UP to 8 total)
} devInitArea_t;

typedef enum {
	DEV_INIT_INDEX_DEV_SYSTEM           = 0,
	DEV_INIT_INDEX_DEV_PWM_LIMIT,
	DEV_INIT_INDEX_DEV_INKJET_NOZZLE,
	DEV_INIT_INDEX_DEV_COOLDOWN,
	DEV_INIT_INDEX_DEV_VALID_COMBO,
	DEV_INIT_INDEX_DEV_ADD_ON_MODULE,

	DEV_INIT_INDEX_MTR_MANUAL_JO               = 0,
	DEV_INIT_INDEX_MTR_MANUAL_ACCEL,
	DEV_INIT_INDEX_MTR_MANUAL_MAX_RATE,
	DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP,
	DEV_INIT_INDEX_MTR_CURRENT_BOOST,
	DEV_INIT_INDEX_MTR_MANUAL_PERIOD_MS,
	// NEW
	DEV_INIT_INDEX_MTR_CONTROL_METHOD,
	DEV_INIT_INDEX_MTR_FULL_STEPS_PER_REV,
	DEV_INIT_INDEX_MTR_MICROSTEPS_PER_REV,
	DEV_INIT_INDEX_MTR_CURRENT_MAX_PCT,
	PLACEHOLDER_DEV_INIT_INDEX_MTR_DEFAULT_DIRECTION,
	PLACEHOLDER_DEV_INIT_INDEX_MTR_STEP_POLARITY,
	DEV_INIT_INDEX_MTR_LIMIT1_POLARITY,
	DEV_INIT_INDEX_MTR_LIMIT2_POLARITY,
	DEV_INIT_INDEX_MTR_PID_KP,
	DEV_INIT_INDEX_MTR_PID_KI,
	DEV_INIT_INDEX_MTR_PID_KD,
	DEV_INIT_INDEX_MTR_CURRENT_MIN_PCT,
	DEV_INIT_INDEX_MTR_PID_ADV,
	DEV_INIT_INDEX_MTR_ENCODER_PREDICTION,
	DEV_INIT_INDEX_MTR_ENCODER_AUTO_CAL,
	DEV_INIT_INDEX_MTR_ANGLE_ERROR_LIMIT,
	DEV_INIT_INDEX_MTR_MOTION_JO,
	DEV_INIT_INDEX_MTR_MOTION_ACCEL,
	DEV_INIT_INDEX_MTR_MOTION_MAX_RATE,
	DEV_INIT_INDEX_MTR_SAVE_CONFIGURATION,
	DEV_INIT_INDEX_MTR_SAVE_CALIBRATION,

	DEV_INIT_INDEX_SWx_FLAGS            = 0,
	DEV_INIT_INDEX_SWx_MIN_TEMP,
	DEV_INIT_INDEX_SWx_MAX_TEMP,
	DEV_INIT_INDEX_SWx_MAX_DC,
	DEV_INIT_INDEX_SWx_KF_PF,
	DEV_INIT_INDEX_SWx_KF_BIAS,
	DEV_INIT_INDEX_SWx_KF_OVERSHOOT,
	DEV_INIT_INDEX_SWx_KF_DENOM,
	DEV_INIT_INDEX_SWx_KF_FAN_DC,

	DEV_INIT_INDEX_ADD_AM23XX_TYPE = 0,
	DEV_INIT_INDEX_ADD_AM23XX_CALIB_SET_POINT,
	DEV_INIT_INDEX_ADD_AM23XX_CALIB_OFFSET,
	DEV_INIT_INDEX_ADD_AM23XX_CALIB_SCALE,
	DEV_INIT_INDEX_ADD_AM23XX_SAVE_CONFIGURATION,
	DEV_INIT_INDEX_ADD_AM23XX_SAVE_CALIBRATION,

	DEV_INIT_INDEX_ADD_HX711_REPORT_MODE,
	DEV_INIT_INDEX_ADD_HX711_TARE,
	DEV_INIT_INDEX_ADD_HX711_GAIN,	// gain/channel selector - could be "TYPE"
	DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_SET_POINT,
	DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_TEMP,
	DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_ZERO_OFFSET,
	DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_GRAMS_PER_TICK,
	DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_ZERO_OFS_PER_DEG,
	DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_GRAMS_PER_TICK_PER_DEG,
	DEV_INIT_INDEX_ADD_HX711_SAVE_CONFIGURATION,
	DEV_INIT_INDEX_ADD_HX711_SAVE_CALIBRATION,
} devInitIndex_t;

#define getDevInitAreaSwitchNum(a) ((a == 0) ? DEV_INIT_AREA_SW0 : DEV_INIT_AREA_SW1)

#define CAN_MOTOR_LEGACY_TICKS_PER_REV	16384	// for backward compatibility with original closed loop motor control firmware

////////////////////////////////////////////////////////////////////////////////
// PickNPlace RELATED //////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum {
	RIGHT = 0,
	LEFT = 1,
	CENTER = 2,
} leftRight_t;

typedef union {
	unsigned word;
	struct {
		unsigned    motor           : 2;
		unsigned    pageOfs         : 2;
		unsigned    writeEnables    : 4; // [3] or D is msb, [0] or A is lsb
	} data;
	struct {
		unsigned    motor           : 2;
		unsigned    autoVacEn       : 1;
		unsigned    parkAfterCmd    : 1;
		unsigned    cmd             : 1;    // [0]
		unsigned    dest            : 1;    // [1]
		unsigned    rate            : 1;    // [2]
		unsigned    spare1          : 1;    // [3]
	} ctrl;
} pnpPageUnion;

typedef enum {
	PNP_CMD_IDLE = 0,       // these are also used as an array index so take care if adding or reordering
	PNP_CMD_HOME,
	PNP_CMD_PARK,
	PNP_CMD_MOVE_ABSOLUTE,
	PNP_CMD_MOVE_RELATIVE,
	PNP_CMD_PICK,
	PNP_CMD_PLACE,
	PNP_CMD_VACUUM,
	PNP_CMD_REEL_ADVANCE
} pnpCommand_t;

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD ADC RELATED /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define HH_NUM_ADC_CHANNELS             5   // legacy (for adc reporting struct)
#define MAX_ADC12                       0x0fff

typedef enum {
	POSITION    = 0,
	RTD1        = 1,
	RTD2        = 2,
	RTD3        = 3,
	TEMP_SENSOR = 4
} adcChanType_t;

// for HYDRA_DIAGS -- AND PROBE selectins
typedef enum {
	HH_PA3      = 0,
	HH_PA2      = 1,
	HH_LIM1     = 2,
	HH_LIM2     = 3,
	HH_RTD1     = 4,
	HH_RTD2     = 5,
	HH_SWC,
	HH_SWD,
	HH_HSS1,
	HH_HSS2,
	HH_LED0,
	HH_LED1,
	HH_LED2,
	HH_LED3,
	HH_LED4,
	HH_LED5,
	HH_LED6,
	HH_LED7,
	HH_NO_PIN,
} HH_IoIndex_t;

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD LIMIT SWITCH RELATED ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD EXTRUSION RELATED ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#define HH_CONTINUOUS_EXTRUSION_STEPS 0xffff

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD HIGH SIDE SWITCH RELATED ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define HH_NUM_HIGH_SIDE_POWER_SWITCHES 2
#define HH_SW0_SWITCH           0   // SW1-AUX on hot head schematic - PCB label for the connector
#define HH_AUX_SWITCH           0   // SW1-AUX on hot head schematic - PCB label for the connector
#define HH_FAN_SWITCH           0   // SW1-AUX on hot head schematic - PCB label for the connector

#define HH_SW1_SWITCH           1   // SW2-HTR on hot head schematic - PCB label for the connector
#define HH_HTR_SWITCH           1   // SW2-HTR on hot head schematic - PCB label for the connector
#define HH_LASER_PWR_SWITCH     1   // SW2-HTR on hot head schematic - PCB label for the connector

typedef enum {
	SWITCH_CONTROL_OFF          = 0,
	SWITCH_CONTROL_ON,
	SWITCH_CONTROL_DUTY_CYCLE,
	SWITCH_CONTROL_PWM,
	SWITCH_CONTROL_BY_TEMP,
} switchMode_t;

typedef union { // basic controls for how the switch behaves
	uint32_t u32;
	struct {
		unsigned usePulseTrain          : 1;    // if "1", pwm count passed through lookup table to get on/off state (with period=100 only)
		unsigned onOnlyWhenExtruding    : 1;    // if "1" switch is only on when the extruder is running (pwm and temp only)
		unsigned blockColdExtrude       : 1;    // if "1", extrusion "steps" are only allow if temp is between hot/cold limits
		unsigned polarity               : 1;    // if "0" for activeLow, then invert the final outputSignal
		// no mcodes for these -- init only between Hydra and head
		unsigned heatingDevice          : 1;    // if "1" heater, "0" is a not heater
		unsigned coolingDevice          : 1;    // if "1" chiller, "0" is a not a chiller
		unsigned allowCtrlByOn          : 1;    // if "1" allow HSS control via SWITCH_CONTROL_ON
		unsigned allowCtrlByDC          : 1;    // if "1" allow HSS control via SWITCH_CONTROL_DUTY_CYCLE
		unsigned allowCtrlByPwm         : 1;    // if "1" allow HSS control via SWITCH_CONTROL_PWM
		unsigned allowCtrlByTemp        : 1;    // if "1" allow HSS control via SWITCH_CONTROL_TEMP (fixed 0 to 100 DC)
		unsigned checkMaxTemp           : 1;    // if "1" indicates there's a max temperature limit to test against (ie, no for a MILL, yes for LASER or HTR)
		unsigned checkMinTemp           : 1;    // if "1" indicates there's a max temperature limit to test against (ie, no for a MILL, yes for a chiller)
		unsigned binaryTempCtrl         : 1;    // if "1", and DC>1 will turn on switch, off otherwise (slam On mode)
		unsigned useWatchdogTimer       : 1;    // if "1" use watchdog timer to prevent switch from staying on too long
		unsigned useCooldownTimer       : 1;    // if "1" fan is kept on for N seconds after turning of the heater switch
	} b;
} switchFlags_t;

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD TEMPERATURE RELATED /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define TEMP_FRAC_BITS                  5
#define TEMP_SCALE                      32
#define TEMP_SCALEF                     32.0f
#define BOGUS_TEMPERATURE               (temp_t)0x8000  // max negative
#define MIN_TEMP                        (temp_t)0x8001  // max negative
#define MAX_TEMP                        (temp_t)0x7fff  // max positive
#define MAX_AMBIENT_TEMPERATURE         (500 << TEMP_FRAC_BITS)
#define PELTIER_COOLING_MODE_RELAY_DUTY 100   // relay ON to enable cooling mode
#define PELTIER_HEATING_MODE_RELAY_DUTY 0

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD PROBE RELATED ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum {
	PROBE_ACTION_STOP           = 0,
	PROBE_ACTION_ARM            = 1,
	PROBE_ACTION_READ           = 2,
	PROBE_ACTION_CONTROL        = 3
} probeAction_t;

typedef enum {
	PROBE_TYPE_NONE    = 0,
	PROBE_TYPE_CONTACT = 1,
	PROBE_TYPE_BLTOUCH = 2,
	PROBE_TYPE_HC_SR04 = 3,
	PROBE_TYPE_IGAGING = 4,
	PROBE_TYPE_UNKNOWN = 0xff,
} probeType_t;

typedef enum {
	BLTOUCH_OFF             = 0,
	BLTOUCH_ARM             = 1,
	BLTOUCH_PROBE_DOWN      = 2,
	BLTOUCH_PROBE_UP        = 3,
	BLTOUCH_SELFTEST        = 4,
	BLTOUCH_ALARM_AND_UP    = 5,
	BLTOUCH_ALARM_AND_SW    = 6,
} bltouch_t;

#define PROBE_ERROR -1
#define IGAGING_NUM_BITS 21

typedef enum {
	PROBE_RESULTS_MOTION_FINISHED                      = 1,     // error - probe did not fire during move
	PROBE_RESULTS_PROBE_CONTACT_BEFORE_MOVE_STARTED    = 2,     // error - probe already contacting on G38 before move
	PROBE_RESULTS_CANBUS_PROBE_REPORT                  = 3,     // report in from canbus
	PROBE_RESULTS_CANBUS_PROBE_ARMING_TIMEOUT          = 4,
} probeResults_t;

#define TOUCH_PROBE_ON_CANBUS 0

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD AUTOSTATUS RELATED //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef struct {
	uint32_t            pageSelect;             // which data to send
	byte                rate;                   // seconds between updates

	byte                secondsCnt;             // count of numbers of seconds past
	byte                prescaleCnt;            // subcounter for number of 100 Hz time slices
	boolean             dumpingStatus;          // flag to indicate it's time to output status
} autoStatusStruct;

////////////////////////////////////////////////////////////////////////////////
// GLOBAL CANBUS STRUCTURES ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef union {
	uint8_t     u8[8];
	int8_t      i8[8];
	uint16_t    u16[4];
	int16_t     i16[4];
	uint32_t    u32[2];
	int32_t     i32[2];
	uint64_t    u64;
	int64_t     i64;
	float		flt[2];
} payloadUnion;

typedef struct {
	unsigned fixed_b0   : 1; // Start-of-Frame - must be '1'
	unsigned RTR        : 1; // RTR = Remote Transmission Request (RTR)
	unsigned IDE        : 1; // IDE = Identifier extension bit (IDE) - must be '1' (only use extended msg)
	unsigned page       : 8;
	unsigned msgId      : 7;
	unsigned immediate  : 1;
	unsigned device     : 8;
	unsigned msgType    : 2;
	unsigned fixed_b100 : 3; // End-of-Frame

	unsigned numBytes   : 4;
	unsigned fromCAN2   : 1;
	unsigned pad        : 27;

	payloadUnion payload;
} canSwStruct;

typedef struct {
	uint32_t IR;        /*!< CAN receive FIFO mailbox identifier register */
	uint32_t DTR;       /*!< CAN receive FIFO mailbox data length control and time stamp register */
	uint32_t DLR;       /*!< CAN receive FIFO mailbox data low register */
	uint32_t DHR;       /*!< CAN receive FIFO mailbox data high register */
} canHwStruct;

// this union allows cleaner access to the data going to/coming from the CAN hardware.
// the size (16-bytes) is defined based on the four 32-bt registers that are the data interface to/from the
// CAN hardware.  this the the canHwStruct.   the other mapping allows for software to directly access the hyrel
// defined fields without having to keep track of offsets/shifting/masking to pull out the desired field.
// the resulting code is efficient as the arm processor has specific instructions dealing with bitfields.

typedef struct {
	union {
		canHwStruct hw __attribute__ ((aligned (8)));       // get on double word boundry for directy copying of data
		canSwStruct sw;
	};
} canStruct;


#if defined(COMPILE_FOR_DEVICE) && defined(BOOTLOADER)
#define CAN_TX_QUEUE_SIZE 256
#define CAN_RX_QUEUE_SIZE 1
#elif defined (COMPILE_FOR_DEVICE)
#define CAN_TX_QUEUE_SIZE 128 //192
#define CAN_RX_QUEUE_SIZE 128 //256
#elif defined (STM32F4XX_HYREL) && defined (HYDRA_DIAGS)
#define CAN_TX_QUEUE_SIZE 384
#define CAN_RX_QUEUE_SIZE 384
#elif defined (STM32F4XX_HYREL)
#define CAN_TX_QUEUE_SIZE 512
#define CAN_RX_QUEUE_SIZE 512
#else
#define CAN_TX_QUEUE_SIZE 32
#define CAN_RX_QUEUE_SIZE 32
#endif

typedef struct {
	canStruct   Q[CAN_TX_QUEUE_SIZE];
	uint16_t    nextIn;
	uint16_t    nextOut;
	uint16_t    numMsg;
} txQueueStruct;

typedef struct {
	canStruct   Q[CAN_RX_QUEUE_SIZE];
	uint16_t    nextIn;
	uint16_t    nextOut;
	uint16_t    numMsg;
} rxQueueStruct;

////////////////////////////////////////////////////////////////////////////////
// HOTHEAD FLASH RELATED ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define ERASED_WORD32                  ((uint32_t)0xffffffff)
#define ERASED_WORD16                  ((uint16_t)0xffff)

#define HH_FLASH_PAGE_SIZE              0x400

#define HH_POSITION_UNPLUGGED           0xFE	//254

#define CAN_NUM_GROUPS                  8
#define HH_GROUP_POSITION_MIN           50
#define HH_GROUP_POSITION_MAX           (HH_GROUP_POSITION_MIN + CAN_NUM_GROUPS - 1)
#define HH_HOTBED_POSITION_ALL          90
#define HH_HOTBED_POSITION_MIN          91
#define HH_HOTBED_POSITION_MAX          92
#define HH_HOTBED_POSITION              91
#define HH_CO2_LASER_POSITION_MIN       41
#define HH_CO2_LASER_POSITION_MAX       41

#define HH_GROUP_IDENTIFIER_MIN         50
#define HH_GROUP_IDENTIFIER_MAX         57

#define HH_BROADCAST_ALL                0
#define HH_BROADCAST_ALL_YOKE1          10
#define HH_BROADCAST_ALL_YOKE2          20
#define HH_BROADCAST_ALL_YOKE3          30
#define HH_BROADCAST_ALL_YOKE4          40
#define HH_BROADCAST_ALL_GROUPS         50
#define HH_BROADCAST_ALL_HOTBEDS        90
#define HH_BROADCAST_ALL_DEVICES        100 // (non-hotbeds)

#define HH_HOTBED_MAX_ALLOWABLE_TEMP    (110 << TEMP_FRAC_BITS)     // for HOTBED (devicePosition == 91 to 99)
#define HH_HEATER_MAX_ALLOWABLE_TEMP    (270 << TEMP_FRAC_BITS)     // for NON HOTBED
#define HH_AUX_MAX_ALLOWABLE_TEMP       (70 << TEMP_FRAC_BITS)
#define HH_MAX_EXTRUSION_RATE           20000                       // 3200 is 1 rev/sec
#define HH_RTD_TEMP_DELTA                       (15 << TEMP_FRAC_BITS)      // 15 per Karl - max number of degrees allowed
// between RTD1 and RTD2 for fusion chamber (8 degrees)

#define HH_PICKNPLACE_CLOCKWISE           0
#define HH_PICKNPLACE_COUNTER_CLOCKWISE   1

#define HH_SPINDLE_CLOCKWISE                0
#define HH_SPINDLE_COUNTER_CLOCKWISE        1
#define HH_SPINDLE_OFF                      2

#define DEFAULT_INKJET_DROPLETS_PER_MM      8.0f
#define DEFAULT_SPINDLE_PWM_FREQ            2000

#define IS_A_VALID_GROUP_IDENIFIER(a)  (((a) >= HH_GROUP_IDENTIFIER_MIN) && ((a) <= HH_GROUP_IDENTIFIER_MAX))


////////////////////////////////////////////////////////////////////////////////
// SONICATOR ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define SONICATOR_MAX_DAC12                 0x0fff
#define SONICATOR_MAX_DAC_INDEX             2
#define SONICATOR_MAX_TIMER_INDEX           14
#define SONICATOR_MAX_CHANNEL_INDEX         7

#define SONICATOR_PAGE_INDEX_SHIFT          0
#define SONICATOR_PAGE_INDEX_MASK           0x1f

#define SONICATOR_PAGE_CTRL_SHIFT           5
#define SONICATOR_PAGE_CTRL_MASK            0xe0

#define SONICATOR_PAGE_CTRL_DISPLAY_PAGE    0x00
#define SONICATOR_PAGE_CTRL_DAC             0x01
#define SONICATOR_PAGE_CTRL_TIMER_PWM1      0x02
#define SONICATOR_PAGE_CTRL_TIMER_PWM2      0x03
#define SONICATOR_PAGE_CTRL_TIMER_TOGGLE    0x04
#define SONICATOR_PAGE_CTRL_TIMER_EXT_CLK   0x05
#define SONICATOR_PAGE_CTRL_TIMER_INT_CLK   0x06
#define SONICATOR_PAGE_CTRL_SCOPE           0x07
#define SONICATOR_PAGE_CTRL_WAVEFORM        0x08

#define SONICATOR_DAC_WAVEFORM_NONE         0
#define SONICATOR_DAC_WAVEFORM_TRIANGLE     1
#define SONICATOR_DAC_WAVEFORM_NOISE        2
#define SONICATOR_DAC_WAVEFORM_SINE         3
#define SONICATOR_DAC_WAVEFORM_SQUARE       4


#define WAVE_GEN_MAX_SAMPLES				360


////////////////////////////////////////////////////////////////////////////////
// HOTHEAD CONTROL WORD ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// Hot Head Control Word
// set via CAN with no params
typedef union controlWordUnion {
	uint32_t    u32;
	struct {
		unsigned registered         : 1;//set if currently registered with host via can bus
		unsigned swReset            : 1;//request for software reset,
		unsigned globalSync         : 1;//causes all timers to reset and synchronize, makes all hot heads on canbus time toghether
		unsigned stopExtruder       : 1;//stop extruder request

		unsigned pauseExtruder      : 1;//block the steps on the extruder
		unsigned stopOnError        : 1;//
		unsigned saveHistory        : 1;//
		unsigned clearError         : 1;//

		unsigned ackOnWrite         : 1;//
		unsigned invertDirection    : 1;//reverse Motor direction polarity, inverts polarity of motor direction bit
		unsigned enableMotor        : 1;//enable motor status bit, set by 103 and can be read from 407
		unsigned disableMotor       : 1;//disable motor, must be command from 407 as

		unsigned spare1             : 1;//disable warnings from rtd1
		unsigned spare2             : 1;//
		unsigned debugRTD           : 1;//enable debug of RTD/ADC
		unsigned blockFlashWrites   : 1;//

		unsigned primeExtruder      : 1;//same as prime run but uses current values for rate and steps
		unsigned unprimeExtruder    : 1;//same as unprime but uses current values for rate and steps
		unsigned gateExtruder       : 1;
		unsigned retractExtruder    : 1;

		unsigned canInitialized     : 1;
		unsigned sentAnnounce       : 1;
		unsigned materialOverTemp   : 1;//error flag to 407
		unsigned motorOverTemp      : 1;//error flag to 407

		unsigned extruding          : 1;//currently extruding
		unsigned processing         : 1;//
		unsigned updateBootloader   : 1;//?
		unsigned commPing           : 1;

		unsigned stepFromCanbusMode : 1;
		unsigned ignoreWatchdog     : 1;  // for debugging 407/429
		unsigned initialize         : 1;
		unsigned sendDebugStrings   : 1;
		//1 more bit open for future expansion
	} bit;
} controlWordUnion;


// the following must align to the controlWord bitfield definition in the HH.

#define HH_REGISTERED_BIT                   ((uint32_t)0x00000001)      //      marks that head has been registered w/ system
#define HH_SW_RESET_BIT                     ((uint32_t)0x00000002)      //      tells software to reset itself
#define HH_GLOBAL_SYNC_BIT                  ((uint32_t)0x00000004)      //      tells software to reset loop counters to 0
#define HH_STOP_EXTRUDER_BIT                ((uint32_t)0x00000008)      //      tells software to enter a hw safe state

#define HH_PAUSE_EXTRUDER_BIT               ((uint32_t)0x00000010)      //      tells software to stop extrusion (no steps)
#define HH_STOP_ON_ERROR_BIT                ((uint32_t)0x00000020)      //      tells software to go to a safe state on an error
#define HH_SAVE_HISTORY_BIT                 ((uint32_t)0x00000040)      //      tells software to update history to flash
#define HH_CLEAR_ERROR_BIT                  ((uint32_t)0x00000080)      //      tells software to clean current error and proceed

#define HH_ACK_ON_WRITE_BIT                 ((uint32_t)0x00000100)      //      tells software to send & expect) ACK packets on writes
#define HH_INVERT_DIRECTION_BIT             ((uint32_t)0x00000200)      //      tells software to invert the direction value
#define HH_ENABLE_MOTOR_BIT                 ((uint32_t)0x00000400)      //      tells software to turn on the motor's enable pin
#define HH_DISABLE_MOTOR_BIT                ((uint32_t)0x00000800)      //      tells software to turn off the motor's enable pin

#define HH_SPARE_1_BIT                      ((uint32_t)0x00001000)      //      tells software to
#define HH_SPARE_2_BIT                      ((uint32_t)0x00002000)      //      tells software to
#define HH_DEBUG_RTD_BIT                    ((uint32_t)0x00004000)      //      tells software to enable RTD/ADC debug
#define HH_BLOCK_FLASH_WRITES_BIT           ((uint32_t)0x00008000)      //      tells software to block all writes to flash (ie, for a pending shutdown)

//---------------------

#define HH_PRIME_EXTRUDER_BIT               ((uint32_t)0x00010000)      //      tells software to prime extruder with prime setting
#define HH_UNPRIME_EXTRUDER_BIT             ((uint32_t)0x00020000)      //      tells software to unprime extruder with unprime setting
#define HH_GATE_EXTRUDER_BIT                ((uint32_t)0x00040000)      //      tells software to close extruder gate
#define HH_RETRACT_EXTRUDER_BIT             ((uint32_t)0x00080000)      //      tells software to retract a retractable head

// internal status
#define HH_CAN_INITIALIZED_BIT              ((uint32_t)0x00100000)      //      marks that the CAN unit is initialized
#define HH_SENT_ANNOUNCE_BIT                ((uint32_t)0x00200000)      //      marks that an Announce message has been sent
#define HH_MATERIAL_OVER_TEMP_BIT           ((uint32_t)0x00400000)      //      marks that the fusion chabmber is over the max allowed
#define HH_MOTOR_OVER_TEMP_BIT              ((uint32_t)0x00800000)      //      marks that the motor temp is over the max allowed

#define HH_EXTRUDING_BIT                    ((uint32_t)0x01000000)      //      marks that a "real" extrude is underway
#define HH_PROCESSING_BIT                   ((uint32_t)0x02000000)      //      marks that a slice is currently underway
#define HH_UPDATE_BOOTLOADER_BIT            ((uint32_t)0x04000000)      //      tells software to allow bootloader pages to be written
#define HH_COMM_PING_BIT                    ((uint32_t)0x08000000)      //      used by main board to let HH know it's still out there

#define HH_STEP_FROM_CANBUS_MODE_BIT        ((uint32_t)0x10000000)      //      tells head to only accept step pulses from direct canbus command and not generate via timer
#define HH_IGNORE_WATCHDOG_BIT              ((uint32_t)0x20000000)      //      ignore the system to device comm watchdog (for debugging 407)
#define HH_INITIALIZE_BIT                   ((uint32_t)0x40000000)      //      tells software to finish its init sequence (V1)
#define HH_SEND_DEBUG_STRINGS_BIT           ((uint32_t)0x80000000)      //      tells software to send debug strings via canbus

#define nnn_BIT                             ((uint32_t)0x00000000)      //      t

/////////////////////////////////////////////////////////////////////////////////
// HOTHEAD ERROR CODES //////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
//
// ERROR_UNITS start at 0xC1
// ERROR and WARNING values also used as return codes for functions.
//
// ERROR_UNIT_xxxx used to indicate which section of code had the error
// ERROR_CODE_xxxx used to expicitly state the error type within that UNIT

#define ERROR_NONE                                      0x00
#define ERROR_UNIT_NONE                                 0x00
#define     ERROR_CODE_NONE                                     0x00
#define ERROR_UNIT_CAN                                  0xC1
#define     CAN_TX_OK                                           0x00
#define     CAN_TX_UNREGISTERED_DESTINATION                     0x01
#define     CAN_RX_OK                                           0x00
#define     WARNING_NOT_INITIALIZED                             0x01
#define     WARNING_RX_NO_MSG_WAITING                           0x02
#define     WARNING_TX_NOT_SENT                                 0x03
#define     WARNING_TX_WAITING_FOR_ACK                          0x04
#define     WARNING_TX_NO_DEVICE_ID                             0x05
#define     WARNING_TX_MAILBOX_FULL                             0x06
#define     ERROR_SWITCH_MSG_TYPE                               0x07
#define     ERROR_SWITCH_MSG_ID                                 0x08
#define     ERROR_BAD_PAGE_NUM                                  0x09
#define     ERROR_NUM_BYTES_MISMATCH                            0x0A
#define     ERROR_RX_QUEUE_FULL                                 0x0B
#define     ERROR_TX_QUEUE_FULL                                 0x0C
//#define     ERROR_ILLEGAL_ACCESS_ADDRESS                        0x0D
#define     ERROR_TOO_MANY_ALIASES                              0x0E
#define     ERROR_BAD_CHECKSUM                                  0x0F
#define     ERROR_PAGE_WRITE_FAILED                             0x10
#define     ERROR_COMM_TIMEOUT                                  0x11
#define     ERROR_GUI_QUEUE_FULL                                0x12
#define ERROR_UNIT_ADC                                  0xC2
//#define     ERROR_ADC_NOISE_PART1                               0x01
//#define     ERROR_ADC_NOISE_PART2                               0x02
//#define     ERROR_POSITION_CHANGE                               0x03
//#define     ERROR_LOST_POSITION_RESISTOR                        0x04
//#define     ERROR_CANT_DETERMINE_POSITION                       0x05
//#define     ERROR_RTD1_RTD2_TEMP_DELTA                          0x06
#define     ERROR_EXCEEDED_TABLE_LENGTH                         0x07
#define     ERROR_ADC_EXCEEDED_RANGE                            0x08
#define     ERROR_ADC_RTD_EXPECTED                              0x09
#define ERROR_UNIT_FLASH                                0xC3
//#define     ERROR_OPTION_BYTE_PROGRAMMING_FAILED                0x01
#define     ERROR_ERASE_FAILED                                  0x02
#define     ERROR_MISSING_TABLES                                0x03
//#define     ERROR_PRIMARY_PROGRAM_CORRUPT                       0x04
#define ERROR_UNIT_LOOP                                 0xC4
#define     ERROR_BAD_VECTOR                                    0x01
#define     ERROR_SLICE_OVERRUN                                 0x02
#define ERROR_UNIT_DEVICE                               0xC5
#define     ERROR_LASER_NO_WATER_PRESSURE                       0x01
#define     ERROR_LASER_INTERLOCK_OPEN                          0x02
#define     ERROR_LASER_IS_OVER_TEMP                            0x03
#define     ERROR_INTERNAL_SHUTDOWN_REQUESTED                   0x04
//#define     ERROR_SPINDLE_TOO_FAST                              0x05
#define     ERROR_MANUAL_SWITCH_FAILED                          0x06
#define     ERROR_ILLEGAL_HSS_CONTROL                           0x07
#define     ERROR_CONFIG_SAVE_FAILED_NO_SPACE					0x08
//#define   ERROR_INKJET_CABLE                                  0x09
#define     ERROR_SPI_ENCODER_ERROR                             0xa
#define     ERROR_SPI_LCD_ERROR                            		0xb
#define ERROR_UNIT_HEATER                               0xC6
#define     ERROR_MATERIAL_IS_OVER_TEMP                         0x01
#define     ERROR_HOTBED_IS_OVER_TEMP                           0x02
#define     ERROR_CHAMBER_IS_OVER_TEMP                          0x03
#define     ERROR_HEATER_SET_TEMP_TOO_HIGH                      0x04
#define     ERROR_MATERIAL_IS_UNDER_TEMP                        0x05
#define     ERROR_HOTBED_IS_UNDER_TEMP                          0x06
#define     ERROR_CHAMBER_IS_UNDER_TEMP                         0x07
#define     ERROR_CHILLER_SET_TEMP_TOO_LOW                      0x08
#define ERROR_UNIT_MOTION                               0xC7
//#define     ERROR_MOTOR_IS_OVER_TEMP                            0x01
//#define     ERROR_MAX_EXTRUSION_RATE_EXCEEDED                   0x02
//#define     ERROR_EXTRUSION_RATE_OVERFLOW                       0x03
//#define     ERROR_EXTRUSION_STEPS_OVERFLOW                      0x04
//#define     ERROR_SPINDLE_HOMING_FAILED                         0x05
#define       ERROR_MAX_ANGLE_ERROR_EXCEEDED					0x06
//#define ERROR_UNIT_HISTORY                              0xC8
//#define     ERROR_BAD_NEW_BANK_ADDR                             0x01
//#define     ERROR_BAD_INIT                                      0x02
#define ERROR_UNIT_UART                                 0xC9
#define     ERROR_RX_BUFFER_FULL                                0x01
#define ERROR_UNIT_INIT                                 0xCA
//#define     ERROR_DEVICE_TYPE_NOT_FOUND                         0x01
#define     ERROR_EXCEEDED_ADC_ARRAY_SIZE                       0x02
#define     ERROR_ILLEGAL_DEV_TYPE_AND_PCB_COMBO                0x03
//#define     ERROR_OPT_BYTES_NOT_PROGRAMMED                      0x04
//#define     ERROR_INVALID_SOAPSTRING                            0x05
#define     ERROR_UNKNOWN_SOAP_DEVICE_TYPE                      0x06
#define     ERROR_UNKNOWN_SOAP_DEVICE_SUBTYPE                   0x07
#define     ERROR_UNKNOWN_SOAP_PCB_TYPE                         0x08
#define     ERROR_UNKNOWN_SOAP_RTD_TYPE                         0x09
#define     ERROR_UNKNOWN_MISSING_RTD_POINTER                   0x0A

////////////////////////////////////////////////////////////////////////////////
// NEW HEAD CODE INITIALIZATION
////////////////////////////////////////////////////////////////////////////////

typedef enum {
	CANBUS_FORMAT_V0 = '0',
	CANBUS_FORMAT_V1 = '1',
	CANBUS_FORMAT_V2 = '2', //for version 5
} canbusFormat_t;

// for each new codebase, please create a unique number (other than 0 or 255/0xff
// (will be compiled into the code and can be sent back to the host via canbus)
typedef enum {
	DEVICE_CODEBASE_UNKNOWN         = 0,        // reserved
	DEVICE_CODEBASE_MEDUSA          = 1,        // original code to cover all print heads/lasers/mill in one project
	DEVICE_CODEBASE_PICKNPLACE      = 2,        // original code to cover the pick_n_place and probe heads
// QQQQ ADD NEW CODEBASE(S) HERE
	DEVICE_CODEBASE_UNPROGRAMMED    = 0xff      // reserved - unprogrammed state
} devCodebase_t;

typedef enum {
	DEVICE_TARGET_UNKNOWN           = 0,        // reserved
	DEVICE_TARGET_MEDUSA103         = 1,        // original code to cover all print heads/lasers/mill in one project
	DEVICE_TARGET_MEDUSA405         = 2,        // new 405 based code
	DEVICE_TARGET_PICKNPLACE        = 3,        // original code to cover the pick_n_place and probe heads
	DEVICE_TARGET_SONICATOR407      = 4,        // mcudev based sonicator
	DEVICE_TARGET_CAN_MOTOR_103		= 5,
// QQQQ ADD NEW TARGET(S) HERE
	DEVICE_TARGET_UNPROGRAMMED      = 0xff      // reserved - unprogrammed state
} devTarget_t;

typedef enum {  // codes for soapstring[0]
	// only add new type at end of the list (before SOAP_DEV_TYPE_BAD_COMBO)
	SOAP_DEV_TYPE_UNKNOWN               = 0,
	SOAP_DEV_TYPE_MK1                   = '0',
	SOAP_DEV_TYPE_MK2                   = '1',
	SOAP_DEV_TYPE_MK450                 = '2',
	SOAP_DEV_TYPE_EMO_25                = '3',
	SOAP_DEV_TYPE_EMO_50                = '4',
	SOAP_DEV_TYPE_VOLCANO_15            = '5',
	SOAP_DEV_TYPE_VOLCANO_25            = '6',
	SOAP_DEV_TYPE_VOLCANO_50            = '7',
	SOAP_DEV_TYPE_KRAKATOA_15           = '8',
	SOAP_DEV_TYPE_TAMBORA_15            = '9',
	SOAP_DEV_TYPE_SKIP_COLON            = ':',
	SOAP_DEV_TYPE_LASER_2               = ';',
	SOAP_DEV_TYPE_LASER_5               = '<',
	SOAP_DEV_TYPE_LASER_6               = '=',
	SOAP_DEV_TYPE_LASER_40              = '>',
	SOAP_DEV_TYPE_LASER_80              = '?',
	SOAP_DEV_TYPE_LASER_150             = '@',
	SOAP_DEV_TYPE_UVLIGHT_RAY           = 'A',
	SOAP_DEV_TYPE_CO2_LENS              = 'B',
	SOAP_DEV_TYPE_INKJET1               = 'C',
	SOAP_DEV_TYPE_INKJET2               = 'D',
	SOAP_DEV_TYPE_QUIET_STORM           = 'E',
	SOAP_DEV_TYPE_MICROSCOPE            = 'F',
	SOAP_DEV_TYPE_HOTBED_100            = 'G',
	SOAP_DEV_TYPE_HOTBED_120            = 'H',
	SOAP_DEV_TYPE_HOTBED_200            = 'I',
	SOAP_DEV_TYPE_HOTCOLDBED            = 'J',
	SOAP_DEV_TYPE_DRILL_MILL            = 'K',
	SOAP_DEV_TYPE_DIGITRAM              = 'L',
	SOAP_DEV_TYPE_PICKPLACE             = 'M',
	SOAP_DEV_TYPE_SDS                   = 'N',
	SOAP_DEV_TYPE_SDS_5                 = 'O',
	SOAP_DEV_TYPE_SDS_10                = 'P',
	SOAP_DEV_TYPE_SDS_30                = 'Q',
	SOAP_DEV_TYPE_SDS_60                = 'R',
	SOAP_DEV_TYPE_SDS_150               = 'S',
	SOAP_DEV_TYPE_BITSCOPE              = 'T',
	SOAP_DEV_TYPE_FIL_DISP              = 'U',
	SOAP_DEV_TYPE_3PH_SPINDLE           = 'V',
	SOAP_DEV_TYPE_GENERIC_HEAD          = 'W',
	SOAP_DEV_TYPE_RH_SYRINGE            = 'X',
	SOAP_DEV_TYPE_SONICATOR             = 'Y',
	SOAP_DEV_TYPE_CAN_MOTOR				= 'Z',
// QQQQ ADD NEW DEVICE(S) HERE
	SOAP_DEV_TYPE_LAST_ENTRY,
	SOAP_DEV_TYPE_BAD_COMBO             = 0xfe, //      used for init values if bad combo of devType/PCB/RTD is detected
	SOAP_DEV_TYPE_UNPROGRAMMED          = 0xff,
} soapDevType_t;

////////////////////////////////////////////////////////////////////////////////

typedef enum {  // codes for soapstring byte[1]
	SOAP_DEV_SUBTYPE_UNKNOWN                    = 0,
	SOAP_DEV_SUBTYPE_0                          = '0',  // standard option
	SOAP_DEV_SUBTYPE_1                          = '1',
	SOAP_DEV_SUBTYPE_2                          = '2',
	SOAP_DEV_SUBTYPE_3                          = '3',
	SOAP_DEV_SUBTYPE_LAST_ENTRY,

	SOAP_DEV_SUBTYPE_C02_LASER_UART             = '0',
	SOAP_DEV_SUBTYPE_C02_LASER_NO_UART          = '1',

	SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_PP    = '0',
	SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_OD    = '1',
	SOAP_DEV_SUBTYPE_DIODE_LASER_NO_COOLDOWN_PP = '2',
	SOAP_DEV_SUBTYPE_DIODE_LASER_NO_COOLDOWN_OD = '3',

	SOAP_DEV_SUBTYPE_CENTER_PNP_HOME_ACT_LOW    = '0',
	SOAP_DEV_SUBTYPE_CENTER_PNP_HOME_ACT_HIGH   = '1',

	// QQQQ ADD NEW SUBTYPES(S) HERE
	SOAP_DEV_SUBTYPE_UNPROGRAMMED               = 0xff,
} soapDevSubtype_t;

////////////////////////////////////////////////////////////////////////////////

typedef enum {  // codes for soapstring byte[2]
	SOAP_PCB_UNKNOWN                = 0xff,
	SOAP_PCB_4988_160J_NOWIRE       = '0',      // 102081 rev 2/3/4/5:  4988 motor driver; correct VND5E160J HSS
	SOAP_PCB_4982_160J_NOWIRE       = '1',      // 102081 rev 2: 4982 motor driver; correct VND5E160J HSS   --- DO THESE EXIST????
	SOAP_PCB_4982_160MJ_WIRE        = '2',      // 102081 rev 2: 4982 motor driver; wrong VND5E160MJ HSS w/ jumper (lose LIMIT1)
	SOAP_PCB_4982_160MJ_NOWIRE      = '3',      // 102081 rev 2: 4982 motor driver; wrong VND5E160MJ HSS (unable to use HSS2; LIMIT1 intact)
#if 1 //prefer
	SOAP_PCB_BTT					= '4',		// big tree tech V2 motor driver w/ encoder
	SOAP_PCB_405_4988				= '5',		// 405 based head board with A4988 motor stamp and display
	SOAP_PCB_405_ENC_PWM			= '6',		// 405 based head board with bridge, encoder and display --> PWM control of bridge vref
	SOAP_PCB_405_ENC_DAC			= '7',		// 405 based head board with bridge, encoder and display --> DAC control of bridge vref
#else
	SOAP_PCB_405_4988				= '4',		// 405 based head board with A4988 motor stamp and display
	SOAP_PCB_BTT					= '5',		// big tree tech V2 motor driver w/ encoder
	SOAP_PCB_405_ENC_PWM			= '6',		// 405 based head board with bridge, encoder and display --> PWM control of bridge vref
	SOAP_PCB_405_ENC_DAC			= '7',		// 405 based head board with bridge, encoder and display --> DAC control of bridge vref
#endif

	// QQQQ ADD NEW PCB(S) HERE
	SOAP_PCB_LAST_ENTRY,
	SOAP_PCB_UNPROGRAMMED           = 0xff,
} soapPcb_t;

////////////////////////////////////////////////////////////////////////////////
//NUKE -- change to #defines
typedef enum {  // codes for soapstring byte[3]
	SOAP_RTD_UNKNOWN    = 0xff,
	SOAP_RTD_NONE       = '0',
	SOAP_RTD_1M         = '1',
	SOAP_RTD_1K         = '2',
	SOAP_RTD_10K        = '3',
	SOAP_RTD_50K        = '4',
	SOAP_RTD_4_TO_20MA  = '5',
	SOAP_RTD_RAW_ADC    = '6',
	SOAP_RTD_100		= '7',
	// QQQQ ADD NEW RTD(S) HERE
	SOAP_RTD_LAST_ENTRY,
	SOAP_RTD_UNPROGRAMMED   = 0xff,
} soapRtd_t;

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	ADD_NONE = 0,
	ADD_CPROBE,
	ADD_AM23XX,
	ADD_HX711,
	ADD_KEYPAD,
} devAddOn_t;

#define CPROBE_MASK 	(1 << ADD_CPROBE)
#define AM23XX_MASK 	(1 << ADD_AM23XX)
#define HX711_MASK 		(1 << ADD_HX711)
#define KEYPAD_MASK 	(1 << ADD_KEYPAD)

////////////////////////////////////////////////////////////////////////////////

// lower 4 bits are to control loading for u16[3:0]
// upper 4 biths are control flags
// u16[0]
#define HEAD_FUNCTION_PAGE_LOAD_RATE                0x01    //extruders-laser-inkjets-spindle
// u16[1]
#define HEAD_FUNCTION_PAGE_LOAD_QTY                 0x02    //extruders
#define HEAD_FUNCTION_PAGE_LOAD_POWER               0x02    //laser-inkjets-spindle
// u16[2]
#define HEAD_FUNCTION_PAGE_LOAD_DIRECTION           0x04    //spindle
#define HEAD_FUNCTION_PAGE_LOAD_WATCHDOG            0x04    //laser-inkjets
// u16[3]
// spare
#define HEAD_FUNCTION_PAGE_FLAG_RUN                 0x10    // all
#define HEAD_FUNCTION_PAGE_FLAG_PRIME               0x20    // all
#define HEAD_FUNCTION_PAGE_FLAG_PRIME_RUN           (HEAD_FUNCTION_PAGE_FLAG_RUN | HEAD_FUNCTION_PAGE_FLAG_PRIME)
#define HEAD_FUNCTION_PAGE_FLAG_WATCHDOG_IN_SEC     0x40    //laser-inkjets (ms otherwise)

#define PAGE_MASK_AUTO_STATUS_CONTROL_MASK_BIT      0x01
#define PAGE_MASK_AUTO_STATUS_CONTROL_RATE_BIT      0x02

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#endif // #ifndef hyrel_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
