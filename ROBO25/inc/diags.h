#include "stm32f4xx_adc.h"

#define sizeof_member(type, member) sizeof(((type *)0)->member)
#define invertedState(state)  ((state==LOW) ? HIGH : LOW)

////////////////////////////////////////////////////////////////////////////////

#define PA0     (PIN_PORT_A | PIN_NUM_0)
#define PA1     (PIN_PORT_A | PIN_NUM_1)
#define PA2     (PIN_PORT_A | PIN_NUM_2)
#define PA3     (PIN_PORT_A | PIN_NUM_3)
#define PA4     (PIN_PORT_A | PIN_NUM_4)
#define PA5     (PIN_PORT_A | PIN_NUM_5)
#define PA6     (PIN_PORT_A | PIN_NUM_6)
#define PA7     (PIN_PORT_A | PIN_NUM_7)
#define PA8     (PIN_PORT_A | PIN_NUM_8)
#define PA9     (PIN_PORT_A | PIN_NUM_9)
#define PA10    (PIN_PORT_A | PIN_NUM_10)
#define PA11    (PIN_PORT_A | PIN_NUM_11)
#define PA12    (PIN_PORT_A | PIN_NUM_12)
#define PA13    (PIN_PORT_A | PIN_NUM_13)
#define PA14    (PIN_PORT_A | PIN_NUM_14)
#define PA15    (PIN_PORT_A | PIN_NUM_15)

#define PB0     (PIN_PORT_B | PIN_NUM_0)
#define PB1     (PIN_PORT_B | PIN_NUM_1)
#define PB2     (PIN_PORT_B | PIN_NUM_2)
#define PB3     (PIN_PORT_B | PIN_NUM_3)
#define PB4     (PIN_PORT_B | PIN_NUM_4)
#define PB5     (PIN_PORT_B | PIN_NUM_5)
#define PB6     (PIN_PORT_B | PIN_NUM_6)
#define PB7     (PIN_PORT_B | PIN_NUM_7)
#define PB8     (PIN_PORT_B | PIN_NUM_8)
#define PB9     (PIN_PORT_B | PIN_NUM_9)
#define PB10    (PIN_PORT_B | PIN_NUM_10)
#define PB11    (PIN_PORT_B | PIN_NUM_11)
#define PB12    (PIN_PORT_B | PIN_NUM_12)
#define PB13    (PIN_PORT_B | PIN_NUM_13)
#define PB14    (PIN_PORT_B | PIN_NUM_14)
#define PB15    (PIN_PORT_B | PIN_NUM_15)

#define PC0     (PIN_PORT_C | PIN_NUM_0)
#define PC1     (PIN_PORT_C | PIN_NUM_1)
#define PC2     (PIN_PORT_C | PIN_NUM_2)
#define PC3     (PIN_PORT_C | PIN_NUM_3)
#define PC4     (PIN_PORT_C | PIN_NUM_4)
#define PC5     (PIN_PORT_C | PIN_NUM_5)
#define PC6     (PIN_PORT_C | PIN_NUM_6)
#define PC7     (PIN_PORT_C | PIN_NUM_7)
#define PC8     (PIN_PORT_C | PIN_NUM_8)
#define PC9     (PIN_PORT_C | PIN_NUM_9)
#define PC10    (PIN_PORT_C | PIN_NUM_10)
#define PC11    (PIN_PORT_C | PIN_NUM_11)
#define PC12    (PIN_PORT_C | PIN_NUM_12)
#define PC13    (PIN_PORT_C | PIN_NUM_13)
#define PC14    (PIN_PORT_C | PIN_NUM_14)
#define PC15    (PIN_PORT_C | PIN_NUM_15)

#define PD0     (PIN_PORT_D | PIN_NUM_0)
#define PD1     (PIN_PORT_D | PIN_NUM_1)
#define PD2     (PIN_PORT_D | PIN_NUM_2)
#define PD3     (PIN_PORT_D | PIN_NUM_3)
#define PD4     (PIN_PORT_D | PIN_NUM_4)
#define PD5     (PIN_PORT_D | PIN_NUM_5)
#define PD6     (PIN_PORT_D | PIN_NUM_6)
#define PD7     (PIN_PORT_D | PIN_NUM_7)
#define PD8     (PIN_PORT_D | PIN_NUM_8)
#define PD9     (PIN_PORT_D | PIN_NUM_9)
#define PD10    (PIN_PORT_D | PIN_NUM_10)
#define PD11    (PIN_PORT_D | PIN_NUM_11)
#define PD12    (PIN_PORT_D | PIN_NUM_12)
#define PD13    (PIN_PORT_D | PIN_NUM_13)
#define PD14    (PIN_PORT_D | PIN_NUM_14)
#define PD15    (PIN_PORT_D | PIN_NUM_15)

#define PE0     (PIN_PORT_E | PIN_NUM_0)
#define PE1     (PIN_PORT_E | PIN_NUM_1)
#define PE2     (PIN_PORT_E | PIN_NUM_2)
#define PE3     (PIN_PORT_E | PIN_NUM_3)
#define PE4     (PIN_PORT_E | PIN_NUM_4)
#define PE5     (PIN_PORT_E | PIN_NUM_5)
#define PE6     (PIN_PORT_E | PIN_NUM_6)
#define PE7     (PIN_PORT_E | PIN_NUM_7)
#define PE8     (PIN_PORT_E | PIN_NUM_8)
#define PE9     (PIN_PORT_E | PIN_NUM_9)
#define PE10    (PIN_PORT_E | PIN_NUM_10)
#define PE11    (PIN_PORT_E | PIN_NUM_11)
#define PE12    (PIN_PORT_E | PIN_NUM_12)
#define PE13    (PIN_PORT_E | PIN_NUM_13)
#define PE14    (PIN_PORT_E | PIN_NUM_14)
#define PE15    (PIN_PORT_E | PIN_NUM_15)

#define PF0     (PIN_PORT_F | PIN_NUM_0)
#define PF1     (PIN_PORT_F | PIN_NUM_1)
#define PF2     (PIN_PORT_F | PIN_NUM_2)
#define PF3     (PIN_PORT_F | PIN_NUM_3)
#define PF4     (PIN_PORT_F | PIN_NUM_4)
#define PF5     (PIN_PORT_F | PIN_NUM_5)
#define PF6     (PIN_PORT_F | PIN_NUM_6)
#define PF7     (PIN_PORT_F | PIN_NUM_7)
#define PF8     (PIN_PORT_F | PIN_NUM_8)
#define PF9     (PIN_PORT_F | PIN_NUM_9)
#define PF10    (PIN_PORT_F | PIN_NUM_10)
#define PF11    (PIN_PORT_F | PIN_NUM_11)
#define PF12    (PIN_PORT_F | PIN_NUM_12)
#define PF13    (PIN_PORT_F | PIN_NUM_13)
#define PF14    (PIN_PORT_F | PIN_NUM_14)
#define PF15    (PIN_PORT_F | PIN_NUM_15)

#define PG0     (PIN_PORT_G | PIN_NUM_0)
#define PG1     (PIN_PORT_G | PIN_NUM_1)
#define PG2     (PIN_PORT_G | PIN_NUM_2)
#define PG3     (PIN_PORT_G | PIN_NUM_3)
#define PG4     (PIN_PORT_G | PIN_NUM_4)
#define PG5     (PIN_PORT_G | PIN_NUM_5)
#define PG6     (PIN_PORT_G | PIN_NUM_6)
#define PG7     (PIN_PORT_G | PIN_NUM_7)
#define PG8     (PIN_PORT_G | PIN_NUM_8)
#define PG9     (PIN_PORT_G | PIN_NUM_9)
#define PG10    (PIN_PORT_G | PIN_NUM_10)
#define PG11    (PIN_PORT_G | PIN_NUM_11)
#define PG12    (PIN_PORT_G | PIN_NUM_12)
#define PG13    (PIN_PORT_G | PIN_NUM_13)
#define PG14    (PIN_PORT_G | PIN_NUM_14)
#define PG15    (PIN_PORT_G | PIN_NUM_15)

#define PH0     (PIN_PORT_H | PIN_NUM_0)
#define PH1     (PIN_PORT_H | PIN_NUM_1)
#define PH2     (PIN_PORT_H | PIN_NUM_2)
#define PH3     (PIN_PORT_H | PIN_NUM_3)
#define PH4     (PIN_PORT_H | PIN_NUM_4)
#define PH5     (PIN_PORT_H | PIN_NUM_5)
#define PH6     (PIN_PORT_H | PIN_NUM_6)
#define PH7     (PIN_PORT_H | PIN_NUM_7)
#define PH8     (PIN_PORT_H | PIN_NUM_8)
#define PH9     (PIN_PORT_H | PIN_NUM_9)
#define PH10    (PIN_PORT_H | PIN_NUM_10)
#define PH11    (PIN_PORT_H | PIN_NUM_11)
#define PH12    (PIN_PORT_H | PIN_NUM_12)
#define PH13    (PIN_PORT_H | PIN_NUM_13)
#define PH14    (PIN_PORT_H | PIN_NUM_14)
#define PH15    (PIN_PORT_H | PIN_NUM_15)

#define PI0     (PIN_PORT_I | PIN_NUM_0)
#define PI1     (PIN_PORT_I | PIN_NUM_1)
#define PI2     (PIN_PORT_I | PIN_NUM_2)
#define PI3     (PIN_PORT_I | PIN_NUM_3)
#define PI4     (PIN_PORT_I | PIN_NUM_4)
#define PI5     (PIN_PORT_I | PIN_NUM_5)
#define PI6     (PIN_PORT_I | PIN_NUM_6)
#define PI7     (PIN_PORT_I | PIN_NUM_7)
#define PI8     (PIN_PORT_I | PIN_NUM_8)
#define PI9     (PIN_PORT_I | PIN_NUM_9)
#define PI10    (PIN_PORT_I | PIN_NUM_10)
#define PI11    (PIN_PORT_I | PIN_NUM_11)
#define PI12    (PIN_PORT_I | PIN_NUM_12)
#define PI13    (PIN_PORT_I | PIN_NUM_13)
#define PI14    (PIN_PORT_I | PIN_NUM_14)
#define PI15    (PIN_PORT_I | PIN_NUM_15)

#define HH_ANALOG_IN           0b000000000000
#define HH_IN_FLOAT            0b010000000000
#define HH_IN_PULLUP           0b100000000000
#define HH_IN_PULLDOWN         0b100000000000
#define HH_OUT_PP_02MHZ        0b001000000000
#define HH_OUT_PP_10MHZ        0b000100000000
#define HH_OUT_PP_50MHZ        0b001100000000
#define HH_OUT_OD_02MHZ        0b011000000000
#define HH_OUT_OD_10MHZ        0b010100000000
#define HH_OUT_OD_50MHZ        0b011100000000
#define HH_AF_OPP_02MHZ        0b101000000000
#define HH_AF_OPP_10MHZ        0b100100000000
#define HH_AF_OPP_50MHZ        0b101100000000
#define HH_AF_OOD_02MHZ        0b111000000000
#define HH_AF_OOD_10MHZ        0b110100000000
#define HH_AF_OOD_50MHZ        0b111100000000

// abbreviations to allow for smaller init arrays below
#define MOT_EN      MOTOR_EN
#define X_FLT       X_FAULT
#define Y_FLT       Y_FAULT
#define Z_FLT       Z_FAULT
#define A_FLT       A_FAULT
#define B_FLT       B_FAULT
#define C_FLT       C_FAULT

#define PIN_NO_PIN PE0
#define LIM1        HH_LIM1 //PB6
#define LIM2        HH_LIM2 //PA10
#define RTD1        HH_RTD1 //PA1
#define RTD2        HH_RTD2 //PA0
#define HPA2        HH_PA2  //PA2
#define HPA3        HH_PA3  //PA3
#ifdef GB_DIAGS_NEED_SWD_PINS_FOR_DEBUG
#define SWC         HH_NO_PIN // HH_SWC //PA14
#define SWD         HH_NO_PIN // HH_SWD //PA13
#else
#define SWC         HH_SWC  //PA14
#define SWD         HH_SWD  //PA13
#endif


////////////////////////////////////////////////////////////////////////////////

#define  DIAG_STRING_START      ">GB:"
#define  DIAG_INDENT_STRING     "____"
#define  DIAG_STRING_SIZE       100

#ifdef USE_HYREL_IO
	#define BOOT1 PIN_UNDEFINED
#endif

#define DIAGS_NO_ADC_VALUE      (0xdead)

#define ADC_TOLERANCE   ((int)400)

#define ADC_GND     0.0, 0
#define ADC_GNDP    0.0, 0
#define ADC_GNDR    0.0, 533
#define ADC_3_3V    3.3, 3437   // divider built into board
#define ADC_5V      5.0, 3156   // simple 10K voltage divider to 2.5V ->  2.5/3.3*4095
#define ADC_5VR     5.0, 3370   // simple 10K voltage divider to 2.5VR ->  2.5/3.3*4095
#define ADC_5VP     5.0, 1241   // simple 10K voltage divider to 2.5VP ->  2.5/3.3*4095
#define ADC_12V     12.0, 2809  // local ADC with 10k pulldown and effective 43k pullup
#define ADC_12VR    12.0, 3052  // canbus on RTDx with 2K pullup to 3.3V from a 1.2K/300 divider)
#define ADC_12VP    12.0, 2978  // canbus on PAx with from a 1.2K/300 divider)
#define ADC_DRAIN   12.0, 984   // derived "pullup" from 12V (w/ 20K series and 10K pud on AIN3)
#define ADC_DRAINP  12.0, 2940  // derived "pullup" from 12V from a 1.2K/300 divider

typedef enum {
	RESULT_PASSED   = 0,
	RESULT_FAILED   = 1,
	RESULT_ERROR    = 2,
	UNKNOWN_RESULT  = 3,
} result_t;

#define MAX_ALLOWABLE_ERRORS    50

typedef enum {
	DIAG_LED_OFF = 0,
	DIAG_LED_NIGHT_RIDER,
	DIAG_LED_BLINK_SLOW,
	DIAG_LED_BLINK_FAST,
} diagLedMode_t;

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	SYS_ID  = CAN_SYSTEM_DEVICE_ID,
	CAN_MX = 11,
	CAN_MY = 12,
	CAN_MZ = 13,
	CAN_MA = 14,
	CAN_MB = 15,
	CAN_MC = 16,
	CAN_PX = 21,
	CAN_PY = 22,
	CAN_PZ = 23,
	CAN_PA = 24,
	CAN_PB = 25,
	CAN_PC = 26,
	CAN_UNKNOWN = 0xff,
} canDevice_t;

typedef struct {
	canDevice_t device;
	char        name[10];
} canDevNameStruct;

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	DIAG_ADC_NONE = 0,

	DIAG_ADC_LOCAL_PA0,
	DIAG_ADC_LOCAL_PA1,
	DIAG_ADC_LOCAL_PA2,
	DIAG_ADC_LOCAL_PA3,
	DIAG_ADC_LOCAL_PA4,
	DIAG_ADC_LOCAL_PA5,
	DIAG_ADC_LOCAL_PA6,
	DIAG_ADC_LOCAL_PA7,
	DIAG_ADC_LOCAL_PB0,
	DIAG_ADC_LOCAL_PB1,
	DIAG_ADC_LOCAL_PC0,
	DIAG_ADC_LOCAL_PC1,
	DIAG_ADC_LOCAL_PC2,
	DIAG_ADC_LOCAL_PC3,
	DIAG_ADC_LOCAL_PC4,
	DIAG_ADC_LOCAL_PC5,

	DIAG_ADC_LOCAL_PF6,
	DIAG_ADC_LOCAL_PF7,
	DIAG_ADC_LOCAL_PF8,
	DIAG_ADC_LOCAL_PF9,
	DIAG_ADC_LOCAL_PF10,
	DIAG_ADC_LOCAL_PF3,
	DIAG_ADC_LOCAL_PF4,
	DIAG_ADC_LOCAL_PF5,

	DIAG_ADC_CAN_RTD1,
	DIAG_ADC_CAN_RTD2,
	DIAG_ADC_CAN_PA2,
	DIAG_ADC_CAN_PA3,

	DIAG_ADC_UNKNOWN,
} diagAdcIndex_t;

typedef struct {
	diagAdcIndex_t  adcIndex;
	ADC_TypeDef     *hwADC;
	int             channel;
	pinType         pin;
	char            name[10];
} diagAdcInfoStruct;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum {
	DIAGS_SUBSTEST_NONE = 0,

	DIAGS_SUBSTEST_CAN_ADC_SETUP,
	DIAGS_SUBSTEST_CAN_ADC_READ_REQ,
	DIAGS_SUBSTEST_CAN_ADC_CHECK_RESULTS,

	DIAGS_SUBSTEST_LOCAL_ADC_SETUP,
	DIAGS_SUBSTEST_LOCAL_ADC_CONVERT,
	DIAGS_SUBSTEST_LOCAL_ADC_READ_REQ,
	DIAGS_SUBSTEST_LOCAL_ADC_CHECK_RESULTS,

	DIAGS_SUBSTEST_LOOPBACK_WRITE,
	DIAGS_SUBSTEST_LOOPBACK_SENDSTRING,
	DIAGS_SUBSTEST_LOOPBACK_READ,
	DIAGS_SUBSTEST_LOOPBACK_COMPARE,

	DIAGS_SUBSTEST_STEP_COUNT_SETUP,
	DIAGS_SUBSTEST_STEP_COUNT_HEADER,
	DIAGS_SUBSTEST_STEP_COUNT_CHECK_RESULTS,
	DIAGS_SUBSTEST_STEP_COUNT_FINISHED,
} subtest_t;

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	CONNECTIVITY_NONE = 0,

	CONNECTIVITY_GEN_0,
	CONNECTIVITY_GEN_1,
	CONNECTIVITY_GEN_2,
	CONNECTIVITY_GEN_3,
	CONNECTIVITY_GEN_4,
	CONNECTIVITY_GEN_5,
	CONNECTIVITY_GEN_6,
	CONNECTIVITY_GEN_7,
	CONNECTIVITY_GEN_8,
	CONNECTIVITY_GEN_9,

	CONNECTIVITY_MX_ENB,
	CONNECTIVITY_MX_DIR,
	CONNECTIVITY_MX_STP,
	CONNECTIVITY_MX_FLT,
	CONNECTIVITY_MX_HOM,
	CONNECTIVITY_MX_LM1,
	CONNECTIVITY_MX_LM2,
	CONNECTIVITY_PX_ENB,
	CONNECTIVITY_PX_DIR,
	CONNECTIVITY_PX_STP,
	CONNECTIVITY_PX_FLT,

	CONNECTIVITY_MY_ENB,
	CONNECTIVITY_MY_DIR,
	CONNECTIVITY_MY_STP,
	CONNECTIVITY_MY_FLT,
	CONNECTIVITY_MY_HOM,
	CONNECTIVITY_MY_LM1,
	CONNECTIVITY_MY_LM2,
	CONNECTIVITY_PY_ENB,
	CONNECTIVITY_PY_DIR,
	CONNECTIVITY_PY_STP,
	CONNECTIVITY_PY_FLT,

	CONNECTIVITY_MZ_ENB,
	CONNECTIVITY_MZ_DIR,
	CONNECTIVITY_MZ_STP,
	CONNECTIVITY_MZ_FLT,
	CONNECTIVITY_MZ_HOM,
	CONNECTIVITY_MZ_LM1,
	CONNECTIVITY_MZ_LM2,
	CONNECTIVITY_PZ_ENB,
	CONNECTIVITY_PZ_DIR,
	CONNECTIVITY_PZ_STP,
	CONNECTIVITY_PZ_FLT,

	CONNECTIVITY_MA_ENB,
	CONNECTIVITY_MA_DIR,
	CONNECTIVITY_MA_STP,
	CONNECTIVITY_MA_FLT,
	CONNECTIVITY_MA_HOM,
	CONNECTIVITY_MA_LM1,
	CONNECTIVITY_MA_LM2,
	CONNECTIVITY_PA_ENB,
	CONNECTIVITY_PA_DIR,
	CONNECTIVITY_PA_STP,
	CONNECTIVITY_PA_FLT,

	CONNECTIVITY_MB_ENB,
	CONNECTIVITY_MB_DIR,
	CONNECTIVITY_MB_STP,
	CONNECTIVITY_MB_FLT,
	CONNECTIVITY_MB_HOM,
	CONNECTIVITY_MB_LM1,
	CONNECTIVITY_MB_LM2,
	CONNECTIVITY_PB_ENB,
	CONNECTIVITY_PB_DIR,
	CONNECTIVITY_PB_STP,
	CONNECTIVITY_PB_FLT,

	CONNECTIVITY_MC_ENB,
	CONNECTIVITY_MC_DIR,
	CONNECTIVITY_MC_STP,
	CONNECTIVITY_MC_FLT,
	CONNECTIVITY_MC_HOM,
	CONNECTIVITY_MC_LM1,
	CONNECTIVITY_MC_LM2,
	CONNECTIVITY_PC_ENB,
	CONNECTIVITY_PC_DIR,
	CONNECTIVITY_PC_STP,
	CONNECTIVITY_PC_FLT,

	CONNECTIVITY_UNKNOWN,
} connectivityIndex_t;

#define LOOPBACK_STRING_SIZE 32

typedef struct {
	canDevice_t device;
	pinType     pin;
	char        connPinStr[LOOPBACK_STRING_SIZE];
} diagPinStruct;

typedef struct {
	connectivityIndex_t connectivityIndex;
	boolean         inverting;
	char            netName[LOOPBACK_STRING_SIZE];
	diagPinStruct   out;
	diagPinStruct   in;
} connectivityStruct;

////////////////////////////////////////////////////////////////////////////////

#define DIAGS_CAN_WRITE_TIME                10
#define DIAGS_CAN_READ_TIME                 10
#define DIAGS_LOCAL_READ_TIME               5

#define DIAGS_CAN_ADC_SETUP_TIME            100
#define DIAGS_LOCAL_ADC_SETUP_TIME          10
#define DIAGS_LOCAL_ADC_CONVERT_TIME        5

#define DIAGS_STRING_TIME                   40  // string limited ... need about 30ms to send 110 chars

#define DIAGS_LOCAL_ADC_TEST_TIME_MS        (DIAGS_LOCAL_ADC_SETUP_TIME + DIAGS_LOCAL_ADC_CONVERT_TIME + DIAGS_LOCAL_READ_TIME + DIAGS_STRING_TIME + 10)
#define DIAGS_CAN_ADC_TEST_TIME_MS          (DIAGS_CAN_ADC_SETUP_TIME + DIAGS_CAN_READ_TIME + DIAGS_STRING_TIME + 10 + DIAGS_CAN_WRITE_TIME)
#define DIAGS_RELAY_CHANGE_TIME_MS          2000
#define DIAGS_MOVE_TIME_MS                  1500

#define DIAGS_TEST_STEPS_TIME_MS            (DIAGS_CAN_WRITE_TIME + (5 * DIAGS_STRING_TIME) + 10)
#define DIAGS_LOOPBACK_TEST_TIME_MS         (DIAGS_STRING_TIME + DIAGS_CAN_WRITE_TIME + DIAGS_CAN_READ_TIME + 10)

#define DIAGS_MS_PER_LOOP                   1   // change depending on sysClk loop for ProcessDiags call
#define DIAGS_SECONDS_TO_REGISTER           6
#define DIAGS_FIRST_TEST_TIME               100 // 100ms pause before starting

#define US_PER_TICK 0.01388888f // used for STEP timing
////////////////////////////////////////////////////////////////////////////////

typedef struct {
	uint16_t testNum;
	uint16_t testInit;
	uint32_t timeMs;
} testStruct;

////////////////////////////////////////////////////////////////////////////////

extern boolean _diagsEnabled;
extern diagLedMode_t _diagsLedDisplayMode;
extern pinStateValue_t  _diagsConnectivityReadVal;
extern uint16_t     _diagsAdcReadValue;

extern int _diagsStepsDetected;
extern float _diagsStepMinTimeUs;
extern float _diagsStepMaxTimeUs;
extern float _diagsStepsAvgTimeUs;

extern void ProcessDiags(void);
extern void diagLedDisplay(void);
extern void runHydraDiagnostics(void);
extern void diagsExit(boolean);
