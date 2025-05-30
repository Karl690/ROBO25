////////////////////////////////////////////////////////////////////////////////
//
// File:    main.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: The
//
////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

// add includes here
#include "main.h"
#include "gpio.h"
#include "Serial.h"
#include "Hardwareinit.h"
#include "mailbox.h"            // outboxStructure
#include "Hydra_can.h"          // NUM_PRE_DEFINED_ALIASES
#include "MotorDriver.h"
#include "GCode.h"
#include "usbd_usr.h"
#include "stm32f4xx_syscfg.h"
#include "gui.h"
#include "display.h"
#include "pnp.h"
#include "adc.h"
#include "display/lcdspi_4xx.h"
#include "display/display.h"
#include "util.h"
void(*callback)(void *); //define cast for call back type
void initKey(boolean a, char *b, char *c)
{
}

#ifdef HYDRA_DIAGS
#include "diags.h"
#endif


////////////////////////////////////////////////////////////////////////////////
//
//  Local #defines
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//  Global Variables Referenced
//
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//
//  Public Global Definitions (exposed in main.h)
//
////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_CRASH_LOGGING
char crash_source[32]="";
#endif

globalStruct _gs;                     // control index, canbus ques and other stuff
MotorStructure Motors[MAX_NUMBER_OF_MOTORS];
uint32_t HeartBeat = 0;
boolean ForceReportXYZLocation=FALSE;
boolean AutoReportXYZLocation=FALSE;
boolean AutoReportFeedRate=FALSE;
boolean AutoReportLineNum=FALSE;
boolean StatusReportXYZLocation=FALSE;
boolean StatusReportVelocity=FALSE;
boolean StatusReportFlowRate=FALSE;
boolean StatusReportLineNum=FALSE;

SPI_LCD_HandleTypeDef LCDSpi1;
float _LastReportedCurrentRequestedFeedrateInMmPerSec = -1.0f;
int _LastReportedExecutingLineNumber = -1;
int PwmTestCounter = 0;
int	SpindleDesiredSpeedPWM = 0;
int	CO2LaserAnalogPwrPWM = 0;
int Co2LaserWatchDogTimer = 0;
int RPMCounter = 0;
//osseo variables
int EnableOsseoVariablesReporting = 0;
int ParticleCounter = 666;
float EnclosureTemperature = 26.3;
int EnclosureHumidity = 99;
int EnclosurePressureDifference = -1;
int EnclosureUvLedPwm = 0;
int EnclosureFanPwm =0;
int EnclosureDoorLock = 0;
uint8_t EnclosureDoorSense = 1;
int doorSenseState = 0;
uint8_t previous_EnclosureDoorSense = 1;
int EnclosureCartridgeSense = 0;
int EnclosurePrintBedSense = 0;


//607 is dcodedrainstate[0]...
int McodeDrainState[9] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int Update595Index = 0;
boolean _printAir=FALSE;
float _autoReverseAndPrimeMinTime=0.0f;    // single non-E of less than this time between two print moves do not generate unprime/prime pair
E_control_t _extrusionControl;
E_control_t _saveExtrusionControl;
motionForFlowControl_t _motionForFlowControl = MOTION_FLOW_USE_FEEDRATE;
unprime_prime_control_t _unprimePrimeControl;
boolean _hijackAxisC = FALSE;
boolean _canbusStepForE = FALSE;
boolean _directStepForE = FALSE;
float E_Q_LastRequestedPositionInUnits = 0.0f;
int E_Q_POSITION = 0;
int CurrentLayer=0;
int _gcodeLineNumber=0; // increments each command line, but also can be set with ARG_N; resets to 0 on M30
int _gcodeMotionLineNumber=0;
float TurboModeDistance=MAXFLOAT; // this effectively disables turbo mode by default.
boolean _reverseFrogToe=FALSE;
boolean _spiralRoughingPass=FALSE;  // SPIRAL
float _deltaRForPass;  // SPIRAL
int G2G3Flag=0;
int G203State=0;
boolean _motionQ_LookAheadDisabled=FALSE; //GB XXX gregkarl -- should default to TRUE (set to FALSE for DEBUG)
boolean _motionQ_ForceQtoEmpty=FALSE;
boolean _blockImmediateMoveOnToolChange=FALSE; // Hydra3 default behavior

int dump407SoapstringFlag=0;
int DDLightFunction=0;

int PrimeTimer=0;//used to keep track of how long since the prime was issued so we can start the motion
int UnPrimeTimer=0;//used to keep track of how long since the unprime was issued
int HostConnectionWatchDog=0;//watchdog to prevent lava flow from head when communincation breaks.

int currentToolIndex=0;
int currentFixtureIndex=0;
int currentHeadIndex=0;

GMCommandStructure *CannedCyclePtr;
int CannedCycleLastMove=TRUE; // always leave set when not doing a canned cycle) (makes the last in a series of cmds to know whick moves get attached to the deferred queue
int CannedCycleFlag=0;
int CannedCycleStep=0;
float CannedCycleZFeedRateInMmPerSec=0.83333f;  // 50.0f / 60.0f;
float CannedZDepthInMM=0.0f;
float CannedZDesiredDepth=0.0f;
float CannedZIncrement=0.0f;
float CannedZClearInMM=1.0f;
int CannedLinearAxis=M_Z;
int CannedRotaryAxis=M_A;
float CannedZQIncrement=0;
float CannedThreadPitch=1.0f;

HssPwmStruct *CannedHssPtr=NULL;
boolean CannedCyclePuffFlag=FALSE;

int _repetrelCommWatchCount = REPETREL_COMM_WATCHDOG_START_VALUE;

boolean _waitingForGuiCommand = FALSE;
int _abortInProgress = 0;   // flag/watchdog counter in one
int _abortCameFromRx = 0; // flag to know if abort was user requested or self inflicted
boolean _abortFinisedNeedToResetProcessSynchronously = FALSE;

boolean _blockAllMotion=FALSE;  // set true when a new motion fault detect and held on until the abort is complete; also used when EMO pressed
boolean _blockAbsoluteMotion=FALSE;
boolean _blockAllMotionMessageSent=FALSE;
boolean _blockAbsoluteMotionMessageSent=FALSE;
boolean _blockMotionForJoggingMessageSent=FALSE;
boolean _blockMotionForHighVoltageMessageSent=FALSE;
boolean _blockJoggingForMotionMessageSent=FALSE;

boolean _motionSensorTripped=FALSE; //any of the possible 18 sensors has tripped 6 Axis x (FAULT, LIMIT1, LIMIT2)
boolean _motionSensorTrippedNext=FALSE;

boolean _gcodePaused=FALSE;
uint32_t _g4DwellTimer=0;//used for causing a delay before going to next step in program
boolean _requestToPauseAtEndOfMove=FALSE;
boolean _requestToPauseAtEndOfLayer=FALSE;
boolean _requestToAbortAtEndOfMove=FALSE;
boolean _abortOccurredWhileMoving=FALSE;

int32_t _needToWiggleDirectionPins=2;

boolean _validFirmwareKey=FALSE; // only for tracking, not for control
uint32_t Head11_Temperature = 0;
uint32_t Head11_HTRDuty = 0;
uint32_t Head11_FanDuty = 0;
uint32_t Head11_Spare = 0;

char GCodeArgComment[COMMENT_STRING_LENGTH]= ";Comment  ";//buffer mismatch, working buffer is only 1024 long
char _GcodeArgStringParam[GCODE_STRING_ARG_LENGTH];

int PreProcessNextCodeFlag=0;
int CommandsInQue;

int toolNumberMap[] = { 11, 12, 13, 14, 15,           // 0 to 4 on yoke 1
						21, 22, 23, 24, 25,           // 5 to 9 on yoke 2
						16,                           // 10 == AUX
						41,                           // 11 == CO2 laser
						26,                           // 12 == 3 phase spindle
						91,                           // 13 == hotbed1
						92                            // 14 == hotbed2
					  };

int MAX_TOOL_NUMBER = (sizeof(toolNumberMap) / sizeof(int)) - 1;

ToolOffsetStructure ToolOffsets[NUM_TOOL_OFFSETS];

#ifdef ALLOW_NATIVE_LIGHTBURN
sensorState_t _lightburnBootModeSensorState = SENSOR_CLOSED;
boolean _lightburnModeEnabled = FALSE;
int volatile _bootCounterMs = 0;
boolean _useTimer7ToSenseStartPin = TRUE;
#else //!ALLOW_NATIVE_LIGHTBURN
boolean _useTimer7ToSenseStartPin = FALSE;
#endif //!ALLOW_NATIVE_LIGHTBURN

boolean _Tim7StillCalculating = FALSE;

sensorStruct startButton;
sensorStruct EMO;
#ifdef USE_AB_ENCODER
sensorStruct ABEncoderSelectButton;
#endif //USE_AB_ENCODER
int _highVoltageIsNotStableCoundownMs = BOOTUP_HIGH_VOLTAGE_SETTLING_TIME_MS;

#ifdef COLLECT_METRICS
metricsStruct _metrics;
#endif


boolean _TouchProbeMoveActive = FALSE;
byte _TouchProbeCanDevice=0;
int _TouchProbeEXTI_Line;
uint8_t _TouchProbeNVIC_IRQChannel;
pinType _TouchProbePin=PIN_UNDEFINED;

boolean _edgeTriggerArmed = FALSE;
boolean _edgeTriggerDetected = FALSE;
int _edgeTriggerSignalState;
int _edgeTriggerDisplayRateCnt;
float _edgeTriggerDisplayRateHz;

int pendingAcknowledge = 0;
int _gcodeCmdsReceived = 0;
int _gcodeAcksSent = 0;
unsigned int _asciiChecksum32=0;
unsigned int _rejected_normalTxChars=0;
unsigned int _asciiCharsRcvd=0;
int normalRxBufHeadroom;

float _FlowRateOverridePct; // affects all head equally
float _FeedRateOverridePct;         // will scale both motion and flow together
float _MotionRateOverridePct;       // will scale motion only
float _CompositeFlowOverridePct;
float _CompositeMotionOverridePct;
int _serialPortRxOverrunCnt=0;
char resetSourceStr[256];  // built up string containing the possible reset source

errorSentStruct _errors;
int _heartbeatRateControl = HEARTBEAT_MODE_BOOT; // mod of 100Hz counter to toggle LED
int32_t lsiActualFreq = 0;
uint32_t lsiCC4sampleIndex = 0;
uint32_t lsiCC4samples[2] = {0, 0};

int homeSensedAxisCntr=0;;

int hssFuncToPinIndex[NUM_HSS_FUNC];
HssPwmStruct HighSideSwitches[NUM_HSS_PINS];

#ifdef ENABLE_CRASH_LOGGING
char blankChar = SPACE_CHAR;
const crashLog_struct _crashLog[] = {
		//"123456789.123456789.123456789.1", "123456789.12345", addr,  }
		{ "slice_1Hz",                       FORMAT_U8_DEC,     (byte *)&_gs._ctrlIndex[HZ_1] },
		{ "slice_10Hz",                      FORMAT_U8_DEC,     (byte *)&_gs._ctrlIndex[HZ_10] },
		{ "slice_100Hz",                     FORMAT_U8_DEC,     (byte *)&_gs._ctrlIndex[HZ_100] },
		{ "slice_1000Hz",                    FORMAT_U8_DEC,     (byte *)&_gs._ctrlIndex[HZ_1000] },
		{ "lsiActualFreq",                   FORMAT_U32_DEC,    (byte *)&lsiActualFreq },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },

		{ "CAN1->MCR",                       FORMAT_U32_HEX,    (byte *)&CAN1->MCR },
		{ "CAN1->MSR",                       FORMAT_U32_HEX,    (byte *)&CAN1->MSR },
		{ "CAN1->TSR",                       FORMAT_U32_HEX,    (byte *)&CAN1->TSR },
		{ "CAN1->IER",                       FORMAT_U32_HEX,    (byte *)&CAN1->IER },
		{ "CAN1->ESR",                       FORMAT_U32_HEX,    (byte *)&CAN1->ESR },
		{ "CAN1->BTR",                       FORMAT_U32_HEX,    (byte *)&CAN1->BTR },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },

		{ "CAN1->sTxMailBox[0].TIR",         FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[0].TIR },
		{ "CAN1->sTxMailBox[0].TDTR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[0].TDTR },
		{ "CAN1->sTxMailBox[0].TDLR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[0].TDLR },
		{ "CAN1->sTxMailBox[0].TDHR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[0].TDHR },
		{ "CAN1->sTxMailBox[1].TIR",         FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[1].TIR },
		{ "CAN1->sTxMailBox[1].TDTR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[1].TDTR },
		{ "CAN1->sTxMailBox[1].TDLR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[1].TDLR },
		{ "CAN1->sTxMailBox[1].TDHR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[1].TDHR },
		{ "CAN1->sTxMailBox[2].TIR",         FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[2].TIR },
		{ "CAN1->sTxMailBox[2].TDTR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[2].TDTR },
		{ "CAN1->sTxMailBox[2].TDLR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[2].TDLR },
		{ "CAN1->sTxMailBox[2].TDHR",        FORMAT_U32_HEX,    (byte *)&CAN1->sTxMailBox[2].TDHR },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
#ifdef COLLECT_METRICS
		{ "total_commandsProcessed",         FORMAT_U32_DEC,    (byte *)&_metrics.total_commandsProcessed },
		{ "total_motionQprocessed",          FORMAT_U32_DEC,    (byte *)&_metrics.total_motionQprocessed },
		{ "total_DeferredProcessed",         FORMAT_U32_DEC,    (byte *)&_metrics.total_DeferredProcessed },
		{ "total_charsRx",                   FORMAT_U32_DEC,    (byte *)&_metrics.total_charsRx },
		{ "total_charsTx",                   FORMAT_U32_DEC,    (byte *)&_metrics.total_charsTx },
		{ "total_canRx",                     FORMAT_U32_DEC,    (byte *)&_metrics.total_canRx },
		{ "total_canTx",                     FORMAT_U32_DEC,    (byte *)&_metrics.total_canTx },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "max_CommandsInQue",               FORMAT_U32_DEC,    (byte *)&_metrics.max_CommandsInQue },
		{ "max_motionQvalidEntries",         FORMAT_U32_DEC,    (byte *)&_metrics.max_motionQvalidEntries },
		{ "max_DeferredCommandsInQue",       FORMAT_U32_DEC,    (byte *)&_metrics.max_DeferredCommandsInQue },
		{ "max_rawRxCharsInBuf",             FORMAT_U32_DEC,    (byte *)&_metrics.max_rawRxCharsInBuf },
		{ "max_urgentRxCharsInBuf",          FORMAT_U32_DEC,    (byte *)&_metrics.max_urgentRxCharsInBuf },
		{ "max_normalRxCharsInBuf",          FORMAT_U32_DEC,    (byte *)&_metrics.max_normalRxCharsInBuf },
		{ "max_directRxCharsInBuf",          FORMAT_U32_DEC,    (byte *)&_metrics.max_directRxCharsInBuf },
		{ "max_normalTxCharsInBuf",          FORMAT_U32_DEC,    (byte *)&_metrics.max_normalTxCharsInBuf },
		{ "max_canRxQ_numMsg",               FORMAT_U32_DEC,    (byte *)&_metrics.max_canRxQ_numMsg },
		{ "max_canTxQ_numMsg",               FORMAT_U32_DEC,    (byte *)&_metrics.max_canTxQ_numMsg },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
#endif
		{ "curr_CommandsInQue",              FORMAT_U32_DEC,    (byte *)&CommandsInQue },
		{ "curr_motionQ.validEntries",       FORMAT_U32_DEC,    (byte *)&motionQ.validEntries },
		{ "curr_DeferredCommandsInQue",      FORMAT_U32_DEC,    (byte *)&DeferredCommandsInQue },
		{ "curr_rawRxCharsInBuf",            FORMAT_U32_DEC,    (byte *)&rawRxCharsInBuf },
		{ "curr_urgentRxCharsInBuf",         FORMAT_U32_DEC,    (byte *)&urgentRxCharsInBuf },
		{ "curr_normalRxCharsInBuf",         FORMAT_U32_DEC,    (byte *)&normalRxCharsInBuf },
		{ "curr_normalTxCharsInBuf",         FORMAT_U32_DEC,    (byte *)&normalTxCharsInBuf },
		{ "curr_gs._canRxQ.numMsg",          FORMAT_U32_DEC,    (byte *)&_gs._canRxQ.numMsg },
		{ "curr_gs._canRxQ.numMsg",          FORMAT_U32_DEC,    (byte *)&_gs._canTxQ.numMsg  },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "curr_rawRxIndexIn",               FORMAT_U32_DEC,    (byte *)&rawRxIndexIn },
		{ "curr_rawRxIndexOut",              FORMAT_U32_DEC,    (byte *)&rawRxIndexOut },
		{ "curr_urgentRxIndexIn",            FORMAT_U32_DEC,    (byte *)&urgentRxIndexIn },
		{ "curr_urgentRxIndexOut",           FORMAT_U32_DEC,    (byte *)&urgentRxIndexOut },
		{ "curr_normalRxIndexIn",            FORMAT_U32_DEC,    (byte *)&normalRxIndexIn },
		{ "curr_normalRxIndexOut",           FORMAT_U32_DEC,    (byte *)&normalRxIndexOut },
		{ "curr_normalTxIndexIn",            FORMAT_U32_DEC,    (byte *)&normalTxIndexIn },
		{ "curr_normalTxIndexOut",           FORMAT_U32_DEC,    (byte *)&normalTxIndexOut },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "_serialPortRxOverrunCnt",            FORMAT_U32_DEC,    (byte *)&_serialPortRxOverrunCnt },
#ifdef COLLECT_METRICS
		{ "flushedRxCharsDuringAbort",       FORMAT_U32_DEC,    (byte *)&_metrics.flushedRxCharsDuringAbort },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
#endif
		{ "pendingAcknowledge",              FORMAT_U32_DEC,    (byte *)&pendingAcknowledge },
		{ "_gcodeCmdsReceived",              FORMAT_U32_DEC,    (byte *)&_gcodeCmdsReceived },
		{ "_gcodeAcksSent",                  FORMAT_U32_DEC,    (byte *)&_gcodeAcksSent },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },

		{ "_g4DwellTimer",                   FORMAT_U32_DEC,    (byte *)&_g4DwellTimer },
		{ "_gcodePaused",                    FORMAT_U8_DEC,     (byte *)&_gcodePaused },
		{ "_abortInProgress",                FORMAT_U8_DEC,     (byte *)&_abortInProgress },

		{ "_blockAllMotion",                 FORMAT_U8_DEC,     (byte *)&_blockAllMotion },
		{ "_blockAbsoluteMotion",            FORMAT_U8_DEC,     (byte *)&_blockAbsoluteMotion },
		{ "_motionSensorTripped",            FORMAT_U8_DEC,     (byte *)&_motionSensorTripped },
		{ "_waitingFor.flags.u32",           FORMAT_U32_HEX,    (byte *)&_MailBoxes._waitingFor.flags.u32 },
		{ "_needToProcessDeferredCommands",  FORMAT_U8_DEC,     (byte *)&_needToProcessDeferredCommands },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "X.HomeSense.State",               FORMAT_U8_DEC,     (byte *)&Motors[M_X].HomeSense.State },
		{ "Y.HomeSense.State",               FORMAT_U8_DEC,     (byte *)&Motors[M_Y].HomeSense.State },
		{ "Z.HomeSense.State",               FORMAT_U8_DEC,     (byte *)&Motors[M_Z].HomeSense.State },
		{ "A.HomeSense.State",               FORMAT_U8_DEC,     (byte *)&Motors[M_A].HomeSense.State },
		{ "B.HomeSense.State",               FORMAT_U8_DEC,     (byte *)&Motors[M_B].HomeSense.State },
		{ "C.HomeSense.State",               FORMAT_U8_DEC,     (byte *)&Motors[M_C].HomeSense.State },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "X.FaultSense.State",              FORMAT_U8_DEC,     (byte *)&Motors[M_X].FaultSense.State },
		{ "Y.FaultSense.State",              FORMAT_U8_DEC,     (byte *)&Motors[M_Y].FaultSense.State },
		{ "Z.FaultSense.State",              FORMAT_U8_DEC,     (byte *)&Motors[M_Z].FaultSense.State },
		{ "A.FaultSense.State",              FORMAT_U8_DEC,     (byte *)&Motors[M_A].FaultSense.State },
		{ "B.FaultSense.State",              FORMAT_U8_DEC,     (byte *)&Motors[M_B].FaultSense.State },
		{ "C.FaultSense.State",              FORMAT_U8_DEC,     (byte *)&Motors[M_C].FaultSense.State },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "X.Limit1Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_X].Limit1Sense.State },
		{ "Y.Limit1Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_Y].Limit1Sense.State },
		{ "Z.Limit1Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_Z].Limit1Sense.State },
		{ "A.Limit1Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_A].Limit1Sense.State },
		{ "B.Limit1Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_B].Limit1Sense.State },
		{ "C.Limit1Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_C].Limit1Sense.State },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "X.Limit2Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_X].Limit2Sense.State },
		{ "Y.Limit2Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_Y].Limit2Sense.State },
		{ "Z.Limit2Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_Z].Limit2Sense.State },
		{ "A.Limit2Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_A].Limit2Sense.State },
		{ "B.Limit2Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_B].Limit2Sense.State },
		{ "C.Limit2Sense.State",             FORMAT_U8_DEC,     (byte *)&Motors[M_C].Limit2Sense.State },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "X.HasBeenHomed",                  FORMAT_U8_DEC,     (byte *)&Motors[M_X].HasBeenHomed },
		{ "Y.HasBeenHomed",                  FORMAT_U8_DEC,     (byte *)&Motors[M_Y].HasBeenHomed },
		{ "Z.HasBeenHomed",                  FORMAT_U8_DEC,     (byte *)&Motors[M_Z].HasBeenHomed },
		{ "A.HasBeenHomed",                  FORMAT_U8_DEC,     (byte *)&Motors[M_A].HasBeenHomed },
		{ "B.HasBeenHomed",                  FORMAT_U8_DEC,     (byte *)&Motors[M_B].HasBeenHomed },
		{ "C.HasBeenHomed",                  FORMAT_U8_DEC,     (byte *)&Motors[M_C].HasBeenHomed },
		{ "",                                FORMAT_I8_CHAR,    (byte *)&blankChar },
		{ "_validFirmwareKey",               FORMAT_U8_DEC,    (byte *)&_validFirmwareKey },
};
const int NUM_CRASHLOG_ENTRIES = sizeof(_crashLog) / sizeof(crashLog_struct);
#endif


////////////////////////////////////////////////////////////////////////////////
//
//  Local Global Definitions (do not expose in main.h)
//
////////////////////////////////////////////////////////////////////////////////

int CommandReadyToProcessFlag=0;
boolean _serialPortRxOverrunFlag=FALSE;

////////////////////////////////////////////////////////////////////////////////

//here is the section to handle the command process que,

// this is working que, commands are parsed and loaded into here for sequential exection
// last one is for execute immediate
// 0 offset in cmdQue is the current instruction conversion results, it is also used for immediate
// execution once the parsing is finished
// for variables that are not to be used we will assign a value of -9999999 this should avoid numeric
// conflicts with movement.
/***
 * Case 1 command is received and parsed and will be executed immediately
 * this is accomplished by loading cmdQue[0] with parsed information and setting the execute immediate flag to 1
 * on the next executeCommand slice, the flag will be checked,
 * CurrentCommandIndex will be set to 0 then executed,
 * CurrentCommandIndex will be set to 0
 * when the que index pointer reaches 100 it will be reset to 1, not 0
 *
 * Case 2 Command is received and parsed and will be placed on que for sequential execution
 * here the values to be parsed into cmdQue[0] and then copied to cmdQue[NextCommandInsertionIndex]
 * NextCommandIndex will be incremented and then reset if it exceeds 100
 *
 * when execueCommand runs, it will place nextcommandPointer into CurrentCommandIndex and then inc NextCommandIndex
 *
 */

motionQStruct motionQ;
GMCommandStructure cmdQue[SIZE_OF_COMMAND_QUEUE];
//NUKE GMCommandStructure *ExecutionPtr = &cmdQue[0];
GMCommandStructure *ExecutionPtr;
GMCommandStructure *NextExecutionPtr;   //NextExecutionPtr=&cmdQue[NextCommandIndex]; tmp pointer
int NextCommandIndex;         // index of the next command to be executed (the one after Current), initially set to 2 after the first command is received and parsed
int CurrentCommandIndex;      // index of the current place in cmdQue[] from which to get prams for execution
int NextCommandInsertionIndex;// index of the next free location to add an incomming command
char *GCodeArgPtr;
//unsigned int counter_1Hz;
//unsigned int counter_10Hz;
//unsigned int counter_100Hz;
//unsigned int counter_1000Hz;


GMDeferredCommandStructure deferredCmdQue[SIZE_OF_DEFERRED_COMMAND_QUEUE];
int NextDeferredCommandIndex;         // index of the next command to be executed (the one after Current), initially set to 2 after the first command is received and parsed
int NextDeferredCommandInsertionIndex;// index of the next free location to add an incomming command
int DeferredCommandsInQue;
int LastCmdQueIndexAddedToMotionQ;
boolean _needToProcessDeferredCommands = FALSE;
int ExecuteImmediateFlag;       // 0 means normal queing, 1 means next received command will go to front of execution, 2 means ready to execute immediate

unsigned int counter_1Hz=0;
unsigned int counter_10Hz=0;
unsigned int counter_100Hz=0;
unsigned int counter_1000Hz=0;

////////////////////////////////////////////////////////////////////////////////
//
//  Forward Declarations
//
////////////////////////////////////////////////////////////////////////////////

void ohNoMrBill(void);
void CommandProcessor(void);
void SequenceEngine(void);
void serialProcessor(void);
void SpeedControl(void);
void sendOutgoingMail(void);
void DDLightSelection(void);

void readInputs(void);
void checkMotorFaultSensor(void);
void checkMotorLimit1Sensor(void);
void checkMotorLimit2Sensor(void);
void spare(void);
void sendUpdateToHost(void);
void heartbeat(void);
void processEverySlice(void);
void loop_1000Hz_simple_work(void);
void loop_100Hz_simple_work(void);
void loop_10Hz_simple_work(void);
void loop_1Hz_simple_work(void);
void systicker(void);
void checkForMia(void);
void reportCalibrationTemperature(void);
void ReportXYZLocation(void);
void ReportOsseoVariables(void);
void FastLigthControl(int);
void CleanUpPointers(void);
void FillArgumentBuffer(char *, char);
void ContinueToNextStep(void);
void AddCommandToQue(command_t);
void initGCodeSequenceEngine(void);
void ProcessG28MultipassHoming(void);
void ProcessG38MultipassProbing(void);
void DwellTimer(void);
void Hss100HzControl(void);
void forceHssPwmOff(HssPwmStruct *hss);
void forceHssPwmOn(HssPwmStruct *hss);
void EchoBackCommentString(void);
void sendPendingAcks(void);
void checkBlockingWaits(void);
void sendEmoMessage(void);
void checkForCompletedAbort(void);
void setAllMotionSensorsToUnknownState(void);

int EnableParallelPrintHeads=0;//each bit will enable that specific head for prime/unprime.
int PauseFlag=0;

int currentArgLength=0;//used for preventing overrun on characters
int ProcessingError=0;
uint32_t _lineChecksum=0;
boolean _checkLineChecksum=FALSE;

char Headnumber= 0;
char HeadLetter=0;
// declared in public globals: char GCodeArgComment[COMMENT_STRING_LENGTH]= ";Comment  ";//buffer mismatch, working buffer is only 1024 long

#define MAX_CHARS_FOR_PARAMETER 20
char GCodeArgA[MAX_CHARS_FOR_PARAMETER]     = "A              ";
char GCodeArgB[MAX_CHARS_FOR_PARAMETER]     = "B              ";
char GCodeArgC[MAX_CHARS_FOR_PARAMETER]     = "C              ";
char GCodeArgD[MAX_CHARS_FOR_PARAMETER]     = "D              ";
char GCodeArgE[MAX_CHARS_FOR_PARAMETER]     = "E              ";
char GCodeArgF[MAX_CHARS_FOR_PARAMETER]     = "F              ";
char GCodeArgG[MAX_CHARS_FOR_PARAMETER]     = "G              ";
char GCodeArgH[MAX_CHARS_FOR_PARAMETER]     = "H              ";
char GCodeArgI[MAX_CHARS_FOR_PARAMETER]     = "I              ";
char GCodeArgJ[MAX_CHARS_FOR_PARAMETER]     = "J              ";
char GCodeArgK[MAX_CHARS_FOR_PARAMETER]     = "K              ";
char GCodeArgL[MAX_CHARS_FOR_PARAMETER]     = "L              ";
char GCodeArgM[MAX_CHARS_FOR_PARAMETER]     = "M              ";
char GCodeArgN[MAX_CHARS_FOR_PARAMETER]     = "N              "; //line number
char GCodeArgO[MAX_CHARS_FOR_PARAMETER]     = "O              ";
char GCodeArgP[MAX_CHARS_FOR_PARAMETER]     = "P              ";
char GCodeArgQ[MAX_CHARS_FOR_PARAMETER]     = "Q              "; //checksum -- using 'Q'
char GCodeArgR[MAX_CHARS_FOR_PARAMETER]     = "R              ";
char GCodeArgS[MAX_CHARS_FOR_PARAMETER]     = "S              ";
char GCodeArgT[MAX_CHARS_FOR_PARAMETER]     = "T              ";
char GCodeArgU[MAX_CHARS_FOR_PARAMETER]     = "U              ";
char GCodeArgV[MAX_CHARS_FOR_PARAMETER]     = "V              ";
char GCodeArgW[MAX_CHARS_FOR_PARAMETER]     = "W              ";
char GCodeArgX[MAX_CHARS_FOR_PARAMETER]     = "X              ";
char GCodeArgY[MAX_CHARS_FOR_PARAMETER]     = "Y              ";
char GCodeArgZ[MAX_CHARS_FOR_PARAMETER]     = "Z              ";
char GCodeArgSplat[MAX_CHARS_FOR_PARAMETER] = "*              ";

boolean _reportCalibrationTemperature = FALSE;
void reportCalibrationTemperature(void);
int ClocksFailedToInitialize = 0;

char _bootupAlertHostChar;
boolean _sendBootupAlertHostChar;
pinType HEARTBEAT_PIN;  // all remapping of pin for special buiilds



void getNextRasterValue(void)
{
	unsigned int dotSubPosition = _gs._laser.rasterTraverseDotCounter % _gs._laser.rasterImageDotsPerBinaryValue;

	if (dotSubPosition == 0)
	{   // only read when need new binary value (due to packing)
		_gs._laser.rasterActiveBinaryValue = DirectGCHAR();
		if (_gs._laser.rasterActiveBinaryValue == -1)
		{   //-1 marks CMD_END_CHAR -- so now we are finished with the transfer so we can turn off the laser and other flags
			_gs._laser.rasterLineComplete = TRUE;
			if (_gs._laser.localControl)
			{   // reset for vector mode
				InitEncoderTimer5();
			}
			_gs._laser.rasterActiveBinaryValue = _gs._laser.rasterOffValue; // force off
#ifdef GB_RASTER_PIXEL_PIN
			pinClear(GB_RASTER_PIXEL_PIN);
#endif //GB_RASTER_PIXEL_PIN
		}
#ifdef GB_RASTER_PIXEL_PIN
		else
		{
			pinToggleOutput(GB_RASTER_PIXEL_PIN);
		}
#endif //GB_RASTER_PIXEL_PIN
	}
	unsigned int dotValueI;
	float dotValueF;
	if ((_gs._laser.rasterTraverseDotCounter < _gs._laser.rasterFirstActiveDot) || (_gs._laser.rasterTraverseDotCounter > _gs._laser.rasterLastActiveDot))
		dotValueI = _gs._laser.rasterOffValue; // force off
	else
		dotValueI = _gs._laser.rasterActiveBinaryValue;

	unsigned int shiftAmount = ((_gs._laser.rasterImageDotsPerBinaryValue - 1) - dotSubPosition) * _gs._laser.rasterBitsPerDot;
	dotValueI = (dotValueI >> shiftAmount) & _gs._laser.rasterBinaryValueMask;

	if (_gs._laser.rasterUseColorIndexTable)
		dotValueF = ((float)(_gs._laser.rasterColorIndexTable[dotValueI])) / 255.0f;
	else
		dotValueF = (float)dotValueI / ((1 << _gs._laser.rasterBitsPerDot) - 1);    // scale to fill range of 0 to 1.0 no matter the number of input pix
	if (_gs._laser.rasterInvertData)
		dotValueF = (-1.0f * dotValueF) + 1.0f;
	float laserPower = fFitWithinRange(dotValueF * _FlowRateOverridePct, 0.0f, 1.0f);
	laserPower = powf(laserPower, _gs._laser.gammaValue);       // perform gamma
	laserPower = fFitWithinRange(laserPower, 0.0f, _gs._laser.rasterMaxPowerPercent);

	//NUKE SetLaserPowerOnly(laserPower); //send the modified data directly to the laser control
	_gs._laser.rasterTraverseDotCounter++;
	_gs._laser.nextValue = laserPower;
}

void processRasterStep(void)
{
	if (_gs._laser.rasterPulsesPerDotCounter > 0)
	{	// still working on raster data
		if (_gs._laser.rasterFrontPorchDotsCounter == 0)
		{	// finished with front porch
			if (_gs._laser.rasterPulsesPerDotCounter == _gs._laser.rasterPulseToFireOn)
			{	// first pulse of new dot, so send out data
				SetLaserPowerOnly(_gs._laser.nextValue);
				if (_gs._laser.rasterLineComplete)
				{
					_gs._laser.rasterizeCurrentMove = FALSE;  // turn off rasterizing....done with this line
				}
			}
		}

		_gs._laser.rasterPulsesPerDotCounter--;	// wait the number of steps for a full dot
		if (_gs._laser.rasterPulsesPerDotCounter == 0)
		{   // done with current dot
			_gs._laser.rasterPulsesPerDotCounter = _gs._laser.rasterPulsesPerDot;	// reload the sub dot counter
			if (_gs._laser.rasterFrontPorchDotsCounter > 0)
			{   // wait until the front porch pixels have been passed (waiting for motion to get up to speed)
				_gs._laser.rasterFrontPorchDotsCounter--;
			}
			if (_gs._laser.rasterFrontPorchDotsCounter == 0)
			{	// finished last subdot -- next dot will print if there's work to do on the line
				// so pre-emptively set up the next value so it's read to go
				getNextRasterValue();
			}
		}
	}
	else
	{
		SetLaserPowerOnly(0);
	}
}

////////////////////////////////////////////////////////////////////////////////

TIM_TypeDef _fakeTimer;

void TIM1_UP_TIM10_IRQHandler()
{
#ifdef GB_TIM1_ISR_PIN
		GB_TIM1_ISR_SET;
#endif //GB_TIM1_ISR_PIN

	if ((TIM1->SR & TIM_FLAG_Update) && TIM1->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM1, TIM_FLAG_Update) != RESET)
	{   // master timer for motion....all motion steps are generated by individual timers for each axis by
		// setting the timers to trigger in a oneshot fashion to create the next pulse

#ifdef SLICE_TIMING_MEASUREMENT
		_motionTimerCalls++;
#endif

		if (_Tim7StillCalculating == FALSE)
		{   // calculations are valid for reload
			loadDominantTimer(); // set up next time interval  (timer values were pre-calculated for accel)
		}

		MotorStructure *M;
		float DominantAxisPulsesMoved = (float)(DominantAxisPtr->PULSES_IN_MOVE - DominantAxisPtr->PULSES_TO_GO);

		if (motionQ.oldest->flags.needToUnprimeDuringMove)
		{
			if (DominantAxisPtr->PULSES_TO_GO == DominantAxisPtr->unprimePulseToGo)
			{
				UnPrime(currentOutboxPtr);
				motionQ.oldest->flags.unprimeIssued = TRUE;
				if (DominantAxisPtr->residualUnprimeTime == 0) // no extra time at the end of the move
					motionQ_unprimeComplete();  // no need to set the UnPrimeTimer ... already finished
			}
		}

		float AxisNextPulse;
		float TimeUntilPulse;
		int  CycleCount;
		TIM_TypeDef *TimerBase;

		int i;
		for (i=0; i<_numMovingAxes; i++)
		{   // only axes that were part of the move will be in the list
			M = _movingAxesPtrs[i];
			if (M->PULSES_TO_GO)
			{   // only waste time on motors that are involved in move
				if (M == DominantAxisPtr)
				{   // handle the dominant axis here. setup dom axis timer for a oneshot as simple as possible
					if (M->PulsePending == FALSE)   // SHOULD ALWAYS BE PENDING!
					{   // arm immediate timer to fire to call ProcessMotion in separate ISR
						// PSC/ARR was preset before move started
						M->PulsePending = TRUE;
						M->TimerBase->EGR |= TIM_EGR_UG; // force the CNT and PSC to reset
						M->TimerBase->CR1 |= TIM_CR1_CEN;  //TIM_Cmd(TimerBase, ENABLE);
					}
				}

				else
				{
					// handle regular axis here. setup axis timer for a oneshot based on when it should fire after dom axis
					if (M->PulsePending == FALSE)
					{   // work to do and not an outstanding pulse, so kick off the next pulse
						AxisNextPulse = (float)(M->PULSES_IN_MOVE - M->PULSES_TO_GO) * M->Dominant2AxisPulseRatio + M->AxisFirstPulseOffset;

						if ((AxisNextPulse - DominantAxisPulsesMoved) < 1.0f)
						{   // next axis pulse will occur before the next dominant pulse
							TimeUntilPulse = fmaxf(0.0f, ((AxisNextPulse - DominantAxisPulsesMoved) * DominantAxisPtr->MotionRateInSPP));
							CycleCount = imax(2, (uint32_t)(M->TimerFreq * TimeUntilPulse));
							TimerBase = M->TimerBase;
							TimerBase->PSC = CycleCount >> 16;      // upper 16-bits; value of 0 means divide by 1)
							TimerBase->ARR = (CycleCount / (TimerBase->PSC + 1)) - 1;  // PSC+1 because TimerPSC value of 0 means divide by 1;
							TimerBase->EGR |= TIM_EGR_UG; // force the CNT and PSC to reset
							TimerBase->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(M->TimerBase,TIM_FLAG_Update);    // clear the interrupt from the update
							M->PulsePending = TRUE;
							TimerBase->CR1 |= TIM_CR1_CEN;  //TIM_Cmd(TimerBase, ENABLE);
						}
					}
#ifdef BALANCED_TIME
					else
					{   // mimic real calcs and timer loads to use the same amount of time no matter if pulsePending or not
						AxisNextPulse = (float)(M->PULSES_IN_MOVE - M->PULSES_TO_GO) * M->Dominant2AxisPulseRatio + M->AxisFirstPulseOffset;
						if (TRUE)
						{   // force the following calse
							TimeUntilPulse = fmaxf(0.0f, ((AxisNextPulse - DominantAxisPulsesMoved) * DominantAxisPtr->MotionRateInSPP));
							CycleCount = imax(2, (uint32_t)(M->TimerFreq * TimeUntilPulse));
							TimerBase = &_fakeTimer;
							TimerBase->PSC = CycleCount >> 16;      // upper 16-bits; value of 0 means divide by 1)
							TimerBase->ARR = (CycleCount / (TimerBase->PSC + 1)) - 1;  // PSC+1 because TimerPSC value of 0 means divide by 1;
							TimerBase->EGR |= TIM_EGR_UG; // force the CNT and PSC to reset
							TimerBase->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(M->TimerBase,TIM_FLAG_Update);    // clear the interrupt from the update
							TimerBase->CR1 |= TIM_CR1_CEN;  //TIM_Cmd(TimerBase, ENABLE);
						}
					}
#endif //BALANCED_TIME
				}
			}
		}

		if (_Tim7StillCalculating == FALSE)
		{   // arm immediate timer to fire to call UpdateNextAccelerationFactor/CalculateTimerControls in separate ISR
			// PSC/ARR was preset before move started
			_Tim7StillCalculating = TRUE;
			TIM7->EGR |= TIM_EGR_UG; // force the CNT and PSC to reset
			TIM7->CR1 |= TIM_CR1_CEN;  //TIM_Cmd(TimerBase, ENABLE);
		}

#ifdef PROCESS_RASTER_WITH_DOMINANT_AXIS
		// if defined, this would move the processing for the the raster value change to this method
		// instead of the selected axis (currently only X).  this would allow any axis to become
		// the raster axis
		if (_gs._laser.rasterizeCurrentMove)
		{   // pulling data from the direct buffer to drive the laser in a raster mode
			if (DominantAxisPtr->PULSES_TO_GO)
			{
#ifdef GB_HIDDEN_WARNINGS
				int moveStepExtruderToTIM7call;	// move to a separate ISR call
#endif //GB_HIDDEN_WARNINGS
				processRasterStep();
			}
		}
#endif //PROCESS_RASTER_WITH_DOMINANT_AXIS

		 TIM1->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	}

	else if ((TIM10->SR & TIM_FLAG_Update) && TIM10->DIER & TIM_FLAG_Update)  //else if (TIM_GetITStatus(TIM10, TIM_FLAG_Update) != RESET)
	{   // Motor? control
#ifdef USE_HYREL_IO
		ProcessMotion(&Motors[M_B]);
#elif defined (USE_HYDRA_IO)
		ProcessMotion(&Motors[M_A]);
#endif
		 TIM10->SR = (uint16_t)~TIM_FLAG_Update;  //TIM_ClearITPendingBit(TIM10, TIM_FLAG_Update);  // clear interrupt flag
	}
#ifdef GB_TIM1_ISR_PIN
		GB_TIM1_ISR_CLEAR;
#endif //GB_TIM1_ISR_PIN
}

////////////////////////////////////////////////////////////////////////////////

void TIM1_BRK_TIM9_IRQHandler(void)
{    // MotorX control
	if ((TIM9->SR & TIM_FLAG_Update) && TIM9->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM9, TIM_FLAG_Update) != RESET)
	{
#ifdef USE_HYREL_IO
		ProcessMotion(&Motors[M_A]);
#elif defined (USE_HYDRA_IO)
		ProcessMotion(&Motors[M_B]);
#endif
		TIM9->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM9, TIM_FLAG_Update);  // clear interrupt flag
	}
}

////////////////////////////////////////////////////////////////////////////////

void TIM1_TRG_COM_TIM11_IRQHandler()
{   // MotorZ control
	if ((TIM11->SR & TIM_FLAG_Update) && TIM11->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM11, TIM_FLAG_Update) != RESET)
	{
		ProcessMotion(&Motors[M_C]);
		TIM11->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM11, TIM_FLAG_Update);  // clear interrupt flag
	}
}

////////////////////////////////////////////////////////////////////////////////


void TIM2_IRQHandler()
{
	TIM2->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);
}


////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////

void TIM3_IRQHandler()
{
	if (TIM_GetITStatus(TIM3, TIM_FLAG_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
	}
}

////////////////////////////////////////////////////////////////////////////////

void TIM4_IRQHandler()
{
	if (TIM_GetITStatus(TIM4, TIM_FLAG_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
	}
}

////////////////////////////////////////////////////////////////////////////////

void TIM5_IRQHandler()
{    // next section is used to calibrate the internal low speed oscillator for use a the IWDG clock
	if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
	{
		lsiCC4samples[lsiCC4sampleIndex++] = TIM_GetCapture4(TIM5); // save the capture compare value
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);  // clear interrupt pending flag
	}
}

////////////////////////////////////////////////////////////////////////////////

void TIM6_DAC_IRQHandler()
{   // Tim6 - Z jogging during move to adjust calibration
	if ((TIM6->SR & TIM_FLAG_Update) && TIM6->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM6, TIM_FLAG_Update) != RESET)
	{   //Jogging control on TIM6;
		if (_jogZ.M->PULSES_TO_GO)
		{   // jog axis moving from a "real" move.. bad news.  kill jog
			_jogZ.pulsesToGo = 0;
		}

		if (_jogZ.pulsesToGo == 0)
		{
			_jogZ.TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN; //TIM_Cmd(_jogZ.TimerBase, DISABLE);
		}
		else
		{
			assertControlBit(&_jogZ.M->Step); // start forming the step pulse
			if (_jogZ.pulsesToGo < 0)
				_jogZ.pulsesToGo++;
			else if (_jogZ.pulsesToGo > 0)
				_jogZ.pulsesToGo--;
			if (_jogZ.pulsesToGo == 0)
				_jogZ.TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN; //TIM_Cmd(_jogZ.TimerBase, DISABLE);
			wait100ns();
			deassertControlBit(&_jogZ.M->Step); // start forming the step pulse

			TIM6->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM6, TIM_FLAG_Update);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

#define BOOT_COUNT_INCR_MS          10  // 10 ms per call
#define BOOT_COUNT_INITIAL_WAIT_MS  1000
#define BOOT_COUNT_CHECK_PIN        1900
#define BOOT_COUNT_DONE_MS          2000

void TIM7_IRQHandler()
{
#ifdef GB_FUNC_TIM7_ISR_PIN
	GB_FUNC_TIM7_ISR_SET;
#endif //GB_FUNC_TIM7_ISR_PIN

	if ((TIM7->SR & TIM_FLAG_Update) && TIM7->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM7, TIM_FLAG_Update) != RESET)
	{
		if (!_useTimer7ToSenseStartPin)
		{
			if (_Tim7StillCalculating)
			{
				if (UpdateNextAccelerationFactor(DominantAxisPtr))
				{   // if the acceleration changed - update time interval for next pass
					CalculateTimerControls(DominantAxisPtr); // get ready for next iteration
				}
				_Tim7StillCalculating = FALSE;
			}
			else
			{
				if (_jogging.reloadTimerControls)
				{   // in jog mode AND a new move came in while current move was underway, need to recalc accel/decel
					_jogging.reloadTimerControls = FALSE;
					joggingCalculateTimerControls();
				}
			}

			TIM7->CR1 &= (uint16_t)~TIM_CR1_CEN;    //TIM_Cmd(TIM7, DISABLE);
		}

#ifdef ALLOW_NATIVE_LIGHTBURN
		else if (_useTimer7ToSenseStartPin)
		{
			_bootCounterMs += BOOT_COUNT_INCR_MS;
			if (_bootCounterMs < BOOT_COUNT_INITIAL_WAIT_MS)
			{   // just killing time
				;
			}
			else if (_bootCounterMs < BOOT_COUNT_CHECK_PIN)
			{   // check the sensor state
				if (readDebouncedSensorState(&startButton) == SENSOR_OPEN)
				{
					_lightburnBootModeSensorState = SENSOR_OPEN;    // flag if ever not pressed
				}
			}
			else if ((_bootCounterMs < BOOT_COUNT_DONE_MS) && (_lightburnBootModeSensorState == SENSOR_CLOSED))
			{   // button was pressed the entire time
				if (_lightburnModeEnabled == FALSE)
				{   // do any init for lightburn
					_lightburnModeEnabled = TRUE;
#ifdef GB_LIGHTBURN_PIN
					pinSet(GB_LIGHTBURN_PIN);
#endif //GB_LIGHTBURN_PIN
					masterCommPort = UART6_MASTER;
					_MailBoxes._hostTrafficReportingPeriodMs = 0;
					_MailBoxes._positionReportingPeriodMs = 0;
				}
			}
			else if (_bootCounterMs > BOOT_COUNT_DONE_MS)
			{
				TIM_Cmd(TIM7, DISABLE);
				_useTimer7ToSenseStartPin = FALSE;
			}
		}
#endif //ALLOW_NATIVE_LIGHTBURN
		TIM7->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM7, TIM_FLAG_Update);
	}
#ifdef GB_FUNC_TIM7_ISR_PIN
	GB_FUNC_TIM7_ISR_CLEAR;
#endif //GB_FUNC_TIM7_ISR_PIN
}

////////////////////////////////////////////////////////////////////////////////

void TIM8_BRK_TIM12_IRQHandler()
{   // MotorX control
	if ((TIM12->SR & TIM_FLAG_Update) && TIM12->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM12, TIM_FLAG_Update) != RESET)
	{
		ProcessMotion(&Motors[M_X]);
		TIM12->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM12, TIM_FLAG_Update);  // clear interrupt flag
	}
}
////////////////////////////////////////////////////////////////////////////////

void TIM8_UP_TIM13_IRQHandler()
{   // MotorY control
	if ((TIM13->SR & TIM_FLAG_Update) && TIM13->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM13, TIM_FLAG_Update) != RESET)
	{
		ProcessMotion(&Motors[M_Y]);
		TIM13->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM13, TIM_FLAG_Update);  // clear interrupt flag
	}
	//should check for TIM8 relaod interrupt
}

////////////////////////////////////////////////////////////////////////////////

void TIM8_TRG_COM_TIM14_IRQHandler()
{   //MotorC control
	if ((TIM14->SR & TIM_FLAG_Update) && TIM14->DIER & TIM_FLAG_Update)  //if (TIM_GetITStatus(TIM14, TIM_FLAG_Update) != RESET)
	{
		ProcessMotion(&Motors[M_Z]);
		TIM14->SR = (uint16_t)~TIM_FLAG_Update; //TIM_ClearITPendingBit(TIM14, TIM_FLAG_Update);  // clear interrupt flag
	}
}

////////////////////////////////////////////////////////////////////////////////

void USARTx_IRQHandler(USART_TypeDef *USARTx, masterPort_t UARTx_MASTER)
{
	// RXNEIE --> RX char avail OR char is avail and shift reg is full (second char avail)
	//         RXNE=1 or ORE=1 (overrun) in _SR reg
	// at a minimum, there is a character available....but before reading the DR (which will
	// clear some flags in the SR), check for any special cases:

#ifdef GB_UART_ISR_PIN
	pinSet(GB_UART_ISR_PIN);
#endif //GB_UART_ISR_PIN
	if (USARTx->SR & USART_FLAG_ORE)    //if (USART_GetFlagStatus(USARTx, USART_FLAG_ORE))
	{   // the above checked that the CR1 RXNEIE is set and that the SR->ORE was set (overrun)
		// Bit 3 ORE:
		// Overrun error This bit is set by hardware when the word currently being received in the shift register is ready to be
		// transferred into the RDR register while RXNE=1. An interrupt is generated if RXNEIE=1 in the USART_CR1 register. It is
		// cleared by a software sequence (an read to the USART_SR register followed by a read to the USART_DR register).
		//     0: No Overrun error
		//     1: Overrun error is detected Note: When this bit is set, the RDR register content will not be lost but the shift
		//        register will be overwritten. An interrupt is generated on ORE flag in case of Multi Buffer communication if the EIE bit is set.
#ifdef GB_UART_OVERRUN_PIN
		pinSet(GB_UART_OVERRUN_PIN);
#endif //GB_UART_OVERRUN_PIN

		char RcvUART = (USARTx->DR & (uint16_t)0x1FF);
		if ((RcvUART == PING_CHAR) || (RcvUART == ABORT_CHAR))
			changeMasterCommPort(UARTx_MASTER);
		if (masterCommPort == UARTx_MASTER)
			ReceiveCharacter(RcvUART);
		_serialPortRxOverrunCnt++;
		_serialPortRxOverrunFlag = TRUE;

		USARTx->SR = (uint16_t)~((uint16_t)0x01 << (uint16_t)(USART_IT_ORE_RX >> 0x08));    //USART_ClearITPendingBit(USARTx, USART_IT_ORE_RX);

#ifdef GB_UART_OVERRUN_PIN
		pinClear(GB_UART_OVERRUN_PIN);
#endif //GB_UART_OVERRUN_PIN
	}

	if (USARTx->SR & USART_FLAG_RXNE)   //if (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE))
	{
		char RcvUART = (USARTx->DR & (uint16_t)0x1FF);      // reading the DR also clears the RXNE bit in the SR
		if ((RcvUART == PING_CHAR) || (RcvUART == ABORT_CHAR))
			changeMasterCommPort(UARTx_MASTER);
		if (masterCommPort == UARTx_MASTER)
			ReceiveCharacter(RcvUART);
	}

#ifdef GB_UART_ISR_PIN
	pinClear(GB_UART_ISR_PIN);
#endif //GB_UART_ISR_PIN
}
////////////////////////////////////////////////////////////////////////////////

//void USART3_IRQHandler(void)
//{
//	USARTx_IRQHandler(USART3, UART3_MASTER);
//}

////////////////////////////////////////////////////////////////////////////////

void UART4_IRQHandler(void)
{
	USARTx_IRQHandler(UART4, UART4_MASTER);
}

////////////////////////////////////////////////////////////////////////////////

void USART6_IRQHandler(void)
{
	USARTx_IRQHandler(USART6, UART6_MASTER);
}

////////////////////////////////////////////////////////////////////////////////

void DisableEXTI(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* EXTI configuration *******************************************************/
	EXTI_ClearITPendingBit(_TouchProbeEXTI_Line); // the pinMask aligns to the EXTI_LineXX for a given bit position
	EXTI_InitStructure.EXTI_Line = _TouchProbeEXTI_Line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);

	interruptSetupAndDisable(_TouchProbeNVIC_IRQChannel, NVIC_PREMPTION_PRIORITY_LOWEST);
}

////////////////////////////////////////////////////////////////////////////////

boolean SetupEXTI(int probeSelect, probeType_t probeType, EXTITrigger_TypeDef edgeType, int initState)
{   // setup routine to enable external interrupts for the TouchProbe
	boolean enableISR = FALSE;
	boolean enableCanbusProbe = FALSE;

	if (_TouchProbeCanDevice == CAN_SYSTEM_DEVICE_ID)
	{
		pinType pin=PIN_UNDEFINED;
		switch (probeSelect)
		// GCODE         0 : No direct connect probe; canbus based probe one
		// GCODE         1 : Limit1 pin on X axis connector
		// GCODE         2 : Limit2 pin on X axis connector
		// GCODE         3 : Fault  pin on X axis connector
		// GCODE         4 : Home   pin on X axis connector
		// GCODE         5 : unused
		// GCODE         6,7,8,9 : Limit1,Limit2,Fault,Home pin on Y axis connector
		// GCODE         10 : unused
		// GCODE         11,12,13,14 : Limit1,Limit2,Fault,Home pin on Z axis connector
		// GCODE         15 : unused
		// GCODE         16,17,18,19 : Limit1,Limit2,Fault,Home pin on A axis connector
		// GCODE         20 : unused
		// GCODE         21,22,23,24 : Limit1,Limit2,Fault,Home pin on B axis connector
		// GCODE         25 : unused
		// GCODE         26,27,28,29 : Limit1,Limit2,Fault,Home pin on C axis connector
		// GCODE
		// GCODE    get position of motion system when probe is contacted.  error reported if
		// GCODE    move completes without the probe being contact
		{
#ifdef USE_HYREL_IO
		case 1:  pin = W_RTD1; enableISR = TRUE; break;
		case 2:  pin = W_RTD2; enableISR = TRUE; break;
#elif defined (USE_HYDRA_IO)
//		case 1:  pin = X_L1;    enableISR = TRUE; break;
//		case 2:  pin = X_L2;    enableISR = TRUE; break;
//		case 3:  pin = X_FAULT; enableISR = TRUE; break;
		case 4:  pin = X_HOME;  enableISR = TRUE; break;
		case 5:  break;  // unused
//		case 6:  pin = Y_L1;    enableISR = TRUE; break;
//		case 7:  pin = Y_L2;    enableISR = TRUE; break;
//		case 8:  pin = Y_FAULT; enableISR = TRUE; break;
		case 9:  pin = Y_HOME;  enableISR = TRUE; break;
		case 10: break;  // unused
//		case 11: pin = Z_L1;    enableISR = TRUE; break;
//		case 12: pin = Z_L2;    enableISR = TRUE; break;
//		case 13: pin = Z_FAULT; enableISR = TRUE; break;
		case 14: pin = Z_HOME;  enableISR = TRUE; break;
		case 15: break;  // unused
//		case 16: pin = A_L1;    enableISR = TRUE; break;
//		case 17: pin = A_L2;    enableISR = TRUE; break;
//		case 18: pin = A_FAULT; enableISR = TRUE; break;
		case 19: pin = A_HOME;  enableISR = TRUE; break;
		case 20: break;  // unused
//		case 21: pin = B_L1;    enableISR = TRUE; break;
//		case 22: pin = B_L2;    enableISR = TRUE; break;
//		case 23: pin = B_FAULT; enableISR = TRUE; break;
		case 24: pin = B_HOME;  enableISR = TRUE; break;
		case 25: break;  // unused
//		case 26: pin = C_L1;    enableISR = TRUE; break;
//		case 27: pin = C_L2;    enableISR = TRUE; break;
//		case 28: pin = C_FAULT; enableISR = TRUE; break;
		case 29: pin = C_HOME;  enableISR = TRUE; break;
#endif
		default: break;
		}
		_TouchProbePin = pin;

		if (enableISR)
		{
			int pinState = pinRead(pin);
			if (((edgeType == EXTI_Trigger_Falling) && (pinState == 0) && (initState == 1)) ||
				((edgeType == EXTI_Trigger_Rising) && (pinState == 1) && (initState == 0)))
			{   // already triggered so abort
				TouchProbeFinished(PROBE_RESULTS_PROBE_CONTACT_BEFORE_MOVE_STARTED, PROBE_TYPE_CONTACT, PROBE_ERROR);
				enableISR = FALSE;
			}
			else
			{
				int portNum = pinExtractPortNum(pin);
				int pinMask = pinExtractPinMask(pin);
				int pinNum = pinExtractPinNum(pin);
				uint8_t NVIC_IRQChannel;

				switch (pinNum)
				{
				case 0: NVIC_IRQChannel = EXTI0_IRQn; break;
				case 1: NVIC_IRQChannel = EXTI1_IRQn; break;
				case 2: NVIC_IRQChannel = EXTI2_IRQn; break;
				case 3: NVIC_IRQChannel = EXTI3_IRQn; break;
				case 4: NVIC_IRQChannel = EXTI4_IRQn; break;
				case 5:
				case 6:
				case 7:
				case 8:
				case 9: NVIC_IRQChannel = EXTI9_5_IRQn; break;
				case 10:
				case 11:
				case 12:
				case 13:
				case 14:
				case 15: NVIC_IRQChannel = EXTI15_10_IRQn; break;
				default : enableISR = FALSE; break;
				}

				if (enableISR)
				{   // all is well, so go ahead and arm the interrupt.
					EXTI_InitTypeDef EXTI_InitStructure;

					SYSCFG_EXTILineConfig(portNum, pinNum); // set up the EXT to pick the right port+pin

					/* EXTI configuration *******************************************************/
					_TouchProbeEXTI_Line = pinMask; // need a copy to verify inside ISR

					EXTI_ClearITPendingBit(_TouchProbeEXTI_Line); // the pinMask aligns to the EXTI_LineXX for a given bit position
					EXTI_InitStructure.EXTI_Line = _TouchProbeEXTI_Line;
					EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
					EXTI_InitStructure.EXTI_Trigger = edgeType;
					EXTI_InitStructure.EXTI_LineCmd = ENABLE;
					EXTI_Init(&EXTI_InitStructure);

					_TouchProbeNVIC_IRQChannel = NVIC_IRQChannel;

					interruptSetupAndEnable(_TouchProbeNVIC_IRQChannel, NVIC_PREMPTION_PRIORITY_TOUCH_PROBE);
				}
			}
		}
	}
	else //_TouchProbeCanDevice != CAN_SYSTEM_DEVICE_ID
	{   // using canbus probe
		HH_IoIndex_t pin;
		enableCanbusProbe = FALSE;
		switch (probeSelect)
		// GCODE         0 : PA3 pin on 10-pin connector
		// GCODE         1 : PA2 pin on 10-pin connector
		// GCODE         2 : LIMIT1 pin on 18-pin connector
		// GCODE         3 : LIMIT2 pin on 18-pin connector
		// GCODE         4 : RTD1 pin on 18-pin connector
		// GCODE         5 : RTD2 pin on 18-pin connector
		{
		case 0:  pin = HH_PA3;      enableCanbusProbe = TRUE; break;
		case 1:  pin = HH_PA2;      enableCanbusProbe = TRUE; break;
		case 2:  pin = HH_LIM1;     enableCanbusProbe = TRUE; break;
		case 3:  pin = HH_LIM2;     enableCanbusProbe = TRUE; break;
		case 4:  pin = HH_RTD1;     enableCanbusProbe = TRUE; break;
		case 5:  pin = HH_RTD2;     enableCanbusProbe = TRUE; break;
		default: break;
		}
		if (enableCanbusProbe)
		{
			// run through all installed heads and make sure at least one is a touch probe
			// this is just a quick hack to arm the probe... need to add a new CAN packet
			sendProbeControlToDevice(_TouchProbeCanDevice, probeType, PROBE_ACTION_ARM, 0, 1, initState, pin);
		}
	}

	if (enableISR || enableCanbusProbe)
	{
		MotorStructure *M;
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors,
			M->TouchPosition = INVALID_ARG_VALUE;
		}
	}
	return(enableISR || enableCanbusProbe);
}

////////////////////////////////////////////////////////////////////////////////

void setTouchPositionToCurrentPosition(void)
{
	uint32_t irq_disabled = interruptsOff();
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors
		M->TouchPosition = M->POSITION;
	}
	interruptsOn(irq_disabled);
}

////////////////////////////////////////////////////////////////////////////////

void TouchProbeSendResults(probeResults_t probeResults, probeType_t probeType, int probeValue)
{
	char code;
	char resultStr[32] = "";
	char valueStr[32] = " :0";
	float usec;
	MotorStructure *M;
	boolean moveFinished;

	switch (probeResults)
	{
	case PROBE_RESULTS_MOTION_FINISHED :
		// two cases -- either at end of move OR stopped mid move
		moveFinished = TRUE;
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{
			if (M->POSITION != M->Q_POSITION)
			{   // never got to final desired destination
				moveFinished = FALSE;
				break;
			}
		}
		if (moveFinished)
		{   // failed to trigger
			code = 'F';
			strcpy(resultStr, " :FAIL");
			setTouchPositionToCurrentPosition();
			sendError("Contact probe - motion complete before contact");
		}
		else
		{
			code = 'O';
			strcpy(resultStr, " :PASS");
		}
		break;
	case PROBE_RESULTS_PROBE_CONTACT_BEFORE_MOVE_STARTED : // error
		code = 'E';
		strcpy(resultStr, " :STARTING_STATE");
		setTouchPositionToCurrentPosition();
		sendError("Contact probe in wrong state to start move");
		break;
	case PROBE_RESULTS_CANBUS_PROBE_ARMING_TIMEOUT : // error
		code = 'A';
		strcpy(resultStr, " :ARM_ERROR");
		setTouchPositionToCurrentPosition();
		sendError("CANBus probe failed to arm");
		break;
	case PROBE_RESULTS_CANBUS_PROBE_REPORT :
		strcpy(resultStr, " :REPORT");
		code = 'R';
		switch (probeType)
		{
		case PROBE_TYPE_NONE    :
		case PROBE_TYPE_CONTACT :
		case PROBE_TYPE_BLTOUCH :
		case PROBE_TYPE_UNKNOWN :
			break;
		case PROBE_TYPE_HC_SR04 :
			if (probeValue == PROBE_ERROR)
			{   // ERROR
				code = 'F';
				sprintf(valueStr, ":U%4.3f", -1.0);
			}
			else
			{
				usec = (float)probeValue / 36.0f / 2.0f; // divide by 8 because 36Mhz clk (each tick is 27.777ns) and div 2 because time was for round trip
				//sprintf(valueStr, " :%d/%3.2f/%3.2fmm", probeValue, usec, usec*0.343f);
				sprintf(valueStr, ":U%4.3f", usec * 0.343f);
			}
#ifdef GB_DEBUGGING_HCSR04
			sprintf(_tmpStr, " (%d)", probeValue);
			strcat(valueStr, _tmpStr);
#endif
		case PROBE_TYPE_IGAGING :
			// GB XXX move #def from HH to to hyrel.h and replace 21
			if (probeValue == PROBE_ERROR)
			{   // ERROR
				code = 'F';
				sprintf(valueStr, ":U%4.3f(ERR)", -1.0);
			}
			else
			{
				probeValue = (probeValue << (32 - IGAGING_NUM_BITS)) >> (32 - IGAGING_NUM_BITS); //sign extend
				// 127/128 for vernier; 0.005mm (1/200) per tick on scale
				sprintf(valueStr, ":U%3.2f:Q%d", (float)probeValue * (127.0f / 128.0f) * 0.005f, probeValue);
			}
#ifdef GB_DEBUGGING_HCSR04
			sprintf(_tmpStr, " (%d)", probeValue);
			strcat(valueStr, _tmpStr);
#endif
			break;
		}
		break;
	default :
		code = '*';
		break;
	}


	sprintf(_rptStr, ">T%c", code);

	sprintf(_tmpStr, ":T%d", _TouchProbeCanDevice);
	strcat(_rptStr, _tmpStr);

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if the motor is installed, the report
		if (M->MotorInstalled)
		{
			sprintf(_tmpStr, ":%c%4.3f", M->AxisLabel, M->TouchPosition * M->UnitsPerPulse);
			strcat(_rptStr, _tmpStr);
		}
	}
	if (code == 'R')
	{   // report
		strcat(_rptStr, valueStr);
	}
	strcat(_rptStr, resultStr);
	sendstringCr(_rptStr);

	if (ARG_H_PRESENT)
	{   // updating tool offset (like an M660)
		if (code == 'O')
		{   // passed
			M = &Motors[M_Z];
			ARG_Z = M->TouchPosition * M->UnitsPerPulse;
			ARG_Z += ARG_O;
			ARG_D = INVALID_ARG_VALUE; // was used by G38, so clear it for M660
			M_Code_M660();
			sprintf(_tmpStr,"Setting tool length for tool index %d to %4.3f (%4.3f + %4.3f)", (int)ARG_H, ARG_Z, ARG_Z-ARG_O, ARG_O);
			sendInfo(_tmpStr);
		}
		else
		{   // any type of fail
			setPauseGcodeFlag(CATASTROPHIC_ERROR_ALERT_CHAR);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void TouchProbeFinished(probeResults_t probeResults, probeType_t probeType, int probeValue)
{
	DisableEXTI();


	TouchProbeSendResults(probeResults, probeType, probeValue);

	// since the move was likely interrupted by the probe contact, we need to adjust the motionQ view of the position to match
	// the current position, since they will be out of sync due to the move stopping mid-move.

	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		M->Q_POSITION =  M->POSITION;
		M->Q_LastRequestedPositionInUnits = (M->POSITION * M->UnitsPerPulse) - SumOfAllMotorOffsets(M);
	}

	if (_TouchProbeCanDevice != 0)
	{   //
		canPackIntoTxQueue2x32(CAN_WRITE, _TouchProbeCanDevice, CAN_MSG_TOUCH_PROBE, PROBE_TYPE_NONE, BUFFERED_MSG, PROBE_ACTION_STOP, 0);
	}

	_TouchProbeMoveActive = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void TouchProbeProcessContact(probeType_t probeType, int probeValue)
{
	// the touch probe as made contact --- have arrived here either from the EXTI ISR
	// or a received canbus message

	setTouchPositionToCurrentPosition();

	motionQ_abortMotionWhenSafe(FLUSH_THEN_CONTINUE);
}

////////////////////////////////////////////////////////////////////////////////

void EdgeTriggerDetected(void)
{
	_edgeTriggerSignalState = pinRead(_TouchProbePin);
	_edgeTriggerDetected = TRUE;
}

////////////////////////////////////////////////////////////////////////////////

void EdgeTriggerSendResults(void)
{
	if (_edgeTriggerArmed && _edgeTriggerDetected)
	{
		_edgeTriggerDisplayRateCnt++;
		if (_edgeTriggerDisplayRateCnt >= (int)((1.0f /_edgeTriggerDisplayRateHz) * 10.0f))  // invert rate to get times per sec; *10 because working from 10Hz LOOP
		{   // waited long enough to allow sending message to host
			_edgeTriggerDisplayRateCnt = 0;
			sprintf(_rptStr, ">ED:%c", (_edgeTriggerSignalState == 0) ? 'F' : 'R');
			sendstringCr(_rptStr);
			_edgeTriggerDetected = FALSE;
		}
	}
	else
	{
		_edgeTriggerDisplayRateCnt = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

void Generic_EXTI_IRQHandler(void)
{
	if (EXTI_GetITStatus(_TouchProbeEXTI_Line) != RESET)
	{
		if (_TouchProbeMoveActive)
		{   // using touch probe
			TouchProbeProcessContact(PROBE_TYPE_CONTACT, 0);
			EXTI_ClearITPendingBit(_TouchProbeEXTI_Line);   // clear this interrupt
			EXTI_DeInit();
		}
		else if (_edgeTriggerArmed)
		{   // using edge trigger
			EdgeTriggerDetected();
		}
	}
	EXTI_ClearITPendingBit(_TouchProbeEXTI_Line);
}

void EXTI0_IRQHandler(void) { Generic_EXTI_IRQHandler(); }
void EXTI1_IRQHandler(void) { Generic_EXTI_IRQHandler(); }
void EXTI2_IRQHandler(void) { Generic_EXTI_IRQHandler(); }
void EXTI3_IRQHandler(void) { Generic_EXTI_IRQHandler(); }
void EXTI4_IRQHandler(void) { Generic_EXTI_IRQHandler(); }
void EXTI9_5_IRQHandler(void)  { Generic_EXTI_IRQHandler(); }
void EXTI15_10_IRQHandler(void)  { Generic_EXTI_IRQHandler(); }

////////////////////////////////////////////////////////////////////////////////

void processEverySlice(void)
{
	checkAndClearFlashStatus();

#ifdef SLICE_TIMING_MEASUREMENT
	_motionTimerCalls = 0;
	int totalTime = -(int)TIM2->CNT;
	if ((_sliceTiming.matchSlice == ++_gs._sliceCnt) || (_sliceTiming.matchSlice == -1))
		pinSet(_sliceTiming.pin);
#else
	_gs._sliceCnt++;
	tickCount++;
#endif
#ifdef GB_CMDQUE_EMPTY_PIN
	pinWrite(GB_CMDQUE_EMPTY_PIN, (CommandsInQue == 0) ? 1 : 0);
#endif
#ifdef GB_MOTIONQ_EMPTY_PIN
	pinWrite(GB_MOTIONQ_EMPTY_PIN, motionQ_empty() ? 1 : 0);
#endif
#ifdef GB_DEFFERED_CMDS_PIN
	pinWrite(GB_DEFFERED_CMDS_PIN, DeferredCommandsInQue ? 1 : 0);
#endif

	if (G2G3Flag||G203State)ExecuteG2G3PointMove();//we are in a print arc points at this time.
	if ((_arc.index > 0) || (_arc.state != ARC_STATE_IDLE)) ExecuteArcPointMove();//we are in a print arc points at this time.

	PrintCheck();
	transmitEchoChar();

	if (_gs._preCannedStepPending)
	{
#ifdef GB_HIDDEN_WARNINGS
		int moveStepExtruderToTIM7call; // otherwise avg 62usec delay to get pulse out
#endif //GB_HIDDEN_WARNINGS
		canAddToTxQueue(&_gs._preCannedPackets[PRECANNED_STEP]);
		protectedDecrement(&_gs._preCannedStepPending);
#ifdef COLLECT_METRICS
	_metrics.canbus_e_steps++;
#endif
	}

	canAddToImmediateRxQueue();     // check if any incomingRX immediate messages
	canAddToRxQueue();              // check if any incomingRX messages and add to rx queue

	ProcessRawRxBuffer();   // grab and pending rx chars and move into proper buffer

#ifdef ADD_ON_SPI_DISPLAY
	LCD_UpdateBacklightPwm();
#endif //ADD_ON_SPI_DISPLAY

#ifdef SLICE_TIMING_MEASUREMENT
	int sliceIndex;
	if (_gs._ctrlIndex[HZ_1000])
		sliceIndex = _gs._ctrlIndex[HZ_1000];
	else if (_gs._ctrlIndex[HZ_100])
		sliceIndex = NUM_1000HZ + _gs._ctrlIndex[HZ_100];
	else if (_gs._ctrlIndex[HZ_10])
		sliceIndex = NUM_1000HZ + NUM_100HZ + _gs._ctrlIndex[HZ_10];
	else // if (_gs._ctrlIndex[HZ_1]) any HZ_1 entry can run
		sliceIndex = NUM_1000HZ + NUM_100HZ + NUM_10HZ + _gs._ctrlIndex[HZ_1];

	if (sliceIndex >= NUM_SLICE_TIME_ENTRIES)
	{
		sliceIndex = 0;  // something is wrong if we get here, but avoid a crash by setting to 0
	}

	if (_sliceTiming.matchSlice == _gs._sliceCnt)
	{
		pinClear(_sliceTiming.pin);
	}

	totalTime += TIM2->CNT;
	if (totalTime < _sliceTiming.slices[sliceIndex].minTime)
		_sliceTiming.slices[sliceIndex].minTime = totalTime;
	if (totalTime > _sliceTiming.slices[sliceIndex].maxTime)
		_sliceTiming.slices[sliceIndex].maxTime = totalTime;
	if (_motionTimerCalls > _sliceTiming.slices[sliceIndex].motionTimerCalls)
		_sliceTiming.slices[sliceIndex].motionTimerCalls = _motionTimerCalls;

	if ((_sliceTiming.matchSlice == _gs._sliceCnt) || (_sliceTiming.matchSlice == -1))
		pinClear(_sliceTiming.pin);
#endif
}

#ifdef CCMRAM_USED_FOR_CODE
void SysTick_Handler(void) __attribute__ ((section (".ccmram")));	// force in to special fast sram
//NUKE __attribute__ ((section (".ccmram")))
void SysTick_Handler(void)
#else
void SysTick_Handler(void)
#endif
//void SysTick_Handler(void)
{   //normally there is a instruction to reset the interrupt request bit here
	//but the SysTick_Handler seems to have no need for that
#ifdef GB_SYSTICK_PIN
	pinToggleOutput(GB_SYSTICK_PIN);
#endif
	if (_MailBoxes._waitingFor.flags.bit.sliceNeedsExtraTime) return;   // multi-slice routine, give it time

	processEverySlice();
	if (++_gs._ctrlIndex[HZ_1000] == NUM_1000HZ)
	{
		_gs._ctrlIndex[HZ_1000] = 0;
		if (++_gs._ctrlIndex[HZ_100] == NUM_100HZ)
		{
			_gs._ctrlIndex[HZ_100] = 0;
			if (++_gs._ctrlIndex[HZ_10] == NUM_10HZ)
			{
				_gs._ctrlIndex[HZ_10] = 0;
				if (++_gs._ctrlIndex[HZ_1] == NUM_1HZ)
				{
					_gs._ctrlIndex[HZ_1] = 0;
					if (!(_sendingGBStringsMask & GB_STRING_CANBUS)) // use M797 S<mask> to enable
					{
						_gs._sliceCnt = 0;	// reset count of total number of slices
					}
				}
				_gs._sliceNum = _gs._ctrlIndex[HZ_1] * 1000;
				F1HZ[_gs._ctrlIndex[HZ_1]]();
			}
			else
			{
				_gs._sliceNum = _gs._ctrlIndex[HZ_10] * 100;
				F10HZ[_gs._ctrlIndex[HZ_10]]();
			}
		}
		else
		{
			_gs._sliceNum = _gs._ctrlIndex[HZ_100] * 10;
			F100HZ[_gs._ctrlIndex[HZ_100]]();
		}
	}
	else
	{
		_gs._sliceNum = _gs._ctrlIndex[HZ_1000];
		F1000HZ[_gs._ctrlIndex[HZ_1000]]();
	}
}

////////////////////////////////////////////////////////////////////////////////

void Crash_Handler(char *source)
{
	__disable_irq(); // disable interrupts to try to preserve current state of the machine

#ifdef ENABLE_CRASH_LOGGING
	strcpy(crash_source, source);
#endif

	NVIC_SystemReset();
}

////////////////////////////////////////////////////////////////////////////////

void checkAndClearFlashStatus(void)
{
	if ((FLASH->SR & 0xc0) != 0)
	{	// if prior to this, there was a bad access to flash, it will be caught, reported, and cleared here ...
		// otherwise, future attempts to erase/write flash will fail
		if (_errors.sent.flags.flashStatusErrorDectected == FALSE)
		{
			_errors.sent.flags.flashStatusErrorDectected = TRUE;
			sprintf(_errorStr, "FLASH ACCESS ERROR (SR=0x%04x) (loop %u.%u.%u.%u)", (uint16_t)FLASH->SR, _gs._ctrlIndex[HZ_1],_gs._ctrlIndex[HZ_10], _gs._ctrlIndex[HZ_100], _gs._ctrlIndex[HZ_1000]);
			sendError(_errorStr);
		}
		FLASH->SR = 0x0c;	// clear error flags in order to proceed
	}
}

////////////////////////////////////////////////////////////////////////////////

void protectedAssigment(int *var, int value)
{

	uint32_t irq_disabled = interruptsOff();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsequence-point"
	*var = value;
#pragma GCC diagnostic pop
	interruptsOn(irq_disabled);
}

void protectedIncrement(int *var)
{
	uint32_t irq_disabled = interruptsOff();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsequence-point"
	//*var = (*var)++;
	int tmp = *var;
	tmp++;
	*var = tmp;
#pragma GCC diagnostic pop
	interruptsOn(irq_disabled);
}

void protectedDecrement(int *var)
{
	uint32_t irq_disabled = interruptsOff();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsequence-point"
	//*var = (*var)--;
	int tmp = *var;
	tmp--;
	*var = tmp;
#pragma GCC diagnostic pop
	interruptsOn(irq_disabled);
}

////////////////////////////////////////////////////////////////////////////////

void catastrophicError(char *s)
{
	sendErrorNoCr("CATASTROPHIC: ");
	sendstringCr(s);
	motionQ_abortMotionWhenSafe(FLUSH_THEN_ABORT);
}

////////////////////////////////////////////////////////////////////////////////

void catastrophicErrorWithDebugInfo(char *s)
{
	sendErrorNoCr("CATASTROPHIC: ");
	sendstringCr(s);
	sprintf(_errorStr, "G%d N%ld", (int)ARG_G,getCurrentExecutingLineNumber());
	if (ARG_X_PRESENT) { sprintf(_tmpStr, " X%4.3f", ARG_X); strcat(_errorStr, _tmpStr); }
	if (ARG_Y_PRESENT) { sprintf(_tmpStr, " Y%4.3f", ARG_X); strcat(_errorStr, _tmpStr); }
	if (ARG_Z_PRESENT) { sprintf(_tmpStr, " Z%4.3f", ARG_X); strcat(_errorStr, _tmpStr); }
	sendGB(_errorStr);
	if (CannedCycleFlag || G2G3Flag || G203State)
	{
		sprintf(_errorStr, " CannedCycleFlag=%d", CannedCycleFlag);
		sprintf(_tmpStr, " G2G3Flag=%d", G2G3Flag); strcat(_errorStr, _tmpStr);
		sprintf(_tmpStr, " G203State=%d", G203State); strcat(_errorStr, _tmpStr);
		sprintf(_tmpStr, " CannedCycleLastMove=%d", CannedCycleLastMove); strcat(_errorStr, _tmpStr);
		sendGB(_errorStr);
	}
	motionQ_abortMotionWhenSafe(FLUSH_THEN_ABORT);
}

////////////////////////////////////////////////////////////////////////////////

//GB XXX gregkarl --- need to protect against compiler optimization
void wait100ns(void)
{
	uint32_t i;
	for (i=0; i<25; i++);
}

////////////////////////////////////////////////////////////////////////////////
//  SAVE CODE FOR FUTURE REFERENCE
//void reportCalibrationTemperature(void)
//{   //this will read the thermo couple max6675 and store the result in _gs._calibrationTemp >RT
//#if defined(USE_HYREL_IO) && defined(USE_TEMPERATURE_CALIBRATION)
//
//#define RTD_CALIBRATION_CSB         (OUTPUT_PP_50MHZ  | PIN_PORT_F | PIN_NUM_13)
//#define RTD_CALIBRATION_CLOCK       (OUTPUT_PP_50MHZ  | PIN_PORT_F | PIN_NUM_14)
//#define RTD_CALIBRATION_DATA_BIT    (OUTPUT_PP_50MHZ  | PIN_PORT_F | PIN_NUM_11)
//    uint16_t temp;
//    int i;
//
//    if (_gs._reportCalibrationTemperature)
//    {   // can be enable via M_Code_M759
//        pinClear(RTD_CALIBRATION_CLOCK);
//        pinClear(RTD_CALIBRATION_CSB);
//        wait100ns();
//
//        temp = 0;
//        for (i=0; i<16; i++)
//        {
//            temp <<= 1;
//            temp |= (uint16_t)pinRead(RTD_CALIBRATION_DATA_BIT);
//            pinSet(RTD_CALIBRATION_CLOCK);
//            wait100ns();
//            pinClear(RTD_CALIBRATION_CLOCK);
//            wait100ns();
//        }
//        pinSet(RTD_CALIBRATION_CSB);
//        // temperature is found in the upper 13 bits and is in a 11.2 format.
//        // to convert to our temp format, blow away the 3 LSBs ...which leaves a 11.5
//        if (TEMP_FRAC_BITS > 5)
//        {
//            _gs._calibrationTemp = (temp & 0x0000fff8) << (TEMP_FRAC_BITS - 5);
//        }
//        else
//        {
//            _gs._calibrationTemp = (temp & 0x0000fff8) >> (5 - TEMP_FRAC_BITS);
//        }
//
//        if (_gs._reportCalibrationTemperatureRate > 0)  // if displaying info
//        {
//            if ((_gs._seconds % _gs._reportCalibrationTemperatureRate) == 0)    // time to report
//            {
//                if (_gs._reportCalibrationTemperatureDevice == 0)       // just report calibration device
//                {
//                    sprintf(_rptStr, ">CD: %d: %3.2f ", 0, (float)(temp>>3)/4);
//                }
//                else // report calibration temp and selected hot head info
//                {
//                    sprintf(_rptStr, ">CD: %d: cal=%2.1f  dev=%2.1f  adcAvg=%d  DC=%d%c",
//                            _gs._reportCalibrationTemperatureDevice,
//                            (float)_gs._calibrationTemp/(float)TEMP_SCALE,
//                            (float)getInboxPointer(_gs._reportCalibrationTemperatureDevice)->adcValues[RTD1].convAvg/(float)TEMP_SCALE,
//                            getInboxPointer(_gs._reportCalibrationTemperatureDevice)->adcValues[RTD1].adcAvg,
//                            getInboxPointer(_gs._reportCalibrationTemperatureDevice)->switchValues[HH_HTR_SWITCH].dutyCycle, '%');
//                }
//                sendstringCr(_rptStr);
//            }
//        }
//    }
//#endif //USE_HYREL_IO
//}

////////////////////////////////////////////////////////////////////////////////

void updateHssPwm(HssPwmStruct *hss)
{
	hss->TerminalCount = (int)(hss->PeriodInSec * hss->TicksPerSecond);
	hss->CompareValue =  (int)(hss->TerminalCount * (hss->DutyCycle / 100.0f));
	hss->Counter = 0;           // restart counter
}

////////////////////////////////////////////////////////////////////////////////

void changeHssDuty(HssPwmStruct *hss, float dutyCycle)
{   // used to change actual pwm control values based on the hssStructs DutyCycle
	hss->DutyCycle = fFitWithinRange(dutyCycle,  0.0f, 100.0f); // force to legal range
	updateHssPwm(hss);  // clear counter, adjust terminal count, etc
}

////////////////////////////////////////////////////////////////////////////////

void initHss(hssPin_t index, int bit)
{
	HssPwmStruct *hss = &HighSideSwitches[index];

	hss->index = index;
	hss->Output.Bit = bit;
	hss->Output.gpioPort = pinExtractPortPtr(hss->Output.Bit);
	hss->Output.gpioMask = pinExtractPinMask(hss->Output.Bit);
	hss->Output.Polarity = ACTIVE_HIGH;
	hss->DutyCycle = HSS_DUTY_CYCLE_OFF;
	hss->PeriodInSec = 1.0f;
	hss->oneShot = FALSE;
	hss->TicksPerSecond = TICKS_PER_SEC_100HZ_LOOP;

	updateHssPwm(hss);  // calc terminal count, clear counter, etc
}

////////////////////////////////////////////////////////////////////////////////

void initAllHss(void)
{
	int i=0;

	// set the default pwm controls for each available switch to OFF
	initHss(NO_HSS_PIN_INDEX, PIN_UNDEFINED);
	initHss(HSS_AUX_PWR1_INDEX, HSS_AUX_PWR1);
	initHss(HSS_AUX_PWR2_INDEX, HSS_AUX_PWR2);
	initHss(HSS_AUX_PWR4_INDEX, HSS_AUX_PWR4);
	initHss(HSS_AUX_PWR5_INDEX, HSS_AUX_PWR5);
	initHss(HSS_AUX_PWR6_INDEX, HSS_AUX_PWR6);
	initHss(HSS_AUX_PWR7_INDEX, HSS_AUX_PWR7);
	initHss(HSS_AUX_PWR8_INDEX, HSS_AUX_PWR8);
	initHss(HSS_AUX_PWR9_INDEX, HSS_AUX_PWR9);
//	initHss(DRAIN1_INDEX,       DRAIN1);
//	initHss(DRAIN2_INDEX,       DRAIN2);
//	initHss(DRAIN3_INDEX,       DRAIN3);
//	initHss(DRAIN4_INDEX,       DRAIN4);

	// set the default function for ALL possible functions to NO_FUNCTION (no pin)
	for (i=0; i<NUM_HSS_FUNC; i++)
	{
		hssFuncToPinIndex[i] = NO_FUNCTION_HSS;
	}

	// set specific functions to pin mapping for a given architecture

	hssFuncToPinIndex[SPINDLE_COOLANT_HSS] = HSS_AUX_PWR1_INDEX;
	hssFuncToPinIndex[FLOOD_COOLANT_HSS]   = HSS_AUX_PWR2_INDEX;
	hssFuncToPinIndex[CO2_POWER_SUPPY_HSS] = HSS_AUX_PWR4_INDEX;
	hssFuncToPinIndex[DDLIGHT_HSS]         = HSS_AUX_PWR5_INDEX;
	hssFuncToPinIndex[RESPONSE_HSS]        = HSS_AUX_PWR6_INDEX;
	hssFuncToPinIndex[CHAMBER_FAN_HSS]     = HSS_AUX_PWR7_INDEX;
	hssFuncToPinIndex[LASER_XHAIR_HSS]     = HSS_AUX_PWR9_INDEX;
	hssFuncToPinIndex[ULTIMUS_HSS]         = HSS_AUX_PWR1_INDEX; // redundant with SPINDLE_COOLANT


	changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_ON); // turn on DDL Light by default
}

////////////////////////////////////////////////////////////////////////////////

void HssControl(int TicksPerSecond)
{   // run through each of the switch controls
	// for all HSS controls operating in this timescale
	//    if the counter exceeds the duty cycle based compare value, then turn on the output
	//    otherwise turn it off

	int i;

	HssPwmStruct *hss;
	for (i=0; i<NUM_HSS_PINS; i++)
	{
		hss = &HighSideSwitches[i];
		if (hss->TicksPerSecond == TicksPerSecond)
		{
			if (hss->Counter < hss->CompareValue)
				assertControlBit(&hss->Output);   // turn on
			else
				deassertControlBit(&hss->Output);    // turn off
			hss->Counter++;
			if (hss->Counter >= hss->TerminalCount)
			{
				if (hss->oneShot)
				{   // kill output if dealing with a one shot
					hss->CompareValue = 0;
				}
				hss->Counter = 0;                       // wrap counter
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void DDLightSelection(void)
{   //determine value and reset hss control based on value
	if (DDLightFunction < 10)
	{
		// do nothing.... just an on/off light
	}
	else {
		int value=0;
		if ((DDLightFunction >= 10 ) && (DDLightFunction <= 69))
		{
			int motor = (int)(DDLightFunction / 10) - 1;
			switch (DDLightFunction % 10)
			{
			case 0: value = (readSensorBitState(&Motors[motor].HomeSense) == SENSOR_TRIPPED); break;
			case 1: value = (readSensorBitState(&Motors[motor].Limit1Sense) == SENSOR_TRIPPED); break;
			case 2: value = (readSensorBitState(&Motors[motor].Limit2Sense) == SENSOR_TRIPPED); break;
			case 3:
			case 4: value = (readSensorBitState(&Motors[motor].FaultSense) == SENSOR_TRIPPED); break;
			default: value = 0;
			}
		}
		else if ((DDLightFunction >= 70 ) && (DDLightFunction <= 79))
		{
			value = readControlBitValue(&HighSideSwitches[DDLightFunction-71].Output); // 71 is sent for the first selection, not 70 ***** XXX GB
		}
		else
		{
			switch (DDLightFunction)
			{
			case 80: value = readSensorBitState(&startButton);
			default: value = 0;
			}
		}

		if (value == 0)
			changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_OFF);
		else
			changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_ON);
	}
}

////////////////////////////////////////////////////////////////////////////////

int convertToolNumberToDevice(int toolNumber)
{
	if ((toolNumber >= 0) && (toolNumber <= MAX_TOOL_NUMBER))
		return (toolNumberMap[toolNumber]);
	else
		return (toolNumberMap[0]);
}

////////////////////////////////////////////////////////////////////////////////

int convertDeviceToToolNumber(int device)
{
	int toolNumber;
	for (toolNumber=0; toolNumber<=MAX_TOOL_NUMBER; toolNumber++)
	{
		if (device == toolNumberMap[toolNumber])
		{
			return(toolNumber);
		}
	}
	return(toolNumberMap[0]);
}
////////////////////////////////////////////////////////////////////////////////

void CopyToEndOfLine(buffer_t useBuffer)
{
	int count=0;
	int rawChar;    // MUST be of type "int" to handle char 0 to 255 AND the -1 "no data" return value

	for (count=0; count<COMMENT_STRING_LENGTH-1; count++)
	{
		rawChar = GCHAR(useBuffer);
		if (rawChar == -1)
		{
			// underRun .... should never happen here as a full command was needed to get this far
			reportRxUnderRun(useBuffer, "CopyToEndOfLine");
			*GCodeArgPtr=0;
			return; // no reason to continue (really a fatal error)
		}
		else if (rawChar == CMD_END_CHAR)
		{   // end of line found
			*GCodeArgPtr = NULL_CHAR;
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
		PostAcknowledge();
#endif //ALLOW_NATIVE_LIGHTBURN
			return;
		}
		else {
			*GCodeArgPtr = rawChar;     // GCodeArgPtr is left pointing to GCodeArgComment[1] by FillArgumentBuffer
			GCodeArgPtr++;              // prep for next char
		}
	}
	//if you get this far, there was a comment string over run and we need to report it

	sprintf(_errorStr,"Comment Length exceeded Buffer Length (%d) ", COMMENT_STRING_LENGTH);
	sendError(_errorStr);
	*GCodeArgPtr=0;                     // null terminate final string
	purgeTillCr(useBuffer, -1); // -1 indicated to just discard data
}

void processEndofCommandChar(char *currentCommandString)
{
	CommandReadyToProcessFlag=1;
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
		PostAcknowledge();
#endif //ALLOW_NATIVE_LIGHTBURN
	if (_serialPortRxOverrunFlag)
	{
		_serialPortRxOverrunFlag = FALSE;
		sprintf(_errorStr, "RX Overrun while receiving (sometime after line %d)", _gcodeLineNumber); //+1 because the argN for this line has not been processed yet.
		sendError(_errorStr);
		sprintf(_errorStr, "after: **%s**", makeAllCharsPrintable(currentCommandString));
		sendError(_errorStr);
	}
}

void SplitCurrentGcodeLine2Arguments(buffer_t useBuffer)
{
	int ParseIndex=0;
	int rawChar;    // MUST be of type "int" to handle char 0 to 255 AND the -1 "no data" return value
	int csIndex;
	_checkLineChecksum = FALSE;
	_lineChecksum=0;
	ProcessingError=0;
	currentArgLength=0;     //reset ascii length counter
	GCodeArgPtr=&GCodeArgComment[0];//start by pointing to the comment selection first
	currentCommandString[0]='\0'; // rebuild command string as it's processed in case of error (to report full line)
	for (ParseIndex=0;ParseIndex<(MAX_STRING_SIZE-1);ParseIndex++)  // -1 to save room for the \0 to terminate the string
	{   //255 characters max, then throw error -- GB XXX gregkarl ... actually, no error checking
		rawChar = GCHAR(useBuffer) ;// pull it from the correct receive buffer, you will get a -1 if there are no more characters;
		if (rawChar == -1)
		{
			// underRun .... should never happen here as a full command was needed to get this far
			reportRxUnderRun(useBuffer, "SplitCurrentGcodeLine2Arguments");
			CommandReadyToProcessFlag=0;
			return; // no reason to continue (really a fatal error)
		}
		currentCommandString[ParseIndex] = rawChar;
		currentCommandString[ParseIndex+1] = '\0';  // set the null term char for the string

		switch (rawChar)
		{
		case   0:                                       return; //null end of the string
		//filtered  out already: case  7:   pingReply();   return;
		case  CMD_END_CHAR:
			processEndofCommandChar(currentCommandString);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;//  \n  carriage return;
		case  13:   break;  // skip char
		case '1':   *GCodeArgPtr='1';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '2':   *GCodeArgPtr='2';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '3':   *GCodeArgPtr='3';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '4':   *GCodeArgPtr='4';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '5':   *GCodeArgPtr='5';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '6':   *GCodeArgPtr='6';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '7':   *GCodeArgPtr='7';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '8':   *GCodeArgPtr='8';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '9':   *GCodeArgPtr='9';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '0':   *GCodeArgPtr='0';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '.':   *GCodeArgPtr='.';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '-':   *GCodeArgPtr='-';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '+':   *GCodeArgPtr='+';   CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case ' ':   break;//*GCodeArgPtr=' ';GCodeArgPtr++;*GCodeArgPtr=0; break;//POINT TO THE NEXT POSITION PLEASE

		//now process the command characters
		case SOAPSTRING_CHAR:
			FillArgumentBuffer(&GCodeArgComment[0], SOAPSTRING_CHAR);
			CopyToEndOfLine(useBuffer);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;

		case STRING_DELIM:
			FillArgumentBuffer(&_GcodeArgStringParam[0], STRING_DELIM);
			CopyToEndOfLine(useBuffer);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;

		case 'A':   FillArgumentBuffer(&GCodeArgA[0],'A');break;
		case 'B':   FillArgumentBuffer(&GCodeArgB[0],'B');break;
		case 'C':   FillArgumentBuffer(&GCodeArgC[0],'C');break;
		case 'D':   FillArgumentBuffer(&GCodeArgD[0],'D');break;
		case 'E':   FillArgumentBuffer(&GCodeArgE[0],'E');break;
		case 'F':   FillArgumentBuffer(&GCodeArgF[0],'F');break;
		case 'G':   FillArgumentBuffer(&GCodeArgG[0],'G');break;
		case 'H':   FillArgumentBuffer(&GCodeArgH[0],'H');break;
		case 'I':   FillArgumentBuffer(&GCodeArgI[0],'I');break;
		case 'J':   FillArgumentBuffer(&GCodeArgJ[0],'J');break;
		case 'K':   FillArgumentBuffer(&GCodeArgK[0],'K');break;
		case 'L':   FillArgumentBuffer(&GCodeArgL[0],'L');break;
		case 'M':   FillArgumentBuffer(&GCodeArgM[0],'M');break;
		case 'N':   FillArgumentBuffer(&GCodeArgN[0],'N');break;
		case 'O':   FillArgumentBuffer(&GCodeArgO[0],'O');break;
		case 'P':   FillArgumentBuffer(&GCodeArgP[0],'P');break;
		case 'Q':   FillArgumentBuffer(&GCodeArgQ[0],'Q');break;
		case 'R':   FillArgumentBuffer(&GCodeArgR[0],'R');break;
		case 'S':   FillArgumentBuffer(&GCodeArgS[0],'S');break;
		case 'T':   FillArgumentBuffer(&GCodeArgT[0],'T');break;
		case 'U':   FillArgumentBuffer(&GCodeArgU[0],'U');break;
		case 'V':   FillArgumentBuffer(&GCodeArgV[0],'V');break;
		case 'W':   FillArgumentBuffer(&GCodeArgW[0],'W');break;
		case 'X':   FillArgumentBuffer(&GCodeArgX[0],'X');break;
		case 'Y':   FillArgumentBuffer(&GCodeArgY[0],'Y');break;
		case 'Z':   FillArgumentBuffer(&GCodeArgZ[0],'Z');break;
		case '*':   FillArgumentBuffer(&GCodeArgSplat[0],'*');
			// at this point, we hit the marker for the checksum, so calc checksum for portion
			//. of the line already received -- from the reprap gcode page:
			for (csIndex=0; csIndex<ParseIndex; csIndex++)
			{   // calc checksum
				//_lineChecksum = _lineChecksum ^ (byte)currentCommandString[csIndex];  // reprap style -- terrible coverage
				_lineChecksum += (byte)currentCommandString[csIndex];
			}
			if (useBuffer == URGENT_BUFFER)
			{   // the urgent char is stripped upstream, but is included in the incoming checksum.
				_lineChecksum += URGENT_911_CMD_CHAR;
			}
			_checkLineChecksum = TRUE;
		break;
		default:
			ProcessingError = 1;
			purgeTillCr(useBuffer, ParseIndex);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;   // drop the command with unknown char and purge until Cr
		}
		if (ParseIndex == (MAX_STRING_SIZE-2))
		{
			sendError("Command string too long");
		}
	}
}

void CleanUpPointers()
{//come here after plugging in a valid character so you can update all the pointers in the system
	//in the case where the same variable is declared twice, the last ocurance is what will be persistant
	currentArgLength++; //increment the curren ascii length and check for 10 digits maximum
	if (currentArgLength>(MAX_CHARS_FOR_PARAMETER-1))  // -1 because need room for the NULL
	{
		ProcessingError=1;
		return;//just dont do anything yet
	}
	GCodeArgPtr++;      //point to the next charaacterin the buffer
	*GCodeArgPtr=0 ;    //plug in a null just in case
}

void FillArgumentBuffer(char *StringPTR,char  CommandChar)
{
	*StringPTR=CommandChar;
	StringPTR++;//now we have registerd the argument by updateing the argument letter, M,G,X,Y,Z... etc
	GCodeArgPtr= StringPTR;
	*GCodeArgPtr=0;//plug in a null to indicate end of string
	currentArgLength=0;
}


void processArgs(char *WorkBuffer,float *OutPutVariable)
{
	if (*WorkBuffer==0)
	{   // arg not present
		*OutPutVariable = INVALID_ARG_VALUE;//set to invalid value so we will not accidentally take a zero as a position or temperature argument
		return;
	}
	WorkBuffer++;//MOVE OVER 1 CHAR SO WE CAN PROCESS THE NUMBER, NOT THE KEY LETTER
	if (*WorkBuffer==0)
	{   // no value to convert
		ProcessingError = 1;
		*OutPutVariable = INVALID_ARG_VALUE;//set to invalid value so we will not accidentally take a zero as a position or temperature argument
		return;
	}
	*OutPutVariable = atof (WorkBuffer);//start with the second character in the string, because the first character is the argument header char, like M or G or X  etc.
	if (_sendingGBStringsMask & GB_STRING_WORKBUFFER)  // use M797 S<mask> to enable
	{
		sendGB(WorkBuffer);
	}
}

void ResetGcodeParseBuffer()
{//this will reset the ascii input buffers by putting a null in the first character
	GCodeArgComment[0] = 0; //set comment to null as well
	GCodeArgComment[1] = 0; //many routines look past the SOAPSTRING_CHAR location for the comment string.
	GCodeArgA[0] = 0;
	GCodeArgB[0] = 0;
	GCodeArgC[0] = 0;
	GCodeArgD[0] = 0;
	GCodeArgE[0] = 0;
	GCodeArgF[0] = 0;
	GCodeArgG[0] = 0;
	GCodeArgH[0] = 0;
	GCodeArgI[0] = 0;
	GCodeArgJ[0] = 0;
	GCodeArgK[0] = 0;
	GCodeArgL[0] = 0;
	GCodeArgM[0] = 0;
	GCodeArgN[0] = 0;
	GCodeArgO[0] = 0;
	GCodeArgP[0] = 0;
	GCodeArgQ[0] = 0;
	GCodeArgR[0] = 0;
	GCodeArgS[0] = 0;
	GCodeArgT[0] = 0;
	GCodeArgU[0] = 0;
	GCodeArgV[0] = 0;
	GCodeArgW[0] = 0;
	GCodeArgX[0] = 0;
	GCodeArgY[0] = 0;
	GCodeArgZ[0] = 0;
	GCodeArgSplat[0] = 0;
}

//the serial data food chain is as follows
//CHECKIN() will grab the characters from the uart and place them in the receive buffer,
// if character received is a linefeedand then increment commandwaiting flag to indicate that a full line is received and ready to process

//serialProcessor() will grab and parse the command line into respective argument strings, no atof() is run yet
//if there is a format error the serial processor will send back a verbos error messge
//OK will be sent back to host to indicate that a full line was received, we can implement the checksum and line number later
//
//CommandProcessor() takes the parsed strings and performs atof on all valid variables,
//then decides wheather this is an immediate or qued action request
//some codes will be exectued right away, others will want to be performed in order they are received so the build comes out
//correct.
//The majority of m and g codes will be qued for sequential execution, in this case the converted parameters will be copied to the
//next slot in the execution que and appropriate pointers will be updated.
//it is possible to force ANY m or g code to execute immediate by first sending a M911,
//which is emergency/immediate execute of next instruction received
//when a M911 is received then the executimediateflag is set. and this will tell the next  to execute it immediate.
//example
// M911             ;execute next instruction immediately
// G28 X0Y0         ;reset the x and y bed, this will be the next instruction to be executed, unless there is a killmotion override command
//
//there will also be some execute immediate g and mcodes to be added to this procedure in the future.
//;ERR
//
boolean cmdQueIsEmpty(void)
{
	return(CommandsInQue == 0);
}

boolean cmdQueIsFull(void)
{
	if (CommandsInQue >= (SIZE_OF_COMMAND_QUEUE - 3)) // 2 entries reserved for processing and immediate mode (911); and 1 more for the command be executed
		return(TRUE);
	else
		return(FALSE);
}

void printSerialInputStreamError(void)
{
	if (ProcessingError>0)
	{
		sprintf(_errorStr, "serialProcessor: Invalid ascii characters, or variables too long (%d char max) on line %d", MAX_CHARS_FOR_PARAMETER, _gcodeLineNumber);
		sendError(_errorStr);
		sprintf(_errorStr, "**%s**", makeAllCharsPrintable(currentCommandString));
		sendError(_errorStr);
	}
}
void serialProcessor()
{
	buffer_t useBuffer;
	if (_abortInProgress) return;
	if (processSoapstringCommands && (soapstringCommandWaiting==0)) {return;} // still processing soapstring
	if (CommandReadyToProcessFlag){return;}// current command still processing, so can't continue
	if (cmdQueIsFull()){return;}//dont take more commands unless you can handle them

	if (soapstringCommandWaiting)
	{
		protectedDecrement(&soapstringCommandWaiting);
		useBuffer = SOAPSTRING_BUFFER;
	}
	else if (urgentCommandWaiting)
	{
		if (ExecuteImmediateFlag==2)
			return; // still processing last immediate command, need to wait
		protectedDecrement(&urgentCommandWaiting);
		useBuffer = URGENT_BUFFER;
	}
	else if (normalCommandWaiting)
	{
		protectedDecrement(&normalCommandWaiting);
		useBuffer = NORMAL_BUFFER;
	}
	else
	{   // no full commands available in serial buffers
		return;
	}
	ResetGcodeParseBuffer();                //clear first character of each argument
	SplitCurrentGcodeLine2Arguments(useBuffer);  //load each of the argument directly into their respective buffers

	if (_checkLineChecksum)
	{
		processArgs(&GCodeArgSplat[0], &ExecutionPtr->CS);  // need to get the checksum
		if ((uint32_t)ARG_CS != _lineChecksum)
		{
			sprintf(_errorStr, "Line Checksum mismatch (%d/%d) **%s**", (int)_lineChecksum, (int)ARG_CS, makeAllCharsPrintable(currentCommandString));
			sendError(_errorStr);
		}
	}

	if (ProcessingError>0)
	{
		printSerialInputStreamError();
	}
	else
	{
		if (useBuffer == URGENT_BUFFER)
			ExecuteImmediateFlag=1;
		CommandReadyToProcessFlag=1;
	}
}

void InvalidateAllCmdArgs(GMCommandStructure *cmdPtr)
{
	cmdPtr->A = INVALID_ARG_VALUE;
	cmdPtr->B = INVALID_ARG_VALUE;
	cmdPtr->C = INVALID_ARG_VALUE;
	cmdPtr->D = INVALID_ARG_VALUE;
	cmdPtr->E = INVALID_ARG_VALUE;
	cmdPtr->F = INVALID_ARG_VALUE;
	cmdPtr->G = INVALID_ARG_VALUE;
	cmdPtr->H = INVALID_ARG_VALUE;
	cmdPtr->I = INVALID_ARG_VALUE;
	cmdPtr->J = INVALID_ARG_VALUE;
	cmdPtr->K = INVALID_ARG_VALUE;
	cmdPtr->L = INVALID_ARG_VALUE;
	cmdPtr->M = INVALID_ARG_VALUE;
	cmdPtr->N = INVALID_ARG_VALUE;
	cmdPtr->O = INVALID_ARG_VALUE;
	cmdPtr->P = INVALID_ARG_VALUE;
	cmdPtr->Q = INVALID_ARG_VALUE;
	cmdPtr->R = INVALID_ARG_VALUE;
	cmdPtr->S = INVALID_ARG_VALUE;
	cmdPtr->T = INVALID_ARG_VALUE;
	cmdPtr->U = INVALID_ARG_VALUE;
	cmdPtr->V = INVALID_ARG_VALUE;
	cmdPtr->W = INVALID_ARG_VALUE;
	cmdPtr->X = INVALID_ARG_VALUE;
	cmdPtr->Y = INVALID_ARG_VALUE;
	cmdPtr->Z = INVALID_ARG_VALUE;
	cmdPtr->CS = INVALID_ARG_VALUE;
}

void CommandProcessor()
{
	if (_abortInProgress) return;
	if (ExecuteImmediateFlag==2)return; //have not yet processed immediate command //NEW
	if (CommandReadyToProcessFlag==0)return;//wait for a message that the incoming command string has been received and successfully parsed
	//the way this is supposed to work is the command processor adds commands to the command que
	//then the sequence engine executes the delayed commands and counts the number of waiting coded to execute down
	if (cmdQueIsFull()){return;}//dont take more commands unless you can handle them

	ExecutionPtr=&cmdQue[0]; //point to the input parse buffer
	processArgs(&GCodeArgA[0], &ExecutionPtr->A);
	processArgs(&GCodeArgB[0], &ExecutionPtr->B);
	processArgs(&GCodeArgC[0], &ExecutionPtr->C);
	processArgs(&GCodeArgD[0], &ExecutionPtr->D);
	processArgs(&GCodeArgE[0], &ExecutionPtr->E);
	processArgs(&GCodeArgF[0], &ExecutionPtr->F);
	processArgs(&GCodeArgG[0], &ExecutionPtr->G);
	processArgs(&GCodeArgH[0], &ExecutionPtr->H);
	processArgs(&GCodeArgI[0], &ExecutionPtr->I);
	processArgs(&GCodeArgJ[0], &ExecutionPtr->J);
	processArgs(&GCodeArgK[0], &ExecutionPtr->K);
	processArgs(&GCodeArgL[0], &ExecutionPtr->L);
	processArgs(&GCodeArgM[0], &ExecutionPtr->M);
	processArgs(&GCodeArgN[0], &ExecutionPtr->N);
	processArgs(&GCodeArgO[0], &ExecutionPtr->O);
	processArgs(&GCodeArgP[0], &ExecutionPtr->P);
	processArgs(&GCodeArgQ[0], &ExecutionPtr->Q);
	processArgs(&GCodeArgR[0], &ExecutionPtr->R);
	processArgs(&GCodeArgS[0], &ExecutionPtr->S);
	processArgs(&GCodeArgT[0], &ExecutionPtr->T);
	processArgs(&GCodeArgU[0], &ExecutionPtr->U);
	processArgs(&GCodeArgV[0], &ExecutionPtr->V);
	processArgs(&GCodeArgW[0], &ExecutionPtr->W);
	processArgs(&GCodeArgX[0], &ExecutionPtr->X);
	processArgs(&GCodeArgY[0], &ExecutionPtr->Y);
	processArgs(&GCodeArgZ[0], &ExecutionPtr->Z);
	ExecutionPtr->cmdType = UNDEFINED;
	ExecutionPtr->cmdLink = UNPROCESSED;

	if (ARG_N_PRESENT)
		_gcodeLineNumber = ARG_N;
	else
		ARG_N = ++_gcodeLineNumber;

	if (ProcessingError>0)
	{
		printSerialInputStreamError();
		return; // skip line
	}

	if (ARG_M_PRESENT)
	{// process mcode
		switch ((int)ARG_M)
		{
		case 0:   AddCommandToQue(UNDEFINED);    break;//return;//Stop
		case 1:   AddCommandToQue(UNDEFINED);    break;//return;//Sleep
		case 3:   AddCommandToQue(SINGLE_STEP);    break;//return;//Spindle On, Clockwise (CNC specific)
		case 4:   AddCommandToQue(SINGLE_STEP);    break;//return;//Spindle On, Counter-Clockwise (CNC specific)
		case 5:   AddCommandToQue(SINGLE_STEP);    break;//return;//Spindle Off (CNC specific)
		case 6:   AddCommandToQue(SINGLE_STEP);    break;//return;//M6Tool Change();
		case 7:   AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//Mist Coolant On (CNC specific)
		case 8:   AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//Flood Coolant On (CNC specific)
		case 9:   AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//Coolant Off (CNC specific)
		case 10:  ContinueToNextStep();    break;//return;//Vacuum On (CNC specific)
		case 11:  ContinueToNextStep();    break;//return;//Vacuum Off (CNC specific)
		case 17:  AddCommandToQue(SINGLE_STEP);   break;//return;//enable the holding torque on stepping motors
		case 18:  AddCommandToQue(SINGLE_STEP);   break;//return;//releases the holding torque on stepping motors
		case 20:  ContinueToNextStep();    break;//return;//List SD card
		case 21:  ContinueToNextStep();    break;//return;//Initialize SD card
		case 22:  ContinueToNextStep();    break;//return;//Release SD card
		case 23:  ContinueToNextStep();    break;//return;//Select SD file
		case 24:  ContinueToNextStep();    break;//return;//Start/resume SD print
		case 25:  ContinueToNextStep();    break;//return;//Pause SD print
		case 26:  ContinueToNextStep();    break;//return;//Set SD position
		case 27:  ContinueToNextStep();    break;//return;//Report SD print status
		case 28:  ContinueToNextStep();    break;//return;//Begin write to SD card
		case 29:  ContinueToNextStep();    break;//return;//Stop writing to SD card
		case 30:  AddCommandToQue(SINGLE_STEP);       break;//return;//notify end of program
		case 40:  ContinueToNextStep();    break;//return;//Eject Part
		case 41:  AddCommandToQue(SINGLE_STEP); break;//return;//set spindle to LOW speed range
		case 42:  AddCommandToQue(SINGLE_STEP); break;//return;//set spindle to HIGH speed range
		case 43:  ContinueToNextStep();    break;//return;//Stand by on material exhausted
		case 44:  ContinueToNextStep();    break;//return;//Spindle Coolant On
		case 45:  ContinueToNextStep();    break;//return;//Spindle Coolant Off
		case 69:  EchoBackCommentString(); ContinueToNextStep(); break;//return;//used for high speed com test
		// this MCode (M73) can nad will be sent at any time and will often occur at times between to printing moves
		// in order no not have a deferred command to process at this critical time, this MCode will be processed
		// earlier than it should be (will be done in front of the motionQ instead of after it.  will cause
		// an error in the timing by the time it takes to empty the queue, but safer this way
		case 73:  AddCommandToQue(IMPACTS_DESTINATION_CALC);    break;// passthru of remaining printing time (uses P, R)
		case 80:  ContinueToNextStep();    break;//return;//ATX Power On
		case 81:  ContinueToNextStep();    break;//return;//ATX Power Off
		case 82:  AddCommandToQue(IMPACTS_DESTINATION_CALC);       break;//return;// set extruder to absolute mode
		case 83:  AddCommandToQue(IMPACTS_DESTINATION_CALC);       break;//return;// set extruder to relative mode
		case 84:  AddCommandToQue(SINGLE_STEP);       break;//return;//releases the holding torque on stepping motors
		case 91:  AddCommandToQue(SINGLE_STEP);       break;//return;//Set axis_MaxTravel in mm
		case 92:  AddCommandToQue(SINGLE_STEP);       break;//return;//Set axis_steps_per_unit
		case 93:  AddCommandToQue(SINGLE_STEP);       break;//return;//Set home sensor polarity
		case 94:  AddCommandToQue(SINGLE_STEP);       break;//return;//Set Motor direction polarity
		case 95:  AddCommandToQue(SINGLE_STEP);       break;//return;// sets stall sensor polarity (uses X, Y, Z, A, B, C)
		case 96:  AddCommandToQue(SINGLE_STEP);       break;//return;// sets the enable bit polarity (uses X, Y, Z, A, B, C)
		case 97:  AddCommandToQue(SINGLE_STEP);       break;//return;// sets the step bit polarity (uses X, Y, Z, A, B, C)
		case 98:  AddCommandToQue(SINGLE_STEP);       break;//return;// sets limit1 sensor polarity (uses X, Y, Z, A, B, C)
		case 99:  AddCommandToQue(SINGLE_STEP);       break;//return;// sets limit2 sensor polarity (uses X, Y, Z, A, B, C)
		//RETIRED case 101: AddCommandToQue(SINGLE_STEP);    break;//return;//M_Code_M101(); return;//turn on extuder, set rate and cont extrude
		//RETIRED case 102: AddCommandToQue(SINGLE_STEP);    break;//return;//M_Code_M102(); return;//turn on extuder, set rate and cont reverse extrude
		case 103: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M103(); return;//turn extruder off
		case 104: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M104(); break;//return;//set desired temperature to Sxxx.x
		case 105: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M105(); break;//return;//get extruder temperature
		case 106: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M106(); break;//return;//turn fan on (S=duty)
		case 107: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M107(); break;//return;//turn fan off
		//RETIRED case 108: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M108(); break;//return;//set extruder speed to Fxx.x amount per min
		case 109: AddCommandToQue(SINGLE_STEP);    break;//return;//M_Code_M109(); break;//return;//wait for extruder to reach temp
		case 114: M_Code_M114();  ContinueToNextStep(); break;//return;//report xyz location immediately
		case 115: M_Code_M115();  ContinueToNextStep(); break;//return;//tell them what firmware revision level we are using
		case 140: AddCommandToQue(SINGLE_STEP);       break;//return;//M_Code_M140();  break;//return;//Set bed temperature
		case 141: AddCommandToQue(SINGLE_STEP);    break;//return;//M_Code_M141(); break;//return;// set chamber temperature (uses S)
		case 142: ContinueToNextStep();  break;//return;// set the holding pressure on build platform, ie vacuum hold down
		case 143: ContinueToNextStep();  break;//return;//Maximum hot-end temperature
		case 160: ContinueToNextStep();  break;//return;//Number of mixed materials
		case 190: AddCommandToQue(SINGLE_STEP);     break;//return;//wait for hotbed to reach temp (uses S)
		case 191: AddCommandToQue(SINGLE_STEP);     break;//return;//wait for chamber to reach temp (uses S)
		case 200: ContinueToNextStep();  break;//return;//Set filament diameter or volume
		case 201: ContinueToNextStep();  break;//return;//Set max printing acceleration
		case 202: ContinueToNextStep();  break;//return;//Set max travel acceleration
		case 203: AddCommandToQue(SINGLE_STEP);     break;//return;//Sets the Maximum G0/Rapid velocity UNIT/MIN (and Homing/Accel ramp)
		case 204: AddCommandToQue(SINGLE_STEP);     break;//return;//set no ramp speed, no acceleration below this point
		case 205: AddCommandToQue(SINGLE_STEP);     break;//return;//set home speed
		case 206: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the homing routine hysteresis location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 207: ContinueToNextStep();  break;//return;//calibrate z axis by detecting z max length
		case 208: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the acceleration constant (uses X, Y, Z, A, B, C)
		case 209: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the minimum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 210: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the maximum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		//RETIRED case 211: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the machine minimum pulse rate limit (uses S)
		//RETIRED case 212: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the machine maximum pulse rate limit (uses S)
		case 213: AddCommandToQue(SINGLE_STEP);  break;//return;// sets the per motor/axis installation status (uses X,Y,Z,A,B,C)
		case 214: AddCommandToQue(SINGLE_STEP);  break;//return;// sets the per motor/axis type (linear/rotary) (uses X,Y,Z,A,B,C)
		case 215: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the homing routine start location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 216: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the homing routine end location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 217: AddCommandToQue(SINGLE_STEP);  break;//return;// sets the max deceleration rate (for abort) (mm/sec/sec)  (uses X, Y, Z, A, B, C)
		case 218: AddCommandToQue(SINGLE_STEP);  break;//return;// set the fixture offsets  (uses O, X, Y, Z, A, B, C)
		case 219: M_Code_M219(); ContinueToNextStep(); break;//return;// sets the serial baud rate (uses B)
		//RETIRED case 220: AddCommandToQue(SINGLE_STEP);  break;// sets the per motor/axis send motorStep pulse as a CANbus command (uses T,X,Y,Z,A,B,C)
		case 221: AddCommandToQue(IMPACTS_DESTINATION_CALC);    break;//return;//set extrude calculation factors (uses T,S,Z,W,P)
		case 222: AddCommandToQue(SINGLE_STEP);  break;//return;// set the centripetal accelleration radius (uses S)
		case 223: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the re-homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 224: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the jog no ramp speed (UNITS/MIN) (no acceleration needed below this) (uses X, Y, Z, A, B, C)
		case 225: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the jog speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 226: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the jog acceleration constant (uses X, Y, Z, A, B, C, P)
		case 227: AddCommandToQue(SINGLE_STEP);  break;//return;// control jogging using the AB encoder on the panel interface (uses XYZABCR>
		case 228: AddCommandToQue(SINGLE_STEP);  break;//return;//Disable Automatic Reverse and Prime
		case 229: AddCommandToQue(SINGLE_STEP);  break;//return;// extrusion control (uses E, P, S)
		case 230: AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;// set global flow rate override percentage (uses S)
		case 231: AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;// set motion feedrate override percentage (uses S)
		case 232: AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;// set motion override percentage (uses S)
		case 233: AddCommandToQue(SINGLE_STEP);  break;//return;// set homing pop-off distance (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 234: AddCommandToQue(SINGLE_STEP);  break;//return;// set motor position (absolute) (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 235: AddCommandToQue(SINGLE_STEP);  break;//return;// set rotary axis plane and offset (uses S, P, A, Y, Z)
		case 236: AddCommandToQue(SINGLE_STEP);  break;//return;// write device type and revision to flash (option bytes) (uses T, S, I, P)
		case 237: AddCommandToQue(SINGLE_STEP);  break;//return;// set cold extrusion prevention parameters (uses T, C, U, L, R)
		case 238: AddCommandToQue(SINGLE_STEP);  break;//return;// sets the per motor/axis execute E value (uses T,X,Y,Z,A,B,C)

		case 240: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// turn device switch off (uses T, I)
		case 241: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// turn device switch on (uses T, I)
		case 242: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// control device switch by dutyCycle (uses T, I, S)
		case 243: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// control device switch by pwm (uses T, I, S, P)
		case 244: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// control device switch by temperature (uses T, I, S)
		case 245: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// set switch flag(s) (uses T, I, P, E, H, C)
		case 253: AddCommandToQue(SINGLE_STEP);  break; // turn on lathe motor for continuous CW motion (uses S)
		case 254: AddCommandToQue(SINGLE_STEP);  break; // turn on lathe motor for continuous CCW motion (uses S)
		case 255: AddCommandToQue(SINGLE_STEP);  break; // turn off lathe motor (uses S)
		case 260: AddCommandToQue(SYNCS_WITH_MOTION);  break; // control display attached to a head (uses S,P,X,Y,I,J,R))
		case 261: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))
		case 262: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))
		case 263: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))

			
			
		case 600: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// disable all HSS outputs
		case 601: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out1
		case 602: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out2
		case 603: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out3
		case 604: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out4
		case 605: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out5
		case 606: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out6
		case 607: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out7
		case 608: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out8
		case 609: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable HSS out9
		case 610: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable OD1
		case 611: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable OD2
		case 612: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// enable OD3
		case 613: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD4
		case 614: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD5
		case 615: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD6
		case 616: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD7
		case 617: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD8
		case 618: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable spi3 relays
		case 619: AddCommandToQue(SYNCS_WITH_MOTION);  break;//return;// sets the function and output pwm of the selected HSS (uses F, I, S, P, J, H)
		case 620: AddCommandToQue(SINGLE_STEP);break;//return;// Laser global control (uses T, E, F, C, P)
		case 621: AddCommandToQue(SYNCS_WITH_MOTION);break;//return;// Laser vector mode control (uses P)
		case 622: AddCommandToQue(SINGLE_STEP);break;//return;// Laser raster mode control (uses O, S, D, P, I)
		case 623: AddCommandToQue(SYNCS_WITH_MOTION);break;//return;// Laser pulse (one-shot) mode control (uses P, D)
		case 624: AddCommandToQue(SINGLE_STEP);break;//return;// setup raster image (uses X, Z, I, J)
		case 625: AddCommandToQue(SINGLE_STEP);break;//return;// inkjet vector mode control (uses S, J)
		case 626: AddCommandToQue(SINGLE_STEP);break;//return;// build color index table (uses C, U, A, D)
		case 627: AddCommandToQue(SINGLE_STEP);break;//return;// set job_kill/abort auto move location (uses AXZABC IJKUVW)
		case 628: AddCommandToQue(SINGLE_STEP);break;//return;// arm/disarm digital trigger (uses P, E, R)
		case 629: AddCommandToQue(SYNCS_WITH_MOTION);break;//return;// open log file
		case 630: AddCommandToQue(SINGLE_STEP);break;//return;// canbus touch probe control (uses T, S, D, P)
		case 631: AddCommandToQue(SINGLE_STEP); break;//return;  // PickNPlace data (uses T, H, P, A, B, C, D)
		case 632: AddCommandToQue(SINGLE_STEP); break;//return;  // PickNPlace control (uses T, S, H, P, V, F, D,)

		case 660: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//set tool length and diameter
		case 670: M_Code_M670(); ContinueToNextStep(); break;//return;//turn on DD light with Duty Sarg
		case 671: M_Code_M671(); ContinueToNextStep(); break;//return;//turn on hotbed danger backlight with duty  Sarg
		case 672: M_Code_M672(); ContinueToNextStep(); break;//return;//attaches the DDlight to one of the sensor status
		case 673: AddCommandToQue(SYNCS_WITH_MOTION);break;//return;// ContinueToNextStep(); break;//return;//turns light on with code synchronized the gcode execution.
		case 674: AddCommandToQue(SINGLE_STEP);break;//return;//sets it in TURBO MODE
		case 675: M_Code_M675(); ContinueToNextStep(); break;//return;   //sets the response light hss pwm
		case 676: M_Code_M676(); ContinueToNextStep(); break;//return;   //sets the chamber fan pwm
		case 677: M_Code_M677(); ContinueToNextStep(); break;//return;   //sets the control panel buzzer pwm
		case 678: M_Code_M678(); ContinueToNextStep(); break;//return;   //set the laser cross-hair pwm
		case 679: M_Code_M679(); ContinueToNextStep(); break;//return;   //set the vacuum pwm
		case 680: M_Code_M680(); ContinueToNextStep(); break;//return;//sets the z axis home offsets only use if you are sure.
		case 681: M_Code_M681(); ContinueToNextStep(); break;//return;//sets the z axis home offsets only use if you are sure.
		case 682: AddCommandToQue(SINGLE_STEP); break;//return;//Calibrate Z axis sensor locations
		case 683: AddCommandToQue(SINGLE_STEP); break;//return; // set the headroom for the normal serial rx buffer (uses S)
		case 685: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M685(); ContinueToNextStep(); break;//return;   //sets the air assist pwm
		case 686: M_Code_M686(); ContinueToNextStep(); break;//return; //AddCommandToQue(SINGLE_STEP); break;//return;//returns the machine info string for machine key creation
		case 687: M_Code_M687(); ContinueToNextStep(); break;//return; //AddCommandToQue(SINGLE_STEP); break;//return;//unlock system with machine specific password
		case 688: AddCommandToQue(SINGLE_STEP); break;//return;//RESERVED - DO NOT USE
#ifdef ALLOW_GCODE_SCRIPTS
		case 690: M_Code_M690(); ContinueToNextStep(); break;//return; // add/delete scripts
#endif
		case 698: AddCommandToQue(SINGLE_STEP); break;//return;  // humiture control (uses T, V)
		case 699: AddCommandToQue(SINGLE_STEP); break;//return; // hx711 control (uses T, V, S, O, Z)
		case 701: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M701(); break;//return; // set auto status update rate and page selection (uses S, P, E)
		case 702: AddCommandToQue(UNDEFINED);   break;  //M_Code_M702(); break;  // seed a group from another head/group's values (uses T, S)
		case 703: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M703(); break;//return; // add device to Group (uses S, P)
		case 704: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M704(); break;//return; // remove device to Group (uses S, P)
		case 705: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M705(); break;//return; // reset HH (uses P)
		case 706: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M706(); break;//return; // sync HH (uses P)
		case 707: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M707(); break;//return; // STOP HH (uses P)
		case 708: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M708(); break;//return; // pause HH (uses P)
		case 709: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M709(); break;//return; // resume device (from pause or stop) (uses T)
		case 710: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M710(); break;//return; // enable/disable RTD1 (uses S, P)
		case 711: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M711(); break;//return; // enable/disable RTD2 (uses S, P)
		case 712: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M712(); break;//return; // enable/disable RTD3 (uses S, P)
		case 713: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M713(); break;//return; // set default P value for missing argP
		case 714: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M714(); break;//return; // update devicePosition alias table
		case 715: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M715(); break;//return; // set LED display selection
		case 716: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M716(); break;//return; // turns off logging of aux comm in repetrel
		case 717: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M717(); break;//return; // turns on logging of aux comm in repetrel
		case 718: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M718(); break;//return; // used by repetrel to synchronize M719 data logging
		case 719: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M719(); break;//return; // set reporting rate and data for host traffic (uses S, E)
		case 720: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M720(); break;//return; // direct MAIN extrusion control (uses P, F, E, S)
		case 721: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M721(); break;//return; // direct UNPRIME extrusion control (uses P, F, E, S)
		case 722: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M722(); break;//return; // direct PRIME extrusion control (uses P, F, E, S)
		case 723: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M723(); break;//return; // direct MANUAL extrusion control (uses P, F, E, S)
		case 724: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M724(); break;//return; // direct DWELL (no stepping) extrusion control (uses P, F, E, S)
		case 725: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M725(); break;//return; // set the Karl Factors for controlling the heater switch (uses T, P, E, S)
		case 726: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M726(); break;//return; // set the Karl Factors for controlling the fan switch (uses T, P, E, S)
		case 727: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M727(); break;//return; // set LED override values and mask (uses T, S, P)
		case 728: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M728(); break;//return; // set motor current boost control (uses T, S)
		case 729: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M729(); break;//return; // set motor microsteps control (uses T, S)
		case 730: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M730(); break;//return; // set not to exceed temp for motor (uses T, S)
		case 731: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M731(); break;//return; // set not to exceed temp for heater (uses T, S)
		case 732: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M732(); break;//return; // set maximum step rate for motor (microsteps/sec) (uses T, F)
		case 733: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M733(); break;//return; // set maximum allowable RTD temperature delta (uses T, S)
		case 734: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M734(); break;//return; // set HH error reporting rate for redundant error codes (uses T, F)
		case 735: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M735(); break;//return; // fill the incoming page data buffer with S (uses S)
		case 736: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M736(); break;//return; // fill the outgoing page data buffer with S (uses S)
		case 737: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M737(); break;//return; // erase flash page in selected (physical) device (uses T, I, P)
		case 738: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M738(); break;//return; // transfer data page from (physical) device to incoming buffer (uses T, I, P, S)
		case 739: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M739(); break;//return; // transfer data page from incoming to outgoing buffer
		case 740: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M740(); break;//return; // transfer data page from outgoing buffer to (physical) device (uses T, I, P, S)
		case 741: M_Code_M741(); ContinueToNextStep(); break;//return; // transfer incoming page related data from the device to the to host (uses S)
		case 742: M_Code_M742(); ContinueToNextStep(); break;//return; // transfer data page in ASCII text from host to outgoing buffer (uses S, P, comment) <-- can't go in queue because of needing comment
		case 743: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M743(); break;//return; // transfer generally (read only) device info from inbox to host (uses T, S)
		case 744: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M744(); break;//return; // transfer alias list from device to inbox (uses T)
		case 745: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M745(); break;//return; // force HH to invert polarity of direction pin (uses P)
		case 746: M_Code_M746(); ContinueToNextStep(); break;//return; // start the bootloader for the selected physical device (uses T)
		case 747: M_Code_M747(); ContinueToNextStep(); break;//return; // prepare device for download (uses P)
		case 748: M_Code_M748(); ContinueToNextStep(); break;//return; // process next line of intel hex format bootloader data (uses P, comment) <-- can't go in queue because of needing comment
		case 749: M_Code_M749(); ContinueToNextStep(); break;//return; // exit the device bootloader
		case 750: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M750(); break;//return;    // unlock flash for erase/write access for the selected physical device (uses T)
		case 751: AddCommandToQue(UNDEFINED);   break;//return; //M_Code_M751(); break;//return;    // lock flash to prevent erase/write access for the selected physical device (uses T)
		case 752: M_Code_M752(); ContinueToNextStep(); break;//return; // write hardware type to flash (option bytes) using device bootloader (uses S)
		case 753: M_Code_M753(); ContinueToNextStep(); break;//return; // write hardware revision to flash (option bytes) using device bootloader (uses  S)
		case 754: M_Code_M754(); ContinueToNextStep(); break;//return; // write hardware key to flash (option bytes) using device bootloader (uses S)
		case 755: AddCommandToQue(SYNCS_WITH_MOTION);   break;//return; //M_Code_M755(); break;//return;    // set extruder heater pwm (uses T, S)
		case 756: AddCommandToQue(IMPACTS_DESTINATION_CALC);   break;//return; //M_Code_M756(); break;//return;    // set layer height (mm) (uses S)
		case 757: AddCommandToQue(IMPACTS_DESTINATION_CALC);   break;//return; //M_Code_M757(); break;//return;    // set layer/path weight (mm) (uses S)
		case 758: AddCommandToQue(IMPACTS_DESTINATION_CALC);   break;//return; //M_Code_M758(); break;//return;    // set extrusion step to volume conversion (steps per nL) (uses T, S)
		case 759: AddCommandToQue(SINGLE_STEP);   break;//return; //M_Code_M759(); break;//return;    // enable temperature calibration (uses T, S)
		case 760: AddCommandToQue(SINGLE_STEP);   break;//return; //M_Code_M760();    break;//return; // disable temperature calibration
		case 761: M_Code_M761(); ContinueToNextStep(); break;//return;  // transfer system info in ASCII text from main board to host (uses S, P)
		case 762: M_Code_M762(); ContinueToNextStep(); break;//return;  // transfer system info in ASCII text from host to main board (uses S, P, comment)
		case 763: AddCommandToQue(SINGLE_STEP);   break;//return; //M_Code_M763(); break;//return;    // clear error on selected device(s) (uses T)
		case 766: M_Code_M766(); ContinueToNextStep(); break;//return; // start the system bootloader process
		case 767: M_Code_M767(); ContinueToNextStep(); break;//return; // prepare system  for download (uses P)
		case 768: M_Code_M768(); ContinueToNextStep(); break;//return; // process next line of intel hex format of system bootloader data (uses P, comment) <-- can't go in queue because of needing comment
		case 769: M_Code_M769(); ContinueToNextStep(); break;//return; // exit the system bootloader
		case 770: M_Code_M770(); ContinueToNextStep(); break;//return; // leave system bootloader and jump to application main()
		case 771: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M771(void)  // load laser image data controls (scale, offset, max)
		case 772: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M772(void)  // reset metrics for new job
		case 773: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M773(void)  // send motion metrics to host
		case 774: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M774(void)  // send queue metrics to host
		case 775: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M775(void)  // send current status/queue values to host
		case 776: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M776(void)  /// send cmd/motionQ usage histograms to host
		case 777: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M777(void)  // send and/or erase the crash log (uses S, E, D)
		case 778: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M778(void)  // enable slice time measurement (uses I, S)
		case 779: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M779(void)  // dump slice time measurements
		case 780: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M780(void)  // enable/disable auto XYZABC position reporting
		case 781: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M781(void)  // write hardware type to flash (option bytes) (uses T, D, O, P)
		case 782: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M782(void)  // enable/disable "print air" feature (uses S)
		case 783: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M783(void)  // set PersistantUltimusControlHeadAddress
		case 784: AddCommandToQue(SINGLE_STEP); break;  //M_Code_M784(void)  // report system info (version numbers of system and all heads)
		case 785: AddCommandToQue(SINGLE_STEP); break;//return; //_Code_M785(void)  // Set motor parameters (uses T,U,A,R,B,P,C,O,S) (V1)
		case 786: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M786(void)  // set closed loop stepper PID control values (uses T, P, I, D)
		case 787: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M787(void)  // calibrate can based closed-loop motor (uses T, P, C)
		case 788: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M788(void)  // reset can based closed-loop axis motor (uses T, F, D, P)
		case 789: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M789(void)  // send sideband step pulses to a canAxisMotor (uses T, S)
		case 790: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //notify when layer is changed
		case 791: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //take a picture, add to que so it can fire at the right time
		case 792: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //SEND A TEXT, add to que so it can fire at the right time
		case 793: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //EXECUTE SHELL COMMAND, add to que so it can fire at the right time
		case 794: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //take a picture, add to que so it can fire at the right time
		case 795: AddCommandToQue(SINGLE_STEP); break;//return; // sets jogvalueValueInUnits (uses S)
		case 796: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the jog increment (uses X, Y, Z, A, B, C)
		case 797: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M797(); break;//return; // enable/disable debug reporting strings ">GB:" (uses S)
		case 798: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M798(); break;//return; // dump strings to host (debug MCODE) -- warning, must reset after using this MCODE (uses T)
		case 799: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M799(); break;//return; // get PLL and Clock status for 407
		case 800: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M800(void)  // sonicator control
		case 868: AddCommandToQue(SINGLE_STEP);  break;//return; //M_Code_M868();  break;//return;// Move table up
		case 869: AddCommandToQue(SINGLE_STEP);  break;//return; //M_Code_M869();  break;//return;// Move table down
		case 960: AddCommandToQue(UNDEFINED); break;//return; //M_Code_960 ; rectangluar pocket mill
		case 7734: CommandReadyToProcessFlag=0; M_Code_M7734(); break;//return; // diags
		default:
			sprintf(_errorStr, "CommandProcessor: Unsupported MCode M%d (~L:%d/%d)", (int)ARG_M, (int)ARG_N, strlen(currentCommandString));
			sendError(_errorStr);
			sprintf(_errorStr, "**%s**", makeAllCharsPrintable(currentCommandString));
			sendError(_errorStr);
			CommandReadyToProcessFlag=0;
			break;//return;
		}
	}
	else if (ARG_G_PRESENT)
	{
		switch ((int)ARG_G)
		{
		case 0:   AddCommandToQue(ADD_TO_MQ);  break;//return;//G_Code_G0();   break;//return;//move rapid
		case 1:
		{
			switch (getArgFraction(ARG_G))
			{
			case 0: AddCommandToQue(ADD_TO_MQ);  break;//return;//G_Code_G1();   break;//return;//move at feed rate
			case 1: AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G1_1();   break;//return;//raster move (laser)
			default : break;
			}
			break;
		}
		case 2:   AddCommandToQue(ADD_TO_MQ);  break;//return;//G_Code_G2();   break;//return;//move arc cw
		case 3:   AddCommandToQue(ADD_TO_MQ);  break;//return;//G_Code_G3();   break;//return;//move arc ccw
		case 4:   AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G4();   break;//return;//Dwell in ms
		case 12:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G12();  break;//return;//CIRCULAR BORE CW
		case 13:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G12();  break;//return;//CIRCULAR BORE CCW
		case 16:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G16(); //set arc plane to NONE (user defineable)
		case 17:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G17(); //set arc plane to XY
		case 18:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G18(); //set arc plane to XZ
		case 19:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G19(); //set arc plane to YZ
		case 20:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G20();  break;//return;//inch dimensions
		case 21:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G21();  break;//return;//mm dimension
		case 28:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G28();  break;//return;//home axis, responds to individual axis if args are passed, X0Y0 homes only x and y not Z
		case 29:  AddCommandToQue(UNDEFINED);  break;//return;//G_Code_G29();  break;//return;//special exercise move
		case 38:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G38();  touch probe
		case 53:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G53();   set fixture offsets OFF
		case 54:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G54();   set fixture offsets ON
		case 55:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G55();   set fixture offsets ON
		case 56:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G56();   set fixture offsets ON
		case 57:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G57();   set fixture offsets ON
		case 58:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G58();   set fixture offsets ON
		case 59:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G59();   set fixture offsets ON
		case 70:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G70(void);//G70 Bolt Hole Circle
		case 71:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G71(void);//G71 Bolt Hole Arc
		case 73:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G73(void);//G73 High-Speed Peck Drilling Canned Cycle
		case 74:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G74(void);//G74 Reverse Tap Canned Cycle
		case 76:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G76(void);//G76 Fine Boring Canned Cycle
		case 77:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G77(void);//G77 Back Bore Canned Cycle
		case 80:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G80(void);//G80 Canned Cycle Cancel
		case 81:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G81(void);//G81 Drill Canned Cycle
		case 82:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G82(void);//G82 Spot Drill Canned Cycle
		case 83:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G83(void);//G83 Normal Peck Drilling Canned Cycle
		case 84:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G84(void);//G84 Tapping Canned Cycle
		case 85:  AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G85(void);// G85 Boring Canned Cycle
		case 90:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G90();  break;//return;//absolute programming coordinates
		case 91:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G91();  break;//return;//incremental programming
		case 92:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G92();  break;//return;//set G92 offsets
		case 93:  AddCommandToQue(IMPACTS_DESTINATION_CALC);  break;//return;//G_Code_G93();  break;//return;//clea G92 offsets
		//NUKE case 101: AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G101();   break;//return;//raster move (laser)
		case 702: AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G702();  break;//return;//cylinder print
		case 703: AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G703();  break;//return;//cylinder print
		case 928: AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G928();//walk motor back and set home to destination.
		default:
			sprintf(_errorStr, "CommandProcessor: Unsupported GCode G%d (~L:%d/%d)", (int)ARG_G, (int)ARG_N, strlen(currentCommandString));
			sendError(_errorStr);
			sprintf(_errorStr, "**%s**", makeAllCharsPrintable(currentCommandString));
			sendError(_errorStr);
			CommandReadyToProcessFlag=0;
			break;//return;
		}
	}
	else if (GCodeArgT[0]=='T')
	{
		// if you get here the line did NOT have a G code or a M code, rather ONLY a 'T' command
		// we can convert it to a M6 command and pass it on
		// ie, M6 T12 O2   means use print head 2 and offset 2, if there is a X Y or Z
		// command then it defines the NEW Offset

		int toolNumber = (int)ARG_T;
		InvalidateAllCmdArgs(ExecutionPtr); // make sure no unexpected args were set
		// build up and M6 command:
		ARG_M = 6.0f;   // setting up M6
		ARG_I = _blockImmediateMoveOnToolChange ? 0.0f : 1.0f;   // 1.0 == force immediate move to new offset location

		if ((toolNumber >= 0) && (toolNumber <= MAX_TOOL_NUMBER))
		{
			ARG_T = (float)convertToolNumberToDevice(toolNumber);
			if (!deviceIsAHotbed((byte)ARG_T))
			{   // only spec an offset for non hotbeds
				ARG_O = toolNumber + 1;                        // offset in motor array is + 1
			}
		}
		else if ((toolNumber >= ALIAS_TOOL_NUMBER_MIN) && (toolNumber <= ALIAS_TOOL_NUMBER_MAX))
		{   // must be using a group
			ARG_T = (float)toolNumber;
			ARG_O = toolNumber + 1;                        // offset in motor array is + 1
		}
		else
		{   // error!!!!
			ARG_T = (float)HH_POSITION_UNPLUGGED;
		}

		if (ARG_T != (float)HH_POSITION_UNPLUGGED)
		{   // only process if valid T arg was given
			AddCommandToQue(SINGLE_STEP);      //send M6 to cmdQue on for later processing in sequence
		}
	}
	CommandReadyToProcessFlag=0;
}
void EchoBackCommentString(void)
{
	sprintf(SendString, ">MC:M%3d: ;",(unsigned int) ARG_M);
	sendstring(SendString);
	sendstringCr(GCodeArgComment);
}
int getNextCommandIndex(int index)
{
	index++;
	if (index >= SIZE_OF_COMMAND_QUEUE)
		index = 2; //circular que so make sure you do not overrun
	return(index);
}
int getNextDeferredCommandIndex(int index)
{
	index++;
	if (index >= SIZE_OF_DEFERRED_COMMAND_QUEUE)
		index = 0; //circular que so make sure you do not overrun
	return(index);
}
void ContinueToNextStep(void)
{   //resets the flag so we can proceed on to the next instruction, will eventually log or announce if this has occurred
	CommandReadyToProcessFlag=0;
	ExecuteImmediateFlag=0;
}
void AddCommandToQue(command_t type)
{   //cmdQue[0] is the freshly parsed command line decoded, so now we can plug it in the que, unless it is execute immediate
	cmdQue[0].cmdType = (_blockAllMotion) ? SINGLE_STEP : type;  // keep things simple when blocking
	cmdQue[0].cmdLink = UNPROCESSED;
	if (ExecuteImmediateFlag==0)
	{
		cmdQue[NextCommandInsertionIndex] = cmdQue[0];
		NextCommandInsertionIndex = getNextCommandIndex(NextCommandInsertionIndex);
		CommandsInQue++;//increment the commands on the que to do
#ifdef COLLECT_METRICS
		if (CommandsInQue > _metrics.max_CommandsInQue)
			_metrics.max_CommandsInQue = CommandsInQue;
#endif
	}
	else // if (ExecuteImmediateFlag > 0)
	{
		cmdQue[1] = cmdQue[0];//load fresh command prams into the immediate execute buffer
		ExecuteImmediateFlag=2;//tell them that the command is loaded and ready to fire
	}
	CommandReadyToProcessFlag=0;//clear the flag as we have already processed the message
}

void resetVariableOnJobEnd(boolean realJobEnd)
{
	_gs._errorThrottledCount=0;
	_gs._errorCount=0;
	//_gcodeLineNumber=0;//this is used to give feedback on which line number is being executed
	CurrentLayer=0;//initialize the layer to its current height
	currentFixtureIndex=0;  //G53
	currentToolIndex=0;
	currentHeadIndex=0;//disable all offsets at end of program
	_hijackAxisC = FALSE;
	_canbusStepForE = FALSE;
	_directStepForE = FALSE;
	_M106_fanDutyCycleRange=100.0f;
	TurboModeDistance=MAXFLOAT;
	_printAir=FALSE;
	_sendingGBStringsMask = 0;
	_sendingGBStringsSubMask = 0;
	resetSoapstringControl();
	removeAllRegisteredDevice();
	resetStickyErrorFlags();
	sendSetSwResetBit(0);//reset all print heads as well = 0
	_LastExtrusionRate = INVALID_ARG_VALUE; // force a resend of the rate
	KillCannedCycle();
#ifdef RESET_ABORT_MOVE_ON_JOB_END
	if (realJobEnd)
	{
		// the cannot be part of the normal resetVariableOnJobEnd() as that is also called with an abort/job kill
		// which would wipe out these setting before they could get used....so only do this as part of a real M30
		MotorStructure *M;
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors
			M->AbortAbsolutePosition = INVALID_ARG_VALUE;
			M->AbortRelativePosition = INVALID_ARG_VALUE;
		}
	}
#endif //RESET_ABORT_MOVE_ON_JOB_END
}

void initGCodeSequenceEngine(void)
{
	int i;
	motionQ_init();
	NextCommandIndex=2;       //points to the next command to be executed, initially set to 2 after the first command is received and parsed
	CurrentCommandIndex=2;    // points to the current place in cmdQue[] from which to get prams for execution
	NextCommandInsertionIndex=2;//points to next place in the command que to place incoming commands
	CommandsInQue=0;                //nothing to process, we just got started and the buffers are flushed
	NextDeferredCommandIndex=0;       //points to the next command to be executed, initially set to 2 after the first command is received and parsed
	NextDeferredCommandInsertionIndex=0;//points to next place in the command que to place incoming commands
	DeferredCommandsInQue=0;     // nothing processed yet, so no deferred commands
	for (i=0; i<SIZE_OF_DEFERRED_COMMAND_QUEUE; i++)
	{
		deferredCmdQue[i].cmdState = DEFERRED_CMD_INVALID;
	}
	_needToProcessDeferredCommands = FALSE;
	ExecuteImmediateFlag=0;     //0 meand normal queing, 1 means next received command will go to front of execution, 2 means ready to execute immediate
	CommandReadyToProcessFlag=0;

	resetVariableOnJobEnd(FALSE); // same settings as when an M30 is executed

	_g4DwellTimer=0;//turn off any processing wait timer so print can continue
	_gcodePaused = FALSE;
	_blockAllMotion=FALSE;  // abort has complete, so clear this flag so relative moves can occur if allowed
	_blockAllMotionMessageSent = FALSE;
	_blockAbsoluteMotionMessageSent = FALSE;
	_blockMotionForJoggingMessageSent = FALSE;
	_blockMotionForHighVoltageMessageSent = FALSE;
	_blockJoggingForMotionMessageSent = FALSE;
	CannedCycleLastMove=TRUE;
	KillCannedCycle();//reset default params for drilling cycles
	CannedCycleFlag=0;
	CannedCycleStep=0;
	_gcodeLineNumber=0;
	_gcodeMotionLineNumber=0;
	G2G3Flag=0;
	initArcStruct(&_arc);   //reset all 'new' arc style variable (will stop current arc)
	_reverseFrogToe=FALSE;
	_deltaRForPass = 0.0f;
	_spiralRoughingPass = FALSE; // SPIRAL
	G203State=0;//just in case we are in the middle of a complex move
	TurboModeDistance=MAXFLOAT;
	_requestToPauseAtEndOfMove=FALSE;
	_requestToAbortAtEndOfMove=FALSE;
	_requestToPauseAtEndOfLayer=FALSE;
	PersistantUltimusControlHeadAddress=HH_POSITION_UNPLUGGED;//turn off the external control of the optimus 3,5
	_MailBoxes._waitingFor.flags.u32 = 0;  // disable any items that are blocking processing (resets ALL flags)
	PrimeTimer=0;
	UnPrimeTimer=0;
	_autoReverseAndPrimeMinTime=0.0f;
	_hijackAxisC = FALSE;
	_canbusStepForE = FALSE;
	_directStepForE = FALSE;

	currentFixtureIndex=0;//use offset zero, which should be all zeros
	currentHeadIndex=0;//reset the tool offset selection to zero which is alway null
	currentToolIndex=0;
	ClearAllMotorG92Offset();

	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors and set the flag showing err msg has not been sent
		M->autoReportPosition = TRUE;
		M->reportPositionWithStatus = FALSE;
		M->HomeSense.ErrorMessageSent = FALSE;
		M->LastReportedPosition = INVALID_ARG_VALUE;
		M->PULSES_IN_MOVE = 0;
		M->PULSES_TO_GO = 0;
		M->Q_POSITION =  M->POSITION;
		M->Q_LastRequestedPositionInUnits = (M->POSITION * M->UnitsPerPulse) - SumOfAllMotorOffsets(M);
		M->TimerPSC = 0;
		M->TimerARR = 0;
		M->SearchingForHomeSensor = FALSE;
		M->HasBeenHomed_BlockedMsgSent = FALSE;
		M->unprimePulseToGo = 0;
		M->residualUnprimeTime = 0;

		M->startVelocity = 0.0f;
		M->cruiseVelocity = 0.0f;
		M->cruiseVelocitySquared = 0.0f;
		M->endVelocity = 0.0f;
		M->currentVelocity = 0.0f;

		M->AccelerationPulses = 0;
		M->DecelerationPulses = 0;
		M->AccelerationScaled = 0.0f;
		M->AccelerationScaledTimes2 = 0.0f;
		M->MotionRateInSPP = 0.0f;
		M->unitVector = 0.0f;
		M->scaleDegreesToMm = 0.0f;

		M->Dominant2AxisPulseRatio = 0.0f;
		M->AxisFirstPulseOffset = 0.0f;
		M->PulsePending = FALSE;
		M->SubstituteAxisArgWithArgE = FALSE;
		M->SendStepViaCanbus = FALSE;
		//NUKE M->DeviceForCanStep = 0;

		if ((M->latheMode == LATHE_MODE_OFF) || (EMO.Enabled && (EMO.State == SENSOR_CLOSED)))
		{
			M->latheTargetSpeedPPS = 0.0f;
			M->latheCurrSpeedPPS = 0.0f;
			M->latheMode = LATHE_MODE_OFF;
		}
		else
		{
			M->latheTargetSpeedPPS = 0.0f;
		}
	}
	_Tim7StillCalculating = FALSE;
	_LastReportedCurrentRequestedFeedrateInMmPerSec = -1.0f;
	_LastReportedExecutingLineNumber = -1;
	_Dominant.TimerARR = 0;
	_Dominant.TimerPSC = 0;

	_ABjogging.enabled = FALSE;
	joggingDeInit();
	_FlowRateOverridePct = 1.0f;
	_FeedRateOverridePct = 1.0f;
	_MotionRateOverridePct = 1.0f;
	_CompositeFlowOverridePct = 1.0f;
	_CompositeMotionOverridePct = 1.0f;
	_extrusionControl = IGNORE_E_VALUES;
	_saveExtrusionControl = INVALID_E_CONTROL;  // key off this to know whether we're saving state to be restored (hokey, yes);
	_unprimePrimeControl = AUTO_UNPRIME_PRIME;
	E_Q_LastRequestedPositionInUnits = 0.0f;
	E_Q_POSITION = 0;
	_LastExtrusionRate = INVALID_ARG_VALUE; // force a resend of the rate
	_IncrementalMove = FALSE;
	_IncrementalEMove = FALSE;
	ForceReportXYZLocation = FALSE;
	AutoReportXYZLocation = FALSE;
	AutoReportFeedRate = FALSE;
	AutoReportLineNum = FALSE;
	StatusReportXYZLocation=FALSE;
	StatusReportVelocity=FALSE;
	StatusReportFlowRate=FALSE;
	StatusReportLineNum=FALSE;
	//NUKE _gs._preCannedLaserPending = 0;
	_gs._preCannedStepPending = 0;

	removeAllRegisteredDevice();
	sendSetSwResetBit(0);//reset all devices as well (which in turn turns off heaters and motors)
}

void setDrillTapMotorArgs(float targetZ)
{
	setAllMotorArg(INVALID_ARG_VALUE);  // only moving 1 or 2 axes
	setMotorArg(&Motors[CannedLinearAxis], targetZ);
	if (CannedCycleFlag == 84)
	{   // convert delta distance for LinAxis to an absolute position for the rotary/tapping axis knowing it started at "0" when linAxis was at the "I" plane
		float zDelta = CannedZClearInMM - targetZ;  // delta distance from clearance plane to next depth
		if (CannedThreadPitch != 0.0f)
		{
			setMotorArg(&Motors[CannedRotaryAxis], zDelta / CannedThreadPitch);
		}
		else
		{
			setMotorArg(&Motors[CannedRotaryAxis], zDelta);
		}

	}
}

void puff(void)
{
	updateHssPwm(CannedHssPtr);  // calc terminal count, clear counter, from prior setup
}

void processCannedCycle(void)
{
	// DO NOT USE "return" statements in the switch statement of this routine as ALL canned cycles
	// upon completion need to reset the ExecuteImmediateFlag when finishing the command.  This is
	// performed after the switch statement

	if (motionQ_full()) return;//no room for more moves, so come back later

	ExecutionPtr = CannedCyclePtr;
	if (CannedCycleFlag) { //come here to process a canned cycle
		CannedCycleLastMove = FALSE; // set to true for the last "move" command in the canned cycle (for proper linking of deferred commands
		switch (CannedCycleFlag) // select the type of calculation
		{
		case 28: // G28 Homing
			ProcessG28MultipassHoming();
			break;
		case 702: // cylinder print
			ProcessG702CylinderPrint();
			break;
#ifdef NEW_G38
		case 38: // G38 multipass probing
			ProcessG38MultipassProbing();
			break;
#endif
			//none of the next canned cycles are valid so they fall thru to turn off the canned cycle flag
		case 70: //G70 Bolt Hole Circle
		case 71: //G71 Bolt Hole Arc

		case 74: //G74 Reverse Tap Canned Cycle
		case 76: //G76 Fine Boring Canned Cycle
		case 77: //G77 Back Bore Canned Cycle
		case 80: CannedCycleFlag = 0;break;//G80 Canned Cycle Cancel  --- SHOULD NEVER GET HERE
		//drilling cycles
		case 73: //G73 High-Speed Peck Drilling Canned Cycle
		case 81: //G81 Drill Canned Cycle, simple drill to depth and then retract
		case 82: //G81 Drill Canned Cycle, simple drill to depth and then retract
		case 83: //G83 deep hole Canned Cycle
		{
			switch (CannedCycleStep) /* select the type of calculation */
			{
			case 4: //send Z to Drill Hole value
				if(CannedZDesiredDepth==CannedZDepthInMM)
				{
					ExecuteSingleAxisMove(&Motors[M_Z], CannedZClearInMM, RAPID_MOTION_RATE);
					CannedCycleStep = 0;//no more sequences, we are finished
					CannedCycleFlag=0;//no more canned cycle for you buddy
					CannedCycleLastMove = TRUE; // we MUST mark the last move command in the series otherwise deferred commands will not work
					break;
				}
				if(CannedZQIncrement)
				{
					ExecuteSingleAxisMove(&Motors[M_Z], CannedZDesiredDepth + CannedZQIncrement, RAPID_MOTION_RATE);
					CannedCycleStep = 2;//skip over the full retract
					break;
				}
				CannedCycleStep = 3;
				//now adjust for next hole to drill
				break;
			case 3: //send Z to Clear Hole value
				CannedCycleStep = 2;
				ExecuteSingleAxisMove(&Motors[M_Z], CannedZClearInMM, RAPID_MOTION_RATE);
				break;
			case 2:
				//at this point we have drilled down and then done a rapid back up
				//we need to do a rapid back to the last desired depth and then go back down slowly again
				if(CannedZDesiredDepth==CannedZDepthInMM)
				{//we are done here, we just retracted from the very bottom of the hole
					CannedCycleStep = 0;//no more sequences, we are finished
					CannedCycleFlag=0;//no more canned cycle for you buddy
					CannedCycleLastMove = TRUE; // we MUST mark the last move command in the series otherwise deferred commands will not work
					break;
				}
				CannedCycleStep = 1;
				ExecuteSingleAxisMove(&Motors[M_Z], CannedZDesiredDepth, RAPID_MOTION_RATE);//CannedCycleZFeedRateInMmPerSec);
				break; //send Z to Drill Hole value
			case 1:
				//so we are NOT at depth and we need to do one more drill cycle,
				CannedZDesiredDepth += CannedZIncrement;
				if(CannedZIncrement>0)
				{//positive movement on the z pecking direction
					if(CannedZDesiredDepth>CannedZDepthInMM)
					{CannedZDesiredDepth=CannedZDepthInMM;}
				}
				else
				{//negative z movement on the z pecking direction
					if(CannedZDesiredDepth<CannedZDepthInMM)
					{CannedZDesiredDepth=CannedZDepthInMM;}
				}
				//now drill the final hole peck please
				ExecuteSingleAxisMove(&Motors[M_Z], CannedZDesiredDepth, CannedCycleZFeedRateInMmPerSec);//CannedCycleZFeedRateInMmPerSec);
				CannedCycleStep = 4;
				break;//let it go to step 4 that will start the next drill cycle do depth
			default:
				break;
			}
			break;
		}
		//next is the tapping cycles
		case 84: //G84 Tapping Canned Cycle
		{
			switch (CannedCycleStep) /* select the type of calculation */
			{
			case 5: // start cycle (go to clearance plane, reset rotary axis for sync
				motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS); // move to initial XYZABC position
				CannedZDesiredDepth = CannedZClearInMM; // start cycle at clearance plane
				ExecuteSingleAxisMove(&Motors[CannedLinearAxis], CannedZClearInMM, RAPID_MOTION_RATE); // rapid move to clearance Z height
				if (CannedCycleFlag == 84)
				{ // if tapping, reset rotary axis
					SetAxisHomePosition(&Motors[CannedRotaryAxis]);
				}
				CannedCyclePuffFlag = TRUE; // will force a call to puff() when the next move executes
				CannedCycleStep = 1;
				break;
			case 4: //send Z to Drill/Tap Hole -- drilling/tapping fresh material - "SLOW" move
				setDrillTapMotorArgs(CannedZDesiredDepth);
				motionQ_addCommand(CannedCycleZFeedRateInMmPerSec, NOT_HOMING_MOTORS);
				CannedCyclePuffFlag = FALSE;
				CannedCycleStep = 3;
				break;
			case 3: //send Z to Clear Hole value - RAPID move
				setDrillTapMotorArgs(CannedZClearInMM);
				motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
				CannedCycleStep = 2;
				break;
			case 2: //send Z to bottom of previously drilled/tapped amount - RAPID move
				setDrillTapMotorArgs(CannedZDesiredDepth);
				CannedCyclePuffFlag = TRUE; // will force a call to puff() when the next move executes
				motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
				CannedCyclePuffFlag = FALSE;
				CannedCycleStep = 1;
				break;
			case 1:  // do final move OR do next increment
				if (CannedZDesiredDepth == CannedZDepthInMM)
				{   //we are at depth so cancel and leave
					CannedCycleLastMove = TRUE; // we MUST mark the last move command in the series otherwise deferred commands will not work
					setDrillTapMotorArgs(CannedZClearInMM);
					motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
					CannedCycleFlag = 0; //no more canned cycle for now
					CannedCycleStep = 0;
					//CannedHssPtr = NULL;  // disable "puff" for next command -- commented out to leave persistant
				}
				else
				{
					CannedZDesiredDepth += CannedZIncrement;
					if (CannedZClearInMM >= CannedZClearInMM)
					{
						if (CannedZDesiredDepth < CannedZDepthInMM)
						{
							CannedZDesiredDepth = CannedZDepthInMM;
						}
					}
					else
					{
						if (CannedZDesiredDepth > CannedZDepthInMM)
						{
							CannedZDesiredDepth = CannedZDepthInMM;
						}
					}
					CannedCycleStep = 4;    // start sequence again
				}
				break;
			default:
				break;
			}
			break;
		}
		case 85: // G85 Boring Canned Cycle,really already done, check on the G12/13 code base
			CannedCycleFlag = 0;
			break;
		default:
			break;
		}
	}
	if ((CannedCycleFlag == 0) && (ExecuteImmediateFlag == 2))
	{
		ExecuteImmediateFlag = 0; // reset flag if we got here via an immediate move
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean deferredCmdQueueFull(void)
{
	return(DeferredCommandsInQue >= SIZE_OF_DEFERRED_COMMAND_QUEUE);
}

////////////////////////////////////////////////////////////////////////////////

void copyCmdQueEntryToDeferredCmdQue(GMCommandStructure *cmdPtr)
{
	DeferredCommandsInQue++;
#ifdef COLLECT_METRICS
	_metrics.total_DeferredProcessed++;
	if (DeferredCommandsInQue > _metrics.max_DeferredCommandsInQue)
		_metrics.max_DeferredCommandsInQue = DeferredCommandsInQue;
#endif
	deferredCmdQue[NextDeferredCommandInsertionIndex].cmd = *cmdPtr;  // copy command to deferred queue
	deferredCmdQue[NextDeferredCommandInsertionIndex].cmdState = DEFERRED_CMD_VALID_BUT_NOT_READY_TO_PROCESS;
	NextDeferredCommandInsertionIndex = getNextDeferredCommandIndex(NextDeferredCommandInsertionIndex);
}

////////////////////////////////////////////////////////////////////////////////

void processNextDeferredCommand(void)
{
	if (DeferredCommandsInQue == 0)
	{
		barf("processNextDeferredCommand: lost a deferred command");
	}
	else if (deferredCmdQue[NextDeferredCommandIndex].cmdState != DEFERRED_CMD_VALID_AND_READY_TO_PROCESS)
	{
		barf("processNextDeferredCommand: links/readyToProcess mismatch");
	}
	else
	{
		GMCommandStructure *saveExecutionPtr = ExecutionPtr; // just in case, let's save and restore.
		processCommand(&deferredCmdQue[NextDeferredCommandIndex].cmd);
		ExecutionPtr = saveExecutionPtr;   // restore
		uint32_t irq_disabled = interruptsOff(); // protect against processMotion touching deferred que from interfereing and vice versa
		deferredCmdQue[NextDeferredCommandInsertionIndex].cmdState = DEFERRED_CMD_INVALID;
		DeferredCommandsInQue--;
		NextDeferredCommandIndex = getNextDeferredCommandIndex(NextDeferredCommandIndex);
		if ((DeferredCommandsInQue == 0) || (deferredCmdQue[NextDeferredCommandIndex].cmdState != DEFERRED_CMD_VALID_AND_READY_TO_PROCESS))
		{
			_needToProcessDeferredCommands = FALSE;
		}
		interruptsOn(irq_disabled);
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkForCompletedAbort(void)
{
	if (_abortFinisedNeedToResetProcessSynchronously)
	{   // need to reset all comm/motion pointers after an abort but not in the middle of any slices running or self-init
		ResetProcess(1);
		_heartbeatRateControl = HEARTBEAT_MODE_NORMAL;  // return to normal
#ifdef GB_ABORT_PIN
		pinClear(GB_ABORT_PIN); // signal to logic analyzer
#endif
		if (EMO.State == SENSOR_TRIPPED)
		{	// need to disable and can axis motors as they may be on a diff power supply
			DisableAllMotionMotors();
		}
		if (_abortOccurredWhileMoving)
		{
#ifdef GB_HIDDEN_WARNINGS
#warning "maybe move to BEFORE Reset process"
#warning "maybe want to change to abortWhenCmdQueNotEmpty????" //goal is to not do this move ANYTIME a reset is sent in, but only during a print
#endif //GB_HIDDEN_WARNINGS
			_abortOccurredWhileMoving = FALSE;
			MotorStructure *M;

			ExecutionPtr = &cmdQue[0]; // ensure pointing to valid memory after the abort

			//process the abort move set up by M627
			InvalidateAllCmdArgs(ExecutionPtr);
			ARG_G = 98765.0f;       // just for traceability if an error is generated downstream
			ARG_N = _gcodeLineNumber;
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
				if (M->AbortRelativePosition != INVALID_ARG_VALUE)
				{
					if ((M->Q_LastRequestedPositionInUnits + M->AbortRelativePosition) < 0.0f)
						setMotorArgInNativeUnits(M, 0.0f - M->Q_LastRequestedPositionInUnits);
					else if ((M->Q_LastRequestedPositionInUnits + M->AbortRelativePosition) > M->MaximumTravelInUnits)
						setMotorArgInNativeUnits(M, M->MaximumTravelInUnits - (M->Q_LastRequestedPositionInUnits));
					else
						setMotorArgInNativeUnits(M, M->AbortRelativePosition);
				}
			}

			_IncrementalMove = TRUE;
			motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);

			InvalidateAllCmdArgs(ExecutionPtr);
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
				setMotorArgInNativeUnits(M, M->AbortAbsolutePosition);
			}
			_IncrementalMove = FALSE;
			motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
#ifdef RESET_ABORT_MOVE_ON_JOB_END //DO WE WANT TO MAKE THIS NON-PERSISTENT
			// job is really finished, so invalidate these (not persistent)
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
				M->AbortRelativePosition = INVALID_ARG_VALUE;
				M->AbortAbsolutePosition = INVALID_ARG_VALUE;
			}
#endif //RESET_ABORT_MOVE_ON_JOB_END
		}
	}
}

void setPauseGcodeFlag(char charToSend)
{
	_gcodePaused = TRUE;
	if ((_abortCameFromRx) && (charToSend == CATASTROPHIC_ERROR_ALERT_CHAR))
	{   // not a self inflicted abort, so don't send CATASTROPHIC_ERROR_ALERT_CHAR
		charToSend = NULL_CHAR;
	}
	if (charToSend != NULL_CHAR)
	{
		if (charToSend == CATASTROPHIC_ERROR_ALERT_CHAR)
		{   // aborting ... dump out any possible error messages back to repetrel before sending the ABORT
			pauseToTransmitBufferToEmpty(); // flush ascii buffer
		}
		sendchar(charToSend);  // tell repetrel the pause has started
	}
}

void ResetProcess(boolean sendAck)
{   //used to kill the process job that is currently running or a general init/reset
	//this is a job abort, flush buffer NOW!!!!

	_gs._errorCount=0; // let errors spew again
	_abortInProgress = 0;
	_abortCameFromRx = 0;
	_abortFinisedNeedToResetProcessSynchronously = FALSE;
	_waitingForGuiCommand = FALSE;
	resetSerialInputBuffer();
	resetSerialOutputBuffer();  //GB XXX this call will ruin crashlog dump on abort

	// flush the canbus fifo
	_gs._canTxQ.numMsg = 0;
	_gs._canTxQ.nextIn = 0;
	_gs._canTxQ.nextOut = 0;
	_gs._canRxQ.numMsg = 0;
	_gs._canRxQ.nextIn = 0;
	_gs._canRxQ.nextOut = 0;

	sprintf(_tmpStr, "Buffer Flushed (%d)", sendAck);
	sendInfo(_tmpStr);
	DeInitLaser();
	initAllHss();
	resetSoapstringControl();
	resetStickyErrorFlags();
	initGCodeSequenceEngine();  // reset the entire command process, sequence engine, and motionQ
	//initKey(TRUE, 0, 0);

#ifdef HYDRA_DIAGS
	_diagsLedDisplayMode = DIAG_LED_OFF;
#endif
	_errors.sent.u32 = 0; // clear all flags marking which errors were sent (on select errors)
	_printAir=FALSE;
	if (sendAck)
	{
		sendRevisionString("Kill/Reset");
		if (EMO.State == SENSOR_CLOSED)
		{   // only report EMO pressed case after a reset
			sendEmoMessage();
		}
	}
	// try to get the GUI in sync
	SendFakeMcodeExecutionNotice(670, INVALID_ARG_VALUE, 100.0f, INVALID_ARG_VALUE);    //M670 to set state of DDL (on)
	SendFakeMcodeExecutionNotice(672, INVALID_ARG_VALUE, (float)DDLightFunction, INVALID_ARG_VALUE);    //M670 to set state of DDL (on)
}

void DwellTimer(void)
{
	if (_g4DwellTimer>0)
	{
		_g4DwellTimer--;
	}
}

void updateHostConnectionWatchDog(void)
{
	if(HostConnectionWatchDog>0)
	{
		HostConnectionWatchDog--;//count down until we hit zero
		if(HostConnectionWatchDog==0)
		{   // shut down extruder motors so they don't spew forever
			StopAllExtruders();
			DisableAllExtruderMotors();
		}
	}
}

void SequenceEngine()
{//this will execute the command stored in the que, if there are any waiting to get executed

	// this must be ahead of any dwell timers to ensure it counts down - SAFETY first!

	if (_needToProcessDeferredCommands)
	{   // path from ProcessMotion to process deferred commands
		processNextDeferredCommand();
		return;
	}

	if (_g4DwellTimer>0) return;

	if ((CommandsInQue == 0) && motionQ_empty())
	{   // only countdown watchdog after all work is complete
		updateHostConnectionWatchDog();
	}

	if (PrimeTimer >  0)
	{   // waiting for prime to complete (prime preamble)
		PrimeTimer--;
		if(PrimeTimer == 0)
		{
#ifdef GB_PRIME_PIN
	pinClear(GB_PRIME_PIN);
#endif
			motionQ_primeComplete();
			if (motionQ_notEmpty() && !_gcodePaused)
			{   // got here from a move in the motionQ
				//motionQ_executeMove();
				StartMove();
			}
		}
		return;
	}

	if (UnPrimeTimer > 0)
	{   // waiting for unprime to complete (unprime preamble)
		UnPrimeTimer--;
		if(UnPrimeTimer == 0)
		{
#ifdef GB_RESIDUAL_PIN
			pinClear(GB_RESIDUAL_PIN);
#endif
#ifdef GB_UNPRIME_PIN
	pinClear(GB_UNPRIME_PIN);
#endif
			if (motionQ_notEmpty())
			{   // got here from the motionQ -- should be only way to get here
				motionQ_unprimeComplete();
				motionQ_deleteCurrentMoveAndExecuteNextMove();
			}
		}
		return;
	}

	if (_gcodePaused) return;

	if (motionQ_getCountdownDelayToExecuteMove())
	{
		if ((motionQ_numEntries() >= 2) && (motionQ_getCountdownDelayToExecuteMove() < MOTIONQ_FIRST_ENTRY_AGE_TIME_MS))
		{   // have a second move in place.... so shorten the wait ONLY is the wait is less than the first entry AGE time (so don't over shorten the MOTOR_EN delay)
			motionQ_setCountdownDelayToExecuteMove(1);
		}
		motionQ_decrCountdownDelayToExecuteMove();
		if (motionQ_getCountdownDelayToExecuteMove() == 0)
		{   // finished letting first element in the motionQ "age" to allow other commands to enter the queue
			motionQ_executeMove();
			return;
		}
	}

	if ((G2G3Flag > 0) || G203State)
	{   // working on a multi-cycle move
		return;
	}
	if (_arc.index > 0)
	{
		return;
	}

	if (_abortInProgress)   // this check must be ahead of any moves;
		return;

	if (CannedCycleFlag)
	{   //come here to process a canned cycle
		//ok if you get here, the motion is complete and we are just going to process the next step
		processCannedCycle();
		return;
	}

	if (ExecuteImmediateFlag==2)
	{//execute out of order when this flag is set to 2
		if (motionQ_full()) return;  // can't always process command, so make sure there's room
		if ((cmdQue[CurrentCommandIndex].G == 928.0f) && (!motionQ_empty())) return; // prevent multiple 928's from stepping on each other (NUKE when GUI no longer sends 911 in front of G928's)
		CurrentCommandIndex=1;//point to cmdQue[1] for the prams to execute
		processCommand(&cmdQue[CurrentCommandIndex]);
		if (CannedCycleFlag == 0)
			ExecuteImmediateFlag=0;//turn off the flag (will be shut off after multipass homing, etc
	}
	else
	{
		if (CommandsInQue == 0)
		{
			return;             //dont do anything if we are not ready to process it
		}

#ifdef ADD_ON_SPI_DISPLAY
		if ((GUI_CommandQueueFull() || _waitingForGuiCommand))
		{
			return;
		}
#endif //ADD_ON_SPI_DISPLAY

		if (_MailBoxes._waitingFor.flags.u32)
		{   //waiting for something, so can't proceed
			return;
		}

		//add more checks for temperature wait and dwell here....RPT  XXX

		// if flow gets here, the last command is complete, so remove it from the cmdQue and execute the next command
		// with the motionQ, program flow needs to alter to keep the cmdQue in sync with the motion queue.
		// ie if G1/G2/G3 occurred but the motionQ was full, need to wait until the mQ frees up a slot.
		// if a non G1/G2/G3 occurs and the motionQ is not empty, need to wait for it to empty before proceeding _motionQ_ForceQtoEmpty will be used
		// G2/G3 will decompose into many small moves which will be stuffed into the motionQ

		NextExecutionPtr = &cmdQue[NextCommandIndex];
		if (motionQ_full())
			return;     // can't do anything

		if (_motionQ_ForceQtoEmpty)
		{   // in single step mode
			if (motionQ_empty())
				_motionQ_ForceQtoEmpty = FALSE;
			else  // if (motionQ_notEmpty())
				return;
		}
		if (_requestToPauseAtEndOfLayer && ((int)NextExecutionPtr->M == 790))
		{   // special case of M790 where we need to stop - so do this in single step mode to not
			// process past the pause in motion
			NextExecutionPtr->cmdType = SINGLE_STEP;
		}

		if (motionQ_empty())
		{  // can do anything
			if ((NextExecutionPtr->cmdType == SINGLE_STEP) || (NextExecutionPtr->cmdType == UNDEFINED))
				_motionQ_ForceQtoEmpty = TRUE;
			; // now fall through and process G/M code
		}
		else if (NextExecutionPtr->cmdType == ADD_TO_MQ)
		{   // let this fall through to process the G1/G2/G3 since there is room in the queue and not forcing queue to drain
			// command will be deleted from cmdQue after move finishes.
			; // now fall through and process G/M code
		}
		else if (_jogging.enabled)
		{
			return; // don't processes other commands until the jog is complete (no mechanism to defer commands when jogging
		}
		else if (NextExecutionPtr->cmdType == IMPACTS_DESTINATION_CALC)
		{   // can still process these in order as only impact dest calc which happens immediately on adding to motionQ
			; // now fall through and process G/M code
		}
		else if (NextExecutionPtr->cmdType == SYNCS_WITH_MOTION)
		{   // TAG is as part of newest (most recently added) mQ entry and then move to deferred fifo
			// for later retrieval (when "newest" move completes, this will be executed .. in sync with motion)
			// entry is still be active in cmdQue
			if (deferredCmdQueueFull())
				return;  // need to wait for things to empty out
			NextExecutionPtr->cmdLink = LastCmdQueIndexAddedToMotionQ;  // create connection to "newest" mQ entry
			motionQ.newest->flags.hasDeferredCommands = TRUE;
			copyCmdQueEntryToDeferredCmdQue(NextExecutionPtr);
		}
		else if ((NextExecutionPtr->cmdType == SINGLE_STEP) || (NextExecutionPtr->cmdType == UNDEFINED))
		{
			return;  // need to wait for the motionQ to empty
		}
		else
		{
			barf("SeqEng(): should never get here");
			return;
		}

		// to get here, either (the motionQ is empty) OR (the motionQ is not full AND not starting a G1/G2/G3/etc to the queue)
		// everything else needs to single step.

		CommandsInQue--;                            //count down the number of commands
		CurrentCommandIndex=NextCommandIndex;   //set the working pointer to the right spot in the execution que
		NextCommandIndex = getNextCommandIndex(NextCommandIndex);
		if ((motionQ_notEmpty()) && (NextExecutionPtr->cmdType == SYNCS_WITH_MOTION))
			return;  // this type of command will be processed later
		// if we get here, we can process the next command
		processCommand(&cmdQue[CurrentCommandIndex]);
	}
}


void processCommand(GMCommandStructure *cmdPtr)
{
	ExecutionPtr = cmdPtr;
	if (ARG_G_PRESENT)
	{
#ifdef GB_TX_LINE_Q_GM
		sprintf(_tmpStr, "L%d:G%d:%d", _gcodeLineNumber, (int)ARG_G, motionQ_numEntries());
		sendGB(_tmpStr);
#endif
		_gcodeMotionLineNumber = (int)ARG_N;
		switch ((int)ARG_G)                  /* select the type of calculation */
		{
		case 0:   G_Code_G0();    break;//return;//rapid move
		case 1:   G_Code_G1();    break;//return;//vector move
		case 2:   G_Code_G2();    break;//return;//move arc cw
		case 3:   G_Code_G3();    break;//return;//move arc ccw
		case 4:   G_Code_G4();    break;//return;//Dwell in ms
		case 12:  G_Code_G12();   break;//return;//circular pocket mill CW
		case 13:  G_Code_G13();   break;//return;//circular pocket mill CCW
		case 16:  G_Code_G16();   break;//return;//set arc plane to NONE (user defineable)
		case 17:  G_Code_G17();   break;//return;//set arc plane to XY
		case 18:  G_Code_G18();   break;//return;//set arc plane to XZ
		case 19:  G_Code_G19();   break;//return;//set arc plane to YZ
		case 20:  G_Code_G20();   break;//return;//inch dimensions
		case 21:  G_Code_G21();   break;//return;//mm dimension
		case 28:  G_Code_G28();   break;//return;//home axis, responds to individual axis if args are passed, X0Y0 homes only x and y not Z
		case 29:  G_Code_G29();   break;//return;//special exercise move
		case 38:  G_Code_G38();   break;//return;//touch probe
		case 53:  G_Code_G53();   break;//return;//set fixture offset off
		case 54:  G_Code_G54();   break;//return;//set fixture offset on
		case 55:  G_Code_G55();   break;//return;//set fixture offset on
		case 56:  G_Code_G56();   break;//return;//set fixture offset on
		case 57:  G_Code_G57();   break;//return;//set fixture offset on
		case 58:  G_Code_G58();   break;//return;//set fixture offset on
		case 59:  G_Code_G59();   break;//return;//set fixture offset on
		case 70:  G_Code_G70();   break;//return;//G70 Bolt Hole Circle
		case 71:  G_Code_G71();   break;//return;//G71 Bolt Hole Arc
		case 73:  G_Code_G73();   break;//return;//G73 High-Speed Peck Drilling Canned Cycle
		case 74:  G_Code_G74();   break;//return;//G74 Reverse Tap Canned Cycle
		case 76:  G_Code_G76();   break;//return;//G76 Fine Boring Canned Cycle
		case 77:  G_Code_G77();   break;//return;//G77 Back Bore Canned Cycle
		case 80:  G_Code_G80();   break;//return;//G80 Canned Cycle Cancel
		case 81:  G_Code_G81();   break;//return;//G81 Drill Canned Cycle
		case 82:  G_Code_G82();   break;//return;//G82 Spot Drill Canned Cycle
		case 83:  G_Code_G83();   break;//return;//G83 Normal Peck Drilling Canned Cycle
		case 84:  G_Code_G84();   break;//return;//G84 Tapping Canned Cycle
		case 85:  G_Code_G85();   break;//return;// G85 Boring Canned Cycle
		case 90:  G_Code_G90();   break;//return;//absolute programming coordinates
		case 91:  G_Code_G91();   break;//return;//incremental programming
		case 92:  G_Code_G92();   break;//return;//Turn on the G92 offset
		case 93:  G_Code_G93();   break;//return;//turn off the G92 offsets
		case 214: G_Code_G214();  break;//return;//rectangular pocket mill CW
		case 215: G_Code_G215();  break;//return;//rectangular Pocket Mill CCw
		case 702: G_Code_G702(FALSE);  break;//return;//cylinder print - CW
		case 703: G_Code_G702(TRUE);  break;//return;//cylinder print - CCW
		case 928: G_Code_G928();  break;//return;//used to override the home sensor and walk the unit back by xxx.xxx mm
		default:
			sprintf(_errorStr, "processCommand: Unsupported GCode G%d (~L:%d)", (int)ARG_G, (int)ARG_N);
			sendError(_errorStr);
			break;//return;
		}
	}
	else if (ARG_M_PRESENT)
	{//if the M code 0 or more then try to jump to a routine
#ifdef GB_TX_LINE_Q_GM
		sprintf(_tmpStr, "L%d:M%d:%d", _gcodeLineNumber, (int)ARG_M, motionQ_numEntries());
		sendGB(_tmpStr);
#endif
		switch ((int)ARG_M)                  /* select the type of calculation */
		{
		case 0:   M_Code_M0();    break;//return;//program pause
		case 1:   M_Code_M1();    break;//return;//program pause
		case 2:   M_Code_M2();    break;//return;//program END
		case 3:   M_Code_M3();    break;//return;//Spindle ON CW
		case 4:   M_Code_M4();    break;//return;//spindle on CCW
		case 5:   M_Code_M5();    break;//return;//spindle OFF
		case 6:   M_Code_M6();    break;//return;//ToolChange
		case 7:   M_Code_M7();    break;//return;//coolant1 /  heat control on
		case 8:   M_Code_M8();    break;//return;//coolant2 / heat conrol on
		case 9:   M_Code_M9();    break;//return;//all coolant/ heat off
		case 17:  M_Code_M17();     break;//return;//enable the holding torque on stepping motors
		case 18:  M_Code_M18();     break;//return;//releases the holding torque on stepping motors
		case 30:  M_Code_M30();     break;//return;//notify start and end of program
		case 41:  M_Code_M41();     break;//return;//set spindle to LOW speed range
		case 42:  M_Code_M42();     break;//return;//set spindle to HIGH speed range
		case 44:  M_Code_M44();     break;//return;//set spindle Motor coolant on
		case 45:  M_Code_M45();     break;//return;//set spindle Motorcoolant Off
		case 73:  M_Code_M73();     break;//return;// passthru of remaining printing time (uses P, R)
		case 82:  M_Code_M82();     break;//return;// set extruder to absolute mode
		case 83:  M_Code_M83();     break;//return;// set extruder to relative mode
		case 84:  M_Code_M84();     break;//return;//releases the holding torque on stepping motors
		case 91:  M_Code_M91();     break;//return;//set axis Max travel in MM, from absolute zero.
		case 92:  M_Code_M92();     break;//return;//set axis steps per unit,  pulses per millimeter
		case 93:  M_Code_M93();     break;//return;//set home sensor polarity
		case 94:  M_Code_M94();     break;//return;//set direction polarity, 0 is normal 1 is reverse
		case 95:  M_Code_M95();     break;//return;//sets stall sensor polarity (uses X, Y, Z, A, B, C)
		case 96:  M_Code_M96();     break;//return;//sets the enable bit polarity (uses X, Y, Z, A, B, C)
		case 97:  M_Code_M97();     break;//return;//sets the step bit polarity (uses X, Y, Z, A, B, C)
		case 98:  M_Code_M98();     break;//return;//sets limit1 sensor polarity (uses X, Y, Z, A, B, C)
		case 99:  M_Code_M99();     break;//return;//sets limit2 sensor polarity (uses X, Y, Z, A, B, C)
		case 101: M_Code_M101();    break;//return;//turn extruder on
		case 102: M_Code_M102();    break;//return;//turn extruder on, reverse
		case 103: M_Code_M103();  break;//return;//turn extruder off
		case 104: M_Code_M104();  break;//return;//set desired temperature to Sxxx.x
		case 105: M_Code_M105();  break;//return;//set desired temperature to Sxxx.x immediate
		case 106: M_Code_M106();  break;//return;//turn fan on
		case 107: M_Code_M107();  break;//return;//turn fan off
		case 108: M_Code_M108();  break;//return;//set extruder speed to Sxx.x  rpm
		case 109: M_Code_M109();  break;//return;//wait for extruder to reach temp
		// case 114: M_Code_M114();  break;//return;//report xyz location immediately
		// case 115: M_Code_M115(); break;//return;//tell them what firmware revision level we are using
		case 140: M_Code_M140();  break;//return;//Set bed temperature
		case 141: M_Code_M141();  break;//return;//set chamber temperature (uses S)
		case 190: M_Code_M190();  break;//return;//wait for hotbed to reach temp (uses S)
		case 191: M_Code_M191();  break;//return;//wait for chamber to reach temp (uses S)
		case 203: M_Code_M203();  break;//return;//set rapid feed rate/ and acceleration constant for smooth ramps
		case 204: M_Code_M204();  break;//return;//set no ramp speed, no acceleration below this point
		case 205: M_Code_M205();  break;//return;//set home speed, when homing to zero
		case 206: M_Code_M206();  break;//return;// Sets the homing routine hysteresis location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 208: M_Code_M208();  break;//return;// Sets the acceleration constant (uses X, Y, Z, A, B, C)
		case 209: M_Code_M209();  break;//return;// Sets the minimum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 210: M_Code_M210();  break;//return;// Sets the maximum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 211: M_Code_M211();  break;//return;// Sets the machine minimum pulse rate limit (uses S)
		case 212: M_Code_M212();  break;//return;// Sets the machine maximum pulse rate limit (uses S)
		case 213: M_Code_M213();  break;//return;// sets the per motor/axis installation status (uses X,Y,Z,A,B,C)
		case 214: M_Code_M214();  break;//return;// sets the per motor/axis type (linear/rotary) (uses X,Y,Z,A,B,C)
		case 215: M_Code_M215();  break;//return;// Sets the homing routine start location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 216: M_Code_M216();  break;//return;// Sets the homing routine end location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 217: M_Code_M217();  break;//return;// sets the max deceleration rate (for abort) (mm/sec/sec)  (uses X, Y, Z, A, B, C)
		case 218: M_Code_M218();  break;//return;// set the fixture offsets  (uses O, X, Y, Z, A, B, C)
		//not added to cmdQuecase 219: M_Code_M219();  break;//return;// sets the serial baud rate (uses B)
		case 220: M_Code_M220();  break;//return;// sets the per motor/axis send motorStep pulse as a CANbus command (uses T,X,Y,Z,A,B,C)
		case 221: M_Code_M221();  break;//return;//set extrude factor override percentage
		case 222: M_Code_M222();  break;//return;// set the centripetal accelleration radius (uses S)
		case 223: M_Code_M223();  break;//return;// Sets the re-homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 224: M_Code_M224();  break;//return;// Sets the jog no ramp speed (UNITS/MIN) (no acceleration needed below this) (uses X, Y, Z, A, B, C)
		case 225: M_Code_M225();  break;//return;// Sets the jog speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 226: M_Code_M226();  break;//return;// Sets the jog acceleration constant (uses X, Y, Z, A, B, C, P)
		case 227: M_Code_M227();  break;//return;// control jogging using the AB encoder on the panel interface (uses XYZABCR>
		case 228: M_Code_M228();  break;//return;//Disable Extruder Reverse
		case 229: M_Code_M229();  break;//return;// extrusion control (uses E, P, S)
		case 230: M_Code_M230();  break;//return;// set global flow rate override percentage (uses S)
		case 231: M_Code_M231();  break;//return;// set motion feedrate override percentage (uses S)
		case 232: M_Code_M232();  break;//return;// set motion override percentage (uses S)
		case 233: M_Code_M233();  break;//return;// set homing pop-off distance (uses X, Y, Z, A, B, C)
		case 234: M_Code_M234();  break;//return;// set motor position (absolute) (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 235: M_Code_M235();  break;//return;// set rotary axis plane and offset (uses S, P, A, Y, Z)
		case 236: M_Code_M236();  break;//return;// write device type and revision to flash (option bytes) (uses T, S, I, P)
		case 237: M_Code_M237();  break;//return;// set cold extrusion prevention parameters (uses T, C, U, L, R)
		case 238: M_Code_M238();  break;//return;// sets the per motor/axis execute E value (uses T,X,Y,Z,A,B,C)

		case 240: M_Code_M240();  break;//return;// turn device switch off (uses T, I)
		case 241: M_Code_M241();  break;//return;// turn device switch on (uses T, I)
		case 242: M_Code_M242();  break;//return;// control device switch by dutyCycle (uses T, I, S)
		case 243: M_Code_M243();  break;//return;// control device switch by pwm (uses T, I, S, P)
		case 244: M_Code_M244();  break;//return;// control device switch by temperature (uses T, I, S)
		case 245: M_Code_M245();  break;//return;// set switch flag(s) (uses T, I, P, E, H, C)
		case 253: M_Code_M253();  break; // turn on lathe motor for continuous CW motion (uses S)
		case 254: M_Code_M254();  break; // turn on lathe motor for continuous CCW motion (uses S)
		case 255: M_Code_M255();  break; // turn off lathe motor (uses S)
		case 260: M_Code_M260();  break; // control display attached to a head (uses S,P,X,Y,I,J,R)
		case 261: M_Code_M261();  break; //osseo Set Recirc Fan Speed
		case 262: M_Code_M262(); break; //osseo Set Uv Led Duty
		case 263: M_Code_M263(); break; //osseo Set door lock PWM
			
		case 600: M_Code_M600();  break;//return;// disable all HSS outputs
		case 601: M_Code_M601();  break;//return;// enable HSS out1
		case 602: M_Code_M602();  break;//return;// enable HSS out2
		case 603: M_Code_M603();  break;//return;// enable HSS out3
		case 604: M_Code_M604();  break;//return;// enable HSS out4
		case 605: M_Code_M605();  break;//return;// enable HSS out5
		case 606: M_Code_M606();  break;//return;// enable HSS out6
		case 607: M_Code_M607();  break;//return;// enable HSS out7
		case 608: M_Code_M608();  break;//return;// enable HSS out8

		case 610: M_Code_M610();  break;//return;// open drain output
		case 611: M_Code_M611();  break;//return;// open drain output
		case 612: M_Code_M612();  break;//return;// open drain output
		case 613: M_Code_M613(); break;//return;// open drain output	
		case 614: M_Code_M614(); break;//return;// open drain output
		case 615: M_Code_M615(); break;//return;// open drain output
		case 616: M_Code_M616(); break;//return;// open drain output
		case 617: M_Code_M617(); break;//return;// open drain output
			
		case 619: M_Code_M619();  break;//return;// sets the function and output pwm of the selected HSS (uses F, I, S, P, J, H)
		case 620: M_Code_M620();  break;//return;// Laser global control (uses T, E, F, C, P)
		case 621: M_Code_M621();  break;//return;// Laser vector mode control (uses P)
		case 622: M_Code_M622();  break;//return;// Laser raster mode control (uses O, S, D, P, I)
		case 623: M_Code_M623();  break;//return;// Laser pulse (one-shot) mode control (uses P, D)
		case 624: M_Code_M624();  break;//return;// setup raster image (uses X, Z, I, J)
		case 625: M_Code_M625();  break;//return;// inkjet vector mode control (uses S, J)
		case 626: M_Code_M626();  break;//return;// build color index table (uses C, U, A, D)
		case 627: M_Code_M627();  break;//return;// set job_kill/abort auto move location (uses AXZABC IJKUVW)
		case 628: M_Code_M628();  break;//return;// arm/disarm digital trigger (uses P, E, R)
		case 629: M_Code_M629();  break;//return;// open log file
		case 630: M_Code_M630();  break;//return;// canbus touch probe control (uses T, S, D, P)
		case 631: M_Code_M631();  break;//return;// PickNPlace data (uses T, H, P, A, B, C, D)
		case 632: M_Code_M632();  break;//return;// PickNPlace control (uses T, S, H, P, V, F, D,)

		case 660: M_Code_M660();  break;//return;//set tool length and diameter
		case 670: M_Code_M670();  break;//return;
		case 673: M_Code_M673();  break;//return;//sets donkey dick to mimick the state of the print /  non print flow
		case 674: M_Code_M674();  break;//return;//sets up turbo mode
		case 675: M_Code_M675();  break;//return;//sets the response light hss
		case 676: M_Code_M676();  break;//return;//sets the chamber fan pwm
		case 677: M_Code_M677();  break;//return;//sets the control panel buzzer pwm
		case 678: M_Code_M678();  break;//return;//set the laser cross-hair pwm
		case 679: M_Code_M679();  break;//return;//set the evacuum pwm
		case 682: M_Code_M682();  break;//return;//issue z axis calibration request, this is so we can do a seek and loop
		case 683: M_Code_M683();  break;//return;//set the headroom for the normal serial rx buffer (uses S)
		case 685: M_Code_M685();  break;//return;//sets the air assist pwm
		case 686: M_Code_M686();  break;//return;//returns the machine info string for machine key creation
		case 687: M_Code_M687();  break;//return;//unlock system with machine specific password
		case 688: M_Code_M688();  break;//return;//RESERVED - DO NOT USE
		//not added to cmdQue .... case 690: M_Code_M690();  break;//return;//add/delete scripts
		case 698: M_Code_M698(); break;//return; // humiture control (uses T, V)
		case 699: M_Code_M699(); break;//return; // hx711 control (uses T, V, S, O, Z)
		case 701: M_Code_M701(); break;//return; // set auto status update rate and page selection (uses S, P, E)
		case 702: M_Code_M702(); break;//return; // select Tx usage in MCODES (uses S)
		case 703: M_Code_M703(); break;//return; // add device to Group (uses S, P)
		case 704: M_Code_M704(); break;//return; // remove device to Group (uses S, P)
		case 705: M_Code_M705(); break;//return; // reset HH (uses P)
		case 706: M_Code_M706(); break;//return; // sync HH (uses P)
		case 707: M_Code_M707(); break;//return; // STOP HH (uses P)
		case 708: M_Code_M708(); break;//return; // pause HH (uses P)
		case 709: M_Code_M709(); break;//return; // resume device (from pause or stop) (uses T)
		case 710: M_Code_M710(); break;//return; // enable/disable RTD1 (uses S, P)
		case 711: M_Code_M711(); break;//return; // enable/disable RTD2 (uses S, P)
		case 712: M_Code_M712(); break;//return; // enable/disable RTD3 (uses S, P)
		case 713: M_Code_M713(); break;//return; // set default P value for missing argP
		case 714: M_Code_M714(); break;//return; // update devicePosition alias table
		case 715: M_Code_M715(); break;//return; // set LED display selection
		case 716: M_Code_M716(); break;//return; // register HH (uses P)
		case 717: M_Code_M717(); break;//return; // force send of outgoing mail
		case 718: M_Code_M718(); break;//return; // used by repetrel to synchronize M719 data logging
		case 719: M_Code_M719(); break;//return; // set reporting rate and data for host traffic (uses S, E)
		case 720: M_Code_M720(); break;//return; // direct MAIN extrusion control (uses P, F, E, S)
		case 721: M_Code_M721(); break;//return; // direct UNPRIME extrusion control (uses P, F, E, S)
		case 722: M_Code_M722(); break;//return; // direct PRIME extrusion control (uses P, F, E, S)
		case 723: M_Code_M723(); break;//return; // direct MANUAL extrusion control (uses P, F, E, S)
		case 724: M_Code_M724(); break;//return; // direct DWELL (no stepping) extrusion control (uses P, F, E, S)
		case 725: M_Code_M725(); break;//return; // set the Karl Factors for controlling the heater switch (uses T, P, E, S)
		case 726: M_Code_M726(); break;//return; // set the Karl Factors for controlling the fan switch (uses T, P, E, S)
		case 727: M_Code_M727(); break;//return; // set LED override values and mask (uses T, S, P)
		case 728: M_Code_M728(); break;//return; // set motor current boost control (uses T, S)
		case 729: M_Code_M729(); break;//return; // set motor microsteps control (uses T, S)
		case 730: M_Code_M730(); break;//return; // set not to exceed temp for motor (uses T, S)
		case 731: M_Code_M731(); break;//return; // set not to exceed temp for heater (uses T, S)
		case 732: M_Code_M732(); break;//return; // set maximum step rate for motor (microsteps/sec) (uses T, F)
		case 733: M_Code_M733(); break;//return; // set maximum allowable RTD temperature delta (uses T, S)
		case 734: M_Code_M734(); break;//return; // set HH error reporting rate for redundant error codes (uses T, F)
		case 735: M_Code_M735(); break;//return; // fill the incoming page data buffer with S (uses S)
		case 736: M_Code_M736(); break;//return; // fill the outgoing page data buffer with S (uses S)
		case 737: M_Code_M737(); break;//return; // erase flash page in selected (physical) device (uses T, I, P)
		case 738: M_Code_M738(); break;//return; // transfer data page from (physical) device to incoming buffer (uses T, I, P, S)
		case 739: M_Code_M739(); break;//return; // transfer data page from incoming to outgoing buffer
		case 740: M_Code_M740(); break;//return; // transfer data page from outgoing buffer to (physical) device (uses T, I, P, S)
		//case 741: M_Code_M741(); break;//return; // transfer incoming page related data from the device to the to host (uses S)
		//case 742: M_Code_M742(); break;//return; // transfer data page in ASCII text from host to outgoing buffer (uses S, P, comment)
		case 743: M_Code_M743(); break;//return; // transfer generally (read only) device info from inbox to host (uses T, S)
		case 744: M_Code_M744(); break;//return; // transfer alias list from device to inbox (uses T)
		case 745: M_Code_M745(); break;//return; // force HH to invert polarity of direction pin (uses P)
		//case 746: M_Code_M746(); break;//return; // start the bootloader for the selected physical device (uses T)
		case 747: M_Code_M747(); break;//return;   // prepare device for download (uses P)
		case 748: M_Code_M748(); break;//return;   // process next line of intel hex format bootloader data (uses P, comment)
		//case 749: M_Code_M749(); break;//return; // exit the device bootloader
		case 750: M_Code_M750(); break;//return;   // unlock flash for erase/write access for the selected physical device (uses T)
		case 751: M_Code_M751(); break;//return;   // lock flash to prevent erase/write access for the selected physical device (uses T)
		//case 752: M_Code_M752(); break;//return; // write hardware type to flash (option bytes) using device bootloader (uses S)
		//case 753: M_Code_M753(); break;//return; // write hardware revision to flash (option bytes) using device bootloader (uses  S)
		//case 754: M_Code_M754(); break;//return; // write hardware key to flash (option bytes) using device bootloader (uses S)
		case 755: M_Code_M755(); break;//return;   // set extruder heater pwm (uses T, S)
		case 756: M_Code_M756(); break;//return;   // set layer height (mm) (uses S)
		case 757: M_Code_M757(); break;//return;   // set layer/path weight (mm) (uses S)
		case 758: M_Code_M758(); break;//return;   // set extrusion step to volume conversion (steps per nL) (uses T, S)
		case 759: M_Code_M759(); break;//return;   // enable temperature calibration (uses T, S)
		case 760: M_Code_M760(); break;//return; // disable temperature calibration
		//case 761: M_Code_M761(); break;//return; // transfer system info in ASCII text from main board to host (uses S, P)
		//case 762: M_Code_M762(); break;//return; // transfer system info in ASCII text from host to main board (uses S, P, comment)
		case 763: M_Code_M763(); break;//return;   // clear error on selected device(s) (uses T)
		case 766: M_Code_M766(); break;//return;   // start the system bootloader process
		//case 767: M_Code_M767(); break;//return; // prepare system  for download (uses P)
		//case 768: M_Code_M768(); break;//return; // process next line of intel hex format of system bootloader data (uses P, comment) <-- can't go in queue because of needing comment
		//case 769: M_Code_M769(); break;//return; // exit the system bootloader
		//case 770: M_Code_M770(); break;//return; // leave system bootloader and jump to application main()
		case 771: M_Code_M771(); break;//return; // load laser image data controls (scale, offset, max)
		case 772: M_Code_M772(); break;//return;  // reset metrics for new job
		case 773: M_Code_M773(); break;//return;  // send motion metrics to host
		case 774: M_Code_M774(); break;//return;  // send queue metrics to host
		case 775: M_Code_M775(); break;//return;  // send current status/queue values to host
		case 776: M_Code_M776(); break;//return;  // send cmd/motionQ usage histograms to host
		case 777: M_Code_M777(); break;//return;  // send and/or erase the crash log (uses S, E, D)
		case 778: M_Code_M778(); break;//return;  // enable slice time measurement (uses I, S)
		case 779: M_Code_M779(); break;//return;  // dump slice time measurements
		case 780: M_Code_M780(); break;//return;  // enable/disable auto XYZABC position reporting
		case 781: M_Code_M781(); break;//return;  // write hardware type to flash (option bytes) (uses T, D, O, P)
		case 782: M_Code_M782(); break;//return;  // enable/disable "print air" feature (uses S)
		case 783: M_Code_M783(); break;//return;  // set PersistantUltimusControlHeadAddress
		case 784: M_Code_M784(); break;   // report system info (version numbers of system and all heads)
		case 785: M_Code_M785(); break;//return;  // Set motor parameters (uses T,U,A,R,B,P,C,O,S) (V1)
		case 786: M_Code_M786(); break;//return;  // set closed loop stepper PID control values (uses T, P, I, D)
		case 787: M_Code_M787(); break;//return;  // calibrate can based closed-loop motor (uses T, P, C)
		case 788: M_Code_M788(); break;//return;  // reset can based closed-loop axis motor (uses T, F, D, P)
		case 789: M_Code_M789(); break;//return;  // send sideband step pulses to a canAxisMotor (uses T, S)
		case 790: M_Code_M790(); break;//return;   //layer increment, non move command, just has printer echo when it is ready to make this move
		case 791: M_Code_M791(); break;//return; //take a picture please
		case 792: M_Code_M791(); break;//return; //SEND A TEXT
		case 793: M_Code_M791(); break;//return; //EXECUTE SHELL
		case 794: M_Code_M791(); break;//return; //DUMMY
		case 795: M_Code_M795(); break;//return; /// sets jogvalueValueInUnits (uses S)
		case 796: M_Code_M796(); break;//return;// Sets the jog increment (uses X, Y, Z, A, B, C)
		case 797: M_Code_M797(); break;//return;   // enable/disable debug reporting strings ">GB:" (uses S)
		case 798: M_Code_M798(); break;//return; // dump strings to host (debug MCODE) -- warning, must reset after using this MCODE (uses T)
		case 799: M_Code_M799(); break;//return; // get PLL and Clock status for 407
		case 800: M_Code_M800(); break;//return; // sonicator control
		case 868: M_Code_M868(); break;//return; // Move table up
		case 869: M_Code_M869(); break;//return; // Move table down
		case 960: M_Code_M960(); break;//return; // M_Code_960 ; rectangular pocket mill
		default:
			sprintf(_errorStr, "processCommand: Unsupported MCode M%d (~L:%d)", (int) ExecutionPtr->M, (int)ExecutionPtr->N);
			sendError(_errorStr);
			break;//return;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

int getSliceCnt(void)
{
	return(_gs._sliceCnt);
}

////////////////////////////////////////////////////////////////////////////////

void pingReply (void)
{
	sendRevisionString("PingReply");
	if (EMO.State == SENSOR_CLOSED)
	{   // only report EMO pressed case after a ping
		sendEmoMessage();
	}
	sendSetSwResetBit(0);   // reset all heads in the system;
}

////////////////////////////////////////////////////////////////////////////////

int32_t getCurrentExecutingLineNumber(void)
{
	return((motionQ_empty() ? ARG_N : motionQ.oldest->lineNumber));
}

////////////////////////////////////////////////////////////////////////////////
void PWMCntrl(void)
{
	
	pinSet(TPIC_6595_CLR);//make sure the pin is set
	pinClear(TPIC_6595_RCLK);//release the Rclock
	if(Update595Index>15)
		{
			pinSet(TPIC_6595_RCLK);
			Update595Index = 0;
		}
	if((Update595Index & 0x01)>0)
		{
			pinSet(TPIC_6595_SCLK);//raise the clock
		}
		else
		{
			pinClear(TPIC_6595_SCLK); //drop the clock
			//set the data output and let it settle
			if (McodeDrainState[Update595Index >> 1]>0)
			{	
				pinSet(TPIC_6595_D);		
			}
			else
			{	
				pinClear(TPIC_6595_D);		
			}
		}
	Update595Index++;
	
	TIM4->CCR3 = SpindleDesiredSpeedPWM;
	TIM4->CCR4 = CO2LaserAnalogPwrPWM/2;
}
void ReportXYZLocation(void)
{
//	RPMCounter = TIM3->CNT;
//	TIM3->CNT = 0;
	//if (EnableOsseoVariablesReporting)return;//osseo karlchris
	if (ForceReportXYZLocation == FALSE)
	{   // not forcing an immediate update, so continue with prescaler
		if (_MailBoxes._positionReportingPeriodMs == 0)  return;             // no status data requested, so return
		_MailBoxes._positionReportingPrescaleCnt -= DEFAULT_POSITION_REPORTING_PERIOD_DECR_MS;
		if (_MailBoxes._positionReportingPrescaleCnt >  0) return; // still counting down prescale
		_MailBoxes._positionReportingPrescaleCnt = _MailBoxes._positionReportingPeriodMs;   //reset the interval timer for next time
	}

	if (ForceReportXYZLocation || AutoReportXYZLocation || AutoReportFeedRate || AutoReportLineNum)
	{
		sprintf(_rptStr, ">PO");

		if (ForceReportXYZLocation || AutoReportXYZLocation)
		{
			MotorStructure *M;
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // whiz through all the motors, if the motor is installed, the report
				if (M->MotorInstalled && M->autoReportPosition)
				{
					if ((M->POSITION != M->LastReportedPosition) || ForceReportXYZLocation)
					{
						M->LastReportedPosition = M->POSITION;
						{
							sprintf(_tmpStr, ":%c%4.3f", M->AxisLabel, M->POSITION * M->UnitsPerPulse);
						}
						strcat(_rptStr, _tmpStr);
					}
				}
			}
			if ((AutoReportFeedRate && (_CurrentRequestedFeedrateInMmPerSec != _LastReportedCurrentRequestedFeedrateInMmPerSec)) || ForceReportXYZLocation)
			{
				_LastReportedCurrentRequestedFeedrateInMmPerSec = _CurrentRequestedFeedrateInMmPerSec;
				sprintf(_tmpStr, ":%c%4.3f", 'F', _CurrentRequestedFeedrateInMmPerSec);  // last "ARG_F" but passed through to end of motionQ
				strcat(_rptStr, _tmpStr);
			}

			int currentLineNumber = getCurrentExecutingLineNumber();
			if ((AutoReportLineNum && (currentLineNumber != _LastReportedExecutingLineNumber)) || ForceReportXYZLocation)
			{
				_LastReportedExecutingLineNumber = currentLineNumber;
				sprintf(_tmpStr, ":%c%d", 'L', currentLineNumber);  // last "ARG_N" but passed through to end of motionQ
				strcat(_rptStr, _tmpStr);
			}
		}
		sprintf(_tmpStr, ":%c%d", 'S', RPMCounter); // last "ARG_N" but passed through to end of motionQ
		strcat(_rptStr, _tmpStr);
		 
		sprintf(_tmpStr, ":%c%d", 'I', laser_PsOutputCurrent); // last "ARG_N" but passed through to end of motionQ
		strcat(_rptStr, _tmpStr);
		
		sprintf(_tmpStr, ":%c%d", 'V', laser_PsOutputVoltage); // last "ARG_N" but passed through to end of motionQ
		strcat(_rptStr, _tmpStr);
		
		sprintf(_tmpStr, ":%c%d", 'Q', laser_PsControlVoltage); // last "ARG_N" but passed through to end of motionQ
		strcat(_rptStr, _tmpStr);
		
		sprintf(_tmpStr, ":%c%d", 'W', laser_PsWaterProt); // last "ARG_N" but passed through to end of motionQ
		strcat(_rptStr, _tmpStr);
		
		laser_PsOutputCurrent	= 0;
		laser_PsOutputVoltage	= 0;
		laser_PsControlVoltage	=0;
		laser_PsWaterProt		=0;
		
//		sprintf(_tmpStr, ":%c%d", 'T', (int)ADC_Channel[4].convAvg); // last "ARG_F" but passed through to end of motionQ
		strcat(_rptStr, _tmpStr);

		if (strlen(_rptStr) > 3)
		{
			// there was info to send
			sendstringCr(_rptStr);
		}
	}
}
void ReportOsseoVariables(void)
{		
	RPMCounter =(int)30*  TIM3->CNT;
	TIM3->CNT = 0;
	if (EnableOsseoVariablesReporting == 0)return;
	//float pressure difference = AdcChannelTable[ch].Channel.RawADCDataBuffer;
//	ParticleCounter +=123;
//	if (ParticleCounter > 6000)ParticleCounter = 0;
//	EnclosureTemperature += 1;
//	if (EnclosureTemperature > 65)EnclosureTemperature = 0;
//	EnclosureHumidity += 1;
//	if (EnclosureHumidity > 65)EnclosureHumidity = 0;
//	EnclosurePressureDifference += 1;
//	if (EnclosurePressureDifference > 65)EnclosurePressureDifference = 0;
//	EnclosureUvLedPwm += 1;
//	if (EnclosureUvLedPwm > 65)EnclosureUvLedPwm = 0;
//	EnclosureFanPwm += 1;
//	if (EnclosureFanPwm > 99)EnclosureFanPwm = 0;
//	EnclosureDoorLock += 1;
//	if (EnclosureDoorLock >1)EnclosureDoorLock = 0;
//	EnclosureDoorSense += 1;
//	if (EnclosureDoorSense > 1)EnclosureDoorSense = 0;
//	EnclosureCartridgeSense += 1;
//	if (EnclosureCartridgeSense > 1)EnclosureCartridgeSense = 0;
//	EnclosurePrintBedSense += 1;
//	if (EnclosurePrintBedSense > 1)EnclosurePrintBedSense = 0;
	int Doorsense=HighSideSwitches[3].DutyCycle;
	
	if (Doorsense > 0)Doorsense = 1;
	//EnclosureDoorSense = READ_BIT(PIN_PORT_E, PIN_NUM_11); 
	EnclosureDoorSense = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
	
	if (EnclosureDoorSense == 0)
	{
		doorSenseState = 1;
		if (previous_EnclosureDoorSense)
		{
			previous_EnclosureDoorSense = 0;	
			//SendFakeMcodeExecutionNotice(1, INVALID_ARG_VALUE, 1, INVALID_ARG_VALUE); // send fake notice for an M1 (S1=active)
		}
	}
	else
	{//reset logic so next time we can trigger a door lock enable
		doorSenseState = 0;
		previous_EnclosureDoorSense = 1;
	}

	sprintf(_rptStr, ">ES");
	//osseo variables to report
	//Monitored Parameter	Prefix Readout interpretation 

	//Particle Counter		: PC	Integer 0 - 100000
	//Temperature			: ET	Celsius with one decimal point
	//Humidity				: HD	Integer 0 - 100 is a relative humidity in percents
	//Pressure Difference	: PD	0 - 100 interpreted in Torrs(1 Torr = 1mmHg barometric)
	//LED Status			: UV	1 – ON, 0 - OFF
	//Light Level			: LL	Integer 0 - 100 interpreted as percentage of full power
	//Fan Status			: FS	1 Running, 0 - disabled
	//Fan Level				: FL	Integer 0 - 100 interpreted as percentage of full power
	//Door Latch			: DL	1 – Engaged, 0 - Disabled
	//Door Open / Close		: DS	0 - Open, 1 - Closed
	//Printing Cartridge	: CR	0 - Absent, 1 - Present
	//Printing Bed			: BD	0 - Absent, 1 - Present
		
	sprintf(_tmpStr, ":PC%d", ParticleCounter); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
		 
	sprintf(_tmpStr, ":ET%2.1f", EnclosureTemperature); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
		
	sprintf(_tmpStr, ":HD%d", EnclosureHumidity); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);

	EnclosurePressureDifference = ADC_Channel[4].adcAvg -512 ;
	sprintf(_tmpStr, ":PD%d", EnclosurePressureDifference); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
		
	sprintf(_tmpStr, ":LL%d", EnclosureUvLedPwm); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
	
	sprintf(_tmpStr, ":FL%d", EnclosureFanPwm); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
	sprintf(_tmpStr, ":FS%d", (RPMCounter)); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
	
	sprintf(_tmpStr, ":DL%d", Doorsense); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
	
	sprintf(_tmpStr, ":DS%d", doorSenseState); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
	
	sprintf(_tmpStr, ":CR%d", EnclosureCartridgeSense); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);
	
	sprintf(_tmpStr, ":BD%d", EnclosurePrintBedSense); // last "ARG_N" but passed through to end of motionQ
	strcat(_rptStr, _tmpStr);

		//		sprintf(_tmpStr, ":%c%d", 'T', (int)ADC_Channel[4].convAvg); // last "ARG_F" but passed through to end of motionQ
		//strcat(_rptStr, _tmpStr);

		if (strlen(_rptStr) > 3)
		{
			// there was info to send
			sendstringCr(_rptStr);
		}
	}

////////////////////////////////////////////////////////////////////////////////

void spare (void)
{
	// placeholder call for empty slice
}

////////////////////////////////////////////////////////////////////////////////

void ohNoMrBill(void)
{
	// XXXX should send an ERROR .... should never get here
}

////////////////////////////////////////////////////////////////////////////////
//uint16_t a = 0x3f00;
//uint16_t p = 0;
//static uint16_t * const DR3_Word = (uint16_t * const)&SPI3->DR;
void heartbeat(void)
{
	_gs._led.heartbeatState ^= 1;
	pinWrite(HEARTBEAT_PIN, _gs._led.heartbeatState);
	HeartBeat++;
//	
//	LCD_CS_CLR;
//	LCD_RS_CLR;
//	SPI3->CR1 |= SPI_CR1_SPE;
//	SPI3->CR1 |= 1 << 9; //SPI_CR1_CSTART;
//	*DR3_Word = a;
//	LCD_CS_SET;
//	p = *DR3_Word;
//	if (a + 1 == 0xff) a = 0;
//	else a++;
#ifdef GB_HEARTBEAT_PIN
	pinWrite(GB_HEARTBEAT_PIN, _gs._led.heartbeatState);
#endif

}

////////////////////////////////////////////////////////////////////////////////

void checkInWithHeads(void)
{
	if ((counter_1Hz % MOTION_TO_HH_CHECKIN_RATE) == 0)
	{
		sendSetCommPingBit(0);
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkBlockingWaits(void)
{	// called from 10Hz
	if (_MailBoxes._waitingFor.flags.u32)
	{   // at least one block;
		if (_MailBoxes._waitingFor.flags.bit.motionPnP)
		{
			_MailBoxes._waitingFor.timerPnP--;
			if (_MailBoxes._waitingFor.timerPnP <= 0)
			{   // timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.motionPnP = FALSE;
				sendError("PNP Motion Timeout");
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.extruderTemp)
		{
			waitingForExtruderTemp();
		}
		if (_MailBoxes._waitingFor.flags.bit.hotbedTemp)
		{
			waitingForHotbedTemp();
		}
		if (_MailBoxes._waitingFor.flags.bit.chamberTemp)
		{
			waitingForChamberTemp();
		}
		if (_MailBoxes._waitingFor.flags.bit.canbusProbeToArm)
		{
			_MailBoxes._waitingFor.timerProbe--;
			if (_MailBoxes._waitingFor.timerProbe <= 0)
			{   // timed out, so just move on
				TouchProbeFinished(PROBE_RESULTS_CANBUS_PROBE_ARMING_TIMEOUT, PROBE_TYPE_UNKNOWN, PROBE_ERROR);
				_MailBoxes._waitingFor.flags.bit.canbusProbeToArm = FALSE;
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.canAxisMotorHoming)
		{
			_MailBoxes._waitingFor.timerHoming--;
			if (_MailBoxes._waitingFor.timerHoming <= 0)
			{   // timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.canAxisMotorHoming = FALSE;
				sendError("HOMING TIMED OUT");
				// reset any canAxisMotors that did not finish...
				MotorStructure *M;
				for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
				{   // whiz through all the motors and clean up any flags
					if (M->canMotor)
					{
						M->axisSelfHomingInProgress = FALSE;
						for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
						{	// check if all them are finished
							if (M->canMotors[canAddrIndex].canAddress)
							{
								if (M->canMotors[canAddrIndex].selfHomingInProgress)
								{
									M->canMotors[canAddrIndex].selfHomingInProgress = FALSE;
									sendMotorResetToDevice(M->canMotors[canAddrIndex].canAddress);
								}
							}
						}
					}
				}
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.canLoopback)
		{
			_MailBoxes._waitingFor.timerCanLoopback--;
			if (_MailBoxes._waitingFor.timerCanLoopback <= 0)
			{   // timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.canLoopback = FALSE;
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.canGuiAck)
		{
			_MailBoxes._waitingFor.timerCanGuiAck--;
			if (_MailBoxes._waitingFor.timerCanGuiAck <= 0)
			{   // timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.canGuiAck = FALSE;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void loop_1000Hz_simple_work(void)
{
	_gs._milliseconds++;
	if (_highVoltageIsNotStableCoundownMs > 0)
	{
		_highVoltageIsNotStableCoundownMs--;
		if (_highVoltageIsNotStableCoundownMs == 0)
		{   //power now deemed good
			;
		}
	}
	if (Co2LaserWatchDogTimer)
	{
		Co2LaserWatchDogTimer--;
		if (Co2LaserWatchDogTimer == 0)
		{
			//CO2LaserAnalogPwrPWM = 0; //turn of 0-5v power
			TIM8->CCR3 = 0; //turn off direct input PWM  karlChris add osseo check
		}
	}
	HssControl(TICKS_PER_SEC_1000HZ_LOOP);
	DwellTimer();

	if (_requestToPauseAtEndOfMove)
	{
		motionQ_pauseMotionWhenSafe();
		_requestToPauseAtEndOfMove = FALSE;
	}

	if (_requestToAbortAtEndOfMove)
	{   // request abort from repetrel to stop at the end of the current move if possible
		_requestToAbortAtEndOfMove = FALSE;
		_abortCameFromRx = 1;   // tag that we were requested to abort
		SpindleDesiredSpeedPWM = 0; //kill power now
		motionQ_abortMotionWhenSafe(FLUSH_THEN_ABORT);  // once motion is stopped, ResetProcess() will be called
	}

	if (_jogging.ignoreInputsCountdownTimer)
	{
		_jogging.ignoreInputsCountdownTimer--;
		if (_jogging.M && _jogging.M->PULSES_TO_GO)
		{   // still decelerating, so make sure turnaround time is met
			_jogging.ignoreInputsCountdownTimer = imax(_jogging.ignoreInputsCountdownTimer, _jogging.dirChangePauseTimeMs);
		}
		if (_jogging.ignoreInputsCountdownTimer <= 0)
		{
			joggingDeInit();
		}
	}

#ifdef USE_CAN_MOTOR_HOME_SENSOR
	if (Motors[homeSensedAxisCntr].MotorInstalled && !Motors[homeSensedAxisCntr].canMotor)
#else //!USE_CAN_MOTOR_HOME_SENSOR
	if (Motors[homeSensedAxisCntr].MotorInstalled)
#endif //!USE_CAN_MOTOR_HOME_SENSOR
	{
		updateSensorHistoryIfEnabledWithoutIrq(&Motors[homeSensedAxisCntr].HomeSense);
	}
	homeSensedAxisCntr++;
	if (homeSensedAxisCntr > LastAxisMotor)
	{
		homeSensedAxisCntr = FirstAxisMotor;
	}

#ifdef ADD_ON_SPI_DISPLAY
	GUI_IncrementIntervalCount();
#endif
	counter_1000Hz++;
}

////////////////////////////////////////////////////////////////////////////////

void loop_100Hz_simple_work(void)
{
	PnP_Rclk_Clr;
	PnP_Enable_Clr;
	if (!_abortInProgress)
	{   // regular processing, so show state of pending ack
		_heartbeatRateControl = pendingAcknowledge ? HEARTBEAT_MODE_PENDING_ACK : HEARTBEAT_MODE_NORMAL;
	}

	if ((counter_100Hz % _heartbeatRateControl) == 0)
	{
		heartbeat();
	}
	HssControl(TICKS_PER_SEC_100HZ_LOOP);

	if (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_NONE)
	{   // normal operation
		DDLightSelection();
		_gs._led.canRxLedCnt = imax(0, _gs._led.canRxLedCnt - 1);
		pinWrite(CAN_TX_LED, (_gs._led.canTxLedCnt > 0) ? 1 : 0);
		_gs._led.canTxLedCnt = imax(0, _gs._led.canTxLedCnt - 1);
	}
	else
	{
		if ((_gs._flasher.varPtr != NULL) && (*_gs._flasher.varPtr))
		{   // have a ptr to a variable AND the variable is non-zero .... light 'em up

			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_LED) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				pinSet(CAN_TX_LED);
			}
			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_DDL) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_ON);
			}
		}
		else
		{
			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_LED) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				pinClear(CAN_TX_LED);
			}
			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_DDL) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_OFF);
			}
		}
	}

	counter_100Hz++;
	if (_needToWiggleDirectionPins > 0)
	{
		_needToWiggleDirectionPins--;

		for (MotorStructure *M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{
			outputDirectionBit(&M->Direction, (_needToWiggleDirectionPins & 1) ? DIRECTION_REVERSE : DIRECTION_FORWARD);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void loop_10Hz_simple_work(void)
{
	HssControl(TICKS_PER_SEC_10HZ_LOOP);

	if (_jogging.enabled && (_jogging.ignoreInputsCountdownTimer == 0) && (_jogging.M->PULSES_TO_GO == 0))
	{   // something went wrong and jogging did not get disabled;
		sendError("Forcing jog mode OFF");
		joggingComplete();
	}
#ifdef USE_AB_ENCODER
	if (_ABjogging.enabled)
	{
		uint32_t irq_disabled = interruptsOff();
		int detents = TIM5->CNT;
		TIM5->CNT = 0;  // reset cnt
		interruptsOn(irq_disabled);
		if (detents)
		{   // encoder moved a non-zero amount
			joggingUpdatePulses(_ABjogging.axisIndex, _ABjogging.incrAmount * detents);
		}
	}
#ifdef ADD_ON_SPI_DISPLAY
	else if (_ABjogging.offMode == JOG_OFF_MODE_GUI)
	{   // GUI control
		if (_gs._displaySelectDebounceCnt > 0)
			_gs._displaySelectDebounceCnt--;
		if (_gs._displaySelectDebounceCnt == 0)
		{
			//uint32_t irq_disabled = interruptsOff();
			int detents = TIM5->CNT;
			TIM5->CNT = 0;  // reset cnt
			//interruptsOn(irq_disabled);
			if (detents)
			{   // encoder moved a non-zero amount
				int nextPage;
				if (detents > 0)
				{   // encoder moved a positive amount
					if (GUI_GetCurrPage() == DISPLAY_PAGE_LAST_ONE)
						nextPage = 0;
					else
						nextPage = GUI_GetCurrPage() + 1;
				}
				else if (detents < 0)
				{   // encoder moved a negative amount
					if (GUI_GetCurrPage() == 0)
						nextPage = DISPLAY_PAGE_LAST_ONE;
					else
						nextPage = GUI_GetCurrPage() - 1;
				}
				GUI_SetNextPage(nextPage);
				_gs._displaySelectDebounceCnt = 3;
			}
		}
		else if (_gs._displaySelectDebounceCnt == 1)
		{
			//uint32_t irq_disabled = interruptsOff();
			TIM5->CNT = 0;  // reset cnt
			//interruptsOn(irq_disabled);
		}
	}
#endif //ADD_ON_SPI_DISPLAY
	else if (_ABjogging.offMode == JOG_OFF_MODE_FILAMENT)
	{
		;
	}
#endif //USE_AB_ENCODER

#ifdef MEASURE_TIME_SLIPPAGE
	if (_sendTimeSlippageData)
	{
		 M_Code_M779();
		_sendTimeSlippageData = FALSE;
	}
#endif //MEASURE_TIME_SLIPPAGE

	counter_10Hz++;
}

////////////////////////////////////////////////////////////////////////////////

void loop_1Hz_simple_work(void)
{
#ifdef GB_SECONDS_PIN
	pinToggleOutput(GB_SECONDS_PIN);
#endif //GB_SECONDS_PIN
	_gs._seconds++;
//	if (_gs._seconds % 2 == 0) LCD_CS_CLR; //pinWrite(SPI3_LCD_CS, 0);// 
//	else LCD_CS_SET;  //pinWrite(SPI3_LCD_CS, 1);
	IWDG_ReloadCounter();  // reset the independent watchdog hardware

	if ((masterCommPort == BOOTUP) && (_gs._seconds > 2) && ((_gs._seconds % 3) == 0))
	{   // every 3 seconds try to get repetrel's attention
		_bootupAlertHostChar = HELLO_WORLD_CHAR;
		_sendBootupAlertHostChar = TRUE;    // reload as USB code resets to FALSE when xfer occurs
		forceCharToHw(_bootupAlertHostChar);    // force out a message to all comm ports to let host know we're alive
	}


	checkInWithHeads();
	HssControl(TICKS_PER_SEC_1HZ_LOOP);
	if ((counter_1Hz % CLEAR_STICKY_ERROR_RATE) == 0)
	{
		resetStickyErrorFlags();
	}

	if (_gs._errorThrottledCount >=  MAX_ERROR_MESSAGES)
	{
		_gs._errorThrottledCount--;  //allow one message per sec, even when spewing
	}

	if (_abortInProgress > 0)
	{   // watchdog -- limit length of abort to "ABORT_TIME" seconds, just in case something goes wrong
		_abortInProgress --;
		if (_abortInProgress == 0)
		{
			_abortFinisedNeedToResetProcessSynchronously = TRUE; //gracefully reset
			sendInfo("ABORTING abort process");
			pauseToTransmitBufferToEmpty();
		}
	}

#ifdef ALLOW_NATIVE_LIGHTBURN
	if ((!_lightburnModeEnabled) && (_repetrelCommWatchCount > 0))
#else //!ALLOW_NATIVE_LIGHTBURN
	if (_repetrelCommWatchCount > 0)
#endif //!ALLOW_NATIVE_LIGHTBURN
	{
		_repetrelCommWatchCount--;
		if (_repetrelCommWatchCount == 0)
		{   // lost repetrel, so go to a safe state
			sendSetSwResetBit(0);   // reset all heads;
			sendError("LOST COMM WITH REPETREL");
		}
	}

	counter_1Hz++;
}

////////////////////////////////////////////////////////////////////////////////

boolean almostTheSameByPct(float new, float old, float pctError)
{
	return(fabsf(old - new) > (pctError * old));
}
boolean notAlmostTheSameByPct(float new, float old, float pctError)
{
	return(!(fabsf(old - new) > (pctError * old)));
}
boolean almostTheSameByDelta(float a, float b, float epsilon)
{
	return(fabsf(a - b) <= epsilon);
}
boolean notAlmostTheSameByDelta(float a, float b, float epsilon)
{
	return(!(fabsf(a - b) <= epsilon));
}

////////////////////////////////////////////////////////////////////////////////

boolean anyAxisNeedingToBeHomed(void)
{
	for (MotorStructure *M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors and set the flag showing axis has been homed
		if (M->MotorInstalled && M->HomeSense.Enabled && !M->HasBeenHomed)
		{
			return(TRUE);
		}
	}
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void requireAllAxesToHome(void)
{
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors and set the flag showing axis has been homed
		if (M->MotorInstalled)
		{
			if (M->HomeSense.Enabled)
			{
				M->HasBeenHomed = FALSE;
				M->HasBeenHomed_BlockedMsgSent = FALSE;
			}
			else
			{
				M->HasBeenHomed = TRUE;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void outputControlBit(controlBitStruct *outputPtr, assertValue_t value)
{
	if (outputPtr->Polarity == ACTIVE_HIGH) // normal
		pinWrite(outputPtr->Bit, value);
	else    //  (outputPtr->polarity == ACTIVE_LOW)
		pinWrite(outputPtr->Bit, (~value) & 0x1);
}

////////////////////////////////////////////////////////////////////////////////

void assertControlBit(controlBitStruct *outputPtr)
{
	if (outputPtr->Polarity == ACTIVE_HIGH) // normal
		PIN_SET(outputPtr->gpioPort, outputPtr->gpioMask);
	else    //  (outputPtr->polarity == ACTIVE_LOW)
		PIN_CLEAR(outputPtr->gpioPort, outputPtr->gpioMask);
}

////////////////////////////////////////////////////////////////////////////////

void deassertControlBit(controlBitStruct *outputPtr)
{
	if (outputPtr->Polarity == ACTIVE_HIGH) // normal
		PIN_CLEAR(outputPtr->gpioPort, outputPtr->gpioMask);
	else    //  (outputPtr->polarity == ACTIVE_LOW)
		PIN_SET(outputPtr->gpioPort, outputPtr->gpioMask);
}

////////////////////////////////////////////////////////////////////////////////

int readControlBitValue(controlBitStruct *outputPtr)
{
	return((outputPtr->gpioPort->ODR & outputPtr->gpioMask) ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////

assertValue_t readControlBitState(controlBitStruct *outputPtr)
{
	if (outputPtr->Polarity == ACTIVE_HIGH)
		return((outputPtr->gpioPort->ODR & outputPtr->gpioMask) ? ASSERTED : DEASSERTED);
	else if (outputPtr->Polarity == ACTIVE_LOW)
		return((outputPtr->gpioPort->ODR & outputPtr->gpioMask) ? DEASSERTED : ASSERTED);
	else
		return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void outputDirectionBit(directionBitStruct *outputPtr, pinStateValue_t value)
{
	if (outputPtr->InvertDefault == NO)
	{
		if (value == 0)
			PIN_CLEAR(outputPtr->gpioPort, outputPtr->gpioMask);
		else
			PIN_SET(outputPtr->gpioPort, outputPtr->gpioMask);
	}
	else //if (outputPtr->InvertDefault == YES)
	{
		if (value == 0)
			PIN_SET(outputPtr->gpioPort, outputPtr->gpioMask);
		else
			PIN_CLEAR(outputPtr->gpioPort, outputPtr->gpioMask);
	}
}

////////////////////////////////////////////////////////////////////////////////

int readDirectionBitValue(directionBitStruct *outputPtr)
{
	return((outputPtr->gpioPort->ODR & outputPtr->gpioMask) ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////

direction_t readDirectionBitState(directionBitStruct *outputPtr)
{
	 if (outputPtr->InvertDefault == NO)
		return((outputPtr->gpioPort->ODR & outputPtr->gpioMask) ? DIRECTION_REVERSE : DIRECTION_FORWARD);
	else
		return((outputPtr->gpioPort->ODR & outputPtr->gpioMask) ? DIRECTION_FORWARD : DIRECTION_REVERSE);
}

////////////////////////////////////////////////////////////////////////////////

int readSensorBitValue(sensorStruct *sensorPtr)
{
	return((sensorPtr->gpioPort->IDR & sensorPtr->gpioMask) ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////

sensorState_t readSensorBitState(sensorStruct *sensorPtr)
{
	if (sensorPtr->Polarity == ACTIVE_HIGH)
		return((sensorPtr->gpioPort->IDR & sensorPtr->gpioMask) ? SENSOR_TRIPPED : SENSOR_OPEN);
	else if (sensorPtr->Polarity == ACTIVE_LOW)
		return((sensorPtr->gpioPort->IDR & sensorPtr->gpioMask) ? SENSOR_OPEN : SENSOR_TRIPPED);
	else
		return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void setSensorBitState(sensorStruct *sensorPtr, uint32_t value)
{
	if (sensorPtr->Polarity == ACTIVE_HIGH)
		sensorPtr->State = (value == 1) ? SENSOR_TRIPPED : SENSOR_OPEN;
	else if (sensorPtr->Polarity == ACTIVE_LOW)
		sensorPtr->State = (value == 0) ? SENSOR_TRIPPED : SENSOR_OPEN;
	else
		sensorPtr->State = SENSOR_STATE_UNKNOWN;
}

////////////////////////////////////////////////////////////////////////////////

boolean sensorEnabledAndTripped(sensorStruct *sensorPtr)
{
	if (sensorPtr->Enabled)
		return(readSensorBitState(sensorPtr) == SENSOR_TRIPPED);
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void initSensor(sensorStruct *sensorPtr,  sensorIndex_t SensorIndex, char *name, boolean enabled, polarity_t polarity, pinType bit, int numDebounceMaskBits)
{
	bzero(sensorPtr, sizeof(sensorStruct));
	sensorPtr->Bit = bit;
	sensorPtr->gpioPort = pinExtractPortPtr(sensorPtr->Bit);
	sensorPtr->gpioMask = pinExtractPinMask(sensorPtr->Bit);
	strcpy(sensorPtr->Name, name);
	sensorPtr->Polarity = polarity;
	sensorPtr->NumDebounceMaskBits = iFitWithinRange(numDebounceMaskBits, 1, 31);   // must be between 1 and 31
	sensorPtr->DebounceMask = ((uint32_t)1 << sensorPtr->NumDebounceMaskBits) - 1;
	sensorPtr->State = SENSOR_STATE_UNKNOWN;
	sensorPtr->History = 0;
	sensorPtr->SensorIndex = SensorIndex;
	sensorPtr->Enabled = enabled;
	setSensorEnable(sensorPtr, enabled);
	sensorPtr->ErrorMessageSent = FALSE;
	sensorPtr->StateChangeDetected = FALSE;
	sensorPtr->TransientDetected = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void setSensorStateToUnknown(sensorStruct *sensorPtr)
{
	sensorPtr->State = SENSOR_STATE_UNKNOWN;
	sensorPtr->History = 0;
	sensorPtr->ErrorMessageSent = FALSE;
	sensorPtr->StateChangeDetected = FALSE;
	sensorPtr->TransientDetected = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void updateSensorHistoryIfEnabledWithoutIrq(sensorStruct *sensorPtr)
{   // if sensor in use, read the current value of the sensor with interrupts off, update sensor history
	// this method does NOT update the sensor history or any state setting/flags

	if (sensorPtr->Enabled)
	{   // sensor is in use
		uint32_t irq_disabled = interruptsOff();
		sensorPtr->History = (sensorPtr->History << 1) | (readSensorBitValue(sensorPtr) == sensorPtr->Polarity); // keep history up to date
		interruptsOn(irq_disabled);
	}
}

////////////////////////////////////////////////////////////////////////////////

sensorState_t readDebouncedSensorState(sensorStruct *sensorPtr)
{   // return current debounced state of the sensor

	if (sensorPtr->Enabled)
	{   // sensor is in use
		//int numCopies = 1; // default is to read once per call
		// reset dynamic flags
		sensorPtr->StateChangeDetected  = FALSE;
		sensorPtr->TransientDetected    = FALSE;

		if (sensorPtr->State == SENSOR_STATE_UNKNOWN)
		{   // first time using sensor, so clear critical flags and reset history to current state
			// should already be setsensorPtr->ErrorMessageSent     = FALSE;
			sensorPtr->History = (readSensorBitState(sensorPtr) == SENSOR_TRIPPED) ? sensorPtr->DebounceMask : 0;
		}
		else
		{   // normal operating case
			sensorPtr->History = (sensorPtr->History << 1) | readSensorBitState(sensorPtr);
		}

		if ((sensorPtr->History & sensorPtr->DebounceMask) == sensorPtr->DebounceMask)
		{   // CLOSED/TRIPPED
			if (sensorPtr->State != SENSOR_TRIPPED)
			{   // changing to an CLOSED/TRIPPD state
				sensorPtr->State                = SENSOR_TRIPPED;
				sensorPtr->StateChangeDetected  = TRUE;
			}
		}
		else if ((sensorPtr->History & sensorPtr->DebounceMask) == 0)
		{   // OPEN
			if (sensorPtr->State != SENSOR_OPEN)
			{   // changing to an OPEN state
				sensorPtr->State                = SENSOR_OPEN;
				sensorPtr->StateChangeDetected  = TRUE;
			}
		}
		else
		{   // not a consistant history in the mask region, so either a noise event or starting to change state
			sensorPtr->TransientDetected = TRUE;
		}
	}
	else
	{
		sensorPtr->State = SENSOR_STATE_UNKNOWN;
	}

	return(sensorPtr->State);
}

////////////////////////////////////////////////////////////////////////////////

sensorState_t fastReadDebouncedSensorState(sensorStruct *sensorPtr)
{   // return current debounced state of the sensor -- but assumes sensor is init, enabled, and no need to track state changes
	if (sensorPtr->Enabled)
	{   // sensor is in use
		sensorPtr->History = (sensorPtr->History << 1) | readSensorBitState(sensorPtr);
		if ((sensorPtr->History & sensorPtr->DebounceMask) == sensorPtr->DebounceMask)
		{   // CLOSED/TRIPPED
			sensorPtr->State                = SENSOR_TRIPPED;
		}
		else if ((sensorPtr->History & sensorPtr->DebounceMask) == 0)
		{   // OPEN
			sensorPtr->State                = SENSOR_OPEN;
		}
	}
	else
	{
		sensorPtr->State = SENSOR_STATE_UNKNOWN;
	}

	return(sensorPtr->State);
}

////////////////////////////////////////////////////////////////////////////////

void setSensorEnable(sensorStruct *sensorPtr, boolean enable)
{   // this method should always be used to change the state of a sensor enable
	if (sensorPtr->Enabled != enable)
	{   // different, so need to update;
		setSensorStateToUnknown(sensorPtr);
		sensorPtr->Enabled = enable;
	}
}

////////////////////////////////////////////////////////////////////////////////

void setSensorPolarityAndEnable(sensorStruct *sensorPtr, polarity_t polarity, boolean initHistory)
{
	if (sensorPtr->Bit == PIN_UNDEFINED)
	{
		polarity = ACTIVE_DISABLED;
	}
	switch (polarity) {
	case 0:
		sensorPtr->Polarity = ACTIVE_LOW;
		setSensorEnable(sensorPtr, TRUE);
		break;
	case 1:
		sensorPtr->Polarity = ACTIVE_HIGH;
		setSensorEnable(sensorPtr, TRUE);
		break;
	default:
		sensorPtr->Polarity = ACTIVE_DISABLED;
		setSensorEnable(sensorPtr, FALSE);
		break;
	}
	if (sensorPtr->Enabled && initHistory)
	{   // initialize sensor histor
		readDebouncedSensorState(sensorPtr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkStartButton(void)
{
	if (_highVoltageIsNotStableCoundownMs)
	{
		setSensorStateToUnknown(&startButton);
	}
	else if ((readDebouncedSensorState(&startButton) == SENSOR_TRIPPED) && startButton.StateChangeDetected)
	{   // leading edge start button pressed
		SendFakeMcodeExecutionNotice(1, INVALID_ARG_VALUE, 1, INVALID_ARG_VALUE); // send fake notice for an M1 (S1=active)
		sendInfo("Start button pressed");

		if (_sendingGBStringsMask & GB_STRING_DUMP_ON_START)  // use M797 S<mask> to enable
		{
			sendCrashDataFromRAMtoFlash(); // save a copy of key variables to flash
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

#ifdef USE_AB_ENCODER
void checkABEncoderSelectButton(void)
{
	if ((readDebouncedSensorState(&ABEncoderSelectButton) == SENSOR_TRIPPED) && ABEncoderSelectButton.StateChangeDetected)
	{   // leading edge start button pressed
		SendFakeMcodeExecutionNotice(1, INVALID_ARG_VALUE, 2, INVALID_ARG_VALUE); // per Karl, send fake notice for an M1 (S1=active)
		sendInfo("AB select button pressed");
	}
}
#endif //USE_AB_ENCODER

////////////////////////////////////////////////////////////////////////////////

void sendEmoMessage(void)
{
	if (EMO.Enabled && (_gs._milliseconds > IGNORE_EMO_DURING_BOOT_TIME_MS))
	{
		if (EMO.State == SENSOR_CLOSED)
		{
			sprintf(_errorStr, "EMO Pressed - Motion BLOCKED");
			sendMotionError('E', '0', 0, 0, _errorStr);
		}
		else if (EMO.State == SENSOR_OPEN)
		{
			sprintf(_errorStr, "EMO Released - Rehome all axes");
			sendMotionInfo('E', '1', 0, 0, _errorStr);
		}
		else // if (state == SENSOR_STATE_UNKNOWN)
		{
			sprintf(_errorStr, "EMO Unknown)");
			sendMotionError('E', '2', 0, 0, _errorStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void setMotionSensorsToUnknownState(MotorStructure *M)
{
	setSensorStateToUnknown(&M->FaultSense);
	setSensorStateToUnknown(&M->Limit1Sense);
	setSensorStateToUnknown(&M->Limit2Sense);
}
////////////////////////////////////////////////////////////////////////////////

void setAllMotionSensorsToUnknownState(void)
{
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		setMotionSensorsToUnknownState(M);
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkEMO(void)
{   //EMO PRESSED == SENSOR_CLOSED/TRIPPED == NO VOLTAGE
#ifdef HYDRA_DIAGS
	if (_diagsEnabled) return;
#endif
#ifdef USE_HYREL_IO
	return;
#endif
	if (EMO.Enabled && (_gs._milliseconds > IGNORE_EMO_DURING_BOOT_TIME_MS))
	{
		readDebouncedSensorState(&EMO);

#ifdef GB_HIDDEN_WARNINGS
		int doWeNeedToAddATransientDetectOnEMO;
#endif //GB_HIDDEN_WARNINGS
//      if (EMO.TransientDetected)
//      {   // either noise, or start of transition
//          _highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
//      }
//      else
		if (EMO.StateChangeDetected && (EMO.State == SENSOR_TRIPPED))
		{   // EMO has been pressed (killing power) OR at the end of the boot time window and tripped/closed

			_highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
			sendEmoMessage();
			requireAllAxesToHome();
			_blockAllMotion = TRUE;
			_blockAbsoluteMotion = TRUE;
			if (motionQ_notEmpty())
			{   // there's at least one move active/in the queue, so need to flush the queue and stop any future moves
				motionQ_abortMotionWhenSafe(FLUSH_THEN_ABORT);
			}
		}
		else if (EMO.StateChangeDetected && (EMO.State == SENSOR_OPEN))
		{   // EMO has been released, so power should be ramping up -- OR first time after bootup
			sendEmoMessage();
			_blockAllMotion = FALSE;
			_blockAbsoluteMotion = _motionSensorTripped; // release block if no faults are present. will allow jogging, but no absolute motion until rehomed.
			_highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
		}
		else if (EMO.State == SENSOR_TRIPPED)
		{   // still no power, so reset coundown to max
			_highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

sensorStruct *getMotorSensorPtr( MotorStructure *M, sensorIndex_t SensorIndex)
{
#ifdef USE_CAN_MOTOR_HOME_SENSOR
	if (M->canMotor)
	{	// will be checked on the head
		return(NULL);
	}
	else
#endif //USE_CAN_MOTOR_HOME_SENSOR
	{
		switch (SensorIndex)
		{
		case SENSOR_INDEX_FAULT  : return(&M->FaultSense);
		case SENSOR_INDEX_LIMIT1 : return(&M->Limit1Sense);
		case SENSOR_INDEX_LIMIT2 : return(&M->Limit2Sense);
		default : return(NULL);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean  checkMotorSensor(sensorIndex_t sensorIndex)
{   // this will check for any faults.  if a new fault is found, it is reported to repetrel
	// and that axis is tagged as having a fault (no more absolute motion)

	// if ANY FAULT detected .....
	// if ANY MOTION had occurred since reset, then motion is stopped, queues are
	//     flushed and ALL motion is blocked until reset/abort
	// if NO MOTION had occurred since reset, then just block absolute moves. this
	//     allows users to reset the system and be able to jog non-faulted motors
	// after fault(s) have been removed, any faulted axis will need to be
	//      re-homed
	//
	// returns TRUE if any sensor is tripped

#ifdef HYDRA_DIAGS
	if (_diagsEnabled) return(FALSE);
#endif
#ifdef USE_HYREL_IO
	return(FALSE);
#endif

	MotorStructure *M;
	boolean newFaultFound = FALSE;
	boolean anyFaultFound = FALSE;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		if (M->MotorInstalled && !((M == &Motors[M_C]) && (sensorIndex == SENSOR_INDEX_LIMIT1)))
		{   // only check if motor is installed and NOT the special case (C_L1 is used for EMO)
			sensorStruct *sensorPtr = getMotorSensorPtr(M, sensorIndex);
			if (sensorPtr == (sensorStruct *)NULL)
				continue;   // skip over undefined sensors

			if (_highVoltageIsNotStableCoundownMs)
			{   // power not good, so reset the sensor to unknown state
				setSensorStateToUnknown(sensorPtr);
			}
			else if (sensorPtr->Enabled)
			{
				boolean stateWasUnknown = (sensorPtr->State == SENSOR_STATE_UNKNOWN);   // first time checking sensor after booting or MCODE to enable sensor

				readDebouncedSensorState(sensorPtr);

				if (sensorPtr->StateChangeDetected)
				{   // special cases for detected change in state
					if (sensorPtr->State == SENSOR_CLOSED)
					{   // new fault detected
						newFaultFound = TRUE;
						anyFaultFound = TRUE;

						sprintf(_errorStr, "%s sensor detected on %c axis %s", sensorPtr->Name, M->AxisLabel, stateWasUnknown ? "reset/enabled" : "");
						sendMotionError('F', M->AxisLabel, M->POSITION * M->UnitsPerPulse, 0, _errorStr);

						if (M->HomeSense.Enabled)
						{   // if the Axis has a home sensor, FORCE A REHOMING OF JUST THE FAULTED AXIS
							M->HasBeenHomed = FALSE;
							M->HasBeenHomed_BlockedMsgSent = FALSE; // re-arm message in case users tries to move without homing
						}

						if (sensorIndex == SENSOR_INDEX_LIMIT1)
							M->Limit1Sense.State = TRUE;
						else if (sensorIndex == SENSOR_INDEX_LIMIT2)
							M->Limit2Sense.State = TRUE;
					}
					else if (sensorPtr->State == SENSOR_OPEN)
					{
						sensorPtr->ErrorMessageSent = FALSE;    // re-arm for next time
						if (sensorIndex == SENSOR_INDEX_LIMIT1)
							M->Limit1Sense.State = FALSE;
						else if (sensorIndex == SENSOR_INDEX_LIMIT2)
							M->Limit2Sense.State = FALSE;
					}
				}
				else if (sensorPtr->State == SENSOR_CLOSED)
				{   // decision based on current state
					anyFaultFound = TRUE;
				}
			}
		}
	}

	if (newFaultFound)
	{   // at least one axis has a newly discovered fault, so report it and take action
		// report is created here to give one line for a given sensor across all motors

		sprintf(_rptStr, ">MF");
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors, if the motor is installed, then report (shows full XYZ position where fault occurred
			if (M->MotorInstalled && !((M == &Motors[M_C]) && (sensorIndex == SENSOR_INDEX_LIMIT1)))
			{   // only check if motor is installed and NOT the special case (C_L1 is used for EMO)
				sensorStruct *sensorPtr = getMotorSensorPtr(M, sensorIndex);
				if (sensorPtr->Enabled && (sensorPtr->State == SENSOR_TRIPPED))
				{
					sprintf(_tmpStr, ":%c_%s", M->AxisLabel, sensorPtr->Name);
				}
				else
				{
					sprintf(_tmpStr, ":%c%4.3f", M->AxisLabel, M->POSITION * M->UnitsPerPulse);
				}
				strcat(_rptStr, _tmpStr);
			}
		}
		strcat(_rptStr, ":POSITION when sensor was detected");
		sendInfo(_rptStr);

		_blockAllMotion = TRUE;
		if (motionQ_notEmpty())
		{   // there's at least one move active/in the queue, so need to flush the queue and stop any future moves
			motionQ_abortMotionWhenSafe(FLUSH_THEN_CONTINUE);
		}
	}

	if (anyFaultFound)
	{
		_blockAbsoluteMotion = TRUE;
	}

	return(anyFaultFound);
}

////////////////////////////////////////////////////////////////////////////////

// These three methods are called in sequence from SysTick lopp.  For each pass, a new check of all sensors is performed and the
// valiable _motionSensorTripped is updated.

void checkMotorFaultSensor(void)
{
	_motionSensorTrippedNext = FALSE; //checkMotorSensor(SENSOR_INDEX_FAULT);
}
void checkMotorLimit1Sensor(void)
{
	_motionSensorTrippedNext |= FALSE;//checkMotorSensor(SENSOR_INDEX_LIMIT1);
}
void checkMotorLimit2Sensor(void)
{
	_blockAbsoluteMotion = FALSE;
	return;
	_motionSensorTrippedNext |= checkMotorSensor(SENSOR_INDEX_LIMIT2);
	_motionSensorTripped = _motionSensorTrippedNext; // update variable based on latest pass

	if ((_motionSensorTripped == FALSE) && !(EMO.Enabled && (EMO.State == SENSOR_TRIPPED)) && !_highVoltageIsNotStableCoundownMs)
	{
		_blockAbsoluteMotion = FALSE;
	}
}

////////////////////////////////////////////////////////////////////////////////

void readInputs(void)
{

	checkStartButton();
	checkEMO();

	//checkABEncoderSelectButton(); //USE_AB_ENCODER

	PWMCntrl();
	Head11_Temperature = _MailBoxes._inbox[0].HBPayload[0].i16[0]>>5;
	Head11_HTRDuty = _MailBoxes._inbox[0].HBPayload[0].i16[1];
	Head11_FanDuty = _MailBoxes._inbox[0].HBPayload[0].i16[2];
	Head11_Spare = _MailBoxes._inbox[0].HBPayload[0].i16[3];
}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

void InitializeFixtureOffsets(void)
{   //this will iniitalize the fixtrue offsets to zero,
	//we might want to use the soap string to automatically reload these values later karlChris
	int i=0;

	ClearAllMotorFixtureOffsets();
	ClearAllMotorHeadOffsets();

	//now zero out the ToolOffsets
	for (i=0;i<NUM_TOOL_OFFSETS;i++)
	{
		ToolOffsets[i].ToolLength=0.0f;
		ToolOffsets[i].ToolDiameter=0.0f;
	}
}

void SetAxisHomePosition(MotorStructure *M)
{
	// set posting homing position
	M->POSITION   = M->HomeDestinationInPulses;
	M->Q_POSITION = M->HomeDestinationInPulses;
	M->Q_LastRequestedPositionInUnits = (M->HomeDestinationInPulses * M->UnitsPerPulse) - SumOfAllMotorOffsets(M);
}

void SetAllAxisHomePosition(boolean checkValidArg)
{
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors and set XYZABC to 0.0 to get the correct final position loaded after homing
		if (!checkValidArg || (checkValidArg && motorArgPresent(M)))
		{
			SetAxisHomePosition(M);
		}
	}
}

boolean _ProcessG28MultipassHomingMoveFailed;
void ProcessG28MultipassHoming()
{   // multi-part homing - countdown to 0 of CannedCycleStep
	// step (7) G28_START_MULTIPASS_HOME         - initial state from G28 call; validate req axis has a home sensor
	// step (6) G28_PASS_MOVE_INTERMEDIATE_POINT - rapid move to the optional XYZABC intermediate position
	// step (5) G28_PASS_HOME                    - homing from that intermediate position using HOMING_SPEED
	// step (4) G28_SET_DESTINATION_HOME         - set the POSITION based on homing direction, home offset, etc
	// step (3) G28_PASS_LEAVE_HOME              - move to the pop-off distance at RAPID_SPEED
	// step (2) G28_PASS_REHOME                  - home again at the REHOMING_SPEED
	// step (1) G28_SET_DESTINATION_REHOME       - set the POSITION based on homing direction, home offset, etc
	// step (0) G28_FINISHED_MULTIPASS_HOME      - all done; cleanup flags, etc

	MotorStructure *M;
	boolean save_IncrementalMove;
	passFail_t moveResult;

#ifdef GB_HIDDEN_WARNINGS
	int fixsave_IncrementalMove;
#endif //GB_HIDDEN_WARNINGS

	if (_blockAllMotion)
	{   // about multi-cycle move ....
		CannedCycleLastMove = TRUE;
		CannedCycleFlag = 0; // drop out the canned cycle loop
		CannedCycleStep = 0;
		return;
	}

	if (motionQ_notEmpty())
	{   // waiting for prior step to complete before jumping on to the next move as ALL steps of this
		// multi-part homing sequence must be run single step (effectively avoiding the motionQ)
		return;
	}

	_motionQ_ForceQtoEmpty = TRUE; // force to single step ... needed to get around control issue with
								   // urgent commands bypassing regular flow control
	save_IncrementalMove = _IncrementalMove; // save state of _IncrementalMove to switch to Absolute Mode.
	ExecutionPtr = CannedCyclePtr; //restore ExecutionPtr to our index

	switch (CannedCycleStep)
	{ // step are listed in order of processing
	case G28_START_MULTIPASS_HOME:  // (7) initial state from G28 call; validate req axis has a home sensor
		_ProcessG28MultipassHomingMoveFailed = FALSE;   // reset to be ready for the series of moves needed to home
		break;
	case G28_PASS_MOVE_INTERMEDIATE_POINT:  // (6) rapid move to the optional XYZABC intermediate position
		_IncrementalMove = FALSE; // absolute move to the intermediate point
		// per NIST and Fanuc manual, the XYZABC specify intermediate points to go to prior to homing (possible safe position)
		if (ARG_I_PRESENT && (ARG_I == 1.0f))   // GB XXX gregkarl G28: ARG_I not part of the spec, but allows testing of intermediate move
		{
			moveResult = motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);   // move to the intermediate point.  motionQ should be empty, so no need to check
			if (moveResult == FAIL)
			{
				_ProcessG28MultipassHomingMoveFailed = TRUE;
			}
		}
		break;
	case G28_PASS_HOME:  // (5) homing from that intermediate position using HOMING_SPEED
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors involved and make sure they have a home sensor; invalidate that axis otherwise
			if (motorArgPresent(M) && !M->HomeSense.Enabled)
			{   // no home sensor enabled, yet trying to home, so Error and remove from list
				 sprintf(_errorStr, "Trying to home %c axis which does not have a home sensor enabled", (int)M->AxisLabel);
				 sendError(_errorStr);
				 setMotorArg(M, INVALID_ARG_VALUE);
			}
		}
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and set the flag showing axis has not been homed and send status
			if (motorArgPresent(M))
			{
				sendAxisHomedStatus(M);
				M->HasBeenHomed = FALSE;
			}
		}
		_IncrementalMove = FALSE;  // absolute move to home
		moveResult = ExecuteHomingMove(HOMING_MOTION_RATE);  // note: speed will be scaled down such that we can safely decelerate in the "hysteresis" distance.
		if (moveResult == FAIL)
		{	// ie failed key check....
			_ProcessG28MultipassHomingMoveFailed = TRUE;
		}
		break;
	case G28_SET_DESTINATION_HOME:  // (4) set the POSITION based on homing direction, home offset, etc
		SetAllAxisHomePosition(TRUE);
		break;
	case G28_PASS_LEAVE_HOME:  // (3) move off home position by a small amount
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and restuff XYZABC with the pop-off amount to get past the sensor
			if (motorArgPresent(M))
			{
				setMotorArgInNativeUnits(M, (M->HomePopOffDistanceInUnits));
			}
		}
		_IncrementalMove = TRUE;  // incremntal move off the sensor
		moveResult = motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);  // motionQ should be empty, so no need to check
		if (moveResult == FAIL)
		{
			_ProcessG28MultipassHomingMoveFailed = TRUE;
		}
		break;
	case G28_PASS_REHOME:  // (2) re-home
		// first make sure the pop-off was successful (alert otherwise)
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and make sure no longer on the home sensor
			if (motorArgPresent(M) && (M->HomePopOffDistanceInUnits != 0.0f) && (!readDebouncedSensorState(&M->HomeSense) == SENSOR_OPEN))
			{
				M->HomingFailed = TRUE;
				sprintf(_errorStr, "Insufficient Home Pop-Off Distance on %c axis (%4.3f)", M->AxisLabel, M->HomePopOffDistanceInUnits);
				sendError(_errorStr);
			}
			else
			{
				M->HomingFailed = FALSE;
			}
		}
		_IncrementalMove = FALSE;  // absolute move to home
		moveResult = ExecuteHomingMove(REHOMING_MOTION_RATE); // rehome at a slower rate
		if (moveResult == FAIL)
		{
			_ProcessG28MultipassHomingMoveFailed = TRUE;
		}
		break;
	case G28_SET_DESTINATION_REHOME:  // (1) set the POSITION based on homing direction, home offset, etc
		SetAllAxisHomePosition(TRUE);
		if (ARG_Z_PRESENT && Motors[M_Z].HomeSense.Enabled && (Motors[M_Z].HomingFailed == FALSE))
		{
			ToolOffsets[0].ToolLength = Motors[M_Z].HomeDestinationInUnits;
		}
		break;
	case G28_FINISHED_MULTIPASS_HOME:  // (0) all done; cleanup flags, etc
		_IncrementalMove = save_IncrementalMove;  // restore state of _IncrementalMove
		CannedCycleLastMove = TRUE;
		CannedCycleFlag = 0; // drop out the canned cycle loop
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and set the flag showing axis has been homed
			if (motorArgPresent(M))
			{
				if (_ProcessG28MultipassHomingMoveFailed)
				{   // at least one of the moves did not complete
					M->HomingFailed = TRUE;
				}
				M->HasBeenHomed = (M->HomingFailed == FALSE) ? TRUE : FALSE;
				M->HomeSense.ErrorMessageSent = FALSE;
				sendAxisHomedStatus(M);
			}
		}
		break;
	default:
		break;
	}
	CannedCycleStep--;
}

////////////////////////////////////////////////////////////////////////////////
#ifdef NEW_G38
void cleanupAfterCannedCycleMove(void)
{
		_IncrementalMove = _Canned.save_IncrementalMove;  // restore state of _IncrementalMove
		CannedCycleLastMove = TRUE;
		CannedCycleFlag = 0; // drop out the canned cycle loop
}

void initCannedCycleMove(int gcode)
{   // grab all pertinent info from command settings to use for loop control
	bzero(_Canned, (sizeof(CannedStructure));
	_Canned.gcode  = gcode;
	_Canned.startX = Motors[X].Q_LastRequestedPositionInUnits;
	_Canned.startY = Motors[Y].Q_LastRequestedPositionInUnits;
	_Canned.startZ = Motors[Z].Q_LastRequestedPositionInUnits;
	_Canned.deltaX = ARG_I;
	_Canned.deltaY = ARG_J;
	_Canned.deltaZ = ARG_K;
	_Canned.endX   = ARG_X;
	_Canned.endX   = ARG_Y;
	_Canned.endX   = ARG_Z;
	_Canned.save_IncrementalMove = _IncrementalMove;
	_Canned.saveTargetFeedRateInMmPerSec = _TargetFeedRateInMmPerSec;
	_Canned.CannedCyclePtr = ExecutionPtr;
	if (gcode == 38)
	{
		_Canned.probeType = (probeType_t)(int)ARG_D;
		_Canned.probeNumber = (int)ARG_P;
	}
	else
	{
		_Canned.diameter = ARG_D;
	}
}

boolean isAContactProbe(probeType_t probeType)
{
	boolean isContact = FALSE;
}
	switch (_Canned.probeType)
	{
	case PROBE_TYPE_NONE:    isContact = FALSE; break;
	case PROBE_TYPE_CONTACT: isContact = TRUE;  break;
	case PROBE_TYPE_BLTOUCH: isContact = TRUE;  break;
	case PROBE_TYPE_HC_SR04: isContact = FALSE; break;
	case PROBE_TYPE_IGAGING: isContact = FALSE; break;
	default: break;
	}
	return(isContact);

void ProcessG38MultipassProbing()
{   // multi-part probing - countdown to 0 of CannedCycleStep
	// step (8) G38_START_MULTIPASS_PROBING      - initial state from G38 call
	// step (7) G28_ARM_PROBE                    - if touch(contact) probe, then arm
	// step (6) G38_LOWER_PROBE                  - drop probe to requested Z amount
	// step (5) G38_PAUSE_BEFORE_READING         - wait for motion to finish and settle
	// step (4) G38_READ_DEPTH                   - issue read request to gauge
	// step (3) G38_WAIT_FOR_RESULTS             - if non touch probe, wait for results on canbus
	// step (2) G38_RAISE_PROBE                  - lift probe from surface
	// step (1) G38_MOVE_TO_NEXT_XY              - move to the next X-Y location (incr by I/J)
	// step (0) G38_FINISHED_MULTIPASS_PROBING   - all done; cleanup flags, etc

	MotorStructure *M;
	boolean save_IncrementalMove;
	if (AnyPotentialMotion())
	{   // waiting for prior step to complete before jumping on to the next move as ALL steps of this
		// multi-part homing sequence must be run single step (effectively avoiding the motionQ)
		return;
	}

	_motionQ_ForceQtoEmpty = TRUE; // force to single step ... needed to get around control issue with
								   // urgent commands bypassing regular flow control
	save_IncrementalMove = _IncrementalMove; // save state of _IncrementalMove to switch to Absolute Mode.
	ExecutionPtr = CannedCyclePtr; //restore ExecutionPtr to our index

	switch (CannedCycleStep)
	{ // step are listed in order of processing
	case G38_START_MULTIPASS_PROBING:
		initCannedCycleMove(38);
		InvalidateAllCmdArgs(); // start fresh, plug in numbers as needed
		break;
	case G28_ARM_PROBE:
		break;
	case G38_LOWER_PROBE:
		if (isAContactProbe(_Canned.probeType))
		{   // arm probe before move
			if (ArmTouchProbe(_Canned.probeNumber, _Canned.probeType))
			{   // safe to move
				_TouchProbeMoveActive = TRUE;
			}
		}


		if (ArmTouchProbe(_Canned.probeNumber, _Canned.probeType))
		{
			_TouchProbeMoveActive = TRUE;
			_IncrementalMove = TRUE;
			motionQ_addCommand(motionRate, NOT_HOMING_MOTORS);  // motionQ should be empty, so no need to check
			_IncrementalMove = _Canned.save_IncrementalMove;
		}
		break;
	case G38_PAUSE_BEFORE_READING:
		break;
	case G38_READ_DEPTH:
		break;
	case G38_WAIT_FOR_RESULTS:
		break;
	case G38_RAISE_PROBE:
		break;
	case G38_MOVE_TO_NEXT_XY:
		break;
	case G38_FINISHED_MULTIPASS_PROBING:
		cleanupAfterCannedCycleMove();
		break;
	default:
		break;
	}
	CannedCycleStep--;

	/*
	case G28_START_MULTIPASS_HOME:  // (7) initial state from G28 call; validate req axis has a home sensor
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors involved and make sure they have a home sensor; invalidate that axis otherwise
			if (motorArgPresent(M) && !M->HomeSense.Enabled)
			{   // no home sensor enabled, yet trying to home, so Error and remove from list
				 sprintf(_errorStr, "Trying to home %c axis which does not have a home sensor enabled", (int)M->AxisLabel);
				 sendError(_errorStr);
				 setMotorArg(M, INVALID_ARG_VALUE);
			}
		}
		break;
	case G28_PASS_MOVE_INTERMEDIATE_POINT:  // (6) rapid move to the optional XYZABC intermediate position
		_IncrementalMove = FALSE; // absolute move to the intermediate point
		// per NIST and Fanuc manual, the XYZABC specify intermediate points to go to prior to homing (possible safe position)
		if (ARG_I_PRESENT)  // GB XXX gregkarl G28: ARG_I not part of the spec, but allows testing of intermediate move
		{
			motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);   // move to the intermediate point.  motionQ should be empty, so no need to check
		}
		break;
	case G28_PASS_HOME:  // (5) homing from that intermediate position using HOMING_SPEED
		_IncrementalMove = FALSE;  // absolute move to home
		ExecuteHomingMove(HOMING_MOTION_RATE);  // note: speed will be scaled down such that we can safely decelerate in the "hysteresis" distance.
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and set the flag showing axis has been homed
			if (motorArgPresent(M))
			{
				M->HasBeenHomed = TRUE;
				M->HomeSense.ErrorMessageSent = FALSE;
			}
		}
		break;
	case G28_SET_DESTINATION_HOME:  // (4) set the POSITION based on homing direction, home offset, etc
		SetAllAxisHomePosition(TRUE);
		break;
	case G28_PASS_LEAVE_HOME:  // (3) move off home position by a small amount
		_IncrementalMove = TRUE;  // incremntal move off the sensor
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and restuff XYZABC with the pop-off amount to get past the sensor
			if (motorArgPresent(M))
			{
				setMotorArgInNativeUnits(M, (M->HomePopOffDistanceInUnits));
			}
		}
		motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);  // motionQ should be empty, so no need to check
		break;
	case G28_PASS_REHOME:  // (2) set the post-homing position
		_IncrementalMove = FALSE;  // absolute move to home
		ExecuteHomingMove(REHOMING_MOTION_RATE); // rehome at a slower rate
		break;
	case G28_SET_DESTINATION_REHOME:  // (1) set the POSITION based on homing direction, home offset, etc
		SetAllAxisHomePosition(TRUE);
		if (ARG_Z_PRESENT && Motors[M_Z].HomeSense.Enabled)
		{
			ToolOffsets[0].ToolLength = Motors[M_Z].HomeDestinationInUnits;
		}
		break;
	case G28_FINISHED_MULTIPASS_HOME:  // (0) all done; cleanup flags, etc
		_IncrementalMove = save_IncrementalMove;  // restore state of _IncrementalMove
		CannedCycleLastMove = TRUE;
		CannedCycleFlag = 0; // drop out the canned cycle loop
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors and set the flag showing axis has been homed
			if (motorArgPresent(M))
			{
				M->HasBeenHomed = TRUE;
				M->HomeSense.ErrorMessageSent = FALSE;
			}
		}
		break;
	default:
		break;
	}

	CannedCycleStep--;
		*/
}
#endif
////////////////////////////////////////////////////////////////////////////////

void sendAxisHomedStatus(MotorStructure *M)
{
	if (M->MotorInstalled)
	{
		sprintf(_tmpStr, "HomeState:%c:%s", M->AxisLabel, M->HomeSense.Enabled ? (M->HasBeenHomed ? "Homed" : "NotHomed") : "NoSensor");
		sendST(_tmpStr);
	}
}

void sendAllAxisHomedStatus(void)
{
	return; // ignore this call for now (output text is not used currently)
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		sendAxisHomedStatus(M);
	}
}

////////////////////////////////////////////////////////////////////////////////

char *getUIDString(char *s)
{
	int index;
	char shortStr[8];
	sprintf(s, "0x");
	for (index=0; index<12; index++)
	{
		sprintf(shortStr, "%02x", _sysInfoPtr->uniqueId[index]);
		strcat(s, shortStr);
		if ((index==3) || (index==7))
		{
			strcat(s, " 0x");
		}
	}
	return(s);
}

////////////////////////////////////////////////////////////////////////////////

boolean validFirmwareKey(boolean p)
{
	return TRUE;
}

void sendFakeM687CompleteCode(void)
{
	sprintf(_rptStr, ">MC:M687:%s%s", validFirmwareKey(0) ? "" : "INVALID-", _sysInfoPtr->lastKeyUsed);
#ifdef ALLOW_NATIVE_LIGHTBURN
						if (!_lightburnModeEnabled)
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstringCr(_rptStr);
}

////////////////////////////////////////////////////////////////////////////////
boolean isSpecialBuildMachine(void)
{
	return TRUE;
}
void sendRevisionString(char *reason)
{
	int index;
	// 0) status
	//    send axis status (homed/unhomed)
	//    M686 info (license key seed)
	//     M686 info (license key result)
	// connection string response order (each item is ':' delimited)
	// 1) reason for sending the revisionString ie, PingReply or Kill/Reset or the source of the hardware reset)
	// 2) sw revision
	// 3) mcu version and revision
	// 4) uid uint0 uint1 uint2
	// 5) current comm port master (UART3/UART4/UART6/USB/BOOTUP/unkUART)
	//    if BOOTUP a ping char needs to be sent to establish comm)
	//       (unkUART is a serious error condition)
	// then

	// send status info first as Repetrel is trigger happy when it sees the "Hy> ...." string and will often immediately reset or ping
	// hydra which would hit mid string on the M686
	sendAllAxisHomedStatus();
	M_Code_M686();  // force correct seed to populate in GUI
	sendFakeM687CompleteCode(); // force correct key to populate in GUI

	sprintf(_tmpStr,         ">Hy: %s :",  reason);
	sendstring(_tmpStr);

	//find the reporting format,
	for (index=0; index<NUM_SYSTEM_REPORT_INFO_TABLE_ENTRIES; index++)
	{
		if (_systemReportInfoTable[index].type == SEND_SYSTEM_REVISION_INFO) break;
	}
	if (index != NUM_SYSTEM_REPORT_INFO_TABLE_ENTRIES)
	{   // found index
//		if (isSpecialBuildMachine())
//		{
//			sprintf(_tmpStr, _systemReportInfoTable[index].formatStr, "KEYGEN" , _sysInfoPtr->softwareMajorVersion, _sysInfoPtr->softwareMinorVersion, _sysInfoPtr->softwareTweakVersion,
//					(_sysInfoPtr->softwareTweakVersion=='z')?_sysInfoPtr->softwareDebugVersion:' ', _sysInfoPtr->mcuDeviceID, _sysInfoPtr->mcuRevisionID);
//		}
//		else
		{
			sprintf(_tmpStr, _systemReportInfoTable[index].formatStr, PLATFORM_STRING, _sysInfoPtr->softwareMajorVersion, _sysInfoPtr->softwareMinorVersion, _sysInfoPtr->softwareTweakVersion,
					(_sysInfoPtr->softwareTweakVersion=='z')?_sysInfoPtr->softwareDebugVersion:' ', _sysInfoPtr->mcuDeviceID, _sysInfoPtr->mcuRevisionID);
		}
		sendstring(_tmpStr);
	}

	sendstring(" :uid ");
	sendstring(getUIDString(_tmpStr));

	switch(masterCommPort)
	{
	case UART3_MASTER : sprintf(_tmpStr, " :UART3"); break;
	case UART4_MASTER : sprintf(_tmpStr, " :UART4"); break;
	case UART6_MASTER : sprintf(_tmpStr, " :UART6"); break;
	case USB_MASTER   : sprintf(_tmpStr, " :USB");   break;
	case BOOTUP       : sprintf(_tmpStr, " :BOOTUP");  break;
	default           : sprintf(_tmpStr, " :unkUART"); break;
	}
	sendstringCr(_tmpStr);
}

////////////////////////////////////////////////////////////////////////////////

boolean rebootingFromCrash(void)
{
	return(
			//RCC_GetFlagStatus(RCC_FLAG_PINRST)   ||     // reset button
			RCC_GetFlagStatus(RCC_FLAG_IWDGRST)  ||     // independant watchdog
			RCC_GetFlagStatus(RCC_FLAG_SFTRST)
		   );        // software reset
}

////////////////////////////////////////////////////////////////////////////////

void setResetSouceStr(void)
{
	//#define RCC_FLAG_BORRST                  ((uint8_t)0x79)  // POR/PDR or BOR (power-on; power-down ; brown out)
	//#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)  // NRST reset
	//#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)  // POR/PDR reset
	//#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)  // software reset
	//#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)  // independent watchdog
	//#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)  // window watchdog
	//#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)  // low power reset
	//FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
	//
	//#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)
	//#define  RCC_CSR_BORRSTF                     ((uint32_t)0x02000000) // POR/PDR/BOR reset (power-on, power-down, brown-out)
	//#define  RCC_CSR_PADRSTF                     ((uint32_t)0x04000000) // nRTP pin reset
	//#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000) // POR/PDR reset
	//#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000) // software reset
	//#define  RCC_CSR_WDGRSTF                     ((uint32_t)0x20000000) // independent watchdog reset
	//#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000) // window watchdog reset
	//#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000) // low power reset
	//RCC_ClearFlag(); // writes RMVF -- which clears all RCC_SCR flags

	resetSourceStr[0] = '\0';

	 if (RCC_GetFlagStatus(RCC_FLAG_PORRST))
	 {   // normal power on
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "POR");
	 }
	 else if (RCC_GetFlagStatus(RCC_FLAG_BORRST))
	 {   // recovering from brownout
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "BRWNOUT");
	 }
	 else if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST))
	 {   // got here via a watchdog reset - crash recovery
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "IWDG");
	 }
	 else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST))
	 {    // got here via a software reset - crash recovery
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "SW");
	 }
	 else if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST))
	 {   // normal reset paths - nothing spec
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "WWDG");
	 }
	 else if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST))
	 {   // normal reset paths - nothing spec
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "LPOW");
	 }
	 else if (RCC_GetFlagStatus(RCC_FLAG_PINRST))
	 {   // reset button pressed -- should acknowledge
		 if (resetSourceStr[0] != '\0')  {strcat(resetSourceStr, "+");}   strcat(resetSourceStr, "nRST");
	 }
	 if (resetSourceStr[0] == '\0')  {strcat(resetSourceStr, "Unknown");}
	 strcat(resetSourceStr, " Reset");
}

////////////////////////////////////////////////////////////////////////////////

/*----------Symbols defined in linker script----------------------------------*/
extern unsigned long _sidata;    /*!< Start address for the initialization
									  values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */
extern unsigned long _edata;     /*!< End address for the .data section       */
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */
extern void _eram;               /*!< End address for ram                     */
#ifdef CCMRAM_USED
#warning "not used"
//extern unsigned long _siccmram;    /*!< Start address for the initialization
//									  values of the .ccmram section.            */
extern unsigned long _sccmram;     /*!< Start address for the .ccmram section     */
extern unsigned long _eccmram;     /*!< End address for the .ccmram section       */
#endif //CCMRAM_USED
extern int main(void);           /*!< The entry point for the application.    */

void Reset_Handler(void)
{
#ifdef ENABLE_CRASH_LOGGING
	SystemInit();    // Setup STM32 system (clock, PLL and Flash configuration) @ 168MHz
	if (rebootingFromCrash())
	{
		sendCrashDataFromRAMtoFlash(); // save a copy of key variables to flash
	}
#endif

	/* Initialize data and bss */
	unsigned long *pulSrc, *pulDest;

	/* Copy the data segment initializers from flash to SRAM */
	pulSrc = &_sidata;

	for(pulDest = &_sdata; pulDest < &_edata; )
	{
		*(pulDest++) = *(pulSrc++);
	}

#ifdef CCMRAM_USED
	/* Copy the ccmram segment initializers from flash to SRAM */
	pulSrc = &_siccmram;

	for(pulDest = &_sccmram; pulDest < &_eccmram; )
	{
		*(pulDest++) = *(pulSrc++);
	}
#endif //CCMRAM_USED

	/* Zero fill the bss segment.  This is done with inline assembly since this
	 will clear the value of pulDest if it is not kept in a register. */
	__asm("  ldr     r0, =_sbss\n"
			"  ldr     r1, =_ebss\n"
			"  mov     r2, #0\n"
			"  .thumb_func\n"
			"zero_loop:\n"
			"    cmp     r0, r1\n"
			"    it      lt\n"
			"    strlt   r2, [r0], #4\n"
			"    blt     zero_loop");

#ifdef __FPU_USED
	/* Enable FPU.*/
	__asm("  LDR.W R0, =0xE000ED88\n"
			"  LDR R1, [R0]\n"
			"  ORR R1, R1, #(0xF << 20)\n"
			"  STR R1, [R0]");
#endif

	setResetSouceStr(); // make a note of the path that led to this boot

	main();   // Call the application's entry point
}

////////////////////////////////////////////////////////////////////////////////

void wiggleDebugIO(int state)
{   // enable to quickly verify LA is connected properly
#ifdef WIGGLE_DEBUG_PINS_AT_BOOT_TIME
	if (state == 1)
	{
#if defined(USE_HYDRA_IO) && defined(USE_PANEL_6TO1_IO_FOR_DEBUG_PINS)
	GB_DEBUG_PIN1_SET;
	GB_DEBUG_PIN2_SET;
	GB_DEBUG_PIN3_SET;
	GB_DEBUG_PIN4_SET;
	GB_DEBUG_PIN5_SET;
	GB_DEBUG_PIN6_SET;
#endif //USE_PANEL_6TO1_IO_FOR_DEBUG_PINS
#if defined(USE_HYDRA_IO) && defined(USE_PANEL_ABSEL_IO_FOR_DEBUG_PINS)
	GB_DEBUG_PIN7_SET;
	GB_DEBUG_PIN8_SET;
	GB_DEBUG_PIN9_SET;
#endif
	}
	else
	{
#if defined(USE_HYDRA_IO) && defined(USE_PANEL_6TO1_IO_FOR_DEBUG_PINS)
	GB_DEBUG_PIN1_CLEAR;
	GB_DEBUG_PIN2_CLEAR;
	GB_DEBUG_PIN3_CLEAR;
	GB_DEBUG_PIN4_CLEAR;
	GB_DEBUG_PIN5_CLEAR;
	GB_DEBUG_PIN6_CLEAR;
#endif //USE_PANEL_6TO1_IO_FOR_DEBUG_PINS
#if defined(USE_HYDRA_IO) && defined(USE_PANEL_ABSEL_IO_FOR_DEBUG_PINS)
	GB_DEBUG_PIN7_CLEAR;
	GB_DEBUG_PIN8_CLEAR;
	GB_DEBUG_PIN9_CLEAR;
#endif
	}
#endif //WIGGLE_DEBUG_PINS_AT_BOOT_TIME
}

////////////////////////////////////////////////////////////////////////////////


void executeAnySpecialDebugStartCode(void)
{

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// WARNING:  IF THE ORDER OF ANY OF THESE CHANGE, PLEASE UPDATE initSliceTiming()
const PFUNC F1000HZ[NUM_1000HZ] =
{
	ohNoMrBill, // can't use slice 0 and this is time slot to execute the next slower slice
	serialProcessor,
	CommandProcessor, // can't move to foreground due to collision on global "ExecutionPtr"
	SequenceEngine, // controls a lot of ms increment timers -- MUST STAY IN 1000Hz loop
	canProcessRxQueueNoReturn,
	canProcessTxQueueNoReturn,
	motionQ_update,
	loop_1000Hz_simple_work, // keep as last call in this array
};

const PFUNC F100HZ[NUM_100HZ] =
{
	ohNoMrBill,
	// can't use slice 0 and this is time slot to execute the next slower slice
	initFromSoapstring,
	readInputs,
	checkMotorFaultSensor,
	checkMotorLimit1Sensor,
	checkMotorLimit2Sensor,
	latheSpeedControl,
	PnP_SetValves,
	LatchPnPData,
	loop_100Hz_simple_work,
	// keep as last call in this array
};

const PFUNC F10HZ[NUM_10HZ] =
{
	ohNoMrBill, // can't use slice 0 and this is time slot to execute the next slower slice
	soapstringController,
	sendUpdateToHost,
	checkBlockingWaits,
	EdgeTriggerSendResults,
	// move into simple_work if space needed
	checkForCompletedAbort,
	ReportXYZLocation,
	ProcessRawADC_Data,
	spare,
	loop_10Hz_simple_work, // keep as last call in this array
};

const PFUNC F1HZ[NUM_1HZ] =
{
	checkForMia,
	spare,
	ReportOsseoVariables,
	spare,
	spare,
	LaserSendRequestStringToPowerSupply,
	spare,
	spare,
	spare,
	loop_1Hz_simple_work, // keep as last call in this array
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	masterCommPort = BOOTUP;
	ExecutionPtr = &cmdQue[0];
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_VECTOR_OFFSET); // Set interrupt jump table location
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);              //4 bits for preemp priority 0 bit for sub priority
	InitGPIO();                 //configure direction and purpose of the I/O pin
	wiggleDebugIO(1);
	initAllHss();
	InitGlobals();              //initialize/clear CAN and MCODE globals
	DeInitLaser();
	InitializeMotorParameters();
	initMailbox();
	initKey(FALSE, _sysInfoPtr->lastKeyUsed, &GCodeArgComment[1]);     // must be after initMailbox; 	// tell the key checker where to put and find data
	InitTimer7();
	Init_ADC();
	__disable_irq();            //prevent any interrupts until everything is initialized

	wiggleDebugIO(0);
	InitAllUARTS(SYSTEM_BAUD_RATE);
//	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
	CAN_Config(CAN1);
	CAN_Config(CAN2);

	InitializeFixtureOffsets();

	ResetProcess(0);
	processSoapstringCommands=TRUE; // get ready to self initialize

	RCC_ClearFlag(); // clear flags indicating boot reason

	executeAnySpecialDebugStartCode();	//this call must remain here


	_heartbeatRateControl = HEARTBEAT_MODE_NORMAL;
	SysTick_Config(SystemCoreClock / SYSTICKS_PER_SECOND);//slice timer has lowest interrupt priority
	InitTim3RpmInput(); //set up the rpm counter
	__enable_irq();  // now everything is ready, so let interrupts occur

	//Init_SPI2();//used for pnp valve control on J21
	ConfigureTimer4PwmOutputsFor0_10V();//setup power control for laser and speed for spindle  TIM4-CCR3 and TIM4-CCR4
	InitTimer8();
	pinSet(TPIC_6595_CLR); //clear the output of the tpsic595, after power on and also abort char
	//timerInitEncoderAB(FALSE);  		// setup for GUI use
	Start_ADC();
	// USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
	Init_Display(&LCDSpi1, LCD_SPI_PORT, COLOR_MODE_INVERT);

	while (1)
	{
//		//just spin for now
		switch (DisplayIndex)
		{
			//case 0:UpdateScreen(&LCDSpi1, SecsVarsTable); break;
			//case 0:UpdateScreen(&LCDSpi1, SecsStringTable); break;
		case 0:UpdateScreen(&LCDSpi1, LcdVarsTable); break;
		case 1:UpdateScreen(&LCDSpi1, UsbGcodeArguments); break;
		case 2:UpdateScreen(&LCDSpi1, CMDQueValues); break;
		case 3:UpdateScreen(&LCDSpi1, TaskTimeTable1); break;
		case 4:UpdateScreen(&LCDSpi1, ADCValueTable); break;
		case 5:UpdateScreen(&LCDSpi1, BarValueTable); break;
		case 6:UpdateScreen(&LCDSpi1, FaultValueTable); break;
		case 7:UpdateScreen(&LCDSpi1, CanRxBufferTable); break;
		case 8:UpdateScreen(&LCDSpi1, CanTxBufferTable); break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////


