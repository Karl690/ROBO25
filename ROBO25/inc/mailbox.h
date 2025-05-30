#ifndef mailbox_HEADER // prevent double dipping
#define mailbox_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    mailbox.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains mailbox specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  mailbox specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

#define DEFAULT_HOST_TRAFFIC_REPORTING_PERIOD_MS        5000
#define DEFAULT_HOST_TRAFFIC_REPORTING_PERIOD_DECR_MS   800//karlchris 100 // currently in 10Hz loop, so 100ms per tick
#define DEFAULT_HOST_TRAFFIC_REPORTING_MASK             0x000001

#define DEFAULT_POSITION_REPORTING_PERIOD_MS        1000    // try to spread out load between status and position
#define DEFAULT_POSITION_REPORTING_PERIOD_DECR_MS   100 // currently in 10Hz loop, so 100ms per tick

#define ONE_MINUTE_IN_MS                    60000
#define DEFAULT_AUTO_STATUS_MASK            0x8f
#define DEFAULT_AUTO_STATUS_PERIOD_MS   1000 //period in MS

#define NUM_PHYSICAL_DEVICES                40  // 6 x 4 yokes +  2 x hotbed = 26 + 14 motors
#define NUM_LOGICAL_DEVICE					24	// 8 aliases + 7 broadcast + 7 motor broadcast + uplugged_head + junkmail
#define NUM_PHYSICAL_PLUS_LOGICAL_DEVICES   (NUM_PHYSICAL_DEVICES + NUM_LOGICAL_DEVICE)
#define DEFAULT_DEVICE                      13

#define FLASH_SIZE_REG_ADDR         0x1FFF7A22          // 32-bit reg
#define UNIQUE_ID_REG_ADDR          0x1FFF7A10          // three 32-bit regs (96-bit total)
#define OPTION_BYTES_ADDR           0x1FFFC000          // option bytes (16 bytes total)

#define FLASH_512_SOAP_SECTOR      FLASH_Sector_7
#define FLASH_512_SOAP_ADDR        (FLASH_BASE + (3 * 128 * 1024))
#define FLASH_512_SOAP_SIZE        (128 * 1024)

#define FLASH_1024_SOAP_SECTOR      FLASH_Sector_11
#define FLASH_1024_SOAP_ADDR        (FLASH_BASE + (7 * 128 * 1024))
#define FLASH_1024_SOAP_SIZE        (128 * 1024)

#define FLASH_2048_SOAP_SECTOR      FLASH_Sector_23
#define FLASH_2048_SOAP_ADDR        (FLASH_BASE + (15 * 128 * 1024))
#define FLASH_2048_SOAP_SIZE        (128 * 1024)

#define FLASH_512_CRASH_SECTOR      0 // no crashlog on 512KB parts
#define FLASH_512_CRASH_ADDR        0
#define FLASH_512_CRASH_SIZE        0

#define FLASH_1024_CRASH_SECTOR     FLASH_Sector_10
#define FLASH_1024_CRASH_ADDR       (FLASH_BASE + (6 * 128 * 1024))
#define FLASH_1024_CRASH_SIZE       (128 * 1024)

#define FLASH_2048_CRASH_SECTOR     FLASH_Sector_22
#define FLASH_2048_CRASH_ADDR       (FLASH_BASE + (14 * 128 * 1024))
#define FLASH_2048_CRASH_SIZE       (128 * 1024)

#define FLASH_SOAP_RESERVE_SIZE    (1 * 1024)   // fits in working buffer
#define FLASH_SOAP_RESERVE_OFFSET  (_sysInfoPtr->soapSize - FLASH_SOAP_RESERVE_SIZE)
#define FLASH_SOAP_RESERVE_ADDR    ((byte *)(_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET))

#define WORKING_BUFFER_BYTE_START_ADDRESS		((byte *)&_MailBoxes._workingBuffer[0])

typedef enum {
	SEND_DEVICE_REVISION_INFO       = 0,
	SEND_DEVICE_FLASH_CONFIG        = 1,
	SEND_DEVICE_UNIQUE_ID           = 2,
	SEND_DEVICE_ALIAS_LIST          = 3,
	SEND_DEVICE_PAGE_DEF            = 7,
	SEND_DEVICE_TABLE_START_OFFSETS = 8,
	SEND_DEVICE_CONTROL_WORD        = 10,
	SEND_DEVICE_PASSWORD            = 11,
	SEND_DEVICE_PAGE_CHECKSUM       = 12,
} deviceInfoType;

typedef enum {
	SEND_DEVICE_POSITION_TABLE      = 0,
	SEND_DEVICE_SOAP_STRING         = 7,
	SEND_DEVICE_RAW_PAGE_DATA       = 10
} tableInfoType;

typedef enum {
	SEND_SYSTEM_SOAP_ERASE          = 0,
	SEND_SYSTEM_SOAP_STRING         = 1,
	SEND_SYSTEM_OPTION_BYTES        = 2,
	SEND_SYSTEM_REVISION_INFO       = 3,
	SEND_SYSTEM_FLASH_CONFIG        = 4,
	SEND_SYSTEM_UNIQUE_ID           = 5,
	SEND_SYSTEM_PASSWORD            = 6,
	SEND_SYSTEM_SOAP_CONFIG         = 7,
	SEND_SYSTEM_CRASH_CONFIG        = 8
} systemInfoType;

typedef struct {
	float               PulsesPerUnit;          //number of pulses per microliter or mm, used to calculate actual flow rate in pulses per second
	float               ExtrusionWidthInMm;     // check your slicer, but this is usually close to the nozzle hole diameter
	float               SliceHeightInMm;        //used for flow calcualtions, this value is passed from the slicer via Mcode and used to

	float               ExtrusionRateOverridePct; //used to adjust flow in real time
	uint16_t            ExtrudeFeedRate;        //extrusion rate in pulses per second, no multiply, all mulitplication is done on the 407 or repetrel
	uint16_t            ExtrudeSteps;           //number of steps to extrude during normal extrusion print move, 0xffff means run continous

	uint16_t            PrimeFeedRate;          //speed in pulses per second to spin the motor during the prime
	uint16_t            PrimeSteps;             //number of pulses to spin at this rate before starting the normal extrusion
	int16_t             PrimeTimeMs;            //how long in ms to wait after issuing the prime before starting the move

	uint16_t            UnPrimeFeedRate;        //speed in pulses per second to spin the motor during the Unprime
	uint16_t            UnPrimeSteps;           //number of pulses to spin during Unprime, extrusion stops after this many pulses
	int16_t             UnPrimeTimeMs;      //When to Fire the unprime sequence in ms relative to the end of the move (neg means before)

	boolean             isManuallyExtruding;
} deviceExtrusionControlStruct;

typedef struct {
	deviceInfoType  type;
	char            header[32];         // string header to print at start of report
	char            abbreviation[8];    // short version of header
	char            formatStr[110];
} reportDeviceInfoStruct;

typedef struct {
	tableInfoType   type;
	char            header[32];         // string header to print at start of report
	char            abbreviation[8];    // short version of header
	char            formatStr[50];
} reportTableInfoStruct;

typedef struct {
	systemInfoType  type;
	char            header[32];         // string header to print at start of report
	char            abbreviation[8];    // short version of header
	char            formatStr[50];
} reportSystemInfoStruct;

typedef struct
{
	byte unit;
	byte code;
	char unitStr[16];
	char codeStr[50];
	char sprintfStr[50];
	byte numArgs;
	boolean argsAreTemperatures;
} errorDescriptionStruct;

#define NUM_DEVICE_ERROR_CODES 64
typedef struct {
	byte        unit;                   // info from last error reported
	byte        code;
	uint16_t    cnt;
	byte        params[4];
} errStruct;

typedef struct {
	uint16_t    adcRaw;
	int16_t     convRaw;
	uint16_t    adcAvg;
	int16_t     convAvg;
	uint16_t    history[8];
} adcReportStruct;

typedef struct {
	uint16_t    period;
	uint16_t    dutyCycle;
	uint16_t    currentCnt;
	byte        pinState;
	byte        invertSignal;
} switchReportStruct;

typedef struct {
	boolean     deviceRegistered;
	boolean     fromCAN2;
	canbusFormat_t  canbusFormat;
	byte        device;                                         // physical or logical (alias) device
	byte        commTicker;                                     // comm heartbeat to know device is still alive
	payloadUnion HBPayload[10];

	byte        preDefinedAliases[NUM_PRE_DEFINED_ALIASES];     // list of pre defined aliases as reported from HH
	byte        userDefinedAliases[NUM_USER_DEFINED_ALIASES];   // list of user defined aliases as reported from HH
	uint16_t    flashNumKBytes;                                 // devices total flash size (in KBytes)
	uint16_t    flashPageSize;                                  // size of each flash page
	uint32_t    flashBaseAddr;                                  // starting address for the the first page of flash
	soapDevType_t       deviceType;                                     // hardware device type as reported by device
	soapDevSubtype_t    deviceSubtype;
	soapPcb_t           devicePcb;
	soapRtd_t           deviceRtd1;
	soapRtd_t           deviceRtd2;
	devFam_t            deviceFamily;                                   // converted from deviceType via a routine common to the heads
	devCodebase_t softwareCodebase;                             // software codebase as reported by device
	byte        softwareMajorVersion;                           // software major revision as reported by device
	byte        softwareMinorVersion;                           // software minor revision as reported by device
	char        softwareTweakVersion;                           // software tweak revision as reported by device
	char        softwareDebugVersion;                           // software debug revision as reported by device

	byte        softwareCompileTarget;                          // index indicating which compiler target was used to create the hex file
	byte        uniqueId[12];                                   // device's 96-bit unique ID
	uint16_t    soapPage; // new for 4.003 and higher
	uint16_t    pageDef[4];   //LEGACY 4.002 and lower          // pages numbers for tables, soap, historyA, historyB
	byte        optionBytes[16];                                // copy of the 8 user option bytes (special flash mem) and their complements
	uint32_t    pageChecksum;                                   // checksum from last read_checksum command
	uint16_t    checksumedPage;                                 // which page was just read;
	controlWordUnion    controlWord;                                    // during HH heartbeat, current controlWord
	boolean     bootloaderRunning;                              // flag to indicate a reset was issued with the desire to start up the bootloader
	int         registrationStep;                               // used for tracking the registration progress
	uint16_t	motorTicksRev;									// used for heardbeat report conversion
} inboxStruct;

typedef struct {
	uint16_t    flashNumKBytes;                                 // devices total flash size (in KBytes)
	uint32_t    flashBaseAddr;                                  // starting address for the the first page of flash
	uint32_t    mcuDeviceID;                                    // mcu device ID
	uint32_t    mcuRevisionID;                                  // mcu device revision ID
	uint32_t    softwareMajorVersion;                           // software major revision
	uint32_t    softwareMinorVersion;                           // software minor revision
	char        softwareTweakVersion;                           // software tweak revision
	char        softwareDebugVersion;
	byte        uniqueId[12];                                   // system's 96-bit unique ID
	byte        optionBytes[16];                                // copy of the 8 user option bytes (special flash mem) and their complements

	uint32_t    soapBaseAddr;
	uint32_t    soapSector;
	uint32_t    soapSize;
	byte        *soapReadPtr;
	boolean     initFromReservedSoap;
	char        lastKeyUsed[33];

	uint32_t    crashlogBaseAddr;
	uint32_t    crashlogSector;
	uint32_t    crashlogSize;
	uint32_t    *crashlogReadPtr;
	uint32_t    *crashlogWritePtr;
} systemInfoStruct;

typedef struct {
	float           temperature;
	byte            dutyCycle;
	switchFlags_t   ctrl;
} switchTargets_t;

typedef struct outboxStruct {
	byte                device;
	devFam_t            deviceFamily;                                   // converted from deviceType via a routine common to the heads
	canbusFormat_t      canbusFormat;
	devInitStruct       *hardInitPtr;
	controlWordUnion    controlWord;
	motorEn_t			motorState;
	boolean             validConfiguration;
	deviceExtrusionControlStruct  ExtrusionControl;
	switchTargets_t     swTarg[HH_NUM_HIGH_SIDE_POWER_SWITCHES];
	uint32_t            autoStatusMask;
	uint16_t            autoStatusPeriodMs;
	byte                numClones;
	byte                clones[NUM_PHYSICAL_DEVICES];
} outboxStruct;//used to talk to extruder controllers via canbus

typedef enum {
	WAIT_FOR_NOTHING = 0,
	WAIT_FOR_ABSOLUTE_TEMP,
	WAIT_FOR_RELATIVE_TEMP,
} waitForMode_t;

typedef struct {
	byte            device;
	waitForMode_t   mode;
	int             minTemp;    // if absolute, then actual temp; if relative, then just deltaTemp
	int             maxTemp;    // if absolute, then actual temp; if relative, then just deltaTemp
} waitForStruct;

typedef struct {
	byte         	_incompatibleDeviceDetected[256];
	inboxStruct     _inbox[NUM_PHYSICAL_PLUS_LOGICAL_DEVICES];  //include logical as it's needed for processing with aliases (ie, to get swRevision)
	outboxStruct    _outbox[NUM_PHYSICAL_PLUS_LOGICAL_DEVICES];
	//NUKE outboxStruct    *_outboxPtr;
	byte            _workingBuffer[WORKING_BUFFER_SIZE_IN_BYTES];  // single buf for all incoming page data from device
	byte            _device2MailboxNum[256];
	byte            _mailboxNum2Device[NUM_PHYSICAL_PLUS_LOGICAL_DEVICES];      // extra location needed to park unknown devices
	byte            _remapDevicePositionTable[256]; //NUKE ???
	int             _hostTrafficReportingPrescaleCnt;               // prescaler for host reporting to get to once/second
	int             _hostTrafficReportingPeriodMs;
	int             _positionReportingPrescaleCnt;               // prescaler for host reporting to get to once/second
	int             _positionReportingPeriodMs;
	struct {
		union {
			uint32_t u32;
			struct
			{
				unsigned sliceNeedsExtraTime    : 1;    // MUST stay in first position (used by linked-in key gen lib too)
				unsigned hotbedTemp             : 1;
				unsigned chamberTemp            : 1;
				unsigned extruderTemp           : 1;
				unsigned hotheadFlashPageErase  : 1;
				unsigned prime                  : 1;
				unsigned canLoopback            : 1;
				unsigned bootloaderAnnounce     : 1;
				unsigned motionPnP              : 1;
				unsigned canbusProbeToArm       : 1;
				unsigned canAxisMotorHoming		: 1;
				unsigned canGuiAck				: 1;
			} bit;
		} flags;
		waitForStruct   extruder;
		waitForStruct   hotbed;
		waitForStruct   chamber;
		int             timerPnP;
		int             timerProbe;
		int				timerHoming;
		int             timerCanLoopback;
		int             timerCanGuiAck;
	} _waitingFor;
	uint32_t        _pageChecksum;
	uint64_t        stickyErrorMsg[NUM_DEVICE_ERROR_CODES];
	inboxStruct *lastRegisteringDeviceInboxPtr;
} mailboxStruct;

typedef struct {
	byte device;
	//boolean completeRegistration;
	int step;
} soapControl_t;

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in mailbox that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////

extern const errorDescriptionStruct _deviceErrorDescriptionTable[];
extern byte NUM_DEVICE_REPORT_STATUS_TABLE_ENTRIES;
extern byte NUM_REPORT_INBOX_TABLE_ENTRIES;
extern mailboxStruct _MailBoxes;
extern outboxStruct *currentOutboxPtr;
extern outboxStruct *_outboxHotbedPtr;
extern outboxStruct *_outboxCO2LaserPtr;
extern systemInfoStruct _sysInfo;
extern systemInfoStruct *_sysInfoPtr;
extern int DeviceSoapstringWatchdogCnt;
extern const reportSystemInfoStruct _systemReportInfoTable[];
extern byte NUM_SYSTEM_REPORT_INFO_TABLE_ENTRIES;;
extern boolean processSoapstringCommands;
extern int soapstringCommandWaiting;
extern boolean _KeepHeadsAliveDuringDebug;
extern int _LaserDebugReportingRate;

////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in mailbox
////////////////////////////////////////////////////////////////////////////////

extern byte remapDevicePosition(byte device);
extern byte getMailboxNum(byte);
extern boolean isAPhysicalDevice(byte);
extern boolean isALogicalDevice(byte);
extern void changeDevice(byte);
extern uint32_t convertUidToPassword(byte uniqueId[]);
extern void addDeviceToGroup(byte);
extern void removeDeviceFromGroup(byte);
extern void resetStickyErrorFlags(void);
extern void initMailbox(void);
extern void initSystemInfo(void);
extern byte *getNextWriteLocation(int);
extern void updateSoapWritePointers(void);
extern void sendUpdateToHost(void);
extern void tryToCleanUpGuiAfterDeviceRegistration(byte device);
extern boolean deviceRegistered(byte device);
extern inboxStruct *findNthRegisteredDevice(int nth);
extern void removeAllRegisteredDevice(void);
extern void checkForMia(void);
extern inboxStruct *getInboxPointer(byte);
extern outboxStruct *getOutboxPointer(byte);
extern uint32_t getWorkingBufferChecksum(byte);
extern boolean deviceSoftwareRevisionIsAtOrAbove(byte device, byte major, byte minor, char tweak);
extern boolean deviceSoftwareRevisionIsAtOrBelow(byte device, byte major, byte minor, char tweak);
extern void getDeviceRegistrationString(inboxStruct *, char *);
extern boolean validCanMotorDeviceAddress(byte device);
extern MotorStructure *getMotorPtrFromDeviceAddress(byte device);
extern int getCanMotorIndexFromDeviceAddress(byte device);
extern void startDeviceRegistration(canSwStruct *);
extern boolean matchesAnAlias(byte, inboxStruct *);;
extern void sendSingleLineReport(byte);
extern void sendInboxInfoToHost(deviceInfoType, byte);
extern boolean deviceIsAHotbed(byte);
extern boolean deviceIsACO2Laser(byte);
extern boolean deviceIsAUvLightRay(byte);
extern boolean deviceIsACanAxisMotor(byte device);
extern boolean deviceIsABttCanAxisMotor(byte device);
extern boolean deviceHasAClosedLoopStepper(byte device);
extern boolean deviceIsAFilamentDispenser(byte device);
extern uint16_t tableInfoTypeToPage(byte, tableInfoType, uint16_t);
extern void transferSystemInfoFromHost(systemInfoType selection);
extern void transferSystemInfoToHost(systemInfoType selection);
extern void transferDevicePageDataFromDeviceToWorkingBuffer(byte, uint16_t);
extern void transferDevicePageDataFromWorkingBufferToHost(byte, tableInfoType);
extern boolean transferDevicePageDataFromHostToWorkingBuffer(byte, tableInfoType);
extern void transferDevicePageDataFromWorkingBufferToDevice(byte, uint16_t);
extern void transferDevicePageDataFromHost(byte, tableInfoType, uint16_t);
extern void transferDevicePageDataToHost(byte, tableInfoType, uint16_t);
extern boolean waitingForHotbedTemp(void);
extern boolean waitingForExtruderTemp(void);
extern boolean waitingForChamberTemp(void);
extern void writeSystemSoapString(int);
extern void initFromSoapstring(void);
extern void eraseSystemSoapString(void);
extern void soapstringController(void);
extern void resetSoapstringControl(void);
extern void eraseSystemCrashLog(void);
extern void writeSystemCrashLog(uint32_t *, uint32_t);
extern void sendCrashDataFromRAMtoFlash(void);
extern void sendCrashDataFromFlash(void);
extern void sendCrashDataFromRAM(void);

#endif // #ifndef mailbox_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
