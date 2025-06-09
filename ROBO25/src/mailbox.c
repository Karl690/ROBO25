////////////////////////////////////////////////////////////////////////////////
//
// File:    mailbox.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: The module contains the basic messaging system to/from the hotheads
//          originally, all communication was to pass through these methods, but
//          over time this has change, which much of the communication going
//          around the mailbox system.
//
//          This is a candidate for a major overhaul to get to a consistant state.
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "Hydra_can.h"
#include "Serial.h"
#include "mailbox.h"
#include "gpio.h"
#include "headSetup.h"
#include "MotorDriver.h"
#include "GCode.h"

// from gcode.h
extern byte PersistantHotheadAddress;
extern byte PersistantHotbedAddress;

void readDeviceSoapString(byte);

////////////////////////////////////////////////////////////////////////////////
//  Local #defines (defines ONLY used in this module)
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Public global definitions (exposed in mailbox.h)
////////////////////////////////////////////////////////////////////////////////

mailboxStruct _MailBoxes;               //current working mailbox
outboxStruct *currentOutboxPtr=NULL;      //current output mail box pointer
outboxStruct *_outboxHotbedPtr; //working buffer for the hotbed controller outbox?
outboxStruct *_outboxCO2LaserPtr; //shortcut to outbox struct for the the co2 laser controller
outboxStruct *_primaryHeadPtr; //shortcut to outbox struct for the the co2 laser controller

//							(payload->i16[0] / field0scale), 		// temperature or rawadc
//							payload->i16[1], 						// HSS2 duty cycle
//							(payload->i16[2] / TEMP_SCALE), 		// ambient temp
//							payload->i16[3]);						// HSS1 duty cycle

systemInfoStruct _sysInfo;
systemInfoStruct *_sysInfoPtr;

int DeviceSoapstringWatchdogCnt=0;
int DeviceSoapstringWatchdogWatcher=0;

boolean sendReportToHost=FALSE;
boolean processSoapstringCommands=FALSE;
int processSoapstringCommandsTurnOffCounter=0;
int soapstringCommandWaiting=0;
#ifdef GB_DEBUGGING_KEEP_HEADS_ALIVE
boolean _KeepHeadsAliveDuringDebug=TRUE;
#else
boolean _KeepHeadsAliveDuringDebug=FALSE;
#endif
int _LaserDebugReportingRate=0;

////////////////////////////////////////////////////////////////////////////////
//  Local global definitions (do not expose in mailbox.h)
////////////////////////////////////////////////////////////////////////////////

// this table enumerates all of the POSSIBLE valid devices in the system and the index in the table is
// the mailbox number for that device


const errorDescriptionStruct _deviceErrorDescriptionTable[NUM_DEVICE_ERROR_CODES] =
{
		// make sure total entries are less than MAX_DEVICE_ERROR_CODES (increase MAX_DEVICE_ERROR_CODES otherwise)

		{ ERROR_UNIT_CAN, ERROR_SWITCH_MSG_TYPE, "CAN Unit", "Invalid msgType", "- msgType=%d msgId=0x%02x page=%d numBytes=%d", 4, FALSE},
		{ ERROR_UNIT_CAN, ERROR_SWITCH_MSG_ID, "CAN Unit", "Invalid msgId", "- msgType=%d msgId=0x%02x page=%d numBytes=%d", 4, FALSE},
		{ ERROR_UNIT_CAN, ERROR_BAD_PAGE_NUM, "CAN Unit", "Invalid page", "- msgType=%d msgId=0x%02x page=%d numBytes=%d", 4, FALSE},
		{ ERROR_UNIT_CAN, ERROR_NUM_BYTES_MISMATCH, "CAN Unit", "Invalid number of bytes", "- msgType=%d msgId=0x%02x expected=%d got=%d", 4, FALSE},
		{ ERROR_UNIT_CAN, ERROR_RX_QUEUE_FULL, "CAN Unit", "Receive queue full", "", 0, FALSE},
		{ ERROR_UNIT_CAN, ERROR_TX_QUEUE_FULL, "CAN Unit", "Transmit queue full", "", 0, FALSE},
		{ ERROR_UNIT_CAN, ERROR_TOO_MANY_ALIASES, "CAN Unit", "Max aliases reached", "- newAlias=%d", 1, FALSE},
		{ ERROR_UNIT_CAN, ERROR_BAD_CHECKSUM, "CAN Unit", "Buffer checksum does not match", "- checksum=%d", 1, FALSE},
		{ ERROR_UNIT_CAN, ERROR_PAGE_WRITE_FAILED, "CAN Unit", "Write/verify failed", "- page=%d", 1, FALSE},
		{ ERROR_UNIT_CAN, ERROR_COMM_TIMEOUT, "CAN Unit", "Comm from system timed out", "", 0, FALSE},
		{ ERROR_UNIT_CAN, ERROR_GUI_QUEUE_FULL, "CAN Unit", "Display ontrol queue full", "", 0, FALSE},

		{ ERROR_UNIT_ADC, ERROR_EXCEEDED_TABLE_LENGTH, "ADC Unit", "Ran past end of table", "- tableAddr=0x%08x", 1, FALSE},
		{ ERROR_UNIT_ADC, ERROR_ADC_EXCEEDED_RANGE, "ADC Unit", "ADC value exceeds range", "- tableAddr=0x%08x", 1, FALSE},
		{ ERROR_UNIT_ADC, ERROR_ADC_RTD_EXPECTED, "ADC Unit", "RTD SOAP definition expected", "- type=%c subtype=%c RTD%d=%c", 4, FALSE},

		{ ERROR_UNIT_FLASH, ERROR_ERASE_FAILED, "FLASH Unit", "Erase page failed", "- page=%d status=%d", 2, FALSE},
		{ ERROR_UNIT_FLASH, ERROR_MISSING_TABLES, "FLASH Unit", "Tables not initialized", "", 0, FALSE},

		{ ERROR_UNIT_LOOP, ERROR_BAD_VECTOR, "LOOP Unit", "Bad vector", "- 1000Hz=%d 100Hz=%d 10Hz=%d 1Hz=%d", 4, FALSE},
		{ ERROR_UNIT_LOOP, ERROR_SLICE_OVERRUN, "LOOP Unit", "Slice overrun", "- 1000Hz=%d 100Hz=%d 10Hz=%d 1Hz=%d", 4, FALSE},

		{ ERROR_UNIT_DEVICE, ERROR_LASER_NO_WATER_PRESSURE, "DEVICE Unit", "Laser has no water pressure", "", 0, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_LASER_INTERLOCK_OPEN, "DEVICE Unit", "Laser interlock open", "", 0, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_LASER_IS_OVER_TEMP, "DEVICE Unit", "Laser over temperature", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_DEVICE, ERROR_INTERNAL_SHUTDOWN_REQUESTED, "DEVICE Unit", "Shutdown pending", "- deviceType=0x%02x deviceSubtype=0x%02x (%d/%d)", 4, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_MANUAL_SWITCH_FAILED, "DEVICE Unit", "Manual switch failed", "", 0, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_ILLEGAL_HSS_CONTROL, "DEVICE Unit", "Illegal HSS control", "", 0, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_CONFIG_SAVE_FAILED_NO_SPACE, "DEVICE Unit", "Calib/Config Block Full", "", 0, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_SPI_ENCODER_ERROR, "DEVICE Unit", "Encoder SPI Comm Lost", "", 0, FALSE},
		{ ERROR_UNIT_DEVICE, ERROR_SPI_LCD_ERROR, "DEVICE Unit", "LCD SPI Comm Lost", "", 0, FALSE},

		{ ERROR_UNIT_HEATER, ERROR_MATERIAL_IS_OVER_TEMP, "HEATER Unit", "Heater over temperature limit", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_HOTBED_IS_OVER_TEMP, "HEATER Unit", "Hotbed over temperature limit", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_CHAMBER_IS_OVER_TEMP, "HEATER Unit", "Chamber over temperature limit", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_HEATER_SET_TEMP_TOO_HIGH, "HEATER Unit", "Heater set temp is too high", "- requestTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_MATERIAL_IS_UNDER_TEMP, "HEATER Unit", "Chiller under temperature limit", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_HOTBED_IS_UNDER_TEMP, "HEATER Unit", "Coldbed under temperature limit", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_CHAMBER_IS_UNDER_TEMP, "HEATER Unit", "Chamber under temperature limit", "- actualTemp=%2.1f limit=%2.1f", 2, TRUE},
		{ ERROR_UNIT_HEATER, ERROR_CHILLER_SET_TEMP_TOO_LOW, "HEATER Unit", "Chiller set temp is too low", "- requestTemp=%2.1f limit=%2.1f", 2, TRUE},

		{ ERROR_UNIT_MOTION, ERROR_MAX_ANGLE_ERROR_EXCEEDED, "MOTION Unit", "Exceeded Angle Error", "- error=%3.2f", 1, FALSE},

		{ ERROR_UNIT_UART, ERROR_RX_BUFFER_FULL, "UART Unit", "Rx Buffer Full", "", 0, FALSE},

		{ ERROR_UNIT_INIT, ERROR_EXCEEDED_ADC_ARRAY_SIZE, "INIT Unit", "Exceeded ADC array size", "- deviceType=%d", 1, FALSE},
		{ ERROR_UNIT_INIT, ERROR_ILLEGAL_DEV_TYPE_AND_PCB_COMBO, "INIT Unit", "Illegal device+PCB combo", "- type=%c subtype=%c PCB=%c RTD=%c", 4, FALSE},
		{ ERROR_UNIT_INIT, ERROR_UNKNOWN_SOAP_DEVICE_TYPE, "INIT Unit", "Soapstring deviceType not initialized", "- deviceType=0x%02x", 1, FALSE},
		{ ERROR_UNIT_INIT, ERROR_UNKNOWN_SOAP_DEVICE_SUBTYPE, "INIT Unit", "Soapstring subtype not initialized", "- deviceSubtype=0x%02x", 1, FALSE},
		{ ERROR_UNIT_INIT, ERROR_UNKNOWN_SOAP_PCB_TYPE, "INIT Unit", "Soapstring PCB not initialized", "- PCB=0x%02x", 1, FALSE},
		{ ERROR_UNIT_INIT, ERROR_UNKNOWN_SOAP_RTD_TYPE, "INIT Unit", "Soapstring RTD not initialized", "- RTD=0x%02x", 1, FALSE},
		{ ERROR_UNIT_INIT, ERROR_UNKNOWN_MISSING_RTD_POINTER, "INIT Unit", "Expected RTD for", "- devType=0x%02x RTD=0x%02x" , 2, FALSE},
};
//uint16_t NUM_DEVICE_ERROR_CODES = sizeof(_deviceErrorDescriptionTable) / sizeof(errorDescriptionStruct);

////////////////////////////////////////////////////////////////////////////////

const reportDeviceInfoStruct _deviceReportInfoTable[] = {
		{ SEND_DEVICE_REVISION_INFO,        "Device Info",          "INF",  "not used" },
		{ SEND_DEVICE_FLASH_CONFIG,         "Flash Config",         "FLC",  "numKbytes=%dKB pageSize=%d baseAddress=0x%08x" },
		{ SEND_DEVICE_UNIQUE_ID,            "Unique ID",            "UID",  "%02x " },
		{ SEND_DEVICE_ALIAS_LIST,           "Alias List",           "AKA",  "%d " },
		{ SEND_DEVICE_PAGE_DEF,          "Page Definitions",     "PAG",  "soap=%d" },
		{ SEND_DEVICE_TABLE_START_OFFSETS,  "Table Start Offsets",  "OFS",  "position=%d rtd1=%d rtd2=%d rtd3=%d tempSensor=%d temp2led=%d pulseTrain=%d soap=%d hist=%d raw=%d" },
//        { SEND_DEVICE_OPTION_BYTES,         "Option Bytes,",        "OPB",  "%02x "},
		{ SEND_DEVICE_CONTROL_WORD,         "Control word,",        "C_W",  "0x%08x"},
		{ SEND_DEVICE_PASSWORD,             "Password,",            "P_W",  "%d/0x%08x"},
		{ SEND_DEVICE_PAGE_CHECKSUM,        "Page Checksum",        "PCS",  "page=%d checksum=%d/0x%08x" }
};

byte NUM_DEVICE_REPORT_INFO_TABLE_ENTRIES = sizeof(_deviceReportInfoTable) / sizeof(reportDeviceInfoStruct);

////////////////////////////////////////////////////////////////////////////////

const reportTableInfoStruct _deviceReportPageInfoTable[] = {
//        { SEND_DEVICE_POSITION_TABLE,       "Position Table",       "POS",  "%d %d " },
//        { SEND_DEVICE_RTD1_TABLE,           "RTD1 Table",           "R1T",  "%d %d " },
//        { SEND_DEVICE_RTD2_TABLE,           "RTD2 Table",           "R2T",  "%d %d " },
//        { SEND_DEVICE_RTD3_TABLE,           "RTD3 Table",           "R3T",  "%d %d " },
//        { SEND_DEVICE_TEMP_SENSOR_TABLE,    "Temp Sensor Table",    "TST",  "%d %d  " },
//        { SEND_DEVICE_TEMP_2_LED_TABLE,     "Temp to LED Table",    "T2L",  "%d %d %d " },
//        { SEND_DEVICE_PULSE_TRAIN_TABLE,    "Pulse Train Table",    "PTR",  "%lld " },
		{ SEND_DEVICE_SOAP_STRING,          "Soap string",          "SOP",  "" },
//        { SEND_DEVICE_HISTORY0_DATA,        "History0 Data",        "HS0",  "%d " },
//        { SEND_DEVICE_HISTORY1_DATA,        "History1 Data",        "HS1",  "%d " },
		{ SEND_DEVICE_RAW_PAGE_DATA,        "Raw Page Data",        "RAW",  "%02x" }
};

byte NUM_DEVICE_REPORT_PAGE_INFO_TABLE_ENTRIES = sizeof(_deviceReportPageInfoTable) / sizeof(reportTableInfoStruct);

////////////////////////////////////////////////////////////////////////////////

const reportSystemInfoStruct _systemReportInfoTable[] = {
		{ SEND_SYSTEM_REVISION_INFO,    "System Info",          "INF",      "%s_%d.%03d%c%c :mcu 0x%03x 0x%04x" },
		{ SEND_SYSTEM_FLASH_CONFIG,     "Flash Config",         "FCF",      "numKbytes=%d baseAddr=0x%08x" },
		{ SEND_SYSTEM_UNIQUE_ID,        "Unique ID",            "UID",      "%02x " },
		{ SEND_SYSTEM_OPTION_BYTES,     "Option Bytes,",        "OPB",      "%02x "},
		{ SEND_SYSTEM_PASSWORD,         "Password,",            "P_W",      "%d/0x%08x"},
		{ SEND_SYSTEM_SOAP_STRING,      "System Soapstring",    "SOP",      "%s" },
		{ SEND_SYSTEM_SOAP_CONFIG,      "Soap Config",          "SCF",      "numKbytes=%d baseAddr=0x%08x sector=%d" },
		{ SEND_SYSTEM_CRASH_CONFIG,     "Crash Config",         "CCF",      "numKbytes=%d baseAddr=0x%08x sector=%d" }
};

byte NUM_SYSTEM_REPORT_INFO_TABLE_ENTRIES = sizeof(_systemReportInfoTable) / sizeof(reportSystemInfoStruct);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if 1 //NUKE?? SAVE FOR NOW
void changeOutgoingDevicePositionRemappingTable(byte device, byte alias)
{
	_MailBoxes._remapDevicePositionTable[device] = alias;
}

////////////////////////////////////////////////////////////////////////////////

byte remapDevicePosition(byte device)
{
	return(_MailBoxes._remapDevicePositionTable[device]);
}
#endif
////////////////////////////////////////////////////////////////////////////////

byte getMailboxNum(byte device)
{
	byte mailbox;

	mailbox = _MailBoxes._device2MailboxNum[device];
	if (mailbox >= NUM_PHYSICAL_PLUS_LOGICAL_DEVICES)
	{
		sprintf(_errorStr, "PROG ERR IN getMailboxNum dev=%d mbox=%d max=%d", device, mailbox, NUM_PHYSICAL_PLUS_LOGICAL_DEVICES);
		sendError(_errorStr);
		return(NUM_PHYSICAL_PLUS_LOGICAL_DEVICES);  // use extra location to dump invalid devices
	}
	return(mailbox);
}

////////////////////////////////////////////////////////////////////////////////

inboxStruct *getInboxPointer(byte device)
{
	return(&_MailBoxes._inbox[getMailboxNum(device)]);
}

////////////////////////////////////////////////////////////////////////////////

outboxStruct *getOutboxPointer(byte device)
{
	return(&_MailBoxes._outbox[getMailboxNum(device)]);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t getWorkingBufferChecksum(byte device)
{
	return(calculateChecksum32((uint32_t *)_MailBoxes._workingBuffer, getInboxPointer(device)->flashPageSize/4));
}

////////////////////////////////////////////////////////////////////////////////

uint32_t convertUidToPassword(byte uniqueId[])
{
	return((uniqueId[4] << 16) | (uniqueId[2] << 8) | uniqueId[0]);
}

////////////////////////////////////////////////////////////////////////////////

boolean isAPhysicalDevice(byte device)
{
	return(getMailboxNum(device) < NUM_PHYSICAL_DEVICES);
}
////////////////////////////////////////////////////////////////////////////////

boolean isALogicalDevice(byte device)
{
	return(getMailboxNum(device) >= NUM_PHYSICAL_DEVICES);
}

////////////////////////////////////////////////////////////////////////////////

void changeDevice(byte device)
{//this will specify which device is currently active
	if (((device >= HH_HOTBED_POSITION_MIN) && (device <= HH_HOTBED_POSITION_MAX)) || (device == HH_HOTBED_POSITION_ALL))  // deviceIsAHotbed(device)
	{
		_outboxHotbedPtr = getOutboxPointer(device);
		PersistantHotbedAddress = device;
	}
	else
	{
		currentOutboxPtr = getOutboxPointer(device);
		PersistantHotheadAddress = device;
	}
}


////////////////////////////////////////////////////////////////////////////////

boolean deviceIsACO2Laser(byte device)
{
	boolean co2Laser = (device >= HH_CO2_LASER_POSITION_MIN) && (device <= HH_CO2_LASER_POSITION_MAX);
	co2Laser &= (getInboxPointer(device)->deviceFamily == DEVICE_FAMILY_LASER);
	co2Laser &= deviceTypeIsACO2Laser(getInboxPointer(device)->deviceType);
	return(co2Laser);
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceIsAUvLightRay(byte device)
{
	return(getInboxPointer(device)->deviceType == SOAP_DEV_TYPE_UVLIGHT_RAY);
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceIsAHotbed(byte device)
{
	return(((device >= HH_HOTBED_POSITION_MIN) && (device <= HH_HOTBED_POSITION_MAX)) || (device == HH_HOTBED_POSITION_ALL));
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceIsACanAxisMotor(byte device)
{
	return(getInboxPointer(device)->deviceType == SOAP_DEV_TYPE_CAN_MOTOR);
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceIsABttCanAxisMotor(byte device)
{
	return((getInboxPointer(device)->devicePcb == SOAP_PCB_BTT));
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceHasAClosedLoopStepper(byte device)
{
	soapPcb_t pcb = getInboxPointer(device)->devicePcb;
	return((pcb == SOAP_PCB_BTT) || (pcb == SOAP_PCB_405_ENC_PWM) || (pcb == SOAP_PCB_405_ENC_DAC));
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceIsAFilamentDispenser(byte device)
{
	return(getInboxPointer(device)->deviceType == SOAP_DEV_TYPE_FIL_DISP);
}

////////////////////////////////////////////////////////////////////////////////

void initInboxStruct(inboxStruct *inboxPtr)
{
	byte i;

	bzero(inboxPtr, sizeof(inboxStruct));               // clear structure to all zeros

	for (i=0; i<NUM_PRE_DEFINED_ALIASES; i++)           // set all aliases to unused
	{
		inboxPtr->preDefinedAliases[i] = ALIAS_UNUSED;
	}
	for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
	{
		inboxPtr->userDefinedAliases[i] = ALIAS_UNUSED;
	}
	inboxPtr->canbusFormat = CANBUS_FORMAT_V1; // set default
	inboxPtr->motorTicksRev = CAN_MOTOR_LEGACY_TICKS_PER_REV; 	// for backward compatibility with original closed loop motor control firmware
}

////////////////////////////////////////////////////////////////////////////////

void initSystemInfo(void)
{
	int i;

	bzero(&_sysInfo, sizeof(systemInfoStruct));
	_sysInfoPtr = &_sysInfo;

	_sysInfoPtr->flashNumKBytes         = *(uint16_t *)FLASH_SIZE_REG_ADDR;                         // devices total flash size (in KBytes)
	_sysInfoPtr->flashBaseAddr          = FLASH_BASE;                                               // starting address for the the first page of flash
	_sysInfoPtr->mcuDeviceID            = DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID;
	_sysInfoPtr->mcuRevisionID          = ((DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID) >> 16) & 0xffff;
	_sysInfoPtr->softwareMajorVersion   = SOFTWARE_MAJOR_REVISION;                                  // software major revision
	_sysInfoPtr->softwareMinorVersion   = SOFTWARE_MINOR_REVISION;                                  // software minor revision
	_sysInfoPtr->softwareTweakVersion   = SOFTWARE_TWEAK_REVISION;                                  // software really minor revision
	_sysInfoPtr->softwareDebugVersion   = SOFTWARE_DEBUG_REVISION;
	for (i=0; i<12; i++)
	{
		_sysInfoPtr->uniqueId[i]    = *(byte *)(UNIQUE_ID_REG_ADDR + i);                            // devices' 96-bit unique ID
	}
	for (i=0; i<16; i++)
	{
		_sysInfoPtr->optionBytes[i] = *(byte *)(OPTION_BYTES_ADDR + i);                               // 8 option bytes plus complements
	}
	switch(_sysInfoPtr->flashNumKBytes)
	{   // assumes 1MB 429 parts are used in single bank mode to match 407 (see mcuDeviceID in DBGMCU reg)
	case 512:
		_sysInfoPtr->soapSector       = FLASH_512_SOAP_SECTOR;
		_sysInfoPtr->soapBaseAddr     = FLASH_512_SOAP_ADDR;
		_sysInfoPtr->soapSize         = FLASH_512_SOAP_SIZE;
		_sysInfoPtr->crashlogSector   = 0;
		_sysInfoPtr->crashlogBaseAddr = 0;
		_sysInfoPtr->crashlogSize     = 0;
		break;
	case 1024:
		_sysInfoPtr->soapSector       = FLASH_1024_SOAP_SECTOR;
		_sysInfoPtr->soapBaseAddr     = FLASH_1024_SOAP_ADDR;
		_sysInfoPtr->soapSize         = FLASH_1024_SOAP_SIZE;
		_sysInfoPtr->crashlogSector   = FLASH_1024_CRASH_SECTOR;
		_sysInfoPtr->crashlogBaseAddr = FLASH_1024_CRASH_ADDR;
		_sysInfoPtr->crashlogSize     = FLASH_1024_CRASH_SIZE;
		break;
	case 2048:
		_sysInfoPtr->soapSector       = FLASH_2048_SOAP_SECTOR;
		_sysInfoPtr->soapBaseAddr     = FLASH_2048_SOAP_ADDR;
		_sysInfoPtr->soapSize         = FLASH_2048_SOAP_SIZE;
		_sysInfoPtr->crashlogSector   = FLASH_2048_CRASH_SECTOR;
		_sysInfoPtr->crashlogBaseAddr = FLASH_2048_CRASH_ADDR;
		_sysInfoPtr->crashlogSize     = FLASH_2048_CRASH_SIZE;
		break;
	default:  // UNKNOWN SIZE
		_sysInfoPtr->soapSector       = 0;
		_sysInfoPtr->soapBaseAddr     = 0;
		_sysInfoPtr->soapSize         = 0;
		_sysInfoPtr->crashlogSector   = 0;
		_sysInfoPtr->crashlogBaseAddr = 0;
		_sysInfoPtr->crashlogSize     = 0;
		sendError("UNKNOWN FLASH SIZE!!!!! FATAL ERROR (SOAPSTRING UNAVAILABLE");
		break;
	}
	_sysInfoPtr->soapReadPtr = (byte *)_sysInfoPtr->soapBaseAddr;

	_sysInfoPtr->initFromReservedSoap = FALSE;
	strcpy(_sysInfoPtr->lastKeyUsed, "NO_KEY_FOUND");

	_sysInfoPtr->crashlogReadPtr = (uint32_t *)_sysInfoPtr->crashlogBaseAddr;
	_sysInfoPtr->crashlogWritePtr = (uint32_t *)_sysInfoPtr->crashlogBaseAddr;
}
////////////////////////////////////////////////////////////////////////////////

void resetStickyErrorFlags(void)
{
	int i;
	for (i=0; i<NUM_DEVICE_ERROR_CODES; i++)
	{
		_MailBoxes.stickyErrorMsg[i] = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

void initMailbox(void)
{
	uint16_t i;

	// assign each valid device number in the system to a mailbox;
	// list AA physical devices first (before any logical devices/groups

	initSystemInfo();

	bzero(&_MailBoxes, sizeof(mailboxStruct));

	_MailBoxes._mailboxNum2Device[0]  = 11;
	_MailBoxes._mailboxNum2Device[1]  = 12;
	_MailBoxes._mailboxNum2Device[2]  = 13;
	_MailBoxes._mailboxNum2Device[3]  = 14;
	_MailBoxes._mailboxNum2Device[4]  = 15;
	_MailBoxes._mailboxNum2Device[5]  = 16;
	_MailBoxes._mailboxNum2Device[6]  = 21;
	_MailBoxes._mailboxNum2Device[7]  = 22;
	_MailBoxes._mailboxNum2Device[8]  = 23;
	_MailBoxes._mailboxNum2Device[9]  = 24;
	_MailBoxes._mailboxNum2Device[10] = 25;
	_MailBoxes._mailboxNum2Device[11] = 26;
	_MailBoxes._mailboxNum2Device[12] = 31;
	_MailBoxes._mailboxNum2Device[13] = 32;
	_MailBoxes._mailboxNum2Device[14] = 33;
	_MailBoxes._mailboxNum2Device[15] = 34;
	_MailBoxes._mailboxNum2Device[16] = 35;
	_MailBoxes._mailboxNum2Device[17] = 36;
	_MailBoxes._mailboxNum2Device[18] = 41;
	_MailBoxes._mailboxNum2Device[19] = 42;
	_MailBoxes._mailboxNum2Device[20] = 43;
	_MailBoxes._mailboxNum2Device[21] = 44;
	_MailBoxes._mailboxNum2Device[22] = 45;
	_MailBoxes._mailboxNum2Device[23] = 46;
	_MailBoxes._mailboxNum2Device[24] = 91;
	_MailBoxes._mailboxNum2Device[25] = 92;

	_MailBoxes._mailboxNum2Device[26] = 60; //X.0
	_MailBoxes._mailboxNum2Device[27] = 61; //X.1
	_MailBoxes._mailboxNum2Device[28] = 62; //Y.0
	_MailBoxes._mailboxNum2Device[29] = 63; //Y.1
	_MailBoxes._mailboxNum2Device[30] = 64; //Z.0
	_MailBoxes._mailboxNum2Device[31] = 65;	//Z.1
	_MailBoxes._mailboxNum2Device[32] = 66; //Z.2
	_MailBoxes._mailboxNum2Device[33] = 67; //Z.3
	_MailBoxes._mailboxNum2Device[34] = 68; //A.0
	_MailBoxes._mailboxNum2Device[35] = 69; //A.1
	_MailBoxes._mailboxNum2Device[36] = 70; //B.0
	_MailBoxes._mailboxNum2Device[37] = 71; //B.1
	_MailBoxes._mailboxNum2Device[38] = 72; //C.0
	_MailBoxes._mailboxNum2Device[39] = 73; //C.1

	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+0] = 0;      // broadcast to all devices
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+1] = 100;    // broadcast to all extruder devices
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+2] = 90;     // broadcast to all hotbed devices
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+3] = 10;     // broadcast to all on yoke 1
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+4] = 20;     // broadcast to all on yoke 2
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+5] = 30;     // broadcast to all on yoke 3
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+6] = 40;     // broadcast to all on yoke 4
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+7] = 80;		// broadcast to all canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+8] = 81;		// broadcast to all X canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+9] = 82;		// broadcast to all Y canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+10] = 83;	// broadcast to all Z canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+11] = 84;	// broadcast to all A canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+12] = 85;	// broadcast to all B canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+13] = 86;	// broadcast to all C canMotors
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+14] = 50;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+15] = 51;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+16] = 52;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+17] = 53;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+18] = 54;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+19] = 55;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+20] = 56;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+21] = 57;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+22] = HH_POSITION_UNPLUGGED;
	_MailBoxes._mailboxNum2Device[NUM_PHYSICAL_DEVICES+23] = 0xff;       // unknown mail will get dumped here

	for (i=0; i<256; i++)
	{
		_MailBoxes._device2MailboxNum[i] = NUM_PHYSICAL_PLUS_LOGICAL_DEVICES-1;     // preset to last slot of mailbox to hold "unknown devices"
		 _MailBoxes._remapDevicePositionTable[i] = i;                            // initial alias table is a pass through
	}

	for (i=0; i<NUM_PHYSICAL_PLUS_LOGICAL_DEVICES; i++)
	{
		_MailBoxes._device2MailboxNum[_MailBoxes._mailboxNum2Device[i]] = i;    // config lookup to convert device number to mailbox num
		_MailBoxes._outbox[i].device = _MailBoxes._mailboxNum2Device[i];
		_MailBoxes._outbox[i].canbusFormat = CANBUS_FORMAT_V1;  // set a default for all aliases
		if (i<NUM_PHYSICAL_PLUS_LOGICAL_DEVICES)
		{
			_MailBoxes._inbox[i].canbusFormat = CANBUS_FORMAT_V1;   // set a default for all aliases
		}
	}

	changeDevice(DEFAULT_HOT_HEAD);
	changeDevice(DEFAULT_HOT_BED);

	_MailBoxes._hostTrafficReportingPeriodMs = DEFAULT_HOST_TRAFFIC_REPORTING_PERIOD_MS;
	_MailBoxes._hostTrafficReportingPrescaleCnt=_MailBoxes._hostTrafficReportingPeriodMs;

	_MailBoxes._positionReportingPeriodMs = DEFAULT_POSITION_REPORTING_PERIOD_MS;
	_MailBoxes._positionReportingPrescaleCnt = _MailBoxes._positionReportingPeriodMs;

	_outboxHotbedPtr = getOutboxPointer(HH_HOTBED_POSITION);
	_primaryHeadPtr = getOutboxPointer(11);
	resetStickyErrorFlags();
}

////////////////////////////////////////////////////////////////////////////////

boolean matchesAnAlias(byte device, inboxStruct *inboxPtr)
{
	byte i;
	for (i=0; i<NUM_PRE_DEFINED_ALIASES; i++)
	{
		if (inboxPtr->preDefinedAliases[i] == device)
		{
			return(TRUE);
		}
	}
	for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
	{
		if (inboxPtr->userDefinedAliases[i] == device)
		{
			return(TRUE);
		}
	}
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

boolean checkForValidDeviceCombination(devInitStruct *initPtr, inboxStruct *inboxPtr)
{

	boolean badCombo = FALSE;
	if (inboxPtr->canbusFormat == CANBUS_FORMAT_V1)
	{
		if ((inboxPtr->deviceType < SOAP_DEV_TYPE_MK1) || (inboxPtr->deviceType >= SOAP_DEV_TYPE_LAST_ENTRY))
			badCombo = TRUE;
		if ((inboxPtr->deviceSubtype < SOAP_DEV_SUBTYPE_0) || (inboxPtr->deviceSubtype >= SOAP_DEV_SUBTYPE_LAST_ENTRY))
			badCombo = TRUE;
		if ((inboxPtr->devicePcb < SOAP_PCB_4988_160J_NOWIRE) || (inboxPtr->devicePcb >= SOAP_PCB_LAST_ENTRY))
			badCombo = TRUE;
		if ((inboxPtr->deviceRtd1 < SOAP_RTD_NONE) || (inboxPtr->deviceRtd1 >= SOAP_RTD_LAST_ENTRY))
			badCombo = TRUE;
		if ((inboxPtr->deviceRtd2 < SOAP_RTD_NONE) || (inboxPtr->deviceRtd2 >= SOAP_RTD_LAST_ENTRY))
					badCombo = TRUE;

		if (initPtr == NULL)
		{
			badCombo = TRUE;
		}
		else
		{
			soapPcb_t pcb = inboxPtr->devicePcb;
			switch (initPtr->allowablePCB)
			{
			case PCB_J :   badCombo = ((pcb == SOAP_PCB_4982_160MJ_WIRE) || (pcb == SOAP_PCB_4982_160MJ_WIRE));
				break;
			case PCB_MJW : badCombo = (pcb == SOAP_PCB_4982_160MJ_NOWIRE);
				break;
			case PCB_MJ :
			case PCB_ALL_HH :
				break; // all pcb's are good
			case PCB_ENC : badCombo = !((pcb == SOAP_PCB_BTT) || (pcb == SOAP_PCB_405_ENC_PWM) || (SOAP_PCB_405_ENC_DAC));
				break;
			default :
				badCombo = TRUE; break;
			}
		}
	}
	if (badCombo == TRUE)
	{
		sprintf(_errorStr, "Illegal Head Combination of Type/Subtype/PCB/RTD for device %d (0x%02x/0x%02x/0x%02x/0x%02x)",
				inboxPtr->device, inboxPtr->deviceType, inboxPtr->deviceSubtype, inboxPtr->devicePcb, inboxPtr->deviceRtd1);
		sendError(_errorStr);
		return(FALSE);
	}
	else
	{
		return(TRUE);
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceSoftwareRevisionIsAtOrAbove(byte device, byte major, byte minor, char tweak)
{
	inboxStruct *inboxPtr = getInboxPointer(device);
	if (inboxPtr->softwareMajorVersion > major) 				return(TRUE);
	else if (inboxPtr->softwareMajorVersion < major) 			return(FALSE);
	else
	{	// @ major so check minor
		if (inboxPtr->softwareMinorVersion > minor)				return(TRUE);
		else if (inboxPtr->softwareMinorVersion < minor)		return(FALSE);
		else
		{	//.. @ major.minor, so check tweak
			if ((inboxPtr->softwareTweakVersion) >= tweak)		return(TRUE);
		}
	}
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceSoftwareRevisionIsAtOrBelow(byte device, byte major, byte minor, char tweak)
{
	inboxStruct *inboxPtr = getInboxPointer(device);
	if (inboxPtr->softwareMajorVersion < major) 				return(TRUE);
	else if (inboxPtr->softwareMajorVersion > major) 			return(FALSE);
	else
	{	// @ major so check minor
		if (inboxPtr->softwareMinorVersion < minor)				return(TRUE);
		else if (inboxPtr->softwareMinorVersion > minor)		return(FALSE);
		else
		{	//.. @ major.minor, so check tweak
			if ((inboxPtr->softwareTweakVersion) <= tweak)		return(TRUE);
		}
	}
	return(FALSE);

	return((inboxPtr->softwareMajorVersion <= major) && (inboxPtr->softwareMinorVersion <= minor) && (inboxPtr->softwareTweakVersion <= tweak));
}

////////////////////////////////////////////////////////////////////////////////

void getDeviceRegistrationString(inboxStruct *inboxPtr, char *str)
{
	if (inboxPtr->canbusFormat == CANBUS_FORMAT_V0)
	{
		sprintf(str, "V%c - CAN%d~%s~%s~PCB=%s~RTD=%s~Project=%s~Versio=%d.%c",
				inboxPtr->canbusFormat,
				inboxPtr->fromCAN2 ? 2 : 1,
				deviceTypeToString(inboxPtr->deviceType),
				deviceSubtypeToString(inboxPtr->deviceType, inboxPtr->deviceSubtype),
				devicePcbToString(inboxPtr->devicePcb),
				deviceRtdToString(inboxPtr->deviceRtd1),
				"Unknown",  // target
				(int)inboxPtr->softwareMajorVersion,
				(int)inboxPtr->softwareMinorVersion);
	}
	else
	{
		sprintf(str, "V%c - CAN%d~%s~%s~PCB=%s~RTD=%s~Project=%s~%s_%d.%03d%c%c",
				inboxPtr->canbusFormat,
				inboxPtr->fromCAN2 ? 2 : 1,
				deviceTypeToString(inboxPtr->deviceType),
				deviceSubtypeToString(inboxPtr->deviceType, inboxPtr->deviceSubtype),
				devicePcbToString(inboxPtr->devicePcb),
				deviceRtdToString(inboxPtr->deviceRtd1),
				softwareCodebaseToString(inboxPtr->softwareCodebase),
				((int)inboxPtr->softwareMinorVersion >= 4) ?
						softwareCompileTargetToString(inboxPtr->softwareCodebase, inboxPtr->softwareCompileTarget) :
						softwareCompileTargetToStringOLD(inboxPtr->softwareCodebase, inboxPtr->softwareCompileTarget),
				(int)inboxPtr->softwareMajorVersion,
				(int)inboxPtr->softwareMinorVersion,
				inboxPtr->softwareTweakVersion,
				inboxPtr->softwareTweakVersion=='z'?inboxPtr->softwareDebugVersion:' ');
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean validCanMotorDeviceAddress(byte device)
{
	return((device >= CAN_AXIS_MOTOR_MIN_DEVICE_POSITION) && (device <= CAN_AXIS_MOTOR_MAX_DEVICE_POSITION));
}

////////////////////////////////////////////////////////////////////////////////

MotorStructure *getMotorPtrFromDeviceAddress(byte device)
{
	MotorStructure *M = (MotorStructure *)NULL;
	switch (device)
	{
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+0):  M = &Motors[M_X];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+1):  M = &Motors[M_X];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+2):  M = &Motors[M_Y];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+3):  M = &Motors[M_Y];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+4):  M = &Motors[M_Z];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+5):  M = &Motors[M_Z];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+6):  M = &Motors[M_Z];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+7):  M = &Motors[M_Z];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+8):  M = &Motors[M_A];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+9):  M = &Motors[M_A];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+10): M = &Motors[M_B];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+11): M = &Motors[M_B];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+12): M = &Motors[M_C];  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+13): M = &Motors[M_C];  break;
	default: break;
	}

	if ((M != (MotorStructure *)NULL) && M->MotorInstalled)
		return(M);
	return((MotorStructure *)NULL);
}

////////////////////////////////////////////////////////////////////////////////

int getCanMotorIndexFromDeviceAddress(byte device)
{
	int index = 0;
	switch (device)
	{
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+0):  index = 0;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+1):  index = 1;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+2):  index = 0;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+3):  index = 1;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+4):  index = 0;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+5):  index = 1;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+6):  index = 2;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+7):  index = 3;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+8):  index = 0;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+9):  index = 1;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+10): index = 0;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+11): index = 1;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+12): index = 0;  break;
	case (CAN_AXIS_MOTOR_MIN_DEVICE_POSITION+13): index = 1;  break;
	default: break;
	}
	return(index);
}

////////////////////////////////////////////////////////////////////////////////

void startDeviceRegistration(canSwStruct *canRx)
{
	if (isAPhysicalDevice(canRx->device))
	{
		if (canRx->payload.u8[0] != CANBUS_FORMAT_V1)
		{		// first check if this device's software is legal
			int version;
			if (canRx->payload.u8[0] == CANBUS_FORMAT_V2)
				version = 5;
			else if (canRx->payload.u8[0] == CANBUS_FORMAT_V0)
				version = 3;
			else
				version = 2;

			if (_MailBoxes._incompatibleDeviceDetected[canRx->device] == FALSE)
			{   // only send the message once
				//printReportGetLocationString(TRUE, canRx->device, _tmpStr);
				sprintf(_errorStr, "Incompatible head software found (T%d, sw=%d.X)", canRx->device, version);
				_MailBoxes._incompatibleDeviceDetected[canRx->device] = TRUE;
			}
			return;
		}

		inboxStruct *inboxPtr = getInboxPointer(canRx->device);
		outboxStruct *outboxPtr = getOutboxPointer(canRx->device);

#ifdef  GB_DEBUG_REGISTRATION
		sprintf(_tmpStr, "registration 1 (%d)", canRx->device); sendInfo(_tmpStr);
#endif  //GB_DEBUG_REGISTRATION
		// start fresh
		initInboxStruct(inboxPtr); //bzero(inboxPtr, sizeof(inboxStruct));                       // clear structure to all zeros
		bzero(outboxPtr, sizeof(outboxStruct));                     // clear structure to all zeros

		if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
		{
			sendGB("register 1");
		}
		inboxPtr->device = canRx->device;
		inboxPtr->fromCAN2 = canRx->fromCAN2;
		inboxPtr->deviceRegistered = TRUE;
		inboxPtr->commTicker = HH_COMM_WATCHDOG_START_VALUE; // reset because of bzero
//		switch (canRx->payload.u8[0])
//		{
//		case  CANBUS_FORMAT_V2 :
//			sprintf(_errorStr, "Incompatible version 5.X head (%d)", inboxPtr->device); sendError(_errorStr);
//			return;
//		case  CANBUS_FORMAT_V1 :
//			inboxPtr->canbusFormat = CANBUS_FORMAT_V1; break;
//		case CANBUS_FORMAT_V0 :
//		default :
//			sprintf(_errorStr, "Incompatible version 3.X head (%d)", inboxPtr->device); sendError(_errorStr);
//			break;
//		}

		outboxPtr->device = inboxPtr->device;
		outboxPtr->canbusFormat = inboxPtr->canbusFormat;
		_MailBoxes._incompatibleDeviceDetected[canRx->device] = FALSE;

		if (inboxPtr->canbusFormat == CANBUS_FORMAT_V1)
		{
			canRx->page = 1; // lie and change page to 1 to match format for dev info (ANNOUNCE uses same format as DEVICE_INFO page 1
			unpackDeviceInfoPayload(inboxPtr, canRx);   // the ANNOUNCE msg from device contains this info
			outboxPtr->hardInitPtr = getDeviceInitPtr(inboxPtr->deviceType);
			//karlchris   possible place to kill the invalid device config crap message I get sometimes
			outboxPtr->validConfiguration = checkForValidDeviceCombination(outboxPtr->hardInitPtr, inboxPtr);
			if (!outboxPtr->validConfiguration)
			{
				outboxPtr->hardInitPtr = getDeviceInitPtr(SOAP_DEV_TYPE_BAD_COMBO);
			}
			initDeviceFromDeviceTypeInitTable(outboxPtr);
			sendInitializeBit(canRx->device);
		}

		readDeviceInfoFromDevice(canRx->device);
		sendSetRegisterBit(canRx->device);

		inboxPtr->registrationStep  = 1;
		sendLoopbackMessageToDevice(canRx->device, DEVICE_INITIALIZATION, (uint16_t)inboxPtr->registrationStep, 0, 0, 0); // mark so we know end of transfer
		inboxPtr->registrationStep = 101;  //pause

		if (validCanMotorDeviceAddress(canRx->device) && deviceIsACanAxisMotor(canRx->device))
		{	// found a can device that claims be an axis motor (based on address and device type)
			MotorStructure *M = getMotorPtrFromDeviceAddress(canRx->device);
			if (M != NULL)
			{
				M->canMotor = TRUE;
				int index = getCanMotorIndexFromDeviceAddress(canRx->device);
				M->canMotors[index].canAddress = canRx->device;	// tag that we've heard from this sub-axis motor
				M->canMotors[index].selfHomed = FALSE;
				M->canMotors[index].selfHomingInProgress = FALSE;
#ifdef USE_CAN_MOTOR_HOME_SENSOR
#warning "FIX ONCE H/L1/L2 on canMotors is resolved"
				canSendDeviceInitValue1x16(canRx->device, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT1_POLARITY, M->HomeSense.Polarity);
				canSendDeviceInitValue1x16(canRx->device, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT2_POLARITY, M->Limit2Sense.Polarity);
#endif //USE_CAN_MOTOR_HOME_SENSOR
			}
		}
	}
	else
	{	// unknown device # trying to register
		if (_errors.sent.flags.unknownDeviceRegistering == FALSE)
		{
			_errors.sent.flags.unknownDeviceRegistering = TRUE;
			sprintf(_errorStr, "Unknown device # attempting to register with the system (%d)", canRx->device);
			sendError(_errorStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void finishDeviceRegistration(byte device)
{
	outboxStruct *outboxPtr = getOutboxPointer(device);

	if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
	{
		sendGB("register 2");
	}
	sendSetGlobalSyncBit(device); // spread out canbus load

	readFlashConfigFromDevice(device);
	readUniqueIdFromDevice(device);
	readPageDefinitionFromDevice(device);

	//would be great to add default values from the soap string here
	outboxPtr->ExtrusionControl.PulsesPerUnit=100.0f;

	outboxPtr->autoStatusPeriodMs = DEFAULT_AUTO_STATUS_PERIOD_MS;
	if (_KeepHeadsAliveDuringDebug)
		sendSetIgnoreWatchdogBit(0);
	else
		sendClrIgnoreWatchdogBit(0);
	outboxPtr->autoStatusMask = DEFAULT_AUTO_STATUS_MASK;
	sendAutoStatusControlToDevice(outboxPtr, 3);

	readAliasListFromDevice(device);
	readControlWordFromDevice(device);
}

////////////////////////////////////////////////////////////////////////////////

void tryToCleanUpGuiAfterDeviceRegistration(byte device)
{   // now try to keep the GUI in sync with the current state of things
	inboxStruct *inboxPtr   = getInboxPointer(device);
	switch (inboxPtr->deviceFamily)
	{
	case DEVICE_FAMILY_LASER:
	case DEVICE_FAMILY_INKJET:
		//LEGACY SendFakeMcodeExecutionNotice(104, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);
		SendFakeMcodeExecutionNotice(620, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // power supply
		break;
	case DEVICE_FAMILY_SPINDLE:
		SendFakeMcodeExecutionNotice(3, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);     // SPINDLE DIR TO CW (POR def)
		SendFakeMcodeExecutionNotice(5, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);     // SPINDLE OFF
		break;
	case DEVICE_FAMILY_PICKNPLACE:
		break;
	case DEVICE_FAMILY_HOTBED:
		SendFakeMcodeExecutionNotice(140, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // BED HTR
		SendFakeMcodeExecutionNotice(141, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // CHAMBER
		break;
	case DEVICE_FAMILY_HEATED_EXTRUDER:
	case DEVICE_FAMILY_UNHEATED_EXTRUDER:
	case DEVICE_FAMILY_ACCESSORY:
	default:
		SendFakeMcodeExecutionNotice(104, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // HTR
		SendFakeMcodeExecutionNotice(106, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // FAN
		SendFakeMcodeExecutionNotice(723, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // manual extrude
		if (inboxPtr->deviceType == SOAP_DEV_TYPE_CO2_LENS)
		{
			SendFakeMcodeExecutionNotice(678, inboxPtr->device, 0.0f, INVALID_ARG_VALUE);   // cross hair off
		}
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceRegistered(byte device)
{
	return(getInboxPointer(device)->deviceRegistered);
}

////////////////////////////////////////////////////////////////////////////////

void removeCanMotorDeviceFromMotorStructure(byte device)
{
	MotorStructure *M = getMotorPtrFromDeviceAddress(device);
	if (M != NULL)
	{
		M->canMotors[getCanMotorIndexFromDeviceAddress(device)].canAddress = 0;
	}
	// Now check if ANY canAxisMotors are still associated with the axis and if not, assume
	// the axis is back to local control my the MC.
	//
	// NOTE: if all devices were really removed, then this should not find any!
	boolean canbusControlledAxis = FALSE;
	for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
	{
		if (M->canMotors[canAddrIndex].canAddress > 0)
		{
			canbusControlledAxis = TRUE;
			break;
		}
	}
	if (canbusControlledAxis == FALSE)
	{
		M->canMotor = FALSE;
	}
}

////////////////////////////////////////////////////////////////////////////////

inboxStruct *findNthRegisteredDevice(int nth)
{
	int found=0;
	for (int i=0; i<NUM_PHYSICAL_DEVICES; i++)
	{
		if (_MailBoxes._inbox[i].deviceRegistered)
		{
			found++;
			if (found == nth)
			{
				return(&_MailBoxes._inbox[i]);
			}
		}
	}
	return((inboxStruct *)NULL);
}

////////////////////////////////////////////////////////////////////////////////

void removeAllRegisteredDevice(void)
{
	inboxStruct *inboxPtr;
	for (int i=0; i<NUM_PHYSICAL_DEVICES; i++)
	{
		inboxPtr = &_MailBoxes._inbox[i];
		if (inboxPtr->deviceRegistered)
		{
			if (validCanMotorDeviceAddress(inboxPtr->device) && deviceIsACanAxisMotor(inboxPtr->device))
			{	// found a can device that claims be an axis motor (based on address and device type)
				// Need to remove it's address from it's axis' MotorStructure
				removeCanMotorDeviceFromMotorStructure(inboxPtr->device);
			}

			sprintf(_rptStr, ">RM:%d ", inboxPtr->device);
			sendstringCr(_rptStr);
			initInboxStruct(inboxPtr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//check for Missing IN action for a head, meaning it died or was removed.
void checkForMia(void)
{
	int i;
	inboxStruct *inboxPtr;

	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)
	{
		inboxPtr = &_MailBoxes._inbox[i];
		if (inboxPtr->commTicker > 0)
		{
			inboxPtr->commTicker--;
		}
		if (_KeepHeadsAliveDuringDebug)
		{
			inboxPtr->commTicker = 1;
		}
		if (inboxPtr->deviceRegistered && (inboxPtr->commTicker == 0) && (inboxPtr->bootloaderRunning == FALSE))  // don't MIA bootloader devices
		{
			if (validCanMotorDeviceAddress(inboxPtr->device) && deviceIsACanAxisMotor(inboxPtr->device))
			{
				removeCanMotorDeviceFromMotorStructure(inboxPtr->device);
				sendMotorResetToDevice(inboxPtr->device);
			}
			else
			{
				sendSetSwResetBit(inboxPtr->device); // try to reset device to get it to re-register
			}

			sprintf(_rptStr, ">RM:%d ", inboxPtr->device);
			sendstringCr(_rptStr);

			if (currentOutboxPtr->device == inboxPtr->device)
			{   // active head removed
				if (!(motionQ_empty() && cmdQueIsEmpty()))
				{   // not idle
					sprintf(_errorStr, "Lost comm from active head (%d) -- aborting job", inboxPtr->device);
					catastrophicError(_errorStr);
				}
			}
			initInboxStruct(inboxPtr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendInboxInfoToHost(byte device, deviceInfoType selection)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	uint16_t i, index;
	inboxStruct *inboxPtr;
	char *formatStr;
	uint32_t password;

	inboxPtr = getInboxPointer(device);

	for (index=0; index<NUM_DEVICE_REPORT_INFO_TABLE_ENTRIES; index++)
	{   // find the requested select
		if (_deviceReportInfoTable[index].type == selection)
		{   // match, so index is the array index of the selection in the table
			break;
		}
	}
	if (index == NUM_DEVICE_REPORT_INFO_TABLE_ENTRIES)
	{   // index points past the table, so not a legal selection
		sprintf(_errorStr, "Illegal selection request in sendInboxInfoToHost(): %d", selection);
		sendError(_errorStr);
		return;
	}

	sprintf(_rptStr, ">RI: %2d: %s:", inboxPtr->device, _deviceReportInfoTable[index].abbreviation);
	formatStr = (char *)&_deviceReportInfoTable[index].formatStr[0];

	switch (selection)
	{
	case SEND_DEVICE_REVISION_INFO :
		getDeviceRegistrationString(inboxPtr, _tmpStr);
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_DEVICE_FLASH_CONFIG :
		sprintf(_tmpStr, formatStr, inboxPtr->flashNumKBytes, inboxPtr->flashPageSize, (unsigned int)inboxPtr->flashBaseAddr);
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_DEVICE_UNIQUE_ID :
		for (i=0; i<12; i++)
		{
			sprintf(_tmpStr, formatStr, inboxPtr->uniqueId[i]);
			strcat(_rptStr, _tmpStr);
		}
		break;
	case SEND_DEVICE_ALIAS_LIST :
		for (i=0; i<NUM_PRE_DEFINED_ALIASES; i++)
		{
			sprintf(_tmpStr, formatStr, inboxPtr->preDefinedAliases[i]);
			strcat(_rptStr, _tmpStr);
		}
		for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
		{
			sprintf(_tmpStr, formatStr, inboxPtr->userDefinedAliases[i]);
			strcat(_rptStr, _tmpStr);
		}
		break;
	case SEND_DEVICE_PAGE_DEF :
		if (inboxPtr->softwareMinorVersion >= 3)
		{
			sprintf(_tmpStr, formatStr, inboxPtr->soapPage, inboxPtr->soapPage, inboxPtr->soapPage, inboxPtr->soapPage);
		}
		else
		{
			sprintf(_tmpStr, formatStr,
					inboxPtr->pageDef[TABLE_PAGE_INDEX],
					inboxPtr->pageDef[SOAPBOX_PAGE_INDEX],
					inboxPtr->pageDef[HISTORY0_PAGE_INDEX],
					inboxPtr->pageDef[HISTORY1_PAGE_INDEX]);
		}
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_DEVICE_CONTROL_WORD :
		sprintf(_tmpStr, formatStr, (unsigned int)inboxPtr->controlWord.u32);
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_DEVICE_PASSWORD :
		password = convertUidToPassword(inboxPtr->uniqueId);
		sprintf(_tmpStr, formatStr, (unsigned int)password, (unsigned int)password);
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_DEVICE_PAGE_CHECKSUM :
		sprintf(_tmpStr, formatStr, (unsigned int)inboxPtr->checksumedPage, (unsigned int)inboxPtr->pageChecksum, (unsigned int)inboxPtr->pageChecksum);
		strcat(_rptStr, _tmpStr);
		break;
	default :
		break;
	}
	sendstringCr(_rptStr);
}
////////////////////////////////////////////////////////////////////////////////

void clearWorkingBuffer(void)
{
	bzero(_MailBoxes._workingBuffer, WORKING_BUFFER_SIZE_IN_BYTES);
}

////////////////////////////////////////////////////////////////////////////////

boolean isAHexChar(char c)
{
	if (((c >= '0') && (c <= '9')) || ((c >= 'A') && (c <= 'F')) || ((c >= 'a') && (c <= 'f')))
		//if (inString(c, "0123456789ABCEDFabcdef")
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean transferDevicePageDataFromHostToWorkingBuffer(byte device, tableInfoType selection)
{
	uint16_t    i;
	uint16_t    index;
	uint16_t    offset;
	uint16_t    maxCount;
	byte        *dataPtr;
	char        *startPtr;
	char            asciiByteStr[] = "0xFF";
	unsigned int    tmpByte;


	if (isALogicalDevice(device))
	{
		sendError("transferDevicePageDataFromHostToWorkingBuffer - invalid device (not a physical device)");
		return(FAIL);
	}

	//check to see if the requested table is valid
	//if this falls through and does not hit a match to a valid table it will fail
	for (index=0; index<NUM_DEVICE_REPORT_PAGE_INFO_TABLE_ENTRIES; index++)
	{
		if (_deviceReportPageInfoTable[index].type == selection) break;
	}

	if (index == NUM_DEVICE_REPORT_PAGE_INFO_TABLE_ENTRIES) return(FAIL);       // nothing to send (no match to table)

	offset = 0;
	dataPtr = (byte *)&_MailBoxes._workingBuffer[offset];
	startPtr = &GCodeArgComment[1]; // data will come in comment field; skip comment delimeter

	switch (selection)
	{
	case SEND_DEVICE_SOAP_STRING :
		maxCount = getInboxPointer(device)->flashPageSize - 1;  // -1 to leave room for null char to terminate string
		if (strlen(startPtr) < maxCount)
		{
			maxCount = strlen(startPtr);
		}
		else
		{
			return(FAIL);  // string too long
		}
		memcpy(dataPtr, startPtr, maxCount+1);
		break;
	case SEND_DEVICE_RAW_PAGE_DATA :
		if (strlen(startPtr) != 2*getInboxPointer(device)->flashPageSize)
		{
			return(FAIL);  // incorrect num of chars for the full fage
		}
		for (i=0; i<getInboxPointer(device)->flashPageSize; i++)
		{
			// covert one byte at a time (2 ASCII chars)
			if (!isAHexChar(*startPtr))
			{
				return(FAIL); // illegal hex format
			}
			asciiByteStr[2] = *startPtr++;      // replace the two "number" positions in the string
			if (!isAHexChar(*startPtr))
			{
				return(FAIL); // illegal hex format
			}
			asciiByteStr[3] = *startPtr++;
			sscanf(asciiByteStr, "%x", &tmpByte);
			*dataPtr++ = (byte)tmpByte;
		}
		break;
	default :
		return(FAIL);       // unknown selection
		break;
	}
	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

void transferDevicePageDataFromWorkingBufferToDevice(byte device, uint16_t page)
{
	uint16_t i;
	byte *dataPtr;
	uint32_t workingBufferChecksum;

	canFillDeviceBuffer(device, 0x00000000);

	dataPtr = _MailBoxes._workingBuffer;
	for (i=0; i<getInboxPointer(device)->flashPageSize/8; i++)  // 8 bytes per can packet
	{
		canWriteDeviceBuffer(device, i, (payloadUnion *)dataPtr);
		dataPtr += 8;
	}

	workingBufferChecksum = getWorkingBufferChecksum(device);
	canCopyDeviceBufferToPage(device, page, workingBufferChecksum);
	//CAN_MSG_COPY_BUFFER_TO_PAGE
	_MailBoxes._pageChecksum = 0;
	readPageChecksumFromDevice(device, page);   // read back checksum from newly written page
}

////////////////////////////////////////////////////////////////////////////////

uint16_t tableInfoTypeToPage(byte device, tableInfoType selection, uint16_t selectedPage)
{
	// determine the page to access for the selected tableInfoType
	// MCODE        SEND_DEVICE_POSITION_TABLE      = 0  (deprecated)
	// MCODE        SEND_DEVICE_RTD1_TABLE          = 1  (deprecated)
	// MCODE        SEND_DEVICE_RTD2_TABLE          = 2  (deprecated)
	// MCODE        SEND_DEVICE_RTD3_TABLE          = 3  (deprecated)
	// MCODE        SEND_DEVICE_TEMP_SENSOR_TABLE   = 4  (deprecated)
	// MCODE        SEND_DEVICE_TEMP_2_LED_TABLE    = 5  (deprecated)
	// MCODE        SEND_DEVICE_PULSE_TRAIN_TABLE   = 6  (deprecated)
	// MCODE        SEND_DEVICE_SOAP_STRING         = 7
	// MCODE        SEND_DEVICE_HISTORY0_DATA       = 8  (deprecated)
	// MCODE        SEND_DEVICE_HISTORY1_DATA       = 9  (deprecated)
	// MCODE        SEND_DEVICE_RAW_PAGE_DATA       = 10 <requires page argument>

	uint16_t page;

	switch (selection)
	{
	case SEND_DEVICE_SOAP_STRING :
		if (getInboxPointer(device)->softwareMinorVersion >= 3)
			page = getInboxPointer(device)->soapPage;
		else
			page = getInboxPointer(device)->pageDef[SOAPBOX_PAGE_INDEX];
		break;
	case SEND_DEVICE_RAW_PAGE_DATA  :
		page = selectedPage;
		break;
	default :
		sprintf(_errorStr, "tableInfoTypeToPage - invalid selection:%d", (int)selection);
		sendError(_errorStr);
		return(0xffff);
	}
	return(page);
}

////////////////////////////////////////////////////////////////////////////////soap


void transferDevicePageDataFromDeviceToWorkingBuffer(byte device, uint16_t page)
{
	canFillDeviceBuffer(device, 0x00000000);//zero out the working buffer on the device
	canCopyPageToDeviceBuffer(device, page);//make a copy of the page/table tothe working buffer on the device
	canReadDeviceBuffer(device);//
	_MailBoxes._pageChecksum = 0;
	clearWorkingBuffer();
	readPageChecksumFromDevice(device, page);
}

////////////////////////////////////////////////////////////////////////////////

void transferDevicePageDataFromWorkingBufferToHost(byte device, tableInfoType selection)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	// process to get data from device to system and then send/display to host:
	//      fill device working buffer with 0's
	//      copy page of flash containing selected data to device working buffer
	//      read data from working buffer (for now, transfer entire page)
	//      read checksum for selected page to verify all data was sent and was rcvd properly
	//      set "waiting_for_checksum" flag to mark end of transfers needed (set breadcrumb to know what to do next)
	//      once checksum received, send requested data to host
	uint16_t    soapStringLength;
	uint16_t    i;
	uint16_t    index;
;
	char        *formatStr;
	byte        *dataPtr;

	if (isALogicalDevice(device))
	{
		sprintf(_errorStr, "transferDevicePageDataFromWorkingBufferToHost - invalid device:%d (not a physical device)", device);
		sendError(_errorStr);
		return;
	}

	for (index=0; index<NUM_DEVICE_REPORT_PAGE_INFO_TABLE_ENTRIES; index++)
	{
		if (_deviceReportPageInfoTable[index].type == selection) break;
	}
	if (index == NUM_DEVICE_REPORT_PAGE_INFO_TABLE_ENTRIES) return;     // nothing to send (no match to table)

	sprintf(_rptStr, ">RI: %2d: %s:", device, _deviceReportPageInfoTable[index].abbreviation);
	sendstring(_rptStr);
	formatStr = (char *)&_deviceReportPageInfoTable[index].formatStr[0];

	switch (selection)
	{
	case SEND_DEVICE_SOAP_STRING :
		_MailBoxes._workingBuffer[WORKING_BUFFER_SIZE_IN_BYTES-1] = 0;  // make sure last char is null
		if (getInboxPointer(device)->registrationStep)
		{
//NUKE			getInboxPointer(device)->deviceFamily = devType2DevFamily(_MailBoxes._workingBuffer[0]);
//			if (getInboxPointer(device)->deviceFamily == DEVICE_FAMILY_UNKNOWN)
			if (devType2DevFamily(_MailBoxes._workingBuffer[0]) == DEVICE_FAMILY_UNKNOWN)
			{
				sprintf(_errorStr, "UNKNOWN FAMILY - SOAPSTRING OUT OF DATE for device=%d (%c)", device, (char)_MailBoxes._workingBuffer[0]);
				sendError(_errorStr);
			}
		}
		soapStringLength=strlen((char *)_MailBoxes._workingBuffer);
		if (soapStringLength < WORKING_BUFFER_SIZE_IN_BYTES)
		{
			sendNchars((char *)_MailBoxes._workingBuffer, strlen((char *)_MailBoxes._workingBuffer));   // can't use sendstring due to string length restriction
			sendstringCr("");
		}
		else
		{
			sprintf(_errorStr, "SOAPSTRING TOO LONG (%d)", strlen((char *)_MailBoxes._workingBuffer));
			sendError(_errorStr);
		}
		break;
	case SEND_DEVICE_RAW_PAGE_DATA :
		dataPtr = (byte *)_MailBoxes._workingBuffer;
		for (i=0; i<WORKING_BUFFER_SIZE_IN_BYTES; i++)
		{
			if ((i % 32) == 0)
			{
				_rptStr[0] = 0;
			}
			sprintf(_tmpStr, formatStr, (char)*dataPtr++);
			strcat(_rptStr, _tmpStr);
			if ((i % 32) == 31)
			{
				sendNchars(_rptStr, 32*2);
			}
		}
		sendstringCr("");
		break;
	default :
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

byte *getNextSoapstringCmdPtr(byte *ptr)
{
	boolean cmdFound = FALSE;
	while ((ptr < (byte *)(_sysInfoPtr->soapBaseAddr + _sysInfoPtr->soapSize)) && (*ptr != 0xff))
	{   // while still pointing to the soap area and pointing to a programmed byte
		if (*ptr == '*')
		{
			cmdFound = TRUE;
			ptr++;  // move past the '*'
			break;
		}
		else
		{
			ptr++;
		}
	}
	if (cmdFound)
		return(ptr);
	else
		return(NULL);
}

////////////////////////////////////////////////////////////////////////////////

void initFromSoapstring(void)
{
	if (processSoapstringCommands && (soapstringCommandWaiting == 0))
	{   // only operate if still working thru the soapstring AND not processing a command
		_sysInfoPtr->soapReadPtr = getNextSoapstringCmdPtr(_sysInfoPtr->soapReadPtr);

		if ((_sysInfoPtr->soapReadPtr == NULL) && (_sysInfoPtr->initFromReservedSoap == FALSE))
		{   // finished with volatile part of the soap; so move on to the reserved part
			_sysInfoPtr->initFromReservedSoap = TRUE;
			_sysInfoPtr->soapReadPtr = FLASH_SOAP_RESERVE_ADDR;
			_sysInfoPtr->soapReadPtr = getNextSoapstringCmdPtr(_sysInfoPtr->soapReadPtr);   // point to an actual command or NULL
		}

		if (_sysInfoPtr->soapReadPtr == NULL)
		{   // no more commands
			processSoapstringCommandsTurnOffCounter = 10; // 100 ms
			processSoapstringCommands = FALSE;  // done processing init code from soap
			_sysInfoPtr->soapReadPtr = (byte *)_sysInfoPtr->soapBaseAddr; // reset pointer to start of soapstring;
		}
		else
		{ // valid pointer to next command
			protectedIncrement(&soapstringCommandWaiting);
		}
	}
	else if (processSoapstringCommandsTurnOffCounter)
	{   // delayed turn off to allow completion of soap init (the M678 can take a while)
		processSoapstringCommandsTurnOffCounter--;
		if (processSoapstringCommandsTurnOffCounter == 0) 		{
			processSoapstringCommands = FALSE;  // done processing init code from soap
			// now that self-init is finished, set all Q positions as if a homing sequence had occurred.
			SetAllAxisHomePosition(FALSE);
		}
	}
	SmoothDataUsingOlympicVotingAverage();
}

////////////////////////////////////////////////////////////////////////////////

void eraseSystemSoapString(void)
{
	FLASH_Status status;

	if (_sysInfoPtr->soapSize != 0)
	{
		FLASH_Unlock();
		status = FLASH_EraseSector(_sysInfoPtr->soapSector, VoltageRange_3);
		FLASH_Lock();
		if (status != FLASH_COMPLETE)
		{
			sprintf(_errorStr, "Flash erase failed for soapstring (status=%d)", (uint16_t)status);
			sendError(_errorStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

byte *getNextWriteLocation(int section)
{
	if (_sysInfoPtr->soapSize)
	{   // device has space for a soapstring
		byte *writePtr;
		byte *maxAddress;

		if (section == 0)
		{
			writePtr = (byte *)_sysInfoPtr->soapBaseAddr;
			maxAddress = (byte *)(_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET);
		}
		else if (section == 1)
		{
			writePtr = (byte *)_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET;   // get to upper section
			maxAddress = (byte *)(_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET + FLASH_SOAP_RESERVE_SIZE);
		}
		else
		{
			return(NULL);   // should never get here
		}

		while ((writePtr < maxAddress) && (*writePtr != 0xFF)) // address with SOAPBOX and valid data
		{
			writePtr++;
		}
		return(writePtr);
	}
	else
	{
		return(NULL);
	}
}

////////////////////////////////////////////////////////////////////////////////

void writeSystemSoapString(int section)
{
	//soapWritePtr is the pointer to the next BYTE in the Flash to be written to, it is initialized by
	//calling initSystemInfo()
	//byte *newDataPtr;//this points to the Start of the SOURCE STRING to be written to the FLASH

	byte *newDataPtr = (byte *)&GCodeArgComment[1];//initialize the SOURCE STRING pointer to the second character in the gcode comment

	byte *soapWritePtr = getNextWriteLocation(section);

	if ((_sysInfoPtr->soapSize == 0) || (strlen((char *)newDataPtr) > ((uint32_t)(_sysInfoPtr->soapBaseAddr + _sysInfoPtr->soapSize) - (uint32_t)soapWritePtr)))
	{
		sprintf(_errorStr, "Insufficient memory to store soapstring (%d)", strlen((char *)newDataPtr));
		sendError(_errorStr);
	}
	else
	{   // do write - assumes erase has already occurred
		FLASH_Unlock();
		while (soapWritePtr < (byte *)(_sysInfoPtr->soapBaseAddr + _sysInfoPtr->soapSize)) // address with SOAPBOX
		{
			FLASH_ProgramByte((uint32_t)soapWritePtr, *newDataPtr);
			soapWritePtr++;
			if (*newDataPtr == '\0')
			{
				break;
			}
			newDataPtr++;
		}
		if (*newDataPtr != '\0')
		{   // exited while loop due to end of flash soap memory and have not found end of string
			sprintf(_errorStr, "Ran out of Flash memory space while writing soapstring (0x%08x)", (unsigned int)soapWritePtr);
			sendError(_errorStr);

		}
		FLASH_Lock();
	}
}

////////////////////////////////////////////////////////////////////////////////

void SendNext407SoapString(void)
{
	if(dump407SoapstringFlag > 0)
	{   // at least one string to send
		sendstring( "RI:Soap:");//header
		while (*_sysInfoPtr->soapReadPtr != 0xFF)
		{
			if(*_sysInfoPtr->soapReadPtr == '\0')
			{
				_sysInfoPtr->soapReadPtr++; // move past the string termination char to be ready for next read
				sendchar(CMD_END_CHAR); //send cr to terminate the line please
				return;
			}
			sendchar(*_sysInfoPtr->soapReadPtr++);//send a character and then index to the next one
		}
		sendstringCr("EOS");  // end of recorded flash
		dump407SoapstringFlag=0;//end of the transfer so quit.
	}
}

////////////////////////////////////////////////////////////////////////////////
#define SOAPSTRING_WATCHDOG_START_VAL 200 //20
void DeviceSoapstringWatchdog(void)
{
	// check if we need to send out a read device soapstring request to any devices
	//
	// registration steps
	// 0 - idle (registered or never heard from
	// 1 - announce recvd and "startRegistration()" called  [which sends a loopback to change to step 2]
	// 2 - call "finishRegistration" [which sends a loopback to change to step 3]
	// 3 - read soapstring  [which sends a loopback to change to step 0 -- finished]

	if (DeviceSoapstringWatchdogCnt == 0)
	{
		int i;
		int skipBreak=0;
		for(i=0; i<NUM_PHYSICAL_DEVICES; i++)
		{//find the first non-zero and read that device
			if (_MailBoxes._inbox[i].registrationStep > 0)
			{
				inboxStruct *inboxPtr = getInboxPointer(_MailBoxes._inbox[i].device);
				switch (inboxPtr->registrationStep)
				{
				case 2: //should be finished with "startDeviceRegistration"
#ifdef GB_DEBUG_REGISTRATION
					sprintf(_tmpStr, "registration 2 (%d)", inboxPtr->device); sendInfo(_tmpStr);
#endif //GB_DEBUG_REGISTRATION
					finishDeviceRegistration(inboxPtr->device);
					DeviceSoapstringWatchdogCnt = SOAPSTRING_WATCHDOG_START_VAL; // will self clear if finished early (each tick is 100ms)
					sendLoopbackMessageToDevice(inboxPtr->device, DEVICE_INITIALIZATION, inboxPtr->registrationStep, 0, 0, 0); // mark so we know end of transfer
					inboxPtr->registrationStep = 102; // pause
					break;
				case 3: // should be finished with "finishDeviceRegistration"
#ifdef GB_DEBUG_REGISTRATION
					sprintf(_tmpStr, "registration 3 (%d)", inboxPtr->device); sendInfo(_tmpStr);
#endif //GB_DEBUG_REGISTRATION
					readDeviceSoapString(inboxPtr->device);
					DeviceSoapstringWatchdogCnt = SOAPSTRING_WATCHDOG_START_VAL; // will self clear if finished early (each tick is 100ms)
					sendLoopbackMessageToDevice(inboxPtr->device, DEVICE_INITIALIZATION, inboxPtr->registrationStep, 0, 0, 0); // mark so we know end of transfer
					inboxPtr->registrationStep = 103; // pause
					break;
				default:
					skipBreak = inboxPtr->device; // record device address
					break;
				}
				_MailBoxes.lastRegisteringDeviceInboxPtr = inboxPtr;
				if (!skipBreak)
				{
					break;  // break out of for loop --  only allow one soap read per call to DeviceSoapstringWatchdog
				}
			}
		}
		if (i==NUM_PHYSICAL_DEVICES)
		{   //went through with no action
			if (DeviceSoapstringWatchdogWatcher > 0)
			{
				DeviceSoapstringWatchdogWatcher--;
				if (DeviceSoapstringWatchdogWatcher==0)
				{
					if (skipBreak)
					{   // sat around for a while and at least one head is "stuck" at one of the registration steps and no action has occurred
						resetSoapstringControl();
						_MailBoxes._waitingFor.flags.bit.canLoopback = FALSE;
						sprintf(_tmpStr, "resetting registration (%d)", skipBreak); sendInfo(_tmpStr);
					}
					else
					{   // wait a while before looking again.
						DeviceSoapstringWatchdogWatcher = 50;   // 5 seconds watchdog watchdog
					}
				}
			}
		}
		else
		{   // something happened so reset watcher
			DeviceSoapstringWatchdogWatcher = 50;   // 5 seconds watchdog watchdog
		}
	}
	else
	{
		if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr, "reg%d",  DeviceSoapstringWatchdogCnt);
			sendGB(_tmpStr);
		}
		DeviceSoapstringWatchdogCnt--;
		if (DeviceSoapstringWatchdogCnt == 0)
		{   // TIMEOUT!!!! - ignore reg request ... head will try again
			if (_MailBoxes.lastRegisteringDeviceInboxPtr)
			{
				sendSetSwResetBit(_MailBoxes.lastRegisteringDeviceInboxPtr->device);    // try to reset head to get it to register
				_MailBoxes.lastRegisteringDeviceInboxPtr->deviceRegistered = FALSE;
				_MailBoxes.lastRegisteringDeviceInboxPtr->registrationStep = 0;
				_MailBoxes.lastRegisteringDeviceInboxPtr = 0;
			}
		}
	}
}


////////////////////////////////////////////////////////////////////////////////

void soapstringController(void)
{
	// check if we need to send out a read device soapstring request to any devices
	DeviceSoapstringWatchdog();

	// check if we need to send any motion controller soap strings back to the host
	SendNext407SoapString();
}

////////////////////////////////////////////////////////////////////////////////

void resetSoapstringControl(void)
{
	int i;
	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)
	{
		_MailBoxes._inbox[i].registrationStep = 0;
	}
	DeviceSoapstringWatchdogWatcher = 50;
}
////////////////////////////////////////////////////////////////////////////////

void readDeviceSoapString(byte device)
{
#ifdef GB_SOAP_READ_PIN
	pinSetBit(GB_SOAP_READ_PIN);
#endif
	if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
	{
		sendGB("register 3");
	}
	uint16_t page = tableInfoTypeToPage(device, (tableInfoType)SEND_DEVICE_SOAP_STRING, 0);
	if (page == 0xffff) { sendError("Read Soap: selection/page combination result in illegal page"); return; }

	transferDevicePageDataFromDeviceToWorkingBuffer(device, page);//request the page data from the hothead via canbus
	sendLoopbackMessageToDevice(device, PAGE_DATA_TO_HOST, (uint16_t)SEND_DEVICE_SOAP_STRING, page, 0, 0); // mark so we know end of transfer
}

////////////////////////////////////////////////////////////////////////////////

void transferSystemInfoToHost(systemInfoType selection)
{
	// MCODE        SEND_SYSTEM_SOAP_STRING     = 1 (check "ALL" setting)
	// MCODE        SEND_SYSTEM_OPTION_BYTES    = 2
	// MCODE        SEND_SYSTEM_REVISION_INFO   = 3
	// MCODE        SEND_SYSTEM_FLASH_CONFIG    = 4
	// MCODE        SEND_SYSTEM_UNIQUE_ID       = 5
	// MCODE        SEND_SYSTEM_PASSWORD        = 6
	// MCODE        SEND_SYSTEM_SOAP_CONFIG     = 7
	// MCODE        SEND_SYSTEM_CRASH_CONFIG    = 8

	uint16_t i, index;
	char *formatStr;
	uint32_t password;

	//find the reporting format,
	for (index=0; index<NUM_SYSTEM_REPORT_INFO_TABLE_ENTRIES; index++)
	{
		if (_systemReportInfoTable[index].type == selection) break;
	}
	//if no match
	if (index == NUM_SYSTEM_REPORT_INFO_TABLE_ENTRIES) return;      // nothing to send

	// send MCODE that likely got us to this point back to the host
	if (selection == SEND_SYSTEM_SOAP_STRING)
		_rptStr[0] = '\0';   // make sure no extra data is sent when returning the soap (special case)
	else
		sprintf(_rptStr, "; %2d: %s:", 0, _systemReportInfoTable[index].abbreviation);  // 0 to indicate "system" and not a hothead

	formatStr = (char *)&_systemReportInfoTable[index].formatStr[0];

	switch (selection)
	{
	case SEND_SYSTEM_SOAP_STRING :
		// send next string from soap....should be terminated by '\0'
		// if encounter a 0xFF, then past the end of the soap and into unwritten FLASH, so send
		// back "EOS" (end of soap)
		SendNext407SoapString();

		//          sendchar(';'); // sending back in gcode comment form

		//          while ((*_sysInfoPtr->soapReadPtr != 0xFF) && (*_sysInfoPtr->soapReadPtr != '\0'))
		//          {
		//              sendchar(*_sysInfoPtr->soapReadPtr++);
		//          }
		//          if (*_sysInfoPtr->soapReadPtr == 0xFF)
		//          {
		//              sendstring("EOS");  // end of recorded flash
		//              dump407SoapstringFlag=0;//end of the transfer so quit.
		//          }
		//          else if (*_sysInfoPtr->soapReadPtr == '\0')
		//          {
		//              _sysInfoPtr->soapReadPtr++; // move past the string termination char to be ready for next read
		//          }
		break;
	case SEND_SYSTEM_OPTION_BYTES :
		for (i=0; i<16; i++)
		{
			sprintf(_tmpStr, formatStr, _sysInfoPtr->optionBytes[i]);
			strcat(_rptStr, _tmpStr);
		}
		break;
	case SEND_SYSTEM_REVISION_INFO :
		sprintf(_tmpStr, formatStr, _sysInfoPtr->softwareMajorVersion, _sysInfoPtr->softwareMinorVersion, _sysInfoPtr->softwareTweakVersion,
				(_sysInfoPtr->softwareTweakVersion=='z')?_sysInfoPtr->softwareDebugVersion:' ', _sysInfoPtr->mcuDeviceID, _sysInfoPtr->mcuRevisionID);
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_SYSTEM_FLASH_CONFIG :
		sprintf(_tmpStr, formatStr, _sysInfoPtr->flashNumKBytes, (unsigned int)_sysInfoPtr->flashBaseAddr);
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_SYSTEM_SOAP_CONFIG :
		sprintf(_tmpStr, formatStr, _sysInfoPtr->soapSize/1024, (unsigned int)_sysInfoPtr->soapBaseAddr, (_sysInfoPtr->soapSector >> 3));
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_SYSTEM_CRASH_CONFIG :
		sprintf(_tmpStr, formatStr, _sysInfoPtr->crashlogSize/1024, (unsigned int)_sysInfoPtr->crashlogBaseAddr, (_sysInfoPtr->crashlogSector >> 3));
		strcat(_rptStr, _tmpStr);
		break;
	case SEND_SYSTEM_UNIQUE_ID :
		for (i=0; i<12; i++)
		{
			sprintf(_tmpStr, formatStr, _sysInfoPtr->uniqueId[i]);
			strcat(_rptStr, _tmpStr);
		}
		break;
	case SEND_SYSTEM_PASSWORD :
		password = convertUidToPassword(_sysInfoPtr->uniqueId);
		sprintf(_tmpStr, formatStr, (unsigned int)password, (unsigned int)password);
		strcat(_rptStr, _tmpStr);
		break;
	default :
		// XXX add error
		break;
	}
	sendstringCr(_rptStr);
}

////////////////////////////////////////////////////////////////////////////////

void sendExtraStatusInfo(char *rptStr)
{
	char tmpStr1[MAX_STRING_SIZE];  // to all other process and would step on the other strings


	sprintf(rptStr, "%s", "");

	sprintf(tmpStr1, ":%c%d", 'M',(int)_gs._milliseconds);
	strcat(rptStr, tmpStr1);

	if (StatusReportXYZLocation)
	{
		MotorStructure *M;
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors, if the motor is installed, then report
			if (M->MotorInstalled && M->reportPositionWithStatus)
			{
				sprintf(tmpStr1, ":%c%4.3f", M->AxisLabel, M->POSITION * M->UnitsPerPulse);
				strcat(rptStr, tmpStr1);
			}
		}
	}
	if (StatusReportVelocity)
	{
		sprintf(tmpStr1, ":%c%4.3f", 'V',_ActualVectorRateInMmPerSec); // actual speed of current move (if moving)
		strcat(rptStr, tmpStr1);
	}
	if (StatusReportFlowRate)
	{
		sprintf(tmpStr1, ":%c%4.3f", 'E', (_ActualVectorRateInMmPerSec == 0.0f) ? 0.0f : _LastExtrusionRate);
		strcat(rptStr, tmpStr1);
	}
	if (StatusReportLineNum)
	{
		sprintf(tmpStr1, ":%c%ld", 'L', getCurrentExecutingLineNumber());
		strcat(rptStr, tmpStr1);
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendSingleLineReport(byte line)
{
	// routine to send one "line" of the total possible report back to the host.  In this case, the "line" is one piece
	// of information from all devices present in the system
	// will create the string to report the temperature
	// format is
	//     >RT: B:xxx T11:xxx yyy aaa bbb T14:xxx yyy aaa bbb T91:xxx yyy aaa bbb
	// uses space between data pairs for delimeter, and : to separate data from data name
	// if NO device is registered only the header will be sent

	byte i;
	char rptStr1[MAX_STRING_SIZE];  // local version of strings needed because this routine is called async
	char tmpStr1[MAX_STRING_SIZE];  // to all other process and would step on the other strings
	inboxStruct *inboxPtr;
	payloadUnion *payload;
	int field0scale = 0;
	rptStr1[0] = 0;//set the pointer to null string
	strcat(rptStr1, ">RT");//set up the first template for the report of the temperature

	if (StatusReportXYZLocation || StatusReportVelocity || StatusReportFlowRate | StatusReportLineNum)
	{
		sendExtraStatusInfo(tmpStr1);
		strcat(rptStr1, tmpStr1);   // append to end of line
	}

	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)        //walks through each and every element to see if it needs to report
	{
		inboxPtr = &_MailBoxes._inbox[i];
		if (inboxPtr->deviceRegistered)
		{   // only report registered lines
			payload = &inboxPtr->HBPayload[0]; //shorter name in code for default heartbeat payload
			sprintf(tmpStr1, " :T%d ", inboxPtr->device);
			strcat(rptStr1, tmpStr1);   // append to end of line
			field0scale = ((inboxPtr->deviceRtd1 == SOAP_RTD_4_TO_20MA) || (inboxPtr->deviceRtd1 == SOAP_RTD_RAW_ADC)) ? 1 : TEMP_SCALE;
			switch (inboxPtr->deviceFamily)
			{
			// this is to cut out the numberous permutations where the first field may or may not contain a fraction temperature
			case DEVICE_FAMILY_MOTOR :
				if (1)
					sprintf(tmpStr1, "%3.2f %3.2f %3.2f %d",
							(360.0f * (float)payload->i16[0] / (float)inboxPtr->motorTicksRev),	//actual angle
							(360.0f * (float)payload->i16[1] / (float)inboxPtr->motorTicksRev),	//desired angle
							(360.0f * (float)payload->i16[2] / (float)inboxPtr->motorTicksRev),	//err angle
							(int)payload->u16[3]);												//low byte is force, upper byte contains flags
				break;
			case DEVICE_FAMILY_HEATED_EXTRUDER :
			case DEVICE_FAMILY_UNHEATED_EXTRUDER :
				if (getOutboxPointer(inboxPtr->device)->hardInitPtr->sw[HH_HTR_SWITCH].type == SW_TEMP_HEAT_COOL)	// heated/chilled head (dual temp to report)
					sprintf(tmpStr1, "%d %d %d %d",
							(payload->i16[0] / field0scale), 		// temperature or rawadc
							payload->i16[1], 						// HSS2 duty cycle
							(payload->i16[2] / TEMP_SCALE), 		// ambient temp
							payload->i16[3]);						// HSS1 duty cycle
				else
					sprintf(tmpStr1, "%d %d %d %d", 				// default extruder config
							(payload->i16[0] / field0scale), 		// temperature or rawadc
							payload->i16[1], 						// HSS2 duty cycle
							payload->i16[2], 						// HSS1 duty cycle
							payload->i16[3]);						// 0
				break;
			case DEVICE_FAMILY_HOTBED :
				if (getOutboxPointer(inboxPtr->device)->hardInitPtr->sw[HH_HTR_SWITCH].type == SW_TEMP_HEAT_COOL)	// heated/chilled bead (dual temp to report)
					sprintf(tmpStr1, "%d %d %d %d",
							(payload->i16[0] / field0scale), 		// temperature or rawadc
							payload->i16[1], 						// HSS2 duty cycle
							(payload->i16[2] / TEMP_SCALE), 		// ambient temp
							payload->i16[3]);						// HSS1 duty cycle
				else
					sprintf(tmpStr1, "%d %d %d %d", 				// default hotbed config
							(payload->i16[0] / field0scale), 		// temperature or rawadc
							payload->i16[1], 						// HSS2 duty cycle
							payload->i16[2],						// 0 or am2302 temperature
							payload->i16[3]);						// 0 or am2302 humidity
				break;
			case DEVICE_FAMILY_LASER :
				if (deviceIsACO2Laser(inboxPtr->device) && inboxPtr->deviceSubtype == SOAP_DEV_SUBTYPE_C02_LASER_UART)
					sprintf(tmpStr1, "%d %d %d %d %d%d%d",
							(payload->i16[0] / field0scale), 		// temperature or rawadc
							payload->i16[1], 						// pwr supply output current
							payload->i16[2], 						// pwr supply output voltage
							payload->i16[3] & 0x7ff,				// pwe supply control voltage
							(payload->i16[3] >> 14) & 1, (payload->i16[3] >> 13) & 1, (payload->i16[3] >> 12) & 1);	//flags 14:waterProt; 13:waterPressure; 12:WPstate
				else
					sprintf(tmpStr1, "%d %d %d %d",
							(payload->i16[0] / field0scale), 		// temperature or rawadc
							payload->i16[1], 						// laser power duty cycle
							payload->i16[2], 						// desired power
							payload->i16[3]);						// laser current (rtd2 raw adc value)
				break;
			case DEVICE_FAMILY_INKJET :
				if (1)
					sprintf(tmpStr1, "%d %d %d %d",
							payload->i16[0], 						// desired pwm freq
							payload->i16[1], 						// pwm "power" (on=1, 0=off)
							payload->i16[2], 						// power supply enable (1=on ; 0=off)
							payload->i16[3]);						// 0;
				break;
			case DEVICE_FAMILY_SPINDLE :
				if (1)
					sprintf(tmpStr1, "%d %d %d %d",
							payload->i16[0], 						// direction
							payload->i16[1], 						// spindle power duty cycle
							payload->i16[2], 						// RPM
							payload->i16[3]);						// 0;
				break;
			case DEVICE_FAMILY_PICKNPLACE :
				if (1)
					sprintf(tmpStr1, "%d %d %d %d",
							payload->i16[0], 						// center motor home state
							payload->i16[1], 						// left adc avg
							payload->i16[2], 						// right adc avg
							payload->i16[3]);						// center motor position (microsteps)
				break;
			case DEVICE_FAMILY_ACCESSORY :
				if (deviceIsAFilamentDispenser(inboxPtr->device))
					sprintf(tmpStr1, "%ld 0 %d %d",
							payload->i32[0], 						// weight in grams
							payload->i16[2]  / TEMP_SCALE,			// 0 or am2302 temperature
							payload->i16[3]);						// 0 or am2302 humidity
				else
					sprintf(tmpStr1, "%d %d %d %d",
							payload->i16[0], 						// HSS1 duty cycle
							payload->i16[1], 						// HSS1 duty cycle
							payload->i16[2], 						// rtd1 adc raw
							payload->i16[3]);						// rtd2 adc raw
				break;
			case DEVICE_FAMILY_UNKNOWN :
			default:
				if (1)
					sprintf(tmpStr1, "%d %d %d %d",
							payload->i16[0], 						// ??
							payload->i16[1], 						// ??
							payload->i16[2], 						// ??
							payload->i16[3]);						// ??
				break;
			}
			strcat(rptStr1, tmpStr1);   // append default head info to end of line

			if (deviceHasAClosedLoopStepper(inboxPtr->device) && !deviceIsACanAxisMotor(inboxPtr->device))
			{	// new 405 heads have a second payload to give additional info
				payload = &inboxPtr->HBPayload[1];
				sprintf(tmpStr1, " %u %3.2f %3.2f %3.2f",
						payload->u16[0],												//low byte is force, upper byte contains flags
						(360.0f * (float)payload->i16[1] / (float)inboxPtr->motorTicksRev),	//err angle
						(float)payload->u16[2] / TEMP_SCALEF,								//v12 voltage
						(float)payload->u16[3] / TEMP_SCALEF);								//v12 amperage
				strcat(rptStr1, tmpStr1);   // append secondary info to end of line
			}

			if ((_sendingGBStringsMask & GB_STRING_CLS_IFO) && (_sendingGBStringsSubMask & GB_STRING_CLS_SUBMASK_EXTRA_RT_FIELDS)) // use M797 S<mask> to enable
			{
				if (deviceHasAClosedLoopStepper(inboxPtr->device) && deviceSoftwareRevisionIsAtOrAbove(inboxPtr->device, 4, 53, 'a'))
				{
					payload = &inboxPtr->HBPayload[4];
					strcat(rptStr1, getPidMethodStr(tmpStr1, payload->u8[0]));
					if (payload->u8[0] != CONTROL_OPEN_LOOP)
					{
						payload = &inboxPtr->HBPayload[5];	sprintf(tmpStr1, " p%3.2f", payload->flt[0]); strcat(rptStr1, tmpStr1);   // append to end of line
						sprintf(tmpStr1, " i%5.4f", payload->flt[1]); strcat(rptStr1, tmpStr1);   // append to end of line
						payload = &inboxPtr->HBPayload[6]; 	sprintf(tmpStr1, " d%3.2f", payload->flt[0]); strcat(rptStr1, tmpStr1);   // append to end of line
						sprintf(tmpStr1, " a%3.2f", payload->flt[1]); strcat(rptStr1, tmpStr1);   // append to end of line
						strcat(rptStr1, " *");
						payload = &inboxPtr->HBPayload[7]; 	sprintf(tmpStr1, " e%2.1f", payload->flt[0]); strcat(rptStr1, tmpStr1);   // append to end of line
						sprintf(tmpStr1, " i%2.1f", payload->flt[1]); strcat(rptStr1, tmpStr1);   // append to end of line
						payload = &inboxPtr->HBPayload[8]; 	sprintf(tmpStr1, " d%2.1f", payload->flt[0]); strcat(rptStr1, tmpStr1);   // append to end of line
					}
					strcat(rptStr1, " *");
					payload = &inboxPtr->HBPayload[8];		sprintf(tmpStr1, " c%ld", payload->i32[1]); strcat(rptStr1, tmpStr1);   // append to end of line
					payload = &inboxPtr->HBPayload[9];		sprintf(tmpStr1, " m%ld", payload->i32[0]); strcat(rptStr1, tmpStr1);   // append to end of line
					sprintf(tmpStr1, " f%ld", payload->i32[1]); strcat(rptStr1, tmpStr1);   // append to end of line
				}
			}
		}
	}
	// M227 test/debug mode - free running encoder reported with >RT string as fake head 16
	if ((_ABjogging.enabled == FALSE) && (_ABjogging.offMode == JOG_OFF_MODE_FILAMENT))
	{
		sprintf(tmpStr1, " :T%d %lu %d %d %d", JOG_OFF_MODE_FILAMENT_DEVICE, TIM5->CNT, 0, 0, 0);
		strcat(rptStr1, tmpStr1);   // append to end of line
	}
	sendstringCr(rptStr1);
	if (_sendingGBStringsMask & GB_STRING_COMM_ACK_CHECK) // use M797 S<mask> to enable
	{
		/// >GB: COM: lineNum: cmds_rcvd: sum_acks_send+pending: acks_sent: acks_pending:
		//  >GB: COM: 653: 50: 49: 42: 7:
		sprintf(_tmpStr, "COM: %d: %d: %d: %d: %d: %d: %d: %d", _gcodeLineNumber, _gcodeCmdsReceived, _gcodeAcksSent + pendingAcknowledge, _gcodeAcksSent, pendingAcknowledge, _asciiCharsRcvd, _asciiChecksum32, _rejected_normalTxChars);
		sendGB(_tmpStr);
	}
}

// SAVE FOR SOMEDAY...
//will create the string to report the temperature
// format is
//>RS: B:xxx T0:xxx T1:xxx T2: xxx T3:xxx      up to T99
//it will send the message once every n seconds
// if NO device is resgistered then there will be NO message sent

//  if (line >= NUM_DEVICcase DEVICE_T_STATUS_TABLE_ENTRIES) return;     // nothing to send
//
//  if (line == 0) // special case to output T: B: format for repetier plotting
//  {
//      _rptStr[0] = 0;
//      // enable to see in can error report:   sprintf(_rptStr, ">RT: ");
//  }
//  else
//  {
//      sprintf(_rptStr, ">RS: %s:", _deviceReportStatusTable[line].abbreviation);
//  }
//  _rptStr=">RT: ";//set up the first template for the report of the temperature
//
//  //uses space between data pairs for delimeter, and : to seperate data from data name
//  //T0: 123  means that Device 0 Temperature is 123
//  for (i=0; i<13; i++)        //walks through each and every element to see if it needs to report
//  {
//      //_MailBoxes._inbox[0].actualMaterialTemp
//      inboxPtr = &_MailBoxes._inbox[i];
//      if (inboxPtr->deviceRegistered) // check to see if this device is reporting on this line
//      {
//              sprintf(_tmpStr, " T%d:", inboxPtr->mailbox);//specify the device address, 0:11 are yoke positiont, number #12 should be the hot bed
//
//              strcat(_rptStr, _tmpStr);//add to end of line
//              //XXX if you want to report in a floating format use the following line
//              //XXX sprintf(_tmpStr, "%2.1f", (((float)inboxPtr->actualMaterialTemp) /((float)TEMP_SCALE)));
//              sprintf(_tmpStr, "%d", (int16_t)(((float)inboxPtr->actualMaterialTemp) /((float)TEMP_SCALE)));//get the temperature to report NEXT
//              strcat(_rptStr, _tmpStr);//add the temperature to the string
//              strcat(_rptStr, " "); // add the space to delimit between the data pairs, ie   head address, head temperature
//
//          }
//  }
//
//  sendstringCr(_rptStr);

//          else
//          {
//              sprintf(_tmpStr, "%d:", inboxPtr->device);
//              strcat(_rptStr, _tmpStr);
//              offset = _deviceReportStatusTable[line].offset;
//              switch (_deviceReportStatusTable[line].format)
//              {
//                  case FORMAT_U8_DEC :
//                      sprintf(_tmpStr,"%d", *(uint8_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I8_DEC :
//                      sprintf(_tmpStr, "%d", *(int8_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U16_DEC :
//                      sprintf(_tmpStr, "%d", *(uint16_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I16_DEC :
//                      sprintf(_tmpStr, "%d", *(int16_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U32_DEC :
//                      sprintf(_tmpStr, "%d", (unsigned int)*(uint32_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I32_DEC :
//                      sprintf(_tmpStr, "%d", (int)*(int32_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U64_DEC :
//                      sprintf(_tmpStr, "%d", (unsigned int)*(uint64_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I64_DEC :
//                      sprintf(_tmpStr, "%d", (int)*(int64_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_FLT :
//                      sprintf(_tmpStr, "%2.1f", *(float *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_DOUB :
//                      sprintf(_tmpStr, "%2.1f", *(double *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U8_HEX :
//                      sprintf(_tmpStr, "0x%02x",  (unsigned int)*(uint8_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I8_HEX :
//                      sprintf(_tmpStr, "0x%02x",  (unsigned int)*(int8_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U16_HEX :
//                      sprintf(_tmpStr, "0x%04x",  (unsigned int)*(uint16_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I16_HEX :
//                      sprintf(_tmpStr, "0x%04x",  (unsigned int)*(int16_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U32_HEX :
//                      sprintf(_tmpStr, "0x%08x",  (unsigned int)*(uint32_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I32_HEX :
//                      sprintf(_tmpStr, "0x%08x",  (unsigned int)*(int32_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_U64_HEX :
//                      sprintf(_tmpStr, "0x%016x",  (unsigned  int)*(uint64_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_I64_HEX :
//                      sprintf(_tmpStr, "0x%016x", (int)*(int64_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_INT_TEMP :
//                      sprintf(_tmpStr, "%d", (int16_t)((float)(*(int16_t *)(((byte *)inboxPtr) + offset))/(float)TEMP_SCALE));
//                      break;
//                  case FORMAT_FLT_TEMP :
//                      sprintf(_tmpStr, "%2.1f", (float)(*(int16_t *)(((byte *)inboxPtr) + offset))/(float)TEMP_SCALE);
//                      break;
//                  case FORMAT_ADDR :
//                      sprintf(_tmpStr, "0x%08x", (unsigned int)*(uint32_t *)(((byte *)inboxPtr) + offset));
//                      break;
//                  case FORMAT_CAL :
//                      sprintf(_tmpStr, "%d %2.1f %d", *(uint16_t *)(((byte *)inboxPtr) + offset), (float)_gs._calibrationTemp/(float)TEMP_SCALE,
//                              *(uint16_t *)(((byte *)inboxPtr) + (MB0.switchValues[HH_HTR_SWITCH].dutyCycle - MB0)));
//                      ;
//                      break;
//
//                  default :
//                      sprintf(_tmpStr, "UNKNOWN_FORMAT:(%d)", (byte)_deviceReportStatusTable[line].format);
//                      break;
//              }
//          }
//          strcat(_rptStr, _tmpStr);
//          strcat(_rptStr, " "); // blank separator after number
//      }
//  }
//
//  sendstringCr(_rptStr);
//}

////////////////////////////////////////////////////////////////////////////////

void sendUpdateToHost(void)
{
	// this routine is called every 10ms, but we will only send a report on 1 second boundaries
	// and at a user programmed number of seconds.   a crude prescale is used to count to 100
	// and on the 100th call to the routine, a "seconds" counter is incremented.  if that count
	// matches the user programmed update rate, the it's time to send status. (a semaphore
	// dumpingStatus is set TRUE;
	//
	// when sending reports, only one "line" of the report (one string over the usart) is sent per call, and
	// only lines that are requested (indicated by a user programmable mask).  each bit position is evaluated
	// during it's respective slice (ie, slice 0 processes bit 0, slice 1 bit 1, etc.  if the
	// mask bit is not set, then no data is sent.  once the last bit is processed, the dumpingStatus
	// semaphore is set to FALSE so that no more status is sent until the next time the seconds
	// count matches the requested rate.

	if (_MailBoxes._hostTrafficReportingPeriodMs == 0)  return;             // no status data requested, so return
	_MailBoxes._hostTrafficReportingPrescaleCnt -= DEFAULT_HOST_TRAFFIC_REPORTING_PERIOD_DECR_MS;
	if (_MailBoxes._hostTrafficReportingPrescaleCnt >  0) return; // still counting down prescale

	_MailBoxes._hostTrafficReportingPrescaleCnt = _MailBoxes._hostTrafficReportingPeriodMs; //reset the interval timer for next time

	sendSingleLineReport(0);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// outbox processing ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

boolean waitingForDeviceTemperature(waitForStruct *waitForPtr, boolean useHSS1) //. useHSS1 is true for chamber
{
	int16_t actualTemp; // needs to be int16_t in order to preserve the sign bit

	int targetTemp, minTemp, maxTemp;

	switch (waitForPtr->mode)
	{
	case WAIT_FOR_ABSOLUTE_TEMP:
		minTemp = waitForPtr->minTemp;
		maxTemp = waitForPtr->maxTemp;
		break;
	case WAIT_FOR_RELATIVE_TEMP:
		if (useHSS1)
			targetTemp = getOutboxPointer(waitForPtr->device)->swTarg[HH_AUX_SWITCH].temperature;
		else
			targetTemp = getOutboxPointer(waitForPtr->device)->swTarg[HH_HTR_SWITCH].temperature;
		minTemp = imax(targetTemp - waitForPtr->minTemp, (MIN_TEMP >> TEMP_FRAC_BITS));
		maxTemp = imin(targetTemp + waitForPtr->maxTemp, (MAX_TEMP >> TEMP_FRAC_BITS));
		break;
	case WAIT_FOR_NOTHING:
	default:
		return(FALSE);
		break;
	}
	if (useHSS1)
		actualTemp = getInboxPointer(waitForPtr->device)->HBPayload[0].i16[2]>>TEMP_FRAC_BITS;
	else
		actualTemp = getInboxPointer(waitForPtr->device)->HBPayload[0].i16[0]>>TEMP_FRAC_BITS;

	if ((actualTemp >= minTemp) && (actualTemp <= maxTemp))
	{   // in the good zone, so release the hold
		return(FALSE);
	}
	else
	{
		return(TRUE);
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean waitingForHotbedTemp(void)
{
	if (_MailBoxes._waitingFor.flags.bit.hotbedTemp == FALSE)
	{   // bail out early if not trying to wait
		return(FALSE);
	}

	_MailBoxes._waitingFor.flags.bit.hotbedTemp = waitingForDeviceTemperature(&_MailBoxes._waitingFor.hotbed, FALSE);

	return(_MailBoxes._waitingFor.flags.bit.hotbedTemp);
}

////////////////////////////////////////////////////////////////////////////////

boolean waitingForChamberTemp(void)
{
	if (_MailBoxes._waitingFor.flags.bit.chamberTemp == FALSE)
	{   // bail out early if not trying to wait
		return(FALSE);
	}

	_MailBoxes._waitingFor.flags.bit.chamberTemp = waitingForDeviceTemperature(&_MailBoxes._waitingFor.chamber, TRUE);

	return(_MailBoxes._waitingFor.flags.bit.chamberTemp);
}

////////////////////////////////////////////////////////////////////////////////

boolean waitingForExtruderTemp(void)
{
	int i;

	if (_MailBoxes._waitingFor.flags.bit.extruderTemp == FALSE)
	{   // bail out early if not trying to wait
		return(FALSE);
	}

	byte saveDevice = _MailBoxes._waitingFor.extruder.device;
	byte newState = FALSE;
	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)  // run though all of possible physical devices
	{
		if (_MailBoxes._inbox[i].deviceRegistered && (_MailBoxes._inbox[i].deviceFamily == DEVICE_FAMILY_HEATED_EXTRUDER))
		{   // check if the device is registered
			if (matchesAnAlias(_MailBoxes._waitingFor.extruder.device, &_MailBoxes._inbox[i])) {
				 _MailBoxes._waitingFor.extruder.device = _MailBoxes._inbox[i].device;  // fake which device to test
				 if (waitingForDeviceTemperature(&_MailBoxes._waitingFor.extruder, FALSE) == TRUE)
				 {
					 newState = TRUE;
					 break;
				 }
			}
		}
	}
	_MailBoxes._waitingFor.extruder.device = saveDevice; // restore device
	_MailBoxes._waitingFor.flags.bit.extruderTemp = newState;

	return(_MailBoxes._waitingFor.flags.bit.extruderTemp);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef ENABLE_CRASH_LOGGING

void eraseSystemCrashLog(void)
{
	FLASH_Status status;

	if (_sysInfoPtr->crashlogSize)
	{   // only if there is a crashlog block in flash
		_sysInfoPtr->crashlogWritePtr = (uint32_t *)_sysInfoPtr->crashlogBaseAddr; // reset write pointer to start of mem

		FLASH_Unlock();
		status = FLASH_EraseSector(_sysInfoPtr->crashlogSector, VoltageRange_3);
		FLASH_Lock();
		if (status != FLASH_COMPLETE)
		{
			sprintf(_errorStr, "Flash erase failed for system crash log (status=%d)", (uint16_t)status);
			sendError(_errorStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void writeSystemCrashLog(uint32_t *dst, uint32_t value)
{
	FLASH_Unlock();
	FLASH_ProgramWord(*dst, value);
	*dst = (*dst) + sizeof(uint32_t);
	FLASH_Lock();
}

////////////////////////////////////////////////////////////////////////////////

void sendCrashDataFromRAMtoFlash(void)
{
	int i;
	uint32_t dstAddr;

	// cannot trust any current memory location, so reconstruct the size/location of the crash dump based on the
	// FLASH_SIZE regsister inside the MCU

	if (*(uint16_t *)FLASH_SIZE_REG_ADDR == 1024)
	{
		FLASH_Unlock();
		if (FLASH_EraseSector(FLASH_1024_CRASH_SECTOR, VoltageRange_3) != FLASH_COMPLETE)  { return; } //  can't do anything
		FLASH_Lock();
		dstAddr = FLASH_1024_CRASH_ADDR;
	}
	else if (*(uint16_t *)FLASH_SIZE_REG_ADDR == 2048)
	{
		FLASH_Unlock();
		if (FLASH_EraseSector(FLASH_2048_CRASH_SECTOR, VoltageRange_3) != FLASH_COMPLETE)  { return; } //  can't do anything
		FLASH_Lock();
		dstAddr = FLASH_2048_CRASH_ADDR;
	}
	else
	{
		return; // no room for the crash log
	}

	writeSystemCrashLog(&dstAddr, CRASHLOG_START);
	for (i=0; i<8; i++)
	{
		writeSystemCrashLog(&dstAddr, *(uint32_t *)(byte *)&crash_source[i*4]);
	}

	unsigned int value;
	for (i=0; i<NUM_CRASHLOG_ENTRIES; i++)
	{
		switch(_crashLog[i].format)
		{   // convert datatypes to 32-bit
		case FORMAT_I8_CHAR  :
			value = (unsigned int)*(char *)_crashLog[i].addr;
			break;
		case FORMAT_U8_DEC   :
		case FORMAT_I8_DEC   :
		case FORMAT_U8_HEX   :
		case FORMAT_I8_HEX   :
			value = (unsigned int)*(uint8_t *)_crashLog[i].addr;
			break;
		case FORMAT_U16_DEC  :
		case FORMAT_I16_DEC  :
		case FORMAT_U16_HEX  :
		case FORMAT_I16_HEX  :
			value = (unsigned int)*(uint16_t *)_crashLog[i].addr;
			break;
		case FORMAT_U32_DEC  :
		case FORMAT_I32_DEC  :
		case FORMAT_U32_HEX  :
		case FORMAT_I32_HEX  :
		case FORMAT_ADDR     :
			value = (unsigned int)*(uint32_t *)_crashLog[i].addr;
			break;
		case FORMAT_FLT      :
		case FORMAT_U64_DEC  :
		case FORMAT_I64_DEC  :
		case FORMAT_U64_HEX  :
		case FORMAT_I64_HEX  :
		case FORMAT_DOUB     :
		default : // not supported
			value = MAXINT;
			break;
		}
		writeSystemCrashLog(&dstAddr, (uint32_t)value);
	}
	writeSystemCrashLog(&dstAddr, CRASHLOG_END);
}
////////////////////////////////////////////////////////////////////////////////

void sendCrashDataFromRAM(void)
{
	int i;
	for (i=0; i<NUM_CRASHLOG_ENTRIES; i++)
	{
		pauseToTransmitBufferToEmpty();
		switch(_crashLog[i].format)
		{   // convert datatypes to 32-bit
		case FORMAT_I8_CHAR  :
			if ((_crashLog[i].label[0] == '\0') && ((*(char *)_crashLog[i].addr) == ' '))
				sprintf(_tmpStr, "%s", "");
			else
				sprintf(_tmpStr, "    %s = %c", _crashLog[i].label, *(char *)_crashLog[i].addr);
			break;
		case FORMAT_U8_DEC   :
		case FORMAT_I8_DEC   :
			sprintf(_tmpStr, "    %s = %d", _crashLog[i].label, (unsigned int)*(uint8_t *)_crashLog[i].addr);
			break;
		case FORMAT_U16_DEC  :
			sprintf(_tmpStr, "    %s = %d", _crashLog[i].label, (unsigned int)*(uint16_t *)_crashLog[i].addr);
			break;
		case FORMAT_U32_DEC  :
		case FORMAT_I32_DEC  :
			sprintf(_tmpStr, "    %s = %d", _crashLog[i].label, (unsigned int)*(uint32_t *)_crashLog[i].addr);
			break;
		case FORMAT_U8_HEX   :
		case FORMAT_I8_HEX   :
			sprintf(_tmpStr, "    %s = 0x%02x", _crashLog[i].label, (unsigned int)*(uint8_t *)_crashLog[i].addr);
			break;
		case FORMAT_U16_HEX  :
		case FORMAT_I16_HEX  :
			sprintf(_tmpStr, "    %s = 0x%04x", _crashLog[i].label, (unsigned int)*(uint16_t *)_crashLog[i].addr);
			break;
		case FORMAT_U32_HEX  :
		case FORMAT_I32_HEX  :
		case FORMAT_ADDR     :
			sprintf(_tmpStr, "    %s = 0x%08x", _crashLog[i].label, (unsigned int)*(uint32_t *)_crashLog[i].addr);
			break;
		case FORMAT_FLT      :
		case FORMAT_U64_DEC  :
		case FORMAT_I64_DEC  :
		case FORMAT_U64_HEX  :
		case FORMAT_I64_HEX  :
		case FORMAT_DOUB     :
		default :
			sprintf(_tmpStr, "    %s = UNKNOWN FORMAT", _crashLog[i].label);
			break;
		}
		sendInfo(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendCrashDataFromFlash(void)
{
	int i;
	unsigned int value;
	uint32_t *dp =  (uint32_t *)(_sysInfoPtr->crashlogBaseAddr+36);;

	if (_sysInfoPtr->crashlogSize == 0)
	{
		sprintf(_tmpStr, "NO CRASH LOG AVAILBLE ON THIS MCU TYPE"); sendInfo(_tmpStr);
	}
	if ((_sysInfoPtr->crashlogSize > 0) && (*(uint32_t *)_sysInfoPtr->crashlogBaseAddr == CRASHLOG_START))
	{
		sendInfo("");
		sendInfo("******************************************************************");
		sendInfo("");

		sprintf(_tmpStr, "CRASH LOG:"); sendInfo(_tmpStr);
		sprintf(_tmpStr, "    Source: %s", (char *)(_sysInfoPtr->crashlogBaseAddr+4)); sendInfo(_tmpStr);
		sendInfo("");

		for (i=0; i<NUM_CRASHLOG_ENTRIES; i++)
		{
			value = *dp++;
			if (value == CRASHLOG_END)
			{
				break;
			}

			switch(_crashLog[i].format)
			{   // convert datatypes to 32-bit
			case FORMAT_I8_CHAR  :
				if ((_crashLog[i].label[0] == '\0') && ((char)value == ' '))
					sprintf(_tmpStr, "%s", "");
				else
					sprintf(_tmpStr, "    %s = %c", _crashLog[i].label, (char)value);
				break;
			case FORMAT_U8_DEC   :
			case FORMAT_I8_DEC   :
			case FORMAT_U16_DEC  :
			case FORMAT_I16_DEC  :
			case FORMAT_U32_DEC  :
			case FORMAT_I32_DEC  :
				sprintf(_tmpStr, "    %s = %d", _crashLog[i].label, value);
				break;
			case FORMAT_U8_HEX   :
			case FORMAT_I8_HEX   :
				sprintf(_tmpStr, "    %s = 0x%02x", _crashLog[i].label, value);
				break;
			case FORMAT_U16_HEX  :
			case FORMAT_I16_HEX  :
				sprintf(_tmpStr, "    %s = 0x%04x", _crashLog[i].label, value);
				break;
			case FORMAT_U32_HEX  :
			case FORMAT_I32_HEX  :
			case FORMAT_ADDR     :
				sprintf(_tmpStr, "    %s = 0x%08x", _crashLog[i].label, value);
				break;
			case FORMAT_FLT      :
				sprintf(_tmpStr, "    %s = %f", _crashLog[i].label, *(float *)&value);
				break;
			case FORMAT_U64_DEC  :
			case FORMAT_I64_DEC  :
			case FORMAT_U64_HEX  :
			case FORMAT_I64_HEX  :
				sprintf(_tmpStr, "    %s = %d **DNF", _crashLog[i].label, value);
			case FORMAT_DOUB     :
				sprintf(_tmpStr, "    %s = %f **DNF", _crashLog[i].label, *(float *)&value);
			default :
				value = 0;
				break;
			}
			sendInfo(_tmpStr);
		}
		if (*(uint32_t *)_sysInfoPtr->crashlogBaseAddr == CRASHLOG_END)
			sendInfo("end of crashlog found");

		sendInfo("");
		sendInfo("******************************************************************");
		sendInfo("");
	}
	else
	{
		sprintf(_tmpStr, "NO CRASH LOG FOUND"); sendInfo(_tmpStr);
	}
}

#endif  // ENABLE_CRASH_LOGGING

////////////////////////////////////////////////////////////////////////////////
