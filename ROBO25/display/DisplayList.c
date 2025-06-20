
#include "ADC.h"
//#include "CAN/can.h"
//#include "Execution/cmdprocessor.h"
#include "displaylist.h"
#include "display.h"
#include "LcdDefinitions.h"
#include "RevisionHistory.h"
#include "mailbox.h"
//#include "taskmanager.h"
//#include "Communication/parser.h"
//#include "SETTINGS/settings.h"
//#include "Communication/secsserial.h"
//#include "Communication/serial.h"
//#include "Utils/secshelper.h"
//#include "Timer/timer.h"
char* AtofVariable="CmdQue[3] Variables";
char* displayTestString2="12345";
//	{(uint32_t)&_gs.CanRxCount1,         "CnRx Cnt", FUNC_INT, 		BLUE,	MAGENTA, 	0},
char TestString[] ="test String 12345";


//osseo display
//LcdVariableInfo LcdVarsTable[] = {
//	{ (uint32_t)1, "GLOBAL1", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
//	{(uint32_t)&HeartBeat,					HB_STRING, FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&ParticleCounter,			"Prtcl_Cnt", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosureTemperature,		"Temp degC", FUNC_FLOAT, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosureHumidity,			"Humidity%", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosurePressureDifference, "Delta P  ",FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosureUvLedPwm,			"UVLED PWM", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosureFanPwm,			"BlowerPWM", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&RPMCounter,					"BlowerRPM", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&doorSenseState,				"DoorSense", FUNC_BAR_FLOAT, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosureCartridgeSense,	"Cartridge", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&EnclosurePrintBedSense,		"Bld Plate", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
////analog conversion data
//	{(uint32_t)&RawADCDataBuffer[0],		"CH01_PA3", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&RawADCDataBuffer[1],        "CH02_PA4", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&RawADCDataBuffer[2],        "CH03_PA5", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&RawADCDataBuffer[3],        "CH04_PA6", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&RawADCDataBuffer[4],		"CH05_PB1", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&RawADCDataBuffer[5],		"CH05_PC5", FUNC_INT16, COLOR_WHITE, COLOR_RED, 0},
////end osseo table
MotorStatusStruct MotorXStatus = { (uint32_t)&Motors[M_X].POSITION, X_HOME, X_LIMIT };
MotorStatusStruct MotorYStatus = { (uint32_t)&Motors[M_Y].POSITION, Y_HOME, Y_LIMIT };
MotorStatusStruct MotorZStatus = { (uint32_t)&Motors[M_Z].POSITION, Z_HOME, Z_LIMIT };
MotorStatusStruct MotorAStatus = { (uint32_t)&Motors[M_A].POSITION, A_HOME, A_LIMIT };
MotorStatusStruct MotorBStatus = { (uint32_t)&Motors[M_B].POSITION, B_HOME, B_LIMIT };
MotorStatusStruct MotorCStatus = { (uint32_t)&Motors[M_C].POSITION, C_HOME, C_LIMIT };

LcdVariableInfo LcdVarsTable[] = {
	{ (uint32_t)1, "ROBO25_X1", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
	{(uint32_t)&HeartBeat,					HB_STRING, FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//analog conversion data
	//_primaryHeadPtr
//	{(uint32_t)&Head11_Temperature, "Head Temp", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&Head11_HTRDuty,		"Head Duty", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&Head11_FanDuty,		"Head Duty", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&Head11_Spare,		"Head Duty", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},

//	{(uint32_t)&RawADCDataBuffer[0], "CH01_PA3", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&RawADCDataBuffer[1], "CH02_PA4", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&RawADCDataBuffer[2], "CH03_PA5", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&RawADCDataBuffer[3], "CH04_PA6", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&RawADCDataBuffer[4], "CH05_PB1", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&laserTemperature,    "LASER_TMP", FUNC_FLOAT, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&RawADCDataBuffer[5], "CH05_PC5", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
	
	{(uint32_t)&MotorXStatus, "MtrX Pos", FUNC_MOTOR_STATUS, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&MotorYStatus, "MtrY Pos", FUNC_MOTOR_STATUS, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&MotorZStatus, "MtrZ Pos", FUNC_MOTOR_STATUS, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&MotorAStatus, "MtrA Pos", FUNC_MOTOR_STATUS, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&MotorBStatus, "MtrB Pos", FUNC_MOTOR_STATUS, COLOR_WHITE, COLOR_MAGENTA, 0},
	{(uint32_t)&MotorCStatus, "MtrC Pos", FUNC_MOTOR_STATUS, COLOR_WHITE, COLOR_MAGENTA, 0},

//{(uint32_t)&MotorZ.POSITION,			"MtrZ Pos", FUNC_INT32,		COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&MotorZ.PULSES_TO_GO,		"Z Pls2Go", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&MotorA.POSITION,			"MtrA Pos", FUNC_INT32,		COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&MotorA.PULSES_TO_GO,		"A Pls2Go", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
////motor control values
//	{(uint32_t)&Tim1reload, "T1_Relod", FUNC_INT32, COLOR_WHITE, COLOR_LIME, 0},
//	{(uint32_t)&MasterMotorAccelerationIndex, "ACC # ", FUNC_BAR_FLOAT, COLOR_WHITE, COLOR_MAGENTA, (uint32_t)&AccelerationTableSize},
//	{(uint32_t)&MasterMotorRemainMoveDistance, "Move D", FUNC_BAR_FLOAT, COLOR_WHITE, COLOR_MAGENTA, (uint32_t)&MasterMotorTotalMoveDistance},
//	
	//{(uint32_t)&Timer3Count,            	"T3_Count", FUNC_INT16, 	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	//{(uint32_t)&Timer4Count,				"T4_Count", FUNC_INT16,		COLOR_WHITE, COLOR_MAGENTA, 0},	
	//{(uint32_t)&Timer5Count,				"T5_Count", FUNC_INT16,		COLOR_WHITE, COLOR_MAGENTA, 0},
	//{(uint32_t)&Timer8Count,				"T8_Count", FUNC_INT16,		COLOR_WHITE, COLOR_MAGENTA, 0},	


	//{(uint32_t)&TIM17->CNT,             	"T1_Relod", FUNC_INT16, 		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&Timer15Count,            	"T15Count", FUNC_INT16, 		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.RxLineCount,"#LinesRx", FUNC_INT16,  	    COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.Head, 		"RxBfHead", FUNC_HEX32,  		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.Tail, 		"RxBfTail", FUNC_HEX32,  		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.CharsInBuf, "ChrInBff", FUNC_INT16,  		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&USBPacketLength,            "UsbRxLen", FUNC_INT16,  		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&USBPacketAddress,           "RxAddrss", FUNC_HEX32,  		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&LastMessageTail, 	    	"Last Msg", FUNC_MEMDUMPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&CommandsInQue,           	"CMDInQUE", FUNC_INT32, 		COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&ParsedLineCounter,       	"Parsed #", FUNC_INT32,	        COLOR_WHITE,	COLOR_MAGENTA,    0},
//	{(uint32_t)&gcodeLineNumber,       	    "G_Line #", FUNC_INT32,	        COLOR_WHITE,	COLOR_MAGENTA,    0},
	{(uint32_t)0,              			 "--------", FUNC_INT, 		COLOR_YELLOW,	COLOR_MAGENTA, 	0},

};

LcdVariableInfo SecsVarsTable[] = {
	{ (uint32_t)1, "SECS VAR", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
//	{(uint32_t)&secsmessagelength,  "Msg_Len ", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&systembytes,		"Sys_Byte", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&systembyte34,		"Sys_Byte", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&receiveid,			"Rcvd_ID ", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&CHECKSUMCALCULATED, "CheckSum", FUNC_INT16, COLOR_WHITE, COLOR_LIME, 0},
//	{(uint32_t)&checksum_passed,	"CksmPASS", FUNC_INT16, COLOR_WHITE, COLOR_LIME, 0},
//	{(uint32_t)&secssendpass,		"Xmt#   ", FUNC_INT16, COLOR_WHITE, COLOR_LIME, 0},
//	{(uint32_t)&secssendfail,		"XmtFail#", FUNC_INT16, COLOR_WHITE, COLOR_RED, 0},
//	{(uint32_t)&numberofretriesleft,"Retries ", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//		//timers
//	{(uint32_t)&secstimer1,						"SecsTim1", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},		
//	{(uint32_t)&secstimer2,						"SecsTim2", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&secstimer3,						"SecsTim3", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&COM6.RxBuffer.Head,             "Head    ", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&COM6.RxBuffer.Tail,		        "Tail    ", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{(uint32_t)&SentSecsCmd,					"SENT",		FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{(uint32_t)&ReceivedSecsCmd ,				"REV",		FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	
//	{(uint32_t)&NumberOfCharactersReceived, "RCV# ", FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},
//	{(uint32_t)&NumberOfCharactersSent, "XMT# ",	FUNC_INT32, COLOR_WHITE, COLOR_MAGENTA, 0},

//end of the display list
	{ (uint32_t)0, "--------", FUNC_INT, COLOR_YELLOW, COLOR_MAGENTA, 0 },

};

LcdVariableInfo SerialRxBufferTable[] = {
	{ (uint32_t)1, "COM RX", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0], "RX_ 0 ", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[8], "RX_0 ", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x10], "RX_8 ", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x18], "RX_8 ", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x20], "RX_0x10", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x28], "RX_0x10", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x30], "RX_0x18", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x38], "RX_0x18", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x40], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x48], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x50], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x58], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x60], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x68], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x70], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x78], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x80], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x88], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x90], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&Rx_Buffer6[0x98], "RX_0x20", FUNC_MEMDISPASCIIHEX, COLOR_WHITE, COLOR_MAGENTA, 0 },
	{ (uint32_t)0, "--------", FUNC_INT, COLOR_YELLOW, COLOR_MAGENTA, 0 },
};
LcdVariableInfo SecsStringTable[] = {
	{ (uint32_t)1, "SECS STR", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[0], "IN  1   ", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[1], "	2	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[2], "	3	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[3], "	4	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[4],	"	5	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[5], "	6	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[6],	"	7	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[7],	"	8	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringReceiveList[8],	"	9	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[0],		"OUT 1   ", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[1], "	2	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[2], "	3	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[3], "	4	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[4], "	5	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[5], "	6	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[6], "	7	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[7], "	8	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&secsstringSendList[8], "	9	", FUNC_ASCII, COLOR_WHITE, COLOR_MAGENTA, 0 },
	{ (uint32_t)0, "--------", FUNC_INT, COLOR_YELLOW, COLOR_MAGENTA, 0 },
};

LcdVariableInfo CanRxBufferTable[] = {
	{ (uint32_t)1, "CAN RX", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.ID, "ID ", FUNC_HEX32, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.TargetAddress, "Target", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.SourceAddress, "Source", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.MsgId, "MSGID", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.MsgType, "MSGTYPE ", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.DataSize, "DataSize", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Page, "Page#", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[0], "Data 0", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[1], "Data 1", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[2], "Data 2", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[3], "Data 3", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[4], "Data 4", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[5], "Data 5", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[6], "Data 6", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastRxBuffer.Data[7], "Data 7", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
	{ (uint32_t)0, "--------", FUNC_INT, COLOR_YELLOW, COLOR_MAGENTA, 0 },
};

LcdVariableInfo CanTxBufferTable[] = {
	{ (uint32_t)1, "CAN RX", FUNC_TITLE, COLOR_RED, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.ID, "ID ", FUNC_HEX32, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.TargetAddress, "Target", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.SourceAddress, "Source", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.MsgId, "MSGID", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.MsgType, "MSGTYPE ", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.DataSize, "DataSize", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Page, "Page#", FUNC_INT16, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[0], "Data 0", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[1], "Data 1", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[2], "Data 2", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[3], "Data 3", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[4], "Data 4", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[5], "Data 5", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[6], "Data 6", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
//	{ (uint32_t)&CanLastTxBuffer.Data[7], "Data 7", FUNC_HEX8, COLOR_WHITE, COLOR_MAGENTA, 0 },
	{ (uint32_t)0, "--------", FUNC_INT, COLOR_YELLOW, COLOR_MAGENTA, 0 },
};
LcdVariableInfo UsbGcodeArguments[] = {
//	{(uint32_t)&LastMessageTail, 	    			  "Last Msg", FUNC_MEMDUMPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgX,  "      X=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgY,  "      Y=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgZ,  "      Z=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgF,  "      F=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgE,  "      E=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgM,  "      M=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgS,  "      S=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgP,  "      P=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgA,  "      A=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgB,  "      B=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgC,  "      C=", FUNC_MEMDISPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)0,              				"--------", FUNC_INT16, 	COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};

LcdVariableInfo CMDQueValues[] = {
//	{(uint32_t)&DisplayIndex,            "CmdQue  ", FUNC_INT16,	COLOR_WHITE,	COLOR_MAGENTA, 	0},          ///Offset
	{(uint32_t)&cmdQue[0].X,"      X=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[0].Y,"      Y=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[0].Z,"      Z=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[0].E,"      E=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].Y,"      Y=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].Z,"      Z=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].E,"      E=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].F,"      F=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].S,"      S=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].M,"      M=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].G,"      G=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].P,"      P=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].Q,"      Q=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].I,"      I=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)&cmdQue[3].J,"      J=", FUNC_FLOAT_QUEVAR,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
	{(uint32_t)0,           "--------", FUNC_INT16, 	COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};
LcdVariableInfo UsbUrgentGcodeArguments[] = {
//	{(uint32_t)&LastMessageTail, 	    			  "Last Msg", FUNC_MEMDUMPASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgX,  "      X=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgY,  "      Y=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgZ,  "      Z=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgF,  "      F=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgE,  "      E=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgM,  "      M=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgS,  "      S=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgP,  "      P=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgA,  "      A=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgB,  "      B=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)&COMUSB.RxBuffer.AsciiArgs.GCodeArgC,  "      C=", FUNC_ASCII,	COLOR_WHITE,	COLOR_MAGENTA, 	0},
//	{(uint32_t)0,              				"--------", FUNC_INT16, 	COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};

LcdVariableInfo TaskTimeTable1[] = {
//	{(uint32_t)&TaskTime[2],            "CNRXPRcs", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&MaxTaskTime[2],     	"Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},
//	//{(uint32_t)&TaskTime[4],      	"CNTXPRcs", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 	(uint32_t)BootStepStrings},
//	{(uint32_t)&MaxTaskTime[4],  		"Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},// i do not now which one of these to use
//	{(uint32_t)&TaskTime[1],            "Can Prcs", FUNC_INT16, 	COLOR_WHITE, 	COLOR_RED, 		0},//
//	{(uint32_t)&MaxTaskTime[1],         "Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE,		0},// //Percent must be 0 to 100.
//	{(uint32_t)&TaskTime[13],         	"Ang Calc", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&MaxTaskTime[13],        "Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},
	{(uint32_t)0,              			"--------", FUNC_INT16, 	COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};

LcdVariableInfo TaskTimeTable2[] = {
//	{(uint32_t)&TaskTime[6],            "SWrk1000", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&MaxTaskTime[6],     	"Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},
//	//{(uint32_t)&TaskTime[12],      		"SWork100", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 	(uint32_t)BootStepStrings},
//	{(uint32_t)&MaxTaskTime[12],  		"Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},// i do not now which one of these to use
//	{(uint32_t)&TaskTime[21],           "SWork10 ", FUNC_INT16, 	COLOR_WHITE,  	COLOR_RED, 		0},//
//	{(uint32_t)&MaxTaskTime[21],        "Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE,		0},// //Percent must be 0 to 100.
//	{(uint32_t)&TaskTime[28],         	"SW0rk1  ", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&MaxTaskTime[28],        "Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},
	{(uint32_t)0,              			"--------", FUNC_INT16, 	COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};

LcdVariableInfo ADCValueTable[] = {
//	{(uint32_t)&adcResultsDMA[0],            "CH00   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&adcResultsDMA[1],            "CH01   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&adcResultsDMA[2],            "CH02   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&ADCValues[3],            "CH03   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&ADCValues[4],            "CH04   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&ADCValues[5],            "CH05   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&ADCValues[6],            "CH06   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
//	{(uint32_t)&ADCValues[7],            "CH07   ", FUNC_INT16, 	COLOR_RED,	COLOR_RED, 		0},
	{(uint32_t)0,              			"--------", FUNC_INT16, 	COLOR_WHITE,	COLOR_MAGENTA, 	0},
};
float MaxFloatBarValue = 3.3;
LcdVariableInfo BarValueTable[] = {
//	{(uint32_t)&BarStatus[0],           "BAR01   ", FUNC_BAR_STATUS, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&BarStatus[1],           "BAR01   ", FUNC_BAR_STATUS, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&Tim1reload,             "BAR02   ", FUNC_BAR_DUTTY, 	COLOR_WHITE,		COLOR_LIME, 		5},
//	{(uint32_t)&BarDutty[1],            "BAR03   ", FUNC_BAR_DUTTY, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&BarDutty[2],            "BAR04   ", FUNC_BAR_DUTTY, 	COLOR_WHITE,	COLOR_BLUE, 		0},
//	{(uint32_t)&BarFloat[0],            "BAR05   ", FUNC_BAR_FLOAT, 	COLOR_WHITE,	COLOR_LIME, 		(uint32_t)&MaxFloatBarValue},
//	{(uint32_t)&BarFloat[1],            "BAR06   ", FUNC_BAR_FLOAT, 	COLOR_WHITE,	COLOR_BLUE, 		(uint32_t)&MaxFloatBarValue},
//	{(uint32_t)&BarFloat[2],            "BAR07   ", FUNC_BAR_FLOAT, 	COLOR_WHITE,	COLOR_RED, 			(uint32_t)&MaxFloatBarValue},
//	{(uint32_t)&BarFloat[3],            "BAR08   ", FUNC_BAR_FLOAT, 	COLOR_WHITE,	COLOR_LIME, 		(uint32_t)&MaxFloatBarValue},
//	{(uint32_t)0,              			"--------", FUNC_INT16, 		COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};

LcdVariableInfo FaultValueTable[] = {
//	{(uint32_t)&faultVariableInfo.TaskTime,           "TaskTime ", FUNC_INT32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.TaskSliceNumber,    "TaskSlicN", FUNC_INT32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.NumberOfFaults,     "NumOfFaul", FUNC_INT16, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.GcodeLineCounter,   "GcodeLine", FUNC_INT16, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.ParsedLineCounter,  "ParsedLin", FUNC_INT16, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.ComRxBufferPoint,   "RxBuffer ", FUNC_HEX32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.ComTxBufferPoint,   "TxBuffer ", FUNC_HEX32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.LastCharsInBuf,     "CharsInB ", FUNC_INT32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.LastMessageTail,     "MsgTail ", FUNC_HEX32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.LastRxHead,     	  "MsgHead  ", FUNC_HEX32, 	COLOR_WHITE,	COLOR_LIME, 		0},
//	{(uint32_t)&faultVariableInfo.LastRxLineCount,    "MsgHead  ", FUNC_INT32, 	COLOR_WHITE,	COLOR_LIME, 		0},
	{(uint32_t)0,              						  "---------", FUNC_INT16, 		COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};
//LcdVariableInfo TaskTimeTable1[] = {
//	{(uint32_t)&TaskTime[2],            "CNRXPRcs", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&MaxTaskTime[2],     	"Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},
//	//{(uint32_t)&TaskTime[4],      	"CNTXPRcs", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 	(uint32_t)BootStepStrings},
//	{(uint32_t)&MaxTaskTime[4],  		"Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},// i do not now which one of these to use
//	{(uint32_t)&TaskTime[1],            "Can Prcs", FUNC_INT16, 	COLOR_WHITE, 	COLOR_RED, 		0},//
//	{(uint32_t)&MaxTaskTime[1],         "Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE,		0},// //Percent must be 0 to 100.
//	{(uint32_t)&TaskTime[13],         	"Ang Calc", FUNC_INT16, 	COLOR_WHITE,	COLOR_RED, 		0},
//	{(uint32_t)&MaxTaskTime[13],        "Max Time", FUNC_INT16, 	COLOR_WHITE,	COLOR_BLUE, 	0},
//	{(uint32_t)0,              			"--------", FUNC_INT16, 	COLOR_YELLOW,	COLOR_MAGENTA, 	0},
//};

//FUNC_MEMDUMPASCII,
//FUNC_MEMDUMPHEX,

// FLASH_BASE + (gs._soapPage * FLASH_PAGE_SIZE)
LcdVariableInfo SoapStringTable[] = {																//Offset
//	{(uint32_t)SoapString,          "VAR_01   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		0},
//	{(uint32_t)SoapString,          "VAR_02   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		2},
//	{(uint32_t)SoapString,          "VAR_03   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		3},
//	{(uint32_t)SoapString,          "VAR_04   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		4},
//	{(uint32_t)SoapString,          "VAR_05   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		5},
//	{(uint32_t)SoapString,          "VAR_06   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		6},
//	{(uint32_t)SoapString,          "VAR_07   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		7},
//	{(uint32_t)SoapString,          "VAR_08   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		8},
//	{(uint32_t)SoapString,          "VAR_09   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		9},
//	{(uint32_t)SoapString,          "VAR_10   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		10},
//	{(uint32_t)SoapString,          "VAR_11   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		11},
//	{(uint32_t)SoapString,          "VAR_12   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		12},
//	{(uint32_t)SoapString,          "VAR_13   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		13},
//	{(uint32_t)SoapString,          "VAR_14   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		14},
//	{(uint32_t)SoapString,          "VAR_15   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		15},
//	{(uint32_t)SoapString,          "VAR_16   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		16},
//	{(uint32_t)SoapString,          "VAR_17   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		17},
//	{(uint32_t)SoapString,          "VAR_18   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		18},
//	{(uint32_t)SoapString,          "VAR_19   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		19},
//	{(uint32_t)SoapString,          "VAR_20   ", FUNC_ASCI_SOAP, 		COLOR_RED,	COLOR_LIME, 		20},
	{(uint32_t)0,              		"--------", FUNC_INT16, 		COLOR_YELLOW,	COLOR_MAGENTA, 	0},
};
