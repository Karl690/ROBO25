#ifndef Serial_HEADER // prevent double dipping
#define Serial_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    Serial.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains Serial specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Serial specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

#define NULL_CHAR                  '\0'    // 0
#define TERMINATE_WAIT_CHAR          1
#define PAUSE_AT_END_OF_MOVE_CHAR    2
#define PAUSE_AT_END_OF_LAYER_CHAR   3
#define AVAILABLE_4                  4  // can be harvested for Repetrel to Hydra comm
#define HELLO_WORLD_CHAR             4  // char to send to Repetrel to get repetrel to talk to Hydra over the master comm port with a PING
#define SEND_STATUS_CHAR             5
#define ASCII_ACK                    6
#define PING_CHAR                    7  // (bell)
#define ABORT_CHAR                   8  // abort all processes, and stop all motion (backspace)
#define URGENT_911_CMD_CHAR          9
#define CATASTROPHIC_ERROR_ALERT_CHAR 9
#define CMD_END_CHAR                 10 // '\n' -  used to terminate every gcode line (linefeet)
#define SENDSTRING_TERM_CHAR         10
#define LF_CHAR                      10 // '\n'
#define JOG_Z_TABLE_UP               11
#define JOG_Z_TABLE_DOWN             12
#define CR_CHAR                      13 // '\r' (carriage return)
#define REPETREL_COMM_WATCHDOG_CHAR  14
#define LIGHTBURN_STOP               0x18
#define LARGEST_SPECIAL_CHAR         31 // anything below this will end a comment string

#define COMMENT_CHAR                 ';'
#ifdef USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
#define SOAPSTRING_CHAR              '?'
#else //!USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
#define SOAPSTRING_CHAR              ';'
#endif //!USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
#define DIRECT_START_CHAR            '$'
#define RUN_LENGTH_ENCODE_CHAR       '#'
#define DIRECT_CONTINUATION_CHAR     '~'
#define SPACE_CHAR                   ' '
#define NOP_CHAR                     ' '

#define CHECKSUM_CHAR                '*'

#define LOOP_COUNT                   '!'
#define SCRIPT_LABEL                 '\''
#define STRING_DELIM                 '"'

#define RAW_CHARS_TO_PROCESS_PER_CALL 128   //  USB bursts at a very high rate
#define USB_RX_CHARS_TO_PROCESS_PER_CALL  8  //

#define MAX_STRING_SIZE              256                                         // includes null char

#define SERIAL_NORMAL_RX_HEADROOM			(2 * 1024)
#define SERIAL_DIRECT_RX_HEADROOM			(2 * 1024)
#define SERIAL_RX_RAW_USB_BUFFER_SIZE		64	//HW buffer; do not change value	// NOT in CCMRAM

#define CCMRAM_BASE_ADDR					CCMDATARAM_BASE

#define SERIAL_SAVE_FOR_FUTURE_SIZE			(768)
#define SERIAL_RX_RAW_BUFFER_SIZE			(256)
#define SERIAL_TX_NORMAL_BUFFER_SIZE		(7 * 1024)
#define SERIAL_RX_URGENT_BUFFER_SIZE		(1 * 1024)
#define SERIAL_TX_ECHO_BUFFER_SIZE			(7 * 1024)
#define SERIAL_RX_NORMAL_BUFFER_SIZE		(8 * 1024)
#define SERIAL_RASTER_LINE_BUFFER_SIZE		(8 * 1024)
#define SERIAL_RX_DIRECT_BUFFER_SIZE		(32 * 1024)

#define SERIAL_SAVE_FOR_FUTURE_ADDR			(CCMRAM_BASE_ADDR)
#define SERIAL_RX_RAW_BUFFER_ADDR			(SERIAL_SAVE_FOR_FUTURE_ADDR + SERIAL_SAVE_FOR_FUTURE_SIZE)
#define SERIAL_TX_NORMAL_BUFFER_ADDR		(SERIAL_RX_RAW_BUFFER_ADDR + SERIAL_RX_RAW_BUFFER_SIZE)
#define SERIAL_RX_URGENT_BUFFER_ADDR		(SERIAL_TX_NORMAL_BUFFER_ADDR + SERIAL_TX_NORMAL_BUFFER_SIZE)
#define SERIAL_TX_ECHO_BUFFER_ADDR			(SERIAL_RX_URGENT_BUFFER_ADDR + SERIAL_RX_URGENT_BUFFER_SIZE)
#define SERIAL_RX_NORMAL_BUFFER_ADDR		(SERIAL_TX_ECHO_BUFFER_ADDR + SERIAL_TX_ECHO_BUFFER_SIZE)
#define SERIAL_RASTER_LINE_BUFFER_ADDR		(SERIAL_RX_NORMAL_BUFFER_ADDR + SERIAL_RX_NORMAL_BUFFER_SIZE)
#define SERIAL_RX_DIRECT_BUFFER_ADDR		(SERIAL_RASTER_LINE_BUFFER_ADDR + SERIAL_RASTER_LINE_BUFFER_SIZE)

//NUKE #define RAW_RX_BUFFER_SIZE           256
//#define RAW_USB_RX_BUFFER_SIZE        64	//HW buffer; do not change value
//
//#define URGENT_RX_BUFFER_SIZE       1024
//#define NORMAL_RX_BUFFER_SIZE     0x1800    // 6K char
//
//#define DIRECT_RX_BUFFER_SIZE     0x5000    // 20K
//#define RASTER_LINE_BUFFER_SIZE   0x2000    // 8K (ie, 1000mm @ 8 dpmm)
//
//#define NORMAL_RX_HEADROOM         0x800    // 2K headroom
//#define DIRECT_RX_HEADROOM         0x800	// 2K headroom
//
//#define NORMAL_TX_BUFFER_SIZE     0x1800    // 6K char
//
//#define ECHO_TX_BUFFER_SIZE       NORMAL_TX_BUFFER_SIZE     // needs to match NORMAL tx buffer size

#define COMMENT_STRING_LENGTH   (1024+3)	//+3 for leading and trailing delim + NULL_CHAR
#define GCODE_STRING_ARG_LENGTH (1024+3)	//+3 for leading and trailing delim + NULL_CHAR

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	SERIAL_PORT_NONE    = 0,
	SERIAL_PORT_USB     = 1,
	SERIAL_PORT_UART3   = 3,
	SERIAL_PORT_UART4   = 4,
	SERIAL_PORT_UART6   = 6,
} serialPort_t;

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in Serial that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////
//NUKE
extern int  rawRxIndexIn;
extern int  rawRxIndexOut;
extern int  rawRxCharsInBuf;
//
//extern int  rawUsbRxIndexIn;
extern int  rawUsbRxIndexOut;
extern int  rawUsbRxCharsInBuf;
//
extern int  urgentRxIndexIn;
extern int  urgentRxIndexOut;
extern int  urgentRxCharsInBuf;
extern int  urgentCommandWaiting;

extern int  normalRxIndexIn;
extern int  normalRxIndexOut;
extern int  normalRxCharsInBuf;
extern int  normalCommandWaiting;

extern char *normalTxBuffer;
extern int normalTxCharsInBuf;
extern int normalTxIndexIn;
extern int normalTxIndexOut;

extern char *echoTxBuffer;
extern int echoTxCharsInBuf;
extern int echoTxIndexIn;
extern int echoTxIndexOut;

extern char SendString[];
extern char _errorStr[];
extern char _rptStr[];
extern char _tmpStr[];
extern char _canString[];
extern int  _canStringChars;
extern int _canStringRate;
extern char currentCommandString[];
extern uint32_t _sendingGBStringsMask;
extern uint32_t _sendingGBStringsSubMask;
extern serialPort_t _echoRawSerialStreamPort;
extern serialPort_t _echoProcessedSerialStreamPort;


extern masterPort_t masterCommPort;
extern int co2UartTxCharsInBuf;
extern int co2UartRxCharsInBuf;

extern int ValidCo2UartRxWatchdog;
extern uint16_t laser_PsOutputCurrent    ;
extern uint16_t laser_PsOutputVoltage    ;
extern uint16_t laser_PsControlVoltage   ;
extern uint16_t laser_PsWaterProt        ;



////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in Serial
////////////////////////////////////////////////////////////////////////////////

extern void PostAcknowledge(void);
extern void ReceiveCharacter(char);
extern void checkForUSBRxData(void);
extern void ProcessRawRxBuffer(void);

extern void SendAcknowledge(void);
extern void sendchar(char);
extern void sendCr(void);
extern void sendstring(char *);
extern void sendstringCr(char *);

extern void sendNchars(char *, uint16_t);
extern void sendMotionInfo(char, char, float, float, char *);
extern void sendMotionError(char, char, float, float, char *);
extern void sendErrorNoCr(char *);
extern void sendError(char *);
extern void sendInfoNoCr(char *);
extern void sendInfo(char *);
extern void sendDeviceInfo(char *);
extern void sendGBNoCr(char *);
extern void sendGB(char *);
extern void sendGBN(char *);
extern void sendST(char *);
extern void sendHexByte(byte);
extern void SendM0TimedOut();
extern void pauseToTransmitBufferToEmpty(void);
extern void barf(char *);
extern void reportRxUnderRun(buffer_t, char *);
extern int GCHAR (buffer_t);
extern int processCurrentScanLine(boolean);
extern void purgeTillCr(buffer_t, int);
extern void copyDataToDirectBuffer (buffer_t);
extern char *makeAllCharsPrintable(char *s);
extern boolean roomInNormalRxBuffer(void);
extern void resetUSBInputBuffer(void);
extern void resetSerialInputBuffer(void);
extern void resetSerialOutputBuffer(void);
extern boolean roomInDirectRxBuffer(void);
extern byte asciihex2bin(char c);
extern int DirectGCHAR (void);
extern void transmitEchoChar(void);
void forceCharToHw(char);
extern void PrintCheck (void);
extern void changeMasterCommPort(masterPort_t);
extern void LaserSendRequestStringToPowerSupply(void);

#endif // #ifndef Serial_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
