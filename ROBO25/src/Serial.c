////////////////////////////////////////////////////////////////////////////////
//
// File:    Serial.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: This modules contains all of the serial communication methods.
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "Serial.h" // (get our own global defines and typedefs)
#include "GCode.h"
#include "MotorDriver.h"
#include "gpio.h"
#include "usbd_usr.h"
#include "mailbox.h"

extern uint8_t USB_Rx_Buffer[];
extern void releaseUsbBuffer(void);

////////////////////////////////////////////////////////////////////////////////
//  Local #defines (defines ONLY used in this module)
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Public global definitions (exposed in Serial.h)
////////////////////////////////////////////////////////////////////////////////

// there's a 64K separate block of SRAM available called core-coupled memory.
// attempts to use the linker to place these blocks in this section worked fine
// EXCEPT CooCox's CoFlash blew up on the resulting dfu files making it unusable
// for production/customers.... SO.... reworking to use pointers to addresses in
// this memory space to circumvent the problem.

masterPort_t masterCommPort;

char *rawRxBuffer = (char *)SERIAL_RX_RAW_BUFFER_ADDR;
int  rawRxIndexIn; 			// index of where to store the next char
int  rawRxIndexOut; 		// index of where to pull the next char
int  rawRxCharsInBuf;		// total valid chars in buffer

//int  rawUsbRxIndexIn; 		// index of where to store the next char
int  rawUsbRxIndexOut; 		// index of where to pull the next char
int  rawUsbRxCharsInBuf;	// total valid chars in buffer

char *urgentRxBuffer = (char *)SERIAL_RX_URGENT_BUFFER_ADDR; 	// "911" commands go here
int  urgentRxIndexIn; 		// index of where to store the next char
int  urgentRxIndexOut; 		// index of where to pull the next char
int  urgentRxCharsInBuf;	// total valid chars in buffer
int  urgentCommandWaiting;
boolean _copyToUrgentRxBuffer;

char *normalRxBuffer = (char *)SERIAL_RX_NORMAL_BUFFER_ADDR;	// non-"911" commands go here
int  normalRxIndexIn; 		// index of where to store the next char
int  normalRxIndexOut; 		// index of where to pull the next char
int  normalRxCharsInBuf;	// total valid chars in buffer
int  normalCommandWaiting;

char *normalTxBuffer = (char *)SERIAL_TX_NORMAL_BUFFER_ADDR;
int  normalTxIndexIn;		// index of where to store the next char
int  normalTxIndexOut;		// index of where to pull the next char
int  normalTxCharsInBuf;	// total valid chars in buffer

char *echoTxBuffer = (char *)SERIAL_TX_ECHO_BUFFER_ADDR;
int  echoTxIndexIn;			// index of where to store the next char
int  echoTxIndexOut;		// index of where to pull the next char
int  echoTxCharsInBuf;		// total valid chars in buffer

char *directRxBuffer = (char *)SERIAL_RX_DIRECT_BUFFER_ADDR;
int  directRxIndexIn;		// index of where to store the next char
int  directRxIndexOut;		// index of where to pull the next char
int  directRxCharsInBuf;	// total valid chars in buffer
boolean rawDirectRxIgnoreToCmdEnd; 	// for dealing with continuation lines
boolean _copyToDirectRxBuffer;

const char *rasterLine = (char *)SERIAL_RASTER_LINE_BUFFER_ADDR;
#ifdef GB_HIDDEN_WARNINGS
#warning "update raster code to use this (rasterLine)"
#endif //GB_HIDDEN_WARNINGS

boolean _processingAComment;
boolean _processingASoapString;

char currentCommandString[MAX_STRING_SIZE];
char SendString[MAX_STRING_SIZE];
char _errorStr[MAX_STRING_SIZE];
char _rptStr[MAX_STRING_SIZE];
char _tmpStr[MAX_STRING_SIZE];


char _canString[MAX_CAN_STRING_SIZE];
int _canStringChars;
int _canStringRate;

uint32_t _sendingGBStringsMask;
uint32_t _sendingGBStringsSubMask;
serialPort_t _echoRawSerialStreamPort;
serialPort_t _echoProcessedSerialStreamPort;

int ValidCo2UartRxWatchdog = 10;
uint16_t laser_PsOutputCurrent       =0;
uint16_t laser_PsOutputVoltage       =0;
uint16_t laser_PsControlVoltage      =0;
uint16_t laser_PsWaterProt           =0;


int co2UartTxCharsInBuf = 0;
int  co2UartRxCharsInBuf = 0;

#define CO2_UART_TX_REQUEST_STRING_SIZE 12 // must be exact size of tx string
#define CO2_UART_RX_RETURN_STRING_SIZE 18 // must be exactly the size of the return string from the CO2 Laser PS + 1 (for NULL_CHAR)
char co2UartRxBuffer[CO2_UART_RX_RETURN_STRING_SIZE]; // raw data from UARTS

//'U'+'5'+0+0+138+'\r' + 'U'+'5'+1+34+173'\r'; -- NO NULL_CHAR
static const char co2UartTxBuffer[CO2_UART_TX_REQUEST_STRING_SIZE] = "\x55\x35\x0\x0\x8a\xd\x55\x35\x1\x22\xad\xd";


////////////////////////////////////////////////////////////////////////////////
//  Local global definitions (do not expose in Serial.h)
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Forward declarations - any local modules needing an early template
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/* implementation of putchar (also used by printf function to output data)    */

void addCharToEchoBuffer (char ch)
{ // add char to outgoing buffer for echo'd chars
	if (echoTxCharsInBuf < (SERIAL_TX_ECHO_BUFFER_SIZE-1))
	{   // room available
		*(echoTxBuffer + echoTxIndexIn) = ch;
		protectedIncrement(&echoTxCharsInBuf);

		echoTxIndexIn++;
		echoTxIndexIn %= SERIAL_TX_ECHO_BUFFER_SIZE;

		transmitEchoChar(); // try to free up one spot in the buffer
	}
}

////////////////////////////////////////////////////////////////////////////////

/* implementation of putchar (also used by printf function to output data)    */
void sendchar (char ch)
{ // add char to outgoing buffer
	if ((normalTxCharsInBuf < (SERIAL_TX_NORMAL_BUFFER_SIZE-1)) && (masterCommPort != BOOTUP))
	{   // room available
		*(normalTxBuffer + normalTxIndexIn) = ch;
		protectedIncrement(&normalTxCharsInBuf);
		normalTxIndexIn++;
		normalTxIndexIn %= SERIAL_TX_NORMAL_BUFFER_SIZE;
#ifdef COLLECT_METRICS
		_metrics.total_charsTx++;
		if (normalTxCharsInBuf > _metrics.max_normalTxCharsInBuf)
			_metrics.max_normalTxCharsInBuf = normalTxCharsInBuf;
#endif
	}
	else
	{   // rejecting character (no room)
		_rejected_normalTxChars++;
	}
}

void sendCr(void)
{
	sendchar(SENDSTRING_TERM_CHAR);
}

void sendstring(char *stringToSend)
{
	int count;
	for(count=0;count<MAX_STRING_SIZE;count++)
	{
		if (stringToSend[count] == NULL_CHAR)   //look for end of string
		{
			break;
		}
		if (stringToSend[count] == ASCII_ACK)
		{   // prevent a string from inadvertantly adding an 'ACK' char to the buffer.
			continue;   // skip ack's
		}
		sendchar(stringToSend[count]);
	}
}


void sendstringCr(char *stringToSend)       //sends the string terminated with LF/CR
{
	sendstring(stringToSend);
	sendCr();
}

void sendNchars(char *stringToSend, uint16_t numChars)
{
	uint16_t count;
	for(count=0;count<numChars;count++)
	{
		if (stringToSend[count] < 20)
			sendchar('*');  // replace lower control chars which can screw up repetrel
		else
			sendchar(stringToSend[count]);
	}
}

void sendMotionInfo(char label, char arg1, float arg2, float arg3, char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	char tmpStr[80];
	sprintf(tmpStr, ">IN:%c:%c:%4.3f:%4.3f:N%ld:T%4.3f:", label, arg1, arg2, arg3, getCurrentExecutingLineNumber(), ((float)_gs._milliseconds)/1000.0f);
	sendstring(tmpStr);
	sendstringCr(s);
}

void sendMotionError(char label, char arg1, float arg2, float arg3, char *s)
{
	_gs._flasher.error = FLASHER_ON_COUNT;
	if ( _gs._errorThrottledCount < MAX_ERROR_MESSAGES)
	{
		char tmpStr[80];
		sendstringCr(">ER:  0: Motion Error");
		sprintf(tmpStr, ">ER:%c:%c:%4.3f:%4.3f:N%ld:T%4.3f:", label, arg1, arg2, arg3, getCurrentExecutingLineNumber(), ((float)_gs._milliseconds)/1000.0f);
		sendstring(tmpStr);
		sendstringCr(s);
		_gs._errorThrottledCount+=2;
		_gs._errorCount+=2;
	}
}

void sendSystemErrorHeader(void)
{
	char localStr[32];
	sprintf(localStr, ">ER:  0: N=%ld: ", getCurrentExecutingLineNumber());
	sendstring(localStr);
}

void sendErrorNoCr(char *s)
{
	_gs._flasher.error = FLASHER_ON_COUNT;
	if ( _gs._errorThrottledCount < MAX_ERROR_MESSAGES)
	{
		sendSystemErrorHeader();
		sendstring(s);
		_gs._errorThrottledCount++;
		_gs._errorCount++;
	}
}

void sendError(char *s)
{
	_gs._flasher.error = FLASHER_ON_COUNT;
	if ( _gs._errorThrottledCount < MAX_ERROR_MESSAGES)
	{
		sendSystemErrorHeader();
		sendstringCr(s);
		_gs._errorThrottledCount++;
		_gs._errorCount++;
	}
}

void sendLineNumber(void)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	char tmpstr[16];
	sprintf(tmpstr, "N%ld: ", getCurrentExecutingLineNumber());
	sendstring(tmpstr);
}

void sendInfoNoCr(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">IN:  0: ");
	sendstring(s);
}
void sendInfo(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">IN:  0: ");
	sendstringCr(s);
}

void sendDeviceInfo(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">IN: ");
	sendstringCr(s);
}

void sendGBNoCr(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">GB: ");
	sendstring(s);
}

void sendGB(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">GB: ");
	sendstringCr(s);
}

void sendGBN(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">GB: ");
	sendLineNumber();
	sendstringCr(s);
}

void sendST(char *s)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sendstring(">ST:");
	sendstringCr(s);
}

void sendHexByte(byte value)
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled) return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sprintf(_tmpStr, "%02x", value);
	sendstring(_tmpStr);
}

////////////////////////////////////////////////////////////////////////////////

void pauseToTransmitBufferToEmpty(void)
{
	while (normalTxCharsInBuf)
	{
		PrintCheck();
	}
}

void barf(char *s)
{
	// something really bad has occurred.  program has detected it's own programming error or unexplained event
	// send an error and stay in this routine until the message has cleared the serial port (will be blocking to
	// some extent)
	__disable_irq(); // avoid collision with other processes
	char localStr[256];
	sprintf(localStr, "BARF: FATAL: %s (MOTION CONTROLLER WILL RESET)", s);
	catastrophicError(localStr);
	pauseToTransmitBufferToEmpty();
	Crash_Handler("Barfing"); //NVIC_SystemReset();
}

void reportRxUnderRun(buffer_t useBuffer, char *error)
{
	if (useBuffer == URGENT_BUFFER)
	{
		if (_errors.sent.flags.urgentRxBufferUnderrun == FALSE)
		{
			sprintf(_errorStr, "URGENT RX BUFFER UNDERRUN (%s)", error);
			sendError(_errorStr);
			_errors.sent.flags.urgentRxBufferUnderrun = TRUE;
		}
	}
	else if (useBuffer == NORMAL_BUFFER)
	{
		if (_errors.sent.flags.normalRxBufferUnderrun == FALSE)
		{
			sprintf(_errorStr, "NORMAL RX BUFFER UNDERRUN (%s)", error);
			sendError(_errorStr);
			_errors.sent.flags.normalRxBufferUnderrun = TRUE;
		}
	}
	else
	{
		sprintf(_errorStr, "UNKNOWN BUFFER UNDERRUN (%s)", error);
		sendError(_errorStr);
	}
}

int GCHAR (buffer_t useBuffer)
{   //this routine will check the input buffer and see if it has data
	// return value is an int so that ALL legal char values (0 to 255) can be returned AS WELL AS
	// return a -1 for cases where no data is available.
	int returnVal = -1; // default is no char available

	if (useBuffer == NORMAL_BUFFER)
	{
		if (normalRxCharsInBuf)
		{   // data available
			returnVal = *(normalRxBuffer + normalRxIndexOut);
			normalRxIndexOut++;
			normalRxIndexOut %= SERIAL_RX_NORMAL_BUFFER_SIZE; //wrap index
			protectedDecrement(&normalRxCharsInBuf);
		}
	}
	else if (useBuffer == URGENT_BUFFER)
	{
		if (urgentRxCharsInBuf)
		{   // data available
			returnVal = *(urgentRxBuffer + urgentRxIndexOut);
			urgentRxIndexOut++;
			urgentRxIndexOut %= SERIAL_RX_URGENT_BUFFER_SIZE; //wrap index
			protectedDecrement(&urgentRxCharsInBuf);
		}
	}
	else if (useBuffer == SOAPSTRING_BUFFER)
	{
		if (_sysInfoPtr->soapReadPtr < (byte *)(_sysInfoPtr->soapBaseAddr + _sysInfoPtr->soapSize))
		{   // data available
			returnVal = *_sysInfoPtr->soapReadPtr;
			if (returnVal == NULL_CHAR)
			{   // convert end of string to end of command as stored commands in the soap are missing the end of cmd char
				returnVal = CMD_END_CHAR;
			}
			_sysInfoPtr->soapReadPtr++;
		}
	}

	if ((_echoProcessedSerialStreamPort != SERIAL_PORT_NONE) && (returnVal != -1))
	{
		addCharToEchoBuffer(returnVal);
	}
	return(returnVal);
}

////////////////////////////////////////////////////////////////////////////////

int DirectGCHAR (void)
{
	int returnVal;

	if (directRxIndexIn==directRxIndexOut)
	{
		if (_errors.sent.flags.directRxBufferUnderrun == FALSE)
		{
			sendError("DIRECT RX BUFFER UNDERRUN  (DirectGCHAR)");
			_errors.sent.flags.directRxBufferUnderrun = TRUE;
		}
		returnVal = CMD_END_CHAR;  // will result in "laser off"
	}
	else
	{
		if (directRxIndexOut == _gs._laser.rasterImageLineCmdEndIndex)
			returnVal = -1;
		else
			returnVal = (int)(unsigned int)*(directRxBuffer + directRxIndexOut);
		directRxIndexOut++;
		directRxIndexOut %= SERIAL_RX_DIRECT_BUFFER_SIZE;
		protectedDecrement(&directRxCharsInBuf);
	}
	return(returnVal);
}

////////////////////////////////////////////////////////////////////////////////

boolean roomInDirectRxBuffer(void)
{
	if ((SERIAL_RX_DIRECT_BUFFER_SIZE - directRxCharsInBuf) > SERIAL_DIRECT_RX_HEADROOM)
		return(TRUE);
	else
		return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

byte asciihex2bin(char c)
{
	if ((c >= '0') && (c <= '9')) {
		return(c - '0');
	}
	else if ((c >= 'A') && (c <= 'F')) {
		return(c - 'A' + 10);
	}
	else if ((c >= 'a') && (c <= 'f')) {
		return(c - 'a' + 10);
	}
	return(0x0);
}

////////////////////////////////////////////////////////////////////////////////

#define INDEX_INCREMENT(a, b) {a++; a %= b;}
#define INDEX_DECREMENT(a, b) {a = (a==0) ? (b-1) : (a-1);}
#define ASCII_HEX_NIBBLE_TO_INT(a)      (((a>='0')&&(a<='9')) ? (a-'0') : ((a>='a')&&(a<='f')) ? (a-('a'-10)) : ((a>='A')&&(a<='F')) ? (a-('A'-10)) : 0)

#define min(a,b) (((a) < (b)) ? (a) : (b))

const unsigned int rasterFlipBit1[16] = {0,8,4,0xc,2,0xa,6,0xe,1,9,5,0xd,3,0xb,7,0xf};
const unsigned int rasterFlipBit2[16] = {0,4,8,0xc,1,5,9,0xd,2,6,0xa,0xe,3,7,0xb,0xf};

typedef struct {
	int buffIndex;
	int lineIndex;
} indexStruct;

typedef struct {
	int first;
	int last;
} firstLastStruct;

////////////////////////////////////////////////////////////////////////////////

boolean isARasterPrintingValue(int binLeft, int binRight, int binBuffIndex)
{
	int binData = (binLeft << 4) | binRight;    // reconstruct input data (no flips/reverses

	if (_gs._laser.rasterSkipBlankBorders == FALSE)
		return(TRUE);   // not optimizing, so print everything
	else if ((binBuffIndex != (_gs._laser.rasterBytesPerFullLine-1)) && (binData != _gs._laser.rasterOffValue))
		return(TRUE);   // anywhere on line but the last value
	else if ((binBuffIndex == (_gs._laser.rasterBytesPerFullLine-1)) && (binData != _gs._laser.rasterOffValueLastWord))
		return(TRUE);   // last value on a line
	else
		return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

int processCurrentScanLine(boolean reverse)
{   // returns the count of pixels in the current line to process into the motionQ
	// as well as reverses the data if needed for bidirectional scanning
	// additionally sets values for first and last non-zero values in case it's desireable
	// to skip over blank borders (fully blank line will be signified by "first == -1")

	int retValue = 0;
	indexStruct ascii;      // tracking raw ascii data in buffer
	indexStruct binary;     // tracking raw binary data in buffer
	indexStruct binFirst;   // tracking location of first printing value
	indexStruct binLast;    // tracking location of last printing value

	unsigned int binLeft = 0;
	unsigned int binRight;  // temp vars to help build binary values
	unsigned int binData;

	ascii.buffIndex = directRxIndexOut;
	ascii.lineIndex = 0; // count of ascii chars read on this line, excludes CMD_END_CHAR

	binary.buffIndex = directRxIndexOut;    // convert ascii to binary as we go (storage pointer since not 1 to 1 map for 8 bit case)
	binary.lineIndex = 0; // count of binary values in buffer (converted from ascii)

	binFirst.lineIndex = -1;
	binFirst.buffIndex = -1; // marks index of first printing binary value
	binLast.lineIndex = -1;
	binLast.buffIndex = -1; // marks index of last printing binary value

	while ((ascii.lineIndex < SERIAL_RX_DIRECT_BUFFER_SIZE) && (ascii.lineIndex <= directRxCharsInBuf))
	{ // get the line length
		if (*(directRxBuffer + ascii.buffIndex) == CMD_END_CHAR)
		{   // found the end
			_gs._laser.rasterImageLineCmdEndIndex = ascii.buffIndex; // save a marker for the END_CHAR
			break;
		}

		if ((ascii.lineIndex % 2) == 0)
		{
			binLeft = ASCII_HEX_NIBBLE_TO_INT(*(directRxBuffer + ascii.buffIndex)); // save first/left char of pair
		}
		else
		{   // now have both halves so proceed
			binRight = ASCII_HEX_NIBBLE_TO_INT(*(directRxBuffer + ascii.buffIndex)); // or in second/right char of pair

			if (reverse && (_gs._laser.rasterBitsPerDot == 1))
				binData = (rasterFlipBit1[binRight] << 4) | rasterFlipBit1[binLeft];
			else if (reverse && (_gs._laser.rasterBitsPerDot == 2))
				binData = (rasterFlipBit2[binRight] << 4) | rasterFlipBit2[binLeft];
			else if (reverse && (_gs._laser.rasterBitsPerDot == 4))
				binData = (binRight << 4) | binLeft;
			else
				binData = (binLeft << 4) | binRight;

			if (isARasterPrintingValue(binLeft, binRight, binary.buffIndex))
			{
				if (binFirst.lineIndex == -1)
				{   // finally have a printing char
					binFirst.lineIndex = binary.lineIndex;
					binFirst.buffIndex = binary.buffIndex;
				}
				binLast.lineIndex = binary.lineIndex;
				binLast.buffIndex = binary.buffIndex;
			}
			*(directRxBuffer + binary.buffIndex) = binData; // save the converted data (write over ascii)
			INDEX_INCREMENT(binary.buffIndex, SERIAL_RX_DIRECT_BUFFER_SIZE);
			binary.lineIndex++;
		}
		INDEX_INCREMENT(ascii.buffIndex, SERIAL_RX_DIRECT_BUFFER_SIZE);
		ascii.lineIndex++;
	}

	// at this point:
	//      directRxIndexOut points to first char
	//      ascii.buffIndex points to the CMD_END_CHAR; (one past last ascii char)
	//      binary.buffIndex points one past last binary value
	//      ascii.lineIndex has the count of ascii chars (nibbles)
	//      binary.lineIndex has the count of binary values saved
	//      data has been converted to binary AND flipped within the byte if needed for reverse path
	//      first/last have the correct binary positions (count) on the scan line

	if ((ascii.lineIndex > directRxCharsInBuf) || (ascii.lineIndex == SERIAL_RX_DIRECT_BUFFER_SIZE))
	{   // error -- did not find end_char
		catastrophicError("RASTER BUFFER: missing CMD_END_CHAR");
		retValue = 0;
	}
	else if ((binary.lineIndex * _gs._laser.rasterImageDotsPerBinaryValue) > _gs._laser.rasterImageDotsXmod)
	{
		catastrophicError("RASTER BUFFER: input data line length exceeds M624 U value");
		retValue = 0;
	}
	else if (binFirst.lineIndex == -1)
	{   //empty line, so get buffer ready for next call
		directRxIndexOut = _gs._laser.rasterImageLineCmdEndIndex;
		directRxIndexOut++; // get past CMD_END_CHAR
		directRxIndexOut %= SERIAL_RX_DIRECT_BUFFER_SIZE;
		protectedDecrement(&directRxCharsInBuf);
	}
	else if ((ascii.lineIndex % 2) == 1)
	{
		catastrophicError("RASTER BUFFER: input data needs it be in bytes converted to 2-char ascii hex (odd # of chars encountered)");
		retValue = 0;
	}
	else
	{   // line with data found

		// ascii data compressed into half the space in the direct buffer as binary data, but in the
		// left half of the line.  move data to the right half, reversing order if needed for reverse
		// path with the end result of  aligning to the CMD_END_CHAR
		//
		// during this process, also drop non-printing values (before "first" and after "last")

		int destIndex = _gs._laser.rasterImageLineCmdEndIndex; // starts at the CMD_END_CHAR (will fill backwards)
		int sourceIndex = (reverse) ? binFirst.buffIndex : binLast.buffIndex; // starts first/last non-zero binary value
		int i;
		for (i=0; i<=(binLast.lineIndex - binFirst.lineIndex); i++) // total non-blank values
		{
			INDEX_DECREMENT(destIndex, SERIAL_RX_DIRECT_BUFFER_SIZE);   // pre-decr
			*(directRxBuffer + destIndex) = *(directRxBuffer + sourceIndex);
			if (reverse)
			{
				INDEX_INCREMENT(sourceIndex, SERIAL_RX_DIRECT_BUFFER_SIZE);
			}
			else
			{
				INDEX_DECREMENT(sourceIndex, SERIAL_RX_DIRECT_BUFFER_SIZE);
			}
		}

		directRxIndexOut = destIndex;   // move rx index to new first value (frees up 50% of the line in the buffer)

		int nonBlankBinValues = (binLast.lineIndex - binFirst.lineIndex + 1);

		uint32_t irq_disabled = interruptsOff();
		directRxCharsInBuf -= (ascii.lineIndex - nonBlankBinValues);    // reduce by amount of emptied spaces
		interruptsOn(irq_disabled);

		if (!reverse)
		{   // FORWARD
			_gs._laser.rasterFirstTraverseDot = binFirst.lineIndex * _gs._laser.rasterImageDotsPerBinaryValue;
			_gs._laser.rasterLastTraverseDot = ((binLast.lineIndex + 1) * _gs._laser.rasterImageDotsPerBinaryValue) - 1;
			_gs._laser.rasterFirstActiveDot = _gs._laser.rasterFirstTraverseDot;
			_gs._laser.rasterLastActiveDot = imin(_gs._laser.rasterLastTraverseDot, _gs._laser.rasterImageDotsX - 1);
		}
		else
		{   // REVERSE
			_gs._laser.rasterFirstTraverseDot = _gs._laser.rasterImageDotsXmod - ((binLast.lineIndex + 1) * _gs._laser.rasterImageDotsPerBinaryValue);
			_gs._laser.rasterLastTraverseDot = (_gs._laser.rasterImageDotsXmod - 1) - (binFirst.lineIndex * _gs._laser.rasterImageDotsPerBinaryValue);
			_gs._laser.rasterFirstActiveDot = imax(_gs._laser.rasterFirstTraverseDot, (_gs._laser.rasterImageDotsXmod - _gs._laser.rasterImageDotsX));
			_gs._laser.rasterLastActiveDot = imin(_gs._laser.rasterLastTraverseDot, _gs._laser.rasterImageDotsXmod - 1);
		}

		retValue = nonBlankBinValues * _gs._laser.rasterImageDotsPerBinaryValue;    // return actual dots in the binary data on the current line (excluding border blanks)
	}

	return(retValue);
}

////////////////////////////////////////////////////////////////////////////////

void purgeTillCr (buffer_t useBuffer, int ParseIndex)
{//this will read to the end of receive que, or until it finds a new line symbol 0xA   10
	int rawChar = 0;    // MUST be of type "int" to handle char 0 to 255 AND the -1 "no data" return value

	while ((rawChar != CMD_END_CHAR) && (rawChar != -1))
	{
		rawChar = GCHAR(useBuffer);

		if (rawChar == -1)
		{
			// underRun .... should never happen here as a full command was needed to get this far
			reportRxUnderRun(useBuffer, "purgeTillCr");
		}

		else if ((rawChar != CMD_END_CHAR) && (ParseIndex < (MAX_STRING_SIZE-2)))
		{
			if (ParseIndex != -1)
			{   // -1 is a special case where we do not need to keep the purged data
				ParseIndex++;
				currentCommandString[ParseIndex] = rawChar;
				currentCommandString[ParseIndex+1] = '\0';  // set the null term char for the string
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void resetSerialInputBuffer(void)
{
#ifdef COLLECT_METRICS
	if (_abortInProgress)
	{
		_metrics.flushedRxCharsDuringAbort = rawRxCharsInBuf;
	}
#endif

	uint32_t irq_disabled = interruptsOff();
	rawRxIndexIn = 0;
	rawRxIndexOut = 0;
	rawRxCharsInBuf = 0;

	_copyToUrgentRxBuffer = FALSE;
	urgentRxIndexIn = 0;
	urgentRxIndexOut = 0;
	urgentRxCharsInBuf = 0;
	urgentCommandWaiting = 0;

	normalRxIndexIn = 0;
	normalRxIndexOut = 0;
	normalRxCharsInBuf = 0;
	normalCommandWaiting = 0;

	//NUKE -- debugging   memset((char *)CCMRAM_ADDR, 0x69, 65536);

	pendingAcknowledge = 0;
	_gcodeCmdsReceived = 0;
	_gcodeAcksSent = 0;
	_asciiChecksum32=0;
	_asciiCharsRcvd=0;
	_rejected_normalTxChars=0;
	normalRxBufHeadroom = SERIAL_NORMAL_RX_HEADROOM;

	_canStringChars = 0;
	_canStringRate = 0;

	_copyToDirectRxBuffer = FALSE;
	directRxIndexIn = 0;
	directRxIndexOut = 0;
	directRxCharsInBuf = 0;
	rawDirectRxIgnoreToCmdEnd = FALSE;

	_processingAComment = FALSE;
	_processingASoapString = FALSE;

	_echoRawSerialStreamPort = 0;
	_echoProcessedSerialStreamPort = SERIAL_PORT_NONE;

	interruptsOn(irq_disabled);
}

void resetSerialOutputBuffer(void)
{
	uint32_t irq_disabled = interruptsOff();
	normalTxIndexIn = 0;
	normalTxIndexOut = 0;
	normalTxCharsInBuf = 0;

	echoTxIndexIn = 0;
	echoTxIndexOut = 0;
	echoTxCharsInBuf = 0;

	interruptsOn(irq_disabled);
}

////////////////////////////////////////////////////////////////////////////////

char *makeAllCharsPrintable(char *s)
{
	char *p = s;
	int i;

	for (i=0; i<MAX_STRING_SIZE; i++)
	{
		if (*p == '\0')
		{
			break;
		}
		else
		{
			if ((*p < ' ') || (*p > '}'))
				*p = '~'; // largest value printable char
			p++;
		}
	}
	return(s);
}

////////////////////////////////////////////////////////////////////////////////

boolean roomInNormalRxBuffer(void)
{
	if ((SERIAL_RX_NORMAL_BUFFER_SIZE - normalRxCharsInBuf) > normalRxBufHeadroom)
		return(TRUE);
	else
		return(FALSE);
}


////////////////////////////////////////////////////////////////////////////////

void PostAcknowledge(void)
{
	pendingAcknowledge++;   //indicate an ACK needs to be sent. no need to protect as all routines called from sysTick
	_gs._flasher.ack_pending = FLASHER_ON_COUNT;
}

////////////////////////////////////////////////////////////////////////////////

void ReceiveCharacter(char chr)
{
	if (processSoapstringCommands)
		return;   // wait until finished booting
	if (rawRxCharsInBuf == SERIAL_RX_RAW_BUFFER_SIZE)
	{
		if (_errors.sent.flags.rawRxBufferOverrun == FALSE)
		{
			sprintf(_errorStr, "RAW RX BUFFER OVERRUN (ReceiveCharacter %d/%d/%d/%d)",
					rawRxCharsInBuf, normalRxCharsInBuf, urgentRxCharsInBuf, directRxCharsInBuf );
			catastrophicError(_errorStr);
			_errors.sent.flags.rawRxBufferOverrun = TRUE;
		}
	}
	else if (_abortInProgress)
	{  // if aborting, then drop incoming chars (but track how many)
#ifdef COLLECT_METRICS
		_metrics.flushedRxCharsDuringAbort++;
#endif
	}
	else
	{
		*(rawRxBuffer + rawRxIndexIn) = chr;
		rawRxIndexIn++;
		rawRxIndexIn %= SERIAL_RX_RAW_BUFFER_SIZE; //wrap index
		protectedIncrement(&rawRxCharsInBuf);

		if (_echoRawSerialStreamPort != SERIAL_PORT_NONE)
		{
			addCharToEchoBuffer(chr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void addCharToNormalBuffer(char rawChar)
{
	if (normalRxCharsInBuf == SERIAL_RX_NORMAL_BUFFER_SIZE) {
		if (_errors.sent.flags.normalRxBufferOverrun == FALSE)
		{
			sendError("NORMAL RX BUFFER OVERRUN (addCharToNormalBuffer)");
			_errors.sent.flags.normalRxBufferOverrun = TRUE;
		}
	}
	else
	{
		*(normalRxBuffer + normalRxIndexIn) = rawChar;
		normalRxIndexIn++;
		normalRxIndexIn %= SERIAL_RX_NORMAL_BUFFER_SIZE; //wrap index
		protectedIncrement(&normalRxCharsInBuf);
		if (rawChar == CMD_END_CHAR)
		{
#ifdef ALLOW_NATIVE_LIGHTBURN
			if (!_lightburnModeEnabled)
			{
				PostAcknowledge();
			}
#else //!ALLOW_NATIVE_LIGHTBURN
			PostAcknowledge();
#endif //!ALLOW_NATIVE_LIGHTBURN
			protectedIncrement(&normalCommandWaiting);
		}
#ifdef COLLECT_METRICS
		if (normalRxCharsInBuf > _metrics.max_normalRxCharsInBuf)
			_metrics.max_normalRxCharsInBuf = normalRxCharsInBuf;
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////

void addCharToUrgentBuffer(char rawChar)
{
	if (urgentRxCharsInBuf >= SERIAL_RX_URGENT_BUFFER_SIZE) {
		if (_errors.sent.flags.urgentRxBufferOverrun == FALSE)
		{
			sendError("URGENT RX BUFFER OVERRUN (addCharToUrgentBuffer)");
			_errors.sent.flags.urgentRxBufferOverrun = TRUE;
		}
	}
	else
	{
		*(urgentRxBuffer + urgentRxIndexIn) = rawChar;
		urgentRxIndexIn++;
		urgentRxIndexIn %= SERIAL_RX_URGENT_BUFFER_SIZE; //wrap index
		protectedIncrement(&urgentRxCharsInBuf);
		if (rawChar == CMD_END_CHAR)
		{
#ifdef ALLOW_NATIVE_LIGHTBURN
			if (!_lightburnModeEnabled)
			{
				PostAcknowledge(); // Blindly send ack -- no flow control on urgent commands
			}
#else //!ALLOW_NATIVE_LIGHTBURN
			PostAcknowledge(); // Blindly send ack -- no flow control on urgent commands
#endif //!ALLOW_NATIVE_LIGHTBURN
			_copyToUrgentRxBuffer = FALSE; // cancel urgency at end of command
			protectedIncrement(&urgentCommandWaiting);
		}
#ifdef COLLECT_METRICS
		if (urgentRxCharsInBuf > _metrics.max_urgentRxCharsInBuf)
			_metrics.max_urgentRxCharsInBuf = urgentRxCharsInBuf;
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////

void addCharToDirectBuffer(char rawChar)
{   //at this point incoming chars are raster data...need to deal with continuation lines here
	//the raw raster data is sent with approx 80 values per line, each starting with a $ and ending
	//with a CMD_END_CHAR.  however, in order to get longer than 80 values, a continuation char is
	//supported.  if the DIRECT_CONTINUATION_CHAR immediately preceeds the CMD_END_CHAR, then the
	//host is sent an ack, but both the DIRECT_CONTINUATION_CHAR and CMD_END_CHAR are dropped
	//from the input buffer and the CMD_END_CHAR does not trigger the end of command.  only
	//when a CMD_END_CHAR that was not preceeded by the DIRECT_CONTINUATION_CHAR will the end of
	//command actually be tagged that a command is ready to process.

	if (rawDirectRxIgnoreToCmdEnd)
	{   // don't add to direct buffer
		if (rawChar == CMD_END_CHAR)
		{
#ifdef ALLOW_NATIVE_LIGHTBURN
			if (!_lightburnModeEnabled)
			{
				PostAcknowledge();  // host still needs a per line acknowledge
			}
#else //!ALLOW_NATIVE_LIGHTBURN
			PostAcknowledge();  // host still needs a per line acknowledge
#endif //!ALLOW_NATIVE_LIGHTBURN
			rawDirectRxIgnoreToCmdEnd = FALSE;
		}
	}
	else if (rawChar == DIRECT_CONTINUATION_CHAR)
	{   // don't add to continuation char to direct buffer
		rawDirectRxIgnoreToCmdEnd = TRUE;
	}
	else
	{   // save the current char to the buffer
		if (directRxCharsInBuf == SERIAL_RX_DIRECT_BUFFER_SIZE) {
			if (_errors.sent.flags.directRxBufferOverrun == FALSE)
			{
				sendError("DIRECT RX BUFFER OVERRUN (addCharToDirectBuffer)");
				_errors.sent.flags.directRxBufferOverrun = TRUE;
			}
		}
		else
		{
			*(directRxBuffer + directRxIndexIn) = rawChar;
			directRxIndexIn++;
			directRxIndexIn %= SERIAL_RX_DIRECT_BUFFER_SIZE;  //wrap index
			protectedIncrement(&directRxCharsInBuf);
		}
		if (rawChar == CMD_END_CHAR)
		{   // end of the direct command
			//NO!!!!  do not send a PostAcknowledge(); // host  needs a per line acknowledge
			//The processing of the inserted G1.1 will send the ack
			_copyToDirectRxBuffer = FALSE;
			addCharToNormalBuffer('G'); // leave a breadcrumb in the normal stream to mark where to draw the next raster line
			addCharToNormalBuffer('1'); // leave a breadcrumb in the normal stream to mark where to draw the next raster line
			addCharToNormalBuffer('.'); // leave a breadcrumb in the normal stream to mark where to draw the next raster line
			addCharToNormalBuffer('1'); // leave a breadcrumb in the normal stream to mark where to draw the next raster line
			addCharToNormalBuffer(CMD_END_CHAR);    // leave a breadcrumb in the normal stream to mark where to draw the next raster line
#ifdef GB_ACK_SENT_PIN
			sprintf(_tmpStr,"DB: num:%d in:%d out:%d  proc:%d pending=%d acks=%d\n", directRxCharsInBuf, directRxIndexIn, directRxIndexOut, directRxIndexCurrentProcess,
					pendingAcknowledge, _debug_acksSent);
			sendInfo(_tmpStr);
#endif
		}
#ifdef COLLECT_METRICS
		if (directRxCharsInBuf > _metrics.max_directRxCharsInBuf)
			_metrics.max_directRxCharsInBuf = directRxCharsInBuf;
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////

void ProcessRawRxChar(char rawChar)
{
	HostConnectionWatchDog = HOST_WATCHDOG_TIMEOUT_MS;  // reset the host watchdog

	_asciiChecksum32 += rawChar;
	_asciiCharsRcvd++;

	switch(rawChar)
	{  // check for any special characters
	case TERMINATE_WAIT_CHAR:      //if (rawChar==1)
		//kill/reset and wait timers
		_g4DwellTimer = 0;//turn off any processing wait timer so print can continue
		_requestToPauseAtEndOfMove = FALSE;//turn off potential waits as well
		_requestToPauseAtEndOfLayer = FALSE;
		_requestToAbortAtEndOfMove = FALSE;
		_gcodePaused = FALSE;
		_MailBoxes._waitingFor.flags.u32 = 0;
		sendchar(TERMINATE_WAIT_CHAR);  // echo back to say term_wait has been processed

		if (motionQ_notEmpty())
		{ // restart the motionQ
			//motionQ_executeMove();
			motionQ_setCountdownDelayToExecuteMove(1);  // will execute move on next call to Sequence Engine
		}
		break;

	case PAUSE_AT_END_OF_MOVE_CHAR:     //if (rawChar==2)
		//set flag to stop at end of this move
		_requestToPauseAtEndOfMove = TRUE;
		break;

	case PAUSE_AT_END_OF_LAYER_CHAR:     //if (rawChar==3)
		// set flag to stop at end of this layer
		_requestToPauseAtEndOfLayer = TRUE;  //(when M790 is executed
		break;

	case AVAILABLE_4:    //if (rawChar==4)
		break;

	case SEND_STATUS_CHAR:     //if (rawChar==5)
		// send live status on health of motion controller
		M_Code_M775();
		break;

	case ASCII_ACK: //if (rawChar==6)
		// used to reset ack/cmd tracking for debug (normally ONLY sent by MC to Repetrel)
		_gcodeCmdsReceived = 0;
		_gcodeAcksSent = 0;
		_asciiChecksum32 = 0;
		_asciiCharsRcvd = 0;
		_gcodeLineNumber = 0;
		//pendingAcknowledge = 0; -- while tempting to do, will screw up command stream, use ABORT_CHAR to reset all 3 of these
		break;

	case PING_CHAR:     //if (rawChar==7)
		// reply to the query and inform the inquisitor that we are alive and well
		pingReply() ;
		break;

#ifdef ALLOW_NATIVE_LIGHTBURN
	case LIGHTBURN_STOP:
#endif //!ALLOW_NATIVE_LIGHTBURN
	case ABORT_CHAR:     //if (rawChar==8)
		//this is a job abort, flush buffer NOW!!!!
#ifdef GB_ABORT_PIN
		pinSetBit(GB_ABORT_PIN); // signal to logic analyzer
#endif
		//SpindleDesiredSpeedPWM = 0;//kill power now stop spindle with motion.
		//CO2LaserAnalogPwrPWM = 0;//kill laser now
		Co2LaserWatchDogTimer = 0;//extra sure no laser is running
		TIM8->CCR3 = 0;//turn off co2 laser just in case
		CO2LaserAnalogPwrPWM = 0;
		_requestToAbortAtEndOfMove = TRUE;
		break;

	case URGENT_911_CMD_CHAR:     //if (rawChar==9)
		_copyToUrgentRxBuffer = TRUE;
		break;

	case CMD_END_CHAR:  //if (rawChar==10)
		_gcodeCmdsReceived++;
		_gs._flasher.cmd_received = FLASHER_ON_COUNT;
		_processingAComment = FALSE;
		_processingASoapString = FALSE;
		break;

	case JOG_Z_TABLE_UP:     //if (rawChar==11)
		//jog Z command (-Z move)
		if (!(_blockAllMotion || (Motors[M_Z].Limit1Sense.State == SENSOR_TRIPPED)))
		{
			JogMotor(&Motors[M_Z], _jogZ.jogValueInUnits * -1.0f); // JogZMotor so the table goes up
		}
		break;

	case JOG_Z_TABLE_DOWN:     //if (rawChar==12)
		//jog Z command (+Z move)
		if (!(_blockAllMotion || (Motors[M_Z].Limit2Sense.State == SENSOR_TRIPPED)))
		{
			JogMotor(&Motors[M_Z], _jogZ.jogValueInUnits * 1.0f); // JogZMotor so the table goes down
		}
		break;
	case REPETREL_COMM_WATCHDOG_CHAR:   // (rawChar==14)
		_repetrelCommWatchCount = REPETREL_COMM_WATCHDOG_START_VALUE;
		break;
	case DIRECT_START_CHAR:  //if (rawChar=='$')
		if (!(_processingAComment || _processingASoapString))
		{   // avoid triggering the direct buffer if someone has a comment with a '$' in it!
			if (_copyToDirectRxBuffer == FALSE)
			{   // not already putting data in the direct buffer, so need to set the flag and then
				// make sure this char does not get in the direct buffer (after the leading char, this
				// same char value could be part of the image stream
				_copyToDirectRxBuffer = TRUE;  // move subsequent characters into the direct buffer
				return; //tossThisChar = TRUE;
			}
		}
		break;
	case COMMENT_CHAR:  //if (rawChar==';')
		if (!_copyToDirectRxBuffer)
		{   // avoid triggering a comment if ';' was part of the direct char
			_processingAComment = TRUE;  // flag that processing ; to CMD_END
		}
		break;
#ifdef USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
	case SOAPSTRING_CHAR:   //if (rawChar=='?')
		if (!_copyToDirectRxBuffer)
		{   // avoid triggering a comment if ';' was part of the direct char
			_processingASoapString = TRUE;  // flag that processing ; to CMD_END
		}
		break;
#endif //USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
	}
#ifdef USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
	if (_processingAComment)
	{
		;   // toss the character
	}
	else
#endif //USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS
		if (( rawChar > LARGEST_SPECIAL_CHAR) || (rawChar == CMD_END_CHAR))
		{   // keep the character and copy to the correct buffer.
			if (_copyToUrgentRxBuffer)
			{
				addCharToUrgentBuffer(rawChar);
			}
			else if (_copyToDirectRxBuffer)
			{
				if (rawChar != DIRECT_START_CHAR)
					addCharToDirectBuffer(rawChar);
			}
			else
			{
				addCharToNormalBuffer(rawChar);
			}
#ifdef COLLECT_METRICS
			if (rawChar == CMD_END_CHAR)
			{
				_metrics.total_commandsProcessed++;
				_metrics.cmdQue_entriesWhenReceived[CommandsInQue]++;
			}
#endif //COLLECT_METRICS
		}
}

////////////////////////////////////////////////////////////////////////////////

void ProcessRawRxBuffer(void)
{
	char rawChar;
	int i;
	int charsToProcess;

	if (rawRxCharsInBuf && rawUsbRxCharsInBuf)
	{	// data coming in via both serial and usb... need to handle both buffers
		// if USB is master, safe to ignore uart traffic (dump it) as the rcv ISRs will auto change the master port
		// else if UART master, then MUST check the USB buffer to see if it's trying to become master, if so, drop usb chars
		// until the ping/abort is detected and change to USB master and ALSO dump all serial chars.

		uint32_t irq_disabled = interruptsOff();
		if (masterCommPort == USB_MASTER)
		{ 	// dump all chars in raw serial buffer
			rawRxCharsInBuf = 0;
			rawRxIndexOut = rawRxIndexIn;
		}
		else // (masterCommPort == UARTx_MASTER)
		{	// check for PING/ABORT in USB buffer
			charsToProcess = rawUsbRxCharsInBuf;
			int charsProcessed = 0;
			for (i=0; i<charsToProcess; i++)
			{
				rawChar = *(USB_Rx_Buffer + rawUsbRxIndexOut);

				if ((rawChar == PING_CHAR) || (rawChar == ABORT_CHAR))
				{	// change master and nuke serial buffer
					changeMasterCommPort(USB_MASTER);
					rawRxCharsInBuf = 0;
					rawRxIndexOut = rawRxIndexIn;
					break;
				}
				else
				{	// drop chars in front of master change
					charsProcessed++;
				}
			}
			// drop any processed usb chars in one shot (rather than incr each var for each char

			rawUsbRxIndexOut += charsProcessed;
			rawUsbRxIndexOut %= SERIAL_RX_RAW_USB_BUFFER_SIZE; //wrap index
			rawUsbRxCharsInBuf -= charsProcessed;
			if (rawUsbRxCharsInBuf == 0)
			{
				releaseUsbBuffer();
			}
		}
		interruptsOn(irq_disabled);
	}

	if (rawUsbRxCharsInBuf)
	{
		charsToProcess = imin(rawUsbRxCharsInBuf, RAW_CHARS_TO_PROCESS_PER_CALL);
#ifdef COLLECT_METRICS
		if (rawUsbRxCharsInBuf > _metrics.max_rawUsbRxCharsInBuf)
			_metrics.max_rawUsbRxCharsInBuf = rawUsbRxCharsInBuf;
		_metrics.total_charsRx+=charsToProcess;
#endif
		for (i=0; i<charsToProcess; i++)  // limit number of chars to process to avoid hogging too much time
		{
			rawChar = USB_Rx_Buffer[rawUsbRxIndexOut++];
			rawUsbRxIndexOut %= SERIAL_RX_RAW_USB_BUFFER_SIZE; //wrap index
			protectedDecrement(&rawUsbRxCharsInBuf);
			if ((rawChar == PING_CHAR) || (rawChar == ABORT_CHAR))
			{
				changeMasterCommPort(USB_MASTER);
			}
			ProcessRawRxChar(rawChar);
		}
		if (rawUsbRxCharsInBuf == 0)
		{
			releaseUsbBuffer();
		}
	}
	else if (rawRxCharsInBuf)
	{
		charsToProcess = imin(rawRxCharsInBuf, RAW_CHARS_TO_PROCESS_PER_CALL);
#ifdef COLLECT_METRICS
		if (rawRxCharsInBuf > _metrics.max_rawRxCharsInBuf)
			_metrics.max_rawRxCharsInBuf = rawRxCharsInBuf;
		_metrics.total_charsRx+=charsToProcess;
#endif
		for (i=0; i<charsToProcess; i++)  // limit number of chars to process to avoid hogging too much time
		{
			rawChar = rawRxBuffer[rawRxIndexOut++];  // copy the char from the raw stream
			rawRxIndexOut %= SERIAL_RX_RAW_BUFFER_SIZE; //wrap index
			protectedDecrement(&rawRxCharsInBuf);
			ProcessRawRxChar(rawChar);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void transmitEchoChar(void)
{   // if we're echo serial rx data back to one one of the serial ports
#ifdef GB_DEBUGGING
	if (echoTxCharsInBuf < 0)
		echoTxCharsInBuf = echoTxCharsInBuf;
#endif
	if (echoTxCharsInBuf)
	{
		boolean sendOK=FALSE;
#ifdef GB_DEBUGGING
	if (echoTxCharsInBuf <= 0)
		echoTxCharsInBuf = echoTxCharsInBuf;
#endif
		switch (_echoProcessedSerialStreamPort | _echoRawSerialStreamPort) {    // only one active
		case SERIAL_PORT_UART3 : if (USART3->SR & USART_FLAG_TXE) { USART3->DR = *(echoTxBuffer + echoTxIndexOut); sendOK = TRUE;} break;
		case SERIAL_PORT_UART4 : if (UART4->SR & USART_FLAG_TXE)  { UART4->DR  = *(echoTxBuffer + echoTxIndexOut); sendOK = TRUE;} break;
		case SERIAL_PORT_UART6 : if (USART6->SR & USART_FLAG_TXE) { USART6->DR = *(echoTxBuffer + echoTxIndexOut); sendOK = TRUE;} break;
		case SERIAL_PORT_USB :  // USB code will auto manage sending data
		default:
			break;
		}
#ifdef GB_DEBUGGING
	if (echoTxCharsInBuf <= 0)
		echoTxCharsInBuf = echoTxCharsInBuf;
#endif
		if (sendOK)
		{
#ifdef GB_DEBUGGING
	if (echoTxCharsInBuf <= 0)
		echoTxCharsInBuf = echoTxCharsInBuf;
#endif
			protectedDecrement(&echoTxCharsInBuf);
			echoTxIndexOut++;
			echoTxIndexOut %= SERIAL_TX_ECHO_BUFFER_SIZE;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void forceCharToHw(char ch)
{
	// this will blindly try to send to all possible comm ports
	if (USART6->SR & USART_FLAG_TXE)
		USART6->DR = ch;
	// USB will occur via a different mechanism
}

////////////////////////////////////////////////////////////////////////////////

boolean transmitCharToHw(char ch)
{
	boolean sendOK = FALSE;
	switch (masterCommPort) {
	case UART3_MASTER :
		if (USART3->SR & USART_FLAG_TXE)
		{
			USART3->DR = ch;
			sendOK = TRUE;
		}
		break;
	case UART4_MASTER :
		if (UART4->SR & USART_FLAG_TXE)
		{
			UART4->DR = ch;
			sendOK = TRUE;
		}
		break;
	case UART6_MASTER :
		if (USART6->SR & USART_FLAG_TXE)
		{
			USART6->DR = ch;
			sendOK = TRUE;
		}
		break;
	case USB_MASTER :   // USB code will auto-manage sending data from the normal  tx buffer
	case BOOTUP:
	default:
		sendOK = FALSE;
		break;
	}
	return(sendOK);
}

////////////////////////////////////////////////////////////////////////////////

void PrintCheck (void)
{   // see if any chars to send to UART HW ....
	// first see if any pending acks and if so AND ok to send, the send an ack
	// else if regular chars to send and OK, then send that char

	if (pendingAcknowledge && roomInNormalRxBuffer() && roomInDirectRxBuffer())
	{   // need to send an ACK AND enough room to receive more input data
#ifdef ALLOW_NATIVE_LIGHTBURN
		if (_lightburnModeEnabled)
			sendstringCr("ok"); // just add to buffer .... no choice with new USB implementation
		else
			sendchar(ASCII_ACK); // just add to buffer .... no choice with new USB implementation
#else //!ALLOW_NATIVE_LIGHTBURN
		sendchar(ASCII_ACK); // just add to buffer .... no choice with new USB implementation
#endif //!ALLOW_NATIVE_LIGHTBURN
		pendingAcknowledge--;  // no need to protect decr... all routines called via sysTick
		_gcodeAcksSent++;   // for GB_STRING reporting
	}
	else if (normalTxCharsInBuf)
	{   // ok if you get here then we have characters in the queue waiting to go out
		// only send to the current comm master port
		if (transmitCharToHw(*(normalTxBuffer + normalTxIndexOut)))
		{
			protectedDecrement(&normalTxCharsInBuf);
			normalTxIndexOut++;
			normalTxIndexOut %= SERIAL_TX_NORMAL_BUFFER_SIZE;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void changeMasterCommPort(masterPort_t newPort)
{
	if (newPort != masterCommPort)
	{   // changing ports
		resetSerialInputBuffer();
		resetSerialOutputBuffer();
		masterPort_t oldPort = masterCommPort;
		masterCommPort = newPort;
		if (oldPort == BOOTUP)
		{
			sendRevisionString(resetSourceStr);
		}
	}
}

void USART3_IRQHandler(void)
{
	// RXNEIE --> RX char avail OR char is avail and shift reg is full (second char avail)
	//         RXNE=1 or ORE=1 (overrun) in _SR reg
	if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE))
	{
		char RcvUART3 = (USART3->DR & (uint16_t)0x1FF);
		if (co2UartRxCharsInBuf < CO2_UART_RX_RETURN_STRING_SIZE)
		{
			co2UartRxBuffer[co2UartRxCharsInBuf] = RcvUART3;
			co2UartRxCharsInBuf++;
		}
		if (co2UartRxCharsInBuf > (CO2_UART_RX_RETURN_STRING_SIZE - 2))
		{
			ValidCo2UartRxWatchdog = 10;
			laser_PsOutputCurrent       = (co2UartRxBuffer[3] << 8) | co2UartRxBuffer[4];
			laser_PsOutputVoltage       = (co2UartRxBuffer[5] << 8) | co2UartRxBuffer[6];
			laser_PsControlVoltage      = (co2UartRxBuffer[7] << 8) | co2UartRxBuffer[8];
			laser_PsWaterProt           = (co2UartRxBuffer[13] == 1);
			co2UartRxCharsInBuf = 0;//reset the pointer
		}
	}
	USART3->SR = (uint16_t)~((uint16_t)0x01 << (uint16_t)(USART_IT_ORE_RX >> 0x08));//clear inerrupt
}

////////////////////////////////////////////////////////////////////////////////

void PCHAR(void)
{
	if (co2UartTxCharsInBuf)
	{
		// ok if you get here then we have characters in the queue waiting to go out
		// only send to the current comm master port
		if (USART3->SR & USART_FLAG_TXE)
		{
			USART3->DR = co2UartTxBuffer[CO2_UART_TX_REQUEST_STRING_SIZE - co2UartTxCharsInBuf];
			co2UartTxCharsInBuf--;
		}
	}
}

void LaserSendRequestStringToPowerSupply(void)
{
	// need to send a specific string to the UART based power supply to for it to report
	// 'U' '5' 0 0 138 \r   'U' '5' 1 0x22 173 \r
	//'U'+'5'+0+0+138+'\r' + 'U'+'5'+1+34+173'\r'; -- NO NULL_CHAR
	// reset controls to force request to be sent to the power supply.  PCHAR will see
	// that there are char in the buffer and start sending
	// contents of the send data are static
	co2UartTxCharsInBuf = CO2_UART_TX_REQUEST_STRING_SIZE;//send the static message, over and over
}