#ifndef bootloader_HEADER // prevent double dipping
#define bootloader_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    bootloader.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains bootloader specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  bootloader specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

#define BOOLTOADER_LINE_SIZE            256
#define MAX_DATA_BYTES_PER_HEX_LINE     128
#define MAX_BOOTLOADER_PAGES            64

#define USER_OPTION_BYTE                1       // special option byte that we can only use upper 5 bits (for deviceRev

typedef enum {
	DATA_RECORD         = 0,
	END_FILE            = 1,
	EXTENTED_SEGMENT    = 2,
	START_SEGMENT       = 3,
	EXTENDED_ADDR       = 4,
	START_ADDR          = 5
} intelHexRecordType;

typedef struct {
	char                start;
	byte                bytes;
	uint32_t            addr;
	intelHexRecordType  record;
	byte                data[MAX_DATA_BYTES_PER_HEX_LINE];
	byte                checksum;
	byte                verifyChecksum;
} intelHexLineStruct;

typedef struct {
	byte                device;
	uint32_t            lineNum;
	uint32_t            password;
	boolean             MCODE748_error_reported;
	boolean             MCODE768_error_reported;
	char                inputLine[BOOLTOADER_LINE_SIZE];
	intelHexLineStruct  intelHexLine;
	uint32_t            fullAddress;
	uint32_t            baseAddress;
	uint32_t            pageAddress;
	uint32_t            page;
	uint32_t            firstCodePage;
	uint32_t            lastCodePage;
	uint32_t            bytesToSend;
	boolean             started;
	boolean             prepped;
	boolean             endOfFileReached;
	uint32_t            totalSentChecksum;
	uint32_t            pageSentChecksum[MAX_BOOTLOADER_PAGES];
	uint32_t            pageReceivedChecksum[MAX_BOOTLOADER_PAGES];
} deviceBootloaderStruct;

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in bootloader that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in bootloader
////////////////////////////////////////////////////////////////////////////////

extern void sendHexLineError(char *);
extern passFail_t intelHexDecomposeLine(char *);
extern void clearWorkingBufferForBootloader(void);

extern void transferBootloaderDataFromWorkingBufferToDevice(loopbackType, boolean, byte);
extern passFail_t sendDeviceBootloaderPage(boolean, byte);
extern void setDeviceCodeChecksum(loopbackType);
extern passFail_t verifyDeviceCodeChecksum(void);
extern void clearDeviceCodePages(loopbackType);
extern void prepareDeviceForBootloaderDownload(void);
extern void endDeviceBootloader(void);
extern byte processDeviceBootloaderLine(boolean, byte);

//extern boolean sendSystemBootloaderPage(void);
extern void setSystemCodeChecksum(void);
//extern boolean verifySytesmCodeChecksum(void);
extern void clearSystemCodePages(void);
extern void prepareSystemForBootloaderDownload(void);
extern void endSystemBootloader(void);
extern byte processSystemBootloaderLine(boolean *, byte *);

#endif // #ifndef bootloader_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
