/*
process line by line intel hex format and send data to device as page boundaries are crossed.
when complete, calculate the device checksum for the code pages and update to the device

format:
[0] = ':'  (start code)
[2:1] = byte count (2 hex digits)
[6:3] = address (4 hex digits) - 16-bit address (64K limit)
[8:7] = record type (2 hex digits)
	00 - data record
	01 - end of file : byte count=0; no data; usually address=0000; 
	02 - extended segment address: 2 hex digit pairs in big endian (byte count = 2); address = 0000; lsb of segment = 0 (really specified bits r['19:16] of address
	03 - start segment address -- ignore??
	04 - extended linear addressing -- (specify upper 16 bits of address) 
	05 - start linear address record -- addr=0; byte count = 4; data=start address

[x:9] = data (2 hex digits per byte)
[len-1:len-2] = checksum = least significant byte of 2's comp of the sum of all fields (not as ascii, but the converted bytes) except start code and checksum.     to check for a correct transfer, the sum all fields except start and the resulting lower byte should be 0;
*/

#include <stdio.h>   
#include <string.h>
#include "Main.h"
#include "Hydra_can.h"
#include "Serial.h"
#include "hardwareinit.h"

////////////////////////////////////////////////////////////////////////////////
//
// GENERIC TO SYSTEM AND DEVICE BOOTLOADER
//
////////////////////////////////////////////////////////////////////////////////

void sendHexLineError(char *s)
{
	sprintf(_errorStr, "BL: Line %d: %s (%s)", (int)_gs._bl.lineNum, s, _gs._bl.inputLine);
	sendError(_errorStr);
}

////////////////////////////////////////////////////////////////////////////////

passFail_t intelHexDecomposeLine(char *s)
{
	int i;

	if (strchr(s, CMD_END_CHAR))     // replace first line feed char (if any) with NULL char
		*strchr(s, CMD_END_CHAR) = NULL_CHAR;

	if (strchr(s, CR_CHAR))     // replace first carriage return char (if any) with NULL char
		*strchr(s, CR_CHAR) = NULL_CHAR;
 
	if (strlen(s) < 11) {
		sendHexLineError("premature end of line");
		return(FAIL);
	}
	
	_gs._bl.intelHexLine.start  = *s++;

	if (_gs._bl.intelHexLine.start != ':') {
		sendHexLineError("missing start character...ignoring input line");
		return(PASS);
	}

	_gs._bl.intelHexLine.bytes  = asciihex2bin(*s++) << 4;;
	_gs._bl.intelHexLine.bytes |= asciihex2bin(*s++);
	_gs._bl.intelHexLine.verifyChecksum = _gs._bl.intelHexLine.bytes;

	_gs._bl.intelHexLine.addr  = asciihex2bin(*s++) << 12;;
	_gs._bl.intelHexLine.addr |= asciihex2bin(*s++) << 8;;
	_gs._bl.intelHexLine.addr |= asciihex2bin(*s++) << 4;;
	_gs._bl.intelHexLine.addr |= asciihex2bin(*s++);
	_gs._bl.intelHexLine.verifyChecksum += (_gs._bl.intelHexLine.addr >> 8) & 0xff;
	_gs._bl.intelHexLine.verifyChecksum += _gs._bl.intelHexLine.addr & 0xff;

	_gs._bl.intelHexLine.record  = asciihex2bin(*s++) << 4;;
	_gs._bl.intelHexLine.record |= asciihex2bin(*s++);
	_gs._bl.intelHexLine.verifyChecksum += _gs._bl.intelHexLine.record;

	if (!((_gs._bl.intelHexLine.record==0) || (_gs._bl.intelHexLine.record==1) || (_gs._bl.intelHexLine.record==3) || 
		  (_gs._bl.intelHexLine.record==4) || (_gs._bl.intelHexLine.record==5) )) {
		sprintf(_errorStr, "unsupported record type [%2X]", _gs._bl.intelHexLine.record);
		sendHexLineError(_errorStr);
		return(FAIL);
	}

	if (strlen(s) != (_gs._bl.intelHexLine.bytes * 2 + 2)) {
		sendHexLineError("invalid line - unexpected number of characters");
		return(FAIL);
	}
	
	if (_gs._bl.intelHexLine.bytes > MAX_DATA_BYTES_PER_HEX_LINE) {
		sprintf(_errorStr, "unsupported number of data bytes per line [%2X]", _gs._bl.intelHexLine.bytes);
		sendHexLineError(_errorStr);
		return(FAIL);
	}
	
	for (i=0; i<_gs._bl.intelHexLine.bytes; i++) {
		_gs._bl.intelHexLine.data[i]  = asciihex2bin(*s++) << 4;;
		_gs._bl.intelHexLine.data[i] |= asciihex2bin(*s++);
		_gs._bl.intelHexLine.verifyChecksum += _gs._bl.intelHexLine.data[i];
	}

	_gs._bl.intelHexLine.checksum  = asciihex2bin(*s++) << 4;;
	_gs._bl.intelHexLine.checksum |= asciihex2bin(*s++);
	_gs._bl.intelHexLine.verifyChecksum += _gs._bl.intelHexLine.checksum;
	if ((_gs._bl.intelHexLine.verifyChecksum & 0xff) != 0) {
		sprintf(_errorStr, "intelHexLine checksum not zero [%2X]", _gs._bl.intelHexLine.verifyChecksum);
		sendHexLineError(_errorStr);
		return(FAIL);
	}
	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

void clearWorkingBufferForBootloader(void)
{
	uint32_t i;
	for (i=0; i<WORKING_BUFFER_SIZE_IN_BYTES; i++) {
		_MailBoxes._workingBuffer[i] = 0xff;
	}
	_gs._bl.bytesToSend = 0;
}

////////////////////////////////////////////////////////////////////////////////
//
// DEVICE BOOTLOADER SPECIFIC
//
////////////////////////////////////////////////////////////////////////////////

void clearDeviceCodePages(loopbackType loopback)
{
	// multi-part function to erase all code pages in device in preparation to load code
	// multiple parts to create round trip timing paths to device to ensure we never overrun
	// the input fifo (3 deep) in the device.

	switch (loopback)
	{
		case CLEARING_PAGES_PART1 :
			canFillDeviceBuffer(_gs._bl.device, 0xffffffff);
			sendLoopbackMessageToDevice(_gs._bl.device, loopback+1, 0, 0, 0, 0);        // wait for completion
			break;
		case CLEARING_PAGES_PART2 :
			// setup loop ... index in u16[0], terminal value in u16[1]
			// for (i=_gs._bl.firstCodePage; i<= _gs._bl.lastCodePage; i++)
			sendLoopbackMessageToDevice(_gs._bl.device, loopback+1, _gs._bl.firstCodePage, _gs._bl.lastCodePage, 0, 0);
			break;
		case CLEARING_PAGES_PART3 :
			// one step of loop
			if (_loopbackPayload.u16[0] <= _loopbackPayload.u16[1])
			{
				canCopyDeviceBufferToPage(_gs._bl.device, _loopbackPayload.u16[0], 0);
				// send control for next iteration of the loop
				sendLoopbackMessageToDevice(_gs._bl.device, loopback, _loopbackPayload.u16[0]+1, _loopbackPayload.u16[1], 0, 0);
			}
			else
			{
				sendLoopbackMessageToDevice(_gs._bl.device, loopback+1, 0, 0, 0, 0);        // loop complete, move on to the next step
			}
			break;
		case CLEARING_PAGES_PART4 :
			///done
			break;
		default :
			sprintf(_errorStr,"clearDeviceCodePages - unknown loopbackType %d", loopback);
			sendError(_errorStr);
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////

passFail_t verifyDeviceCodeChecksum()
{
	// one by one read the device checksum
	uint16_t page;
	uint32_t sentChecksum=0;

	for (page=0; page<MAX_BOOTLOADER_PAGES; page++)
	{
		if (_gs._bl.pageSentChecksum[page] != _gs._bl.pageReceivedChecksum[page])
		{
			sprintf(_errorStr, "bootloader send and receive checksums do not match for page %d", page);
			sendError(_errorStr);
			return(FAIL);
		}
	}

	// sum up page by page the sent and received checksums code related pages
	for (page=_gs._bl.firstCodePage; page<=_gs._bl.lastCodePage; page++)
	{
		sentChecksum += _gs._bl.pageSentChecksum[page];
	}
	// compare the page by page sums with the running total that were kept during processing
	if (_gs._bl.totalSentChecksum != sentChecksum)
	{
		sprintf(_errorStr, "bootloader: total send checksum and sum of page checksums do not match (Tchecksum=0x%08x / Pchecksum=0x%08x)",
																							(int)_gs._bl.totalSentChecksum, (int)sentChecksum);
		sendError(_errorStr);
		return(FAIL);
	}

	if (_gs._bl.totalSentChecksum != 0) {
		sprintf(_errorStr, "bootloader: total checksum does not equal 0 (checksum = %d)", (int)_gs._bl.totalSentChecksum);
		sendError(_errorStr);
		return(FAIL);
	}
	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

void setDeviceCodeChecksum(loopbackType type)
{
	// routine to create and set the checksum field in the device.  this is a multi-step routine that will occur
	// over time since the bootloader in the device has a minimal fifo and no flow control.  with this approach,
	// a synchronizing "loopback" (can packet is parrotted back) to know when the prior real command is complete
	// such that we can move on to the next step

	payloadUnion payload;

	switch (type)
	{
		case DEVICE_CODE_CHECKSUM_PART1 :
			canFillDeviceBuffer(_gs._bl.device, 0x00000000);                        // clear device's working buffer
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, 0, 0);        // wait for completion
			break;
		case DEVICE_CODE_CHECKSUM_PART2 :
			canCopyPageToDeviceBuffer(_gs._bl.device, _gs._bl.lastCodePage);        // move last code page into working buffer
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, 0, 0);        // wait for completion
			break;
		case DEVICE_CODE_CHECKSUM_PART3 :
			payload.u32[0] = CHECKSUM_KEY;                                          // build 8-byte checksum field (first 8-bytes non-0, non-ff const
			_gs._bl.totalSentChecksum += payload.u32[0];
			_gs._bl.pageSentChecksum[_gs._bl.lastCodePage] += payload.u32[0];
			payload.u32[1] = (~_gs._bl.totalSentChecksum) + 1;                      // 2's comp of actual checksum to produce final checksum that equal 0x00000000
			_gs._bl.totalSentChecksum += payload.u32[1];
			_gs._bl.pageSentChecksum[_gs._bl.lastCodePage] += payload.u32[1];
			// write out checksum packet
			canWriteDeviceBuffer(_gs._bl.device, (_MailBoxes._inbox[getMailboxNum(_gs._bl.device)].flashPageSize / 8) - 1, &payload);
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, 0, 0);        // wait for completion
			break;
		case DEVICE_CODE_CHECKSUM_PART4 :
			// copy device working buffer to page
			canCopyDeviceBufferToPage(_gs._bl.device, _gs._bl.lastCodePage, _gs._bl.pageSentChecksum[_gs._bl.lastCodePage]);
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, 0, 0);        // wait for completion
			break;
		case DEVICE_CODE_CHECKSUM_PART5 :
			_MailBoxes._pageChecksum = 0;
			readPageChecksumFromDevice(_gs._bl.device, _gs._bl.lastCodePage);       // read back checksum from newly written page
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, 0, 0);        // wait for completion
			break;
		case DEVICE_CODE_CHECKSUM_PART6 :
			// done!
			sendstringCr(">BL: Code download complete.  Updating checksum");
			if (verifyDeviceCodeChecksum() == FAIL) {
				sendError("Code Checksum Verification FAILED");
			}
			else {
				sendstringCr(">BL: Program Download SUCCESSFUL");
			}
			endDeviceBootloader();
			sendSetSwResetBit(_gs._bl.device);  // reset part without _bl.started will cause part to recheck code checksum and jump to real program
			break;
		default :
			sprintf(_errorStr,"setDeviceCodeChecksum - unknown loopbackType %d", type);
			sendError(_errorStr);
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void transferBootloaderDataFromWorkingBufferToDevice(loopbackType type, boolean reprocessLine, byte bytesSentLastTime)
{
	// routine to transfer the working buffer to a page in the device.  this is a multi-step routine that will occur
	// over time since the bootloader in the device has a minimal fifo and no flow control.  with this approach,
	// a synchronizing "loopback" (can packet is parrotted back) to know when the prior real command is complete
	// such that we can move on to the next step
	uint32_t workingBufferChecksum;

	switch (type)
	{
		case TRANSFER_PAGE_PART1 :
			canFillDeviceBuffer(_gs._bl.device, 0x00000000);
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, reprocessLine, bytesSentLastTime);        // wait for completion
			break;
		case TRANSFER_PAGE_PART2 :
			// setup loop ... index in u16[0], terminal value in u16[1]
			// for (i=0; i<_mb._inbox[getMailboxNum(_gs._bl.device)].flashPageSize/8; i++)
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, _MailBoxes._inbox[getMailboxNum(_gs._bl.device)].flashPageSize/8, reprocessLine, bytesSentLastTime);
			break;
		case TRANSFER_PAGE_PART3 :
			// one step of loop
			if (_loopbackPayload.u16[0] < _loopbackPayload.u16[1])
			{
				canWriteDeviceBuffer(_gs._bl.device, _loopbackPayload.u16[0], (payloadUnion *)&_MailBoxes._workingBuffer[_loopbackPayload.u16[0]*8]);
				// send control for next iteration of the loop
				sendLoopbackMessageToDevice(_gs._bl.device, type, _loopbackPayload.u16[0]+1, _loopbackPayload.u16[1], reprocessLine, bytesSentLastTime);
			}
			else
			{
				sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, reprocessLine, bytesSentLastTime);        // loop complete, move on to the next step
			}
			break;
		case TRANSFER_PAGE_PART4 :
			workingBufferChecksum = getWorkingBufferChecksum(_gs._bl.device);
			canCopyDeviceBufferToPage(_gs._bl.device, _gs._bl.page, workingBufferChecksum);
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, reprocessLine, bytesSentLastTime);            // wait for completion
			break;
		case TRANSFER_PAGE_PART5 :
			_MailBoxes._pageChecksum = 0;
			readPageChecksumFromDevice(_gs._bl.device, _gs._bl.page);                               // read back checksum from newly written page
			sendLoopbackMessageToDevice(_gs._bl.device, type+1, 0, 0, reprocessLine, bytesSentLastTime);            // wait for completion
			break;
		case TRANSFER_PAGE_PART6 :
			///done
			clearWorkingBufferForBootloader();  // clear 407 working buffer to be ready for new data
			if (reprocessLine)
			{
				processDeviceBootloaderLine(reprocessLine, bytesSentLastTime);
			}
			break;
		default :
			sprintf(_errorStr,"transferBootloaderDataFromWorkingBufferToDevice - unknown loopbackType %d", type);
			sendError(_errorStr);
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////

passFail_t sendDeviceBootloaderPage(boolean reprocessLine, byte bytesSentLastTime)
{
	if (_gs._bl.page >= MAX_BOOTLOADER_PAGES) {
		sendError("bootloader - page number too large");
		return(FAIL);
	}
	_gs._bl.pageSentChecksum[_gs._bl.page] = getWorkingBufferChecksum(_gs._bl.device);
	if ((_gs._bl.page >= _gs._bl.firstCodePage) && (_gs._bl.page <= _gs._bl.lastCodePage))
	{   // keep running total of code checksum as we send it out
		_gs._bl.totalSentChecksum += _gs._bl.pageSentChecksum[_gs._bl.page];
	}
	transferBootloaderDataFromWorkingBufferToDevice(TRANSFER_PAGE_PART1, reprocessLine, bytesSentLastTime);
	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

void prepareDeviceForBootloaderDownload()
{
	uint16_t page;
	_gs._extendedSliceTimeNeeded = 1000;
	_gs._bl.lineNum = 0;
	_gs._bl.fullAddress = 0;
	_gs._bl.baseAddress = 0;
	_gs._bl.pageAddress = 0;
	_gs._bl.page = 0;
	_gs._bl.bytesToSend = 0;
	_gs._bl.firstCodePage = 4;
	_gs._bl.lastCodePage = (_MailBoxes._inbox[getMailboxNum(_gs._bl.device)].flashNumKBytes * 1024) / _MailBoxes._inbox[getMailboxNum(_gs._bl.device)].flashPageSize - 4 - 1;
	_gs._bl.started = TRUE;
	_gs._bl.prepped = TRUE;
	_gs._bl.endOfFileReached = FALSE;
	_gs._bl.totalSentChecksum = 0;
	for (page=0; page<MAX_BOOTLOADER_PAGES; page++)
	{
		_gs._bl.pageSentChecksum[page] = 0;
		_gs._bl.pageReceivedChecksum[page] = 0;
	}
	clearWorkingBufferForBootloader();  // clear the 407 buffer to 0's
	clearDeviceCodePages(CLEARING_PAGES_PART1);  // need to erase all code pages in order to build proper checksum
}

////////////////////////////////////////////////////////////////////////////////

void endDeviceBootloader(void)
{
	_MailBoxes._inbox[getMailboxNum(_gs._bl.device)].bootloaderRunning = FALSE;
	_gs._bl.started = FALSE;
	_gs._bl.prepped = FALSE;
	_gs._bl.password = 0;
	_gs._bl.MCODE748_error_reported = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

byte processDeviceBootloaderLine(boolean reprocess, byte bytesSentLastTime)
{
	uint32_t i;
	byte returnCode = PASS;
	uint16_t flashPageSize;
	uint32_t flashBaseAddr;

//  if (_gs._bl.started == FALSE) {
//      sendHexLineError("Hex line received without active bootloader");
//      return(FAIL);
//  }
//  if (_gs._bl.started == FALSE) {
//      sendHexLineError("Hex line received without preparing device");
//      return(FAIL);
//  }
	if (_gs._bl.endOfFileReached == TRUE) {
		sendHexLineError("Hex line received after end of file reached");
		return(FAIL);
	}

	if (reprocess == FALSE)
	{
		if (intelHexDecomposeLine(_gs._bl.inputLine) == FAIL) {
			return(FAIL);
		}
	}
	else
	{   // reprocess same line
		if (bytesSentLastTime > 0)
		{   // already sent some of the bytes, so need to adjust things so that they do not get resent
			_gs._bl.intelHexLine.addr += bytesSentLastTime;     // increase by number of bytes sent
			_gs._bl.intelHexLine.bytes -= bytesSentLastTime;        // decrease by the number of bytes sent

			for (i=0; i <_gs._bl.intelHexLine.bytes-bytesSentLastTime; i++)
			{
				_gs._bl.intelHexLine.data[i] = _gs._bl.intelHexLine.data[bytesSentLastTime + i];
			}
		}
	}

	flashPageSize = _MailBoxes._inbox[getMailboxNum(_gs._bl.device)].flashPageSize;
	flashBaseAddr = _MailBoxes._inbox[getMailboxNum(_gs._bl.device)].flashBaseAddr;

	switch (_gs._bl.intelHexLine.record)
	{
		case DATA_RECORD    :
			_gs._bl.fullAddress = _gs._bl.baseAddress + _gs._bl.intelHexLine.addr;
			if (_gs._bl.page != ((_gs._bl.fullAddress - flashBaseAddr) / flashPageSize))
			{  // change to a new page
				if (_gs._bl.bytesToSend > 0)
				{
					// change of page, so send out old page before starting new one
					returnCode |= sendDeviceBootloaderPage(TRUE, 0);  // no code can occur after sendDeviceBootloaderPage (multi-step code)
					return(returnCode); // will reenter later after page has been sent (and bytesToSend will be 0 at that time, so we'll
								  // jump down to calculate the new page number.
				}
			}
			_gs._bl.page = (_gs._bl.fullAddress - flashBaseAddr) / flashPageSize;
			_gs._bl.pageAddress = (_gs._bl.fullAddress - flashBaseAddr) % flashPageSize;
			for (i=0; i<_gs._bl.intelHexLine.bytes; i++)
			{
				if (_gs._bl.pageAddress == flashPageSize)
				{   // page change within a single hex line....so send out the page and then reenter this routine,
					// but purge the bytes already sent
					returnCode |= sendDeviceBootloaderPage(TRUE, i);
					return(returnCode);
				}
				_MailBoxes._workingBuffer[_gs._bl.pageAddress++] = _gs._bl.intelHexLine.data[i];
				_gs._bl.bytesToSend++;
			}
			break;
		case END_FILE :
			if ((_gs._bl.intelHexLine.bytes == 0) && (_gs._bl.intelHexLine.addr == 0))
			{
				// hit end of file, so dump out any residual data waiting to be written;
				_gs._bl.endOfFileReached = TRUE;
				if (_gs._bl.bytesToSend > 0)
				{
					returnCode |= sendDeviceBootloaderPage(FALSE, 0);  // no code can occur after sendDeviceBootloaderPage (multi-step code)
				}
			}
			else {
				sendHexLineError("invalid END_FILE");
				returnCode |= FAIL;
				break;
			}
			break;
		case EXTENTED_SEGMENT :
			if ((_gs._bl.intelHexLine.bytes == 1) && (_gs._bl.intelHexLine.addr == 0))
			{
				_gs._bl.baseAddress = (_gs._bl.intelHexLine.data[0] << 4) << 16;
			}
			else {
				sendHexLineError("invalid EXTENDED_gs._bl.ADDR");
				returnCode |= FAIL;
				break;
			}
			break;
		case START_SEGMENT :
			// ignore START_SEGMENT
			returnCode |= FAIL;
			break;
		case EXTENDED_ADDR :
			if ((_gs._bl.intelHexLine.bytes == 2) && (_gs._bl.intelHexLine.addr == 0))
			{
				_gs._bl.baseAddress = ((_gs._bl.intelHexLine.data[0] << 8) + _gs._bl.intelHexLine.data[1]) << 16;
			}
			else
			{
				sendHexLineError("invalid EXTENDED_ADDR");
				returnCode |= FAIL;
				break;
			}
			break;
		case START_ADDR :
			// ignore START_ADDR
			break;
		default :
			sendHexLineError("invalid EXTENDED_ADDR");
			sprintf(_errorStr, "unknown record type [%2X]", _gs._bl.intelHexLine.record);
			sendHexLineError(_errorStr);
			returnCode |= FAIL;
			break;
	}
	return(returnCode);
}


////////////////////////////////////////////////////////////////////////////////

