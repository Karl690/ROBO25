////////////////////////////////////////////////////////////////////////////////
//
// File:    hyrel_can.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: common can related function used byte both hot head and main board
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

// There is a great explanation of the canbus located @    http://en.wikipedia.org/wiki/CAN_bus

#ifdef COMPILE_FOR_DEVICE   // head of some sort
#include "main.h"
#endif

#ifdef COMPILE_FOR_SYSTEM   // motion controller
#include "main.h"
#include "serial.h"
#include "mailbox.h"
#endif

////////////////////////////////////////////////////////////////////////////////

uint32_t calculateChecksum8(uint8_t data[], uint32_t numWords)
{
	// common routine needed by 407 and HH to validate can block transfers
	// drop value of 0xffffffff from checksum such that different page sizes and memory
	// sizes can use the same routines without altering the checksum

	uint32_t checksum=0;
	uint32_t i;

	for (i=0; i<numWords; i++)
	{
		if (data[i] != 0xff)
		{
			checksum += data[i];
		}
	}
	return(checksum);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t calculateChecksum16(uint16_t data[], uint32_t numWords)
{
	// common routine needed by 407 and HH to validate can block transfers
	// drop value of 0xffffffff from checksum such that different page sizes and memory
	// sizes can use the same routines without altering the checksum

	uint32_t checksum=0;
	uint32_t i;

	for (i=0; i<numWords; i++)
	{
		if (data[i] != 0xffff)
		{
			checksum += data[i];
		}
	}
	return(checksum);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t calculateChecksum32(uint32_t data[], uint32_t numWords)
{
	// common routine needed by 407 and HH to validate can block transfers
	// drop value of 0xffffffff from checksum such that different page sizes and memory
	// sizes can use the same routines without altering the checksum

	uint32_t checksum=0;
	uint32_t i;

	for (i=0; i<numWords; i++)
	{
		if (data[i] != 0xffffffff)
		{
			checksum += data[i];
		}
	}
	return(checksum);
}

////////////////////////////////////////////////////////////////////////////////
#ifdef COMPILE_FOR_DEVICE

void canInitFilter(CAN_TypeDef *CANx, byte filterNum, byte alias, boolean immediate, boolean enable)
{
#define FMR_FINIT    ((uint32_t)0x00000001) // Filter init mode

	uint32_t filterBit;

	filterBit   = (1 << filterNum);

	CANx->FMR |= FMR_FINIT;                 // Initialization mode for the filter
	CANx->FA1R &= ~filterBit;               // Filter Deactivation

	CANx->FS1R |= filterBit;                // 32-bit scale for the filter
	CANx->FM1R &= ~filterBit;               // Id+Mask mode for the filter

	if (immediate)
	{
		CANx->FFA1R |= filterBit;           // assign FIFO 1 for immediate mode the filters
		CANx->sFilterRegister[filterNum].FR1 = FILTER_CONSTANT_VALUE | FILTER_IMMEDIATE_BIT;                    // compare value
		CANx->sFilterRegister[filterNum].FR2 = FILTER_MASK_CONSTANT_HEAD;                                                // compare mask
	}
	else
	{
		CANx->FFA1R &= ~filterBit;          // assign FIFO 0 for non-immediate mode (buffered) filters
		CANx->sFilterRegister[filterNum].FR1 = FILTER_CONSTANT_VALUE | (alias << FILTER_DEVICE_LSB_POSITION);   // compare value
		CANx->sFilterRegister[filterNum].FR2 = (FILTER_MASK_CONSTANT_HEAD | FILTER_MASK_DEVICE);                     // compare mask
	}

	if (enable == TRUE)
	{
		CANx->FA1R |= filterBit;            // Filter Activation
	}

	CANx->FMR &= ~FMR_FINIT;
}

////////////////////////////////////////////////////////////////////////////////

void canAddUserDefinedAlias(byte newAlias, byte aliases[])
{
	byte i;

	// first, find an open slot within the range specified
	for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
	{
		if (aliases[i] == ALIAS_UNUSED)
		{
			// filters are ordered predef(4), the user(8)
			canInitFilter(CAN1, NUM_PRE_DEFINED_ALIASES+i, newAlias, FALSE, TRUE);
			aliases[i] = newAlias;
			break;
		}
	}
	if (i == NUM_USER_DEFINED_ALIASES)
	{
#if defined (COMPILE_FOR_DEVICE)
		reportError1x32(STICKY_TOO_MANY_ALIASES, ERROR_UNIT_CAN, ERROR_TOO_MANY_ALIASES, (uint32_t)newAlias);
#elif defined (COMPILE_FOR_SYSTEM)
		sprintf(_errorStr, "ERROR_TOO_MANY_ALIASES: newAlias=%d", newAlias);
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////

void canRemoveUserDefinedAlias(byte oldAlias, byte aliases[])
{
	byte i;

	// find matching alias and disable it
	for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
	{
		if (aliases[i] == oldAlias)
		{
			// filters are ordered predef(4), the user(8)
			canInitFilter(CAN1, NUM_PRE_DEFINED_ALIASES+i, ALIAS_UNUSED, FALSE, FALSE);
			aliases[i] = ALIAS_UNUSED;
			break;
		}
	}
}

#endif //COMPILE_FOR_DEVICE

////////////////////////////////////////////////////////////////////////////////

boolean canIsValidAlias(byte device)
{
	byte i;

	for (i=0; i<NUM_PRE_DEFINED_ALIASES; i++)
	{
		if (_gs._preDefinedAliases[i] == device)    // match!!! so it is for us
		{
			return(TRUE);
		}
	}

	for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
	{
		if (_gs._userDefinedAliases[i] == device)   // match!!! so it is for us
		{
			return(TRUE);
		}
	}
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

byte canMsgIdToNumBytes(byte msgId, canbusFormat_t canFmt)
{
	// function to convert the msgId into the expected number of bytes
	switch (msgId)
	{
		case CAN_MSG_PRIME_RUN:
		case CAN_MSG_UNPRIME:
		case CAN_MSG_PRIME:
		case CAN_MSG_RUN    :
		case CAN_MSG_STOP   :
		case CAN_MSG_STEP_MOTOR :
		case CAN_MSG_COPY_PAGE_TO_BUFFER :
		case CAN_MSG_SET_POWER_LEVEL_ONLY :
		case CAN_MSG_START_PRIMARY_PROGRAM :
			return(CAN_BYTES_0);
		//-----------------------------------------------------------------------------------------------------
		case CAN_MSG_MOTOR_ENABLE : //LEGACY
		case CAN_MSG_DEVICE_POSITION :
		case CAN_MSG_ADD_ALIAS :
		case CAN_MSG_REMOVE_ALIAS :
			return(CAN_BYTES_1);
		//-----------------------------------------------------------------------------------------------------
		case CAN_MSG_FLOW_SCALE_FACTORS :
			return(CAN_BYTES_2);
		//-----------------------------------------------------------------------------------------------------
		//-----------------------------------------------------------------------------------------------------
		case CAN_MSG_PRE_DEFINED_ALIASES :
		case CAN_MSG_FILL_BUFFER :
		case CAN_MSG_COPY_BUFFER_TO_PAGE :
		case CAN_MSG_PAGE_CHECKSUM :
		case CAN_MSG_EXTRUSION_TEMP_RANGES :
			return(CAN_BYTES_4);
		//-----------------------------------------------------------------------------------------------------
		//-----------------------------------------------------------------------------------------------------
		case CAN_MSG_AUTO_STATUS_CONTROL :
		case CAN_MSG_UNIQUE_ID :
			return(CAN_BYTES_6);
		//-----------------------------------------------------------------------------------------------------
		//-----------------------------------------------------------------------------------------------------
#ifdef HYDRA_DIAGS
	case CAN_MSG_LED_OVERRIDE :     return(CAN_BYTES_2);
	case CAN_MSG_DIAG_IO_INIT :     return(CAN_BYTES_8); //     ((byte)0x50)    // 8 bytes; page 0
	case CAN_MSG_DIAG_IO_WRITE :    return(CAN_BYTES_1); //     ((byte)0x51)    // 1 byte;  page 0xff
	case CAN_MSG_DIAG_IO_READ :     return(CAN_BYTES_1); //     ((byte)0x52)    // 1 byte;  page 0xff
	case CAN_MSG_DIAG_ADC_READ :    return(CAN_BYTES_2); //     ((byte)0x53)    // 2 bytes; page 0xff
	case CAN_MSG_DIAG_STEP_COUNTER :return(CAN_BYTES_8); //     ((byte)0x54)    // 8 bytes; page 0
#endif
		//-----------------------------------------------------------------------------------------------------
		default :
			return(CAN_BYTES_8);
	}
}

////////////////////////////////////////////////////////////////////////////////

byte canCheckCanStruct(canStruct *canStructPtr)
{
	// routine to check that the number of byte is within valid range
	// (checking page range dropped for V1)
	byte msgIdNumBytes;

#if defined (COMPILE_DEVICE_FOR_CANBUS_FORMAT_V1)
	canbusFormat_t canFmt = CANBUS_FORMAT_V1;
#elif defined (COMPILE_DEVICE_FOR_CANBUS_FORMAT_V2)
	canbusFormat_t canFmt = CANBUS_FORMAT_V2;
#elif defined (COMPILE_FOR_SYSTEM)
	canbusFormat_t canFmt = getInboxPointer(canStructPtr->sw.device)->canbusFormat;
#endif

	msgIdNumBytes = canMsgIdToNumBytes(canStructPtr->sw.msgId, canFmt);
	if (msgIdNumBytes != canStructPtr->sw.numBytes)
	{

#if defined (COMPILE_FOR_DEVICE)
		reportError4x8(STICKY_NUM_BYTES_MISMATCH, ERROR_UNIT_CAN, ERROR_NUM_BYTES_MISMATCH,
				canStructPtr->sw.msgType, canStructPtr->sw.msgId, msgIdNumBytes, canStructPtr->sw.numBytes);
#elif defined (COMPILE_FOR_SYSTEM)
		if (_errors.sent.flags.canNumBytesMismatch == FALSE)
		{
			_errors.sent.flags.canNumBytesMismatch = TRUE;
			sprintf(_errorStr, "ERROR_NUM_BYTES_MISMATCH: msgType=%d msgId=0x%02x expected=%d received=%d",
					canStructPtr->sw.msgType, canStructPtr->sw.msgId, msgIdNumBytes, canStructPtr->sw.numBytes);
			sendError(_errorStr);
		}
#endif
		return(ERROR_NUM_BYTES_MISMATCH);
	}
	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

boolean canTransmitOpenMailbox(CAN_TypeDef *CANx)
{
	if (((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0) ||
		((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1) ||
		((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2))
	{
		return(TRUE);
	}
	else
	{
		return(FALSE);
	}
}

////////////////////////////////////////////////////////////////////////////////


byte canTransmit(CAN_TypeDef *CANx, canHwStruct *canHwStructPtr)
{
	CAN_TxMailBox_TypeDef *mailboxPtr;
	// low level routine to convert the canStruct to hardware registers and tell hardware to begin transfer
	uint8_t transmitMailbox;

	/* Select one empty transmit mailbox */
	if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
	{
		transmitMailbox = 0;
	}
	else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
	{
		transmitMailbox = 1;
	}
	else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
	{
		transmitMailbox = 2;
	}
	else
	{
		transmitMailbox = CAN_TxStatus_NoMailBox;
	}

	if (transmitMailbox != CAN_TxStatus_NoMailBox)
	{
		mailboxPtr = &CANx->sTxMailBox[transmitMailbox];

#if defined (COMPILE_FOR_SYSTEM)
		if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
		{
			if (CANx == CAN1)
			{
				float currTime = (float)_gs._sliceCnt / (float)(SYSTICKS_PER_SECOND);
				canSwStruct *canTx = (canSwStruct *)canHwStructPtr;
				sprintf(_rptStr, "%9.4f - CTX: %2d %02x %2x %d ", currTime, canTx->device, canTx->msgId, canTx->page, canTx->numBytes);
				if (canTx->RTR == 0)
				{
					int i;
					for (i=0; i<canTx->numBytes; i++)
					{
						sprintf(_tmpStr, "%02x.", canTx->payload.u8[i]);
						strcat(_rptStr, _tmpStr);
					}
				}
				else
				{
					sprintf(_tmpStr, "(RTR)");
					strcat(_rptStr, _tmpStr);
				}
				sendGB(_rptStr);
			}
		}
#endif //COMPILE_FOR_SYSTEM

		mailboxPtr->TIR     = canHwStructPtr->IR;
		mailboxPtr->TDTR    = canHwStructPtr->DTR;
		mailboxPtr->TDLR    = canHwStructPtr->DLR;
		mailboxPtr->TDHR    = canHwStructPtr->DHR;

		mailboxPtr->TIR    |= (uint32_t)0x00000001; // TMIDxR_TXRQ -- tell CAN that HW packet is ready to transmit

		_gs._led.canTxLedCnt = LED_COUNTER_START_VALUE_FOR_ONE_SHOT_FLASH; //signal status that we are sending can transmissions;
#ifdef COLLECT_METRICS
		_metrics.total_canTx++;
		if (_gs._canTxQ.numMsg > _metrics.max_canTxQ_numMsg)
			_metrics.max_canTxQ_numMsg = _gs._canTxQ.numMsg;
#endif
	}
	return(transmitMailbox);
}

////////////////////////////////////////////////////////////////////////////////

byte canTransmitSwPacket(canSwStruct *canSwStructPtr)
{       // 3 cases
	//      1: to a physical device that registered from can1
	//  2: to a physical device that registered from can2
	//  3: sending to a non-physical device (broadcast, groups, etc... so no idea if can1 or can2 or both

	byte retValue = CAN_TxStatus_NoMailBox;
#if defined (COMPILE_FOR_DEVICE)
	retValue = canTransmit(CAN1, (canHwStruct *)canSwStructPtr);
#else
	if (isAPhysicalDevice(canSwStructPtr->device))
	{       // physical device
		if (getInboxPointer(canSwStructPtr->device)->fromCAN2 == FALSE)
		{
			retValue = canTransmit(CAN1, (canHwStruct *)canSwStructPtr);    // try to send
		}
#ifdef USE_CAN2
		else if (getInboxPointer(canSwStructPtr->device)->fromCAN2)
		{
				retValue = canTransmit(CAN2, (canHwStruct *)canSwStructPtr);    // try to send
		}
#endif //USE_CAN2
	}
	else
	{       // logical address (broadcast, etc) -- MUST send to both CAN's.... therefore need an open mailbox in both
#ifdef USE_CAN2
			if (canTransmitOpenMailbox(CAN1) && canTransmitOpenMailbox(CAN2))
			{       // both have an open mailbox, so transmit will work
				retValue = canTransmit(CAN1, (canHwStruct *)canSwStructPtr);
				retValue = canTransmit(CAN2, (canHwStruct *)canSwStructPtr);
			}
#else
			if (canTransmitOpenMailbox(CAN1))
			{       // both have an open mailbox, so transmit will work
				retValue = canTransmit(CAN1, (canHwStruct *)canSwStructPtr);
			}
#endif //USE_CAN2
	}
#endif //COMPILE_FOR_DEVICE
	return(retValue);
}

////////////////////////////////////////////////////////////////////////////////

void updateTxQueueIndices(void)
{
	_gs._canTxQ.numMsg--;
	_gs._canTxQ.nextOut++;
	if (_gs._canTxQ.nextOut == CAN_TX_QUEUE_SIZE)
	{
		_gs._canTxQ.nextOut = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkForCanTransmitErrors(CAN_TypeDef *CANx, canTransmitMailbox_t canTransmitMailbox)
{
	// this will check for errors over time for each transmit mailbox...if multiple successive errors,
	// then abort that packet....which currently will be lost forever.

	int canHwIndex = (CANx == CAN1) ? 0 : 1;

	if (CANx->TSR & (CAN_TSR_ABRQ0 << (8*canTransmitMailbox)))
	{   // abort already in progress
		;
	}
	else if (CANx->TSR & (CAN_TSR_TERR0 << (8*canTransmitMailbox)))
	{   // error detected
		_gs._canTransmitMailboxErrorCount[canHwIndex][canTransmitMailbox]++;
		if (_gs._canTransmitMailboxErrorCount[canHwIndex][canTransmitMailbox] >= MAX_TSR_TERR_ALLOWED)
		{
			CANx->TSR |= (CAN_TSR_ABRQ0 << (8*canTransmitMailbox));     // abort the packet that has not been sent
			_gs._canTransmitMailboxErrorCount[canHwIndex][canTransmitMailbox] = 0; // reset mailbox successive tx errors
			_gs._canTransmitAbortedPackets[canHwIndex][canTransmitMailbox]++;
		}
	}
	else
	{
		_gs._canTransmitMailboxErrorCount[canHwIndex][canTransmitMailbox] = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

byte canProcessTxQueue()
{
	// function to take packet from global TX queue and put in in the CAN HW
	byte retValue;
	byte i, j;
	byte canNumTxTransfersPerSlice = CAN_NUM_TX_TRANSFERS_PER_SLICE;

#ifdef USE_CAN2
		if ((CAN1->TSR | CAN2->TSR) & (CAN_TSR_TERR2 | CAN_TSR_TERR1 | CAN_TSR_TERR0))
		{   // at least one transmit error to check out
			for (i=0; i<NUM_CAN_HW_UNITS; i++)
			{
				for (j=0; j<NUM_CAN_TRANSMIT_MAILBOXES; j++)
				{   // check one mailbox at a time
					checkForCanTransmitErrors((i==0) ? CAN1 : CAN2, j);
				}
			}
		}
#else
		if (CAN1->TSR & (CAN_TSR_TERR2 | CAN_TSR_TERR1 | CAN_TSR_TERR0))
		{   // at least one transmit error to check out
			for (j=0; j<NUM_CAN_TRANSMIT_MAILBOXES; j++)
			{   // check one mailbox at a time
				checkForCanTransmitErrors(CAN1, j);
			}
		}
#endif //USE_CAN2

#ifdef COMPILE_FOR_SYSTEM
	if (_gs._bl.started == TRUE)
	{
		canNumTxTransfersPerSlice = 1;  // slow down bootloader comm as bootloader does not have a large input fifo
	}
#endif

	for (i=0; i<canNumTxTransfersPerSlice; i++)
	{
#ifdef COMPILE_FOR_DEVICE
		if ((!CAN_INITIALIZED) || (_gs._canTxQ.numMsg == 0))    // not ready to send message OR nothing to send
			return(CAN_TX_OK);
#elif defined (COMPILE_FOR_SYSTEM)
		if (_gs._canTxQ.numMsg == 0)    // nothing to send
		{
			return(CAN_TX_OK);
		}
#endif


		retValue = canTransmitSwPacket(&_gs._canTxQ.Q[_gs._canTxQ.nextOut].sw);

		if (retValue == CAN_TxStatus_NoMailBox)     // no room in the HW, so exit without changing queue indicies
		{
			return(WARNING_TX_MAILBOX_FULL);
		}
		updateTxQueueIndices();  // otherwise, message was sent, so update TX queue indicies

	}
	return(CAN_TX_OK);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef COMPILE_FOR_DEVICE
canStruct *canGetTxQueueNextInPtr()
{
	// function to return next open spot in the queue.  if there's no slot, wait at least 1ms to

	if (_gs._canTxQ.numMsg >= CAN_TX_QUEUE_SIZE)
	{
		// PROBLEM .... if no room in queue, the the errorReport will not make it into the queue!!!!
		updateTxQueueIndices();             //   so....open up one space in the queue (overwriting oldest
#if defined(COMPILE_FOR_DEVICE)
		reportError1x32(STICKY_TX_QUEUE_FULL, ERROR_UNIT_CAN, ERROR_TX_QUEUE_FULL, CAN_TX_QUEUE_SIZE);
#endif //COMPILE_FOR_DEVICE
		return((canStruct *)0x00000000);    // return NIL pointer if no room in the queue
	}
	else
	{
		return(&_gs._canTxQ.Q[_gs._canTxQ.nextIn]);
	}
}
#elif defined (COMPILE_FOR_SYSTEM)
canStruct *canGetTxQueueNextInPtr()
{
	// function to return next open spot in the queue.  if there's no slot, wait at least 1ms to

	if (_gs._canTxQ.numMsg >= CAN_TX_QUEUE_SIZE)
	{
		if (_errors.sent.flags.canTxFull == FALSE)
		{
			_errors.sent.flags.canTxFull = TRUE;
			sendError("ERROR_TX_QUEUE_FULL");
		}
		return((canStruct *)0x00000000);    // return NIL pointer if no room in the queue
	}
	else
	{
		return(&_gs._canTxQ.Q[_gs._canTxQ.nextIn]);
	}
}
#endif //COMPILE_FOR_SYSTEM

////////////////////////////////////////////////////////////////////////////////

void canFillOutCanStruct(canSwStruct *canSwStructPtr, canbusFormat_t canFmt)
{
	// fill in reserved fields, numBytes, IDE, RTR, etc
	// TIR = Transmit Identifier Regsister (from CAN hardware)
	//
	//     = stdID[10:0], extID[17:0], IDE, RTR, 0
	//
	//     = 0b100, msgType[1:0], device[7:0], msgId[7:0], page[7:0], IDE, RTR, 0
	//
	//        31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
	//     =   1  0  0 | mT|      devicePosition    |      msgId[7:0]       |        page[7:0]      |XI|RT| 0|
	//                                                                                               XI = IDE
	//                                                                                               RT = RTR
	// IDE = Extended IDentifier (additional 18 bits of header info)
	// RTR = Return Transmit Request = (msgType == CAN_READ)
	//
	// IDE is always set
	// upper 3 bits always == 0b100
	// lsb is always 0

	canSwStructPtr->fixed_b100      = 0b100;
	canSwStructPtr->IDE             = 1;
	canSwStructPtr->RTR             = (canSwStructPtr->msgType == CAN_READ);        // set RTR on CAN_READ, 0 otherwise
	canSwStructPtr->fixed_b0        = 0b0;
	canSwStructPtr->numBytes        = canMsgIdToNumBytes(canSwStructPtr->msgId, canFmt);
#ifdef COMPILE_FOR_SYSTEM
	canSwStructPtr->device          = remapDevicePosition(canSwStructPtr->device); // remap alias
#endif //COMPILE_FOR_SYSTEM
}

////////////////////////////////////////////////////////////////////////////////

byte canAddToTxQueue(canSwStruct *canSwStructPtr)
{
	// function to add incoming can hw Struct to transmit queue
	// incoming pointer may already be pointing into the queue, in which case, just update
	// indices, otherwise, data must be copied into the queue from the *pointer.

	canStruct *nextInPtr;

#ifdef COMPILE_FOR_SYSTEM
	if (_printAir && !((canSwStructPtr->msgId==CAN_MSG_CONTROL_WORD) && (canSwStructPtr->payload.u32[1]==HH_COMM_PING_BIT)))
	{   // nuke any non comm ping control word message to heads
		return(CAN_TX_OK);
	}
	if (isAPhysicalDevice(canSwStructPtr->device))
	{       // actual hardware device (not alias, broadcast, etc)
		if (!deviceRegistered(canSwStructPtr->device))
		{       // not registered, then not out there, so do not send
			if (_errors.sent.flags.canDestinationUnknown == FALSE)
			{
				_errors.sent.flags.canDestinationUnknown = TRUE;
				sprintf(_tmpStr, "Dropping CAN packet to unknown device (%d)\n", canSwStructPtr->device);
				sendError(_tmpStr);
			}
			return(CAN_TX_UNREGISTERED_DESTINATION);
		}
	}
#endif // COMPILE_FOR_SYSTEM

#if defined (COMPILE_DEVICE_FOR_CANBUS_FORMAT_V1)
	canbusFormat_t canFmt = CANBUS_FORMAT_V1;
#elif defined (COMPILE_DEVICE_FOR_CANBUS_FORMAT_V2)
	canbusFormat_t canFmt = CANBUS_FORMAT_V2;
#elif defined (COMPILE_FOR_SYSTEM)
	canbusFormat_t canFmt = getInboxPointer(canSwStructPtr->device)->canbusFormat;
#endif

	canFillOutCanStruct(canSwStructPtr, canFmt);    // fill in reserved fields, numBytes, IDE, RTR, etc

	byte retValue = CAN_TX_OK;
#if defined (COMPILE_FOR_SYSTEM)
	if (_gs._canTxQ.numMsg == 0)
#elif defined(COMPILE_FOR_DEVICE)
		if (CAN_INITIALIZED && (_gs._canTxQ.numMsg == 0))
#endif
		{       // queue is empty, so jump right to the hardware
			retValue = canTransmitSwPacket(canSwStructPtr);
			if (retValue != CAN_TxStatus_NoMailBox)
			{
				return(CAN_TX_OK);      // packet sent bypassing the sw queue, so we're done.
			}
		}
	// if we get here, the queue was not empty OR was empty but there was no room in the CAN hw unit 3-deep fifo
	// so add the packet to the software based queue

	if ((nextInPtr = canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	if ((canStruct *)canSwStructPtr != nextInPtr)   // data is not in Q, so copy over
	{
		*nextInPtr = *(canStruct *)canSwStructPtr;
	}

	_gs._canTxQ.numMsg++;
	_gs._canTxQ.nextIn++;


#ifdef COLLECT_METRICS
	_metrics.total_canTx++;
	if (_gs._canTxQ.numMsg > _metrics.max_canTxQ_numMsg)
		_metrics.max_canTxQ_numMsg = _gs._canTxQ.numMsg;
#endif

	if (_gs._canTxQ.nextIn == CAN_TX_QUEUE_SIZE)
	{
		_gs._canTxQ.nextIn = 0;
	}

	return(CAN_TX_OK);
}

////////////////////////////////////////////////////////////////////////////////

byte WriteExtrusionCommandtoTxQue(byte device, byte msgId,uint16_t Parameter0, uint16_t Parameter1,
											 uint16_t Parameter2, uint16_t Parameter3)
{
	return (canPackIntoTxQueue4x16(CAN_WRITE, device, msgId, (byte)0,FALSE,
							   Parameter0,Parameter1,Parameter2,Parameter3));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue1x32(byte msgType, byte device, byte msgId, byte page, boolean immediate,
							uint32_t u32_0)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u32[0]  = u32_0;
	canSwStructPtr->payload.u32[1]  = 0;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue2x32(byte msgType, byte device, byte msgId, byte page, boolean immediate,
		uint32_t u32_0, uint32_t u32_1)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u32[0]  = u32_0;
	canSwStructPtr->payload.u32[1]  = u32_1;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue1x32_2x16(byte msgType, byte device, byte msgId, byte page, boolean immediate,
		uint32_t u32_0, uint16_t u16_2, uint16_t u16_3)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u32[0]  = u32_0;
	canSwStructPtr->payload.u16[2]  = u16_2;
	canSwStructPtr->payload.u16[3]  = u16_3;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue1x32_1x16(byte msgType, byte device, byte msgId, byte page, boolean immediate,
		uint32_t u32_0, uint16_t u16_2)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u32[0]  = u32_0;
	canSwStructPtr->payload.u16[2]  = u16_2;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue1x16(byte msgType, byte device, byte msgId, byte page, boolean immediate,
								uint16_t u16_0)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u16[0]  = u16_0;
	canSwStructPtr->payload.u16[1]  = 0;
	canSwStructPtr->payload.u16[2]  = 0;
	canSwStructPtr->payload.u16[3]  = 0;

	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue2x16(byte msgType, byte device, byte msgId, byte page, boolean immediate,
		uint16_t u16_0, uint16_t u16_1)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u16[0]  = u16_0;
	canSwStructPtr->payload.u16[1]  = u16_1;
	canSwStructPtr->payload.u16[2]  = 0;
	canSwStructPtr->payload.u16[3]  = 0;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue3x16(byte msgType, byte device, byte msgId, byte page, boolean immediate,
		uint16_t u16_0, uint16_t u16_1, uint16_t u16_2)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u16[0]  = u16_0;
	canSwStructPtr->payload.u16[1]  = u16_1;
	canSwStructPtr->payload.u16[2]  = u16_2;
	canSwStructPtr->payload.u16[3]  = 0;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue4x16(byte msgType, byte device, byte msgId, byte page, boolean immediate,
		uint16_t u16_0, uint16_t u16_1, uint16_t u16_2, uint16_t u16_3)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u16[0]  = u16_0;
	canSwStructPtr->payload.u16[1]  = u16_1;
	canSwStructPtr->payload.u16[2]  = u16_2;
	canSwStructPtr->payload.u16[3]  = u16_3;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue1x8(byte msgType, byte device, byte msgId, byte page, boolean immediate, byte u8_0)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u8[0]  = u8_0;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue2x8(byte msgType, byte device, byte msgId, byte page, boolean immediate, byte u8_0, byte u8_1)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u8[0]  = u8_0;
	canSwStructPtr->payload.u8[1]  = u8_1;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueue8x8(byte msgType, byte device, byte msgId, byte page, boolean immediate, byte payload[])
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u64 = *((uint64_t *)payload);
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canPackIntoTxQueueNoData(byte msgType, byte device, byte msgId, byte page, boolean immediate)
{
	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = msgType;
	canSwStructPtr->device          = device;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = immediate;
	canSwStructPtr->page            = page;
	return(canAddToTxQueue(canSwStructPtr));

}

////////////////////////////////////////////////////////////////////////////////

#ifdef COMPILE_FOR_DEVICE
byte canDevicePackWriteIntoTxQueue2x32(byte msgId, byte page, uint32_t u32_0, uint32_t u32_1)
{
	// function to pack can info and add to transmit queue

	canSwStruct *canSwStructPtr;

	if ((canSwStructPtr = (canSwStruct *)canGetTxQueueNextInPtr()) == 0)
	{
		return(ERROR_TX_QUEUE_FULL);
	}

	canSwStructPtr->msgType         = CAN_WRITE;
	canSwStructPtr->device          = _gs._devicePosition;
	canSwStructPtr->msgId           = msgId;
	canSwStructPtr->immediate       = FALSE;
	canSwStructPtr->page            = page;
	canSwStructPtr->payload.u32[0]  = u32_0;
	canSwStructPtr->payload.u32[1]  = u32_1;
	return(canAddToTxQueue(canSwStructPtr));
}

////////////////////////////////////////////////////////////////////////////////

byte canDevicePackEventIntoTxQueue1x32(byte page, uint32_t u32_0)
{
	// function to pack can info and add to transmit queue

	return(canDevicePackWriteIntoTxQueue2x32(CAN_MSG_EVENT_MESSAGE, page, u32_0, 0));
}
#endif

////////////////////////////////////////////////////////////////////////////////
// RECEIVE ROUTINES ////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void canGetMailboxData(CAN_FIFOMailBox_TypeDef *mailboxPtr, canHwStruct *canHwStructPtr)
{
	canHwStructPtr->IR  = mailboxPtr->RIR;
	canHwStructPtr->DTR = mailboxPtr->RDTR;
	canHwStructPtr->DLR = mailboxPtr->RDLR;
	canHwStructPtr->DHR = mailboxPtr->RDHR;
}

////////////////////////////////////////////////////////////////////////////////

boolean canMsgWaitingFifo0(CAN_TypeDef *CANx)
{
	return((uint8_t)(CANx->RF0R&(uint32_t)0x03) != 0);
}

////////////////////////////////////////////////////////////////////////////////

boolean canMsgWaitingFifo1(CAN_TypeDef *CANx)
{
	return((uint8_t)(CANx->RF1R&(uint32_t)0x03) != 0);
}

////////////////////////////////////////////////////////////////////////////////

byte canReceiveFifo0(CAN_TypeDef *CANx, canHwStruct *canHwStructPtr)
{
	// low level routine to copy data from the CAN hardware unit

	CAN_FIFOMailBox_TypeDef *mailboxPtr;

	if (canMsgWaitingFifo0(CANx))
	{
		mailboxPtr = &CANx->sFIFOMailBox[CAN_FIFO0];
	}
	else
	{
		return(WARNING_RX_NO_MSG_WAITING);
	}

	canGetMailboxData(mailboxPtr, canHwStructPtr);

	CANx->RF0R |= CAN_RF0R_RFOM0;   // Release FIFO0
	return(CAN_RX_OK);
}

////////////////////////////////////////////////////////////////////////////////

byte canReceiveFifo1(CAN_TypeDef *CANx, canHwStruct *canHwStructPtr)
{
	// low level routine to copy data from the CAN hardware unit

	CAN_FIFOMailBox_TypeDef *mailboxPtr;

	if (canMsgWaitingFifo1(CANx))
	{
		mailboxPtr = &CANx->sFIFOMailBox[CAN_FIFO1];
	}
	else
	{
		return(WARNING_RX_NO_MSG_WAITING);
	}

	canGetMailboxData(mailboxPtr, canHwStructPtr);

	CANx->RF1R |= CAN_RF1R_RFOM1;   // Release FIFO1
	return(CAN_RX_OK);
}

////////////////////////////////////////////////////////////////////////////////

byte canReceive(CAN_TypeDef *CANx, uint8_t FifoNumber, canHwStruct *canHwStructPtr)
{
	// low level routine to copy data from the CAN hardware unit

	if (FifoNumber == CAN_FIFO0)
	{
		return(canReceiveFifo0(CANx, canHwStructPtr));
	}
	else //if (FifoNumber == CAN_FIFO1)
	{
		return(canReceiveFifo1(CANx, canHwStructPtr));
	}
}

////////////////////////////////////////////////////////////////////////////////

byte canAddToImmediateRxQueue()
{
	// first check if a message is wait and if so, is it an immediate mode
	// ALL immediate messages are directed to Fifo1 (independent of device id/alias)
	// so also need to check against alias table to know the message is for this device
	int returnValue = WARNING_RX_NO_MSG_WAITING;

	if (canMsgWaitingFifo1(CAN1) == TRUE)
	{
		if (!canIsValidAlias(((canSwStruct *)&CAN1->sFIFOMailBox[CAN_FIFO1])->device))  // not for us, so reject here
		{
			CAN1->RF1R |= CAN_RF1R_RFOM1;   // Release FIFO1
		}
		else if (_gs._canImmediateRxIsAvail == FALSE)   // single immediate message buffer is empty
		{
			canReceiveFifo1(CAN1, (canHwStruct *)&_gs._canImmediateRx);
			_gs._canImmediateRx.sw.fromCAN2 = 0;
			_gs._canImmediateRxIsAvail = TRUE;
		}
		returnValue = CAN_RX_OK;
	}
#ifdef COMPILE_FOR_SYSTEM
#ifdef USE_CAN2
		if (canMsgWaitingFifo1(CAN2) == TRUE)
		{
			if (!canIsValidAlias(((canSwStruct *)&CAN2->sFIFOMailBox[CAN_FIFO1])->device))  // not for us, so reject here
			{
				CAN2->RF1R |= CAN_RF1R_RFOM1;   // Release FIFO1
			}
			else if (_gs._canImmediateRxIsAvail == FALSE)   // single immediate message buffer is empty
			{
				canReceiveFifo1(CAN2, (canHwStruct *)&_gs._canImmediateRx);
				_gs._canImmediateRx.sw.fromCAN2 = 1;
				_gs._canImmediateRxIsAvail = TRUE;
			}
			returnValue = CAN_RX_OK;
		}
#endif //USE_CAN2
#endif //COMPILE_FOR_SYSTEM
#if defined(COMPILE_FOR_DEVICE)
if (_gs._canImmediateRxIsAvail == TRUE)
	{
		// activate status lights for receive
		_gs._led.canRxLedCnt = LED_COUNTER_START_VALUE_FOR_ONE_SHOT_FLASH; // will result in 10ms to 20ms on time;
	}
#endif

	return(returnValue);
}

////////////////////////////////////////////////////////////////////////////////

byte canAddToRxQueue()
{
	// function to take data from the CAN hw and add it to the global RX queue
	// if data available in CAN hardware, grab it and put it in the RX queue

	int returnValue = WARNING_RX_NO_MSG_WAITING;

	if (_gs._canRxQ.numMsg >= CAN_RX_QUEUE_SIZE)
	{
#if defined(COMPILE_FOR_DEVICE)
		reportError1x32(STICKY_RX_QUEUE_FULL, ERROR_UNIT_CAN, ERROR_RX_QUEUE_FULL, CAN_RX_QUEUE_SIZE);
#elif defined (COMPILE_FOR_SYSTEM)
		if (_errors.sent.flags.canRxFull == FALSE)
		{
			_errors.sent.flags.canRxFull = TRUE;
			sendError("ERROR_RX_QUEUE_FULL");
		}
#endif
		return(ERROR_RX_QUEUE_FULL);
	}

	if (canMsgWaitingFifo0(CAN1) == TRUE)
	{
		canReceiveFifo0(CAN1, (canHwStruct *)&_gs._canRxQ.Q[_gs._canRxQ.nextIn]);

#if defined (COMPILE_FOR_SYSTEM)
		if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
		{
			float currTime =  (float)_gs._sliceCnt / (float)(SYSTICKS_PER_SECOND);
			canSwStruct *canRx = (canSwStruct *)&_gs._canRxQ.Q[_gs._canRxQ.nextIn];
			sprintf(_rptStr, "%9.4f - CRX: %2d %02x %2x %d ", currTime, canRx->device, canRx->msgId, canRx->page, canRx->numBytes);
			int i;
			for (i=0; i<canRx->numBytes; i++)
			{
				sprintf(_tmpStr, "%02x.", canRx->payload.u8[i]);
				strcat(_rptStr, _tmpStr);
			}
			sendGB(_rptStr);
		}
#endif // COMPILE_FOR_SYSTEM

		_gs._canRxQ.Q[_gs._canRxQ.nextIn].sw.fromCAN2 = 0;
		_gs._canRxQ.numMsg++;
#ifdef COLLECT_METRICS
		_metrics.total_canRx++;
		if (_gs._canRxQ.numMsg > _metrics.max_canRxQ_numMsg)
			_metrics.max_canRxQ_numMsg = _gs._canRxQ.numMsg;
#endif //COLLECT_METRICS
		_gs._canRxQ.nextIn++;
		if (_gs._canRxQ.nextIn == CAN_RX_QUEUE_SIZE)
		{
			_gs._canRxQ.nextIn = 0;
		}
		// activate status lights for receive
		_gs._led.canRxLedCnt = LED_COUNTER_START_VALUE_FOR_ONE_SHOT_FLASH; // will result in 10ms to 20ms on time;
		returnValue = CAN_RX_OK;
	}

#if defined (COMPILE_FOR_SYSTEM)
#ifdef USE_CAN2
		// process CAN2
		if (_gs._canRxQ.numMsg >= CAN_RX_QUEUE_SIZE)
		{
			if (_errors.sent.flags.canRxFull == FALSE)
			{
				_errors.sent.flags.canRxFull = TRUE;
				sendError("ERROR_RX_QUEUE_FULL");
			}
			return(ERROR_RX_QUEUE_FULL);
		}

		if (canMsgWaitingFifo0(CAN2) == TRUE)
		{
			canReceiveFifo0(CAN2, (canHwStruct *)&_gs._canRxQ.Q[_gs._canRxQ.nextIn]);

			if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
			{
				float currTime =  (float)_gs._sliceCnt / (float)(SYSTICKS_PER_SECOND);
				canSwStruct *canRx = (canSwStruct *)&_gs._canRxQ.Q[_gs._canRxQ.nextIn];
				sprintf(_rptStr, "%9.4f - CR2: %2d %02x %2x %d ", currTime, canRx->device, canRx->msgId, canRx->page, canRx->numBytes);
				int i;
				for (i=0; i<canRx->numBytes; i++)
				{
					sprintf(_tmpStr, "%02x.", canRx->payload.u8[i]);
					strcat(_rptStr, _tmpStr);
				}
				sendGB(_rptStr);
			}

			_gs._canRxQ.Q[_gs._canRxQ.nextIn].sw.fromCAN2 = 1;
			_gs._canRxQ.numMsg++;
#ifdef COLLECT_METRICS
			_metrics.total_canRx++;
			if (_gs._canRxQ.numMsg > _metrics.max_canRxQ_numMsg)
				_metrics.max_canRxQ_numMsg = _gs._canRxQ.numMsg;
#endif //COLLECT_METRICS
			_gs._canRxQ.nextIn++;
			if (_gs._canRxQ.nextIn == CAN_RX_QUEUE_SIZE)
			{
				_gs._canRxQ.nextIn = 0;
			}
			returnValue = CAN_RX_OK;
		}
#endif //USE_CAN2
#endif // COMPILE_FOR_SYSTEM
	return(returnValue);
}

////////////////////////////////////////////////////////////////////////////////

boolean canPrepNextRx(void)
{
	boolean canRxIsAvail;
	if (_gs._canImmediateRxIsAvail == TRUE) // immediate packet to process (skipping queue)
	{
		_gs._canRx = _gs._canImmediateRx; // copy to staging area for execution
		_gs._canImmediateRxIsAvail = FALSE; // signal that we've freed up the immediate packet
		canRxIsAvail = TRUE;
	}
	else if (_gs._canRxQ.numMsg > 0)
	{
		_gs._canRx = _gs._canRxQ.Q[_gs._canRxQ.nextOut];  // copy from queue to staging area for execution
		_gs._canRxQ.numMsg--;
		_gs._canRxQ.nextOut++;
		if (_gs._canRxQ.nextOut == CAN_RX_QUEUE_SIZE)
		{
			_gs._canRxQ.nextOut = 0;
		}
		canRxIsAvail = TRUE;
	}
	else
	{
		canRxIsAvail = FALSE;
	}
	if (canRxIsAvail == TRUE)
	{   // packet available to attempt to process
		if (canCheckCanStruct(&_gs._canRx) != PASS)
		{   // bad formatted packet, so don't process
			canRxIsAvail = FALSE;
		}
	}
	return(canRxIsAvail);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef COMPILE_FOR_SYSTEM
extern void canFillDeviceBuffer(byte, uint32_t);
int canSendStringToWorkingBufferOnDevice(byte device, char str[])
{	// sends nullchar as well
	char *subStrPtr = str;
	uint32_t length = umin(strlen(str)+1, MAX_CAN_STRING_SIZE);
	int32_t charsRemaining = length;
	uint32_t page=0;

	canFillDeviceBuffer(device, 0x00000000);//zero out the working buffer on the device
	while (charsRemaining > 0)
	{
		canPackIntoTxQueue8x8(CAN_WRITE, device, CAN_MSG_ACCESS_BUFFER, page++, FALSE, (byte *)subStrPtr);
		charsRemaining -= 8;
		subStrPtr += 8;
	}

	return(length);
}

////////////////////////////////////////////////////////////////////////////////

int canSendAsciiHexStringAsBinaryToWorkingBufferOnDevice(byte device, char str[])
{
	char *subStrPtr = str;
	byte payload[8];
//	payloadUnion payload;
	uint32_t length = umin(strlen(str), WORKING_BUFFER_SIZE_IN_BYTES * 2);	//2:1 compression
	uint32_t charsRemaining = length;
	uint32_t page=0;
	uint32_t charsInPacket;
	byte nibble;

	while (charsRemaining > 0)
	{
		charsInPacket = umin(charsRemaining, 16);
		bzero(payload, 8);
//		payload.u64 = 0;
		for (uint32_t c=0; c<charsRemaining; c++)
		{
			nibble = asciihex2bin(*(subStrPtr+c));
			payload[c/2] = (nibble << 4*((c&1)^1));	// need to "nibble flip" when filling the bytes
//			payload.u64 = (nibble << (4*(c^1)));	// need to "nibble flip" when filling the bytes
		}
		canPackIntoTxQueue8x8(CAN_WRITE, device, CAN_MSG_ACCESS_BUFFER, page++, FALSE, payload);
//		canPackIntoTxQueue8x8(CAN_WRITE, device, CAN_MSG_ACCESS_BUFFER, page++, FALSE, payload.u8);

		charsRemaining -= charsInPacket;
		subStrPtr += charsInPacket;
	}
	return(length);
}

////////////////////////////////////////////////////////////////////////////////

boolean canReceiveString(canSwStruct *canRx, int *charsReceived, char *receiveString)
// receiving string from host, up to 8 chars at a time, incude the NULL_CHAR
// canRx->page is the # of chars in this packet
// so if page = 8; then 8 valid chars in this packet and only need to check byte 7 for NULL_CHAR to know if EOS
//    if page < 8, then this is last packet and NULL_CHAR will be in the page-1 byte
{
	boolean lastPacket = FALSE;

	if ((*charsReceived + canRx->page) <= MAX_CAN_STRING_SIZE)
	{   // room in buffer
		memcpy(receiveString + *charsReceived, canRx->payload.u8, canRx->page);
		*charsReceived += canRx->page;
		lastPacket =  (canRx->payload.u8[canRx->page-1] == '\0');
	}
	else
	{
		*(receiveString + *charsReceived - 1) = 0;
		*charsReceived = 0;
	}
	return(lastPacket);
}
#endif //COMPILE_FOR_SYSTEM

////////////////////////////////////////////////////////////////////////////////

#ifdef COMPILE_FOR_DEVICE

void canSendString(byte device, char str[])
{
	char *subStrPtr = str;
	byte payload[8];
	uint32_t charsInPacket;
	uint32_t charsRemaining = strlen(str) + 1; // includes NULL_CHAR

	while (charsRemaining > 0)
	{
		charsInPacket = umin(charsRemaining, 8);
		memcpy(payload, (byte *)subStrPtr, charsInPacket);
		canPackIntoTxQueue8x8(CAN_WRITE, device, CAN_MSG_STRING, charsInPacket, FALSE, payload);
		charsRemaining -= charsInPacket;
		subStrPtr += charsInPacket;
	}
}
#endif //COMPILE_FOR_DEVICE
////////////////////////////////////////////////////////////////////////////////
// SIMPLE ROUTINES /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// provided so that functions can be used in the main control loop without having
// a return value

void canAddToRxQueueNoReturn(void)
{
	canAddToRxQueue();
}

////////////////////////////////////////////////////////////////////////////////

void canProcessTxQueueNoReturn(void)
{
	canProcessTxQueue();
}

////////////////////////////////////////////////////////////////////////////////

