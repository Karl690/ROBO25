////////////////////////////////////////////////////////////////////////////////
//
// File:    Hydra_can.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: The
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

// add includes here
#include "main.h"
#include "Serial.h"
#include "Hydra_can.h"
#include "gpio.h"
#include "bootloader.h"  //NUKE GB XXX gregkarl -- can we strip out the HH bootloader code?
#include "GCode.h"
#include "MotorDriver.h"
#include "HardwareInit.h"

extern byte PersistantUltimusControlHeadAddress; // very few items needed from GCode.h
extern void ScheduleSoapStringRead(byte); // very few items needed from GCode.h

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
//  Public Global Definitions (exposed in Hydra_can.h)
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
//  Local Global Definitions (do not expose in Hydra_can.h)
//
////////////////////////////////////////////////////////////////////////////////

payloadUnion _loopbackPayload;

////////////////////////////////////////////////////////////////////////////////


void sendControlRegisterBit(byte device, byte set, uint32_t bit, boolean immediate)
{
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_CONTROL_WORD, NO_PAGE, immediate, (set == 1) ? bit : 0, bit);
	if (bit != HH_COMM_PING_BIT)
	{   // don't bother updating things based on the comm watchdog
		if (set)
			getOutboxPointer(device)->controlWord.u32 |= bit;
		else
			getOutboxPointer(device)->controlWord.u32 &= ~bit;
		canIssueReadRequest(device, CAN_MSG_CONTROL_WORD, NO_PAGE);
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendSetRegisterBit(byte device) {
	sendControlRegisterBit(device, 1, HH_REGISTERED_BIT, BUFFERED_MSG);
}
void sendSetSwResetBit(byte device) {
	sendControlRegisterBit(device, 1, HH_SW_RESET_BIT, IMMEDIATE_MSG);
}
void sendSetGlobalSyncBit(byte device) {
	sendControlRegisterBit(device, 1, HH_GLOBAL_SYNC_BIT, IMMEDIATE_MSG);
}
void sendSetStopExtruderBit(byte device) {
	sendControlRegisterBit(device, 1, HH_STOP_EXTRUDER_BIT, IMMEDIATE_MSG);
}

void sendSetPauseExtruderBit(byte device) {
	sendControlRegisterBit(device, 1, HH_PAUSE_EXTRUDER_BIT, IMMEDIATE_MSG);
}
void sendSetStopOnErrorBit(byte device) {
	sendControlRegisterBit(device, 1, HH_STOP_ON_ERROR_BIT, BUFFERED_MSG);
}
void sendSetSaveHistoryBit(byte device) {
	sendControlRegisterBit(device, 1, HH_SAVE_HISTORY_BIT, BUFFERED_MSG);
}
void sendSetClearErrorBit(byte device) {
	sendControlRegisterBit(device, 1, HH_CLEAR_ERROR_BIT, IMMEDIATE_MSG);
}

void sendSetAckOnWriteBit(byte device) {
	sendControlRegisterBit(device, 1, HH_ACK_ON_WRITE_BIT, BUFFERED_MSG);
}
void sendSetInvertDirectionBit(byte device) {
	sendControlRegisterBit(device, 1, HH_INVERT_DIRECTION_BIT, BUFFERED_MSG);
}
void sendSetEnableMotorBit(byte device) {
	sendControlRegisterBit(device, 1, HH_ENABLE_MOTOR_BIT, BUFFERED_MSG);
	getOutboxPointer(device)->motorState = MOTOR_ENABLED_STATE;
}
void sendSetDisableMotorBit(byte device) {
	sendControlRegisterBit(device, 1, HH_DISABLE_MOTOR_BIT, BUFFERED_MSG);
	getOutboxPointer(device)->motorState = MOTOR_DISABLED_STATE;
}

void sendSetDebugRtdBit(byte device) {
	sendControlRegisterBit(device, 1, HH_DEBUG_RTD_BIT, BUFFERED_MSG);
}
void sendSetBlockFashWritesBit(byte device) {
	sendControlRegisterBit(device, 1, HH_BLOCK_FLASH_WRITES_BIT, BUFFERED_MSG);
}

void sendSetPrimeBit(byte device) {
	sendControlRegisterBit(device, 1, HH_PRIME_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendSetUnprimeBit(byte device) {
	sendControlRegisterBit(device, 1, HH_UNPRIME_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendSetGateBit(byte device) {
	sendControlRegisterBit(device, 1, HH_GATE_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendSetRetractBit(byte device) {
	sendControlRegisterBit(device, 1, HH_RETRACT_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendSetUpdateBootloaderBit(byte device) {
	sendControlRegisterBit(device, 1, HH_UPDATE_BOOTLOADER_BIT, BUFFERED_MSG);
}
void sendSetCommPingBit(byte device) {
	sendControlRegisterBit(device, 1, HH_COMM_PING_BIT, BUFFERED_MSG);
}
void sendSetStepFromCanbusModeBit(byte device) {
	sendControlRegisterBit(device, 1, HH_STEP_FROM_CANBUS_MODE_BIT, BUFFERED_MSG);
}
void sendSetIgnoreWatchdogBit(byte device) {
	sendControlRegisterBit(device, 1, HH_IGNORE_WATCHDOG_BIT, BUFFERED_MSG);
}
void sendSetSendDebugStringsBit(byte device) {
	sendControlRegisterBit(device, 1, HH_SEND_DEBUG_STRINGS_BIT, BUFFERED_MSG);
}
void sendInitializeBit(byte device) {
	sendControlRegisterBit(device, 1, HH_INITIALIZE_BIT, BUFFERED_MSG);
}

void sendClrRegisterBit(byte device) {
	sendControlRegisterBit(device, 0, HH_REGISTERED_BIT, BUFFERED_MSG);
}
void sendClrSwResetBit(byte device) {
	sendControlRegisterBit(device, 0, HH_SW_RESET_BIT, BUFFERED_MSG);
}
void sendClrGlobalSyncBit(byte device) {
	sendControlRegisterBit(device, 0, HH_GLOBAL_SYNC_BIT, BUFFERED_MSG);
}
void sendClrStopExtruderBit(byte device) {
	sendControlRegisterBit(device, 0, HH_STOP_EXTRUDER_BIT, IMMEDIATE_MSG);
}

void sendClrPauseExtruderBit(byte device) {
	sendControlRegisterBit(device, 0, HH_PAUSE_EXTRUDER_BIT, IMMEDIATE_MSG);
}
void sendClrStopOnErrorBit(byte device) {
	sendControlRegisterBit(device, 0, HH_STOP_ON_ERROR_BIT, BUFFERED_MSG);
}
void sendClrSaveHistoryBit(byte device) {
	sendControlRegisterBit(device, 0, HH_SAVE_HISTORY_BIT, BUFFERED_MSG);
}
void sendClrClearErrorBit(byte device) {
	sendControlRegisterBit(device, 0, HH_CLEAR_ERROR_BIT, IMMEDIATE_MSG);
}

void sendClrAckOnWriteBit(byte device) {
	sendControlRegisterBit(device, 0, HH_ACK_ON_WRITE_BIT, BUFFERED_MSG);
}
void sendClrInvertDirectionBit(byte device) {
	sendControlRegisterBit(device, 0, HH_INVERT_DIRECTION_BIT, BUFFERED_MSG);
}
void sendClrDebugRtdBit(byte device) {
	sendControlRegisterBit(device, 0, HH_DEBUG_RTD_BIT, BUFFERED_MSG);
}
void sendClrBlockFashWritesBit(byte device) {
	sendControlRegisterBit(device, 0, HH_BLOCK_FLASH_WRITES_BIT, BUFFERED_MSG);
}

void sendClrPrimeBit(byte device) {
	sendControlRegisterBit(device, 0, HH_PRIME_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendClrUnprimeBit(byte device) {
	sendControlRegisterBit(device, 0, HH_UNPRIME_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendClrGateBit(byte device) {
	sendControlRegisterBit(device, 0, HH_GATE_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendClrRetractBit(byte device) {
	sendControlRegisterBit(device, 0, HH_RETRACT_EXTRUDER_BIT, BUFFERED_MSG);
}
void sendClrUpdateBootloaderBit(byte device) {
	sendControlRegisterBit(device, 0, HH_UPDATE_BOOTLOADER_BIT, BUFFERED_MSG);
}
void sendClrCommPingBit(byte device) {
	sendControlRegisterBit(device, 0, HH_COMM_PING_BIT, BUFFERED_MSG);
}
void sendClrStepFromCanbusModeBit(byte device) {
	sendControlRegisterBit(device, 0, HH_STEP_FROM_CANBUS_MODE_BIT, BUFFERED_MSG);
}
void sendClrIgnoreWatchdogBit(byte device) {
	sendControlRegisterBit(device, 0, HH_IGNORE_WATCHDOG_BIT, BUFFERED_MSG);
}
void sendClrSendDebugStringsBit(byte device) {
	sendControlRegisterBit(device, 0, HH_SEND_DEBUG_STRINGS_BIT, BUFFERED_MSG);
}

////////////////////////////////////////////////////////////////////////////////

void canIssueReadRequest(byte device, byte msgId, byte page) {
	canPackIntoTxQueue2x32(CAN_READ, device, msgId, page, BUFFERED_MSG, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void readDeviceInfoFromDevice(byte device) {
	canIssueReadRequest(device, CAN_MSG_DEVICE_INFO, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

void readFlashConfigFromDevice(byte device) {
	canIssueReadRequest(device, CAN_MSG_FLASH_CONFIG, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

void readUniqueIdFromDevice(byte device) {
	canIssueReadRequest(device, CAN_MSG_UNIQUE_ID, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

void readAliasListFromDevice(byte device) {
	canIssueReadRequest(device, CAN_MSG_PRE_DEFINED_ALIASES, NO_PAGE);
	canIssueReadRequest(device, CAN_MSG_USER_DEFINED_ALIASES, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

void readPageDefinitionFromDevice(byte device) {
	canIssueReadRequest(device, CAN_MSG_PAGE_DEF, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

void readPageChecksumFromDevice(byte device, byte page) {
	canIssueReadRequest(device, CAN_MSG_PAGE_CHECKSUM, page);
}

////////////////////////////////////////////////////////////////////////////////

void readControlWordFromDevice(byte device) {
	canIssueReadRequest(device, CAN_MSG_CONTROL_WORD, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

float scaleLaserVectorPowerPct(float overridePct)
{
	return(_gs._laser.vectorPowerPct * overridePct);
}

////////////////////////////////////////////////////////////////////////////////

void sendExtrusionOverridePctToDevice(outboxStruct *outboxPtr, float overridePct)
{

	// ExtrusionRateOverridePct             - individual per head
	// scale factor is set to head as a 6.10 format

	overridePct = fFitWithinRange(overridePct, OVERRIDE_PCT_MIN, OVERRIDE_PCT_MAX);
	canPackIntoTxQueue1x32(CAN_WRITE, outboxPtr->device, CAN_MSG_FLOW_SCALE_FACTORS, 0, BUFFERED_MSG,
			(int32_t)(overridePct * ((float)(1 << HH_SCALE_FACTOR_FRAC_BITS))));

	outboxPtr->ExtrusionControl.ExtrusionRateOverridePct = overridePct; // save a copy

	if (_sendingGBStringsMask & GB_STRING_FLOW) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr,"Flow Fudge Factor Set=%f", overridePct);
		sendGB(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

uint16_t convertFloatPowerPctToUnsignedint(float powerPct, int fracBits)
{   // input float in the range of 0.0 to 1.0 (no checks are performed)
	uint32_t maxVal = (1 << fracBits);
	return((uint16_t)imin((uint32_t)(powerPct * ((float)maxVal) + 0.1f), maxVal)); // fraction in 0.bits-1) format -- supports "1.0"
}

////////////////////////////////////////////////////////////////////////////////

void SetLocalLaserPwmPct(float LaserPowerPct)
{   // only allow local control with a V1 head -- required to pass back watchdog from power supply head
	// per spec, pwm output is high when TIM->CNT < TIM->CCR;  TIM-CNT ranges from 0 to TIM->ARR.'
	// to have fully on (100%, TIM->CCR needs to be set to TIM->ARR+1 (actual arr value), but limited to
	unsigned int maxVal = imin(((unsigned int)_gs._laser.TimerBase->ARR) + 1, 0xffff);
	//_gs._laser.TimerBase->CCR2 = (LASER_ENABLED) ? fminf(LaserPowerPct * (float)maxVal, maxVal) : 0;
	TIM8->CCR3 = LaserPowerPct; //tun on pwm for laser
	
}
////////////////////////////////////////////////////////////////////////////////

void SetLaserPower(float LaserPowerPct, uint32_t LaserWatchDogTimerMs)
{
	if (!LASER_ENABLED)
	{   // shut things down!
		LaserPowerPct = 0.0f;
		LaserWatchDogTimerMs = 0;
	}

	if (_gs._laser.device)
	{   // checking for valid laser device
		if (_gs._laser.outboxPtr)
		{
			SetupLaserToRun(_gs._laser.outboxPtr, (LaserWatchDogTimerMs) ? HEAD_FUNCTION_PAGE_FLAG_RUN : 0, LaserPowerPct, LaserWatchDogTimerMs, 0.0f, FALSE);
		}
	}

#ifdef GB_LASER_POWER_NONZERO_PIN
	pinWriteBit(GB_LASER_POWER_NONZERO_PIN, (LaserPower > 0));
#endif
}

////////////////////////////////////////////////////////////////////////////////

void SetLaserPowerOnly(float LaserPowerPct) {
	// will send out as an immediate can packet to the selected device via
	// CAN_MSG_SET_LASER_POWER with the desired power in the "page" field, such that
	// the can packet is 0 data bytes and therefore the smallest packet supported.
	// NOTE: the rest of the structure is set up in global_inits and should not be altered.
	// remap device to keep M714 functionality working (may choose to nuke at this point)

	if (_gs._laser.enabled == FALSE)
	{   // shut things down!
		LaserPowerPct = 0.0f;
	}

	if (LASER_LOCAL_PWM_CONTROL)
	{
		SetLocalLaserPwmPct(LaserPowerPct);
	}
	else if (_gs._laser.device)
	{
		if (getOutboxPointer(_gs._laser.device)->deviceFamily == DEVICE_FAMILY_LASER)
		{	// diode laser  -- SPECIAL CONTROL ON CANBUS FOR THIS OFT USED PACKET:  CAN_MSG_SET_POWER_LEVEL_ONLY (uses page field for data)
			// packet is pre-filled will all but the destination and power level
			_gs._preCannedPackets[PRECANNED_LASER].page = (byte)convertFloatPowerPctToUnsignedint(LaserPowerPct, HH_U8_POWER_PCT_FRAC_BITS);
			//NUKE protectedIncrement(&_gs._preCannedLaserPending);    // flag to send the packet on the next systick
			canAddToTxQueue(&_gs._preCannedPackets[PRECANNED_LASER]);
		}
		else
		{	// inkjet  -- SPECIAL CONTROL ON CANBUS FOR THIS OFT USED PACKET:  CAN_MSG_STEP (uses page field for 0 | 1)
			// packet is pre-filled will all info
			if (convertFloatPowerPctToUnsignedint(LaserPowerPct, HH_U8_POWER_PCT_FRAC_BITS))
			{	// only send packets that spew ink
				//NUKE protectedIncrement(&_gs._preCannedLaserPending);    // flag to send the packet on the next systick
				canAddToTxQueue(&_gs._preCannedPackets[PRECANNED_LASER]);
			}
		}
	}
#ifdef GB_LASER_POWER_NONZERO_PIN
	pinWrite(GB_LASER_POWER_NONZERO_PIN, (LaserPowerPct > 0.0f));
#endif
}

////////////////////////////////////////////////////////////////////////////////

void sendProbeControlToDevice(byte device, probeType_t probeType, probeAction_t probeAction, int value, int count, int initState, HH_IoIndex_t pin)
{
	if (((getInboxPointer(device)->softwareMinorVersion == 5) && (getInboxPointer(device)->softwareTweakVersion >= 'b')) || (getInboxPointer(device)->softwareMinorVersion > 5))
		canPackIntoTxQueue4x16(CAN_WRITE, device, CAN_MSG_TOUCH_PROBE, probeType, BUFFERED_MSG, probeAction, value, count, (pin << 8) | initState);
	else
		canPackIntoTxQueue4x16(CAN_WRITE, device, CAN_MSG_TOUCH_PROBE_OLD, probeType, BUFFERED_MSG, probeAction, value, count, initState);
}

////////////////////////////////////////////////////////////////////////////////

void sendSpindleControl(outboxStruct *outboxPtr, uint16_t direction, uint16_t pageFlags)
{
	pageFlags |= HEAD_FUNCTION_PAGE_LOAD_POWER | HEAD_FUNCTION_PAGE_LOAD_RATE;  // always load power and direction
	canPackIntoTxQueue3x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HEAD_SPINDLE_CONTROL, pageFlags, FALSE,
			_gs._spindle.pwmFreq,
			(uint16_t)convertFloatPowerPctToUnsignedint(_gs._spindle.desiredPowerPct, HH_U16_POWER_PCT_FRAC_BITS),
			direction);
}

////////////////////////////////////////////////////////////////////////////////

void updateManualExtrudingOffForGUI(byte device)
{
	if (isAPhysicalDevice(device))
	{
		outboxStruct *outboxPtr = getOutboxPointer(device);
		if (outboxPtr->ExtrusionControl.isManuallyExtruding)
		{
			outboxPtr->ExtrusionControl.isManuallyExtruding = FALSE;
			SendFakeMcodeExecutionNotice(723, outboxPtr->device, 0.0f, INVALID_ARG_VALUE);
		}
	}
	else
	{
		int i;
		for (i=0; i<NUM_PHYSICAL_DEVICES; i++)        //walks through each and every element to see if it needs to report
		{
			inboxStruct *inboxPtr = getInboxPointer(_MailBoxes._inbox[i].device);
			if (inboxPtr->deviceRegistered && matchesAnAlias(device, inboxPtr))
			{
				outboxStruct *outboxPtr = getOutboxPointer(inboxPtr->device);
				if (outboxPtr->ExtrusionControl.isManuallyExtruding)
				{
					SendFakeMcodeExecutionNotice(723, outboxPtr->device, 0.0f, INVALID_ARG_VALUE);
					outboxPtr->ExtrusionControl.isManuallyExtruding = FALSE;
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void RunExtrusionManual(outboxStruct *outboxPtr)
{
	canPackIntoTxQueue2x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HEAD_EXTRUDE_CONTROL,
			(HEAD_FUNCTION_PAGE_LOAD_RATE | HEAD_FUNCTION_PAGE_LOAD_QTY | HEAD_FUNCTION_PAGE_FLAG_RUN), BUFFERED_MSG,
			imin((uint16_t)(((float)outboxPtr->ExtrusionControl.ExtrudeFeedRate)), 0xffff),
			(uint16_t)outboxPtr->ExtrusionControl.ExtrudeSteps);

	// only path in to this method is from M723 ... it will handle sending the McodeCompletion notice
	outboxPtr->ExtrusionControl.isManuallyExtruding = (outboxPtr->ExtrusionControl.ExtrudeSteps == 0) ? FALSE : TRUE;
}

////////////////////////////////////////////////////////////////////////////////

void StopExtruding(outboxStruct *outboxPtr)
{ //stop extruder motor
	canPackIntoTxQueueNoData(CAN_WRITE, outboxPtr->device, CAN_MSG_STOP, NO_PAGE, BUFFERED_MSG);
	if (_sendingGBStringsMask & GB_STRING_EXTRUSION) // use M797 S<mask> to enable
	{
		sendGB("StopExtruding");
	}
	updateManualExtrudingOffForGUI(outboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void StopAllExtruders(void)
{ //stop extruder motor
	canPackIntoTxQueueNoData(CAN_WRITE, ALIAS_ALL_EXTRUDERS, CAN_MSG_STOP, NO_PAGE, BUFFERED_MSG);
	if (_sendingGBStringsMask & GB_STRING_EXTRUSION) // use M797 S<mask> to enable
	{
		sendGB("StopAllExtruding");
	}
	updateManualExtrudingOffForGUI(0);
}

////////////////////////////////////////////////////////////////////////////////

void DisableAllExtruderMotors(void)
{
	sendSetDisableMotorBit(ALIAS_ALL_EXTRUDERS);

	if (_sendingGBStringsMask & GB_STRING_EXTRUSION) // use M797 S<mask> to enable
	{
		sendGB("Extruder Motors Disabled");
	}
}

////////////////////////////////////////////////////////////////////////////////

void EnableAllExtruderMotors(void)
{
	sendSetEnableMotorBit(ALIAS_ALL_EXTRUDERS);

	if (_sendingGBStringsMask & GB_STRING_EXTRUSION)
	{ // use M797 S<mask> to enable
		sendGB("Extruder Motors Enabled");
	}
}

////////////////////////////////////////////////////////////////////////////////

int getCurrentUnPrimeTimeMs(outboxStruct *outboxPtr)
{
	return(outboxPtr->ExtrusionControl.UnPrimeTimeMs);
}

int getCurrentPrimeTimeMs(outboxStruct *outboxPtr)
{
	return(outboxPtr->ExtrusionControl.PrimeTimeMs);
}

////////////////////////////////////////////////////////////////////////////////

void UnPrime(outboxStruct *outboxPtr)
{  //stop "material" flow (and retract)
	canPackIntoTxQueueNoData(CAN_WRITE, outboxPtr->device, CAN_MSG_UNPRIME, NO_PAGE, BUFFERED_MSG);

	if (LASER_LOCAL_PWM_CONTROL)
	{   // special case of directly controlling co2 laser from the motherboard's timer
		SetLocalLaserPwmPct(0);
		TIM8->CCR3 = 0; //tun on pwm for laser
	}

	if (PersistantUltimusControlHeadAddress == outboxPtr->device)
	{
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[ULTIMUS_HSS]], HSS_DUTY_CYCLE_OFF);
	}
	if (_sendingGBStringsMask & GB_STRING_PRIME) // use M797 S<mask> to enable
	{
		sendGB("UNPRIME");
	}
#ifdef GB_UNPRIME_PIN
	pinSet(GB_UNPRIME_PIN);
#endif
#ifdef COLLECT_METRICS
	_metrics.unprimes++;
#endif
}

////////////////////////////////////////////////////////////////////////////////

void Prime(outboxStruct *outboxPtr)
{ //prime extruder, but do NOT continue running
	canPackIntoTxQueueNoData(CAN_WRITE, outboxPtr->device, CAN_MSG_PRIME, NO_PAGE, BUFFERED_MSG);

	if (LASER_LOCAL_PWM_CONTROL)    // special case
	{   // still need to send
		SetLocalLaserPwmPct(_gs._laser.piercePowerPct);
	}

	if (_sendingGBStringsMask & GB_STRING_PRIME) // use M797 S<mask> to enable
	{
		sendGB("PRIME");
	}
#ifdef GB_PRIME_PIN
	pinSet(GB_PRIME_PIN);
#endif
#ifdef COLLECT_METRICS
	_metrics.primes++;
#endif
}

////////////////////////////////////////////////////////////////////////////////

void PrimeThenRun(outboxStruct *outboxPtr)
{ //prime extruder, but do NOT continue running
	canPackIntoTxQueueNoData(CAN_WRITE, outboxPtr->device, CAN_MSG_PRIME_RUN, NO_PAGE, BUFFERED_MSG);

	if (LASER_LOCAL_PWM_CONTROL)    // special case
	{
		SetLocalLaserPwmPct(_gs._laser.piercePowerPct);
		//TIM8->CCR3 = DesiredCo2LaserPower; //tun on pwm for laser
	}

	if (PersistantUltimusControlHeadAddress == outboxPtr->device)
	{
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[ULTIMUS_HSS]], HSS_DUTY_CYCLE_ON);
	}

	updateManualExtrudingOffForGUI(outboxPtr->device);

	if (_sendingGBStringsMask & GB_STRING_PRIME) // use M797 S<mask> to enable
	{
		sendGB("PRIME then RUN");
	}
#ifdef GB_PRIME_PIN
	pinSet(GB_PRIME_PIN);
#endif
#ifdef COLLECT_METRICS
	_metrics.primes++;
#endif
}

////////////////////////////////////////////////////////////////////////////////

void Run(outboxStruct *outboxPtr)
{ //prime extruder, but do NOT continue running
	canPackIntoTxQueueNoData(CAN_WRITE, outboxPtr->device, CAN_MSG_RUN, NO_PAGE, BUFFERED_MSG);

	if (LASER_LOCAL_PWM_CONTROL)    // special case
	{   // still need to send
		SetLocalLaserPwmPct(scaleLaserVectorPowerPct(_CompositeFlowOverridePct));
	}

	if (PersistantUltimusControlHeadAddress == outboxPtr->device)
	{
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[ULTIMUS_HSS]], HSS_DUTY_CYCLE_ON);
	}

	if (_sendingGBStringsMask & GB_STRING_PRIME) // use M797 S<mask> to enable
	{
		sendGB("RUN");
	}

	updateManualExtrudingOffForGUI(outboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void Send_Prime_Prams(outboxStruct *outboxPtr)
{ //this will send the current values for prime to the current selectedHead
	if (outboxPtr->deviceFamily == DEVICE_FAMILY_LASER)
	{
		WriteExtrusionCommandtoTxQue(outboxPtr->device, CAN_MSG_EXTRUSION_PRIME,
			(uint16_t)1000, // ms rate
			LASER_LOCAL_PWM_CONTROL ? 0 : (uint16_t)convertFloatPowerPctToUnsignedint(_gs._laser.piercePowerPct, HH_U16_POWER_PCT_FRAC_BITS),
			(uint16_t) outboxPtr->ExtrusionControl.PrimeTimeMs, 0);  // pierceTime from head
	}
	else
	{
		WriteExtrusionCommandtoTxQue(outboxPtr->device, CAN_MSG_EXTRUSION_PRIME,
			(uint16_t) outboxPtr->ExtrusionControl.PrimeFeedRate, //how many steps per second
			(uint16_t) outboxPtr->ExtrusionControl.PrimeSteps,
			(uint16_t) outboxPtr->ExtrusionControl.PrimeTimeMs, 0); //number of pulses if prime or unprime  0xffff for continous movement
	}
}

////////////////////////////////////////////////////////////////////////////////

void Send_UnPrime_Prams(outboxStruct *outboxPtr)
{ //this will send the current values for Unprime to the current selectedHead
	WriteExtrusionCommandtoTxQue(outboxPtr->device, CAN_MSG_EXTRUSION_UNPRIME,
			(uint16_t) outboxPtr->ExtrusionControl.UnPrimeFeedRate,  //how many steps per second during retract
			(uint16_t) outboxPtr->ExtrusionControl.UnPrimeSteps,     //how many steps to take during retract
			(uint16_t) outboxPtr->ExtrusionControl.UnPrimeTimeMs,0); //time (ms) to prefire the unprime before the previous move is finished
}

////////////////////////////////////////////////////////////////////////////////

boolean flagMatchesMask(uint32_t flag, uint32_t mask)
{
	return((flag & mask) == mask);
}

////////////////////////////////////////////////////////////////////////////////

void SetupExtruderToRun(outboxStruct *outboxPtr, uint32_t pageFlags, uint32_t watchdog)
{   // NOTE: watchdog added strictly for cases where a laser family device is cloned to an extruder

	pageFlags |= HEAD_FUNCTION_PAGE_LOAD_RATE | HEAD_FUNCTION_PAGE_LOAD_QTY | HEAD_FUNCTION_PAGE_LOAD_WATCHDOG;

	if (watchdog > (0xffff - 1000))
	{
		watchdog = (watchdog + 999) / 1000; // convert to seconds (ceiling func)
		pageFlags |= HEAD_FUNCTION_PAGE_FLAG_WATCHDOG_IN_SEC;
	}

	pageFlags |= HEAD_FUNCTION_PAGE_LOAD_RATE | HEAD_FUNCTION_PAGE_LOAD_QTY;
	canPackIntoTxQueue3x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HEAD_EXTRUDE_CONTROL, pageFlags, BUFFERED_MSG,
			imin((uint16_t)(((float)outboxPtr->ExtrusionControl.ExtrudeFeedRate)), 0xffff),
			(uint16_t)outboxPtr->ExtrusionControl.ExtrudeSteps,
			watchdog);
	if (flagMatchesMask(pageFlags, HEAD_FUNCTION_PAGE_FLAG_RUN) && (PersistantUltimusControlHeadAddress != HH_POSITION_UNPLUGGED))
	{
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[ULTIMUS_HSS]], HSS_DUTY_CYCLE_ON);
	}

	if (flagMatchesMask(pageFlags, HEAD_FUNCTION_PAGE_FLAG_RUN) || flagMatchesMask(pageFlags, HEAD_FUNCTION_PAGE_FLAG_PRIME_RUN))
	{
		updateManualExtrudingOffForGUI(outboxPtr->device);

#ifdef GB_PRIME_PIN
		pinSet(GB_PRIME_PIN);
#endif
#ifdef COLLECT_METRICS
		_metrics.primes++;
#endif
	}
}

////////////////////////////////////////////////////////////////////////////////

void SetupLaserToRun(outboxStruct *outboxPtr, uint32_t pageFlags, float laserPowerPct, uint32_t watchdog, float motionRate, boolean rasterMove)
{
#ifdef GB_HIDDEN_WARNINGS
	int addMechanismToLoadPoloarityControlForLasers;
#endif //GB_HIDDEN_WARNINGS

	pageFlags |= HEAD_FUNCTION_PAGE_LOAD_RATE | HEAD_FUNCTION_PAGE_LOAD_POWER | HEAD_FUNCTION_PAGE_LOAD_WATCHDOG;

	if (watchdog > (0xffff - 1000))
	{
		watchdog = (watchdog + 999) / 1000; // convert to seconds (ceiling func)
		pageFlags |= HEAD_FUNCTION_PAGE_FLAG_WATCHDOG_IN_SEC;
	}

	if (outboxPtr->deviceFamily == DEVICE_FAMILY_LASER)
	{
		canPackIntoTxQueue3x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HEAD_LASER_CONTROL, pageFlags, BUFFERED_MSG,
				_gs._laser.pwmFreq,
				LASER_LOCAL_PWM_CONTROL ? 0 : (uint16_t)convertFloatPowerPctToUnsignedint(laserPowerPct, HH_U16_POWER_PCT_FRAC_BITS),
				watchdog);
		if (LASER_LOCAL_PWM_CONTROL)
		{
			if (pageFlags & HEAD_FUNCTION_PAGE_FLAG_PRIME)
			{
				if ((_gs._laser.outboxPtr->ExtrusionControl.PrimeTimeMs > 0) && (watchdog > 0))
					SetLocalLaserPwmPct(_gs._laser.piercePowerPct);
				else
					SetLocalLaserPwmPct(0.0f);
			}
			else
			{
				if (watchdog > 0)
					SetLocalLaserPwmPct(laserPowerPct);
				else
					SetLocalLaserPwmPct(0.0f);
			}
		}
	}
	else //if (outboxPtr->deviceFamily == DEVICE_FAMILY_INKJET)
	{
		canPackIntoTxQueue3x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HEAD_INKJET_CONTROL, pageFlags, BUFFERED_MSG,
				(rasterMove ?
						iFitWithinRange((uint32_t)(motionRate * _gs._laser.rasterImageDotsPerMmX), 3, 65535) :
						iFitWithinRange((uint32_t)(motionRate * _gs._inkjet.dropletsPerMm), 3, 65535)), // pwmFreq mm/sec * droplets/mm == droplets/sec (3 is min time for inkjet timers)
				(uint16_t)(laserPowerPct > 0.0f) ? 1 : 0, // it's on or off ..... no range
				watchdog);
	}
#ifdef GB_LASER_POWER_NONZERO_PIN
	pinWrite(GB_LASER_POWER_NONZERO_PIN, (_gs._laser.vectorPowerPct > 0));
#endif

	if (flagMatchesMask(pageFlags, HEAD_FUNCTION_PAGE_FLAG_RUN) && (PersistantUltimusControlHeadAddress != HH_POSITION_UNPLUGGED))
	{
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[ULTIMUS_HSS]], HSS_DUTY_CYCLE_ON);
	}

#if defined(GB_PRIME_PIN) || defined(COLLECT_METRICS)
	if ((pageFlags & HEAD_FUNCTION_PAGE_FLAG_PRIME) || (pageFlags & HEAD_FUNCTION_PAGE_FLAG_PRIME_RUN))
	{
#ifdef GB_PRIME_PIN
		pinSet(GB_PRIME_PIN);
#endif
#ifdef COLLECT_METRICS
		_metrics.primes++;
#endif
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////

void sendSwitchControlOffToDevice(outboxStruct *outboxPtr, byte switchNum)
{
	outboxPtr->swTarg[switchNum].temperature = 0.0f;
	outboxPtr->swTarg[switchNum].dutyCycle   = 0;
	canPackIntoTxQueue1x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HSS_CONTROL, switchNum, BUFFERED_MSG, SWITCH_CONTROL_OFF);
}

////////////////////////////////////////////////////////////////////////////////

void sendSwitchControlOnToDevice(outboxStruct *outboxPtr, byte switchNum)
{
	outboxPtr->swTarg[switchNum].temperature = 0.0f;
	outboxPtr->swTarg[switchNum].dutyCycle   = 100;
	canPackIntoTxQueue1x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HSS_CONTROL, switchNum, BUFFERED_MSG, SWITCH_CONTROL_ON);
}

////////////////////////////////////////////////////////////////////////////////

void sendSwitchControlByDutyCycleToDevice(outboxStruct *outboxPtr, byte switchNum, byte dutyCycle)
{
	outboxPtr->swTarg[switchNum].temperature = 0.0f;
	outboxPtr->swTarg[switchNum].dutyCycle   = dutyCycle;
	canPackIntoTxQueue3x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HSS_CONTROL, switchNum, BUFFERED_MSG,
			SWITCH_CONTROL_DUTY_CYCLE,
			SWITCH_PRESCALE_1,
			dutyCycle);
}

////////////////////////////////////////////////////////////////////////////////

void sendSwitchControlByGeneralPwmToDevice(outboxStruct *outboxPtr, byte switchNum, float onPct, uint16_t pmwFreq)
{
	outboxPtr->swTarg[switchNum].temperature = 0.0f;
	outboxPtr->swTarg[switchNum].dutyCycle   = 0;
	canPackIntoTxQueue4x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HSS_CONTROL, switchNum, BUFFERED_MSG,
			SWITCH_CONTROL_PWM,
			SWITCH_PRESCALE_1,
			convertFloatPowerPctToUnsignedint(onPct, HH_U16_POWER_PCT_FRAC_BITS),
			pmwFreq);
}

////////////////////////////////////////////////////////////////////////////////

void sendSwitchControlByTempToDevice(outboxStruct *outboxPtr, byte switchNum, uint16_t preScale, float targetTemp)
{
	outboxPtr->swTarg[switchNum].temperature = targetTemp;
	outboxPtr->swTarg[switchNum].dutyCycle   = 0;
	canPackIntoTxQueue3x16(CAN_WRITE, outboxPtr->device, CAN_MSG_HSS_CONTROL, switchNum, BUFFERED_MSG,
			SWITCH_CONTROL_BY_TEMP,
			preScale,
			(int16_t)(outboxPtr->swTarg[switchNum].temperature * TEMP_SCALE));
}

////////////////////////////////////////////////////////////////////////////////

void sendAutoStatusControlToDevice(outboxStruct *outboxPtr, byte flags)
{
	if (getInboxPointer(outboxPtr->device)->softwareMinorVersion >= 4)  //LEGACY REMOVE
	{
		canPackIntoTxQueue2x32(CAN_WRITE, outboxPtr->device, CAN_MSG_AUTO_STATUS_CONTROL, flags , BUFFERED_MSG,
				(uint32_t) outboxPtr->autoStatusMask,
				(uint32_t) outboxPtr->autoStatusPeriodMs);
	}
}

////////////////////////////////////////////////////////////////////////////////

void canWriteSwitchFlag_onOnlyWhenExtruding(outboxStruct *outboxPtr, int switchNum, int value)
{
	switchFlags_t mask = {0};
	mask.b.onOnlyWhenExtruding = 1;
	canSendOutboxInitValue2x32(outboxPtr, (switchNum == 0) ? DEV_INIT_AREA_SW0 : DEV_INIT_AREA_SW1, DEV_INIT_INDEX_SWx_FLAGS, (value == 0) ? 0 : mask.u32, mask.u32);
}

////////////////////////////////////////////////////////////////////////////////

//void sendCooldownTimeToDevice(outboxStruct *outboxPtr, uint16_t cooldown)
//{
//	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV,  DEV_INIT_INDEX_DEV_COOLDOWN, cooldown);
//}

////////////////////////////////////////////////////////////////////////////////
//
//void sendMotorCurrentBoostToDevice(outboxStruct *outboxPtr, boostCtrl_t currentBoost)
//{
//	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_BOOST, currentBoost);
//}

////////////////////////////////////////////////////////////////////////////////
//
//void sendMotorMaxRateToDevice(outboxStruct *outboxPtr, uint16_t maxRate)
//{
//	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MAX_RATE, maxRate);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//
//void sendMotorMicroStepsToDevice(outboxStruct *outboxPtr, uint16_t microstepCode)
//{
//	// FIX IN REV5
//	// new heads want to know how many microsteps, not a code based on allegro parts
//	int microsteps;
//	switch (microstepCode)
//	{
//	case  0 : microsteps = 1; break;
//	case  1 : microsteps = 2; break;
//	case  2 : microsteps = 4; break;
//	case  3 : microsteps = 8; break;   // 4988 only
//	case  7 :
//	default : microsteps = 16; break;
//	}
//	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP, microsteps);
//}
//
//////////////////////////////////////////////////////////////////////////////////
//
//void sendInkjetNozzleSelectToDevice(outboxStruct *outboxPtr, uint16_t nozzle)
//{
//	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_INKJET_NOZZLE, nozzle);
//}

////////////////////////////////////////////////////////////////////////////////

void sendAddAliasToDevice(outboxStruct *outboxPtr, byte alias)
{
	canPackIntoTxQueue1x8(CAN_WRITE, outboxPtr->device, CAN_MSG_ADD_ALIAS, NO_PAGE, BUFFERED_MSG, alias);
	readAliasListFromDevice(outboxPtr->device);     // request back results of changes to keep inbox current
}

////////////////////////////////////////////////////////////////////////////////

void sendRemoveAliasToDevice(outboxStruct *outboxPtr, byte alias)
{
	canPackIntoTxQueue1x8(CAN_WRITE, outboxPtr->device, CAN_MSG_REMOVE_ALIAS, NO_PAGE, BUFFERED_MSG, alias);
	readAliasListFromDevice(outboxPtr->device);     // request back results of changes to keep inbox current
}

////////////////////////////////////////////////////////////////////////////////

void sendKarlFactorsToDevice(outboxStruct *outboxPtr, byte switchNum, int pf, int delta, int maxOS, int denom)
{
	canPackIntoTxQueue4x16(CAN_WRITE, outboxPtr->device, CAN_MSG_KARL_FACTORS, switchNum, BUFFERED_MSG,
			pf, delta, maxOS, denom);
}

////////////////////////////////////////////////////////////////////////////////
#ifdef CHECK_FOR_COLD_EXTRUDE
void sendColdExtrudeTempRangesToDevice(outboxStruct *outboxPtr, int switchNum, temp_t min, temp_t max)
{
	canPackIntoTxQueue2x16(CAN_WRITE, outboxPtr->device, CAN_MSG_EXTRUSION_TEMP_RANGES, switchNum, BUFFERED_MSG, min, max);
}
#endif //CHECK_FOR_COLD_EXTRUDE
////////////////////////////////////////////////////////////////////////////////

void canSendDeviceInitValue1x16(byte device, devInitArea_t area, devInitIndex_t index, uint16_t value)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue1x16(CAN_WRITE, device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value);
}

////////////////////////////////////////////////////////////////////////////////

void canSendDeviceInitValue2x16(byte device, devInitArea_t area, devInitIndex_t index, uint16_t value0, uint16_t value1)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue2x16(CAN_WRITE, device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value0, value1);
}

////////////////////////////////////////////////////////////////////////////////

void canSendDeviceInitValue1x32(byte device, devInitArea_t area, devInitIndex_t index, uint32_t value)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue1x32(CAN_WRITE, device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value);
}

////////////////////////////////////////////////////////////////////////////////

void canSendDeviceInitValue2x32(byte device, devInitArea_t area, devInitIndex_t index, uint32_t value0, uint32_t value1)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value0, value1);
}

////////////////////////////////////////////////////////////////////////////////

void canSendOutboxInitValue1x16(outboxStruct *outboxPtr, devInitArea_t area, devInitIndex_t index, uint16_t value)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue1x16(CAN_WRITE, outboxPtr->device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value);
}

////////////////////////////////////////////////////////////////////////////////

void canSendOutboxInitValue2x16(outboxStruct *outboxPtr, devInitArea_t area, devInitIndex_t index, uint16_t value0, uint16_t value1)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue2x16(CAN_WRITE, outboxPtr->device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value0, value1);
}

////////////////////////////////////////////////////////////////////////////////

void canSendOutboxInitValue1x32(outboxStruct *outboxPtr, devInitArea_t area, devInitIndex_t index, uint32_t value)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue1x32(CAN_WRITE, outboxPtr->device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value);
}

////////////////////////////////////////////////////////////////////////////////

void canSendOutboxInitValue2x32(outboxStruct *outboxPtr, devInitArea_t area, devInitIndex_t index, uint32_t value0, uint32_t value1)
{
	devPageFields_t page;  // encode destination
	page.b.area = area;
	page.b.index = index;
	canPackIntoTxQueue2x32(CAN_WRITE, outboxPtr->device, CAN_MSG_DEVICE_INIT, page.u8, BUFFERED_MSG, value0, value1);
}

////////////////////////////////////////////////////////////////////////////////

void sendLoopbackMessageToDevice(byte device, loopbackType type, uint16_t param0, uint16_t param1, uint16_t param2, uint16_t param3)
{
	_MailBoxes._waitingFor.flags.bit.canLoopback = TRUE;
	_MailBoxes._waitingFor.timerCanLoopback = CAN_WAIT_FOR_LOOPBACK_TIME;
	_loopbackPayload.u16[0] = param0;
	_loopbackPayload.u16[1] = param1;
	_loopbackPayload.u16[2] = param2;
	_loopbackPayload.u16[3] = param3;
	canPackIntoTxQueue8x8(CAN_WRITE, device, CAN_MSG_LOOPBACK, type, BUFFERED_MSG, _loopbackPayload.u8);
}

////////////////////////////////////////////////////////////////////////////////

void canFillDeviceBuffer(byte device, uint32_t fillData)
{
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_FILL_BUFFER, NO_PAGE, BUFFERED_MSG, fillData, 0);
}

////////////////////////////////////////////////////////////////////////////////

void canCopyPageToDeviceBuffer(byte device, byte page)
{
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_COPY_PAGE_TO_BUFFER, page, BUFFERED_MSG, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void canCopyDeviceBufferToPage(byte device, byte page, uint32_t checksum)
{
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_COPY_BUFFER_TO_PAGE, page, BUFFERED_MSG, checksum, 0);
}

////////////////////////////////////////////////////////////////////////////////

void canReadDeviceBuffer(byte device)
{   //reads the working buffer of the device
	canIssueReadRequest(device, CAN_MSG_ACCESS_BUFFER, NO_PAGE);
}

////////////////////////////////////////////////////////////////////////////////

void canWriteDeviceBuffer(byte device, byte page, payloadUnion *payload)
{
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_ACCESS_BUFFER, page, BUFFERED_MSG, payload->u32[0], payload->u32[1]);
}

////////////////////////////////////////////////////////////////////////////////

void sendStartPrimaryProgramToDevice(byte device)
{
	canPackIntoTxQueue2x32(CAN_WRITE, device, CAN_MSG_START_PRIMARY_PROGRAM, NO_PAGE, BUFFERED_MSG, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////
//NUKE
//void sendMotorSetFactoryDefaultsToDevice(byte device)
//{
//	canPackIntoTxQueueNoData(CAN_WRITE, device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_RESTORE_DEFAULTS, BUFFERED_MSG);
//}
////////////////////////////////////////////////////////////////////////////////

//void sendMotorStartCalibrationToDevice(byte device)
//{
//	canPackIntoTxQueueNoData(CAN_WRITE, device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_CALIBRATE, BUFFERED_MSG);
//}

////////////////////////////////////////////////////////////////////////////////

void sendMotorResetToDevice(byte device)
{
	canPackIntoTxQueueNoData(CAN_WRITE, device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_RESET_ALL, BUFFERED_MSG);
}

//////////////////////////////////////////////////////////////////////////////////

void sendMotorSaveConfigToDevice(byte device)
{
	canPackIntoTxQueueNoData(CAN_WRITE, device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_SAVE_CONFIG, BUFFERED_MSG);
}

////////////////////////////////////////////////////////////////////////////////

void printDeviceErrorMessage(canSwStruct *canRx)
{
	byte errorIndex;
	payloadUnion *payload;
	payload = &canRx->payload;

	if (getMailboxNum(canRx->device) >= 64)
	{
		barf("Programming Error ... mailboxNum too great for bitfield");
		return;
	}
	for (errorIndex=0; errorIndex<NUM_DEVICE_ERROR_CODES; errorIndex++)
	{
		if ((_deviceErrorDescriptionTable[errorIndex].unit == payload->u8[0]) && (_deviceErrorDescriptionTable[errorIndex].code == payload->u8[1])) // match
		{
			if ((_MailBoxes.stickyErrorMsg[errorIndex] & (1 << getMailboxNum(canRx->device))) == 0)
			{   // haven't seen this error yet for this device
				_MailBoxes.stickyErrorMsg[errorIndex] |= (1 << getMailboxNum(canRx->device));   // mark that this error has now been seen
			// page will contain the deviceType (0=BL)
			sprintf(_rptStr, ">ER: %2d: %d: %s: %s ", canRx->device, canRx->page,  _deviceErrorDescriptionTable[errorIndex].unitStr, _deviceErrorDescriptionTable[errorIndex].codeStr);

			if (_deviceErrorDescriptionTable[errorIndex].argsAreTemperatures == TRUE)  // args are temperatures, need to scale
			{
				switch (_deviceErrorDescriptionTable[errorIndex].numArgs)
				{
				case 1 :
					sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr,    (float)payload->u16[2] / TEMP_SCALE);
					break;
				case 2 :
					sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr,    (float)payload->u16[2] / TEMP_SCALE,
							(float)payload->u16[3] / TEMP_SCALE);
					break;
				default :
					sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr);
					break;
				}
			}
			else
			{
				if ((_deviceErrorDescriptionTable[errorIndex].unit == ERROR_UNIT_MOTION) && (_deviceErrorDescriptionTable[errorIndex].code == ERROR_MAX_ANGLE_ERROR_EXCEEDED))
				{	// special case with float arg
					sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr, payload->flt[1]);
				}
				else
				{
					switch (_deviceErrorDescriptionTable[errorIndex].numArgs)
					{
					case 1 :                            // one uint32_t
						sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr, payload->u32[1]);
						break;
					case 2 :                            // two uint16_t's)
						sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr, payload->u16[2], payload->u16[3]);
						break;
					case 3 :                            // three uint8_t's)
						sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr, payload->u8[4], payload->u8[5], payload->u8[6]);
						break;
					case 4 :                            // four uint8_t's)
						sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr, payload->u8[4], payload->u8[5], payload->u8[6], payload->u8[7]);
						break;
					default :
						sprintf(_tmpStr, _deviceErrorDescriptionTable[errorIndex].sprintfStr);
						break;
					}
				}
			}
			strcat(_rptStr, _tmpStr);
			//sprintf(_tmpStr, " (total errors=%d)", payload->u16[1]);
			//strcat(_rptStr, _tmpStr);
			sendstringCr(_rptStr);
			break;      // jump out of the for loop, as an error match was found
		}
			else
			{ //already display this error type before
				break;      // jump out of the for loop, as an error match was found
			}
		}
	}

	if (errorIndex == NUM_DEVICE_ERROR_CODES) // didn't find it
	{
		sprintf(_rptStr, ">ER: %2d: %02x %02x %02x %02x %02x %02x %02x %02x", canRx->device,
				payload->u8[0], payload->u8[1], payload->u8[2], payload->u8[3],
				payload->u8[4], payload->u8[5], payload->u8[6], payload->u8[7]);
		sendstringCr(_rptStr);
	}

	if ((_deviceErrorDescriptionTable[errorIndex].unit == ERROR_UNIT_CAN) && (_deviceErrorDescriptionTable[errorIndex].code == ERROR_COMM_TIMEOUT)) // special case to check
	{   // something is wrong with comm to the head (or fifo is wrongly back up, so try to recover
		sendSetSwResetBit(canRx->device); // try to reset device to get it to re-register
		if (currentOutboxPtr->device == canRx->device)
		{
			sprintf(_errorStr, "Lost comm to active head (%d) -- aborting job", canRx->device);
			catastrophicError(_errorStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

uint32_t getDevicePassword(byte device)
{
	return( ((uint32_t)getInboxPointer(device)->uniqueId[4] << 16) |
			((uint32_t)getInboxPointer(device)->uniqueId[2] << 8)  |
			((uint32_t)getInboxPointer(device)->uniqueId[0]));
}

////////////////////////////////////////////////////////////////////////////////

void unpackDeviceInfoPayload(inboxStruct *inboxPtr, canSwStruct *canRx)
{
	payloadUnion *payload = &canRx->payload;
	if (canRx->page == 0)
	{
		inboxPtr->softwareCodebase      = payload->u8[0];
		inboxPtr->softwareMajorVersion  = payload->u8[1];
		inboxPtr->softwareMinorVersion  = payload->u8[2];
		inboxPtr->softwareTweakVersion  = payload->u8[3];
		inboxPtr->softwareCompileTarget = payload->u8[4];
		if (payload->u8[5] == 0)
			inboxPtr->softwareDebugVersion = ' ';
		else
			inboxPtr->softwareDebugVersion  = payload->u8[5];
	}
	else if (canRx->page == 1)
	{
		inboxPtr->deviceType            = (soapDevType_t)payload->u8[1];
		inboxPtr->deviceSubtype         = (soapDevSubtype_t)payload->u8[2];
		inboxPtr->devicePcb             = (soapPcb_t)payload->u8[3];
		inboxPtr->deviceRtd1            = (soapRtd_t)payload->u8[4];
		inboxPtr->deviceRtd2            = (soapRtd_t)payload->u8[5];
		inboxPtr->deviceFamily          = devType2DevFamily(inboxPtr->deviceType);
	}
	else if (canRx->page == 2)
	{
		inboxPtr->motorTicksRev			= payload->u16[0];
	}

	getOutboxPointer(inboxPtr->device)->deviceFamily = inboxPtr->deviceFamily;
}

////////////////////////////////////////////////////////////////////////////////

void updateCloneList(void)
{
	int physLog, phys, aka;
	inboxStruct *physInboxPtr;
	outboxStruct *physLogOutboxPtr;
	for (physLog=0; physLog<NUM_PHYSICAL_PLUS_LOGICAL_DEVICES; physLog++)
	{
		if (_MailBoxes._inbox[physLog].deviceRegistered)
		{
			physLogOutboxPtr = &_MailBoxes._outbox[physLog];
			physLogOutboxPtr->numClones = 0;
			for (phys=0; phys<NUM_PHYSICAL_DEVICES; phys++)
			{
				if ((phys != physLog) && _MailBoxes._inbox[phys].deviceRegistered)
				{   // only check registed devices (except oursleves)
					physInboxPtr = &_MailBoxes._inbox[phys];
					for (aka=0; aka<NUM_USER_DEFINED_ALIASES; aka++)
					{
						if (physInboxPtr->userDefinedAliases[aka] != ALIAS_UNUSED)
						{
							physLogOutboxPtr->clones[physLogOutboxPtr->numClones] = physInboxPtr->device;
							physLogOutboxPtr->numClones++;
						}
					}
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

byte canProcessRxQueue(void)
{
	canSwStruct *canRx;
	payloadUnion *payload;
	inboxStruct *inboxPtr;
	byte i;
	uint32_t workingBufferChecksum;

	if (canPrepNextRx())
	{
		canRx = (canSwStruct *)&_gs._canRx.sw;
	}
	else
	{
		return(CAN_RX_OK);
	}

	inboxPtr = getInboxPointer(canRx->device);
	payload = &canRx->payload;
	inboxPtr->commTicker = HH_COMM_WATCHDOG_START_VALUE;        //reload the count down watchdog timer

	switch (canRx->msgType)
	{
	case CAN_WRITE:
		switch (canRx->msgId)
		{
		case CAN_MSG_EVENT_MESSAGE :
			switch(canRx->page)
			{
			case CAN_EVENT_DEVICE_ANNOUNCE:         // unregistered hothead or hotbed announcing their presence to the system
				if (inboxPtr->registrationStep == 0)
				{
					startDeviceRegistration(canRx);
				}
				break;
			case CAN_EVENT_BOOTLOADER_ANNOUNCE: // bootloader starts by default ... check if we care, if so, start the bootloader, otherwise, start main
				//LEGACY REMOVE;
				break;
				// fyi... the values for the BB_x indices are not sequential
			case CAN_EVENT_DEVICE_HEARTBEAT_0: inboxPtr->HBPayload[0] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_1: inboxPtr->HBPayload[1] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_2: inboxPtr->HBPayload[2] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_3: inboxPtr->HBPayload[3] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_4: inboxPtr->HBPayload[4] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_5: inboxPtr->HBPayload[5] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_6: inboxPtr->HBPayload[6] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_7: inboxPtr->HBPayload[7] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_8: inboxPtr->HBPayload[8] = *payload; break;
			case CAN_EVENT_DEVICE_HEARTBEAT_9: inboxPtr->HBPayload[9] = *payload; break;
				break;
				//temperature and laser power reporting
			case CAN_EVENT_MANUAL_Z_MOVE:       // hotbed controller reporting that the Z-Move switch was pressed
				if (cmdQueIsEmpty() && motionQ_empty())
				{   // only process request if no other activity
					// NOTE: default mounting orientation of the hotbed controller in 30M's is such that "reverse" equates to moving the bed down (increase Z)
					joggingUpdatePulses(M_Z, (float)payload->u32[0] * Motors[M_Z].JogIncrement * ((payload->u32[1] == HH_MOTOR_REVERSE) ? 1.0f : -1.0f));
				}
				break;
			case CAN_EVENT_PRIME_COMPLETE :
				_MailBoxes._waitingFor.flags.bit.prime = FALSE;
				break;
			case CAN_EVENT_LIMIT_SWITCH_ON :
				break;
			case CAN_EVENT_LIMIT_SWITCH_OFF :
				break;
			case CAN_EVENT_PROBE_REPORT :
				if (_TouchProbeCanDevice == canRx->device)
				{
					if (_TouchProbeMoveActive)
					{   // probe report tied to a G38 -- CONTACT!!!!
						TouchProbeProcessContact((probeType_t)payload->i32[0], payload->i32[1]);
					}
					else
					{   // requested probe firing not tied to a move
						setTouchPositionToCurrentPosition();
						TouchProbeSendResults(PROBE_RESULTS_CANBUS_PROBE_REPORT, (probeType_t)payload->i32[0], payload->i32[1]);
					}
				}
				else // (_TouchProbeCanDevice != canRx->device)
				{
					sprintf(_errorStr, "Unexpected canbus probe reporting (expected=%d; got=%d",
							_TouchProbeCanDevice, canRx->device);
					sendError(_errorStr);
				}
				break;
			case CAN_EVENT_PNP_DOWN_REPORT :
			case CAN_EVENT_PNP_UP_REPORT :
				sprintf(_rptStr, ">PP:%d:%d:%d:%d:%d", (canRx->page == CAN_EVENT_PNP_DOWN_REPORT) ? 0 : 1,
						payload->u16[0]>>TEMP_FRAC_BITS, payload->u16[1], payload->u16[2], payload->u16[3]);
				sendstringCr(_rptStr);
				break;
			case CAN_EVENT_PNP_MOTION_COMPLETE :
				_MailBoxes._waitingFor.flags.bit.motionPnP = FALSE;
				_MailBoxes._waitingFor.timerPnP = 0;
				break;
			case CAN_EVENT_PNP_MOTION_BLOCKED :
				sendError("Absolute move failed on PnP head -- must home motor");
				_MailBoxes._waitingFor.flags.bit.motionPnP = FALSE;
				_MailBoxes._waitingFor.timerPnP = 0;
				break;
			case CAN_EVENT_PROBE_ARMED :
				startProbingMove(); // now proceed with move
				_MailBoxes._waitingFor.flags.bit.canbusProbeToArm = FALSE;
				_MailBoxes._waitingFor.timerProbe = 0;
				break;
			case CAN_EVENT_PROBE_CONTACT_BEFORE_MOVE :
				// error condition --- probe not in the correct state to start a probe move
				_MailBoxes._waitingFor.flags.bit.canbusProbeToArm = FALSE;
				_MailBoxes._waitingFor.timerProbe = 0;
				TouchProbeFinished(PROBE_RESULTS_PROBE_CONTACT_BEFORE_MOVE_STARTED, (probeType_t)payload->i32[0], PROBE_ERROR);
				break;
			case CAN_EVENT_CO2_WATCHDOG_EXPIRED :
				SetLocalLaserPwmPct(0);
				break;
			case CAN_EVENT_CO2_PRIME_FINISHED_START_RUN :
				if (LASER_ENABLED && (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_LASER) && _gs._laser.localControl)
				{
					SetLocalLaserPwmPct(scaleLaserVectorPowerPct(_CompositeFlowOverridePct));
				}
				else
				{
					SetLocalLaserPwmPct(0.0f);
				}
				break;
#ifdef USE_CAN_MOTOR_HOME_SENSOR
#warning "finish-canMotor homing"
#warning "FIX ONCE H/L1/L2 on canMotors is resolved"
#endif //USE_CAN_MOTOR_HOME_SENSOR
			case CAN_EVENT_MOTION_LIMIT1_STATE_CHANGE :
			{
#ifdef USE_CAN_MOTOR_HOME_SENSOR
				MotorStructure *M = getMotorPtrFromDeviceAddress(canRx->device);
				if (M != NULL)
				{
					setSensorBitState(&M->HomeSense, payload->u32[0]);
				}
				sprintf(_rptStr, "(%d) CAN_EVENT_MOTION_LIMIT1_STATE_CHANGE: %s", canRx->device, (M->HomeSense.State == SENSOR_OPEN) ? "OPEN" : (M->HomeSense.State == SENSOR_TRIPPED) ? "TRIPPED" : "UNKNOWN");;
				sendInfo(_rptStr); break;
#else //!USE_CAN_MOTOR_HOME_SENSOR
				sprintf(_rptStr, "(%d) CAN_EVENT_MOTION_LIMIT1_STATE_CHANGE (%lu)", canRx->device, payload->u32[0]); sendInfo(_rptStr); break;
#endif //!USE_CAN_MOTOR_HOME_SENSOR
			}
			break;
			case CAN_EVENT_MOTION_LIMIT2_STATE_CHANGE :
				sprintf(_rptStr, "(%d) CAN_EVENT_MOTION_LIMIT2_STATE_CHANGE (%lu)", canRx->device, payload->u32[0]); sendInfo(_rptStr); break;
			case CAN_EVENT_ENCODER_CALIBRATION_PASS :
				sprintf(_rptStr, "(%d) CAN_EVENT_ENCODER_CALIBRATION_PASS (%lu)", canRx->device, payload->u32[0]); sendInfo(_rptStr); break;
			case CAN_EVENT_ENCODER_CALIBRATION_FAIL :
				sprintf(_rptStr, "(%d) CAN_EVENT_ENCODER_CALIBRATION_FAIL (%lu)", canRx->device, payload->u32[0]); sendInfo(_rptStr); break;
			case CAN_EVENT_ENCODER_CONFIG_SAVE_PASS :
				sprintf(_rptStr, "(%d) CAN_EVENT_ENCODER_CONFIG_SAVE_PASS", canRx->device); sendInfo(_rptStr); break;
			case CAN_EVENT_ENCODER_CONFIG_SAVE_FAIL :
				sprintf(_rptStr, "(%d) CAN_EVENT_ENCODER_CONFIG_SAVE_FAIL", canRx->device); sendInfo(_rptStr); break;
			case CAN_EVENT_ENCODER_BAD_CALIBRATION :
				sprintf(_rptStr, "(%d) CAN_EVENT_ENCODER_BAD_CALIBRATION", canRx->device); sendInfo(_rptStr); break;
			case CAN_EVENT_ENCODER_CALIBRATING : break;	// nothing to do, just for comm pinging
			case CAN_EVENT_MOTION_HOMING_IN_PROCESS :
				_MailBoxes._waitingFor.timerHoming = CANBUS_HOMING_COMPLETION_TIME;	// someone still homing
				break;
			case CAN_EVENT_MOTION_HOMING_COMPLETE:	//u32[0] == PASS/FAIL
				{
					MotorStructure *M = getMotorPtrFromDeviceAddress(canRx->device);
					if (M != NULL)
					{
						int canAddrIndex = getCanMotorIndexFromDeviceAddress(canRx->device);
						M->canMotors[canAddrIndex].selfHomingInProgress = FALSE;	// finished
						M->canMotors[canAddrIndex].selfHomed = ((passFail_t)payload->u32[0] == PASS) ? TRUE : FALSE;

						boolean axisFinished = TRUE;
						boolean homed = TRUE;

						for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
						{	// check if all them are finished
							if (M->canMotors[canAddrIndex].canAddress)
							{	// only check if motor is on the canbus
								if (M->canMotors[canAddrIndex].selfHomingInProgress)
								{
									axisFinished = FALSE;
									break;
								}
								if (M->canMotors[canAddrIndex].selfHomed == FALSE)
								{
									homed = FALSE;
									break;
								}
							}
						}
						if (axisFinished)
						{	// the reporting axis is done, so check the rest to see if any remain to be homed
							M->HasBeenHomed = homed;
							M->axisSelfHomingInProgress = FALSE;
							// not check ALL axis to see if there's any work remaining....
							boolean allAxesFinished = TRUE;
							for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
							{   // whiz through all the motors and see if any are still homing
								if (M->canMotor && M->axisSelfHomingInProgress)
								{
									allAxesFinished = FALSE;
									break;
								}
							}
							if (allAxesFinished)
							{
								_MailBoxes._waitingFor.flags.bit.canAxisMotorHoming = FALSE;
								_MailBoxes._waitingFor.timerHoming = 0;
							}
						}
					}
				}
				break;
			case CAN_EVENT_MOTION_JOGGING_COMPLETE:
				break;
			case CAN_EVENT_DEVICE_WIPE_SAVED_SETTINGS:
				sprintf(_rptStr, "Wiping config flash on device (%d) (c=0x%08lu/k=0x%08lu)", canRx->device, payload->u32[0], payload->u32[1]);
				sendInfo(_rptStr);
				break;
			default :
				sprintf(_errorStr, "Unknown event message received (%d) received from device (%d)", canRx->page, canRx->device);
				sendError(_errorStr);
			}
			break;
			case CAN_MSG_REPORT_ERROR:      // hothead or hotbed reporting an error condition
				printDeviceErrorMessage(canRx);
				break;
			case CAN_MSG_LOOPBACK :
				//this appears to be fired at the END of a block transfer
				_MailBoxes._waitingFor.flags.bit.canLoopback = FALSE;
				_loopbackPayload = *payload;    // grab a copy of the payload for other routines to use
				switch ((loopbackType)canRx->page)
				{
				case DEVICE_INITIALIZATION :
					if (_sendingGBStringsMask & GB_STRING_CANBUS) // use M797 S<mask> to enable
					{
						sprintf(_tmpStr, "register %d ended",  payload->u16[0]); sendGB(_tmpStr);
					}
					// u16[0] holds sequence/step
					switch (payload->u16[0])
					{
					case 1: // startReg complete so schedule full init.
						inboxPtr->registrationStep = payload->u16[0] + 1;   // let watchdog know step is complete
						break;
					case 2: // finishReg complete
						inboxPtr->registrationStep = payload->u16[0] + 1; // let watchdog know step is complete
						DeviceSoapstringWatchdogCnt =  0; // clear watchdog
						break;
					case 3: // read soap complete
						inboxPtr->registrationStep = 0; //  done .. no more steps
						DeviceSoapstringWatchdogCnt = 0; // clear watchdog
						// registration complete, so let host know:
#ifdef ALLOW_NATIVE_LIGHTBURN
						if (_lightburnModeEnabled)
							sprintf(_rptStr, "TOOL %d (canbus addr %d) :", convertDeviceToToolNumber(inboxPtr->device), inboxPtr->device);
						else
#endif //ALLOW_NATIVE_LIGHTBURN
							sprintf(_rptStr, ">RG:%d :", inboxPtr->device);
						getDeviceRegistrationString(inboxPtr, _tmpStr);
						strcat(_rptStr, _tmpStr);
						sendstringCr(_rptStr);
						if (inboxPtr->canbusFormat == CANBUS_FORMAT_V0) //LEGACY
						{
							sprintf(_errorStr, "Pre-4.x device (%d) registered. Commands to group addresses (0, 10, etc) may not function", inboxPtr->device);
							sendError(_errorStr);
						}
						tryToCleanUpGuiAfterDeviceRegistration(inboxPtr->device);
						break;
					default:
						break;
					}
					break;
				case DEVICE_INFO_TO_HOST :
					sendInboxInfoToHost(canRx->device, (deviceInfoType)payload->u16[0]);
					break;

				case PAGE_DATA_TO_HOST :
					// continuation of M741 ....
					// at this point, an original copy of the page is in the workingBuffer and the page checksum has been read from the device,
					workingBufferChecksum = getWorkingBufferChecksum(canRx->device);

					if (_MailBoxes._pageChecksum != workingBufferChecksum)
					{
						sprintf(_errorStr,"canProcessRxQueue - PAGE_DATA_TO_HOST - page checksum does not match (%d) (%d - %d)", canRx->device, (int)_MailBoxes._pageChecksum, (int)workingBufferChecksum);
						sendError(_errorStr);
						break;
					}
#ifdef GB_SOAP_READ_PIN
					if ((tableInfoType)payload->u16[0] == SEND_DEVICE_SOAP_STRING)
					{
						pinClear(GB_SOAP_READ_PIN);
					}
#endif

					transferDevicePageDataFromWorkingBufferToHost(canRx->device, (tableInfoType)payload->u16[0]);
					break;

				case PAGE_DATA_TO_DEVICE :
					// continuation of M742 ....
					// at this point, an original copy of the page is in the workingBuffer and the page checksum has been read from the device,
					// so now update the portion to be changed based on data from the host and send out new page to device. Transmit queue full
					// after sending, read back checksum from the written page and compare to what was sent.
					workingBufferChecksum = getWorkingBufferChecksum(canRx->device);
					if (_MailBoxes._pageChecksum != workingBufferChecksum)
					{
						sprintf(_errorStr,"canProcessRxQueue - PAGE_DATA_TO_DEVICE - page checksum does not match (%d) (%d - %d)", canRx->device, (int)_MailBoxes._pageChecksum, (int)workingBufferChecksum);
						sendError(_errorStr);
						break;
					}
					// potential partial update (overwrite part or all of the workingBuffer with new data)active bootloader
					// payload->u16[0] holds the selected data field
					transferDevicePageDataFromHostToWorkingBuffer(canRx->device, (tableInfoType)payload->u16[0]);
					// send out updated workingBuffer to device.  payload->u16[1] holds the page to update; will issue a checksum read
					transferDevicePageDataFromWorkingBufferToDevice(canRx->device, payload->u16[1]);  //will also erase affected page
					sendLoopbackMessageToDevice(canRx->device, PAGE_DATA_COMMITTED, 0,0,0,0);
					break;

				case PAGE_DATA_COMMITTED:
					// continuation of M742->PAGE_DATA_TO_DEVICE .. verifying checksum of page data transfer
					workingBufferChecksum = getWorkingBufferChecksum(canRx->device);
					if (_MailBoxes._pageChecksum != workingBufferChecksum)
					{
						sprintf(_errorStr,"canProcessRxQueue - PAGE_DATA_COMMITTED - page checksum does not match (%d) (%d - %d)", canRx->device, (int)_MailBoxes._pageChecksum, (int)workingBufferChecksum);
						sendError(_errorStr);
						break;
					}
					break;

				case STARTING_BOOTLOADER :
					// at this point, all of the registration info for the bootloader is available
					_gs._bl.password = getDevicePassword(_gs._bl.device);
					break;
				case FINISH_OPTION_BYTES :
					//FUNCTIONALITY REMOVED
					break;
				case CLEARING_PAGES_PART1 : // part1 is not called via loopback
				case CLEARING_PAGES_PART2 :
				case CLEARING_PAGES_PART3 :
				case CLEARING_PAGES_PART4 :
					clearDeviceCodePages((loopbackType)canRx->page);
					break;
					//case DEVICE_CODE_CHECKSUM_PART1 : // part1 is not called via loopback
				case DEVICE_CODE_CHECKSUM_PART2 :
				case DEVICE_CODE_CHECKSUM_PART3 :
				case DEVICE_CODE_CHECKSUM_PART4 :
				case DEVICE_CODE_CHECKSUM_PART5 :
				case DEVICE_CODE_CHECKSUM_PART6 :
					setDeviceCodeChecksum((loopbackType)canRx->page);
					break;
					//case TRANSFER_PAGE_PART1 : // part1 is not called via loopback
				case TRANSFER_PAGE_PART2 :
				case TRANSFER_PAGE_PART3 :
				case TRANSFER_PAGE_PART4 :
				case TRANSFER_PAGE_PART5 :
				case TRANSFER_PAGE_PART6 :
					transferBootloaderDataFromWorkingBufferToDevice((loopbackType)canRx->page, payload->u16[2], payload->u16[3]);
					break;
				case GUI_CMD_PROCESSED :
					_MailBoxes._waitingFor.flags.bit.canGuiAck = FALSE;
					break;
				default :
					sprintf(_errorStr, "canProcessRxQueue - unknown LOOPBACK marker: %02x", canRx->msgId);
					sendError(_errorStr);
					break;
				}
				break;

			case CAN_MSG_STRING :
				if (canReceiveString(canRx, &_canStringChars, _canString))
				{   // true if last packet
					_canStringRate++;
					if (_canStringRate == _LaserDebugReportingRate)
					{
						sprintf(_rptStr, "%d)%s", canRx->device, "HEX:");
						for (i=0; i<_canStringChars; i++)
						{
							sprintf(_tmpStr, "%cx%02x", (i==0)?' ':'.', (int)_canString[i]);
							strcat(_rptStr, _tmpStr);
						}
						sendGB(_rptStr);
						sprintf(_rptStr, "%d)%s", canRx->device, "DEC:");
						for (i=0; i<_canStringChars; i++)
						{
							sprintf(_tmpStr, "%c%03d", (i==0)?' ':'.', (int)_canString[i]);
							strcat(_rptStr, _tmpStr);
						}
						sendGB(_rptStr);
						sprintf(_rptStr, "%d)%s", canRx->device, "ASC:");
						for (i=0; i<_canStringChars; i++)
						{
							char tmpChr;
							if ((int)_canString[i] < 0x20)
								tmpChr = ' ';
							else if ((int)_canString[i] > 0x7e)
								tmpChr = ' ';
							else
								tmpChr = _canString[i];
							sprintf(_tmpStr, "%c %c ", (i==0)?' ':'.', tmpChr);
							strcat(_rptStr, _tmpStr);
						}
						sendGB(_rptStr);
						sendGB("");
						_canStringRate = 0;
					}
					_canStringChars = 0;
				}
				break;
			default:
				sprintf(_errorStr, "canProcessRxQueue - unknown CAN_WRITE msgId: %02x", canRx->msgId);
				sendError(_errorStr);
				break;
		}
		break;

		case CAN_READ:          // currently no commands in which the hothead reads info from the host
			sprintf(_errorStr, "canProcessRxQueue - unknown CAN_READ msgId: %02x", canRx->msgId);
			sendError(_errorStr);
			break;
		case CAN_WRITE_ACK:     // currently not using write ack's
			break;

		case CAN_RETURN_DATA:   // hothead or hotbed return data that the system requested
			switch (canRx->msgId)
			{
			case CAN_MSG_DEVICE_INFO :              // return of device info (sw and hw rev, etc
				unpackDeviceInfoPayload(inboxPtr, canRx);
				break;
			case CAN_MSG_FLASH_CONFIG :             // return of flash config information (total bytes, page size, start address)
				inboxPtr->flashNumKBytes    = payload->u16[0];
				inboxPtr->flashPageSize     = payload->u16[1];
				inboxPtr->flashBaseAddr     = payload->u32[1];
				break;
			case CAN_MSG_UNIQUE_ID :                // return of half device's 96-bit unique id
				for (i=0; i<6; i++)
				{
					inboxPtr->uniqueId[canRx->page*canRx->numBytes + i] = payload->u8[i];
				}
				break;
			case CAN_MSG_PRE_DEFINED_ALIASES :      // return of part of device's alias list
				for (i=0; i<NUM_PRE_DEFINED_ALIASES; i++)
				{
					inboxPtr->preDefinedAliases[i] = payload->u8[i];
				}
				break;
			case CAN_MSG_USER_DEFINED_ALIASES :     // return of part of device's alias list
				for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
				{
					inboxPtr->userDefinedAliases[i] = payload->u8[i];
				}
				updateCloneList();
				break;
			case CAN_MSG_STATUS :                   // return of requested status page
				//FUNCTIONALITY REMOVED receiveDeviceStatusPage(inboxPtr, canRx);
				break;
			case CAN_MSG_PAGE_DEF :
				if (inboxPtr->softwareMinorVersion >= 3)
				{
					inboxPtr->soapPage = payload->u16[0];
				}
				else
				{
					for (i=0; i<4; i++)
					{
						inboxPtr->pageDef[i] = payload->u16[i]; // pages numbers for tables, soap, historyA, historyB
					}
				}
				break;
			case CAN_MSG_PAGE_CHECKSUM :
				_MailBoxes._pageChecksum = payload->u32[0];         // used for general table writing
				inboxPtr->pageChecksum = payload->u32[0];       // used for specific query
				inboxPtr->checksumedPage = canRx->page;         // used for specific query
				if ((_gs._bl.started) && (canRx->device == _gs._bl.device))
				{
					_gs._bl.pageReceivedChecksum[canRx->page] = payload->u32[0];
				}
				break;
			case CAN_MSG_CONTROL_WORD :
				inboxPtr->controlWord.u32   = payload->u32[0];
				break;
				// NUKE ??? (might need for bltouch, etc....)
//            case CAN_MSG_TOUCH_PROBE :
//              //float usec = payload->i32[0]/8.0f/2.0f;
//              sprintf(_tmpStr, "canProbeREAD: CNT=%d (%3.2fusec = %3.2fmm)",
//                      (int)payload->i32[0], (payload->i32[0]/8.0f/2.0f), (payload->i32[0]/8.0f/2.0f)*0.343f);
//              sendGB(_tmpStr);
				break;
			case CAN_MSG_ACCESS_BUFFER :
				for (i=0; i<canRx->numBytes; i++)
				{
					_MailBoxes._workingBuffer[canRx->page * canRx->numBytes + i] = payload->u8[i];
				}
				break;
#ifdef HYDRA_DIAGS
			case CAN_MSG_DIAG_IO_READ: //               ((byte)0x52)    // 1 byte;  page 0xff
				_diagsConnectivityReadVal = payload->u16[0];
				break;
			case CAN_MSG_DIAG_ADC_READ: //              ((byte)0x53)    // 2 bytes; page 0xff
				_diagsAdcReadValue = payload->u16[0];
				break;
			case CAN_MSG_DIAG_STEP_COUNTER: //          ((byte)0x54)    // 8 bytes; page 0
				_diagsStepsDetected  = payload->u16[0];
				_diagsStepMinTimeUs  = (payload->u16[1] == 0xffff) ? 0.0f  : (float)payload->u16[1] * US_PER_TICK;
				_diagsStepMaxTimeUs  = (payload->u16[2] == 0)      ? 99.99f:  (float)payload->u16[2] * US_PER_TICK;
				if (_diagsStepsDetected == 0)
					_diagsStepsAvgTimeUs = 0.0f;
				else if (payload->u16[3] == 0xffff)
					_diagsStepsAvgTimeUs = 99.99f; // overflow
				else
					_diagsStepsAvgTimeUs = ((float)payload->u16[3] * US_PER_TICK)  / (float)_diagsStepsDetected;

				break;
#endif
			default :
				sprintf(_errorStr, "canProcessRxQueue - unknown CAN_READ_RETURN msgId: %02x", canRx->msgId);
				sendError(_errorStr);
				break;
			}
			break;
			default:
				sprintf(_errorStr, "canProcessRxQueue - unknown msgType %02x", canRx->msgType);
				sendError(_errorStr);
				return (ERROR_SWITCH_MSG_TYPE);
				break;
	}
	return (CAN_RX_OK);
}

////////////////////////////////////////////////////////////////////////////////

void canProcessRxQueueNoReturn(void)
{
	canProcessRxQueue();
}

////////////////////////////////////////////////////////////////////////////////
