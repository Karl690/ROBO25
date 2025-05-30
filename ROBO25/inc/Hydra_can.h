#ifndef Hydra_can_HEADER // prevent double dipping
#define Hydra_can_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    Hydra_can.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains Hydra_can specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "mailbox.h" // relies on outboxStruct definition

////////////////////////////////////////////////////////////////////////////////
//  Hydra_can specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

#define PULSE_TRAIN_ON      TRUE
#define PULSE_TRAIN_OFF     FALSE
#define INVERT_SIGNAL_ON    TRUE
#define INVERT_SIGNAL_OFF   FALSE
#define IMMEDIATE_MSG       TRUE
#define BUFFERED_MSG        FALSE

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in Hydra_can that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////
extern payloadUnion _loopbackPayload;
////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in Hydra_can
////////////////////////////////////////////////////////////////////////////////

extern void sendSetRegisterBit(byte);
extern void sendSetSwResetBit(byte);
extern void sendSetGlobalSyncBit(byte);
extern void sendSetStopExtruderBit(byte);
extern void sendSetPauseExtruderBit(byte);
extern void sendSetStopOnErrorBit(byte);
extern void sendSetSaveHistoryBit(byte);
extern void sendSetClearErrorBit(byte);
extern void sendSetAckOnWriteBit(byte);
extern void sendSetInvertDirectionBit(byte);
extern void sendSetEnableMotorBit(byte);
extern void sendSetDisableMotorBit(byte);
//extern void sendSetIgnoreRtd1Bit(byte);
//extern void sendSetIgnoreRtd2Bit(byte);
extern void sendSetDebugRtdBit(byte);
extern void sendSetBlockFashWritesBit(byte);
extern void sendSetPrimeBit(byte);
extern void sendSetUnprimeBit(byte);
extern void sendSetGateBit(byte);
extern void sendSetRetractBit(byte);
extern void sendSetCommPingBit(byte);
extern void sendSetStepFromCanbusModeBit(byte);
extern void sendSetIgnoreWatchdogBit(byte);
extern void sendSetSendDebugStringsBit(byte);
extern void sendInitializeBit(byte);

extern void sendClrRegisterBit(byte);
extern void sendClrSwResetBit(byte);
extern void sendClrGlobalSyncBit(byte);
extern void sendClrStopExtruderBit(byte);
extern void sendClrPauseExtruderBit(byte);
extern void sendClrStopOnErrorBit(byte);
extern void sendClrSaveHistoryBit(byte);
extern void sendClrClearErrorBit(byte);
extern void sendClrAckOnWriteBit(byte);
extern void sendClrInvertDirectionBit(byte);
extern void sendClrEnableMotorBit(byte);
extern void sendClrDisableMotorBit(byte);
//extern void sendClrIgnoreRtd1Bit(byte);
//extern void sendClrIgnoreRtd2Bit(byte);
extern void sendClrDebugRtdBit(byte);
extern void sendClrBlockFashWritesBit(byte);
extern void sendClrPrimeBit(byte);
extern void sendClrUnprimeBit(byte);
extern void sendClrGateBit(byte);
extern void sendClrRetractBit(byte);
extern void sendClrStepFromCanbusModeBit(byte);
extern void sendClrIgnoreWatchdogBit(byte);
extern void sendClrSendDebugStringsBit(byte);
extern void sendProbeControlToDevice(byte, probeType_t, probeAction_t, int, int, int, HH_IoIndex_t);

extern void canIssueReadRequest(byte, byte, byte);

extern void readDevicePositionFromDevice(byte);
extern void readDeviceInfoFromDevice(byte);
extern void readFlashConfigFromDevice(byte);
extern void readUniqueIdFromDevice(byte);
extern void readAliasListFromDevice(byte);
extern void readHistoryRangeDefinitionFromDevice(byte);
extern void readHistoryRangesFromDevice(byte);
extern void readHistoryCountersFromDevice(byte);
extern void readPageDefinitionFromDevice(byte);
extern void readPageChecksumFromDevice(byte, byte);
extern void readOptionBytesFromDevice(byte device);
extern void readControlWordFromDevice(byte);
extern float scaleLaserVectorPowerPct(float);
extern void sendExtrusionOverridePctToDevice(outboxStruct *, float);
extern void sendSwitchControlOffToDevice(outboxStruct *, byte);
extern void sendSwitchControlOnToDevice(outboxStruct *, byte);
extern void sendSwitchControlByDutyCycleToDevice(outboxStruct *outboxPtr, byte, byte);
extern void sendSwitchControlByGeneralPwmToDevice(outboxStruct *outboxPtr, byte, float, uint16_t);
extern void sendSwitchControlByTempToDevice(outboxStruct *, byte, uint16_t, float);
extern void sendAutoStatusControlToDevice(outboxStruct *, byte);

extern void canWriteSwitchFlag_onOnlyWhenExtruding(outboxStruct *, int, int);
//extern void sendCooldownTimeToDevice(outboxStruct *, uint16_t);
//extern void sendMotorCurrentBoostToDevice(outboxStruct *, boostCtrl_t);
//extern void sendMotorMaxRateToDevice(outboxStruct *, uint16_t);
//extern void sendRtdTypeToDevice(outboxStruct *, soapRtd_t);
//extern void sendMotorMicroStepsToDevice(outboxStruct *, uint16_t);
//extern void sendInkjetNozzleSelectToDevice(outboxStruct *, uint16_t);
extern void sendAddAliasToDevice(outboxStruct *, byte);
extern void sendRemoveAliasToDevice(outboxStruct *, byte);
extern void sendPnPHomingControlToDevice(outboxStruct *);
extern void sendLedControlToDevice(outboxStruct *);
extern void sendLedOverrideToDevice(outboxStruct *);
extern void sendKarlFactorsToDevice(outboxStruct *, byte, int, int, int, int);
extern void sendColdExtrudeTempRangesToDevice(outboxStruct *, int, temp_t, temp_t);
extern void canSendDeviceInitValue1x16(byte, devInitArea_t, devInitIndex_t, uint16_t);
extern void canSendDeviceInitValue2x16(byte, devInitArea_t, devInitIndex_t, uint16_t, uint16_t);
extern void canSendDeviceInitValue1x32(byte, devInitArea_t, devInitIndex_t, uint32_t);
extern void canSendDeviceInitValue2x32(byte, devInitArea_t, devInitIndex_t, uint32_t, uint32_t);
extern void canSendOutboxInitValue1x16(outboxStruct *, devInitArea_t, devInitIndex_t, uint16_t);
extern void canSendOutboxInitValue2x16(outboxStruct *, devInitArea_t, devInitIndex_t, uint16_t, uint16_t);
extern void canSendOutboxInitValue1x32(outboxStruct *, devInitArea_t, devInitIndex_t, uint32_t);
extern void canSendOutboxInitValue2x32(outboxStruct *, devInitArea_t, devInitIndex_t, uint32_t, uint32_t);
extern void sendErrorReportingIntervalToDevice(outboxStruct *);
extern void sendLoopbackMessageToDevice(byte, loopbackType, uint16_t, uint16_t, uint16_t, uint16_t);

extern void canFillDeviceBuffer(byte, uint32_t);
extern void canCopyPageToDeviceBuffer(byte, byte);
extern void canReadDeviceBuffer(byte);
extern void canWriteDeviceBuffer(byte, byte, payloadUnion *);
extern void canCopyDeviceBufferToPage(byte, byte, uint32_t);
extern void sendOptionBytesToDevice(outboxStruct *);
extern void sendStartPrimaryProgramToDevice(byte);
extern void sendMotorResetToDevice(byte device);
extern void sendMotorSaveConfigToDevice(byte device);

extern void PrimeThenRun(outboxStruct *);
extern void Run(outboxStruct *);
extern void Prime(outboxStruct *);//send Prime only, do not chain to normal extrude
extern int getCurrentUnPrimeTimeMs(outboxStruct *);
extern int getCurrentPrimeTimeMs(outboxStruct *);
extern void UnPrime(outboxStruct *);//execute UnPrime steps @ unprime Rate, then fire event at end of unprime timer count down
extern void StartExtruding(outboxStruct *);//Run Extruder , if steps are already set to 0xffff then run continous until told to stop
//void Prime_ThenRun(void);//execute Prime and then continue in normal extrude continous mode
extern void StopExtruding(outboxStruct *);//stop extruder, by makingthe step count 0
extern void StopAllExtruders(void);
extern void getStatusPage(inboxStruct *, canSwStruct *);
extern void printDeviceErrorMessage(canSwStruct *);
extern void RunExtrusionManual(outboxStruct *);
extern void Send_Prime_Prams(outboxStruct *);
extern void Send_UnPrime_Prams(outboxStruct *);

void SetupExtruderToRun(outboxStruct *, uint32_t, uint32_t);
void SetupLaserToRun(outboxStruct *, uint32_t, float, uint32_t, float, boolean);

extern uint16_t convertFloatPowerPctToUnsignedint(float, int);
extern void SetLaserPower(float, uint32_t );
extern void SetHeadType(outboxStruct *, uint16_t,uint16_t,uint16_t,uint16_t );
extern void sendSpindleControl(outboxStruct *outboxPtr, uint16_t, uint16_t);
extern void updateManualExtrudingOffForGUI(byte device);
extern void DisableAllExtruderMotors(void);
extern void EnableAllExtruderMotors(void);
extern void SetLaserPowerOnly(float);

extern uint32_t getDevicePassword(byte);

void unpackDeviceInfoPayload(inboxStruct *, canSwStruct *);

extern byte canProcessRxQueue(void);
extern void canProcessRxQueueNoReturn(void);

#endif // #ifndef Hydra_can_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
