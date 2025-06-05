#pragma once
#include "main.h"
#define MAX_CHARS_FOR_PARAMETER 20

extern int NextCommandIndex;
extern int NextCommandInsertionIndex;

void ohNoMrBill(void);
void spare(void);

void serialProcessor(void);
void CommandProcessor(void);
void SequenceEngine(void);
void readInputs(void);
void checkMotorFaultSensor(void);
void checkMotorLimit1Sensor(void);
void checkMotorLimit2Sensor(void);
void checkBlockingWaits(void);
void checkForCompletedAbort(void);
boolean cmdQueIsFull(void);
void ContinueToNextStep(void);
void checkInWithHeads(void);
void ResetGcodeParseBuffer();
void SplitCurrentGcodeLine2Arguments(buffer_t useBuffer);
void processArgs(char *WorkBuffer, float *OutPutVariable);
void processEndofCommandChar(char *currentCommandString);
void CleanUpPointers(void);
void FillArgumentBuffer(char *, char);
void CopyToEndOfLine(buffer_t useBuffer);
void printSerialInputStreamError(void);
void HssControl(int TicksPerSecond);
void DwellTimer(void);
void heartbeat(void);
void DDLightSelection(void);
void EchoBackCommentString(void);
void processNextDeferredCommand(void);
void updateHostConnectionWatchDog(void);
int getNextCommandIndex(int index);
int getNextDeferredCommandIndex(int index);
void checkStartButton(void);

void loop_1000Hz_simple_work(void);
void loop_100Hz_simple_work(void);
void loop_10Hz_simple_work(void);
void loop_1Hz_simple_work(void);
