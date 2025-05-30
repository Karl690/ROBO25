////////////////////////////////////////////////////////////////////////////////
//
// File:    Motor.c
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

#include <stdio.h>
#include "main.h"
#include "mailbox.h"
#include "MotorDriver.h"
#include "gpio.h"
#include "Serial.h"
#include "Hydra_can.h"
#include "GCode.h"
#include "HardwareInit.h"
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
//  Public Global Definitions (expose in MotorDriver.h)
//
////////////////////////////////////////////////////////////////////////////////

boolean _IncrementalMove  = FALSE;   //FALSE means absolute positioning, TRUE means incremental moves
boolean _IncrementalEMove = FALSE;   //FALSE means absolute positioning, TRUE means incremental moves

float _TargetFeedRateInMmPerSec = 5.0f;     // last ARG_F converted to mm/sec
float _CurrentRequestedFeedrateInMmPerSec = 5.0f; // based on the current move being executed (target ARG_F but passed through motionQ)
float _ActualVectorRateInMmPerSec = 0.0;    // based on distance of move divided by moveTime (avg speed of move);



MachineLimitsStruct _MachineLimits;

jogZStruct _jogZ = {0, 0, 84000000.0f, TIM6, 0, NULL, (JOG_DEFAULT_VALUE_IN_MICRONS * MICRONS_TO_MM)};
joggingStruct _jogging = {0, FALSE, FALSE, FALSE, 0, 3000, DIRECTION_FORWARD, M_X, 0, 0, 0, NULL};
ABjoggingStruct _ABjogging = {0, 0, 0, 0.0f, JOG_OFF_MODE_GUI };


#ifdef CCMRAM_USED_FOR_DATA

MotorStructure *DominantAxisPtr  __attribute__ ((section (".ccmram")));
int FirstAxisMotor  __attribute__ ((section (".ccmram")));  // used as the starting index for for loops to explore available motors
int LastAxisMotor  __attribute__ ((section (".ccmram")));   // used as the ending index for for loops to explore available motors
MotorStructure *FirstAxisMotorPtr  __attribute__ ((section (".ccmram")));  // update if FirstAxisMotor is changed!
MotorStructure *LastAxisMotorPtr  __attribute__ ((section (".ccmram")));   // update if LastAxisMotor is changed!
MotorStructure *_movingAxesPtrs[MAX_NUMBER_OF_MOTORS]  __attribute__ ((section (".ccmram")));
DominantStruct _Dominant  __attribute__ ((section (".ccmram")));
int _numMovingAxes  __attribute__ ((section (".ccmram")));

#else //!CCMRAM_USED_FOR_DATA

MotorStructure *DominantAxisPtr;
int FirstAxisMotor;  // used as the starting index for for loops to explore available motors
int LastAxisMotor;   // used as the ending index for for loops to explore available motors
MotorStructure *FirstAxisMotorPtr;  // update if FirstAxisMotor is changed!
MotorStructure *LastAxisMotorPtr;   // update if LastAxisMotor is changed!
MotorStructure *_movingAxesPtrs[MAX_NUMBER_OF_MOTORS];
DominantStruct _Dominant;
int _numMovingAxes;

#endif //!CCMRAM_USED_FOR_DATA

float _LastExtrusionRate = INVALID_ARG_VALUE;

////////////////////////////////////////////////////////////////////////////////
//
//  Local Global Definitions (do not expose in MotorDriver.h)
//
////////////////////////////////////////////////////////////////////////////////

#define AxisIsMoving(axisPtr) (axisPtr->PULSES_TO_GO)

////////////////////////////////////////////////////////////////////////////////
//
//  Forward Declarations
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char getAxisLabel(int axisIndex)
{
	switch (axisIndex)
	{
	case 0: return('X');
	case 1: return('Y');
	case 2: return('Z');
	case 3: return('A');
	case 4: return('B');
	case 5: return('C');
	default: return('?');
	}
}

////////////////////////////////////////////////////////////////////////////////

float ClampToAxisLimitsInUPS(MotorStructure *M, float RateInUPS)
{   // make sure the rate is legal relative to the axis's limits
	// if input value is out of range, the value returned is closest machine limit
	// otherwise, the original input value is returned
	// no warning or error is generated for out of range values

	return(fFitWithinRange(RateInUPS, M->RatesInUPS[AXIS_MIN], M->RatesInUPS[AXIS_MAX]));
}

////////////////////////////////////////////////////////////////////////////////

float ClampAndWarnToAxisLimitsInUPS(MotorStructure *M, float MotionRateInUnits)
{
	//make sure we are not over or under the max/min pulse rate for this axis/ motor or machine
	// if input value is out of range, the value returned is closest machine limit
	// otherwise, the original input value is returned
	// an error message is generated for out of range values
	if (MotionRateInUnits > M->RatesInUPS[AXIS_MAX])
	{   // we are going too FAST for this axis, so throttle back to the axis limit and report an error
		if (_errors.sent.flags.machineLimitsMaxRateInUPS == FALSE)
		{
			_errors.sent.flags.machineLimitsMaxRateInUPS = TRUE;
			sprintf(_errorStr, "SetTimerCounters - requested rate overruns axis limit (%c/%3.2f/%3.2f)", (int)M->AxisLabel,
					MotionRateInUnits, M->RatesInUPS[AXIS_MAX]);
			sendError(_errorStr);
		}
		MotionRateInUnits = M->RatesInUPS[AXIS_MAX];
	}
	if (MotionRateInUnits < M->RatesInUPS[AXIS_MIN])
	{   // we are going too SLOW for this axis, so throttle up to the axis limit and report an error
		if (_errors.sent.flags.machineLimitsMinRateInUPS == FALSE)
		{
			_errors.sent.flags.machineLimitsMinRateInUPS = TRUE;
#ifdef GB_DEBUG_MOTION_Q
			sprintf(_errorStr, "SetTimerCounters - requested rate underruns axis limit (%c/%3.2f/%3.2f/%d)", (int)M->AxisLabel,
					MotionRateInUnits, M->RatesInUPS[AXIS_MIN], M->line);
#else
			sprintf(_errorStr, "SetTimerCounters - requested rate underruns axis limit (%c/%3.2f/%3.2f)", (int)M->AxisLabel,
					MotionRateInUnits, M->RatesInUPS[AXIS_MIN]);
#endif
			sendError(_errorStr);
		}
		MotionRateInUnits = M->RatesInUPS[AXIS_MIN];
	}
	return(MotionRateInUnits);
}

////////////////////////////////////////////////////////////////////////////////

#define AXIS_MIN_RATE_IN_UPS 0.001f
#define AXIS_MAX_RATE_IN_UPS 1000.0f    //1 meter a second!

void UpdateAxisLimitAndRates(MotorStructure *M)
{   // run through all of the rates of the axis and make sure the axis min/max rates are with the machine limits
	// and then make sure all of the operational rates (rapid, homing, etc) are within those axis min/max raste limits
	M->RatesInUPS[AXIS_MIN] = fmaxf(M->RatesInUPS[AXIS_MIN], AXIS_MIN_RATE_IN_UPS);
	M->RatesInUPS[AXIS_MAX] = fminf(M->RatesInUPS[AXIS_MAX], AXIS_MAX_RATE_IN_UPS);
	M->RatesInUPS[NO_RAMP] = ClampToAxisLimitsInUPS(M, M->RatesInUPS[NO_RAMP]);
	M->RatesInUPS[RAPID]  = ClampToAxisLimitsInUPS(M, M->RatesInUPS[RAPID]);
	M->RatesInUPS[HOMING] = ClampToAxisLimitsInUPS(M, M->RatesInUPS[HOMING]);
	M->RatesInUPS[REHOMING] = ClampToAxisLimitsInUPS(M, M->RatesInUPS[REHOMING]);
	M->RatesInUPS[JOG_NO_RAMP] = ClampToAxisLimitsInUPS(M, M->RatesInUPS[JOG_NO_RAMP]);
	M->RatesInUPS[JOGGING] = ClampToAxisLimitsInUPS(M, M->RatesInUPS[JOGGING]);
	M->PulsesPerRevolution = (int)roundf(M->PulsesPerUnit * 360.0f);    // for rotary axes
}

////////////////////////////////////////////////////////////////////////////////

void UpdateAxisTravelLimits(MotorStructure *M)
{   //assumes  M->MaximumTravelInUnits, M->PulsesPerUnit and M->AbortTravelScaling are already set
	M->MaximumTravelInPulses = (int)roundf(M->MaximumTravelInUnits * M->PulsesPerUnit);

	float halfAbortRange = (M->MaximumTravelInUnits * M->AbortTravelScaling) / 2.0f; // half scaled range ... extend from center  of original range
	M->MinimumAbortTravelInUnits = (M->MaximumTravelInUnits / 2.0f) - halfAbortRange;
	M->MaximumAbortTravelInUnits = (M->MaximumTravelInUnits / 2.0f) + halfAbortRange;
	M->MinimumAbortTravelInPulses = (int)roundf(M->MinimumAbortTravelInUnits * M->PulsesPerUnit);
	M->MaximumAbortTravelInPulses = (int)roundf(M->MaximumAbortTravelInUnits * M->PulsesPerUnit);
}

////////////////////////////////////////////////////////////////////////////////

float ClampAndWarnToAxisRangeInUnits(boolean *clamped, MotorStructure *M, float OffsetDestinationInUnits, boolean homing)
{
	*clamped = FALSE;
	if (!homing && ((M->AxisType == LINEAR) || ((M->AxisType == ROTARY) && (M->MaximumTravelInUnits != 0.0f))))
	{
		if (M->AbortTravelScaling != 0.0f)
		{
			if (OffsetDestinationInUnits > M->MaximumAbortTravelInUnits)
			{   //if you get here, you have a big problem because we are trying to drive the motors past their software abort limit
				OffsetDestinationInUnits = M->MaximumTravelInUnits; // limit travel to the software limit
				// reverse calc a requested position to get to Max
				M->Q_LastRequestedPositionInUnits = OffsetDestinationInUnits - SumOfAllMotorOffsets(M);
				sprintf(_errorStr, "Travel Requested past Max Axis Range*Scale (%c/%fmm/%fmm)", M->AxisLabel, OffsetDestinationInUnits, M->MaximumAbortTravelInUnits);
				sendMotionError('M', M->AxisLabel, OffsetDestinationInUnits, M->MaximumTravelInUnits, _errorStr);
				catastrophicErrorWithDebugInfo(_errorStr);
				*clamped = TRUE;
			}
			else if (OffsetDestinationInUnits < M->MinimumAbortTravelInUnits)
			{   //when traveling into the negative, which is not possible we need to stop at zero
				OffsetDestinationInUnits = 0.0f;    //do not move past home
				// reverse calc a requested position to get to 0
				M->Q_LastRequestedPositionInUnits = 0.0f - SumOfAllMotorOffsets(M);
				sprintf(_errorStr, "Travel Requested past Min Axis Range*Scale (%c/%fmm/%fmm)", M->AxisLabel, OffsetDestinationInUnits, M->MinimumAbortTravelInUnits);
				sendMotionError('M', M->AxisLabel, OffsetDestinationInUnits, 0.0f, _errorStr);
				catastrophicErrorWithDebugInfo(_errorStr);
				*clamped = TRUE;
			}
		}
		if (OffsetDestinationInUnits > M->MaximumTravelInUnits)
		{   //trying to drive the motors past their software limit
			sprintf(_errorStr, "Travel Requested past Max Axis Range (%c/%fmm/%fmm)", M->AxisLabel, OffsetDestinationInUnits, M->MaximumTravelInUnits);
			sendMotionError('M', M->AxisLabel, OffsetDestinationInUnits, M->MaximumTravelInUnits, _errorStr);
			OffsetDestinationInUnits = M->MaximumTravelInUnits; // limit travel to the software limit
			// reverse calc a requested position to get to Max
			M->Q_LastRequestedPositionInUnits = M->MaximumTravelInUnits - SumOfAllMotorOffsets(M);
			*clamped = TRUE;
		}
		else if (OffsetDestinationInUnits < 0.0f)
		{   //when traveling into the negative, which is not possible we need to stop at zero
			sprintf(_errorStr, "Travel Requested past Min Axis Range (%c/%fmm/%fmm)", M->AxisLabel, OffsetDestinationInUnits, 0.0f);
			sendMotionError('M', M->AxisLabel, OffsetDestinationInUnits, 0.0f, _errorStr);
			OffsetDestinationInUnits = 0.0f;    //do not move past home
			// reverse calc a requested position to get to 0
			M->Q_LastRequestedPositionInUnits = 0.0f - SumOfAllMotorOffsets(M);
			*clamped = TRUE;
		}
	}
	return(OffsetDestinationInUnits);
}

////////////////////////////////////////////////////////////////////////////////

int ClampAndWarnToAxisRangeInPulses(boolean *clamped, MotorStructure *M, int OffsetDestinationInPulses, boolean homing)
{
	*clamped = FALSE;
	if (!homing && ((M->AxisType == LINEAR) || ((M->AxisType == ROTARY) && (M->MaximumTravelInUnits != 0.0f))))
	{
		if (M->AbortTravelScaling != 0.0f)
		{   // check abort limits
			if (OffsetDestinationInPulses > M->MaximumAbortTravelInPulses)
			{   //if you get here, you have a big problem because we are trying to drive the motors past their software abort limit
				OffsetDestinationInPulses = M->MaximumTravelInPulses;   // limit travel to the software limit
				// reverse calc a requested position to get to Max
				M->Q_LastRequestedPositionInUnits = M->MaximumTravelInUnits - SumOfAllMotorOffsets(M);
				sprintf(_errorStr, "Travel Requested past Max Axis Travel Limit*Scale (%c/%f/%f)", M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, M->MaximumAbortTravelInPulses*M->UnitsPerPulse);
				sendMotionError('M', M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, M->MaximumTravelInPulses*M->UnitsPerPulse, _errorStr);
				catastrophicErrorWithDebugInfo(_errorStr);
				*clamped = TRUE;
			}
			else if (OffsetDestinationInPulses < M->MinimumAbortTravelInPulses)
			{   //when traveling into the negative, which is not possible we need to stop at zero
				OffsetDestinationInPulses = 0.0f;   //do not move past home
				// reverse calc a requested position to get to 0
				M->Q_LastRequestedPositionInUnits = 0.0f - SumOfAllMotorOffsets(M);
				sprintf(_errorStr, "Travel Requested past Min Axis Travel Limit*Scale (%c/%f/%f)", M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, M->MinimumAbortTravelInPulses*M->UnitsPerPulse);
				sendMotionError('M', M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, 0.0f, _errorStr);
				catastrophicErrorWithDebugInfo(_errorStr);
				*clamped = TRUE;
			}
		}
		if (OffsetDestinationInPulses > M->MaximumTravelInPulses)
		{   //trying to drive the motors past their software limit
			sprintf(_errorStr, "Travel Requested past Max Axis Travel Limit (%c/%f/%f)", M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, M->MaximumTravelInPulses*M->UnitsPerPulse);
			sendMotionError('M', M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, M->MaximumTravelInPulses*M->UnitsPerPulse, _errorStr);
			OffsetDestinationInPulses = M->MaximumTravelInPulses;   // limit travel to the software limit
			// reverse calc a requested position to get to Max
			M->Q_LastRequestedPositionInUnits = M->MaximumTravelInUnits - SumOfAllMotorOffsets(M);
			*clamped = TRUE;
		}
		else if (OffsetDestinationInPulses < 0)
		{   //when traveling into the negative, which is not possible we need to stop at zero
			sprintf(_errorStr, "Travel Requested past Min Axis Travel Limit (%c/%f/%f)", M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, 0.0f);
			sendMotionError('M', M->AxisLabel, OffsetDestinationInPulses*M->UnitsPerPulse, 0.0f, _errorStr);
			OffsetDestinationInPulses = 0;   //do not move past home
			// reverse calc a requested position to get to 0
			M->Q_LastRequestedPositionInUnits = 0.0f - SumOfAllMotorOffsets(M);
			*clamped = TRUE;
		}
	}
	return(OffsetDestinationInPulses);
}

////////////////////////////////////////////////////////////////////////////////

boolean AllMotionMotorsAreEnabled(void)
{   // Visit each motor structure and report TRUE is all installed are enabled
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		if (M->MotorInstalled)
		{
			if (M->canMotor)
			{	// motor(s) are on the can, so need to look for state elsewhere
				for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
				{
					if (M->canMotors[canAddrIndex].canAddress > 0)
					{
						if (getOutboxPointer(M->canMotors[canAddrIndex].canAddress)->motorState == MOTOR_DISABLED_STATE)
						{
							return(FALSE);
						}
					}
				}
			}
			else
			{
				if (readControlBitState(&M->Enable) == DEASSERTED)
				{
					return(FALSE);
				}
			}
		}
	}
	return(TRUE);
}
////////////////////////////////////////////////////////////////////////////////

void EnableAllMotionMotors(void)
{   // Visit each motor structure and turn "on" the enable pin for that motor
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		if (M->MotorInstalled)
		{
			if (M->canMotor)
			{	// motor(s) are on the can
				for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
				{
					if (M->canMotors[canAddrIndex].canAddress > 0)
					{
						sendSetEnableMotorBit(M->canMotors[canAddrIndex].canAddress);
					}
				}
			}
			else
			{
				assertControlBit(&M->Enable);
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void DisableAllMotionMotors(void)
{   // Visit each motor structure and turn "off" the enable pin for that motor
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		if (M->MotorInstalled)
		{
			if (M->canMotor)
			{	// motor(s) are on the can
				for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
				{
					if (M->canMotors[canAddrIndex].canAddress > 0)
					{
						sendSetDisableMotorBit(M->canMotors[canAddrIndex].canAddress);
					}
				}
			}
			else
			{
				deassertControlBit(&M->Enable);
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean NoAxisIsMoving(void)
{   // Visit each motor involved in the move and if any axis is moving, return false.  true if NONE are moving
	int i;
	for (i=0; i<_numMovingAxes; i++)
	{   // only axes that were part of the move will be in the list
		if (_movingAxesPtrs[i]->PULSES_TO_GO)
		{
			return(FALSE);
		}
	}
	// no axis is moving
#ifdef GB_MOVING_PIN
	pinClear(GB_MOVING_PIN); // signal no motion to logic analyzer
#endif
#ifdef GB_ACCEL_DECEL_PIN
	GB_ACCEL_DECEL_CLEAR; // should be no motion
#endif
	return(TRUE);
}

////////////////////////////////////////////////////////////////////////////////
#ifdef NEW_G38
boolean AnyPotentialMotion(void)
{// return true if any axis is still moving OR anything in the motionQ or waiting to execute a move
	if ((motionQ_getCountdownDelayToExecuteMove() == 0) && motionQ_empty() && NoAxisIsMoving())
		return(FALSE);
	else
		return(TRUE);
}
#endif //NEW_G38
////////////////////////////////////////////////////////////////////////////////

void ClearAllMotorG92Offset(void)
{   // Visit each motor structure and set the G92Offset to 0
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		M->G92Offset = 0.0f;
	}
}

////////////////////////////////////////////////////////////////////////////////

void ClearMotorFixtureOffsets(MotorStructure *M)
{   // Set all of the FixtureOffset in the motor structure to 0
	int i;
	for (i=0; i<NUM_FIXTURE_OFFSETS; i++)
	{
		M->FixtureOffsets[i] = 0.0f;
	}
}

////////////////////////////////////////////////////////////////////////////////

void ClearAllMotorFixtureOffsets(void)
{   // Visit each motor structure and set all of the FixtureOffsets to 0
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		ClearMotorFixtureOffsets(M);
	}
}

////////////////////////////////////////////////////////////////////////////////

void ClearMotorHeadOffsets(MotorStructure *M)
{   // Set all of the HeadOffset in the motor structure to 0
	int i;
	for (i=0; i<NUM_HEAD_OFFSETS; i++)
	{
		M->HeadOffsets[i] = 0.0f;
	}
}

////////////////////////////////////////////////////////////////////////////////

void ClearAllMotorHeadOffsets(void)
{   // Visit each motor structure and set all of the HeadOffsets to 0
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		ClearMotorHeadOffsets(M);
	}
}

////////////////////////////////////////////////////////////////////////////////

float SumOfAllMotorOffsets(MotorStructure *M)
{   // add up all the various offsets that impact target position.  Z is a special case
	// as an additional (potential) offset is involved (ToolLength)
	if (M->Axis == M_Z)
		return(M->G92Offset + M->FixtureOffsets[currentFixtureIndex] + M->HeadOffsets[currentHeadIndex] + ToolOffsets[currentToolIndex].ToolLength);
	else
		return(M->G92Offset + M->FixtureOffsets[currentFixtureIndex] + M->HeadOffsets[currentHeadIndex]);
}

////////////////////////////////////////////////////////////////////////////////

void ResetAllMotionOffsetIndices(void)
{
	ClearAllMotorG92Offset();
	currentFixtureIndex = 0;
	currentHeadIndex = 0;
	currentToolIndex = 0;
}

////////////////////////////////////////////////////////////////////////////////

void ClearAllMotorFlags(void)
{   // Visit each motor structure and set the controls "off" indicating no motion
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		if (M->MotorInstalled)
		{
			M->SearchingForHomeSensor = FALSE;
			M->PulsePending = FALSE;
			M->PULSES_IN_MOVE = 0;
			M->PULSES_TO_GO = 0;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void DisableMotionTimer(TIM_TypeDef *TIM)
{
	TIM->CR1 &= (uint16_t)~TIM_CR1_CEN;         //TIM_Cmd(TIM, DISABLE);
}
////////////////////////////////////////////////////////////////////////////////


void DisableMotionTimers(void)
{   // Visit each motor structure and disable that motors timer. Do the same for. Do the same for
	// the master timer.
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // do ALL whether the were moving or not (except for lathe motor if enabled
		if (M->latheMode == LATHE_MODE_OFF)
		{
			M->TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;        //TIM_Cmd(TIM, DISABLE);
		}
	}
	_Dominant.TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;     //TIM_Cmd(TIM, DISABLE);
}

////////////////////////////////////////////////////////////////////////////////

void loadDominantTimer(void)
{   // load the dominant motor's timer's auto-reload register and prescaler for pulse generation
	_Dominant.TimerBase->PSC = _Dominant.TimerPSC;
	_Dominant.TimerBase->ARR = _Dominant.TimerARR; // load the auto reload register (ARR is offset by 1)
	_Dominant.TimerBase->EGR |= TIM_EGR_UG;     // force the CNT and PSC to reset
	_Dominant.TimerBase->SR = (uint16_t)~TIM_FLAG_Update; // clear the interrupt from the update
}

////////////////////////////////////////////////////////////////////////////////

void loadMotorTimer(MotorStructure *M)
{   // load the motor's timer's auto-reload register and prescaler for pulse generation
	M->TimerBase->PSC = M->TimerPSC;
	M->TimerBase->ARR = M->TimerARR;    // load the auto reload register (ARR is offset by 1)
	M->TimerBase->EGR |= TIM_EGR_UG;    // force the CNT and PSC to reset
	M->TimerBase->SR = (uint16_t)~TIM_FLAG_Update;    // clear the interrupt from the update
}

////////////////////////////////////////////////////////////////////////////////

void EnableMotionTimers(void)
{   // Visit each motor structure and enable that motors timer
	if (motionQ.oldest->flags.homingMove)
	{   // homing move - each axis is independent
		MotorStructure *M;
		int i;
		for (i=0; i<_numMovingAxes; i++)
		{
			M = _movingAxesPtrs[i];
			loadMotorTimer(M);
			M->TimerBase->CR1 |= TIM_CR1_CEN;   //TIM_Cmd(M->TimerBase, ENABLE);
		}
	}
	else
	{   // not homing -- only use one master timer
		if (DominantAxisPtr->PULSES_TO_GO)
		{   // make sure there is a move to perform
			loadDominantTimer();    // setup the dominant (controls in _Dominant)

			// setup the timer for the axis tied to dominant such that it will create a simple one shot
			// with minimal time to reload
			DominantAxisPtr->TimerBase->PSC = 0;
			DominantAxisPtr->TimerBase->ARR = 1;

			_Dominant.TimerBase->CR1 |= TIM_CR1_CEN;    //TIM_Cmd(_Dominant.TimerBase, ENABLE);

			// arm immediate timer to fire to call UpdateNextAccelerationFactor/CalculateTimerControls in separate ISR
			// PSC/ARR was preset before move started
			_Tim7StillCalculating = TRUE;
			TIM7->PSC = 0;
			TIM7->ARR = 1;
			TIM7->EGR |= TIM_EGR_UG; // force the CNT and PSC to reset
			TIM7->CR1 |= TIM_CR1_CEN;  //TIM_Cmd(TimerBase, ENABLE);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void PrepareForNextMove(void)
{   // make sure timers are off, flags are cleared
	DisableMotionTimers();      // shut off timers in case were starting to move before prior move finished
	ClearAllMotorFlags();       // shut off any motion control flag (ie "move" and "homing") to have a fresh start
}

////////////////////////////////////////////////////////////////////////////////

void StartMove(void)
{   // set any necessary flags and start timers to start moving
#ifdef GB_FUNC_START_MOVE_PIN
	GB_FUNC_START_MOVE_SET;
#endif //GB_FUNC_START_MOVE_PIN
#ifdef GB_MOVING_PIN
	pinSet(GB_MOVING_PIN); // signal start of motion to logic analyzer
#endif

	motionQ.oldest->flags.inMotion = TRUE;   // show that the move has started
	EnableMotionTimers();   // load counters and enable timer

#ifdef GB_FUNC_START_MOVE_PIN
	GB_FUNC_START_MOVE_CLEAR;
#endif //GB_FUNC_START_MOVE_PIN
}

////////////////////////////////////////////////////////////////////////////////

void setWatchdogBasedOnMoveTime(float moveTime)
{
	int totalMarginedMoveTimeMs = (int)((moveTime * 1000.0f + getCurrentPrimeTimeMs(currentOutboxPtr)) * 1.25f);     // 1.25 to add margin
	totalMarginedMoveTimeMs = imax(totalMarginedMoveTimeMs, 1000);   // min of 1 second for WD
	totalMarginedMoveTimeMs = imin(totalMarginedMoveTimeMs, MAX_MOTION_WATCHDOG_MS);

	_gs._totalMarginedMoveTimeMs = totalMarginedMoveTimeMs;
	//if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_LASER) && LASER_ENABLED)
	if ((currentOutboxPtr->device == 41) && LASER_ENABLED)//check for co2 laser address
	{
		_gs._laser.watchdogMs = totalMarginedMoveTimeMs;
		Co2LaserWatchDogTimer = _gs._laser.watchdogMs;
		TIM8->CCR3 = DesiredCo2LaserPower;//tun on pwm for laser
	}
	else if (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_INKJET)
		_gs._laser.watchdogMs = totalMarginedMoveTimeMs;
	else
		_gs._laser.watchdogMs = 0;
}

////////////////////////////////////////////////////////////////////////////////

float getMotionRateInUPS(motionEntryStruct *curr)
{
	if (curr->flags.rasterMove)
		return(_gs._laser.rasterScanFeedrate);
	else if (_motionForFlowControl == MOTION_FLOW_USE_FEEDRATE)
		return(_CurrentRequestedFeedrateInMmPerSec);
	else if (_motionForFlowControl == MOTION_FLOW_USE_AVG_SPEED)
		return(_ActualVectorRateInMmPerSec);
	else if (_motionForFlowControl == MOTION_FLOW_USE_CRUISE_SPEED)
		return(curr->speed);
	else
		return(_CurrentRequestedFeedrateInMmPerSec);
}

////////////////////////////////////////////////////////////////////////////////

void changeDominantAxisToC(motionEntryStruct *curr)
{
	MotorStructure *CAxisPtr = &Motors[M_C];
	float pulseRatio = (float)CAxisPtr->PULSES_IN_MOVE / (float)DominantAxisPtr->PULSES_IN_MOVE;
	CAxisPtr->PulsesPerUnit = DominantAxisPtr->PulsesPerUnit * pulseRatio;
	CAxisPtr->UnitsPerPulse = 1.0f / CAxisPtr->PulsesPerUnit;
	CAxisPtr->AccelerationConstant = DominantAxisPtr->AccelerationConstant;
	CAxisPtr->AbortDecelerationRate = DominantAxisPtr->AbortDecelerationRate;
	CAxisPtr->RatesInUPS[NO_RAMP] = DominantAxisPtr->RatesInUPS[NO_RAMP];
	CAxisPtr->RatesInUPS[AXIS_MIN] = DominantAxisPtr->RatesInUPS[AXIS_MIN];
	CAxisPtr->RatesInUPS[AXIS_MAX] = DominantAxisPtr->RatesInUPS[AXIS_MAX];
	curr->unitVector[M_C] = curr->unitVector[curr->flags.dominantAxis];
	DominantAxisPtr = CAxisPtr; // make C dominant
	//DO NOT CHANGE flags.dominantAxis SO WE STILL HAVE REFERENCE TO REAL DOM AXIS -- curr->flags.dominantAxis = M_C;
}

////////////////////////////////////////////////////////////////////////////////

void CheckForNewFlowRate(motionEntryStruct *curr)
{   // executes with motionQ_execute to have late binding scaling for additional fudge factors and then send (if needed) data to head
	// called from  motionQ_addCommand when raw input data is available (ARG_E, ARG_S)
	// do as much work as possible now as their is less time crunch (rather than in motionQ_execute)
	//
	// three primary cases
	//      1) flow via head's timer w/ implied E       (_extrusionControl == IGNORE_E_VALUES) || !(_canbusStepForE || _directStepForE)
	//      2) flow via motorC timer w/ implied E       (_extrusionControl == IGNORE_E_VALUES) ||  (_canbusStepForE || _directStepForE)   -- _hijackAxisC
	//      3) flowvia  motorC timer w/ actual E        (_extrusionControl == USE_E_VALUES)    ||  (_canbusStepForE || _directStepForE)   -- _hijackAxisC
	//
	// at motionQ_addCommand time, all head parameters should be in sync, so can precalc as much as possible that does not require
	// the actual speed of the move or any last second scale factors
	//
	//  extrusionCrossSection - calculated W*H  (will multiply by motionRate to get flowRate) for case 1a
	//  extrusionVolume - calculated L*W*H for case 2a
	//  extrusionPulses - scaled amount (number of pulses to create reqested volume
	//  RATES will be calculated at execute time when actual speed is known.
#ifdef GB_MQ_CHK_FLOWRATE_PIN
	GB_MQ_CHK_FLOWRATE_SET;
#endif //GB_MQ_CHK_FLOWRATE_PIN

	setWatchdogBasedOnMoveTime(curr->moveTime);

	if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_HEATED_EXTRUDER) || (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_UNHEATED_EXTRUDER))
	{
		deviceExtrusionControlStruct *extPtr = &currentOutboxPtr->ExtrusionControl; // local pointer

		if ((curr->extrusionControl == IGNORE_E_VALUES) && FLOW_RATE_CREATED_ON_HEAD)
		{   // derive flow from volumetric calc (L * W * H) *** the L portion will be handled when motion rate is known

			float extrusionRate = getMotionRateInUPS(curr) * curr->flowCrossSectionTimesPPU;    //getMotionRateInUPS is pre-calc to save time in this real-time routine

			if (almostTheSameByPct((float)extPtr->ExtrudeFeedRate, extrusionRate, 0.05))
			{   // rate has changed for this head
				extPtr->ExtrudeFeedRate = imin((uint16_t)extrusionRate, 0xffff);    // save new rate
				extPtr->ExtrudeSteps = HH_CONTINUOUS_EXTRUSION_STEPS;
				_LastExtrusionRate = extrusionRate;      // remember last rate -- used for live reporting to repetrel

				if (curr->flags.needToPrimeBeforeMove)
					SetupExtruderToRun(currentOutboxPtr, HEAD_FUNCTION_PAGE_FLAG_PRIME_RUN, _gs._totalMarginedMoveTimeMs);
				else
					SetupExtruderToRun(currentOutboxPtr, HEAD_FUNCTION_PAGE_FLAG_RUN, _gs._totalMarginedMoveTimeMs);

				if (_sendingGBStringsMask & GB_STRING_FLOW) // use M797 S<mask> to enable
				{
					sprintf(_tmpStr,"N:%ld mr=%3.1f fr=%3.1f h=%3.2f w=%3.2f ppu=%3.2f ff=%3.2ff",
							getCurrentExecutingLineNumber(), getMotionRateInUPS(curr), extrusionRate, currentOutboxPtr->ExtrusionControl.SliceHeightInMm,
							currentOutboxPtr->ExtrusionControl.ExtrusionWidthInMm, currentOutboxPtr->ExtrusionControl.PulsesPerUnit, curr->flowOverridePct);
					sendGB(_tmpStr);
				}
			}
			else
			{   // no new rate
				if (curr->flags.needToPrimeBeforeMove)
				{
					PrimeThenRun(currentOutboxPtr);
				}
			}
			if (currentOutboxPtr->numClones)
			{   // has at least one clone
				int clone;
				outboxStruct *cloneOutboxPtr;
				for (clone=0; clone<currentOutboxPtr->numClones; clone++)
				{
					cloneOutboxPtr = getOutboxPointer(currentOutboxPtr->clones[clone]);
					// if it's decide to allow lasers and uvatas,
					// swap lines
					//if (cloneOutboxPtr->deviceFamily == DEVICE_FAMILY_LASER)
					if (deviceIsAUvLightRay(cloneOutboxPtr->device))
					{
						SetupLaserToRun(cloneOutboxPtr, HEAD_FUNCTION_PAGE_FLAG_RUN, _gs._laser.vectorPowerPct, _gs._totalMarginedMoveTimeMs, getMotionRateInUPS(curr), curr->flags.rasterMove);
					}
				}
			}
		}
		else if (FLOW_RATE_CREATED_ON_MOTORC)
		{
			// sending out individual step pulses or CAN_STEP packets to extruder
			MotorStructure *M = &Motors[M_C];       // hijack motor C's timer for this mode

			if (curr->flowPulses < 0)
			{   // going backwards
				M->PULSES_TO_GO = imax(0, (-1 * curr->flowPulses));
				M->DIRECTION = -1;
				_gs._preCannedPackets[PRECANNED_STEP].page = HH_MOTOR_REVERSE;
			}
			else
			{
				M->PULSES_TO_GO = imax(0, curr->flowPulses);
				M->DIRECTION = 1;       // only moving in positive direction.
				_gs._preCannedPackets[PRECANNED_STEP].page = HH_MOTOR_FORWARD;
			}

			_LastExtrusionRate = (curr->moveTime > 0.0f) ? ((float)M->PULSES_TO_GO / curr->moveTime) : 0.0f;      // remember last rate -- used for live reporting to repetrel

			_gs._preCannedPackets[PRECANNED_STEP].device = currentOutboxPtr->device;
			M->PULSES_IN_MOVE = M->PULSES_TO_GO;

			if (M->PULSES_IN_MOVE)
			{   // actually moving
				_movingAxesPtrs[_numMovingAxes++] = M;  // add M_C to the list
				if (M->PULSES_IN_MOVE > DominantAxisPtr->PULSES_IN_MOVE)
				{   // M_C will become dominant axis
					changeDominantAxisToC(curr);
				}
			}
			else if (curr->lineNumber == 65) //NUKE
			{
				M->PULSES_TO_GO = curr->flowPulses;		// BREAKPOINT ONLY
			}


			if (curr->flags.needToPrimeBeforeMove)
			{   // primes are still handled on the head in case of CAN_STEPS
				PrimeThenRun(currentOutboxPtr);
			}

			if (_sendingGBStringsMask & GB_STRING_FLOW) // use M797 S<mask> to enable
			{
				if (curr->extrusionControl == USE_E_VALUES)
				{
					sprintf(_tmpStr,"N:%ld d=%4.3f p=%ld r=%5.3f", getCurrentExecutingLineNumber(), curr->distance, curr->flowPulses, (float)curr->flowPulses / curr->moveTime);
					sendGB(_tmpStr);
				}
			}
		}
	} //if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_HEATED_EXTRUDER) || (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_UNHEATED_EXTRUDER))
	else if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_LASER) || (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_INKJET))
	{
		if (_gs._laser.watchdogMs <= 0)
		{   // shut things down!
			_gs._laser.vectorPowerPct = 0.0f;
			_gs._laser.watchdogMs = 0;
		}

		if (!curr->flags.rasterMove && (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_INKJET) && Motors[M_C].SendStepViaCanbus)
		{   // special case of spewing drop by drop from the inkject tied to the motion
			; //use M_C and PRECANNED PACKAGE

			_gs._preCannedPackets[PRECANNED_STEP].device = currentOutboxPtr->device;
			_gs._preCannedPackets[PRECANNED_STEP].page = 1;    							//# droplets per canStep or replaced with laser power

			MotorStructure *M = &Motors[M_C];       // hijack motor C's timer for this mode

			M->PULSES_TO_GO = imax(0, curr->flowPulses);
			M->PULSES_IN_MOVE = M->PULSES_TO_GO;
			M->DIRECTION = 1;

			if (M->PULSES_IN_MOVE)
			{   // actually moving
				_movingAxesPtrs[_numMovingAxes++] = M;  // add M_C to the list
				if (M->PULSES_IN_MOVE > DominantAxisPtr->PULSES_IN_MOVE)
				{   // M_C will become dominant axis
					changeDominantAxisToC(curr);
				}
#ifdef GB_HIDDEN_WARNINGS
#warning "ONLY SEND IF RATE CHANGE"
#endif
				SetupLaserToRun(currentOutboxPtr,
						(curr->flags.needToPrimeBeforeMove ? HEAD_FUNCTION_PAGE_FLAG_PRIME_RUN : HEAD_FUNCTION_PAGE_FLAG_RUN),
						scaleLaserVectorPowerPct(curr->flowOverridePct),
						_gs._laser.watchdogMs,
						getMotionRateInUPS(curr),
						curr->flags.rasterMove);
			}
		}
		else
		{
			_gs._preCannedPackets[PRECANNED_LASER].device = currentOutboxPtr->device;   // used if raster mode (ignored otherwise)
			_gs._preCannedPackets[PRECANNED_LASER].page = 1;    							//# droplets per canStep or replaced with laser power


			SetupLaserToRun(currentOutboxPtr,
					(curr->flags.needToPrimeBeforeMove ? HEAD_FUNCTION_PAGE_FLAG_PRIME_RUN : HEAD_FUNCTION_PAGE_FLAG_RUN),
					(curr->flags.rasterMove ? 0.0f : scaleLaserVectorPowerPct(curr->flowOverridePct)),
					_gs._laser.watchdogMs,
					getMotionRateInUPS(curr),
					curr->flags.rasterMove);
		}
	} //else if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_LASER) || (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_INKJET))
#ifdef GB_MQ_CHK_FLOWRATE_PIN
	GB_MQ_CHK_FLOWRATE_CLEAR;
#endif //GB_MQ_CHK_FLOWRATE_PIN
}

////////////////////////////////////////////////////////////////////////////////

float CheckForNewFeedrate(boolean vectorMotion)
{  // Handle ARG_F / Feedrate
	float feedrate;

	if (ARG_F_PRESENT)
	{   // get new motion feed rate (always check for the feedrate, even if the move might be "turbo'd" or not evening moving)
		feedrate = (convertArgToMM(ARG_F) / (float)60.0);  // intended speed goal
		if (vectorMotion)
		{   // ONLY update persistant variable if doing a vector move (instead of a one shot change on a RAPID move)
			_TargetFeedRateInMmPerSec = feedrate; // save a copy for posterity
		}
	}
	else
	{
		feedrate = _TargetFeedRateInMmPerSec;
	}
	return(fmaxf(0.0f, feedrate));
}

////////////////////////////////////////////////////////////////////////////////

void CalculateTimerControls(MotorStructure *M)
{   // calculate the non-acceleration baseline Timer ARR and PSC register values for the M->currentVelocity

#ifdef GB_FUNC_CALC_TIMER_CTRL_PIN
	GB_FUNC_CALC_TIMER_CTRL_SET;
#endif //GB_FUNC_CALC_TIMER_CTRL_PIN

	M->MotionRateInSPP = 1.0f / fmaxf(1.0f, (M->currentVelocity * M->PulsesPerUnit));   // result used elsewhere

	// TimerFreq is the input clock source to the motor timer
	// AccelerationEndLimit is the SMALLEST divider for the timer's logical PSC register
	// so (TimerFreq / AccelerationEndLimit) -- is the fastest clock going to the timer's prescale counter
	// calculate the clock divisor to get the desired pulses per second

	if (motionQ.oldest->flags.homingMove)
	{
		uint32_t quotient = ((uint32_t)(M->TimerFreq * M->MotionRateInSPP)) + 1; // +1 to cover any possible roundoff
		M->TimerPSC = quotient >> 16;    // upper 16-bits; value of 0 means divide by 1) (add 0.999999 to do ceiling func to ensure ARR fits)
		M->TimerARR = (quotient / (M->TimerPSC + 1)) - 1;  // PSC+1 because TimerPSC value of 0 means divide by 1;

		if (_sendingGBStringsMask & GB_STRING_RATE) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"RatePPS=%f PSC=%d ARR=%d", 1.0f / M->MotionRateInSPP, (int)M->TimerPSC, (int)M->TimerARR);
			sendGB(_tmpStr);
		}
	}
	else
	{   // not homing, so need to store timer info in a non motor structure
#define TIM1_LOST_TIME 0.0000007f // 0.7 usec  (measured with LA)
		// TIM1_LOST_TIME -- the time between each pulse of the master timer is pre-calculated and the results loaded once tim1 isr fires.  there is lost time
		// from time time the interrupt occurs until the timer is reloaded/restarted to count down to the next pulse.  the time gap, if not accounted for, would
		// basically decrease the pulse rate relative to what was excpected
		uint32_t quotient = ((uint32_t)(_Dominant.TimerFreq * (M->MotionRateInSPP - TIM1_LOST_TIME))) + 1; // +1 to cover any possible roundoff
		_Dominant.TimerPSC = quotient >> 16;     // upper 16-bits; value of 0 means divide by 1) (add 0.999999 to do ceiling func to ensure ARR fits)
		_Dominant.TimerARR = (quotient / (_Dominant.TimerPSC + 1)) - 1; // PSC+1 because TimerPSC value of 0 means divide by 1;

		if (_sendingGBStringsMask & GB_STRING_RATE) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"RatePPS=%f PSC=%d ARR=%d", 1.0f / M->MotionRateInSPP, (int)_Dominant.TimerPSC, (int)_Dominant.TimerARR);
			sendGB(_tmpStr);
		}
	}

#ifdef GB_FUNC_CALC_TIMER_CTRL_PIN
	GB_FUNC_CALC_TIMER_CTRL_CLEAR;
#endif //GB_FUNC_CALC_TIMER_CTRL_PIN
}

////////////////////////////////////////////////////////////////////////////////

void loadAndRunLatheTimer(void)
{
	if (_gs.latheMotor != NULL)
	{
		MotorStructure *M = _gs.latheMotor;
		if (fabsf(M->latheCurrSpeedPPS) > 0.0f)
		{
			uint32_t quotient = ((uint32_t)(M->TimerFreq / fabsf(M->latheCurrSpeedPPS))) + 1; // +1 to cover any possible roundoff
			M->TimerPSC = quotient >> 16;    // upper 16-bits; value of 0 means divide by 1) (add 0.999999 to do ceiling func to ensure ARR fits)
			M->TimerARR = (quotient / (M->TimerPSC + 1)) - 1;  // PSC+1 because TimerPSC value of 0 means divide by 1;
			uint32_t irq_disabled = interruptsOff();
			M->TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;    // Disable
			loadMotorTimer(M);
			M->TimerBase->CCR2 = 1; // force a single pulse out each pass
			M->TimerBase->CR1|= TIM_CR1_CEN;        // Enable
			interruptsOn(irq_disabled);
		}
		else
		{
			M->TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;    //TIM_Cmd(M->TimerBase, DISABLE);
		}

		if (_sendingGBStringsMask & GB_STRING_RATE) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"RatePPS=%d PSC=%d ARR=%d", (int)M->latheCurrSpeedPPS, (int)M->TimerPSC, (int)M->TimerARR);
			sendGB(_tmpStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void setupLatheModeToRun(MotorStructure *M, float targetRatePPS, direction_t direction)
{
	M->HasBeenHomed = M->HomeSense.Enabled ? FALSE : TRUE;  // force rehome if axis has a sensor
	M->HomeSense.ErrorMessageSent = FALSE;

	M->POSITION = 0;        // clear for safety (other routines may key off these)
	M->PULSES_IN_MOVE = 0;  // clear for safety (other routines may key off these)
	M->PULSES_TO_GO = 0;    // clear for safety (other routines may key off these)

	assertControlBit(&M->Enable);   // make sure motor is enabled
	M->latheMode = LATHE_MODE_ON;
	M->latheAccelPer10ms = (M->AccelerationConstant / 100.0f) * M->PulsesPerUnit;   // update speed every 10ms

	M->latheTargetSpeedPPS = fFitWithinRange(targetRatePPS, 0.0f, (M->RatesInUPS[AXIS_MAX] * M->PulsesPerUnit)); // limit range
	if ((direction == DIRECTION_REVERSE) && (M->latheTargetSpeedPPS != 0.0f))
	{
		M->latheTargetSpeedPPS *= -1.0f;    //reverse speeds stored as negative numbers
	}

	_g4DwellTimer = ((fabsf(M->latheTargetSpeedPPS - M->latheCurrSpeedPPS) / M->latheAccelPer10ms) + 1.0f) * 10.0f;	// wait for motor to reach speed
}

////////////////////////////////////////////////////////////////////////////////

passFail_t ExecuteSingleAxisMove(MotorStructure *AM, float DestinationInUnits, float motionRateInUPS)
{   // this routine will do the complete setup to move a single axis (with accel/decel) the
	// requested destination at the request rate

	setAllMotorArg(INVALID_ARG_VALUE);
	setMotorArgInNativeUnits(AM, DestinationInUnits);
	return(motionQ_addCommand(motionRateInUPS, NOT_HOMING_MOTORS));
}

////////////////////////////////////////////////////////////////////////////////

passFail_t ExecuteHomingMove(float motionRate)
{   // move all selected axis to their home position
	// each axis will move independently until it hit's it's home sensor and the will decelerate
	// a pre-assigned distance until arriving at the actual home.

	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then home that axis
		if (motorArgPresent(M))
		{   // if this motor has a valid ARG, then send that axis home -- set up the counters/times/rates for this axis
			// each axis will move independently until it hit's it's home sensor and then will decelerate
			// a pre-assigned distance until arriving at the actual home.

			if (M->HomingDirection == HOME_TOWARD_ZERO)
			{
				M->POSITION = MAXINT;   //M->MaximumTravelInPulses
				M->Q_POSITION = M->POSITION;
				setMotorArgInNativeUnits(M, 0.0f);
			}
			else if (M->HomingDirection == HOME_AWAY_FROM_ZERO)
			{
				M->POSITION = 0;
				M->Q_POSITION = M->POSITION;
				setMotorArgInNativeUnits(M, MAXFLOAT);
			}
		}
	}
	return(motionQ_addCommand(motionRate, HOMING_MOTORS));
}

////////////////////////////////////////////////////////////////////////////////

void JogMotor(MotorStructure *M, float JogInUnits)
{   // move selected motor the input amount but have the machines understanding
	// of where that motor is remain unchanged.  will require a little white lie
	// be told to the motors to initial change their position.
	//
	// two methods are used.
	//    (1) if the Q is empty, then this can be treated as a normal move.
	//    (2) if the Q is not empty, it must have been a backdoor request, in which case
	//        an alternate timer will be used to spew out the extra pulses to move the
	//        requested AXIS by the job amount.
	//           - If the selected AXIS is moving from a normal move then the Jog is ignored
	//           - If already jogging that AXIS, just add to the pulse count
	//           - If already moving a diff axis, punt

	int JogInPulses = (int)(JogInUnits * M->PulsesPerUnit);

	if (motionQ_empty())
	{
		M->POSITION += -JogInPulses;
		M->Q_POSITION = M->POSITION; // keep queue in sync;

		boolean save_IncrementalMove = _IncrementalMove;
		_IncrementalMove = FALSE;
		ExecuteSingleAxisMove(M, M->Q_LastRequestedPositionInUnits, RAPID_MOTION_RATE);
		_IncrementalMove = save_IncrementalMove;
	}
	else
	{   // motionQ_notEmpty()
		if (M->PULSES_TO_GO == 0)
		{   // not moving, so go ahead with the jog
			uint32_t irq_disabled = interruptsOff();
			if (_jogZ.pulsesToGo != 0)
			{   // already moving so just add on to the current move and adjust direction if needed
				if (_jogZ.M == M) {
					_jogZ.pulsesToGo += JogInPulses;
					if (_jogZ.pulsesToGo >= 1)
						outputDirectionBit(&M->Direction, DIRECTION_FORWARD);
					else if (_jogZ.pulsesToGo <= 1)
						outputDirectionBit(&M->Direction, DIRECTION_REVERSE);
				}
				interruptsOn(irq_disabled);
			}
			else
			{   // need to start a jog move -- set up jogging timer
				interruptsOn(irq_disabled);

				_jogZ.M = M;
				_jogZ.pulsesToGo = JogInPulses;
				if (_jogZ.pulsesToGo >= 1)
					outputDirectionBit(&M->Direction, DIRECTION_FORWARD);
				else if (_jogZ.pulsesToGo <= 1)
					outputDirectionBit(&M->Direction, DIRECTION_REVERSE);

				uint32_t quotient = ((uint32_t)(M->TimerFreq / (M->RatesInUPS[NO_RAMP] * M->PulsesPerUnit))) + 1; // +1 to cover any possible roundoff
				_jogZ.TimerPSC = quotient >> 16;    // upper 16-bits; value of 0 means divide by 1) (add 0.999999 to do ceiling func to ensure ARR fits)
				_jogZ.TimerARR = (quotient / (M->TimerPSC + 1)) - 1;  // PSC+1 because TimerPSC value of 0 means divide by 1;
				_jogZ.TimerBase->PSC = _jogZ.TimerPSC;
				_jogZ.TimerBase->ARR = _jogZ.TimerARR; // load the auto reload register (ARR is offset by 1)
				_jogZ.TimerBase->EGR |= TIM_EGR_UG;     // force the CNT and PSC to reset
				_jogZ.TimerBase->SR = (uint16_t)~TIM_FLAG_Update;    // clear the interrupt from the update
				_jogZ.TimerBase->CR1 |= TIM_CR1_CEN;  //TIM_Cmd(_jogZ.TimerBase, ENABLE);
				_jogZ.TimerBase->EGR |= TIM_EGR_UG; // force an immediate trigger to get the ball rolling!
			}
		}
		else
		{   // axis moving, so can't do anything
			;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void killMotorJogging(void)
{
	TIM_Cmd(_jogZ.TimerBase, DISABLE);
	_jogZ.pulsesToGo = 0;
	_jogZ.M = NULL;
}

////////////////////////////////////////////////////////////////////////////////

float calcVelocity(MotorStructure *M, int X)
{   // high school physics re-lived!  (X is displacement, V is velocity)
	// 2a(X-Xo) = V^2 - Vo^2
	// Xo = 0;
	// V = Vcruise
	// Vo = sqrt(V^2 - 2aX)
	// clamp to 0 prevent velo going negative (and sqrt can't handle negative number)

	return(fpu_sqrtf(fmaxf(0.0f, M->cruiseVelocitySquared - M->AccelerationScaledTimes2 * ((float)X * M->UnitsPerPulse))));
}

////////////////////////////////////////////////////////////////////////////////

boolean UpdateNextAccelerationFactor(MotorStructure *M)
{   // return true if the AF changed
	// each move could be in one of three zones, accerating, cruising, or decelerating
	// if not cruising, need to recalculate the instaneous current velocity based on the current
	// displacement (position) and alert the calling function that updates have been made

	// THIS FUNCTION IS CALLED IN ADVANCE OF STEP PULSE and is use to pre-calc for the NEXT interval (pretend PULSES_TO_GO is
	// already incremented
#ifdef GB_FUNC_UPDATE_NEXT_ACCEL_PIN
	GB_FUNC_UPDATE_NEXT_ACCEL_SET;
#endif //GB_FUNC_UPDATE_NEXT_ACCEL_PIN

#define PULSES_MOVED_SO_FAR (M->PULSES_IN_MOVE - nextPulseToGo)
	boolean update = TRUE;

	int nextPulseToGo = M->PULSES_TO_GO - 1;    // future value after next step
	if (nextPulseToGo > 0)
	{
		if (M->AccelerationPulses >= PULSES_MOVED_SO_FAR)
		{   // accelerating - result must be between startV and cruiseV; and within the legal jump amount (safety)
			M->currentVelocity = fFitWithinRange(calcVelocity(M, M->AccelerationPulses - PULSES_MOVED_SO_FAR), M->startVelocity, M->cruiseVelocity);
#ifdef GB_ACCEL_DECEL_PIN
			GB_ACCEL_DECEL_SET;
#endif //GB_ACCEL_DECEL_PIN
		}
		else if (M->DecelerationPulses >= nextPulseToGo)
		{   // decelerating  -- make sure new accel rate is lower than or equal to current velocity (critical for jumping into a decel routine like aborting)
			M->currentVelocity = fFitWithinRange(calcVelocity(M, M->DecelerationPulses - nextPulseToGo), M->endVelocity, M->currentVelocity);
#ifdef GB_ACCEL_DECEL_PIN
			GB_ACCEL_DECEL_SET;
#endif //GB_ACCEL_DECEL_PIN
		}
		else
		{   // cruising
			if (M->currentVelocity == M->cruiseVelocity)
			{
				update = FALSE;
			}
			else    //(M->currentVelocity != M->cruiseVelocity)
			{   // should have just finished accelerating, so safely force to be a cruise velo
				M->currentVelocity = M->cruiseVelocity;
			}
#ifdef GB_ACCEL_DECEL_PIN
			GB_ACCEL_DECEL_CLEAR;
#endif //GB_ACCEL_DECEL_PIN
		}
	}
#ifdef GB_FUNC_UPDATE_NEXT_ACCEL_PIN
	GB_FUNC_UPDATE_NEXT_ACCEL_CLEAR;
#endif //GB_FUNC_UPDATE_NEXT_ACCEL_PIN
	return(update);
}

////////////////////////////////////////////////////////////////////////////////

void latheSpeedControl(void)
{   // call from 10ms loop (100Hz)
	// curr and target speeds are kept as pulses per second.  forward speeds are positive, reverse speeds are negative
	// this will facilitate a continuous ramp down and back up for a direction changing move
	if (_gs.latheMotor != NULL)
	{   // lathe motor is known, so safe to proceed
		MotorStructure *M = _gs.latheMotor; // M is just a local copy to make the code less verbose
		if (M->latheMode != LATHE_MODE_OFF)
		{
			if (M->latheCurrSpeedPPS == M->latheTargetSpeedPPS)
			{   // at target speed -- not much to do
				if (M->latheTargetSpeedPPS == 0.0f)
				{   // no longer moving
					disableLatheMode(); // re-init timer and STEP pin for normal use and set mode OFF
				}
			}
			else
			{   // not at target speed yet
				if (M->latheCurrSpeedPPS == 0.0f)
				{   // either starting up and at the zero crossing
					outputDirectionBit(&M->Direction, (pinStateValue_t)(M->latheTargetSpeedPPS >= 0.0f ? DIRECTION_FORWARD : DIRECTION_REVERSE));
				}
				else
				{   // ramping to speed
					outputDirectionBit(&M->Direction, (pinStateValue_t)(M->latheCurrSpeedPPS >= 0.0f ? DIRECTION_FORWARD : DIRECTION_REVERSE));
				}

				if (((M->latheCurrSpeedPPS < 0.0f) ^ (M->latheTargetSpeedPPS > 0.0f)) || (M->latheCurrSpeedPPS == 0.0f))
				{   // speeds have same sign, so no direction change, just work towards target

					if (M->latheCurrSpeedPPS < M->latheTargetSpeedPPS)
						M->latheCurrSpeedPPS = fminf((M->latheCurrSpeedPPS + M->latheAccelPer10ms), M->latheTargetSpeedPPS);    //increase towards +infinity
					else
						M->latheCurrSpeedPPS = fmaxf((M->latheCurrSpeedPPS - M->latheAccelPer10ms), M->latheTargetSpeedPPS);    //decrease towards -infinity
				}
				else
				{   // speeds have different signs -- will involve a direction change, so work towards a speed of 0.0 and then flip direction and continue
					// so will be slowing down to zero (0.0f is a fake target)
					if (M->latheCurrSpeedPPS < 0.0f)
						M->latheCurrSpeedPPS = fminf((M->latheCurrSpeedPPS + M->latheAccelPer10ms), 0.0f);
					else
						M->latheCurrSpeedPPS = fmaxf((M->latheCurrSpeedPPS - M->latheAccelPer10ms), 0.0f);
				}
				loadAndRunLatheTimer();
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void finishMove(void)
{   // no pulses left to be generated, so shut down the current move and proceed

	_Dominant.TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;         //TIM_Cmd(_Dominant.TimerBase, DISABLE);
	if (_jogging.enabled)
	{
		joggingComplete();
	}
	else
	{
		motionQ.oldest->flags.inMotion = FALSE;
		if (_TouchProbeMoveActive)
		{   // were trying to do a touch move
			TouchProbeFinished(PROBE_RESULTS_MOTION_FINISHED, PROBE_TYPE_CONTACT, 0);
		}

		_ActualVectorRateInMmPerSec = 0.0f; // stopped moving so set to 0 (for ReportXYZ)
		if (motionQ.oldest->flags.pauseAfterThisMove)
		{
			setPauseGcodeFlag(PAUSE_AT_END_OF_MOVE_CHAR);
		}
		if (motionQ.oldest->flags.abortAfterThisMove)
		{
			setPauseGcodeFlag(CATASTROPHIC_ERROR_ALERT_CHAR);
		}

		if (motionQ.oldest->flags.hasDeferredCommands && motionQ.oldest->flags.lastInSequence)
		{   // since deferrred commands not processed before next move starts, can rely on cmdLink to oldest mQ entry to know
			// which deferred commands to run, to instead, before starting the next move, go ahead and tag those deferred mcodes
			// that they are ready to process.
			int currentDeferredIndex = NextDeferredCommandIndex;
			while (deferredCmdQue[currentDeferredIndex].cmdState == DEFERRED_CMD_VALID_AND_READY_TO_PROCESS)
			{   // skip over cmds already tagged to process
				currentDeferredIndex = getNextDeferredCommandIndex(currentDeferredIndex);
			}
			while ((deferredCmdQue[currentDeferredIndex].cmdState == DEFERRED_CMD_VALID_BUT_NOT_READY_TO_PROCESS) && (deferredCmdQue[currentDeferredIndex].cmd.cmdLink == motionQ.oldest->cmdQueIndex))
			{
				deferredCmdQue[currentDeferredIndex].cmdState = DEFERRED_CMD_VALID_AND_READY_TO_PROCESS;
				currentDeferredIndex = getNextDeferredCommandIndex(currentDeferredIndex);
				_needToProcessDeferredCommands = TRUE;
			}
		}
		motionQ_deleteCurrentMoveAndExecuteNextMove();
	}
}

////////////////////////////////////////////////////////////////////////////////

void ProcessMotion(MotorStructure *M)
{   // Motor step Timer ISR routines call this method each time they fire.
	uint32_t irq_disabled = interruptsOff();
#ifdef GB_FUNC_PROC_MOTION_PIN
	GB_FUNC_PROC_MOTION_SET;
#endif //GB_FUNC_PROC_MOTION_PIN

#ifdef SLICE_TIMING_MEASUREMENT
		_motionTimerCalls++;
#endif
	if (AxisIsMoving(M))
	{    // if there's work to do.....
		if (!((M->Axis == M_C) && _hijackAxisC && !_directStepForE))
		{   // not the one special case on axis C that should prevent the pulseformation
			assertControlBit(&M->Step); // start forming the step pulse
		}

		if (motionQ.oldest->flags.homingMove == FALSE)
		{   // not homing
			M->TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;    //TIM_Cmd(M->TimerBase, DISABLE); // timer is really a one shot
			M->PULSES_TO_GO--;//keep track of how many pulses left on this move
			M->PulsePending = FALSE;

			interruptsOn(irq_disabled);

			M->POSITION += M->DIRECTION;//keep track of the absolute location
			if (M->AxisType == ROTARY)
			{   // special case for infinite rotary spin ... keep view of actual position as 0 to 360 degrees.
				M->POSITION = (M->POSITION + M->PulsesPerRevolution) % M->PulsesPerRevolution;  // (will ensure POSITION never goes negative)
			}

#ifdef ALLOW_JOGGING_HEAD_MOTOR_WITH_CAN_STEP
			if (M->SendStepViaCanbus)
#else //!ALLOW_JOGGING_HEAD_MOTOR_WITH_CAN_STEP
			if (M->SendStepViaCanbus && !_jogging.enabled)
#endif //!ALLOW_JOGGING_HEAD_MOTOR_WITH_CAN_STEP
			{
				protectedIncrement(&_gs._preCannedStepPending); // flag to send the packet on the next systick
			}
		}
		else //if (motionQ.oldest->flags.homingMove == TRUE) //-- HOMING!
		{
			interruptsOn(irq_disabled);

			M->POSITION += M->DIRECTION;//keep track of the absolute location
			if (UpdateNextAccelerationFactor(M))
			{   // if the acceleration changed - update all axes with updated accel/decel and adjust timers
				CalculateTimerControls(M); // coordinated move
				loadMotorTimer(M);
			}
			M->PULSES_TO_GO--;//keep track of how many pulses left on this move

			if (M->SearchingForHomeSensor)
			{   // true while still searching for the homing sensor
#ifdef USE_CAN_MOTOR_HOME_SENSOR
				if ((M->canMotor && M->HomeSense.State == SENSOR_TRIPPED) || (!(M->canMotor) && (fastReadDebouncedSensorState(&M->HomeSense) == SENSOR_TRIPPED))) // check current state
#else //!USE_CAN_MOTOR_HOME_SENSOR
				//NUKE if (fastReadDebouncedSensorState(&M->HomeSense) == SENSOR_TRIPPED) // check current state
				if (fastReadDebouncedSensorState(&M->HomeSense) != SENSOR_OPEN) // check current state -- only proceed if the sensor is not open
#endif //USE_CAN_MOTOR_HOME_SENSOR
					{   // sensor trip.... home detected... prep for hysteresis part of the homing move.
					M->PULSES_TO_GO = M->PulsesPerUnit * M->HomeHysteresisInUnits;
					M->POSITION     = M->PULSES_TO_GO;
					M->SearchingForHomeSensor = FALSE;  // know where we are, so shut down the homing flag
				}
			}
		}

		if (!((M->Axis == M_C) && _hijackAxisC && !_directStepForE))
		{   // not the one special case on axis C that should prevent the pulseformation
			deassertControlBit(&M->Step);    // finish step pulse - set trailing edge
		}

		if (M->PULSES_TO_GO == 0)
		{   // PULSES_TO_GO is 0, so move is over an need to shut down this motors timer
			M->TimerBase->CR1 &= (uint16_t)~TIM_CR1_CEN;    //TIM_Cmd(M->TimerBase, DISABLE);  // needed for the homing case
			if (!AxisIsMoving(DominantAxisPtr))
			{   // dominant is finished....just need to check if any stragglers....
				if (NoAxisIsMoving())
				{    // if this had been the last axis moving, need to shut down the dominant axis timer
					// and determine what to do next
					finishMove();
				}
			}
		}
	}
	else { // (M->PULSES_TO_GO == 0)
		sprintf(_errorStr, "Process Motion (axis=%c dom=%c)", M->AxisLabel, DominantAxisPtr->AxisLabel);
		barf(_errorStr); // should NOT get here EVER
	}

#ifdef GB_FUNC_PROC_MOTION_PIN
	GB_FUNC_PROC_MOTION_CLEAR;
#endif //GB_FUNC_PROC_MOTION_PIN
}

////////////////////////////////////////////////////////////////////////////////

void InitializeMotorParameters(void)
{   // setup all motor related parameters
	// GB XXX gregkarl - InitialMotorParameters() - if we force used to load these at start up (and have a blocking
	// GB XXX gregkarl - InitialMotorParameters() - flag to prevent motion, then defaults do not need to be
	// GB XXX gregkarl - InitialMotorParameters() - loaded (as realistically any default we pick is going
	// GB XXX gregkarl - InitialMotorParameters() - to be be wrong for a given machine/machine type/use/user).
	MotorStructure *M;

	// zero out the entire motor structure to ensure a known starting point
	// by doing this, we DO NOT need to individually set values to a 0.  Only need to set things that
	// specifically need to be non 0.

	_MachineLimits.CentripetalAccelRadius = 0.010;    // centripetal accel radius for intersection speed limit

	bzero(Motors, (sizeof(MotorStructure) * MAX_NUMBER_OF_MOTORS));

	FirstAxisMotor = M_X;   // used as the starting index for for loops to explore available motors
	LastAxisMotor  = MAX_NUMBER_OF_MOTORS - 1;  // used as the ending index for for loops to explore available motors

	FirstAxisMotorPtr = &Motors[FirstAxisMotor];    // update if FirstAxisMotor is changed!
	LastAxisMotorPtr  = &Motors[LastAxisMotor];     // update if FirstAxisMotor is changed!

	// set up common defaults for ALL motors
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		M->Axis = M-Motors; // axis name - allows reference back to the array if only have a ptr to a motor struct
		M->AxisLabel = getAxisLabel(M->Axis);

		switch (M-Motors) { // change MotorInstalled with MCode M211
		// boolean MotorInstalled : set whether or not a given axis/motor is installed.
		case M_X : M->MotorInstalled = TRUE; break;
		case M_Y : M->MotorInstalled = TRUE; break;
		case M_Z : M->MotorInstalled = TRUE; break;
		case M_A : M->MotorInstalled = FALSE; break;
		case M_B : M->MotorInstalled = FALSE; break;
		case M_C : M->MotorInstalled = FALSE; break;
		}

		switch (M-Motors) {
		// uint32_t GCodeArgOffset: byte offset to this motor's GCodeArg in the GMCommandStructure (ie X105.67)
		case M_X : M->GCodeArgOffset = offsetof(GMCommandStructure, X); break;
		case M_Y : M->GCodeArgOffset = offsetof(GMCommandStructure, Y); break;
		case M_Z : M->GCodeArgOffset = offsetof(GMCommandStructure, Z); break;
		case M_A : M->GCodeArgOffset = offsetof(GMCommandStructure, A); break;
		case M_B : M->GCodeArgOffset = offsetof(GMCommandStructure, B); break;
		case M_C : M->GCodeArgOffset = offsetof(GMCommandStructure, C); break;
		}

		switch (M-Motors) { // change AxisType with MCode M212
		// axis_t AxisType: LINEAR or ROTARY;
		case M_X : M->AxisType = LINEAR; break;
		case M_Y : M->AxisType = LINEAR; break;
		case M_Z : M->AxisType = LINEAR; break;
		case M_A : M->AxisType = ROTARY; break;
		case M_B : M->AxisType = ROTARY; break;
		case M_C : M->AxisType = ROTARY; break;
		}

		switch (M-Motors) { // change PulsesPerUnit with MCode M92
		// float PulsesPerUnit : pulses per mm of travel - if changed, all other rates need to be updated
		case M_X : M->PulsesPerUnit = 50.0f; break;
		case M_Y : M->PulsesPerUnit = 50.0f; break;
		case M_Z : M->PulsesPerUnit = 50.0f; break;
		case M_A : M->PulsesPerUnit = 50.0f; break;
		case M_B : M->PulsesPerUnit = 50.0f; break;
		case M_C : M->PulsesPerUnit = 50.0f; break;
		}
		M->UnitsPerPulse = 1.0f / M->PulsesPerUnit;

		switch (M-Motors) { // change MaximumTravelInUnits with MCode M91
		// float MaximumTravelInUnits : software limit for each axis in travel in mm or deg
		case M_X : M->MaximumTravelInUnits = 50.0f; break;
		case M_Y : M->MaximumTravelInUnits = 50.0f; break;
		case M_Z : M->MaximumTravelInUnits = 50.0f; break;
		case M_A : M->MaximumTravelInUnits = 50.0f; break;
		case M_B : M->MaximumTravelInUnits = 50.0f; break;
		case M_C : M->MaximumTravelInUnits = 50.0f; break;
		}
		M->MaximumTravelInPulses = (int)roundf(M->MaximumTravelInUnits * M->PulsesPerUnit);
		M->AbortTravelScaling = 0.0f;

		switch (M-Motors) { // change AccelerationConstant with MCode M208
		// int AccelerationConstant : "constant" to calc initial accel relative to velocity (mm/sec/sec or deg/sec/sec)
		case M_X : M->AccelerationConstant = 10.0f; break;
		case M_Y : M->AccelerationConstant = 10.0f; break;
		case M_Z : M->AccelerationConstant = 10.0f; break;
		case M_A : M->AccelerationConstant = 10.0f; break;
		case M_B : M->AccelerationConstant = 10.0f; break;
		case M_C : M->AccelerationConstant = 10.0f; break;
		}

		switch (M-Motors) { // change AbortDecelerationRate with MCode M217
		// float AbortDecelerationRate : decel rate for panic stops
		case M_X : M->AbortDecelerationRate = 10.0f; break;
		case M_Y : M->AbortDecelerationRate = 10.0f; break;
		case M_Z : M->AbortDecelerationRate = 10.0f; break;
		case M_A : M->AbortDecelerationRate = 10.0f; break;
		case M_B : M->AbortDecelerationRate = 10.0f; break;
		case M_C : M->AbortDecelerationRate = 10.0f; break;
		}

		switch (M-Motors) { // change RatesInUPS[AXIS_MIN] with MCode M209
		// float RatesInUPS[AXIS_MIN] : machine min rate for this axis (mm/sec or deg/sec)
		case M_X : M->RatesInUPS[AXIS_MIN] = 0.01f; break;
		case M_Y : M->RatesInUPS[AXIS_MIN] = 0.01f; break;
		case M_Z : M->RatesInUPS[AXIS_MIN] = 0.01f; break;
		case M_A : M->RatesInUPS[AXIS_MIN] = 0.01f; break;
		case M_B : M->RatesInUPS[AXIS_MIN] = 0.01f; break;
		case M_C : M->RatesInUPS[AXIS_MIN] = 0.01f; break;
		}

		switch (M-Motors) { // change RatesInUPS[AXIS_MAX] with MCode M210
		// float RatesInUPS[AXIS_MAX] : machine max rate for this axis  (mm/sec or deg/sec)
		case M_X : M->RatesInUPS[AXIS_MAX] = 10.0f; break;
		case M_Y : M->RatesInUPS[AXIS_MAX] = 10.0f; break;
		case M_Z : M->RatesInUPS[AXIS_MAX] = 10.0f; break;
		case M_A : M->RatesInUPS[AXIS_MAX] = 10.0f; break;
		case M_B : M->RatesInUPS[AXIS_MAX] = 10.0f; break;
		case M_C : M->RatesInUPS[AXIS_MAX] = 10.0f; break;
		}

		switch (M-Motors) { // change RatesInUPS[NO_RAMP] with MCode M204
		// float RatesInUPS[NO_RAMP] : speed above this value requires acceleration (mm/sec or deg/sec)
		case M_X : M->RatesInUPS[NO_RAMP] = 1.0f; break;
		case M_Y : M->RatesInUPS[NO_RAMP] = 1.0f; break;
		case M_Z : M->RatesInUPS[NO_RAMP] = 1.0f; break;
		case M_A : M->RatesInUPS[NO_RAMP] = 1.0f; break;
		case M_B : M->RatesInUPS[NO_RAMP] = 1.0f; break;
		case M_C : M->RatesInUPS[NO_RAMP] = 1.0f; break;
		}

		switch (M-Motors) { // change RatesInUPS[RAPID] with MCode M203
		// float RatesInUPS[RAPID] : aka G0 travel rate for this axis (mm/sec or deg/sec)
		case M_X : M->RatesInUPS[RAPID] = 10.0f; break;
		case M_Y : M->RatesInUPS[RAPID] = 10.0f; break;
		case M_Z : M->RatesInUPS[RAPID] = 10.0f; break;
		case M_A : M->RatesInUPS[RAPID] = 10.0f; break;
		case M_B : M->RatesInUPS[RAPID] = 10.0f; break;
		case M_C : M->RatesInUPS[RAPID] = 10.0f; break;
		}

		M->RatesInUPS[JOGGING] = M->RatesInUPS[AXIS_MIN];
		M->JogAccelerationConstant = DEFAULT_MIN_ACCELERATION_CONSTANT;
		M->JogIncrement = (JOG_DEFAULT_VALUE_IN_MICRONS / 1000.0f);
		M->JogPauseTimeMs = JOG_DEFAULT_CHANGE_OF_DIRECTION_DELAY_MS;

		switch (M-Motors) { // change RatesInUPS[HOMING] with MCode M205
		// float RatesInUPS[HOMING] : aka G28 and Jog rate for this axis (mm/sec or deg/sec)
		case M_X : M->RatesInUPS[HOMING] = 1.0f; break;
		case M_Y : M->RatesInUPS[HOMING] = 1.0f; break;
		case M_Z : M->RatesInUPS[HOMING] = 1.0f; break;
		case M_A : M->RatesInUPS[HOMING] = 1.0f; break;
		case M_B : M->RatesInUPS[HOMING] = 1.0f; break;
		case M_C : M->RatesInUPS[HOMING] = 1.0f; break;
		}
		M->RatesInUPS[REHOMING] = M->RatesInUPS[HOMING];  // by default, make homing and re-homing the same speed (M223)

		switch (M-Motors) { // change HomingStartInUnits with MCode M215
		// float HomingDirection : Home to or from 0
		case M_X : M->HomingDirection = HOME_TOWARD_ZERO; break;
		case M_Y : M->HomingDirection = HOME_TOWARD_ZERO; break;
		case M_Z : M->HomingDirection = HOME_TOWARD_ZERO; break;
		case M_A : M->HomingDirection = HOME_TOWARD_ZERO; break;
		case M_B : M->HomingDirection = HOME_TOWARD_ZERO; break;
		case M_C : M->HomingDirection = HOME_TOWARD_ZERO; break;
		}

		switch (M-Motors) { // change HomeDestinationInPulses with MCode M216
		// float  HomingEndInUnits : this is motor position to assign after the homing sequence finishes
		case M_X : M->HomeDestinationInUnits = 0.0f; break;
		case M_Y : M->HomeDestinationInUnits = 0.0f; break;
		case M_Z : M->HomeDestinationInUnits = 0.0f; break;
		case M_A : M->HomeDestinationInUnits = 0.0f; break;
		case M_B : M->HomeDestinationInUnits = 0.0f; break;
		case M_C : M->HomeDestinationInUnits = 0.0f; break;
		}
		M->HomeDestinationInPulses = (int)(M->HomeDestinationInUnits * M->PulsesPerUnit);

		switch (M-Motors) { // change HomeHysteresisInUnits with MCode M206
		// float HomeHysteresisInUnits : distance from first tripping sensor to actual zero position
		case M_X : M->HomeHysteresisInUnits = 1.0f; break; // ie 2mm of travel
		case M_Y : M->HomeHysteresisInUnits = 1.0f; break;
		case M_Z : M->HomeHysteresisInUnits = 1.0f; break;
		case M_A : M->HomeHysteresisInUnits = 1.0f; break;
		case M_B : M->HomeHysteresisInUnits = 1.0f; break;
		case M_C : M->HomeHysteresisInUnits = 1.0f; break;
		}
		M->HomePopOffDistanceInUnits = 5.0 *  M->HomeHysteresisInUnits;

		M->HasBeenHomed = FALSE;
		M->HomeSense.ErrorMessageSent = FALSE;
		M->HomingFailed = FALSE;

		switch (M-Motors) { // change HomeSense.Polarity with MCode M93
		// uint32_t HomeSense.Polarity : ACTIVE_HIGH if sensor reads 1 when tripped; ACTIVE_LOW otherwise
#ifdef USE_HYREL_IO
		case M_X : M->HomeSense.Polarity = ACTIVE_HIGH; break;
#else
		case M_X : M->HomeSense.Polarity = ACTIVE_LOW; break;
#endif
		case M_Y : M->HomeSense.Polarity = ACTIVE_LOW; break;
		case M_Z : M->HomeSense.Polarity = ACTIVE_LOW; break;
		case M_A : M->HomeSense.Polarity = ACTIVE_LOW; break;
		case M_B : M->HomeSense.Polarity = ACTIVE_LOW; break;
		case M_C : M->HomeSense.Polarity = ACTIVE_LOW; break;
		}

		uint32_t tmpBit;

		switch (M-Motors) {
		// uint32_t HomeSense.Bit : port/pin/bit for home sensor pin
		case M_X : tmpBit = X_HOME; break;
		case M_Y : tmpBit = Y_HOME; break;
		case M_Z : tmpBit = Z_HOME; break;
		case M_A : tmpBit = A_HOME; break;
		case M_B : tmpBit = B_HOME; break;
		case M_C : tmpBit = C_HOME; break;
		}

		initSensor(&M->HomeSense, SENSOR_INDEX_HOME, "Home", FALSE, M->HomeSense.Polarity, tmpBit, DEFAULT_SENSOR_DEBOUNCE_READS);

#ifdef USE_HYREL_IO

		switch (M-Motors) {
		// uint32_t FaultSense.Bit : port/pin/bit for home sensor pin
		case M_X : tmpBit = X_STALL; break;
		case M_Y : tmpBit = Y_STALL; break;
		case M_Z : tmpBit = Z_STALL; break;
		case M_A : tmpBit = A_STALL; break;
		case M_B : tmpBit = B_STALL; break;
		case M_C : tmpBit = C_STALL; break;
		}
		initSensor(&M->FaultSense, SENSOR_INDEX_FAULT, "Stall", FALSE, ACTIVE_LOW, tmpBit, DEFAULT_SENSOR_DEBOUNCE_READS);

		switch (M-Motors) { // change FaultSense.Polarity with MCode M95
		// uint32_t FaultSense.Polarity : ACTIVE_HIGH if sensor reads 1 when tripped; ACTIVE_LOW otherwise
		case M_X : M->FaultSense.Polarity = ACTIVE_HIGH; break;
		case M_Y : M->FaultSense.Polarity = ACTIVE_LOW; break;
		case M_Z : M->FaultSense.Polarity = ACTIVE_HIGH; break;
		case M_A : M->FaultSense.Polarity = ACTIVE_LOW; break;
		case M_B : M->FaultSense.Polarity = ACTIVE_LOW; break;
		case M_C : M->FaultSense.Polarity = ACTIVE_LOW; break;
		}

		initSensor(&M->Limit1Sense, SENSOR_INDEX_LIMIT1, "Limit1", FALSE, ACTIVE_LOW, PIN_UNDEFINED, DEFAULT_SENSOR_DEBOUNCE_READS);
		initSensor(&M->Limit2Sense, SENSOR_INDEX_LIMIT2, "Limit2", FALSE, ACTIVE_LOW, PIN_UNDEFINED, DEFAULT_SENSOR_DEBOUNCE_READS);

		switch (M-Motors) {
		// uint32_t Enable.Bit : port/pin/bit to control the motor enable
		case M_X : M->Enable.Bit = X_EN; break;
		case M_Y : M->Enable.Bit = Y_EN; break;
		case M_Z : M->Enable.Bit = Z_EN; break;
		case M_A : M->Enable.Bit = A_EN; break;
		case M_B : M->Enable.Bit = B_EN; break;
		case M_C : M->Enable.Bit = C_EN; break;
		}
		M->Enable.gpioPort = pinExtractPortPtr(M->Enable.Bit);
		M->Enable.gpioMask = pinExtractPinMask(M->Enable.Bit);

		// change Enable.Polarity with MCode M96
		// uint32_t Enable.Polarity : flag to invert the default "sense" of the Enable.Bit
		M->Enable.Polarity = ACTIVE_LOW;

#elif defined (USE_HYDRA_IO)

		switch (M-Motors) {
		// uint32_t FaultSense.Bit : port/pin/bit for fault  sensor pin
//		case M_X : tmpBit = X_FAULT; break;
//		case M_Y : tmpBit = Y_FAULT; break;
//		case M_Z : tmpBit = Z_FAULT; break;
//		case M_A : tmpBit = A_FAULT; break;
//		case M_B : tmpBit = B_FAULT; break;
//		case M_C : tmpBit = C_FAULT; break;
		}
		initSensor(&M->FaultSense, SENSOR_INDEX_FAULT, "Fault", FALSE, ACTIVE_LOW, tmpBit, FAULT_SENSOR_DEBOUNCE_READS);

		// change FaultSense.Polarity with MCode M95

		switch (M-Motors) {
		// uint32_t Limit1Sense.Bit : port/pin/bit for Limit1 sensor pin
//		case M_X : tmpBit = X_L1; break;
//		case M_Y : tmpBit = Y_L1; break;
//		case M_Z : tmpBit = Z_L1; break;
//		case M_A : tmpBit = A_L1; break;
//		case M_B : tmpBit = B_L1; break;
//		case M_C : tmpBit = C_L1; break;
		}
		initSensor(&M->Limit1Sense, SENSOR_INDEX_LIMIT1, "Limit1", FALSE, ACTIVE_LOW, tmpBit, DEFAULT_SENSOR_DEBOUNCE_READS);

		// change Limit1Sense.Polarity with MCode M98

		switch (M-Motors) {
		// uint32_t Limit2Sense.Bit : port/pin/bit for Limit2  sensor pin
//		case M_X :tmpBit = X_L2; break;
//		case M_Y :tmpBit = Y_L2; break;
//		case M_Z :tmpBit = Z_L2; break;
//		case M_A :tmpBit = A_L2; break;
//		case M_B :tmpBit = B_L2; break;
//		case M_C :tmpBit = C_L2; break;
		}
		initSensor(&M->Limit2Sense, SENSOR_INDEX_LIMIT2, "Limit2", FALSE, ACTIVE_LOW, tmpBit, DEFAULT_SENSOR_DEBOUNCE_READS);

		// change Limit2Sense.Polarity with MCode M99

		 // uint32_t Enable.Bit : port/pin/bit to control the motor enable
		M->Enable.Bit = 1;
		M->Enable.gpioPort = pinExtractPortPtr(M->Enable.Bit);
		M->Enable.gpioMask = pinExtractPinMask(M->Enable.Bit);

		// change Enable.Polarity with MCode M96
		// uint32_t Enable.Polarity : flag to invert the default "sense" of the Enable.Bit
		M->Enable.Polarity = ACTIVE_LOW;
#endif

		switch (M-Motors) {
		// uint32_t Step.Bit : port/pin/bit to control the motor step
		case M_X : M->Step.Bit = X_STEP; break;
		case M_Y : M->Step.Bit = Y_STEP; break;
		case M_Z : M->Step.Bit = Z_STEP; break;
		case M_A : M->Step.Bit = A_STEP; break;
		case M_B : M->Step.Bit = B_STEP; break;
		case M_C : M->Step.Bit = C_STEP; break;
		}
		M->Step.gpioPort = pinExtractPortPtr(M->Step.Bit);
		M->Step.gpioMask = pinExtractPinMask(M->Step.Bit);

		// change Step.Polarity with MCode M97
		// uint32_t Step.Polarity : flag to invert the default "sense" of the Step.Bit
		M->Step.Polarity = ACTIVE_HIGH;

		switch (M-Motors) {
		// uint32_t Direction.Bit : port/pin/bit to control the motor direction
		case M_X : M->Direction.Bit = X_DIR; break;
		case M_Y : M->Direction.Bit = Y_DIR; break;
		case M_Z : M->Direction.Bit = Z_DIR; break;
		case M_A : M->Direction.Bit = A_DIR; break;
		case M_B : M->Direction.Bit = B_DIR; break;
		case M_C : M->Direction.Bit = C_DIR; break;
		}
		M->Direction.gpioPort = pinExtractPortPtr(M->Direction.Bit);
		M->Direction.gpioMask = pinExtractPinMask(M->Direction.Bit);

		// change Direction.Polarity with MCode M94
		// uint32_t Direction.Polarity : flag to invert the default "sense" of the DirectionBit
		M->Direction.InvertDefault = NO;

#ifdef USE_HYREL_IO
		switch (M-Motors) {
		// TIM_TypeDef* TimerBase : pointer to the Timer that this motor will use for it's step pulse generation
		case M_X : M->TimerBase = TIM12; break;
		case M_Y : M->TimerBase = TIM13; break;
		case M_Z : M->TimerBase = TIM14; break;
		case M_A : M->TimerBase = TIM9; break;  // TIM9 needed for lathe
		case M_B : M->TimerBase = TIM10; break;
		case M_C : M->TimerBase = TIM11; break;
		}
#elif defined (USE_HYDRA_IO)
		switch (M-Motors) {
		// TIM_TypeDef* TimerBase : pointer to the Timer that this motor will use for it's step pulse generation
		case M_X : M->TimerBase = TIM12; break;
		case M_Y : M->TimerBase = TIM13; break;
		case M_Z : M->TimerBase = TIM14; break;
		case M_A : M->TimerBase = TIM10; break;
		case M_B : M->TimerBase = TIM9; break;  // TIM9 needed for lathe
		case M_C : M->TimerBase = TIM11; break;
		}
#endif
		InitMotorTimer(M, TRUE);  // this will set the correct TimerFreq

		M->AbortAbsolutePosition = INVALID_ARG_VALUE;
		M->AbortRelativePosition = INVALID_ARG_VALUE;
		M->SendStepViaCanbus = FALSE;
		M->LastReportedPosition = INVALID_ARG_VALUE;
		UpdateAxisLimitAndRates(M); // make sure everything is within proper limits.
		UpdateAxisTravelLimits(M);

		M->canMotor = FALSE;
		M->axisSelfHomingInProgress = FALSE;
		switch (M-Motors) {
		// TIM_TypeDef* TimerBase : pointer to the Timer that this motor will use for it's step pulse generation
		case M_X : M->maxCanMotors = 2; break;
		case M_Y : M->maxCanMotors = 2; break;
		case M_Z : M->maxCanMotors = 4; break;
		case M_A : M->maxCanMotors = 2; break;
		case M_B : M->maxCanMotors = 2; break;  // TIM9 needed for lathe
		case M_C : M->maxCanMotors = 2; break;
		}

		for (int canAddrIndex=0; canAddrIndex<MAX_NUM_CAN_MOTORS_PER_AXIS; canAddrIndex++)
		{
			M->canMotors[canAddrIndex].canAddress = 0;
			M->canMotors[canAddrIndex].selfHomed = 0;
			M->canMotors[canAddrIndex].selfHomingInProgress = 0;
		}
	}
	InitTimer1();   // master motor timer (_Dominant)
	InitTimer6();   // jog motor timer (_jogZ)

#ifdef USE_HYREL_IO
	_gs.latheMotor = &Motors[M_A];
#elif defined(USE_HYDRA_IO)
	_gs.latheMotor = &Motors[M_B];
#else
	_gs.latheMotor = NULL;
#endif

	M->autoReportPosition = TRUE;
	M->reportPositionWithStatus = FALSE;

	_Dominant.TimerPSC = 0;
	_Dominant.TimerARR = 0;
	_Dominant.TimerFreq = 168000000.0f;
	_Dominant.TimerBase = TIM1;

	DominantAxisPtr = &Motors[0];

	motionQ_init();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//
// motionQ ...
//
// basic idea is to insert and extra queue bewtween the cmdQue and the actual motion processing
// in order to have a place to precalculate accel/decel, vector rates, etc for all moves.  some moves
// can block the lookahead natures ensuring start/stop between move.
//
//
// processing:
//
// - serial commands as normal....converted from ascii to floats.
// - 3 basics things can happen to a command as it becuase the one being processed:
//     1) if it's a MOVE AND room in the motionQ, it is moved into the motionQ and nuked
//         from the cmdQue.  if no room in the motionQ, wait for space to free up.
//     2) if it's a move that decomposes into G1's (ie, G2, G3, canned routines), then
//         as space becomes available in the motionQ, add the decomposed moves to the
//         motionQ.
//     3) if it's anything else, then wait until the motionQ is EMPTY and then process
//         this command.  this will keep these commands in sync with the motion and avoid
//         a lot of booking in know what to change/update in the motionQ should we not block
//         loading of the motionQ on non motion commands.
//
// a scheduled task will periodically check the motionQ.  if it detects no motion in process
// AND a non-empty motion Q, then it will execute the oldest command in the motionQ. think
// of this as a jump start of the motionQ.
//
// when a command executed from the motionQ completes, it will automatically start the next oldest
// command from the motionQ (assuming there is one) and remove the just finished command from the motionQ.
//
// a scheduled task will periodically go through the motionQ and update the component velocities
// and accel/decel values should anything need to be changed so that time spent in between moves is
// limited to just reloading pulse counts and the timers.
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void isAutoPuffNeeded(motionEntryStruct *curr)
{
	curr->flags.needToPuffBeforeMove = CannedCyclePuffFlag;
}

////////////////////////////////////////////////////////////////////////////////

void isAutoPrimeoOrUnprimeNeeded(motionEntryStruct *curr)
{   // check to see if a prime or unprime is needed.
//GB XXX gregkar - isPrimeOrUnpri...() - show we ONLY do this if in a "printing" mode (not for
//GB XXX gregkar - isPrimeOrUnpri...() - //laser or milling or pick and place, etc???)
	// only used for auto priming/unpriming based on the presense or not of an E argument
	// on a vector motion (drawing) command.  if the first E seen after a ANY command that
	// was not printing (ie a G1 with an E arg), the a prime will be issued to restart flow.
	// initially, when a printing command (ie G1 with an E arg) is added to the queue, we have to
	// assume it is the last printing move and flags are set to issue an umprime.  however, if
	// the next move added to the quene is a printing move, then the the flags are adusted to
	// wipe out the unprime flag.
	//
	// additionally, there is a mode to remove unprime/prime pairs for a single non-priming move
	// that occurs between two printing moves if that single move requires less "time" to execute
	// than a predefined limit (time measured assuming no accel or decel and move operating
//GB XXX gregkar - isPrimeOrUnpri...() - are you okay with this time based single non-printing
//GB XXX gregkar - isPrimeOrUnpri...() - move unprmime/prime removal?
	// completely at it's cruiseVelocity -- and comapred to _autoPrimeAndReverseMinTime set with M227 S<minTime>).

	// NOTE "curr" is pointing to the local copy of memory in motionQ_addCommand and is NOT yet in the queue.
	// therefore, it wil be reference separate from Q references.
	// references to # Q entries are adjust accordingly

	if (_unprimePrimeControl == AUTO_UNPRIME_PRIME)
	{   // use the presence of an E or lack of an E to control sending out prime/unprime commands
		if (ARG_E_PRESENT)
		{   // current move will extrude, so check if prior move was extruding...if not, then PRIME
			curr->flags.hadAnE = TRUE;
			curr->flags.needToUnprimeAfterMove = TRUE;  // no moves are known to follow this one (yet), so unprime (can be corrected later)
			if (motionQ_empty())
			{   // first move, so need to prime
				curr->flags.needToPrimeBeforeMove = TRUE;   // GB XXX -- really need to look at M->E state ??? UNPRIMED
			}
			else if (motionQ.newest->flags.hadAnE == TRUE)
			{   // prior move was extruding and hasn't unprimed, then the prior move no longer needs to unprime when it finishes
				if (motionQ.newest->flags.unprimeIssued == FALSE)
				{
					motionQ.newest->flags.needToUnprimeAfterMove = FALSE;  // correct the prev entry....it no longer needs to unprime when it finishes.
					motionQ.newest->flags.needToUnprimeDuringMove = FALSE;  // correct the prev entry....it no longer needs to unprime when it finishes.
				}
			}
			else if (motionQ.newest->flags.hadAnE == FALSE)
			{   // prior move was not extruding, so need to prime before starting this move
				curr->flags.needToPrimeBeforeMove = TRUE;
				if (motionQ_numEntries() >= 2)
				{   // at least 2 entries so we can look for a Prime Unprime Prime sequence)
					// to get here, we know the curr move had an E, the move prior to that did not.
					// so if the move before that DID have have an E, then we check whether the
					// unprime/prime pair can be removed -- if the non-printing move in the middle
					// was short in duration.
					if (motionQ.newest->prev->flags.hadAnE == TRUE)
					{   // hit the magic sequence.
						if (motionQ.newest->moveTime < _autoReverseAndPrimeMinTime)
						{   // remove the prime/unprime if the move time is less than the spec'd number
							curr->flags.needToPrimeBeforeMove = FALSE;
							motionQ.newest->prev->flags.needToUnprimeAfterMove = FALSE;
							motionQ.newest->prev->flags.needToUnprimeDuringMove = FALSE;
#ifdef COLLECT_METRICS
							_metrics.unprime_primes_avoided++;
#endif
						}
					}
				}
			}
		}
	}
	else
	{
		if (ARG_E_PRESENT)
		{
			curr->flags.hadAnE = TRUE;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_init(void)
{   // clear the motionQ storage and setup up the pointers for the bidirectional
	// circular queue/fifo
	//      motionQ.oldest will always point to the oldest entry and will point to
	//          the entry being executed, as the entry is not removed until AFTER all
	//          motion is complete for that move.
	//      motionQ.newest will always point to the move recently added entry.
	//      motionQ.planned will always point to the most recent move that is
	//          completely planned (entry, cruise, and ext speeds are known as well
	//          as acceration ramps).


	int i;

	// set up the doubly linked list motionQ.
	bzero(&motionQ, sizeof(motionQStruct));
	for (i=0; i<SIZE_OF_MOTION_QUEUE; i++)
	{
		if (i < (SIZE_OF_MOTION_QUEUE-1))
			motionQ.entries[i].next = &motionQ.entries[i+1];
		else
			motionQ.entries[i].next = &motionQ.entries[0];

		if (i==0)
			motionQ.entries[i].prev = &motionQ.entries[SIZE_OF_MOTION_QUEUE-1];
		else
			motionQ.entries[i].prev = &motionQ.entries[i-1];
#ifdef GB_DEBUGGING
		motionQ.entries[i].index = i;
#endif //GB_DEBUGGING
	}
	motionQ.validEntries = 0;
	motionQ.countdownDelayToExecuteMove = 0;
	motionQ.oldest = &motionQ.entries[0];
	motionQ.newest = motionQ.oldest->prev;
	motionQ.planned = motionQ.oldest;
	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // setup Q to match current position
		M->Q_POSITION = M->POSITION;;
		M->Q_LastRequestedPositionInUnits = ((float)M->Q_POSITION * M->UnitsPerPulse) - SumOfAllMotorOffsets(M);
	}
}

////////////////////////////////////////////////////////////////////////////////

int motionQ_getCountdownDelayToExecuteMove(void)
{   // get the current countdown timer for delaying execution of the motionQ
	// buying time for it to partially fill
	return(motionQ.countdownDelayToExecuteMove);
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_setCountdownDelayToExecuteMove(int value)
{
	motionQ.countdownDelayToExecuteMove = (value <= 0) ? 1 : value; // must be non-zero when setting a delay
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_decrCountdownDelayToExecuteMove(void)
{   // count down the motion delay timer towards 0.
	if (motionQ_numEntries() > MOTIONQ_ENTIRES_TO_START_PROCESSING)
		motionQ.countdownDelayToExecuteMove = 0;  // don't wait full time, just start motion
	else
		motionQ.countdownDelayToExecuteMove--;
}

////////////////////////////////////////////////////////////////////////////////

boolean motionQ_hasSpace(int reqSpace)
{
	return((SIZE_OF_MOTION_QUEUE - motionQ.validEntries) >= reqSpace);
}

////////////////////////////////////////////////////////////////////////////////

boolean motionQ_almostFull(int headroom)
{
	return(motionQ.validEntries >= (SIZE_OF_MOTION_QUEUE - headroom));
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_deleteNewestCommand(void)
{   // remove the newest (most recently added) command from the queue.  typically
	// needed during an abort sequence to clean up the queue for a rapid stop.
	// no attempt is made by this routine to correct any other flags (such as
	// prime/unprime flags).

	if (motionQ_empty())
	{
		// nothing to do -- error -- should barf() as this would mean a serious programming error
		barf("motionQ_deleteNewestCommand");
	}
	else
	{
		if (motionQ.planned == motionQ.newest)
		{
			motionQ.planned = motionQ.newest->prev;
		}
		motionQ.newest = motionQ.newest->prev;
		motionQ.validEntries--;
	}
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_deleteCurrentMove(void)
{   // remove the oldest (least recently added) command from the queue.  typically
	// needed when the motion is complete on a move.  no attempt is made by this
	// routine to correct any other flags (such as prime/unprime flags).

	if (motionQ.planned == motionQ.oldest)
	{
		motionQ.planned = motionQ.oldest->next;
	}
	motionQ.oldest = motionQ.oldest->next;
	motionQ.validEntries--;
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_deleteCurrentMoveAndExecuteNextMove(void)
{   // remove the oldest (least recently added) command from the queue.  typically
	// needed when the motion is complete on a move.  no attempt is made by this
	// routine to correct any other flags (such as prime/unprime flags).  it will
	// however execute an unprime if called for by this move before it is deleted
	// after the unprime if is finished, the routine will be reentered to complete
	// the delete.  after that, if there there's still work in the queue, kick off
	// the next move.

	if (motionQ_empty())
	{
		barf("motionQ_deleteCurrentMoveAndExecuteNextMove(): motionQ_empty");
		return;
	}

	if (motionQ.oldest->flags.needToUnprimeDuringMove)
	{   // if early unprime, already would have processed unprime..but if it lasted longer than the move, need
		// to wait for the remainder of the time
		UnPrimeTimer = _abortInProgress ? 0 : DominantAxisPtr->residualUnprimeTime;
#ifdef GB_RESIDUAL_PIN
		if (UnPrimeTimer)
			pinSet(GB_RESIDUAL_PIN);
#endif
	}
	else if (motionQ.oldest->flags.needToUnprimeAfterMove)
	{   // normal unprime
		UnPrime(currentOutboxPtr);
		motionQ.oldest->flags.unprimeIssued = TRUE;
		UnPrimeTimer = _abortInProgress ? 0 : abs(getCurrentUnPrimeTimeMs(currentOutboxPtr));
		if (UnPrimeTimer == 0)          // special case, not waiting for countdown timer
			motionQ_unprimeComplete();
	}
	if (_abortInProgress)
		UnPrimeTimer = 0;

	if (UnPrimeTimer == 0)
	{   // only delete after timer has gone to 0
		motionQ_deleteCurrentMove();

		if (motionQ_notEmpty() && !_gcodePaused)
			motionQ_executeMove();
	}

	if (motionQ_empty() && _abortInProgress)
		cleanupAfterAbortFinishes();
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_reset(void)
{   // reset the motionQ so that it appears empty and pointers point to the same
	// physical memory as init time
	motionQ.validEntries = 0;
	motionQ.countdownDelayToExecuteMove = 0;
	motionQ.oldest = &motionQ.entries[0];
	motionQ.newest = motionQ.oldest->prev;
	motionQ.planned = motionQ.oldest;
}

////////////////////////////////////////////////////////////////////////////////

motionEntryStruct *motionQ_nextAvailable(void)
{   // update the queue so one edditional entry and move the "newest" pointer to
	// point to that location.  addditionally, zero out the data portion of the new
	// entries (do not disturb the next/prev pointers that are part of the circular
	// queue structure.

	if (motionQ_full())
	{
		barf("FATAL - attempting to add to a full motionQ");
	}
	else
	{
		motionQ.validEntries++;
#ifdef COLLECT_METRICS
		_metrics.total_motionQprocessed++;
		if (motionQ.validEntries > _metrics.max_motionQvalidEntries)
			_metrics.max_motionQvalidEntries = motionQ.validEntries;
#endif
		motionQ.newest = motionQ.newest->next;

	//safer: 
	bzero(motionQ.newest, offsetof(motionEntryStruct, index));
	//NUKE bzero(motionQ.newest, sizeof(motionEntryStruct) - MOTION_ENTRY_STRUCT_BYTES_TO_PRESERVE); //-N so that index/prev/next pointers are not cleared

	}
	return(motionQ.newest);
}

////////////////////////////////////////////////////////////////////////////////

void sendBlockRelativeAxisMove(sensorStruct *sensorPtr, char AxisLabel)
{
	if (sensorPtr->ErrorMessageSent == FALSE)
	{   // message has not been sent yet
		sprintf(_errorStr, "%s relative motion blocked on %c axis due to tripped %s sensor",
				(sensorPtr->SensorIndex == SENSOR_INDEX_LIMIT1) ? "Negative" : "Positive",
				AxisLabel, sensorPtr->Name);
		sendError(_errorStr);
		sensorPtr->ErrorMessageSent = TRUE;
	}
}

////////////////////////////////////////////////////////////////////////////////

void setupFlowInfo(motionEntryStruct *curr, float E_destinationInPulses)
{
	curr->extrusionControl = _extrusionControl; // save a cope for execute time
	curr->flowOverridePct = 1.0f;
	curr->flowCrossSectionTimesPPU = 0.0f;
	curr->flowPulses = 0;
	if (ARG_E_PRESENT)
	{
		curr->flags.hadAnE = TRUE;
		deviceExtrusionControlStruct *extPtr = &currentOutboxPtr->ExtrusionControl; // local pointer
		if (FLOW_RATE_CREATED_ON_MOTORC)
		{
			if (USING_E_VALUES)
			{   // pre-calc pulses -- need to do here because G92 is a "IMPACT_DEST" command and the "E_Q" would get out of date if this
				// calc was done later
				curr->flowPulses = E_destinationInPulses - E_Q_POSITION;
				E_Q_POSITION = E_destinationInPulses;   // only update if a move is going to take place
			}
			else
			{   // hijacking motor C for step flow creation -- calculate from volume
				if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_HEATED_EXTRUDER) || (currentOutboxPtr->deviceFamily == DEVICE_FAMILY_UNHEATED_EXTRUDER))
				{
					curr->flowPulses = (int32_t)(curr->distance * extPtr->SliceHeightInMm * extPtr->ExtrusionWidthInMm * extPtr->PulsesPerUnit);
				}
				else if ((currentOutboxPtr->deviceFamily == DEVICE_FAMILY_INKJET) && Motors[M_C].SendStepViaCanbus)
				{
					curr->flowPulses = imax(1, (int32_t)(curr->distance * _gs._inkjet.dropletsPerMm));  // imax to ensure at least on dot on short moves
				}
			}
			curr->flowOverridePct = fFitWithinRange((_CompositeFlowOverridePct * extPtr->ExtrusionRateOverridePct * (ARG_S_PRESENT ? ARG_S : 1.0f)), OVERRIDE_PCT_MIN, OVERRIDE_PCT_MAX);
		}
		else
		{   // not using E and not sending using M_C
			curr->flowCrossSectionTimesPPU = extPtr->SliceHeightInMm * extPtr->ExtrusionWidthInMm * extPtr->PulsesPerUnit; // will be modified with the motion rate later to get the flow rate
			curr->flowOverridePct = fFitWithinRange((_CompositeFlowOverridePct * (ARG_S_PRESENT ? ARG_S : 1.0f)), OVERRIDE_PCT_MIN, OVERRIDE_PCT_MAX); // extOverride will be done on the head
		}

		curr->flowPulses = (float)curr->flowPulses * curr->flowOverridePct;
		curr->flowCrossSectionTimesPPU *= curr->flowOverridePct;
	}
}

////////////////////////////////////////////////////////////////////////////////

#define DOMINANT_AXIS_ISR_OVERHEAD_TIME_USEC    (4.5f + 3.8f)   // included dom one-shot setup; TIM1 reload; accel calc; TIM1 retiming calc (using TIM7 isr)
// drops to 7 when cruising, but not taking advantage of that
#define DOMINANT_AXIS_SLAVE_CALC_TIME_USEC      1.6f            // per non-dom axis math/timer setup for one shots
#define AXIS_STEP_ISR_TIME_USEC                 2.5f            // per call "ProcessMotion"
#define MAX_TIME_UTILIZATION_FOR_MOTION_ISR     0.75
#define MAX_CANBUS_UTILIZATION_FOR_STEP_PKTS    0.90f

float calculateIsrLimitedSpeed(motionEntryStruct *curr, int32_t dominantAxisPulses, int32_t totalPulses, int numAxesInvolvedInMove)
{
	int32_t domPulses = imax(dominantAxisPulses, curr->flowPulses);   //either could end up being dominant axis
	float tim1TimeUsec = (float)domPulses * (DOMINANT_AXIS_ISR_OVERHEAD_TIME_USEC + ((float)(numAxesInvolvedInMove - 1) * DOMINANT_AXIS_SLAVE_CALC_TIME_USEC));
	float stepTimeUsec = (float)(totalPulses) * AXIS_STEP_ISR_TIME_USEC;
	float isrTimeSec = ((float)(tim1TimeUsec + stepTimeUsec)) / (MAX_TIME_UTILIZATION_FOR_MOTION_ISR * 1000000.f); // leave N% dead time at peak rate (Much more during accel/decel)
	float canTimeSec = (float)curr->flowPulses / (MAX_CANBUS_UTILIZATION_FOR_STEP_PKTS * CANBUS_BIT_RATE / 64.0f);  // 500Kbs / 64 bits_per_step_packaet -- 75% bus utilization

	float maxAllowableSpeed = curr->distance / fmaxf(isrTimeSec, canTimeSec);
	return(maxAllowableSpeed);
}

////////////////////////////////////////////////////////////////////////////////

// borrowed heavily from grbl
// entry_speed ~ startSpeed
// max_entry_speed ~ StartSpeedLimit
// max_juntion_speed  ~ inntersectionSpeedLimit
// nominal_speed = speed

extern boolean validFirmwareKey(boolean);

passFail_t motionQ_addCommand(float motionRate, boolean homing)
{
	// This covers all types of motion sources (G0, G1, G2, G28 etc) as well as LINEAR and/or ROTARY axes
	// The main differentiation is:
	//   1) Homing - in the homing case, all axes operate independently (unique acceleration to their homing
	//      speed and then once the homeSensor is detected, travel the hysteresis amount to get to the actual home.
	//      Each axes will use it's own timer to calculate the motion rate and will adjust the timer periodicially
	//      (every 1ms at the time of writing this) to account for acceleration and deceleration.
	//   2) Not Homing - all other cases fit under this heading which has all axes operating in a coordinate fashion
	//      such that all axes start the move at the same moment and finish at the same moments.  The pulse rates
	//      for each axes follow the same relative ratio to the other axes such that the positional movement is
	//      uniform. (ie, the relative motion a G1 X Y Z move will be a line in 3 space between the start and end points).
	//
	//      In these moves, an additional timer is employed to effectively create the pulse stream matching the stream
	//      needed for the axes with the great number of pulses (will have the highest pulse rate).  That single timer
	//      will be adjusted periodicially (every 1ms at the time of writing this) to account for acceleration
	//      and deceleration.  Each time that timer fires, the code will check whether any axes will need to generate
	//      a pulse within the amount of time before the timer fires again. If so, it will set up that axes' timer to
	//      generate a single pulse.    This keeps the motion locked stepped and puts the pulses exactly where they
	//      should be in time (sans slight error from simulataneous multiple interrupts).   It should mimic the behavior
	//      of a highly oversampled Bresenham algorithm without the extremely high pulse rate requirement on the dominant
	//      axis.
	//
	//      The coordinated moves fall into three categories based on the types of axes involved:
	//
	//           a) All LINEAR - simplest case where the Pythagorean distance is used along with the requested rate
	//              to determine the component axis speed
	//           b) Mixed LINEAR and ROTARY - in this case the Pythagorean distance of the participating LINEAR axes
	//              is used along with the requested rate to determine the component axis speed.   the time required
	//              for the linear potion of the move is used to define the rotary rate such that the rotary motion starts
	//              and ends with the linear axes.  However, if the time required for the linear portion is not sufficient
	//              to allow the rotary axis to complete it's motion without "speeding" the the motion of linear portion
	//              is slowed down to proportionally such that the rotary axes are right at their speed limits.
	//           c) All Rotary -  this is the most challenging case.  since there's no linear move to define a time target
	//              for the move, the next step is to try to keep the apparent linear motion of the rotary past the
	//              perpendicular axis (ie, under the tip or bit) at the requested motion rate.  The danger is that as the
	//              the axis of rotation of the rotary axis is close to or is coincident to the axis perpendicular to the
	//              plane of rotation, the math explodes, so we'll need to clamp to some limit.
	//
#ifdef GB_FUNC_ADD_COMMAND_PIN
	GB_FUNC_ADD_COMMAND_SET;
#endif //GB_FUNC_ADD_COMMAND_PIN

	int axisIndex;
	MotorStructure *M;
	motionEntryStruct localMotionQEntry;
	motionEntryStruct *curr = &localMotionQEntry;                            // current entry that is being added
	int numAxesInvolvedInMove=0;
	float destinationInUnits[MAX_NUMBER_OF_MOTORS];     // target destination in mm or deg
	int32_t destinationInPulses[MAX_NUMBER_OF_MOTORS]; // target destination in motor pulses (steps) (could temporarily go negative. will be clamped)
	int32_t dominantAxisPulses = 0;
	int32_t linearPulses = 0;                               // total linear axis pulses
	int32_t rotaryPulses = 0;                               // total rotary axis pulses
	float intersectionSpeedLimit = 0.0f;                // used for determining speed at vector intersections


	if ((_validFirmwareKey = validFirmwareKey(1)) == FALSE)
	{
		_g4DwellTimer = 1000;   // set a short delay (1000ms) to keep repretrel from swamping itself with sending data to 407/429 that
							// the gui becomes non-responsive
		return(FAIL);
	}

	if (_blockAllMotion)
	{   // do not process an new moves
		_g4DwellTimer = 100;    // set a short delay to keep repetrel from swamping itself
		if (_blockAllMotionMessageSent == FALSE)
		{
			sendError("All motion blocked -- try to clear the source of the fault and the press RESET in the GUI to continue");
			_blockAllMotionMessageSent = TRUE;
		}
		return(FAIL);
	}
	else if (_blockAbsoluteMotion && !_IncrementalMove &!homing)
	{
		if (_blockAbsoluteMotionMessageSent == FALSE)
		{
			sendError("Motion fault present -- Only relative moves are allowed until fault is cleared");
			_blockAbsoluteMotionMessageSent = TRUE;
		}
		return(FAIL);
	}
	else if (_jogging.enabled || _ABjogging.enabled)
	{
		if (_blockMotionForJoggingMessageSent == FALSE)
		{
			sendError("Jogging enabled -- normal GCODE motion is blocked");
			_blockMotionForJoggingMessageSent = TRUE;
		}
		return(FAIL);
	}
	else if (_highVoltageIsNotStableCoundownMs)
	{
		if (_blockMotionForHighVoltageMessageSent == FALSE)
		{
			sendError("Motion blocked - waiting for power to stabilize");
			_blockMotionForHighVoltageMessageSent = TRUE;
		}
		return(FAIL);
	}

#ifdef GB_DEBUG_MOTION_Q
	for (axisIndex=0; axisIndex<=MAX_NUMBER_OF_MOTORS; axisIndex++)
		destinationInUnits[axisIndex] = 0.0f;
	for (axisIndex=0; axisIndex<=MAX_NUMBER_OF_MOTORS; axisIndex++)
		destinationInPulses[axisIndex] = 0;
#endif

	bzero(&localMotionQEntry, sizeof(motionEntryStruct)); // code relies on a clear start
	curr->cmdQueIndex = ExecutionPtr - cmdQue;   // save the index of the cmdQue were our params are located.
	curr->flags.lastInSequence = CannedCycleLastMove;

	curr->lineNumber = _gcodeMotionLineNumber;

#ifdef GB_DEBUG_MOTION_Q    // allow control over data dependent breakpoints
	curr->debugArgDValue = ARG_D;

	float matchingArgD = 0.0f;  // change this value as needed
	if (curr->debugArgDValue == matchingArgD)
		matchingArgD = INVALID_ARG_VALUE; // stick breakpoint here

	int matchingIndex = 0;      // change this value as needed
	if (curr->index == matchingIndex)
		matchingIndex = -1;     // stick breakpoint here
#endif

	if (_sendingGBStringsMask & GB_STRING_MOTION) // use M797 S<mask> to enable
	{
		M = &Motors[M_Z];
		sprintf(_tmpStr, "iZ:  Z=%4.3f  mov=%s  POS=%ld  QPOS=%ld  LRP=%4.3f  OFS=%4.3f",
				ARG_Z, _IncrementalMove==TRUE ? "inc" : "abs", M->POSITION, M->Q_POSITION, M->Q_LastRequestedPositionInUnits, SumOfAllMotorOffsets(M));
		sendGB(_tmpStr);
	}

	if(ARG_H_PRESENT && (ARG_G != 38.0f))
	{   // check if this move is using a different tool -- ignore for a G38 though (has a different meaning)
		int toolIndex = (int)ARG_H;
		if ((toolIndex < 0) || (toolIndex >= NUM_TOOL_OFFSETS)) //GB XXX gregkarl would a toolIndex of 0 be an error here?
		{
			ReportInvalidGcodeHArg();
			toolIndex = 0;  // index 0 should have a 0 offset
		}
		currentToolIndex = toolIndex; //now we have the current tool index
	}

	if (ARG_C_PRESENT && (FLOW_RATE_CREATED_ON_MOTORC))
	{   // illegal dual use of axis C
		if (_errors.sent.flags.illegalUseOfAxisC == FALSE)
		{
			_errors.sent.flags.illegalUseOfAxisC = TRUE;
			sendError("move includes axis C when also creating extrusion STEP pulse for head using motor C");
			sprintf(_errorStr, "G%d X%3.2f Y%3.2f Z%3.2f A%3.2f B%3.2f C%3.2f", (int)ARG_G, ARG_X, ARG_Y, ARG_Z, ARG_A, ARG_B, ARG_C);
			sendError(_errorStr);
		}
		ARG_C = INVALID_ARG_VALUE;
	}

	// determine distance of travel, including all offsets and determine which axis/axes are involved in the move
	// such that any additional processing can be limited to just that axis/those axes.

	for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
	{
		M = &Motors[axisIndex];
		boolean tmpMotorArgPresent = motorArgPresent(M);    // record results... checked a lot so avoid multiple calls

		if (M->MotorInstalled && (tmpMotorArgPresent || (M->SubstituteAxisArgWithArgE && ARG_E_PRESENT)))
		{   // has a chance of moving
			boolean killAxisMove = FALSE;

			float tmpMotorArgValue; // used to unravel the 5 valid permutations of args present combined with substitute
			if (tmpMotorArgPresent)
			{
				tmpMotorArgValue = getMotorArgInNativeUnits(M);
			}
			else if (M->SubstituteAxisArgWithArgE)
			{
				if (_IncrementalMove)
				{
					if (_IncrementalEMove)
						tmpMotorArgValue = convertArgToMM(ARG_E);
					else
						tmpMotorArgValue = convertArgToMM(ARG_E) - E_Q_LastRequestedPositionInUnits;
				}
				else
				{   // absoluteMove
					if (_IncrementalEMove)
						tmpMotorArgValue = E_Q_LastRequestedPositionInUnits + convertArgToMM(ARG_E);
					else
						tmpMotorArgValue = convertArgToMM(ARG_E);
				}
			}
			else
			{
				barf("lost in sustitute_E code 0");
			}

			//at top of method: killAxisMove |= _blockAllMotion;
			//part of "if":     killAxisMove |= !M->MotorInstalled;
			//part of "if":     killAxisMove |= !(!M->SubstituteAxisArgWithArgE && tmpMotorArgPresent);
			//part of "if":     killAxisMove |= !(M->SubstituteAxisArgWithArgE && ARG_E_PRESENT);

			killAxisMove |=  ((ARG_G == 928.0f) && !tmpMotorArgPresent);    // avoid issue with argE and substitute E axis
			if (homing)
			{
				killAxisMove |= !tmpMotorArgPresent;    // avoid issue with argE and substitute E axis
				if (!killAxisMove && (killAxisMove = ((M->Limit1Sense.State == SENSOR_TRIPPED) && (M->HomingDirection == HOME_TOWARD_ZERO))))
					sendBlockRelativeAxisMove(&M->Limit1Sense, M->AxisLabel);
				if (!killAxisMove && (killAxisMove = ((M->Limit2Sense.State == SENSOR_TRIPPED) && (M->HomingDirection == HOME_AWAY_FROM_ZERO))))
					sendBlockRelativeAxisMove(&M->Limit2Sense, M->AxisLabel);
			}
			else if (_IncrementalMove)
			{
				if (!killAxisMove && (killAxisMove = ((M->Limit1Sense.State == SENSOR_TRIPPED) && (tmpMotorArgValue < 0.0f))))
					sendBlockRelativeAxisMove(&M->Limit1Sense, M->AxisLabel);
				if (!killAxisMove && (killAxisMove = ((M->Limit2Sense.State == SENSOR_TRIPPED) && (tmpMotorArgValue > 0.0f))))
					sendBlockRelativeAxisMove(&M->Limit2Sense, M->AxisLabel);
			}
			else // AbsoluteMove
			{
				killAxisMove |= _blockAbsoluteMotion;
			}

			if (M->latheMode != LATHE_MODE_OFF)
			{   // can't have a potential move on this axis when axis is in lathe mode!!!
				sprintf(_errorStr, "Illegal move involving lathe mode axis %c", M->AxisLabel);
				catastrophicError(_errorStr);
				killAxisMove = TRUE;
			}

			if (!killAxisMove)
			{   // still a valid move until proven otherwise
				if ((_sendingGBStringsMask & GB_STRING_ADD_MOTIONQ) && (CannedCycleFlag == 28)) // use M797 S<mask> to enable
				{
					sprintf(_tmpStr,"step=%d axis=%c POS=%ld Q_POS=%ld Q_LRP=%4.3f MotARG=%4.3f",
							CannedCycleStep, M->AxisLabel, M->POSITION, M->Q_POSITION, M->Q_LastRequestedPositionInUnits,
							(getMotorArgInNativeUnits(M)==MAXFLOAT)?999999.999:getMotorArgInNativeUnits(M)); sendGB(_tmpStr);
					sprintf(_tmpStr,"       MaxTr=%4.3f MaxTr=%d HO=%ld HD=%d",
							M->MaximumTravelInUnits, M->MaximumTravelInPulses, M->HomeDestinationInPulses, M->HomingDirection); sendGB(_tmpStr);
					sprintf(_tmpStr,"       G92=%4.3f fix=%4.3f head=%4.3f tool=%4.3f",
							M->G92Offset, M->FixtureOffsets[currentFixtureIndex], M->HeadOffsets[currentHeadIndex], ToolOffsets[currentToolIndex].ToolLength); sendGB(_tmpStr);
					sprintf(_tmpStr,"       fixIdx=%d head=Idx%d toolIdx=%d", currentFixtureIndex, currentHeadIndex, currentToolIndex); sendGB(_tmpStr);
				}

				// need temp variables to handle the using E value for the motor axis value

				if (_IncrementalMove)
				{
					destinationInUnits[axisIndex] = M->Q_LastRequestedPositionInUnits + tmpMotorArgValue;
				}
				else if (homing)
				{   // homing is an absoluteMove
					destinationInUnits[axisIndex] = tmpMotorArgValue;
				}
				else if (M->HasBeenHomed)
				{   // absoluteMove
					destinationInUnits[axisIndex] = tmpMotorArgValue;
				}
				else //if (!M->HasBeenHomed)
				{   // absoluteMove -- not homed yet, so don't allow move and report error
					if (tmpMotorArgValue != M->Q_LastRequestedPositionInUnits)
					{   // actually trying to move to somewhere other than the current position when NOT homed (error)
						if (M->HomeSense.ErrorMessageSent == FALSE)
						{
							sprintf(_errorStr, "Absolute motion blocked on unhomed %c axis dest: %4.3f", (int)M->AxisLabel, getMotorArgInNativeUnits(M));
							sendMotionError('H', M->AxisLabel, getMotorArgInNativeUnits(M), 0.0f, _errorStr);
							M->HomeSense.ErrorMessageSent = TRUE;
						}
						killAxisMove = TRUE;
					}
				}
			}

			M->Q_LastRequestedPositionInUnits = destinationInUnits[axisIndex];

			if (killAxisMove)
			{   // some additional case arrived to prevent the axis move
				destinationInPulses[axisIndex] = M->Q_POSITION; // force zero pulses
				destinationInUnits[axisIndex] = ((float)M->Q_POSITION * M->UnitsPerPulse) - SumOfAllMotorOffsets(M);    //make sure Q position matches

				M->Q_LastRequestedPositionInUnits = destinationInUnits[axisIndex];
				curr->PULSES_TO_GO[axisIndex] = 0; // force zero pulses
			}
			else
			{   // allow the move
				destinationInPulses[axisIndex] = (int)roundf((destinationInUnits[axisIndex] + SumOfAllMotorOffsets(M)) * M->PulsesPerUnit);
				boolean clamped;
				destinationInPulses[axisIndex] = ClampAndWarnToAxisRangeInPulses(&clamped, M, destinationInPulses[axisIndex], homing);    // make sure travel stays in the workarea
				if (clamped)
				{
					destinationInUnits[axisIndex] = (destinationInPulses[axisIndex] * M->UnitsPerPulse) - SumOfAllMotorOffsets(M);    //re-calc after clamping
				}
				curr->PULSES_TO_GO[axisIndex] = destinationInPulses[axisIndex] - M->Q_POSITION;   // will take abs value later .. need to extract direction info first
			}
			if (homing)
			{   // check state of home sensor to see if move needs to be killed because already on the homing sensor
#ifdef USE_CAN_MOTOR_HOME_SENSOR
				if ((M->canMotor && M->HomeSense.State == SENSOR_TRIPPED) || (!(M->canMotor) && (fastReadDebouncedSensorState(&M->HomeSense) == SENSOR_TRIPPED))) // check current state
#else //!USE_CAN_MOTOR_HOME_SENSOR
				//NUKE if (fastReadDebouncedSensorState(&M->HomeSense) == SENSOR_TRIPPED) // check current state
				if (fastReadDebouncedSensorState(&M->HomeSense) != SENSOR_OPEN) // check current state -- only proceed if the sensor is not open
#endif //USE_CAN_MOTOR_HOME_SENSOR
				{
					curr->PULSES_TO_GO[axisIndex] = 0;  // force no motion
				}
			}

			if ((_sendingGBStringsMask & GB_STRING_ADD_MOTIONQ) && (CannedCycleFlag == 28)) // use M797 S<mask> to enable
			{
				sprintf(_tmpStr,"       IM=%d, destU_QLRP=%4.3f destP=%ld sum=%4.3f P2G=%ld",
						(int)_IncrementalMove, (destinationInUnits[axisIndex]==MAXFLOAT)?999999.999:destinationInUnits[axisIndex], destinationInPulses[axisIndex], SumOfAllMotorOffsets(M), curr->PULSES_TO_GO[axisIndex]);
				sendGB(_tmpStr);
				sprintf(_tmpStr," "); sendGB(_tmpStr);
			}

			if (curr->PULSES_TO_GO[axisIndex])
			{   // really will move
				numAxesInvolvedInMove++;
				if (curr->PULSES_TO_GO[axisIndex] < 0)
					curr->direction[axisIndex] = -1;        // moving forward
				else if (curr->PULSES_TO_GO[axisIndex] > 0)
					curr->direction[axisIndex] = 1;         // moving in reverse

				if (M->AxisType == LINEAR)
				{
					curr->unitVector[axisIndex] = curr->PULSES_TO_GO[axisIndex] * M->UnitsPerPulse; // divide to normalize will occur once distance is known
					curr->distance += sqr(curr->unitVector[axisIndex]);     // sqrt will be taken later, after all sum of squares
					linearPulses += abs(curr->PULSES_TO_GO[axisIndex]);
					M->scaleDegreesToMm = 1.0f;
				}
				else if (M->AxisType == ROTARY)
				{
 #ifdef GB_HIDDEN_WARNINGS
					int shortcut_to_determine_radius; // not full 3D orientation
#endif //GB_HIDDEN_WARNINGS

					float circumference = TWO_PI * (Motors[M_Z].Q_LastRequestedPositionInUnits - M->RotaryOffset[M_Z]);
					float degrees = curr->PULSES_TO_GO[axisIndex] * M->UnitsPerPulse;

					if (circumference < TWO_PI)
					{   // radius less than 1, so have to ignore
						curr->unitVector[axisIndex] = degrees;
						M->scaleDegreesToMm = 1.0f;
					}
					else
					{ // linear distance, based on circumference & rotations  ==  (2*PI*r * rotations) * (totalDegrees/360)
						curr->unitVector[axisIndex] = circumference * (degrees / 360.0f);  // circumference * rotations
						M->scaleDegreesToMm = 360.0f / circumference;
					}
					curr->distance += sqr(curr->unitVector[axisIndex]);     // sqrt will be taken later, after all sum of squares
					rotaryPulses += abs(curr->PULSES_TO_GO[axisIndex]);
				}
				curr->PULSES_TO_GO[axisIndex] = abs(curr->PULSES_TO_GO[axisIndex]);

				// also determine the dominant axis along the way.  in the case of a tie, the first axis
				// encountered of those axes that tied becomes the dominant axis
				//
				if (curr->PULSES_TO_GO[axisIndex] > dominantAxisPulses)
				{
					dominantAxisPulses = curr->PULSES_TO_GO[axisIndex];
					curr->flags.dominantAxis = (unsigned)axisIndex;
				}
			}
			else
			{   // (curr->PULSES_TO_GO[axisIndex] == 0)
				curr->direction[axisIndex] = 0;     // not moving  (dir is used as +/- increment, so 0 will ensure no change to position
			}
		} // end of: if (M->AxisInstalled && (tmpMotorArgPresent || (M->SubstituteAxisArgWithArgE && ARG_E_PRESENT)))
	} // end of: for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)

	//============================ visual break =============

	// for non-substitute E uses !copy_E) AND most important to keep _E_Q up to date;
	float E_destinationInUnits = fmaxf(0.0f, convertArgToMM(ARG_E) + (_IncrementalEMove ? E_Q_LastRequestedPositionInUnits : 0.0f));
	float E_destinationInPulses = fmaxf(0, (int)(E_destinationInUnits * currentOutboxPtr->ExtrusionControl.PulsesPerUnit));
	E_Q_LastRequestedPositionInUnits = E_destinationInUnits; //always do, even if no move occurs

	// must execute, even if move might be tossed out
	curr->requestedFeedrate = CheckForNewFeedrate(motionRate == VECTOR_MOTION_RATE);

	if (dominantAxisPulses)
	{   // at least one axis with a non-zero number of pulses to move -- so there will be a move
		rate_t rateIndex = NO_RAMP; // safe default

		isAutoPuffNeeded(curr);

		curr->distance = fpu_sqrtf(fmaxf(0.0f, curr->distance));     // finish distance calculation (take sqrt of already collected sum of squares of linear axis)
		curr->scaleDegreesToMm = Motors[curr->flags.dominantAxis].scaleDegreesToMm; // scaling is 1.0f for linear, but allows matching feedrate to the working circum of a rotary

		setupFlowInfo(curr, E_destinationInPulses);
		if (curr->flowPulses > 0)
		{
			numAxesInvolvedInMove++;        // add in C axis
		}
		else // NUKE
		{
			ARG_CS = numAxesInvolvedInMove;	// JUST A PLACE FOR A BREAKPOINT when flowPulses = 0
		}

		// handle special cases where a move may change forms (ie, VECTOR to RAPID or vice versa)
		// convert first and then let regular processing occur
		if ((motionRate == VECTOR_MOTION_RATE) && ((curr->distance > TurboModeDistance) && (curr->flags.hadAnE == FALSE)))
		{   // non-RAPID, non printing,  short move -- promote to RAPID
			// we're in turbo mode because relatively long non printing move
			motionRate = RAPID_MOTION_RATE; // force move to rapid rate
		}
		else if ((motionRate == RAPID_MOTION_RATE) && ARG_F_PRESENT)
		{   // convert to act like a VECTOR move
			motionRate = VECTOR_MOTION_RATE;  // no longer looking up speed from predefined rapid rate.
		}

		// now set up the target speed of the plethora of options:
		if (motionRate == VECTOR_MOTION_RATE)           // vector rate, use "F" from gcode
		{
			curr->speed = curr->requestedFeedrate * curr->scaleDegreesToMm; // scaling is 1.0f for linear, but allows matching feedrate to the working circum of a rotary
		}
		else if (motionRate == RAPID_MOTION_RATE)       // use the RAPID rate defined for each axis
		{
			curr->speed = MAXFLOAT; // will be scaled to the real RAPID rate later
			rateIndex = RAPID;
		}
		else if (motionRate == HOMING_MOTION_RATE)       // use the HOMING rate defined for each axis
		{
			curr->speed = MAXFLOAT; // will be scaled to the real HOMING rate later
			rateIndex = HOMING;
		}
		else if (motionRate == REHOMING_MOTION_RATE)       // use the REHOMING rate defined for each axis
		{
			curr->speed = MAXFLOAT; // will be scaled to the real REHOMING rate later
			rateIndex = REHOMING;
		}
		else if (motionRate == RASTER_MOTION_RATE)       // use the RAPID_RATE for X axis
		{
			curr->speed = _gs._laser.rasterScanFeedrate; // raster move, so special X rate
			curr->flags.rasterMove = 1;
			rateIndex = AXIS_MAX;   // really a don't care!
		}
		else if (motionRate == JOGGING_MOTION_RATE)       // use the RAPID_RATE for X axis
		{
			curr->speed = MAXFLOAT; // will be scaled to the real JOGGING rate later
			rateIndex = JOGGING;   // really a don't care!
		}
		else if (motionRate > 0.0f)    // use specified rate
		{
			curr->speed = motionRate;
		}
		else // (motionRate <= 0.0f)
		{   // something is wrong!
			barf("motionQ_addCommand(): unknown motionRate");
			return(FAIL);
		}
#ifdef GB_DEBUG_ARC
		curr->TAG = G2G3Flag;
#endif

		if (homing)
		{   // independent axis move
			curr->flags.homingMove = TRUE;
			for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
			{
				if (curr->PULSES_TO_GO[axisIndex])
					curr->unitVector[axisIndex] = 1.0f;  // lie.  each axis works independant when homing
			}
			// these will be adjusted in motionQ_execute()
			curr->distance                  = 0.0f;
			curr->startSpeedLimit           = 0.0f;
			curr->noRampSpeed               = 0.0f;
			intersectionSpeedLimit          = 0.0f;
			curr->acceleration              = 0.0f;
			curr->speed                     = motionRate;
			curr->moveTime                  = MAXFLOAT;  // really unknown, so just a LONG time
		}
		else
		{   // not homing
			for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
			{   // finish unitVector calc
				if (curr->PULSES_TO_GO[axisIndex])
				{   // axis involved in the move
					M = &Motors[axisIndex];
					curr->unitVector[axisIndex] /= curr->distance;  // finish unit vector calc

					if (motionRate < VECTOR_MOTION_RATE) // using one of the pre-stored rates
					{   // need to look up target speed
						curr->speed = fminf(curr->speed, M->RatesInUPS[rateIndex] / fabsf(curr->unitVector[axisIndex])); // fabs because just want magnitude
					}
				}
			}

			curr->acceleration = MAXFLOAT;
			curr->noRampSpeed = curr->speed;    // will be reduced as needed
			float intersectionCosTheta = 0.0f;  // dot product of last two unit vectors gives the cos of the angle between them.  ranging for -1 to 1,
												// with -1 = 180 degrees (or the two motion vectors are in the same direction to a 1 = 0 which is an about face.
			float maxAllowableSpeed = calculateIsrLimitedSpeed(curr, dominantAxisPulses, (linearPulses + rotaryPulses + curr->flowPulses), numAxesInvolvedInMove);

			for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
			{
				if (curr->PULSES_TO_GO[axisIndex])
				{   // axis involved in the move
					M = &Motors[axisIndex];
					// feedrate * axisUnitVec will be axis speed, which must be less than Axis Max, so feedrate must be less than axisMax / axisUnitVec
					curr->speed *= _CompositeMotionOverridePct;
					curr->speed = fminf(curr->speed, M->RatesInUPS[AXIS_MAX] / fabsf(curr->unitVector[axisIndex])); // fabs because just want magnitude

					if (curr->speed > maxAllowableSpeed)
					{
						curr->speed = maxAllowableSpeed;
						if (_sendingGBStringsMask & GB_STRING_FLOW) // use M797 S<mask> to enable
						{
							sprintf(_tmpStr, "THROTTLE: N:%d cs=%3.2f, mas=%3.2f d=%3.2f #a=%d dp=%ld lp=%ld rp=%ld fp=%ld ai=%d uv=%3.2f",
									curr->lineNumber, curr->speed, maxAllowableSpeed, curr->distance, numAxesInvolvedInMove, dominantAxisPulses,
									linearPulses, rotaryPulses, curr->flowPulses, axisIndex, curr->unitVector[axisIndex]);
							sendGB(_tmpStr);
						}
					}

					curr->acceleration = fminf(curr->acceleration, M->AccelerationConstant / fabsf(curr->unitVector[axisIndex])); // fabs because just want magnitude
					// want to find the angle of intersection between this line and the prior line.  dot product of the two init vectors gives
					// the cosine of the angle between them.  for this to work, the tail of both vectors needs to be at the intersection, but
					// given the unit vectors follow the lines in move order, the head of the prior is at the tail of the new, so need to negate
					// the prior vector to get tail to tail;
					intersectionCosTheta +=  -motionQ.newest->unitVector[axisIndex] * curr->unitVector[axisIndex];  // will only get used if another Q entry
					curr->noRampSpeed = fminf(curr->noRampSpeed, M->RatesInUPS[NO_RAMP] / fabsf(curr->unitVector[axisIndex]));
				}
			}
			curr->noRampSpeed = fmaxf(curr->noRampSpeed, MIN_VECTOR_SPEED);
			curr->moveTime = curr->distance / curr->speed;
			curr->perfectMoveTime = curr->moveTime; // moveTime adjusted later for accel, etc
//			curr->perfectWorldSpeed = curr->speed;

			isAutoPrimeoOrUnprimeNeeded(curr);  // update prime/unprime flags now that it is know the move will take place AND moveTIme is known

			float intersectionSinThetaOver2;
			if (motionQ_empty() || _motionQ_LookAheadDisabled || ((motionQ_numEntries() == 1) && motionQ.oldest->flags.inMotion))
			{   // only entry in queue, so will start from a standstill OR not looking ahead to the next move
				intersectionSpeedLimit = curr->noRampSpeed;
			}
			else
			{
				intersectionCosTheta = fFitWithinRange(intersectionCosTheta, -1.0f, 1.0f);  // clamp to legal cos values in case of round off errors
				// determine speed through the intersection using centripetal accel.
				intersectionSinThetaOver2 = fpu_sqrtf(fmaxf(0.0f, (1.0f - intersectionCosTheta) * 0.5f));
				float tmp = fpu_sqrtf(fmaxf(0.0f, (curr->acceleration * _MachineLimits.CentripetalAccelRadius * intersectionSinThetaOver2) / (1.0f - intersectionSinThetaOver2)));
				intersectionSpeedLimit = fmaxf(sqr(curr->noRampSpeed), tmp);
			}

			// set up the newest entry in the queue
			curr->startSpeedLimit = fminf(intersectionSpeedLimit, curr->speed);
			if (!motionQ_empty())
			{   // start no faster than the prior moves' cruising speed
				curr->startSpeedLimit = fminf(curr->startSpeedLimit, motionQ.newest->speed); // start no faster than the prior moves' cruising speed
			}
			curr->startSpeed = fminf(curr->startSpeedLimit, fpu_sqrtf(2.0f * curr->acceleration * curr->distance)); // entry speed to decel to 0

			curr->startSpeedLimit = fmaxf(curr->startSpeedLimit, curr->noRampSpeed);
			curr->startSpeed = fmaxf(curr->startSpeed, curr->noRampSpeed);
			curr->speed = fmaxf(curr->speed, curr->noRampSpeed);

			if (!motionQ_empty())
			{
				if (motionQ.newest->flags.needToUnprimeAfterMove)
				{   // if the move prior to this one is unpriming, then this move will start slow as motion will pause for the unprime
					curr->startSpeedLimit = curr->noRampSpeed;
					curr->startSpeed = curr->noRampSpeed;
				}
			}
#ifdef GB_DEBUG_ARC
			sprintf(_tmpStr,"A:%2d) [%5.2f, %5.2f] S=%4.3f C=%4.3f isl=%4.3f a=%4.3f nr=%4.3f ssl=%4.3f ss=%4.3f s=%4.3f",
					curr->TAG, curr->unitVector[0], curr->unitVector[1],
					intersectionSinThetaOver2, intersectionCosTheta, intersectionSpeedLimit, curr->acceleration,
					curr->noRampSpeed, curr->startSpeedLimit, curr->startSpeed, curr->speed);
			sendGB(_tmpStr);//announce the fudge factor being set.
#endif
		}

		//=====================

		for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
		{
			if (curr->PULSES_TO_GO[axisIndex])
			{   // axis involved in the move
				M = &Motors[axisIndex];
#ifdef GB_HIDDEN_WARNINGS
				int add_mcode_and_finish_this; // do we need a modulo 360 degs on the rotaty stuff (and be careful with "POSITION" at the execute level)
#endif //GB_HIDDEN_WARNINGS
				if (curr->flags.homingMove) // &no_home_sensor && enableAbsolute_mode
				{
					M->Q_POSITION = 0;
					M->Q_LastRequestedPositionInUnits = 0.0f;
				}
				else if (M->AxisType == ROTARY)
				{   // infinite spin, but limit view of position to 0 to 360
					M->Q_LastRequestedPositionInUnits = fmodf(M->Q_LastRequestedPositionInUnits, 360.0f);
					M->Q_POSITION = destinationInPulses[axisIndex] % M->PulsesPerRevolution;
				}
				else
				{
					M->Q_POSITION = destinationInPulses[axisIndex];
				}
			}
			else
			{
				curr->unitVector[axisIndex] = 0.0f;
			}
		}

		// at this point, the local entry has been filled and a real move will occur, so move the local entry into the queue
		curr =  motionQ_nextAvailable();    // get pointer in actual queue
		// safer: 
		memcpy((byte *)curr, (byte *)&localMotionQEntry, offsetof(motionEntryStruct, index));
		//NUKE memcpy((byte *)curr, (byte *)&localMotionQEntry, sizeof(motionEntryStruct) - MOTION_ENTRY_STRUCT_BYTES_TO_PRESERVE); // copy data only, not linked list pointers

		LastCmdQueIndexAddedToMotionQ = curr->cmdQueIndex;
		motionQ_update();
		if (_sendingGBStringsMask & GB_STRING_MOTION) // use M797 S<mask> to enable
		{
			MotorStructure *M = &Motors[M_Z];
			sprintf(_tmpStr,"oZ:  Z=%4.3f  %s  POS=%ld  QPOS=%ld  LRP=%4.3f  P2G=%ld  D=%d", ARG_Z, _IncrementalMove==TRUE ? "inc" : "abs",
							M->POSITION, M->Q_POSITION, M->Q_LastRequestedPositionInUnits, curr->PULSES_TO_GO[M_Z], curr->direction[M_Z]);
			sendGB(_tmpStr);  sendGB(" ");
		}
		if ((_sendingGBStringsMask & GB_STRING_CYL_INFO) && (_sendingGBStringsSubMask & GB_STRING_CYL_INFO_SUBMASK_MQ_INFO)) // use M797 S<mask> to enable
		{
			MotorStructure *M = &Motors[M_B];
			sprintf(_tmpStr,"B: B=%4.3f %s POS=%ld QPOS=%ld LRP=%4.3f P2G=%ld D=%d S:%4.3f", ARG_B, _IncrementalMove==TRUE ? "inc" : "abs",
							M->POSITION, M->Q_POSITION, M->Q_LastRequestedPositionInUnits, curr->PULSES_TO_GO[M_B], curr->direction[M_B], M->scaleDegreesToMm);
			sendGB(_tmpStr);
		}

		if (motionQ_numEntries() == 1)
		{   // only entry in the queue, so decide whether to immediately process it or wait before starting it
			if (!AllMotionMotorsAreEnabled())
			{
				EnableAllMotionMotors();
				motionQ_setCountdownDelayToExecuteMove(MOTIONQ_WAIT_FOR_MOTOR_EN_TIME_MS);
			}
			else if (_motionQ_ForceQtoEmpty)
				motionQ_executeMove();  // no need to wait, just execute
			else
				motionQ_setCountdownDelayToExecuteMove(MOTIONQ_FIRST_ENTRY_AGE_TIME_MS); //  will give time for other commands to get in the queue
		}
	}
	else
	{   // no motion axis moving ... not adding to motionQ, but need to pass important info back to the Q
		if ((localMotionQEntry.flags.lastInSequence) && !motionQ_empty())
		{   // force prior command to be last in seq so deferred commands will link properly
			motionQ.newest->flags.lastInSequence = TRUE;
		}
	}
#ifdef GB_FUNC_ADD_COMMAND_PIN
	GB_FUNC_ADD_COMMAND_CLEAR;
#endif //GB_FUNC_ADD_COMMAND_PIN
	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_update(void)
{
	PCHAR();//check to see if we should be sending a charcter out uart3 tothe co2 laser
	float decelStartSpeed;
	float accelEndSpeed;
	motionEntryStruct *curr = motionQ.newest;

	if (motionQ_numEntries() > 1)
	{   // need at least 2 Q entries in order to update speeds.

		// reverse pass to make sure start speeds are low enough to ensure motion can stop in time at the end of a series of moves
		while (curr->prev != motionQ.oldest)
		{
			if (curr->prev->flags.needToUnprimeAfterMove)
			{   // prior move should be stopping, so need to restart motion from "noramp"
				curr->startSpeed = curr->noRampSpeed;
				curr->startSpeedLimit = curr->noRampSpeed;
			}
			if (curr->flags.needToPrimeBeforeMove)
			{   // by definition, starting from a "stopped" situation
				curr->startSpeed = curr->noRampSpeed;
				curr->startSpeedLimit = curr->noRampSpeed;
			}

			curr = curr->prev;  // working backwards in time through the queue
			// determine start velocity if you assume accelerating in reverse from exit velocity
			if (curr->startSpeed != curr->startSpeedLimit)
			{
				decelStartSpeed = fpu_sqrtf(sqr(curr->next->startSpeed) + 2.0f * curr->acceleration * curr->distance);
				curr->startSpeed = fminf(decelStartSpeed, curr->startSpeedLimit);
			}
		}

		// forward pass

#ifdef GB_HIDDEN_WARNINGS
#warning "motionQ.planned;  start at the last planned location -- SHOULD REINSTATE WHEN SAFE" // save processing time, but make sure to invalidate for abort cases
		//GB XXX ATLANTA curr = motionQ.planned; // start at the last planned location
#endif //GB_HIDDEN_WARNINGS

		curr = motionQ.oldest;
		if (curr->flags.inMotion)  // if already moving, cannot update it's speeds
			curr = curr->next;

		while (curr != motionQ.newest)
		{
			if (curr->startSpeed < curr->next->startSpeed)
			{   // allow things to accelerate up to the start speed of the next entry
				accelEndSpeed = fpu_sqrtf(sqr(curr->startSpeed) + 2.0f * curr->acceleration * curr->distance);
				if (accelEndSpeed < curr->next->startSpeed)
				{
					curr->next->startSpeed = accelEndSpeed;
					motionQ.planned = curr->next;
				}
			}
			curr = curr->next;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_primeComplete(void)
{
	motionQ.oldest->flags.needToPrimeBeforeMove = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_unprimeComplete(void)
{
	motionQ.oldest->flags.needToUnprimeAfterMove = FALSE;
	motionQ.oldest->flags.needToUnprimeDuringMove = FALSE;
	DominantAxisPtr->residualUnprimeTime = 0;
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_executeMove(void)
{
#ifdef GB_FUNC_EXECUTE_MOVE_PIN
	GB_FUNC_EXECUTE_MOVE_SET;
#endif //GB_FUNC_EXECUTE_MOVE_PIN

	float startSpeed;
	float endSpeed;
	float speed;
	float crossoverDist;
	float accelDistance;
	float decelDistance;

	int axisIndex;
	float axisScale;
	MotorStructure *M;
	motionEntryStruct *curr;
	float accelTime = 0.0f;
	float cruiseTime = 0.0f;
	float decelTime = 0.0f;
	int i;

	if (motionQ_empty())
	{
		barf("motionQ_executeMove(): motionQ_empty");
		return;
	}
	if (motionQ.oldest->flags.needToPuffBeforeMove)
	{
		puff();
		motionQ.oldest->flags.needToPuffBeforeMove = 0;
	}

	curr = motionQ.oldest;

#ifdef GB_DEBUG_MOTION_Q    // allow control over data dependent breakpoints
	{
		float matchingArgD = 0.0f;  // change this value as needed
		if (curr->debugArgDValue == matchingArgD)
			matchingArgD = INVALID_ARG_VALUE; // stick breakpoint here
	}
#endif

	_numMovingAxes = 0;

#ifdef GB_MQ_EXECUTE_PIN
	GB_MQ_EXECUTE_TOGGLE;
#endif //GB_MQ_EXECUTE_PIN

	for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
	{
		if (curr->PULSES_TO_GO[axisIndex])
		{   // axis involved in the move
			M = &Motors[axisIndex];
			_movingAxesPtrs[_numMovingAxes++] = M;  // add this motor to list of moving axis
#ifdef GB_DEBUG_MOTION_Q
			M->lineNumber = curr->lineNumber;
			M->debugArgDValue = curr->debugArgDValue;
#endif
			M->PULSES_TO_GO = curr->PULSES_TO_GO[axisIndex];
			M->PULSES_IN_MOVE = curr->PULSES_TO_GO[axisIndex];
			M->DIRECTION = curr->direction[axisIndex];
			// actual direction bits will be changed later.   if done here, there is not enough hold time for the last step generated for
			// the last move and a direction change for this new move (as discovered in 4.101a)
		}
	}

	if (curr->flags.homingMove)  // - HOMING
	{   // independent axis move -- need to correct/generate each axis rates and ramps
		for (i=0; i<_numMovingAxes; i++)
		{
			M = _movingAxesPtrs[i];

			M->SearchingForHomeSensor = TRUE;
			M->startVelocity = M->RatesInUPS[NO_RAMP];
			M->endVelocity = M->RatesInUPS[NO_RAMP];
			M->currentVelocity = M->startVelocity;

			if (curr->speed == RAPID_MOTION_RATE)
				M->cruiseVelocity = M->RatesInUPS[RAPID];
			else if (curr->speed == HOMING_MOTION_RATE)
				M->cruiseVelocity = M->RatesInUPS[HOMING];
			else if (curr->speed == REHOMING_MOTION_RATE)
				M->cruiseVelocity = M->RatesInUPS[REHOMING];
			else if (curr->speed == JOGGING_MOTION_RATE)
				M->cruiseVelocity = M->RatesInUPS[JOGGING];
			else
				M->cruiseVelocity = curr->speed;
			// make sure what ever rate is used, there's is enough time to decelerate
			// 2a(X-Xo) = V^2 - Vo^2   with Vo = endVelocity; Xo=0; X=hysteresis
			// V = sqrt(2aX + Vo^2)
			float stoppableRate = fpu_sqrtf(2.0f * M->AccelerationConstant * M->HomeHysteresisInUnits + sqr(M->endVelocity));
			M->cruiseVelocity = fminf(M->cruiseVelocity, stoppableRate);
			M->cruiseVelocitySquared = sqr(M->cruiseVelocity);
			accelDistance = (M->cruiseVelocitySquared - sqr(M->startVelocity)) / (2.0f *  M->AccelerationConstant);
			decelDistance = (M->cruiseVelocitySquared - sqr(M->endVelocity)) / (2.0f * M->AccelerationConstant);
			axisScale = 1.0f;
			curr->acceleration = M->AccelerationConstant;   // all axes independent, so use axis accel limit, not vector

			M->AccelerationPulses = axisScale * accelDistance * M->PulsesPerUnit;
			M->DecelerationPulses = axisScale * decelDistance * M->PulsesPerUnit;
			M->AccelerationScaled = axisScale * curr->acceleration;
			M->AccelerationScaledTimes2 = 2.0f * M->AccelerationScaled;

			M->currentVelocity = calcVelocity(M, M->AccelerationPulses - (M->PULSES_IN_MOVE - M->PULSES_TO_GO));
			CalculateTimerControls(M);
		}
	}
	else // (curr->flags.homingMove == 0)  -- NOT HOMING
	{
		// now figure out the intersection of the accel and decel side of this move and see if there's room to
		// run at full speed.  using point slope form for the accel and decel "lines", each with a known point and slope
		// m=2A and (0, startSpeed) for the accel and m=-2A and (dist, EndSpeed) for the decel side.  solving
		// for the intersection distance (Xi), yields
		// Xi = 0.5 * (dist + ((EndSpeed - StartSpeed) / 2A))
		// if Xi < 0, then only decelerating
		// elseif Xi > distance, then only accelerating
		// elseif accelDist <= Xi, then normal trapeziod of accel, cruise, decel
		// else not enough room to accel to speed and decel back to endSpeed, so need to clip target speed
		startSpeed = curr->startSpeed;
		endSpeed = (curr==motionQ.newest) ? MIN_VECTOR_SPEED : curr->next->startSpeed;
		endSpeed = fmaxf(endSpeed, curr->noRampSpeed);
		float currAccelerationTimesTwo = 2.0f * curr->acceleration;

		crossoverDist = 0.5f * (curr->distance + ((sqr(endSpeed) - sqr(startSpeed)) / currAccelerationTimesTwo));

		if (crossoverDist < 0)
		{   // decel only
			speed = fpu_sqrtf(sqr(endSpeed) + (currAccelerationTimesTwo * curr->distance));
			accelDistance = 0.0f;
			decelDistance = curr->distance;
			decelTime = (speed - endSpeed) / curr->acceleration;  // for metrics and unprime pulse calc
		}
		else if (crossoverDist > curr->distance)
		{   // accel only
			speed = fpu_sqrtf(sqr(startSpeed) + (currAccelerationTimesTwo * curr->distance));
			accelDistance = curr->distance;
			decelDistance = 0.0f;
			accelTime = (speed - startSpeed) / curr->acceleration;  // for metrics and unprime pulse calc
		}
		else
		{
			speed = curr->speed;
			accelDistance = (sqr(speed) - sqr(startSpeed)) / currAccelerationTimesTwo;
			if (accelDistance > crossoverDist)
			{   // not enough room, so reduce speed
				speed = fpu_sqrtf(sqr(startSpeed) + (currAccelerationTimesTwo * crossoverDist));
				accelDistance = crossoverDist;
				decelDistance = (curr->distance) - crossoverDist;
				accelTime = (speed - startSpeed) / curr->acceleration;   // for metrics and unprime pulse calc
				decelTime = (speed - endSpeed) / curr->acceleration;     // for metrics and unprime pulse calc
			}
			else
			{   // enough room to accel, cruise, and decel
				decelDistance = (sqr(speed) - sqr(endSpeed)) / currAccelerationTimesTwo;
				accelTime = (speed - startSpeed) / curr->acceleration;  // for metrics and unprime pulse calc
				cruiseTime = (curr->distance - (accelDistance + decelDistance)) / speed;   // for metrics and unprime pulse calc
				decelTime = (speed - endSpeed) / curr->acceleration;    // for metrics and unprime pulse calc
			}
		}

		curr->moveTime = (accelTime + cruiseTime + decelTime);   // for metrics and unprime pulse calc

#ifdef GB_DEBUG_ARC
		sprintf(_tmpStr,"E:%2d) d=%4.3f t=%4.3f cd=%4.3f s=%4.3f at=%4.3f ct=%4.3f dt=%4.3f",
				curr->TAG, curr->distance, curr->moveTime, crossoverDist,
				curr->speed, accelTime, cruiseTime, decelTime);
		sendGB(_tmpStr);//announce the fudge factor being set.
#endif

		// only need to set up dominant axis for controlling acceleration/deceleration
		DominantAxisPtr = &Motors[curr->flags.dominantAxis];
		axisScale = fabsf(curr->unitVector[curr->flags.dominantAxis]);
		_CurrentRequestedFeedrateInMmPerSec = curr->requestedFeedrate;  // last ARG_F passed along through the motionQ (for reportXYZ)
		_ActualVectorRateInMmPerSec = (curr->moveTime > 0.0) ? (curr->distance / curr->moveTime) / curr->scaleDegreesToMm : 0.0f; // avgSpeed (for ReportXYZ)

		if ((curr->flags.hadAnE) || (curr->flags.rasterMove))
		{   // this might muck with axis C
			CheckForNewFlowRate(curr);  // this call must be after moveTime and distance are known
		}

		DominantAxisPtr->startVelocity = axisScale * startSpeed;
		DominantAxisPtr->cruiseVelocity = ClampToAxisLimitsInUPS(DominantAxisPtr, axisScale * speed);    // GB XXX SHOULD NOT CLAMP
		DominantAxisPtr->cruiseVelocitySquared = sqr(DominantAxisPtr->cruiseVelocity);
		DominantAxisPtr->endVelocity = axisScale * endSpeed;
		DominantAxisPtr->AccelerationPulses = axisScale * accelDistance * DominantAxisPtr->PulsesPerUnit;
		DominantAxisPtr->DecelerationPulses = axisScale * decelDistance * DominantAxisPtr->PulsesPerUnit;
		DominantAxisPtr->AccelerationScaled = axisScale * curr->acceleration;
		DominantAxisPtr->AccelerationScaledTimes2 = 2.0f * DominantAxisPtr->AccelerationScaled;
		DominantAxisPtr->currentVelocity = calcVelocity(DominantAxisPtr, DominantAxisPtr->AccelerationPulses - (DominantAxisPtr->PULSES_IN_MOVE - DominantAxisPtr->PULSES_TO_GO));

		if ((curr->flags.needToUnprimeAfterMove) && (getCurrentUnPrimeTimeMs(currentOutboxPtr) < 0))
		{   // negative preamble time, so unprime before move ends
			float unprimeTime = (float)getCurrentUnPrimeTimeMs(currentOutboxPtr) / -1000.0f;  // convert ms to sec
			curr->flags.needToUnprimeDuringMove = TRUE;
			DominantAxisPtr->residualUnprimeTime = 0;

			float dist;
			if (unprimeTime <= decelTime)
			{   // unprime during decel
				dist = endSpeed * unprimeTime + (0.5 * DominantAxisPtr->AccelerationScaled * sqr(unprimeTime));
				DominantAxisPtr->unprimePulseToGo = dist * DominantAxisPtr->PulsesPerUnit;
			}
			else if (unprimeTime <= (cruiseTime + decelTime))
			{   // unprime during cruise
				dist = axisScale * speed * (unprimeTime - decelTime);
				DominantAxisPtr->unprimePulseToGo = DominantAxisPtr->DecelerationPulses + dist * DominantAxisPtr->PulsesPerUnit;
				DominantAxisPtr->unprimePulseToGo = imin(DominantAxisPtr->unprimePulseToGo, DominantAxisPtr->PULSES_IN_MOVE); // safety net for roundoff
			}
			else if (unprimeTime <= curr->moveTime)
			{   // unprime during accel
				float tmpTime = curr->moveTime - unprimeTime;
				dist = axisScale * startSpeed * tmpTime + (0.5 * DominantAxisPtr->AccelerationScaled * sqr(tmpTime));
				DominantAxisPtr->unprimePulseToGo = DominantAxisPtr->PULSES_IN_MOVE - (dist * DominantAxisPtr->PulsesPerUnit);
				DominantAxisPtr->unprimePulseToGo = imin(DominantAxisPtr->unprimePulseToGo, DominantAxisPtr->PULSES_IN_MOVE); // safety net for roundoff
			}
			else
			{   // unprime is longer than move, so will have residual wait time.
				DominantAxisPtr->unprimePulseToGo = DominantAxisPtr->PULSES_IN_MOVE;
				DominantAxisPtr->residualUnprimeTime = imax(0, (int)roundf((unprimeTime - curr->moveTime) * 1000.0f));  // extra time that will occur after the move
				//GB XXX  if < 1ms drop ...
			}

			if (DominantAxisPtr->unprimePulseToGo == 0)
			{   // was too small to issue before the end of the move, so change to be coincident with the move end
				curr->flags.needToUnprimeDuringMove = FALSE;
				DominantAxisPtr->residualUnprimeTime = 0;
			}
		}

#ifdef COLLECT_METRICS
		if (curr->flags.hadAnE)
		{
			_metrics.printing.time += curr->moveTime;
			_metrics.printing.dist += curr->distance;
//			if (curr->perfectWorldSpeed > 0.0f)
//				_metrics.printing.perfectTime += (curr->distance / curr->perfectWorldSpeed);
			_metrics.printing.perfectTime += curr->perfectMoveTime;
		}
		else
		{
			_metrics.nonPrinting.time += curr->moveTime;
			_metrics.nonPrinting.dist += curr->distance;
//			if (curr->perfectWorldSpeed > 0.0f)
//				_metrics.nonPrinting.perfectTime += (curr->distance / curr->perfectWorldSpeed);
			_metrics.nonPrinting.perfectTime += curr->perfectMoveTime;
		}
		_metrics.accel.time += accelTime;
		_metrics.accel.dist += accelDistance;
		_metrics.cruise.time += cruiseTime;
		_metrics.cruise.dist += (curr->distance - (accelDistance + decelDistance));
		_metrics.decel.time += decelTime;
		_metrics.decel.dist += decelDistance;

	_metrics.numMoves++;
#endif //COLLECT_METRICS

		if (curr->flags.rasterMove)
		{   // clear counters and set up for a raster scan --
			_gs._laser.rasterPulsesPerDotCounter   = _gs._laser.rasterPulsesPerDot;     // down counter so load start value
			_gs._laser.rasterFrontPorchDotsCounter = _gs._laser.rasterFrontPorchDots;   // down counter so load start value
			_gs._laser.rasterizeCurrentMove = TRUE; // NEEDS TO BE SET before calling CheckForNewFlowRate()

			if (_gs._laser.localControl)
			{   // setup raster timing
				InitEncoderTimer5();
			}
		}

		for (i=0; i<_numMovingAxes; i++)
		{
			M = _movingAxesPtrs[i];
			if (M != DominantAxisPtr)
			{
				M->Dominant2AxisPulseRatio = (float)DominantAxisPtr->PULSES_IN_MOVE / (float)M->PULSES_IN_MOVE;
				M->AxisFirstPulseOffset = ((float)DominantAxisPtr->PULSES_IN_MOVE - ((float)(M->PULSES_IN_MOVE - 1) * M->Dominant2AxisPulseRatio)) / 2.0f;
			}
		}

		if ((_sendingGBStringsMask & GB_STRING_CYL_INFO) && (_sendingGBStringsSubMask & GB_STRING_CYL_INFO_SUBMASK_MQ_SPEED)) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"X=%7.2f B=%7.2f ssl=%5.2f ss=%5.2f cs=%5.2f es=%5.2f d=%5.2f cd=%5.2f mT=%5.2f aT=%5.2f cT=%5.2f dT=%5.2f E%d S=%3.2f R=%2.1f R=%d C:%ld",
					// ABS
					//(Motors[M_X].POSITION + (Motors[M_X].DIRECTION * Motors[M_X].PULSES_TO_GO)) / Motors[M_X].PulsesPerUnit,
					//(Motors[M_B].POSITION + (Motors[M_B].DIRECTION * Motors[M_B].PULSES_TO_GO)) / Motors[M_B].PulsesPerUnit,
					// REL
					(Motors[M_X].DIRECTION * Motors[M_X].PULSES_TO_GO) * Motors[M_X].UnitsPerPulse,
					(Motors[M_B].DIRECTION * Motors[M_B].PULSES_TO_GO) * Motors[M_B].UnitsPerPulse,
					curr->startSpeedLimit, startSpeed, speed, endSpeed, curr->distance, crossoverDist, curr->moveTime, accelTime, cruiseTime,
					decelTime, (curr->flags.hadAnE?1:0), curr->scaleDegreesToMm, _LastExtrusionRate, currentOutboxPtr->ExtrusionControl.ExtrudeFeedRate,
					Motors[M_C].PULSES_IN_MOVE);

			sendGB(_tmpStr);
		}
		CalculateTimerControls(DominantAxisPtr); // coordinated move

		if (curr->flags.needToPrimeBeforeMove)
		{
			PrimeTimer = abs(getCurrentPrimeTimeMs(currentOutboxPtr)); // set the timer to hold motion until prime finishes.
			if (PrimeTimer == 0)
			{   // special case, no countdown timer to wait for... need to cleanup flags
				motionQ_primeComplete();
			}
		}
#ifdef GB_DEBUG_MOTION_Q
		float matchingArgD = 0.0f;  // change this value as needed
#ifdef GB_DARG_MATCH_PIN
		if (M->debugArgDValue == matchingArgD)
			pinSet(GB_DARG_MATCH_PIN);
		else
			pinClear(GB_DARG_MATCH_PIN);
#endif
		if (M->debugArgDValue == matchingArgD)
			matchingArgD = INVALID_ARG_VALUE; // stick breakpoint here
#endif
	}

	// set the direction bits here .... after calculations to ensure enough hold time margin after last step of the last
	// move.   should still provide enough setup time until the first pulse of the next move.
	for (i=0; i<_numMovingAxes; i++)
	{
		M = _movingAxesPtrs[i];
		if (M->DIRECTION == 1)
			outputDirectionBit(&M->Direction, DIRECTION_FORWARD);
		else if (M->DIRECTION == -1)
			outputDirectionBit(&M->Direction, DIRECTION_REVERSE);
		else // should never get here
			barf("motionQ_executeMove(): invalid direction");
	}

#ifdef GB_TX_LINENUM
	sprintf(_tmpStr, "L%d:%d", curr->line, motionQ_numEntries());
	sendstringCr(_tmpStr);
#endif


#ifdef GB_UNPRIME_PIN
	pinClear(GB_UNPRIME_PIN);
#endif
#ifdef GB_PRIME_PIN
	pinClear(GB_PRIME_PIN);
#endif

#ifdef COLLECT_METRICS
	// record mQ utilization.
	if (_motionQ_ForceQtoEmpty && motionQ_numEntries() == 1)
		_metrics.motionQ_entriesWhenExecuting[0]++;
	else
		_metrics.motionQ_entriesWhenExecuting[motionQ_numEntries()]++;
#endif
	if (_sendingGBStringsMask & GB_STRING_FLOW)  // use M797 S<mask> to enable
	{
		sprintf(_tmpStr,"N:%5ld X=%7.2f Y=%7.2f ssl=%5.2f ss=%5.2f cs=%5.2f es=%5.2f d=%5.2f cd=%5.2f mT=%5.2f aT=%5.2f cT=%5.2f dT=%5.2f E%d R=%2.1f R=%u C:%ld P=%lu A=%lu",
				// ABS
				//(Motors[M_X].POSITION + (Motors[M_X].DIRECTION * Motors[M_X].PULSES_TO_GO)) / Motors[M_X].PulsesPerUnit,
				//(Motors[M_B].POSITION + (Motors[M_B].DIRECTION * Motors[M_B].PULSES_TO_GO)) / Motors[M_B].PulsesPerUnit,
				// REL(),
				getCurrentExecutingLineNumber(),
				(Motors[M_X].DIRECTION * Motors[M_X].PULSES_TO_GO) * Motors[M_X].UnitsPerPulse,
				(Motors[M_Y].DIRECTION * Motors[M_Y].PULSES_TO_GO) * Motors[M_Y].UnitsPerPulse,
				curr->startSpeedLimit, startSpeed, speed, endSpeed, curr->distance, crossoverDist, curr->moveTime, accelTime, cruiseTime,
				decelTime, (curr->flags.hadAnE?1:0), _LastExtrusionRate, currentOutboxPtr->ExtrusionControl.ExtrudeFeedRate,
				Motors[M_C].PULSES_IN_MOVE, _Dominant.TimerPSC, _Dominant.TimerARR);

		sendGB(_tmpStr);
	}

	if (PrimeTimer == 0)
	{
		StartMove(); // set any necessary flags, loads PSC/ARR and start timers
	}

#ifdef GB_FUNC_EXECUTE_MOVE_PIN
	GB_FUNC_EXECUTE_MOVE_CLEAR;
#endif //GB_FUNC_EXECUTE_MOVE_PIN
}

////////////////////////////////////////////////////////////////////////////////

void cleanupAfterAbortFinishes(void)
{
	motionQ_init();
	_abortFinisedNeedToResetProcessSynchronously = TRUE;
#ifdef HYDRA_DIAGS
	if (_diagsEnabled)
	{
		diagsExit(TRUE);
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////

motionEntryStruct *changeMotionQToPauseAtNextMoveBoundary(void)
{   // return a pointer to the oldest motionQ entry in which all axes can be brought to a safe stop
	//
	// the "stoppable" move is found my successively running through Q with the assuming all moves are going to
	// decel from either the current speed (for current move) or from the newly calculated exit speed of the
	// prior move.
	//
	// if the stoppable move is the current move underway, then a lot a decisions/work is needed to alter the move underway.
	//    if the current move can be stopped within the  remaining distance of the move
	//        if (accelerating) && (can be stopped after hitting cruiseSpeed)
	//            let it reach cruise
	//            calc decel point in order to exit move at to noRamp
	//        else if (accelerating )
	//            immediately start cruising
	//            calc decel point in order to exit move at to noRamp
	//        else if (decelerating)
	//            nothing to do, already slowing down
	//        else (cruising)
	//           calc decel point in order to exit move at to noRamp
	//    else  // cannot fully stop move underway, so will stop another Q entry
	//         set current move to immediately decel

	motionEntryStruct *curr = motionQ.oldest;

	float distance, startSpeed, decelDistance;
	float exitSpeed = 0.0f;

	while (1)
	{
		if (curr == motionQ.oldest)
		{
			distance = curr->distance * ((float)DominantAxisPtr->PULSES_TO_GO / (float)DominantAxisPtr->PULSES_IN_MOVE);
			startSpeed = DominantAxisPtr->currentVelocity;
		}
		else
		{   // not oldest, so full moves in play
			distance = curr->distance;  // full move
			startSpeed = exitSpeed; // same as exit speed of prior move
		}

		// calculate the slowest possible exit speed assume decel were to start ASAP
		float tmp = sqr(startSpeed) - 2.0f * curr->acceleration * distance;
		if (tmp < sqr(curr->noRampSpeed))
			exitSpeed = curr->noRampSpeed;  // prevent going too slow
		else
			exitSpeed = fpu_sqrtf(tmp);

		if ((exitSpeed <= curr->noRampSpeed) || (curr == motionQ.newest))
		{   // the current move will be able to (or have to) stop
			break;
		}
		else
		{   // haven't stopped yet, so move to the next entry in the Q
			if (curr == motionQ.oldest)
			{   // to keep things simple... if current move underway cannot stop in time, the just let that move complete
				// as is and plan to stop with successive moves from there (in other words, don't alter that move)
				exitSpeed = curr->next->startSpeed;
			}
			curr = curr->next;
		}
	}

	// at this point, we have a pointer (curr) that is the move that will stop.   now need to adjust the exist speed of the current (oldest)
	// move based on when (if) it needs to slow down.

	if (curr == motionQ.oldest)
	{   // special case to stop move currently underway
		// need to fix current move if it's needed as part of slowdown....

		float axisScale = fabsf(curr->unitVector[curr->flags.dominantAxis]);
		// stopping at something other than oldest move AND more the 2 moves in Q
		if (DominantAxisPtr->AccelerationPulses > (DominantAxisPtr->PULSES_IN_MOVE - DominantAxisPtr->PULSES_TO_GO))
		{   // still accelerating
			// if possible to still stop once cruiseSpeed is reached, the let accel cont
			// otherwise, make current speed the cruise speed and go from there.

			decelDistance = (sqr(curr->speed) - sqr(curr->noRampSpeed)) / (2.0f * curr->acceleration); //curr->speed is target cruising speed
			int decelPulses = axisScale * decelDistance * DominantAxisPtr->PulsesPerUnit;

			if ((decelPulses <= (DominantAxisPtr->PULSES_IN_MOVE - DominantAxisPtr->AccelerationPulses)) && (decelPulses < DominantAxisPtr->PULSES_TO_GO))
			{   // will be able to stop after accellerating to cruise
				DominantAxisPtr->DecelerationPulses = decelPulses;
			}
			else
			{   // stop accelerating NOW and treat current speed as cruise and slow down from there
				DominantAxisPtr->AccelerationPulses = 0;    // stop accel
				decelDistance = (sqr(DominantAxisPtr->currentVelocity) - sqr(curr->noRampSpeed)) / (2.0f * curr->acceleration); // use current speed as cruise speed
				DominantAxisPtr->DecelerationPulses = imin(DominantAxisPtr->PULSES_TO_GO, (int)(axisScale * decelDistance * DominantAxisPtr->PulsesPerUnit));
				DominantAxisPtr->cruiseVelocity = DominantAxisPtr->currentVelocity;
			}
		}
		else if (DominantAxisPtr->DecelerationPulses > DominantAxisPtr->PULSES_TO_GO)
		{   // decelerating already -- so slowing down, so nothing new can be done
			;
		}
		else // cruising
		{   // redo decel pulses to decel to new exitSpeed
			decelDistance = (sqr(DominantAxisPtr->currentVelocity) - sqr(curr->noRampSpeed)) / (2.0f * curr->acceleration); //since we're cruising, current speed is by def the cruisingSpeed.
			DominantAxisPtr->DecelerationPulses = imin(DominantAxisPtr->PULSES_TO_GO, (int)(axisScale * decelDistance * DominantAxisPtr->PulsesPerUnit));
			DominantAxisPtr->cruiseVelocity = DominantAxisPtr->currentVelocity;
		}
		DominantAxisPtr->cruiseVelocitySquared = sqr(DominantAxisPtr->cruiseVelocity);
		DominantAxisPtr->endVelocity = DominantAxisPtr->RatesInUPS[NO_RAMP];
	}
	// insert a pause in the queue
	if (curr != motionQ.newest)
	{   // setup the move after the stop to start at a slow speed
		curr->next->startSpeed = curr->next->noRampSpeed;
	}
	return(curr);
}

////////////////////////////////////////////////////////////////////////////////

void fixUnprimePrimeForPause(motionEntryStruct *stoppableEntry)
{
	if (stoppableEntry->flags.hadAnE)
	{   // since motion will stop, force an unprime and then possibly a prime on the next move
		if (stoppableEntry == motionQ.oldest)
		{   // stopping the current in-process move - need to adjust a few things mid flight
			if (stoppableEntry->flags.unprimeIssued == FALSE)
			{   // not unprimed yet; force to unprime after the move
				DominantAxisPtr->unprimePulseToGo = 0;
				stoppableEntry->flags.needToUnprimeAfterMove = TRUE;
				stoppableEntry->flags.needToUnprimeDuringMove = FALSE;
			}
			else
			{
				;   // unprime already issued during move, so no need to change anything
			}
		}
		else
		{   //  yet to be started move, so just adjust flags
			stoppableEntry->flags.needToUnprimeAfterMove = TRUE;
			stoppableEntry->flags.needToUnprimeDuringMove = FALSE;
		}
	}

	if (stoppableEntry != motionQ.newest)
	{   // fix up remaining part of Q (after the pause)
		// need to check the 'next' move after the stoppable one to adjust it's need to reprime
		if (stoppableEntry->next->flags.hadAnE)
		{   // inserting a pause before a printing move, so will need to start with a prime
			stoppableEntry->next->flags.needToPrimeBeforeMove = TRUE;
		}
		stoppableEntry->next->startSpeedLimit = stoppableEntry->next->noRampSpeed;
		stoppableEntry->next->startSpeed = stoppableEntry->next->noRampSpeed;
	}
}

////////////////////////////////////////////////////////////////////////////////

void fixUnprimePrimeForAbort(motionEntryStruct *stoppableEntry)
{
	if (stoppableEntry->flags.hadAnE)
	{   // since motion will stop, force an unprime and then possibly a prime on the next move
		if (stoppableEntry == motionQ.oldest)
		{   // stopping the current in-process move - need to adjust a few things mid flight
			if (stoppableEntry->flags.unprimeIssued == FALSE)
			{   // not unprimed yet; force to unprime after the move
				DominantAxisPtr->unprimePulseToGo = 0;
				stoppableEntry->flags.needToUnprimeAfterMove = TRUE;
				stoppableEntry->flags.needToUnprimeDuringMove = FALSE;
			}
			else
			{
				;   // unprime already issued during move, so no need to change anything
			}
		}
		else
		{   //  yet to be started move, so just adjust flags
			stoppableEntry->flags.needToUnprimeAfterMove = TRUE;
			stoppableEntry->flags.needToUnprimeDuringMove = FALSE;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean stopHomingMove(void)
{
	MotorStructure *M;
	int decelPulses;
	boolean axisMoving = FALSE;

	if (motionQ.oldest->flags.homingMove)
	{   // homing, should be only move in queue
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors
			boolean axisWasInvolvedInMove = (M->PULSES_IN_MOVE > 0);
			if (AxisIsMoving(M))
			{
				decelPulses = (int)((fabsf(sqr(M->currentVelocity) - sqr(M->RatesInUPS[NO_RAMP])) / (2.0f * M->AbortDecelerationRate)) * M->PulsesPerUnit);
				M->PULSES_TO_GO = imin(decelPulses, M->PULSES_TO_GO);
				M->PULSES_IN_MOVE = M->PULSES_TO_GO;
				M->DecelerationPulses = M->PULSES_TO_GO;
				M->AccelerationPulses = 0;
				if (M->PULSES_TO_GO == 0)
				{
					DisableMotionTimer(M->TimerBase);
				}
				else
				{
					axisMoving = TRUE;
				}
			}
			if (axisWasInvolvedInMove)
			{   // axis had been part of the homing sequence (whether it finished moving or not)
				M->POSITION = M->HomeDestinationInPulses - (M->PULSES_TO_GO * M->DIRECTION); // force final position after move to be HomeDestinationInPulses.
				if (M->Axis == M_Z)
				{
					ToolOffsets[0].ToolLength = Motors[M_Z].HomeDestinationInUnits;
				}
			}
		}
	}
	return(axisMoving);
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_pauseMotionWhenSafe(void)
{
	// goal is to insert a pause in the motion as soon as possible, but between moves
	// while also inserting any needed unprime/prime commands to stop/start flow
	//
	// if no moves in the Q
	//    then already stopped
	// else if homing,
	//    then abort (gracefully) the move and stop (rest of queue should be empty and/or stopped
	// else
	//    the "stoppable" move is found my successively running through Q with the assuming all moves are going to
	//    decel from either the current speed (for current move) or from the newly calculated exit speed of the
	//    prior move.
	//
	//    if the stoppable move is the current move underway, then a lot a decisions/work is needed to alter the move underway.
	//       if the current move can be stopped within the  remaining distance of the move
	//           if (accelerating) && (can be stopped after hitting cruiseSpeed)
	//               let it reach cruise
	//               calc decel point in order to exit move at to noRamp
	//           else if (accelerating )
	//               immediately start cruising
	//               calc decel point in order to exit move at to noRamp
	//           else if (decelerating)
	//               nothing to do, already slowing down
	//           else (cruising)
	//              calc decel point in order to exit move at to noRamp
	//       else  //    cannot fully stop move underway, so will stop another Q entry
	//            set current move to immediately decel
	//
	//     tag which move is stopping so processing motion can flip up the _gcodePaused flag
	//     update unprime/prime flags in Q to reflect the motion stop mid Q
	//     call motionQ_update() to reset all target velocities and let things proceed
	//

	uint32_t irq_disabled = interruptsOff();

	if (motionQ_empty())
	{   // no current move, so go ahead and pause
		setPauseGcodeFlag(PAUSE_AT_END_OF_MOVE_CHAR);
	}
	else if (motionQ.oldest->flags.homingMove)
	{   // homing, should be only move in queue
		if (stopHomingMove() == TRUE)
		{   // returns true if needs to complete move
			motionQ.oldest->flags.pauseAfterThisMove = TRUE;
		}
		else
		{   // current homing move is now stopped (no more pulses to go, so need to cleanup)
			setPauseGcodeFlag(PAUSE_AT_END_OF_MOVE_CHAR);
			motionQ_deleteCurrentMoveAndExecuteNextMove();
		}
	}
	else
	{   // at least one non-homing move in Q
		motionEntryStruct *stoppableEntry = changeMotionQToPauseAtNextMoveBoundary();

		stoppableEntry->flags.pauseAfterThisMove = TRUE;    // flag which move to pause after
		fixUnprimePrimeForPause(stoppableEntry);
		motionQ.planned = motionQ.oldest;   // force a recalc of all entries.
		motionQ_update();   // force the queue to recalculate
	}

	interruptsOn(irq_disabled);
}

////////////////////////////////////////////////////////////////////////////////

motionEntryStruct *changeMotionQToAbortAsSoonAsPossible(boolean *axisMoving)
{   // return a pointer to the oldest motionQ entry in which all axes can be stopped mid vector
	//
	// goal is to abort (stop) motion as soon as possible, but along the vector of
	// travel, unpriming at the stop point if the move had an E.  any remaining
	// Q entries will be flushed.
	//
	// if no moves in the Q
	//    then already stopped
	// else if homing,
	//    then abort (gracefully) the move and stop (rest of queue should be empty and/or stopped)
	// else
	//    the "stoppable" move is found by successively running through Q with the assuming all moves are going to
	//    decel from either the current speed (for current move) or from the newly calculated exit speed of the
	//    prior move.
	//
	//    in all cases, the current move underway if forced to decel.  if that move cannot be fully slowed, then
	//    continue through the queue until motion can be stopped.
	//
	//     tag which move is stopping so processing motion can flip up the _gcodePaused flag
	//     update unprime flag in Q to reflect the motion stop mid Q
	//     call motionQ_update() to reset all target velocities and let things proceed

	motionEntryStruct *curr;;
	MotorStructure *M;
	float distance, startSpeed, exitSpeed, decelDistance, pulseScale;


	if (FLOW_RATE_CREATED_ON_MOTORC)
	{   // when hijacking M_C, stop flow on any future moves (don't change current move so as not to
		// possibly screw with dominant timer
		int i;
		for (i=0; i<SIZE_OF_MOTION_QUEUE; i++)
		{   // kill E_pulses passed in the Q so any secondary moves after the abort don't screw up.
			if (&motionQ.entries[i] != motionQ.oldest)
			{
				motionQ.entries[i].flowPulses = 0;
				motionQ.entries[i].flowCrossSectionTimesPPU = 0.0f;
				motionQ.entries[i].PULSES_TO_GO[M_C] = 0;
			}
		}
	}

	curr = motionQ.oldest;
	while (1)
	{
		if ((curr < &motionQ.entries[0]) || (curr > &motionQ.entries[SIZE_OF_MOTION_QUEUE-1]))
		{   // safety ceck
			sprintf(_errorStr, "changeMotionQToAbortAsSoonAsPossible (0x%08x / 0x%08x / 0x%08x (%d)", (int)curr,
					(int)&motionQ.entries[0], (int)&motionQ.entries[SIZE_OF_MOTION_QUEUE-1], sizeof(motionEntryStruct));
			barf(_errorStr);
			// barf will force a sw reset, so remaining code won't execute.... but just for safety in case barf behavior is altered.
			curr = motionQ.oldest;
			return(curr);
		}

		if (curr == motionQ.oldest)
		{   // partial move (move is currently underway)
			distance = curr->distance * ((float)DominantAxisPtr->PULSES_TO_GO / (float)DominantAxisPtr->PULSES_IN_MOVE);
			startSpeed = DominantAxisPtr->currentVelocity;
		}
		else
		{   // not oldest, so full moves in play
			distance = curr->distance;  // full move
			startSpeed = curr->startSpeed;      // same as exit speed of prior move
		}

#ifdef GB_HIDDEN_WARNINGS
		int enabledToUseAbortDecel; //enable to use AbortDecel NUKE???
#endif
#if 0 //(save for now)
		// calc new accel for this move based on abort decel rate
		curr->acceleration = MAXFLOAT;
		int axisIndex;
		for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
		{
			if (curr->PULSES_TO_GO[axisIndex])
			{   // axis involved in the move
				M = &Motors[axisIndex];
				curr->acceleration = fminf(curr->acceleration, M->AbortDecelerationRate / fabsf(curr->unitVector[axisIndex])); // fabs because just want magnitude
			}
		}
		curr->acceleration = fmaxf(0.01f, curr->acceleration);  // force to be slightly above zero to avoid div/0 later;
#endif
		// calculate the slowest possible exit speed for this move assume decel were to start ASAP
		exitSpeed = fmaxf(sqr(curr->noRampSpeed), sqr(startSpeed) - 2.0f * curr->acceleration * distance);
		exitSpeed = fpu_sqrtf(exitSpeed);   // delay sqrt in case prior calc yield a negative value

		if (curr != motionQ.newest)
		{   // update Q with expected exit speed for this move.
			curr->next->startSpeed = exitSpeed;
		}

		if ((exitSpeed <= curr->noRampSpeed) || (curr == motionQ.newest))
		{   // the move pointed to by "curr" will be able to (or have to) stop
			if (curr == motionQ.oldest)
			{   // special case to stop move currently underway
				if (DominantAxisPtr->PULSES_TO_GO)
				{
					float axisScale = fabsf(curr->unitVector[curr->flags.dominantAxis]);
					decelDistance = (sqr(DominantAxisPtr->currentVelocity) - sqr(curr->noRampSpeed)) / (2.0f * curr->acceleration); //curr->speed is target cruising speed
					int decelPulses = axisScale * decelDistance * DominantAxisPtr->PulsesPerUnit;
					pulseScale = (float)decelPulses / (float)DominantAxisPtr->PULSES_TO_GO;
					DominantAxisPtr->AccelerationPulses = 0;
					DominantAxisPtr->DecelerationPulses = decelPulses; // ensure decel
					DominantAxisPtr->cruiseVelocity = DominantAxisPtr->currentVelocity;
					DominantAxisPtr->endVelocity = DominantAxisPtr->RatesInUPS[NO_RAMP];
					DominantAxisPtr->PULSES_TO_GO *= pulseScale;
				}
				else
				{   // dominant was finished; so shut down all axes
					pulseScale = 0.0f;
					DominantAxisPtr->AccelerationPulses = 0;
					DominantAxisPtr->DecelerationPulses = 0;
					DominantAxisPtr->cruiseVelocity = DominantAxisPtr->RatesInUPS[NO_RAMP];
					DominantAxisPtr->endVelocity = DominantAxisPtr->RatesInUPS[NO_RAMP];
					DominantAxisPtr->PULSES_TO_GO = 0;
				}
				if (DominantAxisPtr->PULSES_TO_GO == 0)
				{   // no move do -- make sure it's dead
					for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
					{
						DisableMotionTimer(M->TimerBase);
						M->PULSES_TO_GO = 0;
					}
				}
				else
				{   // still a possibility of a move
					for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
					{   // whiz through all the motors and correct PULSES_TO_GO
						if (AxisIsMoving(M))
						{   // still a move to do and this axis is part of it
							if (M != DominantAxisPtr)
							{   // Dom axis was already scaled
								M->PULSES_TO_GO *= pulseScale;    // scale back PULSES_TO_GO uniformly
							}
							if (AxisIsMoving(M))
							{   // axis still moving after scaling
								*axisMoving = TRUE;
							}
							else
							{   // just in case scaling changed PULSES_TO_GO to 0
								DisableMotionTimer(M->TimerBase);
							}
						}
					}
				}
			}   // end of curr == oldest
			else
			{   // at least 2 Q entries and not the oldest will be the move that stops
				// adjust oldest move to start decel immediately

				DominantAxisPtr->AccelerationPulses = 0;
				DominantAxisPtr->DecelerationPulses = DominantAxisPtr->PULSES_TO_GO;
				DominantAxisPtr->endVelocity = motionQ.oldest->next->startSpeed;
				DominantAxisPtr->cruiseVelocity = DominantAxisPtr->currentVelocity;
				if (DominantAxisPtr->PULSES_TO_GO)
				{
					*axisMoving = TRUE;
				}

				// now need to shorten "curr" move so motion stops as soon as possible
				decelDistance = (sqr(curr->startSpeed) - sqr(curr->noRampSpeed)) / (2.0f * curr->acceleration);
				pulseScale = fminf(1.0f, decelDistance / curr->distance);
				curr->distance = decelDistance;
				int axisIndex;
				for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
				{   // whiz through all the motors and correct PULSES_TO_GO for the (post abort) final move in the Q
					curr->PULSES_TO_GO[axisIndex] *= pulseScale;    // scale back pulses2go uniformly
				}
			}
			DominantAxisPtr->cruiseVelocitySquared = sqr(DominantAxisPtr->cruiseVelocity);
			break;  // done
		}
		else
		{   // haven't stopped yet, so move to the next entry in the Q
			curr = curr->next;
		}
	}

	return(curr);
}

////////////////////////////////////////////////////////////////////////////////

void motionQ_abortMotionWhenSafe(flushAction_t flushAction)
{
	// goal is to abourt (stop) motion as soon as possible, but along the vector of
	// travel, unpriming at the stop point if the move had an E.  any remaining
	// Q entries will be flushed.
	//
	// if no moves in the Q
	//    then already stopped
	// else if homing,
	//    then abort (gracefully) the move and stop (rest of queue should be empty and/or stopped)
	// else
	//    the "stoppable" move is found my successively running through Q with the assuming all moves are going to
	//    decel from either the current speed (for current move) or from the newly calculated exit speed of the
	//    prior move.
	//
	//    in all cases, the current move underway if forced to decel.  if that move cannot be fully slowed, then
	//    continue through the queue until motion can be stopped.
	//
	//     tag which move is stopping so processing motion can flip up the _gcodePaused flag
	//     update unprime flag in Q to reflect the motion stop mid Q
	//     call motionQ_update() to reset all target velocities and let things proceed
	//

	boolean axisMoving = FALSE;

	uint32_t irq_disabled = interruptsOff();

	_abortInProgress = (flushAction == FLUSH_THEN_ABORT) ? ABORT_TIME : 0;   // flag to the rest of the code an abort is underway (and how long of time).
	killMotorJogging(); // stop any back door Z motor jogs that may be underway
	KillCannedCycle();  // stop generation of additional moves
	UnPrimeTimer = 0;
	PrimeTimer = 0;
	_MailBoxes._waitingFor.flags.u32 = 0;   // aborting, so make sure we kill any waits for temp, etc

	MotorStructure *M = _gs.latheMotor;
	if (M->latheMode != LATHE_MODE_OFF)
	{
		M->latheAccelPer10ms = (M->AbortDecelerationRate / 100.0f) * M->PulsesPerUnit;  // update speed every 10ms
		M->latheTargetSpeedPPS = 0.0f;
	}

	if (_jogging.enabled)
	{
		joggingAbort("");
		_ABjogging.enabled = FALSE;
	}
	else if (motionQ_empty())
	{   // no current move, so go ahead and pause
		if ( !_TouchProbeMoveActive)
		{
			setPauseGcodeFlag(CATASTROPHIC_ERROR_ALERT_CHAR);
		}
		else
		{
			TouchProbeFinished(PROBE_RESULTS_MOTION_FINISHED, PROBE_TYPE_CONTACT, 0);
		}
	}
	else if (motionQ.oldest->flags.homingMove)
	{   // homing, should be only move in queue
		axisMoving = stopHomingMove();
		if (axisMoving)
		{   // need to complete move
			motionQ.oldest->flags.abortAfterThisMove = TRUE;
		}
		else
		{   // current homing move is now stopped (no more pulses to go, so need to cleanup)
			setPauseGcodeFlag(CATASTROPHIC_ERROR_ALERT_CHAR);
			motionQ_deleteCurrentMoveAndExecuteNextMove();
		}
	}
	else
	{   // at least one non-homing move in Q
		motionEntryStruct *stoppableEntry = changeMotionQToAbortAsSoonAsPossible(&axisMoving);
		while (stoppableEntry != motionQ.newest)  // flush all other entries in the queue
		{   // stoppable entry points to one of the queue entries where motion will stop ... any moves
			// after (newer) are not needed and can/should be removed from the quete
			motionQ_deleteNewestCommand();
		}

		if (axisMoving)
		{
			if ( !_TouchProbeMoveActive)
			{   // stop at the end of the move if we are moving and it's not a probe move
				stoppableEntry->flags.abortAfterThisMove = TRUE;    // flag which move to pause after
			}
			fixUnprimePrimeForAbort(stoppableEntry);
			motionQ.planned = motionQ.oldest;   // force a recalc of all entries.
			motionQ_update();   // force the queue to recalculate
		}
		else if (stoppableEntry ==  motionQ.oldest)
		{   // only entry left in Q and no longer moving
			if ( !_TouchProbeMoveActive)
			{
				setPauseGcodeFlag(CATASTROPHIC_ERROR_ALERT_CHAR);
			}
			else
			{
				TouchProbeFinished(PROBE_RESULTS_MOTION_FINISHED, PROBE_TYPE_CONTACT, 0);
			}
			motionQ_deleteCurrentMoveAndExecuteNextMove();
		}
		else
		{   // oldest move was complete, but still need to get through other moves to safely stop motion
			motionQ_deleteCurrentMoveAndExecuteNextMove();
		}
		if (flushAction == FLUSH_THEN_ABORT)
		{
			_abortOccurredWhileMoving = TRUE;
		}
	}

	interruptsOn(irq_disabled);

	_heartbeatRateControl = HEARTBEAT_MODE_ABORT;

	StopAllExtruders();

	switch (flushAction)
	{
	case FLUSH_THEN_ABORT :
		if (motionQ_empty())
		{   // either was empty to start or is empty after the attempt to stop
			cleanupAfterAbortFinishes();
		}
		break;
	case FLUSH_THEN_CONTINUE :
	default :
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// jogging - options
//      (1) if change of direction, just come to a graceful stop, ignore inputs {for 0.5 seconds}
//      (2) if change of direction, subtract from amount to move until remaining pulses is just enough to stop
//          then store up remaining amounnt for a "next jog"
//      Karl picks #1 with a 0.5 second delay before accepting new inputs

////////////////////////////////////////////////////////////////////////////////

void joggingDeInit(void)
{
	bzero(&_jogging, sizeof(joggingStruct));
	_jogging.enabled = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void joggingInit(motorIndex_t axisIndex, float relativeAmountInUnits)
{   // a new jogging move is starting
	joggingDeInit();
	int i;
	for (i=0; i<SIZE_OF_MOTION_QUEUE; i++)
	{   // flush any old flags in queue .. might be some referenced during jogging
		*((unsigned *)&(motionQ.entries[i].flags)) = 0;
	}
	PrepareForNextMove();
	_jogging.Hz = TICKS_PER_SEC_1000HZ_LOOP;
	_jogging.enabled = TRUE;
	_jogging.firstMove = TRUE;
	_jogging.reloadTimerControls = FALSE;
	_jogging.ignoreInputsCountdownTimer = 0;
	_jogging.axisIndex = axisIndex;
	_jogging.M = &Motors[axisIndex];

	// deal with case where user has not loaded defaults for the jogging rate, etc.   create "useful" values by scaling the values set
	// for normal motion on that axis
#define JOG_DEFAULT_MIN_SCALE   0.167f  // 1/6th of axis min
#define JOG_DEFAULT_MAX_SCALE   0.5f    // half of axis max
#define JOG_DEFAULT_ACC_SCALE   0.167f  // 1/6th of axis accel constant

	if (_jogging.M->joggingRatesSet.JOG_NO_RAMP)
		_jogging.minRate = _jogging.M->RatesInUPS[JOG_NO_RAMP];
	else
		_jogging.minRate = ClampToAxisLimitsInUPS(_jogging.M, _jogging.M->RatesInUPS[NO_RAMP] * JOG_DEFAULT_MIN_SCALE); // jog no ramp was not set, so use a scaled value

	if (_jogging.M->joggingRatesSet.JOGGING)
		_jogging.maxRate = _jogging.M->RatesInUPS[JOGGING];
	else
		_jogging.maxRate = ClampToAxisLimitsInUPS(_jogging.M, _jogging.M->RatesInUPS[RAPID] * JOG_DEFAULT_MAX_SCALE);   // jog rate was not set, so use a scaled value

	if (_jogging.M->joggingRatesSet.ACCEL)
		_jogging.acceleration = _jogging.M->JogAccelerationConstant;
	else
		_jogging.acceleration = fmaxf(DEFAULT_MIN_ACCELERATION_CONSTANT, _jogging.M->AccelerationConstant * JOG_DEFAULT_ACC_SCALE); // accel rate was not, so use a scaled value

	_jogging.dirChangePauseTimeMs = _jogging.M->JogPauseTimeMs;

	if (relativeAmountInUnits > 0.0f)
	{   //positive travel
		_jogging.direction = DIRECTION_FORWARD;
		_jogging.M->DIRECTION = 1;
	}
	else
	{   //negative travel
		_jogging.direction = DIRECTION_REVERSE;
		_jogging.M->DIRECTION = -1;
	}
	outputDirectionBit(&_jogging.M->Direction, _jogging.direction);
	_jogging.M->currentVelocity = _jogging.minRate;
}

////////////////////////////////////////////////////////////////////////////////

void joggingComplete(void)
{
	_jogging.enabled = FALSE;
	PrepareForNextMove();
	if ((_sendingGBStringsMask & GB_STRING_JOG_INFO) && (_sendingGBStringsSubMask & GB_STRING_JOG_INFO_SUBMASK_1)) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr,"J%c: %6.3f %ld", 'E', _jogging.M->POSITION * _jogging.M->UnitsPerPulse, _jogging.M->POSITION);
		sendGB(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void joggingSetQPosition(float destinationInUnits, int destinationInPulses)
{
	if (destinationInPulses > _jogging.M->MaximumTravelInPulses)
	{   //tried to jog too far... limit destination
		destinationInPulses = _jogging.M->MaximumTravelInPulses;   // limit travel to the software limit
		// reverse calc a requested position to get to Max
		_jogging.M->Q_LastRequestedPositionInUnits = _jogging.M->MaximumTravelInUnits - SumOfAllMotorOffsets(_jogging.M);
	}
	else if (destinationInPulses < 0)
	{   //travel less than 0 (illegal)
		destinationInPulses = 0;   //do not move past home
		// reverse calc a requested position to get to 0
		_jogging.M->Q_LastRequestedPositionInUnits = 0.0f - SumOfAllMotorOffsets(_jogging.M);
	}
	else
	{
		_jogging.M->Q_LastRequestedPositionInUnits = destinationInUnits;
	}
	_jogging.M->Q_POSITION = destinationInPulses;
}

////////////////////////////////////////////////////////////////////////////////

void joggingAbort(char *s)
{
	uint32_t irq_disabled = interruptsOff();

	_jogging.ignoreInputsCountdownTimer = _jogging.dirChangePauseTimeMs;

	if (_jogging.enabled && AxisIsMoving(_jogging.M))
	{
		int decelPulses = (int)((fabsf(sqr(_jogging.M->currentVelocity) - sqr(_jogging.minRate)) / (2.0f * _jogging.acceleration)) * _jogging.M->PulsesPerUnit);
		if (decelPulses < _jogging.M->PULSES_TO_GO)
		{   // need to shorten naturally ending move
			_jogging.M->PULSES_IN_MOVE = decelPulses;
			_jogging.M->PULSES_TO_GO = decelPulses;

			_jogging.M->AccelerationPulses = 0;
			_jogging.M->DecelerationPulses = decelPulses; // decel immediately

			_jogging.M->cruiseVelocity = _jogging.M->currentVelocity;   // change cruise to current, since decel keys off cruise speed
			_jogging.M->cruiseVelocitySquared = sqr(_jogging.M->cruiseVelocity);
			_jogging.M->endVelocity = _jogging.minRate; // shouldn;t need this, but just to be safe.... (should already be this value

			// abort changed the Q so reset values
			int destinationInPulses = _jogging.M->POSITION + (_jogging.M->PULSES_TO_GO * _jogging.M->DIRECTION);
			float destinationInUnits = (destinationInPulses * _jogging.M->UnitsPerPulse) - SumOfAllMotorOffsets(_jogging.M);
			joggingSetQPosition(destinationInUnits, destinationInPulses);

		}
		if (_jogging.M->PULSES_TO_GO == 0)
		{
			joggingComplete();
		}
	}

	interruptsOn(irq_disabled);

	if (s[0] != NULL_CHAR)
	{
		sendError(s);
	}
}

////////////////////////////////////////////////////////////////////////////////

void joggingCalculateTimerControls(void)
{
	// now figure out the intersection of the accel and decel side of this move and see if there's room to
	// run at full speed.  using point slope form for the accel and decel "lines", each with a known point and slope
	// m=2A and (0, startSpeed) for the accel and m=-2A and (dist, EndSpeed) for the decel side.  solving
	// for the intersection distance (Xi), yields
	// Xi = 0.5 * (dist + ((EndSpeed - StartSpeed) / 2A))
	// if Xi < 0, then only decelerating
	// elseif Xi > distance, then only accelerating
	// elseif accelDist <= Xi, then normal trapeziod of accel, cruise, decel
	// else not enough room to accel to speed and decel back to endSpeed, so need to clip target speed

#ifdef GB_HIDDEN_WARNINGS
	int pickOne; //NUKE
#endif //GB_HIDDEN_WARNINGS
#if 1  //remove after decision
#define SKIP_ACCELERATION_DISTANCE      0.1f
#define SKIP_ACCELERATION               (distance <= SKIP_ACCELERATION_DISTANCE)
#else
#define SKIP_ACCELERATION_PULSES        4
#define SKIP_ACCELERATION               (_jogging.M->PULSES_IN_MOVE <= SKIP_ACCELERATION_PULSES)
	int changedDefaultForNoAccelInJogging;
#endif

	_jogging.M->PULSES_IN_MOVE = abs(_jogging.M->Q_POSITION - _jogging.M->POSITION);
	_jogging.M->PULSES_TO_GO = _jogging.M->PULSES_IN_MOVE;

	if (_jogging.M->PULSES_TO_GO == 0)
	{
		joggingComplete();
	}
	else
	{
		DominantAxisPtr = _jogging.M;
		DominantAxisPtr->currentVelocity = fmaxf(_jogging.minRate, DominantAxisPtr->currentVelocity);
		DominantAxisPtr->startVelocity  = ClampToAxisLimitsInUPS(DominantAxisPtr, DominantAxisPtr->currentVelocity);
		DominantAxisPtr->endVelocity    = ClampToAxisLimitsInUPS(DominantAxisPtr, _jogging.minRate);

		float distance = (float)abs(_jogging.M->Q_POSITION - _jogging.M->POSITION) * _jogging.M->UnitsPerPulse;
		float accelDistance;
		float decelDistance;
		float cruiseSpeed;
		float startSpeedSquared = sqr(DominantAxisPtr->startVelocity);      // used multiple times
		float endSpeedSquared = sqr(DominantAxisPtr->endVelocity);          // used multiple times
		float accelerationTimesTwo = 2.0f * _jogging.acceleration;          // used multiple times
		float inverseAccelerationTimesTwo = 1.0f / accelerationTimesTwo;    // used multiple times

		float crossoverDist = 0.5f * (distance + ((endSpeedSquared - startSpeedSquared) * inverseAccelerationTimesTwo));

		if (crossoverDist < 0)
		{   // decel only
			cruiseSpeed = fpu_sqrtf(endSpeedSquared + (accelerationTimesTwo * distance));
			accelDistance = 0.0f;
			decelDistance = distance;
		}
		else if (crossoverDist > distance)
		{   // accel only
			cruiseSpeed = fpu_sqrtf(startSpeedSquared + (accelerationTimesTwo * distance));
			accelDistance = distance;
			decelDistance = 0.0f;
		}
		else if (SKIP_ACCELERATION)
		{   // would have been some combo of accel/decel; but move is too short to bother (minimize vibration)
			cruiseSpeed = DominantAxisPtr->startVelocity;
			accelDistance = 0.0f;
			decelDistance = 0.0f;
		}
		else
		{   // some combo of accel/decel
			float cruiseSpeedSquared = sqr(_jogging.maxRate);
			accelDistance = (cruiseSpeedSquared - startSpeedSquared) * inverseAccelerationTimesTwo;
			if (accelDistance > crossoverDist)
			{   // not enough room, so reduce cruiseSpeed
				cruiseSpeed = fpu_sqrtf(startSpeedSquared + (accelerationTimesTwo * crossoverDist));
				accelDistance = crossoverDist;
				decelDistance = (distance) - crossoverDist;
			}
			else
			{   // enough room to accel, cruise, and decel
				cruiseSpeed = _jogging.maxRate;
				decelDistance = (cruiseSpeedSquared - endSpeedSquared) * inverseAccelerationTimesTwo;
			}
		}

		DominantAxisPtr->cruiseVelocity = ClampToAxisLimitsInUPS(DominantAxisPtr, cruiseSpeed);
		DominantAxisPtr->cruiseVelocitySquared = sqr(DominantAxisPtr->cruiseVelocity);
		DominantAxisPtr->AccelerationPulses = accelDistance * DominantAxisPtr->PulsesPerUnit;
		DominantAxisPtr->DecelerationPulses = decelDistance * DominantAxisPtr->PulsesPerUnit;
		DominantAxisPtr->AccelerationScaled = _jogging.acceleration;
		DominantAxisPtr->AccelerationScaledTimes2 = 2.0f * DominantAxisPtr->AccelerationScaled;

		CalculateTimerControls(_jogging.M);

		if ((_sendingGBStringsMask & GB_STRING_JOG_INFO) && (_sendingGBStringsSubMask & GB_STRING_JOG_INFO_SUBMASK_1)) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"J%c: (%ld)%6.3f  %4.3f %4.3f %4.3f  %4.3f %4.3f %4.3f  %4ld %3ld %3ld (%lu)",
					DominantAxisPtr->AxisLabel, DominantAxisPtr->DIRECTION, distance,
					DominantAxisPtr->startVelocity, DominantAxisPtr->cruiseVelocity, DominantAxisPtr->endVelocity,
					crossoverDist, accelDistance, decelDistance,
					DominantAxisPtr->PULSES_IN_MOVE, DominantAxisPtr->AccelerationPulses, DominantAxisPtr->DecelerationPulses,
					_gs._milliseconds);
			sendGB(_tmpStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean joggingOkToProcessUpdate(motorIndex_t axisIndex)
{
	if (_blockAllMotion)
	{   // do not process an new moves
		if (_blockAllMotionMessageSent == FALSE)
		{
			sendError("All motion blocked -- try to clear the source of the fault and the press RESET in the GUI to continue");
			_blockAllMotionMessageSent = TRUE;
		}
		return(FALSE);
	}

	if (motionQ_notEmpty())
	{
		if (_blockJoggingForMotionMessageSent == FALSE)
		{
			sendError("Normal GCODE motion in process -- jogging blocked");
			_blockJoggingForMotionMessageSent = TRUE;
		}
		return(FALSE);
	}

	if (_jogging.ignoreInputsCountdownTimer)
	{
		return(FALSE);

	}
	return(Motors[axisIndex].MotorInstalled);
}

////////////////////////////////////////////////////////////////////////////////

void joggingUpdatePulses(motorIndex_t axisIndex, float relativeAmountInUnits)
{
	//path in to this method...
	// G0.1 (from wifi/usb pendant)
	// A/B encoder (from direct connect pendant OR new control panel push encoder
	// Z move from hotbed

	if (relativeAmountInUnits == 0.0f) return; // nothing to do
	if (joggingOkToProcessUpdate(axisIndex))
	{
		if (_jogging.enabled == FALSE)
		{   // jogging was not enabled, so go ahead and set up jogging
			joggingInit(axisIndex, relativeAmountInUnits);
		}

		if (axisIndex != _jogging.axisIndex)
		{
			joggingAbort("JOG AXIS CHANGED WHILE JOGGING");
		}
		else if (((relativeAmountInUnits < 0.0f) && (_jogging.direction == DIRECTION_FORWARD)) || ((relativeAmountInUnits > 0.0f) && (_jogging.direction == DIRECTION_REVERSE)))
		{   // direction change
			joggingAbort("");
		}
		else if (((relativeAmountInUnits > 0.0f) && (_jogging.direction == DIRECTION_FORWARD)) || ((relativeAmountInUnits < 0.0f) && (_jogging.direction == DIRECTION_REVERSE)))
		{   // motion in same direction, so add to current pulse count;
			float destinationInUnits = _jogging.M->Q_LastRequestedPositionInUnits + relativeAmountInUnits;
			int destinationInPulses = (int)roundf((destinationInUnits + SumOfAllMotorOffsets(_jogging.M)) * _jogging.M->PulsesPerUnit);
			joggingSetQPosition(destinationInUnits, destinationInPulses);

			if ((_sendingGBStringsMask & GB_STRING_JOG_INFO) && (_sendingGBStringsSubMask & GB_STRING_JOG_INFO_SUBMASK_1)) // use M797 S<mask> to enable
			{
				if (_jogging.firstMove)
				{
					sprintf(_tmpStr,"J%c: %6.3f %ld", 'P', _jogging.M->POSITION * _jogging.M->UnitsPerPulse, _jogging.M->POSITION);
					sendGB(_tmpStr);
				}
				sprintf(_tmpStr,"J%c: %6.3f %ld  (%4.3f) (%d)", (_jogging.firstMove ? 'S' : 'M'), _jogging.M->Q_LastRequestedPositionInUnits, _jogging.M->Q_POSITION,
						relativeAmountInUnits, (int)_gs._milliseconds);
				sendGB(_tmpStr);
			}

			if (_jogging.firstMove)
			{
				EnableAllMotionMotors(); // turn on enable bits just in case they were off
				joggingCalculateTimerControls();

				_numMovingAxes = 0;
				_movingAxesPtrs[0] = _jogging.M;    // only one axis will be moving
				_numMovingAxes = 1;

				EnableMotionTimers();    // load counters and enable timer
				_jogging.firstMove = FALSE;
			}
			else
			{
				_jogging.reloadTimerControls = TRUE;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
