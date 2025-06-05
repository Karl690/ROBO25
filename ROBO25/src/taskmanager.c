#include "taskmanager.h"
#include "MotorDriver.h"
#include "pinout.h"
#include "Serial.h"
#include "mailbox.h"
#include "pnp.h"
#include "adc.h"
#include "GCode.h"
#include "Hydra_can.h"
uint32_t HeartBeat = 0;

unsigned int counter_1Hz = 0;
unsigned int counter_10Hz = 0;
unsigned int counter_100Hz = 0;
unsigned int counter_1000Hz = 0;
int currentArgLength = 0; //used for preventing overrun on characters

boolean _motionSensorTrippedNext = FALSE;

boolean _checkLineChecksum = FALSE;
uint32_t _lineChecksum = 0;
int ProcessingError = 0;

int homeSensedAxisCntr = 0;
int CurrentCommandIndex = 2; // index of the current place in cmdQue[] from which to get prams for execution
							// points to the current place in cmdQue[] from which to get prams for execution
uint32_t Head11_Temperature = 0;
uint32_t Head11_HTRDuty = 0;
uint32_t Head11_FanDuty = 0;
uint32_t Head11_Spare = 0;


GMCommandStructure cmdQue[SIZE_OF_COMMAND_QUEUE];
//NUKE GMCommandStructure *ExecutionPtr = &cmdQue[0];
GMCommandStructure *ExecutionPtr;
GMCommandStructure *NextExecutionPtr; //NextExecutionPtr=&cmdQue[NextCommandIndex]; tmp pointer
int NextCommandIndex; // index of the next command to be executed (the one after Current), initially set to 2 after the first command is received and parsed
int NextCommandInsertionIndex; // index of the next free location to add an incomming command
char *GCodeArgPtr;
//unsigned int counter_1Hz;
//unsigned int counter_10Hz;
//unsigned int counter_100Hz;
//unsigned int counter_1000Hz;


char GCodeArgA[MAX_CHARS_FOR_PARAMETER]     = "A              ";
char GCodeArgB[MAX_CHARS_FOR_PARAMETER]     = "B              ";
char GCodeArgC[MAX_CHARS_FOR_PARAMETER]     = "C              ";
char GCodeArgD[MAX_CHARS_FOR_PARAMETER]     = "D              ";
char GCodeArgE[MAX_CHARS_FOR_PARAMETER]     = "E              ";
char GCodeArgF[MAX_CHARS_FOR_PARAMETER]     = "F              ";
char GCodeArgG[MAX_CHARS_FOR_PARAMETER]     = "G              ";
char GCodeArgH[MAX_CHARS_FOR_PARAMETER]     = "H              ";
char GCodeArgI[MAX_CHARS_FOR_PARAMETER]     = "I              ";
char GCodeArgJ[MAX_CHARS_FOR_PARAMETER]     = "J              ";
char GCodeArgK[MAX_CHARS_FOR_PARAMETER]     = "K              ";
char GCodeArgL[MAX_CHARS_FOR_PARAMETER]     = "L              ";
char GCodeArgM[MAX_CHARS_FOR_PARAMETER]     = "M              ";
char GCodeArgN[MAX_CHARS_FOR_PARAMETER]     = "N              "; //line number
char GCodeArgO[MAX_CHARS_FOR_PARAMETER]     = "O              ";
char GCodeArgP[MAX_CHARS_FOR_PARAMETER]     = "P              ";
char GCodeArgQ[MAX_CHARS_FOR_PARAMETER]     = "Q              "; //checksum -- using 'Q'
char GCodeArgR[MAX_CHARS_FOR_PARAMETER]     = "R              ";
char GCodeArgS[MAX_CHARS_FOR_PARAMETER]     = "S              ";
char GCodeArgT[MAX_CHARS_FOR_PARAMETER]     = "T              ";
char GCodeArgU[MAX_CHARS_FOR_PARAMETER]     = "U              ";
char GCodeArgV[MAX_CHARS_FOR_PARAMETER]     = "V              ";
char GCodeArgW[MAX_CHARS_FOR_PARAMETER]     = "W              ";
char GCodeArgX[MAX_CHARS_FOR_PARAMETER]     = "X              ";
char GCodeArgY[MAX_CHARS_FOR_PARAMETER]     = "Y              ";
char GCodeArgZ[MAX_CHARS_FOR_PARAMETER]     = "Z              ";
char GCodeArgSplat[MAX_CHARS_FOR_PARAMETER] = "*              ";


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// WARNING:  IF THE ORDER OF ANY OF THESE CHANGE, PLEASE UPDATE initSliceTiming()
const PFUNC F1000HZ[NUM_1000HZ] =
{
	ohNoMrBill,
	// can't use slice 0 and this is time slot to execute the next slower slice
	serialProcessor,
	CommandProcessor,
	// can't move to foreground due to collision on global "ExecutionPtr"
	SequenceEngine,
	// controls a lot of ms increment timers -- MUST STAY IN 1000Hz loop
	canProcessRxQueueNoReturn,
	canProcessTxQueueNoReturn,
	motionQ_update,
	loop_1000Hz_simple_work,
	// keep as last call in this array
};

const PFUNC F100HZ[NUM_100HZ] =
{
	ohNoMrBill,
	// can't use slice 0 and this is time slot to execute the next slower slice
	initFromSoapstring,
	readInputs,
	checkMotorFaultSensor,
	checkMotorLimit1Sensor,
	checkMotorLimit2Sensor,
	latheSpeedControl,
	PnP_SetValves,
	LatchPnPData,
	loop_100Hz_simple_work,
	// keep as last call in this array
};

const PFUNC F10HZ[NUM_10HZ] =
{
	ohNoMrBill,
	// can't use slice 0 and this is time slot to execute the next slower slice
	soapstringController,
	sendUpdateToHost,
	checkBlockingWaits,
	EdgeTriggerSendResults,
	// move into simple_work if space needed
	checkForCompletedAbort,
	ReportXYZLocation,
	ProcessRawADC_Data,
	spare,
	loop_10Hz_simple_work,
	// keep as last call in this array
};

const PFUNC F1HZ[NUM_1HZ] =
{
	checkForMia,
	spare,
	ReportOsseoVariables,
	spare,
	spare,
	LaserSendRequestStringToPowerSupply,
	spare,
	spare,
	spare,
	loop_1Hz_simple_work,
	// keep as last call in this array
};

////////////////////////////////////////////////////////////////////////////////

void loop_1000Hz_simple_work(void)
{
	_gs._milliseconds++;
	if (_highVoltageIsNotStableCoundownMs > 0)
	{
		_highVoltageIsNotStableCoundownMs--;
		if (_highVoltageIsNotStableCoundownMs == 0)
		{
			//power now deemed good
			;
		}
	}
	if (Co2LaserWatchDogTimer)
	{
		Co2LaserWatchDogTimer--;
		if (Co2LaserWatchDogTimer == 0)
		{
			//CO2LaserAnalogPwrPWM = 0; //turn of 0-5v power
			TIM8->CCR3 = 0; //turn off direct input PWM  karlChris add osseo check
		}
	}
	HssControl(TICKS_PER_SEC_1000HZ_LOOP);
	DwellTimer();

	if (_requestToPauseAtEndOfMove)
	{
		motionQ_pauseMotionWhenSafe();
		_requestToPauseAtEndOfMove = FALSE;
	}

	if (_requestToAbortAtEndOfMove)
	{
		// request abort from repetrel to stop at the end of the current move if possible
		_requestToAbortAtEndOfMove = FALSE;
		_abortCameFromRx = 1; // tag that we were requested to abort
		SpindleDesiredSpeedPWM = 0; //kill power now
		motionQ_abortMotionWhenSafe(FLUSH_THEN_ABORT); // once motion is stopped, ResetProcess() will be called
	}

	if (_jogging.ignoreInputsCountdownTimer)
	{
		_jogging.ignoreInputsCountdownTimer--;
		if (_jogging.M && _jogging.M->PULSES_TO_GO)
		{
			// still decelerating, so make sure turnaround time is met
			_jogging.ignoreInputsCountdownTimer = imax(_jogging.ignoreInputsCountdownTimer, _jogging.dirChangePauseTimeMs);
		}
		if (_jogging.ignoreInputsCountdownTimer <= 0)
		{
			joggingDeInit();
		}
	}

#ifdef USE_CAN_MOTOR_HOME_SENSOR
	if (Motors[homeSensedAxisCntr].MotorInstalled && !Motors[homeSensedAxisCntr].canMotor)
#else //!USE_CAN_MOTOR_HOME_SENSOR
		if (Motors[homeSensedAxisCntr].MotorInstalled)
#endif //!USE_CAN_MOTOR_HOME_SENSOR
		{
			updateSensorHistoryIfEnabledWithoutIrq(&Motors[homeSensedAxisCntr].HomeSense);
		}
	homeSensedAxisCntr++;
	if (homeSensedAxisCntr > LastAxisMotor)
	{
		homeSensedAxisCntr = FirstAxisMotor;
	}

#ifdef ADD_ON_SPI_DISPLAY
	GUI_IncrementIntervalCount();
#endif
	counter_1000Hz++;
}

////////////////////////////////////////////////////////////////////////////////

void loop_100Hz_simple_work(void)
{
	PnP_Rclk_Clr;
	PnP_Enable_Clr;
	if (!_abortInProgress)
	{
		// regular processing, so show state of pending ack
		_heartbeatRateControl = pendingAcknowledge ? HEARTBEAT_MODE_PENDING_ACK : HEARTBEAT_MODE_NORMAL;
	}

	if ((counter_100Hz % _heartbeatRateControl) == 0)
	{
		heartbeat();
	}
	HssControl(TICKS_PER_SEC_100HZ_LOOP);

	if (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_NONE)
	{
		// normal operation
		DDLightSelection();
		_gs._led.canRxLedCnt = imax(0, _gs._led.canRxLedCnt - 1);
		pinWrite(CAN_TX_LED, (_gs._led.canTxLedCnt > 0) ? 1 : 0);
		_gs._led.canTxLedCnt = imax(0, _gs._led.canTxLedCnt - 1);
	}
	else
	{
		if (_gs._flasher.varPtr)
		{
			// have a ptr to a variable AND the variable is non-zero .... light 'em up

			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_LED) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				pinSet(CAN_TX_LED);
			}
			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_DDL) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_ON);
			}
		}
		else
		{
			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_LED) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				pinClear(CAN_TX_LED);
			}
			if ((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_DDL) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH))
			{
				changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_OFF);
			}
		}
	}

	counter_100Hz++;
	if (_needToWiggleDirectionPins > 0)
	{
		_needToWiggleDirectionPins--;

		for (MotorStructure *M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
		{
			outputDirectionBit(&M->Direction, (_needToWiggleDirectionPins & 1) ? DIRECTION_REVERSE : DIRECTION_FORWARD);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void loop_10Hz_simple_work(void)
{
	HssControl(TICKS_PER_SEC_10HZ_LOOP);

	if (_jogging.enabled && (_jogging.ignoreInputsCountdownTimer == 0) && (_jogging.M->PULSES_TO_GO == 0))
	{
		// something went wrong and jogging did not get disabled;
		sendError("Forcing jog mode OFF");
		joggingComplete();
	}
#ifdef USE_AB_ENCODER
	if (_ABjogging.enabled)
	{
		uint32_t irq_disabled = interruptsOff();
		int detents = TIM5->CNT;
		TIM5->CNT = 0; // reset cnt
		interruptsOn(irq_disabled);
		if (detents)
		{
			// encoder moved a non-zero amount
			joggingUpdatePulses(_ABjogging.axisIndex, _ABjogging.incrAmount * detents);
		}
	}
#ifdef ADD_ON_SPI_DISPLAY
	else if (_ABjogging.offMode == JOG_OFF_MODE_GUI)
	{
		// GUI control
		if (_gs._displaySelectDebounceCnt > 0)
			_gs._displaySelectDebounceCnt--;
		if (_gs._displaySelectDebounceCnt == 0)
		{
			//uint32_t irq_disabled = interruptsOff();
			int detents = TIM5->CNT;
			TIM5->CNT = 0; // reset cnt
			//interruptsOn(irq_disabled);
			if (detents)
			{
				// encoder moved a non-zero amount
				int nextPage;
				if (detents > 0)
				{
					// encoder moved a positive amount
					if (GUI_GetCurrPage() == DISPLAY_PAGE_LAST_ONE)
						nextPage = 0;
					else
						nextPage = GUI_GetCurrPage() + 1;
				}
				else if (detents < 0)
				{
					// encoder moved a negative amount
					if (GUI_GetCurrPage() == 0)
						nextPage = DISPLAY_PAGE_LAST_ONE;
					else
						nextPage = GUI_GetCurrPage() - 1;
				}
				GUI_SetNextPage(nextPage);
				_gs._displaySelectDebounceCnt = 3;
			}
		}
		else if (_gs._displaySelectDebounceCnt == 1)
		{
			//uint32_t irq_disabled = interruptsOff();
			TIM5->CNT = 0; // reset cnt
			//interruptsOn(irq_disabled);
		}
	}
#endif //ADD_ON_SPI_DISPLAY
	else if (_ABjogging.offMode == JOG_OFF_MODE_FILAMENT)
	{
		;
	}
#endif //USE_AB_ENCODER

#ifdef MEASURE_TIME_SLIPPAGE
	if (_sendTimeSlippageData)
	{
		M_Code_M779();
		_sendTimeSlippageData = FALSE;
	}
#endif //MEASURE_TIME_SLIPPAGE

	counter_10Hz++;
}

////////////////////////////////////////////////////////////////////////////////

void loop_1Hz_simple_work(void)
{
#ifdef GB_SECONDS_PIN
	pinToggleOutput(GB_SECONDS_PIN);
#endif //GB_SECONDS_PIN
	_gs._seconds++;
	//	if (_gs._seconds % 2 == 0) LCD_CS_CLR; //pinWrite(SPI3_LCD_CS, 0);// 
	//	else LCD_CS_SET;  //pinWrite(SPI3_LCD_CS, 1);
	IWDG_ReloadCounter(); // reset the independent watchdog hardware

	if ((masterCommPort == BOOTUP) && (_gs._seconds > 2) && ((_gs._seconds % 3) == 0))
	{
		// every 3 seconds try to get repetrel's attention
		_bootupAlertHostChar = HELLO_WORLD_CHAR;
		_sendBootupAlertHostChar = TRUE; // reload as USB code resets to FALSE when xfer occurs
		forceCharToHw(_bootupAlertHostChar); // force out a message to all comm ports to let host know we're alive
	}


	checkInWithHeads();
	HssControl(TICKS_PER_SEC_1HZ_LOOP);
	if ((counter_1Hz % CLEAR_STICKY_ERROR_RATE) == 0)
	{
		resetStickyErrorFlags();
	}

	if (_gs._errorThrottledCount >=  MAX_ERROR_MESSAGES)
	{
		_gs._errorThrottledCount--; //allow one message per sec, even when spewing
	}

	if (_abortInProgress > 0)
	{
		// watchdog -- limit length of abort to "ABORT_TIME" seconds, just in case something goes wrong
		_abortInProgress--;
		if (_abortInProgress == 0)
		{
			_abortFinisedNeedToResetProcessSynchronously = TRUE; //gracefully reset
			sendInfo("ABORTING abort process");
			pauseToTransmitBufferToEmpty();
		}
	}

#ifdef ALLOW_NATIVE_LIGHTBURN
	if ((!_lightburnModeEnabled) && (_repetrelCommWatchCount > 0))
#else //!ALLOW_NATIVE_LIGHTBURN
		if (_repetrelCommWatchCount > 0)
#endif //!ALLOW_NATIVE_LIGHTBURN
		{
			_repetrelCommWatchCount--;
			if (_repetrelCommWatchCount == 0)
			{
				// lost repetrel, so go to a safe state
				sendSetSwResetBit(0); // reset all heads;
				sendError("LOST COMM WITH REPETREL");
			}
		}

	counter_1Hz++;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void spare(void)
{
	// placeholder call for empty slice
}

////////////////////////////////////////////////////////////////////////////////

void ohNoMrBill(void)
{
	// XXXX should send an ERROR .... should never get here
}

void serialProcessor()
{
	buffer_t useBuffer;
	if (_abortInProgress) return;
	if (processSoapstringCommands && (soapstringCommandWaiting == 0)) {return;} // still processing soapstring
	if (CommandReadyToProcessFlag){return;}// current command still processing, so can't continue
	if (cmdQueIsFull()){return;}//dont take more commands unless you can handle them

	if (soapstringCommandWaiting)
	{
		protectedDecrement(&soapstringCommandWaiting);
		useBuffer = SOAPSTRING_BUFFER;
	}
	else if (urgentCommandWaiting)
	{
		if (ExecuteImmediateFlag == 2)
			return; // still processing last immediate command, need to wait
		protectedDecrement(&urgentCommandWaiting);
		useBuffer = URGENT_BUFFER;
	}
	else if (normalCommandWaiting)
	{
		protectedDecrement(&normalCommandWaiting);
		useBuffer = NORMAL_BUFFER;
	}
	else
	{
		// no full commands available in serial buffers
		return;
	}
	ResetGcodeParseBuffer(); //clear first character of each argument
	SplitCurrentGcodeLine2Arguments(useBuffer); //load each of the argument directly into their respective buffers

	if (_checkLineChecksum)
	{
		processArgs(&GCodeArgSplat[0], &ExecutionPtr->CS); // need to get the checksum
		if ((uint32_t)ARG_CS != _lineChecksum)
		{
			sprintf(_errorStr, "Line Checksum mismatch (%d/%d) **%s**", (int)_lineChecksum, (int)ARG_CS, makeAllCharsPrintable(currentCommandString));
			sendError(_errorStr);
		}
	}

	if (ProcessingError > 0)
	{
		printSerialInputStreamError();
	}
	else
	{
		if (useBuffer == URGENT_BUFFER)
			ExecuteImmediateFlag = 1;
		CommandReadyToProcessFlag = 1;
	}
}



void CommandProcessor()
{
	if (_abortInProgress) return;
	if (ExecuteImmediateFlag == 2)return; //have not yet processed immediate command //NEW
	if (CommandReadyToProcessFlag == 0)return;//wait for a message that the incoming command string has been received and successfully parsed
	//the way this is supposed to work is the command processor adds commands to the command que
	//then the sequence engine executes the delayed commands and counts the number of waiting coded to execute down
	if (cmdQueIsFull()){return;}//dont take more commands unless you can handle them

	ExecutionPtr = &cmdQue[0]; //point to the input parse buffer
	processArgs(&GCodeArgA[0], &ExecutionPtr->A);
	processArgs(&GCodeArgB[0], &ExecutionPtr->B);
	processArgs(&GCodeArgC[0], &ExecutionPtr->C);
	processArgs(&GCodeArgD[0], &ExecutionPtr->D);
	processArgs(&GCodeArgE[0], &ExecutionPtr->E);
	processArgs(&GCodeArgF[0], &ExecutionPtr->F);
	processArgs(&GCodeArgG[0], &ExecutionPtr->G);
	processArgs(&GCodeArgH[0], &ExecutionPtr->H);
	processArgs(&GCodeArgI[0], &ExecutionPtr->I);
	processArgs(&GCodeArgJ[0], &ExecutionPtr->J);
	processArgs(&GCodeArgK[0], &ExecutionPtr->K);
	processArgs(&GCodeArgL[0], &ExecutionPtr->L);
	processArgs(&GCodeArgM[0], &ExecutionPtr->M);
	processArgs(&GCodeArgN[0], &ExecutionPtr->N);
	processArgs(&GCodeArgO[0], &ExecutionPtr->O);
	processArgs(&GCodeArgP[0], &ExecutionPtr->P);
	processArgs(&GCodeArgQ[0], &ExecutionPtr->Q);
	processArgs(&GCodeArgR[0], &ExecutionPtr->R);
	processArgs(&GCodeArgS[0], &ExecutionPtr->S);
	processArgs(&GCodeArgT[0], &ExecutionPtr->T);
	processArgs(&GCodeArgU[0], &ExecutionPtr->U);
	processArgs(&GCodeArgV[0], &ExecutionPtr->V);
	processArgs(&GCodeArgW[0], &ExecutionPtr->W);
	processArgs(&GCodeArgX[0], &ExecutionPtr->X);
	processArgs(&GCodeArgY[0], &ExecutionPtr->Y);
	processArgs(&GCodeArgZ[0], &ExecutionPtr->Z);
	ExecutionPtr->cmdType = UNDEFINED;
	ExecutionPtr->cmdLink = UNPROCESSED;

	if (ARG_N_PRESENT)
		_gcodeLineNumber = ARG_N;
	else
		ARG_N = ++_gcodeLineNumber;

	if (ProcessingError > 0)
	{
		printSerialInputStreamError();
		return; // skip line
	}

	if (ARG_M_PRESENT)
	{
		// process mcode
		switch ((int)ARG_M)
		{
		case 0:   AddCommandToQue(UNDEFINED); break;//return;//Stop
		case 1:   AddCommandToQue(UNDEFINED); break;//return;//Sleep
		case 3:   AddCommandToQue(SINGLE_STEP); break;//return;//Spindle On, Clockwise (CNC specific)
		case 4:   AddCommandToQue(SINGLE_STEP); break;//return;//Spindle On, Counter-Clockwise (CNC specific)
		case 5:   AddCommandToQue(SINGLE_STEP); break;//return;//Spindle Off (CNC specific)
		case 6:   AddCommandToQue(SINGLE_STEP); break;//return;//M6Tool Change();
		case 7:   AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//Mist Coolant On (CNC specific)
		case 8:   AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//Flood Coolant On (CNC specific)
		case 9:   AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//Coolant Off (CNC specific)
		case 10:  ContinueToNextStep(); break;//return;//Vacuum On (CNC specific)
		case 11:  ContinueToNextStep(); break;//return;//Vacuum Off (CNC specific)
		case 17:  AddCommandToQue(SINGLE_STEP); break;//return;//enable the holding torque on stepping motors
		case 18:  AddCommandToQue(SINGLE_STEP); break;//return;//releases the holding torque on stepping motors
		case 20:  ContinueToNextStep(); break;//return;//List SD card
		case 21:  ContinueToNextStep(); break;//return;//Initialize SD card
		case 22:  ContinueToNextStep(); break;//return;//Release SD card
		case 23:  ContinueToNextStep(); break;//return;//Select SD file
		case 24:  ContinueToNextStep(); break;//return;//Start/resume SD print
		case 25:  ContinueToNextStep(); break;//return;//Pause SD print
		case 26:  ContinueToNextStep(); break;//return;//Set SD position
		case 27:  ContinueToNextStep(); break;//return;//Report SD print status
		case 28:  ContinueToNextStep(); break;//return;//Begin write to SD card
		case 29:  ContinueToNextStep(); break;//return;//Stop writing to SD card
		case 30:  AddCommandToQue(SINGLE_STEP); break;//return;//notify end of program
		case 40:  ContinueToNextStep(); break;//return;//Eject Part
		case 41:  AddCommandToQue(SINGLE_STEP); break;//return;//set spindle to LOW speed range
		case 42:  AddCommandToQue(SINGLE_STEP); break;//return;//set spindle to HIGH speed range
		case 43:  ContinueToNextStep(); break;//return;//Stand by on material exhausted
		case 44:  ContinueToNextStep(); break;//return;//Spindle Coolant On
		case 45:  ContinueToNextStep(); break;//return;//Spindle Coolant Off
		case 69:  EchoBackCommentString(); ContinueToNextStep(); break;//return;//used for high speed com test
		// this MCode (M73) can nad will be sent at any time and will often occur at times between to printing moves
		// in order no not have a deferred command to process at this critical time, this MCode will be processed
		// earlier than it should be (will be done in front of the motionQ instead of after it.  will cause
		// an error in the timing by the time it takes to empty the queue, but safer this way
		case 73:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;// passthru of remaining printing time (uses P, R)
		case 80:  ContinueToNextStep(); break;//return;//ATX Power On
		case 81:  ContinueToNextStep(); break;//return;//ATX Power Off
		case 82:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;// set extruder to absolute mode
		case 83:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;// set extruder to relative mode
		case 84:  AddCommandToQue(SINGLE_STEP); break;//return;//releases the holding torque on stepping motors
		case 91:  AddCommandToQue(SINGLE_STEP); break;//return;//Set axis_MaxTravel in mm
		case 92:  AddCommandToQue(SINGLE_STEP); break;//return;//Set axis_steps_per_unit
		case 93:  AddCommandToQue(SINGLE_STEP); break;//return;//Set home sensor polarity
		case 94:  AddCommandToQue(SINGLE_STEP); break;//return;//Set Motor direction polarity
		case 95:  AddCommandToQue(SINGLE_STEP); break;//return;// sets stall sensor polarity (uses X, Y, Z, A, B, C)
		case 96:  AddCommandToQue(SINGLE_STEP); break;//return;// sets the enable bit polarity (uses X, Y, Z, A, B, C)
		case 97:  AddCommandToQue(SINGLE_STEP); break;//return;// sets the step bit polarity (uses X, Y, Z, A, B, C)
		case 98:  AddCommandToQue(SINGLE_STEP); break;//return;// sets limit1 sensor polarity (uses X, Y, Z, A, B, C)
		case 99:  AddCommandToQue(SINGLE_STEP); break;//return;// sets limit2 sensor polarity (uses X, Y, Z, A, B, C)
		//RETIRED case 101: AddCommandToQue(SINGLE_STEP);    break;//return;//M_Code_M101(); return;//turn on extuder, set rate and cont extrude
		//RETIRED case 102: AddCommandToQue(SINGLE_STEP);    break;//return;//M_Code_M102(); return;//turn on extuder, set rate and cont reverse extrude
		case 103: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M103(); return;//turn extruder off
		case 104: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M104(); break;//return;//set desired temperature to Sxxx.x
		case 105: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M105(); break;//return;//get extruder temperature
		case 106: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M106(); break;//return;//turn fan on (S=duty)
		case 107: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M107(); break;//return;//turn fan off
		//RETIRED case 108: AddCommandToQue(SYNCS_WITH_MOTION);    break;//return;//M_Code_M108(); break;//return;//set extruder speed to Fxx.x amount per min
		case 109: AddCommandToQue(SINGLE_STEP); break;//return;//M_Code_M109(); break;//return;//wait for extruder to reach temp
		case 114: M_Code_M114(); ContinueToNextStep(); break;//return;//report xyz location immediately
		case 115: M_Code_M115(); ContinueToNextStep(); break;//return;//tell them what firmware revision level we are using
		case 140: AddCommandToQue(SINGLE_STEP); break;//return;//M_Code_M140();  break;//return;//Set bed temperature
		case 141: AddCommandToQue(SINGLE_STEP); break;//return;//M_Code_M141(); break;//return;// set chamber temperature (uses S)
		case 142: ContinueToNextStep(); break;//return;// set the holding pressure on build platform, ie vacuum hold down
		case 143: ContinueToNextStep(); break;//return;//Maximum hot-end temperature
		case 160: ContinueToNextStep(); break;//return;//Number of mixed materials
		case 190: AddCommandToQue(SINGLE_STEP); break;//return;//wait for hotbed to reach temp (uses S)
		case 191: AddCommandToQue(SINGLE_STEP); break;//return;//wait for chamber to reach temp (uses S)
		case 200: ContinueToNextStep(); break;//return;//Set filament diameter or volume
		case 201: ContinueToNextStep(); break;//return;//Set max printing acceleration
		case 202: ContinueToNextStep(); break;//return;//Set max travel acceleration
		case 203: AddCommandToQue(SINGLE_STEP); break;//return;//Sets the Maximum G0/Rapid velocity UNIT/MIN (and Homing/Accel ramp)
		case 204: AddCommandToQue(SINGLE_STEP); break;//return;//set no ramp speed, no acceleration below this point
		case 205: AddCommandToQue(SINGLE_STEP); break;//return;//set home speed
		case 206: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the homing routine hysteresis location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 207: ContinueToNextStep(); break;//return;//calibrate z axis by detecting z max length
		case 208: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the acceleration constant (uses X, Y, Z, A, B, C)
		case 209: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the minimum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 210: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the maximum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		//RETIRED case 211: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the machine minimum pulse rate limit (uses S)
		//RETIRED case 212: AddCommandToQue(SINGLE_STEP);  break;//return;// Sets the machine maximum pulse rate limit (uses S)
		case 213: AddCommandToQue(SINGLE_STEP); break;//return;// sets the per motor/axis installation status (uses X,Y,Z,A,B,C)
		case 214: AddCommandToQue(SINGLE_STEP); break;//return;// sets the per motor/axis type (linear/rotary) (uses X,Y,Z,A,B,C)
		case 215: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the homing routine start location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 216: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the homing routine end location (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 217: AddCommandToQue(SINGLE_STEP); break;//return;// sets the max deceleration rate (for abort) (mm/sec/sec)  (uses X, Y, Z, A, B, C)
		case 218: AddCommandToQue(SINGLE_STEP); break;//return;// set the fixture offsets  (uses O, X, Y, Z, A, B, C)
		case 219: M_Code_M219(); ContinueToNextStep(); break;//return;// sets the serial baud rate (uses B)
		//RETIRED case 220: AddCommandToQue(SINGLE_STEP);  break;// sets the per motor/axis send motorStep pulse as a CANbus command (uses T,X,Y,Z,A,B,C)
		case 221: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//set extrude calculation factors (uses T,S,Z,W,P)
		case 222: AddCommandToQue(SINGLE_STEP); break;//return;// set the centripetal accelleration radius (uses S)
		case 223: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the re-homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 224: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the jog no ramp speed (UNITS/MIN) (no acceleration needed below this) (uses X, Y, Z, A, B, C)
		case 225: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the jog speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
		case 226: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the jog acceleration constant (uses X, Y, Z, A, B, C, P)
		case 227: AddCommandToQue(SINGLE_STEP); break;//return;// control jogging using the AB encoder on the panel interface (uses XYZABCR>
		case 228: AddCommandToQue(SINGLE_STEP); break;//return;//Disable Automatic Reverse and Prime
		case 229: AddCommandToQue(SINGLE_STEP); break;//return;// extrusion control (uses E, P, S)
		case 230: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;// set global flow rate override percentage (uses S)
		case 231: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;// set motion feedrate override percentage (uses S)
		case 232: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;// set motion override percentage (uses S)
		case 233: AddCommandToQue(SINGLE_STEP); break;//return;// set homing pop-off distance (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 234: AddCommandToQue(SINGLE_STEP); break;//return;// set motor position (absolute) (inch/mm/deg) (uses X, Y, Z, A, B, C)
		case 235: AddCommandToQue(SINGLE_STEP); break;//return;// set rotary axis plane and offset (uses S, P, A, Y, Z)
		case 236: AddCommandToQue(SINGLE_STEP); break;//return;// write device type and revision to flash (option bytes) (uses T, S, I, P)
		case 237: AddCommandToQue(SINGLE_STEP); break;//return;// set cold extrusion prevention parameters (uses T, C, U, L, R)
		case 238: AddCommandToQue(SINGLE_STEP); break;//return;// sets the per motor/axis execute E value (uses T,X,Y,Z,A,B,C)

		case 240: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// turn device switch off (uses T, I)
		case 241: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// turn device switch on (uses T, I)
		case 242: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// control device switch by dutyCycle (uses T, I, S)
		case 243: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// control device switch by pwm (uses T, I, S, P)
		case 244: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// control device switch by temperature (uses T, I, S)
		case 245: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// set switch flag(s) (uses T, I, P, E, H, C)
		case 253: AddCommandToQue(SINGLE_STEP); break; // turn on lathe motor for continuous CW motion (uses S)
		case 254: AddCommandToQue(SINGLE_STEP); break; // turn on lathe motor for continuous CCW motion (uses S)
		case 255: AddCommandToQue(SINGLE_STEP); break; // turn off lathe motor (uses S)
		case 260: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))
		case 261: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))
		case 262: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))
		case 263: AddCommandToQue(SYNCS_WITH_MOTION); break; // control display attached to a head (uses S,P,X,Y,I,J,R))

			
			
		case 600: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// disable all HSS outputs
		case 601: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out1
		case 602: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out2
		case 603: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out3
		case 604: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out4
		case 605: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out5
		case 606: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out6
		case 607: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out7
		case 608: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out8
		case 609: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable HSS out9
		case 610: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD1
		case 611: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD2
		case 612: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD3
		case 613: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD4
		case 614: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD5
		case 615: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD6
		case 616: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD7
		case 617: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable OD8
		case 618: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// enable spi3 relays
		case 619: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// sets the function and output pwm of the selected HSS (uses F, I, S, P, J, H)
		case 620: AddCommandToQue(SINGLE_STEP); break;//return;// Laser global control (uses T, E, F, C, P)
		case 621: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// Laser vector mode control (uses P)
		case 622: AddCommandToQue(SINGLE_STEP); break;//return;// Laser raster mode control (uses O, S, D, P, I)
		case 623: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// Laser pulse (one-shot) mode control (uses P, D)
		case 624: AddCommandToQue(SINGLE_STEP); break;//return;// setup raster image (uses X, Z, I, J)
		case 625: AddCommandToQue(SINGLE_STEP); break;//return;// inkjet vector mode control (uses S, J)
		case 626: AddCommandToQue(SINGLE_STEP); break;//return;// build color index table (uses C, U, A, D)
		case 627: AddCommandToQue(SINGLE_STEP); break;//return;// set job_kill/abort auto move location (uses AXZABC IJKUVW)
		case 628: AddCommandToQue(SINGLE_STEP); break;//return;// arm/disarm digital trigger (uses P, E, R)
		case 629: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// open log file
		case 630: AddCommandToQue(SINGLE_STEP); break;//return;// canbus touch probe control (uses T, S, D, P)
		case 631: AddCommandToQue(SINGLE_STEP); break;//return;  // PickNPlace data (uses T, H, P, A, B, C, D)
		case 632: AddCommandToQue(SINGLE_STEP); break;//return;  // PickNPlace control (uses T, S, H, P, V, F, D,)

		case 660: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//set tool length and diameter
		case 670: M_Code_M670(); ContinueToNextStep(); break;//return;//turn on DD light with Duty Sarg
		case 671: M_Code_M671(); ContinueToNextStep(); break;//return;//turn on hotbed danger backlight with duty  Sarg
		case 672: M_Code_M672(); ContinueToNextStep(); break;//return;//attaches the DDlight to one of the sensor status
		case 673: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;// ContinueToNextStep(); break;//return;//turns light on with code synchronized the gcode execution.
		case 674: AddCommandToQue(SINGLE_STEP); break;//return;//sets it in TURBO MODE
		case 675: M_Code_M675(); ContinueToNextStep(); break;//return;   //sets the response light hss pwm
		case 676: M_Code_M676(); ContinueToNextStep(); break;//return;   //sets the chamber fan pwm
		case 677: M_Code_M677(); ContinueToNextStep(); break;//return;   //sets the control panel buzzer pwm
		case 678: M_Code_M678(); ContinueToNextStep(); break;//return;   //set the laser cross-hair pwm
		case 679: M_Code_M679(); ContinueToNextStep(); break;//return;   //set the vacuum pwm
		case 680: M_Code_M680(); ContinueToNextStep(); break;//return;//sets the z axis home offsets only use if you are sure.
		case 681: M_Code_M681(); ContinueToNextStep(); break;//return;//sets the z axis home offsets only use if you are sure.
		case 682: AddCommandToQue(SINGLE_STEP); break;//return;//Calibrate Z axis sensor locations
		case 683: AddCommandToQue(SINGLE_STEP); break;//return; // set the headroom for the normal serial rx buffer (uses S)
		case 685: AddCommandToQue(SYNCS_WITH_MOTION); break;//return;//M_Code_M685(); ContinueToNextStep(); break;//return;   //sets the air assist pwm
		case 686: M_Code_M686(); ContinueToNextStep(); break;//return; //AddCommandToQue(SINGLE_STEP); break;//return;//returns the machine info string for machine key creation
		case 687: M_Code_M687(); ContinueToNextStep(); break;//return; //AddCommandToQue(SINGLE_STEP); break;//return;//unlock system with machine specific password
		case 688: AddCommandToQue(SINGLE_STEP); break;//return;//RESERVED - DO NOT USE
#ifdef ALLOW_GCODE_SCRIPTS
		case 690: M_Code_M690(); ContinueToNextStep(); break;//return; // add/delete scripts
#endif
		case 698: AddCommandToQue(SINGLE_STEP); break;//return;  // humiture control (uses T, V)
		case 699: AddCommandToQue(SINGLE_STEP); break;//return; // hx711 control (uses T, V, S, O, Z)
		case 701: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M701(); break;//return; // set auto status update rate and page selection (uses S, P, E)
		case 702: AddCommandToQue(UNDEFINED); break;  //M_Code_M702(); break;  // seed a group from another head/group's values (uses T, S)
		case 703: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M703(); break;//return; // add device to Group (uses S, P)
		case 704: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M704(); break;//return; // remove device to Group (uses S, P)
		case 705: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M705(); break;//return; // reset HH (uses P)
		case 706: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M706(); break;//return; // sync HH (uses P)
		case 707: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M707(); break;//return; // STOP HH (uses P)
		case 708: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M708(); break;//return; // pause HH (uses P)
		case 709: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M709(); break;//return; // resume device (from pause or stop) (uses T)
		case 710: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M710(); break;//return; // enable/disable RTD1 (uses S, P)
		case 711: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M711(); break;//return; // enable/disable RTD2 (uses S, P)
		case 712: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M712(); break;//return; // enable/disable RTD3 (uses S, P)
		case 713: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M713(); break;//return; // set default P value for missing argP
		case 714: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M714(); break;//return; // update devicePosition alias table
		case 715: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M715(); break;//return; // set LED display selection
		case 716: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M716(); break;//return; // turns off logging of aux comm in repetrel
		case 717: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M717(); break;//return; // turns on logging of aux comm in repetrel
		case 718: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M718(); break;//return; // used by repetrel to synchronize M719 data logging
		case 719: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M719(); break;//return; // set reporting rate and data for host traffic (uses S, E)
		case 720: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M720(); break;//return; // direct MAIN extrusion control (uses P, F, E, S)
		case 721: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M721(); break;//return; // direct UNPRIME extrusion control (uses P, F, E, S)
		case 722: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M722(); break;//return; // direct PRIME extrusion control (uses P, F, E, S)
		case 723: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M723(); break;//return; // direct MANUAL extrusion control (uses P, F, E, S)
		case 724: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M724(); break;//return; // direct DWELL (no stepping) extrusion control (uses P, F, E, S)
		case 725: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M725(); break;//return; // set the Karl Factors for controlling the heater switch (uses T, P, E, S)
		case 726: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M726(); break;//return; // set the Karl Factors for controlling the fan switch (uses T, P, E, S)
		case 727: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M727(); break;//return; // set LED override values and mask (uses T, S, P)
		case 728: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M728(); break;//return; // set motor current boost control (uses T, S)
		case 729: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M729(); break;//return; // set motor microsteps control (uses T, S)
		case 730: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M730(); break;//return; // set not to exceed temp for motor (uses T, S)
		case 731: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M731(); break;//return; // set not to exceed temp for heater (uses T, S)
		case 732: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M732(); break;//return; // set maximum step rate for motor (microsteps/sec) (uses T, F)
		case 733: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M733(); break;//return; // set maximum allowable RTD temperature delta (uses T, S)
		case 734: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M734(); break;//return; // set HH error reporting rate for redundant error codes (uses T, F)
		case 735: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M735(); break;//return; // fill the incoming page data buffer with S (uses S)
		case 736: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M736(); break;//return; // fill the outgoing page data buffer with S (uses S)
		case 737: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M737(); break;//return; // erase flash page in selected (physical) device (uses T, I, P)
		case 738: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M738(); break;//return; // transfer data page from (physical) device to incoming buffer (uses T, I, P, S)
		case 739: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M739(); break;//return; // transfer data page from incoming to outgoing buffer
		case 740: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M740(); break;//return; // transfer data page from outgoing buffer to (physical) device (uses T, I, P, S)
		case 741: M_Code_M741(); ContinueToNextStep(); break;//return; // transfer incoming page related data from the device to the to host (uses S)
		case 742: M_Code_M742(); ContinueToNextStep(); break;//return; // transfer data page in ASCII text from host to outgoing buffer (uses S, P, comment) <-- can't go in queue because of needing comment
		case 743: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M743(); break;//return; // transfer generally (read only) device info from inbox to host (uses T, S)
		case 744: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M744(); break;//return; // transfer alias list from device to inbox (uses T)
		case 745: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M745(); break;//return; // force HH to invert polarity of direction pin (uses P)
		case 746: M_Code_M746(); ContinueToNextStep(); break;//return; // start the bootloader for the selected physical device (uses T)
		case 747: M_Code_M747(); ContinueToNextStep(); break;//return; // prepare device for download (uses P)
		case 748: M_Code_M748(); ContinueToNextStep(); break;//return; // process next line of intel hex format bootloader data (uses P, comment) <-- can't go in queue because of needing comment
		case 749: M_Code_M749(); ContinueToNextStep(); break;//return; // exit the device bootloader
		case 750: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M750(); break;//return;    // unlock flash for erase/write access for the selected physical device (uses T)
		case 751: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M751(); break;//return;    // lock flash to prevent erase/write access for the selected physical device (uses T)
		case 752: M_Code_M752(); ContinueToNextStep(); break;//return; // write hardware type to flash (option bytes) using device bootloader (uses S)
		case 753: M_Code_M753(); ContinueToNextStep(); break;//return; // write hardware revision to flash (option bytes) using device bootloader (uses  S)
		case 754: M_Code_M754(); ContinueToNextStep(); break;//return; // write hardware key to flash (option bytes) using device bootloader (uses S)
		case 755: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //M_Code_M755(); break;//return;    // set extruder heater pwm (uses T, S)
		case 756: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return; //M_Code_M756(); break;//return;    // set layer height (mm) (uses S)
		case 757: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return; //M_Code_M757(); break;//return;    // set layer/path weight (mm) (uses S)
		case 758: AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return; //M_Code_M758(); break;//return;    // set extrusion step to volume conversion (steps per nL) (uses T, S)
		case 759: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M759(); break;//return;    // enable temperature calibration (uses T, S)
		case 760: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M760();    break;//return; // disable temperature calibration
		case 761: M_Code_M761(); ContinueToNextStep(); break;//return;  // transfer system info in ASCII text from main board to host (uses S, P)
		case 762: M_Code_M762(); ContinueToNextStep(); break;//return;  // transfer system info in ASCII text from host to main board (uses S, P, comment)
		case 763: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M763(); break;//return;    // clear error on selected device(s) (uses T)
		case 766: M_Code_M766(); ContinueToNextStep(); break;//return; // start the system bootloader process
		case 767: M_Code_M767(); ContinueToNextStep(); break;//return; // prepare system  for download (uses P)
		case 768: M_Code_M768(); ContinueToNextStep(); break;//return; // process next line of intel hex format of system bootloader data (uses P, comment) <-- can't go in queue because of needing comment
		case 769: M_Code_M769(); ContinueToNextStep(); break;//return; // exit the system bootloader
		case 770: M_Code_M770(); ContinueToNextStep(); break;//return; // leave system bootloader and jump to application main()
		case 771: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M771(void)  // load laser image data controls (scale, offset, max)
		case 772: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M772(void)  // reset metrics for new job
		case 773: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M773(void)  // send motion metrics to host
		case 774: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M774(void)  // send queue metrics to host
		case 775: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M775(void)  // send current status/queue values to host
		case 776: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M776(void)  /// send cmd/motionQ usage histograms to host
		case 777: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M777(void)  // send and/or erase the crash log (uses S, E, D)
		case 778: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M778(void)  // enable slice time measurement (uses I, S)
		case 779: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M779(void)  // dump slice time measurements
		case 780: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M780(void)  // enable/disable auto XYZABC position reporting
		case 781: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M781(void)  // write hardware type to flash (option bytes) (uses T, D, O, P)
		case 782: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M782(void)  // enable/disable "print air" feature (uses S)
		case 783: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M783(void)  // set PersistantUltimusControlHeadAddress
		case 784: AddCommandToQue(SINGLE_STEP); break;  //M_Code_M784(void)  // report system info (version numbers of system and all heads)
		case 785: AddCommandToQue(SINGLE_STEP); break;//return; //_Code_M785(void)  // Set motor parameters (uses T,U,A,R,B,P,C,O,S) (V1)
		case 786: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M786(void)  // set closed loop stepper PID control values (uses T, P, I, D)
		case 787: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M787(void)  // calibrate can based closed-loop motor (uses T, P, C)
		case 788: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M788(void)  // reset can based closed-loop axis motor (uses T, F, D, P)
		case 789: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M789(void)  // send sideband step pulses to a canAxisMotor (uses T, S)
		case 790: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //notify when layer is changed
		case 791: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //take a picture, add to que so it can fire at the right time
		case 792: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //SEND A TEXT, add to que so it can fire at the right time
		case 793: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //EXECUTE SHELL COMMAND, add to que so it can fire at the right time
		case 794: AddCommandToQue(SYNCS_WITH_MOTION); break;//return; //take a picture, add to que so it can fire at the right time
		case 795: AddCommandToQue(SINGLE_STEP); break;//return; // sets jogvalueValueInUnits (uses S)
		case 796: AddCommandToQue(SINGLE_STEP); break;//return;// Sets the jog increment (uses X, Y, Z, A, B, C)
		case 797: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M797(); break;//return; // enable/disable debug reporting strings ">GB:" (uses S)
		case 798: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M798(); break;//return; // dump strings to host (debug MCODE) -- warning, must reset after using this MCODE (uses T)
		case 799: AddCommandToQue(UNDEFINED); break;//return; //M_Code_M799(); break;//return; // get PLL and Clock status for 407
		case 800: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M800(void)  // sonicator control
		case 868: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M868();  break;//return;// Move table up
		case 869: AddCommandToQue(SINGLE_STEP); break;//return; //M_Code_M869();  break;//return;// Move table down
		case 960: AddCommandToQue(UNDEFINED); break;//return; //M_Code_960 ; rectangluar pocket mill
		case 7734: CommandReadyToProcessFlag = 0; M_Code_M7734(); break;//return; // diags
		default:
			sprintf(_errorStr, "CommandProcessor: Unsupported MCode M%d (~L:%d/%d)", (int)ARG_M, (int)ARG_N, strlen(currentCommandString));
			sendError(_errorStr);
			sprintf(_errorStr, "**%s**", makeAllCharsPrintable(currentCommandString));
			sendError(_errorStr);
			CommandReadyToProcessFlag = 0;
			break;//return;
		}
	}
	else if (ARG_G_PRESENT)
	{
		switch ((int)ARG_G)
		{
		case 0:   AddCommandToQue(ADD_TO_MQ); break;//return;//G_Code_G0();   break;//return;//move rapid
		case 1:
			{
				switch (getArgFraction(ARG_G))
				{
				case 0: AddCommandToQue(ADD_TO_MQ); break;//return;//G_Code_G1();   break;//return;//move at feed rate
				case 1: AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G1_1();   break;//return;//raster move (laser)
				default : break;
				}
				break;
			}
		case 2:   AddCommandToQue(ADD_TO_MQ); break;//return;//G_Code_G2();   break;//return;//move arc cw
		case 3:   AddCommandToQue(ADD_TO_MQ); break;//return;//G_Code_G3();   break;//return;//move arc ccw
		case 4:   AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G4();   break;//return;//Dwell in ms
		case 12:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G12();  break;//return;//CIRCULAR BORE CW
		case 13:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G12();  break;//return;//CIRCULAR BORE CCW
		case 16:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G16(); //set arc plane to NONE (user defineable)
		case 17:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G17(); //set arc plane to XY
		case 18:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G18(); //set arc plane to XZ
		case 19:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G19(); //set arc plane to YZ
		case 20:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G20();  break;//return;//inch dimensions
		case 21:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G21();  break;//return;//mm dimension
		case 28:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G28();  break;//return;//home axis, responds to individual axis if args are passed, X0Y0 homes only x and y not Z
		case 29:  AddCommandToQue(UNDEFINED); break;//return;//G_Code_G29();  break;//return;//special exercise move
		case 38:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G38();  touch probe
		case 53:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G53();   set fixture offsets OFF
		case 54:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G54();   set fixture offsets ON
		case 55:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G55();   set fixture offsets ON
		case 56:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G56();   set fixture offsets ON
		case 57:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G57();   set fixture offsets ON
		case 58:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G58();   set fixture offsets ON
		case 59:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G59();   set fixture offsets ON
		case 70:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G70(void);//G70 Bolt Hole Circle
		case 71:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G71(void);//G71 Bolt Hole Arc
		case 73:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G73(void);//G73 High-Speed Peck Drilling Canned Cycle
		case 74:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G74(void);//G74 Reverse Tap Canned Cycle
		case 76:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G76(void);//G76 Fine Boring Canned Cycle
		case 77:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G77(void);//G77 Back Bore Canned Cycle
		case 80:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G80(void);//G80 Canned Cycle Cancel
		case 81:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G81(void);//G81 Drill Canned Cycle
		case 82:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G82(void);//G82 Spot Drill Canned Cycle
		case 83:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G83(void);//G83 Normal Peck Drilling Canned Cycle
		case 84:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G84(void);//G84 Tapping Canned Cycle
		case 85:  AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G85(void);// G85 Boring Canned Cycle
		case 90:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G90();  break;//return;//absolute programming coordinates
		case 91:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G91();  break;//return;//incremental programming
		case 92:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G92();  break;//return;//set G92 offsets
		case 93:  AddCommandToQue(IMPACTS_DESTINATION_CALC); break;//return;//G_Code_G93();  break;//return;//clea G92 offsets
		//NUKE case 101: AddCommandToQue(SINGLE_STEP);  break;//return;//G_Code_G101();   break;//return;//raster move (laser)
		case 702: AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G702();  break;//return;//cylinder print
		case 703: AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G703();  break;//return;//cylinder print
		case 928: AddCommandToQue(SINGLE_STEP); break;//return;//G_Code_G928();//walk motor back and set home to destination.
		default:
			sprintf(_errorStr, "CommandProcessor: Unsupported GCode G%d (~L:%d/%d)", (int)ARG_G, (int)ARG_N, strlen(currentCommandString));
			sendError(_errorStr);
			sprintf(_errorStr, "**%s**", makeAllCharsPrintable(currentCommandString));
			sendError(_errorStr);
			CommandReadyToProcessFlag = 0;
			break;//return;
		}
	}
	else if (GCodeArgT[0] == 'T')
	{
		// if you get here the line did NOT have a G code or a M code, rather ONLY a 'T' command
		// we can convert it to a M6 command and pass it on
		// ie, M6 T12 O2   means use print head 2 and offset 2, if there is a X Y or Z
		// command then it defines the NEW Offset

		int toolNumber = (int)ARG_T;
		InvalidateAllCmdArgs(ExecutionPtr); // make sure no unexpected args were set
		// build up and M6 command:
		ARG_M = 6.0f; // setting up M6
		ARG_I = _blockImmediateMoveOnToolChange ? 0.0f : 1.0f; // 1.0 == force immediate move to new offset location

		if ((toolNumber >= 0) && (toolNumber <= MAX_TOOL_NUMBER))
		{
			ARG_T = (float)convertToolNumberToDevice(toolNumber);
			if (!deviceIsAHotbed((byte)ARG_T))
			{
				// only spec an offset for non hotbeds
				ARG_O = toolNumber + 1; // offset in motor array is + 1
			}
		}
		else if ((toolNumber >= ALIAS_TOOL_NUMBER_MIN) && (toolNumber <= ALIAS_TOOL_NUMBER_MAX))
		{
			// must be using a group
			ARG_T = (float)toolNumber;
			ARG_O = toolNumber + 1; // offset in motor array is + 1
		}
		else
		{
			// error!!!!
			ARG_T = (float)HH_POSITION_UNPLUGGED;
		}

		if (ARG_T != (float)HH_POSITION_UNPLUGGED)
		{
			// only process if valid T arg was given
			AddCommandToQue(SINGLE_STEP); //send M6 to cmdQue on for later processing in sequence
		}
	}
	CommandReadyToProcessFlag = 0;
}


void SequenceEngine()
{
	//this will execute the command stored in the que, if there are any waiting to get executed

	// this must be ahead of any dwell timers to ensure it counts down - SAFETY first!

	if (_needToProcessDeferredCommands)
	{
		// path from ProcessMotion to process deferred commands
		processNextDeferredCommand();
		return;
	}

	if (_g4DwellTimer > 0) return;

	if ((CommandsInQue == 0) && motionQ_empty())
	{
		// only countdown watchdog after all work is complete
		updateHostConnectionWatchDog();
	}

	if (PrimeTimer >  0)
	{
		// waiting for prime to complete (prime preamble)
		PrimeTimer--;
		if (PrimeTimer == 0)
		{
#ifdef GB_PRIME_PIN
			pinClear(GB_PRIME_PIN);
#endif
			motionQ_primeComplete();
			if (motionQ_notEmpty() && !_gcodePaused)
			{
				// got here from a move in the motionQ
				//motionQ_executeMove();
				StartMove();
			}
		}
		return;
	}

	if (UnPrimeTimer > 0)
	{
		// waiting for unprime to complete (unprime preamble)
		UnPrimeTimer--;
		if (UnPrimeTimer == 0)
		{
#ifdef GB_RESIDUAL_PIN
			pinClear(GB_RESIDUAL_PIN);
#endif
#ifdef GB_UNPRIME_PIN
			pinClear(GB_UNPRIME_PIN);
#endif
			if (motionQ_notEmpty())
			{
				// got here from the motionQ -- should be only way to get here
				motionQ_unprimeComplete();
				motionQ_deleteCurrentMoveAndExecuteNextMove();
			}
		}
		return;
	}

	if (_gcodePaused) return;

	if (motionQ_getCountdownDelayToExecuteMove())
	{
		if ((motionQ_numEntries() >= 2) && (motionQ_getCountdownDelayToExecuteMove() < MOTIONQ_FIRST_ENTRY_AGE_TIME_MS))
		{
			// have a second move in place.... so shorten the wait ONLY is the wait is less than the first entry AGE time (so don't over shorten the MOTOR_EN delay)
			motionQ_setCountdownDelayToExecuteMove(1);
		}
		motionQ_decrCountdownDelayToExecuteMove();
		if (motionQ_getCountdownDelayToExecuteMove() == 0)
		{
			// finished letting first element in the motionQ "age" to allow other commands to enter the queue
			motionQ_executeMove();
			return;
		}
	}

	if ((G2G3Flag > 0) || G203State)
	{
		// working on a multi-cycle move
		return;
	}
	if (_arc.index > 0)
	{
		return;
	}

	if (_abortInProgress)   // this check must be ahead of any moves;
	return;

	if (CannedCycleFlag)
	{
		//come here to process a canned cycle
		//ok if you get here, the motion is complete and we are just going to process the next step
		processCannedCycle();
		return;
	}

	if (ExecuteImmediateFlag == 2)
	{
		//execute out of order when this flag is set to 2
		if (motionQ_full()) return;  // can't always process command, so make sure there's room
		if ((cmdQue[CurrentCommandIndex].G == 928.0f) && (!motionQ_empty())) return; // prevent multiple 928's from stepping on each other (NUKE when GUI no longer sends 911 in front of G928's)
		CurrentCommandIndex = 1; //point to cmdQue[1] for the prams to execute
		processCommand(&cmdQue[CurrentCommandIndex]);
		if (CannedCycleFlag == 0)
			ExecuteImmediateFlag = 0; //turn off the flag (will be shut off after multipass homing, etc
	}
	else
	{
		if (CommandsInQue == 0)
		{
			return;             //dont do anything if we are not ready to process it
		}

#ifdef ADD_ON_SPI_DISPLAY
		if ((GUI_CommandQueueFull() || _waitingForGuiCommand))
		{
			return;
		}
#endif //ADD_ON_SPI_DISPLAY

		if (_MailBoxes._waitingFor.flags.u32)
		{
			//waiting for something, so can't proceed
			return;
		}

		//add more checks for temperature wait and dwell here....RPT  XXX

		// if flow gets here, the last command is complete, so remove it from the cmdQue and execute the next command
		// with the motionQ, program flow needs to alter to keep the cmdQue in sync with the motion queue.
		// ie if G1/G2/G3 occurred but the motionQ was full, need to wait until the mQ frees up a slot.
		// if a non G1/G2/G3 occurs and the motionQ is not empty, need to wait for it to empty before proceeding _motionQ_ForceQtoEmpty will be used
		// G2/G3 will decompose into many small moves which will be stuffed into the motionQ

		NextExecutionPtr = &cmdQue[NextCommandIndex];
		if (motionQ_full())
			return;     // can't do anything

		if (_motionQ_ForceQtoEmpty)
		{
			// in single step mode
			if (motionQ_empty())
				_motionQ_ForceQtoEmpty = FALSE;
			else  // if (motionQ_notEmpty())
			return;
		}
		if (_requestToPauseAtEndOfLayer && ((int)NextExecutionPtr->M == 790))
		{
			// special case of M790 where we need to stop - so do this in single step mode to not
			// process past the pause in motion
			NextExecutionPtr->cmdType = SINGLE_STEP;
		}

		if (motionQ_empty())
		{
			// can do anything
			if ((NextExecutionPtr->cmdType == SINGLE_STEP) || (NextExecutionPtr->cmdType == UNDEFINED))
				_motionQ_ForceQtoEmpty = TRUE;
			; // now fall through and process G/M code
		}
		else if (NextExecutionPtr->cmdType == ADD_TO_MQ)
		{
			// let this fall through to process the G1/G2/G3 since there is room in the queue and not forcing queue to drain
			// command will be deleted from cmdQue after move finishes.
			; // now fall through and process G/M code
		}
		else if (_jogging.enabled)
		{
			return; // don't processes other commands until the jog is complete (no mechanism to defer commands when jogging
		}
		else if (NextExecutionPtr->cmdType == IMPACTS_DESTINATION_CALC)
		{
			// can still process these in order as only impact dest calc which happens immediately on adding to motionQ
			; // now fall through and process G/M code
		}
		else if (NextExecutionPtr->cmdType == SYNCS_WITH_MOTION)
		{
			// TAG is as part of newest (most recently added) mQ entry and then move to deferred fifo
			// for later retrieval (when "newest" move completes, this will be executed .. in sync with motion)
			// entry is still be active in cmdQue
			if (deferredCmdQueueFull())
				return;  // need to wait for things to empty out
			NextExecutionPtr->cmdLink = LastCmdQueIndexAddedToMotionQ; // create connection to "newest" mQ entry
			motionQ.newest->flags.hasDeferredCommands = TRUE;
			copyCmdQueEntryToDeferredCmdQue(NextExecutionPtr);
		}
		else if ((NextExecutionPtr->cmdType == SINGLE_STEP) || (NextExecutionPtr->cmdType == UNDEFINED))
		{
			return;  // need to wait for the motionQ to empty
		}
		else
		{
			barf("SeqEng(): should never get here");
			return;
		}

		// to get here, either (the motionQ is empty) OR (the motionQ is not full AND not starting a G1/G2/G3/etc to the queue)
		// everything else needs to single step.

		CommandsInQue--; //count down the number of commands
		CurrentCommandIndex = NextCommandIndex; //set the working pointer to the right spot in the execution que
		NextCommandIndex = getNextCommandIndex(NextCommandIndex);
		if ((motionQ_notEmpty()) && (NextExecutionPtr->cmdType == SYNCS_WITH_MOTION))
			return;  // this type of command will be processed later
		// if we get here, we can process the next command
		processCommand(&cmdQue[CurrentCommandIndex]);
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkEMO(void)
{
	//EMO PRESSED == SENSOR_CLOSED/TRIPPED == NO VOLTAGE
#ifdef HYDRA_DIAGS
	if (_diagsEnabled) return;
#endif
#ifdef USE_HYREL_IO
	return;
#endif
	if (EMO.Enabled && (_gs._milliseconds > IGNORE_EMO_DURING_BOOT_TIME_MS))
	{
		readDebouncedSensorState(&EMO);

#ifdef GB_HIDDEN_WARNINGS
		int doWeNeedToAddATransientDetectOnEMO;
#endif //GB_HIDDEN_WARNINGS
		//      if (EMO.TransientDetected)
		//      {   // either noise, or start of transition
		//          _highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
		//      }
		//      else
		if (EMO.StateChangeDetected && (EMO.State == SENSOR_TRIPPED))
		{
			// EMO has been pressed (killing power) OR at the end of the boot time window and tripped/closed

			_highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
			sendEmoMessage();
			requireAllAxesToHome();
			_blockAllMotion = TRUE;
			_blockAbsoluteMotion = TRUE;
			if (motionQ_notEmpty())
			{
				// there's at least one move active/in the queue, so need to flush the queue and stop any future moves
				motionQ_abortMotionWhenSafe(FLUSH_THEN_ABORT);
			}
		}
		else if (EMO.StateChangeDetected && (EMO.State == SENSOR_OPEN))
		{
			// EMO has been released, so power should be ramping up -- OR first time after bootup
			sendEmoMessage();
			_blockAllMotion = FALSE;
			_blockAbsoluteMotion = _motionSensorTripped; // release block if no faults are present. will allow jogging, but no absolute motion until rehomed.
			_highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
		}
		else if (EMO.State == SENSOR_TRIPPED)
		{
			// still no power, so reset coundown to max
			_highVoltageIsNotStableCoundownMs = EMO_HIGH_VOLTAGE_SETTLING_TIME_MS;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void readInputs(void)
{

	checkStartButton();
	checkEMO();

	//checkABEncoderSelectButton(); //USE_AB_ENCODER

	PWMCntrl();
	Head11_Temperature = _MailBoxes._inbox[0].HBPayload[0].i16[0] >> 5;
	Head11_HTRDuty = _MailBoxes._inbox[0].HBPayload[0].i16[1];
	Head11_FanDuty = _MailBoxes._inbox[0].HBPayload[0].i16[2];
	Head11_Spare = _MailBoxes._inbox[0].HBPayload[0].i16[3];
}


////////////////////////////////////////////////////////////////////////////////

// These three methods are called in sequence from SysTick lopp.  For each pass, a new check of all sensors is performed and the
// valiable _motionSensorTripped is updated.

void checkMotorFaultSensor(void)
{
	_motionSensorTrippedNext = FALSE; //checkMotorSensor(SENSOR_INDEX_FAULT);
}


void checkMotorLimit1Sensor(void)
{
	_motionSensorTrippedNext |= FALSE; //checkMotorSensor(SENSOR_INDEX_LIMIT1);
}
////////////////////////////////////////////////////////////////////////////////

boolean  checkMotorSensor(sensorIndex_t sensorIndex)
{
	// this will check for any faults.  if a new fault is found, it is reported to repetrel
	// and that axis is tagged as having a fault (no more absolute motion)

	// if ANY FAULT detected .....
	// if ANY MOTION had occurred since reset, then motion is stopped, queues are
	//     flushed and ALL motion is blocked until reset/abort
	// if NO MOTION had occurred since reset, then just block absolute moves. this
	//     allows users to reset the system and be able to jog non-faulted motors
	// after fault(s) have been removed, any faulted axis will need to be
	//      re-homed
	//
	// returns TRUE if any sensor is tripped

#ifdef HYDRA_DIAGS
	if (_diagsEnabled) return (FALSE);
#endif
#ifdef USE_HYREL_IO
	return (FALSE);
#endif

	MotorStructure *M;
	boolean newFaultFound = FALSE;
	boolean anyFaultFound = FALSE;

	for (M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
	{
		if (M->MotorInstalled && !((M == &Motors[M_C]) && (sensorIndex == SENSOR_INDEX_LIMIT1)))
		{
			// only check if motor is installed and NOT the special case (C_L1 is used for EMO)
			sensorStruct *sensorPtr = getMotorSensorPtr(M, sensorIndex);
			if (sensorPtr == (sensorStruct *)NULL)
				continue;   // skip over undefined sensors

			if (_highVoltageIsNotStableCoundownMs)
			{
				// power not good, so reset the sensor to unknown state
				setSensorStateToUnknown(sensorPtr);
			}
			else if (sensorPtr->Enabled)
			{
				boolean stateWasUnknown = (sensorPtr->State == SENSOR_STATE_UNKNOWN); // first time checking sensor after booting or MCODE to enable sensor

				readDebouncedSensorState(sensorPtr);

				if (sensorPtr->StateChangeDetected)
				{
					// special cases for detected change in state
					if (sensorPtr->State == SENSOR_CLOSED)
					{
						// new fault detected
						newFaultFound = TRUE;
						anyFaultFound = TRUE;

						sprintf(_errorStr, "%s sensor detected on %c axis %s", sensorPtr->Name, M->AxisLabel, stateWasUnknown ? "reset/enabled" : "");
						sendMotionError('F', M->AxisLabel, M->POSITION * M->UnitsPerPulse, 0, _errorStr);

						if (M->HomeSense.Enabled)
						{
							// if the Axis has a home sensor, FORCE A REHOMING OF JUST THE FAULTED AXIS
							M->HasBeenHomed = FALSE;
							M->HasBeenHomed_BlockedMsgSent = FALSE; // re-arm message in case users tries to move without homing
						}

						if (sensorIndex == SENSOR_INDEX_LIMIT1)
							M->Limit1Sense.State = TRUE;
						else if (sensorIndex == SENSOR_INDEX_LIMIT2)
							M->Limit2Sense.State = TRUE;
					}
					else if (sensorPtr->State == SENSOR_OPEN)
					{
						sensorPtr->ErrorMessageSent = FALSE; // re-arm for next time
						if (sensorIndex == SENSOR_INDEX_LIMIT1)
							M->Limit1Sense.State = FALSE;
						else if (sensorIndex == SENSOR_INDEX_LIMIT2)
							M->Limit2Sense.State = FALSE;
					}
				}
				else if (sensorPtr->State == SENSOR_CLOSED)
				{
					// decision based on current state
					anyFaultFound = TRUE;
				}
			}
		}
	}

	if (newFaultFound)
	{
		// at least one axis has a newly discovered fault, so report it and take action
		// report is created here to give one line for a given sensor across all motors

		sprintf(_rptStr, ">MF");
		for (M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
		{
			// whiz through all the motors, if the motor is installed, then report (shows full XYZ position where fault occurred
			if (M->MotorInstalled && !((M == &Motors[M_C]) && (sensorIndex == SENSOR_INDEX_LIMIT1)))
			{
				// only check if motor is installed and NOT the special case (C_L1 is used for EMO)
				sensorStruct *sensorPtr = getMotorSensorPtr(M, sensorIndex);
				if (sensorPtr->Enabled && (sensorPtr->State == SENSOR_TRIPPED))
				{
					sprintf(_tmpStr, ":%c_%s", M->AxisLabel, sensorPtr->Name);
				}
				else
				{
					sprintf(_tmpStr, ":%c%4.3f", M->AxisLabel, M->POSITION * M->UnitsPerPulse);
				}
				strcat(_rptStr, _tmpStr);
			}
		}
		strcat(_rptStr, ":POSITION when sensor was detected");
		sendInfo(_rptStr);

		_blockAllMotion = TRUE;
		if (motionQ_notEmpty())
		{
			// there's at least one move active/in the queue, so need to flush the queue and stop any future moves
			motionQ_abortMotionWhenSafe(FLUSH_THEN_CONTINUE);
		}
	}

	if (anyFaultFound)
	{
		_blockAbsoluteMotion = TRUE;
	}

	return (anyFaultFound);
}


void checkMotorLimit2Sensor(void)
{
	_blockAbsoluteMotion = FALSE;
	return;
	_motionSensorTrippedNext |= checkMotorSensor(SENSOR_INDEX_LIMIT2);
	_motionSensorTripped = _motionSensorTrippedNext; // update variable based on latest pass

	if ((_motionSensorTripped == FALSE) && !(EMO.Enabled && (EMO.State == SENSOR_TRIPPED)) && !_highVoltageIsNotStableCoundownMs)
	{
		_blockAbsoluteMotion = FALSE;
	}
}


////////////////////////////////////////////////////////////////////////////////

void checkBlockingWaits(void)
{
	// called from 10Hz
	if (_MailBoxes._waitingFor.flags.u32)
	{
		// at least one block;
		if (_MailBoxes._waitingFor.flags.bit.motionPnP)
		{
			_MailBoxes._waitingFor.timerPnP--;
			if (_MailBoxes._waitingFor.timerPnP <= 0)
			{
				// timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.motionPnP = FALSE;
				sendError("PNP Motion Timeout");
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.extruderTemp)
		{
			waitingForExtruderTemp();
		}
		if (_MailBoxes._waitingFor.flags.bit.hotbedTemp)
		{
			waitingForHotbedTemp();
		}
		if (_MailBoxes._waitingFor.flags.bit.chamberTemp)
		{
			waitingForChamberTemp();
		}
		if (_MailBoxes._waitingFor.flags.bit.canbusProbeToArm)
		{
			_MailBoxes._waitingFor.timerProbe--;
			if (_MailBoxes._waitingFor.timerProbe <= 0)
			{
				// timed out, so just move on
				TouchProbeFinished(PROBE_RESULTS_CANBUS_PROBE_ARMING_TIMEOUT, PROBE_TYPE_UNKNOWN, PROBE_ERROR);
				_MailBoxes._waitingFor.flags.bit.canbusProbeToArm = FALSE;
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.canAxisMotorHoming)
		{
			_MailBoxes._waitingFor.timerHoming--;
			if (_MailBoxes._waitingFor.timerHoming <= 0)
			{
				// timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.canAxisMotorHoming = FALSE;
				sendError("HOMING TIMED OUT");
				// reset any canAxisMotors that did not finish...
				MotorStructure *M;
				for (M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
				{
					// whiz through all the motors and clean up any flags
					if (M->canMotor)
					{
						M->axisSelfHomingInProgress = FALSE;
						for (int canAddrIndex = 0; canAddrIndex < M->maxCanMotors; canAddrIndex++)
						{
							// check if all them are finished
							if (M->canMotors[canAddrIndex].canAddress)
							{
								if (M->canMotors[canAddrIndex].selfHomingInProgress)
								{
									M->canMotors[canAddrIndex].selfHomingInProgress = FALSE;
									sendMotorResetToDevice(M->canMotors[canAddrIndex].canAddress);
								}
							}
						}
					}
				}
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.canLoopback)
		{
			_MailBoxes._waitingFor.timerCanLoopback--;
			if (_MailBoxes._waitingFor.timerCanLoopback <= 0)
			{
				// timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.canLoopback = FALSE;
			}
		}
		if (_MailBoxes._waitingFor.flags.bit.canGuiAck)
		{
			_MailBoxes._waitingFor.timerCanGuiAck--;
			if (_MailBoxes._waitingFor.timerCanGuiAck <= 0)
			{
				// timed out, so just move on
				_MailBoxes._waitingFor.flags.bit.canGuiAck = FALSE;
			}
		}
	}
}


////////////////////////////////////////////////////////////////////////////////

void checkForCompletedAbort(void)
{
	if (_abortFinisedNeedToResetProcessSynchronously)
	{
		// need to reset all comm/motion pointers after an abort but not in the middle of any slices running or self-init
		ResetProcess(1);
		_heartbeatRateControl = HEARTBEAT_MODE_NORMAL; // return to normal
#ifdef GB_ABORT_PIN
		pinClear(GB_ABORT_PIN); // signal to logic analyzer
#endif
		if (EMO.State == SENSOR_TRIPPED)
		{
			// need to disable and can axis motors as they may be on a diff power supply
			DisableAllMotionMotors();
		}
		if (_abortOccurredWhileMoving)
		{
#ifdef GB_HIDDEN_WARNINGS
#warning "maybe move to BEFORE Reset process"
#warning "maybe want to change to abortWhenCmdQueNotEmpty????" //goal is to not do this move ANYTIME a reset is sent in, but only during a print
#endif //GB_HIDDEN_WARNINGS
			_abortOccurredWhileMoving = FALSE;
			MotorStructure *M;

			ExecutionPtr = &cmdQue[0]; // ensure pointing to valid memory after the abort

			//process the abort move set up by M627
			InvalidateAllCmdArgs(ExecutionPtr);
			ARG_G = 98765.0f; // just for traceability if an error is generated downstream
			ARG_N = _gcodeLineNumber;
			for (M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
			{
				// whiz through all the motors, if there's a valid ARG from the command, then load max distance
				if (M->AbortRelativePosition != INVALID_ARG_VALUE)
				{
					if ((M->Q_LastRequestedPositionInUnits + M->AbortRelativePosition) < 0.0f)
						setMotorArgInNativeUnits(M, 0.0f - M->Q_LastRequestedPositionInUnits);
					else if ((M->Q_LastRequestedPositionInUnits + M->AbortRelativePosition) > M->MaximumTravelInUnits)
						setMotorArgInNativeUnits(M, M->MaximumTravelInUnits - (M->Q_LastRequestedPositionInUnits));
					else
						setMotorArgInNativeUnits(M, M->AbortRelativePosition);
				}
			}

			_IncrementalMove = TRUE;
			motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);

			InvalidateAllCmdArgs(ExecutionPtr);
			for (M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
			{
				// whiz through all the motors, if there's a valid ARG from the command, then load max distance
				setMotorArgInNativeUnits(M, M->AbortAbsolutePosition);
			}
			_IncrementalMove = FALSE;
			motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
#ifdef RESET_ABORT_MOVE_ON_JOB_END //DO WE WANT TO MAKE THIS NON-PERSISTENT
			// job is really finished, so invalidate these (not persistent)
			for (M = FirstAxisMotorPtr; M <= LastAxisMotorPtr; M++)
			{
				// whiz through all the motors, if there's a valid ARG from the command, then load max distance
				M->AbortRelativePosition = INVALID_ARG_VALUE;
				M->AbortAbsolutePosition = INVALID_ARG_VALUE;
			}
#endif //RESET_ABORT_MOVE_ON_JOB_END
		}
	}
}

boolean cmdQueIsFull(void)
{
	if (CommandsInQue >= (SIZE_OF_COMMAND_QUEUE - 3)) // 2 entries reserved for processing and immediate mode (911); and 1 more for the command be executed
	return (TRUE);
	else
		return(FALSE);
}
void ContinueToNextStep(void)
{
	//resets the flag so we can proceed on to the next instruction, will eventually log or announce if this has occurred
	CommandReadyToProcessFlag = 0;
	ExecuteImmediateFlag = 0;
}
////////////////////////////////////////////////////////////////////////////////

void checkInWithHeads(void)
{
	if ((counter_1Hz % MOTION_TO_HH_CHECKIN_RATE) == 0)
	{
		sendSetCommPingBit(0);
	}
}

void ResetGcodeParseBuffer()
{
	//this will reset the ascii input buffers by putting a null in the first character
	GCodeArgComment[0] = 0; //set comment to null as well
	GCodeArgComment[1] = 0; //many routines look past the SOAPSTRING_CHAR location for the comment string.
	GCodeArgA[0] = 0;
	GCodeArgB[0] = 0;
	GCodeArgC[0] = 0;
	GCodeArgD[0] = 0;
	GCodeArgE[0] = 0;
	GCodeArgF[0] = 0;
	GCodeArgG[0] = 0;
	GCodeArgH[0] = 0;
	GCodeArgI[0] = 0;
	GCodeArgJ[0] = 0;
	GCodeArgK[0] = 0;
	GCodeArgL[0] = 0;
	GCodeArgM[0] = 0;
	GCodeArgN[0] = 0;
	GCodeArgO[0] = 0;
	GCodeArgP[0] = 0;
	GCodeArgQ[0] = 0;
	GCodeArgR[0] = 0;
	GCodeArgS[0] = 0;
	GCodeArgT[0] = 0;
	GCodeArgU[0] = 0;
	GCodeArgV[0] = 0;
	GCodeArgW[0] = 0;
	GCodeArgX[0] = 0;
	GCodeArgY[0] = 0;
	GCodeArgZ[0] = 0;
	GCodeArgSplat[0] = 0;
}

void SplitCurrentGcodeLine2Arguments(buffer_t useBuffer)
{
	int ParseIndex = 0;
	int rawChar; // MUST be of type "int" to handle char 0 to 255 AND the -1 "no data" return value
	int csIndex;
	_checkLineChecksum = FALSE;
	_lineChecksum = 0;
	ProcessingError = 0;
	currentArgLength = 0; //reset ascii length counter
	GCodeArgPtr = &GCodeArgComment[0]; //start by pointing to the comment selection first
	currentCommandString[0] = '\0'; // rebuild command string as it's processed in case of error (to report full line)
	for (ParseIndex = 0; ParseIndex < (MAX_STRING_SIZE - 1); ParseIndex++)  // -1 to save room for the \0 to terminate the string
	{
		//255 characters max, then throw error -- GB XXX gregkarl ... actually, no error checking
		rawChar = GCHAR(useBuffer); // pull it from the correct receive buffer, you will get a -1 if there are no more characters;
		if (rawChar == -1)
		{
			// underRun .... should never happen here as a full command was needed to get this far
			reportRxUnderRun(useBuffer, "SplitCurrentGcodeLine2Arguments");
			CommandReadyToProcessFlag = 0;
			return; // no reason to continue (really a fatal error)
		}
		currentCommandString[ParseIndex] = rawChar;
		currentCommandString[ParseIndex + 1] = '\0'; // set the null term char for the string

		switch (rawChar)
		{
		case   0:                                       return; //null end of the string
		//filtered  out already: case  7:   pingReply();   return;
		case  CMD_END_CHAR:
			processEndofCommandChar(currentCommandString);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;//  \n  carriage return;
		case  13:   break;  // skip char
		case '1':   *GCodeArgPtr = '1'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '2':   *GCodeArgPtr = '2'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '3':   *GCodeArgPtr = '3'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '4':   *GCodeArgPtr = '4'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '5':   *GCodeArgPtr = '5'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '6':   *GCodeArgPtr = '6'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '7':   *GCodeArgPtr = '7'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '8':   *GCodeArgPtr = '8'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '9':   *GCodeArgPtr = '9'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '0':   *GCodeArgPtr = '0'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '.':   *GCodeArgPtr = '.'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '-':   *GCodeArgPtr = '-'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case '+':   *GCodeArgPtr = '+'; CleanUpPointers(); break;//POINT TO THE NEXT POSITION PLEASE
		case ' ':   break;//*GCodeArgPtr=' ';GCodeArgPtr++;*GCodeArgPtr=0; break;//POINT TO THE NEXT POSITION PLEASE

		//now process the command characters
		case SOAPSTRING_CHAR:
			FillArgumentBuffer(&GCodeArgComment[0], SOAPSTRING_CHAR);
			CopyToEndOfLine(useBuffer);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;

		case STRING_DELIM:
			FillArgumentBuffer(&_GcodeArgStringParam[0], STRING_DELIM);
			CopyToEndOfLine(useBuffer);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;

		case 'A':   FillArgumentBuffer(&GCodeArgA[0], 'A'); break;
		case 'B':   FillArgumentBuffer(&GCodeArgB[0], 'B'); break;
		case 'C':   FillArgumentBuffer(&GCodeArgC[0], 'C'); break;
		case 'D':   FillArgumentBuffer(&GCodeArgD[0], 'D'); break;
		case 'E':   FillArgumentBuffer(&GCodeArgE[0], 'E'); break;
		case 'F':   FillArgumentBuffer(&GCodeArgF[0], 'F'); break;
		case 'G':   FillArgumentBuffer(&GCodeArgG[0], 'G'); break;
		case 'H':   FillArgumentBuffer(&GCodeArgH[0], 'H'); break;
		case 'I':   FillArgumentBuffer(&GCodeArgI[0], 'I'); break;
		case 'J':   FillArgumentBuffer(&GCodeArgJ[0], 'J'); break;
		case 'K':   FillArgumentBuffer(&GCodeArgK[0], 'K'); break;
		case 'L':   FillArgumentBuffer(&GCodeArgL[0], 'L'); break;
		case 'M':   FillArgumentBuffer(&GCodeArgM[0], 'M'); break;
		case 'N':   FillArgumentBuffer(&GCodeArgN[0], 'N'); break;
		case 'O':   FillArgumentBuffer(&GCodeArgO[0], 'O'); break;
		case 'P':   FillArgumentBuffer(&GCodeArgP[0], 'P'); break;
		case 'Q':   FillArgumentBuffer(&GCodeArgQ[0], 'Q'); break;
		case 'R':   FillArgumentBuffer(&GCodeArgR[0], 'R'); break;
		case 'S':   FillArgumentBuffer(&GCodeArgS[0], 'S'); break;
		case 'T':   FillArgumentBuffer(&GCodeArgT[0], 'T'); break;
		case 'U':   FillArgumentBuffer(&GCodeArgU[0], 'U'); break;
		case 'V':   FillArgumentBuffer(&GCodeArgV[0], 'V'); break;
		case 'W':   FillArgumentBuffer(&GCodeArgW[0], 'W'); break;
		case 'X':   FillArgumentBuffer(&GCodeArgX[0], 'X'); break;
		case 'Y':   FillArgumentBuffer(&GCodeArgY[0], 'Y'); break;
		case 'Z':   FillArgumentBuffer(&GCodeArgZ[0], 'Z'); break;
		case '*':   FillArgumentBuffer(&GCodeArgSplat[0], '*');
			// at this point, we hit the marker for the checksum, so calc checksum for portion
			//. of the line already received -- from the reprap gcode page:
			for (csIndex = 0; csIndex < ParseIndex; csIndex++)
			{
				// calc checksum
				//_lineChecksum = _lineChecksum ^ (byte)currentCommandString[csIndex];  // reprap style -- terrible coverage
				_lineChecksum += (byte)currentCommandString[csIndex];
			}
			if (useBuffer == URGENT_BUFFER)
			{
				// the urgent char is stripped upstream, but is included in the incoming checksum.
				_lineChecksum += URGENT_911_CMD_CHAR;
			}
			_checkLineChecksum = TRUE;
			break;
		default:
			ProcessingError = 1;
			purgeTillCr(useBuffer, ParseIndex);
			if (_sendingGBStringsMask & GB_STRING_ECHO_COMM) // use M797 S<mask> to enable
			{
				sendGB(currentCommandString);
			}
			return;   // drop the command with unknown char and purge until Cr
		}
		if (ParseIndex == (MAX_STRING_SIZE - 2))
		{
			sendError("Command string too long");
		}
	}
}
void processArgs(char *WorkBuffer, float *OutPutVariable)
{
	if (*WorkBuffer == 0)
	{
		// arg not present
		*OutPutVariable = INVALID_ARG_VALUE; //set to invalid value so we will not accidentally take a zero as a position or temperature argument
		return;
	}
	WorkBuffer++; //MOVE OVER 1 CHAR SO WE CAN PROCESS THE NUMBER, NOT THE KEY LETTER
	if (*WorkBuffer == 0)
	{
		// no value to convert
		ProcessingError = 1;
		*OutPutVariable = INVALID_ARG_VALUE; //set to invalid value so we will not accidentally take a zero as a position or temperature argument
		return;
	}
	*OutPutVariable = atof(WorkBuffer); //start with the second character in the string, because the first character is the argument header char, like M or G or X  etc.
	if (_sendingGBStringsMask & GB_STRING_WORKBUFFER)  // use M797 S<mask> to enable
	{
		sendGB(WorkBuffer);
	}
}



void printSerialInputStreamError(void)
{
	if (ProcessingError > 0)
	{
		sprintf(_errorStr, "serialProcessor: Invalid ascii characters, or variables too long (%d char max) on line %d", MAX_CHARS_FOR_PARAMETER, _gcodeLineNumber);
		sendError(_errorStr);
		sprintf(_errorStr, "**%s**", makeAllCharsPrintable(currentCommandString));
		sendError(_errorStr);
	}
}

void processEndofCommandChar(char *currentCommandString)
{
	CommandReadyToProcessFlag = 1;
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
		PostAcknowledge();
#endif //ALLOW_NATIVE_LIGHTBURN
	if (_serialPortRxOverrunFlag)
	{
		_serialPortRxOverrunFlag = FALSE;
		sprintf(_errorStr, "RX Overrun while receiving (sometime after line %d)", _gcodeLineNumber); //+1 because the argN for this line has not been processed yet.
		sendError(_errorStr);
		sprintf(_errorStr, "after: **%s**", makeAllCharsPrintable(currentCommandString));
		sendError(_errorStr);
	}
}



void CleanUpPointers()
{
	//come here after plugging in a valid character so you can update all the pointers in the system
	//in the case where the same variable is declared twice, the last ocurance is what will be persistant
	currentArgLength++; //increment the curren ascii length and check for 10 digits maximum
	if (currentArgLength > (MAX_CHARS_FOR_PARAMETER - 1))  // -1 because need room for the NULL
	{
		ProcessingError = 1;
		return;//just dont do anything yet
	}
	GCodeArgPtr++; //point to the next charaacterin the buffer
	*GCodeArgPtr = 0; //plug in a null just in case
}


void CopyToEndOfLine(buffer_t useBuffer)
{
	int count = 0;
	int rawChar; // MUST be of type "int" to handle char 0 to 255 AND the -1 "no data" return value

	for (count = 0; count < COMMENT_STRING_LENGTH - 1; count++)
	{
		rawChar = GCHAR(useBuffer);
		if (rawChar == -1)
		{
			// underRun .... should never happen here as a full command was needed to get this far
			reportRxUnderRun(useBuffer, "CopyToEndOfLine");
			*GCodeArgPtr = 0;
			return; // no reason to continue (really a fatal error)
		}
		else if (rawChar == CMD_END_CHAR)
		{
			// end of line found
			*GCodeArgPtr = NULL_CHAR;
#ifdef ALLOW_NATIVE_LIGHTBURN
			if (_lightburnModeEnabled)
				PostAcknowledge();
#endif //ALLOW_NATIVE_LIGHTBURN
			return;
		}
		else {
			*GCodeArgPtr = rawChar; // GCodeArgPtr is left pointing to GCodeArgComment[1] by FillArgumentBuffer
			GCodeArgPtr++; // prep for next char
		}
	}
	//if you get this far, there was a comment string over run and we need to report it

	sprintf(_errorStr, "Comment Length exceeded Buffer Length (%d) ", COMMENT_STRING_LENGTH);
	sendError(_errorStr);
	*GCodeArgPtr = 0; // null terminate final string
	purgeTillCr(useBuffer, -1); // -1 indicated to just discard data
}
////////////////////////////////////////////////////////////////////////////////

void FillArgumentBuffer(char *StringPTR, char  CommandChar)
{
	*StringPTR = CommandChar;
	StringPTR++; //now we have registerd the argument by updateing the argument letter, M,G,X,Y,Z... etc
	GCodeArgPtr = StringPTR;
	*GCodeArgPtr = 0; //plug in a null to indicate end of string
	currentArgLength = 0;
}


////////////////////////////////////////////////////////////////////////////////

void HssControl(int TicksPerSecond)
{
	// run through each of the switch controls
	// for all HSS controls operating in this timescale
	//    if the counter exceeds the duty cycle based compare value, then turn on the output
	//    otherwise turn it off

	int i;

	HssPwmStruct *hss;
	for (i = 0; i < NUM_HSS_PINS; i++)
	{
		hss = &HighSideSwitches[i];
		if (hss->TicksPerSecond == TicksPerSecond)
		{
			if (hss->Counter < hss->CompareValue)
				assertControlBit(&hss->Output); // turn on
			else
				deassertControlBit(&hss->Output); // turn off
			hss->Counter++;
			if (hss->Counter >= hss->TerminalCount)
			{
				if (hss->oneShot)
				{
					// kill output if dealing with a one shot
					hss->CompareValue = 0;
				}
				hss->Counter = 0; // wrap counter
			}
		}
	}
}

void DwellTimer(void)
{
	if (_g4DwellTimer > 0)
	{
		_g4DwellTimer--;
	}
}

////////////////////////////////////////////////////////////////////////////////
//uint16_t a = 0x3f00;
//uint16_t p = 0;
//static uint16_t * const DR3_Word = (uint16_t * const)&SPI3->DR;
void heartbeat(void)
{
	_gs._led.heartbeatState ^= 1;
	pinWrite(HEARTBEAT_PIN, _gs._led.heartbeatState);
	HeartBeat++;
	//	
	//	LCD_CS_CLR;
	//	LCD_RS_CLR;
	//	SPI3->CR1 |= SPI_CR1_SPE;
	//	SPI3->CR1 |= 1 << 9; //SPI_CR1_CSTART;
	//	*DR3_Word = a;
	//	LCD_CS_SET;
	//	p = *DR3_Word;
	//	if (a + 1 == 0xff) a = 0;
	//	else a++;
#ifdef GB_HEARTBEAT_PIN
	pinWrite(GB_HEARTBEAT_PIN, _gs._led.heartbeatState);
#endif

}



////////////////////////////////////////////////////////////////////////////////

void DDLightSelection(void)
{
	//determine value and reset hss control based on value
	if (DDLightFunction < 10)
	{
		// do nothing.... just an on/off light
	}
	else {
		int value = 0;
		if ((DDLightFunction >= 10) && (DDLightFunction <= 69))
		{
			int motor = (int)(DDLightFunction / 10) - 1;
			switch (DDLightFunction % 10)
			{
			case 0: value = (readSensorBitState(&Motors[motor].HomeSense) == SENSOR_TRIPPED); break;
			case 1: value = (readSensorBitState(&Motors[motor].Limit1Sense) == SENSOR_TRIPPED); break;
			case 2: value = (readSensorBitState(&Motors[motor].Limit2Sense) == SENSOR_TRIPPED); break;
			case 3:
			case 4: value = (readSensorBitState(&Motors[motor].FaultSense) == SENSOR_TRIPPED); break;
			default: value = 0;
			}
		}
		else if ((DDLightFunction >= 70) && (DDLightFunction <= 79))
		{
			value = readControlBitValue(&HighSideSwitches[DDLightFunction - 71].Output); // 71 is sent for the first selection, not 70 ***** XXX GB
		}
		else
		{
			switch (DDLightFunction)
			{
			case 80: value = readSensorBitState(&startButton);
			default: value = 0;
			}
		}

		if (value == 0)
			changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_OFF);
		else
			changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]], HSS_DUTY_CYCLE_ON);
	}
}

void EchoBackCommentString(void)
{
	sprintf(SendString, ">MC:M%3d: ;", (unsigned int) ARG_M);
	sendstring(SendString);
	sendstringCr(GCodeArgComment);
}
////////////////////////////////////////////////////////////////////////////////

void processNextDeferredCommand(void)
{
	if (DeferredCommandsInQue == 0)
	{
		barf("processNextDeferredCommand: lost a deferred command");
	}
	else if (deferredCmdQue[NextDeferredCommandIndex].cmdState != DEFERRED_CMD_VALID_AND_READY_TO_PROCESS)
	{
		barf("processNextDeferredCommand: links/readyToProcess mismatch");
	}
	else
	{
		GMCommandStructure *saveExecutionPtr = ExecutionPtr; // just in case, let's save and restore.
		processCommand(&deferredCmdQue[NextDeferredCommandIndex].cmd);
		ExecutionPtr = saveExecutionPtr; // restore
		uint32_t irq_disabled = interruptsOff(); // protect against processMotion touching deferred que from interfereing and vice versa
		deferredCmdQue[NextDeferredCommandInsertionIndex].cmdState = DEFERRED_CMD_INVALID;
		DeferredCommandsInQue--;
		NextDeferredCommandIndex = getNextDeferredCommandIndex(NextDeferredCommandIndex);
		if ((DeferredCommandsInQue == 0) || (deferredCmdQue[NextDeferredCommandIndex].cmdState != DEFERRED_CMD_VALID_AND_READY_TO_PROCESS))
		{
			_needToProcessDeferredCommands = FALSE;
		}
		interruptsOn(irq_disabled);
	}
}

void updateHostConnectionWatchDog(void)
{
	if (HostConnectionWatchDog > 0)
	{
		HostConnectionWatchDog--; //count down until we hit zero
		if (HostConnectionWatchDog == 0)
		{
			// shut down extruder motors so they don't spew forever
			StopAllExtruders();
			DisableAllExtruderMotors();
		}
	}
}


int getNextCommandIndex(int index)
{
	index++;
	if (index >= SIZE_OF_COMMAND_QUEUE)
		index = 2; //circular que so make sure you do not overrun
	return (index);
}
int getNextDeferredCommandIndex(int index)
{
	index++;
	if (index >= SIZE_OF_DEFERRED_COMMAND_QUEUE)
		index = 0; //circular que so make sure you do not overrun
	return (index);
}


////////////////////////////////////////////////////////////////////////////////

void checkStartButton(void)
{
	if (_highVoltageIsNotStableCoundownMs)
	{
		setSensorStateToUnknown(&startButton);
	}
	else if ((readDebouncedSensorState(&startButton) == SENSOR_TRIPPED) && startButton.StateChangeDetected)
	{   // leading edge start button pressed
		SendFakeMcodeExecutionNotice(1, INVALID_ARG_VALUE, 1, INVALID_ARG_VALUE); // send fake notice for an M1 (S1=active)
		sendInfo("Start button pressed");

		if (_sendingGBStringsMask & GB_STRING_DUMP_ON_START)  // use M797 S<mask> to enable
		{
			sendCrashDataFromRAMtoFlash(); // save a copy of key variables to flash
		}
	}
}
