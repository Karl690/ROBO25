#include "taskmanager.h"
#include "MotorDriver.h"
#include "pinout.h"
#include "Serial.h"
#include "mailbox.h"
#include "Hydra_can.h"
#include "adc.h"
#include "pnp.h"
unsigned int counter_1Hz = 0;
unsigned int counter_10Hz = 0;
unsigned int counter_100Hz = 0;
unsigned int counter_1000Hz = 0;

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void checkInWithHeads(void)
{
	if ((counter_1Hz % MOTION_TO_HH_CHECKIN_RATE) == 0)
	{
		sendSetCommPingBit(0);
	}
}
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
		if ((_gs._flasher.varPtr != NULL) && (*_gs._flasher.varPtr))
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

void spare(void)
{
	// placeholder call for empty slice
}

////////////////////////////////////////////////////////////////////////////////

void ohNoMrBill(void)
{
	// XXXX should send an ERROR .... should never get here
}

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
////////////////////////////////////////////////////////////////////////////////
