#ifdef ADD_ON_SPI_DISPLAY
////////////////////////////////////////////////////////////////////////////////
//
// File:    gui.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: various gui releated functions
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "gui.h"
#include "display.h"
#include "HardwareInit.h"
#include "Serial.h"
#include "MotorDriver.h"
#include "mailbox.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// COMMON //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char _dispStr[64];	//persistant "local" variable -- do not use anywhere else (64 is safe with small displays so space can be used for justifying, etc

////////////////////////////////////////////////////////////////////////////////

void display_Init(void)
{
#ifdef BB_ADDR_PIN_LCD_BKL    //if this config has a backlight control pin, this is the bit-bang (mem mapped IO) address
	uint32_t backlightBBpinAddr = BB_ADDR_PIN_LCD_BKL;
#else //!BB_ADDR_PIN_LCD_BKL
	uint32_t backlightBBpinAddr = 0;
#endif //!BB_ADDR_PIN_LCD_BKL

	_gs._displayIsAlive = GUI_Init(LCD_TFT_DEVEBOX_SPI_TFT_280, LCD_ROTATION_180, LCD_FONT_11x16, GUI_DISPLAY_SPI, backlightBBpinAddr, TRUE);    // sets up IO, SPI, and LCD regs

	if (_gs._displayIsAlive)
	{
		GUI_DrawPage(DISPLAY_PAGE_SPLASH_SCREEN);   // splash screen, then black to allow display of boot process.
		GUI_SetNextPage(DISPLAY_PAGE_DEFAULT);
		GUI_SetRefreshInternalInMs(200);
	}
	//updateBootSequence(BOOT_STAGE_DISPLAY_INIT);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#if 0 //SAVE FOR FUTURE
#ifdef ADD_ON_CLOSED_LOOP_STEPPER

char *getMotorStateStr(char *s)
{
	sprintf(s, "%s-%c", _scs.currentMotorControlMethod == CONTROL_OPEN_LOOP ? "OL" : "CL", MOTOR_IS_DISABLED ? 'D' : 'E');
	return(s);
}

#endif //ADD_ON_CLOSED_LOOP_STEPPER

////////////////////////////////////////////////////////////////////////////////

char *getSwitchModeStr(char *s, switchMode_t mode)
{
	switch(mode)
	{
	case SWITCH_CONTROL_OFF: 			strcpy(s, "OFF"); break;
	case SWITCH_CONTROL_ON: 			strcpy(s, "ON"); break;
	case SWITCH_CONTROL_DUTY_CYCLE: 	strcpy(s, "DC"); break;
	case SWITCH_CONTROL_PWM: 			strcpy(s, "PWM"); break;
	case SWITCH_CONTROL_BY_TEMP: 		strcpy(s, "TMP"); break;
	default:							strcpy(s, "HUH"); break;
	}
	return(s);
}
#endif //SAVE FOR FUTURE

////////////////////////////////////////////////////////////////////////////////

char getSensorValueChar(sensorStruct *sensorPtr)
{
	char returnChr;
	if (sensorPtr->Enabled)
		returnChr = (readSensorBitValue(sensorPtr) == 0) ? '0' : '1';
	else
		returnChr = ' ';
	return(returnChr);
}

////////////////////////////////////////////////////////////////////////////////

char getSensorStateChar(sensorStruct *sensorPtr, char trippedChr)
{
	char returnChr;
	if (sensorPtr->Enabled)
	{
		sensorState_t state = readSensorBitState(sensorPtr);
		returnChr = (state == SENSOR_TRIPPED) ? trippedChr : (state == SENSOR_OPEN) ? '.' : 'X';
	}
	else
		returnChr = ' ';
	return(returnChr);
}

////////////////////////////////////////////////////////////////////////////////

//char *getSensorStateStr(char *s, sensorStruct *sensorPtr, char *trippedStr)
//{
//	*s = NULL_CHAR;		// empty string in case nothing is found
//	if (sensorPtr->Enabled)
//	{
//		sensorState_t state = readSensorBitState(sensorPtr);
//		if (state == SENSOR_OPEN)
//			strcpy(s, "Open");
//		else if (state == SENSOR_TRIPPED)
//			strcpy(s, trippedStr);
//		else
//			strcpy(s, "Unknown");
//	}
//	return(s);
//}

////////////////////////////////////////////////////////////////////////////////

char *getNthDevicePositionString(char *s, byte strSubIndex)
{
	inboxStruct *inboxPtr;
	*s = NULL_CHAR;		// empty string in case nothing is found
	 if ((inboxPtr = findNthRegisteredDevice(strSubIndex)))
	 {
		 sprintf(s, "T%d", inboxPtr->device);
	 }
	 return(s);
}

////////////////////////////////////////////////////////////////////////////////

char *getNthDeviceTypeString(char *s, byte strSubIndex)
{
	inboxStruct *inboxPtr;
	*s = NULL_CHAR;		// empty string in case nothing is found
	 if ((inboxPtr = findNthRegisteredDevice(strSubIndex)))
	 {
		 strcpy(_dispStr, deviceTypeToString(inboxPtr->deviceType));
	 }
	 return(s);
}

////////////////////////////////////////////////////////////////////////////////

char *getAnyAxisNeedingToBeHomedStr(char *s)
{
	int index=0;

	for (MotorStructure *M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors and set the flag showing axis has been homed
		if (M->MotorInstalled && M->HomeSense.Enabled && !M->HasBeenHomed)
		{
//			if (index != 0)
//			{	//add a spacer
//				s[index++] = '_';
//			}
			s[index++] = M->AxisLabel;
		}
	}
	s[index] = NULL_CHAR;		// terminate the string
	return(s);
}

////////////////////////////////////////////////////////////////////////////////

char *display_getStringForIndexedValue(byte strIndex, byte strSubIndex, byte postDecimalPtDigits, byte *colorIndex)
{	// index lookup of a select set of desired values to look at on the display.   may of these are not stored or calculated in normal
	// operation, so this provideds one stop shopping to the get interesting values, calculated on the spot and formated as follows:
	//     "%s" - natural strings:   will be returned as generated
	//	   "%s" - natural chars:	  will be returned as a 1 char long string
	//	   "%d" - any signed 8 to 16-bit integers
	//	   "%u" - any unsigned 8 to 16-bit integers
	//	   "%ld" - any signed 32-bit integers
	//	   "%lu" - any unsigned 32-bit integers
	//	   "%lld" - any signed 64-bit integers
	//	   "%llu" - any unsigned 64-bit integers
	//     "%{(postDecimalPtDigits+1).postDecimalPtDigits}f - and float  --- in this case (and only in the case of floats), the calling
	//		function can alter the format string.
	//
	//     NO CHANGE TO THE FINAL OVERAL WIDTH OF THE STRING IS MADE BY THIS ROUTINE

	_dispStr[0] = NULL_CHAR;		//start with empty string
	char *fmtStr = GUI_GetFloatFormatStr(postDecimalPtDigits);	// only used for	floats!
	int newCI = -1;	//if still -1 at end of routine, then no change to color
	MotorStructure *M = &Motors[iFitWithinRange(strSubIndex, 0, MAX_NUMBER_OF_MOTORS)];

	switch(strIndex)
	{
	case IDX_UPTIME: 			sprintf(_dispStr, "%lu", _gs._seconds); break;
	case IDX_SW_REV_STR:
		if (SOFTWARE_TWEAK_REVISION == 'z')
			sprintf(_dispStr, "Z%d.%03d%c", SOFTWARE_MAJOR_REVISION, SOFTWARE_MINOR_REVISION, SOFTWARE_DEBUG_REVISION);
		else
			sprintf(_dispStr, "v%d.%03d%c", SOFTWARE_MAJOR_REVISION, SOFTWARE_MINOR_REVISION, SOFTWARE_TWEAK_REVISION);
		break;

	case IDX_POS_MM:		  	sprintf(_dispStr, fmtStr, M->POSITION * M->UnitsPerPulse); break;
	case IDX_POS_STEPS:  		sprintf(_dispStr, "%ld", M->POSITION); break;

	case IDX_FLOW_RATE:			sprintf(_dispStr, fmtStr, (_ActualVectorRateInMmPerSec == 0.0f) ? 0.0f : (_LastExtrusionRate == INVALID_ARG_VALUE) ? 0.0f : _LastExtrusionRate); break;
	case IDX_LINE_NUM:			sprintf(_dispStr, "%ld", getCurrentExecutingLineNumber()); break;

	case IDX_CURR_FEEDRATE:		sprintf(_dispStr, fmtStr, _CurrentRequestedFeedrateInMmPerSec); break;
	case IDX_ACT_VECTRATE:		sprintf(_dispStr, fmtStr, _ActualVectorRateInMmPerSec); break;

	case IDX_NTH_DEV_POS: 		getNthDevicePositionString(_dispStr, strSubIndex); break;
	case IDX_NTH_DEV_TYPE_STR:	getNthDeviceTypeString(_dispStr, strSubIndex); break;

	case IDX_VAL_CHAR_STEP:		sprintf(_dispStr, "%c", (M->MotorInstalled?((readControlBitValue(&M->Step)==0)?'0':'1'):' ')); break;
	case IDX_VAL_CHAR_DIR:		sprintf(_dispStr, "%c", (M->MotorInstalled?((readDirectionBitValue(&M->Direction)==0)?'0':'1'):' ')); break;
	case IDX_VAL_CHAR_HOME:		sprintf(_dispStr, "%c", getSensorValueChar(&M->HomeSense)); break;
	case IDX_VAL_CHAR_LIM1:		sprintf(_dispStr, "%c", getSensorValueChar(&M->Limit1Sense)); break;
	case IDX_VAL_CHAR_LIM2:		sprintf(_dispStr, "%c", getSensorValueChar(&M->Limit2Sense)); break;
	case IDX_VAL_CHAR_FAULT:	sprintf(_dispStr, "%c", getSensorValueChar(&M->FaultSense)); break;
	case IDX_VAL_CHAR_START:	sprintf(_dispStr, "%c", getSensorValueChar(&startButton)); break;
	case IDX_VAL_CHAR_ABENC:	sprintf(_dispStr, "%c", getSensorValueChar(&ABEncoderSelectButton)); break;
	case IDX_VAL_CHAR_EMO:		sprintf(_dispStr, "%c", getSensorValueChar(&EMO)); break;

	case IDX_STATE_CHAR_STEP:	sprintf(_dispStr, "%c", (M->MotorInstalled?((readControlBitState(&M->Step)==DEASSERTED)?'.':'S'):' ')); break;
	case IDX_STATE_CHAR_DIR:	sprintf(_dispStr, "%c", (M->MotorInstalled?((readDirectionBitState(&M->Direction)==DIRECTION_REVERSE)?'R':'F'):' ')); break;
	case IDX_STATE_CHAR_HOME:	sprintf(_dispStr, "%c", getSensorStateChar(&M->HomeSense, 'H')); break;
	case IDX_STATE_CHAR_LIM1:	sprintf(_dispStr, "%c", getSensorStateChar(&M->Limit1Sense, 'L')); break;
	case IDX_STATE_CHAR_LIM2:	sprintf(_dispStr, "%c", getSensorStateChar(&M->Limit2Sense, 'L')); break;
	case IDX_STATE_CHAR_FAULT:	sprintf(_dispStr, "%c", getSensorStateChar(&M->FaultSense, 'F')); break;
	case IDX_STATE_CHAR_START:	sprintf(_dispStr, "%c", getSensorStateChar(&startButton, 'R')); break;
	case IDX_STATE_CHAR_ABENC:	sprintf(_dispStr, "%c", getSensorStateChar(&ABEncoderSelectButton, 'S')); break;
	case IDX_STATE_CHAR_EMO:	sprintf(_dispStr, "%c", getSensorStateChar(&EMO, 'E')); break;

	case IDX_CURR_SLICE:		sprintf(_dispStr, "%04d", _gs._sliceNum); break;
	case IDX_PEND_ACK:			sprintf(_dispStr, "%d", pendingAcknowledge); break;
	case IDX_CMD_IN_CMDQUE:		sprintf(_dispStr, "%d", CommandsInQue); break;
	case IDX_CMD_IN_MOTIONQ:	sprintf(_dispStr, "%d", motionQ.validEntries); break;
	case IDX_BLOCK_ALL:			if (_blockAllMotion) { strcpy(_dispStr, "BLOCK"); newCI = CI_RED; } break;
	case IDX_BLOCK_ABS:			if (_blockAbsoluteMotion) { strcpy(_dispStr, "BLOCK"); newCI = CI_RED; } break;
	case IDX_UNHOMED:			getAnyAxisNeedingToBeHomedStr(_dispStr); if (_dispStr[0] != NULL_CHAR) newCI = CI_RED; break;
	case IDX_GCCODE_PAUSED:		if (_gcodePaused) { strcpy(_dispStr, "PAUSE"); newCI = CI_RED; } break;
	case IDX_DWELL_TIME:		if (_g4DwellTimer) { sprintf(_dispStr, "%ld", _g4DwellTimer); newCI = CI_RED; } break;
	case IDX_IN_MOTION:			sprintf(_dispStr, "%s", motionQ.oldest->flags.inMotion ? "YES":""); if (motionQ.oldest->flags.inMotion) newCI = CI_GRN;break;

	case IDX_STATE_STR_EMO:		if (sensorEnabledAndTripped(&EMO)) { strcpy(_dispStr, "PRESSED");  newCI = CI_RED; } break;

	case IDX_NONE:
	default:					break;
	}
	if ((colorIndex != NULL) && (newCI != -1))
		*colorIndex = (byte)newCI;
	return(_dispStr);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void initShowControl(void)
{
	int8_t line=0;
	LCD_addr_t labX = GUI_LeftEdge();
	LCD_addr_t valX = GUI_RightEdge();

	GUI_SetupDefaultPageHeader(&line, TRUE);

	GUI_AddLabel(line++, GUI_HorizMiddle(), JC, CI_RED, CI_LBG, "Controls");

	GUI_AddDefaultEntry(line++,  labX, "Uptime",	valX,  9, 0, IDX_UPTIME,			0);
	GUI_AddDefaultEntry(line++,  labX, "Slice",		valX,  4, 0, IDX_CURR_SLICE,		0);
	GUI_AddDefaultEntry(line++,  labX, "PendAck",	valX,  3, 0, IDX_PEND_ACK,			0);
	GUI_AddDefaultEntry(line++,  labX, "CmdQ",		valX,  3, 0, IDX_CMD_IN_CMDQUE,		0);
	GUI_AddDefaultEntry(line++,  labX, "MotionQ",	valX,  3, 0, IDX_CMD_IN_MOTIONQ,	0);
	GUI_AddDefaultEntry(line++,  labX, "RelMot",	valX,  5, 0, IDX_BLOCK_ALL,			0);	//ALL == RELATIVE
	GUI_AddDefaultEntry(line++,  labX, "AbsMot",	valX,  5, 0, IDX_BLOCK_ABS,			0);
	GUI_AddDefaultEntry(line++,  labX, "Unhomed",	valX,  6, 0, IDX_UNHOMED,			0);
	GUI_AddDefaultEntry(line++,  labX, "Active",	valX,  5, 0, IDX_GCCODE_PAUSED,		0);
	GUI_AddDefaultEntry(line++,  labX, "Dwell",		valX,  7, 0, IDX_DWELL_TIME,		0);
	GUI_AddDefaultEntry(line++,  labX, "Moving",	valX,  3, 0, IDX_IN_MOTION,			0);
#ifdef USE_HYDRA_IO
	GUI_AddDefaultEntry(line++,  labX, "EMO:",      valX,  7, 0, IDX_STATE_STR_EMO,		0);	//getSensorValueStr&EMO);
#endif //USE_HYDRA_IO
}

////////////////////////////////////////////////////////////////////////////////

void initShowHeadList(void)
{	// page to show physical or logica state of common pins/sensors in the system in a simple table
	int8_t line=0;
	LCD_addr_t labX = GUI_LeftEdge();
	LCD_addr_t valX = GUI_RightEdge();

	GUI_SetupDefaultPageHeader(&line, TRUE);

	GUI_AddLabel(line++, GUI_HorizMiddle(), JC, CI_RED, CI_LBG, "Device List");

	int8_t startLine = line;
	for (int i=0; i<10; i++)
	{
		GUI_AddValue(line, labX,  JL, CI_LFG, CI_LBG,  4, 0, IDX_NTH_DEV_POS,		line-startLine+1);	// first Nth value is 1
		GUI_AddValue(line, valX,  JR, CI_VFG, CI_VBG, 12, 0, IDX_NTH_DEV_TYPE_STR,	line-startLine+1);
		line++;
	}
}

////////////////////////////////////////////////////////////////////////////////

void initShowPinsPage(int page)
{	// page to show physical or logica state of common pins/sensors in the system in a simple table
	int8_t line=0;
	LCD_addr_t labX = GUI_LeftEdge();
	LCD_addr_t valX = GUI_RightEdge();

	GUI_SetupDefaultPageHeader(&line, TRUE);

	boolean phys = (page == DISPLAY_PAGE_PHYSICAL_PINS);
	int idxOfs = phys ? 0 : (IDX_STATE_CHAR_STEP - IDX_VAL_CHAR_STEP);

	GUI_AddLabel(line++, GUI_HorizMiddle(), JC, CI_RED, CI_LBG, phys ? "Physical Values" : "Logical Values");
	GUI_AddLabel(line++, valX, JR, CI_LFG, CI_LBG, "S D H 1 2 F");

	LCD_addr_t columnWidth = 2 * LCD_GetFontWidth();	// column width for table
	valX -= 5 * columnWidth;
	int8_t startLine = line;
	GUI_AddDefaultEntry(line++,  labX, "X:", valX,  1, 0, IDX_VAL_CHAR_STEP+idxOfs,	M_X);   //step
	GUI_AddDefaultEntry(line++,  labX, "Y:", valX,  1, 0, IDX_VAL_CHAR_STEP+idxOfs,	M_Y);
	GUI_AddDefaultEntry(line++,  labX, "Z:", valX,  1, 0, IDX_VAL_CHAR_STEP+idxOfs,	M_Z);
	GUI_AddDefaultEntry(line++,  labX, "A:", valX,  1, 0, IDX_VAL_CHAR_STEP+idxOfs,	M_A);
	GUI_AddDefaultEntry(line++,  labX, "B:", valX,  1, 0, IDX_VAL_CHAR_STEP+idxOfs,	M_B);
	valX += columnWidth;  line = startLine;
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_DIR+idxOfs,	M_X);   //dir
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_DIR+idxOfs,	M_Y);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_DIR+idxOfs,	M_Z);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_DIR+idxOfs,	M_A);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_DIR+idxOfs,	M_B);
	valX += columnWidth;  line = startLine;
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_HOME+idxOfs,	M_X);   //home
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_HOME+idxOfs,	M_Y);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_HOME+idxOfs,	M_Z);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_HOME+idxOfs,	M_A);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_HOME+idxOfs,	M_B);
	valX += columnWidth;  line = startLine;
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM1+idxOfs,	M_X);   //limit1
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM1+idxOfs,	M_Y);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM1+idxOfs,	M_Z);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM1+idxOfs,	M_A);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM1+idxOfs,	M_B);
	valX += columnWidth;  line = startLine;
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM2+idxOfs,	M_X);   //limit2
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM2+idxOfs,	M_Y);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM2+idxOfs,	M_Z);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM2+idxOfs,	M_A);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_LIM2+idxOfs,	M_B);
	valX += columnWidth;  line = startLine;
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_FAULT+idxOfs,	M_X);   //fault
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_FAULT+idxOfs,	M_Y);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_FAULT+idxOfs,	M_Z);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_FAULT+idxOfs,	M_A);
	GUI_AddDefaultValue(line++, valX,  1, 0, IDX_VAL_CHAR_FAULT+idxOfs,	M_B);
	line++;		// this line intentionally left blank
	GUI_AddDefaultEntry(line++,  labX, "RUN:", valX,  1, 0, IDX_VAL_CHAR_START+idxOfs,	0);	//getSensorValueChar(&startButton);
	GUI_AddDefaultEntry(line++,  labX, "ENC:", valX,  1, 0, IDX_VAL_CHAR_ABENC+idxOfs,	0);	//getSensorValueChar(&ABEncoderSelectButton);
	GUI_AddDefaultEntry(line++,  labX, "EMO:", valX,  1, 0, IDX_VAL_CHAR_EMO+idxOfs,	0);	//getSensorValueChar(&EMO);
}

////////////////////////////////////////////////////////////////////////////////

void initDefaultPage(void)
{	// default page, shows current motion position.
	int8_t line=0;
	LCD_addr_t labX = GUI_LeftEdge();
	LCD_addr_t valX = GUI_RightEdge();

	GUI_SetupDefaultPageHeader(&line, TRUE);

	GUI_AddDefaultEntry(line++,  labX, "X:",        valX,  9, 4, IDX_POS_MM,			M_X);		// 1 == pos in mm
	GUI_AddDefaultEntry(line++,  labX, "Y:",        valX,  9, 4, IDX_POS_MM,			M_Y);
	GUI_AddDefaultEntry(line++,  labX, "Z:",        valX,  9, 4, IDX_POS_MM,			M_Z);
	GUI_AddDefaultEntry(line++,  labX, "A:",        valX,  9, 4, IDX_POS_MM,			M_A);
	GUI_AddDefaultEntry(line++,  labX, "B:",        valX,  9, 4, IDX_POS_MM,			M_B);

	GUI_AddDefaultEntry(line++,  labX, "Feed:",     valX,  9, 1, IDX_CURR_FEEDRATE,		0);		// (uint32_t)&_CurrentRequestedFeedrateInMmPerSec);
	GUI_AddDefaultEntry(line++,  labX, "Velo:",     valX,  9, 1, IDX_ACT_VECTRATE, 		0);		// (uint32_t)&_ActualVectorRateInMmPerSec);
	GUI_AddDefaultEntry(line++,  labX, "Flow:",     valX,  9, 1, IDX_FLOW_RATE,			0);   	// flowRate
//	GUI_AddDefaultEntry(line++,  labX, "LineNum:",  valX,  9, 0, IDX_LINE_NUM,			0);   	// lineNumber

	GUI_AddDefaultEntry(line++,  labX, "RelMot:",	valX,  5, 0, IDX_BLOCK_ALL,			0);	//ALL == Relative
	GUI_AddDefaultEntry(line++,  labX, "AbsMot:",	valX,  5, 0, IDX_BLOCK_ABS,			0);
	GUI_AddDefaultEntry(line++,  labX, "Unhomed:",	valX,  6, 0, IDX_UNHOMED,			0);
	GUI_AddDefaultEntry(line++,  labX, "Active:",	valX,  5, 0, IDX_GCCODE_PAUSED,		0);
#ifdef USE_HYDRA_IO
	GUI_AddDefaultEntry(line++,  labX, "EMO:",      valX,  7, 0, IDX_STATE_STR_EMO,		0);	//getSensorValueStr&EMO);
#endif //USE_HYDRA_IO
}

////////////////////////////////////////////////////////////////////////////////

void display_initializePage(int page)
{
	switch(page)
	{
	case DISPLAY_PAGE_DEFAULT:  		LCD_SetFont(LCD_FONT_16x22);	initDefaultPage();		break;
	case DISPLAY_PAGE_CONTROLS:			LCD_SetFont(LCD_FONT_16x22);	initShowControl();		break;
	case DISPLAY_PAGE_HEAD_LIST:		LCD_SetFont(LCD_FONT_16x22);	initShowHeadList();		break;
	case DISPLAY_PAGE_PHYSICAL_PINS:
	case DISPLAY_PAGE_LOGICAL_PINS: 	LCD_SetFont(LCD_FONT_13x20); 	initShowPinsPage(page);	break;

	default:  break;
	}
}

////////////////////////////////////////////////////////////////////////////////

#include "hyrel3d_logo_128x128.h"	// must be included at the top level (not inside a method)

void display_showSplashScreen(void)
{
	GUI_SetFakeLedEnable(FALSE);
	int pauseSec=1;
	if (1)
	{
		LCD_FillDisplay(LCD_COLOR_WHITE);
		LCD_DrawRGBImage((LCD_GetDisplayWidth()-128)/2, (LCD_GetDisplayHeight()-128)/2, 128, 128, (LCD_color_t *)hyrel3d_logo_128x128);
		delaySec(pauseSec);
	}
	if (1)
	{
		LCD_FillDisplay(LCD_COLOR_BLACK);
		LCD_DrawRGBImageChromaKey((LCD_GetDisplayWidth()-128)/2, (LCD_GetDisplayHeight()-128)/2, 128, 128, (LCD_color_t *)hyrel3d_logo_128x128, LCD_COLOR_WHITE);
		delaySec(pauseSec);
	}
	if (0)
	{
		int i;
		LCD_FillDisplay(LCD_COLOR_BLACK);
		for(i=0; i<imin(LCD_GetDisplayWidth(), LCD_GetDisplayHeight())/2-1; i+=5)
		{
			LCD_DrawRect(i, i, LCD_GetDisplayWidth()-2*i, LCD_GetDisplayHeight()-2*i, 1, ((i/5)&1) ? LCD_COLOR_WHITE : LCD_COLOR_BLUE);
		}
	}
	if (1)
	{
		int i;
		LCD_FillDisplay(LCD_COLOR_WHITE);
		int maxR = fpu_sqrtf(((LCD_GetDisplayWidth()/2) * (LCD_GetDisplayWidth()/2)) + ((LCD_GetDisplayHeight()/2) * (LCD_GetDisplayHeight()/2)));
		for(i=3; i<maxR; i+=5)
		{
			LCD_DrawCircle(LCD_GetDisplayWidth()/2, LCD_GetDisplayHeight()/2, i, 1, ((i/5)&1) ? LCD_COLOR_RED : LCD_COLOR_BLUE);
		}
	}

	if (0)
	{
		int i;
		LCD_FillDisplay(LCD_COLOR_BLACK);
		for(i=0; i<LCD_GetDisplayWidth()-1; i+=5)
		{
			LCD_DrawLine(i, 0, LCD_GetDisplayWidth()-i-1, LCD_GetDisplayHeight()-1, 1, LCD_COLOR_RED);
		}
		for(i=0; i<LCD_GetDisplayHeight()-1; i+=5)
		{
			LCD_DrawLine(0, LCD_GetDisplayHeight()-i-1, LCD_GetDisplayWidth()-1, i, 1, LCD_COLOR_RED);
		}
	}
	if (0)
	{
		LCD_DrawFilledRect(0, 0, LCD_GetDisplayWidth(), LCD_GetDisplayHeight(), 5, LCD_COLOR_BLUE, LCD_COLOR_PINK);
		LCD_DrawFilledCircle(0, 0, imin(LCD_GetDisplayWidth(), LCD_GetDisplayHeight())/2-20, 5, LCD_COLOR_RED, LCD_COLOR_GREEN_50PCT);
		LCD_DrawFilledCircle(LCD_GetDisplayWidth()/2, LCD_GetDisplayHeight()/2, imin(LCD_GetDisplayWidth(), LCD_GetDisplayHeight())/2-20, 5, LCD_COLOR_RED, LCD_COLOR_GREEN);
		LCD_SetFontForegroundColor(LCD_COLOR_BLACK);
		LCD_SetFontBackgroundColor(LCD_COLOR_TRANSPARENT);
		byte saveFont = LCD_GetFont();
		LCD_SetFont(LCD_FONT_5x8);      LCD_DrawString(5, 5, "5x8_FONT");
		LCD_SetFont(LCD_FONT_7x12);     LCD_DrawString(5, 23, "7x12_FONT");
		LCD_SetFont(LCD_FONT_11x16);    LCD_DrawString(5, 45, "11x16_FONT");
		LCD_SetFont(LCD_FONT_13x20);    LCD_DrawString(5, 71, "13x20_FONT");
		LCD_SetFont(LCD_FONT_16x22);    LCD_DrawString(5, 101, "16x22_FONT");
		LCD_SetFont(saveFont);
		delaySec(pauseSec);
	}
}

////////////////////////////////////////////////////////////////////////////////
#endif // ADD_ON_SPI_DISPLAY
