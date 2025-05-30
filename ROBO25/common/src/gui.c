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

#ifdef COMPILE_FOR_SYSTEM
extern void sendError(char *s);
#include "mailbox.h"
#endif //COMPILE_FOR_SYSTEM

////////////////////////////////////////////////////////////////////////////////
// local globals
////////////////////////////////////////////////////////////////////////////////

GUI_driverStruct _GUI;

////////////////////////////////////////////////////////////////////////////////
// forward declarations
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//////////////////////////// internal use methods //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void setGuiDisplayOffsets(byte panelIndex)
{	// set the GUI based offset (not internal to the panel, those are dealt with elsewhere
	// these are to get the screen, headers, etc to look ok on each screen type
	switch(panelIndex)
	{
	case LCD_TFT_DEVEBOX_SPI_TFT_280:
		_GUI.vOfs = 3;
		_GUI.hOfs = 1;
		break;
	case LCD_TFT_MCUDEV_TFT_180:
		_GUI.vOfs = 2;
		_GUI.hOfs = 2;
		break;
	case LCD_TFT_MCUDEV_TFT_144:
		_GUI.vOfs = 1;
		_GUI.hOfs = 1;
		break;
#if 0	// NOT READY FOR PRIME TIME
	case LCD_TFT_MCUDEV_TFT_096:
		_LCD.driver = &_ssd1306;
		_GUI.vOfs = 0;
		_GUI.hOfs = 0;
#endif
	default:
		_GUI.vOfs = 0;
		_GUI.hOfs = 0;
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void prepareDisplayList(void)
{
	memZero((byte *)&_GUI.displayList[0], sizeof(displayListEntryStruct) * NUM_DISPLAY_LIST_ENTRIES);

	_GUI.current_dPtr = &_GUI.displayList[0];

	for (displayListEntryStruct *dlp=_GUI.current_dPtr; dlp<&_GUI.displayList[NUM_DISPLAY_LIST_ENTRIES]; dlp++)
	{
		dlp->line = -1;   // -1 will flag that the entry in not in use (skipped)
	}
}

////////////////////////////////////////////////////////////////////////////////

void drawValue(int x, int y, displayListEntryStruct *dlp)
{
	if (!((dlp->valueIndex == 0) || (dlp->valueWidth == 0)))
	{
		byte colorIndex = dlp->valueFgColorIndex;	// need temp location in case display_getStringForIndexedValue needs to force new color
		char *s = display_getStringForIndexedValue(dlp->valueIndex, dlp->valueSubIndex, dlp->valueSigDigits, &colorIndex);
		LCD_FastSetFontColorsFromIndex(colorIndex, dlp->valueBgColorIndex);
		LCD_PadAndJustifyStringToSize(s, dlp->valueJust, dlp->valueWidth);
		LCD_DrawStringJust(x, y, s, dlp->valueJust); 	//x, y, str, just
	}
}

////////////////////////////////////////////////////////////////////////////////

void drawLabel(int x, int y, displayListEntryStruct *dlp)
{
	if (dlp->label[0] != '\0')
	{
		LCD_FastSetFontColorsFromIndex(dlp->labelFgColorIndex, dlp->labelBgColorIndex);
		LCD_DrawStringJust(x, y, dlp->label, dlp->labelJust);
	}
}

////////////////////////////////////////////////////////////////////////////////

void drawDynamicPageEntries(void)
{
	if (_GUI.displayList[0].line == -1) return;	// not a normal page, let user control all

	uint16_t fHeight = LCD_GetFontHeight();
	uint16_t yOfs = _GUI.vOfs + fHeight + 1;            // offset to horiz line (below header text
	yOfs += 2;                                                  			// move over double wide horizontal like
	yOfs += _GUI.vOfs;                                  		    // starting point for line 1 (leave gap from horiz line
	yOfs -= fHeight;                                    	    // adjust because first real line starts at line==1

	displayListEntryStruct *dlp;
	for (dlp=&_GUI.displayList[0]; dlp<&_GUI.displayList[NUM_DISPLAY_LIST_ENTRIES]; dlp++)
	{
		if (dlp->line == -1)
			break;	//entries are in order in the array, so can stop when first unused one is detected
		else if (dlp->line == 0)
			drawValue(dlp->valueX, _GUI.vOfs, dlp); //header, special offset
		else
			drawValue(dlp->valueX, yOfs + (dlp->line*fHeight), dlp);
		GUI_DrawFakeLeds(FALSE);//keep them up to date (will quickly exit if no change)
	}
}

////////////////////////////////////////////////////////////////////////////////

void drawStaticPageEntries(void)
{
	if (_GUI.displayList[0].line == -1) return;	// not a normal page, let user control all

	uint16_t fHeight = LCD_GetFontHeight();
	uint16_t yOfs = _GUI.vOfs + fHeight + 1;                // offset to horiz line (below header text
	LCD_DrawHLine(0, yOfs, LCD_GetDisplayWidth(), 2, LCD_COLOR_BLUE);	// double wide line
	yOfs += 2;                                                  // move over double wide horizontal like
#ifdef SONIC407
	if (DEVICE_IS_A_SONICATOR)
	{
		LCD_DrawVLine((LCD_GetDisplayWidth()/2)-1, yOfs, LCD_GetDisplayHeight() - yOfs, 2, LCD_COLOR_BLUE);   // double wide line
	}
#endif //SONIC407
	yOfs += _GUI.vOfs;                                          // starting point for line 1 (leave gap from horiz line
	yOfs -= fHeight;                                // adjust because first real line starts at line==1

	displayListEntryStruct *dlp;
	for (dlp=&_GUI.displayList[0]; dlp<&_GUI.displayList[NUM_DISPLAY_LIST_ENTRIES]; dlp++)
	{
		if (dlp->line == -1)
			break;	//entries are in order in the array, so can stop when first unused one is detected
		else if (dlp->line == 0)
			drawLabel(dlp->labelX, _GUI.vOfs, dlp); //header, special offset
		else
			drawLabel(dlp->labelX, yOfs + (dlp->line*fHeight), dlp);
		GUI_DrawFakeLeds(FALSE);//keep them up to date (will quickly exit if no change)
	}
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////// Available externally /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const char _GUI_formatStrings[NUM_FORMATS][6] = {	// really not meant for general use
		"%s",		// FMT_OFF,
		"%s",		// FMT_STR,
		"%c",		// FMT_CHR,
		"%d",		// FMT_I8,
		"%d",		// FMT_I16,
		"%ld",		// FMT_I32,
		"%lld",		// FMT_I64,
		"%u",		// FMT_U8,
		"%u",		// FMT_U16,
		"%ld",		// FMT_U32,
		"%llu",		// FMT_U64,
		"%1.0f",	// FMT_FL0,
		"%2.1f",	// FMT_FL1,
		"%3.2f",	// FMT_FL2,
		"%4.3f",	// FMT_FL3,
		"%5.4f",	// FMT_FL4,
		"%6.5f",	// FMT_FL5,
		"%7.6f",	// FMT_FL6,
};

char *GUI_GetFormatStr(byte format)
{
	return((char *)&_GUI_formatStrings[umin(format, FMT_FL6)][0]);
}

char *GUI_GetFloatFormatStr(byte postDecimalPtDigits)
{
	return((char *)&_GUI_formatStrings[FMT_FL0+umin(postDecimalPtDigits, 6)][0]);
}
////////////////////////////////////////////////////////////////////////////////

int GUI_LeftEdge(void)
{
	return(_GUI.hOfs);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_RightEdge(void)
{
	return((LCD_GetDisplayWidth() - 1) - _GUI.hOfs);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_HorizMiddle(void)
{
	return(LCD_GetDisplayWidth()/2);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_TopEdge(void)
{
	return(_GUI.vOfs);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_BottomEdge(void)
{
	return((LCD_GetDisplayHeight() - 1) - _GUI.vOfs);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_VertMiddle(void)
{
	return(LCD_GetDisplayHeight()/2);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_VertOffset(void)
{
	return(_GUI.vOfs);
}

////////////////////////////////////////////////////////////////////////////////

int GUI_HorizOffset(void)
{
	return(_GUI.hOfs);
}
////////////////////////////////////////////////////////////////////////////////

void GUI_SetRefreshInternalInMs(int ms)
{
	{
		_GUI.updateIntervalMs = ms;		// set terminal count
		_GUI.updateIntervalCountMs = 0;  // reset counter to 0 to "start over"
	}
}

////////////////////////////////////////////////////////////////////////////////

void GUI_IncrementIntervalCount(void)
{
	if (_GUI.alive)
	{
		_GUI.updateIntervalCountMs++;
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean GUI_RefreshIntervalReached(void)
{
	if (_GUI.alive)
	{
		if (_GUI.updateIntervalCountMs >= _GUI.updateIntervalMs)
		{
			_GUI.updateIntervalCountMs = 0;
			return(TRUE);
		}
	}
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddEntry( int8_t line, LCD_addr_t labelX, byte labelJust, byte labelFgColorIndex, byte labelBgColorIndex, char *label,
		LCD_addr_t valueX, byte valueJust, byte valueFgColorIndex, byte valueBgColorIndex, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex)
{	// maximal info to create entry for display.
	if (_GUI.alive && (_GUI.current_dPtr < &_GUI.displayList[NUM_DISPLAY_LIST_ENTRIES]))
	{	// alive and have room in the buffer
		displayListEntryStruct *dlp = _GUI.current_dPtr;
		dlp->line = line;
		dlp->labelX = labelX;
		dlp->labelJust = labelJust;
		dlp->labelFgColorIndex = labelFgColorIndex;
		dlp->labelBgColorIndex = labelBgColorIndex;
		strncpy(dlp->label, label, GUI_MAX_LABEL_LENGTH);
		dlp->label[GUI_MAX_LABEL_LENGTH] = NULL_CHAR;

		dlp->valueX = valueX;
		dlp->valueJust = valueJust;
		dlp->valueFgColorIndex = valueFgColorIndex;
		dlp->valueBgColorIndex = valueBgColorIndex;
		dlp->valueWidth = valueWidth;
		dlp->valueSigDigits = valueSigDigits;

		dlp->valueIndex = valueIndex;
		dlp->valueSubIndex = valueSubIndex;
		_GUI.current_dPtr++;
	}
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddDefaultEntry(int8_t line, LCD_addr_t labelX, char *label, LCD_addr_t valueX, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex)
{	// minimal info to create an entry (label and value) on the display.  assumes default color/justifications  and labels and value X positions (but allows foe a value offset
	GUI_AddEntry(line, labelX, GUI_JUSTIFICATION_LEFT, LCD_COLOR_INDEX_THEME_LABEL_FG, LCD_COLOR_INDEX_THEME_LABEL_BG, label,
			valueX, GUI_JUSTIFICATION_RIGHT, LCD_COLOR_INDEX_THEME_VALUE_FG, LCD_COLOR_INDEX_THEME_VALUE_BG, valueWidth, valueSigDigits, valueIndex, valueSubIndex);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddLabel(int8_t line, LCD_addr_t labelX, int8_t labelJust, byte labelFgColorIndex, byte labelBgColorIndex, char *label)
{	// add a label to the display with control over position, just, and color
	GUI_AddEntry(line, labelX, labelJust, labelFgColorIndex, labelBgColorIndex, label, 0, 0, 0, 0, 0, 0, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddDefaultLabel(int8_t line, LCD_addr_t labelX, char *label)
{	// add a label to the display with control over position
	GUI_AddEntry(line, labelX, GUI_JUSTIFICATION_LEFT, LCD_COLOR_INDEX_THEME_LABEL_FG, LCD_COLOR_INDEX_THEME_LABEL_BG, label, 0, 0, 0, 0, 0, 0, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddValue(int8_t line, LCD_addr_t valueX, byte valueJust, byte valueFgColorIndex, byte valueBgColorIndex, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex)
{	// add a value to the display with control over position, just, and color
	GUI_AddEntry(line, 0, 0, 0, 0, "", valueX, valueJust, valueFgColorIndex, valueBgColorIndex, valueWidth, valueSigDigits, valueIndex, valueSubIndex);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddDefaultValue(int8_t line, LCD_addr_t valueX, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex)
{	// add a value to the display with control over position
	GUI_AddEntry(line, 0, 0, 0, 0, "", valueX, GUI_JUSTIFICATION_RIGHT, LCD_COLOR_INDEX_THEME_VALUE_FG, LCD_COLOR_INDEX_THEME_VALUE_BG, valueWidth, valueSigDigits, valueIndex, valueSubIndex);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_SetupDefaultPageHeader(int8_t *line, boolean useFakeLeds)
{	// common routine for all head/pages to get uniformity.  shows dev revision AND leaves room for the fake LEDs
	GUI_SetFakeLedEnable(useFakeLeds); //all pages use FAKE LEDs unless they shut this off
	GUI_AddLabel(0, GUI_RightEdge(), GUI_JUSTIFICATION_RIGHT, CI_RED, CI_LBG, display_getStringForIndexedValue(IDX_SW_REV_STR, 0, 0, NULL));
	*line = 1;   	// user will start on line 1 (this header will be line 0)
}
void GUI_SelectThemeColors(byte theme)
{
	if (_GUI.alive)
	{
		switch(theme)
		{
		case GUI_THEME_BLACK_AND_BLUE_ON_WHITE:
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_LABEL_FG, LCD_COLOR_BLACK);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_VALUE_FG, LCD_COLOR_BLUE);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_LABEL_BG, LCD_COLOR_WHITE);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_VALUE_BG, LCD_COLOR_WHITE);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_BACKGROUND, LCD_COLOR_WHITE);
			break;
		case GUI_THEME_WHITE_AND_YELLOW_ON_BLACK:
		case GUI_THEME_DEFAULT:
		default:
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_LABEL_FG, LCD_COLOR_WHITE);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_VALUE_FG, LCD_COLOR_YELLOW);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_LABEL_BG, LCD_COLOR_BLACK);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_VALUE_BG, LCD_COLOR_BLACK);
			LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_BACKGROUND, LCD_COLOR_BLACK);
			break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

byte GUI_GetCurrPage(void)
{
	return(_GUI.currentPage);
}

////////////////////////////////////////////////////////////////////////////////

byte GUI_GetNextPage(void)
{
	return(_GUI.nextPageToDisplay);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_SetNextPage(byte page)
{
	_GUI.nextPageToDisplay = page;
}

////////////////////////////////////////////////////////////////////////////////

void GUI_DrawPage(byte page)
{
	if (_GUI.alive)
	{
		_GUI.currentPage = page;
		_GUI.nextPageToDisplay = page;	// force both to be the same at this moment
		prepareDisplayList();

		switch(page)
		{
		case GUI_PAGE_SPLASH_SCREEN:	display_showSplashScreen(); break;
		default:
			LCD_FillDisplay(LCD_GetColorFromIndex(LCD_COLOR_INDEX_THEME_BACKGROUND));
			display_initializePage(page);	// call to users code to initialize the labels/values/gfx on this page
			GUI_DrawFakeLeds(TRUE);//full redraw for new page -- display_initializePage() will set whether or not to use fake LEDs
			drawStaticPageEntries();
			drawDynamicPageEntries();
			break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void GUI_RedrawCurrentPage(void)
{
	GUI_DrawPage(_GUI.currentPage);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_DrawJustDynamicValues(void)
{	// refresh any potential dynamic portions of the current display page
	if (_GUI.alive && (_GUI.currentPage != 0))
	{
		drawDynamicPageEntries();
	}
}

////////////////////////////////////////////////////////////////////////////////

uint32_t getCurrentLedValue(void)
{
#ifdef COMPILE_FOR_SYSTEM
	_gs._led.currValue.baseline.heartbeat	= _gs._led.heartbeatState 			? 1 : 0;
	_gs._led.currValue.baseline.canTx 		= (_gs._led.canTxLedCnt > 0) 		? 1 : 0;
	_gs._led.currValue.baseline.canRx		= (_gs._led.canRxLedCnt > 0) 		? 1 : 0;
	_gs._led.currValue.baseline.error 		= _gs._errorCount 					? 1 : 0;

	_gs._led.currValue.baseline.blockAll	= _blockAllMotion 					? 1 : 0;
	_gs._led.currValue.baseline.blockAbs	= (_blockAbsoluteMotion || anyAxisNeedingToBeHomed()) ? 1 : 0;
	_gs._led.currValue.baseline.paused		= (_gcodePaused || _g4DwellTimer)	? 1 : 0;
	_gs._led.currValue.baseline.moving		= motionQ.oldest->flags.inMotion	? 1 : 0;	//(motionQ.validEntries)
#endif //COMPILE_FOR_SYSTEM
	return(_gs._led.currValue.u32);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_DrawFakeLeds(boolean forceRedraw)
{	// force redraw will for ALL LED position to draw AND draw their border
	if (_GUI.alive && _GUI.fakeLedEnabled)
	{
		LCD_color_t bgColor = LCD_GetColorFromIndex(LCD_COLOR_INDEX_THEME_BACKGROUND);
		uint32_t currValue = getCurrentLedValue();
		uint32_t lastValue = _gs._led.lastLcdLedValue;

		if ((currValue != lastValue) || forceRedraw)
		{   // something changed, so update changed fake led(s)
			uint32_t mask;
			LCD_color_t color;
			uint16_t fWidth = LCD_GetFontWidth() - 1; //shrink a smidge from the font size
			uint16_t fHeight = LCD_GetFontHeight() - 1; //shrink a smidge from the font size
			for (int i=0; i<8; i++)
			{
				mask = 1 << i;
				if (((currValue & mask) != (lastValue & mask)) || forceRedraw)
				{   // value changed, so update display
					switch(i)
					{
					case 0: color = LCD_COLOR_RED; break;
					case 1:
					case 2:
					case 3: color = LCD_COLOR_ORANGE; break;
					case 4:
					case 5:
					case 6: color = LCD_COLOR_YELLOW; break;
					case 7: color = LCD_COLOR_BLUE; break;
					default: color = LCD_COLOR_BLACK; break;
					}
					if (forceRedraw)
						LCD_DrawFilledRect(_GUI.hOfs+((7-i)*fWidth), _GUI.vOfs, fWidth, fHeight, 1, bgColor, ((currValue & mask) ? color : LCD_COLOR_GRAY_50PCT));
					else
						LCD_FillRect(_GUI.hOfs+((7-i)*fWidth)+1, _GUI.vOfs+1, fWidth-2, fHeight-2, ((currValue & mask) ? color : LCD_COLOR_GRAY_50PCT));
				}
			}
			_gs._led.lastLcdLedValue = currValue;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean GUI_GetFakeLedEnable(void)
{
	return(_GUI.fakeLedEnabled);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_SetFakeLedEnable(boolean enable)
{
	_GUI.fakeLedEnabled = enable;
}
//NUKE
//#ifdef COMPILE_FOR_SYSTEM
//void getStringFromWorkingBuffer(char *dstr, int16_t length)
//{
//	memcpy(dstr, &_GcodeArgtringParam[1], length);
//	dstr[length] = NULL_CHAR;	//add null, not sent
//}
//#endif //COMPILE_FOR_SYSTEM

////////////////////////////////////////////////////////////////////////////////
///////////////////////////// GUI COMMANDS /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void getImageFromWorkingBuffer(void)
{//GB XXX FINISH

}

////////////////////////////////////////////////////////////////////////////////

void sendCanGuiCmdAckknowledge(void)
{
	canDevicePackWriteIntoTxQueue2x32(CAN_MSG_LOOPBACK, GUI_CMD_PROCESSED, _GUI.alive, 0);	// all GUI_CAN_CMDs need to be acked via the LOOPBACK message
}

////////////////////////////////////////////////////////////////////////////////

void processGuiCommand(GUI_canCommand_t cmd, payloadUnion *payload)
{
#ifdef COMPILE_FOR_DEVICE
	sendCanGuiCmdAckknowledge(); // presend the "ACK" to try to speed things up. -- WARNING: MIGHT BE A RISK FOR IMAGE COMMANDS!!!
#endif //COMPILE_FOR_DEVICE
	boolean redraw = FALSE;
	switch(cmd)
	{
	case GUI_CMD_DISPLAY_OFF:
		LCD_DisplayOff();
		break;
	case GUI_CMD_DISPLAY_ON:
		LCD_DisplayOn();
		break;
	case GUI_CMD_DISPLAY_INVERT:
		LCD_DisplayInvert(payload->u8[0]);
		break;
	case GUI_CMD_DISPLAY_CLEAR:
		LCD_FillDisplay((LCD_color_t)payload->u16[0]);
		break;
	case GUI_CMD_DISPLAY_ROTATION:
		LCD_SetRotation(payload->u8[0]);
		redraw = TRUE;
		break;
	case GUI_CMD_SELECT_PANEL:
		GUI_Init(payload->u8[0], LCD_ROTATION_DEFAULT, LCD_GetFont(), _GUI.spi, _GUI.backlightBBpinAddr, FALSE);
		redraw = TRUE;
		break;
	case GUI_CMD_SELECT_FONT:
		LCD_SetFont(payload->u8[0]);
		redraw = TRUE;
		break;
	case GUI_CMD_SET_PAGE:
		GUI_SetNextPage(payload->u8[0]);
		break;
	case GUI_CMD_SET_REFRESH_INTERVAL:
		_GUI.updateIntervalMs = payload->u16[0];
		break;
	case GUI_CMD_SELECT_THEME:
		GUI_SelectThemeColors(payload->u8[0]);
		redraw = TRUE;
		break;
	case GUI_CMD_FORCE_REDRAW:
		redraw = TRUE;
		break;
	case GUI_CMD_DISPLAY_BRIGHTNESS:
		LCD_UpdateBacklightDutyCycle(((int)100 * (int)payload->u16[0]) >> HH_U16_POWER_PCT_FRAC_BITS);
		break;
	case GUI_CMD_FAKE_LEDS_ENABLE:
		GUI_SetFakeLedEnable(payload->u8[0]);
		GUI_DrawFakeLeds(TRUE);
		break;
	case GUI_CMD_SET_BORDER_WIDTH:
		_GUI.cmd.borderWidth = payload->u16[0];
		break;
	case GUI_CMD_SET_COLOR_BORDER:
		_GUI.cmd.borderColor = (LCD_color_t)payload->u16[0];
		break;
	case GUI_CMD_SET_COLOR_FILL:
		_GUI.cmd.fillColor = (LCD_color_t)payload->u16[0];
		break;
	case GUI_CMD_SET_COLOR_FONT:
		_GUI.cmd.fontBackgroundColor = (LCD_color_t)payload->u16[0];
		_GUI.cmd.fontForegroundColor = (LCD_color_t)payload->u16[1];
		break;
	case GUI_CMD_SET_COLOR_THEME_LABEL:
		LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_LABEL_BG, (LCD_color_t)payload->u16[0]);
		LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_LABEL_FG, (LCD_color_t)payload->u16[1]);
		redraw = TRUE;
		break;
	case GUI_CMD_SET_COLOR_THEME_VALUE:
		LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_VALUE_BG, (LCD_color_t)payload->u16[1]);
		LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_VALUE_FG, (LCD_color_t)payload->u16[0]);
		redraw = TRUE;
		break;
	case GUI_CMD_SET_COLOR_THEME_BG:
		LCD_SetColorIndexData(LCD_COLOR_INDEX_THEME_BACKGROUND, (LCD_color_t)payload->u16[0]);
		redraw = TRUE;
		break;
	case GUI_CMD_SET_CHROMA_KEY:
		LCD_SetChromaKey((LCD_color_t)payload->u16[0]);
		break;
	case GUI_CMD_DRAW_PIXEL:
		LCD_DrawPixel(payload->i16[0], payload->i16[1], _GUI.cmd.fillColor);
		break;
	case GUI_CMD_DRAW_PIXEL_W_COLOR:
		LCD_DrawPixel(payload->i16[0], payload->i16[1], (LCD_color_t)payload->u16[2]);
		break;
	case GUI_CMD_DRAW_HORIZ_LINE:
		LCD_DrawHLine(payload->i16[0], payload->i16[1], payload->u16[2], payload->u16[3], _GUI.cmd.borderColor);
		break;
	case GUI_CMD_DRAW_VERT_LINE:
		LCD_DrawVLine(payload->i16[0], payload->i16[1], payload->u16[2], payload->u16[3], _GUI.cmd.borderColor);
		break;
	case GUI_CMD_DRAW_LINE:
		LCD_DrawLine(payload->i16[0], payload->i16[1], payload->i16[2], payload->i16[3], _GUI.cmd.borderWidth, _GUI.cmd.borderColor);
		break;
	case GUI_CMD_DRAW_RECT:
		LCD_DrawRect(payload->i16[0], payload->i16[1], payload->u16[2], payload->u16[3], _GUI.cmd.borderWidth, _GUI.cmd.borderColor);
		break;
	case GUI_CMD_FILL_RECT:
		LCD_FillRect(payload->i16[0], payload->i16[1], payload->u16[2], payload->u16[3], _GUI.cmd.fillColor);
		break;
	case GUI_CMD_DRAW_FILL_RECT:
		LCD_DrawFilledRect(payload->i16[0], payload->i16[1], payload->u16[2], payload->u16[3], _GUI.cmd.borderWidth, _GUI.cmd.borderColor, _GUI.cmd.fillColor);
		break;
	case GUI_CMD_DRAW_CIRC:
		LCD_DrawCircle(payload->i16[0], payload->i16[1], payload->u16[2], _GUI.cmd.borderWidth, _GUI.cmd.borderColor);
		break;
	case GUI_CMD_FILL_CIRC:
		LCD_FillCircle(payload->i16[0], payload->i16[1], payload->u16[2], _GUI.cmd.fillColor);
		break;
	case GUI_CMD_DRAW_FILL_CIRC:
		LCD_DrawFilledCircle(payload->i16[0], payload->i16[1], payload->u16[2], _GUI.cmd.borderWidth, _GUI.cmd.borderColor, _GUI.cmd.fillColor);
		break;
	case GUI_CMD_DRAW_CHAR:
		//i16[0]=x i16[1]=y  u8[4]=char u8[5]=just
		LCD_SetFontColors(_GUI.cmd.fontForegroundColor, _GUI.cmd.fontBackgroundColor);
		LCD_DrawCharJust(payload->i16[0], payload->i16[1], payload->u8[4], payload->u8[4]);	// x, y, chr, just
		break;
	case GUI_CMD_DRAW_STRING:
		//i16[0]=x i16[1]=y             u8[5]=just
		LCD_SetFontColors(_GUI.cmd.fontForegroundColor, _GUI.cmd.fontBackgroundColor);
		//NUKE getStringFromWorkingBuffer(_GUI.cmd.displayString)
		strcpy(_GUI.cmd.displayString, (char *)WORKING_BUFFER_BYTE_START_ADDRESS);
		LCD_DrawStringJust(payload->i16[0], payload->i16[1], _GUI.cmd.displayString, payload->u8[5]);	//x, y, str, just
		break;
	case GUI_CMD_DRAW_INDEXED_VARIABLE:
	{	//i16[0]=x i16[1]=y u8[4]=index u8[5]=just u8[6]=width u8[7]=sigDigits

		LCD_addr_t x = payload->i16[0];
		LCD_addr_t y = payload->i16[1];
		byte index = payload->u8[4];
		byte just = payload->u8[5];
		byte valueWidth = payload->u8[6];
		byte valueSigDigits = payload->u8[7];

		byte colorIndex = _GUI.cmd.fontForegroundColor;

		char *s = display_getStringForIndexedValue(index, 0, valueSigDigits, &colorIndex);		// index, fmt, CI (str returned)
		LCD_SetFontColors(LCD_GetColorFromIndex(colorIndex), _GUI.cmd.fontBackgroundColor);
		LCD_PadAndJustifyStringToSize(s, just, valueWidth);				// str, just, width (str returned in same loc)
		LCD_DrawStringJust(x, y, s, just); 		//x, y, str, just
		break;
	}
	case GUI_CMD_START_NEW_SCREEN:
	{	//u16[0]=g-page; u8[2]=header; u8[3]=leds
		boolean useFakeLeds = (payload->u8[3] == 1);
		GUI_SetFakeLedEnable(useFakeLeds);
		GUI_SetNextPage(payload->u16[0]);
		int8_t line=0;
		if (payload->u8[2] == 1)
		{
			GUI_SetupDefaultPageHeader(&line, useFakeLeds);	// will set X positions to their respective defaults
		}
		redraw = TRUE;
	}
		break;
	case GUI_CMD_ADD_SCREEN_ENTRY:
	{	//i16[0]=x i16[1]=y  u8[4]=index  u8[5]=line  u8[6]=valueWidth, u8[7]=valueSigDigits
		int labOfs = payload->i16[0];
		int valOfs = payload->i16[1];
		byte index = payload->u8[4];
		byte line = payload->u8[5];
		int8_t valueWidth = payload->i8[6];
		byte valueSigDigits = payload->u8[7];
		//NUKE getStringFromWorkingBuffer(_GUI.cmd.displayString);
		strcpy(_GUI.cmd.displayString, (char *)WORKING_BUFFER_BYTE_START_ADDRESS);
		GUI_AddDefaultEntry(line, GUI_LeftEdge()+labOfs, _GUI.cmd.displayString, GUI_RightEdge()+valOfs, valueWidth, valueSigDigits, index, 0);
		redraw = TRUE;
		break;
	}

#ifdef GB_HIDDEN_WARNINGS
#warning "FINISH - ADD CAN_GUI IMAGE COMMANDS"
#endif //GB_HIDDEN_WARNINGS

	//		case GUI_CMD_IMAGE_REGION:
	//#ifdef OPTIMIZED_CAN_GUI_IMAGES
	//			// requires no other display activity (like fakeLEDs) occur while full image is sent
	//			if (needToClip)
	//			GUI.cmd.totalPixels = _LCD.driver->SetDisplayWindow(payload->i16[0], payload->i16[1], payload->i16[2], payload->i16[3]);
	//#else //!
	//			_GUI.cmd.origX = payload->i16[0];
	//			_GUI.cmd.origY = payload->i16[1];
	//			_GUI.cmd.width = payload->u16[2];
	//			_GUI.cmd.height = payload->u16[3];
	//			_GUI.cmd.currX = _GUI.cmd.origX;
	//			_GUI.cmd.currY = _GUI.cmd.origY;
	//			_GUI.cmd.widthX = LCD_GetDisplayWidth();
	//			_GUI.cmd.widthY = LCD_GetDisplayHeight();
	//			_GUI.cmd.totalPixels = 0;
	//			_GUI.cmd.pixels[8];
	//			_GUI.cmd.numPixels;	//pixels in next xfer
	//			break;
	//		case GUI_CMD_IMAGE_COLOR_16_BIT:			// 4, 16-bit colors RGB565
	//			break;
	//		case GUI_CMD_IMAGE_STR_COLOR_16_BIT: 		// 4, 16-bit colors RGB565
	//			_GUI.cmd.numPixels = imin16(_GUI.cmd.widthX-_GUI.cmd.currX, 4);
	//#ifdef OPTIMIZED_CAN_GUI_IMAGES
	//			LCD_SPI_WriteArray16(&payload->u16[0], _GUI.cmd.numPixels);
	//#else //!OPTIMIZED_CAN_GUI_IMAGES
	//			LCD_DrawRGBImage(_GUI.cmd.currX, _GUI.cmd.currY, _GUI.cmd.numPixels, 1, _gs.workingBuffer);
	//			//void LCD_DrawRGBImage(LCD_addr_t x, LCD_addr_t y, nt16_t width, uint16_t height, LCD_color_t *dataPtr)
	//#endif //!!OPTIMIZED_CAN_GUI_IMAGES
	//			break;
	//		case GUI_CMD_IMAGE_STR_COLOR_8_BIT:  		// 8, 8-bit colors RGB332
	//		case GUI_CMD_IMAGE_STR_COLOR_4_BIT:  		// 16, 4-bit colors RGB121
	//		case GUI_CMD_IMAGE_STR_INDEX_8_BIT:  		// 8, 8-bit color index
	//		case GUI_CMD_IMAGE_STR_INDEX_4_BIT:  		// 16, 4-bit color index
	//		case GUI_CMD_IMAGE_STR_INDEX_1_BIT:  		// 64, 1-bit colors (0)BG and (1)FG
	//			getImageFromWorkingBuffer();
	//			break;

	//for debug, but could be useful...
	case GUI_CMD_SET_PANEL_OFS_XY:
		LCD_SetPanelOfsets(payload->i16[0], payload->i16[1]);
		break;
	default:
		break;
	}
	if (redraw == TRUE)
	{	// redraw in the foreground
		_gs._displayForceRedraw = TRUE;
	}
#ifdef COMPILE_FOR_SYSTEM
	_waitingForGuiCommand = FALSE;
#endif //COMPILE_FOR_SYSTEM
}

////////////////////////////////////////////////////////////////////////////////

boolean GUI_CommandQueueFull(void)
{
	return(_GUI.cmd.Q_validEntries >= CAN_GUI_QUEUE_SIZE);
}

////////////////////////////////////////////////////////////////////////////////

void GUI_AddCommandToQueue(GUI_canCommand_t cmd, payloadUnion *payload)
{
	// need to put in a queue for later processing in the FOREGROUOND to not step on any other display activity in process
	if (_GUI.cmd.Q_validEntries >= CAN_GUI_QUEUE_SIZE)
	{   // no room in the queue, so report error and ignore command
#ifdef COMPILE_FOR_DEVICE
		reportError1x32(STICKY_GUI_QUEUE_FULL, ERROR_UNIT_CAN, ERROR_GUI_QUEUE_FULL, CAN_GUI_QUEUE_SIZE);
#endif //COMPILE_FOR_DEVICE
#ifdef COMPILE_FOR_SYSTEM
		if (_errors.sent.flags.guiQueueFull == FALSE)
		{
			_errors.sent.flags.guiQueueFull = TRUE;
			sendError("GUI_QUEUE_FULL");
		}
#endif //COMPILE_FOR_SYSTEM
	}
	else
	{   // add to queue
		_GUI.cmd.Q[_GUI.cmd.Q_indexIn].cmd = cmd;
		_GUI.cmd.Q[_GUI.cmd.Q_indexIn].payload = *payload;
		_GUI.cmd.Q_validEntries++;
		_GUI.cmd.Q_indexIn = (_GUI.cmd.Q_indexIn + 1) % CAN_GUI_QUEUE_SIZE;
	}
}

////////////////////////////////////////////////////////////////////////////////

void GUI_ProcessCommandsFromQueue(int quantity)
{
	for (int i=0; i<quantity; i++)
	{
		uint32_t irq_disabled = interruptsOff();
		if (_GUI.cmd.Q_validEntries > 0)
		{   // at least one valid command
			interruptsOn(irq_disabled);

			processGuiCommand(_GUI.cmd.Q[_GUI.cmd.Q_indexOut].cmd, &_GUI.cmd.Q[_GUI.cmd.Q_indexOut].payload);

			uint32_t irq_disabled = interruptsOff();
			_GUI.cmd.Q_indexOut = (_GUI.cmd.Q_indexOut + 1) % CAN_GUI_QUEUE_SIZE;
			_GUI.cmd.Q_validEntries--;
			interruptsOn(irq_disabled);
		}
		else
		{	// no entries left, so bail out
			interruptsOn(irq_disabled);
			break;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void GUI_SetCmdDefaults(void)
{
	memZero((byte *)&_GUI.cmd, sizeof(guiCmdStruct));

	_GUI.cmd.borderWidth = 1;
	_GUI.cmd.borderColor = LCD_COLOR_BLACK;
	_GUI.cmd.fillColor = LCD_COLOR_WHITE;
	_GUI.cmd.fontBackgroundColor = LCD_COLOR_BLACK;
	_GUI.cmd.fontForegroundColor = LCD_COLOR_WHITE;
}

////////////////////////////////////////////////////////////////////////////////

boolean GUI_Init(byte panelIndex, byte rotation, byte font, SPI_TypeDef *SPIx, uint32_t backlightBBpinAddr, boolean zeroMemory)
{
	if (zeroMemory)
		memZero((byte *)&_GUI, sizeof(GUI_driverStruct)); // start fresh

	_GUI.backlightBBpinAddr = backlightBBpinAddr;	// copy in case need to re-init from the canbus later

	if (LCD_Init(panelIndex, SPIx, _GUI.backlightBBpinAddr))
	{	// legal panel
		// note, LCD_Init() loads a black screen
		LCD_SetRotation(rotation);
		LCD_SetFont(font);
		_GUI.alive = TRUE;
		_GUI.spi = SPIx;	// needed in case display is re-init from the canbus
		//load reasonable defaults
		setGuiDisplayOffsets(panelIndex);
		if (zeroMemory)
		{
			GUI_SetCmdDefaults();
			GUI_SelectThemeColors(GUI_THEME_WHITE_AND_YELLOW_ON_BLACK);
			LCD_SetFontColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);
			_GUI.updateIntervalMs = 200; // set 5Hz update rate as default
		}
	}
	return(_GUI.alive);
}

////////////////////////////////////////////////////////////////////////////////

#endif // ADD_ON_SPI_DISPLAY

