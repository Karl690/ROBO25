#ifdef ADD_ON_SPI_DISPLAY
#ifndef gui_HEADER // prevent double dipping
#define gui_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    gui.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: include various gui releated functions
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "hyrel.h"
#include "hyrel_can.h"
#include "lcd.h"

////////////////////////////////////////////////////////////////////////////////

// shortened names for condensed tables
#define CI_INV		LCD_COLOR_INDEX_TRANSPARENT
#define CI_BLK		LCD_COLOR_INDEX_BLACK
#define CI_BRN		LCD_COLOR_INDEX_BROWN
#define CI_RED		LCD_COLOR_INDEX_RED
#define CI_ORA		LCD_COLOR_INDEX_ORANGE
#define CI_YEL		LCD_COLOR_INDEX_YELLOW
#define CI_GRN		LCD_COLOR_INDEX_GREEN
#define CI_BLU		LCD_COLOR_INDEX_BLUE
#define CI_PUR		LCD_COLOR_INDEX_PURPLE
#define CI_GRY		LCD_COLOR_INDEX_GRAY
#define CI_WHI		LCD_COLOR_INDEX_WHITE
#define CI_CYN		LCD_COLOR_INDEX_CYAN
#define CI_MAG		LCD_COLOR_INDEX_MAGENTA

#define CI_LFG		LCD_COLOR_INDEX_THEME_LABEL_FG
#define CI_LBG		LCD_COLOR_INDEX_THEME_LABEL_BG
#define CI_VFG		LCD_COLOR_INDEX_THEME_VALUE_FG
#define CI_VBG		LCD_COLOR_INDEX_THEME_VALUE_BG
#define CI_BG		LCD_COLOR_INDEX_THEME_BACKGROUND

#define	GUI_JUSTIFICATION_LEFT			LCD_JUSTIFICATION_LEFT
#define GUI_JUSTIFICATION_CENTER		LCD_JUSTIFICATION_CENTER
#define GUI_JUSTIFICATION_RIGHT			LCD_JUSTIFICATION_RIGHT
#define GUI_JUSTIFICATION_TOP			LCD_JUSTIFICATION_TOP
#define GUI_JUSTIFICATION_BOTTOM		LCD_JUSTIFICATION_BOTTOM

#define JL 			GUI_JUSTIFICATION_LEFT
#define JC			GUI_JUSTIFICATION_CENTER
#define JR 			GUI_JUSTIFICATION_RIGHT
#define JT			GUI_JUSTIFICATION_TOP
#define JB			GUI_JUSTIFICATION_BOTTOM

#define INV			LCD_COLOR_TRANSPARENT
#define BLK			LCD_COLOR_BLACK
#define BRN			LCD_COLOR_BROWN
#define RED			LCD_COLOR_RED
#define ORA			LCD_COLOR_ORANGE
#define YEL			LCD_COLOR_YELLOW
#define GRN			LCD_COLOR_GREEN
#define BLU			LCD_COLOR_BLUE
#define PUR			LCD_COLOR_PURPLE
#define GRY			LCD_COLOR_GRAY
#define WHI			LCD_COLOR_WHITE
#define CYN			LCD_COLOR_CYAN
#define MAG			LCD_COLOR_MAGENTA

#ifdef COMPILE_FOR_DEVICE
#define CAN_GUI_QUEUE_SIZE 1	// these commands will always be sent by the MC requiring an ACK before it will move on, so need for a QUEUE
#endif
#ifdef COMPILE_FOR_SYSTEM
#define CAN_GUI_QUEUE_SIZE 64
#endif
#ifdef BASIC_GUI_ONLY
#define NUM_DISPLAY_LIST_ENTRIES 20
#define GUI_MAX_LABEL_LENGTH 11
#else //!BASIC_GUI_ONLY
#define NUM_DISPLAY_LIST_ENTRIES 42
#define GUI_MAX_LABEL_LENGTH 23
#endif //!BASIC_GUI_ONLY

#define GUI_PAGE_SPLASH_SCREEN		0

// DO NOT CHANGE VALUES
#define FMT_OFF		0
#define FMT_STR		1
#define FMT_CHR		2
#define FMT_I8		3
#define FMT_I16		4
#define FMT_I32		5
#define FMT_I64		6
#define FMT_U8		7
#define FMT_U16		8
#define FMT_U32		9
#define FMT_U64		10
#define FMT_FL0		11	// floating point		 no digits after the decimal point
#define FMT_FL1		12	// floating point		 one digit after the decimal point
#define FMT_FL2		13
#define FMT_FL3		14
#define FMT_FL4		15
#define FMT_FL5		16
#define FMT_FL6		17
#define NUM_FORMATS		18

typedef struct {
	int8_t 		line;

	LCD_addr_t  labelX;
	byte        labelJust;
	byte   		labelFgColorIndex;
	byte   		labelBgColorIndex;
	char        label[GUI_MAX_LABEL_LENGTH+1]; //+1 for the NULL_CHAR

	LCD_addr_t  valueX;
	byte  		valueJust;
	byte    	valueFgColorIndex;
	byte        valueBgColorIndex;
	int8_t      valueWidth;
	byte    	valueSigDigits;

	byte    	valueIndex;
	byte		valueSubIndex;
} displayListEntryStruct;

typedef struct {
	byte cmd;
	payloadUnion payload;
} canGuiCmdQueueStruct;

typedef struct {
	LCD_addr_t 	origX;
	LCD_addr_t 	origY;
	int 		width;
	int			height;
	LCD_addr_t 	currX;
	LCD_addr_t 	currY;
	LCD_addr_t	widthX;
	LCD_addr_t	widthY;
	LCD_color_t pixels[16];	// up to 16 pixels in a single can packet (4-bit CI mode)
	int			numPixels;	//pixels in the current xfer
} canImageRectStruct;

typedef struct {
	int                     borderWidth;
	LCD_color_t             borderColor;
	LCD_color_t             fillColor;
	LCD_color_t				fontForegroundColor;
	LCD_color_t				fontBackgroundColor;

	canImageRectStruct		image;

	int                     Q_validEntries;
	int                     Q_indexIn;
	int                     Q_indexOut;
	canGuiCmdQueueStruct    Q[CAN_GUI_QUEUE_SIZE];

	char                    displayString[MAX_CAN_STRING_SIZE+1]; // string to display sent via canbus
} guiCmdStruct;

typedef struct {
	boolean 				alive; //TRUE if init called and a valid LCD panel selected
	boolean					fakeLedEnabled;
	SPI_TypeDef 			*spi;

	byte                    currentPage;
	byte					nextPageToDisplay;
	int                     updateIntervalCountMs;
	int                     updateIntervalMs;
	int                     vOfs;
	int                     hOfs;

	uint32_t				backlightBBpinAddr;	// copy in case need to re-init LCD from can command
	guiCmdStruct			cmd;
	displayListEntryStruct  *current_dPtr;
	displayListEntryStruct	displayList[NUM_DISPLAY_LIST_ENTRIES];
} GUI_driverStruct;

////////////////////////////////////////////////////////////////////////////////

extern char *GUI_GetFormatStr(byte format);
extern char *GUI_GetFloatFormatStr(byte postDecimalPtDigits);
extern int GUI_LeftEdge(void);
extern int GUI_RightEdge(void);
extern int GUI_HorizMiddle(void);
extern int GUI_TopEdge(void);
extern int GUI_BottomEdge(void);
extern int GUI_VertMiddle(void);
extern int GUI_VertOffset(void);
extern int GUI_HorizOffset(void);
extern void GUI_SetRefreshInternalInMs(int ms);
extern void GUI_IncrementIntervalCount(void);
extern boolean GUI_RefreshIntervalReached(void);
extern void GUI_AddEntry( int8_t line, LCD_addr_t labelX, byte labelJust, byte labelFgColorIndex, byte labelBgColorIndex, char *label,
		LCD_addr_t valueX, byte valueJust, byte valueFgColorIndex, byte valueBgColorIndex, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex);
extern void GUI_AddSimpleEntry (int8_t line, LCD_addr_t labelX, char *label, LCD_addr_t valueX, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex);
extern void GUI_AddDefaultEntry(int8_t line, LCD_addr_t labelX, char *label, LCD_addr_t valueX, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex);
extern void GUI_AddLabel(int8_t line, LCD_addr_t labelX, int8_t labelJust, byte labelFgColorIndex, byte labelBgColorIndex, char *label);
extern void GUI_AddDefaultLabel(int8_t line, LCD_addr_t labelX, char *label);
extern void GUI_AddValue(int8_t line, LCD_addr_t valueX, byte valueJust, byte valueFgColorIndex, byte valueBgColorIndex, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex);
extern void GUI_AddDefaultValue(int8_t line, LCD_addr_t valueX, int8_t valueWidth, byte valueSigDigits, byte valueIndex, byte valueSubIndex);
extern void GUI_SetupDefaultPageHeader(int8_t *line, boolean useFakeLeds);

extern void GUI_SelectThemeColors(byte theme);
extern byte GUI_GetCurrPage(void);
extern byte GUI_GetNextPage(void);
extern void GUI_SetNextPage(byte page);
extern void GUI_DrawPage(byte page);
extern void GUI_RedrawCurrentPage(void);
extern void GUI_DrawJustDynamicValues(void);
extern void GUI_DrawFakeLeds(boolean);
extern boolean GUI_GetFakeLedEnable(void);
extern void GUI_SetFakeLedEnable(boolean enable);
extern void GUI_updateBrightness(void);
extern void GUI_UpdateDisplayStringFromCanPacket(canSwStruct *canRx);
extern boolean GUI_CommandQueueFull(void);
extern void GUI_AddCommandToQueue(GUI_canCommand_t cmd, payloadUnion *payload);
extern void GUI_ProcessCommandsFromQueue(int quantity);
//extern boolean GUI_isAlive(void);
extern boolean GUI_Init(byte panelIndex, byte rotation, byte font, SPI_TypeDef *SPIx, uint32_t backlightBBpinAddr, boolean zeroMemory);

////////////////////////////////////////////////////////////////////////////////
#endif // #ifndef gui_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
#endif //ADD_ON_SPI_DISPLAY

