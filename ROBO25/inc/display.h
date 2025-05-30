#ifdef ADD_ON_SPI_DISPLAY
#ifndef display_HEADER // prevent double dipping
#define display_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    display.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: include various display releated functions
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#define DISPLAY_PAGE_SPLASH_SCREEN		0
#define DISPLAY_PAGE_DEFAULT			1
#define DISPLAY_PAGE_CONTROLS			2
#define DISPLAY_PAGE_HEAD_LIST			3
#define DISPLAY_PAGE_PHYSICAL_PINS		4
#define DISPLAY_PAGE_LOGICAL_PINS		5
#define DISPLAY_PAGE_LAST_ONE			DISPLAY_PAGE_LOGICAL_PINS	// round robin through pages, need to know when to roll over

#define IDX_NONE				0
#define IDX_UPTIME				1
#define IDX_SW_REV_STR			2
#define IDX_POS_MM				3
#define IDX_POS_STEPS			4
#define IDX_FLOW_RATE			5
#define IDX_LINE_NUM			6
#define IDX_CURR_FEEDRATE		7
#define IDX_ACT_VECTRATE		8
#define IDX_NTH_DEV_POS			9
#define IDX_NTH_DEV_TYPE_STR	10

#define IDX_VAL_CHAR_STEP		20
#define IDX_VAL_CHAR_DIR		21
#define IDX_VAL_CHAR_HOME		22
#define IDX_VAL_CHAR_LIM1		23
#define IDX_VAL_CHAR_LIM2		24
#define IDX_VAL_CHAR_FAULT		25
#define IDX_VAL_CHAR_START		26
#define IDX_VAL_CHAR_ABENC		27
#define IDX_VAL_CHAR_EMO		28

#define IDX_STATE_CHAR_STEP		30
#define IDX_STATE_CHAR_DIR		31
#define IDX_STATE_CHAR_HOME		32
#define IDX_STATE_CHAR_LIM1		33
#define IDX_STATE_CHAR_LIM2		34
#define IDX_STATE_CHAR_FAULT	35
#define IDX_STATE_CHAR_START	36
#define IDX_STATE_CHAR_ABENC	37
#define IDX_STATE_CHAR_EMO		38

#define IDX_STATE_STR_EMO		48

#define IDX_CURR_SLICE			50
#define IDX_PEND_ACK			51
#define IDX_CMD_IN_CMDQUE		52
#define IDX_CMD_IN_MOTIONQ		53
#define IDX_BLOCK_ALL			54
#define IDX_BLOCK_ABS			55
#define IDX_UNHOMED				56
#define IDX_GCCODE_PAUSED		57
#define IDX_DWELL_TIME			58
#define IDX_IN_MOTION			59

typedef int16_t LCD_addr_t;

extern void display_Init(void);
extern void display_initializePage(int page);
extern void display_showSplashScreen(void);
extern char *display_getStringForIndexedValue(byte strIndex, byte strSubIndex, byte postDecimalPtDigits, byte *colorIndex);

////////////////////////////////////////////////////////////////////////////////
#endif // #ifndef display_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
#endif //ADD_ON_SPI_DISPLAY

