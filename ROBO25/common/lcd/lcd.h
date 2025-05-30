#ifdef ADD_ON_SPI_DISPLAY
#ifndef lcd_HEADER // prevent double dipping
#define lcd_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    lcd.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: include for spi display / font related functions
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include "main.h"
#include "hyrel.h"
#include "hyrel_can.h"
#include "pinout.h"

typedef uint16_t  LCD_color_t;

// RGB565 color definitions   5-bit R : 6-bit G : 5-bit B
#define LCD_COLOR_BLACK					0x0000				// RGB=0,0,0  REPETREL=0
#define LCD_COLOR_WHITE					0xFFFF				// RGB=255,255,255  REPETREL=65535
#define LCD_COLOR_RED					0xF800				// RGB=255,0,0  REPETREL=63488
#define LCD_COLOR_GREEN					0x07E0				// RGB=0,255,0  REPETREL=2016
#define LCD_COLOR_BLUE					0x001F				// RGB=0,0,255  REPETREL=31
#define LCD_COLOR_YELLOW				0xFFE0				// RGB=255,255,0  REPETREL=65504
#define LCD_COLOR_MAGENTA				0xF81F				// RGB=255,0,255  REPETREL=63519
#define LCD_COLOR_CYAN					0x07FF				// RGB=0,255,255  REPETREL=2047
#define LCD_COLOR_BROWN					0x92C3				// RGB=140,89,25  REPETREL=37571
#define LCD_COLOR_ORANGE				0xFD40				// RGB=255,165,0  REPETREL=64832
#define LCD_COLOR_PURPLE				0x801F				// RGB=127,0,255  REPETREL=32799
#define LCD_COLOR_YELLOW_GREEN			0xB7E6				// RGB=173,255,45  REPETREL=47078
#define LCD_COLOR_PINK					0xFE7D				// RGB=255,204,229  REPETREL=65149
#define LCD_COLOR_OLIVE					0xBDCA				// RGB=183,183,79  REPETREL=48586
#define LCD_COLOR_GRAY					0xD67A				// RGB=204,204,204  REPETREL=54906
#define LCD_COLOR_TRANSPARENT			0x0020				// RGB=0,5,0  REPETREL=32

#define LCD_COLOR_RED_100PCT			0xF800				// RGB=255,0,0  REPETREL=63488
#define LCD_COLOR_RED_75PCT				0xC000				// RGB=191,0,0  REPETREL=49152
#define LCD_COLOR_RED_50PCT				0x8000				// RGB=127,0,0  REPETREL=32768
#define LCD_COLOR_RED_25PCT				0x4000				// RGB=63,0,0  REPETREL=16384
#define LCD_COLOR_GREEN_100PCT			0x07E0				// RGB=0,255,0  REPETREL=2016
#define LCD_COLOR_GREEN_75PCT			0x0600				// RGB=0,191,0  REPETREL=1536
#define LCD_COLOR_GREEN_50PCT			0x0400				// RGB=0,127,0  REPETREL=1024
#define LCD_COLOR_GREEN_25PCT			0x0200				// RGB=0,63,0  REPETREL=512
#define LCD_COLOR_BLUE_100PCT			0x001F				// RGB=0,0,255  REPETREL=31
#define LCD_COLOR_BLUE_75PCT			0x0018				// RGB=0,0,191  REPETREL=24
#define LCD_COLOR_BLUE_50PCT			0x0010				// RGB=0,0,127  REPETREL=16
#define LCD_COLOR_BLUE_25PCT			0x0008				// RGB=0,0,63  REPETREL=8
#define LCD_COLOR_YELLOW_100PCT			0xFFE0				// RGB=255,255,0  REPETREL=65504
#define LCD_COLOR_YELLOW_75PCT			0xC600				// RGB=191,191,0  REPETREL=50688
#define LCD_COLOR_YELLOW_50PCT			0x8400				// RGB=127,127,0  REPETREL=33792
#define LCD_COLOR_YELLOW_25PCT			0x4200				// RGB=63,63,0  REPETREL=16896
#define LCD_COLOR_MAGENTA_100PCT		0xF81F				// RGB=255,0,255  REPETREL=63519
#define LCD_COLOR_MAGENTA_75PCT			0xC018				// RGB=191,0,191  REPETREL=49176
#define LCD_COLOR_MAGENTA_50PCT			0x8010				// RGB=127,0,127  REPETREL=32784
#define LCD_COLOR_MAGENTA_25PCT			0x4008				// RGB=63,0,63  REPETREL=16392
#define LCD_COLOR_CYAN_100PCT			0x07FF				// RGB=0,255,255  REPETREL=2047
#define LCD_COLOR_CYAN_75PCT			0x0618				// RGB=0,191,191  REPETREL=1560
#define LCD_COLOR_CYAN_50PCT			0x0410				// RGB=0,127,127  REPETREL=1040
#define LCD_COLOR_CYAN_25PCT			0x0208				// RGB=0,63,63  REPETREL=520
#define LCD_COLOR_GRAY_100PCT			0xD67A				// RGB=204,204,204  REPETREL=54906
#define LCD_COLOR_GRAY_75PCT			0x9CD3				// RGB=153,153,153  REPETREL=40147
#define LCD_COLOR_GRAY_50PCT			0x6B4D				// RGB=102,102,102  REPETREL=27469
#define LCD_COLOR_GRAY_25PCT			0x31A6				// RGB=51,51,51  REPETREL=12710

//WARNING: do not change the order .. hardcoded array order has to match index value
//         ALSO used in Mcode args by users
#define LCD_COLOR_INDEX_BLACK			0x00
#define LCD_COLOR_INDEX_WHITE			0x01
#define LCD_COLOR_INDEX_RED				0x02
#define LCD_COLOR_INDEX_GREEN			0x03
#define LCD_COLOR_INDEX_BLUE			0x04
#define LCD_COLOR_INDEX_YELLOW			0x05
#define LCD_COLOR_INDEX_MAGENTA			0x06
#define LCD_COLOR_INDEX_CYAN			0x07
#define LCD_COLOR_INDEX_BROWN			0x08
#define LCD_COLOR_INDEX_ORANGE			0x09
#define LCD_COLOR_INDEX_PURPLE			0x0A
#define LCD_COLOR_INDEX_YELLOW_GREEN	0x0B
#define LCD_COLOR_INDEX_PINK			0x0C
#define LCD_COLOR_INDEX_OLIVE			0x0D
#define LCD_COLOR_INDEX_GRAY			0x0E
#define LCD_COLOR_INDEX_TRANSPARENT		0x0F


// index into loadable/variable data starts at 0x80 (msb set)
#define LCD_COLOR_INDEX_THEME_LABEL_FG		0x80
#define LCD_COLOR_INDEX_THEME_LABEL_BG		0x81
#define LCD_COLOR_INDEX_THEME_VALUE_FG		0x82
#define LCD_COLOR_INDEX_THEME_VALUE_BG		0x83
#define LCD_COLOR_INDEX_THEME_BACKGROUND	0x84
#define LCD_COLOR_INDEX_FONT_FOREGROUND		0x85
#define LCD_COLOR_INDEX_FONT_BACKGROUND		0x86


#ifdef COMPILE_FOR_SYSTEM
#define LCD_RST_ENABLE      pinClear(PIN_LCD_RST)
#define LCD_RST_DISABLE     pinSet(PIN_LCD_RST)

#define LCD_CS_ENABLE       pinClear(PIN_LCD_CS)
#define LCD_CS_DISABLE      pinSet(PIN_LCD_CS)

#define LCD_CMD_ENABLE      pinClear(PIN_LCD_D_CB)
#define LCD_CMD_DISABLE     pinSet(PIN_LCD_D_CB)

#define LCD_DATA_ENABLE     pinSet(PIN_LCD_D_CB)
#define LCD_DATA_DISABLE    pinClear(PIN_LCD_D_CB)
#else
#define LCD_RST_ENABLE      {BB_PIN_LCD_RST = 0;}
#define LCD_RST_DISABLE     {BB_PIN_LCD_RST = 1;}

#define LCD_CS_ENABLE       {BB_PIN_LCD_CS = 0;}
#define LCD_CS_DISABLE      {BB_PIN_LCD_CS = 1;}

#define LCD_CMD_ENABLE      {BB_PIN_LCD_D_CB = 0;}
#define LCD_CMD_DISABLE     {BB_PIN_LCD_D_CB = 1;}

#define LCD_DATA_ENABLE     {BB_PIN_LCD_D_CB = 1;}
#define LCD_DATA_DISABLE    {BB_PIN_LCD_D_CB = 0;}

#define LCD_CLK_LOW			{BB_PIN_LCD_SCK = 0;}
#define LCD_CLK_HIGH		{BB_PIN_LCD_SCK = 1;}

// for bit banging display on BTT103 board
#ifdef BB_PIN_LCD_MOSI
#define LCD_SDO_LOW         {BB_PIN_LCD_MOSI = 0;}
#define LCD_SDO_HIGH        {BB_PIN_LCD_MOSI = 1;}
#else //BB_PIN_LCD_MOSI
#define LCD_SDO_LOW         {;}
#define LCD_SDO_HIGH        {;}
#endif //!BB_PIN_LCD_MOSI
#endif

#define NULL_CHAR '\0'

typedef int16_t LCD_addr_t;

typedef struct {
	byte         index;
	byte        *dataPtr;
	char        firstChar;
	int         width;
	int         height;
	int         bytesWide;
	int         bytesPerChar;
	LCD_color_t foregroundColor;
	LCD_color_t backgroundColor;
} LCD_fontStruct;

// DO NOT CHANGE THESE (font rot, just, tft .... also used as arguments in MCODEs

#define LCD_FONT_5x8					0
#define LCD_FONT_7x12					1
#define LCD_FONT_11x16					2
#define LCD_FONT_13x20					3
//#define LCD_FONT_14x20					3
#define LCD_FONT_16x22					4
//#define LCD_FONT_17x24					5	// RETIRED

#define LCD_ROTATION_0					0
#define LCD_ROTATION_90					1
#define LCD_ROTATION_180				2
#define LCD_ROTATION_270				3
#define LCD_ROTATION_0_MX				4
#define LCD_ROTATION_90_MX				5
#define LCD_ROTATION_180_MX				6
#define LCD_ROTATION_270_MX				7
#define LCD_ROTATION_DEFAULT			LCD_ROTATION_180	// most displays seem to orient best this way

#define	LCD_JUSTIFICATION_LEFT			0
#define LCD_JUSTIFICATION_CENTER		1
#define LCD_JUSTIFICATION_RIGHT			2
#define LCD_JUSTIFICATION_TOP			3
#define LCD_JUSTIFICATION_BOTTOM		4

#define LCD_TFT_DEVEBOX_SPI_TFT_280		0    	// ILI9341   240x320 no offsets
#define LCD_TFT_MCUDEV_TFT_180			1        // SS7735(s) 128x160 w/ 2,2 offset
#define LCD_TFT_MCUDEV_TFT_144          2		// SS7735(s) 128x128 w/ screwy offsets
#define LCD_TFT_MCUDEV_TFT_096          3		// SSD1306(s) 128x64 monochrome (spi)
#define LCD_TFT_MCUDEV_TFT_096_BB       4		// SSD1306(s) 128x64 monochrome (bit-bang)

typedef struct {
	// remaining are set by calls to Init
	byte     		index;          // diff displays using same controller
	uint16_t        defaultWidth;       // width of disp at rot 0
	uint16_t        defaultHeight;      // height of disp at rot 0
	byte  			rotation;           // curr rot index
	byte			xyMirrorRgbCtrl;	// value to send to display to control X, Y, Mirror, andRGB format
	uint16_t        width;              // current width (post rot)
	uint16_t        height;             // current height (post rot)
	uint16_t        maxXaddr;           // current width (post rot)
	uint16_t        maxYaddr;           // current height (post rot)
	uint16_t        ofsX;               // curr global X ofs for tft
	uint16_t        ofsY;               // curr global Y ofs for tft
	uint32_t        maxSpiFrequency;    // fastest baudrate suppported by display
	uint16_t        supports16BitSpi;   // 1 if tft/drvr combo support 16-bit spi
	uint16_t        flipRGB;            // 1 if need to flip RGB order in controller
	uint16_t        hasMISO;            // 1 if MISO is present (allowing reads)
	uint16_t        hasResetPin;        // 1 if the module has a physical reset pin
} LCD_panelStruct;

typedef enum {
	LCD_DRIVER_NONE = 0,
	LCD_DRIVER_ILI9341,
	LCD_DRIVER_ST7735,
	LCD_DRIVER_SSD1306,
} LCD_driver_t;

typedef struct {
	LCD_driver_t    index;
	void            (*InitPanelStruct)(LCD_panelStruct *panelPtr, byte index);
	void            (*InitDriver)(void);
	void            (*ReadID)(byte *rdata);
	void            (*ReadStatus)(byte *rdata);
	void            (*ReadColorMode)(byte *rdata);
	void            (*DisplayOn)(void);
	void            (*DisplayOff)(void);
	void            (*DisplayInvert)(uint16_t invert);
	void            (*DisplayFramebuffer)(void);
	void            (*SetRotation)(byte);
	uint32_t        (*SetCursor)(LCD_addr_t x, LCD_addr_t y);
	uint32_t        (*SetDisplayWindow)(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height);
	void            (*WritePixel)(LCD_addr_t x, LCD_addr_t y, LCD_color_t color);
	uint16_t        (*ReadPixel)(LCD_addr_t x, LCD_addr_t y);
} LCD_driverStruct;

typedef struct {
	int				dutyCycle;
	int				pwmCnt;
	//NUKE int				resolution;
	uint32_t		BBpinAddr;
	//NUKE boolean			usePulseTrain;
} LCD_backlightStruct;

typedef struct {
	LCD_driverStruct *driver;
	LCD_panelStruct  *panel;
	LCD_backlightStruct *backlight;
	LCD_color_t		chromaKey;
} LCD_struct;

extern LCD_struct *_lcd;
extern LCD_fontStruct *_font;
extern int _lcdSpiWatchdog;

extern boolean LCD_Init(byte tft, SPI_TypeDef *SPIx, uint32_t backlightBBpin);
extern void LCD_InitPanelStruct(LCD_panelStruct *panelPtr, byte index);
extern void LCD_InitDriver(void);
extern void LCD_DisplayOff(void);
extern void LCD_DisplayOn(void);
extern void LCD_DisplayInvert(byte invert);
extern void LCD_UpdateBacklightDutyCycle(int dutyCycle);
extern void LCD_UpdateBacklightPwm(void);
extern void LCD_BacklightInit(uint32_t addr);
extern void LCD_DisplayFramebuffer(void);
extern void LCD_SetPanelOfsets(LCD_addr_t ofsX, LCD_addr_t ofsY);
extern void LCD_SetRotation(byte rotation);
extern void LCD_SetFont(byte index);
extern void LCD_SetFontColors(LCD_color_t foregroundColor, LCD_color_t backgroundColor);
extern void LCD_FastSetFontColorsFromIndex(LCD_color_t foregroundColorIndex, LCD_color_t backgroundColorIndex);
extern void LCD_SetFontForegroundColor(LCD_color_t foregroundColor);
extern void LCD_SetFontBackgroundColor(LCD_color_t backgroundColor);

extern uint16_t LCD_JustifyCharX(LCD_addr_t x, char chr, byte justX);
extern uint16_t LCD_JustifyStringX(LCD_addr_t x, char *s, byte justX);
extern uint16_t LCD_JustifyCharY(LCD_addr_t y, char chr, byte justY);
extern uint16_t LCD_JustifyStringY(LCD_addr_t y, byte justY);

extern void LCD_DrawChar(LCD_addr_t x, LCD_addr_t y, char chr);
extern void LCD_DrawCharJust(LCD_addr_t x, LCD_addr_t y, char chr, byte justX);
extern void LCD_DrawString(LCD_addr_t x, LCD_addr_t y, char *s);
extern void LCD_DrawStringJust(LCD_addr_t x, LCD_addr_t y, char *s, byte justX);

extern void LCD_DrawPixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color);
extern void LCD_DrawHLine(LCD_addr_t x, LCD_addr_t y, uint16_t length, uint16_t lineWidth, LCD_color_t color);
extern void LCD_DrawVLine(LCD_addr_t x, LCD_addr_t y, uint16_t length, uint16_t lineWidth, LCD_color_t color);
extern void LCD_DrawLine(LCD_addr_t x0, LCD_addr_t y0, LCD_addr_t x1, LCD_addr_t y1, uint16_t lineWidth, LCD_color_t color);

extern void LCD_DrawRect(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, uint16_t lineWidth, LCD_color_t edgeColor);
extern void LCD_DrawCircle(uint16_t centerX, uint16_t centerY, uint16_t radius, uint16_t lineWidth, LCD_color_t edgeColor);

extern void LCD_FillString(LCD_addr_t x, LCD_addr_t y, char *s, LCD_color_t fillColor);
extern void LCD_FillStringJust(LCD_addr_t x, LCD_addr_t y, char *s, LCD_color_t fillColor, byte justX);
extern void LCD_FillRect(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, LCD_color_t fillColor);
extern void LCD_FillCircle(uint16_t centerX, uint16_t centerY, uint16_t radius, LCD_color_t fillColor);

extern void LCD_DrawFilledRect(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, uint16_t lineWidth, LCD_color_t edgeColor, LCD_color_t fillColor);
extern void LCD_DrawFilledCircle(uint16_t centerX, uint16_t centerY, uint16_t radius, uint16_t lineWidth, LCD_color_t edgeColor, LCD_color_t fillColor);

extern void LCD_DrawBitMap(LCD_addr_t x, LCD_addr_t y, uint8_t *bmpPtr);
extern void LCD_DrawRGBImage(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, LCD_color_t *dataPtr);
extern void LCD_DrawRGBImageChromaKey(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, LCD_color_t *dataPtr, LCD_color_t chromaKey);

extern void LCD_FillDisplay(LCD_color_t fillColor);

extern byte LCD_GetDisplayIndex(void);
extern uint16_t LCD_GetDisplayWidth(void);
extern uint16_t LCD_GetDisplayHeight(void);
extern uint16_t LCD_GetDisplayNarrowDimension(void);
extern byte LCD_GetRotation(void);
extern byte LCD_GetFont(void);
extern uint16_t LCD_GetFontWidth(void);
extern uint16_t LCD_GetFontHeight(void);
extern uint16_t LCD_GetCharWidth(char chr);
extern uint16_t LCD_GetStringWidth(char *s);
extern char *LCD_PadAndJustifyStringToSize(char str[], byte just, int8_t finalWidth);
extern LCD_color_t LCD_GetColorFromIndex(byte index);
extern void LCD_SetColorIndexData(byte index, LCD_color_t color);
extern LCD_color_t LCD_GetChromaKey(void);
extern void LCD_SetChromaKey(LCD_color_t chromaKey);

///////////////////////////////////////////////////////////////////////////////////

#define LINE_H	16
#define YLINE1	(LINE_H * 0)
#define YLINE2	(LINE_H * 1)
#define YLINE3	(LINE_H * 2)
#define YLINE4	(LINE_H * 3)

#define CHAR_WIDTH	7
#define XCHAR1		(0 * CHAR_WIDTH)
#define XCHAR2		(1 * CHAR_WIDTH)
#define XCHAR3		(2 * CHAR_WIDTH)
#define XCHAR4		(3 * CHAR_WIDTH)
#define XCHAR5		(4 * CHAR_WIDTH)
#define XCHAR6		(5 * CHAR_WIDTH)
#define XCHAR7		(6 * CHAR_WIDTH)
#define XCHAR8		(7 * CHAR_WIDTH)
#define XCHAR9		(8 * CHAR_WIDTH)
#define XCHAR10		(9 * CHAR_WIDTH)
#define XCHAR11		(10 * CHAR_WIDTH)
#define XCHAR12		(11 * CHAR_WIDTH)
#define XCHAR13		(12 * CHAR_WIDTH)
#define XCHAR14		(13 * CHAR_WIDTH)
#define XCHAR15		(14 * CHAR_WIDTH)
#define XCHAR16		(15 * CHAR_WIDTH)
#define XCHAR17		(16 * CHAR_WIDTH)
#define XCHAR18		(17 * CHAR_WIDTH)
#define XCHAR19		(18 * CHAR_WIDTH)

////////////////////////////////////////////////////////////////////////////////
#endif // #ifndef lcd_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE

#endif //ADD_ON_SPI_DISPLAY
