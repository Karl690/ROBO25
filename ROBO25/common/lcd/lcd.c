#ifdef ADD_ON_SPI_DISPLAY

////////////////////////////////////////////////////////////////////////////////
//
// File:    lcd.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: spi display / font related functions
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "fonts.h"
#include "math.h"

//#define M_SQRT1_2   0.70710678118654752440  // from math.h
//extern float roundf _PARAMS((float));       // from math.h

#define LCD_SPI_SPIN_UNTIL_TX_READY        LCD_spinUntilSpiTxIsReady()
#define LCD_SPI_SPIN_UNTIL_RX_AVAIL        LCD_spinUntilSpiRxIsAvailable()

#define LCD_SPI_DUMMY_TX            {LCD_SPIx->DR = 0;}
#define LCD_SPI_DUMMY_RX            {(void)LCD_SPIx->DR;}

extern LCD_driverStruct _ili9341;
extern LCD_driverStruct _st7735;
extern LCD_driverStruct _ssd1306;

SPI_TypeDef         *LCD_SPIx;                      // SPI handle

LCD_fontStruct      _LCD_fontStruct;                // storage for current font info
LCD_fontStruct      *_font = &_LCD_fontStruct;      // ptr to current font struct

LCD_panelStruct     _LCD_panelStruct;               // storage for LCD's panel info
LCD_backlightStruct	_LCD_backlightStruct;
LCD_struct          _LCD = {NULL, &_LCD_panelStruct, &_LCD_backlightStruct};

//WARNING: do not change the order .. hardcoded array order has to match index value
// FIRST 8 entries match microsoft's order (see Excel)
const LCD_color_t _lcdFixedColorIndexData[] = {
		LCD_COLOR_BLACK,			//LCD_COLOR_INDEX_BLACK   0x00
		LCD_COLOR_WHITE,			//LCD_COLOR_INDEX_WHITE   0x01
		LCD_COLOR_RED,				//LCD_COLOR_INDEX_RED   0x02
		LCD_COLOR_GREEN,			//LCD_COLOR_INDEX_GREEN   0x03
		LCD_COLOR_BLUE,				//LCD_COLOR_INDEX_BLUE   0x04
		LCD_COLOR_YELLOW,			//LCD_COLOR_INDEX_YELLOW   0x05
		LCD_COLOR_MAGENTA,			//LCD_COLOR_INDEX_MAGENTA   0x06
		LCD_COLOR_CYAN,				//LCD_COLOR_INDEX_CYAN   0x07
		LCD_COLOR_BROWN,			//LCD_COLOR_INDEX_BROWN   0x08
		LCD_COLOR_ORANGE,			//LCD_COLOR_INDEX_ORANGE   0x09
		LCD_COLOR_PURPLE,			//LCD_COLOR_INDEX_PURPLE   0xA
		LCD_COLOR_YELLOW_GREEN,		//LCD_COLOR_INDEX_YELLOW_GREEN   0xB
		LCD_COLOR_PINK,				//LCD_COLOR_INDEX_PINK   0xC
		LCD_COLOR_OLIVE,			//LCD_COLOR_INDEX_OLIVE   0xD
		LCD_COLOR_GRAY,				//LCD_COLOR_INDEX_GRAY   0xE
		LCD_COLOR_TRANSPARENT,		//LCD_COLOR_INDEX_TRANSPARENT   0xF
};

LCD_color_t _lcdVariableColorIndexData[16];

////////////////////////////////////////////////////////////////////////////////

#ifndef SWAP_UINT16
#define SWAP_UINT16(a, b)                                                      \
  {                                                                            \
	uint16_t t = a;                                                            \
	a = b;                                                                     \
	b = t;                                                                     \
  }
#endif //!SWAP_UINT16

////////////////////////////////////////////////////////////////////////////////

#define LCD_SPI_WATCHDOG_INIT_VALUE 1000
int _lcsSpiFailureCounter = 0;
boolean _lcdSpiIsDead = FALSE;

void LCD_markSpiAsDead(void)
{
	_lcsSpiFailureCounter++;
	if (_lcsSpiFailureCounter == 1000)
	{	// report once
#ifdef COMPILE_FOR_DEVICE
		// uncomment if you want this reported
		//reportErrorNoParams(STICKY_SPI_ERROR, ERROR_UNIT_DEVICE, ERROR_SPI_LCD_ERROR);
#endif //COMPILE_FOR_DEVICE
		_lcdSpiIsDead = TRUE;	// stop trying to access spi
	}
}

////////////////////////////////////////////////////////////////////////////////

void LCD_spinUntilSpiTxIsReady(void)
{
	if (_lcdSpiIsDead) return;
	//ENC_SPI_SPIN_UNTIL_SPI_READY
	int i;
	for (i=0; i<LCD_SPI_WATCHDOG_INIT_VALUE; i++)
	{
		if ((LCD_SPIx != SPI1) && (LCD_SPIx != SPI2) && (LCD_SPIx != SPI3))
			break;	// bad spi pointer
		if ((LCD_SPIx->CR1 & SPI_CR1_SPE) == 0)
			break; //spi is disabled
		if ((LCD_SPIx->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) == SPI_SR_TXE)
			return; //spi is now ready
	}
	LCD_markSpiAsDead(); // if we get here, then something bad happened or the watchdog countdown expired and the transfer did not occur
}

////////////////////////////////////////////////////////////////////////////////

void LCD_spiTx(uint16_t value)
{
	LCD_spinUntilSpiTxIsReady();		// ENC_SPI_SPIN_UNTIL_SPI_READY;
	LCD_SPIx->DR = value;				// SPI_I2S_SendData(LCD_SPIx, value);
}

////////////////////////////////////////////////////////////////////////////////

void LCD_spinUntilSpiRxIsAvailable(void)
{
	if (_lcdSpiIsDead) return;
	//ENC_SPI_SPIN_UNTIL_RX_AVAIL
	int i;
	for (i=0; i<LCD_SPI_WATCHDOG_INIT_VALUE; i++)
	{
		if ((LCD_SPIx != SPI1) && (LCD_SPIx != SPI2) && (LCD_SPIx != SPI3))
			break;	// bad spi pointer
		if ((LCD_SPIx->CR1 & SPI_CR1_SPE) == 0)
			break; //spi is disabled
		if ((LCD_SPIx->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) == (SPI_SR_TXE | SPI_SR_RXNE))
			return; //spi is now ready
	}
	LCD_markSpiAsDead(); // if we get here, then something bad happened or the watchdog countdown expired and the transfer did not occur
}

////////////////////////////////////////////////////////////////////////////////

uint16_t LCD_spiRx(void)
{
	LCD_spinUntilSpiRxIsAvailable();		// ENC_SPI_SPIN_UNTIL_RX_AVAIL;
	return(LCD_SPIx->DR);					// return(SPI_I2S_ReceiveData(LCD_SPIx));
}

////////////////////////////////////////////////////////////////////////////////

void LCD_SPI_Write8(byte wdata)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPIx->DR = wdata;
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	LCD_SPI_DUMMY_RX;                       // clear RNXE flag
}

void LCD_SPI_WriteRepeat8(byte wdata, int numCopies)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;
	for (i=0; i<numCopies; i++)
	{
		LCD_SPIx->DR = wdata;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		LCD_SPI_DUMMY_RX;                   // clear RNXE flag
	}
}

void LCD_SPI_WriteArray8(byte *wdata, int numBytes)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;
	for (i=0; i<numBytes; i++)
	{
		LCD_SPIx->DR = *wdata++;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		LCD_SPI_DUMMY_RX;                   // clear RNXE flag
	}
}

void LCD_SPI_WriteReg8(uint8_t reg)
{
	if (LCD_SPIx == NULL) return;
	LCD_CMD_ENABLE;                         // change Data/-Cmd line
	LCD_SPI_Write8(reg);
	LCD_DATA_ENABLE;                        // always return to data mode
}

void LCD_SPI_DataSize16(void)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPIx->CR1 |= SPI_CR1_DFF;           // change data size to 16
}
void LCD_SPI_DataSize8(void)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPIx->CR1 &= ~SPI_CR1_DFF;          // change data size to 16
}

void LCD_SPI_Write16(uint16_t wdata)
{   // do point is changing to 16-bit SPI mode for this (not worth the overhead), so stay in 8-bit mode
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPIx->DR = wdata >> 8;
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	LCD_SPI_DUMMY_RX;                       // clear RNXE flag
	LCD_SPIx->DR = wdata;
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	LCD_SPI_DUMMY_RX;                       // clear RNXE flag
}

void LCD_SPI_WriteRepeat16(uint16_t wdata, int numCopies)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;

	if (_LCD.panel->supports16BitSpi)
	{
		LCD_SPI_DataSize16();
		for (i=0; i<numCopies; i++)
		{
			LCD_SPI_SPIN_UNTIL_TX_READY;                                           // make sure any prior transmission in complete
			LCD_SPIx->DR = wdata; //((wdata << 8) & 0xff) | ((wdata >> 8) & 0xff);  // wdata;
			LCD_SPI_SPIN_UNTIL_RX_AVAIL;                                            // make sure prior transaction is complete before proceeding
			LCD_SPI_DUMMY_RX;                                                       // clear RNXE flag
		}
		LCD_SPI_DataSize8();                                                        // return to default of 8 bit
	}
	else
	{
		for (i=0; i<numCopies; i++)
		{
			LCD_SPIx->DR = wdata >> 8;
			LCD_SPI_SPIN_UNTIL_RX_AVAIL;    // make sure prior transaction is complete before proceeding
			LCD_SPI_DUMMY_RX;               // clear RNXE flag
			LCD_SPIx->DR = wdata;
			LCD_SPI_SPIN_UNTIL_RX_AVAIL;    // make sure prior transaction is complete before proceeding
			LCD_SPI_DUMMY_RX;               // clear RNXE flag
		}
	}
}

void LCD_SPI_WriteArray16(uint16_t *wdata, int numWords)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;

	if (_LCD.panel->supports16BitSpi)
	{
		LCD_SPI_DataSize16();
		for (i=0; i<numWords; i++)
		{
			LCD_SPIx->DR = *wdata++;
			LCD_SPI_SPIN_UNTIL_RX_AVAIL;    // make sure prior transaction is complete before proceeding
			LCD_SPI_DUMMY_RX;               // clear RNXE flag
		}
		LCD_SPI_DataSize8();
	}
	else
	{
		for (i=0; i<numWords; i++)
		{
			LCD_SPIx->DR = (*wdata) >> 8;
			LCD_SPI_SPIN_UNTIL_RX_AVAIL;    // make sure prior transaction is complete before proceeding
			LCD_SPI_DUMMY_RX;               // clear RNXE flag
			LCD_SPIx->DR = *wdata++;
			LCD_SPI_SPIN_UNTIL_RX_AVAIL;    // make sure prior transaction is complete before proceeding
			LCD_SPI_DUMMY_RX;               // clear RNXE flag
		}
	}
}

byte LCD_SPI_Read8(void)
{
	if (LCD_SPIx == NULL) return(0);
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPI_DUMMY_TX;                       // load dummy data ... need to push out data to get data back
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	return(LCD_SPIx->DR);
}

void LCD_SPI_ReadArray8(byte *rdata, int numBytes)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;
	for (i=0; i<numBytes; i++)
	{
		LCD_SPI_DUMMY_TX;                   // load dummy data ... need to push out data to get data back
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		*rdata++ = LCD_SPIx->DR;
	}
}

uint16_t LCD_SPI_Read16(void)
{
	if (LCD_SPIx == NULL) return(0);
	uint16_t tmp16;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete

	LCD_SPI_DUMMY_TX;                       // load dummy data ... need to push out data to get data back
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	tmp16 = (LCD_SPIx->DR) << 8;            // grab upper byte

	LCD_SPI_DUMMY_TX;                       // load dummy data ... need to push out data to get data back
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	return(tmp16 | LCD_SPIx->DR);
}

void LCD_SPI_ReadArray16(uint16_t *rdata, int numWords)
{
	if (LCD_SPIx == NULL) return;
	uint16_t tmp16;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;
	for (i=0; i<numWords; i++)
	{
		LCD_SPI_DUMMY_TX;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		tmp16 = (LCD_SPIx->DR) << 8;        // grab upper byte

		LCD_SPI_DUMMY_TX;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		*rdata++ = tmp16 | LCD_SPIx->DR;
	}
}

byte LCD_SPI_WriteRead8(byte wdata)
{
	if (LCD_SPIx == NULL) return(0);
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPIx->DR = wdata;
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	return LCD_SPIx->DR;
}

void LCD_SPI_WriteReadArray8(byte *wdata, byte *rdata, int numBytes)
{
	if (LCD_SPIx == NULL) return;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;
	for (i=0; i<numBytes; i++)
	{
		LCD_SPIx->DR = *wdata++;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		*rdata++ = LCD_SPIx->DR;
	}
}

uint16_t LCD_SPI_WriteRead16(uint16_t wdata)
{
	if (LCD_SPIx == NULL) return(0);
	uint16_t tmp16;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	LCD_SPIx->DR = wdata >> 8;
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	tmp16 = LCD_SPIx->DR;                   // grab upper byte
	LCD_SPIx->DR = wdata;                   // load dummy data ... need to push out data to get data back
	tmp16 <<= 8;
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;            // make sure prior transaction is complete before proceeding
	return(tmp16 | LCD_SPIx->DR);
}

void LCD_SPI_WriteReadArray16(uint16_t *wdata, uint16_t *rdata, int numWords)
{
	if (LCD_SPIx == NULL) return;
	uint16_t tmp16;
	LCD_SPI_SPIN_UNTIL_TX_READY;           // make sure any prior transmission in complete
	int i;
	for (i=0; i<numWords; i++)
	{
		LCD_SPIx->DR = (*wdata) >> 8;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		tmp16 = (LCD_SPIx->DR) << 8;        // grab upper byte

		LCD_SPIx->DR = *wdata++;
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		*rdata++ = tmp16 | LCD_SPIx->DR;
	}
}

uint32_t LCD_SPI_ReadRegAddr32(byte reg, int numBytes)
{
	if (LCD_SPIx == NULL) return(0);
	LCD_SPI_SPIN_UNTIL_TX_READY;   // make sure any prior transmission in complete

	LCD_SPI_WriteReg8(reg);
	LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding

	uint32_t tmp32=0;
	int i;
	for (i=0; i<numBytes; i++)
	{
		LCD_SPI_DUMMY_TX;                   // load dummy data ... need to push out data to get data back
		LCD_SPI_SPIN_UNTIL_RX_AVAIL;        // make sure prior transaction is complete before proceeding
		tmp32 = (tmp32 << 8) | LCD_SPIx->DR;    // grab next byte
	}
	return(tmp32);
}

void LCD_SendRegDataSequence(byte seq[], uint16_t numBytes, byte delayReg)
{
	if (LCD_SPIx == NULL) return;
	int i=0;
	while (i < numBytes)
	{
		byte reg = seq[i++];
		int numArgs = seq[i++];

		if ((reg == delayReg) && (numArgs != 0))
		{   // use NOP and if has arg, then thats the delay in ms'
			delayMsec(seq[i++]);
		}
		else
		{   // real command
			LCD_SPI_WriteReg8(reg);
			LCD_SPI_WriteArray8(&seq[i], numArgs);
			i += numArgs;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////


void LCD_SPI_Init(void)
{
	if (LCD_SPIx == NULL) return;
	// regular i/o's automatically set up via gpioInit routines
	if (LCD_SPIx == SPI1) {
		initClkAndResetAPB2(RCC_APB2Periph_SPI1);
	} else if (LCD_SPIx == SPI2) {
		initClkAndResetAPB1(RCC_APB1Periph_SPI2);
	} else if (LCD_SPIx == SPI3) {
		initClkAndResetAPB1(RCC_APB1Periph_SPI3);
	} else {
		return;
	}

	SPI_InitTypeDef SPI_InitStruct;
	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;

	int preSCale;
	if (LCD_SPIx == SPI1)
		preSCale = 84000000 / umax(1,_LCD.panel->maxSpiFrequency);  // imax 1 to prevent 0 divide
	else
		preSCale = 42000000 / umax(1,_LCD.panel->maxSpiFrequency);  // imax 1 prevent 0 divide

	if (preSCale <= 2)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	else if (preSCale <= 4)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	else if (preSCale <= 8)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	else if (preSCale <= 16)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	else if (preSCale <= 32)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	else if (preSCale <= 64)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	else if (preSCale <= 128)
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	else //slow down as much as possible
		SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;

	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;

	SPI_Init(LCD_SPIx, &SPI_InitStruct);
	SPI_Cmd(LCD_SPIx, ENABLE);

	LCD_RST_DISABLE;    // make sure reset is off
	LCD_CS_ENABLE;      // turn on and leave on (not sharing the spi bus)
}

////////////////////////////////////////////////////////////////////////////////

boolean LCD_Init(byte panelIndex, SPI_TypeDef *SPIx, uint32_t backlightBBpinAddr)
{	// returns TRUE is panel was legal/supported
	LCD_SPIx = SPIx;

	switch(panelIndex)
	{
	case LCD_TFT_MCUDEV_TFT_180:                // SS7735(s) 128x160 w/ 2,2 offset
	case LCD_TFT_MCUDEV_TFT_144:                // SS7735(s) 128x128 w/ screwy offsets
		_LCD.driver = &_st7735;
		break;
#ifndef BASIC_GUI_ONLY
	case LCD_TFT_DEVEBOX_SPI_TFT_280:           // ILI9341   240x320 no offsets
		_LCD.driver = &_ili9341;
		break;
#if 0	// NOT READY FOR PRIME TIME
	case LCD_TFT_MCUDEV_TFT_096:				//SSD1306 0.96" 128x64 mono oled display
		_LCD.driver = &_ssd1306;
		break;
	case LCD_TFT_MCUDEV_TFT_096_BB:				//SSD1306 0.96" 128x64 mono oled display bit-bang I/I (ie, BTT103 board)
		LCD_SPIx = NULL;
		_LCD.driver = &_ssd1306_BB;
		break;
#endif //0
#endif //!BASIC_GUI_ONLY

	default:
		return(FALSE);
		break;
	}

	LCD_InitPanelStruct(_LCD.panel, panelIndex);      //fill in panel struct -- MUST BE DONE BEFORE SETTING UP SPI

	if (_LCD.panel->hasMISO)
		pinInit(PIN_LCD_MISO);
	if (_LCD.panel->hasResetPin)
		pinInit(PIN_LCD_RST);

	LCD_BacklightInit(backlightBBpinAddr);
	LCD_SetChromaKey(LCD_COLOR_TRANSPARENT);
	LCD_SPI_Init();                             // init spi and set IO states
	LCD_InitDriver();                           // do panel/driver specific init (registers, etc)
	LCD_FillDisplay(LCD_COLOR_BLACK);           // start fresh
	LCD_DisplayOn();
	return(TRUE);
}

void LCD_InitPanelStruct(LCD_panelStruct *panelPtr, byte index)
{
	_LCD.driver->InitPanelStruct(_LCD.panel, index);      //fill in panel struct -- MUST BE DONE BEFORE SETTING UP SPI
}

void LCD_InitDriver(void)
{
	_LCD.driver->InitDriver();
}
void LCD_DisplayOff(void)
{
	_LCD.driver->DisplayOff();
}

void LCD_DisplayOn(void)
{
	_LCD.driver->DisplayOn();
}

void LCD_DisplayInvert(byte invert)
{
	_LCD.driver->DisplayInvert((invert == 0) ? 0 : 1);
}

void LCD_UpdateBacklightDutyCycle(int dutyCycle)
{
	_LCD.backlight->dutyCycle = dutyCycle;
}

void LCD_UpdateBacklightPwm(void)
{
	if (_LCD.backlight->BBpinAddr)
	{
		//NUKE_LCD.backlight->pwmCnt += _LCD.backlight->resolution;	// N% jumps to control update rate (tradeoff of speed vs resolution
		_LCD.backlight->pwmCnt += 5;		//NUKE_LCD.backlight->resolution;	// N% jumps to control update rate (tradeoff of speed vs resolution
		if (_LCD.backlight->pwmCnt >= 100)
			_LCD.backlight->pwmCnt = 0;
		MEM_ADDR(_LCD.backlight->BBpinAddr) = (_LCD.backlight->dutyCycle > _LCD.backlight->pwmCnt); // simple compare to determine the state
	}
}

void LCD_BacklightInit(uint32_t addr)
{
	_LCD.backlight->BBpinAddr = addr;
	_LCD.backlight->dutyCycle = 100;
	_LCD.backlight->pwmCnt = 0;
	LCD_UpdateBacklightPwm();
	//NUKE _LCD.backlight->usePulseTrain = TRUE;
	//NUKE _LCD.backlight->resolution = 5;	// 5% increments
}

void LCD_DisplayFramebuffer(void)
{
	_LCD.driver->DisplayFramebuffer();
}

void LCD_SetPanelOfsets(LCD_addr_t ofsX, LCD_addr_t ofsY)
{
	_LCD.panel->ofsX = ofsX;
	_LCD.panel->ofsY = ofsY;
}
void LCD_SetRotation(byte rotation)
{
	_LCD.driver->SetRotation(rotation);
}

void LCD_SetFont(byte index)
{
	sFONT *sFontPtr;
	_font->index = index;   // save index
	switch (index)
	{
#ifndef BASIC_GUI_ONLY
	case LCD_FONT_5x8:      sFontPtr = &Font8;  break; //     adds 776 byes of flash
	case LCD_FONT_11x16:    sFontPtr = &Font16; break;
	case LCD_FONT_13x20:    sFontPtr = &Font20; break;
	case LCD_FONT_16x22:    sFontPtr = &Font22; break;
	//case LCD_FONT_17x24:    sFontPtr = &Font24; break;	//REMOVED... too "empty"
#endif //!BASIC_GUI_ONLY
	case LCD_FONT_7x12:     //sFontPtr = &Font12; break;
	default:                sFontPtr = &Font12;  break;
	}
	_font->dataPtr      = (byte *)sFontPtr->table;
	_font->firstChar    = ' ';  // currently all fonts start with a blank char (0x20, 32)
	_font->width        = sFontPtr->Width;  //getFontWidth(font);
	_font->height       = sFontPtr->Height; //getFontHeight(font);
	_font->bytesWide    = (_font->width + 7) / 8;   //getFontBytesWide(font);
	_font->bytesPerChar = _font->bytesWide * _font->height; // used often, so pre-mult
}

void LCD_SetFontColors(LCD_color_t foregroundColor, LCD_color_t backgroundColor)
{
	_font->foregroundColor = foregroundColor;
	_font->backgroundColor = backgroundColor;
	LCD_SetColorIndexData(LCD_COLOR_INDEX_FONT_FOREGROUND, foregroundColor);
	LCD_SetColorIndexData(LCD_COLOR_INDEX_FONT_BACKGROUND, backgroundColor);
}

void LCD_FastSetFontColorsFromIndex(LCD_color_t foregroundColorIndex, LCD_color_t backgroundColorIndex)
{
	_font->foregroundColor = LCD_GetColorFromIndex(foregroundColorIndex);
	_font->backgroundColor = LCD_GetColorFromIndex(backgroundColorIndex);
}

void LCD_SetFontForegroundColor(LCD_color_t foregroundColor)
{
	_font->foregroundColor = foregroundColor;
}

void LCD_SetFontBackgroundColor(LCD_color_t backgroundColor)
{
	_font->backgroundColor = backgroundColor;
}

uint16_t LCD_JustifyCharX(LCD_addr_t x, char chr, byte justX)
{
	if (justX == LCD_JUSTIFICATION_LEFT)
		return(x);
	else if (justX == LCD_JUSTIFICATION_CENTER)
		return (x - (LCD_GetCharWidth(chr) / 2));
	else if (justX == LCD_JUSTIFICATION_RIGHT)
		return (x - (LCD_GetCharWidth(chr) + 1));
	return(x);
}

uint16_t LCD_JustifyStringX(LCD_addr_t x, char *s, byte justX)
{
	if (justX == LCD_JUSTIFICATION_LEFT)
		return(x);
	else if (justX == LCD_JUSTIFICATION_CENTER)
		return (x - (LCD_GetStringWidth(s) / 2));
	else if (justX == LCD_JUSTIFICATION_RIGHT)
		return (x - (LCD_GetStringWidth(s) + 1));
	return(x);
}

uint16_t LCD_JustifyCharY(LCD_addr_t y, char chr, byte justY)
{
	if (justY == LCD_JUSTIFICATION_TOP)
		return(y);
	else if (justY == LCD_JUSTIFICATION_CENTER)
		return (y - (_font->height / 2));
	else if (justY == LCD_JUSTIFICATION_BOTTOM)
		return (y - (_font->height + 1));
	return(y);
}

uint16_t LCD_JustifyStringY(LCD_addr_t y, byte justY)
{
	if (justY == LCD_JUSTIFICATION_TOP)
		return(y);
	else if (justY == LCD_JUSTIFICATION_CENTER)
		return (y - (_font->height / 2));
	else if (justY == LCD_JUSTIFICATION_BOTTOM)
		return (y - (_font->height + 1));
	return(y);
}

void LCD_DrawChar(LCD_addr_t x, LCD_addr_t y, char chr)
{
	// uses current defined font

	int i, j;
	uint32_t charBits;
	boolean fullyOpaque = (_font->foregroundColor != _LCD.chromaKey) && (_font->backgroundColor != _LCD.chromaKey);

	if (((x + _font->width) <= _LCD.panel->width) && ((y + _font->height) <= _LCD.panel->height))
	{   // fits on display
		_LCD.driver->SetDisplayWindow(x, y, _font->width, _font->height);

		byte *chrDataPtr = _font->dataPtr + ((chr - _font->firstChar) * _font->bytesPerChar);

		for (j=0; j<_font->height; j++)
		{   // row at a time to match addressing to lcd
			charBits = 0;
			for (i=0; i<_font->bytesWide; i++)
			{   // pack bytes into single word, avoid any alignment issues
				charBits = (charBits << 8) | *chrDataPtr++;
			}
			for (i=((8*_font->bytesWide)-1); i>=((8*_font->bytesWide)-_font->width); i--)
			{
				if (fullyOpaque && (LCD_SPIx != NULL))
				{   // optimized rectangular write
					LCD_SPI_Write16(((charBits >> i) & 1) ? _font->foregroundColor : _font->backgroundColor);
				}
				else
				{
					LCD_DrawPixel(x+((8*_font->bytesWide)-i), y+j, ((charBits >> i) & 1) ? _font->foregroundColor : _font->backgroundColor);
				}
			}
		}
	}
}

 void LCD_DrawCharJust(LCD_addr_t x, LCD_addr_t y, char chr, byte justX)
{
	 x = LCD_JustifyCharX(x, chr, justX);
	 LCD_DrawChar(x, y, chr);
}

void LCD_DrawString(LCD_addr_t x, LCD_addr_t y, char *s)
{
	// uses currently defined font

	int i, j, k, lineOfs;
	uint16_t numChars;
	uint32_t charBits;
	boolean fullyOpaque = (_font->foregroundColor != _LCD.chromaKey) && (_font->backgroundColor != _LCD.chromaKey);

	numChars = uFitWithinRange((_LCD.panel->width - x) / _font->width, 0, strlen(s));   // make sure it fits

	if (numChars && ((y + _font->height) <= _LCD.panel->height))
	{   // fits vertically on display (numCHanr clipped to fit horizontally)
		_LCD.driver->SetDisplayWindow(x, y, _font->width * numChars, _font->height);

		for (j=0; j<_font->height; j++)
		{   // row at a time to match addressing to lcd
			lineOfs = j * _font->bytesWide;
			for (k=0; k<numChars; k++)
			{   // cycle through one line of each char

				byte *chrDataPtr = _font->dataPtr + ((s[k] - _font->firstChar) * _font->bytesPerChar) + lineOfs;
				charBits = 0;
				for (i=0; i<_font->bytesWide; i++)
				{   // pack bytes into single word, avoid any alignment issues
					charBits = (charBits << 8) | *chrDataPtr++;
				}
				for (i=((8*_font->bytesWide)-1); i>=((8*_font->bytesWide)-_font->width); i--)
				{
					if (fullyOpaque && (LCD_SPIx != NULL))
					{   // optimized rectangular write
						LCD_SPI_Write16(((charBits >> i) & 1) ? _font->foregroundColor : _font->backgroundColor);
					}
					else
					{
						LCD_DrawPixel(x+(k*_font->width)+((8*_font->bytesWide)-i), y+j, ((charBits >> i) & 1) ? _font->foregroundColor : _font->backgroundColor);
					}
				}
			}
		}
	}
}

void LCD_DrawStringJust(LCD_addr_t x, LCD_addr_t y, char *s, byte justX)
{
	x = LCD_JustifyStringX(x, s, justX);
	LCD_DrawString(x, y, s);
}

void LCD_DrawPixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color)
{
	if (color == _LCD.chromaKey) return;

	_LCD.driver->WritePixel(x, y, color);
}

void LCD_DrawHLine(LCD_addr_t x, LCD_addr_t y, uint16_t length, uint16_t lineWidth, LCD_color_t color)
{
	LCD_FillRect(x, y, length, lineWidth, color);
}

void LCD_DrawVLine(LCD_addr_t x, LCD_addr_t y, uint16_t length, uint16_t lineWidth, LCD_color_t color)
{
	LCD_FillRect(x, y, lineWidth, length, color);
}

void LCD_DrawLine(LCD_addr_t x0, LCD_addr_t y0, LCD_addr_t x1, LCD_addr_t y1, uint16_t lineWidth, LCD_color_t color)
{   // from adafruit_GFX
	if (color == _LCD.chromaKey) return;

	if (x0 == x1)
	{
		if (y0 > y1)
			SWAP_UINT16(y0, y1);
		LCD_DrawVLine(x0, y0, y1 - y0 + 1, lineWidth, color);
	}
	else if (y0 == y1)
	{
		if (x0 > x1)
			SWAP_UINT16(x0, x1);
		LCD_DrawHLine(x0, y0, x1 - x0 + 1, lineWidth, color);
	}
	else
	{   // off axis line -- lineWidth not supported yet
		int16_t steep = abs(y1 - y0) > abs(x1 - x0);
		if (steep)
		{
			SWAP_UINT16(x0, y0);
			SWAP_UINT16(x1, y1);
		}

		if (x0 > x1)
		{
			SWAP_UINT16(x0, x1);
			SWAP_UINT16(y0, y1);
		}

		int16_t dx, dy;
		dx = x1 - x0;
		dy = abs(y1 - y0);

		int16_t err = dx / 2;
		int16_t ystep;

		if (y0 < y1)
		{
			ystep = 1;
		}
		else
		{
			ystep = -1;
		}
		for (; x0 <= x1; x0++)
		{
			if (steep)
			{
				LCD_DrawPixel(y0, x0, color);
			}
			else
			{
				LCD_DrawPixel(x0, y0, color);
			}
			err -= dy;
			if (err < 0)
			{
				y0 += ystep;
				err += dx;
			}
		}
	}
}

void LCD_DrawRect(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, uint16_t lineWidth, LCD_color_t edgeColor)
{
	LCD_DrawVLine(x, y, height, lineWidth, edgeColor);
	LCD_DrawVLine(x+width-lineWidth, y, height, lineWidth, edgeColor);
	LCD_DrawHLine(x+lineWidth, y, width-2*lineWidth, lineWidth, edgeColor);
	LCD_DrawHLine(x+lineWidth, y+height-lineWidth, width-2*lineWidth, lineWidth, edgeColor);
}

void LCD_DrawCircle(uint16_t centerX, uint16_t centerY, uint16_t radius, uint16_t lineWidth, LCD_color_t edgeColor)
{
	if (edgeColor == _LCD.chromaKey) return;

	int yi, xi, r, rSquared;
	float x, lastX;

	for (r=radius-(lineWidth-1); r<=radius; r++)
	{
		LCD_DrawPixel(centerX, centerY+r, edgeColor);
		LCD_DrawPixel(centerX, centerY-r, edgeColor);
		LCD_DrawPixel(centerX+r, centerY, edgeColor);
		LCD_DrawPixel(centerX-r, centerY, edgeColor);
		rSquared = r * r;
		lastX = -(float)r * M_SQRT1_2;
		for (x=-r; x<=-lastX; x+=0.8f)
		{
			yi = fpu_sqrtf(rSquared - (x * x)) + 0.5f;
			xi = x + 0.5f;
			LCD_DrawPixel(centerX+xi, centerY+yi, edgeColor);
			LCD_DrawPixel(centerX-xi, centerY+yi, edgeColor);
			LCD_DrawPixel(centerX+xi, centerY-yi, edgeColor);
			LCD_DrawPixel(centerX-xi, centerY-yi, edgeColor);
			LCD_DrawPixel(centerX+yi, centerY+xi, edgeColor);
			LCD_DrawPixel(centerX-yi, centerY+xi, edgeColor);
			LCD_DrawPixel(centerX+yi, centerY-xi, edgeColor);
			LCD_DrawPixel(centerX-yi, centerY-xi, edgeColor);
		}
	}
}

void LCD_FillString(LCD_addr_t x, LCD_addr_t y, char *s, LCD_color_t fillColor)
{
	LCD_FillRect(x, y, LCD_GetStringWidth(s), _font->height,  fillColor);
}

void LCD_FillStringJust(LCD_addr_t x, LCD_addr_t y, char *s, LCD_color_t fillColor, byte justX)
{
	x = LCD_JustifyStringX(x, s, justX);
	LCD_FillString(x, y, s, fillColor);
}

void LCD_FillRect(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, LCD_color_t fillColor)
{
	if (fillColor == _LCD.chromaKey) return;

	uint32_t pixels = _LCD.driver->SetDisplayWindow(x, y, width, height);
	LCD_SPI_WriteRepeat16(fillColor, pixels);
}

void LCD_FillCircle(uint16_t centerX, uint16_t centerY, uint16_t radius, LCD_color_t fillColor)
{
	if (fillColor == _LCD.chromaKey) return;

	int x, y, x0, x1, y0, len, radiusSquared;

	radiusSquared = radius * radius;
	for (x=-radius; x<=0; x++)
	{
		y = (int)fpu_sqrtf(radiusSquared - (x * x));
		x0 = centerX + x;
		x1 = centerX - x;
		y0 = centerY - y;
		len = 2 * y;
		LCD_DrawVLine(x0, y0, len, 1, fillColor);
		LCD_DrawVLine(x1, y0, len, 1, fillColor);
	}
}

void LCD_DrawFilledRect(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, uint16_t lineWidth, LCD_color_t edgeColor, LCD_color_t fillColor)
{
	LCD_FillRect(x+lineWidth, y+lineWidth, width-2*lineWidth, height-2*lineWidth, fillColor);
	LCD_DrawRect(x, y, width, height, lineWidth, edgeColor);
}

void LCD_DrawFilledCircle(uint16_t centerX, uint16_t centerY, uint16_t radius, uint16_t lineWidth, LCD_color_t edgeColor, LCD_color_t fillColor)
{
	LCD_FillCircle(centerX, centerY, radius-lineWidth, fillColor);
	LCD_DrawCircle(centerX, centerY, radius, lineWidth, edgeColor);
}

void LCD_DrawBitMap(LCD_addr_t x, LCD_addr_t y, uint8_t *bmpPtr)
{
	uint32_t filesize = *(volatile uint16_t *) (bmpPtr + 0x2);  // size is a 32-bit value @ ofs=0x2
	filesize |= (*(volatile uint16_t *) (bmpPtr + (0x2+2))) << 16;
	uint32_t index = *(volatile uint16_t *) (bmpPtr + 0xa);  // index to img data is a 32-bit value @ ofs=0xa
	index |= (*(volatile uint16_t *) (bmpPtr + (0xa+2))) << 16;
	uint32_t width = *(volatile uint16_t *) (bmpPtr + 0x12);  // width is a 32-bit value @ ofs=0x12
	width |= (*(volatile uint16_t *) (bmpPtr + (0x12+2))) << 16;
	int height = *(volatile uint16_t *) (bmpPtr + 0x16);  // height is a 32-bit value @ ofs=0x16
	height |= (*(volatile uint16_t *) (bmpPtr + (0x16+2))) << 16;
	//uint32_t pixels = (filesize - index) / 2; // assumes 16-bit color data
	bmpPtr += index;        // point to first word of color
	if (height < 0)
	{
		height *= -1;
#ifdef GB_HIDDEN_WARNINGS
		int LCD_DrawBitmap__flip_draw_order; //maybe temp flip of MY in LCD_MAC reg?
#endif
	}
	LCD_DrawRGBImage(x, y, width, height, (LCD_color_t *)bmpPtr); // does not clip image properly (tradeoff for speed)
}

void LCD_DrawRGBImage(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, LCD_color_t *dataPtr)
{
	boolean fitsOnTheDisplay = TRUE;
	fitsOnTheDisplay &= (x >= 0);
	fitsOnTheDisplay &= ((x + width) <= LCD_GetDisplayWidth());
	fitsOnTheDisplay &= (y >= 0);
	fitsOnTheDisplay &= ((y + height) <= LCD_GetDisplayHeight());
	if (fitsOnTheDisplay)
	{
		uint32_t pixels = _LCD.driver->SetDisplayWindow(x, y, width, height);
		LCD_SPI_WriteArray16((uint16_t *)dataPtr, pixels);
	}
	else
	{	// need to either clip data and send clipped line at a time OR send pixel by pixel which will auto clip
		for (int yofs=y; yofs<y+height; yofs++)
			for (int xofs=x; xofs<x+width; xofs++)
				LCD_DrawPixel(xofs, yofs, *dataPtr++);
	}
}

void LCD_DrawRGBImageChromaKey(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height, LCD_color_t *dataPtr, LCD_color_t chromaKey)
{	// only update pixels from image that do not match chromaKey
	LCD_color_t saveChromaKey = LCD_GetChromaKey();
	LCD_SetChromaKey(chromaKey);

	for (int yofs=y; yofs<y+height; yofs++)
		for (int xofs=x; xofs<x+width; xofs++)
			LCD_DrawPixel(xofs, yofs, *dataPtr++);

	LCD_SetChromaKey(saveChromaKey);
}

void LCD_FillDisplay(LCD_color_t fillColor)
{
	LCD_FillRect(0, 0, _LCD.panel->width, _LCD.panel->height, fillColor);
}

byte LCD_GetDisplayIndex(void)
{
	return(_LCD.panel->index);
}

uint16_t LCD_GetDisplayWidth(void)
{
	return(_LCD.panel->width);
}

uint16_t LCD_GetDisplayHeight(void)
{
	return(_LCD.panel->height);
}

uint16_t LCD_GetDisplayNarrowDimension(void)
{
	return(umin(_LCD.panel->width, _LCD.panel->height));
}

byte LCD_GetRotation(void)
{
	return(_LCD.panel->rotation);
}

byte LCD_GetFont(void)
{
	return(_font->index);
}

uint16_t LCD_GetFontWidth(void)
{
	return(_font->width);
}

uint16_t LCD_GetFontHeight(void)
{
	return(_font->height);
}

uint16_t LCD_GetCharWidth(char chr)
{
	return(_font->width);
}

uint16_t LCD_GetStringWidth(char *s)
{
	return(strlen(s) * _font->width);
}

char *LCD_PadAndJustifyStringToSize(char str[], byte just, int8_t finalWidth)
{	// rearrange str (in-place) to be exactly finalWidth wide and be justified as requested
	if (finalWidth == -1) return(str);	// leave string alone.

	int origWidth = strlen(str);
	int shift;

	if (finalWidth > origWidth)
	{	//need to pad
		switch (just)
		{
		case LCD_JUSTIFICATION_LEFT:
			memset(&str[origWidth], ' ', (finalWidth-origWidth)); // add trailing blanks
			break;
		case LCD_JUSTIFICATION_CENTER:
			shift = (finalWidth-origWidth)/2;
			memmove(&str[shift], &str[0], origWidth);	// shift str right
			memset(&str[0], ' ', shift); 				// back fill with blanks
			memset(&str[origWidth+shift], ' ', (finalWidth-(origWidth+shift))); // add trailing blanks
			break;
		case LCD_JUSTIFICATION_RIGHT:
			shift = finalWidth-origWidth;
			memmove(&str[shift], &str[0], origWidth);	// shift str right;
			memset(&str[0], ' ', shift); 				// back fill with blanks
			break;
		default:
			break;
		}
	}
	else if (origWidth > finalWidth)
	{	//need to truncate
		switch (just)
		{
		case LCD_JUSTIFICATION_CENTER:	//center string (clip both ends of str)
			shift = (origWidth-finalWidth)/2;	break;
		case LCD_JUSTIFICATION_RIGHT:	//clip beginning of str
			shift = origWidth-finalWidth;		break;
		case LCD_JUSTIFICATION_LEFT:	//clip end
		default:
			shift = 0;					break;
		}
		//NUKE for (i=0; i<finalWidth; i++) 	str[i] = str[shift+i];					// shift str left
		memcpy(&str[0], &str[shift], finalWidth);		// shift str left
	}
	str[finalWidth] = NULL_CHAR;	// terminate the string
	return(str);
}

LCD_color_t LCD_GetColorFromIndex(byte index)
{
	if (index & 0x80)
	{	// from variable data
		return(_lcdVariableColorIndexData[index & 0x7f]);
	}

	// from fixed data
	return(_lcdFixedColorIndexData[index]);
}

void LCD_SetColorIndexData(byte index, LCD_color_t color)
{
	if (index & 0x80)
	{	// variable index, so safe to load data
		_lcdVariableColorIndexData[index & 0x7f] = color;
	}
}

LCD_color_t LCD_GetChromaKey(void)
{
	return(_LCD.chromaKey);
}

void LCD_SetChromaKey(LCD_color_t chromaKey)
{
	_LCD.chromaKey = chromaKey;
}

#endif //ADD_ON_SPI_DISPLAY



