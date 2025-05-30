#ifdef ADD_ON_SPI_DISPLAY
/*
 ******************************************************************************
 * @file    ili9341.c
 * @author  MCD Application Team
 * @version V1.0.2
 * @date    02-December-2014
 * @brief   This file includes the LCD driver for ILI9341 LCD.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

// HEAVILY MODIFIED FOR Hyrel3D

////////////////////////////////////////////////////////////////////////////////
// includes
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "lcd.h"

////////////////////////////////////////////////////////////////////////////////
// local defines
////////////////////////////////////////////////////////////////////////////////

#define ILI9341_NOP                 0x00
#define ILI9341_SWRESET             0x01   /* Software Reset */
#define ILI9341_READ_DISPLAY_ID     0x04   /* Read display identification information */
#define ILI9341_RDDST               0x09   /* Read Display Status */
#define ILI9341_RDDPM               0x0A   /* Read Display Power Mode */
#define ILI9341_RDDMADCTL           0x0B   /* Read Display MADCTL */
#define ILI9341_RDDCOLMOD           0x0C   /* Read Display Pixel Format */
#define ILI9341_RDDIM               0x0D   /* Read Display Image Format */
#define ILI9341_RDDSM               0x0E   /* Read Display Signal Mode */
#define ILI9341_RDDSDR              0x0F   /* Read Display Self-Diagnostic Result */
#define ILI9341_SPLIN               0x10   /* Enter Sleep Mode */
#define ILI9341_SLEEP_OUT           0x11   /* Sleep out register */
#define ILI9341_PTLON               0x12   /* Partial Mode ON */
#define ILI9341_NORMAL_MODE_ON      0x13   /* Normal Display Mode ON */
#define ILI9341_DINVOFF             0x20   /* Display Inversion OFF */
#define ILI9341_DINVON              0x21   /* Display Inversion ON */
#define ILI9341_GAMMA               0x26   /* Gamma register */
#define ILI9341_DISPLAY_OFF         0x28   /* Display off register */
#define ILI9341_DISPLAY_ON          0x29   /* Display on register */
#define ILI9341_COLUMN_ADDR         0x2A   /* Colomn address register */
#define ILI9341_ROW_ADDR            0x2B   /* Row/Page address register */
#define ILI9341_PAGE_ADDR           0x2B   /* Row/Page address register */
#define ILI9341_GRAM                0x2C   /* GRAM register */
#define ILI9341_RGBSET              0x2D   /* Color SET */
#define ILI9341_RAMRD               0x2E   /* Memory Read */
#define ILI9341_PLTAR               0x30   /* Partial Area */
#define ILI9341_VSCRDEF             0x33   /* Vertical Scrolling Definition */
#define ILI9341_TEOFF               0x34   /* Tearing Effect Line OFF */
#define ILI9341_TEON                0x35   /* Tearing Effect Line ON */
#define ILI9341_MAC                 0x36   /* Memory Access Control register*/
#define ILI9341_VSCRSADD            0x37   /* Vertical Scrolling Start Address */
#define ILI9341_IDMOFF              0x38   /* Idle Mode OFF */
#define ILI9341_IDMON               0x39   /* Idle Mode ON */
#define ILI9341_PIXEL_FORMAT        0x3A   /* Pixel Format register */
#define ILI9341_WRITE_MEM_CONTINUE  0x3C   /* Write Memory Continue */
#define ILI9341_READ_MEM_CONTINUE   0x3E   /* Read Memory Continue */
#define ILI9341_SET_TEAR_SCANLINE   0x44   /* Set Tear Scanline */
#define ILI9341_GET_SCANLINE        0x45   /* Get Scanline */
#define ILI9341_WDB                 0x51   /* Write Brightness Display register */
#define ILI9341_RDDISBV             0x52   /* Read Display Brightness */
#define ILI9341_WCD                 0x53   /* Write Control Display register*/
#define ILI9341_RDCTRLD             0x54   /* Read CTRL Display */
#define ILI9341_WRCABC              0x55   /* Write Content Adaptive Brightness Control */
#define ILI9341_RDCABC              0x56   /* Read Content Adaptive Brightness Control */
#define ILI9341_WRITE_CABC          0x5E   /* Write CABC Minimum Brightness */
#define ILI9341_READ_CABC           0x5F   /* Read CABC Minimum Brightness */
#define ILI9341_RGB_INTERFACE       0xB0   /* RGB Interface Signal Control */
#define ILI9341_FRMCTR1             0xB1   /* Frame Rate Control (In Normal Mode) */
#define ILI9341_FRMCTR2             0xB2   /* Frame Rate Control (In Idle Mode) */
#define ILI9341_FRMCTR3             0xB3   /* Frame Rate Control (In Partial Mode) */
#define ILI9341_INV_CTRL            0xB4   /* Display Inversion Control */
#define ILI9341_BPC                 0xB5   /* Blanking Porch Control register */
#define ILI9341_DFC                 0xB6   /* Display Function Control register */
#define ILI9341_ETMOD               0xB7   /* Entry Mode Set */
#define ILI9341_BACKLIGHT1          0xB8   /* Backlight Control 1 */
#define ILI9341_BACKLIGHT2          0xB9   /* Backlight Control 2 */
#define ILI9341_BACKLIGHT3          0xBA   /* Backlight Control 3 */
#define ILI9341_BACKLIGHT4          0xBB   /* Backlight Control 4 */
#define ILI9341_BACKLIGHT5          0xBC   /* Backlight Control 5 */
#define ILI9341_BACKLIGHT7          0xBE   /* Backlight Control 7 */
#define ILI9341_BACKLIGHT8          0xBF   /* Backlight Control 8 */
#define ILI9341_POWER1              0xC0   /* Power Control 1 register */
#define ILI9341_POWER2              0xC1   /* Power Control 2 register */
#define ILI9341_VCOM1               0xC5   /* VCOM Control 1 register */
#define ILI9341_VCOM2               0xC7   /* VCOM Control 2 register */
#define ILI9341_POWERA              0xCB   /* Power control A register */
#define ILI9341_POWERB              0xCF   /* Power control B register */
#define ILI9341_NVMWR               0xD0   /* NV Memory Write */
#define ILI9341_NVMPKEY             0xD1   /* NV Memory Protection Key */
#define ILI9341_RDNVM               0xD2   /* NV Memory Status Read */
#define ILI9341_READ_ID4            0xD3   /* Read ID4 */
#define ILI9341_READ_ID1            0xDA   /* Read ID1 */
#define ILI9341_READ_ID2            0xDB   /* Read ID2 */
#define ILI9341_READ_ID3            0xDC   /* Read ID3 */
#define ILI9341_PGAMMA              0xE0   /* Positive Gamma Correction register */
#define ILI9341_NGAMMA              0xE1   /* Negative Gamma Correction register */
#define ILI9341_DGAMCTRL1           0xE2   /* Digital Gamma Control 1 */
#define ILI9341_DGAMCTRL2           0xE3   /* Digital Gamma Control 2 */
#define ILI9341_DTCA                0xE8   /* Driver timing control A */
#define ILI9341_DTCB                0xEA   /* Driver timing control B */
#define ILI9341_POWER_SEQ           0xED   /* Power on sequence register */
#define ILI9341_3GAMMA_EN           0xF2   /* 3 Gamma enable register */
#define ILI9341_INTERFACE           0xF6   /* Interface control register */
#define ILI9341_PRC                 0xF7   /* Pump ratio control register */

#define ILI9341_DELAY               ILI9341_NOP // overload no-op as key for inserting a delay in init seq

/* Timing configuration  (Typical configuration from ILI9341 datasheet)
HSYNC=10 (9+1)
HBP=20 (29-10+1)
ActiveW=240 (269-20-10+1)
HFP=10 (279-240-20-10+1)

VSYNC=2 (1+1)
VBP=2 (3-2+1)
ActiveH=320 (323-2-2+1)
VFP=4 (327-320-2-2+1)
 */
#define  ILI9341_HSYNC            ((uint32_t)9)   /* Horizontal synchronization */
#define  ILI9341_HBP              ((uint32_t)29)    /* Horizontal back porch      */
#define  ILI9341_HFP              ((uint32_t)2)    /* Horizontal front porch     */
#define  ILI9341_VSYNC            ((uint32_t)1)   /* Vertical synchronization   */
#define  ILI9341_VBP              ((uint32_t)3)    /* Vertical back porch        */
#define  ILI9341_VFP              ((uint32_t)2)    /* Vertical front porch       */

////////////////////////////////////////////////////////////////////////////////
// externally referenced
////////////////////////////////////////////////////////////////////////////////

extern void     LCD_SPI_Write8(byte wdata);
extern void     LCD_SPI_WriteArray8(byte *wdata, int numBytes);
extern void     LCD_SPI_Write16(uint16_t wdata);
extern void     LCD_SPI_WriteReg8(uint8_t reg);
extern void     LCD_SPI_ReadArray8(byte *rdata, int numBytes);
extern uint16_t LCD_SPI_Read16(void);
extern void     LCD_SendRegDataSequence(byte seq[], uint16_t numBytes, byte delayReg);

////////////////////////////////////////////////////////////////////////////////
// forward declarations
////////////////////////////////////////////////////////////////////////////////

void            ili9341_InitPanelStruct(LCD_panelStruct *panelPtr, byte index);
void            ili9341_InitDriver(void);
void            ili9341_ReadID(byte *rdata);
void            ili9341_ReadStatus(byte *rdata);
void            ili9341_ReadColorMode(byte *rdata);
void            ili9341_DisplayOn(void);
void            ili9341_DisplayOff(void);
void            ili9341_DisplayInvert(uint16_t invert);
void            ili9341_DisplayFramebuffer(void);
void            ili9341_SetRotation(byte rotation);

uint32_t        ili9341_SetCursor(LCD_addr_t x, LCD_addr_t y);
uint32_t        ili9341_SetDisplayWindow(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height);
void            ili9341_WritePixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color);
LCD_color_t     ili9341_ReadPixel(LCD_addr_t x, LCD_addr_t y);

void            ili9341_EnableGraphicsRamAccess(void);

////////////////////////////////////////////////////////////////////////////////
// variable declarations
////////////////////////////////////////////////////////////////////////////////

LCD_panelStruct *tftp;

const LCD_driverStruct _ili9341 =
{
		LCD_DRIVER_ILI9341,
		ili9341_InitPanelStruct,
		ili9341_InitDriver,
		ili9341_ReadID,
		ili9341_ReadStatus,
		ili9341_ReadColorMode,
		ili9341_DisplayOn,
		ili9341_DisplayOff,
		ili9341_DisplayInvert,
		ili9341_DisplayFramebuffer,
		ili9341_SetRotation,
		ili9341_SetCursor,
		ili9341_SetDisplayWindow,
		ili9341_WritePixel,
		ili9341_ReadPixel,
};

const byte ili9431InitSeq[] = {
		0,      // placeholder for total number of bytes
		ILI9341_SWRESET,        0,
		ILI9341_DELAY,          1,  100,                                // 100ms delay
		ILI9341_DISPLAY_OFF,    0,
	  //0xCA,               3,  0xC3,   0x08,   0x50,           // was in original ST code
		ILI9341_POWERA,         5,  0x39,   0x2C,   0x00,   0x34,   0x02,
		ILI9341_POWERB,         3,  0x00,   0xC1,   0x30,
		ILI9341_DTCA,           3,  0x85,   0x00,   0x78,
		ILI9341_DTCB,           2,  0x00,   0x00,
		ILI9341_POWER_SEQ,      4,  0x64,   0x03,   0x12,   0x81,
		ILI9341_PRC,            1,  0x20,
		ILI9341_POWER1,         1,  0x09,               // GVDD level.  0x09=3.3V;  TM was 0x23=4.6V
		ILI9341_POWER2,         1,  0x10,
		ILI9341_VCOM1,          2,  0x45,   0x15,       // (VMH, VML)  VMH  0x45=4.425V  TM was 0x3E=4.25V; VML  0x15=-1.975V TM was 0x28=-1.5V
		ILI9341_VCOM2,          1,  0x90,               // VMF  0x90=-48   TM was 0x86=-58
		ILI9341_MAC,            1,  0x08,               // TM was 0x48
		ILI9341_PIXEL_FORMAT,   1,  0x55,               // 16-bits/pixel for MCU and display
		ILI9341_FRMCTR1,        2,  0x00,   0x1B,       // (2 params)  XXX; #of clks/line (0x1B=27 clks)  TM was 0x18 (0x18=24 clks)
		ILI9341_DFC,            3,  0x08,   0x82,   0x27,   // 3 params  DEF=0x0A(ST=0x0A);  DEF=0x82(ST=0xA7)  DEF=0x27;
		ILI9341_COLUMN_ADDR,    4,  0x00,   0x00,   0x00,   0xEF,
		ILI9341_ROW_ADDR,       4,  0x00,   0x00,   0x01,   0x3F,
		ILI9341_GAMMA,          1,  0x01,               // select gamma2.2
		ILI9341_3GAMMA_EN,      1,  0x00,               // diabled
		ILI9341_PGAMMA,         15, 0x0F,   0x29,   0x24,   0x0C,   0x0E,   0x09,   0x4E,   0x78,   0x3C,   0x09,   0x13,   0x05,   0x17,   0x11,   0x00,   //ST
	  //ILI9341_PGAMMA,         15, 0x0F,   0x31,   0x2B,   0x0C,   0x0E,   0x08,   0x4E,   0xF1,   0x37,   0x07,   0x10,   0x03,   0x0E,   0x09,   0x00,   //TM
		ILI9341_NGAMMA,         15, 0x00,   0x16,   0x1B,   0x04,   0x11,   0x07,   0x31,   0x33,   0x42,   0x05,   0x0C,   0x0A,   0x28,   0x2F,   0x0F,   //ST
	  //ILI9341_NGAMMA,         15, 0x00,   0x0E,   0x14,   0x03,   0x11,   0x07,   0x31,   0xC1,   0x48,   0x08,   0x0F,   0x0C,   0x31,   0x36,   0x0F,   //TM
	  //ILI9341_RGB_INTERFACE,  1,  0xC2,
	  //ILI9341_INTERFACE,      3,  0x01,   0x00,   0x06,
		ILI9341_SLEEP_OUT,      0,
		ILI9341_DELAY,          1,  200,
		ILI9341_GRAM,           0,
};

////////////////////////////////////////////////////////////////////////////////
// // functions
////////////////////////////////////////////////////////////////////////////////

void ili9341_updateOffsetsAndFriends(void)
{
	byte value = 0;
	int my = 0;
	int mx = 0;
	int mv = 0;
	int ml = 0;
	int bgr = tftp->flipRGB;
	int mh = 0;

	switch(tftp->rotation)
	{
	case LCD_ROTATION_0:        my=1; mx=0; mv=0; break;
	case LCD_ROTATION_90:       my=1; mx=1; mv=1; break;
	case LCD_ROTATION_180:      my=0; mx=1; mv=0; break;
	case LCD_ROTATION_270:      my=0; mx=0; mv=1; break;
	case LCD_ROTATION_0_MX:     my=1; mx=1; mv=0; break;
	case LCD_ROTATION_90_MX:    my=1; mx=0; mv=1; break;
	case LCD_ROTATION_180_MX:   my=0; mx=0; mv=0; break;
	case LCD_ROTATION_270_MX:   my=0; mx=1; mv=1; break;
	default:                    my=1; mx=0; mv=0; break;    //LCD_ROTATION_0
	}

	//MY:MX:MV:ML:BGR:MH:0:0
	value |= (my  << 7);
	value |= (mx  << 6);
	value |= (mv  << 5);
	value |= (ml  << 4);
	value |= (bgr << 3);
	value |= (mh  << 2);

	tftp->xyMirrorRgbCtrl = value;
	
	tftp->ofsX = 0;
	tftp->ofsY = 0;
	
	if (mv)
	{
		tftp->width  = tftp->defaultHeight;
		tftp->height = tftp->defaultWidth;
	}
	else
	{
		tftp->width  = tftp->defaultWidth;
		tftp->height = tftp->defaultHeight;
	}

	tftp->maxXaddr = tftp->ofsX + tftp->width - 1;
	tftp->maxYaddr = tftp->ofsY + tftp->height - 1;
}

void ili9341_InitPanelStruct(LCD_panelStruct *panelPtr, byte index)
{
	tftp = panelPtr;
	tftp->index = index;
	// common to all flavors
	tftp->defaultWidth = 240;
	tftp->defaultHeight = 320;

	tftp->flipRGB = 1;
	tftp->rotation = LCD_ROTATION_DEFAULT;

	ili9341_updateOffsetsAndFriends();
	// common to all flavors
	tftp->maxSpiFrequency = 6666666; // 6.66MHz .. spec is 1 / 150ns
	tftp->supports16BitSpi = 1;
	tftp->hasMISO = 1;
	tftp->hasResetPin = 0;
}

void ili9341_InitDriver(void)
{
	if (tftp->hasResetPin)
	{
		LCD_RST_ENABLE;
		delayMsec(10);
		LCD_RST_DISABLE;
		delayMsec(130);
	}
	LCD_SendRegDataSequence((byte *)&ili9431InitSeq[0], sizeof(ili9431InitSeq), ILI9341_DELAY);
	ili9341_SetRotation(tftp->rotation);
}

void ili9341_ReadID(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(ILI9341_READ_DISPLAY_ID);
		LCD_SPI_ReadArray8(rdata, 5);
	}
}

void ili9341_ReadStatus(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(ILI9341_RDDST);
		LCD_SPI_ReadArray8(rdata, 5);
	}
}

void ili9341_ReadColorMode(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(ILI9341_RDDCOLMOD);
		LCD_SPI_ReadArray8(rdata, 2);
	}
}

void ili9341_DisplayOn(void)
{
	LCD_SPI_WriteReg8(ILI9341_DISPLAY_ON);
	ili9341_EnableGraphicsRamAccess();
}

void ili9341_DisplayOff(void)
{
	LCD_SPI_WriteReg8(ILI9341_DISPLAY_OFF);
	ili9341_EnableGraphicsRamAccess();
}

void ili9341_DisplayInvert(uint16_t invert)
{
	LCD_SPI_WriteReg8((invert == 1) ? ILI9341_DINVON : ILI9341_DINVOFF);
	ili9341_EnableGraphicsRamAccess();
}

void ili9341_DisplayFramebuffer(void)
{
	;
}

void ili9341_SetRotation(byte rotation)
{
	tftp->rotation = rotation;
	ili9341_updateOffsetsAndFriends();
	LCD_SPI_WriteReg8(ILI9341_MAC);



	LCD_SPI_Write8(tftp->xyMirrorRgbCtrl);


	ili9341_EnableGraphicsRamAccess();
}

uint32_t ili9341_SetCursor(LCD_addr_t x, LCD_addr_t y)
{   // app is responsible to enure x,y is within prior setting of active window

	if ((x>=0) && (x<tftp->width) && (y>=0) && (y<tftp->height))
	{
		LCD_SPI_WriteReg8(ILI9341_COLUMN_ADDR);
		LCD_SPI_Write16(tftp->ofsX + x);            // lower addr
		LCD_SPI_Write16(tftp->maxXaddr);            // upper addr
		LCD_SPI_WriteReg8(ILI9341_ROW_ADDR);
		LCD_SPI_Write16(tftp->ofsY + y);            // lower addr
		LCD_SPI_Write16(tftp->maxYaddr);            // upper addr
		ili9341_EnableGraphicsRamAccess();
		return(1);
	}
	else
	{
		return(0);  //off display
	}
}

uint32_t ili9341_SetDisplayWindow(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height)
{   // clip to screen and return the actual area

	uint16_t x1 = iFitWithinRange(x+width-1, 0, tftp->width-1);
	x = iFitWithinRange(x, 0, tftp->width-1);
	width = x1 - x + 1;

	uint16_t y1 = iFitWithinRange(y+height-1, 0, tftp->height-1);
	y = iFitWithinRange(y, 0, tftp->height-1);
	height = y1 - y + 1;

	LCD_SPI_WriteReg8(ILI9341_COLUMN_ADDR);
	LCD_SPI_Write16(tftp->ofsX + x);                // lower addr
	LCD_SPI_Write16(tftp->ofsX + x1);               // upper addr
	LCD_SPI_WriteReg8(ILI9341_ROW_ADDR);
	LCD_SPI_Write16(tftp->ofsY + y);                // lower addr
	LCD_SPI_Write16(tftp->ofsY + y1);               // upper addr
	ili9341_EnableGraphicsRamAccess();
	return((uint32_t)(x1 - x + 1) * (uint32_t)(y1 - y + 1));
}

void ili9341_WritePixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color)
{
	if (ili9341_SetCursor(x, y))
	{   //x, y was on the display
		LCD_SPI_Write16(color);
	}
}

LCD_color_t ili9341_ReadPixel(LCD_addr_t x, LCD_addr_t y)
{
	uint16_t rdata = 0;
	if (tftp->hasMISO)
	{
		if (ili9341_SetCursor(x, y))
		{   //x, y was on the display
			rdata = LCD_SPI_Read16();
		}
	}
	else
	{
		rdata = 0;
	}
	return(rdata);
}

////////////////////////////////////////////////////////////////////////////////

void ili9341_EnableGraphicsRamAccess(void)
{
	LCD_SPI_WriteReg8(ILI9341_GRAM);    // restore default access to graphics ram
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif //ADD_ON_SPI_DISPLAY
