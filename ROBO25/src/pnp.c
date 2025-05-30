#include "pnp.h"
#include "pinout.h"
#include "pins_stm32f4xx.h"
#include "stm32f4xx.h"
////////////////////////////////////////////////////////////////////////////////
uint16_t PnPResetTimer = 0; //spi3 valve timer dwell time
uint16_t PNPSPIData = 0; //spi3 valve control data word;

//this sets up a pointer to allow us to simply write to the DR_WORD variable to  update the
//spi data register.
static uint16_t * const DR_Word = (uint16_t * const)&SPI2->DR;


//PB10 SCK
//PC2 RCK
//PC03 MOSI
//PC0  RST
void Init_SPI2()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //PORTC clock enable
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; //SPI3 clock enable RCC_APB1RSTR RCC_APB1ENR_SPI3EN
	// SCK GPIOB pin 10 = alternate function mode 3
	GPIOB->MODER &= ~((GPIO_MODER_MODER10_0));
	GPIOB->MODER |= ((GPIO_MODER_MODER10_1));
	GPIOB->AFR[1] |= 6 << 8;// | 6 << 16;	// alternate mux
	// MOSI GPIOC pin 03 = alternate function mode 3
	GPIOC->MODER &= ~((GPIO_MODER_MODER3_0));
	GPIOC->MODER |= ((GPIO_MODER_MODER3_1));
	GPIOC->AFR[0] |= 6 << 12; // alternate mux

	// CR1
	SPI2->CR1 |= SPI_CR1_CPHA; // clock phase with respect to data
	SPI2->CR1 |= SPI_CR1_CPOL; // clock polarity when Idle, High, clocks on falling edge
	SPI2->CR1 |= SPI_CR1_MSTR; //master mode, we are driving the peripheral , tpic595
	SPI2->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2); //div by 256 for slowest possible buad
	SPI2->CR1 &= ~SPI_CR1_LSBFIRST; //most significant bit first, like shifting LEFT logical
	SPI2->CR1 |= SPI_CR1_SSI; //if controlling CS using the Nss pin, this defines cs state, high or low
	SPI2->CR1 |= SPI_CR1_SSM; //must be set to 1 for spi controller to drive cs, if zero, software drives CS.
	SPI2->CR1 &= ~SPI_CR1_RXONLY; //bi directional so we can send data
	SPI2->CR1 |= SPI_CR1_DFF; //0=8 bits transfer, 1=a6bit transfer
	SPI2->CR1 &= ~SPI_CR1_CRCNEXT; //next byte is from the data register, always , no crc
	SPI2->CR1 &= ~SPI_CR1_CRCEN; //disable crc hardware
	SPI2->CR1 |= SPI_CR1_BIDIOE; //output mode only

	SPI2->CR2 = 0; // no interrupts and no dma enabled
	
	SPI2->CR1 |= SPI_CR1_SPE; //spiEnable bit
}

//PC10 SCK
//PC11 RCK
//PC12 MOSI
//PA9  RST
void Init_SPI3()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //PORTC clock enable
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; //SPI3 clock enable RCC_APB1RSTR RCC_APB1ENR_SPI3EN
	// GPIOC pin 10, 12 = alternate function mode
	GPIOC->MODER &= ~((GPIO_MODER_MODER10_0) | (GPIO_MODER_MODER12_0));
	GPIOC->MODER |= ((GPIO_MODER_MODER10_1) | (GPIO_MODER_MODER12_1));
	// alternate mux
	GPIOC->AFR[1] |= 6 << 8 | 6 << 16;

	// CR1
	SPI3->CR1 |= SPI_CR1_CPHA; // clock phase with respect to data
	SPI3->CR1 |= SPI_CR1_CPOL; // clock polarity when Idle, High, clocks on falling edge
	SPI3->CR1 |= SPI_CR1_MSTR; //master mode, we are driving the peripheral , tpic595
	SPI3->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2); //div by 256 for slowest possible buad
	SPI3->CR1 &= ~SPI_CR1_LSBFIRST; //most significant bit first, like shifting LEFT logical
	SPI3->CR1 |= SPI_CR1_SSI; //if controlling CS using the Nss pin, this defines cs state, high or low
	SPI3->CR1 |= SPI_CR1_SSM; //must be set to 1 for spi controller to drive cs, if zero, software drives CS.
	SPI3->CR1 &= ~SPI_CR1_RXONLY; //bi directional so we can send data
	SPI3->CR1 |= SPI_CR1_DFF; //0=8 bits transfer, 1=a6bit transfer
	SPI3->CR1 &= ~SPI_CR1_CRCNEXT; //next byte is from the data register, always , no crc
	SPI3->CR1 &= ~SPI_CR1_CRCEN; //disable crc hardware
	SPI3->CR1 |= SPI_CR1_BIDIOE; //output mode only

	SPI3->CR2 = 0; // no interrupts and no dma enabled
	
	SPI3->CR1 |= SPI_CR1_SPE; //spiEnable bit
}
void SendPNPSPIDataToSpi2(uint16_t DataToSend)
{
	PnP_Enable_Set;
	PnP_Rclk_Clr;
	SPI2->CR1 |= SPI_CR1_SPE;
	SPI2->CR1 |= 1 << 9; //SPI_CR1_CSTART;


	//*(volatile uint8_t*)&_handle->dd->DR = val; 		// Push 1 byte
	//*(volatile uint16_t*)&SPI3->TXDR = PNPSPIData; 	// Push 2 bytes
	*DR_Word = DataToSend;
	//*((__IO uint16_t*)&SPI3->TXDR) = PNPSPIData;
	//PnPFeederValue++;
}
//PnP_Enable_Set; // set the enable on the tpic595
//PnP_Rclk_Clr; // raise the Rclock
void LatchPnPData()
{
	//GPIOC->BSRRH = PIN_MASK_11;
	PnP_Rclk_Set;
	//PnP_Enable_Clr;
}
void PnP_SetValves()
{
	//	PNPExercisor++;
	//	SendPNPSPIDataToSpi3((uint16_t)PNPExercisor); //turn off all valves,we arefinished
	//	return;
		//if(	PNPSPIData==0)return;
	if (PnPResetTimer > 0)
	{
		SendPNPSPIDataToSpi2(PNPSPIData); //update the data word;
		PnPResetTimer--;
		if (PnPResetTimer == 0)PNPSPIData = 0;
	}
}
void PnP_TurnOffAllValves()
{
	PNPSPIData = 0;
	//SendPNPSPIDataToSpi3(PNPSPIData);//update the data word;
}
