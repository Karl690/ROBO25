////////////////////////////////////////////////////////////////////////////////
//
// File:    HardwareInit.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: This module contains the method to initialize the various hardware
//          peripherals that are used.   Additionally, much of the global variable
//          space is initiialized, though a lot of it is in single variables and those
//          are not captured here.
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "gpio.h"
#include "mailbox.h"
#include "HardwareInit.h" // (get our own global defines and typedefs)
#include "serial.h"
#include "Hydra_can.h"
#include "GCode.h"
#include "Motordriver.h"
#ifdef HYDRA_DIAGS
#include "stm32f4xx_adc.h"
#endif

////////////////////////////////////////////////////////////////////////////////
//  Local #defines (defines ONLY used in this module)
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
//  Public global definitions (exposed in HardwareInit.h)
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Local global definitions (do not expose in HardwareInit.h)
////////////////////////////////////////////////////////////////////////////////

TIM_TimeBaseInitTypeDef MyTIM_TimeBaseInitStruct;

////////////////////////////////////////////////////////////////////////////////
//  Forward declarations - any local modules needing an early template
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////


void InitGPIO(void)
{   // Description: Sets General purpouse IO Pins direction

	// Enable GPIO clock and release reset
	//FOR REFERENCE, YOU CAN LOOK AT STM32F4XX_RCC.H FOR BUS DESIGNATORS

	RCC_AHB1PeriphClockCmd(             //ENABLE THE CLOCK SO THE CONFIGURATION REGS CAN BE CHANGED
			RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
			RCC_AHB1Periph_GPIOC |
			RCC_AHB1Periph_GPIOD |
			RCC_AHB1Periph_GPIOE |
			RCC_AHB1Periph_GPIOF |
			RCC_AHB1Periph_GPIOG |
			RCC_AHB1Periph_GPIOH |
			RCC_AHB1Periph_GPIOI,
			ENABLE);
	RCC_APB1PeriphResetCmd(             //RELEASE THE RESET STATE
			RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
			RCC_AHB1Periph_GPIOC |
			RCC_AHB1Periph_GPIOD |
			RCC_AHB1Periph_GPIOE |
			RCC_AHB1Periph_GPIOF |
			RCC_AHB1Periph_GPIOG |
			RCC_AHB1Periph_GPIOH |
			RCC_AHB1Periph_GPIOI,
			DISABLE);

	pinInitAll();
	
	initSensor(&startButton, SENSOR_INDEX_START, "Start", ((START == PIN_UNDEFINED) ? FALSE : TRUE), ACTIVE_LOW, START, DEFAULT_SENSOR_DEBOUNCE_READS);

	initSensor(&EMO, SENSOR_INDEX_EMO, "EMO", FALSE, ACTIVE_LOW, EMO_PIN, DEFAULT_SENSOR_DEBOUNCE_READS);

//#ifdef USE_AB_ENCODER
//	initSensor(&ABEncoderSelectButton, SENSOR_INDEX_ABSEL, "ABSel", ((PANEL_ENC_SEL == PIN_UNDEFINED) ? FALSE : TRUE), ACTIVE_LOW, PANEL_ENC_SEL, DEFAULT_SENSOR_DEBOUNCE_READS);
//#endif //USE_AB_ENCODER
}

////////////////////////////////////////////////////////////////////////////////

void InitUSART3(unsigned int baudrate)
{	// Configure UART @ SYSTEM_BAUD_RATE w/ 8 bits no parity and 1 stop bit, no flow control
	//used to talk to the EZ80 laser power supply, 9600 baud
	initClkAndResetAPB1(RCC_APB1Periph_USART3);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);

	interruptSetupAndEnable(USART3_IRQn, NVIC_PREMPTION_PRIORITY_UARTS);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////

void InitUART4(unsigned int baudrate)
{	// Configure UART @ SYSTEM_BAUD_RATE w/ 8 bits no parity and 1 stop bit, no flow control
	initClkAndResetAPB1(RCC_APB1Periph_UART4);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART4, &USART_InitStructure);
	USART_Cmd(UART4, ENABLE);

	interruptSetupAndEnable(UART4_IRQn, NVIC_PREMPTION_PRIORITY_UARTS);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////

void InitUSART6(unsigned int baudrate)
{	// Configure UART @ SYSTEM_BAUD_RATE w/ 8 bits no parity and 1 stop bit, no flow control
	initClkAndResetAPB2(RCC_APB2Periph_USART6);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART6, &USART_InitStructure);
	USART_Cmd(USART6, ENABLE);

	interruptSetupAndEnable(USART6_IRQn, NVIC_PREMPTION_PRIORITY_UARTS);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
}

////////////////////////////////////////////////////////////////////////////////

void InitAllUARTS(unsigned int baudrate)
{

	InitUSART3(9600);               //external ftdi usb com port
	InitUSART6(baudrate);              //external ftdi usb com port
}

void CAN_Config(CAN_TypeDef* CANx)
{
	initClkAndResetAPB1((CANx == CAN1) ? RCC_APB1Periph_CAN1 : RCC_APB1Periph_CAN2);

	CAN_InitTypeDef CAN_InitStructure;
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 56; // 56 = 125Kbit/sec based on 42MHz APB1 clock
#ifdef CAN_500KBPS
	CAN_InitStructure.CAN_Prescaler = 14; // 14 = 500Kbit/sec based on 42MHz APB1 clock
#endif
#ifdef CAN_1000KBPS
	CAN_InitStructure.CAN_Prescaler = 7; // 7 = 1000Kbit/sec based on 42MHz APB1 clock
#endif

	CAN_Init(CANx, &CAN_InitStructure);

	if (CANx == CAN1)
	{   // filters are all sent up via CAN1 (CAN1 owns the memory for the shared filters)
		// shared filter bank -- can only access them via CAN1 address.
		// configure for filters 0-13 for CAN1 and 14-27 for CAN2
		// filter 0 will allow all CAN1 traffic in to the can1 hw
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)((FILTER_CONSTANT_VALUE >> 16) & 0xfff); // was 0x8000
		CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t)(FILTER_CONSTANT_VALUE & 0xfff);    // was 0x0004
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)(((FILTER_MASK_CONSTANT_SYS) >> 16) & 0xfff); // was 0xe000
		CAN_FilterInitStructure.CAN_FilterMaskIdLow = (uint16_t)(FILTER_MASK_CONSTANT_SYS & 0xfff); // was 0x0004
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
		CAN_FilterInitStructure.CAN_FilterNumber = 0;
		CAN_FilterInit(&CAN_FilterInitStructure);
	}
	else if (CANx == CAN2)
	{   // filters are all sent up via CAN1 (CAN1 owns the memory for the shared filters)
		// shared filter bank -- can only access them via CAN1 address.
		// configure for filters 0-13 for CAN1 and 14-27 for CAN2
		// filter 14 will allow all CAN2 traffic in to the can2 hw
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)((FILTER_CONSTANT_VALUE >> 16) & 0xfff); // was 0x8000
		CAN_FilterInitStructure.CAN_FilterIdLow = (uint16_t)(FILTER_CONSTANT_VALUE & 0xfff);    // was 0x0004
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)(((FILTER_MASK_CONSTANT_SYS) >> 16) & 0xfff); // was 0xe000
		CAN_FilterInitStructure.CAN_FilterMaskIdLow = (uint16_t)(FILTER_MASK_CONSTANT_SYS & 0xfff); // was 0x0004
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
		CAN_FilterInitStructure.CAN_FilterNumber = 14;
		CAN_FilterInit(&CAN_FilterInitStructure);
	}
}

////////////////////////////////////////////////////////////////////////////////

void InitMotorTimer(MotorStructure *M, boolean enableISR)
{
	TIM_TypeDef *TimerBase = M->TimerBase;
	uint32_t apb;
	uint32_t periph;
	uint32_t IRQ;

	if      (TimerBase == TIM1)  { apb=2;   periph=RCC_APB2Periph_TIM1;   IRQ = TIM1_UP_TIM10_IRQn; }
	else if (TimerBase == TIM2)  { apb=1;   periph=RCC_APB1Periph_TIM2;   IRQ = TIM2_IRQn; }
	else if (TimerBase == TIM3)  { apb=1;   periph=RCC_APB1Periph_TIM3;   IRQ = TIM3_IRQn; }
	else if (TimerBase == TIM4)  { apb=1;   periph=RCC_APB1Periph_TIM4;   IRQ = TIM4_IRQn; }
	else if (TimerBase == TIM5)  { apb=1;   periph=RCC_APB1Periph_TIM5;   IRQ = TIM5_IRQn; }
	else if (TimerBase == TIM6)  { apb=1;   periph=RCC_APB1Periph_TIM6;   IRQ = TIM6_DAC_IRQn; }
	else if (TimerBase == TIM7)  { apb=1;   periph=RCC_APB1Periph_TIM7;   IRQ = TIM7_IRQn; }
	else if (TimerBase == TIM8)  { apb=2;   periph=RCC_APB2Periph_TIM8;   IRQ = TIM8_UP_TIM13_IRQn; }
	else if (TimerBase == TIM9)  { apb=2;   periph=RCC_APB2Periph_TIM9;   IRQ = TIM1_BRK_TIM9_IRQn; }
	else if (TimerBase == TIM10) { apb=2;   periph=RCC_APB2Periph_TIM10;  IRQ = TIM1_UP_TIM10_IRQn; }
	else if (TimerBase == TIM11) { apb=2;   periph=RCC_APB2Periph_TIM11;  IRQ = TIM1_TRG_COM_TIM11_IRQn; }
	else if (TimerBase == TIM12) { apb=1;   periph=RCC_APB1Periph_TIM12;  IRQ = TIM8_BRK_TIM12_IRQn; }
	else if (TimerBase == TIM13) { apb=1;   periph=RCC_APB1Periph_TIM13;  IRQ = TIM8_UP_TIM13_IRQn; }
	else if (TimerBase == TIM14) { apb=1;   periph=RCC_APB1Periph_TIM14;  IRQ = TIM8_TRG_COM_TIM14_IRQn; }
	else return;

	// in case a timer is re-init, make sure it's ISR is disabled to avoid spurious interrupts
	interruptSetupAndDisable(IRQ, NVIC_PREMPTION_PRIORITY_STEP_TIMER);

	TIM_ITConfig(TimerBase, TIM_FLAG_Update, DISABLE);

	// Enable Timer clock and release reset
	if (apb == 1)
	{
		initClkAndResetAPB1(periph);
		M->TimerFreq = 84000000.0f;
	}
	else {
		initClkAndResetAPB2(periph);
		M->TimerFreq = 168000000.0f;
	}

	MyTIM_TimeBaseInitStruct.TIM_Prescaler = 0xffff;
	MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	MyTIM_TimeBaseInitStruct.TIM_Period = 0xffff;
	MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TimerBase, &MyTIM_TimeBaseInitStruct);

	TIM_UpdateRequestConfig(TimerBase, TIM_UpdateSource_Regular);   // limit interrupt to over/underflow  and DMA only

	if (enableISR)
	{
		NVIC_EnableIRQ(IRQ);

		TIM_ClearITPendingBit(TimerBase, TIM_FLAG_Update);  // Clear update interrupt bit
		TIM_ITConfig(TimerBase, TIM_FLAG_Update, ENABLE);   // Enable update interrupt
	}
}

////////////////////////////////////////////////////////////////////////////////

void InitTimer1(void)
{   // Timer1 will be used as the master timer for coordinated motion.  It will run at the rate needed for the dominant axis,
	// including changes for acceleration and deceleration.  This will set the timer basics.  actual speed and enable is done
	// elsewhere

	initClkAndResetAPB2(RCC_APB2Periph_TIM1);

	interruptSetupAndDisable(TIM1_UP_TIM10_IRQn, NVIC_PREMPTION_PRIORITY_MASTER_MOTION_TIMER);

	TIM_ITConfig(TIM1, TIM_FLAG_Update, DISABLE);
	MyTIM_TimeBaseInitStruct.TIM_Prescaler = 0xffff;
	MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	MyTIM_TimeBaseInitStruct.TIM_Period = 0xffff;
	MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&MyTIM_TimeBaseInitStruct);
	TIM_UpdateRequestConfig(TIM1, TIM_UpdateSource_Regular);    // limit interrupt to over/underflow  and DMA only

	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);   // Clear update interrupt bit
	TIM_ITConfig(TIM1, TIM_FLAG_Update, ENABLE);    // Enable update interrupt

	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

////////////////////////////////////////////////////////////////////////////////

void InitTimer2usec(void)
{   // Timer2 will be used as the interval timer for measuring the time taken to process slices

	initClkAndResetAPB1(RCC_APB1Periph_TIM2);

	interruptSetupAndDisable(TIM2_IRQn, NVIC_PREMPTION_PRIORITY_SLICETIME_MEASURE);

	MyTIM_TimeBaseInitStruct.TIM_Prescaler = 83;     // drop down to a 1 usec clock to the 32-counter
	MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	MyTIM_TimeBaseInitStruct.TIM_Period = 0xffffffff; // full run of the 32-bit counter
	MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&MyTIM_TimeBaseInitStruct);
	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);    // limit interrupt to over/underflow  and DMA only
	TIM_Cmd(TIM2, ENABLE);
}

/////////////////////////////////////////////////////////////////////////////////

void InitTim3RpmInput(void)
{   // Timer2 will be used as the interval timer for measuring time slip relative to systick timer

//    // Enable the clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_ITConfig(TIM3, TIM_FLAG_Update, DISABLE); //NO Interrupts!
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	// Configure the Timer2 base structure
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;              // No prescaling
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;        // Maximum period
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	

	// Configure PD2 as an input pin
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	// Connect PD2 to TIM3 (AF1)
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_TIM3);

	// Configure TIM2 to use PA0 as ETR input
	// breaks TIM_ETRConfig(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	TIM_ETRClockMode1Config(TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);

	TIM_Cmd(TIM3, ENABLE);

}

/////////////////////////////////////////////////////////////////////////////////


///
void ConfigureTimer4PwmOutputsFor0_10V(void)
{
		initClkAndResetAPB1(RCC_APB1Periph_TIM4);

		TIM_ITConfig(TIM4, TIM_FLAG_Update, DISABLE); //NO Interrupts!

		MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		MyTIM_TimeBaseInitStruct.TIM_Prescaler = 30;
		MyTIM_TimeBaseInitStruct.TIM_Period = 101;
		MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &MyTIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_OCStructInit(&TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0; // 0% "off"  sets CCR4
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;


	

		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); //CCR3
	
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //CCR4
	
		
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_CtrlPWMOutputs(TIM4, ENABLE); //  Enable PWM outputs
		TIM_Cmd(TIM4, ENABLE);
}

/////////////////////////////////////////////////////////////////////////////////

void InitEncoderTimer5(void)
{   //encoder wheel works on input A0 and A1 Timer5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	// Configure the pins for encoder mode (assuming PA0 and PA1 for Channels A and B)
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Connect the pins to TIM5 (AF2)
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	// Configure the Timer5 base structure
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0; // No prescaling
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF; // Maximum period
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// Configure TIM5 in encoder mode
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	// Enable TIM5
	TIM_Cmd(TIM5, ENABLE);

}

////////////////////////////////////////////////////////////////////////////////

void enableLatheMode(void)
{   // setup timer and I/O for lathe mode


	if (_gs.latheMotor)
	{
		pinInit(LATHE_STEP);    // CCR2 result in output on this pin

		MotorStructure *M = _gs.latheMotor;
		InitMotorTimer(M, FALSE);

		MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		MyTIM_TimeBaseInitStruct.TIM_Prescaler = 0xffff; //psc
		MyTIM_TimeBaseInitStruct.TIM_Period = 0xffff; //arr;
		MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(M->TimerBase,&MyTIM_TimeBaseInitStruct);

		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_OCStructInit(&TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		//TIM_OCInitStructure.TIM_OutputNState  = TIM_OutputNState_Enable ;
		TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable ;
		TIM_OCInitStructure.TIM_Pulse = 0;  // "off"  sets CCR2
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
		TIM_OC2Init(M->TimerBase, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(M->TimerBase, TIM_OCPreload_Enable);   //CCR2

		TIM_ARRPreloadConfig(M->TimerBase, ENABLE);
		TIM_CtrlPWMOutputs(M->TimerBase, ENABLE);  //  Enable PWM outputs
	}
}

////////////////////////////////////////////////////////////////////////////////

void disableLatheMode(void)
{   // restore timer and I/O pin to normal use
	if (_gs.latheMotor)
	{
		MotorStructure *M = _gs.latheMotor;
		InitMotorTimer(M, TRUE);    // set if up for normal use
		pinInit(M->Step.Bit);
		M->latheMode = LATHE_MODE_OFF;
	}
}


////////////////////////////////////////////////////////////////////////////////

void InitTimer6(void)
{   // Timer6 will be used as the backdoor timer for jogging motors while other axis are moving.
	// set the timer basics.  actual speed and enable is done elsewhere

	initClkAndResetAPB1(RCC_APB1Periph_TIM6);

	// in case a timer is re-init, make sure it's ISR is disabled to avoid spurious interrupts
	interruptSetupAndDisable(TIM6_DAC_IRQn, NVIC_PREMPTION_PRIORITY_STEP_ZJOG);

	TIM_ITConfig(TIM6, TIM_FLAG_Update, DISABLE);
	MyTIM_TimeBaseInitStruct.TIM_Prescaler = 0xffff;
	MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	MyTIM_TimeBaseInitStruct.TIM_Period = 0xffff;
	MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6,&MyTIM_TimeBaseInitStruct);

	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	TIM_UpdateRequestConfig(TIM6, TIM_UpdateSource_Regular);

	TIM_ClearITPendingBit(TIM6, TIM_FLAG_Update);   // Clear update interrupt bit
	TIM_ITConfig(TIM6, TIM_FLAG_Update, ENABLE);    // Enable update interrupt
}

////////////////////////////////////////////////////////////////////////////////

void InitTimer7(void)
{   // Timer7 will be used during startup to measure a time interval for testing the "START" button
	// to determine if the lightburn mode is enabled
	//
	// after boot (lightburn) TIM7 is used as a path to a simple sw controlled interrupt as a place to kick
	// off dom axis accel calc in a lower pri ISR than the step gen

	initClkAndResetAPB1(RCC_APB1Periph_TIM7);

	// in case a timer is re-init, make sure it's ISR is disabled to avoid spurious interrupts
	interruptSetupAndDisable(TIM7_IRQn, NVIC_PREMPTION_PRIORITY_MASTER_ACCEL_CALC);

	TIM_ITConfig(TIM7, TIM_FLAG_Update, DISABLE);
	MyTIM_TimeBaseInitStruct.TIM_Prescaler = 8399;  //84000000 / 8400 = 100usec clk to the counter
	MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	MyTIM_TimeBaseInitStruct.TIM_Period = 99;       // interrupt every 10ms
	MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7,&MyTIM_TimeBaseInitStruct);

	NVIC_EnableIRQ(TIM7_IRQn);

	TIM_UpdateRequestConfig(TIM7, TIM_UpdateSource_Regular);

	TIM_ClearITPendingBit(TIM7, TIM_FLAG_Update);   // Clear update interrupt bit
	TIM_ITConfig(TIM7, TIM_FLAG_Update, ENABLE);    // Enable update interrupt
	TIM_Cmd(TIM7, ENABLE);
}
void SetCO2LaserPwmFrequency(int desiredFreqInKhz)
{
	float invertedF = (float)1000.0f / desiredFreqInKhz;
	invertedF *= 1633;
	if (invertedF < 30)invertedF = 30;
	if (invertedF > 10000)invertedF = 10000;
	TIM8->PSC = (int16_t)invertedF;
}

void InitTimer8(void)
{
	// Timer8 is used for CO2 laser PWM direct drive to TH pin on power supply
	//Timer4 pwm>>analog 0-10v goes to the analog control input on the laser powersupply

	initClkAndResetAPB2(RCC_APB2Periph_TIM8);

	TIM_ITConfig(TIM8, TIM_FLAG_Update, DISABLE); //NO Interrupts!

	MyTIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	MyTIM_TimeBaseInitStruct.TIM_Prescaler = 160;//set to 10khz to begin with
	MyTIM_TimeBaseInitStruct.TIM_Period = 101;
	MyTIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV2;
	MyTIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &MyTIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//TIM_OutputNState
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	
	TIM_OCInitStructure.TIM_OutputNState  = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // 0% "off"  sets CCR4
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); //CCR4
		
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CtrlPWMOutputs(TIM8, ENABLE); //  Enable PWM outputs
	TIM_Cmd(TIM8, ENABLE);
	TIM8->CCR3 = 0;
	CO2LaserAnalogPwrPWM = 0;
}
////////////////////////////////////////////////////////////////////////////////

void InitDAC(void)
{
	initClkAndResetAPB1(RCC_APB1Periph_DAC);
}

////////////////////////////////////////////////////////////////////////////////
//B1 co2 laser water temp
//A5 HOtbed temp
void adcInit(ADC_TypeDef *ADCx)
{
	ADC_InitTypeDef ADC_InitStructure;

	uint32_t mask;
	if (ADCx == ADC1)
		mask = RCC_APB2Periph_ADC1;
	else if (ADCx == ADC2)
		mask = RCC_APB2Periph_ADC2;
	else if (ADCx == ADC3)
		mask = RCC_APB2Periph_ADC3;
	else
		mask = 0;

	RCC->APB2ENR |= (mask);
	RCC->APB2RSTR &= ~(mask);

	// define ADC config
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;             // we work in single sampling mode
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADCx, &ADC_InitStructure);   //set config of ADC1

	ADC_Cmd(ADCx, ENABLE);
}


////////////////////////////////////////////////////////////////////////////////

void DeInitLaser(void)
{
	if (_gs._laser.outboxPtr != NULL)
	{   // safe to talk to the laser to shut it down
		SetLaserPower(0, 0); //set the power level to 0 (off)
		sendSwitchControlOffToDevice(_gs._laser.outboxPtr, HH_HTR_SWITCH);

		if (_gs._laser.localControl)
		{   // reset timers and IO to non laser use
			InitEncoderTimer5();
		}
		_g4DwellTimer = 100; // let laser power supply have time to settle (off)
	}
	initializeLaserStruct();
}

////////////////////////////////////////////////////////////////////////////////

void InitLaser(void)
{
	_gs._laser.gammaValue = 1.0f;
	int i;
	for (i=0; i<256; i++)
	{   // set unity map
		_gs._laser.rasterColorIndexTable[i] = i;
	}

	if (_gs._laser.enabled && _gs._laser.outboxPtr)
	{   // safe to enable the laser power
		sendSetClearErrorBit(_gs._laser.outboxPtr->device); // reset error flags
		int i;
		for (i=0; i<NUM_DEVICE_ERROR_CODES; i++)
		{   // clear sticky bit for the laser device
			_MailBoxes.stickyErrorMsg[i] &= ~(1 << getMailboxNum(_gs._laser.outboxPtr->device));
		}
		//NUKE sendCooldownTimeToDevice(_gs._laser.outboxPtr, _gs._laser.cooldown);//  send the cooldown time AFTER laser power is turned off
		canSendOutboxInitValue1x16(_gs._laser.outboxPtr, DEV_INIT_AREA_DEV,  DEV_INIT_INDEX_DEV_COOLDOWN, _gs._laser.cooldown);
		sendSwitchControlOnToDevice(_gs._laser.outboxPtr, HH_HTR_SWITCH);

		if (_gs._laser.localControl)
		{
			InitEncoderTimer5();
			pinInit(CO2_LASER_PWM);
		}
		else
		{
#ifdef GB_HIDDEN_WARNINGS
			int ADD_DIODE_LASER_PWM_SETUP_CODE_HERE;
#endif //GB_HIDDEN_WARNINGS
		}
		SetLaserPower(0, 0); //set the power level to 0 (off)
	}
	else
	{   // something wrong so shut down
		DeInitLaser();
	}
}

////////////////////////////////////////////////////////////////////////////////

void initializeSpindleStruct(void)
{
	_gs._spindle.desiredPowerPct = 0.0f;
	_gs._spindle.pwmFreq = DEFAULT_SPINDLE_PWM_FREQ;
}
////////////////////////////////////////////////////////////////////////////////

void initializeInkjetStruct(void)
{
	_gs._inkjet.dropletsPerMm = DEFAULT_INKJET_DROPLETS_PER_MM;
}

////////////////////////////////////////////////////////////////////////////////

void initializeLaserStruct(void)
{
	_gs._laser.device = HH_POSITION_UNPLUGGED;
	_gs._laser.outboxPtr = NULL;
	_gs._laser.enabled = FALSE;
	_gs._laser.polarity = ACTIVE_HIGH;
	_gs._laser.watchdogMs = 0;
	_gs._laser.cooldown = 0;
	_gs._laser.localControl = FALSE;
	_gs._laser.pwmFreq   = 2000;
	_gs._laser.vectorPowerPct = 0.0f;
	_gs._laser.piercePowerPct = 0.0f;

#ifdef USE_AB_ENCODER
	_gs._laser.TimerBase = TIM4;
#else //!USE_AB_ENCODER
	_gs._laser.TimerBase = TIM5;
#endif //!USE_AB_ENCODER

	_gs._laser.rasterPulsesPerDot = 1;
	_gs._laser.rasterPulsesPerDotCounter = 0;
	_gs._laser.rasterFrontPorchDots = 0;
	_gs._laser.rasterFrontPorchDotsCounter = 0;
	_gs._laser.rasterLineCounter = 0;

	_gs._laser.rasterScanFeedrate = 0.0f;
	_gs._laser.rasterImageOriginMmX = 0.0f;
	_gs._laser.rasterImageOriginMmY = 0.0f;
	_gs._laser.rasterImageDotsPerMmX = 0.0f;
	_gs._laser.rasterImageDotsPerMmY = 0.0f;

	_gs._laser.rasterBidirectionalScan = FALSE;
	_gs._laser.rasterBidirReturnStartAdjust = RASTER_REVERSE_SCAN_TWEAK;

	_gs._laser.rasterizeCurrentMove = FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void setupIWDG(float interval)
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_LSICmd(ENABLE);  // enable Low Speed Internal oscillator (L.S.I.)
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET); // wait for LSI to be ready

	// use Timer5 (32-bit) to count the number of APB1 clocks (84MHz) that go by
	// for 8 (TIM_ICPSC_DIV8) rising edges of the LSI using TIM5_CH4 for input capture
	// of the LSI output

	initClkAndResetAPB1(RCC_APB1Periph_TIM5);

	TIM_RemapConfig(TIM5, TIM5_LSI);  // TIM5_CH4 Input Capture connected to the LSI clock output
	TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate); // no PSC divide, immed reload

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;                  // LSI
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;       // rising edge of LSI
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;             // count 8 rising edges
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	interruptSetupAndEnable(TIM5_IRQn, NVIC_PREMPTION_PRIORITY_XTAL_TIMING_CHECK);

	TIM5->SR = 0;                             // Reset all timer flags
	TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);   // Enable CC4 Interrupt Request
	TIM_Cmd(TIM5, ENABLE);

	while(lsiCC4sampleIndex != 2);    // wait untit 2 capture compare samples are recorded in the TIM5 ISR

	TIM_DeInit(TIM5); // done with TIM5 so restore to it's default (POR) state

	lsiActualFreq = (uint32_t)(84000000.0f / ((float)(lsiCC4samples[1] - lsiCC4samples[0]) / 8.0f));

	// set up IWDG for a 5 second timeout;

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  // Enable write access to IWDG_PR and IWDG_RLR register

	IWDG_SetPrescaler(IWDG_Prescaler_256); // prescale from nom 32KHz clock to nom 0.125KHz (8ms period)

	// if goal is a 5 second watchdog....CNT = 5 / 0.008 = 625 nom....however, we need to use the measure LSI
	// freq, so for 5 second:    CNT = 5 * (lsiActualFreq / 256)

	IWDG_SetReload((uint16_t)(interval * ((float)lsiActualFreq / 256.0f)));
	IWDG_ReloadCounter(); // Reload IWDG counter
	IWDG_Enable();

	// shutdown TIM5 and interrups
	deinitClkAndResetAPB1(RCC_APB1Periph_TIM5);
	interruptSetupAndDisable(TIM5_IRQn, NVIC_PREMPTION_PRIORITY_LOWEST);
}

////////////////////////////////////////////////////////////////////////////////

void setupDBGMCU(void)
{
// DBGMCU->IDCODE
// DBGMCU->CR
// DBGMCU->APB1FZ
// DBGMCU->APB2FZ

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;   //            ((uint32_t)0x00000001)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;   //            ((uint32_t)0x00000002)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;   //            ((uint32_t)0x00000004)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;   //            ((uint32_t)0x00000008)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP;   //            ((uint32_t)0x00000010)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP;   //            ((uint32_t)0x00000020)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM12_STOP;   //           ((uint32_t)0x00000040)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM13_STOP;   //           ((uint32_t)0x00000080)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM14_STOP;   //           ((uint32_t)0x00000100)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_RTC_STOP;   //             ((uint32_t)0x00000400)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_WWDG_STOP;   //            ((uint32_t)0x00000800)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;   //            ((uint32_t)0x00001000)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT;   //   ((uint32_t)0x00200000)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT;   //   ((uint32_t)0x00400000)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT;   //   ((uint32_t)0x00800000)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_CAN1_STOP;   //            ((uint32_t)0x02000000)
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_CAN2_STOP;   //            ((uint32_t)0x04000000)
/* Old IWDGSTOP bit definition, maintained for legacy purpose */
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDEG_STOP;   //           DBGMCU_APB1_FZ_DBG_IWDG_STOP

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM1_STOP;   //        ((uint32_t)0x00000001)
	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM8_STOP;   //        ((uint32_t)0x00000002)
	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM9_STOP;   //        ((uint32_t)0x00010000)
	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM10_STOP;   //       ((uint32_t)0x00020000)
	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM11_STOP;   //       ((uint32_t)0x00040000)
}

////////////////////////////////////////////////////////////////////////////////

void setFlasherVarPtr(flasherVarSel_t varSel)
{
	 _gs._flasher.varSel = varSel;
	switch (varSel)
	{
	case FLASHER_VAR_SEL_ERROR:                     _gs._flasher.varPtr = (int *)&_gs._flasher.error; break;
	case FLASHER_VAR_SEL_CMD_RECEIVED:              _gs._flasher.varPtr = (int *)&_gs._flasher.cmd_received; break;
	case FLASHER_VAR_SEL_ACK_PENDING:               _gs._flasher.varPtr = (int *)&_gs._flasher.ack_pending; break;
	case FLASHER_VAR_SEL_G4_DWELL_TIMER:            _gs._flasher.varPtr = (int *)&_g4DwellTimer; break;
	case FLASHER_VAR_SEL_GCODE_PAUSED:              _gs._flasher.varPtr = (int *)&_gcodePaused; break;
	case FLASHER_VAR_SEL_ABORTING:                  _gs._flasher.varPtr = (int *)&_abortInProgress; break;
	case FLASHER_VAR_SEL_BLOCK_ALL_MOTION:          _gs._flasher.varPtr = (int *)&_blockAllMotion; break;
	case FLASHER_VAR_SEL_BLOCK_ABS_MOTION:          _gs._flasher.varPtr = (int *)&_blockAbsoluteMotion; break;
	//case FLASHER_VAR_SEL_MOTION_OCCURRED:         _gs._flasher.varPtr = (int *)&_motionHasOccurred; break;
	case FLASHER_VAR_SEL_VALID_KEY:                 _gs._flasher.varPtr = (int *)&_validFirmwareKey; break;

	case FLASHER_VAR_SEL_FAULT_STATE_X:             _gs._flasher.varPtr = (int *)&Motors[M_X].FaultSense.State; break;
	case FLASHER_VAR_SEL_FAULT_STATE_Y:             _gs._flasher.varPtr = (int *)&Motors[M_Y].FaultSense.State; break;
	case FLASHER_VAR_SEL_FAULT_STATE_Z:             _gs._flasher.varPtr = (int *)&Motors[M_Z].FaultSense.State; break;
	case FLASHER_VAR_SEL_FAULT_STATE_A:             _gs._flasher.varPtr = (int *)&Motors[M_A].FaultSense.State; break;
	case FLASHER_VAR_SEL_FAULT_STATE_B:             _gs._flasher.varPtr = (int *)&Motors[M_B].FaultSense.State; break;
	case FLASHER_VAR_SEL_FAULT_STATE_C:             _gs._flasher.varPtr = (int *)&Motors[M_C].FaultSense.State; break;

	case FLASHER_VAR_SEL_LIMIT1_STATE_X:            _gs._flasher.varPtr = (int *)&Motors[M_X].Limit1Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT1_STATE_Y:            _gs._flasher.varPtr = (int *)&Motors[M_Y].Limit1Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT1_STATE_Z:            _gs._flasher.varPtr = (int *)&Motors[M_Z].Limit1Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT1_STATE_A:            _gs._flasher.varPtr = (int *)&Motors[M_A].Limit1Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT1_STATE_B:            _gs._flasher.varPtr = (int *)&Motors[M_B].Limit1Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT1_STATE_C:            _gs._flasher.varPtr = (int *)&Motors[M_C].Limit1Sense.State; break;

	case FLASHER_VAR_SEL_LIMIT2_STATE_X:            _gs._flasher.varPtr = (int *)&Motors[M_X].Limit2Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT2_STATE_Y:            _gs._flasher.varPtr = (int *)&Motors[M_Y].Limit2Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT2_STATE_Z:            _gs._flasher.varPtr = (int *)&Motors[M_Z].Limit2Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT2_STATE_A:            _gs._flasher.varPtr = (int *)&Motors[M_A].Limit2Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT2_STATE_B:            _gs._flasher.varPtr = (int *)&Motors[M_B].Limit2Sense.State; break;
	case FLASHER_VAR_SEL_LIMIT2_STATE_C:            _gs._flasher.varPtr = (int *)&Motors[M_C].Limit2Sense.State; break;

	case FLASHER_VAR_SEL_NONE:
	default: _gs._flasher.varSel = FLASHER_VAR_SEL_NONE;  _gs._flasher.varPtr = NULL; break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void InitGlobals(void)
{
	byte i;

	bzero(&_gs, sizeof(globalStruct));
	for (i=0; i<NUM_PRE_DEFINED_ALIASES; i++)
	{
		_gs._preDefinedAliases[i] = ALIAS_UNUSED;       // needed for can routines....in the future, the system can have aliases too!
	}
	for (i=0; i<NUM_USER_DEFINED_ALIASES; i++)
	{
		_gs._userDefinedAliases[i] = ALIAS_UNUSED;  // needed for can routines....in the future, the system can have aliases too!
	}

	// Karl's debug request ...

	_gs._flasher.lightSel = DEFAULT_FLASHER_LIGHT_SEL;
	setFlasherVarPtr(DEFAULT_FLASHER_VAR_SEL);
	_gs._selectedArcPlane = ARC_PLANE_NONE;
	initializeInkjetStruct();
	initializeSpindleStruct();
	initializeLaserStruct();

	_gs._preCannedPackets[PRECANNED_LASER].msgType      = CAN_WRITE;
	_gs._preCannedPackets[PRECANNED_LASER].msgId        = CAN_MSG_SET_POWER_LEVEL_ONLY;
	_gs._preCannedPackets[PRECANNED_LASER].immediate    = TRUE;
	_gs._preCannedPackets[PRECANNED_LASER].page         = NO_PAGE;
	canFillOutCanStruct(&_gs._preCannedPackets[PRECANNED_LASER], CANBUS_FORMAT_V1); //LEGACY

	_gs._preCannedPackets[PRECANNED_STEP].msgType       = CAN_WRITE;
	_gs._preCannedPackets[PRECANNED_STEP].msgId         = CAN_MSG_STEP_MOTOR;
	_gs._preCannedPackets[PRECANNED_STEP].immediate     = TRUE;
	_gs._preCannedPackets[PRECANNED_STEP].page          = NO_PAGE;
	canFillOutCanStruct(&_gs._preCannedPackets[PRECANNED_STEP], CANBUS_FORMAT_V1);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef USE_AB_ENCODER
void timerInitEncoderAB(boolean reverseDirection)
{

	initClkAndResetAPB1(RCC_APB1Periph_TIM5);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	TIM5->PSC = 0;
	TIM5->ARR = 0xffffffff;
	TIM5->CNT = 0;

	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICStructInit(&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = 0x05;

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM5, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM5, &TIM_ICInitStruct);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI2, (reverseDirection ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling), TIM_ICPolarity_Rising); // TIM_EncoderMode_TI2 == TI1-edge, TI2-level
	TIM_Cmd(TIM5, ENABLE);
}
//
//void Init_ADC(void)
//{
//	ADC_Common_TypeDef  *tmpADC_Common = ADC_BASE;
//	
//	/* Set the ADC clock prescaler */
//	tmpADC_Common->CCR &= ~(ADC_CCR_ADCPRE);
//	tmpADC_Common->CCR |=  ADC_Prescaler_Div2;
//	
//	/* Set ADC scan mode */
//	ADC2->CR1 &= ~(ADC_CR1_SCAN);
//	ADC2->CR1 |=  (ADC_CR1_SCAN) << 8U;
//	
//	
//	/* Set ADC resolution */
//	ADC2->CR1 &= ~(ADC_CR1_RES);
//	ADC2->CR1 |=  ADC_Resolution_12b;
//  
//	/* Set ADC data alignment */
//	ADC2->CR2 &= ~(ADC_CR2_ALIGN);
//	ADC2->CR2 |= ADC_DataAlign_Right;
//	
//	/* Reset the external trigger */
//	ADC2->CR2 &= ~(ADC_CR2_EXTSEL);
//	ADC2->CR2 &= ~(ADC_CR2_EXTEN);
//	
//	/* Enable or disable ADC continuous conversion mode */
//	ADC2->CR2 &= ~(ADC_CR2_CONT);
//	ADC2->CR2 |= ADC_CR2_CONT << 1U; //not sure about this 
//	
//    /* Disable the selected ADC regular discontinuous mode */
//	ADC2->CR1 &= ~(ADC_CR1_DISCEN); //disable
//	
//	/* Set ADC number of conversion */
//	ADC2->SQR1 &= ~(ADC_SQR1_L);
//	ADC2->SQR1 |=  (((6) - (uint8_t)1U) << 20U);
//  
//	/* Enable or disable ADC DMA continuous request */
//	ADC2->CR2 &= ~(ADC_CR2_DDS);
//	ADC2->CR2 |= (ENABLE << 9U);
//  
//	/* Enable or disable ADC end of conversion selection */
//	ADC2->CR2 &= ~(ADC_CR2_EOCS);
//	ADC2->CR2 |= (0x00000001U << 10U);
//	
//
//	//		{ADC_CHANNEL_3, 1, ADC2_03_PA3, 0, ConvertionTable},
//	//		{ADC_CHANNEL_4, 2, ADC2_04_PA4, 0, ConvertionTable},
//	//		{ADC_CHANNEL_5, 3, ADC2_05_PA5, 0, ConvertionTable},
//	//		{ADC_CHANNEL_6, 4, ADC2_06_PA6, 0, ConvertionTable},
//	//		{ADC_CHANNEL_9, 5, ADC2_09_PB1, 0, ConvertionTable},
//	//		{ADC_CHANNEL_15, 6, ADC2_15_PC5, 0, ConvertionTable},
//	pinInit(ADC2_03_PA3);
//	pinInit(ADC2_04_PA4);
//	pinInit(ADC2_05_PA5);
//	pinInit(ADC2_06_PA6);
//	pinInit(ADC2_09_PB1);
//	pinInit(ADC2_15_PC5);
//	//configure the pins and their order of execution
//	ADC_RegularChannelConfig(ADC2, 3, 1, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC2, 4, 2, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC2, 5, 3, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC2, 6, 4, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC2, 9, 5, ADC_SampleTime_480Cycles);
//	ADC_RegularChannelConfig(ADC2, 15, 6, ADC_SampleTime_480Cycles);
//	
//}
//void Start_ADC(void)
//{
//	RawADCDataBuffer;
//}


#endif //USE_AB_ENCODER

////////////////////////////////////////////////////////////////////////////////

