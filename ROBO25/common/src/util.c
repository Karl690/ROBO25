////////////////////////////////////////////////////////////////////////////////
//
// File:    init.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: various initialization functions
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "util.h"

////////////////////////////////////////////////////////////////////////////////

void memZero(byte *addr, uint32_t qty)
{
	memset(addr, 0, qty);
}

////////////////////////////////////////////////////////////////////////////////

void deinitClkAndResetAPB1(uint32_t periph)
{
	RCC->APB1ENR  &= ~(periph); // disable clock for peripheral
	RCC->APB1RSTR |=  (periph); // set reset for peripheral
}

////////////////////////////////////////////////////////////////////////////////

void deinitClkAndResetAPB2(uint32_t periph)
{
	RCC->APB2ENR  &= ~(periph); // disable clock for peripheral
	RCC->APB2RSTR |=  (periph); // set reset for peripheral
}

////////////////////////////////////////////////////////////////////////////////

void initClkAndResetAPB1(uint32_t periph)
{
	RCC->APB1ENR  &= ~(periph); // disable clock for peripheral
	RCC->APB1RSTR |=  (periph); // set reset for peripheral
	RCC->APB1ENR  |=  (periph); // enable clock for peripheral
	RCC->APB1RSTR &= ~(periph); // remove reset from peripheral
}

////////////////////////////////////////////////////////////////////////////////

void initClkAndResetAPB2(uint32_t periph)
{
	RCC->APB2ENR  &= ~(periph); // disable clock for peripheral
	RCC->APB2RSTR |=  (periph); // set reset for peripheral
	RCC->APB2ENR  |=  (periph); // enable clock for peripheral
	RCC->APB2RSTR &= ~(periph); // remove reset from peripheral
}

////////////////////////////////////////////////////////////////////////////////
#ifdef STM32F4XX_HYREL

void initClkAndResetAHB1(uint32_t periph)
{
	RCC->AHB1ENR  &= ~(periph); // disable clock for peripheral
	RCC->AHB1RSTR |=  (periph); // set reset for peripheral
	RCC->AHB1ENR  |=  (periph); // enable clock for peripheral
	RCC->AHB1RSTR &= ~(periph); // remove reset from peripheral
}

////////////////////////////////////////////////////////////////////////////////

void initClkAndResetAHB2(uint32_t periph)
{
	RCC->AHB2ENR  &= ~(periph); // disable clock for peripheral
	RCC->AHB2RSTR |=  (periph); // set reset for peripheral
	RCC->AHB2ENR  |=  (periph); // enable clock for peripheral
	RCC->AHB2RSTR &= ~(periph); // remove reset from peripheral
}

////////////////////////////////////////////////////////////////////////////////

void initClkAndResetAHB3(uint32_t periph)
{
	RCC->AHB3ENR  &= ~(periph); // disable clock for peripheral
	RCC->AHB3RSTR |=  (periph); // set reset for peripheral
	RCC->AHB3ENR  |=  (periph); // enable clock for peripheral
	RCC->AHB3RSTR &= ~(periph); // remove reset from peripheral
}

#endif //STM32F4XX_HYREL

////////////////////////////////////////////////////////////////////////////////

void interruptSetupAndDisable(uint8_t channel, uint8_t priority)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel                      = channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                   = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}

////////////////////////////////////////////////////////////////////////////////

void interruptSetupAndEnable(uint8_t channel, uint8_t priority)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel                      = channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                   = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//use 32-bit flavors for all 8, 16, and 32 bit data types.   NO useful savings adding more methods

int32_t imin(int32_t a, int32_t b)
{
	if (a < b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

int32_t imax(int32_t a, int32_t b)
{
	if (a > b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

int32_t iabs(int32_t a)
{
	if (a < 0)
		return(-a);
	return(a);
}

////////////////////////////////////////////////////////////////////////////////

int32_t iFitWithinRange(int32_t value, int32_t low, int32_t high)
{
	if (value <= low)
		return(low);
	else if (value >= high)
		return(high);
	else
		return(value);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t umin(uint32_t a, uint32_t b)
{
	if (a < b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t umax(uint32_t a, uint32_t b)
{
	if (a > b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

uint32_t uFitWithinRange(uint32_t value, uint32_t low, uint32_t high)
{
	if (value <= low)
		return(low);
	else if (value >= high)
		return(high);
	else
		return(value);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int64_t imin64(int64_t a, int64_t b)
{
	if (a < b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

int64_t imax64(int64_t a, int64_t b)
{
	if (a > b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

int64_t iabs64(int64_t a)
{
	if (a < 0)
		return(-a);
	return(a);
}

////////////////////////////////////////////////////////////////////////////////

int64_t iFitWithinRange64(int64_t value, int64_t low, int64_t high)
{
	if (value <= low)
		return(low);
	else if (value >= high)
		return(high);
	else
		return(value);
}

////////////////////////////////////////////////////////////////////////////////

uint64_t umin64(uint64_t a, uint64_t b)
{
	if (a < b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

uint64_t umax64(uint64_t a, uint64_t b)
{
	if (a > b)
		return(a);
	else
		return(b);
}

////////////////////////////////////////////////////////////////////////////////

uint64_t uFitWithinRange64(uint64_t value, uint64_t low, uint64_t high)
{
	if (value <= low)
		return(low);
	else if (value >= high)
		return(high);
	else
		return(value);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

float fFitWithinRange(float value, float low, float high)
{
	if (value <= low)
		return(low);
	else if (value >= high)
		return(high);
	else
		return(value);

}////////////////////////////////////////////////////////////////////////////////

double dFitWithinRange(double value, double low, double high)
{
	if (value <= low)
		return(low);
	else if (value >= high)
		return(high);
	else
		return(value);

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifdef DELAY_TIMER
uint64_t tickCount = 0;
void delayUsec(uint32_t us)
{	// timer MUST already be set up and free running .... every 1 tick is 1 us
//	DELAY_TIMER->EGR |= TIM_EGR_UG;	//reset CNT/PS_CNT
//	DELAY_TIMER->CNT = 0;	// kill a smidge of time to let UG take place; otherwise check may be screwed up!
//	while (DELAY_TIMER->CNT < us);
	for (int i = 0; i < us; i++) ;
}

////////////////////////////////////////////////////////////////////////////////

void delayMsec(uint32_t ms)
{
	for (int i=0; i<ms; i++)
	{
		delayUsec(1000);
	}
}

////////////////////////////////////////////////////////////////////////////////

void delaySec(uint32_t sec)
{
	for (int i=0; i<sec; i++)
	{
		delayMsec(1000);
	}
}

#endif //DELAY_TIMER

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//#if defined(ADD_ON_SPI_DISPLAY) || defined(ADD_ON_CLOSED_LOOP_STEPPER)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
uint32_t  __attribute__((always_inline)) interruptsOff(void)
{
		uint32_t irq_disabled = __get_PRIMASK();
		if (!irq_disabled)
		{
				__disable_irq();
#ifdef GB_DEBUG_IRQ_OFFON_PIN
				pinSet(GB_DEBUG_IRQ_OFFON_PIN);
#endif //GB_DEBUG_IRQ_OFFON_PIN
		}
		return(irq_disabled);
}
#pragma GCC diagnostic pop

////////////////////////////////////////////////////////////////////////////////

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
void  __attribute__((always_inline)) interruptsOn(uint32_t irq_disabled)
{
		if (!irq_disabled)
		{
#ifdef GB_DEBUG_IRQ_OFFON_PIN
		pinClear(GB_DEBUG_IRQ_OFFON_PIN);
#endif //GB_DEBUG_IRQ_OFFON_PIN
				__enable_irq();
		}
}
#pragma GCC diagnostic pop

//#endif //defined(ADD_ON_SPI_DISPLAY) || defined(ADD_ON_CLOSED_LOOP_STEPPER)

////////////////////////////////////////////////////////////////////////////////

//#ifdef ADD_ON_SPI_DISPLAY

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#ifdef STM32F4XX
float fpu_sqrtf(float op)
{
		__asm("  vsqrt.f32 s15, s0");
		//                      "  vldr      s15, [r7, #4]\n"
		//                      "  vmov.f32  s0, s15");
}
#else
#include "math.h"
float fpu_sqrtf(float op)
{
	return(sqrtf(op));
}
#endif
#pragma GCC diagnostic pop
//#endif //ADD_ON_SPI_DISPLAY

////////////////////////////////////////////////////////////////////////////////

char *getPidMethodStr(char *s, byte method)
{
	switch(method)
	{
	case CONTROL_OPEN_LOOP:		strcpy(s, "OPL"); break;
	case CONTROL_BTT_PID:		strcpy(s, "BTT"); break;
	case CONTROL_STP_PID:		strcpy(s, "STP"); break;
	case CONTROL_ANG_PID:		strcpy(s, "ANG"); break;
	case CONTROL_RPM_PID:		strcpy(s, "RPM"); break;
	case CONTROL_EXP_PID:		strcpy(s, "EXP"); break;
	case CONTROL_SP0_PID:		strcpy(s, "SP0"); break;
	case CONTROL_SP1_PID:		strcpy(s, "SP1"); break;
	default:					strcpy(s, "HUH"); break;
	}
	return(s);
}

////////////////////////////////////////////////////////////////////////////////

