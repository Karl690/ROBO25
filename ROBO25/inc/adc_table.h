#pragma once
#include <stdint.h>
#include "main.h"
typedef struct {
	uint16_t    adcRaw; // expected adcValue for specified temp
	temp_t      value; // if temperature (s10.5 format - 1/32 degree) at specified adcValue; position, just value, etc
} AdcTableStruct;

typedef struct {
	uint32_t Channel;
	pinType Pin;
	uint16_t Prioity;
	const AdcTableStruct* ConvertionTable;
}ADC_ChannelDef;

#define ADC_CHANNEL_NUM 6

extern ADC_ChannelDef AdcChannelTable[ADC_CHANNEL_NUM];

float convertRtdDataFromRawADCValue(const AdcTableStruct* adcTable, uint16_t raw);