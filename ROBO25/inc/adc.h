#pragma once
#include "main.h"
#include "adc_table.h"

#define ANALOG_VOLTAGE  (0x34cd)    // 3.3V in a 4.12 format
#define VDDA_FLOAT		3.3f

#define ADC_NUM_SAMPLES                 10  // 10 values saved; toss high and low to get average
#define ADC_SHIFT_FOR_AVG               3
extern uint16_t RawADCDataBuffer[];
extern float laserTemperature;

typedef enum {
	CONV_UNDEFINED = 0,
	CONV_NONE = 1, // return input
	CONV_NEAREST = 2,
	CONV_INTERPOLATE = 3,
} adcConv_t;

typedef struct {
	uint16_t    adcRaw; // last raw value read from ADC
	int16_t     convRaw; // converted value from last rawValue read
	int16_t     adcAvg; // average of the last 10 read (or converted reads) after tossing high and low
	float		convAvg; // converted value using the rawAvg
	int16_t     sampleHistory[ADC_NUM_SAMPLES]; // last N reads from ADC
	uint8_t     sampleIndex; // index of last valid value in the history
	uint8_t     inputChannel; // actual adc channel input selection
	adcConv_t   conversionType;
} adcStruct;

extern adcStruct ADC_Channel[ADC_CHANNEL_NUM];

void Init_ADC();
void Start_ADC(void);
void ProcessRawADC_Data(void);
extern float ScaledADCData[]; //converted adc buffer values
