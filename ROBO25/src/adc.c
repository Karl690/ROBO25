#include "main.h"
#include "adc.h"
#include "adc_table.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"

uint16_t   RawADCDataBuffer[ADC_CHANNEL_NUM] = { 0};

float ScaledADCData[ADC_CHANNEL_NUM] = { 0 }; //converted adc buffer values
uint16_t ADC_Work_Channel_Index = 0; //used to walk thru the channels and update the working variables.
adcStruct ADC_Channel[ADC_CHANNEL_NUM];
adcStruct *ADC_Work_Channel;
float laserTemperature = 0;

void Init_ADC(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	DMA_InitTypeDef dma; //dma2/stream0/channel0
	dma.DMA_Channel = DMA_Channel_1;
	dma.DMA_Memory0BaseAddr = (uint32_t) &RawADCDataBuffer[0];
	dma.DMA_PeripheralBaseAddr = (uint32_t)&(ADC2->DR);
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = ADC_CHANNEL_NUM;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_Low;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2, &dma);
	DMA_Cmd(DMA2_Stream2, ENABLE); //dMA2_Stream0 enable
	
	 //ADC common init
    ADC_CommonInitTypeDef adcCommon;
	adcCommon.ADC_Mode = ADC_Mode_Independent;
	adcCommon.ADC_Prescaler = ADC_Prescaler_Div2;
	adcCommon.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	adcCommon.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	
	ADC_CommonInit(&adcCommon);

	//setup adc1: in1,2,3,8,9,15
	ADC_InitTypeDef adc;
	ADC_DeInit(); //set adc to default state
	
	adc.ADC_DataAlign = ADC_DataAlign_Right; //mask 0b 00001111 11111111
	adc.ADC_Resolution = ADC_Resolution_12b; //12 bit = 4096
	adc.ADC_ContinuousConvMode = ENABLE; //continuous: constantly converting data - can always read register
	adc.ADC_ExternalTrigConv = DISABLE; //ADC_ExternalTrigConv_T1_CC1;//external trigger conversion (?)
	
	adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adc.ADC_NbrOfConversion = ADC_CHANNEL_NUM;
	adc.ADC_ScanConvMode = ENABLE; //single/multichannel
	ADC_Init(ADC2, &adc);
	
	/////////////////////////////////////////
	for (uint8_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) {
		ADC_RegularChannelConfig(ADC2, AdcChannelTable[ch].Channel, ch + 1, ADC_SampleTime_480Cycles); 
		//initialize pin for channel
		pinInit(AdcChannelTable[ch].Pin);
	}
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);

	ADC_DMACmd(ADC2, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	///////////////////////
}
void Start_ADC(void)
{
	// Start ADC conversion
	ADC_SoftwareStartConv(ADC2);
}


void SmoothDataUsingOlympicVotingAverage(void)
{
	ADC_Work_Channel = &ADC_Channel[ADC_Work_Channel_Index];
	ADC_Work_Channel->adcRaw = RawADCDataBuffer[ADC_Work_Channel_Index]; //update last reading
	//now we need to plug into the 10 reading buffer for smoothing.
	ADC_Work_Channel->sampleIndex++; //point to next place to enter the data in smoothing array
	if (ADC_Work_Channel->sampleIndex > 9)ADC_Work_Channel->sampleIndex = 0; //limit to 10 entries
	ADC_Work_Channel->sampleHistory[ADC_Work_Channel->sampleIndex] = ADC_Work_Channel->adcRaw; //plug in the latest reading.
	//at this point, channel.sampleHistory has the raw data to be smoothed.

	{
		// history buffer is full, so enough good adc values to proceed

		// this code does olympic voting (toss high and low and then average the rest)
		// the ADC_NUM_SAMPLES must be equal to (2^n) + 2 and ADC_SHIFT_FOR_AVG
		// must equal "n", as the code will shift to get the average instead of divide. set temporary
		// variables to record highest and lowest values as each of the ADC_NUM_SAMPLES is inspected
		// at the same time, record the sum of all ADC_NUM_SAMPLES samples.  when done looking at all values,
		// subtract the high and low from the sum and then calculate the average of the remaining sum.
		int32_t sum, raw, low, high, i;
		low = 0x7fffffff; // MAXINT
		high = 0x80000000; // MININT
		sum = 0;
		for (i = 0; i < ADC_NUM_SAMPLES; i++)
		{
			raw = ADC_Work_Channel->sampleHistory[i];
			sum += raw; // keep running total
			if (raw < low) low = raw; // update the lowest reading
			if (raw > high)high = raw; // update the highest reading
		}
		sum -= (low + high); // sum is now the total of the middle N values

		//next we will shift by n to effect a divide by 2^n to get the average of the 2^n remaining samples
		ADC_Work_Channel->adcAvg = (sum >> ADC_SHIFT_FOR_AVG); // update the RAW average
		//ADC_Work_Channel->convAvg = ScaledADCData[ADC_Work_Channel_Index] = (float)(((float)ADC_Work_Channel->adcAvg * 3.3) / 4095);
		ADC_Work_Channel->convAvg = convertRtdDataFromRawADCValue(AdcChannelTable[ADC_Work_Channel_Index].ConvertionTable, ADC_Work_Channel->adcAvg);
	}
	// setup next conversion so data will be ready for the next call in ~10ms
	if (ADC_Work_Channel_Index == 4)laserTemperature = ADC_Work_Channel->convAvg;
	ADC_Work_Channel_Index++;
	if (ADC_Work_Channel_Index >= ADC_CHANNEL_NUM) ADC_Work_Channel_Index = 0; //keep in range

}

void ProcessRawADC_Data(void)
{
	SmoothDataUsingOlympicVotingAverage();
}