#include "adc_table.h"
#include "pinout.h"

AdcTableStruct const HeadPositionTable[] __attribute__((aligned(4))) =
{ // from the Hot Head Resistor Value and 12-bit ADC Value spec
		// 27 entries, 4 bytes each, 108 total bytes
		// (uint16_t)adcVal, (uint16)devPos   (MUST be in order with increasing adcVals)
		{ 108, 91 }, // max adc value for hotbed1 position
		{ 294, 26 }, // max adc value for yoke 2 aux2
		{ 452, 16 }, // max adc value for yoke 1 aux2
		{ 635, 21 }, // max adc value for yoke 2 hot head 1
		{ 804, 11 }, // max adc value for yoke 1 hot head 1
		{ 943, 22 }, // max adc value for yoke 2 hot head 2
		{ 1082, 12 }, // max adc value for yoke 1 hot head 2
		{ 1228, 23 }, // max adc value for yoke 2 hot head 3
		{ 1389, 13 }, // max adc value for yoke 1 hot head 3
		{ 1563, 24 }, // max adc value for yoke 2 hot head 4
		{ 1751, 14 }, // max adc value for yoke 1 hot head 4
		{ 1946, 25 }, // max adc value for yoke 2 aux1
		{ 2140, 15 }, // max adc value for yoke 1 aux1
		{ 2345, 92 }, // max adc value for hotbed2 position
		{ 2544, 46 }, // max adc value for yoke 4 aux2
		{ 2723, 36 }, // max adc value for yoke 3 aux2
		{ 2901, 41 }, // max adc value for yoke 4 hot head 1
		{ 3065, 31 }, // max adc value for yoke 3 hot head 1
		{ 3200, 42 }, // max  value for yoke 4 hot head 2
		{ 3317, 32 }, // max adc value for yoke 3 hot head 2
		{ 3439, 43 }, // max adc value for yoke 4 hot head 3
		{ 3557, 33 }, // max adc value for yoke 3 hot head 3
		{ 3667, 44 }, // max adc value for yoke 4 hot head 4
		{ 3780, 34 },
	// max adc value for yoke 3 hot head 4
		{ 3906, 45 }, // max adc value for yoke 4 aux1
		{ 4034, 35 }, // max adc value for yoke 3 aux1
		{ MAX_ADC12, HH_POSITION_UNPLUGGED },
	// max adc value an unplugged hothead
};


AdcTableStruct const RtdTable_1M[] __attribute__((aligned(4))) =
{
	// 1M RTD temperature conversion table
		// 18 entries, 4 bytes each, 72 total bytes
		// (uint16_t)adcVal, (uint16_t)(tempC * 32)    (MUST be in order with increasing adcVals)
		{ 0, (427 << TEMP_FRAC_BITS) },
	// fake datapoint to have a 0 adc value //   13653 },
		{ 400, (260 << TEMP_FRAC_BITS) },
	//   8320 },
		{ 500, (250 << TEMP_FRAC_BITS) },
	//   8000 },
		{ 540, (247 << TEMP_FRAC_BITS) },
	//   7896 },
		{ 630, (240 << TEMP_FRAC_BITS) },
	//   7656 },
		{ 745, (230 << TEMP_FRAC_BITS) },
	//   7360 },
		{ 918, (218 << TEMP_FRAC_BITS) },
	//   6960 },
		{ 1003, (213 << TEMP_FRAC_BITS) },
	//   6824 },
		{ 1242, (201 << TEMP_FRAC_BITS) },
	//   6424 },
		{ 1613, (185 << TEMP_FRAC_BITS) },
	//   5904 },
		{ 2502, (152 << TEMP_FRAC_BITS) },
	//   4864 },
		{ 3108, (129 << TEMP_FRAC_BITS) },
	//   4136 },
		{ 3550, (106 << TEMP_FRAC_BITS) },
	//   3376 },
		{ 3900, (71 << TEMP_FRAC_BITS) },
	//   2272 },
		{ 3974, (54 << TEMP_FRAC_BITS) },
	//   1736 },
		{ 4016, (33 << TEMP_FRAC_BITS) },
	//   1040 },
		{ 4080, (3 << TEMP_FRAC_BITS) },
	//    100 },
		{ MAX_ADC12, (0 << TEMP_FRAC_BITS) },
	// fake data point of 0C
};

AdcTableStruct const RtdTable_50K[] __attribute__((aligned(4))) =
{
	// 50K RTD temperature conversion table
		// 17 entries, 4 bytes each, 68 total bytes
		// (uint16_t)adcVal, (uint16_t)(tempC * 32)    (MUST be in order with increasing adcVals)
		{ 0, (427 << TEMP_FRAC_BITS) },
	// fake datapoint to have a 0 adc value
		{ 42, (271 << TEMP_FRAC_BITS) },
	{ 597, (131 << TEMP_FRAC_BITS) },
	{ 667, (126 << TEMP_FRAC_BITS) },
	{ 833, (116 << TEMP_FRAC_BITS) },
	{ 1039, (106 << TEMP_FRAC_BITS) },
	{ 1288, (96 << TEMP_FRAC_BITS) },
	{ 1580, (86 << TEMP_FRAC_BITS) },
	{ 1913, (76 << TEMP_FRAC_BITS) },
	{ 2270, (66 << TEMP_FRAC_BITS) },
	{ 2633, (56 << TEMP_FRAC_BITS) },
	{ 2977, (46 << TEMP_FRAC_BITS) },
	{ 3281, (36 << TEMP_FRAC_BITS) },
	{ 3530, (26 << TEMP_FRAC_BITS) },
	{ 3721, (16 << TEMP_FRAC_BITS) },
	{ 3848, (7 << TEMP_FRAC_BITS) },
	{ MAX_ADC12, (0 << TEMP_FRAC_BITS) },
	// fake data point of 0C
};

AdcTableStruct const RtdTable_1K[] __attribute__((aligned(4))) =
{ // 1K RTD -- based on datasheet
		// 20 entries; 4 bytes each; 80 bytes total
		// (uint16_t)adcVal, (uint16_t)(tempC * 32)    (MUST be in order with increasing adcVals)
		{ 0, (-242 << TEMP_FRAC_BITS) }, // extrapolated to get -242 as 0 ohms (yielding 0 adc value with 2K divider)
		{ 347, (-200 << TEMP_FRAC_BITS) }, // 1K mfg table starts at -200
		{ 678, (-150 << TEMP_FRAC_BITS) },
	{ 948, (-100 << TEMP_FRAC_BITS) },
	{ 1173, (-50 << TEMP_FRAC_BITS) },
	{ 1365, (0 << TEMP_FRAC_BITS) },
	{ 1531, (50 << TEMP_FRAC_BITS) },
	{ 1676, (100 << TEMP_FRAC_BITS) },
	{ 1803, (150 << TEMP_FRAC_BITS) },
	{ 1916, (200 << TEMP_FRAC_BITS) },
	{ 2017, (250 << TEMP_FRAC_BITS) },
	{ 2107, (300 << TEMP_FRAC_BITS) },
	{ 2189, (350 << TEMP_FRAC_BITS) },
	{ 2263, (400 << TEMP_FRAC_BITS) },
	{ 2330, (450 << TEMP_FRAC_BITS) },
	{ 2392, (500 << TEMP_FRAC_BITS) },
	{ 2448, (550 << TEMP_FRAC_BITS) },
	{ 2500, (600 << TEMP_FRAC_BITS) },
	// 1K mfg tables ends at 600
	{ 2809, (999 << TEMP_FRAC_BITS) }, // extrapolated to get 999 as 4370 ohms (yielding 2809 adc value with 2K divider)
	{ MAX_ADC12, MAX_TEMP },
	// in reality, to get to MAX_ADC, would take about 10 billion degrees.
};

////////////////////////////////////////////////////////////////////////////////


AdcTableStruct const RtdTable_100[] __attribute__((aligned(4))) =
{
	// 100 RTD -- based on datasheet
		// 14 entries; 4 bytes each; 56 bytes total
		// (uint16_t)adcVal, (uint16_t)(tempC * 32)    (MUST be in order with increasing adcVals)
		{ 0, (-260 << TEMP_FRAC_BITS) },
	// extrapolated to get -242 as 0 ohms (yielding 0 adc value with 2K divider)
		{ 159, (-50 << TEMP_FRAC_BITS) },
	// 1K mfg table starts at -200
		{ 195, (0 << TEMP_FRAC_BITS) },
	{ 209, (20 << TEMP_FRAC_BITS) },
	{ 223, (40 << TEMP_FRAC_BITS) },
	{ 237, (60 << TEMP_FRAC_BITS) },
	{ 251, (80 << TEMP_FRAC_BITS) },
	{ 265, (100 << TEMP_FRAC_BITS) },
	{ 299, (150 << TEMP_FRAC_BITS) },
	{ 333, (200 << TEMP_FRAC_BITS) },
	{ 366, (250 << TEMP_FRAC_BITS) },
	{ 522, (500 << TEMP_FRAC_BITS) },
	{ 799, (999 << TEMP_FRAC_BITS) },
	// extrapolated to get 999 as 4370 ohms (yielding 2809 adc value with 2K divider)
	{ MAX_ADC12, MAX_TEMP },
	// in reality, to get to MAX_ADC, would take about 10 billion degrees.
};


ADC_ChannelDef AdcChannelTable[ADC_CHANNEL_NUM] = {
	{ ADC_Channel_3, ADC2_03_PA3, 0, &RtdTable_1K[0]},
	{ ADC_Channel_4, ADC2_04_PA4, 0, &RtdTable_1K[0]},
	{ ADC_Channel_5, ADC2_05_PA5, 0, &RtdTable_1K[0]},
	{ ADC_Channel_6, ADC2_06_PA6, 0, &RtdTable_1K[0]},
	{ ADC_Channel_9, ADC2_09_PB1, 0, &RtdTable_1K[0] },
	{ ADC_Channel_15, ADC2_15_PC5, 0, &RtdTable_1K[0]},
};

/*this method will use a lookup table to convert the raw adc value to a useful temperature
 *it first finds the RAW value range in the table by looking for the first entry that is NOT
 *greater than the raw value
 *then it mulitplies the raw value by the coefficient from the table to get the ACTUAL temperature.
 *this assumes that each table starts with RAW data of 0
 *and ends with MAX_ADC12 value
 **/
float convertRtdDataFromRawADCValue(const AdcTableStruct* adcTable, uint16_t rawData)
{
	uint16_t Index = 0; //need a pointer to know where in the table we are working
	int tablesize = sizeof adcTable;
	float conversionValue = 0;
	if (rawData == 0)return conversionValue;//IF raw is 0 then we are at 0
	//need check for max a2d value also, to prevent bogus information.
	//now we will walk thru the table to find the index to the first RAW data that is >= rawData
	for (Index = 0; Index < 32; Index++)  //table size did not work, it gives the 4 bytes of the entry
	{
		if (adcTable[Index].adcRaw >= rawData) break;//found our index
	}
	//check for 3 unique cases
	//if rawdata and table.rawdata match, then the temperature also matches
	//if rawdata is 0
	//if rawdata is == max_adc12 value
	//if any  of these are true,then we simply return the adctable.value
	if ((adcTable[Index].adcRaw == rawData) || (rawData==0)||(adcTable[Index].adcRaw >= MAX_ADC12))
	{//lucky, we hit the value exaclty, so we are actually exaclty the table entry in size
		conversionValue = (float)adcTable[Index].value;
		return conversionValue;
	}
	//ok at this point were have a index to the first RAW value that is greater than the actual raw
	//dat a value, we need to make sure our rawdata value falls between the 2 points in the table
	if (Index > 0) Index--; //if we are already at zero, we will use this pointer, 
	float DeltaTemperatureValue = adcTable[Index + 1].value - adcTable[Index].value;
	float DeltaRawData = adcTable[Index + 1].adcRaw - adcTable[Index].adcRaw;
	float conversionCoeffecient = DeltaTemperatureValue / DeltaRawData;
	//now we have the coeffecient between the 2 points in the table
	//next we will get the offset value between the first point and the raw data
	int rawRemainder = rawData-adcTable[Index].adcRaw;
	float offsetTemperatureValue = rawRemainder*conversionCoeffecient;
	conversionValue = offsetTemperatureValue + adcTable[Index].value;
	conversionValue /= TEMP_SCALEF; //divide by the temp bits to scale to degrees c.
	return conversionValue;
	
//	uint8_t leftIndex = 0, rightIndex = 0;
//	while (adcTable[rightIndex].adcRaw != MAX_ADC12)
//	{
//		if (raw <= adcTable[rightIndex].adcRaw)	break;//found the index, so lets go convert
//		rightIndex++;
//	}
//	if (rightIndex == 0) return adcTable[rightIndex].adcRaw;
//	else if (rightIndex > 0) leftIndex = rightIndex - 1;
//	if (adcTable[rightIndex].adcRaw == MAX_ADC12) return adcTable[leftIndex].value;
//	
//	float a = (adcTable[rightIndex].value - adcTable[leftIndex].value) / (float)(adcTable[rightIndex].adcRaw - adcTable[leftIndex].adcRaw);
//	float y = a * (raw - adcTable[leftIndex].adcRaw) + adcTable[leftIndex].value;
//	return y;
}
