#pragma once
#include <stdint.h>
#include "main.h"
typedef enum {
	FUNC_INT,
	FUNC_INT16,
	FUNC_INT32,
	FUNC_FLOAT,
	FUNC_FLOAT_QUEVAR,
	FUNC_HEX8,
	FUNC_HEX16,
	FUNC_HEX32,
	FUNC_TITLE,
	FUNC_ASCII,
	FUNC_BOOLEAN,
	FUNC_BAR_STATUS,
	FUNC_BAR_DUTTY,
	FUNC_BAR_FLOAT,
	FUNC_MEMHEX_64,
	FUNC_MEMHEX_128,
	FUNC_MEMHEX_256,
	FUNC_MEMDUMPASCII,
	FUNC_MEMDUMPHEX,
	FUNC_MEMDISPASCII,
	FUNC_MEMDISPASCIIHEX,
	FUNC_ASCI_SOAP,
	FUNC_MOTOR_STATUS
}DISPLAYFUNCTYPE;

typedef struct {
	uint32_t 		VariablePointer;//points to the variable that holds the data we want to display
	char 			Label[9];
	DISPLAYFUNCTYPE FuncType;//function type how we show the data, i.e. hex, int, float
	uint8_t			Color_1; // this is for Label or background in Bar.
	uint8_t			Color_2; // this is for Value or progress bar in Bar.
	uint32_t 		Offset; //for FUNC_MEMDUMPASCII if string is array variable(char a[]), it would be 1 otherwise 0. it is only for memory ascii function
							//for FUNC_MEMDUMPHEX it means offset.
} LcdVariableInfo;

typedef struct {
	uint32_t MotorPosition; //points to the motor actual position  (uint32_t)&Motors[M_X].POSITION
	pinType GrossMomePin; //used to find the gross positions
	pinType FineHome; //pin connected to the fine home position	
}MotorStatusStruct;

extern uint32_t Head11_Temperature;
extern uint32_t Head11_HTRDuty;
extern uint32_t Head11_FanDuty;
extern uint32_t Head11_Spare;

extern uint32_t HeartBeat;
extern LcdVariableInfo SecsVarsTable[];
extern LcdVariableInfo SerialRxBufferTable[];
extern LcdVariableInfo CanRxBufferTable[];
extern LcdVariableInfo CanTxBufferTable[];
extern LcdVariableInfo SecsStringTable[];
extern LcdVariableInfo LcdVarsTable[];
extern LcdVariableInfo TaskTimeTable1[];
extern LcdVariableInfo TaskTimeTable2[];
extern LcdVariableInfo SoapString1[];
extern LcdVariableInfo GcodeArguments[];
extern LcdVariableInfo ADCValueTable[];
extern LcdVariableInfo BarValueTable[];
extern LcdVariableInfo SoapStringTable[];
extern LcdVariableInfo FaultValueTable[];
extern LcdVariableInfo UsbGcodeArguments[];
extern LcdVariableInfo CMDQueValues[];
//#define VARSNUM sizeof(LcdVarsTable) / sizeof(LcdVariableInfo)
