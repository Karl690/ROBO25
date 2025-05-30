#include "main.h"

#ifdef HYDRA_DIAGS

#include "GCode.h"
#include "Serial.h"
#include "Hydra_can.h"
#include "gpio.h"
#include "MotorDriver.h"
#include "HardwareInit.h"
#include "diags.h"

canDevNameStruct canDevName[] = {
		{ SYS_ID,  "J23" },
		{ CAN_MX, "canMX" },
		{ CAN_MY, "canMY" },
		{ CAN_MZ, "canMZ" },
		{ CAN_MA, "canMA" },
		{ CAN_MB, "canMB" },
		{ CAN_MC, "canMC" },
		{ CAN_PX, "canPX" },
		{ CAN_PY, "canPY" },
		{ CAN_PZ, "canPZ" },
		{ CAN_PA, "canPA" },
		{ CAN_PB, "canPB" },
		{ CAN_PC, "canPC" },
		{ CAN_UNKNOWN, "UNKNOWN" },
};
#define NUM_CAN_DIAG_DEVICES    (sizeof(canDevName) / sizeof(canDevNameStruct))

////////////////////////////////////////////////////////////////////////////////

diagAdcInfoStruct diagAdcInfo[] = {
		{ DIAG_ADC_NONE, ADC1, 0, PIN_UNDEFINED, ""},

		{ DIAG_ADC_LOCAL_PA3,   ADC1,   3,  PA3,    "ADC1_AIN3" },
		{ DIAG_ADC_LOCAL_PA6,   ADC1,   6,  PA6,    "ADC1_AIN6" },
		{ DIAG_ADC_LOCAL_PF7,   ADC3,   5,  PF7,    "ADC3_AIN5" },
		{ DIAG_ADC_LOCAL_PF8,   ADC3,   6,  PF8,    "ADC3_AIN6" },
		{ DIAG_ADC_LOCAL_PF9,   ADC3,   7,  PF9,    "ADC3_AIN7" },
		{ DIAG_ADC_LOCAL_PF10,  ADC3,   8,  PF10,   "ADC3_AIN8" },

		{ DIAG_ADC_CAN_RTD1,    ADC1,   1,  RTD1,   "ADC1_RTD1" },
		{ DIAG_ADC_CAN_RTD2,    ADC1,   0,  RTD2,   "ADC1_RTD2" },
		{ DIAG_ADC_CAN_PA2,     ADC1,   2,  HPA2,   "ADC1_PA2" },
		{ DIAG_ADC_CAN_PA3,     ADC1,   3,  HPA3,   "ADC1_PA3"},
};
#define NUM_DIAG_ADCS       (sizeof(diagAdcInfo) / sizeof(diagAdcInfoStruct))

////////////////////////////////////////////////////////////////////////////////

connectivityStruct connectivityList[] = {
		{CONNECTIVITY_NONE,  0, "BAD_JuJu", { 0,  PIN_UNDEFINED, ""},   {0, PIN_UNDEFINED,  ""}},
		{CONNECTIVITY_GEN_0, 0, "AINconns", { SYS_ID, PA4,    "R15:J17-9(BLU)"},    {SYS_ID, PA5,    "J18-9:R14"}}, //AIN conns
		{CONNECTIVITY_GEN_1, 0, "AINconns", { SYS_ID, PH8,    "J17-13(WHI)"},       {SYS_ID, PB1,    "J18-13"}},
		{CONNECTIVITY_GEN_2, 0, "AINconns", { SYS_ID, PH10,   "J17-14(GRN)"},       {SYS_ID, PH9,    "J17-15"}},
		{CONNECTIVITY_GEN_3, 0, "AINconns", { SYS_ID, PH3,    "J18-14(ORA)"},       {SYS_ID, PH4,    "J18-15"}},
		{CONNECTIVITY_GEN_4, 0, "AINconns", { SYS_ID, PH11,   "J17-16(PUR)"},       {SYS_ID, PB0,    "J18-16"}},
		{CONNECTIVITY_GEN_5, 0, "RTD_J19",  { SYS_ID, PA0,    "J19-10(WHI)"},       {SYS_ID, PH2,    "J19-9"}},
		{CONNECTIVITY_GEN_6, 0, "SPI2conn", { SYS_ID, PH12,   "J34-6(WHI)"},        {SYS_ID, PB12,   "J34-5"}}, // SPI-2 conn
		{CONNECTIVITY_GEN_7, 0, "SPI2conn", { SYS_ID, PB13,   "J34-8(PUR)"},        {SYS_ID, PB14,   "J34-7"}},
		{CONNECTIVITY_GEN_8, 0, "SPI2conn", { SYS_ID, PB15,   "J34-10(ORA)"},       {SYS_ID, PC8,    "J34-9"}},
		{CONNECTIVITY_GEN_9, 0, "YokeConn", { SYS_ID, PH3,    "J29-19(GRY)"},       {SYS_ID, PH4,    "J29-20"}}, //yoke conn

		//motor X
		{CONNECTIVITY_MX_ENB, 1, "MX_ENB",  { SYS_ID, MOT_EN, "J38-14(PUR)"},       {CAN_MX, LIM2,    ""}},
		{CONNECTIVITY_MX_DIR, 0, "MX_DIR",  { SYS_ID, X_DIR,  "J38-13(WHI)"},       {CAN_MX, LIM1,   ""}},
		{CONNECTIVITY_MX_STP, 0, "MX_STEP", { SYS_ID, X_STEP, "U6-10:J38-12(ORA)"}, {CAN_MX, SWD,   ""}},
		{CONNECTIVITY_MX_FLT, 0, "MX_FLT",  { CAN_MX, SWC,    "(GRN)"},             {SYS_ID, X_FLT,  "J38-5:RN9:CN2:U9-1:RN3"}},
		{CONNECTIVITY_MX_HOM, 0, "MX_HOM",  { CAN_MX, RTD2,   "(ORA)"},             {SYS_ID, X_HOME, "J38-6:RN9:CN2:U9-2:RN3"}},
		{CONNECTIVITY_MX_LM2, 0, "MX_LM2",  { CAN_MX, HPA3,   "(WHI)"},             {SYS_ID, X_L2,   "J38-3:RN9:CN2:U9-13:RN3"}},
		{CONNECTIVITY_MX_LM1, 0, "MX_LM1",  { CAN_MX, HPA2,   "(PUR)"},             {SYS_ID, X_L1,   "J38-4:RN9:CN2:U9-14:RN3"}},
		{CONNECTIVITY_PX_ENB, 1, "PX_ENB",  { SYS_ID, MOT_EN, "J27-8(BLU)"},        {CAN_PX, LIM2,    ""}},
		{CONNECTIVITY_PX_DIR, 0, "PX_DIR",  { SYS_ID, X_DIR,  "J27-6(WHI)"},        {CAN_PX, LIM1,   ""}},
		{CONNECTIVITY_PX_STP, 0, "PX_STEP", { SYS_ID, X_STEP, "U6-10:J27-4(YEL)"},  {CAN_PX, SWD,   ""}},
		{CONNECTIVITY_PX_FLT, 0, "PX_FLT",  { CAN_PX, SWC,    "(GRN)"},             {SYS_ID,  X_FLT, "J27-2:RN9:CN2:U9-1:RN3"}},

		//motor Y
		{CONNECTIVITY_MY_ENB, 1, "MY_ENB",  { SYS_ID, MOT_EN, "J39-14(PUR)"},       {CAN_MY, LIM2,    ""}},
		{CONNECTIVITY_MY_DIR, 0, "MY_DIR",  { SYS_ID, Y_DIR,  "J39-13(WHI)"},       {CAN_MY, LIM1,   ""}},
		{CONNECTIVITY_MY_STP, 0, "MY_STEP", { SYS_ID, Y_STEP, "U6-2:J39-12(ORA)"},  {CAN_MY, SWD,   ""}},
		{CONNECTIVITY_MY_FLT, 0, "MY_FLT",  { CAN_MY, SWC,    "(GRN)"},             {SYS_ID, Y_FLT,  "J39-5:RN10:CN3:U10-1:RN4"}},
		{CONNECTIVITY_MY_HOM, 0, "MY_HOM",  { CAN_MY, RTD2,   "(ORA)"},             {SYS_ID, Y_HOME, "J39-6:RN10:CN3:U10-2:RN4"}},
		{CONNECTIVITY_MY_LM2, 0, "MY_LM2",  { CAN_MY, HPA3,   "(WHI)"},             {SYS_ID, Y_L2,   "J39-3:RN10:CN3:U10-13:RN4"}},
		{CONNECTIVITY_MY_LM1, 0, "MY_LM1",  { CAN_MY, HPA2,   "(PUR)"},             {SYS_ID, Y_L1,   "J39-4:RN10:CN3:U10-14:RN4"}},
		{CONNECTIVITY_PY_ENB, 1, "PY_ENB",  { SYS_ID, MOT_EN, "J28-8(BLU)"},        {CAN_PY, LIM2,    ""}},
		{CONNECTIVITY_PY_DIR, 0, "PY_DIR",  { SYS_ID, Y_DIR,  "J28-6(WHI)"},        {CAN_PY, LIM1,   ""}},
		{CONNECTIVITY_PY_STP, 0, "PY_STEP", { SYS_ID, Y_STEP, "U6-2:J28-4(YEL)"},   {CAN_PY, SWD,   ""}},
		{CONNECTIVITY_PY_FLT, 0, "PY_FLT",  { CAN_PY, SWC,    "(GRN)"},             {SYS_ID,  Y_FLT, "J28-2:RN10:CN3:U10-1:RN4"}},

		//motor Z
		{CONNECTIVITY_MZ_ENB, 1, "MZ_ENB",  { SYS_ID, MOT_EN, "J40-14(PUR)"},       {CAN_MZ, LIM2,    ""}},
		{CONNECTIVITY_MZ_DIR, 0, "MZ_DIR",  { SYS_ID, Z_DIR,  "J40-13(WHI)"},       {CAN_MZ, LIM1,   ""}},
		{CONNECTIVITY_MZ_STP, 0, "MZ_STEP", { SYS_ID, Z_STEP, "U7-10:J40-12(ORA)"}, {CAN_MZ, SWD,   ""}},
		{CONNECTIVITY_MZ_FLT, 0, "MZ_FLT",  { CAN_MZ, SWC,    "(GRN)"},             {SYS_ID, Z_FLT,  "J40-5:RN11:CN4:U11-1:RN5"}},
		{CONNECTIVITY_MZ_HOM, 0, "MZ_HOM",  { CAN_MZ, RTD2,   "(ORA)"},             {SYS_ID, Z_HOME, "J40-6:RN11:CN4:U11-2:RN5"}},
		{CONNECTIVITY_MZ_LM2, 0, "MZ_LM2",  { CAN_MZ, HPA3,   "(WHI)"},             {SYS_ID, Z_L2,   "J40-3:RN11:CN4:U11-13:RN5"}},
		{CONNECTIVITY_MZ_LM1, 0, "MZ_LM1",  { CAN_MZ, HPA2,   "(PUR)"},             {SYS_ID, Z_L1,   "J40-4:RN11:CN4:U11-14:RN5"}},
		{CONNECTIVITY_PZ_ENB, 1, "PZ_ENB",  { SYS_ID, MOT_EN, "J45-8(BLU)"},        {CAN_PZ, LIM2,     ""}},
		{CONNECTIVITY_PZ_DIR, 0, "PZ_DIR",  { SYS_ID, Z_DIR,  "J45-6(WHI)"},        {CAN_PZ, LIM1,    ""}},
		{CONNECTIVITY_PZ_STP, 0, "PZ_STEP", { SYS_ID, Z_STEP, "U7-10:J45-4(YEL)"},  {CAN_PZ, SWD,    ""}},
		{CONNECTIVITY_PZ_FLT, 0, "PZ_FLT",  { CAN_PZ, SWC,    "(GRN)"},             {SYS_ID,  Z_FLT,  "J45-2:RN11:CN4:U11-1:RN5"}},

		//motor A
		{CONNECTIVITY_MA_ENB, 1, "MA_ENB",  { SYS_ID, MOT_EN, "J41-14(PUR)"},       {CAN_MA, LIM2,    ""}},
		{CONNECTIVITY_MA_DIR, 0, "MA_DIR",  { SYS_ID, A_DIR,  "J41-13(WHI)"},       {CAN_MA, LIM1,   ""}},
		{CONNECTIVITY_MA_STP, 0, "MA_STEP", { SYS_ID, A_STEP, "U7-2:J41-12(ORA)"},  {CAN_MA, SWD,   ""}},
		{CONNECTIVITY_MA_FLT, 0, "MA_FLT",  { CAN_MA, SWC,    "(GRN)"},             {SYS_ID, A_FLT,  "J41-5:RN12:CN5:U12-1:RN6"}},
		{CONNECTIVITY_MA_HOM, 0, "MA_HOM",  { CAN_MA, RTD2,   "(ORA)"},             {SYS_ID, A_HOME, "J41-6:RN12:CN5:U12-2:RN6"}},
		{CONNECTIVITY_MA_LM2, 0, "MA_LM2",  { CAN_MA, HPA3,   "(WHI)"},             {SYS_ID, A_L2,   "J41-3:RN12:CN5:U12-13:RN6"}},
		{CONNECTIVITY_MA_LM1, 0, "MA_LM1",  { CAN_MA, HPA2,   "(PUR)"},             {SYS_ID, A_L1,   "J41-4:RN12:CN5:U12-14:RN6"}},
		{CONNECTIVITY_PA_ENB, 1, "PA_ENB",  { SYS_ID, MOT_EN, "J46-8(BLU)"},        {CAN_PA, LIM2,    ""}},
		{CONNECTIVITY_PA_DIR, 0, "PA_DIR",  { SYS_ID, A_DIR,  "J46-6(WHI)"},        {CAN_PA, LIM1,   ""}},
		{CONNECTIVITY_PA_STP, 0, "PA_STEP", { SYS_ID, A_STEP, "U7-2:J46-4(YEL)"},   {CAN_PA, SWD,   ""}},
		{CONNECTIVITY_PA_FLT, 0, "PA_FLT",  { CAN_PA, SWC,    "(GRN)"},             {SYS_ID,  A_FLT, "J46-2:RN12:CN5:U12-1:RN6"}},

		//motor B
		{CONNECTIVITY_MB_ENB, 1, "MB_ENB",  { SYS_ID, MOT_EN, "J42-14(PUR)"},       {CAN_MB, LIM2,    ""}},
		{CONNECTIVITY_MB_DIR, 0, "MB_DIR",  { SYS_ID, B_DIR,  "J42-13(WHI)"},       {CAN_MB, LIM1,   ""}},
		{CONNECTIVITY_MB_STP, 0, "MB_STEP", { SYS_ID, B_STEP, "U8-10:J42-12(ORA)"}, {CAN_MB, SWD,   ""}},
		{CONNECTIVITY_MB_FLT, 0, "MB_FLT",  { CAN_MB, SWC,    "(GRN)"},             {SYS_ID, B_FLT,  "J42-5:RN13:CN6:U13-1:RN7"}},
		{CONNECTIVITY_MB_HOM, 0, "MB_HOM",  { CAN_MB, RTD2,   "(ORA)"},             {SYS_ID, B_HOME, "J42-6:RN13:CN6:U13-2:RN7"}},
		{CONNECTIVITY_MB_LM2, 0, "MB_LM2",  { CAN_MB, HPA3,   "(WHI)"},             {SYS_ID, B_L2,   "J42-3:RN13:CN6:U13-13:RN7"}},
		{CONNECTIVITY_MB_LM1, 0, "MB_LM1",  { CAN_MB, HPA2,   "(PUR)"},             {SYS_ID, B_L1,   "J42-4:RN13:CN6:U13-14:RN7"}},
		{CONNECTIVITY_PB_ENB, 1, "PB_ENB",  { SYS_ID, MOT_EN, "J47-8(BLU)"},        {CAN_PB, LIM2,    ""}},
		{CONNECTIVITY_PB_DIR, 0, "PB_DIR",  { SYS_ID, B_DIR,  "J47-6(WHI)"},        {CAN_PB, LIM1,   ""}},
		{CONNECTIVITY_PB_STP, 0, "PB_STEP", { SYS_ID, B_STEP, "U8-10:J47-4(YEL)"},  {CAN_PB, SWD,   ""}},
		{CONNECTIVITY_PB_FLT, 0, "PB_FLT",  { CAN_PB, SWC,    "(GRN)"},             {SYS_ID,  B_FLT, "J47-2:RN13:CN6:U13-1:RN7"}},

		//motor C
		{CONNECTIVITY_MC_ENB, 1, "MC_ENB",  { SYS_ID, MOT_EN, "J43-14(PUR)"},       {CAN_MC, LIM2,    ""}},
		{CONNECTIVITY_MC_DIR, 0, "MC_DIR",  { SYS_ID, C_DIR,  "J43-13(WHI)"},       {CAN_MC, LIM1,   ""}},
		{CONNECTIVITY_MC_STP, 0, "MC_STEP", { SYS_ID, C_STEP, "U8-2:J43-12(ORA)"},  {CAN_MC, SWD,   ""}},
		{CONNECTIVITY_MC_FLT, 0, "MC_FLT",  { CAN_MC, SWC,    "(GRN)"},             {SYS_ID, C_FLT,  "J43-5:RN14:CN7:U14-1:RN8"}},
		{CONNECTIVITY_MC_HOM, 0, "MC_HOM",  { CAN_MC, RTD2,   "(ORA)"},             {SYS_ID, C_HOME, "J43-6:RN14:CN7:U14-2:RN8"}},
		{CONNECTIVITY_MC_LM2, 0, "MC_LM2",  { CAN_MC, HPA3,   "(WHI)"},             {SYS_ID, C_L2,   "J43-3:RN14:CN7:U14-13:RN8"}},
		{CONNECTIVITY_MC_LM1, 0, "MC_LM1",  { CAN_MC, HPA2,   "(PUR)"},             {SYS_ID, C_L1,   "J43-4:RN14:CN7:U14-14:RN8"}},
		{CONNECTIVITY_PC_ENB, 1, "PC_ENB",  { SYS_ID, MOT_EN, "J48-8(BLU)"},        {CAN_PC, LIM2,    ""}},
		{CONNECTIVITY_PC_DIR, 0, "PC_DIR",  { SYS_ID, C_DIR,  "J48-6(WHI)"},        {CAN_PC, LIM1,   ""}},
		{CONNECTIVITY_PC_STP, 0, "PC_STEP", { SYS_ID, C_STEP, "U8-2:J48-4(YEL)"},   {CAN_PC, SWD,   ""}},
		{CONNECTIVITY_PC_FLT, 0, "PC_FLT",  { CAN_PC, SWC,    "(GRN)"},             {SYS_ID,  C_FLT, "J48-2:RN14:CN7:U14-1:RN8"}},
};
#define DIAGS_CONNECTIVITY_PAIRS (sizeof(connectivityList) / sizeof(connectivityStruct))

////////////////////////////////////////////////////////////////////////////////

testStruct _diagsTests[] = {
	// DIAGS_INIT_TIME
		{   0, 0, DIAGS_STRING_TIME},
		{   1, 0, DIAGS_STRING_TIME},
		{   2, 0, DIAGS_STRING_TIME},
		{   3, 0, DIAGS_STRING_TIME},
		{   4, 0, DIAGS_STRING_TIME},

		{  10, 0, DIAGS_STRING_TIME},
		{  11, 0, DIAGS_STRING_TIME},
		{  12, 0, (DIAGS_SECONDS_TO_REGISTER * 1000)},  // swReset of heads

	// DIAGS_POWER_TIME
		{  20, 0, DIAGS_STRING_TIME},
		{  21, 0, DIAGS_STRING_TIME},
		{  22, 0, DIAGS_LOCAL_ADC_TEST_TIME_MS},
		{  23, 0, DIAGS_LOCAL_ADC_TEST_TIME_MS},

		{  30, 0, DIAGS_STRING_TIME},
		{  31, 0, DIAGS_LOCAL_ADC_TEST_TIME_MS},
		{  32, 0, DIAGS_LOCAL_ADC_TEST_TIME_MS},

	// DIAGS_CANBUS_TIME
		{  50, 0, DIAGS_STRING_TIME},
		{  51, 0, DIAGS_STRING_TIME},
		{  52, 0, DIAGS_STRING_TIME},
		{  53, 0, DIAGS_STRING_TIME},
		{  54, 0, DIAGS_STRING_TIME},
		{  55, 0, DIAGS_STRING_TIME},
		{  56, 0, DIAGS_STRING_TIME},
		{  57, 0, DIAGS_STRING_TIME},

		{  61, 0, DIAGS_STRING_TIME},
		{  62, 0, DIAGS_STRING_TIME},
		{  63, 0, DIAGS_STRING_TIME},
		{  64, 0, DIAGS_STRING_TIME},
		{  65, 0, DIAGS_STRING_TIME},
		{  66, 0, DIAGS_STRING_TIME},
		{  67, 0, DIAGS_STRING_TIME},

	// DIAGS_ADD_POWER_TIME
		{ 100, 0, DIAGS_STRING_TIME},

		{ 101, 0, DIAGS_STRING_TIME},
		{ 102, 0, DIAGS_RELAY_CHANGE_TIME_MS},
		{ 103, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 104, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 105, 0, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 111, 0, DIAGS_STRING_TIME},
		{ 112, 0, DIAGS_RELAY_CHANGE_TIME_MS},
		{ 113, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 114, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 115, 0, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 120, 0, DIAGS_STRING_TIME},
		{ 121, 0, DIAGS_LOCAL_ADC_TEST_TIME_MS},

		{ 130, 0, DIAGS_STRING_TIME},
		{ 131, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 132, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 133, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 134, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 135, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 136, 0, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 140, 0, DIAGS_STRING_TIME},
		{ 141, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 142, 0, DIAGS_CAN_WRITE_TIME},
		{ 143, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 144, 0, DIAGS_CAN_WRITE_TIME},
		{ 145, 0, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 146, 0, DIAGS_CAN_WRITE_TIME},
		{ 147, 0, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 150, 0, DIAGS_STRING_TIME},
		{ 151, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 152, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 153, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 154, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 155, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 156, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 157, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 158, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 159, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 160, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 161, 1, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 170, 0, DIAGS_STRING_TIME},
		{ 171, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 172, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 173, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 174, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 175, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 176, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 177, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 178, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 179, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 180, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 181, 1, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 190, 0, DIAGS_STRING_TIME},
		{ 191, 1, DIAGS_LOCAL_ADC_TEST_TIME_MS},
		{ 192, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 193, 1, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 195, 0, DIAGS_STRING_TIME},
		{ 196, 1, DIAGS_LOCAL_ADC_TEST_TIME_MS},
		{ 197, 1, DIAGS_CAN_ADC_TEST_TIME_MS},
		{ 198, 1, DIAGS_CAN_ADC_TEST_TIME_MS},

		{ 200, 0, DIAGS_STRING_TIME},
		{ 201, 0, DIAGS_STRING_TIME},
		{ 202, 0, DIAGS_STRING_TIME},

		{ 300, 0, DIAGS_STRING_TIME},
		{ 301, 0, DIAGS_STRING_TIME},
		{ 302, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 303, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 304, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 305, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 306, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 307, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 308, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 309, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 310, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 311, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 320, 0, DIAGS_STRING_TIME},
		{ 321, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 322, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 323, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 324, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 325, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 326, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 327, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 328, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 329, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 340, 0, DIAGS_STRING_TIME},
		{ 341, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 342, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 343, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 344, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 345, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 346, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 347, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 348, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 349, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 360, 0, DIAGS_STRING_TIME},
		{ 361, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 362, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 363, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 364, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 365, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 366, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 367, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 368, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 369, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 380, 0, DIAGS_STRING_TIME},
		{ 381, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 382, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 383, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 384, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 385, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 386, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 387, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 388, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 389, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 400, 0, DIAGS_STRING_TIME},
		{ 401, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 402, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 403, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 404, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 405, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 406, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 407, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 408, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 409, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 420, 0, DIAGS_STRING_TIME},
		{ 421, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 422, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 423, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 424, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 425, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 426, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 427, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 428, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 429, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 500, 0, DIAGS_STRING_TIME},
		{ 501, 0, DIAGS_STRING_TIME},
		{ 502, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 503, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 504, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 505, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 506, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 507, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 508, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 509, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 510, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 511, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 520, 0, DIAGS_STRING_TIME},
		{ 521, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 522, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 523, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 524, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 525, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 526, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 527, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 528, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 529, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 540, 0, DIAGS_STRING_TIME},
		{ 541, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 542, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 543, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 544, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 545, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 546, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 547, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 548, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 549, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 560, 0, DIAGS_STRING_TIME},
		{ 561, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 562, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 563, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 564, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 565, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 566, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 567, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 568, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 569, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 580, 0, DIAGS_STRING_TIME},
		{ 581, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 582, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 583, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 584, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 585, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 586, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 587, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 588, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 589, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 600, 0, DIAGS_STRING_TIME},
		{ 601, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 602, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 603, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 604, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 605, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 606, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 607, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 608, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 609, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 620, 0, DIAGS_STRING_TIME},
		{ 621, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 622, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 623, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 624, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 625, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 626, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 627, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 628, 0, DIAGS_LOOPBACK_TEST_TIME_MS},
		{ 629, 0, DIAGS_LOOPBACK_TEST_TIME_MS},

		{ 700, 0, DIAGS_STRING_TIME },
		{ 701, 0, DIAGS_STRING_TIME },
		{ 702, 0, DIAGS_STRING_TIME },
		{ 703, 0, DIAGS_MOVE_TIME_MS },

		{ 710, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 711, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 712, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 713, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 714, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 715, 0, DIAGS_TEST_STEPS_TIME_MS},

		{ 720, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 721, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 722, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 723, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 724, 0, DIAGS_TEST_STEPS_TIME_MS},
		{ 725, 0, DIAGS_TEST_STEPS_TIME_MS},

		{ 999, 0, DIAGS_STRING_TIME },
		{ 1000, 0, 3000 },
		{ 1001, 0, 3000 },
};
#define DIAGS_NUM_TESTS (sizeof(_diagsTests) / sizeof(testStruct))

////////////////////////////////////////////////////////////////////////////////


int                 _diagsSaveReportingRate[3];
boolean             _diagsEnabled = FALSE;
int                 _diagsAllowedErrors;
uint32_t            _diagsClockCount = 0;
uint32_t            _diagsTotalErrors = 0;
uint32_t            _diagsCurrentTestIndex = 0;
uint32_t            _diagsNextTestIndex = 0;
uint32_t            _diagsNextTestTime = 0;
uint32_t            _diagsNextSubtestTime = 0;
subtest_t           _diagsNextSubtestState  = DIAGS_SUBSTEST_NONE;
boolean             _diagsRunSingleTest;
uint32_t            _diagsSingleTestLoopCount = 0;

canDevice_t         _diagsSubtestDevice;
int                 _diagsSubtestStepCount;
diagAdcInfoStruct   *_diagsAdcPtr;
uint16_t            _diagsSubtestNomAdc;
//float             _diagsSubtestNomVoltage;
char                *_diagsSubtestStr;

connectivityStruct  *_diagsConnPtr;
pinStateValue_t     _diagsConnectivityTestValue;
pinStateValue_t     _diagsConnectivityBackgroundValue;
pinStateValue_t     _diagsConnectivityReadVal;
uint16_t            _diagsAdcReadValue;

int                 _diagsStepsDetected;
float               _diagsStepMinTimeUs;
float               _diagsStepMaxTimeUs;
float               _diagsStepsAvgTimeUs;

uint32_t            _diagCurrentIndentLevel = 0;

diagLedMode_t       _diagsLedDisplayMode = DIAG_LED_OFF;
uint32_t            _diagsLedValue = 0;
int                 _diagLedDisplayCnt = 0;
payloadUnion        _diagsNR;

char                _diagsOutPinStr[16];
char                _diagsInPinStr[16];

void runDiagSubtest(subtest_t); // forward declaration
void setEMO(FunctionalState);

////////////////////////////////////////////////////////////////////////////////

void diagSetLed(byte device, byte value, byte mask)
{
	canPackIntoTxQueue1x16(CAN_WRITE, device, CAN_MSG_LED_OVERRIDE, NO_PAGE, BUFFERED_MSG,
			(uint16_t)((uint32_t)mask << 8 | (uint32_t)value));
}
////////////////////////////////////////////////////////////////////////////////

void diagLedDisplay(void)
{
	int device;
	int tmpCnt;
	switch (_diagsLedDisplayMode)
	{
	case DIAG_LED_OFF:
		break;
	case DIAG_LED_NIGHT_RIDER:
		tmpCnt = _diagLedDisplayCnt;

		if (tmpCnt == 0)
			_diagsNR.u64 = 0x3f;
		else if (tmpCnt < 64)
			_diagsNR.u64 *= 2;
		else if (tmpCnt == 64)
			_diagsNR.u64 = 0xfc00000000000000;
		else
			_diagsNR.u64 /= 2;

		for (device=0; device<6; device++)
		{
			diagSetLed(CAN_MX+device, _diagsNR.u8[device+1], 0xff);
			diagSetLed(CAN_PC-device, _diagsNR.u8[device+1], 0xff);
		}
		break;
	case DIAG_LED_BLINK_SLOW:
		if ((_diagLedDisplayCnt % 100) == 0)
		{
			_diagsLedValue = (~_diagsLedValue & 0xff);
			diagSetLed(0, _diagsLedValue, 0xff);
		}
		break;
	case DIAG_LED_BLINK_FAST:
		if ((_diagLedDisplayCnt % 10) == 0)
		{
			_diagsLedValue = (~_diagsLedValue & 0xff);
			diagSetLed(0, _diagsLedValue, 0xff);
		}
		break;
	default: break;
	}

	_diagLedDisplayCnt++;
	_diagLedDisplayCnt %= (_diagsLedDisplayMode == DIAG_LED_NIGHT_RIDER) ? 128 : 1000;
}
////////////////////////////////////////////////////////////////////////////////

void sendDIAG(char *s)
{
	sendstring(DIAG_STRING_START);
	sendstring(s);
}

////////////////////////////////////////////////////////////////////////////////

void sendDIAGCr(char *s)
{
	sendDIAG(s);
	sendCr();
}
////////////////////////////////////////////////////////////////////////////////

void indentAndSendDiagStr(char *s)
{
	int level=0;
	sendDIAG("");
	while (level < _diagCurrentIndentLevel)
	{
		sendstring(DIAG_INDENT_STRING);
		level++;
	}
	sendstring(s);
}

////////////////////////////////////////////////////////////////////////////////

void indentAndSendDiagStrCr(char *s)
{
	indentAndSendDiagStr(s);
	sendCr();
}

////////////////////////////////////////////////////////////////////////////////

void errorOccurred(void)
{
	_diagsTotalErrors++;
	if (_diagsTotalErrors >= _diagsAllowedErrors)
	{
		diagsExit(TRUE);
	}
}

void sendTestResult(result_t result)
{
	if (result == RESULT_PASSED)
		sendstringCr(" PASS");
	else if (result == RESULT_FAILED)
		sendstring(" FAIL   "); // calling routine will add failure info, so no Cr
	else if (result == RESULT_ERROR)
		sendstring(" FAIL   "); // calling routine will add error info, so no Cr

	if ((result == RESULT_FAILED) || (result == RESULT_ERROR))
	{
		errorOccurred();
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendTestStr(result_t result, char *s)
{
	char str[DIAG_STRING_SIZE+1];
	int i, j;

	bzero(str, DIAG_STRING_SIZE+1);

	//sprintf(str, "(%03d/%d/%1d) ", _diagsTests[_diagsCurrentTestIndex].testNum, (int)_diagsClockCount, (int)normalTxCharsInBuf);
	sprintf(str, "(%03d) ", _diagsTests[_diagsCurrentTestIndex].testNum);

	j=0;
	for (i=strlen(str); i<DIAG_STRING_SIZE - 10; i++)   // leave a little room (-10)
	{   // add in requested string, but truncate if needed
		str[i] = s[j++];
		if (s[j] == '\0')
			break;
	}
	if (s[j] != '\0')
	{
		sendDIAGCr("TRUNCATING TEST STRING");
	}

	if (i < DIAG_STRING_SIZE)
	{   // add a blank after users string
		strcat(str, " ");
	}

	for (i=strlen(str); i<DIAG_STRING_SIZE; i++)
	{   // dot fill for easy visual alignment in report
		str[i] = (i & 0x1) ? '.' : ' ';
	}

	str[DIAG_STRING_SIZE - (_diagCurrentIndentLevel * strlen(DIAG_INDENT_STRING))] = '\0'; //terminate to align after indent
	indentAndSendDiagStr(str);

	sendTestResult(result);
}

////////////////////////////////////////////////////////////////////////////////

char *getCompareString(pinStateValue_t shouldBe, pinStateValue_t actualValue, char *str)
{
	sprintf(str, "(sb:%d  is:%d)", (unsigned int)shouldBe, (unsigned int)actualValue);
	return(str);
}

////////////////////////////////////////////////////////////////////////////////

void sendHeaderStr(int level, char *s)
{
	if (level == -1)
	{   // add blank line
		sendDIAGCr("");
		level = 0;
	}
	_diagCurrentIndentLevel = level;
	indentAndSendDiagStrCr(s);
	_diagCurrentIndentLevel++;  // tests indented one extra
}

////////////////////////////////////////////////////////////////////////////////

char *diagDeviceName(canDevice_t device)
{
	int i = 0;
	while (i < NUM_CAN_DIAG_DEVICES-1)
	{
		if (device == canDevName[i].device)
		{
			break;
		}
		i++;
	}
	return(canDevName[i].name);
}

////////////////////////////////////////////////////////////////////////////////

char *diagPinName(byte device, pinType pin, char *str, boolean addDevice)
{
	if (addDevice)
		sprintf(_tmpStr, "%s-", diagDeviceName(device));
	else
		sprintf(_tmpStr, "%s", "");

	if (device == SYS_ID)
		sprintf(str, "%sP%c%d", _tmpStr, pinPortChar(pin), (int)pinExtractBitNum(pin));
	else
	{
		switch(pin)
		{
			case HH_LIM1: sprintf(str, "%sPB6{LIM1}", _tmpStr); break;
			case HH_LIM2: sprintf(str, "%sPA10{LIM2}", _tmpStr); break;
			case HH_RTD1: sprintf(str, "%sPA1{RTD1}", _tmpStr); break;
			case HH_RTD2: sprintf(str, "%sPA0{RTD2}", _tmpStr); break;
			case HH_PA2:  sprintf(str, "%sPA2", _tmpStr); break;
			case HH_PA3:  sprintf(str, "%sPA3", _tmpStr); break;
			case HH_SWC:  sprintf(str, "%sPA14{SWC}", _tmpStr); break;
			case HH_SWD:  sprintf(str, "%sPA13{SWD}", _tmpStr); break;
			default:      sprintf(str, "%sPXX", _tmpStr); break;
		}
	}
	return(str);
}

////////////////////////////////////////////////////////////////////////////////

char *diagPinNameFromPtr(diagPinStruct *pinPtr, char *str, boolean addDevice)
{
	return(diagPinName(pinPtr->device, pinPtr->pin, str, addDevice));
}

////////////////////////////////////////////////////////////////////////////////

char *prepDiagString(char *newStr, char *constStr, int space)
{
	strncpy(newStr, constStr, DIAG_STRING_SIZE-space);
	if (strlen(constStr) > DIAG_STRING_SIZE-space)
		newStr[DIAG_STRING_SIZE] = '\0';
	else
		newStr[strlen(constStr)] = '\0';
	return(newStr);
}

////////////////////////////////////////////////////////////////////////////////

result_t checkCanDeviceRegistered(canDevice_t device, char *s)
{
	char str[DIAG_STRING_SIZE+1];
	prepDiagString(str, s, sizeof_member(canDevNameStruct,name)+1);

	sprintf(_tmpStr, ":%s", diagDeviceName(device));
	strcat(str, _tmpStr);

	result_t result = deviceRegistered(device) ? RESULT_PASSED : RESULT_FAILED;
	sendTestStr(result, str);
	if (result == RESULT_FAILED)
	{
		sendstringCr("(not registered)");
	}
	return(result);
}

////////////////////////////////////////////////////////////////////////////////

void sendAdcResults(char *str)
{
	if (_diagsAdcReadValue == DIAGS_NO_ADC_VALUE)
	{   // no read occurred -- ERROR
		sendTestStr(RESULT_ERROR, str);
		if (_diagsSubtestDevice == SYS_ID)
			sendstringCr("Local ADC value not read");
		else
			sendstringCr("CAN ADC value not returned");
	}
	else
	{   // got an adc value via the canbus, so proceed
		int minValue = imax(0, (int)_diagsSubtestNomAdc - ADC_TOLERANCE);
		int maxValue = imin(4095, (int)_diagsSubtestNomAdc + ADC_TOLERANCE);
//      float approxReadVoltage;
//      if ((_diagsSubtestNomVoltage == 0.0f) || (_diagsSubtestNomAdc == 0))
//          approxReadVoltage = 99.9f;
//      else
//          approxReadVoltage = (_diagsSubtestNomVoltage / _diagsSubtestNomAdc) * _diagsAdcReadValue;
		result_t result = ((_diagsAdcReadValue < minValue) || (_diagsAdcReadValue > maxValue)) ? RESULT_FAILED : RESULT_PASSED;
		sendTestStr(result, str);
		if (result == RESULT_FAILED)
		{
//          sprintf(_tmpStr, "(adcVal too %s. is:%d sb:%d<=VAL<=%d; ~%3.1fV)", (_diagsAdcReadValue < maxValue) ? "low" : "high",
//                  _diagsAdcReadValue, minValue, maxValue, approxReadVoltage);
			sprintf(_tmpStr, "(adcVal too %s. is:%d sb:%d<=VAL<=%d)", (_diagsAdcReadValue < maxValue) ? "low" : "high",
								_diagsAdcReadValue, minValue, maxValue);
			sendstringCr(_tmpStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void finishCheckCanAdc(void)
{
	char str[DIAG_STRING_SIZE+1];
	prepDiagString(str, _diagsSubtestStr, sizeof_member(canDevNameStruct, name)+sizeof_member(diagAdcInfoStruct, name)+12);

	sprintf(_tmpStr, ":PROBE:%s:%s", diagDeviceName(_diagsSubtestDevice), _diagsAdcPtr->name);
	strcat(str, _tmpStr);

	sendAdcResults(str);
}

////////////////////////////////////////////////////////////////////////////////`

void finishCheckLocalAdc(void)
{
	char str[DIAG_STRING_SIZE+1];
	prepDiagString(str, _diagsSubtestStr, 16);
	strcat(str, ":"); strcat(str, diagPinName(SYS_ID, _diagsAdcPtr->pin, _rptStr, TRUE));
	strcat(str, ":"); strcat(str, _diagsAdcPtr->name);
	sendAdcResults(str);
}

////////////////////////////////////////////////////////////////////////////////

diagAdcInfoStruct *getAdcInfoPtr(diagAdcIndex_t adcIndex)
{
	int i;
	for (i=0; i<NUM_DIAG_ADCS; i++)
	{
		if (adcIndex == diagAdcInfo[i].adcIndex)
		{   // found a match
			return(&diagAdcInfo[i]);
		}
	}
	// get this far, then no match found, so return a benign pointer
	return(&diagAdcInfo[0]);
}

////////////////////////////////////////////////////////////////////////////////

void checkViaAdc(canDevice_t device, diagAdcIndex_t adcIndex, float nomVoltage, uint16_t nomVal, char *s)
{
	// store values to re-use du2ing subsequent steps for this test
	_diagsSubtestDevice     = device;
	_diagsAdcPtr            = getAdcInfoPtr(adcIndex);
	_diagsSubtestNomAdc     = nomVal;
//  _diagsSubtestNomVoltage = nomVoltage;
	_diagsSubtestStr        = s;

	if (device == SYS_ID)
	{
		runDiagSubtest(DIAGS_SUBSTEST_LOCAL_ADC_SETUP);
	}
	else
	{
		runDiagSubtest(DIAGS_SUBSTEST_CAN_ADC_SETUP);
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkLocalIoState(pinType pin, pinStateValue_t state, char *s)
{
	uint32_t readValue;

	pin = (pin & PIN_PORT_PIN_NUM_MASK) | INPUT_FLOATING;
	pinInit(pin);   // force to be set up as an input without pullup/pulldown

	result_t result = ((readValue = pinRead(pin)) == state) ? RESULT_PASSED : RESULT_FAILED;
	sendTestStr(result, s);

	if (result == RESULT_FAILED)
	{
		getCompareString(state, readValue, _tmpStr);
		sendstringCr(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void diagInitPin(byte device, pinType pin, uint32_t setupValue)
{
	if (device == SYS_ID)
	{   // local pin
		pinInit((pin & PIN_PORT_PIN_NUM_MASK) | setupValue);
	}
	else
	{   // pin on a canbus device
		canPackIntoTxQueue1x16(CAN_WRITE, device, CAN_MSG_DIAG_IO_INIT, pin, BUFFERED_MSG, setupValue);
	}
}

void diagInitPinPtr(diagPinStruct *pinPtr, uint32_t setupValue)
{
	diagInitPin(pinPtr->device, pinPtr->pin, setupValue);
}

////////////////////////////////////////////////////////////////////////////////

pinStateValue_t diagReadPin(byte device, pinType pin)
{
	pinStateValue_t value;
	if (device == SYS_ID)
	{   // local pin
		value = pinRead(pin);
	}
	else
	{
		canIssueReadRequest(device, CAN_MSG_DIAG_IO_READ, pin);
		value = UNKNOWN_STATE;  // set to something 'wrong' until can packet comes back
	}
	return(value);
}

pinStateValue_t diagReadPinPtr(diagPinStruct *pinPtr)
{
	return(diagReadPin(pinPtr->device, pinPtr->pin));
}

////////////////////////////////////////////////////////////////////////////////

void diagWritePin(byte device, pinType pin, pinStateValue_t value)
{
	if (device == SYS_ID)
	{   // local pin
		pinWrite(pin, value);
	}
	else
	{
		canPackIntoTxQueue1x16(CAN_WRITE, device, CAN_MSG_DIAG_IO_WRITE, pin, BUFFERED_MSG, value);
	}
}

void diagWritePinPtr(diagPinStruct *pinPtr, pinStateValue_t value)
{
	diagWritePin(pinPtr->device, pinPtr->pin, value);
}

////////////////////////////////////////////////////////////////////////////////

void sendConnectivityTestStr(void)
{
	sprintf(_rptStr, "%s", "");
	if (strlen(_diagsConnPtr->netName)) {
		sprintf(_tmpStr, "(%s):", _diagsConnPtr->netName);
		strcat(_rptStr, _tmpStr);
	}
	sprintf(_tmpStr, "%s:",  diagPinNameFromPtr(&_diagsConnPtr->out, _diagsOutPinStr, TRUE));
	strcat(_rptStr, _tmpStr);
	if (strlen(_diagsConnPtr->out.connPinStr)) {
		sprintf(_tmpStr, "%s:", _diagsConnPtr->out.connPinStr);
		strcat(_rptStr, _tmpStr);
	}
	sprintf(_tmpStr, "PROBE:");
	strcat(_rptStr, _tmpStr);
	if (strlen(_diagsConnPtr->in.connPinStr)) {
		sprintf(_tmpStr, "%s:", _diagsConnPtr->in.connPinStr);
		strcat(_rptStr, _tmpStr);
	}
	sprintf(_tmpStr, "%s",  diagPinNameFromPtr(&_diagsConnPtr->in, _diagsInPinStr, TRUE));
	strcat(_rptStr, _tmpStr);

	sendTestStr(UNKNOWN_RESULT, _rptStr);
}

////////////////////////////////////////////////////////////////////////////////

void initializeConnectivityTests(void)
{
	int i;
	for (i=0; i< DIAGS_CONNECTIVITY_PAIRS; i++)
	{   // first, force the state for ALL inputs involved in connectivity tests to floating inputs (reset default state)
		if (&connectivityList[i].connectivityIndex != CONNECTIVITY_NONE)
		{
			if (connectivityList[i].in.device == SYS_ID)
				diagInitPinPtr(&connectivityList[i].in, INPUT_FLOATING);
			else
				diagInitPinPtr(&connectivityList[i].in, HH_IN_FLOAT);
		}
	}
	for (i=0; i< DIAGS_CONNECTIVITY_PAIRS; i++)
	{   // next, force the state for ALL outputs involved in connectivity tests to push-pull outputs
		if (&connectivityList[i].connectivityIndex != CONNECTIVITY_NONE)
		{
			if (connectivityList[i].out.device == SYS_ID)
			{
				diagInitPinPtr(&connectivityList[i].out, OUTPUT_PP_2MHZ);
			}
			else
			{   // canbus device
				if (connectivityList[i].out.pin == SWC)
				{   // special case with dual drivers for FAULT.... need to leave both floating
					diagInitPinPtr(&connectivityList[i].out, HH_IN_FLOAT);
				}
				else
				{
					diagInitPinPtr(&connectivityList[i].out, HH_OUT_PP_02MHZ);
				}
			}
		}
	}

	_diagsConnectivityTestValue = UNKNOWN_STATE;
	_diagsConnectivityBackgroundValue = UNKNOWN_STATE;
}

////////////////////////////////////////////////////////////////////////////////

void setConnectivityBackground(pinStateValue_t backgroundState)
{
	int i;
	for (i=0; i< DIAGS_CONNECTIVITY_PAIRS; i++)
	{   // set state for ALL I/O in connectivityList list
		if (&connectivityList[i].connectivityIndex != CONNECTIVITY_NONE)
			diagWritePinPtr(&connectivityList[i].out, backgroundState);
	}
	_diagsConnectivityBackgroundValue = backgroundState;
	_diagsConnectivityTestValue = (_diagsConnectivityBackgroundValue == LOW) ? HIGH : LOW;  // start opposite background

}
////////////////////////////////////////////////////////////////////////////////

connectivityStruct *getConnectivityListPtr(connectivityIndex_t connIndex)
{
	int i;
	for (i=0; i<DIAGS_CONNECTIVITY_PAIRS; i++)
	{
		if (connIndex == connectivityList[i].connectivityIndex)
		{   // found a match
			return(&connectivityList[i]);
		}
	}
	// get this far, then no match found, so return a benign pointer
	return(&connectivityList[0]);
}

////////////////////////////////////////////////////////////////////////////////

void checkConnectivity(connectivityIndex_t connIndex)
{
	_diagsConnPtr = getConnectivityListPtr(connIndex);
	_diagsSubtestDevice = _diagsConnPtr->in.device;
	if (_diagsConnPtr->connectivityIndex != CONNECTIVITY_NONE)
	{
		runDiagSubtest(DIAGS_SUBSTEST_LOOPBACK_WRITE);
	}
}

////////////////////////////////////////////////////////////////////////////////

void connectivityTestCompare(void)
{
	if (_diagsConnectivityReadVal == UNKNOWN_STATE)
	{   // no read occurred -- ERROR
		sendTestResult(RESULT_ERROR);
		if (_diagsSubtestDevice == SYS_ID)
			sendstringCr("Local conn value not read");
		else
			sendstringCr("CAN conn value not returned");
	}
	else
	{
		pinStateValue_t testValue = _diagsConnectivityTestValue;
		if (_diagsConnPtr->inverting)
		{
			testValue = (_diagsConnectivityTestValue == LOW) ? HIGH : LOW;
		}

		result_t result = (_diagsConnectivityReadVal == testValue) ? RESULT_PASSED : RESULT_FAILED;
		sendTestResult(result);

		if (result == RESULT_FAILED)
		{   // mismatch!!!
			getCompareString(testValue, _diagsConnectivityReadVal, _tmpStr);
			sendstringCr(_tmpStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void clearStepPins(void)
{
	pinClear(X_STEP);
	pinClear(Y_STEP);
	pinClear(Z_STEP);
	pinClear(A_STEP);
	pinClear(B_STEP);
	pinClear(C_STEP);
}

void clearStepCounts(void)
{
	byte device;

	for (device=CAN_MX; device<=CAN_MC; device++)
	{
		diagInitPin(device, HH_SWD, HH_IN_FLOAT);   // force step pin to be an input
		canPackIntoTxQueueNoData(CAN_WRITE, device, CAN_MSG_DIAG_STEP_COUNTER, 0, BUFFERED_MSG);
	}
	for (device=CAN_PX; device<=CAN_PC; device++)
	{
		diagInitPin(device, HH_SWD, HH_IN_FLOAT);   // force step pin to be an input
		canPackIntoTxQueueNoData(CAN_WRITE, device, CAN_MSG_DIAG_STEP_COUNTER, 0, BUFFERED_MSG);
	}
}

////////////////////////////////////////////////////////////////////////////////

void executeFakeMove(int pulses)
{
	_diagsSubtestStepCount = pulses;
	// set pulses per mm; then do a 1mm move
	MotorStructure *M;
	InitializeMotorParameters();
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		M->MotorInstalled = TRUE;
		M->PulsesPerUnit = (float)pulses;
		M->AxisType = LINEAR;
		M->RatesInUPS[AXIS_MIN] = 1.0f; // 1mm/sec
		M->RatesInUPS[AXIS_MAX] = 1.0f; // 1mm/sec
		M->RatesInUPS[NO_RAMP] = 1.0f;  // 1mm/sec
		M->RatesInUPS[RAPID] = 1.0f;    // 1mm/sec
	}
	IncrementalMove = TRUE;
	ExecutionPtr = &cmdQue[0];
	InvalidateAllCmdArgs(ExecutionPtr);
	ExecutionPtr->G = 0.0f;
	ExecutionPtr->X = 1.0f;
	ExecutionPtr->Y = 1.0f;
	ExecutionPtr->Z = 1.0f;
	ExecutionPtr->A = 1.0f;
	ExecutionPtr->B = 1.0f;
	ExecutionPtr->C = 1.0f;
	SetAllAxisHomePosition(TRUE);
	G_Code_G0();
}

////////////////////////////////////////////////////////////////////////////////

#define DIAGS_MIN_STEP_TIME 3.0f
#define DIAGS_MAX_STEP_TIME 5.0f

void checkStepCountResults(void)
{
	result_t result = (_diagsSubtestStepCount == _diagsStepsDetected) ? RESULT_PASSED : RESULT_FAILED;
	sprintf(_rptStr, "Number of steps (%d)",_diagsSubtestStepCount);
	sendTestStr(result, _rptStr);

	if (result == RESULT_FAILED)
	{   // mismatch!!!
		getCompareString(_diagsSubtestStepCount, _diagsStepsDetected, _tmpStr);

		sendstringCr(_tmpStr);
	}

	result = (_diagsStepMinTimeUs >= DIAGS_MIN_STEP_TIME) ? RESULT_PASSED : RESULT_FAILED;
	sprintf(_rptStr, "Min step time (%4.2f uS) ",_diagsStepMinTimeUs);
	sendTestStr(result, _rptStr);

	if (result == RESULT_FAILED)
	{   // mismatch!!!
		sprintf(_tmpStr, " (sb:>%3.1f  is:%4.2f)", DIAGS_MIN_STEP_TIME, _diagsStepMinTimeUs);
		sendstringCr(_tmpStr);
	}

	result = (_diagsStepMaxTimeUs <= DIAGS_MAX_STEP_TIME) ? RESULT_PASSED : RESULT_FAILED;
	sprintf(_rptStr, "Max step time (%4.2f uS) ", _diagsStepMaxTimeUs);
	sendTestStr(result, _rptStr);

	if (result == RESULT_FAILED)
	{   // mismatch!!!
		sprintf(_tmpStr, " (sb:<%3.1f  is:%4.2f)", DIAGS_MAX_STEP_TIME, _diagsStepMaxTimeUs);
		sendstringCr(_tmpStr);
	}

	result = ((_diagsStepsAvgTimeUs >= DIAGS_MIN_STEP_TIME) && (_diagsStepsAvgTimeUs <= DIAGS_MAX_STEP_TIME))  ? RESULT_PASSED : RESULT_FAILED;
	sprintf(_rptStr, "Avg step time (%4.2f uS) ", _diagsStepsAvgTimeUs);
	sendTestStr(result, _rptStr);

	if (result == RESULT_FAILED)
	{   // mismatch!!!
		sprintf(_tmpStr, " (sb:%3.1f<=AVG<=%3.1f  is:%4.2f)", DIAGS_MIN_STEP_TIME, DIAGS_MAX_STEP_TIME, _diagsStepsAvgTimeUs);
		sendstringCr(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendStepCountHeader(void)
{
	 sprintf(_rptStr, "%s", "");
	if (strlen(_diagsConnPtr->netName)) {
		sprintf(_tmpStr, "(%s):", _diagsConnPtr->netName);
		strcat(_rptStr, _tmpStr);
	}
	sprintf(_tmpStr, "%s:",  diagPinNameFromPtr(&_diagsConnPtr->out, _diagsOutPinStr, TRUE));
	strcat(_rptStr, _tmpStr);
	if (strlen(_diagsConnPtr->out.connPinStr)) {
		sprintf(_tmpStr, "%s:", _diagsConnPtr->out.connPinStr);
		strcat(_rptStr, _tmpStr);
	}
	sprintf(_tmpStr, "PROBE:");
	strcat(_rptStr, _tmpStr);
	if (strlen(_diagsConnPtr->in.connPinStr)) {
		sprintf(_tmpStr, "%s:", _diagsConnPtr->in.connPinStr);
		strcat(_rptStr, _tmpStr);
	}
	sprintf(_tmpStr, "%s",  diagPinNameFromPtr(&_diagsConnPtr->in, _diagsInPinStr, TRUE));
	strcat(_rptStr, _tmpStr);

	sendTestStr(UNKNOWN_RESULT, _rptStr);

	result_t result = (_diagsStepsDetected == -1) ? RESULT_ERROR : RESULT_PASSED;

	if (result == RESULT_ERROR)
	{   // no read occurred -- ERROR
		sendTestResult(result);
		if (_diagsConnPtr->in.device == SYS_ID)
			sendstringCr("Local step value not read");
		else
			sendstringCr("CAN step value not returned");
		_diagsNextSubtestState = DIAGS_SUBSTEST_STEP_COUNT_FINISHED; // no point in continuing
	}
	else
	{
		sendCr();
	}
}

////////////////////////////////////////////////////////////////////////////////

void checkStepPulse(connectivityIndex_t connIndex)
{
	_diagsConnPtr = getConnectivityListPtr(connIndex);
	if (_diagsConnPtr->connectivityIndex != CONNECTIVITY_NONE)
	{
		_diagsSubtestDevice = _diagsConnPtr->in.device;
		runDiagSubtest(DIAGS_SUBSTEST_STEP_COUNT_SETUP);
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void sleepUs(uint32_t microseconds)
{
	TIM2->EGR |= TIM_EGR_UG; // force the CNT and PSC to reset
	while (TIM2->CNT < microseconds)
		;   // killing time.
}

void sleepMs(uint32_t milliseconds)
{
	sleepUs(milliseconds * (uint32_t)1000);
}

void sleepSec(uint32_t seconds)
{
	sleepUs(seconds * (uint32_t)1000000);
}

////////////////////////////////////////////////////////////////////////////////

void diagsExit(boolean abort)
{
	setEMO(DISABLE);

	_diagsEnabled = FALSE;

	if (abort)
	{
		sprintf(_rptStr, "ABORTING TEST"); sendHeaderStr(-1, _rptStr);
	}
	sprintf(_rptStr, "Total Test Time = %3.3f seconds", (float)_diagsClockCount/1000.0f); sendHeaderStr(-1, _rptStr);
	sprintf(_rptStr, "Total Errors = %d - %s", (int)_diagsTotalErrors, (_diagsTotalErrors==0)?"SHIP IT!!!":"FIX IT!!!"); sendHeaderStr(-1, _rptStr);
	pauseToTransmitBufferToEmpty();

	if (_diagsTotalErrors == 0)
	{
		_diagsLedDisplayMode =  DIAG_LED_NIGHT_RIDER;
	}
	else
	{
		_diagsLedDisplayMode =  DIAG_LED_BLINK_FAST;
	}

	_MailBoxes._hostTrafficReportingRate = _diagsSaveReportingRate[0]; // restore reporting rate
	AutoReportXYZLocation = _diagsSaveReportingRate[1];
	AutoReportFlowRate = _diagsSaveReportingRate[2];

	canPackIntoTxQueue1x16(CAN_WRITE, 0, CAN_MSG_DIAG_IO_INIT, 0xFF, BUFFERED_MSG, 0);  // reset SWD port

	sendstringCr(">MC:M629: S0:"); // send fake SendCurrentMcodeExecutionNotice for to close log file
	sprintf(_tmpStr, ">MC:M7734: S%d:", (int)_diagsTotalErrors);
	sendstringCr(_tmpStr); // send fake SendCurrentMcodeExecutionNotice for end of M7734
	//XXX NVIC_SystemReset();
}

////////////////////////////////////////////////////////////////////////////////

void disableAllHss(void)
{
	int i;
	for (i=0; i<NUM_HSS_PINS; i++)
	{
		if (i != DRAIN1_INDEX)
		{   // DRAIN1 is used to control EMO, so don't change it's state
			changeHssDuty(&HighSideSwitches[i], HSS_DUTY_CYCLE_OFF);
			outputControlBit(&HighSideSwitches[i].Output, DEASSERTED);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void canControlHss(byte device, int hss, pinStateValue_t state)
{
	diagWritePin(device, (hss==HH_AUX_SWITCH ? HH_HSS1 : HH_HSS2), state);
//  canPackIntoTxQueue4x16(CAN_WRITE, device, CAN_MSG_SWITCH_DUTY, hss, BUFFERED_MSG,
//          1, (state == HIGH) ? 100 : 0, (PULSE_TRAIN_ON << 8) | INVERT_SIGNAL_OFF, 0);
}

////////////////////////////////////////////////////////////////////////////////

void setEMO(FunctionalState state)
{
	diagInitPin(SYS_ID, PA7, OUTPUT_PP_2MHZ);
	pinWrite(PA7, (state == ENABLE) ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////

void diagInit(uint16_t testInit)
{
	switch (testInit)
	{
	case 0: // default has no extra init
		break;
	case 1:
		disableAllHss();
		break;
	default:
		sprintf(_errorStr, "Unknown testInit value (%d) for testNum %d",
				testInit, _diagsTests[_diagsCurrentTestIndex].testNum);
		sendError(_errorStr);
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void runDiagTest(int testIndex)
{
	_diagsCurrentTestIndex = testIndex;

	int testNum = _diagsTests[testIndex].testNum;
	int testInit = _diagsTests[testIndex].testInit;

	diagInit(testInit);

	switch(testNum)
	{
	case 0: setEMO(DISABLE); sendHeaderStr(0, "TESTING 102207 Rev 7 Board"); break;
	case 1: sendHeaderStr(0, ""); break;
	case 2: sprintf(_rptStr, "MCU: 0x%03x 0x%04x", (int)_sysInfoPtr->mcuDeviceID, (int)_sysInfoPtr->mcuRevisionID); sendHeaderStr(1, _rptStr); break;
	case 3: sprintf(_rptStr, "UID: %s", getUIDString(_tmpStr)); sendHeaderStr(1, _rptStr); break;
	case 4: sprintf(_rptStr, "HYDRA_DIAGS_%d.%d%c", (int)_sysInfoPtr->softwareMajorVersion, (int)_sysInfoPtr->softwareMinorVersion, _sysInfoPtr->softwareTweakVersion); sendHeaderStr(1, _rptStr); break;

	//=======================================================================================

	case 10: sendHeaderStr(-1, "Reset all CANbus devices"); break;
	case 11: sprintf(_rptStr, "Wait (%d sec) for canbus devices to reset and register", DIAGS_SECONDS_TO_REGISTER); sendHeaderStr(0, _rptStr); break;
	case 12: sendSetSwResetBit(0); break;

	//=======================================================================================

	case 20: sendHeaderStr(-1, "Checking ATX Power Supply"); break;
	case 21: sendHeaderStr(1, "Checking ATX 5V"); break;
	case 22: checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PF7, ADC_5V, "+5V:J20,J12,J13:J17-1:PROBE:J17-11:RN2"); break;
	case 23: checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PF9, ADC_5V, "+5V:J20,J12,J13:J18-1:PROBE:J18-11:RN2"); break;

	case 30: sendHeaderStr(1, "Checking ATX 12V"); break;
	case 31: checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PF8, ADC_12V, "+12V:J20,J7,J8,J9,J12,J13:J17-4:PROBE:J17-12:RN2"); break;
	case 32: checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PF10, ADC_12V, "+12V:J20,J7,J8,J9,J12,J13:J18-4:PROBE:J18-12:RN2"); break;

	//=======================================================================================

	case 50: sendHeaderStr(-1, "Verify canbus connections");  break;

	case 51: sendHeaderStr(1, "Check CAN1:  J23-PB8,PB9:U5:R17,R20,R21,C23:->"); break;
	case 52: checkCanDeviceRegistered(CAN_MX, "J29-23,24");  break;     // doubled up with CAN_MY
	case 53: checkCanDeviceRegistered(CAN_MY, "J29-23,24");  break;
	case 54: checkCanDeviceRegistered(CAN_MZ, "J49-oddPins");  break;
	case 55: checkCanDeviceRegistered(CAN_MA, "J49-evenPins");  break;
	case 56: checkCanDeviceRegistered(CAN_MB, "J50-oddPins");  break;
	case 57: checkCanDeviceRegistered(CAN_MC, "J50-evenPins");  break;

	case 61: sendHeaderStr(1, "Check CAN2:  J23-PB5,PB6:U4:R16,R18,R19,C22:->");  break;
	case 62: checkCanDeviceRegistered(CAN_PX, "J51-oddPins");  break;   // doubled up with CAN_PZ
	case 63: checkCanDeviceRegistered(CAN_PY, "J51-evenPins");  break;  // doubled up with CAN_PA
	case 64: checkCanDeviceRegistered(CAN_PZ, "J51-oddPins");  break;
	case 65: checkCanDeviceRegistered(CAN_PA, "J51-evenPins");  break;
	case 66: checkCanDeviceRegistered(CAN_PB, "J52-oddPins");  break;
	case 67: checkCanDeviceRegistered(CAN_PC, "J52-evenPins");  break;

	//=======================================================================================

	case 100: sendHeaderStr(-1, "Verify extra power connections"); break;
	case 101: sendHeaderStr(1, "Checking EMO pressed (PA7:Drain1:Relay_Ctrl)"); break;
	case 102: setEMO(DISABLE);  break;
	case 103: checkViaAdc(CAN_PZ, DIAG_ADC_CAN_PA3,  ADC_GNDP, "12V_SW:K2_Relay:(J5/J21/J24/J30/J49-52):J24-3"); break;
	case 104: checkViaAdc(CAN_PB, DIAG_ADC_CAN_PA2,  ADC_GNDP, "12V_SW:K2_Relay:(J5/J21/J24/J30/J49-52):J49-9"); break;
	case 105: checkViaAdc(CAN_PB, DIAG_ADC_CAN_PA3,  ADC_GNDP, "48V_SW:J3-1,2:K1_Relay:F1_Fuse:J1-27to30,J3-5"); break;

	case 111: sendHeaderStr(1, "Checking EMO released (PA7:Drain1:Relay_Ctrl)"); break;
	case 112: setEMO(ENABLE);  break;
	case 113: checkViaAdc(CAN_PZ, DIAG_ADC_CAN_PA3,  ADC_12VP, "12V_SW:K2_Relay:(J5/J21/J24/J30/J49-52):J24-3"); break;
	case 114: checkViaAdc(CAN_PB, DIAG_ADC_CAN_PA2,                                                                                    ADC_12VP, "12V_SW:K2_Relay:(J5/J21/J24/J30/J49-52):J49-9"); break;
	case 115: checkViaAdc(CAN_PB, DIAG_ADC_CAN_PA3,  ADC_12VP, "48V_SW:J3-1,2:K1_Relay:F1_Fuse:J1-27to30,J3-5"); break;

	case 120: sendHeaderStr(1, "Checking 3.3V Supply (5V to 3.3V regulator is on the 429 module)"); break;
	case 121: checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PA6, ADC_3_3V, "+3.3V:R10:J19-7:PROBE:"); break;

	case 130: sendHeaderStr(1, "Checking other 5V supply connections"); break;
	case 131: checkViaAdc(CAN_PB, DIAG_ADC_CAN_RTD1, ADC_5VR, "+5V:J22-1"); break;
	case 132: checkViaAdc(CAN_PB, DIAG_ADC_CAN_RTD2, ADC_5VR, "+5V:J49-7"); break;
	case 133: checkViaAdc(CAN_PA, DIAG_ADC_CAN_RTD1, ADC_5VR, "+5V:J29-9"); break;
	case 134: checkViaAdc(CAN_MX, DIAG_ADC_CAN_RTD1, ADC_5VR, "5V_F1:5V:F9_Fuse:J38-15"); break;
	case 135: checkViaAdc(CAN_MZ, DIAG_ADC_CAN_RTD1, ADC_5VR, "5V_F1:5V:F9_Fuse:J40-15"); break;
	case 136: checkViaAdc(CAN_MB, DIAG_ADC_CAN_RTD1, ADC_5VR, "5V_F1:5V:F9_Fuse:J42-15"); break;


	case 140: sendHeaderStr(1, "Checking other 12V supply connections->"); break;
	case 141: canControlHss(CAN_MY, HH_HTR_SWITCH, HIGH); checkViaAdc(CAN_MY, DIAG_ADC_CAN_RTD1, ADC_12VR, "12V_F1:+12V:F10_Fuse:J39-1"); break;
	case 142: canControlHss(CAN_MY, HH_HTR_SWITCH, LOW);  break;
	case 143: canControlHss(CAN_MA, HH_HTR_SWITCH, HIGH); checkViaAdc(CAN_MA, DIAG_ADC_CAN_RTD1, ADC_12VR, "12V_F1:+12V:F10_Fuse:J41-1"); break;
	case 144: canControlHss(CAN_MA, HH_HTR_SWITCH, LOW);  break;
	case 145: canControlHss(CAN_MC, HH_HTR_SWITCH, HIGH); checkViaAdc(CAN_MC, DIAG_ADC_CAN_RTD1, ADC_12VR, "12V_F1:+12V:F10_Fuse:J43-1"); break;
	case 146: canControlHss(CAN_MC, HH_HTR_SWITCH, LOW);  break;
	case 147: checkViaAdc(CAN_PA, DIAG_ADC_CAN_RTD2, ADC_12VR, "12V_SW2:12V_SW:F7_Fuse:J29-12"); break;
#if 0
	case 150:
		setEMO(ENABLE);
		diagInitPin(CAN_MX, HH_SWC, HH_OUT_PP_02MHZ);
		diagWritePin(CAN_MX, HH_SWC, LOW);
		diagWritePin(CAN_MX, HH_SWC, HIGH);
		diagInitPin(CAN_MX, HH_SWD, HH_OUT_PP_02MHZ);
		diagWritePin(CAN_MX, HH_SWD, LOW);
		diagWritePin(CAN_MX, HH_SWD, HIGH);

		diagWritePin(CAN_MX, HH_SWC, LOW);
		diagWritePin(CAN_MX, HH_SWC, HIGH);
		diagWritePin(CAN_MX, HH_SWD, LOW);
		diagWritePin(CAN_MX, HH_SWD, HIGH);
		break;
#else
	case 150: sendHeaderStr(1, "Checking AUX_POWER connections (enabled)->"); break;
#endif
	case 151: pinWrite(HSS_AUX_PWR1, 1); checkViaAdc(CAN_PX, DIAG_ADC_CAN_RTD1, ADC_12VR, "AUX_POWER_1:J23-PH15:U15-8/9:R28:C55:J44-1"); break;
	case 152: pinWrite(HSS_AUX_PWR1, 1); checkViaAdc(CAN_PX, DIAG_ADC_CAN_RTD2, ADC_12VR, "AUX_POWER_1:J23-PH15:U15-8/9:R28:C55:J37-1"); break;
	case 153: pinWrite(HSS_AUX_PWR2, 1); checkViaAdc(CAN_PX, DIAG_ADC_CAN_PA2,  ADC_12VP, "AUX_POWER_2:J23-PH13:U15-10/11:R28:C55:J44-2"); break;
	case 154: pinWrite(HSS_AUX_PWR2, 1); checkViaAdc(CAN_PX, DIAG_ADC_CAN_PA3,  ADC_12VP, "AUX_POWER_2:J23-PH13:U15-10/11:R28:C55:J35-1"); break;
	case 155:                            checkViaAdc(CAN_PY, DIAG_ADC_CAN_RTD1, ADC_12VR, "AUX_POWER_3:+12V:F5_Fuse:J32-1"); break;
	case 156: pinWrite(HSS_AUX_PWR4, 1); checkViaAdc(CAN_PY, DIAG_ADC_CAN_RTD2, ADC_12VR, "AUX_POWER_4:J23-PH14:U16-8/9:R29:C54:J44-3"); break;
	case 157: pinWrite(HSS_AUX_PWR5, 1); checkViaAdc(CAN_PY, DIAG_ADC_CAN_PA2,  ADC_12VP, "AUX_POWER_5:J23-PC9:U16-10/11:R29:C54:J44-4"); break;
	case 158: pinWrite(HSS_AUX_PWR6, 1); checkViaAdc(CAN_PY, DIAG_ADC_CAN_PA3,  ADC_12VP, "AUX_POWER_6:J23-PG14:U18-8/9:R22:C65:J21-14"); break;
	case 159: pinWrite(HSS_AUX_PWR7, 1); checkViaAdc(CAN_PZ, DIAG_ADC_CAN_RTD1, ADC_12VR, "AUX_POWER_7:J23-PG13:U18-10/11:R22:C65:J21-16"); break;
	case 160: pinWrite(HSS_AUX_PWR8, 1); checkViaAdc(CAN_PZ, DIAG_ADC_CAN_RTD2, ADC_12VR, "AUX_POWER_8:J23-PC1:U19-8/9:R44:C66:J29-33"); break;
	case 161: pinWrite(HSS_AUX_PWR9, 1); checkViaAdc(CAN_PZ, DIAG_ADC_CAN_PA2,  ADC_12VP, "AUX_POWER_9:J23-PA2:U19-10/11:R244:C66:J29-35"); break;

	case 170: sendHeaderStr(1, "Checking AUX_POWER connections (disabled)->"); break;
	case 171: pinWrite(HSS_AUX_PWR1, 0); checkViaAdc(CAN_PX, DIAG_ADC_CAN_RTD1, ADC_GNDR, "AUX_POWER_1:J23-PH15:U15-8/9:R28:C55:J44-1"); break;
	case 172: pinWrite(HSS_AUX_PWR1, 0); checkViaAdc(CAN_PX, DIAG_ADC_CAN_RTD2, ADC_GNDR, "AUX_POWER_1:J23-PH15:U15-8/9:R28:C55:J37-1"); break;
	case 173: pinWrite(HSS_AUX_PWR2, 0); checkViaAdc(CAN_PX, DIAG_ADC_CAN_PA2,  ADC_GNDP, "AUX_POWER_2:J23-PH13:U15-10/11:R28:C55:J44-2"); break;
	case 174: pinWrite(HSS_AUX_PWR2, 0); checkViaAdc(CAN_PX, DIAG_ADC_CAN_PA3,  ADC_GNDP, "AUX_POWER_2:J23-PH13:U15-10/11:R28:C55:J35-1"); break;
	case 175: break; // no control over aux_pwr_3
	case 176: pinWrite(HSS_AUX_PWR4, 0); checkViaAdc(CAN_PY, DIAG_ADC_CAN_RTD2, ADC_GNDR, "AUX_POWER_4:J23-PH14:U16-8/9:R29:C54:J44-3"); break;
	case 177: pinWrite(HSS_AUX_PWR5, 0); checkViaAdc(CAN_PY, DIAG_ADC_CAN_PA2,  ADC_GNDP, "AUX_POWER_5:J23-PC9:U16-10/11:R29:C54:J44-4"); break;
	case 178: pinWrite(HSS_AUX_PWR6, 0); checkViaAdc(CAN_PY, DIAG_ADC_CAN_PA3,  ADC_GNDP, "AUX_POWER_6:J23-PG14:U18-8/9:R22:C65:J21-14"); break;
	case 179: pinWrite(HSS_AUX_PWR7, 0); checkViaAdc(CAN_PZ, DIAG_ADC_CAN_RTD1, ADC_GNDR, "AUX_POWER_7:J23-PG13:U18-10/11:R22:C65:J21-16"); break;
	case 180: pinWrite(HSS_AUX_PWR8, 0); checkViaAdc(CAN_PZ, DIAG_ADC_CAN_RTD2, ADC_GNDR, "AUX_POWER_8:J23-PC1:U19-8/9:R44:C66:J29-33"); break;
	case 181: pinWrite(HSS_AUX_PWR9, 0); checkViaAdc(CAN_PZ, DIAG_ADC_CAN_PA2,  ADC_GNDP, "AUX_POWER_9:J23-PA2:U19-10/11:R244:C66:J29-35"); break;

	case 190: sendHeaderStr(1, "Checking DRAINS (enabled)"); break;
	case 191: pinWrite(DRAIN2, 1); checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PA3, ADC_GND, "Active DRAIN2:J23-PC4:J21-5:PROBE:J19-8"); break;
	case 192: pinWrite(DRAIN3, 1); checkViaAdc(CAN_PA, DIAG_ADC_CAN_PA2, ADC_GNDP, "Active DRAIN3:J23-PC5:J29-38"); break;
	case 193: pinWrite(DRAIN4, 1); checkViaAdc(CAN_PA, DIAG_ADC_CAN_PA3, ADC_GNDP, "Active DRAIN4:J23-PB11:J29-37"); break;

	case 195: sendHeaderStr(1, "Checking DRAINS (disabled)"); break;
	case 196: pinWrite(DRAIN2, 0); checkViaAdc(SYS_ID, DIAG_ADC_LOCAL_PA3, ADC_DRAIN, "Inactive DRAIN2:J23-PC4:J21-5:PROBE:J19-8"); break;
	case 197: pinWrite(DRAIN3, 0); checkViaAdc(CAN_PA, DIAG_ADC_CAN_PA2, ADC_DRAINP, "Inactive DRAIN3:J23-PC5:J29-38"); break;
	case 198: pinWrite(DRAIN4, 0); checkViaAdc(CAN_PA, DIAG_ADC_CAN_PA3, ADC_DRAINP, "Inactive DRAIN4:J23-PB11:J29-37"); break;

	//=======================================================================================

	case 200: sendHeaderStr(-1, "Verifying static I/O"); break;
	//case 201: checkLocalIoState(PB2, LOW, "BOOT1: GND:R35:J23-PB2 (If failed, check that R35 is installed and R37 isn't)"); break;
	case 201: checkLocalIoState(PB2, LOW, "BOOT1: GND:R35:J23-PB2 (If FAIL, check PB2->GND wire jumper is installed on 429 board)"); break;

	case 202: checkLocalIoState(PA6, HIGH, "+3.3V:R23:J23-PA6 (if failed, check that R10 is installed)"); break;

	case 300:
	case 500:
		sprintf(_tmpStr, "Connectivity tests: verifying basic signal wiring (background=%d)", (testNum==300) ? (int)LOW : (int)HIGH);
		sendHeaderStr(-1, _tmpStr);
		initializeConnectivityTests();  // set all involved output pins to the background state
		setConnectivityBackground((testNum==300) ? LOW : HIGH);
		break;
	case 301: case 501: sendHeaderStr(1, "Misc I/O"); break;
	case 302: case 502: checkConnectivity(CONNECTIVITY_GEN_0); break;
	case 303: case 503: checkConnectivity(CONNECTIVITY_GEN_1); break;
	case 304: case 504: checkConnectivity(CONNECTIVITY_GEN_2); break;
	case 305: case 505: checkConnectivity(CONNECTIVITY_GEN_3); break;
	case 306: case 506: checkConnectivity(CONNECTIVITY_GEN_4); break;
	case 307: case 507: checkConnectivity(CONNECTIVITY_GEN_5); break;
	case 308: case 508: checkConnectivity(CONNECTIVITY_GEN_6); break;
	case 309: case 509: checkConnectivity(CONNECTIVITY_GEN_7); break;
	case 310: case 510: checkConnectivity(CONNECTIVITY_GEN_8); break;
	case 311: case 511: checkConnectivity(CONNECTIVITY_GEN_9); break;

	case 320: case 520: sendHeaderStr(1, "Motor X"); break;
	case 321: case 521: checkConnectivity(CONNECTIVITY_MX_ENB); break;
	case 322: case 522: checkConnectivity(CONNECTIVITY_MX_DIR); break;
	case 323: case 523: checkConnectivity(CONNECTIVITY_MX_FLT); break;
	case 324: case 524: checkConnectivity(CONNECTIVITY_MX_HOM); break;
	case 325: case 525: checkConnectivity(CONNECTIVITY_MX_LM1); break;
	case 326: case 526: checkConnectivity(CONNECTIVITY_MX_LM2); break;
	case 327: case 527: checkConnectivity(CONNECTIVITY_PX_ENB); break;
	case 328: case 528: checkConnectivity(CONNECTIVITY_PX_DIR); break;
	case 329: case 529: checkConnectivity(CONNECTIVITY_PX_FLT); break;

	case 340: case 540: sendHeaderStr(1, "Motor Y"); break;
	case 341: case 541: checkConnectivity(CONNECTIVITY_MY_ENB); break;
	case 342: case 542: checkConnectivity(CONNECTIVITY_MY_DIR); break;
	case 343: case 543: checkConnectivity(CONNECTIVITY_MY_FLT);  break;
	case 344: case 544: checkConnectivity(CONNECTIVITY_MY_HOM); break;
	case 345: case 545: checkConnectivity(CONNECTIVITY_MY_LM1); break;
	case 346: case 546: checkConnectivity(CONNECTIVITY_MY_LM2); break;
	case 347: case 547: checkConnectivity(CONNECTIVITY_PY_ENB); break;
	case 348: case 548: checkConnectivity(CONNECTIVITY_PY_DIR); break;
	case 349: case 549: checkConnectivity(CONNECTIVITY_PY_FLT); break;

	case 360: case 560: sendHeaderStr(1, "Motor Z"); break;
	case 361: case 561: checkConnectivity(CONNECTIVITY_MZ_ENB); break;
	case 362: case 562: checkConnectivity(CONNECTIVITY_MZ_DIR); break;
	case 363: case 563: checkConnectivity(CONNECTIVITY_MZ_FLT); break;
	case 364: case 564: checkConnectivity(CONNECTIVITY_MZ_HOM); break;
	case 365: case 565: checkConnectivity(CONNECTIVITY_MZ_LM1); break;
	case 366: case 566: checkConnectivity(CONNECTIVITY_MZ_LM2); break;
	case 367: case 567: checkConnectivity(CONNECTIVITY_PZ_ENB); break;
	case 368: case 568: checkConnectivity(CONNECTIVITY_PZ_DIR); break;
	case 369: case 569: checkConnectivity(CONNECTIVITY_PZ_FLT); break;

	case 380: case 580: sendHeaderStr(1, "Motor A"); break;
	case 381: case 581: checkConnectivity(CONNECTIVITY_MA_ENB); break;
	case 382: case 582: checkConnectivity(CONNECTIVITY_MA_DIR); break;
	case 383: case 583: checkConnectivity(CONNECTIVITY_MA_FLT); break;
	case 384: case 584: checkConnectivity(CONNECTIVITY_MA_HOM); break;
	case 385: case 585: checkConnectivity(CONNECTIVITY_MA_LM1); break;
	case 386: case 586: checkConnectivity(CONNECTIVITY_MA_LM2); break;
	case 387: case 587: checkConnectivity(CONNECTIVITY_PA_ENB); break;
	case 388: case 588: checkConnectivity(CONNECTIVITY_PA_DIR); break;
	case 389: case 589: checkConnectivity(CONNECTIVITY_PA_FLT); break;

	case 400: case 600: sendHeaderStr(1, "Motor B"); break;
	case 401: case 601: checkConnectivity(CONNECTIVITY_MB_ENB); break;
	case 402: case 602: checkConnectivity(CONNECTIVITY_MB_DIR); break;
	case 403: case 603: checkConnectivity(CONNECTIVITY_MB_FLT); break;
	case 404: case 604: checkConnectivity(CONNECTIVITY_MB_HOM); break;
	case 405: case 605: checkConnectivity(CONNECTIVITY_MB_LM1); break;
	case 406: case 606: checkConnectivity(CONNECTIVITY_MB_LM2); break;
	case 407: case 607: checkConnectivity(CONNECTIVITY_PB_ENB); break;
	case 408: case 608: checkConnectivity(CONNECTIVITY_PB_DIR); break;
	case 409: case 609: checkConnectivity(CONNECTIVITY_PB_FLT); break;

	case 420: case 620: sendHeaderStr(1, "Motor C"); break;
	case 421: case 621: checkConnectivity(CONNECTIVITY_MC_ENB); break;
	case 422: case 622: checkConnectivity(CONNECTIVITY_MC_DIR); break;
	case 423: case 623: checkConnectivity(CONNECTIVITY_MC_FLT); break;
	case 424: case 624: checkConnectivity(CONNECTIVITY_MC_HOM); break;
	case 425: case 625: checkConnectivity(CONNECTIVITY_MC_LM1); break;
	case 426: case 626: checkConnectivity(CONNECTIVITY_MC_LM2); break;
	case 427: case 627: checkConnectivity(CONNECTIVITY_PC_ENB); break;
	case 428: case 628: checkConnectivity(CONNECTIVITY_PC_DIR); break;
	case 429: case 629: checkConnectivity(CONNECTIVITY_PC_FLT); break;


	case 700: sendHeaderStr(-1, "Verify step pulse integrity"); break;
	case 701: clearStepPins(); break;
	case 702: clearStepCounts(); break;
	case 703: executeFakeMove(100); break;

	case 710: checkStepPulse(CONNECTIVITY_MX_STP); break;
	case 711: checkStepPulse(CONNECTIVITY_MY_STP); break;
	case 712: checkStepPulse(CONNECTIVITY_MZ_STP); break;
	case 713: checkStepPulse(CONNECTIVITY_MA_STP); break;
	case 714: checkStepPulse(CONNECTIVITY_MB_STP); break;
	case 715: checkStepPulse(CONNECTIVITY_MC_STP); break;

	case 720: checkStepPulse(CONNECTIVITY_PX_STP); break;
	case 721: checkStepPulse(CONNECTIVITY_PY_STP); break;
	case 722: checkStepPulse(CONNECTIVITY_PZ_STP); break;
	case 723: checkStepPulse(CONNECTIVITY_PA_STP); break;
	case 724: checkStepPulse(CONNECTIVITY_PB_STP); break;
	case 725: checkStepPulse(CONNECTIVITY_PC_STP); break;


	case 999:
		diagsExit(FALSE);
		break;

	case 1000:
		setEMO(ENABLE);
		pinWrite(HSS_AUX_PWR1, 1);
		pinWrite(HSS_AUX_PWR2, 1);
		pinWrite(HSS_AUX_PWR4, 1);
		pinWrite(HSS_AUX_PWR5, 1);
		pinWrite(HSS_AUX_PWR6, 1);
		pinWrite(HSS_AUX_PWR7, 1);
		pinWrite(HSS_AUX_PWR8, 1);
		pinWrite(HSS_AUX_PWR9, 1);
		pinWrite(DRAIN1, 1);
		pinWrite(DRAIN2, 1);
		pinWrite(DRAIN3, 1);
		pinWrite(DRAIN4, 1); // emo
		break;
	case 1001:
		setEMO(ENABLE);
		pinWrite(HSS_AUX_PWR1, 0);
		pinWrite(HSS_AUX_PWR2, 0);
		pinWrite(HSS_AUX_PWR4, 0);
		pinWrite(HSS_AUX_PWR5, 0);
		pinWrite(HSS_AUX_PWR6, 0);
		pinWrite(HSS_AUX_PWR7, 0);
		pinWrite(HSS_AUX_PWR8, 0);
		pinWrite(HSS_AUX_PWR9, 0);
		pinWrite(DRAIN4, 0);
		pinWrite(DRAIN3, 0);
		pinWrite(DRAIN2, 0);
		pinWrite(DRAIN1, 0);    // emo
		break;

	default:
		sprintf(_rptStr, "********   Programming Error: Skipping Test %d   ********", testNum); sendHeaderStr(-1, _rptStr);
		sendHeaderStr(0, "");
		return;
	};
}

////////////////////////////////////////////////////////////////////////////////

void completedAllSubtests(void)
{
	_diagsNextSubtestState = DIAGS_SUBSTEST_NONE;
	_diagsNextSubtestTime = 0;
}

////////////////////////////////////////////////////////////////////////////////

void runDiagSubtest(subtest_t test)
{
	_diagsNextSubtestState = test + 1;

	switch (test)
	{
	case DIAGS_SUBSTEST_CAN_ADC_SETUP:
		//assume head always sampling all adc channels; so no need to issue can commands to setup channels
		diagInitPin(_diagsSubtestDevice, _diagsAdcPtr->pin, HH_ANALOG_IN); // make sre pin is in state for adc
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_CAN_ADC_SETUP_TIME;
		_diagsNextSubtestState = DIAGS_SUBSTEST_CAN_ADC_READ_REQ;
		break;
	case DIAGS_SUBSTEST_CAN_ADC_READ_REQ:
		_diagsAdcReadValue = DIAGS_NO_ADC_VALUE; // set to an illegal value before read request to know if read actually occurred
		canIssueReadRequest(_diagsSubtestDevice, CAN_MSG_DIAG_ADC_READ,  _diagsAdcPtr->pin);    // pinIndex converted to channel on head
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_CAN_READ_TIME;
		_diagsNextSubtestState = DIAGS_SUBSTEST_CAN_ADC_CHECK_RESULTS;
		break;
	case DIAGS_SUBSTEST_CAN_ADC_CHECK_RESULTS:
		finishCheckCanAdc();
		completedAllSubtests();
		break;


	case DIAGS_SUBSTEST_LOCAL_ADC_SETUP:
		_diagsAdcReadValue = DIAGS_NO_ADC_VALUE;
		diagInitPin(_diagsSubtestDevice, _diagsAdcPtr->pin, ANALOG_FLOATING);
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_LOCAL_ADC_SETUP_TIME;
		_diagsNextSubtestState = DIAGS_SUBSTEST_LOCAL_ADC_CONVERT;
		break;
	case DIAGS_SUBSTEST_LOCAL_ADC_CONVERT:
		ADC_RegularChannelConfig(_diagsAdcPtr->hwADC, _diagsAdcPtr->channel, 1, ADC_SampleTime_84Cycles);
		ADC_Cmd (_diagsAdcPtr->hwADC, ENABLE);
		ADC_SoftwareStartConv(_diagsAdcPtr->hwADC);
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_LOCAL_ADC_CONVERT_TIME;
		_diagsNextSubtestState = DIAGS_SUBSTEST_LOCAL_ADC_READ_REQ;
		break;
	case DIAGS_SUBSTEST_LOCAL_ADC_READ_REQ:
		//grab local adc value
		_diagsAdcReadValue = ADC_GetConversionValue(_diagsAdcPtr->hwADC);
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_LOCAL_READ_TIME;
		_diagsNextSubtestState = DIAGS_SUBSTEST_LOCAL_ADC_CHECK_RESULTS;
		break;
	case DIAGS_SUBSTEST_LOCAL_ADC_CHECK_RESULTS:
		finishCheckLocalAdc();
		completedAllSubtests();
		break;

	case DIAGS_SUBSTEST_LOOPBACK_WRITE:
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_CAN_WRITE_TIME;
		if ((_diagsConnPtr->out.device != SYS_ID) && (_diagsConnPtr->out.pin == SWC))
		{   // was special case with dual driver for FAULT, so init driver
			diagInitPinPtr(&_diagsConnPtr->out, HH_OUT_PP_02MHZ);
		}
		diagWritePinPtr(&_diagsConnPtr->out, _diagsConnectivityTestValue);
		_diagsNextSubtestState = DIAGS_SUBSTEST_LOOPBACK_SENDSTRING;
		break;
	case DIAGS_SUBSTEST_LOOPBACK_SENDSTRING:
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_STRING_TIME;
		_diagCurrentIndentLevel = 1;
		sendConnectivityTestStr();
		_diagsNextSubtestState = DIAGS_SUBSTEST_LOOPBACK_READ;
		_diagsConnectivityReadVal = UNKNOWN_STATE;
		break;
	case DIAGS_SUBSTEST_LOOPBACK_READ:
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_CAN_READ_TIME;
		_diagsConnectivityReadVal = diagReadPinPtr(&_diagsConnPtr->in);
		_diagsNextSubtestState = DIAGS_SUBSTEST_LOOPBACK_COMPARE;
		break;
	case DIAGS_SUBSTEST_LOOPBACK_COMPARE:
		_diagCurrentIndentLevel = 2;
		connectivityTestCompare();
		diagWritePinPtr(&_diagsConnPtr->out, _diagsConnectivityBackgroundValue); // restore original state
		if ((_diagsConnPtr->out.device != SYS_ID) && (_diagsConnPtr->out.pin == SWC))
		{   // was special case with dual driver for FAULT, so turn off driver
			diagInitPinPtr(&_diagsConnPtr->out, HH_IN_FLOAT);
		}
		completedAllSubtests();
		break;

	case DIAGS_SUBSTEST_STEP_COUNT_SETUP:
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_CAN_WRITE_TIME;
		_diagsNextSubtestState = DIAGS_SUBSTEST_STEP_COUNT_HEADER;
		_diagsStepsDetected = -1;
		_diagsStepMinTimeUs = 0.0f;
		_diagsStepMaxTimeUs = 0.0f;
		_diagsStepsAvgTimeUs = 0.0f;
		canPackIntoTxQueueNoData(CAN_READ, _diagsSubtestDevice, CAN_MSG_DIAG_STEP_COUNTER, 0, BUFFERED_MSG);
		break;
	case DIAGS_SUBSTEST_STEP_COUNT_HEADER:
		_diagsNextSubtestTime = _diagsClockCount + DIAGS_STRING_TIME;   // string write
		_diagsNextSubtestState = DIAGS_SUBSTEST_STEP_COUNT_CHECK_RESULTS;
		sendStepCountHeader();
		_diagCurrentIndentLevel++;
		break;
	case DIAGS_SUBSTEST_STEP_COUNT_CHECK_RESULTS:
		_diagsNextSubtestTime = _diagsClockCount + 4*DIAGS_STRING_TIME; // string writes
		_diagsNextSubtestState = DIAGS_SUBSTEST_STEP_COUNT_FINISHED;
		checkStepCountResults();
	case DIAGS_SUBSTEST_STEP_COUNT_FINISHED:
		_diagCurrentIndentLevel--;  // restore to prior level
		completedAllSubtests();
		break;

	default:
		completedAllSubtests();
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

uint16_t getTestIndex(uint16_t testNum)
{
	int i;
	for (i=0; i<DIAGS_NUM_TESTS; i++)
	{
		if (testNum == _diagsTests[i].testNum)
		{   // found a match
			return(i);
		}
	}
	// get this far, then no match found, so return a benign pointer
	return(0);
}

////////////////////////////////////////////////////////////////////////////////

void ProcessDiags(void)
{
	if (_diagsEnabled)
	{
		if (_diagsClockCount == _diagsNextTestTime)
		{   // time to run a test
			_diagsNextTestTime += _diagsTests[_diagsNextTestIndex].timeMs;  // setup the test after this one
			runDiagTest(_diagsNextTestIndex);   // inside routine will set _diagsCurrentTestIndex
			if (_diagsRunSingleTest)
			{
				if (_diagsSingleTestLoopCount == 0)
					_diagsNextTestIndex = getTestIndex(999);    // finish
				else
					_diagsSingleTestLoopCount--;    // else repeat same index
			}
			else
			{
				_diagsNextTestIndex++;
			}
			if (_diagsNextTestIndex == DIAGS_NUM_TESTS)
			{   // last test has been processed
				_diagsEnabled = FALSE;
			}
		}
		else if (_diagsClockCount > _diagsNextTestTime)
		{
			sprintf(_errorStr, "DIAGS TIME: testIndex=%d currTime=%d nextTime=%d",
					(int)_diagsCurrentTestIndex, (int)_diagsClockCount, (int)_diagsNextTestTime);
			barf(_errorStr);
		}
		else if ((_diagsClockCount == _diagsNextSubtestTime) && (_diagsNextSubtestTime > 0))
		{
			_diagsNextSubtestTime = 0;
			runDiagSubtest(_diagsNextSubtestState);
		}
		_diagsClockCount += DIAGS_MS_PER_LOOP;
	}
}

////////////////////////////////////////////////////////////////////////////////

void runHydraDiagnostics(void)  // 102207 rev 7 manufacturing test  (T, L, E) (Test, Sms, Loop, EallowableErrors, Allow)
{
	sendstringCr(">MC:M629: S1:"); // send fake SendCurrentMcodeExecutionNotice for to open log file
	_diagsSaveReportingRate[0] = _MailBoxes._hostTrafficReportingRate;
	_diagsSaveReportingRate[1] = AutoReportXYZLocation;
	_diagsSaveReportingRate[2] = AutoReportFlowRate;
	_MailBoxes._hostTrafficReportingRate = 0; // no reporting during test
	AutoReportXYZLocation = FALSE;
	AutoReportFlowRate = FALSE;

	_diagsEnabled = FALSE;
	_diagsClockCount = 0;
	_diagsNextTestTime = DIAGS_FIRST_TEST_TIME;
	_diagsTotalErrors = 0;
	_diagsAllowedErrors = (ARG_E_PRESENT) ? (int)ARG_E : MAX_ALLOWABLE_ERRORS;

	adcInit(ADC1);
	adcInit(ADC3);
	InitTimer2usec();

	InitGPIO(); // start with going defaults
	_diagsLedDisplayMode = DIAG_LED_OFF;
	_diagLedDisplayCnt = 0;
	_diagsLedValue = 0;
	diagSetLed(0, 0, 0);
	initAllHss();
	int i;
	for (i=0; i<NUM_HSS_FUNC; i++)
	{
		hssFuncToPinIndex[i] = NO_FUNCTION_HSS;
	}

	disableAllHss();
	setEMO(DISABLE);

	if (ARG_T_PRESENT)
	{   // single test only, optional permanent looping
		_diagsRunSingleTest = TRUE;
		_diagsNextTestIndex = getTestIndex((int)ARG_T);
		_diagsSingleTestLoopCount = imax(1, (ARG_L_PRESENT) ? (uint32_t)ARG_L : 1);
	}
	else
	{   // run the whole suite
		_diagsRunSingleTest = FALSE;
		_diagsNextTestIndex = 0;    // start with the first test
	}
	_diagsEnabled = TRUE;
}

////////////////////////////////////////////////////////////////////////////////

#endif

