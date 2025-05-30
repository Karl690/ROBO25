
////////////////////////////////////////////////////////////////////////////////
//
// File:    headSetup.c
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
#include "hyrel.h"
#include "headSetup.h"
#include "Hydra_can.h"
#include "mailbox.h"

#define ACT_HIGH    TRUE        // ACTIVE_HIGH
#define ACT_LOW     FALSE       // ACTIVE_LOW
#define MIN_DEG     (MIN_TEMP >> TEMP_FRAC_BITS)    // MIN Temp in DEGREES
#define MAX_DEG     (MAX_TEMP >> TEMP_FRAC_BITS)    // MAX Temp in DEGREES

#define SLAM    1


////////////////////////////////////////////////////////////////////////////////

const devInitStruct devInit[] = {
// **** HEADS CAN
//                                              SW0/AUX/HSS1                                                max       SW1/HTR/HSS2                                                  max                 pwm    allowable    allow-
//devType (soapDevType_t)       Dev Type Str    devSwitchType_t  slam   polarity    minTemp     maxTemp     DC        devSwitchType_t    slam   polarity    minTemp     maxTemp     DC      devMotor_t  freq   add-on       ablePCB
//=======================       =============   ===============  ====   =========   =======     =======     ===       ===============    ====   =========   =======     =======     ===     ==========  =====  ===========  =======
{ SOAP_DEV_TYPE_MK1,            "MK1",          {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        260,    100 }}, MOTOR_10K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_MK2,            "MK2",          {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        260,    100 }}, MOTOR_10K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_MK450,          "MK450",        {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        450,    100 }}, MOTOR_10K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_VOLCANO_15,     "Volcano_15",   {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        100,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_VOLCANO_25,     "Volcano_25",   {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        100,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_VOLCANO_50,     "Volcano_50",   {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        100,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_KRAKATOA_15,    "Krakatoa_15",  {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        200,     80 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_TAMBORA_15,     "Tambora_15",   {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        300,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_MJW },
{ SOAP_DEV_TYPE_RH_SYRINGE,     "RH_Syringe",   {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT_COOL,    1,  ACT_HIGH,      -100,        100,    100 }}, MOTOR_20K,      0, ADD_NONE,    PCB_J   },

{ SOAP_DEV_TYPE_EMO_25,         "Emo_25",       {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MIN_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_EMO_50,         "Emo_50",       {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MIN_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_SDS,            "SDS",          {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_SDS_5,          "SDS_5",        {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_SDS_10,         "SDS_10",       {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_SDS_30,         "SDS_30",       {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_SDS_60,         "SDS_60",       {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_SDS_150,        "SDS_150",      {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_20K,      0, ADD_CPROBE,  PCB_ALL_HH },

{ SOAP_DEV_TYPE_HOTBED_100,     "HotBed_100",   {{ SW_TEMP_HEAT,    1,  ACT_HIGH,   MIN_DEG,         55,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        120,    100 }}, MOTOR_ZMOVE,    0, ADD_AM23XX,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_HOTBED_120,     "HotBed_120",   {{ SW_TEMP_HEAT,    1,  ACT_HIGH,   MIN_DEG,         55,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        130,    100 }}, MOTOR_ZMOVE,    0, ADD_AM23XX,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_HOTBED_200,     "HotBed_200",   {{ SW_TEMP_HEAT,    1,  ACT_HIGH,   MIN_DEG,         55,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,        231,    100 }}, MOTOR_ZMOVE,    0, ADD_AM23XX,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_HOTCOLDBED,     "HotColdBed",   {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT_COOL,    1,  ACT_HIGH,      -100,        150,    100 }}, MOTOR_ZMOVE,    0, ADD_NONE,    PCB_J   },

{ SOAP_DEV_TYPE_UVLIGHT_RAY,    "UvLight_Ray",  {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 30000, ADD_NONE,    PCB_ALL_HH },
{ SOAP_DEV_TYPE_LASER_2,        "Laser_2",      {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 30000, ADD_NONE,    PCB_ALL_HH },
{ SOAP_DEV_TYPE_LASER_5,        "Laser_5",      {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 30000, ADD_NONE,    PCB_ALL_HH },
{ SOAP_DEV_TYPE_LASER_6,        "Laser_6",      {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 30000, ADD_NONE,    PCB_ALL_HH },
{ SOAP_DEV_TYPE_LASER_40,       "Laser_40",     {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 50000, ADD_NONE,    PCB_J   },
{ SOAP_DEV_TYPE_LASER_80,       "Laser_80",     {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 50000, ADD_NONE,    PCB_J   },
{ SOAP_DEV_TYPE_LASER_150,      "Laser_150",    {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,        35,         55,    100 }}, MOTOR_NONE, 50000, ADD_NONE,    PCB_J   },

{ SOAP_DEV_TYPE_INKJET1,        "InkJet_1",     {{ SW_NOT_USED,     0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,  1241, ADD_CPROBE,  PCB_J   },
{ SOAP_DEV_TYPE_INKJET2,        "InkJet_2",     {{ SW_NOT_USED,     0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,  1241, ADD_CPROBE,  PCB_J   },

{ SOAP_DEV_TYPE_QUIET_STORM,    "Quiet_Storm",  {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_MICROSCOPE,     "Microscope",   {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_FIL_DISP,       "FilDispense",  {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_TEMP_HEAT,         1,  ACT_HIGH,   MIN_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_HX711,   PCB_J   },
{ SOAP_DEV_TYPE_CO2_LENS,       "LaserLens",    {{ SW_DC,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_DC,                0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_DIGITRAM,       "DigiTram",     {{ SW_NOT_USED,     0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },

{ SOAP_DEV_TYPE_DRILL_MILL,     "Drill_Mill",   {{ SW_TIM_PWM,      0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_DC,                0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_SPDL,  5000, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_3PH_SPINDLE,    "3ph_Spindle",  {{ SW_TIM_PWM,      0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_DC,                0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_SPDL,  5000, ADD_CPROBE,  PCB_ALL_HH },

{ SOAP_DEV_TYPE_PICKPLACE,      "PickPlace",    {{ SW_ON_OFF,       0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_ON_OFF,            0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_10K,      0, ADD_NONE,    PCB_ALL_HH },
{ SOAP_DEV_TYPE_BITSCOPE,       "BitScope",     {{ SW_NOT_USED,     0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_NOT_USED,          0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_10K,      0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_GENERIC_HEAD,   "GenericHead",  {{ SW_ALL_OPTIONS,  0,  ACT_HIGH,   MIN_DEG,    MAX_DEG,    100 },  { SW_ALL_OPTIONS,       0,  ACT_HIGH,   MIN_DEG,    MAX_DEG,    100 }}, MOTOR_20K,  10000, ADD_CPROBE,  PCB_ALL_HH },

{ SOAP_DEV_TYPE_SONICATOR,      "Sonicator",    {{ SW_ALL_OPTIONS,  0,  ACT_HIGH,   MIN_DEG,    MAX_DEG,    100 },  { SW_ALL_OPTIONS,       0,  ACT_HIGH,   MIN_DEG,    MAX_DEG,    100 }}, MOTOR_20K,  10000, ADD_CPROBE,  PCB_405 },

{ SOAP_DEV_TYPE_CAN_MOTOR,      "CanMotor",     {{ SW_NOT_USED,  	0,  ACT_HIGH,   MIN_DEG,    MAX_DEG,    100 },  { SW_NOT_USED,			0,  ACT_HIGH,   MIN_DEG,    MAX_DEG,    100 }}, MOTOR_AXIS,  10000, ADD_NONE,   PCB_ENC },

// THESE 4 ENTIRES MUST BE LEFT AS THE LAST 4 ENTRIES OF THE DECLARATION.
{ SOAP_DEV_TYPE_LAST_ENTRY,     "LAST_ENTRY",   {{ SW_UNKNOWN,      0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_UNKNOWN,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_BAD_COMBO,      "BAD_COMBO",    {{ SW_UNKNOWN,      0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_UNKNOWN,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_UNKNOWN,        "UNKNOWN_DEV",  {{ SW_UNKNOWN,      0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_UNKNOWN,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
{ SOAP_DEV_TYPE_UNPROGRAMMED,   "NOT_PROG",     {{ SW_UNKNOWN,      0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 },  { SW_UNKNOWN,           0,  ACT_HIGH,   MAX_DEG,    MIN_DEG,    100 }}, MOTOR_NONE,     0, ADD_CPROBE,  PCB_ALL_HH },
};
#define INIT_NUM_DEVICES (sizeof(devInit) / sizeof(devInitStruct))

////////////////////////////////////////////////////////////////////////////////

const devFamilyStruct devFamilies[] =
{
	{ SOAP_DEV_TYPE_MK1                 , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_MK2                 , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_MK450               , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_EMO_25              , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_EMO_50              , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_VOLCANO_15          , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_VOLCANO_25          , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_VOLCANO_50          , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_KRAKATOA_15         , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_LASER_2             , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_SKIP_COLON          , DEVICE_FAMILY_UNKNOWN },
	{ SOAP_DEV_TYPE_LASER_5             , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_LASER_6             , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_LASER_40            , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_LASER_80            , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_LASER_150           , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_QUIET_STORM         , DEVICE_FAMILY_ACCESSORY },
	{ SOAP_DEV_TYPE_MICROSCOPE          , DEVICE_FAMILY_ACCESSORY },
	{ SOAP_DEV_TYPE_HOTBED_100          , DEVICE_FAMILY_HOTBED },
	{ SOAP_DEV_TYPE_HOTBED_120          , DEVICE_FAMILY_HOTBED },
	{ SOAP_DEV_TYPE_HOTBED_200          , DEVICE_FAMILY_HOTBED },
	{ SOAP_DEV_TYPE_HOTCOLDBED          , DEVICE_FAMILY_HOTBED },
	{ SOAP_DEV_TYPE_UVLIGHT_RAY         , DEVICE_FAMILY_LASER },
	{ SOAP_DEV_TYPE_DRILL_MILL          , DEVICE_FAMILY_SPINDLE },
	{ SOAP_DEV_TYPE_DIGITRAM            , DEVICE_FAMILY_ACCESSORY },
	{ SOAP_DEV_TYPE_PICKPLACE           , DEVICE_FAMILY_PICKNPLACE },
	{ SOAP_DEV_TYPE_SDS                 , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_SDS_5               , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_SDS_10              , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_SDS_30              , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_SDS_60              , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_SDS_150             , DEVICE_FAMILY_UNHEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_INKJET1             , DEVICE_FAMILY_INKJET },
	{ SOAP_DEV_TYPE_INKJET2             , DEVICE_FAMILY_INKJET },
	{ SOAP_DEV_TYPE_BITSCOPE            , DEVICE_FAMILY_PICKNPLACE },
	{ SOAP_DEV_TYPE_FIL_DISP            , DEVICE_FAMILY_ACCESSORY },
	{ SOAP_DEV_TYPE_CO2_LENS            , DEVICE_FAMILY_ACCESSORY },
	{ SOAP_DEV_TYPE_TAMBORA_15          , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_3PH_SPINDLE         , DEVICE_FAMILY_SPINDLE },
	{ SOAP_DEV_TYPE_GENERIC_HEAD        , DEVICE_FAMILY_HEATED_EXTRUDER },  // best match for now...may need a new family
	{ SOAP_DEV_TYPE_RH_SYRINGE          , DEVICE_FAMILY_HEATED_EXTRUDER },
	{ SOAP_DEV_TYPE_SONICATOR           , DEVICE_FAMILY_STONE },
	{ SOAP_DEV_TYPE_CAN_MOTOR           , DEVICE_FAMILY_MOTOR },
};
#define INIT_NUM_FAMILIES (sizeof(devFamilies) / sizeof(devFamilyStruct))

////////////////////////////////////////////////////////////////////////////////

const devPcbStrStruct devPcbs[] =
{
	{ SOAP_PCB_UNKNOWN,             "UNKNOWN" },
	{ SOAP_PCB_4982_160MJ_NOWIRE,   "4982_MJ" },
	{ SOAP_PCB_4982_160MJ_WIRE,     "4982_MJW" },
	{ SOAP_PCB_4982_160J_NOWIRE,    "4982_J" },
	{ SOAP_PCB_4988_160J_NOWIRE,    "4988_J" },
	{ SOAP_PCB_BTT,					"BigTree" },
	{ SOAP_PCB_405_4988,			"405_4988" },
	{ SOAP_PCB_405_ENC_PWM,			"405_ENC_PWM" },
	{ SOAP_PCB_405_ENC_DAC,			"405_ENC_DAC" },
	{ SOAP_PCB_UNPROGRAMMED,        "NOT_PROG" },

};
#define INIT_NUM_PCB (sizeof(devPcbs) / sizeof(devPcbStrStruct))

////////////////////////////////////////////////////////////////////////////////

const devRtdStrStruct devRtds[] =
{
	{ SOAP_RTD_UNKNOWN,         "UNKNOWN" },
	{ SOAP_RTD_NONE,            "None" },
	{ SOAP_RTD_1M,              "1M" },
	{ SOAP_RTD_1K,              "1K" },
	{ SOAP_RTD_10K,             "10K" },
	{ SOAP_RTD_50K,             "50K" },
	{ SOAP_RTD_4_TO_20MA,       "4-20ma" },
	{ SOAP_RTD_RAW_ADC,         "RawADC" },
	{ SOAP_RTD_100,             "100" },
	{ SOAP_RTD_UNPROGRAMMED,    "NOT_PROG" },
};

#define INIT_NUM_RTD (sizeof(devRtds) / sizeof(devRtdStrStruct))

////////////////////////////////////////////////////////////////////////////////

const devCodebaseStrStruct devCodebases[] =
{
	{ DEVICE_CODEBASE_UNKNOWN,          "Unknown" },
	{ DEVICE_CODEBASE_MEDUSA,           "Medusa4" },
	{ DEVICE_CODEBASE_PICKNPLACE,       "PickNPlace4" },
	{ DEVICE_CODEBASE_UNPROGRAMMED,     "CODEBASE_NOT_PROGRAMMED" },
};
#define INIT_NUM_DEVICE_CODEBASES (sizeof(devCodebases) / sizeof(devCodebaseStrStruct))

////////////////////////////////////////////////////////////////////////////////

const devTargetStrStruct devTargets[] =
{
	{ DEVICE_CODEBASE_MEDUSA,       DEVICE_TARGET_MEDUSA103,        "Heads_Beds_103" },
	{ DEVICE_CODEBASE_MEDUSA,       DEVICE_TARGET_MEDUSA405,        "Heads_Beds_405" },
	{ DEVICE_CODEBASE_PICKNPLACE,   DEVICE_TARGET_PICKNPLACE,       "PICKNPLACE" },
	{ DEVICE_CODEBASE_MEDUSA,       DEVICE_TARGET_SONICATOR407,     "Sonicator_407" },
	{ DEVICE_CODEBASE_MEDUSA,       DEVICE_TARGET_CAN_MOTOR_103,    "Can_Motor_103" },
};
#define INIT_NUM_DEVICE_CODEBASE_TARGETS (sizeof(devTargets) / sizeof(devTargetStrStruct))

const devTargetStrStruct devTargetsOLD[] =
{
	{ DEVICE_CODEBASE_MEDUSA,       DEVICE_TARGET_MEDUSA103,        "Medusa103" },
	{ DEVICE_CODEBASE_MEDUSA,       DEVICE_TARGET_MEDUSA405,        "Medusa405" },
	{ DEVICE_CODEBASE_PICKNPLACE,   DEVICE_TARGET_PICKNPLACE,       "PICKNPLACE" },
};
#define INIT_NUM_DEVICE_CODEBASE_TARGETS_OLD (sizeof(devTargetsOLD) / sizeof(devTargetStrStruct))
////////////////////////////////////////////////////////////////////////////////

devInitStruct *getDeviceInitPtr(soapDevType_t deviceType)
{
	int i=0;
	while (i < INIT_NUM_DEVICES)
	{
		if (devInit[i].devType == deviceType)
		{   // match!
			return((devInitStruct *)&devInit[i]);
		}
		i++;
	}
	return((devInitStruct *)getDeviceInitPtr(SOAP_DEV_TYPE_UNKNOWN));
}

////////////////////////////////////////////////////////////////////////////////

char *deviceTypeToString(soapDevType_t deviceType)
{
	int i=0;
	while (i < INIT_NUM_DEVICES)
	{
		if (devInit[i].devType == deviceType)
		{   // match!
			return((char *)devInit[i].devTypeStr);
		}
		i++;
	}
	return("DEVICE_NOT_FOUND");
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceTypeIsADiodeLaser(soapDevType_t devType)
{
	return ((devType == SOAP_DEV_TYPE_LASER_2) || (devType == SOAP_DEV_TYPE_LASER_5) || (devType == SOAP_DEV_TYPE_LASER_6));
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceTypeIsACO2Laser(soapDevType_t devType)
{
	return ((devType == SOAP_DEV_TYPE_LASER_40) || (devType == SOAP_DEV_TYPE_LASER_80) || (devType == SOAP_DEV_TYPE_LASER_150));
}

////////////////////////////////////////////////////////////////////////////////

devFam_t devType2DevFamily(soapDevType_t deviceType)
{
	int i=0;
	while (i < INIT_NUM_FAMILIES)
	{
		if (devFamilies[i].devType == deviceType)
		{   // match!
			return(devFamilies[i].devFamily);
		}
		i++;
	}
	return(DEVICE_FAMILY_UNKNOWN);
}

////////////////////////////////////////////////////////////////////////////////

char *deviceSubtypeToString(soapDevType_t deviceType, soapDevSubtype_t deviceSubtype)
{
	devFam_t devFamily = devType2DevFamily(deviceType);
	switch(devFamily)
	{
	case DEVICE_FAMILY_HEATED_EXTRUDER :
	case DEVICE_FAMILY_UNHEATED_EXTRUDER :
		break;
	case DEVICE_FAMILY_LASER :
		if (deviceTypeIsACO2Laser(deviceType))
		{
			switch (deviceSubtype)
			{
			case SOAP_DEV_SUBTYPE_C02_LASER_UART :              return("PS w/ UART");
			case SOAP_DEV_SUBTYPE_C02_LASER_NO_UART :           return("PS w/o UART");
			default: return("LOST");
			}
		}
		else if (deviceTypeIsADiodeLaser(deviceType))
		{
			switch(deviceSubtype)
			{
			case SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_PP :     return("AutoCool-PushPull");
			case SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_OD :     return("AutoCool-OpenDrain");
			case SOAP_DEV_SUBTYPE_DIODE_LASER_NO_COOLDOWN_PP :  return("FanOn-PushPull");
			case SOAP_DEV_SUBTYPE_DIODE_LASER_NO_COOLDOWN_OD :  return("FanOn-OpenDrain");
			default: return("LOST");
			}
		}
		break;
	case DEVICE_FAMILY_SPINDLE :
		//      if (deviceTypeIsA3phSpindle(deviceType))
		//      {
		//          return("");
		//      }
		//      else
		//      {
		//          switch(deviceSubtype)
		//          {
		//          case SOAP_DEV_SUBTYPE_MILL_1_TICK_PER_REV :             return("1X");
		//          case SOAP_DEV_SUBTYPE_MILL_2_TICK_PER_REV :             return("2X");
		//          default: return("LOST");
		//          }
		//      }
		break;
	case DEVICE_FAMILY_HOTBED :
	{
		switch (deviceType)
		{
		case SOAP_DEV_TYPE_HOTBED_100 :
		{
			switch (deviceSubtype)
			{
			case SOAP_DEV_SUBTYPE_0 :   return("use-LIMIT1");
			case SOAP_DEV_SUBTYPE_1 :   return("use-HTR_HSS");
			default: return("LOST");
			}
		}
		break;
		case SOAP_DEV_TYPE_HOTBED_120 :
		case SOAP_DEV_TYPE_HOTBED_200 :
		{
			switch (deviceSubtype)
			{
			case SOAP_DEV_SUBTYPE_0 :   return("use-HTR_HSS");
			case SOAP_DEV_SUBTYPE_1 :   return("use-LIMIT1");
			default: return("LOST");
			}
		}
		break;
		default:
			break;
		}
	}
	break;
//	case DEVICE_FAMILY_MOTOR :
//	{
//		switch (deviceSubtype)
//		{
//		case SOAP_DEV_SUBTYPE_MOTOR_1_8_DEGREE_PER_STEP :   return("1.8deg");
//		case SOAP_DEV_SUBTYPE_MOTOR_0_9_DEGREE_PER_STEP :   return("0.9deg");
//		default: return("LOST");
//		}
//		break;
//	}

	case DEVICE_FAMILY_ACCESSORY :
	case DEVICE_FAMILY_PICKNPLACE :
	case DEVICE_FAMILY_INKJET :
	default :
		break;
	}
	return("Standard");
}

////////////////////////////////////////////////////////////////////////////////

char *devicePcbToString(soapPcb_t devicePcb)
{
	int i=0;
	while (i < INIT_NUM_PCB)
	{
		if (devPcbs[i].pcbIndex == devicePcb)
		{   // match!
			return((char *)devPcbs[i].pcbStr);
		}
		i++;
	}
	return("Unknown");
}
////////////////////////////////////////////////////////////////////////////////

char *deviceRtdToString(soapRtd_t deviceRtd)
{
	int i=0;
	while (i < INIT_NUM_RTD)
	{
		if (devRtds[i].rtdIndex == deviceRtd)
		{   // match!
			return((char *)devRtds[i].rtdStr);
		}
		i++;
	}
	return("Unknown");
}

////////////////////////////////////////////////////////////////////////////////

char *softwareCodebaseToString(devCodebase_t softwareCodebase)
{
	int i=0;
	while (i < INIT_NUM_DEVICE_CODEBASES)
	{
		if (devCodebases[i].codebaseIndex == softwareCodebase)
		{   // match!
			return((char *)devCodebases[i].codebaseStr);
		}
		i++;
	}
	return("CODEBASE_NOT_FOUND");
}

////////////////////////////////////////////////////////////////////////////////

//NUKE SOMEDAY -- needed for 4.003x and earlier LEGACY REMOVE
char *softwareCompileTargetToStringOLD(devCodebase_t softwareCodebase, devTarget_t deviceCompileTarget)
{
	int i=0;
	while (i < INIT_NUM_DEVICE_CODEBASE_TARGETS_OLD)
	{
		if ((devTargetsOLD[i].codebaseIndex == softwareCodebase) && (devTargets[i].compileTargetIndex == deviceCompileTarget))
		{   // match!
			return((char *)devTargetsOLD[i].targetStr);
		}
		i++;
	}
	return("TARGET_NOT_FOUND");
}

////////////////////////////////////////////////////////////////////////////////

char *softwareCompileTargetToString(devCodebase_t softwareCodebase, devTarget_t deviceCompileTarget)
{
	int i=0;
	while (i < INIT_NUM_DEVICE_CODEBASE_TARGETS)
	{
		if ((devTargets[i].codebaseIndex == softwareCodebase) && (devTargets[i].compileTargetIndex == deviceCompileTarget))
		{   // match!
			return((char *)devTargets[i].targetStr);
		}
		i++;
	}
	return("TARGET_NOT_FOUND");
}

////////////////////////////////////////////////////////////////////////////////

void initSendSwitchFlagsAndMask(outboxStruct *outboxPtr, int switchNum)
{
	switchFlags_t flags = {0};
	switchFlags_t mask = {0};
	devFam_t deviceFamily = outboxPtr->deviceFamily;
	inboxStruct *inboxPtr = getInboxPointer(outboxPtr->device);

	mask.b.usePulseTrain        = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.onOnlyWhenExtruding  = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.blockColdExtrude     = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.polarity             = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.heatingDevice        = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.coolingDevice        = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.allowCtrlByOn        = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.allowCtrlByDC        = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.allowCtrlByPwm       = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.allowCtrlByTemp      = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.checkMaxTemp         = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.checkMinTemp         = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.binaryTempCtrl       = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.useWatchdogTimer     = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below
	mask.b.useCooldownTimer     = 1;    // corresponding flag is off by default with flags.u32 = 0 - change default below

	flags.b.polarity            = (outboxPtr->hardInitPtr->sw[switchNum].activeHigh) ? 1 : 0;       // from init table
	flags.b.binaryTempCtrl      = (outboxPtr->hardInitPtr->sw[switchNum].slamSwOnOff) ? 1 : 0;      // from init table
	flags.b.useWatchdogTimer    = ((switchNum == HH_HTR_SWITCH) && (deviceFamily == DEVICE_FAMILY_LASER));
	flags.b.useCooldownTimer    = ((switchNum == HH_HTR_SWITCH) && ((deviceTypeIsADiodeLaser(inboxPtr->deviceType) &&
									((inboxPtr->deviceSubtype == SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_PP) ||
									(inboxPtr->deviceSubtype == SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_PP))) ||
									deviceTypeIsACO2Laser(inboxPtr->deviceType)));
	flags.b.allowCtrlByOn       = ((switchNum == HH_HTR_SWITCH) && ((deviceTypeIsADiodeLaser(inboxPtr->deviceType) &&
									((inboxPtr->deviceSubtype == SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_PP) ||
									(inboxPtr->deviceSubtype == SOAP_DEV_SUBTYPE_DIODE_LASER_COOLDOWN_PP))) ||
									deviceTypeIsACO2Laser(inboxPtr->deviceType)));


	switch (outboxPtr->hardInitPtr->sw[switchNum].type)
	{
	case SW_ON_OFF :
		flags.b.allowCtrlByOn = 1;
		break;
	case SW_DC :
		flags.b.allowCtrlByDC = 1;
		break;
	case SW_PWM :
		flags.b.allowCtrlByPwm = 1;
		break;
	case SW_TEMP_HEAT :
		flags.b.allowCtrlByTemp = 1;
		flags.b.heatingDevice = 1;
		flags.b.usePulseTrain = 1;      // all heads to date have used the pulse train for temp control
		break;
	case SW_TEMP_COOL :
		flags.b.allowCtrlByTemp = 1;
		flags.b.coolingDevice = 1;
		flags.b.usePulseTrain = 1;      // all heads to date have used the pulse train for temp control
		break;
	case SW_TEMP_HEAT_COOL :
		flags.b.allowCtrlByTemp = 1;
		flags.b.heatingDevice = 1;
		flags.b.coolingDevice = 1;
		flags.b.usePulseTrain = 1;      // all heads to date have used the pulse train for temp control
		break;
	case SW_TIM_PWM :
		flags.b.allowCtrlByOn = 1;
		break;
	case SW_ALL_OPTIONS :
		flags.b.allowCtrlByOn = 1;
		flags.b.allowCtrlByDC = 1;
		flags.b.allowCtrlByPwm = 1;
		flags.b.allowCtrlByTemp = 1;
		flags.b.heatingDevice = 1;
		//flags.b.coolingDevice = 1;    //-- wait to enable
	default :
		break;
	}

	if (flags.b.allowCtrlByDC)
	{
		flags.b.usePulseTrain = 1;      // all heads to date have used the pulse train for dc control
	}

	if (flags.b.heatingDevice || ((switchNum == HH_HTR_SWITCH) && (deviceFamily == DEVICE_FAMILY_LASER)))
	{
		flags.b.checkMaxTemp = 1;
	}

	if (flags.b.coolingDevice)
	{
		flags.b.checkMinTemp = 1;
	}

	if (outboxPtr->validConfiguration)
	{
		canSendOutboxInitValue2x32(outboxPtr, getDevInitAreaSwitchNum(switchNum), DEV_INIT_INDEX_SWx_FLAGS, flags.u32, mask.u32);
	}
}

////////////////////////////////////////////////////////////////////////////////

void initSendDefaultSwitchParams(outboxStruct *outboxPtr, int switchNum)
{
	//powerFactor;              // Karl's K-factor/Power Factor for temperature control
	//deltaTempBiasCoeff;       // used to effect a larger duty cycle when delta temp is large
	//maxOvershoot;             // amount above targetTemp before switch is off completely.
	//denominator;              // denominator for PF/XXX part of Karl's equation
	//fanDcToHtrDcScaleFactor;  // the calculated Heater DC from the temp is goosed up by this scaleFactor * the FAN's DC 1.7 format

	devSwitchStruct *swPtr = &outboxPtr->hardInitPtr->sw[switchNum];
	devInitArea_t devInitAreaSwX = getDevInitAreaSwitchNum(switchNum);

	int16_t     powerFactor             = 65;
	int16_t     deltaTempBiasCoeff      = 5;
	uint16_t    maxOvershoot            = (outboxPtr->hardInitPtr->devType == SOAP_DEV_TYPE_KRAKATOA_15) ? 0 : 5;
	int16_t     denominator             = (outboxPtr->hardInitPtr->devType == SOAP_DEV_TYPE_KRAKATOA_15) ? 100 : 300;
	uint16_t    fanDcToHtrDcScaleFactor = (switchNum = HH_HTR_SWITCH) ? (50 / 100) * (1 << 7) : 0;  // 50% in a 1.7 format if heater;

	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_MIN_TEMP,      swPtr->minTemp << TEMP_FRAC_BITS);
	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_MAX_TEMP,      swPtr->maxTemp << TEMP_FRAC_BITS);
	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_MAX_DC,        swPtr->maxDutyCycle);

	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_KF_PF,         powerFactor);
	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_KF_BIAS,       deltaTempBiasCoeff);
	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_KF_OVERSHOOT,  maxOvershoot << TEMP_FRAC_BITS);
	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_KF_DENOM,      denominator);
	canSendOutboxInitValue1x16(outboxPtr, devInitAreaSwX, DEV_INIT_INDEX_SWx_KF_FAN_DC,     fanDcToHtrDcScaleFactor);
}

////////////////////////////////////////////////////////////////////////////////

void initDeviceFromDeviceTypeInitTable(outboxStruct *outboxPtr)
{
	devInitStruct *initPtr = outboxPtr->hardInitPtr;    // shorter name, local reference

	if (initPtr == NULL) return;

	// initialize basic device values
;
#ifdef USE_HYDRA_IO
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_SYSTEM, SYSTEM_HYDRA);
#elif defined(USE_HYREL_IO)
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_SYSTEM, SYSTEM_HYREL);
#elif defined(MEGASONIC)
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_SYSTEM, SYSTEM_MEGASONIC);
#endif

	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_PWM_LIMIT, initPtr->maxPwmFreq);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_INKJET_NOZZLE, 0);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_COOLDOWN, DEFAULT_LASER_COOLDOWN_TIME);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_VALID_COMBO, outboxPtr->validConfiguration);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_ADD_ON_MODULE, initPtr->allowableAddon);

	// initialize motor parameters
	uint16_t Jo, accel, maxRate, microsteps, periodMs;
	switch (initPtr->motor)
	{
	//                  init velo   delta velo      max velo            num microsteps
	//                  =========   ============    ================    ===============
	case MOTOR_2K:      Jo = 100;   accel = 100;    maxRate = 2000;     microsteps = 16;    periodMs = 100;     break;
	case MOTOR_10K:     Jo = 100;   accel = 100;    maxRate = 10000;    microsteps = 16;    periodMs = 100;     break;
	case MOTOR_20K:     Jo = 200;   accel = 200;    maxRate = 20000;    microsteps = 16;    periodMs = 100;     break;
	case MOTOR_SPDL:    Jo = 0;     accel = 0;      maxRate = 60000;    microsteps = 0;     periodMs = 100;     break;
	case MOTOR_ZMOVE:   Jo = 1;     accel = 3;      maxRate = 127;      microsteps = 0;     periodMs = 100;     break;
	//microsteps overloaded to be ticks per step pulse for MOTOR_
	case MOTOR_AXIS:    Jo = 10;    accel = 30;     maxRate = 65535;    microsteps = 0; 	periodMs = 1;     	break;
	default:            Jo = 0;     accel = 0;      maxRate = 0;        microsteps = 0;     periodMs = 100;     break;
	}

	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_JO, Jo);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_ACCEL, accel);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_MAX_RATE, maxRate);
	canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_PERIOD_MS, periodMs);

	if (deviceHasAClosedLoopStepper(outboxPtr->device))
	{
	// stored on chip, no need to init
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CONTROL_METHOD, OPEN_LOOP);
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_FULL_STEPS_PER_REV, 200);		// HARDCODED FOR NOW FOR a 1.8deg stepper
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_REV, CAN_MOTOR_TICKS_PER_REV / 4);
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_MAX_MILLIAMPS, 1600);					// HARDCODED FOR NOW (will set 50% power)
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT1_POLARITY, ACTIVE_DISABLED);
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT2_POLARITY, ACTIVE_DISABLED);
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KP, 30);
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KI, 10);
//		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KD, 250);
	}
	else
	{
		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP, microsteps);
		canSendOutboxInitValue1x16(outboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_BOOST, IN_FLOAT);
	}

	// initialize switch 0
	initSendSwitchFlagsAndMask(outboxPtr, HH_SW0_SWITCH);
	initSendDefaultSwitchParams(outboxPtr, HH_SW0_SWITCH);

	// initialize switch 1
	initSendSwitchFlagsAndMask(outboxPtr, HH_SW1_SWITCH);
	initSendDefaultSwitchParams(outboxPtr, HH_SW1_SWITCH);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
