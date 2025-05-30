#ifndef headSetup_HEADER // prevent double dipping
#define headSetup_HEADER
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

typedef enum {  // only add new families at end of the list (before DEVICE_FAMILY_UNPROGRAMMED)
	DEVICE_FAMILY_UNKNOWN           = 0,
	DEVICE_FAMILY_HEATED_EXTRUDER   = 1,
	DEVICE_FAMILY_UNHEATED_EXTRUDER = 2,
	DEVICE_FAMILY_LASER             = 3,
	DEVICE_FAMILY_SPINDLE           = 4,
	DEVICE_FAMILY_HOTBED            = 5,
	DEVICE_FAMILY_ACCESSORY         = 6,
	DEVICE_FAMILY_PICKNPLACE        = 7,
	DEVICE_FAMILY_INKJET            = 8,
	DEVICE_FAMILY_STONE             = 9,
	DEVICE_FAMILY_MOTOR             = 10,
// QQQQ ADD NEW FAMILY(S) HERE
	DEVICE_FAMILY_UNPROGRAMMED = 0xff
} devFam_t;

typedef enum {
	SW_UNKNOWN = 0,
	SW_ON_OFF,
	SW_DC,
	SW_PWM,
	SW_TEMP_HEAT,           // only a heater
	SW_TEMP_COOL,           // only a chiller
	SW_TEMP_HEAT_COOL,      // could be either based on ambient temp (RTD2)

	SW_HEAT_KF,         // only a heater
	SW_COOL_KF,         // only a chiller
	SW_HEATCOOL_KF,     // could be either based on ambient temp (RTD2)

	SW_HEAT_SLAM,           // only a heater
	SW_COOL_SLAM,           // only a chiller
	SW_HEATCOOL_SLAM,       // could be either based on ambient temp (RTD2)

	SW_TIM_PWM,             // will be driven from timer
	SW_ALL_OPTIONS,             // able to do all functions
	SW_NOT_USED,
} devSwitchType_t;

typedef enum {
	ADC_NONE = 0,
	ADC_TEMP,
	ADC_CURR,
	ADC_VOLT,
} devADC_t;

typedef enum {
	MOTOR_NONE = 0,
	MOTOR_2K,
	MOTOR_10K,
	MOTOR_20K,
	MOTOR_SPDL,
	MOTOR_ZMOVE,
	MOTOR_AXIS,
} devMotor_t;

typedef enum {
	PCB_NONE = 0,
	PCB_J,          // only boards with correct HSS, either motor driver
	PCB_MJW,        // includes all PCB_J PLUS 4982+MJ_w_wire   [excludes SOAP_PCB_4982_160MJ_NOWIRE]
	PCB_MJ,         // includes all PCB_MJW PLUS 4982+MJ_wo_wire; 4982 only
	PCB_ALL_HH,  	// same as PCB_MJ,
	PCB_405,
	PCB_ENC,
} devLegalPCB_t;

typedef struct {
	devSwitchType_t type;
	boolean         slamSwOnOff;
	boolean         activeHigh;
	int16_t         minTemp;
	int16_t         maxTemp;
	uint16_t        maxDutyCycle;
} devSwitchStruct;

typedef struct {
	soapDevType_t   devType;
	char            devTypeStr[24];
	devSwitchStruct sw[2];
	devMotor_t      motor;
	uint16_t        maxPwmFreq;
	devAddOn_t      allowableAddon;
	devLegalPCB_t   allowablePCB;
} devInitStruct;


typedef struct {
	soapDevType_t       devType;
	devFam_t            devFamily;
} devFamilyStruct;

typedef struct {
	soapDevType_t       devType;
	soapDevSubtype_t    devSubtype;
	char                devSubtypeStr[12];
} devSubtypeStrStruct;

typedef struct {
	soapPcb_t           pcbIndex;
	char                pcbStr[24];
} devPcbStrStruct;

typedef struct {
	soapRtd_t           rtdIndex;
	char                rtdStr[24];
} devRtdStrStruct;

typedef struct {
	devCodebase_t       codebaseIndex;
	char                codebaseStr[24];
} devCodebaseStrStruct;

typedef struct {
	devCodebase_t       codebaseIndex;
	devTarget_t         compileTargetIndex;
	char                targetStr[24];
} devTargetStrStruct;

////////////////////////////////////////////////////////////////////////////////

typedef struct outboxStruct outboxStruct;   // forward declaration of struct

extern devInitStruct *getDeviceInitPtr(soapDevType_t);
extern char *deviceTypeToString(soapDevType_t);
extern char *deviceSubtypeToString(soapDevType_t, soapDevSubtype_t);
extern char *devicePcbToString(soapPcb_t);
extern char *deviceRtdToString(soapRtd_t);
extern char *softwareCodebaseToString(devCodebase_t);
extern char *softwareCompileTargetToStringOLD(devCodebase_t, devTarget_t); //NUKE SOMEDAY LEGACY
extern char *softwareCompileTargetToString(devCodebase_t, devTarget_t);
extern boolean deviceTypeIsADiodeLaser(soapDevType_t);
extern boolean deviceTypeIsACO2Laser(soapDevType_t);
extern devFam_t devType2DevFamily(soapDevType_t);
extern void initDeviceFromDeviceTypeInitTable(outboxStruct *);

////////////////////////////////////////////////////////////////////////////////

#endif // #ifndef headSetup_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
