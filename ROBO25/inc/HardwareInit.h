#ifndef HardwareInit_HEADER // prevent double dipping
#define HardwareInit_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    HardwareInit.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains HardwareInit specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  HardwareInit specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

//power on defaults.   change with
#ifdef GB_FAST_UART
#define SYSTEM_BAUD_RATE 115200 //38400 // 115200 // 76800 // 57600 //38400 //115200//19200 //
#else
#define SYSTEM_BAUD_RATE 38400 // 115200 // 76800 // 57600 //38400 //115200//19200 //
#endif
#define LIGHTBURN_BAUD_RATE 38400 //250000


////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in HardwareInit that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in HardwareInit
////////////////////////////////////////////////////////////////////////////////

extern void InitGPIO(void);
extern void InitUSART3(unsigned int);
extern void InitUART4(unsigned int);
extern void InitUSART6(unsigned int);
extern void InitAllUARTS(unsigned int);
extern void CAN_Config(CAN_TypeDef*);
extern void InitMotorTimer(MotorStructure *, boolean);
extern void InitTimer1(void);
extern void InitTimer2usec(void);
extern void InitTim3RpmInput(void);
extern void ConfigureTimer4PwmOutputsFor0_10V(void);
extern void InitTimer8(void);
extern void SetCO2LaserPwmFrequency(int);
extern void InitEncoderTimer5(void);
extern void enableLatheMode(void);
extern void disableLatheMode(void);
extern void InitTimer6(void);
extern void InitTimer7(void);
extern void adcInit(ADC_TypeDef *);
extern int getLaserPwmFreq(void);
extern int getLaserPwmRange(void);
extern void initializeSpindleStruct(void);
extern void initializeInkjetStruct(void);
extern void initializeLaserStruct(void);
extern void InitLaser(void);
extern void DeInitLaser(void);
extern void setupIWDG(float);
extern void setupDBGMCU(void);
extern void setFlasherVarPtr(flasherVarSel_t);
extern void InitGlobals(void);
#ifdef USE_AB_ENCODER
extern void timerInitEncoderAB(boolean);
#endif //USE_AB_ENCODER
extern void pinInitAll(void);
//extern void Init_ADC(void);
//extern void Start_ADC(void);

#endif // #ifndef HardwareInit_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
