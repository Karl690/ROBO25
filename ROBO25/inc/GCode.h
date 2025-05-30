#ifndef GCode_HEADER // prevent double dipping
#define GCode_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    GCode.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains GCode specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  GCode specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in GCode that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////

// from GCode.c:

#define DELAY_2_HOURS_OF_MILLISECONDS   (2 * 60 * 60 * 1000)
#define DELAY_24_HOURS_OF_MILLISECONDS  (24 * 60 * 60 * 1000)
#define DELAY_48_HOURS_OF_MILLISECONDS  (48 * 60 * 60 * 1000)

extern float ConvertValueToMM;
extern float ConvertValueFromMM;
extern float ConvertValueToDeg;
extern float ConvertValueFromDeg;
extern float CurrentToolDiameter;
extern uint16_t DesiredCo2LaserPower;
extern arcStruct _arc;

// from MCode.c
extern byte PersistantUltimusControlHeadAddress;
extern byte PersistantHotheadAddress;
extern byte PersistantHotbedAddress;
////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in GCode
////////////////////////////////////////////////////////////////////////////////

// from GCode.c
extern void ReportInvalidGcodeHArg(void);
extern int getArgFraction(float arg);
extern float getMotorArg(MotorStructure *);
extern float getMotorArgInMM(MotorStructure *);
extern float getMotorArgInDeg(MotorStructure *);
extern float getMotorArgInNativeUnits(MotorStructure *);
extern void setMotorArg(MotorStructure *, float);
extern void setMotorArgInMM(MotorStructure *, float);
extern void setMotorArgInDeg(MotorStructure *, float);
extern void setMotorArgInNativeUnits(MotorStructure *, float);
extern void setAllMotorArg(float);
extern float convertArgFromMM(float);
extern float convertArgToMM(float);
extern float convertArgToDeg(float);
extern float convertArgToNativeUnits(MotorStructure *, float);
extern void setupDrillCycleParameters(float);
extern boolean motorArgPresent(MotorStructure *M);
extern boolean motorArgPositive(MotorStructure *M);
extern boolean motorArgNegative(MotorStructure *M);
extern boolean motorArgGreaterThanZero(MotorStructure *M);
extern boolean axisOnCanbus(MotorStructure *M);
extern void KillCannedCycle(void);
extern void G_Code_G0(void);
extern void G_Code_G1(void);
extern void initArcStruct(arcStruct *);
extern void ExecuteArcPointMove(void);
extern void ExecuteG2G3PointMove(void);
extern void G_Code_G2(void);
extern void G_Code_G3(void);
extern void ProcessG2_1G3_1Command(int);
extern void G_Code_G4(void);
extern void G_Code_G12(void);
extern void G_Code_G13(void);
extern void G_Code_G16(void);
extern void G_Code_G17(void);
extern void G_Code_G18(void);
extern void G_Code_G19(void);
extern void G_Code_G20(void);
extern void G_Code_G21(void);
extern void G_Code_G28(void);
extern void G_Code_G928(void);
extern void G_Code_G29(void);
extern void startProbingMove(void);
extern void G_Code_G38(void);
extern void G_Code_G53(void);
extern void G_Code_G54(void);
extern void G_Code_G55(void);
extern void G_Code_G56(void);
extern void G_Code_G57(void);
extern void G_Code_G58(void);
extern void G_Code_G59(void);
extern void G_Code_G70(void);
extern void G_Code_G71(void);
extern void G_Code_G73(void);
extern void G_Code_G74(void);
extern void G_Code_G76(void);
extern void G_Code_G77(void);
extern void G_Code_G80(void);
extern void G_Code_G81(void);
extern void G_Code_G82(void);
extern void G_Code_G83(void);
extern void G_Code_G84(void);
extern void G_Code_G85(void);
extern void G_Code_G90(void);
extern void G_Code_G91(void);
extern void G_Code_G92(void);
extern void G_Code_G93(void);
extern void G_Code_G101(void);
extern void G_Code_G202(void);//circular hole bore milling Clockwise
extern void G_Code_G203(void);//circular hole bore milling Counter Clockwise
extern void G_Code_G212(void);  //clockwise circular pocket mill
extern void G_Code_G213(void);  //Cclockwise circular pocket mill
extern void G_Code_G214(void);  //clockwise Rectangular pocket mill
extern void G_Code_G215(void);  //clockwise Rectangular pocket mill

extern void G_Code_G702(boolean);
extern void ProcessG702CylinderPrint(void);

extern vectorStruct *assignVector(vectorStruct *, float, float, float, boolean);

// from MCode.c


typedef struct {
	MotorStructure *rotaryMotorAxisPtr;
	MotorStructure *axisParallelToRotaryAxisPtr;
	vectorStruct liftUV;
} M235Struct;

extern M235Struct _M235;
extern float _M106_fanDutyCycleRange;

//DEPRECATED - extern void SendZOffsets(void);
extern void SendFakeMcodeExecutionNotice(int, float, float, float);

extern void M_Code_M0(void);    // program stop - program pause sequence
extern void M_Code_M1(void);  // does nothing (program pause)
extern void M_Code_M2(void);  // does nothing (program end)
extern void M_Code_M3(void);  // spindle on CW
extern void M_Code_M4(void);  // spindle on CCW
extern void M_Code_M5(void);  // spindle OFF
extern void M_Code_M6(void);  // set tool offsets (uses T, D, I, O, X, Y, Z, A, B, C)
extern void M_Code_M7(void);  // turn coolant/ heater1 on (uses T)
extern void M_Code_M8(void);  // coolant / heater2 on
extern void M_Code_M9(void);  // all coolant and heaters OFF
extern void M_Code_M17(void);  // releases the holding torque on stepping motors
extern void M_Code_M18(void);  // releases the holding torque on stepping motors
extern void M_Code_M30(void);  // start or end of program
extern void M_Code_M41(void);  // spindle in Low Range
extern void M_Code_M42(void);  // spindle in High Range
extern void M_Code_M44(void);  // spindle Motor Coolant ON
extern void M_Code_M45(void);  // spindle  Motor Coolant OFF
extern void M_Code_M73(void); // passthru of remaining printing time (uses P, R)
extern void M_Code_M82(void);  // enable absolute E moves
extern void M_Code_M83(void);  // enable incremental (relative) E moves
extern void M_Code_M84(void);  // releases the holding torque on stepping motors
extern void M_Code_M91(void);  // set Max travel distance (uses X, Y, Z, A, B, C)
extern void M_Code_M92(void);  // set axis steps per unit,  (pulses/mm;  pulses/in(if inch mode); pulses/deg(rotary) (uses X, Y, Z, A, B, C)
extern void M_Code_M93(void);  // sets home sensor polarity (uses X, Y, Z, A, B, C)
extern void M_Code_M94(void);  // sets the default motor direction (uses X, Y, Z, A, B, C)
extern void M_Code_M95(void);  // sets stall sensor polarity (uses X, Y, Z, A, B, C)
extern void M_Code_M96(void);  // sets the enable bit polarity (uses X, Y, Z, A, B, C)
extern void M_Code_M97(void);  // sets the step bit polarity (uses X, Y, Z, A, B, C)
extern void M_Code_M98(void);  //sets limit1 sensor polarity (uses X, Y, Z, A, B, C)
extern void M_Code_M99(void);  //sets limit2 sensor polarity (uses X, Y, Z, A, B, C)
extern void setLaserWatchdogForPrime(outboxStruct *);
extern void M_Code_M101(void);  // enable HH motor and optionally set feed rate (uses T, S, E, P)
extern void M_Code_M102(void);  // enable HH motor and optionally set feed rate in reverse (uses T, S, E)
extern void M_Code_M103(void);  // disable HH motor (uses T)
extern void M_Code_M104(void);  // set HH temperature (uses T, S)
extern void M_Code_M105(void);  // read HH temperature (uses T)
extern void M_Code_M106(void);  // turn fan on and (optionally) set pwm (uses T, I, S, W, J, P)
extern void M_Code_M107(void);  // turn fan off (uses T)
extern void M_Code_M108(void);  // set extrusion rate (uses T, S)
extern void M_Code_M109(void);  // wait for hothead to reach temp (uses T, S)
extern void M_Code_M114(void);  // report xyz location immediately
extern void M_Code_M115(void);  // report firmware revision level
extern void M_Code_M140(void);  // set hotbed temperature (uses T, S)
extern void M_Code_M141(void);  // set chamber temperature (uses S)
extern void M_Code_M190(void);  // wait for hotbed to reach temp (uses S)
extern void M_Code_M191(void);  // wait for chamber to reach temp (uses S)
extern void M_Code_M203(void);  // Sets the Maximum G0/Rapid velocity UNIT/MIN (and Homing/Accel ramp) (uses X, Y, Z, A, B, C, H, P)
extern void M_Code_M204(void);  // Sets the no ramp speed (UNITS/MIN) (no acceleration needed below this) (uses X, Y, Z, A, B, C)
extern void M_Code_M205(void);  // Sets the homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
extern void M_Code_M206(void);  // Sets the homing routine hysteresis location (inch/mm/deg) (uses X, Y, Z, A, B, C)
extern void M_Code_M208(void);  // Sets the acceleration constant (uses X, Y, Z, A, B, C)
extern void M_Code_M209(void);  // Sets the minimum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
extern void M_Code_M210(void);  // Sets the maximum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
extern void M_Code_M211(void);  // Sets the machine minimum pulse rate limit (uses S)
extern void M_Code_M212(void);  // Sets the machine maximum pulse rate limit (uses S)
extern void M_Code_M213(void);  // sets the per motor/axis installation status (uses X,Y,Z,A,B,C)
extern void M_Code_M214(void);  // sets the per motor/axis type (linear/rotary) (uses X,Y,Z,A,B,C)
extern void M_Code_M215(void);  // Sets the homing routine start location (inch/mm/deg) (uses X, Y, Z, A, B, C)
extern void M_Code_M216(void);  // Sets the homing routine end location (inch/mm/deg) (uses X, Y, Z, A, B, C)
extern void M_Code_M217(void);  // sets the max deceleration rate (for abort) (mm/sec/sec)  (uses X, Y, Z, A, B, C)
extern void M_Code_M218(void);  // set the fixture offsets  (uses O, X, Y, Z, A, B, C)
extern void M_Code_M219(void);  // sets the serial baud rate (uses B)
extern void M_Code_M220(void);  // sets the per motor/axis send motorStep pulse as a CANbus command (uses T,X,Y,Z,A,B,C)
extern void M_Code_M221(void);  // set extrude calculation factors (uses T,S,Z,W,P)
extern void M_Code_M222(void);  // set the centripetal accelleration radius (uses S)
extern void M_Code_M223(void);  // Sets the re-homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
extern void M_Code_M224(void);  //Enable extruder motor during fast move - just a shell
extern void M_Code_M225(void);  //Disable extruder motor during fast move - just a shell
extern void M_Code_M226(void);  // Issue a Pause command from G-Code - just a shell
extern void M_Code_M227(void);  // control jogging using the AB encoder on the panel interface (uses XYZABCR>
extern void M_Code_M228(void);  // disable auto extruder unprime and prime
extern void M_Code_M229(void);  // extrusion control (uses E, P, S)
extern void updateCompositeOverridePct(void);
extern void M_Code_M230(void);  // set global flow rate override percentage (uses S)
extern void M_Code_M231(void);  // set motion feedrate override percentage (uses S)
extern void M_Code_M232(void);  // set motion override percentage (uses S)
extern void M_Code_M233(void);  // set homing pop-off distance (uses X, Y, Z, A, B, C)
extern void M_Code_M234(void);  // set motor position (absolute) (inch/mm/deg) (uses X, Y, Z, A, B, C)
extern void M_Code_M235(void);  // set rotary axis plane and offset (uses S, P, A, Y, Z)
extern void M_Code_M236(void);  // write device type and revision to flash (option bytes) (uses T, S, I, P)
extern void M_Code_M237(void);  // set cold extrusion prevention parameters (uses T, C, U, L, R)
extern void M_Code_M238(void);  // sets the per motor/axis execute E value (uses T,X,Y,Z,A,B,C)

extern void M_Code_M240(void);  // turn device switch off (uses T, I)
extern void M_Code_M241(void);  // turn device switch on (uses T, I)
extern void M_Code_M242(void);  // control device switch by dutyCycle (uses T, I, S)
extern void M_Code_M243(void);  // control device switch by pwm (uses T, I, S, P)
extern void M_Code_M244(void);  // control device switch by temperature (uses T, I, S)
extern void M_Code_M245(void);  // set switch flag(s) (uses T, I, P, E, H, C)
extern void sendMotorMotionValues(byte device, MotorStructure *M);
extern void M_Code_M253(void);  // turn on lathe motor for continuous CW motion (uses S)
extern void M_Code_M254(void);  // turn on lathe motor for continuous CCW motion (uses S)
extern void M_Code_M255(void);  // turn off lathe motor (uses S)
extern void M_Code_M260(void);  // control display attached to a head (uses S,P,X,Y,I,J,R)
extern void setupHssPwm(HssPwmStruct *hss);
extern void M_Code_M261(void);  //osseo control
extern void M_Code_M262(void); //osseo control
extern void M_Code_M263(void); //osseo control
extern void M_Code_M600(void);  // disable all HSS outputs
extern void M_Code_M601(void);  // enable HSS out1
extern void M_Code_M602(void);  // enable HSS out2
extern void M_Code_M603(void);  // enable HSS out3
extern void M_Code_M604(void);  // enable HSS out4
extern void M_Code_M605(void);  // enable HSS out5
extern void M_Code_M606(void);  // enable HSS out6
extern void M_Code_M607(void);  // CO2 Laser SSR
extern void M_Code_M608(void);  // CO2 Laser PUMP

extern void M_Code_M610(void);  // enable HSS out10
extern void M_Code_M611(void);  // enable HSS out11
extern void M_Code_M612(void);  // enable HSS out12
extern void M_Code_M613(void);
extern void M_Code_M614(void);
extern void M_Code_M615(void);
extern void M_Code_M616(void);
extern void M_Code_M617(void);

extern void M_Code_M619(void);  // sets the function and output pwm of the selected HSS (uses F, I, S, P, J, H)
extern void M_Code_M620(void);  // Laser global control (uses T, E, F, C, P)
extern void M_Code_M621(void);  // Laser vector mode control (uses P)
extern void M_Code_M622(void);  // Laser raster mode control (uses O, S, D, P, I)
extern void M_Code_M623(void);  // Laser pulse (one-shot) mode control (uses P, D)
extern void M_Code_M624(void);  // setup raster image (uses B, X, Z, I, J, U, V, P, C, O)
extern void M_Code_M625(void);  // inkjet vector mode control (uses S, J)
extern void M_Code_M626(void);  // build color index table (uses C, U, A, D)
extern void M_Code_M627(void);  // set job_kill/abort auto move location (uses AXZABC IJKUVW)
extern void M_Code_M628(void);  // arm/disarm digital trigger (uses P, E, R)
extern void M_Code_M629(void);  // open log file
extern void M_Code_M630(void);  // canbus touch probe control (uses T, S, D, P)
extern void M_Code_M631(void);  // PinkNPlace data (uses T, H, P, A, B, C, D)
extern void M_Code_M632(void);  /// PinkNPlace control (uses T, S, H, P, V, F, D,)
extern void M_Code_M633(void);  // enable/disable CAN2 (uses E)
extern void M_Code_M660(void);  //set tool diameter and length (uses T, D, Z)
extern void M_Code_M670(void);  // sets the Y-Axis Bar Light PWM value (uses S)
extern void M_Code_M671(void);  // sets the DangerHot PWM value (uses S)
extern void M_Code_M672(void);  // sets the select for controlled the Y-Axis light (uses S)
extern void M_Code_M673(void);  // calls M670
extern void M_Code_M674(void);  // enable/disable turbo mode (convert non-print G1 to G0) (uses S)
extern void M_Code_M675(void);  // sets the response light hss
extern void M_Code_M676(void);  // sets the chamber fan pwm
extern void M_Code_M677(void);  // sets the control panel buzzer pwm
extern void M_Code_M678(void);  // set the laser cross-hair pwm
extern void M_Code_M679(void);  // set the exhaust fan pwm
extern void M_Code_M680(void);  // set new values for the z home sensor offsets (uses S,W,X,Y,Z)
extern void M_Code_M681(void);  // send the selected z axis offsets to the host
extern void M_Code_M682(void);  // start Z axis sensor calibration run
extern void M_Code_M683(void);  // set the headroom for the normal serial rx buffer (uses S)
extern void M_Code_M685(void);  // sets the air assist pwm
extern void M_Code_M686(void);  // returns the machine info string for machine key creation
extern void M_Code_M687(void);  // unlock system with machine specific password
extern void M_Code_M688(void);  //RESERVED - DO NOT USE
#ifdef ALLOW_GCODE_SCRIPTS
extern void M_Code_M690(void);  // add/delete scripts
#endif

extern void M_Code_M698(void);  // humiture control (uses T, V)
extern void M_Code_M699(void);  // hx711 control (uses T, V, S, O, Z)
extern void M_Code_M701(void);  // set auto status update rate and page selection (uses T, S, P)
extern void M_Code_M702(void);  // select Tx usage in MCODES (uses S)
extern void M_Code_M703(void);  // add an alias to device (uses T, S)
extern void M_Code_M704(void);  // remove an alias from device (uses T, S)
extern void M_Code_M705(void);  // reset device (uses T)
extern void M_Code_M706(void);  // sync device (uses T)
extern void M_Code_M707(void);  // stop device (uses T)
extern void M_Code_M708(void);  // pause device (uses T)
extern void M_Code_M709(void);  // resume device (from pause or stop) (uses T)
extern void M_Code_M710(void);  // disable(0), enable(1) RTD1 (uses T, S)
extern void M_Code_M711(void);  // disable(0), enable(1) RTD2 (uses T, S)
extern void M_Code_M712(void);  // disable(0), enable(1) RTD3 (uses T, S)
extern void M_Code_M713(void);  // tool change (uses T)
extern void M_Code_M714(void);  // update outgoing devicePosition remapping table
extern void M_Code_M715(void);  // set LED display control and data selection (uses T, P, S, E)
extern void M_Code_M716(void);  // turns off logging of aux comm in repetrel
extern void M_Code_M717(void);  // turns on logging of aux comm in repetrel
extern void M_Code_M718(void);  // used by repetrel to synchronize M719 data logging
extern void M_Code_M719(void);  // set reporting update rate and page selection for host traffic (uses T, S, P)
extern void M_Code_M720(void);  // direct MAIN extrusion control (uses T, S, E, P)
extern void M_Code_M721(void);  // direct UNPRIME extrusion control (uses T, S, E, P)
extern void M_Code_M722(void);  // direct PRIME extrusion control (uses T, S, E, P)
extern void M_Code_M723(void);  // direct MANUAL extrusion control, Unmodified (uses T, S, E, P)
extern void M_Code_M724(void);  // direct DWELL (no stepping) extrusion control (uses T, S, E, P)
extern void M_Code_M725(void);  // set the Karl Factors for controlling the heater switch (uses T, S, E, P)
extern void M_Code_M726(void);  // set the Karl Factors for controlling the fan switch (uses T, P, E, S)
extern void M_Code_M727(void);  // set LED override values and mask (uses T, S, P)
extern void M_Code_M728(void);  // set motor current boost control (uses T, S)
extern void M_Code_M729(void);  // set motor microsteps control (uses T, S)
extern void M_Code_M730(void);  // set not to exceed temp for motor (uses T, S)
extern void M_Code_M731(void);  // set not to exceed temp for heater (uses T, S)
extern void M_Code_M732(void);  // set maximum step rate for motor (microsteps/sec) (uses T, S)
extern void M_Code_M733(void);  // set maximum allowable RTD temperature delta (uses T, S)
extern void M_Code_M734(void);  // set HH error reporting rate for redundant error codes (uses T, S)
extern void M_Code_M735(void);  // fill the incoming page data buffer with S (uses S)
extern void M_Code_M736(void);  // fill the outgoing page data buffer with S (uses S)
extern void M_Code_M737(void);  // erase flash page in selected (physical) device (uses T, P, I)
extern void M_Code_M738(void);  // transfer data page from (physical) device to incoming buffer (uses T, P, I)
extern void M_Code_M739(void);  // transfer data page from incoming to outgoing buffer
extern void M_Code_M740(void);  // transfer data page from outgoing buffer to (physical) device (uses T, P, I, S)
extern void M_Code_M741(void);  // transfer selected table data from the device to the host (uses T, S, I)
extern void M_Code_M742(void);  // transfer select table data in ASCII text from host to device (uses T, S, P, I, comment)
extern void M_Code_M743(void);  // transfer non table/page related  device info from inbox/device to host (uses T, S, I)
extern void M_Code_M744(void);  // transfer alias list from device to inbox (uses T)
extern void M_Code_M745(void);  // change polarity definition of direction pin (uses T, S)
extern void M_Code_M746(void);  // start the bootloader for the selected physical device (uses T)
extern void M_Code_M747(void);  // erase page for bootloader
extern void M_Code_M748(void);  // process next line of intel hex format bootloader data (uses P, comment)
extern void M_Code_M749(void);  // exit the device bootloader
extern void M_Code_M750(void);  // unlock flash for erase/write access for the selected physical device (uses T)
extern void M_Code_M751(void);  // lock flash to prevent erase/write access for the selected physical device (uses T)
extern void M_Code_M752(void);  // write hardware type to flash (option bytes) using device bootloader (uses S, P)
extern void M_Code_M753(void);  // write hardware revision to flash (option bytes) using device bootloader (uses  S, P)
extern void M_Code_M754(void);  // write hardware key to flash (option bytes) using device bootloader (uses S, P)
extern void M_Code_M755(void);  // set extruder heater pwm (uses T, S)
extern void M_Code_M756(void);  // set layer height (mm) (uses S)
extern void M_Code_M757(void);  // set layer/path weight (mm) (uses S)
extern void M_Code_M758(void);  // set extrusion step to volume conversion (steps per 10 nL) (uses T, S)
extern void M_Code_M759(void);  // enable temperature calibration (uses T, S)
extern void M_Code_M760(void);  // disable temperature calibration
extern void M_Code_M761(void);  // transfer system info in ASCII text from main board to host (uses S, P)
extern void M_Code_M762(void);  // transfer system info in ASCII text from host to main board (uses S, P, comment)
extern void M_Code_M763(void);  // clear error on selected device(s) (uses T)
extern void M_Code_M764(void);  // set max duty cycle for device
extern void M_Code_M765(void);  // read X, Y, Z and dial indicator
extern void M_Code_M766(void);  // start the system bootloader process
extern void M_Code_M767(void);  // prepare system for download (erase pages, etc) (uses E, P)
extern void M_Code_M768(void);  // process next line of intel hex format of system bootloader data (uses P, comment)
extern void M_Code_M769(void);  // exit the device bootloader
extern void M_Code_M770(void);  // leave system bootloader and jump to application main()
extern void M_Code_M771(void);  // load laser image data controls (scale, offset, max)
extern void M_Code_M772(void);  // reset metrics for new job
extern void M_Code_M773(void);  // send motion metrics to host
extern void M_Code_M774(void);  // send queue metrics to host
extern void M_Code_M775(void);  // send current status/queue values to host
extern void M_Code_M776(void); // send cmd/motionQ usage histograms to host
extern void M_Code_M777(void);  // send and/or erase the crash log (uses S, E)
extern void M_Code_M778(void);  // enable slice time measurement (uses I, S)
extern void M_Code_M779(void);  // dump slice time measurements
extern void M_Code_M780(void);  // enable/disable auto XYZABC position reporting
extern void M_Code_M781(void);  // write hardware type to flash (option bytes) (uses T, D, O, P)
extern void M_Code_M782(void);  // enable/disable "print air" feature (uses S)
extern void M_Code_M783(void);  // set PersistantUltimusControlHeadAddress
extern void M_Code_M784(void);  // report system info (version numbers of system and all heads)
extern void M_Code_M785(void);  // Set motor parameters (uses T,U,A,R,B,P,C,O,S) (V1)
extern void M_Code_M786(void);  // set closed loop stepper PID control values (uses T, P, I, D)
extern void M_Code_M787(void);  // calibrate can based closed-loop motor (uses T, P, C)
extern void M_Code_M788(void);  // reset can based closed-loop axis motor (uses T, F, D, P)
extern void M_Code_M789(void);  // send sideband step pulses to a canAxisMotor (uses T, S)
extern void M_Code_M790(void);  // notify layer change
extern void M_Code_M791(void);  // notify take picture event
extern void M_Code_M795(void);  // sets jogvalueValueInUnits (uses S)
extern void M_Code_M796(void);  // DEBUG: kill return OK's bakc to repetier for testing flow control
extern void M_Code_M797(void);  // enable/disable debug reporting strings ">GB:" (uses S)
extern void M_Code_M798(void);  // dump strings to host (debug MCODE) -- warning, must reset after using this MCODE (uses T)
extern void M_Code_M799(void);  // get PLL and Clock status for processor
extern void M_Code_M800(void);  // sonicator control
extern void M_Code_M960(void);  // rectangular pocket mill (uses X, Y, E)
extern void M_Code_M868(void); // Bump Z axis Down ( positive ) 10 pulses
extern void M_Code_M869(void); //bump Z axis up (negative) 10 pulses
extern void M_Code_M7734(void);

#endif // #ifndef GCode_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
