#ifndef Motordriver_HEADER // prevent double dipping
#define Motordriver_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// File:    Motordriver.h
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Contains Motordriver specific defines, global references, and method prototypes
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  Motordriver specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

#define AxisIsMoving(axisPtr)   (axisPtr->PULSES_TO_GO)
#define motionQ_empty()         (motionQ.validEntries == 0)
#define motionQ_notEmpty()      (motionQ.validEntries != 0)
#define motionQ_full()          (motionQ.validEntries == SIZE_OF_MOTION_QUEUE)
#define motionQ_notFull()       (motionQ.validEntries < SIZE_OF_MOTION_QUEUE)
#define motionQ_numEntries()    (motionQ.validEntries)


#define MOTIONQ_FIRST_ENTRY_AGE_TIME_MS   50 // milliseconds - time to allow commands to age in the motionQ before starting motion (allow lookahead)
#define MOTIONQ_WAIT_FOR_MOTOR_EN_TIME_MS 500
#define MOTIONQ_ENTIRES_TO_START_PROCESSING 4 // num entries in queue to allow commands to age in the motionQ before starting motion (allow lookahead)

typedef struct  {
	float CentripetalAccelRadius;
	float DraftModeTolerance;
} MachineLimitsStruct;

typedef struct {
	uint32_t TimerPSC;
	uint32_t TimerARR;
	float TimerFreq;
	TIM_TypeDef* TimerBase;
} DominantStruct;

typedef struct {
	uint32_t TimerPSC;
	uint32_t TimerARR;
	float TimerFreq;
	TIM_TypeDef* TimerBase;
	int pulsesToGo;
	MotorStructure *M;
	float jogValueInUnits;
} jogZStruct;

typedef struct {
	int             Hz; // which loop is inthe jogging routine called from.
	boolean         enabled;
	boolean         firstMove;
	boolean         reloadTimerControls;
	int             dirChangePauseTimeMs;
	int             ignoreInputsCountdownTimer;
	direction_t     direction;
	int             axisIndex;
	float           minRate;    //no ramp
	float           maxRate;    // max cruise rate
	float           acceleration;
	MotorStructure  *M;
} joggingStruct;

#define JOG_OFF_MODE_GUI				0
#define JOG_OFF_MODE_FILAMENT			1
#define JOG_OFF_MODE_FILAMENT_DEVICE	16

typedef struct {
	boolean         enabled;
	boolean         reverseDefaultDirection;
	int             axisIndex;
	float           incrAmount;
	int				offMode;
} ABjoggingStruct;

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in Motordriver that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////

extern boolean _IncrementalMove;
extern boolean _IncrementalEMove;

extern float _TargetFeedRateInMmPerSec;
extern float _CurrentRequestedFeedrateInMmPerSec;
extern float _ActualVectorRateInMmPerSec;

extern int FirstAxisMotor;
extern int LastAxisMotor;
extern MotorStructure *FirstAxisMotorPtr;
extern MotorStructure *LastAxisMotorPtr;
extern MotorStructure *DominantAxisPtr;
extern MotorStructure *_movingAxesPtrs[];
extern int _numMovingAxes;

extern MachineLimitsStruct _MachineLimits;

extern float _LastExtrusionRate;

extern DominantStruct _Dominant;
extern jogZStruct _jogZ;
extern joggingStruct _jogging;
extern ABjoggingStruct _ABjogging;

////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in Motor
////////////////////////////////////////////////////////////////////////////////

extern float ClampToMachineLimitsInPulsesPerSec(float);
extern float ClampToAxisLimitsInUPS(MotorStructure *, float);
extern void UpdateAxisLimitAndRates(MotorStructure *);
extern void UpdateAxisTravelLimits(MotorStructure *);
extern void EnableAllMotionMotors(void);
extern void DisableAllMotionMotors(void);
extern void cleanupAfterAbortFinishes(void);
extern void motionQ_pauseMotionWhenSafe(void);
extern void motionQ_abortMotionWhenSafe(flushAction_t);
extern boolean NoAxisIsMoving(void);
extern boolean AnyPotentialMotion(void);
extern void ClearAllMotorG92Offset(void);
extern void ClearMotorFixtureOffsets(MotorStructure *);
extern void ClearAllMotorFixtureOffsets(void);
extern void ClearMotorHeadOffsets(MotorStructure *);
extern void ClearAllMotorHeadOffsets(void);
extern float SumOfAllMotorOffsets(MotorStructure *);
extern void ResetAllMotionOffsetIndices(void);
extern void DisableMotionTimers(void);
extern void loadDominantTimer(void);
extern void loadMotorTimer(MotorStructure *M);
extern void StartMove(void);
extern void setupLatheModeToRun(MotorStructure *, float, direction_t);
extern passFail_t ExecuteSingleAxisMove(MotorStructure *, float, float);
extern passFail_t ExecuteHomingMove(float);
extern boolean UpdateNextAccelerationFactor(MotorStructure *);
extern void CalculateTimerControls(MotorStructure *);
extern void SpeedControl(void);
extern void latheSpeedControl(void);
extern void ProcessMotion( MotorStructure *);
extern void InitializeMotorParameters(void);
extern void motionQ_init(void);
extern int motionQ_getCountdownDelayToExecuteMove(void);
extern void motionQ_setCountdownDelayToExecuteMove(int);
extern void motionQ_decrCountdownDelayToExecuteMove(void);
extern boolean motionQ_hasSpace(int);
extern boolean motionQ_almostFull(int);
extern passFail_t motionQ_addCommand(float, boolean);
extern void motionQ_primeComplete(void);
extern void motionQ_unprimeComplete(void);
extern void motionQ_reset(void);
extern void motionQ_update(void);
extern void motionQ_executeMove(void);
extern void motionQ_deleteCurrentMoveAndExecuteNextMove(void);
extern void JogMotor(MotorStructure *,float);

#define MICRONS_TO_MM                               0.001f
#define DEFAULT_MIN_ACCELERATION_CONSTANT           0.1f
#define JOG_MAXIMUM_VALUE_IN_MICRONS                10000.0f    // 10000 microns == 10mm
#define JOG_DEFAULT_VALUE_IN_MICRONS                20.0f       //
#define JOG_MINIMUM_CHANGE_OF_DIRECTION_DELAY_MS    100         // 100 ms
#define JOG_MAXIMUM_CHANGE_OF_DIRECTION_DELAY_MS    5000        // 5 seconds
#define JOG_DEFAULT_CHANGE_OF_DIRECTION_DELAY_MS    500         // 0.5 seconds

extern void joggingDeInit(void);
extern void joggingComplete(void);
extern void joggingAbort(char *);
extern void joggingCalculateTimerControls(void);
extern void joggingUpdatePulses(motorIndex_t, float);
extern void CalibrateZHomeSensors(void);

#endif // #ifndef Motordriver_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE
