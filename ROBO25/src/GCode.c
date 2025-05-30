////////////////////////////////////////////////////////////////////////////////
//
// File:    GCode.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: The module contains all of the methods to interpret and evaluate all
//          of the GCODE related commands.
//  
//          Any unit coversions are done at this level so the rest of the
//          operation of the machine can work in consistent units.  These include
//
//              Dimensions to millimeters
//              Feeds and Speed to millimeters per second
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "mailbox.h"
#include "MotorDriver.h"
#include "Serial.h" 
#include "Hydra_can.h"
#include "GCode.h"
#include "hardwareInit.h"

arcStruct _arc;

///////////////////////////////////////////////////////////

int getArgFraction(float arg)
{
	return((int)roundf(((arg - (float)(int)arg) * 10.0f)));
}

///////////////////////////////////////////////////////////

void ReportUnknownFractionalGcode(void)
{
	sprintf(_errorStr, "UNKNOWN GCODE %d.%d", (int)ARG_G, getArgFraction(ARG_G));
	sendError(_errorStr);
}

///////////////////////////////////////////////////////////

void ReportGcodeError(char *s)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "GCODE %d -- %s", (int) ARG_G, s);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportInvalidGcodeArg(char *arg, float val)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "GCODE %d -- Invalid %s arg (%4.4f)", (int)ARG_G, arg, val);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportInvalidGcodeArgInt(char *arg, uint32_t val)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "GCODE %d -- Invalid %s arg (%d/0x%08x)", (int)ARG_G, arg, (int)val, (int)val);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportMissingGcodeArg(char *arg)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "GCODE %d -- Missing %s arg (required)", (int)ARG_G, arg);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportMissingGcodeAArg(void)  { ReportMissingGcodeArg("A"); }
void ReportMissingGcodeBArg(void)  { ReportMissingGcodeArg("B"); }
void ReportMissingGcodeCArg(void)  { ReportMissingGcodeArg("C"); }
void ReportMissingGcodeDArg(void)  { ReportMissingGcodeArg("D"); }
void ReportMissingGcodeEArg(void)  { ReportMissingGcodeArg("E"); }
void ReportMissingGcodeFArg(void)  { ReportMissingGcodeArg("F"); }
void ReportMissingGcodeGArg(void)  { ReportMissingGcodeArg("G"); }
void ReportMissingGcodeHArg(void)  { ReportMissingGcodeArg("H"); }
void ReportMissingGcodeIArg(void)  { ReportMissingGcodeArg("I"); }
void ReportMissingGcodeJArg(void)  { ReportMissingGcodeArg("J"); }
void ReportMissingGcodeKArg(void)  { ReportMissingGcodeArg("K"); }
void ReportMissingGcodeLArg(void)  { ReportMissingGcodeArg("L"); }
void ReportMissingGcodeMArg(void)  { ReportMissingGcodeArg("M"); }
void ReportMissingGcodeNArg(void)  { ReportMissingGcodeArg("N"); }
void ReportMissingGcodeOArg(void)  { ReportMissingGcodeArg("O"); }
void ReportMissingGcodePArg(void)  { ReportMissingGcodeArg("P"); }
void ReportMissingGcodeQArg(void)  { ReportMissingGcodeArg("Q"); }
void ReportMissingGcodeRArg(void)  { ReportMissingGcodeArg("R"); }
void ReportMissingGcodeSArg(void)  { ReportMissingGcodeArg("S"); }
void ReportMissingGcodeTArg(void)  { ReportMissingGcodeArg("T"); }
void ReportMissingGcodeUArg(void)  { ReportMissingGcodeArg("U"); }
void ReportMissingGcodeVArg(void)  { ReportMissingGcodeArg("V"); }
void ReportMissingGcodeWArg(void)  { ReportMissingGcodeArg("W"); }
void ReportMissingGcodeXArg(void)  { ReportMissingGcodeArg("X"); }
void ReportMissingGcodeYArg(void)  { ReportMissingGcodeArg("Y"); }
void ReportMissingGcodeZArg(void)  { ReportMissingGcodeArg("Z"); }

void ReportMissingGcodeSEArg(void)  { ReportMissingGcodeArg("SE"); }
void ReportMissingGcodeSPArg(void)  { ReportMissingGcodeArg("SP"); }
void ReportMissingGcodeFPArg(void)  { ReportMissingGcodeArg("FP"); }
void ReportMissingGcodeFSArg(void)  { ReportMissingGcodeArg("FS"); }
void ReportMissingGcodePIArg(void)  { ReportMissingGcodeArg("PI"); }
void ReportMissingGcodePLArg(void)  { ReportMissingGcodeArg("PL"); }
void ReportMissingGcodeEPSArg(void)  { ReportMissingGcodeArg("EPS"); }

////////////////////////////////////////////////////////////////////////////////

void ReportInvalidGcodeAArg(void)  { ReportInvalidGcodeArg("A", ARG_A); }
void ReportInvalidGcodeBArg(void)  { ReportInvalidGcodeArg("B", ARG_B); }
void ReportInvalidGcodeCArg(void)  { ReportInvalidGcodeArg("C", ARG_C); }
void ReportInvalidGcodeDArg(void)  { ReportInvalidGcodeArg("D", ARG_D); }
void ReportInvalidGcodeEArg(void)  { ReportInvalidGcodeArg("E", ARG_E); }
void ReportInvalidGcodeFArg(void)  { ReportInvalidGcodeArg("F", ARG_F); }
void ReportInvalidGcodeGArg(void)  { ReportInvalidGcodeArg("G", ARG_G); }
void ReportInvalidGcodeHArg(void)  { ReportInvalidGcodeArg("H", ARG_H); }
void ReportInvalidGcodeIArg(void)  { ReportInvalidGcodeArg("I", ARG_I); }
void ReportInvalidGcodeJArg(void)  { ReportInvalidGcodeArg("J", ARG_J); }
void ReportInvalidGcodeKArg(void)  { ReportInvalidGcodeArg("K", ARG_K); }
void ReportInvalidGcodeLArg(void)  { ReportInvalidGcodeArg("L", ARG_L); }
void ReportInvalidGcodeMArg(void)  { ReportInvalidGcodeArg("M", ARG_M); }
void ReportInvalidGcodeOArg(void)  { ReportInvalidGcodeArg("O", ARG_O); }
void ReportInvalidGcodeNArg(void)  { ReportInvalidGcodeArg("N", ARG_N); }
void ReportInvalidGcodePArg(void)  { ReportInvalidGcodeArg("P", ARG_P); }
void ReportInvalidGcodeQArg(void)  { ReportInvalidGcodeArg("Q", ARG_Q); }
void ReportInvalidGcodeRArg(void)  { ReportInvalidGcodeArg("R", ARG_R); }
void ReportInvalidGcodeSArg(void)  { ReportInvalidGcodeArg("S", ARG_I); }
void ReportInvalidGcodeTArg(void)  { ReportInvalidGcodeArg("T", ARG_T); }
void ReportInvalidGcodeUArg(void)  { ReportInvalidGcodeArg("U", ARG_U); }
void ReportInvalidGcodeVArg(void)  { ReportInvalidGcodeArg("V", ARG_V); }
void ReportInvalidGcodeWArg(void)  { ReportInvalidGcodeArg("W", ARG_W); }
void ReportInvalidGcodeXArg(void)  { ReportInvalidGcodeArg("X", ARG_X); }
void ReportInvalidGcodeYArg(void)  { ReportInvalidGcodeArg("Y", ARG_Y); }
void ReportInvalidGcodeZArg(void)  { ReportInvalidGcodeArg("Z", ARG_Z); }

void ReportInvalidGcodeAArgInt(void)  { ReportInvalidGcodeArgInt("A", ARG_A); }
void ReportInvalidGcodeBArgInt(void)  { ReportInvalidGcodeArgInt("B", ARG_B); }
void ReportInvalidGcodeCArgInt(void)  { ReportInvalidGcodeArgInt("C", ARG_C); }
void ReportInvalidGcodeDArgInt(void)  { ReportInvalidGcodeArgInt("D", ARG_D); }
void ReportInvalidGcodeEArgInt(void)  { ReportInvalidGcodeArgInt("E", ARG_E); }
void ReportInvalidGcodeFArgInt(void)  { ReportInvalidGcodeArgInt("F", ARG_F); }
void ReportInvalidGcodeGArgInt(void)  { ReportInvalidGcodeArgInt("G", ARG_G); }
void ReportInvalidGcodeHArgInt(void)  { ReportInvalidGcodeArgInt("H", ARG_H); }
void ReportInvalidGcodeIArgInt(void)  { ReportInvalidGcodeArgInt("I", ARG_I); }
void ReportInvalidGcodeJArgInt(void)  { ReportInvalidGcodeArgInt("J", ARG_J); }
void ReportInvalidGcodeKArgInt(void)  { ReportInvalidGcodeArgInt("K", ARG_K); }
void ReportInvalidGcodeLArgInt(void)  { ReportInvalidGcodeArgInt("L", ARG_L); }
void ReportInvalidGcodeMArgInt(void)  { ReportInvalidGcodeArgInt("M", ARG_M); }
void ReportInvalidGcodeNArgInt(void)  { ReportInvalidGcodeArgInt("N", ARG_N); }
void ReportInvalidGcodeOArgInt(void)  { ReportInvalidGcodeArgInt("O", ARG_O); }
void ReportInvalidGcodePArgInt(void)  { ReportInvalidGcodeArgInt("P", ARG_P); }
void ReportInvalidGcodeQArgInt(void)  { ReportInvalidGcodeArgInt("Q", ARG_Q); }
void ReportInvalidGcodeRArgInt(void)  { ReportInvalidGcodeArgInt("R", ARG_R); }
void ReportInvalidGcodeSArgInt(void)  { ReportInvalidGcodeArgInt("S", ARG_S); }
void ReportInvalidGcodeTArgInt(void)  { ReportInvalidGcodeArgInt("T", ARG_T); }
void ReportInvalidGcodeUArgInt(void)  { ReportInvalidGcodeArgInt("U", ARG_U); }
void ReportInvalidGcodeVArgInt(void)  { ReportInvalidGcodeArgInt("V", ARG_V); }
void ReportInvalidGcodeWArgInt(void)  { ReportInvalidGcodeArgInt("W", ARG_W); }
void ReportInvalidGcodeXArgInt(void)  { ReportInvalidGcodeArgInt("X", ARG_X); }
void ReportInvalidGcodeYArgInt(void)  { ReportInvalidGcodeArgInt("Y", ARG_Y); }
void ReportInvalidGcodeZArgInt(void)  { ReportInvalidGcodeArgInt("Z", ARG_Z); }

////////////////////////////////////////////////////////////////////////////////
//  
//  Local #defines
//  
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//  
//  Public Global Definition (expose in GCode.h)
//  
////////////////////////////////////////////////////////////////////////////////

float ConvertValueToMM = 1.0f; // conversion of Args to mm (set to 25.4 convert inches to mm or 1.0 to stay mm)
float ConvertValueFromMM = 1.0f; // conversion of mm to Args (set to 1/25.4 convert mm to inches or 1.0 to stay mm)
float ConvertValueToDeg = 1.0f; // conversion of Args to degrees (set to RADIANS_TO_DEGREES convert rad to deg or 1.0 to stay deg)
float ConvertValueFromDeg = 1.0f; // conversion of degrees to Args (set to DEGREES_TO_RADIANS convert deg to rad or 1.0 to stay deg)

float CurrentToolDiameter=0;    // GB XXX gregkarl 911 -- this seems wrong ... should be using the ToolOffsets[].Diameter

////////////////////////////////////////////////////////////////////////////////
//  
//  Local Global Definitions (do not expose in GCode.h)
//  
////////////////////////////////////////////////////////////////////////////////
float SpiralRadiusIncrement=0;
int SpiralPocket=0;
float SpiralSegments=0;
int SpiralPasses=0;
int _frogToePasses=0;
float Radius=1;
float QuarterRadius=0;
float WorkingDiameter=0;//used for G12 where we want a pocket mill
float LeadInArcCenterX;
float LeadOutArcCenterX;
float ArcCenterX=0;
float ArcCenterY=0;
float LeadInArcCenterX=0;
float LeadInArcCenterY=0;
float theta=0.0f;
float StartAngleInDegrees=0.0f;
float EndAngleInDegrees=0.0f;
float SweepAngle=0.0f; //granularity of G2 arc move
float CW_CCW_Modifier=1; //1 means clockwise -1 means CCWise
float CurrentAngle=0.0f;//current vector
float AngleIncrement=0.0f;
float G2FinalX=0.0f;
float G2FinalY=0.0f;
float G2FinalZ=0.0f;
float G2DeltaX=0.0f;
float G2DeltaY=0.0f;
float G2DeltaZ=0.0f;
float EndAngleDeltaX=0.0f;
float EndAngleDeltaY=0.0f;
float WorkAngleInRadians=0.0f;
float arcArgS=INVALID_ARG_VALUE;

float XRadvalue=0.0f;
float yRADvalue=0.0f;
int ArcSubcounter=0;
int NumberOfArcSegments;

//the following variables are used for death Spiral bore

int spiralBoreNumberOfArcSegments=0;
float SpiralBoreZIncrement=0.0f;
float spiralBoreZEndPosition=0.0f;
float spiralBoreZStartPosition=0.0f;
float spiralBoreDeltaZ=0.0f;
float spiralBorePitch=0.0f;
int spiralBoreNumberOfSpirals=0;//used to repeat the last bore cycle with z increment
int spiralBoreArcSegmentsPerRevolution=300;

GMCommandStructure *G2G3Ptr;
float LastG2X=0.0f;
float LastG2Y=0.0f;
float LastG2Z=0.0f;

boolean _save_IncrementalEMove;
boolean _save_IncrementalMove;

////////////////////////////////////////////////////////////////////////////////
//  
//  Forward declarions;
//  
////////////////////////////////////////////////////////////////////////////////

void ProcessG2G3Command(int);
void frogToeCircle(void);
void setupArcMove(float, float, float, float);
void StartFinalReturnToCenterArc(void);
void StartMainFinishCircle(void);
void startMainRoughCircle(void);
void StartNextArcForPocketMill(void);
void SetFixtureOffset(int);
void sendZBackToSpiralStartPosition(void);
void StartMainFinishCircleSpiral(void);

////////////////////////////////////////////////////////////////////////////////

float getMotorArg(MotorStructure *M)
{
	return(*(float *)(((byte *)ExecutionPtr)+M->GCodeArgOffset));
}

////////////////////////////////////////////////////////////////////////////////

float getMotorArgInMM(MotorStructure *M)
{
	return((*(float *)(((byte *)ExecutionPtr)+M->GCodeArgOffset)) * ConvertValueToMM );
}

////////////////////////////////////////////////////////////////////////////////

float getMotorArgInDeg(MotorStructure *M)
{
	return((*(float *)(((byte *)ExecutionPtr)+M->GCodeArgOffset)) * ConvertValueToDeg );
}

////////////////////////////////////////////////////////////////////////////////

float getMotorArgInNativeUnits(MotorStructure *M)
{   // convert the raw number from the input data stream (referenced through the ExecutionPtr) into
	// the correct native units for internal processing which is in mm or degrees.  If the user
	// has selected to operate in either inches or radians, those will be converted to mm or deg
	// (depending on whether it's a linear axis or a rotary one)
	if (M->AxisType == LINEAR)
		return(getMotorArgInMM(M));
	else    // ROTARY
		return(getMotorArgInDeg(M));
}

////////////////////////////////////////////////////////////////////////////////

void setMotorArg(MotorStructure *M, float Value)
{
	(*(float *)(((byte *)ExecutionPtr)+M->GCodeArgOffset)) = Value;
}

////////////////////////////////////////////////////////////////////////////////

void setMotorArgInMM(MotorStructure *M, float ValueInMM)
{
	(*(float *)(((byte *)ExecutionPtr)+M->GCodeArgOffset)) = (ValueInMM * ConvertValueFromMM);
}
////////////////////////////////////////////////////////////////////////////////

void setMotorArgInDeg(MotorStructure *M, float ValueInMM)
{
	(*(float *)(((byte *)ExecutionPtr)+M->GCodeArgOffset)) = (ValueInMM * ConvertValueFromDeg);
}

////////////////////////////////////////////////////////////////////////////////

void setMotorArgInNativeUnits(MotorStructure *M, float Value)
{
	if (M->AxisType == LINEAR)
		setMotorArgInMM(M, Value);
	else {
		setMotorArgInDeg(M, Value);
	}
}

////////////////////////////////////////////////////////////////////////////////

void setAllMotorArg(float Value)
{
	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, setting it's motor Arg
		setMotorArg(M, Value);
	}

}

////////////////////////////////////////////////////////////////////////////////

float convertArgFromMM(float Value)
{   // restore to input units
	return(Value * ConvertValueFromMM);
}

////////////////////////////////////////////////////////////////////////////////

float convertArgToMM(float Value)
{
	return(Value * ConvertValueToMM);
}

////////////////////////////////////////////////////////////////////////////////

float convertArgToDeg(float Value)
{
	return(Value * ConvertValueToDeg);
}

////////////////////////////////////////////////////////////////////////////////

float convertArgToNativeUnits(MotorStructure *M, float Value)
{
	if (M->AxisType == LINEAR)
		return(convertArgToMM(Value));
	else {
		return(convertArgToDeg(Value));
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean motorArgPresent(MotorStructure *M)
{
	return(getMotorArg(M) != INVALID_ARG_VALUE);
}

////////////////////////////////////////////////////////////////////////////////

boolean motorArgPositive(MotorStructure *M)
{
	return(getMotorArg(M) >= 0.0);
}

////////////////////////////////////////////////////////////////////////////////

boolean motorArgNegative(MotorStructure *M)
{   // be careful of INVALID_ARG_VALUE
	return(getMotorArg(M) < 0.0);
}

////////////////////////////////////////////////////////////////////////////////

boolean motorArgNegativeOrZero(MotorStructure *M)
{   // be careful of INVALID_ARG_VALUE
	return(getMotorArg(M) <= 0.0);
}

////////////////////////////////////////////////////////////////////////////////

boolean motorArgGreaterThanZero(MotorStructure *M)
{
	return(getMotorArg(M) > 0.0);
}

///////////////////////////////////////////////////////////////////////////////

boolean axisOnCanbus(MotorStructure *M)
{
	return(M->canMotor);
}

///////////////////////////////////////////////////////////////////////////////

float getDeltaAxisValueInInputUnits(MotorStructure *M)
{
	float delta;

	if (_IncrementalMove)
	{
		delta = getMotorArg(&Motors[M->Axis]);
	}
	else
	{
		if (M->AxisType == LINEAR)
			delta = getMotorArg(M) - (M->Q_LastRequestedPositionInUnits * ConvertValueFromMM);
		else
			delta = getMotorArg(M) - (M->Q_LastRequestedPositionInUnits * ConvertValueFromDeg);
	}
	return(fmaxf(0.0f, delta));
}

///////////////////////////////////////////////////////////////////////////////

float getDeltaAxisValueInMm(MotorStructure *M)
{
	float delta;

	if (_IncrementalMove)
	{
		delta = getMotorArgInMM(&Motors[M->Axis]);
	}
	else
	{
		delta = getMotorArgInMM(M) - M->Q_LastRequestedPositionInUnits;
	}
	return(fmaxf(0.0f, delta));
}

///////////////////////////////////////////////////////////////////////////////

float getDeltaAxisValueInNativeUnits(MotorStructure *M)
{
	float delta;

	if (_IncrementalMove)
	{
		delta = getMotorArgInNativeUnits(&Motors[M->Axis]);
	}
	else
	{
		delta = getMotorArgInNativeUnits(M) - M->Q_LastRequestedPositionInUnits;
	}
	return(fmaxf(0.0f, delta));
}

////////////////////////////////////////////////////////////////////////////////

float getDeltaEValueInInputUnits(void)
{
	float deltaE;

	if (_IncrementalEMove)
	{
		deltaE = ARG_E;
	}
	else
	{
		deltaE = ARG_E - convertArgFromMM(E_Q_LastRequestedPositionInUnits);
	}
	return(fmaxf(0.0f, deltaE));
}

////////////////////////////////////////////////////////////////////////////////
////////////////////// GEOMETRY FUNCTIONS //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void assignPoint(pointStruct *p, float X, float Y, float Z)
{
	p->X = X;
	p->Y = Y;
	p->Z = Z;
}

////////////////////////////////////////////////////////////////////////////////

void copyPoint(pointStruct *p1, pointStruct *p2)
{
	p1->X = p2->X;
	p1->Y = p2->Y;
	p1->Z = p2->Z;
}
////////////////////////////////////////////////////////////////////////////////

void addPoints(pointStruct *s, pointStruct *p1, pointStruct *p2)
{
	s->X = p1->X + p2->X;
	s->Y = p1->Y + p2->Y;
	s->Z = p1->Z + p2->Z;
}

////////////////////////////////////////////////////////////////////////////////

void subtractPoints(pointStruct *s, pointStruct *p1, pointStruct *p2)
{
	s->X = p1->X - p2->X;
	s->Y = p1->Y - p2->Y;
	s->Z = p1->Z - p2->Z;
}

////////////////////////////////////////////////////////////////////////////////

boolean pointsAreTheSame(pointStruct *p1, pointStruct *p2, float epsilon)
{
	return(almostTheSameByDelta(p1->X, p2->X, epsilon) && almostTheSameByDelta(p1->Y, p2->Y, epsilon) && almostTheSameByDelta(p1->Z, p2->Z, epsilon));

}
////////////////////////////////////////////////////////////////////////////////

boolean vectorsAreTheSame(vectorStruct *v1, vectorStruct *v2, float epsilon)
{
	return(almostTheSameByDelta(v1->X, v2->X, epsilon) && almostTheSameByDelta(v1->Y, v2->Y, epsilon) && almostTheSameByDelta(v1->Z, v2->Z, epsilon) && almostTheSameByDelta(v1->mag, v2->mag, epsilon));
}

////////////////////////////////////////////////////////////////////////////////

float getMagnitude(float X, float Y, float Z)
{
	return(fpu_sqrtf(sqr(X) + sqr(Y) + sqr(Z)));
}

////////////////////////////////////////////////////////////////////////////////

float getVectorMagnitude(vectorStruct *v)
{
	return(fpu_sqrtf(sqr(v->X) + sqr(v->Y) + sqr(v->Z)));
}

////////////////////////////////////////////////////////////////////////////////

void normalizeVector(vectorStruct *v)
{
	if (v->mag > 0.0f)
	{
		v->X /= v->mag;
		v->Y /= v->mag;
		v->Z /= v->mag;
		v->mag = getVectorMagnitude(v); // recalc for safety ... but should be 1.0f
	}
	else
	{   // bad vector to start, so zero out
		v->X = 0.0f;
		v->Y = 0.0f;
		v->Z = 0.0f;
		v->mag = 0.0f;
	}
}
////////////////////////////////////////////////////////////////////////////////

vectorStruct *assignVector(vectorStruct *v, float X, float Y, float Z, boolean normalize)
{
	v->X = X;
	v->Y = Y;
	v->Z = Z;
	v->mag = getVectorMagnitude(v);
	if (normalize)
	{
		normalizeVector(v);
	}
	return(v);
}

////////////////////////////////////////////////////////////////////////////////

vectorStruct *assignVector2Pts(vectorStruct *v, pointStruct *p1, pointStruct *p2, boolean normalize)
{
	v->X = p1->X - p2->X;
	v->Y = p1->Y - p2->Y;
	v->Z = p1->Z - p2->Z;
	v->mag = getVectorMagnitude(v);
	if (normalize)
	{
		normalizeVector(v);
	}
	return(v);
}

////////////////////////////////////////////////////////////////////////////////

vectorStruct *scaleVector(vectorStruct *v1, vectorStruct *v2, float scale)
{
	v1->X = v2->X * scale;
	v1->Y = v2->Y * scale;
	v1->Z = v2->Z * scale;
	v1->mag = getVectorMagnitude(v1);
	return(v1);
}

////////////////////////////////////////////////////////////////////////////////

vectorStruct *entrywiseVectorSum(vectorStruct *v1, vectorStruct *v2, vectorStruct *v3)
{   // foreach ij, find V1ij = V2ij + V3ij;
	v1->X = v2->X + v3->X;
	v1->Y = v2->Y + v3->Y;
	v1->Z = v2->Z + v3->Z;
	v1->mag = getVectorMagnitude(v1);
	return(v1);
}
////////////////////////////////////////////////////////////////////////////////

vectorStruct *entrywiseVectorProduct(vectorStruct *v1, vectorStruct *v2, vectorStruct *v3)
{   // foreach ij, find V1ij = V2ij * V3ij;
	v1->X = v2->X * v3->X;
	v1->Y = v2->Y * v3->Y;
	v1->Z = v2->Z * v3->Z;
	v1->mag = getVectorMagnitude(v1);
	return(v1);
}

////////////////////////////////////////////////////////////////////////////////

float dotProduct(vectorStruct *u, vectorStruct *v)
{
	return((u->X * v->X) + (u->Y * v->Y) + (u->Z * v->Z));
}

////////////////////////////////////////////////////////////////////////////////

vectorStruct *crossProduct(vectorStruct *uv, vectorStruct *u, vectorStruct *v)
{
	uv->X = (u->Y * v->Z) - (v->Y * u->Z);
	uv->Y = - (u->X * v->Z) + (v->X * u->Z);
	uv->Z = (u->X * v->Y) - (v->X * u->Y);
	uv->mag = getVectorMagnitude(uv);
	return(uv);
}

////////////////////////////////////////////////////////////////////////////////

void initArcStruct(arcStruct *arc)
{
	bzero(arc, sizeof(arcStruct));
	arc->plane = _gs._selectedArcPlane;
}

////////////////////////////////////////////////////////////////////////////////

void sprintfPoint(char *s1, char *s2, pointStruct *p)
{
	sprintf(s1, "%s(%6.5f, %6.5f, %6.5f)", s2, p->X, p->Y, p->Z);
}

void sprintfVector(char *s1, char *s2, vectorStruct *v)
{
	sprintf(s1, "%s(%6.5f, %6.5f, %6.5f, %6.5f)", s2, v->X, v->Y, v->Z, v->mag);
}

void printPoint(pointStruct *p, char *s)
{
	sprintf(_tmpStr, "arc: %s.(x,y,z) = (%6.5f, %6.5f, %6.5f)", s, p->X, p->Y, p->Z);
	sendGB(_tmpStr);
}

void printVector(vectorStruct *v, char *s)
{
	sprintf(_tmpStr, "arc: %s->(x,y,z,mag) = (%6.5f, %6.5f, %6.5f, %6.5f)", s, v->X, v->Y, v->Z, v->mag);
	sendGB(_tmpStr);
}

void printExecutionPtrXYZ(float other)
{
	if (_arc.inscribedShape || (ARC_LENGTH_PER_SEGMENT > 3))
	{
		sprintf(_tmpStr, "arc: %03d: (x,y,z) = (%6.5f, %6.5f, %6.5f) [%4.3f]", getArgFraction(ARG_G) == 3 ?_arc.index : G2G3Flag, ARG_X, ARG_Y, ARG_Z, other);
		sendGB(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G0_0(void)  // Vector move at RAPID rate (uses ABCXYZF))
{
	// GCODE G0 [XYZABC dest/incr] [F feedrate]
	// GCODE    argXYZABC - destination/increment (G90/G91)
	// GCODE    argF - one-shot override of RAPID rate
	// GCODE
	// GCODE    NOTE: set per axis RAPID rate set via M203
	// GCODE
	// GCODE    Vector move at RAPID rate

	if (ARG_E_PRESENT)
	{
		ReportInvalidGcodeEArg();
		ARG_E = INVALID_ARG_VALUE;  // remove E value to avoid screwing up flow rate
	}
	motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
}

////////////////////////////////////////////////////////////////////////////////

MotorStructure *checkG0JogArgumentsOkay(void)
{
	if (ARG_E_PRESENT)
	{
		ReportInvalidGcodeEArg();
		ARG_E = INVALID_ARG_VALUE;  // remove E value to avoid screwing up flow rate
	}

	MotorStructure *M = NULL;

	if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { ReportGcodeError("Must specify one jog axis"); }
	else if (ARG_X_PRESENT && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_X]; }
	else if (ARG_X_MISSING && ARG_Y_PRESENT && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_Y]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_PRESENT && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_Z]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_PRESENT && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_A]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_PRESENT && ARG_C_MISSING) { M = &Motors[M_B]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_PRESENT) { M = &Motors[M_C]; }
	else { ReportGcodeError("Must specify one and only one jog axis"); }

	return(M);
}

////////////////////////////////////////////////////////////////////////////////


void G_Code_G0_1(void)  // Incremental single axis move at JOG rate based on incr distance
{
	// GCODE G0.1 [XYZABC incr]
	// GCODE    argXYZABC - incremental change of position based on incr distance
	// GCODE
	// GCODE    Incremental single axis move at JOG rate based on incr distance

	MotorStructure *M = checkG0JogArgumentsOkay();
	if (M != NULL)
	{
	   joggingUpdatePulses(M->Axis, getMotorArg(M));
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G0_2(void)  // Incremental single axis move at JOG rate based on incr quantity
{
	// GCODE G0.2 [XYZABC qty]
	// GCODE    argXYZABC - incremental change of position
	// GCODE
	// GCODE          set per axis incr amount set with M796
	// GCODE
	// GCODE    Incremental single axis move at JOG rate

	MotorStructure *M = checkG0JogArgumentsOkay();
	if (M != NULL)
	{
		joggingUpdatePulses(M->Axis, getMotorArg(M) * M->JogIncrement);
	}
}
////////////////////////////////////////////////////////////////////////////////

void G_Code_G0_3(void)  // Incremental single axis move at JOG rate based on incr step quantity
{
	// GCODE G0.3 [XYZABC steps]
	// GCODE    argXYZABC - incremental change of position based on incr step quantity
	// GCODE
	// GCODE    Incremental single axis move at JOG rate based on incr step quantity

	MotorStructure *M = checkG0JogArgumentsOkay();
	if (M != NULL)
	{
		joggingUpdatePulses(M->Axis, getMotorArg(M) * M->UnitsPerPulse);
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G0(void)    // G0 master command
{
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
	{   // lightburn has a F0 on all G0 lines
		ARG_F = INVALID_ARG_VALUE;
		if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING)
		{
			sendstringCr("Entering Lightburn Mode");
		}
	}
#endif //ALLOW_NATIVE_LIGHTBURN
	switch (getArgFraction(ARG_G))
	{
	case 0: G_Code_G0_0(); break;   // rapid move
	case 1: G_Code_G0_1(); break;   // jog move - distance
	case 2: G_Code_G0_2(); break;   // jog move - qty * setUnit (ie, direct from AB encoder
	case 3: G_Code_G0_3(); break;   // jog move - steps (ie, direct from AB encoder)
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G1_0(void)  //Coordinated move with optional extrude (uses ABCEFHSXYZ)
{
	// GCODE G1.1 [XYZABC dest/incr] [F rate] [E extrude] [H toolindex] [S scaleFlow]
	// GCODE    argXYZABC - destination/increment (G90/G91)
	// GCODE    argF - optional persistent change of the feedrate
	// GCODE    argE - optional enable extrusion or extrusion amount
	// GCODE    argH - optional persistent change of the tool length index
	// GCODE    argS - optional one-shot scaling of the current flow rate
	// GCODE
	// GCODE    Coordinated move with optional extrude

#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled && (_IncrementalMove == FALSE))
	{   // lightburn uses G1 to draw in absolute mode, but does not include an E value
		// uses G91 (incremental) to jog around
		ARG_E = 1.0;
	}
	if (ARG_S_PRESENT)
	{
		_gs._laser.vectorPowerPct  = ARG_S_MISSING ? 0.0f : (fFitWithinRange(ARG_S, 0.0f, 100.0f)) / 100.0f; // pct 0 to 1.0
		_gs._laser.piercePowerPct  = _gs._laser.vectorPowerPct;

		SetLaserPower(_gs._laser.vectorPowerPct, 0); // setup desired power in head, but do not turn on
		if (_gs._laser.outboxPtr)
			Send_Prime_Prams(_gs._laser.outboxPtr);//update the hothead

		if (_gs._laser.vectorPowerPct == 0.0f)
		{   // make sure laser is off for the move.
			ARG_E = INVALID_ARG_VALUE;
		}
	}
#endif //ALLOW_NATIVE_LIGHTBURN
	motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G1_1(void) //raster move (laser)
{
	if (_blockAllMotion)
	{
		catastrophicError("Cannot proceed with raster motion when motion is blocked");
		return;
	}
	_gs._laser.rasterReverseDirection = _gs._laser.rasterBidirectionalScan && (_gs._laser.rasterLineCounter & 1);

	_gs._laser.rasterTotalTraverseDots = processCurrentScanLine(_gs._laser.rasterReverseDirection);
	if (_gs._laser.rasterTotalTraverseDots)
	{   // work to do.
		float startX, startY, endX;
		float traverseLineLength = (float)((_gs._laser.rasterTotalTraverseDots + (2 * _gs._laser.rasterFrontPorchDots)) * _gs._laser.rasterPulsesPerDot) * Motors[M_X].UnitsPerPulse;

		if (_gs._laser.rasterReverseDirection)
		{   // odd line and scanning backwards
			startX = _gs._laser.rasterImageOriginMmX;
			startX += (_gs._laser.rasterTotalLineLength + _gs._laser.rasterBidirReturnStartAdjust);
			startX -= (((_gs._laser.rasterFirstTraverseDot + _gs._laser.rasterFrontPorchDots) * _gs._laser.rasterPulsesPerDot) * Motors[M_X].UnitsPerPulse);
			endX = startX - traverseLineLength;
		}
		else
		{   // scanning forward
			startX = _gs._laser.rasterImageOriginMmX;
			startX += (((_gs._laser.rasterFirstTraverseDot - _gs._laser.rasterFrontPorchDots) * _gs._laser.rasterPulsesPerDot) * Motors[M_X].UnitsPerPulse);
			endX = startX + traverseLineLength;
		}

		// which subDot-step to send out packet to head (inkjet should be in the middle, laser at the start)
		_gs._laser.rasterPulseToFireOn = ((getOutboxPointer(_gs._laser.device)->deviceFamily == DEVICE_FAMILY_LASER)) ? _gs._laser.rasterPulsesPerDot : (_gs._laser.rasterPulsesPerDot + 1) / 2;
		_gs._laser.rasterLineComplete = FALSE;
		// this code moves on fixed step grid.

		startY = _gs._laser.rasterImageOriginMmY + ((float)_gs._laser.rasterLineCounter / _gs._laser.rasterImageDotsPerMmY);

		_gs._laser.rasterTraverseDotCounter = _gs._laser.rasterFirstTraverseDot;

		if ((_sendingGBStringsMask & GB_STRING_RASTER) && (_sendingGBStringsSubMask & GB_STRING_RASTER_SUBMASK_DETAILS)) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr, "%c-%d) pp=%d t=%d f=%d Xs=%4.3f Y=%4.3f Xe=%4.3f L=%4.3f T=(=%d-%d) A=(%d-%d) ",
					_gs._laser.rasterReverseDirection ? 'R' : 'F',
					_gs._laser.rasterLineCounter, _gs._laser.rasterPulsesPerDot, _gs._laser.rasterTotalTraverseDots, _gs._laser.rasterFrontPorchDots,
					startX, startY, endX, traverseLineLength,
					_gs._laser.rasterFirstTraverseDot, _gs._laser.rasterLastTraverseDot, _gs._laser.rasterFirstActiveDot, _gs._laser.rasterLastActiveDot);
			sendGBNoCr(_tmpStr);
		}

		// do rapid move to start position
		InvalidateAllCmdArgs(ExecutionPtr); // clean out so a new command can be built
		ARG_X = startX;
		ARG_Y = startY;
		motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);

		if ((_sendingGBStringsMask & GB_STRING_RASTER) && (_sendingGBStringsSubMask & GB_STRING_RASTER_SUBMASK_DETAILS)) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"QP=(%ld/", Motors[M_X].Q_POSITION);
			sendstring(_tmpStr);
		}

		// do a raster scan move to the end position
		InvalidateAllCmdArgs(ExecutionPtr); // clean out so a new command can be built
		ARG_X = endX;
		//SAVE ARG_F = _gs._laser.rasterScanFeedrate;

		motionQ_addCommand(RASTER_MOTION_RATE, NOT_HOMING_MOTORS);

		if ((_sendingGBStringsMask & GB_STRING_RASTER) && (_sendingGBStringsSubMask & GB_STRING_RASTER_SUBMASK_DETAILS)) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr,"%ld/%ld) ", Motors[M_X].Q_POSITION, motionQ.newest->PULSES_TO_GO[M_X]);
			sendstringCr(_tmpStr);
			//sendGBNoCr("");
		}
	}
	_gs._laser.rasterLineCounter++; // line in the queue finished, so be ready for the next;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G1(void)    // G1 master command
{
	switch (getArgFraction(ARG_G))
	{
	case 0: G_Code_G1_0(); break;   // feedrate move
	case 1: G_Code_G1_1(); break;   // raster move
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void ExecuteArcPointMove(void)
{
	if (motionQ_full()) return;//no place to go, so get out of here until the que has space for next TWO move

	ExecutionPtr=_arc.saveExecutionPtr;//keep pointing at the arc command for information about how to move

	if (_arc.index > 0)
	{   // still work to do
		if ((_arc.index == 1) && (_arc.state == ARC_STATE_IDLE))
		{   //get to specified end point
			ARG_X = _arc.endPt.X;//end at the last point for sure
			ARG_Y = _arc.endPt.Y;
			ARG_Z = _arc.endPt.Z;
			ARG_A = _arc.endPtABC.X;
			ARG_B = _arc.endPtABC.Y;
			ARG_C = _arc.endPtABC.Z;
			CannedCycleLastMove = TRUE; //indicate this is last move of the arc

			if (_sendingGBStringsMask & GB_STRING_ARC_INFO) // use M797 S<mask> to enable
			{
				printExecutionPtrXYZ(0.0f);
			}
			motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);

			//last move has been added, so restore state
			_extrusionControl = _arc.saveExtrusionControl;
			_IncrementalEMove = _arc.saveIncrementalEMove;
			_IncrementalMove = _arc.saveIncrementalMove;
		}
		else
		{   //if we are still in the arc
			float pctIntoArc;
			float angle;
			float sinAngle;
			float cosAngle;

			pctIntoArc = ((float)_arc.segments - (float)(_arc.index - 1)) / (float)_arc.segments;

			if (_arc.dir == ARC_CW)
				angle = 0.0f + _arc.sweepAngle * pctIntoArc;
			else
				angle = (TWO_PI * (_arc.numFullThreads+1)) - _arc.sweepAngle * pctIntoArc;

			sinAngle = sin(angle);
			cosAngle = cos(angle);

			scaleVector(&_arc.displacementV, &_arc.rotationNormalUV, _arc.displacement * pctIntoArc);

			ARG_X = _arc.centerPt.X + (_arc.radius * ((cosAngle * _arc.center2startUV.X) + (sinAngle * _arc.ortho2center2startUV.X))) + _arc.displacementV.X;
			ARG_Y = _arc.centerPt.Y + (_arc.radius * ((cosAngle * _arc.center2startUV.Y) + (sinAngle * _arc.ortho2center2startUV.Y))) + _arc.displacementV.Y;
			ARG_Z = _arc.centerPt.Z + (_arc.radius * ((cosAngle * _arc.center2startUV.Z) + (sinAngle * _arc.ortho2center2startUV.Z))) + _arc.displacementV.Z;

			ARG_A = (_arc.deltaABC.X == 0) ? INVALID_ARG_VALUE : (pctIntoArc * _arc.deltaABC.X);
			ARG_B = (_arc.deltaABC.Y == 0) ? INVALID_ARG_VALUE : (pctIntoArc * _arc.deltaABC.Y);
			ARG_C = (_arc.deltaABC.Z == 0) ? INVALID_ARG_VALUE : (pctIntoArc * _arc.deltaABC.Z);

			CannedCycleLastMove = FALSE;
			if (_sendingGBStringsMask & GB_STRING_ARC_INFO) // use M797 S<mask> to enable
			{
				printExecutionPtrXYZ(angle);
			}
			motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);
			ARG_F = INVALID_ARG_VALUE;  // after the first point, feed rate is no longer needed and just wastes processing time
		}
		_arc.index--;//count down the number of points we are going to calculate this
	}
}

////////////////////////////////////////////////////////////////////////////////

void ExecuteG2G3PointMove(void)
{
	if (motionQ_full()) return; //no room to add next move, so leave

	ExecutionPtr=G2G3Ptr;//keep pointing at the arc command for information about how to move

	if (G2G3Flag > 0)
	{   // still work to do
		if (_spiralRoughingPass)
		{
			//WANT THIS: Radius = StartingRadus + ((((fabsf(CurrentAngle) - StartAngleInDegrees) / 360.0f) * ARG_I) / 2.0f); // slowly grow radius
			// divide by 2 because ARG_I is a diameter
			if (_deltaRForPass != 0.0f)
			{   // frogToe Mode uses _deltaRForPass
				Radius += ((fabsf(AngleIncrement) / 360.0f) * _deltaRForPass); // slowly grow radius
			}
			else
			{   //G12 G13
				Radius=(SpiralSegments-G2G3Flag)*SpiralRadiusIncrement;
				//Radius += SpiralRadiusIncrement; // slowly grow radius
			}
		}

		WorkAngleInRadians = (float)CurrentAngle * DEGREES_TO_RADIANS;
		XRadvalue=Radius * cosf(WorkAngleInRadians);
		yRADvalue=Radius * sinf(WorkAngleInRadians);
		CurrentAngle += AngleIncrement;//walk to the next point

		if ((G2G3Flag == 1) && (G203State == 0))
		{   //last move is always to the original start position/destination
			// no special move (regular G2/G3, so get to specified end point
//			if (_spiralRoughingPass) // kill material flow on last move
//			{
//				ARG_E = INVALID_ARG_VALUE;
//			}
			ARG_X = LastG2X;//end at the last point for sure
			ARG_Y = LastG2Y;
			ARG_Z = LastG2Z;
			CannedCycleLastMove = TRUE; // REVERTING Karl's change and restoring this line of code
			_spiralRoughingPass=FALSE;
			_deltaRForPass = 0.0f;
			G2DeltaZ = 0.0f;
			motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);

			if (_saveExtrusionControl != INVALID_E_CONTROL)
			{
				_extrusionControl = _saveExtrusionControl;  // restore state after a G2.1 or G3.1
				_saveExtrusionControl = INVALID_E_CONTROL; // flag that we're no longer saving state
			}
		}
		else
		{   //if we are still in the arc, include the delta z spiral if applicable
			ARG_Y = (float)(ArcCenterY + yRADvalue );//ArcPointsX[NextArcPointIndex];
			ARG_X = (float)(ArcCenterX + XRadvalue );//ArcPointsY[NextArcPointIndex];
			ARG_Z = (float)(LastG2Z - (((float)G2G3Flag / (float)NumberOfArcSegments) * G2DeltaZ)); // calc Z each step to avoid cumm summ error
			CannedCycleLastMove = FALSE;  // REVERTING Karl's change and restoring this line of code
			motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);
		}

		if (_sendingGBStringsMask & GB_STRING_CMD_QUE) // use M797 S<mask> to enable
		{
			if ((G2G3Flag == 1) && (G203State == 0))
			{
				sprintf(_tmpStr, "G%d finished (last move added to mQ)", (int)ARG_G); sendGB(_tmpStr);
			}
		}

		ARG_F = INVALID_ARG_VALUE;  // after the first point, feed rate is no longer needed and just wastes proc time
		G2G3Flag--;//count down the number of points we are going to calculate this

		if (G2G3Flag == 0)
		{   //last move has been added, so restore state
			_IncrementalEMove = _save_IncrementalEMove;  // restore state to before G2G3
			_IncrementalMove = _save_IncrementalMove;  // restore state to before G2G3
		}
	}
	else
	{
		//if you get here, we have just counted down the last arc segment motion and we should terminate if
		//the G203State is 0
		//otherwise we should check to see if we should be starting the next phase in the sequence engine
		//the motion que will have at least 1 open spot when you get here, so you can execute the first move of the new
		//step during this slice

		//      if(G203State==0)return;//finished now
		//          //_spiralRoughingPass = FALSE;  //needs to be somewhere else, when the logic is changing states

		int lastState=G203State; // needed for DEBUG STRING
		switch (G203State)
		{
		case 0 : return;
		case 1 : StartFinalReturnToCenterArc()  ;G203State=0; break;//final arc back to home
		case 2 : StartMainFinishCircle()        ;G203State=1; ;break;//main arc to make large bore circle
		case 3 : startMainRoughCircle()         ;break;//main arc-clearance for nice clean bore cycle
		case 4 : StartNextArcForPocketMill()    ;break;//initial lead in

		case 6 : frogToeCircle(); break;

		case 10 : StartMainFinishCircleSpiral()  ;G203State=9; break;//initial lead in for spiral plunge finished
		case 9 : G2DeltaZ=0 ; LastG2Z=spiralBoreZEndPosition;
				 StartMainFinishCircle()         ;G203State=8; break;//final arc back to home
		case 8 : StartFinalReturnToCenterArc()   ;G203State=7; break;//one loop at the bottom to clean up the bore
		case 7 : sendZBackToSpiralStartPosition();G203State=0;  break;//return to Z start before spiral
		}

		if (_sendingGBStringsMask & GB_STRING_CMD_QUE) // use M797 S<mask> to enable
		{
			if (G203State != lastState)
			{
				sprintf(_tmpStr, "G%d %d->%d", (int)ARG_G, lastState, G203State);  sendGB(_tmpStr);
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void sendZBackToSpiralStartPosition()
{
	ARG_Y = LastG2Y;//ArcPointsX[NextArcPointIndex];
	ARG_X = LastG2X;//ArcPointsY[NextArcPointIndex];
	LastG2Z = spiralBoreZStartPosition;
	ARG_Z = (spiralBoreZStartPosition); // calc Z each step to avoid cumm summ error
	CannedCycleLastMove = FALSE;
	motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
}

////////////////////////////////////////////////////////////////////////////////

passFail_t checkG2G3Errors(void)
{   // ERROR CHECKING

	// sweepIsMultipleOf180Degrees will be true for co-linear start and end vectors (180 and 360 deg)
	_arc.dotCheck = dotProduct(&_arc.center2startUV, &_arc.center2endUV);
	_arc.sweepIsMultipleOf180Degrees = (almostTheSameByDelta(_arc.dotCheck, 1.0f, ARC_TOLERANCE) || almostTheSameByDelta(_arc.dotCheck, -1.0f, ARC_TOLERANCE));

	if (_arc.radius == 0.0f)
	{
		sprintfPoint(_errorStr, "INVALID ARC CENTER (ZERO RADIUS): (I,J,K) = ", &_arc.centerOffset);
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	if (fabsf(_arc.center2startV.mag - _arc.center2endV.mag) > ARC_TOLERANCE)
	{
		sprintf(_errorStr, "INVALID ARC: center is not equidistant from start and end points (s=%4.3f, e=%4.3f):", _arc.center2startV.mag, _arc.center2endV.mag);
		sprintfPoint(_tmpStr, "  startPt=", &_arc.startPt); strcat(_errorStr, _tmpStr);
		sprintfPoint(_tmpStr, "  endPt=", &_arc.endPt); strcat(_errorStr, _tmpStr);
		sprintfPoint(_tmpStr, "  centerOffset=", &_arc.centerOffset); strcat(_errorStr, _tmpStr);
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	if (_arc.rotationNormalUV.mag == 0.0f)
	{   // bad vector
		sprintf(_errorStr, "INVALID ARC NORMAL (specified): G16=%d G17=%d G18=%d G19=%d ",
				(_arc.plane==ARC_PLANE_NONE), (_arc.plane==ARC_PLANE_XY),
				(_arc.plane==ARC_PLANE_XZ), (_arc.plane==ARC_PLANE_YZ));
		sprintfVector(_tmpStr, "uvw=", &_arc.rotationNormalUV); strcat(_errorStr, _tmpStr);
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	if ((_arc.derivedNormalUV.mag == 0.0f) && !_arc.sweepIsMultipleOf180Degrees)
	{   // can only check if not 180 or 360
		sprintf(_errorStr, "INVALID ARC NORMAL (derived): G16=%d G17=%d G18=%d G19=%d ",
				(_arc.plane==ARC_PLANE_NONE), (_arc.plane==ARC_PLANE_XY),
				(_arc.plane==ARC_PLANE_XZ), (_arc.plane==ARC_PLANE_YZ));
		sprintfVector(_tmpStr, "uvw=", &_arc.derivedNormalUV); strcat(_errorStr, _tmpStr);
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	if (ARG_D_PRESENT && (_arc.plane != ARC_PLANE_NONE))
	{   // check for redundant arguments  (G17 with argZ and argD; G18 with argY and argD; G19 with argX and argD
		if (((_arc.plane == ARC_PLANE_XY) && ARG_Z_PRESENT) || ((_arc.plane == ARC_PLANE_XZ) && ARG_Y_PRESENT) || ((_arc.plane == ARC_PLANE_YZ) && ARG_X_PRESENT))
		{
			sprintf(_errorStr, "INVALID ARC HELIX - dual displacements specified (argD and orthogonal axis");
			catastrophicError(_errorStr);
			return(FAIL);//exit with out moving, as there is NO valid option
		}
	}

	if ((_arc.normalsSameDir == _arc.normalsOppoDir) && !_arc.sweepIsMultipleOf180Degrees)
	{   // derived and spec'd normals do not match, so flag error (should either be just SameDir or just OppoDir, not both or neither
		// can only check if not 180 or 360
		sprintf(_errorStr, "ARC NORMALS MISMATCH (%d/%d): ", _arc.normalsSameDir, _arc.normalsOppoDir);
		sprintfVector(_tmpStr, "specified=", &_arc.rotationNormalUV); strcat(_errorStr, _tmpStr);
		sprintfVector(_tmpStr, " derived=", &_arc.derivedNormalUV); strcat(_errorStr, _tmpStr);
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	if ((_arc.plane != ARC_PLANE_NONE) && (ARG_U_PRESENT || ARG_V_PRESENT || ARG_W_PRESENT))
	{   // check for dual specified normals
		sprintf(_errorStr, "DUAL ARC NORMALS:  G17/G17/G19 active and U/V/W specified");
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	if (_arc.normalWasSpecified)
	{   // normal provided either via G17/18/19 OR UVW -- make sure it's perpendicular to start and end vectos
		if (!((almostTheSameByDelta(dotProduct(&_arc.center2startUV, &_arc.rotationNormalUV), 0.0f, ARC_TOLERANCE)) &&
				(almostTheSameByDelta(dotProduct(&_arc.center2endUV, &_arc.rotationNormalUV), 0.0f, ARC_TOLERANCE))))
		{   // both dot products need to be 0 in order to confirm specified normal is perpendicular to the plane formed by the 3 arc points
			sprintf(_errorStr, "SPECIFIED NORMAL NOT PERPENDICULAR TO PLANE FORMED BY ARC START/CENTER/END");
			catastrophicError(_errorStr);
			return(FAIL);//exit with out moving, as there is NO valid option
		}
	}

	if ((_arc.normalWasSpecified == FALSE) && _arc.sweepIsMultipleOf180Degrees)
	{   // letting code derive the axis of ratation works unless angle is multiple of 180
		sprintf(_errorStr, "Free space arcs of 180 or 360 MUST specify a vector (u,v,w) or G17/G18/G19 normal to the plane of rotation (%4.3f)", _arc.dotCheck);
		catastrophicError(_errorStr);
		return(FAIL);//exit with out moving, as there is NO valid option
	}

	return(PASS);
}

////////////////////////////////////////////////////////////////////////////////

void ProcessG2_3G3_3Command(int gcode) //traverse a CW/CCW arc
{
	// GCODE G2.3/G3.3 [XYZ arcDest] [ABC dest] [IJK ofs2center] [UVW normal] [D displacement] [P pitch] [E amount] [F feedrate]
	// GCODE
	// GCODE    the current XYZ position is starting point of the clockwise arc
	// GCODE
	// GCODE    the specified XYZ position is the final destination of the arc  {the current position is used
	// GCODE        to fill in any missing XYZ fields}
	// GCODE
	// GCODE    the specified ABC axes will uniformly move during the arc
	// GCODE
	// GCODE    the specified IJK are the relative offset from current XYZ to the arc center/origin {'0.0' is
	// GCODE        used to fill in any missing IJK fields}
	// GCODE
	// GCODE    if G17/G18/G19 are active, the plane of rotation defined as XY, XZ, YZ respectively and
	// GCODE        and the remaining axis or argD specified the displacement (helical arc). for example
	// GCODE        if G17-PLANE_XY is active, argZ can be used to specify the displacement for Z following
	// GCODE        the same G90/G91 definition.  argD is always a relative value.
	// GCODE    else if G16 active (no pre-defined plane)
	// GCODE        the relative displacement can only be specified by argD (free space helical arc).
	// GCODE        the plane of rotation can be defined one of two ways:
	// GCODE        1)  a vector, UVW, normal to to the surface/plane of rotation (no need to be a unit vector).
	// GCODE            it is an error if the UVW vector is not perpendicular to both the center2start and c
	// GCODE            enter2end vectors; both of which are in the plane of rotation. {'0.0' is used to fill
	// GCODE            in any missing UVW fields}.
	// GCODE        2)  for all cases except for integer multiples of 180 (180, 360, 540, 720, etc), the normal
	// GOCDE            is automatically inferred from the 3 defined points of the arc (start, center, end).
	// GCODE            in the N*180 case, the 3 points are co-linear and do not define a plane.  an error
	// GCODE            generated for the N*180 case when using an inferred normal.
	// GCODE
	// GCODE    if argP specified, then the integer portion of displacement/pitch will determine the
	// GCODE        the number of full circles to traverse and the fractional remainder will be the final
	// GCODE        arc to the endpoint.  it is an error if the remainder and the angle formed by the
	// GCODE        start/center/end points are not consistent.
	// GCODE
	// GCODE    if argF specified, then use the newly specified feedrate
	// GCODE
	// GCODE    if argE specified then  argE is material for ENTIRE move (including multi lap moves)
	// GCODE
	// GCODE    traverse a CW/CCW arc following the LHR relative to the direction of the normal
	// GCODE        {for reference, G17 sets PLANE_XY with a normal of [0,0,1] such that the clockwise
	// GCODE        arc is traversed on the print bed when looking down on the bed}


	initArcStruct(&_arc); // reset everything
	_arc.saveExecutionPtr = ExecutionPtr;   // save a copy
	_arc.dir = (gcode == 2) ? ARC_CW : ARC_CCW;

	// multiple ways to specify which plane of rotation to use
	//        1.  *using G17/18/19 - user can force one of the 3 primary planes (XY, XZ, YZ)
	//            *when using these modes, the axis variable not involved in the arc can specify
	//             the displacement/final position to use during the move (form a helix)
	//            *alternatively, the user can use argD to specify the displacement along the axis
	//             normal to plane of rotation
	//            *if both argD and axis normal arg are specified, and error is issued
	//            *arg U, V, and W are ignored is G17/18/19 are invoked (warning issued)
	//
	//        2.  *invoking G16 and the specifying an arbitrary normal vector using U, V, and W
	//             (the system will normalize the vector so a unit vector is not necessary).
	//            *when in this mode, a helix can still be formed, but the displacement can
	//             only be specified with argD.
	//
	//        3.  *for non-180 and non-360 degree arcs, the code can calculate the normal from
	//             the 3 specified points of the arc (start, end, and center).  this is to allow
	//             for cases where G16 is invoked AND U, V, and W are omitted
	//            *if this mode is used to specify a 180 or 360 deg arc, an error is generated
	//            *when in this mode, a helix can still be formed, but the displacement can
	//             only be specified with argD.
	//
	//  steps 1 and 2 must be performed/checked before trying to derive the normal for use
	//  directly for step 3 or for error check later, in case one of XYZ is used for displacement
	//  that displacement value must be removed before the start/end/center points are determined

	if (_arc.plane != ARC_PLANE_NONE)
	{   //case 1 above -  arc is defined to be in one of the 3 primary planes XY, XZ, YZ
		switch(_arc.plane)
		{
		case ARC_PLANE_XY :
			assignVector(&_arc.rotationNormalUV, 0.0f, 0.0f, 1.0f, TRUE);
			_arc.displacement = (ARG_D_PRESENT) ? ARG_D : (ARG_Z_PRESENT ? getDeltaAxisValueInInputUnits(&Motors[M_Z]) : 0.0f);
			ARG_Z = INVALID_ARG_VALUE;  // displacement added pre arc-step later
			_arc.normalWasSpecified = TRUE;
			break;
		case ARC_PLANE_XZ :
			assignVector(&_arc.rotationNormalUV, 0.0f, 1.0f, 0.0f, TRUE);
			_arc.displacement = (ARG_D_PRESENT) ? ARG_D : (ARG_Y_PRESENT ? getDeltaAxisValueInInputUnits(&Motors[M_Y]) : 0.0f);
			ARG_Y = INVALID_ARG_VALUE;  // displacement added pre arc-step later
			_arc.normalWasSpecified = TRUE;
			break;
		case ARC_PLANE_YZ :
			assignVector(&_arc.rotationNormalUV, 1.0f, 0.0f, 0.0f, TRUE); break;
			_arc.displacement = (ARG_D_PRESENT) ? ARG_D : (ARG_X_PRESENT ? getDeltaAxisValueInInputUnits(&Motors[M_X]) : 0.0f);
			ARG_X = INVALID_ARG_VALUE;  // displacement added pre arc-step later
			_arc.normalWasSpecified = TRUE;
		default:
			sprintf(_errorStr, "UNKNOWN ARC PLANE (%d)", _arc.plane);
			barf(_errorStr);
			return;
		}
	}
	else if (ARG_U_PRESENT || ARG_V_PRESENT || ARG_W_PRESENT)
	{   // case 2:  normal specified with U/V/W --- get vector and normalize
		assignVector(&_arc.rotationNormalUV, ARG_U_MISSING ? 0.0f : ARG_U, ARG_V_MISSING ? 0.0f : ARG_V, ARG_W_MISSING ? 0.0f : ARG_W, TRUE);
		normalizeVector(&_arc.rotationNormalUV);
		_arc.displacement = (ARG_D_PRESENT) ? ARG_D : 0.0f;
		_arc.normalWasSpecified = TRUE;
	}

	// now that any helical displacements have been removed from the XYZ args, it is safe to derive the normal based on the 3 arc points

	assignPoint(&_arc.startPt, Motors[M_X].Q_LastRequestedPositionInUnits, Motors[M_Y].Q_LastRequestedPositionInUnits, Motors[M_Z].Q_LastRequestedPositionInUnits); // Arc Start Pt

	if (_IncrementalMove)
	{
		pointStruct incr;   // temp pt variable to deal with incremental move
		assignPoint(&incr, ARG_X_PRESENT ? ARG_X : 0.0f,
						   ARG_Y_PRESENT ? ARG_Y : 0.0f,
						   ARG_Z_PRESENT ? ARG_Z : 0.0f);

		addPoints(&_arc.endPt, &_arc.startPt, &incr); // Arc End Pt

		assignPoint(&_arc.deltaABC, ARG_A_PRESENT ? ARG_A : 0.0f,
									ARG_B_PRESENT ? ARG_B : 0.0f,
									ARG_C_PRESENT ? ARG_C : 0.0f);

		assignPoint(&_arc.endPtABC, ARG_A_PRESENT ? Motors[M_A].Q_LastRequestedPositionInUnits + ARG_A : INVALID_ARG_VALUE,
									ARG_B_PRESENT ? Motors[M_B].Q_LastRequestedPositionInUnits + ARG_B : INVALID_ARG_VALUE,
									ARG_C_PRESENT ? Motors[M_C].Q_LastRequestedPositionInUnits + ARG_C : INVALID_ARG_VALUE);
	}
	else
	{   // absolute move
		assignPoint(&_arc.endPt, ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits,
								 ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits,
								 ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits); // Arc End Pt

		assignPoint(&_arc.deltaABC, ARG_A_PRESENT ? getDeltaAxisValueInInputUnits(&Motors[M_A]) : 0.0f,
									ARG_B_PRESENT ? getDeltaAxisValueInInputUnits(&Motors[M_B]) : 0.0f,
									ARG_C_PRESENT ? getDeltaAxisValueInInputUnits(&Motors[M_C]) : 0.0f); // delta ABC

		assignPoint(&_arc.endPtABC, ARG_A_PRESENT ? ARG_A : INVALID_ARG_VALUE,
									ARG_B_PRESENT ? ARG_B : INVALID_ARG_VALUE,
									ARG_C_PRESENT ? ARG_C : INVALID_ARG_VALUE);
	}

	_arc.fullCircle = pointsAreTheSame(&_arc.startPt, &_arc.endPt, ARC_TOLERANCE);  // special case

	assignPoint(&_arc.centerOffset, ARG_I_MISSING ? 0.0f : ARG_I, ARG_J_MISSING ? 0.0f : ARG_J, ARG_K_MISSING ? 0.0f : ARG_K);
	addPoints(&_arc.centerPt, &_arc.startPt, &_arc.centerOffset);                   //arc center pt

	assignVector2Pts(&_arc.center2startV, &_arc.startPt, &_arc.centerPt, FALSE);    //vector from center pt to start point (mag == radius)
	_arc.center2startUV = _arc.center2startV;                                       //copy vector
	normalizeVector(&_arc.center2startUV);                                          //normalized vector from center pt to start point (mag == radius)

	assignVector2Pts(&_arc.center2endV, &_arc.endPt, &_arc.centerPt, FALSE);        //vector from center pt to end point (mag == radius)
	_arc.center2endUV = _arc.center2endV;                                           //copy vector
	normalizeVector(&_arc.center2endUV);                                            //normalized vector from center pt to end point (mag == radius)

	_arc.radius = _arc.center2startV.mag;

	normalizeVector(crossProduct(&_arc.derivedNormalUV, &_arc.center2endV, &_arc.center2startV)); // derive perpendicular vector from start/end/center points/vectors

	if (((_arc.plane == ARC_PLANE_NONE) && ARG_U_MISSING && ARG_V_MISSING && ARG_W_MISSING) || (_arc.normalWasSpecified == FALSE))
	{   // step 3:  no uvw normal specified and no normal selected with G17/G18/G19 - so use derived one for the rotation
		_arc.rotationNormalUV = _arc.derivedNormalUV;   //use the derived normal, none specified
		_arc.displacement = (ARG_D_PRESENT) ? convertArgToMM(ARG_D) : 0.0f;
		_arc.normalWasSpecified = FALSE;
	}

	scaleVector(&_arc.reverseDerivedNormalUV, &_arc.derivedNormalUV, -1.0f);    // reverse dir of derived normal
	_arc.normalsSameDir = vectorsAreTheSame(&_arc.rotationNormalUV, &_arc.derivedNormalUV, ARC_TOLERANCE);
	_arc.normalsOppoDir = vectorsAreTheSame(&_arc.rotationNormalUV, &_arc.reverseDerivedNormalUV, ARC_TOLERANCE);

	if (_arc.normalsOppoDir)
	{
		//_arc.dir = (_arc.dir == ARC_CW) ? ARC_CCW : ARC_CW;   // > 180 so flip dir for rest of code to be happy
		_arc.displacement *= -1.0f; // oppo dir so flip displacement
	}

	crossProduct(&_arc.ortho2center2startUV, &_arc.center2startUV, &_arc.rotationNormalUV); // get a vector perpendicular to the center2start vecotr, but in the plane of rotation
																							// used calc incremental arc points
	if (_sendingGBStringsMask & GB_STRING_ARC_INFO) // use M797 S<mask> to enable
	{   // dump debug data
		sendGB("");
		sprintf(_rptStr, "Starting G%d Arc from ", gcode);
		sprintfPoint(_tmpStr, "", &_arc.startPt);   strcat(_rptStr, _tmpStr);
		sprintfPoint(_tmpStr, " to ", &_arc.endPt);   strcat(_rptStr, _tmpStr);
		sprintfPoint(_tmpStr, " w/ center ", &_arc.centerPt);   strcat(_rptStr, _tmpStr);
		sprintf(_tmpStr, " r=%4.3f ", _arc.radius);   strcat(_rptStr, _tmpStr);
		sendGB(_rptStr);
		printVector(&_arc.derivedNormalUV, "derivedNormalUV");  // derived normal
		printVector(&_arc.reverseDerivedNormalUV, "reverseDerivedNormalUV");    // reverse derived normal
		sprintf(_tmpStr, "displacement=%4.3f", _arc.displacement); sendGB(_tmpStr);
		sprintf(_tmpStr, "normalsSame=%d  normalsOppo=%d", _arc.normalsSameDir, _arc.normalsOppoDir); sendGB(_tmpStr);
		printVector(&_arc.rotationNormalUV, "rotationalAxisUV");    // specified normal
		printVector(&_arc.ortho2center2startUV, "ortho2center2startUV");    // 90 deg ortho vector
	}

	// need to check at this point to avoid possible divide by 0 is there is bad input data
	if (checkG2G3Errors() == FAIL)
	{
		return;
	}

	if (_arc.fullCircle)
		_arc.sweepAngle = TWO_PI;
	else if ((_arc.normalsOppoDir && (_arc.dir == ARC_CW)) || (_arc.normalsSameDir && (_arc.dir == ARC_CCW)))
		_arc.sweepAngle = TWO_PI - acosf(dotProduct(&_arc.center2startV, &_arc.center2endV) / (_arc.center2startV.mag * _arc.center2endV.mag));
	else
		_arc.sweepAngle = acosf(dotProduct(&_arc.center2startV, &_arc.center2endV) / (_arc.center2startV.mag * _arc.center2endV.mag));

	if (_arc.fullCircle && ARG_S_PRESENT && (ARG_S >= 3.0f))
	{ // full circle -- and want to do an inscribed shape
		_arc.segments = (int)ARG_S; // full circles can force number of points/sides/segments with ARG_S
		_arc.inscribedShape = TRUE;
		_arc.fullCircleSegments = (int)ARG_S;
	}
	else
	{
		float segmentLength = (ARG_S_PRESENT && (ARG_S < 3.0f) && (ARG_S > 0.0f)) ? ARG_S : ARC_LENGTH_PER_SEGMENT;   //user can override seg length (up to 3.0mm)
		_arc.segments = imax(1, (int)roundf(_arc.radius * fabsf(_arc.sweepAngle) / segmentLength));
		_arc.fullCircleSegments = imax(1, (int)roundf(_arc.radius * TWO_PI / segmentLength));   // needed for threading
	}
	ARG_S = INVALID_ARG_VALUE;  // ARG_S is used as a one time flow scale on printing moves, so need to shut this off for arcs

	if (_sendingGBStringsMask & GB_STRING_ARC_INFO) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr, "pre-T sweepAngle = acosf(dotProduct(X)) = %4.3f", _arc.sweepAngle); sendGB(_tmpStr);
		sprintf(_tmpStr, "pre-T segments = %d", _arc.segments); sendGB(_tmpStr);
	}

	if (ARG_P_PRESENT)
	{   // process threading
		float numTotalThreads = fabsf(_arc.displacement / ARG_P);
		_arc.numFullThreads = (int)numTotalThreads;
		float partialThread = numTotalThreads - (float)_arc.numFullThreads;
		if (_arc.fullCircle)
		{   // check full circle case
			_arc.threadingOK = almostTheSameByDelta(partialThread, 0.0f, ARC_TOLERANCE);  // should be integral number of threads
			_arc.sweepAngle *= _arc.numFullThreads;
			_arc.segments *= _arc.numFullThreads;
		}
		else
		{
			_arc.threadingOK = almostTheSameByDelta(partialThread, _arc.sweepAngle / TWO_PI, ARC_TOLERANCE);
			_arc.sweepAngle += TWO_PI * _arc.numFullThreads;
			_arc.segments += _arc.fullCircleSegments * _arc.numFullThreads;
		}

		if (_arc.threadingOK == FALSE)
		{
			sprintf(_errorStr, "INVALID G2/G3: Pitch vs Displacemet vs Angle mismatch:  disp=%4.3f pitch=%4.3f rotations=%4.3f angle=%4.3f (%4.3f)",
					_arc.displacement, ARG_P, numTotalThreads, _arc.sweepAngle*RADIANS_TO_DEGREES, _arc.sweepAngle/TWO_PI);
			catastrophicError(_errorStr);
			return;//exit with out moving, as there is NO Valid OPTION
		}
	}

	if (_sendingGBStringsMask & GB_STRING_ARC_INFO) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr, "post-T sweepAngle = acosf(dotProduct(X)) = %4.3f", _arc.sweepAngle); sendGB(_tmpStr);
		sprintf(_tmpStr, "post-T segments = %d", _arc.segments); sendGB(_tmpStr);
		sprintfPoint(_tmpStr, "endPt w/o disp = ", &_arc.endPt);   sendGB(_tmpStr);
	}

	scaleVector(&_arc.displacementV, &_arc.rotationNormalUV, _arc.displacement);
	_arc.endPt.X += _arc.displacementV.X;
	_arc.endPt.Y += _arc.displacementV.Y;
	_arc.endPt.Z += _arc.displacementV.Z;

	if (_sendingGBStringsMask & GB_STRING_ARC_INFO) // use M797 S<mask> to enable
	{
		sprintfVector(_tmpStr, "displacementV = ", &_arc.displacementV);   sendGB(_tmpStr);
		sprintfPoint(_tmpStr, "endPt w/ disp = ", &_arc.endPt);   sendGB(_tmpStr);
	}

	_IncrementalMove = FALSE;                       // force absolute mode for XYZ to make the arc work; will be restorded after the arc completes
	_IncrementalEMove = TRUE;                       // force incremental mode to make the arc work; will be restorded after the arc completes

	if ((ARG_E_PRESENT) && (_extrusionControl == USE_E_VALUES))  //using E values
	{   // E values matter, so adjust value to be correct per segment
		// if an inscribed shape (full cicle with args >= 3), then find the % for the circum of inscribed shape vs a full circle;
		float scaleEForInscribedShape = _arc.inscribedShape ? (_arc.fullCircleSegments * sin(PI / _arc.fullCircleSegments)) / PI : 1.0f;
		ARG_E = (getDeltaEValueInInputUnits() * scaleEForInscribedShape) / (float)_arc.segments; // divide by num segments to get per seg amount
	}

	_arc.index = _arc.segments;
	_arc.state = ARC_STATE_IDLE;

	ExecuteArcPointMove();
}

////////////////////////////////////////////////////////////////////////////////

void ProcessG2_2G3_2Command(int G2_G3)
{
	// GCODE G2.2/G3.2 [XYZ spiralDest] [IJ ofs2center] <P pitch> {L laps] [E amount] [S segLength/#segments] [F feedrate]
	// GCODE
	// GCODE    the current XYZ position is starting point of the spiral arc
	// GCODE
	// GCODE    the specified XYZ position is the final destination of the arc  {the current position is used
	// GCODE        to fill in any missing XYZ fields}
	// GCODE
	// GCODE    ABC axes will not move
	// GCODE
	// GCODE    the specified IJ are the relative offset from current XY to the arc center/origin {'0.0' is
	// GCODE        used to fill in any missing IJ fields}
	// GCODE
	// GCODE    argP required: the centerline distance between spiral arcs
	// GCODE
	// GCODE    if argL specified, then integer number of laps = argL
	// GCODE    else (no argL or argL==0), laps = integer(radius/argP))
	// GCODE
	// GCODE    if argF specified, then use the newly specified feedrate
	// GCODE
	// GCODE    if argE specified the M229 Ex setting is ignored and flow is autocalculated based on M221(M756/M575) settings
	// GCODE
	// GCODE    if argS specified, then controls the number of segments in the arc as follows
	// GCODE        argS < 3, then argS is the segment length (# segments = argLength / argS)
	// GCODE        arcs >= 3, the argS is the number of sides for a full circular arc (ie, argS=4 will make a square)
	// GCODE            if argE and USING_E_VALUES, argE is scaled by the ratio of the perimeter of the inscribed shape
	// GCODE            over the circumference of the defined circle.
	// GCODE
	// GCODE    traverse a CW/CCW arc following the LHR relative to the direction of the normal
	// GCODE        {for reference, G17 sets PLANE_XY with a normal of [0,0,1] such that the clockwise
	// GCODE        arc is traversed on the print bed when looking down on the bed}

	arcArgS = ARG_S;
	ARG_S = INVALID_ARG_VALUE;  // ARG_S is used as a one time flow scale on printing moves

	ARG_A = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3
	ARG_B = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3
	ARG_C = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3

	G2G3Ptr=ExecutionPtr; //save the index in the que so we can come back to this.
	Radius=0;
	theta=0;
	XRadvalue=0;
	yRADvalue=0;
	ArcSubcounter=0;

	if (ARG_P_MISSING && ARG_L_MISSING)   { ReportMissingGcodePLArg();  return; }

	if (ARG_I_MISSING)
		ARG_I = 0.0f;           //no argument means it has zero offset from axis
	if (ARG_J_MISSING)
		ARG_J = 0.0f;

	pointStruct startPt;
	pointStruct endPt;
	pointStruct centerPt;

	assignPoint(&startPt,   Motors[M_X].Q_LastRequestedPositionInUnits,
							Motors[M_Y].Q_LastRequestedPositionInUnits,
							Motors[M_Z].Q_LastRequestedPositionInUnits);

	assignPoint(&centerPt,  startPt.X + ARG_I,
							startPt.Y + ARG_J,
							0.0f);

	if (_IncrementalMove)
	{
		assignPoint(&endPt, startPt.X + (ARG_X_PRESENT ? ARG_X : 0.0f),
							startPt.Y + (ARG_Y_PRESENT ? ARG_Y : 0.0f),
							startPt.Z + (ARG_Z_PRESENT ? ARG_Z : 0.0f));

	}
	else
	{   // absolute move
		assignPoint(&endPt, ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits,
							ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits,
							ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits);
	}

	float distStartToCenter = fpu_sqrtf(sqr(startPt.X - centerPt.X) + sqr(startPt.Y - centerPt.Y));
	float distEndToCenter   = fpu_sqrtf(sqr(endPt.X - centerPt.X) + sqr(endPt.Y - centerPt.Y));
	float distStartToEnd    = fpu_sqrtf(sqr(startPt.X - endPt.X) + sqr(startPt.Y - endPt.Y));

	if ((distStartToEnd > distStartToCenter) && (distStartToEnd > distEndToCenter))
	{   // start and end are on opposites sides of the centerPt
		sprintf(_errorStr, "G%d_2: start, center, and end points must form a N*360 deg spiral", G2_G3);
		catastrophicError(_errorStr);
		return;
	}

	//if ((distStartToCenter != 0.0f) || (distEndToCenter != 0.0f))
	//if ((distStartToCenter != 0.0f) && (distEndToCenter != 0.0f))
	if (notAlmostTheSameByDelta(distStartToCenter, 0.0f, ARC_TOLERANCE) && notAlmostTheSameByDelta(distEndToCenter, 0.0f, ARC_TOLERANCE))
	{ //not a fully filled spiral  (doesn't start or end at the center')
		float slopeStartToCenter = ((startPt.Y - centerPt.Y) != 0.0f) ? ((startPt.X - centerPt.X) / (startPt.Y - centerPt.Y)) : MAXFLOAT;
		float slopeEndToCenter   = ((endPt.Y - centerPt.Y)   != 0.0f) ? ((endPt.X - centerPt.X)   / (endPt.Y - centerPt.Y))   : MAXFLOAT;

		if (notAlmostTheSameByDelta(slopeStartToCenter, slopeEndToCenter, ARC_TOLERANCE))
		{
			sprintf(_errorStr, "G%d_2: start, center, and end points must be colinear (%4.3f/%4.3f", G2_G3, slopeStartToCenter, slopeEndToCenter);
			catastrophicError(_errorStr);
			return;
		}
	}

	_reverseFrogToe = FALSE;
	if (almostTheSameByDelta(distStartToEnd, 0.0f, ARC_TOLERANCE))
	{   // already at destination -- do a full circle
			sprintf(_errorStr, "No spiral detected: check L(%d), P(%3.1f) and effective radius (%3.1f) in G%2.1f", (int)ARG_L, ARG_P, Radius, ARG_G);
			catastrophicError(_errorStr);
			return;//exit with out moving, as there is NO Valid OPTION

	}
	else if (distStartToCenter > distEndToCenter)
	{   // spiral in
		Radius = distStartToCenter; //starting radius

	}
	else if (distStartToCenter < distEndToCenter)
	{   // spiral out
		_reverseFrogToe = TRUE;
		Radius = distStartToCenter;	// starting radius
	}
	else //if (distStartToCenter == distEndToCenter)
	{   // half circle
		return; // alread trapped with checks above
	}
	/////////////////////////////////////////////////////////

	ARG_X = startPt.X;
	ARG_Y = startPt.Y;
	ARG_Z = startPt.Z;

	LastG2X = endPt.X;
	LastG2Y = endPt.Y;
	LastG2Z = endPt.Z;

	ArcCenterX=centerPt.X;
	ArcCenterY=centerPt.Y;

	_frogToePasses = 0;

	if (ARG_L_MISSING)
	{   // pitch defines number of laps
		if (ARG_P > 0.0f)   // would otherwise divide by 0 or worse
			_frogToePasses = (int)(distStartToEnd / ARG_P);
		else
			_frogToePasses = 0;
	}
	else if (ARG_P_MISSING)
	{   // laps defines pitch
		_frogToePasses = imax(1, (int)ARG_L);
	}
	else
	{   // both P and L present
		_frogToePasses = imax(1, (int)ARG_L);
		if ((ARG_P * _frogToePasses) > distStartToEnd)
		{   // doesn't fit, so reduce number of passes
			_frogToePasses = (int)(distStartToEnd / ARG_P);
		}
	}
	if (_frogToePasses > 0)
	{
		ARG_P = distStartToEnd / (float)_frogToePasses; // recalc P to match reality
	}

	if (_frogToePasses == 0)
	{
		sprintf(_errorStr, "No spiral detected: check L(%d), P(%3.1f) and effective radius (%3.1f) in G%2.1f", (int)ARG_L, ARG_P, Radius, ARG_G);
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}

	ARG_L = INVALID_ARG_VALUE;  // nullify -- no longer needed

	G2DeltaZ = (LastG2Z - Motors[M_Z].Q_LastRequestedPositionInUnits) / (float)_frogToePasses; // change of Z per lap

	_saveExtrusionControl = _extrusionControl;  // save current state so more can be forced to auto calc flow
	_extrusionControl = IGNORE_E_VALUES;
	G2G3Flag = 0;
	G203State = 6;  // frogToeCircle mode
}

////////////////////////////////////////////////////////////////////////////////

void ProcessG2_1G3_1Command(int G2_G3) // spiral helical arc in XY plane w/ Z displacement (uses Z,I,J,L,P,S,E,F)
{
	// GCODE G2.1/G3.1 [Z spiralDest] [IJ ofs2center] <P pitch> {L laps] [E amount] [S segLength/#segments] [R reverseCode] [F feedrate[
	// GCODE
	// GCODE    the current XYZ position is starting point of the spiral arc
	// GCODE
	// GCODE    the specified Z position is the final destination of the arc  {the current position is used
	// GCODE        to fill in any missing Z fields} {ERROR: to specify X/Y]
	// GCODE
	// GCODE    ABC axes will not move
	// GCODE
	// GCODE    the specified IJ are the relative offset from current XY to the arc center/origin {'0.0' is
	// GCODE        used to fill in any missing IJ fields}
	// GCODE
	// GCODE    argP required: the centerline distance between spiral arcs
	// GCODE
	// GCODE    if argL specified, then integer number of laps = argL
	// GCODE    else (no argL or argL==0), laps = integer(radius/argP)
	// GCODE
	// GCODE    if argF specified, then use the newly specified feedrate
	// GCODE
	// GCODE    if argE specified the M229 Ex setting is ignored and flow is autocalculated based on M221(M756/M575) settings
	// GCODE
	// GCODE    if argS specified, then controls the number of segments in the arc as follows
	// GCODE        argS < 3, then argS is the segment length (# segments = argLength / argS)
	// GCODE        arcs >= 3, the argS is the number of sides for a full circular arc (ie, argS=4 will make a square)
	// GCODE            if argE and USING_E_VALUES, argE is scaled by the ratio of the perimeter of the inscribed shape
	// GCODE            over the circumference of the defined circle.
	// GCODE
	// GCODE    if argR == 1,
	// GCODE        spiral out from an "inward" point (start with an initial jump from the current position to an "inward" position)
	// GOCDE        and spiral out from there to the current position
	// GCODE    if argR == 2,
	// GCODE        spiral out from the current position and then ending at a position "outward" of the current position
	// GCODE    else
	// GCODE        spiral in from current position
	// GCODE
	// GCODE    traverse a CW/CCW arc following the LHR relative to the direction of the normal
	// GCODE        {for reference, G17 sets PLANE_XY with a normal of [0,0,1] such that the clockwise
	// GCODE        arc is traversed on the print bed when looking down on the bed}
	// GCODE
	// GCODE NOTE: end position for non-reverse is at end of spiral, NOT back to origin (behavior change with 4.000i)

	arcArgS = ARG_S;
	ARG_S = INVALID_ARG_VALUE;  // ARG_S is used as a one time flow scale on printing moves

	ARG_A = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3
	ARG_B = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3
	ARG_C = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3

	boolean jumpInwardToStartReverseSpiral = FALSE;
	_reverseFrogToe = FALSE;

	if (ARG_R_PRESENT)
	{
		if (ARG_R == 1.0f)
		{
			_reverseFrogToe = TRUE;
			jumpInwardToStartReverseSpiral = TRUE;

		}
		if (ARG_R == 2.0f)
		{
			_reverseFrogToe = TRUE;
			jumpInwardToStartReverseSpiral = FALSE;
		}
		ARG_R = INVALID_ARG_VALUE;
	}

	G2G3Ptr=ExecutionPtr; //save the index in the que so we can come back to this.
	Radius=0;
	theta=0;
	XRadvalue=0;
	yRADvalue=0;
	ArcSubcounter=0;

	if (ARG_X_PRESENT || ARG_Y_PRESENT)
	{   // for now, don't allow specing spiral endpoint
		sprintf(_errorStr, "Illegal X/Y on G%2.1f", ARG_G );
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}
	if (ARG_P_MISSING)   { ReportMissingGcodePArg();  return; }

	// fill in any missing arguments with values consistent with the current position of the axis
	// and save the end position of the move

	if (_IncrementalMove)
	{
		if (ARG_X_MISSING) ARG_X = 0.0f;
		if (ARG_Y_MISSING) ARG_Y = 0.0f;
		if (ARG_Z_MISSING) ARG_Z = 0.0f;

		LastG2X = Motors[M_X].Q_LastRequestedPositionInUnits + ARG_X;
		LastG2Y = Motors[M_Y].Q_LastRequestedPositionInUnits + ARG_Y;
		LastG2Z = Motors[M_Z].Q_LastRequestedPositionInUnits + ARG_Z;
	}
	else
	{
		if (ARG_X_MISSING) ARG_X = Motors[M_X].Q_LastRequestedPositionInUnits;  //x target is the same as the beginning if there is no new target x
		if (ARG_Y_MISSING) ARG_Y = Motors[M_Y].Q_LastRequestedPositionInUnits;
		if (ARG_Z_MISSING) ARG_Z = Motors[M_Z].Q_LastRequestedPositionInUnits;

		LastG2X = ARG_X;
		LastG2Y = ARG_Y;
		LastG2Z = ARG_Z;
	}
	// LastG2X/LastG2Y/LastG2Z are now absolute position (remaining G2/G3 will use absolute coords


	if (ARG_I_MISSING)ARG_I = 0.0f;//no argument means it has zero offset from axis
	if (ARG_J_MISSING)ARG_J = 0.0f;

	if ((ARG_I == 0.0f) && (ARG_J == 0.0f))
	{
		sprintf(_errorStr, "INVALID I J--> I=%4.3f J=%4.3f for G%2.1f", ARG_I,ARG_J, ARG_G);
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}

	Radius = fpu_sqrtf(sqr(ARG_I) + sqr(ARG_J));
	ArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + ARG_I;//get the center of the arc
	ArcCenterY=Motors[M_Y].Q_LastRequestedPositionInUnits + ARG_J;//get the center of the arc

	_frogToePasses = 0;

	if (_reverseFrogToe && !jumpInwardToStartReverseSpiral)
	{   // radius is irrelevant when reversing outward from current position
		if (ARG_L_MISSING)  { ReportMissingGcodeLArg();  return; }
		_frogToePasses = imax(0, (int)ARG_L);
	}
	else if (ARG_L_MISSING)
	{
		if (ARG_P > 0.0f)   // would otherwise divide by 0 or worse
			_frogToePasses = (int)(Radius / ARG_P);
		else
			_frogToePasses = 0;
	}
	else
	{   // argL present
		_frogToePasses = (int)ARG_L;
		if ((_frogToePasses == 0.0f) || ((ARG_P * _frogToePasses) > Radius))
		{
			if (ARG_P > 0.0f)   // would otherwise divide by 0 or worse
				_frogToePasses = (int)(Radius / ARG_P);
			else
				_frogToePasses = 0;
		}
	}

	if (_frogToePasses == 0)
	{
		sprintf(_errorStr, "No spiral detected: check L(%d), P(%3.1f) and effective radius (%3.1f) in G%2.1f", (int)ARG_L, ARG_P, Radius, ARG_G);
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}

	ARG_L = INVALID_ARG_VALUE;  // nullify -- no longer needed

	G2DeltaZ = (LastG2Z - Motors[M_Z].Q_LastRequestedPositionInUnits) / (float)_frogToePasses; // change of Z per lap

	theta = atan2f(ARG_J * -1.0f, ARG_I * -1.0f);//get the angle in radians
	StartAngleInDegrees = theta * RADIANS_TO_DEGREES;//now we have the start angle in degrees;
	if (StartAngleInDegrees < 0)
		StartAngleInDegrees += 360;
	WorkAngleInRadians = (float)StartAngleInDegrees * DEGREES_TO_RADIANS;

	if (_reverseFrogToe)
	{
		if (jumpInwardToStartReverseSpiral)
		{   // match original G2.1 reversing behavior
			_save_IncrementalMove = _IncrementalMove;  // save initial state
			_IncrementalMove = FALSE; // force absolute
			Radius = Radius - (_frogToePasses * ARG_P);

			XRadvalue=Radius * cosf(WorkAngleInRadians);
			yRADvalue=Radius * sinf(WorkAngleInRadians);
			ARG_Y = (float)(ArcCenterY + yRADvalue);
			ARG_X = (float)(ArcCenterX + XRadvalue);
			ARG_Z = LastG2Z;        // reverse start and end Z
			LastG2Z = Motors[M_Z].Q_LastRequestedPositionInUnits;
			G2DeltaZ *= -1.0f;
			float saveE = ARG_E;
			ARG_E = INVALID_ARG_VALUE;
			motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
			ARG_E = saveE;  //restore
			_IncrementalMove = _save_IncrementalMove;  // restore state to first move
		}
		else
		{
			float finalRadius = Radius + (_frogToePasses * ARG_P);
			XRadvalue=finalRadius * cosf(WorkAngleInRadians);
			yRADvalue=finalRadius * sinf(WorkAngleInRadians);
			LastG2X = (float)(ArcCenterX + XRadvalue);  // lie so it stops on the inside of the spiral instead of a return to the outside
			LastG2Y = (float)(ArcCenterY + yRADvalue);
		}
	}
	else
	{   // find final frog toe position for last move
		float finalRadius = Radius - (_frogToePasses * ARG_P);
		XRadvalue=finalRadius * cosf(WorkAngleInRadians);
		yRADvalue=finalRadius * sinf(WorkAngleInRadians);
		LastG2X = (float)(ArcCenterX + XRadvalue);  // lie so it stops on the inside of the spiral instead of a return to the outside
		LastG2Y = (float)(ArcCenterY + yRADvalue);
	}

	_saveExtrusionControl = _extrusionControl;  // save current state so more can be forced to auto calc flow
	_extrusionControl = IGNORE_E_VALUES;
	G2G3Flag = 0;
	G203State = 6;  // frogToeCircle mode
}

////////////////////////////////////////////////////////////////////////////////

void frogToeCircle(void)
{
	LastG2Z += G2DeltaZ;

	if (_frogToePasses > 0)
	{   // real work to do
		//start angle is the ratio of DX/DY from ending point to the circlecenter
#if 1
		if ((ARG_J == 0.0f) && (ARG_I == 0.0f))	//starting at center
			theta = atan2f((LastG2Y-ArcCenterY), (LastG2X-ArcCenterX));//get the angle in radians
		else
			theta = atan2f(ARG_J * -1.0f, ARG_I * -1.0f);//get the angle in radians
#else
		theta = atan2f(ARG_J * -1.0f, ARG_I * -1.0f);//get the angle in radians
#endif
		StartAngleInDegrees = theta * RADIANS_TO_DEGREES;//now we have the start angle in degrees;
		if (StartAngleInDegrees < 0)
			StartAngleInDegrees += 360;

		//now calculate the end angle
		EndAngleInDegrees=StartAngleInDegrees;

		SweepAngle = ((int)ARG_G==2) ? -360.0f : 360.0f;//circle so we are sweeping 360 degrees
		_deltaRForPass = -ARG_P;

		if (_reverseFrogToe)
		{
			SweepAngle *= -1.0f;
			_deltaRForPass *= -1.0f;
		}
		_spiralRoughingPass = TRUE;
		if (_frogToePasses == 1)
		{
			G203State = 0;
		}
		else
		{
			G203State = 6;
		}

		setupArcMove(StartAngleInDegrees, SweepAngle, Radius, arcArgS);
		_frogToePasses--;
	}
}

////////////////////////////////////////////////////////////////////////////////

void ProcessG2G3Command(int G2_G3)  // circular/helical arc in XY plane w/ Z displacement (uses X,Y,Z,I,J,S,E, F)
{
	// GCODE G2.0/G3.0 [XYZ arcDest] [IJ ofs2center] [E amount] [S segLength/#segments] [F feedrate]
	// GCODE
	// GCODE    the current XYZ position is starting point of the arc
	// GCODE
	// GCODE    the specified XYZ position is the final destination of the arc  {the current position is used
	// GCODE        to fill in any missing XYZ fields}
	// GCODE
	// GCODE    ABC axes will not move
	// GCODE
	// GCODE    the specified IJ are the relative offset from current XY to the arc center/origin {'0.0' is
	// GCODE        used to fill in any missing IJ fields}
	// GCODE
	// GCODE    if argF specified, then use the newly specified feedrate
	// GCODE
	// GCODE    if argE specified and USING_E_VALUES (M229 E1) then the amount of material specified with argE
	// GCODE        will be uniformly distributed across the entire arc
	// GCODE
	// GCODE    if argS specified, then controls the number of segments in the arc as follows
	// GCODE        argS < 3, then argS is the segment length (# segments = argLength / argS)
	// GCODE        arcs >= 3, the argS is the number of sides for a full circular arc (ie, argS=4 will make a square)
	// GCODE            if argE and USING_E_VALUES, argE is scaled by the ratio of the perimeter of the inscribed shape
	// GCODE            over the circumference of the defined circle.
	// GCODE
	// GCODE    traverse a CW/CCW arc in the XY plane, with Z displacement
	// GCODE        arc is traversed on the print bed when looking down on the bed}

	arcArgS = ARG_S;
	ARG_S = INVALID_ARG_VALUE;  // ARG_S is used as a one time flow scale on printing moves

	ARG_A = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3
	ARG_B = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3
	ARG_C = INVALID_ARG_VALUE;  // illegal to move ABC during G2/G3

	G2G3Ptr=ExecutionPtr; //save the index in the que so we can come back to this.
	Radius=0;
	theta=0;
	XRadvalue=0;
	yRADvalue=0;
	ArcSubcounter=0;

	// fill in any missing arguments with values consistent with the current position of the axis
	// and save the end position of the move
	if (_IncrementalMove)
	{
		if (ARG_X_MISSING) ARG_X = 0.0f;
		if (ARG_Y_MISSING) ARG_Y = 0.0f;
		if (ARG_Z_MISSING) ARG_Z = 0.0f;

		LastG2X = Motors[M_X].Q_LastRequestedPositionInUnits + ARG_X;
		LastG2Y = Motors[M_Y].Q_LastRequestedPositionInUnits + ARG_Y;
		LastG2Z = Motors[M_Z].Q_LastRequestedPositionInUnits + ARG_Z;
	}
	else
	{
		if (ARG_X_MISSING) ARG_X = Motors[M_X].Q_LastRequestedPositionInUnits;  //x target is the same as the beginning if there is no new target x
		if (ARG_Y_MISSING) ARG_Y = Motors[M_Y].Q_LastRequestedPositionInUnits;
		if (ARG_Z_MISSING) ARG_Z = Motors[M_Z].Q_LastRequestedPositionInUnits;

		LastG2X = ARG_X;
		LastG2Y = ARG_Y;
		LastG2Z = ARG_Z;
	}
	// LastG2X/LastG2Y/LastG2Z are now absolute position (remaining G2/G3 will use absolute coords
	G2DeltaZ = LastG2Z - Motors[M_Z].Q_LastRequestedPositionInUnits;

	if (ARG_I_MISSING)ARG_I = 0.0f;//no argument means it has zero offset from axis
	if (ARG_J_MISSING)ARG_J = 0.0f;

	if ((ARG_I == 0.0f) && (ARG_J == 0.0f))
	{
		sprintf(_errorStr, "INVALID ARC: I J--> I=%4.3f J=%4.3f", ARG_I, ARG_J );
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}

	Radius = fpu_sqrtf(sqr(ARG_I) + sqr(ARG_J));
	ArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + ARG_I;//get the center of the arc
	ArcCenterY=Motors[M_Y].Q_LastRequestedPositionInUnits + ARG_J;//get the center of the arc

	// by spec, the center must be equidistant from the start and end points (error otherwise)
	float CheckRadius = fpu_sqrtf(sqr(ArcCenterX-LastG2X) + sqr(ArcCenterY-LastG2Y));
	if (fabsf(CheckRadius - Radius) > ARC_TOLERANCE)
	{
		sprintf(_errorStr, "INVALID ARC: center is not equidistant from start and end points: X=%4.3f Y=%4.3f I=%4.3f J=%4.3f",
				LastG2X, LastG2Y, ARG_I, ARG_J);
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}

	//start angle is the ratio of DX/DY from circlecenter to starting point,
	//this is actually the I/J
	//now we need to do a simple test to see if we are using 0,90,180,270 as a start angle
	theta = atan2f(ARG_J * -1.0f, ARG_I * -1.0f);//get the angle in radians
	StartAngleInDegrees = theta * RADIANS_TO_DEGREES;//now we have the start angle in degrees;
	if (StartAngleInDegrees < 0)
		StartAngleInDegrees += 360;

	//now calculate the end angle
	if ((Motors[M_X].Q_LastRequestedPositionInUnits == ARG_X) && (Motors[M_Y].Q_LastRequestedPositionInUnits == LastG2Y))
	{   //this is a CIRCLE, so the end angle is same as the start angle
		EndAngleInDegrees=StartAngleInDegrees;
		SweepAngle= (G2_G3==2) ? -360.0f : 360.0f;//circle so we are sweeping 360 degrees
		if (ARG_P_PRESENT)
		{
			float fLaps = fabsf(G2DeltaZ / ARG_P); // ->> needs to be an integer
			int iLaps = (int)fLaps;
			SweepAngle *= (float)(iLaps);
			if (arcArgS >= 3.0f)
			{
				arcArgS *= (float)iLaps;
			}
			if (fabsf((fLaps - (float)iLaps)) > G2G3_THREAD_PITCH_TOLERANCE)
			{
				sprintf(_errorStr, "INVALID G2/G3: Pitch-Z mismatch totalZ=%4.3f pitch=%4.3f rotations=%4.3f",
						G2DeltaZ, ARG_P, fLaps);
				catastrophicError(_errorStr);
				return;//exit with out moving, as there is NO Valid OPTION
			}
		}
	}
	else
	{   //calculate the final angle from the final x,y and circleCenter
		EndAngleDeltaX = ARG_X- ArcCenterX;
		EndAngleDeltaY = ARG_Y- ArcCenterY;
		theta = atan2f(EndAngleDeltaY, EndAngleDeltaX);//get the angle in radians
		EndAngleInDegrees = theta * RADIANS_TO_DEGREES;//now we have the start angle in degrees;

		if (G2_G3 == 2)
		{
			SweepAngle = EndAngleInDegrees - StartAngleInDegrees;
			if (SweepAngle > 0.0f)
				SweepAngle -= 360.0f; // StartAngleInDegrees-EndAngleInDegrees;
		}
		else if (G2_G3 == 3)
		{
			SweepAngle = EndAngleInDegrees - StartAngleInDegrees;
			if (SweepAngle < 0.0f)
				SweepAngle += 360.0f;//  StartAngleInDegrees -EndAngleInDegrees ;
		}
		SweepAngle = fmodf(SweepAngle,360.0f);
		if (arcArgS >= 3.0f)
		{
			arcArgS = INVALID_ARG_VALUE;    // can't force #sides on non-full-circular arcs
		}
	}

	//now lets figure out how many arc segments we need to make a smooth circle

	setupArcMove(StartAngleInDegrees, SweepAngle, Radius, arcArgS);
	ExecuteG2G3PointMove();
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G2(void)    // G2 master command // traverse a clockwise arc
{
	// GCODE G2 - CCW arc

	int cmd = (int)ARG_G;
	switch (getArgFraction(ARG_G))
	{
	case 0: ProcessG2G3Command(cmd); break;
	case 1: ProcessG2_1G3_1Command(cmd); break;
	case 2: ProcessG2_2G3_2Command(cmd); break;
	case 3: ProcessG2_3G3_3Command(cmd); break;
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G3(void)  // G2 master command // traverse a counter clockwise arc
{
	// GCODE G3 - CCW arc

	int cmd = (int)ARG_G;
	switch (getArgFraction(ARG_G))
	{
	case 0: ProcessG2G3Command(cmd); break;
	case 1: ProcessG2_1G3_1Command(cmd); break;
	case 2: ProcessG2_2G3_2Command(cmd); break;
	case 3: ProcessG2_3G3_3Command(cmd); break;
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G4(void)    // Set dwell time (uses PS)
{
	// GCODE G4 [S seconds] [P milliseconds
	// GCODE
	// GCODE    P and S are additive if both present
	// GCODE
	// GCODE    Set dwell time

	_g4DwellTimer = 0;

	if (ARG_P_PRESENT)
	{
		_g4DwellTimer = (uint32_t) ARG_P;   // in milliseconds
	}

	if (ARG_S_PRESENT)
	{
		_g4DwellTimer += (uint32_t) (ARG_S * 1000.0f);   // in seconds
	}

	_g4DwellTimer = iFitWithinRange(_g4DwellTimer, 0, DELAY_2_HOURS_OF_MILLISECONDS);
}

////////////////////////////////////////////////////////////////////////////////

void setUpBoringCycle()
{
	G2G3Ptr=ExecutionPtr;//save the index in the que so we can come back to this.
	G203State=3;
	Radius=0;
	theta=0;
	XRadvalue=0.0f;
	yRADvalue=0.0f;
	ArcSubcounter=0;
	if (ARG_D_PRESENT)CurrentToolDiameter=ARG_D;//update working diameter of the tool
	_spiralRoughingPass = ARG_P_PRESENT;//set up for spiral pocket bore, removes ALL material
	spiralBoreZStartPosition=Motors[M_Z].Q_LastRequestedPositionInUnits;//figure out where to return to when donw
	spiralBorePitch=0.0f;
	spiralBoreNumberOfSpirals=0;//used to repeat the last bore cycle with z increment
	spiralBoreDeltaZ=0.0f;
	SpiralBoreZIncrement=0.0f;
	spiralBoreNumberOfArcSegments=0;
	spiralBoreArcSegmentsPerRevolution=300;
	//now store the final xy for the last move in the arc command
	LastG2X=Motors[M_X].Q_LastRequestedPositionInUnits;
	LastG2Y=Motors[M_Y].Q_LastRequestedPositionInUnits;
	LastG2Z=Motors[M_Z].Q_LastRequestedPositionInUnits;
	G2DeltaZ=0.0f;
	StartAngleInDegrees = 180.0f;//180.0f;//(ARG_A_PRESENT) ? ARG_A : 180.0f;//half arc
}

////////////////////////////////////////////////////////////////////////////////

//bores a hole with cutter compensation to make the finished size = J
//if I is included, it is the rough diameter before the final J cut
//if J-I  is smaller than the diameter of the cutter, the cutter becomes the step size for making a pocket mill

//  ARG_C can be present force use of ARG_E thoughout an intended milling cut (really to use inkjet  on moves)
//  without ARG_C, an ARG_E indicates printing over milling in which case the final tapered spiral and final "cut" are dropped
//
//it is possible to invoke a spiral bore where the z is constantly moving down as the spiral is being executed,
//one complete revolution will move the Z down the Pitch which is ARGP
//
//
//HALF ARC IN TO I
//FULL ARC TO I
//JOG TO J
//FULL ARC TO J
//HALF ARC TO CENTER
//or
//archimedies spiral to J, leave off the I, P defines the amount each circle will shrink
//Does 360 deg circle at J diameter, with cutter comp, then half arc back to original center

void G_Code_G12(void)
{//circular pocket mill CW
	//this will include a two pass bore, rough which is defined by I and finish which is defined by J
	if (_sendingGBStringsMask & GB_STRING_CMD_QUE) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr, "G%d starting", (int)ARG_G); sendGB(_tmpStr);
	}
	arcArgS = ARG_S;
	ARG_S = INVALID_ARG_VALUE;  // ARG_S is used as a one time flow scale on printing moves
	if (ARG_J_MISSING)
	{
		G203State=0;
		sprintf(_errorStr, "G12 INVALID J--> J%f", ARG_J );
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}
	G2G3Ptr=ExecutionPtr;//save the index in the que so we can come back to this.
	setUpBoringCycle();
	CW_CCW_Modifier=-1.0f;//clockwise decreases the angle as it moves
	SweepAngle=-180.0f;


	if (ARG_I_MISSING)
	{//simple bore cycle, no roughing
		//lead in arc
		//360 deg arc
		//lead out arc
		ARG_I = ARG_J;//if no I argument it means that here is NO rough cutting before final pass
		//OR it is a archimedeas spiral rough cut

		G203State=2;//If so this is a one pass bore cycle.
		QuarterRadius = (ARG_J - CurrentToolDiameter)/4.0f;
		if(QuarterRadius<0)
		{
			G203State=0;
			sprintf(_errorStr, "G12 INVALID Diameter too small--> J%f", ARG_J );
			catastrophicError(_errorStr);
			return;//exit with out moving, as there is NO Valid OPTION
		}
		Radius=QuarterRadius;
		LeadOutArcCenterX=LeadInArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + QuarterRadius;   //get the center oftheho
		LeadInArcCenterY=Motors[M_Y].Q_LastRequestedPositionInUnits;
		ArcCenterX=LeadInArcCenterX;   //get the center of the lead in arc
		ArcCenterY=LeadInArcCenterY;      //get the center of the half arc
		WorkingDiameter=ARG_J-CurrentToolDiameter;
		if (_spiralRoughingPass)
		{//now we have frog toe mode
			SpiralPasses=(int)(WorkingDiameter/2.0f)/ARG_P;//number of spiral laps
			SpiralSegments=36*SpiralPasses;//20 segments per circle time number of passes
			SpiralRadiusIncrement=(WorkingDiameter/2.0f)/SpiralSegments;
			ArcCenterX=LastG2X; //  center of rotation used for point calcualtions
			ArcCenterY=LastG2Y; //  center of rotation used for point calcualtions
			StartAngleInDegrees =0;
			G2G3Flag=SpiralSegments-1;
			AngleIncrement=-10;//now we have the increment.
			CurrentAngle=StartAngleInDegrees+AngleIncrement;//POINT TO THE VERY FIRST POINT IN THE ARC FROM HERE.
			Radius=0;
			ExecuteG2G3PointMove();
			return;
		}
		else
		{SpiralPocket=0;
		}
		setupArcMove(StartAngleInDegrees,SweepAngle,QuarterRadius, INVALID_ARG_VALUE);
		ExecuteG2G3PointMove();
		return;//now it is started
	}
	//at this point we are going to do a rough then final pass

	if(ARG_I > ARG_J)ARG_I = ARG_J;//dont start too big
	WorkingDiameter=ARG_I;

	//now calculate the lead out arc center
	QuarterRadius = (ARG_J - CurrentToolDiameter)/4.0f;
	LeadOutArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + QuarterRadius;
	//now calculate the lead IN arc center
	QuarterRadius = (ARG_I - CurrentToolDiameter)/4.0f;   //effective working radius for leadin ;
	if(QuarterRadius<0)
	{
		G203State=0;
		sprintf(_errorStr, "G12 INVALID Diameter too small--> J%f", ARG_J );
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}
	Radius=QuarterRadius;//set up for the short arc first
	LeadInArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + QuarterRadius;   //get the center oftheho
	LeadInArcCenterY=Motors[M_Y].Q_LastRequestedPositionInUnits;//same Y for all operations


	ArcCenterX=LeadInArcCenterX;   //get the center of the ho
	ArcCenterY=LeadInArcCenterY;      //get the center of the starting half arc


	//    if(ARG_Z_PRESENT)
	//    {//it is possible we are having a delta Z now
	//      if (Motors[M_Z].Q_LastRequestedPositionInUnits!=ARG_Z)
	//      {
	//          spiralBoreZEndPosition=ARG_Z;//save this for later please
	//          if(ARG_P_MISSING)ARG_P=1.0f;//set the default to 1mm per revolution
	//          spiralBoreDeltaZ = ARG_Z-Motors[M_Z].Q_LastRequestedPositionInUnits;//get the delta move please
	//          spiralBoreNumberOfSpirals=(int)(spiralBoreDeltaZ/ARG_P);
	//          if(spiralBoreNumberOfSpirals==0)spiralBoreNumberOfSpirals=1;//at least one trip around the park please
	//          NumberOfArcSegments = imax(1, (int)roundf(Radius * fabsf(SweepAngle) * TWO_PI_OVER_360 / ARC_LENGTH_PER_SEGMENT));
	//          spiralBoreNumberOfArcSegments=spiralBoreNumberOfSpirals*NumberOfArcSegments;
	//          SpiralBoreZIncrement= spiralBoreDeltaZ/spiralBoreNumberOfArcSegments;
	//          G203State=10;//initial arc, no g2 delta z stuff going on, this is only the lead in please
	//          setupArcMove(StartAngleInDegrees,SweepAngle,QuarterRadius, INVALID_ARG_VALUE);//start the lead in arc
	//          ExecuteG2G3PointMove();
	//          //StartAngleInDegrees -= 180.0f; // be ready for additional roughing and final pass
	//      }
	//    }
	//    else
	{//normal operation, not spiraling down
		setupArcMove(StartAngleInDegrees,SweepAngle,QuarterRadius, INVALID_ARG_VALUE);
		ExecuteG2G3PointMove();
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G13(void)
{//circular pocket mill CCW
	//this will include a two pass bore, rough which is defined by I and finish which is defined by J
	if (_sendingGBStringsMask & GB_STRING_CMD_QUE) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr, "G%d starting", (int)ARG_G); sendGB(_tmpStr);
	}
	arcArgS = ARG_S;
	ARG_S = INVALID_ARG_VALUE;  // ARG_S is used as a one time flow scale on printing moves
	if (ARG_J_MISSING)
	{
		G203State=0;
		sprintf(_errorStr, "G13 INVALID J--> J%f", ARG_J );
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}
	G2G3Ptr=ExecutionPtr;//save the index in the que so we can come back to this.
	setUpBoringCycle(); //sets G203State=3
	CW_CCW_Modifier=1.0f;//clockwise decreases the angle as it moves
	SweepAngle=180.0f;


	if (ARG_I_MISSING)
	{//simple bore cycle, no roughing
		//lead in arc
		//360 deg arc
		//lead out arc
		ARG_I = ARG_J;//if no I argument it means that here is NO rough cutting before final pass
		//OR it is a archimedeas spiral rough cut

		G203State=2;//If so this is a one pass bore cycle.
		QuarterRadius = (ARG_J - CurrentToolDiameter)/4.0f;
		if(QuarterRadius<0)
		{
			G203State=0;
			sprintf(_errorStr, "G13 INVALID Diameter too small--> J%f", ARG_J );
			catastrophicError(_errorStr);
			return;//exit with out moving, as there is NO Valid OPTION
		}
		Radius=QuarterRadius;
		LeadOutArcCenterX=LeadInArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + QuarterRadius;   //get the center oftheho
		LeadInArcCenterY=Motors[M_Y].Q_LastRequestedPositionInUnits;
		ArcCenterX=LeadInArcCenterX;   //get the center of the lead in arc
		ArcCenterY=LeadInArcCenterY;      //get the center of the half arc
		WorkingDiameter=ARG_J-CurrentToolDiameter;
		if (_spiralRoughingPass)
		{//now we have frog toe mode
			SpiralPasses=(int)(WorkingDiameter/2.0f)/ARG_P;//number of spiral laps
			SpiralSegments=36*SpiralPasses;//20 segments per circle time number of passes
			SpiralRadiusIncrement=(WorkingDiameter/2.0f)/SpiralSegments;
			ArcCenterX=LastG2X; //  center of rotation used for point calcualtions
			ArcCenterY=LastG2Y; //  center of rotation used for point calcualtions
			StartAngleInDegrees =0;
			G2G3Flag=SpiralSegments-1;
			AngleIncrement=10;//now we have the increment.
			CurrentAngle=StartAngleInDegrees+AngleIncrement;//POINT TO THE VERY FIRST POINT IN THE ARC FROM HERE.
			Radius=0;
			ExecuteG2G3PointMove();
			return;
		}
		else
		{SpiralPocket=0;
		}
		setupArcMove(StartAngleInDegrees,SweepAngle,QuarterRadius, INVALID_ARG_VALUE);
		ExecuteG2G3PointMove();
		return;//now it is started
	}
	//at this point we are going to do a rough then final pass

	if(ARG_I > ARG_J)ARG_I = ARG_J;//dont start too big
	WorkingDiameter=ARG_I;

	//now calculate the lead out arc center
	QuarterRadius = (ARG_J - CurrentToolDiameter)/4.0f;
	LeadOutArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + QuarterRadius;
	//now calculate the lead IN arc center
	QuarterRadius = (ARG_I - CurrentToolDiameter)/4.0f;   //effective working radius for leadin ;
	if(QuarterRadius<0)
	{
		G203State=0;
		sprintf(_errorStr, "G13 INVALID Diameter too small--> J%f", ARG_J );
		catastrophicError(_errorStr);
		return;//exit with out moving, as there is NO Valid OPTION
	}
	Radius=QuarterRadius;//set up for the short arc first
	LeadInArcCenterX=Motors[M_X].Q_LastRequestedPositionInUnits + QuarterRadius;   //get the center oftheho
	LeadInArcCenterY=Motors[M_Y].Q_LastRequestedPositionInUnits;//same Y for all operations


	ArcCenterX=LeadInArcCenterX;   //get the center of the ho
	ArcCenterY=LeadInArcCenterY;      //get the center of the starting half arc


	//    if(ARG_Z_PRESENT)
	//    {//it is possible we are having a delta Z now
	//      if (Motors[M_Z].Q_LastRequestedPositionInUnits!=ARG_Z)
	//      {
	//          spiralBoreZEndPosition=ARG_Z;//save this for later please
	//          if(ARG_P_MISSING)ARG_P=1.0f;//set the default to 1mm per revolution
	//          spiralBoreDeltaZ = ARG_Z-Motors[M_Z].Q_LastRequestedPositionInUnits;//get the delta move please
	//          spiralBoreNumberOfSpirals=(int)(spiralBoreDeltaZ/ARG_P);
	//          if(spiralBoreNumberOfSpirals==0)spiralBoreNumberOfSpirals=1;//at least one trip around the park please
	//          NumberOfArcSegments = imax(1, (int)roundf(Radius * fabsf(SweepAngle) * TWO_PI_OVER_360 / ARC_LENGTH_PER_SEGMENT));
	//          spiralBoreNumberOfArcSegments=spiralBoreNumberOfSpirals*NumberOfArcSegments;
	//          SpiralBoreZIncrement= spiralBoreDeltaZ/spiralBoreNumberOfArcSegments;
	//          G203State=10;//initial arc, no g2 delta z stuff going on, this is only the lead in please
	//          setupArcMove(StartAngleInDegrees,SweepAngle,QuarterRadius, INVALID_ARG_VALUE);//start the lead in arc
	//          ExecuteG2G3PointMove();
	//          //StartAngleInDegrees -= 180.0f; // be ready for additional roughing and final pass
	//      }
	//    }
	//    else
	{//normal operation, not spiraling down
		setupArcMove(StartAngleInDegrees,SweepAngle,QuarterRadius, INVALID_ARG_VALUE);
		ExecuteG2G3PointMove();
	}
}

////////////////////////////////////////////////////////////////////////////////

void setupArcMove(float StartAngleInDegrees, float SweepAngle, float Radius, float Segments)
{
	float radiusForSegmentCalc = (_deltaRForPass > 0.0f) ? Radius+_deltaRForPass : Radius;	// _deltaRForPass>0.0f reverse spiral, so use ending radius for calc

	if ((Segments != INVALID_ARG_VALUE) && (Segments > 0.0f))
	{
		if (Segments >= 3.0f)
		{   // let user define the number of segments -- FOR COMPLETE CIRCLES only
			NumberOfArcSegments = (int)Segments;
		}
		else
		{   // user define segment length
			NumberOfArcSegments = imax(1, (int)roundf(radiusForSegmentCalc * fabsf(SweepAngle) * TWO_PI_OVER_360 / Segments));
		}
	}
	else
	{
		NumberOfArcSegments = imax(1, (int)roundf(radiusForSegmentCalc * fabsf(SweepAngle) * TWO_PI_OVER_360 / ARC_LENGTH_PER_SEGMENT));
	}

	AngleIncrement=SweepAngle/(float)NumberOfArcSegments;//now we have the increment.
	CurrentAngle=StartAngleInDegrees+AngleIncrement;//POINT TO THE VERY FIRST POINT IN THE ARC FROM HERE.
	G2G3Flag=NumberOfArcSegments; // +1; // +1 to cover sins ...
	_save_IncrementalMove = _IncrementalMove;  // will be restored after the arc completes
	_IncrementalMove = FALSE;  // force absolute mode for XYZ to make the arc work

	// need to handle the E value if its value is being used.
	_save_IncrementalEMove = _IncrementalEMove;  // will be restored after the arc completes (always save, because it's always restored at end of arc
	_IncrementalEMove = TRUE;  // force incremental mode to make the arc work
	if ((ARG_E_PRESENT) && (_extrusionControl == USE_E_VALUES))  //using E values
	{   // E values matter, so adjust value to be correct per segment
		float scaleEForInscribedShape = 1.0f;
		if (((Segments != INVALID_ARG_VALUE) && (Segments >= 3.0f)) && (((SweepAngle <= -360.f) || (SweepAngle >= 360.f))))
		{   // -- FOR COMPLETE CIRCLES only (or muliple circles -- adjust flow if needed for inscribed shapes
			// mult E by ratio of (NumSeg * segLen) / (2PiR);   where segLen = 2Rsin(Pi/NumSeg) {perim of N-sided shape / perim of circle}
			// simplify to  ((NumSeg * 2Rsin(Pi/NumSeg)) / (2PiR) ==>>>   ((NumSeg * sin(Pi/NumSeg)) / (Pi)
			float segmentsPerLap = Segments / (fabsf(SweepAngle) / 360.0f);
			scaleEForInscribedShape = (segmentsPerLap * sin(PI / segmentsPerLap)) / PI;
		}
		ARG_E = (getDeltaEValueInInputUnits() * scaleEForInscribedShape) / (float)NumberOfArcSegments; // divide by num segments to get per seg amount
	}
}

////////////////////////////////////////////////////////////////////////////////

void StartFinalReturnToCenterArc(void)
{//final arc back to home
	if (ARG_C_MISSING)ARG_E = INVALID_ARG_VALUE; // never print on the return
	Radius = (ARG_J - CurrentToolDiameter) / 4.0f;//Working radius with cutter comensation;
	ArcCenterX=LeadOutArcCenterX;// LastG2X + (Radius * CW_CCW_Modifier); //  center of rotation used for point calcualtions
	ArcCenterY=LeadInArcCenterY; //  center of rotation used for point calcualtions
	SweepAngle = CW_CCW_Modifier * 180.0f;
	StartAngleInDegrees=0;
	setupArcMove(StartAngleInDegrees,SweepAngle,Radius,INVALID_ARG_VALUE);
}

////////////////////////////////////////////////////////////////////////////////

void StartMainFinishCircle(void)
{//main arc to make large bore circle
	Radius = (ARG_J - CurrentToolDiameter)/2.0f;//Working radius with cutter comensation;
	ArcCenterX=LastG2X; //  center of rotation used for point calcualtions
	ArcCenterY=LastG2Y; //  center of rotation used for point calcualtions
	SweepAngle = 360 * CW_CCW_Modifier;
	StartAngleInDegrees=0;
	_spiralRoughingPass=FALSE;
	setupArcMove(StartAngleInDegrees,SweepAngle,Radius,arcArgS);
}

void StartMainFinishCircleSpiral(void)
{//main arc to make large bore circle
#ifdef GB_HIDDEN_WARNINGS
	int spiralBoreNumberOfArcSegments___IS_NEVER_SET___DIVIDE_BY_ZERO; // the variable spiralBoreNumberOfArcSegments is not set to a non zero value
#endif //GB_HIDDEN_WARNINGS
	return;

	Radius = (ARG_J-CurrentToolDiameter)/2;//Working radius with cutter comensation;
	ArcCenterX=LastG2X; //  center of rotation used for point calcualtions
	ArcCenterY=LastG2Y; //  center of rotation used for point calcualtions

	SweepAngle = spiralBoreNumberOfSpirals*-360.0f * CW_CCW_Modifier;
	G2DeltaZ=SpiralBoreZIncrement;
	G2G3Flag=spiralBoreNumberOfArcSegments;
	StartAngleInDegrees+=180.0f;
	AngleIncrement=SweepAngle/(float)spiralBoreNumberOfArcSegments;//now we have the increment.
	CurrentAngle=StartAngleInDegrees+AngleIncrement;//POINT TO THE VERY FIRST POINT IN THE ARC FROM HERE.
	NumberOfArcSegments=spiralBoreNumberOfArcSegments;
	G2G3Flag=NumberOfArcSegments;
}
////////////////////////////////////////////////////////////////////////////////

void startMainRoughCircle(void)
{//main arc-clearance for nice clean bore cycle
	Radius = (WorkingDiameter-CurrentToolDiameter)/2.0f;//Working radius with cutter compensation;
	ArcCenterX=LastG2X; //  center of rotation used for point calcualtions
	ArcCenterY=LastG2Y; //  center of rotation used for point calcualtions
	SweepAngle = 360.0f * CW_CCW_Modifier;//which way are we going cw or ccw
   // StartAngleInDegrees + (180*CW_CCW_Modifier);//adjust the start angle based on cw or ccw

	// 3 cases to worry about:
	//   milling and not spiral -- same as original code
	//   milling and spiral     -- spiral out to final diameter, then do final pass
	//   printing and spiral    -- uniformly spiral out, but stop before deltaRforPass changes (otherwise overlap plastic)

	// NUKE - kjg dropped _spiralEnabled
//    if (_spiralEnabled)
//    {
//      if (((WorkingDiameter + ARG_I) >= ARG_J) && (ARG_E_MISSING || ARG_C_PRESENT))
//      {   // going to cut too far on next move
//          _deltaRForPass = (ARG_J - WorkingDiameter) / 2.0f;
//          WorkingDiameter = ARG_J;    // set up working diameter for NEXT pass
//      }
//      else
//      {
//          _deltaRForPass = ARG_I / 2.0;
//          WorkingDiameter += ARG_I;   // set up working diameter for NEXT pass
//      }
//    }
//    else
//    {
		_deltaRForPass = 0.0f;
		WorkingDiameter += ARG_I;   // set up working diameter for NEXT pass
//    }

	if ((WorkingDiameter >= ARG_J) || (ARG_C_MISSING && ARG_E_PRESENT && (WorkingDiameter+ARG_I > ARG_J)))
	{   // have to look 2 ahead when printing to avoid tapering pass
		if (ARG_E_MISSING || ARG_C_PRESENT) // milling
			G203State = 2; //next time it will be the finish circle.
		else // printing
			G203State = 1; //next time it will be the lead out back to the origin (no finish circle)
	}

	setupArcMove(StartAngleInDegrees + (180*CW_CCW_Modifier),SweepAngle,Radius,arcArgS);
	//ExecuteG2G3PointMove();
}

////////////////////////////////////////////////////////////////////////////////

void StartNextArcForPocketMill(void)
{
	;//makes sequencially larger circles until the max J is found

}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G16(void)  // disable the working plane
{
	// GCODE G16
	// GCODE
	// GCODE disabale the working plane (allows arbitrary plane definition)

	_gs._selectedArcPlane = ARC_PLANE_NONE;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G17(void)  // set the working plane to PLANE_XY
{
	// GCODE G17
	// GCODE
	// GCODE set the working plane to PLANE_XY with a normal of [0,0,1]

	_gs._selectedArcPlane = ARC_PLANE_XY;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G18(void)  // set the working plane to PLANE_XZ
{
	// GCODE G18
	// GCODE
	// GCODE set the working plane to PLANE_XZ with a normal of [0,1,0]

	_gs._selectedArcPlane = ARC_PLANE_XZ;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G19(void)  // set the working plane to PLANE_YZ
{
	// GCODE G19
	// GCODE
	// GCODE set the working plane to PLANE_YZ with a normal of [1,0,0]

	_gs._selectedArcPlane = ARC_PLANE_YZ;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G20(void)
{   //programming is in inches
	ConvertValueToMM = 25.4f; // conversion of Args to MM (set to 25.4 convert in to mm)
	ConvertValueFromMM = 1.0f/25.4f; // conversion of MM to Args (set to 1/25.4 convert mm to in)
	sendError("G20 (inch) mode is not fully supported -- expect issues");
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G21(void)
{   //programming is in mm
	ConvertValueToMM = 1.0f; // conversion of Args to MM (set to 1.0 to stay mm)
	ConvertValueFromMM = 1.0f; // conversion of Args to MM (set to 1.0 to stay mm)
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G28_0(void)
{   //Go Home
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
	{   // lightburn does not provide the list of axis to home
		ARG_X = 0.0f;
		ARG_Y = 0.0f;
	}
#endif //ALLOW_NATIVE_LIGHTBURN
	ResetAllMotionOffsetIndices();

	CannedCyclePtr = ExecutionPtr; //save this pointer for future reference
	CannedCycleFlag = 28;
	CannedCycleStep = G28_START_MULTIPASS_HOME;
	//sequence engine will make the call: processCannedCycle();
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G28_1(void)   // home a canAxisMotor (uses XYZABC, F, R, L)
{
	// GCODE G28.1 ["X,Y,Z,A,B,C"] [P forcePct] [L limitSwitch] [F rate | Hx]
	// GCODE
	// GCODE    if X,Y,Z,A,B,C are present, then home those axes provided they are canAxisMotors
	// GCODE
	// GCODE	if argL is present, then home to the on-head sensor based on home to zero / home to infinity
	// GCODE    else home until the force limit is reached based on home to zero / home to infinity
	// GCODE
	// GCODE	if (argH==1, then using the HOMING_RATE
	// GCODE    else if argF present, then use argF speed for the homing move
	// GCODE    else use the REHOMING_RATE
	// GCODE
	// GCODE    if argP present, set the force limit for the homing move
	// GCODE
	// GCODE    home a canAxisMotor


	MotorStructure *M;
	ResetAllMotionOffsetIndices();  // NOTE:  the toollength for toolIndex 0 is still in play (set after homing)

	uint16_t forcePct = (ARG_P_PRESENT) ? convertFloatPowerPctToUnsignedint(fFitWithinRange(ARG_P/100.0f, 0.0f, 1.0f), HH_U16_POWER_PCT_FRAC_BITS) : 0;
	uint16_t limitIndex = (ARG_L_PRESENT) ? iFitWithinRange((int32_t)ARG_L, 0, 1) : 0;
	byte method = (ARG_L_PRESENT) ? CLOSED_LOOP_MOTOR_HOME_TO_LIMIT : CLOSED_LOOP_MOTOR_HOME_BY_FORCE;


	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG for the motor, update the 0 position and move there
		if (motorArgPresent(M) && M->canMotor)
		{
			byte allAxisAlias = ALIAS_ALL_CAN_AXIS_MOTORS_X + M->Axis;

			int16_t steps = (M->HomingDirection == HOME_TOWARD_ZERO) ? -1 : 1;

			if (M->Direction.InvertDefault)
				steps *= -1;

			float rateUPS = M->RatesInUPS[REHOMING];	//default
			if (ARG_H_PRESENT)
				rateUPS = M->RatesInUPS[HOMING];
			else if (ARG_F_PRESENT)
				rateUPS = convertArgToMM(ARG_F) / 60.0f;

			uint16_t stepRate = iFitWithinRange((uint32_t)(rateUPS * M->PulsesPerUnit), 1, 65535);

			for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
			{
				if (M->canMotors[canAddrIndex].canAddress)
				{
					M->canMotors[canAddrIndex].selfHomingInProgress = TRUE;
					M->canMotors[canAddrIndex].selfHomed = FALSE;
				}
			}
			M->axisSelfHomingInProgress = TRUE; // tag that the axis as a whole was homing .. save processing at the end
			sendMotorMotionValues(allAxisAlias, M);		// set up limits on speeds and feeds on motors
			// set up the actal move
			canPackIntoTxQueue4x16(CAN_WRITE, allAxisAlias, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, method, BUFFERED_MSG,
					*(uint16_t *)&steps, //cheap way to get the int16 into the uint16 parameter without losing the sign
					stepRate,
					forcePct,
					limitIndex);
			_MailBoxes._waitingFor.flags.bit.canAxisMotorHoming = TRUE;
			_MailBoxes._waitingFor.timerHoming = CANBUS_HOMING_COMPLETION_TIME;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G28(void)    // G28 master command
{
	switch (getArgFraction(ARG_G))
	{
	case 0: G_Code_G28_0(); break;   // normal homing
	case 1: G_Code_G28_1(); break;   // canAxisMotor self homing
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G928(void)
{   // Jog and reset origin.
	// need to treat this as a relative move in order for MotionQ_addCommand to not need to special case this everywhere
	// for blokcingAbsoluteMotion and for checking for limited incr moves (ie, L1 or L2 tripped)
	// effectively treating this as a jog command

	MotorStructure *M;
	ResetAllMotionOffsetIndices();  // NOTE:  the toollength for toolIndex 0 is still in play (set after homing)

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG for the motor, update the 0 position and move there
		if (motorArgPresent(M))
		{
			if (motorArgNegativeOrZero(M))
			{   // must be a negative value (or zero) to reset the motor zero and move again.
				// fake the qqueue and actual position such that the a relative move of the req offset will land at zero.
				M->Q_LastRequestedPositionInUnits = -1.0f * getMotorArgInMM(M);
				M->Q_POSITION = M->Q_LastRequestedPositionInUnits * M->PulsesPerUnit;
				M->POSITION = M->Q_POSITION;
				M->LastReportedPosition = INVALID_ARG_VALUE; // make sure display is updated properly

				setMotorArg(M, getMotorArgInMM(M) + (-1.0f * SumOfAllMotorOffsets(M)));   //incr of the requested amout ... -SumOfAll added in so it will be cancelled out later when the move is calculated
			}
			else
			{
				ReportGcodeError("args to G928 must be <= 0.0");
				return;
			}
		}
	}

	boolean save_IncrementalMove = _IncrementalMove; // save state of _IncrementalMove to switch to Absolute Mode.
	_IncrementalMove = TRUE;    // force incremental move

	motionQ_addCommand(HOMING_MOTION_RATE, NOT_HOMING_MOTORS);

	_IncrementalMove = save_IncrementalMove;  // restore state of _IncrementalMove
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G29(void)
{
	return;
}

////////////////////////////////////////////////////////////////////////////////

void startProbingMove(void)
{
	ExecutionPtr = CannedCyclePtr; //restore ExecutionPtr to our index
	float save_TargetFeedRateInMmPerSec = _TargetFeedRateInMmPerSec; // save current feedrate;
	boolean save_IncrementalMove = _IncrementalMove; // save state of _IncrementalMove to switch to Absolute Mode.
	_IncrementalMove = TRUE;
	motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);  // motionQ should be empty, so no need to check
	_IncrementalMove = save_IncrementalMove;
	_TargetFeedRateInMmPerSec = save_TargetFeedRateInMmPerSec;       // restore feedrate
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G38(void)   // touch probe move (uses XYZABC, P, T, F, D, I, H, O]
{
	// GCODE G38 ["X,Y,Z,A,B,C" <relativeDist>] <T probeHead> <P probe#> <D probeType> [I initState] [H toolIndex] [O addedToolIndexOffset] [F feedrate]
	// GCODE G38 ["X,Y,Z,A,B,C" <relativeDist>]               <P probe#> <D probeType> [I initState] [H toolIndex] [O addedToolIndexOffset] [F feedrate]
	// GCODE
	// GCODE    if X,Y,Z,A,B,C are present, then relative, coordinated move of those axes
	// GCODE    if argF, the temporary feedrate change for this move only (note, rate must be
	// GCODE       noRamp rate for each axis.  noRamp rate used if argF not spec'd
	// GCODE    if ArgT specified, then using probe head and argP selects which pins to use on the probe head
	// GCODE         0 : PA3 pin on 10-pin connector
	// GCODE         1 : PA2 pin on 10-pin connector
	// GCODE         2 : LIMIT1 pin on 18-pin connector
	// GCODE         3 : LIMIT2 pin on 18-pin connector
	// GCODE         4 : RTD1 pin on 18-pin connector
	// GCODE         5 : RTD2 pin on 18-pin connector
	// GCODE    else if ArgT not specified, then using system probe and argP selects which pin to use on the motherboard
	// GCODE         0 : No direct connect probe; canbus based probe one  *** must specify probeHead via T arg
	// GCODE         1 : Limit1 pin on X axis connector or W_RTD1 (sys30)
	// GCODE         2 : Limit2 pin on X axis connector or W_RTD2 (sys30)
	// GCODE         3 : Fault  pin on X axis connector
	// GCODE         4 : Home   pin on X axis connector
	// GCODE         5 : unused
	// GCODE         6,7,8,9 : Limit1,Limit2,Fault,Home pin on Y axis connector
	// GCODE         10 : unused
	// GCODE         11,12,13,14 : Limit1,Limit2,Fault,Home pin on Z axis connector
	// GCODE         15 : unused
	// GCODE         16,17,18,19 : Limit1,Limit2,Fault,Home pin on A axis connector
	// GCODE         20 : unused
	// GCODE         21,22,23,24 : Limit1,Limit2,Fault,Home pin on B axis connector
	// GCODE         25 : unused
	// GCODE         26,27,28,29 : Limit1,Limit2,Fault,Home pin on C axis connector
	// GCODE    ARG_D  (same as M630)
	// GCODE       0 = PROBE_TYPE_NONE
	// GCODE       1 = PROBE_TYPE_CONTACT
	// GCODE       2 = PROBE_TYPE_BLTOUCH (currently not supported in production code)
	// GCODE       3 = PROBE_TYPE_HC_SR04 (currently not supported in production code)
	// GCODE       4 = PROBE_TYPE_IGAGING (currently not supported in production code)
	// GCODE    ARG_I - initial state of contact probe
	// GOCDE    ARG_H - if specified, the height of toolIndex=ARG_H is set to the probed_Z_height (+ ARG_O, is ARG_O is specified)
	// GCODE    ARG_O - added tool offset to the probe Z height when setting tool height using ARG_H.  for example, if the Z
	// GCODE            position is 3.56 when the probe makes contact, then toollength[ARG_H] = 3.56 + ARG_O
	// GCODE
	// GCODE    get position of motion system when probe is contacted.  error reported if
	// GCODE    move completes without the probe being contact

	if (ARG_D_MISSING) { ReportMissingGcodeDArg(); return; }
	if (ARG_P_MISSING) { ReportMissingGcodePArg(); return; }
	if ((ARG_P == 0) && ARG_T_MISSING) { ReportMissingGcodeTArg(); return; }
	if ((ARG_P < 0.0f) || ((int)ARG_P > 29)) { ReportInvalidGcodePArg(); return; }

	if (ARG_H_PRESENT)
	{
		if ((((int)ARG_H) < 0) || (((int)ARG_H) >= NUM_TOOL_OFFSETS)) { ReportGcodeError("ToolIndex (ARG_H) Out of Range"); return; }
		if (ARG_O_MISSING)
		{   // make sure ARG_O is set to 0 for the "unused" case
			ARG_O = 0.0f;
		}
	}

	probeType_t probeType = (probeType_t)(int)ARG_D;

	if (ARG_T_PRESENT)
		_TouchProbeCanDevice = (byte)ARG_T;             // using canbus based probe
	else
		_TouchProbeCanDevice = CAN_SYSTEM_DEVICE_ID;    // using motherboard based probe

	int initState = (ARG_I == 1.0f) ? 1 : 0;
	int edge;
	if (initState == 1)
		edge = EXTI_Trigger_Falling;
	else // (initState == 0)
		edge = EXTI_Trigger_Rising;

	if (SetupEXTI((int)ARG_P, probeType, edge, initState))
	{
		CannedCyclePtr = ExecutionPtr; //save this pointer for future reference
		_TouchProbeMoveActive = TRUE;  // let system know we intend to move and
		if (_TouchProbeCanDevice != CAN_SYSTEM_DEVICE_ID)
		{
			_MailBoxes._waitingFor.flags.bit.canbusProbeToArm = TRUE;   // wait until the can device has armed before moving
			_MailBoxes._waitingFor.timerProbe = CANBUS_PROBE_ARM_COMPLETION_TIME;
		}
		else
		{
			startProbingMove();
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G53(void)
{ //Set current as home
	currentFixtureIndex=0;
}
void G_Code_G54(void)
{ //Set current as home
	currentFixtureIndex=1;
	SetFixtureOffset(currentFixtureIndex);//set the offset values
}
void G_Code_G55(void)
{ //Set current as home
	currentFixtureIndex=2;
	SetFixtureOffset(currentFixtureIndex);//set the offset values
}
void G_Code_G56(void)
{ //Set current as home
	currentFixtureIndex=3;
	SetFixtureOffset(currentFixtureIndex);//set the offset values
}
void G_Code_G57(void)
{ //Set current as home
	currentFixtureIndex=4;
	SetFixtureOffset(currentFixtureIndex);//set the offset values
}
void G_Code_G58(void)
{ //Set current as home
	currentFixtureIndex=5;
	SetFixtureOffset(currentFixtureIndex);//set the offset values
}
void G_Code_G59(void)
{ //Set current as home
	currentFixtureIndex=6;
	SetFixtureOffset(currentFixtureIndex);//set the offset values
}

////////////////////////////////////////////////////////////////////////////////

void SetFixtureOffset(int fixture)
{
	MotorStructure *M;

	if (fixture < NUM_FIXTURE_OFFSETS)
	{   // valid range
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors, if there's a valid ARG for the motor
			if (motorArgPresent(M))
			{
				M->FixtureOffsets[currentFixtureIndex] = getMotorArgInMM(M);
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
//G70 Bolt Hole Circle
void G_Code_G70(void)
{
	CannedCycleFlag=70;
}

//G71 Bolt Hole Arc
void G_Code_G71(void)
{
	CannedCycleFlag=71;
}

//G73 High-Speed Peck Drilling Canned Cycle
void G_Code_G73(void)
{//peck drill cycle  not implemented yet
	setupDrillCycleParameters(ARG_G);
}

//G74 Reverse Tap Canned Cycle not implemented yet
void G_Code_G74(void)
{
	CannedCycleFlag=74;
}

//G76 Fine Boring Canned Cycle not implemented yet
void G_Code_G76(void)
{
	CannedCycleFlag=76;
}

//G77 Back Bore Canned Cycle not implemented yet
void G_Code_G77(void)
{
	CannedCycleFlag=77;
}

void KillCannedCycle(void)
{
	G2G3Flag = 0;
	G203State = 0;
	CannedCycleFlag=0;
	CannedCycleStep=0;
	CannedZClearInMM=0.0f;
	CannedZDepthInMM=0.0f;
	CannedCycleZFeedRateInMmPerSec=0.0f;
	CannedZIncrement=0.0f;
	CannedLinearAxis=M_Z;
	CannedRotaryAxis=M_A;
	CannedHssPtr=NULL;
	CannedZQIncrement=0;
	CannedThreadPitch=1.0f;
}
//G80 Canned Cycle Cancel
void G_Code_G80(void)
{
	KillCannedCycle();
}

////////////////////////////////////////////////////////////////////////////////

void setupDrillCycleParameters(float cmd)
{
	//G81 X0 Y0 Z(depth)  I(clearance plane) F(DRILL feed rate) P(Incremental Z down)
	if (motionQ_full())
	{
		barf("G_Code_G83(): motionQ not empty");
		return;
	}
	if (_IncrementalMove)
	{//no incremental moves allowed for this canned cycle
		sprintf(_errorStr, "G%d must be executed in absolute coordinates (mode G90)", (int)ARG_G);
		sendError(_errorStr);
		return;
	}
	if (ARG_X_MISSING &&  ARG_Y_MISSING)
	{   // karl requires at least one to be specified
		sprintf(_errorStr, "G%d - X or Y must be specified", (int)ARG_G);
		sendError(_errorStr);
		return;
	}
	CannedCyclePtr = ExecutionPtr;
	if (ARG_I_PRESENT) CannedZClearInMM=convertArgToMM(ARG_I);//set up the initial plane, ie the retract level
	if (ARG_R_PRESENT) CannedZClearInMM=convertArgToMM(ARG_R);//set up the initial plane, ie the repid and retract level
	if (ARG_Z_PRESENT) CannedZDepthInMM=convertArgToMM(ARG_Z);//that is how deep the hole will be
	if (ARG_F_PRESENT) CannedCycleZFeedRateInMmPerSec=convertArgToMM(ARG_F/60.0f);//set the drilling feed rate
	if (ARG_P_PRESENT) CannedZIncrement=ARG_P;//pecking distance is the delta Z for each drill peck
	if (ARG_Q_PRESENT) CannedZQIncrement=ARG_Q;//amount to retract between each peck if missing then it is full retract to R plane
	if ((cmd==71)||(cmd==71))CannedZIncrement= Motors[M_Z].Q_LastRequestedPositionInUnits-CannedZDepthInMM;//no pecking for these 2 gcodes
	if (CannedZIncrement==0)
	{//must have some peck distance please
		sprintf(_errorStr, "G%d -- CannedZIncrement (ARG_P) must be non-zero", (int)ARG_G);
		sendError(_errorStr);
		return;//do nothing if no increment
	}
	//setup initial rapid move
	CannedZDesiredDepth = CannedZClearInMM; //since we jump into the state machine as if we are clearing a hole, make sure there's no rapid return to the bottom of the hole.
	CannedThreadPitch = 1.0f;
	ARG_F=INVALID_ARG_VALUE; // not a motion feedrate change
	ARG_Z=INVALID_ARG_VALUE;//disable the z so we do not accidently go to the bottom of the hole on the first seek

	CannedCycleStep=3;//set up the state engine to start execution after XY Move
	motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
	CannedCycleFlag=83;
}

float validateThreadPitch(float pitch)
{
	return(((pitch == INVALID_ARG_VALUE) || (pitch <= 0.0f)) ? 1.0f : pitch);	// range/value check
}

void setupDrillTapCannedCycleVersion1(float cmd)
{   // new G83.1/G84.1 Peck Drilling/Tapping Canned Cycle
	// GCODE G83.1/G84.1 [XYZABC initPos] [L linearAxis] <R tappingAxis> [I clearPlane] [D depth] [J pitch] [P linIncr] [F linFeed] [<W washSwitchFunc> <O washTime>]
	// GCODE
	// GCODE    (L)linearAxis or (R)tappingAxis -->  0=X 1=Y 2=Z 3=A 4=B 5=C
	// GCODE
	// GCODE    step 1) rapid move to initPos XYZABC (less the L and R axis
	// GCODE    step 2) rapid move of linear axis to clearPlane (I)
	// GCODE    step 3) coordinate move of linear and rotary axis incrementally(P) at F rate until depth D is reached
	// GCODE

	// ARG_W and ARG_S -- hss control already processed

	// check for additional errors
	if (ARG_R_MISSING) { ReportMissingGcodeRArg(); return; }

	CannedLinearAxis = ARG_L_PRESENT ? (int)ARG_L : M_Z;    // Z axis is default
	CannedRotaryAxis = ARG_R_PRESENT ? (int)ARG_R : M_A;    // A axis is default
	CannedZClearInMM = ARG_I_PRESENT ? convertArgToMM(ARG_I) : Motors[CannedLinearAxis].Q_LastRequestedPositionInUnits;
	CannedZDepthInMM = ARG_D_PRESENT ? ARG_D : CannedZDepthInMM;

	CannedThreadPitch = validateThreadPitch(ARG_J);

	CannedCycleZFeedRateInMmPerSec = ARG_F_PRESENT ? convertArgToMM(ARG_F/60.0f) : CannedCycleZFeedRateInMmPerSec;  //set the drilling feed rate

	if ((int)cmd == 81)
	{   // no incremental depth -- all in one shot
		CannedZIncrement = CannedZDepthInMM;
	}
	else
	{
		CannedZIncrement = ARG_P_PRESENT ? ARG_P : CannedZIncrement;
		if (CannedZIncrement == 0.0f)
		{   //do nothing if no increment
			sprintf(_errorStr, "G%3.1f -- CannedZIncrement (ARG_P) must be non-zero", cmd);
			sendError(_errorStr);
			return;
		}
	}

	ARG_L = ARG_R = ARG_I = ARG_J = ARG_D = ARG_F = ARG_P = INVALID_ARG_VALUE;
	CannedCycleFlag = (int)cmd;
	CannedCyclePtr = ExecutionPtr;
	CannedCycleStep = 5;    //set up for drill/tap height
}

////////////////////////////////////////////////////////////////////////////////

void setupDrillTapCannedCycleVersion0(float cmd)
{   // original G83/G84 Peck Drilling/Tapping Canned Cycle
	// GCODE G83/G84 [XY initPos] [ABC tapping] [I clearPlane] [Z depth] [P linIncr] [F linFeed] [J pitch] [<W washSwitchFunc> <O washTime>]
	// GCODE
	// GCODE X/Y - starting position
	// GCODE A|B|C == 1 to select tapping axis
	// GCODE I is clearance Z position
	// GCODE Z is the final Z position
	// GCODE P is the incremental Z axis change per pass
	// GCODE F is the feed when actually cutting material
	// GCODE L & S together turn on the selected switch for a requested duration each pass of the cycle
	// GCODE J is the thread pitch
	// GCODE
	// GCODE    step 0) rapid move to initPos XY
	// GCODE    step 2) rapid move of linear axis to clearPlane (I)
	// GCODE    step 3) coordinate move of linear and rotary axis incrementally (P) at F rate until Z(depth) is reached

	// ARG_W and ARG_S -- hss control already processed

	// check for additional errors
	if (ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING)
	{
		sprintf(_errorStr, "G%3.1f -- CannedRotaryAxis not selected via ARG_A or ARG_B or ARG_C", cmd);
		sendError(_errorStr);
		return;
	}

	CannedLinearAxis =  M_Z;    // Z axis is default

	if (ARG_A_PRESENT) CannedRotaryAxis =  M_A;
	else if (ARG_B_PRESENT) CannedRotaryAxis =  M_B;
	else if (ARG_C_PRESENT) CannedRotaryAxis =  M_C;

	CannedZClearInMM = ARG_I_PRESENT ? convertArgToMM(ARG_I) : Motors[CannedLinearAxis].Q_LastRequestedPositionInUnits;
	CannedZDepthInMM = ARG_Z_PRESENT ? ARG_Z : CannedZDepthInMM;
	CannedCycleZFeedRateInMmPerSec = ARG_F_PRESENT ? convertArgToMM(ARG_F/60.0f) : CannedCycleZFeedRateInMmPerSec;  //set the drilling feed rate

	CannedThreadPitch = validateThreadPitch(ARG_J);

	if ((int)cmd == 81)
	{   // no incremental depth -- all in one shot
		CannedZIncrement = CannedZDepthInMM;
	}
	else
	{
		CannedZIncrement = ARG_P_PRESENT ? ARG_P : CannedZIncrement;
		if (CannedZIncrement == 0.0f)
		{   //do nothing if no increment
			sprintf(_errorStr, "G%3.1f -- CannedZIncrement (ARG_P) must be non-zero", cmd);
			sendError(_errorStr);
			return;
		}
	}

	ARG_A = ARG_B = ARG_C = ARG_I = ARG_Z = ARG_F = ARG_P = INVALID_ARG_VALUE;
	CannedCycleFlag = (int)cmd;
	CannedCyclePtr = ExecutionPtr;
	CannedCycleStep = 5;    //set up for drill/tap height
}

////////////////////////////////////////////////////////////////////////////////
void setupDrillCannedCycle(float cmd)
{
	if (motionQ_notEmpty()) { barf("setupDrillTapCannedCycle(): motionQ not empty"); return; }
}

void setupDrillTapCannedCycle(float cmd)
{   // Generic call to the family of G83/G84 Tapping Canned Cycle commands
	// Specific G84 flavors are called from this routine after common args and errors are processed
	// check errors common to all flavors and process any common args

	if (motionQ_notEmpty()) { barf("setupDrillTapCannedCycle(): motionQ not empty"); return; }
	if (_IncrementalMove) { sprintf(_errorStr, "G%3.1f must execute in absolute coordinates (mode G90)", cmd); sendError(_errorStr); return; }

	if (ARG_W_PRESENT && ARG_S_PRESENT)
	{   // wash active during cycle)
		if ((int)ARG_S < 1) ReportInvalidGcodeSArg();
		if ((int)ARG_S > NUM_HSS_FUNC) ReportInvalidGcodeSArg();

		CannedHssPtr = &HighSideSwitches[hssFuncToPinIndex[(int)ARG_S]];
		CannedHssPtr->DutyCycle = HSS_DUTY_CYCLE_OFF;
		CannedHssPtr->PeriodInSec = ARG_W  / 1000.0f;
		CannedHssPtr->oneShot = TRUE;
		CannedHssPtr->TicksPerSecond = TICKS_PER_SEC_1000HZ_LOOP; // set for ms
		updateHssPwm(CannedHssPtr); // set up the switch to start off 'OFF'
		CannedHssPtr->DutyCycle = HSS_DUTY_CYCLE_ON;    // now it's ready to go for creating one shots
	}
	else
	{   // lube inactive during cycle
		CannedHssPtr = NULL;
	}

	switch (getArgFraction(cmd))
	{
	case 0: setupDrillTapCannedCycleVersion0(cmd); break;   // original version
	case 1: setupDrillTapCannedCycleVersion1(cmd); break;   // newer version allowing more control
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G81(void)
{   // Generic call to the family of G81 Drill Canned Cycle commands
	// Specific G81 flavors are called from this routine after common args and errors are processed
	//G81 X0 Y0 Z(depth)  I(clearance plane) F(DRILL feed rate)

	setupDrillCycleParameters(ARG_G);
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G82(void)
{   //G82 Spot Drill Canned Cycle
	setupDrillCycleParameters(ARG_G);
}

////////////////////////////////////////////////////////////////////////////////


//G83 XYZ R P Q F L
//XY: Coordinates of the hole
//Z: Hole bottom
//R: Retract position in Z. Motions from initial Z to R are performed at rapids speeds. From R to hole bottom is done at feed speed.
//P: Dwell time at bottom of hole.
//Q: Depth to increase on each peck.
//F: Cutting feedrate
//L: Number of repeats

void G_Code_G83(void)
{   // Generic call to the family of G83 Drill Canned Cycle commands
	// Specific G83 flavors are called from this routine after common args and errors are processed

	setupDrillCycleParameters(ARG_G);
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G84(void)
{   // Generic call to the family of G84 Tapping Canned Cycle commands
	// Specific G84 flavors are called from this routine after common args and errors are processed
	//	G84   (tapping with Z restricted to linear move and ABorC as tapping axis)
	//	          XY initial move
	//	          Z is the depth
	//	          A,B or C<arg> .... any arg, selects the threading axis
	//	          J is the pitch
	//
	//	G84.1 (fully generic, ANY axis can play either role in the tapping process)
	//	          XYZABC initial move
	//	          L is the 'linear' move axis index (x=0, y=1....)
	//	          R is the tapping axis index (x=0, y=1....)
	//	          D is the depth
	//	          J is the pitch

	setupDrillTapCannedCycle(ARG_G);
}

////////////////////////////////////////////////////////////////////////////////

// G85 Boring Canned Cycle
void G_Code_G85(void)
{
	CannedCycleFlag=85;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G90(void)
{   //Absolute Positioning
	_IncrementalMove = FALSE;    //now it is absolute motion
}

void G_Code_G91(void)
{   //Incremental Positioning
	_IncrementalMove = TRUE;    // now it is relative motion
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G92(void)
{   // Set the G92 Offsets
	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command update the G92Offset
		if (motorArgPresent(M))
		{
			M->G92Offset = M->Q_LastRequestedPositionInUnits - getMotorArgInNativeUnits(M);
		}
	}
	// special case to pick up the extruder position (slic3r reports it this way)
	if (ARG_E_PRESENT)
	{   // if there's an E argument, it is the Extruder's current position
		E_Q_LastRequestedPositionInUnits = convertArgToMM(ARG_E);
		E_Q_POSITION = imax(0, (int)(E_Q_LastRequestedPositionInUnits * currentOutboxPtr->ExtrusionControl.PulsesPerUnit));

		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors, if there's a valid ARG from the command
			if (motorArgPresent(M) && M->SubstituteAxisArgWithArgE)
			{       // this forces an axis to copy the E value as it's position for the next move (and handle the separate relative/absolute diff between E and rest of the axis
				M->Q_LastRequestedPositionInUnits = convertArgToMM(ARG_E);
				M->Q_POSITION = imax(0, (int)(M->Q_LastRequestedPositionInUnits * M->PulsesPerUnit));
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G93(void)
{   // cancels the G92 offset
	ClearAllMotorG92Offset();
}

////////////////////////////////////////////////////////////////////////////////

//Circular hole, G202/203
//  Format:
//  G203 r_ f_
//  r is the radius.
//
//  Example:
//
//  g x y z5
//  g1 z-10 f200
//  g203 r100 f1000

void G_Code_G202(void);//circular hole bore milling Clockwise
void G_Code_G203(void);//circular hole bore milling Counter Clockwise

//  Pocket G-codes
//
//  For all pockets:
//  A pocket is generated by successive cuts, based on a percentage of the tool diameter.
//  Tool radius cannot be zero.
//  n – percentage of tool diameter per cut, cannot be zero.
//  k – finish cut, may be zero.
//  The finish cut is entered by a tangent arc from the previous cut and left by a tangent arc to the center. The last move also raises the tool 1 mm (0.04").
//  If the finish cut is zero the last cut repeats twice.
//  Rotation can be used. Scale shouldn't be used.
//
//  Note: Even numbers cut CW, odd numbers cut CCW.
//  Circular pocket, G212/213
//  Format:
//  G213 r_ n_ k_ f_
//  r is the desired radius.
//
//  Example:
//
//  g x y z5
//  g1 z-11 f200
//  g213 r120 n50 k2 f1000


//  Circular pocket, G212/213
//  Format:
//  G213 r_ n_ k_ f_
//  r is the desired radius.
//
//  Example:
//
//  g x y z5
//  g1 z-11 f200
//  g213 r120 n50 k2 f1000

void G_Code_G212(void)
{}  //clockwise circular pocket mill
void G_Code_G213(void)
{}  //Cclockwise circular pocket mill

//  Rectangular pocket, G214/215
//  Format:
//  G215 i_ j_ r_ n_ k_ f_
//  ij are half the width and the height.
//  r is a fillet, may be zero.
//
//  Example:
//
//  g x y z5
//  g1 z-11 f200
//  g215 i100 j120 r25 n50 k2 f1000

void G_Code_G214(void)  //clockwise Rectangular pocket mill
{//lets make sure that all arguments are there
	sendError("G214 not implemented yet");
	return;
	if(ARG_F_MISSING) { ReportMissingGcodeFArg(); return; }
	if(ARG_I_MISSING) { ReportMissingGcodeIArg(); return; }
	if(ARG_J_MISSING) { ReportMissingGcodeJArg(); return; }
	//if(ARG_Z_MISSING) { ReportMissingGcodeZArg(); return; }
	if(ARG_D_MISSING) {  ReportMissingGcodeDArg(); return; }
	//if(ARG_P_MISSING) { ReportMissingGcodePArg(); return; }
	//this will start by dropping the head down to the programmed Z
	//float currentZ=Motors[M_Z].LastRequestedPositionInUnits;


	//  PointF P1,P2,P3,P4;
	//  float XCenter,YCenter;
	//  ToolDiameter = .3f;
	//  float ToolRadiusOffset = ToolDiameter / 2.0f;
	//  float WorkingRadius = 0;
	//  StringBuilder Sb = new StringBuilder();
	//  Sb.Append(";Gerber action Tool " + toolNumber + " D=" + aperatureX +", "+ aperatureY + "\n");
	//  float ToolDiameterIncrement = .2f;// = ToolDiameter * .6f;//either the laser or the end mill diameter
	//
	//  float halfDiameter = (aperatureX - ToolDiameter) / 2.0f;
	//
	//  float halfX = (aperatureX- ToolDiameter) / 2.0f;
	//  float halfY = (aperatureY- ToolDiameter) / 2.0f;
	//  if (halfX < .001) halfX = 0;// ToolDiameter;//do this so that if the laser or cutter is bigget than the pad, it will make only 1 pass
	//  if (halfY < .001) halfY = 0;// ToolDiameter;// halfX;
}
void ProcessNextPocketMillMove()
{
	//  XCenter = PadCenter.X + offsetX;
	//      YCenter = PadCenter.Y + offsetY;
	//      {//this is simple rectangle so go to center + 1/2 diameter and perform G2 command, then shrink and repeat
	//          P1 = new PointF(XCenter - halfX, YCenter - halfY);
	//          P2 = new PointF(XCenter + halfX, YCenter + halfY);
	//          if (halfX <= .001) P1.X = P2.X = XCenter;
	//          if (halfY <= .001) P1.Y = P2.Y = YCenter;
	//          Sb.Append("G0 X" + XCenter + " Y" + YCenter + "\n");//haul ass to get over the desired drill center.
	//          Sb.Append("G1 E.1 F"+ zFeedRate +" X" + P1.X.ToString() + " Y" + P1.Y.ToString() + "\n");//haul ass to get over the desired drill center.
	//          GenerateGcodeRectangle(Sb, P1, P2);
	//          if (LINEWIDTH == 0)
	//          { continue; }
	//          while ((P1.X<P2.X)||(P1.Y < P2.Y))
	//          {
	//              if(P1.X < P2.X)
	//              {
	//                      P1.X += ToolDiameterIncrement;//add .2mm and then try again
	//                      P2.X -= ToolDiameterIncrement;
	//                      if (P1.X > P2.X)
	//                      {
	//                          P1.X = P2.X = XCenter;
	//                      }
	//              }
	//
	//              if (P1.Y < P2.Y)
	//              {
	//                      P1.Y += ToolDiameterIncrement;//add .2mm and then try again
	//                      P2.Y -= ToolDiameterIncrement;
	//                      if (P1.Y > P2.Y)
	//                      {
	//                          P1.Y = P2.Y = YCenter;
	//                      }
	//              }
	//
	//              GenerateGcodeRectangle(Sb, P1, P2);
	//          }
	//      }
}
void G_Code_G215(void)  //clockwise Rectangular pocket mill
{
	sendError("G215 not implemented yet");
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum
{
	CYLINDER_NON            = 0,
	CYLINDER_ANGLED         = 1,
	CYLINDER_PARALLEL       = 2,
	CYLINDER_PERPENDICULAR  = 3,
} cylType_t;


typedef struct {
	pointStruct     P1;
	pointStruct     P2;
	vectorStruct    V;  // V from P1 t P2
	vectorStruct    UV; // UV from P1 t P2
} pointVectorStruct;

typedef struct {
	MotorStructure  *rotaryMotorAxisPtr;

	pointStruct     startPt;
	pointStruct     currPt; // needed ONLY for CYLINDER_PERPENDICULAR case (not updated otherwise)
	pointStruct     destPt;
	float           rotStartPt;
	float           rotDestPt;
	vectorStruct    moveV;      // magnitude of XYZ move need (length of cyl)
	pointStruct     perpOfs;    // needed for CYLINDER_PERPENDICULAR case
	vectorStruct    liftUV;     // UV in direction of printing
	vectorStruct    rotUV;

	cylType_t       type;
	direction_t     dir;
	int             passes; // not used
	float           length;
	float           circumference;
	float           requestedAngleInRadians;
	float           actualAngleInRadians;
	float           requestedPitch;
	float           actualPitch; // just for CYLINDER_PERPENDICULAR case
	float           rotLength;  // temp var
	float           rotDegrees;
	float           jogLength;  // temp var
	float           jogDegrees;

	float           pathLength; // used for self check
	float           totalTravel; // just for info

	float           save_IncrementalMove;
	E_control_t     save_extrusionControl;

	float           argF;
	float           argE;

	float           flowScale;  //no persistant scaling of flow rate
	float           liftAmount;
	float           overlapReduction;
	boolean         alignedSeam;
	boolean         angleIsKing;
} cylStruct;

cylStruct _cyl;


////////////////////////////////////////////////////////////////////////////////

void initCylStruct(cylStruct *cylPtr)
{
	bzero(cylPtr, sizeof(cylStruct));
}

////////////////////////////////////////////////////////////////////////////////

boolean G702_checkRotaryAxisArg(cylStruct *cylPtr, float arg, motorIndex_t index)
{   // return true if failed
	if (arg != INVALID_ARG_VALUE)
	{
		if (cylPtr->rotaryMotorAxisPtr)
		{
			ReportGcodeError("multiple rotary axes defined");
			return(TRUE);
		}
		else
		{
			cylPtr->rotaryMotorAxisPtr = &Motors[index];
		}
	}
	return(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

#if 0 // NUKE ?save for now -- until 3d 702 finished
boolean G702_setupNormalsAndOffset(cylStruct *cylPtr)
{   // return true if failed

	boolean badArgs = FALSE;

	if (cylPtr->plane != ARC_PLANE_NONE)
	{   //case 1 above -  arc is defined to be in one of the 3 primary planes XY, XZ, YZ
		// last/specified XYZ is the interestection point with the plane (P1)
		// create P2 along rotationalUV to form rotational line which is needed later to
		// determine displacement from rotational axis

		vectorStruct rotAxisUV;
		vectorStruct planeV;

		switch(cylPtr->plane)
		{
		case ARC_PLANE_XY :
			planeV.z = 0.0f;
			assignVector(&planeV, 1.0f, 1.0f, 0.0f, FALSE);
			assignVector(&rotAxisUV, 0.0f, 0.0f, 1.0f, TRUE);
			break;
		case ARC_PLANE_XZ :
			assignVector(&planeV, 1.0f, 0.0f, 1.0f, FALSE);
			assignVector(&rotAxisUV, 0.0f, 1.0f, 0.0f, TRUE);
			break;
		case ARC_PLANE_YZ :
			assignVector(&planeV, 0.0f, 1.0f, 1.0f, FALSE);
			assignVector(&rotAxisUV, 1.0f, 0.0f, 0.0f, TRUE); break;
		default:
			sprintf(_errorStr, "unknown ARC plane (%d)", (int)ARG_G, cylPtr->plane);
			ReportGcodeError(_errorStr);
			return(TRUE);
		}

		assignPoint(&pt1,       // interesection of rot axis and plane
				((ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits) * planeV.X),
				((ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits) * planeV.Y),
				((ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits) * planeV.Z));
		assignPoint(&pt2, 1000.0f * rotAxisUV.X, 1000.0f * normUV.Y, 1000.0f * normUV.Z);
		addPoints(&pt2, &pt1, &pt2);    //second point on rot axis, but far from plane.



		entrywiseVectorProduct(&tmp2, &tmp1, &plane);   // cancel out axis not in the plane

		entrywiseVectorSum
		assignVector(&planeV, 1.0f, 1.0f, 1.0f, FALSE);
		entrywiseVectorSum(&planeV, &planeV, &normUV);

		entrywiseVectorSum(&notNorm, &normUV, &minus1);
		scaleVector(&notNorm, &notNorm, -1.0f);


		switch(cylPtr->plane)
		{
		case ARC_PLANE_XY :
			assignVector(&cylPtr->rotationalAxisUV, 0.0f, 0.0f, 1.0f, TRUE);
			break;
		case ARC_PLANE_XZ :
			assignVector(&cylPtr->rotationalAxisUV, 0.0f, 1.0f, 0.0f, TRUE);
			break;
		case ARC_PLANE_YZ :
			assignVector(&cylPtr->rotationalAxisUV, 1.0f, 0.0f, 0.0f, TRUE); break;
		default:
			sprintf(_errorStr, "unknown ARC plane (%d)", (int)ARG_G, cylPtr->plane);
			ReportGcodeError(_errorStr);
			return(TRUE);
		}
	}
	else
	{   // find the rotational axis from the specified points
		if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING) {ReportGcodeError("in G16 mode: missing point 1 (XYZ)"); badArgs = TRUE;}
		if (ARG_I_MISSING && ARG_J_MISSING && ARG_K_MISSING) {ReportGcodeError("in G16 mode: missing point 2 (IJK)"); badArgs = TRUE;}

		assignPoint(&cylPtr->rotAxisPt1,
				(ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits),
				(ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits),
				(ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits));

		if (ARG_R_PRESENT && (ARG_R == 1.0f))
		{   // IJK are absolute coords
			assignPoint(&cylPtr->rotAxisPt2,
					(ARG_I_PRESENT ? ARG_I : Motors[M_X].Q_LastRequestedPositionInUnits)
					(ARG_J_PRESENT ? ARG_J : Motors[M_Y].Q_LastRequestedPositionInUnits),
					(ARG_K_PRESENT ? ARG_K : Motors[M_Z].Q_LastRequestedPositionInUnits));
		}
		else
		{   // IJK are incremental values to add to rotAxisPt1 to get rotAxisPt2
			pointStruct incr;   // temp point variable to deal with incremental values
			assignPoint(&incr,
					(ARG_I_PRESENT ? ARG_I : 0.0f),
					(ARG_J_PRESENT ? ARG_J : 0.0f),
					(ARG_K_PRESENT ? ARG_K : 0.0f));
			addPoints(&cylPtr->rotAxisPt2, &cylPtr->rotAxisPt1, &incr); // increment rotAxisPt1 to get rotAxisPt2
		}
		assignVector2Pts(&cylPtr->rotationalAxisUV, &cylPtr->rotAxisPt1, &cylPtr->rotAxisPt2, TRUE);
	}

	if (ARG_U_MISSING && ARG_V_MISSING && ARG_W_MISSING)
	{   // use default orientation if not specified
		assignVector(&cylPtr->liftUV, 0.0f, 0.0f, 1.0f, TRUE);
	}
	else //if (ARG_U_PRESENT || ARG_V_PRESENT || ARG_W_PRESENT)
	{   // case 2:  normal specified with U/V/W --- get vector and normalize
		assignVector(&cylPtr->liftUV, ARG_U_MISSING ? 0.0f : ARG_U, ARG_V_MISSING ? 0.0f : ARG_V, ARG_W_MISSING ? 0.0f : ARG_W, TRUE);
	}

	return(badArgs);
}
#endif
////////////////////////////////////////////////////////////////////////////////

boolean G702_getMoveVector(cylStruct *cylPtr)
{   // return true if failed

	boolean badArgs = FALSE;
	assignPoint(&cylPtr->startPt,
			(Motors[M_X].Q_LastRequestedPositionInUnits),
			(Motors[M_Y].Q_LastRequestedPositionInUnits),
			(Motors[M_Z].Q_LastRequestedPositionInUnits));
	cylPtr->rotStartPt = Motors[cylPtr->rotaryMotorAxisPtr->Axis].Q_LastRequestedPositionInUnits;

	float ARG_ROT = getMotorArg(cylPtr->rotaryMotorAxisPtr);
	boolean ROT_ARG_PRESENT = (ARG_ROT != INVALID_ARG_VALUE);

	if (_IncrementalMove)
	{   // XYZ are incremental values to add to startPt
		pointStruct incr;   // temp point variable to deal with incremental values
		assignPoint(&incr,
				(ARG_X_PRESENT ? ARG_X : 0.0f),
				(ARG_Y_PRESENT ? ARG_Y : 0.0f),
				(ARG_Z_PRESENT ? ARG_Z : 0.0f));
		addPoints(&cylPtr->destPt, &cylPtr->startPt, &incr);    // increment startPt to get destPt

		cylPtr->rotDestPt = cylPtr->rotStartPt + (ROT_ARG_PRESENT ? ARG_ROT : 0.0f);
	}
	else
	{   // XYZ are absolute coords
		assignPoint(&cylPtr->destPt,
				(ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits),
				(ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits),
				(ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits));

		cylPtr->rotDestPt = ROT_ARG_PRESENT ? ARG_ROT : Motors[cylPtr->rotaryMotorAxisPtr->Axis].Q_LastRequestedPositionInUnits;
	}

	assignVector2Pts(&cylPtr->moveV, &cylPtr->startPt, &cylPtr->destPt, FALSE);

	copyPoint(&cylPtr->currPt, &cylPtr->startPt);

	return(badArgs);
}

////////////////////////////////////////////////////////////////////////////////

int makeOdd(float valueF)
{
	return((int)valueF + ((((int)valueF & 0x01) == 0) ? 1 : 0));    // force to be odd and min of 1
}

////////////////////////////////////////////////////////////////////////////////

float determineCircumferenceBasedOnZ(cylStruct *cylPtr)
{
#ifdef GB_HIDDEN_WARNINGS
	int usingShortCut;
#endif //GB_HIDDEN_WARNINGS

	// desire :  mag((currPosV * planeV) - rotOfsV);

	float radius = Motors[M_Z].Q_LastRequestedPositionInUnits - cylPtr->rotaryMotorAxisPtr->RotaryOffset[M_Z];
	return(TWO_PI * radius);
}

////////////////////////////////////////////////////////////////////////////////

boolean cylCheckLength(cylStruct *cylPtr)
{
	float checkLength;
	boolean badArgs = FALSE;
	if (cylPtr->actualAngleInRadians < PI_OVER_TWO)
	{   // do some consistency checks
		checkLength = sqr(cylPtr->pathLength) - sqr(cylPtr->length) - sqr(cylPtr->rotLength);
	}
	else //if (cylPtr->actualAngleInRadians == PI_OVER_TWO)
	{
		checkLength = cylPtr->pathLength - cylPtr->rotLength;
	}

	if (fabsf(checkLength) > (0.001f * cylPtr->pathLength))  // allow error up to 0.1% of the path length
	{   //check: cylPtr->length^2 + cylPtr->rotLength^2 = cylPtr->pathLength^2
		sprintf(_errorStr, "math doesnt work (%3.2f %3.2f %3.2f %3.2f [%4.3f])",
				cylPtr->pathLength, cylPtr->length, cylPtr->rotLength, cylPtr->actualAngleInRadians * RADIANS_TO_DEGREES, checkLength);
		ReportGcodeError(_errorStr);
		badArgs = TRUE;
	}
	return(badArgs);
}

////////////////////////////////////////////////////////////////////////////////

void loadExecutionPtrGXYZRotE(int argG, pointStruct *p, float argRot, boolean incrementalRot, float argE)
{   // WARNING:  destination p.XYZ is in ABSOLUTE coords.   argRot might RELATIVE, so must be dealt with in order to
	// have a proper move.
	InvalidateAllCmdArgs(ExecutionPtr);
	ARG_G = (float)argG;
	if (argG == 1)
	{
		ARG_F = _cyl.argF;
		ARG_S = _cyl.flowScale;
	}
	ARG_X = p->X;
	ARG_Y = p->Y;
	ARG_Z = p->Z;
	ARG_E = (argE == 0.0f) ? INVALID_ARG_VALUE : argE;

	if (incrementalRot)
	{   // convert argRot from INCR to ASB position
		argRot += _cyl.rotaryMotorAxisPtr->Q_LastRequestedPositionInUnits;
	}
	setMotorArgInDeg(_cyl.rotaryMotorAxisPtr, argRot);

	if ((_sendingGBStringsMask & GB_STRING_CYL_INFO) && (_sendingGBStringsSubMask & GB_STRING_CYL_INFO_SUBMASK_MOVE_COORDS)) // use M797 S<mask> to enable
	{
		sprintf(_tmpStr, "CYL: %d G%d X%4.3f Y%4.3f Z%4.3f  %c%4.3f  I%d", CannedCycleStep, argG, p->X, p->Y, p->Z, _cyl.rotaryMotorAxisPtr->AxisLabel, argRot, (_IncrementalMove ? 1 : 0));
		sendGB(_tmpStr);
	}

	if (argG == 0)
		motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
	else
		motionQ_addCommand(VECTOR_MOTION_RATE, NOT_HOMING_MOTORS);

}

////////////////////////////////////////////////////////////////////////////////

void ProcessG702CylinderPrint(void)
{   // multi-part cylinder print
	//XYZ position is ALWAYS asbsolute location to loadExecutionPtrGXYZRotE
	// Rot can be sent either as absolute or rel (with flag)

	if (motionQ_hasSpace(2) && (CannedCycleStep > 0))
	{   // still more to print

		float dir = (_cyl.dir == DIRECTION_FORWARD) ? 1.0f : -1.0f;
		float rotDeg = dir * _cyl.rotDegrees;
		float jogDeg = dir * _cyl.jogDegrees;

		if ((_cyl.type == CYLINDER_ANGLED) || (_cyl.type == CYLINDER_PARALLEL))
		{
			if (CannedCycleStep & 1)
			{   // ODD, so head to dest tp
				loadExecutionPtrGXYZRotE(1, &_cyl.destPt, rotDeg, TRUE, _cyl.argE);
				if (CannedCycleStep > 1)
				{   // jog except after last pass)
					loadExecutionPtrGXYZRotE(0, &_cyl.destPt, jogDeg, TRUE, 0.0f);
				}
			}
			else
			{   // EVEN, so head to start pt
				loadExecutionPtrGXYZRotE(1, &_cyl.startPt, -1.0f * rotDeg, TRUE, _cyl.argE);
				loadExecutionPtrGXYZRotE(0, &_cyl.startPt, jogDeg, TRUE, 0.0f);
			}
			CannedCycleStep--;
		}
		else if (_cyl.type == CYLINDER_PERPENDICULAR)
		{   // special case for 90 ... will take a series of separate moves to get across the cylinder
			// enter thsi code sitting at "start" of next circular ring ... so:
			//      rotate cyclinder (no lateral move) -- relative move
			//      if (laps_to_go == 1)
			//          need to get to dest point plus any rot jog
			//      else
			//          need to offset 1 actualPitch plus any rot jog


			loadExecutionPtrGXYZRotE(1, &_cyl.currPt, rotDeg, TRUE, _cyl.argE);
			CannedCycleStep--;

			if (CannedCycleStep > 0)
			{   // will still work to do, so go ahead and jog to next start point
				if (CannedCycleStep == 1)
				{   // will be last lap, so force position to destination
					copyPoint(&_cyl.currPt, &_cyl.destPt);
				}
				else
				{   // better to calc from start (or end) to minimize error than to just incrementally offset
					_cyl.currPt.X = _cyl.startPt.X + (_cyl.passes - CannedCycleStep) * _cyl.perpOfs.X; // next lap starting point
					_cyl.currPt.Y = _cyl.startPt.Y + (_cyl.passes - CannedCycleStep) * _cyl.perpOfs.Y; // next lap starting point
					_cyl.currPt.Z = _cyl.startPt.Z + (_cyl.passes - CannedCycleStep) * _cyl.perpOfs.Z; // next lap starting point
				}
				loadExecutionPtrGXYZRotE(0, &_cyl.currPt, jogDeg, TRUE, 0.0f);  // move to next start, including rot jog if needed
			}
		}
		else
		{
			barf("unknown cylinder type");
		}
	}
	else if (motionQ_hasSpace(1) && (CannedCycleStep == 0))
	{   // doen with making the cylinder, so do any final position move
		// including the lift and the rotation to the final requested location

		pointStruct liftDest;
		liftDest.X = _cyl.destPt.X + (_cyl.liftUV.X * _cyl.liftAmount);
		liftDest.Y = _cyl.destPt.Y + (_cyl.liftUV.Y * _cyl.liftAmount);
		liftDest.Z = _cyl.destPt.Z + (_cyl.liftUV.Z * _cyl.liftAmount);
		float deltaRot = _cyl.rotDestPt - _cyl.rotaryMotorAxisPtr->Q_LastRequestedPositionInUnits;
		if (deltaRot > 180.0f)
			deltaRot -= 360.0f;
		else if (deltaRot < -180.0f)
			deltaRot += 360.0f;
		loadExecutionPtrGXYZRotE(0, &liftDest, deltaRot, TRUE, 0.0f);

		if (_cyl.liftAmount)
		{   // only send notice if lifting (changing layer)
			SendFakeMcodeExecutionNotice(790, INVALID_ARG_VALUE, INVALID_ARG_VALUE, INVALID_ARG_VALUE); // fake repetrel into thinking layer is complete
		}

		// done with cylinder, so restore settings that were altered.
		_IncrementalMove = _cyl.save_IncrementalMove;
		_extrusionControl = _cyl.save_extrusionControl;
		CannedCycleFlag = 0;    // done with cylinder
	}
	else
	{   // waiting on room in the queue
		;
	}
}

////////////////////////////////////////////////////////////////////////////////

#if 0 //NUKE?? save for now until 3D pipe finished
void G_Code_G702_X(boolean flipDir) // Setup for the general multi-layer three-space rotary surface print (uses ABCDFIJKLOPRUVWXYZ)
{
	// GCODE G702.1X<A|B|C rotAxis> <XYZ point_1> <IJK point_2> [R relative] <UVW normal> <O offset> [D diam] [F feedrate] [P pitch] [L lift]
	// GCODE
	// GCODE    argABC - required rotary axis selection
	// GCODE    if a G17/G18/G19 plane is enabled -- then the rotational axis is specified by G17/G18/G19 (perpendicular to the plane)
	// GOCDE
	// GCODE        argXYZ - optional intersection point of the axis of rotation definition and the specified plane
	// GCODE                 (if not specified, then current position is used)
	// GCODE    else if M235 is used to set plane
	// GCODE        ;
	// GCODE    else two points are needed to determine the axis of rotation (allows any three space vector to work
	// GCODE        argXYZ - required axis of rotation definition: if not G17/G18/G19, then point 1 is required
	// GCODE                 (absolute position of point 1 on the axis of rotation line)
	// GCODE                 (this will also serve as the default edge 1 of the print)
	// GCODE        argIJK - required axis of rotation definition: point 2  (if R==1 IJK are incremental from point1; otherwise IJK are an absolute position)
	// GCODE                 (this will also serve as the default edge 2 of the print)
	// GCODE        argR   - optional flag (R==1) to mark IJK are Absolute values from point; otherwise IJK are incremental position
	// GCODE    argUVW - optional normal vector perpendicular and away from the axis of rotation (def (0,0,1)
	// GCODE    argO   - initial offset (perpendicular distance) from the current position to the rotational axis along the UVW normal
	// GCODE             to the printing height above build surface
	// GCODE    argF   - optional change to the persistent feedrate
	// GCODE    argP   - optional (persistant) pitch between lines when traversing the build surface in that layer (useful for rafts)
	// GCODE                 if argP missing or argP <= 0; use the active head's defined nozzle width is used
	// GCODE                 else use argP as the pitch between adjacent lines on the current print surface
	// GCODE    argL   - optional (persistant) lift amount along the UVW normal at the end of the surface print
	// GCODE                 if argL missing or argL == 0; no lift is performed at the end of the current surface print
	// GCODE                 else use argL as the lift amount along the UVW normal at the end of the current surface print
	// GCODE
	// GCODE    Setup for the general multi-layer three-space rotary surface print

	boolean badArgs = FALSE;
	cylStruct *cylPtr = &_cyl;
	initCylStruct(cylPtr);  // reset structure and fill in defaults

	if (ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) {ReportGcodeError("missing rotary axis selection (ABC)"); badArgs = TRUE;}
	badArgs |= G702_checkRotaryAxisArg(cylPtr, ARG_A, M_A);
	badArgs |= G702_checkRotaryAxisArg(cylPtr, ARG_B, M_B);
	badArgs |= G702_checkRotaryAxisArg(cylPtr, ARG_C, M_C);
	badArgs |= G702_setupNormals(cylPtr);
	if (ARG_O_MISSING) {ReportGcodeError("missing offet from axis of rotation to build surface (O)"); badArgs = TRUE;}
	if (badArgs) { barf("G702.1 argument errors detected");  return; }

	// if processing gets this far, args are ok, so safely proceed

	cylPtr->surfaceOffset   = ARG_O;
	cylPtr->desiredFeedrate = ARG_F_PRESENT ? ARG_F : _TargetFeedRateInMmPerSec;  //CheckForNewFeedrate(TRUE);
	cylPtr->requestedPitch  = ARG_P_PRESENT ? ARG_P : getInboxPointer(currentOutboxPtr->device)->ExtrusionControl.ExtrusionWidthInMm;
	cylPtr->liftAmount      = ARG_L_PRESENT ? ARG_L : 0.0f;
}
////////////////////////////////////////////////////////////////////////////////
#endif

////////////////////////////////////////////////////////////////////////////////

void G_Code_G702_1(boolean flipDir) // Setup for the general multi-layer primary plane rotary surface print (uses OPRSXYZ)
{
	// GCODE G702.1 <"R" rotaryMotorIndex> <"P" parallelAxisIndex> <"X" offset> <"Y" offset> <"Z" offset> (for I==90):[O overlapRed [S alignSeam]]
	// GCODE
	// GCODE    argXYZ - required offset to the parallel axis in the perpendicular plane
	// GCODE    argR   - required index of the rotary motor (ie B=4)
	// GCODE    argP   - required index of the parallel axis (ie, X=0);  only X and Y supported
	// GCODE
	// GCODE    for the special case of a G702/G703 cylinder build that is perpendicular to the axis of rotation (S90)
	// GCODE        argO -  amount to reduce the path around the circumference to prevent overlapping end points (and a bulge)
	// GCODE        argS - if argS=1, then the seem on the build will be aligned to the axis of rotation
	// GCODE               otherwise, it will drift based on the the argO amount per "lap"
	// GCODE
	// GCODE    Setup for the general multi-layer three-space rotary surface print

	// NOTEL flipDir is ignored.  same setup for 702 and 703

	boolean badArgs = FALSE;
	cylStruct *cylPtr = &_cyl;
	initCylStruct(cylPtr);  // reset structure and fill in defaults


	cylPtr->overlapReduction    = ARG_O_PRESENT ? ARG_O : 0.0f;
	ARG_O = INVALID_ARG_VALUE;
	cylPtr->alignedSeam         = (ARG_S_PRESENT && (ARG_S == 1.0f)) ? TRUE : FALSE;
	ARG_S = INVALID_ARG_VALUE;

	M_Code_M235(); // process args based on same M235 controls.

	if ((_M235.rotaryMotorAxisPtr == NULL) || (_M235.axisParallelToRotaryAxisPtr == NULL))
		{ ReportGcodeError("Improper setup"); badArgs = TRUE;}

	// if processing gets this far, args are ok, so safely proceed

	cylPtr->rotaryMotorAxisPtr = _M235.rotaryMotorAxisPtr;
	cylPtr->liftUV = _M235.liftUV;
	if (_M235.axisParallelToRotaryAxisPtr->Axis == M_X)
		assignVector(&cylPtr->rotUV, 1.0f, 0.0f, 0.0f, TRUE);
	else if (_M235.axisParallelToRotaryAxisPtr->Axis == M_X)
		assignVector(&cylPtr->rotUV, 0.0f, 1.0f, 0.0f, TRUE);
	else if (_M235.axisParallelToRotaryAxisPtr->Axis == M_X)
		assignVector(&cylPtr->rotUV, 0.0f, 0.0f, 1.0f, TRUE);
	else
		{ ReportGcodeError("invalid M235.axisParallelToRotaryAxisPtr"); badArgs = TRUE;}

	if (badArgs) { barf("G702.1 argument errors detected");  return; }
}

////////////////////////////////////////////////////////////////////////////////

void cylChangeDir(cylStruct *cylPtr)
{
	if (cylPtr->dir == DIRECTION_REVERSE)
		cylPtr->dir =DIRECTION_FORWARD;
	else
		cylPtr->dir =DIRECTION_REVERSE;
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G702_0(boolean flipDir) // Execute a single layer three-space rotary surface print (uses ABCEFHILPSWXYZ)
{
	// GCODE G702 <XYZA|B|C dest> <I inclineAngle> [P pitch] [K king] [F rate] [E extrude] [L lift] [H toolIndex] [S oneTimeFlowScale]
	// GCODE G702 <XYZA|B|C dest> <I inclineAngle> <W wraps>          [F rate] [E extrude] [L lift] [H toolIndex] [S oneTimeFlowScale]
	// GCODE G702 <XYZA|B|C dest> <W wraps> [P pitch]                 [F rate] [E extrude] [L lift] [H toolIndex] [S oneTimeFlowScale]
	// GCODE
	// GCODE    there are three ways to define the cylindrical layer:
	// GCODE        (1) state the angle and pitch  <argI> [argP] [argK]; wraps will be auto-calculated.
	// GCODE            angle or pitch adjusted based on argK.  pitch HAS to be adjusted in the 0 and 90 angle cases.
	// GCODE        (2) state number of angle and wraps  <argI> <argW>; pitch will be auto-calculated
	// GCODE        (3) state number of wraps and pitch  <argW> [argP]; angle will be auto-calculated
	// GCODE
	// GCODE    argXYZ - required destination point. can also include dest position for rotational axis  (G90/G91 in play))
	// GCODE    argI   - required offset angle for the path of motion between along the current print surface OR
	// GCODE    argW   - required number of wraps/passes ("parallel lines") around the cylinder.  must be an odd number >= 1
	// GCODE    argP   - optional pitch between lines when traversing the build surface.
	// GCODE                 if argP missing or argP <= 0; then the active head's defined nozzle width is used as the requested pitch
	// GCODE                 else use argP as the pitch between adjacent lines on the current print surface
	// GCODE    argK   - simple flag (K=1 or K=anything else) for the case where angle and pitch are provided.
	// GCODE             if K=1 is given, then "inclineAngle" is king and the requested pitch will be modified to honor the requested the requested angle.
	// GCODE             if no K is given or K != 1, then "pitch" is king and the requested angle is modified to honor the requested pitch.
	// GCODE    argF   - optional change to the persistent feedrate
	// GCODE    argE   - optional flag to indicate auto-calculated extrusion should occur for this surface (any E present)
	// GCODE    argL   - optional (persistant) lift amount (UVW print normal must have been specified during setup)
	// GCODE                 if argL missing or argL == 0; no lift is performed at the end of the current surface print
	// GCODE                 else use argL as the lift amount in the direction of UVW normal at the end of the current surface print
	// GCODE    argH   - optional change to the persistent toolindex
	// GCODE
	// GCODE        NOTE: an odd number of parallel print lines (wraps) are always used to create the surface in order to finish at the dest
	// GCODE        NOTE: a special case exists if the S angle results in desired number of parallel lines is less than or equal to 1.  in this
	// GCODE              a single move is performed wrapping the cylinder at the requested pitch (like a single thread)
	// GOCDE        NOTE: if S=0, then the surface is covered with discrete lines running parallel to the axis of the cylinder
	// GCODE        NOTE: if S=90, then surface is covered with with discrete lines running perpendiclar to the axis of the cylinder
	// GCODE
	// GCODE    Execute a single layer three-space rotary surface print

	boolean badArgs = FALSE;
	cylStruct *cylPtr = &_cyl;

	cylPtr->dir = (flipDir == FALSE) ? DIRECTION_FORWARD : DIRECTION_REVERSE;   // starting dir based on 702 vs 703

	cylPtr->circumference = determineCircumferenceBasedOnZ(cylPtr); // distance for curr position to axis of rotation

	badArgs |= G702_getMoveVector(cylPtr);
	cylPtr->length = cylPtr->moveV.mag;
	if (cylPtr->length < CYL_TOLERANCE) { ReportGcodeError("width too small"); badArgs = TRUE; }

#ifdef GB_HIDDEN_WARNINGS
	int finishFor3dCase;
	//badArgs |= distanceBetweenParallelVectors(&cylPtr->surfaceOffset, &cylPtr->rotationalAxisV, &cylPr->moveV);
#endif //GB_HIDDEN_WARNINGS

	if (ARG_I_PRESENT && ARG_W_PRESENT && ARG_P_PRESENT) { ReportGcodeError("overconstrained, spec at most two of I, W, and P"); badArgs = TRUE; }
	if (ARG_I_MISSING && ARG_W_MISSING) { ReportGcodeError("need to specify at least one of I or W"); badArgs = TRUE; }
	if (ARG_H_PRESENT && (((int)ARG_H < 0) || ((int)ARG_H >= NUM_TOOL_OFFSETS))) { ReportInvalidGcodeHArg(); badArgs = TRUE; }

	cylPtr->angleIsKing     = ARG_K_PRESENT ? ((ARG_L == 1.0f) ? TRUE : FALSE) : FALSE;
	cylPtr->argF            = ARG_F;    // will be passed along to moves
	cylPtr->argE            = ARG_E;    // will be passed along to non jog-moves
	cylPtr->liftAmount      = ARG_L_PRESENT ? ARG_L : 0.0f;
	currentToolIndex        = ARG_H_PRESENT ? (int)ARG_H : currentToolIndex;
	cylPtr->flowScale       = ARG_S;

	if (ARG_W_PRESENT && (ARG_W < 0.0f))  { ARG_W = -1.0f * ARG_W; cylChangeDir(cylPtr); }  // neg val means go oppo dir
	if (ARG_I_PRESENT && (ARG_I < 0.0f))  { ARG_I = -1.0f * ARG_I; cylChangeDir(cylPtr); }  // neg val means go oppo dir
	if (ARG_P_PRESENT && (ARG_P < 0.0f))  { ARG_P = -1.0f * ARG_P; cylChangeDir(cylPtr); }  // neg val means go oppo dir

	if (ARG_W_PRESENT && ARG_I_PRESENT)
	{   // user specified number of wraps (W) and desired angle (S), so derive the pitch
		// simple geo:  hyp = edge length along circum based on W wraps;   adj = pitch.  cos(S) = adj / hyp;
		cylPtr->requestedAngleInRadians = ARG_I * DEGREES_TO_RADIANS;
		cylPtr->requestedPitch = cos(cylPtr->requestedAngleInRadians) * (cylPtr->circumference / ARG_W);
	}
	else if (ARG_W_PRESENT)
	{   // user specified number of wraps (W) and the pitch - either explicitly (P) or implicitly from head settings, so derive the angle;
		// simple geo:  hyp = edge length along circum based on W wraps;   adj = pitch.  cos(S) = adj / hyp;
		cylPtr->requestedPitch  = ARG_P_PRESENT ? ARG_P : currentOutboxPtr->ExtrusionControl.ExtrusionWidthInMm;
		cylPtr->requestedAngleInRadians = acos(cylPtr->requestedPitch / (cylPtr->circumference / ARG_W));
	}
	else
	{   // use specified the angle (S) and the pitch - either explicitly (P) or implicitly from head settings, so derive the angle;
		cylPtr->requestedAngleInRadians = ARG_I * DEGREES_TO_RADIANS;
		cylPtr->requestedPitch  = ARG_P_PRESENT ? ARG_P : currentOutboxPtr->ExtrusionControl.ExtrusionWidthInMm;
	}

	if (fabsf(cylPtr->requestedPitch) <= 0.05f) { ReportGcodeError("Pitch too small (under 0.05)"); badArgs = TRUE; }
	if (cylPtr->requestedAngleInRadians < MINUS_PI_OVER_TWO) { ReportGcodeError("Angle less than 90 degrees"); badArgs = TRUE; }
	if (cylPtr->requestedAngleInRadians > PI_OVER_TWO) { ReportGcodeError("Angle greater than 90 degrees"); badArgs = TRUE; }

	//////////////////////////////

	if (cylPtr->requestedAngleInRadians == PI_OVER_TWO)
	{   // special case of running perpendicular to the axis of motion
		cylPtr->type = CYLINDER_PERPENDICULAR;
		cylPtr->passes = makeOdd(cylPtr->length / cylPtr->requestedPitch);
		cylPtr->actualPitch = cylPtr->length / (float)cylPtr->passes;   //recalc pitch based on filling surface with odd # integral passes
		cylPtr->actualAngleInRadians = PI_OVER_TWO;

		// special case for the perpendicular offset
		cylPtr->perpOfs.X = cylPtr->rotUV.X * cylPtr->actualPitch * (((cylPtr->destPt.X - cylPtr->startPt.X) < 0.0f) ? -1.0f : 1.0f);
		cylPtr->perpOfs.Y = cylPtr->rotUV.Y * cylPtr->actualPitch * (((cylPtr->destPt.Y - cylPtr->startPt.Y) < 0.0f) ? -1.0f : 1.0f);
		cylPtr->perpOfs.Z = cylPtr->rotUV.Z * cylPtr->actualPitch * (((cylPtr->destPt.Z - cylPtr->startPt.Z) < 0.0f) ? -1.0f : 1.0f);

		cylPtr->rotLength = cylPtr->circumference - cylPtr->overlapReduction;   // avoid overlap of endpoints
		cylPtr->rotDegrees = (cylPtr->rotLength / cylPtr->circumference) * 360.0f;
		cylPtr->jogLength = (cylPtr->alignedSeam) ? cylPtr->overlapReduction : 0.0f;
		cylPtr->jogDegrees = (cylPtr->jogLength / cylPtr->circumference) * 360.0f;
		cylPtr->pathLength = cylPtr->rotLength; // actual distance to travel per pass
		cylPtr->totalTravel = cylPtr->passes * cylPtr->pathLength;                                              // total of all paths
		cylPtr->totalTravel += (cylPtr->passes - 1) * fpu_sqrtf(sqr(cylPtr->jogLength) + sqr(cylPtr->actualPitch)); // plus all the jogs
	}
	else if (cylPtr->requestedAngleInRadians == 0.0f)
	{   // special case of running parallel to the axis of rotation
		cylPtr->type = CYLINDER_PARALLEL;
		cylPtr->passes = makeOdd(cylPtr->circumference / cylPtr->requestedPitch);
		cylPtr->actualPitch = cylPtr->circumference / (float)cylPtr->passes;    //recalc pitch based on filling surface with odd # integral passes
		cylPtr->actualAngleInRadians = 0.0f;
		cylPtr->rotLength = 0.0f;
		cylPtr->rotDegrees = 0.0f;
		cylPtr->jogLength = cylPtr->actualPitch;
		cylPtr->jogDegrees = (cylPtr->jogLength / cylPtr->circumference) * 360.0f;
		cylPtr->pathLength = cylPtr->length / cos(cylPtr->actualAngleInRadians); // actual distance to travel per pass
		cylPtr->totalTravel = cylPtr->passes * cylPtr->pathLength;      // total of all paths
		cylPtr->totalTravel += (cylPtr->passes -1) * cylPtr->jogLength; // plus all the jogs
	}
	else
	{   // somewhere in between 0 and 90 degrees...
		cylPtr->type = CYLINDER_ANGLED;
		cylPtr->passes = makeOdd(cylPtr->circumference / (cylPtr->requestedPitch / cos(cylPtr->requestedAngleInRadians)));
		if (cylPtr->angleIsKing)
		{   // modify the pitch to preserve the requested angle
			cylPtr->actualAngleInRadians = cylPtr->requestedAngleInRadians;
			cylPtr->actualPitch = (cylPtr->circumference / (float)cylPtr->passes) * cos(cylPtr->actualAngleInRadians);
		}
		else
		{   // modify the angle to preserve the requested pitch
			cylPtr->actualPitch = cylPtr->requestedPitch;
			cylPtr->actualAngleInRadians = acos(cylPtr->actualPitch / (cylPtr->circumference / (float)cylPtr->passes));
		}
		cylPtr->rotLength = cylPtr->length * tan(cylPtr->actualAngleInRadians);
		cylPtr->rotDegrees = (cylPtr->rotLength / cylPtr->circumference) * 360.0f;
		cylPtr->jogLength = cylPtr->actualPitch / cos(cylPtr->actualAngleInRadians);
		cylPtr->jogDegrees = (cylPtr->jogLength / cylPtr->circumference) * 360.0f;
		cylPtr->pathLength = cylPtr->length / cos(cylPtr->actualAngleInRadians); // actual distance to travel per pass
		cylPtr->totalTravel = cylPtr->passes * cylPtr->pathLength;      // total of all paths
		cylPtr->totalTravel += (cylPtr->passes - 1) * cylPtr->jogLength;    // plus all the jogs
	}

	if (badArgs) { barf("G702 argument errors detected");  return; }

	// at this point, all info is known to kick off the multi-move routine for form the specified cylinder

	_cyl.save_IncrementalMove = _IncrementalMove;
	_cyl.save_extrusionControl = _extrusionControl;

	_IncrementalMove = FALSE;   // force absolute
	_extrusionControl = IGNORE_E_VALUES;

	CannedCycleFlag = 702;
	CannedCycleStep = cylPtr->passes;
	CannedCyclePtr = ExecutionPtr;

	if ((_sendingGBStringsMask & GB_STRING_CYL_INFO) && (_sendingGBStringsSubMask & GB_STRING_CYL_INFO_SUBMASK_MOVE_SETUP)) // use M797 S<mask> to enable
	{
		sendGB("");
		sprintf(_tmpStr, "CYLINDER:  (passes: %d   circumference: %3.2f  I%d)", _cyl.passes, _cyl.circumference, (_IncrementalMove ? 1 : 0)); sendGB(_tmpStr);
		sprintf(_tmpStr, "    area:    cyl:  %9.3f     prt: %9.3f", _cyl.circumference*_cyl.length, (float)_cyl.passes*(_cyl.pathLength*_cyl.actualPitch)); sendGB(_tmpStr);
		sprintf(_tmpStr, "    pitch:   req:  %9.3f     act: %9.3f", _cyl.requestedPitch, _cyl.actualPitch); sendGB(_tmpStr);
		sprintf(_tmpStr, "    angle:   req:  %9.3f     act: %9.3f", _cyl.requestedAngleInRadians*RADIANS_TO_DEGREES, _cyl.actualAngleInRadians*RADIANS_TO_DEGREES); sendGB(_tmpStr);
		sprintf(_tmpStr, "    length:  cart: %9.3f     rot: %9.3f", _cyl.length, _cyl.rotLength); sendGB(_tmpStr);
		sprintf(_tmpStr, "    rotate:  pass: %9.3f     jog: %9.3f", _cyl.rotDegrees, _cyl.jogDegrees); sendGB(_tmpStr);
		sprintf(_tmpStr, "    travel:  pass: %9.3f     jog: %9.3f", _cyl.pathLength, _cyl.jogLength); sendGB(_tmpStr);
		sprintf(_tmpStr, "    total travel:  %9.3f", _cyl.totalTravel); sendGB(_tmpStr);
		sprintf(_tmpStr, "    start: (%4.3f, %4.3f, %4.3f)", _cyl.startPt.X, _cyl.startPt.Y, _cyl.startPt.Z); sendGB(_tmpStr);
		sprintf(_tmpStr, "    dest:  (%4.3f, %4.3f, %4.3f)", _cyl.destPt.X, _cyl.destPt.Y, _cyl.destPt.Z); sendGB(_tmpStr);
		sendGB("");
	}
}

////////////////////////////////////////////////////////////////////////////////

void G_Code_G702(boolean flipDir)   // G702 master command
{
	// GCODE G702          - Execute a single layer three-space rotary surface print
	// GCCDE G702.1        - Setup for the general multi-layer three-space rotary surface print
	// GCODE
	// GCODE EXAMPLE:
	// GCODE    Assume: Axis B is set up as a ROTARY axis with steps/unit as steps/degree (axis must have
	// GCODE            infinite rotation, no physical limits or sensors)
	// GCODE    Assume: Axis B is aligned such that it is rotating in the YZ plane, rotating about a line
	// GCODE            parallel to the X axis that intersects the YZ the plane at Y=40, Z=30.
	// GCODE    Assume: Mandral is 200 mm long and 1" (25.4 mm) in diameter and is between X50 and X250
	// GCODE    Assume: Normal print head orientation (along Z axis)
	// GCODE    Assume: Desire constant material flow based on current feedrate at a diameter of 40mm
	// GCODE    Assume: Default line pitch is the defined nozzle with and layer height is 0.3
	// GCODE
	// GCODE        G702.1 B1  X0   I100   U0 V0 W1  O12.7  D40  P0  setup rot axis, center line of rotation, mandral size, and normal to print head
	// GCODE
	// GCODE        G0 Z43 ; height of first to print is 30 (Z component of axis of rotation) + 12.7 (radius of mandral) + 0.3 (layer height)
	// GCODE        G0 Y40 ; position over/perpendicular to the center of rotation
	// GCODE        G0 X70 ; starting 20 mm inside of the edge of the mandral
	// GCODE
	// GCODE        G702 X70 I20 S45 L0.25     ; print a solid cylinder 20mm wide from X70 to X90 wide with parallel
	// GCODE                                   ; print lines at a 45 degree angle.  lift head 0.25 in Z when done
	// GCODE        G702 S-45 L0.25            ; print a new layer 20mm wide with print lines at a -45 degree angle lift when done
	// GCODE        G702 S0 L0.25              ; print a new layer with print lines parallel to the axis of rotation. lift when done
	// GCODE        G702 S89.9 L0.25           ; print a new layer with a single rotary pass across the surface (like a fine pitch thread). lift when done
	// GCODE        G702 S90                   ; print a complete cylinder 20mm wide with passes at a 90 degree angle (perpendicular to the X-axis (SPECIAL CASE)
	// GCODE        G702 S0 P1.2               ; lay down roughly 50% sparse raft (helpful for soluable support materal to allow solvent in
	// GCODE        G702 S45 P1.2              ; lay down roughly 50% sparse raft
	// GCODE        G702 S-45 P1.2             ; lay down roughly 50% sparse raft
	// GCODE        G702 S0 P0                 ; lay down a solid layer of soluable support to improve surface finish of actual part sparse raft

	switch (getArgFraction(ARG_G))
	{
	case 0: G_Code_G702_0(flipDir); break;      // rotary cylinder layer print
	case 1: G_Code_G702_1(flipDir); break;      // rotary cylinder print setup
	default: ReportUnknownFractionalGcode(); break;
	}
}

////////////////////////////////////////////////////////////////////////////////

