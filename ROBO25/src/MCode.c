////////////////////////////////////////////////////////////////////////////////
//
// File:    Mcode.c
//
////////////////////////////////////////////////////////////////////////////////
//
// Purpose: Mcode processing functions called by the Command Processor
//
//          Each MCODE has zero or more reqired parameters and zero or more
//          optional parameters.   For the Hyrel system, the following are the
//          general guidelines for parameter definitions
//
//              X - X axis position (relative or absolute)
//              Y - Y axis position (relative or absolute)
//              Z - Z axis position (relative or absolute)
//              A - A axis position (relative or absolute)
//              B - B axis position (relative or absolute)
//              C - C axis position (relative or absolute)
//              C - Spindle Feed rate (or generally used for "rate" situations)
//              F - Vector feed rate
//              E - Extrusion amount
//              T - Tool selection (generally used to select a given hothead or group of hotheads)
//              S - S is a general purpose argument.
//              I - used in G2/G3 arc commands
//              J - used in G2/G3 arc commands
//              D - tool diameter
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "GCode.h"
#include "Serial.h"
#include "mailbox.h"
#include "Hydra_can.h"
#include "gpio.h"
#include "MotorDriver.h"
#include "HardwareInit.h"
#include "bootloader.h"  //NUKE? GB XXX gregkarl -- can we strip out the HH bootloader code?
#include "usbd_core.h"
#include "gui.h"
#include "pnp.h"
#ifdef HYDRA_DIAGS
#include "diags.h"
#endif

////////////////////////////////////////////////////////////////////////////////
//  Local #defines (defines ONLY used in this module)
////////////////////////////////////////////////////////////////////////////////

#ifdef NEW_MCODE_TOOL_USE
typedef enum {
	MCODE_TARG_IS_CANBUS_ADDRESS = 1,
	MCODE_TARG_IS_TOOL_NUMBER = 2,
	MCODE_TARG_IS_CANBUS_ADDRESS_OR_TOOL_NUMBER = 3,
} targMode_t;
targMode_t _mcodeModeForArgT = MCODE_TARG_IS_CANBUS_ADDRESS;
#endif

float _M106_fanDutyCycleRange = 100.0f; // allow interpretation of  Slic3r/Marlin 0 to 255 M106 range
M235Struct _M235;

////////////////////////////////////////////////////////////////////////////////
//  Public global definitions (exposed in MCode.h)
////////////////////////////////////////////////////////////////////////////////

// default head control address,if the current move is a printing move, this will active the same output as M7
byte PersistantUltimusControlHeadAddress=HH_POSITION_UNPLUGGED;

////////////////////////////////////////////////////////////////////////////////
//  Local global definitions (do not expose in MCode.h)
////////////////////////////////////////////////////////////////////////////////

byte PersistantHotheadAddress=DEFAULT_HOT_HEAD;
byte PersistantHotbedAddress=DEFAULT_HOT_BED;
uint16_t DesiredCo2LaserPower = 0;

////////////////////////////////////////////////////////////////////////////////
//  Forward declarations - any local modules needing an early template
////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
void ReportMcodeError(char *s)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "MCODE %d -- %s", (int) ARG_M, s);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportInvalidMcodeArg(char *arg, float val)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "MCODE %d -- Invalid %s arg (%4.4f)", (int)ARG_M, arg, val);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportInvalidMcodeArgInt(char *arg, uint32_t val)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "MCODE %d -- Invalid %s arg (%d/0x%08x)", (int)ARG_M, arg, (int)val, (int)val);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportMissingMcodeArg(char *arg)
{
	char localStr[MAX_STRING_SIZE];
	sprintf(localStr, "MCODE %d -- Missing %s arg (required)", (int)ARG_M, arg);
	sendError(localStr);
}

////////////////////////////////////////////////////////////////////////////////

void ReportMissingMcodeAArg(void)  { ReportMissingMcodeArg("A"); }
void ReportMissingMcodeBArg(void)  { ReportMissingMcodeArg("B"); }
void ReportMissingMcodeCArg(void)  { ReportMissingMcodeArg("C"); }
void ReportMissingMcodeDArg(void)  { ReportMissingMcodeArg("D"); }
void ReportMissingMcodeEArg(void)  { ReportMissingMcodeArg("E"); }
void ReportMissingMcodeFArg(void)  { ReportMissingMcodeArg("F"); }
void ReportMissingMcodeGArg(void)  { ReportMissingMcodeArg("G"); }
void ReportMissingMcodeHArg(void)  { ReportMissingMcodeArg("H"); }
void ReportMissingMcodeIArg(void)  { ReportMissingMcodeArg("I"); }
void ReportMissingMcodeJArg(void)  { ReportMissingMcodeArg("J"); }
void ReportMissingMcodeKArg(void)  { ReportMissingMcodeArg("K"); }
void ReportMissingMcodeLArg(void)  { ReportMissingMcodeArg("L"); }
void ReportMissingMcodeMArg(void)  { ReportMissingMcodeArg("M"); }
void ReportMissingMcodeNArg(void)  { ReportMissingMcodeArg("N"); }
void ReportMissingMcodeOArg(void)  { ReportMissingMcodeArg("O"); }
void ReportMissingMcodePArg(void)  { ReportMissingMcodeArg("P"); }
void ReportMissingMcodeQArg(void)  { ReportMissingMcodeArg("Q"); }
void ReportMissingMcodeRArg(void)  { ReportMissingMcodeArg("R"); }
void ReportMissingMcodeSArg(void)  { ReportMissingMcodeArg("S"); }
void ReportMissingMcodeTArg(void)  { ReportMissingMcodeArg("T"); }
void ReportMissingMcodeUArg(void)  { ReportMissingMcodeArg("U"); }
void ReportMissingMcodeVArg(void)  { ReportMissingMcodeArg("V"); }
void ReportMissingMcodeWArg(void)  { ReportMissingMcodeArg("W"); }
void ReportMissingMcodeXArg(void)  { ReportMissingMcodeArg("X"); }
void ReportMissingMcodeYArg(void)  { ReportMissingMcodeArg("Y"); }
void ReportMissingMcodeZArg(void)  { ReportMissingMcodeArg("Z"); }

void ReportMissingMcodeSEArg(void)  { ReportMissingMcodeArg("SE"); }
void ReportMissingMcodeSPArg(void)  { ReportMissingMcodeArg("SP"); }
void ReportMissingMcodeSRArg(void)  { ReportMissingMcodeArg("SR"); }
void ReportMissingMcodeFPArg(void)  { ReportMissingMcodeArg("FP"); }
void ReportMissingMcodeFSArg(void)  { ReportMissingMcodeArg("FS"); }
void ReportMissingMcodePIArg(void)  { ReportMissingMcodeArg("PI"); }
void ReportMissingMcodeEPSArg(void)  { ReportMissingMcodeArg("EPS"); }
void ReportMissingMcodeSRPArg(void)  { ReportMissingMcodeArg("SRP"); }

////////////////////////////////////////////////////////////////////////////////

void ReportInvalidMcodeAArg(void)  { ReportInvalidMcodeArg("A", ARG_A); }
void ReportInvalidMcodeBArg(void)  { ReportInvalidMcodeArg("B", ARG_B); }
void ReportInvalidMcodeCArg(void)  { ReportInvalidMcodeArg("C", ARG_C); }
void ReportInvalidMcodeDArg(void)  { ReportInvalidMcodeArg("D", ARG_D); }
void ReportInvalidMcodeEArg(void)  { ReportInvalidMcodeArg("E", ARG_E); }
void ReportInvalidMcodeFArg(void)  { ReportInvalidMcodeArg("F", ARG_F); }
void ReportInvalidMcodeGArg(void)  { ReportInvalidMcodeArg("G", ARG_G); }
void ReportInvalidMcodeHArg(void)  { ReportInvalidMcodeArg("H", ARG_H); }
void ReportInvalidMcodeIArg(void)  { ReportInvalidMcodeArg("I", ARG_I); }
void ReportInvalidMcodeJArg(void)  { ReportInvalidMcodeArg("J", ARG_J); }
void ReportInvalidMcodeKArg(void)  { ReportInvalidMcodeArg("K", ARG_K); }
void ReportInvalidMcodeLArg(void)  { ReportInvalidMcodeArg("L", ARG_L); }
void ReportInvalidMcodeMArg(void)  { ReportInvalidMcodeArg("M", ARG_M); }
void ReportInvalidMcodeNArg(void)  { ReportInvalidMcodeArg("N", ARG_N); }
void ReportInvalidMcodeOArg(void)  { ReportInvalidMcodeArg("O", ARG_O); }
void ReportInvalidMcodePArg(void)  { ReportInvalidMcodeArg("P", ARG_P); }
void ReportInvalidMcodeQArg(void)  { ReportInvalidMcodeArg("Q", ARG_Q); }
void ReportInvalidMcodeRArg(void)  { ReportInvalidMcodeArg("R", ARG_R); }
void ReportInvalidMcodeSArg(void)  { ReportInvalidMcodeArg("S", ARG_S); }
void ReportInvalidMcodeTArg(void)  { ReportInvalidMcodeArg("T", ARG_T); }
void ReportInvalidMcodeUArg(void)  { ReportInvalidMcodeArg("U", ARG_U); }
void ReportInvalidMcodeVArg(void)  { ReportInvalidMcodeArg("V", ARG_V); }
void ReportInvalidMcodeWArg(void)  { ReportInvalidMcodeArg("W", ARG_W); }
void ReportInvalidMcodeXArg(void)  { ReportInvalidMcodeArg("X", ARG_X); }
void ReportInvalidMcodeYArg(void)  { ReportInvalidMcodeArg("Y", ARG_Y); }
void ReportInvalidMcodeZArg(void)  { ReportInvalidMcodeArg("Z", ARG_Z); }

void ReportInvalidMcodeAArgInt(void)  { ReportInvalidMcodeArgInt("A", ARG_A); }
void ReportInvalidMcodeBArgInt(void)  { ReportInvalidMcodeArgInt("B", ARG_B); }
void ReportInvalidMcodeCArgInt(void)  { ReportInvalidMcodeArgInt("C", ARG_C); }
void ReportInvalidMcodeDArgInt(void)  { ReportInvalidMcodeArgInt("D", ARG_D); }
void ReportInvalidMcodeEArgInt(void)  { ReportInvalidMcodeArgInt("E", ARG_E); }
void ReportInvalidMcodeFArgInt(void)  { ReportInvalidMcodeArgInt("F", ARG_F); }
void ReportInvalidMcodeGArgInt(void)  { ReportInvalidMcodeArgInt("G", ARG_G); }
void ReportInvalidMcodeHArgInt(void)  { ReportInvalidMcodeArgInt("H", ARG_H); }
void ReportInvalidMcodeIArgInt(void)  { ReportInvalidMcodeArgInt("I", ARG_I); }
void ReportInvalidMcodeJArgInt(void)  { ReportInvalidMcodeArgInt("J", ARG_J); }
void ReportInvalidMcodeKArgInt(void)  { ReportInvalidMcodeArgInt("K", ARG_K); }
void ReportInvalidMcodeLArgInt(void)  { ReportInvalidMcodeArgInt("L", ARG_L); }
void ReportInvalidMcodeMArgInt(void)  { ReportInvalidMcodeArgInt("M", ARG_M); }
void ReportInvalidMcodeNArgInt(void)  { ReportInvalidMcodeArgInt("N", ARG_N); }
void ReportInvalidMcodeOArgInt(void)  { ReportInvalidMcodeArgInt("O", ARG_O); }
void ReportInvalidMcodePArgInt(void)  { ReportInvalidMcodeArgInt("P", ARG_P); }
void ReportInvalidMcodeQArgInt(void)  { ReportInvalidMcodeArgInt("Q", ARG_Q); }
void ReportInvalidMcodeRArgInt(void)  { ReportInvalidMcodeArgInt("R", ARG_R); }
void ReportInvalidMcodeSArgInt(void)  { ReportInvalidMcodeArgInt("S", ARG_S); }
void ReportInvalidMcodeTArgInt(void)  { ReportInvalidMcodeArgInt("T", ARG_T); }
void ReportInvalidMcodeUArgInt(void)  { ReportInvalidMcodeArgInt("U", ARG_U); }
void ReportInvalidMcodeVArgInt(void)  { ReportInvalidMcodeArgInt("V", ARG_V); }
void ReportInvalidMcodeWArgInt(void)  { ReportInvalidMcodeArgInt("W", ARG_W); }
void ReportInvalidMcodeXArgInt(void)  { ReportInvalidMcodeArgInt("X", ARG_X); }
void ReportInvalidMcodeYArgInt(void)  { ReportInvalidMcodeArgInt("Y", ARG_Y); }
void ReportInvalidMcodeZArgInt(void)  { ReportInvalidMcodeArgInt("Z", ARG_Z); }

////////////////////////////////////////////////////////////////////////////////

#ifdef NEW_MCODE_TOOL_USE
byte ConvertArgTtoDeviceNum(byte argT)
{
	byte device;

	switch (_mcodeModeForArgT)
	{
	case MCODE_TARG_IS_CANBUS_ADDRESS:  // original case
		device = argT;
		break;
	case MCODE_TARG_IS_TOOL_NUMBER:
		if (argT < NUM_PHYSICAL_DEVICES)
		{   // argT is the tool number
			device = convertToolNumberToDevice(argT);
		}
		else
		{   // alias
			switch (argT)
			{   // remap new alias values to old
			case 200: device = 0; break;
			case 201: device = 10; break;
			case 202: device = 20; break;
			case 203: device = 30; break;
			case 204: device = 40; break;
			case 205: device = 50; break;
			case 208: device = 100; break;
			case 209: device = 90; break;
			default: device = argT; break;
			}
		}
		break;
	case MCODE_TARG_IS_CANBUS_ADDRESS_OR_TOOL_NUMBER:
		if (argT <= 9.0f)
		{   // argT is the tool number
			device = convertToolNumberToDevice(argT);
		}
		else
		{   // argT is the head address or alias
			if (argT == 200)
			{   // special case to remap new aliases into old aliases
				argT = 0;
			}
			device = argT;
		}
		device = argT;
		break;
	default:
		break;
	}

	return(device);
}

////////////////////////////////////////////////////////////////////////////////

outboxStruct *ConvertArgTtoHotheadOutboxPtr(void)
{
	byte device = (ARG_T_PRESENT) ? ConvertArgTtoDeviceNum((byte)ARG_T) : PersistantHotheadAddress;
	return(getOutboxPointer(device));
}
////////////////////////////////////////////////////////////////////////////////

outboxStruct *ConvertArgTtoHotbedOutboxPtr(void)
{
	byte device = (ARG_T_PRESENT) ? ConvertArgTtoDeviceNum((byte)ARG_T) : PersistantHotbedAddress;
	return(getOutboxPointer(device));
}
#else //!NEW_MCODE_TOOL_USE

outboxStruct *ConvertArgTtoHotheadOutboxPtr(void)
{
	byte device = (ARG_T_PRESENT) ? (byte)ARG_T : PersistantHotheadAddress;
	return(getOutboxPointer(device));
}
////////////////////////////////////////////////////////////////////////////////

outboxStruct *ConvertArgTtoHotbedOutboxPtr(void)
{
	byte device = (ARG_T_PRESENT) ? (byte)ARG_T : PersistantHotbedAddress;
	return(getOutboxPointer(device));
}

#endif //!NEW_MCODE_TOOL_USE

////////////////////////////////////////////////////////////////////////////////

float ConvertArg0to100ToPct0To1(char *arg, float val)
{	// take float range 0 to 1.0 and converts to an integer range 0 to 100 after appropriate error checking
	float dutyCycle = 0.0f;  // default unless changes
	if (val != INVALID_ARG_VALUE)
	{   // asummes duty cycle is spec'd as a float from 0 to 1
		if ((val < 0.0f) || (val > 1.0f))
			ReportInvalidMcodeArg(arg, val);
		else
			dutyCycle = val / 100.0f;
	}
	return(dutyCycle);
}

////////////////////////////////////////////////////////////////////////////////

int ConvertArg0to100ToDutyCycle0To100(char *arg, float val)
{	// take float range 0 to 1.0 and converts to an integer range 0 to 100 after appropriate error checking
	int dutyCycle = 0;  // default unless changes
	if (val != INVALID_ARG_VALUE)
	{   // asummes duty cycle is spec'd as a float from 0 to 100
		if ((val < 0.0f) || (val > 100.0f))
			ReportInvalidMcodeArg(arg, val);
		else
			dutyCycle = (int)val;
	}
	return(dutyCycle);
}

////////////////////////////////////////////////////////////////////////////////

void setupSwitchControlByTemperature(outboxStruct *outboxPtr, int switchNum)
{
	if ((ARG_S_MISSING) || (ARG_S == 0.0f))  // turn off heater
	{
		sendSwitchControlOffToDevice(outboxPtr, switchNum);
	}
	else
	{
		sendSwitchControlByTempToDevice(outboxPtr, switchNum, SWITCH_PRESCALE_1, ARG_S);
	}
}

////////////////////////////////////////////////////////////////////////////////

void setupChamberTemperature(outboxStruct *outboxPtr)
{
	if ((ARG_S_MISSING) || (ARG_S == 0.0f))  // turn off chamber
	{
		sendSwitchControlOffToDevice(outboxPtr, HH_AUX_SWITCH);
	}
	else
	{
		sendSwitchControlByTempToDevice(outboxPtr, HH_AUX_SWITCH, SWITCH_PRESCALE_10, ARG_S);
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean deviceIsAPhysicalDevice(byte device)
{
	if (getMailboxNum(device) >= NUM_PHYSICAL_DEVICES) // command only good for a single physical device
	{
		ReportMcodeError("ARG_T not a physical device");
		return(FALSE);
	}
	return(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void SetHeadOffsets(int headIndex)
{
	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then load the head offset
		if (motorArgPresent(M))
		{
			M->HeadOffsets[headIndex] = getMotorArgInNativeUnits(M);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//
// Actual M-Code processing of Args to CAN Bus packets destined for the HH
//
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void sendNotice(void)
{//sends the current Mcode back to the host controller
#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
		return;
#endif //ALLOW_NATIVE_LIGHTBURN
	sprintf(SendString, ">MC:M%d: ",(int) ARG_M);
	sendstring(SendString);

	if (ARG_T_PRESENT)
	{
		sprintf(SendString, "T%d: ",(int) ARG_T);
		sendstring(SendString);
	}

	if (ARG_S_PRESENT)
	{
		sprintf(SendString, "S%d: ",(int) ARG_S);
		sendstring(SendString);
	}

	if (ARG_O_PRESENT)
	{
		sprintf(SendString, "O%d: ",(int) ARG_O);
		sendstring(SendString);
	}

	if (ARG_N_PRESENT)
	{
		//now send the approximate line number it is on
		sprintf(SendString,"N%d: ", (int)ARG_N);
		sendstring(SendString);
	}

#ifdef GB_DEBUG_MOTION_Q
	sprintf(SendString,"(%d)", motionQ_numEntries());
	sendstring(SendString);
#endif
	sendstringCr("");
}

void SendCurrentMcodeExecutionNotice(boolean argTIsADevice)
{   // sends the current Mcode back to the Repetrel so it knows the command was processed
	if (argTIsADevice)
	{
		byte device = (ARG_T_PRESENT) ? (byte)ARG_T : PersistantHotheadAddress;
		if (isAPhysicalDevice(device))
		{
			sendNotice();
		}
		else
		{   // alias or group, so need to walk though all physical devices and report on any that match
			int i;
			for (i=0; i<NUM_PHYSICAL_DEVICES; i++)        //walks through each and every element to see if it needs to report
			{
				inboxStruct *inboxPtr = getInboxPointer(_MailBoxes._inbox[i].device);
				if (inboxPtr->deviceRegistered && matchesAnAlias(device, inboxPtr))
				{
					ARG_T = (float)inboxPtr->device;
					sendNotice();
				}
			}
		}
	}
	else
	{
		sendNotice();
	}
}

////////////////////////////////////////////////////////////////////////////////

void SendFakeMcodeExecutionNotice(int M, float T, float S, float O)
{   // sends a fake mcode completion (using in cases where Repetrel gets out of sync with the hardware to force the GUI to be correct
	sprintf(SendString, ">MC:M%3d: ", M);
	sendstring(SendString);

	if (T != INVALID_ARG_VALUE)
	{
		sprintf(SendString, "T%2d: ", (int) T);
		sendstring(SendString);
	}

	if (S != INVALID_ARG_VALUE)
	{
		sprintf(SendString, "S%d: ",(int) S);
		sendstring(SendString);
	}

	if ((int) ARG_M == 620)
	{   // special case to send arg E, using ARG_S value
		if ((S == INVALID_ARG_VALUE))
		{
			ARG_S = 0.0f;
			sprintf(SendString, "S%d: ",(int) S);
			sendstring(SendString);
		}
		sprintf(SendString, "E%d: ",(int) ARG_S);
		sendstring(SendString);
	}

	if (O != INVALID_ARG_VALUE)
	{
		sprintf(SendString, "O%d: ",(int) O);
		sendstring(SendString);
	}

	//now send the approximate line number it is on
	sprintf(SendString,"N%d: ", 0);
	sendstring(SendString);
	sendstringCr("");
}

////////////////////////////////////////////////////////////////////////////////

void ReportRetiredMcode(char *s)
{
	sprintf(_rptStr, "Retired MCODE%s%s", (s[0]==NULL_CHAR) ? "" : " - ", s);
	ReportMcodeError(_rptStr);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M0(void)    // program stop - program pause sequence
{
	// MCODE M0
	// MCODE
	// MCODE    program stop - this will implement the program pause sequence after
	// MCODE    most recent move completes.
	// MCODE    code preceeding this will need to drop z and move to safe waiting area

	SendCurrentMcodeExecutionNotice(FALSE);
	_gcodePaused = TRUE;
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M1(void)  // does nothing (program pause)
{
	// MCODE M1
	// MCODE
	// MCODE    program stop - this will implement the program pause sequence
	// MCODE    code preceeding this will need to drop z and move to safe waiting area
	// MCODE SHELL ONLY
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M2(void)  // does nothing (program end)
{
	// MCODE M2
	// MCODE
	// MCODE    does nothing (program end)
	// MCODE SHELL ONLY
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void processSpindleCommand(int direction)
{
#ifdef USE_HYDRA_IO
	if (ARG_B_PRESENT)
#elif defined (USE_HYREL_IO)
	if (ARG_A_PRESENT)
#else
	if (0)
#endif
	{   // lathe mode on a normal axis
		if (_gs.latheMotor)
		{
			MotorStructure *M = _gs.latheMotor;
			if (motorArgPresent(M) && M->MotorInstalled)
			{
				if ((M->latheMode == LATHE_MODE_OFF) && (getMotorArg(M) != 0.0f))
				{   // was off and not being used -- need to change things to set up lathe mode
					enableLatheMode();
				}

				if ((M->latheMode != LATHE_MODE_OFF) || (getMotorArg(M) != 0.0f))
				{   // !OFF = already running OR was off and want a non-zero rate
					setupLatheModeToRun(M, (getMotorArg(M) / 60.0f) * M->PulsesPerUnit, (direction == HH_SPINDLE_CLOCKWISE) ? DIRECTION_FORWARD : DIRECTION_REVERSE);
				}
			}
		}
		else if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT)
		{
#ifdef USE_HYDRA_IO
			ARG_B = INVALID_ARG_VALUE;  // kill B so no error
#elif defined (USE_HYREL_IO)
			ARG_A = INVALID_ARG_VALUE;  // kill A so no error
#else
			;
#endif
			MotorStructure *M;
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // whiz through all the motors, if there's a valid ARG from the command, then set accel constant
				if (motorArgPresent(M))
				{
					sprintf(_tmpStr, "%c", M->AxisLabel);
					ReportInvalidMcodeArg(_tmpStr, getMotorArg(M));
				}
			}
		}
	}

	if (ARG_S_PRESENT)
	{   // normal spindle command
		outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

		if (localOutboxPtr->deviceFamily == DEVICE_FAMILY_SPINDLE)
		{
			_gs._spindle.desiredPowerPct = ARG_S_MISSING ? 0.0f : (fFitWithinRange(ARG_S, 0.0f, 100.0f)) / 100.0f; // pct 0 to 1.0

			if (ARG_F_PRESENT)
			{   // change the freq of the pwm
				_gs._spindle.pwmFreq = ARG_F;
			}

			sendSpindleControl(localOutboxPtr, direction, HEAD_FUNCTION_PAGE_LOAD_DIRECTION);   // update direction
		}

	}
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M3(void)  // spindle on CW (uses T, S, F)
{
	// MCODE M3 ["T" toolSelector] ["S" speed] ["F" pwmFreq] ["latheAxis" speed]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is present, turn on the spindle running clockwise
	// MCODE    else       not present, turn off the spindle
	// MCODE    if FArg is present, set the pwmFreq for the spindle power control
	// MCODE    If "latheAxis" is present ("B" for EnginerHR_Hydra and "A" for EnginerSR_30M)
	// MCODE       set latheAxis to free spin at rate specified with the lathe axis in a CW direction
	// MCODE           pulseRate = (Arg / 60) * pulses_per_unit(from M92) ....
	// MCODE       set pulses_per_unit to pulses pre REVOLUTION in order to work in RPM
	// MCODE
	// MCODE    turn on the spindle in a clockwise direction

	if (ARG_S_PRESENT)
	{//generic M3 Sxxx command, just set the PWM for it and on signal
		SpindleDesiredSpeedPWM = ARG_S;
		McodeDrainState[M617_State_Ofset] = 1; //enable Spindle power
		McodeDrainState[M616_State_Ofset] = 0; //forward
		return;
	}
	
	if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT)
	{
		sendError("Legacy use of M3 to control lathe mode.  Please migrate to M253");
	}
	processSpindleCommand(HH_SPINDLE_CLOCKWISE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M4(void)  // spindle on CCW (uses T, S, F)
{
	// MCODE M4 ["T" toolSelector] ["S" speed] ["F" pwmFreq] ["latheAxis" speed]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is present, turn on the spindle running counter clockwise
	// MCODE    else       not present, turn off the spindle
	// MCODE    if FArg is present, set the pwmFreq for the spindle power control
	// MCODE    If "latheAxis" is present ("B" for EnginerHR_Hydra and "A" for EnginerSR_30M)
	// MCODE       set latheAxis to free spin at rate specified with the lathe axis in a CCW direction
	// MCODE           pulseRate = (Arg / 60) * pulses_per_unit(from M92) ....
	// MCODE       set pulses_per_unit to pulses pre REVOLUTION in order to work in RPM
	// MCODE
	// MCODE    turn on the spindle in a counter-clockwise direction
	if (ARG_S_PRESENT)
	{
		//generic M3 Sxxx command, just set the PWM for it and on signal
		SpindleDesiredSpeedPWM = ARG_S;
		McodeDrainState[M617_State_Ofset] = 1; //enable Spindle power
		McodeDrainState[M616_State_Ofset] = 1; //forward
		return;
	}
	
	if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT)
	{
		sendError("Legacy use of M4 to control lathe mode.  Please migrate to M254");
	}
	processSpindleCommand(HH_SPINDLE_COUNTER_CLOCKWISE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M5(void)  // spindle/lathe OFF
{
	// MCODE M5 ["T" toolSelector]
	// MCODE
	// MCODE    turn off the spindle motor and lathe motor


		//generic M3 Sxxx command, just set the PWM for it and on signal
		SpindleDesiredSpeedPWM = 0;
		McodeDrainState[M616_State_Ofset] = 0; //enable Spindle power
		McodeDrainState[M617_State_Ofset] = 0; //forward
		//return;

	
	if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT)
	{
		sendError("Legacy use of M5 to control lathe mode.  Please migrate to M255");
	}

	ARG_S = 0.0f;   // force SPINDLE off
	ARG_A = 0.0f;
	ARG_B = 0.0f;
	processSpindleCommand(HH_SPINDLE_CLOCKWISE);

}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M6(void)  // set tool offsets (uses T, D, I, O, X, Y, Z, A, B, C, K)
{
	// MCODE M6 ["T" toolSelector] ["D" diameter] [<"O" offsetIndex> ["X,Y,Z,A,B,C" <offset>] ["I" immediate move]]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used (T must map to a valid physical device or alias)
	// MCODE    if DArg is present, set the CurrentToolDiameter (does not save value in ToolOffset array
	// MCODE    if Aarg is present, set the head offsetIndex
	// MCODE    if IArg is present, the immedaitely move to the new offset position
	// MCODE    if KArg is present, then set state of flag to kill/block immediate moves on Tx tool changes (blockFlag == (ArgK == 1.0f)
	// MCODE    if X,Y,Z,A,B,C are present, then load the offset table with the corresponding value
	// MCODE
	// MCODE    set tool offsets and diameter-clockwise direction
	// MCODE    it is possible to use M6 with No Tool argument, this is used to set ONLY the fixture
	// MCODE    offsets, and does NOT select GMCommand

	if (ARG_T_PRESENT)
	{   // This is the ONLY way to change the PersistantHeadAddress;
		changeDevice((byte)ARG_T);//update the mailbox pointers to the desired target head(s)
	}

	if (ARG_D_PRESENT)
	{   //set up the diameter for the tool for pocket mills etc.
		//GB XXX gregkarl - 911 M6: why only a single ToolDiameter set in M6 and not stored in ToolOffsets?
		CurrentToolDiameter=convertArgToMM(ARG_D);
	}

	if (ARG_K_PRESENT)
	{
		_blockImmediateMoveOnToolChange = (ARG_K == 1.0f);
	}

	// need to ensure the head is in the right state for CAN_STEPs
	if (_canbusStepForE)
		sendSetStepFromCanbusModeBit(currentOutboxPtr->device);
	else
		sendClrStepFromCanbusModeBit(currentOutboxPtr->device);

	// warning---SendCurrentMcodeExecutionNotice will/can change ARG_T value
	SendCurrentMcodeExecutionNotice(TRUE); //this call must be ahead of the InvalidateAllCmdArgs call

	// it is possible to use M6 with No Tool argument, this is used to set ONLY the
	// fixture offsets, and does NOT select GMCommand
	// a new head
	if (ARG_O_PRESENT)
	{   //headoffsets
		int headIndex = (int)ARG_O;
		if ((headIndex < 0) || (headIndex >= NUM_HEAD_OFFSETS))
		{
			ReportInvalidMcodeOArg();
			headIndex = 0;  // index 0 should have 0 offsets
		}
		else
		{
			currentHeadIndex = headIndex; //select new offset if supplied
			if (currentHeadIndex != 0)  // don't allow location 0 to change
			{
				SetHeadOffsets(currentHeadIndex);

				if (ARG_I_PRESENT && (ARG_I == 1.0f) && !_blockImmediateMoveOnToolChange)
				{   //move the head to the last know position, but with the new offsets
					MotorStructure *M;
					InvalidateAllCmdArgs(ExecutionPtr);
					for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
					{   //setup a rapid move the head to super impose over the previous head location
						setMotorArgInNativeUnits(M, M->Q_LastRequestedPositionInUnits);
					}
					if (FLOW_RATE_CREATED_ON_MOTORC)
					{   // taking over axis C for step pulse generation
						ARG_C = INVALID_ARG_VALUE;
					}
					boolean save_IncrementalMove = _IncrementalMove;
					_IncrementalMove = FALSE;   // force ABSOLUTE mode
					motionQ_addCommand(RAPID_MOTION_RATE, NOT_HOMING_MOTORS);
					_IncrementalMove = save_IncrementalMove;
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
#define OVERLOADED_M7
#ifdef OVERLOADED_M7 // GB XXX really should use the other M7 IF IF IF we don't split M7 into two separate Mcode (see Hydra4, M7 & M783)
void M_Code_M7(void)  // turn spindle coolant/ heater1 on (uses T) -- overloaded to handle PersistantUltimusControlHeadAddress
{
#ifdef GB_HIDDEN_WARNINGS
	int whichM7_flavor;
#endif //GB_HIDDEN_WARNINGS
	// MCODE M7 ["T" toolSelector](sets PersistantUltimusControlHeadAddress)
	// MCODE M7 ["S" dutyCylce]  // turn on coolant / heater / output
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    turn coolant/ heater1 on (sets PersistantUltimusControlHeadAddress)

	if (ARG_T_PRESENT)
	{
		PersistantUltimusControlHeadAddress=((byte)ARG_T);
		if(PersistantUltimusControlHeadAddress)
		{   // if the PersistantUltimusControlHeadAddress is set, do not process additional args.
			return;
		}
	}
	PersistantUltimusControlHeadAddress=0; // force to 0 if no argT

	if (ARG_S_PRESENT)
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], ARG_S);  // turn on the output
	else
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], HSS_DUTY_CYCLE_ON);  // turn on the output
	SendCurrentMcodeExecutionNotice(FALSE);
}
#elif defined(OVERLOAD_M7_CLEAN)
void M_Code_M7(void)  // turn spindle coolant/ heater1 on (uses T) -- overloaded to handle PersistantUltimusControlHeadAddress
{
	// MCODE M7 -- WARNING DUAL USE (overloaded)
	// MCODE M7 ["T" toolSelector](sets PersistantUltimusControlHeadAddress) {use T0 to disable)
	// MCODE M7 ["S" dutyCylce]  {turn on coolant / HSS}
	// MCODE
	// MCODE    turn coolant HSS OR set PersistantUltimusControlHeadAddress

	if (ARG_T_PRESENT)
	{   // ARG_T_PRESENT -- new M7 usage to set the PersistantUltimusControlHeadAddress
		PersistantUltimusControlHeadAddress = ((byte)ARG_T);
		if (PersistantUltimusControlHeadAddress == 0)
		{
			PersistantUltimusControlHeadAddress = HH_POSITION_UNPLUGGED;
		}
	}
	else
	{   // ARG_T_MISSING -- original/default M7 usage
		if (ARG_S_PRESENT)
			changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], ARG_S);  // turn on the output
		else
			changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], HSS_DUTY_CYCLE_ON);  // turn on the output
	}
	SendCurrentMcodeExecutionNotice(FALSE);
}
#elif defined(SPLIT_M7)
void M_Code_M7(void)  // turn spindle coolant HSS (uses S)
{
	// MCODE M7 ["S" dutyCylce]  // turn on coolant / heater / output
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    turn coolant HSS

	if (ARG_T_PRESENT)
	{
		ReportMcodeError("M783 replaces M7 to set PersistantUltimusControlHeadAddress");
		return;
	}

	if (ARG_S_PRESENT)
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], ARG_S);  // turn on the output
	else
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], HSS_DUTY_CYCLE_ON);  // turn on the output
	SendCurrentMcodeExecutionNotice(FALSE);
}
#endif

////////////////////////////////////////////////////////////////////////////////

void M_Code_M8(void)  // flood coolant on
{
	// MCODE M8 ["S" dutyCylce]  // turn on coolant HSS output
	// MCODE
	// MCODE    If SArg present, pwm set to 0 <= Sarg <= 100
	// MCODE    If SArg missing, pwm 100
	// MCODE
	// MCODE    turn coolant on

	if (ARG_S_PRESENT)
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[FLOOD_COOLANT_HSS]], ARG_S);  // turn on the output
	else
		changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[FLOOD_COOLANT_HSS]], HSS_DUTY_CYCLE_ON);  // turn on the output
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M9(void)  // all coolant off
{
	// MCODE M9
	// MCODE
	// MCODE    turn off all coolant off

	changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[SPINDLE_COOLANT_HSS]], HSS_DUTY_CYCLE_OFF);  // turn off the output
	changeHssDuty(&HighSideSwitches[hssFuncToPinIndex[FLOOD_COOLANT_HSS]], HSS_DUTY_CYCLE_OFF);  // turn off the output
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M17(void)  // enable the holding torque on stepping motors
{
	// MCODE M17
	// MCODE
	// MCODE    enable the holding torque on the stepping motors

	EnableAllMotionMotors();
	EnableAllExtruderMotors();
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M18(void)  // releases the holding torque on stepping motors
{
	// MCODE M18
	// MCODE
	// MCODE    releases the holding torque on the stepping motors
	// MCODE    will automatically be enabled on next move command

	//GB XXX gregkarl: M18/M84 -- we should set a flag that forces a "Home" after a release, otherwise position is invalid
	//GB XXX gregkarl: M18/M84: _needToHome = TRUE;  // block motion until a homing cycle is performed

	requireAllAxesToHome();

	DisableAllMotionMotors();
	DisableAllExtruderMotors();
	SendCurrentMcodeExecutionNotice(FALSE);
	updateManualExtrudingOffForGUI(0);
}

////////////////////////////////////////////////////////////////////////////////


void M_Code_M30(void)  // marks end of program
{
	// MCODE M30
	// MCODE
	// MCODE    marks the end of a program

#ifdef COLLECT_METRICS
	if (_metrics.autoDumpOnM30)
	{
		if (_metrics.autoDumpOnM30 & 0x01)
			M_Code_M773();					// basic print metric
		if (_metrics.autoDumpOnM30 & 0x02)
			M_Code_M784();
		if (_metrics.autoDumpOnM30 & 0x04)
			M_Code_M774();
		if (_metrics.autoDumpOnM30 & 0x08)
			M_Code_M775();
		if (_metrics.autoDumpOnM30 & 0x10)
			M_Code_M776();
		if (_metrics.autoDumpOnM30 & 0x20)
			M_Code_M779();

		_metrics.autoDumpOnM30 = 0;
	}
#endif
	CO2LaserAnalogPwrPWM = 0;
	TIM8->CCR3 = 0;//make sure the lasers are disabled
	resetVariableOnJobEnd(TRUE);
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M41(void)  // spindle in Low Range
{
	// MCODE M41
	// MCODE
	// MCODE    set spindle in Low Range

#ifdef USE_HYREL_IO
	pinClear(HIGHLOWSPEED);
#endif  //USE_HYREL_IO
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M42(void)  // spindle in High Range
{
	// MCODE M42
	// MCODE
	// MCODE    set spindle in High Range

#ifdef USE_HYREL_IO
	pinSet(HIGHLOWSPEED);
#endif  //USE_HYREL_IO
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M44(void)  // spindle Motor Coolant ON
{
	// MCODE M44
	// MCODE
	// MCODE    turn spindle Motor Coolant ON

#ifdef USE_HYREL_IO
	pinSet(SPINDLECOOLANT);
#endif  //USE_HYREL_IO
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M45(void)  // spindle Motor Coolant OFF
{
	// MCODE M45
	// MCODE
	// MCODE    turn spindle Motor Coolant OFF

#ifdef USE_HYREL_IO
	//pinClear(SPINDLECOOLANT);
#endif  //USE_HYREL_IO
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

int hours(int seconds) { return(seconds / 3600); }
int minutes(int seconds) { return((seconds % 3600) / 60.0f); }

void M_Code_M73(void)  // passthru of remaining printing time (uses P, R)
{
	// MCODE M73 ["P" percentComplete] ["R" remainingTime]
	// MCODE
	// MCODE    passthru of remaining printing time (generated by Prusa Slicer)
	// MCODE SendCurrentMcodeExecutionNotice: M73 S<pctComplete> O<secondsSoFar>

	// this Mcode can be sent at any time and will occur at times between to printing moves.
	if (ARG_P_MISSING) { ReportMissingMcodePArg(); return; }
	//if (ARG_R_MISSING) { ReportMissingMcodeRArg(); return; }  // don't care if it's missing

	float pctComplete = ARG_P / 100.0f;
	int currentPrintTimeSeconds = _gs._seconds - _metrics.startTimeSec;

	ARG_S = ARG_P;  // copy to ARG_S for McodeExecutionNotice (ARG_P not echoed)
	ARG_O = currentPrintTimeSeconds;
	SendCurrentMcodeExecutionNotice(FALSE);

#ifdef COLLECT_METRICS
	if (pctComplete > 0.0f)
	{
		int extrapolatedTotalTimeSeconds = (int)((float)currentPrintTimeSeconds / pctComplete);
		int extrapolateTimeRemainingSeconds = extrapolatedTotalTimeSeconds - currentPrintTimeSeconds;
		sprintf(_tmpStr, "%2d:%02d left of %d:%02d (%d%c complete @ N:%d)",
				hours(extrapolateTimeRemainingSeconds), minutes(extrapolateTimeRemainingSeconds),
				hours(extrapolatedTotalTimeSeconds), minutes(extrapolatedTotalTimeSeconds),
				(int)(pctComplete*100.0f), '%',
				(int)ARG_N);
		sendGB(_tmpStr);
	}
#endif //COLLECT_METRICS
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M82(void)  // enable absolute E moves
{
	// MCODE M82
	// MCODE
	// MCODE    enable absolute E moves

	_IncrementalEMove = 0; // set extruder to absolute mode
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M83(void)  // enable incremental (relative) E moves
{
	// MCODE M83
	// MCODE
	// MCODE    enable incremental (relative) E moves


	_IncrementalEMove = 1; // set extruder to relative mode
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M84(void)  // releases the holding torque on stepping motors
{
	// MCODE M84
	// MCODE
	// MCODE    releases the holding torque on the stepping motors
	// MCODE    will automatically enabled on next move command

	M_Code_M18();
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M91(void)  // set Max travel distance (uses X, Y, Z, A, B, C, S)
{
	// MCODE M91 ["X,Y,Z,A,B,C" <distance>] ["S" abortAxisScaling]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the max travel distance for that axis
	// MCODE
	// MCODE    sets the per axis travel distance - used to prevent driving the bed past the the limit of travel,

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
		if (motorArgPresent(M))
		{
			M->MaximumTravelInUnits = getMotorArgInNativeUnits(M);
			UpdateAxisTravelLimits(M);
		}
	}
	if (ARG_S_PRESENT)
	{
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{
			M->AbortTravelScaling = (ARG_S < 1.0f) ? 0.0f : ARG_S;  // off for anything less the 1.0f
			UpdateAxisTravelLimits(M);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M92(void)  // set axis steps per unit,  (pulses/mm;  pulses/in(if inch mode); pulses/deg(rotary) (uses X, Y, Z, A, B, C)
{
	// MCODE M92 ["X,Y,Z,A,B,C" <pulses_per_unit>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the pulse per unit for that axis
	// MCODE
	// MCODE    used to scale new drive systems,

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set pulses per unit
		if (motorArgPresent(M))
		{
			M->PulsesPerUnit = getMotorArgInNativeUnits(M);
			M->UnitsPerPulse = 1.0f / M->PulsesPerUnit;

			// recalculate any state variables that are affected by the new scale factor
			UpdateAxisLimitAndRates(M);
			UpdateAxisTravelLimits(M);
			M->HomeDestinationInPulses = (int)roundf(M->HomeDestinationInUnits * M->PulsesPerUnit);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M93(void)  // sets home sensor polarity (uses X, Y, Z, A, B, C)
{
	// MCODE M93 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the home sense value for that axis
	// MCODE
	// MCODE    sets the per axis home sensor polarity (0 == ACTIVE_LOW; 1 == ACTIVE_HIGH)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set home polarity
		if (motorArgPresent(M))
		{
			setSensorPolarityAndEnable(&M->HomeSense, (polarity_t)(int)getMotorArg(M), TRUE);
			if (M->HomeSense.Enabled == FALSE)
			{   // if no sensor, then no need to home
				M->HasBeenHomed = TRUE;
			}
#ifdef USE_CAN_MOTOR_HOME_SENSOR
#warning "REMOVE ONCE H/L1/L2 on canMotors is resolved"
			if (axisOnCanbus(M))
			{
				for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
				{
					if (M->canMotors[canAddrIndex].canAddress > 0)
					{
						canSendDeviceInitValue1x16(M->canMotors[canAddrIndex].canAddress, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT1_POLARITY,  M->HomeSense.Polarity);
					}
				}
			}
#endif //USE_CAN_MOTOR_HOME_SENSOR
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M94(void)  // sets the default motor direction (uses X, Y, Z, A, B, C)
{
	// MCODE M94 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the default motor direction for that axis
	// MCODE
	// MCODE    sets the per axis default motor direction (0==NORMAL; 1==INVERT_DEFAULT)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set motor dir
		if (motorArgPresent(M))
		{
			if (getMotorArg(M) == 0.0f)
			{
				M->Direction.InvertDefault = NO;    // don't change the default "forward" direction
			}
			else
			{
				M->Direction.InvertDefault = YES;   // invert the default "forward" direction
			}
			outputDirectionBit(&M->Direction, DIRECTION_FORWARD);   // set an initial state
		}
	}
	// need to create false edges on the DIR pins to set up the motor driver to properly interpret the step pins
	_needToWiggleDirectionPins = 2;
	_g4DwellTimer = (_needToWiggleDirectionPins+1) * 10;	// will pause the processing of the cmdQueue while we wiggle
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M95(void)  // sets stall sensor polarity (uses X, Y, Z, A, B, C)
{
	// MCODE M95 ["X,Y,Z,A,B,C" <0|1|2>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the stall sense value for that axis
	// MCODE
	// MCODE    sets the per axis stall sensor polarity (0 == ACTIVE_LOW; 1 == ACTIVE_HIGH; 2 == DISABLED)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set home polarity
		if (motorArgPresent(M))
		{
			setSensorPolarityAndEnable(&M->FaultSense, (polarity_t)(int)getMotorArg(M), FALSE);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M96(void)  // sets the enable bit polarity (uses X, Y, Z, A, B, C)
{
	// MCODE M96 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the polarity for the enable bit
	// MCODE
	// MCODE    sets the per axis default motor enable polarity (0 == ACTIVE_LOW; 1 == ACTIVE_HIGH)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set motor enable polarity
		if (motorArgPresent(M))
		{
			if (getMotorArg(M) == 0.0f)
			{
				M->Enable.Polarity = ACTIVE_LOW;
			}
			else
			{
				M->Enable.Polarity = ACTIVE_HIGH;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M97(void)  // sets the step bit polarity (uses X, Y, Z, A, B, C)
{
	// MCODE M97 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the polarity for the step bit
	// MCODE
	// MCODE    sets the per axis default motor step polarity (0 == ACTIVE_LOW; 1 == ACTIVE_HIGH)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set motor step bit polarity
		if (motorArgPresent(M))
		{
			if (getMotorArg(M) == 0.0f)
			{
				M->Step.Polarity = ACTIVE_LOW;
			}
			else
			{
				M->Step.Polarity = ACTIVE_HIGH;
			}
			deassertControlBit(&M->Step);   // set an initial state
		}
	}

}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M98(void)  // sets limit1 sensor polarity (uses X, Y, Z, A, B, C)
{
	// MCODE M98 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the limit1 sense value for that axis
	// MCODE     ****WARNING: axis C-L1 is hard coded to control EMO detection
	// MCODE
	// MCODE    sets the per axis limit1 sensor polarity (0 == ACTIVE_LOW; 1 == ACTIVE_HIGH; 2 == DISABLED)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set home polarity
		if (motorArgPresent(M))
		{
			setSensorPolarityAndEnable(&M->Limit1Sense, (polarity_t)(int)getMotorArg(M), FALSE);
#ifdef USE_CAN_MOTOR_HOME_SENSOR
#warning "REINSTATE ONCE H/L1/L2 on canMotors is resolved"
//			if (axisOnCanbus(M))
//			{
//				for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
//				{
//					if (M->canMotors[canAddrIndex].canAddress > 0)
//					{
//						canSendDeviceInitValue1x16(M->canMotors[canAddrIndex].canAddress, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT1_POLARITY, M->Limit1Sense.Polarity);
//					}
//				}
//			}
#endif //USE_CAN_MOTOR_HOME_SENSOR
		}
	}
#ifdef GB_HIDDEN_WARNINGS
	int make_separate_mcode_or_use_E_arg_for_EMO_sensor;
#endif //GB_HIDDEN_WARNINGS
	if (ARG_C_PRESENT)
	{   // EMO piggybacks on C_L1 sensor)
		setSensorPolarityAndEnable(&EMO, (polarity_t)(int)ARG_C, FALSE);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M99(void)  // sets limit2 sensor polarity (uses X, Y, Z, A, B, C)
{
	// MCODE M99 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the limit2 sense value for that axis
	// MCODE
	// MCODE    sets the per axis limit2 sensor polarity (0 == ACTIVE_LOW; 1 == ACTIVE_HIGH)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set home polarity
		if (motorArgPresent(M))
		{
			setSensorPolarityAndEnable(&M->Limit2Sense, (polarity_t)(int)getMotorArg(M), FALSE);
#ifdef USE_CAN_MOTOR_HOME_SENSOR
			if (axisOnCanbus(M))
			{
				for (int canAddrIndex=0; canAddrIndex<M->maxCanMotors; canAddrIndex++)
				{
					if (M->canMotors[canAddrIndex].canAddress > 0)
					{
						canSendDeviceInitValue1x16(M->canMotors[canAddrIndex].canAddress, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_LIMIT2_POLARITY, M->Limit2Sense.Polarity);
					}
				}
			}
#endif //USE_CAN_MOTOR_HOME_SENSOR
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void setLaserWatchdogForPrime(outboxStruct *outboxPtr)
{
	if ((outboxPtr->deviceFamily == DEVICE_FAMILY_LASER) || (outboxPtr->deviceFamily == DEVICE_FAMILY_INKJET))
	{
		_gs._laser.watchdogMs = abs(getCurrentPrimeTimeMs(outboxPtr)) * 1000 * 2;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M101(void)  // enable HH motor and optionally set feed rate (uses T, S, E, P)
{
	// MCODE M101 ["T" toolSelector] ["S" feed rate #/min] ["E" continuous(!=0)] ["P"]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is present set to extrude at the given rate (assume Extrusion Scale Factor is set correctly)
	// MCODE    if EArg is present and non 0, set continuous extrude
	// MCODE    if PArg is present, then start extruding immediately
	// MCODE
	// MCODE    Turns on the motor enable pin on the selected HHs and optionally sets the feed rate and
	// MCODE    optionally turns on continuous extruding
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M102(void)  // enable HH motor and optionally set feed rate in reverse (uses T, S, E)
{
	// MCODE M102 ["T" toolSelector] ["S" feed rate #/min] ["E" continuous(!=0)]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is present set to extrude at the given rate (assume Extrusion Scale Factor is set correctly)
	// MCODE    if EArg is present and non 0, set continuous extrude
	// MCODE
	// MCODE    Turns on the motor enable pin on the selected HHs and optionally sets the feed rate and
	// MCODE    optionally turns on continuous extruding
	// MCODE
	// MCODE XXX THIS IS severely outdated and was not executing. M102
	// MCODE XXX should be symmetric to M101 (which has been changed a lot, but M102 was not kept in sync)
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M103(void)  // disable HH motor (uses T)
{
	// MCODE M103 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    Turns OFF the motor enable pin on the selected HHs

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	StopExtruding(localOutboxPtr);
	sendSetDisableMotorBit(localOutboxPtr->device);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void enableChiller(outboxStruct *outboxPtr)
{
	sendSwitchControlByDutyCycleToDevice(outboxPtr, HH_AUX_SWITCH, (ARG_C == 1.0f) ? 100 : 0);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M104(void)  // set HH temperature (uses T, S)
{
	// MCODE M104 ["T" toolSelector] <"S" desiredTemperature> <"C" enableChiller>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg not present OR (SArg == 0.0f), then heater is turned off
	// MCODE    otherwise, desired temperature is set
	// MCODE    if ArgC==1, enable chiller
	// MCODE
	// MCODE    set the desired temperature of the selected HHs

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_C_PRESENT)
	{   // enable chiller mode
		enableChiller(localOutboxPtr);
		if (ARG_S_MISSING)
		{
			return; // not trying to set a temperature
		}
	}

	setupSwitchControlByTemperature(localOutboxPtr, HH_HTR_SWITCH);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M105(void)  // read HH temperature (uses T)
{
	// MCODE M105 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    requests the selected HHs return their current fusion chamber temperature

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	canIssueReadRequest(localOutboxPtr->device, CAN_MSG_STATUS, STATUS_PAGE_HEATER_TEMP);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M106(void)  // turn fan on and (optionally) set pwm (uses T, I, S, W, J, P, C) - overloaded for laser
{
	// MCODE M106 ["T" toolSelector] ["S" dutyCycle] ["P" dutyCycle] ["R" nonPersistantDutyCycleRange] ["C" persistantDutyCycleRange] ["A" cooldownTimeSec]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present and PArg is not present, fan will be turned on 100%
	// MCODE    else if SArg is present, dutyCycle = SArg / dutyCycleRange) * 100
	// MCODE    else if PArg is present, dutyCycle = PArg / dutyCycleRange) * 100
	// MCODE        dutyCylce = PArg
	// MCODE        switchOnOnlyWhenExtruding == (PArg > 0)
	// MCODE    if AArg us present, then set the laserCooldown time
	// MCODE    if RArg is present, then nonPersistantDutyCycleRange = RArg    (this is to allow the GUI to operate in a 0-100
	// MCODE      mode in the event the persistantDutyCycleRange is set to something other than 100. }
	// MCODE    if CArg is present, then persistantDutyCycleRange = CArg    (this is to allow compatibility with programs
	// MCODE      like slic3r+Marlin using 0 to 255 for the dutyCycle (setting resets at end of job to default) (default=100)
	// MCODE
	// MCODE    turn on the fan of the selected HHs

#ifdef ALLOW_NATIVE_LIGHTBURN
	if (_lightburnModeEnabled)
	{   //M106 is used to set the laser power, so need to map to a M621
		// LBURN M106 <"S" vectorPowerPct>     (ARG_S in range of 0 to 255)
		// MCODE M621 <"P" vectorPowerPct> ["D" piercePowerPct]   (ARG
		float vectorPowerPct = (ARG_S_MISSING ? 0.0f : (fFitWithinRange(ARG_S, 0.0f, 100.0f))); // pct 0 to 100;
		InvalidateAllCmdArgs(ExecutionPtr);
		ARG_M = 621.0f;
		ARG_P = vectorPowerPct;
		M_Code_M621();
		return;
	}
#endif //ALLOW_NATIVE_LIGHTBURN
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	uint16_t dutyCycle = 0;
	if (ARG_C_PRESENT)
	{   // set the range for the fan duty cycle -- slic3r/marlin uses 0 to 255; our default is 0 to 100;
		_M106_fanDutyCycleRange = fFitWithinRange(ARG_C, 1.0f, 255.0f);
		if (ARG_S_MISSING)
		{   // no S, so just special case of setting the Range for compatibility with....
			return;
		}
	}

	if (ARG_S_PRESENT)
	{
		if (ARG_R_PRESENT)
		{   // using non-persistant scaling
				float tempFanDutyCycleRange = fFitWithinRange(ARG_R, 1.0f, 255.0f);
				dutyCycle = (uint16_t)fFitWithinRange(((ARG_S / tempFanDutyCycleRange) * 100.0f), 0.0f, 100.0f);  //duty is in percent
		}
		else
		{
			dutyCycle = (uint16_t)fFitWithinRange(((ARG_S / _M106_fanDutyCycleRange) * 100.0f), 0.0f, 100.0f);  //duty is in percent
		}
		ARG_S = dutyCycle;  // hack to get the GUI to show the selected duty cycle
	}
	else if (ARG_P_PRESENT)
	{   // set dutyCyle AND flag to have AUX/FAN HSS only on when extruding
		dutyCycle = (uint16_t)fFitWithinRange(ARG_P, 0.0f, 100.0f);  //ARG_P has the desired duty cycle
		// the use of argP in the case of an EMO is to set up the aux switch to only be on when extruding
		canWriteSwitchFlag_onOnlyWhenExtruding(localOutboxPtr, HH_AUX_SWITCH, (ARG_P == 0.0f) ? 0 : 1);
		ARG_S = ARG_P;  // hack to get the GUI to show the selected duty cycle
	}
	else    // ARG_S and ARG_P missing
	{
		dutyCycle = 100.0f;                           // if SArg is ommitted, turn fan on full speed
	}
	sendSwitchControlByDutyCycleToDevice(localOutboxPtr, HH_AUX_SWITCH, dutyCycle);


	if (ARG_A_PRESENT)
	{   // set the laser cooldown time (time to stay on after laser is disabled)
		_gs._laser.cooldown = (int)ARG_A;
		canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_DEV,  DEV_INIT_INDEX_DEV_COOLDOWN, _gs._laser.cooldown);
	}

	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M107(void)  // turn fan off (uses T)
{
	// MCODE M107 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    turn off the fan of the selected HHs

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	sendSwitchControlOffToDevice(localOutboxPtr, HH_AUX_SWITCH);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M108(void)  // set extrusion rate (uses T, S)
{
	// MCODE M108 ["T" toolSelector] <"S" rate #/sec>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, ERROR
	// MCODE    otherwise, HH is set to extrude at the given rate (assume Extrusion Scale Factor is set correctly)
	// MCODE
	// MCODE XXX THIS IS severely outdated and was not executing. M108
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void setupWaitForTemperature(byte device, waitForStruct *waitForPtr, boolean useHSS1) // useHSS1 is true for chamber
{
	// mcode    if (Carg AND Harg are present)    // asymmetric lower and upper absolute temp range
	// mcode        wait until (Harg <= currentTemp <= Carg) is true
	// mcode    else if (Harg is present)  // heater absolute temp threshold
	// mcode        wait until the currentTemp >= Harg
	// mcode    else if (Carg is present)  // chiller absolute temp threshold
	// mcode        wait until the currentTemp <= Carg
	// mcode    else if (Larg AND Uarg are present)    // asymmetric lower and upper delta temp range
	// mcode        wait until ((desiredTemp - Larg) <= currentTemp <= (desiredTemp + Uarg)) is true
	// mcode    else if (Larg is present)    // lower bound delta only    // heater delta temp threshold
	// mcode        wait until (currentTemp >= (desiredTemp - Larg)
	// mcode    else if (Uarg is present)    // upper bound delta only    // chiller delta temp threshold
	// mcode        wait until (currentTemp <= (desiredTemp + Uarg)
	// mcode    else if (Rarg is present)    // uniform temperature range
	// mcode        wait until ((desiredTemp - Rarg) <= currentTemp <= (desiredTemp + Rarg)) is true
	// mcode    else
	// mcode        wait until the desired temperature (set by Sarg or last M104 Sarg) is reached (heater only)
	// mcode
	// mcode    optionally set the desired temperature of the hotbed and
	// mcode    wait for selected hotbed to reach desired or requested temperature threshold or range

	waitForPtr->device = device;    // track which device to check

	// all options are converted into a desired range

	if (ARG_H_PRESENT || ARG_C_PRESENT)
	{
		waitForPtr->mode = WAIT_FOR_ABSOLUTE_TEMP;
		waitForPtr->minTemp = ARG_H_PRESENT ? (uint16_t)ARG_H : (MIN_TEMP >> TEMP_FRAC_BITS);
		waitForPtr->maxTemp = ARG_C_PRESENT ? (uint16_t)ARG_C : (MAX_TEMP >> TEMP_FRAC_BITS);
	}
	else if (ARG_L_PRESENT || ARG_U_PRESENT)
	{
		waitForPtr->mode = WAIT_FOR_RELATIVE_TEMP;
		waitForPtr->minTemp = ARG_L_PRESENT ? (uint16_t)ARG_L : (MIN_TEMP >> TEMP_FRAC_BITS);
		waitForPtr->maxTemp = ARG_U_PRESENT ? (uint16_t)ARG_U : (MAX_TEMP >> TEMP_FRAC_BITS);
	}
	else if (ARG_R_PRESENT)
	{
		waitForPtr->mode = WAIT_FOR_RELATIVE_TEMP;
		waitForPtr->minTemp = (uint16_t)ARG_R;
		waitForPtr->maxTemp = (uint16_t)ARG_R;
	}
	else
	{   // original behavior; wait for set desired temperature is reached.
		waitForPtr->mode = WAIT_FOR_ABSOLUTE_TEMP;
		if (useHSS1)
			waitForPtr->minTemp = getOutboxPointer(device)->swTarg[HH_AUX_SWITCH].temperature - WAIT_FOR_TEMP_ALLOWABLE_SHORTFALL;
		else
			waitForPtr->minTemp = getOutboxPointer(device)->swTarg[HH_HTR_SWITCH].temperature - WAIT_FOR_TEMP_ALLOWABLE_SHORTFALL;
		waitForPtr->maxTemp = MAX_TEMP >> TEMP_FRAC_BITS;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M109(void)  // wait for hothead to reach temp (uses T, S)
{
	// MCODE M109 ["T" toolSelector] ["S" desiredTemperature]
	// MCODE            [["H" waitForHeaterTemperature] ["C" waitForChillerTemperature]]   |
	// MCODE            [["L" lowerBoundDelta] ["U" upperBoundDelta]]                      |
	// MCODE            ["R" range (deltaAmount)]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE        else use the Targ to select which device to set/monitor
	// MCODE    if SArg is not present, no change to hothead temperature settings
	// MCODE        else desired temperature is set
	// MCODE
	// MCODE    if (Carg AND Harg are present)    // asymmetric lower and upper absolute temp range
	// MCODE        wait until (Harg <= currentTemp <= Carg) is true
	// MCODE    else if (Harg is present)  // heater absolute temp threshold
	// MCODE        wait until the currentTemp >= Harg
	// MCODE    else if (Carg is present)  // chiller absolute temp threshold
	// MCODE        wait until the currentTemp <= Carg
	// MCODE    else if (Larg AND Uarg are present)    // asymmetric lower and upper delta temp range
	// MCODE        wait until ((desiredTemp - Larg) <= currentTemp <= (desiredTemp + Uarg)) is true
	// MCODE    else if (Larg is present)    // lower bound delta only    // heater delta temp threshold
	// MCODE        wait until (currentTemp >= (desiredTemp - Larg)
	// MCODE    else if (Uarg is present)    // upper bound delta only    // chiller delta temp threshold
	// MCODE        wait until (currentTemp <= (desiredTemp + Uarg)
	// MCODE    else if (Rarg is present)    // uniform temperature range
	// MCODE        wait until ((desiredTemp - Rarg) <= currentTemp <= (desiredTemp + Rarg)) is true
	// MCODE    else
	// MCODE        wait until the desired temperature (set by Sarg or last M104 Sarg) is reached (heater only)
	// MCODE
	// MCODE    optionally set the desired temperature of the hothead and
	// MCODE    wait for selected hotbed to reach desired or requested temperature threshold or range

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT)
	{
		setupSwitchControlByTemperature(localOutboxPtr, HH_HTR_SWITCH);        // setup the parameters for the desired temp
	}
	if (deviceIsAPhysicalDevice(localOutboxPtr->device))
	{
		_MailBoxes._waitingFor.flags.bit.extruderTemp = TRUE;           // tell system to wait until the extruder reaches temp
		setupWaitForTemperature(localOutboxPtr->device, &_MailBoxes._waitingFor.extruder, FALSE);
		//_MailBoxes._waitingFor.extruderDevice = localOutboxPtr->device; // track which device to check
	}
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M114(void)  // report xyz location immediately
{
	// MCODE M114
	// MCODE
	// MCODE    report xyz location immediately
#ifdef ALLOW_NATIVE_LIGHTBURN
	sprintf(_tmpStr,"X:%4.3f Y:%4.3f Z:%4.3f E:%4.3f ok",
			Motors[M_X].POSITION * Motors[M_X].UnitsPerPulse,
			Motors[M_Y].POSITION * Motors[M_Y].UnitsPerPulse,
			Motors[M_Z].POSITION * Motors[M_Z].UnitsPerPulse,
			0.0f);
	sendstringCr(_tmpStr);
#else //!#ifdef ALLOW_NATIVE_LIGHTBURN
	ForceReportXYZLocation = TRUE;
	ReportXYZLocation();
	ForceReportXYZLocation = FALSE;
#endif //!ALLOW_NATIVE_LIGHTBURN
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M115(void)  // report firmware revision level
{
	// MCODE M115
	// MCODE
	// MCODE    report firmware revision level to host

	sendstring( "FIRMWARE_NAME:");
	if (SOFTWARE_TWEAK_REVISION == ' ')
		sprintf(_tmpStr, ">Hy: %s_%d.%03d", PLATFORM_STRING, SOFTWARE_MAJOR_REVISION, SOFTWARE_MINOR_REVISION);
	else if (SOFTWARE_TWEAK_REVISION == 'z')
		sprintf(_tmpStr, ">Hy: %s_%d.%03d%c_%c", PLATFORM_STRING, SOFTWARE_MAJOR_REVISION, SOFTWARE_MINOR_REVISION, SOFTWARE_TWEAK_REVISION, SOFTWARE_DEBUG_REVISION);
	else
		sprintf(_tmpStr, ">Hy: %s_%d.%03d%c", PLATFORM_STRING, SOFTWARE_MAJOR_REVISION, SOFTWARE_MINOR_REVISION, SOFTWARE_TWEAK_REVISION);
	sendstring(_tmpStr);
	sendstring( " FIRMWARE_URL:https://WWW.HYREL3d.COM ");
	sendstring( "PROTOCOL_VERSION:1.0 ");
	sendstring( "MACHINE_TYPE:HYREL_HYDRA EXTRUDER_COUNT:1 ");
	sendstringCr( "HYREL_PROTOCOL:1 " );
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M140(void)  // set hotbed temperature (uses T, S)
{
	// MCODE M140 ["T" toolSelector] <"S" desiredTemperature> <"C" enableChiller>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if (SArg not present OR (SArg == 0.0f), then hotbed heater is turned off
	// MCODE    otherwise, desired temperature is set for the hotbed
	// MCODE    if ArgC==1, enable chiller
	// MCODE
	// MCODE    set the desired temperature of the hotbed

	outboxStruct *localOutboxPtr = ConvertArgTtoHotbedOutboxPtr();

	if (ARG_C_PRESENT)
	{   // enable chiller mode
		enableChiller(localOutboxPtr);
		if (ARG_S_MISSING)
		{
			return; // not trying to set a temperature
		}
	}

	setupSwitchControlByTemperature(localOutboxPtr, HH_HTR_SWITCH);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M141(void)  // set chamber temperature (uses T, S)
{
	// MCODE M141 ["T" toolSelector] <"S" desiredTemperature>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg not present OR (SArg == 0.0f), then chamber heater is turned off
	// MCODE    otherwise, desired temperature is set
	// MCODE
	// MCODE    set the desired chamber temperature

	// fan output used on hotbed to control chamber
	outboxStruct *localOutboxPtr = ConvertArgTtoHotbedOutboxPtr();

	setupChamberTemperature(localOutboxPtr);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M190(void)  // wait for hotbed to reach temp (uses T, S, W)
{
	// MCODE M190 ["T" toolSelector] ["S" desiredTemperature]
	// MCODE            [["H" waitForHeaterTemperature] ["C" waitForChillerTemperature]]   |
	// MCODE            [["L" lowerBoundDelta] ["U" upperBoundDelta]]                      |
	// MCODE            ["R" range (deltaAmount)]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE        else use the Targ to select which device to set/monitor
	// MCODE    if SArg is not present, no change to hotbed temperature settings
	// MCODE        else desired temperature is set
	// MCODE
	// MCODE    if (Carg AND Harg are present)    // asymmetric lower and upper absolute temp range
	// MCODE        wait until (Harg <= currentTemp <= Carg) is true
	// MCODE    else if (Harg is present)  // heater absolute temp threshold
	// MCODE        wait until the currentTemp >= Harg
	// MCODE    else if (Carg is present)  // chiller absolute temp threshold
	// MCODE        wait until the currentTemp <= Carg
	// MCODE    else if (Larg AND Uarg are present)    // asymmetric lower and upper delta temp range
	// MCODE        wait until ((desiredTemp - Larg) <= currentTemp <= (desiredTemp + Uarg)) is true
	// MCODE    else if (Larg is present)    // lower bound delta only    // heater delta temp threshold
	// MCODE        wait until (currentTemp >= (desiredTemp - Larg)
	// MCODE    else if (Uarg is present)    // upper bound delta only    // chiller delta temp threshold
	// MCODE        wait until (currentTemp <= (desiredTemp + Uarg)
	// MCODE    else if (Rarg is present)    // uniform temperature range
	// MCODE        wait until ((desiredTemp - Rarg) <= currentTemp <= (desiredTemp + Rarg)) is true
	// MCODE    else
	// MCODE        wait until the desired temperature (set by Sarg or last M104 Sarg) is reached (heater only)
	// MCODE
	// MCODE    optionally set the desired temperature of the hotbed and
	// MCODE    wait for selected hotbed to reach desired or requested temperature threshold or range

	outboxStruct *localOutboxPtr = ConvertArgTtoHotbedOutboxPtr();

	if (ARG_S_PRESENT)
	{
		setupSwitchControlByTemperature(localOutboxPtr, HH_HTR_SWITCH);   // setup the parameters for the desired temp
	}

#ifdef SKIP_M190
	//donothing at this time until the hot bed is performing better. //GB XXX gregkarl - M190 skipped
	sendInfo("Skipping wait for hotbed temp (M190)");
#else
	if (deviceIsAPhysicalDevice(localOutboxPtr->device))
	{
		_MailBoxes._waitingFor.flags.bit.hotbedTemp = TRUE;             // tell system to wait until the hotbed reaches temp

		setupWaitForTemperature(localOutboxPtr->device, &_MailBoxes._waitingFor.hotbed, FALSE);
	}
#endif
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M191(void)  // wait for chamber to reach temp (uses T, S)
{
	// MCODE M191 ["T" toolSelector] ["S" desiredTemperature]
	// MCODE            [["H" waitForHeaterTemperature] ["C" waitForChillerTemperature]]   |
	// MCODE            [["L" lowerBoundDelta] ["U" upperBoundDelta]]                      |
	// MCODE            ["R" range (deltaAmount)]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE        else use the Targ to select which device to set/monitor
	// MCODE    if SArg is not present, no change to chamber temperature settings
	// MCODE        else desired temperature is set
	// MCODE
	// MCODE    if (Carg AND Harg are present)    // asymmetric lower and upper absolute temp range
	// MCODE        wait until (Harg <= currentTemp <= Carg) is true
	// MCODE    else if (Harg is present)  // heater absolute temp threshold
	// MCODE        wait until the currentTemp >= Harg
	// MCODE    else if (Carg is present)  // chiller absolute temp threshold
	// MCODE        wait until the currentTemp <= Carg
	// MCODE    else if (Larg AND Uarg are present)    // asymmetric lower and upper delta temp range
	// MCODE        wait until ((desiredTemp - Larg) <= currentTemp <= (desiredTemp + Uarg)) is true
	// MCODE    else if (Larg is present)    // lower bound delta only    // heater delta temp threshold
	// MCODE        wait until (currentTemp >= (desiredTemp - Larg)
	// MCODE    else if (Uarg is present)    // upper bound delta only    // chiller delta temp threshold
	// MCODE        wait until (currentTemp <= (desiredTemp + Uarg)
	// MCODE    else if (Rarg is present)    // uniform temperature range
	// MCODE        wait until ((desiredTemp - Rarg) <= currentTemp <= (desiredTemp + Rarg)) is true
	// MCODE    else
	// MCODE        wait until the desired temperature (set by Sarg or last M104 Sarg) is reached (heater only)
	// MCODE
	// MCODE    optionally set the desired temperature of the chamber and
	// MCODE    wait for selected hotbed to reach desired or requested temperature threshold or range

	outboxStruct *localOutboxPtr = ConvertArgTtoHotbedOutboxPtr();

	if (ARG_S_PRESENT)
	{
		// fan output used on hotbed to control chamber
		setupChamberTemperature(localOutboxPtr);
	}

#ifdef SKIP_M191
	//donothing at this time until the chamber is performing better. //GB XXX gregkarl - M191 skipped
	sendInfo("Skipping wait for chamber temp (M191)");
#else
	if (deviceIsAPhysicalDevice(localOutboxPtr->device))
	{
		_MailBoxes._waitingFor.flags.bit.chamberTemp = TRUE;       // tell system to wait until the chamber reaches temp
		setupWaitForTemperature(localOutboxPtr->device, &_MailBoxes._waitingFor.chamber, TRUE);
	}
#endif
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M203(void)  // Sets the Maximum G0/Rapid velocity UNIT/MIN (and Homing/Accel ramp) (uses X, Y, Z, A, B, C)
{
	// MCODE M203 ["X,Y,Z,A,B,C" <G0_max_velocity>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the G0 velocity for that axis
	// MCODE    if HArg - then load ALL motors with the new homing speed
	// MCODE    if PArg - then load ALL motors with the acceleration ramp factor
	// MCODE
	// MCODE    Sets the per axis Maximum G0 velocity and ALL AXIS Homing, and Acceleration Ramp

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set G0 max velocity
		if (motorArgPresent(M))
		{
			M->RatesInUPS[RAPID] = ClampToAxisLimitsInUPS(M, getMotorArgInNativeUnits(M) / 60.0f);
			if (M->RatesInUPS[JOGGING] <= M->RatesInUPS[AXIS_MIN])
			{
				M->RatesInUPS[JOGGING] = M->RatesInUPS[RAPID];  // set a default JOG rate
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M204(void)  // Sets the no ramp speed (UNITS/MIN) (no acceleration needed below this) (uses X, Y, Z, A, B, C)
{
	// MCODE M204 ["X,Y,Z,A,B,C" <NORampSpeed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the no ramp speed for that axis
	// MCODE
	// MCODE    Sets the per axis no ramp speed (no acceleration needed below this velocity)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set no ramp speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[NO_RAMP] = ClampToAxisLimitsInUPS(M, getMotorArgInNativeUnits(M) / 60.0f);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M205(void)  // Sets the homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
{
	// MCODE M205 ["X,Y,Z,A,B,C" <homingSpeed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the homing speed for that axis
	// MCODE
	// MCODE    Sets the per axis homing speed

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set the homing speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[HOMING] = ClampToAxisLimitsInUPS(M, getMotorArgInNativeUnits(M) / 60.0f);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M206(void)  // Sets the homing routine hysteresis (inch/mm/deg) (uses X, Y, Z, A, B, C)
{
	// MCODE M206 ["X,Y,Z,A,B,C" <homeHysteresisAmount >]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the hysteresis amount
	// MCODE
	// MCODE    Sets the homing routine hysteresis amount (inch/mm/deg)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set homing hysteresis distance
		if (motorArgPresent(M))
		{
			M->HomeHysteresisInUnits = getMotorArgInNativeUnits(M);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M208(void)  // Sets the acceleration constant (uses X, Y, Z, A, B, C)
{
	// MCODE M208 ["X,Y,Z,A,B,C" <accelerationConstant>] ["J1"]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the normal motion acceleration constant for that axis
	// MCODE
	// MCODE    Sets the per axis acceleration constant for either normal motion

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set accel constant
		if (motorArgPresent(M))
		{   // limit to small positive number to avoid divide by 0 and other issues
			M->AccelerationConstant = fmaxf(DEFAULT_MIN_ACCELERATION_CONSTANT, getMotorArgInNativeUnits(M));  //set the new acceleration constant
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M209(void)  // Sets the minimum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
{
	// MCODE M209 ["X,Y,Z,A,B,C" <min_axis_speed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the minimum axis speed for that axis
	// MCODE
	// MCODE    Sets the per axis minimum speed

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set mix axis speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[AXIS_MIN] = getMotorArgInNativeUnits(M) / 60.0f;
			UpdateAxisLimitAndRates(M); // ensure this rate meets the machine limits and adjust other rates match
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M210(void)  // Sets the maximum axis speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
{
	// MCODE M210 ["X,Y,Z,A,B,C" <max_axis_speed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the maximum axis speed for that axis
	// MCODE
	// MCODE    Sets the per axis maximum speed

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set max axis speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[AXIS_MAX] = getMotorArgInNativeUnits(M) / 60.0f;
			UpdateAxisLimitAndRates(M);// ensure this rate meets the machine limits and adjust other rates match
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M211(void)  // Sets the machine minimum pulse rate limit (uses S)
{
	// MCODE M211 <S min_pulse_rate>
	// MCODE
	// MCODE    if SArg is not present or negative, error
	// MCODE    else the min axis pulse rate is set
	// MCODE
	// MCODE    sets the machine minimum pulse rate limit
	// MCODE RETIRED
	ReportRetiredMcode("now self tuning");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M212(void)  // Sets the machine maximum pulse rate limits (uses S, R)
{
	// MCODE M212 [S maxPulseRate] [R maxRasterPulseRate]
	// MCODE
	// MCODE    if argS negative, error
	// MCODE    else the max axis pulse rate is set to argS
	// MCODE    if argR negative, error
	// MCODE    else the max axis pulse rate is set to argR
	// MCODE
	// MCODE    sets the machine maximum pulse rate limits
	// MCODE RETIRED
	ReportRetiredMcode("now self tuning");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M213(void)  // sets the per motor/axis installation status (uses X,Y,Z,A,B,C)
{
	// MCODE M213 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then 0=not_installed; 1=installed
	// MCODE
	// MCODE    sets the per motor/axis installation status

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set installed status
		if (motorArgPresent(M))
		{
			if (getMotorArg(M) == 0.0f)
			{
				M->MotorInstalled = FALSE;  // to indicate "disabled"
			}
			else // cover 1.0 and 2.0 cases
			{
				M->MotorInstalled = TRUE;   // to indicate "enabled"
				UpdateAxisLimitAndRates(M); // make sure it's rates are well behaved.
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M214(void)  // sets the per motor/axis type (linear/rotary) (uses X,Y,Z,A,B,C)
{
	// MCODE M214 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then 0=Linear; 1=Rotary
	// MCODE
	// MCODE    sets the per motor/axis axis type (linera/rotary)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set axis type
		if (motorArgPresent(M))
		{
			if (getMotorArg(M) == 0.0f)
			{
				M->AxisType = LINEAR;
			}
			else
			{
				M->AxisType = ROTARY;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M215(void)  // Sets the homing direction (uses X, Y, Z, A, B, C)
{
	// MCODE M215 ["X,Y,Z,A,B,C" <0 | 1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the homing direction
	// MCODE
	// MCODE    Sets the homing direction (0=TOWARD_ZERO; 1=AWAY_FROM_ZERO)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then home start location
		if (motorArgPresent(M))
		{
			if (getMotorArg(M) == 0.0f)
			{
				M->HomingDirection = HOME_TOWARD_ZERO;
			}
			else
			{
				M->HomingDirection = HOME_AWAY_FROM_ZERO;
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M216(void)  // Sets the home position in units (uses X, Y, Z, A, B, C)
{
	// MCODE M216 ["X,Y,Z,A,B,C" <home_position>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the home position
	// MCODE
	// MCODE    Sets the home position in units

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set home end location
		if (motorArgPresent(M))
		{
			M->HomeDestinationInUnits = getMotorArgInNativeUnits(M);
			M->HomeDestinationInPulses = (int)roundf(M->HomeDestinationInUnits * M->PulsesPerUnit);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M217(void)  // sets the max deceleration rate (for abort) (mm/sec/sec)  (uses X, Y, Z, A, B, C)
{
	// MCODE M217 ["X,Y,Z,A,B,C" <abort_deceleration_rate>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the minimum stopping distance
	// MCODE
	// MCODE    sets the per axis max deceleration rate for arbitrary motion stopping (ie, abort)
	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set motor min stopping distance
		if (motorArgPresent(M))
		{
			M->AbortDecelerationRate = getMotorArgInNativeUnits(M);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M218(void)  // set the fixture offsets  (uses O, X, Y, Z, A, B, C)
{
	// MCODE M218 <"O" fixtureIndex> ["X,Y,Z,A,B,C" <stopping_distance>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the selected fixures offset
	// MCODE
	// MCODE    set the fixture offsets

	MotorStructure *M;

	if (ARG_O_PRESENT)
	{   //fixture offset//only 16 of them stored  -- gregkarl -- you really mean headoffsets and not fixture, right???
		int fixtureIndex = (int)ARG_O;
		if ((fixtureIndex < 0) || (fixtureIndex >= NUM_FIXTURE_OFFSETS))    //GB XXX gregkarl would a 0 be an error here?
		{
			ReportInvalidMcodeOArg();
			fixtureIndex = 0;  // index 0 should have 0 offsets
		}
		else
		{
			currentFixtureIndex = fixtureIndex;   //select new offset if supplied
			if (currentFixtureIndex != 0)   // don't allow location 0 to change
			{
				for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
				{   // whiz through all the motors, if there's a valid ARG from the command, then load the fixture offset
					if (motorArgPresent(M))
					{
						M->FixtureOffsets[currentFixtureIndex] = getMotorArgInNativeUnits(M);
					}
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M219(void)  // sets the serial baud rate (uses B)
{
	// MCODE M219 <"B" baudrate>]
	// MCODE
	// MCODE    sets the serial baud rate

	if (ARG_B_MISSING) { ReportMissingMcodeBArg(); return; }
	switch((int)ARG_B)
	{
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 115200:
	case 250000:
	case 500000:
	case 750000:
		InitAllUARTS((int)ARG_B);
		break;
	default:
		sprintf(_errorStr,"INVALID BAUD RATE (valid rates: 9600/1922/38400/57600/115200/250000/500000/750000");
		sendError(_errorStr);
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M220(void)  // sets the per motor/axis send motorStep pulse as a CANbus command (uses T,X,Y,Z,A,B,C)
{
	// MCODE M220 <T deviceForCanStep> ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then 0=stop; 1=start sending "step" via canbus
	// MCODE
	// MCODE    sets the per motor/axis state of send motorStep pulse as a CANbus command
	// MCODE RETIRED
	ReportRetiredMcode("unsupported");
}


////////////////////////////////////////////////////////////////////////////////

void M_Code_M221(void)  // set extrude calculation factors (uses T,S,Z,W,P)
{
	// MCODE
	// MCODE M221 ["T" toolSelector] ["S" overridePct] ["Z" layerHeight] ["W" pathWidth] ["P" pulsesPerMicroLiter]
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE S "Fudge factor" (0 to 15.99)
	// MCODE Z Layer height in millimeters
	// MCODE W nozzle width in millimeters
	// MCODE P pulses per microliter
	// MCODE
	// MCODE flowrate is actually set when the G1/G2/etc code is run

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT)
	{
		//localOutboxPtr->ExtrusionControl.ExtrusionRateOverridePct = ARG_S;
		sendExtrusionOverridePctToDevice(localOutboxPtr, ARG_S);    // update device
	}
	if (ARG_Z_PRESENT)
	{
		if (ARG_Z <= 0.0f)
			ReportInvalidMcodeZArg();
		else
			localOutboxPtr->ExtrusionControl.SliceHeightInMm = ARG_Z;  //  layer height
	}

	if (ARG_W_PRESENT)
	{
		if (ARG_W <= 0.0f)
			ReportInvalidMcodeWArg();
		else
			localOutboxPtr->ExtrusionControl.ExtrusionWidthInMm = ARG_W;   //nozzle width
	}

	if (ARG_P_PRESENT)
	{
		if (ARG_P <= 0.0f)
			ReportInvalidMcodePArgInt();
		else
			localOutboxPtr->ExtrusionControl.PulsesPerUnit = ARG_P;
	}
	SendCurrentMcodeExecutionNotice(TRUE);//echo back to Host so it can reflect the true state of the printer
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M222(void)  // set the centripetal acceleration radius (uses S)
{
	// MCODE M222 <"S" centripetalAccelRadius>
	// MCODE
	// MCODE    if SArg is not present or negative, error
	// MCODE    else thedraft mode error tolerance is set
	// MCODE
	// MCODE    set the draft mode error tolerance

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (ARG_S < 0.0f) { ReportInvalidMcodeSArg(); return; }

	_MachineLimits.CentripetalAccelRadius = ARG_S;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M223(void)  // Sets the re-homing speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
{
	// MCODE M223 ["X,Y,Z,A,B,C" <re-homingSpeed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the re-homing speed for that axis
	// MCODE
	// MCODE    Sets the per axis re-homing speed

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set the re-homing speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[REHOMING] = ClampToAxisLimitsInUPS(M, getMotorArgInNativeUnits(M) / 60.0f);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M224(void)  // Sets the jog no ramp speed (UNITS/MIN) (no acceleration needed below this) (uses X, Y, Z, A, B, C)
{
	// MCODE M224 ["X,Y,Z,A,B,C" <jogNoRampSpeed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the no ramp speed for that axis
	// MCODE
	// MCODE    Sets the per axis no ramp speed (no acceleration needed below this velocity)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set no ramp speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[JOG_NO_RAMP] = ClampToAxisLimitsInUPS(M, getMotorArgInNativeUnits(M) / 60.0f);
			M->joggingRatesSet.JOG_NO_RAMP = 1;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M225(void)  // Sets the jog speed (UNITS/MIN) (uses X, Y, Z, A, B, C)
{
	// MCODE M225 ["X,Y,Z,A,B,C" <jogSpeed>] (UNITS/MIN)
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the jogging speed for that axis
	// MCODE
	// MCODE    Sets the per axis jog speed

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set the jogging speed
		if (motorArgPresent(M))
		{
			M->RatesInUPS[JOGGING] = ClampToAxisLimitsInUPS(M, getMotorArgInNativeUnits(M) / 60.0f);
			M->joggingRatesSet.JOGGING = 1;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M226(void)  // Sets the jog acceleration constant (uses X, Y, Z, A, B, C, P)
{
	// MCODE M226 ["X,Y,Z,A,B,C" <jogAccelerationConstant>] ["P" pauseTime]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the jog acceleration constant for that axis
	// MCODE    if P present, then set the pause time (seconds) when then direction changes
	// MCODE
	// MCODE    Sets the per axis acceleration constant for jogging

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set accel constant
		if (motorArgPresent(M))
		{   // limit to small positive number to avoid divide by 0 and other issues
			M->JogAccelerationConstant = fmaxf(DEFAULT_MIN_ACCELERATION_CONSTANT, getMotorArgInNativeUnits(M));  //set the new acceleration constant
			M->joggingRatesSet.ACCEL = 1;
		}
	}

	if (ARG_P_PRESENT)
	{
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{   // whiz through all the motors, and set the pauseTime
			M->JogPauseTimeMs = iFitWithinRange((int)(ARG_P * 1000.0f), 0, JOG_MAXIMUM_CHANGE_OF_DIRECTION_DELAY_MS);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M227(void)  // control jogging using the AB encoder on the panel interface (uses XYZABCRO>
{
	// MCODE M227 [XYZABC incr]  [R1 (reverseDirection)]  ; enable jogging for selected axis using AB encoder on panel interface
	// MCODE M227                                         ; disable jogging
	// MCODE M227 [O 0|1]                                 ; set the "off" usage of the AB encoder.
	// MCODE
	// MCODE    argXYZABC - incremental change of position per encoder detent based on "incr" value as follows:
	// MCODE        "incr" has the following meanings
	// MCODE            0         : one motor "step" at a time (fineest resolution)
	// MCODE            -1        : use the jog interval defined using M796
	// MCODE            otherwise : use the "incr" value as the jog interval
	// MCODE    if no axis is specified, exit jogging mode
	// MCODE
	// MCODE    argR - if argR==1, then reverse default direction
	// MCODE
	// MCODE    argO - when not jogging, the AB encoder is available for an alternate use:
	// MCODE           0 = local (SPI) display/GUI control
	// MCODE           1 = test/debug mode - free running encoder reported with >RT string as fake head 16
	// MCODE
	// MCODE control jogging using the AB encoder on the panel interface
#ifdef USE_AB_ENCODER
	MotorStructure *M = NULL;

	if (ARG_O_PRESENT)
	{
		switch ((int)ARG_O)
		{
		case JOG_OFF_MODE_GUI:		_ABjogging.offMode = JOG_OFF_MODE_GUI;			break;
		case JOG_OFF_MODE_FILAMENT: _ABjogging.offMode = JOG_OFF_MODE_FILAMENT;		break;
		default:  					_ABjogging.offMode = JOG_OFF_MODE_GUI;			break;
		}
	}

	if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING)
	{
		_ABjogging.enabled = FALSE;

		uint32_t irq_disabled = interruptsOff();
		if (_jogging.M->PULSES_TO_GO != 0)
		{
			joggingAbort("");
		}
		interruptsOn(irq_disabled);
		timerInitEncoderAB(FALSE);  // reset for GUI use
	}
	else if (ARG_X_PRESENT && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_X]; }
	else if (ARG_X_MISSING && ARG_Y_PRESENT && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_Y]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_PRESENT && ARG_A_MISSING && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_Z]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_PRESENT && ARG_B_MISSING && ARG_C_MISSING) { M = &Motors[M_A]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_PRESENT && ARG_C_MISSING) { M = &Motors[M_B]; }
	else if (ARG_X_MISSING && ARG_Y_MISSING && ARG_Z_MISSING && ARG_A_MISSING && ARG_B_MISSING && ARG_C_PRESENT) { M = &Motors[M_C]; }
	else { ReportMcodeError("Must specify one and only one jog axis"); }

	if (M != NULL)
	{
		_ABjogging.axisIndex = M->Axis;
		boolean lastDirection = _ABjogging.reverseDefaultDirection;
		_ABjogging.reverseDefaultDirection = (ARG_R == 1.0f) ? TRUE : FALSE;
		if ((_ABjogging.enabled == FALSE) || (_ABjogging.reverseDefaultDirection != lastDirection))
		{   // setup timer for AB encoder -- either enabling AB jog or request for a CCW/CW direction flip (handle in timer setup)
			_ABjogging.enabled = FALSE; // disable while dir change
			timerInitEncoderAB(_ABjogging.reverseDefaultDirection);
			_ABjogging.enabled = TRUE;
		}
		float arg = getMotorArg(M);
		if (arg == 0.0f)
			_ABjogging.incrAmount = M->UnitsPerPulse;   // 1 step's worth
		else if (arg == -1.0f)
			_ABjogging.incrAmount = M->JogIncrement;    // M796 defined amount
		else
			_ABjogging.incrAmount = arg;                // user defined amount
	}
#endif //USE_AB_ENCODER
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M228(void)  // disable auto extruder unprime and prime
{
	// MCODE M228
	// MCODE
	// MCODE disable auto extruder unprime and prime
	// MCODE RETIRED
	ReportRetiredMcode("use M229 Sxxx");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M229(void)  // extrusion control (uses T, E, P, S, D, F)
{
	// MCODE M229 [T device] ["E" extrusionControl] ["D" defineStepFormation] ["P" unprimePrimeControl] ["S" minUnprimePrimeTime] ["F" motionFlowControl]
	// MCODE
	// MCODE set up how to interpret the gcode stream with regards to extruding (E values)
	// MCODE
	// MCODE ARG_T : if ARG_T is not present, the prior selected device/alias is used
	// MCODE         else use the ARG_T to select which device to set/monitor
	// MCODE
	// MCODE ARG_E : extrusionControl
	// MCODE      0 : Ignore E Values (determine flow from motion rate, path width and layer thickness)
	// MCODE      1 : Use E values from gcode (cubic mm or mm/inch)
	// MCODE
	// MCODE ARG_D : extruder step formation control
	// MCODE      0 : auto generated on device  (NOT VALID with "Use E Value")                          [DEFAULT when E=0]
	// MCODE      1 : extrusion steps sent one at a time via canbus
	// MCODE      2 : extrusion steps sent on motor C connector
	// MCODE      3 : BOTH extrusion steps sent one at a time via canbus AND sent on motor C connector  [DEFAULT when E=1]
	// MCODE
	// MCODE ARG_P : unprimePrimeControl
	// MCODE      0 : Automatic Unprime/Prime detection                   <ONLY VALID OPTION AT THIS TIME>
	// MCODE      1 : Gcode (G10/G11) used for Unprime/Prime
	// MCODE      2 : Mcode (M721/M722) used for Unprime/Prime
	// MCODE      3 : E Value controls Unprime/Prime (G1 Exxx Fxxx)
	// MCODE      x : NO Unprime/Prime
	// MCODE
	// MCODE ARG_S : minUnprimePrimeTime (used in Automatic Unprime/Prime only)
	// MCODE      if the nominal time for a single non-printing move that occurs
	// MCODE      between two printing moves is less than minUnprimePrimeTime
	// MCODE      the unprime/prime pair will not be issued.
	// MCODE
	// MCODE ARG_F : auto Flow calc motion selection
	// MCODE      0 : use FeedRate (from Gx Fxxx)
	// MCODE      1 : use calculated cruising (top) speed for each move
	// MCODE      2 : use calculated average speed for each move

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT)
	{
		if (ARG_S < 0.0f)
			ReportInvalidMcodeSArg();
		else
			_autoReverseAndPrimeMinTime = ARG_S;
	}

	if (ARG_D_PRESENT)
	{
		switch ((int)ARG_D)
		{
		case 1 :  _hijackAxisC = TRUE;  _canbusStepForE = TRUE;  _directStepForE = FALSE; break;
		case 2 :  _hijackAxisC = TRUE;  _canbusStepForE = FALSE; _directStepForE = TRUE; break;
		case 3 :  _hijackAxisC = TRUE;  _canbusStepForE = TRUE;  _directStepForE = TRUE;  break;
		default : _hijackAxisC = FALSE; _canbusStepForE = FALSE; _directStepForE = FALSE; break;
		}
	}

	if (ARG_P_PRESENT)
	{
		_unprimePrimeControl = AUTO_UNPRIME_PRIME;
		// ONLY XXX AUTO_UNPRIME_PRIME supported at the moment
		switch ((unprime_prime_control_t)(int)ARG_P)
		{
		case AUTO_UNPRIME_PRIME  : _unprimePrimeControl = AUTO_UNPRIME_PRIME; break;
//		case GCODE_UNPRIME_PRIME : _unprimePrimeControl = GCODE_UNPRIME_PRIME; break;
//		case MCODE_UNPRIME_PRIME : _unprimePrimeControl = MCODE_UNPRIME_PRIME; break;
//		case E_ARG_UNPRIME_PRIME : _unprimePrimeControl = E_ARG_UNPRIME_PRIME; break;
		default :                  _unprimePrimeControl = NO_UNPRIME_PRIME;  break;
		}

	}

	if (ARG_F_PRESENT)
	{
		switch ((motionForFlowControl_t)(int)ARG_F)
		{
		case MOTION_FLOW_USE_FEEDRATE :
		case MOTION_FLOW_USE_CRUISE_SPEED :
		case MOTION_FLOW_USE_AVG_SPEED :
			_motionForFlowControl = (motionForFlowControl_t)(int)ARG_F;
			break;
		default :
			_motionForFlowControl = MOTION_FLOW_USE_FEEDRATE;
			break;
		}
	}

	if (ARG_E_PRESENT)
	{
		switch ((E_control_t)(int)ARG_E)
		{
		case IGNORE_E_VALUES :
			_extrusionControl = IGNORE_E_VALUES;
			break;
		case USE_E_VALUES :
			_extrusionControl = USE_E_VALUES;
			if ((_canbusStepForE == FALSE) && (_directStepForE == FALSE))   // ARG_D was 0
			{   // force canbus E steps when using E values.  ... can't rely on thrashing the head timers with rate changes per move
				_hijackAxisC    = TRUE;
				_canbusStepForE = TRUE;
				_directStepForE = TRUE;
			}
			break;
		default :
			_extrusionControl = INVALID_E_CONTROL;
			ReportInvalidMcodeEArg(); break;
		}
	}

	if (_canbusStepForE)
	{
		sendSetStepFromCanbusModeBit(localOutboxPtr->device);
		Motors[M_C].SendStepViaCanbus = TRUE;
	}
	else
	{
		sendClrStepFromCanbusModeBit(localOutboxPtr->device);
		Motors[M_C].SendStepViaCanbus = FALSE;
	}

	_LastExtrusionRate = 0; // force a reload of timers
	E_Q_LastRequestedPositionInUnits = 0.0f;
	E_Q_POSITION = 0;
}

////////////////////////////////////////////////////////////////////////////////

void updateCompositeOverridePct(void)
{
	_CompositeFlowOverridePct   = _FeedRateOverridePct * _FlowRateOverridePct;
	_CompositeMotionOverridePct = _FeedRateOverridePct * _MotionRateOverridePct;
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M230(void)  // set global flow rate override percentage (uses S)
{
	// MCODE M230 <"S" globalFlowRateMultiplier>
	// MCODE
	// MCODE set the global flow rate scale factor (sent to ALL devices)
	// MCODE rate is scaled along with motion (stays consistent)
	// MCODE globalFlowRateMultiplier is limited to the range 0.0 to 15.99

   if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
   _FlowRateOverridePct = fFitWithinRange(ARG_S, OVERRIDE_PCT_MIN, OVERRIDE_PCT_MAX);

   updateCompositeOverridePct();
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M231(void)  // set motion feedrate override percentage (uses S)
{
	// MCODE M231 <"S" motionFeedrateOverridePct>
	// MCODE
	// MCODE set motion feedrate override percentage for G1/G2/G3 motion. flow
	// MCODE flow rate is scaled along with motion rate (stays consistent)
	// MCODE motionFeedrateOverridePct is limited to the range 0.0 to 15.99

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	_FeedRateOverridePct = fFitWithinRange(ARG_S, OVERRIDE_PCT_MIN, OVERRIDE_PCT_MAX);

	updateCompositeOverridePct();
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M232(void)  // set motion override percentage (uses S)
{
	// MCODE M232 <"S" motionOverridePct>
	// MCODE
	// MCODE set motion override percentage for non-homing moves (change occurs
	// MCODE after accel calcs. use with caution. does not change material flow rate.
	// MCODE motionOverridePct is limited to the range 0.0 to 15.99

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	_MotionRateOverridePct = fFitWithinRange(ARG_S, OVERRIDE_PCT_MIN, OVERRIDE_PCT_MAX);

	updateCompositeOverridePct();
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M233(void)  // set homing pop-off distance (inch/mm/deg) (uses X, Y, Z, A, B, C)
{
	// MCODE M233 ["X,Y,Z,A,B,C" <homingPopOffDistance>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the homing pop off distance
	// MCODE
	// MCODE    Sets the homing pop-off distance (inch/mm/deg)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set homing pop off distance
		if (motorArgPresent(M))
		{
			M->HomePopOffDistanceInUnits = getMotorArgInNativeUnits(M);
		}
	}
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M234(void)  // set motor position (absolute) (inch/mm/deg) (uses X, Y, Z, A, B, C)
{
	// MCODE M234 ["X,Y,Z,A,B,C" <setMotorPosition>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load new motor position
	// MCODE
	// MCODE    Sets the motor position (inch/mm/deg)

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors
		if (motorArgPresent(M))
		{
			M->Q_LastRequestedPositionInUnits = getMotorArgInNativeUnits(M);
			M->POSITION =  M->Q_LastRequestedPositionInUnits * M->PulsesPerUnit;
			M->Q_POSITION = M->POSITION;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

boolean validAxisIndex(int index)
{
	if (index < 0)
		return(FALSE);
	else if (index >= (MAX_NUMBER_OF_MOTORS - 1))
		return(FALSE);
	else
		return(TRUE);
}
////////////////////////////////////////////////////////////////////////////////

boolean oneOfThreeIs1(float f1, float f2, float f3)
{
	int oneCount = 0;
	if ((f1 != INVALID_ARG_VALUE) && (f1 == 1.0f)) oneCount++;
	if ((f2 != INVALID_ARG_VALUE) && (f2 == 1.0f)) oneCount++;
	if ((f3 != INVALID_ARG_VALUE) && (f3 == 1.0f)) oneCount++;
	return(oneCount == 1);
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M235(void)  // set rotary motor, rotary axis plane and intersection offset (uses R, P, X, Y, Z)
{
	// MCODE M235 <"S" rotaryMotor> <"P" perpAxisIndex> <"X" offset> <"Y" offset> <"Z" offset>  (legacy) decrecated
	// MCODE M235 <"R" rotaryMotorIndex> <"P" parallelAxisIndex> <"X" offset> <"Y" offset> <"Z" offset>
	// MCODE
	// MCODE    set rotary axis plane and offset
#ifdef GB_HIDDEN_WARNINGS
#warning "ROTARY - M235 not finished"
	int add_UVW_print_direction;
#endif //GB_HIDDEN_WARNINGS

	boolean badArgs = FALSE;

	bzero(&_M235, sizeof(M235Struct));

	if (ARG_S_PRESENT){ ReportMcodeError("Please use argR rotary motor (argS deprecated)1111"); return; } // legacy check

	if (ARG_R_MISSING || ARG_P_MISSING) { ReportMcodeError("Please use argR rotary motor index, argP for parallel axis index and XYZ for offset"); return; }

	if (((int)ARG_R < M_A) > ((int)ARG_R > M_C))  { ReportMcodeError("Rotary Index must be either A:3, B:4, or C:5)"); return; }
	if (((int)ARG_P < M_X) > ((int)ARG_P > M_Y))  { ReportMcodeError("Rotary Index must be either X:0 or Y:0)"); return; }

	MotorStructure *R = &Motors[(int)ARG_R];
	MotorStructure *P = &Motors[(int)ARG_P];

	int axisIndex;
	for (axisIndex=FirstAxisMotor; axisIndex<=LastAxisMotor; axisIndex++)
	{   // whiz through all the motors
		R->RotaryOffset[axisIndex] = 0.0f;  // set offset to 0.0 for all axes
	}

	switch(P->Axis)
	{   // make sure parallel axis offset is 0.  just want offset to parallel axis for other two axes
	case M_X :
		if (ARG_X_PRESENT && (ARG_X != 0.0f)) { ReportInvalidMcodeXArg(); badArgs = TRUE; };
		R->RotaryOffset[M_Y] = (ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits);
		R->RotaryOffset[M_Z] = (ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits);

		break;
	case M_Y :
		if (ARG_Y_PRESENT && (ARG_Y != 0.0f)) { ReportInvalidMcodeYArg(); badArgs = TRUE; };
		R->RotaryOffset[M_X] = (ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits);
		R->RotaryOffset[M_Z] = (ARG_Z_PRESENT ? ARG_Z : Motors[M_Z].Q_LastRequestedPositionInUnits);
	case M_Z :
		if (ARG_Z_PRESENT && (ARG_Z != 0.0f)) { ReportInvalidMcodeZArg(); badArgs = TRUE; };
		R->RotaryOffset[M_X] = (ARG_X_PRESENT ? ARG_X : Motors[M_X].Q_LastRequestedPositionInUnits);
		R->RotaryOffset[M_Y] = (ARG_Y_PRESENT ? ARG_Y : Motors[M_Y].Q_LastRequestedPositionInUnits);
	}

	if (badArgs)
	{
		barf("M235 setup failed - unsafe to proceed");
	}
	else
	{
		_M235.rotaryMotorAxisPtr = R;
		_M235.axisParallelToRotaryAxisPtr = P;
		assignVector(&_M235.liftUV, 0.0f, 0.0f, 1.0f, TRUE); // generic positive Z motion for increased extruder height
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M236(void)  // write device type and revision to flash (option bytes) (uses T, S, I, P)
{
	// MCODE M236  <"T" toolSelector> <"S" deviceType> <"I" deviceRevision> <"P" password>
	// MCODE
	// MCODE    if TArg, is not present, error.
	// MCODE    if SArg, is not present, error.
	// MCODE    if IArg, is not present, error.
	// MCODE    if PArg, is not present, error.
	// MCODE    SArg - device type (0-255)
	// MCODE    IArg - device revision (0-255)
	// MCODE
	// MCODE    write device type and revision to flash (option bytes)
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M237(void)  // set cold extrusion prevention parameters (uses T, C, U, L, R)
{
	// MCODE M237 ["T" toolSelector] ["C" blockColdExtrude]
	// MCODE            ["R" range (deltaAmount)]  |
	// MCODE            ["L" lowerBoundDelta] ["U" upperBoundDelta>]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE        else use the Targ to select which device to set/monitor
	// MCODE    if CArg == 1.0
	// MCODE        the enable blocking cold extrude
	// MCODE    else
	// MCODE        allow extrusion at any temperature
	// MCODE
	// MCODE    IF (blocking cold extrusion):
	// MCODE
	// MCODE    if (Rarg is present)    // uniform temperature range
	// MCODE        extrude if ((desiredTemp - Rarg) <= currentTemp <= (desiredTemp + Rarg)) is true
	// MCODE    else if (Larg AND Uarg are present)    // asymmetric lower and upper delta temp range
	// MCODE        extrude if  ((desiredTemp - Larg) <= currentTemp <= (desiredTemp + Uarg)) is true
	// MCODE    else if (Larg is present)    // lower bound delta only    // heater delta temp threshold
	// MCODE        extrude  (currentTemp >= (desiredTemp - Larg)
	// MCODE    else if (Uarg is present)    // upper bound delta only    // chiller delta temp threshold
	// MCODE        extrude if  (currentTemp <= (desiredTemp + Uarg)
	// MCODE
	// MCODE    set the valid extrusion temperature range (prevent cold extrusion)

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

#ifdef CHECK_FOR_COLD_EXTRUDE
	switchFlags_t flags = {0};
	switchFlags_t mask  = {0};

	mask.b.blockColdExtrude     = 1;
	flags.b.blockColdExtrude    = (ARG_C == 1.0f) ? 1 : 0;

	temp_t minTemp = MAX_TEMP;
	temp_t maxTemp = MAX_TEMP;

	if (ARG_R_PRESENT)
	{   // spec uniform range
		minTemp = maxTemp = ((int)ARG_R) << TEMP_FRAC_BITS;
	}
	else
	{   // asymmetric range
		if (ARG_L_PRESENT)
		{   //spec just lower limit
			minTemp = ((int)ARG_L) << TEMP_FRAC_BITS;
		}
		if (ARG_U_PRESENT)
		{   // spec just upper limit
			maxTemp = ((int)ARG_U) << TEMP_FRAC_BITS;
		}
	}

	int switchNum = HH_HTR_SWITCH;

	canSendOutboxInitValue2x32(localOutboxPtr, getDevInitAreaSwitchNum(switchNum), DEV_INIT_INDEX_SWx_FLAGS, flags.u32, mask.u32);
	sendColdExtrudeTempRangesToDevice(localOutboxPtr, switchNum, minTemp, maxTemp);
#endif //CHECK_FOR_COLD_EXTRUDE
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M238(void)  // sets the per motor/axis execute E value (uses T,X,Y,Z,A,B,C)
{
	// MCODE M238 ["X,Y,Z,A,B,C" <0|1>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then 0=don't copy E; 1=copy E
	// MCODE
	// MCODE    sets the per motor/axis execute E value

	MotorStructure *M;
	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command
		if (motorArgPresent(M))
		{   // this forces an axis to copy the E value as it's position for the next move (and handle the separate relative/absolute diff between E and rest of the axis
			M->SubstituteAxisArgWithArgE = (getMotorArg(M) == 1.0f) ? TRUE : FALSE;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

int getSwitchNum(void)
{
	int switchNum = -1; // default 'failed' value

	if (ARG_I_MISSING)
		ReportMissingMcodeIArg();
	else if (ARG_I == 0.0f)
		switchNum = HH_FAN_SWITCH;
	else if (ARG_I == 1.0f)
		switchNum = HH_HTR_SWITCH;
	else
		ReportInvalidMcodeIArgInt();

	return(switchNum);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M240(void)  // turn device switch off (uses T, I)
{
	// MCODE M240 ["T" toolSelector] <"I" switchIndex>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    if IArg is not present or a valid switch, error
	// MCODE    else IArg is the switch index  (0=AUX/FAN  1=HTR)
	// MCODE
	// MCODE    turn device switch off

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	int switchNum = getSwitchNum();
	if (switchNum < 0) return;  // invalid or missing index

	sendSwitchControlOffToDevice(localOutboxPtr, switchNum);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M241(void)  // turn device switch on (uses T, I)
{
	// MCODE M241 ["T" toolSelector] <"I" switchIndex>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    if IArg is not present or a valid switch, error
	// MCODE    else IArg is the switch index  (0=AUX/FAN  1=HTR)
	// MCODE
	// MCODE    turn device switch on

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	int switchNum = getSwitchNum();
	if (switchNum < 0) return;  // invalid or missing index

	sendSwitchControlOnToDevice(localOutboxPtr, switchNum);
	SendCurrentMcodeExecutionNotice(TRUE);
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M242(void)  // control device switch by dutyCycle (uses T, I, S)
{
	// MCODE M242 ["T" toolSelector] <"I" switchIndex> <"S" dutyCycle>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    if IArg is not present or a valid switch, error
	// MCODE    else IArg is the switch index  (0=AUX/FAN  1=HTR)
	// MCODE
	// MCODE    if (SArg < 0) or (SArg > 100), error (dutyCycle=0)
	// MCODE    if SArg is not present, dutyCycle=0
	// MCODE    if (0 <= SArg <= 100) the HH will use SArg as a dutyCycle for the switch
	// MCODE
	// MCODE    control device switch by dutyCycle
#ifdef GB_HIDDEN_WARNINGS
	int add_granularity_control; // for M242, 243, 244
#endif //GB_HIDDEN_WARNINGS
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	int switchNum = getSwitchNum();
	if (switchNum < 0) return;  // invalid or missing index

	sendSwitchControlByDutyCycleToDevice(localOutboxPtr, switchNum, ConvertArg0to100ToDutyCycle0To100("S", ARG_S));
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M243(void)  // control device switch by pwm (uses T, I, S, F)
{
	// MCODE M243 ["T" toolSelector] <"I" switchIndex> <"S" dutyCycle> <"F" pwmFreq>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    if IArg is not present or a valid switch, error
	// MCODE    else IArg is the switch index  (0=AUX/FAN  1=HTR)
	// MCODE    if (SArg < 0) or (SArg > 100), error (dutyCycle=0)
	// MCODE    if SArg is not present, dutyCycle=0
	// MCODE    if (0 <= SArg <= 100) the HH will use SArg as the pct for the switch on time
	// MCODE
	// MCODE    if FArg is not present, error,
	// MCODE
	// MCODE    control device switch by generic pwm
	// MCODE
	// MCODE    NOTE: This is primarily used in software loop running at 4Khz to generate the pwm, so in general
	// MCODE          that limits to a max 2kHz freq and in that case, only 0, 50% and 100% will occur. A slower
	// MCODE          pwmFreq will allow for greater granularity
#ifdef GB_HIDDEN_WARNINGS
int finishM243;
#endif //GB_HIDDEN_WARNINGS
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	int switchNum = getSwitchNum();
	if (switchNum < 0) return;  // invalid or missing index

	if (ARG_F_MISSING) { ReportMissingMcodeFArg(); return; }

	sendSwitchControlByGeneralPwmToDevice(localOutboxPtr, switchNum, ConvertArg0to100ToPct0To1("S", ARG_S), (int)ARG_F);
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M244(void)  // control device switch by temperature (uses T, I, S)
{
	// MCODE M244 ["T" toolSelector] <"I" switchIndex> <"S" temperature>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    if IArg is not present or a valid switch, error
	// MCODE    else IArg is the switch index  (0=AUX/FAN  1=HTR)
	// MCODE
	// MCODE    if SArg not present OR (SArg == 0.0f), then switch is turned off
	// MCODE    otherwise, desired temperature is set
	// MCODE
	// MCODE    control device switch by temperature

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	int switchNum = getSwitchNum();
	if (switchNum < 0) return;  // invalid or missing index (error already reported)
	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }

	setupSwitchControlByTemperature(localOutboxPtr, switchNum);
	SendCurrentMcodeExecutionNotice(TRUE);

}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M245(void)  // set switch flag(s) (uses T, I, P, E, H, C)
{
	// MCODE M245 ["T" toolSelector] <"I" switchIndex> ["P" usePulseTrain] ["E" onOnlyWhenExtruding] ["H" polarity] ["C" blockColdExtrude]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    if IArg is not present or a valid switch, error
	// MCODE    else IArg is the switch index  (0=AUX/FAN  1=HTR)
	// MCODE
	// MCODE    if PArg is present, then set the switch control flag "usePulseTrain" to ArgP (1==usePT)
	// MCODE
	// MCODE    if EArg is present, then set the switch control flag "onOnlyWhenExtruding" to ArgE (1==onOnlyWhenExtruding)
	// MCODE
	// MCODE    if HArg is present, then set the switch control flag "polarity" to ArgH (1==activeHigh, else activeLow)
	// MCODE
	// MCODE    if CArg is present, then set the switch control flag "blockColdExtrude" to ArgC (1==blockColdExtrude)
	// MCODE
	// MCODE    set the switch flag(s) for the selected switch on the selected head

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	int switchNum = getSwitchNum();
	if (switchNum < 0) return;  // invalid or missing index

	switchFlags_t flags = {0};
	switchFlags_t mask  = {0};

	if (ARG_P_PRESENT)
	{
		mask.b.usePulseTrain    = 1;
		flags.b.usePulseTrain   = (ARG_P == 1.0f) ? 1 : 0;
	}
	if (ARG_H_PRESENT)
	{
		mask.b.polarity = 1;
		flags.b.polarity    = (ARG_H == 1.0f) ? 1 : 0;
	}
	if (ARG_E_PRESENT)
	{
		mask.b.onOnlyWhenExtruding  = 1;
		flags.b.onOnlyWhenExtruding = (ARG_E == 1.0f) ? 1 : 0;
	}
	if (ARG_C_PRESENT)
	{
		mask.b.blockColdExtrude     = 1;
		flags.b.blockColdExtrude    = (ARG_C == 1.0f) ? 1 : 0;
	}
	canSendOutboxInitValue2x32(localOutboxPtr, getDevInitAreaSwitchNum(switchNum), DEV_INIT_INDEX_SWx_FLAGS, flags.u32, mask.u32);
}

////////////////////////////////////////////////////////////////////////////////

void sendMotorMotionValues(byte device, MotorStructure *M)
{
	canSendDeviceInitValue1x32(device, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MOTION_JO, iFitWithinRange((uint32_t)(M->RatesInUPS[NO_RAMP] * M->PulsesPerUnit), 1, MAXINT));
	canSendDeviceInitValue1x32(device, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MOTION_MAX_RATE, iFitWithinRange((uint32_t)(M->RatesInUPS[AXIS_MAX] * M->PulsesPerUnit), 1, MAXINT));
	canSendDeviceInitValue1x32(device, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MOTION_ACCEL, iFitWithinRange((uint32_t)(M->AccelerationConstant), 0, MAXINT));
}

////////////////////////////////////////////////////////////////////////////////

void processLatheCommand(int direction)
{
	MotorStructure *M;
	boolean foundACanAxisMotor = FALSE;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{
		if (motorArgPresent(M) && M->canMotor)
		{
			foundACanAxisMotor = TRUE;
			if (direction == HH_SPINDLE_OFF)
			{
				canPackIntoTxQueue3x16(CAN_WRITE, ALIAS_ALL_CAN_AXIS_MOTORS_X + M->Axis, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_LATHE_MODE, BUFFERED_MSG, 0, 0, 0);
			}
			else
			{
				int16_t dir = (direction == HH_SPINDLE_CLOCKWISE) ? 1 : -1; //DIRECTION_FORWARD : DIRECTION_REVERSE)
				uint16_t speedRpm = getMotorArg(M);	// rev per min
				uint16_t forcePct = ARG_P_PRESENT ? convertFloatPowerPctToUnsignedint(fFitWithinRange(ARG_P/100.0f, 0.0f, 1.0f), HH_U16_POWER_PCT_FRAC_BITS) : 0;

				sendMotorMotionValues(ALIAS_ALL_CAN_AXIS_MOTORS_X + M->Axis, M);
				canPackIntoTxQueue3x16(CAN_WRITE, ALIAS_ALL_CAN_AXIS_MOTORS_X + M->Axis, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_LATHE_MODE, BUFFERED_MSG,
						dir,
						speedRpm,
						forcePct);
			}
		}
	}

	if (foundACanAxisMotor == FALSE)
	{	// prior behavior only one choice for latheMotor and relies on arg S for speed setting
		if (ARG_S_MISSING)
		{
			ARG_S = 0.0f;   // shut things down
		}

		if (_gs.latheMotor)
		{   // lathe mode on a normal axis
			M = _gs.latheMotor;
			if (motorArgPresent(M) && M->MotorInstalled)
			{
				if ((M->latheMode == LATHE_MODE_OFF) && (direction != HH_SPINDLE_OFF))
				{   // was off and not being used -- need to change things to set up lathe mode
					enableLatheMode();
				}
				if ((M->latheMode != LATHE_MODE_OFF) || (direction != HH_SPINDLE_OFF))
				{   // !OFF = already running OR was off and want a non-zero rate
					setupLatheModeToRun(M, (ARG_S / 60.0f) * M->PulsesPerUnit, (direction == HH_SPINDLE_CLOCKWISE) ? DIRECTION_FORWARD : DIRECTION_REVERSE);
				}
			}
		}
		if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT)
		{
			MotorStructure *M;
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // whiz through all the motors, if there's a ARG from a non-legal lathe motor, send an error
				if (motorArgPresent(M))
				{
					sprintf(_tmpStr, "%c", M->AxisLabel);
					ReportInvalidMcodeArg(_tmpStr, getMotorArg(M));
				}
			}
		}
	}
	SendCurrentMcodeExecutionNotice(TRUE);
}

//NUKE
//void processLatheCommand(int direction)
//{   // uses argS for speed
//	if (ARG_S_MISSING)
//	{
//		ARG_S = 0.0f;   // shut things down
//	}
//
//	if (_gs.latheMotor)
//	{   // lathe mode on a normal axis
//		MotorStructure *M = _gs.latheMotor;
//		if (motorArgPresent(M) && M->MotorInstalled)
//		{
//			if ((M->latheMode == LATHE_MODE_OFF) && (ARG_S != 0.0f))
//			{   // was off and not being used -- need to change things to set up lathe mode
//				enableLatheMode();
//			}
//			if ((M->latheMode != LATHE_MODE_OFF) || (ARG_S != 0.0f))
//			{   // !OFF = already running OR was off and want a non-zero rate
//				setupLatheModeToRun(M, (ARG_S / 60.0f) * M->PulsesPerUnit, (direction == HH_SPINDLE_CLOCKWISE) ? DIRECTION_FORWARD : DIRECTION_REVERSE);
//			}
//		}
//	}
//	if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT)
//	{
//		MotorStructure *M;
//		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
//		{   // whiz through all the motors, if there's a ARG from a non-legal lathe motor, send an error
//			if (motorArgPresent(M))
//			{
//				sprintf(_tmpStr, "%c", M->AxisLabel);
//				ReportInvalidMcodeArg(_tmpStr, getMotorArg(M));
//			}
//		}
//	}
//	SendCurrentMcodeExecutionNotice(TRUE);
//}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M253(void)  // turn on lathe motor for continuous CW motion (uses Axis)
{
	// MCODE M253 ["latheAxis" speed] ["P" forcePct]
	// MCODE
	// MCODE    if "latheAxis" is a canAxisMotor, then ANY axis can be used and will be locally controlled by the driver
	// MCODE    else "latheAxis" is present ("A" for EnginerHR_Hydra and "B" for EnginerSR_30M)
	// MCODE       set latheAxis to free spin at rate specified with the lathe axis in a CW direction
	// MCODE           pulseRate = (Arg / 60) * pulses_per_unit(from M92) ....
	// MCODE       set pulses_per_unit to pulses per REVOLUTION in order to work in RPM
	// MCODE    if argP specified, set the max allowable force to argP (if 0 or not specified, the head's default is used)
	// MCODE
	// MCODE    turn on lathe motor for continuous CW motion
	processLatheCommand(HH_SPINDLE_CLOCKWISE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M254(void)  // turn on lathe motor for continuous CCW motion (uses S)
{
	// MCODE M254 ["latheAxis" speed] ["P" forcePct]
	// MCODE
	// MCODE    if "latheAxis" is a canAxisMotor, then ANY axis can be used and will be locally controlled by the driver
	// MCODE    else "latheAxis" is present ("A" for EnginerHR_Hydra and "B" for EnginerSR_30M)
	// MCODE       set latheAxis to free spin at rate specified with the lathe axis in a CW direction
	// MCODE           pulseRate = (Arg / 60) * pulses_per_unit(from M92) ....
	// MCODE       set pulses_per_unit to pulses per REVOLUTION in order to work in RPM
	// MCODE    if argP specified, set the max allowable force to argP (if 0 or not specified, the head's default is used)
	// MCODE
	// MCODE    turn on lathe motor for continuous CCW motion
	processLatheCommand(HH_SPINDLE_COUNTER_CLOCKWISE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M255(void)  // turn off lathe motor
{
	// MCODE M255 ["canLatheAxis" 0]
	// MCODE M255	                     {stop direct connected lathe motor}
	ARG_S = 0.0f;   // force SPINDLE off
	processLatheCommand(HH_SPINDLE_OFF);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M256(void)  // Servo mode control (uses T, S, A)
{
	// MCODE M256 [T tool] [E 0|1] [A angle]
	// MCODE M256
	// MCODE    if
	// MCODE M256 ["T" toolSelector] <"E" enable> ["A" angle]
	// MCODE
	// MCODE    if argT is not present, the prior selected device/alias is used (must be a physical device -- no aliases
	// MCODE
	// MCODE    if argE is present and argE==1 AND argA is present, then enable (or keep enabled) servo mode
	// MCODE    else disable servo mode on device
	// MCODE
	// MCODE    if argA is present and argE==1, then set the desired servo angle to argA

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

//	payloadUnion payload;
//	payload.u8[0] = (ARG_E_PRESENT && (ARG_E == 1.0f) && ARG_A_PRESENT);	// enable/disable
//	payload.u16[1] = ARG_A_PRESENT ? getInboxPointer(localOutboxPtr->device)->motorTicksRev * (ARG_A / 360.0f) : 0;
//	canPackIntoTxQueue8x8(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_SERVO_MODE, BUFFERED_MSG, &payload);
//
//#define CLOSED_LOOP_SERVO_INIT_BY_TEACHING			10
//#define CLOSED_LOOP_SERVO_INIT_BY_CURR_MIN_MAX		11
//#define CLOSED_LOOP_SERVO_INIT_BY_FORCE				12
//#define CLOSED_LOOP_SERVO_RUN						13
//#define CLOSED_LOOP_SERVO_STOP						14
#ifdef GB_HIDDEN_WARNINGS
#warning "WORK IN PROGRESS - M256 SERVO"
#endif //GB_HIDDEN_WARNINGS
	canPackIntoTxQueue2x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_SERVO_RUN, BUFFERED_MSG,
			(ARG_E_PRESENT && (ARG_E == 1.0f) && ARG_A_PRESENT),	// enable/disable,
			ARG_A_PRESENT ? getInboxPointer(localOutboxPtr->device)->motorTicksRev * (ARG_A / 360.0f) : 0);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M260(void)
{
	return ;
}

//Parameter Name   Command  Control Type   Air Handler Fan
//M261 S###			1 - 100 – ON to preset percentage, 0 - OFF UV LED
//M262 S###			1 - 100 – ON to preset percentage, 0 - Off FAN
//M263 S###			1 – Engage, 0 – Disable  Door Lock
	

void M_Code_M261(void)  // set FAN Duty cycle and enable
{
	if (ARG_S_MISSING)return;//do nothing with out valid data
	
	if (ARG_F_PRESENT)SetCO2LaserPwmFrequency((int)ARG_F);
	TIM8->CCR2 = 101;//100 percent maximum duty cycle
	if (ARG_S > 100)ARG_S = 100;
	TIM8->CCR3 = (int)ARG_S;
	
	EnclosureFanPwm =(int) ARG_S;
	if (EnclosureFanPwm == 0)
	{
		changeHssDuty(&HighSideSwitches[8], 0);
	}
	else
	{
		changeHssDuty(&HighSideSwitches[8], 100);
	}
//	setupHssPwm(&HighSideSwitches[8]); 
	sendNotice(); //echo back the command
}
void M_Code_M262(void)  // set UVLED Duty cycle and enable
{
	if (ARG_S_MISSING)return;//do nothing with out valid data
	if (ARG_S > 0)ARG_S = 100.0f;
	EnclosureUvLedPwm = (int) ARG_S;
	changeHssDuty(&HighSideSwitches[1], (int) ARG_S); 
	sendNotice(); //echo back the command
}
void M_Code_M263(void)  // door lock
{
	if (ARG_S_MISSING)return;//do nothing with out valid data
	EnclosureDoorLock = (int) ARG_S;
	if (ARG_S > 0)ARG_S = 100.0f;
	changeHssDuty(&HighSideSwitches[3], (int) ARG_S); 
	changeHssDuty(&HighSideSwitches[6], (int) ARG_S); 
	sendNotice(); //echo back the command
}

////////////////////////////////////////////////////////////////////////////////

void setupHssPwm(HssPwmStruct *hss)  // sets the output X HSS PWM dutyCycle (uses S)
{
	// MCODE M60X <S PWM(%)> <P period(seconds)>
	// MCODE
	// MCODE    if SArg is missing  PWM is set to 100 (ALWAYS ON)
	// MCODE    else if SARG < 0,   PWM is set to 0 (ALWAYS OFF)
	// MCODE    else if SARG > 100, PWM is set to 100 (ALWAYS ON)
	// MCODE    else                PWM is set to SArg (useful range is 0 to 100%)
	// MCODE
	// MCODE    sets the output X HSS  PWM% and period values (P's resolution is at best 0.01 seconds)

	float dutyCycle;

	if (ARG_S_MISSING)
		dutyCycle = 100.0f;            // force always on
	else if (ARG_S < 1.0f)
		dutyCycle = 0.0f;              // force always off
	else if (ARG_S > 100.0f)
		dutyCycle = 100.0f;            // force always on
	else
		dutyCycle = (int)ARG_S;     // set to user's request

	if (ARG_P_PRESENT) hss->PeriodInSec = fFitWithinRange(ARG_P, 0.010f, 3600.0f); // 10ms to 1 hours

	changeHssDuty(hss, dutyCycle);
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M600 (void)  // disable all HSS
{
	// MCODE M600
	// MCODE
	// MCODE    disable all HSS
	int i;

	for (i=0; i<NUM_HSS_PINS; i++)
		changeHssDuty(&HighSideSwitches[i], HSS_DUTY_CYCLE_OFF);
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////
void M_Code_M601(void) {setupHssPwm(&HighSideSwitches[1]); } // MCODE enable HSS out1
void M_Code_M602(void) {setupHssPwm(&HighSideSwitches[2]); } // MCODE enable HSS out2
void M_Code_M603(void) {setupHssPwm(&HighSideSwitches[3]); }  // MCODE enable HSS out3
void M_Code_M604(void) {setupHssPwm(&HighSideSwitches[4]); }  // MCODE enable HSS out4
void M_Code_M605(void) {setupHssPwm(&HighSideSwitches[5]); }  // MCODE enable HSS out5
void M_Code_M606(void) {setupHssPwm(&HighSideSwitches[6]); }  // MCODE enable HSS out6
void M_Code_M607(void) {setupHssPwm(&HighSideSwitches[7]); }  // C02 Laser SSR 
void M_Code_M608(void) {setupHssPwm(&HighSideSwitches[8]); }  // CO2 Laser Pump
//now open drain control via 595

void M_Code_M610(void) { if (ARG_S_PRESENT)McodeDrainState[M610_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out10
void M_Code_M611(void) { if (ARG_S_PRESENT)McodeDrainState[M611_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out11
void M_Code_M612(void) { if (ARG_S_PRESENT)McodeDrainState[M612_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out12
void M_Code_M613(void) { if (ARG_S_PRESENT)McodeDrainState[M613_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out12
void M_Code_M614(void) { if (ARG_S_PRESENT)McodeDrainState[M614_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out12
void M_Code_M615(void) { if (ARG_S_PRESENT)McodeDrainState[M615_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out9
void M_Code_M616(void) { if (ARG_S_PRESENT)McodeDrainState[M616_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out9
void M_Code_M617(void) { if (ARG_S_PRESENT)McodeDrainState[M617_State_Ofset] = (int) ARG_S; } // MCODE enable HSS out9

void M_Code_M618(void)
{
	//setup pnp valve timer
	if (ARG_V_PRESENT)
	{
		PNPSPIData = 1 << ((uint16_t) ARG_V - 1);
	}
	if (ARG_D_PRESENT)
	{
		PnPResetTimer = (uint16_t)ARG_D;
	}
} // MCODE reserved

////////////////////////////////////////////////////////////////////////////////

void M_Code_M619(void)  // sets the function and output pwm of the selected HSS (uses F, I, S, P, J, H, O)
{
	// MCODE M619 <[F functionIndex] &| [I ioIndex]> [S PWM(%)] [P periodSec] [J timingLoop] [H polarity] [O oneShot]
	// MCODE
	// MCODE    FArg and Iarg select current function and/or phycical HSS to change
	// MCODE        only one needs to be spec'd.  if both are provided, the func will
	// MCODE        now be associated with the provided ioIndex (allowing remapping)
	// MCODE    if SArg is missing  PWM is left as the last set value
	// MCODE    else if SARG < 0,   PWM is set to 0 (ALWAYS OFF)
	// MCODE    else if SARG > 100, PWM is set to 100 (ALWAYS ON)
	// MCODE    else                PWM is set to SArg (0.0 to 100.0)
	// MCODE
	// MCODE    if PArg is not present, Period is left as the last set value
	// MCODE    else if PARG < 0,    Period is set to 1 second (default)
	// MCODE    else if PARG > 3600, Period is set to 3600 seconds (1 hour)
	// MCODE    else                 Period is set to PArg (useful range is 0 to 3600 seconds)
	// MCODE
	// MCODE    if JArg is not present, timingLoop is left as the last set value
	// MCODE    else if JARG is not one of 1, 10, 100, 1000 then timingLoop is aet to 100
	// MCODE    else timningLoop is set to JArg (ticksPerSecond)
	// MCODE
	// MCODE    if HArg is not present, polarity is left as the last set value
	// MCODE    else if HARG == 0.0f,   polarity is set to ACTIVE_LOW
	// MCODE                            polarity is set to ACTIVE_HIGH
	// MCODE
	// MCODE    if OArg is not present, oneShot flag is left as the last set value
	// MCODE    else if O_ARG == 1.0f,  neShot flag is set to TRUE
	// MCODE                            neShot flag is set to FALSE
	// MCODE
	// MCODE    sets the function and output pwm of the selected HSS
	// MCODE
	// MCODE    FUNCTIONS:
	// MCODE        NO_FUNCTION_HSS         = 0
	// MCODE        SPINDLE_COOLANT_HSS     = 1
	// MCODE        FLOOD_COOLANT_HSS       = 2
	// MCODE        DANGER_LIGHT_HSS        = 3
	// MCODE        DDLIGHT_HSS             = 4
	// MCODE        RESPONSE_HSS            = 5
	// MCODE        BUZZER_HSS              = 6
	// MCODE        CHAMBER_FAN_HSS         = 7
	// MCODE        LASER_XHAIR_HSS         = 8
	// MCODE        EXHAUST_FAN_HSS         = 9
	// MCODE        VACUUM_HSS              = 10
	// MCODE        AIR_ASSIST_HSS          = 11
	// MCODE        ULTIMUS_HSS             = 12
#ifdef NEW_CO2_LASER_CONTROL_MODULE // no 103 board at power supply; co2 laser lens used for soapstring, etc
	// MCODE        CO2_POWER_SUPPY_HSS     = 13
#endif //NEW_CO2_LASER_CONTROL_MODULE
	// MCODE
	// MCODE    IO INDICES (Engine/Sys30)
	// MCODE        X_HSS1_INDEX        = 1
	// MCODE        X_HSS2_INDEX        = 2
	// MCODE        Y_HSS1_INDEX        = 3
	// MCODE        Y_HSS2_INDEX        = 4
	// MCODE        Z_HSS1_INDEX        = 5
	// MCODE        Z_HSS2_INDEX        = 6
	// MCODE        W_HSS1_INDEX        = 7
	// MCODE        W_HSS2_INDEX        = 8
	// MCODE
	// MCODE    IO INDICES (Hydra)
	// MCODE        HSS_AUX_PWR1_INDEX  = 1
	// MCODE        HSS_AUX_PWR2_INDEX  = 2
	// MCODE        HSS_AUX_PWR4_INDEX  = 3
	// MCODE        HSS_AUX_PWR5_INDEX  = 4
	// MCODE        HSS_AUX_PWR6_INDEX  = 5
	// MCODE        HSS_AUX_PWR7_INDEX  = 6
	// MCODE        HSS_AUX_PWR8_INDEX  = 7
	// MCODE        HSS_AUX_PWR9_INDEX  = 8
	// MCODE        DRAIN1_INDEX        = 9
	// MCODE        DRAIN2_INDEX        = 10
	// MCODE        DRAIN3_INDEX        = 11
	// MCODE        DRAIN4_INDEX        = 12

	HssPwmStruct *hss;

	if (ARG_F_PRESENT)
	{
		if ((int)ARG_F < 1) ReportInvalidMcodeFArg();
		if ((int)ARG_F > NUM_HSS_FUNC) ReportInvalidMcodeFArg();
	}
	if (ARG_I_PRESENT)
	{
		if ((int)ARG_I < 1) ReportInvalidMcodeIArg();
		if ((int)ARG_I > NUM_HSS_PINS) ReportInvalidMcodeIArg();
	}
	if (ARG_F_MISSING && ARG_I_MISSING)
	{
		ReportMissingMcodeFArg();
		ReportMissingMcodeIArg();
	}
	int ioIndex = 0;
	if (ARG_F_PRESENT && ARG_I_PRESENT)
	{
		ioIndex = (int)ARG_I;
		hssFuncToPinIndex[(int)ARG_F] = ioIndex;
	}
	else if (ARG_F_PRESENT)
	{
		ioIndex = hssFuncToPinIndex[(int)ARG_F];
	}
	else if (ARG_I_PRESENT)
	{
		ioIndex = (int)ARG_I;
	}

	hss = &HighSideSwitches[ioIndex];

	if (ARG_S_PRESENT) hss->DutyCycle = ARG_S;
	if (ARG_J_PRESENT)
	{
		switch((int)ARG_J)
		{
		case 1:
		case 10:
		case 100:
		case 1000:  hss->TicksPerSecond = (int)ARG_J; break;
		default: ReportInvalidMcodeJArg(); break;
		}
	}
	if (ARG_P_PRESENT) hss->PeriodInSec = fFitWithinRange(ARG_P, 0.010f, 3600.0f); // 10ms to 1 hours
	if (ARG_H_PRESENT) hss->Output.Polarity = (ARG_H == 0.0f) ? ACTIVE_LOW : ACTIVE_HIGH;
	if (ARG_O_PRESENT) hss->oneShot = (ARG_O == 1.0f) ? TRUE : FALSE;
	updateHssPwm(hss);
}

////////////////////////////////////////////////////////////////////////////////

//WARNNG: for future "growth" of HSS's do NOT use M613-M619

////////////////////////////////////////////////////////////////////////////////

void M_Code_M620(void)  // Laser global control (uses T, E, D, F, A)
{
	// MCODE M620 <"T" device> ["E" enable(0)] ["D" localDrive(0)] ["F" freqPWM]  ["A" cooldownTime(30)]
	// MCODE
	// MCODE    ARG_T = required head/device number
	// MCODE    ARG_E = 1.0 to enable laser; off otherwise {def OFF}
	// MCODE    ARG_D = 0=CanBus control, 1=Local control {def CanBus control} {CO2 Laser only}
	// MCODE    ARG_F = pwm freq for power supply control (1 to 65K)  {def = CO2=20K, DIODE=2K}
	// MCODE    ARG_A = cooldown time (seconds) after laser powerdown
	// MCODE
	// MCODE    laser enable and timer setup

	if (ARG_T_MISSING && (ARG_E == 1.0f))
	{   // part of the safety setup. must explicity announce laser location if trying to turn it on
		ReportMissingMcodeTArg();
		if (_gs._laser.enabled == TRUE)
		{
			DeInitLaser();
		}
	}
	else
	{
		outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr(); // get pointer to laser head
		_gs._laser.enabled = FALSE; // set default
		_gs._laser.device    = localOutboxPtr->device;
		if ((getInboxPointer(_gs._laser.device)->deviceFamily == DEVICE_FAMILY_LASER) || (getInboxPointer(_gs._laser.device)->deviceFamily == DEVICE_FAMILY_INKJET))
		{   // make sure we're dealing with a laser/inkjet type device

			_gs._laser.outboxPtr    = getOutboxPointer(_gs._laser.device);
			_gs._laser.enabled      = ARG_E_MISSING ? FALSE : (ARG_E == 1.0f);
			_gs._laser.pwmFreq      = ARG_F_MISSING ? (deviceIsACO2Laser(_gs._laser.device) ? 20000 : 2000) : iFitWithinRange((int)ARG_F, 1, 0xffff); // uint16_t limitation
			_gs._laser.cooldown     = ARG_A_MISSING ? DEFAULT_LASER_COOLDOWN_TIME : iFitWithinRange((int)ARG_A, 1, 65535); // uint16_t limitation

			// to enable local control; device MUST be in a CO2 laser position AND ArgD==1)
			_gs._laser.localControl = (ARG_D_PRESENT && (ARG_D == 1.0f) && deviceIsACO2Laser(_gs._laser.device)) ? TRUE : FALSE;

			if (_gs._laser.localControl)
			{   // local control is active low, so make the default active low
				_gs._laser.polarity  = ACTIVE_LOW;
			}
			else
			{   // can control is active high, so make the default active high CAN_SetLaser
				_gs._laser.polarity  = ACTIVE_HIGH;
			}
		}
		if (_gs._laser.enabled && deviceIsACO2Laser(_gs._laser.device))
		{   // need to make sure there's is a LASER_LENS installed AND that there is no head installed to the left of it that could be hit by the beam
			// first, find laser_lens
			boolean anyoneInstalled = FALSE;
			boolean lensInstalled = FALSE;
			int i;
			for (i=0; i<NUM_PHYSICAL_DEVICES; i++)  // in physical order of 11-15, 21-25, etc
			{
				if (_MailBoxes._inbox[i].deviceRegistered)
				{
					if (_MailBoxes._inbox[i].deviceType == SOAP_DEV_TYPE_CO2_LENS)
					{   // found the lens
						lensInstalled = TRUE;
						if (anyoneInstalled)
						{   // ERROR...already detected a device installed to the left of the LENS position
							_gs._laser.enabled = FALSE;
							sprintf(_errorStr, "Trying to enable CO2 laser with a head in the laser beam path prior to the final laser mirror+lens (%d)", _MailBoxes._inbox[i].device);
							sendError(_errorStr);
						}
						break;
					}
					else
					{   // found another device
						anyoneInstalled = TRUE;
					}
				}
			}
			if (lensInstalled == FALSE)
			{
				_gs._laser.enabled = FALSE;
				sprintf(_errorStr, "Trying to enable CO2 laser without the laser lens installed");
				sendError(_errorStr);
			}
		}

		if (_gs._laser.enabled)
		{
			InitLaser();
			if (deviceIsACO2Laser(_gs._laser.device))
			{
				_g4DwellTimer = 4000; // force 4 second delay to allow water pressure to build
			}
			else
			{
				_g4DwellTimer = 1000; // force a 1 second delay to allow power supply to power up
			}
			ARG_S = ARG_E;  // just faking things for the completion notice
		}
		else
		{
			DeInitLaser();
			ARG_E = 0.0f; // just faking things for the completion notice
			ARG_S = 0.0f;
		}
	}
	SendCurrentMcodeExecutionNotice(TRUE);
}

//////////////////////////////////////////////////////////////////////////////

void M_Code_M621(void)  // Laser vector mode control (uses P, D)
{
	// MCODE M621 <"P" vectorPowerPct> ["D" piercePowerPct]
	// MCODE
	// MCODE    ARG_P = current vector laser power percentage (0 to 100)) {def = 0}
	// MCODE    ARG_D = current pierce laser power percentage (0 to 100)) {def = vectorPower}
	// MCODE
	// MCODE    laser vector mode control

	_gs._laser.vectorPowerPct  = ARG_P_MISSING ? 0.0f : (fFitWithinRange(ARG_P, 0.0f, 100.0f)) / 100.0f; // pct 0 to 1.0
	_gs._laser.piercePowerPct  = ARG_D_MISSING ? _gs._laser.vectorPowerPct : (fFitWithinRange(ARG_D, 0.0f, 100.0f)) / 100.0f; // pct 0 to 1.0

	SetLaserPower(_gs._laser.vectorPowerPct, 0); // setup desired power in head, but do not turn on
	if (_gs._laser.outboxPtr)
		Send_Prime_Prams(_gs._laser.outboxPtr);//update the hothead
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M622(void)  // Laser raster mode control (uses F, B, A)
{
	// MCODE M622  ["F" rasterRate] ["B" bidirScan] ["A" bidirStartAdjust]
	// MCODE    ARG_F = motion rate during rasterizing (def = prior Gx feedrate)
	// MCODE    ARG_B = 0 scan L->R only; 1 scan both directions
	// MCODE    ARG_A = added offset to start position for return trip on bidir scanning
	// MCODE
	// MCODE    laser raster mode control

	_gs._laser.rasterScanFeedrate           = ARG_F_MISSING ? _TargetFeedRateInMmPerSec : convertArgToMM(ARG_F) / (float)60.0;
	_gs._laser.rasterBidirectionalScan      = ARG_B_MISSING ? FALSE : (ARG_B == 1.0f) ? TRUE : FALSE;
	_gs._laser.rasterBidirReturnStartAdjust = ARG_A_MISSING ? RASTER_REVERSE_SCAN_TWEAK : RASTER_REVERSE_SCAN_TWEAK + ARG_A;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M623(void)  // Laser pulse (one-shot) mode control (uses P, D)
{
	// MCODE M623 ["P" pulsePowerPct] ["D" durationMs]
	// MCODE
	// MCODE    ARG_P = current laser power percentage (0-100) {def = 0}
	// MCODE    ARG_D = pulse duration in ms (0-1000) {def = 0}
	// MCODE
	// MCODE    laser pulse (one-shot) mode
	// Arg_A sets the ANALOG power PWM
	// Arg_F sets the frequency of pwm
	if (ARG_T_PRESENT)
	{//head is specified, so lets see if it is 41, meaning internal CO2 laser
		if (ARG_P_MISSING)return;//cant work without power specification
		if (ARG_D_MISSING)return;//need specific on dwell time in ms
		if (ARG_T == 41)
		{//new internal co2 laser control
			if (ARG_A_PRESENT)CO2LaserAnalogPwrPWM = (int)ARG_A;//set analog range
			if (ARG_F_PRESENT)SetCO2LaserPwmFrequency((int)ARG_F);
			Co2LaserWatchDogTimer = (int)ARG_D;
			TIM8->CCR3 = DesiredCo2LaserPower=(int)ARG_P;
			_gs._laser.enabled = 1;
			_gs._laser.localControl = 1;
			return;			
		}	
	}

float oneShotPowerPct   = ARG_P_MISSING ? 0.0f : (fFitWithinRange(ARG_P, 0.0f, 100.0f)) / 100.0f; // pct 0 to 1.0
	if (deviceIsAUvLightRay(_gs._laser.device))
		_gs._laser.watchdogMs   = ARG_D_MISSING ? 0 : iFitWithinRange((int)ARG_D, 0, MAX_MANUAL_UV_LIGHTRAY_PULSE_TIME_MS);
	else
		_gs._laser.watchdogMs   = ARG_D_MISSING ? 0 : iFitWithinRange((int)ARG_D, 0, MAX_MANUAL_LASER_PULSE_TIME_MS);
	SetLaserPower(oneShotPowerPct, _gs._laser.watchdogMs);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M624(void)  // setup raster image (uses B, X, Z, I, J, U, V, P, C, L, O)
{
	// MCODE M624 <"B" numBitsPerDot> <"X" X_origin> <"Y" Y_origin> <"I" X_DPUnit> <"J" Y_DPUnit>> <"U" imageDotsX> <"V" imageDotsY>
	// MODE       ["P" maxPwrPct] ["C" useColIdxTable] ["L" gammaVal] ["O" optimize] ["F" invertData]
	// MCODE    ARG_X, ARG_Y = starting position for first pixel (image built in increaing X/Y direction
	// MCODE    ARG_I, ARG_J = dots per mm of travel (float) for X/Y directions
	// MCODE    ARG_U, ARG_V = total image dots in X/Y
	// MCODE    ARG_P = max power percentage (0 to 100) {def = 100}
	// MCODE    ARG_C = (C===1.0) use color index table to map input value to new 8-bit value (see MXXX)
	// MCODE    ARG_L = set gamma function exponent for final data map to get laser power (-100 < ARG_O < 100 ..
	// MCODE            (no mapping otherwise -- gamma==1.0)
	// MCODE    ARG_O = (O==1.0) then optimize travel to avoid lasing "0" areas on the left and right side of a scan line (includes
	// MCODE            skipping blank lines ("0" for non-inverted and "11..." for inverted ... PRE COLOR INDEX)
	// MCODE    ARG_F = (F==1.0) invert data post-color index lookup tabl
	// MCODE
	// MCODE    final laser power is calculated as follows:
	// MCODE        inputData = nextValueFromDataStream  //range is 0 to (1-numBitsPerDot))
	// MCODE            if (inactiveDot)
	// MCODE                dotValueI = _gs._laser.rasterOffValue; // MCODE         force off
	// MCODE            else
	// MCODE                dotValueI = inputData;
	// MCODE
	// MCODE            unsigned int shiftAmount = ((_gs._laser.rasterImageDotsPerBinaryValue - 1) - dotSubPosition) * _gs._laser.rasterBitsPerDot;
	// MCODE            dotValueI = (dotValueI >> shiftAmount) & _gs._laser.rasterBinaryValueMask;
	// MCODE
	// MCODE            if (_gs._laser.rasterUseColorIndexTable)
	// MCODE                dotValueF = ((float)(_gs._laser.rasterColorIndexTable[dotValueI])) / 255.0f;
	// MCODE            else
	// MCODE                dotValueF = (float)dotValueI / ((1 << _gs._laser.rasterBitsPerDot) - 1);    // scale to fill range of 0 to 1.0 no matter the number of input pix
	// MCODE            if (_gs._laser.rasterInvertData)
	// MCODE                dotValueF = (-1.0f * dotValueF) + 1.0f;
	// MCODE            float laserPower = fFitWithinRange(dotValueF * _FlowRateOverridePct, 0.0f, 1.0f);
	// MCODE            laserPower = powf(laserPower, _gs._laser.gammaValue);       //  perform gamma
	// MCODE            laserPower = fFitWithinRange(laserPower, 0.0f, _gs._laser.rasterMaxPowerPercent);
	// MCODE
	// MCODE setup raster image

	boolean fail = FALSE;

	if (ARG_B_MISSING) { ReportMissingMcodeBArg(); fail = TRUE; }
	if (((int)ARG_B != 1) && ((int)ARG_B != 2) && ((int)ARG_B != 4) && ((int)ARG_B != 8)) { ReportInvalidMcodeBArgInt(); fail = TRUE; }
	if (ARG_X_MISSING) { ReportMissingMcodeXArg(); fail = TRUE; }
	if (ARG_Y_MISSING) { ReportMissingMcodeYArg(); fail = TRUE; }
	if (ARG_I_MISSING) { ReportMissingMcodeIArg(); fail = TRUE; }
	if (ARG_J_MISSING) { ReportMissingMcodeJArg(); fail = TRUE; }
	if (ARG_U_MISSING) { ReportMissingMcodeUArg(); fail = TRUE; }
	if (ARG_V_MISSING) { ReportMissingMcodeVArg(); fail = TRUE; }

	if (fail)
	{
		if (_gs._laser.enabled)
		{
			DeInitLaser();
		}
	}
	else
	{
		_gs._laser.rasterBitsPerDot = (int)ARG_B;
		_gs._laser.rasterImageDotsPerBinaryValue = 8 / _gs._laser.rasterBitsPerDot;
		_gs._laser.rasterBinaryValueMask = (1 << _gs._laser.rasterBitsPerDot) - 1;

		_gs._laser.rasterImageOriginMmX = convertArgToMM(ARG_X);
		_gs._laser.rasterImageOriginMmY = convertArgToMM(ARG_Y);
		_gs._laser.rasterImageDotsPerMmX = convertArgToMM(ARG_I);
		_gs._laser.rasterImageDotsPerMmY = convertArgToMM(ARG_J);
		// round up to match input width  boundary;
		_gs._laser.rasterImageDotsX = (int)ARG_U;
		_gs._laser.rasterImageDotsXmod = ((int)((int)((int)ARG_U + _gs._laser.rasterImageDotsPerBinaryValue - 1))  / _gs._laser.rasterImageDotsPerBinaryValue) * _gs._laser.rasterImageDotsPerBinaryValue;
		_gs._laser.rasterBytesPerFullLine = _gs._laser.rasterImageDotsXmod / _gs._laser.rasterImageDotsPerBinaryValue;
		_gs._laser.rasterImageDotsY = (int)ARG_V;

		_gs._laser.rasterMaxPowerPercent  = ARG_P_MISSING ? 1.0f : fFitWithinRange(ARG_P, 0.0f, 1.0f);
		_gs._laser.rasterUseColorIndexTable = ARG_C_MISSING ? FALSE : (ARG_C == 1.0f);
		_gs._laser.gammaValue =  (ARG_L_PRESENT && ((ARG_L > -100.0f) && (ARG_L < 100.0f))) ? 1.0f / ARG_L : 1.0f;
		_gs._laser.rasterSkipBlankBorders = ARG_O_MISSING ? FALSE : (ARG_O == 1.0f);
		_gs._laser.rasterInvertData = ARG_F_MISSING ? FALSE : (ARG_F == 1.0f);
		if (_gs._laser.rasterInvertData)
		{
			sendError("M624 - Invert Data mode is currently disabled");
		}
		if (_gs._laser.rasterInvertData)
			_gs._laser.rasterOffValue = 0xff;
		else
			_gs._laser.rasterOffValue = 0x00;

		_gs._laser.rasterLineExcessDots = (_gs._laser.rasterImageDotsPerBinaryValue - (_gs._laser.rasterImageDotsX % _gs._laser.rasterImageDotsPerBinaryValue)) % _gs._laser.rasterImageDotsPerBinaryValue;
		_gs._laser.rasterOffValueLastWord = (_gs._laser.rasterOffValue << (_gs._laser.rasterLineExcessDots * _gs._laser.rasterBitsPerDot)) & 0xff;

		_gs._laser.rasterLineCounter = 0;   // reset line count
		_gs._laser.rasterPulsesPerDot = round(Motors[M_X].PulsesPerUnit / _gs._laser.rasterImageDotsPerMmX);
		_gs._laser.rasterFrontPorchDots = (((sqr(_gs._laser.rasterScanFeedrate) / ( 2 * Motors[M_X].AccelerationConstant)) * _gs._laser.rasterImageDotsPerMmX) * RASTER_FRONT_PORCH_EXTRA_PIXELS_MULTIPLIER) + 1;
		_gs._laser.rasterTotalLineLength = (float)((_gs._laser.rasterImageDotsXmod + (2 * _gs._laser.rasterFrontPorchDots)) * _gs._laser.rasterPulsesPerDot) / Motors[M_X].PulsesPerUnit;


		float dotsPerSec = _gs._laser.rasterScanFeedrate * _gs._laser.rasterImageDotsPerMmX;
		int pwmFreq = imin(_gs._laser.pwmFreq, getOutboxPointer(_gs._laser.device)->hardInitPtr->maxPwmFreq);
		if (dotsPerSec > pwmFreq)
		{	// dotRate exceeds pwmRate --- cannot resolve dot -- throttle feedrate
			_gs._laser.rasterScanFeedrate = (float)pwmFreq / _gs._laser.rasterImageDotsPerMmX;
			sprintf(_tmpStr, "Raster dot rate exceeds pwmRate of the head (%4.1f vs %d). Limiting scan rate to F%d", dotsPerSec, pwmFreq, (int)_gs._laser.rasterScanFeedrate*60);
			sendError(_tmpStr);

		}

		if ((_sendingGBStringsMask & GB_STRING_RASTER) && (_sendingGBStringsSubMask & GB_STRING_RASTER_SUBMASK_CALIB)) // use M797 S<mask> to enable
		{
			sprintf(_tmpStr, "LASER BIDIR CALIB:  steps/dot=%d  mm/step=%7.5f  mm/dot=%7.5f", _gs._laser.rasterPulsesPerDot, Motors[M_X].UnitsPerPulse, (float)_gs._laser.rasterPulsesPerDot * Motors[M_X].UnitsPerPulse);
			sendGB(_tmpStr);
			sprintf(_tmpStr, "BIDIR ADJUST = vernier_steps * %7.5f", (float)_gs._laser.rasterPulsesPerDot * Motors[M_X].UnitsPerPulse);
			sendGB(_tmpStr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M625(void)  // inkjet vector mode control (uses D, J)
{
	// MCODE M625 ["D" droplets/mm] ["J" jet]

	// MCODE    ARG_D = droplets per mm of travel (def = 8)
	// MCODE    ARG_J = current jet to use
	// MCODE
	// MCODE    inkjet vector mode control

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_D_PRESENT)
		_gs._inkjet.dropletsPerMm = fFitWithinRange(ARG_D, 0.0f, 100000.0f);    // droplets/mm
	else
		_gs._inkjet.dropletsPerMm = DEFAULT_INKJET_DROPLETS_PER_MM;

	if (ARG_J_PRESENT)
	{
		canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_DEV, DEV_INIT_INDEX_DEV_INKJET_NOZZLE, (uint16_t)ARG_J);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M626(void)  // build color index table (uses C, U, A, D)
{
	// MCODE M626 ["C" clearValue] ["U" unityBitsPerDot] [<"A" addr> <"D" data>]

	// MCODE    if ARG_C_PRESENT = force entire table to ARG_C (range 0 to 255)
	// MCODE    else if ARG_U_PRESENT = force table to unity mapping based on ARG_U (bits per pixel)
	// MCODE       ARG_U==1 :  tab[0]=0x00 tab[1]=0xff
	// MCODE       ARG_U==2 :  tab[0]=0x00 tab[1]=0x55 tab[2]=0xaa tab[3]=0xff
	// MCODE       ARG_U==4 :  tab[0]=0x00 tab[1]=0x11 ... tab[14]=0xee tab[15]=0xff
	// MCODE       ARG_U==8 :  tab[0]=0x00 tab[1]=0x01 ... tab[254]=0xfe tab[255]=0xff
	// MCODE    else if (ARG_A_PRESENT && ARG_D_PRESENT)  (ARG_A and ARG_D must be in range 0 to 255)
	// MCODE       tab[ARG_A] = ARG_D
	// MOCDE
	// MCODE   build color index table for raster data

	int i;
	byte value;

	if (ARG_C_PRESENT)
	{
		if (((int)ARG_C < 0) || ((int)ARG_C > 255)) { ReportInvalidMcodeCArgInt(); return; }
		value = iFitWithinRange((int)ARG_C, 0, 255);
		for (i=0; i<256; i++)
		{
			_gs._laser.rasterColorIndexTable[i] = value;
		}
	}
	else if (ARG_U_PRESENT)
	{
		if (((int)ARG_U != 1) && ((int)ARG_U != 2) && ((int)ARG_U != 4) && ((int)ARG_U != 8)) { ReportInvalidMcodeUArgInt(); return; }
		for (i=0; i<(1<<((int)ARG_U)); i++)
		{
			switch ((int)ARG_U)
			{
			case 1: value = (i==0) ? 0x00 : 0xff; break;
			case 2: value = i << 6 | i << 4 | i << 2 | i; break;
			case 4: value = i << 4 | i; break;
			case 8: value = i; break;
			default: value = 0; break;
			}
			_gs._laser.rasterColorIndexTable[i] = value;
		}
	}
	else if (ARG_A_PRESENT && ARG_D_PRESENT)
	{
		if (((int)ARG_A < 0) || ((int)ARG_A > 255)) { ReportInvalidMcodeAArgInt(); return; }
		if (((int)ARG_D < 0) || ((int)ARG_D > 255)) { ReportInvalidMcodeDArgInt(); return; }
		_gs._laser.rasterColorIndexTable[(int)ARG_A] = (int)ARG_D;
	}
}

////////////////////////////////////////////////////////////////////////////////
#if 1
void M_Code_M627(void)  // set job_kill/abort auto move location (uses XYZABC IJKUVW)
{
	// MCODE M627 ["XYZABC" absoluteMove] ["IJKUVW" incrementalMove]
	// MCODE
	// MCODE    Set up the requested motion that is to occur in the event an abort/reset signal
	// MCODE    is received from the host.  This motion will only occur if motion was underway
	// MCODE    at the time the absort signal was received.
	// MCODE
	// MCODE    Two rapid moves will occur as soon as possible after the current motion is brought to
	// MCODE    a safe stop.
	// MCODE        move1: incremental move to [currentPosition] + IJKUVW
	// MCODE        move2: absolute move to XYZABC
	// MCODE
	// MCODE    set job_kill/abort auto move location

//  job start code:
//  if (Z_Drop_value != 0)
//      send("M627 Z<Z_Drop_value>");  // DO NOT SEND if Z is 0 (will override user settings)
	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
		M->AbortAbsolutePosition = getMotorArgInNativeUnits(M);
	}

	// in order to use motorArgPresent/getMotorArgInNativeUnits, need to move IJKUVW into XYZABC args (which
	// are no longer needed as they have been archived.
	ARG_X = ARG_I;
	ARG_Y = ARG_J;
	ARG_Z = ARG_K;
	ARG_A = ARG_U;
	ARG_B = ARG_V;
	ARG_C = ARG_W;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
		M->AbortRelativePosition = getMotorArgInNativeUnits(M);
	}
}

////////////////////////////////////////////////////////////////////////////////
#ifdef GB_HIDDEN_WARNINGS
#warning "unfinished business -- to improve M627 to define a whole range of moves during certain cases"
#endif //GB_HIDDEN_WARNINGS
#else
#warning "unfinished business -- to improve M627 to define a whole range of moves during certain cases"
typedef enum {
	AUTO_MOVE_ABORTED_MOTION = 0,
	AUTO_MOVE_PAUSE_END_OF_MOVE,
	AUTO_MOVE_PAUSE_END_OF_LAYER,
	AUTO_MOVE_PAUSE_END_OF_MQ,
	AUTO_MOVE_FILAMENT_OUT,
	AUTO_MOVE_LAST_ENTRY
} autoMove_t;

//FOLD THESE IN TO THE MOTOR STRUCTURE
typedef struct {
	float positionBeforeMove[MAX_NUMBER_OF_MOTORS];			// save location in order to return afterwards
	float positionAfterRelativeMove[MAX_NUMBER_OF_MOTORS];			// save location after the relative move .. will use as midpoint when returning
	float relativePositionInUnits[MAX_NUMBER_OF_MOTORS];	// initial relative move to execute when condition occur
	float absolutePositionInUnits[MAX_NUMBER_OF_MOTORS];	// absolute move to execute after the relative mode
	float feedrate; //or rate code
} autoMoveStruct_t;

autoMoveStruct_t _autoMoves[AUTO_MOVE_LAST_ENTRY];

void M_Code_M627(void)  // set auto move parameters (uses S, F, XYZABC, IJKUVW)
{
	// MCODE M627  [S moveIndex] ["XYZABC" absoluteMove] ["IJKUVW" incrementalMove]
	// MCODE
	// MCODE    Set up the requested motion that is to occur in the event an abort/reset signal
	// MCODE    is received from the host.  This motion will only occur if motion was underway
	// MCODE    at the time the absort signal was received.
	// MCODE
	// MCODE    Two rapid moves will occur as soon as possible after the current motion is brought to
	// MCODE    a safe stop.
	// MCODE        move1: incremental move to [currentPosition] + IJKUVW
	// MCODE        move2: absolute move to XYZABC
	// MCODE
	// MCODE    set auto move parameters

	MotorStructure *M;

	autoMove_t index = AUTO_MOVE_ABORTED_MOTION;	//backward compatibility with orig command when no argS was used

	if ((ARG_S_PRESENT && ((int)ARG_S < 0)) || (ARG_S_PRESENT && ((autoMove_t)ARG_S < AUTO_MOVE_LAST_ENTRY)))  { ReportInvalidMcodeSArgInt(); return; }
	else index = (autoMove_t)ARG_S;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
		M->AbortAbsolutePosition = getMotorArgInNativeUnits(M);
		M->AutoMove[index].absolutePositionInUnits = getMotorArgInNativeUnits(M);
		_autoMove[index].absolutePositionInUnits[M->Axis] = getMotorArgInNativeUnits(M);
	}

	// in order to use motorArgPresent/getMotorArgInNativeUnits, need to move IJKUVW into XYZABC args (which
	// are no longer needed as they have been archived.
	ARG_X = ARG_I;
	ARG_Y = ARG_J;
	ARG_Z = ARG_K;
	ARG_A = ARG_U;
	ARG_B = ARG_V;
	ARG_C = ARG_W;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then load max distance
		M->AbortRelativePosition = getMotorArgInNativeUnits(M);
		M->AutoMove[index].relativePositionInUnits = getMotorArgInNativeUnits(M);
		_autoMove[index].relativePositionInUnits[M->Axis] = getMotorArgInNativeUnits(M);
	}
}
#endif
////////////////////////////////////////////////////////////////////////////////

void M_Code_M628(void)  // arm/disarm digital trigger (uses P, E, R) [system only, not for heads)
{
	// MCODE M628  <"P" probe#  <"E" edge(s)> ["R" maxRearmingRateHz (def=1 sec)
	// MCODE
	// MCODE    argP selects which pins to look at for the probe:
	// MCODE         0 : None (off)
	// MCODE         1 : Limit1 pin on X axis connector or W_RTD1 (sys30)
	// MCODE         2 : Limit2 pin on X axis connector or W_RTD2 (sys30)
	// MCODE         3 : Fault  pin on X axis connector
	// MCODE         4 : Home   pin on X axis connector
	// MCODE         5 : unused
	// MCODE         6,7,8,9 : Limit1,Limit2,Fault,Home pin on Y axis connector
	// MCODE         10 : unused
	// MCODE         11,12,13,14 : Limit1,Limit2,Fault,Home pin on Z axis connector
	// MCODE         15 : unused
	// MCODE         16,17,18,19 : Limit1,Limit2,Fault,Home pin on A axis connector
	// MCODE         20 : unused
	// MCODE         21,22,23,24 : Limit1,Limit2,Fault,Home pin on B axis connector
	// MCODE         25 : unused
	// MCODE         26,27,28,29 : Limit1,Limit2,Fault,Home pin on C axis connector
	// MCODE    argE selects which edge(s) will trigger messages
	// MCODE       0 = None (off)
	// MCODE       1 = Falling edge only
	// MCODE       2 = Rising edge only
	// MCODE       3 = Both rising and falling edges
	// MCODE
	// MCODE    event trigger - message sent to host

	if (ARG_E_MISSING) { ReportMissingMcodeEArg(); return; }
	if (ARG_P_MISSING) { ReportMissingMcodePArg(); return; }

	if ((ARG_P < 0.0f) || ((int)ARG_P > 29)) { ReportInvalidMcodePArg(); return; }

	if ((ARG_P == 0.0f) || (ARG_E == 0.0f))
	{
		DisableEXTI();
		_edgeTriggerArmed = FALSE;
		_edgeTriggerDetected = FALSE;
	}
	else
	{
		EXTITrigger_TypeDef edgeType;
		int initState = 0;
		switch((int)ARG_E)
		{
		case 1: edgeType = EXTI_Trigger_Rising;  initState = 0; break;
		case 2: edgeType = EXTI_Trigger_Falling; initState = 1; break;
		case 3: edgeType = EXTI_Trigger_Rising_Falling; break;
		default: edgeType = 0; break;
		}
		if (SetupEXTI((int)ARG_P, PROBE_TYPE_NONE, edgeType, initState))
		{
			_edgeTriggerArmed = TRUE;
			if (ARG_R <= 0.00001f)
				_edgeTriggerDisplayRateHz = 0.00001f;   // approx once a day
			else
				_edgeTriggerDisplayRateHz = ARG_R;
		}
		else
		{
			DisableEXTI();
			_edgeTriggerArmed = FALSE;
			_edgeTriggerDetected = FALSE;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M629(void)  // open log file
{
	// MCODE M629 ; logFile
	 SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M630(void)  // canbus touch probe control (uses T, S, D, P, C, I, E)
{
	// MCODE M630 <"T" probe> <"S" action> <"D" probeType> ["C" count/control] ["P" period] ["I" initState] ["E" externalPin]
	// MCODE
	// MCODE    ARG_S
	// MCODE       0 = PROBE_ACTION_STOP
	// MCODE       1 = PROBE_ACTION_ARM
	// MCODE       2 = PROBE_ACTION_READ
	// MCODE       3 = PROBE_ACTION_CONTROL
	// MCODE    ARG_D
	// MCODE       0 = PROBE_TYPE_NONE
	// MCODE       1 = PROBE_TYPE_CONTACT
	// MCODE       2 = PROBE_TYPE_BLTOUCH
	// MCODE       3 = PROBE_TYPE_HC_SR04
	// MCODE       4 = PROBE_TYPE_IGAGING
	// MCODE
	// MCODE    ARG_C - repeat count (0=OFF, 1=once, n=numTimes, -1=continuous)
	// MCODE       or
	// MCODE     command
	// MCODE       0 = BLTOUCH_OFF
	// MCODE       1 = BLTOUCH_ARM
	// MCODE       2 = BLTOUCH_PROBE_DOWN
	// MCODE       3 = BLTOUCH_PROBE_UP
	// MCODE       4 = BLTOUCH_SELFTEST
	// MCODE       5 = BLTOUCH_ALARM_AND_UP
	// MCODE       6 = BLTOUCH_ALARM_AND_SW
	// MCODE       0 = IGAGING_CLEAR_ZERO_OFFSET
	// MCODE       1 = IGAGING_SET_ZERO_OFFSET
	// MCODE
	// MCODE    ARG_P (for BLTOUCH control or period in seconds for continuous mode)
	// MCODE       period == resolution 0.01sec [ie, P=0.05 will report every 50ms]
	// MCODE
	// MCODE    ARG_I - initial state of contact probe
	// MCODE
	// MCODE    ARG_E - selects which pins to use on the probe head
	// MCODE         0 : PA3 pin on 10-pin connector  (default)
	// MCODE         1 : PA2 pin on 10-pin connector
	// MCODE         2 : LIMIT1 pin on 18-pin connector
	// MCODE         3 : LIMIT2 pin on 18-pin connector
	// MCODE         4 : RTD1 pin on 18-pin connector
	// MCODE         5 : RTD2 pin on 18-pin connector
	// MCODE
	// MCODE    canbus touch/measuring probe control

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (ARG_D_MISSING) { ReportMissingMcodeDArg(); return; }
	probeAction_t probeAction = (probeAction_t)(int)ARG_S;
	probeType_t probeType = (probeType_t)(int)ARG_D;
	HH_IoIndex_t pin = ARG_E_PRESENT ? (HH_IoIndex_t)ARG_E : HH_PA3;

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	_TouchProbeCanDevice = localOutboxPtr->device;

	if (probeAction == PROBE_ACTION_READ)
	{
		canIssueReadRequest(localOutboxPtr->device, CAN_MSG_TOUCH_PROBE, probeType);
	}
	else
	{
		int value = 0;
		int count = 0;
		if (probeAction == PROBE_ACTION_ARM)
		{
			value = ARG_P_PRESENT ? (int)(100.0f*ARG_P) : 1;    // value is the period, in 1/100Hz increment
			if (ARG_C_MISSING)
				count = 1; // default to one shot
			else if (ARG_C == -1.0f)
				count = HH_CONTINUOUS_EXTRUSION_STEPS; // max value to head will signify continuous
			else
				count = (uint16_t)ARG_C;

		}
		else if (probeAction == PROBE_ACTION_CONTROL)
		{
			value = ARG_C_PRESENT ? (int)(ARG_C) : 0;   // value is raw command
			count = 1;
		}
		int initState = (ARG_I == 1.0f) ? 1 : 0;
		sendProbeControlToDevice(_TouchProbeCanDevice, probeType, probeAction, value, count, initState, pin);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M631(void)  // PickNPlace data (uses T, H, P, A, B, C, D)
{
	// MCODE M631 ["T" toolSelector] <"H" motor> <"P" dataPage> ["ABCD" dataVal]
	// MCODE
	// MCODE    if ArgT is not present, the prior selected device/alias is used
	// MCODE    if ArgH is missing, error. Otherwise, it is the motor:
	// MCODE        0 = RIGHT motor
	// MCODE        1 = LEFT motor
	// MCODE        2 = CENTER motor
	// MCODE    if ArgP is missing, error. Otherwise, it is the data page selector as follows:
	// MCODE    ArgP selects the page to load and ABCD are the 4 values for that page
	// MCODE        {all values are interpreted as uint16_t); any individual value can be omitted
	// MCODE        and it will not update.
	// MCODE
	// MCODE        ArgP    ArgA            ArgB            ArgC            ArgD            Subject
	// MCODE        ----    --------------  --------------  --------------  --------------  --------------
	// MCODE        0       HomeHysteris    HomePopOff      ParkPosition    MaxTravel       Positions
	// MCODE        1       HomeMotRate     ReHomeMotRate   ParkMotRate     MotionRate      Motion Rates
	// MCODE        2       PickDownPos     PickVacPos      PickWaitTimeMs  VacAcqThresh    Pick Related
	// MCODE        3       PlaceDownPos    PlaceVacPos     PlaceWaitTimeMs VacLostThresh   Place Related
	// MCODE
	// MCODE        NOTE:  dataPages 2 and 3 are for RIGHT and LEFT motors only)
	// MCODE
	// MCODE    PickNPlace data load

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	if (ARG_H_MISSING) { ReportMissingMcodeHArg(); return; }
	if (ARG_P_MISSING) { ReportMissingMcodePArg(); return; }

	pnpPageUnion canPage;

	canPage.data.motor          = (int)ARG_H;
	canPage.data.pageOfs        = (int)ARG_P;
	canPage.data.writeEnables   = (ARG_D_PRESENT<<3) | (ARG_C_PRESENT<<2) | (ARG_B_PRESENT<<1) | ARG_A_PRESENT;

	canPackIntoTxQueue4x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_PICKNPLACE_DATA, (byte)canPage.word, BUFFERED_MSG,
			(uint16_t)ARG_A, (uint16_t)ARG_B, (uint16_t)ARG_C, (uint16_t)ARG_D);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M632(void)  // PickNPlace control (uses T, S, H, P, V, F, D, W)
{
	// MCODE M632 ["T" toolSelector] <"S" cmd> <"H" motor> ["P" park] ["V" autoVac/vacState] ["F" motionRate] ["D" dest] ["W"wait]
	// MCODE
	// MCODE    if ArgT is not present, the prior selected device/alias is used
	// MCODE    if ArgS is missing, error. otherwise, it is the command:
	// MCODE        0 - IDLE
	// MCODE        1 = HOME            {optional D and F used}
	// MCODE        2 = PARK            {optional D and F used}
	// MCODE        3 = MOVE_ASBSOLUTE  {optional D and F used}
	// MCODE        4 = MOVE_RELATIVE   {optional D and F used}
	// MCODE        5 = PICK            {optional F used}
	// MCODE        6 = PLACE           {optional F used}
	// MCODE        7 = VACUUM_CONTROL  {V1 to turn on vac}
	// MCODE        8 = SHOOTER_ADVANCE (W=waitMS; ABCD=4 bytes of mask  (max 32 shooters)
	// MCODE    if ArgH is missing, error. Otherwise, it is the motor:
	// MCODE        0 = RIGHT motor
	// MCODE        1 = LEFT motor
	// MCODE        2 = CENTER motor
	// MCODE    if (ArgP==1) && (HOME/PICK/PLACE) then go to park position after move
	// MCODE    if (ArgV==1) && (PICK/PLACE) then enable auto vacuum during move (disable otherwise)
	// MCODE    if (ArgV==1) && (ArgS==7) then enable vacuum (disable otherwise)
	// MCODE    if (ArgF_present) set PnP motion rate to ArgF
	// MCODE    if (ArgD_present) set PnP motion dest to ArgD
	// MCODE    if (ArgW==1) then XYZ motion will pause until the commands completes (except shooter advance
	// MCODE		where ArgW = number of milliseconds to enable relay (motion paused as well)
	// MCODE
	// MCODE    PickNPlace control

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (((pnpCommand_t)ARG_S != PNP_CMD_REEL_ADVANCE) && ARG_H_MISSING) { ReportMissingMcodeHArg(); return; }

	pnpPageUnion canPage;

	canPage.ctrl.motor          = (int)ARG_H;
	canPage.ctrl.autoVacEn      = (ARG_V == 1.0f);
	canPage.ctrl.parkAfterCmd   = (ARG_P == 1.0f);
	canPage.ctrl.cmd            = 1;    // load command
	canPage.ctrl.dest           = ARG_D_PRESENT;
	canPage.ctrl.rate           = ARG_F_PRESENT;

	if ((pnpCommand_t)ARG_S == PNP_CMD_REEL_ADVANCE)
	{
		canPackIntoTxQueue4x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_PICKNPLACE_CONTROL, (byte)canPage.word, BUFFERED_MSG,
				(uint16_t)ARG_S,
				(uint16_t)imax((ARG_W_PRESENT ? (uint16_t)fFitWithinRange(ARG_W, 0.0f, 65535.0f) : PNP_WAIT_REEL_ADVANCE_DEFAULT_TIME_MS), PNP_WAIT_REEL_ADVANCE_DEFAULT_TIME_MS),
				(((ARG_B_PRESENT ? (byte)ARG_B : 0) << 8) | (ARG_A_PRESENT ? (byte)ARG_A : 0)),
				(((ARG_D_PRESENT ? (byte)ARG_D : 0) << 8) | (ARG_C_PRESENT ? (byte)ARG_C : 0)));
	}
	else
	{
		canPackIntoTxQueue3x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_PICKNPLACE_CONTROL, (byte)canPage.word, BUFFERED_MSG,
				(uint16_t)ARG_S, (int16_t)ARG_D, (uint16_t)ARG_F);
	}
	if (ARG_W_PRESENT && (ARG_W != 0.0f))
	{
		_MailBoxes._waitingFor.flags.bit.motionPnP = TRUE;
		_MailBoxes._waitingFor.timerPnP = PNP_WAIT_FOR_MOTION_TO_COMPLETE_TIME; // countdown timer load (for watchdog)
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void M_Code_M660(void)  //set tool diameter and length (uses H, D, Z)
{
	// MCODE M660 <"H" tool> ["D" diameter] ["Z" length]
	// MCODE
	// MCODE    if HArg in not present, error
	// MCODE    else if HArg <= 0, error
	// MCODE    else if HArg > MAX_TOOL, error
	// MCODE    else load tool diameter and length
	// MCODE
	// MCODE set tool offset: diameter and length

	if (ARG_H_MISSING) { ReportMissingMcodeHArg(); return; }
	int toolIndex=(int) ARG_H;
	if (toolIndex<0){ReportMcodeError("ToolIndex (ARG_H) Out of Range, Less than Zero");return;}
	if (toolIndex>=NUM_TOOL_OFFSETS){ReportMcodeError("ToolIndex (ARG_H) Out of Range, Too Big");return;}

	if (ARG_D_PRESENT) ToolOffsets[toolIndex].ToolDiameter = convertArgToMM(ARG_D);
	if (ARG_Z_PRESENT) ToolOffsets[toolIndex].ToolLength = convertArgToMM(ARG_Z);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M670(void)  // sets the Y-Axis/Diagnostic light PWM value (uses S)
{
	// MCODE M670 <S PWM(%)> <P period(seconds)>      // Y-Axis/Diagnostic light

	if (!((_gs._flasher.lightSel == FLASHER_LIGHT_SEL_DDL) || (_gs._flasher.lightSel == FLASHER_LIGHT_SEL_BOTH)))
	{
		setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]]);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M671(void)  // sets the DangerHot light PWM value (uses S)
{
	// MCODE M671 <S PWM(%)> <P period(seconds)>      // DangerHot light

#ifdef USE_HYREL_IO
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[DANGER_LIGHT_HSS]]);
#endif
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M672(void)  // sets the select for controlling the Y-Axis light (uses S, O)
{
	// MCODE M672 <S select> [O DDL_sel]
	// MCODE
	// MCODE    if OArg, the set the HSS index for the DDL to OArg
	// MCODE    if SArg is not present, error
	// MCODE    else select is set to SArg
	// MCODE
	// MCODE    sets the select for controlling the Y-Axis light

	if (ARG_O_PRESENT)
	{
		hssFuncToPinIndex[DDLIGHT_HSS] = iFitWithinRange((int)ARG_O, 0, NUM_HSS_PINS);
	}

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }

	DDLightFunction=(int)ARG_S;
	updateHssPwm(&HighSideSwitches[hssFuncToPinIndex[DDLIGHT_HSS]]);    // get controls ready for the new selection.
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////
void M_Code_M673(void) { M_Code_M670(); }  // sets the Y-Axis Bar Light PWM value synchronized to motion (uses S)
////////////////////////////////////////////////////////////////////////////////

void M_Code_M674(void)  // set turboModeDistance (convert non-print G1 to G0) (uses S)
{
	// MCODE M674 <"S" turboModeDistance>
	// MCODE
	// MCODE    if SArg in not present or SArg<0, error
	// MCODE    else turboMode enabled as follows:
	// MCODE        any non printing G1 moves, with a move length greater than the
	// MCODE        turboModeDistance will move at the pre=defined RAPID rate (G0 rate)
	// MCODE		instead of the FEEDRATE
	// MCODE
	// MCODE set turboModeDistance (convert non-print G1 to G0) (uses S)
	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (ARG_S < 0.0f) { ReportMcodeError("negative Sarg not allowed"); return; }
	TurboModeDistance = ARG_S;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M675(void)   //sets the response light hss pwm
{
	// MCODE M675 <S PWM(%)> <P period(seconds)>      // response light
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[RESPONSE_HSS]]);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M676(void)   //sets the chamber fan pwm
{
	// MCODE M676 <S PWM(%)> <P period(seconds)>      // chamber fan
#ifdef USE_HYDRA_IO
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[CHAMBER_FAN_HSS]]);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M677(void)   //sets the control panel buzzer pwm
{
	// MCODE M677 <S PWM(%)> <P period(seconds)>      // buzzer
#ifdef USE_HYREL_IO
	setupHssPwm(&HighSideSwitches[BUZZER_HSS]);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M678(void)   //set the laser cross-hair pwm
{
	// MCODE M678 <S PWM(%)> <P period(seconds)>      // laser cross-hair
#ifdef USE_HYREL_IO
	setupHssPwm(&HighSideSwitches[BUZZER_HSS]);
#endif
#ifdef USE_HYDRA_IO
	int i;
	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)  // in physical order of 11-15, 21-25, etc
	{
		if (_MailBoxes._inbox[i].deviceRegistered && (_MailBoxes._inbox[i].deviceType == SOAP_DEV_TYPE_CO2_LENS))
		{   // found the lens
			ARG_T = _MailBoxes._inbox[i].device;    //crosshair should be connected to final lens assembly
			outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
			sendSwitchControlByDutyCycleToDevice(localOutboxPtr, HH_AUX_SWITCH, ConvertArg0to100ToDutyCycle0To100("S", ARG_S));
			SendCurrentMcodeExecutionNotice(TRUE);
			break;
		}
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M679(void)   //set the vacumm pwm
{
	// MCODE M679 <S PWM(%)> <P period(seconds)>      // vacuum
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[VACUUM_HSS]]);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M680(void)  //enable CO2 power supply ac power the vacumm pwm
{
	// MCODE M680 <S PWM(%)> <P period(seconds)>      // vacuum
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[CO2_POWER_SUPPY_HSS]]);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M681(void)  //enable CO2 cooling pump
{
	// MCODE M681 <S PWM(%)> <P period(seconds)>      // vacuum
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[CO2_POWER_SUPPY_HSS]]);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M682(void)  // start Z axis sensor calibration run
{
	// MCODE M682 ["P" pulsePowerPct] ["D" durationMs]
	// MCODE
	// MCODE    ARG_P = current laser power percentage (0-100) {def = 0}
	// MCODE    ARG_D = pulse duration in ms (0-1000) {def = 0}
	// MCODE
	// MCODE    laser pulse (one-shot) mode
	// Arg_A sets the ANALOG power PWM
	// Arg_F sets the frequency of pwm
	if (ARG_T_PRESENT)
	{
		//head is specified, so lets see if it is 41, meaning internal CO2 laser
		if (ARG_P_MISSING)return;//cant work without power specification
		//if (ARG_D_MISSING)return;//need specific on dwell time in ms
		if (ARG_T == 44)//44 is the chamber Fan
		{
			//new internal co2 laser control
			if (ARG_F_PRESENT)SetCO2LaserPwmFrequency((int)ARG_F);
			TIM8->CCR2 = 101;
			TIM8->CCR3 = (int)ARG_P;
			//if(ARG_F_PRESENT)
			return;			
		}	
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M683(void)  // set the headroom for the normal serial rx buffer (uses S)
{
	// MCODE M683 <"S" normalRxBufHeadroom>
	// MCODE
	// MCODE set the headroom for the normal serial rx buffer

   if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
   if ((ARG_S < 0) || (ARG_S > SERIAL_RX_NORMAL_BUFFER_SIZE)) { ReportInvalidMcodeSArg(); return; }

   normalRxBufHeadroom = iFitWithinRange((int)ARG_S, (int)(SERIAL_RX_NORMAL_BUFFER_SIZE*0.05), SERIAL_RX_NORMAL_BUFFER_SIZE/2);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M685(void)   //sets the air assist pwm
{
	// MCODE M685 <S PWM(%)> <P period(seconds)>      // air assist (compressor)
#ifdef USE_HYDRA_IO
	setupHssPwm(&HighSideSwitches[hssFuncToPinIndex[AIR_ASSIST_HSS]]);
#endif
}

////////////////////////////////////////////////////////////////////////////////


void real_M_Code_M686(void) {} // RESERVED
void real_M_Code_M687(void) {} // RESERVED
void real_M_Code_M688(void) {} // RESERVED
void M_Code_M686(void){real_M_Code_M686();} // RESERVED
void M_Code_M687(void){real_M_Code_M687();} // RESERVED
void M_Code_M688(void){real_M_Code_M688();} // RESERVED

////////////////////////////////////////////////////////////////////////////////
#ifdef ALLOW_GCODE_SCRIPTS
void M_Code_M690(void)  // add/delete scripts
{
	// MCODE M690 <"D">             ; delete all scripts
	// MCODE M690 <"D"> '<label>    ; delete script "label"
	// MCODE M690 <"S"> '<label>    ; start adding script "label"
	// MCODE M690 <"E">             ; end adding script
	// MCODE
	// MCODE    delete and/or load scripts

	if (ARG_D_PRESENT && ARG_LABEL_PRESENT)
	{
		deleteScript(ARG_LABEL);
	}
	else if (ARG_D_PRESENT)
	{
		deleteAllScripts();
	}
	else if (ARG_S_PRESENT)
	{
		startaddingScript(ARG_LABEL);
	}
	else if (ARG_E_PRESENT)
	{
		endAddingScript();
	}
}
#else
void M_Code_M690(void) {;}
#endif

void M_Code_M698(void)  // humiture control (uses T, V)
{
	// MCODE M698 <"T" toolSelector> ["V" version]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if VArg is present, select the sensor version
	// MCODE    VArg -
	// MCODE		HUMITURE_DEFAULT		 = 0
	// MCODE        HUMITURE_DHT22_AM2302    = 1  (BitBang/10  (DEFAULT)
	// MCODE        HUMITURE_DHT11_AM2301    = 2  (BitBang/256)
	// MCODE
	// MCODE    humiture control
#ifdef GB_HIDDEN_WARNINGS
#warning "ADD AM2302 calibraion options -- match M699"
#endif //GB_HIDDEN_WARNINGS
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAFilamentDispenser(localOutboxPtr->device))
	{
		if (ARG_V_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_AM23XX_TYPE, iFitWithinRange((uint16_t)ARG_V, 0, 3));
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M699(void)  // hx711 control (uses T,Z,R,V,C,O,S,I,J,P,W)
{
	// MCODE M699 <"T" toolSelector> ["Z" setZero/tare] ["R" reportRaw]
	// MCODE M699 <"T" toolSelector> ["V" gainSelection] ["C" calibTemp] ["O" calibOfs]  ["S" calibScale] ["I" tempCompOfsPerDeg] ["J" tempCompScalePerDeg]  [<"P" password "W1 {save}>]
	// MCODE M699 <"T" toolSelector> ["U" pointIndex] ["W" weight grams] ; send 4 specific points and part will self calibrate
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if ZArg == 1, set tare to -1*current value; (auto saves config to flash)
	// MCODE    if RArg == 1, report raw sensor value with heartbeat (auto saves config to flash)
	// MCODE    else report result of gram calculation
	// MCODE
	// MCODE    VArg -
	// MCODE		HX711_GAIN_DEFAULT		 = 0
	// MCODE        HX711_GAIN_128_CH_A      = 1
	// MCODE        HX711_GAIN_64_CH_A       = 2
	// MCODE        HX711_GAIN_32_CH_B       = 3
	// MCODE	if CArg present, set the calibration temperature (temp at which calib factors were measured)
	// MCODE	if OArg present, set the calibration zero offset factor
	// MCODE	if SArg present, set the calibration scale factor
	// MCODE	if IArg present, set the temp compensation offset scale factor (per deg)
	// MCODE	if JArg present, set the temp compensation scale scale factor (per deg)
	// MCODE    else clear tare
	// MCODE    if WArg is present, and WArg==1, write calibration settings to flash {must set password P}
	// MCODE
	// MOCDE    Reported weight in grams:
	// MCODE        deltaT = currentTemp - calibTemp;
	// MCODE        zeroOfs = calibZeroOfs + (deltaT * tempCompOfsPerDeg);
	// MCODE        scale = calibScale + ((deltaT * tempCompScalePerDeg);
	// MOCODE       grams  =(rawReading - zeroOfs - tareValue) * scale;
	// MCODE
	// MCODE    hx711 control

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAFilamentDispenser(localOutboxPtr->device))
	{
		if (ARG_Z_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TARE, iFitWithinRange((uint16_t)ARG_Z, 0, 1));

		if (ARG_R_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_REPORT_MODE, iFitWithinRange((uint16_t)ARG_R, 0, 1));
		if (ARG_Z_PRESENT || ARG_R_PRESENT)
			canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_SAVE_CONFIGURATION, 0);

		if (ARG_V_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_GAIN, iFitWithinRange((uint16_t)ARG_V, 0, 3));
		if (ARG_C_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_TEMP, (temp_t)(ARG_C * TEMP_SCALEF));
		if (ARG_O_PRESENT) canSendOutboxInitValue1x32(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_ZERO_OFFSET, (uint32_t)ARG_O);
		if (ARG_S_PRESENT) canSendOutboxInitValue1x32(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_GRAMS_PER_TICK, *(uint32_t *)&ARG_S);	// sending float
		if (ARG_I_PRESENT) canSendOutboxInitValue1x32(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_ZERO_OFS_PER_DEG, *(uint32_t *)&ARG_I);
		if (ARG_J_PRESENT) canSendOutboxInitValue1x32(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_GRAMS_PER_TICK_PER_DEG, *(uint32_t *)&ARG_J);	// sending float

		if (ARG_U_PRESENT)
		{
			if (ARG_W_MISSING) ARG_W = 0.0f;
			canSendOutboxInitValue2x32(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_TEMP_COMP_SET_POINT, (uint32_t)ARG_U, *(uint32_t *)&ARG_W);
		}

		if ((ARG_P_PRESENT && (ARG_P == KARLS_PASSWORD)) && (ARG_W_PRESENT && (ARG_W == 1.0f)))
			canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_ADD, DEV_INIT_INDEX_ADD_HX711_SAVE_CALIBRATION, 0);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M701(void)  // set auto status update rate and page selection (uses T, S, P)
{
	// MCODE M701 ["T" toolSelector] ["P" periodInSeconds] ["V" valueMask]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg and PArg both missing, report error
	// MCODE    if PArg is present, update the reporting rate based on the period range is 0 to 60 seconds in 1ms increments
	// MCODE    if VArg is present, update the status value selection (mask)
	// MCODE
	// MCODE    set autostatus update rate and page selection for selected HHs

	//NUKE if (ARG_S_MISSING && ARG_P_MISSING) { ReportMissingMcodeSPArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	byte flags = 0;
	if (ARG_P_PRESENT)
	{
		localOutboxPtr->autoStatusPeriodMs = iFitWithinRange((int)(ARG_P * 1000.0f), 0, ONE_MINUTE_IN_MS); // range of 0ms to one minute
		flags |= PAGE_MASK_AUTO_STATUS_CONTROL_RATE_BIT;
	}

	if (ARG_V_PRESENT)
	{
		localOutboxPtr->autoStatusMask = (uint32_t)(ARG_V);
		flags |= PAGE_MASK_AUTO_STATUS_CONTROL_MASK_BIT;
	}
	sendAutoStatusControlToDevice(localOutboxPtr, flags);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M702(void)  // select Tx usage in MCODES (uses S)
{
	// MCODE M702 <"S" mcode_Targ_usage>
	// MCODE
	// MCODE    if SArg not present OR (SArg < 1) OR (SArg > 3) error
	// MCODE    else is S==1  (MCODE_TARG_IS_CANBUS_ADDRESS)
	// MCODE        Targ on MCODEs is the canbus address or alias
	// MCODE    else is S==2  (MCODE_TARG_IS_TOOL_NUMBER)
	// MCODE        Targ on MCODEs is the tool number
	// MCODE    else is S==3  (MCODE_TARG_IS_CANBUS_ADDRESS_OR_TOOL_NUMBER)
	// MCODE        Targ on MCODEs is the tool number if Targ <= 9
	// MCODE        Targ on MCODEs is the canbus address or alias if Targ >= 10
	// MCODE
	// MCODE    select Tx usage in MCODES
	// MCODE
	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (!IS_A_VALID_GROUP_IDENIFIER((int)ARG_S)) { ReportInvalidMcodeSArg(); return; } // legal window for group identifier
	if (!(isAPhysicalDevice((int)ARG_T) || isALogicalDevice((int)ARG_T))) { ReportInvalidMcodeSArg(); return; } // can only copy from a valid mailbox

	inboxStruct  *sourceInboxPtr    = getInboxPointer((byte)ARG_T);
	outboxStruct *sourceOutboxPtr   = getOutboxPointer((byte)ARG_T);
	inboxStruct  *destInboxPtr      = getInboxPointer((byte)ARG_S);
	outboxStruct *destOutboxPtr     = getOutboxPointer((byte)ARG_S);
	memcpy((byte *)destInboxPtr, (byte *)sourceInboxPtr, sizeof(inboxStruct));
	memcpy((byte *)destOutboxPtr, (byte *)sourceOutboxPtr, sizeof(outboxStruct));
	destInboxPtr->device = (byte)ARG_S;
	destOutboxPtr->device = (byte)ARG_S;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M703(void)  // add a group or alias to device (uses T, S, C)
{
	// MCODE M703 ["T" toolSelector] <"S" alias>
	// MCODE M703 ["T" group] <"S" physicalDevice> ["C" copyToAliasStructure(don't care)]
	// MCODE
	// MCODE    if argT is not present, the prior selected device/alias is used
	// MCODE    if argS is not present, ERROR
	// MCODE    if argC is present
	// MCOE         then device argS's current data will be used to seed the group defined by argT
	// MCODE        otherwise, the device will be given the additional group/alias specified by argS
	// MCODE
	// MCODE    add a group or alias to the selected HHs or seed a group from a head's values

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();



		//give group its identity
		if ((ARG_S < 1.0f) || (ARG_S > 250.0f))  { ReportInvalidMcodeSArg(); return; } // legal window for aliases

		sendAddAliasToDevice(localOutboxPtr, (byte)ARG_S);
		if (IS_A_VALID_GROUP_IDENIFIER((int)ARG_S))
		{
			sendAddAliasToDevice(localOutboxPtr, (byte)HH_BROADCAST_ALL_GROUPS);
		}
		_g4DwellTimer = 20; // 20ms pause to allow reasonable time for full alias to be read back
	}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M704(void)  // remove a user alias from device (uses T, S)
{
	// MCODE M704 ["T" toolSelector] <"S" alias>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if S is not present, ERROR
	// MCODE    otherwise, the alias specified by S will be removed from device
	// MCODE
	// MCODE    remove a user alias from the selected HHs

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if ((ARG_S < 1.0f) || (ARG_S > 250.0f))   // legal window for aliases
	{ ReportInvalidMcodeSArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	sendRemoveAliasToDevice(localOutboxPtr, (byte)ARG_S);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M705(void)  // reset device (uses T)
{
	// MCODE M705 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    reset specified devices

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	sendSetSwResetBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M706(void)  // sync device (uses T)
{
	// MCODE M706 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    sync specified devices - get their control state machines lined up

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	sendSetGlobalSyncBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M707(void)  // stop device (uses T)
{
	// MCODE M707 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    stop specified devices (extruder off, heaters off, etc)

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	sendSetStopExtruderBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M708(void)  // pause device (uses T)
{
	// MCODE M708 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    pause specified devices (prevents motor stepping. no change to rate or heater)

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	sendSetPauseExtruderBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M709(void)  // resume device (from pause or stop) (uses T)
{
	// MCODE M707 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    resume device (from pause or stop)

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	sendClrStopExtruderBit(localOutboxPtr->device);
	sendClrPauseExtruderBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M710(void)  // disable(0), enable(1) RTD1 (uses T, S)
{
	// MCODE M710 ["T" toolSelector] <"S" 0|1>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg != 0) and (SArg != 1.0), error
	// MCODE    else if SARg == 0.0, disable use of RTD1
	// MCODE    else [if SARg == 1.0], enable use of RTD1
	// MCODE
	// MCODE    disable/enable RTD1
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M711(void)  // disable(0), enable(1) RTD2 (uses T, S)
{
	// MCODE M711 ["T" toolSelector] <"S" 0|1>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg != 0) and (SArg != 1.0), error
	// MCODE    else if SARg == 0.0, disable use of RTD2
	// MCODE    else [if SARg == 1.0], enable use of RTD2
	// MCODE
	// MCODE    disable/enable RTD2
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M712(void)  // disable(0), enable(1) DEBUG_RTD (changes heartbeat reporting 2nd / 3rd fields (uses T, S)
{
	// MCODE M712 ["T" toolSelector] <"S" 0|1>
	// MCODE
	// MCODE enable/disable RTD debug (head sends raw ADC value in heartbeat message rtd1=u16[2] rtd2=u16[3]

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT && (ARG_S == 1.0f))
		sendSetDebugRtdBit(localOutboxPtr->device);
	else
		sendClrDebugRtdBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M713(void)  // tool change (uses T)
{
	// MCODE M713 <"T" toolSelector>
	// MCODE
	// MCODE    if T is missing - error
	// MCODE
	// MCODE    set the default device to use when TArg is not specified in following Mcodes
	// MCODE    (performs same function as GCODE 'T'#  command
	// MCODE RETIRED
	ReportRetiredMcode("use Tx command");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M714(void)  // update outgoing devicePosition remapping table  // SHOULD THIS BE REMOVED?  (NUKE? SAVE FOR SOMEDAY) LEGACT
{
	// MCODE M714 <"T" toolSelector> <"S" alias>
	// MCODE
	// MCODE    tool selection alias table : table[T] = S;
	// MCODE
	// MCODE    if T is missing or (T < 1) or (T > 250) or (T == 90) or (T == 100) error
	// MCODE            (do not let position 0 get aliased (broadcast all)
	// MCODE            (do not let position 90 get aliased (broadcast all hotbeds)
	// MCODE            (do not let position 100 get aliased (broadcast all extruders)
	// MCODE    if S is missing or (S < 0) or (S > 250) error
	// MCODE    else update table[T] = S;
	// MCODE
	// MCODE    update the outgoing devicePosition remapping table.  prior to a message
	// MCODE    being sent on the bus, the device position is used at the index into
	// MCODE    this table with the result being the device ID that will be put out
	// MCODE    on the canbus.  this allows simple mapping from one device to a group
	// MCODE    of devices without changing the GCODE.
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
//    if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
//    if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
//
//    if ((ARG_T < 1) || (ARG_T > 250) || (ARG_T == 90) || (ARG_T == 100)) { ReportInvalidMcodeTArgInt(); return; }
//    if ((ARG_S < 0.0f) || (ARG_S > 250.0f)) { ReportInvalidMcodeSArg(); return; }
//
//    changeOutgoingDevicePositionRemappingTable((byte)ARG_T, (byte)ARG_S);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M715(void)  // set LED display control and data selection (uses T, P, S, E)
{
	// MCODE M715 ["T" toolSelector] ["P" displayPage] ["S" dutyCycle] ["E" errorDisplayCount]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if P is present, set LED display selection
	// MCODE    if S is present, set LED PWM dutyCycle
	// MCODE    if E is present, set LED display error count
	// MCODE
	// MCODE    set the LED display parameters (what to display, PWM, and error display)
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M716(void)  // turns off logging of aux comm in repetrel
{
	// MCODE M716
	// MCODE
	// MCODE   turns off logging of aux comm in repetrel
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M717(void)  // turns on logging of aux comm in repetrel
{
	// MCODE M717
	// MCODE
	// MCODE    turns on logging of aux comm in repetrel
	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M718(void)  // used by repetrel to synchronize M719 data logging (stops logging)
{
	// MCODE M718
	// MCODE
	// MCODE    used by repetrel to synchronize M719 data logging (stop comm logging)

	SendCurrentMcodeExecutionNotice(FALSE);
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M719(void)  // set reporting update rate and page selection for host traffic (uses T, S, P, V, K, L, XYZABCVEL)
{
	// MCODE M719 ["P" periodInSeconds] ["K" keepAliveDuringDebug] [["S" enableAllInfo] | [XYZABCVEL enableExtraInfo]]
	// MCODE M719 ["T" toolSelector] ["P" periodInSeconds] ["V" valueMask]
	// MCODE M719 ["T" toolSelector] ["D" laserPS debug rate]

	// MCODE
	// MCODE    if TArg is not present then only affects motion controller to repetrel reporting
	// MCODE    iif PArg is present, update the reporting rate based on the period range is 0 to 60 seconds in 1ms increments
	// MCODE    if PArg is present, update the report data selection
	// MCODE    if KArg is present
	// MCODE         if KArg==1.0f - keep heads alive during ST-Link debug (ignore comm watchdogs)
	// MCODE         else          - normal function
	// MCODE    if VArg is present, update the status value selection (mask) on head
	// MCODE
	// MCODE    set reporting update rate and page selection for host traffic (enables comm logging in repetrel)

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_P_PRESENT)
	{
		_MailBoxes._hostTrafficReportingPeriodMs = iFitWithinRange((int)(ARG_P * 1000.0f), 0, ONE_MINUTE_IN_MS); // range of 0ms to one minute
		_MailBoxes._hostTrafficReportingPrescaleCnt = _MailBoxes._hostTrafficReportingPeriodMs;
	}

	if (ARG_S_PRESENT)
	{   // all or nothing
		boolean commonFlag      = (ARG_S == 1.0f) ? TRUE : FALSE;
		StatusReportXYZLocation = commonFlag;
		StatusReportVelocity    = commonFlag;
		StatusReportFlowRate    = commonFlag;
		StatusReportLineNum     = commonFlag;
		MotorStructure *M;
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{
			M->reportPositionWithStatus = StatusReportXYZLocation;
		}
	}
	else if (ARG_T_MISSING && (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT || ARG_V_PRESENT || ARG_E_PRESENT || ARG_L_PRESENT))
	{
		if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT )
		{
			StatusReportXYZLocation = FALSE;
			MotorStructure *M;
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // first, kill all flags
				M->reportPositionWithStatus = StatusReportXYZLocation;
			}
			if (ARG_X_PRESENT) { Motors[M_X].reportPositionWithStatus = (ARG_X == 1.0f) ? TRUE : FALSE;  StatusReportXYZLocation |= Motors[M_X].reportPositionWithStatus; }
			if (ARG_Y_PRESENT) { Motors[M_Y].reportPositionWithStatus = (ARG_Y == 1.0f) ? TRUE : FALSE;  StatusReportXYZLocation |= Motors[M_Y].reportPositionWithStatus; }
			if (ARG_Z_PRESENT) { Motors[M_Z].reportPositionWithStatus = (ARG_Z == 1.0f) ? TRUE : FALSE;  StatusReportXYZLocation |= Motors[M_Z].reportPositionWithStatus; }
			if (ARG_A_PRESENT) { Motors[M_A].reportPositionWithStatus = (ARG_A == 1.0f) ? TRUE : FALSE;  StatusReportXYZLocation |= Motors[M_A].reportPositionWithStatus; }
			if (ARG_B_PRESENT) { Motors[M_B].reportPositionWithStatus = (ARG_B == 1.0f) ? TRUE : FALSE;  StatusReportXYZLocation |= Motors[M_B].reportPositionWithStatus; }
			if (ARG_C_PRESENT) { Motors[M_C].reportPositionWithStatus = (ARG_C == 1.0f) ? TRUE : FALSE;  StatusReportXYZLocation |= Motors[M_C].reportPositionWithStatus; }
		}
		if (ARG_V_PRESENT) StatusReportVelocity = (ARG_V == 1.0f) ? TRUE : FALSE;
		if (ARG_E_PRESENT) StatusReportFlowRate = (ARG_E == 1.0f) ? TRUE : FALSE;
		if (ARG_L_PRESENT) StatusReportLineNum  = (ARG_L == 1.0f) ? TRUE : FALSE;
	}


	if (ARG_T_PRESENT)
	{   // update just one head/alias
		byte flags = 0;
		if (ARG_P_PRESENT)
		{
			localOutboxPtr->autoStatusPeriodMs = iFitWithinRange((int)(ARG_P * 1000.0f), 0, ONE_MINUTE_IN_MS); // range of 0ms to one minute

			flags |= PAGE_MASK_AUTO_STATUS_CONTROL_RATE_BIT;
		}
		if (ARG_V_PRESENT)
		{
			localOutboxPtr->autoStatusMask = (uint32_t)(ARG_V);
			flags |= PAGE_MASK_AUTO_STATUS_CONTROL_MASK_BIT;
		}

		sendAutoStatusControlToDevice(localOutboxPtr, flags);
	}

	if (ARG_K_PRESENT)
	{
		_KeepHeadsAliveDuringDebug = (ARG_K == 1.0f);
		if (_KeepHeadsAliveDuringDebug)
			sendSetIgnoreWatchdogBit(0);
		else
			sendClrIgnoreWatchdogBit(0);
	}


	if (ARG_D_PRESENT)
	{   //CO2 laser power supply debug
		_LaserDebugReportingRate = (int)ARG_D;
		_canStringRate = 0; // reset count
		if (_LaserDebugReportingRate > 0)
			sendSetSendDebugStringsBit(localOutboxPtr->device); // tell head it's okay to send out debug strings
		else
			sendClrSendDebugStringsBit(localOutboxPtr->device);
	}

	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M720(void)  // direct MAIN extrusion control (uses T, S, E, I)
{
	// MCODE M720 ["T" toolSelector] ["S" Pulses/sec] ["E" qty] ["I" immediate execute]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg present, new rate is used (otherwise prior rate is maintained)
	// MCODE    if EArg is present, new extrusion quatity is set (pos or neg) (otherwise no extrusion)
	// MCODE                    E can be set to to 100000000.0 or -1000000000.0 for continuous extrude
	// MCODE    if PArg is present, parameters are updated in hot head, but no extrusion takes place
	// MCODE
	// MCODE    direct MAIN (no stepping) extrusion control for the selected HHs
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M721(void)  // direct UNPRIME extrusion control (uses T, S, E, P, I)
{
	// MCODE M721 ["T" toolSelector] ["S" rate qty/sec] ["E" qty] ["P" preamble ms] ["I" immediate execute]
	// MCODE M721 ["T" toolSelector] ["S" rate qty/sec] ["E" qty] ["P" preamble ms] ["I" immediate execute]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg present, new rate is used (otherwise prior rate is maintained)
	// MCODE    if EArg is present, new extrusion quatity is set (pos or neg) (otherwise no extrusion)
	// MCODE                    E can be set to to 100000000.0 or -1000000000.0 for continuous extrude
	// MCODE    if PArg is present, parameters are updated in hot head, but no extrusion takes place
	// MCODE    To execute immedite, indlude a I argument, I is for Immediate, otherwise just the prams are updated
	// MCODE    direct UNPRIME (no stepping) extrusion control for the selected HHs

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT)  localOutboxPtr->ExtrusionControl.UnPrimeFeedRate = (uint16_t)ARG_S;    //set up the rate
	if (ARG_E_PRESENT)  localOutboxPtr->ExtrusionControl.UnPrimeSteps    = (uint16_t)ARG_E;    //set up the steps
	if (ARG_P_PRESENT)  localOutboxPtr->ExtrusionControl.UnPrimeTimeMs = (int16_t)fFitWithinRange(ARG_P, -5000.0f, 5000.0f); //set up delay
	Send_UnPrime_Prams(localOutboxPtr);//transmit the parameters
	if (ARG_I_PRESENT)  {
		UnPrime(localOutboxPtr);//if I present then you need to execute Unprime immediate
	}

	if (_sendingGBStringsMask & GB_STRING_PRIME)  // use M797 S<mask> to enable
	{
		sprintf(_tmpStr,"UnPRIMEFR=%u UnPRIMESTEPS=%u UnPrimeTimeMs=%d ExtrideRate=%u",
				localOutboxPtr->ExtrusionControl.UnPrimeFeedRate,localOutboxPtr->ExtrusionControl.UnPrimeSteps,
				localOutboxPtr->ExtrusionControl.UnPrimeTimeMs,localOutboxPtr->ExtrusionControl.ExtrudeFeedRate);
		sendGB(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M722(void)  // direct PRIME extrusion control (uses T, S, E, P, I, J)
{
	// MCODE M722 ["T" toolSelector] ["S" rate qty/sec] ["E" qty]       ["P" preambleMs] ["I" {immediate execute}] ["J" {extrude}]       (EXTRUDE)
	// MCODE M722 ["T" toolSelector]                    ["E" piercePwr] ["P" pierceMs] ["I" {immediate execute}] ["J" {extrude}]   (V1 LASER)  TBD
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg present, new rate is used (otherwise prior rate is maintained)
	// MCODE    if EArg is present, new extrusion quatity is set (pos or neg) (otherwise no extrusion)
	// MCODE                    E can be set to to 100000000.0 or -1000000000.0 for continuous extrude
	// MCODE            for piercePower, E must be between 0 and 100
	// MCODE    if PArg is present, parameters are updated in hot head, but no extrusion takes place
	// MCODE
	// MCODE    direct PRIME (no stepping) extrusion control for the selected HHs

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT)  localOutboxPtr->ExtrusionControl.PrimeFeedRate = (uint16_t)ARG_S;  //set up the rate
	if (ARG_E_PRESENT)  localOutboxPtr->ExtrusionControl.PrimeSteps = (uint16_t)ARG_E;  //set up the steps
	if (ARG_P_PRESENT)  localOutboxPtr->ExtrusionControl.PrimeTimeMs = (int16_t)fFitWithinRange(ARG_P, 0.0f, 5000.0f);
	Send_Prime_Prams(localOutboxPtr);//update the hothead

	if (ARG_I_PRESENT)
	{
		if (ARG_J_PRESENT)
		{
			PrimeThenRun(localOutboxPtr); // I & J present, then execute prime then run
		}
		else
		{
			Prime(localOutboxPtr);        // I present then execute prime
		}
	}

	if (_sendingGBStringsMask & GB_STRING_PRIME)  // use M797 S<mask> to enable
	{
		sprintf(_tmpStr,"PRIMEFR=%u PRIMESTEPS=%u PrimeTimeMs=%u ExtrideRate=%u",
				localOutboxPtr->ExtrusionControl.PrimeFeedRate,localOutboxPtr->ExtrusionControl.PrimeSteps,
				localOutboxPtr->ExtrusionControl.PrimeTimeMs,localOutboxPtr->ExtrusionControl.ExtrudeFeedRate);
		sendGB(_tmpStr);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M723(void)  // direct MANUAL extrusion control, Unmodified (uses T, S, E, P)
{
	// MCODE M723 ["T" toolSelector] ["S" rate qty/sec] ["E" qty] ["P" store only] ["C1" continuous]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg present, new rate is used (otherwise prior rate is maintained)
	// MCODE    if EArg is present, new extrusion quatity is set (pos or neg) (otherwise no extrusion)
	// MCODE                    E can be set to to 100000000.0 or -1000000000.0 for continuous extrude
	// MCODE    if PArg is present, parameters are updated in hot head, but no extrusion takes place
	// MCODE
	// sets the pulse rate in Raw pulses per second, does not use fudge, Z-Height, nozzle width for adjustment
	//simply sets up the extrusion rate, must be careful, as this will over ride the calculated extrusion pp/sec.
	//to run continous, send 65535 as the steps value,
	//to stop, send 0 as the steps value

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if ((localOutboxPtr->deviceFamily == DEVICE_FAMILY_HEATED_EXTRUDER) || (localOutboxPtr->deviceFamily == DEVICE_FAMILY_UNHEATED_EXTRUDER))
	{
		if (ARG_E_PRESENT && (ARG_E == 0.0f))
		{
			StopExtruding(localOutboxPtr);
		}
		else
		{
			if (ARG_S_PRESENT)
				localOutboxPtr->ExtrusionControl.ExtrudeFeedRate = (uint16_t)ARG_S;    //set up the rate
			if (ARG_C_PRESENT && (ARG_C == 1.0f))
				localOutboxPtr->ExtrusionControl.ExtrudeSteps    = 0xffff;	// force continuou
			else if (ARG_E_PRESENT)
				localOutboxPtr->ExtrusionControl.ExtrudeSteps    = (uint16_t)iFitWithinRange((int)ARG_E, 0, 0xffff);    //set up the amount
			RunExtrusionManual(localOutboxPtr);
		}
		if (localOutboxPtr->ExtrusionControl.isManuallyExtruding == FALSE)
		{   // little hack to try to keep Repetrel in sync
			ARG_S = 0.0f;
		}
		SendCurrentMcodeExecutionNotice(TRUE);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M724(void)  // direct DWELL (no stepping) extrusion control (uses T, S, E, P)
{
	// MCODE M724 ["T" toolSelector] ["S" rate qty/sec] ["E" qty] ["P" store only]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg present, new rate is used (otherwise prior rate is maintained)
	// MCODE    if EArg is present, new extrusion quatity is set (pos or neg) (otherwise no extrusion)
	// MCODE                    E can be set to to 100000000.0 or -1000000000.0 for continuous extrude
	// MCODE    if PArg is present, parameters are updated in hot head, but no extrusion takes place
	// MCODE
	// MCODE    direct DWELL (no stepping) extrusion control for the selected HHs
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void setupKarlFactors(byte switchNum)
{
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_P_MISSING) { ReportMissingMcodePArg(); return; }
	if (ARG_E_MISSING) { ReportMissingMcodeEArg(); return; }
	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	//if (ARG_D_MISSING) { ReportMissingMcodeDArg(); return; }

	devInitArea_t devInitArea = getDevInitAreaSwitchNum(switchNum);

	if (ARG_P_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, devInitArea, DEV_INIT_INDEX_SWx_KF_PF, (uint16_t)ARG_P);
	if (ARG_E_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, devInitArea, DEV_INIT_INDEX_SWx_KF_BIAS, (uint16_t)ARG_E);
	if (ARG_S_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, devInitArea, DEV_INIT_INDEX_SWx_KF_OVERSHOOT, (uint16_t)(ARG_S * TEMP_SCALE));
	if (ARG_D_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, devInitArea, DEV_INIT_INDEX_SWx_KF_DENOM, (uint16_t)ARG_D);
	if (ARG_F_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, devInitArea, DEV_INIT_INDEX_SWx_KF_FAN_DC, (uint16_t)(ARG_F * ((float)(1 << HH_SCALE_FACTOR_FRAC_BITS))));
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M725(void)  // set the Karl Factors for controlling the heater switch (sw1) (uses T, S, E, P, D)
{
	// MCODE M725 ["T" toolSelector] <"P" powerFactor  "E" deltaTempBiasCoeff  "S" temperatureOvershoot>                  (canFmt==0)
	// MCODE M725 ["T" toolSelector] <"P" powerFactor  "E" deltaTempBiasCoeff  "S" temperatureOvershoot "D" denominator> ["F" fanDcPct] (canFmt==1)
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if P, E, and S are not all present, then error.  (canFmt==0)
	// MCODE    if P, E, S and D are not all present, then error.  (canFmt==1)
	// MCODE    if F present, then send amount to goose the heater DC (pct of fanDC)
	// MCODE    otherwise set the switch control factors for the heater (sw1) switch
	// MCODE
	// MCODE    set the Karl Factors for the controller the heater switch

	setupKarlFactors(HH_HTR_SWITCH);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M726(void)  // set the Karl Factors for controlling the fan switch (sw0) (uses T, S, E, P, D)
{
	// MCODE M726 ["T" toolSelector] <"P" powerFactor  "E" deltaTempBiasCoeff  "S" temperatureOvershoot>                  (canFmt==0)
	// MCODE M726 ["T" toolSelector] <"P" powerFactor  "E" deltaTempBiasCoeff  "S" temperatureOvershoot "D" denominator>  ["F" fanDcPct] (canFmt==1)
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if P, E, and S are not all present, then error.  (canFmt==0)
	// MCODE    if P, E, S and D are not all present, then error.  (canFmt==1)
	// MCODE    otherwise set the switch control factors for the fan (sw0) switch
	// MCODE
	// MCODE    set the Karl Factors for the controller the fan switch (sw0)

	setupKarlFactors(HH_AUX_SWITCH);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M727(void)  // set LED override values and mask (uses T, S, P)
{
	// MCODE M727 ["T" toolSelector] <"S" override value>  <"P" override mask>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg or PArg is not present, error.
	// MCODE    set override value to S and override mask to P
	// MCODE
	// MCODE    set the LED override display value and make
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M728(void)  // set motor current boost control (uses T, S)
{
	// MCODE M728 ["T" toolSelector] <"S" boostValue>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error.
	// MCODE    set boost control value to S
	// MCODE        IN_FLOAT    = 0,
	// MCODE        IN_ANALOG   = 1,
	// MCODE        IN_PULLDOWN = 2,
	// MCODE        IN_PULLUP   = 3,
	// MCODE        OUT_0       = 4,
	// MCODE        OUT_1       = 5
	// MCODE
	// MCODE    set motor current boost control

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_BOOST, (uint16_t)ARG_S);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M729(void)  // set motor microsteps control (uses T, S)
{
	// MCODE M729 ["T" toolSelector] <"S" microstep code>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error.
	// MCODE    set microstep control value to S
	// MCODE
	// MCODE    set motor microstep control

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	// FIX IN REV5
	// new heads want to know how many microsteps, not a code based on allegro parts
	int microsteps;
	switch ((uint16_t)ARG_S)
	{
	case  0 : microsteps = 1; break;
	case  1 : microsteps = 2; break;
	case  2 : microsteps = 4; break;
	case  3 : microsteps = 8; break;   // 4988 only
	case  7 :
	default : microsteps = 16; break;
	}
	if (!deviceIsABttCanAxisMotor(localOutboxPtr->device))
	{	//BTT motors have an issue with this command
		canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP, microsteps);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M730(void)  // set not to exceed temp for motor (uses T, S)
{
	// MCODE M730 ["T" toolSelector] <"S" temp>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error.
	// MCODE    set extreme temp to S
	// MCODE
	// MCODE    set not to exceed temp for motor
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M731(void)  // set not to exceed temp for heater (uses T, S)
{
	// MCODE M731 ["T" toolSelector] <"S" temp>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error.
	// MCODE    set extreme temp to S
	// MCODE
	// MCODE    set not to exceed temp for heater
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M732(void)  // set maximum step rate for motor (microsteps/sec) (uses T, S)
{
	// MCODE M732 ["T" toolSelector] <"S" rate (usteps/sec)>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if FArg is not present, error.
	// MCODE    set max step rate value to F
	// MCODE
	// MCODE    set maximum step rate for motor (microsteps/sec)

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_MAX_RATE, (uint16_t)ARG_S);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M733(void)  // set maximum allowable RTD temperature delta (uses T, S)
{
	// MCODE M733 ["T" toolSelector] <"S" tempDelta>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error.
	// MCODE    set temp delta to S
	// MCODE
	// MCODE    set maximum allowable RTD temperature delta
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M734(void)  // set HH error reporting rate for redundant error codes (uses T, S)
{
	// MCODE M734 ["T" toolSelector] <"S" rate>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error.
	// MCODE    set rate value to F
	// MCODE
	// MCODE    set HH error reporting rate for redundant error codes
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M735(void)  // fill the incoming page data buffer with S (uses S)
{
	// MCODE M735 ["S" fill byte]
	// MCODE
	// MCODE    if SArg is not present, fill with 0's
	// MCODE    else fill with (byte)SArg (0 <= SArg <= 255
	// MCODE
	// MCODE    fill the incoming page data buffer with S
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M736(void)  // fill the outgoing page data buffer with S (uses S)
{
	// MCODE M736 ["S" fill byte]
	// MCODE
	// MCODE    if SArg is not present, fill with 0's
	// MCODE    else fill with (byte)SArg (0 <= SArg <= 255
	// MCODE
	// MCODE    fill the outgoing page data buffer with S
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M737(void)  // erase flash page in selected (physical) device (uses T, P, I)
{
	// MCODE M737 ["T" toolSelector] <<"P" physicalPage> | <"I" indirectPage>>
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE    if PArg or IArg is not present, error. (one has to be specified)
	// MCODE    PArg - physical flash page number (0-max)
	// MCODE    IArg - indirect page (0=tables; 1=soapstring; 2=hist0; 3=hist1)
	// MCODE
	// MCODE    erase flash page in selected (physical) device
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M738(void)  // transfer data page from (physical) device to incoming buffer (uses T, P, I)
{
	// MCODE M738 ["T" toolSelector] <<"P" physicalPage> | <"I" indirectPage>>
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE    if PArg or IArg is not present, error. (one has to be specified)
	// MCODE    PArg - physical flash page number (0-max)
	// MCODE    IArg - indirect page (0=tables; 1=soapstring; 2=hist0; 3=hist1)
	// MCODE
	// MCODE    transfer data page from (physical) device to incoming buffer
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M739(void)  // transfer data page from incoming to outgoing buffer
{
	// MCODE M739
	// MCODE
	// MCODE    transfer dataPage from inbox to outbox
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M740(void)  // transfer data page from outgoing buffer to (physical) device (uses T, P, I, S)
{
	// MCODE M740 ["T" toolSelector] <<"P" physicalPage> | <"I" indirectPage>> ["S" size in bytes]
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE    if PArg or IArg is not present, error. (one has to be specified)
	// MCODE    PArg - physical flash page number (0-max)
	// MCODE    IArg - indirect page (0=tables; 1=soapstring; 2=hist0; 3=hist1)
	// MCODE    SArg - optional number of bytes (8 to max) ... really should be in 8 byte
	// MCODE           increments (canbus protocol limit)
	// MCODE
	// MCODE    transfer data page from outgoing buffer to (physical) device
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M741(void)  // transfer selected table data from the device to the host (uses T, S, I)
{
	// MCODE M741 ["T" toolSelector] <"S" dataSelection> ["I" page]
	// MCODE
	// MCODE    if SArg is not present, error. >RT
	// MCODE    SArg -
	// MCODE        SEND_DEVICE_POSITION_TABLE      = 0 (DEPRECATED)
	// MCODE        SEND_DEVICE_RTD1_TABLE          = 1 (DEPRECATED)
	// MCODE        SEND_DEVICE_RTD2_TABLE          = 2 (DEPRECATED)
	// MCODE        SEND_DEVICE_RTD3_TABLE          = 3 (DEPRECATED)
	// MCODE        SEND_DEVICE_TEMP_SENSOR_TABLE   = 4 (DEPRECATED)
	// MCODE        SEND_DEVICE_TEMP_2_LED_TABLE    = 5 (DEPRECATED)
	// MCODE        SEND_DEVICE_PULSE_TRAIN_TABLE   = 6 (DEPRECATED)
	// MCODE        SEND_DEVICE_SOAP_STRING         = 7
	// MCODE        SEND_DEVICE_HISTORY0_DATA       = 8 (DEPRECATED)
	// MCODE        SEND_DEVICE_HISTORY1_DATA       = 9 (DEPRECATED)
	// MCODE        SEND_DEVICE_RAW_PAGE_DATA       = 10 <requires page argument>
	// MCODE
	// MCODE    transfer selected data from the device to the host

	uint16_t page;

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if ((ARG_S < (byte)SEND_DEVICE_POSITION_TABLE) || (ARG_S > (byte)SEND_DEVICE_RAW_PAGE_DATA)) { ReportInvalidMcodeSArg(); return; }
	if ((ARG_S == (byte)SEND_DEVICE_RAW_PAGE_DATA) && (ARG_I_MISSING))  { ReportMissingMcodeIArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAPhysicalDevice(localOutboxPtr->device) == TRUE)
	{   // only allow operation on a real, physical device (no aliases)
		page = tableInfoTypeToPage(localOutboxPtr->device, (tableInfoType)(byte)ARG_S, (uint16_t)ARG_I);
		if (page == 0xffff)
		{
			ReportMcodeError("selection/page combination result in illegal page");
		}
		else
		{
			if (ARG_S == (byte)SEND_DEVICE_SOAP_STRING)
			{
				if (getInboxPointer(localOutboxPtr->device)->registrationStep == 0)
				{   // soapstring watchdog will trigger the read
					getInboxPointer(localOutboxPtr->device)->registrationStep = 3;
				}
			}
			else
			{
				transferDevicePageDataFromDeviceToWorkingBuffer(localOutboxPtr->device, page);//request the page data from the hothead via canbus
				sendLoopbackMessageToDevice(localOutboxPtr->device, PAGE_DATA_TO_HOST, (uint16_t)(tableInfoType)(byte)ARG_S, page, 0, 0); // mark so we know end of transfer
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M742(void)  // transfer select table data in ASCII text from host to device (uses T, S, P, I, comment)
{
	// MCODE M742 ["T" toolSelector] <"S" dataSelection> <password> ["I" page] <;page data>
	// MCODE
	// MCODE    if SArg is not present, error.
	// MCODE    SArg -
	// MCODE        SEND_DEVICE_POSITION_TABLE      = 0 (DEPRECATED)
	// MCODE        SEND_DEVICE_RTD1_TABLE          = 1 (DEPRECATED)
	// MCODE        SEND_DEVICE_RTD2_TABLE          = 2 (DEPRECATED)
	// MCODE        SEND_DEVICE_RTD3_TABLE          = 3 (DEPRECATED)
	// MCODE        SEND_DEVICE_TEMP_SENSOR_TABLE   = 4 (DEPRECATED)
	// MCODE        SEND_DEVICE_TEMP_2_LED_TABLE    = 5 (DEPRECATED)
	// MCODE        SEND_DEVICE_PULSE_TRAIN_TABLE   = 6 (DEPRECATED)
	// MCODE        SEND_DEVICE_SOAP_STRING         = 7
	// MCODE        SEND_DEVICE_HISTORY0_DATA       = 8 (*currently blocked by device)
	// MCODE        SEND_DEVICE_HISTORY1_DATA       = 9 (*currently blocked by device)
	// MCODE        SEND_DEVICE_RAW_PAGE_DATA       = 10 <requires page argument>
	// MCODE
	// MCODE    // transfer select table data in ASCII text from host to device

	uint16_t page;

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if ((ARG_S < (byte)SEND_DEVICE_POSITION_TABLE) || (ARG_S > (byte)SEND_DEVICE_RAW_PAGE_DATA)) { ReportInvalidMcodeSArg(); return; }
	if ((ARG_S == (byte)SEND_DEVICE_RAW_PAGE_DATA) && (ARG_I_MISSING)) { ReportMissingMcodeIArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAPhysicalDevice(localOutboxPtr->device) == TRUE)
	{   // only allow operation on a real, physical device (no aliases)
		page = tableInfoTypeToPage(localOutboxPtr->device, (tableInfoType)(byte)ARG_S, (uint16_t)ARG_I);
		if (page == 0xffff)
		{
			ReportMcodeError("selection/page combination result in illegal page");
		}
		else
		{
			transferDevicePageDataFromDeviceToWorkingBuffer(localOutboxPtr->device, page);//request the page data from the hothead via canbus
			sendLoopbackMessageToDevice(localOutboxPtr->device, PAGE_DATA_TO_DEVICE, (uint16_t)(tableInfoType)(byte)ARG_S, page, 0, 0);
		}
	}
	else
	{
		ReportMcodeError("ARG_T not a physical device");
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M743(void)  // transfer non table/page related  device info from inbox/device to host (uses T, S, I)
{
	// MCODE M743 ["T" toolSelector] <"S" dataSelection> ["I" page]
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE    if SArg is not present, error.
	// MCODE    SArg -
	// MCODE        SEND_DEVICE_REVISION_INFO           = 0
	// MCODE        SEND_DEVICE_FLASH_CONFIG            = 1
	// MCODE        SEND_DEVICE_UNIQUE_ID(12)           = 2
	// MCODE        SEND_DEVICE_ALIAS_LIST(12)          = 3
	// MCODE        SEND_DEVICE_HISTORY_RANGE_DEF(16)   = 4 (deprecated)
	// MCODE        SEND_DEVICE_HISTORY_RANGES(16)      = 5 (deprecated)
	// MCODE        SEND_DEVICE_HISTORY_COUNTERS(16)    = 6 (deprecated)
	// MCODE        SEND_DEVICE_PAGE_DEF(4)             = 7
	// MCODE        SEND_DEVICE_TABLE_START_OFFSETS(7)  = 8 (deprecated)
	// MCODE        SEND_DEVICE_OPTION_BYTES(16)        = 9  (deprecated)
	// MCODE        SEND_DEVICE_CONTROL_WORD            = 10
	// MCODE        SEND_DEVICE_PASSWORD                = 11
	// MCODE        SEND_DEVICE_PAGE_CHECKSUM           = 12
	// MCODE
	// MCODE    // transfer non table/page related device info from inbox/device to host

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if ((ARG_S < (byte)SEND_DEVICE_REVISION_INFO) || (ARG_S > (byte)SEND_DEVICE_PAGE_CHECKSUM)) { ReportInvalidMcodeSArg(); return; }
	if ((ARG_S == (byte)SEND_DEVICE_PAGE_CHECKSUM) && (ARG_I_MISSING))  { ReportMissingMcodeIArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAPhysicalDevice(localOutboxPtr->device) == TRUE)
	{   // only allow operation on a real, physical device (no aliases)
		switch ((deviceInfoType)(byte)ARG_S)
		{
		case SEND_DEVICE_FLASH_CONFIG :
		case SEND_DEVICE_PAGE_DEF :
			sendInboxInfoToHost(localOutboxPtr->device, (deviceInfoType)(byte)ARG_S);
			break;
		case SEND_DEVICE_UNIQUE_ID :    // for both UID and password, need to read the UID from the device before display
		case SEND_DEVICE_PASSWORD :
			readUniqueIdFromDevice(localOutboxPtr->device);
			break;
		case SEND_DEVICE_REVISION_INFO :                // could change if option bytes change
			readDeviceInfoFromDevice(localOutboxPtr->device);
			break;
		case SEND_DEVICE_ALIAS_LIST :
			readAliasListFromDevice(localOutboxPtr->device);
			break;
		case SEND_DEVICE_CONTROL_WORD :         // can change anytime
			readControlWordFromDevice(localOutboxPtr->device);
			break;
		case SEND_DEVICE_PAGE_CHECKSUM  :           // changes based on page
			readPageChecksumFromDevice(localOutboxPtr->device, (uint16_t)ARG_I);
			break;
		default :
			break;
		}
		sendLoopbackMessageToDevice(localOutboxPtr->device, DEVICE_INFO_TO_HOST, (uint16_t)(deviceInfoType)(byte)ARG_S, 0, 0, 0);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M744(void)  // transfer alias list from device to inbox (uses T)
{
	// MCODE M744 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device is used
	// MCODE
	// MCODE    transfer alias list from device to inbox

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	readAliasListFromDevice(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M745(void)  // change polarity definition of direction pin (uses T, S)
{
	// MCODE M745 ["T" toolSelector] <"S" 0|1>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg != 0) and (SArg != 1.0), error
	// MCODE    else if SARg == 0.0, restore polarity of direction pin to default setting
	// MCODE    else [if SARg == 1.0], the invert polarity of the direction pin
	// MCODE
	// MCODE    optional change the polarity of the direction pin for ALL uses

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if ((ARG_S != 0.0f) && (ARG_S != 1.0f)) { ReportInvalidMcodeSArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S == 0.0f)
		sendClrInvertDirectionBit(localOutboxPtr->device);
	else
		sendSetInvertDirectionBit(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M746(void)  // start the bootloader for the selected physical device (uses T)
{
	// MCODE M746 <"T" toolSelector>
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE
	// MCODE    start the bootloader for the selected physical device

	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAPhysicalDevice(localOutboxPtr->device) == TRUE)
	{   // only allow operation on a real, physical device (no aliases)
		_gs._bl.started = TRUE;
		_gs._bl.prepped = FALSE;
		_gs._bl.endOfFileReached = FALSE;
		_gs._bl.MCODE748_error_reported = FALSE;
		_gs._bl.device = localOutboxPtr->device;
		_MailBoxes._waitingFor.flags.bit.bootloaderAnnounce = TRUE; // stop processing input commands until bootloader has officially started
		sendSetSwResetBit(_gs._bl.device);  // reset the selected device which will initially wake up in bootloader mode and send CAN_MSG_BOOTLOADER_ANNOUNCE
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M747(void) // erase page for bootloader
{
	// MCODE M747 <"T" toolSelector> <"P" key>  (key=666)
	// MCODE
	// MCODE    if TArg is not present, error (TArg must be a physical device)
	// MCODE
	// MCODE    this is really the erase page routine and
	// MCODE    it MUST have a P of 666 and a specific address for a individual head

	if (ARG_T_MISSING)   { ReportMissingMcodeTArg();  return; }  // must have a descrete head address for this command
	if (ARG_P_MISSING)   { ReportMissingMcodePArg();  return; }  // must have a valid password for this command
	if ((uint32_t)ARG_P != KARLS_PASSWORD)  {ReportInvalidMcodePArgInt(); return; }   //must have a valid password for this command

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (deviceIsAPhysicalDevice(localOutboxPtr->device) == TRUE)
	{   // only allow operation on a real, physical device (no aliases)
		prepareDeviceForBootloaderDownload();
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M748(void)  // process next line of intel hex format bootloader data (uses P, comment)
{
	// MCODE M748 <"P" lineNum> ;:<;intel hex format line>
	// MCODE
	// MCODE    process next line of intel hex format bootloader data

	if (strlen(GCodeArgComment) >= BOOLTOADER_LINE_SIZE)
	{ sendError("bootloader hex line too long for buffer"); endDeviceBootloader(); return; }

	strcpy(_gs._bl.inputLine, &GCodeArgComment[1]);//add incoming string to last incoming string
	_gs._bl.lineNum++;

	if ((uint32_t)ARG_P != _gs._bl.lineNum)
	{
		sendHexLineError("Device bootloader - line number mismatch");
		endDeviceBootloader();
	}
	else if (processDeviceBootloaderLine(FALSE, 0) == FAIL)
	{
		endDeviceBootloader();
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M749(void)  // exit the device bootloader
{
	// MCODE M749
	// MCODE
	// MCODE    end the bootloader

	if (_gs._bl.started == FALSE) { ReportMcodeError("Device bootloader is not running"); return; }

	if (_gs._bl.lineNum > 0)
	{
		if ( _gs._bl.endOfFileReached == FALSE)
		{
			ReportMcodeError("exiting bootloader prematurely (intel.hex EOF record not found)");
			endDeviceBootloader();
		}
		else
		{
			setDeviceCodeChecksum(DEVICE_CODE_CHECKSUM_PART1);
		}
	}
	else
	{
		endDeviceBootloader();
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M750(void)  // unlock flash for erase/write access for the selected physical device (uses T)
{
	// MCODE M750 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE
	// MCODE    unlock flash for erase/write access for the selected physical device
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}


////////////////////////////////////////////////////////////////////////////////

void M_Code_M751(void)  // lock flash to prevent erase/write access for the selected physical device (uses T)
{
	// MCODE M751 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE
	// MCODE    lock flash to prevent erase/write access for the selected physical device
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M752(void)  // write hardware type to flash (option bytes) (uses T, S, P)
{
	// MCODE M752  <"T" toolSelector> <"S" hwType> <"P" password>
	// MCODE
	// MCODE    if SArg, is not present, error.
	// MCODE    if PArg, is not present, error.
	// MCODE    SArg - hardware device type (0-255)
	// MCODE
	// MCODE    write hardware type to flash (option bytes) using device bootloader (uses S)
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M753(void)  // write hardware revision to flash (option bytes) using device bootloader (uses  S, P)
{
	// MCODE M753 <"S" hwRev> <"P" password>
	// MCODE
	// MCODE    if SArg, is not present, error.
	// MCODE    if PArg, is not present, error.
	// MCODE    SArg - hardware revision (0-31)
	// MCODE
	// MCODE    write hardware revision to flash (option bytes) using device bootloader
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M754(void)  // write hardware key to flash (option bytes) using device bootloader (uses S, P)
{
	// MCODE M754 <"S" key> <"P" password>
	// MCODE
	// MCODE    if SArg, is not present, error.
	// MCODE    if PArg, is not present, error.
	// MCODE    SArg - hardware device key (0-255)
	// MCODE
	// MCODE    write hardware key to flash (option bytes) using device bootloader
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M755(void)  // set extruder heater pwm (uses T, S)
{
	// MCODE M755 ["T" toolSelector] <"S" duty cycle>
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg < 0) or (SArg > 1.0), error
	// MCODE    if (0 <= SArg <= 1.0) the HH will use SArg as a dutyCycle for the heater (ie, 0.25 is 25%)
	// MCODE
	// MCODE    set selected extruders heater switch control's dutyCycle
	// MCODE RETIRED
	ReportRetiredMcode("Use M242 Tx I1 Sx");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M756(void)  // set layer height (mm) (uses T, S)
{
	// MCODE M756 ["T" toolSelector] <"S" layer height>
	// MCODE    (REDUNADANT with M221 "Z")
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg <= 0), error
	// MCODE
	// MCODE    set layer height (mm)

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (ARG_S <= 0.0f) { ReportInvalidMcodeSArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	localOutboxPtr->ExtrusionControl.SliceHeightInMm = ARG_S;
	SendCurrentMcodeExecutionNotice(TRUE);
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M757(void)  // set layer/path width (mm) (uses S)
{
	// MCODE M757 ["T" toolSelector] <"S" layer width>
	// MCODE    (REDUNADANT with M221 "W")
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg <= 0), error
	// MCODE
	// MCODE    set layer/path width (mm)

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (ARG_S <= 0.0f) { ReportInvalidMcodeSArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	localOutboxPtr->ExtrusionControl.ExtrusionWidthInMm = ARG_S;
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M758(void)  // set extrusion step to volume conversion (steps per micro liter) (uses T, S)
{
	// MCODE M758 ["T" toolSelector] <"S" scale steps/uL>
	// MCODE    (REDUNADANT with M221 "P")
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if SArg is not present, error
	// MCODE    if (SArg<= 0), error
	// MCODE    otherwise, set the extrusion step to volume conversion to get the
	// MCODE        correct number of steps per micro liter of material
	// MCODE
	// MCODE    set extrusion step to volume conversion

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if (ARG_S <= 0.0f) { ReportInvalidMcodeSArg(); return; }

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	localOutboxPtr->ExtrusionControl.PulsesPerUnit = ARG_S;
	SendCurrentMcodeExecutionNotice(TRUE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M759(void)  // enable temperature calibration (uses T, S) - overloaded for XYZLocation reporting
{
	// MCODE M759 ["T" toolSelector] ["S" rate]
	// MCODE
	// MCODE    if TArg is present, device rawADC, converted temp, and calibration temp reported
	// MCODE        otherwise just calibration temp reported (along with specifics of the calibration device)
	// MCODE    if SArg is present, the reporting rate is update.
	// MCODE
	// MCODE    enable/disable temperature calibration
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M760(void)  // disable temperature calibration
{
	// MCODE M760
	// MCODE
	// MCODE    disable temperature calibration
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M761(void)  // transfer system info in ASCII text from main board to host (uses S, P, R)
{
	// MCODE M761 <"S" dataSelection> ["P" 1] [Rx]
	// MCODE
	// MCODE    if SArg is not present, error.
	// MCODE    SArg -
	// MCODE        SEND_SYSTEM_SOAP_STRING     = 1   (no ARG_R)   - send non-persistant area of the soap
	// MCODE                                          (ARG_R==1) - send persistant area of the soap
	// MCODE            PArg:
	// MCODE                1 - read ALL strings (padded with and "EOS" string at the end)
	// MCODE        SEND_SYSTEM_OPTION_BYTES    = 2
	// MCODE        SEND_SYSTEM_REVISION_INFO   = 3
	// MCODE        SEND_SYSTEM_FLASH_CONFIG    = 4
	// MCODE        SEND_SYSTEM_UNIQUE_ID       = 5
	// MCODE        SEND_SYSTEM_PASSWORD        = 6
	// MCODE        SEND_SYSTEM_SOAP_CONFIG     = 7
	// MCOODE       SEND_SYSTEM_CRASH_CONFIG    = 8
	// MCODE
	// MCODE    transfer system info in ASCII text from main board to host

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
	if ((ARG_S < (byte)SEND_SYSTEM_SOAP_STRING) || (ARG_S > (byte)SEND_SYSTEM_SOAP_CONFIG)) { ReportInvalidMcodeSArg(); return; }

	if (ARG_S == (byte)SEND_SYSTEM_SOAP_STRING)
	{
		if ((ARG_P_PRESENT) && (ARG_P==1.0f) && (ARG_R != 1.0f))
		{   // dump volatile part of soapstring
			_sysInfoPtr->soapReadPtr = (byte *)_sysInfoPtr->soapBaseAddr;//reset the transfer pointer please
			dump407SoapstringFlag=1;
		}
		else if ((ARG_P_PRESENT) && (ARG_P==1.0f) && (ARG_R==1.0f))
		{   // dump reserved part of soapstring
			_sysInfoPtr->soapReadPtr = (byte *)(_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET);//reset the transfer pointer please
			dump407SoapstringFlag=1;
		}
	}
	else
	{
		transferSystemInfoToHost((systemInfoType)ARG_S);
	}
}

////////////////////////////////////////////////////////////////////////////////
//
//boolean validFirmwareKey(boolean a)
//{
//	return TRUE;
//}
void invalidateFirmwareKey(void)
{
}

void M_Code_M762(void)  // transfer system info in ASCII text from host to main board (uses S, P, R, comment)
{
	// MCODE M762 <"S" dataSelection> [<"P" key>]  [Rx] (key=666)
	// MCODE
	// MCODE    if SArg is not present, error.
	// MCODE    if PArg, is not present, error.
	// MCODE    SArg -
	// MCODE        SEND_SYSTEM_SOAP_ERASE      = 0 (no ARG_R)   - erase non-persistant area of the soap
	// MCODE                                        (R=1) - erase non-persistant and persistant areas of the soap
	// MCODE        SEND_SYSTEM_SOAP_STRING     = 1 (no ARG_R)   - append string to non-persistant area of the soap
	// MCODE                                        (ARG_R=1) - append string to persistant area of the soap
	// MCODE        SEND_SYSTEM_OPTION_BYTES    = 2 (not implemented yet)
	// MCODE
	// MCODE    transfer system info in ASCII text from host to main board

	if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }//must have a valid S and P argument
	if (ARG_P_MISSING) { ReportMissingMcodePArg(); return; }//P is the password
	if ((uint32_t)ARG_P != KARLS_PASSWORD) { ReportMcodeError("password mismatch"); return; }//which must be 666 to prevent accidental erase.

	checkAndClearFlashStatus();	//need to do this here to try to ensure the soap update occurs

	switch ((byte)ARG_S)
	{
	case SEND_SYSTEM_SOAP_ERASE :
		if (ARG_R != 1.0f)
		{   // preseve a copy of the reserved area
			memcpy(_MailBoxes._workingBuffer, (byte *)(_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET), FLASH_SOAP_RESERVE_SIZE);
		}
		eraseSystemSoapString();
		if (ARG_R != 1.0f)
		{   //put the reserved data back
			byte *soapWritePtr = FLASH_SOAP_RESERVE_ADDR;
			int i;
			FLASH_Unlock();
			for (i=0; i<FLASH_SOAP_RESERVE_SIZE; i++)
			{
				FLASH_ProgramByte((uint32_t)soapWritePtr++, _MailBoxes._workingBuffer[i]);
			}
			FLASH_Lock();
		}
		else
		{
			invalidateFirmwareKey();
			sendError("Firmware key has been erased, please re-Flash a valid firmware key");
		}
		SendCurrentMcodeExecutionNotice(FALSE); // let host know we're done
		break;//erase the soap flash sector
	case SEND_SYSTEM_SOAP_STRING :
		if (ARG_R != 1.0f)
		{
			writeSystemSoapString(0);
		}
		else if ((ARG_R == 1.0f) &&  validFirmwareKey(0) && (strncmp((char *)(_sysInfoPtr->soapBaseAddr + FLASH_SOAP_RESERVE_OFFSET), "*M687", 5) == 0))
		{ // already verified key AND it's in flash and flash not erased), then no need to write
			sendError("Flash write request of firmware key ignored -- valid key already exists");
			return;
		}
		else
		{   // need to write info to restricted area
			writeSystemSoapString(1);
		}
		break;//append this comment line to the soap string storage in Flash
	case SEND_SYSTEM_OPTION_BYTES :
		ReportMcodeError("Option Bytes Not Implemented");
		break;//not supported at this time, but not an error
	default : //unsupported argument.....
		ReportInvalidMcodeSArg();
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M763(void)  // clear error on selected device(s) (uses T)
{
	// MCODE M763 ["T" toolSelector]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE
	// MCODE    clear error on selected device(s)

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();
	sendSetClearErrorBit(localOutboxPtr->device);

	if (ARG_T == 0.0f)
	{   // sending to ALL, so also clear system ERROR flags
		resetStickyErrorFlags();
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M764(void)  // set max duty cycle for device
{
	// MCODE M764 - just a shell. does nothing
	// MCODE
	// MCODE RETIRED
	 ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M765(void)  // read X, Y, Z and dial indicator
{
	// MCODE M763 - just a shell. does nothing
	// MCODE
	// MCODE RETIRED
	 ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

extern USB_OTG_CORE_HANDLE      USB_OTG_dev;

void M_Code_M766(void)  // start the system bootloader process
{
	// MCODE M766  jump to system bootloader
	// MCODE
	// MCODE    start the system bootloader process
	//      USART_DeInit(USART1);//reset all uarts, they will be used by boot loader com over USB port
	//      USART_DeInit(USART3);
	//      USART_DeInit(UART4);
	//      USART_DeInit(USART6);
	//      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//reset timers so we do not
	//      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3,DISABLE);//  get interrupts from them
	//      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	//      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4,DISABLE);
	//      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	//      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6,DISABLE);
	//      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
	//      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM7,DISABLE);
	//
	//      RCC_DeInit();
	//      SysTick->CTRL = 0;
	//      SysTick->LOAD = 0;
	//      SysTick->VAL = 0;
	//      __set_PRIMASK(1);
	//      *((unsigned long *)0x2000FFF0) = 0xDEADBEEF; // 64KBSTM32F103//set up the semiphore telling them to jump to DFU mode
	//

	USBD_DeInit(&USB_OTG_dev);

	DisableAllMotionMotors();

	SysTick->CTRL = 0;  // disable systick timer
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	NVIC->ICER[0]   = 0xffffffff;      //clear interrupt enables
	NVIC->ICER[1]   = 0xffffffff;      //clear interrupt enables
	NVIC->ICER[2]   = 0xffffffff;      //clear interrupt enables

	RCC->AHB1RSTR   = 0x226011ff;      //RCC AHB1 peripheral reset register - FORCE RESET (includes reset GPIO)
	RCC->AHB2RSTR   = 0x000000f1;      //RCC AHB2 peripheral reset register - FORCE RESET
	RCC->AHB3RSTR   = 0x00000001;      //RCC AHB3 peripheral reset register - FORCE RESET

	if ((DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID) == MCU_DEVICE_407)
	{   // sysInfoPtr->mcuDeviceID
		RCC->APB1RSTR   = 0x36fec9ff;      //RCC APB1 peripheral reset register - FORCE RESET
		RCC->APB2RSTR   = 0x00075933;      //RCC APB2 peripheral reset register - FORCE RESET
	}
	else if ((DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID) == MCU_DEVICE_429)
	{   // sysInfoPtr->mcuDeviceID
		RCC->APB1RSTR   = 0xf6fec9ff;      //RCC APB1 peripheral reset register - FORCE RESET
		RCC->APB2RSTR   = 0x00377933;      //RCC APB2 peripheral reset register - FORCE RESET
	}
	RCC->CR         = 0x00007f83;      //RCC clock control register
	RCC->PLLCFGR    = 0x24003010;      //RCC PLL configuration register
	RCC->CFGR       = 0x00000000;      //RCC clock configuration register
	RCC->CIR        = 0x00000000;      //RCC clock interrupt register
	RCC->AHB1RSTR   = 0x00000000;      //RCC AHB1 peripheral reset register - CLEAR RESET
	RCC->AHB2RSTR   = 0x00000000;      //RCC AHB2 peripheral reset register - CLEAR RESET
	RCC->AHB3RSTR   = 0x00000000;      //RCC AHB3 peripheral reset register - CLEAR RESET
	RCC->APB1RSTR   = 0x00000000;      //RCC APB1 peripheral reset register - CLEAR RESET
	RCC->APB2RSTR   = 0x00000000;      //RCC APB2 peripheral reset register - CLEAR RESET
	RCC->AHB1ENR    = 0x00100000;      //RCC AHB1 peripheral clock register
	RCC->AHB2ENR    = 0x00000000;      //RCC AHB2 peripheral clock register
	RCC->AHB3ENR    = 0x00000000;      //RCC AHB3 peripheral clock register
	RCC->APB1ENR    = 0x00000000;      //RCC APB1 peripheral clock enable register
	RCC->APB2ENR    = 0x00000000;      //RCC APB2 peripheral clock enable register
	RCC->AHB1LPENR  = 0x7eef97ff;      //RCC AHB1 peripheral clock enable in low power mode register
	RCC->AHB2LPENR  = 0x000000f1;      //RCC AHB2 peripheral clock enable in low power mode register
	RCC->AHB3LPENR  = 0x00000001;      //RCC AHB3 peripheral clock enable in low power mode register
	RCC->APB1LPENR  = 0xf6fec9ff;      //RCC APB1 peripheral clock enable in low power mode register
	RCC->APB2LPENR  = 0x04777f33;      //RCC APB2 peripheral clock enable in low power mode register
	RCC->BDCR       = 0x00000000;      //RCC Backup domain control register
	RCC->CSR        = 0x14000000;      //RCC clock control & status register
	RCC->SSCGR      = 0x00000000;      //RCC spread spectrum clock generation register
	RCC->PLLI2SCFGR = 0x24003000;      //RCC PLLI2S configuration register

	__asm("LDR     R0, =0x40023844\n" // ; RCC_APB2ENR
			"LDR     R1, =0x00004000\n" // ; ENABLE SYSCFG CLOCK
			"STR     R1, [R0, #0]\n"
			"LDR     R0, =0x40013800\n" //; SYSCFG_MEMRMP
			"LDR     R1, =0x00000001\n" //; MAP ROM AT ZERO
			"STR     R1, [R0, #0]\n"
			"LDR     R0, =0x1FFF0000\n" //; ROM BASE
			"LDR     SP,[R0, #0]\n"     //; SP @ +0
			"LDR     R0,[R0, #4]\n"     //; PC @ +4
			"BX      R0\n");
	//jump_to_BootLoader();
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M767(void)  // prepare system for download (erase pages, etc) (uses E, P)
{
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}
////////////////////////////////////////////////////////////////////////////////

void M_Code_M768(void)  // process next line of intel hex format of system bootloader data (uses P, comment)
{
	// MCODE M768 <"P" lineNum> <;intel hex format line>
	// MCODE
	// MCODE    process next line of intel hex format of system bootloader data
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M769(void)  // exit the system bootloader
{
	// MCODE M769
	// MCODE
	// MCODE    end the bootloader
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M770(void)  // leave system bootloader and jump to application main()
{
	// MCODE M770
	// MCODE
	// MCODE    leave system bootloader and jump to application main()
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M771(void)  // load laser image data controls (S, O, P)
{
	// MCODE M771 ["S" scale] ["O" offset] ["P" max]
	// MCODE
	// MCODE    controls for manipulation laser raster image data
	// MCODE    data_to_laser = maximum(((inputData - offset) * scale), max)
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("see M622");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M772(void)  // reset metrics for new job (uses S)
{
	// MCODE M772 ["S" autoDump on M30]
	// MCODE
	// MCODE    reset metrics for new job
#ifdef COLLECT_METRICS
	bzero(&_metrics, sizeof(metricsStruct));
	_metrics.startTimeSec = _gs._seconds;   // record job start time
	if (ARG_S_PRESENT)
		_metrics.autoDumpOnM30 = (int)ARG_S;
#endif
}

////////////////////////////////////////////////////////////////////////////////

void printReportBlankLine(void)
{
	sendInfo("");
}
void printReportHorizontalLine(void)
{
	sendInfo("******************************************************************");
	pauseToTransmitBufferToEmpty();
}

void printReportHeader(char *s)
{
	pauseToTransmitBufferToEmpty();
	printReportBlankLine();
	printReportHorizontalLine();
	sprintf(_tmpStr, "**  Begin %s Report", s); sendInfo(_tmpStr);
	printReportHorizontalLine();
	printReportBlankLine();
	pauseToTransmitBufferToEmpty();
}

void printReportDivider(char *s)
{
	pauseToTransmitBufferToEmpty();
	printReportBlankLine();
	printReportHorizontalLine();
	printReportBlankLine();
	pauseToTransmitBufferToEmpty();
}

void printReportTrailer(char *s)
{
	pauseToTransmitBufferToEmpty();
	printReportBlankLine();
	printReportHorizontalLine();
	sprintf(_tmpStr, "**  End %s Report", s); sendInfo(_tmpStr);
	printReportHorizontalLine();
	printReportBlankLine();
	pauseToTransmitBufferToEmpty();
}

void printReportGetLocationString(boolean oldLocation, byte device, char *s)
{
	if (oldLocation)
	{
		int head = 0; int yoke = 0; int special = 0;
		if (device == 41) { special = device; }
		else if (device == 16)  { special = device; }
		else if (device == 26)  { special = device; }
		else if (device == 36)  { special = device; }
		else if (device == 46)  { special = device; }
		else if (device == 91)  { special = device; }
		else if (device == 92)  { special = device; }
		else if ((device >= 11) && (device <= 15)) { head = device - 10;   yoke = 1; }
		else if ((device >= 21) && (device <= 25)) { head = device - 20;   yoke = 2; }
		else if ((device >= 31) && (device <= 35)) { head = device - 30;   yoke = 3; }
		else if ((device >= 41) && (device <= 45)) { head = device - 40;   yoke = 4; }

		if (special)
		{
			switch (special)
			{
			case 41: sprintf(s, "CO2 Laser");   break;
			case 16: sprintf(s, "Head AUX1");   break;
			case 26: sprintf(s, "3PH Spindle"); break;
			case 36: sprintf(s, "Head AUX2");   break;
			case 46: sprintf(s, "Head AUX3");   break;
			case 91: sprintf(s, "Hotbed1");     break;
			case 92: sprintf(s, "Hotbed2");     break;
			default: break;
			}
		}
		else if (head && yoke)
		{
			sprintf(s, "Yoke%d Head%d", yoke, head);
		}
		else
		{
			sprintf(s, "Unknown    ");
		}
	}
	else
	{   // new mapping
		int head = 0; int yoke = 0; int special = 0;
		if ((device >= 0) && (device <= 19))
		{
			head = (device % 5) + 1;
			yoke = (device / 5) + 1;
		}
		else if ((device >= 20) && (device <= 25))
		{
			special = device;
		}

		if (special)
		{
			switch (special)
			{
			case 20: sprintf(s, "3PH Spindle"); break;
			case 21: sprintf(s, "CO2 laser");   break;
			case 22: sprintf(s, "Head AUX1");   break;
			case 23: sprintf(s, "Head AUX2");   break;
			case 24: sprintf(s, "Hotbed1"); break;
			case 25: sprintf(s, "Hotbed2"); break;
			default: break;
			}
		}
		else if (head && yoke)
		{
			sprintf(s, "Yoke%d Head%d", yoke, head);
		}
		else
		{
			sprintf(s, "Unknown");
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M773(void)  // send print job metrics to host
{
	// MCODE M773
	// MCODE
	// MCODE    send print job metrics to host
#ifdef COLLECT_METRICS
	printReportHeader("M773 - print job metrics");
	sendInfo("                      Time (s)    Dist (m)");
	sendInfo("                      --------    --------");
	sprintf(_tmpStr, "Totals:               %8.0f    %8.3f    (%lu)    [noAccel=%8.0f]",
			_metrics.nonPrinting.time + _metrics.printing.time,
			(_metrics.nonPrinting.dist+_metrics.printing.dist) / 1000.0f,
			_gs._seconds - _metrics.startTimeSec,
			_metrics.nonPrinting.perfectTime + _metrics.printing.perfectTime);      sendInfo(_tmpStr);
	printReportBlankLine();
	sprintf(_tmpStr, "Printing moves:       %8.1f    %8.3f", _metrics.printing.time, _metrics.printing.dist / 1000.0f);        sendInfo(_tmpStr);
	sprintf(_tmpStr, "Non-printing moves    %8.1f    %8.3f", _metrics.nonPrinting.time, _metrics.nonPrinting.dist / 1000.0f);  sendInfo(_tmpStr);
	printReportBlankLine();
	sprintf(_tmpStr, "Accelerating:         %8.1f    %8.3f", _metrics.accel.time, _metrics.accel.dist / 1000.0f);              sendInfo(_tmpStr);
	sprintf(_tmpStr, "Cruising:             %8.1f    %8.3f", _metrics.cruise.time, _metrics.cruise.dist / 1000.0f);            sendInfo(_tmpStr);
	sprintf(_tmpStr, "Decelerating:         %8.1f    %8.3f", _metrics.decel.time, _metrics.decel.dist / 1000.0f);              sendInfo(_tmpStr);
	sendInfo("");
	if ((_metrics.nonPrinting.time > 0) || (_metrics.printing.time > 0))
	{
		if (_metrics.printing.time > 0)
		{
			sprintf(_tmpStr, "avg print speed:     %4.3f mm/sec  (%4.1f mm/min)  [noAccel=%4.1f mm/min]", _metrics.printing.dist / _metrics.printing.time,
					(_metrics.printing.dist / _metrics.printing.time) * 60.0f,
					(_metrics.printing.dist / _metrics.printing.perfectTime) * 60.0f);
			sendInfo(_tmpStr);
		}
		if (_metrics.nonPrinting.time > 0)
		{
			sprintf(_tmpStr, "avg non-print speed: %4.3f mm/sec  (%4.1f mm/min)  [noAccel=%4.1f mm/min]", _metrics.nonPrinting.dist / _metrics.nonPrinting.time,
					(_metrics.nonPrinting.dist / _metrics.nonPrinting.time) * 60.0f,
					(_metrics.nonPrinting.dist / _metrics.nonPrinting.perfectTime) * 60.0f);
			sendInfo(_tmpStr);
		}
		printReportBlankLine();
	}
	sprintf(_tmpStr, "can e steps issued:       %d", _metrics.canbus_e_steps);                                                          sendInfo(_tmpStr);
#define HOBBED_SHAFT_DIAMETER 4.35f
	sprintf(_tmpStr, "approx filament (PI*d)    %4.3f m", ((float)_metrics.canbus_e_steps/3200.0f)*PI*HOBBED_SHAFT_DIAMETER/1000.0f);       sendInfo(_tmpStr);
#define FIL_RADIUS  (1.75f/2.0f)
	sprintf(_tmpStr, "approx filament (PIr^2)   %4.3f m", (float)_metrics.canbus_e_steps/currentOutboxPtr->ExtrusionControl.PulsesPerUnit/PI/sqr(FIL_RADIUS)/1000.0f);          sendInfo(_tmpStr);

	sprintf(_tmpStr, "unprimes issued:          %d", _metrics.unprimes);                                                                    sendInfo(_tmpStr);
	sprintf(_tmpStr, "primes issued:            %d", _metrics.primes);                                                                      sendInfo(_tmpStr);
	sprintf(_tmpStr, "unprime-primes avoided:   %d", _metrics.unprime_primes_avoided);                                                      sendInfo(_tmpStr);
											printReportTrailer("M773");
#endif
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M774(void)  // send queue metrics to host
{
	// MCODE M774
	// MCODE
	// MCODE    send queue metrics to host
#ifdef COLLECT_METRICS
	printReportHeader("M774 - queue metrics");
	int i, total;
	total = 0;
	for (i=0; i<=SIZE_OF_COMMAND_QUEUE; i++)
		total += _metrics.cmdQue_entriesWhenReceived[i];
	sprintf(_tmpStr, "Total commands:                 %4d", total);  sendInfo(_tmpStr);

	total = 0;
	for (i=0; i<=SIZE_OF_MOTION_QUEUE; i++)
		total += _metrics.motionQ_entriesWhenExecuting[i];
	sprintf(_tmpStr, "Total moves:                    %4d", total); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max commands in CmdQue:         %4d (%d)", _metrics.max_CommandsInQue, SIZE_OF_COMMAND_QUEUE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max commands in motionQ:        %4d (%d)", _metrics.max_motionQvalidEntries, SIZE_OF_MOTION_QUEUE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max commands in deferredCmdQue: %4d (%d)", _metrics.max_DeferredCommandsInQue, SIZE_OF_DEFERRED_COMMAND_QUEUE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max chars in raw RX buffer:     %4d (%d)", _metrics.max_rawRxCharsInBuf, SERIAL_RX_RAW_BUFFER_SIZE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max chars in raw USB RX buffer: %4d (%d)", _metrics.max_rawUsbRxCharsInBuf, SERIAL_RX_RAW_USB_BUFFER_SIZE); sendInfo(_tmpStr);
	pauseToTransmitBufferToEmpty();
	sprintf(_tmpStr, "Max chars in urgent RX buffer:  %4d (%d)", _metrics.max_urgentRxCharsInBuf, SERIAL_RX_URGENT_BUFFER_SIZE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max chars in normal RX buffer:  %4d (%d)", _metrics.max_normalRxCharsInBuf, SERIAL_RX_NORMAL_BUFFER_SIZE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max chars in direct RX buffer:  %4d (%d)", _metrics.max_directRxCharsInBuf, SERIAL_RX_DIRECT_BUFFER_SIZE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Max chars in serial TX buffer:  %4d (%d)", _metrics.max_normalTxCharsInBuf, SERIAL_TX_NORMAL_BUFFER_SIZE); sendInfo(_tmpStr);
	sprintf(_tmpStr, "Total serial TX chars rejected: %4d (%d)", _rejected_normalTxChars, SERIAL_TX_NORMAL_BUFFER_SIZE); sendInfo(_tmpStr);
	printReportTrailer("M774");
#endif
}

///////////////////////////////////////////////////////////////////////////////

void M_Code_M775(void)  // send current crashDataFromRAM to host
{
	// MCODE M775
	// MCODE
	// MCODE    send current crashDataFromRAM to host
#ifdef ENABLE_CRASH_LOGGING
	pauseToTransmitBufferToEmpty();
	printReportHeader("M775 - current crashDataFromRAM");
	sendCrashDataFromRAM();
	printReportTrailer("M775");
#endif
}

///////////////////////////////////////////////////////////////////////////////

void M_Code_M776(void)  // send cmd/motionQ usage histograms to host
{
	// MCODE M776
	// MCODE
	// MCODE    send cmd/motionQ usage histograms to host
#ifdef COLLECT_METRICS
	printReportHeader("M776 - cmd/motionQ usage histograms");
	int i, total;
	total = 0;
	for (i=0; i<=SIZE_OF_MOTION_QUEUE; i++)
		total += _metrics.motionQ_entriesWhenExecuting[i];
	sprintf(_tmpStr, "motionQ numEntries histogram at start of move (total moves: %d)", total); sendInfo(_tmpStr);
	sprintf(_tmpStr, " --: %5d (executed while forcing Q empty)", _metrics.motionQ_entriesWhenExecuting[0]);  sendInfo(_tmpStr);
	for (i=1; i<=SIZE_OF_MOTION_QUEUE; i++)
	{
		sprintf(_tmpStr, "%3d: %5d", i, _metrics.motionQ_entriesWhenExecuting[i]);              sendInfo(_tmpStr);
		pauseToTransmitBufferToEmpty();
	}
	printReportBlankLine();
	printReportHorizontalLine();
	printReportBlankLine();
	total = 0;
	for (i=0; i<=SIZE_OF_COMMAND_QUEUE; i++)
		total += _metrics.cmdQue_entriesWhenReceived[i];
	sprintf(_tmpStr, "cmdQue numEntries histogram when cmd received (total cmds: %d)", total); sendInfo(_tmpStr);
	sendInfo("");
	for (i=0; i<=SIZE_OF_COMMAND_QUEUE; i++)
	{
		sprintf(_tmpStr, "%3d: %5d", i, _metrics.cmdQue_entriesWhenReceived[i]);              sendInfo(_tmpStr);
		pauseToTransmitBufferToEmpty();
	}
	printReportTrailer("M776");
#endif
}

///////////////////////////////////////////////////////////////////////////////

void M_Code_M777(void)  // send and/or erase the crash log (uses S, E, D)
{
	// MCODE M777 ["S" "1"] ["E" "1"] ["D" "1"]
	// MCODE
	// MCODE    optionally send and/or clear crashlog
	// MCODE    if ARG_S = 1, the send data
	// MCODE    if ARG_E = 1, erase the log
	// MCODE    if ARG_D = 1, cause a crash

#ifdef ENABLE_CRASH_LOGGING
	if ((ARG_S_PRESENT) && ((int)ARG_S == 1.0f))
		sendCrashDataFromFlash();
	if ((ARG_E_PRESENT) && ((int)ARG_E == 1.0f))
		eraseSystemCrashLog();
	if ((ARG_D_PRESENT) && ((int)ARG_D == 1.0f))
		Crash_Handler("M777");
#endif
}

///////////////////////////////////////////////////////////////////////////////

void M_Code_M778(void)  // enable slice time measurement (uses S)
{
	// MCODE M778 ["S" <sliceNumber>]
	// MCODE
	// MCODE    ARG_S sets the slice number to echo timing to a pin (S=0 for off)
	// MCODE
	// MCODE    enable slice time measurement

#ifdef SLICE_TIMING_MEASUREMENT
	// reset the counters
	initSliceTiming(FALSE);

	if (ARG_S_PRESENT)
		_sliceTiming.matchSlice = (int)ARG_S;
#endif
}

///////////////////////////////////////////////////////////////////////////////

void M_Code_M779(void)  // dump slice time measurements
{
	// MCODE M779
	// MCODE
	// MCODE    dump slice time measurements

}

///////////////////////////////////////////////////////////////////////////////

void M_Code_M780(void)  // enable/disable auto XYZABC position reporting
{
	// MCODE M780 ["P" periodInSeconds] [["S" enableAllInfo] || ["F" enableFeedrate][XYZABC enableSpecificAxis] ["L" lineNum]]
	// MCODE
	// MCODE if no Args present, force a position report based on current settings
	// MCODE    enable/disable auto XYZABC position and feedrate reporting
	// MCODE    if PArg is present, update the reporting rate based on the period range is 0 to 60 seconds in 1ms increments

	if (ARG_P_PRESENT)
	{
		_MailBoxes._positionReportingPeriodMs = iFitWithinRange((int)(ARG_P * 1000.0f), 0, ONE_MINUTE_IN_MS); // range of 0ms to one minute
		_MailBoxes._positionReportingPrescaleCnt = _MailBoxes._positionReportingPeriodMs;
	}
	if (ARG_S_PRESENT)
	{   // all or nothing
		boolean commonFlag      = (ARG_S == 1.0f) ? TRUE : FALSE;
		AutoReportXYZLocation   = commonFlag;
		AutoReportFeedRate      = commonFlag;
		AutoReportLineNum       = commonFlag;
		MotorStructure *M;
		for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
		{
			M->autoReportPosition = AutoReportXYZLocation;
		}
		ForceReportXYZLocation=TRUE;
		ReportXYZLocation();
		ForceReportXYZLocation=FALSE;
	}
	else if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT || ARG_F_PRESENT || ARG_L_PRESENT)
	{
		if (ARG_X_PRESENT || ARG_Y_PRESENT || ARG_Z_PRESENT || ARG_A_PRESENT || ARG_B_PRESENT || ARG_C_PRESENT )
		{
			AutoReportXYZLocation = FALSE;
			MotorStructure *M;
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{   // first, kill all flags
				M->autoReportPosition = AutoReportXYZLocation;
			}
			if (ARG_X_PRESENT) { Motors[M_X].autoReportPosition = (ARG_X == 1.0f) ? TRUE : FALSE;  AutoReportXYZLocation |= Motors[M_X].autoReportPosition; }
			if (ARG_Y_PRESENT) { Motors[M_Y].autoReportPosition = (ARG_X == 1.0f) ? TRUE : FALSE;  AutoReportXYZLocation |= Motors[M_Y].autoReportPosition; }
			if (ARG_Z_PRESENT) { Motors[M_Z].autoReportPosition = (ARG_Z == 1.0f) ? TRUE : FALSE;  AutoReportXYZLocation |= Motors[M_Z].autoReportPosition; }
			if (ARG_A_PRESENT) { Motors[M_A].autoReportPosition = (ARG_A == 1.0f) ? TRUE : FALSE;  AutoReportXYZLocation |= Motors[M_A].autoReportPosition; }
			if (ARG_B_PRESENT) { Motors[M_B].autoReportPosition = (ARG_B == 1.0f) ? TRUE : FALSE;  AutoReportXYZLocation |= Motors[M_B].autoReportPosition; }
			if (ARG_C_PRESENT) { Motors[M_C].autoReportPosition = (ARG_C == 1.0f) ? TRUE : FALSE;  AutoReportXYZLocation |= Motors[M_C].autoReportPosition; }
		}
		if (ARG_F_PRESENT) AutoReportFeedRate = (ARG_F == 1.0f) ? TRUE : FALSE;
		if (ARG_L_PRESENT) AutoReportLineNum = (ARG_L == 1.0f) ? TRUE : FALSE;
	}
	else
	{   // no args, so force a report of the XYZ position
		ForceReportXYZLocation=TRUE;
		ReportXYZLocation();
		ForceReportXYZLocation=FALSE;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M781(void)  // write hardware type to flash (option bytes) (uses T, D, O, P)
{
	// MCODE M781  <"T" toolSelector> <"D" hwType> <"O" hwRev> <"P" password>
	// MCODE
	// MCODE    if TArg is not present, error
	// MCODE    if SArg, is not present, error.
	// MCODE    if PArg, is not present, error.
	// MCODE    DArg - hardware device type (0-255)
	// MCODE    OArg - hardware device revision (0-255)
	// MCODE
	// MCODE    write hardware type to flash (option bytes) using device bootloader (uses S)
	// MCODE
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M782(void)  // enable/disable "print air" feature (uses S)
{
	// MCODE M782 <"S" 0|1>
	// MCODE
	// MCODE    enable/disable extrusion during motion

	if (ARG_S_MISSING) ReportMissingMcodeSArg();

	_printAir = (ARG_S == 1.0f) ? TRUE : FALSE;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M783(void)  // set PersistantUltimusControlHeadAddress
{
	// MCODE M783 <"T" toolSelector>
	// MCODE
	// MCODE    if TArg is not present, then PersistantUltimusControlHeadAddress off
	// MCODE        otherwise PersistantUltimusControlHeadAddress = ARG_T
	// MCODE
	// MCODE    set the PersistantUltimusControlHeadAddress

	if (ARG_T_PRESENT)
	{
		PersistantUltimusControlHeadAddress = (byte)ARG_T;
		if (PersistantUltimusControlHeadAddress == 0)
		{
			PersistantUltimusControlHeadAddress = HH_POSITION_UNPLUGGED;
		}
	}
	else
	{
		PersistantUltimusControlHeadAddress = HH_POSITION_UNPLUGGED; // force to 0 if no argT
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M784(void)  // report system info (version numbers of system and all heads)
{
	// MCODE M784
	// MCODE
	// MCODE    report system info (version numbers of system and all heads)

	int i;
	int found;

	printReportHeader("M784 - System Summary");
	sendInfo("Motion Controller: ");
	printReportBlankLine();
	sendInfoNoCr("   ");
	sendRevisionString("M784");
	sprintf(_tmpStr, "   License key: %s", _sysInfoPtr->lastKeyUsed);           sendInfo(_tmpStr);

	found = 0;
	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)
	{
		if (_MailBoxes._inbox[i].deviceRegistered)
		{
			found++;
			break;
		}
	}

	if (found)
	{   // at least one registered device
		printReportBlankLine();
		printReportHorizontalLine();
		printReportBlankLine();
		sendInfo("Compatible Devices:");
		printReportBlankLine();
		sendInfo("    T#   Location    DeviceType   SW Loaded                  PCB      RTD1     RTD2     RTD3     SubType      SW Codebase  Can# Fmt# ");
		sendInfo("   ----- ----------- ------------ -------------------------- ---___----- -------- -------- -------- ------------ ------------ ---- ---- ");

		found = 0;
		for (i=0; i<NUM_PHYSICAL_DEVICES; i++)
		{
			if (_MailBoxes._inbox[i].deviceRegistered)
			{
				found++;
				inboxStruct *inboxPtr = &_MailBoxes._inbox[i];

				sprintf(_tmpStr, "    T%-3d", inboxPtr->device);                                                        sendInfoNoCr(_tmpStr);
				printReportGetLocationString(TRUE, inboxPtr->device, _rptStr); sprintf(_tmpStr, " %-11s", _rptStr);    sendstring(_tmpStr);
				sprintf(_tmpStr, " %-12s", deviceTypeToString(inboxPtr->deviceType));                                   sendstring(_tmpStr);

				char tweakVersion[8];
				if (inboxPtr->softwareTweakVersion == 'z')
					sprintf(tweakVersion, "%c%c", inboxPtr->softwareTweakVersion, inboxPtr->softwareDebugVersion);
				else
					sprintf(tweakVersion, "%c", inboxPtr->softwareTweakVersion);

				sprintf(_tmpStr, " %s_%u.%03u%s.hex        ", softwareCompileTargetToString(inboxPtr->softwareCodebase, inboxPtr->softwareCompileTarget),
						(int)inboxPtr->softwareMajorVersion,
						(int)inboxPtr->softwareMinorVersion,
						tweakVersion);
				sendNchars(_tmpStr, 27);

				sprintf(_tmpStr, " %-11s",  devicePcbToString(inboxPtr->devicePcb));                                     sendstring(_tmpStr);
				sprintf(_tmpStr, " %-8s",  deviceRtdToString(inboxPtr->deviceRtd1));                                    sendstring(_tmpStr);
				sprintf(_tmpStr, " %-8s",  deviceRtdToString(inboxPtr->deviceRtd2));                                    sendstring(_tmpStr);
				sprintf(_tmpStr, " %-8s",  deviceRtdToString(SOAP_RTD_NONE));                                           sendstring(_tmpStr);
				sprintf(_tmpStr, " %-12s", deviceSubtypeToString(inboxPtr->deviceType, inboxPtr->deviceSubtype));       sendstring(_tmpStr);
				sprintf(_tmpStr, " %-12s", softwareCodebaseToString(inboxPtr->softwareCodebase));                       sendstring(_tmpStr);
				sprintf(_tmpStr, "  %c    %c  ", inboxPtr->fromCAN2 ? '2' : '1', inboxPtr->canbusFormat);               sendstring(_tmpStr);
				sendstringCr("");
				pauseToTransmitBufferToEmpty();
			}
		}
	}
	else
	{
		printReportBlankLine();
		sendInfo("No valid heads/beds detected");
		printReportBlankLine();
	}

	found = 0;
	for (i=0; i<NUM_PHYSICAL_DEVICES; i++)
	{
		if (_MailBoxes._incompatibleDeviceDetected[i])
		{
			found++;
			break;
		}
	}

	if (found)
	{   // at least one registered device
		printReportBlankLine();
		printReportHorizontalLine();
		printReportBlankLine();
		sendInfo("Incompatible Devices (must be loaded with 4.x software):");
		printReportBlankLine();
		sendInfo("     T#  Location   ");
		sendInfo("   ----- -----------");

		found = 0;
		for (i=0; i<256; i++)
		{
			if (_MailBoxes._incompatibleDeviceDetected[i])
			{
				found++;
				sprintf(_tmpStr, "    T%-3d", i);                       sendInfoNoCr(_tmpStr);
				printReportGetLocationString(TRUE, i, _rptStr);
				sprintf(_tmpStr, " %-11s", _rptStr);                    sendstring(_tmpStr);
				sendstringCr("");
				pauseToTransmitBufferToEmpty();
			}
		}
	}
	else
	{
		printReportBlankLine();
		sendInfo("No invalid heads/beds detected");
		printReportBlankLine();
	}
	printReportTrailer("M784");
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M785(void)  // Set motor parameters (uses T,U,A,R,B,P,C,O,S,W) (V1)
{
	// MCODE M785 <"T" toolSelector> ["U" uSteps/fullStep] ["R" maxRate] ["J" Jo] ["A" accel] ["B" boost] ["C" maxCurrPct] ["H" holdingCurrPct] ["O"operatingMethod]
	// MCODE                         ["S"usteps/rev]  ["P" accelPeriodSec] ["D"deg/fullStep] ["E" encAutoCalMeth] ["F" encPredMode] ["L" angErrLimit] [W1 (save)]
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    if UArg is present, set the number of microsteps per full step (A4988/A4982 drivers)
	// MCODE    if RArg is present, set the max allowable step rate
	// MCODE    if JArg is present, set the starting pulse rate for manual feeding
	// MCODE    if AArg is present, set the incremental pulse rate for manual feeding
	// MCODE    if BArg is present, set the currentBoost on the motor driver chip
	// MCODE    if CArg is present, set the maxCurremt draw of the motor as a percentage 0 to 100
	// MCODE    if HArg is present, set the minCurremt draw of the motor as a percentage 0 to 100
	// MCODE    if OArg is present, set the control mechanism of the motor (0=open-loop; 1=btt pid; 2=sim-pid; 3=pos-pid; 4=velo-pid; 5=exp pid)
	// MCODE    if SArg is present, set the # microsteps per rev (ie, 2K) (for closed loop steppers)
	// MCODE    if PArg is present, set the default update period in sec (1/rate) for accel incr
	// MCODE    if DArg is present, set the degrees per full step (ie, F1.8 for a 1..8 degree stepper)
	// MCODE    if EArg is present, set the encoder auto cal method (0=none; 1=every update; 2=every 1.5 rev; 3=32 times a rev
	// MCODE    if FArg is present, set the encoder prediction mode (0=off; 1=encoder pred on -- 47.5usec advance; 2=software pred)
	// MCODE    if LArg is present, set the encoder angle error reporting limit (in degrees)
	// MCODE    if WArg is present, and WArg==1, write settings to flash
	// MCODE
	// MCODE    NOTE: THE E AND F VALUES MUST BE SAVED TO FLASH AND THEN FULLLY RESTART THE CLS DEVICE TO TAKE EFFE
	// MCODE
	// MCODE    Set extruder motor parameters

	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_R_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_MAX_RATE, (uint16_t)ARG_R);
	if (ARG_A_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_ACCEL, (uint16_t)ARG_A);
	if (ARG_J_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_JO, (uint16_t)ARG_J);
	if (ARG_P_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MANUAL_PERIOD_MS, iFitWithinRange((int)(ARG_P * 1000.0f), 0, ONE_MINUTE_IN_MS));

	if (deviceHasAClosedLoopStepper(localOutboxPtr->device))
	{
		if (ARG_C_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_MAX_PCT, convertFloatPowerPctToUnsignedint(fFitWithinRange(ARG_C/100.0f, 0.0f, 1.0f), HH_U16_POWER_PCT_FRAC_BITS));
		if (ARG_H_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_MIN_PCT, convertFloatPowerPctToUnsignedint(fFitWithinRange(ARG_H/100.0f, 0.0f, 1.0f), HH_U16_POWER_PCT_FRAC_BITS));
		if (ARG_O_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CONTROL_METHOD, (uint16_t)(ARG_O));
		if (ARG_D_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_FULL_STEPS_PER_REV, (uint16_t)(360.0f/ARG_D));

		if (deviceIsABttCanAxisMotor(localOutboxPtr->device))
		{	//has native 16384 motor ticks, so cannot use simple uSteps/Step (full step size is 81.92 ticks), not even multiples to power of two
			if (ARG_S_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_REV, (uint16_t)ARG_S); // must be a integer that evenly divides into 16384
		}
		else
		{	// has native 25600 motor ticks, can use integer power of two uSteps/per step .. full step size = 128 ticks.
			if (ARG_S_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_REV, (uint16_t)ARG_S);  // must be a integer that evenly divides into 16384
			if (ARG_U_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP, (uint16_t)ARG_U);  // must be power of 2
		}

		if (ARG_F_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_ENCODER_PREDICTION, (uint16_t)ARG_F);
		if (ARG_E_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_ENCODER_AUTO_CAL, (uint16_t)ARG_E);
		if (ARG_L_PRESENT) canSendOutboxInitValue1x32(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_ANGLE_ERROR_LIMIT, *(uint32_t *)&ARG_L); //sending flt

		if ((ARG_W_PRESENT) && (ARG_W == 1.0f)) sendMotorSaveConfigToDevice(localOutboxPtr->device);
	}
	else
	{
		if (ARG_B_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_CURRENT_BOOST, (uint16_t)ARG_B);
		if (ARG_U_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP, (uint16_t)ARG_U);
		else if (ARG_S_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_MICROSTEPS_PER_STEP, (uint16_t)(ARG_S / 200.0f));
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M786(void)  // set closed loop stepper PID control values (uses T, P, I, D, A, W)
{
	// MCODE M786 <T<toolSelector>> [S selectPID] [P<Kp>] [I<Ki>] [D<Kd>] [A<advScale][W1 (save)]
	// MCODE
	// MCODE    if argT is not present, the prior selected device/alias is used
	// MCODE    if argS present, set which PID values to change (0=open-loop; 1=btt pid; 2=basic-pid(405bd); 3=rpm-pid; 4=experimental-pid; 5=spare pid)
	// MCODE    else loads POS_PID by default
	// MCODE    if argP is present, set the PID Kp term
	// MCODE    if argI is present, set the PID Ki term
	// MCODE    if argD is present, set the PID Kd term
	// MCODE    if argA is present, set the sw advance limit as a scaling of full step (1.0 == fullStep)
	// MCODE    if rgW==1, write settings to flash
	// MCODE
	// MCODE    set closed loop stepper PID control values

	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	inboxStruct *inboxPtr = getInboxPointer(localOutboxPtr->device);
	//NUKE if (isAPhysicalDevice(localOutboxPtr->device) && (inboxPtr->softwareMajorVersion <= 4) && (inboxPtr->softwareMinorVersion <= 52))
	if (isAPhysicalDevice(localOutboxPtr->device) && deviceSoftwareRevisionIsAtOrBelow(inboxPtr->device, 4, 52, 'z'))
	{	//old format
		if (ARG_P_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KP, (uint16_t)ARG_P);
		if (ARG_I_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KI, (uint16_t)ARG_I);
		if (ARG_D_PRESENT) canSendOutboxInitValue1x16(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KD, (uint16_t)ARG_D);
	}
	else
	{
		if (ARG_S_MISSING) { ReportMissingMcodeSArg(); return; }
		if (ARG_P_PRESENT) canSendOutboxInitValue2x32(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KP, *(uint32_t *)&ARG_P, (uint16_t)ARG_S);
		if (ARG_I_PRESENT) canSendOutboxInitValue2x32(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KI, *(uint32_t *)&ARG_I, (uint16_t)ARG_S);
		if (ARG_D_PRESENT) canSendOutboxInitValue2x32(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_KD, *(uint32_t *)&ARG_D, (uint16_t)ARG_S);
		if (ARG_A_PRESENT) canSendOutboxInitValue2x32(localOutboxPtr, DEV_INIT_AREA_MTR, DEV_INIT_INDEX_MTR_PID_ADV, *(uint32_t *)&ARG_A, (uint16_t)ARG_S);
	}
	if ((ARG_W_PRESENT) && (ARG_W == 1.0f)) sendMotorSaveConfigToDevice(localOutboxPtr->device);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M787(void)  // calibrate can based closed-loop motor (uses T, P, C, F)
{
	// MCODE M787 <"T" toolSelector> <"P" password> <C1 {calibrate}> [F1 {restoreFactoryDefaults}]
	// MCODE
	// MCODE	if argT missing, error;
	// MCODE    if argP != 666, error
	// MCODE    if argC==1, calibrate motor
	// MCODE    if argF==1, reset factory defaults
	// MCODE
	// MCODE    calibrate can based closed-loop motor


	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
	if (ARG_P_MISSING)   { ReportMissingMcodePArg();  return; }  // must have a valid password for this command
	if ((uint32_t)ARG_P != KARLS_PASSWORD)  {ReportInvalidMcodePArgInt(); return; }   //must have a valid password for this command
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (isAPhysicalDevice(localOutboxPtr->device) && deviceHasAClosedLoopStepper(localOutboxPtr->device))
	{	// no aliases
		if (ARG_F_PRESENT && (ARG_F == 1.0f))
		{
			canPackIntoTxQueueNoData(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_RESTORE_DEFAULTS, BUFFERED_MSG);
		}
		if (ARG_C_PRESENT && (ARG_C == 1.0f))
		{
			canPackIntoTxQueueNoData(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_CALIBRATE, BUFFERED_MSG);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M788(void)  // reset can based closed-loop axis motor (uses T, F, D, P)
{
	// MCODE M788 <"T" toolSelector> [F restoreFactoryDefaults] [D deviceReset] [P resetPosition] [C canReset]
	// MCODE
	// MCODE	if argT missing, error;
	// MCODE    if argF==1, reset factory defaults
	// MCODE    if argD==1, "power-on-reset" of device
	// MCODE    if argC==1, force device to re-check in with system
	// MCODE    if argP==1, reset the desired position to match the current encoder position
	// MCODE    if NO ARGS are specified, behavior same as D1
	// MCODE
	// MCODE    calibrate can based closed-loop motor

	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	// allow aliases
	if (ARG_F_PRESENT && (ARG_F == 1.0f))
	{
		canPackIntoTxQueueNoData(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_RESTORE_DEFAULTS, BUFFERED_MSG);
	}
	else if (ARG_P_PRESENT && (ARG_P == 1.0f))
	{
		canPackIntoTxQueueNoData(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_RESET_POSITION, BUFFERED_MSG);
	}
	else if (ARG_D_PRESENT && (ARG_D == 1.0f))
	{
		sendMotorResetToDevice(localOutboxPtr->device);
	}
	else if (ARG_C_PRESENT && (ARG_C == 1.0f))
	{
		sendSetSwResetBit(localOutboxPtr->device);
	}
	else
	{
		sendMotorResetToDevice(localOutboxPtr->device);
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M789(void)   // send sideband step pulses to a canAxisMotor (uses T, P, S, C, F)
{
	// MCODE M789 <"T" toolSelector> <"P" password> <S distance> [C forcePct] [F rate]
	// MCODE
	// MCODE	if argT missing, error;
	// MCODE    if argP != 666, error
	// MCODE    if argS is the distance to move (+/-)	[LIMITED DISTANCE
	// MCODE    if argC present, set the force limit for the sideband move
	// MCODE    if argF present, then use argF speed for the sideband move
	// MCODE    else use the REHOMING_RATE
	// MCODE
	// MCODE    send sideband step pulses to a canAxisMotor

	if (ARG_T_MISSING) { ReportMissingMcodeTArg(); return; }
	if (ARG_P_MISSING)   { ReportMissingMcodePArg();  return; }  // must have a valid password for this command
	if ((uint32_t)ARG_P != KARLS_PASSWORD)  {ReportInvalidMcodePArgInt(); return; }   //must have a valid password for this command
	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if (ARG_S_PRESENT && isAPhysicalDevice(localOutboxPtr->device) && deviceIsACanAxisMotor(localOutboxPtr->device))
	{	// no aliases
		MotorStructure *M = getMotorPtrFromDeviceAddress(localOutboxPtr->device);
		if ((M != NULL) && M->canMotor)
		{	// device has checked in as a canAxisMotor
			float distance = fFitWithinRange((int32_t)convertArgToMM(ARG_S), -1.0f, 1.0f);	// per Karl, limit to +/- 1mm
			if (M->Direction.InvertDefault)
				distance *= -1.0f;

			int16_t steps = distance * M->PulsesPerUnit;

			float rateUPS = M->RatesInUPS[REHOMING];	//default
			if (ARG_F_PRESENT)
				rateUPS = convertArgToMM(ARG_F) / 60.0f;
			uint16_t stepRate = iFitWithinRange((uint32_t)(rateUPS * M->PulsesPerUnit), 1, 65535);

			uint16_t forcePct = ARG_C_PRESENT ? convertFloatPowerPctToUnsignedint(fFitWithinRange(ARG_C/100.0f, 0.0f, 1.0f), HH_U16_POWER_PCT_FRAC_BITS) : 0;

			sendMotorMotionValues(localOutboxPtr->device, M);
			canPackIntoTxQueue3x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_CLOSED_LOOP_MOTOR_CONTROL, CLOSED_LOOP_MOTOR_SIDE_STEP, BUFFERED_MSG, steps, stepRate, forcePct);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M790(void)  // notify layer change
{
	// MCODE M790
	// MCODE
	// MCODE    notify layer change -- send info back to the host

	CurrentLayer++;
	ARG_S=(float)CurrentLayer;  // stuff into ARG_S for that Send... will print the CurrentLayer Value
	SendCurrentMcodeExecutionNotice(FALSE);

	if (_requestToPauseAtEndOfLayer)
	{   //this will pause before the next Move (already SINGLE_STEP, so no need to muck with the motionQ
		setPauseGcodeFlag(PAUSE_AT_END_OF_LAYER_CHAR); // let repetrel know layer pause has started
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M791(void)  // notify take picture event
{
	// MCODE M791  (stub only? - no action taken)
	// MCODE
	// MCODE    notify take picture event

	SendCurrentMcodeExecutionNotice(FALSE);
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M795(void)  // sets jogvalueValueInUnits (uses S)
{
	// MCODE M795 <"S" jogZvalueInMicrons>
	// MCODE
	// MCODE    if SArg, is not present, error.
	// MCODE    else if SArg < 0, error.
	// MCODE    else if SArg > 10000, jogvalue = 10000 (10mm)
	// MCODE    else jogvalue is set to SArg
	// MCODE
	// MCODE    sets jogvalueValueInUnits

	if (ARG_S_MISSING) ReportMissingMcodeSArg();
	else if (ARG_S < 0.0f)  ReportInvalidMcodeSArg();
	else
	{
		ARG_S = fminf(JOG_MAXIMUM_VALUE_IN_MICRONS, ARG_S);
		_jogZ.jogValueInUnits = ARG_S * MICRONS_TO_MM;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M796(void)  // Sets the jog increment (uses X, Y, Z, A, B, C)
{
	// MCODE M796 ["X,Y,Z,A,B,C" <jogIncrementInMicrons>]
	// MCODE
	// MCODE    if X,Y,Z,A,B,C are present, then load the jog increment in microns for that axis
	// MCODE
	// MCODE    Sets the per axis jog increment

	MotorStructure *M;

	for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
	{   // whiz through all the motors, if there's a valid ARG from the command, then set accel constant
		if (motorArgPresent(M))
		{   // limit to small positive number to avoid divide by 0 and other issues
			M->JogIncrement = fFitWithinRange(getMotorArg(M), 0.0f, JOG_MAXIMUM_VALUE_IN_MICRONS) * MICRONS_TO_MM;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M797(void)  // enable/disable debug reporting strings ">GB:" (uses SPRVLIC, DZ)
{
	// MCODE M797 ["S" <enableMask] [["P" echoProcessedSerialPort] | ["R" echoRawSerialPort]] [["V" varToWatch] ["L" lightSel]] ["I" 0|1] ["C" subMask]
	// MCODE
	// MCODE    ArgS enable/disable debug reporting strings (MASK)
	// MCODE
	// MCODE       * GB_STRING_MOTION               0x0001  (1)
	// MCODE       * GB_STRING_PRIME                0x0002  (2)
	// MCODE       * GB_STRING_PRIME_RUN            0x0004  (4)
	// MCODE       * GB_STRING_EXTRUSION            0x0008  (8)
	// MCODE       *
	// MCODE       * GB_STRING_COMM_ACK_CHECK       0x0010  (16)
	// MCODE       * GB_STRING_CMD_QUE              0x0020  (32)
	// MCODE       * GB_STRING_FLOW                 0x0040  (64)
	// MCODE       * GB_STRING_RATE                 0x0080  (128)
	// MCODE       *
	// MCODE       * GB_STRING_CANBUS               0x0100  (256)
	// MCODE       * GB_STRING_DUMP_ON_START        0x0200  (512)
	// MCODE       * GB_STRING_ADD_MOTIONQ          0x0400  (1024)
	// MCODE       * GB_STRING_ECHO_COMM            0x0800  (2048)
	// MCODE       *
	// MCODE       * GB_STRING_WORKBUFFER           0x1000  (4096)
	// MCODE       * GB_STRING_STEPS_PER_NL         0x2000  (8192)
	// MCODE       * GB_STRING_LOOKAHEAD            0x4000  (16384)
	// MCODE       * GB_STRING_ARC_INFO             0x8000  (32768)

	// MCODE       * GB_STRING_RASTER             0x010000  (65536)  (uses subType)
	// MCODE       * GB_STRING_JOG_INFO           0x020000  (131072)
	// MCODE       * GB_STRING_CYL_INFO           0x040000  (262144)
	// MCODE       * GB_STRING_CLS_IFO            0x080000  (524288)  (uses subType)
	// MCODE
	// MCODE    ArgP or ArgR enable/disable echo of input character stream
	// MCODE       ArgR - echo raw RX char before being added to the input serial buffer
	// MCODE       ArgP - echo chars that are pulled out of the input serial buffer
	// MCODE
	// MCODE       0 = NO_ECHO
	// MCODE       1 = ECHO to USB
	// MCODE       3 = ECHO to UART3
	// MCODE       4 = ECHO to UART4
	// MCODE       6 = ECHO to UART6
	// MCODE
	// MCODE   ArgV - variable to watch (non-zero values will flash selected light source)
	// MCODE       FLASHER_VAR_SEL_NONE                   = 0,
	// MCODE       FLASHER_VAR_SEL_ERROR                  = 1,
	// MCODE       FLASHER_VAR_SEL_CMD_RECEIVED           = 2,
	// MCODE       FLASHER_VAR_SEL_ACK_PENDING            = 3,
	// MCODE       FLASHER_VAR_SEL_G4_DWELL_TIMER         = 4,
	// MCODE       FLASHER_VAR_SEL_GCODE_PAUSED           = 5,
	// MCODE       FLASHER_VAR_SEL_ABORTING               = 6,
	// MCODE       FLASHER_VAR_SEL_BLOCK_ALL_MOTION       = 7,
	// MCODE       FLASHER_VAR_SEL_BLOCK_ABS_MOTION       = 8,
	// MCODE       FLASHER_VAR_SEL_MOTION_OCCURRED        = 9, (deprecated)
	// MCODE       FLASHER_VAR_SEL_VALID_KEY              = 10,
	// MCODE
	// MCODE       FLASHER_VAR_SEL_HOME_STATE_X           = 21,
	// MCODE       FLASHER_VAR_SEL_FAULT_STATE_X          = 22,
	// MCODE       FLASHER_VAR_SEL_LIMIT1_STATE_X         = 23,
	// MCODE       FLASHER_VAR_SEL_LIMIT2_STATE_X         = 24,
	// MCODE
	// MCODE       FLASHER_VAR_SEL_HOME_STATE_Y           = 31,
	// MCODE       FLASHER_VAR_SEL_FAULT_STATE_Y          = 32,
	// MCODE       FLASHER_VAR_SEL_LIMIT1_STATE_Y         = 33,
	// MCODE       FLASHER_VAR_SEL_LIMIT2_STATE_Y         = 34,
	// MCODE
	// MCODE       FLASHER_VAR_SEL_HOME_STATE_Z           = 41,
	// MCODE       FLASHER_VAR_SEL_FAULT_STATE_Z          = 42,
	// MCODE       FLASHER_VAR_SEL_LIMIT1_STATE_Z         = 43,
	// MCODE       FLASHER_VAR_SEL_LIMIT2_STATE_Z         = 44,
	// MCODE
	// MCODE       FLASHER_VAR_SEL_HOME_STATE_A           = 51,
	// MCODE       FLASHER_VAR_SEL_FAULT_STATE_A          = 52,
	// MCODE       FLASHER_VAR_SEL_LIMIT1_STATE_A         = 53,
	// MCODE       FLASHER_VAR_SEL_LIMIT2_STATE_A         = 54,
	// MCODE
	// MCODE       FLASHER_VAR_SEL_HOME_STATE_B           = 61,
	// MCODE       FLASHER_VAR_SEL_FAULT_STATE_B          = 62,
	// MCODE       FLASHER_VAR_SEL_LIMIT1_STATE_B         = 63,
	// MCODE       FLASHER_VAR_SEL_LIMIT2_STATE_B         = 64,
	// MCODE
	// MCODE       FLASHER_VAR_SEL_HOME_STATE_C           = 71,
	// MCODE       FLASHER_VAR_SEL_FAULT_STATE_C          = 72,
	// MCODE       FLASHER_VAR_SEL_LIMIT1_STATE_C         = 73,
	// MCODE       FLASHER_VAR_SEL_LIMIT2_STATE_C         = 74,
	// MCODE
	// MCODE   ArgL - selected light source
	// MCODE       FLASHER_LIGHT_SEL_NONE       = 0,
	// MCODE       FLASHER_LIGHT_SEL_LED        = 1,
	// MCODE       FLASHER_LIGHT_SEL_DDL        = 2,
	// MCODE       FLASHER_LIGHT_SEL_BOTH       = 3,
	// MCODE
	// MCODE   ArgI - dump motor I/O (step,dir,home,L1,L2,Fault)
	// MCODE         if ArgI==1, then show the logical state of the I/O  (based on the polarity in the Printer Settings)
	// MCODE         if ArgI==2, then show the physical state of thge I/O

	if (ARG_I_PRESENT)
	{
		MotorStructure *M;
		sendGB("");
		if (ARG_I == 1.0f)
			sendGB("Logical States  (M797 I1)");
		else if (ARG_I == 2.0f)
			sendGB("Physical values  (M797 I2)");
		else if (ARG_I == 3.0f)
			sendGB("Config values  (M797 I3)");

		sendGB("AX ST DR  HM L1 L2 FT (HD)");
		sendGB("== == ==  == == == == ====");

		if (ARG_I == 1.0f)
		{   // Logical state
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{
				if (M->MotorInstalled)
				{
					sprintf(_rptStr, "%c  %c  %c   %c  %c  %c  %c (%c)",
							M->AxisLabel,
							(readControlBitState(&M->Step) == DEASSERTED) ? '-' : 'S',
							(readDirectionBitState(&M->Direction) == DIRECTION_REVERSE) ? 'R' : 'F',
							(M->HomeSense.Enabled == FALSE)   ? '.' : ((readSensorBitState(&M->HomeSense)   == SENSOR_OPEN) ? '-' : 'H'),
							(M->Limit1Sense.Enabled == FALSE) ? '.' : ((readSensorBitState(&M->Limit1Sense) == SENSOR_OPEN) ? '-' : 'L'),
							(M->Limit2Sense.Enabled == FALSE) ? '.' : ((readSensorBitState(&M->Limit2Sense) == SENSOR_OPEN) ? '-' : 'L'),
							(M->FaultSense.Enabled == FALSE)  ? '.' : ((readSensorBitState(&M->FaultSense)  == SENSOR_OPEN) ? '-' : 'F'),
							0x30 + M->HomingDirection);
					sendGB(_rptStr);
				}
				else
				{
					if (M->AxisLabel != 'C')
					{
						sprintf(_rptStr, "%c  not installed", M->AxisLabel);
						sendGB(_rptStr);
					}
				}
			}
			sendGB("<end>");
		}
		else if (ARG_I == 2.0f)
		{   // physical values
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{
				if (M->MotorInstalled)
				{
					sprintf(_rptStr, "%c  %c  %c   %c  %c  %c  %c (%c)",
							M->AxisLabel,
							0x30 + readControlBitValue(&M->Step),
							0x30 + readDirectionBitValue(&M->Direction),
							(M->HomeSense.Enabled == FALSE)   ? '.' : (readSensorBitValue(&M->HomeSense)   ? '1' : '0'),
							(M->Limit1Sense.Enabled == FALSE) ? '.' : (readSensorBitValue(&M->Limit1Sense) ? '1' : '0'),
							(M->Limit2Sense.Enabled == FALSE) ? '.' : (readSensorBitValue(&M->Limit2Sense) ? '1' : '0'),
							(M->FaultSense.Enabled == FALSE)  ? '.' : (readSensorBitValue(&M->FaultSense)  ? '1' : '0'),
							0x30 + M->HomingDirection);
					sendGB(_rptStr);
				}
				else
				{
					if (M->AxisLabel != 'C')
					{
						sprintf(_rptStr, "%c  not installed", M->AxisLabel);
						sendGB(_rptStr);
					}
				}
			}
			sendGB("<end>");
		}
		else if (ARG_I == 3.0f)
		{   // physical values
			for (M=FirstAxisMotorPtr; M<=LastAxisMotorPtr; M++)
			{
				if (M->MotorInstalled)
				{
					sprintf(_rptStr, "%c  %c  %c   %c  %c  %c  %c (%c)",
							M->AxisLabel,
							0x30 + M->Step.Polarity,
							0x30 + M->Direction.InvertDefault,
							0x30 + M->HomeSense.Polarity,
							0x30 + M->Limit1Sense.Polarity,
							0x30 + M->Limit2Sense.Polarity,
							0x30 + M->FaultSense.Polarity,
							0x30 + M->HomingDirection);
					sendGB(_rptStr);
				}
				else
				{
					if (M->AxisLabel != 'C')
					{
						sprintf(_rptStr, "%c  not installed", M->AxisLabel);
						sendGB(_rptStr);
					}
				}
			}
			sendGB("<end>");
		}
		else
		{
			ReportInvalidMcodeIArgInt();
		}
		return;
	}

	if (ARG_S_PRESENT)
	{
		_sendingGBStringsMask = (uint32_t)ARG_S;    //store it as a mask
		_sendingGBStringsSubMask = 0xffffffff;      // default to all on
	}
	if (ARG_C_PRESENT)
	{
		_sendingGBStringsSubMask = (uint32_t)ARG_C;    //store it as a mask
	}

	if (ARG_P_PRESENT)
	{
		switch ((serialPort_t)(uint16_t)ARG_P)
		{
		case SERIAL_PORT_NONE:
		case SERIAL_PORT_USB:
		case SERIAL_PORT_UART3:
		case SERIAL_PORT_UART4:
		case SERIAL_PORT_UART6:
			_echoProcessedSerialStreamPort = (serialPort_t)(uint16_t)ARG_P;
			_echoRawSerialStreamPort = SERIAL_PORT_NONE;
			break;
		default:
			ReportInvalidMcodePArgInt();
			_echoProcessedSerialStreamPort = SERIAL_PORT_NONE;
			break;
		}
	}

	if (ARG_R_PRESENT)
	{
		switch ((serialPort_t)(uint16_t)ARG_R)
		{
		case SERIAL_PORT_NONE:
		case SERIAL_PORT_USB:
		case SERIAL_PORT_UART3:
		case SERIAL_PORT_UART4:
		case SERIAL_PORT_UART6:
			_echoRawSerialStreamPort = (serialPort_t)(uint16_t)ARG_R;
			_echoProcessedSerialStreamPort = SERIAL_PORT_NONE;
			break;
		default:
			ReportInvalidMcodeRArgInt();
			_echoRawSerialStreamPort = SERIAL_PORT_NONE;
			break;
		}
	}

	if (ARG_V_PRESENT)
	{
		setFlasherVarPtr((flasherVarSel_t)(uint16_t)ARG_V);
	}

	if (ARG_L_PRESENT)
	{
		_gs._flasher.lightSel = (flasherLightSel_t)(uint16_t)ARG_L;
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M798(void)  // dump strings to host (debug MCODE) -- warning, must reset  ? '1' : '0'),fter using this MCODE (uses T)
{
	// MCODE M798 <"T" toolSelector>
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE
	// MCODE    dump strings to host  -- warning, must reset after using this MCODE
	// MCODE    changes inbox, current tool, and reportingMask for all devices
	// MCODE    FOR BETTER RESULTS, issue a M738 P28 S1024 prior to M763

	uint16_t i;
	canSwStruct can;

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	_MailBoxes._waitingFor.flags.bit.sliceNeedsExtraTime = 1;

	if (deviceIsAPhysicalDevice(localOutboxPtr->device) == TRUE)
	{   // only allow operation on a real, physical device (no aliases)
		_MailBoxes._hostTrafficReportingPeriodMs = 0;   // turn off auto reporting
		sendInfo("HEAD ERROR MESSAGES (from M798)");

		can.msgType = CAN_WRITE;
		can.msgId = CAN_MSG_REPORT_ERROR;
		can.page = NO_PAGE;
		can.numBytes = 8;
		for (i=4; i<8; i++)
		{
			can.payload.u8[i] = 0x50 + i;  // make up 4 error values
		}
		can.device = localOutboxPtr->device;
		for (i=0; i<NUM_DEVICE_ERROR_CODES; i++)
		{
			can.page = i;
			can.payload.u8[0] = _deviceErrorDescriptionTable[i].unit;
			can.payload.u8[1] = _deviceErrorDescriptionTable[i].code;
			can.payload.u16[1] = i; // increment error count
			printDeviceErrorMessage(&can);
		}
		can.payload.u8[0] = 0x69;
		printDeviceErrorMessage(&can);
	}

	_MailBoxes._waitingFor.flags.bit.sliceNeedsExtraTime = 0;
}

////////////////////////////////////////////////////////////////////////////////

#define DEBUG_DEVICE_POSITION ((byte)0xff)
#define DEBUG_CLOCKS
#ifdef DEBUG_CLOCKS
#include "stm32f4xx_rcc.h"
#endif

void M_Code_M799(void)  // get PLL and Clock status for processor
{
	// MCODE M798
	// MCODE
	// MCODE    get PLL and Clock status for the processor
	// MCODE RETIRED
	ReportRetiredMcode("");
}

////////////////////////////////////////////////////////////////////////////////

uint16_t convertArg100ToUint16Pct(float arg)
{
	return(convertFloatPowerPctToUnsignedint((fFitWithinRange(arg, 0.0f, 100.0f) / 100.0f), HH_U16_POWER_PCT_FRAC_BITS));
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M800(void)  // sonicator control
{
	// MCODE M800 ["T" toolSelector] <"C0"> <"I" page>
	// MCODE M800 ["T" toolSelector] <"C" control> <"I" index> <"D" dacValue> ["W" waveformMode]
	// MCODE M800 ["T" toolSelector] <"C" control> <"I" index> <"F" freq  "A" duty> ["B" duty]>
	// MCODE M800 ["T" toolSelector] <"C" control> <"I9"> <<"F" freq ["W" wobble] ["S" scale]>
	// MCODE M800 ["T" toolSelector] <"C7"> ["I" enable]
	// MCODE M800 ["T" toolSelector] <"C8"> <"W4"> <"F" freq> ["S" duty]                        -- experimental -- square wave freq generator
	// MCODE M800 ["T" toolSelector] <"C8"> <"W3"> <"F" freq> <"P" points> <"D" dacMaxValue>    -- experimental -- sine wave freq generator
	// MCODE
	// MCODE    if TArg is not present, the prior selected device/alias is used
	// MCODE    argC required - "control selection"
	// MCODE        0 : select display page
	// MCODE        1 : set DAC value
	// MCODE        2 : set Timer as PWM1
	// MCODE        3 : set Timer as PWM2
	// MCODE        4 : set Timer as Toggle
	// MCODE        5 : set Timer as free running external clock (50% DC)
	// MCODE        6 : set Timer as free running internal clock (50% DC)
	// MCODE        7 : oscope
	// MCODE		8 : waveform generator -- experimental for 405 board
	// MCODE
	// MCODE    argI index
	// MCODE        if (argc == display) then argI selects display page
	// MCODE            0 : general info
	// MCODE            1 : oscilloscope
	// MCODE        if (argC == 1 == DAC) then argI selects:
	// MCODE            1 : Channel_1 (VCO control)
	// MCODE            2 : Channel_2 (Gate Drive Bias)
	// MCODE            NOTE: add W=1 for triangle waveform; W=2 for noise. (D > 0 sets amplitude/range)
	// MCODE        if (argC == TimerX) then argI selects:
	// MCODE            1 : Scope
	// MCODE            2 : Gate Drive (freq/pwm control method)
	// MCODE            3 : Backlight
	// MCODE            4 : DC-DC
	// MCODE            5 : (debug VCO)
	// MCODE            6 : (debug DAC1   -- set frequency for DAC1 waveform mode
	// MCODE            7 : (debug DAC2)  -- set frequency for DAC2 waveform mode
	// MCODE            8 : Oscope trigger offset (also debug GPIO DMA)
	// MCODE            9 : VCO count
	// MCODE           10 : Boost
	// MCODE           11 : Buck
	// MCODE           12 : (internal usec timer)
	// MCODE           13 : HSS1
	// MCODE           14 : HSS2
	// MCODE
	// MCODE        argS is used as:   dacIncr = ((target - actual) * argS) + 1
	// MCODE            so argS is in the range of 0.0 to 1.0;  (sent as a 1.15 fixed point fraction)
	// MCODE
	// MCODE 		argW is waveform selecton (0=none; 1=sawtooth, 2-noise; 3=sine; 4=square/pwm
	// MCODE		argS is number of sample points in sine waveform mode or % for square wave
	// MCODE
	// MCODE    Sonicator Head Control
	// MCODE
	// MCODE    NOTE:  if DAC1 data is written, then this static value will be the input
	// MCODE           to the DAC.  If the Timer VCO frequency is written, then this will
	// MCODE           will be the target frequency and the device will begin to sweep the
	// MCODE           DAC input until the target freq is achieved.
	// MCODE    NOTE: setting F to 0 for any timer will disable that clock/timer.
	// MCODE
	// MCODE      EXAMPLES:
	// MCODE        M800 Tx C0 I1                       ; show page 1 on the display (page 0 is first page)
	// MCODE
	// MCODE        M800 Tx C1 I1  D1234                ; write 1234 to DAC1 (VCO auto tune DISABLED)
	// MCODE        M800 Tx C1 I1  D0                   ; write 0 to DAC1 (VCO auto tune ENABLED)
	// MCODE        M800 Tx C1 I2  D1234                ; write 1234 to DAC2 (set gate drive bias)
	// MCODE
	// MCODE        M800 Tx C2 I1  F21000000 A50 B50  ; set oscope clock to 21MHz with a 50% duty cycle on clks A & B
	// MCODE        M800 Tx C2 I3  F2000 A100           ; tft backlight - full brightness
	// MCODE        M800 Tx C4 I4  F100000 A0 B25     ; set DC-DC clkB 25% phase shift from clkA  <=== critical -- this one requires toggle mode>
	// MCODE        M800 Tx C5 I10 F75900               ; set boost to 75.9Khz (forced 50% duty)
	// MCODE        M800 Tx C2 I11 F69333 A37.3         ; set buck to 69.3Khz (37.3% duty)
	// MCODE
	// MCODE        M800 Tx C6 I6  F2000                ; set Timer 6 to 2KHz for DAC1 waveform mode
	// MCODE        M800 Tx C1 I1  D2047 W1             ; force dac1 into triangle wave mode (max ampl 2047)
	// MCODE        M800 Tx C6 I7  F9000                ; set Timer 7 to 9KHz for DAC2 waveform mode
	// MCODE        M800 Tx C1 I2  D4000 W1             ; force dac1 into triangle wave mode (max ampl 2047)
	// MCODE
	// MCODE        M800 Tx C2 I9  F765432 W50 S0.0001  ; set VCO target freq to 765432Hz (used if DAC1 set to 0)
	// MCODE                                            ; wobble factor 50; incr=((targ-act)*0.0001) + 1
	// MCODE
	// MCODE        M800 Tx C7 I0                       ; disable oscilloscope
	// MCODE        M800 Tx C7 I1                       ; disable oscilloscope

	outboxStruct *localOutboxPtr = ConvertArgTtoHotheadOutboxPtr();

	if ((int)ARG_C == SONICATOR_PAGE_CTRL_WAVEFORM)
	{
		//special case for waveform generation testing
		//page = waveform
		//u16[0] = max dac value
		//u16[1] = points or pwm/duty
		//u32[1] = freq
		// MCODE M800 ["T" toolSelector] <"C8"> <"W4"> <"F" freq> ["S" duty]                        -- experimental -- square wave freq generator
		// MCODE M800 ["T" toolSelector] <"C8"> <"W3"> <"F" freq> <"S" points> <"D" dacMaxValue>    -- experimental -- sine wave freq generator
		payloadUnion payload;
		byte page = (byte)ARG_W;

		if (ARG_D_MISSING)
			payload.u16[0] = SONICATOR_MAX_DAC12;
		else
			payload.u16[0] = iFitWithinRange((int)ARG_D, 0, SONICATOR_MAX_DAC12);

		if (page == SONICATOR_DAC_WAVEFORM_SQUARE)
		{
			float dutyCycle = (ARG_S_PRESENT) ? fFitWithinRange(ARG_S, 0.0f, 100.0f) : 50.f;
			payload.u16[1] = convertFloatPowerPctToUnsignedint(dutyCycle / 100.0f, HH_U16_POWER_PCT_FRAC_BITS);
		}
		else
		{
			payload.u16[1] = iFitWithinRange((uint16_t)ARG_S, 0, WAVE_GEN_MAX_SAMPLES);
		}
		payload.u32[1] = imax(0, (uint32_t)ARG_F);

		canPackIntoTxQueue8x8(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG, &payload.u8[0]);

	}
	else if (getInboxPointer(localOutboxPtr->device)->deviceType == SOAP_DEV_TYPE_SONICATOR)
	{
		if (ARG_C_MISSING) { ReportMissingMcodeCArg(); return; }
		if (ARG_I_MISSING) { ReportMissingMcodeIArg(); return; }
		if (ARG_I < 0.0f) { ReportInvalidMcodeIArg(); return; }

		int control = (int)ARG_C;
		int periphIndex = (int)ARG_I;

		byte page = 0;
		page |= ((control << SONICATOR_PAGE_CTRL_SHIFT) & SONICATOR_PAGE_CTRL_MASK);
		page |= ((periphIndex << SONICATOR_PAGE_INDEX_SHIFT) & SONICATOR_PAGE_INDEX_MASK);

		switch (control)
		{
		case SONICATOR_PAGE_CTRL_DISPLAY_PAGE:
			if ((periphIndex >= 0) || (periphIndex <= 1000))
			{
				canPackIntoTxQueueNoData(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG);
			}
			break;
		case SONICATOR_PAGE_CTRL_DAC:
			if ((periphIndex >= 1) || (periphIndex <= SONICATOR_MAX_DAC_INDEX))
			{
				canPackIntoTxQueue2x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG,
						iFitWithinRange((int)ARG_D, 0, SONICATOR_MAX_DAC12),
						ARG_W_PRESENT ? (uint16_t)ARG_W : 0);
			}
			else { ReportInvalidMcodeIArg(); }
			break;
		case SONICATOR_PAGE_CTRL_TIMER_PWM1:
		case SONICATOR_PAGE_CTRL_TIMER_PWM2:
		case SONICATOR_PAGE_CTRL_TIMER_TOGGLE:
		case SONICATOR_PAGE_CTRL_TIMER_EXT_CLK:
		case SONICATOR_PAGE_CTRL_TIMER_INT_CLK:
			if ((periphIndex >= 1) || (periphIndex <= SONICATOR_MAX_TIMER_INDEX))
			{
				if (periphIndex == 9) // set up VCO
				{   // special case for VCO control
					uint16_t wobble = iFitWithinRange((int)ARG_W, 0, 200);
					uint16_t scale = (ARG_S_PRESENT) ? convertFloatPowerPctToUnsignedint( fFitWithinRange(ARG_S, 0.0f, 1.0f), HH_U16_POWER_PCT_FRAC_BITS) : 0;
					canPackIntoTxQueue1x32_2x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG,
							imax(0, (uint32_t)ARG_F), wobble, scale);
				}
				else
				{
					uint16_t dutyA = (ARG_A_PRESENT) ? convertArg100ToUint16Pct(ARG_A) : convertArg100ToUint16Pct(50.0f);
					uint16_t dutyB = (ARG_B_PRESENT) ? convertArg100ToUint16Pct(ARG_B) : convertArg100ToUint16Pct(50.0f);

					switch (control)
					{   // force special cases
					case SONICATOR_PAGE_CTRL_TIMER_EXT_CLK: dutyA = convertArg100ToUint16Pct(50.0f); dutyB = 0; break; // force 50% / 0%
					case SONICATOR_PAGE_CTRL_TIMER_INT_CLK: dutyA = 0; dutyB = 0; break;
					case SONICATOR_PAGE_CTRL_TIMER_PWM1:
					case SONICATOR_PAGE_CTRL_TIMER_PWM2:
					case SONICATOR_PAGE_CTRL_TIMER_TOGGLE:
					default: break;
					}

					canPackIntoTxQueue1x32_2x16(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG,
							imax(0, (uint32_t)ARG_F), dutyA, dutyB);
				}
			}
			else { ReportInvalidMcodeIArg(); }
			break;
		case SONICATOR_PAGE_CTRL_SCOPE:
			canPackIntoTxQueueNoData(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG);
			break;
		case SONICATOR_PAGE_CTRL_WAVEFORM:
			{
				//special case for waveform generation testing
				//page = waveform
				//u16[0] = max dac value
				//u16[1] = points or pwm/duty
				//u32[1] = freq
				// MCODE M800 ["T" toolSelector] <"C8"> <"W4"> <"F" freq> ["S" duty]                        -- experimental -- square wave freq generator
				// MCODE M800 ["T" toolSelector] <"C8"> <"W3"> <"F" freq> <"S" points> <"D" dacMaxValue>    -- experimental -- sine wave freq generator
				payloadUnion payload;
				page = (byte)ARG_W;

				payload.u16[0] = iFitWithinRange((int)ARG_D, 0, SONICATOR_MAX_DAC12);

				if (page == SONICATOR_DAC_WAVEFORM_SINE)
				{
					float dutyCycle = (ARG_S_PRESENT) ? fFitWithinRange(ARG_S, 0.0f, 100.0f) : 50.f;
					payload.u16[1] = convertFloatPowerPctToUnsignedint(dutyCycle / 100.0f, HH_U16_POWER_PCT_FRAC_BITS);
				}
				else
				{
					payload.u16[1] = iFitWithinRange((uint16_t)ARG_S, 0, WAVE_GEN_MAX_SAMPLES);
				}
				payload.u32[1] = imax(0, (uint32_t)ARG_F);

				canPackIntoTxQueue8x8(CAN_WRITE, localOutboxPtr->device, CAN_MSG_V1_HEAD_FUNCTION_CONTROL, page, BUFFERED_MSG, &payload.u8[0]);

			}
			break;
		default:
			ReportInvalidMcodeCArg();
			break;
		}
	}
	else
	{
		ReportMcodeError("Head is not a Sonicator");
	}
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M868(void)  // adjust the Z position in a positive direction (move table up)
{
	// MCODE M868
	// MCODE
	// MCODE adjust the Z table up (absolute amount set by M795)
	// MCODE resulting in the '0' position moving UP

	JogMotor(&Motors[M_Z], _jogZ.jogValueInUnits * -1.0f); // JogMotor jogs in mm or degrees;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M869(void)  // adjust the Z position in a negative direction (move table down)
{
	// MCODE M868
	// MCODE
	// MCODE adjust the Z table down (absolute amount set by M795)
	// MCODE resulting in the '0' position moving DOWN

	JogMotor(&Motors[M_Z], _jogZ.jogValueInUnits * 1.0f); // JogMotor jogs in mm or degrees;
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M960(void)  // rectangular pocket mill (uses X, Y, E)
{
	// MCODE M960 - stub -- does nothing
	// MCODE M960 <"X" pocketWidth> <"Y" pocketHeight> <"E" toolDiameter>
	// MCODE
	// MCODE    if TArg is not present, the prior selected physical device is used
	// MCODE
	// MCODE    rectangular pocket mill setup for pocket width and height; and tool diameter

	if (ARG_X_MISSING) { ReportMissingMcodeXArg(); return; }//final width of the pocket
	if (ARG_Y_MISSING) { ReportMissingMcodeYArg(); return; }//final height of the pocket
	if (ARG_E_MISSING) { ReportMissingMcodeEArg(); return; }//tool diameter
}

////////////////////////////////////////////////////////////////////////////////

void M_Code_M7734(void)  // 102207 rev 7 manufacturing test  (T, L, E) (Test, Sms, Loop, EallowableErrors, Allow)
{
#ifdef HYDRA_DIAGS
	runHydraDiagnostics();
#else
	sendstringCr(">MC:M7734: S-1:"); // send fake SendCurrentMcodeExecutionNotice indicating "did not run" with the S-1
#endif
}
///////////////////////////////////////////////////////////////////////////////


