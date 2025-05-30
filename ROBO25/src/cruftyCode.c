// home for dead / unused / outdated / etc code
#if 0

// from main.c

//DEPRECATED - NUKE
//// array element 0 is position of first/top/closest to the hothead
//// array element 4 is position of fifth/bottom/farthest from the hothead
//int Z_Offset[5]={10000,50000,100000,500000,650000};
//int TempZ_Offset[5]={0,0,0,0,0};
//sensorStruct Z_CalibrationSensor[5];
//int ReportZHomeFlag=0;  //used to ask for a new Z offset to be sent to the host
//boolean calibratingZAxis=FALSE;

#endif //0
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if 0
// from MotorDrivers.c

// from ProcessMotion()
#if 0 //NUKE old way
		if (M->Axis != M_C)
		{   // normal axis, proceed with forming step
			assertControlBit(&M->Step); // start forming the step pulse
		}
		else
		{   // axis C is multi-purpose,
#if 0 // expanded version  NUKE
			// 4 cases:  actual 6-axis motion or stunt double for E --> which can send via canbus or direct or both
			if (!_canbusStepForE && !_directStepForE)
			{   //normal C axis motion
				assertControlBit(&M->Step); // start forming the step pulse
			}
			else if (!_canbusStepForE && _directStepForE)
			{   // direct drive of the extruder
				assertControlBit(&M->Step); // start forming the step pulse
			}
			else if (_canbusStepForE && !_directStepForE)
			{   // send can packet to tell extruder to advance one step
				StepExtruder(currentOutboxPtr);
			}
			else if (_canbusStepForE && _directStepForE)
			{   // send can packet to tell extruder to advance one step
				StepExtruder(currentOutboxPtr, HH_MOTOR_FORWARD);
				assertControlBit(&M->Step); // start forming the step pulse
			}
#else // concise version
			// which simplifes to:
			if ((_canbusStepForE == FALSE) || (_directStepForE == TRUE))
			{   // either normal C axis motion OR C is used for direct drive of the extruder
				assertControlBit(&M->Step); // start forming the step pulse
			}
			if (_canbusStepForE == TRUE)
			{   // send can packet to tell extruder to advance one step
				StepExtruder(currentOutboxPtr, HH_MOTOR_FORWARD);
			}
#endif
#endif // old way
//DEPRECATED - NUKE?
//            if (M->CallBackHomeSense)
//            {
//                M->CallBackHomeSense(M);
//            }
//            else
//            {
//                M->HomeSensed = (readDebouncedSensorState(&M->HomeSense) == SENSOR_TRIPPED);
//            }
//DEPRECATED - NUKE?
//                if(calibratingZAxis) // special mode for the engine and system30 with multiple Z sensors on the spine
//                {
//                    CalibrateZHomeSensors();
//                }
//                else if (M->HomeSensed)
#ifdef LATHE_FOOBAR //NUKE
	else if (M->latheMode != LATHE_MODE_OFF)
	{
		int removeThisOnce_Timer_working_for_lathe_pulse_gen;
#ifdef GB_LATHE_STEP_PIN
		assertControlBit(&Motors[M_C].Step);
		deassertControlBit(&Motors[M_C].Step);
#endif
		assertControlBit(&M->Step); // start forming the step pulse
		if (M->SendStepViaCanbus)
		{
			StepExtruder(M->DeviceForCanStep, (M->DIRECTION == 1) ? HH_MOTOR_FORWARD : HH_MOTOR_REVERSE, M->PageMaskForCanStep);
		}
		deassertControlBit(&M->Step);    // finish step pulse - set trailing edge
	}
#endif
=============================

//DEPRECATED - NUKE??
//void CalibrateZHomeSensors(void)
//{//we hard coaded this routine because it is used when calibrating the home sensor
//    MotorStructure *M = &Motors[M_Z];
//    int i;
//
//    for (i=0; i<NUM_Z_CALIBRATION_SENSORS; i++)
//    {
//        if (TempZ_Offset[i] == 0)
//        {
//            if (readSensorBit(&Z_CalibrationSensor[i]) == SENSOR_TRIPPED)
//            {
//                TempZ_Offset[i] = M->POSITION;
//                ReportZHomeFlag = i + 1;  // flag goes from 1 to 5
//
//                if (ReportZHomeFlag == NUM_Z_CALIBRATION_SENSORS)   // array index offset by 1
//                {   // bottom sensor trippped, so stop motion
//                    // now calibration is complete and we can stop the motors from ramming the bottom
//                    M->PULSES_TO_GO = M->PulsesPerUnit * M->HomeHysteresisInUnits;
//                    M->SearchingForHomeSensor = FALSE;  // know where we are, so shut down the homing flag
//                    calibratingZAxis = FALSE;   // kill flag
//                }
//            }
//        }
//    }
//}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#if 0 // original code
void ZHomeSense(void)
{
	MotorZ.Stall_Sense=GPIO_ReadInputData(MotorZ.StallsensePort) & MotorZ.StallSensorBit;
	tempHomeSense=GPIO_ReadInputData(MotorZ.HomePort1) & MotorZ.HomeBitMask;
	if(tempHomeSense< MotorZ.HomeBitMask)
	{
		MotorZ.Home_Sense=1;
		return;
	}
	tempHomeSense = GPIO_ReadInputData(MotorZ.HomePort2) & MotorZ.HomeBitMask2;
	if (tempHomeSense < MotorZ.HomeBitMask2)
	{
		MotorZ.Home_Sense=1;
		return;
	}

	MotorZ.Home_Sense=0;
}
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
////UNUSED and possibly broken
//void HomeSense(MotorStructure *M)
//{   // check each of the extra Z sesors and if any tripped, set the master flag
//    //if (M == &Motors[M_Z])
//#ifdef USE_HYREL_IO
//    if (0)
//    {
//        //GB XXX gregkarl -- warning this does NOT check Z_HOME (only Z_HOME1-Z_HOME5) ... should it be included?
//      if (M->FaultSense.Enabled)
//          readSensorBit(&M->FaultSense);
//
//        int i;
//        for (i=0; i<NUM_Z_CALIBRATION_SENSORS; i++)
//        {
//            if (readSensorBit(&Z_CalibrationSensor[i]) == SENSOR_TRIPPED)
//            {
//                M->HomeSensed = TRUE;
//                return; // save time and exit immediately if one is found tripped
//            }
//        }
//        M->HomeSensed = FALSE;
//    }
//    else
//    {
//        if (M->FaultSense.Enabled)
//          readSensorBit(&M->FaultSense);
//        M->HomeSensed = (readSensorBit(&M->HomeSense) == SENSOR_TRIPPED);
//        if (M->HomeSensed)
//            if (M->FaultSense.Enabled)
//              readSensorBit(&M->FaultSense);
//    }
//#endif
//}
////GB XXX FIX ZHome ZHomeSense -- need to set up IO and everything
//void ZHomeSense(void)
//{   // check each of the extra Z sesors and if any tripped, set the master flag
//#ifdef USE_HYREL_IO
//    //GB XXX gregkarl -- warning this does NOT check Z_HOME (only ZHOME1-5) ... should it be included?
//
//    readSensorBit(&Motors[M_Z].FaultSense);
//
//    int i;
//    for (i=0; i<NUM_Z_CALIBRATION_SENSORS; i++)
//    {
//        if (readSensorBit(&Z_CalibrationSensor[i]) == SENSOR_TRIPPED)
//        {
//            Motors[M_Z].HomeSensed = TRUE;
//            return; // save time and exit immediately if one is found tripped
//        }
//    }
//    Motors[M_Z].HomeSensed = FALSE;
//#endif
//}

////////////////////////////////////////////////////////////////////////////////
#if 0 //NUKE

////////////////////////////////////////////////////////////////////////////////
//void (*CallBackPrimeRun)(void);///signals when to restart the flow of material after a non extrude move or event
//void (*CallBackRetract)(void);///signals when to start retracting the material to stop the flow sharply.


#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#endif // 0
