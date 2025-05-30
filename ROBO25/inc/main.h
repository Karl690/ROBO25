#pragma once
#ifndef main_HEADER // prevent double dipping
#define main_HEADER
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2013  HYREL 3D, LLC.   All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////
//
// since this is the main project include file, also include files the ALL modules
// will need.  Do not list includes that only a few of the modules will need, let
// those modules include those files.
//#define RevisionHeader ">Hy:Pheonix:"
//
////////////////////////////////////////////////////////////////////////////////
///
// File:    main.h
//
////////////////////////////////////////////////////////////////////////////////
typedef void(*PFUNC)(void);

//old revision string, no longer used, please look in revisionHistory.h for current revision
#define SOFTWARE_MAJOR_REVISION     4   // XXX  update when a major change occurs (ie, protocol)
#define SOFTWARE_MINOR_REVISION     253 // XXX  update for major and minor changes
#define SOFTWARE_TWEAK_REVISION    'A'  // XXX  update for small changes ('z' is for experimental ONLY)
#define SOFTWARE_DEBUG_REVISION    'a'  // XXX  char update for debug versions  (applies to 'z' versions only  is for experimental ONLY) (display with M115)
extern const PFUNC F1000HZ[];
extern const PFUNC F100HZ[];
extern const PFUNC F10HZ[];
extern const PFUNC F1HZ[];

#if (SOFTWARE_TWEAK_REVISION == 'z')
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// WARNING:  IF THE ORDER OF ANY OF THESE CHANGE, PLEASE UPDATE initSliceTiming()


////////////////////////////////////////////////////////////////////////////////

//
// Purpose: Contains main specific defines, global references, and method prototypes
//
// Code Revision History:
//
// 4.000a - Initial release (similar level to 3.083m
//      - added CATASTROPHIC abort on bad arcs and motion exceeding scaled limits (M91 ArgS)
//////////RELEASED 2019/05/16
// 4.000b
//      - added to  M686 to also return the key seed as
//        >MC:M686:"seed_value"
//////////RELEASED 2019/05/16
// 4.000c
//      - add to M687 to also return the key that was used to unlock motion
//        >MC:M687:"key_value"
//        >MC:M687:INVALID-"key_value"
//      - return "fake" >MC:M687 whenever connection string is sent to the host containing the last
//        key that was used to (attempt) to unlock motion
//      - change G12 and G13 to single step
//        change ExecuteG2G3PointMove() to not execute until at least 2 spots are open in the motionQ
//      - added GB_STRING for debugging multicycle routines (ie G12 G13)
//////////RELEASED 2019/05/17
// 4.000d
//      - added error throttle per print job... first 25 can go as fast as possible....after than, limit is one per second.
//      - fixed bug with # bytes for CAN_MSG_EXTREMES to V0 heads
//      - started adding code to handle alias to mixture of V0 / V1 heads - does not look promising
//      - fixed case where USING_E_VALUES was not handled during an abort (E_Pulsees nor zeroed out)
//      - fixed bug in V0 heads for setting type/subtype/etc from soap string (assumes ALL heads will get a new soapstring)
//      - changed strings for reading device info to be more terse and less caps to better fit in the small window.
//      - moved >RG: string to after soapstring has been read.
//////////RELEASED 2019/05/18
// 4.000e
//      - changed default behavior for logical devices to be CANBUS_FORMAT_V1 and not CANBUS_FORMAT_V0
//      - added warning when a pre-4.x head registers that logical address may not work
//      - added support for addition subtypes for DIODE lasers (open drain and no cooldown fan)
//      - tweak PICK_N_PLACE to PICKNPLACE to match head definitions
//////////RELEASED 2019/05/19
// 4.000f
//      - changed master.c to autosend seed as >MC along with connection string to auto populaste the seed in the GUI
//      - limit M91 S arg to >= 1.0f; (bounding box scaling)
//      - add check to not write to flash if (validLicenseKey && soap[R1_AREA+0]='*M687"). an error is sent to
//        alert the user
//      - changed M762 to explicity require R=1.0 for talking to new restricted area for firmware key
//      - special case M623 to allow long pulse time for UV Light Rays headtype (which is overloading the LASER code path)
//      - added support for new 3PH_spindle tool.
//      - changed heartbeat led rate to 0.5 bps and abort rate to 5 bps
//      - changed function and name of ERROR_LED to CAN_TX_LED to show canbus RX traffic instead of ERROR conditions to
//      - changed tambora default settings for maxDc from 80 to 100 (per joe & karl) 8amp fuse for customer to install
//      - tweak PICK_N_PLACE to PICKNPLACE to match head definitions
//////////RELEASED 2019/05/21
// 4.000g
//      - changed M98 axis C to enable/disable EMO.  removed EMO control from M96 (backdoor)
//      - added M220 to allow connecting a head to any axis for motion where a CAN_STEP packet will be sent to the head
//        each time the motor steps.  NOTE: the motor does not physically need to be installed, but does need to be
//        properly set up the Machine I/O tab of repetrel
//      - added new deviceType SOAP_DEV_TYPE_GENERIC_HEAD (first use is weathervaning)
//      - added generic head switch control functions for off, on, dc, temp, and pwm M240-M244
//      - change devCodebases Medusa/PickNPlace to Medusa4/PickNPlace4 in headSetup.c
//////////RELEASED 2019/05/23
// 4.000h
//      - fixed bugs with mechanism to store to 2 diff areas of flash, keep one persistent across an erase, have
//        separate read/write control to each, and to prevent users from trying to reflash a firmware key multiple times
//        if they already have a valid key flashed.
//////////RELEASED 2019/05/23
// 4.000i
//      - added init fields for allowable add-ons (probes, am2303, hx711) and allowable PCBs
//      - change M763 to also clear system error flags is ARG_T == 0
//      - change M678 to turn on head 11 AUX for laser cross hair
//      - M240-M245 cleanup
//      - fix AbortTravelScaling calc such that resulting range of a given axis is range*scale, centered on the midpoint
//        of the original axis range.  prior code extended the range in each direction but the amount of scale*range.
//      - reformatted head registration/INF string to better fit in the GUI "Read Params" textbox -- and to match Karl's
//        formatting (newline) option replacing '~' in the string with a newline.
//      - added abort motion call if lost comm with a head AND that head is the current active head
//      - pulled M237 from 2017 attempt at Hydra4 and merged into M245 (switch control)
//      - G2.1 and G3.1 now no longer return to the origin of the move, but rather stop at the end of the archimedes spiral
//        users can then add code to lift Z and move out of the frog toe back to the origin (or where else they need to go)
//        THIS IS A MAJOR CHANGE IN BEHAVIOR
//      - G2.1/G3.1 - added optional ArgR which causes the path of the spiral to be reversed.  A non-printing move is added
//        to get to the new starting point (former end point without R) and the spiral is made backwards with the final
//        position being equal to the start position.
//      - G2/G3 -- if using E, then E = amount for entire arc  if using ArgS to limit # of sides, then E is NOW auto scaled by
//        the ratio of the perimeter length of the inscribed N-sided figure to the circumference of the circle defined by the
//        I/J displacement.
//      - fix issue where very small negative unprime time combined with slow speeds could result in unprimes not being issued.
//////////RELEASED 2019/06/02
// 4.000j
//      - fix bug in head combo checking.  PCB_ALL option was missed.
//////////RELEASED 2019/06/03
// 4.000k
//      - found severe bug in abort code.... if the abort occurred during the motionQ_addCommand call, the 'newest' entry in the
//        queue could be incomplete, because the newest location in the Q was being built live (get a pointer to open spot in the queue
//        and then fill it in.   because of this, a scratchpad piece of memory is now being used to fill in the entry and only once complete
//        will it be copied into the actua queue.   this might clear up a few anomolous aborts that occasionally occur
//      - limit accelerationConstant to a min of 0.01 in M208 to avoid div by 0
//      - added band-aid of a watchdog on an abort to clear everything after 5 seconds ... just in case
//      - dropped the reset of processSoapstringCommands from ResetProcess() as the self init was aborting sometimes if
//        repretrel pinged at the wrong time.  this prevented the license key from being seen as well as other self init
//        code from the soap area
//      - change alert char to repetrel from 8 to 9 (ABORT_CHAR to 911_CHAR --> CATASTROPIC_ERROR_ALERT_CHAR)
//      - change abort code to only send 911_CHAR if self inflicted abort, not a requested abort to a avoid a death
//        spiral of aborts between rep and the MC
//      - limited burst error messages to 3, then once per second after that until reset (was burst of 25)
//      - added gcode line number to all system generated error messages (not head originating messages)
//      - changed self-init code to block incoming RX until the self-init is done.  too many paths wish special chars (ABORT_CHAR) that
//        could step on the processing.
//      - change warning about 3.x heads registering to an ERROR.
//////////RELEASED 2019/06/04
// 4.000l
//      - EXTRUDERS: fixed bug in PCB check in checkForValidDeviceCombination()   PCB_MJ was wrong (excluded wrong board)
//      - fix formatting for line number on motionErrors
//      - fix bug introduced with cleaning up abort code in motionQ_addCommand -- missed propogating "lastInSequence" into the Q if the new
//        move did not get added to the queue, but that move was supposed to be last in sequence
//      - fix bug introduced with cleaning up abort code in motionQ_addCommand -- screwed up start speed limit
//////////RELEASED 2019/06/04
// 4.000m
//      - added option to tracking commands rcvd and acks sent to track down issue where repetrel and the MC are getting out of sync at times
//        counters are reset each GUI Reset; at powerup, and if a control char '6' is sent (ASCII_ACK)  hijacked GB_STRING_HEARTBEAT (0x10) to enable
//        dump of information.
//                  >GB: COM: lineNum: cmds_rcvd: sum_acks_send+pending: acks_sent: acks_pending: charsRcvd: checksum32:
//        the checksum is literally a 32-bit unsigned int summation of each character arriving  (checksum32 += newChar).   in rough terms, about 50M chars
//        can be received before the counter rolls over.
//      - added code sync tge sending of the fake M687 completion code to the systick call as it was happening async to everything
//      - removed _gcodeLineNumber=0 from M30 call.  no more auto changing it...up to user to insert N=xxxx in their code
//////////RELEASED 2019/06/09
// 4.000n
//      - added standard gcode per-line checksum calc qith a "*XX" at the end of line.   all chars before the '*' are part of the checksum.
//        the ascii digits after the '*' are the one byte checksum displayed as a decimal integer in ascii.
//              int cs = 0;
//              for(i = 0; i<parseIndex; i++)
//                  cs = cs ^ cmd[i];
//      - undefined ALLOW_DIRECT_BUFFER_FOR_RASTER_DATA as part of hunt for dropped acks
//      - changed GB_STRING_HEARTBEAT to GB_STRING_COMM_ACK_CHECK (0x10)
//      - re-enabled COLLECT_METRICS #define ..... add a M772 S31 as fist line of a print job... will auto dump all stats on M30.
//////////RELEASED 2019/06/11
// 4.000o
//      - fixed CRITCAL bug in the calculation for when to unprime during the move.  the case for the unpriming occuring during the cruise
//        portion of the move was missing a term (axisScale) and the calc resulted in a pulse count that was outside of the move.  the term
//        was added in AND a safety check was added to ensure the pulsecount was inside of the move.
//////////RELEASED 2019/06/11
// 4.000p
//      - added options to M797 (P and R) to enable echo-ing on serial data to a specified port. select port with P and chars processed to
//        parse ascii commands are echoed (excludes single 'special' char commands.   if port is selected with an R, the ALL chars are echoed
//        as received. ports 0=none/off; 1=usb, 3/4/6 = uart3/4/6;)
//      - changed per-line ascii checksum to use a 2-byte summation instead of the not-so-good) unshifted xor reprap method
//      - added THREE temporary ERROR messages for CAN_E_VALES for negative E movement to help debug unwinding of material
//////////RELEASED 2019/06/13
// 4.000q
//      - added outbut buffer on serial echo path
//////////RELEASED 2019/06/13
// 4.000r
//      - added fakeMcodeCompletion notices for M104 and M106 when a head registers to try to get the GUI in sync;
//      - added fakeMcodeCompletion notices for M670 after boot/reset to try to get the GUI in sync;
//      - added a C option to M703 (add alias).  if C=1, then the selected head's (T) data structure is copied to the structure for the alias
//        this should ease the burden of greating groups.
//      - CRITICAL CHANGE: moved serialProcessor back from loop in main back it's own slice time
//      - added echoBuffer variables to resetSerialOutputBuffer()  {echo failed after a reset}
//////////RELEASED 2019/06/16
// 4.000s
//      - fixed fake M670 return for default DDL behavior after reset
//      - change heartbeart 2bps for case where pending ACKs, 0.5pbs otherwise
//      - move more of foregroup tasks from main() while(1) to slices
//////////RELEASED 2019/06/17
// 4.000t
//      - identical to 's' but with 19.2K baud
//////////RELEASED 2019/06/17
// 4.000u
//      - returned to 38.4K baud
//      - special build with license key debug
//////////RELEASED TO DAVO ONLY 2019/06/17
// 4.000v
//      - move executeMove out of processRawBuffer() when pause ends.flag set to next call to sequenceEngine motion will restart (at most a 1ms delay)
//      - cleanup legacy code in sendChar (took out legacy crap to try to stuff a few extra chars in a full tx queue)
//      - added tracking of tx chars rejected.  will report as last field in GB_STRING 0x10 and with "metrics".  if will NOT report otherwise (so no error msg)
//      - redo sendAck code to make ALL ack's pending and use printCheck() to send acks
//      - ASCII_ACKS are now sent at PrintCheck() time only (if room in serialRX buffer).  if no ACK is sent, then a regular char is sent.
//        pendinAck counter is no longer "protected" with incr/decr as ProcessRawRxBuffer() and PrintCheck() are both called from SysTick.
//      - EMO changes still work in progress -- EMO turned off
//////////RELEASED 2019/06/18
// 4.000w
//      - HOTBEDS:  init table was wrong for chamber heater (was incorrectly set to active low); fix default prescale for chamber in MCode.c
//      - change min/max for all OverridePct to 0 to 15.999
//      - added fake mcode completion notice for M723 to get GUI in sync after comm timeout OR after a reset
//      - add second option for all HOTBEDS to control whether they drive via LIMIT1 or the HTR_HSS
//              HOTBED_100 (Std/Opt0 = LIMIT1; Opt1 = HSS)
//              HOTBED_120, HOTBED_200  (Std/Opt0 = HSS;  Opt1 = LIMIT1)    {LIMIT1 used by engineHR)
//      - cleaned up error message for missing RTD selection on heated heads
//      - clean up naming for default (unnamed) device subtypes
//      - WARNING START BUTTON/EMO/MOTOR FAULTS are DISABLED in the version
//////////RELEASED 2019/06/19
// 4.000x
//      - swap option0/option1 subtype for CO2 laser
//      - change CO2_LASER_LENS check from forced in slot 11 to being the leftmost installed device
//      - added Co2 lasers to "useCooldown" init and change from useDC to UseOn
//      - allow M686/M687/M688 to take extra slice time for security key stuff (only occurs at boot or reset time)
//      - moved up sending home state and m686/m687 completion codes in sendRevisionString() to BEFORE the connection
//        string to avoid issues from Repetrel sending multiple back to back resets/pings
//      - added M238 to allow any axis to copy the E value in a line of GCODE to use as it's position
//      - added option to press physical START button on Hydra control panel to dump all of metrics if GB_STRING 0x10 is enabled (COMM)
//      - reinstated checks on StartButton, EMO, Motor Sensors Fault, L1, and L2
//        more robust noise rejection and a unified routing for all motor sensors
//      - unified naming on HYREL StallSense and HYDRA FaultSense to FaultSense
//////////RELEASED 2019/06/21
// 4.001a
//      - KEYS:  fixed severe bug where some seed data was stepped on causing aliasing of keys.
//      - KEYS:  changed list of key gen from firmwareKey to UUID based
//////////RELEASED 2019/06/24
// 4.001b
//      - fix bug in USB which was introduced in 4.000v
//      - KEY: removed ENABLE_SECURITY (made permanent)
//////////RELEASED 2019/06/24
// 4.001c
//      - added debug options for Karl to allow selecting from a small list of variables that when set will cause a selected light to blink (_gs._flasher)
//      - added code to reset sticky errors each time and M620 E1 is issued
//      - changed minTemp init for LASERS to be threshold for turning on cooling (set to 35C)
//////////RELEASED 2019/06/24
// 4.001d
//      - no behavior change, just code opt for sending EMO message
//      - changed default for HYDRA to disable EMO.  must be enabled with M98 C0 or C1 (active low or active high) ...
//      - additional coding on 3D arc
//      - changed ARC_TOLERANCE from 0.1f to 0.002f per the RS374/NGC Interpreter spec
//      - added G16-G19 to control work plane for G2/G3 arcs (default is PLANE_XY)
//      - added controlWord bit to enable/disable heads from sending strings via canbus (controlled via M719)
//        needed for CO2 laser power supply with UART.
//      - fix bug where a tripped EMO at boot time did not aabort motion
//      - RASTER: Changed M624 to take dots/mm instead of mm_per_dot
//      - RASTER: copied code from original Hydra4 project
//      - RASTER: ALLOW_DIRECT_BUFFER_FOR_RASTER_DATA is STILL UNDEFINED (no RASTER)
//////////RELEASED 2019/07/02
// 4.001e
//      - RASTER: continued debug of code merge
//      - RASTER: added protection so if a '$' (DIRECT_CHAR_START) appears in a comment, it will not trigger
//        a switch to used the direct buffer
//      - RASTER: added protection such that a ';' in a direct char stream will not trigger a comment
//      - RASTER: #define ALLOW_DIRECT_BUFFER_FOR_RASTER_DATA  reinstated for this release.
//////////RELEASED 2019/07/03
// 4.002a --- SYNC RELEASE WITH MEDUSA4.002A  (format change)
//      - change from a 0.16/0.8 to 1.15/1.7 for powerLevelPct
//      - SPINDLE/LASER: changed scaling on powerPct from 0.16 to 1.15 to allow for a true 1.0 to be sent.  coincides
//        with MEDUSA4.002a
//      - increased maxPwmRate to 5000 for spindle tools (default is 2K)
//////////RELEASED 2019/07/03
// 4.002b
//      - RASTER: for bidir scanning, add the DEF startAdjust value to user input instead of replacing the def with user
//        input.
//      - fix issue in SetupLasetToRun for local control (1) power to head 41 set to 0 (2) vector power was incorrectly used
//        to setup CCR for timer instead of requested power (input to method)
//      - change default for pierce power to be vector power (M621) when argD is omitted
//      - fixed bug with pierce power and local control
//////////RELEASED 2019/07/12
// 4.002c
//      - added 2 more "flasher" variable (M797 Vx) for debug (_gcodePaused and BlockAllMotion)
//      - changed "EMO Released" to clear the BlockAllMotion flag (still requires homing, but will all relative moves and NOT
//        require a "RESET" (from GUI)
//      - fix bug in reporting of subtype for HOTBED120/200 (inadvertantly keyed off deviceType instead of deviceSubtype)
//      - added small dwell (50ms) between trying to add commands to motion Q when motion is blocked to prevent repetrel from
//        becoming non-responsive (lines are being sent and acked fast and "clicks" (ie, Kill) get buffered in the GUI but
//        were not sent to the hardward --- THIS WOULD LOOK LIKE A MOTION CONTROLLER HANG, when in fact it was the GUI.
//      - added flag ("homingFailed" for when pop off fails to make sure "axisHomed" does not get set if this pop off failed
//      - added extra checks in abort when safe to flag if 'curr' pointer was getting out of range
//      - fixed nagging bug with arc commands sometimes generating a bad move.   two-fold issue (though one may only have been an
//        issue because of the other.    in seq eng, there was a return for "geG2Flag > 0" but should have also checked for G203State
//        being non zero.     in execcuteG2G3PointMove, ExecutionPtr was not reloaded in the case of G2G3Flag == 0.
//      - changed G2.1 to calc finish position of the sprial and use that as last point instead of skipping move back to start
//      - revert change executeG2G3PointMove to exit on motionQ_full (instead of almostFull)
//////////RELEASED 2019/07/29
// 4.002d
//      - updated new ExecuteArcPointMove to match changes made in 4.002c for multi move commands
//      - added more variables to crashLog and to value to map to DDL  (for Kill/Reset lockup issue
//      - changed loading to urgent buffer take priority over loading direct buffer (raster)
//      - clear M238 (M->SubstituteAxisArgWithArgE) on kill/reset
//      - G38 added Oarg to specify an addition offset for toolOffset (H=Z+O)
//////////RELEASED 2019/08/06
// 4.002e
//      - fixed cut and paste error with G38 change.
//////////RELEASED 2019/08/06
// 4.002f
//      - changed info message for G38 setting toolIndex H value
//      - clean up error messages originating from heads
//      - added M797 I1 option to dump state of certain I/O (all motor IO)
//      - force 5 second wait when enabling CO2 power supply to give time for water pressure to build. (M620)
//      - recovered 3 head control word bits (ignoreRTDx) ..  ignoreRTD3 repuposed as debugRTD flag going to heads
//      - retired M710/711/712 (ignoreRTDx control).   M712 converted to code to set debugRTD flag (M712 S1)
//      - replaced raster format with new one relying on simple binary number of bits per pixel;
//      - added color index table (M626) for translating raster data
//      - added gamma option for modifying raster data to tube (similar to crt monitor gamma function)
//        newLaserPower = oldLaserPower ^ (1.0/gammaVal);   (laserPower values in range of 0.0 to 1.0
//      - added optimization for laser raster data to non print (or move over) '0' values on left and right
//        sides of an image.
//////////RELEASED 2019/08/08
// 4.002g
//      - raster - added imageInvertData option
//      - fixed typo in M245 -- "onWhenExtruding" referenced wrong arg; also in M245 fixed wrong "area" passed to devInit method
//      - fixed bug in decision op in canWriteSwitchFlag_onOnlyWhenExtruding that would result in the flag being set in
//        wrong switch
//////////RELEASED 2019/08/09
// 4.002h
//      - more raster cleanup for optimizations for non-printing leading and trailing portions of a scan line
//      - started mucking with USB code (flow control in USB lib for tx data (hydra to PC) is non-existant (and hydra code called
//        the one and only (but wrong) method for flow control)
//      - MAJOR overhaul of the USB code.  removed all of TM's code and redid part of stm_lib's to remove
//        redundant data buffering and to add working flow control.  this code is less generic than it was (strictly
//        a FULL SPEED, Virtual Comm Port.
//      - remaining issue with this is REPETREL needs to respond to a 0x7 (PING/ANNOUNCE CHAR) with a PING in order to set the comm
//        master at boot/hw_reset time
//      - the new implementation has a different sw burden (ReceiveChar and ProcessRaxRx have a more bursty load, though total CPU load is
//        reduced
//////////RELEASED 2019/08/18
// 4.002i
//      - changed "ANNOUNCE CHAR" (HELLO_WORLD) from 0x7 to 0x4
//      - -added lathe mode for A/B axis using M3/4/5
//      - switched around all motor timers so that TIM9 could be used for direct output for STEP from TIM9_CH2 for lathe mode
//      - limited lathe mode to axis A for HYREL and axis B for Hydra (only available STEP pins connected to a useable timer
//      - aded PULLUPs in pin def fro CANx_RX pins to all bench testing of just ram 407/429 parts and be able to look at the CAN_TX line
//        (which will get killed in CAN_RX ever sees a 0 when CAN_TX is driving a 1.
//      - slight cleanup of CAN_Config()  [no functional change]
//      - added M633 Ex to enable/disable the use of CAN2 -- power on default is enabled to match prior behavior.
//////////RELEASED 2019/08/22
// 4.002j
//      - added SOFTWARE_DEBUG_REVISION to allow embedding a revision for 'z' version.   M115 can be used to send out the value in the interface window
//      - when lathe mode switched from ISR to direct step created from timer, the ability to send the step over the canbus for this mode
//        was lost...but the mode change to the head was still sent.  this mode change init to the can device has been removed.
//      - change lathe mode to allow change of direction without having user stop spindle first
//      - add separate max PPS check for timers for raster mode (can be set with an M212 Rxx
//      - change max time for M623 for a UVATA (UVLIGHT_RAY) head from 10 seconds to 60 seconds.
//      - change HYREL build target to not enable CAN2 (new 100094 rev 9 board is CAN1 only)
//      - removed M633 now that CAN2 issue is resolved on the rev9 100094 board (was added for ease of debug)
//      - change M106 Cxxx (_M106_fanDutyCycleRange) to reset to 100 on a job kill.  was reset after an M30, but missed on the kill.
//////////RELEASED 2019/08/28
// 4.003a
//      - added support for DEVICE_TARGET_MEDUSA_407
//      - changed pageDef reporting for medusa 4.003 or greater to only report page or sector for the soapstring
//      - complete remapping of bit positions in pin defs to allow use of bitfields (adopting Medusa methods) to make it cleaner
//        to support a dual 103/407 ... now allows sharing of the gpio code between the two projects.
//      - added support for two addition rtd types (4-20ma and adcRaw).
// 4.003b
//      - change all TEMP controlled heads to SLAM ON.
//////////RELEASED 2019/09/26
// 4.003c
//      - added RH_SYRINGE head type (refrigerated & heated syringe head)
//      - added repetrel comm watchdog, but did not enable head reset until time to test
//////////RELEASED 2019/10/16
// 4.003d
//      - reinstated call to initKey(FALSE) in main() that was inadvertantly deleted in version 4.002h with the net result of
//        keeping key generators from working.
//      - change LARGEST_SPECIAL_CHAR to 31 (0x1f) to provide headroom for future use
//      - fix UVLIGHT_RAY setup info (maxPwmFree was 0 instead of 10000)
//      - added software based millisecond counter for debug
//      - fixed bug in MotionIsNotComplete() (now renamed to AnyPotentialMotion()) .... which was used by the Homing sequence
//        to know to move on the next step.  Needed to also check if a the start of a move was deferred by N milliseconds for either
//        first move in the queue or for enabling the MOTOR_ENABLE
//      - changed shortcut for the first mQ entery aging to not abort the wait for longer waits (such as the MOTOR_EN wait)
//////////RELEASED 2019/10/24
// 4.003e
//      - added M800 for sonicator control
//      - added sonicator head type
//      - moved call to initInboxStruct() to after sending reset in checkForMia()
//      - added watchdog value to packet sent to extruders to run so that cases where laser_family (ie UVATA) devices are cloned
//        to an extruder.
//////////RELEASED 2019/11/11
// 4.004a
//      - fixed two bug in "pause at end of next move" code. (1) failed to properly insert a "prime" command when motion
//        resumed (wrong pointer in queue was referenced) and (2) _requestToPauseAtEndOfMove was not set to FALSE immediately
//        after the motionQ was modifed for the pause (and that method was called multiple times).
//      - fixed offset by 1 issue with prescale count for sending heartbeat status message back to the host (tied to M719)
//      - added per axis semaphores for L1/L2 sensors to block neg/pos relative moves (in addition to absolute moves) if sensor is tripped
//      - increase motionQ size from 15 to 30 (faster arc speed for G2/G3)
//      - reworked compiler targets based on new naming scheme
//      - fixed bug in checkMotorSensor() (typo '=' instead of '==' in check for motor 'C" and L1 sensor (highjacked for EMO)
//      - increased NORMAL_RX_HEADROOM from 1K to 2K
//      - major rework of sensor handling
//      - major rework of motionQ_addCommand relatively to blocked moves (abs, rel, etc)
//      - revamped M780/PO position reporting
//      - added more control to M719 to allow XYZ, tiem, etc to be printed along with status reporting
//      - changed M791 and friedns (M710) to use reriod instead of rate
//////////RELEASED 2019/11/20
// 4.004b
//      - fixed bug in check for blocked absolute motion
//      - added more error messages for when motion is attempted during different blocked conditions
//      - add JOGGING speed (set with M234) and added a G0.1 move to to a RELATIVE move at the jogging rate
//        laying groundwork for smoother motion with the job wheel
//////////RELEASED 2019/11/21
// 4.004c
//      - added Gxx code for cylindrical surface build using a rotary axis
//      - improved M235
//      - fixed bug in G928 last release force it to be an incremental move, but failed to set the Q_LastRequestedPositionInUnits variable.
//        (prior code assumed it was operating in absulute mode, even though G91 could have been in effect).
//      - fix bug in Mcode/Gcode generic error reporting methods... they were relying on _errorStr, which should
//        not have been the case as the calling routine may have also used _errorStr. changed to a local string.
//////////RELEASED 2019/11/26
// 4.004d
//      - expanded and renamed the cylinder move to karl's choice of G702 (CW) and G703 (CCW)
//      - added argC to M797 for a submask (_sendingGBStringsSubMask)
//      - changed ROTARY code to allow infinite distance, but keep view of position to code/world as 0 to 360
//      - changed clamp and warn code to check rotary if MAX_TRAVEL is not 0
//      - fix bug in M6 that would have screwed up the position if a tool change had occurred during a G91 mode (incremental)
//      - fix bug with 30M version that references FAULT_SENSOR.
//      - initial code to re-instate CAN_EVENT_MANUAL_Z_MOVE control from hotbeds
//////////RELEASED 2019/12/01
// 4.004e
//      - fix bug in unitVector assignment in motionQ_addCommand for ROTARY that was causing a speed limit error
//      - adding separate mechanism for jogging.  relies on recently added G0.1 and Jogging rate and CAN_EVENT_MANUAL_Z_MOVE.
//      - fix typo in M719 for setting which axis to report (had an X in one place that should have been a Y)
//      - added argP to M785 to allow controlling the period for updates of the manual switch on the head.
//      - added M224/M225/M226/M796 to facilitate new jogging capability
//      - changed acceleration calc for coordinated moves to just calc next move time based on accel/decel rather than trying
//        to limit change to once per millisecond (effectively makes DominantAxis timer a one shot when accelerating/deceleration
//        and a constant interval timer when at cruise speed.   removed SpeedControl();
//      - fixed long existing bug that would lock up the motion controller.  occurred in the uart ISRs.... if an overrun was detected
//        (UARTx->SR.ORE_FLAG set).  to clear it, a read of SR followed by a read of DR was needed.  the DR read was not in the code, so
//        stuck in a loop of servicing the ISR for uart over and over...
//      - misc rotary cleanup
//////////RELEASED 2019/12/08
// 4.004f
//      - change EMO released message from an ERROR to INFO
//      - increased wait for high voltage to 3 seconds
//      - fixed bug in accel code ... 4.004e moved next accel calc to bottom of TIM1 ISR, which caused a delay in setting up the timer for
//        the next TIM1 pulse.   restored call to the top of the TIM1 ISR, but kept the change for the CALC of the next timer values that
//        the bottom to greatly speed up the time to get the timer set up for the next pulse
//      - created UnitsPerPulse value in each motor structure and got rid of ALL divides of PulsesPerUnit and replace with mult of UnitsPerPulse
//        (each replacement saves 13 cycles).... several of which were in time critical places
//      - fix bug in cylinder 90 degree (perpendicular case ... perpOfs move did not occur)
////////////RELEASED 2019/12/10
// 4.004g
//      - reduced wait for high voltage after emo released to 1 second.   Z_STEP issue resolved with added 10K pullup on board, so this should be ok.
//      - gave M718 new life as a sync mechanism for repetrel
//      - change 'a' to 'V' for actual vector velocity.
////////////RELEASED 2019/12/12
// 4.004h
//      - redo M719 and M780 reporting (changes for F, E, V, and adding Line number reporting)
////////////RELEASED 2019/12/12
// 4.004i
//      - added more debug info to error report if E rate exceeds dominant axis rate.
//      - tweaked ISR priority for "probe" commands
//      - fix bug where the outbox canbusFormat was not set to CANBUS_FORMAT_V1 by default for all aliases
//      - fixed issue with using checksum for "urgent" commands (the urgent char was part of repetrel checksum, but was stripped off
//        before the serial processors calculated the checksum
//      - changed catastrophic failure for active head being lost to only reset if active head lost and Que's not empty
//      - reduced head comm watchdog from 15 to 10 seconds
//      - added code to allow users to select Tx mapping for standalone Tx command AND mapping for Tx arguments on mcodes -- ifdef'd OUT
//        (code is not tested -- ON HOLD PENDING DECISION FROM KARL)
////////////RELEASED 2020/01/08
// 4.004j
//      - move call to NVIC_PriorityGroupConfig() in main() up to prior to all the hw init calls to get around a bug in the st libs
//        for setting the interrupt priority
//      - change USB code to no longer change the split the preemptive/sub priority bits diff from the programs selection (was bad form
//        of the USB driver to do this)
////////////RELEASED 2020/01/22
// 4.004k
//      - moved irq disable/enable code in to inline function calls to ensure uniform code for all off/on and to allow optional
//        debug code to be uniformly added
//      - tweaked print format for SendCurrentMcodeExecutionNotice() to use %d instead of %3d, etc
//      - moved UARTs to Preemption Priority from 1 to 0 and IWDG from 0 to 1 when checking xtal speed
//      - revamp GB_DEBUG_PINs to use newly wired PANEL interface
//      - fix bug in NVIC_Init that did not handle all the legal states of the SCB->AIRCR reg (including it's default reset state!).  this
//        was the root cause of Serial Overrun errors, as the UART did not have highest priority as intended
//      - changed way deferred commands are processed.  now instead of immediately processing all deferred linked to a just-finished move
//        in processMotion, they are processed once per call to SequenceEngine() (NEW_DEFERRED)
//      - changed way the next move is called after the last pulse is issued for a move.  instead of in processMotion, it is called in
//        in SequenceEngine() (NEW_DEFERRED)
//      - started adding code to support scripting (ALLOW_GCODE_SCRIPTS)
//      - started inserting code for AB-select encoder (NEW_4_LAYER_PCB)
//      - added check for a gcode arg with just a letter and no number.
//      - fixed bug in setting up start speed for moves that "prime" .... was not staying at noRamp speed and could jump off too fast
//      - removed some very old "NUKE" code and many unused sections of #ifdef'd out debug code
//      - added compile time code option (MEASURE_TIME_SLIPPAGE) to measure real time vs systck time to see if there is any time slippage
//        in the systick control loop (slices taking too long, etc)
//      - added call to XXXXX in Timer init routines to limit interrupt source to over/underflow and DMA
//      - some more cleanup of crufty pin defs for DEBUG and compatibility between build targets (MEASURE_TIME_SLIPPAGE)
////////////RELEASED 2020/01/24
// 4.100a
//      - rebranded 4.004k to signify semi-major release (improved serial port comm)
////////////RELEASED 2020/01/24
// 4.100b
//      - undo part of the NEW_DEFERRED.  instead of delaying move end to next move to a call to commandProcessor from ProcessMotion
//        revert to pre-4.004k behavior of just immediately starting the next move.  however, still retain the change in 4.004k of
//        pushing deferred mcode processing through commandProcessor
////////////RELEASED 2020/01/25
// 4.100c
//      - enabled sw reset of all heads if comm with repetrel (or any host) is lost for than 5 minutes)
//      - changed a few of the messages that occur during an abort from sendError() to sendInfo()
//      - added 2 routines to convert from toolNumber to device and vice versa
//      - slight tweak to _gs._errorCount (error throttling)... still limits to once a second when bursting, but if no errors for a while
//        will allow more than one error
//      - split NEW_4_LAYER_PCB into two ifdefs (USE_HYDRA_IO):  USE_AB_ENCODER and USE_6_TO_ONE_SELECT
//      - making USE_AB_ENCODER the default  -- WARNING -- this moves the CO2 direct laser control from TIM5/PA1 to TO TIM2/PA3
//      - added code to throttle speed based on total time needed to service the ISR routines for motion (LIMIT_SPEED_DUE_TO_ISR_LIMITS)
//      - reworked some of the I/O routines to add a path for "fast" I/O access (GPIOx and bit mask are stored in ram at init time)
//        used primarily for the STEP/DIR, sensors, and HSS (and debug pins).  saves close to 0.5usec per I/O access (USE_FAST_IO)
//      - removed all min/max related macros.  found issue if an arg to the min/max was a calculation, the calc was repeated multiple
//        times (once for the "if" and once for the "assign").  replaced all with corresponding math func calls (fminf/fmaxf) or user
//        create imin/minx and iFitWithinRange, fFitWithinRangth.  In one example, a range check in the accel calc resulted in the
//        sqrtf function being called 4 times instead of just once.
//      - reinstated balanced math calls in TIM1 to minimize jitter of the dom axis pulse (saves about 2usec of jitter) (BALANCED_TIME)
//      - simplified the arr/psc calc in several routines (NEW_TIMER_ARR_PSC_CALC)
//      - reordered ISR priority to accomodate using TIM7 as part of the motion timers.  changed priority assignments to a set
//        of #defines to allow easier tweaking in the future
//      - moved accel/timer calc for Dominant timer (TIM1) ouuside of the TIM1 ISR to a lower priority ISR (using TIM7 to kick
//        off calc....  this removes some jitter in the step positioning (USE_TIM7_FOR_ACCEL_ISR)
//      - changed default value for CentripetalAccelRadius from 0.02 to 0.005 (user changeable with M222)
//      - added compiler option  -ffast-math
//      - added compiler option: -no-strict-aliasing
//      - added replacement for sqrtf() -- fpu_sqrtf() that takes advantage of the FPU's built in 32-bit sqrt.f32 function (14 cycles)
//        vs 336 cycles in the software math.o reoutine for sqrtf.  (tried a bunch of things to get the compiler/lib to use the
//        FPU instruction to no avail. (USE_FPU_BASED_SQRTF)
//      - started  native Lightburn mode.    (to enable:   power-on or HW reset + "run" button held in for 5 seconds)
//        must compile with ALLOW_NATIVE_LIGHTBURN.   changes to code in the mode include:
//          - per line ack is "ok\n"
//          - M106 Sx to set vector power {auto convert to M621)
//          - all G1's have an implied E1 (auto add E1)
//          - some G0's have argF=0 (auto remove arg F)
//          - M620 has to be auto inserted for CO2???? (Use one of the 6 macros for this)
//          - disabled all sendInfo/etc only directed msgs (sendstring) and sendErr messages will go out
//        enabled under GB_DEBUGGING for now (#define ALLOW_NATIVE_LIGHTBURN)
//      - modifed code to fully pre-load the can packet for generating a can step pulse in StartMove rather than modifying for device/dir
//        in ProcessMotion  (PRELOAD_CAN_STEP_PACKET)
//      - modified code to preload laser device in pre canned laser canbus packet (PRELOAD_LASER_PACKET)
//      - added pointers for first and last motors involved in a move to reduce processing time in the TIM1 isr (shorten for loop)
//        (FirstAxisMotorPtrInvolvedInMove and LastAxisMotorPtrInvolvedInMove).  update in motionQ_exectuteMOve() and joggingUpdatePulses()
//      - increased debounce mask on "Fault" sensor to try to cut down on spurious triggers when EMO is released (and not monitoring EMO)
//        downside is greater time to detect an actual fault
//      - cleaned up the code for keeping the homeSensor state up to date when not homing
////////////RELEASED 2020/02/02
// 4.100d
//      - added more motorStructure variables to the reset list in initGCodeSequenceEngine()
//      - modified printout for GB_STRING_FLOW to aid debug of flakey "slow motion" issue
////////////RELEASED 2020/02/05
// 4.100e
//      - swapped out FirstAxisMotorPtrInvolvedInMove/LastAxisMotorPtrInvolvedInMove for an array of motor pointers involved in the move
//        which improves upon the prior method as only moving axis are in the list (say MY and MC, where as the prior would have still had
//        all but MX.  s_movingAxesPtrs and _numMovingAxes.
//      - made USE_FPU_BASED_SQRTF permanent and replace ALL sqrtf usage in the codebase (instead of just inside of calcVelocity() method)
//      - added sendstringCr(">ER:  0: Motion Error"); in front of sending a motion error as Repetrel was not turning on the Error flag with
//        the normal motion error message
//      - added Direction bit to "USE_FAST_IO" list
//      - removed restriction on axis C to have fewer pulses than the dominant axis.
//      - added to isr throttling to include M_C when using canbus step -- both for step count AND for canbus utilization (limited to 75% usage)
//      - scrubbed motionQ_execute() to minimize time spent in this routine (critical timing for inter move timing.  pushed all (non-head-based)
//        flow scaling to motionQ_addCommand() (non-real-time).
//      - changed INKJET code to allow use of M_C for sdrop by drop control via can to truly match motion (with M229 to enable) instead of just
//        setting a rate on the head (should provide for better cornering)
//      - removed (made permanent): USE_TIM7_FOR_ACCEL_ISR, NEW_TIMER_ARR_PSC_CALC, LIMIT_SPEED_DUE_TO_ISR_LIMITS, PRELOAD_CAN_STEP_PACKET,
//        PRELOAD_LASER_PACKET, USE_FAST_IO
//      - added M716, M717 for controlling data logging on the aux serial port
//      - fixed bug affecting backdoor Zjog and lathe modes.   error in timer calc introduced with NEW_TIMER_ARR_PSC_CALC in in 4.100c
//      - added USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS but left turned off.   enable to change soapstring delimeter from ';'
//        to another char ('?') and then auto strip ALL comments before hitting the rx buffer.
//      - move called isAutoPrimeoOrUnprimeNeeded later in motionQ_AddCommand. durng code opt, it inadvertantly was moved ahead of
//        the point in which the moveTime was known (and it relies on move time)
//      - converted several common small motionQ status methods to inline macros (ie, motionQ_empty())
//      - added M219 to allow changing the baudrate of the UARTs
//      - added G2,2.G3,2 commands to be a better behaved version of G2.1/G3.1 (actually end at the spec'd XY ... unlike the kludge added in 4.001i
//        this allows for an inductor build in 3 lines of gcode with no Z lift.
////////////RELEASED 2020/02/10
// 4.100f
//      - in last release, moving isAutoPrimeoOrUnprimeNeeded() left hadAnE unset at the time it was checked for TurboMode.  Added code
//        to SetFlowInfo to set hadAnE, so now it's set before it's used
//      - added additional option to reversing in G2.1/G3.1 currently, R1 does an inward jump, then spirals out to the currnent position.   added R2
//        which goes from current position and works out from there
////////////RELEASED 2020/02/10
// 4.100g
//      - changed M632 (PnP commands) to not block motion unless a new arg, ARG_W==1
//      - modified NEW_DREFERRED as follows:
//          - no longer insert a break in motion (slow down/speed up) {and this did not unprime/prime, but did put a break in motion)
//          - next move is started immediately after prior move completes, rather than after deferred commands complete.
//          - the deferred commands are posted and will be serviced with priority with each subsequent all to SequenceEngine()
//      - change M221/756/757/758/230/231/232 to IMPACT_DESTINATION_CALC ... the result of these is used by motionQ_addCommand (had been used
//        in motionQ_execute() prior to Hyrda4.100e.   so this was an oversight to have left them as SYNCS_WITH_MOTION
//      - change M625/628/630 to SINGLE_STEP... these must complete before the next move starts, which can't be guaranteed with the
//        the NEW_DEFERRED path where the deferred mcodes are processed some time (1ms or so) after the next move starts (and therefore can not
//        apply to the flow calcs, probe arming, etc)
//      - change M233 to SINGLE_STEP. this had been incorrectly set to SYNCS_WITH_MOTION
//      - changed default value for CentripetalAccelRadius 0.010 (user changeable with M222) -- this had been 0.020 for all of time
//        except for 4.100c to now where it was dropped to 0.005
////////////RELEASED 2020/02/22
// 4.101a
//      - unify use of ARG_x in GCode.c (replaced legacy access via ExecutionPtr->x)
//      - remove unused _skipFinalArcMove variable (always FALSE) and associate unused code
//      - unified "common" with Medusa4 after adding SPI display support
//      - added spi display to Hydra with first few test screens
//      - added M246 to support display control of remote head and control local display
//      - added basic use of AB encoder to select display page
////////////RELEASED 2020/03/08
// 4.101b
//      - fix bug in 5th entry of CO2 laser heartbeat reporting (had >> 1 instead of >> 13)
//      - move change for motor direction lower in motionQ_execute to increase hold time relative to last step
//      = added two additonal options to M797 to allow booking of X steps after each layer.  (REMOVE WHEN FINISHED)
//      - added MOVE_DIRECTION_CHANGE to change where in motionQ_execute() the change of direction bits occur.  this is in an
//        attempt to increase hold time from the last step of the prior move.
////////////RELEASED 2020/03/24
// 4.101c
//      - made MOVE_DIRECTION_CHANGE from 4.101b permanent (removed #ifdefs).  change added enough hold time from  last step to dir change.
//      - removed debug code added in 4.101b for "X-shift" issue (added two additonal options to M797 to allow booking of X steps after each layer)
//      - made delayMs and delayUsec always exist and not just available when ADD_SPI_DISPLAY was defined
//      - added M73 to be compatible with Prusa Slic3r's time remaining and % complete report.  forwards info to repetrel and also sends a GB string
//      - added M627 to allow setting of a relative and absolute destination to occur on a job kill (much faster response than repetrel trying to
//        accomplish the same thing, which is handcuffed waiting for the full abort/reset sequence to finish
//      - change G38 to allow pin selection for the probe connection on the head
//      - change headSetup for PnP to not allow contact probe
//      - fix bug in GUI code for changing pages with the AB encoder.   accidently put the DrawPage call in a ISR based  routine instead of the foreground process
////////////ESCAPED FROM QUARANTINE - 2020/04/06
// 4.101d
//      - cleaned up the ability to seed a group/alias with the settings from a given head so that it is much easier to create a new group can then contain
//        a set of heads and still be able to individually talk to each head (no "master, so no worries about affecting other heads), while being able to
//        talk to all of them via the group identifier (51-59, 50 is all groups with any %x group).   to "copy" settings to a group, use M703 Tx Sx C1 (which
//        will copy physical head Tx's current settings to group Sx.
//      - update registration code to send out fake completion notices when a head registers to try to get the GUI to match reality (Repetrel really should
//        reset the gui for each device to "unused" when it sees the >RM notice
//      - added code throughout to better attempt to keep repetrel in sync.  still work to go (see Repetrel list below)
//      - changed SendCurrentMcodeExecutionNotice() to handle aliases for the device .... in cases where it's an alias, multiple notices are
//        sent, one for each head matching the alias.
//      - add option to M106 to set the Range of numbers in a non-persistent fashion.  this is meant to be used by repetrel, so the GUI
//        can always operation in 0-100 range, independent of the range set by the used with the persistent M106 Cxxx setting
////////////RELEASED - 2020/04/09
// 4.101e
//      - fix issue with shutting off GUI for manually running the extruder when starting a print job
////////////RELEASED - 2020/04/09
// 4.101f
//      - fixed bug where variables use by M627 should have been initialized to INVALID_ARG_VALUE instead of 0.0f
//      - fixed bug where _hijackAxisC was not reset to FALSE after a job kill
//      - force "USE_E_VALUES" related variables to "off" after an M30 (not persistant across jobs)
//        this has likely caused grief for a while for anyone mixing print jobs with and with using E values
//        and jobs that had no M229 would inherit the prior jobs state.  if it was a kill job between jobs,
//        the only part of state of the prior M229 was inherited (due to prior bug)
////////////RELEASED - 2020/04/10
// 4.101g
//      - re-sync "common" after changes to Medusa for the new 405 board.
//      - update key generator list to move away from 407 board to use the devebox mcudev board (4 made).  obsoleted prior key generators
////////////LIMITED RELEASED - 2020/04/27 (only released in new key generators
// 4.101h
//      - tweaked calc for "filament used" in M773 to use volumetric calc instead of linear off a guess of the hobbed diam.  makes assumption
//        for 1.75 fillament and that the head is programmed with volumetric pulses/unit
//      - no longer send Ex in the SendCurrentMcodeExecutionNotice() for an M620 [GUI will key off S]
//      - added SendCurrentMcodeExecutionNotice() to M678 (laser cross hair)
//      - update tryToCleanUpGuiAfterDeviceRegistration() for M678
//      - G28 - have Ix rapid move apply to all axis specified, not just those with home sensors
//      - G38 fix bug where feedrate for move (ARG_F) was ignored
//      - minor cleanup (mostly comments) for G2/G3 family
//      - fixed bug in G4 where Sarg was taken as an integer number of seconds only instead of float (ie, 3.5 seconds)
////////////RELEASED - 2020/05/02
// 4.200a
//      - cleanup before fork to 5.0 (remove NUKE, CAN_V0)
//      - remove NEW_DEFERRED
//      - re-expand tabs
//////////// RELEASED - 2020/05/03
// 4.200b
//      - add clone tracking for each registered device in order to know whether an different" head type is cloned to the active head (such as
//        a UV_LIGHT_RAY) and if so, to send controls (like watchdogs) to that device.
//        function updateCloneList()called in RETURN_DATA for USER_ALIAS list,
//        added code in CheckForNewFlowRate() to check in a UVATA is cloned to the current selected head and if so, send out a LaserControl packet
//      - setupHssPwm() was missing processing for ARG_P  (same processing from M619)
//      - modified M797 Ix output for showing state information
//      - redid tab expansion as some print strings ended up with tabs.
//      - output deasserted Step pin when polarity is set via M97
//      - output FORWARD direction on Dir pin when polarity is set via M94
//////////// RELEASED - 2020/05/08
// 4.200c (mimic a few things added to Hydra5.000a
//      - redid tab expansion as some print strings ended up with tabs.
//      - created M253/M254/M255 for lathe mode and put in "legacy" warning for M3/M4/M5 (warning in V5 only)
//      - cleaned up M773-M776, M779 reports to give a common look
//      - added check for I==1 to enable move to G28 intermediate location
//      - move M246 to M260 (gui)
//      - added M702 for setting up group (seeded from a physical device)
//////////// RELEASED - 2020/05/20
// 4.200d
//      - redid tab expansion
//////////// RELEASED - 2020/05/20
// 4.200e
//      - fixed bug in getMotionRateInUPS() which did not return the correct rate when in raster mode
//		- fixed bug in initiializing the JOGGING rate in motionQ_addCommand to MAX_FLOAT (was wrongly set to the rasterRate)
//		- fix issue where last segment in a spiral arc lost it's E value
//		- changed inkjet code to ensure at least one dot is output on small views (when dist * dots/mm would have been < 1)
//		- fixed M229 argP ... when it was dumbed down to not accept non-AUTO choices, broke the ability to turn it off. restored.
//		- make sure M723 only works with extruders to ensure to accidental turn on of laser/uvata heads
//		- added option to test using the AB encoder to track filament.... using M227 to set the non-jogging behavior of the encoder
//		  to either be GUI or free-running+reporting as T16 in the RT> report
//////////// RELEASED - 2020/07/28
// 4.200f
//      - change raster code to send raster value to laser on process of first step of the dot instead of while processing
//		  the last step of the prior dot (which had been done to try to cancel some of the canbus time).  not good for direct
//		  drive or for dealing with speed related variability with the bidir option
//		- minimized "extra" front porch pixels for raster (had padded with 10 dots ... way too many when running slow)
//		- flipped polarity on bidir adjust. negative now moves to 0
//		- increased max allowable rate for diode lasers to 30000
//////////// RELEASED - 2020/08/04
// 4.200g
//		- re-wrote usb_lib processing for raw chartio input.   now better behaved and is processed in a slice rather an in the usb_libs ISR.
//		- changed INKJET pwm rate to coincide with raster scan rate and image_dpmmX
//		- using CAN_MSG_STEP for inkjet raster processing since no grayscale exists for the inkjets (fixed dot size)
//////////// RELEASED - 2020/08/05
// 4.200h
//		- changed inkjet raster to only send can packet on "spew" packet instead of at dot rate.
//		- major rework of raster step process/ISR code.
//		- send inkjet raster "spew" at center subdot-step instead of start subdot-step
//		- added 100 ohm PRTD option for heads
//////////// RELEASED - 2020/08/14
// 4.200i
//		- enabled CCM SRAM for use with data and moved a bunch of data there to free up space in the main SRAM
//		- fixed warning on M5 checking for legacy use
//////////// RELEASED - 2020/10/05
// 4.200j
//		- added PNP REEL ADVANCE controls to M632
//////////// RELEASED - 2020/10/24
// 4.200k
//		- remove CCRAM ... linker issue ... builds a huge .bin that then fails to convert to a .dfu for the usb bootloader
//////////// RELEASED - 2020/11/04
// 4.200l
//		- merged in changes from "common" (changed hard coded delay countdown value in for flashing LEDs to a #def (from
//		  medusa inkjet change to show spewing ink in raster mode
//		- fix bug where DominantAxisPtr was not initialized at POR time and was inadvertently accessed in an
//		  unintended call to TIM7 ISR prior to when it would have normally been set to the correct axis.   This
//		  former was a mistake when setting things up for the CCRAM. The latter was the result of not checking
//		  a flag in the ISR.     THE RESULT OF THE BUGS was that the FLASH controller threw an error which was
//		  not cleared and resulted in blocking any soapstring writes (system soap, which included license keys)
//////////// RELEASED - 2020/12/07
// 4.200mz
//		- fixed case where lastRequestedQPoisition was set wrong if poistion has been clamped due to exceding print limits (V5-done)
//		- reset TurboModeDistance=MAXFLOAT after an M30 (V5-done)
//		- add check for (_MailBoxes._inbox[i].deviceFamily == DEVICE_FAMILY_HEATED_EXTRUDER)) in waitingForExtruderTemp check so it
//		  will not check for non-heater extruders (V5 TODO)
//		- initial pass at adding canMotor head type -- PERVASIVE change.
// 4.200nz
//		- fixed (along with medusa 4.050hz and is with L1/L2 state getting hosed across a job kill.
//		- tweaked M627 to clamp the relative move amounts to not exceed the printing volume (to avoid needless error message) (V5 TODO)
//////////// RELEASED - 2020/12/28
// 4.200oz
//		- putting in can_motor and new 405 based force feedback head into medusa project for real.
//		- reverting "force" control from mA back to % (M785 Cxxx is now pct 0 to 100)
//		- M787 now requires a C1 to calibrate and/or an F1 to restore factory defaults.  both can be issued at the same time if desired
//        no full release
// 4.201a
//		- cleanup for canMotor and new 405 board
//		- support NEW PCB list/order
//		- pass-thru "Debug" revision from head and display with showParams
//////////// RELEASED - 2021/01/09
// 4.201b
//		- remove remote HOME sensors option for CanMotors (now need #define USE_CAN_MOTOR_HOME_SENSOR)
//		- added extra device event so closed loop steppers can easily ping the MC during calibration
//////////// RELEASED - 2021/01/11
// 4.201c
//		- fixed bug in move for AbortRelativePosition that was introduced in 4.200nz
//		- set AbortRelativePosition/AbortAbsolutePosition to INVALID_ARG_VALUE after a job end or job kill
//////////// RELEASED - 2021/01/12
// 4.201d
//		- blocked canMotor update of sensor state in HydraCan.c (EVENT reporting) (forgot to remove this in 4.201b)
//		- fix bug adding in 4.200m which caused an illegal access to flash (tied to adding flags to the init struct for a head) for
//		  knowing if a head had a heater.   removed bad code that was inadvertently left in place (from original attempt to solve problem)
//		- added check for a FLASH controller error just prior to the soapstring erase ....   if prior error had occurred, it will send an
//		  error, then try to clear it so that the flash erase for the soap can proceed (it would fail otherwise).
//////////// RELEASED - 2021/01/12
// 4.202a
//		- modify the reporting for the "force" field for CanMotor and 405 Heads.
//		- adding holding torgue (min current) to closed loop steppers   *REQUIRES Medusa 4.052a or higher on CLS motors
//		- fixed bug with M627 (abort/job kill relative/absolute move) which was introduced in 4.201c when attempting to
//		  set the values to INVALID_ARG_VALUE after a job end or job kill .... which they were, except they were set to INVALID
//		  BEFORE the abort move was generated.   now they are set to INVALID after the abort move occurs OR with an M30
//		  added option #define RESET_ABORT_MOVE_ON_JOB_END for whether or not to INVALIDATE the abort move after and abort or M30
//		  currently not defined, so values will be persistent
//////////// RELEASED - 2021/01/18
// 4.202b
//		- added C8 option to M800 for experimental waveformGeneration mode on using the new 405 board.
//////////// RELEASED - 2021/01/21
// 4.202c
//		- changed M788 to no longer need a password.    also added two args:
//			argA --- if A==1, then perform a full reset of the CLS head
//			argP --- if P==1, then reset the current position to match the encoder position (resets error to 0, etc)
//////////// RELEASED - 2021/01/21
// 4.203a
//		- added E and F args to M785 to allow setting the autoCal and prediction modes for the TLE5012 in the CLS setup.
//		- added L arg to M785 to allow setting the angular error reporting limit for canAxisMotors
//		  WARNING!!!!  THE VALUES MUST BE SAVED TO FLASH AND THEN FULLLY RESTART THE CLS DEVICE
//		- one more revamp of M788 ... covers all levels of resets not for canAxisMotors
//		      MCODE M788 <"T" toolSelector> [F restoreFactoryDefaults] [D deviceReset] [P resetPosition] [C canReset]
//		- added G28.1 to locally home canAxisMotors either to the sensor or until the force limit is reached/
//		- added M789 for locally sending sideband step pulses to a specific canAxisMotor (similar to M868/M869 or
//		  sending in crtl char #11/#12, but NOT restricted to the Z axis.
//		- added M698 for filament dispenser control (AM2302)
//		- added M699 for filament dispenser control (HX711 calibration, tare, etc)
//		- changed G2.2 to allow full disk spiral in/out by allow center to be either the start or end point.
//		- fixed lastRequestedQPoisition change from 4.200mz.  inadvertanlty this was set to a reverse calc of the position
//		  rather than the user's requested position (often different do to one being at the resolution of the step grid).
//		- fixed bug introduced about 4.200g with per-dot flow control on inkjets
//		- fixed bug in error throttling for sendMotionError().... errorCnt was not incremented
//		- change M723 in two ways:  (1) if ARG_E exceeds 65535 it is clamped to 65535 (max supported) (2) added C1 option
//		  to force continuous extrude (30 second timeout still applies)
//		- added thread pitch control to G84/G84.1 (request from Karl)
//		- changed heartbeat for anything with closed loop stepper that is not a canAxisMotor to get an additional 4 values
//		- printed in the message to repetrel (forces, error, v12-voltage, v12-amperage
//		- in initFromSoapstring(), fixed countdown variable.... had been executing when count was non-zero, rather then on
//		  on the transition to zero.
//		- added a "wiggle" all Motor direction pins to work around a bug in the BTT V1 motors in that they need to see an
//		  edge transition to properly go the correct direction.  this is ONLY for their code and noy hyrel's code that
//		  runs on their hardware.
//		- changed two checked for the home sensor in Motordrivers.c from ==TRIPPED to !=OPEN ... in order to safely cover
//		  other possible cases (like UNKNOWN).
//		- using CCMRAM again via pointer access to get around Coocox CoFlash bug. ALL serial buffers are now in CCMRAM.
//      - RASTER: #define ALLOW_DIRECT_BUFFER_FOR_RASTER_DATA  removed all ifdefs, made permanent
//		- enables modifed M253/M254/M255 to work with canMotors.   the critical issue is M255 REQUIRES an axis arg IF trying
//		  to shut down a can axis motor
//		- added a check and "fix" of the FLASH->SR reg which will show illegal address access.  the error is "sticky" and
//		  reports the last slice that executed to help with tracking down issues.  by adding this, it will hopefully prevent
//		  cases were we fail to write the soapstring due to a prior error (if the FLASH->SR shows issues, erases and writes
//		  are blocked until the SR is cleared.
//		- major overhaul of the diplay and GUI code (currently if'defed out) as work is not complete.   matching new look and
//		  feel of the head code and far more code is now shared between the projects.
//		- merged a lot of code from medusa into util.c in the common folder
//		- added several new display pages for SPI_DISPLAY
//		- created new .o for master.o to separate v4 from v5 to support the link file change to get the CCMRAM working
//		- ported M784 from Hydra5 to provide a report of installed devices as part or the end of job metrics
//		- added a couple of new metrics....to show the "best case" print time had accel rate been infinite.  also shows
//		  the average print and non-print speeds in this ideal cases
//////////// RELEASED - 2021/03/15
// 4.203b
//      - cleanup a few unused variables and code improvements found while creating the print time calculator code for repetrel.
//////////// RELEASED - 2021/04/25 -------------
// 4.203c
//		- fixed bug for temperature report for chamber (4.203a revamped the reporting code based on head type to more cleanly support
//		  several new head types.   chamber temp was inadvertantly divided by 32)
//////////// RELEASED - 2021/07/07 -------------
////
//4.203D
//changed maximum temp for hotbed 100 to 120c
////////////////////////////////////////////////////////////////////////////////


#define GB_DEBUGGING
#endif

////////////////////////////////////////////////////////////////////////////////
//
// HYDRA NEAR TERM FUTURE WORK ("TO DO" REMINDERS)
//      - remove CANBUS V0 code
//      - finish adding "scripts" to gcode processor (see ALLOW_GCODE_SCRIPTS)
//      - finish ALLOW_NATIVE_LIGHTBURN and enable in regular release
//      - if Karl decides to do direct control of CO2 Laser supply, then redo laser control / pins / HSS
//        etc SEE: NEW_CO2_LASER_CONTROL_MODULE
//      - add debug display+menu control from display via push encoder
//      - which flavor of M7 for karl?
//      - add M104/140/141 check for maxTemp based on head type
//      - FIX FLOW RATE change with M756 and friends ... no flow rate change should ever be sent to head except
//        manual AND view motionQ_execute.
//      - finish adding 1 of 6 selection code (USE_6_TO_ONE_SELECT)
//      - finish reverse G2.1
//      - full 3 space 3D rotary pipe command
//      - remove all GB_HIDDEN_WARNINGS
//      - change all overridePct to be 0.0 to 15.99 --- and clamp composite values to allowable range
//      - remove abortDecel constant and MCODE ... only used for homing
//      - merge G2/3/12/13/etc into one set of code and clean up multi cycle
//              * add common entry (save all state)
//              * and common exit (restore state
//      - minimize slowdown/stopping of unprime and prime commands
//      - allow prime to also go negative, but on the PRIOR move.
//      - DMA -- look into using DMA for serial RX/TX (especially USB)
//      - revamp ERROR message for uniformity and criticalilty (which "abort", etc)
//
//      - diags: wiring change for PH3-PH4 dual connector fix and for getting second connector coverage for
//        MX_L2 and PB0/START
//      - diags: check pullup state
//      - diags: adc read of internal voltage or bias voltage (research what's possible
//
// HEAD CODE
//      - wireless head -- see notes on workbench riser
//      - laser pierce
//      - PnP LEDS are wrong
//
// NOTES:
//      - SOAP:   0x80E 0000     0x80FFC00
//

#ifndef COMPILE_FOR_SYSTEM
#define COMPILE_FOR_SYSTEM
#endif

#define PLATFORM_STRING       "Hydra_X10"
#define USE_CAN2    // enable use of second canbus controller, CAN2


//#ifdef USE_HYDRA_IO
//#define USE_AB_ENCODER      // affects IO definitions
//#endif //USE_HYDRA_IO

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "includes.h" // contains list of all the STM library includes used
#include "hyrel.h"
#include "hyrel_can.h"
#include "util.h"
#include "bootloader.h"
#include "string.h" // random utilities (including bzero)
#include "headSetup.h"
#include "pins.h"
#ifdef ADD_ON_SPI_DISPLAY
#include "lcd.h"
#endif //ADD_ON_SPI_DISPLAY

////////////////////////////////////////////////////////////////////////////////
//  main specific global defines and macros needed by other modules
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////  "NEW" FEATURE ifdef's -- one accepted, then remove from code
////////////////////////////////////////////////////////////////////////////////

// defined starting in 4.100c

#define BALANCED_TIME

#ifdef GB_DEBUGGING
#define ALLOW_NATIVE_LIGHTBURN  // not for public consumption yet
#endif //GB_DEBUGGING

//#define ALLOW_JOGGING_HEAD_MOTOR_WITH_CAN_STEP    // enable if karl decides to let jog wheel control the head motor
//#define NEW_MCODE_TOOL_USE                        // enable if karl decides to use one of the new mappings he requested
//LEGACY
// end of new 4.100c items

//defined in 4.100e
//#define USE_NEW_SOAPSTRING_DELIMTER_AND_STRIP_COMMENTS    // enable to change soapstring delimeter from ';' to another char ('?')
															// and then auto strip ALL comments before hitting the rx buffer.
#define CHECK_FOR_COLD_EXTRUDE
#define COLLECT_METRICS
#define ENABLE_CRASH_LOGGING

//#define SKIP_M190 // wait for hotbed temperature
//#define SKIP_M191 // wait for chamber temperature
//#define LASER_LOCAL_CONTROL_TESTING_ON_SYS30
//#define GB_FAST_UART                   // crank up the UART rate to 115K


////////////////////////////////////////////////////////////////////////////////
//  END OF DEBUG DEFINTIONS --- SHOULD BE DISABLED WHEN COMPILER FOR RELEASE
////////////////////////////////////////////////////////////////////////////////

#define GB_STRING_MOTION            0x000001    // 1
#define GB_STRING_PRIME             0x000002    // 2
#define GB_STRING_PRIME_RUN         0x000004    // 4
#define GB_STRING_EXTRUSION         0x000008    // 8

#define GB_STRING_COMM_ACK_CHECK    0x000010    // 16
#define GB_STRING_CMD_QUE           0x000020    // 32
#define GB_STRING_FLOW              0x000040    // 64
#define GB_STRING_RATE              0x000080    // 128

#define GB_STRING_CANBUS            0x000100    // 256
#define GB_STRING_DUMP_ON_START     0x000200    // 512
#define GB_STRING_ADD_MOTIONQ       0x000400    // 1024
#define GB_STRING_ECHO_COMM         0x000800    // 2048

#define GB_STRING_WORKBUFFER        0x001000    // 4096
#define GB_STRING_STEPS_PER_NL      0x002000    // 8192
#define GB_STRING_LOOKAHEAD         0x004000    // 16384
#define GB_STRING_ARC_INFO          0x008000    // 32768

#define GB_STRING_RASTER            0x010000    // 65536
#define GB_STRING_JOG_INFO          0x020000    // 131072
#define GB_STRING_CYL_INFO          0x040000    // 262144
#define GB_STRING_CLS_IFO           0x080000    // 524288

#define GB_STRING_CYL_INFO_SUBMASK_MOVE_SETUP   0x0001
#define GB_STRING_CYL_INFO_SUBMASK_MOVE_COORDS  0x0002
#define GB_STRING_CYL_INFO_SUBMASK_MQ_INFO      0x0004
#define GB_STRING_CYL_INFO_SUBMASK_MQ_SPEED     0x0008

#define GB_STRING_JOG_INFO_SUBMASK_1            0x0001
#define GB_STRING_JOG_INFO_SUBMASK_2            0x0002

#define GB_STRING_RASTER_SUBMASK_DETAILS		0x0001
#define GB_STRING_RASTER_SUBMASK_CALIB			0x0002

#define GB_STRING_CLS_SUBMASK_EXTRA_RT_FIELDS	0x0001

////////////////////////////////////////////////////////////////////////////////

// PICK ONE
#define DEFAULT_FLASHER_VAR_SEL     FLASHER_VAR_SEL_NONE
//#define DEFAULT_FLASHER_VAR_SEL       FLASHER_VAR_SEL_ERROR
//#define DEFAULT_FLASHER_VAR_SEL       FLASHER_VAR_SEL_CMD_RECEIVED
//#define DEFAULT_FLASHER_VAR_SEL       FLASHER_VAR_SEL_ACK_PENDING
//#define DEFAULT_FLASHER_VAR_SEL       FLASHER_VAR_SEL_ABORTING

//PICK ONE
#define DEFAULT_FLASHER_LIGHT_SEL   FLASHER_LIGHT_SEL_NONE
//#define DEFAULT_FLASHER_LIGHT_SEL FLASHER_LIGHT_SEL_LED
//#define DEFAULT_FLASHER_LIGHT_SEL FLASHER_LIGHT_SEL_DDL
//#define DEFAULT_FLASHER_LIGHT_SEL FLASHER_LIGHT_SEL_BOTH

// if forgot to select one, set the default to OFF
#ifndef DEFAULT_FLASHER_VAR_SEL
#define DEFAULT_FLASHER_VAR_SEL     FLASHER_VAR_SEL_NONE
#endif
#ifndef DEFAULT_FLASHER_LIGHT_SEL
#define DEFAULT_FLASHER_LIGHT_SEL   FLASHER_LIGHT_SEL_NONE
#endif
#define FLASHER_ON_COUNT 2

////////////////////////////////////////////////////////////////////////////////

//#define RESET_ABORT_MOVE_ON_JOB_END
#define LED_COUNTER_START_VALUE_FOR_ONE_SHOT_FLASH              2
#define REPETREL_COMM_WATCHDOG_START_VALUE (5 * 60)  // each ping is once per minute.... will countdown each second -- 5 minute timeout
#define CLEAR_STICKY_ERROR_RATE 300  // allow repeat error messages ever N seconds
#define MAX_MANUAL_LASER_PULSE_TIME_MS 1000 // 1 second
#define MAX_MANUAL_UV_LIGHTRAY_PULSE_TIME_MS 60000 // 60 seconds
#define LASER_ENABLED               (_gs._laser.enabled)
#define MAX_MOTION_WATCHDOG_MS       700000 // 700 seconds (700mm @ 1mm sec)
#define LASER_LOCAL_PWM_CONTROL     (_gs._laser.enabled && _gs._laser.localControl)   // ensures current device is a laser and under local control

#define KARLS_PASSWORD ((uint32_t)666)
#define DEFAULT_HOT_HEAD        11
#define DEFAULT_HOT_BED         91

#define SIZE_OF_COMMAND_QUEUE   60
#define SIZE_OF_DEFERRED_COMMAND_QUEUE   15
#define SIZE_OF_MOTION_QUEUE    30
#define NUM_HEAD_OFFSETS        60
#define NUM_FIXTURE_OFFSETS     16
#define NUM_TOOL_OFFSETS        60
#define NUM_Z_CALIBRATION_SENSORS 5
#define HOST_WATCHDOG_TIMEOUT_MS 30000

#define ALIAS_TABLE_SIZE 256    // used to map incoming HH device number to diff number
#define DLY_100US  450

// ALL IN 10Hz loop
#define PNP_WAIT_FOR_MOTION_TO_COMPLETE_TIME    150 // 10Hz loop countdown (15 seconds)
#define PNP_WAIT_REEL_ADVANCE_DEFAULT_TIME_MS	100
#define CANBUS_PROBE_ARM_COMPLETION_TIME        10  // way more than enough time
#define CANBUS_HOMING_COMPLETION_TIME			40	// heads check in every second when they are homing
#define GUI_WAIT_FOR_CMD_TO_COMPLETE_TIME	20	// 2 sec
#define CAN_WAIT_FOR_LOOPBACK_TIME				30	// 3 sec

#define BOOTUP_HIGH_VOLTAGE_SETTLING_TIME_MS        2000    // give 2 seconds of settling
#define EMO_HIGH_VOLTAGE_SETTLING_TIME_MS           1000    // give 1 seconds of settling

#define IGNORE_EMO_DURING_BOOT_TIME_MS              BOOTUP_HIGH_VOLTAGE_SETTLING_TIME_MS

#define OVERRIDE_PCT_MAX    (15.99f) // (1.99f) // must be below 2.0f
#define OVERRIDE_PCT_MIN    (0.0f) // (0.5f)

////////////////////////////////////////////////////////////////////////////////
//
//  system mem map of 407
//  sector0 - 16KB - start of real program (up to 896KB total)
//  sector1 - 16KB
//  sector2 - 16KB
//  sector3 - 16KB
//  sector5 - 128KB
//  sector6 - 128KB
//           :               :
//  sector10 - 128KB - max end of real program
//  sector11 - 128KB - soapstring


#define MCU_DEVICE_407 0x413
#define MCU_DEVICE_429 0x419

#define NVIC_VECTOR_OFFSET      0x00000000  // Set the Vector Table base location at 0x08000000

#define NVIC_PREMPTION_PRIORITY_MEASURE_ISR_RATE        0   // debug/test only
#define NVIC_PREMPTION_PRIORITY_UARTS                   1
#define NVIC_PREMPTION_PRIORITY_USB                     2
#define NVIC_PREMPTION_PRIORITY_XTAL_TIMING_CHECK       3   // (boot time only)
#define NVIC_PREMPTION_PRIORITY_SLICETIME_MEASURE       4   // debug/test only
#define NVIC_PREMPTION_PRIORITY_TOUCH_PROBE             5
//#define NVIC_PREMPTION_PRIORITY_                          6
#define NVIC_PREMPTION_PRIORITY_MASTER_MOTION_TIMER     7
#define NVIC_PREMPTION_PRIORITY_STEP_TIMER              8
#define NVIC_PREMPTION_PRIORITY_MASTER_ACCEL_CALC       9
#define NVIC_PREMPTION_PRIORITY_STEP_ZJOG               10
//#define NVIC_PREMPTION_PRIORITY_                          11
//#define NVIC_PREMPTION_PRIORITY_                          12
#define NVIC_PREMPTION_PRIORITY_DELAY                   13
#define NVIC_PREMPTION_PRIORITY_SYSTICK                 14
#define NVIC_PREMPTION_PRIORITY_SYSTICK_MEASURE         15  // debug/test only
#define NVIC_PREMPTION_PRIORITY_LOWEST                  15

#define TIM_INDEX_DELAY					3
#define DELAY_TIMER						TIM3

////////////////////////////////////////////////////////////////////////////////

#define NUM_1000HZ              8
#define NUM_100HZ               10      // must remain 10 otherwise loop timing will not be correct
#define NUM_10HZ                10      // must remain 10 otherwise loop timing will not be correct
#define NUM_1HZ                 10      // must remain 10 otherwise loop timing will not be correct

#define SYSTICKS_PER_SECOND     (NUM_1000HZ * NUM_100HZ * NUM_10HZ * NUM_1HZ)

typedef enum {
	HZ_1000 = 0,
	HZ_100  = 1,
	HZ_10   = 2,
	HZ_1    = 3
} indexType;

#define HEARTBEAT_MODE_BOOT         5  // 10 bps
#define HEARTBEAT_MODE_ABORT        10  // 5 bps
#define HEARTBEAT_MODE_PENDING_ACK  25  // 2 bps bps
#define HEARTBEAT_MODE_NORMAL       100  // 0.5 bps

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	FORMAT_U8_DEC   = 0,
	FORMAT_I8_DEC   = 1,
	FORMAT_U16_DEC  = 2,
	FORMAT_I16_DEC  = 3,
	FORMAT_U32_DEC  = 4,
	FORMAT_I32_DEC  = 5,
	FORMAT_U64_DEC  = 6,
	FORMAT_I64_DEC  = 7,
	FORMAT_FLT      = 8,
	FORMAT_DOUB     = 9,
	FORMAT_U8_HEX   = 10,
	FORMAT_I8_HEX   = 11,
	FORMAT_U16_HEX  = 12,
	FORMAT_I16_HEX  = 13,
	FORMAT_U32_HEX  = 14,
	FORMAT_I32_HEX  = 15,
	FORMAT_U64_HEX  = 16,
	FORMAT_I64_HEX  = 17,
	FORMAT_INT_TEMP = 18,
	FORMAT_FLT_TEMP = 19,
	FORMAT_ADDR     = 20,
	FORMAT_CAL      = 21,
	FORMAT_I8_CHAR  = 22
} formatType;

#define CRASHLOG_START ((uint32_t)0xDEADBEEF)
#define CRASHLOG_END   ((uint32_t)0xFEEDF00D)

typedef struct {
	char label[32];
	formatType format;
	byte *addr;
} crashLog_struct;

typedef enum {
	FLASHER_VAR_SEL_NONE                    = 0,
	FLASHER_VAR_SEL_ERROR                   = 1,
	FLASHER_VAR_SEL_CMD_RECEIVED            = 2,
	FLASHER_VAR_SEL_ACK_PENDING             = 3,
	FLASHER_VAR_SEL_G4_DWELL_TIMER          = 4,
	FLASHER_VAR_SEL_GCODE_PAUSED            = 5,
	FLASHER_VAR_SEL_ABORTING                = 6,
	FLASHER_VAR_SEL_BLOCK_ALL_MOTION        = 7,
	FLASHER_VAR_SEL_BLOCK_ABS_MOTION        = 8,
	FLASHER_VAR_SEL_MOTION_OCCURRED         = 9,
	FLASHER_VAR_SEL_VALID_KEY               = 10,

	FLASHER_VAR_SEL_HOME_STATE_X            = 21,
	FLASHER_VAR_SEL_FAULT_STATE_X           = 22,
	FLASHER_VAR_SEL_LIMIT1_STATE_X          = 23,
	FLASHER_VAR_SEL_LIMIT2_STATE_X          = 24,

	FLASHER_VAR_SEL_HOME_STATE_Y            = 31,
	FLASHER_VAR_SEL_FAULT_STATE_Y           = 32,
	FLASHER_VAR_SEL_LIMIT1_STATE_Y          = 33,
	FLASHER_VAR_SEL_LIMIT2_STATE_Y          = 34,

	FLASHER_VAR_SEL_HOME_STATE_Z            = 41,
	FLASHER_VAR_SEL_FAULT_STATE_Z           = 42,
	FLASHER_VAR_SEL_LIMIT1_STATE_Z          = 43,
	FLASHER_VAR_SEL_LIMIT2_STATE_Z          = 44,

	FLASHER_VAR_SEL_HOME_STATE_A            = 51,
	FLASHER_VAR_SEL_FAULT_STATE_A           = 52,
	FLASHER_VAR_SEL_LIMIT1_STATE_A          = 53,
	FLASHER_VAR_SEL_LIMIT2_STATE_A          = 54,

	FLASHER_VAR_SEL_HOME_STATE_B            = 61,
	FLASHER_VAR_SEL_FAULT_STATE_B           = 62,
	FLASHER_VAR_SEL_LIMIT1_STATE_B          = 63,
	FLASHER_VAR_SEL_LIMIT2_STATE_B          = 64,

	FLASHER_VAR_SEL_HOME_STATE_C            = 71,
	FLASHER_VAR_SEL_FAULT_STATE_C           = 72,
	FLASHER_VAR_SEL_LIMIT1_STATE_C          = 73,
	FLASHER_VAR_SEL_LIMIT2_STATE_C          = 74,
} flasherVarSel_t;


typedef enum {
	FLASHER_LIGHT_SEL_NONE      = 0,
	FLASHER_LIGHT_SEL_LED       = 1,
	FLASHER_LIGHT_SEL_DDL       = 2,
	FLASHER_LIGHT_SEL_BOTH      = 3,
} flasherLightSel_t;

typedef struct {
	flasherLightSel_t   lightSel;
	flasherVarSel_t     varSel;
	int                 *varPtr;
	int                 error;
	int                 cmd_received;
	int                 ack_pending;
	//int                   aborting; //use live variable since not 'flashing'
} flasherStruct;

typedef enum {
	UNKNOWN_BUFFER    = 0,
	SOAPSTRING_BUFFER = 1,
	URGENT_BUFFER     = 2,
	NORMAL_BUFFER     = 3
} buffer_t;

typedef enum {
	UNKNOWN_MASTER  = 0,
	UART3_MASTER    = 1,
	UART4_MASTER    = 2,
	UART6_MASTER    = 3,
	USB_MASTER      = 4,
	BOOTUP          = 5
} masterPort_t;

#define ABORT_TIME 5 // 5 second watchdog to complete abort process.
typedef enum {
	FLUSH_THEN_ABORT,   // used for real abort --
	FLUSH_THEN_CONTINUE, // used for touch probe
} flushAction_t;

////////////////////////////////////////////////////////////////////////////////


#define EXTRUDE_CONTINUOUS_FORWARD   1000000000.0
#define EXTRUDE_CONTINUOUS_REVERSE  -1000000000.0

#define WAIT_FOR_TEMP_ALLOWABLE_SHORTFALL   1

#ifdef GB_DEBUG_ARC
#define ARC_LENGTH_PER_SEGMENT 20 //(0.33333f)
#else   //!GB_DEBUG_ARC
#define ARC_LENGTH_PER_SEGMENT (0.33333f)
#endif //!GB_DEBUG_ARC
#define ARC_TOLERANCE   (0.005f) // 0.005 mm  per RS274/NGC Interpreter spec  was:(0.1f)
#define G2G3_THREAD_PITCH_TOLERANCE (0.05f)
#define CYL_TOLERANCE   (0.01f)
// could be an enum instead
#define NO_OK           0
#define FRONT_DOOR_OK   1
#define BACK_DOOR_OK    2
#define BOTH_OK         3

#define FLOW_RATE_CREATED_ON_HEAD       (!_hijackAxisC)     //(!(_canbusStepForE || _directStepForE))
#define FLOW_RATE_CREATED_ON_MOTORC     (_hijackAxisC)      //(_canbusStepForE || _directStepForE)
#define USING_E_VALUES                  (_extrusionControl == USE_E_VALUES)
#define IGNORING_E_VALUES               (_extrusionControl == IGNORE_E_VALUES)

////////////////////////////////////////////////////////////////////////////////

#define INVALID_ARG_VALUE ((float)-999999) // replace extern float Invalid;

#define ARG_A   (ExecutionPtr->A)
#define ARG_B   (ExecutionPtr->B)
#define ARG_C   (ExecutionPtr->C)
#define ARG_D   (ExecutionPtr->D)
#define ARG_E   (ExecutionPtr->E)
#define ARG_F   (ExecutionPtr->F)
#define ARG_G   (ExecutionPtr->G)
#define ARG_H   (ExecutionPtr->H)
#define ARG_I   (ExecutionPtr->I)
#define ARG_J   (ExecutionPtr->J)
#define ARG_K   (ExecutionPtr->K)
#define ARG_L   (ExecutionPtr->L)
#define ARG_M   (ExecutionPtr->M)
#define ARG_N   (ExecutionPtr->N)
#define ARG_O   (ExecutionPtr->O)
#define ARG_P   (ExecutionPtr->P)
#define ARG_Q   (ExecutionPtr->Q)
#define ARG_R   (ExecutionPtr->R)
#define ARG_S   (ExecutionPtr->S)
#define ARG_T   (ExecutionPtr->T)
#define ARG_U   (ExecutionPtr->U)
#define ARG_V   (ExecutionPtr->V)
#define ARG_W   (ExecutionPtr->W)
#define ARG_X   (ExecutionPtr->X)
#define ARG_Y   (ExecutionPtr->Y)
#define ARG_Z   (ExecutionPtr->Z)
#define ARG_CS  (ExecutionPtr->CS)

#define ARG_A_PRESENT   (ARG_A != INVALID_ARG_VALUE)
#define ARG_B_PRESENT   (ARG_B != INVALID_ARG_VALUE)
#define ARG_C_PRESENT   (ARG_C != INVALID_ARG_VALUE)
#define ARG_D_PRESENT   (ARG_D != INVALID_ARG_VALUE)
#define ARG_E_PRESENT   (ARG_E != INVALID_ARG_VALUE)
#define ARG_F_PRESENT   (ARG_F != INVALID_ARG_VALUE)
#define ARG_G_PRESENT   (ARG_G != INVALID_ARG_VALUE)
#define ARG_H_PRESENT   (ARG_H != INVALID_ARG_VALUE)
#define ARG_I_PRESENT   (ARG_I != INVALID_ARG_VALUE)
#define ARG_J_PRESENT   (ARG_J != INVALID_ARG_VALUE)
#define ARG_K_PRESENT   (ARG_K != INVALID_ARG_VALUE)
#define ARG_L_PRESENT   (ARG_L != INVALID_ARG_VALUE)
#define ARG_M_PRESENT   (ARG_M != INVALID_ARG_VALUE)
#define ARG_N_PRESENT   (ARG_N != INVALID_ARG_VALUE)
#define ARG_O_PRESENT   (ARG_O != INVALID_ARG_VALUE)
#define ARG_P_PRESENT   (ARG_P != INVALID_ARG_VALUE)
#define ARG_Q_PRESENT   (ARG_Q != INVALID_ARG_VALUE)
#define ARG_R_PRESENT   (ARG_R != INVALID_ARG_VALUE)
#define ARG_S_PRESENT   (ARG_S != INVALID_ARG_VALUE)
#define ARG_T_PRESENT   (ARG_T != INVALID_ARG_VALUE)
#define ARG_U_PRESENT   (ARG_U != INVALID_ARG_VALUE)
#define ARG_V_PRESENT   (ARG_V != INVALID_ARG_VALUE)
#define ARG_W_PRESENT   (ARG_W != INVALID_ARG_VALUE)
#define ARG_X_PRESENT   (ARG_X != INVALID_ARG_VALUE)
#define ARG_Y_PRESENT   (ARG_Y != INVALID_ARG_VALUE)
#define ARG_Z_PRESENT   (ARG_Z != INVALID_ARG_VALUE)

#define ARG_A_MISSING   (ARG_A == INVALID_ARG_VALUE)
#define ARG_B_MISSING   (ARG_B == INVALID_ARG_VALUE)
#define ARG_C_MISSING   (ARG_C == INVALID_ARG_VALUE)
#define ARG_D_MISSING   (ARG_D == INVALID_ARG_VALUE)
#define ARG_E_MISSING   (ARG_E == INVALID_ARG_VALUE)
#define ARG_F_MISSING   (ARG_F == INVALID_ARG_VALUE)
#define ARG_G_MISSING   (ARG_G == INVALID_ARG_VALUE)
#define ARG_H_MISSING   (ARG_H == INVALID_ARG_VALUE)
#define ARG_I_MISSING   (ARG_I == INVALID_ARG_VALUE)
#define ARG_J_MISSING   (ARG_J == INVALID_ARG_VALUE)
#define ARG_K_MISSING   (ARG_K == INVALID_ARG_VALUE)
#define ARG_L_MISSING   (ARG_L == INVALID_ARG_VALUE)
#define ARG_M_MISSING   (ARG_M == INVALID_ARG_VALUE)
#define ARG_N_MISSING   (ARG_N == INVALID_ARG_VALUE)
#define ARG_O_MISSING   (ARG_O == INVALID_ARG_VALUE)
#define ARG_P_MISSING   (ARG_P == INVALID_ARG_VALUE)
#define ARG_Q_MISSING   (ARG_Q == INVALID_ARG_VALUE)
#define ARG_R_MISSING   (ARG_R == INVALID_ARG_VALUE)
#define ARG_S_MISSING   (ARG_S == INVALID_ARG_VALUE)
#define ARG_T_MISSING   (ARG_T == INVALID_ARG_VALUE)
#define ARG_U_MISSING   (ARG_U == INVALID_ARG_VALUE)
#define ARG_V_MISSING   (ARG_V == INVALID_ARG_VALUE)
#define ARG_W_MISSING   (ARG_W == INVALID_ARG_VALUE)
#define ARG_X_MISSING   (ARG_X == INVALID_ARG_VALUE)
#define ARG_Y_MISSING   (ARG_Y == INVALID_ARG_VALUE)
#define ARG_Z_MISSING   (ARG_Z == INVALID_ARG_VALUE)

typedef enum {
	//NOTE: items 2 and 3 behave the same, but commands should be defined as one or the other for possible future enhancements.
	UNDEFINED                     = 0,
	SINGLE_STEP                   = 1,  // single step .... wait for empty queue, set ForceQToEmpty, process, reset ForceQToEmpty
	ADD_TO_MQ                     = 2,  // add to Q or process in order with Q (tag as processed if not added to Q, ie G90)
	IMPACTS_DESTINATION_CALC      = 3,  // ONLY affects position (ie, change of offsets) or has no impact on position/motion and does not need to be synced
	SYNCS_WITH_MOTION             = 4   // skip, but tag, until prior motion complete...unless Q is empty, the just process
} command_t;

#define UNPROCESSED -1 // used to init cmdLink on cmdQue

typedef struct {
	unsigned A : 1;
	unsigned B : 1;
	unsigned C : 1;
	unsigned D : 1;
	unsigned E : 1;
	unsigned F : 1;
	unsigned G : 1;
	unsigned H : 1;
	unsigned I : 1;
	unsigned J : 1;
	unsigned K : 1;
	unsigned L : 1;
	unsigned M : 1;
	unsigned N : 1;
	unsigned O : 1;
	unsigned P : 1;
	unsigned Q : 1;
	unsigned R : 1;
	unsigned S : 1;
	unsigned T : 1;
	unsigned U : 1;
	unsigned V : 1;
	unsigned W : 1;
	unsigned X : 1;
	unsigned Y : 1;
	unsigned Z : 1;
} flag_t;

typedef struct {
	float A;    // motor A, 4th axis
	float B;    // motor B, 4th axis
	float C;    // motor C, 4th axis
	float D;    // tool working diameter offset
	float E;    // used for extrusion value
	float F;
	float G;    // gcode
	float H;    // Tool working length offset
	float I;    // used for circular arc commands, defines offset to center of circles
	float J;    // used for circular arc commands, defines offset to center of circle
	float K;
	float L;
	float M;    // mcode
	float N;
	float O;
	float P;
	float Q;
	float R;
	float S;
	float T;    // tool number
	float U;
	float V;
	float W;
	float X;    // motor X, 1st axis
	float Y;    // motor Y, 2nd axis
	float Z;    // motor Z, 3rd axis
	float CS;
	//union {
	//   flag_t flag;
	//   unsigned int flags;
	//} valid;
	command_t cmdType;  // defined type indicating how this type of command interracts with the motionQ
	int cmdLink;        // index to parent command (link to motionQ from deferredQ)
} GMCommandStructure;

typedef enum {
	DEFERRED_CMD_INVALID = 0,
	DEFERRED_CMD_VALID_BUT_NOT_READY_TO_PROCESS,
	DEFERRED_CMD_VALID_AND_READY_TO_PROCESS,
} deferred_cmd_t;

typedef struct {
	GMCommandStructure  cmd;
	deferred_cmd_t      cmdState;
} GMDeferredCommandStructure;

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	NO  = 0,
	YES = 1
} yes_no_t;

typedef enum {
	HOME_TOWARD_ZERO = 0,
	HOME_AWAY_FROM_ZERO = 1
} homingDirection_t;

typedef enum {
	DIRECTION_FORWARD = 0,
	DIRECTION_REVERSE = 1
} direction_t;

typedef enum {
	SENSOR_INDEX_NONE = 0,
	SENSOR_INDEX_FAULT,
	SENSOR_INDEX_LIMIT1,
	SENSOR_INDEX_LIMIT2,
	SENSOR_INDEX_START,
	SENSOR_INDEX_EMO,
	SENSOR_INDEX_HOME,
	SENSOR_INDEX_ABSEL,
} sensorIndex_t;

#define DEFAULT_SENSOR_DEBOUNCE_READS	3   // N consectutive reads of same value to be valid state.. one or two reads of alternate value will be rejected
#define FAULT_SENSOR_DEBOUNCE_READS 	15 	// N consectutive reads of same value to be valid state.. a few reads of alternate value will be rejected

typedef struct {
	sensorIndex_t   SensorIndex;            // which sensor;
	uint32_t        Bit;                    // bit definition for pin/port
	GPIO_TypeDef    *gpioPort;
	uint32_t        gpioMask;
	char            Name[8];                // give bit a name for error reporting
	polarity_t      Polarity;               // acitive hi/low
	int             NumDebounceMaskBits;    // number of successive consistent reads needed to debounce and get a valid state
	uint32_t        DebounceMask;           // mask of how many successive consistent reads are needed to debounce (ie, 0x007 for 3 reads)
	sensorState_t   State;                  // last read state
	uint32_t        History;                // optionally used for debounce, record of last 32 "states"
	boolean         Enabled;                // TRUE is sensor is being checked
	boolean         ErrorMessageSent;       // TRUE when message sent to host that the sensor was tripped
	boolean         StateChangeDetected;    // last read of sensor caused a change of state (ie, OPEN to CLOSED/TRIPPED
	boolean         TransientDetected;      // flag to indicate the a non-uniform (history & mask) so either noise OR a tranisition starting
} sensorStruct;

typedef struct {
	uint32_t        Bit;
	GPIO_TypeDef    *gpioPort;
	uint32_t        gpioMask;
	polarity_t      Polarity;
} controlBitStruct;

typedef struct {
	uint32_t        Bit;
	GPIO_TypeDef    *gpioPort;
	uint32_t        gpioMask;
	yes_no_t        InvertDefault;
} directionBitStruct;

typedef enum {
	IGNORE_E_VALUES     = 0,  // calculate flow based on layer H, W and motion
	USE_E_VALUES        = 1,
	INVALID_E_CONTROL   = 2
} E_control_t;

typedef enum {
	MOTION_FLOW_USE_FEEDRATE        = 0,    // calculate flow rate based on requested Feedrate (Gx Fxxx)
	MOTION_FLOW_USE_CRUISE_SPEED    = 1,    // calculate flow rate based on cruising speed for each move
	MOTION_FLOW_USE_AVG_SPEED       = 2,    // calculate flow rate based on average speed for each move
} motionForFlowControl_t;

typedef enum {
	AUTO_UNPRIME_PRIME = 0,
	GCODE_UNPRIME_PRIME = 1,
	MCODE_UNPRIME_PRIME = 2,
	E_ARG_UNPRIME_PRIME = 3,
	NO_UNPRIME_PRIME    = 4
} unprime_prime_control_t;

typedef enum {
	ARC_CW = 0,
	ARC_CCW = 1,
} arcDir_t;

typedef enum {
	ARC_PLANE_NONE = 0,
	ARC_PLANE_XY = 1,
	ARC_PLANE_XZ = 2,
	ARC_PLANE_YZ = 3,
} arcPlane_t;

typedef enum {
	ARC_STATE_IDLE = 0,
	ARC_STATE_1 = 1,
	ARC_STATE_2 = 2,
	ARC_STATE_3 = 3,
} arcState_t;

typedef struct {
	float X;
	float Y;
	float Z;
} pointStruct;

typedef struct {
	float X;
	float Y;
	float Z;
	float mag;
} vectorStruct;

typedef struct {
	pointStruct     centerOffset;
	pointStruct     startPt;
	pointStruct     endPt;
	pointStruct     endPtABC;
	pointStruct     centerPt;
	pointStruct     deltaABC;
	float           radius;
	float           sweepAngle;
	int             segments;
	int             fullCircleSegments;
	int             numFullThreads;

	vectorStruct    center2startV;      //vector from center to start point
	vectorStruct    center2startUV;     //unit vector from center to start point
	vectorStruct    center2endV;        //vector from center to end point
	vectorStruct    center2endUV;       //unit vector from center to end point
	vectorStruct    rotationNormalUV;
	vectorStruct    derivedNormalUV;
	vectorStruct    reverseDerivedNormalUV;
	vectorStruct    displacementV;      // temp variable used in point gen loop


	vectorStruct    ortho2center2startUV; //unit vector orthogonal to center2startUV
	arcPlane_t      plane;
	arcDir_t        dir;
	float           displacement;   // helical displacement along normal
	float           dotCheck;

	boolean         sweepIsMultipleOf180Degrees;
	boolean         normalsSameDir;
	boolean         normalsOppoDir;
	boolean         skipFinalArcMove;   // used for frogToe;
	boolean         fullCircle;
	boolean         inscribedShape;     // true when full circle and ARG_S specified
	boolean         normalWasSpecified;
	boolean         threadingOK;

	GMCommandStructure  *saveExecutionPtr;
	boolean         saveIncrementalMove;
	boolean         saveIncrementalEMove;
	E_control_t     saveExtrusionControl;

	int             index;
	arcState_t      state;
} arcStruct;

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	float ToolLength;
	float ToolDiameter;
} ToolOffsetStructure;

typedef enum {
	AXIS_MIN    = 0,    // machine min rate for this axis in raw units per second (UPS)
	AXIS_MAX    = 1,    // machine max rate for this axis in raw units per second (UPS)
	NO_RAMP     = 2,    // speeds at or below this do not need acceleration  in raw units per second (UPS)
	RAPID       = 3,    // aka G0 travel rate for this axis in raw units per second (UPS)
	HOMING      = 4,    // aka G28 and Jog rate for this axis in raw units per second (UPS)
	REHOMING    = 5,    // speed for second pass homing
	JOG_NO_RAMP = 6,    // no ram speed for jogging
	JOGGING     = 7,    // speed for jogging
	NUM_RATES
} rate_t;

#define VECTOR_MOTION_RATE     0.0f
#define RAPID_MOTION_RATE     -1.0f
#define HOMING_MOTION_RATE    -2.0f
#define REHOMING_MOTION_RATE  -3.0f
#define RASTER_MOTION_RATE    -4.0f
#define JOGGING_MOTION_RATE   -5.0f
//#define ONESHOT_MOTION_RATE   -5.0f

#define RASTER_REVERSE_SCAN_TWEAK 0 //-0.125    // offset from end position of positive dir line to start return line
#define RASTER_FRONT_PORCH_EXTRA_PIXELS_MULTIPLIER 1.4f  // force extra (1.2 = 20%)front porch to make sure motion really at speed

typedef enum {
	LINEAR = 0,
	ROTARY
} axis_t;


#define SWITCH_PRESCALE_1       1
#define SWITCH_PRESCALE_10      10
#define SWITCH_PRESCALE_100     100
#define SWITCH_PRESCALE_500     500

#define TICKS_PER_SEC_1000HZ_LOOP 1000 // 1000 Hz loop
#define TICKS_PER_SEC_100HZ_LOOP  100  // 100 Hz loop
#define TICKS_PER_SEC_10HZ_LOOP   10   // 10 Hz loop
#define TICKS_PER_SEC_1HZ_LOOP    1    // 1 Hz loop

#define DONT_CHANGE_PERIOD -1

typedef enum  {
		NO_HSS_PIN_INDEX = 0,
		HSS_AUX_PWR1_INDEX = 1,
		HSS_AUX_PWR2_INDEX = 2,
		HSS_AUX_PWR4_INDEX = 3,
		HSS_AUX_PWR5_INDEX = 4,
		HSS_AUX_PWR6_INDEX = 5,
		HSS_AUX_PWR7_INDEX = 6,
		HSS_AUX_PWR8_INDEX = 7,
		HSS_AUX_PWR9_INDEX = 8,
		NUM_HSS_PINS
} hssPin_t;

typedef enum {
	NO_FUNCTION_HSS         = 0,
	SPINDLE_COOLANT_HSS     = 1,
	FLOOD_COOLANT_HSS       = 2,
	DANGER_LIGHT_HSS        = 3,
	DDLIGHT_HSS             = 4,
	RESPONSE_HSS            = 5,
	BUZZER_HSS              = 6,
	CHAMBER_FAN_HSS         = 7,
	LASER_XHAIR_HSS         = 8,
	EXHAUST_FAN_HSS         = 9,
	VACUUM_HSS              = 10,
	AIR_ASSIST_HSS          = 11,
	ULTIMUS_HSS             = 12,
	CO2_POWER_SUPPY_HSS     = 13,
	CO2_Coolant_Pump		= 14,
	NUM_HSS_FUNC
} hssFunc_t;

typedef struct {
	controlBitStruct Output;

	boolean oneShot;
	float DutyCycle;
	float PeriodInSec;
	int TicksPerSecond;  // ticks=100 for 100Hz loop; 10 for 10Hz loop, 1 for 1Hz loop, etc
	int TerminalCount;
	int CompareValue;
	int Counter;
	hssPin_t index;
} HssPwmStruct;

#define HSS_DUTY_CYCLE_ON     100.0f
#define HSS_DUTY_CYCLE_OFF    0.0f

#define HOMING_MOTORS (TRUE)
#define NOT_HOMING_MOTORS (FALSE)

//#undef M_E      // was defined in math.h for "e" #define M_E 2.7182818284590452354

typedef enum {   // used for motor array indices
	M_X = 0,
	M_Y = 1,
	M_Z = 2,
	M_A = 3,
	M_B = 4,
	M_C = 5,
	MAX_NUMBER_OF_MOTORS
} motorIndex_t;


typedef enum {
	LATHE_MODE_OFF          = 0,
	LATHE_MODE_ON           = 1,
} latheMode_t;

typedef struct MotorStructure MotorStructure;   // forward declaration of struct so we can declare a ptr inside struct
typedef void (*MotorStructureFunc)(MotorStructure *);

#define MAX_NUM_CAN_MOTORS_PER_AXIS 4
typedef struct {
	byte canAddress;				// list of physical canbus addresses for motor driving this axis
	boolean selfHomingInProgress;	// individual canAxisMotor is in the process of homing
	boolean selfHomed;				// individual canAxisMotor has been self homed
} canMotorState_t;

typedef struct MotorStructure {
	uint32_t Axis;                      // array index
#ifdef GB_DEBUG_MOTION_Q
	float debugArgDValue;
	int lineNumber;
#endif
	uint32_t GCodeArgOffset;            // byte offset to this motor's GCodeArg in the GMCommandStructure
	axis_t AxisType;                    // LINEAR or ROTARY;
	char AxisLabel;                     // single char label for axis used for reporting

	boolean canMotor;					// TRUE is axis is controlled via canbus
	int32_t maxCanMotors;				// per axis limit on # can motors allowed
	boolean axisSelfHomingInProgress;	// tag that the axis as a whole was homing .. save processing at the end
	canMotorState_t canMotors[MAX_NUM_CAN_MOTORS_PER_AXIS];

	boolean MotorInstalled;             // TRUE if motor/axis is installed and operational
	boolean SendStepViaCanbus;          // if TRUE, when motor steps, it will send out a step message via the canbus to specified device
	//NUKE byte DeviceForCanStep;              // destination address of device to receive canStep packet
	boolean SubstituteAxisArgWithArgE;  // flag to indicate whether to substitute the E value for this axis' value in a gcode line

	int32_t POSITION;                   // absolute position in pulses of the current position (must be signed int for JogMotor)
	int32_t DIRECTION;                  // == 1 for forward and == -1 for reverse
	int32_t PULSES_TO_GO;               // pulses remaining in this step
	int32_t PULSES_IN_MOVE;             // total pulses needed for the move (stays constant during the move)
	float RatesInUPS[NUM_RATES];        // contains all of the various rate limits and targets
	int32_t LastReportedPosition;

	float Q_LastRequestedPositionInUnits;   // last actual location without offets in mm or deg added to the motionQ
	int32_t Q_POSITION;                 // last absolute FINAL position in pulses of a move added to the mo
	int32_t TouchPosition;              // recorded position when using touch probe

	float startVelocity;                // starting velocity for move (used when homing or dominant)
	float cruiseVelocity;               // "at speed" velocity for move (used when homing or dominant)
	float cruiseVelocitySquared;
	float endVelocity;                  // final velocity for move (used when homing or dominant)
	float currentVelocity;              // velocity at the current moment for move (used when homing or dominant)

	int32_t AccelerationPulses;         // number of pulses to accelerate from startVelocity to cruiseVelocity (used when homing or dominant)
	int32_t DecelerationPulses;         //  number of pulses to decell from cruiseVelocity to endVelocity (used when homing or dominant)
	float AccelerationScaled;           // acceleration rate for the current move (scaled from the AccelerationConstant) (used when homing or dominant)
	float AccelerationScaledTimes2;
	float MotionRateInSPP;              // motion rate in seconds / pulse (inverted)
	float unitVector;                   // component of unit vector
	float scaleDegreesToMm;             // used to convert speed on rotary axis from deg to mm to match feedrate

	float Dominant2AxisPulseRatio;      // simple ratio pulses in the dominant axis to the current axis ... used to schedule "next" pulses.
	float AxisFirstPulseOffset;         // offset for when the first pulse will occur
	boolean PulsePending;               // marks that the next pulse has been scheduled but not yet occurred

	float RotaryOffset[MAX_NUMBER_OF_MOTORS]; // insection point of the rotary axis plane  axis of the cylinder

	int32_t unprimePulseToGo;           // pulse to match to PULSES_TO_GO at the time to issue the unprime to the hothead
	int32_t residualUnprimeTime;        // time in ms remaining for the unprime AFTER the move completes

	float AccelerationConstant;         // SPEC'D: "constant" to calc initial accel relative to velocity (units/sec/sec)
	float AbortDecelerationRate;        // SPEC'D: "constant" to calc final deceleration relative to velocity (units/sec/sec) when motion is aborted so that the motion can be gracefully (but agressively) stopped

	float JogAccelerationConstant;      // SPEC'D: "constant" to calc initial accel relative to velocity (units/sec/sec)
	float JogIncrement;
	int32_t JogPauseTimeMs;
	struct {
		unsigned    JOG_NO_RAMP : 1;
		unsigned    JOGGING : 1;
		unsigned    ACCEL : 1;
	} joggingRatesSet;

	float PulsesPerUnit;                // SPEC'D: pulses per mm of travel - if changed, all other rates need to be updated
	float UnitsPerPulse;                // inverse of PulsesPerUnit (allows use of multiply instead of divide saving 13 cycles)
	int32_t PulsesPerRevolution;            // inferred for rotary axis
	float MaximumTravelInUnits;         // SPEC'D: software limit for each axis in travel in mm or deg
	int MaximumTravelInPulses;          // DERIVED: software limit for each axis in travel in pulses

	float AbortTravelScaling;
	float MinimumAbortTravelInUnits;
	float MaximumAbortTravelInUnits;
	int32_t MinimumAbortTravelInPulses;
	int32_t MaximumAbortTravelInPulses;
	

	float AbortAbsolutePosition;
	float AbortRelativePosition;

	float G92Offset;                            // SPEC'D: G92offset in mm or degrees
	float FixtureOffsets[NUM_FIXTURE_OFFSETS];  // SPEC'D: tool offsets in mm or degrees
	float HeadOffsets[NUM_HEAD_OFFSETS];        // SPEC'D: head offsets in mm or degrees

	// HomingStart and End Point allow flipping the coordinate system and putting "Home" opposite from 0
	boolean SearchingForHomeSensor;     // used for homing routine. set high to start; cleared when home detected
	homingDirection_t HomingDirection;  // whether to home toward 0 or away from 0
	float HomeDestinationInUnits;       // need to store in case pulse/unit changes so able to recalc HomeDestinationInPulses
	int32_t HomeDestinationInPulses;        // motor POSITION after the homing sequence finishes
	float HomeHysteresisInUnits;        // SPEC'D: distance from first tripping sensor to actual zero position
	float HomePopOffDistanceInUnits;    // SPEC'D: distance to pop off from home during the remhoming pass

	boolean HasBeenHomed;               // true if axis has been home (set false again on "unlock motors" and EMO press (if wired)
	boolean HasBeenHomed_BlockedMsgSent; // true is error message has already been sent about moves being rejected if not homed
	boolean HomingFailed;               // temp variable to indicate homing procedure had an issue (ie, insufficient pop off)

	boolean autoReportPosition;
	boolean reportPositionWithStatus;

	latheMode_t latheMode;
	float latheAccelPer10ms;
	float latheTargetSpeedPPS;
	float latheCurrSpeedPPS;

	sensorStruct HomeSense;
	sensorStruct FaultSense;
	sensorStruct Limit1Sense;
	sensorStruct Limit2Sense;

	controlBitStruct Enable;
	controlBitStruct Step;
	directionBitStruct Direction;

	TIM_TypeDef* TimerBase;             // pointer to Timer HW that this motor will use for it's step pulse generation
	float TimerFreq;                    // raw input clock frequency to the timer
	uint32_t TimerPSC;                  // timer PSC load value (already offset by 1)  - used when homing
	uint32_t TimerARR;                  // timer ARR load value (already offset by 1)) - used when homing

	//DEPRECATED - NUKE?????  SAVE FOR SOMEDAY
	// these are unused at the moment....useful if there are unique per motor things that need to be done
	//GX XXX FIX void (*CallBackHomeSense)(MotorStructure*);    //method for checking Home status
	//typedef void (*myFuncDef)(MotorStructure *);
	//void (*CallBackHomeSense)(void);  //method for checking Home status
	//MotorStructureFunc CallBackHomeSense;   //ptr to a func with one arg void func(MotorStructure *);
	//void (*CallBackPrimeRun)(void);   //signals when to restart the flow of material after a non extrude move or event
	//void (*CallBackRetract)(void);    //signals when to start retracting the material to stop the flow sharply.
	//void (*MotionCallBack)(void);     // called after EVERY motor step issue
	//void (*CallBackHomeComplete)(void);   //man I really hope this works, we can add a lot of routines
} MotorStructure;

typedef enum { // used in countdown fashion
	G28_FINISHED_MULTIPASS_HOME         = 0,
	G28_SET_DESTINATION_REHOME          = 1,
	G28_PASS_REHOME                     = 2,
	G28_PASS_LEAVE_HOME                 = 3,
	G28_SET_DESTINATION_HOME            = 4,
	G28_PASS_HOME                       = 5,
	G28_PASS_MOVE_INTERMEDIATE_POINT    = 6,
	G28_START_MULTIPASS_HOME            = 7
} g28Pass_t;

typedef enum { // used in countdown fashion
	G38_FINISHED_MULTIPASS_PROBING      = 0,
	G38_MOVE_TO_NEXT_XY                 = 1,
	G38_RAISE_PROBE                     = 2,
	G38_WAIT_FOR_RESULTS                = 3,
	G38_READ_DEPTH                      = 4,
	G38_PAUSE_BEFORE_READING            = 5,
	G38_LOWER_PROBE                     = 6,
	G28_ARM_PROBE                       = 7,
	G38_START_MULTIPASS_PROBING         = 8
} g38Pass_t;

#define MIN_VECTOR_SPEED 0.01f  // minimum vector speed (include start and end speed) ie, 0.25f = 0.25mm/spec == 0.59 inch/min

typedef struct MotionEntryStruct {
	 int lineNumber;         // try to track gcode line number through the Q
#ifdef GB_DEBUG_MOTION_Q
	float debugArgDValue;   // gcode "D" arg value -- used to trip a breakpoint on a given line
#endif
#ifdef GB_DEBUG_ARC
	int TAG;
#endif
	int cmdQueIndex;        // index of cell this command occupied in the command queue
	struct {
		unsigned needToPrimeBeforeMove   : 1; // flag to indicate a need to issue a extrusion "prime" before this move
		unsigned needToUnprimeAfterMove  : 1; // flag to indicate a need to issue a extrusion "upprime" after this move
		unsigned needToUnprimeDuringMove : 1; // flag to indicate a need to issue a extrusion "upprime" during this move
		unsigned unprimeIssued           : 1; // flag to indicate the unprime can message has been sent
		unsigned hadAnE                  : 1; // flag to indicate an "E" value was present with the line of Gcode leading to the move
		unsigned homingMove              : 1; // flag to indicate this move is a homing move in search of the home sensors
		unsigned rasterMove              : 1; // flag to indicate the move is a rasterizing move (ie, inkjet or laser)
		unsigned dominantAxis            : 3; // axis index of the dominant axis
		unsigned inMotion                : 1; // flag to indicate motion has already started (can't be updated; and speedControl can start
		unsigned lastInSequence          : 1; // flag to indicate the move is the last of a sequence (needed in order to know when it is
											  // time to process deferred commands)
		unsigned needToPuffBeforeMove    : 1; // flag used in G81/G83/G84 to tag moves that start with a "puff" (turning on a HSS)
		unsigned pauseAfterThisMove      : 1; // flag used to tell sequence engine to stop moving until a restart is issued;
		unsigned abortAfterThisMove      : 1; // flag used to tell sequence engine to stop moving until a restart is issued;
		unsigned hasDeferredCommands         : 1; // flag to indicate there are deferred commands to process after this command
	} flags;

	int8_t direction[MAX_NUMBER_OF_MOTORS]; // == 1 for forward and == -1 for reverse (0 for not moving)
	int32_t PULSES_TO_GO[MAX_NUMBER_OF_MOTORS]; // pulses remaining in this move === WILL BE NEGATIVE TEMPORARILY, so leave as "int"
	float unitVector[MAX_NUMBER_OF_MOTORS]; // unit vector along the path from the prior dest to the dest of this move
	float requestedFeedrate;            // pass through for ReportXYZ
	float moveTime;                     // move time .. used for unprime/prime removal (starts without accel/decel, but added later)
	float perfectMoveTime;				// move time .. with INFINITE acceleration -- used for metrics
	float distance;                     // in "units"   -- distance of travel for the move
	float speed;                        // in units/sec -- cruising speed
	float startSpeed;                   // in units/sec -- speed to start the move
	float startSpeedLimit;              // in units/sec -- max start speed based on intersection with prior move and prior moves' speed
	float noRampSpeed;                  // in units/sec -- safe starting speed (no need to accearate to this value .. just start with this)
	float acceleration;                 // in units/sec/sec -- acceleration for the move (along the vector)
	float scaleDegreesToMm;             // used to convert speed on rotary axis from deg to mm to match feedrate
	float flowOverridePct;              // composite of all flow mutipliers at the time adding the move to the motionQ
	float flowCrossSectionTimesPPU;     // if !USING_E, will be cross section of volume (W*H) times the PPuL -- just need motion rate to get pulse rate, 0 otherwise
	int32_t flowPulses;                   // if USING_E, will be number of E steps; otherwise 0
	E_control_t extrusionControl;   // need to pass this through to execute so commands can more easily save/restore (ie, G2.1)

	// WARNING!!!!
	// index, next and prev pointers MUST be declared at the end of the struct so it's easy to bzero rest of struct
	 int index;              // physical entry position to make it easier to peek into memory
	struct MotionEntryStruct *next; // pointer to next physical entry (not necessarily valid entry)
	struct MotionEntryStruct *prev; // pointer to prior physical entry (not necessarily valid entry)
} motionEntryStruct;
#define MOTION_ENTRY_STRUCT_BYTES_TO_PRESERVE   12

typedef struct {
	int validEntries;
	int countdownDelayToExecuteMove; // ms countdown timer to allow "aging" first few entries in the motionQ to allow forward planning
	motionEntryStruct entries[SIZE_OF_MOTION_QUEUE];
	motionEntryStruct *oldest;  // pointer to oldest addition to the motionQ
	motionEntryStruct *newest;  // pointer to newest addition to the motionQ
	motionEntryStruct *planned; // pointer to the oldest entry that still needs to be planned
} motionQStruct;

typedef struct {
	float time;
	float dist;
	float perfectTime;	//used in print/non-print tallies to tabulate "infinite accelerating" time
} timeDistStruct;

typedef struct outboxStruct outboxStruct;   // forward declaration of struct so we can declare a ptr inside struct

typedef struct {
	byte device;
	outboxStruct  *outboxPtr;
	boolean enabled;
	polarity_t polarity;
	int   watchdogMs;
	int   cooldown;
	boolean localControl;
	int   pwmFreq;
	int PwmTestCounter;

	float vectorPowerPct;
	float piercePowerPct; //power pct for pierce (during prime)

	int rasterBitsPerDot;
	int rasterImageDotsPerBinaryValue;
	int rasterPulsesPerDot;  // how man motor steps per pixel (width of pixel in steps)
	int rasterPulseToFireOn; // which subDot step to send out packet to head (inkjet should be in the middle, laser at the start)
	int rasterPulsesPerDotCounter;
	int rasterFrontPorchDots; // number of pixels to wait for before starting raster data
	int rasterFrontPorchDotsCounter;
	int rasterLineCounter;

	boolean rasterReverseDirection;
	int rasterTotalTraverseDots;
	int rasterFirstTraverseDot;
	int rasterLastTraverseDot;
	int rasterFirstActiveDot;
	int rasterLastActiveDot;
	int rasterTraverseDotCounter;
	int rasterActiveBinaryValue;
	unsigned int rasterBinaryValueMask;

	float rasterScanFeedrate;
	float rasterImageOriginMmX;
	float rasterImageOriginMmY;
	float rasterImageDotsPerMmX;
	float rasterImageDotsPerMmY;

	int rasterImageDotsX;
	int rasterImageDotsXmod;    // round up to match groupings of 8 pixels
	int rasterBytesPerFullLine;
	float rasterTotalLineLength;
	int rasterImageDotsY;
	int rasterImageLineCmdEndIndex;
	float rasterMaxPowerPercent;

	int rasterOffValue;
	int rasterOffValueLastWord;
	int rasterLineExcessDots;
	byte rasterColorIndexTable[256];
	float gammaValue;
	float rasterBidirReturnStartAdjust;
	float nextValue;

	boolean rasterBidirectionalScan;
	boolean rasterUseColorIndexTable;
	boolean rasterSkipBlankBorders;
	boolean rasterInvertData;
	boolean rasterizeCurrentMove;
	boolean fireLaserOnNextStep;
	boolean rasterLineComplete;

	TIM_TypeDef *TimerBase;
} laserStruct;

typedef struct {
	float   desiredPowerPct;
	int     pwmFreq;
} spindleStruct;

typedef struct {
	float     dropletsPerMm;
} inkjetStruct;

typedef struct {
	uint32_t startTimeSec;
	timeDistStruct nonPrinting;
	timeDistStruct printing;
	timeDistStruct accel;
	timeDistStruct cruise;
	timeDistStruct decel;
	timeDistStruct prime;
	int canbus_e_steps;
	int unprimes;
	int primes;
	int unprime_primes_avoided;
	int numMoves;
	int motionQ_entriesWhenExecuting[SIZE_OF_MOTION_QUEUE+1];
	int cmdQue_entriesWhenReceived[SIZE_OF_COMMAND_QUEUE+1];
	int max_CommandsInQue;
	int max_motionQvalidEntries;
	int max_DeferredCommandsInQue;
	int max_rawRxCharsInBuf;
	int max_rawUsbRxCharsInBuf;

	int max_urgentRxCharsInBuf;
	int max_normalRxCharsInBuf;
	int max_directRxCharsInBuf;

	int max_normalTxCharsInBuf;
	int max_canTxQ_numMsg;
	int max_canRxQ_numMsg;

	int rejected_normalTxChars;

	int total_commandsProcessed;
	int total_motionQprocessed;
	int total_DeferredProcessed;
	int total_charsRx;
	int total_charsTx;
	int total_canRx;
	int total_canTx;
	int flushedRxCharsDuringAbort;
	int autoDumpOnM30;
} metricsStruct;

#define NUM_SLICE_TIME_ENTRIES  (NUM_1000HZ + NUM_100HZ + NUM_10HZ + NUM_1HZ)

typedef struct {
	char name[16];
	int minTime;
	int maxTime;
	int motionTimerCalls;
} sliceStruct;

typedef struct {
	int matchSlice;
	float counterPeriod;
	uint32_t pin;
	sliceStruct slices[NUM_SLICE_TIME_ENTRIES];
} sliceTimingStruct;

typedef struct {
	union {
		uint32_t u32;
		struct
		{
			unsigned canNumBytesMismatch            : 1;
			unsigned canDestinationUnknown          : 1;
			unsigned canTxFull                      : 1;
			unsigned canRxFull                      : 1;

			unsigned rawRxBufferOverrun             : 1;
			unsigned directRxBufferUnderrun         : 1;
			unsigned directRxBufferOverrun          : 1;

			unsigned urgentRxBufferUnderrun         : 1;
			unsigned urgentRxBufferOverrun          : 1;
			unsigned normalRxBufferUnderrun         : 1;
			unsigned normalRxBufferOverrun          : 1;

			unsigned machineLimitsMaxRateInUPS      : 1;
			unsigned machineLimitsMinRateInUPS      : 1;

			unsigned illegalUseOfAxisC              : 1;
			unsigned unknownDeviceRegistering		: 1;
			unsigned flashStatusErrorDectected		: 1;
			unsigned guiQueueFull					: 1;
		} flags;
	} sent;
} errorSentStruct;

typedef enum {
	PRECANNED_STEP = 0,
	PRECANNED_LASER = 1,
	NUM_PRECANNED,
} preCannedIndex_t;

#ifdef ADD_ON_SPI_DISPLAY
#define GUI_DISPLAY_SPI SPI2
#endif //ADD_ON_SPI_DISPLAY

//NUKE
//#ifdef ADD_ON_SPI_DISPLAY
//#define GUI_DISPLAY_SPI SPI2
//typedef struct {
//	offOn_t             offOnState;
//	boolean             invertedState;
//	int                 page;
//	int                 nextPage;
//	int                 maxPage;
//	byte                rotation;
//	int                 updateIntervalCountMs;
//	int                 updateIntervalMs;
//	int                 vOfs;
//	int                 hOfs;
//	int                 debouceCnt;
//} displayStruct;
//#endif //ADD_ON_SPI_DISPLAY

typedef struct {
	union {
		uint32_t u32;
		struct {
			unsigned led0       : 1;
			unsigned led1       : 1;
			unsigned led2       : 1;
			unsigned led3       : 1;
			unsigned led4       : 1;
			unsigned led5       : 1;
			unsigned led6       : 1;
			unsigned led7       : 1;
		} generic;
		struct {
			unsigned moving     : 1;
			unsigned paused     : 1;
			unsigned blockAbs   : 1;
			unsigned blockAll   : 1;
			unsigned error      : 1;
			unsigned canRx      : 1;
			unsigned canTx      : 1;
			unsigned heartbeat  : 1;
		} baseline;
		struct {
			unsigned lower4     : 4;
			unsigned upper4     : 4;
		} mgmt;
	} currValue;
	uint32_t    lastLcdLedValue;
	int16_t     canRxLedCnt;            // counter to know when to turn off Rx can bus activity indicator
	int16_t     canTxLedCnt;            // counter to know when to turn off Tx can bus activity indicator
	int16_t     usageLedCnt;            // counter to know when to fire generic "usage" LED (step, spew, etc)
	int			heartbeatState;
} ledStruct;

////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	byte            _ctrlIndex[4];              // index values for each of the control loops
	uint32_t        _sliceCnt;                  // count of slices executed
	int				_sliceNum;					// which slice number is executing

	uint32_t        _milliseconds;
	uint32_t         _seconds;                   // number of seconds since powerup (more accurately, number of times through the systick loop)

	int32_t         _extendedSliceTimeNeeded;   // counter for number of extra slices needed by long routines

	canStruct       _canImmediateRx; // buffer to hold next can message to process
	boolean         _canImmediateRxIsAvail;
	canStruct       _canRx; // buffer to hold next can message to process
	boolean         _canRxIsAvail;
	rxQueueStruct   _canRxQ;
	txQueueStruct   _canTxQ;
	uint32_t        _canTransmitMailboxErrorCount[NUM_CAN_HW_UNITS][NUM_CAN_TRANSMIT_MAILBOXES];
	uint32_t        _canTransmitAbortedPackets[NUM_CAN_HW_UNITS][NUM_CAN_TRANSMIT_MAILBOXES];
	byte            _preDefinedAliases[NUM_PRE_DEFINED_ALIASES];        // needed for can routines....in the future, the system can have aliases too!
	byte            _userDefinedAliases[NUM_USER_DEFINED_ALIASES];  // needed for can routines....in the future, the system can have aliases too!
	deviceBootloaderStruct  _bl;
	int16_t         _calibrationTemp;
	byte            _reportCalibrationTemperatureDevice;
	boolean         _reportCalibrationTemperature;
	uint32_t        _reportCalibrationTemperatureRate;
	boolean         _scannerAttached;
	boolean         _jumpToApplicationMain;
	laserStruct     _laser;
	inkjetStruct    _inkjet;
	spindleStruct   _spindle;
	//NUKE int             _preCannedLaserPending;
	int             _preCannedStepPending;
	canSwStruct     _preCannedPackets[NUM_PRECANNED]; // canStep, canLaserPower
	int             _errorCount;
	int				_errorThrottledCount;
	ledStruct		_led;
	flasherStruct   _flasher;
	arcPlane_t      _selectedArcPlane;
	MotorStructure *latheMotor;
	int             _totalMarginedMoveTimeMs;
//#ifdef ADD_ON_SPI_DISPLAY
//   char             _revStr[16];
//NUKE 	displayStruct   _display;
	boolean			_displayIsAlive;
	boolean			_displayForceRedraw;
	int				_displaySelectDebounceCnt;
//#endif //ADD_ON_SPI_DISPLAY
} globalStruct;

#define MAX_ERROR_MESSAGES  5
////////////////////////////////////////////////////////////////////////////////
//  SIMPLE MACROS
////////////////////////////////////////////////////////////////////////////////

#define sqr(X) ((X) * (X))
//#define max(a,b) (((a) > (b)) ? (a) : (b))
//#define min(a,b) (((a) < (b)) ? (a) : (b))

////////////////////////////////////////////////////////////////////////////////
//  Global Variables defined in main that can be referenced by other modules
////////////////////////////////////////////////////////////////////////////////

extern globalStruct _gs;                      // control index, canbus ques and other stuff
extern motionQStruct motionQ;
extern GMCommandStructure cmdQue[SIZE_OF_COMMAND_QUEUE];
extern GMDeferredCommandStructure deferredCmdQue[SIZE_OF_DEFERRED_COMMAND_QUEUE];
extern int NextDeferredCommandIndex;         // index of the next command to be executed (the one after Current), initially set to 2 after the first command is received and parsed
extern int NextDeferredCommandInsertionIndex;// index of the next free location to add an incomming command
extern boolean DeferredCmdReadyToProcess;

extern MotorStructure Motors[];
extern GMCommandStructure *ExecutionPtr;   //executionPtr=&cmdQue[CurrentCommandPointer];
extern boolean ForceReportXYZLocation;
extern boolean AutoReportXYZLocation;   // GB XXX FLAG
extern boolean AutoReportFeedRate;   // GB XXX FLAG
extern boolean AutoReportLineNum;
extern boolean StatusReportXYZLocation;   // GB XXX FLAG
extern boolean StatusReportVelocity;   // GB XXX FLAG
extern boolean StatusReportFlowRate;   // GB XXX FLAG
extern boolean StatusReportLineNum;
extern boolean _printAir; // GB XXX FLAG
extern float _autoReverseAndPrimeMinTime;
extern E_control_t _extrusionControl;
extern E_control_t _saveExtrusionControl;
extern motionForFlowControl_t _motionForFlowControl;
extern unprime_prime_control_t _unprimePrimeControl;
extern boolean _hijackAxisC;
extern boolean _canbusStepForE;
extern boolean _directStepForE;
extern float E_Q_LastRequestedPositionInUnits;
extern int E_Q_POSITION;
extern float TurboModeDistance;
extern int G1Flag;
extern int G0Flag;
extern boolean _reverseFrogToe;
extern boolean _spiralRoughingPass; // SPIRAL
extern float _deltaRForPass; // SPIRAL
extern int G2G3Flag;
extern int G203State;
extern boolean _motionQ_LookAheadDisabled; //GB XXX FLAG
extern boolean _motionQ_ForceQtoEmpty; //GB XXX FLAG
extern boolean _blockImmediateMoveOnToolChange; //GB XXX FLAG
extern int dump407SoapstringFlag;
extern int CurrentLayer;
extern int _gcodeLineNumber;
extern int _gcodeMotionLineNumber;
extern int DDLightFunction;
extern int PrimeTimer;
extern int UnPrimeTimer;
extern ToolOffsetStructure ToolOffsets[NUM_TOOL_OFFSETS];
extern int currentToolIndex;
extern int currentFixtureIndex;
extern int currentHeadIndex;
extern GMCommandStructure *CannedCyclePtr;
extern int CannedCycleLastMove;
extern int CannedCycleFlag;
extern int CannedCycleStep;
extern float CannedCycleZFeedRateInMmPerSec;
extern float CannedZDepthInMM;
extern float CannedZDesiredDepth;
extern float CannedZIncrement;
extern float CannedZQIncrement;
extern float CannedThreadPitch;
extern float CannedZClearInMM;
extern int CannedLinearAxis;
extern int CannedRotaryAxis;
extern HssPwmStruct *CannedHssPtr;
extern boolean CannedCyclePuffFlag;
extern int _repetrelCommWatchCount;
extern boolean _waitingForGuiCommand;
extern int _abortInProgress;
extern int _abortCameFromRx;
extern boolean _abortFinisedNeedToResetProcessSynchronously;
extern uint32_t _g4DwellTimer;
extern boolean _gcodePaused;
extern boolean _blockAllMotion;
extern boolean _blockAbsoluteMotion;
extern boolean _blockAllMotionMessageSent;
extern boolean _blockAbsoluteMotionMessageSent;
extern boolean _blockMotionForJoggingMessageSent;
extern boolean _blockMotionForHighVoltageMessageSent;
extern boolean _blockJoggingForMotionMessageSent;
extern boolean _motionSensorTripped;
extern boolean _validFirmwareKey;
extern char GCodeArgComment[];
extern char _GcodeArgStringParam[];
extern boolean _requestToPauseAtEndOfMove;
extern boolean _requestToPauseAtEndOfLayer;
extern boolean _requestToAbortAtEndOfMove;
extern boolean _abortOccurredWhileMoving;
extern int32_t _needToWiggleDirectionPins;
extern GMCommandStructure *G1UnprimePtr;
extern GMCommandStructure *LookAheadPointer;
extern int PreProcessNextCodeFlag;
extern int CommandsInQue;
extern int DeferredCommandsInQue;
extern int LastCmdQueIndexAddedToMotionQ;
extern boolean _needToProcessDeferredCommands;
extern boolean _needToDeleteAndExecuteNextMove;

extern boolean _Tim7StillCalculating;
#ifdef ALLOW_NATIVE_LIGHTBURN
extern boolean _lightburnModeEnabled;
#endif //ALLOW_NATIVE_LIGHTBURN
extern sensorStruct startButton;
extern sensorStruct EMO;
#ifdef USE_AB_ENCODER
extern sensorStruct ABEncoderSelectButton;
#endif //USE_AB_ENCODER
extern int _highVoltageIsNotStableCoundownMs;

#ifdef COLLECT_METRICS
extern metricsStruct _metrics;
#endif //COLLECT_METRICS

//#ifdef MEASURE_TIME_SLIPPAGE
extern int _timeSlipTimer2Passes;
extern boolean _sendTimeSlippageData;
//#endif //MEASURE_TIME_SLIPPAGE

extern boolean _TouchProbeMoveActive;
extern byte _TouchProbeCanDevice;
extern int _TouchProbeEXTI_Line;
extern uint8_t _TouchProbeNVIC_IRQChannel;

extern boolean _edgeTriggerArmed;
extern boolean _edgeTriggerDetected;
extern float _edgeTriggerDisplayRateHz;

extern int pendingAcknowledge;
extern int _gcodeCmdsReceived;
extern int _gcodeAcksSent;
extern unsigned int _asciiChecksum32;
extern unsigned int _rejected_normalTxChars;
extern unsigned int _asciiCharsRcvd;
extern int normalRxBufHeadroom;
extern float _FlowRateOverridePct;
extern float _FeedRateOverridePct;
extern float _MotionRateOverridePct;
extern float _CompositeFlowOverridePct;
extern float _CompositeMotionOverridePct;
extern const crashLog_struct _crashLog[];
extern const int NUM_CRASHLOG_ENTRIES;
extern sliceTimingStruct _sliceTiming;
extern int _motionTimerCalls;
extern errorSentStruct _errors;
extern int HostConnectionWatchDog;
#ifdef ENABLE_CRASH_LOGGING
extern char crash_source[];
extern char blankChar;
extern int SerialPortOverRunCnt;
#endif
extern char resetSourceStr[];

extern int _heartbeatRateControl;
extern int32_t lsiActualFreq;
extern uint32_t lsiCC4sampleIndex;
extern uint32_t lsiCC4samples[];

extern int hssFuncToPinIndex[];
extern HssPwmStruct HighSideSwitches[];

extern char _bootupAlertHostChar;
extern boolean _sendBootupAlertHostChar;

extern pinType HEARTBEAT_PIN;
//osseo variables
extern int EnableOsseoVariablesReporting;
extern int ParticleCounter;
extern float EnclosureTemperature;
extern int EnclosureHumidity;
extern int EnclosurePressureDifference;
extern int EnclosureUvLedPwm;
extern int EnclosureFanPwm;
extern int EnclosureDoorLock;
extern uint8_t EnclosureDoorSense;
extern int doorSenseState;
extern uint8_t previous_EnclosureDoorSense;
extern int EnclosureCartridgeSense;
extern int EnclosurePrintBedSense;

extern int	SpindleDesiredSpeedPWM;
extern int RPMCounter;
extern int	CO2LaserAnalogPwrPWM;
extern int Co2LaserWatchDogTimer;
extern int Update595Index;
extern int McodeDrainState[];
#define M610_State_Ofset 7			//open drain aux outputs J27
#define M611_State_Ofset 6
#define M612_State_Ofset 3
#define M613_State_Ofset 2
#define M614_State_Ofset 1
#define M615_State_Ofset 0
#define M616_State_Ofset 5            //Spindle motor direction
#define M617_State_Ofset 4            //Spindle Motor Enable


////////////////////////////////////////////////////////////////////////////////
//  Public Methods available in main
////////////////////////////////////////////////////////////////////////////////

extern uint32_t interruptsOff(void);
extern void interruptsOn(uint32_t);
extern void setTouchPositionToCurrentPosition(void);
extern void TouchProbeSendResults(probeResults_t, probeType_t, int);
extern void TouchProbeFinished(probeResults_t, probeType_t, int);
extern void TouchProbeProcessContact(probeType_t, int);
extern int getSliceCnt(void);
extern void pingReply (void);
extern int32_t getCurrentExecutingLineNumber(void);
extern void ReportXYZLocation(void);
extern void ReportOsseoVariables(void);
extern void PWMCntrl(void);
extern void wait100ns(void);
extern void SendNext407SoapString(void);
extern void setPauseGcodeFlag(char);
extern void ResetProcess(boolean);
extern boolean anyAxisNeedingToBeHomed(void);
extern sensorState_t readSensorBit(sensorStruct *);
extern void outputControlBit(controlBitStruct *, assertValue_t);
extern void assertControlBit(controlBitStruct *);
extern void deassertControlBit(controlBitStruct *);
extern int readControlBitValue(controlBitStruct *);
extern assertValue_t readControlBitState(controlBitStruct *);
extern void outputDirectionBit(directionBitStruct *, pinStateValue_t);
extern int readDirectionBitValue(directionBitStruct *);
extern direction_t readDirectionBitState(directionBitStruct *outputPtr);
extern int readSensorBitValue(sensorStruct *sensorPtr);
extern sensorState_t readSensorBitState(sensorStruct *);
extern void setSensorBitState(sensorStruct *sensorPtr, uint32_t value);
extern boolean sensorEnabledAndTripped(sensorStruct *sensorPtr);
extern int getNextDeferredCommandIndex(int);

extern void initSensor(sensorStruct *, sensorIndex_t, char *, boolean, polarity_t, pinType, int);
extern void setSensorToUnknownState(sensorStruct *);
extern sensorState_t readDebouncedSensorState(sensorStruct *);
extern sensorState_t fastReadDebouncedSensorState(sensorStruct *);
extern void updateSensorHistoryIfEnabledWithoutIrq(sensorStruct *);
extern void setSensorEnable(sensorStruct *, boolean);
extern void setSensorPolarityAndEnable(sensorStruct *, polarity_t, boolean);
extern sensorStruct *getMotorSensorPtr( MotorStructure *, sensorIndex_t);
extern float fpu_sqrtf(float);

extern boolean almostTheSameByPct(float, float, float);
extern boolean notAlmostTheSameByPct(float, float, float);
extern boolean almostTheSameByDelta(float, float, float);
extern boolean notAlmostTheSameByDelta(float, float, float);

extern void resetVariableOnJobEnd(boolean realJobEnd);
extern void puff(void);
extern void processCannedCycle(void);
extern void ProcessG28MultipassHoming(void);
extern void SetAxisHomePosition(MotorStructure *);
extern void SetAllAxisHomePosition(boolean);
extern void updateHssPwm(HssPwmStruct *);
extern void changeHssDuty(HssPwmStruct *, float);
extern void initAllHss(void);
extern int convertToolNumberToDevice(int toolNumber);
extern int convertDeviceToToolNumber(int device);
extern boolean cmdQueIsEmpty(void);
extern void processCommand(GMCommandStructure *);
extern void InvalidateAllCmdArgs(GMCommandStructure *);
extern boolean deferredCmdQueueFull(void);
extern void copyCmdQueEntryToDeferredCmdQue(GMCommandStructure *);
extern void processDeferredCommands(void);
extern void sendAxisHomedStatus(MotorStructure *);
extern void sendAllAxisHomedStatus(void);
extern char *getUIDString(char *);
extern void sendFakeM687CompleteCode(void);
extern void sendRevisionString(char *);
extern boolean ArmTouchProbe(int, probeType_t);
extern void ProcessTouchProbeAfterContact(void);
extern void Crash_Handler(char *);
extern void checkAndClearFlashStatus(void);
extern void protectedIncrement(int *);
extern void protectedDecrement(int *);
extern void catastrophicError(char *);
extern void catastrophicErrorWithDebugInfo(char *);
extern void requireAllAxesToHome(void);
extern void sendEmoMessage(void);
extern void EdgeTriggerSendResults(void);
extern boolean SetupEXTI(int, probeType_t, EXTITrigger_TypeDef, int);
extern void DisableEXTI(void);

extern void SendPNPSPIDataToSpi2(uint16_t);
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
#endif // #ifndef main_HEADER // prevent double dipping - MUST BE LAST LINE OF FILE


#if 0
// PRIOR HISTORY INFORMATION
// 1.04 new dynamic accelration for all motors
// 1.79 change the m790 pass thru, now it passes it on and adds S arg for Layer
// 1.811 added check tosee if unprime timer was zero, if it was disabled then we execut the move on the same slice.
// 1.82 changed M6 command to require a I0 statement to execute immediate movement
// 1.83 changed Initial Z axis settings
// 2.303 fixed pause at end of layer mistake
// 2.403 ADDED M792-795
// 2.404 fixed casting problem with xy move
// 2.405 added M673   turns on light programatically
// 2.406 added extruder unlock feature to M103 E1
// 2.407 changed the way the watchdog timer acts, instead of unprime, it stops extrusion and unlocks motors.
// 2.409 switched polarity on motor enable on hot head, 4988 is enabled LOW now
// 2.410 added W_RTD1 and W_RTD2 to toggle indicating start of motion and start of prime and unprime.
// 2.411 changed the G4 so P is millisecond and S is seconds with a 100 minute maximum wait
// 2.413 added M7 Txx to assign M7 output to mimic print/no print condition so you can run relay
//       when ever it is supposed to be printing.
// 2.424 revamped GPIO definitions to mimic hothead style to allow remapping to other pinouts - should
//       be transparent to user
// 2.425 added system SOAP, mcu type detection, uniq mcu ID
// 2.426 changed motor storage to an array of structs instead of indiviually named structs
//       (also in prep of expanding to 6 motors)
// 2.426 added G12 circular pocket and also circular boring cycle
// 2.431 merged gb's 424+425+426 with kjg's 426-430
// 2.432 converted Motors array indices to use an enumerated type (saved 8KB of code space and performance gain)
// 2.500 renamed 2.432 to 2.500 and put in repository
// 2.501 accel/decel bandaids
// 2.503 accel/decel changes from karl
// 2.504 added decel code to mirror accel ramp; added MOTOR_EN to motor struct, minor cleanup
// 2.505 karl added motion mods; soapstring mods
// 2.506 merger of 2.504 and 2.505 + HomeMotors fix
// 2.507 picked up things missed in 2.506
// 2.508 karl's updates for canned routine
// 2.509 greg's motion changes for coordinated moves across all axis, more scaffolding for up to 6 motors
// 2.512 CHANGED JOG ROUTINES TO MICRONS AND USES SCALE MULTIPLIER AND JOG VALUE FROM M795
// 2.513 changed z axis to HOMESPEED for random jogs.
// 2.614 major change -- motionQ, complete redo of motion (physics based const accel); removed timer drift on
//       moves; numbers bugs fixes (and probably some bug insertion!)
// 2.615 unwound stallsense check (was not wired and cause issues)
// 2.620 added new I/O for aux power (HSS) control
// 2.621 added new controls for all HSS and DDL selection
//december 21, 2016 moved to 429 and new motion logic from Greg Buchner,  looking good  :-)
// 7.001 initial version with optional 429 support
// 7.005 added:
//        -deferred motion que to allow key G/M codes to be processed in sync with motion without stalling the motion (such as M790)
//        -fixed bug with starting velocity of 2nd element in the mQ if the oldest (1st) entry had already started moving
//        -added motion/print metrics (M772 init/clear; M773 print out metrics)  shows accel/decel time, cruise time, and avg print speed
//        -cleaned up I/O spreadheet for 429 board and update #define for debug I/O
//        -added code to allow unprime to occur before the end of the move (enable with M227 P1).
//        -changed mQ startup condition to be either time based (50ms) OR number of moves in que (4)  we can adjust as needed
//        -most commands no do not block the mQ, so karl's handwheel should move a lot nicer!
//        -reinstated turboMode (now set to be OFF by default); fixed bug where the Farg on a turbo'd G1 was ignored.
//        -added remaining setup mcodes (rehoming, etc)
//        -abort now uses abortDecelerationRate for controlling motion stoppage
//        -simple code cleanup (nuking debug/unused code; clean up some GB_ ifdef, etc
//        -use the average vector speed for a move (including acc/dec effects) to set flowRate instead of cruising speed.
//        -fixed bug if tried to go past 0 on any axis
// 7.006 - { this rev intentionally left blank }
// 7.007 changes:
//        -fixed logic bug in _autoReverseAndPrimeMinTime (M227 Sxxx) ..
//        -added single char (^K/^L, ascii 11/12) to nudge the Z-table up/down via an extra timer (TIM13)
//        -M868/M869 now added to the cmdQue
//        -tweaked g2/g3 arc code to based segment count on arc distance (one segment per 0.333mm of arc travel)
//        -added ARG_P option to g2/g3 to force the number of segments (ie, P=5 will draw a pentagon
//        -added protection to gracefull handle the deferred queue getting full (rather than spewing an error)
//        -added code to very the centroid specified with the IJ in a G2/G3 is truly equidistant from the start and end points
// 7.008 changes
//        -change material flow during accel/decel ... using fudge_factor for now :-)  this is just to allow testing
//         of the concept.   a HACK for now, use M208 E0 or E1 to disable/enable the feature (default is disabled)
// 7.009 changes:
//        -made new change flow based on current speed during accel/decel to use speed instead of avg speed to set baseline flow
//        -fixed issue where fudgefactor was getting stepped on by multiple use of the variable.
//        -fixed bug where _LastExtrusionRate was not invalidated when the hothead was reset (if next job had a matching rate, no
//         extrusion parameters were sent.
//        -fixed bug in turboMode logic.   Rate index was not set causing wrong rate lookup (or invalid address).
// 7.010 changes:
//        -increased NUM_TOOL_OFFSETS to 22 (21 offsets, plus the 0 location to reference no offset)
//        -change readSoap logic such that any attempt to read the soap is throttle to at most one soap read per second to avoid overloading the
//         single buffer in the 407.  this is NOT a good long term solution, as the right solution should involve true protection of access to
//         to the buffer.
//        -tweaked the motionQ_stopAndFlushQueue() method to ensure any axis that can immediately stop does and it's timer no longer fires
//        -enabled use of REHOMING rate on second homing pass now the repetrel 2.888 supports sending it.
//        -changed M215/M216 to be homingDirection and homeDestination
// 7.011 changes:
//        -fixed bug in decel code for homing if homing motion didn't reach it's cruising speed before hitting the sensor
//        -finished cleanup of motionQ_execute motion (grouping non-homing code together).
//        -realigned timers and setup timers for all 6 axes (X=TIM2, Y=TIM3, Z=TIM4, A=TIM5, B=TIM6, C=TIM7)
// 7.012 changes:
//         karl's working version.....(changes to M3/M4/M7/M8 and reporting strings)
// 7.013 changes:
//        -added more info to M773 to dump queue usage max usage for serial buffers and queue entries
//        -added option to M772 (any S arg) to auto enabling dumping of metrics when an M30 is encountered
//         so now, just stick M772 S1 at the top of any gcode and the stats will be dumped at the end of the
//         program automatically.
//        -merged a few of the simple 10Hz entries to free up some systick slots
//         (heartbeat on and heartbeat off now one routing; soapstring management now one)
//        -added G10/G10 (unprime/prime) to match slic3r "firmware retract" option ...
//         this will not support "early" unprime.
//*        -changed control for "early" unprime to key off a negative unprime preamble
//        -altered mechanism to control how E values are dealt with and how unprime/prime
//         (ORIGINAL BEHAVIOR IS UNTOUCHED)
//         style is selected (see M229).  M227/M228 should be deprecated.
//        -head registration tweaked to also put a throttle on the request for all
//         the head specs along with the soapstring request to avoid overloading the canbus
//        -started adding support for G1 Ex Fx unprime/prime (part of "using E values")
//         NOT WORKING YET.
//        -(CRITICAL) changed units on pulsePerXXX for the extrusion to be pulsesPerUnit.
//         if ignoring E values OR if the E values are volumetric, then user should
//         enter the pulses_per_microliter for pulses_per_unit (NOTE:  this is a
//         change of value from the prior form of entry in repetrel....the new entry
//         is 100X the prior entry);  if E values are length, then the use should enter
//         the pulses_per_mm for the pulses_per_unit
//        -simplified handling of Tx only commands.  includes the following updated mapping to
//         provide a straightforward mapping to yoke 1 and yoke 2
//             T0-T5 will become T11 to T16 to match yoke 1  -- SAME AS CURRENT BEHAVIOR
//             T6-T8 will become T7 to T9 (for future undefined devices)
//             T11-T16 will stay T11 to T16 to directly match yoke 1
//             T21-T26 will stay T21 to T26 to directly match yoke 2.
//             (in reality, the code is set that ANYTHING about T9 is passed through unscathed)
//         OPEN: headOffset associated the second yoke needs to still be addressed.
//        -enabled flow change with accel/decel.  can disable with M208 E0
//        -increased cmdQue size to 100 (had been dropped to 50 to free up debug space)
//         NOTE:  it is FAR FAR more efficient to store commands as ASCII in the serial RX
//         buffer than in the cmdQue (88 bytes per cmd).....so we are better off using any extra
//         space in the RX buffer than in the cmdQue.
//        -increased DeferredQue to 20 (from 10)
//        -increased motionQ to 20 (from 10)
//        -fixed bug where resetting during a homing operation would cause "bad" motion
//         (G28Flag was not reset to "off" during a process reset.
//        -unregister all heads on a reset (when char '08' received, RM> is sent back for each
//         device that was registered at the time of the reset.
//        -block transmitting direct to a device can packets that are destined to an
//         unregistered device (which will back up the can bus).  broadcast address
//         are not blocked
//        -fixed bug where the fudge factor was being sent to the hothead the relative speed
//         difference for accel on a non printing move
//*        -at the end of a move, the head fudge factor is now restored to the users default value
//         to clear any effects of changing the fudge factor based on motion
//        -changed Serial ACK behavior.  added new non-acking command termination (#9);  changed
//         things so ACK is not send if the serial rx buffer is near full (ACK is deferred until
//         the serial buffer drains below the required headroom level (which can be set with M683).
// 7.014 changes:
//        -fixed host connection timeout which was not begin reset for immediate mode commands
//         (this was the root cause of why manually extruding would sometimes turn off seemingly
//         at  times.  now it will always get the full 30 seconds.
//        -added mechanism to scale the G1/G2/G3 motion motion rate (feedrate) by 0.25x to 4x.
//         scaling is set with M231 Sx and does so prior to any accel calculations, so motion is
//         handled properly.  it's a good candidate for executing with a '911'
//         NOTE: using M231 will ALSO cause the material flow rate to scale with the rate change as
//            the flow rate is function of G1 motion rate (ie E values are present or used)
//        -added mechanism to scale any non-homing motion motion rate by 0.5x to 2.0x.
//         scaling is set with M232 Sx and does so after to any accel calculations, so there are
//         still motion ramps, but the actual ramps will not match the requested constant accel
//         value set by the user.    it's a good candidate for executing with a '911'
//         NOTE: using M232 will only change the motion rate and will not directly change the
//            material flow rate.  Indirectly it is changing through (diff motion speed with same
//            flow rate will have a different apparent flow rate.
//        -added roundf to the MaximumTravelInPulses calc to prevent moves to the exact axis
//         max from exceed the max position in some cases.
//        -fixed bug where metrics "may" have gotten armed to auto dump on M30.
//        -remove redunant entry in 407 for NI_J13_P2 (PC5) and changed to INPUT_FLOATING
//        -rearranged the debug pins fro GB1-GB4 using PF11-PF14
//        -made homing pop off distance (re-homing) programmable via M233
//        -add M234 to allow setting of the current position in absolute terms (needed for rotary debug)
//        -added M235 to allow setting offset and perpendicular axis for rotary axes
//        -cleaned up rotary motion/printing code
//        -added FADAL_407 as a build target for Karl's CNC mill
// 7.015 changes:
//        -moved to arm-gcc 6.2 2016.q4
//        -fixed all new warnings from new compiler (some of which were legit errors!)
//        -merged in karl's changes from .012
// 7.016 changes:
//        -changed M660 to use ARG_H instead of ARG_Z for toolength
// 7.017 changes:
//        -added error code for PnP head for failing to home
//        -added M141 for temperature chamber control
//        -added M191 for "waiting for" chamber to reach temp ;  the waiting part is stubbed out as it is in M190
//        -changed M190 to at least set the bed temperature, but still does not wait for it to reach temperature
//        -changed revisionString to include processor type and UID
//        -fixed final position for case when aborting during a homing move
//        -moved resetProcess() higher in main.
//        -add M775 to dump current state of a bunch of variables
//        -started work on Default_handler recovery.  Attempts to save key values to flash.
//         M776 added to send and/or erase crash log or to trigger a fake crash.
//        -cleaned up code for init of _hostTrafficReportingRate (no functional change, still every 5 seconds)
//        -added better error reporting for gcode parser
//        -reinstated histogram printing (in M775)
// 7.018 changes:
//        -added trap for serial overrun
//        -dropped baud rate back to 38400 due to serial issues
// 7.019 changes:
//        -shrink queue sizes
// 7.020 changes:
//        -move DDLight to HSS_AUX_PWR5
//        -added controls for 4 new "DRAIN" i/o (M609 to M612)
// 7.021 changes:
//        -disabled irq around setting of charsInRxBuf in GCHAR() to prevent collision with same variable being
//         changed in ReceiveCharacter()
//        -added more checks on the serial size (underrun in GCHAR())
// 7.022 changes:
//        -added measurement of slice timing (see M778/M779) (compiler option!)
//        -tweak controls for loading timer (DIER)
//        -reverted flow control to simple continuous stream based on "F" feedrate
//        -added new serial processing method to separate the Rx of a char and
//         the processing of it (minimize time in the rx ISR) (#define NEW_RX_METHOD)
//        -added option to move some "slices" to the foreground, but only if NEW_RX_METHOD is invoked
//         (#define RUN_IN_FORGROUND)
//        -add protection (irq_off/on) around a few key variables in the serial routines
//         that are changed by two diff routines that could step on eachother if interrupt
//         aligned.
//        -changed resportSingleLine to build its string in a private buffer as now the
//         routine can run in the foreground and other routines could have stepped on the
//         public/global buffer it had been using
// 7.023 changes:
//        -fixed bug in M234 where args were not read to set the position
//        -quick hack to limit the canbus TX_FULL error reporting.  if it shows up
//         once, the systems is a messs (or has no heads).
// 7.024 changes:
//        -prepped (not enabled yet) check of the motor fault signal and auto aborting if detected.
//         code is NOT finished.
//        -all HSS/DRAIN are set to their init state after an abort.  this should be
//         replaced with a user defined "safe state"
//        -set default for _varyExtrusionRateWithAcceleration to FALSE. should have been done for 023
//        -tool change did not cause motion Q to empty ... which in turn kept and
//         unprime/prime pair from occuring on the tool change
// 7.025 changes:
//        -fixed bug where needToUnprimeDuringMove was not being reset in isAutoPrimeoOrUnprimeNeeded()
//         (led to unprimes occuring with no prime() after word ... only happened with negative preambles
//         changed pending acknowledge to queue up more than 1 pending ack
//        -fixed bug where "911" immediate commands did not get processed if the motion queue is full.
// 7.026 changes:
//        -changed serial input to now have 3 buffers.  first is a small fifo to hold raw chars from
//         the UART ISRs.   a new "hot" character (9) is enabled to indicate next command is urgent,
//         similar to the M911, except not the command can jump the serial queue as well.  if a 0x9 is
//         detected, the next command will be processed as an immediate command.  the two additional
//         buffers are used to split the urgent vs normal commands.  ack flow control still only
//         references room in the normal buffer.  no backpressure is provided for the urgent buffer, so
//         it is possible to overrun it (an error is sent if an overrun occurs).
//        -make sure HostConnectionWatchDog only counts down when command and motionQ are empty.  reset
//         after any char received via UART.
//        -made M683 official
// 7.027 changes:
//        -beginning work on 407 to hothead comm watchdog
//        -fixed acknowledge bug on urgent commands.  now a blind acknowledge (no flow control on fifo
//         fullness
// 7.028 changes:
//        -added (missing) parens to sqr(X) macro
//        -added (temporary) option to select using average move speed for setting flow rate.
//         (M208 S0 or S1 to disable/enable using average.)
//        -fixed bug where cmdType in cmdQue was not set for immediate commands.
//        -adding blocking based on ExecuteImmediateFlag==2 to prevent G28 from being messed
//         with (by cmdQue[1] being changed, ruining the multipass G28)
//        -changed code in protectedIncrement/Decrement to work with gcc 6.2.
// 7.029 changes:
//        -finished installing M41 and M42
//        -added optional code to play with the PWM on timer8....prepping for lasers (using M42 to control
//         pwm as a temp HACK -- this will get its own mcode in the next release). default is 20K bandwidth
//        -changed G70-G85 to be SINGLE_STEP commands when added to cmdQue
//        -more sideband serial code added
// 7.030 changes:
//        -fixed (fatal) typo in addition of sideband code. sideband now available on the DB9 (UART4) for
//         SYS30_407
//        -reinstate/cleanup direct buffer code (not quite ready for use)
//        -added M621 for start of CO2 laser control code -- complicated Mcode (many options)
// 7.031 changes:
//        -added argument range checking to M683
//        -moved CO2 Laser from Yoke 1 Aux 2 to Yoke 4 Head 1 (and 2)
//        -debugged direct mode (laser raster data)
// 7.032 changes:
//        -split M621 into M620-M623 for clarity and ease of use
//        -retired M771 (old laser raster control)
//        -expanded mailbox system to handle all defined devices on 4 yokes + 2 hotbed
// 7.033 changes:
//        -cleaned up all MCODEs to make sure there restore the current tool to the PersistantHeadAddress
// 7.034 changes:
//        -fixed two issues related to deferred MCodes that occur after single GCodes that expand into
//         multiple moves (G2/G3/G28G83, etc). the fundamental issue was the deferred MCodes were tagged
//         to follow the original GCode, but the code was not tracking the last move in the series to
//         force the deferred commands to occur immediately after the last command in the series. the fix
//         was to add an addition global flag the must be set when the last move is being added via
//         motionQ_addCommand().  the flag should be set for all single commands as well.
//         the second issue is the case where the the last move in the series (or any single GCode) resulted
//         in NO move (ie, a move to the same location as the current position).  If deferred commands were
//         tagged to this command, then the deferred command would not execute as the code complete tossed
//         any non-moving moves (they were not added to the motionQ).   The fix involves allowing the
//         non-moving moves to be added to the motionQ such that the deferred commands can remain in sync.
//         the non-moving moves are blocked from execution (ie, timers are never set).
//        -moved G28 into the same structure as the CannedCycles
// 7.035 changes:
//        -made all MCodes (except M6) use a local variable for the tool number (allow argT
//         but the argT never changes the current active device/head (even momentarily)
//        -added device target for "waitingFor" for temperature separate from currentDevice
//        -added M780 to enable/disable of XYZABC position reporting (split from M759)
//        -added M781 to allow writing device optionBytes for hwType and hwRev outside of bootloasder
//        -changed HostCommWatchDog to walk through list of registered heads and send out the same set
//         of "stop" commands as before (instead of T10 broadcast, which does not work beyond yoke1).
//        -added motion_controller to heads watchdog (1 Hz ping to heads letting them know motion controller
//         is still alive (heads now need to add receiving side and their own watchdog
//        -added ability to enable/disable a given sensor input (set either act_high, act_low, or disabled)
//        -added simple routines for each of the 1/10/100/1000 Hz loops for grouping simple/fast routines into
//         one routine to free up slots.
//        -added RI> strings for M190 and M191 to let uses know we are ignoring the "wait" for temp on the hotbed and chamber
// 7.036 changes:
//        -fixed M761 (non-read soapstring options had been accidentally cut out)
//        -check flash size and then set the soapstring and crashlog sector based on the top sector
//        -remove crashlog from flash for 512KB flash parts (insufficient space)
//        -check RCC->CSR on boot up to know source of device reset (POR, NMI, SW, etc .. better than breadcrumbs)
//         if nRST (reset switch), SW reset or IWG (independent watchdog) is detected, the a crashlog
//         is saved to flash.   The log can later be displayed with M777 S1
//        -added M761 S7 to dump soapstring config (size, address, sector)
//        -added M761 S8 to dump crashlog config (size, address, sector)
//        -fixed bug in singleLineReport to key of NUM_PHYSICAL_DEVICES and not a harded value (13)
// 7.037 changes:
//        -enable built in USB hardware in FS (full speed) mode.  HS mode not available due to pinout choices and no PHY
//        -drop sideband.  karl will now have repetrel maintain a second comm string and will inject the sideband as
//         urgent commands to the 407/429.
//        -now only one active serial port (as determined by where there last ping came from).   all ports are listened
//         to for a ping, but if not from master, data is ignored
//        -abort and ping now flush input and output serial buffers
//        -minor cleanup around 407 vs 429 IO (mostly gpio.h)
//        -M782 added for "printing Air" (drops all can traffic)
//        -enable independant watchdog with a 5 second timeout (include LSI calibration; IWDG reset in 1Hz loop
//         so plenty of margin in case a slice overruns.
//        -added setupDBGMCU() init routine to control peripheral (ie, IWDG and TIMx behavior when puased in the debugger
// 7.038 changes:
//        -fixed bug in M718 (error from dirty.bit conversion)
//        -updated laser controls to work with latest wiring plan
//        -switch diode laser HSS1 enable from using dutyCycle (as hoped) to use Temp
//         (much line an M104 would due).  This switch was due to the dutyCycle control
//         code being removed from the head code.
// 7.039 changes:
//        -major rearrangement of all the timers in order to move the local CO2 laser
//         pwm control from TImer1_CH1N on PA7 (DRAIN1) to Timer5_Ch2 on PA1 (J16-7)
//         on 429 board motion motions are now TIM9-14 (moved from TIM2-7); master
//         motion still on TIM1; LSI calib still on TIM5; jog move from TIM13 to TIM6;
//         slice timing now TIM2 (was TIM8) (and no long shared with laser pwm.
// 7.040 change
//        -added explicit pin init value to gpio #defs (had been an init path from the
//         spreadsheet interface.  that path has been altered to add to the #def rather
//         than adding the explicit pinWriteBit() after the pinInit()
//        -reworked the previously included pin_init_4xx.c files to be properly installed
//         project files.
//        -updated to gnu arm gcc 2017 Q1
// 7.041 change
//        -no code change...only updated project files to resolve coocox issue
//         from reverting back to 1.7.8 from 2.0.6(beta)
// 7.042 change
//        -added code to measure ISR rate.  determined response rate is 1usec (time
//         to respond to TIM2 updated, handle the interrupt, toggle a pin, and exit.
//         (max rate 1 million interrupts / second.
//        -starting another pass at getting dynamic flow rates working (for using E-values
//         accel/decel etc)
//        -fix casting issue on LaserWatchdog being passed to the SetLaserPower routines
//         (from uint16_t to uint32_t)....would screw up on moves > 52 seconds otherwise.
//         now limited to moves around 600 seconds.
//        -send out LaserPower setting
//        -added a hack/patch in initializeLaser() to set the _g4DwellTimer to 100 to allow
//         time for the diode laser power supply to be turned off (wait for the effects
//         of changing the switch control by temperature of the HSS to propogate to turn
//         off the supply, otherwise a short duration, but full power burst could happen
//         at the end of a program.  THIS CAN BE REMOVED once the Laser
//         head code can be reworked to have more direct (and immediate) control of the HSS.
//         THE PROBLEM STILL EXISTS FOR A RESET!
// 7.043 changes:
//        -fixed bug in 429 pin defs where the START pin did not get defined properly due to
//         an empty cell the pinout spreadsheet for (missing "IN" for "in/out" column)
//        -change the "START PRESSED" message to the equivalent of a
//         SendCurrentMcodeExecutionNotice() for an M1 command (which now means an M1 will
//         act as a stunt double for pressing the start button
//        -changed "start" button debounce routine to a history-shift based instead of counters
//         a little cleaner interface and easy to check for vaid rising of falling edge
//        -added a method to test the start button the sys30.
// 7.044 changes:
//        -replaced a call to SystemInit() that was inaverently deleted in 037
//        -added trap in main to check if clocks were set up properly, if not, turn on ERROR
//         LED.  SetSysClock() in cmsis_boot/system_stm32f4xx.c was modified to set a global
//         flag (ClocksFailedToInitialize) to 1 if failure occurred.  Also, if clocks are not
//         right, the watchdog rate is slowed down./
// 7.045 changes:
//        -set major release to "3"
//        -fixed bug in the deferred command processing
// 3.046 changes:
//        -fixed bug with multi-pass (cannedCycle) moves when issued at immediate (911)
//         these commands needed to block the serialProcess/commandProcessor from stepping
//         on the cmdQue[1] location AND the ExecutionPtr variable until the full canned
//         cycle is complete.
//        -removed IWDG() as it may have been interfering with the dfuse loading
//        -reordered the processing for "kill/reset" (abort) conditions to do the following:
//         (prior code reset all counters/pointer, then tried to stop motion, but that relied
//          on some of the counters that we reset, such as UnprimeTimer;
//            -stop (gracefully) current motion, including unprime
//            -flush all serial and command buffers (stop processing commands)
//            -reset all counters/pointer/etc.
//        -added a USB DeInit call prior to the jump for the Dfuse bootloader (M766)
// 3.047 changes:
//        -fixed bug where arc commands would not immediately stop on an Abort (the state
//         machine for the arc was not cleared when the abort began.
//        -fix for G2G3 Sweep angle that had cause some arcs to overdraw
//        -added attempted fixed for USB hang issue after using Dfuse loader (interrept for
//         TXFE was constantly firing
// 3.048 changes:
//        -fixed mistake in init if motor params for USING_16A_IO (wrong polarity for MOTOR_EN)
//        -fixed init bug in laser code (unitialized pointer getting accessed) which indirectly
//         prevented FLASH from being erased (soapstring write)
// 3.049 changes:
//        -added M236 for setting deviceType and deviceRevision option bytes
//        -fixed bug in serial stream where an underrun could occer during a job kill
//        -fixed bug were a direct mcode generate prime could cause an error to be generated
//         for the motionQ being empty
// 3.050 changes:
//        -modified change of comm port master to include ABORT char (along with PING char)
//        -fixed bug in G28 where the CannedCycleLastMove was being set TRUE one step in the
//         cycle too soon.  This broke the processing of deferred commands.
// 3.051 changes:
//        -picked up a change in G12 from Karl
//        -fixed source of the UNDERRUN errors that could occur during a kill/reset:
//         1)moved ResetProcess call for cleaning up after a kill/abort to the systick routine
//           so that all the variables that get reset occur outside of the execution of any slice
//         2)add blocking of execution of a few methods while an abort was in process
//           and in the async code running in the while loop in main()
//        -dropped RX and TX serial buffers to 6K from 8K as getting tight on memory (just to be
//         safe for the stack
//        -re-numbered enums for buffer_t and masterPort_t to make 0 not a valid choice
//        -added a temporary dump of a lot of system variables to help with underrun bug debug.
// 3.056 changes: (rolls up karl's 052-055, plus new items)
//        -fixed G28 bug for homing away from zero AND for all cases where home offset (M216) was not 0
//        -G28 now resets all motion offset indices and sets the G92 offsets to 0.0
//        -changed M216 to be in "units" instead of "pulses"
//        -fixed bug in Z_HOME pin def for engines/sys30.   should have matched Z_HOME1
//        -defined START button on SYS30 (PF5) (START button now works like a 16A)
//        -added #define DUMP_ON_START_BUTTON to force a dump of key variables on START press
//        -flipped back to keeping zero length moves out of the motionQ and fixed prior
//         issue with multi-part (i G2, G83) commands not working with deferrded commands
//         if the last move in the series was zero length.
//        -fixed bug with new (all axis) ReportXYZLocation printout. now all installd axes report
//        -fixed bug for full circle G2/G3(fmodf call was in wrong place).
// 3.057 changes:
//        -moved DUMP_ON_START_BUTTON to a GB_STRING_DUMP_ON_START to allow M797 to turn it on/off.
//         also, crashdata is sent to flash instead of to the uart. (use M777 S1 to dump flash)
//        -added USB flush input buffer to ResetProcess()
//        -added code to block chars in receiveCharacture during an abort
//        -changed heartbeat rate for diff conditions (fast during abort)
//        -changed saveCrashLog to flash to use format table directly
// 3.058 changes:
//        -moved DeInitLaser ahead of initGcodeSeq to ensure laser is off before the swReset
//         request is sent to the heads and try to shut off laser power supply at the very start
//         of the abort sequence
//        -change default laser pwm to 50K ... (max is 64K due to uint16_t size, had been 100K but does not fit)
//        -add Farg to M780 to dump calculated flow rate as an option in the reportXYZLocation call
//        -changed sendchar's behavior in the case where there's no room in the transmit buffer.
//         if full, it will try to open up space for one char by calling PrintCheck.
//        -added PLATFORM_STRING based on compile #define for use in announce string
//        -fixed bug in M6 mcode for the sendExectutionNotice....needed to move in front of the call to
//         InvalidateAllCmdArgs for setting up the immediate move
//        -add auto init of the system from the soapstring.  any string starting with an '*' is processed
//         as a command.  no limit on the number of commands.   these commands are processed before
//         commands are accepted from any of comm ports (single char commands, such as abort, will
//         still be processed)
//        -reworked .coproj and BIN2DFU.bat to handle muliple targets and automatically insert the
//         target name and version number into the output file names
// 3.059 changes:
//        -added C arg to M106 to allow setting duty cycle range (for slic3r support of 0 to 255)
// 3.060 changes: (from karl)
//        -changed to G83 and processCannedCycle for 83
//        -fixed M660 to prevent toolindex from going past array length
// 3.061 changes:
//        -changed reportXYZLocation to include display the last requested vector rate (ARG_F) but in sync
//         with the motionQ.    will also display the actual vector rate for current move being executed in the
//         motion queue (this is the rate for that move based on move distance divided by expected move time --
//         so average speed for that move).  Also changed the reports to limit float printing to 3 decimal places
//         (so 0.001mm resolution of 0.001mm/sec resolution)
// 3.062 changes:
//        -allow G0 commands to use an F arg to set a non persistent one time rate for the move.  This is intended
//         to be used with the Wx hand controller to adjust the time of each move such the moves can be stacked
//         together by the motion queue (it outputs a new move at most every 93ms).
//        -_printAir now resets on an M30 (as well as a kill/reset)
// 3.063 changes:
//        -fixed bug in travel limit checking for homing (wrong homing flag used to prevent error message)
//         resulting in erroneous clamping
// 3.064 changes:
//        -changed the setting of lastRequestDest in last step of G28 to take into account offsets
//        -changed G928 to more gracefully handle the homeToInfinity case ... it's now symmetric to the homeToZero
//         case in that instead of moving the '0' towards negative infinity, it moves the homeOffset towards positive
//         infinity.  This may not be the desired behavior in the end, but this is more graceful than it was !!
//        -added code to only allow G928's to run when motionQ is empty (currently sent by GUI as "urgent" and multiples
//         in a row are stepping on the motion controller)
// 3.065 changes:
//        -changed ReportXYZ to only send text for axes that have changed position.
//        -changes add arc type moves to use the same # segments calculate (G12/G13 now use same calc that was used
//         on G2/G3).   now ALL commands can also override the number of segments with the ARG_P to set the number of
//         segments in the polygon.
//        -fixed M0, pause_on_next_move, and pause_on_next_layer to use a semaphore and not a counter
//        -added ability to have a Z change (and define thread pitch) to G2/G3
//        -ARG_S no longer used to force number of segments on a G2/G3 (was ARGP)
// 3.066 changes:
//        -fixed bug in gpio.h that affected any "INIT to 1" pins (two of the #def shift values were wrong)
//        -fixed bug where toolOffset[0] was not taken in to account where setting up a homing move when already
//         on the home sensor (it was expected that ALL offsets would be 0 at that time, but one wasn't
//        -ProcessG28MultipassHoming - added error checking for homing an axis with that has no home sensor;
//         tweaked step order to include explicit step for setting home position after first home; cleaned up comments
//        -changed M91/M215 to auto update the HomeDestinationInPulses if either changed and deprecate M216 to no longer
//         directly set the HomeDestinationInPulses as it MUST be either 0 or MaxTravel (based on homing direction) so
//         dangerous to allow redundant entry of the same info.
// 3.067-3.069: (fork from Karl) built on top of 3.065 changes:
//         -allow DDLight to be mapped to a diff HSS
//         -changed homing for Z away from zero (many changes), similar to 3.066 in function.
// 3.070 changes:
//         -reconcile Karl's fork
//             - allow DDLight mapping to another HSS (uses DDLightHSSIndex).  In the future, may want to make a generic
//               mapping for all HSS by function.
//             - adopt 3.066 method for Z homing (3.069 broke homing away from zero for non-Z axes and broke
//               broke homing multiple axes if one axes was already at home)
//         -change DDLFunction to handle the S1 case (could get in a state where the light won't respond)
//         -after self init; set POSITION/Q_POSITION/lastRequested position as if a homing seq had occurred (fix
//          issue where these were incorrect for the first move in a home away from zero scenario
//        -RESPONSE_HSS on HSS6 in code, but wired on HSS4 (chamber fan is on HSS6). moved RESONSE_HSS to HSS4
//        -add #defines for other (used) HSS (ie, CHAMBER_HSS is on HSS6, VAC on HSS9; AIR on HSS10
//        -fix bug in M601-M612 where array index could exceed array size depending on compile TARGET due to
//         varying number of HSS per target
//        -fix bug were autoXYZ reporting would not update after booting
//        -revert deprication of M216 and changes to M91/M215....so back to independent homeOffset and maxTravel
//        -fix bug where HomeDestinationInPulses was not recalculated if M92 was issued.
// 3.071 changes:
//        -G38 touch probe added for 429 based systems (TBD pin for sys30/engine)
//        -add code to block absolute moves (except G928 is allowed) until homing is performed (per axis block);
//         block is reinstated after M84 FUTURE: if we had a 12V/24V power monitor, then at loss of power (ie, EMO pressed)
//        -resolved L1/L2 silkscreen not matching layout(and code) issue.  changed code to match silkscreen. karl
//         to have schem/layout updated to match
//        -renamed pin ERROR to ERROR_LED to avoid conflict with typedef of same name
// 3.072 changes:
//        -clear ALL offset indices to 0 at start of G928
//        -change motorFault routine to report with new >MF: string and halt motion until an abort/reset
//        -add 2 pins targets for use on sys30 for G38 probe (P1=W_RTD1/P2=W_RTD2 on top edge connector)
//        -remove SYS16A_407 target, FADAL_407 target (now only USE_HYREL_IO and USE_HYDRA_IO
//        -added control signals for SYS30 LED(response light) and ALARM(buzzer)  on switch panel
//        -added M676 for controlling chamber fan
//        -added M677 for controlling alarm buzzer (sys30)
//        -added M678 for controlling laser x-hair
//        -added M679 for controlling exhaust fan
//        -added M685 for controlling air assist
// // 3.073 changes:
//        -fix bug in touchProbe to ignore "touch" reports from the canbus when not 'armed'
//        -after homing Z, tooloffset0 is set to the Z home offset position (per karl)
//        -reworked motorFault logic to allow relative moves AFTER a reset/abort until all faults are clear
//        -remove HYRES compiler target
//        -reworked HSS controls....now based on function select which allows mapping any function to any
//         physical switch.   can also set the polarity, timescale, period, and dutycycle for any switch
//        -added quick hack to bypass check for _laser.device and just send data to heads
//        -added WDG for reportXYZ
// 3.074 changes:
//        -added Karl's new M660 code (swap Z of H, then H for T
//        -added an M114 to force a report of the XYZ Location
//        -M780 issued with no args will report the XYZ location
//        -renamed master HSS control to M619 to not alias with the prior M601 (which should be deprecated)
// 3.075 changes:
//        -added G2.1 and G3.1 for spiral-in (uses P for pitch; C for number of passes)
//        -changed G12/G12 to handle spiral (ARG_W=1);
// 3.076 changes:
//        -added M630 to allow direct control of probes
//        -changed G38 to allow control of can probe directly
//        -fixed bug in singleLine reporting (counters were bytes, not ints)
// 3.077 changes:
//        -change M630 to have a repeat count and merge ONE_SHOT and CONTINUTION (count=0)
//        -change M630 control of BLTouch "commands" to an enumeration instead of setting the pulse width
// 3.078 changes: 2018/05/10
//        -remove scanner attached message
//        -fix bug in motionQ_addCommand for blocking absolute moves if not homed.
//        -changed M211/M212 to not check if motor is installed before updating values
//        -remove redundant call to motionQ_init at top of initialMotorParams.   should not have been
//         there and resulted in a reference to a NULL pointer with prevented FLASH from being written
//         (so soapstring loads failed)
//        -updated reportXYZ to show "FAULT" if the motor is currently faulted or -999.999 if the fault is clear
//         but the motion controller has not been reset
//        -now processing all deferred commands on an abort instead of flushing them
//        -tweak fault recovery logic
//        -added a >ER: error message when a motion fault is detected
//        -fixed bug in setting curr->speed in motionQ_addCommand when sending in a raw rate
//        -added testing for Limit2 in a symemmetric fashion to "fault'.
// 3.079 changes: 2018/05/11
//        -re-fixed bug in setting curr->speed in motionQ_addCommand when sending in a raw rate
//        -fixed ifdef position in reportXYZ
//        -fixed bug in one-shot override of G0 speed (allows G0 Fxx).  F is not an overall feedrate change
//        -added reset seq to M766 before jumping to bootloader
//        -change M679 to control VACUUM (def to DRAIN1) instead of EXHAUST_FAN (no def MCODE for EXHAUST)
//        -added option for a 'TWEAK' revision (single extra char) ie.  3.079b
// 3.080 changes: 2018/05/13
//        -change period resolution to 0.01sec for probe rearming (M630)
//        -added igaging support for 35-12x style dial indicators
//        -changed M721/M722 to not set timers when in "I" immediate mode
//        -add EMO check using the former C_L2 pin (check at 100Hz, send out message) -- still be
//         be added:  action to taken when EMO proessed (ie, flush queues, force rehome, etc)
//        -recalc Q_POSITION and Q_LastReqPos in initGcode (for after a reset).  issue was after a reset/abort/kill
//         during motion that had offsets, then next incremental move would be based off WQ_LastReqPos which was
//         based on old offsets (which had since been cleared after the reset/abort/kill.
// 3.081 RELEASE: 2018/05/18
//     a - reworked motion errors messaging to repetrel
//     a - moved EMO detect to C_L1 fo HYDRA systems (none for sys30)
//     a - remove blanks in reportXYZ and probe reporting messages
//     b - started G38 changes to multicycle beast
//     b - send a temp of 0 for heads reporting over 900 degrees  (non-heated heads)
//     b - add delay (1-sec) before sampling any sensor bits (ie, LIMIT2, FAULT)
//     b - started code to delay motion start if MOTOR_EN is not active at move stasrt.
//     c - add delay to motion execution if one or more of the involved axes motor_en is not
//         yet enabled
//     d - avoid unhomed absolute move it dest = last_requested position (no move anyway)
//     d - added backdoor C arg to M719 to allow heads to not time out when using debugger
//     d - added M631 for initial test of pick and place head.  this will be expanded in the next release
//         (adding M632 as well)
//     e - rev'd for release on 06/01/2018
//     f - fixed bug in motionQ_addCommand left behind when debugging arc speed change issue
//     f - split M631 into M631/M632...one for data and one for control of PnP
//     f - released on 06/06/2018
//     g - turned off KILL_EQUAL_TIME_FOR_PULSE_GEN to allow higher ISR rate
//     g - started perf improvements on ISR (TIM1)
//     g - remove check for deviceRegistered before enabling/disabling lasers (and inkjet)
//     g - fixed bug in M6() where a a check for a valid headIndex referenced NUM_FIXTURE_OFFSETS
//         instead of NUM_HEAD_OFFSETS
//     g - increase CentripetalAccelRadius from 0.015 to 0.020
//     g - released on 06/10/2018
//     h - started coding diags for 102207 board (remove by commenting out #def for HYDRA_DIAGS)
//     h - changed motion Abort reouting for himing to reset the Home position after the abort to the
//         homeOffsetPosition AND to only do that for axes that had been part of that homing moving.
//         also, update the ToolOffsets[0].ToolLength if the Z axis was part of the move (same as a
//         completed home cycle). Prior code just set to 0 AND did all axes.
//     h - added status report of homed/unhomed condition of motors at each connection strings and
//         before/after a homing sequence (for axis that are part of the move;
//     h - released on 6/12/2018
//     i - skipped
//     j - added check for after home pop-off move to alert user if move did not leave sensor (message is not
//         sent if pop-off distance is 0 (per Karl's request)
//     j - added M106 Ax to override default diode laser cooldown time.
//     j - released on 6/23/2018
//     k - fixed bug where ULTIMUS_HSS did not get a default pin
//     k - enabled HEATER_DUTY_CYCLE_CONTROL_RESTORED as defaults (affects M620 - initLaser)
//     k - force T arg to be required on G38 if canProbe selected
//     k - add T# to probe report (T0 == system for local probes)
//     k - allow laser watchdog to be sent as ms or sec (pulled from 4.x)
//     k - released on 7/1/2018
//     l - skipped
//     m - completed diags
//     m - added "HYDRA_DIAGS" target
//     m - added support for second canbus (working in a single device id address space).  when a device registers, a
//         breadcrumb is kept indicating which canbus it's on.   All future traffic directed to that device is sent only
//         to that canbus.   however, non-physical address (broadcast, aliases, etc) are sent to both canbusses
//     m - fixed bug in low level can routine checking fifo status.....which could have led to deadlock in some cases
//     m - added a mechanism to kill any packets stuck trying to transmit for 5ms (about 15 retries). This could have
//         occurred when removing a device from a running system or when NO devices were installed.
//
// 3.082 RELEASE: 2018/07/14
//     a - tweaked error hanndling in diags
//     a - released on 7/15/2018
//     b - fixed DIAG_STRING_LENGTH ... had trimmed it too short before the last release.
//     b - minor log file format cleanup
//     b - add approxiate voltage on net for CheckViaAdc failed cases
//     b - change number of steps tested to 100 to 'hide' rollover potenial on step pulse average time (8D for hothead will
//         fix this)
//     b - added G84 tapping command (highly leveraging G83)
//     b - fixed bug in G83 where first move after clearing was a rapid move to the first incr position (should not be a move there)
//     b - added check+errorReport+punt if trying to do a G83 with relative coords enabled (G91)
//     b - added error meggage to user if neither X or Y is specified in G83
//     b - added M629 to force repetrel to open a log file
//     b - added an M30 execution notice back to repetrel wen M7734 is complete
//     b - fixed bug in checkViaAdc (nomVoltage and nomValue were switched)
//     b - released on 7/21/2018
//     c - fixed bug accidentally introduced in 082b which prevented the execution countdown timer from reaching 0 in some cases
//     c - released on 7/23/2018
//     d - made M140/M141/M190/M191 work with a T arg to select the hotbed(chamber) of choice.
//     d - T13/T14 command to set persistent hotbed (T0-T12 to select persistent devices)
//     d - enabled use of waitForTemp Mcodes M190 (hotbed) and M191 (chamber) to go along with the
//         already enabled hothead (M109) wait command.  each leaves the wait state slightly early to accommodate
//         the slow glide up to the final few degrees) - hothead 1 deg early; hotbed 3 deg early and chamber 5 deg early)
//     d - fixed issue with UltimusHeadAddress ... PrimeThenRun and Unprime() only checked for a non-zero head addr and
//         not explicitly matching the UltimusHeadAddr with the currentOutboxPtr->device;
//     d - pulled M783 back from Hydra4 to allow setting the UltimusHeadAddr.  the "off state" is to set the device
//         address to the unplugged position (user can enter 0 and the code maps it.
//     d - pulled M7 back from Hydra4 which only sets coolant on/off an no longer can set the
//         UltimusHeadAddr
//     d - added G84.1 command to allow user to specify a single linear and single tapping axis
//     d - changed sendRevisionString() to use getUIDString() method
//     d - added CAN_MSG_STRING_SEND canbus packet for the laser power supply display module
//     d - released on 8/2/2018
//     e - fixed bug in SetLaserPower where is checked for the 103 bd revision ... should have looked
//         at Major and not Minor revision level
//     e - added argL to M719 to all setting laser string reporting rate
//     e - released on 8/3/2018
//     f - added - PIN_INIT_HIGH to CO2_LASER_PWM pin definition
//     f - added extra line to break up log file when printing STRING_SEND in Hydra_can.c
//     f - revert local laser device id back to 41 (was flipped to 16 for a while)
//     f - change default PMW polarity for local control (M620) to active low;
//     f - released on 8/3/2018
//     g - fix bug in HYREL target for the new can1/can2 code (can2 pins were not initialized properly)
//     g - completed code for new "WaitForTemp" style with more options for selecting the actual temp to wait for (independent of set temp)
//     g - added HYDRA_DIAGS version number to diags log file
//     g - released on 8/7/2018
//     h - add W and S options to G84 and G84_1 to allow turning on/off a HSS (S) of ms duration (L) at top of each stroke
//     h - fix 3 bugs in M631 (two error messages and setting page to ARG_H instead of ARG_P)
//     h - diags - change BOOT1 error message
//     h - M632 - change cmd S=7 to use V for set vac state instead of D
//     h - tweaked >PP & Heartbeaat reporting string format
//     h - released on 8/11/2018
//     i - add waitFor to avoid back to back motion commands to the PnP head and fixed other waitFor issues (non temperature ones)
//     i - released on 8/12/2018
//     j - added CAN_EVENT_PNP_MOTION_BLOCKED event message from PnP head to signify absolute motion was blocked due to unhomed motor
//     j - released on 8/13/2018
//     k - added timer to keep from checking for motorFaults after EMO is released to avoid spurious faults
//     k - released on 8/14/2018
//     l - worked on spiral bore 8/30/2018
//     O - karl release with changes to mult-cycle commands
//     p - added M628 for edge detection reporting (using same  signals as touch probe EXTI)
//     p - archived on 10/8/2018
//     q - revert part of karl's 082O change in mulit-cycle that broke deferred Mcodes (2 places, comment: REVERTING)
//     q - revert karl's 082O change that forced the laser watchdog to 10,000 ms instead of request time (broke fix pulse generation;
//     q - added optimization to reduce calcs for motion step timing (precalc some terms)
//     q - use master motion timer to ALSO be the timer for the dominant axis, saving a lot of ISR calls and some extra calcs.
//     q - added mode (M229 Dx) to use Motor C timer for motion coordinated E control and sending "STEP" commands directly to head
//         the STEP can be sent as a no data can packet and/or via the C_STEP pin.
//         WARNING:  prime and unprime commands are still sent via the canbus only (the rapid pulse stream will not appear on the C_STEP pin.
//         user can not independently select:
//              - USE_E (take value from gcode) or IGNORE_E (calc value based on layerW, layerH and distance);
//              - when USE_E==1: either absolute E values (M82)(default) or relative E value (M83).  abssolute will be more accurate
//              - choose source of E_step for extruding: pulse stream created by hothead; set pulse by pulse as can packet; from C_STEP pin
//                (using either of the latter two will also use the resources for the 6th motion axis: C)
//     q - made M756, M757, M758 SINGLE_STEP as they affect flow calc which needs to be updated before adding moves to the queue
//       - added canbus_e_steps to _metrics to correlate filament used to slicers estimate.
//     q - archived on 10/17/2018
//     r - add M104 Cx for enabling chiller mode
//     s - fixed issue where can packets were being sent during ISR calls which could collide with SYSTICK based calls (realistically, the
//         only collisions oould be between setting laser power during RASTER or direct send of the san step command. (this was occurring
//         approximately 1 out of every 1,000,000 can packets.  Now the former ISR calls are replaced with a flag/counter of number of packets
//         to send and the actual packets are sent during SYSTICK calls.
//       - added separate method for sending pre-canned packets (laser and step)
//     t - did a little cleanup/optimization in the can transmit area
//       - archived 11/1/2018
//     u - fixed bug in motionQ_execute() where DominantAxisPtr->currentVelocity was referenced before being set
//         which meant DominantAxisPtr->PulsesForNextMs was incorrect for the first ms of a move
//       - add new method to deal with "pause at the end of the move".  It will pause now at the next move where
//         motion can safely stop based on allowable accel (DOES NOT USE ABORT DECEL or non-homing moving)
//       - archived 11/4/2018
//     v - reworked motionQ_stopAndFlushQueue() to better cover the cases where an attempt is made to stop the current move
//         when there's not enough room to safely decel. (DOES NOT USE ABORT DECEL or non-homing moving)
//       - changed pause to correct for an issue with exit velo of current move when pausing
//     w - fixed issue from laster pause/abort code not interacting properly with the unPrimeTimer.
//       - archived 11/8/2018
//     x - removing "NUKE" crufty code
//       - reworked from of the security code (added secure list of mcu's that can gen keys
//       - move security gen source code out of the project
//       - archived on 11/12/2018
//     y - basic cleanup (remove NUKE)
//       - remove ONE_SHOT (not needed)
//       - finished use E values for G2/G3
//       - tested EMO code; added report of state and startup and flipped polarity (was backwards)...actual pin is
//         sensed HIGH when the emo is pressed, disconnecting the path the GND allowing line to float HIGH from on PCB pullup
//       - added delay of 250ms for each move that is tossed due to no license key.  without the dealy, repretrel was swamping
//         itself and GUI became un-responsive (could not KILL jobs, etc).
//       - archived 11/14
//     z - restore code for per move Z for G2/G3 spiral in Z (merge in kjg's 3.082s)
//       - merge code from Hydra4 to limit display of repeated head errors to once ever XXX minutes.
// 3.083 - change initial EMO state to and >IN instead of >ER
//     a - skipped
//     b - reverted change done in 3.082q: " made M756, M757, M758 SINGLE_STEP as they affect flow calc ......."
//         that change introduced another problem with joe's spiral cylinder test case -- a pause on the M756 as
//         that was sent out with each "layer change" in the gcode.   a more exhaustive fix is needed which will
//         move all flow rate changes to motionQ_execute OR monaul flow.  (not sent from a M756)
//     c - force _LastExtrusionRate to invaid during a "reset"
//       - changed M106 to remove some of the "overloaded" uses and set it up more cleanly for the future as
//         M106 Sx Px  (Sx to set normal dutyCycle and Px to set on when extruding dutyCycle
//     d - (KJG)
//       - trying to fix the G12 G13 bore cycle.which is NOT starting in the correct lead in quarter arc
//         and also has erratic Z final move
//     e - (KJG)
//         trying to fix g12 g13
//     f - fix contact probe race condition with contact probe on a canbus.  added semaphore such that the probing move is held off until
//         the requested head has 'armed' it's probe and able to communicate if it's already at contact (to prevent the move from occurring)
//       - added I arg to G38 to select initial (valid) state of contact probe
//       - changed motionQ_abortMotionWhenSafe() to not set motionQ_abortMotionWhenSafe if _TouchProbeMoveActive
//         otherwise, motion is blocked
//       - fixed G2.1/G3.1 that were broken with G12/G13 changes  (main issue was the change made for the Radius calc in ExecuteG2G3PointMove()
//         with the G12/G13 the radius calc used new variables that were only set in G12/G13 ... so a bogus radius was used when G2.1/G3.1 were
//         issued.
//     g - fix one case where motion paused at the end of probe move (if move stopped on a dime) [in motionQ_abortMotionWhenSafe]
//     h - fixed issue where Q_POSIION and Q_LastRequestedPositionInUnits would be out of sync after a G38 commmand due to aborted moved
//       - changed G38 can probe timeout to report error with >TA to match other probe messaging
//     i - added H arg option to G38 to allow auto setting of the tool offset (a >RI: message is sent showing the assigned value)
//       - added auto abort if using H arg on G38 and the probe option fails (ABORT_CHAR send to repetrel .... needs a
//         TERMINATE_WAIT_CHAR or reset to continue)
//       - increased NUM_TOOL_OFFSETS to 60
//     j - added errorMessage throttle for canRxFull
//       - tweak registration process to ignore multiple "ANNOUNCE" messages from a head if the reg was in process
//       - added option for debug strings on can traffic (GB_STRING_CANBUS = 0x0100 = 256 == M797 S256)
//////////// merged through 083j


//     X - added _blockImmediateMoveOnToolChange (M6 argK) to allow control over whether or not to do an
//         immediate move on a tool change.   default behavior matches Hydra3 (force the move).
//         disabling allows changing to a new tool and them do some actions with that tool prior to jumping
//         to an active are of a print when using slic3r.
//       - added enable chiller mode to M140 (copied code from M104)
//       - retired M714 and the devicePositionRemap table (was never used);
//       - prep for new merge head codebase
//             - added headSetup.c and .h
//             - added code to detect new versus old code to differentiate to allow new/diff/mode canbus packets and still
//               interoperate with old code heads
//       - semi-big change on flow rate calc.   due to M756 issue from 082q/083b, the code has been changed to ONLY send
//         flow rate parameters from motionQ_execute...so they stay in sync with motion.  the problem was the layerHeight
//         could change and the M756 calculated the flow change.  if M756 single step, then a slight pause in motion.  if
//         SYNC_WITH_MOTION, then had an issue because the flow rate changes were done at motionQ_add time.
//       - tons of cleanup -- removing unused variables from structures, minimizing what's kept in the outbox, etc.
#endif // 0

////////////////////////////////////////////////////////////////////////////////
//  DEBUG DEFINTIONS --- SHOULD BE DISABLED WHEN COMPILER FOR RELEASE
////////////////////////////////////////////////////////////////////////////////

#ifdef GB_DEBUGGING

#define GB_DEBUG_REGISTRATION

//#define GB_HIDDEN_WARNINGS  // uncomment to see breadcrumbs in code that need to be addressed as compiler warnings
//#define GB_DEBUG_ARC
//#define GB_DEBUGGING_HCSR04
//#define GB_DEBUG_LSI_FPU
#define GB_DEBUGGING_KEEP_HEADS_ALIVE
//#define GB_DEBUG_MOTION_Q
//#define GB_TX_LINE_Q_GM
//#define GB_TX_LINENUM
#define GB_DEBUG_KILL

//#define MEASURE_TIME_SLIPPAGE
//#define MEASURE_ISR_RATE // only for hand testing with the LA == will be a NON-FUNCTIONING-PROGRAM (roughly 2M ISR/sec measured
#if defined(MEASURE_TIME_SLIPPAGE) && defined(MEASURE_ISR_RATE)
#warning "cannot enable both MEASURE_TIME_SLIPPAGE and MEASURE_ISR_RATE -- each needs TIM2"
#undef MEASURE_TIME_SLIPPAGE
#undef MEASURE_ISR_RATE
#endif //defined(MEASURE_ISR_RATE) || defined(MEASURE_TIME_SLIPPAGE)

#ifdef USE_HYDRA_IO

//#define USE_PANEL_6TO1_IO_FOR_DEBUG_PINS
#ifdef USE_PANEL_6TO1_IO_FOR_DEBUG_PINS

#define WIGGLE_DEBUG_PINS_AT_BOOT_TIME

//#define GB_DEBUG_IRQ_OFFON_PIN                GB_DEBUG_PIN1
//#define GB_UART_ISR_PIN                       GB_DEBUG_PIN2
//#define GB_UART_OVERRUN_PIN                   GB_DEBUG_PIN3
//#define GB_STARTUP_PIN        GB_DEBUG_PIN1
//#define GB_SECONDS_PIN        GB_DEBUG_PIN2
//#define GB_SYSTICK_PIN        GB_DEBUG_PIN3
//#define GB_LIGHTBURN_PIN  GB_DEBUG_PIN6

//#ifdef MEASURE_ISR_RATE
//#define GB_TIM2_ISR_PIN                   GB_DEBUG_PIN1
//#define GB_TIM2_ISR_SET                   GB_DEBUG_PIN1_SET
//#define GB_TIM2_ISR_CLEAR             GB_DEBUG_PIN1_CLEAR
//#define GB_TIM2_ISR_READ              GB_DEBUG_PIN1_READ
//#define GB_TIM2_ISR_TOGGLE                GB_DEBUG_PIN1_TOGGLE
//#endif
//
#define GB_MQ_EXECUTE_PIN             GB_DEBUG_PIN1
#define GB_MQ_EXECUTE_SET             GB_DEBUG_PIN1_SET
#define GB_MQ_EXECUTE_CLEAR               GB_DEBUG_PIN1_CLEAR
#define GB_MQ_EXECUTE_READ                GB_DEBUG_PIN1_READ
#define GB_MQ_EXECUTE_TOGGLE          GB_DEBUG_PIN1_TOGGLE
//
//#define GB_MQ_PRE_CALC_PIN                GB_DEBUG_PIN2
//#define GB_MQ_PRE_CALC_SET                GB_DEBUG_PIN2_SET
//#define GB_MQ_PRE_CALC_CLEAR          GB_DEBUG_PIN2_CLEAR
//
//#define GB_MQ_CHK_FLOWRATE_PIN            GB_DEBUG_PIN3
//#define GB_MQ_CHK_FLOWRATE_SET            GB_DEBUG_PIN3_SET
//#define GB_MQ_CHK_FLOWRATE_CLEAR      GB_DEBUG_PIN3_CLEAR

//#define GB_TIM1_ISR_PIN                   GB_DEBUG_PIN1
//#define GB_TIM1_ISR_SET                   GB_DEBUG_PIN1_SET
//#define GB_TIM1_ISR_CLEAR             GB_DEBUG_PIN1_CLEAR
//
//#define GB_TIMX_ISR_PIN                   GB_DEBUG_PIN2
//#define GB_TIMX_ISR_SET                   GB_DEBUG_PIN2_SET
//#define GB_TIMX_ISR_CLEAR             GB_DEBUG_PIN2_CLEAR

//#define GB_TIMY__ISR_PIN                  GB_DEBUG_PIN3
//#define GB_TIMY__ISR_SET              GB_DEBUG_PIN3_SET
//#define GB_TIMY__ISR_CLEAR                GB_DEBUG_PIN3_CLEAR

//#define GB_SYSTICK_PIN                        GB_DEBUG_PIN3
//
//#define GB_FUNC_PROC_MOTION_PIN               GB_DEBUG_PIN4
//#define GB_FUNC_PROC_MOTION_SET               GB_DEBUG_PIN4_SET
//#define GB_FUNC_PROC_MOTION_CLEAR         GB_DEBUG_PIN4_CLEAR

//#define GB_FUNC_TIM7_ISR_PIN                      GB_DEBUG_PIN4
//#define GB_FUNC_TIM7_ISR_SET                      GB_DEBUG_PIN4_SET
//#define GB_FUNC_TIM7_ISR_CLEAR                    GB_DEBUG_PIN4_CLEAR
//
//#define GB_FUNC_UPDATE_NEXT_ACCEL_PIN     GB_DEBUG_PIN5
//#define GB_FUNC_UPDATE_NEXT_ACCEL_SET     GB_DEBUG_PIN5_SET
//#define GB_FUNC_UPDATE_NEXT_ACCEL_CLEAR       GB_DEBUG_PIN5_CLEAR
//
//#define GB_FUNC_CALC_TIMER_CTRL_PIN       GB_DEBUG_PIN5
//#define GB_FUNC_CALC_TIMER_CTRL_SET       GB_DEBUG_PIN5_SET
//#define GB_FUNC_CALC_TIMER_CTRL_CLEAR     GB_DEBUG_PIN5_CLEAR
//
//#define GB_FUNC_ADD_COMMAND_PIN                   GB_DEBUG_PIN3
//#define GB_FUNC_ADD_COMMAND_SET                   GB_DEBUG_PIN3_SET
//#define GB_FUNC_ADD_COMMAND_CLEAR             GB_DEBUG_PIN3_CLEAR
//
//#define GB_FUNC_EXECUTE_MOVE_PIN              GB_DEBUG_PIN4
//#define GB_FUNC_EXECUTE_MOVE_SET              GB_DEBUG_PIN4_SET
//#define GB_FUNC_EXECUTE_MOVE_CLEAR            GB_DEBUG_PIN4_CLEAR
//
//#define GB_FUNC_START_MOVE_PIN                GB_DEBUG_PIN5
//#define GB_FUNC_START_MOVE_SET                GB_DEBUG_PIN5_SET
//#define GB_FUNC_START_MOVE_CLEAR              GB_DEBUG_PIN5_CLEAR
//
//#define GB_ACCEL_DECEL_PIN                GB_DEBUG_PIN6
//#define GB_ACCEL_DECEL_SET                GB_DEBUG_PIN6_SET
//#define GB_ACCEL_DECEL_CLEAR          GB_DEBUG_PIN6_CLEAR

//#define GB_ACCEL_DECEL_PIN                GB_DEBUG_PIN6
//#define GB_ACCEL_DECEL_SET                GB_DEBUG_PIN6_SET
//#define GB_ACCEL_DECEL_CLEAR          GB_DEBUG_PIN6_CLEAR

#endif //USE_PANEL_6TO1_IO_FOR_DEBUG_PINS

//#define USE_PANEL_ABSEL_IO_FOR_DEBUG_PINS
#ifdef USE_PANEL_ABSEL_IO_FOR_DEBUG_PINS
//#define GB_TIM1_ISR_PIN                       GB_DEBUG_PIN7



#endif //USE_PANEL_ABSEL_IO_FOR_DEBUG_PINS

//#define SLICE_TIMING_MEASUREMENT // cannot measure slice timing and use co2 laser at same time on a sys30
#ifdef SLICE_TIMING_MEASUREMENT
//#define GB_SLICE_TIMING_PIN       GB3
#endif //SLICE_TIMING_MEASUREMENT
//#define GB_CMDQUE_EMPTY_PIN       GB1
//#define GB_MOTIONQ_EMPTY_PIN      GB2
//#define GB_DEFFERED_CMDS_PIN      GB2
//#define GB_SOAP_READ_PIN          GB1
//#define GB_MOVING_PIN             GB1
//#define GB_ABORT_PIN              GB1
//#define GB_ACCEL_DECEL_PIN        GB2
//#define GB_DARG_MATCH_PIN         GB2
//#define GB_HEARTBEAT_PIN          GB5

//#define GB_UNPRIME_PIN            GB1
//#define GB_PRIME_PIN              GB2
//#define GB_RESIDUAL_PIN           GB2
//#define DDL_STUNT_DOUBLE_PIN      PIN_UNDEFINED
//#define GB_LASER_POWER_NONZERO_PIN GB2
#endif //USE_HYDRA_IO
#endif //GB_DEBUGGING

//to change target just uncomment ONLY ONE of the following defines
#define FK_407   //102207 board, rev X series, works with stm32F407VBT6 100 pin lqfp parts.
//#define FK_750   //102207 board, rev X series, works with STM32H750VBT6 100 pin lqfp parts.
//#define CORE_407I     //waveshare CORE407I on legacy 102207 boards, for EHR, and Hydras

/* Display Compiling Option */
#define LCD_SPI_PORT 3
// #define SECS_USART6


//#define ST7735
#define ST7789
//#define ILI9341
