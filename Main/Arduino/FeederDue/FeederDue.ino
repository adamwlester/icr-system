
//-------FEEDERDUE-------

#pragma region ---------DEBUG SETTINGS---------

/*
NOTES
XBee DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size)
XBee DO (to UART rx) buffer = 202 bytes
DUE SERIAL_BUFFER_SIZE = 128
SerialUSB receive buffer size is now 512 (ARDUINO 1.5.2 BETA - 2013.02.06)
*/

// LOG DEBUGGING

// Do log
const bool doDB_Log = false;

// What to print
const bool doLog_flow = false;
const bool doLog_irSync = false;
const bool doLog_motorControl = false;
const bool doLog_rcvd = false;
const bool doLog_r2c = false;
const bool doLog_r2a = false;
const bool doLog_pid = false;
const bool doLog_bull = false;
const bool doLog_resent = false;
const bool doLog_dropped = false;

// PRINT DEBUGGING

// Where to print
const bool doDB_PrintConsole = false;
const bool doDB_PrintLCD = true;

// What to print
const bool doPrint_flow = true;
const bool doPrint_irSync = false;
const bool doPrint_motorControl = false;
const bool doPrint_rcvd = false;
const bool doPrint_r2c = false;
const bool doPrint_r2a = false;
const bool doPrint_pid = false;
const bool doPrint_bull = false;
const bool doPrint_resent = false;
const bool doPrint_dropped = false;

// PID CALIBRATION
// Set kC and run ICR_Run.cs
const bool do_pidCalibration = false;
const float kC = 4; // critical gain [2,3,4]
const float pC = 2.13; // oscillation period [3.28,2.32,2.13]  
const float c_speedSteps[4] = { 10, 20, 30, 40 }; // (cm/sec) [{ 10, 20, 30, 40 }]
uint32_t c_durSteps = 30000; // (ms)


// POSITION
const bool do_posDebug = false;

#pragma endregion


#pragma region ---------LIBRARIES & PACKAGES---------


//-------SOFTWARE RESET----------
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

//----------LIBRARIES------------

// General
#include <string.h>

// AutoDriver

#include <SPI.h>

#include "AutoDriver_Due.h"

// Pixy

#include <Wire.h>

#include <PixyI2C.h>

// LCD
#include <LCD5110_Graph.h>

// TinyEKF
#define N 4     // States
#define M 6     // Measurements
#include <TinyEKF.h>

#pragma endregion 


#pragma region ---------PIN DECLARATION---------

// Autodriver
const int pin_AD_CSP_R = 5;
const int pin_AD_CSP_F = 6;
const int pin_AD_RST = 7;

// Display
const int pin_Disp_SCK = 8;
const int pin_Disp_MOSI = 9;
const int pin_Disp_DC = 10;
const int pin_Disp_RST = 11;
const int pin_Disp_CS = 12;
const int pin_Disp_LED = 13;

// LEDs
const int pin_RewLED_R = 4;
const int pin_RewLED_C = 3;
const int pin_TrackLED = 2;

// Relays
const int pin_Rel_EtOH = 22;
const int pin_Rel_Rew = 23;

// BigEasyDriver
const int pin_ED_RST = 47;
const int pin_ED_SLP = 49;
const int pin_ED_DIR = 51;
const int pin_ED_STP = 53;
const int pin_ED_ENBL = 35;
const int pin_ED_MS1 = 37;
const int pin_ED_MS2 = 39;
const int pin_ED_MS3 = 41;

// Feeder switch
/*
Note: Do not use real ground pin as this will cause
an upload error if switch is shorted when writing sketch
*/
const int pin_FeedSwitch_Gnd = 14;
const int pin_FeedSwitch = 15;

// Power off
const int pin_PwrOff = 45;

// Voltage monitor
const int pin_BatVolt = A11;

// Buttons
const int pin_Btn[3] = { A3, A2, A1 };

/*
Note: pins bellow are all used for external interupts
and must all be members of the same port (PortA)
*/

// IR proximity sensors
const int pin_IRprox_Rt = 42;
const int pin_IRprox_Lft = 43;

// IR detector
const int pin_IRdetect = 17;

#pragma endregion


#pragma region ---------VARIABLE SETUP---------

// Log debugging
String logList[150];
uint16_t logCnt = 0;

// Print debugging
String printQueue[10];
const int printQueue_lng =
sizeof(printQueue) / sizeof(printQueue[0]);
int printQueueInd = printQueue_lng;
bool doPrint = false;
bool doBlockLCDlog = false;

// Debug tracking
uint32_t t_loopMain = millis();
uint32_t t_loopRead = millis();
int cnt_droppedPacks;
int cnt_overflowEvt = 0;
int cnt_timeoutEvt = 0;
int cnt_packResend = 0;

// State control
String fc_motorControl = "None"; // ["None", "Open", "MoveTo", "Bull", "PID"]
uint32_t t_blockTill;
bool fc_isBlockingTill = false;
bool fc_doQuit = false;
bool fc_isFirstPass = true;
bool fc_doStreamCheck = false;
bool fc_isStreaming = false;
bool fc_isManualSes = false;
bool fc_isRatIn = false;
bool fc_isTrackingEnabled = false;
bool fc_doMove = false;
bool fc_doRew = false;
bool fc_doCueReward = false;
bool fc_isRewarding = false;
bool fc_isCueing = false;
bool fc_doHalt = false;
bool fc_isHalted = false;
bool fc_doBulldoze = false;
bool fc_doCheckDoneRcvd = false;
bool fc_doLogSend = false;
bool fc_doLogResend = false;

// Start/Quit
byte msg_setupCmd[2];
uint32_t t_quitCmd;

// Serial from CS
const char c2r_head[2] = { '_', '<' };
const char c2r_foot = '>';
const char c2r_id[14] = {
	'T', // system test command
	'S', // start session
	'Q', // quit session
	'M', // move to position
	'R', // run reward
	'C', // cue reward
	'H', // halt movement
	'B', // bulldoze rat
	'I', // rat in/out
	'P', // position data
	'V', // request stream status
	'L', // request log send/resend
	'Y', // confirm done recieved
};
char msg_id = ' ';
bool msg_pass = false;
uint32_t t_rsvd = millis(); // (ms)
uint32_t t_rsvdLast; // (ms)

// Serial to CS
const char r2c_head = '{';
const char r2c_foot = '}';
const char r2c_id[15] = {
	'T', // system test command
	'S', // start session
	'Q', // quit session
	'M', // move to position
	'R', // run reward
	'C', // cue reward
	'H', // halt movement
	'B', // bulldoze rat
	'I', // rat in/out
	'D', // execution done
	'V', // connected and streaming
	'L', // request log send/resend
	'U', // log pack
	'J', // battery voltage
	'A', // reward zone
};
uint32_t t_resendDone = millis(); // (ms)
uint32_t t_sent = millis(); // (ms)
//uint32_t resendDone_cnt = 0;
//uint32_t resendDone_max = 5;

// Serial to other ard
const char r2a_head = '[';
const char r2a_foot = ']';
uint16_t r2a_packCnt = 0;
const char r2a_id[10] = {
	't', // set sync time
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
};

// Outgoing data
byte r2_queue[10][7];
const int r2_lngR = 10;
const int r2_lngC = 7;
int sendQueueInd = r2_lngR - 1;
bool doSend = false;

// Serial packet tracking
uint16_t packNow;
uint16_t packLast = 0;
int packTot = 0;
// last packet
const int c2r_idLng = sizeof(c2r_id) / sizeof(c2r_id[0]);
const int r2c_idLng = sizeof(r2c_id) / sizeof(r2c_id[0]);
const int r2a_idLng = sizeof(r2a_id) / sizeof(r2a_id[0]);
uint16_t c2r_packLast[c2r_idLng];
uint16_t r2c_packLast[r2c_idLng];
uint16_t c2r_packRepeat[c2r_idLng];
int c2r_cntRepeat[c2r_idLng];
// packet history
uint16_t c2r_packHist[10];
uint16_t r2c_packHist[10];
char c2r_idHist[10];
char r2c_idHist[10];
const int c2r_hLng = sizeof(c2r_packHist) / sizeof(c2r_packHist[0]);
const int r2c_hLng = sizeof(r2c_packHist) / sizeof(r2c_packHist[0]);

// Serial VT
byte msg_vtEnt;
float msg_vtCM[2];
uint32_t msg_vtTS[2];

// Pixy
const double pixyCoeff[5] = {
	0.000000043550534,
	-0.000023239535204,
	0.005033059128963,
	-0.677050955917591,
	75.424132382709260
};
float vtpixyVelAvg;
float vtpixyPosAvg;

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 100; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)
const float frontMoterScale = 1.0375;
const byte kAcc = 60 * 2;
const byte kDec = 60 * 2;
const byte kRun = 60;
const byte kHold = 60 / 2;

// Kalman model measures
float ekfRatPos;
float ekfRobPos;
float ekfRatVel;
float ekfRobVel;

// PID Settings
bool do_includeTerm[2] = { true, true };
const float defualtSetPoint = 42; // (cm)
const float guardDist = 4.5;
const float feedDist = 66;
float errorFeeder;
float errorDefault;

// Movement
float moveToSpeed = 80;
float msg_moveToTarg;
float moveToDist;
float moveToStartPos;
byte msg_bullDel;
byte msg_bullSpeed;

// Reward
float msg_cueTarg;
float cueDist[2];
float cueStartPos[2];
float msg_rewPos;
byte msg_rewDurByte;
int rewCnt = 0;

// EtOH 
/*
EtOH run after min time or distance
*/
bool isEtOHOpen = false;
const uint32_t t_durEtOH = 1000; // (ms)
const uint32_t t_delEtOH = 10000; // (ms)
const float distMaxEtOH = (140 * PI) / 2; // (cm)

// Volt tracking
/*
Updated when EtOH relay opened
*/
const float bit2volt = 0.0164;
float batVoltArr[100];

// LEDs
const int trackLEDduty = 75; // value between 0 and 255
const int rewLEDduty = 15; // value between 0 and 255
const int rewLEDmin = 0; // value between 0 and 255

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
bool lcdLightOn = false;

// Buttons
volatile bool btn_doChangeSolState = false;
volatile bool btn_doRew = false;
volatile bool btn_doChangeLCDstate = false;

// Interrupts 
volatile uint32_t intrpt_irProxDebounce = millis();
volatile bool intrpt_doIRhardStop = false;
volatile uint32_t intrpt_irDetectDebounce = millis();
volatile uint32_t t_irSyncLast = millis();
volatile uint32_t t_sync = 0;
volatile bool doLogIR = false;

#pragma endregion 


#pragma region ---------CONSTRUCT CLASSES---------

//----------INITILIZE OBJECTS----------

// AutoDriver
AutoDriver_Due ad_R(pin_AD_CSP_R, pin_AD_RST);
AutoDriver_Due ad_F(pin_AD_CSP_F, pin_AD_RST);
// Pixy
PixyI2C pixy(0x54);
// LCD
LCD5110 myGLCD(pin_Disp_CS, pin_Disp_RST, pin_Disp_DC, pin_Disp_MOSI, pin_Disp_SCK);

//----------CLASS: PosDat----------
class PosDat
{
public:
	char objID = ' ';
	int nSamp = 0;
	float posArr[6] = { 0,0,0,0,0,0 };
	uint32_t tsArr[6] = { 0,0,0,0,0,0 };
	int t_skip = 0;
	float velNow = 0.0f;
	float posNow = 0.0f;
	float posAbs = 0.0f;
	uint32_t t_tsNow = 0;
	uint32_t t_msNow = millis();
	float nLaps = 0;
	int sampCnt = 0;
	bool newData = false;
	uint32_t dtFrame = 0;

	// constructor
	PosDat(char id, int l)
	{
		this->objID = id;
		this->nSamp = l;
		for (int i = 0; i < l; i++) this->posArr[i] = 0.0f;
	}

	void UpdatePosVel(float pos_new, uint32_t ts_new)
	{
		// Store input
		this->t_msNow = millis();
		this->posAbs = pos_new;
		this->t_tsNow = ts_new;

		// Update itteration count
		this->sampCnt++;

		// Shift and add data
		for (int i = 0; i < this->nSamp - 1; i++)
		{
			this->posArr[i] = this->posArr[i + 1];
			this->tsArr[i] = this->tsArr[i + 1];
		}
		// Add new variables
		this->posArr[nSamp - 1] = pos_new;
		this->tsArr[nSamp - 1] = ts_new;

		// Do not process early samples
		if (this->sampCnt < this->nSamp + 1)
		{
			this->posNow = pos_new;
			this->velNow = 0.0f;
			this->dtFrame = 0;
			newData = false;
		}
		else
		{
			newData = true;

			// Store frame capture dt
			this->dtFrame = this->tsArr[nSamp - 1] - this->tsArr[nSamp - 2];

			// COMPUTE TOTAL targ_dist RAN

			// Get dif of current pos and last pos
			float posDiff = pos_new - this->posArr[this->nSamp - 2];
			if (abs(posDiff) > 140 * (PI / 2))
			{
				if (posDiff < 0) // crossed over
				{
					this->nLaps++;
				}
				else // crossed back
				{
					this->nLaps--;
				}
			}
			this->posNow = pos_new + this->nLaps*(140 * PI);

			// COMPUTE VELOCITY

			float dist;
			float distSum = 0;
			int dt;
			float dtSum = 0;
			float vel;
			for (int i = 0; i < this->nSamp - 1; i++)
			{
				// compute targ_dist
				dist = this->posArr[i + 1] - this->posArr[i];
				distSum += dist;
				// compute dt
				dt = this->tsArr[i + 1] - this->tsArr[i];
				dtSum += (float)dt;
			}
			// compute vel
			vel = distSum / (dtSum / 1000.0f);

			// ignore outlyer values unless too many frames discarted
			if (this->t_skip > 500 || abs(this->velNow - vel) < 150)
			{
				this->velNow = vel;
				this->t_skip = 0;
			}
			// add to skip time
			else this->t_skip += dt;
		}
	}

	float GetPos()
	{
		newData = false;
		return posNow;
	}

	float GetVel()
	{
		return velNow;
	}

	void SetDat(float set_pos, uint32_t ms)
	{
		// Compute ts
		uint32_t ts = t_tsNow + (ms - t_msNow);

		// Update pos
		UpdatePosVel(set_pos, ts);
	}

	void ResetDat(float set_pos, float set_laps)
	{
		posNow = set_pos;
		sampCnt = 0;
		newData = false;
		if (set_pos != -100)
		{
			nLaps = set_laps;
		}
	}

};
// Initialize objects
PosDat pos_ratVT('A', 4);
PosDat pos_robVT('C', 4);
PosDat pos_ratPixy('B', 6);

//----------CLASS: PID----------
class PID
{

public:
	uint32_t t_lastLoop = 0;
	float dtLoop = 0;
	bool wasLoopRan = false;
	float p_term = 0;
	float i_term = 0;
	float d_term = 0;
	bool firstRun = true;
	String mode = "Manual"; // ["Manual" "Automatic" "Halted"]
	bool isHolding4cross = false;
	bool doDamp = false;
	bool isDampened = false;
	float error = 0;
	float errorLast = 0;
	float integral = 0;
	float derivative = 0;
	float velUpdate = 0;
	float runSpeed = 0;
	float speedMax = maxSpeed;
	float dT = 0;
	float kP = 0;
	float kI = 0;
	float kD = 0;
	float setPoint = 0;
	uint32_t t_ekfStr = 0;
	uint32_t t_ekfSettle = 500; // (ms)
	bool ekfReady = false;
	bool ekfNew = false;
	float dampAcc = 40;
	float dampSpeedCut = 20;
	uint32_t dampTimeCut = 4000;
	uint32_t t_dampTill = 0;
	uint32_t t_lastDamp = millis();

	// PID calibration
	uint32_t cal_t_calStr = millis();
	uint32_t cal_dtMin = 40; // (ms)
	int cal_stepNow = 0;
	float cal_PcCnt = 0;  // oscillation count
	float cal_PcSum = 0; // oscillation period sum
	uint32_t cal_t_PcNow = 0;
	uint32_t cal_t_PcLast = 0;
	float cal_PcArr[4] = { 0, 0, 0, 0 };
	float cal_PcAvg = 0;
	float cal_PcNow = 0;
	float cal_PcAll = 0;
	float cal_errNow = 0;
	float cal_errLast = 0;
	float cal_errAvg = 0;
	float cal_errArr[4] = { 0, 0, 0, 0 };
	float cal_dtLoop = 0;
	float cal_errCnt = 0;
	float cal_errSum = 0;
	float cal_errMax = 0;
	float cal_errMin = 0;
	float cal_ekfRatPos = 0;
	float cal_ekfRatVel = 0;
	bool cal_isPidUpdated = false;
	bool cal_isCalFinished = false;

	// constructor
	PID(const float kC, const float pC, const float set_point)
	{
		this->kP = 0.6 * kC; // proportional constant
		this->kI = 2 * kP / pC; // integral constant
		this->kD = kP*pC / 8; // derivative constant
		this->setPoint = set_point;
	}

	float UpdatePID()
	{

		// Wait till ekf ready
		if (!ekfReady)
		{
			return -1;
		}
		else
		{

			// Compute error 
			error = ekfRatPos - (ekfRobPos + setPoint);

			// Compute other error measures
			errorFeeder = ekfRatPos - (ekfRobPos + feedDist);
			errorDefault = ekfRatPos - (ekfRobPos + defualtSetPoint);

			// Check if motor is open
			CheckMotorControl();

			// Check dampening 
			CheckDamp();

			// Check if in auto mode
			if (mode != "Automatic")
			{
				return -1;
			}
			else
			{

				// Check if rat stopped behind default setpoint
				if (ekfRatVel < 1 && errorDefault < -15 && !isHolding4cross)
				{
					// halt running
					return runSpeed = 0;
				}

				// Check if dampening 
				SetDamp();

				// Check for setpoint crossing
				CheckSetpointCrossing();

				// Update PID and speed
				if (
					!ekfNew || // New EKF data 
					isHolding4cross // wait for setpoint pass
					)
				{
					return -1;
				}
				else
				{

					// Re-compute error 
					error = ekfRatPos - (ekfRobPos + setPoint);

					// Compute new integral
					if (do_includeTerm[0])
					{
						// Catch setpoint (i.e. error == 0) crossing
						if ((abs(error) + abs(errorLast)) > abs(error + errorLast))
						{
							integral = 0;
						}
						//else if (error < 0) integral = integral + error * 5;
						else integral = integral + error;
					}
					else integral = 0;

					// Compute new derivative
					if (do_includeTerm[1])
					{
						derivative = error - errorLast;
					}
					else derivative = 0;

					// Compute new terms
					p_term = kP*error;
					i_term = kI*dtLoop*integral;
					d_term = kD / dtLoop*derivative;
					//if (error < 0) d_term = d_term * 2.0f;

					// Compute updated vel
					velUpdate = p_term + i_term + d_term;

					// Get new run speed
					runSpeed = ekfRobVel + velUpdate;

					// Keep speed in range [0, speedMax]
					if (runSpeed > speedMax) runSpeed = speedMax;
					else if (runSpeed < 0) runSpeed = 0;

					errorLast = error;
					wasLoopRan = true;
					ekfNew = false;

					if (firstRun)
					{
						PrintPID("pid: first run");
						firstRun = false;
					}

					// Return new run speed
					return runSpeed;
				}

			}

		}

	}

	void Run(String called_from)
	{
		// Take motor control
		SetMotorControl("PID", "PID.Run");

		// Reset
		Reset();
		mode = "Automatic";

		// Tell ard pid is running
		Store4_Ard('p', 1);

		PrintPID("pid: run [" + called_from + "]");
	}

	void Stop(String called_from)
	{
		if (fc_motorControl == "PID")
		{
			// Stop movement
			RunMotor('f', 0, "PID");

			// Set run speed
			runSpeed = 0;

			// Give over control
			SetMotorControl("Open", "PID.Stop");
		}

		// Tell ard pid is stopped
		if (
			// Dont send if rewarding
			!fc_isRewarding &&
			!fc_isCueing
			)
		{
			Store4_Ard('p', 0);
		}

		// Set mode
		mode = "Manual";

		// Print
		PrintPID("pid: stop [" + called_from + "]");
	}

	void Hold(String called_from)
	{
		// Call Stop
		Stop("PID.Hold");

		// But set mode to "Hold"
		mode = "Hold";

		PrintPID("pid: hold [" + called_from + "]");
	}

	void Reset()
	{
		integral = 0;
		t_lastLoop = millis();
		isHolding4cross = true;
		doDamp = true;
	}

	void SetDamp()
	{
		if (doDamp)
		{
			// Only run if rat ahead of setpoint
			if (
				errorDefault > 5 &&
				millis() > t_lastDamp + 5000
				)
			{
				// Change acc to rat pos
				//ad_R.setAcc(dampAcc*cm2stp);
				//ad_F.setAcc(dampAcc*cm2stp);
				delayMicroseconds(100);

				// Set time to dampen till
				t_dampTill = millis() + dampTimeCut;

				// Set flags
				isDampened = true;
				doDamp = false;

				char str[50];
				sprintf(str, "pid: dampen to %0.2f", dampAcc);
				PrintPID(str);
			}
			else
			{
				doDamp = false;
			}
		}
	}

	void CheckDamp()
	{
		if (isDampened)
		{

			// Check for criteria
			if (
				millis() >= t_dampTill ||
				runSpeed > dampSpeedCut ||
				errorDefault < 5
				)
			{
				// Set acc back to normal
				//ad_R.setAcc(maxAcc*cm2stp);
				//ad_F.setAcc(maxAcc*cm2stp);
				delayMicroseconds(100);

				// Store time
				t_lastDamp = millis();

				// Reset flag
				isDampened = false;
				PrintPID("pid: finished dampening");
			}
		}
	}

	void CheckMotorControl()
	{
		// Check if motor control available
		if ((fc_motorControl == "PID" || fc_motorControl == "Open") &&
			mode == "Hold")
		{
			// Run pid
			Run("PID.CheckMotorControl");
		}
		else if ((fc_motorControl != "PID" && fc_motorControl != "Open") &&
			mode == "Automatic")
		{
			// Hold pid
			Hold("PID.CheckMotorControl");
		}
	}

	void CheckSetpointCrossing()
	{
		// Check if rat has moved in front of setpoint
		if (isHolding4cross && errorDefault > 0)
		{
			isHolding4cross = false;
			PrintPID("pid: crossed setpoint");
		}
	}

	void ChangeSetpoint(float delta)
	{
		setPoint = defualtSetPoint + delta;
	}

	void CheckEKF(uint32_t now_ms)
	{
		if (!ekfReady)
		{
			if ((now_ms - t_ekfStr) > t_ekfSettle)
			{
				ekfReady = true;
			}
		}
	}

	void ResetEKF(String called_from)
	{
		ekfReady = false;
		t_ekfStr = millis();
		PrintPID("pid: reset ekf [" + called_from + "]");
	}

	void SetLoopTime(uint32_t now_ms)
	{
		ekfNew = true;
		if (wasLoopRan)
		{
			dtLoop = (float)(now_ms - t_lastLoop) / 1000.0f;
			wasLoopRan = false;
			t_lastLoop = now_ms;
		}
	}

	void PrintPID(String str)
	{
		// Add to print queue
		if (doPrint_pid && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_pid && doDB_Log)
			StoreDBLogStr(str, millis());
	}

	float RunPidCalibration()
	{

		/*
		Calibration based on the Ziegler–Nichols method:
		http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
		Equations:
		kP = 0.6*Kc
		Kd = 2*Kp * dT/Pc
		Ki = Kp*Pc / (8*dT)
		*/

		if (!cal_isCalFinished)
		{
			// End of calibration 
			if (
				millis() > (4 * (c_durSteps)+cal_t_calStr) &&
				cal_PcArr[3] > 0
				)
			{
				float _pc_sum = 0;
				// Compute overal average
				for (int i = 0; i < 4; i++)
				{
					_pc_sum += cal_PcArr[i];
				}
				cal_PcAll = _pc_sum / 4;
				cal_isPidUpdated = true;
				cal_isCalFinished = true;
				return -1;
			}
			else if (!ekfNew || ekfRobPos <= 0)
			{
				return -1;
			}
			else if (millis() > t_lastLoop + cal_dtMin)
			{

				// Setup stuff
				if (cal_ekfRatPos == 0)
				{
					cal_ekfRatPos = ekfRobPos + setPoint;
					t_lastLoop = millis();
					cal_t_calStr = millis();
				}

				// Check if speed should be incrimented
				if (millis() > (cal_stepNow + 1)*c_durSteps + cal_t_calStr)
				{
					cal_PcArr[cal_stepNow] = cal_PcAvg;
					cal_PcSum = 0;
					cal_PcCnt = 0;
					cal_errArr[cal_stepNow] = cal_errAvg;
					cal_errSum = 0;
					cal_errCnt = 0;
					if (cal_stepNow < 3) cal_stepNow++;
				}

				// Get loop dt
				cal_dtLoop = (float)(millis() - t_lastLoop) / 1000.0f;
				t_lastLoop = millis();

				// Compute PID
				cal_ekfRatVel = c_speedSteps[cal_stepNow];
				cal_ekfRatPos += cal_ekfRatVel * (cal_dtLoop / 1);

				// Compute error 
				error = cal_ekfRatPos - (ekfRobPos + setPoint);
				cal_errNow = error;

				// Compute new terms
				p_term = kC*error;

				// Get new run speed
				runSpeed = ekfRobVel + p_term;

				// Keep speed in range [0, speedMax]
				if (runSpeed > speedMax) runSpeed = speedMax;
				else if (runSpeed < 0) runSpeed = 0;

				// Catch occilation edge
				if (cal_errLast > 0 && error < 0)
				{
					cal_t_PcLast = cal_t_PcNow;
					cal_t_PcNow = millis();
					if (cal_t_PcLast > 0)
					{
						cal_PcCnt++;
						cal_PcNow = float(cal_t_PcNow - cal_t_PcLast) / 1000;
						cal_PcSum += cal_PcNow;
						cal_PcAvg = cal_PcSum / cal_PcCnt;
					}
				}
				cal_errCnt++;
				cal_errSum += abs(error);
				cal_errAvg = cal_errSum / cal_errCnt;
				cal_errMax = max(cal_errMax, error);
				cal_errMin = max(cal_errMin, error);

				// Update vars
				cal_errLast = error;
				cal_isPidUpdated = true;
				wasLoopRan = true;
				ekfNew = false;

				return runSpeed;
			}

		}

	}

};
// Initialize object
PID pid(kC, pC, defualtSetPoint);

//----------CLASS: Bulldozer----------
class Bulldozer
{

public:
	uint32_t t_loopNext = 0;
	uint32_t dtLoop = 50; // (ms)
	float moveMin = 5; // (cm)
	String mode = "Inactive"; // ["Active" "Inactive"]
	String state = "Off"; // ["off", "On", "Hold"]
	uint32_t t_bullNext = 0; // (ms)
	int bSpeed = 0;
	int bDelay = 0; // (ms)
	float posCheck = 0;
	float posNow = 0;
	float distMoved = 0;
	float guardPos = 0;
	bool hasMoved = false;
	bool timesUp = false;
	bool passedReset = false;

	void UpdateBull()
	{
		// Check who has motor control
		CheckMotorControl();

		// Bulldoze is on
		if (state == "On")
		{

			// Update when ready
			if (
				pid.ekfReady &&
				millis() > t_loopNext)
			{

				// Update rat pos
				posNow = ekfRatPos;
				guardPos = ekfRobPos + guardDist;

				// Get targ_dist traveled
				distMoved = posNow - posCheck;

				// Check for movement
				hasMoved = distMoved >= moveMin ? true : false;

				// Check if rat passed reset
				passedReset = errorDefault > 1 ? true : false;

				// Check time
				timesUp = millis() > t_bullNext ? true : false;

				// Check if has not moved in time
				if (!hasMoved)
				{
					// Bulldoze him!
					if (timesUp &&
						mode == "Inactive")
					{
						Run("Bulldozer.UpdateBull");
					}
				}
				// Has moved minimal targ_dist
				else
				{
					// Reset check pos
					posCheck = posNow;

					// Reset bull next
					t_bullNext = millis() + bDelay;

					// Stop bulldoze if rat ahead of set point and not 0 delay
					if (passedReset &&
						mode == "Active" &&
						bDelay != 0)
					{
						Stop("Bulldozer.UpdateBull");
					}
				}

				// Set next loop time
				t_loopNext = millis() + dtLoop;
			}
		}
	}

	void Reinitialize(byte del, byte spd, String called_from)
	{
		bSpeed = (int)spd;
		bDelay = (int)del * 1000;
		t_bullNext = millis() + bDelay;
		posCheck = ekfRatPos;
		// print
		PrintBull("bull: reinitialize " + called_from);
	}

	void Run(String called_from)
	{
		// Take control
		SetMotorControl("Bull", "Bulldozer.Run");

		// Start bulldozer
		RunMotor('f', bSpeed, "Bull");

		// Tell ard bull is running
		Store4_Ard('b', 1);

		// Set mode
		mode = "Active";

		// Print
		PrintBull("bull: run [" + called_from + "]");
	}

	void Stop(String called_from)
	{
		// Stop movement
		RunMotor('f', 0, "Bull");

		// Give over control
		if (fc_motorControl == "Bull")
		{
			SetMotorControl("Open", "Bulldozer.Stop");
		}

		// Reset bull next
		t_bullNext = millis() + bDelay;

		// Tell ard bull is stopped
		Store4_Ard('b', 0);

		// Set mode
		mode = "Inactive";

		// Print
		PrintBull("bull: stop [" + called_from + "]");
	}

	void TurnOn(String called_from)
	{
		// Change state
		state = "On";

		// Reset 
		Reset();

		PrintBull("bull: on [" + called_from + "]");
	}

	void TurnOff(String called_from)
	{
		// Change state
		state = "Off";

		// Stop bulldozer if running
		if (mode == "Active")
		{
			// Stop bull
			Stop("Bulldozer.TurnOff");
		}

		PrintBull("bull: off [" + called_from + "]");
	}

	void Hold(String called_from)
	{
		// Change state
		state = "Hold";

		// Stop running
		if (mode == "Active")
		{
			// Run stop bulldozer
			Stop("Bulldozer.Hold");

			// Set mode back to active for later
			mode = "Active";
		}

		PrintBull("bull: hold [" + called_from + "]");
	}

	void Resume(String called_from)
	{
		// Set state back to "On"
		state = "On";

		// Reset 
		Reset();

		PrintBull("bull: resume [" + called_from + "]");
	}

	void Reset()
	{
		// Set mode
		mode = "Inactive";

		// Reset bull next
		t_bullNext = millis() + bDelay;

		// Reset check pos
		posCheck = ekfRatPos;
	}

	void CheckMotorControl()
	{
		if ((fc_motorControl == "Bull" || fc_motorControl == "PID" || fc_motorControl == "Open") &&
			state == "Hold")
		{
			// Turn bull on
			Resume("Bulldozer.CheckMotorControl");
		}
		else if ((fc_motorControl != "Bull" && fc_motorControl != "PID" && fc_motorControl != "Open") &&
			state == "On")
		{
			// Turn bull off
			Hold("Bulldozer.CheckMotorControl");
		}
	}

	void PrintBull(String str)
	{
		// Add to print queue
		if (doPrint_bull && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_bull && doDB_Log)
			StoreDBLogStr(str, millis());
	}

};
// Initialize object
Bulldozer bull;

//----------CLASS: Target----------
class Target
{
public:
	String objID = " ";
	float posRel = 0;
	float moveDiff = 0;
	float minSpeed = 0;
	const uint32_t t_updateDT = 10;
	uint32_t t_updateNext = 0;
	float distLeft = 0;
	float newSpeed = 0;
	float posStart = 0;
	float targ = 0;
	float offsetTarget = 0;
	float targDist = 0;
	char moveDir = ' ';
	float baseSpeed = 0;
	bool isTargSet = false;
	bool isTargReached = false;
	float haltError = 0;
	const double velCoeff[3] = {
		0.001830357142857,
		0.131160714285714,
		-2.425892857142854,
	};

	// constructor
	Target(String id)
	{
		this->objID = id;
	}

	bool CompTarg(float now_pos, float targ_pos, float offset)
	{
		// Local vars
		int diam = 0;
		int pos = 0;

		// Copy to public vars
		posStart = now_pos;
		targ = targ_pos;

		// Run only if targ not set
		if (!isTargSet)
		{
			// Check pos data is ready
			if (pid.ekfReady)
			{
				// Compute target targ_dist and move_dir
				offsetTarget = targ + offset;
				if (offsetTarget < 0)
					offsetTarget = offsetTarget + (140 * PI);
				else if (offsetTarget > (140 * PI))
					offsetTarget = offsetTarget - (140 * PI);

				// Current relative pos on track
				diam = (int)(140 * PI * 100);
				pos = (int)(now_pos * 100);
				posRel = (float)(pos % diam) / 100;

				// Diff and absolute targ_dist
				moveDiff = offsetTarget - posRel;
				targDist =
					min((140 * PI) - abs(moveDiff), abs(moveDiff));

				// Set to negative for reverse move
				if ((moveDiff > 0 && abs(moveDiff) == targDist) ||
					(moveDiff < 0 && abs(moveDiff) != targDist))
				{
					moveDir = 'f';
				}
				else
				{
					moveDir = 'r';
					targDist = targDist*-1;
				}

				// Set vars for later
				t_updateNext = millis();
				baseSpeed = 0;

				// Set flag true
				isTargSet = true;
			}

		}

		// Retern move targ_dist
		return isTargSet;
	}

	float DecelToTarg(float now_pos, float now_vel, float dec_pos, float speed_min)
	{
		// Run if targ not reached
		if (!isTargReached)
		{

			// Compute remaining targ_dist
			distLeft = abs(targDist) -
				min((140 * PI) - abs(now_pos - posStart), abs(now_pos - posStart));

			// Check if rob is dec_pos cm from target
			if (distLeft <= dec_pos)
			{
				// Get base speed to decelerate from
				if (baseSpeed == 0)
				{
					baseSpeed = abs(now_vel);
				}
				// Update decel speed
				else if (millis() > t_updateNext)
				{
					// Compute new speed
					newSpeed = (distLeft / dec_pos) * baseSpeed;

					// Maintain at min speed
					if (newSpeed < speed_min) newSpeed = speed_min;

					// Update loop time
					t_updateNext = millis() + t_updateDT;
				}

			}

			// Target reached
			if (distLeft < 1)
			{
				// Set flag true
				isTargReached = true;

				newSpeed = 0;
			}
		}
		else newSpeed = 0;

		return newSpeed;
	}

	bool CheckTargReached(float now_pos, float now_vel)
	{
		// Run if targ not reached
		if (!isTargReached)
		{
			// Get velocity corrected targ_dist threshold
			if (now_vel >= 20)
			{
				haltError =
					velCoeff[0] * (now_vel * now_vel) +
					velCoeff[1] * now_vel +
					velCoeff[2];
			}
			else haltError = 1;

			// Cap halt error at min of 1
			haltError = haltError < 1 ? 1 : haltError;

			// Compute remaining targ_dist
			distLeft = abs(targDist) -
				min((140 * PI) - abs(now_pos - posStart), abs(now_pos - posStart));

			// Target is reached
			if (distLeft < haltError)
			{
				// Set flag true
				isTargReached = true;
			}
		}

		return isTargReached;
	}

	float GetError(float now_pos)
	{
		// Local vars
		int diam = 0;
		int pos = 0;

		// Current relative pos on track
		diam = (int)(140 * PI * 100);
		pos = (int)(now_pos * 100);
		posRel = (float)(pos % diam) / 100;

		// Target error
		return offsetTarget - posRel;
	}

	void Reset()
	{
		isTargSet = false;
		isTargReached = false;
	}
};
// Initialize object
Target targ_moveTo("targ_moveTo");
Target targ_cueRat("targ_cueRat");
Target targ_cueRob("targ_cueRob");

//----------CLASS: Reward----------
class Reward
{
public:
	const uint32_t t_block = 15000; // (ms)
	const float velThresh = 5; // (cm/sec)
	const uint32_t targRewDurs[9] = { // (ms)
		500,
		910,
		1420,
		1840,
		2000,
		1840,
		1420,
		910,
		500
	};
	const float targLocs[9] = { // (deg)
		20,
		15,
		10,
		5,
		0,
		-5,
		-10,
		-15,
		-20,
	};
	uint32_t duration; // (ms) 
	uint32_t durationByte; // (ms) 
	float targBounds[9][2];
	const int targLng =
		sizeof(targLocs) / sizeof(targLocs[0]);
	float boundMin = 0;
	float boundMax = 0;
	uint32_t t_closeSol = 0;
	uint32_t t_retractArm = 0;
	float rewCenter = 0;
	bool isboundSet = false;
	bool isTriggerReady = false;
	bool isAllRargPassed = false;
	bool is_ekfNew = false;
	float rewardedTarg = 0;
	float rewardedBounds[2];
	float lapN = 0;
	bool doArmMove = false;
	bool isArmExtended = false;
	const int armExtStps = 200;
	int armPos = 0;
	int armTarg = 0;
	bool armStpOn = false;
	uint32_t t_lastOnStep = millis();
	uint32_t t_lastOffStep = millis();

	// Constructor
	Reward()
	{
		this->duration = 2000;
		this->durationByte = (byte)(duration / 10);
		ResetFeedArm();
	}

	// START REWARD
	bool StartRew(bool do_stop)
	{
		// Local vars
		bool is_rewarding = true;

		// Track rewards
		rewCnt++;

		// Set to extend feeder arm 
		ExtendFeedArm();

		// Stop robot
		if (do_stop)
		{
			HardStop("StartRew");
			// Set hold time
			BlockMotorTill(t_block);
		}

		// Trigger reward tone on
		Store4_Ard('r', durationByte);

		// Compute reward end time
		t_closeSol = millis() + duration;

		// Turn on reward LED
		analogWrite(pin_RewLED_R, round(rewLEDduty*0.75));
		analogWrite(pin_RewLED_C, rewLEDduty);
		// Open solenoid
		digitalWrite(pin_Rel_Rew, HIGH);

		// Print to LCD for manual rewards
		if (btn_doRew)
		{
			PrintLCD("REWARDING...");
		}
		else
		{
			char str[50];
			sprintf(str, "REWARDING(%dms)...", duration);
			DebugFlow(str);
		}

		// indicate reward in progress
		return is_rewarding;

	}

	// END REWARD
	bool EndRew()
	{

		// Local vars
		bool do_continue_rew = true;

		if (millis() > t_closeSol)
		{

			// Close solenoid
			digitalWrite(pin_Rel_Rew, LOW);

			// Turn off reward LED
			analogWrite(pin_RewLED_R, rewLEDmin);
			analogWrite(pin_RewLED_C, rewLEDmin);

			// Clear LCD
			if (btn_doRew)
			{
				ClearLCD();
			}
			else
			{
				DebugFlow("REWARD FINISHED");
			}

		}
		else do_continue_rew = false;
		return do_continue_rew;

	}

	// Set reward duration
	void SetRewDur(uint32_t dur)
	{
		duration = dur;
		durationByte = (byte)(duration / 10);
	}
	void SetRewDur(byte dur_byte)
	{
		SetRewDur((uint32_t)dur_byte * 10);
	}

	// Compute target bounds
	bool CompTargBounds(float now_pos, float rew_pos)
	{
		// Run only if bounds are not set
		if (!isboundSet)
		{
			// Local vars
			int diam = 0;
			int pos = 0;
			float pos_rel = 0;
			float dist_center_cm = 0;
			float dist_start_cm = 0;
			float dist_end_cm = 0;

			// Compute laps
			diam = (int)(140 * PI * 100);
			pos = (int)(now_pos * 100);
			lapN = round(now_pos / (140 * PI) - (float)(pos % diam) / diam);
			// Check if rat 'ahead' of rew pos
			pos_rel = (float)(pos % diam) / 100;
			// add lap
			lapN = pos_rel > rew_pos ? lapN + 1 : lapN;

			// Compute reward center
			rewCenter = rew_pos + lapN*(140 * PI);

			// Compute bounds for each targ
			for (int i = 0; i < targLng; i++)
			{
				// Compute 5 deg bounds
				dist_center_cm = -1 * targLocs[i] * ((140 * PI) / 360);
				dist_start_cm = dist_center_cm - (2.5 * ((140 * PI) / 360));
				dist_end_cm = dist_center_cm + (2.5 * ((140 * PI) / 360));
				// Store in array
				targBounds[i][0] = rewCenter + dist_start_cm;
				targBounds[i][1] = rewCenter + dist_end_cm;
				// Save bound min/max
				boundMin = i == 0 ? targBounds[i][0] : boundMin;
				boundMax = i == targLng - 1 ? targBounds[i][1] : boundMax;
			}
			// Set flag
			isboundSet = true;
		}
		return isboundSet;
	}

	// Check bounds
	bool CheckTargBounds(float now_pos, float now_vel)
	{
		// Run only if reward not already triggered
		if (!isTriggerReady)
		{

			// Check if all bounds passed
			if (now_pos > boundMax + 5)
			{
				isAllRargPassed = true;
				return isTriggerReady;
			}
			// Check velocity
			else if (
				now_pos > boundMin &&
				now_vel <= velThresh
				)
			{

				// Check if rat in any bounds
				for (int i = 0; i < targLng; i++)
				{
					if (
						now_pos > targBounds[i][0] &&
						now_pos < targBounds[i][1]
						)
					{
						// Reward at this pos
						SetRewDur(targRewDurs[i]);

						// Store rewarded target for debugging
						rewardedTarg = targLocs[i];
						rewardedBounds[0] = targBounds[i][0];
						rewardedBounds[1] = targBounds[i][1];

						// Set flag
						isTriggerReady = true;
					}
				}
			}

			// Reset flag
			is_ekfNew = false;
		}
		return isTriggerReady;
	}

	// Check if feeder arm should be moved
	void CheckFeedArm()
	{
		if (doArmMove)
		{
			// Check if arm should be moved
			if (armPos != armTarg)
			{
				MoveFeedArm();
			}
			// Target reached
			else
			{
				// Unstep motor
				if (digitalRead(pin_ED_STP) == HIGH)
				{
					digitalWrite(pin_ED_STP, LOW);
					armStpOn = false;
				}
				// Sleep motor
				if (digitalRead(pin_ED_SLP) == HIGH)
				{
					digitalWrite(pin_ED_SLP, LOW);
				}

				// Set flag
				if (!isArmExtended && armPos > 0)
					isArmExtended = true;
				else if (isArmExtended && armPos == 0)
					isArmExtended = false;
				doArmMove = false;
			}
		}
		// Check if its time to retract arm
		else if (
			isArmExtended &&
			millis() > t_retractArm
			)
		{
			RetractFeedArm();
		}
	}

	// Reset feeder arm
	void ResetFeedArm()
	{
		isArmExtended = true;
		armPos = armExtStps;
		RetractFeedArm();
	}

	// Extend feeder arm
	void ExtendFeedArm()
	{
		if (!isArmExtended)
		{
			t_retractArm = millis() + t_block;
			armTarg = armExtStps;
			doArmMove = true;
		}
	}

	// Retract feeder arm
	void RetractFeedArm()
	{
		if (isArmExtended)
		{
			armTarg = 0;
			doArmMove = true;
		}
	}

	// Move feeder arm
	void MoveFeedArm()
	{

		// Wake motor
		if (digitalRead(pin_ED_SLP) == LOW)
		{
			digitalWrite(pin_ED_SLP, HIGH);
		}

		// Step motor
		if (!armStpOn && millis() > t_lastOffStep + 1)
		{

			// Extend arm
			if (armPos < armTarg)
			{
				armPos++;
				digitalWrite(pin_ED_DIR, LOW); // extend
			}

			// Retract arm
			else
			{
				if (digitalRead(pin_FeedSwitch) == HIGH)
				{
					digitalWrite(pin_ED_DIR, HIGH); // retract
				}
				// Home pos reached
				else
				{
					// Take presure off botton
					armPos = -20;
					isArmExtended = false;
				}
			}

			// Set step high
			digitalWrite(pin_ED_STP, HIGH);

			// Save step time
			t_lastOnStep = millis();

			// Set flag
			armStpOn = true;
		}
		// Unstep motor
		else if (armStpOn && millis() > t_lastOnStep + 1)
		{
			// Set step low
			digitalWrite(pin_ED_STP, LOW);

			// Save step time
			t_lastOffStep = millis();

			// Set flag
			armStpOn = false;
		}
	}

	// Reset
	void Reset()
	{
		isboundSet = false;
		isTriggerReady = false;
		isAllRargPassed = false;
		is_ekfNew = false;
	}
};
// Initialize object
Reward reward;

//----------CLASS: Fuser----------
class Fuser : public TinyEKF
{

public:

	Fuser()
	{
		// We approximate the process noise using a small constant
		this->setQ(0, 0, .0001); // Rat pos
		this->setQ(1, 1, .0001); // Rob pos
		this->setQ(2, 2, .0001); // Rat vel
		this->setQ(3, 3, .0001); // Rob vel

								 // Same for measurement noise
		this->setR(0, 0, .001); // Rat pos vt
		this->setR(1, 1, .001); // Rat pos pixy
		this->setR(2, 2, .0001); // Rob pos vt
		this->setR(3, 3, .01); // Rat vel vt
		this->setR(4, 4, .01); // Rat vel pixy
		this->setR(5, 5, .001); // Rob vel vt
	}

protected:

	void model(double fx[N], double F[N][N], double hx[M], double H[M][N])
	{
		// Process model is f(x) = x
		fx[0] = this->x[0];
		fx[1] = this->x[1];
		fx[2] = this->x[2];
		fx[3] = this->x[3];

		// So process model Jacobian is identity matrix
		F[0][0] = 1;
		F[1][1] = 1;
		F[2][2] = 1;
		F[3][3] = 1;

		// Measurement function
		hx[0] = this->x[0]; // Rat pos vt from previous state
		hx[1] = this->x[0]; // Rat pos pixy from previous state
		hx[2] = this->x[1]; // Rob pos vt from previous state
		hx[3] = this->x[2]; // Rat vel vt from previous state
		hx[4] = this->x[2]; // Rat vel pixy from previous state
		hx[5] = this->x[3]; // Rob vel vt from previous state

							// Jacobian of measurement function
		H[0][0] = 1; // Rat pos vt from previous state
		H[1][0] = 1; // Rat pos pixy from previous state
		H[2][1] = 1; // Rob pos vt from previous state
		H[3][2] = 1; // Rat vel vt from previous state
		H[4][2] = 1; // Rat vel pixy from previous state
		H[5][3] = 1; // Rob vel vt from previous state
	}
};
// Initialize object
Fuser ekf;


//----------CLASS: union----------
union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (int16) 2 byte
	uint32_t i32; // (int32) 4 byte
	int i; // (int) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
}
// Initialize object
u;

#pragma endregion 


// ---------SETUP---------
void setup() {

	delayMicroseconds(100);

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// XBee
	Serial1.begin(57600);

	// SETUP OUTPUT POWER AND GROUND PINS

	// Set output pins
	pinMode(pin_Rel_Rew, OUTPUT);
	pinMode(pin_Rel_EtOH, OUTPUT);
	pinMode(pin_ED_STP, OUTPUT);
	pinMode(pin_ED_DIR, OUTPUT);
	pinMode(pin_ED_SLP, OUTPUT);
	pinMode(pin_ED_MS1, OUTPUT);
	pinMode(pin_ED_MS2, OUTPUT);
	pinMode(pin_ED_ENBL, OUTPUT);
	pinMode(pin_PwrOff, OUTPUT);
	pinMode(pin_FeedSwitch_Gnd, OUTPUT);

	// Set power/ground pins
	digitalWrite(pin_FeedSwitch_Gnd, LOW);

	// Set button pins enable internal pullup
	for (int i = 0; i <= 2; i++) {
		pinMode(pin_Btn[i], INPUT_PULLUP);
	}
	pinMode(pin_FeedSwitch, INPUT_PULLUP);

	// Make sure relay pins low
	digitalWrite(pin_Rel_Rew, LOW);
	digitalWrite(pin_Rel_EtOH, LOW);

	// SETUP AUTODRIVER

	// Configure SPI
	ad_R.SPIConfig();
	delayMicroseconds(100);
	ad_F.SPIConfig();
	delayMicroseconds(100);
	// Reset each axis
	ad_R.resetDev();
	delayMicroseconds(100);
	ad_F.resetDev();
	delayMicroseconds(100);
	// Configure each axis
	dSPINConfig_board();
	delayMicroseconds(100);
	// Get the status to clear the UVLO Flag
	ad_R.getStatus();
	delayMicroseconds(100);
	ad_F.getStatus();

	// Make sure motor is stopped and in high impedance
	ad_R.hardHiZ();
	ad_F.hardHiZ();

	// Print ad board status
	SerialUSB.println(' ');
	SerialUSB.print("BOARD R STATUS: ");
	SerialUSB.println(ad_R.getStatus(), HEX);
	SerialUSB.print("BOARD F STATUS: ");
	SerialUSB.println(ad_F.getStatus(), HEX);

	// SETUP BIG EASY DRIVER

	// Set to 1/2 step mode
	digitalWrite(pin_ED_MS1, HIGH);
	digitalWrite(pin_ED_MS2, LOW);
	digitalWrite(pin_ED_MS3, LOW);

	// Start BigEasyDriver in sleep
	digitalWrite(pin_ED_SLP, LOW);

	// DEFINE EXTERNAL INTERUPTS

	// IR prox right
	attachInterrupt(digitalPinToInterrupt(pin_IRprox_Rt), Interupt_IRprox_Halt, FALLING);
	// IR prox left
	attachInterrupt(digitalPinToInterrupt(pin_IRprox_Lft), Interupt_IRprox_Halt, FALLING);
	// IR detector
	attachInterrupt(digitalPinToInterrupt(pin_IRdetect), Interupt_IR_Detect, HIGH);

	// INITIALIZE LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);
	myGLCD.invert(true);

	// INITIALIZE PIXY
	pixy.init();
	Wire.begin();

	// INITIALIZE ARRAY VARIABLES

	// Initialize print queue
	for (int i = 0; i < printQueue_lng; i++)
	{
		printQueue[i] = " ";
	}

	// Initialize send msg matrix
	for (int i = 0; i < r2_lngR; i++)
	{
		for (int j = 0; j < r2_lngC; j++)
		{
			r2_queue[i][j] = 0;
		}
	}

	// Initilize packet history
	for (int i = 0; i < c2r_hLng; i++)
	{
		c2r_packHist[i] = 0;
		c2r_idHist[i] = ' ';
	}
	for (int i = 0; i < r2c_hLng; i++)
	{
		r2c_packHist[i] = 0;
		r2c_idHist[i] = ' ';
	}

	// Initialize last packet and repeat packet array
	for (int i = 0; i < c2r_idLng; i++)
	{
		c2r_packLast[i] = 0;
		c2r_packRepeat[i] = 0;
		c2r_cntRepeat[i] = 0;
	}
	for (int i = 0; i < r2c_idLng; i++)
	{
		r2c_packLast[i] = 0;
	}

	// Initialize bat volt array
	for (int i = 0; i < 100; i++)
	{
		batVoltArr[i] = 0;
	}
	delay(5000);
	myGLCD.invert(false);
	delay(5000);
	myGLCD.invert(true);
}


// ---------MAIN LOOP---------
void loop() {

#pragma region //--- DEBUG ---

	// Store loop time
	t_loopMain = millis();

	// Print debug
	if (doPrint)
	{
		PrintDebug();
	}

	// Debug pos
	if (do_posDebug && fc_isRatIn)
	{
		static float rat_rob_dist;
		if (millis() % 100 == 0)
		{
			// Hold all motor control
			if (!fc_isHalted)
			{
				fc_isHalted = true;
				SetMotorControl("None", "PosDebug");
			}
			rat_rob_dist = ekfRatPos - ekfRobPos;
			// Plot pos
			millis();
			// Turn on rew led when near setpoint
			if (errorDefault > -0.5 && errorDefault < 0.5) { analogWrite(pin_RewLED_C, 50); }
			else { analogWrite(pin_RewLED_C, 0); }
			// Print to LCD
			analogWrite(pin_Disp_LED, 25);
			PrintLCD(String(rat_rob_dist, 2));
		}
	}

	// Run PID calibration
	if (do_pidCalibration)
	{
		float new_speed = pid.RunPidCalibration();
		float speed_steps;

		// Run motors
		if (pid.cal_isPidUpdated)
		{
			if (new_speed <= 0)
			{
				HardStop("PID");
			}
			else if (new_speed > 0)
			{
				speed_steps = new_speed*cm2stp;
				ad_R.run(FWD, speed_steps);
				ad_F.run(FWD, speed_steps*frontMoterScale);
			}
			// Print/plot values
			millis();
			// Reset flag
			pid.cal_isPidUpdated = false;
		}
	}

#pragma endregion

#pragma region //--- FIRST PASS SETUP ---
	if (fc_isFirstPass)
	{

		// Make sure Xbee buffer empty
		while (Serial1.available() > 0) Serial1.read();

		// Make sure relays are off
		digitalWrite(pin_Rel_Rew, LOW);
		digitalWrite(pin_Rel_EtOH, LOW);

		// Clear LCD
		ClearLCD();

		// Reset volatiles
		btn_doChangeSolState = false;
		btn_doRew = false;
		btn_doChangeLCDstate = false;
		intrpt_doIRhardStop = false;

		fc_isFirstPass = false;
		DebugFlow("RESET");

		// Blink to show setup done
		SetupBlink();

		//// TEST
		//float now_pos = 348 + (140 * PI);
		//float rew_pos = 92;
		//for (int i = 0; i < 100; i++)
		//{
		//	reward.is_bound_set = false;
		//	reward.CompTargBounds(now_pos, rew_pos);
		//	now_pos = now_pos + 10 + (140 * PI);
		//}

		/*
		// TEST
		for (int i = 0; i < 100; i++)
		{
			char chr[50];
			sprintf(chr, "Log %d: Bunch of stuff", i);
			String str = chr;
			StoreDBLogStr(str, millis());
		}
		*/

	}

#pragma endregion

#pragma region //--- PARSE SERIAL ---
	msg_pass = false; // reset before next loop
	if (Serial1.available() > 0)
	{
		msg_pass = ParseSerial();
	}
#pragma endregion

#pragma region //--- SEND SERIAL DATA ---
	if (doSend)
	{
		SendPacketData();
	}
#pragma endregion

#pragma region //--- (T) SYSTEM TEST ---
	if (msg_id == 'T' && msg_pass)
	{
		if (!fc_isHalted || fc_isHalted)
		{
			// Update Hault Error test run speed
			if (msg_setupCmd[0] == 2)
			{
				float new_speed = float(msg_setupCmd[1]);
				float speed_steps = new_speed*cm2stp;


				if (new_speed > 0)
				{
					// Run motor
					ad_R.run(FWD, speed_steps);
					ad_F.run(FWD, speed_steps*frontMoterScale);
				}
				else
				{
					// Halt robot
					ad_R.hardStop();
					ad_F.hardStop();
				}

				// Print speed
				char str[50];
				sprintf(str, "HAULT ERROR SPEED = %0.0f cm/sec", new_speed);
				SerialUSB.println(str);
			}
		}
	}
#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (msg_id == 'S' && msg_pass)
	{
		// Set condition
		if (msg_setupCmd[0] == 1)
		{
			fc_isManualSes = false;
			DebugFlow("DO TRACKING");
		}
		else
		{
			fc_isManualSes = true;
			DebugFlow("DONT DO TRACKING");
		}
		// Set reward tone
		if (msg_setupCmd[1] == 0)
		{
			// No sound
			Store4_Ard('s', 0);
			DebugFlow("NO SOUND");
		}
		else if (msg_setupCmd[1] == 1)
		{
			// Use white noise only
			Store4_Ard('s', 1);
			DebugFlow("DONT DO TONE");
		}
		else
		{
			// Use white and reward noise
			Store4_Ard('s', 2);
			DebugFlow("DO TONE");
		}

		// Make sure lcd led is off
		if (!fc_isManualSes &&
			lcdLightOn)
		{
			btn_doChangeLCDstate = true;
		}
		// Clear LCD
		ClearLCD();
	}
#pragma endregion

#pragma region //--- (Q) DO QUIT ---
	if (msg_id == 'Q' && msg_pass)
	{
		fc_doQuit = true;
		t_quitCmd = millis() + 1000;

		// Tell ard to quit
		Store4_Ard('q', 255);
		DebugFlow("DO QUIT");

		// Hold all motor control
		fc_isHalted = true;
		SetMotorControl("None", "MsgQ");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (fc_doQuit && millis() > t_quitCmd && !doSend)
	{
		// Retell ard to quit
		Store4_Ard('q', 255);
		DebugFlow("QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (msg_id == 'M' && msg_pass)
	{
		fc_doMove = true;
		DebugFlow("DO MOVE");
	}

	// Perform movement
	if (fc_doMove)
	{

		// Compute move target
		if (!targ_moveTo.isTargSet)
		{
			// If succesfull
			if (targ_moveTo.CompTarg(ekfRobPos, msg_moveToTarg, -1 * feedDist))
			{
				// Start running
				if (
					SetMotorControl("MoveTo", "MsgM") &&
					RunMotor(targ_moveTo.moveDir, moveToSpeed, "MoveTo")
					)
				{
					// Print message
					char str[50];
					sprintf(str, "MOVEING FROM %0.2fcm TO %0.2fcm BY %0.2fcm",
						targ_moveTo.posStart, targ_moveTo.offsetTarget, targ_moveTo.targDist);
					DebugFlow(str);
				}
				// Reset motor cotrol if run fails
				else SetMotorControl("Open", "MsgM");
			}
		}

		// Check if robot is ready to be stopped
		if (targ_moveTo.isTargSet)
		{
			// Do deceleration
			float new_speed = targ_moveTo.DecelToTarg(ekfRobPos, ekfRobVel, 40, 10);

			// Change speed if > 0
			if (new_speed > 0)
			{
				RunMotor(targ_moveTo.moveDir, new_speed, "MoveTo");
			}
			// Check if target reached
			else if (targ_moveTo.isTargReached)
			{
				// Hard stop
				HardStop("MsgM");

				// Hold motor control for 5 sec
				BlockMotorTill(5000);

				// Tell CS movement is done
				Store4_CS('D', 255, c2r_packLast[CharInd('M', "c2r")]);

				// Reset flags
				fc_doMove = false;
				targ_moveTo.Reset();

				// Print message
				char str[50];
				sprintf(str, "FINISHED MOVE TO %0.2fcm WITHIN %0.2fcm",
					targ_moveTo.offsetTarget, targ_moveTo.GetError(ekfRobPos));
				DebugFlow(str);
			}
		}

	}
#pragma endregion

#pragma region //--- (R) RUN REWARD ---
	if (msg_id == 'R' && msg_pass)
	{

		if (msg_rewPos == 0)
		{
			// Set reward duration in ms
			reward.SetRewDur(msg_rewDurByte);

			DebugFlow("REWARD NOW");
			// Start reward
			fc_isRewarding = reward.StartRew(true);
		}
		else
		{
			fc_doRew = true;
			DebugFlow("RUN REWARD");
		}
	}

	// Perform reward
	if (fc_doRew)
	{
		// If not rewarding 
		if (!fc_isRewarding)
		{
			// Compute reward bounds
			if (!reward.isboundSet)
			{
				reward.CompTargBounds(ekfRatPos, msg_rewPos);
				// Print message
				char str[50];
				sprintf(str, "REWARD AT %0.2fcm FROM %0.2fcm TO %0.2fcm",
					reward.rewCenter, reward.targBounds[0][0], reward.targBounds[8][1]);
				DebugFlow(str);
			}
			else if (!reward.isTriggerReady)
			{
				bool ekf_pass = reward.CheckTargBounds(ekfRatPos, ekfRatVel);
				bool raw_pass = false; // 
				if (ekf_pass || raw_pass)
				{
					// Start reward
					fc_isRewarding = reward.StartRew(true);
					// Print message
					char str[50];
					sprintf(str, "REWARDED TARG %0.2fcm BOUNDS %0.2fcm TO %0.2fcm USING %s",
						reward.rewardedTarg, reward.rewardedBounds[0], reward.rewardedBounds[1], ekf_pass ? "EKF" : "Raw");
					DebugFlow(str);
				}
			}
			// Check if rat bassed all bounds
			if (
				reward.isAllRargPassed &&
				!reward.isTriggerReady
				)
			{
				// Reset flags
				reward.Reset();
				fc_doRew = false;
			}
		}
		// End ongoing reward
		else if (reward.EndRew())
		{
			// Reset flags
			reward.Reset();
			fc_doRew = false;
		}
	}

#pragma endregion

#pragma region //--- (C) CUE REWARD ---

	if (msg_id == 'C' && msg_pass)
	{
		// Set reward duration in ms
		reward.SetRewDur(msg_rewDurByte);

		// Cue reward
		fc_doCueReward = true;
	}

	// Perform cued reward
	if (fc_doCueReward)
	{
		// If not rewarding 
		if (!fc_isRewarding || !fc_isCueing)
		{

			// Compute target targ_dist
			if (!(targ_cueRat.isTargSet && targ_cueRob.isTargSet))
			{
				// Compute target targ_dist for rat and robot
				if (
					targ_cueRat.CompTarg(ekfRatPos, msg_cueTarg, 0) &&
					targ_cueRob.CompTarg(ekfRobPos, msg_cueTarg, -1 * defualtSetPoint)
					)
				{
					// Print message
					char str[50];
					sprintf(str, "CUEING REWARD AT %0.2fcm/%0.2fcm FROM DIST %0.2fcm/%0.2fcm",
						targ_cueRat.offsetTarget, targ_cueRob.offsetTarget, targ_cueRat.targDist, targ_cueRob.targDist);
					DebugFlow(str);
				}
			}

			// Check if cue has been reached
			if (targ_cueRat.isTargSet && targ_cueRob.isTargSet)
			{
				// Check if robot reached targed
				if (!targ_cueRob.isTargReached)
				{
					if (
						targ_cueRob.CheckTargReached(pos_robVT.posNow, ekfRobVel) ||
						targ_cueRob.CheckTargReached(ekfRobPos, ekfRobVel)
						)
					{
						DebugFlow("ROBOT REACHED CUE TARGET");
						// Hard stop
						HardStop("MsgC");
						// Set flag
						fc_isCueing = true;
						// Set hold time
						BlockMotorTill(reward.t_block);
					}
				}
				// Check if rat reached target
				if (!targ_cueRat.isTargReached)
				{
					if (
						targ_cueRat.CheckTargReached(vtpixyPosAvg, 0) ||
						targ_cueRat.CheckTargReached(ekfRatPos, 0)
						)
					{
						DebugFlow("RAT REACHED CUE TARGET");
						// Trigger reward without stopping
						fc_isRewarding = reward.StartRew(false);
					}
				}
				if (targ_cueRat.isTargReached && targ_cueRob.isTargReached)
				{
					// Print message
					char str[50];
					sprintf(str, "FINISHED CUEING AT %0.2fcm/%0.2fcm WITHIN %0.2fcm/%0.2fcm",
						targ_cueRat.offsetTarget, targ_cueRob.offsetTarget, targ_cueRat.GetError(ekfRatPos), targ_cueRob.GetError(ekfRobPos));
					DebugFlow(str);
				}
			}
		}

		// End ongoing reward
		else if (reward.EndRew())
		{
			// Reset flags
			targ_cueRat.Reset();
			targ_cueRob.Reset();
			fc_doCueReward = false;
			fc_isCueing = false;
		}
	}

#pragma endregion

#pragma region //--- (H) HALT ROBOT STATUS ---
	if (msg_id == 'H' && msg_pass)
	{
		if (fc_doHalt)
		{
			DebugFlow("HALT STARTED");
			// Stop pid and set to manual
			HardStop("MsgH");
			// Remove motor control
			fc_isHalted = true;
			SetMotorControl("None", "MsgH");
			fc_doHalt = false;
		}
		else
		{
			DebugFlow("HALT FINISHED");
			// Open motor control
			fc_isHalted = false;
			SetMotorControl("Open", "MsgH");
		}
	}
#pragma endregion

#pragma region //--- (B) BULLDOZE RAT STATUS ---
	if (msg_id == 'B' && msg_pass)
	{
		// Local vars
		bool is_mode_changed = false;

		// Reinitialize bulldoze
		bull.Reinitialize(msg_bullDel, msg_bullSpeed, "MsgB");

		// Check if mode should be changedchanged
		if (msg_bullSpeed > 0)
		{
			// Mode changed
			if (!fc_doBulldoze)
			{
				is_mode_changed = true;
				fc_doBulldoze = true;
				DebugFlow("DO BULLDOZE");
			}
			// Only settings changed
			else is_mode_changed = false;
		}
		else
		{
			// Mode changed
			if (fc_doBulldoze)
			{
				is_mode_changed = true;
				fc_doBulldoze = false;
				DebugFlow("DONT DO BULLDOZE");
			}
			// Only settings changed
			else is_mode_changed = false;
		}

		// Don't exicute until rat is in and mode is changed
		if (fc_isTrackingEnabled &&
			is_mode_changed)
		{
			if (fc_doBulldoze)
			{
				// Turn bulldoze on
				bull.TurnOn("MsgB");
				DebugFlow("BULLDOZE ON");
			}
			else
			{
				// Turn bulldoze off
				bull.TurnOff("MsgB");
				DebugFlow("BULLDOZE OFF");
			}
		}

	}
#pragma endregion

#pragma region //--- (I) START/STOP PID ---
	if (msg_id == 'I' && msg_pass)
	{
		if (fc_isRatIn)
		{
			// Reset all vt data
			pos_ratVT.ResetDat(0, 0);
			pos_ratPixy.ResetDat(0, 0);
			pos_robVT.ResetDat(0, 0);

			// Pid started by InitializeTracking()
			DebugFlow("RAT IN");
		}
		else
		{

			// Turn off bulldoze
			bull.TurnOff("MsgI");
			fc_doBulldoze = false;

			// Turn off pid
			pid.Stop("MsgI");

			// Set motor control to "None"
			SetMotorControl("None", "MsgI");

			// Halt robot 
			HardStop("MsgI");

			// Set to stop tracking
			fc_isTrackingEnabled = false;

			DebugFlow("RAT OUT");
		}
	}
#pragma endregion

#pragma region //--- (V) GET STREAM STATUS ---
	if (msg_id == 'V' && msg_pass)
	{
		fc_doStreamCheck = true;
	}

	// Check for streaming
	if (fc_doStreamCheck && fc_isStreaming)
	{
		Store4_CS('D', 255, c2r_packLast[CharInd('V', "c2r")]);
		fc_doStreamCheck = false;
		DebugFlow("STREAMING CONFIRMED");
	}
#pragma endregion

#pragma region //--- (Y) DONE RECIEVED ---
	if (msg_id == 'Y' && msg_pass)
	{
		fc_doCheckDoneRcvd = false;
		DebugFlow("CS RESIEVED 'DONE'");
	}
	// Request done recieved confirmation
	if (fc_doCheckDoneRcvd && millis() > t_resendDone)
	{
		// Resend done confirmation
		Store4_CS('D', 255, r2c_packLast[CharInd('D', "r2c")]);
		// Send again after 1 sec
		t_resendDone = millis() + 1000;
		DebugFlow("RESENT 'D'");
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (msg_id == 'P' && msg_pass)
	{
		// Update vt pos data
		if (msg_vtEnt == 0)
		{
			pos_ratVT.UpdatePosVel(msg_vtCM[msg_vtEnt], msg_vtTS[msg_vtEnt]);
		}
		else
		{
			pos_robVT.UpdatePosVel(msg_vtCM[msg_vtEnt], msg_vtTS[msg_vtEnt]);
		}
	}
#pragma endregion

#pragma region //--- (L) SEND LOG ---
	if (msg_id == 'L' && msg_pass)
	{
		bool do_resend;
		// Send log data
		if (msg_setupCmd[0] == 1)
		{
			do_resend = false;
			DebugFlow("SEND NEXT LOG PACKET");
		}
		else
		{
			do_resend = true;
			DebugFlow("RESEND LAST LOG PACKET");
		}
		// Send log data if end not reached
		if (!SendLogData(do_resend))
		{
			// Send confirm done
			Store4_CS('D', 255, c2r_packLast[CharInd('L', "c2r")]);
		}
	}

	if (fc_doLogSend)
	{


	}
	else if (fc_doLogResend)
	{

	}
#pragma endregion

#pragma region //--- BUTTON/INTERUPT TRIGGERED ---

	// Check for button input
	CheckButtons();

	// Open/close solonoid
	if (btn_doChangeSolState)
	{
		OpenCloseRewSolenoid();
		btn_doChangeSolState = false;
	}

	// Button triggered reward
	if (btn_doRew)
	{
		if (!fc_isRewarding)
		{
			fc_isRewarding = reward.StartRew(false);
		}
		else if (reward.EndRew())
		{
			btn_doRew = false;
		}
	}

	// Turn LCD on/off
	if (btn_doChangeLCDstate)
	{
		ChangeLCDlight();
		btn_doChangeLCDstate = false;
	}

	// IR triggered halt
	if (intrpt_doIRhardStop)
	{
		Function_IRprox_Halt();
		intrpt_doIRhardStop = false;
	}


#pragma endregion

#pragma region //--- RUN TRACKING ---

	// UPDATE PIXY
	UpdatePixyPos();

	// INITIALIZE RAT AHEAD
	InitializeTracking();

	// CHECK IF RAT POS FRAMES DROPPING
	CheckSampDT();

	// UPDATE EKF
	UpdateEKF();

	// CHECK IF STILL BLOCKING
	CheckBlockTimElapsed();

	// UPDATE PID AND SPEED
	float new_speed = pid.UpdatePID();

	if (new_speed == 0)
	{
		HardStop("PID");
	}
	else if (new_speed > 0)
	{
		RunMotor('f', new_speed, "PID");
	}

	// UPDATE BULLDOZER
	bull.UpdateBull();

#pragma endregion

#pragma region //--- OTHER OPPERATIONS ---

	// Check if feeder arm should be moved
	reward.CheckFeedArm();

	// Check if EtOH should be dispensed
	CheckEtOH();

	// Check voltage sensor
	CheckBattery();

	// Log new ir events
	if (doLogIR) DebugIRSync("IR Detected");

#pragma endregion

}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{
	// Local vars
	byte buff = 0;
	char head[2] = { ' ',' ' };
	char foot = ' ';
	bool pass = false;
	bool loop = 0;
	msg_id == ' ';

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(c2r_head[0]);
	if (buff == 255)
	{
		return pass = false;
	}

	// get header
	u.f = 0.0f;
	u.b[0] = buff;
	u.b[1] = WaitBuffRead(0);
	head[0] = u.c[0];
	head[1] = u.c[1];

	// get packet num
	u.f = 0.0f;
	u.b[0] = WaitBuffRead(0);
	u.b[1] = WaitBuffRead(0);
	packNow = u.i16[0];
	// get id
	u.f = 0.0f;
	u.b[0] = WaitBuffRead(0);
	msg_id = u.c[0];

	// Check for second header
	if (head[1] != c2r_head[1])
	{
		// mesage will be dumped
		return pass = false;
	}

	// Get system test data
	if (msg_id == 'T')
	{

		// Get test id
		msg_setupCmd[0] = WaitBuffRead(0);

		// Get test argument
		msg_setupCmd[1] = WaitBuffRead(0);

	}

	// Get setup data
	if (msg_id == 'S')
	{

		// Get session comand
		msg_setupCmd[0] = WaitBuffRead(0);

		// Get tone condition
		msg_setupCmd[1] = WaitBuffRead(0);

	}

	// Get MoveTo data
	if (msg_id == 'M')
	{

		// Reset buffer
		u.f = 0.0f;

		// Get move posm
		u.b[0] = WaitBuffRead(0);
		u.b[1] = WaitBuffRead(0);
		u.b[2] = WaitBuffRead(0);
		u.b[3] = WaitBuffRead(0);
		msg_moveToTarg = u.f;

	}

	// Get Reward data
	if (msg_id == 'R')
	{
		// Reset buffer
		u.f = 0.0f;

		// Get stop pos
		u.b[0] = WaitBuffRead(0);
		u.b[1] = WaitBuffRead(0);
		u.b[2] = WaitBuffRead(0);
		u.b[3] = WaitBuffRead(0);
		msg_rewPos = u.f;

		// Get reward diration 
		msg_rewDurByte = WaitBuffRead(0);
	}

	// Get Cue Reward data
	if (msg_id == 'C')
	{
		// Reset buffer
		u.f = 0.0f;

		// Get stop pos
		u.b[0] = WaitBuffRead(0);
		u.b[1] = WaitBuffRead(0);
		u.b[2] = WaitBuffRead(0);
		u.b[3] = WaitBuffRead(0);
		msg_cueTarg = u.f;

		// Get reward diration 
		msg_rewDurByte = WaitBuffRead(0);
	}

	// Get halt robot data
	if (msg_id == 'H')
	{
		// Get halt bool
		byte do_halt = WaitBuffRead(0);
		fc_doHalt = do_halt == 1 ? true : false;
	}

	// Get bulldoze rat data
	if (msg_id == 'B')
	{
		// Get delay in sec
		msg_bullDel = WaitBuffRead(0);
		// Get speed
		msg_bullSpeed = WaitBuffRead(0);
	}

	// Get rat in/out
	if (msg_id == 'I')
	{
		// Get session comand
		byte rat_in = WaitBuffRead(0);
		fc_isRatIn = rat_in == 1 ? true : false;
	}

	// Get VT data
	else if (msg_id == 'P')
	{
		// Get Ent
		msg_vtEnt = WaitBuffRead(0);
		// Get TS
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = WaitBuffRead(0);
		}
		msg_vtTS[msg_vtEnt] = u.i32;
		// Get pos cm
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = WaitBuffRead(0);
		}
		msg_vtCM[msg_vtEnt] = u.f;
	}

	// Check for footer
	u.f = 0.0f;
	u.b[0] = WaitBuffRead(0);
	foot = u.c[0];

	// Footer missing
	if (foot != c2r_foot) {
		// mesage will be dumped
		return pass = false;
	}
	// Process complete mesage
	else
	{
		// Update recive time
		t_rsvdLast = t_rsvd;
		t_rsvd = millis();

		// CHECK FOR STREAMING STARTED
		if (msg_id == 'P' && !fc_isStreaming)
		{
			// Signal streaming started
			fc_isStreaming = true;

			// send sync start cmd to ard
			Store4_Ard('t', 255);
		}

		// Process packet number
		pass = CheckPack(msg_id, packNow);

		// Return final status
		return pass;

	}

}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(byte match)
{
	// Local vars
	static uint32_t time_out = 100;
	uint32_t t_out = millis() + time_out;
	byte buff = 0;
	bool pass = false;

	// Check for overflow
	if (Serial1.available() < SERIAL_BUFFER_SIZE - 1)
	{
		// Wait for at least 1 byte
		while (Serial1.available() < 1 &&
			millis() < t_out);

		// Store next byte
		if (match == 0)
		{
			if (Serial1.available() > 0)
			{
				buff = Serial1.read();
				pass = true;
			}
		}
		// Find specific byte
		else
		{
			// loop till match found 
			do
			{
				if (Serial1.available() > 0)
				{
					buff = Serial1.read(); // dump
				}
			} while (buff != match &&
				millis() < t_out &&
				Serial1.available() < SERIAL_BUFFER_SIZE - 1);
			// check match was found
			if (buff == match)
			{
				pass = true;
			}
		}
	}

	// Failed
	if (!pass)
	{

		// Buffer flooded
		if (Serial1.available() >= SERIAL_BUFFER_SIZE - 1)
		{
			// DUMP IT ALL
			while (Serial1.available() > 0)
			{
				Serial1.read();
			}
			cnt_overflowEvt++;
		}

		// Timed out
		if (millis() > t_out)
		{
			cnt_timeoutEvt++;
		}

		// Set buff to 255 ((byte)-1) if !pass
		buff = 255;
	}

	// Store time
	t_loopRead = millis() - (t_out - time_out);
	// Return buffer
	return buff;
}

// CHECK AND PROCESS RSVD PACKET NUMBER
bool CheckPack(char id, uint16_t pack)
{
	// Local vars
	bool pass = false;
	int pack_diff = 0;
	int dropped_packs = 0;
	bool do_use_pack = false;
	int id_ind = 0;

	// SEND AND PRINT PACKET CONFIRMATON
	if (id != 'P')
	{
		// Print revieved pack details
		DebugRsvd(id, pack);

		// Send back packet confirmation
		if (id != 'Y')
		{
			// Send revieved pack
			Store4_CS(id, 255, pack);
		}
	}

	// Update packet history
	if (id != 'P')
	{
		for (int i = 0; i < c2r_hLng - 1; i++)
		{
			c2r_packHist[i] = c2r_packHist[i + 1];
			c2r_idHist[i] = c2r_idHist[i + 1];
		}
		c2r_packHist[c2r_hLng - 1] = pack;
		c2r_idHist[c2r_hLng - 1] = id;
	}

	// CHECK FOR DROPPED PACKETS
	pack_diff = (int)(pack - packLast);

	if (pack_diff > 0)
	{
		// Save packet and set to process
		packLast = pack;

		// Check for dropped packet
		packTot += pack_diff;
		dropped_packs = pack_diff - 1;
		// print dropped packs
		if (dropped_packs > 0)
		{
			cnt_droppedPacks += dropped_packs;
			DebugDropped(dropped_packs, cnt_droppedPacks, packTot);
		}

	}

	// CHECK IF NEW PACK
	if (c2r_packLast[CharInd(id, "c2r")] != pack)
	{
		// Update last packet
		c2r_packLast[CharInd(id, "c2r")] = pack;

		// Pack should be used
		do_use_pack = true;
	}
	// PROCESS RESENT PACKETS
	else
	{
		// Do not reuse pack
		do_use_pack = false;

		// Loop through pack history
		for (int i = 0; i < c2r_hLng - 1; i++)
		{
			if (pack == c2r_packHist[i])
			{

				// Make sure packet history corresponds to current id
				if (c2r_idHist[i] == id)
				{

					// Get list index
					id_ind = CharInd(c2r_idHist[i], c2r_id);

					// Check caught pack is what we have stored
					if (c2r_packRepeat[id_ind] != pack)
					{
						// reset count
						c2r_cntRepeat[id_ind] = 1;
						// update stored packet
						c2r_packRepeat[id_ind] = c2r_packHist[i];
					}
					// itterate count
					else
					{
						c2r_cntRepeat[id_ind]++;
						cnt_packResend++;
					}

					if (c2r_cntRepeat[id_ind] > 0)
					{
						DebugResent(id, c2r_packRepeat[id_ind], c2r_cntRepeat[id_ind]);
						//Serial1.end();
						//Serial1.begin(57600);
					}
				}
			}
		}
	}

	// Packet not found in hist so process
	return do_use_pack;
}

// STORE BYTE SERIAL DATA FOR CS
void Store4_CS(char id, byte d1, uint16_t pack)
{
	// Local vars
	int queue_ind = 0;

	// Update queue index
	queue_ind = sendQueueInd;
	sendQueueInd--;

	// Store reciever id in last col
	r2_queue[queue_ind][6] = 'c';

	// Store header
	u.f = 0.0f;
	u.c[0] = r2c_head;
	r2_queue[queue_ind][0] = u.b[0];
	// Store mesage id
	u.f = 0.0f;
	u.c[0] = id;
	r2_queue[queue_ind][1] = u.b[0];
	// Store mesage data 
	r2_queue[queue_ind][2] = d1;
	// Store packet number
	u.f = 0.0f;
	u.i16[0] = pack;
	r2_queue[queue_ind][3] = u.b[0];
	r2_queue[queue_ind][4] = u.b[1];
	// Store footer
	u.f = 0.0f;
	u.c[0] = r2c_foot;
	r2_queue[queue_ind][5] = u.b[0];


	// Set to send
	doSend = true;

	// Update last packet array
	r2c_packLast[CharInd(id, "r2c")] = pack;

	// Update packet history 
	for (int i = 0; i < r2c_hLng - 1; i++)
	{
		// shift old values back
		r2c_packHist[i] = r2c_packHist[i + 1];
		r2c_idHist[i] = r2c_idHist[i + 1];
	}
	r2c_packHist[r2c_hLng - 1] = pack;
	r2c_idHist[r2c_hLng - 1] = id;

	// Check for done id
	if (id == 'D')
	{
		// Set to check for done recieved
		fc_doCheckDoneRcvd = true;
		// Send done recieved request after 1 sec
		t_resendDone = millis() + 1000;
	}
}

// STORE BYTE SERIAL DATA FOR ARD
void Store4_Ard(char id, byte d1)
{
	// Local vars
	int queue_ind = 0;

	// Itterate packet
	r2a_packCnt++;

	// Shift data back so ard msg is first in queue
	for (int i = 0; i < r2_lngR - 1; i++)
	{
		for (int j = 0; j < r2_lngC; j++)
		{
			r2_queue[i][j] = r2_queue[i + 1][j];
		}
	}

	// Update queue index
	queue_ind = r2_lngR - 1;
	sendQueueInd--;

	// Store reciever id in last col
	r2_queue[queue_ind][6] = 'a';

	// Store header
	u.f = 0.0f;
	u.c[0] = r2a_head;
	r2_queue[queue_ind][0] = u.b[0];
	// Store mesage id
	u.f = 0.0f;
	u.c[0] = id;
	r2_queue[queue_ind][1] = u.b[0];
	// Store mesage data 
	r2_queue[queue_ind][2] = d1;
	// Store packet number
	u.f = 0.0f;
	u.i16[0] = r2a_packCnt;
	r2_queue[queue_ind][3] = u.b[0];
	r2_queue[queue_ind][4] = u.b[1];
	// Store footer
	u.f = 0.0f;
	u.c[0] = r2a_foot;
	r2_queue[queue_ind][5] = u.b[0];

	// Set to send
	doSend = true;

}

// SEND SERIAL PACKET DATA
void SendPacketData()
{
	// Local vars
	const int msg_size = r2_lngC - 1;
	static byte msg[msg_size];
	char rcv_id = ' ';
	bool do_send = false;
	int buff_tx = 0;

	// save reviever id
	rcv_id = r2_queue[r2_lngR - 1][6];

	// Move next in queue to temp msg array
	for (int j = 0; j < msg_size; j++)
	{
		msg[j] = r2_queue[r2_lngR - 1][j];
	}

	// Get total data in buffer
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial.availableForWrite();

	// Send r2a sync time or rew tone immediately
	if ((msg[1] == 'r' ||
		msg[1] == 't'))
	{
		do_send = true;
	}
	// Avoid overlap between sent or rcvd events
	else if (millis() > t_sent + 10 &&
		millis() > t_rsvd + 15)
	{
		do_send = true;
	}

	// Send if conditions met
	if (do_send && Serial.availableForWrite() >= 10)
	{
		// Send
		Serial1.write(msg, msg_size);

		// Update send time 
		t_sent = millis();

		// Update queue index
		sendQueueInd++;

		// Remove sent msg from front of queue
		for (int i = r2_lngR - 1; i >= 1; i--)
		{
			for (int j = 0; j < r2_lngC; j++)
			{
				r2_queue[i][j] = r2_queue[i - 1][j];
			}
		}
		// Set first entry to all zeros
		for (int j = 0; j < r2_lngC; j++)
		{
			r2_queue[0][j] = 0;
		}

		// Set to not send again if all sent
		if (sendQueueInd == r2_lngR - 1)
		{
			doSend = false;
		}

		// Print
		if (((doPrint_r2c && rcv_id == 'c') ||
			(doPrint_r2a && rcv_id == 'a')) &&
			(doDB_PrintConsole || doDB_PrintLCD))
		{
			char str[50];
			char id = ' ';
			byte dat = 0;
			uint16_t pack = 0;

			// Store mesage id
			u.f = 0.0f;
			u.b[0] = msg[1];
			id = u.c[0];
			// Store dat
			dat = msg[2];
			// Store packet number
			u.f = 0.0f;
			u.b[0] = msg[2];
			u.b[1] = msg[3];
			pack = u.i16[0];

			// Store
			sprintf(str, "sent: i:%c d:%d p:%d", id, dat, pack);
			StoreDBPrintStr(str, t_sent);
		}

	}
}

// SEND SERIAL LOG DATA
bool SendLogData(bool do_resend)
{
	// Local vars
	static int listInd = 0;
	String msg_str = " ";
	char msg_char[100];
	int msg_size = 0;
	byte msg[100];

	// Incriment list ind
	if (!do_resend) listInd++;

	// Check if end of list reached
	if (listInd > logCnt)
	{
		return false;
	}

	// Pull out string
	msg_str = logList[listInd - 1];

	// Get message size
	msg_size = msg_str.length();

	// Convert to char array
	msg_str.toCharArray(msg_char, msg_size);

	// Load byte array
	msg[0] = r2c_head;
	msg[1] = msg_size;
	for (int i = 0; i < msg_size; i++)
		msg[i + 2] = msg_char[i];
	msg[msg_size + 3] = r2c_head;

	// Send
	Serial1.write(msg, msg_size + 3);

	// TEST
	//SerialUSB.println(msg_str);

	return true;
}

#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------

// HARD STOP
void HardStop(String called_from)
{
	// Normal hard stop
	ad_R.hardStop();
	ad_F.hardStop();

	// Reset pid
	pid.Reset();

	// Set to high impedance for manual session
	if (fc_isManualSes)
	{
		delay(100);
		ad_R.hardHiZ();
		ad_F.hardHiZ();
	}

	// Store string
	DebugFlow("HARD STOP [" + called_from + "]");
}

// IR TRIGGERED HARD STOP
void Function_IRprox_Halt()
{
	if (!(bull.mode == "Active" && bull.state == "On") &&
		!fc_isManualSes)
	{
		HardStop("Function_IRprox_Halt");
	}
}

// RUN AUTODRIVER
bool RunMotor(char dir, float speed, String agent)
{
	// Local vars
	float speed_steps = speed*cm2stp;

	// Check that caller has control
	if (agent == fc_motorControl)
	{
		if (dir == 'f')
		{
			ad_R.run(FWD, speed_steps);
			ad_F.run(FWD, speed_steps*frontMoterScale);
		}
		else if (dir == 'r')
		{
			ad_R.run(REV, speed_steps);
			ad_F.run(REV, speed_steps*frontMoterScale);
		}
		return true;
	}
	else return false;
}

// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(String set_to, String called_from)
{
	// VALUES:  ["None", "Open", "MoveTo", "Bull", "PID"]

	// Can always set to "None"
	if (set_to == "None")
	{
		fc_motorControl = "None";
	}

	// Do not change control if halted
	if (!fc_isHalted)
	{
		// Cannot unset "None" unless certain conditions met
		if (fc_motorControl == "None")
		{
			if (!fc_isBlockingTill)
			{
				// Can set to "MoveTo" 
				if (set_to == "MoveTo")
				{
					fc_motorControl = set_to;
				}
				// Can set to open if tracking setup
				else if (set_to == "Open" &&
					fc_isTrackingEnabled)
				{
					fc_motorControl = set_to;
				}
			}
			// Can unblock but only if rat in
			else if (
				fc_isTrackingEnabled &&
				called_from == "CheckBlockTimElapsed"
				)
			{
				fc_motorControl = set_to;
			}
		}
		// "MoveTo" can only be set to "Open"
		else if (fc_motorControl == "MoveTo")
		{
			if (set_to == "Open")
			{
				fc_motorControl = set_to;
			}
		}
		// "Bull" can only be set to "MoveTo" or "Open"
		else if (fc_motorControl == "Bull")
		{
			if (set_to == "MoveTo" || set_to == "Open")
			{
				fc_motorControl = set_to;
			}
		}
		// Otherwise can set to anything
		else if (fc_motorControl == "Open" || fc_motorControl == "PID")
		{
			fc_motorControl = set_to;
		}
	}

	// Store current controller
	DebugMotorControl(set_to, called_from);

	// Return true if set to input
	if (set_to == fc_motorControl)
	{
		return true;
	}
	else return false;
}

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(uint32_t dt)
{
	// Set blocking and time
	fc_isBlockingTill = true;

	// Update time to hold till
	t_blockTill = millis() + dt;

	// Remove all motor controll
	SetMotorControl("None", "BlockMotorTill");

}

// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed()
{
	if (fc_isBlockingTill)
	{
		// Check that all 3 measures say rat has passed
		bool passed_feeder =
			fc_isTrackingEnabled &&
			ekfRatPos - (ekfRobPos + feedDist) > 0 &&
			pos_ratVT.posNow - (ekfRobPos + feedDist) > 0 &&
			pos_ratPixy.posNow - (ekfRobPos + feedDist) > 0;

		// Check for time elapsed or rat moved at least 3cm past feeder
		if (millis() > t_blockTill || passed_feeder)
		{
			// Retract feeder arm
			reward.RetractFeedArm();

			// Set flag to stop checking
			fc_isBlockingTill = false;

			// Open up control
			SetMotorControl("Open", "CheckBlockTimElapsed");
		}
	}
}

// DO SETUP TO BEGIN TRACKING
void InitializeTracking()
{
	// Reset pos data once after rat in
	if (fc_isRatIn &&
		!fc_isTrackingEnabled &&
		pos_ratVT.newData &&
		pos_ratPixy.newData &&
		pos_robVT.newData)
	{
		// Local vars
		int n_laps = 0;
		float cm_diff = 0;
		float cm_dist = 0;

		// Check that rat pos < robot pos
		n_laps = pos_ratVT.posNow > pos_robVT.posNow ? 0 : 1;
		if (n_laps > 0)
		{
			DebugFlow("SET RAT POS AHEAD");
		}
		else
		{
			DebugFlow("RAT POS ALREADY AHEAD");
		}

		// Reset all vt data
		pos_ratVT.ResetDat(pos_ratVT.posNow, n_laps);
		pos_ratPixy.ResetDat(pos_ratPixy.posNow, n_laps);

		// Check that results make sense
		cm_diff = pos_ratVT.posNow - pos_robVT.posNow;
		cm_dist = min((140 * PI) - abs(cm_diff), abs(cm_diff));

		// Rat should be no more than 90 deg rom rob
		if (cm_dist > ((140 * PI) / 4))
		{
			// Will have to run again with new samples
			pos_ratVT.ResetDat(0, 0);
			pos_ratPixy.ResetDat(0, 0);
			pos_robVT.ResetDat(0, 0);
		}
		// Good to go
		else
		{

			// Set flag
			fc_isTrackingEnabled = true;

			// Reset ekf
			pid.ResetEKF("InitializeTracking");

			// Don't start pid for manual sessions
			if (!fc_isManualSes)
			{
				// Open up motor control
				SetMotorControl("Open", "InitializeTracking");

				// Run PID
				pid.Run("InitializeTracking");
				DebugFlow("PID STARTED");
			}

			// Initialize bulldoze
			if (fc_doBulldoze)
			{
				// Run from initial blocked mode
				bull.TurnOn("InitializeTracking");
				DebugFlow("BULLDOZE INITIALIZED");
			}

			// Blink lcd display
			RatInBlink();

		}
	}
}

// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT() {

	if (fc_isRatIn)
	{

		// Local vars
		uint32_t max_frame_dt = 100;
		uint32_t vt_dt = 0;
		uint32_t pixy_dt = 0;

		// Compute dt
		vt_dt = millis() - pos_ratVT.t_msNow;
		pixy_dt = millis() - pos_ratPixy.t_msNow;

		// Check pixy
		if (
			pixy_dt >= max_frame_dt &&
			vt_dt < max_frame_dt
			)
		{
			// Use VT for Pixy data
			pos_ratPixy.SetDat(pos_ratVT.posAbs, pos_ratVT.t_msNow);
		}
		// Check VT 
		else if (
			vt_dt >= max_frame_dt &&
			pixy_dt < max_frame_dt
			)
		{
			// Use Pixy for VT data
			pos_ratVT.SetDat(pos_ratPixy.posAbs, pos_ratPixy.t_msNow);
		}

	}

}

// PROCESS PIXY STREAM
void UpdatePixyPos() {

	// Local vars
	float px_rel = 0;
	double px_abs = 0;
	uint32_t px_ts = 0;
	double pixy_pos_y = 0;

	// Get new blocks
	uint16_t blocks = pixy.getBlocks();

	// Check for new data
	if (blocks)
	{

		// Save time stamp
		px_ts = millis();

		// Get Y pos from last block and convert to CM
		pixy_pos_y = pixy.blocks[blocks - 1].y;
		px_rel =
			pixyCoeff[0] * (pixy_pos_y * pixy_pos_y * pixy_pos_y * pixy_pos_y) +
			pixyCoeff[1] * (pixy_pos_y * pixy_pos_y * pixy_pos_y) +
			pixyCoeff[2] * (pixy_pos_y * pixy_pos_y) +
			pixyCoeff[3] * pixy_pos_y +
			pixyCoeff[4];

		// Scale to abs space with rob vt data
		px_abs = px_rel + pos_robVT.posAbs;
		if (px_abs > (140 * PI))
		{
			px_abs = px_abs - (140 * PI);
		}
		// Update pixy pos and vel
		pos_ratPixy.UpdatePosVel((float)px_abs, px_ts);

	}

}

// UPDATE EKF
void UpdateEKF()
{
	// Check for new data w or w/o rat tracking
	if ((pos_ratVT.newData && pos_ratPixy.newData && pos_robVT.newData) ||
		(pos_robVT.newData && !fc_isRatIn))
	{

		// Check EKF progress
		pid.CheckEKF(millis());

		// Update pid next loop time
		pid.SetLoopTime(millis());

		// Update PID loop time
		if (fc_isRatIn)
		{
			// Set flag for reward 
			reward.is_ekfNew = true;
			// Store pixy/vt average
			vtpixyVelAvg =
				(pos_ratVT.velNow + pos_ratPixy.velNow) / 2;
			vtpixyPosAvg =
				(pos_ratVT.posNow + pos_ratPixy.posNow) / 2;
		}
		// Set rat pos data to match robot
		else
		{
			pos_ratVT.ResetDat(0, 0);
			pos_ratPixy.ResetDat(0, 0);
		}

		//----------UPDATE EKF---------
		double z[M] = {
			pos_ratVT.GetPos(),
			pos_ratPixy.GetPos(),
			pos_robVT.GetPos(),
			pos_ratVT.GetVel(),
			pos_ratPixy.GetVel(),
			pos_robVT.GetVel(),
		};

		// Run EKF
		ekf.step(z);

		// Update error estimate
		ekfRatPos = ekf.getX(0);
		ekfRobPos = ekf.getX(1);
		ekfRatVel = ekf.getX(2);
		ekfRobVel = ekf.getX(3);

	}
}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid()
{
	// Local vars
	byte sol_state = digitalRead(pin_Rel_Rew);

	// Change state
	sol_state = !sol_state;

	// Open/close solenoid
	digitalWrite(pin_Rel_Rew, sol_state);

	// Print to LCD
	char str[50];
	sprintf(str, "%s", digitalRead(pin_Rel_Rew) == HIGH ? "OPEN" : "CLOSED");
	PrintLCD("REW SOLENOID", str, 's');
}

// CHECK FOR ETOH UPDATE
void CheckEtOH()
{
	// Local vars
	static uint32_t t_etoh_start = millis(); // (ms)
	static float etoh_dist_start = 0; // (cm)
	static float etoh_dist_diff = 0; // (cm)

	// Get distance traveled
	etoh_dist_diff = ekfRobPos - etoh_dist_start;

	// Check if EtOH should be run
	if (
		!isEtOHOpen &&
		(millis() > (t_etoh_start + t_delEtOH) || etoh_dist_diff > distMaxEtOH)
		)
	{

		// Open solenoid
		digitalWrite(pin_Rel_EtOH, HIGH);

		// Reset vars
		t_etoh_start = millis();
		etoh_dist_start = ekfRobPos;

		// Set flag
		isEtOHOpen = true;

		// Print to debug
		DebugFlow("EtOH SOLENOID OPEN");
	}
	else if (
		isEtOHOpen &&
		millis() > (t_etoh_start + t_durEtOH)
		)
	{
		// Close solenoid
		digitalWrite(pin_Rel_EtOH, LOW);

		// Set flag
		isEtOHOpen = false;

		// Print to debug
		DebugFlow("EtOH SOLENOID CLOSE");
	}
}

// CHECK BATTERY VOLTAGE
void CheckBattery()
{
	// Local vars
	static bool do_volt_update = false;
	static float volt_avg = 0;
	float bit_in = 0;
	float volt_in = 0;
	float volt_sum = 0;
	byte byte_out = 0;


	// Only run if relay open
	if (isEtOHOpen)
	{
		bit_in = analogRead(pin_BatVolt);
		volt_in = bit_in * bit2volt;
		volt_sum = 0;
		// Shift array and compute average
		for (int i = 99; i > 0; i--) {
			batVoltArr[i] = batVoltArr[i - 1];
			volt_sum += batVoltArr[i];
		}
		batVoltArr[0] = volt_in;
		volt_avg = volt_sum / 99;

		// Set flag to send update
		do_volt_update = true;
	}
	else if (do_volt_update)
	{
		// Convert float to byte
		byte_out = byte(round(volt_avg * 10));

		// Add to queue
		Store4_CS('J', byte_out, 0);

		char str[50];
		sprintf(str, "V: (float)%0.2fV (byte)%d", volt_avg, byte_out);
		DebugFlow(str);

		// Reset flag
		do_volt_update = false;
	}
}

// TURN LCD LIGHT ON/OFF
void ChangeLCDlight()
{
	if (!lcdLightOn) {
		analogWrite(pin_Disp_LED, 50);
		lcdLightOn = true;
	}
	else {
		analogWrite(pin_Disp_LED, 0);
		lcdLightOn = false;
	}
}

// CHECK FOR BUTTON INPUT
void CheckButtons()
{
	// Local vars
	static uint32_t t_debounce[3] = { millis(), millis(), millis() };

	// RUN BUTTON 1 OPPERATIONS (Trigger reward)
	if (digitalRead(pin_Btn[0]) == LOW)
	{
		// exit if < reward time has not passed
		if (t_debounce[0] > millis()) return;

		// Set to start reward function
		btn_doRew = true;

		t_debounce[0] = millis() + reward.duration + 100;
	}
	// RUN BUTTON 2 OPPERATIONS (Open/close solonoid)
	else if (digitalRead(pin_Btn[1]) == LOW)
	{
		// exit if < 250 ms has not passed
		if (t_debounce[1] > millis()) return;

		// Run open close function
		btn_doChangeSolState = true;

		t_debounce[1] = millis() + 250;
	}
	// RUN BUTTON 3 OPPERATIONS (Turn on/off LCD LED)
	else if (digitalRead(pin_Btn[2]) == LOW)
	{
		// exit if < 250 ms has not passed
		if (t_debounce[2] > millis()) return;

		btn_doChangeLCDstate = true;

		t_debounce[2] = millis() + 250;
	}
	// Exit
	else return;
}

// QUIT AND RESTART ARDUINO
void QuitSession()
{
	// Stop all movement
	HardStop("QuitSession");
	pid.Stop("QuitSession");
	bull.TurnOff("QuitSession");
	delayMicroseconds(100);
	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

#pragma endregion


#pragma region --------DEBUGGING---------

void DebugMotorControl(String set_to, String called_from)
{
	if (
		(doPrint_motorControl && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_motorControl && doDB_Log)
		)
	{
		String str;
		str = "mc set:" + fc_motorControl + " " + "in:" + set_to + " [" + called_from + "]";

		// Add to print queue
		if (doPrint_motorControl && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_motorControl && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

void DebugDropped(int missed, int missed_total, int total)
{
	if (
		(doPrint_dropped && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_dropped && doDB_Log)
		)
	{
		char str[50];
		sprintf(str, "!dropped p:%d [t:%d/%d]!", missed, missed_total, total);

		// Add to print queue
		if (doPrint_dropped && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_dropped && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

void DebugResent(char id, uint16_t pack, int total)
{
	if (
		(doPrint_resent && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_resent && doDB_Log)
		)
	{
		char str[50];
		sprintf(str, "!resent i:%c p:%d t:%d!", id, pack, total);

		// Add to print queue
		if (doPrint_resent && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_resent && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

void DebugRsvd(char id, uint16_t pack)
{
	if (
		(doPrint_rcvd && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_rcvd && doDB_Log)
		)
	{
		char str[50];

		// Print specific pack contents
		if (id == 'T')
		{
			sprintf(str, "rcvd: [i:%c d1:%d d2:%d p:%d", id, msg_setupCmd[0], msg_setupCmd[1], pack);
		}
		else if (id == 'S')
		{
			sprintf(str, "rcvd: [i:%c d1:%d d2:%d p:%d", id, msg_setupCmd[0], msg_setupCmd[1], pack);
		}
		else if (id == 'M')
		{
			sprintf(str, "rcvd: [i:%c d1:%0.2f p:%d", id, msg_moveToTarg, pack);
		}
		else if (id == 'R')
		{
			sprintf(str, "rcvd: [i:%c d1:%0.2f p:%d", id, msg_rewPos, pack);
		}
		else if (id == 'H')
		{
			sprintf(str, "rcvd: [i:%c d1:%d p:%d", id, fc_doHalt, pack);
		}
		else if (id == 'B')
		{
			sprintf(str, "rcvd: [i:%c d1:%d p:%d", id, msg_bullDel, pack);
		}
		else if (id == 'I')
		{
			sprintf(str, "rcvd: [i:%c d1:%d p:%d", id, fc_isRatIn, pack);
		}
		else sprintf(str, "rcvd: [i:%c p:%d", id, pack);

		// Add to print queue
		if (doPrint_rcvd && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_rcvd && doDB_Log)
			StoreDBLogStr(str, millis());
	}

}

void DebugIRSync(String str)
{
	if (
		(doPrint_irSync && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_irSync && doDB_Log)
		)
	{
		// Add to print queue
		if (doPrint_irSync && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, t_irSyncLast);
		// Add to log queue
		if (doLog_irSync && doDB_Log)
			StoreDBLogStr(str, t_irSyncLast);
	}
}

void DebugFlow(String str)
{
	if (
		(doPrint_flow && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_flow && doDB_Log)
		)
	{
		// Add to print queue
		if (doPrint_flow && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_flow && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

void PrintDebug()
{

	if ((doDB_PrintLCD && !doBlockLCDlog) ||
		doDB_PrintConsole)
	{

		// Print to LCD
		if (doDB_PrintLCD && !doBlockLCDlog)
		{
			// Local vars 
			static int scale_ind = 40 / printQueue_lng;
			int ind = 0;

			// Change settings
			myGLCD.setFont(TinyFont);
			myGLCD.invert(false);

			// Clear
			myGLCD.clrScr();
			myGLCD.update();

			// Print time
			myGLCD.print(printQueue[0], LEFT, ind);

			// Print everything
			for (int i = printQueue_lng - 1; i >= printQueue_lng - 9; i--)
			{
				// Break for empty strings
				if (printQueue[i] == " ")
					break;

				// Get pos ind
				ind += 6;

				// Print
				myGLCD.print(printQueue[i], LEFT, ind);

				// Update
				myGLCD.update();
			}
		}

		// Print to console
		if (doDB_PrintConsole)
		{
			// Get current string
			String str =
				"\n" + printQueue[printQueueInd] + " (" + printQueue[0] + ")\n";

			// Send
			SerialUSB.print(str);
		}

		// Update queue index
		printQueueInd++;

		// Set to not print again if all printed
		if (printQueueInd == printQueue_lng)
		{
			doPrint = false;
		}
	}
}

void StoreDBPrintStr(String str, uint32_t ts)
{
	// Local vars
	char t_str_long[50];
	char t_str_ellapsed[50];
	static uint32_t t_last = millis();
	uint32_t ts_norm = 0;
	float t_c = 0;
	float t_m = 0;
	float t_ellapsed = 0;

	// Time now
	ts_norm = t_sync == 0 ? ts : t_sync;

	// Total time
	t_c = (float)((millis() - ts_norm) / 1000.0f);
	t_m = (float)millis() / 1000.0f;

	// Ellapsed time
	t_ellapsed = (float)((millis() - t_last) / 1000.0f);
	t_last = millis();

	// Save long and short string
	sprintf(t_str_long, "%0.2fsec/%0.2fsec", t_c, t_m);
	sprintf(t_str_ellapsed, "%0.0f-", t_ellapsed);

	// Shift queue
	for (int i = 0; i < printQueue_lng - 1; i++)
	{
		printQueue[i] = printQueue[i + 1];
	}

	// Set first entry to new string
	printQueue[printQueue_lng - 1] = t_str_ellapsed + str;

	// Set last entry to time 
	printQueue[0] = t_str_long;

	// Set queue ind
	printQueueInd--;

	// Set flag
	doPrint = true;

}

void StoreDBLogStr(String str, uint32_t ts)
{
	// Local vars
	char str_c[50];
	char msg_c[50];
	uint32_t ts_norm = 0;

	// Itterate log entry count
	logCnt++;

	// Time now
	ts_norm = t_sync == 0 ? ts : ts - t_sync;

	// Concatinate ts with message
	str.toCharArray(str_c, str.length());
	sprintf(msg_c, "%d %s", ts_norm, str_c);
	logList[logCnt - 1] = msg_c;

	// TEST
	delay(100);
	PrintLCD(str, " ", 't');
	//SerialUSB.println(logList[logCnt - 1]);
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

void PrintLCD(String str_1)
{
	// Run default
	PrintLCD(str_1, " ", 's');
}
void PrintLCD(String str_1, String str_2, char f_siz)
{

	// Change settings
	if (f_siz == 's')
		myGLCD.setFont(SmallFont);
	else if (f_siz == 't')
		myGLCD.setFont(TinyFont);
	myGLCD.invert(true);

	// Clear
	myGLCD.clrScr();

	// Print
	if (str_2 != " ")
	{
		myGLCD.print(str_1, CENTER, 15);
		myGLCD.print(str_2, CENTER, 25);
	}
	else myGLCD.print(str_1, CENTER, 20);

	// Update
	myGLCD.update();

	// Block LCD logging/printing while displayed
	doBlockLCDlog = true;
}

void ClearLCD()
{
	// Clear
	myGLCD.clrScr();

	// Update
	myGLCD.update();

	// Stop blocking LCD log
	doBlockLCDlog = false;
}

void SetupBlink()
{
	int duty[2] = { 100, 0 };
	bool is_on = false;
	int del = 100;
	// Flash sequentially
	for (int i = 0; i < 8; i++)
	{
		analogWrite(pin_Disp_LED, duty[(int)is_on]);
		delay(del);
		analogWrite(pin_TrackLED, duty[(int)is_on]);
		delay(del);
		analogWrite(pin_RewLED_R, duty[(int)is_on]);
		delay(del);
		is_on = !is_on;
	}
	// Reset LEDs
	analogWrite(pin_Disp_LED, 0);
	analogWrite(pin_TrackLED, trackLEDduty);
	analogWrite(pin_RewLED_R, rewLEDmin);
}

void RatInBlink()
{
	// Local vars
	int duty[2] = { 255, 0 };
	bool is_on = true;
	int del = 50;

	// Flash 
	for (int i = 0; i < 3; i++)
	{
		analogWrite(pin_TrackLED, duty[(int)is_on]);
		delay(del);
		is_on = !is_on;
	}
	// Reset LED
	analogWrite(pin_TrackLED, trackLEDduty);
}

int CharInd(char id, String a_lab)
{

	int ind = -1;
	if (a_lab == "c2r")
	{
		for (int i = 0; i < c2r_idLng; i++)
		{
			if (id == c2r_id[i]) ind = i;
		}
	}
	else if (a_lab == "r2c")
	{
		for (int i = 0; i < r2c_idLng; i++)
		{
			if (id == r2c_id[i]) ind = i;
		}
	}
	else if (a_lab == "r2a")
	{
		for (int i = 0; i < r2a_idLng; i++)
		{
			if (id == r2a_id[i]) ind = i;
		}
	}
	return ind;

}

#pragma endregion


#pragma region ---------INTERUPTS---------

// Halt run on IR trigger
void Interupt_IRprox_Halt() {

	// Exit if < 250 ms has not passed
	if (intrpt_irProxDebounce > millis()) return;

	// Run stop in main loop
	intrpt_doIRhardStop = true;

	// Update debounce
	intrpt_irProxDebounce = millis() + 250;
}

// IR detector
void Interupt_IR_Detect()
{
	// Exit if < 250 ms has not passed
	if (intrpt_irDetectDebounce > millis()) return;

	// Store time
	t_irSyncLast = millis();

	// Check if this if first event
	if (t_sync == 0)
	{
		t_sync = t_irSyncLast;
	}

	// Set flag
	doLogIR = true;

	// Update debounce
	intrpt_irDetectDebounce = millis() + 250;
}
#pragma endregion

