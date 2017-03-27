
//-------FEEDERDUE-------

// XBee DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size) 
// XBee DO (to UART rx) buffer = 202 bytes
// DUE SERIAL_BUFFER_SIZE = 128
// SerialUSB receive buffer size is now 512 (ARDUINO 1.5.2 BETA - 2013.02.06)

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
const int pin_Disp_Pwr = 1;
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
const int pin_Rel_Ens = 26;
const int pin_Rel_Eth = 27;

// XBee power
const int pin_XBee_Pwr = 52;

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
const int pin_FeedSwitch = 0;

// Power off
const int pin_PwrOff = 31;

// Voltage monitor
const int pin_BatVolt = A11;

/*
Note: pins bellow are all used for external interupts
and must all be members of the same port (PortA)
*/

// IR proximity sensors
const int pin_IRprox_Pwr = 22;
const int pin_IRprox_Rt = 42;
const int pin_IRprox_Lft = 43;

// IR detector
const int pin_IRdetect = A4;
const int pin_IRdetect_Gnd = A5;
const int pin_IRdetect_Pwr = A6;

// Buttons
const int pin_Btn[3] = { A3, A2, A1 };

#pragma endregion


#pragma region ---------VARIABLE SETUP---------

// Debug print
const bool doDebugConsole = false;
const bool doDebugLCD = false;
const bool doPrint_flow = false;
const bool doPrint_motorControl = false;
const bool doPrint_rcvd = false;
const bool doPrint_r2c = false;
const bool doPrint_r2a = false;
const bool doPrint_pid = false;
const bool doPrint_bull = false;
const bool doPrint_resent = false;
const bool doPrint_dropped = false;
String printQueue[10];
const int printQueue_lng =
sizeof(printQueue) / sizeof(printQueue[0]);
int printQueueInd = printQueue_lng;
bool doPrint = false;
// debug tracking
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

// Start/Quit
byte msg_setupCmd[2];
uint32_t t_quitCmd;

// Serial from CS
const char c2r_head[2] = { '_', '<' };
const char c2r_foot = '>';
const char c2r_id[11] = {
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
	'Y', // confirm done recieved
};
char msg_id = ' ';
bool msg_pass = false;
uint32_t t_rsvd = millis(); // (ms)
uint32_t t_rsvdLast; // (ms)

					 // Serial to CS
const char r2c_head = '{';
const char r2c_foot = '}';
const char r2c_id[10] = {
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
};
uint32_t t_resendDone = millis();
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
uint32_t t_sent = millis();

// Outgoing data
byte r2_queue[10][7];
const int r2_lngR = 10;
const int r2_lngC = 7;
int sendQueueInd = r2_lngR - 1;
bool doSend = false;
uint32_t t_sync = 0;

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
	0.000000039357552,
	-0.000020900586036,
	0.004379255805114,
	-0.572754842881408,
	69.251766070592168
};
float vtpixyVelAvg;
float vtpixyPosAvg;

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 80; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)
const byte kAcc = 60*2;
const byte kDec = 60*2;
const byte kRun = 60;
const byte kHold = 60 / 2;

// Kalman model measures
float ekf_ratPos = 0;
float ekf_robPos = 0;
float ekf_ratVel = 0;
float ekf_robVel = 0;

// PID Settings
bool do_includeTerm[2] = { true, true };
const uint32_t loopTim = 60; // pid sample rate (ms)
const float kC = 5; // critical gain kC = 5 * (1 / 0.6)
const float pC = 3; // oscillation period   
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

// LEDs
const int trackLEDduty = 75; // value between 0 and 255
const int rewLEDduty = 15; // value between 0 and 255
const int rewLEDmin = 0; // value between 0 and 255

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
bool lcdLightOn = false;

// Interrupt and button vars
volatile bool vol_doChangeSolState = false;
volatile bool vol_doRew = false;
volatile bool vol_doChangeLCDstate = false;
volatile bool vol_doIRhardStop = false;
volatile bool vol_doBlockLCDlog = false;
volatile uint32_t t_debounce[4] = { millis(), millis(), millis(), millis() };

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
	char objID;
	int nSamp;
	float posArr[6];
	uint32_t tsArr[6];
	int t_skip = 0;
	float velNow = 0.0f;
	float posNow = 0.0f;
	float posAbs = 0.0f;
	uint32_t tsNow = 0;
	uint32_t msNow = millis();
	float nLaps = 0;
	int sampCnt = 0;
	bool newData = false;
	uint32_t dtFrame;

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
		this->msNow = millis();
		this->posAbs = pos_new;
		this->tsNow = ts_new;

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
		uint32_t ts = tsNow + (ms - msNow);

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
	uint32_t t_lastMotorRun = 0;
	float dtLoop;
	bool wasLoopRan = false;
	float p_term;
	float i_term;
	float d_term;
	bool firstRun = true;
	String mode = "Manual"; // ["Manual" "Automatic" "Halted"]
	bool isHolding4cross = false;
	bool doDamp = false;
	bool isDampened = false;
	float error = 0;
	float errorLast = 0;
	float integral = 0;
	float derivative = 0;
	float velUpdate;
	float runSpeed = 0;
	float speedMax = maxSpeed;
	float kP_default;
	float kI_default;
	float kD_default;
	float dT;
	float kP;
	float kI;
	float kD;
	float setPoint;
	uint32_t t_ekfStr = 0;
	uint32_t t_ekfSettle = 500; // (ms)
	bool ekfReady = false;
	bool ekfNew = false;
	float dampAcc = 40;
	float dampSpeedCut = 20;
	uint32_t dampTimeCut = 4000;
	uint32_t t_dampTill;
	uint32_t t_lastDamp = millis();

	// constructor
	PID(const float kC, const float pC, const float set_point)
	{
		this->kP_default = 0.6 * kC; // proportional constant
		this->kI_default = 2 * kP_default / pC; // integral constant
		this->kD_default = kP_default*pC / 8; // derivative constant
		this->kP = kP_default; // proportional constant
		this->kI = kI_default; // integral constant
		this->kD = kD_default; // derivative constant
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
			error = ekf_ratPos - (ekf_robPos + setPoint);

			// Compute other error measures
			errorFeeder = ekf_ratPos - (ekf_robPos + feedDist);
			errorDefault = ekf_ratPos - (ekf_robPos + defualtSetPoint);

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
				if (ekf_ratVel < 1 && errorDefault < -15 && !isHolding4cross)
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
					error = ekf_ratPos - (ekf_robPos + setPoint);

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
					runSpeed = ekf_robVel + velUpdate;

					// Keep speed in range [0, speedMax]
					if (runSpeed > speedMax) runSpeed = speedMax;
					else if (runSpeed < 0) runSpeed = 0;

					errorLast = error;
					wasLoopRan = true;
					ekfNew = false;

					t_lastMotorRun = millis();
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
		if (doPrint_pid && (doDebugConsole || doDebugLCD))
		{
			StoreDebugStr(str);
		}
	}

	void RunCalibration()
	{
		float PcCount = 0;  // oscillation count
		uint32_t t_Pc[2] = { 0, 0 }; // oscillation period sum
		uint32_t PcSum = 0; // oscillation period sum
		float PcAvg;
		float PcNow;
		float errCount = 0;
		float errSum = 0;
		float errAvg;
		float errMax = 0;
		float errMin = 0;
		if (errorLast > 0 && error < 0)
		{
			t_Pc[0] = t_Pc[1];
			t_Pc[1] = millis() / 1000;
			if (t_Pc[0] > 0 && t_Pc[1] > 0)
			{
				PcCount++;
				PcNow = t_Pc[1] - t_Pc[0];
				PcSum += PcNow;
				PcAvg = (float)(PcSum) / PcCount;
			}
		}
		errCount++;
		errSum += abs(error);
		errAvg = errSum / errCount;
		errMax = max(errMax, error);
		errMin = max(errMin, error);

		// If robot runs over rat
		if (error < setPoint*-1);
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
	int bSpeed;
	int bDelay; // (ms)
	float posCheck = 0;
	float posNow = 0;
	float distMoved;
	float guardPos;
	bool hasMoved;
	bool timesUp;
	bool passedReset;

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
				posNow = ekf_ratPos;
				guardPos = ekf_robPos + guardDist;

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
		posCheck = ekf_ratPos;
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
		posCheck = ekf_ratPos;
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
		if (doPrint_bull && (doDebugConsole || doDebugLCD))
		{
			StoreDebugStr(str);
		}
	}

};
// Initialize object
Bulldozer bull;

//----------CLASS: Target----------
class Target
{
public:
	String obj_id;
	float pos_rel;
	float move_diff;
	float min_speed;
	const uint32_t t_updateDT = 10;
	uint32_t t_updateNext;
	float dist_left;
	float new_speed;
	float pos_start;
	float targ;
	float offset_target;
	float targ_dist;
	char move_dir;
	float base_speed = 0;
	bool is_targ_set = false;
	bool is_targ_reached = false;
	float halt_error;
	const double velCoeff[3] = {
		0.001830357142857,
		0.131160714285714,
		-2.425892857142854,
	};

	// constructor
	Target(String id)
	{
		this->obj_id = id;
	}

	bool CompTarg(float now_pos, float targ_pos, float offset)
	{
		// Copy to public vars
		pos_start = now_pos;
		targ = targ_pos;

		// Run only if targ not set
		if (!is_targ_set)
		{
			// Check pos data is ready
			if (pid.ekfReady)
			{
				// Compute target targ_dist and move_dir
				offset_target = targ + offset;
				if (offset_target < 0)
					offset_target = offset_target + (140 * PI);
				else if (offset_target > (140 * PI))
					offset_target = offset_target - (140 * PI);

				// Current relative pos on track
				int diam = (int)(140 * PI * 100);
				int pos = (int)(now_pos * 100);
				pos_rel = (float)(pos % diam) / 100;

				// Diff and absolute targ_dist
				move_diff = offset_target - pos_rel;
				targ_dist =
					min((140 * PI) - abs(move_diff), abs(move_diff));

				// Set to negative for reverse move
				if ((move_diff > 0 && abs(move_diff) == targ_dist) ||
					(move_diff < 0 && abs(move_diff) != targ_dist))
				{
					move_dir = 'f';
				}
				else
				{
					move_dir = 'r';
					targ_dist = targ_dist*-1;
				}

				// Set vars for later
				t_updateNext = millis();
				base_speed = 0;

				// Set flag true
				is_targ_set = true;
			}

		}

		// Retern move targ_dist
		return is_targ_set;
	}

	float DecelToTarg(float now_pos, float now_vel, float dec_pos, float speed_min)
	{
		// Run if targ not reached
		if (!is_targ_reached)
		{

			// Compute remaining targ_dist
			dist_left = abs(targ_dist) -
				min((140 * PI) - abs(now_pos - pos_start), abs(now_pos - pos_start));

			// Check if rob is dec_pos cm from target
			if (dist_left <= dec_pos)
			{
				// Get base speed to decelerate from
				if (base_speed == 0)
				{
					base_speed = abs(now_vel);
				}
				// Update decel speed
				else if (millis() > t_updateNext)
				{
					// Compute new speed
					new_speed = (dist_left / dec_pos) * base_speed;

					// Maintain at min speed
					if (new_speed < speed_min) new_speed = speed_min;

					// Update loop time
					t_updateNext = millis() + t_updateDT;
				}

			}

			// Target reached
			if (dist_left < 1)
			{
				// Set flag true
				is_targ_reached = true;

				new_speed = 0;
			}
		}
		else new_speed = 0;

		return new_speed;
	}

	bool CheckTargReached(float now_pos, float now_vel)
	{
		// Run if targ not reached
		if (!is_targ_reached)
		{
			// Get velocity corrected targ_dist threshold
			if (now_vel >= 20)
			{
				halt_error =
					velCoeff[0] * (now_vel * now_vel) +
					velCoeff[1] * now_vel +
					velCoeff[2];
			}
			else halt_error = 1;

			// Cap halt error at min of 1
			halt_error = halt_error < 1 ? 1 : halt_error;

			// Compute remaining targ_dist
			dist_left = abs(targ_dist) -
				min((140 * PI) - abs(now_pos - pos_start), abs(now_pos - pos_start));

			// Target is reached
			if (dist_left < halt_error)
			{
				// Set flag true
				is_targ_reached = true;
			}
		}

		return is_targ_reached;
	}

	float GetError(float now_pos)
	{
		// Current relative pos on track
		int diam = (int)(140 * PI * 100);
		int pos = (int)(now_pos * 100);
		pos_rel = (float)(pos % diam) / 100;

		// Target error
		return offset_target - pos_rel;
	}

	void Reset()
	{
		is_targ_set = false;
		is_targ_reached = false;
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
	uint32_t duration_byte; // (ms) 
	float targBounds[9][2];
	const int targ_lng =
		sizeof(targLocs) / sizeof(targLocs[0]);
	float boundMin = 0;
	float boundMax = 0;
	uint32_t t_end;
	float rewCenter;
	bool is_bound_set = false;
	bool is_trigger_ready = false;
	bool is_all_targ_passed = false;
	bool is_ekf_new = false;
	float rewardedTarg;
	float rewardedBounds[2];
	float lap_n;

	// Constructor
	Reward()
	{
		this->duration = 2000;
		this->duration_byte = (byte)(duration / 10);
	}

	// Set reward duration
	void SetRewDur(uint32_t dur)
	{
		duration = dur;
		duration_byte = (byte)(duration / 10);
	}
	void SetRewDur(byte dur_byte)
	{
		SetRewDur((uint32_t)dur_byte * 10);
	}

	// Compute target bounds
	bool CompTargBounds(float now_pos, float rew_pos)
	{
		// Run only if bounds are not set
		if (!is_bound_set)
		{
			// Compute laps
			int diam = (int)(140 * PI * 100);
			int pos = (int)(now_pos * 100);
			lap_n = round(now_pos / (140 * PI) - (float)(pos % diam) / diam);
			// Check if rat 'ahead' of rew pos
			float pos_rel = (float)(pos % diam) / 100;
			// add lap
			lap_n = pos_rel > rew_pos ? lap_n + 1 : lap_n;

			// Compute reward center
			rewCenter = rew_pos + lap_n*(140 * PI);

			// Compute bounds for each targ
			float dist_center_cm;
			float dist_start_cm;
			float dist_end_cm;
			for (int i = 0; i < targ_lng; i++)
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
				boundMax = i == targ_lng - 1 ? targBounds[i][1] : boundMax;
			}
			// Set flag
			is_bound_set = true;
		}
		return is_bound_set;
	}

	// Check bounds
	bool CheckTargBounds(float now_pos, float now_vel)
	{
		// Run only if reward not already triggered
		if (!is_trigger_ready)
		{

			// Check if all bounds passed
			if (now_pos > boundMax + 5)
			{
				is_all_targ_passed = true;
				return is_trigger_ready;
			}
			// Check velocity
			else if (
				now_pos > boundMin &&
				now_vel <= velThresh
				)
			{

				// Check if rat in any bounds
				for (int i = 0; i < targ_lng; i++)
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
						is_trigger_ready = true;
					}
				}
			}

			// Reset flag
			is_ekf_new = false;
		}
		return is_trigger_ready;
	}

	// Reset
	void Reset()
	{
		is_bound_set = false;
		is_trigger_ready = false;
		is_all_targ_passed = false;
		is_ekf_new = false;
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
	long l; // (long) 4 byte
	float f; // (float) 4 byte
}
// Initialize object
u;

#pragma endregion 


// ---------SETUP---------
void setup() {

	delayMicroseconds(100);
	//while (!SerialUSB);
	SerialUSB.begin(0);

	// XBee
	Serial1.begin(57600);

	// Set button pins enable internal pullup
	for (int i = 0; i <= 2; i++) {
		pinMode(pin_Btn[i], INPUT_PULLUP);
	}
	pinMode(pin_FeedSwitch, INPUT_PULLUP);

	// Set output pins
	pinMode(pin_Rel_Ens, OUTPUT);
	pinMode(pin_Rel_Eth, OUTPUT);
	pinMode(pin_Disp_Pwr, OUTPUT);
	pinMode(pin_XBee_Pwr, OUTPUT);
	pinMode(pin_ED_STP, OUTPUT);
	pinMode(pin_ED_DIR, OUTPUT);
	pinMode(pin_ED_SLP, OUTPUT);
	pinMode(pin_ED_MS1, OUTPUT);
	pinMode(pin_ED_MS2, OUTPUT);
	pinMode(pin_ED_ENBL, OUTPUT);
	pinMode(pin_PwrOff, OUTPUT);
	pinMode(pin_IRprox_Pwr, OUTPUT);
	pinMode(pin_IRdetect_Pwr, OUTPUT);
	pinMode(pin_IRdetect_Gnd, OUTPUT);

	// Set power/ground pins
	digitalWrite(pin_Rel_Ens, LOW);
	digitalWrite(pin_Rel_Eth, LOW);
	digitalWrite(pin_Disp_Pwr, HIGH);
	digitalWrite(pin_XBee_Pwr, HIGH);
	digitalWrite(pin_IRprox_Pwr, HIGH);
	digitalWrite(pin_IRdetect_Pwr, HIGH);
	digitalWrite(pin_IRdetect_Gnd, LOW);

	// Start BigEasyDriver in sleep
	digitalWrite(pin_ED_SLP, LOW);

	// Define external interrupt
	attachInterrupt(digitalPinToInterrupt(pin_Btn[0]), Interupt_Btn1, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_Btn[1]), Interupt_Btn2, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_Btn[2]), Interupt_Btn3, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_IRprox_Rt), Interupt_IRprox_Halt, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_IRprox_Lft), Interupt_IRprox_Halt, FALLING);

	// Initialize AutoDriver

	// Configure SPI
	ad_R.SPIConfig();
	ad_F.SPIConfig();
	delayMicroseconds(100);
	// Reset each axis
	ad_R.resetDev();
	ad_F.resetDev();
	delayMicroseconds(100);
	// Configure each axis
	dSPINConfig_board();
	delayMicroseconds(100);
	// Get the status to clear the UVLO Flag
	ad_R.getStatus();
	ad_F.getStatus();

	// Print ad board status
	SerialUSB.println(' ');
	SerialUSB.print("BOARD R STATUS: ");
	SerialUSB.println(ad_R.getStatus(), HEX);
	SerialUSB.print("BOARD F STATUS: ");
	SerialUSB.println(ad_F.getStatus(), HEX);

	// Initialize LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);
	myGLCD.invert(true);

	// Initailize Pixy
	pixy.init();
	Wire.begin();
	
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

	// Make sure motor is stopped and in high impedance
	ad_R.hardHiZ();
	ad_F.hardHiZ();
	//RunMotor('f', 0, "None");

}


// ---------MAIN LOOP---------
void loop() {

	// Store loop time
	t_loopMain = millis();

#pragma region //--- FIRST PASS SETUP ---
	if (fc_isFirstPass)
	{
		// Blink to show setup done
		SetupBlink();

		// Make sure Xbee buffer empty
		while (Serial1.available() > 0) Serial1.read();

		// Make sure relays are off
		digitalWrite(pin_Rel_Ens, LOW);
		digitalWrite(pin_Rel_Eth, LOW);

		// Clear LCD
		ClearLCD();

		// Reset volatiles
		vol_doChangeSolState = false;
		vol_doRew = false;
		vol_doChangeLCDstate = false;
		vol_doIRhardStop = false;

		fc_isFirstPass = false;
		DebugState("RESET");

		//// TEST
		//float now_pos = 348 + (140 * PI);
		//float rew_pos = 92;
		//for (int i = 0; i < 100; i++)
		//{
		//	reward.is_bound_set = false;
		//	reward.CompTargBounds(now_pos, rew_pos);
		//	now_pos = now_pos + 10 + (140 * PI);
		//}

	}

#pragma endregion

#pragma region //--- PRINT DEBUG ---

	// Print debug
	if (doPrint)
	{
		PrintDebug();
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
		SendData();
	}
#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (msg_id == 'S' && msg_pass)
	{
		// Set condition
		if (msg_setupCmd[0] == 1)
		{
			fc_isManualSes = false;
			DebugState("DO TRACKING");
		}
		else
		{
			fc_isManualSes = true;
			DebugState("DONT DO TRACKING");
		}
		// Set reward tone
		if (msg_setupCmd[1] == 0)
		{
			// No sound
			Store4_Ard('s', 0);
			DebugState("NO SOUND");
		}
		else if (msg_setupCmd[1] == 1)
		{
			// Use white noise only
			Store4_Ard('s', 1);
			DebugState("DONT DO TONE");
		}
		else
		{
			// Use white and reward noise
			Store4_Ard('s', 2);
			DebugState("DO TONE");
		}

		// Make sure lcd led is off
		if (!fc_isManualSes &&
			lcdLightOn)
		{
			vol_doChangeLCDstate = true;
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
		DebugState("DO QUIT");

		// Hold all motor control
		fc_isHalted = true;
		SetMotorControl("None", "MsgQ");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (fc_doQuit && millis() > t_quitCmd && !doSend)
	{
		// Retell ard to quit
		Store4_Ard('q', 255);
		DebugState("QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (msg_id == 'M' && msg_pass)
	{
		fc_doMove = true;
		DebugState("DO MOVE");
	}

	// Perform movement
	if (fc_doMove)
	{

		// Compute move target
		if (!targ_moveTo.is_targ_set)
		{
			// If succesfull
			if (targ_moveTo.CompTarg(ekf_robPos, msg_moveToTarg, -1 * feedDist))
			{
				// Start running
				if (
					SetMotorControl("MoveTo", "MsgM") &&
					RunMotor(targ_moveTo.move_dir, moveToSpeed, "MoveTo")
					)
				{
					// Print message
					char str[50];
					sprintf(str, "MOVEING FROM %0.2fcm TO %0.2fcm BY %0.2fcm",
						targ_moveTo.pos_start, targ_moveTo.offset_target, targ_moveTo.targ_dist);
					DebugState(str);
				}
				// Reset motor cotrol if run fails
				else SetMotorControl("Open", "MsgM");
			}
		}

		// Check if robot is ready to be stopped
		if (targ_moveTo.is_targ_set)
		{
			// Do deceleration
			float new_speed = targ_moveTo.DecelToTarg(ekf_robPos, ekf_robVel, 40, 10);

			// Change speed if > 0
			if (new_speed > 0)
			{
				RunMotor(targ_moveTo.move_dir, new_speed, "MoveTo");
			}
			// Check if target reached
			else if (targ_moveTo.is_targ_reached)
			{
				// Hard stop
				HardStop("MsgM");

				// Hold motor control for 5 sec
				BlockMotorTill(5000);

				// Tell CS movement is done
				Store4_CS('D', c2r_packLast[CharInd('M', "c2r")]);

				// Reset flags
				fc_doMove = false;
				targ_moveTo.Reset();

				// Print message
				char str[50];
				sprintf(str, "FINISHED MOVE TO %0.2fcm WITHIN %0.2fcm",
					targ_moveTo.offset_target, targ_moveTo.GetError(ekf_robPos));
				DebugState(str);
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

			DebugState("REWARD NOW");
			// Start reward
			StartRew(true);
		}
		else
		{
			fc_doRew = true;
			DebugState("RUN REWARD");
		}
	}

	// Perform reward
	if (fc_doRew)
	{
		// If not rewarding 
		if (!fc_isRewarding)
		{
			// Compute reward bounds
			if (!reward.is_bound_set)
			{
				reward.CompTargBounds(ekf_ratPos, msg_rewPos);
				// Print message
				char str[50];
				sprintf(str, "REWARD AT %0.2fcm FROM %0.2fcm TO %0.2fcm",
					reward.rewCenter, reward.targBounds[0][0], reward.targBounds[8][1]);
				DebugState(str);
			}
			else if (!reward.is_trigger_ready)
			{
				bool ekf_pass = reward.CheckTargBounds(ekf_ratPos, ekf_ratVel);
				bool raw_pass = false; // reward.CheckTargBounds(vtpixyPosAvg, vtpixyVelAvg);
				if (ekf_pass || raw_pass)
				{
					// Start reward
					StartRew(true);
					// Print message
					char str[50];
					sprintf(str, "REWARDED TARG %0.2fcm BOUNDS %0.2fcm TO %0.2fcm USING %s",
						reward.rewardedTarg, reward.rewardedBounds[0], reward.rewardedBounds[1], ekf_pass ? "EKF" : "Raw");
					DebugState(str);
				}
			}
			// Check if rat bassed all bounds
			if (
				reward.is_all_targ_passed &&
				!reward.is_trigger_ready
				)
			{
				// Reset flags
				reward.Reset();
				fc_doRew = false;
			}
		}
		// End ongoing reward
		else if (EndRew())
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
			if (!(targ_cueRat.is_targ_set && targ_cueRob.is_targ_set))
			{
				// Compute target targ_dist for rat and robot
				if (
					targ_cueRat.CompTarg(ekf_ratPos, msg_cueTarg, 0) &&
					targ_cueRob.CompTarg(ekf_robPos, msg_cueTarg, -1 * defualtSetPoint)
					)
				{
					// Print message
					char str[50];
					sprintf(str, "CUEING REWARD AT %0.2fcm/%0.2fcm FROM DIST %0.2fcm/%0.2fcm",
						targ_cueRat.offset_target, targ_cueRob.offset_target, targ_cueRat.targ_dist, targ_cueRob.targ_dist);
					DebugState(str);
				}
			}

			// Check if cue has been reached
			if (targ_cueRat.is_targ_set && targ_cueRob.is_targ_set)
			{
				// Check if robot reached targed
				if (!targ_cueRob.is_targ_reached)
				{
					if (
						targ_cueRob.CheckTargReached(pos_robVT.posNow, ekf_robVel) ||
						targ_cueRob.CheckTargReached(ekf_robPos, ekf_robVel)
						)
					{
						DebugState("ROBOT REACHED CUE TARGET");
						// Hard stop
						HardStop("MsgC");
						// Set flag
						fc_isCueing = true;
						// Set hold time
						BlockMotorTill(reward.t_block);
					}
				}
				// Check if rat reached target
				if (!targ_cueRat.is_targ_reached)
				{
					if (
						targ_cueRat.CheckTargReached(vtpixyPosAvg, 0) ||
						targ_cueRat.CheckTargReached(ekf_ratPos, 0)
						)
					{
						DebugState("RAT REACHED CUE TARGET");
						// Trigger reward without stopping
						StartRew(false);
					}
				}
				if (targ_cueRat.is_targ_reached && targ_cueRob.is_targ_reached)
				{
					// Print message
					char str[50];
					sprintf(str, "FINISHED CUEING AT %0.2fcm/%0.2fcm WITHIN %0.2fcm/%0.2fcm",
						targ_cueRat.offset_target, targ_cueRob.offset_target, targ_cueRat.GetError(ekf_ratPos), targ_cueRob.GetError(ekf_robPos));
					DebugState(str);
				}
			}
		}

		// End ongoing reward
		else if (EndRew())
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
			DebugState("HALT STARTED");
			// Stop pid and set to manual
			HardStop("MsgH");
			// Remove motor control
			fc_isHalted = true;
			SetMotorControl("None", "MsgH");
			fc_doHalt = false;
		}
		else
		{
			DebugState("HALT FINISHED");
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
		static bool is_mode_changed;

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
				DebugState("DO BULLDOZE");
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
				DebugState("DONT DO BULLDOZE");
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
				DebugState("BULLDOZE ON");
			}
			else
			{
				// Turn bulldoze off
				bull.TurnOff("MsgB");
				DebugState("BULLDOZE OFF");
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
			DebugState("RAT IN");
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

			DebugState("RAT OUT");
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
		Store4_CS('D', c2r_packLast[CharInd('V', "c2r")]);
		fc_doStreamCheck = false;
		DebugState("STREAMING CONFIRMED");
	}
#pragma endregion

#pragma region //--- (Y) DONE RECIEVED ---
	if (msg_id == 'Y' && msg_pass)
	{
		fc_doCheckDoneRcvd = false;
		DebugState("CS RESIEVED 'DONE'");
	}
	// Request done recieved confirmation
	if (fc_doCheckDoneRcvd && millis() > t_resendDone)
	{
		// Resend done confirmation
		Store4_CS('D', r2c_packLast[CharInd('D', "r2c")]);
		// Send again after 1 sec
		t_resendDone = millis() + 1000;
		DebugState("RESENT 'D'");
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

#pragma region //--- INTERUPT TRIGGERED ---

	// Open/close solonoid
	if (vol_doChangeSolState)
	{
		OpenCloseSolonoid();
		vol_doChangeSolState = false;
	}

	// Button triggered reward
	if (vol_doRew)
	{
		if (!fc_isRewarding)
		{
			StartRew(false);
		}
		else if (EndRew())
		{
			vol_doRew = false;
		}
	}

	// Turn LCD on/off
	if (vol_doChangeLCDstate)
	{
		ChangeLCDlight();
		vol_doChangeLCDstate = false;
	}

	// IR triggered halt
	if (vol_doIRhardStop)
	{
		Function_IRprox_Halt();
		vol_doIRhardStop = false;
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

}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{
	byte buff;
	char head[2];
	char foot;
	bool pass;
	bool loop;
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

			// Get sync time
			t_sync = millis();
			// send sync time cmd to ard
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
	byte buff;
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
	bool pass;
	int pack_diff;
	int dropped_packs;
	bool do_use_pack;
	int id_ind;

	// SEND AND PRINT PACKET CONFIRMATON
	if (id != 'P')
	{
		// Print revieved pack details
		DebugRsvd(id, pack);

		// Send back packet confirmation
		if (id != 'Y')
		{
			// Send revieved pack
			Store4_CS(id, pack);
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
void Store4_CS(char id, uint16_t pack)
{
	// Local vars
	int queue_ind;

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
	r2_queue[queue_ind][2] = (byte)255;
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
	int queue_ind;

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

// SEND SERIAL DATA
void SendData()
{
	// Local vars
	const int ard_msg_size = r2_lngC - 1;
	const int cs_msg_size = r2_lngC - 2;
	static byte msg[ard_msg_size];
	char rcv_id;
	bool do_send = false;
	int buff_tx;

	// save reviever id
	rcv_id = r2_queue[r2_lngR - 1][6];

	// Move next in queue to temp msg array
	for (int j = 0; j < ard_msg_size; j++)
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
		// Skip data byte for cs mesages
		if (rcv_id == 'a')
		{
			Serial1.write(msg, ard_msg_size);
		}
		else
		{
			byte cs_msg[cs_msg_size];
			// Move next in queue to temp msg array
			int m1_ind = -1;
			int m2_ind = -1;
			for (int j = 0; j < ard_msg_size; j++)
			{
				m1_ind++;
				if (j == 2) continue;
				m2_ind++;
				cs_msg[m2_ind] = msg[m1_ind];
			}
			Serial1.write(cs_msg, cs_msg_size);
		}

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
			(doDebugConsole || doDebugLCD))
		{
			char str[50];
			char id;
			byte dat;
			uint16_t pack;

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
			StoreDebugStr(str);
		}

	}
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
	DebugState("HARD STOP [" + called_from + "]");
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
			ad_F.run(FWD, speed_steps);
		}
		else if (dir == 'r')
		{
			ad_R.run(REV, speed_steps);
			ad_F.run(REV, speed_steps);
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
			ekf_ratPos - (ekf_robPos + feedDist) > 0 &&
			pos_ratVT.posNow - (ekf_robPos + feedDist) > 0 &&
			pos_ratPixy.posNow - (ekf_robPos + feedDist) > 0;

		// Check for time elapsed or rat moved at least 3cm past feeder
		if (millis() > t_blockTill || passed_feeder)
		{

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
		int n_laps;
		float cm_diff;
		float cm_dist;

		// Check that rat pos < robot pos
		n_laps = pos_ratVT.posNow > pos_robVT.posNow ? 0 : 1;
		if (n_laps > 0)
		{
			DebugState("SET RAT POS AHEAD");
		}
		else
		{
			DebugState("RAT POS ALREADY AHEAD");
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
				DebugState("PID STARTED");
			}

			// Initialize bulldoze
			if (fc_doBulldoze)
			{
				// Run from initial blocked mode
				bull.TurnOn("InitializeTracking");
				DebugState("BULLDOZE INITIALIZED");
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
		uint32_t vt_dt;
		uint32_t pixy_dt;

		// Compute dt
		vt_dt = millis() - pos_ratVT.msNow;
		pixy_dt = millis() - pos_ratPixy.msNow;

		// Check pixy
		if (
			pixy_dt >= max_frame_dt &&
			vt_dt < max_frame_dt
			)
		{
			// Use VT for Pixy data
			pos_ratPixy.SetDat(pos_ratVT.posAbs, pos_ratVT.msNow);
		}
		// Check VT 
		else if (
			vt_dt >= max_frame_dt &&
			pixy_dt < max_frame_dt
			)
		{
			// Use Pixy for VT data
			pos_ratVT.SetDat(pos_ratPixy.posAbs, pos_ratPixy.msNow);
		}

	}

}

// PROCESS PIXY STREAM
void UpdatePixyPos() {

	// Local vars
	static float pxRel;
	static double pxAbs;
	static uint32_t pxTS;

	// Get new blocks
	uint16_t blocks = pixy.getBlocks();

	// Check for new data
	if (blocks)
	{

		// Save time stamp
		pxTS = millis();

		// Get Y pos from last block and convert to CM
		double pixyPosY = pixy.blocks[blocks - 1].y;
		pxRel =
			pixyCoeff[0] * (pixyPosY * pixyPosY * pixyPosY * pixyPosY) +
			pixyCoeff[1] * (pixyPosY * pixyPosY * pixyPosY) +
			pixyCoeff[2] * (pixyPosY * pixyPosY) +
			pixyCoeff[3] * pixyPosY +
			pixyCoeff[4];

		// Scale to abs space with rob vt data
		pxAbs = pxRel + pos_robVT.posAbs;
		if (pxAbs > (140 * PI))
		{
			pxAbs = pxAbs - (140 * PI);
		}
		// Update pixy pos and vel
		pos_ratPixy.UpdatePosVel((float)pxAbs, pxTS);

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

		// Update PID loop time
		if (fc_isRatIn)
		{
			// Update pid next loop time
			pid.SetLoopTime(millis());
			// Set flag for reward 
			reward.is_ekf_new = true;
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
		ekf_ratPos = ekf.getX(0);
		ekf_robPos = ekf.getX(1);
		ekf_ratVel = ekf.getX(2);
		ekf_robVel = ekf.getX(3);

	}
}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// START REWARD
void StartRew(bool do_stop)
{
	// Track rewards
	rewCnt++;

	// Stop robot
	if (do_stop)
	{
		HardStop("StartRew");
		// Set hold time
		BlockMotorTill(reward.t_block);
	}

	// Trigger reward tone on
	Store4_Ard('r', reward.duration_byte);

	// Compute reward end time
	reward.t_end = millis() + reward.duration;

	// Turn on reward LED
	analogWrite(pin_RewLED_R, rewLEDduty);
	analogWrite(pin_RewLED_C, rewLEDduty);

	// Open solenoid
	digitalWrite(pin_Rel_Ens, HIGH);

	// Print to LCD for manual rewards
	if (vol_doRew)
	{
		PrintLCD("REWARDING...");
	}
	else
	{
		char str[50];
		sprintf(str, "REWARDING(%dms)...", reward.duration);
		DebugState(str);
	}

	// indicate reward in progress
	fc_isRewarding = true;
}

// END REWARD
bool EndRew()
{
	bool is_rew_end = false;

	if (millis() > reward.t_end)
	{

		// Close solenoid
		digitalWrite(pin_Rel_Ens, LOW);

		// Turn off reward LED
		analogWrite(pin_RewLED_R, rewLEDmin);
		analogWrite(pin_RewLED_C, rewLEDmin);

		// Clear LCD
		if (vol_doRew)
		{
			ClearLCD();
		}
		else
		{
			DebugState("REWARD FINISHED");
		}

		// Set flags
		fc_isRewarding = false;
		is_rew_end = true;
	}
	return is_rew_end;
}

// OPEN/CLOSE SOLONOID
void OpenCloseSolonoid()
{
	// Local vars
	byte sol_state = digitalRead(pin_Rel_Ens);

	// Change state
	sol_state = !sol_state;

	// Open/close solenoid
	digitalWrite(pin_Rel_Ens, sol_state);

	// Print to LCD
	char str[50];
	sprintf(str, "%s", digitalRead(pin_Rel_Ens) == HIGH ? "OPEN" : "CLOSED");
	PrintLCD("SOLONOID", str);
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

void StoreDebugStr(String str)
{
	// Local vars
	char t_str_long[50];
	char t_str_ellapsed[50];
	static uint32_t t_last = millis();
	uint32_t ts;
	float t_c;
	float t_m;
	float t_ellapsed;

	// Time now
	ts = t_sync == 0 ? millis() : t_sync;

	// Total time
	t_c = (float)((millis() - ts) / 1000.0f);
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

void PrintDebug()
{

	if ((doDebugLCD && !vol_doBlockLCDlog) ||
		doDebugConsole)
	{

		// Print to LCD
		if (doDebugLCD && !vol_doBlockLCDlog)
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
		if (doDebugConsole)
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

void DebugMotorControl(String set_to, String called_from)
{
	if (doPrint_motorControl && (doDebugConsole || doDebugLCD))
	{
		String str;
		// Store
		str = "mc set:" + fc_motorControl + " " + "in:" + set_to + " [" + called_from + "]";
		StoreDebugStr(str);
	}
}

void DebugDropped(int missed, int missed_total, int total)
{
	if (doPrint_dropped && (doDebugConsole || doDebugLCD))
	{
		char str[50];
		// Store
		sprintf(str, "!dropped p:%d [t:%d/%d]!", missed, missed_total, total);
		StoreDebugStr(str);
	}
}

void DebugResent(char id, uint16_t pack, int total)
{
	if (doPrint_resent && (doDebugConsole || doDebugLCD))
	{
		char str[50];
		// Store
		sprintf(str, "!resent i:%c p:%d t:%d!", id, pack, total);
		StoreDebugStr(str);
	}
}

void DebugRsvd(char id, uint16_t pack)
{
	//// Print
	if (doPrint_rcvd && (doDebugConsole || doDebugLCD))
	{
		char str[50];

		// Print specific pack contents
		if (id == 'S')
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

		// Store
		StoreDebugStr(str);

	}
}

void DebugState(String msg)
{
	if (doPrint_flow && (doDebugConsole || doDebugLCD))
	{
		// Store
		StoreDebugStr(msg);
	}
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

void PrintLCD(String str1)
{

	// Change settings
	myGLCD.setFont(SmallFont);
	myGLCD.invert(true);

	// Clear
	myGLCD.clrScr();

	// Print
	myGLCD.print(str1, LEFT, 20);

	// Update
	myGLCD.update();

	// Block LCD log
	vol_doBlockLCDlog = true;

}
void PrintLCD(String str1, String str2)
{

	// Change settings
	myGLCD.setFont(SmallFont);
	myGLCD.invert(true);

	// Clear
	myGLCD.clrScr();

	// Print
	myGLCD.print(str1, CENTER, 15);
	myGLCD.print(str2, CENTER, 25);

	// Update
	myGLCD.update();

}

void ClearLCD()
{
	// Clear
	myGLCD.clrScr();

	// Update
	myGLCD.update();

	// Stop blocking LCD log
	vol_doBlockLCDlog = false;
}

void SetupBlink()
{
	int duty[2] = { 100, 0 };
	bool isOn = false;
	int del = 100;
	// Flash sequentially
	for (int i = 0; i < 8; i++)
	{
		analogWrite(pin_Disp_LED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin_TrackLED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin_RewLED_R, duty[(int)isOn]);
		delay(del);
		isOn = !isOn;
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
	bool isOn = true;
	int del = 50;

	// Flash 
	for (int i = 0; i < 3; i++)
	{
		analogWrite(pin_TrackLED, duty[(int)isOn]);
		delay(del);
		isOn = !isOn;
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

// Trigger reward
void Interupt_Btn1() {

	// exit if < reward time has not passed
	if (t_debounce[1] > millis()) return;

	// Set to start reward function
	vol_doRew = true;

	t_debounce[1] = millis() + reward.duration + 100;

}

// Open/close solonoid
void Interupt_Btn2() {

	// exit if < 250 ms has not passed
	if (t_debounce[0] > millis()) return;

	// Run open close function
	vol_doChangeSolState = true;

	t_debounce[0] = millis() + 250;
}

// Turn on/off LCD LED
void Interupt_Btn3() {

	// exit if < 250 ms has not passed
	if (t_debounce[2] > millis()) return;

	vol_doChangeLCDstate = true;

	t_debounce[2] = millis() + 250;

}

// Halt run on IR trigger
void Interupt_IRprox_Halt() {

	// exit if < 250 ms has not passed
	if (t_debounce[3] > millis()) return;

	// Run stop in main loop
	vol_doIRhardStop = true;

	t_debounce[3] = millis() + 250;
}


#pragma endregion

