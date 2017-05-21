
//-------FEEDERDUE-------

#pragma region ---------NOTES---------

/*
	* XBee DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size)
	* XBee DO (to UART rx) buffer = 202 bytes
	* DUE SERIAL_BUFFER_SIZE = 128
	* SerialUSB receive buffer size is now 512 (ARDUINO 1.5.2 BETA - 2013.02.06)
	* DATA TYPES:
		byte = 1 byte
		char = 1 byte
		int = 4 byte
		long = 4 byte
		float = 4 byte
*/

#pragma endregion


#pragma region ---------DEBUG SETTINGS---------

// LOG DEBUGGING

// Do log
bool doDB_Log = true;

// What to print
const bool doLog_flow = true;
const bool doLog_irSync = true;
const bool doLog_motorControl = true;
const bool doLog_motorBlocking = true;
const bool doLog_rcvd = true;
const bool doLog_r2c = true;
const bool doLog_r2a = true;
const bool doLog_pid = true;
const bool doLog_bull = true;
const bool doLog_resent = true;
const bool doLog_dropped = true;

// PRINT DEBUGGING

// Where to print
const bool doDB_PrintConsole = false;
const bool doDB_PrintLCD = false;

// What to print
const bool doPrint_flow = false;
const bool doPrint_irSync = false;
const bool doPrint_motorControl = false;
const bool doPrint_motorBlocking = false;
const bool doPrint_rcvd = false;
const bool doPrint_r2c = false;
const bool doPrint_r2a = false;
const bool doPrint_pid = false;
const bool doPrint_bull = false;
const bool doPrint_resent = false;
const bool doPrint_dropped = false;
const bool doPrint_log = false;

// TESTING

// Test message
byte msg_testCmd[2];

// Position
const bool do_posDebug = false;

// PID Calibration
/*
Set kC and run ICR_Run.cs
*/
const float kC = 5; // critical gain [2,3,4,5]
const float pC = 1.5; // oscillation period [0,0,0,1.5]  
const float cal_speedSteps[4] = { 10, 20, 30, 40 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;
bool do_pidCalibration = false;

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

// Flow/state control
String fc_motorControl = "None"; // ["None", "Open", "MoveTo", "Bull", "PID"]
bool fc_isBlockingTill = false;
bool fc_doQuit = false;
bool fc_isFirstPass = true;
bool fc_doStreamCheck = false;
bool fc_isStreaming = false;
bool fc_isManualSes = false;
bool fc_isRatIn = false;
bool fc_isTrackingEnabled = false;
bool fc_isRewarding = false;
bool fc_doMove = false;
bool fc_doRew = false;
bool fc_doHalt = false;
bool fc_isHalted = false;
bool fc_doBulldoze = false;
bool fc_doCheckDoneRcvd = false;
bool fc_isSendingLog = false;
bool fc_doLogResend = false;
bool fc_doLogSend = false;
bool fc_isEKFReady = false;

// Log debugging
const int logLng = 1000;
String logList[logLng];
int cnt_logStore = 0;
int cnt_logSend = 0;
int logByteTrack = 0;
int logByteMax = 32000;

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
int cnt_loop;
int cnt_droppedPacks = 0;
int cnt_overflowEvt = 0;
int cnt_timeoutEvt = 0;
int cnt_packResend = 0;

// Outgoing packet data
byte r2_queue[10][8];
const int r2_lngR = 10;
const int r2_lngC = 8;
int sendQueueInd = r2_lngR - 1;
bool doPackSend = false;
const int dt_sendSent = 1;
const int dt_sendRcvd = 1;
const int dt_logSent = 1;
const int dt_logRcvd = 1;

// Serial from CS
const char c2r_head = '<';
const char c2r_foot = '>';
char c2r_id[14] = {
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
const int c2r_idLng = sizeof(c2r_id) / sizeof(c2r_id[0]);
uint16_t c2r_packLast[c2r_idLng];
int c2r_cntRepeat[c2r_idLng];
char c2r_msg_id = ' ';
bool c2r_isNew = false;
uint32_t t_rcvd = millis(); // (ms)
uint32_t t_rcvdLast = 0; // (ms)

// Serial to CS
const char r2c_head = '<';
const char r2c_foot = '>';
char r2c_id[15] = {
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
	'Z', // reward zone
};
const int r2c_idLng = sizeof(r2c_id) / sizeof(r2c_id[0]);
uint32_t t_resendDone = millis(); // (ms)
uint32_t t_sent = millis(); // (ms)
uint16_t r2c_packLast[r2c_idLng];
byte r2c_datLast[r2c_idLng];

// Robot to Matlab communication
const char r2m_id[2] = {
	'J', // battery voltage
	'Z', // reward zone
};
const int r2m_idLng = sizeof(r2m_id) / sizeof(r2m_id[0]);

// Serial to other ard
const char r2a_head = '{';
const char r2a_foot = '}';
uint16_t r2a_packCnt = 0;
char r2a_id[6] = {
	't', // set sync time
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
};
const int r2a_idLng = sizeof(r2a_id) / sizeof(r2a_id[0]);
uint16_t r2a_packLast[r2a_idLng];
byte r2a_datLast[r2a_idLng];
uint16_t r2a_sendTim[r2a_idLng];
bool r2a_doRcvdCheck[r2a_idLng];
int r2a_resendCnt[r2a_idLng];
uint16_t r2a_resendDel = 100;
uint16_t r2a_resendMax = 3;

// Serial from other ard
const char a2r_head = '{';
const char a2r_foot = '}';

// Start/Quit
byte msg_setupCmd[2];
uint32_t t_quitCmd = 0;

// Serial VT
byte msg_vtEnt = 0;
float msg_vtCM[2] = { 0,0 };
uint32_t t_msg_vtTS[2] = { 0,0 };

// Pixy
const double pixyCoeff[5] = {
	0.000000043550534,
	-0.000023239535204,
	0.005033059128963,
	-0.677050955917591,
	75.424132382709260
};

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
int dt_blockMotor = 0;

// Kalman model measures
float ekfRatPos = 0;
float ekfRobPos = 0;
float ekfRatVel = 0;
float ekfRobVel = 0;
float ekfSetPos = 0;

// PID Settings
bool do_includeTerm[2] = { true, true };
const float defualtSetPoint = 42; // (cm)
const float guardDist = 4.5;
const float feedDist = 66;
float errorFeeder = 0;
float errorDefault = 0;

// Movement
float moveToSpeed = 80;
float msg_moveToTarg = 0;
float moveToDist = 0;
float moveToStartPos = 0;
byte msg_bullDel = 0;
byte msg_bullSpeed = 0;

// Reward
int dt_blockRew = 10000; // (ms)
float cuePos = 0;
float cueDist[2] = { 0,0 };
float cueStartPos[2] = { 0,0 };
float msg_rewPos = 0;
byte msg_rewDelay = 0;
byte msg_rewZoneInd = 0;

// EtOH 
/*
EtOH run after min time or distance
*/
bool doEtOHRun = true;
bool isEtOHOpen = false;
const int dt_durEtOH = 1000; // (ms)
const int dt_delEtOH = 30000; // (ms)
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
bool btn_doRewSolStateChange = false;
bool btn_doEtOHSolStateChange = false;
bool btn_doRew = false;
bool btn_doChangeLCDstate = false;

// Interrupts 
volatile uint32_t t_irProxDebounce = millis();
volatile bool t_doIRhardStop = false;
volatile uint32_t t_irDetectDebounce = millis();
volatile uint32_t t_irSyncLast = millis();
volatile uint32_t t_sync = 0;
volatile bool intrpt_doLogIR = false;

#pragma endregion 


#pragma region ---------CONSTRUCT FUNCTIONS---------
// PARSE SERIAL INPUT
bool ParseSerial();
// PARSE CS MESSAGE
bool ParseC2R();
// PARSE CHEEAH DUE MESSAGE
void ParseA2R();
// WAIT FOR BUFFER TO FILL
byte WaitBuffRead();
byte WaitBuffRead(char chr1);
byte WaitBuffRead(char chr1, char chr2);
// CHECK AND PROCESS RCVD PACKET NUMBER
bool CheckPack(char id, uint16_t pack);
// STORE PACKET DATA TO BE SENT
void StorePacketData(char targ, char id);
void StorePacketData(char targ, char id, byte d1);
void StorePacketData(char targ, char id, byte d1, uint16_t pack);
void StorePacketData(char targ, char id, byte d1, uint16_t pack, bool conf);
// SEND SERIAL PACKET DATA
void SendPacketData();
// SEND SERIAL LOG DATA
void SendLogData();
// CHECK IF ARD TO ARD PACKET SHOULD BE RESENT
bool CheckArdResend();
// HARD STOP
void HardStop(String called_from);
// IR TRIGGERED HARD STOP
void Function_IRprox_Halt();
// RUN AUTODRIVER
bool RunMotor(char dir, float speed, String agent);
// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(String set_to, String called_from);
// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(int dt, String called_from);
// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed();
// DO SETUP TO BEGIN TRACKING
void InitializeTracking();
// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT();
// PROCESS PIXY STREAM
void UpdatePixyPos();
// UPDATE EKF
void UpdateEKF();
// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid();
// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid();
// CHECK FOR ETOH UPDATE
void CheckEtOH();
// CHECK BATTERY VOLTAGE
void CheckBattery();
// TURN LCD LIGHT ON/OFF
void ChangeLCDlight();
// CHECK FOR BUTTON INPUT
void CheckButtons();
// QUIT AND RESTART ARDUINO
void QuitSession();
// LOG/PRING MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, String set_to, String called_from);
// LOG/PRING MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(String msg, uint32_t t, String called_from);
// LOG/PRING DROPPED PACKET DEBUG STRING
void DebugDropped(int missed, int missed_total, int total);
// LOG/PRING RESENT PACKET DEBUG STRING
void DebugResent(char id, uint16_t pack, int total);
// LOG/PRING RECIEVED PACKET DEBUG STRING
void DebugRcvd(char from, char id, uint16_t pack);
// LOG/PRING IR SENSOR EVENT
void DebugIRSync(String msg, uint32_t t);
// LOG/PRING MAIN EVENT
void DebugFlow(String msg);
// STORE STRING FOR PRINTING
void StoreDBPrintStr(String msg, uint32_t t);
// STORE STRING FOR LOGGING
void StoreDBLogStr(String msg, uint32_t t);
// PRINT DEBUG STRINGS TO CONSOLE/LCD
void PrintDebug();
// FOR PRINTING TO LCD
void PrintLCD(String str_1);
void PrintLCD(String str_1, String str_2, char f_siz);
void ClearLCD();
// GET ID INDEX
int ID_Ind(char id, char id_arr[], int arr_size);
// BLINK LEDS AT SETUP
void SetupBlink();
// BLICK LEDS WHEN RAT FIRST DETECTED
void RatInBlink();
// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt();
// DETECT IR SYNC EVENT
void Interupt_IR_Detect();
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
	String objID = " ";
	int nSamp = 0;
	float posArr[6] = { 0,0,0,0,0,0 };
	uint32_t t_tsArr[6] = { 0,0,0,0,0,0 };
	int dt_skip = 0;
	float velNow = 0.0f;
	float posNow = 0.0f;
	float posAbs = 0.0f;
	uint32_t t_tsNow = 0;
	uint32_t t_msNow = millis();
	float nLaps = 0;
	int sampCnt = 0;
	bool newData = false;
	int dt_frame = 0;

	// constructor
	PosDat(String id, int l)
	{
		this->objID = id;
		this->nSamp = l;
		for (int i = 0; i < l; i++) this->posArr[i] = 0.0f;
	}

	void UpdatePos(float pos_new, uint32_t ts_new)
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
			this->t_tsArr[i] = this->t_tsArr[i + 1];
		}
		// Add new variables
		this->posArr[nSamp - 1] = pos_new;
		this->t_tsArr[nSamp - 1] = ts_new;

		// Do not process early samples
		if (this->sampCnt < this->nSamp + 1)
		{
			this->posNow = pos_new;
			this->velNow = 0.0f;
			this->dt_frame = 0;
			newData = false;
		}
		else
		{
			newData = true;

			// Store frame capture dt
			this->dt_frame = this->t_tsArr[nSamp - 1] - this->t_tsArr[nSamp - 2];

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
				dt = this->t_tsArr[i + 1] - this->t_tsArr[i];
				dtSum += (float)dt;
			}
			// compute vel
			vel = distSum / (dtSum / 1000.0f);

			// ignore outlyer values unless too many frames discarted
			if (this->dt_skip > 500 || abs(this->velNow - vel) < 150)
			{
				this->velNow = vel;
				this->dt_skip = 0;
			}
			// add to skip time
			else this->dt_skip += dt;
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

	void SetDat(float set_pos, uint32_t t)
	{
		// Compute ts
		uint32_t ts = t_tsNow + (t - t_msNow);

		// Update pos
		UpdatePos(set_pos, ts);
	}

	void SetPos(float set_pos, float set_laps)
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
PosDat pos_ratVT("pos_ratVT", 4);
PosDat pos_robVT("pos_robVT", 4);
PosDat pos_ratPixy("pos_ratPixy", 6);

//----------CLASS: PID----------
class PID
{

public:
	uint32_t t_lastLoop = 0;
	float dt_loop = 0;
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
	int dt_ekfSettle = 250; // (ms)
	bool ekfNew = false;
	float dampAcc = 40;
	float dampSpeedCut = 20;
	const int dt_damp = 4000;
	uint32_t t_dampTill = 0;
	uint32_t t_lastDamp = millis();

	// PID calibration
	int cal_dtMin = 40; // (ms)
	int cal_cntPcArr[4] = { 0, 0, 0, 0 };
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
		if (!fc_isEKFReady)
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
					i_term = kI*dt_loop*integral;
					d_term = kD / dt_loop*derivative;
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
		StorePacketData('a', 'p', 1);

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
			!fc_isRewarding
			)
		{
			StorePacketData('a', 'p', 0);
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
				t_dampTill = millis() + dt_damp;

				// Set flags
				isDampened = true;
				doDamp = false;

				char str[50];
				sprintf(str, "pid: dampen to %0.2f", dampAcc);
				//PrintPID(str);
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
			// Print taking conrol
			PrintPID("pid: take motor conrol [PID.CheckMotorControl]");

			// Run pid
			Run("PID.CheckMotorControl");
		}
		else if ((fc_motorControl != "PID" && fc_motorControl != "Open") &&
			mode == "Automatic")
		{
			// Print taking conrol
			PrintPID("pid: give up motor conrol [PID.CheckMotorControl]");

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

	void CheckEKF(uint32_t t)
	{
		if (!fc_isEKFReady)
		{
			if ((t - t_ekfStr) > dt_ekfSettle)
			{
				fc_isEKFReady = true;
			}
		}
	}

	void ResetEKF(String called_from)
	{
		fc_isEKFReady = false;
		t_ekfStr = millis();
		PrintPID("pid: reset ekf [" + called_from + "]");
	}

	void SetLoopTime(uint32_t t)
	{
		ekfNew = true;
		if (wasLoopRan)
		{
			dt_loop = (float)(t - t_lastLoop) / 1000.0f;
			wasLoopRan = false;
			t_lastLoop = t;
		}
	}

	void PrintPID(String str)
	{
		// Add to print queue
		if (doPrint_pid && (doDB_PrintConsole || doDB_PrintLCD)) {
			StoreDBPrintStr(str, millis());
		}
		// Add to log queue
		if (doLog_pid && doDB_Log) {
			StoreDBLogStr(str, millis());
		}
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
				cal_cntPcArr[3] == cal_nMeasPerSteps &&
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
				return 0;
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
					return -1;
				}

				// Check if speed should be incrimented
				if (cal_cntPcArr[cal_stepNow] == cal_nMeasPerSteps)
				{
					cal_PcArr[cal_stepNow] = cal_PcAvg;
					cal_PcSum = 0;
					cal_PcCnt = 0;
					cal_errArr[cal_stepNow] = cal_errAvg;
					cal_errSum = 0;
					cal_errCnt = 0;

					// Incriment step or bail if max reached
					if (cal_stepNow < 3) cal_stepNow++;
					else return -1;
				}

				// Get loop dt
				cal_dtLoop = (float)(millis() - t_lastLoop) / 1000.0f;
				t_lastLoop = millis();

				// Compute PID
				cal_ekfRatVel = cal_speedSteps[cal_stepNow];
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
						cal_cntPcArr[cal_stepNow]++;
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
	int dt_loop = 50; // (ms)
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
				fc_isEKFReady &&
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
				t_loopNext = millis() + dt_loop;
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
		StorePacketData('a', 'b', 1);

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
		StorePacketData('a', 'b', 0);

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
		if (doPrint_bull && (doDB_PrintConsole || doDB_PrintLCD)) {
			StoreDBPrintStr(str, millis());
		}
		// Add to log queue
		if (doLog_bull && doDB_Log) {
			StoreDBLogStr(str, millis());
		}
	}

};
// Initialize object
Bulldozer bull;

//----------CLASS: Target----------
class Target
{
public:
	String objID = " ";
	int targSetTimeout = 1000;
	int moveTimeout = 5000;
	uint32_t t_tryTargSetTill = 0;
	uint32_t t_tryMoveTill = 0;
	bool abortMove = false;
	float posRel = 0;
	float moveDiff = 0;
	float minSpeed = 0;
	const int dt_update = 10;
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

		// Get abort timeout
		if (t_tryTargSetTill == 0)
			t_tryTargSetTill = millis() + targSetTimeout;
		// Check if time out reached
		if (millis() > t_tryTargSetTill) {
			abortMove = true;
			return false;
		}

		// Run only if targ not set
		if (!isTargSet)
		{
			// Check pos data is ready
			if (fc_isEKFReady)
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

			// Get abort timeout
			if (t_tryMoveTill == 0)
				t_tryMoveTill = millis() + moveTimeout;
			// Check if time out reached
			if (millis() > t_tryMoveTill) {
				abortMove = true;
				return false;
			}

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
					t_updateNext = millis() + dt_update;
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
		abortMove = false;
		t_tryTargSetTill = 0;
		t_tryMoveTill = 0;
	}
};
// Initialize object
Target targ_moveTo("targ_moveTo");

//----------CLASS: Reward----------
class Reward
{
public:
	const int zoneRewDurs[9] = { // (ms)
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
	const float zoneLocs[9] = { // (deg)
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
	static const int zoneLng =
		sizeof(zoneLocs) / sizeof(zoneLocs[0]);
	float zoneBounds[zoneLng][2];
	int zoneOccTim[zoneLng];
	int zoneOccCnt[zoneLng];
	uint32_t t_nowZoneCheck = 0;
	uint32_t t_lastZoneCheck = 0;
	String mode = "None"; // ["None" "Free" "Cue" "Now"]
	int occThresh = 0; // (ms)
	int dt_block = 0; // (ms)
	int duration = 0; // (ms) 
	byte durationByte = 0; // (ms) 
	int zoneMin = 0;
	int zoneMax = 0;
	float boundMin = 0;
	float boundMax = 0;
	uint32_t t_closeSol = 0;
	uint32_t t_retractArm = 0;
	uint32_t t_moveArmStr = 0;
	float rewCenterRel = 0;
	bool isRewarding = false;
	bool isButtonReward = false;
	bool isBoundsSet = false;
	bool isZoneTriggered = false;
	bool isAllZonePassed = false;
	bool is_ekfNew = false;
	byte zoneIndByte = 255;
	float zoneRewarded = 0;
	float boundsRewarded[2];
	int occRewarded = 0;
	float lapN = 0;
	bool doArmMove = false;
	bool isArmExtended = false;
	const int armExtStps = 200;
	int armPos = 0;
	int armZone = 0;
	bool armStpOn = false;

	// Constructor
	Reward(int t_b)
	{
		this->dt_block = t_b;
		this->duration = 2000;
		this->durationByte = (byte)(duration / 10);

		// Reset reward stuff
		Reset();

		// Initialize feeder arm
		isArmExtended = true;
		armPos = armExtStps;
		RetractFeedArm();
	}

	// START REWARD
	bool StartRew(bool do_stop, bool is_button_reward)
	{
		// Set flag
		isButtonReward = is_button_reward;

		// Set to extend feeder arm 
		ExtendFeedArm();

		// Stop robot
		if (do_stop)
		{
			HardStop("StartRew");
			// Set hold time
			BlockMotorTill(dt_block, "REWARD.StartRew");
		}

		// Trigger reward tone on
		StorePacketData('a', 'r', durationByte);

		// Compute reward end time
		t_closeSol = millis() + duration;

		// Turn on reward LED
		analogWrite(pin_RewLED_R, round(rewLEDduty*0.75));
		analogWrite(pin_RewLED_C, rewLEDduty);
		// Open solenoid
		digitalWrite(pin_Rel_Rew, HIGH);

		// Print to LCD for manual rewards
		if (isButtonReward)
		{
			PrintLCD("REWARDING...");
		}
		else
		{
			char str[50];
			sprintf(str, "REWARDING FOR %dms...", duration);
			DebugFlow(str);
		}

		// Set flags
		isRewarding = true;

		// indicate reward in progress
		return isRewarding;

	}

	// END REWARD
	bool EndRew()
	{

		bool reward_finished = false;

		if (millis() > t_closeSol)
		{

			// Close solenoid
			digitalWrite(pin_Rel_Rew, LOW);

			// Turn off reward LED
			analogWrite(pin_RewLED_R, rewLEDmin);
			analogWrite(pin_RewLED_C, rewLEDmin);

			// Clear LCD
			if (isButtonReward)
			{
				ClearLCD();
			}
			else
			{
				DebugFlow("REWARD FINISHED");
			}

			// Set/reset flags
			reward_finished = true;
			Reset();
		}

		return reward_finished;

	}

	// Set reward duration
	void SetRewDur(byte zone_ind)
	{
		// Set duration
		duration = zoneRewDurs[zone_ind];
		durationByte = (byte)(duration / 10);

		// Save zone ind
		zoneIndByte = zone_ind;
	}

	// Set rew mode
	void SetRewMode(String mode_str, byte arg2)
	{
		// Store mode
		mode = mode_str;

		// Store info
		if (mode == "Free") {

			// Include all zones
			zoneMin = 0;
			zoneMax = zoneLng - 1;

			// Store threshold
			occThresh = (int)arg2 * 1000;

		}
		else if (mode == "Cue") {

			// Include one zone
			zoneMin = arg2;
			zoneMax = arg2;

			// Store threshold
			occThresh = 0;

		}
		else if (mode == "Now") {

			// Set duration
			SetRewDur(arg2);

		}
	}

	// Compute zone bounds
	bool CompZoneBounds(float now_pos, float rew_pos)
	{
		// Run only if bounds are not set
		if (!isBoundsSet)
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
			rewCenterRel = rew_pos + lapN*(140 * PI);

			// Compute bounds for each zone
			for (int i = zoneMin; i <= zoneMax; i++)
			{
				// Compute 5 deg bounds
				dist_center_cm = -1 * zoneLocs[i] * ((140 * PI) / 360);
				dist_start_cm = dist_center_cm - (2.5 * ((140 * PI) / 360));
				dist_end_cm = dist_center_cm + (2.5 * ((140 * PI) / 360));
				// Store in array
				zoneBounds[i][0] = rewCenterRel + dist_start_cm;
				zoneBounds[i][1] = rewCenterRel + dist_end_cm;
			}
			// Save bound min/max
			boundMin = zoneBounds[zoneMin][0];
			boundMax = zoneBounds[zoneMax][0];
			// Set flag
			isBoundsSet = true;
		}
		return isBoundsSet;
	}

	// Check bounds
	bool CheckZoneBounds(float now_pos)
	{
		// Run only if reward not already triggered
		if (!isZoneTriggered)
		{

			// Check if all bounds passed
			if (now_pos > boundMax + 5)
			{
				isAllZonePassed = true;
			}
			// Check if first bound reached
			else if (
				now_pos > boundMin
				)
			{

				// Check if rat in any bounds
				for (int i = zoneMin; i <= zoneMax; i++)
				{
					if (
						now_pos > zoneBounds[i][0] &&
						now_pos < zoneBounds[i][1]
						)
					{

						// Update timers
						t_lastZoneCheck = t_lastZoneCheck == 0 ? millis() : t_lastZoneCheck;
						t_nowZoneCheck = millis();

						// Store occupancy time
						zoneOccTim[i] += t_nowZoneCheck - t_lastZoneCheck;
						zoneOccCnt[i]++;
						t_lastZoneCheck = t_nowZoneCheck;

						// Check if occ thresh passed
						if (zoneOccTim[i] >= occThresh)
						{

							// Reward at this pos
							SetRewDur(i);

							// Store reward info for debugging
							zoneRewarded = zoneLocs[i] * -1;
							boundsRewarded[0] = zoneBounds[i][0];
							boundsRewarded[1] = zoneBounds[i][1];
							occRewarded = zoneOccTim[i];

							// Set flag
							isZoneTriggered = true;
						}
					}
				}
			}

			// Reset flag
			is_ekfNew = false;
		}
		return isZoneTriggered;
	}

	// Check if feeder arm should be moved
	void CheckFeedArm()
	{
		if (doArmMove)
		{
			// Save start time
			if (t_moveArmStr == 0)
				t_moveArmStr = millis();

			// Check if arm should be moved
			if (
				armPos != armZone &&
				millis() < t_moveArmStr + 1000)
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
				digitalWrite(pin_ED_SLP, LOW);

				// Set flag
				if (!isArmExtended && armPos > 0)
					isArmExtended = true;
				else if (isArmExtended && armPos == 0)
					isArmExtended = false;
				doArmMove = false;
				t_moveArmStr = 0;
			}
		}
		// Check if its time to retract arm
		else if (
			isArmExtended &&
			millis() > t_retractArm
			) {
			RetractFeedArm();
		}
		else if (digitalRead(pin_ED_SLP) == HIGH) {
			// Sleep motor
			digitalWrite(pin_ED_SLP, LOW);
		}

	}

	// Extend feeder arm
	void ExtendFeedArm()
	{
		if (!isArmExtended)
		{
			t_retractArm = millis() + dt_block;
			armZone = armExtStps;
			doArmMove = true;
		}
	}

	// Retract feeder arm
	void RetractFeedArm()
	{
		if (isArmExtended)
		{
			armZone = 0;
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
		if (!armStpOn)
		{

			// Extend arm
			if (armPos < armZone)
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

			// Set flag
			armStpOn = true;
		}
		// Unstep motor
		else
		{
			// Set step low
			digitalWrite(pin_ED_STP, LOW);

			// Set flag
			armStpOn = false;
		}
	}

	// Reset
	void Reset()
	{
		// Log zone info
		if (mode == "Free" || mode == "Cue")
		{
			String str1 = "ZONE OCC:";
			String str2 = "ZONE CNT:";
			for (int i = zoneMin; i <= zoneMax; i++)
			{
				char chr1[50];
				sprintf(chr1, " z%d=%dms", i, zoneOccTim[i]);
				str1 += chr1;
				char chr2[50];
				sprintf(chr2, " z%d=%d", i, zoneOccCnt[i]);
				str2 += chr2;
			}
			DebugFlow(str1);
			DebugFlow(str2);
		}

		// Reset flags
		mode = "None";
		isRewarding = false;
		isButtonReward = false;
		isBoundsSet = false;
		isZoneTriggered = false;
		isAllZonePassed = false;
		is_ekfNew = false;

		// Reset occ time
		for (int i = 0; i < zoneLng; i++) {
			zoneOccTim[i] = 0;
			zoneOccCnt[i] = 0;
		}
		t_nowZoneCheck = 0;
		t_lastZoneCheck = 0;
	}
};
// Initialize object
Reward reward(dt_blockRew);

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

	// Make sure certain pins low
	digitalWrite(pin_Rel_Rew, LOW);
	digitalWrite(pin_Rel_EtOH, LOW);
	digitalWrite(pin_PwrOff, LOW);

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

	// SETUP BIG EASY DRIVER

	// Set to 1/2 step mode
	digitalWrite(pin_ED_MS1, HIGH);
	digitalWrite(pin_ED_MS2, LOW);
	digitalWrite(pin_ED_MS3, LOW);

	// Start BigEasyDriver in sleep
	digitalWrite(pin_ED_SLP, LOW);

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


	// Initialize c2r stuff
	for (int i = 0; i < c2r_idLng; i++)
	{
		c2r_packLast[i] = 0;
		c2r_cntRepeat[i] = 0;
	}

	// Initialize r2c stuff
	for (int i = 0; i < r2c_idLng; i++)
	{
		r2c_packLast[i] = 0;
		r2c_datLast[i] = 0;
	}

	// Initialize r2a stuff
	for (int i = 0; i < r2a_idLng; i++)
	{
		r2a_doRcvdCheck[i] = false;
		r2a_sendTim[i] = 0;
		r2a_packLast[i] = 0;
		r2a_datLast[i] = 0;
		r2a_resendCnt[i] = 0;
	}

	// Initialize bat volt array
	for (int i = 0; i < 100; i++)
	{
		batVoltArr[i] = 0;
	}

	// DEFINE EXTERNAL INTERUPTS

	// IR prox right
	attachInterrupt(digitalPinToInterrupt(pin_IRprox_Rt), Interupt_IRprox_Halt, FALLING);
	// IR prox left
	attachInterrupt(digitalPinToInterrupt(pin_IRprox_Lft), Interupt_IRprox_Halt, FALLING);
	// IR detector
	uint32_t t_check_ir = millis() + 500;
	while (digitalRead(pin_IRdetect) == HIGH && t_check_ir < millis());
	if (digitalRead(pin_IRdetect) == LOW)
		attachInterrupt(digitalPinToInterrupt(pin_IRdetect), Interupt_IR_Detect, HIGH);

}


// ---------MAIN LOOP---------
void loop() {

#pragma region //--- DEBUG ---

	// Store loop time
	t_loopMain = millis();
	cnt_loop++;

	// Store every 10 thousand loops
	if (cnt_loop % 10000 == 0) {
		// Print loop count and dt
		static uint32_t t_loop_last = millis();
		char chr[50];
		sprintf(chr, "LOOP %d: dt=%lu", cnt_loop, millis() - t_loop_last);
		DebugFlow(chr);
		// Store loop time
		t_loop_last = millis();
	}

	// Print debug
	if (doPrint)
	{
		PrintDebug();
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
		t_doIRhardStop = false;
		intrpt_doLogIR = false;

		// Print ad board status
		char chr[20];
		sprintf(chr, "BOARD R STATUS: %04x", ad_R.getStatus());
		DebugFlow(chr);
		sprintf(chr, "BOARD F STATUS: %04x", ad_F.getStatus());
		DebugFlow(chr);

		// Blink to show setup done
		SetupBlink();

		fc_isFirstPass = false;
		DebugFlow("RESET");

		// Check if ir sensor needs to be disabled
		if (digitalRead(pin_IRdetect) == HIGH)
			DebugFlow("!!IR SENSOR DISABLED!!");


		/*
		// TEST
		float now_pos = 348 + (140 * PI);
		float rew_pos = 92;
		for (int i = 0; i < 100; i++)
		{
			reward.is_bound_set = false;
			reward.CompZoneBounds(now_pos, rew_pos);
			now_pos = now_pos + 10 + (140 * PI);
		}
		*/

	}

#pragma endregion

#pragma region //--- PARSE SERIAL ---
	c2r_isNew = false; // reset before next loop
	if (Serial1.available() > 0)
	{
		c2r_isNew = ParseSerial();
	}
#pragma endregion

#pragma region //--- SEND SERIAL DATA ---

	// Prioritize packet
	if (doPackSend)
	{
		SendPacketData();
	}
	// Send log
	else if (fc_doLogSend)
	{
		SendLogData();
	}
#pragma endregion

#pragma region //--- SYSTEM TESTS ---

	// Run position debugging
	if (do_posDebug)
	{
		static float rat_rob_dist;
		if (millis() % 100 == 0)
		{
			rat_rob_dist = ekfRatPos - ekfRobPos;
			// Plot pos
			/*
			{@Plot.Pos.ratPixy.Green pos_ratPixy.posNow}{@Plot.Pos.ratVT.Blue pos_ratVT.posNow}{@Plot.Pos.ratEKF.Black ekfRatPos}{@Plot.Pos.robVT.Orange pos_robVT.posNow}{@Plot.Pos.robEKF.Red ekfRobPos}
			*/
			millis();
			// Turn on rew led when near setpoint
			if (errorDefault > -0.5 && errorDefault < 0.5) { analogWrite(pin_RewLED_C, 50); }
			else { analogWrite(pin_RewLED_C, 0); }
			// Print to LCD
			analogWrite(pin_Disp_LED, 10);
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
			if (new_speed >= 0)
			{
				speed_steps = new_speed*cm2stp;
				ad_R.run(FWD, speed_steps);
				ad_F.run(FWD, speed_steps*frontMoterScale);
			}
			// Print values
			/*
			{pid.cal_isCalFinished}{"ERROR"}{pid.cal_errNow}{pid.cal_errArr[0]}{pid.cal_errArr[1]}{pid.cal_errArr[2]}{pid.cal_errArr[3]}{"PERIOD"}{pid.cal_PcNow}{pid.cal_cntPcArr[0]}{pid.cal_PcArr[0]}{pid.cal_cntPcArr[1]}{pid.cal_PcArr[1]}{pid.cal_cntPcArr[2]}{pid.cal_PcArr[2]}{pid.cal_cntPcArr[3]}{pid.cal_PcArr[3]}{pid.cal_PcAll}
			*/
			millis();
			// Plot vel
			/*
				{@Plot.Vel.ratVelCal.Black pid.cal_ekfRatVel} {@Plot.Vel.robVelEKF.Red ekfRobVel}
			*/
			millis();
			// Reset flag
			pid.cal_isPidUpdated = false;
		}
	}

	if (c2r_msg_id == 'T' && c2r_isNew)
	{
		// Set run pid calibration flag
		if (msg_testCmd[0] == 2)
		{
			do_pidCalibration = true;
		}

		// Update Hault Error test run speed
		else if (msg_testCmd[0] == 3)
		{
			float new_speed = float(msg_testCmd[1]);
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
#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (c2r_msg_id == 'S' && c2r_isNew)
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
			StorePacketData('a', 's', 0);
			DebugFlow("NO SOUND");
		}
		else if (msg_setupCmd[1] == 1)
		{
			// Use white noise only
			StorePacketData('a', 's', 1);
			DebugFlow("DONT DO TONE");
		}
		else
		{
			// Use white and reward noise
			StorePacketData('a', 's', 2);
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
	if (c2r_msg_id == 'Q' && c2r_isNew)
	{
		fc_doQuit = true;
		t_quitCmd = millis() + 1000;

		// Tell ard to quit
		StorePacketData('a', 'q');
		DebugFlow("DO QUIT");

		// Hold all motor control
		fc_isHalted = true;
		SetMotorControl("None", "MsgQ");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (
		fc_doQuit && millis() > t_quitCmd &&
		!CheckArdResend() &&
		!doPackSend)
	{
		// Quit session
		DebugFlow("QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (c2r_msg_id == 'M' && c2r_isNew)
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
					sprintf(str, "MOVING: from=%0.2fcm to=%0.2fcm by=%0.2fcm",
						targ_moveTo.posStart, targ_moveTo.offsetTarget, targ_moveTo.targDist);
					DebugFlow(str);
				}
				// Reset motor cotrol if run fails
				else SetMotorControl("Open", "MsgM");
			}
			else if (!fc_isEKFReady)
				DebugFlow("NO MOVE BECAUSE EKF NOT READY");
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
		}

		// Check if target reached or move aborted
		if (targ_moveTo.isTargReached || targ_moveTo.abortMove)
		{
			// Hard stop
			HardStop("MsgM");

			// Set motor cotrol to None
			SetMotorControl("None", "MsgM");

			// Reset flags
			fc_doMove = false;
			targ_moveTo.Reset();

			char str[50];
			if (!targ_moveTo.abortMove)
			{
				// Tell CS movement is done
				StorePacketData('c', 'D', 255, c2r_packLast[ID_Ind('M', c2r_id, c2r_idLng)]);

				// Print success message
				sprintf(str, "FINISHED MOVE: to=%0.2fcm within=%0.2fcm",
					targ_moveTo.offsetTarget, targ_moveTo.GetError(ekfRobPos));
				DebugFlow(str);
			}
			else
			{
				// Print failure message
				sprintf(str, "!!ABORTED MOVE: to=%0.2fcm within=%0.2fcm!!",
					targ_moveTo.offsetTarget, targ_moveTo.GetError(ekfRobPos));
				DebugFlow(str);
			}
		}
	}
#pragma endregion

#pragma region //--- (R/C) RUN REWARD ---
	if (c2r_msg_id == 'R' && c2r_isNew)
	{
		// Set mode
		reward.SetRewMode("Free", msg_rewDelay);

		// Reward zone
		char str[100];
		sprintf(str, "REWARD FREE: pos=%0.2fcm occ_thresh=%dms",
			msg_rewPos, reward.occThresh);
		DebugFlow(str);
		fc_doRew = true;
	}

	if (c2r_msg_id == 'C' && c2r_isNew)
	{

		// Cued reward
		if (msg_rewPos > 0)
		{
			// Set mode
			reward.SetRewMode("Cue", msg_rewZoneInd);

			// Cued reward
			char str[100];
			sprintf(str, "REWARD CUED: pos=%0.2fcm occ_thresh=%ldms",
				msg_rewPos, reward.occThresh);
			DebugFlow(str);
			fc_doRew = true;
		}

		// Imediate reward
		else {

			// Set mode
			reward.SetRewMode("Now", msg_rewZoneInd);
			DebugFlow("REWARD NOW");

			// Start reward
			fc_isRewarding = reward.StartRew(true, false);

		}

	}

	// CHECK REWARD BOUNDS
	if (fc_doRew)
	{
		// If not rewarding 
		if (!reward.isRewarding)
		{
			// Compute reward bounds
			if (!reward.isBoundsSet)
			{
				reward.CompZoneBounds(ekfRatPos, msg_rewPos);
				// Print message
				char str[100];
				sprintf(str, "SET REWARD ZONE: center=%0.2fcm from=%0.2fcm to=%0.2fcm",
					reward.rewCenterRel, reward.boundMin, reward.boundMax);
				DebugFlow(str);
			}
			else if (!reward.isZoneTriggered)
			{
				if (reward.CheckZoneBounds(ekfRatPos))
				{
					// Start reward
					fc_isRewarding = reward.StartRew(true, false);
					// Print message
					char str[50];
					sprintf(str, "REWARDED ZONE: occ=%dms zone=%0.2fcm from=%0.2fcm to=%0.2fcm",
						reward.occRewarded, reward.zoneRewarded, reward.boundsRewarded[0], reward.boundsRewarded[1]);
					DebugFlow(str);
				}
			}
			// Check if rat passed all bounds
			if (
				reward.isAllZonePassed &&
				!reward.isZoneTriggered
				)
			{
				// Print reward missed
				char str[50];
				sprintf(str, "!!REWARD MISSED!!: rat=%0.2fcm bound_max=%0.2fcm",
					ekfRatPos, reward.boundMax);
				DebugFlow(str);

				// Reset flags
				reward.Reset();
				fc_doRew = false;
			}
		}
	}

#pragma endregion

#pragma region //--- (H) HALT ROBOT STATUS ---
	if (c2r_msg_id == 'H' && c2r_isNew)
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
	if (c2r_msg_id == 'B' && c2r_isNew)
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
				DebugFlow("SET BULLDOZE ON");
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
				DebugFlow("SET BULLDOZE OFF");
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
	if (c2r_msg_id == 'I' && c2r_isNew)
	{
		if (fc_isRatIn)
		{
			// Reset all vt data
			pos_ratVT.SetPos(0, 0);
			pos_ratPixy.SetPos(0, 0);
			pos_robVT.SetPos(0, 0);

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
	if (c2r_msg_id == 'V' && c2r_isNew)
	{
		fc_doStreamCheck = true;
	}

	// Check for streaming
	if (fc_doStreamCheck && fc_isStreaming)
	{
		StorePacketData('c', 'D', 255, c2r_packLast[ID_Ind('V', c2r_id, c2r_idLng)]);
		fc_doStreamCheck = false;
		DebugFlow("STREAMING CONFIRMED");
	}
#pragma endregion

#pragma region //--- (Y) DONE RECIEVED ---
	if (c2r_msg_id == 'Y' && c2r_isNew)
	{
		fc_doCheckDoneRcvd = false;
		DebugFlow("CS RESIEVED 'DONE'");
	}
	// Request done recieved confirmation
	if (fc_doCheckDoneRcvd && millis() > t_resendDone)
	{
		// Resend done confirmation
		StorePacketData('c', 'D', 255, r2c_packLast[ID_Ind('D', r2c_id, r2c_idLng)]);
		// Send again after 1 sec
		t_resendDone = millis() + 1000;
		DebugFlow("RESENT 'D'");
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (c2r_msg_id == 'P' && c2r_isNew)
	{
		// Update vt pos data
		if (msg_vtEnt == 0)
		{
			pos_ratVT.UpdatePos(msg_vtCM[msg_vtEnt], t_msg_vtTS[msg_vtEnt]);
		}
		else
		{
			pos_robVT.UpdatePos(msg_vtCM[msg_vtEnt], t_msg_vtTS[msg_vtEnt]);
		}
	}
#pragma endregion

#pragma region //--- (L) SEND LOG ---
	if (c2r_msg_id == 'L' && c2r_isNew)
	{
		// Stop logging
		if (doDB_Log)
			doDB_Log = false;

		// Send log data
		if (!fc_doLogResend)
		{
			cnt_logSend++;
		}
		else DebugFlow("!!RESEND LAST LOG PACKET!!");

		// Check if end of list reached
		if (cnt_logSend <= cnt_logStore)
		{
			fc_doLogSend = true;
			fc_isSendingLog = true;
		}
		// End reached
		else
		{
			// Send confirm done
			StorePacketData('c', 'D', 255, c2r_packLast[ID_Ind('L', c2r_id, c2r_idLng)]);
			DebugFlow("LOG SEND COMPLETE");
			fc_doLogSend = false;
			fc_isSendingLog = false;
		}
	}
#pragma endregion

#pragma region //--- BUTTON/INTERUPT TRIGGERED ---

	// Check for button input
	CheckButtons();

	// Open/close rew solonoid
	if (btn_doRewSolStateChange)
	{
		OpenCloseRewSolenoid();
		btn_doRewSolStateChange = false;
	}

	// Open/close etoh solonoid
	if (btn_doEtOHSolStateChange)
	{
		OpenCloseEtOHSolenoid();
		btn_doEtOHSolStateChange = false;
	}

	// Button triggered reward
	if (btn_doRew)
	{
		if (!reward.isRewarding)
		{
			fc_isRewarding = reward.StartRew(false, true);
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
	if (t_doIRhardStop)
	{
		Function_IRprox_Halt();
		t_doIRhardStop = false;
	}

	// Log new ir events
	if (intrpt_doLogIR)
	{
		// Log first sync event
		static bool first_sync = true;
		if (first_sync) {
			DebugFlow("SET SYNC TIME", t_sync);
			first_sync = false;
		}

		// Log event if streaming started
		if (fc_isStreaming)
			DebugIRSync("ir sync event", t_irSyncLast);

		// Reset flag
		intrpt_doLogIR = false;
	}


#pragma endregion

#pragma region //--- RUN TRACKING ---

	// UPDATE PIXY
	UpdatePixyPos();

	// CHECK IF RAT POS FRAMES DROPPING
	CheckSampDT();

	// INITIALIZE RAT AHEAD
	InitializeTracking();

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

	// End any ongoing reward
	if (fc_isRewarding)
	{
		if (reward.EndRew())
		{
			fc_doRew = false;
			fc_isRewarding = false;

			// Tell CS what zone was rewarded
			if (reward.mode != "Now")
				StorePacketData('c', 'Z', reward.zoneIndByte, 0);
		}
	}

	// Check if feeder arm should be moved
	reward.CheckFeedArm();

	// Check if EtOH should be dispensed
	CheckEtOH();

	// Check voltage sensor
	CheckBattery();

	// Check if ard data should be resent
	CheckArdResend();


#pragma endregion

}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{
	// Local vars
	byte buff = 0;
	char head = ' ';
	bool is_new_c2a = false;

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(c2r_head, a2r_head);
	if (buff == 255)
	{
		return is_new_c2a = false;
	}
	// store header
	u.f = 0.0f;
	u.b[0] = buff;
	head = u.c[0];

	// Determine sender;
	if (head == c2r_head)
	{
		is_new_c2a = ParseC2R();
	}
	else if (head == a2r_head)
	{
		ParseA2R();
	}

	// Return if this is new c2r data
	return is_new_c2a;

}

// PARSE CS MESSAGE
bool ParseC2R()
{
	// Local vars
	bool pass = false;
	byte buff = 0;
	uint16_t pack = 0;
	char foot = ' ';
	c2r_msg_id == ' ';

	// get id
	u.f = 0.0f;
	u.b[0] = WaitBuffRead();
	c2r_msg_id = u.c[0];

	// Get system test data
	if (c2r_msg_id == 'T')
	{

		// Get test id
		msg_testCmd[0] = WaitBuffRead();

		// Get test argument
		msg_testCmd[1] = WaitBuffRead();

	}

	// Get setup data
	if (c2r_msg_id == 'S')
	{

		// Get session comand
		msg_setupCmd[0] = WaitBuffRead();

		// Get tone condition
		msg_setupCmd[1] = WaitBuffRead();

	}

	// Get MoveTo data
	if (c2r_msg_id == 'M')
	{

		// Reset buffer
		u.f = 0.0f;

		// Get move posm
		u.b[0] = WaitBuffRead();
		u.b[1] = WaitBuffRead();
		u.b[2] = WaitBuffRead();
		u.b[3] = WaitBuffRead();
		msg_moveToTarg = u.f;

	}

	// Get Reward data
	if (c2r_msg_id == 'R')
	{
		// Reset buffer
		u.f = 0.0f;

		// Get stop pos
		u.b[0] = WaitBuffRead();
		u.b[1] = WaitBuffRead();
		u.b[2] = WaitBuffRead();
		u.b[3] = WaitBuffRead();
		msg_rewPos = u.f;

		// Get reward diration 
		msg_rewDelay = WaitBuffRead();
	}

	// Get Cue data
	if (c2r_msg_id == 'C')
	{
		// Reset buffer
		u.f = 0.0f;

		// Get stop pos
		u.b[0] = WaitBuffRead();
		u.b[1] = WaitBuffRead();
		u.b[2] = WaitBuffRead();
		u.b[3] = WaitBuffRead();
		msg_rewPos = u.f;

		// Get reward diration 
		msg_rewZoneInd = WaitBuffRead();
	}

	// Get halt robot data
	if (c2r_msg_id == 'H')
	{
		// Get halt bool
		byte do_halt = WaitBuffRead();
		fc_doHalt = do_halt == 1 ? true : false;
	}

	// Get bulldoze rat data
	if (c2r_msg_id == 'B')
	{
		// Get delay in sec
		msg_bullDel = WaitBuffRead();
		// Get speed
		msg_bullSpeed = WaitBuffRead();
	}

	// Get rat in/out
	if (c2r_msg_id == 'I')
	{
		// Get session comand
		byte rat_in = WaitBuffRead();
		fc_isRatIn = rat_in == 1 ? true : false;
	}

	// Get log request data
	if (c2r_msg_id == 'L')
	{
		// Get session comand
		byte resend = WaitBuffRead();
		fc_doLogResend = resend == 1 ? true : false;

	}

	// Get VT data
	else if (c2r_msg_id == 'P')
	{
		// Get Ent
		msg_vtEnt = WaitBuffRead();
		// Get TS
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = WaitBuffRead();
		}
		t_msg_vtTS[msg_vtEnt] = u.i32;
		// Get pos cm
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = WaitBuffRead();
		}
		msg_vtCM[msg_vtEnt] = u.f;
	}

	// Get packet num
	u.f = 0.0f;
	u.b[0] = WaitBuffRead();
	u.b[1] = WaitBuffRead();
	pack = u.i16[0];

	// Check for footer
	u.f = 0.0f;
	u.b[0] = WaitBuffRead();
	foot = u.c[0];

	// Footer missing
	if (foot != c2r_foot) {
		// mesage will be dumped
		pass = false;
	}
	// Process complete mesage
	else
	{
		// Update recive time
		t_rcvdLast = t_rcvd;
		t_rcvd = millis();

		// CHECK FOR STREAMING STARTED
		if (c2r_msg_id == 'P' && !fc_isStreaming)
		{
			// Signal streaming started
			fc_isStreaming = true;

			// send sync start cmd to ard
			StorePacketData('a', 't');
		}

		// Process packet number
		pass = CheckPack(c2r_msg_id, pack);

	}

	// Return final status
	return pass;

}

// PARSE CHEETAH DUE MESSAGE
void ParseA2R()
{
	// Local vars
	byte buff = 0;
	char foot = ' ';
	char id = ' ';
	uint16_t pack = 0;
	int id_ind = 0;

	// get id
	u.f = 0.0f;
	u.b[0] = WaitBuffRead();
	id = u.c[0];

	// Get packet num
	u.f = 0.0f;
	u.b[0] = WaitBuffRead();
	u.b[1] = WaitBuffRead();
	pack = u.i16[0];

	// Check for footer
	u.f = 0.0f;
	u.b[0] = WaitBuffRead();
	foot = u.c[0];

	// Update sent history
	if (foot == a2r_foot) {
		id_ind = ID_Ind(id, r2a_id, r2a_idLng);
		if (id_ind != -1)
		{
			// Reset flags
			r2a_doRcvdCheck[id_ind] = false;

			// Store revieved pack details
			DebugRcvd('a', id, pack);
		}
	}
}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead()
{
	return WaitBuffRead(' ', ' ');
}
byte WaitBuffRead(char chr1)
{
	return WaitBuffRead(chr1, chr1);
}
byte WaitBuffRead(char chr1, char chr2)
{
	// Local vars
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	byte buff = 0;
	bool pass = false;

	// Convert char to byte
	byte match1 = chr1 == ' ' ? 255 : byte(chr1);
	byte match2 = chr2 == ' ' ? 255 : byte(chr2);

	// Check for overflow
	if (Serial1.available() < SERIAL_BUFFER_SIZE - 1)
	{
		// Wait for at least 1 byte
		while (Serial1.available() < 1 &&
			millis() < t_timeout);

		// Store next byte
		if (match1 == 255)
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
			} while (
				buff != match1  &&
				buff != match2  &&
				millis() < t_timeout &&
				Serial1.available() < SERIAL_BUFFER_SIZE - 1);
			// check match was found
			if (buff == match1 || buff == match2)
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
		if (millis() > t_timeout)
		{
			cnt_timeoutEvt++;
		}

		// Set buff to 255 ((byte)-1) if !pass
		buff = 255;
	}

	// Store time
	t_loopRead = millis() - (t_timeout - timeout);

	// Return buffer
	return buff;
}

// CHECK AND PROCESS RCVD PACKET NUMBER
bool CheckPack(char id, uint16_t pack)
{
	// Local vars
	static uint16_t pack_last = 0;
	bool pass = false;
	int pack_diff = 0;
	int pack_tot = 0;
	int dropped_packs = 0;
	bool do_use_pack = false;
	int id_ind = 0;
	
	// Get id ind
	id_ind = ID_Ind(id, c2r_id, c2r_idLng);

	// SEND AND PRINT PACKET CONFIRMATON
	if (id != 'P')
	{
		// Store revieved pack details
		DebugRcvd('c', id, pack);

		// Send back packet confirmation
		if (id != 'Y')
		{
			// Send revieved pack
			StorePacketData('c', id, 255, pack);
		}
	}

	// CHECK FOR DROPPED PACKETS
	pack_diff = (int)(pack - pack_last);

	if (pack_diff > 0)
	{
		// Save packet and set to process
		pack_last = pack;

		// Check for dropped packet
		pack_tot += pack_diff;
		dropped_packs = pack_diff - 1;
		// print dropped packs
		if (dropped_packs > 0)
		{
			cnt_droppedPacks += dropped_packs;
			DebugDropped(dropped_packs, cnt_droppedPacks, pack_tot);
		}

	}

	// CHECK IF NEW PACK
	if (c2r_packLast[id_ind] != pack)
	{
		// Update last packet
		c2r_packLast[id_ind] = pack;

		// Reset count
		c2r_cntRepeat[id_ind] = 0;

		// Pack should be used
		do_use_pack = true;
	}
	// PROCESS RESENT PACKETS
	else
	{
		// Do not reuse pack
		do_use_pack = false;

		// Itterate count
		c2r_cntRepeat[id_ind]++;

		// Print dropped packets
		DebugResent(id, pack, c2r_cntRepeat[id_ind]);

	}

	// Packet not found in hist so process
	return do_use_pack;
}

// STORE PACKET DATA TO BE SENT
void StorePacketData(char targ, char id)
{
	bool conf = targ == 'a' ? true : false;
	StorePacketData(targ, id, 255, 0, conf);
}
void StorePacketData(char targ, char id, byte d1)
{
	bool conf = targ == 'a' ? true : false;
	StorePacketData(targ, id, d1, 0, conf);
}
void StorePacketData(char targ, char id, byte d1, uint16_t pack)
{
	bool conf = targ == 'a' ? true : false;
	StorePacketData(targ, id, d1, pack, conf);
}
void StorePacketData(char targ, char id, byte d1, uint16_t pack, bool conf)
{
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: head, id, data, packet num, footer
	*/

	// Local vars
	int queue_ind = 0;
	char head = ' ';
	char foot = ' ';
	int id_ind = 0;

	// Store r2a data
	if (targ == 'a')
	{
		// Shift data back so ard msg is first in queue
		for (int i = 0; i < r2_lngR - 1; i++)
		{
			for (int j = 0; j < r2_lngC; j++)
			{
				r2_queue[i][j] = r2_queue[i + 1][j];
			}
		}

		// Set queue ind to front
		queue_ind = r2_lngR - 1;

		// Itterate r2a packet number
		if (pack == 0)
		{
			r2a_packCnt++;
			pack = r2a_packCnt;
		}

		// Store header and footer
		head = r2a_head;
		foot = r2a_foot;

		// Update last packet arrays
		id_ind = ID_Ind(id, r2a_id, r2a_idLng);
		r2a_datLast[id_ind] = d1;
		r2a_packLast[id_ind] = pack;
	}

	// Store r2c data
	else if (targ == 'c')
	{
		// Set queue ind to back
		queue_ind = sendQueueInd;

		// Store header and footer
		head = r2c_head;
		foot = r2c_foot;

		// Update last packet arrays
		id_ind = ID_Ind(id, r2c_id, r2c_idLng);
		r2c_datLast[id_ind] = d1;
		r2c_packLast[id_ind] = pack;

		// Check for done id
		if (id == 'D')
		{
			// Set to check for done recieved
			fc_doCheckDoneRcvd = true;
			// Send done recieved request after 1 sec
			t_resendDone = millis() + 1000;
		}
	}

	// Update queue index
	sendQueueInd--;

	// Store reciever id in last col
	r2_queue[queue_ind][r2_lngC - 1] = targ;

	// Store header
	u.f = 0.0f;
	u.c[0] = head;
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
	// Store confirm request
	r2_queue[queue_ind][5] = conf ? 1 : 0;
	// Store footer
	u.f = 0.0f;
	u.c[0] = foot;
	r2_queue[queue_ind][6] = u.b[0];

	// Set to send
	doPackSend = true;

}

// SEND SERIAL PACKET DATA
void SendPacketData()
{
	// Local vars
	const int msg_size = r2_lngC - 1;
	static byte msg[msg_size];
	char targ = ' ';
	bool do_send = false;
	int buff_tx = 0;
	int buff_rx = 0;
	char id = ' ';
	byte dat = 0;
	bool conf = 0;
	uint16_t pack = 0;

	// Get total data in buffers
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	buff_rx = Serial1.available();

	// save msg id and reviever id
	id = r2_queue[r2_lngR - 1][1];
	targ = r2_queue[r2_lngR - 1][r2_lngC - 1];

	// Move next in queue to temp msg array
	for (int j = 0; j < msg_size; j++)
	{
		msg[j] = r2_queue[r2_lngR - 1][j];
	}

	// Send r2a sync time or rew tone immediately
	if ((id == 'r' ||
		id == 't'))
	{
		do_send = true;
	}
	// Avoid overlap between sent or rcvd events
	else if (millis() > t_sent + dt_sendSent &&
		millis() > t_rcvd + dt_sendRcvd)
	{
		do_send = true;
	}

	// Send if conditions met
	if (
		do_send &&
		buff_tx == 0 &&
		buff_rx == 0
		)
	{
		// Send
		Serial1.write(msg, msg_size);

		// Update send time 
		t_sent = millis();

		// Set flags for r2a comm
		if (targ == 'a')
		{
			int id_ind = ID_Ind(id, r2a_id, r2a_idLng);
			r2a_sendTim[id_ind] = t_sent;
			r2a_doRcvdCheck[id_ind] = true;
		}

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
			doPackSend = false;
		}

		// Print
		bool do_print = ((doPrint_r2c && targ == 'c') ||
			(doPrint_r2a && targ == 'a')) &&
			(doDB_PrintConsole || doDB_PrintLCD);
		bool do_log = ((doLog_r2c && targ == 'c') ||
			(doLog_r2a && targ == 'a')) &&
			doDB_Log;
		if (do_print || do_log)
		{
			// Store dat
			dat = msg[2];
			// Store packet number
			u.f = 0.0f;
			u.b[0] = msg[3];
			u.b[1] = msg[4];
			pack = u.i16[0];
			// Store conf 
			conf = msg[5] == 1 ? true : false;

			// Make string
			char str[50];
			sprintf(str, "sent_r2%c: id=%c dat=%d pack=%d conf=%s", targ, id, dat, pack, conf ? "true" : "false");

			// Store
			if (do_print)
				StoreDBPrintStr(str, t_sent);
			if (do_log)
				StoreDBLogStr(str, t_sent);

		}

	}
}

// SEND SERIAL LOG DATA
void SendLogData()
{
	// Local vars
	String msg_str = " ";
	char msg_char[100];
	byte str_size = 0;
	byte msg_size = 0;
	byte msg[100];
	bool do_send = false;
	int buff_tx = 0;
	int buff_rx = 0;

	// Get total data in buffers
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	buff_rx = Serial1.available();


	// Avoid overlap between sent or rcvd events
	if (millis() > t_sent + dt_logSent &&
		millis() > t_rcvd + dt_logRcvd)
	{
		do_send = true;
	}

	// Send if conditions met
	if (
		do_send &&
		buff_tx == 0 &&
		buff_rx == 0
		)
	{
		// Update send time 
		t_sent = millis();

		// Pull out string
		msg_str = logList[cnt_logSend - 1];

		// Get message size
		str_size = msg_str.length();

		// Convert to char array
		msg_str.toCharArray(msg_char, 100);

		// Load byte array
		msg[0] = r2c_head;
		u.f = 0.0f;
		u.c[0] = 'U';
		msg[1] = u.b[0];
		msg[2] = str_size;
		for (int i = 0; i < str_size; i++)
			msg[i + 3] = msg_char[i];
		msg[str_size + 3] = r2c_foot;

		// Compute message size
		msg_size = str_size + 4;

		// Send
		Serial1.write(msg, msg_size);

		// Print
		if (doPrint_log &&
			(doDB_PrintConsole || doDB_PrintLCD))
		{
			char chr[100];
			String str;

			// Store
			sprintf(chr, "sent log: sent=%d stored=%d chksum=%d: ", cnt_logSend, cnt_logStore, str_size);
			str = chr;
			StoreDBPrintStr(str + "\"" + msg_str + "\"", t_sent);
		}

		// Reset flag
		fc_doLogSend = false;

	}
}

// CHECK IF ARD TO ARD PACKET SHOULD BE RESENT
bool CheckArdResend()
{
	// Local vars
	bool do_pack_resend = false;

	// Loop through and check for data needing to be resent
	for (int i = 0; i < r2a_idLng; i++)
	{
		if (
			r2a_doRcvdCheck[i] &&
			millis() > r2a_sendTim[i] + r2a_resendDel
			)
		{
			if (r2a_resendCnt[i] < r2a_resendMax) {

				// Resend data
				StorePacketData('a', r2a_id[i], r2a_datLast[i], r2a_packLast[i]);

				// Update count
				r2a_resendCnt[i]++;
				DebugResent(r2a_id[i], r2a_packLast[i], r2a_resendCnt[i]);

				// Set flags
				do_pack_resend = true;
				r2a_doRcvdCheck[i] = false;
			}
			// Com likely down
			else {
				r2a_doRcvdCheck[i] = false;
			}
		}
	}

	// Return
	return do_pack_resend;
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
	bool pass = false;

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
			// Can still move robot if rat not in
			if (
				set_to == "MoveTo" &&
				!fc_isRatIn
				)
			{
				fc_motorControl = set_to;
			}

			// CheckBlockTimElapsed can unblock if tracking setup
			else if (
				fc_isTrackingEnabled &&
				(called_from == "CheckBlockTimElapsed")
				)
			{
				fc_motorControl = set_to;
			}

			// InitializeTracking can always unset "None"
			if (called_from == "InitializeTracking")
			{
				fc_motorControl = set_to;
			}

		}

		// "MoveTo" can only be set to "Open" or "None"
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

	// Return true if set to input
	if (set_to == fc_motorControl)
	{
		pass = true;
	}

	// Store current controller
	DebugMotorControl(pass, set_to, called_from);

	return pass;
}

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(int dt, String called_from)
{
	// Set blocking and time
	fc_isBlockingTill = true;

	// Update time to hold till
	dt_blockMotor = millis() + dt;

	// Remove all motor controll
	SetMotorControl("None", "BlockMotorTill");

	// Print blocking finished
	DebugMotorBocking("blocking motor for: dt=", dt, called_from);
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
		if (millis() > dt_blockMotor || passed_feeder)
		{
			// Print blocking finished
			DebugMotorBocking("finished blocking motor: tim=", millis(), "CheckBlockTimElapsed");

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

		// Print process
		DebugFlow("INITIALIZING RAT TRACKING");

		// Check that rat pos > robot pos
		n_laps = pos_ratVT.posNow > pos_robVT.posNow ? 0 : 1;
		if (n_laps > 0)
			DebugFlow("SET RAT POS AHEAD");
		// set n_laps for rat vt data
		pos_ratVT.SetPos(pos_ratVT.posNow, n_laps);
		pos_ratPixy.SetPos(pos_ratPixy.posNow, n_laps);

		// Check that results make sense
		cm_diff = pos_ratVT.posNow - pos_robVT.posNow;
		cm_dist = min((140 * PI) - abs(cm_diff), abs(cm_diff));

		// Rat should be no more than 90 deg rom rob
		if (cm_dist > ((140 * PI) / 4))
		{
			// Will have to run again with new samples
			pos_ratVT.SetPos(0, 0);
			pos_ratPixy.SetPos(0, 0);
			pos_robVT.SetPos(0, 0);
			DebugFlow("RAT POS WRONG SO POS DATA RESET");
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
		int dt_max_frame = 100;
		int dt_vt = 0;
		int dt_pixy = 0;

		// Compute dt
		dt_vt = millis() - pos_ratVT.t_msNow;
		dt_pixy = millis() - pos_ratPixy.t_msNow;

		// Check pixy
		if (
			dt_pixy >= dt_max_frame &&
			dt_vt < dt_pixy
			)
		{
			// Use VT for Pixy data
			pos_ratPixy.SetDat(pos_ratVT.posAbs, pos_ratVT.t_msNow);
		}
		// Check VT 
		else if (
			dt_vt >= dt_max_frame &&
			dt_pixy < dt_vt
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
	uint32_t t_px_ts = 0;
	double pixy_pos_y = 0;

	// Get new blocks
	uint16_t blocks = pixy.getBlocks();

	// Check for new data
	if (blocks)
	{

		// Save time stamp
		t_px_ts = millis();

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
		pos_ratPixy.UpdatePos((float)px_abs, t_px_ts);

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

		// Set rat pos data to match robot
		if (!fc_isRatIn)
		{
			pos_ratVT.SetPos(0, 0);
			pos_ratPixy.SetPos(0, 0);
		}
		// Set rat pos data to match robot
		else if (fc_isTrackingEnabled)
		{
			// Set flag for reward 
			reward.is_ekfNew = true;
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
		ekfSetPos = ekfRobPos + pid.setPoint;

		// Check for nan values
		if (isnan(ekfRatPos) || isnan(ekfRobPos) || isnan(ekfRatVel) || isnan(ekfRobVel)) {
			char chr[50];
			sprintf(chr, "!! \"nan\" EKF OUTPUT: ratVT.pos=%0.2f ratPixy.pos=%0.2f robVT.pos=%0.2f ratVT.vel=%0.2f ratPixy.vel=%0.2f robVT.vel=%0.2f !!",
				pos_ratVT.posNow, pos_ratPixy.posNow, pos_robVT.posNow, pos_ratVT.velNow, pos_ratPixy.velNow, pos_robVT.velNow);
			DebugFlow(chr);
		}

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

// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid()
{
	// Local vars
	byte sol_state = digitalRead(pin_Rel_EtOH);

	// Change state
	sol_state = !sol_state;

	// Open/close solenoid
	digitalWrite(pin_Rel_EtOH, sol_state);

	// Make sure periodic drip does not run
	if (sol_state)
		doEtOHRun = false;
	else doEtOHRun = true;

	// Print to LCD
	char str[50];
	sprintf(str, "%s", digitalRead(pin_Rel_EtOH) == HIGH ? "OPEN" : "CLOSED");
	PrintLCD("EtOH SOLENOID", str, 's');
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
	if (doEtOHRun)
	{
		if (
			!isEtOHOpen &&
			(millis() > (t_etoh_start + dt_delEtOH) || etoh_dist_diff > distMaxEtOH)
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
			millis() > (t_etoh_start + dt_durEtOH)
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

		// Add to queue if streaming established
		if (fc_isStreaming)
			StorePacketData('c', 'J', byte_out, 0);

		char str[50];
		sprintf(str, "VCC: %0.2fV", volt_avg);
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
	static bool is_pressed[3] = { false, false, false };
	static uint32_t t_debounce[3] = { millis() + 1000, millis() + 1000, millis() + 1000 };
	static int dt_hold[3] = { 0, 0, 0 };
	static int dt_long_hold = 500;
	int btn_ind = 0;

	// RUN BUTTON 1 OPPERATIONS (Trigger reward)
	btn_ind = 0;
	if (digitalRead(pin_Btn[btn_ind]) == LOW)
	{
		// exit if < debounce time has not passed
		if (t_debounce[btn_ind] > millis()) return;

		// Set to start reward function
		btn_doRew = true;

		// Update debounce time
		t_debounce[btn_ind] = millis() + reward.duration + 100;
	}

	// RUN BUTTON 2 OPPERATIONS (Open/close solonoid)
	/*
	Note: Long hold to open/close EtOH
	*/
	btn_ind = 1;
	if (
		digitalRead(pin_Btn[btn_ind]) == LOW &&
		!is_pressed[btn_ind]
		)
	{
		// exit if < debounce time has not passed
		if (t_debounce[btn_ind] > millis()) return;

		// Get long hold time
		dt_hold[btn_ind] = millis() + dt_long_hold;

		// Set flag
		is_pressed[btn_ind] = true;
	}
	// Check hold time
	else if (is_pressed[btn_ind])
	{
		// short hold
		bool is_short_hold =
			digitalRead(pin_Btn[btn_ind]) == HIGH &&
			millis() < dt_hold[btn_ind];
		// long hold
		bool is_long_hold = millis() > dt_hold[btn_ind];

		// Check for either condition
		if (is_short_hold || is_long_hold) {

			// Run function 1
			if (is_short_hold) {
				btn_doRewSolStateChange = true;
				t_debounce[btn_ind] = millis() + 250;
			}

			// Run function 2
			if (is_long_hold) {
				btn_doEtOHSolStateChange = true;
				t_debounce[btn_ind] = millis() + 500;
			}

			// Reset flags
			dt_hold[btn_ind] = 0;
			is_pressed[btn_ind] = false;
		}

	}

	// RUN BUTTON 3 OPPERATIONS (Turn on/off LCD LED)
	btn_ind = 2;
	if (digitalRead(pin_Btn[btn_ind]) == LOW)
	{
		// exit if < 250 ms has not passed
		if (t_debounce[btn_ind] > millis()) return;

		btn_doChangeLCDstate = true;

		t_debounce[btn_ind] = millis() + 250;
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

// LOG/PRING MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, String set_to, String called_from)
{
	if (
		(doPrint_motorControl && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_motorControl && doDB_Log)
		)
	{
		char chr[50];
		sprintf(chr, "mc change %s: set_in=", pass ? "succeeded" : "failed");
		String str;
		str = chr + set_to + " set_out=" + fc_motorControl + " [" + called_from + "]";

		// Add to print queue
		if (doPrint_motorControl && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_motorControl && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

// LOG/PRING MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(String msg, uint32_t t, String called_from)
{
	if (
		(doPrint_motorBlocking && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_motorBlocking && doDB_Log)
		)
	{
		char chr[50];
		sprintf(chr, " %lu ms ", t);
		String str;
		str = msg + chr + " [" + called_from + "]";

		// Add to print queue
		if (doPrint_motorBlocking && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_motorBlocking && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

// LOG/PRING DROPPED PACKET DEBUG STRING
void DebugDropped(int missed, int missed_total, int total)
{
	if (
		(doPrint_dropped && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_dropped && doDB_Log)
		)
	{
		// Local vars
		int buff_tx = 0;
		int buff_rx = 0;

		// Get total data in buffers
		buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		buff_rx = Serial1.available();

		char str[50];
		sprintf(str, "!!PACK LOST: tot=%d/%d/%d tx=%d rx=%d!!", missed, missed_total, total, buff_tx, buff_rx);

		// Add to print queue
		if (doPrint_dropped && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_dropped && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

// LOG/PRING RESENT PACKET DEBUG STRING
void DebugResent(char id, uint16_t pack, int total)
{
	if (
		(doPrint_resent && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_resent && doDB_Log)
		)
	{
		// Local vars
		int buff_tx = 0;
		int buff_rx = 0;

		// Get total data in buffers
		buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		buff_rx = Serial1.available();

		char str[50];
		sprintf(str, "!!RESENT PACK: tot=%d tx=%d rx=%d id=%c pack=%d!!", total, buff_tx, buff_rx, id, pack);

		// Add to print queue
		if (doPrint_resent && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (doLog_resent && doDB_Log)
			StoreDBLogStr(str, millis());
	}
}

// LOG/PRING RECIEVED PACKET DEBUG STRING
void DebugRcvd(char from, char id, uint16_t pack)
{
	if (
		(doPrint_rcvd && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_rcvd && doDB_Log)
		)
	{
		char chr[50];

		// Print specific pack contents
		if (id == 'T')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d dat2=%d pack=%d", from, id, msg_testCmd[0], msg_testCmd[1], pack);
		}
		else if (id == 'S')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d dat2=%d pack=%d", from, id, msg_setupCmd[0], msg_setupCmd[1], pack);
		}
		else if (id == 'Q')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d dat2=%d pack=%d", from, id, pack);
		}
		else if (id == 'M')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%0.2f pack=%d", from, id, msg_moveToTarg, pack);
		}
		else if (id == 'R')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%0.2f dat2=%d pack=%d", from, id, msg_rewPos, msg_rewDelay, pack);
		}
		else if (id == 'C')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%0.2f dat2=%d pack=%d", from, id, msg_rewPos, msg_rewZoneInd, pack);
		}
		else if (id == 'H')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d pack=%d", from, id, fc_doHalt, pack);
		}
		else if (id == 'B')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d pack=%d", from, id, msg_bullDel, pack);
		}
		else if (id == 'I')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d pack=%d", from, id, fc_isRatIn, pack);
		}
		else if (id == 'L')
		{
			sprintf(chr, "rcvd_%c2r: id=%c dat1=%d pack=%d", from, id, fc_doLogResend, pack);
		}
		else sprintf(chr, "rcvd_%c2r: id=%c pack=%d", from, id, pack);

		// Convert to string
		String str = chr;

		// Add to print queue
		if (doPrint_rcvd &&
			(doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(str, t_rcvd);

		// Add to log queue
		if (doLog_rcvd && doDB_Log)
			StoreDBLogStr(str, t_rcvd);
	}

}

// LOG/PRING IR SENSOR EVENT
void DebugIRSync(String msg, uint32_t t)
{
	if (
		(doPrint_irSync && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_irSync && doDB_Log)
		)
	{
		// Add to print queue
		if (doPrint_irSync && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(msg, t);
		// Add to log queue
		if (doLog_irSync && doDB_Log)
			StoreDBLogStr(msg, t);
	}
}

// LOG/PRING MAIN EVENT
void DebugFlow(String msg)
{
	DebugFlow(msg, millis());
}
void DebugFlow(String msg, uint32_t t)
{
	if (
		(doPrint_flow && (doDB_PrintConsole || doDB_PrintLCD)) ||
		(doLog_flow && doDB_Log)
		)
	{
		// Add to print queue
		if (doPrint_flow && (doDB_PrintConsole || doDB_PrintLCD))
			StoreDBPrintStr(msg, t);
		// Add to log queue
		if (doLog_flow && doDB_Log)
			StoreDBLogStr(msg, t);
	}
}

// STORE STRING FOR PRINTING
void StoreDBPrintStr(String msg, uint32_t t)
{
	// Local vars
	static String last_msg = " ";
	static uint32_t t_ts_last = millis();
	char t_str_long[200];
	char t_str_ellapsed[200];
	uint32_t t_ts_norm = 0;
	float t_c = 0;
	float t_ellapsed = 0;

	// Dont store repeated string
	if (msg != last_msg) {

		// Save string
		last_msg = msg;

		// Time now
		t_ts_norm = t_sync == 0 ? t : t - t_sync;

		// Total time
		t_c = (float)(t_ts_norm) / 1000.0f;

		// Ellapsed time
		t_ellapsed = (float)((millis() - t_ts_last) / 1000.0f);
		t_ts_last = millis();

		// Save long and short string
		sprintf(t_str_long, "[%0.2fs]", t_c);
		sprintf(t_str_ellapsed, "[%0.0fs] ", t_ellapsed);

		// Shift queue
		for (int i = 0; i < printQueue_lng - 1; i++)
		{
			printQueue[i] = printQueue[i + 1];
		}

		// Set first entry to new string 
		if (doDB_PrintLCD) printQueue[printQueue_lng - 1] = t_str_ellapsed + msg;
		else if (doDB_PrintConsole) printQueue[printQueue_lng - 1] = msg;

		// Save total time to end
		printQueue[0] = t_str_long;

		// Set queue ind
		printQueueInd--;

		// Set flag
		doPrint = true;

	}
}

// STORE STRING FOR LOGGING
void StoreDBLogStr(String msg, uint32_t t)
{
	// Local vars
	static String last_msg = " ";
	char str_c[200];
	char msg_c[200];
	uint32_t t_ts_norm = 0;

	// Dont store repeated string or log data while sending log
	if (
		fc_isStreaming &&
		!fc_isSendingLog &&
		msg != last_msg
		)
	{

		// Save string
		last_msg = msg;

		// Itterate log entry count
		cnt_logStore++;

		// Check for overflow 
		if (
			cnt_logStore < logLng &&
			logByteTrack <= logByteMax
			)
		{
			// Concatinate ts with message
			msg.toCharArray(str_c, 200);
			sprintf(msg_c, "[%d],%lu,%s", cnt_logStore, t, str_c);
			logList[cnt_logStore - 1] = msg_c;

			// Update byte count
			logByteTrack += logList[cnt_logStore - 1].length();
		}
		// Log if full
		else
		{
			// Store final message
			sprintf(msg_c, "[%d],%lu,!!LOG FULL: bytes=%d!!", cnt_logStore, t, logByteTrack);
			logList[cnt_logStore - 1] = msg_c;

			// Set flag to stop logging
			doDB_Log = false;
		}
	}
}

// PRINT DEBUG STRINGS TO CONSOLE/LCD
void PrintDebug()
{

	// Avoid overlap between sent or rcvd events
	if (millis() < t_sent + dt_sendSent ||
		millis() < t_rcvd + dt_sendRcvd)
	{
		return;
	}

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

			// Pad string
			char chr[50];
			char arg[50];
			sprintf(arg, "%%%ds", 20 - printQueue[0].length());
			sprintf(chr, arg, '_');

			// Append string
			String str = printQueue[0] + chr + printQueue[printQueueInd] + "\n";

			// Print
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

// FOR PRINTING TO LCD
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

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// GET ID INDEX
int ID_Ind(char id, char id_arr[], int arr_size)
{

	int ind = -1;
	for (int i = 0; i < arr_size; i++)
	{
		if (id == id_arr[i])
			ind = i;
	}

	return ind;

}

// BLINK LEDS AT SETUP
void SetupBlink()
{
	int duty[2] = { 100, 0 };
	bool is_on = false;
	int dt = 100;
	// Flash sequentially
	for (int i = 0; i < 8; i++)
	{
		analogWrite(pin_Disp_LED, duty[(int)is_on]);
		delay(dt);
		analogWrite(pin_TrackLED, duty[(int)is_on]);
		delay(dt);
		analogWrite(pin_RewLED_R, duty[(int)is_on]);
		delay(dt);
		is_on = !is_on;
	}
	// Reset LEDs
	analogWrite(pin_Disp_LED, 0);
	analogWrite(pin_TrackLED, trackLEDduty);
	analogWrite(pin_RewLED_R, rewLEDmin);
}

// BLICK LEDS WHEN RAT FIRST DETECTED
void RatInBlink()
{
	// Local vars
	int duty[2] = { 255, 0 };
	bool is_on = true;
	int dt = 50;

	// Flash 
	for (int i = 0; i < 3; i++)
	{
		analogWrite(pin_RewLED_R, duty[(int)is_on]);
		analogWrite(pin_TrackLED, duty[(int)is_on]);
		delay(dt);
		is_on = !is_on;
	}
	// Reset LED
	analogWrite(pin_RewLED_R, 0);
	analogWrite(pin_TrackLED, trackLEDduty);
}

#pragma endregion


#pragma region ---------INTERUPTS---------

// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt() {

	// Exit if < 250 ms has not passed
	if (t_irProxDebounce > millis()) return;

	// Run stop in main loop
	t_doIRhardStop = true;

	// Update debounce
	t_irProxDebounce = millis() + 250;
}

// DETECT IR SYNC EVENT
void Interupt_IR_Detect()
{
	// Exit if < 250 ms has not passed
	if (t_irDetectDebounce > millis()) return;

	// Store time
	t_irSyncLast = millis();

	// Check if this if first event
	if (t_sync == 0)
	{
		t_sync = t_irSyncLast;
	}

	// Set flag
	intrpt_doLogIR = true;

	// Update debounce
	t_irDetectDebounce = millis() + 250;
}
#pragma endregion
