#pragma once
#include "Arduino.h"

#pragma region ============ DEBUG SETTINGS =============

// LOG DEBUGGING
struct DB
{
	// Logging
	bool Log = true;
	// What to print
	const bool log_errors = true;
	const bool log_flow = true;
	const bool log_c2r = true;
	const bool log_a2r = true;
	const bool log_r2c = true;
	const bool log_r2a = true;
	const bool log_pid = true;
	const bool log_bull = true;
	const bool log_motorControl = true;
	const bool log_runSpeed = false;
	// tracking data
	const bool log_pos = false;
	const bool log_pos_rat_vt = false;
	const bool log_pos_rat_pixy = false;
	const bool log_pos_rob_vt = false;
	const bool log_pos_rat_ekf = false;
	const bool log_pos_rob_ekf = false;
	const bool log_vel_rat_vt = false;
	const bool log_vel_rat_pixy = false;
	const bool log_vel_rob_vt = false;
	const bool log_vel_rat_ekf = false;
	const bool log_vel_rob_ekf = false;

	// Printing
	bool Console = true;
	bool LCD = false;
	// What to print
	const bool print_errors = true;
	const bool print_flow = true;
	const bool print_logging = false;
	const bool print_c2r = true;
	const bool print_r2c = true;
	const bool print_a2r = true;
	const bool print_r2a = true;
	const bool print_rcvdVT = false;
	const bool print_pid = false;
	const bool print_bull = false;
	const bool print_logMode = false;
	const bool print_logStore = false;
	const bool print_a2o = false;
	const bool print_o2a = false;
	const bool print_o2aRaw = false;
	const bool print_motorControl = false;
	const bool print_runSpeed = false;

	// Testing
	const bool do_posDebug = false;
	const bool do_posPlot = false;
	bool do_pidCalibration = false;
	bool do_simRatTest = false;

	// Other
	bool isErrLoop = false;
	bool wasErrLoop = false;
}
// Initialize
db;

// Pid calibration parameters
/*
Set kC and run ICR_Run.cs
*/
const float kC = 5; // critical gain [1.5,3,5]
const float pC = 1.5; // oscillation period [0,1.75,1.5]  
const double cal_speedSteps[4] = { 10, 20, 30, 40 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;

#pragma endregion


#pragma region ============== PIN MAPPING ==============

// Pin mapping
struct PIN
{
	// Autodriver
	const int AD_CSP_R = 5;
	const int AD_CSP_F = 6;
	const int AD_RST = 7;

	// XBees
	const int X1a_CTS = 29;
	const int X1b_CTS = 27;
	const int X1a_UNDEF = 28;
	const int X1b_UNDEF = 26;

	// Display
	const int Disp_SCK = 8;
	const int Disp_MOSI = 9;
	const int Disp_DC = 10;
	const int Disp_RST = 11;
	const int Disp_CS = 12;
	const int Disp_LED = 13;

	// LEDs
	const int RewLED_R = 4;
	const int RewLED_C = 3;
	const int TrackLED = 2;

	// Relays
	const int Rel_EtOH = 23;
	const int Rel_Rew = 22;
	const int Rel_Vcc = A5;

	// Voltage Regulators
	const int REG_24V_ENBLE = 34;
	const int REG_12V_ENBLE = 46;
	const int REG_5V_ENBLE = 48;

	// BigEasyDriver
	const int ED_RST = 47;
	const int ED_SLP = 49;
	const int ED_DIR = 51;
	const int ED_STP = 53;
	const int ED_ENBL = 35;
	const int ED_MS1 = 37;
	const int ED_MS2 = 39;
	const int ED_MS3 = 41;

	// OpenLog
	const int OL_RST = 30;

	// Feeder switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int FeedSwitch_Gnd = 24;
	const int FeedSwitch = 25;

	// Power off
	const int KillSwitch = 45;

	// Voltage monitor
	const int BatVcc = A6;
	const int BatIC = A7;

	// Buttons
	const int Btn[3] = { A2, A1, A0 };

	/*
	Note: pins bellow are all used for external interupts
	and must all be members of the same port (PortA)
	*/

	// IR proximity sensors
	const int IRprox_Rt = 42;
	const int IRprox_Lft = 43;

	// IR detector
	const int IRdetect = 31;
}
// Initialize
pin;

#pragma endregion


#pragma region ============= VARIABLE SETUP ============

// Flow/state control
struct FC
{
	String motorControl = "None"; // ["None", "Halt", "Open", "MoveTo", "Bull", "Pid"]
	bool doAllowRevMove = false;
	bool isBlockingTill = false;
	bool doQuit = false;
	bool isSesStarted = false;
	bool doStreamCheck = false;
	bool isComsStarted = false;
	bool doSendVCC = false;
	bool isManualSes = false;
	bool isRatIn = false;
	bool isTrackingEnabled = false;
	bool doMove = false;
	bool doRew = false;
	bool doHalt = false;
	bool doBulldoze = false;
	bool doLogSend = false;
	bool doBlockVccSend = false;
	bool doBlockLogWrite = false;
	bool doBlockWriteLCD = false;
	bool isEKFReady = false;
	bool doPackSend = false;
	bool doEtOHRun = true;
	bool isLitLCD = false;
	bool doBtnRew = false;
	bool doRewSolStateChange = false;
	bool doEtOHSolStateChange = false;
	bool doChangeLCDstate = false;
	bool doMoveRobFwd = false;
	bool doMoveRobRev = false;
}
// Initialize
fc;

// Debugging general
uint32_t cnt_loop_tot = 0;
uint16_t cnt_loop_short = 0;
uint16_t cnt_warn = 0;
uint16_t cnt_err = 0;
uint16_t warn_line[100] = { 0 };
uint16_t err_line[100] = { 0 };

// Print debugging
const int printQueueSize = 15;
char printQueue[printQueueSize][300] = { { 0 } };
int printQueueIndStore = 0;
int printQueueIndRead = 0;

// Serial com general
const int sendQueueSize = 10;
const int sendQueueBytes = 19;
byte sendQueue[sendQueueSize][sendQueueBytes] = { { 0 } };
int sendQueueInd = sendQueueSize - 1;
const int resendMax = 5;
const int dt_resend = 100; // (ms)
const int dt_sendSent = 7; // (ms) 
const int dt_sendRcvd = 1; // (ms) 
uint32_t t_xBeeSent = 0; // (ms)
uint32_t t_xBeeRcvd = 0; // (ms)
int dt_xBeeSent = 0; // (ms)
int dt_xBeeRcvd = 0; // (ms)
int cnt_packBytesRead = 0;
int cnt_packBytesSent = 0;
int cnt_packBytesDiscarded = 0;

// Start/Quit
uint32_t t_ardQuit = 0;
uint32_t t_quit = 0;

// Pixy
const double pixyCoeff[5] = {
	0.000000043550534,
	-0.000023239535204,
	0.005033059128963,
	-0.677050955917591,
	75.424132382709260
};

// AutoDriver
const double cm2stp = 200 / (9 * PI);
const double stp2cm = (9 * PI) / 200;
const float maxSpeed = 100; // (cm) 
const float maxAcc = 80; // (cm) 
const float maxDec = 80; // (cm)
const double scaleFrontAD = 1.031; // 1.0375
const byte kAcc = 60 * 2;
const byte kDec = 60 * 2;
const byte kRun = 60;
const byte kHold = 60 / 2;
double runSpeedNow = 0;
char runDirNow = 'f';
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 10; // (ms)
uint32_t t_checkAD = millis() + dt_checkAD; // (ms)

// Kalman model measures
struct KAL
{
	double RatPos = 0;
	double RobPos = 0;
	double RatVel = 0;
	double RobVel = 0;
	int cnt = 0;
	uint32_t t_update = 0;
}
kal;

// Pid Settings
bool doIncludeTerm[2] = { true, true };
const float pidSetPoint = 50; // (cm)
const float guardDist = 4.5;
const float feedDist = 66;

// Movement
float moveToSpeed = 80; // (cm/sec)

						// REWARD
const int dt_rewBlock = 15000; // (ms)
uint32_t t_rewBlockMove = 0; // (ms)

// Solonoids
/*
EtOH run after min time or distance
*/
const int dt_durEtOH[2] = { 200, 100 }; // (ms)
const int dt_delEtOH[2] = { 10000, 60000 }; // (ms)
uint32_t t_solOpen = 0;
uint32_t t_solClose = 0;

// Battery tracking
/*
Updated when EtOH relay opened
*/
const float bit2volt = 0.01545; // was 0.0164;
uint32_t t_vccSend = 0;
uint32_t t_vccUpdate = 0;
const int dt_vccSend = 30000;
const int dt_vccUpdate = 1000;
const int dt_vccCheck = 500;
float vccNow = 0;
float vccCutoff = 11.6;
float batVoltArr[100] = { 0 };
float icNow = 0;

// LEDs
const int trackLEDduty = 75; // value between 0 and 255
const int rewLEDduty = 15; // value between 0 and 255
const int rewLEDmin = 0; // value between 0 and 255

						 // LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];

// Interrupts 
volatile uint32_t t_sync = 0; // (ms)
volatile uint32_t v_t_irProxDebounce = millis(); // (ms)
volatile uint32_t v_t_irDetectDebounce = millis(); // (ms)
volatile uint32_t v_t_irSyncLast = 0; // (ms)
volatile int v_dt_ir = 0;
volatile int v_cnt_ir = 0;
volatile bool v_doIRhardStop = false;
volatile bool v_doLogIR = false;
volatile bool v_stepState = false;
volatile bool v_stepTimerActive = false;
volatile bool v_isArmMoveDone = false;
volatile int v_cnt_steps = 0;
volatile int v_stepTarg = 0;
volatile char v_stepDir = 'e'; // ['e','r']

#pragma endregion 


#pragma region ============ COM STRUCT SETUP ===========

// C2R command vars
struct CMD
{
	byte testCond = 0;
	byte testDat = 0;
	byte sesCond = 0;
	byte soundCond = 0;
	byte vtEnt = 0;
	float vtCM[2] = { 0,0 };
	uint32_t vtTS[2] = { 0,0 };
	float moveToTarg = 0;
	byte bullDel = 0;
	byte bullSpeed = 0;
	float rewPos = 0;
	byte rewZoneInd = 0;
	byte rewDelay = 0;
	byte rewCond = 0;
} 
cmd;

// Serial to robot
struct R4
{
	USARTClass &port;
	char instID;
	int lng;
	char head;
	char foot;
	char id[20];
	uint16_t pack[20];
	uint16_t packLast[20];
	int packTot;
	int cnt_repeat;
	int cnt_dropped;
	char idNow;
	bool isNew;
	float dat[3];
};

// Initialize C2R
R4 c2r
{
	// serial
	Serial3,
	// instID
	'c',
	// lng
	16,		
	// head
	'<',		
	// foot
	'>',		
	// id
	 { 			
		'h', // setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
		'H', // halt movement
		'B', // bulldoze rat
		'I', // rat in/out
		'V', // request stream status
		'L', // request log conf/send
		'J', // battery voltage
		'Z', // reward zone
		'U', // log size
		'D', // execution done
		'P', // position data
		' ', ' ', ' ', ' '
	},
	// pack
	{ 0 },	
	// packLast
	{ 0 },	
	// packTot
	0,		
	// cnt_repeat
	0,		
	// cnt_dropped
	0,		
	// idNow
	'\0',	
	// isNew
	false,	
	// dat
	{ 0, 0, 0 }, 
};

// Serial from other ard
R4 a2r
{
	// serial
	Serial2,
	// instID
	'a',
	// lng
	5,
	// head
	'{',
	// foot
	'}',
	// id
	{
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
		' ', ' ', ' ', ' ', ' ',
		' ', ' ', ' ', ' ', ' ',
		' ', ' ', ' ', ' ', ' '
	},
	// pack
	{ 0 },
	// packLast
	{ 0 },
	// packTot
	0,
	// cnt_repeat
	0,
	// cnt_dropped
	0,
	// idNow
	'\0',
	// isNew
	false,
	// dat
	{ 0, 0, 0 },
};

// Serial from Robot
struct R2
{
	USARTClass &port;
	char instID;
	int lng;
	char head;
	char foot;
	const char id[20];
	uint16_t pack[20];
	uint16_t packLast[20];
	uint16_t cnt_pack;
	int cnt_repeat;
	float datList[20][3];
	uint32_t t_sentList[20];
	bool doRcvCheck[20];
	int cnt_resend[20];
	int pinCTS;
	bool stateCTS;
};

// Serial to CS
R2 r2c
{
	// serial
	Serial3,
	// instID
	'c',
	// lng
	16,
	// head
	'<',
	// foot
	'>',
	// id
	{
		'h', // setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
		'H', // halt movement
		'B', // bulldoze rat
		'I', // rat in/out
		'V', // request stream status
		'L', // request log conf/send
		'J', // battery voltage
		'Z', // reward zone
		'U', // log size
		'D', // execution done
		'P', // position data
		' ', ' ', ' ', ' '
	},
	// pack
	{ 0 },
	// packLast
	{ 0 },
	// cnt_pack
	0,
	// cnt_repeat
	0,
	// datList
	{ { 0 } },
	// t_sentList
	{ 0 },
	// doRcvCheck
	{ false },
	// cnt_resend
	{ 0 },
	// pinCTS
	pin.X1a_CTS,
	// stateCTS
	false,
};

// Serial to other ard
R2 r2a
{
	// serial
	Serial2,
	// instID
	'a',
	// lng
	5,
	// head
	'{',
	// foot
	'}',
	// id
	{
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
		' ', ' ', ' ', ' ', ' ',
		' ', ' ', ' ', ' ', ' ',
		' ', ' ', ' ', ' ', ' '
	},
	// pack
	{ 0 },
	// packLast
	{ 0 },
	// cnt_pack
	0,
	// cnt_repeat
	0,
	// datList
	{ { 0 } },
	// t_sentList
	{ 0 },
	// doRcvCheck
	{ false },
	// cnt_resend
	{ 0 },
	// pinCTS
	pin.X1b_CTS,
	// stateCTS
	false,
};

#pragma endregion 


#pragma region ========== CLASS DECLARATIONS ===========

#pragma region ----------CLASS: POSTRACK----------
class POSTRACK
{
public:
	// VARS
	char instID[20] = { 0 };
	int nSamp = 0;
	double posArr[6] = { 0,0,0,0,0,0 }; // (cm)
	uint32_t t_tsArr[6] = { 0,0,0,0,0,0 }; // (ms)
	int dt_skip = 0;
	double velNow = 0.0f; // (cm/sec)
	double velLast = 0.0f; // (cm/sec)
	double posNow = 0.0f; // (cm)
	double posAbs = 0.0f; // (cm)
	int cnt_error = 0;
	uint32_t t_tsNow = 0;
	uint32_t t_msNow = millis();
	int nLaps = 0;
	int sampCnt = 0;
	bool isNew = false;
	bool is_streamStarted = false;
	uint32_t t_init;
	// METHODS
	POSTRACK(uint32_t t, char obj_id[], int n_samp);
	void UpdatePos(double pos_new, uint32_t ts_new);
	double GetPos();
	double GetVel();
	void SwapPos(double set_pos, uint32_t t);
	void Reset();
};

#pragma endregion 

#pragma region ----------CLASS: PID----------
class PID
{

public:
	// VARS
	uint32_t t_updateLast = 0;
	double dt_update = 0;
	bool isPidUpdated = false;
	double p_term = 0;
	double i_term = 0;
	double d_term = 0;
	bool isFirstRun = true;
	String mode = "Manual"; // ["Manual" "Automatic" "Halted"]
	bool isHolding4cross = false;
	bool doThrottle = false;
	bool isThrottled = false;
	double error = 0;
	double errorLast = 0;
	double errorFeeder = 0;
	double integral = 0;
	double derivative = 0;
	double velUpdate = 0;
	double runSpeed = 0;
	int speedMax = maxSpeed;
	double dT = 0;
	double kP = 0;
	double kI = 0;
	double kD = 0;
	float setPoint = 0;
	uint32_t t_ekfReady = 0;
	const int dt_ekfSettle = 250; // (ms)
	bool is_ekfNew = false;
	float throttleAcc = 20;
	float throttleSpeedStop = 40;
	const int dt_throttle = 4000;
	uint32_t t_throttleTill = 0;
	uint32_t t_lastThrottle = millis();
	int cal_dt_min = 40; // (ms)
	int cal_cntPcArr[4] = { 0, 0, 0, 0 };
	int cal_stepNow = 0;
	float cal_PcCnt = 0; // oscillation count
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
	float cal_dt_update = 0;
	float cal_errCnt = 0;
	float cal_errSum = 0;
	float cal_errMax = 0;
	float cal_errMin = 0;
	double cal_ratPos = 0;
	double cal_ratVel = 0;
	bool cal_isPidUpdated = false;
	bool cal_isCalFinished = false;
	uint32_t t_init;
	// METHODS
	PID(uint32_t t, const float kC, const float pC, const float set_point);
	double UpdatePID();
	void Run(char called_from[]);
	void Stop(char called_from[]);
	void Hold(char called_from[]);
	void Reset();
	void SetThrottle();
	void CheckThrottle();
	void CheckMotorControl();
	void CheckSetpointCrossing();
	void CheckEKF(uint32_t t);
	void ResetEKF(char called_from[]);
	void SetUpdateTime(uint32_t t);
	void PrintPID(char msg[]);
	double RunPidCalibration();
};

#pragma endregion 

#pragma region ----------CLASS: BULLDOZE----------
class BULLDOZE
{
public:
	// VARS
	uint32_t t_updateNext = 0;
	int dt_update = 50; // (ms)
	float moveMin = 5; // (cm)
	String mode = "Inactive"; // ["Active" "Inactive"]
	String state = "Off"; // ["off", "On", "Hold"]
	uint32_t t_bullNext = 0; // (ms)
	int bSpeed = 0;
	int bDelay = 0; // (ms)
	double posCheck = 0;
	double posNow = 0;
	double distMoved = 0;
	double guardPos = 0;
	bool isMoved = false;
	bool isTimeUp = false;
	bool isPassedReset = false;
	uint32_t t_init;
	// METHODS
	BULLDOZE(uint32_t t);
	void UpdateBull();
	void Reinitialize(byte del, byte spd, char called_from[]);
	void Run(char called_from[]);
	void Stop(char called_from[]);
	void TurnOn(char called_from[]);
	void TurnOff(char called_from[]);
	void Hold(char called_from[]);
	void Resume(char called_from[]);
	void Reset();
	void CheckMotorControl();
	void PrintBull(char msg[]);
};

#pragma endregion 

#pragma region ----------CLASS: MOVETO----------
class MOVETO
{
public:
	// VARS
	int targSetTimeout = 1000;
	int moveTimeout = 10000;
	uint32_t t_tryTargSetTill = 0;
	uint32_t t_tryMoveTill = 0;
	bool doAbortMove = false;
	double posAbs = 0;
	float minSpeed = 0;
	const int dt_update = 10;
	uint32_t t_updateNext = 0;
	double distLeft = 0;
	double posCumStart = 0;
	double posAbsStart = 0;
	double targPos = 0;
	double targDist = 0;
	char moveDir = 'f';
	double baseSpeed = 0;
	bool isTargSet = false;
	bool isMoveStarted = false;
	bool isTargReached = false;
	double haltError = 0;
	const double velCoeff[3] = {
		0.001830357142857,
		0.131160714285714,
		-2.425892857142854,
	};
	uint32_t t_init;
	// METHODS
	MOVETO(uint32_t t);
	bool CompTarg(double now_pos, double targ_pos);
	double DecelToTarg(double now_pos, double now_vel, double dist_decelerate, double speed_min);
	double GetError(double now_pos);
	void Reset();
};

#pragma endregion 

#pragma region ----------CLASS: REWARD----------
class REWARD
{
public:
	// VARS
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
	const int zoneLocs[9] = { // (deg)
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
	double zoneBounds[zoneLng][2] = { { 0 } };
	int zoneOccTim[zoneLng] = { 0 };
	int zoneOccCnt[zoneLng] = { 0 };
	uint32_t t_nowZoneCheck = 0;
	uint32_t t_lastZoneCheck = 0;
	char mode_str[10] = { 0 }; // ["None" "Free" "Cue" "Now"]
	String mode = mode_str;
	int occThresh = 0; // (ms)
	int durationDefault = 1420; // (ms) 
	int duration = 0; // (ms) 
	byte durationByte = 0; // (ms) 
	int zoneMin = 0;
	int zoneMax = 0;
	double boundMin = 0;
	double boundMax = 0;
	uint32_t t_rew_str = 0;
	uint32_t t_rew_end = 0;
	uint32_t t_closeSol = 0;
	uint32_t t_retractArm = 0;
	uint32_t t_moveArmStr = 0;
	double rewCenterRel = 0;
	bool isRewarding = false;
	bool isBoundsSet = false;
	bool isZoneTriggered = false;
	bool isAllZonePassed = false;
	bool is_ekfNew = false;
	int zoneInd = 255;
	int zoneRewarded = 0;
	double boundsRewarded[2] = { 0 };
	int occRewarded = 0;
	int lapN = 0;
	uint32_t armMoveTimeout = 5000;
	bool doArmMove = false;
	bool doExtendArm = false;
	bool doRetractArm = false;
	bool doTimedRetract = false;
	bool isArmExtended = true;
	const int armExtStps = 150;
	const int dt_step_high = 500; // (us)
	const int dt_step_low = 500; // (us)
	bool isArmStpOn = false;
	uint32_t t_init;
	// METHODS
	REWARD(uint32_t t);
	void StartRew();
	bool EndRew();
	void SetRewDur(int zone_ind);
	void SetRewMode(char mode_now[], int arg2 = 0);
	bool CompZoneBounds(double now_pos, double rew_pos);
	bool CheckZoneBounds(double now_pos);
	void ExtendFeedArm();
	void RetractFeedArm();
	void CheckFeedArm();
	void Reset();
};

#pragma endregion 

#pragma region ----------CLASS: LOGGER----------
class LOGGER
{
public:
	// VARS
	USARTClass &port = Serial1;
	uint32_t t_sent = 0; // (ms)
	uint32_t t_rcvd = 0; // (ms)
	uint32_t t_write = 0; // (us)
	int dt_write = 50 * 1000; // (us)
	uint32_t t_beginSend = 0;
	int dt_beginSend = 1000;
	static const int logQueueSize = 30;
	char logQueue[logQueueSize][300] = { { 0 } };
	int queueIndStore = 0;
	int queueIndRead = 0;
	int cnt_logsStored = 0;
	static const int maxBytesStore = 2500;
	char rcvdArr[maxBytesStore] = { 0 };
	char mode = ' '; // ['<', '>']
	char openLgSettings[100] = { 0 };
	char logFile[50] = { 0 };
	int logNum = 0;
	char logCntStr[10] = { 0 };
	int cnt_logBytesStored = 0;
	int cnt_logBytesSent = 0;
	uint32_t t_init;
	// METHODS
	LOGGER(uint32_t t);
	bool Setup();
	int OpenNewLog();
	bool SetToCmdMode();
	void GetCommand();
	char SendCommand(char msg[], bool do_conf = true, uint32_t timeout = 5000);
	char GetReply(uint32_t timeout);
	bool SetToWriteMode(char log_file[]);
	void QueueLog(char msg[], uint32_t t = millis());
	bool WriteLog(bool do_send = false);
	void StreamLogs();
	void TestLoad(int n_entry, char log_file[] = '\0');
	int GetFileSize(char log_file[]);
	void PrintLOGGER(char msg[], bool start_entry = false);

};

#pragma endregion 

#pragma region ----------CLASS: FUSER----------
class FUSER : public TinyEKF
{
public:
	FUSER()
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

#pragma endregion 

#pragma region ----------UNION----------
union UTAG {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (uint16_t) 2 byte
	uint32_t i32; // (uint32_t) 4 byte
	float f; // (float) 4 byte
};

#pragma endregion 

#pragma region ----------INITILIZE OBJECTS----------

FUSER ekf;
UTAG U;
AutoDriver_Due AD_R(pin.AD_CSP_R, pin.AD_RST);
AutoDriver_Due AD_F(pin.AD_CSP_F, pin.AD_RST);
PixyI2C Pixy(0x54);
LCD5110 LCD(pin.Disp_CS, pin.Disp_RST, pin.Disp_DC, pin.Disp_MOSI, pin.Disp_SCK);
POSTRACK Pos[3] = {
	POSTRACK(millis(), "RatVT", 4),
	POSTRACK(millis(), "RobVT", 4),
	POSTRACK(millis(), "RatPixy", 6)
};
PID Pid(millis(), kC, pC, pidSetPoint);
BULLDOZE Bull(millis());
MOVETO Move(millis());
REWARD Reward(millis());
LOGGER Log(millis());

#pragma endregion 

#pragma endregion 


#pragma region ========= FUNCTION DECLARATIONS =========

// PARSE SERIAL INPUT
void GetSerial(R4 *r4, R2 *r2);
// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4 *r4, char mtch = '\0');
// STORE PACKET DATA TO BE SENT
void QueuePacket(R2 *r2, char id, float dat1 = 0, float dat2 = 0, float dat3 = 0, uint16_t pack = 0, bool do_conf = true);
// SEND SERIAL PACKET DATA
void SendPacket();
// CHECK IF ARD TO ARD PACKET SHOULD BE RESENT
bool CheckResend(R2 *r2);
// RESET AUTODRIVER BOARDS
void AD_Reset(float max_speed = maxSpeed, float max_acc = maxAcc, float max_dec = maxDec);
// CONFIGURE AUTODRIVER BOARDS
void AD_Config(float max_speed, float max_acc, float max_dec);
// CHECK AUTODRIVER STATUS
void AD_CheckOC(bool force_check = false);
// HARD STOP
void HardStop(char called_from[]);
// IR TRIGGERED HARD STOP
void IRprox_Halt();
// RUN AUTODRIVER
bool RunMotor(char dir, double new_speed, String agent);
// RUN MOTOR MANUALLY
bool ManualRun(char dir);
// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(char set_to[], char called_from[]);
// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(int dt, char called_from[]);
// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed();
// DO SETUP TO BEGIN TRACKING
void InitializeTracking();
// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT();
// PROCESS PIXY STREAM
void CheckPixy();
// UPDATE EKF
void UpdateEKF();
// LOG TRACKING
void LogTrackingData();
// CHECK FOR BUTTON INPUT
bool GetButtonInput();
// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid();
// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid();
// CHECK FOR ETOH UPDATE
void CheckEtOH();
// CHECK BATTERY VALUES
float CheckBattery(bool force_check = false);
// TURN LCD LIGHT ON/OFF
void ChangeLCDlight(uint32_t duty = 256);
// QUIT AND RESTART ARDUINO
void QuitSession();
// CHECK LOOP TIME AND MEMORY
void CheckLoop();
// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t = millis());
// LOG/PRINT ERRORS
void DebugError(char msg[], bool is_error = false, uint32_t t = millis());
// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[]);
// LOG/PRINT MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(char msg[], char called_from[], uint32_t t = millis());
// LOG/PRINT MOTOR SPEED CHANGE
void DebugRunSpeed(String agent, double speed_last, double speed_now);
// LOG/PRINT RECIEVED PACKET DEBUG STRING
void DebugRcvd(R4 *r4, char msg[], bool is_repeat = false);
// LOG/PRINT SENT PACKET DEBUG STRING
void DebugSent(R2 *r2, char msg[], bool is_repeat = false);
// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t);
// PRINT DEBUG STRINGS TO CONSOLE/LCD
bool PrintDebug();
// FOR PRINTING TO LCD
void PrintLCD(bool do_block, char msg_1[], char msg_2[] = { 0 }, char f_siz = 's');
// CLEAR LCD
void ClearLCD();
// PRINT SPECIAL CHARICTERS
char* PrintSpecialChars(char chr, bool do_show_byte = false);
// CHECK AUTODRIVER BOARD STATUS
int CheckAD_Status(uint16_t stat_reg, String stat_str);
// SEND TEST PACKET
void TestSendPack(R2 *r2, R4 *r4, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf);
// HOLD FOR CRITICAL ERRORS
void RunErrorHold(char msg[], uint32_t t_kill = 0);
// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size);
// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink(int n_blinks, int dt_led = 0);
// TIMER INTERUPT/HANDLER
void Interupt_TimerHandler();
// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt();
// DETECT IR SYNC EVENT
void Interupt_IR_Detect();

#pragma endregion
