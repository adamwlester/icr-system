// ######################################

// ============= FEEDERDUE ==============

// ######################################

/* NOTES
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
double = 8 byte
*/


#pragma region ========= LIBRARIES & EXT DEFS =========


//-------SOFTWARE RESET----------
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

//----------LIBRARIES------------

// General
#include <string.h>
//
#include <MemoryFree.h>

// AutoDriver

#include <SPI.h>
//
#include "AutoDriver_Due.h"

// Pixy

#include <Wire.h> 
//
#include <PixyI2C.h>

// LCD
#include <LCD5110_Graph.h>

// TinyEKF
#define N 4     // States
#define M 6     // Measurements
#include <TinyEKF.h>

#pragma endregion 


#pragma region ========= DEBUG SETTINGS =========

// LOG DEBUGGING

struct DB
{
	// Do log
	bool Log = false;
	// What to print
	const bool log_errors = true;
	const bool log_flow = true;
	const bool log_motorControl = true;
	const bool log_c2r = true;
	const bool log_r2c = true;
	const bool log_r2a = true;
	const bool log_pid = true;
	const bool log_bull = true;

	// Where to print
	bool Console = true;
	bool LCD = false;
	// What to print
	const bool print_errors = false;
	const bool print_flow = false;
	const bool print_motorControl = false;
	const bool print_c2r = false;
	const bool print_r2c = false;
	const bool print_r2a = false;
	const bool print_pid = false;
	const bool print_bull = false;
	const bool print_logging = true;
	const bool print_a2o = false;
	const bool print_o2a = false;
	const bool print_o2aRaw = false;
}
// Initialize
db;

// TESTING

// Position
const bool do_posDebug = false;

// Pid Calibration
/*
Set kC and run ICR_Run.cs
*/
const float kC = 3; // critical gain [2,3,4,5]
const float pC = 1.9; // oscillation period [0,0,1.9,1.5]  
const float cal_speedSteps[4] = { 10, 20, 30, 40 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;
bool do_pidCalibration = false;

#pragma endregion


#pragma region ========= VARIABLE SETUP =========

// Pin mapping
struct PIN
{
	// Autodriver
	const int AD_CSP_R = 5;
	const int AD_CSP_F = 6;
	const int AD_RST = 7;

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
	const int Rel_EtOH = 22;
	const int Rel_Rew = 23;

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
	const int OL_RST = 16;

	// Feeder switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int FeedSwitch_Gnd = 24;
	const int FeedSwitch = 25;

	// Power off
	const int PwrOff = 45;

	// Voltage monitor
	const int BatVolt = A11;

	// Buttons
	const int Btn[3] = { A3, A2, A1 };

	/*
	Note: pins bellow are all used for external interupts
	and must all be members of the same port (PortA)
	*/

	// IR proximity sensors
	const int IRprox_Rt = 42;
	const int IRprox_Lft = 43;

	// IR detector
	const int IRdetect = 17;
}
// Initialize
pin;

// Flow/state control
struct FC
{
	String motorControl = "None"; // ["None", "Open", "MoveTo", "Bull", "Pid"]
	bool isBlockingTill = false;
	bool doQuit = false;
	bool isFirstPass = true;
	bool doStreamCheck = false;
	bool isStreaming = false;
	bool isManualSes = false;
	bool isRatIn = false;
	bool isTrackingEnabled = false;
	bool isRewarding = false;
	bool doMove = false;
	bool doRew = false;
	bool doHalt = false;
	bool isHalted = false;
	bool doBulldoze = false;
	bool doLogResend = false;
	bool doLogSend = false;
	bool isEKFReady = false;
}
// Initialize
fc;

// Print debugging
const int printQueueRows = 15;
const int printQueueCols = 200;
char printQueue[printQueueRows][printQueueCols] = { {0} };
int printQueueInd = printQueueRows;
bool doPrint = false;
bool doBlockLCDlog = false;

// Debug tracking
int cnt_droppedPacks = 0;
int cnt_overflowEvt = 0;
int cnt_timeoutEvt = 0;
int cnt_packResend = 0;

// Serial com general
const int sendQueueRows = 10;
const int sendQueueCols = 8;
byte sendQueue[sendQueueRows][sendQueueCols] = { {0 } };
int sendQueueInd = sendQueueRows - 1;
bool doPackSend = false;
const int resendMax = 3;
const int dt_sendSent = 1; // (ms)
const int dt_sendRcvd = 1; // (ms)
const int dt_logSent = 1; // (ms)
const int dt_logRcvd = 1; // (ms)
const int dt_resend = 100; // (ms)
uint32_t t_sent = millis(); // (ms)
uint32_t t_rcvd = millis(); // (ms)

// Serial from CS
struct C2R
{
	const char head = '<';
	const char foot = '>';
	const char idList[12] = {
		'+', // Setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
		'H', // halt movement
		'B', // bulldoze rat
		'I', // rat in/out
		'P', // position data
		'V', // request stream status
		'L', // request log send/resend
	};
	const static int idLng = sizeof(idList) / sizeof(idList[0]);
	uint16_t packList[idLng] = { 0 };
	int cntRepeat[idLng] = { 0 };
	char idNew = '\0';
	bool isNew = false;
	float dat[3] = { 0, 0, 0 };

	// Data vars
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
}
// Initialize
c2r;

// Serial to CS
struct R2C
{
	const char idList[15] = {
		'+', // Setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
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
	const char head = '<';
	const char foot = '>';
	const static int idLng = sizeof(idList) / sizeof(idList[0]);
	uint16_t packList[idLng] = { 0 };
	byte datList[idLng] = { 0 };
	uint16_t sendTim[idLng] = { 0 };
	bool doRcvCheck[idLng] = { false };
	int resendCnt[idLng] = { 0 };
}
// Initialize
r2c;

// Serial to other ard
struct R2A
{
	const char idList[5] = {
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
	};
	const char head = '{';
	const char foot = '}';
	uint16_t packCnt = 0;
	const static int idLng = sizeof(idList) / sizeof(idList[0]);
	uint16_t packList[idLng] = { 0 };
	byte datList[idLng] = { 0 };
	uint16_t sendTim[idLng] = { 0 };
	bool doRcvCheck[idLng] = { false };
	int resendCnt[idLng] = { 0 };
}
// Initialize
r2a;

// Serial from other ard
struct A2R
{
	const char head = '{';
	const char foot = '}';
	byte dat[1] = { 255 };
}
// Initialize
a2r;

// Start/Quit
uint32_t t_quitCmd = 0;

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
const float scaleFrontAD = 1.0375;
const byte kAcc = 60 * 2;
const byte kDec = 60 * 2;
const byte kRun = 60;
const byte kHold = 60 / 2;
int dt_blockMotor = 0; // (ms)
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 10; // (ms)

// Kalman model measures
struct EKF
{
	float RatPos = 0;
	float RobPos = 0;
	float RatVel = 0;
	float RobVel = 0;
}
ekf;

// Pid Settings
bool doIncludeTerm[2] = { true, true };
const float pidSetPoint = 42; // (cm)
const float guardDist = 4.5;
const float feedDist = 66;

// Movement
float moveToSpeed = 80;
float moveToDist = 0;
float moveToStartPos = 0;

// REWARD
int dt_blockRew = 5000; // (ms)

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
float voltNew = 0;
float batVoltArr[100] = { 0 };

// LEDs
const int trackLEDduty = 75; // value between 0 and 255
const int rewLEDduty = 15; // value between 0 and 255
const int rewLEDmin = 0; // value between 0 and 255

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
bool isLitLCD = false;

// Buttons
bool btn_doRewSolStateChange = false;
bool btn_doEtOHSolStateChange = false;
bool btn_doRew = false;
bool btn_doChangeLCDstate = false;

// Interrupts 
volatile uint32_t t_irProxDebounce = millis(); // (ms)
volatile uint32_t t_irDetectDebounce = millis(); // (ms)
volatile uint32_t t_irSyncLast = 0; // (ms)
volatile uint32_t t_sync = 0; // (ms)
volatile int dt_ir = 0;
volatile int cnt_ir = 0;
volatile bool doIRhardStop = false;
volatile bool doLogIR = false;

#pragma endregion 


#pragma region ========= CLASS DECLARATIONS =========

#pragma region ----------CLASS: POSTRACK----------
class POSTRACK
{
public:
	// VARS
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
	bool isDataNew = false;
	int dt_frame = 0;
	uint32_t t_init;
	// METHODS
	POSTRACK(uint32_t t, String obj_id, int l);
	void UpdatePos(float pos_new, uint32_t ts_new);
	float GetPos();
	float GetVel();
	void SetDat(float set_pos, uint32_t t);
	void SetPos(float set_pos, float set_laps);
};

#pragma endregion 

#pragma region ----------CLASS: PID----------
class PID
{

public:
	// VARS
	uint32_t t_lastLoop = 0;
	float dt_loop = 0;
	bool isLoopRan = false;
	float p_term = 0;
	float i_term = 0;
	float d_term = 0;
	bool isFirstRun = true;
	String mode = "Manual"; // ["Manual" "Automatic" "Halted"]
	bool isHolding4cross = false;
	bool doThrottle = false;
	bool isThrottled = false;
	float error = 0;
	float errorLast = 0;
	float errorFeeder = 0;
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
	bool is_ekfNew = false;
	float throttleAcc = 20;
	float throttleSpeedStop = 40;
	const int dt_throttle = 4000;
	uint32_t t_throttleTill = 0;
	uint32_t t_lastThrottle = millis();
	int cal_dtMin = 40; // (ms)
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
	float cal_dtLoop = 0;
	float cal_errCnt = 0;
	float cal_errSum = 0;
	float cal_errMax = 0;
	float cal_errMin = 0;
	float cal_ratPos = 0;
	float cal_ratVel = 0;
	bool cal_isPidUpdated = false;
	bool cal_isCalFinished = false;
	uint32_t t_init;
	// METHODS
	PID(uint32_t t, const float kC, const float pC, const float set_point);
	float UpdatePID();
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
	void SetLoopTime(uint32_t t);
	void PrintPID(char msg[]);
	float RunPidCalibration();
};

#pragma endregion 

#pragma region ----------CLASS: BULLDOZE----------
class BULLDOZE
{
public:
	// VARS
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

#pragma region ----------CLASS: TARGET----------
class TARGET
{
public:
	// VARS
	int targSetTimeout = 1000;
	int moveTimeout = 5000;
	uint32_t t_tryTargSetTill = 0;
	uint32_t t_tryMoveTill = 0;
	bool doAbortMove = false;
	float posRel = 0;
	float minSpeed = 0;
	const int dt_update = 10;
	uint32_t t_updateNext = 0;
	float distLeft = 0;
	float newSpeed = 0;
	float posStart = 0;
	float targ = 0;
	float offsetTarget = 0;
	float targDist = 0;
	char moveDir = 'f';
	float baseSpeed = 0;
	bool isTargSet = false;
	bool isTargReached = false;
	float haltError = 0;
	const double velCoeff[3] = {
		0.001830357142857,
		0.131160714285714,
		-2.425892857142854,
	};
	uint32_t t_init;
	// METHODS
	TARGET(uint32_t t);
	bool CompTarg(float now_pos, float targ_pos, float offset);
	float DecelToTarg(float now_pos, float now_vel, float dec_pos, float speed_min);
	float GetError(float now_pos);
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
	float zoneBounds[zoneLng][2] = { {0} };
	int zoneOccTim[zoneLng] = { 0 };
	int zoneOccCnt[zoneLng] = { 0 };
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
	float boundsRewarded[2] = { 0 };
	int occRewarded = 0;
	float lapN = 0;
	bool doArmMove = false;
	bool isArmExtended = false;
	const int armExtStps = 220;
	int armPos = 0;
	int armZone = 0;
	bool isArmStpOn = false;
	uint32_t t_init;
	// METHODS
	REWARD(uint32_t t, int t_b);
	bool StartRew(bool do_stop, bool is_button_reward = false);
	bool EndRew();
	void SetRewDur(byte zone_ind);
	void SetRewMode(String mode_str, byte arg2);
	bool CompZoneBounds(float now_pos, float rew_pos);
	bool CheckZoneBounds(float now_pos);
	void CheckFeedArm();
	void ExtendFeedArm();
	void RetractFeedArm();
	void MoveFeedArm();
	void Reset();
};

#pragma endregion 

#pragma region ----------CLASS: LOGGER----------
class LOGGER
{
public:
	// VARS
	uint32_t t_sent = 0;
	uint32_t t_rcvd = 0;
	uint32_t t_write = 0;
	int cntLogStored = 0;
	int cntLogSent = 0;
	bool isAllSent = false;
	static const int maxBytesStore = 1000;
	char rcvdArr[maxBytesStore] = { 0 };
	char mode = '\0'; // ['<', '>']
	char logFile[50] = { 0 };
	int logNum = 0;
	char logCntStr[10] = { 0 };
	int logLength = 0;
	uint16_t donePack = 0;
	uint32_t t_init;
	// METHODS
	LOGGER(uint32_t t);
	bool Setup();
	int OpenNewLog();
	bool SetToCmdMode();
	void GetCommand();
	char SendCommand(char msg[], bool do_comf = true);
	char GetReply(uint32_t timeout = 10000);
	bool SetToLogMode();
	bool StoreLogEntry(char msg[], uint32_t t = millis());
	bool SendLogEntry();
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

#pragma region -----------UNION----------
union UTAG {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (int16) 2 byte
	uint32_t i32; // (int32) 4 byte
	int i; // (int) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
};

#pragma endregion 

#pragma region ----------INITILIZE OBJECTS----------

FUSER Fuser;
UTAG U;
AutoDriver_Due AD_R(pin.AD_CSP_R, pin.AD_RST);
AutoDriver_Due AD_F(pin.AD_CSP_F, pin.AD_RST);
PixyI2C Pixy(0x54);
LCD5110 LCD(pin.Disp_CS, pin.Disp_RST, pin.Disp_DC, pin.Disp_MOSI, pin.Disp_SCK);
POSTRACK RatVT(millis(), "RatVT", 4);
POSTRACK RobVT(millis(), "RobVT", 4);
POSTRACK RatPixy(millis(), "RatPixy", 6);
PID Pid(millis(), kC, pC, pidSetPoint);
BULLDOZE Bull(millis());
TARGET Targ(millis());
REWARD Reward(millis(), dt_blockRew);
LOGGER Log(millis());

#pragma endregion 

#pragma endregion 


#pragma region ========= FUNCTION DECLARATIONS =========

// PARSE SERIAL INPUT
void GetSerial();
// PARSE CS MESSAGE
void ParseC2RData(char id);
// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char chr1 = '\0', char chr2 = '\0');
// STORE PACKET DATA TO BE SENT
void StorePacketData(char targ, char id, byte d1 = 255, uint16_t pack = 0, bool do_conf = false);
// SEND SERIAL PACKET DATA
void SendPacketData();
// CHECK IF ARD TO ARD PACKET SHOULD BE RESENT
bool CheckResend(char targ);
// CONFIGURE AUTODRIVER BOARDS
void AD_Config();
// RESET AUTODRIVER BOARDS
void AD_Reset();
// CHECK AUTODRIVER STATUS
void AD_CheckOC();
// HARD STOP
void HardStop(char called_from[]);
// IR TRIGGERED HARD STOP
void Function_IRprox_Halt();
// RUN AUTODRIVER
bool RunMotor(char dir, float speed, String agent);
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
void GetBattVolt();
// TURN LCD LIGHT ON/OFF
void ChangeLCDlight();
// CHECK FOR BUTTON INPUT
bool GetButtonInput();
// QUIT AND RESTART ARDUINO
void QuitSession();
// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t = millis());
// LOG/PRINT ERRORS
void DebugError(char msg[], uint32_t t = millis());
// LOG/PRINT DROPPED PACKET DEBUG STRING
void DebugDropped(int missed, int missed_total, int total);
// LOG/PRINT RESENT PACKET DEBUG STRING
void DebugResent(char id, uint16_t pack, int total);
// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[]);
// LOG/PRINT MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(char msg[], uint32_t t, char called_from[]);
// LOG/PRINT RECIEVED PACKET DEBUG STRING
void DebugRcvd(char from, char id, uint16_t pack);
// LOG/PRINT SENT PACKET DEBUG STRING
void DebugSent(char targ, char id, byte d1, uint16_t pack, bool do_conf);
// STORE STRING FOR PRINTING
void StoreDBPrintStr(char msg[], uint32_t t);
// PRINT DEBUG STRINGS TO CONSOLE/LCD
void PrintDebug();
// FOR PRINTING TO LCD
void PrintLCD(char msg_1[], char msg_2[] = { 0 }, char f_siz = 's');
// CLEAR LCD
void ClearLCD();
// PRINT SPECIAL CHARICTERS
char* PrintSpecialChars(char chr, bool do_show_byte = false);
// CHECK AUTODRIVER BOARD STATUS
int CheckAD_Status(uint16_t stat_reg, String stat_str);
// CHECK LOOP TIME AND MEMORY
void CheckLoop(int free_mem);
// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size);
// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink(int n_blinks, int dt_led = 0);
// BLICK LEDS WHEN RAT FIRST DETECTED
void RatInBlink();
// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt();
// DETECT IR SYNC EVENT
void Interupt_IR_Detect();

#pragma endregion


#pragma region ========= CLASS DEFINITIONS =========

#pragma region ----------CLASS: POSTRACK----------

POSTRACK::POSTRACK(uint32_t t, String obj_id, int l)
{
	this->t_init = t;
	this->objID = obj_id;
	this->nSamp = l;
	for (int i = 0; i < l; i++) this->posArr[i] = 0.0f;
}

void POSTRACK::UpdatePos(float pos_new, uint32_t ts_new)
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
		isDataNew = false;
	}
	else
	{
		isDataNew = true;

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

float POSTRACK::GetPos()
{
	isDataNew = false;
	return posNow;
}

float POSTRACK::GetVel()
{
	return velNow;
}

void POSTRACK::SetDat(float set_pos, uint32_t t)
{
	// Compute ts
	uint32_t ts = t_tsNow + (t - t_msNow);

	// Update pos
	UpdatePos(set_pos, ts);
}

void POSTRACK::SetPos(float set_pos, float set_laps)
{
	posNow = set_pos;
	sampCnt = 0;
	isDataNew = false;
	if (set_pos != -100)
	{
		nLaps = set_laps;
	}
}

#pragma endregion 

#pragma region ----------CLASS: PID----------

PID::PID(uint32_t t, const float kC, const float pC, const float set_point)
{
	this->t_init = t;
	this->kP = 0.6 * kC; // proportional constant
	this->kI = 2 * kP / pC; // integral constant
	this->kD = kP*pC / 8; // derivative constant
	this->setPoint = set_point;
}

float PID::UpdatePID()
{

	// Wait till ekf ready
	if (!fc.isEKFReady)
	{
		return -1;
	}
	else
	{

		// Compute error 
		error = ekf.RatPos - (ekf.RobPos + setPoint);
		errorFeeder = ekf.RatPos - (ekf.RobPos + feedDist);

		// Check if motor is open
		CheckMotorControl();

		// Check throttling 
		CheckThrottle();

		// Check if in auto mode
		if (mode != "Automatic")
		{
			return -1;
		}
		else
		{

			// Check if rat stopped behind setpoint
			if (ekf.RatVel < 1 && error < -15 && !isHolding4cross)
			{
				// halt running
				return runSpeed = 0;
			}

			// Check if throttling
			SetThrottle();

			// Check for setpoint crossing
			CheckSetpointCrossing();

			// Update Pid and speed
			if (
				!is_ekfNew || // New EKF data 
				isHolding4cross // wait for setpoint pass
				)
			{
				return -1;
			}
			else
			{

				// Re-compute error 
				error = ekf.RatPos - (ekf.RobPos + setPoint);

				// Compute new integral
				if (doIncludeTerm[0])
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
				if (doIncludeTerm[1])
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
				runSpeed = ekf.RobVel + velUpdate;

				// Keep speed in range [0, speedMax]
				if (runSpeed > speedMax) runSpeed = speedMax;
				else if (runSpeed < 0) runSpeed = 0;

				errorLast = error;
				isLoopRan = true;
				is_ekfNew = false;

				if (isFirstRun)
				{
					PrintPID("pid: first run");
					isFirstRun = false;
				}

				// Return new run speed
				return runSpeed;
			}

		}

	}

}

void PID::Run(char called_from[])
{
	// Take motor control
	SetMotorControl("Pid", "Pid.Run");

	// Reset
	Reset();
	mode = "Automatic";

	// Tell ard pid is running
	StorePacketData('a', 'p', 1);

	// Print event
	char str[100] = { 0 };
	sprintf(str, "pid: run [%s]", called_from);
	PrintPID(str);
}

void PID::Stop(char called_from[])
{
	if (fc.motorControl == "Pid")
	{
		// Stop movement
		RunMotor('f', 0, "Pid");

		// Set run speed
		runSpeed = 0;

		// Give over control
		SetMotorControl("Open", "Pid.Stop");
	}

	// Tell ard pid is stopped
	if (
		// Dont send if rewarding
		!fc.isRewarding
		)
	{
		StorePacketData('a', 'p', 0);
	}

	// Set mode
	mode = "Manual";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "pid: stop [%s]", called_from);
	PrintPID(str);
}

void PID::Hold(char called_from[])
{
	// Call Stop
	Stop("Pid.Hold");

	// But set mode to "Hold"
	mode = "Hold";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "pid: hold [%s]", called_from);
	PrintPID(str);
}

void PID::Reset()
{
	integral = 0;
	t_lastLoop = millis();
	isHolding4cross = true;
	doThrottle = true;
}

void PID::SetThrottle()
{
	if (doThrottle)
	{
		// Only run if rat ahead of setpoint
		if (
			error > 5 &&
			millis() > t_lastThrottle + 5000
			)
		{
			// Change acc to rat pos
			AD_Reset();
			AD_R.setAcc(throttleAcc*cm2stp);
			AD_F.setAcc(throttleAcc*cm2stp);
			delayMicroseconds(100);

			// Set time to throttle till
			t_throttleTill = millis() + dt_throttle;

			// Set flags
			isThrottled = true;
			doThrottle = false;

			char str[100] = { 0 };
			sprintf(str, "pid: throttle acc to %0.2fcm/sec", throttleAcc);
			PrintPID(str);
		}
		else
		{
			doThrottle = false;
		}
	}
}

void PID::CheckThrottle()
{
	if (isThrottled)
	{

		// Check for criteria
		if (
			millis() >= t_throttleTill ||
			ekf.RatVel > throttleSpeedStop ||
			error < 0
			)
		{
			// Set acc back to normal
			AD_Reset();
			AD_R.setAcc(maxAcc*cm2stp);
			AD_F.setAcc(maxAcc*cm2stp);
			delayMicroseconds(100);

			// Store time
			t_lastThrottle = millis();

			// Reset flag
			isThrottled = false;
			PrintPID("pid: finished throttle");
		}
	}
}

void PID::CheckMotorControl()
{
	// Check if motor control available
	if ((fc.motorControl == "Pid" || fc.motorControl == "Open") &&
		mode == "Hold")
	{
		// Print taking conrol
		PrintPID("pid: take motor conrol [Pid.CheckMotorControl]");

		// Run pid
		Run("Pid.CheckMotorControl");
	}
	else if ((fc.motorControl != "Pid" && fc.motorControl != "Open") &&
		mode == "Automatic")
	{
		// Print taking conrol
		PrintPID("pid: give up motor conrol [Pid.CheckMotorControl]");

		// Hold pid
		Hold("Pid.CheckMotorControl");
	}
}

void PID::CheckSetpointCrossing()
{
	// Check if rat has moved in front of setpoint
	if (isHolding4cross && error > 0)
	{
		isHolding4cross = false;
		PrintPID("pid: crossed setpoint");
	}
}

void PID::CheckEKF(uint32_t t)
{
	if (!fc.isEKFReady)
	{
		if ((t - t_ekfStr) > dt_ekfSettle)
		{
			fc.isEKFReady = true;
		}
	}
}

void PID::ResetEKF(char called_from[])
{
	fc.isEKFReady = false;
	t_ekfStr = millis();

	// Print event
	char str[100] = { 0 };
	sprintf(str, "pid: reset ekf [%s]", called_from);
	PrintPID(str);
}

void PID::SetLoopTime(uint32_t t)
{
	is_ekfNew = true;
	if (isLoopRan)
	{
		dt_loop = (float)(t - t_lastLoop) / 1000.0f;
		isLoopRan = false;
		t_lastLoop = t;
	}
}

void PID::PrintPID(char msg[])
{
	// Add to print queue
	if (db.print_pid && (db.Console || db.LCD)) {
		StoreDBPrintStr(msg, millis());
	}
	// Add to log queue
	if (db.log_pid && db.Log) {
		Log.StoreLogEntry(msg, millis());
	}
}

float PID::RunPidCalibration()
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
		else if (!is_ekfNew || ekf.RobPos <= 0)
		{
			return -1;
		}
		else if (millis() > t_lastLoop + cal_dtMin)
		{

			// Setup stuff
			if (cal_ratPos == 0)
			{
				cal_ratPos = ekf.RobPos + setPoint;
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

			// Compute Pid
			cal_ratVel = cal_speedSteps[cal_stepNow];
			cal_ratPos += cal_ratVel * (cal_dtLoop / 1);

			// Compute error 
			error = cal_ratPos - (ekf.RobPos + setPoint);
			cal_errNow = error;

			// Compute new terms
			p_term = kC*error;

			// Get new run speed
			runSpeed = ekf.RobVel + p_term;

			// Keep speed in range [0, speedMax]
			if (runSpeed > speedMax) runSpeed = speedMax;
			else if (runSpeed < 0) runSpeed = 0;

			// Catch occilation edge
			if (cal_errLast > 0 && error < 0)
			{
				cal_t_PcLast = cal_t_PcNow;
				cal_t_PcNow = millis();
				// Skip first period
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
			isLoopRan = true;
			is_ekfNew = false;

			return runSpeed;
		}

	}

}

#pragma endregion 

#pragma region ----------CLASS: BULLDOZE----------

BULLDOZE::BULLDOZE(uint32_t t)
{
	this->t_init = t;
}

void BULLDOZE::UpdateBull()
{
	// Check who has motor control
	CheckMotorControl();

	// Bulldoze is on
	if (state == "On")
	{

		// Update when ready
		if (
			fc.isEKFReady &&
			millis() > t_loopNext)
		{

			// Update rat pos
			posNow = ekf.RatPos;
			guardPos = ekf.RobPos + guardDist;

			// Get targ_dist traveled
			distMoved = posNow - posCheck;

			// Check for movement
			isMoved = distMoved >= moveMin ? true : false;

			// Check if rat passed reset
			float error = ekf.RatPos - (ekf.RobPos + pidSetPoint);
			isPassedReset = error > 1 ? true : false;

			// Check time
			isTimeUp = millis() > t_bullNext ? true : false;

			// Check if has not moved in time
			if (!isMoved)
			{
				// Bulldoze him!
				if (isTimeUp &&
					mode == "Inactive")
				{
					Run("BULLDOZE.UpdateBull");
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
				if (isPassedReset &&
					mode == "Active" &&
					bDelay != 0)
				{
					Stop("BULLDOZE.UpdateBull");
				}
			}

			// Set next loop time
			t_loopNext = millis() + dt_loop;
		}
	}
}

void BULLDOZE::Reinitialize(byte del, byte spd, char called_from[])
{
	bSpeed = (int)spd;
	bDelay = (int)del * 1000;
	t_bullNext = millis() + bDelay;
	posCheck = ekf.RatPos;

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: reinitialize [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Run(char called_from[])
{
	// Take control
	SetMotorControl("Bull", "BULLDOZE.Run");

	// Start bulldozer
	RunMotor('f', bSpeed, "Bull");

	// Tell ard bull is running
	StorePacketData('a', 'b', 1);

	// Set mode
	mode = "Active";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: run [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Stop(char called_from[])
{
	// Stop movement
	RunMotor('f', 0, "Bull");

	// Give over control
	if (fc.motorControl == "Bull")
	{
		SetMotorControl("Open", "BULLDOZE.Stop");
	}

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Tell ard bull is stopped
	StorePacketData('a', 'b', 0);

	// Set mode
	mode = "Inactive";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: stop [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::TurnOn(char called_from[])
{
	// Change state
	state = "On";

	// Reset 
	Reset();

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: on [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::TurnOff(char called_from[])
{
	// Change state
	state = "Off";

	// Stop bulldozer if running
	if (mode == "Active")
	{
		// Stop bull
		Stop("BULLDOZE.TurnOff");
	}

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: off [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Hold(char called_from[])
{
	// Change state
	state = "Hold";

	// Stop running
	if (mode == "Active")
	{
		// Run stop bulldozer
		Stop("BULLDOZE.Hold");

		// Set mode back to active for later
		mode = "Active";
	}

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: hold [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Resume(char called_from[])
{
	// Set state back to "On"
	state = "On";

	// Reset 
	Reset();

	// Print event
	char str[100] = { 0 };
	sprintf(str, "bull: resume [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Reset()
{
	// Set mode
	mode = "Inactive";

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Reset check pos
	posCheck = ekf.RatPos;
}

void BULLDOZE::CheckMotorControl()
{
	if ((fc.motorControl == "Bull" || fc.motorControl == "Pid" || fc.motorControl == "Open") &&
		state == "Hold")
	{
		// Turn bull on
		Resume("BULLDOZE.CheckMotorControl");
	}
	else if ((fc.motorControl != "Bull" && fc.motorControl != "Pid" && fc.motorControl != "Open") &&
		state == "On")
	{
		// Turn bull off
		Hold("BULLDOZE.CheckMotorControl");
	}
}

void BULLDOZE::PrintBull(char msg[])
{
	// Add to print queue
	if (db.print_bull && (db.Console || db.LCD)) {
		StoreDBPrintStr(msg, millis());
	}
	// Add to log queue
	if (db.log_bull && db.Log) {
		Log.StoreLogEntry(msg, millis());
	}
}

#pragma endregion 

#pragma region ----------CLASS: TARGET----------

TARGET::TARGET(uint32_t t)
{
	this->t_init = t;
}

bool TARGET::CompTarg(float now_pos, float targ_pos, float offset)
{
	// Local vars
	int circ = 0;
	int pos = 0;

	// Copy to public vars
	posStart = now_pos;
	targ = targ_pos;

	// Get abort timeout
	if (t_tryTargSetTill == 0)
		t_tryTargSetTill = millis() + targSetTimeout;
	// Check if time out reached
	if (millis() > t_tryTargSetTill) {
		doAbortMove = true;
		return false;
	}

	// Run only if targ not set
	if (!isTargSet)
	{
		// Check pos data is ready
		if (fc.isEKFReady)
		{
			// Compute target targ_dist and move_dir
			offsetTarget = targ + offset;
			offsetTarget = offsetTarget < 0 ? offsetTarget + (140 * PI) : offsetTarget;

			// Current relative pos on track
			circ = (int)(140 * PI * 100);
			pos = (int)(now_pos * 100);
			posRel = (float)(pos % circ) / 100;

			// Diff and absolute targ_dist
			targDist = offsetTarget - posRel;
			targDist = targDist < 0 ? targDist + (140 * PI) : targDist;

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

float TARGET::DecelToTarg(float now_pos, float now_vel, float dec_pos, float speed_min)
{
	// Run if targ not reached
	if (!isTargReached)
	{

		// Get abort timeout
		if (t_tryMoveTill == 0)
			t_tryMoveTill = millis() + moveTimeout;
		// Check if time out reached
		if (millis() > t_tryMoveTill) {
			doAbortMove = true;
			return false;
		}

		// Compute remaining targ_dist
		distLeft = targDist - (now_pos - posStart);
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

		// TARGET reached
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

float TARGET::GetError(float now_pos)
{
	// Local vars
	int diam = 0;
	int pos = 0;

	// Current relative pos on track
	diam = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posRel = (float)(pos % diam) / 100;

	// TARGET error
	return offsetTarget - posRel;
}

void TARGET::Reset()
{
	isTargSet = false;
	isTargReached = false;
	doAbortMove = false;
	t_tryTargSetTill = 0;
	t_tryMoveTill = 0;
}

#pragma endregion 

#pragma region ----------CLASS: REWARD----------

REWARD::REWARD(uint32_t t, int t_b)
{
	this->t_init = t;
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

bool REWARD::StartRew(bool do_stop, bool is_button_reward)
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

	// Store and send packet imediately
	StorePacketData('a', 'r', durationByte);
	SendPacketData();

	// Compute reward end time
	t_closeSol = millis() + duration;

	// Turn on reward LED
	analogWrite(pin.RewLED_R, round(rewLEDduty*0.75));
	analogWrite(pin.RewLED_C, rewLEDduty);
	// Open solenoid
	digitalWrite(pin.Rel_Rew, HIGH);

	// Print to LCD for manual rewards
	if (isButtonReward)
	{
		PrintLCD("REWARDING...");
	}
	else
	{
		char str[100] = { 0 };
		sprintf(str, "REWARDING FOR %dms...", duration);
		DebugFlow(str);
	}

	// Set flags
	isRewarding = true;

	// indicate reward in progress
	return isRewarding;

}

bool REWARD::EndRew()
{

	bool reward_finished = false;

	if (millis() > t_closeSol)
	{

		// Close solenoid
		digitalWrite(pin.Rel_Rew, LOW);

		// Turn off reward LED
		analogWrite(pin.RewLED_R, rewLEDmin);
		analogWrite(pin.RewLED_C, rewLEDmin);

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

void REWARD::SetRewDur(byte zone_ind)
{
	// Set duration
	duration = zoneRewDurs[zone_ind];
	durationByte = (byte)(duration / 10);

	// Save zone ind
	zoneIndByte = zone_ind;
}

void REWARD::SetRewMode(String mode_str, byte arg2)
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

bool REWARD::CompZoneBounds(float now_pos, float rew_pos)
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

bool REWARD::CheckZoneBounds(float now_pos)
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

						// REWARD at this pos
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

void REWARD::CheckFeedArm()
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
		// TARGET reached
		else
		{
			// Unstep motor
			if (digitalRead(pin.ED_STP) == HIGH)
			{
				digitalWrite(pin.ED_STP, LOW);
				isArmStpOn = false;
			}
			// Sleep motor
			digitalWrite(pin.ED_SLP, LOW);

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
	else if (digitalRead(pin.ED_SLP) == HIGH) {
		// Sleep motor
		digitalWrite(pin.ED_SLP, LOW);
	}

}

void REWARD::ExtendFeedArm()
{
	if (!isArmExtended)
	{
		t_retractArm = millis() + dt_block;
		armZone = armExtStps;
		doArmMove = true;
	}
}

void REWARD::RetractFeedArm()
{
	if (isArmExtended)
	{
		armZone = 0;
		doArmMove = true;
	}
}

void REWARD::MoveFeedArm()
{
	// Local vars
	static uint32_t t_step_high = millis();
	static uint32_t t_step_low = millis();

	// Wake motor
	if (digitalRead(pin.ED_SLP) == LOW)
	{
		digitalWrite(pin.ED_SLP, HIGH);
	}

	// Step motor
	if (!isArmStpOn)
	{

		// Extend arm
		if (armPos < armZone)
		{
			digitalWrite(pin.ED_DIR, LOW); // extend
		}

		// Retract arm
		else
		{
			if (digitalRead(pin.FeedSwitch) == HIGH)
			{
				digitalWrite(pin.ED_DIR, HIGH); // retract
			}
			// Home pos reached
			else
			{
				// Take presure off botton
				armPos = -20;
				isArmExtended = false;
			}
		}

		// Check for min time ellapsed
		if (millis() - t_step_low < 1)
			return;

		// Set step high
		digitalWrite(pin.ED_STP, HIGH);
		t_step_high = millis();

		// Incriment step
		armPos++;

		// Set flag
		isArmStpOn = true;
	}
	// Unstep motor
	else
	{
		// Check for min time ellapsed
		if (millis() - t_step_high < 1)
			return;

		// Set step low
		digitalWrite(pin.ED_STP, LOW);
		t_step_low = millis();

		// Set flag
		isArmStpOn = false;
	}
}

void REWARD::Reset()
{
	// Log zone info
	if (mode == "Free" || mode == "Cue")
	{
		char str1[200] = "ZONE OCC:";
		char str2[200] = "ZONE CNT:";
		for (int i = zoneMin; i <= zoneMax; i++)
		{
			char ss1[50];
			sprintf(ss1, " z%d=%dms", i, zoneOccTim[i]);
			strcat(str1, ss1);
			char ss2[50];
			sprintf(ss2, " z%d=%d", i, zoneOccCnt[i]);
			strcat(str2, ss2);
		}
		DebugFlow(str1);
		DebugFlow(str2);
	}

	// Reset flags
	mode = "None";
	isRewarding = false;
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

#pragma endregion 

#pragma region ----------CLASS: LOGGER----------

LOGGER::LOGGER(uint32_t t)
{
	this->t_init = t;
}

bool LOGGER::Setup()
{
	/*
	NOTE:
	config.txt settings:
	57600,26,3,2,0,0,0
	*/

	// Local vars
	bool pass = false;
	byte match = '\0';
	char str[100] = { 0 };

	// Start serial
	Serial3.begin(57600);

	// Reset OpenLog
	digitalWrite(pin.OL_RST, HIGH);
	delay(100);
	digitalWrite(pin.OL_RST, LOW);
	delay(100);
	match = GetReply(5000);
	pass = match == '>' || match == '<' ? true : false;

	// Bail if setup failed
	if (!pass)
		return false;

	// Set to command mode;
	if (match == '<')
		pass = SetToCmdMode();

	// Turn off verbose mode and echo mode
	SendCommand("verbose off\r");
	SendCommand("echo off\r");

	// Print config.txt
	sprintf(str, "read config.txt\r");
	if (SendCommand(str) == '!')
		pass = false;


	return pass;
}

int LOGGER::OpenNewLog()
{
	// Local vars
	byte match = 0;
	char str[100] = { 0 };

	// Check/cd to log directory
	if (SendCommand("cd LOGS\r") == '!')
	{
		if (SendCommand("md LOGS\r") == '!')
			return 0;
		if (SendCommand("cd LOGS\r") == '!')
			return 0;
	}

	// Check for log count file
	if (SendCommand("read LOGCNT.TXT\r") == '!')
	{
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!')
			return 0;
	}
	// Store count
	else
	{
		logNum = atoi(logCntStr) + 1;
	}

	// Check if more than 1000 logs saved
	if (logNum > 1000)
	{
		// Step out of directory
		if (SendCommand("cd ..\r") == '!')
			return 0;
		// Delete directory
		if (SendCommand("rm -rf LOGS\r") == '!')
			return 0;
		// Rerun 
		OpenNewLog();
	}

	// Update count
	if (SendCommand("write LOGCNT.TXT\r") == '!')
		return 0;
	// write int string
	sprintf(str, "{{%d}}\r", logNum);
	SendCommand(str);
	// exit with empty line
	str[0] = '\r';
	str[1] = '\0';
	if (SendCommand(str) == '!')
		return 0;

	// Create new log file
	sprintf(logFile, "LOG%05u.CSV", logNum);

	// Begin logging to this file
	SetToLogMode();

	// Write first log entry
	sprintf(str, "Begin %s", logFile);
	StoreLogEntry(str);

	// Return log number
	return logNum;

}

bool LOGGER::SetToCmdMode()
{
	// Local vars
	bool pass = false;
	char match = '\0';
	char rstArr[3] = { 26,26,26 };
	uint32_t t_timeout = millis() + 10000;

	// Check if already in cmd mode
	if (mode == '>')
		return true;

	// Add delay if log just sent
	if ((millis() - t_write) < 10)
		delay(10);

	//Send "$$$" command mode
	while (
		millis() < t_timeout &&
		match != '>'
		) {
		match = SendCommand(rstArr);
	}
	pass = match == '>' ? true : false;

	// Pause to let OpenLog get its shit together
	if (pass) {
		delay(100);
	}
	// Log/print error
	else {
		char str[100];
		sprintf(str, "!!ERROR!! LOGGER::SetToCmdMode Failed: mode=%c", mode);
		DebugError(str);
	}

	return pass;
}

void LOGGER::GetCommand()
{
	// Local vars
	char str[100] = { 0 };
	static int byte_ind_out = 0;

	// Get terminal command
	if (SerialUSB.available() > 0)
	{
		byte_ind_out = 0;
		while (SerialUSB.available() > 0)
		{
			str[byte_ind_out] = SerialUSB.read();
			byte_ind_out++;
			delayMicroseconds(100);
		}

		// Print command
		str[byte_ind_out] = '\0';
		SendCommand(str);

	}
}

char LOGGER::SendCommand(char msg[], bool do_comf)
{
	// Send
	Serial3.write(msg);
	t_sent = millis();
	char m = '\0';

	// Print sent
	if (db.print_a2o)
	{
		PrintLOGGER("SENT[=======================", true);
		for (int i = 0; i < strlen(msg) + 1; i++)
			PrintLOGGER(PrintSpecialChars(msg[i]));
		PrintLOGGER("\n=======================]SENT\n\n");
	}

	// Get confirmation
	if (do_comf)
		m = GetReply();
	else m = mode;

	// Return mode
	return m;
}

char LOGGER::GetReply(uint32_t timeout)
{
	// Local vars
	char str[100] = { 0 };
	uint32_t t_start = millis();
	uint32_t t_timeout = millis() + timeout;
	int dat_ind[2] = { 0,0 };
	int arr_ind = -1;
	char cmd_reply = '\0';
	bool pass = false;

	// Wait for new data
	while (
		Serial3.available() == 0 &&
		millis() < t_timeout
		);
	t_rcvd = millis();

	// Check for match byte
	if (db.print_o2a)
		PrintLOGGER("RCVD_FORMATED[===================", true);
	while (
		!pass &&
		millis() < t_timeout
		)
	{
		// Get new data
		if (Serial3.available() > 0)
		{
			// Get next byte
			char c = Serial3.read();

			// Print new byte
			if (db.print_o2a)
				PrintLOGGER(str);

			// Itterate byte ind
			if (
				arr_ind < maxBytesStore - 2 ||
				rcvdArr[arr_ind] == '!' ||
				rcvdArr[arr_ind] == '<' ||
				rcvdArr[arr_ind] == '>'
				)
				arr_ind++;

			// Store data
			rcvdArr[arr_ind] = c;
		}
		// Make sure message finished
		else if (arr_ind >= 0)
		{
			bool p_arr[3] = { false, false, false };
			for (int i = arr_ind; i >= 0; i--)
			{
				// Check that certain specific comnination of chars recieved
				p_arr[0] = rcvdArr[i] == '\r' || rcvdArr[i] == '1' || rcvdArr[i] == '~' ? true : p_arr[0];
				p_arr[1] = rcvdArr[i] == '\n' || rcvdArr[i] == '2' || rcvdArr[i] == '~' ? true : p_arr[1];
				p_arr[2] = rcvdArr[i] == '>' || rcvdArr[i] == '<' ? true : p_arr[2];
			}
			if (p_arr[0] && p_arr[1] && p_arr[2])
				pass = true;
		}
	}
	// Set null terminator
	rcvdArr[arr_ind + 1] = '\0';

	if (db.print_o2a)
		PrintLOGGER("\n===================]RCVD_FORMATED\n");

	// Print formated string
	if (db.print_o2aRaw)
	{
		PrintLOGGER("RCVD_RAW[==============", true);
		for (int i = 0; i <= arr_ind + 1; i++)
		{
			//sprintf(str, "[%d]\'%s\'", rcvdArr[i], PrintSpecialChars(rcvdArr[i]));
			sprintf(str, "\'%s\'", PrintSpecialChars(rcvdArr[i]));
			PrintLOGGER(str);
		}
		PrintLOGGER("\n==============]RCVD_RAW\n");
	}

	// Save values
	for (int i = 0; i <= arr_ind; i++)
	{
		// Save cmd 
		if (
			rcvdArr[i] == '!' ||
			rcvdArr[i] == '<' ||
			rcvdArr[i] == '>'
			)
			cmd_reply = cmd_reply != '!' ? rcvdArr[i] : cmd_reply;

		// Check for data start indeces
		dat_ind[0] =
			dat_ind[0] == 0 && rcvdArr[i - 1] == '{' && rcvdArr[i] == '{' ?
			i + 1 : dat_ind[0];
		// Check for end indeces
		if (
			dat_ind[0] > 0 &&
			rcvdArr[i] == '}' &&
			rcvdArr[i + 1] == '}'
			)
			dat_ind[1] = i - 1;
	}

	// Get data
	if (
		dat_ind[0] != 0 &&
		dat_ind[1] != 0
		)
	{
		int ii = 0;
		for (int i = dat_ind[0]; i <= dat_ind[1]; i++)
		{
			logCntStr[ii] = rcvdArr[i];
			ii++;
		}
		logCntStr[ii + 1] = '\0';
	}

	// Get current mode
	mode = cmd_reply == '>' || cmd_reply == '<' ? cmd_reply : mode;

	// Print mode and round trip time
	if (db.print_o2a || db.print_o2aRaw) {
		sprintf(str, "mode=\'%c\' reply=\'%c\' dt=%dms\n\n", mode, cmd_reply, t_rcvd - t_sent);
		PrintLOGGER(str);
	}

	// Log/print error
	if (!pass) {
		sprintf(str, "!!ERROR!! LOGGER::GetReply Timedout: dt=%d bytes_read=%d", timeout, arr_ind);
		DebugError(str);
	}

	// Return cmd 
	return cmd_reply;
}

bool LOGGER::SetToLogMode()
{
	// Local varsFINISHED: OpenLog Setup
	char str[100] = { 0 };
	bool pass = false;

	// Check if already in log mode
	if (mode == '<')
		return true;

	// Send append file command
	sprintf(str, "append %s\r", logFile);
	if (SendCommand(str) != '!')
		pass = true;

	// Pause to let OpenLog get its shit together
	if (pass) {
		delay(100);
	}
	// Log/print error
	else {
		sprintf(str, "!!ERROR!! LOGGER::SetToLogMode Failed: mode=%c", mode);
		DebugError(str);
	}

	return pass;
}

bool LOGGER::StoreLogEntry(char msg[], uint32_t t)
{
	/*
	STORE LOG DATA FOR CS
	FORMAT: "[log_cnt],ts_ms,message,\r\n"
	*/

	// Local vars
	uint32_t t_timeout = millis() + 1000;
	char msg_out[200] = { 0 };
	char str[200];
	uint32_t t_m = 0;

	// Send
	if (mode == '<')
	{
		// Get sync correction
		t_m = t - t_sync;

		// itterate count
		cntLogStored++;

		// Put it all together
		sprintf(msg_out, "[%d],%lu,%s\r\n", cntLogStored, t_m, msg);

		// Send it
		Serial3.write(msg_out);
		t_write = millis();

		// Print stored log
		if (db.print_logging) {
			msg_out[strlen(msg_out) - 1] = '\0';
			sprintf(str, "stored log %d: \"%s\"", cntLogStored, msg_out);
			StoreDBPrintStr(str, millis());
		}

		// Return success
		return true;
	}
	else
		return false;
}

bool LOGGER::SendLogEntry()
{
	// Local vars
	const int timeout = 1000;
	uint32_t t_timeout = millis() + timeout;
	const static int read_len = 70;
	static bool is_msg_ready = false;
	static int log_ind[2] = { 0, read_len };
	static char msg_out[200] = { 0 };
	static int msg_lng = 0;
	static int log_num = 0;
	char num_str[10] = { 0 };
	char msg_in[200] = { 0 };
	int log_bound[2] = { 0 };
	char str[200] = { 0 };
	byte chksum = 0;
	bool pass = false;

	// Bail if all logs sent
	if (isAllSent)
		return false;

	// Make sure in command mode
	if (!SetToCmdMode())
		return false;

	// Check if resending last log
	if (
		!fc.doLogResend &&
		!is_msg_ready
		)
	{
		// Send command to read
		sprintf(str, "read %s %d %d\r", logFile, log_ind[0], log_ind[1]);
		SendCommand(str, false);

		// Get entry bounds
		int read_ind = 0;
		while (millis() < t_timeout)
		{
			if (Serial3.available() > 0)
			{
				// Get next byte
				msg_in[read_ind] = Serial3.read();

				// Check for start
				if (log_bound[0] == 0 && read_ind >= 2)
					log_bound[0] =
					msg_in[read_ind - 2] == '\r' && msg_in[read_ind - 1] == '\n' ?
					read_ind : log_bound[0];

				// Check for end
				else if (log_bound[0] != 0)
					log_bound[1] =
					msg_in[read_ind - 1] == '\r' && msg_in[read_ind] == '\n' ?
					read_ind - 2 : log_bound[1];

				// Break
				if (log_bound[1] != 0) {
					pass = true;
					break;
				}

				// Itterate
				read_ind++;
			}
		}

		// Check if timedout
		if (!pass) {
			// Print error
			sprintf(str, "!!ERROR!! LOGGER::SendLogEntry Timedout: dt-%dms bytes_read=%d", timeout, read_ind);
			DebugError(str);

			// Set flag so read is attempted again
			is_msg_ready = false;
			return false;
		}

		// Get length
		chksum = log_bound[1] - log_bound[0] + 1;

		// Get log number
		int ii = 0;
		for (int i = strcspn(msg_in, "[") + 1; i < strcspn(msg_in, "]"); i++) {
			num_str[ii] = msg_in[i];
			ii++;
		}
		num_str[ii + 1] = '\0';
		log_num = atoi(num_str);

		// Check if log numbers dont match up
		if (cntLogSent + 1 != log_num) {
			sprintf(str, "!!ERROR!! LOGGER::SendLogEntry Wrong/Corrupted Log: cnt=%d read=%d", cntLogSent, log_num);
			DebugError(str);

			// TEMP
			SerialUSB.print("\nmsg_in-------------------\n");
			for (int i = 0; i < strlen(msg_in) + 1; i++)
			{
				sprintf(str, "%s", PrintSpecialChars(msg_in[i]));
				SerialUSB.print(str);
			}
			SerialUSB.print("-------------------------\n");

			// Set flag so read is attempted again
			is_msg_ready = false;
			return false;
		}

		// Load byte array
		msg_lng = 0;
		// head
		msg_out[msg_lng++] = r2c.head;
		// id
		msg_out[msg_lng++] = 'U';
		// checksum
		msg_out[msg_lng++] = chksum;
		// msg
		for (int i = log_bound[0]; i <= log_bound[1]; i++)
			msg_out[msg_lng++] = msg_in[i];
		// foot
		msg_out[msg_lng++] = r2c.foot;
		// null
		msg_out[msg_lng] = '\0';

		// Update log ind to exclude terminating '\r' '\n'
		log_ind[0] = log_ind[0] + chksum + 2;
		log_ind[1] = log_ind[0] + read_len;

		//// TEMP

		//PrintLOGGER("\n\nmsg_in\n");
		//for (int i = 0; i < strlen(msg_in) + 1; i++)
		//{
		//	sprintf(str, "%s", PrintSpecialChars(msg_in[i]));
		//	PrintLOGGER(str);
		//}

		//PrintLOGGER("\n\nmsg_out\n");
		//for (int i = 0; i < strlen(msg_out) + 1; i++)
		//{
		//	sprintf(str, "%s", PrintSpecialChars(msg_out[i]));
		//	PrintLOGGER(str);
		//}
		//PrintLOGGER("\n\n");

		//sprintf(str, "\n\npass=%s\nlog_bound[0]=%d\nlog_bound[1]=%d\nread_ind=%d\n\n", pass ? "true" : "false", log_bound[0], log_bound[1], read_ind);
		//PrintLOGGER(str);


	}

	// Exit if not ready to send
	if (
		millis() < t_sent + dt_logSent ||
		millis() < t_rcvd + dt_logRcvd ||
		Serial1.availableForWrite() < msg_lng + 10 ||
		Serial1.available() > 0
		) {
		// Resend on next pass
		is_msg_ready = true;
		return false;
	}

	//TEMP
	//PrintLOGGER(msg_out, true);

	// Send data
	Serial1.write(msg_out, msg_lng);
	delay(100);

	// Itterate count
	cntLogSent++;

	// Print send status
	if (db.print_logging) {
		sprintf(str, "sent log: cnt=%d/%d range=%d-%d msg=\"%s\"", cntLogSent, cntLogStored, log_ind[0], log_ind[1], msg_out);
		StoreDBPrintStr(str, millis());
	}

	// Print error if log was resent
	if (fc.doLogResend) {
		sprintf(str, "!!ERROR!! LOGGER::SendLogEntry Resent: cnt=%d/%d range=%d-%d msg=\"%s\"", cntLogSent, cntLogStored, log_ind[0], log_ind[1], msg_out);
		DebugError(str);
	}

	// TEMP
	// Make sure all read in
	pass = false;
	while (millis() < t_timeout) {
		if (Serial3.available() > 0) {
			if (Serial3.read() == '>') {
				pass = true;
				break;
			}
		}
	}
	if (!pass)
		SerialUSB.print("!!ERROR!! LOGGER::SendLogEntry Failed to Empty Buffer\n");

	// Reset flags
	is_msg_ready = false;
	fc.doLogSend = false;
	fc.doLogResend = false;

	// Check if all sent
	if (cntLogSent == cntLogStored)
	{
		// Send confirm done
		StorePacketData('c', 'D', 255, donePack);
		DebugFlow("LOG SEND COMPLETE");
		fc.doLogSend = false;

		// Set flag
		isAllSent = true;
	}
}

void LOGGER::PrintLOGGER(char msg[], bool start_entry)
{
	// Local vars
	char str[10] = { 0 };
	uint32_t t_m = 0;
	float t_s = 0;

	// Print time
	if (start_entry) {
		t_m = millis() - t_sync;
		t_s = t_m > 0 ? (float)(t_m) / 1000.0f : 0;
		sprintf(str, "[%0.2fs] ", t_s);
		SerialUSB.print(str);
	}

	// Print message
	SerialUSB.print(msg);

	// Print new line
	if (start_entry)
		SerialUSB.print('\n');
}

#pragma endregion 

#pragma endregion 


#pragma region ========= FUNCTION DEFINITIONS =========

#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
void GetSerial()
{
	// Local vars
	byte buff = 0;
	char head = '\0';
	char id = '\0';
	int id_ind = 0;
	static uint16_t pack_last = 0;
	static int pack_tot = 0;
	uint16_t pack = 0;
	char foot = '\0';
	bool do_conf;
	bool is_cs_msg = false;

	// Reset c2r flags
	c2r.isNew = false;
	c2r.idNew = '\0';

	// Bail if no new input
	if (Serial1.available() == 0)
		return;

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(c2r.head, a2r.head);
	if (buff == 0)
	{
		return;
	}

	// Store header
	head = buff;

	// Get id
	id = WaitBuffRead();

	// Parse data;
	if (head == c2r.head)
	{
		is_cs_msg = true;
		ParseC2RData(id);
	}
	else if (head == a2r.head)
	{
		a2r.dat[0] = WaitBuffRead();
	}

	// Get packet num
	U.f = 0.0f;
	U.b[0] = WaitBuffRead();
	U.b[1] = WaitBuffRead();
	pack = U.i16[0];

	// Get recieved confirmation
	U.f = 0.0f;
	U.b[0] = WaitBuffRead();
	do_conf = U.b[0] != 0 ? true : false;

	// Get footer
	foot = WaitBuffRead();

	// Process ard packet
	if (!is_cs_msg && foot == a2r.foot)
	{
		id_ind = CharInd(id, r2a.idList, r2a.idLng);
		if (id_ind != -1)
		{
			// Update recive time
			t_rcvd = millis();

			// Reset flags
			r2a.doRcvCheck[id_ind] = false;

			// Store revieved pack details
			DebugRcvd('a', id, pack);

			// Send confirmation
			if (do_conf)
				StorePacketData('a', id, a2r.dat[0], pack);
		}
	}

	// Process cs packet
	if (is_cs_msg && foot != c2r.foot)
	{
		// mesage will be dumped
		c2r.isNew = false;
	}
	else if (is_cs_msg && foot == c2r.foot)
	{
		// Update recive time
		t_rcvd = millis();

		// Store id
		c2r.idNew = id;

		// Check for streaming started
		if (!fc.isStreaming)
		{
			// Signal streaming started
			fc.isStreaming = true;
		}

		// Get id ind
		id_ind = CharInd(id, c2r.idList, c2r.idLng);

		// Store revieved pack details
		if (id != 'P')
			DebugRcvd('c', id, pack);

		// Reset flags
		r2c.doRcvCheck[id_ind] = false;

		// Send packet confirmation
		if (do_conf)
			StorePacketData('c', id, 0, pack);

		// Check for dropped packets
		int pack_diff = (int)(pack - pack_last);

		if (pack_diff > 0)
		{
			// Save packet and set to process
			pack_last = pack;

			// Check for dropped packet
			pack_tot += pack_diff;
			int dropped_packs = pack_diff - 1;
			// print dropped packs
			if (dropped_packs > 0)
			{
				cnt_droppedPacks += dropped_packs;
				DebugDropped(dropped_packs, cnt_droppedPacks, pack_tot);
			}

		}

		// New pack
		if (c2r.packList[id_ind] != pack)
		{
			// Update last packet
			c2r.packList[id_ind] = pack;

			// Reset count
			c2r.cntRepeat[id_ind] = 0;

			// Pack should be used
			c2r.isNew = true;
		}
		// Resent pack
		else
		{
			// Do not reuse pack
			c2r.isNew = false;

			// Itterate count
			c2r.cntRepeat[id_ind]++;

			// Print dropped packets
			DebugResent(id, pack, c2r.cntRepeat[id_ind]);

		}
	}

}

// PARSE CS MESSAGE
void ParseC2RData(char id)
{
	// Local vars
	bool pass = false;
	byte buff = 0;
	int id_ind = 0;
	c2r.dat[0] = 0;
	c2r.dat[1] = 0;
	c2r.dat[2] = 0;
	a2r.dat[0] = 0;

	// Get system test data
	if (id == 'T')
	{
		// Get test id
		c2r.dat[0] = (float)WaitBuffRead();

		// Get test argument
		c2r.dat[1] = (float)WaitBuffRead();
	}

	// Get setup data
	if (id == 'S')
	{
		// Get session comand
		c2r.dat[0] = (float)WaitBuffRead();

		// Get tone condition
		c2r.dat[1] = (float)WaitBuffRead();
	}

	// Get MoveTo data
	if (id == 'M')
	{
		// Get move pos
		U.f = 0.0f;
		U.b[0] = WaitBuffRead();
		U.b[1] = WaitBuffRead();
		U.b[2] = WaitBuffRead();
		U.b[3] = WaitBuffRead();
		c2r.dat[0] = U.f;
	}

	// Get REWARD data
	if (id == 'R')
	{
		// Get stop pos
		U.f = 0.0f;
		U.b[0] = WaitBuffRead();
		U.b[1] = WaitBuffRead();
		U.b[2] = WaitBuffRead();
		U.b[3] = WaitBuffRead();
		c2r.dat[0] = U.f;

		// Get zone ind 
		c2r.dat[1] = (float)WaitBuffRead();

		// Get reward diration 
		c2r.dat[2] = (float)WaitBuffRead();
	}

	// Get halt robot data
	if (id == 'H')
	{
		// Get halt bool
		c2r.dat[0] = (float)WaitBuffRead();
	}

	// Get bulldoze rat data
	if (id == 'B')
	{
		// Get delay in sec
		c2r.dat[0] = (float)WaitBuffRead();

		// Get speed
		c2r.dat[1] = (float)WaitBuffRead();
	}

	// Get rat in/out
	if (id == 'I')
	{
		// Get session comand
		c2r.dat[0] = (float)WaitBuffRead();
	}

	// Get log request data
	if (id == 'L')
	{
		// Get session comand
		c2r.dat[0] = (float)WaitBuffRead();
	}

	// Get VT data
	else if (id == 'P')
	{

		// Get Ent
		c2r.vtEnt = WaitBuffRead();
		// Get TS
		U.f = 0.0f;
		U.b[0] = WaitBuffRead();
		U.b[1] = WaitBuffRead();
		U.b[2] = WaitBuffRead();
		U.b[3] = WaitBuffRead();
		c2r.vtTS[c2r.vtEnt] = U.l;
		// Get pos cm
		U.f = 0.0f;
		U.b[0] = WaitBuffRead();
		U.b[1] = WaitBuffRead();
		U.b[2] = WaitBuffRead();
		U.b[3] = WaitBuffRead();
		c2r.vtCM[c2r.vtEnt] = U.f;
	}

}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char chr1, char chr2)
{
	// Local vars
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	byte buff = 0;
	bool pass = false;

	// Check for overflow
	if (Serial1.available() < SERIAL_BUFFER_SIZE - 1)
	{
		// Wait for at least 1 byte
		while (Serial1.available() < 1 &&
			millis() < t_timeout);

		// Store next byte
		if (chr1 == '\0')
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
				buff != chr1  &&
				buff != chr2  &&
				millis() < t_timeout &&
				Serial1.available() < SERIAL_BUFFER_SIZE - 1);
			// check match was found
			if (buff == chr1 || buff == chr2)
			{
				pass = true;
			}
		}
	}

	// Failed
	if (!pass)
	{
		char str[100];

		// Buffer flooded
		if (Serial1.available() >= SERIAL_BUFFER_SIZE - 1)
		{
			// DUMP IT ALL
			while (Serial1.available() > 0)
			{
				Serial1.read();
			}
			cnt_overflowEvt++;
			// Log/print error
			sprintf(str, "!!ERROR!! WaitBuffRead(): Buffer Overflowed RX = %d", Serial1.available());
		}
		// Timed out
		else if (millis() > t_timeout) {
			cnt_timeoutEvt++;
			// Log/print error
			sprintf(str, "!!ERROR!! WaitBuffRead(): Timeout %d", cnt_timeoutEvt);
		}
		// Byte not found
		else if (chr1 != '\0' || chr2 != '\0') {
			// Log/print error
			sprintf(str, "!!ERROR!! WaitBuffRead(): Char %c and %c Not Found", chr1, chr2);
		}

		// Set buff to '\0' ((byte)-1) if !pass
		buff = '\0';
	}

	// Return buffer
	return buff;
}

// STORE PACKET DATA TO BE SENT
void StorePacketData(char targ, char id, byte d1, uint16_t pack, bool do_conf)
{
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2]dat, [3]pack, [4]do_conf, [5]footer, [6]targ
	*/

	// Local vars
	int queue_ind = 0;
	char head = '\0';
	char foot = '\0';
	int id_ind = 0;

	// Store r2a data
	if (targ == 'a')
	{
		// Shift data back so ard msg is first in queue
		for (int i = 0; i < sendQueueRows - 1; i++)
		{
			for (int j = 0; j < sendQueueCols; j++)
			{
				sendQueue[i][j] = sendQueue[i + 1][j];
			}
		}

		// Set queue ind to front
		queue_ind = sendQueueRows - 1;

		// Itterate r2a packet number
		if (pack == 0)
		{
			r2a.packCnt++;
			pack = r2a.packCnt;
		}

		// Always get confirmation 
		do_conf = true;

		// Store header and footer
		head = r2a.head;
		foot = r2a.foot;

		// Update last packet arrays
		id_ind = CharInd(id, r2a.idList, r2a.idLng);
		r2a.datList[id_ind] = d1;
		r2a.packList[id_ind] = pack;
	}

	// Store r2c data
	else if (targ == 'c')
	{
		// Set queue ind to back
		queue_ind = sendQueueInd;

		// Store header and footer
		head = r2c.head;
		foot = r2c.foot;

		// Update last packet arrays
		id_ind = CharInd(id, r2c.idList, r2c.idLng);
		r2c.datList[id_ind] = d1;
		r2c.packList[id_ind] = pack;
	}

	// Update queue index
	sendQueueInd--;

	// Store reciever id in last col
	sendQueue[queue_ind][sendQueueCols - 1] = targ;

	// Store header
	sendQueue[queue_ind][0] = head;
	// Store mesage id
	sendQueue[queue_ind][1] = id;
	// Store mesage data 
	sendQueue[queue_ind][2] = d1;
	// Store packet number
	U.f = 0.0f;
	U.i16[0] = pack;
	sendQueue[queue_ind][3] = U.b[0];
	sendQueue[queue_ind][4] = U.b[1];
	// Store get_confirm request
	sendQueue[queue_ind][5] = do_conf ? 1 : 0;
	// Store footer
	sendQueue[queue_ind][6] = foot;

	// Set to send
	doPackSend = true;

}

// SEND SERIAL PACKET DATA
void SendPacketData()
{
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2]dat, [3]pack, [4]do_conf, [5]footer, [6]targ
	*/

	// Local vars
	const int msg_lng = sendQueueCols - 1;
	static byte msg[msg_lng] = { 0 };
	char targ = '\0';
	bool do_send = false;
	char id = '\0';
	byte dat = 0;
	bool do_conf = 0;
	uint16_t pack = 0;
	int id_ind = 0;

	// pull out msg id
	id = sendQueue[sendQueueRows - 1][1];
	// dat
	dat = msg[2];
	// pack
	U.f = 0.0f;
	U.b[0] = msg[3];
	U.b[1] = msg[4];
	pack = U.i16[0];
	// do_conf 
	do_conf = msg[5] == 1 ? true : false;
	// targ
	targ = sendQueue[sendQueueRows - 1][sendQueueCols - 1];

	// Move next in queue to temp msg array
	for (int j = 0; j < msg_lng; j++)
	{
		msg[j] = sendQueue[sendQueueRows - 1][j];
	}

	// Send sync time or rew tone immediately
	if (
		(id == 'r' || id == '+') &&
		Serial1.availableForWrite() > msg_lng + 10 &&
		Serial1.available() < 100
		)
	{
		do_send = true;
	}
	// Avoid overlap between sent or rcvd events
	else if (
		Serial1.availableForWrite() > msg_lng + 10 &&
		Serial1.available() == 0 &&
		millis() > t_sent + dt_sendSent &&
		millis() > t_rcvd + dt_sendRcvd
		)
	{
		do_send = true;
	}

	// Send if conditions met
	if (do_send)
	{
		// Send
		Serial1.write(msg, msg_lng);

		// Update send time 
		t_sent = millis();

		// Set flags for recieve confirmation
		if (do_conf)
		{
			if (targ == 'a')
			{
				id_ind = CharInd(id, r2a.idList, r2a.idLng);
				r2a.sendTim[id_ind] = t_sent;
				r2a.doRcvCheck[id_ind] = true;
			}
			else if (targ == 'c')
			{
				id_ind = CharInd(id, r2c.idList, r2c.idLng);
				r2c.sendTim[id_ind] = t_sent;
				r2c.doRcvCheck[id_ind] = true;
			}
		}

		// Update queue index
		sendQueueInd++;

		// Remove sent msg from front of queue
		for (int i = sendQueueRows - 1; i >= 1; i--)
		{
			for (int j = 0; j < sendQueueCols; j++)
			{
				sendQueue[i][j] = sendQueue[i - 1][j];
			}
		}
		// Set first entry to all zeros
		for (int j = 0; j < sendQueueCols; j++)
		{
			sendQueue[0][j] = 0;
		}

		// Set to not send again if all sent
		if (sendQueueInd == sendQueueRows - 1)
		{
			doPackSend = false;
		}

		// Print
		DebugSent(targ, id, dat, pack, do_conf);
	}
}

// CHECK IF ROB TO ARD PACKET SHOULD BE RESENT
bool CheckResend(char targ)
{
	// Local vars
	bool do_pack_resend = false;

	// Loop and check ard flags
	if (targ == 'a')
	{
		for (int i = 0; i < r2a.idLng; i++)
		{
			if (
				r2a.doRcvCheck[i] &&
				millis() > r2a.sendTim[i] + dt_resend
				)
			{
				if (r2a.resendCnt[i] < resendMax) {

					// Resend data
					StorePacketData('a', r2a.idList[i], r2a.datList[i], r2a.packList[i]);

					// Update count
					r2a.resendCnt[i]++;
					DebugResent(r2a.idList[i], r2a.packList[i], r2a.resendCnt[i]);

					// Set flags
					do_pack_resend = true;
					r2a.doRcvCheck[i] = false;
				}
				// Com likely down
				else {
					r2a.doRcvCheck[i] = false;
				}
			}
		}
	}

	// Loop and check cs flags
	else if (targ == 'c')
	{
		for (int i = 0; i < r2c.idLng; i++)
		{
			if (
				r2c.doRcvCheck[i] &&
				millis() > r2c.sendTim[i] + dt_resend
				)
			{
				if (r2c.resendCnt[i] < resendMax) {

					// Resend data
					StorePacketData('c', r2c.idList[i], r2c.datList[i], r2c.packList[i]);

					// Update count
					r2c.resendCnt[i]++;
					DebugResent(r2c.idList[i], r2c.packList[i], r2c.resendCnt[i]);

					// Set flags
					do_pack_resend = true;
					r2c.doRcvCheck[i] = false;
				}
				// Com likely down
				else {
					r2c.doRcvCheck[i] = false;
				}
			}
		}
	}

	// Return
	return do_pack_resend;
}

#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------

// CONFIGURE AUTODRIVER BOARDS
void AD_Config()
{
	// Set busy pin as BUSY_PIN or SYNC_PIN;
	/*
	SYNC_FS_2 - two pulses on sync pin per full step of motor
	SYNC_FS - one pulse per full step
	SYNC_XFS - where X can be 2, 4, 8, 16, 32, or 64, and X indicates the number of full steps between pulses on the sync pin
	*/
	AD_R.configSyncPin(BUSY_PIN, 0);
	AD_F.configSyncPin(BUSY_PIN, 0);

	// Microsteps per step
	/*
	STEP_FS - Full-step mode; microstepping disabled
	STEP_FS_X - Enable microstepping with X microsteps per full step. X can be 2, 4, 8, 16, 32, 64, or 128.
	*/
	AD_R.setParam(STEP_MODE, STEP_FS_128);
	AD_F.setParam(STEP_MODE, STEP_FS_128);

	// PWM freq
	/*
	PWM_DIV_X, where X can be any value 1-7.
	PWM_MUL_X, where X can be 0_625 (for 0.625), 0_75 (for 0.75), 0_875, 1, 1_25, 1_5, 1_75, or 2.
	*/
	AD_R.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq
	AD_F.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq		

												// Overcurent enable
	AD_R.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC
	AD_F.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC

												// Motor V compensation
												/*
												VS_COMP_ENABLE, VS_COMP_DISABLE
												*/
	AD_R.setVoltageComp(VS_COMP_ENABLE);
	AD_F.setVoltageComp(VS_COMP_ENABLE);

	// Switch pin mode
	AD_R.setSwitchMode(SW_USER);				// Switch is not hard stop
	AD_F.setSwitchMode(SW_USER);				// Switch is not hard stop

												// Slew rate
												/*
												Upping the edge speed increases torque
												SR_180V_us, SR_290V_us, SR_530V_us
												*/
	AD_R.setSlewRate(SR_530V_us);
	AD_F.setSlewRate(SR_530V_us);


	// Overcurrent threshold
	/*
	375, 750, 1125, 1500, 1875, 2250, 2625, 3000,
	3375, 3750, 4125, 4500, 4875, 5250, 5625, or 6000
	Peak Amp for 1.2 A stepper = 1.2*1.41 = 1690 mA
	Peak Amp for 1.7 A stepper = 1.7*1.41 = 2397 mA
	Peak Amp for 2.82 A stepper = 2.82*1.41 = 3.97 mA
	*/
	AD_R.setOCThreshold(OC_4875mA);
	AD_F.setOCThreshold(OC_3750mA);

	// Low speed compensation
	/*
	Enabled low speed compensation. If enabled, MinSpeed is upper threshold at which this compensation is employed.
	*/
	AD_R.setLoSpdOpt(true);
	AD_F.setLoSpdOpt(true);

	// ---------SPEED SETTTINGS---------

	// Steps/s max
	AD_R.setMaxSpeed(maxSpeed * cm2stp);
	AD_F.setMaxSpeed(maxSpeed * cm2stp);

	// Minimum speed
	AD_R.setMinSpeed(10 * cm2stp);
	AD_F.setMinSpeed(10 * cm2stp);

	// Full speed
	AD_R.setFullSpeed(maxSpeed * cm2stp);
	AD_F.setFullSpeed(maxSpeed * cm2stp);

	// Acceleration
	/*
	Accelerate at maximum steps/s/s; 0xFFF = infinite
	*/
	AD_R.setAcc(maxAcc * cm2stp);
	AD_F.setAcc(maxAcc * cm2stp);

	// Deceleration
	/*
	Deccelerate at maximum steps/s/s; 0xFFF = infinite
	*/
	AD_R.setDec(maxDec * cm2stp);
	AD_F.setDec(maxDec * cm2stp);

	// ---------KVAL SETTTINGS---------
	/*
	K Val settings
	KVAL = [(KVAL_X + BEMF_COMP) * VSCOMP * K_THERM] * microstep
	KVAL = Rm * Iph / Vs = %
	Pololu item #: 1200: 1.2A, 4V, 3.3Ohm, 2.8mH = 84.15
	Pololu item #: 2267: 1.68A, 2.8V, 1.65Ohm, 3.2mH = 58.9
	AA item #: 23Y108D-LW8: 2.82A, 2.82V, 1.65Ohm, 3.2mH = 58.9
	*/

	// NIMA 23 24V MIN KVALS
	AD_R.setAccKVAL(50);				        // This controls the acceleration current
	AD_R.setDecKVAL(50);				        // This controls the deceleration current
	AD_R.setRunKVAL(50);					    // This controls the run current
	AD_R.setHoldKVAL(20);				        // This controls the holding current keep it low

												// NIMA 17 24V
	AD_F.setAccKVAL(50);				        // This controls the acceleration current
	AD_F.setDecKVAL(50);				        // This controls the deceleration current
	AD_F.setRunKVAL(50);					    // This controls the run current
	AD_F.setHoldKVAL(20);				        // This controls the holding current keep it low

												/*
												// NIMA 17 12V
												AD_F.setAccKVAL(100);				        // This controls the acceleration current
												AD_F.setDecKVAL(100);				        // This controls the deceleration current
												AD_F.setRunKVAL(120);					    // This controls the run current
												AD_F.setHoldKVAL(35);				        // This controls the holding current keep it low
												*/
}

// RESET AUTODRIVER BOARDS
void AD_Reset()
{
	// Reset each axis
	AD_R.resetDev();
	delayMicroseconds(100);
	AD_F.resetDev();
	delayMicroseconds(100);
	// Configure each axis
	AD_Config();
	delayMicroseconds(100);
	AD_R.getStatus();
	delayMicroseconds(100);
	AD_F.getStatus();
	delayMicroseconds(100);
}

// CHECK AUTODRIVER STATUS
void AD_CheckOC()
{
	// Local vars
	static uint32_t t_checkAD = 0;
	static bool dp_disable = false;
	static int cnt_errors = 0;
	char str[100] = { 0 };
	int ocd_r;
	int ocd_f;

	if (dp_disable)
		return;

	if (
		millis() > t_checkAD &&
		cnt_errors < 5
		)
	{
		adR_stat = AD_R.getStatus();
		ocd_r = CheckAD_Status(adR_stat, "OCD");
		adF_stat = AD_F.getStatus();
		ocd_f = CheckAD_Status(adF_stat, "OCD");

		// Check for overcurrent shut down
		if (ocd_r == 0 || ocd_f == 0)
		{
			sprintf(str, "!!ERROR!! AD OCD: R_OCD=%d F_OCD=%d", ocd_r, ocd_f);
			DebugError(str);
			cnt_errors++;
			//AD_Reset();
		}

		// Set next check
		t_checkAD = millis() + dt_checkAD;
	}
	// Disable error checking after 5 hits
	else if (cnt_errors >= 5) {
		sprintf(str, "!!ERROR!! DISABLED AD CHECK AFTER %d ERRORS", cnt_errors);
		DebugError(str);
		dp_disable = true;
	}

}

// HARD STOP
void HardStop(char called_from[])
{
	// Normal hard stop
	AD_R.hardStop();
	AD_F.hardStop();

	// Reset pid
	Pid.Reset();

	// Set to high impedance for manual session
	if (fc.isManualSes)
	{
		delay(100);
		AD_R.hardHiZ();
		AD_F.hardHiZ();
	}

	// Print event
	char str[100] = { 0 };
	sprintf(str, "HARD STOP [%s]", called_from);
	DebugFlow(str);
}

// IR TRIGGERED HARD STOP
void Function_IRprox_Halt()
{
	if (!(Bull.mode == "Active" && Bull.state == "On") &&
		!fc.isManualSes)
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
	if (agent == fc.motorControl)
	{
		if (dir == 'f')
		{
			AD_R.run(FWD, speed_steps);
			AD_F.run(FWD, speed_steps*scaleFrontAD);
		}
		else if (dir == 'r')
		{
			AD_R.run(REV, speed_steps);
			AD_F.run(REV, speed_steps*scaleFrontAD);
		}
		return true;
	}
	else return false;
}

// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(char set_to[], char called_from[])
{
	// VALUES:  ["None", "Open", "MoveTo", "Bull", "Pid"]
	bool pass = false;

	// Can always set to "None"
	if (set_to == "None")
	{
		fc.motorControl = "None";
	}

	// Do not change control if halted
	if (!fc.isHalted)
	{
		// Cannot unset "None" unless certain conditions met
		if (fc.motorControl == "None")
		{
			// Can still move robot if rat not in
			if (
				set_to == "MoveTo" &&
				!fc.isRatIn
				)
			{
				fc.motorControl = set_to;
			}

			// CheckBlockTimElapsed can unblock if tracking setup
			else if (
				fc.isTrackingEnabled &&
				(called_from == "CheckBlockTimElapsed")
				)
			{
				fc.motorControl = set_to;
			}

			// InitializeTracking can always unset "None"
			if (called_from == "InitializeTracking")
			{
				fc.motorControl = set_to;
			}

		}

		// "MoveTo" can only be set to "Open" or "None"
		else if (fc.motorControl == "MoveTo")
		{
			if (set_to == "Open")
			{
				fc.motorControl = set_to;
			}
		}

		// "Bull" can only be set to "MoveTo" or "Open"
		else if (fc.motorControl == "Bull")
		{
			if (set_to == "MoveTo" || set_to == "Open")
			{
				fc.motorControl = set_to;
			}
		}

		// Otherwise can set to anything
		else if (fc.motorControl == "Open" || fc.motorControl == "Pid")
		{
			fc.motorControl = set_to;
		}

	}

	// Return true if set to input
	if ((String)set_to == fc.motorControl)
	{
		pass = true;
	}

	// Store current controller
	DebugMotorControl(pass, set_to, called_from);

	return pass;
}

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(int dt, char called_from[])
{
	// Set blocking and time
	fc.isBlockingTill = true;

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
	if (fc.isBlockingTill)
	{
		// Check that all 3 measures say rat has passed
		bool is_passed_feeder =
			fc.isTrackingEnabled &&
			ekf.RatPos - (ekf.RobPos + feedDist) > 0 &&
			RatVT.posNow - (ekf.RobPos + feedDist) > 0 &&
			RatPixy.posNow - (ekf.RobPos + feedDist) > 0;

		// Check for time elapsed or rat moved at least 3cm past feeder
		if (millis() > dt_blockMotor || is_passed_feeder)
		{
			// Print blocking finished
			DebugMotorBocking("finished blocking motor: tim=", millis(), "CheckBlockTimElapsed");

			// Retract feeder arm
			Reward.RetractFeedArm();

			// Set flag to stop checking
			fc.isBlockingTill = false;

			// Open up control
			SetMotorControl("Open", "CheckBlockTimElapsed");
		}
	}
}

// DO SETUP TO BEGIN TRACKING
void InitializeTracking()
{
	// Reset pos data once after rat in
	if (fc.isRatIn &&
		!fc.isTrackingEnabled &&
		RatVT.isDataNew &&
		RatPixy.isDataNew &&
		RobVT.isDataNew)
	{
		// Local vars
		int n_laps = 0;
		float cm_diff = 0;
		float cm_dist = 0;

		// Print process
		DebugFlow("INITIALIZING RAT TRACKING");

		// Check that rat pos > robot pos
		n_laps = RatVT.posNow > RobVT.posNow ? 0 : 1;
		if (n_laps > 0)
			DebugFlow("SET RAT POS AHEAD");
		// set n_laps for rat vt data
		RatVT.SetPos(RatVT.posNow, n_laps);
		RatPixy.SetPos(RatPixy.posNow, n_laps);

		// Check that results make sense
		cm_diff = RatVT.posNow - RobVT.posNow;
		cm_dist = min((140 * PI) - abs(cm_diff), abs(cm_diff));

		// Rat should be no more than 90 deg rom rob
		if (cm_dist > ((140 * PI) / 4))
		{
			// Will have to run again with new samples
			RatVT.SetPos(0, 0);
			RatPixy.SetPos(0, 0);
			RobVT.SetPos(0, 0);
			DebugFlow("RAT POS WRONG SO POS DATA RESET");
		}
		// Good to go
		else
		{

			// Set flag
			fc.isTrackingEnabled = true;

			// Reset ekf
			Pid.ResetEKF("InitializeTracking");

			// Don't start pid for manual sessions
			if (!fc.isManualSes)
			{
				// Open up motor control
				SetMotorControl("Open", "InitializeTracking");

				// Run Pid
				Pid.Run("InitializeTracking");
				DebugFlow("Pid STARTED");
			}

			// Initialize bulldoze
			if (fc.doBulldoze)
			{
				// Run from initial blocked mode
				Bull.TurnOn("InitializeTracking");
				DebugFlow("BULLDOZE INITIALIZED");
			}

			// Blink lcd display
			RatInBlink();

		}
	}
}

// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT() {

	if (fc.isRatIn)
	{

		// Local vars
		int dt_max_frame = 100;
		int dt_vt = 0;
		int dt_pixy = 0;

		// Compute dt
		dt_vt = millis() - RatVT.t_msNow;
		dt_pixy = millis() - RatPixy.t_msNow;

		// Check pixy
		if (
			dt_pixy >= dt_max_frame &&
			dt_vt < dt_pixy
			)
		{
			// Use VT for Pixy data
			RatPixy.SetDat(RatVT.posAbs, RatVT.t_msNow);
		}
		// Check VT 
		else if (
			dt_vt >= dt_max_frame &&
			dt_pixy < dt_vt
			)
		{
			// Use Pixy for VT data
			RatVT.SetDat(RatPixy.posAbs, RatPixy.t_msNow);
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
	uint16_t blocks = Pixy.getBlocks();

	// Check for new data
	if (blocks)
	{

		// Save time stamp
		t_px_ts = millis();

		// Get Y pos from last block and convert to CM
		pixy_pos_y = Pixy.blocks[blocks - 1].y;
		px_rel =
			pixyCoeff[0] * (pixy_pos_y * pixy_pos_y * pixy_pos_y * pixy_pos_y) +
			pixyCoeff[1] * (pixy_pos_y * pixy_pos_y * pixy_pos_y) +
			pixyCoeff[2] * (pixy_pos_y * pixy_pos_y) +
			pixyCoeff[3] * pixy_pos_y +
			pixyCoeff[4];

		// Scale to abs space with rob vt data
		px_abs = px_rel + RobVT.posAbs;
		if (px_abs > (140 * PI))
		{
			px_abs = px_abs - (140 * PI);
		}
		// Update pixy pos and vel
		RatPixy.UpdatePos((float)px_abs, t_px_ts);

	}

}

// UPDATE EKF
void UpdateEKF()
{
	// Check for new data w or w/o rat tracking
	if ((RatVT.isDataNew && RatPixy.isDataNew && RobVT.isDataNew) ||
		(RobVT.isDataNew && !fc.isRatIn))
	{

		// Check EKF progress
		Pid.CheckEKF(millis());

		// Update pid next loop time
		Pid.SetLoopTime(millis());

		// Set rat pos data to match robot
		if (!fc.isRatIn)
		{
			RatVT.SetPos(0, 0);
			RatPixy.SetPos(0, 0);
		}
		// Set rat pos data to match robot
		else if (fc.isTrackingEnabled)
		{
			// Set flag for reward 
			Reward.is_ekfNew = true;
		}

		//----------UPDATE EKF---------
		double z[M] = {
			RatVT.GetPos(),
			RatPixy.GetPos(),
			RobVT.GetPos(),
			RatVT.GetVel(),
			RatPixy.GetVel(),
			RobVT.GetVel(),
		};

		// Run EKF
		Fuser.step(z);

		// Update error estimate
		float rat_pos = Fuser.getX(0);
		float rob_pos = Fuser.getX(1);
		float rat_vel = Fuser.getX(2);
		float rob_vel = Fuser.getX(3);

		// Copy over values
		ekf.RatPos = !isnan(rat_pos) ? rat_pos : ekf.RatPos;
		ekf.RobPos = !isnan(rob_pos) ? rob_pos : ekf.RobPos;
		ekf.RatVel = !isnan(rat_vel) ? rat_vel : ekf.RatVel;
		ekf.RobVel = !isnan(rob_vel) ? rob_vel : ekf.RobVel;

		// Check for nan values
		if (isnan(rat_pos) || isnan(rob_pos) || isnan(rat_vel) || isnan(rob_vel)) {
			char str[200] = { 0 };
			sprintf(str, "!!ERROR!! \"nan\" EKF OUTPUT: ratVT=%0.2f|%0.2f ratPixy=%0.2f|%0.2f robVT=%0.2f|%0.2f",
				RatVT.posNow, RatVT.velNow, RatPixy.posNow, RatPixy.velNow, RobVT.posNow, RatVT.velNow);
			DebugError(str);
		}

	}
}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid()
{
	// Local vars
	byte sol_state = digitalRead(pin.Rel_Rew);

	// Change state
	sol_state = !sol_state;

	// Open/close solenoid
	digitalWrite(pin.Rel_Rew, sol_state);

	// Print to LCD
	char str[50] = { 0 };
	sprintf(str, "%s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN" : "CLOSED");
	PrintLCD("REW SOLENOID", str, 's');
}

// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid()
{
	// Local vars
	byte sol_state = digitalRead(pin.Rel_EtOH);

	// Change state
	sol_state = !sol_state;

	// Open/close solenoid
	digitalWrite(pin.Rel_EtOH, sol_state);

	// Make sure periodic drip does not run
	if (sol_state)
		doEtOHRun = false;
	else doEtOHRun = true;

	// Print to LCD
	char str[50] = { 0 };
	sprintf(str, "%s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN" : "CLOSED");
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
	etoh_dist_diff = ekf.RobPos - etoh_dist_start;

	// Check if EtOH should be run
	if (doEtOHRun)
	{

		// Check if sol should be opened
		if (
			millis() > (t_etoh_start + dt_delEtOH) ||
			etoh_dist_diff > distMaxEtOH
			)
		{

			// Check motor stopped
			// Volt read only accurate when motor stopped
			if (
				!isEtOHOpen &&
				CheckAD_Status(adR_stat, "MOT_STATUS") == 0 &&
				CheckAD_Status(adF_stat, "MOT_STATUS") == 0
				)
			{
				// Open solenoid
				digitalWrite(pin.Rel_EtOH, HIGH);

				// Reset vars
				t_etoh_start = millis();
				etoh_dist_start = ekf.RobPos;

				// Set flag
				isEtOHOpen = true;

				// Print to debug
				DebugFlow("EtOH SOLENOID OPEN");
			}
		}

		// Check if sol should be closed
		else if (
			isEtOHOpen &&
			millis() > (t_etoh_start + dt_durEtOH)
			)
		{
			// Close solenoid
			digitalWrite(pin.Rel_EtOH, LOW);

			// Set flag
			isEtOHOpen = false;

			// Print to debug
			DebugFlow("EtOH SOLENOID CLOSE");
		}
	}
}

// CHECK BATTERY VOLTAGE
void GetBattVolt()
{
	// Local vars
	static bool do_volt_update = false;
	static float volt_avg = 0;
	static int n_samples = 0;
	float bit_in = 0;
	float volt_in = 0;
	float volt_sum = 0;
	byte byte_out = 0;


	// Only run if relay open and motor stopped
	if (
		isEtOHOpen &&
		CheckAD_Status(adR_stat, "MOT_STATUS") == 0 &&
		CheckAD_Status(adF_stat, "MOT_STATUS") == 0
		)
	{
		bit_in = analogRead(pin.BatVolt);
		volt_in = bit_in * bit2volt;
		volt_sum = 0;
		// Shift array and compute average
		for (int i = 99; i > 0; i--) {
			batVoltArr[i] = batVoltArr[i - 1];
			volt_sum += batVoltArr[i];
		}
		batVoltArr[0] = volt_in;
		volt_avg = volt_sum / 99;

		// Set flag to send update if array full
		n_samples = n_samples < 100 ? n_samples + 1 : n_samples;
		if (n_samples >= 100)
			do_volt_update = true;
	}

	// Send updated voltage
	else if (do_volt_update)
	{
		// Store new voltage level
		voltNew = volt_avg;

		// Convert float to byte
		byte_out = byte(round(volt_avg * 10));

		// Add to queue if streaming established
		if (fc.isStreaming)
			StorePacketData('c', 'J', byte_out, 0);

		char str[50] = { 0 };
		sprintf(str, "VCC: %0.2fV", volt_avg);
		DebugFlow(str);

		// Reset flag
		do_volt_update = false;
	}
}

// TURN LCD LIGHT ON/OFF
void ChangeLCDlight()
{
	if (!isLitLCD) {
		analogWrite(pin.Disp_LED, 50);
		isLitLCD = true;
	}
	else {
		analogWrite(pin.Disp_LED, 0);
		isLitLCD = false;
	}
}

// CHECK FOR BUTTON INPUT
bool GetButtonInput()
{
	// Local vars
	bool is_new_input = false;
	static bool is_pressed[3] = { false, false, false };
	static uint32_t t_debounce[3] = { millis() + 1000, millis() + 1000, millis() + 1000 };
	static int dt_hold[3] = { 0, 0, 0 };
	static int dt_long_hold = 500;
	int btn_ind = 0;

	// RUN BUTTON 1 OPPERATIONS (Trigger reward)
	btn_ind = 0;
	if (digitalRead(pin.Btn[btn_ind]) == LOW)
	{
		// exit if < debounce time has not passed
		if (t_debounce[btn_ind] > millis()) return false;

		// Set to start reward function
		btn_doRew = true;

		// Update debounce time
		t_debounce[btn_ind] = millis() + Reward.duration + 100;

		// Flag input rcvd
		is_new_input = true;
	}

	// RUN BUTTON 2 OPPERATIONS (Open/close solonoid)
	/*
	Note: Long hold to open/close EtOH
	*/
	btn_ind = 1;
	if (
		digitalRead(pin.Btn[btn_ind]) == LOW &&
		!is_pressed[btn_ind]
		)
	{
		// exit if < debounce time has not passed
		if (t_debounce[btn_ind] > millis()) return false;

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
			digitalRead(pin.Btn[btn_ind]) == HIGH &&
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

			// Ret flags
			dt_hold[btn_ind] = 0;
			is_pressed[btn_ind] = false;

			// Flag input rcvd
			is_new_input = true;
		}

	}

	// RUN BUTTON 3 OPPERATIONS (Turn on/off LCD LED)
	btn_ind = 2;
	if (digitalRead(pin.Btn[btn_ind]) == LOW)
	{
		// exit if < 250 ms has not passed
		if (t_debounce[btn_ind] > millis()) return false;

		// Set flag to change lcd state
		btn_doChangeLCDstate = true;

		// Update debounce time
		t_debounce[btn_ind] = millis() + 250;

		// Flag input rcvd
		is_new_input = true;
	}

	// Return flag
	return is_new_input;
}

// QUIT AND RESTART ARDUINO
void QuitSession()
{
	// Stop all movement
	HardStop("QuitSession");
	Pid.Stop("QuitSession");
	Bull.TurnOff("QuitSession");
	delayMicroseconds(100);
	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

#pragma endregion


#pragma region --------DEBUGGING---------

// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t)
{
	// Local vars
	bool do_print = db.print_flow && (db.Console || db.LCD);
	bool do_log = db.log_flow && db.Log;

	if (do_print || do_log)
	{
		// Add to print queue
		if (do_print)
			StoreDBPrintStr(msg, t);
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(msg, t);
	}
}

// LOG/PRINT ERRORS
void DebugError(char msg[], uint32_t t)
{
	// Local vars
	bool do_print = db.print_errors && (db.Console || db.LCD);
	bool do_log = db.log_errors && db.Log;

	if (do_print || do_log)
	{
		// Add to print queue
		if (do_print)
			StoreDBPrintStr(msg, t);
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(msg, t);
	}
}

// LOG/PRINT DROPPED PACKET DEBUG STRING
void DebugDropped(int missed, int missed_total, int total)
{
	// Local vars
	bool do_print = db.print_errors && (db.Console || db.LCD);
	bool do_log = db.log_errors && db.Log;

	if (do_print || do_log)
	{
		// Local vars
		int buff_tx = 0;
		int buff_rx = 0;

		// Get total data in buffers
		buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		buff_rx = Serial1.available();

		char str[200] = { 0 };
		sprintf(str, "!!ERROR!! PACK LOST: tot=%d/%d/%d tx=%d rx=%d", missed, missed_total, total, buff_tx, buff_rx);

		// Add to print queue
		if (do_print)
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(str, millis());
	}
}

// LOG/PRINT RESENT PACKET DEBUG STRING
void DebugResent(char id, uint16_t pack, int total)
{
	// Local vars
	bool do_print = db.print_errors && (db.Console || db.LCD);
	bool do_log = db.log_errors && db.Log;

	if (do_print || do_log)
	{
		// Local vars
		int buff_tx = 0;
		int buff_rx = 0;

		// Get total data in buffers
		buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		buff_rx = Serial1.available();

		char str[200] = { 0 };
		sprintf(str, "!!ERROR!! RESENT PACK: tot=%d tx=%d rx=%d id=%c pack=%d", total, buff_tx, buff_rx, id, pack);

		// Add to print queue
		if (do_print)
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(str, millis());
	}
}

// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[])
{
	// Local vars
	bool do_print = db.print_motorControl && (db.Console || db.LCD);
	bool do_log = db.log_motorControl && db.Log;

	if (do_print || do_log)
	{
		char str[200] = { 0 };
		sprintf(str, "mc change %s: set_in=%s set_out=%s [%s]", pass ? "succeeded" : "failed", set_to, fc.motorControl.c_str(), called_from);

		// Add to print queue
		if (do_print)
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(str, millis());
	}
}

// LOG/PRINT MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(char msg[], uint32_t t, char called_from[])
{
	// Local vars
	bool do_print = db.print_motorControl && (db.Console || db.LCD);
	bool do_log = db.log_motorControl && db.Log;

	if (do_print || do_log)
	{
		char str[100] = { 0 };
		sprintf(str, "%s %lu ms [%s]", msg, t, called_from);

		// Add to print queue
		if (do_print)
			StoreDBPrintStr(str, millis());
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(str, millis());
	}
}

// LOG/PRINT RECIEVED PACKET DEBUG STRING
void DebugRcvd(char from, char id, uint16_t pack)
{
	// Local vars
	bool do_print = db.print_c2r && (db.Console || db.LCD);
	bool do_log = db.log_c2r && db.Log;

	if (do_print || do_log)
	{
		// Print specific pack contents
		char str[200] = { 0 };
		if (from == 'c')
		{
			sprintf(str, "rcvd_%c2r: id=%c dat1=%0.2f dat2=%0.2f dat3=%0.2f pack=%d", from, id, c2r.dat[0], c2r.dat[1], c2r.dat[2], pack);
		}
		else
			sprintf(str, "rcvd_%c2r: id=%c dat1=%0.2f pack=%d", from, id, a2r.dat[0], pack);

		// Add to print queue
		if (do_print)
			StoreDBPrintStr(str, t_rcvd);
		// Add to log queue
		if (do_log)
			Log.StoreLogEntry(str, t_rcvd);
	}

}

// LOG/PRINT SENT PACKET DEBUG STRING
void DebugSent(char targ, char id, byte d1, uint16_t pack, bool do_conf)
{
	// Local vars
	bool do_print = ((db.print_r2c && targ == 'c') || (db.print_r2a && targ == 'a')) &&
		(db.Console || db.LCD);
	bool do_log = ((db.log_r2c && targ == 'c') || (db.log_r2a && targ == 'a')) &&
		db.Log;

	if (do_print || do_log)
	{

		// Make string
		char str[200];
		sprintf(str, "sent_r2%c: id=%c dat=%d pack=%d do_conf=%s", targ, id, d1, pack, do_conf ? "true" : "false");

		// Store
		if (do_print)
			StoreDBPrintStr(str, t_sent);
		if (do_log)
			Log.StoreLogEntry(str, t_sent);

	}

}

// STORE STRING FOR PRINTING
void StoreDBPrintStr(char msg[], uint32_t t)
{
	// Local vars
	char msg_store[200] = { 0 };
	char str_tim[200] = { 0 };
	uint32_t t_m = 0;
	float t_s = 0;

	// Make sure data should be printed
	if (!db.LCD && !db.Console)
		return;

	// Shift queue
	for (int r = 0; r < printQueueRows - 1; r++)
		for (int c = 0; c < printQueueCols; c++)
			printQueue[r][c] = printQueue[r + 1][c];

	// Set queue ind and check for overflow
	if (printQueueInd == 0)
	{
		// Store error so this is printed
		char err_str[100] = "!!ERROR!! PRINT QUEUE OVERFLOWED";
		for (int c = 0; c < strlen(err_str) + 1; c++)
			msg[c] = err_str[c];
	}
	else
		printQueueInd--;

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Make time string
	sprintf(str_tim, "[%0.2fs]", t_s);

	// Get string with time
	char spc[50] = { 0 };
	char arg[50] = { 0 };
	sprintf(arg, "%%%ds", 20 - strlen(str_tim));
	sprintf(spc, arg, '_');

	// Put it all together
	char msg_send[200] = { 0 };
	sprintf(msg_store, "%s%s%s\n", str_tim, spc, msg);

	// Store current message
	for (int c = 0; c < printQueueCols; c++) {
		printQueue[printQueueRows - 1][c] = msg_store[c];
		if (printQueue[printQueueRows - 1][c] == '\0')
			break;
	}

	// Set flag
	doPrint = true;
}

// PRINT DEBUG STRINGS TO CONSOLE/LCD
void PrintDebug()
{
	// Local vars 
	static int scale_ind = 40 / printQueueRows;
	int lcd_pos = 0;
	char msg_print[200] = { 0 };

	// Avoid overlap between sent or rcvd events
	if (millis() < t_sent + dt_sendSent ||
		millis() < t_rcvd + dt_sendRcvd)
	{
		return;
	}

	if ((db.LCD && !doBlockLCDlog) ||
		db.Console)
	{
		// Print to console
		if (db.Console)
		{
			// Get current message
			for (int c = 0; c < printQueueCols; c++) {
				msg_print[c] = printQueue[printQueueInd][c];
				if (msg_print[c] == '\0')
					break;
			}

			// Print
			SerialUSB.print(msg_print);
		}

		// Print to LCD
		if (db.LCD && !doBlockLCDlog)
		{
			// Change settings
			LCD.setFont(TinyFont);
			LCD.invert(false);

			// Clear
			LCD.clrScr();
			LCD.update();

			// Print all entries
			for (int r = printQueueRows - 1; r >= printQueueRows - 9; r--)
			{
				// Break for empty strings
				if (printQueue[r][0] == '\0')
					break;

				// Get pos ind
				lcd_pos += 6;

				// Print next entry including '\0'
				for (int c = 0; c < printQueueCols; c++) {
					msg_print[c] = printQueue[r][c];
					if (msg_print[c] == '\0')
						break;
				}
				LCD.print(msg_print, LEFT, lcd_pos);

				// Update
				LCD.update();
			}
		}

		// Update queue index
		printQueueInd++;

		// Set to not print again if all printed
		if (printQueueInd >= printQueueRows)
			doPrint = false;

	}
}

// FOR PRINTING TO LCD
void PrintLCD(char msg_1[], char msg_2[], char f_siz)
{

	// Change settings
	if (f_siz == 's')
		LCD.setFont(SmallFont);
	else if (f_siz == 't')
		LCD.setFont(TinyFont);
	LCD.invert(true);

	// Clear
	LCD.clrScr();

	// Print
	if (msg_2[0] != '\0')
	{
		LCD.print(msg_1, CENTER, 15);
		LCD.print(msg_2, CENTER, 25);
	}
	else LCD.print(msg_1, CENTER, 20);

	// Update
	LCD.update();

	// Block LCD logging/printing while displayed
	doBlockLCDlog = true;
}

// CLEAR LCD
void ClearLCD()
{
	// Clear
	LCD.clrScr();

	// Update
	LCD.update();

	// Stop blocking LCD log
	doBlockLCDlog = false;
}

// PRINT SPECIAL CHARICTERS
char* PrintSpecialChars(char chr, bool do_show_byte)
{
	/*
	Character					ASCII Representation	ASCII Value		Escape Sequence
	---------                   --------------------    -----------     ---------------
	Newline						NL(LF)					10				\n
	Horizontal tab				HT						9				\t
	Vertical tab				VT						11				\v
	Backspace					BS						8				\b
	Carriage return				CR						13				\r
	Formfeed					FF						12				\f
	Alert						BEL						7				\a
	Backslash					\						92				\\
	Question mark				?						63				\?
	Single quotation mark		'						39				\'
	Double quotation mark		"						34				\"
	Null character				NUL						0				\0
	*/

	// Local vars
	static char str[10] = { 0 };
	byte b = chr;

	for (int i = 0; i < 10; i++)
		str[i] = '\0';

	if (!do_show_byte)
	{
		// Get normal and special chars in quots
		switch (b) {
		case 10: sprintf(str, "\\n\n"); break;
		case 9: sprintf(str, "\\t\t"); break;
		case 11: sprintf(str, "\\v\v"); break;
		case 8: sprintf(str, "\\b\b"); break;
		case 13: sprintf(str, "\\r\r"); break;
		case 12: sprintf(str, "\\f\f"); break;
		case 7: sprintf(str, "\\a\a"); break;
		case 92: sprintf(str, "\\\\"); break;
		case 63: sprintf(str, "\\?\?"); break;
		case 39: sprintf(str, "\\'\""); break;
		case 34: sprintf(str, "\\\"\""); break;
		case 0: sprintf(str, "\\0\0"); break;
		default: sprintf(str, "%c", b); break;
		}
	}
	else
	{
		switch (b) {
		case 10: sprintf(str, "[10]\'\\n\'\n"); break;
		case 9: sprintf(str, "[9]\'\\t\'\t"); break;
		case 11: sprintf(str, "[11]\'\\v\'\v"); break;
		case 8: sprintf(str, "[8]\'\\b\'\b"); break;
		case 13: sprintf(str, "[13]\'\\r\'\r"); break;
		case 12: sprintf(str, "[12]\'\\f\'\f"); break;
		case 7: sprintf(str, "[7]\'\\a\'\a"); break;
		case 92: sprintf(str, "[92]\'\\\'\\"); break;
		case 63: sprintf(str, "[63]\'\\?\'\?"); break;
		case 39: sprintf(str, "[39]\'\\'\'\""); break;
		case 34: sprintf(str, "[34]\'\\\"\'\""); break;
		case 0: sprintf(str, "[0]\'\\0\'\0"); break;
		default: sprintf(str, "[%d]\'%c\'", b, b); break;
		}
	}

	return str;
}

// CHECK AUTODRIVER BOARD STATUS
int CheckAD_Status(uint16_t stat_reg, String stat_str)
{
	// Local vars
	String status_list[16] =
	{
		"HiZ",
		"BUSY",
		"SW_F",
		"SW_EVN",
		"DIR",
		"MOT_STATUS",
		"MOT_STATUS",
		"NOTPERF_CMD",
		"WRONG_CMD",
		"UVLO",
		"TH_WRN",
		"TH_SD",
		"OCD",
		"STEP_LOSS_A",
		"STEP_LOSS_B",
		"SCK_MOD"
	};
	byte bit_ind[2] = { 0, 0 };
	bool is_bit_set[2] = { false, false };
	uint16_t bit_val = 0x0;

	// Get id ind
	for (int i = 0; i < 16; i++)
	{
		if (stat_str == status_list[i])
		{
			bit_ind[is_bit_set[0] ? 1 : 0] = i;
			is_bit_set[is_bit_set[0] ? 1 : 0] = true;
		}
	}

	// Get bit value
	int n_loop = is_bit_set[1] ? 2 : 1;
	for (int i = 0; i < n_loop; i++)
	{
		uint16_t mask = 1 << bit_ind[i];
		uint16_t masked_n = stat_reg & mask;
		uint16_t k = masked_n >> bit_ind[i];
		bit_val |= bit_val & ~(1 << i) | (k << i);
	}

	// return bit value
	return (int)bit_val;
}

// CHECK LOOP TIME AND MEMORY
void CheckLoop(int free_mem)
{
	// Local static vars
	static int cnt_loop = 0;
	static bool first_run = true;
	static bool first_log = true;
	static int last_mem = 0;
	static uint32_t t_loop_last = millis();
	static int dt_loop_last = 0;

	cnt_loop++;
	if (
		first_log ||
		first_run ||
		cnt_loop % 1000 == 0
		)
	{
		// Get current vals
		int dt_mem = abs(free_mem - last_mem);
		uint32_t t_loop = millis();
		int dt_loop = t_loop - t_loop_last;
		int dt_dt_loop = abs(dt_loop - dt_loop_last);

		// Bail on first run
		if (first_run) {
			first_run = false;
			return;
		}

		// Check for big changes
		if (
			first_log ||
			dt_mem > 100 ||
			dt_dt_loop > 1000
			)
		{
			char str[200] = { 0 };
			sprintf(str, "LOOP CHECK: loop=%d dt=%lums free_ram=%dKB", cnt_loop, dt_loop, free_mem);
			DebugFlow(str);
			first_log = false;
		}

		// Store vars
		last_mem = free_mem;
		t_loop_last = t_loop;
		dt_loop_last = dt_loop;
	}
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size)
{
	// Return -1 if not found
	int ind = -1;
	for (int i = 0; i < arr_size; i++)
	{
		if (id == id_arr[i])
			ind = i;
	}

	// Print error if not found
	if (ind == -1) {
		char str[100];
		sprintf(str, "!!ERROR!! FeederDue::CharInd: ID \'%c\' Not Found", id);
		DebugError(str);
	}

	return ind;

}

// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink(int n_blinks, int dt_led)
{
	int duty[2] = { 100, 0 };
	bool is_on = false;
	int cnt_blnk = 0;

	// Flash sequentially
	while (cnt_blnk <= n_blinks)
	{
		analogWrite(pin.Disp_LED, duty[(int)is_on]);
		delay(dt_led);
		analogWrite(pin.TrackLED, duty[(int)is_on]);
		delay(dt_led);
		analogWrite(pin.RewLED_R, duty[(int)is_on]);
		delay(100);
		is_on = !is_on;
		cnt_blnk = !is_on ? cnt_blnk + 1 : cnt_blnk;
	}
	// Reset LEDs
	analogWrite(pin.Disp_LED, 0);
	analogWrite(pin.TrackLED, trackLEDduty);
	analogWrite(pin.RewLED_R, rewLEDmin);
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
		analogWrite(pin.RewLED_R, duty[(int)is_on]);
		analogWrite(pin.TrackLED, duty[(int)is_on]);
		delay(dt);
		is_on = !is_on;
	}
	// Reset LED
	analogWrite(pin.RewLED_R, 0);
	analogWrite(pin.TrackLED, trackLEDduty);
}

#pragma endregion


#pragma region ---------INTERUPTS---------

// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt() {

	// Exit if < 250 ms has not passed
	if (t_irProxDebounce > millis()) return;

	// Run stop in main loop
	doIRhardStop = true;

	// Update debounce
	t_irProxDebounce = millis() + 250;
}

// DETECT IR SYNC EVENT
void Interupt_IR_Detect()
{
	// Exit if < 25 ms has not passed
	if (millis() < t_irDetectDebounce) return;

	// Store time
	dt_ir = millis() - t_irSyncLast;
	t_irSyncLast += dt_ir;
	cnt_ir++;

	// Set flag
	if (t_sync != 0)
		doLogIR = true;

	// Update debounce
	t_irDetectDebounce = millis() + 50;
}

#pragma endregion

#pragma endregion


void setup() {



	// SET UP SERIAL STUFF
	delayMicroseconds(100);

	// XBee
	Serial1.begin(57600);

	// Serial monitor
	SerialUSB.begin(0);
	if (db.Console)
		while (!SerialUSB);

	// PRINT SETUP RUNNING
	char str[200];
	sprintf(str, "RUNNING FeederDue::Setup: Free RAM = %dKB", freeMemory());
	DebugFlow(str);

	// PRINT OBJECT INIT TIME
	/*
	sprintf(str, "FINISHED: Initilized RatVT(%dms) RobVT(%dms) RatPixy(%dms) Pid(%dms) Bull(%dms) Targ(%dms) Reward(%dms) Log(%dms)",
		RatVT.t_init, RobVT.t_init, RatPixy.t_init, Pid.t_init, Bull.t_init, Targ.t_init, Reward.t_init, Log.t_init);
	DebugFlow(str);
	*/

	// SHOW RESTART BLINK
	StatusBlink(1, 0);

	// SETUP OUTPUT PINS

	// Autodriver
	pinMode(pin.AD_CSP_R, OUTPUT);
	pinMode(pin.AD_CSP_F, OUTPUT);
	pinMode(pin.AD_RST, OUTPUT);
	// Display
	pinMode(pin.Disp_SCK, OUTPUT);
	pinMode(pin.Disp_MOSI, OUTPUT);
	pinMode(pin.Disp_DC, OUTPUT);
	pinMode(pin.Disp_RST, OUTPUT);
	pinMode(pin.Disp_CS, OUTPUT);
	pinMode(pin.Disp_LED, OUTPUT);
	// LEDs
	pinMode(pin.RewLED_R, OUTPUT);
	pinMode(pin.RewLED_C, OUTPUT);
	pinMode(pin.TrackLED, OUTPUT);
	// Relays
	pinMode(pin.Rel_Rew, OUTPUT);
	pinMode(pin.Rel_EtOH, OUTPUT);
	// BigEasyDriver
	pinMode(pin.ED_RST, OUTPUT);
	pinMode(pin.ED_SLP, OUTPUT);
	pinMode(pin.ED_DIR, OUTPUT);
	pinMode(pin.ED_STP, OUTPUT);
	pinMode(pin.ED_ENBL, OUTPUT);
	pinMode(pin.ED_MS1, OUTPUT);
	pinMode(pin.ED_MS2, OUTPUT);
	pinMode(pin.ED_MS3, OUTPUT);
	// OpenLog
	pinMode(pin.OL_RST, OUTPUT);
	// Feeder switch
	pinMode(pin.FeedSwitch_Gnd, OUTPUT);
	// Power off
	pinMode(pin.PwrOff, OUTPUT);
	delayMicroseconds(100);

	// Autodriver
	digitalWrite(pin.AD_CSP_R, LOW);
	digitalWrite(pin.AD_CSP_F, LOW);
	digitalWrite(pin.AD_RST, LOW);
	// Display
	digitalWrite(pin.Disp_SCK, LOW);
	digitalWrite(pin.Disp_MOSI, LOW);
	digitalWrite(pin.Disp_DC, LOW);
	digitalWrite(pin.Disp_RST, LOW);
	digitalWrite(pin.Disp_CS, LOW);
	digitalWrite(pin.Disp_LED, LOW);
	// LEDs
	digitalWrite(pin.RewLED_R, LOW);
	digitalWrite(pin.RewLED_C, LOW);
	digitalWrite(pin.TrackLED, LOW);
	// Relays
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);
	// OpenLog
	digitalWrite(pin.OL_RST, LOW);
	// Feeder switch
	digitalWrite(pin.FeedSwitch_Gnd, LOW);
	// Power off
	digitalWrite(pin.PwrOff, LOW);
	delayMicroseconds(100);

	// SET INPUT PINS

	// Voltage monitor
	pinMode(pin.BatVolt, INPUT);
	// IR proximity sensors
	pinMode(pin.IRprox_Rt, INPUT);
	pinMode(pin.IRprox_Lft, INPUT);
	// IR detector
	pinMode(pin.IRdetect, INPUT);

	// Set button and switch pins enable internal pullup
	for (int i = 0; i <= 2; i++) {
		pinMode(pin.Btn[i], INPUT_PULLUP);
	}
	pinMode(pin.FeedSwitch, INPUT_PULLUP);
	delayMicroseconds(100);

	// SETUP AUTODRIVER

	// Configure SPI
	AD_R.SPIConfig();
	delayMicroseconds(100);
	AD_F.SPIConfig();
	delayMicroseconds(100);
	// Reset boards
	AD_Reset();

	// Make sure motor is stopped and in high impedance
	AD_R.hardHiZ();
	AD_F.hardHiZ();

	// SETUP BIG EASY DRIVER

	// Set to 1/2 step mode
	digitalWrite(pin.ED_MS1, HIGH);
	digitalWrite(pin.ED_MS2, LOW);
	digitalWrite(pin.ED_MS3, LOW);

	// Start BigEasyDriver in sleep
	digitalWrite(pin.ED_RST, HIGH);
	digitalWrite(pin.ED_SLP, LOW);
	digitalWrite(pin.ED_DIR, LOW);
	digitalWrite(pin.ED_STP, LOW);
	digitalWrite(pin.ED_ENBL, LOW);

	// INITIALIZE LCD
	LCD.InitLCD();
	LCD.setFont(SmallFont);
	LCD.invert(true);

	// INITIALIZE PIXY
	Pixy.init();
	Wire.begin();

	// DUMP BUFFER
	while (Serial1.available() > 0)
		Serial1.read();

	// RESET VOLITILES AND RELAYS
	t_irProxDebounce = millis(); // (ms)
	t_irDetectDebounce = millis(); // (ms)
	t_irSyncLast = 0; // (ms)
	t_sync = 0; // (ms)
	dt_ir = 0;
	cnt_ir = 0;
	doIRhardStop = false;
	doLogIR = false;
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);

	// SETUP OPENLOG

	// Setup OpenLog
	DebugFlow("RUNNING: OpenLog Setup");
	if (Log.Setup())
		DebugFlow("FINISHED: OpenLog Setup");
	else
		DebugError("!!ERROR!! FeederDue::Setup: Aborted OpenLog Setup");
	// Create new log file
	if (Log.OpenNewLog() != 0) {
		sprintf(str, "FINISHED: Open Log File %s", Log.logFile);
		DebugFlow(str);
	}
	else
		DebugError("!!ERROR!! FeederDue::Setup: Aborted Open Log File");

	// DEFINE EXTERNAL INTERUPTS

	// IR prox right
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Rt), Interupt_IRprox_Halt, FALLING);
	// IR prox left
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Lft), Interupt_IRprox_Halt, FALLING);
	// IR detector
	uint32_t t_check_ir = millis() + 500;
	while (digitalRead(pin.IRdetect) == HIGH && t_check_ir < millis());
	if (digitalRead(pin.IRdetect) == LOW)
		attachInterrupt(digitalPinToInterrupt(pin.IRdetect), Interupt_IR_Detect, HIGH);
	else
	{
		// Skip ir sync setup
		t_sync = 1;
		DebugFlow("!!ERROR!! IR SENSOR DISABLED");
	}

	// CLEAR LCD
	ClearLCD();

	// PRINT SETUP FINISHED
	sprintf(str, "FINISHED FeederDue::Setup: Free RAM = %dKB", freeMemory());
	DebugFlow(str);

	// PRINT ALL IN QUEUE
	while (doPrint)
		PrintDebug();

	// TEMP
	int num = random(999);
	for (int i = 0; i < 25; i++)
	{
		char str[50];
		sprintf(str, "New Entry fuck %d ducks and stuff", num);
		Log.StoreLogEntry(str);
		num = random(999);
	}
}


void loop() {

#pragma region //--- ONGOING OPPERATIONS ---

	// Local vars
	static char horeStr[200] = { 0 };

	// PARSE SERIAL INPUT
	GetSerial();

	// CHECK LOOP TIME AND MEMORY
	CheckLoop(freeMemory());

	// SEND SERIAL DATA

	// Prioritize packet
	if (doPackSend)
	{
		SendPacketData();
	}
	// Send log
	else if (fc.doLogSend)
	{
		Log.SendLogEntry();
	}

	// PRINT DB
	if (doPrint)
	{
		PrintDebug();
	}

	// GET AD STATUS
	AD_CheckOC();

	// GET BUTTON INPUT
	if (GetButtonInput())
	{

		// Button triggered reward
		if (btn_doRew)
		{
			if (!Reward.isRewarding)
			{
				fc.isRewarding = Reward.StartRew(false, true);
				btn_doRew = false;
			}
		}

		// Open/close Rew sol
		if (btn_doRewSolStateChange)
		{
			OpenCloseRewSolenoid();
			btn_doRewSolStateChange = false;
		}

		// Open/close EtOH sol
		if (btn_doEtOHSolStateChange)
		{
			OpenCloseEtOHSolenoid();
			btn_doEtOHSolStateChange = false;
		}

		// Turn LCD on/off
		if (btn_doChangeLCDstate)
		{
			ChangeLCDlight();
			btn_doChangeLCDstate = false;
		}

	}

	// IR triggered halt
	if (doIRhardStop)
	{
		Function_IRprox_Halt();
		doIRhardStop = false;
	}

	// Log new ir events
	if (doLogIR)
	{
		// Log event if streaming started
		sprintf(horeStr, "IR Sync Event: tot=%d dt=%dms", cnt_ir, dt_ir);
		DebugFlow(horeStr, t_irSyncLast);

		// Reset flag
		doLogIR = false;
	}

	// End any ongoing reward
	if (fc.isRewarding)
	{
		if (Reward.EndRew())
		{
			fc.doRew = false;
			fc.isRewarding = false;

			// Tell CS what zone was rewarded and get confirmation
			if (
				Reward.mode != "Now" &&
				!Reward.isButtonReward
				)
				StorePacketData('c', 'Z', Reward.zoneIndByte + 1, 0, true);
		}
	}

	// Check if feeder arm should be moved
	Reward.CheckFeedArm();

	// Check if EtOH should be dispensed
	CheckEtOH();

	// Get and send voltage level
	GetBattVolt();

	// Check if ard data should be resent
	CheckResend('a');

	// Check if cs data should be resent
	CheckResend('c');

#pragma endregion

#pragma region //--- FIRST PASS SETUP ---
	if (fc.isFirstPass)
	{
		// Pulse tracker while in wait state
		static bool is_on = false;
		static int t_pulse_last = millis() - 1001;
		if (
			(is_on && millis() >= t_pulse_last + 10) ||
			(!is_on && millis() >= t_pulse_last + 1000)
			) {
			analogWrite(pin.TrackLED, is_on ? 0 : 100);
			is_on = !is_on;
			t_pulse_last = is_on ? millis() : t_pulse_last;
		}

		// Wait for first sync event to start
		if (t_sync == 0)
		{
			// Check for setup ir pulse
			if (
				abs(75 - dt_ir) < 10
				)
			{
				// Set sync time
				t_sync = t_irSyncLast;

				// Store and send CS handshake recieved
				StorePacketData('c', 'D', 255, c2r.packList[CharInd('+', c2r.idList, c2r.idLng)]);
				SendPacketData();

				// Log/print sync time
				sprintf(horeStr, "SET SYNC TIME: %dms", t_sync);
				DebugFlow(horeStr);
			}
			// Restart loop
			else return;
		}

		// Indicate setup complete
		StatusBlink(5);

		// CLEAR LCD
		ClearLCD();

		// Print ad board status
		sprintf(horeStr, "BOARD R STATUS: %04X", AD_R.getStatus());
		DebugFlow(horeStr);
		sprintf(horeStr, "BOARD F STATUS: %04X", AD_F.getStatus());
		DebugFlow(horeStr);

		fc.isFirstPass = false;
		DebugFlow("READY TO ROCK!");

	}

#pragma endregion

#pragma region //--- (T) SYSTEM TESTS ---

	if (c2r.idNew == 'T' && c2r.isNew)
	{
		// Store message data
		c2r.testCond = (byte)c2r.dat[0];
		c2r.testDat = (byte)c2r.dat[1];

		// Set run pid calibration flag
		if (c2r.testCond == 1)
		{
			do_pidCalibration = true;

			// Print settings
			sprintf(horeStr, "RUN PID CALIBRATION = kC=%0.2f", kC);
			DebugFlow(horeStr);
		}

		// Update Hault Error test run speed
		else if (c2r.testCond == 2)
		{
			float new_speed = float(c2r.testDat);
			float speed_steps = new_speed*cm2stp;


			if (new_speed > 0)
			{
				// Run motor
				AD_R.run(FWD, speed_steps);
				AD_F.run(FWD, speed_steps*scaleFrontAD);
			}
			else
			{
				// Halt robot
				AD_R.hardStop();
				AD_F.hardStop();
			}

			// Print speed
			sprintf(horeStr, "HAULT ERROR SPEED = %0.0f cm/sec", new_speed);
			DebugFlow(horeStr);
		}
	}

	// Run position debugging
	if (do_posDebug)
	{
		static float rat_rob_dist;
		if (millis() % 100 == 0)
		{
			rat_rob_dist = ekf.RatPos - ekf.RobPos;
			// Plot pos
			/*
			{@Plot.Pos.ratPixy.Green RatPixy.posNow}{@Plot.Pos.ratVT.Blue RatVT.posNow}{@Plot.Pos.ratEKF.Black ekf.RatPos}{@Plot.Pos.robVT.Orange RobVT.posNow}{@Plot.Pos.robEKF.Red ekf.RobPos}
			*/
			millis();
			// Turn on rew led when near setpoint
			if (Pid.error > -0.5 && Pid.error < 0.5) { analogWrite(pin.RewLED_C, 50); }
			else { analogWrite(pin.RewLED_C, 0); }
		}
	}

	// Run Pid calibration
	if (do_pidCalibration)
	{
		float new_speed = Pid.RunPidCalibration();
		float speed_steps;

		// Run motors
		if (Pid.cal_isPidUpdated)
		{
			if (new_speed >= 0)
			{
				speed_steps = new_speed*cm2stp;
				AD_R.run(FWD, speed_steps);
				AD_F.run(FWD, speed_steps*scaleFrontAD);
			}
			// Print values
			/*
			{Pid.cal_isCalFinished}{"ERROR"}{Pid.cal_errNow}{Pid.cal_errArr[0]}{Pid.cal_errArr[1]}{Pid.cal_errArr[2]}{Pid.cal_errArr[3]}{"PERIOD"}{Pid.cal_PcNow}{Pid.cal_cntPcArr[0]}{Pid.cal_PcArr[0]}{Pid.cal_cntPcArr[1]}{Pid.cal_PcArr[1]}{Pid.cal_cntPcArr[2]}{Pid.cal_PcArr[2]}{Pid.cal_cntPcArr[3]}{Pid.cal_PcArr[3]}{Pid.cal_PcAll}
			*/
			millis();
			// Plot error
			/*
				{@Plot.Vel.Error.Black Pid.error} {@Plot.Vel.Setpoint.Red 0}
			*/
			millis();
			// Reset flag
			Pid.cal_isPidUpdated = false;
		}
	}

#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (c2r.idNew == 'S' && c2r.isNew)
	{
		// Store message data
		c2r.sesCond = (byte)c2r.dat[0];
		c2r.soundCond = (byte)c2r.dat[1];
		millis();
		// Set condition
		if (c2r.sesCond == 1)
		{
			fc.isManualSes = false;
			DebugFlow("DO TRACKING");
		}
		else
		{
			fc.isManualSes = true;
			DebugFlow("DONT DO TRACKING");
		}
		// Set reward tone
		if (c2r.soundCond == 0)
		{
			// No sound
			StorePacketData('a', 's', 0);
			DebugFlow("NO SOUND");
		}
		else if (c2r.soundCond == 1)
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
		if (!fc.isManualSes &&
			isLitLCD)
		{
			btn_doChangeLCDstate = true;
		}
		// Clear LCD
		ClearLCD();
	}
#pragma endregion

#pragma region //--- (Q) DO QUIT ---
	if (c2r.idNew == 'Q' && c2r.isNew)
	{
		fc.doQuit = true;
		t_quitCmd = millis() + 1000;

		// Tell ard to quit
		StorePacketData('a', 'q');
		DebugFlow("DO QUIT");

		// Hold all motor control
		fc.isHalted = true;
		SetMotorControl("None", "MsgQ");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (
		fc.doQuit && millis() > t_quitCmd &&
		!CheckResend('a') &&
		!CheckResend('c') &&
		!doPackSend)
	{
		// Quit session
		DebugFlow("QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (c2r.idNew == 'M' && c2r.isNew)
	{
		// Store message data
		c2r.moveToTarg = c2r.dat[0];

		// Set flags
		fc.doMove = true;

		sprintf(horeStr, "DO MOVE: pos=%0.2fcm", c2r.moveToTarg);
		DebugFlow(horeStr);
	}

	// Perform movement
	if (fc.doMove)
	{

		// Compute move target
		if (!Targ.isTargSet)
		{
			// If succesfull
			if (Targ.CompTarg(ekf.RobPos, c2r.moveToTarg, -1 * feedDist))
			{
				// Start running
				if (
					SetMotorControl("MoveTo", "MsgM") &&
					RunMotor(Targ.moveDir, moveToSpeed, "MoveTo")
					)
				{
					// Print message
					sprintf(horeStr, "MOVING: from=%0.2fcm to=%0.2fcm by=%0.2fcm",
						Targ.posStart, Targ.offsetTarget, Targ.targDist);
					DebugFlow(horeStr);
				}
				// Reset motor cotrol if run fails
				else SetMotorControl("Open", "MsgM");
			}
		}

		// Check if robot is ready to be stopped
		if (Targ.isTargSet)
		{
			// Do deceleration
			float new_speed = Targ.DecelToTarg(ekf.RobPos, ekf.RobVel, 40, 10);

			// Change speed if > 0
			if (new_speed > 0)
			{
				RunMotor(Targ.moveDir, new_speed, "MoveTo");
			}
		}

		// Check if target reached or move aborted
		if (Targ.isTargReached || Targ.doAbortMove)
		{
			// Hard stop
			HardStop("MsgM");

			// Set motor cotrol to None
			SetMotorControl("None", "MsgM");

			// Reset flags
			fc.doMove = false;
			Targ.Reset();

			// Print final move status
			if (!Targ.doAbortMove)
			{
				// Tell CS movement is done
				StorePacketData('c', 'D', 255, c2r.packList[CharInd('M', c2r.idList, c2r.idLng)]);

				// Print success message
				sprintf(horeStr, "FINISHED MOVE: to=%0.2fcm within=%0.2fcm",
					Targ.offsetTarget, Targ.GetError(ekf.RobPos));
				DebugFlow(horeStr);
			}
			else
			{
				// Print failure message
				if (!fc.isEKFReady)
					DebugFlow("!!ERROR!! NO MOVE BECAUSE EKF NOT READY");
				sprintf(horeStr, "!!ERROR!! ABORTED MOVE: to=%0.2fcm within=%0.2fcm",
					Targ.offsetTarget, Targ.GetError(ekf.RobPos));
				DebugFlow(horeStr);
			}
		}
	}
#pragma endregion

#pragma region //--- (R) RUN REWARD ---
	if (c2r.idNew == 'R' && c2r.isNew)
	{
		// Store message data
		c2r.rewPos = c2r.dat[0];
		c2r.rewZoneInd = (byte)c2r.dat[1] - 1;
		c2r.rewDelay = (byte)c2r.dat[2];

		// Free reward
		if (
			c2r.rewPos > 0 &&
			c2r.rewZoneInd == 255
			)
		{
			// Set mode
			Reward.SetRewMode("Free", c2r.rewDelay);

			// REWARD zone
			sprintf(horeStr, "REWARD FREE: pos=%0.2fcm occ_thresh=%dms",
				c2r.rewPos, Reward.occThresh);
			DebugFlow(horeStr);
			fc.doRew = true;
		}

		// Cued reward
		else if (
			c2r.rewPos > 0 &&
			c2r.rewZoneInd != 255
			)
		{
			// Set mode
			Reward.SetRewMode("Cue", c2r.rewZoneInd);

			// Cued reward
			sprintf(horeStr, "REWARD CUED: pos=%0.2fcm occ_thresh=%ldms",
				c2r.rewPos, Reward.occThresh);
			DebugFlow(horeStr);
			fc.doRew = true;
		}

		// Imediate reward
		else if (c2r.rewPos == 0)
		{

			// Set mode
			Reward.SetRewMode("Now", c2r.rewZoneInd);
			DebugFlow("REWARD NOW");

			// Start reward
			fc.isRewarding = Reward.StartRew(true, false);

		}

	}

	// CHECK REWARD BOUNDS
	if (fc.doRew)
	{
		// If not rewarding 
		if (!Reward.isRewarding)
		{
			// Compute reward bounds
			if (!Reward.isBoundsSet)
			{
				Reward.CompZoneBounds(ekf.RatPos, c2r.rewPos);
				// Print message
				sprintf(horeStr, "SET REWARD ZONE: center=%0.2fcm from=%0.2fcm to=%0.2fcm",
					Reward.rewCenterRel, Reward.boundMin, Reward.boundMax);
				DebugFlow(horeStr);
			}
			else if (!Reward.isZoneTriggered)
			{
				if (Reward.CheckZoneBounds(ekf.RatPos))
				{
					// Start reward
					fc.isRewarding = Reward.StartRew(true, false);
					// Print message
					sprintf(horeStr, "REWARDED ZONE: occ=%dms zone=%0.2fcm from=%0.2fcm to=%0.2fcm",
						Reward.occRewarded, Reward.zoneRewarded, Reward.boundsRewarded[0], Reward.boundsRewarded[1]);
					DebugFlow(horeStr);
				}
			}
			// Check if rat passed all bounds
			if (
				Reward.isAllZonePassed &&
				!Reward.isZoneTriggered
				)
			{
				// Print reward missed
				sprintf(horeStr, "REWARD MISSED: rat=%0.2fcm bound_max=%0.2fcm",
					ekf.RatPos, Reward.boundMax);
				DebugFlow(horeStr);

				// Reset flags
				Reward.Reset();
				fc.doRew = false;
			}
		}
	}

#pragma endregion

#pragma region //--- (H) HALT ROBOT STATUS ---
	if (c2r.idNew == 'H' && c2r.isNew)
	{
		// Store message data
		fc.doHalt = c2r.dat[0] != 0 ? true : false;

		if (fc.doHalt)
		{
			DebugFlow("HALT STARTED");
			// Stop pid and set to manual
			HardStop("MsgH");
			// Remove motor control
			fc.isHalted = true;
			SetMotorControl("None", "MsgH");
			fc.doHalt = false;
		}
		else
		{
			DebugFlow("HALT FINISHED");
			// Open motor control
			fc.isHalted = false;
			SetMotorControl("Open", "MsgH");
		}
	}
#pragma endregion

#pragma region //--- (B) BULLDOZE RAT STATUS ---
	if (c2r.idNew == 'B' && c2r.isNew)
	{
		// Store message data
		c2r.bullDel = (byte)c2r.dat[0];
		c2r.bullSpeed = (byte)c2r.dat[1];

		// Local vars
		bool is_mode_changed = false;

		// Reinitialize bulldoze
		Bull.Reinitialize(c2r.bullDel, c2r.bullSpeed, "MsgB");

		// Check if mode should be changedchanged
		if (c2r.bullSpeed > 0)
		{
			// Mode changed
			if (!fc.doBulldoze)
			{
				is_mode_changed = true;
				fc.doBulldoze = true;
				DebugFlow("SET BULLDOZE ON");
			}
			// Only settings changed
			else is_mode_changed = false;
		}
		else
		{
			// Mode changed
			if (fc.doBulldoze)
			{
				is_mode_changed = true;
				fc.doBulldoze = false;
				DebugFlow("SET BULLDOZE OFF");
			}
			// Only settings changed
			else is_mode_changed = false;
		}

		// Don't exicute until rat is in and mode is changed
		if (fc.isTrackingEnabled &&
			is_mode_changed)
		{
			if (fc.doBulldoze)
			{
				// Turn bulldoze on
				Bull.TurnOn("MsgB");
				DebugFlow("BULLDOZE ON");
			}
			else
			{
				// Turn bulldoze off
				Bull.TurnOff("MsgB");
				DebugFlow("BULLDOZE OFF");
			}
		}

	}
#pragma endregion

#pragma region //--- (I) START/STOP PID ---
	if (c2r.idNew == 'I' && c2r.isNew)
	{
		// Store message data
		fc.isRatIn = c2r.dat[0] != 0 ? true : false;

		if (fc.isRatIn)
		{
			// Reset all vt data
			RatVT.SetPos(0, 0);
			RatPixy.SetPos(0, 0);
			RobVT.SetPos(0, 0);

			// Pid started by InitializeTracking()
			DebugFlow("RAT IN");
		}
		else
		{

			// Turn off bulldoze
			Bull.TurnOff("MsgI");
			fc.doBulldoze = false;

			// Turn off pid
			Pid.Stop("MsgI");

			// Set motor control to "None"
			SetMotorControl("None", "MsgI");

			// Halt robot 
			HardStop("MsgI");

			// Set to stop tracking
			fc.isTrackingEnabled = false;

			DebugFlow("RAT OUT");
		}
	}
#pragma endregion

#pragma region //--- (V) GET STREAM STATUS ---
	if (c2r.idNew == 'V' && c2r.isNew)
	{
		fc.doStreamCheck = true;
	}

	// Check for streaming
	if (fc.doStreamCheck && fc.isStreaming)
	{
		StorePacketData('c', 'D', 255, c2r.packList[CharInd('V', c2r.idList, c2r.idLng)]);
		fc.doStreamCheck = false;
		DebugFlow("STREAMING CONFIRMED");
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (c2r.idNew == 'P' && c2r.isNew)
	{

		// Update vt pos data
		if (c2r.vtEnt == 0)
		{
			RatVT.UpdatePos(c2r.vtCM[c2r.vtEnt], c2r.vtTS[c2r.vtEnt]);
		}
		else
		{
			RobVT.UpdatePos(c2r.vtCM[c2r.vtEnt], c2r.vtTS[c2r.vtEnt]);
		}
	}
#pragma endregion

#pragma region //--- (L) SEND LOG ---
	if (c2r.idNew == 'L' && c2r.isNew)
	{
		// Store message data
		fc.doLogResend = c2r.dat[0] == 0 ? true : false;

		// Stop logging
		db.Log = false;

		// Save first log request packet for later
		Log.donePack = Log.donePack != 0 ? Log.donePack :
			c2r.packList[CharInd('L', c2r.idList, c2r.idLng)];

		// Check if end of list reached
		if (!Log.isAllSent)
			fc.doLogSend = true;

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
	float new_speed = Pid.UpdatePID();

	if (new_speed == 0)
	{
		HardStop("Pid");
	}
	else if (new_speed > 0)
	{
		RunMotor('f', new_speed, "Pid");
	}

	// UPDATE BULLDOZER
	Bull.UpdateBull();

#pragma endregion

}
