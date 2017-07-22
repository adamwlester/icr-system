// ######################################

// ============= FEEDERDUE ==============

// ######################################

// NOTES:
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
	double = 8 byte

* config.txt settings:
	57600,26,3,2,0,0,0
	baud,escape,esc#,mode,verb,echo,ignoreRX

*/


#pragma region ========== LIBRARIES & EXT DEFS =========


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


#pragma region ============ DEBUG SETTINGS =============

// LOG DEBUGGING
struct DB
{
	// Do log
	bool Log = true;
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
	bool Console = false;
	bool LCD = false;
	// What to print
	const bool print_errors = true;
	const bool print_flow = true;
	const bool print_motorControl = false;
	const bool print_c2r = false;
	const bool print_r2c = false;
	const bool print_r2a = false;
	const bool print_pid = false;
	const bool print_bull = false;
	const bool print_logging = false;
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
const float kC = 5; // critical gain [1.5,2,3,4,5]
const float pC = 1.5; // oscillation period [2.75,0,1.9,0,1.5]  
const float cal_speedSteps[4] = { 10, 20, 30, 40 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;
bool do_pidCalibration = false;

#pragma endregion


#pragma region ============= VARIABLE SETUP ============

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
	const int KillSwitch = 45;

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
	bool isSesStarted = false;
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
	bool doLogSend = false;
	bool isEKFReady = false;
	bool doPrint = false;
	bool doBlockLCDlog = false;
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

// Print debugging
const int printQueueRows = 15;
const int printQueueCols = 200;
char printQueue[printQueueRows][printQueueCols] = { { 0 } };
int printQueueInd = printQueueRows;

// Debug tracking
int cnt_droppedPacks = 0;
int cnt_overflowEvt = 0;
int cnt_timeoutEvt = 0;
int cnt_packResend = 0;

// Serial com general
const int sendQueueRows = 10;
const int sendQueueCols = 8;
byte sendQueue[sendQueueRows][sendQueueCols] = { { 0 } };
int sendQueueInd = sendQueueRows - 1;
const int resendMax = 3;
const int dt_resend = 100; // (ms)
const int dt_sendSent = 1; // (ms)
const int dt_sendRcvd = 1; // (ms)
const int dt_logSent = 1; // (ms)
const int dt_logRcvd = 1; // (ms)
uint32_t t_sent = millis(); // (ms)
uint32_t t_rcvd = millis(); // (ms)

// Serial from CS
struct C2R
{
	const char head = '<';
	const char foot = '>';
	const char idList[14] = {
		'+', // Setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
		'H', // halt movement
		'B', // bulldoze rat
		'I', // rat in/out
		'V', // request stream status
		'L', // request log send/resend
		'J', // battery voltage
		'Z', // reward zone
		'P', // position data
	};
	const static int idLng = sizeof(idList) / sizeof(idList[0]);
	uint16_t packList[idLng] = { 0 };
	int cntRepeat[idLng] = { 0 };
	char idNew = '\0';
	bool isNew = false;
	float dat[3] = { 0, 0, 0 };
	uint16_t packLast = 0;

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
	const char idList[14] = {
		'+', // Setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
		'H', // halt movement
		'B', // bulldoze rat
		'I', // rat in/out
		'V', // connected and streaming
		'L', // request log send/resend
		'J', // battery voltage
		'Z', // reward zone
		'D', // execution done
	};
	const char head = '<';
	const char foot = '>';
	const static int idLng = sizeof(idList) / sizeof(idList[0]);
	uint16_t packList[idLng] = { 0 };
	byte datList[idLng] = { 0 };
	uint32_t t_sentList[idLng] = { 0 };
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
	uint32_t t_sentList[idLng] = { 0 };
	bool doRcvCheck[idLng] = { false };
	int resendCnt[idLng] = { 0 };
}
// Initialize
r2a;

// Serial from other ard
struct A2R
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
const double cm2stp = 200 / (9 * PI);
const double stp2cm = (9 * PI) / 200;
const float maxSpeed = 100; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)
const double scaleFrontAD = 1.0375;
const byte kAcc = 60 * 2;
const byte kDec = 60 * 2;
const byte kRun = 60;
const byte kHold = 60 / 2;
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 10; // (ms)
double runSpeedNow = 0;
char runDirNow = 'f';

// Kalman model measures
struct EKF
{
	double RatPos = 0;
	double RobPos = 0;
	double RatVel = 0;
	double RobVel = 0;
}
ekf;

// Pid Settings
bool doIncludeTerm[2] = { true, true };
const float pidSetPoint = 42; // (cm)
const float guardDist = 4.5;
const float feedDist = 66;

// Movement
float moveToSpeed = 80;

// REWARD
const int dt_rewBlockMoveRng[2] = { 2500, 10000 }; // (ms)
uint32_t t_rewBlockMove = 0; // (ms)

// Solonoids
/*
EtOH run after min time or distance
*/
const int dt_durEtOH = 500; // (ms)
const int dt_delEtOH = 10000; // (ms)
uint32_t t_solOpen = 0;
uint32_t t_solClose = 0;

// Volt tracking
/*
Updated when EtOH relay opened
*/
const float bit2volt = 0.0164;
uint32_t t_voltUpdate = 0;
float voltNow = 0;
float voltCutoff = 11.6;
float batVoltArr[100] = { 0 };

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

#pragma endregion 


#pragma region ========== CLASS DECLARATIONS ===========

#pragma region ----------CLASS: POSTRACK----------
class POSTRACK
{
public:
	// VARS
	char objID[20] = { 0 };
	int nSamp = 0;
	double posArr[6] = { 0,0,0,0,0,0 }; // (cm)
	uint32_t t_tsArr[6] = { 0,0,0,0,0,0 }; // (ms)
	int dt_skip = 0;
	double velNow = 0.0f; // (cm/sec)
	double posNow = 0.0f; // (cm)
	double posAbs = 0.0f; // (cm)
	uint32_t t_tsNow = 0;
	uint32_t t_msNow = millis();
	int nLaps = 0;
	int sampCnt = 0;
	bool isDataNew = false;
	uint32_t t_init;
	// METHODS
	POSTRACK(uint32_t t, char obj_id[], int n_samp);
	void UpdatePos(double pos_new, uint32_t ts_new);
	double GetPos();
	double GetVel();
	void SetDat(double set_pos, uint32_t t);
	void SetPos(double set_pos, int set_laps);
};

#pragma endregion 

#pragma region ----------CLASS: PID----------
class PID
{

public:
	// VARS
	uint32_t t_lastLoop = 0;
	double dt_loop = 0;
	bool isLoopRan = false;
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
	uint32_t t_ekfStr = 0;
	int dt_ekfSettle = 250; // (ms)
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
	float cal_dt_loop = 0;
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
	void SetLoopTime(uint32_t t);
	void PrintPID(char msg[]);
	double RunPidCalibration();
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

#pragma region ----------CLASS: TARGET----------
class TARGET
{
public:
	// VARS
	int targSetTimeout = 1000;
	int moveTimeout = 10000;
	uint32_t t_tryTargSetTill = 0;
	uint32_t t_tryMoveTill = 0;
	bool doAbortMove = false;
	double posRel = 0;
	float minSpeed = 0;
	const int dt_update = 10;
	uint32_t t_updateNext = 0;
	double distLeft = 0;
	double posStart = 0;
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
	TARGET(uint32_t t);
	bool CompTarg(double now_pos, float targ_pos);
	double DecelToTarg(double now_pos, double now_vel, float dec_pos, double speed_min);
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
	String mode = "None"; // ["None" "Free" "Cue" "Now"]
	int occThresh = 0; // (ms)
	int dt_block = 0; // (ms)
	int duration = 0; // (ms) 
	byte durationByte = 0; // (ms) 
	int zoneMin = 0;
	int zoneMax = 0;
	double boundMin = 0;
	double boundMax = 0;
	uint32_t t_closeSol = 0;
	uint32_t t_retractArm = 0;
	uint32_t t_moveArmStr = 0;
	double rewCenterRel = 0;
	bool isRewarding = false;
	bool isButtonReward = false;
	bool isBoundsSet = false;
	bool isZoneTriggered = false;
	bool isAllZonePassed = false;
	bool is_ekfNew = false;
	byte zoneIndByte = 255;
	int zoneRewarded = 0;
	double boundsRewarded[2] = { 0 };
	int occRewarded = 0;
	int lapN = 0;
	bool doArmMove = false;
	bool isArmExtended = false;
	const int armExtStps = 220;
	int armPos = 0;
	int armZone = 0;
	bool isArmStpOn = false;
	uint32_t t_init;
	// METHODS
	REWARD(uint32_t t);
	bool StartRew(bool do_stop, bool is_button_reward = false);
	bool EndRew();
	void SetRewDur(byte zone_ind);
	void SetRewMode(String mode_str, byte arg2);
	bool CompZoneBounds(double now_pos, double rew_pos);
	bool CheckZoneBounds(double now_pos);
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
	static const int maxBytesStore = 2500;
	char rcvdArr[maxBytesStore] = { 0 };
	char mode = '\0'; // ['<', '>']
	char openLgSettings[100] = { 0 };
	char logFile[50] = { 0 };
	int logNum = 0;
	char logCntStr[10] = { 0 };
	int bytesStored = 0;
	int bytesSent = 0;
	uint16_t donePack = 0;
	uint32_t t_init;
	// METHODS
	LOGGER(uint32_t t);
	bool Setup();
	int OpenNewLog();
	bool SetToCmdMode();
	void GetCommand();
	char SendCommand(char msg[], uint32_t timeout = 5000, bool do_conf = true);
	char GetReply(uint32_t timeout);
	bool SetToLogMode(char log_file[]);
	bool StoreLogEntry(char msg[], uint32_t t = millis());
	void SendLogEntry();
	void TestLoad(int n_entry = 0, char log_file[] = '\0');
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
REWARD Reward(millis());
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
void QueuePacket(char targ, char id, byte d1 = 255, uint16_t pack = 0, bool do_conf = false);
// SEND SERIAL PACKET DATA
void SendPacket();
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
void UpdatePixyPos();
// UPDATE EKF
void UpdateEKF();
// CHECK FOR BUTTON INPUT
bool GetButtonInput();
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
// QUIT AND RESTART ARDUINO
void QuitSession();
// HOLD FOR CRITICAL ERRORS
void RunErrorHold(char msg[], uint32_t t_kill = 0);
// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t = millis());
// LOG/PRINT ERRORS
void DebugError(char msg[], uint32_t t = millis());
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
bool PrintDebug();
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


#pragma region =========== CLASS DEFINITIONS ===========

#pragma region ----------CLASS: POSTRACK----------

POSTRACK::POSTRACK(uint32_t t, char obj_id[], int n_samp)
{
	this->t_init = t;
	strcpy(objID, obj_id);
	this->nSamp = n_samp;
	for (int i = 0; i < n_samp; i++) this->posArr[i] = 0.0f;
}

void POSTRACK::UpdatePos(double pos_new, uint32_t ts_new)
{
	// Local vars
	static int cnt_error = 0;
	static bool is_error_last = false;
	double pos_diff = 0;
	double dist = 0;
	double dist_sum = 0;
	int dt = 0;
	int dt_sum = 0;
	double dt_sec = 0;
	double vel_diff = 0;
	double vel = 0;

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
		this->velNow = 0;

		// Set flag
		isDataNew = false;

		// Bail 
		return;
	}
	// Ready to use
	else
		isDataNew = true;

	// COMPUTE TOTAL DISTANCE RAN

	// Check for zero crossing
	pos_diff = pos_new - this->posArr[this->nSamp - 2];
	if (abs(pos_diff) > 140 * (PI / 2))
	{
		// Crossed over
		if (pos_diff < 0)
			this->nLaps++;

		// Crossed back
		else
			this->nLaps--;
	}

	// Store cumulative position in cm
	this->posNow = pos_new + this->nLaps*(140 * PI);

	// COMPUTE VELOCITY
	for (int i = 0; i < this->nSamp - 1; i++)
	{
		// Compute total distance
		dist = this->posArr[i + 1] - this->posArr[i];
		dist_sum += dist;

		// Compute total time
		dt = this->t_tsArr[i + 1] - this->t_tsArr[i];
		dt_sum += dt;
	}

	// Convert dt to sec 
	dt_sec = (double)dt_sum / 1000;

	// Compute vel
	vel = dist_sum / dt_sec;

	// Compute change in vel
	vel_diff = abs(velNow - vel);

	// Check for problem vals
	bool do_skip_vel =
		dist_sum == 0 ||
		dt_sec == 0 ||
		(dt_skip < 500 && vel_diff > 150);

	// Ignore outlyer values unless too many frames discarted
	if (!do_skip_vel) {
		this->velNow = vel;
		this->dt_skip = 0;
	}
	// Add to skip time
	else
		this->dt_skip += dt;

	// Check for errors
	bool is_error =
		dt_sec == 0 ||
		vel_diff > 150;

	// Log/print error
	if (is_error) {
		cnt_error++;
		if (!is_error_last) {
			char str[200];
			sprintf(str, "!!ERROR!! [POSTRACK::UpdatePos] %s Update Failed: dist_sum=%0.2f dt_sec=%0.2f vel_diff=%0.2f errors=%d", objID, dist_sum, dt_sec, vel_diff, cnt_error);
			DebugError(str);
		}
		is_error_last = true;
	}
	else
		is_error_last = false;

}

double POSTRACK::GetPos()
{
	isDataNew = false;
	return posNow;
}

double POSTRACK::GetVel()
{
	return velNow;
}

void POSTRACK::SetDat(double set_pos, uint32_t t)
{
	// Compute ts
	uint32_t ts = t_tsNow + (t - t_msNow);

	// Update pos
	UpdatePos(set_pos, ts);
}

void POSTRACK::SetPos(double set_pos, int set_laps)
{
	posNow = set_pos;
	nLaps = set_laps;
	sampCnt = 0;
	isDataNew = false;
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

double PID::UpdatePID()
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
					PrintPID("[PID::UpdatePID] First Run");
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
	SetMotorControl("Pid", "PID::Run");

	// Reset
	Reset();
	mode = "Automatic";

	// Tell ard pid is running
	QueuePacket('a', 'p', 1);

	// Print event
	char str[100] = { 0 };
	sprintf(str, "[PID::Run] Run [%s]", called_from);
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
		SetMotorControl("Open", "PID::Stop");
	}

	// Tell ard pid is stopped
	if (
		// Dont send if rewarding
		!fc.isRewarding
		)
	{
		QueuePacket('a', 'p', 0);
	}

	// Set mode
	mode = "Manual";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "[PID::Stop] Stop [%s]", called_from);
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
	sprintf(str, "[PID::Hold] Hold [%s]", called_from);
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
			sprintf(str, "[PID::SetThrottle] Throttle ACC to %0.2fcm/sec", throttleAcc);
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
			PrintPID("[PID::CheckThrottle] Finished Throttle");
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
		PrintPID("[PID::CheckMotorControl] Take Motor Control [PID::CheckMotorControl]");

		// Run pid
		Run("PID::CheckMotorControl");
	}
	else if ((fc.motorControl != "Pid" && fc.motorControl != "Open") &&
		mode == "Automatic")
	{
		// Print taking conrol
		PrintPID("[PID::CheckMotorControl] Surender Motor Control [PID::CheckMotorControl]");

		// Hold pid
		Hold("PID::CheckMotorControl");
	}
}

void PID::CheckSetpointCrossing()
{
	// Check if rat has moved in front of setpoint
	if (isHolding4cross && error > 0)
	{
		isHolding4cross = false;
		PrintPID("[PID::CheckSetpointCrossing] Crossed Setpoint");
	}
}

void PID::CheckEKF(uint32_t t)
{
	if (!fc.isEKFReady)
	{
		if ((t - t_ekfStr) > dt_ekfSettle)
		{
			fc.isEKFReady = true;
			// Log/print
			PrintPID("[PID::CheckEKF] EKF Ready");
		}
	}
}

void PID::ResetEKF(char called_from[])
{
	fc.isEKFReady = false;
	t_ekfStr = millis();

	// Log/print
	char str[100] = { 0 };
	sprintf(str, "[PID::ResetEKF] Reset EKF [%s]", called_from);
	PrintPID(str);
}

void PID::SetLoopTime(uint32_t t)
{
	is_ekfNew = true;
	if (isLoopRan)
	{
		dt_loop = (double)(t - t_lastLoop) / 1000;
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

double PID::RunPidCalibration()
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
		else if (millis() > t_lastLoop + cal_dt_min)
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
			cal_dt_loop = (float)(millis() - t_lastLoop) / 1000.0f;
			t_lastLoop = millis();

			// Compute Pid
			cal_ratVel = cal_speedSteps[cal_stepNow];
			cal_ratPos += cal_ratVel * (cal_dt_loop / 1);

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

			// Get distance traveled
			distMoved = posNow - posCheck;

			// Check for movement
			isMoved = distMoved >= moveMin ? true : false;

			// Check if rat passed reset
			double error = ekf.RatPos - (ekf.RobPos + pidSetPoint);
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
					Run("BULLDOZE::UpdateBull");
				}
			}
			// Has moved minimal distance
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
					Stop("BULLDOZE::UpdateBull");
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
	sprintf(str, "[BULLDOZE::Reinitialize] Reinitialize Bull [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Run(char called_from[])
{
	// Take control
	SetMotorControl("Bull", "BULLDOZE::Run");

	// Start bulldozer
	RunMotor('f', bSpeed, "Bull");

	// Tell ard bull is running
	QueuePacket('a', 'b', 1);

	// Set mode
	mode = "Active";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "[BULLDOZE::Run] Run [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Stop(char called_from[])
{
	// Stop movement
	RunMotor('f', 0, "Bull");

	// Give over control
	if (fc.motorControl == "Bull")
	{
		SetMotorControl("Open", "BULLDOZE::Stop");
	}

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Tell ard bull is stopped
	QueuePacket('a', 'b', 0);

	// Set mode
	mode = "Inactive";

	// Print event
	char str[100] = { 0 };
	sprintf(str, "[BULLDOZE::Stop] Stop [%s]", called_from);
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
	sprintf(str, "[BULLDOZE::TurnOn] Turn On [%s]", called_from);
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
		Stop("BULLDOZE::TurnOff");
	}

	// Print event
	char str[100] = { 0 };
	sprintf(str, "[BULLDOZE::TurnOff] Turn Off [%s]", called_from);
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
		Stop("BULLDOZE::Hold");

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
	sprintf(str, "[BULLDOZE::Resume] Resume [%s]", called_from);
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
		Resume("BULLDOZE::CheckMotorControl");
	}
	else if ((fc.motorControl != "Bull" && fc.motorControl != "Pid" && fc.motorControl != "Open") &&
		state == "On")
	{
		// Turn bull off
		Hold("BULLDOZE::CheckMotorControl");
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

bool TARGET::CompTarg(double now_pos, float targ_pos)
{
	// Run only if targ not set
	if (isTargSet)
		return isTargSet;

	// Local vars
	char str[200] = { 0 };
	double move_diff = 0;
	int circ = 0;
	int pos = 0;

	// Copy to public vars
	posStart = now_pos;
	targPos = targ_pos;

	// Get abort timeout
	if (t_tryTargSetTill == 0)
		t_tryTargSetTill = millis() + targSetTimeout;

	// Check if time out reached
	if (millis() > t_tryTargSetTill) {
		doAbortMove = true;
		// Log/print error
		sprintf(str, "!!ERROR!! [TARGET::CompTarg] Timedout after %dms", targSetTimeout);
		DebugError(str);
		return isTargSet;
	}

	// Bail if ekf pos data not ready
	if (!fc.isEKFReady)
		return isTargSet;

	// Current relative pos on track
	circ = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posRel = (double)(pos % circ) / 100;

	// Diff and absolute distance
	move_diff = targPos - posRel;
	move_diff = move_diff < 0 ? move_diff + (140 * PI) : move_diff;

	// Get minimum distance to target
	targDist = min((140 * PI) - abs(move_diff), abs(move_diff));

	// Set to negative for reverse move
	if ((move_diff > 0 && abs(move_diff) == targDist) ||
		(move_diff < 0 && abs(move_diff) != targDist))
		moveDir = 'f';
	else
		moveDir = 'r';

	// Set vars for later
	t_updateNext = millis();
	baseSpeed = 0;

	// Set flag true
	isTargSet = true;

	// Log/print
	sprintf(str, "[TARGET::CompTarg] FINISHED: Set Target: start=%0.2fcm targ=%0.2fcm dist_move=%0.2fcm", posStart, targPos, targDist);
	DebugFlow(str);

	// Retern flag
	return isTargSet;
}

double TARGET::DecelToTarg(double now_pos, double now_vel, float dec_pos, double speed_min)
{
	// Local vars
	char str[200] = { 0 };
	double new_speed = 0;

	// Run if targ not reached
	if (isTargReached)
		return 0;

	// Get abort timeout
	if (t_tryMoveTill == 0)
		t_tryMoveTill = millis() + moveTimeout;

	// Check if time out reached
	if (millis() > t_tryMoveTill) {
		doAbortMove = true;
		// Log/print error
		sprintf(str, "!!ERROR!! [TARGET::DecelToTarg] Timedout after %dms", moveTimeout);
		DebugError(str);
		return 0;
	}

	// Compute remaining distance
	distLeft = targDist - abs(now_pos - posStart);

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
			new_speed = (distLeft / dec_pos) * baseSpeed;

			// Maintain at min speed
			if (new_speed < speed_min) new_speed = speed_min;

			// Update loop time
			t_updateNext = millis() + dt_update;
		}

	}

	// TARGET reached
	if (distLeft < 1)
	{
		// Set flag true
		isTargReached = true;

		// Stop movement
		new_speed = 0;

		// Log/print
		sprintf(str, "[TARGET::DecelToTarg] FINISHED: MoveTo: start=%0.2fcm targ=%0.2fcm dist_move=%0.2fcm dist_left=%0.2fcm", posStart, targPos, targDist, distLeft);
		DebugFlow(str);
	}

	// Return new speed
	return new_speed;
}

double TARGET::GetError(double now_pos)
{
	// Local vars
	int diam = 0;
	int pos = 0;

	// Current relative pos on track
	diam = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posRel = (double)(pos % diam) / 100;

	// TARGET error
	return targPos - posRel;
}

void TARGET::Reset()
{
	isTargSet = false;
	isMoveStarted = false;
	isTargReached = false;
	doAbortMove = false;
	t_tryTargSetTill = 0;
	t_tryMoveTill = 0;
}

#pragma endregion 

#pragma region ----------CLASS: REWARD----------

REWARD::REWARD(uint32_t t)
{
	this->t_init = t;
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
		// Hard stop
		HardStop("StartRew");

		// Set hold time
		if (ekf.RatPos == 0)
			dt_block = dt_rewBlockMoveRng[0];
		else
			dt_block = dt_rewBlockMoveRng[1];
		BlockMotorTill(dt_block, "REWARD::StartRew");
	}

	// Store and send packet imediately
	QueuePacket('a', 'r', durationByte);
	SendPacket();

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
		fc.doBlockLCDlog = true;
	}
	else
	{
		char str[100] = { 0 };
		sprintf(str, "{REWARD::StartRew] RUNNING: Reward for %dms...", duration);
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
			DebugFlow("[REWARD::EndRew] FINISHED: Reward");
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

bool REWARD::CompZoneBounds(double now_pos, double rew_pos)
{
	// Run only if bounds are not set
	if (!isBoundsSet)
	{
		// Local vars
		int diam = 0;
		int pos_int = 0;
		double pos_rel = 0;
		double dist_center_cm = 0;
		double dist_start_cm = 0;
		double dist_end_cm = 0;

		// Compute laps
		diam = (int)(140 * PI * 100);
		pos_int = (int)(now_pos * 100);
		lapN = round(now_pos / (140 * PI) - (float)(pos_int % diam) / diam);
		// Check if rat 'ahead' of rew pos
		pos_rel = (double)(pos_int % diam) / 100;
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

bool REWARD::CheckZoneBounds(double now_pos)
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
		// Set to half step
		if (digitalRead(pin.ED_MS2) == HIGH)
			digitalWrite(pin.ED_MS2, LOW);
	}
}

void REWARD::RetractFeedArm()
{
	if (isArmExtended)
	{
		armZone = 0;
		doArmMove = true;
		// Set to quarter step
		if (digitalRead(pin.ED_MS2) == LOW)
			digitalWrite(pin.ED_MS2, HIGH);
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
		if (armPos < armZone) {
			digitalWrite(pin.ED_DIR, LOW); // extend
		}

		// Retract arm
		else
		{
			if (digitalRead(pin.FeedSwitch) == HIGH) {
				digitalWrite(pin.ED_DIR, HIGH); // retract
			}
			// Home pos reached
			else {
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
		char str1[200] = "[REWARD::Reset] ZONE OCC:";
		char str2[200] = "[REWARD::Reset] ZONE CNT:";
		for (int i = zoneMin; i <= zoneMax; i++)
		{
			char ss1[50];
			sprintf(ss1, " z%d=%dms", i + 1, zoneOccTim[i]);
			strcat(str1, ss1);
			char ss2[50];
			sprintf(ss2, " z%d=%d", i + 1, zoneOccCnt[i]);
			strcat(str2, ss2);
		}
		DebugFlow(str1);
		DebugFlow(str2);
	}

	// Reset flags etc
	mode = "None";
	isRewarding = false;
	isBoundsSet = false;
	isZoneTriggered = false;
	isAllZonePassed = false;
	is_ekfNew = false;
	dt_block = dt_rewBlockMoveRng[0];

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
	baud,escape,esc#,mode,verb,echo,ignoreRX
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
	if (!pass) {
		DebugError("!!ERROR!! [LOGGER::Setup] ABORTING: Did Not Get Initial \'>\' or \'<\'");
		return false;
	}

	// Set to command mode;
	if (match == '<')
		pass = SetToCmdMode();

	// Turn off verbose mode and echo mode
	SendCommand("verbose off\r");
	SendCommand("echo off\r");

	// Get settings
	sprintf(str, "get\r");
	if (SendCommand(str) == '!')
		pass = false;
	else
		strcpy(openLgSettings, rcvdArr);

	return pass;
}

int LOGGER::OpenNewLog()
{
	// Local vars
	byte match = 0;
	char str[100] = { 0 };
	char new_file[50] = { 0 };

	// Make sure in command mode
	if (!SetToCmdMode())
		return 0;

	// Check/cd to log directory
	if (SendCommand("cd LOGS\r") == '!')
	{
		DebugFlow("[OpenNewLog] Make \"LOGS\" Directory");
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

	// Check if more than 100 logs saved
	if (logNum > 100)
	{
		// Step out of directory
		if (SendCommand("cd ..\r") == '!')
			return 0;
		// Delete directory
		if (SendCommand("rm -rf LOGS\r") == '!')
			return 0;
		// Print 
		sprintf(str, "[OpenNewLog] Deleted Log Directory: log_count=%d", logNum);
		DebugFlow(str);
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
	sprintf(new_file, "LOG%05u.CSV", logNum);

	// Begin logging to this file
	if (!SetToLogMode(new_file))
		return 0;

	// Write first log entry
	sprintf(str, "Begin Logging to \"%s\"", logFile);
	StoreLogEntry(str);

	// Return log number
	return logNum;

}

bool LOGGER::SetToCmdMode()
{
	// Local vars
	bool pass = false;
	char rstArr[3] = { 26,26,26 };

	// Check if already in cmd mode
	if (mode == '>')
		return true;

	// Add delay if log just sent
	delay(500);

	//Send "$$$" command mode
	if (SendCommand(rstArr, 500, true) == '>')
	{
		// Set flag
		pass = true;
		// Pause to let OpenLog get its shit together
		delay(100);
		// Print status
		char str[100];
		sprintf(str, "[LOGGER::SetToCmdMode] OpenLog Set to Cmd Mode: mode = %c", mode);
		DebugFlow(str);
	}
	// Log/print error
	else {
		pass = false;
		char str[100];
		sprintf(str, "!!ERROR!! [LOGGER::SetToCmdMode] ABORTED: mode=%c", mode);
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

char LOGGER::SendCommand(char msg[], uint32_t timeout, bool do_conf)
{
	// Send
	Serial3.write(msg);
	t_sent = millis();
	char reply = '\0';

	// Print sent
	if (db.print_a2o)
	{
		PrintLOGGER("SENT[=======================", true);
		for (int i = 0; i < strlen(msg) + 1; i++)
			PrintLOGGER(PrintSpecialChars(msg[i]));
		PrintLOGGER("\n=======================]SENT\n\n");
	}

	// Get confirmation
	if (do_conf) {
		reply = GetReply(timeout);
		// Check for error
		if (reply == '!') {
			// Remove '\r'
			msg[strlen(msg) - 1] = msg[strlen(msg) - 1] == '\r' ? '\0' : msg[strlen(msg) - 1];
			char str[100];
			sprintf(str, "WARNING [LOGGER::SendCommand] Command %s Failed", msg);
			DebugError(str);
		}
	}
	else reply = mode;

	// Return mode
	return reply;
}

char LOGGER::GetReply(uint32_t timeout)
{
	// Local vars
	char str[200] = { 0 };
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
			if (db.print_o2a) {
				SerialUSB.print(c);
			}

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
		sprintf(str, "mode=\'%c\' reply=\'%c\' dt=%dms bytes=%d\n\n", mode, cmd_reply, t_rcvd - t_sent, arr_ind + 1);
		PrintLOGGER(str);
	}

	// Log/print error
	if (!pass) {
		sprintf(str, "WARNING [LOGGER::GetReply] Timedout: dt=%d bytes=%d", timeout, arr_ind + 1);
		DebugError(str);
	}

	// Return cmd 
	return cmd_reply;
}

bool LOGGER::SetToLogMode(char log_file[])
{
	// Local vars
	char str[100] = { 0 };
	bool pass = false;

	// Check if already in log mode
	if (mode == '<')
		return true;

	// Send append file command
	sprintf(str, "append %s\r", log_file);
	if (SendCommand(str) != '!')
		pass = true;

	// Store new log file
	if (pass) {
		strcpy(logFile, log_file);
		delay(100);
	}

	// Log/print error
	else {
		sprintf(str, "!!ERROR!! [LOGGER::SetToLogMode] ABORTED: mode=%c", mode);
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
	int log_bytes = 0;
	uint32_t t_m = 0;

	// Bail if not in write mode
	if (mode != '<')
		return false;

	// Get sync correction
	t_m = t - t_sync;

	// itterate count
	cntLogStored++;

	//// Remove leading white spaces
	//char str[300];
	//int ii = 0;
	//while (msg[ii] == ' ')
	//	ii++;
	//int ws_lng = ii;
	//for (int i = 0; i < strlen(msg) - ws_lng; i++) {
	//	str[i] = msg[ii];
	//	ii++;
	//}
	//str[strlen(msg) - ws_lng] = '\0';

	// Put it all together
	sprintf(msg_out, "[%d],%lu,%s\r\n", cntLogStored, t_m, msg);

	// Add min delay
	if (millis() - t_write < 15) {
		int del = 15 - (millis() - t_write);
		if (del > 0)
			delay(del);
	}

	// Send it
	Serial3.write(msg_out);
	t_write = millis();

	// Add to byte total
	log_bytes = strlen(msg_out);
	bytesStored += log_bytes;

	// Print stored log
	if (db.print_logging) {
		msg_out[strlen(msg_out) - 1] = '\0';
		sprintf(msg, "   [LOG] r2c: num=%d bytes=%d/%d msg=\"%s\"", cntLogStored, log_bytes, bytesStored, msg_out);
		StoreDBPrintStr(msg, millis());
	}

	// Return success
	return true;
}

void LOGGER::SendLogEntry()
{
	// Local vars
	const int timeout = 1200000;
	const int read_max = 5000;
	static int cnt_err_read = 0;
	static int cnt_err_set_mode = 0;
	uint32_t t_start = millis(); // (ms)
	uint32_t t_last_read = micros(); // (us)
	uint32_t t_last_write = micros(); // (us)
	int read_start = 0;
	int read_ind = 0;
	char str[200] = { 0 };
	bool head_passed = false;
	bool send_done = false;
	bool do_abort = false;
	char err_str[100] = { 0 };
	char c_arr[3] = { '\0', '\0', '\0' };
	float dt_read_mu = 0; // (us)
	float dt_write_mu = 0; // (us)
	int dt_read[4] = { 0, 0, 10000, 0 }; // {sum, cnt, min, max}
	int dt_write[4] = { 0, 0, 10000, 0 }; // {sum, cnt, min, max}

	// Make sure in command mode
	if (!SetToCmdMode()) {
		cnt_err_set_mode++;
		// Leave function
		if (cnt_err_set_mode < 3)
			return;
		// Abort after 3 failures
		else {
			do_abort = true;
			sprintf(err_str, "Log \"$$$\" Failed");
		}
	}

	// Request/Re-request log data
	while (millis() < (t_start + timeout)) {

		// Print current send ind
		sprintf(str, "[LOGGER::SendLogEntry] RUNNING: Send Logs: sent=%dB stored=%dB", bytesSent, bytesStored);
		DebugFlow(str);
		while (PrintDebug());

		// Dump anything left
		uint32_t t_out = millis() + 10;
		while (millis() < t_out || Serial3.available() > 0)
			if (Serial3.available() > 0)
				Serial3.read();

		// Send/resend command to read
		if (!send_done &&
			!do_abort) {
			sprintf(str, "read %s %d %d\r", logFile, read_start, read_max);
			SendCommand(str, 5000, false);
		}
		// Finished
		else
			break;

		// Reset vars
		head_passed = false;
		read_ind = 0;
		c_arr[0] = 0; c_arr[1] = 0; c_arr[2] = 0;
		if (bytesSent == 0) {
			t_last_read = micros();
			t_last_write = micros();
		}

		// Read byte at a time
		while (millis() < (t_start + timeout)) {

			// Check for new data
			if (Serial3.available() == 0)
			{
				// Wait for new data
				if (micros() - t_last_read < 500000 ||
					bytesSent == 0)
					continue;
				// Abort if too much time ellapsed sinse last read
				else
				{
					do_abort = true;
					sprintf(err_str, "Read Timedout");
					break;
				}
			}

			// Track dt read
			if (bytesSent > 0) {
				int dt = micros() - t_last_read;
				dt_read[0] += dt;
				dt_read[1]++;
				dt_read[2] = dt < dt_read[2] ? dt : dt_read[2];
				dt_read[3] = dt > dt_read[3] ? dt : dt_read[3];
			}

			// Get next bytes
			c_arr[0] = c_arr[1];
			c_arr[1] = c_arr[2];
			c_arr[2] = Serial3.read();
			t_last_read = micros();
			read_ind++;

			// Check for leading "\r\n"
			if (!head_passed) {
				if (c_arr[2] != '\r' &&
					c_arr[2] != '\n')
					// Header found
					head_passed = true;
				// Continue till header found
				else continue;
			}

			// Check for error
			if (c_arr[2] == '!') {
				if ((c_arr[0] == '\r' && c_arr[1] == '\n') ||
					read_ind < 3)
				{
					// Retry only once
					cnt_err_read++;
					if (cnt_err_read > 1) {
						do_abort = true;
						sprintf(err_str, "Log \"read\" Failed");
					}
					else {
						sprintf(str, "WARNING [LOGGER::GetReply] Log \"read\" Failure %d", cnt_err_read);
						DebugError(str);
						while (PrintDebug());
					}

					// Break
					break;
				}
			}

			// Break for first \n before range passed
			if (
				c_arr[0] == '\r' &&
				c_arr[1] == '\n' &&
				read_ind > read_max - 200
				)
			{
				// Set next read start
				read_start = bytesSent;

				// Dump remaining
				while (Serial3.read() != '>');
				break;
			}

			// Track dt write
			int dt = micros() - t_last_write;
			// Add min delay
			if (dt < 150) {
				int del = 150 - dt;
				if (del > 0) {
					delayMicroseconds(del);
					dt = micros() - t_last_write;
				}
			}
			if (bytesSent > 0) {
				dt_write[0] += dt;
				dt_write[1]++;
				dt_write[2] = dt < dt_write[2] ? dt : dt_write[2];
				dt_write[3] = dt > dt_write[3] ? dt : dt_write[3];
			}

			// Send new byte
			/*
			SerialUSB.write(PrintSpecialChars(c_arr[2]));
			*/
			Serial1.write(c_arr[2]);
			t_last_write = micros();
			bytesSent++;

			// Check if all bytes sent
			if (bytesSent == bytesStored) {
				send_done = true;
				break;
			}

		}
	}
	// Check for timeout
	if (millis() >= t_start + timeout) {
		do_abort = true;
		sprintf(err_str, "Run Timedout");
	}

	// Compute mean dt read/write
	/*
	{dt_read_mu}{dt_read[2]}{dt_read[3]}{dt_write_mu}{dt_write[2]}{dt_write[3]}
	*/
	dt_read_mu = (float)dt_write[0] / (float)dt_write[1];
	dt_write_mu = (float)dt_read[0] / (float)dt_read[1];

	// End reached send ">>>"
	byte msg_end[3] = { '>','>','>' };

	Serial1.write(msg_end, 3);
	delay(100);

	// Send confirm done in case CS missed footer
	QueuePacket('c', 'D', 255, donePack);

	// Print status
	float t_s = (float)(millis() - t_start) / 1000.0f;
	if (!do_abort) {
		sprintf(str, "[LOGGER::SendLogEntry] FINISHED: Send Logs: dt=%0.2fs sent=%dB stored=%dB", t_s, bytesSent, bytesStored);
		DebugFlow(str);
	}
	else {
		sprintf(str, "!!ERROR!! [LOGGER::SendLogEntry] ABORTED: Send Logs: %s: dt=%0.2fs sent=%dB stored=%dB", err_str, t_s, bytesSent, bytesStored);
		DebugError(str);
	}

	// Reset vars
	bytesSent = 0;
	cnt_err_read = 0;
	cnt_err_set_mode = 0;

	// Reset flag
	fc.doLogSend = false;
}

void LOGGER::TestLoad(int n_entry, char log_file[])
{
	// Local vars
	char str[200] = { 0 };
	bool pass = false;

	// Load existing log file
	if (log_file != '\0') {
		sprintf(str, "[LOGGER::TestLoad] RUNNING: Load Log: file_name=%s", log_file);
		DebugFlow(str);
		if (SetToCmdMode())
		{
			// Get bytes
			if (SendCommand("ls\r") != '!') {
				bytesStored = GetFileSize(log_file);
				pass = true;
			}
			// Start writing to file
			if (pass)
				if (!SetToLogMode(log_file))
					pass = false;
		}
		if (pass) {
			sprintf(str, "[LOGGER::TestLoad] FINISHED: Load Log: file_name=%s size=%dB", logFile, bytesStored);
			DebugFlow(str);
		}
		else
			DebugError("!!ERROR!! [LOGGER::TestLoad] ABORTED: Load Log File");
	}

	// Write n_entry entries to log
	else if (n_entry != 0) {
		sprintf(str, "[LOGGER::TestLoad] RUNNING: Write %d Logs", n_entry);
		DebugFlow(str);
		while (PrintDebug());
		randomSeed(analogRead(A0));
		for (int i = 0; i < n_entry - 1; i++)
		{
			// Make random sized strings
			//char num_str[15] = { 0 };
			//char msg[200] = { 0 };
			//for (int i = 0; i < 18; i++)
			//{
			//	int num = random(999999999);
			//	sprintf(num_str, "%d", num);
			//	num_str[random(10) + 1] = '\0';
			//	strcat(msg, num_str);
			//}
			//StoreLogEntry(msg, millis());
			//msg[0] = '\0';

			// Make uniformed size strings
			char msg[200] = "AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFFGGGGGGGGGGHHHHHHHHHHIIIIIIIIIIJJJJJJJJJJKKKKKKKKKKLLLLLLLLLL";
			//char msg[200] = "AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEE";

			StoreLogEntry(msg, millis());
		}
		sprintf(str, "[LOGGER::TestLoad] FINISHED: Write %d Logs", n_entry);
		DebugFlow(str);
	}
}

int LOGGER::GetFileSize(char log_file[])
{
	// Local vars
	int ind = -1;
	int fi_size = 0;
	char num_str[20] = { 0 };

	// Find file index in array
	for (int i = strlen(rcvdArr) - strlen(log_file); i >= 0; i--)
	{
		int ii = 0;
		for (ii = strlen(log_file) - 1; ii >= 0; ii--) {
			millis();
			if (log_file[ii] != rcvdArr[i + ii])
				break;
			else if (ii == 0) {
				ind = ii + i;
				break;
			}
		}
		if (ind != -1)
			break;
	}

	// Get file size
	if (ind != -1) {
		int i = ind + strlen(log_file) + 3;
		int ii = 0;
		while (i < strlen(rcvdArr) && rcvdArr[i] != '\r')
		{
			num_str[ii] = rcvdArr[i];
			i++;
			ii++;
		}
		num_str[ii] = '\0';
		fi_size = atoi(num_str);
	}

	return fi_size;
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


#pragma region ========== FUNCTION DEFINITIONS =========

#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
void GetSerial()
{
	// Local vars
	char str[200] = { 0 };
	int buff_tx = 0;
	int buff_rx = 0;
	byte buff = 0;
	char head = ' ';
	char id = ' ';
	int id_ind = 0;
	uint16_t pack = 0;
	char foot = ' ';
	bool do_conf;
	bool is_cs_msg = false;

	// Reset vars
	c2r.isNew = false;
	c2r.idNew = ' ';
	c2r.dat[0] = 0;
	c2r.dat[1] = 0;
	c2r.dat[2] = 0;
	a2r.dat[0] = 0;

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

	// Get total data left in buffers
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	buff_rx = Serial1.available();

	// Check for matching footer
	if (foot == a2r.foot || foot == c2r.foot)
	{
		// Update recive time
		t_rcvd = millis();

		// Store revieved pack details
		if (id != 'P')
			DebugRcvd(is_cs_msg ? 'c' : 'a', id, pack);

		// Send confirmation
		if (do_conf)
			QueuePacket(is_cs_msg ? 'c' : 'a', id, 255, pack);

		// Check for streaming started
		if (!fc.isStreaming)
			fc.isStreaming = true;
	}
	// Packet lost
	else {
		cnt_droppedPacks++;
		sprintf(str, "!!ERROR!! [GetSerial] Missing Footer: dropped_tot=%d head=%c id=%c pack=%d foot=%c tx=%d rx=%d", cnt_droppedPacks, head, id, pack, foot, buff_tx, buff_rx);
		DebugError(str);
	}

	// Process ard packet
	if (!is_cs_msg && foot == a2r.foot) {

		// Get id ind
		id_ind = CharInd(id, r2a.idList, r2a.idLng);

		// Reset flags
		r2a.doRcvCheck[id_ind] = false;

	}

	// Process cs packet
	if (is_cs_msg && foot == c2r.foot)
	{
		// Store id
		c2r.idNew = id;

		// Get id ind
		id_ind = CharInd(id, c2r.idList, c2r.idLng);

		// Reset flags
		r2c.doRcvCheck[id_ind] = false;

		// Check for dropped packets
		int pack_diff = (pack - c2r.packLast);

		if (pack_diff > 0)
		{
			// Save packet and set to process
			c2r.packLast = pack;

			// Check for dropped packet
			int dropped_packs = pack_diff - 1;

			// print dropped packs
			if (dropped_packs > 0)
			{
				cnt_droppedPacks += dropped_packs;
				sprintf(str, "!!ERROR!! [GetSerial] Dropped C2R Packets: dropped_tot=%d head=%c id=%c pack=%d foot=%c tx=%d rx=%d", cnt_droppedPacks, head, id, pack, foot, buff_tx, buff_rx);
				DebugError(str);
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

			// Print resent packet
			sprintf(str, "!!ERROR!! [GetSerial] Duplicate C2R Packet: id=%c pack=%d duplicates=%d tx=%d rx=%d", id, pack, c2r.cntRepeat[id_ind], buff_tx, buff_rx);
			DebugError(str);

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
		char str[200];

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
void QueuePacket(char targ, char id, byte d1, uint16_t pack, bool do_conf)
{
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2]dat, [3:4]pack, [5]do_conf, [6]footer, [7]targ
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

	// Store reciever id in last col
	sendQueue[queue_ind][7] = targ;

	// Set to send
	fc.doPackSend = true;

}

// SEND SERIAL PACKET DATA
void SendPacket()
{
	/*
	STORE DATA TO SEND
	FORMAT: [0]head, [1]id, [2]dat, [3:4]pack, [5]do_conf, [6]footer, [7]targ
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

	// Move next in queue to temp msg array
	for (int j = 0; j < msg_lng; j++) {
		msg[j] = sendQueue[sendQueueRows - 1][j];
	}

	// Copy target from last row
	targ = sendQueue[sendQueueRows - 1][sendQueueCols - 1];

	// pull out msg id
	id = msg[1];
	// dat
	dat = msg[2];
	// pack
	U.f = 0.0f;
	U.b[0] = msg[3];
	U.b[1] = msg[4];
	pack = U.i16[0];
	// do_conf 
	do_conf = msg[5] == 1 ? true : false;

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
				r2a.t_sentList[id_ind] = t_sent;
				r2a.doRcvCheck[id_ind] = true;
			}
			else if (targ == 'c')
			{
				id_ind = CharInd(id, r2c.idList, r2c.idLng);
				r2c.t_sentList[id_ind] = t_sent;
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
			fc.doPackSend = false;
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
	int dt_sent = 0;

	// Get pointer to struct vars
	int id_lng = targ == 'c' ? r2c.idLng : r2a.idLng;
	const char *id_list = targ == 'c' ? r2c.idList : r2a.idList;
	uint16_t *pack_list = targ == 'c' ? r2c.packList : r2a.packList;
	byte *dat_list = targ == 'c' ? r2c.datList : r2a.datList;
	uint32_t *t_sent_list = targ == 'c' ? r2c.t_sentList : r2a.t_sentList;
	bool *do_rcv_check = targ == 'c' ? r2c.doRcvCheck : r2a.doRcvCheck;
	int *resend_cnt = targ == 'c' ? r2c.resendCnt : r2a.resendCnt;

	// Loop and check ard flags
	for (int i = 0; i < id_lng; i++)
	{
		// Get dt sent
		dt_sent = millis() - t_sent_list[i];

		// Check if should resend
		if (
			do_rcv_check[i] &&
			dt_sent > dt_resend
			)
		{
			if (resend_cnt[i] < resendMax) {

				// Get total data left in buffers
				int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
				int buff_rx = Serial1.available();

				// Resend data
				QueuePacket(targ, id_list[i], dat_list[i], pack_list[i], true);

				// Update count
				resend_cnt[i]++;

				// Print resent packet
				char str[200];
				sprintf(str, "!!ERROR!! [CheckResend] Resending %s Packet: id=%c pack=%d dt=%dms resends=%d tx=%d rx=%d",
					targ == 'c' ? "r2c" : "r2a", id_list[i], pack_list[i], dt_sent, resend_cnt[i], buff_tx, buff_rx);
				DebugError(str);

				// Set flags
				do_pack_resend = true;
				do_rcv_check[i] = false;
			}
			// Com likely down
			else {
				do_rcv_check[i] = false;
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
	char str[200] = { 0 };
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
			sprintf(str, "!!ERROR!! [AD_CheckOC] Overcurrent Detected: R_OCD=%d F_OCD=%d", ocd_r, ocd_f);
			DebugError(str);
			cnt_errors++;
			//AD_Reset();
		}

		// Set next check
		t_checkAD = millis() + dt_checkAD;
	}
	// Disable error checking after 5 hits
	else if (cnt_errors >= 5) {
		sprintf(str, "!!ERROR!! [AD_CheckOC] Disabled AD Check After %d Errors", cnt_errors);
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

	// Reset speed
	runSpeedNow = 0;

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
	sprintf(str, "[HardStop] Hard Stop [%s]", called_from);
	DebugFlow(str);
}

// IR TRIGGERED HARD STOP
void IRprox_Halt()
{
	if (!(Bull.mode == "Active" && Bull.state == "On") &&
		!fc.isManualSes)
	{
		HardStop("IRprox_Halt");
	}
}

// RUN AUTODRIVER
bool RunMotor(char dir, double new_speed, String agent)
{
	// Local vars
	double speed_steps = new_speed*cm2stp;

	// Check that caller has control
	if (agent == fc.motorControl ||
		agent == "Override")
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

		// Update run speed and dir
		runSpeedNow = new_speed;
		runDirNow = dir;

		return true;
	}
	else return false;
}

// RUN MOTOR MANUALLY
bool ManualRun(char dir)
{
	// Local vars
	char speed_str[100] = { 0 };
	char volt_str[100] = { 0 };
	int inc_speed = 10; // (cm/sec)
	double new_speed;

	// Incriment speed
	if (dir != runDirNow ||
		runSpeedNow == 0)
		// Set to start speed
		new_speed = 5;
	else
		new_speed = runSpeedNow <= maxSpeed - inc_speed ?
		runSpeedNow + inc_speed : runSpeedNow;

	// Run motor
	RunMotor(dir, new_speed, "Override");

	// Print voltage and speed to LCD
	sprintf(volt_str, "VCC=%0.2fV", voltNow);
	sprintf(speed_str, "VEL=%s%dcm/s", runDirNow == 'f' ? "->" : "<-", (int)runSpeedNow);
	PrintLCD(volt_str, speed_str);
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
	t_rewBlockMove = millis() + dt;

	// Remove all motor controll
	SetMotorControl("None", "BlockMotorTill");

	// Print blocking finished
	DebugMotorBocking("Blocking Motor for ", dt, called_from);
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
		if (millis() > t_rewBlockMove || is_passed_feeder)
		{
			// Print blocking finished
			DebugMotorBocking("Finished Blocking Motor at ", millis(), "CheckBlockTimElapsed");

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
		char str[200] = { 0 };
		int n_laps = 0;
		double cm_diff = 0;
		double cm_dist = 0;

		// Log/Print
		DebugFlow("[InitializeTracking] RUNNING: Initialize Rat Tracking");

		// Check that rat pos > robot pos
		n_laps = RatVT.posNow > RobVT.posNow ? 0 : 1;
		// set n_laps for rat vt data
		RatVT.SetPos(RatVT.posNow, n_laps);
		RatPixy.SetPos(RatPixy.posNow, n_laps);
		if (n_laps > 0)
			DebugFlow("WARINING [InitializeTracking] Set Rat Ahead of Robot");

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
			DebugFlow("WARINING [InitializeTracking] Had to Reset Position Data");
			return;
		}

		// Log/print rat and robot starting pos
		sprintf(str, "[InitializeTracking] Starting Positions: rat_vt=%0.2f rat_pixy=%0.2f rob_vt=%0.2f", RatVT.posNow, RatPixy.posNow, RobVT.posNow);
		DebugFlow(str);

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
			DebugFlow("[InitializeTracking] PID STARTED");
		}

		// Initialize bulldoze
		if (fc.doBulldoze)
		{
			// Run from initial blocked mode
			Bull.TurnOn("InitializeTracking");
			DebugFlow("[InitializeTracking] BULLDOZE INITIALIZED");
		}

		// Blink lcd display
		RatInBlink();

		// Log/Print
		DebugFlow("[InitializeTracking] FINISHED: Initialize Rat Tracking");

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
	double px_rel = 0;
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
		RatPixy.UpdatePos(px_abs, t_px_ts);

	}

}

// UPDATE EKF
void UpdateEKF()
{
	// Local vars
	static bool is_nans_last = false;
	double rat_pos = 0;
	double rob_pos = 0;
	double rat_vel = 0;
	double rob_vel = 0;

	// Check for new data w or w/o rat tracking
	if ((RatVT.isDataNew && RatPixy.isDataNew && RobVT.isDataNew) ||
		(RobVT.isDataNew && !fc.isRatIn))
	{

		// Check EKF progress
		Pid.CheckEKF(millis());

		// Update pid next loop time
		Pid.SetLoopTime(millis());

		// Hold rat pos at 0
		if (!fc.isRatIn) {
			RatVT.SetPos(0, 0);
			RatPixy.SetPos(0, 0);
		}
		// Set flag for reward
		else if (fc.isTrackingEnabled) {
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
		rat_pos = Fuser.getX(0);
		rob_pos = Fuser.getX(1);
		rat_vel = Fuser.getX(2);
		rob_vel = Fuser.getX(3);

		// Copy over values
		ekf.RatPos = !isnan(rat_pos) ? rat_pos : ekf.RatPos;
		ekf.RobPos = !isnan(rob_pos) ? rob_pos : ekf.RobPos;
		ekf.RatVel = !isnan(rat_vel) ? rat_vel : ekf.RatVel;
		ekf.RobVel = !isnan(rob_vel) ? rob_vel : ekf.RobVel;

		// Check for nan values
		if (isnan(rat_pos) || isnan(rob_pos) || isnan(rat_vel) || isnan(rob_vel)) {

			// Do not print consecutively
			if (is_nans_last)
				return;

			char str[200] = { 0 };
			sprintf(str, "!!ERROR!! [UpdateEKF] \"nan\" EKF Output: ratVT=%0.2f|%0.2f ratPixy=%0.2f|%0.2f robVT=%0.2f|%0.2f",
				RatVT.posNow, RatVT.velNow, RatPixy.posNow, RatPixy.velNow, RobVT.posNow, RatVT.velNow);
			DebugError(str);

			// Set flag
			is_nans_last = true;
		}
		else is_nans_last = false;

	}
}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// CHECK FOR BUTTON INPUT
bool GetButtonInput()
{
	// Notes
	/*
	BUTTON 1:
		Short Hold: Trigger reward
		Long Hold:	Move robot forward
	BUTTON 2:
		Short Hold: open/close Reward solonoid
		Long Hold:	Move robot backward
	BUTTON 3:
		Short Hold: open/close EtOH solonoid
		Long Hold:	Turn LCD LED on/off
	BUTTON 4: Shorted to reset
	*/

	// Local vars
	bool is_new_input = false;
	static bool is_pressed[3] = { false, false, false };
	static bool is_running[3] = { false, false, false };
	int do_flag_fun[3][2] = { { false,false },{ false,false },{ false,false } };
	static uint32_t t_debounce[3] = { millis() + 1000, millis() + 1000, millis() + 1000 };
	static uint32_t t_hold_min[3] = { 0, 0, 0 };
	static uint32_t t_long_hold[3] = { 0, 0, 0 };
	int dt_debounce[3] = { 100, 100, 100 };
	int dt_hold_min = 50;
	int dt_long_hold = 500;
	int btn_ind = 0;

	// Loop through and check each button
	for (int i = 0; i < 3; i++)
	{
		btn_ind = i;

		// Detect press
		if (
			digitalRead(pin.Btn[btn_ind]) == LOW &&
			!is_pressed[btn_ind])
		{
			// Exit if < debounce time has not passed
			if (t_debounce[btn_ind] > millis()) return false;

			// Get long hold time
			t_long_hold[btn_ind] = millis() + dt_long_hold - dt_hold_min;

			// Set flag
			is_pressed[btn_ind] = true;

			// Log/print
			char str[100];
			sprintf(str, "[GetButtonInput] Pressed button %d", i);
			DebugFlow(str);
		}

		// Check released
		else if (
			is_pressed[btn_ind] &&
			!is_running[btn_ind])
		{

			// Bail if not dt hold min
			t_hold_min[btn_ind] = t_hold_min[btn_ind] == 0 ? millis() + dt_hold_min : t_hold_min[btn_ind];
			if (millis() < t_hold_min[btn_ind])
				return false;

			// Check for short hold
			bool is_short_hold =
				digitalRead(pin.Btn[btn_ind]) == HIGH &&
				millis() < t_long_hold[btn_ind];

			// Check for long hold
			bool is_long_hold = millis() > t_long_hold[btn_ind];

			// Set flag for either condition
			if (is_short_hold || is_long_hold) {

				// Run short hold function
				if (is_short_hold)
					do_flag_fun[btn_ind][0] = true;

				// Run long hold function
				if (is_long_hold)
					do_flag_fun[btn_ind][1] = true;

				// Make tracker LED brighter
				analogWrite(pin.TrackLED, 255);

				// Set running flag
				is_running[btn_ind] = true;

				// Flag input rcvd
				is_new_input = true;

				// Log/print
				char str[100];
				sprintf(str, "[GetButtonInput] Triggered button %d", i);
				DebugFlow(str);
			}
		}

		// Check if needs to be reset
		else if (digitalRead(pin.Btn[btn_ind]) == HIGH &&
			is_pressed[btn_ind])
		{
			if (is_running[btn_ind] ||
				millis() > t_long_hold[btn_ind])
			{
				// Reset flags etc
				t_debounce[btn_ind] = millis() + dt_debounce[btn_ind];
				analogWrite(pin.TrackLED, trackLEDduty);
				is_running[btn_ind] = false;
				t_hold_min[btn_ind] = 0;
				t_long_hold[btn_ind] = 0;
				is_pressed[btn_ind] = false;

				// Log/print
				char str[100];
				sprintf(str, "[GetButtonInput] Reset button %d", i);
				DebugFlow(str);
			}
		}

	}

	// Check if more than one button triggered
	int btn_trg[2] = { 0 };
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 2; j++) {
			// Another button already triggered
			if (btn_trg[0] > 0 && do_flag_fun[i][j]) {
				char str[100];
				sprintf(str, "!!ERROR!![GetButtonInput] Multiple Buttons Triggered: btn %d %s and btn %d %s",
					btn_trg[0], btn_trg[1] == 0 ? "short hold" : "long hold", i, j == 0 ? "short hold" : "long hold");
				DebugError(str);
				return false;
			}
			btn_trg[0] = do_flag_fun[i][j] ? i : btn_trg[0];
			btn_trg[1] = do_flag_fun[i][j] ? j : btn_trg[1];
		}
	}

	// Set button 1 function flag
	if (do_flag_fun[0][0]) {
		fc.doBtnRew = true;
		DebugFlow("[GetButtonInput] Button 1 \"fc.doBtnRew\" Triggered");
	}
	else if (do_flag_fun[0][1]) {
		fc.doMoveRobFwd = true;
		DebugFlow("[GetButtonInput] Button 1 \"fc.doMoveRobFwd\" Triggered");
	}
	// Set button 2 function flag
	else if (do_flag_fun[1][0]) {
		fc.doRewSolStateChange = true;
		DebugFlow("[GetButtonInput] Button 2 \"fc.doRewSolStateChange\" Triggered");
	}
	else if (do_flag_fun[1][1]) {
		fc.doMoveRobRev = true;
		DebugFlow("[GetButtonInput] Button 2 \"fc.doMoveRobRev\" Triggered");
	}
	// Set button 3 function flag
	else if (do_flag_fun[2][0]) {
		fc.doEtOHSolStateChange = true;
		DebugFlow("[GetButtonInput] Button 3 \"fc.doEtOHSolStateChange\" Triggered");
	}
	else if (do_flag_fun[2][1]) {
		fc.doChangeLCDstate = true;
		DebugFlow("[GetButtonInput] Button 3 \"fc.doChangeLCDstate\" Triggered");
	}

	// Return flag
	return is_new_input;
}

// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid()
{
	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_Rew);
	char etoh_str[100] = { 0 };
	char rew_str[100] = { 0 };

	// Change state
	is_sol_open = !is_sol_open;

	// Store open time
	if (is_sol_open)
		t_solOpen = millis();
	else
		t_solClose = millis();

	// Open/close solenoid
	digitalWrite(pin.Rel_Rew, is_sol_open);

	// Print etoh and rew sol state to LCD
	fc.doBlockLCDlog = false;
	sprintf(rew_str, "Food   %s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN  " : "CLOSED");
	sprintf(etoh_str, "EtOH   %s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN  " : "CLOSED");
	PrintLCD(rew_str, etoh_str, 's');
	if (is_sol_open) fc.doBlockLCDlog = true;
}

// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid()
{
	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_EtOH);
	char etoh_str[100] = { 0 };
	char rew_str[100] = { 0 };

	// Change state
	is_sol_open = !is_sol_open;

	// Open/close solenoid
	digitalWrite(pin.Rel_EtOH, is_sol_open);

	// Store time and make sure periodic drip does not run
	if (is_sol_open) {
		t_solOpen = millis();
		fc.doEtOHRun = false;
	}
	else {
		t_solClose = millis();
		fc.doEtOHRun = true;
	}

	// Print etoh and rew sol state to LCD
	fc.doBlockLCDlog = false;
	sprintf(rew_str, "Food   %s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN  " : "CLOSED");
	sprintf(etoh_str, "EtOH   %s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN  " : "CLOSED");
	PrintLCD(rew_str, etoh_str, 's');
	if (is_sol_open) fc.doBlockLCDlog = true;
}

// CHECK FOR ETOH UPDATE
void CheckEtOH()
{
	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_EtOH);
	static float etoh_dist_start = 0; // (cm)
	static float etoh_dist_diff = 0; // (cm)
	bool do_open = false;
	bool do_close = false;

	// Bail if etoh should not be run
	if (!fc.doEtOHRun)
		return;

	// Get distance traveled
	etoh_dist_diff = abs(ekf.RobPos - etoh_dist_start);

	if (!is_sol_open) {

		// Open if dt run has ellapsed and robot has moved
		do_open = millis() > (t_solOpen + dt_delEtOH);// &&
			//etoh_dist_diff > 1;

		// Open if robot has covered half the trak
		do_open = do_open ||
			etoh_dist_diff > (140 * PI) / 2;

		// Open if vel has not been updated recently
		do_open =
			do_open || (millis() > t_voltUpdate + 60000 &&
				CheckAD_Status(adR_stat, "MOT_STATUS") == 0 &&
				CheckAD_Status(adF_stat, "MOT_STATUS") == 0);
	}

	// Close if open and dt close has ellapsed
	else
		do_close = millis() > (t_solOpen + dt_durEtOH);

	// Check if sol should be opened
	if (do_open)
	{
		// Open solenoid
		digitalWrite(pin.Rel_EtOH, HIGH);

		// Reset vars
		t_solOpen = millis();
		etoh_dist_start = ekf.RobPos;
	}

	// Check if sol should be closed
	else if (do_close)
	{
		// Close solenoid
		digitalWrite(pin.Rel_EtOH, LOW);

		// Print to debug
		if (fc.isSesStarted) {
			char str[100];
			sprintf(str, "[CheckEtOH] Ran EtOH: dt=%d", millis() - t_solOpen);
			DebugFlow(str);
		}
	}
}

// CHECK BATTERY VOLTAGE
void GetBattVolt()
{
	// Local vars
	static float volt_arr[10] = { 0 };
	static bool do_volt_update = false;
	static float volt_avg = 0;
	static int n_samples = 0;
	float bit_in = 0;
	float volt_in = 0;
	float volt_sum = 0;
	byte byte_out = 0;
	byte do_shutdown = false;


	// Only run if relay open and motor stopped
	if (
		digitalRead(pin.Rel_EtOH) == HIGH &&
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
		voltNow = volt_avg;

		// Store time
		t_voltUpdate = millis();

		// Add to array
		for (int i = 0; i < 10 - 1; i++)
			volt_arr[i] = volt_arr[i + 1];
		volt_arr[9] = voltNow;

		// Convert float to byte
		byte_out = byte(round(volt_avg * 10));

		// Add to queue if streaming established
		if (fc.isStreaming)
			QueuePacket('c', 'J', byte_out, (uint16_t)1);

		// Log/print voltage
		char str[100] = { 0 };
		sprintf(str, "[GetBattVolt] VCC=%0.2fV", volt_avg);
		if (fc.isSesStarted)
			DebugFlow(str);

		// Print voltage and speed to LCD
		if (millis() > t_solClose + 100) {
			char volt_str[100];
			char speed_str[100];
			sprintf(volt_str, "VCC=%0.2fV", voltNow);
			sprintf(speed_str, "VEL=%s%dcm/s", runDirNow == 'f' ? "->" : "<-", (int)runSpeedNow);
			PrintLCD(volt_str, speed_str);
		}

		// Check if voltage critically low
		do_shutdown = true;
		for (int i = 0; i < 10 - 1; i++) {
			do_shutdown = do_shutdown && volt_arr[i] < voltCutoff && volt_arr[i] > 0 ?
				true : false;
		}

		// Perform shutdown
		if (do_shutdown) {
			// Run error hold then shutdown
			char str[100] = { 0 };
			sprintf(str, "BATT LOW %0.2fV", voltNow);
			RunErrorHold(str, 60000);
		}

		// Reset flag
		do_volt_update = false;
	}
}

// TURN LCD LIGHT ON/OFF
void ChangeLCDlight()
{
	if (!fc.isLitLCD) {
		analogWrite(pin.Disp_LED, 50);
		fc.isLitLCD = true;
	}
	else {
		analogWrite(pin.Disp_LED, 0);
		fc.isLitLCD = false;
	}
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

// HOLD FOR CRITICAL ERRORS
void RunErrorHold(char msg[], uint32_t t_kill)
{
	// Local vars
	static uint32_t t_shutdown = t_kill > 0 ? millis() + t_kill : 0;
	char str[50] = { 0 };
	int duty[2] = { 255, 0 };
	bool is_on = true;
	int dt = 250;
	float t_s = 0;

	// Turn on LCD LED
	digitalWrite(pin.Disp_LED, 100);

	// Get time seconds
	t_s = (float)(millis() - t_sync) / 1000.0f;

	// Print error
	sprintf(str, "ERROR %0.2fs", t_s);
	PrintLCD(msg, str, 't');

	// Loop indefinaitely
	while (true)
	{
		// Flicker lights
		analogWrite(pin.RewLED_R, duty[(int)is_on]);
		analogWrite(pin.TrackLED, duty[(int)is_on]);
		delay(dt);
		is_on = !is_on;

		// Check if should shutdown
		if (t_shutdown > 0 && millis() > t_shutdown) {
			// Log/print
			sprintf(str, "!!ERROR!! [RunErrorHold] SHUTTING DOWN BECAUSE: %s", msg);
			DebugError(str);
			delay(1000);

			// Set kill switch high
			digitalWrite(pin.KillSwitch, HIGH);

			// Quit if still powered by USB
			QuitSession();
		}

	}
}

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

// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[])
{
	// Local vars
	bool do_print = db.print_motorControl && (db.Console || db.LCD);
	bool do_log = db.log_motorControl && db.Log;

	if (do_print || do_log)
	{
		char str[200] = { 0 };
		sprintf(str, "[DebugMotorControl] Change %s: set_in=%s set_out=%s [%s]", pass ? "Succeeded" : "Failed", set_to, fc.motorControl.c_str(), called_from);

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
		sprintf(str, "[DebugMotorBocking] %s %lu ms [%s]", msg, t, called_from);

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
			sprintf(str, "   [RCVD] %c2r: id=%c dat1=%0.2f dat2=%0.2f dat3=%0.2f pack=%d", from, id, c2r.dat[0], c2r.dat[1], c2r.dat[2], pack);
		}
		else
			sprintf(str, "   [RCVD] %c2r: id=%c dat1=%d pack=%d", from, id, a2r.dat[0], pack);

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
		sprintf(str, "   [SENT] r2%c: id=%c dat=%d pack=%d do_conf=%s", targ, id, d1, pack, do_conf ? "true" : "false");

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
	char msg_in[200] = { 0 };
	char msg_store[200] = { 0 };
	char str_tim[200] = { 0 };
	uint32_t t_m = 0;
	float t_s = 0;

	// Copy message
	strcpy(msg_in, msg);

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
		char err_str[100] = "!!ERROR!! [StoreDBPrintStr] Print Queue Overflowed";
		for (int c = 0; c < strlen(err_str) + 1; c++)
			msg_in[c] = err_str[c];
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
	sprintf(msg_store, "%s%s%s\n", str_tim, spc, msg_in);

	// Store current message
	for (int c = 0; c < printQueueCols; c++) {
		printQueue[printQueueRows - 1][c] = msg_store[c];
		if (printQueue[printQueueRows - 1][c] == '\0')
			break;
	}

	// Set flag
	fc.doPrint = true;
}

// PRINT DEBUG STRINGS TO CONSOLE/LCD
bool PrintDebug()
{
	// Local vars 
	static int scale_ind = 40 / printQueueRows;
	int lcd_pos = 0;
	char msg_print[200] = { 0 };

	// Bail if nothing new to print
	if (!fc.doPrint)
		return fc.doPrint;

	// Avoid overlap between sent or rcvd events
	if (millis() < t_sent + dt_sendSent ||
		millis() < t_rcvd + dt_sendRcvd)
	{
		return fc.doPrint;
	}

	if ((db.LCD && !fc.doBlockLCDlog) ||
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
		if (db.LCD && !fc.doBlockLCDlog)
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
			fc.doPrint = false;

	}

	// Return print status
	return fc.doPrint;
}

// FOR PRINTING TO LCD
void PrintLCD(char msg_1[], char msg_2[], char f_siz)
{

	// Check if printing blocked
	if (fc.doBlockLCDlog)
		return;

	// Change settings
	if (f_siz == 's')
		LCD.setFont(SmallFont);
	else if (f_siz == 't')
		LCD.setFont(TinyFont);
	LCD.invert(true);

	// Clear
	LCD.clrScr();

	// Make both strings same length
	if (msg_2[0] != '\0' &&
		strlen(msg_1) != strlen(msg_2))
	{
		bool is_m1_shorter = strlen(msg_1) < strlen(msg_2);
		int len_max = !is_m1_shorter ? strlen(msg_1) : strlen(msg_2);
		int len_min = is_m1_shorter ? strlen(msg_1) : strlen(msg_2);
		for (int i = len_min; i < len_max; i++) {
			if (is_m1_shorter)
				msg_1[i] = ' ';
			else
				msg_2[i] = ' ';
		}
		msg_1[len_max] = '\0';
		msg_2[len_max] = '\0';
	}

	// Print
	if (msg_2[0] != '\0')
	{
		LCD.print(msg_1, CENTER, 15);
		LCD.print(msg_2, CENTER, 25);
	}
	else LCD.print(msg_1, CENTER, 20);

	// Update
	LCD.update();
}

// CLEAR LCD
void ClearLCD()
{
	// Clear
	LCD.clrScr();

	// Update
	LCD.update();

	// Stop blocking LCD log
	fc.doBlockLCDlog = false;
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
			sprintf(str, "[CheckLoop] Loop %d: dt=%lums|%lums free_ram=%dKB|%dKB", cnt_loop, dt_loop, dt_loop_last, free_mem, last_mem);
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
		sprintf(str, "WARNING [CharInd] ID \'%c\' Not Found", id);
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
		analogWrite(pin.RewLED_C, duty[(int)is_on]);
		delay(100);
		is_on = !is_on;
		cnt_blnk = !is_on ? cnt_blnk + 1 : cnt_blnk;
	}
	// Reset LEDs
	analogWrite(pin.Disp_LED, 0);
	analogWrite(pin.TrackLED, trackLEDduty);
	analogWrite(pin.RewLED_C, rewLEDmin);
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


#pragma region --------INTERUPTS---------

// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt() {

	// Exit if < 250 ms has not passed
	if (v_t_irProxDebounce > millis()) return;

	// Run stop in main loop
	v_doIRhardStop = true;

	// Update debounce
	v_t_irProxDebounce = millis() + 250;
}

// DETECT IR SYNC EVENT
void Interupt_IR_Detect()
{
	// Exit if < 25 ms has not passed
	if (millis() < v_t_irDetectDebounce) return;

	// Store time
	v_dt_ir = millis() - v_t_irSyncLast;
	v_t_irSyncLast += v_dt_ir;
	v_cnt_ir++;

	// Set flag
	if (t_sync != 0)
		v_doLogIR = true;

	// Update debounce
	v_t_irDetectDebounce = millis() + 50;
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

	// Wait for SerialUSB if debugging
	uint32_t t_check = millis() + 100;
	if (db.Console)
		while (!SerialUSB && millis() < t_check);

	// PRINT SETUP RUNNING
	char str[200];
	sprintf(str, "[setup] RUNNING: Setup: free_ram=%dKB", freeMemory());
	DebugFlow(str);
	while (PrintDebug());

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
	pinMode(pin.KillSwitch, OUTPUT);
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
	digitalWrite(pin.KillSwitch, LOW);
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
	v_t_irProxDebounce = millis(); // (ms)
	v_t_irDetectDebounce = millis(); // (ms)
	v_t_irSyncLast = 0; // (ms)
	t_sync = 0; // (ms)
	v_dt_ir = 0;
	v_cnt_ir = 0;
	v_doIRhardStop = false;
	v_doLogIR = false;
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);

	// SHOW RESTART BLINK
	StatusBlink(1, 0);

	// SETUP OPENLOG
	DebugFlow("[setup] RUNNING: OpenLog Setup");
	while (PrintDebug());

	// Setup OpenLog
	if (Log.Setup())
	{
		// Log/print setup finished
		DebugFlow("[setup] FINISHED: OpenLog Setup");
		while (PrintDebug());
	}
	else {
		// Hold for error
		DebugError("!!ERROR!! [setup] ABORTED: OpenLog Setup");
		while (PrintDebug());
		RunErrorHold("OPENLOG SETUP");
	}

	// Create new log file
	DebugFlow("[setup] RUNNING: Make New Log");
	if (Log.OpenNewLog() == 0) {
		// Hold for error
		DebugError("!!ERROR!! [setup] ABORTED: Setup");
		while (PrintDebug());
		RunErrorHold("OPEN LOG FILE");
	}
	else
	{
		sprintf(str, "[setup] FINISHED: Make New Log: file_name=%s", Log.logFile);
		DebugFlow(str);
		while (PrintDebug());
	}

	// DEFINE EXTERNAL INTERUPTS
	uint32_t t_check_ir = millis() + 500;

	// IR detector
	while (digitalRead(pin.IRdetect) == HIGH && t_check_ir < millis());
	// Do not enable if ir detector pin shorted to vcc
	if (digitalRead(pin.IRdetect) == LOW)
		// IR detector
		attachInterrupt(digitalPinToInterrupt(pin.IRdetect), Interupt_IR_Detect, HIGH);
	else {
		// Skip ir sync setup
		t_sync = 1;
		DebugError("!!ERROR!! [setup] IR SENSOR DISABLED");
		while (PrintDebug());
	}

	// IR prox right
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Rt), Interupt_IRprox_Halt, FALLING);

	// IR prox left
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Lft), Interupt_IRprox_Halt, FALLING);

	// CLEAR LCD
	ClearLCD();

	// PRINT SETUP FINISHED
	sprintf(str, "[setup] FINISHED: Setup: free_ram=%dKB", freeMemory());
	DebugFlow(str);
	while (PrintDebug());

	// PRINT ALL IN QUEUE
	while (PrintDebug());

	// TEMP
	//Log.TestLoad(0, "LOG00074.CSV");
	//Log.TestLoad(5000);

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
	if (fc.doPackSend)
	{
		SendPacket();
	}
	// Send log
	else if (fc.doLogSend)
	{
		Log.SendLogEntry();
	}

	// PRINT DB
	if (fc.doPrint)
	{
		PrintDebug();
	}

	// GET AD STATUS
	AD_CheckOC();

	// GET BUTTON INPUT
	if (GetButtonInput())
	{

		// Button triggered reward
		if (fc.doBtnRew) {
			if (!Reward.isRewarding) {
				fc.isRewarding = Reward.StartRew(false, true);
			}
			fc.doBtnRew = false;
		}

		// Open/close Rew sol
		if (fc.doRewSolStateChange) {
			OpenCloseRewSolenoid();
			fc.doRewSolStateChange = false;
		}

		// Open/close EtOH sol
		if (fc.doEtOHSolStateChange) {
			OpenCloseEtOHSolenoid();
			fc.doEtOHSolStateChange = false;
		}

		// Turn LCD on/off
		if (fc.doChangeLCDstate) {
			ChangeLCDlight();
			fc.doChangeLCDstate = false;
		}

		// Move robot foward/backward
		if (fc.doMoveRobFwd) {
			ManualRun('f');
			fc.doMoveRobFwd = false;
		}
		else if (fc.doMoveRobRev) {
			ManualRun('r');
			fc.doMoveRobRev = false;
		}


	}

	// IR triggered halt
	if (v_doIRhardStop)
	{
		IRprox_Halt();
		v_doIRhardStop = false;
	}

	// Log new ir events
	if (v_doLogIR)
	{
		// Log event if streaming started
		sprintf(horeStr, "[loop] IR Sync Event: tot=%d dt=%dms", v_cnt_ir, v_dt_ir);
		DebugFlow(horeStr, v_t_irSyncLast);

		// Reset flag
		v_doLogIR = false;
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
				QueuePacket('c', 'Z', Reward.zoneIndByte + 1, (uint16_t)1, true);
		}
	}

	// Check if feeder arm should be moved
	Reward.CheckFeedArm();

	// Check if solonoids have been open more than 2 min
	if (digitalRead(pin.Rel_EtOH) == HIGH && millis() > (t_solOpen + 120000))
		// Close etoh sol
		OpenCloseEtOHSolenoid();
	if (digitalRead(pin.Rel_Rew) == HIGH && millis() > (t_solOpen + 120000))
		// Close rew sol
		OpenCloseRewSolenoid();

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
	if (!fc.isSesStarted)
	{
		// Pulse tracker while in wait state
		static bool is_on = false;
		static uint32_t t_pulse_last = millis() - 1001;
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
				abs(75 - v_dt_ir) < 10
				)
			{
				// Set sync time
				t_sync = v_t_irSyncLast;

				// Store and send CS handshake recieved
				QueuePacket('c', 'D', 255, c2r.packList[CharInd('+', c2r.idList, c2r.idLng)]);
				SendPacket();

				// Log/print sync time
				sprintf(horeStr, "SET SYNC TIME: %dms", t_sync);
				DebugFlow(horeStr);
			}
			// Restart loop
			else return;
		}

		// Indicate setup complete
		StatusBlink(5);

		// Make sure lcd led is off
		if (!fc.isManualSes &&
			fc.isLitLCD)
		{
			fc.doChangeLCDstate = true;
		}
		// Clear LCD
		ClearLCD();

		// Print ad board status
		sprintf(horeStr, "[loop] BOARD R STATUS: %04X", AD_R.getStatus());
		DebugFlow(horeStr);
		sprintf(horeStr, "[loop] BOARD F STATUS: %04X", AD_F.getStatus());
		DebugFlow(horeStr);

		fc.isSesStarted = true;
		DebugFlow("[loop] READY TO ROCK!");

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
			sprintf(horeStr, "[loop] RUN PID CALIBRATION = kC=%0.2f", kC);
			DebugFlow(horeStr);
		}

		// Update Hault Error test run speed
		else if (c2r.testCond == 2)
		{
			double new_speed = double(c2r.testDat);
			double speed_steps = new_speed*cm2stp;


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
			sprintf(horeStr, "[loop] HAULT ERROR SPEED = %0.0f cm/sec", new_speed);
			DebugFlow(horeStr);
		}
	}

	// Run position debugging
	if (do_posDebug)
	{
		static double rat_rob_dist;
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
		double new_speed = Pid.RunPidCalibration();
		double speed_steps;

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
			millis()%50 == 0
			{Pid.cal_isCalFinished}{"ERROR"}{Pid.cal_errNow}{Pid.cal_errArr[0]}{Pid.cal_errArr[1]}{Pid.cal_errArr[2]}{Pid.cal_errArr[3]}{"PERIOD"}{Pid.cal_PcNow}{Pid.cal_cntPcArr[0]}{Pid.cal_PcArr[0]}{Pid.cal_cntPcArr[1]}{Pid.cal_PcArr[1]}{Pid.cal_cntPcArr[2]}{Pid.cal_PcArr[2]}{Pid.cal_cntPcArr[3]}{Pid.cal_PcArr[3]}{Pid.cal_PcAll}
			*/
			millis();
			// Plot error
			/*
			{@Plot.Vel.Error.Red Pid.cal_errNow} {@Plot.Vel.Setpoint.Black 0}
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
			DebugFlow("[loop] DO TRACKING");
		}
		else
		{
			fc.isManualSes = true;
			DebugFlow("[loop] DONT DO TRACKING");
		}
		// Set reward tone
		if (c2r.soundCond == 0)
		{
			// No sound
			QueuePacket('a', 's', 0);
			DebugFlow("[loop] NO SOUND");
		}
		else if (c2r.soundCond == 1)
		{
			// Use white noise only
			QueuePacket('a', 's', 1);
			DebugFlow("[loop] DONT DO TONE");
		}
		else
		{
			// Use white and reward noise
			QueuePacket('a', 's', 2);
			DebugFlow("[loop] DO TONE");
		}

	}
#pragma endregion

#pragma region //--- (Q) DO QUIT ---
	if (c2r.idNew == 'Q' && c2r.isNew)
	{
		fc.doQuit = true;
		t_quitCmd = millis() + 1000;

		// Tell ard to quit
		QueuePacket('a', 'q');
		DebugFlow("[loop] DO QUIT");

		// Hold all motor control
		fc.isHalted = true;
		SetMotorControl("None", "MsgQ");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (
		fc.doQuit && millis() > t_quitCmd &&
		!CheckResend('a') &&
		!CheckResend('c') &&
		!fc.doPackSend)
	{
		// Quit session
		DebugFlow("[loop] QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (c2r.idNew == 'M' && c2r.isNew)
	{
		// Store move pos
		c2r.moveToTarg = c2r.dat[0];

		// Align target pos to feeder
		c2r.moveToTarg = c2r.moveToTarg - feedDist;
		c2r.moveToTarg = c2r.moveToTarg < 0 ? c2r.moveToTarg + (140 * PI) : c2r.moveToTarg;

		// Set flags
		fc.doMove = true;

		sprintf(horeStr, "[loop] DO MOVE: start=%0.2fcm targ=%0.2fcm", ekf.RobPos, c2r.moveToTarg);
		DebugFlow(horeStr);
	}

	// Perform movement
	if (fc.doMove)
	{

		// Compute move target
		if (!Targ.isTargSet)
		{
			// If succesfull
			if (Targ.CompTarg(ekf.RobPos, c2r.moveToTarg))
			{
				// Start running
				if (
					SetMotorControl("MoveTo", "MsgM") &&
					RunMotor(Targ.moveDir, moveToSpeed, "MoveTo")
					)
				{
					// Print message
					sprintf(horeStr, "[loop] RUNNING: MoveTo: start=%0.2fcm targ=%0.2fcm dist=%0.2fcm dir=\'%c\'",
						Targ.posStart, Targ.targPos, Targ.targDist, Targ.moveDir);
					DebugFlow(horeStr);
					// Set flag
					Targ.isMoveStarted = true;
				}
				// Failed to take motor control
				else {
					SetMotorControl("Open", "MsgM");
					// Set flags
					Targ.doAbortMove = true;
				}
			}
		}

		// Check if robot is ready to be stopped
		if (Targ.isTargSet)
		{
			// Do deceleration
			double new_speed = Targ.DecelToTarg(ekf.RobPos, ekf.RobVel, 40, 10);

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

			// Print final move status
			if (!Targ.doAbortMove)
			{
				// Tell CS movement is done
				QueuePacket('c', 'D', 255, c2r.packList[CharInd('M', c2r.idList, c2r.idLng)]);

				// Print success message
				sprintf(horeStr, "[loop] FINISHED: MoveTo: start=%0.2fcm targ=%0.2fcm dist=%0.2fcm error=%0.2fcm dir=\'%c\'",
					Targ.posStart, Targ.targPos, Targ.targDist, Targ.GetError(ekf.RobPos), Targ.moveDir);
				DebugFlow(horeStr);
			}
			else
			{
				// Print failure message
				sprintf(horeStr, "!!ERROR!! [loop] ABORTED: MoveTo: targ_set=%s ekf_ready=%s move_started=%s start=%0.2fcm targ=%0.2fcm dist=%0.2fcm error=%0.2fcm dir=\'%c\'",
					Targ.isTargSet ? "true" : "false", fc.isEKFReady ? "true" : "false", Targ.isMoveStarted ? "true" : "false", Targ.posStart, Targ.targPos, Targ.targDist, Targ.GetError(ekf.RobPos), Targ.moveDir);
				DebugError(horeStr);
			}

			// Reset flags
			fc.doMove = false;
			Targ.Reset();
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
			sprintf(horeStr, "[loop] REWARD FREE: pos=%0.2fcm occ_thresh=%dms",
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
			sprintf(horeStr, "[loop] REWARD CUED: pos=%0.2fcm occ_thresh=%ldms",
				c2r.rewPos, Reward.occThresh);
			DebugFlow(horeStr);
			fc.doRew = true;
		}

		// Imediate reward
		else if (c2r.rewPos == 0)
		{

			// Set mode
			Reward.SetRewMode("Now", c2r.rewZoneInd);
			DebugFlow("[loop] REWARD NOW");

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
				sprintf(horeStr, "[loop] SET REWARD ZONE: center=%0.2fcm from=%0.2fcm to=%0.2fcm",
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
					sprintf(horeStr, "[loop] REWARDED ZONE: occ=%dms zone=%d from=%0.2fcm to=%0.2fcm",
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
				sprintf(horeStr, "[loop] REWARD MISSED: rat=%0.2fcm bound_max=%0.2fcm",
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
			DebugFlow("[loop] HALT STARTED");
			// Stop pid and set to manual
			HardStop("MsgH");
			// Remove motor control
			fc.isHalted = true;
			SetMotorControl("None", "MsgH");
			fc.doHalt = false;
		}
		else
		{
			DebugFlow("[loop] HALT FINISHED");
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
				DebugFlow("[loop] SET BULLDOZE ON");
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
				DebugFlow("[loop] SET BULLDOZE OFF");
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
				DebugFlow("[loop] BULLDOZE ON");
			}
			else
			{
				// Turn bulldoze off
				Bull.TurnOff("MsgB");
				DebugFlow("[loop] BULLDOZE OFF");
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
			DebugFlow("[loop] RAT IN");
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

			DebugFlow("[loop] RAT OUT");
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
		QueuePacket('c', 'D', 255, c2r.packList[CharInd('V', c2r.idList, c2r.idLng)]);
		fc.doStreamCheck = false;
		DebugFlow("[loop] STREAMING CONFIRMED");
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (c2r.idNew == 'P' && c2r.isNew)
	{

		// Update Rat VT
		if (c2r.vtEnt == 0)
			RatVT.UpdatePos(c2r.vtCM[c2r.vtEnt], c2r.vtTS[c2r.vtEnt]);

		// Update Robot VT
		else
			RobVT.UpdatePos(c2r.vtCM[c2r.vtEnt], c2r.vtTS[c2r.vtEnt]);

	}
#pragma endregion

#pragma region //--- (L) SEND LOG ---
	if (c2r.idNew == 'L' && c2r.isNew)
	{
		// Stop logging
		db.Log = false;

		// Save first log request packet for later
		Log.donePack = Log.donePack != 0 ? Log.donePack :
			c2r.packList[CharInd('L', c2r.idList, c2r.idLng)];

		// Check if end of list reached
		fc.doLogSend = true;

		// Log/print
		DebugFlow("[loop] SEND LOG");

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
	double new_speed = Pid.UpdatePID();

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
