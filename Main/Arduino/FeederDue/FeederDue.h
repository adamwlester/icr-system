
#ifndef FeederDue_h
#define FeederDue_h
#include "FeederDue_PinMap.h"

#pragma region =============== EXT DEFS ================

// TinyEKF
#define N 4     // States
#define M 6     // Measurements

//-------SOFTWARE RESET----------
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#pragma endregion 

#pragma region ============ DEBUG SETTINGS =============


// POWER SETTING
#define DO_AUTO_POWER 1

// DEBUG SETTING

// CONSOLE
#define DO_PRINT_DEBUG 1
#define DO_FAST_PRINT 0

// TEENSY LOGGING
#define DO_TEENSY_DEBUG 0

// OPENLOG LOGGING
#define DO_LOG 1
#define DO_FAST_LOG 0

// DEBUGGING STRUCT
struct DB
{

	// PRINTING

	const bool print_errors = true;
	const bool print_flow = true;
	const bool print_logging = false;
	const bool print_c2r = true;
	const bool print_r2c = true;
	const bool print_a2r = true;
	const bool print_r2a = true;
	const bool print_motorControl = true;
	const bool print_pid = true;
	const bool print_bull = true;
	const bool print_rcvdVT = false;
	const bool print_logMode = false;
	const bool print_logStore = false;
	const bool print_a2o = false;
	const bool print_o2a = false;
	const bool print_o2aRaw = false;
	const bool print_runSpeed = false;
	const bool print_pixy = false;

	// LOGGING

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
	const bool log_pixy = true;

	// Tracking data
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

	// TESTING

	// Manually set
	const bool do_posDebug = true; 
	const bool do_posPrint = false; 
	const bool do_posPlot = false; 

	// Set by system
	bool is_runTest = false;
	bool do_simRatTest = false; 
	bool do_pidCalibration = false; 
	volatile bool do_v_irSyncCalibration = false;

	// Other
	bool isErrLoop = false;
}
// Initialize
db;

// PID CALIBRATION
/*
Set kC and run ICR_Run.cs
*/
const float kC = 5; // critical gain [1.5,3,5]
const float pC = 1.9; // oscillation period [0,2.25,1.9]  
const double cal_speedSteps[4] = { 20, 40, 60, 80 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;

// MAIN DEBUG FLAG
#if DO_PRINT_DEBUG || DO_TEENSY_DEBUG
#define DO_DEBUG 1
#else
#define DO_DEBUG 0
#endif

// DEBUG VIA TEENSY

// Put at start of function
#define DB_FUN_STR() StoreTeensyDebug(__FUNCTION__, __LINE__, freeMemory(), "S");
// Put at end of function
#define DB_FUN_END() StoreTeensyDebug(__FUNCTION__, __LINE__, freeMemory(), "E");

#pragma endregion

#pragma region ============= VARIABLE SETUP ============

// FLOW/STATE FLAGS
struct FC
{
	char motorControl[25] = "None"; // ["None", "Halt", "Open", "MoveTo", "Bull", "Pid"]
	bool isSetup = false;
	bool isBlockingTill = false;
	bool doQuit = false;
	bool isQuitConfirmed = false;
	bool isHandShook = false;
	bool isSesStarted = false;
	bool doStreamCheck = false;
	bool isComsStarted = false;
	bool doSendVCC = false;
	bool isManualSes = false;
	bool isForageTask = false;
	bool isRatOnTrack = false;
	bool isTaskDone = false;
	bool isTrackingEnabled = false;
	bool doMove = false;
	bool doRew = false;
	bool doHalt = false;
	bool doBulldoze = false;
	bool doLogSend = false;
	bool doBlockLogQueue = false;
	bool doBlockPrintQueue = false;
	bool doBlockLogWrite = false;
	bool doBlockWriteLCD = false;
	bool isEKFReady = false;
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

// DEBUGGING GENERAL
uint32_t cnt_loopTot = 0;
uint16_t cnt_loopShort = 0;
uint16_t cnt_warn = 0;
uint16_t cnt_err = 0;
uint16_t warn_line[100] = { 0 };
uint16_t err_line[100] = { 0 };
const uint16_t n_pings = 5;
float dt_pingRoundTrip[2] = { 0 };
const uint16_t maxStoreStrLng = 300;
const uint16_t maxMsgStrLng = maxStoreStrLng - 50;

// PRINT DEBUGGING
const int printQueueSize = 30;
char printQueue[printQueueSize][maxStoreStrLng] = { { 0 } };
int printQueueIndStore = 0;
int printQueueIndRead = 0;

// SERIAL COM GENERAL
const int sendQueueSize = 10;
const int sendQueueBytes = 18;
const int resendMax = 5;
const int dt_resend = 100; // (ms)
const int dt_sendSent = 5; // (ms) 
const int dt_sendRcvd = 0; // (ms) 
int cnt_bytesRead = 0;
int cnt_bytesSent = 0;
int cnt_bytesDiscarded = 0;

// START/QUIT
uint32_t t_ardQuit = 0;
uint32_t t_quit = 0;

// PIXY
const uint8_t pixyAddress = 0x54;
const uint16_t pixyMaxBlocks = 1;
const int pixyOrd = 5;
double pixyCoeff[pixyOrd] = {0};
const double pixyPackCoeff[pixyOrd] = {
	0.000000031316275,
	-0.000017574918569,
	0.003934022977557,
	-0.650941078574675,
	81.611186742642417,
};
const double pixyCubeCoeff[pixyOrd] = {
	0.000000031316275,
	-0.000017574918569,
	0.003934022977557,
	-0.650941078574675,
	81.611186742642417,
};
double pixyShift = 0;
const double pixyPackShift = 0; // (cm)
const double pixyCubeShift = -5; // (cm)
const int dt_pixyCheck[2] = { 10, 20 }; // (ms)
uint16_t cnt_pixyReset = 0;

// AUTODRIVER
const double cm2stp = 200 / (9 * PI);
const double stp2cm = (9 * PI) / 200;
const float maxAccArr[2] = { 80, 30 }; // (cm/sec) 
float maxAcc = maxAccArr[0]; // (cm/sec) 
const float maxDec = 160; // (cm/sec)
const float maxSpeed = 100; // (cm/sec) 
double runSpeedNow = 0;
char runDirNow = 'f';
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 1000; // (ms)
const int velOrd = 5;
const double rearVelCoeff[velOrd] = {
	0.000000044120830,
	-0.000007753772088,
	0.000418793299060,
	0.945620729817402,
	0.047445535533065,
};
const double frontVelCoeff[velOrd] = {
	0.000000032971731,
	-0.000006928907732,
	0.000457085441358,
	0.972947848949920,
	0.021557249590414,
};

// BIGEASYDRIVER
const byte ezDirExtState = 1; // 1
const byte ezDirRetState = 0; // 0
const int16_t ezExtStps = 250; // 250
const int16_t ezRestStps = 50; // 50

// KALMAN MODEL
struct KAL
{
	double RatPos = 0;
	double RobPos = 0;
	double RatVel = 0;
	double RobVel = 0;
	int cnt_ekf = 0;
	uint32_t t_last = 0;
}
kal;

// PID SETTINGS
const float setPointHead = 70;
const float feedDist = 72;
const float guardDist = 4.5;

// MOVEMENT
float moveToSpeedMax = 80; // (cm/sec)
float moveToSpeedMin = 15; // (cm/sec)
float moveToDecelDist = 40; // cm

// REWARD
double feedTrackPastDist = 0; // (cm) 
const double feedHeadPastDist = 10; // (cm)
const long armStepFreq = 1000; // (us)
const double dt_armStep = 1000; // (us)
const int dt_rewBlock = 15000; // (ms)
uint32_t t_rewBlockMove = 0; // (ms)
const float solOpenScaleArr[2] = { 1, 0.5 };

// SOLONOIDS
/*
EtOH run after min time or distance
*/
const int dt_durEtOH[2] = { 100, 100 }; // (ms)
const int dt_delEtOH[2] = { 30000, 60000 }; // (ms)
uint32_t t_solOpen = 0;
uint32_t t_solClose = 0;

// BATTERY
/*
Updated when EtOH relay opened
*/
const float bit2volt = 0.01545; // was 0.0164;
const int vccMaxSamp = 100;
const int dt_vccUpdate = 5000;
const int dt_vccSend = 30000;
const int dt_vccPrint = 30000;
float vccArr[vccMaxSamp] = { 0 };
float vccNow = 0;
float vccAvg = 0;
float vccCutoff = 11.6;
const int dt_icUpdate = 10;
float icNow = 0;

// LEDs
const int trackLEDdutyDefault[2] = { 200, 250 }; // value between 0 and 255
int trackLEDduty[2] = { trackLEDdutyDefault[0], trackLEDdutyDefault[1] }; 
const int rewLEDmin[2] = { 0, 2};
int rewLEDduty[2] = { rewLEDmin[0], 15 }; // value between 0 and 255

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];

// INTERUPTS
volatile uint32_t t_sync = 0; // (ms)
volatile uint32_t v_t_irSyncLast = 0; // (ms)
volatile int v_dt_ir = 0;
volatile byte v_cnt_ir = 0;
volatile byte v_doIRhardStop = false;
volatile byte v_doLogIR = false;
volatile byte v_doBlockIR = false;
volatile byte v_stepState = false;
volatile byte v_doStepTimer = false;
volatile byte v_isArmMoveDone = false;
volatile int16_t v_cnt_steps = 0;
volatile uint16_t v_stepTarg = 0;
volatile byte v_stepDir = 'e'; // ['e','r']

#pragma endregion 

#pragma region ============ COM STRUCT SETUP ===========

const char cs_id_list[19] =
{
	'h', // setup handshake
	't', // hardware test
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
	'O', // confirm task or sleep done
	'U', // log size
	'D', // execution done
	'P', // position data
	'\0'
};

const char ard_id_list[7] =
{
	't', // hardware test
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
	'\0'
};

const char tnsy_id_list[1] =
{
	'\0'
};

// C2R command vars
struct CMD
{
	byte testCond = 0;
	byte testRun = 0;
	byte testDat = 0;
	byte sesMsg = 0;
	byte sesCond = 0;
	byte sesTask = 0;
	byte sesSound = 0;
	float sesSetpointHeadDist = 0;
	byte vtEnt = 0;
	float vtCM[2] = { 0,0 };
	uint32_t vtTS[2] = { 0,0 };
	byte cnt_move = 0;
	float moveToTarg = 0;
	float bullDel = 0;
	float bullSpeed = 0;
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
	const char *instID;
	const int lng;
	const char head;
	const char foot;
	const char *id;
	uint16_t packArr[20];
	uint16_t packLastArr[20];
	uint16_t packInd;
	uint32_t packTot;
	int cnt_repeat;
	int cnt_dropped;
	char idNow;
	bool isNew;
	float dat[3];
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
};

// Serial from Robot
struct R2
{
	USARTClass &port;
	const char *instID;
	const int lng;
	const char head;
	const char foot;
	const char *id;
	uint16_t packArr[20];
	uint16_t packLastArr[20];
	uint16_t packInd;
	uint16_t packTot;
	int cnt_repeat;
	float datMat[20][3];
	uint32_t t_sentListArr[20];
	bool do_rcvCheckArr[20];
	int cnt_repeatArr[20];
	int pinCTS;
	bool stateCTS;
	byte sendQueue[sendQueueSize][sendQueueBytes];
	int sendQueueIndStore;
	int sendQueueIndRead;
	uint32_t t_sent; // (ms)
	int dt_sent; // (ms)
};

// CS SERIAL COMS

// Initialize C2R
R4 c2r
{
	// serial
	Serial2,
	// instID
	"c2r",
	// lng
	strlen(cs_id_list),
	// head
	'<',
	// foot
	'>',
	// id
	cs_id_list,
	// packArr
	{ 0 },
	// packLastArr
	{ 0 },
	// packInd
	0,
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
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

// Serial to CS
R2 r2c
{
	// serial
	Serial2,
	// instID
	"r2c",
	// lng
	strlen(cs_id_list),
	// head
	'<',
	// foot
	'>',
	// id
	cs_id_list,
	// packArr
	{ 0 },
	// packLastArr
	{ 0 },
	// packInd
	UINT16_MAX / 2,
	// packTot
	0,
	// cnt_repeat
	0,
	// datMat
	{ { 0 } },
	// t_sentListArr
	{ 0 },
	// do_rcvCheckArr
	{ false },
	// cnt_repeatArr
	{ 0 },
	// pinCTS
	pin.X1a_CTS,
	// stateCTS
	false,
	// sendQueue
	{ { 0 } },
	// sendQueueIndStore
	0,
	// sendQueueIndRead
	0,
	// t_sent
	0,
	// dt_sent
	0,
};

// CHEETAHDUE SERIAL COMS

// Serial from other ard
R4 a2r
{
	// serial
	Serial3,
	// instID
	"a2r",
	// lng
	strlen(ard_id_list),
	// head
	'{',
	// foot
	'}',
	// id
	ard_id_list,
	// packArr
	{ 0 },
	// packLastArr
	{ 0 },
	// packInd
	0,
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
	// t_rcvd
	0,
	// dt_rcvd
	0,
};


// Serial to other ard
R2 r2a
{
	// serial
	Serial3,
	// instID
	"r2a",
	// lng
	strlen(ard_id_list),
	// head
	'{',
	// foot
	'}',
	// id
	ard_id_list,
	// packArr
	{ 0 },
	// packLastArr
	{ 0 },
	// packInd
	UINT16_MAX / 2,
	// packTot
	0,
	// cnt_repeat
	0,
	// datMat
	{ { 0 } },
	// t_sentListArr
	{ 0 },
	// do_rcvCheckArr
	{ false },
	// cnt_repeatArr
	{ 0 },
	// pinCTS
	pin.X1b_CTS,
	// stateCTS
	false,
	// sendQueue
	{ { 0 } },
	// sendQueueIndStore
	0,
	// sendQueueIndRead
	0,
	// t_sent
	0,
	// dt_sent
	0,
};

// TEENSY SERIAL COMS
#if DO_TEENSY_DEBUG

struct R42T
{
	UARTClass &port;
	const char *instID;
	const int lng;
	const char head;
	const char foot;
	const char *id;
	uint16_t packInd;
	int cnt_dropped;
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
};

// Initialize C2R
R42T r42t
{
	// serial
	Serial,
	// instID
	"t2r",
	// lng
	0,
	// head
	'(',
	// foot
	')',
	// id
	tnsy_id_list,
	// packInd
	0,
	// cnt_dropped
	0,
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

#endif

#pragma endregion 


#endif