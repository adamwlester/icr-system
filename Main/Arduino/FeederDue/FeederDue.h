#ifndef FeederDue_h
#define FeederDue_h

#pragma region ========== LIBRARIES & EXT DEFS =========

//----------LIBRARIES------------

// General
#include "FeederDue_PinMap.h"
//
#include "Arduino.h"
//
#include <string.h>

// Memory
#include <MemoryFree.h>


// Timers
#include <DueTimer.h>

// AutoDriver

#include <SPI.h>
//
#include "AutoDriver_Due.h"

// Pixy

#include <Wire.h> 
//
#include <PixyI2C.h>

// LCD
#include <LCD5110_Basic.h>

// TinyEKF
#define N 4     // States
#define M 6     // Measurements
#include <TinyEKF.h>

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

// Logging
#define DO_LOG 1
#define DO_FAST_LOG 0

// Console
#define DO_PRINT_DEBUG 0
#define DO_FAST_PRINT 0

// Other
#define DO_TEENSY_DEBUG 0

// Main debug flag
#if DO_PRINT_DEBUG || DO_TEENSY_DEBUG
 #define DO_DEBUG 1
#else
 #define DO_DEBUG 0
#endif

// DEBUG VIA TEENSY
#define DB_FUN_STR() StoreTeensyDebug(__FUNCTION__, __LINE__, freeMemory(), "S");
#define DB_FUN_END() StoreTeensyDebug(__FUNCTION__, __LINE__, freeMemory(), "E");

// DEBUGGING STRUCT
struct DB
{

	// Printing
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

	// Logging
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

	// Testing
	const bool do_posDebug = false; // I set
	const bool do_posPlot = false; // I set
	bool is_runTest = false; // set by system
	bool do_simRatTest = false; // set by system
	bool do_pidCalibration = false; // set by system
	volatile bool do_v_irSyncCalibration = false; // set by system

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
uint32_t cnt_loop_tot = 0;
uint16_t cnt_loop_short = 0;
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
int cnt_packBytesRead = 0;
int cnt_packBytesSent = 0;
int cnt_packBytesDiscarded = 0;

// START/QUIT
uint32_t t_ardQuit = 0;
uint32_t t_quit = 0;

// PIXY
const double pixyCoeff[5] = {
	0.000000043550534,
	-0.000023239535204,
	0.005033059128963,
	-0.677050955917591,
	75.424132382709260
};
const int dt_pixyCheck[2] = { 5, 10 }; // (ms)

// AUTODRIVER
const double cm2stp = 200 / (9 * PI);
const double stp2cm = (9 * PI) / 200;
const float maxSpeed = 100; // (cm) 
const float maxAcc = 80; // (cm) 
const float maxDec = 160; // (cm)
double runSpeedNow = 0;
char runDirNow = 'f';
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 1000; // (ms)
const double rearMotCoeff[5] = {
	0.000000044120830,
	-0.000007753772088,
	0.000418793299060,
	0.945620729817402,
	0.047445535533065,
};
const double frontMotCoeff[5] = {
	0.000000032971731,
	-0.000006928907732,
	0.000457085441358,
	0.972947848949920,
	0.021557249590414,
};

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
const float setPointBackpack = 50;
const float setPointImplant = 56;
const float guardDist = 4.5;
const float feedDist = 66;

// MOVEMENT
float moveToSpeedMax = 80; // (cm/sec)
float moveToSpeedMin = 15; // (cm/sec)
float moveToDecelDist = 30; // cm

// REWARD
const long armStepFreq = 1000; // (us)
const double dt_armStep = 1000; // (us)
const int dt_rewBlock = 15000; // (ms)
uint32_t t_rewBlockMove = 0; // (ms)

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
int trackLEDduty[2] = { 74, 255 }; // value between 0 and 255
const int rewLEDmin[2] = { 0, 3};
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
volatile uint16_t v_cnt_steps = 0;
volatile byte v_stepTarg = 0;
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
	byte sesCond = 0;
	byte taskCond = 0;
	byte soundCond = 0;
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
	uint16_t pack[20];
	uint16_t packLast[20];
	int packTot;
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
	Serial3,
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
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

// Serial to CS
R2 r2c
{
	// serial
	Serial3,
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
	// pack
	{ 0 },
	// packLast
	{ 0 },
	// cnt_pack
	UINT16_MAX / 2,
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
	Serial2,
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
	// t_rcvd
	0,
	// dt_rcvd
	0,
};


// Serial to other ard
R2 r2a
{
	// serial
	Serial2,
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
	uint16_t cnt_pack;
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
	// cnt_pack
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