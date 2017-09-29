#ifndef FeederDue_h
#define FeederDue_h

#pragma region ========== LIBRARIES & EXT DEFS =========

//----------LIBRARIES------------

// General
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

// LOG DEBUGGING
struct DB
{
	// Debugging
	const bool DEBUG = false;

	// Printing
	bool CONSOLE = false;
	bool LCD = false;
	// What to print
	const bool print_errors = true;
	const bool print_flow = true;
	const bool print_logging = false;
	const bool print_c2r = false;
	const bool print_r2c = false;
	const bool print_a2r = false;
	const bool print_r2a = false;
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
	const bool do_posDebug = false; // I set
	const bool do_posPlot = false; // I set
	bool do_pidCalibration = false; // set by system
	bool do_simRatTest = false; // set by system
	bool is_runTest = false; // set by system

	// Other
	bool isErrLoop = false;

	// Logging
	bool LOG = true;
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
}
// Initialize
db;

// Pid calibration parameters
/*
Set kC and run ICR_Run.cs
*/
const float kC = 5; // critical gain [1.5,3,5]
const float pC = 1.9; // oscillation period [0,2.25,1.9]  
const double cal_speedSteps[4] = { 20, 40, 60, 80 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;

#pragma endregion


#pragma region ============== PIN MAPPING ==============

// Pin mapping
struct PIN
{
	// Power off
	const int PWR_OFF = 45;
	const int PWR_ON = 44;
	const int PWR_Swtch = 24;
	const int PWR_Swtch_Grn = 25;

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
	const int FeedSwitch_Gnd = 33;
	const int FeedSwitch = 32;

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
	bool isQuitConfirmed = false;
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
byte n_pings = 5;
byte cnt_ping = 0;
uint32_t dt_ping[10][2] = { { 0 } };

// Print debugging
const int printQueueSize = 30;
char printQueue[printQueueSize][300] = { { 0 } };
int printQueueIndStore = 0;
int printQueueIndRead = 0;

// Serial com general
const int sendQueueSize = 10;
const int sendQueueBytes = 18;
const int resendMax = 5;
const int dt_resend = 100; // (ms)
const int dt_sendSent = 7; // (ms) 
const int dt_sendRcvd = 1; // (ms) 
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
const float maxDec = 160; // (cm)
double runSpeedNow = 0;
char runDirNow = 'f';
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 10; // (ms)
uint32_t t_checkAD = millis() + dt_checkAD; // (ms)
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
const int dt_durEtOH[2] = { 100, 100 }; // (ms)
const int dt_delEtOH[2] = { 30000, 60000 }; // (ms)
uint32_t t_solOpen = 0;
uint32_t t_solClose = 0;

// Battery tracking
/*
Updated when EtOH relay opened
*/
const float bit2volt = 0.01545; // was 0.0164;
const int dt_vccUpdate = 1000;
const int dt_vccCheck = 500;
const int dt_vccSend = 15000;
const int dt_vccPrint = 60000;
float vccNow = 0;
float vccCutoff = 11.6;
float batVoltArr[100] = { 0 };
const int dt_icUpdate = 10;
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
volatile bool v_doStepTimer = false;
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
	char *instID;
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
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
};

// Initialize C2R
R4 c2r
{
	// serial
	Serial3,
	// instID
	"c2r",
	// lng
	17,
	// head
	'<',
	// foot
	'>',
	// id
	 {
		'h', // setup handshake
		't', // ping test
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
		' ', ' ', ' '
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
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

// Serial from other ard
R4 a2r
{
	// serial
	Serial2,
	// instID
	"a2r",
	// lng
	6,
	// head
	'{',
	// foot
	'}',
	// id
	{
		't', // ping test
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
		' ', ' ', ' ', ' ', ' ',
		' ', ' ', ' ', ' ', ' ',
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
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

// Serial from Robot
struct R2
{
	USARTClass &port;
	char *instID;
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
	byte sendQueue[sendQueueSize][sendQueueBytes];
	int sendQueueIndStore;
	int sendQueueIndRead;
	uint32_t t_sent; // (ms)
	int dt_sent; // (ms)
};

// Serial to CS
R2 r2c
{
	// serial
	Serial3,
	// instID
	"r2c",
	// lng
	17,
	// head
	'<',
	// foot
	'>',
	// id
	{
		'h', // setup handshake
		't', // ping test
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
		' ', ' ', ' '
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

// Serial to other ard
R2 r2a
{
	// serial
	Serial2,
	// instID
	"r2a",
	// lng
	6,
	// head
	'{',
	// foot
	'}',
	// id
	{
		't', // ping test
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
		' ', ' ', ' ', ' ', ' ',
		' ', ' ', ' ', ' ', ' ',
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

#pragma endregion 


#endif