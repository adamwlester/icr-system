#pragma once

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
	const bool print_c2r = false;
	const bool print_r2c = false;
	const bool print_a2r = true;
	const bool print_r2a = false;
	const bool print_rcvdVT = false;
	const bool print_pid = false;
	const bool print_bull = false;
	const bool print_logMode = false;
	const bool print_logStore = false;
	const bool print_a2o = false;
	const bool print_o2a = false;
	const bool print_o2aRaw = false;
	const bool print_motorControl = true;
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
const float kC = 3; // critical gain [1.5,3,5]
const float pC = 1.75; // oscillation period [0,1.75,1.5]  
const double cal_speedSteps[4] = { 10, 20, 30, 40 }; // (cm/sec) [{ 10, 20, 30, 40 }]
int cal_nMeasPerSteps = 10;

#pragma endregion


#pragma region ============= VARIABLE SETUP ============

// Debugging general
uint32_t cnt_loop_tot = 0;
uint16_t cnt_loop_short = 0;

// Print debugging
const int printQueueSize = 15;
char printQueue[printQueueSize][300] = { { 0 } };
int printQueueIndStore = 0;
int printQueueIndRead = 0;

// Serial com general
const int sendQueueSize = 10;
const int sendQueueBytes = 10;
byte sendQueue[sendQueueSize][sendQueueBytes] = { { 0 } };
int sendQueueInd = sendQueueSize - 1;
const int resendMax = 3;
const int dt_resend = 100; // (ms)
const int dt_sendSent = 7; // (ms) 
const int dt_sendRcvd = 7; // (ms) 
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
const double scaleFrontAD = 1.0375;
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
const int dt_durEtOH[2] = { 100, 100 }; // (ms)
const int dt_delEtOH[2] = { 10000, 60000 }; // (ms)
uint32_t t_solOpen = 0;
uint32_t t_solClose = 0;

// Volt tracking
/*
Updated when EtOH relay opened
*/
const float bit2volt = 0.0164;
uint32_t t_vccUpdate = 0;
float vccNow = 0;
float vccCutoff = 11.6;
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
volatile bool v_stepTimerState = false;
volatile bool v_stepTimerActive = false;
volatile int v_cnt_steps = 0;
volatile int v_stepTarg = 0;

#pragma endregion 


#pragma region ============== STRUCT SETUP =============

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
	bool isEKFReady = false;
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
	int lng;
	char head;
	char foot;
	const char id[20];
	uint16_t pack[20];
	uint16_t packLast[20];
	uint16_t cnt_pack;
	int cnt_repeat;
	byte datList[20][3];
	uint32_t t_sentList[20];
	bool doRcvCheck[20];
	int cnt_resend[20];
};

// Serial to CS
R2 r2c
{
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
	{ 0 }
};

// Serial to other ard
R2 r2a
{
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
	{ 0 }
};

#pragma endregion 