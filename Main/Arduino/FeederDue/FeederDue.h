//######################################

//============ FeederDue.h =============

//######################################

#ifndef FEEDERDUE_H
#define FEEDERDUE_H

//========== DEBUG EXT DEFS =============

// DEBUG TO CONSOLE
#define DO_PRINT_DEBUG 0 // 0

// DEBUG TO OPENLOG LOGGER
#define DO_LOG 1 // 1

// DEBUT TO TEENSY LOGGER
#define DO_TEENSY_DEBUG 0 // 0

// IMIDATE PRINT/LOG
#define DO_FAST_PRINT 0 // 0
#define DO_FAST_LOG 0 // 0

// DEBUG SAFEVECTOR ERRORS TO CONSOLE
#define DO_VEC_DEBUG 1 // 0


//============= INCLUDE ================

// LOCAL
#include "FeederDue_PinMap.h"
//
#include "SafeVector.h"

//========== EXT DEFS OTHER ============

// TINY EKF
#define N 4     // States
#define M 6     // Measurements

// SOFTWARE RESET
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)


#pragma region ============ DEBUG SETTINGS ============

// DEBUGGING FLAGS STRUCT
struct DB_FLAG
{

	// PRINTING

	const bool print_errors = DO_PRINT_DEBUG && true; // true 
	const bool print_general = DO_PRINT_DEBUG && true; // true
	const bool print_c2r = DO_PRINT_DEBUG && true; // true
	const bool print_r2c = DO_PRINT_DEBUG && true; // true
	const bool print_r2cQueued = DO_PRINT_DEBUG && true; // true
	const bool print_a2r = DO_PRINT_DEBUG && true; // true
	const bool print_r2a = DO_PRINT_DEBUG && true; // true
	const bool print_r2aQueued = DO_PRINT_DEBUG && true; // true
	const bool print_motorControl = DO_PRINT_DEBUG && true; // true
	const bool print_pid = DO_PRINT_DEBUG && true; // true
	const bool print_bull = DO_PRINT_DEBUG && true; // true
	const bool print_rcvdVT = DO_PRINT_DEBUG && false; // false
	const bool print_runSpeed = DO_PRINT_DEBUG && false; // false
	const bool print_pixy = DO_PRINT_DEBUG && false; // false
	const bool print_logWrite = DO_PRINT_DEBUG && false; // false
	const bool print_openLog = DO_PRINT_DEBUG && false; // false
	const bool print_logMode = DO_PRINT_DEBUG && print_openLog && false; // false
	const bool print_a2o = DO_PRINT_DEBUG && print_openLog && false; // false
	const bool print_o2a = DO_PRINT_DEBUG && print_openLog && false; // false
	const bool print_o2aRaw = DO_PRINT_DEBUG && print_openLog && false; // false

	// LOGGING

	const bool log_errors = DO_LOG && true; // true
	const bool log_general = DO_LOG && true; // true
	const bool log_c2r = DO_LOG && true; // true
	const bool log_r2c = DO_LOG && true; // true
	const bool log_r2cQueued = DO_LOG && true; // true
	const bool log_a2r = DO_LOG && true; // true
	const bool log_r2a = DO_LOG && true; // true
	const bool log_r2aQueued = DO_LOG && true; // true
	const bool log_pid = DO_LOG && true; // true
	const bool log_bull = DO_LOG && true; // true
	const bool log_motorControl = DO_LOG && true; // true
	const bool log_runSpeed = DO_LOG && false; // false
	const bool log_pixy = DO_LOG && false; // false

	// Tracking data
	const bool log_pos = DO_LOG && false; // false
	const bool log_pos_rat_vt = DO_LOG && false; // false
	const bool log_pos_rat_pixy = DO_LOG && false; // false
	const bool log_pos_rob_vt = DO_LOG && false; // false
	const bool log_pos_rat_ekf = DO_LOG && false; // false
	const bool log_pos_rob_ekf = DO_LOG && false; // false
	const bool log_vel_rat_vt = DO_LOG && false; // false
	const bool log_vel_rat_pixy = DO_LOG && false; // false
	const bool log_vel_rob_vt = DO_LOG && false; // false
	const bool log_vel_rat_ekf = DO_LOG && false; // false
	const bool log_vel_rob_ekf = DO_LOG && false; // false

	// CHECK LOOP
	const bool do_loopCheck = true;
	const bool do_loopCheckDT = do_loopCheck && true;
	const bool do_loopCheckSerialOverflow = do_loopCheck && true;
	const bool do_loopCheckError = do_loopCheck && true;


	// TESTING

	// Manually set testing
	const bool do_manualTesting = false;
	const bool do_posDebug = do_manualTesting && false;
	const bool do_posPrint = do_manualTesting && do_posDebug && false;
	const bool do_posPlot = do_manualTesting && do_posDebug && false;
	const bool do_digitalpinGraph = do_manualTesting && false;
	const bool do_analogpinGraph = do_manualTesting && false;
	const bool do_irTimePrint = do_manualTesting && false;

	// Set by system
	bool do_systemTesting = false;
	bool do_simRatTest = false;
	bool do_pidCalibration = false;
	volatile bool do_v_irSyncTest = false;
};

// PID CALIBRATION
/*
Set kC and run ICR_Run.cs
*/
const float kC = 5; // critical gain [1.5,3,5]
const float pC = 1.9; // oscillation period [0,2.25,1.9]  
const double _cal_speedSteps[4] = { 20, 40, 60, 80 }; // (cm/sec) [{ 10, 20, 30, 40 }]
const byte n_calRuns = sizeof(_cal_speedSteps) / sizeof(_cal_speedSteps[0]);
const VEC<double> cal_speedSteps(n_calRuns, __LINE__, _cal_speedSteps);
int cal_nMeasPerSteps = 10;

// MAIN DEBUG FLAG
#if DO_PRINT_DEBUG || DO_TEENSY_DEBUG
#define DO_DEBUG 1
#else
#define DO_DEBUG 0
#endif

// DEBUG VIA TEENSY

// Put at start of function
#define DB_FUN_STR() SendTeensy(__FUNCTION__, __LINE__, freeMemory(), 'S');
// Put at end of function
#define DB_FUN_END() SendTeensy(__FUNCTION__, __LINE__, freeMemory(), 'E');

#pragma endregion


#pragma region ============ VARIABLE SETUP ============

// FLOW/STATE FLAGS
struct FC
{
	bool is_MotBlocking = false;
	bool do_Quit = false;
	bool is_QuitConfirmed = false;
	bool is_IRHandshakeDone = false;
	bool is_CSHandshakeDone = false;
	bool is_CheetahDueHandshakeDone = false;
	bool is_SesStarted = false;
	bool is_ComsStarted = false;
	bool do_SendVCC = false;
	bool is_ManualSes = false;
	bool is_ForageTask = false;
	bool is_RatOnTrack = false;
	bool is_TaskDone = false;
	bool is_TrackingEnabled = false;
	bool do_RunMove = false;
	bool do_RunRew = false;
	bool do_Halt = false;
	bool do_Bulldoze = false;
	bool do_LogSend = false;
	bool do_BlockLogQueue = false;
	bool do_BlockPrintQueue = false;
	bool do_BlockLogWrite = false;
	bool do_BlockWriteLCD = false;
	bool is_TeensyReady = false;
	bool is_EKFReady = false;
	bool do_EtOHRun = true;
	bool is_LitLCD = false;
	bool do_BtnRew = false;
	bool do_RewSolStateChange = false;
	bool do_EtOHSolStateChange = false;
	bool do_ChangeLCDstate = false;
	bool do_MoveRobFwd = false;
	bool do_MoveRobRev = false;
	bool do_BlockDetectIR = false;
	bool is_ErrLoop = false;
}
// Initialize
FC;

// DEBUGGING BUFFER PARAMETERS
const uint16_t buffMed = 50;
const uint16_t buffLrg = 250;
const uint16_t buffMax = buffLrg + 100;
const uint16_t buffTerm = 4;

// LOG QUEUE
const int LQ_Capacity = 40;
char LQ_Queue[LQ_Capacity][buffMax] = { { 0 } };
int LQ_StoreInd = 0;
int LQ_ReadInd = 0;

// PRINT QUEUE
const int PQ_Capacity = 30;
char PQ_Queue[PQ_Capacity][buffMax] = { { 0 } };
int PQ_StoreInd = 0;
int PQ_ReadInd = 0;

// DEBUGGING GENERAL
uint32_t dt_timeoutHandshake = 5000; // (ms)
uint32_t cnt_loopTot = 0;
byte cnt_loopShort = 0;
uint16_t cnt_errAD = 0;
uint16_t cnt_errEKF = 0;
uint32_t cnt_overflowRX = 0;
uint32_t cnt_timeoutRX = 0;
uint32_t cnt_irProxHaltR = 0;
uint32_t cnt_irProxHaltL = 0;
byte n_testPings = 0;
VEC<float> dt_pingRoundTrip(2, __LINE__);
int memStart = 0;
int memEnd = 0;

// SERIAL COM GENERAL
const uint16_t expectedSerialBufferSize = 1028;
const int tx_sizeMaxSend = SERIAL_BUFFER_SIZE / 4;
const int rx_sizeMaxSend = SERIAL_BUFFER_SIZE / 4;
int cnt_bytesRead = 0;
int cnt_bytesDiscarded = 0;

//UNION FOR SERIAL COMS
union UNION_SERIAL {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (uint16_t) 2 byte
	uint32_t i32; // (uint32_t) 4 byte
	float f; // (float) 4 byte
};
UNION_SERIAL U;

//UNION FOR TEENSY COMS
union UNION_TEENSY {
	byte b[16]; // (byte) 1 byte
	char c[16]; // (char) 1 byte
	uint16_t i16[8]; // (uint16_t) 2 byte
	uint32_t i32[4]; // (uint32_t) 4 byte
	uint64_t i64[2]; // (uint64_t) 8 byte
};
UNION_TEENSY Utnsy;

// START/QUIT
uint32_t t_ardQuit = 0;
uint32_t t_quit = 0;

// PIXY
const uint8_t pixyAddress = 0x54;
const uint16_t pixyMaxBlocks = 1;
const int pixyOrd = 5;
VEC<double> pixyCoeff(pixyOrd, __LINE__);
const double _pixyPackCoeff[pixyOrd] = {
	0.000000024551930,
	-0.000014298999215,
	0.003283287634146,
	-0.584298073942136,
	79.031445369696740,
};
const VEC<double> pixyPackCoeff(pixyOrd, __LINE__, _pixyPackCoeff);
const double _pixyCubeCoeff[pixyOrd] = {
	0.000000024551930,
	-0.000014298999215,
	0.003283287634146,
	-0.584298073942136,
	79.031445369696740,
};
const VEC<double> pixyCubeCoeff(pixyOrd, __LINE__, _pixyCubeCoeff);
double pixyShift = 0;
const double pixyPackShift = 0; // (cm)
const double pixyCubeShift = -5; // (cm)
const int _dt_pixyCheck[2] = { 10, 20 }; // (ms)
const VEC<int> dt_pixyCheck(2, __LINE__, _dt_pixyCheck);
uint16_t cnt_pixyReset = 0;

// AUTODRIVER
const double cm2stp = 200 / (9 * PI);
const double stp2cm = (9 * PI) / 200;
const float _maxAccArr[2] = { 80, 30 }; // (cm/sec) 
const VEC<float> maxAccArr(2, __LINE__, _maxAccArr);
float maxAcc = maxAccArr[0]; // (cm/sec) 
const float maxDec = 160; // (cm/sec)
const float maxSpeed = 100; // (cm/sec) 
uint16_t adR_stat = 0x0;
uint16_t adF_stat = 0x0;
const int dt_checkAD = 1000; // (ms)
const int velOrd = 5;
const double _rearVelCoeff[velOrd] = {
	0.000000044120830,
	-0.000007753772088,
	0.000418793299060,
	0.945620729817402,
	0.047445535533065,
};
const VEC<double> rearVelCoeff(velOrd, __LINE__, _rearVelCoeff);
const double _frontVelCoeff[velOrd] = {
	0.000000032971731,
	-0.000006928907732,
	0.000457085441358,
	0.972947848949920,
	0.021557249590414,
};
const VEC<double> frontVelCoeff(velOrd, __LINE__, _frontVelCoeff);

// MOTOR CONTROL
double runSpeedNow = 0;
char runDirNow = 'f';
namespace MC_CALL
{
	enum ID {
		HALT,
		MOVETO,
		BULL,
		PID,
		SETUP_TRACKING,
		BLOCKER,
		QUIT,
		OVERIDE
	};
	const char str_list_id[8][buffMed] =
	{ { "HALT" },{ "MOVETO" },{ "BULL" },{ "PID" },{ "SETUP_TRACKING" },{ "BLOCKER" },{ "QUIT" },{ "OVERIDE" } };
};
namespace MC_CON
{
	enum ID {
		HALT,
		MOVETO,
		BULL,
		PID,
		OPEN,
		HOLD,
	};
	const char str_list_id[6][buffMed] =
	{ { "HALT" },{ "MOVETO" },{ "BULL" },{ "PID" },{ "OPEN" },{ "HOLD" } };
};
MC_CON::ID motorControlNow = MC_CON::ID::HOLD;
MC_CON::ID motorControlLast = MC_CON::ID::HOLD;
const static char str_med_statusList[16][buffMed] =
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

// PID
const float setPointHead = 60;
const float feedDist = 72;
const float guardDist = 4.5;

// MOVEMENT
float moveToSpeedMax = 80; // (cm/sec)
float moveToSpeedMin = 15; // (cm/sec)
float moveToDecelDist = 40; // cm

// REWARD
const int rewZoneWdith = 5; // (deg)
double feedTrackPastDist = 0; // (cm) 
const double feedHeadPastDist = 15; // (cm)
const int dt_rewBlock = 30000; // (ms)
uint32_t t_blockMoter = 0; // (ms)
const float _solOpenScaleArr[2] = { 1, 0.5 };
const VEC<float> solOpenScaleArr(2, __LINE__, _solOpenScaleArr);

// BIGEASYDRIVER
const byte ezDirExtState = 1; // 1
const byte ezDirRetState = 0; // 0
const byte ezResetExtStps = 100; // max [255] 100
const byte ezRewExtStps = 200; // max [255] 255
const byte ezRestStps = 15; // max [255] 25
const double ezStepPeriod = 1000; // (us)

// SOLONOIDS
/*
EtOH run after min time or distance
*/
const int _dt_durEtOH[2] = { 100, 100 }; // (ms)
const VEC<int> dt_durEtOH(2, __LINE__, _dt_durEtOH);
const int _dt_delEtOH[2] = { 30000, 60000 }; // (ms)
const VEC<int> dt_delEtOH(2, __LINE__, _dt_delEtOH);
uint32_t t_solOpen = 0;
uint32_t t_solClose = 0;

// BATTERY
/*
OLD NOTE: Updated when EtOH relay opened
bit2volt = (3.3/1023) * (12.6/3)
Set pot to 12.6V == 3V
*/
const float vccCutoff = 11.0;
const float bit2volt = 0.0135; // 0.01334
const int vccMaxSamp = 100;
const int dt_vccUpdate = 5000;
const int dt_vccSend = 30000;
const int dt_vccPrint = 30000;
VEC<float> vccArr(vccMaxSamp, __LINE__);
float vccNow = 0;
float vccAvg = 0;

// LEDs
const byte _trackLEDdutyDefault[2] = { 175, 250 };
const VEC<byte> trackLEDdutyDefault(2, __LINE__, _trackLEDdutyDefault);
const byte _trackLEDduty[2] = { trackLEDdutyDefault[0], trackLEDdutyDefault[1] };
VEC<byte> trackLEDduty(2, __LINE__, _trackLEDduty);
const byte _rewLEDmin[2] = { 0, 2 };
const VEC<byte> rewLEDmin(2, __LINE__, _rewLEDmin);
const byte _rewLEDduty[2] = { rewLEDmin[0], 15 };
VEC<byte> rewLEDduty(2, __LINE__, _rewLEDduty);

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];

// IR SYNC
const int dt_irHandshakePulse = 75; // (ms) 75

// INTERUPTS/VOLATILES
volatile uint32_t t_sync = 0; // (ms)
volatile uint32_t v_t_irSyncLast = 0; // (ms)
volatile int v_dt_ir = 0;
volatile byte v_cnt_ir = 0;
volatile byte v_isNewIR = false;
volatile byte v_stepState = false;
volatile byte v_doStepTimer = false;
volatile byte v_isArmMoveDone = false;
volatile byte v_cnt_steps = 0;
volatile byte v_stepTarg = 0;
volatile byte v_stepDir = 'e'; // ['e','r']

#pragma endregion 


#pragma region ============== COM SETUP ===============

// SERIAL QUEUE 
const int SQ_Capacity = 10;
const int SQ_MsgBytes = 18;


// PACKET RANGE
const uint16_t _pack_range[2] = { 1, UINT16_MAX - 1 };

// COM INSTANCE ID
namespace COM
{
	enum ID {
		r2c,
		c2r,
		r2a,
		a2r,
		r2t,
		t2r
	};
	const char str_list_id[6][buffMed] =
	{ { "r2c" },{ "c2r" },{ "r2a" },{ "a2r" },{ "r2t" },{ "t2r" } };
}

// CS COMS IDs
const char _cs_id_list[18] =
{
	'h', // setup handshake
	'n', // ping test packets
	'T', // system test command
	'K', // feederdue status
	'S', // start session
	'Q', // quit session
	'M', // move to position
	'R', // run reward
	'H', // halt movement
	'B', // bulldoze rat
	'I', // rat in/out
	'L', // request log conf/send
	'J', // battery voltage
	'Z', // reward zone
	'O', // confirm task or sleep done
	'U', // log size
	'P', // position data
	'\0'
};

// CHEETAHDUE COM IDs
const char _ard_id_list[9] =
{
	'h', // setup handshake
	'n', // ping test packets
	't', // hardware test
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
	'\0'
};

const char _tnsy_id_list[2] =
{
	'.', // dummy
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
	byte cnt_move = 0;
	float moveTarg = 0;
	float bullDel = 0;
	float bullSpeed = 0;
	byte rewType = 0;
	float goalPos = 0;
	byte rewZoneOrDelay = 0;
	VEC<float> vtCM;
	VEC<uint32_t> vtTS;
	CMD() :
		vtCM(2, __LINE__),
		vtTS(2, __LINE__)
	{}
}
cmd;

// FEEDERDUE OUTGOING SERIAL
template <typename HW>
struct R2_COM
{
	HW &hwSerial;
	const COM::ID comID;
	const int lng;
	const char head;
	const char foot;
	VEC<char> id;
	VEC<uint16_t> packRange;
	VEC<uint16_t> packArr;
	VEC<uint16_t> packConfArr;
	uint16_t packInd;
	uint32_t packSentAll;
	uint32_t packRcvdAll;
	VEC<float> dat1;
	VEC<float> dat2;
	VEC<float> dat3;
	VEC<byte> flagArr;
	VEC<uint32_t> t_sentArr;
	VEC<uint32_t> t_queuedArr;
	VEC<bool> do_rcvCheckArr;
	VEC<int> cnt_repeatArr;
	uint32_t cnt_repeat;
	uint32_t t_sent; // (ms)
	int dt_sent; // (ms)
	int dt_minSentRcvd; // (ms) 
	int resendMax;
	int dt_resend; // (ms)
	byte SQ_Queue[SQ_Capacity][SQ_MsgBytes];
	int SQ_StoreInd;
	int SQ_ReadInd;
	R2_COM(HW &_hwSerial, const COM::ID _comID, const char _head, const char _foot, const char *_id, const uint16_t *_packRange, int _dt_minSentRcvd = 0, int _resendMax = 0, int _dt_resend = 0) :
		hwSerial(_hwSerial),
		comID(_comID),
		lng(strlen(_id)),
		head(_head),
		foot(_foot),
		id(lng, __LINE__, _id),
		packRange(2, __LINE__, _packRange),
		packArr(lng, __LINE__),
		packConfArr(lng, __LINE__),
		packInd(_packRange[0] - 1),
		packSentAll(0),
		packRcvdAll(0),
		dat1(lng, __LINE__),
		dat2(lng, __LINE__),
		dat3(lng, __LINE__),
		flagArr(lng, __LINE__),
		t_sentArr(lng, __LINE__),
		t_queuedArr(lng, __LINE__),
		do_rcvCheckArr(lng, __LINE__),
		cnt_repeatArr(lng, __LINE__),
		cnt_repeat(0),
		t_sent(0),
		dt_sent(0),
		dt_minSentRcvd(_dt_minSentRcvd),
		resendMax(_resendMax),
		dt_resend(_dt_resend),
		SQ_Queue(),
		SQ_StoreInd(0),
		SQ_ReadInd(0)
	{}
};
R2_COM<USARTClass> r2c(Serial2, COM::ID::r2c, '<', '>', _cs_id_list, _pack_range, 5, 5, 250);
R2_COM<USARTClass> r2a(Serial3, COM::ID::r2a, '{', '}', _ard_id_list, _pack_range, 5, 5, 100);
R2_COM<UARTClass> r2t(Serial, COM::ID::r2t, '{', '}', _tnsy_id_list, _pack_range);

// FEEDERDUE INCOMING SERIAL
template <typename HW>
struct R4_COM
{
	HW &hwSerial;
	const COM::ID comID;
	const int lng;
	const char head;
	const char foot;
	VEC<char> id;
	VEC<uint16_t> packRange;
	VEC<uint16_t> packArr;
	VEC<uint16_t> packConfArr;
	uint16_t packInd;
	uint32_t packSentAll;
	uint32_t packRcvdAll;
	VEC<float> dat;
	uint32_t cnt_repeat;
	uint32_t cnt_dropped;
	char idNew;
	bool is_new;
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
	int dt_minSentRcvd; // (ms) 
	R4_COM(HW &_hwSerial, const COM::ID _comID, const char _head, const char _foot, const char *_id, const uint16_t *_packRange, int _dt_minSentRcvd = 0) :
		hwSerial(_hwSerial),
		comID(_comID),
		lng(strlen(_id)),
		head(_head),
		foot(_foot),
		id(lng, __LINE__, _id),
		packRange(2, __LINE__, _packRange),
		packArr(lng, __LINE__),
		packConfArr(lng, __LINE__),
		packInd(_packRange[0] - 1),
		packSentAll(0),
		packRcvdAll(0),
		dat(3, __LINE__),
		cnt_repeat(0),
		cnt_dropped(0),
		idNew('\0'),
		is_new(false),
		t_rcvd(0),
		dt_rcvd(0),
		dt_minSentRcvd(_dt_minSentRcvd)
	{}
};
R4_COM<USARTClass> c2r(Serial2, COM::ID::c2r, '<', '>', _cs_id_list, _pack_range, 5);
R4_COM<USARTClass> a2r(Serial3, COM::ID::a2r, '{', '}', _ard_id_list, _pack_range, 5);
#if DO_TEENSY_DEBUG
R4_COM<UARTClass> t2r(Serial, COM::ID::t2r, '{', '}', _tnsy_id_list, _pack_range);
#endif

#pragma endregion 


#endif