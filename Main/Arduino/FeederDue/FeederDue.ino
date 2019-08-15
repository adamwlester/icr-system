//######################################

//=========== FeederDue.ino ============

//######################################

//============== NOTES =================
/*

* ARDUINO DUE
	CPU						Atmel 32-bit SAM3XB8E
	Family					ARM Cortex-M3
	Clock Speed				84Mhz
	Operating Voltage		3.3V
	Input Voltage			7-12V
	Input Voltage (max)		6-20V
	Digital I/O Pins		54 (12 that support PWM)
	Analog Input Pins		12
	Analog Output Pins		2
	Flash Memory			512KB
	SRAM					96KB in 2 banks of 64KB and 32KB each

* XBEE
	DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size)
	DO (to UART rx) buffer = 202 bytes

* ARDUINO SERIAL_BUFFER_SIZE CHANGED FROM 128 TO "expectedSerialBufferSize"
	Path: "C:\Users\lester\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.8\cores\arduino\RingBuffer.h"

* DATA TYPES
	DATA TYPE				SIZE (IN BYTES)				RANGE
	short int					2						-32,768 to 32,767
	unsigned short int			2						0 to 65,535
	unsigned int				4						0 to 4,294,967,295
	int							4						-2,147,483,648 to 2,147,483,647
	long int					4						-2,147,483,648 to 2,147,483,647
	unsigned long int			4						0 to 4,294,967,295
	long long int				8						-(2^63) to (2^63)-1
	unsigned long long int		8						0 to 18,446,744,073,709,551,615
	signed char					1						-128 to 127
	unsigned char				1						0 to 255


* Step down resistor for vcc monitoring:
	To ground = 2.2k Ohm
	To vcc = 8.2k Ohm

* OpenLog 
config.txt settings:
	57600,26,3,2,0,0,0
	baud,escape,esc#,mode,verb,echo,ignoreRX
Reset
	Short 

* VARIABLE INFO
	"flag_byte" = [0, 0, 0, 0, is_resend, is_done, is_conf, do_conf]

*/


//============= INCLUDE ================

// LOCAL
#include "FeederDue.h"
//
#include "FeederDue_PinMap.h"
//
#include "SafeVector.h"

// GENERAL
#include <stdarg.h>

// MEMORY
#include <MemoryFree.h>

// TIMERS
#include <DueTimer.h>

// AUTODRIVER
#include <SPI.h>
//
#include <AutoDriver_Due.h>

// PIXY
#include <Wire.h> 

// LCD
#include <LCD5110_Basic.h>

// TinyEKF
#include <TinyEKF.h>


#pragma region ========== CLASS DECLARATIONS ==========

#pragma region ----------CLASS: DEBUG----------
class DEBUG
{
public:

	// VARIABLES
	DB_FLAG flag;
	uint16_t cnt_warn = 0;
	uint16_t cnt_err = 0;
	uint16_t err_cap = 100;
	VEC<uint16_t> warn_line;
	VEC<uint16_t> err_line;
	char buff_lrg_sprintfErr[buffLrg] = { 0 };
	char cnt_sprintfErr = 0;
	char buff_lrg_strcatErr[buffLrg] = { 0 };
	char cnt_strcatErr = 0;

	// METHODS
	DEBUG();
	// CHECK EVERY LOOP
	void CheckLoop();
	// LOG/PRINT MAIN EVENT
	void DB_General(const char *p_fun, int line, char *p_msg, uint32_t ts = millis());
	// LOG/PRINT WARNINGS
	void DB_Warning(const char *p_fun, int line, char *p_msg, uint32_t ts = millis());
	// LOG/PRINT ERRORS
	void DB_Error(const char *p_fun, int line, char *p_msg, uint32_t ts = millis());
	// LOG/PRINT RCVD PACKET
	void DB_Rcvd(R4_COM<USARTClass> *p_r4, char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte);
	// LOG/PRINT QUEUED SEND PACKET
	void DB_SendQueued(R2_COM<USARTClass> *p_r2, char *p_msg, uint32_t ts);
	// LOG/PRINT SENT PACKET
	void DB_Sent(R2_COM<USARTClass> *p_r2, char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte);
	// PRINT LOG WRITE
	void DB_LogWrite(char *p_msg);
	// LOG/PRINT MOTOR CONTROL DEBUG STRING
	void DB_MotorControl(const char *p_fun, int line, char *p_msg);
	// LOG/PRINT MOTOR SPEED CHANGE
	void DB_RunSpeed(const char *p_fun, int line, MC_CALL::ID caller, double speed_last, double speed_now);
	// LOG/PRINT PIXY 
	void DB_Pixy(const char *p_fun, int line, char *p_msg);
	// LOG/PRINT PID
	void DB_Pid(const char *p_fun, int line, char *p_msg);
	// LOG/PRINT BULLDOZE
	void DB_Bull(const char *p_fun, int line, char *p_msg);
	// LOG TRACKING
	void DB_TrackData();
	// LOG/PRINT OPENLOG ACTIVITY
	void DB_OpenLog(char *p_msg, bool start_entry = false);
	// STORE STRING FOR PRINTING
	void Queue(char *p_msg, uint32_t ts = millis(), char *p_type = "NOTICE", const char *p_fun = "", int line = 0);
	// PRINT DEBUG STRINGS TO CONSOLE
	bool Print();
	// PRINT EVERYTHING IN QUEUE
	bool PrintAll(uint32_t timeout);
	// PRINT DEBUG STRINGS TO LCD
	void PrintLCD(bool do_block, char *p_msg_1, char *p_msg_2 = { 0 }, char f_siz = 's');
	// CLEAR LCD
	void ClearLCD();
	// FORMAT SPECIAL CHARITERS FOR PRINTING
	char* FormatSpecialChars(char chr, bool do_show_byte = false);
	// FORMAT INT AS BINARY
	char* FormatBinary(unsigned int int_in);
	// FORMAT TIME STAMP STRING
	char* FormatTimestamp(uint32_t ts);
	// GET CURRENT NUMBER OF ENTRIES IN PRINT QUEUE
	int GetPrintQueueAvailable();
	// SAFE VERSION OF SPRINTF
	void sprintf_safe(uint16_t buff_cap, char *p_buff, char *p_fmt, ...);
	// SAFE VERSION OF STRCAT
	void strcat_safe(uint16_t buff_cap, uint16_t buff_lng_1, char *p_buff_1, uint16_t buff_lng_2, char *p_buff_2);
	// RUN ERROR HOLD WITH OPTIONAL SHUTDOWN
	void RunErrorHold(char *p_msg_print, char *p_msg_lcd, uint32_t dt_shutdown_sec = 0);

};
#pragma endregion

#pragma region ----------CLASS: PIXY----------
class PIXY
{
public:

	// VARIABLES
	uint16_t wordStr = 0xaa55;
	uint16_t wordStrCC = 0xaa56;
	uint16_t wordStrX = 0x55aa;
	uint16_t cnt_blocks = 0;
	uint16_t checksum = 0xffff;
	uint16_t wordNew = 0xffff;
	uint16_t wordLast = 0xffff;
	bool skipStart = false;
	struct Block
	{
		uint16_t sum() {
			return signature + x + y + width + height + angle;
		}
		uint16_t signature;
		uint16_t x;
		uint16_t y;
		uint16_t width;
		uint16_t height;
		uint16_t angle;
	};
	Block block{ 0,0,0,0,0,0 };
	enum BLOCKTYPE {
		NORMAL_BLOCK,
		CC_BLOCK
	};
	const char *p_str_list_blockType[2] =
	{ { "NORMAL_BLOCK" },{ "CC_BLOCK" } };
	BLOCKTYPE blockType = NORMAL_BLOCK;

	// METHODS
	PIXY();
	void PixyBegin();
	double PixyUpdate(bool is_hardware_test = false);
	uint16_t PixyGetBlocks();
	bool PixyCheckStart();
	uint16_t PixyGetWord();
	uint8_t PixyGetByte();

};
#pragma endregion

#pragma region ----------CLASS: POSTRACK----------
class POSTRACK
{
public:

	// VARIABLES
	int nSamp = 0;
	VEC<double> posArr; // (cm)
	VEC<uint32_t> t_tsArr; // (ms)
	int dt_skip = 0;
	double velNow = 0.0f; // (cm/sec)
	double velLast = 0.0f; // (cm/sec)
	double posCum = 0.0f; // (cm)
	double posAbs = 0.0f; // (cm)
	uint16_t cnt_err = 0;
	uint16_t cnt_swap = 0;
	uint32_t cnt_rec = 0;
	uint32_t cnt_samp = 0;
	uint32_t t_tsNow = 0;
	uint32_t t_update = millis();
	uint32_t t_rec = 0;
	uint32_t dt_recSum = 0;
	int nLaps = 0;
	bool isNew = false;
	bool is_streamStarted = false;
	enum POSOBJ {
		RatVT,
		RobVT,
		RatPixy
	};
	char *p_str_list_objID[3] =
	{ { "RatVT" },{ "RobVT" },{ "RatPixy" } };
	POSOBJ objID;

	// METHODS
	POSTRACK(POSOBJ obj_id, int n_samp);
	void UpdatePos(double pos_new, uint32_t ts_new, bool is_new_rec = true);
	double GetPos();
	double GetVel();
	void SwapPos(double set_pos, uint32_t ts);
	void PosReset(bool do_lap_reset = false);
};
#pragma endregion

#pragma region ----------CLASS: PID----------
class PID
{

public:

	// VARIABLES
	uint32_t t_updateLast = 0;
	double dt_update = 0;
	bool isPidUpdated = false;
	double p_term = 0;
	double i_term = 0;
	double d_term = 0;
	bool isFirstRun = true;
	bool isHolding4cross = false;
	double error = 0;
	double errorLast = 0;
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
	int cal_dt_min = 40; // (ms)
	VEC<int> cal_cntPcArr;
	int cal_stepNow = 0;
	float cal_PcCnt = 0; // oscillation count
	float cal_PcSum = 0; // oscillation period sum
	uint32_t cal_t_PcNow = 0;
	uint32_t cal_t_PcLast = 0;
	VEC<int> cal_PcArr;
	float cal_PcAvg = 0;
	float cal_PcNow = 0;
	float cal_PcAll = 0;
	float cal_errNow = 0;
	float cal_errLast = 0;
	float cal_errAvg = 0;
	VEC<int> cal_errArr;
	float cal_dt_update = 0;
	float cal_errCnt = 0;
	float cal_errSum = 0;
	float cal_errMax = 0;
	float cal_errMin = 0;
	double cal_ratPos = 0;
	double cal_ratVel = 0;
	bool cal_isPidUpdated = false;
	bool cal_isCalFinished = false;
	enum PIDMODE {
		MANUAL,
		AUTOMATIC,
		HOLDING
	};
	const char *p_str_list_pidMode[3] =
	{ { "MANUAL" },{ "AUTOMATIC" },{ "HOLDING" } };
	PIDMODE pidMode = MANUAL;

	// METHODS
	PID(const float kC, const float pC, const byte n_calRuns);
	double PidUpdate();
	void PidRun();
	void PidStop(PIDMODE set_mode = MANUAL);
	void PidHold();
	void PidReset();
	void PidCheckMotorControl();
	void PidCheckEKF(uint32_t ts);
	void PidResetEKF();
	void PidSetUpdateTime(uint32_t ts);
	double PidCalibration();
};
#pragma endregion

#pragma region ----------CLASS: BULLDOZE----------
class BULLDOZE
{
public:

	// VARIABLES
	uint32_t t_updateNext = 0;
	int dt_update = 50; // (ms)
	float moveMin = 5; // (cm)
	uint32_t t_bullNext = 0; // (ms)
	int bullDelay = 0; // (ms)
	double bullSpeed = 0;
	double posCheck = 0;
	double posCum = 0;
	double distMoved = 0;
	double guardPos = 0;
	bool isMoved = false;
	bool isTimeUp = false;
	bool isPassedReset = false;
	enum BULLSTATE {
		OFF,
		ON,
		HOLDING
	};
	const char *p_str_list_bullState[3] =
	{ { "OFF" },{ "ON" },{ "HOLD" } };
	BULLSTATE bullState = OFF;
	enum BULLMODE {
		ACTIVE,
		INACTIVE
	};
	char *p_str_bullMode[2] =
	{ { "ACTIVE" },{ "INACTIVE" } };
	BULLMODE bullMode = INACTIVE;

	// METHODS
	BULLDOZE();
	void UpdateBull();
	void BullReinitialize(float bull_delay, float bull_speed);
	void BullRun();
	void BullStop(BULLMODE set_mode = INACTIVE);
	void BullOn();
	void BullOff();
	void BullHold();
	void BullResume();
	void BullReset();
	void BullCheckMotorControl();
};
#pragma endregion

#pragma region ----------CLASS: MOVETO----------
class MOVETO
{
public:

	// VARIABLES
	int dt_moveTimeout = 10000;
	uint32_t t_moveTimeout = 0;
	byte cnt_move = 0;
	char str_med_move[buffMed] = { 0 };
	bool do_AbortMove = false;
	const int dt_update = 10;
	uint32_t t_updateNext = 0;
	double distLeft = 0;
	double startPosCum = 0;
	double targPosAbs = 0;
	double targDist = 0;
	char moveDir = 'f';
	double baseSpeed = 0;
	double lastSpeed = 0;
	bool isTargSet = false;
	bool isMoveStarted = false;
	bool isTargReached = false;
	double haltError = 0;
	enum MOVEEV {
		FIRST,
		LAST,
		OTHER
	};
	MOVEEV moveEv = FIRST;
	const char *p_str_list_moveEv[3] =
	{ { "FIRST" },{ "LAST" },{ "OTHER" } };

	// METHODS
	MOVETO();
	void ProcMoveCmd(byte cmd_cnt, float cmd_targ);
	bool RunMove();
	bool SetMoveTarg();
	double DecelToTarg(double dist_decel, double speed_min);
	double GetMoveError();
	void MoveToReset();

};
#pragma endregion

#pragma region ----------CLASS: REWARD----------
class REWARD
{
public:

	// VARIABLES
	int durationDefault = 2000; // (ms) 2000 
	const int _zoneRewDurs[9] = { // (ms)
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
	const int _zoneLocs[9] = { // (deg)
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
	const VEC<int> zoneRewDurs;
	const VEC<int> zoneLocs;
	static const int zoneLng =
		sizeof(_zoneLocs) / sizeof(_zoneLocs[0]);
	VEC<double> zoneBoundCumMin;
	VEC<double> zoneBoundCumMax;
	VEC<int> zoneOccTim;
	VEC<int> zoneOccCnt;
	VEC<double> zoneBoundCumRewarded;
	char str_med_rew[buffMed] = { 0 };
	int cnt_cmd = 0;
	int cnt_rew = 0;
	uint32_t t_nowZoneCheck = 0;
	uint32_t t_lastZoneCheck = 0;
	int rewDelay = 0; // (ms)
	int rewDuration = 0; // (ms) 
	float solOpenScale = 1;
	int zoneMin = 0;
	int zoneMax = 0;
	uint32_t t_rewStr = 0;
	uint32_t t_rewEnd = 0;
	uint32_t t_closeSol = 0;
	uint32_t t_retractArm = 0;
	uint32_t t_moveArmStr = 0;
	double goalPosCum = 0;
	bool isRewarding = false;
	bool isZoneTriggered = false;
	bool isAllZonePassed = false;
	bool is_ekfNew = false;
	int zoneInd = 0;
	int zoneRewarded = 0;
	int occRewarded = 0;
	int lapN = 0;
	uint32_t armMoveTimeout = 5000;
	bool do_ArmMove = false;
	bool do_ExtendArm = false;
	bool do_RetractArm = false;
	bool do_TimedRetract = false;
	bool isArmExtended = false;
	const int dt_step_high = 500; // (us)
	const int dt_step_low = 500; // (us)
	bool isArmStpOn = false;
	enum REWMODE {
		BUTTON,
		NOW,
		CUE,
		FREE,
	};
	const char *p_str_list_rewMode[4] =
	{ { "BUTTON" },{ "NOW" },{ "CUE" },{ "FREE" } };
	REWMODE rewMode = BUTTON;
	enum MICROSTEP {
		FULL,
		HALF,
		QUARTER,
		EIGHTH,
		SIXTEENTH
	};
	const char *p_str_microstep[5] =
	{ { "FULL" },{ "HALF" },{ "QUARTER" },{ "EIGHTH" },{ "SIXTEENTH" } };
	const MICROSTEP ezExtMicroStep = QUARTER;
	const MICROSTEP ezRetMicroStep = QUARTER;

	// METHODS
	REWARD();
	bool RunReward();
	void StartRew();
	bool CheckEnd();
	void ProcRewCmd(byte cmd_type, float cmd_goal = -1, int cmd_zone_delay = -1);
	void SetZoneDur(int zone_ind = -1);
	void SetZoneBounds(float cmd_goal);
	bool CheckZoneBounds();
	void ExtendFeedArm(byte ext_steps);
	void RetractFeedArm();
	void SetMicroSteps(MICROSTEP microstep);
	void CheckFeedArm();
	void RewardReset(bool was_rewarded = false);
};
#pragma endregion

#pragma region ----------CLASS: LOGGER----------
class LOGGER
{
public:

	// VARIABLES
	USARTClass &hwSerial;
	const char _str_sml_reset[buffTerm] = { 26,26,26 };
	const VEC<char> str_sml_reset;
	const char _str_sml_head[buffTerm] = { '<','<','<' };
	const VEC<char> str_sml_head;
	const char _str_sml_foot[buffTerm] = { '>','>','>' };
	const VEC<char> str_sml_foot;
	const char _str_sml_success[buffTerm] = { '>','>','>' };
	const VEC<char> str_sml_success;
	const char _str_sml_warnings[buffTerm] = { '>','>','*' };
	const VEC<char> str_sml_warnings;
	const char _str_sml_abort[buffTerm] = { '>','>','!' };
	const VEC<char> str_sml_abort;
	char buff_med_logFile[buffMed] = { 0 };
	char buff_med_fiCnt[buffMed] = { 0 };
	static const int maxBytesStore = 500;
	char buff_rcvdArr[maxBytesStore] = { 0 };
	uint32_t t_sent = 0; // (ms)
	uint32_t t_rcvd = 0; // (ms)
	uint32_t t_write = 0; // (us)
	const int dt_write = 50 * 1000; // (us)
	uint32_t t_beginSend = 0;
	int dt_beginSend = 1000;
	int cnt_logsStored = 0;
	char mode = ' '; // ['<', '>']
	int logNum = 0;
	int cnt_logBytesStored = 0;
	int cnt_logBytesSent = 0;
	bool isFileReady = false;

	// METHODS
	LOGGER(USARTClass &_hwSerial);
	bool Setup();
	int OpenNewLog();
	bool SetToCmdMode();
	void GetCommand();
	char SendCommand(char *p_msg, bool do_conf = true, uint32_t timeout = 5000);
	char GetReply(uint32_t timeout);
	bool SetToWriteMode();
	void QueueLog(char *p_msg, uint32_t ts = millis(), char *p_type = "NOTICE", const char *p_fun = "", int line = 0);
	bool WriteLog();
	bool WriteAll(uint32_t timeout);
	void StreamLogs();
	void TestLoad(int n_entry, char *p_log_file = '\0');
	int GetFileSize(char *p_log_file);
	int GetLogQueueAvailable();

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

#pragma endregion 


#pragma region ======== OBJECT INITIALIZATION =========

// Initialize DEBUG class instance
DEBUG Debug;

// Initialize FUSER class instance
FUSER ekf;

// Initialize AutoDriver_Due class instances
AutoDriver_Due AD_R(pin.AD_CSP_R, pin.AD_RST);
AutoDriver_Due AD_F(pin.AD_CSP_F, pin.AD_RST);

// Initialize PixyCom class instance
PIXY Pixy;

// Initialize LCD5110 class instance
LCD5110 LCD(pin.LCD_SCK, pin.LCD_MOSI, pin.LCD_DC, pin.LCD_RST, pin.LCD_CS);

// Initialize array of POSTRACK class instances
POSTRACK Pos[3] = {
	POSTRACK(POSTRACK::POSOBJ::RatVT, 4),
	POSTRACK(POSTRACK::POSOBJ::RobVT, 4),
	POSTRACK(POSTRACK::POSOBJ::RatPixy, 6)
};

// Initialize PID class instance
PID Pid(kC, pC, n_calRuns);

// Initialize BULLDOZE class instance
BULLDOZE Bull;

// Initialize MOVETO class instance
MOVETO Move;

// Initialize REWARD class instance
REWARD Reward;

// Initialize LOGGER class instance
LOGGER Log(Serial1);

// Initialize DueTimer class instance
DueTimer FeederArmTimer = DueTimer(1);


#pragma endregion 


#pragma region ========= FUNCTION DECLARATIONS ========

// CHECK FOR HANDSHAKE
bool CheckForHandshake();

// PARSE SERIAL INPUT
void GetSerial(R4_COM<USARTClass> *p_r4);

// PROCESS PIXY STREAM
uint16_t GetPixy(bool is_hardware_test);

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4_COM<USARTClass> *p_r4, char mtch = '\0');

// STORE PACKET DATA TO BE SENT
void QueuePacket(R2_COM<USARTClass> *p_r2, char id, float dat1 = 0, float dat2 = 0, float dat3 = 0, uint16_t pack = 0, bool do_conf = true, bool is_conf = false, bool is_done = false, bool is_resend = false);

// SEND SERIAL PACKET DATA
bool SendPacket(R2_COM<USARTClass> *p_r2);

// CHECK IF ARD TO ARD PACKET SHOULD BE RESENT
bool CheckResend(R2_COM<USARTClass> *p_r2);

// LOG FUNCTION RUN TO TEENSY
void SendTeensy(const char *p_fun, int line, int mem, char id, char *p_msg = { 0 });

// GET LAST TEENSY LOG
void ImportTeensy();

// RESET AUTODRIVER BOARDS
void AD_Reset(float max_acc, float max_dec, float max_speed);

// CONFIGURE AUTODRIVER BOARDS
void AD_Config(float max_acc, float max_dec, float max_speed);

// CHECK AUTODRIVER STATUS
void AD_CheckOC(bool force_check = false);

// HARD STOP
void HardStop(const char *p_fun, int line, bool do_block_hz = false);

// CHECK IF IR TRIGGERED
void Check_IRprox_Halt();

// RUN AUTODRIVER
bool RunMotor(char dir, double new_speed, MC_CALL::ID caller);

// RUN MOTOR MANUALLY
bool ManualRun(char dir);

// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(MC_CON::ID set_to, MC_CALL::ID caller);

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotor(int dt);

// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed();

// CHECK AUTODRIVER BOARD STATUS
int GetAD_Status(uint16_t stat_reg, char *p_status_name);

// DO SETUP TO BEGIN TRACKING
void InitializeTracking();

// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT();

// UPDATE EKF
void UpdateEKF();

// CHECK IR DETECTOR
void IR_SyncCheck();

// CHECK FOR BUTTON INPUT
bool GetButtonInput();

// PROCESS BUTTON INPUT
void ProcButtonInput();

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

// HANDLE SESSION QUIT
void QuitSession();

// RESTART ARDUINO
void RestartArduino();

// UPDATE TESTS
void TestUpdate();

// DO HARDWARE TEST
void HardwareTest(bool do_stress_test, bool do_pixy_test, bool do_ping_test);

// GET ID INDEX
template <typename R24> int ID_Ind(char id, R24 *p_r24);

// GET/SET BYTE BIT VALUE
bool GetSetByteBit(byte * b_set, int bit, bool do_set);

// BLINK LEDS AT RESTART/UPLOAD
bool StatusBlink(bool do_set = false, byte n_blinks = 0, uint16_t dt_led = 0, bool rat_in_blink = false);

// TIMER INTERUPT/HANDLER
void Interupt_TimerHandler();

// POWER OFF
void Interupt_Power();

// DETECT IR SYNC EVENT
void Interupt_IR_Detect();

#pragma endregion


#pragma region =========== CLASS DEFINITIONS ==========

#pragma region ----------CLASS: DEBUG----------

DEBUG::DEBUG() :
	warn_line(err_cap, __LINE__),
	err_line(err_cap, __LINE__)
{}

void DEBUG::CheckLoop()
{

	// Local static vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static uint32_t t_loop_last = millis();
	static int dt_loop = 0;
	static int dt_loop_last = 0;
	static int c_rx_last = 0;
	static int c_tx_last = 0;
	static int a_rx_last = 0;
	static int a_tx_last = 0;
	static uint32_t t_led = 0;
	static bool is_led_high = false;
	bool do_dt_db = false;
	bool do_cs_buff_db = false;
	bool do_ard_buff_db = false;
	bool do_loop_error_db = false;
	int c_rx = 0;
	int c_tx = 0;
	int a_rx = 0;
	int a_tx = 0;

	// Keep short count of loops
	cnt_loopShort++;

	// Track total loops
	cnt_loopTot++;

	// Check for VEC errors
#if DO_VEC_DEBUG
	// Log each error
	if (VEC_CNT_ERR > 0) {
		for (int i = 0; i < min(VEC_CNT_ERR, VEC_MAX_ERR); i++) {
			Debug.DB_Error(__FUNCTION__, __LINE__, VEC_STR_LIST_ERR[i]);
		}
	}
	// Reset counter
	VEC_CNT_ERR = 0;
#endif

	// Check for SPRINTF_SAFE error
	if (cnt_sprintfErr > 0) {
		// Use standard sprintf()
		sprintf(buff_lrg, "**SPRINTF_SAFE**: cnt=%d buff=\"%s\"",
			cnt_sprintfErr, buff_lrg_sprintfErr);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Reset counter and buff
	buff_lrg_sprintfErr[0] = '\0';
	cnt_sprintfErr = 0;

	// Check for STRCAT_SAFE error
	if (cnt_strcatErr > 0) {
		// Use standard sprintf()
		sprintf(buff_lrg, "**STRCAT_SAFE**: cnt=%d buff=\"%s\"",
			cnt_strcatErr, buff_lrg_strcatErr);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Reset counter and buff
	buff_lrg_strcatErr[0] = '\0';
	cnt_strcatErr = 0;

	// Bail till ses started
	if (!FC.is_SesStarted) {
		return;
	}

	// Flicker led
	if (!StatusBlink()) {
		if (millis() > t_led) {
			analogWrite(pin.LED_TRACKER, is_led_high ? trackLEDduty[0] : trackLEDduty[1]);
			is_led_high = !is_led_high;
			t_led = millis() + 100;
		}
	}

	// Bail if not doing loop check
	if (!flag.do_loopCheck) {
		return;
	}

	// Bail if this is a test run
	if (flag.do_systemTesting) {
		return;
	}

	// Get total data left in buffers
	c_rx = c2r.hwSerial.available();
	c_tx = SERIAL_BUFFER_SIZE - 1 - c2r.hwSerial.availableForWrite();
	a_rx = a2r.hwSerial.available();
	a_tx = SERIAL_BUFFER_SIZE - 1 - a2r.hwSerial.availableForWrite();

	// Compute current loop dt
	dt_loop = millis() - t_loop_last;
	t_loop_last = millis();

	// Check long loop time
	do_dt_db = flag.do_loopCheckDT && dt_loop > 60;

	// Check if either buffer more than 1/4 full
	do_cs_buff_db = flag.do_loopCheckSerialOverflow && (c_rx >= SERIAL_BUFFER_SIZE / 4 || c_tx >= SERIAL_BUFFER_SIZE / 4);
	do_ard_buff_db = flag.do_loopCheckSerialOverflow && (a_rx >= SERIAL_BUFFER_SIZE / 4 || a_tx >= SERIAL_BUFFER_SIZE / 4);

	// Check for loop error
	do_loop_error_db = flag.do_loopCheckError && FC.is_ErrLoop;
	FC.is_ErrLoop = false;

	if (
		do_dt_db ||
		do_cs_buff_db ||
		do_ard_buff_db ||
		do_loop_error_db
		)
	{

		// Get message id
		Debug.sprintf_safe(buffLrg, buff_lrg_2, "**CHANGE DETECTED** [CheckLoop] |%s%s%s%s:",
			do_dt_db ? "Loop DT Flagged|" : "",
			do_cs_buff_db ? "CS Buffer Flooding|" : "",
			do_ard_buff_db ? "ARD Buffer Flooding|" : "",
			do_loop_error_db ? "Error Flagged|" : "");

		// Log message
		Debug.sprintf_safe(buffLrg, buff_lrg, "%s cnt_loop:%d|%d dt_loop=%d|%d c_rx=%d|%d c_tx=%d|%d a_rx=%d|%d a_tx=%d|%d",
			buff_lrg_2, cnt_loopShort, cnt_loopTot, dt_loop, dt_loop_last, c_rx, c_rx_last, c_tx, c_tx_last, a_rx, a_rx_last, a_tx, a_tx_last);
		DB_General(__FUNCTION__, __LINE__, buff_lrg);

	}

	// Store vars
	dt_loop_last = dt_loop;
	c_rx_last = c_rx;
	c_tx_last = c_tx;
	a_rx_last = a_rx;
	a_tx_last = a_tx;
}

void DEBUG::DB_General(const char *p_fun, int line, char *p_msg, uint32_t ts)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_general;
	do_log = flag.log_general;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, ts, "NOTICE", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(p_msg, ts, "NOTICE", p_fun, line - 11);
	}

}

void DEBUG::DB_Warning(const char *p_fun, int line, char *p_msg, uint32_t ts)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_errors;
	do_log = flag.log_errors;

	// Set error flag
	FC.is_ErrLoop = true;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, ts, "**WARNING**", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(p_msg, ts, "**WARNING**", p_fun, line - 11);
	}

	// Store warning info
	warn_line[cnt_warn < 100 ? cnt_warn++ : 99] = Log.cnt_logsStored;

}

void DEBUG::DB_Error(const char *p_fun, int line, char *p_msg, uint32_t ts)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_errors;
	do_log = flag.log_errors;

	// Set error flag
	FC.is_ErrLoop = true;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, ts, "!!ERROR!!", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(p_msg, ts, "!!ERROR!!", p_fun, line - 11);
	}

	// Store error info
	err_line[cnt_err < 100 ? cnt_err++ : 99] = Log.cnt_logsStored;

}

void DEBUG::DB_Rcvd(R4_COM<USARTClass> *p_r4, char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';
	bool do_print = false;
	bool do_log = false;
	bool is_resend = false;
	bool is_conf = false;

	// Get print and log flags
	do_print =
		(p_r4->comID == COM::ID::c2r && ((Debug.flag.print_c2r && p_r4->idNew != 'P') || (Debug.flag.print_rcvdVT && p_r4->idNew == 'P'))) ||
		(p_r4->comID == COM::ID::a2r && Debug.flag.print_a2r);
	do_log =
		(p_r4->comID == COM::ID::c2r && Debug.flag.log_c2r && p_r4->idNew != 'P') ||
		(p_r4->comID == COM::ID::a2r && Debug.flag.log_a2r);

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Parse flag
	is_conf = GetSetByteBit(&flag_byte, 1, false);
	is_resend = GetSetByteBit(&flag_byte, 3, false);

	// Format prefix
	Debug.sprintf_safe(buffMed, buff_med_1, "   [%sRCVD%s:%s]",
		is_resend ? "RSND-" : is_repeat ? "RPT-" : "",
		is_conf ? "-CONF" : "",
		COM::str_list_id[p_r4->comID]);

	// Format message
	if (p_r4->idNew != 'P') {
		Debug.sprintf_safe(buffMax, buff_max, "%s %s %s",
			buff_med_1, p_msg_1, p_msg_2);
	}
	// Add samp dt for pos data
	else {
		U.f = p_r4->dat[2];
		Debug.sprintf_safe(buffMed, buff_med_2, "ts_int=%d dt_samp=%d",
			U.i32, millis() - Pos[cmd.vtEnt].t_update);
		Debug.sprintf_safe(buffMax, buff_max, " %s %s %s %s",
			buff_med_1, buff_med_2, p_msg_1, p_msg_2);
	}

	// Add to print queue
	if (do_print) {
		Queue(buff_max, p_r4->t_rcvd, "COM");
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(buff_max, p_r4->t_rcvd, "COM");
	}

}

void DEBUG::DB_SendQueued(R2_COM<USARTClass> *p_r2, char *p_msg, uint32_t ts)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print =
		(Debug.flag.print_r2cQueued && p_r2->comID == COM::ID::r2c) ||
		(Debug.flag.print_r2aQueued && p_r2->comID == COM::ID::r2a);
	do_log =
		(Debug.flag.log_r2cQueued && p_r2->comID == COM::ID::r2c) ||
		(Debug.flag.log_r2aQueued && p_r2->comID == COM::ID::r2a)
		;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format message
	Debug.sprintf_safe(buffMax, buff_max, "   [SEND-QUEUED:%s] %s", COM::str_list_id[p_r2->comID], p_msg);

	if (do_print) {
		Debug.Queue(buff_max, ts, "COM");
	}

	if (do_log) {
		Log.QueueLog(buff_max, ts, "COM");
	}

}

void DEBUG::DB_Sent(R2_COM<USARTClass> *p_r2, char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	bool do_print = false;
	bool do_log = false;
	bool is_conf = false;
	bool is_done = false;
	bool is_resend = false;

	// Get print and log flags
	do_print =
		(Debug.flag.print_r2c && p_r2->comID == COM::ID::r2c) ||
		(Debug.flag.print_r2a && p_r2->comID == COM::ID::r2a);
	do_log =
		(Debug.flag.log_r2c && p_r2->comID == COM::ID::r2c) ||
		(Debug.flag.log_r2a && p_r2->comID == COM::ID::r2a);

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Parse flag
	is_conf = GetSetByteBit(&flag_byte, 1, false);
	is_done = GetSetByteBit(&flag_byte, 2, false);
	is_resend = GetSetByteBit(&flag_byte, 3, false);

	// Format message
	Debug.sprintf_safe(buffMax, buff_max, "   [%sSENT%s%s:%s] %s %s",
		is_resend ? "RSND-" : is_repeat ? "RPT-" : "",
		is_conf ? "-CONF" : "",
		is_done ? "-DONE" : "",
		COM::str_list_id[p_r2->comID], p_msg_1, p_msg_2);

	// Store
	if (do_print) {
		Debug.Queue(buff_max, p_r2->t_sent, "COM");
	}

	if (do_log) {
		Log.QueueLog(buff_max, p_r2->t_sent, "COM");
	}

}

void DEBUG::DB_LogWrite(char *p_msg)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	bool do_print = false;

	// Get print and log flags
	do_print = flag.print_logWrite;

	// Bail if not set
	if (!do_print) {
		return;
	}

	// Add prefix
	Debug.sprintf_safe(buffMax, buff_max, "   [LOG-WRITE] %s", p_msg);

	// Store
	if (do_print) {
		Debug.Queue(buff_max, millis());
	}

}

void DEBUG::DB_MotorControl(const char *p_fun, int line, char *p_msg)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_motorControl;
	do_log = flag.log_motorControl;

	// Bail if neither set and passed
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, millis(), "NOTICE", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(p_msg, millis(), "NOTICE", p_fun, line - 11);
	}

}

void DEBUG::DB_RunSpeed(const char *p_fun, int line, MC_CALL::ID caller, double speed_last, double speed_now)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_runSpeed;
	do_log = flag.log_runSpeed;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format message
	Debug.sprintf_safe(buffMax, buff_max, "[%s:%d] Changed Motor Speed: caller=%s speed_last=%0.2f speed_new=%0.2f",
		p_fun, line - 23, MC_CALL::str_list_id[caller], speed_last, speed_now);

	// Add to print queue
	if (do_print) {
		Queue(buff_max, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(buff_max, millis());
	}

}

void DEBUG::DB_Pixy(const char *p_fun, int line, char *p_msg)
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = Debug.flag.print_pixy;
	do_log = Debug.flag.log_pixy;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Format data string
	Debug.sprintf_safe(buffLrg, buff_lrg, "block_type=%s word_new=%X word_last=%X checksum=%d blocks=%d",
		Pixy.p_str_list_blockType[Pixy.blockType], Pixy.wordNew, Pixy.wordLast, Pixy.checksum, Pixy.cnt_blocks);

	// Add to print queue
	if (do_print) {
		Debug.Queue(buff_lrg, millis(), "NOTICE", p_fun, line - 11);
	}
	// Add to log queue
	if (do_log) {
		Log.QueueLog(buff_lrg, millis(), "NOTICE", p_fun, line - 11);
	}
}

void DEBUG::DB_Pid(const char *p_fun, int line, char *p_msg)
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = Debug.flag.print_pid;
	do_log = Debug.flag.log_pid;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format string
	Debug.sprintf_safe(buffLrg, buff_lrg, "%s: mode=\"%s\" mot_ctrl=\"%s\"",
		p_msg, Pid.p_str_list_pidMode[Pid.pidMode], MC_CON::str_list_id[motorControlNow]);

	// Add to print queue
	if (do_print) {
		Debug.Queue(buff_lrg, millis(), "NOTICE", p_fun, line - 11);
	}
	// Add to log queue
	if (do_log) {
		Log.QueueLog(buff_lrg, millis(), "NOTICE", p_fun, line - 11);
	}
}

void DEBUG::DB_Bull(const char *p_fun, int line, char *p_msg)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = Debug.flag.print_bull;
	do_log = Debug.flag.log_bull;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format string
	Debug.sprintf_safe(buffLrg, buff_lrg, "%s: state=\"%s\" mode=\"%s\" mot_ctrl=\"%s\"",
		p_msg, Bull.p_str_list_bullState[Bull.bullState], Bull.p_str_bullMode[Bull.bullMode], MC_CON::str_list_id[motorControlNow]);

	// Add to print queue
	if (do_print) {
		Debug.Queue(buff_lrg, millis(), "NOTICE", p_fun, line - 11);
	}
	// Add to log queue
	if (do_log) {
		Log.QueueLog(buff_lrg, millis(), "NOTICE", p_fun, line - 11);
	}
}

void DEBUG::DB_TrackData()
{

	// Bail if not doing any pos logging
	if (!flag.log_pos) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	static const byte n_samps = 40;
	static int16_t pos_hist[10][n_samps] = { { 0 } };
	static const char str_med_mat[n_samps][buffMed] = { { "Rat VT Pos: |" } ,{ "Rat Px Pos: |" } ,{ "Rob VT Pos: |" } ,{ "Rat EKF Pos: |" } ,{ "Rob EKF Pos: |" },
	{ "Rat VT Vel: |" } ,{ "Rat Px Vel: |" } ,{ "Rob VT Vel: |" } ,{ "Rat EKF Vel: |" } ,{ "Rob EKF Vel: |" }
	};
	bool _do_log[10] = { flag.log_pos_rat_vt, flag.log_pos_rat_pixy, flag.log_pos_rob_vt, flag.log_pos_rat_ekf, flag.log_pos_rob_ekf,
		flag.log_vel_rat_vt, flag.log_vel_rat_pixy, flag.log_vel_rob_vt, flag.log_vel_rat_ekf, flag.log_vel_rob_ekf };
	static VEC<bool> do_log(10, __LINE__, _do_log);
	static int hist_ind = 0;
	static int cnt_last = 0;

	// Bail in ekf not new or not logging
	if (kal.cnt_ekf == cnt_last ||
		(!do_log[0] && !do_log[1] && !do_log[2] && !do_log[3])) {
		return;
	}

	// Initilazie store time
	static uint32_t t_last_log = kal.t_last;

	// Store pos values
	if (do_log[0]) {
		pos_hist[0][hist_ind] = Pos[0].posCum;
	}
	if (do_log[1]) {
		pos_hist[1][hist_ind] = Pos[2].posCum;
	}
	if (do_log[2]) {
		pos_hist[2][hist_ind] = Pos[1].posCum;
	}
	if (do_log[3]) {
		pos_hist[3][hist_ind] = kal.RatPos;
	}
	if (do_log[4]) {
		pos_hist[4][hist_ind] = kal.RobPos;
	}

	// Store vel values
	if (do_log[5]) {
		pos_hist[5][hist_ind] = Pos[0].velNow;
	}
	if (do_log[6]) {
		pos_hist[6][hist_ind] = Pos[2].velNow;
	}
	if (do_log[7]) {
		pos_hist[7][hist_ind] = Pos[1].velNow;
	}
	if (do_log[8]) {
		pos_hist[8][hist_ind] = kal.RatVel;
	}
	if (do_log[9]) {
		pos_hist[9][hist_ind] = kal.RobVel;
	}

	// Store counts
	hist_ind++;
	cnt_last = kal.cnt_ekf;

	// Check if hist filled
	if (millis() >= t_last_log + 1000 || hist_ind == n_samps) {

		// Store values in respective string
		for (int i = 0; i < 10; i++) {

			if (!do_log[i]) {
				continue;
			}

			// Add identfiyer string
			Debug.sprintf_safe(buffMax, buff_max, "%s", str_med_mat[i]);

			// Store each value in string
			for (int j = 0; j < hist_ind; j++) {

				Debug.sprintf_safe(buffMed, buff_med, "%d|", pos_hist[i][j]);
				Debug.strcat_safe(buffMax, strlen(buff_max), buff_max, strlen(buff_med), buff_med);
			}

			// Log
			Log.QueueLog(buff_max, kal.t_last);
		}

		// Reset vals
		hist_ind = 0;
		t_last_log = kal.t_last;
	}
}

void DEBUG::DB_OpenLog(char *p_msg, bool start_entry)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Bail if logging should not be printed
	if (!Debug.flag.print_openLog) {
		return;
	}

	// Print like normal entry
	if (start_entry) {
		Debug.DB_General(__FUNCTION__, __LINE__, p_msg, millis());
		// Print right away
		Debug.PrintAll(500);
	}

	// Print directly 
	else {
		SerialUSB.print(p_msg);
	}
	}

void DEBUG::Queue(char *p_msg, uint32_t ts, char *p_type, const char *p_fun, int line)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

#if DO_PRINT_DEBUG

	// Local vars
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';
	static char buff_med_3[buffMed] = { 0 }; buff_med_3[0] = '\0';
	static char buff_med_4[buffMed] = { 0 }; buff_med_4[0] = '\0';
	static char buff_med_5[buffMed] = { 0 }; buff_med_5[0] = '\0';
	static char buff_med_save[buffMed] = { 0 };
	static uint16_t overflow_cnt = 0;
	uint32_t t_m = 0;

	// Bail if queue store blocked
	if (FC.do_BlockPrintQueue) {
		return;
	}

	// Update queue ind
	PQ_StoreInd++;

	// Check if ind should roll over 
	if (PQ_StoreInd == PQ_Capacity) {
		PQ_StoreInd = 0;
	}

	// Handle overflow
	if (PQ_Queue[PQ_StoreInd][0] != '\0') {

		// Add to count 
		overflow_cnt += overflow_cnt < 1000 ? 1 : 0;

		// Update string
		Debug.sprintf_safe(buffMed, buff_med_save, "[*DB_Q-DROP:%d*] ", overflow_cnt);

		// Set queue back
		PQ_StoreInd = PQ_StoreInd - 1 >= 0 ? PQ_StoreInd - 1 : PQ_Capacity - 1;

		// Bail
		return;
	}

	// Get sync correction
	t_m = ts - t_sync;

	// Make time string
	Debug.sprintf_safe(buffMed, buff_med_1, "[%s][%d]", FormatTimestamp(t_m), cnt_loopShort);

	// Add space after time
	Debug.sprintf_safe(buffMed, buff_med_2, "%%%ds", 20 - strlen(buff_med_1));
	Debug.sprintf_safe(buffMed, buff_med_3, buff_med_2, '_');

	// Add type
	if (strcmp(p_type, "!!ERROR!!") == 0 || strcmp(p_type, "**WARNING**") == 0) {
		Debug.sprintf_safe(buffMed, buff_med_4, "%s ", p_type);
	}

	// Add function and line number
	if (strcmp(p_fun, "") != 0) {
		Debug.sprintf_safe(buffMed, buff_med_5, "[%s:%d] ", p_fun, line);
	}

	// Put it all together
	Debug.sprintf_safe(buffMax, PQ_Queue[PQ_StoreInd], "%s%s%s%s%s%s\n", buff_med_1, buff_med_3, buff_med_save, buff_med_4, buff_med_5, p_msg);

	// Reset overlow stuff
	overflow_cnt = 0;
	buff_med_save[0] = '\0';

	// Print now
#if DO_FAST_PRINT
	Debug.PrintAll(500);
#endif

#endif
}

bool DEBUG::Print()
{

#if DO_PRINT_DEBUG

	// Bail if nothing in queue
	if (GetPrintQueueAvailable() == PQ_Capacity) {
		return false;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Incriment send ind
	PQ_ReadInd++;

	// Check if ind should roll over 
	if (PQ_ReadInd == PQ_Capacity) {
		PQ_ReadInd = 0;
	}

	// Print
	SerialUSB.print(PQ_Queue[PQ_ReadInd]);

	// Set entry to null
	PQ_Queue[PQ_ReadInd][0] = '\0';

	// Return success
	return true;

#else
	return false;

#endif
}

bool DEBUG::PrintAll(uint32_t timeout)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	uint16_t cnt_print = 0;
	int queue_size = 0;
	uint32_t t_timeout = millis() + timeout;
	bool is_timedout = false;

	// Loop till done or timeout reached
	while (Debug.Print()) {

		// Incriment counter
		cnt_print++;

		// Check for timeout
		if (millis() > t_timeout) {
			is_timedout = true;
			break;
		}

	}

	// Get queue left
	queue_size = PQ_Capacity - Debug.GetPrintQueueAvailable();

	// Check for timeout
	if (is_timedout) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: cnt_print%d queued=%d dt_run=%d",
			cnt_print, queue_size, timeout);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Return all printed flag
	return queue_size == 0;

}

void DEBUG::PrintLCD(bool do_block, char *p_msg_1, char *p_msg_2, char f_siz)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Check if printing blocked
	if (do_block) {
		FC.do_BlockWriteLCD = false;
	}
	else if (FC.do_BlockWriteLCD) {
		return;
	}

	// Reset
	LCD.clrScr();
	LCD.invert(true);
	LCD.setFont(SmallFont);

	// Print
	if (p_msg_2[0] != '\0')
	{
		LCD.print(p_msg_1, 5, 24 - 4);
		LCD.print(p_msg_2, 5, 24 + 4);
	}
	else {
		LCD.print(p_msg_1, 5, 24);
	}


	// Prevent overwrite till cleared
	if (do_block) {
		FC.do_BlockWriteLCD = true;
	}
	}

void DEBUG::ClearLCD()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Clear
	LCD.clrScr();

	// Stop blocking LCD log
	FC.do_BlockWriteLCD = false;
}

char* DEBUG::FormatSpecialChars(char chr, bool do_show_byte)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

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
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	byte b = chr;

	for (int i = 0; i < buffMed; i++) {
		buff_med[i] = '\0';
	}

	if (!do_show_byte)
	{
		// Get normal and special chars in quots
		switch (b) {
		case 10: Debug.sprintf_safe(buffMed, buff_med, "\\n\n"); break;
		case 9: Debug.sprintf_safe(buffMed, buff_med, "\\t\t"); break;
		case 11: Debug.sprintf_safe(buffMed, buff_med, "\\v\v"); break;
		case 8: Debug.sprintf_safe(buffMed, buff_med, "\\b\b"); break;
		case 13: Debug.sprintf_safe(buffMed, buff_med, "\\r\r"); break;
		case 12: Debug.sprintf_safe(buffMed, buff_med, "\\f\f"); break;
		case 7: Debug.sprintf_safe(buffMed, buff_med, "\\a\a"); break;
		case 92: Debug.sprintf_safe(buffMed, buff_med, "\\\\"); break;
		case 63: Debug.sprintf_safe(buffMed, buff_med, "\\?\?"); break;
		case 39: Debug.sprintf_safe(buffMed, buff_med, "\\'\""); break;
		case 34: Debug.sprintf_safe(buffMed, buff_med, "\\\"\""); break;
		case 0: Debug.sprintf_safe(buffMed, buff_med, "\\0\0"); break;
		default: Debug.sprintf_safe(buffMed, buff_med, "%c", b); break;
		}
	}
	else
	{
		switch (b) {
		case 10: Debug.sprintf_safe(buffMed, buff_med, "[10]\'\\n\'\n"); break;
		case 9: Debug.sprintf_safe(buffMed, buff_med, "[9]\'\\t\'\t"); break;
		case 11: Debug.sprintf_safe(buffMed, buff_med, "[11]\'\\v\'\v"); break;
		case 8: Debug.sprintf_safe(buffMed, buff_med, "[8]\'\\b\'\b"); break;
		case 13: Debug.sprintf_safe(buffMed, buff_med, "[13]\'\\r\'\r"); break;
		case 12: Debug.sprintf_safe(buffMed, buff_med, "[12]\'\\f\'\f"); break;
		case 7: Debug.sprintf_safe(buffMed, buff_med, "[7]\'\\a\'\a"); break;
		case 92: Debug.sprintf_safe(buffMed, buff_med, "[92]\'\\\'\\"); break;
		case 63: Debug.sprintf_safe(buffMed, buff_med, "[63]\'\\?\'\?"); break;
		case 39: Debug.sprintf_safe(buffMed, buff_med, "[39]\'\\'\'\""); break;
		case 34: Debug.sprintf_safe(buffMed, buff_med, "[34]\'\\\"\'\""); break;
		case 0: Debug.sprintf_safe(buffMed, buff_med, "[0]\'\\0\'\0"); break;
		default: Debug.sprintf_safe(buffMed, buff_med, "[%d]\'%c\'", b, b); break;
		}
	}

	return buff_med;
}

char* DEBUG::FormatBinary(unsigned int int_in)
{
	static char bit_str[100]; bit_str[0] = '\0';
	UNION_SERIAL U;
	byte bit_ind = 0;

	// Check for zero
	if (int_in == 0) {
		Debug.sprintf_safe(100, bit_str, "00000000");
	}

	else {
		U.i32 = int_in;

		bool do_write = false;
		for (int i = 3; i >= 0; i--)
		{
			do_write = do_write || U.b[i] > 0;
			if (!do_write) {
				continue;
			}

			for (int j = 7; j >= 0; j--) {
				bit_str[bit_ind++] = ((U.b[i] >> j) & 0x01) == 1 ? '1' : '0';
			}

			if (i > 0) {
				bit_str[bit_ind++] = ',';
			}
		}
		bit_str[bit_ind++] = '\0';
	}

	return bit_str;
}

char* DEBUG::FormatTimestamp(uint32_t ts)
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	int ts_m = 0;
	int s = 0;
	int ts_s = 0;
	int ts_ms = 0;

	// Get minutes
	ts_m = (ts - (ts % (60 * 1000))) / (60 * 1000);

	// Get seconds
	s = ts - (ts_m * 60 * 1000);
	ts_s = (s - (s % 1000)) / 1000;

	// Get milliseconds
	ts_ms = ts - (ts_m * 60 * 1000) - (ts_s * 1000);

	// Format string
	Debug.sprintf_safe(buffMed, buff_med, "%02u:%02u:%03u", ts_m, ts_s, ts_ms);
	return buff_med;

}

int DEBUG::GetPrintQueueAvailable() {
#if DO_TEENSY_DEBUG
	//DB_FUN_STR();
#endif

	// Local vars
	int n_entries = 0;

	// Check each entry
	for (int i = 0; i < PQ_Capacity; i++) {
		n_entries += PQ_Queue[i][0] != '\0' ? 1 : 0;
	}

	// Get total available
	return PQ_Capacity - n_entries;
}

void DEBUG::sprintf_safe(uint16_t buff_cap, char *p_buff, char *p_fmt, ...) {

	// Local vars
	static const uint16_t buff_size = buffMax * 2;
	static char buff[buff_size]; buff[0] = '\0';
	const char str_prfx_med[buffMed] = "*T*";
	int cut_ind = 0;

	// Reset output buffer
	p_buff[0] = '\0';

	// Hide Due specific intellisense errors
#ifndef __INTELLISENSE__

	// Get variable length input arguments
	va_list args;

	// Begin retrieving additional arguments
	va_start(args, p_fmt);

	// Use vsnprintf to format string from argument list
	vsnprintf(buff, buff_size, p_fmt, args);

	// End retrval
	va_end(args);

#endif

	// Formated string too long
	if (strlen(buff) + 1 > buff_cap) {

		// Store part of buff for error
		int err_str_len = min(50, buff_cap);
		for (int i = 0; i < err_str_len; i++)
		{
			buff_lrg_sprintfErr[i] = buff[i];
		}
		buff_lrg_sprintfErr[err_str_len] = '\0';
		cnt_sprintfErr++;

		// Get cut ind and truncate buff
		cut_ind = buff_cap - strlen(str_prfx_med);

		// Check for negative cut ind
		if (cut_ind > 0) {
			// Cut buff
			buff[cut_ind] = '\0';
		}
		else {
			// Clear buff
			buff[0] = '\0';
		}

		// Append error prefix
		strcat(p_buff, str_prfx_med);


}

	// Copy/concat buff to p_buff
	strcat(p_buff, buff);

}

void DEBUG::strcat_safe(uint16_t buff_cap, uint16_t buff_lng_1, char *p_buff_1, uint16_t buff_lng_2, char *p_buff_2)
{
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	const char str_prfx_med[buffMed] = "*T*";
	int cut_ind = 0;

	// Make sure buffer large enough
	if (buff_cap > buff_lng_1 + buff_lng_2) {

		// Concatinate strings
		strcat(p_buff_1, p_buff_2);
	}

	// String too long
	else {

		// Store part of buff for error
		int err_str_len = min(50, buff_lng_1);
		for (int i = 0; i < err_str_len; i++)
		{
			buff_lrg_strcatErr[i] = p_buff_1[i];
		}
		buff_lrg_strcatErr[err_str_len] = '\0';
		cnt_strcatErr++;

		// Insert prefix at front of buffer 1
		for (int i = 0; i < strlen(str_prfx_med); i++)
		{
			p_buff_1[i] = str_prfx_med[i];
		}

	}

}

void DEBUG::RunErrorHold(char *p_msg_print, char *p_msg_lcd, uint32_t dt_shutdown_sec)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static uint32_t t_shutdown = 0;
	byte _duty[2] = { 255, 0 };
	VEC<byte> duty(2, __LINE__, _duty);
	bool do_led_on = true;
	int dt_cycle = 100;
	float t_s = 0;

	// Compute shutdown time
	t_shutdown = dt_shutdown_sec > 0 ? millis() + (dt_shutdown_sec * 1000) : 0;

	// Log error message
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUN ERROR HOLD FOR %d sec: %s", dt_shutdown_sec, p_msg_print);
	DB_Error(__FUNCTION__, __LINE__, buff_lrg);

	// Turn on LCD LED
	ChangeLCDlight(128);

	// Get time seconds
	t_s = (float)(millis() - t_sync) / 1000.0f;

	// Print error
	Debug.sprintf_safe(buffLrg, buff_lrg, "ERROR %0.2fs", t_s);
	PrintLCD(true, p_msg_lcd, buff_lrg, 't');

	// Loop till shutdown
	while (true)
	{
		// Log anything in queue
		Log.WriteLog();
		Print();

		// Flicker lights
		analogWrite(pin.LED_REW_R, duty[(int)do_led_on]);
		analogWrite(pin.LED_TRACKER, duty[(int)do_led_on]);
		do_led_on = !do_led_on;

		// Pause
		delay(dt_cycle);

		// Check if should shutdown
		if (t_shutdown > 0 && millis() > t_shutdown) {

			// Set kill switch high
			digitalWrite(pin.PWR_OFF, HIGH);

			// Restart
			RestartArduino();
		}

	}
}

#pragma endregion 

#pragma region ----------CLASS: PIXY----------

PIXY::PIXY() {}

void PIXY::PixyBegin()
{
	// Begin I2C
	Wire.begin();

	// Set clock frequency
	Wire.setClock(100000); // 100k default

}

double PIXY::PixyUpdate(bool is_hardware_test)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static uint32_t t_check = 0;
	static bool do_power_reset = false;
	static bool do_pixy_reset = false;
	static uint32_t t_power_reset = 0;
	double pixy_cum = 0;
	double pixy_abs = 0;
	uint32_t t_pixy_ts = 0;
	double pixy_pos_y = 0;
	uint16_t blocks = 0;

	// Bail if robot not streaming yet
	if (!is_hardware_test &&
		!Pos[1].is_streamStarted) {
		return pixy_cum;
	}

	// Bail if task done
	if (FC.is_TaskDone) {
		return pixy_cum;
	}

	// Bail if rat not on track or doing simulation test
	if (!is_hardware_test &&
		(!FC.is_RatOnTrack || Debug.flag.do_simRatTest)) {
		return pixy_cum;
	}

	// Bail if not time to check
	if (millis() < t_check) {
		return pixy_cum;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Check if resetting power
	if (do_power_reset) {

		// Check if time to turn power back on
		if (millis() > t_power_reset + 25) {

			// Turn 5V power back on
			digitalWrite(pin.REG_5V1_ENBLE, HIGH);

			// Set flags
			do_power_reset = false;
			do_pixy_reset = true;

			// Log error
			Debug.sprintf_safe(buffLrg, buff_lrg, "POWERING ON 5V PIXY POWER SUPPLY: dt_power_off=%d", millis() - t_power_reset);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);

		}

		// Bail
		return pixy_cum;

	}

	// Check if resetting pixy I2C
	if (do_pixy_reset) {

		// Check if time to reset
		if (millis() > t_power_reset + 50) {

			// Restart com
			PixyBegin();

			// Reset flag
			do_pixy_reset = false;

			// Log error
			Debug.sprintf_safe(buffLrg, buff_lrg, "RESETTING PIXY COM: dt_power_off=%d", millis() - t_power_reset);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);

		}

		// Bail
		return pixy_cum;

	}

	// Get new blocks
	blocks = Pixy.PixyGetBlocks();

	// Check for pixy error
	if (blocks == UINT16_MAX - 1) {

		// Incriment count
		cnt_pixyReset++;

		// Log error
		Debug.sprintf_safe(buffLrg, buff_lrg, "PixyCom.GetBlocks() RETURNED ERROR: pixy_com_cnt=%d", cnt_pixyReset);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

		// Turn off 5V power
		Debug.DB_Warning(__FUNCTION__, __LINE__, "POWERING OFF 5V PIXY POWER SUPPLY");
		digitalWrite(pin.REG_5V1_ENBLE, LOW);

		// Set flag and store time
		do_power_reset = true;
		t_power_reset = millis();

		// Bail
		return pixy_cum;

	}

	// Bail if no new data
	if (blocks == 0) {

		// Set next check with short dt
		t_check = millis() + dt_pixyCheck[0];

		// Bail
		return pixy_cum;
	}

	else {
		// Set next check with longer dt
		t_check = millis() + dt_pixyCheck[1];
	}

	// Save time stamp
	t_pixy_ts = millis();

	// Get Y pos from last block
	pixy_pos_y = Pixy.block.y;

	// Transform to CM
	for (int i = 0; i < pixyOrd; i++) {
		pixy_cum += pixyCoeff[i] * pow(pixy_pos_y, pixyOrd - 1 - i);
	}

	// Shift pixy data
	pixy_cum = pixy_cum + pixyShift;

	// Return rel val if testing
	if (is_hardware_test) {
		return pixy_cum;
	}

	// Scale to abs space with rob vt data
	pixy_abs = pixy_cum + Pos[1].posAbs;
	pixy_abs = pixy_abs > (140 * PI) ? pixy_abs - (140 * PI) : pixy_abs;

	// Update pixy pos and vel
	Pos[2].UpdatePos(pixy_abs, t_pixy_ts);

	// Log first sample
	if (!Pos[2].is_streamStarted) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FIRST RAT PIXY RECORD: pos_abs=%0.2f pos_cum=%0.2f n_laps=%d",
			Pos[2].posAbs, Pos[2].posCum, Pos[2].nLaps);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		Pos[2].is_streamStarted = true;
	}

	// Return relative pos
	return pixy_cum;
}

uint16_t PIXY::PixyGetBlocks()
{
	/*
	I2C BLOCK FORMAT:
	Bytes    16-bit words   Description
	----------------------------------------------------------------
	0, 1     0              sync (0xaa55)
	2, 3     1              checksum (sum of all 16-bit words 2-6)
	4, 5     2              signature number
	6, 7     3              x center of object
	8, 9     4              y center of object
	10, 11   5              width of object
	12, 13   6              height of object
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	uint16_t word_sync = 0;

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Reset variables
	checksum = 0xffff;
	cnt_blocks = 0;

	// Check for frame start
	if (!skipStart) {

		// Bail if start not found
		if (!PixyCheckStart()) {
			return cnt_blocks;
		}
	}

	// Reset skip start flag
	else {
		skipStart = false;
	}

	// Loop through pixyBlocksMax
	for (cnt_blocks = 0; cnt_blocks < pixyMaxBlocks;)
	{
		// Get checksum
		checksum = PixyGetWord();

		// Check for sync word of the next frame
		if (checksum == wordStr || checksum == wordStrCC) {

			// Save next frame block type
			if (checksum == wordStr) {
				blockType = NORMAL_BLOCK;
			}
			else if (checksum == wordStrCC) {
				blockType = CC_BLOCK;
			}

			// Flag skip start
			skipStart = true;

			// Log
			Debug.DB_Pixy(__FUNCTION__, __LINE__, "Reached Frame End");

			// Bail
			break;

		}

		// Pixy has not found any obects (i.e., returned zero)
		else if (checksum == 0) {

			// Log
			Debug.DB_Pixy(__FUNCTION__, __LINE__, "No New Data");

			// Bail
			break;
		}

		// Get and store data
		block.signature = PixyGetWord();
		block.x = PixyGetWord();
		block.y = PixyGetWord();
		block.width = PixyGetWord();
		block.height = PixyGetWord();

		// Get angle for color code blocks
		if (blockType == CC_BLOCK) {
			block.angle = PixyGetWord();
		}
		else {
			block.angle = 0;
		}

		// Check that expected bytes read
		if (checksum == block.sum()) {

			// Incriment block count
			cnt_blocks++;

			// Log
			Debug.DB_Pixy(__FUNCTION__, __LINE__, "Block Recieved");
		}

		// Checksum error
		else {

			// Log error
			Debug.sprintf_safe(buffLrg, buff_lrg, "CHECKSUM MISSMATCH: checksum=%d read_sum=%d", checksum, block.sum());
			Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

			// Return value indicating error
			return UINT16_MAX - 1;
		}

		// Bail if max blocks reached
		if (cnt_blocks == pixyMaxBlocks) {
			break;
		}

		// Get sync word for next block
		word_sync = PixyGetWord();

		// Start of next block
		if (word_sync == wordStr) {
			blockType = NORMAL_BLOCK;
		}
		else if (word_sync == wordStrCC) {
			blockType = CC_BLOCK;
		}
		// End of frame
		else {
			break;
		}

	}

	// Return block count
	return cnt_blocks;
}

bool PIXY::PixyCheckStart()
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool pass = false;

	// Reinitialize last word
	wordLast = 0xffff;

	// Read 1-2 words
	for (int i = 0; i < 2; i++)
	{

		// Get next word
		wordNew = PixyGetWord();

		// Check if we have read two sync words
		if ((wordNew == wordStr || wordNew == wordStrCC) &&
			wordLast == wordStr) {

			// Set pass flag
			pass = true;

			// Log
			Debug.DB_Pixy(__FUNCTION__, __LINE__, "Recieved Start");

			// Bail
			break;
		}

		// Pixy has not found any obects (i.e., returned zero)
		else if (wordNew == 0 && wordLast == 0) {

			// Bail
			delayMicroseconds(10);
			break;
		}

		// Check if com is juxtaposed
		else if (wordNew == wordStrX)
		{
			// Correct out of sync
			PixyGetByte();

			// Log warning
			Debug.DB_Warning(__FUNCTION__, __LINE__, "PIXY COM OUT OF SYNC");

			// Bail
			break;
		}

		// Read next word
		else {

			// Save last word
			wordLast = wordNew;
		}

	}

	// Check for pass
	if (pass) {

		// Check for normal block
		if (wordNew == wordStr) {
			blockType = NORMAL_BLOCK;
		}

		// Check for color code block
		else if (wordNew == wordStrCC) {
			blockType = CC_BLOCK;
		}

	}

	// Return status
	return pass;

}

uint16_t PIXY::PixyGetWord()
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	uint16_t w1;
	uint8_t w2;

	// Request two bytes from Pixy
	Wire.requestFrom((int)pixyAddress, 2);
	w2 = Wire.read();
	w1 = Wire.read();

	// Do something to the binary data ???
	w1 <<= 8;
	w1 |= w2;

	// Return word
	return w1;


}

uint8_t PIXY::PixyGetByte()
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Request single byte from Pixy
	Wire.requestFrom((int)pixyAddress, 1);
	return Wire.read();
}


#pragma endregion 

#pragma region ----------CLASS: POSTRACK----------

POSTRACK::POSTRACK(POSOBJ obj_id, int n_samp) :
	posArr(n_samp, __LINE__),
	t_tsArr(n_samp, __LINE__)
{
	this->objID = obj_id;
	this->nSamp = n_samp;

	for (int i = 0; i < n_samp; i++) {
		this->posArr[i] = 0.0f;
	}
}

void POSTRACK::UpdatePos(double pos_new, uint32_t ts_new, bool is_new_rec)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	double pos_diff = 0;
	double dist = 0;
	double dist_sum = 0;
	int dt = 0;
	int dt_sum = 0;
	double dt_sec = 0;
	double vel_diff = 0;
	double vel = 0;

	// Store input
	this->t_update = millis();
	this->posAbs = pos_new;
	this->t_tsNow = ts_new;

	// Update itteration count
	this->cnt_samp++;

	// Update record info
	if (is_new_rec) {
		this->dt_recSum += millis() - this->t_rec <= 999 ? millis() - this->t_rec : this->cnt_rec > 0 ? 999 : 0;
		this->t_rec = millis();
		this->cnt_rec++;
	}

	// Shift and add data
	for (int i = 0; i < this->nSamp - 1; i++)
	{
		this->posArr[i] = this->posArr[i + 1];
		this->t_tsArr[i] = this->t_tsArr[i + 1];
	}
	// Add new variables
	this->posArr[this->nSamp - 1] = pos_new;
	this->t_tsArr[this->nSamp - 1] = ts_new;

	// Do not process early samples
	if (this->cnt_samp < this->nSamp + 1) {

		this->posCum = this->posCum;
		this->velNow = 0;

		// Set flag
		this->isNew = false;

		// Bail 
		return;
	}
	// Ready to use
	else {
		this->isNew = true;
	}

	// COMPUTE TOTAL DISTANCE RAN

	// Check for zero crossing
	pos_diff = pos_new - this->posArr[this->nSamp - 2];
	if (abs(pos_diff) > 140 * PI * 0.75) {

		// Crossed over
		if (pos_diff < 0) {
			this->nLaps++;
		}

		// Crossed back
		else {
			this->nLaps--;
		}
	}

	// Store cumulative position in cm
	this->posCum = pos_new + this->nLaps*(140 * PI);

	// COMPUTE VELOCITY
	for (int i = 0; i < this->nSamp - 1; i++)
	{
		// Compute sample distance
		dist = this->posArr[i + 1] - this->posArr[i];

		// Correct for zero crossing
		if (abs(dist) > 140 * PI * 0.75) {
			dist = ((140 * PI) - abs(dist)) * abs(dist) == dist ? -1 : 1;
		}

		// Add to total
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
	vel_diff = abs(this->velNow - vel);

	// Check for problem vals
	bool do_skip_vel =
		dt_sec == 0 ||
		(this->dt_skip < 500 && vel_diff > 300);

	// Ignore outlyer values unless too many frames discarted
	if (!do_skip_vel) {
		this->velLast = this->velNow;
		this->velNow = vel;
		this->dt_skip = 0;
	}
	// Add to skip time
	else {
		this->dt_skip += dt;
	}

	// Check for errors
	bool is_error =
		(FC.is_RatOnTrack || this->objID == RobVT) &&
		(dt_sec == 0 ||
			vel_diff > 300);

	// Log error
	if (is_error) {

		// Add to count
		this->cnt_err++;

		// Log first and every 10 errors
		if (this->cnt_err == 1 ||
			this->cnt_err % 10 == 0) {

			// Log warning
			Debug.sprintf_safe(buffLrg, buff_lrg, "Bad Values |%s%s: obj=\"%s\" cnt_err=%d pos_new=%0.2f pos_last=%0.2f dist_sum=%0.2f ts_new=%lu ts_last=%lu dt_sec=%0.2f vel_new=%0.2f vel_last=%0.2f",
				vel_diff > 300 ? "Vel|" : "", dt_sec == 0 ? "DT|" : "", p_str_list_objID[this->objID], this->cnt_err, pos_new, this->posArr[this->nSamp - 2], dist_sum, this->t_tsArr[this->nSamp - 1], this->t_tsArr[this->nSamp - 2], dt_sec, vel, this->velLast);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
		}
	}
}

double POSTRACK::GetPos()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Reset flag
	this->isNew = false;

	// Return newest pos value
	return this->posCum;
}

double POSTRACK::GetVel()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Return newest vel value
	return this->velNow;
}

void POSTRACK::SwapPos(double set_pos, uint32_t ts)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	double update_pos = 0;
	uint32_t vt_ts = 0;

	// Make sure pos val range [0, 140*PI]
	if (set_pos < 0) {
		update_pos = set_pos + (140 * PI);
	}
	else if (set_pos > (140 * PI)) {
		update_pos = set_pos - (140 * PI);
	}
	else {
		update_pos = set_pos;
	}

	// Compute ts
	vt_ts = this->t_tsNow + (ts - this->t_update);

	// Update pos
	UpdatePos(update_pos, vt_ts, false);

}

void POSTRACK::PosReset(bool do_lap_reset)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Reset sample count and new flag
	this->cnt_samp = 0;
	this->isNew = false;

	// Reset lap number
	if (do_lap_reset) {
		this->nLaps = 0;
	}
}

#pragma endregion 

#pragma region ----------CLASS: PID----------

PID::PID(const float kC, const float pC, const byte n_calRuns) :
	cal_cntPcArr(n_calRuns, __LINE__),
	cal_PcArr(n_calRuns, __LINE__),
	cal_errArr(n_calRuns, __LINE__)
{
	// Proportional constant
	this->kP = 0.6 * kC;

	// Integral constant
	this->kI = 2 * kP / pC;

	// Derivative constant
	this->kD = kP*pC / 8;

}

double PID::PidUpdate()
{

	// Wait till ekf ready
	if (!FC.is_EKFReady) {
		return -1;
	}

	// Compute error 
	error = kal.RatPos - (kal.RobPos + setPoint);

	// Check if motor is open
	PidCheckMotorControl();

	// Bail if in "MANUAL" or "HOLDING" mode
	if (pidMode == MANUAL || pidMode == HOLDING) {

		return -1;
	}

	// Check if rat stopped behind setpoint
	if (kal.RatVel < 1 && error < -15 && !isHolding4cross) {
		// halt running
		return runSpeed = 0;
	}

	// Check for setpoint crossing
	if (isHolding4cross && error > 0)
	{
		// Reset flag
		isHolding4cross = false;
	}

	// Bail if waiting on cross or ekf
	if (
		!is_ekfNew ||
		isHolding4cross
		) {
		return -1;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log first run
	if (isFirstRun) {
		Debug.DB_Pid(__FUNCTION__, __LINE__, "First PID Run");
		isFirstRun = false;
	}

	// Re-compute error 
	error = kal.RatPos - (kal.RobPos + setPoint);

	// Set integral to zero if rat just crossed setpoint
	if ((abs(error) + abs(errorLast)) > abs(error + errorLast)) {
		integral = 0;
	}

	// Update normally
	else {
		integral = integral + error;
	}

	// Compute new derivative
	derivative = error - errorLast;

	// Compute new terms
	p_term = kP*error;
	i_term = kI*dt_update*integral;
	d_term = kD / dt_update*derivative;
	//if (error < 0) d_term = d_term * 2.0f;

	// Compute updated vel
	velUpdate = p_term + i_term + d_term;

	// Get new run speed
	runSpeed = kal.RobVel + velUpdate;

	// Keep speed in range [0, speedMax]
	if (runSpeed > speedMax) {
		runSpeed = speedMax;
	}
	else if (runSpeed < 0) {
		runSpeed = 0;
	}

	// Store error
	errorLast = error;

	// Set flags
	isPidUpdated = true;
	is_ekfNew = false;

	// Return new run speed
	return runSpeed;

}

void PID::PidRun()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Take motor control
	if (SetMotorControl(MC_CON::ID::PID, MC_CALL::ID::PID)) {

		// Reset
		PidReset();

		// Set mode to "AUTOMATIC"
		pidMode = AUTOMATIC;

		// Tell ard pid is running
		QueuePacket(&r2a, 'p', 1);

		// Log event
		Debug.DB_Pid(__FUNCTION__, __LINE__, "Run PID");

	}

	// Log error
	else {
		Debug.DB_Error(__FUNCTION__, __LINE__, "PID FAILED TO TAKE MOTOR CONTROL");
	}

}

void PID::PidStop(PIDMODE set_mode)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Pid(__FUNCTION__, __LINE__, "Stop PID");

	if (motorControlNow == MC_CON::ID::PID)
	{
		// Stop movement
		RunMotor('f', 0, MC_CALL::ID::PID);

		// Set run speed
		runSpeed = 0;

		// Give over control
		SetMotorControl(MC_CON::ID::OPEN, MC_CALL::ID::PID);
	}

	// Tell ard pid is stopped if not rewarding
	if (!Reward.isRewarding) {
		QueuePacket(&r2a, 'p', 0);
	}

	// Set mode 
	pidMode = set_mode;
}

void PID::PidHold()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Pid(__FUNCTION__, __LINE__, "Hold PID");

	// Call stop and set mode to "HOLDING"
	PidStop(HOLDING);
}

void PID::PidReset()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	integral = 0;
	t_updateLast = millis();
	isHolding4cross = true;
}

void PID::PidCheckMotorControl()
{

	// Run pid
	if ((motorControlNow == MC_CON::ID::PID || motorControlNow == MC_CON::ID::OPEN) &&
		pidMode == HOLDING) {

		// Run pid
		PidRun();
	}

	// Put pid on hold
	else if ((motorControlNow != MC_CON::ID::PID && motorControlNow != MC_CON::ID::OPEN) &&
		pidMode == AUTOMATIC) {

		// Hold pid
		PidHold();
	}
}

void PID::PidCheckEKF(uint32_t ts)
{
	// Bail if not checking
	if (FC.is_EKFReady) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	if ((ts - t_ekfReady) > dt_ekfSettle &&
		kal.cnt_ekf > 100)
	{
		// Log event 
		Debug.DB_Pid(__FUNCTION__, __LINE__, "EKF Ready for PID");

		// Set flag
		FC.is_EKFReady = true;
}
}

void PID::PidResetEKF()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Pid(__FUNCTION__, __LINE__, "Reset EKF");

	// Set flag and time
	FC.is_EKFReady = false;
	t_ekfReady = millis();
}

void PID::PidSetUpdateTime(uint32_t ts)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	is_ekfNew = true;
	if (isPidUpdated)
	{
		dt_update = (double)(ts - t_updateLast) / 1000;
		isPidUpdated = false;
		t_updateLast = ts;
}
}

double PID::PidCalibration()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	/*
	Calibration based on the Ziegler–Nichols method:
	http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
	Equations:
	kP = 0.6*Kc
	Kd = 2*Kp * dT/Pc
	Ki = Kp*Pc / (8*dT)
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	float pc_sum = 0;

	// Bail if finished
	if (cal_isCalFinished) {
		return -1;
	}

	// End of calibration 
	if (
		cal_cntPcArr[3] == cal_nMeasPerSteps &&
		cal_PcArr[3] > 0
		) {

		// Log
		Debug.DB_General(__FUNCTION__, __LINE__, "Finished PID Calibration");

		// Compute overal average
		for (int i = 0; i < 4; i++)
		{
			pc_sum += cal_PcArr[i];
		}
		cal_PcAll = pc_sum / 4;

		// Set flags
		cal_isPidUpdated = true;
		cal_isCalFinished = true;

		// Stop run
		return 0;
	}

	// Bail if pos data not ready
	if (!is_ekfNew) {
		return -1;
	}

	// Check if ready to get new pc val
	if (millis() < t_updateLast + cal_dt_min) {
		return -1;
	}

	// Setup stuff
	if (cal_ratPos == 0) {
		cal_ratPos = kal.RobPos + setPoint;
		t_updateLast = millis();
		return -1;
	}

	// Check if speed should be incrimented
	if (cal_cntPcArr[cal_stepNow] == cal_nMeasPerSteps) {

		// Store values
		cal_PcArr[cal_stepNow] = cal_PcAvg;
		cal_PcSum = 0;
		cal_PcCnt = 0;
		cal_errArr[cal_stepNow] = cal_errAvg;
		cal_errSum = 0;
		cal_errCnt = 0;

		// Incriment step or bail if max reached
		if (cal_stepNow < 3) {
			cal_stepNow++;
		}
		else {
			return -1;
		}

		// Log
		Debug.sprintf_safe(buffLrg, buff_lrg, "Set Speed to %0.2fcm/sec", cal_speedSteps[cal_stepNow]);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Get update dt
	cal_dt_update = (float)(millis() - t_updateLast) / 1000.0f;
	t_updateLast = millis();

	// Compute Pid
	cal_ratVel = cal_speedSteps[cal_stepNow];
	cal_ratPos += cal_ratVel * (cal_dt_update / 1);

	// Compute error 
	error = cal_ratPos - (kal.RobPos + setPoint);
	cal_errNow = error;

	// Compute new terms
	p_term = kC*error;

	// Get new run speed
	runSpeed = kal.RobVel + p_term;

	// Keep speed in range [0, speedMax]
	if (runSpeed > speedMax) {
		runSpeed = speedMax;
	}
	else if (runSpeed < 0) {
		runSpeed = 0;
	}

	// Catch occilation edge
	if (cal_errLast > 0 && error < 0) {

		// Update values
		cal_t_PcLast = cal_t_PcNow;
		cal_t_PcNow = millis();

		// Skip first period
		if (cal_t_PcLast > 0) {
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
	isPidUpdated = true;
	is_ekfNew = false;

	return runSpeed;

}

#pragma endregion 

#pragma region ----------CLASS: BULLDOZE----------

BULLDOZE::BULLDOZE() {}

void BULLDOZE::UpdateBull()
{
	// Check who has motor control
	BullCheckMotorControl();

	// Bail if "OFF" or "HOLDING"
	if (bullState == OFF ||
		bullState == HOLDING) {
		return;
	}

	// Bail if not ready
	if (
		!(FC.is_EKFReady &&
			millis() > t_updateNext)) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Update cumulative rat pos
	posCum = kal.RatPos;
	guardPos = kal.RobPos + guardDist;

	// Get distance traveled
	distMoved = posCum - posCheck;

	// Check for movement
	isMoved = distMoved >= moveMin ? true : false;

	// Check if rat passed reset
	double error = kal.RatPos - (kal.RobPos + Pid.setPoint);
	isPassedReset = error > 1 ? true : false;

	// Check time
	isTimeUp = millis() > t_bullNext ? true : false;

	// Check if rat has not moved in time
	if (!isMoved) {

		// Bulldoze him!
		if (isTimeUp &&
			bullMode == INACTIVE) {
			BullRun();
		}
	}

	// Has moved minimal distance
	else {
		// Reset check pos
		posCheck = posCum;

		// Reset bull next
		t_bullNext = millis() + bullDelay;

		// Stop bulldoze if rat ahead of set point and not 0 delay
		if (isPassedReset &&
			bullMode == ACTIVE &&
			bullDelay != 0) {

			BullStop();
		}
	}

	// Set next update time
	t_updateNext = millis() + dt_update;

}

void BULLDOZE::BullReinitialize(float bull_delay, float bull_speed)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Log event
	Debug.sprintf_safe(buffLrg, buff_lrg, "Reinitialize Bull with bull_delay=%0.2f spd=%0.2f", bull_delay, bull_speed);
	Debug.DB_Bull(__FUNCTION__, __LINE__, buff_lrg);

	// Update vars
	bullSpeed = bull_speed;
	bullDelay = (int)bull_delay * 1000;
	t_bullNext = millis() + bullDelay;
	posCheck = kal.RatPos;
}

void BULLDOZE::BullRun()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Format string with stop time
	Debug.sprintf_safe(buffLrg, buff_lrg, "Running Bull: Rat has moved %0.2fcm in %lums",
		distMoved, t_bullNext - bullDelay);

	// Log event
	Debug.DB_Bull(__FUNCTION__, __LINE__, buff_lrg);

	// Take motor control
	if (!SetMotorControl(MC_CON::ID::BULL, MC_CALL::ID::BULL)) {

		// Log error
		Debug.DB_Error(__FUNCTION__, __LINE__, "BULL FAILED TO TAKE MOTOR CONTROL");

		// Bail
		return;
	}

	// Start bulldozer
	RunMotor('f', bullSpeed, MC_CALL::ID::BULL);

	// Tell ard bull is running
	QueuePacket(&r2a, 'b', 1);

	// Set mode to "ACTIVE"
	bullMode = ACTIVE;
}

void BULLDOZE::BullStop(BULLMODE set_mode)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Bull(__FUNCTION__, __LINE__, "Stop Bull");

	// Stop movement
	RunMotor('f', 0, MC_CALL::ID::BULL);

	// Give over control
	if (motorControlNow == MC_CON::ID::BULL) {
		SetMotorControl(MC_CON::ID::OPEN, MC_CALL::ID::BULL);
	}

	// Reset bull next
	t_bullNext = millis() + bullDelay;

	// Tell ard bull is stopped
	QueuePacket(&r2a, 'b', 0);

	// Set mode
	bullMode = set_mode;
}

void BULLDOZE::BullOn()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Bull(__FUNCTION__, __LINE__, "Turn On Bull");

	// Set state to "ON"
	bullState = ON;

	// Reset 
	BullReset();
}

void BULLDOZE::BullOff()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Bull(__FUNCTION__, __LINE__, "Turn Off Bull");

	// Set state to "OFF"
	bullState = OFF;

	// Stop bulldozer if running
	if (bullMode == ACTIVE) {

		// Stop bull
		BullStop();
	}
}

void BULLDOZE::BullHold()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Bull(__FUNCTION__, __LINE__, "Hold Bull");

	// Set state to "HOLDING"
	bullState = HOLDING;

	// Stop running
	if (bullMode == ACTIVE) {

		// Run stop bull but keep mode active for later
		BullStop(ACTIVE);
	}
}

void BULLDOZE::BullResume()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log event
	Debug.DB_Bull(__FUNCTION__, __LINE__, "Resume and Reset");

	// Set state back to "ON"
	bullState = ON;

	// Reset 
	BullReset();
}

void BULLDOZE::BullReset()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Reset mode to "INACTIVE"
	bullMode = INACTIVE;

	// Reset bull next
	t_bullNext = millis() + bullDelay;

	// Reset check pos
	posCheck = kal.RatPos;
}

void BULLDOZE::BullCheckMotorControl()
{

	// Resume bull
	if ((motorControlNow == MC_CON::ID::BULL || motorControlNow == MC_CON::ID::PID || motorControlNow == MC_CON::ID::OPEN) &&
		bullState == HOLDING) {

		// Turn bull on
		BullResume();
	}

	// Put bull on hold
	else if ((motorControlNow != MC_CON::ID::BULL && motorControlNow != MC_CON::ID::PID && motorControlNow != MC_CON::ID::OPEN) &&
		bullState == ON) {

		// Turn bull off
		BullHold();
	}
}

#pragma endregion 

#pragma region ----------CLASS: MOVETO----------

MOVETO::MOVETO() {}

void MOVETO::ProcMoveCmd(byte cmd_cnt, float cmd_targ)
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Update count
	cnt_move++;

	// Update move event
	moveEv = cmd_cnt == 0 ? LAST : cmd_cnt == 1 ? FIRST : OTHER;

	// Align target pos to feeder
	targPosAbs = cmd_targ - feedDist;
	targPosAbs = targPosAbs < 0 ? targPosAbs + (140 * PI) : targPosAbs;

	// Format string
	if (moveEv == OTHER) {
		// Format as number
		Debug.sprintf_safe(buffMed, str_med_move, "MOVE [%d]", cmd_cnt);
	}
	else {
		// Format as "FIRST" or "LAST"
		Debug.sprintf_safe(buffMed, str_med_move, "MOVE [%s]", p_str_list_moveEv[moveEv]);
	}

	// Set abort timeout
	t_moveTimeout = millis() + dt_moveTimeout;

	// Log setup
	Debug.sprintf_safe(buffLrg, buff_lrg, "SETUP: %s: targ=%0.2fcm", str_med_move, targPosAbs);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

}

bool MOVETO::RunMove()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool move_done = false;
	int dt_ekf = 0;

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Store last ekf update
	dt_ekf = millis() - kal.t_last;

	// Check if time out reached
	if (millis() > t_moveTimeout) {

		// Check if EKF not updating
		if (dt_ekf >= dt_moveTimeout / 2) {

			// Log error
			Debug.sprintf_safe(buffLrg, buff_lrg, "%s: ABORTING: EKF Hanging: dt_ekf=%dms", str_med_move, dt_ekf);
			Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Log error
		Debug.sprintf_safe(buffLrg, buff_lrg, "%s: TIMEDOUT AFTER %dms: dt_ekf=%dms", str_med_move, dt_moveTimeout, dt_ekf);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

		// Set abort flag
		do_AbortMove = true;
	}

	// Compute move target
	if (!isTargSet && !do_AbortMove) {

		// If succesfull
		if (SetMoveTarg()) {

			// Start running
			if (SetMotorControl(MC_CON::ID::MOVETO, MC_CALL::ID::MOVETO)) {

				if (RunMotor(moveDir, moveToSpeedMax, MC_CALL::ID::MOVETO)
					) {

					// Print message
					Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING: %s: Begin: speed=0.2fcm/sec targ_dist=%0.2fcm move_dir=\'%c\'...",
						str_med_move, moveToSpeedMax, targDist, moveDir);
					Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

					// Store new speed
					lastSpeed = moveToSpeedMax;

					// Set flag
					isMoveStarted = true;
				}

				// Failed to run motor
				else {

					// Log error
					Debug.sprintf_safe(buffLrg, buff_lrg, "%s: FAILED TO RUN MOTOR", str_med_move);
					Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

					// Reset control
					if (!SetMotorControl(MC_CON::ID::HOLD, MC_CALL::ID::MOVETO)) {

						// Log error
						Debug.sprintf_safe(buffLrg, buff_lrg, "%s: FAILED TO SET MOTOR CONTROL TO \"NONE\" AFTER ABORTING", str_med_move);
						Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
					}

					// Set flags
					do_AbortMove = true;
				}
			}

			// Failed to take motor control
			else {
				// Set flags
				do_AbortMove = true;

				// Log error
				Debug.sprintf_safe(buffLrg, buff_lrg, "%s: FAILED TO TAKE MOTOR CONTROL TO \"NONE\" AFTER ABORTING", str_med_move);
				Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
			}

		}

	}

	// Check if robot is ready to be stopped
	else if (!isTargReached && !do_AbortMove && !FC.do_Halt) {

		// Do deceleration
		double new_speed = DecelToTarg(moveToDecelDist, moveToSpeedMin);

		// Change speed if > 0
		if (new_speed > 0 && new_speed != runSpeedNow) {

			// Run motor
			RunMotor(moveDir, new_speed, MC_CALL::ID::MOVETO);

			// Store speed
			lastSpeed = new_speed;

		}

	}

	// Check if target reached or move aborted
	else {

		// Hard stop
		HardStop(__FUNCTION__, __LINE__);

		// Reset control to "HOLD"
		SetMotorControl(MC_CON::ID::HOLD, MC_CALL::ID::MOVETO);

		// Log warning
		if (FC.do_Halt) {
			// Print abort message
			Debug.sprintf_safe(buffLrg, buff_lrg, "ABORTED: %s: Robot Halted", str_med_move);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Log failure
		else if (do_AbortMove) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED: %s: targ_set=%d ekf_ready=%d dt_ekf=%dms move_started=%d targ_dist=%0.2fcm dist_error=%0.2fcm move_dir=\'%c\'",
				str_med_move, isTargSet, FC.is_EKFReady, dt_ekf, isMoveStarted, targDist, GetMoveError(), moveDir);
			Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Log final move status
		else if (isTargReached) {

			// Print success message
			Debug.sprintf_safe(buffLrg, buff_lrg, "SUCCEEDED: %s: targ_dist=%0.2fcm dist_error=%0.2fcm move_dir=\'%c\'",
				str_med_move, targDist, GetMoveError(), moveDir);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Send confirmation for first and last move
			if (moveEv == FIRST || moveEv == LAST) {

				// Log
				Debug.sprintf_safe(buffLrg, buff_lrg, "%s: Sending 'M' Done Confirmation", str_med_move);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				// Send done confirmation
				QueuePacket(&r2c, 'M', 0, 0, 0, c2r.packArr[ID_Ind<R4_COM<USARTClass>>('M', &c2r)], true, false, true);

			}

		}

		// Set done flag
		move_done = true;

		// Reset Move
		MoveToReset();

	}

	// Return flag
	return move_done;

}

bool MOVETO::SetMoveTarg()
{

	// Run only if targ not set
	if (isTargSet) {
		return isTargSet;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	double pos_abs;
	double move_diff = 0;
	int lap_cm = 0;
	int pos = 0;

	// Bail if ekf pos data not ready
	if (!FC.is_EKFReady) {
		return isTargSet;
	}

	// Current absolute pos on track
	lap_cm = (int)(140 * PI * 100);
	pos = (int)(kal.RobPos * 100);
	pos_abs = (double)(pos % lap_cm) / 100;
	pos_abs = pos_abs < 0 ? pos_abs + (140 * PI) : pos_abs;

	// Diff and absolute distance
	move_diff = targPosAbs - pos_abs;

	// Get minimum distance to target
	targDist = min((140 * PI) - abs(move_diff), abs(move_diff));

	// Set to negative for reverse move
	if ((move_diff > 0 && abs(move_diff) == targDist) ||
		(move_diff < 0 && abs(move_diff) != targDist)) {
		moveDir = 'f';
	}
	else {
		moveDir = 'r';
	}

	// Set vars for later
	t_updateNext = millis();
	baseSpeed = 0;

	// Store starting pos
	startPosCum = kal.RobPos;

	// Set flag true
	isTargSet = true;

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "%s: Set Move Target: start_cum=%0.2fcm start_abs=%0.2fcm targ=%0.2fcm dist_move=%0.2fcm move_dir=\'%c\'",
		str_med_move, pos_abs, startPosCum, targPosAbs, targDist, moveDir);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Retern flag
	return isTargSet;
}

double MOVETO::DecelToTarg(double dist_decel, double speed_min)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	double new_speed = 0;
	int dt_ekf = 0;

	// Run if targ not reached
	if (isTargReached) {
		return 0;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Compute remaining distance
	distLeft = targDist - abs(kal.RobPos - startPosCum);

	// Check if motor stopped
	if (distLeft > 1 && runSpeedNow == 0) {

		// Set to last speed or min speed 
		new_speed = lastSpeed > 0 ? lastSpeed : speed_min;

		// Bail 
		return new_speed;
	}

	// Check if rob is dec_pos cm from target
	if (distLeft <= dist_decel) {

		// Get base speed to decelerate from
		if (baseSpeed == 0 && kal.RobVel != 0) {
			baseSpeed = abs(kal.RobVel);

			// Log decel distance
			Debug.sprintf_safe(buffLrg, buff_lrg, "%s: Reached %0.2fcm From Target", str_med_move, distLeft);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Update decel speed
		else if (millis() > t_updateNext) {
			// Compute new speed so range constrained to [min_speed, base_speed]
			new_speed = (((baseSpeed - speed_min) * distLeft) / dist_decel) + speed_min;

			// Get next update time
			t_updateNext = millis() + dt_update;
		}

	}

	// Target reached
	if (distLeft < 1)
	{
		// Log
		Debug.sprintf_safe(buffLrg, buff_lrg, "FINISHED: %s: start_cum=%0.2fcm now_cum=%0.2f targ=%0.2fcm dist_move=%0.2fcm dist_left=%0.2fcm",
			str_med_move, startPosCum, kal.RobPos, targPosAbs, targDist, distLeft);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Set flag true
		isTargReached = true;

		// Stop movement
		new_speed = 0;
	}

	// Return new speed
	return new_speed;
}

double MOVETO::GetMoveError()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	double pos_abs;
	int diam = 0;
	int pos = 0;

	// Current relative pos on track
	diam = (int)(140 * PI * 100);
	pos = (int)(kal.RobPos * 100);
	pos_abs = (double)(pos % diam) / 100;

	// Target error
	return targPosAbs - pos_abs;
}

void MOVETO::MoveToReset()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	isTargSet = false;
	isMoveStarted = false;
	isTargReached = false;
	do_AbortMove = false;
	t_moveTimeout = 0;
	lastSpeed = 0;
}

#pragma endregion 

#pragma region ----------CLASS: REWARD----------

REWARD::REWARD() :
	zoneRewDurs(zoneLng, __LINE__, _zoneRewDurs),
	zoneLocs(zoneLng, __LINE__, _zoneLocs),
	zoneBoundCumMin(zoneLng, __LINE__),
	zoneBoundCumMax(zoneLng, __LINE__),
	zoneOccTim(zoneLng, __LINE__),
	zoneOccCnt(zoneLng, __LINE__),
	zoneBoundCumRewarded(2, __LINE__)
{
	this->rewDuration = durationDefault;
}

void REWARD::ProcRewCmd(byte cmd_type, float cmd_goal, int cmd_zone_delay)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// NOTE: arg2 = reward delay or zone ind or reward duration

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	int cmd_zone_ind = -1;
	int cmd_delay = -1;

	// Store mode
	rewMode =
		cmd_type == 0 ? REWMODE::BUTTON :
		cmd_type == 1 ? REWMODE::NOW :
		cmd_type == 2 ? REWMODE::CUE :
		REWMODE::FREE;

	// Update counts
	if (rewMode != BUTTON) {
		cnt_cmd++;
	}
	cnt_rew++;

	// Format string
	Debug.sprintf_safe(buffMed, str_med_rew, "REWARD \"%s\" [%d/%d]",
		p_str_list_rewMode[rewMode], cnt_rew, cnt_cmd);

	// Handle zone/delay arg
	if (rewMode == NOW || rewMode == CUE) {

		// Set to zero based index
		cmd_zone_ind = cmd_zone_delay - 1;
	}
	else if (rewMode == FREE) {
		cmd_delay = cmd_zone_delay;
	}

	// Setup "BUTTON" reward
	if (rewMode == BUTTON) {

		// Set duration to default
		SetZoneDur();
	}

	// Setup "NOW" reward
	else if (rewMode == NOW) {

		// Set duration
		SetZoneDur(cmd_zone_ind);
	}

	// Setup "CUE" reward
	else if (rewMode == CUE) {

		// Include specified zone
		zoneMin = cmd_zone_ind;
		zoneMax = cmd_zone_ind;

		// Set zone bounds
		SetZoneBounds(cmd_goal);

		// Set delay to zero
		rewDelay = 0;
	}

	// Setup "FREE" reward
	else if (rewMode == FREE) {

		// Include all zones
		zoneMin = 0;
		zoneMax = zoneLng - 1;

		// Set zone bounds
		SetZoneBounds(cmd_goal);

		// Store reward delay time in ms
		rewDelay = cmd_delay * 1000;
	}

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "SETUP: %s: cmd_type=%d cmd_goal=%0.2f cmd_zone_delay=%d",
		str_med_rew, cmd_type, cmd_goal, cmd_zone_delay);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
}

bool REWARD::RunReward()
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool reward_done = false;

	// Bail if rewarding
	if (isRewarding) {
		return reward_done;
	}

	// Zone not triggered yet
	if (!isZoneTriggered) {

		// Check each zone
		if (CheckZoneBounds()) {

			// Start reward
			StartRew();

			// Reset flag
			FC.do_RunRew = false;

			// Print message
			Debug.sprintf_safe(buffLrg, buff_lrg, "%s: REWARDED ZONE: occ=%dms zone=%d from=%0.2fcm to=%0.2fcm",
				str_med_rew, occRewarded, zoneRewarded, zoneBoundCumRewarded[0], zoneBoundCumRewarded[1]);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Set done flag
			reward_done = true;

		}
	}

	// Check if rat passed all bounds
	if (isAllZonePassed &&
		!isZoneTriggered) {

		// Print reward missed
		Debug.sprintf_safe(buffLrg, buff_lrg, "MISSED: %s: rat=%0.2fcm bound_max=%0.2fcm",
			str_med_rew, kal.RatPos, zoneBoundCumMin[zoneMax]);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Send missed reward msg
		QueuePacket(&r2c, 'Z', cnt_rew, 0, zoneInd + 1, 0, true);

		// Decriment reward count
		cnt_rew--;

		// Reset flags
		RewardReset();

		// Set done flag
		reward_done = true;

	}

	// Return flag
	return reward_done;

}

void REWARD::StartRew()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Set motor hold time
	if (!FC.is_ForageTask && rewMode != BUTTON) {
		BlockMotor(dt_rewBlock);
	}

	// Hard stop
	HardStop(__FUNCTION__, __LINE__);

	// Send ard packet imediately
	if (FC.is_SesStarted) {
		QueuePacket(&r2a, 'r', rewDuration, cnt_rew);
		SendPacket(&r2a);
	}

	// Turn on reward LED
	analogWrite(pin.LED_REW_C, rewLEDduty[1]);
	analogWrite(pin.LED_REW_R, rewLEDduty[1]);

	// Open solenoid
	digitalWrite(pin.REL_FOOD, HIGH);

	// Compute reward end time
	t_rewStr = millis();
	t_rewEnd = t_rewStr + rewDuration;
	t_closeSol = t_rewStr + (int)((float)rewDuration*solOpenScale);

	// Extend feeder arm
	if (!FC.is_ForageTask) {
		ExtendFeedArm(ezRewExtStps);
	}

	// Compute retract arm time
	if (rewMode != BUTTON &&
		!FC.is_ForageTask) {

		// Compute time and set flag
		t_retractArm = t_rewStr + dt_rewBlock;
		do_TimedRetract = true;
	}
	else {
		do_TimedRetract = false;
	}

	// Log 
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING: %s: dt_sol=%dms dt_rew=%dms dt_retract=%d...",
		str_med_rew, t_closeSol - t_rewStr, rewDuration, do_TimedRetract ? t_retractArm - t_rewStr : 0);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, t_rewStr);


	// Print to LCD for manual rewards
	if (rewMode == BUTTON) {
		Debug.PrintLCD(true, "REWARDING...");
	}

	// Set flags
	isRewarding = true;

}

bool REWARD::CheckEnd()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Bail if not rewarding
	if (!isRewarding) {
		return false;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Check if not time to close solonoid
	if (millis() < t_closeSol) {
		return false;
	}

	// Close solenoid
	if (digitalRead(pin.REL_FOOD) == HIGH) {
		digitalWrite(pin.REL_FOOD, LOW);

		// Store actual time
		t_closeSol = millis();

		// Log
		Debug.sprintf_safe(buffLrg, buff_lrg, "%s: CLOSED SOLONOID: dt_sol=%dms",
			str_med_rew, t_closeSol - t_rewStr);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, t_rewEnd);
	}

	// Bail if not time to end reward
	if (millis() < t_rewEnd) {
		return false;
	}

	// Turn off reward LED
	analogWrite(pin.LED_REW_C, rewLEDduty[0]);
	analogWrite(pin.LED_REW_R, rewLEDduty[0]);

	// Store actual time
	t_rewEnd = millis();

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "FINISHED: %s: dt_sol=%dms dt_rew=%dms dt_retract=%d",
		str_med_rew, t_closeSol - t_rewStr, t_rewEnd - t_rewStr, do_TimedRetract ? t_retractArm - t_rewStr : 0);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, t_rewEnd);

	// Clear LCD
	if (rewMode == BUTTON) {
		Debug.ClearLCD();
	}

	// Reset flags etc
	RewardReset(true);

	// Tell CS what zone was rewarded and get confirmation
	if (rewMode == REWARD::REWMODE::FREE ||
		rewMode == REWARD::REWMODE::CUE) {

		QueuePacket(&r2c, 'Z', cnt_rew, 1, zoneInd + 1, 0, true);
	}

	// Return end reward status
	return true;

	}

void REWARD::SetZoneDur(int zone_ind)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Set zone ind
	if (zone_ind != -1) {
		zoneInd = zone_ind;
	}

	// Find default ind
	else {
		for (int i = 0; i < zoneLng; i++) {
			zoneInd = zoneRewDurs[i] == durationDefault ? i : zoneInd;
		}
	}

	// Set duration
	rewDuration = zoneRewDurs[zoneInd];

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "%s: Set Reward Duration: zone_ind=%d duration=%d",
		str_med_rew, zoneInd, rewDuration);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
}

void REWARD::SetZoneBounds(float cmd_goal)
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	int diam = 0;
	int pos_int = 0;
	double pos_cum = 0;
	double dist_center_cm = 0;
	double dist_start_cm = 0;
	double dist_end_cm = 0;

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Compute laps
	diam = (int)(140 * PI * 100);
	pos_int = (int)(kal.RatPos * 100);
	lapN = round(kal.RatPos / (140 * PI) - (float)(pos_int % diam) / diam);
	// Check if rat 'ahead' of rew pos
	pos_cum = (double)(pos_int % diam) / 100;
	// Add lap
	lapN = pos_cum > cmd_goal ? lapN + 1 : lapN;

	// Compute reward center
	goalPosCum = cmd_goal + lapN*(140 * PI);

	// Compute bounds for each zone
	for (int i = zoneMin; i <= zoneMax; i++)
	{
		// Get zone width with overlap for center bins
		int zone_bnd_start = zoneMin == zoneMax || i == zoneMin ?
			rewZoneWdith / 2 : rewZoneWdith;
		int zone_bnd_end = zoneMin == zoneMax || i == zoneMax ?
			rewZoneWdith / 2 : rewZoneWdith;

		// Compute zone bounds
		dist_center_cm = -1 * zoneLocs[i] * ((140 * PI) / 360);
		dist_start_cm = dist_center_cm - (zone_bnd_start * ((140 * PI) / 360));
		dist_end_cm = dist_center_cm + (zone_bnd_end * ((140 * PI) / 360));

		// Store in array
		zoneBoundCumMin[i] = goalPosCum + dist_start_cm;
		zoneBoundCumMax[i] = goalPosCum + dist_end_cm;
	}

	// Print message
	Debug.sprintf_safe(buffLrg, buff_lrg, "%s: Set Goal Zone: goal_pos_cum=%0.2fcm from=%0.2fcm to=%0.2fcm",
		str_med_rew, goalPosCum, zoneBoundCumMin[zoneMin], zoneBoundCumMax[zoneMax]);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

}

bool REWARD::CheckZoneBounds()
{

	// Run only if reward not already triggered
	if (isZoneTriggered) {
		return isZoneTriggered;
	}

	// Bail if pos data not new
	if (!is_ekfNew) {
		return isZoneTriggered;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Reset flag
	is_ekfNew = false;

	// Check if all bounds passed
	if (kal.RatPos > zoneBoundCumMax[zoneMax] + 5) {
		isAllZonePassed = true;
		return isZoneTriggered;
	}

	// Bail if first bound not reached
	if (kal.RatPos < zoneBoundCumMin[zoneMin]) {
		return isZoneTriggered;
	}

	// Check if rat in any bounds
	for (int i = zoneMin; i <= zoneMax; i++)
	{
		if (
			kal.RatPos > zoneBoundCumMin[i] &&
			kal.RatPos < zoneBoundCumMax[i]
			) {

			// Update timers
			t_lastZoneCheck = t_lastZoneCheck == 0 ? millis() : t_lastZoneCheck;
			t_nowZoneCheck = millis();

			// Store occupancy time
			zoneOccTim[i] += t_nowZoneCheck - t_lastZoneCheck;
			zoneOccCnt[i]++;
			t_lastZoneCheck = t_nowZoneCheck;

			// Check if occ thresh passed
			if (zoneOccTim[i] >= rewDelay)
			{

				// REWARD at this pos
				SetZoneDur(i);

				// Store reward info for debugging
				zoneRewarded = zoneLocs[i] * -1;
				zoneBoundCumRewarded[0] = zoneBoundCumMin[i];
				zoneBoundCumRewarded[1] = zoneBoundCumMax[i];
				occRewarded = zoneOccTim[i];

				// Set flag
				isZoneTriggered = true;
			}
		}
	}

	return isZoneTriggered;
}

void REWARD::ExtendFeedArm(byte ext_steps)
{

	// Notes
	/*
	Step mode:
	MS1	MS2	MS3	Microstep Resolution	Excitation Mode
	L	L	L	Full Step	2 Phase
	H	L	L	Half Step	1 - 2 Phase
	L	H	L	Quarter Step	W1 - 2 Phase
	H	H	L	Eigth Step	2W1 - 2 Phase
	H	H	H	Sixteenth Step	4W1 - 2 Phase
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Bail if arm already extended
	if (isArmExtended) {

		// Log warning
		Debug.DB_Warning(__FUNCTION__, __LINE__, "ABORTED: Arm Already Extended");
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "Set Extend Feed Arm: steps=%d", ext_steps);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Block handler
	v_doStepTimer = false;
	delayMicroseconds(ezStepPeriod + 100);

	// Set targ and flag
	v_stepTarg = ext_steps;
	do_RetractArm = false;
	do_ExtendArm = true;
	v_isArmMoveDone = false;

	// Wake motor
	digitalWrite(pin.ED_SLP, HIGH);

	// Make sure step low
	digitalWrite(pin.ED_STP, LOW);

	// Set to quarter step
	SetMicroSteps(ezExtMicroStep);

	// Set direction to extend
	digitalWrite(pin.ED_DIR, ezDirExtState == 1 ? HIGH : LOW);
	delayMicroseconds(100);
	v_stepDir = 'e';

	// Store start time
	t_moveArmStr = millis();

	// Set intterupt flag
	v_doStepTimer = true;
	delayMicroseconds(100);

	// Start timer
	FeederArmTimer.start();
	delayMicroseconds(100);
}

void REWARD::RetractFeedArm()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Step mode:
	/*
	MS1	MS2	MS3	Microstep Resolution	Excitation Mode
	L	L	L	Full Step	2 Phase
	H	L	L	Half Step	1 - 2 Phase
	L	H	L	Quarter Step	W1 - 2 Phase
	H	H	L	Eigth Step	2W1 - 2 Phase
	H	H	H	Sixteenth Step	4W1 - 2 Phase
	*/

	// Log
	Debug.DB_General(__FUNCTION__, __LINE__, "Set Retract Feed Arm");

	// Block handler
	v_doStepTimer = false;
	delayMicroseconds(ezStepPeriod + 100);

	// Set targ and flag
	v_stepTarg = 0;
	do_ExtendArm = false;
	do_RetractArm = true;
	v_isArmMoveDone = false;

	// Wake motor
	digitalWrite(pin.ED_SLP, HIGH);

	// Make sure step low
	digitalWrite(pin.ED_STP, LOW);

	// Set to quarter step
	SetMicroSteps(ezRetMicroStep);

	// Set direction to retract
	digitalWrite(pin.ED_DIR, ezDirRetState == 1 ? HIGH : LOW);
	delayMicroseconds(100);
	v_stepDir = 'r';

	// Store start time
	t_moveArmStr = millis();

	// Set intterupt flag
	v_doStepTimer = true;
	delayMicroseconds(100);

	// Start timer
	FeederArmTimer.start();
	delayMicroseconds(100);

	// Unset timed retract flag
	do_TimedRetract = false;

}

void REWARD::SetMicroSteps(MICROSTEP microstep)
{
	// INPUT ARGS: 
	//	FULL, HALF, QUARTER, EIGHTH, SIXTEENTH 

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	switch (microstep)
	{
	case FULL:
		digitalWrite(pin.ED_MS1, LOW);
		digitalWrite(pin.ED_MS2, LOW);
		digitalWrite(pin.ED_MS3, LOW);
		break;
	case HALF:
		digitalWrite(pin.ED_MS1, HIGH);
		digitalWrite(pin.ED_MS2, LOW);
		digitalWrite(pin.ED_MS3, LOW);
		break;
	case QUARTER:
		digitalWrite(pin.ED_MS1, LOW);
		digitalWrite(pin.ED_MS2, HIGH);
		digitalWrite(pin.ED_MS3, LOW);
		break;
	case EIGHTH:
		digitalWrite(pin.ED_MS1, HIGH);
		digitalWrite(pin.ED_MS2, HIGH);
		digitalWrite(pin.ED_MS3, LOW);
		break;
	case SIXTEENTH:
		digitalWrite(pin.ED_MS1, HIGH);
		digitalWrite(pin.ED_MS2, HIGH);
		digitalWrite(pin.ED_MS3, HIGH);
		break;
	}

	// Print change
	Debug.sprintf_safe(buffLrg, buff_lrg, "Set Microsteps to \"%s\"", p_str_microstep[microstep]);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

}

void REWARD::CheckFeedArm()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool is_move_done = false;
	bool is_timedout = false;

	// Bail if nothing to do
	if (!do_ExtendArm &&
		!do_RetractArm &&
		!do_TimedRetract)
	{

		// Make sure motor asleep if arm not extended
		if (!isArmExtended && digitalRead(pin.ED_SLP) == HIGH) {
			digitalWrite(pin.ED_SLP, LOW);
		}

		// Bail
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Do timed retract
	if (do_TimedRetract) {

		// Check if its time to retract arm
		if (millis() > t_retractArm) {

			// Log
			Debug.sprintf_safe(buffLrg, buff_lrg, "Time to Retract Feeder Arm: dt_rew=%d",
				millis() - t_rewStr);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Set to retract
			RetractFeedArm();
		}
	}

	// Bail if still nothing to do
	if (!do_ExtendArm &&
		!do_RetractArm)
	{
		// Make sure motor asleep if not extended
		if (!isArmExtended && digitalRead(pin.ED_SLP) == HIGH) {
			// Sleep motor
			digitalWrite(pin.ED_SLP, LOW);
		}
		return;
	}

	// Release switch when switch triggered on retract
	if (v_stepDir == 'r' &&
		digitalRead(pin.SWITCH_DISH) == LOW) {

		// Block handler
		v_doStepTimer = false;
		delayMicroseconds(ezStepPeriod + 100);

		// Stop timer
		FeederArmTimer.stop();

		// Make sure step off
		v_stepState = false;
		digitalWrite(pin.ED_STP, v_stepState);

		// Set direction to extend
		digitalWrite(pin.ED_DIR, ezDirExtState == 1 ? HIGH : LOW);
		delayMicroseconds(100);

		// Move arm x steps
		for (int i = 0; i < ezRestStps; i++)
		{
			digitalWrite(pin.ED_STP, HIGH);
			delayMicroseconds(500);
			digitalWrite(pin.ED_STP, LOW);
			delayMicroseconds(500);
			v_cnt_steps--;
		}

		// Set done flag
		v_isArmMoveDone = true;

		// Bail
		return;

	}

	// Check if done extending/retracting
	if ((do_ExtendArm || do_RetractArm) &&
		v_isArmMoveDone) {

		is_move_done = true;
	}

	// Check if timedout
	else if (millis() > t_moveArmStr + armMoveTimeout) {

		is_timedout = true;
	}

	// Target reached
	if (is_move_done || is_timedout) {

		// Block handler
		v_doStepTimer = false;
		delayMicroseconds(ezStepPeriod + 100);

		// Stop timer
		FeederArmTimer.stop();

		// Unstep motor
		if (digitalRead(pin.ED_STP) == HIGH) {
			digitalWrite(pin.ED_STP, LOW);
		}

		// Set arm state flags
		if (do_ExtendArm) {
			isArmExtended = true;
		}
		else if (do_RetractArm) {
			isArmExtended = false;
		}

		// Sleep motor if arm not extended
		if (!isArmExtended) {
			digitalWrite(pin.ED_SLP, LOW);
		}

		// Log status
		if (!is_timedout) {

			// Print success
			Debug.sprintf_safe(buffLrg, buff_lrg, "SUCCEEDED: Arm %s: cnt_steps=%d step_targ=%d dt_move=%d",
				isArmExtended ? "Extend" : "Retract", v_cnt_steps, v_stepTarg, millis() - t_moveArmStr);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		}
		else {

			// Print timeout error
			Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT AFTER %dms: Arm %s: cnt_steps=%d step_targ=%d",
				millis() - t_moveArmStr, isArmExtended ? "Extend" : "Retract", v_cnt_steps, v_stepTarg);
			Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Reset pos time and flags
		v_cnt_steps = 0;
		v_stepTarg = 0;
		t_moveArmStr = 0;
		isArmStpOn = false;
		do_ExtendArm = false;
		do_RetractArm = false;
		v_isArmMoveDone = true;
	}
}

void REWARD::RewardReset(bool was_rewarded)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';

	// Log event
	Debug.DB_General(__FUNCTION__, __LINE__, "Reseting Reward");

	// Log zone info
	if (rewMode == FREE || rewMode == CUE)
	{
		// Format zone occ message
		Debug.sprintf_safe(buffLrg, buff_lrg, "ZONE OCC:");
		for (int i = zoneMin; i <= zoneMax; i++) {
			Debug.sprintf_safe(buffMed, buff_med, " z%d=%dms", i + 1, zoneOccTim[i]);
			Debug.strcat_safe(buffLrg, strlen(buff_lrg), buff_lrg, strlen(buff_med), buff_med);
		}
		// Log
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Format zone cnt message
		Debug.sprintf_safe(buffLrg, buff_lrg, "ZONE CNT:");
		for (int i = zoneMin; i <= zoneMax; i++) {
			Debug.sprintf_safe(buffMed, buff_med, " z%d=%d", i + 1, zoneOccCnt[i]);
			Debug.strcat_safe(buffLrg, strlen(buff_lrg), buff_lrg, strlen(buff_med), buff_med);
		}
		// Log
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	}

	// Reset flags etc
	isRewarding = false;
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

LOGGER::LOGGER(USARTClass &_hwSerial) :
	hwSerial(_hwSerial),
	str_sml_reset(buffTerm, __LINE__, _str_sml_reset),
	str_sml_head(buffTerm, __LINE__, _str_sml_head),
	str_sml_foot(buffTerm, __LINE__, _str_sml_foot),
	str_sml_success(buffTerm, __LINE__, _str_sml_success),
	str_sml_warnings(buffTerm, __LINE__, _str_sml_warnings),
	str_sml_abort(buffTerm, __LINE__, _str_sml_abort)
{}

bool LOGGER::Setup()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

#if DO_LOG
	/*
	NOTE:
	config.txt settings:
	57600,26,3,2,0,0,0
	baud,escape,esc#,mode,verb,echo,ignoreRX
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	byte match = '\0';

	// Start serial
	hwSerial.begin(57600);

	// Reset OpenLog
	t_sent = millis();
	digitalWrite(pin.OL_RST, HIGH);
	delay(100);
	digitalWrite(pin.OL_RST, LOW);
	delay(100);
	match = GetReply(5000);

	// Bail if setup failed
	if (match != '>' && match != '<') {
		Debug.DB_Error(__FUNCTION__, __LINE__, "ABORTING: DID NOT GET INITIAL \'>\' or \'<\'");
		return false;
	}

	// Set to command mode;
	if (match == '<') {
		if (!SetToCmdMode()) {
			return false;
		}
	}

	// Turn off verbose mode and echo mode
	SendCommand("verbose off\r");
	SendCommand("echo off\r");

	// Get settings
	if (SendCommand("get\r") == '!') {
		return false;
	}
	else if (strlen(buff_rcvdArr) + 10 < buffLrg) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "%s", buff_rcvdArr);
	}

	return true;

#else
	return true;

#endif
}

int LOGGER::OpenNewLog()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

#if DO_LOG

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	byte match = 0;

	// Make sure in command mode
	if (!SetToCmdMode())
		return 0;

	// Check/cd to log directory
	if (SendCommand("cd LOGS\r") == '!')
	{
		// Make new log dir
		if (SendCommand("md LOGS\r") == '!') {
			return 0;
		}
		if (SendCommand("cd LOGS\r") == '!') {
			return 0;
		}
		Debug.DB_General(__FUNCTION__, __LINE__, "Made \"LOGS\" Directory");

		// Make new log count file
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!') {
			return 0;
		}
		Debug.DB_General(__FUNCTION__, __LINE__, "Made \"LOGCNT.TXT\" File");
	}

	// Get log count
	else if (SendCommand("read LOGCNT.TXT\r") != '!') {
		// Store count
		logNum = atoi(buff_med_fiCnt) + 1;
	}
	else {
		return 0;
	}


	// Check if more than 250 logs saved
	if (logNum > 250)
	{
		// Step out of directory
		if (SendCommand("cd ..\r") == '!') {
			return 0;
		}

		// Delete directory
		if (SendCommand("rm -rf LOGS\r") == '!') {
			return 0;
		}

		// Print 
		Debug.sprintf_safe(buffLrg, buff_lrg, "Deleted Log Directory: log_count=%d", logNum);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Make new log dir
		if (SendCommand("md LOGS\r") == '!') {
			return 0;
		}
		if (SendCommand("cd LOGS\r") == '!') {
			return 0;
		}
		Debug.DB_General(__FUNCTION__, __LINE__, "Made \"LOGS\" Directory");

		// Make new log count file
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!') {
			return 0;
		}
		Debug.DB_General(__FUNCTION__, __LINE__, "Made \"LOGCNT.TXT\" File");
	}

	// Update count
	if (SendCommand("write LOGCNT.TXT\r") == '!') {
		return 0;
	}
	// write int string
	Debug.sprintf_safe(buffLrg, buff_lrg, "{{%d}}\r", logNum);
	SendCommand(buff_lrg);
	// exit with empty line
	buff_lrg[0] = '\r';
	buff_lrg[1] = '\0';
	if (SendCommand(buff_lrg) == '!') {
		return 0;
	}

	// Format log file name
	Debug.sprintf_safe(buffMed, buff_med_logFile, "LOG%05u.CSV", logNum);

	// Begin logging to this file
	if (!SetToWriteMode()) {
		return 0;
	}

	// Write log header
	hwSerial.write(str_sml_head.data(), 3);

	// Write first log entry
	Debug.sprintf_safe(buffLrg, buff_lrg, "BEGIN LOGGING TO \"%s\"", buff_med_logFile);
	QueueLog(buff_lrg);

	// Set flag
	isFileReady = true;

	// Return log number
	return logNum;

#else
	sprintf(logFile, "LOGNULL");
	return -1;

#endif

}

bool LOGGER::SetToCmdMode()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_sml[buffTerm] = { 0 }; buff_sml[0] = '\0';
	bool pass = false;

	// Check if already in cmd mode
	if (mode == '>') {
		return true;
	}

	// Add delay if command just sent
	int del = 100 - (millis() - t_sent);
	if (del > 0) {
		delay(del);
	}

	// Add delay if log just sent
	del = dt_write - (micros() - t_write);
	if (del > 0) {
		delayMicroseconds(del);
	}

	//Send 3x escape char (byte)26 command mode
	if (SendCommand(str_sml_reset.data(), true, 500) == '>')
	{
		// Set flag
		pass = true;

		// Pause to let OpenLog get its shit together
		delay(100);

		// Print status
		Debug.sprintf_safe(buffLrg, buff_lrg, "OpenLog Set to Cmd Mode: mode = %c", mode);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Log error
	else {
		// Set flag
		pass = false;

		// Log
		Debug.sprintf_safe(buffLrg, buff_lrg, "ABORTED: mode=%c", mode);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	return pass;
}

void LOGGER::GetCommand()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static int byte_ind_out = 0;

	// Get terminal command
	if (SerialUSB.available() > 0)
	{
		byte_ind_out = 0;
		while (SerialUSB.available() > 0)
		{
			buff_lrg[byte_ind_out] = SerialUSB.read();
			byte_ind_out++;
			delayMicroseconds(100);
		}

		// Print command
		buff_lrg[byte_ind_out] = '\0';
		SendCommand(buff_lrg);

	}
}

char LOGGER::SendCommand(char *p_msg, bool do_conf, uint32_t timeout)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	char reply = '\0';

	// Add min delay
	int del = 15 - (millis() - t_sent);
	if (del > 0) {
		delay(del);
	}

	// Copy message
	for (int i = 0; i < strlen(p_msg); i++) {
		buff_lrg_2[i] = p_msg[i];
	}
	buff_lrg_2[strlen(p_msg)] = '\0';

	// Send
	hwSerial.write(buff_lrg_2);
	t_sent = millis();

	// Print sent
	if (Debug.flag.print_a2o)
	{
		Debug.DB_OpenLog("SENT[=======================", true);
		for (int i = 0; i < strlen(buff_lrg_2) + 1; i++) {
			Debug.DB_OpenLog(Debug.FormatSpecialChars(buff_lrg_2[i]));
		}
		Debug.DB_OpenLog("\n=======================]SENT\n\n");
	}

	// Get confirmation
	if (do_conf) {
		reply = GetReply(timeout);
		// Check for error
		if (reply == '!') {
			// Remove '\r'
			buff_lrg_2[strlen(buff_lrg_2) - 1] = buff_lrg_2[strlen(buff_lrg_2) - 1] == '\r' ? '\0' : buff_lrg_2[strlen(buff_lrg_2) - 1];
			Debug.sprintf_safe(buffLrg, buff_lrg, "Command %s Failed", buff_lrg_2);
			Debug.DB_OpenLog(buff_lrg, true);
		}
	}
	else {
		reply = mode;
	}

	// Return mode
	return reply;
}

char LOGGER::GetReply(uint32_t timeout)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	VEC<bool> f_arr(3, __LINE__);
	uint32_t t_start = millis();
	uint32_t t_timeout = millis() + timeout;
	VEC<int> dat_ind(2, __LINE__);
	int arr_ind = -1;
	char cmd_reply = ' ';
	bool pass = false;

	// Wait for new data
	while (
		hwSerial.available() == 0 &&
		millis() < t_timeout
		);
	t_rcvd = millis();

	// Check for match byte
	if (Debug.flag.print_o2a) {
		Debug.DB_OpenLog("RCVD_FORMATED[==========", true);
	}
	while (
		!pass &&
		millis() < t_timeout
		)
	{
		// Get new data
		if (hwSerial.available() > 0)
		{
			// Get next byte
			char c = hwSerial.read();

			// Print new byte
			if (Debug.flag.print_o2a) {
				SerialUSB.print(c);
			}

			// Incriment byte ind
			if (
				arr_ind < maxBytesStore - 2 ||
				buff_rcvdArr[arr_ind] == '!' ||
				buff_rcvdArr[arr_ind] == '<' ||
				buff_rcvdArr[arr_ind] == '>'
				) {
				arr_ind++;
			}

			// Store data
			buff_rcvdArr[arr_ind] = c;
		}
		// Make sure message finished
		else if (arr_ind >= 0) {
			for (int i = arr_ind; i >= 0; i--)
			{
				// Check that certain specific comnination of chars recieved
				f_arr[0] = buff_rcvdArr[i] == '\r' || buff_rcvdArr[i] == '1' || buff_rcvdArr[i] == '~' ? true : f_arr[0];
				f_arr[1] = buff_rcvdArr[i] == '\n' || buff_rcvdArr[i] == '2' || buff_rcvdArr[i] == '~' ? true : f_arr[1];
				f_arr[2] = buff_rcvdArr[i] == '>' || buff_rcvdArr[i] == '<' ? true : f_arr[2];
			}
			if (f_arr[0] && f_arr[1] && f_arr[2]) {
				pass = true;
			}
		}
	}
	// Set null terminator
	buff_rcvdArr[arr_ind + 1] = '\0';

	if (Debug.flag.print_o2a) {
		Debug.DB_OpenLog("\n============================================]RCVD_FORMATED\n");
	}

	// Print formated string
	if (Debug.flag.print_o2aRaw) {
		Debug.DB_OpenLog("RCVD_RAW[==========", true);
		for (int i = 0; i <= arr_ind + 1; i++)
		{
			Debug.sprintf_safe(buffLrg, buff_lrg, "\'%s\'", Debug.FormatSpecialChars(buff_rcvdArr[i]));
			Debug.DB_OpenLog(buff_lrg);
		}
		Debug.DB_OpenLog("\n============================================]RCVD_RAW\n");
	}

	// Save values
	for (int i = 0; i <= arr_ind; i++)
	{
		// Save cmd 
		if (
			buff_rcvdArr[i] == '!' ||
			buff_rcvdArr[i] == '<' ||
			buff_rcvdArr[i] == '>'
			) {
			cmd_reply = cmd_reply != '!' ? buff_rcvdArr[i] : cmd_reply;
		}

		// Check for data start indeces
		dat_ind[0] =
			dat_ind[0] == 0 && buff_rcvdArr[i - 1] == '{' && buff_rcvdArr[i] == '{' ?
			i + 1 : dat_ind[0];
		// Check for end indeces
		if (
			dat_ind[0] > 0 &&
			buff_rcvdArr[i] == '}' &&
			buff_rcvdArr[i + 1] == '}'
			) {
			dat_ind[1] = i - 1;
		}
	}

	// Get data
	if (
		dat_ind[0] != 0 &&
		dat_ind[1] != 0
		) {
		int ii = 0;
		for (int i = dat_ind[0]; i <= dat_ind[1]; i++)
		{
			// Bail if overflow
			if (ii >= buffMed - 1) {
				break;
			}

			// Store next value
			buff_med[ii] = buff_rcvdArr[i];
			ii++;
		}
		buff_med[ii + 1] = '\0';

		// Copy
		Debug.sprintf_safe(buffMed, buff_med_fiCnt, "%s", buff_med);

	}

	// Get current mode
	mode = cmd_reply == '>' || cmd_reply == '<' ? cmd_reply : mode;

	// Print mode and round trip time
	if (Debug.flag.print_logMode || Debug.flag.print_o2a || Debug.flag.print_o2aRaw) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "mode=\'%c\' reply=\'%c\' dt=%dms bytes=%d", mode, cmd_reply, t_rcvd - t_sent, arr_ind + 1);
		Debug.DB_OpenLog(buff_lrg, true);
	}

	// Log error
	if (!pass) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Timedout: dt=%d bytes=%d", timeout, arr_ind + 1);
		Debug.DB_OpenLog(buff_lrg, true);
	}

	// Return cmd 
	return cmd_reply;
}

bool LOGGER::SetToWriteMode()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool pass = false;

	// Check if already in log mode
	if (mode == '<') {
		return true;
	}

	// Send append file command
	Debug.sprintf_safe(buffLrg, buff_lrg, "append %s\r", buff_med_logFile);
	if (SendCommand(buff_lrg) != '!') {
		pass = true;
	}

	// Store new log file
	if (pass) {
		delay(100);
		Debug.sprintf_safe(buffLrg, buff_lrg, "OpenLog Set to Write Mode: file_name=%s mode = %c",
			buff_med_logFile, mode);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log error
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED: OpenLog Set to Write Mode: file_name=%s mode = %c",
			buff_med_logFile, mode);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	return pass;
}

void LOGGER::QueueLog(char *p_msg, uint32_t ts, char *p_type, const char *p_fun, int line)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

#if DO_LOG

	// Local vars
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_save[buffMed] = { 0 };
	static uint16_t overflow_cnt = 0;
	uint32_t t_m = 0;

	// Bail if queue store blocked
	if (FC.do_BlockLogQueue) {
		return;
	}

	// Update queue ind
	LQ_StoreInd++;

	// Check if ind should roll over 
	if (LQ_StoreInd == LQ_Capacity) {
		LQ_StoreInd = 0;
	}

	// Handle overflow
	if (LQ_Queue[LQ_StoreInd][0] != '\0') {

		// Add to count 
		overflow_cnt += overflow_cnt < 1000 ? 1 : 0;

		// Update string
		Debug.sprintf_safe(buffMed, buff_med_save, "[*LOG_Q-DROP:%d*] ", overflow_cnt);

		// Set queue back
		LQ_StoreInd = LQ_StoreInd - 1 >= 0 ? LQ_StoreInd - 1 : LQ_Capacity - 1;

		// Bail
		return;
	}

	// Incriment count
	cnt_logsStored++;

	// Get sync correction
	t_m = ts - t_sync;

	// Add function and line number
	if (strcmp(p_fun, "") != 0) {
		Debug.sprintf_safe(buffMed, buff_med_1, "[%s:%d] ", p_fun, line);
	}

	// Store log
	Debug.sprintf_safe(buffMax, LQ_Queue[LQ_StoreInd], "[%d],%lu,=\"%s\",%d,%s,%s%s%s\r\n",
		cnt_logsStored, t_m, Debug.FormatTimestamp(t_m), cnt_loopShort, p_type, buff_med_save, buff_med_1, p_msg);

	// Reset overlow stuff
	overflow_cnt = 0;
	buff_med_save[0] = '\0';

	// Write now
#if DO_FAST_LOG
	if (mode == '<') {
		Log.WriteAll(500);
	}
#endif

#endif
}

bool LOGGER::WriteLog()
{
#if DO_LOG
	/*
	STORE LOG DATA FOR CS
	FORMAT: "[log_cnt],ts_ms,message,\r\n"
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool is_logs = false;

	// Bail if no new logs
	if (GetLogQueueAvailable() == LQ_Capacity) {

		// Indicate no logs to write
		is_logs = false;
		return is_logs;
	}
	else {
		is_logs = true;
	}

	// Bail if writing blocked
	if (FC.do_BlockLogWrite) {

		return is_logs;
	}

	// Bail if not in write mode
	if (mode != '<') {

		// Return queue status
		return is_logs;
	}

	// Bail if too little time since last log write
	if (!DO_FAST_LOG && micros() < t_write + dt_write) {
		// Indicate still logs to store
		return is_logs;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Incriment send ind
	LQ_ReadInd++;

	// Check if ind should roll over 
	if (LQ_ReadInd == LQ_Capacity) {
		LQ_ReadInd = 0;
	}

	// Write to SD
	hwSerial.write(LQ_Queue[LQ_ReadInd]);
	t_write = micros();

	// Update bytes stored
	cnt_logBytesStored += strlen(LQ_Queue[LQ_ReadInd]);

	// Print stored log
	if (Debug.flag.print_logWrite) {

		// Remove \r\n from message string
		LQ_Queue[LQ_ReadInd][strlen(LQ_Queue[LQ_ReadInd]) - 2] = '\0';

		// Format string
		Debug.sprintf_safe(buffLrg, buff_lrg, "r2c: cnt=%d b_stored=%d/%d b_sent=%d q_store=%d q_read=%d \"%s\"",
			cnt_logsStored, strlen(LQ_Queue[LQ_ReadInd]), cnt_logBytesStored, cnt_logBytesSent, LQ_StoreInd, LQ_ReadInd, LQ_Queue[LQ_ReadInd]);

		// Debug
		Debug.DB_LogWrite(buff_lrg);

	}

	// Set entry to null
	LQ_Queue[LQ_ReadInd][0] = '\0';

	// Return queue status
	return is_logs;

#else
	return false;

#endif
}

bool LOGGER::WriteAll(uint32_t timeout)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	uint16_t cnt_write = 0;
	int queue_size = 0;
	uint32_t t_timeout = millis() + timeout;
	bool is_timedout = false;

	// Bail if writing blocked
	if (FC.do_BlockLogWrite) {
		Debug.DB_Warning(__FUNCTION__, __LINE__, "WriteLog Is Blocked");
		return false;
	}

	// Loop till done or timeout reached
	while (Log.WriteLog()) {

		// Incriment counter
		cnt_write++;

		// Check for timeout
		if (millis() > t_timeout) {
			is_timedout = true;
			break;
		}
	}

	// Get queue left
	queue_size = LQ_Capacity - Log.GetLogQueueAvailable();

	// Check for timeout
	if (is_timedout) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: cnt_write%d queued=%d dt_run=%d",
			cnt_write, queue_size, timeout);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Return all writen flag
	return queue_size == 0;

}

void LOGGER::StreamLogs()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

#if DO_LOG

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '|';
	static char buff_lrg_3[buffLrg] = { 0 }; buff_lrg_3[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	static char buff_sml[buffTerm] = { 0 }; buff_sml[0] = '\0';
	const int dt_send_timeout = 1200000; // (ms)
	const int dt_read_timeout = 100; // (ms)
	static int cnt_err_change_mode = 0;
	static int cnt_err_read_request = 0;
	static int cnt_err_read_timeout = 0;
	uint32_t t_start = millis(); // (ms)
	uint32_t t_last_read = millis(); // (ms)
	uint32_t t_dump = millis(); // (ms)
	int read_ind = 0;
	bool head_passed = false;
	bool send_done = false;
	bool do_flag_warnings = false;
	bool do_abort = false;
	bool is_read_timedout = false;
	bool is_send_timedout = false;
	VEC<int>  milestone_incriment(11, __LINE__);
	int milestone_ind = 0;

	// Bail if not ready to send
	if (millis() < t_beginSend) {
		return;
	}

	// Store memory for summary
	memEnd = freeMemory();

	// Store remaining logs
	WriteAll(1000);
	delay(100);

	// Stop storing and writing logs
	FC.do_BlockLogQueue = true;
	FC.do_BlockLogWrite = true;

	// Write footer
	hwSerial.write(str_sml_foot.data(), 3);
	delay(100);

	// Make sure in command mode
	if (!SetToCmdMode()) {

		// Add to error counter
		cnt_err_change_mode++;

		// Leave function
		if (cnt_err_change_mode < 3) {
			return;
		}

		// Abort after 3 failures
		else {
			do_abort = true;
		}

		// Store error
		Debug.sprintf_safe(buffLrg, buff_lrg, "\"%c%c%c\" Failed: cnt=%d|",
			26, 26, 26, cnt_err_change_mode);
		Debug.strcat_safe(buffLrg, strlen(buff_lrg_2), buff_lrg_2, strlen(buff_lrg), buff_lrg);
	}

	// Print anything left in queue
	Debug.PrintAll(1000);

	// Start timers
	t_start = millis();
	t_dump = t_start;

	// Get incriments to update status
	for (int i = 1; i < 10; i++)
	{
		milestone_incriment[i] = round((cnt_logBytesStored / 10) * (i + 1));
	}
	milestone_incriment[0] = 1;
	milestone_incriment[10] = cnt_logBytesStored;

	// Begin streaming data
	while (true) {

		// Dump anything in openlog buffer
		uint32_t t_out = millis() + 100;
		while (millis() < t_dump || hwSerial.available() > 0) {

			if (hwSerial.available() > 0) {
				hwSerial.read();
			}
		}

		// Send new "read" command
		if (!send_done &&
			!do_abort) {

			Debug.sprintf_safe(buffLrg, buff_lrg, "read %s %d %d\r", buff_med_logFile, 0, (uint32_t)-1);
			SendCommand(buff_lrg, false, 5000);
		}

		// Finished
		else {
			break;
		}

		// Reset vars
		head_passed = false;
		read_ind = 0;
		buff_sml[0] = 0;
		buff_sml[1] = 0;
		buff_sml[2] = 0;
		t_last_read = millis();

		// Read one byte at a time
		while (!do_abort) {

			// Check for send timeout
			if (is_send_timedout = is_send_timedout || millis() > (t_start + dt_send_timeout)) {
				do_abort = true;
				break;
			}

			// Check for new data
			if (hwSerial.available() == 0)
			{

				// Check for read timeout
				is_read_timedout = millis() - t_last_read > dt_read_timeout;

				// Continue if not read timeout
				if (!is_read_timedout ||
					cnt_logBytesSent == 0) {

					continue;
				}

				// Handle read timeout
				else {

					// Add to error counter
					cnt_err_read_timeout++;

					// Try re-requesting data at least once
					if (cnt_err_read_timeout > 1) {
						do_abort = true;
					}

					// Store error
					Debug.sprintf_safe(buffLrg, buff_lrg, "Read Timedout: cnt=%d read_ind=%d dt_read=%d|",
						cnt_err_read_timeout, read_ind, millis() - t_last_read);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_2), buff_lrg_2, strlen(buff_lrg), buff_lrg);

					// Break
					break;
				}
			}

			// Get next bytes
			buff_sml[0] = buff_sml[1];
			buff_sml[1] = buff_sml[2];
			buff_sml[2] = hwSerial.read();
			t_last_read = millis();
			read_ind++;

			// Check for header
			if (!head_passed) {

				// Header found
				if (buff_sml[0] == str_sml_head[0] &&
					buff_sml[1] == str_sml_head[1] &&
					buff_sml[2] == str_sml_head[2]) {

					head_passed = true;
					continue;
				}

				// Continue till header found
				else {
					continue;
				}
			}

			// Check for error
			if (buff_sml[2] == '!') {

				if ((buff_sml[0] == '\r' && buff_sml[1] == '\n') ||
					read_ind < 3) {

					// Add to read error counter
					cnt_err_read_request++;

					// Retry "read" command only once
					if (cnt_err_read_request > 1) {
						do_abort = true;
					}

					// Store error
					Debug.sprintf_safe(buffLrg, buff_lrg, "\"read\" Failed: cnt=%d|", cnt_err_read_request);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_2), buff_lrg_2, strlen(buff_lrg), buff_lrg);

					// Break
					break;
				}
			}

			// Check for footer
			if (buff_sml[2] == str_sml_foot[0] ||
				buff_sml[2] == str_sml_foot[1] ||
				buff_sml[2] == str_sml_foot[2]) {

				// Check if complete footer found
				if (buff_sml[0] == str_sml_foot[0] &&
					buff_sml[1] == str_sml_foot[1] &&
					buff_sml[2] == str_sml_foot[2]) {

					send_done = true;
					break;
				}
				else {
					continue;
				}
			}

			// Send byte
			r2c.hwSerial.write(buff_sml[2]);
			cnt_logBytesSent++;

			// Print status
			if (cnt_logBytesSent == milestone_incriment[milestone_ind]) {

				// Print
				Debug.sprintf_safe(buffLrg, buff_lrg, "Log Write %d%% Complete: b_sent=%d/%d",
					milestone_ind * 10, cnt_logBytesSent, cnt_logBytesStored);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, millis());
				Debug.Print();

				// Incriment count
				milestone_ind++;
			}

			// Check if all bytes sent
			if (cnt_logBytesSent >= cnt_logBytesStored) {

				send_done = true;
				break;
			}

		}
	}

	// Check for send timeout
	if (is_send_timedout) {

		do_abort = true;
		Debug.sprintf_safe(buffLrg, buff_lrg, "Send Timedout|");
		Debug.strcat_safe(buffLrg, strlen(buff_lrg_2), buff_lrg_2, strlen(buff_lrg), buff_lrg);
	}

	// Unblock log store
	FC.do_BlockLogQueue = false;
	delay(50);

	// Tracknig summary info
	Debug.sprintf_safe(buffLrg, buff_lrg, "TRACKING SUMMARY: ERR=|ad=%d|ekf=%d|rat_vt=%d|rat_pixy=%d|rob_vt=%d| RECS=|rat_vt=%lu|rat_pixy=%lu|rob_vt=%lu| DT=|rat_vt=%0.0f|rat_pixy=%0.0f|rob_vt=%0.0f| SWAP=|rat_vt=%d|rat_pixy=%d|",
		cnt_errAD, cnt_errEKF, Pos[0].cnt_err, Pos[2].cnt_err, Pos[1].cnt_swap,
		Pos[0].cnt_rec, Pos[2].cnt_rec, Pos[1].cnt_rec,
		(double)Pos[0].dt_recSum / Pos[0].cnt_rec, (double)Pos[2].dt_recSum / Pos[2].cnt_rec, (double)Pos[1].dt_recSum / Pos[1].cnt_rec,
		Pos[0].cnt_swap, Pos[2].cnt_swap);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Get total data left in buffers
	uint16_t xbee_tx_size = SERIAL_BUFFER_SIZE - 1 - r2c.hwSerial.availableForWrite();
	uint16_t xbee_rx_size = r2c.hwSerial.available();
	uint16_t log_tx_size = SERIAL_BUFFER_SIZE - 1 - hwSerial.availableForWrite();
	uint16_t log_rx_size = hwSerial.available();

	// Com summary info general
	Debug.sprintf_safe(buffLrg, buff_lrg, "COM SUMMARY GENERAL: RX=|flood=%lu|tout=%lu|xbeebuff=%d|logbuff=%d| TX=|xbeebuff=%d|logbuff=%d|",
		cnt_overflowRX, cnt_timeoutRX, xbee_rx_size, log_rx_size,
		xbee_tx_size, log_tx_size);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Com summary info CS
	Debug.sprintf_safe(buffLrg, buff_lrg, "COM SUMMARY CS: R2C=|pind=%d|psent=%lu|resnd=%lu| C2R=|pind=%d|psent=%lu|prcvd=%lu|rercv=%lu|drop=%d|",
		r2c.packInd, r2c.packSentAll, r2c.cnt_repeat,
		c2r.packInd, c2r.packSentAll, c2r.packRcvdAll, c2r.cnt_repeat, c2r.cnt_dropped);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Com summary info CheetahDue
	Debug.sprintf_safe(buffLrg, buff_lrg, "COM SUMMARY CHEETAHDUE: R2A=|pind=%d|psent=%lu|resnd=%lu| A2R=|pind=%d|psent=%lu|prcvd=%lu|rercv=%lu|drop=%d|",
		r2a.packInd, r2a.packSentAll, r2a.cnt_repeat,
		a2r.packInd, a2r.packSentAll, a2r.packRcvdAll, a2r.cnt_repeat, a2r.cnt_dropped);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Com summary info Teensy
#if DO_TEENSY_DEBUG
	Debug.sprintf_safe(buffLrg, buff_lrg, "COM SUMMARY TEENSY: R2T=|pind=%d|psent=%lu|",
		t2r.packInd, t2r.packSentAll);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);
#endif

	// Warnings summary
	Debug.sprintf_safe(buffLrg, buff_lrg_3, "ON LINES |");
	for (int i = 0; i < Debug.cnt_warn; i++) {
		Debug.sprintf_safe(buffMed, buff_med, "%d|", Debug.warn_line[i]);
		Debug.strcat_safe(buffLrg, strlen(buff_lrg_3), buff_lrg_3, strlen(buff_med), buff_med);
}
	Debug.sprintf_safe(buffLrg, buff_lrg, "TOTAL WARNINGS: %d %s", Debug.cnt_warn, Debug.cnt_warn > 0 ? buff_lrg_3 : "");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	// Send
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Errors summary
	Debug.sprintf_safe(buffLrg, buff_lrg_3, "ON LINES |");
	for (int i = 0; i < Debug.cnt_err; i++) {
		Debug.sprintf_safe(buffMed, buff_med, "%d|", Debug.err_line[i]);
		Debug.strcat_safe(buffLrg, strlen(buff_lrg_3), buff_lrg_3, strlen(buff_med), buff_med);
	}
	Debug.sprintf_safe(buffLrg, buff_lrg, "TOTAL ERRORS:  %d %s", Debug.cnt_err, Debug.cnt_err > 0 ? buff_lrg_3 : "");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Log summary info
	float dt_s = (float)(millis() - t_start) / 1000.0f;
	Debug.sprintf_safe(buffLrg, buff_lrg, "LOG SUMMARY: dt_run=%0.2fs b_sent=%d b_stored=%d err_change_mode=%d err_read_request=%d err_read_timeout=%d",
		dt_s, cnt_logBytesSent, cnt_logBytesStored, cnt_err_change_mode, cnt_err_read_request, cnt_err_read_timeout);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Run summary info
	dt_s = (float)millis() / 1000.0f;
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUN SUMMARY: DT_RUN_ALL=%0.2fs MEM_START=%dKB MEM_END=%dKB",
		dt_s, memStart, memEnd);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Print final status then send as log
	if (!do_abort) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "SUCCEEDED: Sent %d Logs", cnt_logsStored + 1);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "ABORTED: Sending %d Logs: errors=%s", cnt_logsStored + 1, buff_lrg_2);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}
	r2c.hwSerial.write(LQ_Queue[LQ_StoreInd]);
	delay(50);

	// Flag warnings
	do_flag_warnings = !do_abort && (do_flag_warnings = cnt_err_change_mode > 0 || cnt_err_read_request > 0 || cnt_err_read_timeout > 0);

	// End reached send ">>>"
	if (!do_abort) {
		r2c.hwSerial.write(str_sml_success.data(), 3);
	}
	// Flag warnings ">>*"
	else if (do_flag_warnings) {
		r2c.hwSerial.write(str_sml_warnings.data(), 3);
	}
	// Aborted send ">>!"
	else {
		r2c.hwSerial.write(str_sml_abort.data(), 3);
	}

	// Print anything left in queue
	Debug.PrintAll(1000);

#else

	// End reached send ">>>"
	r2c.hwSerial.write(msg_streamSuccess, 3);

#endif

	// Reset flag
	FC.do_LogSend = false;

	// Set back to write mode
	if (SetToWriteMode()) {
		FC.do_BlockLogWrite = false;
	}

}

void LOGGER::TestLoad(int n_entry, char *p_log_file)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	//EXAMPLE:
	/*
	Log.TestLoad(0, "LOG00035.CSV");
	Log.TestLoad(2500);
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	bool pass = false;

	// Prevent any new info from logging
	WriteAll(5000);
	FC.do_BlockLogQueue = true;

	// Load existing log file
	if (p_log_file != '\0') {
		Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING: Load Log: file_name=%s...", p_log_file);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, millis());
		Debug.PrintAll(1000);

		if (SetToCmdMode()) {

			// Get bytes
			if (SendCommand("ls\r") != '!') {
				cnt_logBytesStored = GetFileSize(p_log_file);
				pass = true;
			}

			// Store log file name
			Debug.sprintf_safe(buffMed, buff_med_logFile, "%s", p_log_file);

			// Start writing to file
			if (pass)
				if (!SetToWriteMode())
					pass = false;
		}
		if (pass) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "SUCCEEDED: Load Log: file_name=%s size=%dB",
				buff_med_logFile, cnt_logBytesStored);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, millis());
			Debug.PrintAll(1000);
		}
		else {
			Debug.DB_Error(__FUNCTION__, __LINE__, "ABORTED: Load Log File");
			Debug.PrintAll(1000);
		}
	}

	// Write n_entry entries to log
	else if (n_entry != 0) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING: Write %d Logs...", n_entry);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, millis());
		Debug.PrintAll(1000);
		delayMicroseconds(100);
		WriteAll(1000);

		// Add n new logs
		randomSeed(analogRead(A0));
		int milestone_incriment = n_entry / 10;
		for (int i = 0; i < n_entry - 1; i++)
		{
			// Print status
			if (i%milestone_incriment == 0) {
				Debug.sprintf_safe(buffLrg, buff_lrg, "Log Write %d%% Complete", i / milestone_incriment * 10);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, millis());
				Debug.PrintAll(1000);
				WriteAll(1000);
			}

			// Store a 120 charicter string
			Debug.sprintf_safe(buffLrg, buff_lrg_2,
				"AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFFGGGGGGGGGGHHHHHHHHHHIIIIIIIIIIJJJJJJJJJJKKKKKKKKKKLLLLLLLLLL");

			// Store log
			QueueLog(buff_lrg_2, millis());
			t_write = micros() - dt_write;
			WriteAll(1000);
		}
		Debug.sprintf_safe(buffLrg, buff_lrg, "FINISHED: Write %d Logs", n_entry);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, millis());
		Debug.PrintAll(1000);
		WriteAll(1000);
	}
}

int LOGGER::GetFileSize(char *p_log_file)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	int ind = -1;
	int fi_size = 0;

	// Find file index in array
	for (int i = strlen(buff_rcvdArr) - strlen(p_log_file); i >= 0; i--)
	{
		int ii = 0;
		for (ii = strlen(p_log_file) - 1; ii >= 0; ii--) {

			if (p_log_file[ii] != buff_rcvdArr[i + ii]) {
				break;
			}
			else if (ii == 0) {
				ind = ii + i;
				break;
			}
		}
		if (ind != -1) {
			break;
		}
	}

	// Get file size
	if (ind != -1) {

		int i = ind + strlen(p_log_file) + 3;
		int ii = 0;

		while (i < strlen(buff_rcvdArr) && buff_rcvdArr[i] != '\r')
		{
			buff_med[ii] = buff_rcvdArr[i];
			i++;
			ii++;
		}

		buff_med[ii] = '\0';
		fi_size = atoi(buff_med);
	}

	return fi_size;
}

int LOGGER::GetLogQueueAvailable() {
#if DO_TEENSY_DEBUG
	//DB_FUN_STR();
#endif

	// Local vars
	int n_entries = 0;

	// Check each entry
	for (int i = 0; i < LQ_Capacity; i++) {
		n_entries += LQ_Queue[i][0] != '\0' ? 1 : 0;
	}

	// Get total available
	return LQ_Capacity - n_entries;

}

#pragma endregion 

#pragma endregion 


#pragma region ========== FUNCTION DEFINITIONS ========


#pragma region --------COMMUNICATION---------

// CHECK FOR HANDSHAKE
bool CheckForHandshake()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static bool is_on = false;
	static uint32_t t_timeout = 0;
	static uint32_t t_pulse_last = 0;
	static uint16_t dt_blink_on = 10;
	static uint16_t dt_blink_off = 490;
	int dt_ir = 0;

	if (FC.is_SesStarted) {
		return true;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// CHECK FOR IR HANDSHAKE
	if (!FC.is_IRHandshakeDone) {

		// Pulse tracker led
		if (!is_on && millis() >= t_pulse_last + dt_blink_on + dt_blink_off) {

			analogWrite(pin.LED_TRACKER, trackLEDduty[1]);
			t_pulse_last = millis();
			is_on = true;
		}
		else if (is_on && millis() >= t_pulse_last + dt_blink_on) {

			analogWrite(pin.LED_TRACKER, 0);
			is_on = false;
		}

		// Check for sync handshake pulse
		if (t_sync == 0)
		{

			// Copy ir dt
			dt_ir = v_dt_ir;

			// Check for setup ir pulse
			if (abs(dt_irHandshakePulse - dt_ir) <= 5) {

				// Set sync time
				t_sync = v_t_irSyncLast;

				// Log pulse time
				Debug.sprintf_safe(buffLrg, buff_lrg, "HANDSHAKE IR SYNC PULSE DETECTED: ACTUAL DT=%dms EXPECTED DT=75ms", dt_ir);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				// Log sync time
				Debug.sprintf_safe(buffLrg, buff_lrg, "SET SYNC TIME: %dms", t_sync);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				// Set tracker led high
				analogWrite(pin.LED_TRACKER, trackLEDduty[1]);

				// Set flag
				FC.is_IRHandshakeDone = true;
				Debug.DB_General(__FUNCTION__, __LINE__, "IR Handshake Confirmed");

				// Send handshake to CS
				QueuePacket(&r2c, 'h', 1, 0, 0, 0, true);
				while (SendPacket(&r2c));

				// Set abort time
				t_timeout = millis() + dt_timeoutHandshake;

			}

		}

	}

	// Check for handshake confirmation from CS
	else if (!FC.is_CSHandshakeDone) {

		if (!r2c.do_rcvCheckArr[ID_Ind<R2_COM<USARTClass>>('h', &r2c)]) {

			// Set flag
			FC.is_CSHandshakeDone = true;
			Debug.DB_General(__FUNCTION__, __LINE__, "CS Handshake Confirmed");

			// Send handshake to CheetahDue
			QueuePacket(&r2a, 'h', 1, 0, 0, 0, true);
			while (SendPacket(&r2a));

		}
	}

	// Check for handshake confirmation from CheetahDue
	else if (!FC.is_CheetahDueHandshakeDone) {

		if (!r2a.do_rcvCheckArr[ID_Ind<R2_COM<USARTClass>>('h', &r2a)]) {

			// Set flag
			FC.is_CheetahDueHandshakeDone = true;
			Debug.DB_General(__FUNCTION__, __LINE__, "CheetaDue Handshake Confirmed");
		}

	}

	// Check if handshake complete
	if (FC.is_CSHandshakeDone && FC.is_IRHandshakeDone && FC.is_CheetahDueHandshakeDone) {

		// Reset LCD
		FC.do_ChangeLCDstate = FC.is_LitLCD;
		Debug.ClearLCD();

		// Reset solonoids
		if (digitalRead(pin.REL_ETOH) == HIGH) {
			OpenCloseEtOHSolenoid();
		}
		if (digitalRead(pin.REL_FOOD) == HIGH) {
			OpenCloseRewSolenoid();
		}

		// Print board status
		Debug.sprintf_safe(buffLrg, buff_lrg, "BOARD R STATUS: %04X", AD_R.getStatus());
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		Debug.sprintf_safe(buffLrg, buff_lrg, "BOARD F STATUS: %04X", AD_F.getStatus());
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// PRINT AVAILABLE MEMORY
		Debug.sprintf_safe(buffLrg, buff_lrg, "AVAILABLE MEMORY: %0.2fKB",
			(float)freeMemory() / 1000);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Send final handshake confirmation to CS with log number
		QueuePacket(&r2c, 'h', 2, (float)Log.logNum, 0, 0, true);
		SendPacket(&r2c);

		// Set flag
		FC.is_SesStarted = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "HANDSHAKE COMPLETE");

		// Indicate completion
		StatusBlink(true, 10, 100);

		// Return success
		return true;

	}

	// Check for handshake timeout
	else if (t_timeout > 0 && millis() > t_timeout) {

		// Format message
		Debug.sprintf_safe(buffLrg, buff_lrg, "HANDSHAKE TIMEDOUT AFTER %d ms: |%s%s%s",
			dt_timeoutHandshake,
			!FC.is_IRHandshakeDone ? "NO IR HANDSHAKE|" : "",
			!FC.is_CSHandshakeDone ? "NO CS CONFERMATION|" : "",
			!FC.is_CheetahDueHandshakeDone ? "NO CHEEDAHDUE CONFERMATION|" : "");

		// Run error hold and restart
		Debug.RunErrorHold(buff_lrg, "HANDSHAKE", 15);
	}

	// Handshake not complete
	else {
		return false;
	}

}

// PARSE SERIAL INPUT
void GetSerial(R4_COM<USARTClass> *p_r4)
{

	/*
	PARSE DATA FROM CS
	FORMAT: [0]head, [1]id, [2:5]dat[0], [6:9]dat[1], [10:13]dat[1], [14:15]pack, [16]flag_byte, [17]footer, [18]targ
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static char buff_lrg_3[buffLrg] = { 0 }; buff_lrg_3[0] = '\0';
	static char buff_lrg_4[buffLrg] = { 0 }; buff_lrg_4[0] = '\0';
	uint32_t t_start = millis();
	int dt_parse = 0;
	int dt_sent = 0;
	uint16_t tx_size = 0;
	uint16_t rx_size = 0;
	byte b = 0;
	char head = ' ';
	char id = ' ';
	VEC<float> dat(3, __LINE__);
	int r4_ind = 0;
	int r2_ind = 0;
	uint16_t pack = 0;
	char foot = ' ';
	bool do_conf = false;
	bool is_conf = false;
	bool is_resend = false;
	bool is_repeat = false;
	bool is_missed = false;
	bool is_dropped = false;
	int n_missed = 0;
	byte flag_byte = 0;
	R2_COM<USARTClass> *p_r2;
	R2_COM<USARTClass> *p_r2o;
	R4_COM<USARTClass> *p_r4o;

	// Set pointer to R2_COM struct
	if (p_r4->comID == COM::ID::c2r) {
		p_r2 = &r2c;
		p_r2o = &r2a;
		p_r4o = &a2r;
	}
	else if (p_r4->comID == COM::ID::a2r) {
		p_r2 = &r2a;
		p_r2o = &r2c;
		p_r4o = &c2r;
	}

	// Reset vars
	cnt_bytesRead = 0;
	cnt_bytesDiscarded = 0;
	p_r4->is_new = false;
	p_r4->idNew = ' ';

	// Bail if no new input
	if (p_r4->hwSerial.available() == 0) {

		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Dump data till p_msg header byte is reached
	b = WaitBuffRead(p_r4, p_r4->head);
	if (b == 0) {

		return;
}

	// Store header
	head = b;

	// Get id
	id = WaitBuffRead(p_r4);

	// Parse data
	for (int i = 0; i < 3; i++)
	{
		U.f = 0.0f;
		U.b[0] = WaitBuffRead(p_r4);
		U.b[1] = WaitBuffRead(p_r4);
		U.b[2] = WaitBuffRead(p_r4);
		U.b[3] = WaitBuffRead(p_r4);
		dat[i] = U.f;
	}

	// Get packet num
	U.f = 0.0f;
	U.b[0] = WaitBuffRead(p_r4);
	U.b[1] = WaitBuffRead(p_r4);
	pack = U.i16[0];

	// Get confirmation flag
	U.f = 0.0f;
	U.b[0] = WaitBuffRead(p_r4);
	flag_byte = U.b[0];
	do_conf = GetSetByteBit(&flag_byte, 0, false);
	is_conf = GetSetByteBit(&flag_byte, 1, false);
	is_resend = GetSetByteBit(&flag_byte, 3, false);

	// Get footer
	foot = WaitBuffRead(p_r4);

	// Get total data in buffers
	rx_size = p_r4->hwSerial.available();
	tx_size = SERIAL_BUFFER_SIZE - 1 - p_r4->hwSerial.availableForWrite();

	// Compute parce and sent dt
	dt_parse = millis() - t_start;
	dt_sent = p_r2->t_sent > 0 ? millis() - p_r2->t_sent : 0;

	// Check for dropped packet
	if (foot != p_r4->foot) {

		// Incriment dropped count
		p_r4->cnt_dropped++;

		// Set flag
		is_dropped = true;
	}

	// Footer found so process packet
	else {

		// Send confirmation
		if (do_conf) {
			QueuePacket(p_r2, id, dat[0], dat[1], dat[2], pack, false, true);
		}

		// Get id ind
		r4_ind = ID_Ind<R4_COM<USARTClass>>(id, p_r4);
		r2_ind = ID_Ind<R2_COM<USARTClass>>(id, p_r2);

		// Set coms started flag
		if (p_r4->comID == COM::ID::c2r && !FC.is_ComsStarted) {
			FC.is_ComsStarted = true;
		}

		// Reset check
		p_r2->do_rcvCheckArr[r2_ind] = false;
		p_r2->cnt_repeatArr[r2_ind] = 0;

		// Update times
		p_r4->dt_rcvd = p_r4->t_rcvd > 0 ? millis() - p_r4->t_rcvd : 0;
		p_r4->t_rcvd = millis();

		// Get last pack
		uint16_t pack_last;
		if (!is_conf)
			pack_last = p_r4->packArr[r4_ind];
		else
			pack_last = p_r4->packConfArr[r4_ind];

		// Flag repeat pack
		is_repeat = pack == pack_last;

		// Incriment repeat count
		p_r4->cnt_repeat += is_repeat || is_resend ? 1 : 0;

		// Update packet history
		if (!is_conf)
			p_r4->packArr[r4_ind] = pack;
		else
			p_r4->packConfArr[r4_ind] = pack;

		// Update data
		p_r4->dat[0] = dat[0];
		p_r4->dat[1] = dat[1];
		p_r4->dat[2] = dat[2];

		// Update for new packets
		if (!is_conf && !is_repeat) {

			// Get pack diff accounting for packet rollover
			int pack_diff =
				abs(pack - p_r4->packInd) < (p_r4->packRange[1] - p_r4->packRange[0]) ?
				pack - p_r4->packInd :
				pack - (p_r4->packRange[0] - 1);

			// Update dropped packets
			n_missed = pack_diff - 1;
			p_r4->cnt_dropped += n_missed > 0 ? n_missed : 0;

			// flag missed packets
			if (n_missed > 0) {
				is_missed = true;
			}

			// Update packets sent
			p_r4->packSentAll += pack_diff;

			// Update packet ind 
			p_r4->packInd = pack;

			// Incriment packets recieved
			p_r4->packRcvdAll++;

			// Update new message info
			p_r4->idNew = id;
			p_r4->is_new = true;

		}

	}

	// Format data strings
	Debug.sprintf_safe(buffLrg, buff_lrg_2, "\'%c\': dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s",
		id, dat[0], dat[1], dat[2], pack, Debug.FormatBinary(flag_byte));
	Debug.sprintf_safe(buffLrg, buff_lrg_3, " b_read=%d b_dump=%d rx=%d tx=%d dt(snd|rcv|prs)=|%d|%d|%d|",
		cnt_bytesRead, cnt_bytesDiscarded, rx_size, tx_size, dt_sent, p_r4->dt_rcvd, dt_parse);

	// Log recieved
	if (!is_dropped) {

		// Log as warning if re-revieving packet
		if (is_repeat || is_resend) {
			Debug.sprintf_safe(buffLrg, buff_lrg_4, "Recieved %s %s Packet: %s %s",
				is_repeat ? "Duplicate" : is_resend ? "Resent" : "", COM::str_list_id[p_r4->comID], buff_lrg_2, buff_lrg_3);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg_4, p_r4->t_rcvd);
		}

		// Log recieved
		Debug.DB_Rcvd(p_r4, buff_lrg_2, buff_lrg_3, is_repeat, flag_byte);
	}
	// Log dropped packs warning
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Dropped %s Packs: cnt=%lu %s %s",
			COM::str_list_id[p_r4->comID], p_r4->cnt_dropped, buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log missed packs warning
	if (is_missed) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Missed %s Packs: (cns|tot)=|%d|%lu| p_last=%d %s %s",
			COM::str_list_id[p_r4->comID], n_missed, p_r4->cnt_dropped, p_r4->packInd, buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log discarded bytes warning
	if (cnt_bytesDiscarded > 0) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Dumped Bytes: %s %s", buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log parse hanging warning
	if (dt_parse > 30) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Parser Hanging: %s %s", buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	return;
}

// PROCESS PIXY STREAM
uint16_t GetPixy(bool is_hardware_test)
{


}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4_COM<USARTClass> *p_r4, char mtch)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	bool is_overflowed = false;
	byte b = 0;

	// Get total data in buffers now
	uint16_t rx_size_start = p_r4->hwSerial.available();

	// Check for overflow
	is_overflowed = rx_size_start >= SERIAL_BUFFER_SIZE - 1;

	// Wait for at least 1 byte
	while (p_r4->hwSerial.available() < 1 &&
		millis() < t_timeout);

	// Get any byte
	if (!is_overflowed &&
		mtch == '\0') {

		if (p_r4->hwSerial.available() > 0) {

			b = p_r4->hwSerial.read();
			cnt_bytesRead++;

			// Bail
			return b;
		}
	}

	// Find specific byte
	while (
		b != mtch  &&
		millis() < t_timeout &&
		!is_overflowed) {

		// Check new data
		if (p_r4->hwSerial.available() > 0) {

			b = p_r4->hwSerial.read();
			cnt_bytesRead++;

			// check match was found
			if (b == mtch) {

				// Bail
				return b;
			}

			// Otherwise add to discard count
			else {
				cnt_bytesDiscarded++;
			}

			// Check for overflow
			is_overflowed =
				!is_overflowed ? p_r4->hwSerial.available() >= SERIAL_BUFFER_SIZE - 1 : is_overflowed;
		}

	}

	// Check if buffer flooded
	if (is_overflowed) {

		// DUMP IT ALL
		while (p_r4->hwSerial.available() > 0) {
			if (p_r4->hwSerial.available() > 0) {
				p_r4->hwSerial.read();
				cnt_bytesRead++;
			}
		}
	}

	// Get buffer 
	uint16_t tx_size = SERIAL_BUFFER_SIZE - 1 - p_r4->hwSerial.availableForWrite();
	uint16_t rx_size = p_r4->hwSerial.available();

	// Store current info
	Debug.sprintf_safe(buffLrg, buff_lrg_2, " from=%s buff_lrg=\'%s\' b_read=%d b_dump=%d rx_start=%d rx_now=%d tx_now=%d dt_chk=%d",
		COM::str_list_id[p_r4->comID], Debug.FormatSpecialChars(b), cnt_bytesRead, cnt_bytesDiscarded, rx_size_start, rx_size, tx_size, (millis() - t_timeout) + timeout);

	// Buffer flooded
	if (is_overflowed) {

		// Incriment count
		cnt_overflowRX++;

		// Print only every 10th message after first 10
		if (cnt_overflowRX < 10 || cnt_overflowRX % 10 == 0) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "BUFFER OVERFLOWED: cnt=%d", cnt_overflowRX);
		}
		// Bail
		else {
			return 0;
		}
	}

	// Timed out
	else if (millis() > t_timeout) {

		// Incriment count
		cnt_timeoutRX++;

		// Print only every 10th message after first 10
		if (cnt_timeoutRX < 10 || cnt_timeoutRX % 10 == 0) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: cnt=%d", cnt_timeoutRX);
		}
		// Bail
		else {
			return 0;
		}
	}

	// Byte not found
	else if (mtch != '\0') {
		Debug.sprintf_safe(buffLrg, buff_lrg, "CHAR \'%c\' NOT FOUND:",
			mtch);
	}

	// Failed for unknown reason
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED FOR UNKNOWN REASON:");
	}

	// Compbine strings
	Debug.strcat_safe(buffLrg, strlen(buff_lrg), buff_lrg, strlen(buff_lrg_2), buff_lrg_2);

	// Log error
	Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

	// Return 0

	// Bail
	return 0;

}

// STORE PACKET DATA TO BE SENT
void QueuePacket(R2_COM<USARTClass> *p_r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf, bool is_conf, bool is_done, bool is_resend)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	float _dat[3] = { dat1 , dat2 , dat3 };
	VEC<float> dat(3, __LINE__, _dat);
	byte flag_byte = 0;
	int id_ind = 0;
	uint16_t tx_size = 0;
	uint16_t rx_size = 0;
	R4_COM<USARTClass> *p_r4;

	// Set pointer to R4_COM struct
	if (p_r2->comID == COM::ID::r2c) {
		p_r4 = &c2r;
	}
	else if (p_r2->comID == COM::ID::r2a) {
		p_r4 = &a2r;
	}

	// Set flag_byte
	GetSetByteBit(&flag_byte, 0, do_conf);
	GetSetByteBit(&flag_byte, 1, is_conf);
	GetSetByteBit(&flag_byte, 2, is_done);
	GetSetByteBit(&flag_byte, 3, is_resend);

	// Get buffers
	tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
	rx_size = p_r2->hwSerial.available();

	// Update sendQueue ind
	p_r2->SQ_StoreInd++;

	// Check if ind should roll over 
	if (p_r2->SQ_StoreInd == SQ_Capacity) {
		p_r2->SQ_StoreInd = 0;
	}

	// Check if overfloweed
	if (p_r2->SQ_Queue[p_r2->SQ_StoreInd][0] != '\0')
	{

		// Get list of empty entries
		for (int i = 0; i < SQ_Capacity; i++) {
			buff_lrg[i] = p_r2->SQ_Queue[i][0] == '\0' ? '0' : '1';
		}
		buff_lrg[SQ_Capacity] = '\0';

		// Format queue overflow error string
		Debug.sprintf_safe(buffLrg, buff_lrg, "[*%s_Q-DROP*]: queue_s=%d queue_r=%d queue_state=|%s|",
			COM::str_list_id[p_r2->comID], p_r2->SQ_StoreInd, p_r2->SQ_ReadInd, buff_lrg);

		// Log error
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

		// Set queue back 
		p_r2->SQ_StoreInd = p_r2->SQ_StoreInd - 1 >= 0 ? p_r2->SQ_StoreInd - 1 : SQ_Capacity - 1;

		// Bail
		return;

	}

	// Incriment packet ind if originating from robot
	if (pack == 0) {

		// Incriment packet
		p_r2->packInd++;

		// Reset packet if out of range
		if (p_r2->packInd > p_r2->packRange[1])
		{
			Debug.sprintf_safe(buffLrg, buff_lrg, "RESETTING %s PACKET INDEX FROM %d TO %d",
				COM::str_list_id[p_r2->comID], p_r2->packInd, p_r2->packRange[0]);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Set to lowest range value
			p_r2->packInd = p_r2->packRange[0];
		}

		// Copy packet number
		pack = p_r2->packInd;

	}

	// Create byte packet
	int b_ind = 0;
	// Store header
	p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = p_r2->head;
	// Store mesage id
	p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = id;
	// Store mesage data 
	for (int i = 0; i < 3; i++)
	{
		U.f = dat[i];
		p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[0];
		p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[1];
		p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[2];
		p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[3];
	}
	// Store packet number
	U.f = 0.0f;
	U.i16[0] = pack;
	p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[0];
	p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = U.b[1];
	// Store flag_byte flag
	p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = flag_byte;
	// Store footer
	p_r2->SQ_Queue[p_r2->SQ_StoreInd][b_ind++] = p_r2->foot;

	// Store time
	id_ind = ID_Ind<R2_COM<USARTClass>>(id, p_r2);
	p_r2->t_queuedArr[id_ind] = millis();

	// Format data string
	Debug.sprintf_safe(buffLrg, buff_lrg, "\'%c\': dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s",
		id, dat[0], dat[1], dat[2], pack, Debug.FormatBinary(flag_byte));

	// Log queued packet info
	Debug.DB_SendQueued(p_r2, buff_lrg, p_r2->t_queuedArr[id_ind]);

	return;

}

// SEND SERIAL PACKET DATA
bool SendPacket(R2_COM<USARTClass> *p_r2)
{

	/*
	STORE DATA TO SEND
	FORMAT IN:  [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer, [9]targ
	FORMAT OUT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static char buff_lrg_3[buffLrg] = { 0 }; buff_lrg_3[0] = '\0';
	static char buff_lrg_4[buffLrg] = { 0 }; buff_lrg_4[0] = '\0';
	int dt_rcvd = 0;
	int dt_queue = 0;
	char id = '\0';
	VEC<float> dat(3, __LINE__);
	bool do_conf = false;
	bool is_conf = false;
	bool is_resend = false;
	bool is_repeat = false;
	byte flag_byte = 0;
	uint16_t pack = 0;
	uint16_t pack_last = 0;
	uint16_t tx_size = 0;
	uint16_t rx_size = 0;
	int id_ind = 0;
	R4_COM<USARTClass> *p_r4;
	R4_COM<USARTClass> *p_r4o;
	R2_COM<USARTClass> *p_r2o;

	// Set pointer to R4_COM struct
	if (p_r2->comID == COM::ID::r2c) {
		p_r4 = &c2r;
		p_r2o = &r2a;
		p_r4o = &a2r;
	}
	else if (p_r2->comID == COM::ID::r2a) {
		p_r4 = &a2r;
		p_r2o = &r2c;
		p_r4o = &c2r;
	}

	// Bail if nothing in queue
	if (p_r2->SQ_ReadInd == p_r2->SQ_StoreInd &&
		p_r2->SQ_Queue[p_r2->SQ_StoreInd][0] == '\0') {

		// Bail
		return false;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Get buffer 
	tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
	rx_size = p_r2->hwSerial.available();

	// Bail if buffer or time inadequate
	if (tx_size > tx_sizeMaxSend ||
		rx_size > rx_sizeMaxSend ||
		millis() < p_r2->t_sent + p_r2->dt_minSentRcvd) {

		// Indicate still packs to send and bail
		return true;
	}

	// Add small delay if just recieved
	else if (millis() < p_r4->t_rcvd + p_r4->dt_minSentRcvd) {
		delayMicroseconds(500);
	}

	// Incriment send ind
	p_r2->SQ_ReadInd++;

	// Check if ind should roll over 
	if (p_r2->SQ_ReadInd == SQ_Capacity) {
		p_r2->SQ_ReadInd = 0;
	}

	// Send
	p_r2->hwSerial.write(p_r2->SQ_Queue[p_r2->SQ_ReadInd], SQ_MsgBytes);

	// Update dt stuff
	p_r2->dt_sent = p_r2->t_sent > 0 ? millis() - p_r2->t_sent : 0;
	p_r2->t_sent = millis();
	dt_rcvd = p_r4->t_rcvd > 0 ? millis() - p_r4->t_rcvd : 0;
	dt_queue = millis() - p_r2->t_queuedArr[id_ind];

	// Get buffers
	tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
	rx_size = p_r2->hwSerial.available();

	// pull out packet data
	int b_ind = 1;
	// id
	id = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
	// dat
	for (int i = 0; i < 3; i++)
	{
		U.f = 0;
		U.b[0] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
		U.b[1] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
		U.b[2] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
		U.b[3] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
		dat[i] = U.f;
	}
	// pack
	U.f = 0.0f;
	U.b[0] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
	U.b[1] = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
	pack = U.i16[0];
	// conf flag
	flag_byte = p_r2->SQ_Queue[p_r2->SQ_ReadInd][b_ind++];
	do_conf = GetSetByteBit(&flag_byte, 0, false);
	is_conf = GetSetByteBit(&flag_byte, 1, false);
	is_resend = GetSetByteBit(&flag_byte, 3, false);

	// Set entry to null
	p_r2->SQ_Queue[p_r2->SQ_ReadInd][0] = '\0';

	// Get id ind
	id_ind = ID_Ind<R2_COM<USARTClass>>(id, p_r2);

	// Set flags for recieve confirmation
	if (do_conf) {
		p_r2->do_rcvCheckArr[id_ind] = true;
	}

	// Get last pack
	pack_last = is_conf ? p_r2->packConfArr[id_ind] : p_r2->packArr[id_ind];

	// Flag resent pack
	is_repeat = pack == pack_last;

	// Incriment repeat count
	p_r2->cnt_repeat += is_repeat || is_resend ? 1 : 0;

	// Incriment total sent packet count
	p_r2->packSentAll++;

	// Update packet history
	if (!is_conf)
		p_r2->packArr[id_ind] = pack;
	else
		p_r2->packConfArr[id_ind] = pack;

	// Update other struct info
	p_r2->dat1[id_ind] = dat[0];
	p_r2->dat2[id_ind] = dat[1];
	p_r2->dat3[id_ind] = dat[2];
	p_r2->flagArr[id_ind] = flag_byte;
	p_r2->t_sentArr[id_ind] = p_r2->t_sent;

	// Format data string
	Debug.sprintf_safe(buffLrg, buff_lrg_2, "\'%c\': dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s",
		id, dat[0], dat[1], dat[2], pack, Debug.FormatBinary(flag_byte));
	Debug.sprintf_safe(buffLrg, buff_lrg_3, "b_sent=%d tx=%d rx=%d dt(snd|rcv|q)=|%d|%d|%d| ez(on|act)=|%d|%d| sol_act(etoh|food)=|%d|%d| vcc_rel=%d",
		SQ_MsgBytes, tx_size, rx_size, p_r2->dt_sent, dt_rcvd, dt_queue, digitalRead(pin.ED_SLP), v_doStepTimer, digitalRead(pin.REL_ETOH) == HIGH, digitalRead(pin.REL_FOOD) == HIGH, digitalRead(pin.REL_VCC) == HIGH);

	// Log as warning if resending duplicate packet
	if (is_repeat && !is_resend) {
		Debug.sprintf_safe(buffLrg, buff_lrg_4, "Sent Duplicate %s Packet: %s %s", COM::str_list_id[p_r2->comID], buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg_4, p_r4->t_rcvd);
	}

	// Log sent
	Debug.DB_Sent(p_r2, buff_lrg_2, buff_lrg_3, is_repeat, flag_byte);

	// Return success
	return true;
}

// CHECK IF ROB TO ARD PACKET SHOULD BE RESENT
bool CheckResend(R2_COM<USARTClass> *p_r2)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	bool is_waiting_for_pack = false;
	uint16_t pack_last = 0;
	bool is_done = false;
	int dt_sent = 0;

	// Bail if nothing to send
	for (int i = 0; i < p_r2->lng; i++)
	{
		is_waiting_for_pack = is_waiting_for_pack || p_r2->do_rcvCheckArr[i];
	}
	if (!is_waiting_for_pack) {
		return false;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Loop and check ard flags
	for (int i = 0; i < p_r2->lng; i++)
	{
		// Flag if waiting on anything
		if (p_r2->do_rcvCheckArr[i]) {

			// Set flag
			is_waiting_for_pack = true;

			// Get dt sent
			dt_sent = millis() - p_r2->t_sentArr[i];
		}

		// Bail if no action requred
		if (
			!p_r2->do_rcvCheckArr[i] ||
			dt_sent < p_r2->dt_resend
			) {
			continue;
		}

		// Get total data left in buffers
		uint16_t tx_size = SERIAL_BUFFER_SIZE - 1 - p_r2->hwSerial.availableForWrite();
		uint16_t rx_size = p_r2->hwSerial.available();

		// Get dat string
		Debug.sprintf_safe(buffLrg, buff_lrg_2, "id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s dt_sent=%dms tx=%d rx=%d",
			p_r2->id[i], p_r2->dat1[i], p_r2->dat2[i], p_r2->dat3[i], p_r2->packArr[i], Debug.FormatBinary(p_r2->flagArr[i]), dt_sent, tx_size, rx_size);

		// Get done flag
		is_done = GetSetByteBit(&p_r2->flagArr[i], 2, false);

		if (p_r2->cnt_repeatArr[i] < p_r2->resendMax) {

			// Resend with same packet number
			QueuePacket(p_r2, p_r2->id[i], p_r2->dat1[i], p_r2->dat2[i], p_r2->dat3[i], p_r2->packArr[i], true, false, is_done, true);

			// Update count
			p_r2->cnt_repeatArr[i]++;

			// Print resent packet
			Debug.sprintf_safe(buffLrg, buff_lrg, "Resending %s Packet: cnt=%d %s",
				COM::str_list_id[p_r2->comID], p_r2->cnt_repeatArr[i], buff_lrg_2);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);

		}

		// Coms failed
		else {

			// Log error
			Debug.sprintf_safe(buffLrg, buff_lrg, "ABORTED: Resending %s Packet: cnt=%d %s",
				COM::str_list_id[p_r2->comID], p_r2->cnt_repeatArr[i], buff_lrg_2);
			Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Reset flag
		p_r2->do_rcvCheckArr[i] = false;

	}

	// Return
	return is_waiting_for_pack;
}

// LOG FUNCTION RUN TO TEENSY p_msg=["S", "E", other]
void SendTeensy(const char *p_fun, int line, int mem, char id, char *p_msg)
{
	// Notes
	/*
	!!!!CALLING ANY OTHER METHOD FROM HERE WILL RESULT IN RECURSIVE CALL!!!!!
	// Coms Union (20B):
	1B/1B: (char)head: c[0]
	1B/2B: (char)id: c[1]
	2B/4B: (uint16_t)pack: i16[1]
	4B/8B: (uint32_t)time_ms: i32[1]
	2B/10B: (uint16_t)line_number: i16[4]
	2B/12B: (char)function_abreviation: c[10-11]
	1B/13B: (byte)loop_count: b[12]
	1B/14B: (byte)skip_count: b[13]
	1B/15B: (byte)memory_GB: b[14]
	1B/16B: (char)foot: c[15]
	*/

#if DO_TEENSY_DEBUG

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_save[buffLrg] = { 0 };
	static char buff_sml[buffTerm] = { 0 }; buff_sml[0] = '\0';
	static bool do_skip_repeat = false;
	static uint32_t dt_run = 0;
	static uint32_t cnt_chk = 0;
	static uint32_t chk_last = 0;
	uint32_t dt_send_wait = 1000; // (us)
	int cnt_skip = 0;
	int chk_diff = 0;
	uint16_t b_ind = 0;
	uint32_t t_start = micros();
	uint32_t t_m = 0;
	uint16_t line_num = line - 22;
	byte mem_gb = 0;
	static byte msg_bytes = sizeof(Utnsy.b);

	// Bail if not ready
	if (!FC.is_TeensyReady) {
		return;
	}

	// Bail if message repeat
	if (strcmp(p_fun, buff_lrg_save) == 0) {

		// Only bail for repeat start message
		if (do_skip_repeat ||
			id == 'S') {

			// Set flag and bail
			do_skip_repeat = true;
			return;
		}
	}

	// Reset flag
	do_skip_repeat = false;

	// Incriment count
	cnt_chk++;

	// Wait if message just sent
	uint32_t dt_send = micros() - r2t.t_sent;
	uint32_t t_send_wait =
		dt_send < dt_send_wait ? micros() + (dt_send_wait - dt_send) : 0;
	while (micros() < t_send_wait) {
		if (micros() > t_send_wait) {
			break;
		}
		else {
			delayMicroseconds(10);
		}
	}

	// Bail if buffer still full
	if (r2t.hwSerial.availableForWrite() < 2 * msg_bytes) {
		return;
	}

	// Copy function for start calls
	if (id == 'S') {
		Debug.sprintf_safe(buffLrg, buff_lrg_save, "%s", p_fun);
	}

	// Get number of times skipped logging
	cnt_skip = cnt_chk - chk_last - 1;
	chk_last = cnt_chk;

	// Get abreviated function name
	for (int i = 0; i < strlen(p_fun); i++) {
		if (isUpperCase(p_fun[i])) {

			// Store first char
			if (buff_sml[0] == '\0') {
				buff_sml[0] = p_fun[i];
			}

			// Store second char and break
			else {
				buff_sml[1] = p_fun[i];
				break;
			}

		}
	}

	// Store time
	t_m = (millis() - t_sync);

	// Store memory
	mem_gb = (byte)((float)mem / 1000);

	// Incriment packet
	r2t.packInd++;

	// Reset packet if out of range
	if (r2t.packInd > r2t.packRange[1]) {
		// Note: cannot log/print this or will result in recursive call
		r2t.packInd = r2t.packRange[0];
	}

	// Update packet total
	r2t.packSentAll++;

	// Clear union entries
	Utnsy.i64[0] = 0;
	Utnsy.i64[1] = 0;

	// Store header
	Utnsy.c[0] = r2t.head;
	// Store message id
	Utnsy.c[1] = id;
	// Store packet number
	Utnsy.i16[1] = r2t.packInd;

	// Store only message if included
	if (p_msg[0] != '\0') {

		// Incriment through message
		int b_ind = 4;
		for (int i = 0; i < strlen(p_msg); i++) {

			// Store char
			Utnsy.c[b_ind] = p_msg[i];
			b_ind++;
		}
		Utnsy.c[b_ind] = '\0';

	}

	// Store call function details
	else {

		// Store time
		Utnsy.i32[1] = t_m;
		// Store line number
		Utnsy.i16[4] = line_num;
		// Store function abbreviation
		Utnsy.c[10] = buff_sml[0];
		Utnsy.c[11] = buff_sml[1];
		// Store loop count
		Utnsy.b[12] = cnt_loopShort;
		// Store skip count
		Utnsy.b[13] = (byte)cnt_skip;
		// Store memory
		Utnsy.b[14] = mem_gb;

	}

	// Store footer
	Utnsy.c[15] = r2t.foot;

	// Send message
	r2t.hwSerial.write(Utnsy.b, msg_bytes);

	// Get dt run
	dt_run = micros() - t_start;

	// Store send time
	r2t.t_sent = micros();

#endif
}

// GET LAST TEENSY LOG
void ImportTeensy()
{
#if DO_TEENSY_DEBUG

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static char buff_sml[buffTerm] = { 0 }; buff_sml[0] = '\0';
	uint32_t t_check = millis() + 5000;
	uint32_t t_wait_reset = 0;
	uint16_t msg_ind = 0;
	uint16_t cnt_log = 0;
	bool is_com_fail = false;
	bool is_timeout = false;
	VEC<bool> is_reset(2, __LINE__);

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "Running: Import Teensy Logs...");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	Debug.PrintAll(1000);

	// Set send pin high for 150 ms
	digitalWrite(pin.TEENSY_SEND, HIGH);
	delay(10);
	// Send start string
	r2t.hwSerial.write("<<<");
	delay(140);
	digitalWrite(pin.TEENSY_SEND, LOW);

	// Start getting logs
	while (strcmp(buff_sml, ">>>") != 0 &&
		millis() < t_check + 500) {

		// Wait for new data
		if (t2r.hwSerial.available() < 1) {
			continue;
		}

		// Get next byte
		byte b = t2r.hwSerial.read();

		// Update short array
		buff_sml[0] = buff_sml[1];
		buff_sml[1] = buff_sml[2];
		buff_sml[2] = b;

		// Check for head
		if (b == t2r.head) {

			// Reset p_msg string
			buff_lrg_2[0] = '\0';

			// Reset index
			msg_ind = 0;

			// Bail
			continue;
		}

		// Check for foot
		else if (b == t2r.foot) {

			// Terminate message string
			buff_lrg_2[msg_ind] = '\0';

			// Print
			Debug.Queue(buff_lrg_2, millis());
			Debug.Print();

			// Log
			Log.QueueLog(buff_lrg_2, millis());
			Log.WriteLog();

			// Incriment count
			cnt_log++;

			// Bail
			continue;
		}

		// Check for end signal
		else if (digitalRead(pin.TEENSY_RESET) == HIGH) {

			// Set flag
			is_com_fail = true;

			// Bail
			break;
		}

		// Check for timeout
		else if (millis() > t_check) {

			// Set flag
			is_timeout = true;

			// Bail
			break;
		}

		// Check for out of bounds
		else if (msg_ind == buffLrg) {

			// Bail
			return;
		}

		// Add to message
		buff_lrg_2[msg_ind] = b;

		// Incriment count
		msg_ind++;
	}

	// Check for success
	if (strcmp(buff_sml, ">>>") == 0) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FINISHED: Import Teensy Logs: cnt=%d", cnt_log);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Check for com fail
	else if (is_com_fail) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED: Import Teensy Logs: cnt=%d", cnt_log);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Check for timeout
	else if (is_timeout) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: Import Teensy Logs: cnt=%d", cnt_log);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Check for random fail
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED: REASON UNKNOWN: Import Teensy Logs: cnt=%d", cnt_log);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Check for Teensy reset
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Wait for Teensy Reset...");

	// Wait till reset indicator pin goes back low
	t_wait_reset = millis() + 1000;
	while (millis() < t_wait_reset) {

		// Check for pin high
		if (!is_reset[0]) {
			if (digitalRead(pin.TEENSY_RESET) == HIGH) {
				is_reset[0] = true;
			}
		}
		else if (digitalRead(pin.TEENSY_RESET) == LOW) {
			is_reset[1] = true;
			break;
		}
	}

	if (is_reset[0] && is_reset[1]) {
		Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Teensy Reset");

		// Hold for 100 ms for Teensy to finish reset
		delay(100);
	}
	else {
		Debug.DB_Error(__FUNCTION__, __LINE__, "FAILED: Teensy Reset");
	}

	// Print all
	Debug.PrintAll(1000);

#endif
}

#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------

// CONFIGURE AUTODRIVER BOARDS
void AD_Config(float max_acc, float max_dec, float max_speed)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

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
	AD_R.setParam(STEP_MODE, STEP_FS_64);
	AD_F.setParam(STEP_MODE, STEP_FS_64);

	// PWM freq
	/*
	PWM_DIV_X, where X can be any value 1-7.
	PWM_MUL_X, where X can be 0_625 (for 0.625), 0_75 (for 0.75), 0_875, 1, 1_25, 1_5, 1_75, or 2.
	*/
	AD_R.setPWMFreq(PWM_DIV_2, PWM_MUL_2); // 31.25kHz PWM freq
	AD_F.setPWMFreq(PWM_DIV_2, PWM_MUL_2); // 31.25kHz PWM freq		

	// Overcurent enable
	AD_R.setOCShutdown(OC_SD_ENABLE); // shutdown on OC
	AD_F.setOCShutdown(OC_SD_ENABLE); // shutdown on OC

	// Motor V compensation
	/*
	VS_COMP_ENABLE, VS_COMP_DISABLE
	*/
	AD_R.setVoltageComp(VS_COMP_ENABLE);
	AD_F.setVoltageComp(VS_COMP_ENABLE);

	// Switch pin mode
	AD_R.setSwitchMode(SW_USER); // Switch is not hard stop
	AD_F.setSwitchMode(SW_USER); // Switch is not hard stop

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
	AD_R.setOCThreshold(OC_6000mA);
	AD_F.setOCThreshold(OC_6000mA);

	// Low speed compensation
	/*
	Enabled low speed compensation. If enabled, MinSpeed is upper threshold at which this compensation is employed.
	*/
	AD_R.setLoSpdOpt(false);
	AD_F.setLoSpdOpt(false);

	// ---------SPEED SETTTINGS---------

	// Steps/s max
	AD_R.setMaxSpeed(max_speed * cm2stp);
	AD_F.setMaxSpeed(max_speed * cm2stp);

	// Minimum speed
	//AD_R.setMinSpeed(10 * cm2stp);
	//AD_F.setMinSpeed(10 * cm2stp);

	// Full speed
	AD_R.setFullSpeed(max_speed * cm2stp);
	AD_F.setFullSpeed(max_speed * cm2stp);

	// Acceleration
	/*
	Accelerate at maximum steps/s/s; 0xFFF = infinite
	*/
	AD_R.setAcc(max_acc * cm2stp);
	AD_F.setAcc(max_acc * cm2stp);

	// Deceleration
	/*
	Deccelerate at maximum steps/s/s; 0xFFF = infinite
	*/
	AD_R.setDec(max_dec * cm2stp);
	AD_F.setDec(max_dec * cm2stp);

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
	AD_R.setAccKVAL(40); // This controls the acceleration current
	AD_R.setDecKVAL(40); // This controls the deceleration current
	AD_R.setRunKVAL(30); // This controls the run current
	AD_R.setHoldKVAL(25); // This controls the holding current keep it low

	// NIMA 17 24V
	AD_F.setAccKVAL(40); // This controls the acceleration current
	AD_F.setDecKVAL(40); // This controls the deceleration current
	AD_F.setRunKVAL(30); // This controls the run current
	AD_F.setHoldKVAL(25); // This controls the holding current keep it low

}

// RESET AUTODRIVER BOARDS
void AD_Reset(float max_acc, float max_dec, float max_speed)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Reset each axis
	AD_R.resetDev();
	AD_F.resetDev();
	delayMicroseconds(100);

	// Configure each axis
	AD_Config(max_acc, max_dec, max_speed);
	delayMicroseconds(100);
	AD_CheckOC(true);
	delayMicroseconds(100);

	// Run motor at last speed
	if (runSpeedNow > 0) {
		RunMotor(runDirNow, runSpeedNow, MC_CALL::ID::OVERIDE);
	}
	else {
		// Make sure motor is in correct impedance state
		HardStop(__FUNCTION__, __LINE__);
	}

	// Log autodriver settings
	Debug.sprintf_safe(buffLrg, buff_lrg, "Reset AD Boards: max_acc=%0.2f max_dec=%0.2f max_speed=%0.2f",
		max_acc, max_dec, max_speed);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

}

// CHECK AUTODRIVER STATUS
void AD_CheckOC(bool force_check)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static uint32_t t_checkAD = millis() + dt_checkAD; // (ms)
	static bool do_reset_disable = false;
	static int ocd_last_r = 1;
	static int ocd_last_f = 1;
	int ocd_r;
	int ocd_f;

	// Bail if not time for next check
	if (!force_check &&
		millis() < t_checkAD) {

		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Get/check 16 bit status flag
	adR_stat = AD_R.getStatus();
	ocd_r = GetAD_Status(adR_stat, "OCD");
	adF_stat = AD_F.getStatus();
	ocd_f = GetAD_Status(adF_stat, "OCD");

	// Check for overcurrent shut down
	if (ocd_r == 0 || ocd_f == 0) {

		// Track events
		cnt_errAD++;

		// Log
		Debug.sprintf_safe(buffLrg, buff_lrg, "Overcurrent Detected in |%s%s Driver: Resetting AD: cnt=%d now_ocd_R|F=%d|%d last_ocd_R|F=%d|%d",
			ocd_r == 0 ? "REAR|" : "", ocd_f == 0 ? "FRONT|" : "", cnt_errAD, ocd_r, ocd_f, ocd_last_r, ocd_last_f);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);

		// Reset motors
		if (!do_reset_disable) {
			AD_Reset(maxAcc, maxDec, maxSpeed);
		}

	}

	// Store status
	ocd_last_r = ocd_r;
	ocd_last_f = ocd_f;

	// Set next check
	t_checkAD = millis() + dt_checkAD;

	// Disable resetting after 5 errors
	if (cnt_errAD >= 5 && !do_reset_disable) {

		// Log
		Debug.sprintf_safe(buffLrg, buff_lrg, "DISABLED AD RESET AFTER %d ERRORS", cnt_errAD);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

		// Set flag
		do_reset_disable = true;
	}

}

// HARD STOP
void HardStop(const char *p_fun, int line, bool do_block_hz)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Log event
	Debug.sprintf_safe(buffLrg, buff_lrg, "Hard Stop [%s:%d]", p_fun, line - 23);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Normal hard stop
	AD_R.hardStop();
	AD_F.hardStop();

	// Reset speed
	runSpeedNow = 0;

	// Reset pid
	Pid.PidReset();

	// Set to high impedance so robot can be moved
	if (FC.is_ManualSes && !do_block_hz) {
		AD_R.hardHiZ();
		AD_F.hardHiZ();
	}
}

// CHECK IF IR TRIGGERED
void Check_IRprox_Halt()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool is_lft_ir_trigg = false;
	bool is_rt_ir_trigg = false;

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Bail if manual ses
	if (FC.is_ManualSes) {
		return;
	}

	// Bail if already blocking
	if (FC.is_MotBlocking) {
		return;
	}

	// Bail if Bull "ON"
	if (Bull.bullState == BULLDOZE::BULLSTATE::ON) {
		return;
	}

	// Bail if MoveTo active
	if (motorControlNow == MC_CON::ID::MOVETO) {
		return;
	}

	// Bail if already stopped
	if (runSpeedNow == 0) {
		return;
	}

	// Get pin stage
	is_rt_ir_trigg = digitalRead(pin.IRPROX_R) == LOW;
	is_lft_ir_trigg = digitalRead(pin.IRPROX_L) == LOW;

	// Bail if neither IR triggered
	if (!(is_rt_ir_trigg || is_lft_ir_trigg)) {
		return;
	}

	// Run hard stop
	HardStop(__FUNCTION__, __LINE__);

	// Update counters
	cnt_irProxHaltR += is_rt_ir_trigg ? 1 : 0;
	cnt_irProxHaltL += is_lft_ir_trigg ? 1 : 0;

	// Log warning
	Debug.sprintf_safe(buffLrg, buff_lrg, "HALTED FOR IR |%s%s TRIGGER: cnt(rt|lft)=|%lu|%lu|",
		is_rt_ir_trigg ? "RIGHT|" : "", is_lft_ir_trigg ? "LEFT|" : "", cnt_irProxHaltR, cnt_irProxHaltL);
	Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);

}

// RUN AUTODRIVER
bool RunMotor(char dir, double new_speed, MC_CALL::ID caller)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	double speed_rear = 0;
	double speed_front = 0;
	static MC_CALL::ID caller_last = MC_CALL::ID::OVERIDE;

	// Bail if caller does not have control
	if (motorControlNow != caller &&
		caller != MC_CALL::ID::OVERIDE) {

		// Log warning
		if (caller != caller_last) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "Ignored Run Request: conroller=%s caller=%s",
				MC_CON::str_list_id[motorControlNow], MC_CALL::str_list_id[caller]);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
		}

		caller_last = caller;
		return false;
	}

	// Log speed change
	Debug.DB_RunSpeed(__FUNCTION__, __LINE__, caller, runSpeedNow, new_speed);

	// Scale vel for each motor
	for (int i = 0; i < velOrd; i++) {
		speed_rear += rearVelCoeff[i] * pow(new_speed, velOrd - 1 - i);
		speed_front += frontVelCoeff[i] * pow(new_speed, velOrd - 1 - i);
	}

	// Run forward
	if (dir == 'f') {
		AD_R.run(FWD, speed_rear*cm2stp);
		AD_F.run(FWD, speed_front*cm2stp);
	}

	// Run reverse
	else if (dir == 'r') {
		AD_R.run(REV, speed_rear*cm2stp);
		AD_F.run(REV, speed_front*cm2stp);
	}

	// Update run speed and dir
	runSpeedNow = new_speed;
	runDirNow = dir;

	return true;
}

// RUN MOTOR MANUALLY
bool ManualRun(char dir)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';
	int inc_speed = 10; // (cm/sec)
	double new_speed;

	// Check for first run
	if (dir != runDirNow ||
		runSpeedNow == 0) {

		// Set to start speed
		new_speed = 5;
	}

	// Incriment speed
	else {
		new_speed = runSpeedNow <= maxSpeed - inc_speed ?
			runSpeedNow + inc_speed : runSpeedNow;
	}

	// Run motor
	RunMotor(dir, new_speed, MC_CALL::ID::OVERIDE);

	// Print voltage and speed to LCD
	Debug.sprintf_safe(buffMed, buff_med_1, "VCC=%0.2fV", vccAvg);
	Debug.sprintf_safe(buffMed, buff_med_2, "VEL=%s%dcm/s", runDirNow == 'f' ? "->" : "<-", (int)runSpeedNow);
	Debug.PrintLCD(false, buff_med_1, buff_med_2);
}

// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(MC_CON::ID set_to, MC_CALL::ID caller)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool pass = false;
	bool do_change = false;

	// Store current conroller
	MC_CON::ID set_from = motorControlNow;

	// "OVERIDE" CAN DO ANYTHING
	if (caller == MC_CALL::ID::OVERIDE) {
		do_change = true;
	}

	// SET TO: "HALT"
	if (set_to == MC_CON::ID::HALT) {
		// Only "HALT" can set "HALT"
		do_change = set_to == MC_CALL::ID::HALT;
	}

	// SET FROM: "HALT"
	else if (motorControlNow == MC_CON::ID::HALT) {

		// Only "HALT" and "QUIT" can unset "HALT"
		if (caller == MC_CALL::ID::HALT ||
			caller == MC_CALL::ID::QUIT) {

			do_change = true;
		}
	}

	// ALL NON "HALT" CASES
	else {

		// SET TO: "HOLD"
		if (set_to == MC_CON::ID::HOLD) {

			// Only "MOVETO" and "BLOCKER" can set "HOLD"
			if (caller == MC_CALL::ID::MOVETO ||
				caller == MC_CALL::ID::BLOCKER) {
				do_change = true;
			}

		}

		// SET FROM: "HOLD"
		else if (motorControlNow == MC_CON::ID::HOLD) {

			switch (caller)
			{

				// CALLER: "SETUP_TRACKING"
			case MC_CALL::ID::SETUP_TRACKING:

				// Can always unset "HOLD"
				do_change = true;
				break;

				// CALLER: "MOVETO"
			case MC_CALL::ID::MOVETO:

				// Can unset "HOLD" if rat not on track
				if (!FC.is_RatOnTrack || FC.is_TaskDone) {
					do_change = true;
				}
				break;

				// CALLER: "BLOCKER"
			case MC_CALL::ID::BLOCKER:

				// Can unset "HOLD" if tracking setup
				if (FC.is_TrackingEnabled) {
					do_change = true;
				}
				break;

			default:
				do_change = true;
				break;

			}

		}

		// ALL NON "HOLD" CASES
		else {
			// Otherwise can set to anything
			do_change = true;
		}

	}

	// Change controller
	if (do_change) {
		motorControlLast = motorControlLast != motorControlNow ? motorControlNow : motorControlLast;
		motorControlNow = set_to;
	}

	// Check if set control set success
	pass = motorControlNow == set_to;

	// Format string
	Debug.sprintf_safe(buffLrg, buff_lrg, "Change %s: set_from=%s set_in=%s set_out=%s caller=%s",
		pass ? "Succeeded" : "Failed", MC_CON::str_list_id[set_from],
		MC_CON::str_list_id[motorControlNow], MC_CON::str_list_id[set_to], MC_CALL::str_list_id[caller]);

	// Log as warning if failed 
	if (!pass) {
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}
	else {
		Debug.DB_MotorControl(__FUNCTION__, __LINE__, buff_lrg);
	}

	return pass;
}

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotor(int dt)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Set blocking and time
	FC.is_MotBlocking = true;

	// Update time to hold till
	t_blockMoter = millis() + dt;

	// Remove all motor controll
	if (!SetMotorControl(MC_CON::ID::HOLD, MC_CALL::ID::BLOCKER)) {

		// Log warning
		Debug.DB_Error(__FUNCTION__, __LINE__, "FAILED TO SET MOTOR CONTROL TO \"HOLD\"");

		// Bail
		return;
	}

	// Format message
	Debug.sprintf_safe(buffLrg, buff_lrg, "Blocking Motor for %lu ms", dt);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
}

// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	bool is_block_done = false;
	bool is_passed_feeder = false;
	bool is_mot_running = false;

	// Bail if not blocking
	if (!FC.is_MotBlocking) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Flag block time ellapsed
	is_block_done = millis() > t_blockMoter;

	// Check if rat passed feeder 
	is_passed_feeder =
		FC.is_TrackingEnabled &&
		kal.RatPos - (kal.RobPos + feedTrackPastDist) > 0 &&
		Pos[0].posCum - (kal.RobPos + feedTrackPastDist) > 0 &&
		Pos[2].posCum - (kal.RobPos + feedTrackPastDist) > 0;

	// Check if motor already running again
	is_mot_running = runSpeedNow > 0;

	// Bail if still blocking
	if (!(is_block_done || is_passed_feeder || is_mot_running)) {
		return;
	}

	// Bail if ir prox still triggered
	if (digitalRead(pin.IRPROX_R) == LOW ||
		digitalRead(pin.IRPROX_L) == LOW) {
		return;
	}

	// Log
	if (is_block_done) {
		Debug.DB_General(__FUNCTION__, __LINE__, "Finished Blocking Motor");
	}
	else if (is_passed_feeder) {
		Debug.DB_General(__FUNCTION__, __LINE__, "Unblocking Early: Rat Passed Feeder");
	}
	else if (is_mot_running) {
		Debug.DB_Warning(__FUNCTION__, __LINE__, "Unblocking Early: Motor Started Early");
	}

	// Set flag to stop checking
	FC.is_MotBlocking = false;

	// Retract arm early if extended
	if ((Reward.do_ExtendArm || Reward.isArmExtended) &&
		(is_passed_feeder || is_mot_running)) {
		Reward.RetractFeedArm();
	}

	// Unset control from "HOLD"
	if (motorControlNow == MC_CON::ID::HOLD) {

		// Set motor control to "OPEN"
		if (FC.is_TrackingEnabled) {
			SetMotorControl(MC_CON::ID::OPEN, MC_CALL::ID::BLOCKER);
		}
		// Set motor control to "HOLD"
		else {
			SetMotorControl(MC_CON::ID::HOLD, MC_CALL::ID::BLOCKER);
		}
	}

}

// GET AUTODRIVER BOARD STATUS
int GetAD_Status(uint16_t stat_reg, char *p_status_name)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	VEC<byte> bit_ind(2, __LINE__);
	VEC<bool> is_bit_set(2, __LINE__);
	uint16_t bit_val = 0x0;

	// Get id ind
	for (int i = 0; i < 16; i++)
	{
		if (strcmp(p_status_name, str_med_statusList[i]) == 0)
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

// DO SETUP TO BEGIN TRACKING
void InitializeTracking()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	int n_laps = 0;
	double cm_diff = 0;
	double cm_dist = 0;

	// Bail if finished or task done
	if (FC.is_TrackingEnabled || FC.is_TaskDone) {
		return;
	}

	// Wait for new data
	if (!FC.is_RatOnTrack ||
		!Pos[0].isNew ||
		!Pos[2].isNew ||
		!Pos[1].isNew) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Log/Print
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Initialize Rat Tracking...");

	// Check that pos values make sense
	cm_diff = Pos[0].posCum - Pos[1].posCum;
	cm_dist = min((140 * PI) - abs(cm_diff), abs(cm_diff));

	// Log rat and robot starting pos
	Debug.sprintf_safe(buffLrg, buff_lrg, "Starting Positions: pos(abs|rel|laps) rat_vt=%0.2f|%0.2f|%d rat_pixy=%0.2f|%0.2f|%d rob_vt=%0.2f|%0.2f|%d rat_dist=%0.2f",
		Pos[0].posCum, Pos[0].posAbs, Pos[0].nLaps, Pos[2].posCum, Pos[2].posAbs, Pos[2].nLaps, Pos[1].posCum, Pos[1].posAbs, Pos[1].nLaps, cm_diff);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Rat should be ahead of robot and by no more than 90 deg
	if (cm_diff < 0 ||
		cm_dist >((140 * PI) / 4))
	{
		// Log error
		Debug.DB_Error(__FUNCTION__, __LINE__, "RESETTING POSITION DATA DUE TO BAD VALUES");

		// Will have to run again with new samples
		Pos[0].PosReset(true);
		Pos[2].PosReset(true);
		Pos[1].PosReset(true);

		// Bail
		return;
	}

	// Set flag
	FC.is_TrackingEnabled = true;

	// Reset ekf
	Pid.PidResetEKF();

	// Don't start pid for manual or forage sessions
	if (!FC.is_ManualSes &&
		!FC.is_ForageTask)
	{
		// Open up motor control
		if (!SetMotorControl(MC_CON::ID::OPEN, MC_CALL::ID::SETUP_TRACKING)) {

			// Log error
			Debug.DB_Error(__FUNCTION__, __LINE__, "FAILED TO SET MOTOR CONTROL TO \"Open\"");
		}

		// Run Pid
		Pid.PidRun();
		Debug.DB_General(__FUNCTION__, __LINE__, "PID STARTED");
	}

	// Initialize bulldoze
	if (FC.do_Bulldoze)
	{
		// Run from initial blocked mode
		Bull.BullOn();
		Debug.DB_General(__FUNCTION__, __LINE__, "BULLDOZE INITIALIZED");
	}

	// Do status blink
	StatusBlink(true, 5, 100, true);

	// Log/Print
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Initialize Rat Tracking");
}

// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static uint32_t t_swap_vt = 0;
	static uint32_t t_swap_pixy = 0;
	bool do_swap_vt = false;
	bool do_swap_pixy = false;
	int dt_max_vt = 150;
	int dt_max_pixy = 100;
	int dt_vt = 0;
	int dt_pixy = 0;

	// Bail if doing pos debug
	if (Debug.flag.do_posDebug) {
		return;
	}

	// Bail if rat not on track or task done
	if (!FC.is_RatOnTrack || FC.is_TaskDone) {
		return;
	}

	// Bail if streaming not started
	if (!Pos[0].is_streamStarted ||
		!Pos[2].is_streamStarted) {
		return;
	}

	// Compute dt
	dt_vt = millis() - Pos[0].t_update;
	dt_pixy = millis() - Pos[2].t_update;

	// Check VT 
	do_swap_vt =
		!Pos[0].isNew &&
		dt_vt > dt_max_vt &&
		dt_pixy < dt_vt;

	// Check pixy
	do_swap_pixy =
		!Pos[2].isNew &&
		dt_pixy > dt_max_pixy &&
		dt_vt < dt_pixy;

	// Bail if all good
	if (!do_swap_vt && !do_swap_pixy) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Use Pixy for VT data
	if (do_swap_vt && !do_swap_pixy) {

		// Incriment count
		Pos[0].cnt_swap++;

		// Log if > 1 sec sinse last swap
		if (millis() - t_swap_vt > 1000) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "Swapped VT for Pixy");
		}

		// Swap
		Pos[0].SwapPos(Pos[2].posAbs, Pos[2].t_update);
		t_swap_vt = millis();
	}

	// Use VT for Pixy data
	if (do_swap_pixy && !do_swap_vt) {

		// Incriment count
		Pos[2].cnt_swap++;

		// Log if > 1 sec sinse last swap
		if (millis() - t_swap_pixy > 1000) {
			Debug.sprintf_safe(buffLrg, buff_lrg, "Swapped Pixy for VT");
		}

		// Swap
		Pos[2].SwapPos(Pos[0].posAbs, Pos[0].t_update);
		t_swap_pixy = millis();
	}

	// Log warning 
	if (buff_lrg[0] != '\0') {
		Debug.sprintf_safe(buffLrg, buff_lrg_2, "%s: cnt=|vt=%d|px=%d| dt=|vt=%d|px=%d| pos=|vt=%0.2f|px=%0.2f|",
			buff_lrg, Pos[0].cnt_swap, Pos[2].cnt_swap, dt_vt, dt_pixy, Pos[0].posAbs, Pos[2].posAbs);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg_2);
	}

}

// UPDATE EKF
void UpdateEKF()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static bool is_nans_last = false;
	double rat_pos = 0;
	double rob_pos = 0;
	double rat_vel = 0;
	double rob_vel = 0;

	// Bail if all data not new
	if (
		!Pos[0].isNew ||
		!Pos[2].isNew ||
		!Pos[1].isNew
		) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Check EKF progress
	Pid.PidCheckEKF(millis());

	// Set pid update time
	Pid.PidSetUpdateTime(millis());

	// Set flag for reward
	if (FC.is_TrackingEnabled) {
		Reward.is_ekfNew = true;
	}

	//----------UPDATE EKF---------
	double z[M] = {
		Pos[0].GetPos(),
		Pos[2].GetPos(),
		Pos[1].GetPos(),
		Pos[0].GetVel(),
		Pos[2].GetVel(),
		Pos[1].GetVel(),
	};

	// Run EKF
	ekf.step(z);

	// Update error estimate
	rat_pos = ekf.getX(0);
	rob_pos = ekf.getX(1);
	rat_vel = ekf.getX(2);
	rob_vel = ekf.getX(3);

	// Copy over values
	kal.RatPos = !isnan(rat_pos) ? rat_pos : kal.RatPos;
	kal.RobPos = !isnan(rob_pos) ? rob_pos : kal.RobPos;
	kal.RatVel = !isnan(rat_vel) ? rat_vel : kal.RatVel;
	kal.RobVel = !isnan(rob_vel) ? rob_vel : kal.RobVel;

	// Update count and time
	kal.t_last = millis();
	kal.cnt_ekf++;

	// Check for nan values
	if (isnan(rat_pos) || isnan(rob_pos) || isnan(rat_vel) || isnan(rob_vel)) {

		// Do not print consecutively
		if (!is_nans_last)
		{
			Debug.sprintf_safe(buffLrg, buff_lrg, "\"nan\" EKF Output: Pos[0]=%0.2f|%0.2f Pos[2]=%0.2f|%0.2f Pos[1]=%0.2f|%0.2f",
				Pos[0].posCum, Pos[0].velNow, Pos[2].posCum, Pos[2].velNow, Pos[1].posCum, Pos[0].velNow);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg, true);

			// Set flag
			is_nans_last = true;
		}
	}
	else {
		is_nans_last = false;
	}

	// Check if too much time elapsed between updates
	if (millis() > kal.t_last + 250 &&
		kal.t_last != 0) {

		// Add to count
		cnt_errEKF++;

		// Log first and every 10 errors
		if (cnt_errEKF == 1 ||
			cnt_errEKF % 10 == 0) {

			// Log warning
			Debug.sprintf_safe(buffLrg, buff_lrg, "EKF Hanging: cnt_err=%d dt_update=%d", cnt_errEKF, millis() - kal.t_last);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
		}
	}

}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// CHECK IR DETECTOR
void IR_SyncCheck()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Check for new event
	if (!v_isNewIR) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Only run after sync setup
	if (t_sync != 0 &&
		!FC.do_BlockDetectIR) {

		// Log event if streaming started
		Debug.sprintf_safe(buffLrg, buff_lrg, "IR Sync Event: dt=%dms", v_dt_ir);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, v_t_irSyncLast);

	}

	// Reset flag
	v_isNewIR = false;

}

// CHECK FOR BUTTON INPUT
bool GetButtonInput()
{

	// Notes
	/*
	BUTTON 1:
	Short Hold: Trigger reward or retract feeder arm
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
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	const int _arr_pin_btn[3] = { pin.BTN_1, pin.BTN_2, pin.BTN_3 };
	const VEC<int> arr_pin_btn(3, __LINE__, _arr_pin_btn);
	const int _dt_debounce[3] = { 100, 100, 100 };
	const VEC<int> dt_debounce(3, __LINE__, _dt_debounce);
	bool is_new_input = false;
	static VEC<bool> is_pressed(3, __LINE__);
	static VEC<bool> is_running(3, __LINE__);
	VEC<bool> do_flag_fun_shold(3, __LINE__);
	VEC<bool> do_flag_fun_lhold(3, __LINE__);
	static VEC<uint32_t> t_debounce(3, __LINE__);
	static VEC<uint32_t> t_hold_min(3, __LINE__);
	static VEC<uint32_t> t_long_hold(3, __LINE__);
	int dt_hold_min = 50;
	int dt_long_hold = 500;
	bool do_check = false;

	// Bail if nothing to do
	for (int i = 0; i < 3; i++) {

		do_check = do_check ||
			digitalRead(arr_pin_btn[i]) == LOW ||
			is_pressed[i] ||
			is_running[i];
	}
	if (!do_check) {
		return false;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Loop through and check each button
	for (int i = 0; i < 3; i++) {

		// Detect press
		if (
			digitalRead(arr_pin_btn[i]) == LOW &&
			!is_pressed[i])
		{
			// Exit if < debounce time has not passed
			if (t_debounce[i] > millis()) {
				return false;
			}

			// Log
			Debug.sprintf_safe(buffLrg, buff_lrg, "Pressed button %d", i);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Get long hold time
			t_long_hold[i] = millis() + dt_long_hold - dt_hold_min;

			// Set flag
			is_pressed[i] = true;
		}

		// Check released
		else if (
			is_pressed[i] &&
			!is_running[i])
		{

			// Bail if not dt hold min
			t_hold_min[i] = t_hold_min[i] == 0 ? millis() + dt_hold_min : t_hold_min[i];
			if (millis() < t_hold_min[i]) {
				return false;
			}

			// Check for short hold
			bool is_short_hold =
				digitalRead(arr_pin_btn[i]) == HIGH &&
				millis() < t_long_hold[i];

			// Check for long hold
			bool is_long_hold = millis() > t_long_hold[i];

			// Set flag for either condition
			if (is_short_hold || is_long_hold) {

				// Log
				Debug.sprintf_safe(buffLrg, buff_lrg, "Triggered button %d", i);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				// Run short hold function
				if (is_short_hold) {
					do_flag_fun_shold[i] = true;
				}

				// Run long hold function
				if (is_long_hold) {
					do_flag_fun_lhold[i] = true;
				}

				// Make tracker LED brighter
				analogWrite(pin.LED_TRACKER, 255);

				// Set running flag
				is_running[i] = true;

				// Flag input rcvd
				is_new_input = true;
			}
		}

		// Check if needs to be reset
		else if (digitalRead(arr_pin_btn[i]) == HIGH &&
			is_pressed[i]) {

			if (is_running[i] ||
				millis() > t_long_hold[i]) {

				// Log
				Debug.sprintf_safe(buffLrg, buff_lrg, "Reset button %d", i);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				// Reset flags etc
				t_debounce[i] = millis() + dt_debounce[i];
				analogWrite(pin.LED_TRACKER, trackLEDduty[1]);
				is_running[i] = false;
				t_hold_min[i] = 0;
				t_long_hold[i] = 0;
				is_pressed[i] = false;
			}
		}

	}

	// Bail if no new flags
	if (!is_new_input) {
		return false;
	}

	// Set button 1 function flag
	if (do_flag_fun_shold[0]) {

		// Reward or retract feeder arm
		if (!Reward.isArmExtended) {
			FC.do_BtnRew = true;
			Debug.DB_General(__FUNCTION__, __LINE__, "Button 1 \"FC.doBtnRew\" Triggered");
		}
		else {
			Reward.RetractFeedArm();
			Debug.DB_General(__FUNCTION__, __LINE__, "Button 1 \"Reward.RetractFeedArm()\" Triggered");
		}
	}
	else if (do_flag_fun_lhold[0]) {
		FC.do_MoveRobFwd = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "Button 1 \"FC.doMoveRobFwd\" Triggered");
	}

	// Set button 2 function flag
	else if (do_flag_fun_shold[1]) {
		FC.do_RewSolStateChange = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "Button 2 \"FC.doRewSolStateChange\" Triggered");
	}
	else if (do_flag_fun_lhold[1]) {
		FC.do_MoveRobRev = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "Button 2 \"FC.doMoveRobRev\" Triggered");
	}

	// Set button 3 function flag
	else if (do_flag_fun_shold[2]) {
		FC.do_EtOHSolStateChange = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "Button 3 \"FC.doEtOHSolStateChange\" Triggered");
	}
	else if (do_flag_fun_lhold[2]) {
		FC.do_ChangeLCDstate = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "Button 3 \"FC.doChangeLCDstate\" Triggered");
	}

	// Turn on LCD if any fucntion flagged durring manual mode
	if (FC.is_ManualSes && !FC.do_ChangeLCDstate) {
		// Turn on LCD LED
		ChangeLCDlight(8);
	}

	// Return flag
	return true;
}

// PROCESS BUTTON INPUT
void ProcButtonInput()
{

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// REWARD
	if (FC.do_BtnRew) {
		// Bail if already rewarding
		if (Reward.isRewarding) {
			Debug.DB_Warning(__FUNCTION__, __LINE__, "ABORTED: TRIGGERED DURING ONGOING REWARD");
		}
		else {
			// Process button command
			Reward.ProcRewCmd(0);
			Reward.StartRew();
		}
		FC.do_BtnRew = false;
	}

	// OPEN/CLOSE REW SOL
	if (FC.do_RewSolStateChange) {
		OpenCloseRewSolenoid();
		FC.do_RewSolStateChange = false;
	}

	// OPEN/CLOSE ETOH SOL
	if (FC.do_EtOHSolStateChange) {
		OpenCloseEtOHSolenoid();
		FC.do_EtOHSolStateChange = false;
	}

	// TURN LCD LED ON/OFF
	if (FC.do_ChangeLCDstate) {
		ChangeLCDlight();
		FC.do_ChangeLCDstate = false;
	}

	// MOVE ROBOT
	if (FC.do_MoveRobFwd) {
		ManualRun('f');
		FC.do_MoveRobFwd = false;
	}
	else if (FC.do_MoveRobRev) {
		ManualRun('r');
		FC.do_MoveRobRev = false;
	}

}

// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	byte is_sol_open = digitalRead(pin.REL_FOOD);
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';

	// Change state
	is_sol_open = !is_sol_open;

	// Store open time
	if (is_sol_open) {
		t_solOpen = millis();
	}
	else {
		t_solClose = millis();
	}

	// Open/close solenoid
	digitalWrite(pin.REL_FOOD, is_sol_open);

	// Print etoh and rew sol state to LCD
	Debug.sprintf_safe(buffMed, buff_med_1, "Food   %s", digitalRead(pin.REL_FOOD) == HIGH ? "OPEN  " : "CLOSED");
	Debug.sprintf_safe(buffMed, buff_med_2, "EtOH   %s", digitalRead(pin.REL_ETOH) == HIGH ? "OPEN  " : "CLOSED");
	Debug.PrintLCD(true, buff_med_1, buff_med_2, 's');

}

// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	byte is_sol_open = digitalRead(pin.REL_ETOH);
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';

	// Change state
	is_sol_open = !is_sol_open;

	// Open/close solenoid
	digitalWrite(pin.REL_ETOH, is_sol_open);

	// Store time and make sure periodic drip does not run
	if (is_sol_open) {
		t_solOpen = millis();
		FC.do_EtOHRun = false;
	}
	else {
		t_solClose = millis();
		FC.do_EtOHRun = true;
	}

	// Print etoh and rew sol state to LCD
	Debug.sprintf_safe(buffMed, buff_med_1, "Food   %s", digitalRead(pin.REL_FOOD) == HIGH ? "OPEN  " : "CLOSED");
	Debug.sprintf_safe(buffMed, buff_med_2, "EtOH   %s", digitalRead(pin.REL_ETOH) == HIGH ? "OPEN  " : "CLOSED");
	Debug.PrintLCD(true, buff_med_1, buff_med_2, 's');

}

// CHECK FOR ETOH UPDATE
void CheckEtOH()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	byte is_sol_open = digitalRead(pin.REL_ETOH);
	int dt_open = 0;
	int dt_close = 0;
	bool do_open = false;
	bool do_close = false;

	// Bail if etoh should not be run
	if (!FC.do_EtOHRun) {
		return;
	}

	// Check if should be opened
	do_open = !is_sol_open &&
		millis() > (t_solOpen + dt_delEtOH[FC.is_SesStarted ? 0 : 1]);

	// Open only if motor not running
	if (do_open) {

		if (runSpeedNow > 0) {
			// Reset flag
			do_open = false;
		}
	}

	// Close if open and dt close has ellapsed
	do_close = is_sol_open &&
		millis() > (t_solOpen + dt_durEtOH[FC.is_SesStarted ? 0 : 1]);

	// Bail if nothing to do
	if (!do_open && !do_close) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Check if sol should be opened
	if (do_open &&
		GetAD_Status(adR_stat, "AD_STAT") == 0 &&
		GetAD_Status(adF_stat, "AD_STAT") == 0) {

		// Open solenoid
		digitalWrite(pin.REL_ETOH, HIGH);

		// Store current time and pos
		t_solOpen = millis();

		// Compute dt
		dt_close = t_solClose > 0 ? t_solOpen - t_solClose : 0;

		// Print to debug
		Debug.sprintf_safe(buffLrg, buff_lrg, "Open EtOH: dt_close=%d", dt_close);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Check if sol should be closed
	else if (do_close) {

		// Close solenoid
		digitalWrite(pin.REL_ETOH, LOW);

		// Store current time 
		t_solClose = millis();

		// Compute dt
		dt_open = t_solOpen > 0 ? t_solClose - t_solOpen : 0;

		// Print to debug
		Debug.sprintf_safe(buffLrg, buff_lrg, "Close EtOH: dt_open=%d", dt_open);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}
}

// CHECK BATTERY VALUES
float CheckBattery(bool force_check)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	static VEC<float> vcc_shutdown_arr(10, __LINE__);
	static uint32_t t_vcc_update = 0;
	static uint32_t t_vcc_send = 0;
	static uint32_t t_vcc_print = 0;
	static uint32_t t_update_start = 0;
	static uint32_t t_relay_ready = 0;
	static float vcc_avg = 0;
	static float vcc_last = 0;
	static float vcc_out = 0;
	static int cnt_samples = 0;
	uint32_t vcc_bit_in = 0;
	float vcc_sum = 0;
	bool is_mot_off = false;
	byte do_shutdown = false;

	// Set out to avg if enough samples collected
	vcc_out = cnt_samples < vccMaxSamp ? 0 : vccAvg;

	// Check for forced check
	if (!force_check) {

		// Not time to check
		if (millis() < t_update_start) {
			// Bail
			return vcc_out;
		}

		// Done checking
		else if (cnt_samples >= vccMaxSamp) {

			// Compute next check time
			t_update_start = millis() + dt_vccUpdate;

			// Reset samples
			cnt_samples = 0;

			// Turn off switch and bail
			digitalWrite(pin.REL_VCC, LOW);
			return vcc_out;
		}
	}

	// Bail if run speed > 0
	if (runSpeedNow > 0) {

		// Turn off switch and bail
		digitalWrite(pin.REL_VCC, LOW);
		return vcc_out;

	}

	// Turn on relay and skip this run to alow switch to open
	if (digitalRead(pin.REL_VCC) == LOW) {

		// Turn on switch 
		digitalWrite(pin.REL_VCC, HIGH);

		// Allow time for switch to activate
		t_relay_ready = millis() + 10;

		// Bail
		return vcc_out;
	}

	// Bail if relay not ready
	if (millis() < t_relay_ready) {
		return vcc_out;
	}


#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Calculate voltage
	vcc_bit_in = analogRead(pin.BAT_VCC);
	vccNow = (float)vcc_bit_in * bit2volt;
	vcc_sum = 0;

	// Shift array and compute average
	for (int i = 0; i < vccMaxSamp - 1; i++) {
		vccArr[i] = vccArr[i + 1];
		vcc_sum += vccArr[i];
	}
	vccArr[vccMaxSamp - 1] = vccNow;
	vcc_avg = vcc_sum / vccMaxSamp;


	// Bail till array full
	cnt_samples = cnt_samples < vccMaxSamp ? cnt_samples + 1 : vccMaxSamp;
	if (cnt_samples < vccMaxSamp) {
		return vcc_out;
	}
	else {
		vcc_out = vcc_avg;
	}

	// Store new voltage level
	vcc_last = vccAvg;
	vccAvg = vcc_avg;

	// Keep a list of averages to check for shutdown
	for (int i = 0; i < 9; i++) {
		vcc_shutdown_arr[i] = vcc_shutdown_arr[i + 1];
	}
	vcc_shutdown_arr[9] = vccAvg;

	// Send if min dt ellapsed
	if (
		FC.do_SendVCC &&
		millis() > t_vcc_send + dt_vccSend) {

		// Send vcc
		QueuePacket(&r2c, 'J', vccAvg, 0, 0, 0, true);

		// Store time
		t_vcc_send = millis();
	}

	// Log voltage 
	if (millis() > t_vcc_print + dt_vccPrint) {

		// Log voltage
		Debug.sprintf_safe(buffLrg, buff_lrg, "Battery VCC: vcc=%0.2fV dt_chk=%d",
			vccAvg, millis() - t_vcc_update);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Print to lcd
		Debug.sprintf_safe(buffMed, buff_med, "VCC=%0.2fV", vccAvg);
		Debug.PrintLCD(false, buff_med);

		// Store time
		t_vcc_print = millis();

		// Check if voltage critically low
		do_shutdown = true;
		for (int i = 0; i < 10 - 1; i++) {
			do_shutdown = do_shutdown && vcc_shutdown_arr[i] < vccCutoff && vcc_shutdown_arr[i] > 0 ?
				true : false;
		}

		// Perform shutdown
		if (do_shutdown) {

			// Format messege
			Debug.sprintf_safe(buffLrg, buff_lrg, "BATTERY CRITICALLY LOW AT < %0.2fv", vccCutoff);

			// Run error hold then shutdown after 5 min
			Debug.sprintf_safe(buffMed, buff_med, "VCC LOW %0.2fV", vccAvg);
			Debug.RunErrorHold(buff_lrg, buff_med, dt_vccShutDown);
		}
	}

	// Store time
	t_vcc_update = millis();

	// Return battery voltage
	return vcc_out;
}

// TURN LCD LIGHT ON/OFF
void ChangeLCDlight(uint32_t duty)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Check if new duty given
	if (duty == 256) {
		FC.is_LitLCD = !FC.is_LitLCD;
		duty = FC.is_LitLCD ? 8 : 0;
	}
	else {
		FC.is_LitLCD = duty > 0;
	}

	// Log
	Debug.sprintf_safe(buffLrg, buff_lrg, "Set LCD Light: is_lit=%d duty=%d",
		FC.is_LitLCD, duty);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Set LCD duty
	analogWrite(pin.LCD_LED, duty);
}

// QUIT AND RESTART ARDUINO
void QuitSession()
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Bail if anything in queues
	if (SendPacket(&r2a) ||
		SendPacket(&r2c) ||
		CheckResend(&r2a) ||
		CheckResend(&r2c) ||
		Debug.Print()) {

		return;
	}

	// Tell CS quit is done
	if (!FC.is_QuitConfirmed) {
		Debug.DB_General(__FUNCTION__, __LINE__, "Sending 'Q' Done Confirmation");
		QueuePacket(&r2c, 'Q', 0, 0, 0, c2r.packArr[ID_Ind<R4_COM<USARTClass>>('Q', &c2r)], true, false, true);
		FC.is_QuitConfirmed = true;
		return;
	}

	// Set quit time 100 ms
	if (t_quit == 0) {
		t_quit = millis() + 100;
		return;
	}

	// Bail if time not ellapsed
	else if (millis() < t_quit) {
		return;
	}

	// Restart
	RestartArduino();

}

// RESTART ARDUINO
void RestartArduino()
{

	// Log quiting
	Debug.DB_General(__FUNCTION__, __LINE__, "QUITING...");

	// Stop all movement
	HardStop(__FUNCTION__, __LINE__);
	Pid.PidStop();
	Bull.BullOff();

	// Log anything left in queue
	Log.WriteAll(1000);
	Debug.PrintAll(1000);

	// Wait 1 sec for any backround stuff to finish
	delay(1000);

	// Restart Arduino
	REQUEST_EXTERNAL_RESET;

}


#pragma endregion


#pragma region --------TESTING---------

// UPDATE TESTS
void TestUpdate()
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';
	static VEC<double> pos_last(3, __LINE__);
	static bool is_halted = false;
	static int32_t t_check_next = 0;
	static int32_t dt_ir = 0;
	int dt_check = 250;
	bool is_new = false;
	int dt_vt = 0;
	int dt_pixy = 0;

	// Bail if not doing any testing
	if (!Debug.flag.do_systemTesting && !Debug.flag.do_manualTesting) {
		return;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Graph pin state
	if (Debug.flag.do_digitalpinGraph) {
		/*
		millis()%10 == 0
		{@ReportDigital}
		*/

	}
	if (Debug.flag.do_analogpinGraph) {
		/*
		millis()%10 == 0
		{@ReportAnalog}
		*/

	}

	// Print IR detector dt
	if (Debug.flag.do_irTimePrint) {

		// Check for new ir
		if (v_isNewIR) {

			// Copy ir dt
			dt_ir = v_dt_ir;

			// Print new ir time
			Debug.sprintf_safe(buffLrg, buff_lrg, "NEW IR DT: %dms", dt_ir);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		}
	}

	// Run position debugging
	if (Debug.flag.do_posDebug)
	{

		// Block motor
		if (Move.cnt_move == 1 &&
			!FC.do_RunMove &&
			!FC.is_TaskDone &&
			!is_halted) {

			// Halt robot
			HardStop(__FUNCTION__, __LINE__);
			SetMotorControl(MC_CON::ID::HALT, MC_CALL::ID::OVERIDE);
			is_halted = true;
		}

		// Unblock motor
		if (FC.is_TaskDone && is_halted) {

			// Unhalt robot
			SetMotorControl(MC_CON::ID::OPEN, MC_CALL::ID::OVERIDE);
			is_halted = false;
		}

		// Make sure LCD light on 
		if (!FC.is_LitLCD) {
			ChangeLCDlight(8);
		}

		// Check if position values changed
		for (int i = 0; i < 3; i++) {
			if (Pos[i].posCum != pos_last[i]) {
				is_new = true;
			}
			pos_last[i] = Pos[i].posCum;
		}

		// Plot and print new data
		if (is_new && millis() > t_check_next)
		{

			// Update check time 
			t_check_next = millis() + dt_check;

			// Plot pos
			if (Debug.flag.do_posPlot) {
				/*
				{@Plot.WinPos.PosRatVT.Blue Pos[0].posRel}{@Plot.WinPos.PosRatPixy.Green Pos[2].posRel}{@Plot.WinPos.PosRobVT.Orange Pos[1].posRel}{@Plot.WinPos.RatEKF.Black kal.RatPos}{@Plot.WinPos.RobEKF.Red kal.RobPos}
				*/

			}

			// Turn on rew led when near setpoint
			if (Pid.error > -0.5 && Pid.error < 0.5 &&
				Pos[0].is_streamStarted &&
				Pos[1].is_streamStarted) {

				analogWrite(pin.LED_REW_C, 10);
			}
			else {
				analogWrite(pin.LED_REW_C, rewLEDduty[0]);
			}

			// Compute distances
			double rat_vt_dist = Pos[0].posCum - Pos[1].posCum;
			double rat_pixy_dist = Pos[2].posCum - Pos[1].posCum;

			// Print pos data
			if (Debug.flag.do_posPrint) {
				Debug.sprintf_safe(buffLrg, buff_lrg, "POS DEBUG (abs|rel|laps|dist): rat_vt=%0.2f|%0.2f|%d|%0.2f rat_pixy=%0.2f|%0.2f|%d|%0.2f rob_vt=%0.2f|%0.2f|%d",
					Pos[0].posCum, Pos[0].posAbs, Pos[0].nLaps, rat_vt_dist, Pos[2].posCum, Pos[2].posAbs, Pos[2].nLaps, rat_pixy_dist, Pos[1].posCum, Pos[1].posAbs, Pos[1].nLaps);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
			}

			// Compute dt sinse last record
			dt_vt = min(999, millis() - Pos[0].t_update);
			dt_pixy = min(999, millis() - Pos[2].t_update);

			// Display rat vt dist
			Debug.sprintf_safe(buffMed, buff_med_1, "VT %d %d", (int)rat_vt_dist, dt_vt);
			Debug.sprintf_safe(buffMed, buff_med_2, "PX %d %d", (int)rat_pixy_dist, dt_pixy);
			Debug.PrintLCD(true, buff_med_1, buff_med_2);

		}
	}

	// Run Pid calibration
	if (Debug.flag.do_pidCalibration)
	{
		double new_speed = Pid.PidCalibration();

		// Run motors
		if (Pid.cal_isPidUpdated)
		{
			if (new_speed >= 0) {
				RunMotor('f', new_speed, MC_CALL::ID::OVERIDE);
			}
			// Print values
			/*
			millis()%50 == 0
			{Pid.cal_isCalFinished}{"SPEED"}{Pid.cal_ratVel}{"ERROR"}{Pid.cal_errNow}{Pid.cal_errArr[0]}{Pid.cal_errArr[1]}{Pid.cal_errArr[2]}{Pid.cal_errArr[3]}{"PERIOD"}{Pid.cal_PcNow}{Pid.cal_cntPcArr[0]}{Pid.cal_PcArr[0]}{Pid.cal_cntPcArr[1]}{Pid.cal_PcArr[1]}{Pid.cal_cntPcArr[2]}{Pid.cal_PcArr[2]}{Pid.cal_cntPcArr[3]}{Pid.cal_PcArr[3]}{Pid.cal_PcAll}
			*/

			// Plot error
			/*
			{@Plot.WinPID.Error.Red Pid.cal_errNow} {@Plot.WinPID.Setpoint.Black 0}
			*/

			// Reset flag
			Pid.cal_isPidUpdated = false;
		}
	}

	// Run IR sync time
	if (Debug.flag.do_v_irSyncTest) {

		// Set "TEST_SIGNAL" to LOW after 50ms
		if (digitalRead(pin.TEST_SIGNAL) == HIGH &&
			millis() - v_t_irSyncLast > 10) {
			digitalWrite(pin.TEST_SIGNAL, LOW);
		}

	}

}

// DO HARDWARE TEST
void HardwareTest(bool do_stress_test, bool do_pixy_test, bool do_ping_test)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// ------------------------ LOCAL VARS ------------------------

	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static char buff_lrg_3[buffLrg] = { 0 }; buff_lrg_3[0] = '\0';
	static char buff_lrg_4[buffLrg] = { 0 }; buff_lrg_4[0] = '\0';
	static char buff_lrg_5[buffLrg] = { 0 }; buff_lrg_5[0] = '\0';
	static char buff_lrg_6[buffLrg] = { 0 }; buff_lrg_6[0] = '\0';
	static char buff_lrg_7[buffLrg] = { 0 }; buff_lrg_7[0] = '\0';
	static char buff_lrg_8[buffLrg] = { 0 }; buff_lrg_8[0] = '\0';
	const int dt_timeout = 120000;
	uint32_t t_test_start = 0;

	// Stress test
	const byte n_stress_samp = 10;
	const double _speed_range[2] = { 5,65 };
	VEC<double> speed_range(2, __LINE__, _speed_range);
	byte speed_step = (speed_range[1] - speed_range[0]) / (n_stress_samp / 2);
	byte s_now = speed_range[0];
	byte cnt_stress = 0;
	uint32_t t_stress_run = 0;
	uint32_t dt_stress_run = dt_timeout / n_stress_samp;
	uint32_t dt_close_sol = 100;
	VEC<double> run_speed(n_stress_samp, __LINE__);
	bool is_stressin = false;
	float vcc_baseline = 0;
	VEC<float> vcc_arr(n_stress_samp, __LINE__);
	float vcc_sum = 0;
	float vcc_avg = 0;
	for (int i = 0; i < n_stress_samp; i++) {
		run_speed[i] = i % 2 == 0 ? s_now += speed_step : s_now;
	}
	uint32_t t_lcd_start = 0;
	uint32_t t_print_start = 0;
	uint32_t t_log_start = 0;
	VEC<uint32_t> lcd_arr(n_stress_samp, __LINE__);
	uint32_t lcd_sum = 0;
	double lcd_avg = 0;
	VEC<uint32_t> print_arr(n_stress_samp, __LINE__);
	uint32_t print_sum = 0;
	double print_avg = 0;
	VEC<uint32_t> log_arr(n_stress_samp, __LINE__);
	uint32_t log_sum = 0;
	double log_avg = 0;
	bool is_stress_test_done = false;

	// Pixy test
	const byte n_pixy_samp = 10;
	uint32_t t_pixy_check = 0;
	uint32_t dt_pixy_check = dt_timeout / n_pixy_samp / 4;
	byte cnt_pixy = 0;
	bool is_pixy_led_on = false;
	VEC<double> pixy_pos_arr(n_pixy_samp, __LINE__);
	double pixy_pos_sum = 0;
	double pixy_pos_avg = 0;
	bool is_pixy_test_done = false;

	// Ping test
	VEC<uint16_t> cnt_ping(2, __LINE__);
	bool _do_send_ping[2] = { true, true };
	VEC<bool> do_send_ping(2, __LINE__, _do_send_ping);
	VEC<uint32_t> dt_ping_mat_cs(50, __LINE__);
	VEC<uint32_t> dt_ping_mat_ard(50, __LINE__);
	uint32_t dt_ping_sum = 0;
	byte r2i = 0;
	R2_COM<USARTClass> *p_r2;
	R4_COM<USARTClass> *p_r4;
	bool is_ping_test_done = false;

	// ------------------------ SETUP TEST ------------------------

	// Log
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING HARDWARE TEST...");

	// Set other test flags
	if (!do_stress_test) {
		is_stress_test_done = true;
	}
	if (!do_pixy_test) {
		is_pixy_test_done = true;
	}
	if (!do_ping_test) {
		is_ping_test_done = true;
	}

	// Make sure all data sent
	do {
		GetSerial(&c2r);
	} while (SendPacket(&r2c));
	do {
		GetSerial(&a2r);
	} while (SendPacket(&r2a));

	// Store baseline vcc
	CheckBattery(true);
	vcc_baseline = vccNow;

	// Log all
	Debug.PrintAll(2500);
	Log.WriteAll(2500);

	// Store start time
	t_test_start = millis();

	// Run Test
	while (
		(!is_stress_test_done ||
			!is_pixy_test_done ||
			!is_ping_test_done
			) &&
		millis() < t_test_start + dt_timeout * 2) {

		// ----------------------- STRESS TEST ------------------------
		if (!is_stress_test_done) {

			// Block printing and logging
			FC.do_BlockLogQueue = true;
			FC.do_BlockPrintQueue = true;

			// Extend retract feed arm
			if (!Reward.do_ExtendArm &&
				!Reward.do_RetractArm) {
				Reward.isArmExtended ? Reward.RetractFeedArm() : Reward.ExtendFeedArm(ezRewExtStps);
			}
			Reward.CheckFeedArm();

			// Do next stage
			if (cnt_stress < n_stress_samp &&
				millis() > t_stress_run + dt_stress_run) {

				// Flip state
				is_stressin = !is_stressin;

				// Run motors
				HardStop(__FUNCTION__, __LINE__, true);
				char run_dir = runDirNow == 'f' ? 'r' : 'f';
				RunMotor(run_dir, run_speed[cnt_stress], MC_CALL::ID::OVERIDE);

				// Open Solenoids
				digitalWrite(pin.REL_FOOD, HIGH);
				digitalWrite(pin.REL_ETOH, HIGH);

				// Change LEDS
				analogWrite(pin.LCD_LED, is_stressin ? 255 : 0);
				analogWrite(pin.LED_TRACKER, 255);
				if (!is_pixy_led_on) {
					analogWrite(pin.LED_REW_C, is_stressin ? 255 : 0);
					analogWrite(pin.LED_REW_R, is_stressin ? 255 : 0);
				}

				// Print to LCD
				t_lcd_start = micros();
				Debug.sprintf_safe(buffLrg, buff_lrg, "C=%d S=%0.0f V=%0.0f", cnt_stress + 1, run_speed[cnt_stress], vccNow);
				Debug.PrintLCD(false, buff_lrg);
				lcd_arr[cnt_stress] = micros() - t_lcd_start;

				// Print to console
				FC.do_BlockPrintQueue = false;
				t_print_start = micros();
				Debug.sprintf_safe(buffLrg, buff_lrg, "Stress Test %d: Speed=%0.2f VCC=%0.2f", cnt_stress + 1, run_speed[cnt_stress], vccNow);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
				FC.do_BlockPrintQueue = true;
				Debug.PrintAll(500);
				print_arr[cnt_stress] = micros() - t_print_start;

				// Store log
				FC.do_BlockLogQueue = false;
				t_log_start = micros();
				Debug.sprintf_safe(buffLrg, buff_lrg, "Stress Test %d: Speed=%0.2f VCC=%0.2f", cnt_stress + 1, run_speed[cnt_stress], vccNow);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
				FC.do_BlockLogQueue = true;
				Log.WriteAll(1000);
				log_arr[cnt_stress] = micros() - t_log_start;

				// Store time
				t_stress_run = millis();

				// Store battery voltage
				CheckBattery(true);
				vcc_arr[cnt_stress] = vccNow;

				// Incriment count
				cnt_stress++;
			}

			else if (cnt_stress == n_stress_samp &&
				millis() > t_stress_run + dt_stress_run) {

				// Stop motors
				HardStop(__FUNCTION__, __LINE__, true);

				// Turn all off
				analogWrite(pin.LCD_LED, 0);
				analogWrite(pin.LED_REW_C, 0);
				analogWrite(pin.LED_REW_R, 0);
				analogWrite(pin.LED_TRACKER, 0);
				digitalWrite(pin.REL_FOOD, LOW);
				digitalWrite(pin.REL_ETOH, LOW);

				// Get averages
				for (int i = 0; i < n_stress_samp; i++) {

					// LCD
					lcd_sum += lcd_arr[i];
					Debug.sprintf_safe(buffLrg, buff_lrg, "%0.2f|", (double)lcd_arr[i] / 1000);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_2), buff_lrg_2, strlen(buff_lrg), buff_lrg);

					// Print
					print_sum += print_arr[i];
					Debug.sprintf_safe(buffLrg, buff_lrg, "%0.2f|", (double)print_arr[i] / 1000);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_3), buff_lrg_3, strlen(buff_lrg), buff_lrg);

					// Log
					log_sum += log_arr[i];
					Debug.sprintf_safe(buffLrg, buff_lrg, "%0.2f|", (double)log_arr[i] / 1000);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_4), buff_lrg_4, strlen(buff_lrg), buff_lrg);

					// VCC
					vcc_sum += vcc_arr[i];
					Debug.sprintf_safe(buffLrg, buff_lrg, "%0.2f|", vcc_arr[i]);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_5), buff_lrg_5, strlen(buff_lrg), buff_lrg);
				}

				// Compute average
				lcd_avg = (double)lcd_sum / 1000 / (n_stress_samp);
				print_avg = (double)print_sum / 1000 / (n_stress_samp);
				log_avg = (double)log_sum / 1000 / (n_stress_samp);
				vcc_avg = vcc_sum / (n_stress_samp);

				// Make sure arm retracted
				Reward.RetractFeedArm();

				// Set flag
				is_stress_test_done = true;
			}

			// Close solonoids
			if (millis() > t_stress_run + dt_close_sol) {

				// Close reward sol
				if (digitalRead(pin.REL_FOOD) == HIGH) {
					digitalWrite(pin.REL_FOOD, LOW);
				}

				// Close reward sol
				if (digitalRead(pin.REL_ETOH) == HIGH) {
					digitalWrite(pin.REL_ETOH, LOW);
				}
			}

			// Unblock printing and logging
			FC.do_BlockLogQueue = false;
			FC.do_BlockPrintQueue = false;

			// Check motor status
			AD_CheckOC();
		}


		// ------------------------ PIXY TEST -------------------------
		if (!is_pixy_test_done) {

			// Get new sample
			if (cnt_pixy < n_pixy_samp &&
				millis() > t_pixy_check + dt_pixy_check) {

				// Flip led state
				is_pixy_led_on = !is_pixy_led_on;

				// Turn on/off LED
				analogWrite(pin.LED_REW_R, is_pixy_led_on ? 64 : 0);
				analogWrite(pin.LED_REW_C, 0);
				t_pixy_check = millis();

				// Check pixy and store pos
				if (!is_pixy_led_on) {

					// Store value
					pixy_pos_arr[cnt_pixy] = Pixy.PixyUpdate(true);

					// Incriment count
					cnt_pixy++;
				}
			}

			// Get final average
			else if (cnt_pixy == n_pixy_samp) {

				// Loop samples
				for (int i = 0; i < n_pixy_samp; i++) {

					pixy_pos_sum += pixy_pos_arr[i];
					Debug.sprintf_safe(buffLrg, buff_lrg, "%0.2f|", pixy_pos_arr[i]);
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_6), buff_lrg_6, strlen(buff_lrg), buff_lrg);
				}

				// Compute average
				pixy_pos_avg = pixy_pos_sum / (n_pixy_samp);

				// Set flag
				is_pixy_test_done = true;
			}

		}


		// ----------------------- PING TEST ------------------------
		if (!is_ping_test_done) {

			// Swap target
			p_r2 = r2i == 0 ? &r2c : &r2a;
			p_r4 = r2i == 0 ? &c2r : &a2r;

			// Check for reply
			GetSerial(p_r4);

			// Check resend
			CheckResend(&r2a);
			CheckResend(&r2c);

			// Store round trip time
			if (p_r4->dat[0] == cnt_ping[r2i]) {

				// Store dt send
				if (r2i == 0) {
					dt_ping_mat_cs[cnt_ping[r2i]] = p_r4->t_rcvd - p_r2->t_sent;
				}
				else {
					dt_ping_mat_ard[cnt_ping[r2i]] = p_r4->t_rcvd - p_r2->t_sent;
				}

				// Incriment count
				cnt_ping[r2i]++;

				// Set flag to send next
				do_send_ping[r2i] = cnt_ping[r2i] <= n_testPings;

				// Flip destination
				r2i = r2i == 0 ? 1 : 0;

				// Log all
				Debug.PrintAll(1000);
				Log.WriteAll(1000);

				// Next loop
				continue;
			}

			// Send next p_r2 ping
			if (do_send_ping[r2i]) {

				// Send pack
				float dat1 = cnt_ping[r2i];
				float dat2 = cnt_ping[0] > 0 ? dt_ping_mat_cs[cnt_ping[0] - 1] : 0;
				float dat3 = cnt_ping[1] > 0 ? dt_ping_mat_ard[cnt_ping[1] - 1] : 0;
				QueuePacket(p_r2, 'n', dat1, dat2, dat3, 0, true);

				// Send now
				SendPacket(p_r2);

				// Reset flag
				do_send_ping[r2i] = false;
			}

			// Send any packets
			SendPacket(&r2c);
			SendPacket(&r2a);

			// Check if done
			if (!(cnt_ping[0] <= n_testPings || cnt_ping[1] <= n_testPings)) {
				is_ping_test_done = true;
			}

		}

		// Log all
		Debug.Print();
		Log.WriteLog();
	}

	// ----------------------- FINISH PING TEST ------------------------

	if (do_ping_test) {

		// Compute ping average times
		for (int i = 0; i < 2; i++) {

			// Reset sum
			dt_ping_sum = 0;

			// Loop pings
			for (int j = 0; j < n_testPings; j++) {

				uint32_t dt = i == 0 ? dt_ping_mat_cs[j] : dt_ping_mat_ard[j];
				dt_ping_sum += dt;
				Debug.sprintf_safe(buffLrg, buff_lrg, "%d|", dt);

				// Add to string
				if (i == 0) {
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_7), buff_lrg_7, strlen(buff_lrg), buff_lrg);
				}
				else {
					Debug.strcat_safe(buffLrg, strlen(buff_lrg_8), buff_lrg_8, strlen(buff_lrg), buff_lrg);
				}

			}

			// Compute average
			dt_pingRoundTrip[i] = (float)dt_ping_sum / (n_testPings);
		}

		// Log ping time
		Debug.sprintf_safe(buffLrg, buff_lrg, "R2C PING ROUND TRIP TIME (ms): avg=%0.2f all=|%s", dt_pingRoundTrip[0], buff_lrg_7);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		Debug.sprintf_safe(buffLrg, buff_lrg, "R2A PING ROUND TRIP TIME (ms): avg=%0.2f all=|%s", dt_pingRoundTrip[1], buff_lrg_8);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	}

	// Send final ping times with resend count
	QueuePacket(&r2c, 'n', dt_pingRoundTrip[0], dt_pingRoundTrip[1], (float)(r2c.cnt_repeat + r2a.cnt_repeat), 0, true);
	SendPacket(&r2c);

	// ----------------------- FINISH OTHER TEST ------------------------

	// Log stress test summary
	if (do_stress_test) {

		// Log vcc
		Debug.sprintf_safe(buffLrg, buff_lrg, "VCC: baseline=%0.2f avg=%0.2f all=|%s",
			vcc_baseline, vcc_avg, buff_lrg_5);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Log LCD and Print and log times
		Debug.sprintf_safe(buffLrg, buff_lrg, "LCD PRINT TIME (ms): avg=%0.2f all=|%s", lcd_avg, buff_lrg_2);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		Debug.sprintf_safe(buffLrg, buff_lrg, "CONSOLE PRINT TIME (ms): avg=%0.2f all=|%s", print_avg, buff_lrg_3);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		Debug.sprintf_safe(buffLrg, buff_lrg, "LOG WRITE TIME (ms): avg=%0.2f all=|%s", log_avg, buff_lrg_4);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	}

	// Log pixy test summary
	if (do_pixy_test) {

		// Log pixy pos
		Debug.sprintf_safe(buffLrg, buff_lrg, "PIXY POS (cm): avg=%0.2f all=|%s", pixy_pos_avg, buff_lrg_6);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	}

	// Log
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED HARDWARE TEST");

}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// GET ID INDEX
template <typename R24> int ID_Ind(char id, R24 *p_r24)
{
#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Return -1 if not found
	int ind = -1;
	for (int i = 0; i < p_r24->lng; i++) {

		if (id == p_r24->id[i]) {
			ind = i;
		}
	}

	// Print warning if not found
	if (ind == -1) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "ID \'%c\' Not Found in %s", id, COM::str_list_id[p_r24->comID]);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	return ind;

}

// GET/SET BYTE BIT VALUE
bool GetSetByteBit(byte * b_set, int bit, bool do_set)
{
	// Set bit
	if (do_set) {
		*b_set = *b_set | 0x01 << bit;
	}

	// Return state
	return ((*b_set >> bit) & 0x01) == 1;
}

// BLINK LEDS AT RESTART/UPLOAD
bool StatusBlink(bool do_set, byte n_blinks, uint16_t dt_led, bool rat_in_blink)
{

	static uint32_t t_blink_last = 0;
	static byte n_cycles = 0;
	static uint16_t dt_cycle = 0;
	static bool do_led_on = true;
	static bool do_blink = true;
	static int cnt_blink = 0;
	static int is_rat_blink = false;
	byte _duty[2] = { 100, 0 };
	VEC<byte> duty(2, __LINE__, _duty);

	// Set values
	if (do_set) {
		n_cycles = n_blinks;
		dt_cycle = dt_led;
		do_blink = true;
		is_rat_blink = rat_in_blink;
	}

	// Bail if not running
	else if (!do_blink) {
		return false;
	}

#if DO_TEENSY_DEBUG
	DB_FUN_STR();
#endif

	// Flash sequentially
	if (cnt_blink <= n_cycles) {
		if (millis() > t_blink_last + dt_cycle) {

			// Set LEDs
			analogWrite(pin.LED_TRACKER, duty[(int)do_led_on]);
			if (!is_rat_blink) {
				analogWrite(pin.LCD_LED, duty[(int)do_led_on]);
				analogWrite(pin.LED_REW_C, duty[(int)do_led_on]);
			}
			else {
				analogWrite(pin.LED_REW_C, duty[(int)do_led_on]);
			}

			// Update stuff
			t_blink_last = millis();
			do_led_on = !do_led_on;
			cnt_blink += !do_led_on ? 1 : 0;
		}
		return true;
	}

	// Reset LEDs
	else {
		ChangeLCDlight(0);
		analogWrite(pin.LED_REW_C, rewLEDduty[0]);
		analogWrite(pin.LED_REW_R, rewLEDduty[0]);
		analogWrite(pin.LED_TRACKER, trackLEDduty[1]);
		do_led_on = true;
		cnt_blink = 0;
		do_blink = false;
		is_rat_blink = false;
		return false;
	}
}

#pragma endregion


#pragma region --------INTERUPTS---------

// TIMER INTERUPT/HANDLER
void Interupt_TimerHandler()
{
	// Local vars
	bool is_done = false;

	// Bail if not active now
	if (!v_doStepTimer) {
		return;
	}

	// Extend target reached
	else if (v_stepDir == 'e' &&
		v_cnt_steps >= v_stepTarg) {

		// Set flag
		is_done = true;
	}

	// Release switch triggered
	else if (v_stepDir == 'r' &&
		digitalRead(pin.SWITCH_DISH) == LOW) {

		// Set flag
		is_done = true;
	}

	// Clean up and bail
	if (is_done) {

		// Make sure step off
		v_stepState = false;
		digitalWrite(pin.ED_STP, v_stepState);

		// Block handler
		v_doStepTimer = false;

		// Set done flag
		v_isArmMoveDone = true;

		// Bail
		return;
	}

	// Incriment count
	v_cnt_steps += v_stepState ? 1 : 0;

	// Set state
	v_stepState = !v_stepState;

	// Step motor
	digitalWrite(pin.ED_STP, v_stepState);
}

// POWER OFF
void Interupt_Power()
{

	// Turn off power
	digitalWrite(pin.PWR_OFF, HIGH);

	// Disable regulators
	digitalWrite(pin.REG_24V_ENBLE, LOW);
	digitalWrite(pin.REG_12V2_ENBLE, LOW);
	digitalWrite(pin.REG_5V1_ENBLE, LOW);

	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

// DETECT IR SYNC EVENT
void Interupt_IR_Detect()
{
	// Local vars
	static uint32_t t_debounce = 0;

	// Exit if < 50 ms has not passed
	if (millis() < t_debounce) {
		return;
	}

	// Handle test
	if (Debug.flag.do_v_irSyncTest) {

		// Set tracker LED high
		digitalWrite(pin.TEST_SIGNAL, HIGH);
	}

	// Store time
	v_dt_ir = millis() - v_t_irSyncLast;
	v_t_irSyncLast = millis();

	// Incriment count 
	v_cnt_ir++;

	// Set flag
	v_isNewIR = true;

	// Update debounce
	t_debounce = millis() + 50;
}

#pragma endregion

#pragma endregion


void setup() {

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// Wait for SerialUSB
	uint32_t t_check_ser_usb = millis() + 500;
	while (!SerialUSB && millis() < t_check_ser_usb);

	// XBee R2
	r2c.hwSerial.begin(57600);
	r2a.hwSerial.begin(57600);

	// SETUP PINS
	SetupPins();

	// INITIALIZE LCD
	LCD.InitLCD();

	// DISABLE VOLTAGE REGULATORS
	digitalWrite(pin.REG_24V_ENBLE, LOW);
	digitalWrite(pin.REG_12V2_ENBLE, LOW);
	// Keep Teensy (5V) powered
#if !DO_TEENSY_DEBUG
	digitalWrite(pin.REG_5V1_ENBLE, LOW);
#endif

	// HANDLE ANY PREVIOUS POWER OFF BUTTON PRESS

	// Wait for power on flag
	bool do_wait_pwr = !DO_DEBUG && !DO_AUTO_POWER;

	// Wait for button release 
	while (do_wait_pwr && digitalRead(pin.PWR_SWITCH) == LOW) {
		delay(10);
	}

	// Set off switch back to high
	digitalWrite(pin.PWR_OFF, HIGH);

	// WAIT FOR POWER ON BUTTON PRESS

	// Wait for button press
	if (do_wait_pwr) {
		while (digitalRead(pin.PWR_SWITCH) == HIGH);
	}
	// Otherwise pause before powering on
	else {
		delay(1000);
	}

	// TURN ON POWER

	// Set power off pin low
	delay(100);
	digitalWrite(pin.PWR_OFF, LOW);
	delayMicroseconds(100);

	// Pulse power on switch high
	digitalWrite(pin.PWR_ON, HIGH);
	delayMicroseconds(100);
	digitalWrite(pin.PWR_ON, LOW);
	delayMicroseconds(100);

	// Wait for button release
	while (do_wait_pwr && digitalRead(pin.PWR_SWITCH) == LOW) {
		delay(10);
	}

	// ENABLE VOLTGAGE REGULATORS
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_12V2_ENBLE, HIGH);
	digitalWrite(pin.REG_5V1_ENBLE, HIGH);
	// Power Teensy
#if DO_TEENSY_DEBUG
	digitalWrite(pin.REG_5V1_ENBLE, HIGH);
#endif

	// LOG/PRINT SETUP RUNNING

	// Log run mode
	if (DO_DEBUG) {
		Debug.DB_General(__FUNCTION__, __LINE__, "RUN MODE = DEBUG");
	}
	else {
		Debug.DB_General(__FUNCTION__, __LINE__, "RUN MODE = RELEASE");
	}

	// Log compile time
	Debug.sprintf_safe(buffLrg, buff_lrg, "BUILD DATE: %s %s", __DATE__, __TIME__);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Print to LCD
	ChangeLCDlight(64);
	Debug.PrintLCD(true, "SETUP", "MAIN");

	// Log and print to console
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Setup...");
	Debug.PrintAll(1000);

	// SHOW RESTART BLINK
	delayMicroseconds(100);
	StatusBlink(true, 1, 100);
	while (StatusBlink());
	ChangeLCDlight(64);

	// SETUP AUTODRIVER

	// Configure SPI
	Debug.PrintLCD(true, "RUN SETUP", "AutoDriver");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: AutoDriver Setup...");
	Debug.PrintAll(500);
	AD_R.SPIConfig();
	delayMicroseconds(100);
	AD_F.SPIConfig();
	delayMicroseconds(100);
	// Reset boards
	AD_Reset(maxAcc, maxDec, maxSpeed);
	Debug.PrintLCD(true, "DONE SETUP", "AutoDriver");
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: AutoDriver Setup...");
	Debug.PrintAll(500);
	// Set to high impedance so robot can be moved
	AD_R.softHiZ();
	AD_F.softHiZ();

	// SETUP BIG EASY DRIVER

	// Start BigEasyDriver in sleep
	Debug.PrintLCD(true, "RUN SETUP", "Big Easy");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Big Easy Driver Setup...");
	Debug.PrintAll(500);
	digitalWrite(pin.ED_RST, HIGH);
	digitalWrite(pin.ED_SLP, LOW);
	Debug.PrintLCD(true, "DONE SETUP", "Big Easy");
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISEHD: Big Easy Driver Setup...");
	Debug.PrintAll(500);

	// SETUP PIXY I2C COM
	Debug.PrintLCD(true, "RUN SETUP", "Pixy I2C");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Pixy I2C Setup...");
	Debug.PrintAll(500);
	Pixy.PixyBegin();
	Debug.PrintLCD(true, "DONE SETUP", "Pixy I2C");
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Pixy I2C Setup...");
	Debug.PrintAll(500);

	// DUMP BUFFER
	Debug.PrintLCD(true, "RUN SETUP", "Dump Serial");
	while (c2r.hwSerial.available() > 0) {
		c2r.hwSerial.read();
	}
	while (a2r.hwSerial.available() > 0) {
		a2r.hwSerial.read();
	}
	Debug.PrintLCD(true, "DONE SETUP", "Dump Serial");

	// RESET VOLITILES AND RELAYS
	t_sync = 0;
	v_t_irSyncLast = 0;
	v_dt_ir = 0;
	v_cnt_ir = 0;
	v_isNewIR = false;
	v_stepState = false;
	v_doStepTimer = false;
	v_isArmMoveDone = false;
	v_cnt_steps = 0;
	v_stepTarg = 0;
	v_stepDir = 'e';
	digitalWrite(pin.REL_FOOD, LOW);
	digitalWrite(pin.REL_ETOH, LOW);

	// GET BATTERY LEVEL
	uint32_t t_check_vcc = millis() + 1000;
	Debug.PrintLCD(true, "RUN SETUP", "Battery Check");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Battery Check...");
	Debug.PrintAll(500);
	while (CheckBattery(true) == 0 && millis() < t_check_vcc);

	// EXIT WITH ERROR IF POWER OFF
	if (CheckBattery(true) == 0) {
		// Run error hold
		Debug.RunErrorHold("FAILED BATTERY CHECK", "POWER OFF");
	}
	Debug.PrintLCD(true, "DONE SETUP", "Battery Check");
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Battery Check...");
	Debug.PrintAll(500);

	// SETUP OPENLOG
	Debug.PrintLCD(true, "RUN SETUP", "OpenLog");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: OpenLog Setup...");
	Debug.PrintAll(500);

	// Setup OpenLog
	if (Log.Setup())
	{
		// Log setup finished
		Debug.PrintLCD(true, "DONE SETUP", "OpenLog");
		Debug.DB_General(__FUNCTION__, __LINE__, "SUCCEEDED: OpenLog Setup");
		Debug.PrintAll(500);
	}
	else {
		// Hold for error
		Debug.PrintLCD(true, "FAILED SETUP", "OpenLog");
		Debug.RunErrorHold("FAILED OPENLOG SETUP", "OPENLOG SETUP");
	}

	// Create new log file
	Debug.PrintLCD(true, "RUN SETUP", "Log File");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Make New Log...");
	Debug.PrintAll(500);
	if (Log.OpenNewLog() == 0) {
		// Hold for error
		Debug.PrintLCD(true, "FAILED SETUP", "Log File");
		Debug.RunErrorHold("FAILED TO CREATE OPENLOG FILE", "OPEN LOG FILE");
	}
	else {
		Debug.PrintLCD(true, "DONE SETUP", "Log File");
		Debug.sprintf_safe(buffLrg, buff_lrg, "SUCCEEDED: Make New Log: file_name=%s", Log.buff_med_logFile);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		Debug.PrintAll(500);
	}

	// DEFINE EXTERNAL INTERUPTS
	Debug.PrintLCD(true, "RUN SETUP", "Interrupts");
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Interrupts Setup...");
	Debug.PrintAll(500);
	uint32_t t_check_ir = millis() + 1000;
	bool is_ir_off = false;

	// Check if IR detector already low
	while (!is_ir_off && millis() < t_check_ir) {
		is_ir_off = digitalRead(pin.INTERUPT_IR_DETECT) == HIGH;
	}

	// Enable ir detector interupt
	if (is_ir_off) {
		// IR detector
		Debug.DB_General(__FUNCTION__, __LINE__, "ENABLING IR SENSOR INTERUPT");
		Debug.PrintAll(500);
		attachInterrupt(digitalPinToInterrupt(pin.INTERUPT_IR_DETECT), Interupt_IR_Detect, FALLING);
	}

	// Do not enable if ir detector pin already low
	else {
		// Skip ir sync setup
		t_sync = 1;
		Debug.DB_Error(__FUNCTION__, __LINE__, "IR SENSOR DISABLED");
		Debug.PrintAll(500);
	}

	// Power off
#if !DO_AUTO_POWER
	Debug.DB_General(__FUNCTION__, __LINE__, "ENABLING POWER SWITCH INTERUPT");
	Debug.PrintAll(500);
	attachInterrupt(digitalPinToInterrupt(pin.PWR_SWITCH), Interupt_Power, FALLING);
#endif

	// Log interupts setup
	Debug.PrintLCD(true, "DONE SETUP", "Interrupts");
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Interrupts Setup...");
	Debug.PrintAll(500);

	// Start Feed Arm timer
	FeederArmTimer.setPeriod(ezStepPeriod);
	FeederArmTimer.attachInterrupt(Interupt_TimerHandler);

	// RESET FEEDER ARM
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Reset Feeder Arm...");
	Debug.PrintAll(500);
	Debug.PrintLCD(true, "RUN SETUP", "Reset Arm");

	// Extend arm
	uint32_t t_check_ext = millis() + 1000;
	Reward.ExtendFeedArm(ezResetExtStps);
	while (Reward.do_ExtendArm && millis() < t_check_ext) {
		Reward.CheckFeedArm();
	}

	// Log/Print error
	if (Reward.do_ExtendArm) {
		Debug.DB_Error(__FUNCTION__, __LINE__, "FAILED: Reset Feeder Arm: Extend Feeder Arm");
		Debug.PrintAll(500);
	}

	// Retract arm if extend succeeded
	else {
		uint32_t t_check_ret = millis() + 1000;
		Reward.RetractFeedArm();
		while (Reward.do_RetractArm && millis() < t_check_ext) {
			Reward.CheckFeedArm();
		}

		// Log/Print error
		if (Reward.do_RetractArm) {
			Debug.DB_Error(__FUNCTION__, __LINE__, "FAILED: Reset Feeder Arm: Retract Feeder Arm");
			Debug.PrintAll(500);
		}
	}

	// Log success
	if (!Reward.do_ExtendArm && !Reward.do_RetractArm) {
		Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Reset Feeder Arm");
		Debug.PrintAll(500);
	}

	// IMPORT LAST TEENSY LOG
#if DO_TEENSY_DEBUG

	// Log everything in queue
	Log.WriteAll(5000);

	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Get Teensy Log...");
	Debug.PrintLCD(true, "RUN SETUP", "Teensy Log");
	Debug.PrintAll(500);

	// Begin Teensy serial  [57600, 115200, 256000]
	r2t.hwSerial.begin(256000);

	// Wait for SerialUSB
	uint32_t t_check_ser_r2t = millis() + 500;
	while (!r2t.hwSerial && millis() < t_check_ser_r2t);

	// Get last db log
	ImportTeensy();
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Get Teensy Log");
	Debug.PrintLCD(true, "DONE SETUP", "Teensy Log");
	Debug.PrintAll(500);
	delay(10);

	// Set flag to begin sending Teensy logs
	FC.is_TeensyReady = true;

	// Send log number to Teensy as optional argument
	Debug.sprintf_safe(buffLrg, buff_lrg, "%05u", Log.logNum);
	SendTeensy(__FUNCTION__, __LINE__, freeMemory(), 'L', buff_lrg);

	// Log everything in queue
	Log.WriteAll(5000);
#endif

	// PRINT SERIAL RING BUFFER SIZE
	Debug.sprintf_safe(buffLrg, buff_lrg, "RING BUFFER SIZE: %dB", SERIAL_BUFFER_SIZE);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	Debug.PrintAll(500);

	// CHECK SERIAL BUFFER SIZE
	Debug.sprintf_safe(buffLrg, buff_lrg, "SERIAL BUFFER SIZE: ACTUAL=%dB EXPECTED=%dB", SERIAL_BUFFER_SIZE, expectedSerialBufferSize);
	if (SERIAL_BUFFER_SIZE == expectedSerialBufferSize) {
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Buffer size is wrong
	else {
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	// PRINT DEBUG STATUS
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING IN %s MODE: |%s%s%s%s%s",
		DO_DEBUG ? "DEBUG" : "RELEASE",
		DO_LOG ? "LOGGING ENABLED|" : "",
		DO_PRINT_DEBUG ? "PRINTING ENABLED|" : "",
		DO_TEENSY_DEBUG ? "TEENSYHELPER ENABLED|" : "",
		DO_FAST_PRINT ? "FAST PRINTING ENABLED|" : "",
		DO_FAST_LOG ? "FAST LOGGING ENABLED|" : "");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// PRINT AVAILABLE MEMORY
	memStart = freeMemory();
	Debug.sprintf_safe(buffLrg, buff_lrg, "AVAILABLE MEMORY: %0.2fKB", (float)memStart / 1000);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	Debug.PrintAll(500);

	// PRINT BATTERY VOLTAGE
	Debug.sprintf_safe(buffLrg, buff_lrg, "BATTERY VCC: %0.2fV", vccAvg);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	Debug.PrintAll(500);
	Debug.sprintf_safe(buffLrg, buff_lrg, "%0.2fV", vccAvg);
	Debug.PrintLCD(true, "BATTERY VCC", buff_lrg);

	// TURN OFF LCD LIGHT
	ChangeLCDlight(0);

	// PRINT SETUP FINISHED
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Setup");
	Debug.PrintAll(500);

}


void loop() {

#pragma region //--- ONGOING OPPERATIONS ---

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// DO LOOP CHECK
	Debug.CheckLoop();

	// PARSE CHEETAHDUE SERIAL INPUT
	GetSerial(&a2r);

	// SEND CHEETAHDUE DATA
	SendPacket(&r2a);

	// PARSE CS SERIAL INPUT
	GetSerial(&c2r);

	// SEND CS DATA
	SendPacket(&r2c);

	// RESEND DATA
	CheckResend(&r2a);
	CheckResend(&r2c);

	// PRINT QUEUED DB
	Debug.Print();

	// STORE QUEUED LOGS
	Log.WriteLog();

	// SEND SERIAL DATA
	if (FC.do_LogSend) {
		// Send log
		Log.StreamLogs();

		// Print
		if (!FC.do_LogSend) {
			Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED SENDING LOGS");
		}
	}

	// UPDATE TESTS
	TestUpdate();

	// GET BUTTON INPUT
	if (GetButtonInput()) {

		// Process button input
		ProcButtonInput();
	}

	// CHECK FOR IR TRIGGERED HALT
	Check_IRprox_Halt();

	// CHECK FOR IR EVENT
	IR_SyncCheck();

	// RUN REWARD
	if (FC.do_RunRew) {
		if (Reward.RunReward()) {

			// Reset flag
			FC.do_RunRew = false;
		}
	}

	// END ONGOING REWARD
	Reward.CheckEnd();

	// RETRACT FEEDER ARM
	Reward.CheckFeedArm();

	// RUN MOVE
	if (FC.do_RunMove) {
		if (Move.RunMove()) {
			FC.do_RunMove = false;
		}
	}

	// CHECK IF STILL BLOCKING
	CheckBlockTimElapsed();

	// GET AD STATUS
	AD_CheckOC();

	// RUN ETOH SOLONOID
	CheckEtOH();

	// GET VCC
	CheckBattery();

	// CHECK STATUS BLINK
	StatusBlink();

	// Check if time to quit
	if (FC.do_Quit) {
		QuitSession();
	}

	// WAIT FOR HANDSHAKE
	if (!CheckForHandshake()) {
		return;
	}

	// UPDATE PIXY
	Pixy.PixyUpdate();

	// CHECK IF RAT POS FRAMES DROPPING
	CheckSampDT();

	// INITIALIZE RAT AHEAD
	InitializeTracking();

	// UPDATE EKF
	UpdateEKF();

	// UPDATE PID AND SPEED
	double new_speed = Pid.PidUpdate();
	if (new_speed >= 0) {
		RunMotor('f', new_speed, MC_CALL::ID::PID);
	}

	// UPDATE BULLDOZER
	Bull.UpdateBull();

	// LOG TRACKING DATA
	Debug.DB_TrackData();

#pragma endregion

#pragma region //--- PROCESS NEW MESSAGES ---

	// CONTINUE LOOP IF NO NEW MESSAGE
	if (!c2r.is_new) {
		return;
	}

	//-------------- (T) SYSTEM TESTS --------------

	if (c2r.idNew == 'T')
	{
		// Store message data
		cmd.testCond = (byte)c2r.dat[0];
		cmd.testRun = (byte)c2r.dat[1];
		cmd.testDat = c2r.dat[2];

		// TEST SETUP
		if (cmd.testRun == 0) {

			// Set testing flag
			Debug.flag.do_systemTesting = true;

			// Get number of test pings to send
			if (cmd.testRun == 0) {
				n_testPings = cmd.testDat;
			}

			// Simulated rat test
			if (cmd.testCond == 1) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TEST: SIMULATED RAT TEST");
				Debug.flag.do_simRatTest = true;
			}

			// PID calibration test
			else if (cmd.testCond == 2) {
				Debug.sprintf_safe(buffLrg, buff_lrg, "DO TEST: PID CALIBRATION = kC=%0.2f", kC);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
				Debug.flag.do_pidCalibration = true;
			}

			// IR sync timing test
			if (cmd.testCond == 6) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TEST: IR SYNC TIME");
				Debug.flag.do_v_irSyncTest = true;
			}

			// Robot hardware test
			if (cmd.testCond == 7) {
				// Run hardware test
				HardwareTest(true, true, true);
			}

			// Run ping test alone
			else {
				HardwareTest(false, false, true);
			}

			// Pass test info info to ard
			QueuePacket(&r2a, 't', c2r.dat[0], c2r.dat[1], c2r.dat[2], 0, true);

			// Send test done confirmation
			Debug.DB_General(__FUNCTION__, __LINE__, "Sending 'T' Done Confirmation");
			QueuePacket(&r2c, 'T', 0, 0, 0, c2r.packArr[ID_Ind<R4_COM<USARTClass>>('T', &c2r)], true, false, true);

		}

		// VT CALIBRATION TEST UPDATE
		else if (cmd.testCond == 3) {

			// RUNNING
			if (cmd.testRun == 1) {

				// Store new speed
				double new_speed = double(cmd.testDat);

				// Run at new speed
				if (new_speed > 1) {

					// Run motor
					RunMotor('f', new_speed, MC_CALL::ID::OVERIDE);

					// Set tracker duty to max
					trackLEDduty[0] = 255;
					trackLEDduty[1] = 255;
					analogWrite(pin.LED_TRACKER, trackLEDduty[0]);

					// Print speed
					Debug.sprintf_safe(buffLrg, buff_lrg, "VT CALIBRATION SPEED = %0.0f cm/sec", new_speed);
					Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				}

				// Halt robot
				else {
					HardStop(__FUNCTION__, __LINE__, true);
				}


			}

			// END OF RUN
			if (cmd.testRun == 2) {

				// Halt robot
				HardStop(__FUNCTION__, __LINE__, true);

				// Set tracker duty to default
				trackLEDduty[0] = trackLEDdutyDefault[0];
				trackLEDduty[1] = trackLEDdutyDefault[1];
				analogWrite(pin.LED_TRACKER, trackLEDduty[0]);
			}

		}

		// HALT ERROR TEST UPDATE
		else if (cmd.testCond == 4) {

			// Store new speed
			double new_speed = double(cmd.testDat);

			// Print speed
			Debug.sprintf_safe(buffLrg, buff_lrg, "HALT ERROR SPEED = %0.0f cm/sec", new_speed);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			if (new_speed > 0) {
				// Run motor
				RunMotor('f', new_speed, MC_CALL::ID::OVERIDE);
			}
			else {
				// Halt robot
				HardStop(__FUNCTION__, __LINE__, true);
			}
		}

		// HARDWARE TEST UPDATE
		else if (cmd.testCond == 7) {
			// Send test info to CheetaDue
			QueuePacket(&r2a, 't', c2r.dat[0], c2r.dat[1], c2r.dat[2], 0, true);
		}


	}


	//-------------- (S) SESSION SETUP --------------

	if (c2r.idNew == 'S')
	{
		// Store message data
		cmd.sesMsg = (byte)c2r.dat[0];

		// Store info from first 'S' packet
		if (cmd.sesMsg == 1) {
			cmd.sesCond = (byte)c2r.dat[1];
			cmd.sesTask = (byte)c2r.dat[2];
		}

		// Store info from second 'S' packet
		if (cmd.sesMsg == 2) {
			cmd.sesSound = (byte)c2r.dat[1];
			cmd.sesSetpointHeadDist = c2r.dat[2];
		}

		// Handle first 'S' packet
		if (cmd.sesMsg == 1) {

			// Reset flags
			FC.is_ManualSes = false;
			FC.is_ForageTask = false;

			// Handle Manual session
			if (cmd.sesCond == 1) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO MANUAL SESSION");

				// Turn on LCD light
				FC.do_ChangeLCDstate = !FC.is_LitLCD;

				// Set to high impedance so robot can be moved
				AD_R.softHiZ();
				AD_F.softHiZ();

				// Set flag
				FC.is_ManualSes = true;
			}

			// Handle Behavior session
			if (cmd.sesCond == 2) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO BEHAVIOR SESSION");

				// Update pixy coeff
				for (int i = 0; i < pixyOrd; i++) {
					pixyCoeff[i] = pixyPackCoeff[i];
				}

				// Update pixy shift
				pixyShift = pixyPackShift;
			}

			// Handle Implant session
			if (cmd.sesCond == 3) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO IMPLANT SESSION");

				// Update pixy coeff
				for (int i = 0; i < pixyOrd; i++) {
					pixyCoeff[i] = pixyPackCoeff[i];
				}

				// Update pixy shift
				pixyShift = pixyCubeShift;
			}

			// Handle Track task
			if (cmd.sesTask == 1) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TRACK TASK");

				// Set rew led min
				rewLEDduty[0] = rewLEDmin[0];

				// Change autodriver max acc
				maxAcc = maxAccArr[0];

				// Change reward solonoid on scale
				Reward.solOpenScale = solOpenScaleArr[0];
			}

			// Handle Forage task
			if (cmd.sesTask == 2) {
				Debug.DB_General(__FUNCTION__, __LINE__, "DO FORAGE TASK");

				// Set rew led forage min
				rewLEDduty[0] = rewLEDmin[1];

				// Set to min
				analogWrite(pin.LED_REW_C, rewLEDduty[0]);
				analogWrite(pin.LED_REW_R, rewLEDduty[0]);

				// Change autodriver max acc
				maxAcc = maxAccArr[1];

				// Change reward solonoid on scale
				Reward.solOpenScale = solOpenScaleArr[1];

				// Set flag
				FC.is_ForageTask = true;
			}

			// Update autodriver settings
			AD_Reset(maxAcc, maxDec, maxSpeed);

		}

		// Handle second 'S' packet
		if (cmd.sesMsg == 2) {

			// Handle no sound condition
			if (cmd.sesSound == 0) {

				// No sound
				QueuePacket(&r2a, 's', 0);
				Debug.DB_General(__FUNCTION__, __LINE__, "NO SOUND");
			}

			// Handle white noise only condition
			if (cmd.sesSound == 1) {

				// Use white noise only
				QueuePacket(&r2a, 's', 1);
				Debug.DB_General(__FUNCTION__, __LINE__, "DONT DO TONE");
			}

			// Handle white noise and reward tone condition
			if (cmd.sesSound == 2) {

				// Use white and reward noise
				QueuePacket(&r2a, 's', 2);
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TONE");
			}

			// Compute and store pid setpoint
			Pid.setPoint = setPointHead + cmd.sesSetpointHeadDist;

			// Update tracker feeder pass distance
			feedTrackPastDist = feedDist + feedHeadPastDist + cmd.sesSetpointHeadDist;

			// Log set setpoint
			Debug.sprintf_safe(buffLrg, buff_lrg, "PID SETPOINT %0.2fcm", Pid.setPoint);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Log feed pass distance
			Debug.sprintf_safe(buffLrg, buff_lrg, "DISH PASS DISTANCE %0.2fcm", feedTrackPastDist);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		}

	}


	//-------------- (Q) QUIT SESSION --------------

	if (c2r.idNew == 'Q') {

		// Log event
		Debug.DB_General(__FUNCTION__, __LINE__, "DO QUIT");

		// Set flags and time
		FC.do_Quit = true;

		// Block IR sensor in case CheetahDue resets early
		FC.do_BlockDetectIR = true;

		// Tell CheetahDue to quit 
		QueuePacket(&r2a, 'q', 0, 0, 0, 0, true);

		// Block all motor control
		if (!SetMotorControl(MC_CON::ID::HALT, MC_CALL::ID::QUIT)) {

			// Log error
			Debug.DB_Error(__FUNCTION__, __LINE__, "\"Quit\" FAILED TO SET MOTOR CONTROL TO \"Halt\"");
		}

	}



	//-------------- (M) DO MOVE --------------

	if (c2r.idNew == 'M') {

		// Store move count and pos
		cmd.cnt_move = (byte)c2r.dat[0];
		cmd.moveTarg = c2r.dat[1];

		// Store move count and format as string
		Move.ProcMoveCmd(cmd.cnt_move, cmd.moveTarg);

		// Set flags
		FC.do_RunMove = true;

	}



	//-------------- (R) REWARD --------------

	if (c2r.idNew == 'R') {

		// Store message data
		cmd.rewType = (byte)c2r.dat[0];
		cmd.goalPos = c2r.dat[1];
		cmd.rewZoneOrDelay = (byte)c2r.dat[2];

		// Bail if in the process of rewarding
		if (Reward.isRewarding) {
			Debug.DB_Warning(__FUNCTION__, __LINE__, "SKIPPED REWARD: RECIEVED DURING ONGOING REWARD");
			return;
		}

		// Abort ongoing reward reward
		if (cmd.rewType == 1 && FC.do_RunRew) {

			// Log aborting last reward
			Debug.DB_Warning(__FUNCTION__, __LINE__, "ABORTING PREVIOUS REWARD");

			// Decriment count and reset
			Reward.cnt_rew--;
			Reward.RewardReset();
			FC.do_RunRew = false;
		}

		// Process reward command
		Reward.ProcRewCmd(cmd.rewType, cmd.goalPos, cmd.rewZoneOrDelay);

		// NOW reward
		if (Reward.rewMode == REWARD::REWMODE::NOW) {

			// Start reward imediately
			Reward.StartRew();
		}

		// CUED or FREE reward
		else if (Reward.rewMode == REWARD::REWMODE::CUE ||
			Reward.rewMode == REWARD::REWMODE::FREE) {

			// Set flag
			FC.do_RunRew = true;
		}

	}


	//-------------- (H) HALT ROBOT STATUS --------------

	if (c2r.idNew == 'H') {

		// Store message data
		FC.do_Halt = c2r.dat[0] != 0 ? true : false;

		if (FC.do_Halt) {

			// Log
			Debug.DB_General(__FUNCTION__, __LINE__, "HALT STARTED");

			// Stop pid and set to manual
			HardStop(__FUNCTION__, __LINE__);

			// Halt all motor activity
			SetMotorControl(MC_CON::ID::HALT, MC_CALL::ID::HALT);

		}
		else {

			// Log
			Debug.DB_General(__FUNCTION__, __LINE__, "HALT FINISHED");

			// Set motor control to "OPEN"
			if (FC.is_TrackingEnabled) {
				SetMotorControl(MC_CON::ID::OPEN, MC_CALL::ID::HALT);
			}
			// Set motor control to "HOLD"
			else {
				SetMotorControl(MC_CON::ID::HOLD, MC_CALL::ID::HALT);
			}


		}
	}


	//-------------- (B) BULLDOZE RAT STATUS --------------

	if (c2r.idNew == 'B') {

		// Store message data
		cmd.bullDel = c2r.dat[0];
		cmd.bullSpeed = c2r.dat[1];

		// Local vars
		bool is_mode_changed = false;

		// Reinitialize bulldoze
		Bull.BullReinitialize(cmd.bullDel, cmd.bullSpeed);

		// Check if mode should be changedchanged
		if (cmd.bullSpeed > 0) {

			// Mode changed
			if (!FC.do_Bulldoze) {

				// Log event
				Debug.DB_General(__FUNCTION__, __LINE__, "SET BULLDOZE ON");

				// Set flags
				is_mode_changed = true;
				FC.do_Bulldoze = true;

			}
			// Only settings changed
			else {
				is_mode_changed = false;
			}
		}
		else {
			// Mode changed
			if (FC.do_Bulldoze)
			{
				// Log event
				Debug.DB_General(__FUNCTION__, __LINE__, "SET BULLDOZE OFF");

				// Set flags
				is_mode_changed = true;
				FC.do_Bulldoze = false;

			}
			// Only settings changed
			else {
				is_mode_changed = false;
			}
		}

		// Don't exicute until rat is in and mode is changed
		if (FC.is_TrackingEnabled &&
			is_mode_changed) {

			if (FC.do_Bulldoze) {

				// Log event
				Debug.DB_General(__FUNCTION__, __LINE__, "BULLDOZE ON");

				// Turn bulldoze on
				Bull.BullOn();
			}
			else {

				// Log event
				Debug.DB_General(__FUNCTION__, __LINE__, "BULLDOZE OFF");

				// Turn bulldoze off
				Bull.BullOff();
			}
		}

	}


	//-------------- (I) RAT IN --------------

	if (c2r.idNew == 'I')
	{
		// Store message data
		FC.is_RatOnTrack = c2r.dat[0] == 1 ? true : false;

		if (FC.is_RatOnTrack) {

			// Log
			Debug.DB_General(__FUNCTION__, __LINE__, "RAT ON TRACK");

			// Reset rat pos data
			Pos[0].PosReset();
			Pos[2].PosReset();
		}
		else
		{
			// Log
			Debug.DB_General(__FUNCTION__, __LINE__, "RAT ON FORAGE PLATFORM");
		}

	}


	//-------------- (O) TASK DONE --------------

	if (c2r.idNew == 'O')
	{
		// Store message data
		FC.is_TaskDone = c2r.dat[0] > 0 ? true : false;

		if (FC.is_TaskDone) {

			// Log event
			Debug.DB_General(__FUNCTION__, __LINE__, "TASK DONE");

			// Turn off bulldoze
			Bull.BullOff();
			FC.do_Bulldoze = false;

			// Turn off pid
			Pid.PidStop();

			// Set to stop tracking
			FC.is_TrackingEnabled = false;
		}
	}


	//-------------- (P) VT DATA RECIEVED --------------

	if (c2r.idNew == 'P')
	{
		// Store message data
		cmd.vtEnt = (byte)c2r.dat[0];
		cmd.vtCM[cmd.vtEnt] = c2r.dat[1];
		U.f = c2r.dat[2];
		cmd.vtTS[cmd.vtEnt] = U.i32;

		// Handle rob vt data
		if (cmd.vtEnt == 1) {

			// Update VT
			Pos[cmd.vtEnt].UpdatePos(cmd.vtCM[cmd.vtEnt], cmd.vtTS[cmd.vtEnt]);

			// Set rat vt and pixy to setpoint if rat not in or task done
			if (!FC.is_RatOnTrack || FC.is_TaskDone) {

				Pos[0].SwapPos(Pos[1].posAbs + Pid.setPoint, Pos[1].t_update);
				Pos[2].SwapPos(Pos[1].posAbs + Pid.setPoint, Pos[1].t_update);
			}

			// Log first sample
			if (!Pos[1].is_streamStarted) {

				// Log
				Debug.sprintf_safe(buffLrg, buff_lrg, "FIRST ROBOT VT RECORD: pos_abs=%0.2f pos_cum=%0.2f n_laps=%d",
					Pos[1].posAbs, Pos[1].posCum, Pos[1].nLaps);
				Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

				// Set flag
				Pos[1].is_streamStarted = true;

				// Send robot streaming confirmation
				Debug.DB_General(__FUNCTION__, __LINE__, "SENDING STREAMING CONFIRMATION");
				QueuePacket(&r2c, 'K', 1, 0, 0, 0, true);

				// Set flag to begin sending vcc
				FC.do_SendVCC = true;

			}
		}

		// Handle rat vt data
		else if (cmd.vtEnt == 0) {

			// Update only after rat in before task done
			if (FC.is_RatOnTrack && !FC.is_TaskDone) {

				// Update rat VT
				Pos[cmd.vtEnt].UpdatePos(cmd.vtCM[cmd.vtEnt], cmd.vtTS[cmd.vtEnt]);

				// Use rat vt for pixy if running simulated rat test
				if (cmd.vtEnt == 0 && Debug.flag.do_simRatTest) {
					Pos[2].SwapPos(Pos[0].posAbs, Pos[0].t_update);
				}

				// Log first sample
				if (!Pos[0].is_streamStarted) {

					// Log
					Debug.sprintf_safe(buffLrg, buff_lrg, "FIRST RAT VT RECORD: pos_abs=%0.2f pos_cum=%0.2f n_laps=%d",
						Pos[0].posAbs, Pos[0].posCum, Pos[0].nLaps);
					Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

					// Set flag
					Pos[0].is_streamStarted = true;
				}
			}

		}

	}


	//-------------- (L) SEND LOG --------------

	if (c2r.idNew == 'L') {

		// Flag to begin sending
		FC.do_LogSend = c2r.dat[0] == 1 ? true : false;

		// Check for 2 way confirmation before sending log
		if (!FC.do_LogSend) {

			// Log
			Debug.sprintf_safe(buffLrg, buff_lrg, "SENDING LOG: logs_stored=~%d b_stored=~%d",
				Log.cnt_logsStored, Log.cnt_logBytesStored);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Send number of log bytes being sent
			QueuePacket(&r2c, 'U', Log.cnt_logBytesStored, 0, 0, 0, true);

			// Block sending vcc updates
			FC.do_SendVCC = false;
		}

		// Begin sending log
		else {

			// Log
			Debug.DB_General(__FUNCTION__, __LINE__, "DO SEND LOG");

			// Set send time
			Log.t_beginSend = millis() + Log.dt_beginSend;

			// Store remaining logs
			Log.WriteAll(1000);
		}

	}


#pragma endregion

}
