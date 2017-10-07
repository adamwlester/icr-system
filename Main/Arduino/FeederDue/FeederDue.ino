// ######################################

// ============= FEEDERDUE ==============

// ######################################

#include "FeederDue.h"
//
#include "FeederDue_PinMap.h"

// NOTES:
/*

* XBee DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size)

* XBee DO (to UART rx) buffer = 202 bytes

* ARDUINO SERIAL_BUFFER_SIZE CHANGED FROM 128 TO 512
Path: "C:\Users\lester\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.8\cores\arduino\RingBuffer.h"

* SerialUSB receive buffer size is now 512 (ARDUINO 1.5.2 BETA - 2013.02.06)

* DATA TYPES:
byte = 1 byte
char = 1 byte
int = 4 byte
long = 4 byte
float = 4 byte
double = 8 byte

* Step down resistor for vcc monitoring:
To ground = 2.2k Ohm
To vcc = 8.2k Ohm

* config.txt settings:
57600,26,3,2,0,0,0
baud,escape,esc#,mode,verb,echo,ignoreRX

*/


#pragma region ========== CLASS DECLARATIONS ===========

#pragma region ----------CLASS: POSTRACK----------
class POSTRACK
{
public:
	// VARS
	char instID[20] = { 0 };
	int nSamp = 0;
	double posArr[6] = { 0 }; // (cm)
	uint32_t t_tsArr[6] = { 0 }; // (ms)
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
	void PosReset();
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
	char modePID[25] = { 0 }; // ["Manual" "Automatic" "Halted"]
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
	int cal_cntPcArr[4] = { 0 };
	int cal_stepNow = 0;
	float cal_PcCnt = 0; // oscillation count
	float cal_PcSum = 0; // oscillation period sum
	uint32_t cal_t_PcNow = 0;
	uint32_t cal_t_PcLast = 0;
	float cal_PcArr[4] = { 0 };
	float cal_PcAvg = 0;
	float cal_PcNow = 0;
	float cal_PcAll = 0;
	float cal_errNow = 0;
	float cal_errLast = 0;
	float cal_errAvg = 0;
	float cal_errArr[4] = { 0 };
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
	void PID_Run(char called_from[]);
	void PID_Stop(char called_from[]);
	void PID_Hold(char called_from[]);
	void PID_Reset();
	void SetThrottle();
	void CheckThrottle();
	void PID_CheckMotorControl();
	void CheckSetpointCrossing();
	void PID_CheckEKF(uint32_t t);
	void PID_ResetEKF(char called_from[]);
	void PID_SetUpdateTime(uint32_t t);
	void PrintPID(char msg[]);
	double RunCalibration();
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
	char modeBull[25] = { 0 }; // ["Active" "Inactive"]
	char stateBull[25] = { 0 }; // ["off", "On", "Hold"]
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
	void BullReinitialize(byte del, byte spd, char called_from[]);
	void BullRun(char called_from[]);
	void BullStop(char called_from[]);
	void BullOn(char called_from[]);
	void BullOff(char called_from[]);
	void BullHold(char called_from[]);
	void BullResume(char called_from[]);
	void BullReset();
	void BullCheckMotorControl();
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
	double GetMoveError(double now_pos);
	void MoveToReset();
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
	char modeReward[10] = { 0 }; // ["None" "Free" "Cue" "Now"]
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
	const byte armExtStps = 160;
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
	void RewardReset();
};

#pragma endregion 

#pragma region ----------CLASS: LOGGER----------
class LOGGER
{
public:
	// VARS
	USARTClass &port = Serial1;
	byte msg_streamSuccess[3] = { '>','>','>' };
	byte msg_streamFail[3] = { '>','>','!' };
	uint32_t t_sent = 0; // (ms)
	uint32_t t_rcvd = 0; // (ms)
	uint32_t t_write = 0; // (us)
	const int dt_write = 50 * 1000; // (us)
	uint32_t t_beginSend = 0;
	int dt_beginSend = 1000;
	static const int logQueueSize = 50;
	char logQueue[logQueueSize][maxStoreStrLng] = { { 0 } };
	int logQueueIndStore = 0;
	int logQueueIndRead = 0;
	int cnt_logsStored = 0;
	static const int maxBytesStore = 500;
	char rcvdArr[maxBytesStore] = { 0 };
	char mode = ' '; // ['<', '>']
	char openLgSettings[100] = { 0 };
	char logFile[50] = { 0 };
	int logNum = 0;
	char logCntStr[10] = { 0 };
	int cnt_logBytesStored = 0;
	int cnt_logBytesSent = 0;
	bool isFileReady = false;
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

// Initialize FUSER class instance
FUSER ekf;

// Initialize UTAG union instance
UTAG U;

// Initialize AutoDriver_Due class instances
AutoDriver_Due AD_R(pin.AD_CSP_R, pin.AD_RST);
AutoDriver_Due AD_F(pin.AD_CSP_F, pin.AD_RST);
 
// Initialize PixyI2C class instance
PixyI2C Pixy(0x54);

// Initialize LCD5110 class instance
LCD5110 LCD(pin.Disp_CS, pin.Disp_RST, pin.Disp_DC, pin.Disp_MOSI, pin.Disp_SCK);

// Initialize array of POSTRACK class instances
POSTRACK Pos[3] = {
	POSTRACK(millis(), "RatVT", 4),
	POSTRACK(millis(), "RobVT", 4),
	POSTRACK(millis(), "RatPixy", 6)
};

// Initialize PID class instance
PID Pid(millis(), kC, pC, pidSetPoint);

// Initialize BULLDOZE class instance
BULLDOZE Bull(millis());

// Initialize MOVETO class instance
MOVETO Move(millis());

// Initialize REWARD class instance
REWARD Reward(millis());

// Initialize LOGGER class instance
LOGGER Log(millis());

// Initialize DueTimer class instance
DueTimer FeederArmTimer = DueTimer::getAvailable().setFrequency(dt_armStep);
#pragma endregion 

#pragma endregion 


#pragma region ========= FUNCTION DECLARATIONS =========

// PARSE SERIAL INPUT
void GetSerial(R4 *r4);
// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4 *r4, char mtch = '\0');
// STORE PACKET DATA TO BE SENT
void QueuePacket(R2 *r2, char id, float dat1 = 0, float dat2 = 0, float dat3 = 0, uint16_t pack = 0, bool do_conf = true);
// SEND SERIAL PACKET DATA
bool SendPacket(R2 *r2);
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
bool RunMotor(char dir, double new_speed, char agent[]);
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
double CheckPixy(bool do_test_check = false);
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
// DO HARDWARE TEST
void HardwareTest();
// CHECK LOOP TIME AND MEMORY
void CheckLoop();
// PRINT ALL FUNCTION ENTRY AND EXITS: str_where = ["start", "end"]
void DebugAllFun(const char *fun, int line, int mem);
// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t = millis());
// LOG/PRINT ERRORS
void DebugError(char msg[], bool is_error = false, uint32_t t = millis());
// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[]);
// LOG/PRINT MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(char msg[], char called_from[], uint32_t t = millis());
// LOG/PRINT MOTOR SPEED CHANGE
void DebugRunSpeed(char agent[], double speed_last, double speed_now);
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
int GetAD_Status(uint16_t stat_reg, char stat_str[]);
// SEND TEST PACKET
void TestSendPack(R2 *r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf);
// HOLD FOR CRITICAL ERRORS
void RunErrorHold(char msg[], uint32_t t_kill = 0);
// GET ID INDEX
template <typename T> int CharInd(char id, T *r24);
// BLINK LEDS AT RESTART/UPLOAD
bool StatusBlink(bool do_set = false, byte n_blinks = 0, uint16_t dt_led = 0, bool rat_in_blink = false);
// PRINT ALL IN QUEUE : fun_id = ["PrintDebug", "WriteLog"]
bool DoAll(char fun_id[]);
// TIMER INTERUPT/HANDLER
void Interupt_TimerHandler();
// POWER OFF
void Interupt_Power();
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
	strcpy(this->instID, obj_id);
	this->nSamp = n_samp;

	for (int i = 0; i < n_samp; i++) {
		this->posArr[i] = 0.0f;
	}
}

void POSTRACK::UpdatePos(double pos_new, uint32_t ts_new)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	char str[maxStoreStrLng] = { 0 };
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
	this->posArr[this->nSamp - 1] = pos_new;
	this->t_tsArr[this->nSamp - 1] = ts_new;

	// Do not process early samples
	if (this->sampCnt < this->nSamp + 1) {

		this->posNow = this->posNow;
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
	this->posNow = pos_new + this->nLaps*(140 * PI);

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
		dt_sec == 0 ||
		vel_diff > 300;

	// Log/print error
	if (is_error) {

		// Add to count
		this->cnt_error++;

		// Log/print first and every 10 errors
		if (this->cnt_error == 1 ||
			this->cnt_error % 10 == 0) {

			// Log/print error
			sprintf(str, "**WARNING** [POSTRACK::UpdatePos] Bad Values |%s%s: obj=\"%s\" cnt_err=%d pos_new=%0.2f pos_last=%0.2f dist_sum=%0.2f dt_sec=%0.2f vel_new=%0.2f vel_last=%0.2f",
				vel_diff > 300 ? "Vel|" : "", dt_sec == 0 ? "DT|" : "", this->instID, this->cnt_error, pos_new, this->posArr[this->nSamp - 2], dist_sum, dt_sec, vel, this->velLast);
			DebugError(str);
		}
	}
}

double POSTRACK::GetPos()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Reset flag
	this->isNew = false;

	// Return newest pos value
	return this->posNow;
}

double POSTRACK::GetVel()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Return newest vel value
	return this->velNow;
}

void POSTRACK::SwapPos(double set_pos, uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Make sure pos val range [0, 140*PI]
	set_pos = set_pos < 0 ? set_pos + (140 * PI) : set_pos;
	set_pos = set_pos > (140 * PI) ? set_pos - (140 * PI) : set_pos;

	// Compute ts
	uint32_t ts = this->t_tsNow + (t - this->t_msNow);

	// Update pos
	UpdatePos(set_pos, ts);
}

void POSTRACK::PosReset()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	this->sampCnt = 0;
	this->isNew = false;
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
	sprintf(this->modePID, "Manual");
}

double PID::UpdatePID()
{

	// Wait till ekf ready
	if (!fc.isEKFReady) {
		return -1;
	}

	// Compute error 
	error = kal.RatPos - (kal.RobPos + setPoint);
	errorFeeder = kal.RatPos - (kal.RobPos + feedDist);

	// Check if motor is open
	PID_CheckMotorControl();

	// Check throttling 
	CheckThrottle();

	// Check if not in auto mode
	if (strcmp(modePID, "Automatic") != 0) {
		return -1;
	}

	// Check if rat stopped behind setpoint
	if (kal.RatVel < 1 && error < -15 && !isHolding4cross) {
		// halt running
		return runSpeed = 0;
	}

	// Set throttle
	SetThrottle();

	// Check for setpoint crossing
	CheckSetpointCrossing();

	// Update Pid and speed
	if (
		!is_ekfNew || // New EKF data 
		isHolding4cross // wait for setpoint pass
		) {
		return -1;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	if (isFirstRun) {
		PrintPID("[PID::UpdatePID] First Run");
		isFirstRun = false;
	}

	// Re-compute error 
	error = kal.RatPos - (kal.RobPos + setPoint);

	// Compute new integral
	if (doIncludeTerm[0]) {

		// Catch setpoint (i.e. error == 0) crossing
		if ((abs(error) + abs(errorLast)) > abs(error + errorLast)) {

			integral = 0;
		}

		//else if (error < 0) integral = integral + error * 5;
		else {
			integral = integral + error;
		}
	}
	else {
		integral = 0;
	}

	// Compute new derivative
	if (doIncludeTerm[1]) {
		derivative = error - errorLast;
	}
	else {
		derivative = 0;
	}

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

	errorLast = error;
	isPidUpdated = true;
	is_ekfNew = false;

	// Return new run speed
	return runSpeed;

}

void PID::PID_Run(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[PID::Run] Run [%s]", called_from);
	PrintPID(str);

	// Take motor control
	SetMotorControl("Pid", "PID::Run");

	// Reset
	PID_Reset();
	sprintf(modePID, "Automatic");

	// Tell ard pid is running
	QueuePacket(&r2a, 'p', 1);
}

void PID::PID_Stop(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[PID::Stop] Stop [%s]", called_from);
	PrintPID(str);

	if (strcmp(fc.motorControl, "Pid") == 0)
	{
		// Stop movement
		RunMotor('f', 0, "Pid");

		// Set run speed
		runSpeed = 0;

		// Give over control
		SetMotorControl("Open", "PID::Stop");
	}

	// Tell ard pid is stopped if not rewarding
	if (!Reward.isRewarding) {
		QueuePacket(&r2a, 'p', 0);
	}

	// Set mode
	sprintf(modePID, "Manual");
}

void PID::PID_Hold(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[PID::Hold] Hold [%s]", called_from);
	PrintPID(str);

	// Call Stop
	PID_Stop("Pid.Hold");

	// But set mode to "Hold"
	sprintf(modePID, "Hold");
}

void PID::PID_Reset()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	integral = 0;
	t_updateLast = millis();
	isHolding4cross = true;
	doThrottle = true;
}

void PID::SetThrottle()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	if (!doThrottle) {
		return;
	}

	// Only run if rat ahead of setpoint
	if (
		error < 5 &&
		millis() < t_lastThrottle + 5000
		) {
		doThrottle = false;
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Log/print
	sprintf(str, "[PID::SetThrottle] Throttle ACC to %0.2fcm/sec", throttleAcc);
	PrintPID(str);

	// Change acc to rat pos
	AD_Reset(maxSpeed, throttleAcc, maxDec);

	// Set time to throttle till
	t_throttleTill = millis() + dt_throttle;

	// Set flags
	isThrottled = true;
	doThrottle = false;

}

void PID::CheckThrottle()
{

	// Bail if not active
	if (!isThrottled) {
		return;
	}

	// Bail if not ready to stop
	if (
		!(millis() >= t_throttleTill ||
			kal.RatVel > throttleSpeedStop ||
			error < 0)
		) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Log/print event
	PrintPID("[PID::CheckThrottle] Finished Throttle");

	// Set acc back to normal
	AD_Reset(maxSpeed, maxAcc, maxDec);

	// Store time
	t_lastThrottle = millis();

	// Reset flag
	isThrottled = false;
}

void PID::PID_CheckMotorControl()
{

	// Check if motor control available
	if ((strcmp(fc.motorControl, "Pid") == 0 || strcmp(fc.motorControl, "Open") == 0) &&
		strcmp(modePID, "Hold") == 0) {

		// Print taking conrol
		PrintPID("[PID::CheckMotorControl] Take Motor Control [PID::CheckMotorControl]");

		// Run pid
		PID_Run("PID::CheckMotorControl");
	}
	else if ((strcmp(fc.motorControl, "Pid") != 0 && strcmp(fc.motorControl, "Open") != 0) &&
		strcmp(modePID, "Automatic") == 0) {

		// Print taking conrol
		PrintPID("[PID::CheckMotorControl] Surender Motor Control [PID::CheckMotorControl]");

		// Hold pid
		PID_Hold("PID::CheckMotorControl");
	}
}

void PID::CheckSetpointCrossing()
{

	// Bail if not holding for cross
	if (!isHolding4cross) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Check if rat has moved in front of setpoint
	if (error > 0)
	{
		// Log/print event 
		PrintPID("[PID::CheckSetpointCrossing] Crossed Setpoint");

		// Set flag
		isHolding4cross = false;
	}
}

void PID::PID_CheckEKF(uint32_t t)
{
	// Bail if not checking
	if (fc.isEKFReady)
	{
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	if ((t - t_ekfReady) > dt_ekfSettle)
	{
		// Log/print event 
		PrintPID("[PID::CheckEKF] EKF Ready");

		// Set flag
		fc.isEKFReady = true;
	}
}

void PID::PID_ResetEKF(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[PID::ResetEKF] Reset EKF [%s]", called_from);
	PrintPID(str);

	// Set flag and time
	fc.isEKFReady = false;
	t_ekfReady = millis();
}

void PID::PID_SetUpdateTime(uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	is_ekfNew = true;
	if (isPidUpdated)
	{
		dt_update = (double)(t - t_updateLast) / 1000;
		isPidUpdated = false;
		t_updateLast = t;
	}
}

void PID::PrintPID(char msg[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Add to print queue
	if (db.print_pid && (db.CONSOLE || db.LCD)) {
		QueueDebug(msg, millis());
	}
	// Add to log queue
	if (db.log_pid && DO_LOG) {
		Log.QueueLog(msg, millis());
	}
}

double PID::RunCalibration()
{
#if DO_DEBUG_XXX
	DB_INF();
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
	static char str[200] = { 0 }; str[0] = '\0';
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

		// Log/print
		DebugFlow("[PID::RunPidCalibration] Finished PID Calibration");

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

		// Log/print
		sprintf(str, "[PID::RunPidCalibration] Set Speed to %0.2fcm/sec", cal_speedSteps[cal_stepNow]);
		DebugFlow(str);
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

BULLDOZE::BULLDOZE(uint32_t t)
{
	this->t_init = t;
	sprintf(this->modeBull, "Inactive");
	sprintf(this->stateBull, "Off");
}

void BULLDOZE::UpdateBull()
{
	// Check who has motor control
	BullCheckMotorControl();

	// Bail if off
	if (strcmp(stateBull, "Off") == 0)
	{
		return;
	}

	// Bail if not ready
	if (
		!(fc.isEKFReady &&
			millis() > t_updateNext))
	{
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Update rat pos
	posNow = kal.RatPos;
	guardPos = kal.RobPos + guardDist;

	// Get distance traveled
	distMoved = posNow - posCheck;

	// Check for movement
	isMoved = distMoved >= moveMin ? true : false;

	// Check if rat passed reset
	double error = kal.RatPos - (kal.RobPos + pidSetPoint);
	isPassedReset = error > 1 ? true : false;

	// Check time
	isTimeUp = millis() > t_bullNext ? true : false;

	// Check if has not moved in time
	if (!isMoved) {

		// Bulldoze him!
		if (isTimeUp &&
			strcmp(modeBull, "Inactive") == 0) {
			BullRun("BULLDOZE::UpdateBull");
		}
	}

	// Has moved minimal distance
	else {
		// Reset check pos
		posCheck = posNow;

		// Reset bull next
		t_bullNext = millis() + bDelay;

		// Stop bulldoze if rat ahead of set point and not 0 delay
		if (isPassedReset &&
			strcmp(modeBull, "Active") == 0 &&
			bDelay != 0) {

			BullStop("BULLDOZE::UpdateBull");
		}
	}

	// Set next update time
	t_updateNext = millis() + dt_update;

}

void BULLDOZE::BullReinitialize(byte del, byte spd, char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[BULLDOZE::Reinitialize] Reinitialize Bull: del=%d spd=%d [%s]", del, spd, called_from);
	PrintBull(str);

	// Update vars
	bSpeed = (int)spd;
	bDelay = (int)del * 1000;
	t_bullNext = millis() + bDelay;
	posCheck = kal.RatPos;
}

void BULLDOZE::BullRun(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[BULLDOZE::Run] Run [%s]", called_from);
	PrintBull(str);

	// Take control
	SetMotorControl("Bull", "BULLDOZE::Run");

	// Start bulldozer
	RunMotor('f', bSpeed, "Bull");

	// Tell ard bull is running
	QueuePacket(&r2a, 'b', 1);

	// Set mode
	sprintf(modeBull, "Active");
}

void BULLDOZE::BullStop(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[BULLDOZE::Stop] Stop [%s]", called_from);
	PrintBull(str);

	// Stop movement
	RunMotor('f', 0, "Bull");

	// Give over control
	if (strcmp(fc.motorControl, "Bull") == 0) {

		// Stop movement
		RunMotor('f', 0, "Bull");

		// Give over control
		SetMotorControl("Open", "BULLDOZE::Stop");;
	}

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Tell ard bull is stopped
	QueuePacket(&r2a, 'b', 0);

	// Set mode
	sprintf(modeBull, "Inactive");
}

void BULLDOZE::BullOn(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[BULLDOZE::TurnOn] Turn On [%s]", called_from);
	PrintBull(str);

	// Change state
	sprintf(stateBull, "On");

	// Reset 
	BullReset();
}

void BULLDOZE::BullOff(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[BULLDOZE::TurnOff] Turn Off [%s]", called_from);
	PrintBull(str);

	// Change state
	sprintf(stateBull, "Off");

	// Stop bulldozer if running
	if (strcmp(modeBull, "Active") == 0) {
		// Stop bull
		BullStop("BULLDOZE::TurnOff");
	}
}

void BULLDOZE::BullHold(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "bull: hold [%s]", called_from);
	PrintBull(str);

	// Change state
	sprintf(stateBull, "Hold");

	// Stop running
	if (strcmp(modeBull, "Active") == 0) {

		// Run stop bulldozer
		BullStop("BULLDOZE::Hold");

		// Set mode back to active for later
		sprintf(modeBull, "Active");
	}
}

void BULLDOZE::BullResume(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[BULLDOZE::Resume] Resume [%s]", called_from);
	PrintBull(str);

	// Set state back to "On"
	sprintf(stateBull, "On");

	// Reset 
	BullReset();
}

void BULLDOZE::BullReset()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Set mode
	sprintf(modeBull, "Inactive");

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Reset check pos
	posCheck = kal.RatPos;
}

void BULLDOZE::BullCheckMotorControl()
{

	if ((strcmp(fc.motorControl, "Bull") == 0 || strcmp(fc.motorControl, "Pid") == 0 || strcmp(fc.motorControl, "Open") == 0) &&
		strcmp(stateBull, "Hold") == 0) {

		// Turn bull on
		BullResume("BULLDOZE::CheckMotorControl");
	}
	else if ((fc.motorControl != "Bull" && strcmp(fc.motorControl, "Pid") != 0 && strcmp(fc.motorControl, "Open") != 0) &&
		strcmp(stateBull, "On") == 0) {

		// Turn bull off
		BullHold("BULLDOZE::CheckMotorControl");
	}
}

void BULLDOZE::PrintBull(char msg[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Add to print queue
	if (db.print_bull && (db.CONSOLE || db.LCD)) {
		QueueDebug(msg, millis());
	}
	// Add to log queue
	if (db.log_bull && DO_LOG) {
		Log.QueueLog(msg, millis());
	}
}

#pragma endregion 

#pragma region ----------CLASS: MOVETO----------

MOVETO::MOVETO(uint32_t t)
{
	this->t_init = t;
}

bool MOVETO::CompTarg(double now_pos, double targ_pos)
{

	// Run only if targ not set
	if (isTargSet) {
		return isTargSet;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	double move_diff = 0;
	int circ = 0;
	int pos = 0;

	// Get abort timeout
	if (t_tryTargSetTill == 0) {
		t_tryTargSetTill = millis() + targSetTimeout;
	}

	// Check if time out reached
	if (millis() > t_tryTargSetTill) {
		doAbortMove = true;

		// Log/print error
		sprintf(str, "!!ERROR!! [MOVETO::CompTarg] Timedout after %dms", targSetTimeout);
		DebugError(str, true);

		// Return failure
		return isTargSet;
	}

	// Bail if ekf pos data not ready
	if (!fc.isEKFReady) {
		return isTargSet;
	}

	// Current absolute pos on track
	circ = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posAbs = (double)(pos % circ) / 100;

	// Diff and absolute distance
	move_diff = targ_pos - posAbs;
	move_diff = move_diff < 0 ? move_diff + (140 * PI) : move_diff;

	// Allow for reverse move
	if (fc.doAllowRevMove) {

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
	}

	// Only move forward
	else {
		targDist = move_diff;
		moveDir = 'f';
	}

	// Set vars for later
	t_updateNext = millis();
	baseSpeed = 0;

	// Copy to public vars
	targPos = targ_pos;
	posCumStart = now_pos;
	posAbsStart = posAbs;

	// Set flag true
	isTargSet = true;

	// Log/print
	sprintf(str, "[MOVETO::CompTarg] FINISHED: Set Target: start_cum=%0.2fcm start_abs=%0.2fcm targ=%0.2fcm dist_move=%0.2fcm move_dir=\'%c\'",
		posCumStart, posAbsStart, targPos, targDist, moveDir);
	DebugFlow(str);

	// Retern flag
	return isTargSet;
}

double MOVETO::DecelToTarg(double now_pos, double now_vel, double dist_decelerate, double speed_min)
{

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	double new_speed = 0;

	// Run if targ not reached
	if (isTargReached) {
		return 0;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Get abort timeout
	if (t_tryMoveTill == 0) {
		t_tryMoveTill = millis() + moveTimeout;
	}

	// Check if time out reached
	if (millis() > t_tryMoveTill) {

		// Log/print error
		sprintf(str, "!!ERROR!! [MOVETO::DecelToTarg] Timedout after %dms", moveTimeout);
		DebugError(str, true);

		// Set error flag
		doAbortMove = true;

		// Stop run
		return 0;
	}

	// Compute remaining distance
	distLeft = targDist - abs(now_pos - posCumStart);

	// Check if rob is dec_pos cm from target
	if (distLeft <= dist_decelerate) {

		// Get base speed to decelerate from
		if (baseSpeed == 0) {
			baseSpeed = abs(now_vel);
		}

		// Update decel speed
		else if (millis() > t_updateNext) {
			// Compute new speed so range constrained to [min_speed, base_speed]
			new_speed = (((baseSpeed - speed_min) * distLeft) / dist_decelerate) + speed_min;

			// Get next update time
			t_updateNext = millis() + dt_update;
		}

	}

	// Target reached
	if (distLeft < 1)
	{
		// Log/print
		sprintf(str, "[MOVETO::DecelToTarg] FINISHED: MoveTo: start_abs=%0.2fcm now_abs=%0.2f targ=%0.2fcm dist_move=%0.2fcm dist_left=%0.2fcm",
			posAbsStart, now_pos, targPos, targDist, distLeft);
		DebugFlow(str);

		// Set flag true
		isTargReached = true;

		// Stop movement
		new_speed = 0;
	}

	// Return new speed
	return new_speed;
}

double MOVETO::GetMoveError(double now_pos)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	int diam = 0;
	int pos = 0;

	// Current relative pos on track
	diam = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posAbs = (double)(pos % diam) / 100;

	// Target error
	return targPos - posAbs;
}

void MOVETO::MoveToReset()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

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
	this->duration = durationDefault;
	this->durationByte = (byte)(duration / 10);
	sprintf(this->modeReward, "None");
}

void REWARD::StartRew()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Set to extend feeder arm 
	ExtendFeedArm();

	// Hard stop
	HardStop("StartRew");

	// Set hold time
	BlockMotorTill(dt_rewBlock, "REWARD::StartRew");

	// Store and send packet imediately if coms setup
	if (fc.isSesStarted) {
		QueuePacket(&r2a, 'r', duration);
		SendPacket(&r2a);
	}

	// Turn on reward LED
	analogWrite(pin.RewLED_R, round(rewLEDduty*0.75));
	analogWrite(pin.RewLED_C, rewLEDduty);

	// Open solenoid
	digitalWrite(pin.Rel_Rew, HIGH);

	// Compute reward end time
	t_rew_str = millis();
	t_closeSol = t_rew_str + duration;

	// Compute retract arm time for non-button reward
	if (strcmp(modeReward, "Button") != 0) {
		t_retractArm = t_rew_str + dt_rewBlock;
		doTimedRetract = true;
	}
	else {
		doTimedRetract = false;
	}

	// Log/print 
	sprintf(str, "[REWARD::StartRew] RUNNING: \"%s\" Reward: dt_rew=%dms dt_retract=%d...",
		modeReward, duration, doTimedRetract ? t_retractArm - t_rew_str : 0);
	DebugFlow(str, t_rew_str);


	// Print to LCD for manual rewards
	if (strcmp(modeReward, "Button") == 0) {
		PrintLCD(true, "REWARDING...");
	}

	// Set flags
	isRewarding = true;

}

bool REWARD::EndRew()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Bail if not rewarding
	if (!isRewarding) {
		return false;
	}

	// Bail if time not up
	if (millis() < t_closeSol) {
		return false;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Close solenoid
	digitalWrite(pin.Rel_Rew, LOW);

	// Turn off reward LED
	analogWrite(pin.RewLED_R, rewLEDmin);
	analogWrite(pin.RewLED_C, rewLEDmin);

	// Store time
	t_rew_end = millis();

	// Log/print
	sprintf(str, "[REWARD::EndRew] FINISHED: \"%s\" Reward: dt_rew=%dms dt_retract=%d",
		modeReward, t_rew_end - t_rew_str, doTimedRetract ? t_retractArm - t_rew_str : 0);
	DebugFlow(str, t_rew_end);

	// Clear LCD
	if (strcmp(modeReward, "Button") == 0) {
		ClearLCD();
	}

	// Reset flags etc
	RewardReset();

	// Return end reward status
	return true;

}

void REWARD::SetRewDur(int zone_ind)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Set duration
	duration = zoneRewDurs[zone_ind];

	// Save zone ind
	zoneInd = zone_ind;

	// Log/print
	sprintf(str, "[REWARD::SetRewDur] Set Reward Diration: zone_ind=%d duration=%d",
		zone_ind, duration);
	DebugFlow(str);
}

void REWARD::SetRewMode(char mode_now[], int arg2)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// NOTE: arg2 = reward delay or zone ind or reward duration

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static char dat_str[100] = { 0 }; dat_str[0] = '\0';

	// Store mode
	sprintf(modeReward, "%s", mode_now);

	// Store info
	if (strcmp(modeReward, "Button") == 0) {

		// Set duration to default
		duration = durationDefault;
		sprintf(dat_str, "mode=\"Button\" duration=%d", duration);
	}
	else if (strcmp(modeReward, "Now") == 0) {

		// Set duration
		SetRewDur(arg2);
		sprintf(dat_str, "mode=\"Now\" duration=%d", duration);

		// Change duration default
		durationDefault = duration;
	}
	else if (strcmp(modeReward, "Free") == 0) {

		// Include all zones
		zoneMin = 0;
		zoneMax = zoneLng - 1;

		// Store threshold
		occThresh = arg2 * 1000;
		sprintf(dat_str, "mode=\"Free\" occ_thresh=%d", occThresh);
	}
	else if (strcmp(modeReward, "Cue") == 0) {

		// Include one zone
		zoneMin = arg2;
		zoneMax = arg2;

		// Store zone ind
		occThresh = 0;
		sprintf(dat_str, "mode=\"Cue\" zone_ind=%d", arg2);
	}

	// Log/print
	sprintf(str, "[REWARD::SetRewMode] Set Reward Mode: %s", dat_str);
	DebugFlow(str);
}

bool REWARD::CompZoneBounds(double now_pos, double rew_pos)
{
	// Local vars
	int diam = 0;
	int pos_int = 0;
	double pos_rel = 0;
	double dist_center_cm = 0;
	double dist_start_cm = 0;
	double dist_end_cm = 0;

	// Run only if bounds are not set
	if (isBoundsSet) {
		return isBoundsSet;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

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

	return isBoundsSet;
}

bool REWARD::CheckZoneBounds(double now_pos)
{

	// Run only if reward not already triggered
	if (isZoneTriggered) {
		return isZoneTriggered;
	}

	// Bail if pos data not new
	if (!is_ekfNew) {
		return isZoneTriggered;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Reset flag
	is_ekfNew = false;

	// Check if all bounds passed
	if (now_pos > boundMax + 5) {
		isAllZonePassed = true;
		return isZoneTriggered;
	}

	// Bail if first bound not reached
	if (now_pos < boundMin) {
		return isZoneTriggered;
	}

	// Check if rat in any bounds
	for (int i = zoneMin; i <= zoneMax; i++)
	{
		if (
			now_pos > zoneBounds[i][0] &&
			now_pos < zoneBounds[i][1]
			) {

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

	return isZoneTriggered;
}

void REWARD::ExtendFeedArm()
{

	// Step mode:
	/*
	MS1	MS2	MS3	Microstep Resolution	Excitation Mode
	L	L	L	Full Step	2 Phase
	H	L	L	Half Step	1 - 2 Phase
	L	H	L	Quarter Step	W1 - 2 Phase
	H	H	L	Eigth Step	2W1 - 2 Phase
	H	H	H	Sixteenth Step	4W1 - 2 Phase
	*/

	// Bail if arm already extended
	if (isArmExtended) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Log/print
	DebugFlow("[REWARD::ExtendFeedArm] Set Extend Feed Arm");

	// Block handler
	v_doStepTimer = false;
	delayMicroseconds(1100);

	// Set targ and flag
	v_stepTarg = armExtStps;
	doExtendArm = true;
	v_isArmMoveDone = false;

	// Wake motor
	digitalWrite(pin.ED_SLP, HIGH);

	// Make sure step low
	digitalWrite(pin.ED_STP, LOW);

	// Set to half step
	digitalWrite(pin.ED_MS1, HIGH);
	digitalWrite(pin.ED_MS2, LOW);
	digitalWrite(pin.ED_MS3, LOW);

	// Set direction to extend
	digitalWrite(pin.ED_DIR, LOW);
	v_stepDir = 'e';

	// Store start time
	t_moveArmStr = millis();

	// Set intterupt flag
	v_doStepTimer = true;
	delayMicroseconds(100);

	// Start timer
	FeederArmTimer.start();

}

void REWARD::RetractFeedArm()
{
#if DO_DEBUG_XXX
	DB_INF();
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

	// Log/print
	DebugFlow("[REWARD::RetractFeedArm] Set Retract Feed Arm");

	// Block handler
	v_doStepTimer = false;
	delayMicroseconds(1100);

	// Set targ and flag
	v_stepTarg = 0;
	doRetractArm = true;
	v_isArmMoveDone = false;

	// Wake motor
	digitalWrite(pin.ED_SLP, HIGH);

	// Make sure step low
	digitalWrite(pin.ED_STP, LOW);

	// Set to quarter step
	digitalWrite(pin.ED_MS1, LOW);
	digitalWrite(pin.ED_MS2, HIGH);
	digitalWrite(pin.ED_MS3, LOW);

	// Set direction to retract
	digitalWrite(pin.ED_DIR, HIGH);
	v_stepDir = 'r';

	// Store start time
	t_moveArmStr = millis();

	// Set intterupt flag
	v_doStepTimer = true;
	delayMicroseconds(100);

	// Start timer
	FeederArmTimer.start();

}

void REWARD::CheckFeedArm()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	bool is_move_done = false;
	bool is_timedout = false;

	// Bail if still nothing to do
	if (!doExtendArm &&
		!doRetractArm &&
		!doTimedRetract)
	{
		// Make sure motor asleep
		if (digitalRead(pin.ED_SLP) == HIGH) {
			digitalWrite(pin.ED_SLP, LOW);
		}
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Do timed retract
	if (doTimedRetract) {

		// Check if its time to retract arm
		if (millis() > t_retractArm) {

			// Log/print
			sprintf(str, "[REWARD::CheckFeedArm] Time to Retract Feeder Arm: dt_rew=%d",
				millis() - t_rew_str);
			DebugFlow(str);

			// Set to retract
			RetractFeedArm();

			// Reset flag
			doTimedRetract = false;
		}
	}

	// Bail if still nothing to do
	if (!doExtendArm &&
		!doRetractArm)
	{
		// Make sure motor asleep
		if (digitalRead(pin.ED_SLP) == HIGH) {
			// Sleep motor
			digitalWrite(pin.ED_SLP, LOW);
		}
		return;
	}

	// Release switch when switch triggered on retract
	if (v_stepDir == 'r' &&
		digitalRead(pin.FeedSwitch) == LOW) {

		// Block handler
		v_doStepTimer = false;
		delayMicroseconds(500);

		// Stop timer
		FeederArmTimer.stop();

		// Make sure step off
		v_stepState = false;
		digitalWrite(pin.ED_STP, v_stepState);

		// Set direction to extend
		digitalWrite(pin.ED_DIR, LOW);
		delayMicroseconds(100);

		// Move arm x steps
		for (int i = 0; i < 20; i++)
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
	if ((doExtendArm || doRetractArm) &&
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
		delayMicroseconds(500);

		// Stop timer
		FeederArmTimer.stop();

		// Unstep motor
		if (digitalRead(pin.ED_STP) == HIGH) {
			digitalWrite(pin.ED_STP, LOW);
		}

		// Sleep motor
		digitalWrite(pin.ED_SLP, LOW);

		// Set arm state flags
		if (doExtendArm) {
			isArmExtended = true;
		}
		else if (doRetractArm) {
			isArmExtended = false;
		}

		// Log/print status
		if (!is_timedout) {

			// Print success
			sprintf(str, "[REWARD::CheckFeedArm] SUCCEEDED: Arm %s: cnt_steps=%d step_targ=%d dt_move=%d",
				isArmExtended ? "Extend" : "Retract", v_cnt_steps, v_stepTarg, millis() - t_moveArmStr);
			DebugFlow(str);
		}
		else {

			// Print timeout error
			sprintf(str, "!!ERROR!! [REWARD::CheckFeedArm] TIMEDOUT: Arm %s: cnt_steps=%d step_targ=%d dt_move=%d",
				isArmExtended ? "Extend" : "Retract", v_cnt_steps, v_stepTarg, millis() - t_moveArmStr);
			DebugError(str, true);
		}

		// Reset pos time and flags
		v_cnt_steps = 0;
		v_stepTarg = 0;
		t_moveArmStr = 0;
		isArmStpOn = false;
		doExtendArm = false;
		doRetractArm = false;
		v_isArmMoveDone = true;
	}
}

void REWARD::RewardReset()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str1[200] = { 0 }; str1[0] = '\0';
	static char str2[200] = { 0 }; str2[0] = '\0';
	static char ss1[50]; ss1[0] = '\0';
	static char ss2[50]; ss2[0] = '\0';

	// Log/print event
	DebugFlow("[REWARD::Reset] Reseting Reward");

	// Log zone info
	if (strcmp(modeReward, "Free") == 0 || strcmp(modeReward, "Cue") == 0)
	{
		sprintf(str1, "[REWARD::Reset] ZONE OCC:");
		sprintf(str2, "[REWARD::Reset] ZONE CNT:");
		for (int i = zoneMin; i <= zoneMax; i++)
		{
			sprintf(ss1, " z%d=%dms", i + 1, zoneOccTim[i]);
			strcat(str1, ss1);
			sprintf(ss2, " z%d=%d", i + 1, zoneOccCnt[i]);
			strcat(str2, ss2);
		}
		DebugFlow(str1);
		DebugFlow(str2);
	}

	// Reset flags etc
	sprintf(modeReward, "None");
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
#if DO_DEBUG_XXX
	DB_INF();
#endif
	/*
	NOTE:
	config.txt settings:
	57600,26,3,2,0,0,0
	baud,escape,esc#,mode,verb,echo,ignoreRX
	*/

	// Local vars
	byte match = '\0';

	// Start serial
	port.begin(57600);

	// Reset OpenLog
	t_sent = millis();
	digitalWrite(pin.OL_RST, HIGH);
	delay(100);
	digitalWrite(pin.OL_RST, LOW);
	delay(100);
	match = GetReply(5000);

	// Bail if setup failed
	if (match != '>' && match != '<') {
		DebugError("!!ERROR!! [LOGGER::Setup] ABORTING: Did Not Get Initial \'>\' or \'<\'", true);
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
	else {
		strcpy(openLgSettings, rcvdArr);
	}

	return true;
}

int LOGGER::OpenNewLog()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	static char file_str[50] = { 0 }; file_str[0] = '\0';
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
		DebugFlow("[OpenNewLog] Made \"LOGS\" Directory");

		// Make new log count file
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGCNT.TXT\" File");
	}

	// Get log count
	else if (SendCommand("read LOGCNT.TXT\r") != '!') {
		// Store count
		logNum = atoi(logCntStr) + 1;
	}
	else {
		return 0;
	}


	// Check if more than 100 logs saved
	if (logNum > 100)
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
		sprintf(str, "[OpenNewLog] Deleted Log Directory: log_count=%d", logNum);
		DebugFlow(str);

		// Make new log dir
		if (SendCommand("md LOGS\r") == '!') {
			return 0;
		}
		if (SendCommand("cd LOGS\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGS\" Directory");

		// Make new log count file
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGCNT.TXT\" File");
	}

	// Update count
	if (SendCommand("write LOGCNT.TXT\r") == '!') {
		return 0;
	}
	// write int string
	sprintf(str, "{{%d}}\r", logNum);
	SendCommand(str);
	// exit with empty line
	str[0] = '\r';
	str[1] = '\0';
	if (SendCommand(str) == '!') {
		return 0;
	}

	// Create new log file
	sprintf(file_str, "LOG%05u.CSV", logNum);

	// Begin logging to this file
	if (!SetToWriteMode(file_str)) {
		return 0;
	}

	// Write first log entry
	if (DO_LOG) {
		sprintf(str, "[OpenNewLog] Begin Logging to \"%s\"", logFile);
		QueueLog(str);
	}

	// Set flag
	isFileReady = true;

	// Return log number
	return logNum;

}

bool LOGGER::SetToCmdMode()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	bool pass = false;
	char rstArr[3] = { 26,26,26 };

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

	//Send "$$$" command mode
	if (SendCommand(rstArr, true, 500) == '>')
	{
		// Set flag
		pass = true;

		// Pause to let OpenLog get its shit together
		delay(100);

		// Print status
		sprintf(str, "[LOGGER::SetToCmdMode] OpenLog Set to Cmd Mode: mode = %c", mode);
		DebugFlow(str);
	}
	// Log/print error
	else {
		// Set flag
		pass = false;

		// Log/print
		sprintf(str, "**WARNING** [LOGGER::SetToCmdMode] ABORTED: mode=%c", mode);
		DebugError(str);
	}

	return pass;
}

void LOGGER::GetCommand()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
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

char LOGGER::SendCommand(char msg[], bool do_conf, uint32_t timeout)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static char msg_copy[maxStoreStrLng] = { 0 }; msg_copy[0] = '\0';
	char reply = '\0';

	// Add min delay
	int del = 15 - (millis() - t_sent);
	if (del > 0) {
		delay(del);
	}

	// Copy message
	for (int i = 0; i < strlen(msg); i++)
	{
		msg_copy[i] = msg[i];
	}
	msg_copy[strlen(msg)] = '\0';

	// Send
	port.write(msg_copy);
	t_sent = millis();

	// Print sent
	if (db.print_a2o)
	{
		PrintLOGGER("SENT[=======================", true);
		for (int i = 0; i < strlen(msg_copy) + 1; i++) {
			PrintLOGGER(PrintSpecialChars(msg_copy[i]));
		}
		PrintLOGGER("\n=======================]SENT\n\n");
	}

	// Get confirmation
	if (do_conf) {
		reply = GetReply(timeout);
		// Check for error
		if (reply == '!') {
			// Remove '\r'
			msg_copy[strlen(msg_copy) - 1] = msg_copy[strlen(msg_copy) - 1] == '\r' ? '\0' : msg_copy[strlen(msg_copy) - 1];
			sprintf(str, "**WARNING** [LOGGER::SendCommand] Command %s Failed", msg_copy);
			PrintLOGGER(str, true);
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
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	uint32_t t_start = millis();
	uint32_t t_timeout = millis() + timeout;
	int dat_ind[2] = { 0,0 };
	int arr_ind = -1;
	char cmd_reply = ' ';
	bool pass = false;

	// Wait for new data
	while (
		port.available() == 0 &&
		millis() < t_timeout
		);
	t_rcvd = millis();

	// Check for match byte
	if (db.print_o2a) {
		PrintLOGGER("[LOGGER::GetReply] RCVD_FORMATED[==========", true);
	}
	while (
		!pass &&
		millis() < t_timeout
		)
	{
		// Get new data
		if (port.available() > 0)
		{
			// Get next byte
			char c = port.read();

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
				) {
				arr_ind++;
			}

			// Store data
			rcvdArr[arr_ind] = c;
		}
		// Make sure message finished
		else if (arr_ind >= 0) {
			bool p_arr[3] = { false, false, false };
			for (int i = arr_ind; i >= 0; i--)
			{
				// Check that certain specific comnination of chars recieved
				p_arr[0] = rcvdArr[i] == '\r' || rcvdArr[i] == '1' || rcvdArr[i] == '~' ? true : p_arr[0];
				p_arr[1] = rcvdArr[i] == '\n' || rcvdArr[i] == '2' || rcvdArr[i] == '~' ? true : p_arr[1];
				p_arr[2] = rcvdArr[i] == '>' || rcvdArr[i] == '<' ? true : p_arr[2];
			}
			if (p_arr[0] && p_arr[1] && p_arr[2]) {
				pass = true;
			}
		}
	}
	// Set null terminator
	rcvdArr[arr_ind + 1] = '\0';

	if (db.print_o2a) {
		PrintLOGGER("\n============================================]RCVD_FORMATED\n");
	}

	// Print formated string
	if (db.print_o2aRaw) {
		PrintLOGGER("[LOGGER::GetReply] RCVD_RAW[==========", true);
		for (int i = 0; i <= arr_ind + 1; i++)
		{
			//sprintf(str, "[%d]\'%s\'", rcvdArr[i], PrintSpecialChars(rcvdArr[i]));
			sprintf(str, "\'%s\'", PrintSpecialChars(rcvdArr[i]));
			PrintLOGGER(str);
		}
		PrintLOGGER("\n============================================]RCVD_RAW\n");
	}

	// Save values
	for (int i = 0; i <= arr_ind; i++)
	{
		// Save cmd 
		if (
			rcvdArr[i] == '!' ||
			rcvdArr[i] == '<' ||
			rcvdArr[i] == '>'
			) {
			cmd_reply = cmd_reply != '!' ? rcvdArr[i] : cmd_reply;
		}

		// Check for data start indeces
		dat_ind[0] =
			dat_ind[0] == 0 && rcvdArr[i - 1] == '{' && rcvdArr[i] == '{' ?
			i + 1 : dat_ind[0];
		// Check for end indeces
		if (
			dat_ind[0] > 0 &&
			rcvdArr[i] == '}' &&
			rcvdArr[i + 1] == '}'
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
			logCntStr[ii] = rcvdArr[i];
			ii++;
		}
		logCntStr[ii + 1] = '\0';
	}

	// Get current mode
	mode = cmd_reply == '>' || cmd_reply == '<' ? cmd_reply : mode;

	// Print mode and round trip time
	if (db.print_logMode || db.print_o2a || db.print_o2aRaw) {
		sprintf(str, "[LOGGER::GetReply] mode=\'%c\' reply=\'%c\' dt=%dms bytes=%d", mode, cmd_reply, t_rcvd - t_sent, arr_ind + 1);
		PrintLOGGER(str, true);
	}

	// Log/print error
	if (!pass) {
		sprintf(str, "**WARNING** [LOGGER::GetReply] Timedout: dt=%d bytes=%d", timeout, arr_ind + 1);
		PrintLOGGER(str, true);
	}

	// Return cmd 
	return cmd_reply;
}

bool LOGGER::SetToWriteMode(char log_file[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	bool pass = false;

	// Check if already in log mode
	if (mode == '<') {
		return true;
	}

	// Send append file command
	sprintf(str, "append %s\r", log_file);
	if (SendCommand(str) != '!') {
		pass = true;
	}

	// Store new log file
	if (pass) {
		strcpy(logFile, log_file);
		delay(100);
		sprintf(str, "[LOGGER::SetToWriteMode] OpenLog Set to Write Mode: file_name=%s mode = %c",
			log_file, mode);
		DebugFlow(str);
	}

	// Log/print error
	else {
		sprintf(str, "!!ERROR!! [LOGGER::SetToWriteMode] FAILED: OpenLog Set to Write Mode: file_name=%s mode = %c",
			log_file, mode);
		DebugError(str, true);
	}

	return pass;
}

void LOGGER::QueueLog(char msg[], uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

#if DO_LOG
	/*
	STORE LOG DATA FOR CS
	FORMAT: "[log_cnt],ts_ms,message,\r\n"
	*/

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static char msg_copy[maxStoreStrLng] = { 0 }; msg_copy[0] = '\0';
	static char queue_state[logQueueSize + 1] = { 0 }; queue_state[0] = '\0';
	bool is_queue_overflowed = false;
	bool is_mem_overflowed = false;
	uint32_t t_m = 0;

	// Bail if queue store blocked
	if (fc.doBlockLogQueue) {
		return;
	}

	// Update queue ind
	logQueueIndStore++;

	// Check if ind should roll over 
	if (logQueueIndStore == logQueueSize) {

		// Reset queueIndWrite
		logQueueIndStore = 0;
	}

	// Get message length
	is_mem_overflowed = strlen(msg) >= maxMsgStrLng;

	// Check for overflow
	is_queue_overflowed = logQueue[logQueueIndStore][0] != '\0';

	// Check if queue overflowed or message too long
	if (is_queue_overflowed || is_mem_overflowed) {

		// Handle overflow queue
		if (is_queue_overflowed) {

			// Get list of empty entries
			for (int i = 0; i < logQueueSize; i++) {
				queue_state[i] = logQueue[i][0] == '\0' ? '0' : '1';
			}
			queue_state[logQueueSize] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [LOGGER::QueueLog] LOG QUEUE OVERFLOWED: queue_s=%d queue_r=%d queue_state=|%s|",
				logQueueIndStore, logQueueIndRead, queue_state);

			// Set queue back so overflow will write over last log
			logQueueIndStore = logQueueIndStore - 1 >= 0 ? logQueueIndStore - 1 : logQueueSize - 1;
		}

		// Handle overflow char array
		else if (is_mem_overflowed) {

			// Store part of message
			for (int i = 0; i < 100; i++) {
				str[i] = msg[i];
			}
			str[100] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [LOGGER::QueueLog] MESSAGE TOO LONG: msg_lng=%d max_lng=%d \"%s%s\"",
				strlen(msg), maxMsgStrLng, str, "...");
		}

		// Print error
		if (db.print_errors && db.CONSOLE) {
			QueueDebug(msg_copy, t);
		}
	}

	// Copy message and Update log count
	else {
		for (int i = 0; i < strlen(msg); i++) {
			msg_copy[i] = msg[i];
		}
		msg_copy[strlen(msg)] = '\0';

		// Itterate count
		cnt_logsStored++;
	}

	// Get sync correction
	t_m = t - t_sync;

	// Store log
	sprintf(logQueue[logQueueIndStore], "[%d],%lu,%d,%s\r\n", cnt_logsStored, t_m, cnt_loop_short, msg_copy);

	// Check if should write now
	if (db.FASTLOG &&
		mode == '<') {
		DoAll("WriteLog");
	}

#endif
}

bool LOGGER::WriteLog(bool do_send)
{
#if DO_LOG
	/*
	STORE LOG DATA FOR CS
	FORMAT: "[log_cnt],ts_ms,message,\r\n"
	*/

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	bool is_logs = false;

	// Bail if no new logs
	if (logQueueIndRead == logQueueIndStore &&
		logQueue[logQueueIndStore][0] == '\0') {

		// Indicate no logs to write
		is_logs = false;
		return is_logs;
	}
	else {
		is_logs = true;
	}

	// Bail if writing blocked
	if (fc.doBlockLogWrite) {

		return is_logs;
	}

	// Bail if not in write mode
	if (mode != '<' &&
		!do_send) {

		// Return queue status
		return is_logs;
	}

	// Bail if less than 50ms sinse last write
	if (!db.FASTLOG && micros() < t_write + dt_write) {
		// Indicate still logs to store
		return is_logs;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Itterate send ind
	logQueueIndRead++;

	// Check if ind should roll over 
	if (logQueueIndRead == logQueueSize) {
		logQueueIndRead = 0;
	}

	// Write to SD
	if (mode == '<') {
		port.write(logQueue[logQueueIndRead]);
		t_write = micros();
	}

	// Send now
	if (do_send) {
		r2c.port.write(logQueue[logQueueIndRead]);
		cnt_logBytesSent++;
		delay(10);
	}

	// Update bytes stored
	cnt_logBytesStored += strlen(logQueue[logQueueIndRead]);

	// Print stored log
	if (db.print_logStore) {
		logQueue[logQueueIndRead][strlen(logQueue[logQueueIndRead]) - 2] = '\0';
		sprintf(str, "   [LOG] r2c: cnt=%d b_stored=%d/%d b_sent=%d q_store=%d q_read=%d \"%s\"",
			cnt_logsStored, strlen(logQueue[logQueueIndRead]), cnt_logBytesStored, cnt_logBytesSent, logQueueIndStore, logQueueIndRead, logQueue[logQueueIndRead]);
		QueueDebug(str, millis());
	}

	// Set entry to null
	logQueue[logQueueIndRead][0] = '\0';

	// Return queue status
	return is_logs;

#else
	return false;

#endif
}

void LOGGER::StreamLogs()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

#if DO_LOG

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char err_str[200] = { 0 }; err_str[0] = '|';
	static char warn_lines[200] = { 0 }; sprintf(warn_lines, "ON LINES |");
	static char err_lines[200] = { 0 }; sprintf(err_lines, "ON LINES |");
	const int timeout = 1200000;
	static int cnt_err_1 = 0;
	static int cnt_err_2 = 0;
	static int cnt_err_3 = 0;
	uint32_t t_start = millis(); // (ms)
	uint32_t t_last_read = millis(); // (ms)
	int read_ind = 0;
	bool head_passed = false;
	bool send_done = false;
	bool do_abort = false;
	bool is_timedout = false;
	char c_arr[3] = { 0 };
	int milestone_incriment[11] = { 0 };
	int milestone_ind = 0;

	// Bail if not ready to send
	if (millis() < t_beginSend) {
		return;
	}

	// Store remaining logs
	DoAll("WriteLog");
	delay(100);

	// Stop writing logs
	fc.doBlockLogWrite = true;

	// Make sure in command mode
	if (!SetToCmdMode()) {

		// Add to error counter
		cnt_err_1++;

		// Leave function
		if (cnt_err_1 < 3) {
			return;
		}

		// Abort after 3 failures
		else {
			do_abort = true;
		}

		// Store error
		if (strlen(err_str) < 200) {
			sprintf(str, "\"%c%c%c\" Failed: cnt=%d|",
				26, 26, 26, cnt_err_1);
			strcat(err_str, str);
		}
	}

	// Print anything left in queue
	DoAll("PrintDebug");

	// Start timers
	t_start = millis();

	// Get incriments to update status
	for (int i = 1; i < 10; i++)
	{
		milestone_incriment[i] = round((cnt_logBytesStored / 10) * (i + 1));
	}
	milestone_incriment[0] = 1;
	milestone_incriment[10] = cnt_logBytesStored;

	// Begin streaming data
	while (millis() < (t_start + timeout)) {

		// Dump anything in openlog buffer
		uint32_t t_out = millis() + 100;
		while (millis() < t_out || port.available() > 0) {
			if (port.available() > 0) {
				port.read();
				t_out += 100;
			}
		}

		// Send new "read" command
		if (!send_done &&
			!do_abort) {
			sprintf(str, "read %s %d %d\r", logFile, 0, (uint32_t)-1);
			SendCommand(str, false, 5000);
		}

		// Finished
		else {
			break;
		}

		// Reset vars
		head_passed = false;
		read_ind = 0;
		c_arr[0] = 0;
		c_arr[1] = 0;
		c_arr[2] = 0;
		t_last_read = millis();

		// Read one byte at a time
		while (millis() < (t_start + timeout)) {

			// Check for new data
			if (port.available() == 0)
			{
				// Wait a max of 1 sec for new data
				if (millis() - t_last_read < 1000 ||
					cnt_logBytesSent == 0) {
					continue;
				}

				// Handle read timeout
				else {

					// Add to error counter
					cnt_err_3++;

					// Try re-requesting data at least once
					if (cnt_err_3 > 1) {
						do_abort = true;
					}

					// Store error
					if (strlen(err_str) < 200) {
						sprintf(str, "Read Timedout: cnt=%d read_ind=%d dt_read=%d|",
							cnt_err_3, read_ind, millis() - t_last_read);
						strcat(err_str, str);
					}

					// Break
					break;
				}
			}

			// Get next bytes
			c_arr[0] = c_arr[1];
			c_arr[1] = c_arr[2];
			c_arr[2] = port.read();
			t_last_read = millis();
			read_ind++;

			// Check for leading "\r\n"
			if (!head_passed) {

				// Header found
				if (c_arr[2] != '\r' &&
					c_arr[2] != '\n') {

					head_passed = true;
				}

				// Continue till header found
				else {
					continue;
				}
			}

			// Check for error
			if (c_arr[2] == '!') {

				if ((c_arr[0] == '\r' && c_arr[1] == '\n') ||
					read_ind < 3) {

					// Add to error counter
					cnt_err_2++;

					// Retry "read" command only once
					if (cnt_err_2 > 1) {
						do_abort = true;
					}

					// Store error
					if (strlen(err_str) < 200) {
						sprintf(str, "\"read\" Failed: cnt=%d|", cnt_err_2);
						strcat(err_str, str);
					}

					// Break
					break;
				}
			}

			// Send byte
			r2c.port.write(c_arr[2]);
			cnt_logBytesSent++;

			// Print status
			if (cnt_logBytesSent == milestone_incriment[milestone_ind]) {

				// Print
				sprintf(str, "[LOGGER::StreamLogs] Log Write %d%% Complete: b_sent=%d/%d",
					milestone_ind * 10, cnt_logBytesSent, cnt_logBytesStored);
				DebugFlow(str, millis());
				DoAll("PrintDebug");

				// Itterate count
				milestone_ind++;
			}

			// Check if all bytes sent
			if (cnt_logBytesSent >= cnt_logBytesStored) {

				send_done = true;
				break;
			}

		}
	}

	// Check for unflagged timeout
	if (!do_abort &&
		millis() >= t_start + timeout) {

		do_abort = true;
		sprintf(str, "Run Timedout|");
		if (strlen(err_str) < 200) {
			strcat(err_str, str);
		}
	}

	// Resume logging
	delay(100);
	fc.doBlockLogWrite = false;
	SetToWriteMode(logFile);

	// Log warnings summary
	for (int i = 0; i < cnt_warn; i++) {
		char str_lin[10];
		sprintf(str_lin, "%d|", warn_line[i]);
		strcat(warn_lines, str_lin);
	}
	sprintf(str, "TOTAL WARNINGS: %d %s", cnt_warn, cnt_warn > 0 ? warn_lines : "");
	DebugFlow(str);

	// Log errors summary
	for (int i = 0; i < cnt_err; i++) {
		char str_lin[10];
		sprintf(str_lin, "%d|", err_line[i]);
		strcat(err_lines, str_lin);
	}
	sprintf(str, "TOTAL ERRORS:  %d %s", cnt_err, cnt_err > 0 ? warn_lines : "");
	DebugFlow(str);

	// Get total data left in buffers
	int xbee_buff_tx = SERIAL_BUFFER_SIZE - 1 - r2c.port.availableForWrite();
	int xbee_buff_rx = r2c.port.available();
	int ol_buff_tx = SERIAL_BUFFER_SIZE - 1 - port.availableForWrite();
	int ol_buff_rx = port.available();

	// Print log time info
	float dt_s = (float)(millis() - t_start) / 1000.0f;
	sprintf(str, "[LOGGER::StreamLogs] Run Info: dt_run=%0.2fs b_sent=%d b_stored=%d cnt_err_1=%d cnt_err_2=%d cnt_err_3=%d log_tx=%d log_rx=%d xbee_tx=%d xbee_rx=%d",
		dt_s, cnt_logBytesSent, cnt_logBytesStored, cnt_err_1, cnt_err_2, cnt_err_3, ol_buff_tx, ol_buff_rx, xbee_buff_tx, xbee_buff_rx);
	DebugFlow(str);

	// Print final status then send as log
	if (!do_abort) {
		sprintf(str, "[LOGGER::StreamLogs] SUCCEEDED: Sent %d Logs", cnt_logsStored + 1);
		DebugFlow(str);
	}
	else {
		sprintf(str, "!!ERROR!! [LOGGER::StreamLogs] ABORTED: Sending %d Logs: errors=%s", cnt_logsStored + 1, err_str);
		DebugError(str, true);
	}

	// Send remaining logs imediately
	while (WriteLog(true) && mode == '<');
	delay(100);

	// End reached send ">>>"
	if (!do_abort) {
		r2c.port.write(msg_streamSuccess, 3);
		delay(100);
	}
	// Aborted send ">>!"
	else {
		r2c.port.write(msg_streamFail, 3);
		delay(100);
	}

	// Print anything left in queue
	DoAll("PrintDebug");

#else

	// End reached send ">>>"
	r2c.port.write(msg_streamSuccess, 3);

#endif

	// Reset flag
	fc.doLogSend = false;
	fc.doSendVCC = true;
}

void LOGGER::TestLoad(int n_entry, char log_file[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	bool pass = false;

	// Prevent any new info from logging
	DoAll("WriteLog");
	fc.doBlockLogQueue = true;

	// Load existing log file
	if (log_file != '\0') {
		sprintf(str, "[LOGGER::TestLoad] RUNNING: Load Log: file_name=%s...", log_file);
		DebugFlow(str, millis());
		DoAll("PrintDebug");

		if (SetToCmdMode()) {

			// Get bytes
			if (SendCommand("ls\r") != '!') {
				cnt_logBytesStored = GetFileSize(log_file);
				pass = true;
			}

			// Start writing to file
			if (pass)
				if (!SetToWriteMode(log_file))
					pass = false;
		}
		if (pass) {
			sprintf(str, "[LOGGER::TestLoad] SUCCEEDED: Load Log: file_name=%s size=%dB",
				logFile, cnt_logBytesStored);
			DebugFlow(str, millis());
			DoAll("PrintDebug");
		}
		else {
			DebugError("!!ERROR!! [LOGGER::TestLoad] ABORTED: Load Log File", true);
			DoAll("PrintDebug");
		}
	}

	// Write n_entry entries to log
	else if (n_entry != 0) {
		sprintf(str, "[LOGGER::TestLoad] RUNNING: Write %d Logs...", n_entry);
		DebugFlow(str, millis());
		DoAll("PrintDebug");
		delayMicroseconds(100);
		DoAll("WriteLog");

		// Add n new logs
		randomSeed(analogRead(A0));
		int milestone_incriment = n_entry / 10;
		for (int i = 0; i < n_entry - 1; i++)
		{
			// Print status
			if (i%milestone_incriment == 0) {
				sprintf(str, "[LOGGER::TestLoad] Log Write %d%% Complete", i / milestone_incriment * 10);
				DebugFlow(str, millis());
				DoAll("PrintDebug");
				DoAll("WriteLog");
			}

			// Store a 120 charicter string
			char msg[200] = "AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFFGGGGGGGGGGHHHHHHHHHHIIIIIIIIIIJJJJJJJJJJKKKKKKKKKKLLLLLLLLLL";

			//// Make random sized strings
			//char num_str[15] = { 0 };
			//char msg[maxStrLng] = { 0 };
			//for (int i = 0; i < 18; i++)
			//{
			//	int num = random(999999999);
			//	sprintf(num_str, "%d", num);
			//	num_str[random(10) + 1] = '\0';
			//	strcat(msg, num_str);
			//}

			// Store log
			QueueLog(msg, millis());
			t_write = micros() - dt_write;
			DoAll("WriteLog");
		}
		sprintf(str, "[LOGGER::TestLoad] FINISHED: Write %d Logs", n_entry);
		DebugFlow(str, millis());
		DoAll("PrintDebug");
		DoAll("WriteLog");
	}
}

int LOGGER::GetFileSize(char log_file[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	int ind = -1;
	int fi_size = 0;
	static char num_str[20] = { 0 }; num_str[0] = '\0';

	// Find file index in array
	for (int i = strlen(rcvdArr) - strlen(log_file); i >= 0; i--)
	{
		int ii = 0;
		for (ii = strlen(log_file) - 1; ii >= 0; ii--) {

			if (log_file[ii] != rcvdArr[i + ii]) {
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
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	uint32_t t_m = 0;
	float t_s = 0;

	// Bail if logging should not be printed
	if (!db.print_logging) {
		return;
	}

	// Print like normal entry
	if (start_entry) {
		DebugFlow(msg, millis());
		// Print right away
		DoAll("PrintDebug");
	}

	// Print directly 
	else {
		SerialUSB.print(msg);
	}
}

#pragma endregion 

#pragma endregion 


#pragma region ========== FUNCTION DEFINITIONS =========

#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
void GetSerial(R4 *r4)
{

	/*
	PARSE DATA FROM CS
	FORMAT: [0]head, [1]id, [2:5]dat[0], [6:9]dat[1], [10:13]dat[1], [14:15]pack, [16]do_conf, [17]footer, [18]targ
	*/


	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char dat_str_1[200] = { 0 }; dat_str_1[0] = '\0';
	static char dat_str_2[200] = { 0 }; dat_str_2[0] = '\0';
	uint32_t t_str = millis();
	int dt_parse = 0;
	int buff_tx = 0;
	int buff_rx = 0;
	byte buff = 0;
	char head = ' ';
	char id = ' ';
	float dat[3] = { 0 };
	int r4_ind = 0;
	int r2_ind = 0;
	uint16_t pack = 0;
	char foot = ' ';
	bool do_conf;
	R2 *r2;

	// Set pointer to R2 struct
	if (r4->instID == "c2r") {
		r2 = &r2c;
	}
	else if (r4->instID == "a2r") {
		r2 = &r2a;
	}

	// Reset vars
	cnt_packBytesRead = 0;
	cnt_packBytesDiscarded = 0;
	r4->isNew = false;
	r4->idNow = ' ';

	// Bail if no new input
	if (r4->port.available() == 0) {

		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(r4, r4->head);
	if (buff == 0) {

		return;
	}

	// Store header
	head = buff;

	// Get id
	id = WaitBuffRead(r4);

	// Parse data
	for (int i = 0; i < 3; i++)
	{
		U.f = 0.0f;
		U.b[0] = WaitBuffRead(r4);
		U.b[1] = WaitBuffRead(r4);
		U.b[2] = WaitBuffRead(r4);
		U.b[3] = WaitBuffRead(r4);
		dat[i] = U.f;
	}

	// Get packet num
	U.f = 0.0f;
	U.b[0] = WaitBuffRead(r4);
	U.b[1] = WaitBuffRead(r4);
	pack = U.i16[0];

	// Get recieved confirmation
	U.f = 0.0f;
	U.b[0] = WaitBuffRead(r4);
	do_conf = U.b[0] != 0 ? true : false;

	// Get footer
	foot = WaitBuffRead(r4);

	// Strore parse time
	dt_parse = millis() - t_str;

	// Get total data in buffers
	buff_rx = r4->port.available();
	buff_tx = SERIAL_BUFFER_SIZE - 1 - r4->port.availableForWrite();

	// Store data strings
	sprintf(dat_str_1, "head=%c id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d foot=%c do_conf=%s b_read=%d b_dump=%d",
		head, id, dat[0], dat[1], dat[2], pack, foot, do_conf ? "1" : "0", cnt_packBytesRead, cnt_packBytesDiscarded);

	// Check for dropped packet
	if (foot != r4->foot) {

		// Store data strings
		sprintf(dat_str_2, "rx=%d tx=%d dt_prs=%d dt_snd=%d dt_rcv=%d",
			buff_rx, buff_tx, dt_parse, r2->t_sent > 0 ? millis() - r2->t_sent : 0, r4->dt_rcvd);

		// Itterate dropped count
		r4->cnt_dropped++;

		// Log/print dropped packet info
		sprintf(str, "**WARNING** [GetSerial] Dropped %s Packs: cnt=%d %s %s",
			r4->instID, r4->cnt_dropped, dat_str_1, dat_str_2);
		DebugError(str);
	}

	// Footer found so process packet
	else {

		// Update recive time
		r4->dt_rcvd = r4->t_rcvd > 0 ? millis() - r4->t_rcvd : 0;
		r4->t_rcvd = millis();

		// Store data strings
		sprintf(dat_str_2, "rx=%d tx=%d dt_prs=%d dt_snd=%d dt_rcv=%d",
			buff_rx, buff_tx, dt_parse, r2->t_sent > 0 ? millis() - r2->t_sent : 0, r4->dt_rcvd);

		// Get id ind
		r4_ind = CharInd<R4>(id, r4);
		r2_ind = CharInd<R2>(id, r2);

		// Send confirmation
		if (do_conf) {
			QueuePacket(r2, id, dat[0], dat[1], dat[2], pack, false);
		}

		// Set coms started flag
		if (r4->instID == "c2r" && !fc.isComsStarted) {
			fc.isComsStarted = true;
		}

		// Reset check
		r2->doRcvCheck[r2_ind] = false;
		r2->cnt_resend[r2_ind] = 0;

		// Check for missed packets
		int pack_diff = (pack - r4->packTot);

		// Get number of dropped/missed packets
		int cnt_dropped = pack_diff - 1;

		if (cnt_dropped > 0)
		{
			// Add to count and get last pack
			int cnt_dropped_tot = 0;
			uint16_t pack_tot_last = 0;

			// Store dropped data
			r4->cnt_dropped += cnt_dropped;
			cnt_dropped_tot = r4->cnt_dropped;
			pack_tot_last = r4->packTot;

			// Log/print skipped packet info
			sprintf(str, "**WARNING** [GetSerial] Missed %s Packs: cnt=%d|%d pack_last=%d %s %s",
				r4->instID, cnt_dropped, cnt_dropped_tot, pack_tot_last, dat_str_1, dat_str_2);
			DebugError(str);
		}

		// Update packet history
		r4->packLast[r4_ind] = r4->pack[r4_ind];
		r4->pack[r4_ind] = pack;
		r4->packTot = pack > r4->packTot ? pack : r4->packTot;
		uint16_t pack_last = r4->packLast[r4_ind];

		// Update id
		r4->idNow = id;

		// Combine data strings
		sprintf(str, "%s %s", dat_str_1, dat_str_2);

		// New pack
		if (pack != pack_last)
		{
			// Log/print received
			DebugRcvd(r4, str);

			// Store data and flag
			r4->isNew = true;
			r4->dat[0] = dat[0];
			r4->dat[1] = dat[1];
			r4->dat[2] = dat[2];
		}

		// Resent packet
		else {
			// Add to counters
			r4->cnt_repeat++;

			// Log/print received
			DebugRcvd(r4, str, true);
		}

	}

	// Check if data was discarded
	if (cnt_packBytesDiscarded > 0) {
		sprintf(str, "**WARNING** [GetSerial] Dumped Bytes: %s %s", dat_str_1, dat_str_2);
		DebugError(str);
	}

	// Check if parsing took unusually long
	if (dt_parse > 30) {
		sprintf(str, "**WARNING** [GetSerial] Parser Hanging: %s %s", dat_str_1, dat_str_2);
		DebugError(str);
	}

	return;
}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4 *r4, char mtch)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	static int cnt_overflow = 0;
	static int cnt_timeout = 0;
	bool is_overflowed = false;
	byte buff = 0;

	// Get total data in buffers now
	int buff_rx_start = r4->port.available();

	// Check for overflow
	is_overflowed = buff_rx_start >= SERIAL_BUFFER_SIZE - 1;

	// Wait for at least 1 byte
	while (r4->port.available() < 1 &&
		millis() < t_timeout);

	// Get any byte
	if (!is_overflowed &&
		mtch == '\0') {

		if (r4->port.available() > 0) {

			buff = r4->port.read();
			cnt_packBytesRead++;

			return buff;
		}
	}

	// Find specific byte
	while (
		buff != mtch  &&
		millis() < t_timeout &&
		!is_overflowed) {

		// Check new data
		if (r4->port.available() > 0) {

			buff = r4->port.read();
			cnt_packBytesRead++;

			// check match was found
			if (buff == mtch) {

				return buff;
			}

			// Otherwise add to discard count
			else {
				cnt_packBytesDiscarded++;
			}

			// Check for overflow
			is_overflowed =
				!is_overflowed ? r4->port.available() >= SERIAL_BUFFER_SIZE - 1 : is_overflowed;
		}

	}

	// Print issue
	char msg_str[200];
	char dat_str[200];

	// Check if buffer flooded
	if (is_overflowed) {

		// DUMP IT ALL
		while (r4->port.available() > 0) {
			if (r4->port.available() > 0) {
				r4->port.read();
				cnt_packBytesRead++;
			}
		}
	}

	// Get buffer 
	int buff_tx = SERIAL_BUFFER_SIZE - 1 - r4->port.availableForWrite();
	int buff_rx = r4->port.available();

	// Store current info
	sprintf(dat_str, " from=%s buff=\'%s\' b_read=%d b_dump=%d rx_str=%d rx_now=%d tx_now=%d dt_chk=%d",
		r4->instID, PrintSpecialChars(buff), cnt_packBytesRead, cnt_packBytesDiscarded, buff_rx_start, buff_rx, buff_tx, (millis() - t_timeout) + timeout);

	// Buffer flooded
	if (is_overflowed) {
		cnt_overflow++;
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Buffer Overflowed: cnt=%d", cnt_overflow);
	}
	// Timed out
	else if (millis() > t_timeout) {
		cnt_timeout++;
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Timedout: cnt=%d", cnt_timeout);
	}
	// Byte not found
	else if (mtch != '\0') {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Char \'%c\' Not Found:",
			mtch);
	}
	// Failed for unknown reason
	else {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Failed for Unknown Reason:");
	}

	// Compbine strings
	strcat(msg_str, dat_str);

	// Log/print error
	DebugError(msg_str);

	// Return 0

	return 0;

}

// STORE PACKET DATA TO BE SENT
void QueuePacket(R2 *r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char queue_state[sendQueueSize + 1] = { 0 }; queue_state[0] = '\0';
	int id_ind = 0;
	float dat[3] = { dat1 , dat2 , dat3 };
	R4 *r4;

	// Set pointer to R4 struct
	if (r2->instID == "r2c") {
		r4 = &c2r;
	}
	else if (r2->instID == "r2a") {
		r4 = &a2r;
	}

	// Update sendQueue ind
	r2->sendQueueIndStore++;

	// Check if ind should roll over 
	if (r2->sendQueueIndStore == sendQueueSize) {

		// Reset queueIndWrite
		r2->sendQueueIndStore = 0;
	}

	// Check if overfloweed
	if (r2->sendQueue[r2->sendQueueIndStore][0] != '\0')
	{

		// Get list of empty entries
		for (int i = 0; i < sendQueueSize; i++) {
			queue_state[i] = r2->sendQueue[i][0] == '\0' ? '0' : '1';
		}
		queue_state[sendQueueSize] = '\0';

		// Get buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
		int buff_rx = r2->port.available();

		// Store overflow error instead
		sprintf(str, "!!ERRROR!! [QueuePacket] %s SEND QUEUE OVERFLOWED: queue_s=%d queue_r=%d queue_state=|%s| dt_snd=%d dt_rcv=%d tx=%d rx=%d",
			r2->instID, r2->sendQueueIndStore, r2->sendQueueIndRead, queue_state, millis() - r2->t_sent, millis() - r4->t_rcvd, buff_tx, buff_rx);

		// Log/print error
		DebugError(str, true);

		// Set queue back 
		r2->sendQueueIndStore = r2->sendQueueIndStore - 1 >= 0 ? r2->sendQueueIndStore - 1 : sendQueueSize - 1;

		// Bail
		return;

	}

	// Itterate packet number
	if (pack == 0) {
		pack = ++r2->cnt_pack;
	}

	// Create byte packet
	int b_ind = 0;
	// Store header
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = r2->head;
	// Store mesage id
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = id;
	// Store mesage data 
	for (int i = 0; i < 3; i++)
	{
		U.f = dat[i];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[0];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[1];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[2];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[3];
	}
	// Store packet number
	U.f = 0.0f;
	U.i16[0] = pack;
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[0];
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[1];
	// Store get_confirm request
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = do_conf ? 1 : 0;
	// Store footer
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = r2->foot;

	return;
}

// SEND SERIAL PACKET DATA
bool SendPacket(R2 *r2)
{

	/*
	STORE DATA TO SEND
	FORMAT IN:  [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer, [9]targ
	FORMAT OUT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	const int msg_lng = sendQueueBytes;
	static char dat_str[200] = { 0 }; dat_str[0] = '\0';
	byte msg[msg_lng];
	uint32_t t_queue = millis();
	bool is_resend = false;
	char id = '\0';
	float dat[3] = { 0 };
	bool do_conf = 0;
	uint16_t pack = 0;
	int buff_tx;
	int buff_rx;
	int id_ind = 0;
	R4 *r4;
	R4 *r4o;
	R2 *r2o;

	// Reset bytes sent
	cnt_packBytesSent = 0;

	// Set pointer to R4 struct
	if (r2->instID == "r2c") {
		r4 = &c2r;
		r2o = &r2a;
		r4o = &a2r;
	}
	else if (r2->instID == "r2a") {
		r4 = &a2r;
		r2o = &r2c;
		r4o = &c2r;
	}

	// Bail if nothing in queue
	if (r2->sendQueueIndRead == r2->sendQueueIndStore &&
		r2->sendQueue[r2->sendQueueIndStore][0] == '\0') {

		return false;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Get buffer 
	buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
	buff_rx = r2->port.available();

	// Bail if buffer or time inadequate
	if (buff_tx > 0 ||
		buff_rx > 0 ||
		millis() < r2->t_sent + dt_sendSent ||
		millis() < r2o->t_sent + dt_sendSent ||
		millis() < r4->t_rcvd + dt_sendRcvd ||
		millis() < r4o->t_rcvd + dt_sendRcvd) {

		// Indicate still packs to send

		return true;
	}

	// Itterate send ind
	r2->sendQueueIndRead++;

	// Check if ind should roll over 
	if (r2->sendQueueIndRead == sendQueueSize) {
		r2->sendQueueIndRead = 0;
	}

	// Send
	r2->port.write(r2->sendQueue[r2->sendQueueIndRead], msg_lng);
	r2->dt_sent = r2->t_sent > 0 ? millis() - r2->t_sent : 0;
	r2->t_sent = millis();
	cnt_packBytesSent = msg_lng;

	// Get buffers
	buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
	buff_rx = r2->port.available();

	// pull out packet data
	int b_ind = 1;
	// id
	id = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
	// dat
	for (int i = 0; i < 3; i++)
	{
		U.f = 0;
		U.b[0] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		U.b[1] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		U.b[2] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		U.b[3] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		dat[i] = U.f;
	}
	// pack
	U.f = 0.0f;
	U.b[0] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
	U.b[1] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
	pack = U.i16[0];
	// do_conf 
	do_conf = r2->sendQueue[r2->sendQueueIndRead][b_ind++] == 1 ? true : false;

	// Set entry to null
	r2->sendQueue[r2->sendQueueIndRead][0] = '\0';

	// Get id ind
	id_ind = CharInd<R2>(id, r2);

	// Check if resending
	is_resend = pack == r2a.packLast[id_ind];

	// Set flags for recieve confirmation
	if (do_conf) {
		r2->doRcvCheck[id_ind] = true;
	}

	// Update struct info
	r2->datList[id_ind][0] = dat[0];
	r2->datList[id_ind][1] = dat[1];
	r2->datList[id_ind][2] = dat[2];
	r2->packLast[id_ind] = r2->pack[id_ind];
	r2->pack[id_ind] = pack;
	r2->t_sentList[id_ind] = r2->t_sent;

	// Check if resending
	is_resend = pack == r2->packLast[id_ind];

	// Make log/print string
	sprintf(dat_str, "id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d do_conf=%s b_sent=%d tx=%d rx=%d cts=%s dt_snd=%d dt_rcv=%d dt_q=%d",
		id, dat[0], dat[1], dat[2], pack, do_conf ? "1" : "0", cnt_packBytesSent, buff_tx, buff_rx, r2->stateCTS ? "1" : "0", r2->dt_sent, r4->t_rcvd > 0 ? millis() - r4->t_rcvd : 0, millis() - t_queue);

	// Log/print
	DebugSent(r2, dat_str, is_resend);

	// Return success
	return true;
}

// CHECK IF ROB TO ARD PACKET SHOULD BE RESENT
bool CheckResend(R2 *r2)
{

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char dat_str[200] = { 0 }; dat_str[0] = '\0';
	bool do_pack_resend = false;
	bool is_waiting_for_pack = false;
	int dt_sent = 0;

	// Bail if nothing to send
	for (int i = 0; i < r2->lng; i++)
	{
		is_waiting_for_pack = r2->doRcvCheck[i];
	}
	if (!is_waiting_for_pack) {
		return false;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Loop and check ard flags
	for (int i = 0; i < r2->lng; i++)
	{
		// Flag if waiting on anything
		if (r2->doRcvCheck[i]) {

			// Set flag
			is_waiting_for_pack = true;

			// Get dt sent
			dt_sent = millis() - r2->t_sentList[i];
		}

		// Bail if no action requred
		if (
			!r2->doRcvCheck[i] ||
			dt_sent < dt_resend
			) {
			continue;
		}

		// Get total data left in buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
		int buff_rx = r2->port.available();

		// Get dat string
		sprintf(dat_str, "id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d dt_sent=%dms tx=%d rx=%d",
			r2->id[i], r2->datList[i][0], r2->datList[i][1], r2->datList[i][2], r2->pack[i], dt_sent, buff_tx, buff_rx);

		if (r2->cnt_resend[i] < resendMax) {

			// Resend data
			QueuePacket(r2, r2->id[i], r2->datList[i][0], r2->datList[i][1], r2->datList[i][2], r2->pack[i], true);

			// Update count
			r2->cnt_resend[i]++;

			// Print resent packet
			sprintf(str, "**WARNING** [CheckResend] Resending %s Packet: cnt=%d %s",
				r2->instID, r2->cnt_resend[i], dat_str);
			DebugError(str);

			// Set flags
			do_pack_resend = true;
			r2->doRcvCheck[i] = false;
		}

		// Coms failed
		else {

			// Log/print error
			sprintf(str, "!!ERROR!! [CheckResend] ABORTED: Resending %s Packet: cnt=%d %s",
				r2->instID, r2->cnt_resend[i], dat_str);
			DebugError(str, true);

			// Reset flag
			r2->doRcvCheck[i] = false;
		}
	}

	// Return
	return is_waiting_for_pack;
}

#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------

// CONFIGURE AUTODRIVER BOARDS
void AD_Config(float max_speed, float max_acc, float max_dec)
{
#if DO_DEBUG_XXX
	DB_INF();
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
	AD_F.setOCThreshold(OC_4875mA);

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
	AD_R.setAccKVAL(40);				        // This controls the acceleration current
	AD_R.setDecKVAL(40);				        // This controls the deceleration current
	AD_R.setRunKVAL(30);					    // This controls the run current
	AD_R.setHoldKVAL(25);				        // This controls the holding current keep it low

												// NIMA 17 24V
	AD_F.setAccKVAL(40);				        // This controls the acceleration current
	AD_F.setDecKVAL(40);				        // This controls the deceleration current
	AD_F.setRunKVAL(30);					    // This controls the run current
	AD_F.setHoldKVAL(25);				        // This controls the holding current keep it low

}

// RESET AUTODRIVER BOARDS
void AD_Reset(float max_speed, float max_acc, float max_dec)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Reset each axis
	AD_R.resetDev();
	AD_F.resetDev();
	delayMicroseconds(100);

	// Configure each axis
	AD_Config(max_speed, max_acc, max_dec);
	delayMicroseconds(100);
	AD_CheckOC(true);
	delayMicroseconds(100);

	// Run motor at last speed
	RunMotor(runDirNow, runSpeedNow, "Override");
}

// CHECK AUTODRIVER STATUS
void AD_CheckOC(bool force_check)
{

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static uint32_t t_checkAD = millis() + dt_checkAD; // (ms)
	static bool dp_disable = false;
	static int cnt_errors = 0;
	static int ocd_last_r = 1;
	static int ocd_last_f = 1;
	int ocd_r;
	int ocd_f;

	// Bail if check disabled
	if (dp_disable) {
		return;
	}

	// Bail if not time for next check
	if (!force_check &&
		millis() < t_checkAD) {

		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Get/check 16 bit status flag
	adR_stat = AD_R.getStatus();
	ocd_r = GetAD_Status(adR_stat, "OCD");
	adF_stat = AD_F.getStatus();
	ocd_f = GetAD_Status(adF_stat, "OCD");

	// Check for overcurrent shut down
	if (ocd_r == 0 || ocd_f == 0) {

		// Track events
		cnt_errors++;

		// Log/print
		sprintf(str, "!!ERROR!! [AD_CheckOC] Overcurrent Detected Resetting Motor Reset: cnt=%d now_ocd_R|F=%d|%d last_ocd_R|F=%d|%d",
			cnt_errors, ocd_r, ocd_f, ocd_last_r, ocd_last_f);
		DebugError(str, true);

		// Reset motors
		AD_Reset();
	}

	// Store status
	ocd_last_r = ocd_r;
	ocd_last_f = ocd_f;

	// Set next check
	t_checkAD = millis() + dt_checkAD;

	// Disable checking after 5 errors
	if (cnt_errors >= 5) {

		// Log/print
		sprintf(str, "!!ERROR!! [AD_CheckOC] Disabled AD Check After %d Errors", cnt_errors);
		DebugError(str, true);

		// Set flag
		dp_disable = true;
	}

}

// HARD STOP
void HardStop(char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Log/print event
	sprintf(str, "[HardStop] Hard Stop [%s]", called_from);
	DebugFlow(str);

	// Normal hard stop
	AD_R.hardStop();
	AD_F.hardStop();

	// Reset speed
	runSpeedNow = 0;

	// Reset pid
	Pid.PID_Reset();

	// Set to high impedance so robot can be moved
	if (fc.isManualSes) {

		AD_R.hardHiZ();
		AD_F.hardHiZ();
	}
}

// IR TRIGGERED HARD STOP
void IRprox_Halt()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Run if Bull not active and on
	if (!(strcmp(Bull.modeBull, "Active") == 0 && strcmp(Bull.stateBull, "On") == 0)) {

		HardStop("IRprox_Halt");
	}
}

// RUN AUTODRIVER
bool RunMotor(char dir, double new_speed, char agent[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	double speed_rear = 0;
	double speed_front = 0;

	// Bail if caller does not have control
	if (strcmp(fc.motorControl, agent) != 0 &&
		strcmp("Override", agent) != 0) {
		return false;
	}

	// Log/print speed change
	DebugRunSpeed(agent, runSpeedNow, new_speed);

	// Scale vel
	speed_rear =
		rearMotCoeff[0] * (new_speed * new_speed * new_speed * new_speed) +
		rearMotCoeff[1] * (new_speed * new_speed * new_speed) +
		rearMotCoeff[2] * (new_speed * new_speed) +
		rearMotCoeff[3] * new_speed +
		rearMotCoeff[4];
	speed_front =
		frontMotCoeff[0] * (new_speed * new_speed * new_speed * new_speed) +
		frontMotCoeff[1] * (new_speed * new_speed * new_speed) +
		frontMotCoeff[2] * (new_speed * new_speed) +
		frontMotCoeff[3] * new_speed +
		frontMotCoeff[4];

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
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char speed_str[100] = { 0 }; speed_str[0] = '\0';
	static char vcc_str[100] = { 0 }; vcc_str[0] = '\0';
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
	RunMotor(dir, new_speed, "Override");

	// Print voltage and speed to LCD
	sprintf(vcc_str, "VCC=%0.2fV", vccAvg);
	sprintf(speed_str, "VEL=%s%dcm/s", runDirNow == 'f' ? "->" : "<-", (int)runSpeedNow);
	PrintLCD(false, vcc_str, speed_str);
}

// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(char set_to[], char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// VALUES:  ["None", "Halt", "Open", "MoveTo", "Bull", "Pid"]
	bool pass = false;

	// Set to/from "Halt"
	if (set_to == "Halt" || strcmp(fc.motorControl, "Halt") == 0) {

		// Only "Halt" and "Quit" can set/unset "Halt"
		if (called_from == "Halt" || called_from == "Quit") {

			sprintf(fc.motorControl, "%s", set_to);
		}
	}

	// If not in "Hault" mode
	else {

		// Can always set to "None"
		if (set_to == "None") {
			sprintf(fc.motorControl, "None");
		}

		// Cannot unset "None" unless certain conditions met
		if (strcmp(fc.motorControl, "None") == 0) {

			// Can still move robot if rat not in
			if (
				set_to == "MoveTo" &&
				!fc.isRatIn
				) {

				sprintf(fc.motorControl, "%s", set_to);
			}

			// Can set to "Open" under these conditions
			else if (set_to == "Open") {

				// InitializeTracking can always unset "None"
				if (called_from == "InitializeTracking") {

					sprintf(fc.motorControl, "%s", set_to);
				}

				// CheckBlockTimElapsed can unblock if tracking setup
				if (fc.isTrackingEnabled &&
					(called_from == "CheckBlockTimElapsed")
					) {

					sprintf(fc.motorControl, "%s", set_to);
				}

			}

		}

		// "MoveTo" can only be set to "Open" or "None"
		else if (strcmp(fc.motorControl, "MoveTo") == 0) {

			if (set_to == "Open") {

				sprintf(fc.motorControl, "%s", set_to);
			}
		}

		// "Bull" can only be set to "MoveTo" or "Open"
		else if (strcmp(fc.motorControl, "Bull") == 0) {

			if (set_to == "MoveTo" || set_to == "Open") {

				sprintf(fc.motorControl, "%s", set_to);
			}
		}

		// Otherwise can set to anything
		else if (strcmp(fc.motorControl, "Open") == 0 || strcmp(fc.motorControl, "Pid") == 0) {

			sprintf(fc.motorControl, "%s", set_to);
		}

	}

	// Return true if set to input
	if (strcmp(fc.motorControl, set_to) == 0) {
		pass = true;
	}

	// Store current controller
	DebugMotorControl(pass, set_to, called_from);

	return pass;
}

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(int dt, char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Set blocking and time
	fc.isBlockingTill = true;

	// Update time to hold till
	t_rewBlockMove = millis() + dt;

	// Remove all motor controll
	SetMotorControl("None", "BlockMotorTill");

	// Print blocking finished
	DebugMotorBocking("Blocking Motor for ", called_from, dt);
}

// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed()
{
	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	bool is_passed_feeder = false;
	bool is_mot_running = false;

	// Bail if not checking
	if (!fc.isBlockingTill) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Check that all 3 measures say rat has passed
	is_passed_feeder =
		fc.isTrackingEnabled &&
		kal.RatPos - (kal.RobPos + feedDist) > 0 &&
		Pos[0].posNow - (kal.RobPos + feedDist) > 0 &&
		Pos[2].posNow - (kal.RobPos + feedDist) > 0;

	// Check if motor already running again
	is_mot_running = runSpeedNow > 0;

	// Check for time elapsed or rat moved at least 3cm past feeder
	if (
		millis() > t_rewBlockMove ||
		is_passed_feeder ||
		is_mot_running)
	{
		// Print blocking finished
		DebugMotorBocking("Finished Blocking Motor at ", "CheckBlockTimElapsed");

		// Set flag to stop checking
		fc.isBlockingTill = false;

		// Retract arm early if rat ahead
		if (is_passed_feeder ||
			is_mot_running) {

			// Log/print
			if (is_passed_feeder) {
				DebugFlow("[CheckBlockTimElapsed] Unblocking Early: Rat Passed Feeder");
			}
			else if (is_mot_running) {
				DebugError("**WARNING** [CheckBlockTimElapsed] Unblocking Early: Motor Started Early");
			}

			// Retract feeder arm
			Reward.RetractFeedArm();
		}

		// Open up control
		SetMotorControl("Open", "CheckBlockTimElapsed");
	}

}

// DO SETUP TO BEGIN TRACKING
void InitializeTracking()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	int n_laps = 0;
	double cm_diff = 0;
	double cm_dist = 0;

	// Bail if finished
	if (fc.isTrackingEnabled) {
		return;
	}

	// Wait for new data
	if (!fc.isRatIn ||
		!Pos[0].isNew ||
		!Pos[2].isNew ||
		!Pos[1].isNew) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Log/Print
	DebugFlow("[InitializeTracking] RUNNING: Initialize Rat Tracking...");

	// Check that pos values make sense
	cm_diff = Pos[0].posNow - Pos[1].posNow;
	cm_dist = min((140 * PI) - abs(cm_diff), abs(cm_diff));

	// Log/print rat and robot starting pos
	sprintf(str, "[InitializeTracking] Starting Positions: rat_vt=%0.2f rat_pixy=%0.2f rob_vt=%0.2f rat_dist=%0.2f",
		Pos[0].posNow, Pos[2].posNow, Pos[1].posNow, cm_dist);
	DebugFlow(str);

	// Rat should be ahead of robot and by no more than 90 deg
	if (cm_diff < 0 ||
		cm_dist >((140 * PI) / 4))
	{
		// Log/print error
		DebugError("**WARNING** [InitializeTracking] Reseting Position Data Due to Bad Values");

		// Will have to run again with new samples
		Pos[0].PosReset();
		Pos[2].PosReset();
		Pos[1].PosReset();

		// Bail
		return;
	}

	// Set flag
	fc.isTrackingEnabled = true;

	// Reset ekf
	Pid.PID_ResetEKF("InitializeTracking");

	// Don't start pid for manual sessions
	if (!fc.isManualSes)
	{
		// Open up motor control
		SetMotorControl("Open", "InitializeTracking");

		// Run Pid
		Pid.PID_Run("InitializeTracking");
		DebugFlow("[InitializeTracking] PID STARTED");
	}

	// Initialize bulldoze
	if (fc.doBulldoze)
	{
		// Run from initial blocked mode
		Bull.BullOn("InitializeTracking");
		DebugFlow("[InitializeTracking] BULLDOZE INITIALIZED");
	}

	// Do status blink
	StatusBlink(true, 5, 100, true);

	// Log/Print
	DebugFlow("[InitializeTracking] FINISHED: Initialize Rat Tracking");
}

// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static char dat_str[200] = { 0 }; dat_str[0] = '\0';
	static int cnt_swap_vt = 0;
	static int cnt_swap_pixy = 0;
	bool do_swap_vt = false;
	bool do_swap_pixy = false;
	int dt_max_vt = 150;
	int dt_max_pixy = 100;
	int dt_vt = 0;
	int dt_pixy = 0;

	// Bail if rat not in yet
	if (!fc.isRatIn) {
		return;
	}

	// Bail if streaming not started
	if (!Pos[0].is_streamStarted ||
		!Pos[2].is_streamStarted) {
		return;
	}

	// Compute dt
	dt_vt = millis() - Pos[0].t_msNow;
	dt_pixy = millis() - Pos[2].t_msNow;

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

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Use Pixy for VT data
	if (do_swap_vt && !do_swap_pixy) {
		Pos[0].SwapPos(Pos[2].posAbs, Pos[2].t_msNow);
		cnt_swap_vt++;
	}

	// Use VT for Pixy data
	else if (do_swap_pixy && !do_swap_vt) {
		Pos[2].SwapPos(Pos[0].posAbs, Pos[0].t_msNow);
		cnt_swap_pixy++;
	}

	// Log/print first and every 10 errors
	if (cnt_swap_vt + cnt_swap_pixy == 1 ||
		cnt_swap_vt + cnt_swap_pixy % 10 == 0) {

		// Format data string
		sprintf(dat_str, "cnt=%d|%d dt=%d|%d pos=%0.2f|%0.2f",
			cnt_swap_vt, cnt_swap_pixy, dt_vt, dt_pixy, Pos[0].posAbs, Pos[2].posAbs);

		// Swapped one for the other
		if (!(do_swap_vt && do_swap_pixy)) {
			sprintf(str, "**WARNING** [CheckSampDT] Swapped %s: %s",
				do_swap_vt ? "VT with Pixy" : "Pixy with VT", dat_str);
		}

		// Both sources dt too long
		else {
			sprintf(str, "**WARNING** [CheckSampDT] All Rat Tracking Hanging: %s", dat_str);
		}

		// Log/print
		DebugError(str);
	}

}

// PROCESS PIXY STREAM
double CheckPixy(bool do_test_check)
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	double px_rel = 0;
	double px_abs = 0;
	uint32_t t_px_ts = 0;
	double pixy_pos_y = 0;

	// Bail if robot not streaming yet
	if (!do_test_check &&
		!Pos[1].is_streamStarted) {
		return px_rel;
	}

	// Bail if rat not in or doing sym test
	if (!do_test_check &&
		!db.do_posDebug &&
		(!fc.isRatIn || db.do_simRatTest)) {
		return px_rel;
	}

	// Get new blocks
	uint16_t blocks = Pixy.getBlocks();

	// Bail in no new data
	if (!blocks) {
		return px_rel;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

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

	// Return rel val is testing
	if (do_test_check) {
		return px_rel;
	}

	// Scale to abs space with rob vt data
	px_abs = px_rel + Pos[1].posAbs;
	if (px_abs > (140 * PI)) {
		px_abs = px_abs - (140 * PI);
	}

	// Update pixy pos and vel
	Pos[2].UpdatePos(px_abs, t_px_ts);

	// Log/print first sample
	if (!Pos[2].is_streamStarted) {
		sprintf(str, "[CheckPixy] FIRST RAT PIXY RECORD: pos_abs=%0.2f pos_rel=%0.2f n_laps=%d",
			Pos[2].posAbs, Pos[2].posNow, Pos[2].nLaps);
		DebugFlow(str);
		Pos[2].is_streamStarted = true;
	}

	// Return relative pos
	return px_rel;
}

// UPDATE EKF
void UpdateEKF()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static int cnt_error = 0;
	static bool is_nans_last = false;
	double rat_pos = 0;
	double rob_pos = 0;
	double rat_vel = 0;
	double rob_vel = 0;

	// Bail if all data not new
	if (!(Pos[0].isNew && Pos[2].isNew && Pos[1].isNew)) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Check EKF progress
	Pid.PID_CheckEKF(millis());

	// Set pid update time
	Pid.PID_SetUpdateTime(millis());

	// Set flag for reward
	if (fc.isTrackingEnabled) {
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
	kal.t_update = millis();
	kal.cnt++;

	// Check for nan values
	if (isnan(rat_pos) || isnan(rob_pos) || isnan(rat_vel) || isnan(rob_vel)) {

		// Do not print consecutively
		if (!is_nans_last)
		{
			sprintf(str, "!!ERROR!! [UpdateEKF] \"nan\" EKF Output: Pos[0]=%0.2f|%0.2f Pos[2]=%0.2f|%0.2f Pos[1]=%0.2f|%0.2f",
				Pos[0].posNow, Pos[0].velNow, Pos[2].posNow, Pos[2].velNow, Pos[1].posNow, Pos[0].velNow);
			DebugError(str, true);

			// Set flag
			is_nans_last = true;
		}
	}
	else {
		is_nans_last = false;
	}

	// Check if too much time elapsed between updates
	if (millis() > kal.t_update + 250 &&
		kal.t_update != 0) {

		// Add to count
		cnt_error++;

		// Log/print first and every 10 errors
		if (cnt_error == 1 ||
			cnt_error % 10 == 0) {

			// Log/print warning
			sprintf(str, "**WARNING** [UpdateEKF] EKF Hanging: cnt_err=%d dt_update=%d", cnt_error, millis() - kal.t_update);
			DebugError(str);
		}
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
	static char str[200] = { 0 }; str[0] = '\0';
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
	bool do_check = false;

	// Bail if nothing to do
	for (int i = 0; i < 3; i++) {

		do_check = do_check ||
			digitalRead(pin.Btn[i]) == LOW ||
			is_pressed[i] ||
			is_running[i];
	}
	if (!do_check) {
		return false;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Loop through and check each button
	for (int i = 0; i < 3; i++) {

		// Detect press
		if (
			digitalRead(pin.Btn[i]) == LOW &&
			!is_pressed[i])
		{
			// Exit if < debounce time has not passed
			if (t_debounce[i] > millis()) {
				return false;
			}

			// Log/print
			sprintf(str, "[GetButtonInput] Pressed button %d", i);
			DebugFlow(str);

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
				digitalRead(pin.Btn[i]) == HIGH &&
				millis() < t_long_hold[i];

			// Check for long hold
			bool is_long_hold = millis() > t_long_hold[i];

			// Set flag for either condition
			if (is_short_hold || is_long_hold) {

				// Log/print
				sprintf(str, "[GetButtonInput] Triggered button %d", i);
				DebugFlow(str);

				// Run short hold function
				if (is_short_hold) {
					do_flag_fun[i][0] = true;
				}

				// Run long hold function
				if (is_long_hold) {
					do_flag_fun[i][1] = true;
				}

				// Make tracker LED brighter
				analogWrite(pin.TrackLED, 255);

				// Set running flag
				is_running[i] = true;

				// Flag input rcvd
				is_new_input = true;
			}
		}

		// Check if needs to be reset
		else if (digitalRead(pin.Btn[i]) == HIGH &&
			is_pressed[i]) {

			if (is_running[i] ||
				millis() > t_long_hold[i]) {

				// Log/print
				sprintf(str, "[GetButtonInput] Reset button %d", i);
				DebugFlow(str);

				// Reset flags etc
				t_debounce[i] = millis() + dt_debounce[i];
				analogWrite(pin.TrackLED, trackLEDdutyMax);
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
	if (do_flag_fun[0][0]) {

		// Reward or retract feeder arm
		if (!Reward.isArmExtended) {
			fc.doBtnRew = true;
			DebugFlow("[GetButtonInput] Button 0 \"fc.doBtnRew\" Triggered");
		}
		else {
			Reward.RetractFeedArm();
			DebugFlow("[GetButtonInput] Button 0 \"Reward.RetractFeedArm()\" Triggered");
		}
	}

	else if (do_flag_fun[0][1]) {
		fc.doMoveRobFwd = true;
		DebugFlow("[GetButtonInput] Button 0 \"fc.doMoveRobFwd\" Triggered");
	}
	// Set button 2 function flag
	else if (do_flag_fun[1][0]) {
		fc.doRewSolStateChange = true;
		DebugFlow("[GetButtonInput] Button 1 \"fc.doRewSolStateChange\" Triggered");
	}
	else if (do_flag_fun[1][1]) {
		fc.doMoveRobRev = true;
		DebugFlow("[GetButtonInput] Button 1 \"fc.doMoveRobRev\" Triggered");
	}
	// Set button 3 function flag
	else if (do_flag_fun[2][0]) {
		fc.doEtOHSolStateChange = true;
		DebugFlow("[GetButtonInput] Button 2 \"fc.doEtOHSolStateChange\" Triggered");
	}
	else if (do_flag_fun[2][1]) {
		fc.doChangeLCDstate = true;
		DebugFlow("[GetButtonInput] Button 2 \"fc.doChangeLCDstate\" Triggered");
	}

	// Reset function flag
	for (int i = 0; i < 3; i++) {

		// Reset flag
		do_flag_fun[i][0] = false;
		do_flag_fun[i][1] = false;
	}

	// Turn on LCD if any fucntion flagged durring manual mode
	if (fc.isManualSes && !fc.doChangeLCDstate) {
		// Turn on LCD LED
		ChangeLCDlight(10);
	}

	// Return flag
	return true;
}

// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_Rew);
	static char etoh_str[100] = { 0 }; etoh_str[0] = '\0';
	static char rew_str[100] = { 0 }; rew_str[0] = '\0';

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
	digitalWrite(pin.Rel_Rew, is_sol_open);

	// Print etoh and rew sol state to LCD
	sprintf(rew_str, "Food   %s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN  " : "CLOSED");
	sprintf(etoh_str, "EtOH   %s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN  " : "CLOSED");
	PrintLCD(true, rew_str, etoh_str, 's');

}

// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_EtOH);
	static char etoh_str[100] = { 0 }; etoh_str[0] = '\0';
	static char rew_str[100] = { 0 }; rew_str[0] = '\0';

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
	sprintf(rew_str, "Food   %s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN  " : "CLOSED");
	sprintf(etoh_str, "EtOH   %s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN  " : "CLOSED");
	PrintLCD(true, rew_str, etoh_str, 's');

}

// CHECK FOR ETOH UPDATE
void CheckEtOH()
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	byte is_sol_open = digitalRead(pin.Rel_EtOH);
	int dt_open = 0;
	int dt_close = 0;
	bool do_open = false;
	bool do_close = false;

	// Bail if etoh should not be run
	if (!fc.doEtOHRun) {
		return;
	}

	// Check if should be opened
	do_open = !is_sol_open &&
		millis() > (t_solOpen + dt_delEtOH[fc.isSesStarted ? 0 : 1]);

	// Open only if motor not running
	if (do_open) {

		if (runSpeedNow > 0) {
			// Reset flag
			do_open = false;
		}
	}

	// Close if open and dt close has ellapsed
	do_close = is_sol_open &&
		millis() > (t_solOpen + dt_durEtOH[fc.isSesStarted ? 0 : 1]);

	// Bail if nothing to do
	if (!do_open && !do_close) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Check if sol should be opened
	if (do_open &&
		GetAD_Status(adR_stat, "MOT_STATUS") == 0 &&
		GetAD_Status(adF_stat, "MOT_STATUS") == 0) {

		// Open solenoid
		digitalWrite(pin.Rel_EtOH, HIGH);

		// Store current time and pos
		t_solOpen = millis();

		// Compute dt
		dt_close = t_solClose > 0 ? t_solOpen - t_solClose : 0;

		// Print to debug
		sprintf(str, "[CheckEtOH] Open EtOH: dt_close=%d", dt_close);
		DebugFlow(str);
	}

	// Check if sol should be closed
	else if (do_close) {

		// Close solenoid
		digitalWrite(pin.Rel_EtOH, LOW);

		// Store current time 
		t_solClose = millis();

		// Compute dt
		dt_open = t_solOpen > 0 ? t_solClose - t_solOpen : 0;

		// Print to debug
		sprintf(str, "[CheckEtOH] Close EtOH: dt_open=%d", dt_open);
		DebugFlow(str);
	}
}

// CHECK BATTERY VALUES
float CheckBattery(bool force_check)
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static char vcc_str[100]; vcc_str[0] = '\0';
	static char ic_str[100]; ic_str[0] = '\0';
	static uint32_t t_vcc_update = 0;
	static uint32_t t_vcc_send = 0;
	static uint32_t t_vcc_print = 0;
	static uint32_t t_ic_update = 0;
	static uint32_t t_update_str = 0;
	static float vcc_shutdown_arr[10] = { 0 };
	static float vcc_avg = 0;
	static float vcc_last = 0;
	static int cnt_samples = 0;
	uint32_t vcc_bit_in = 0;
	uint32_t ic_bit_in = 0;
	float vcc_sum = 0;
	bool is_mot_off = false;
	byte do_shutdown = false;

	// Bail if nothing to do
	if (!force_check) {

		// Not time to check
		if (millis() < t_update_str) {
			// Bail
			return vccAvg;
		}

		// Done checking
		else if (cnt_samples >= vccMaxSamp) {

			// Compute next check time
			t_update_str = millis() + dt_vccUpdate;

			// Reset samples
			cnt_samples = 0;

			// Turn off switch and bail
			digitalWrite(pin.Rel_Vcc, LOW);
			return vccAvg;
		}
	}

	// Bail if run speed > 0
	if (runSpeedNow > 0) {

		// Turn off switch and bail
		digitalWrite(pin.Rel_Vcc, LOW);
		return vccAvg;

	}

	// Turn on relay and skip this run to alow switch to open
	if (digitalRead(pin.Rel_Vcc) == LOW) {

		// Turn on switch 
		digitalWrite(pin.Rel_Vcc, HIGH);

		// Allow time for switch to activate
		t_update_str = millis() + 10;

		// Bail
		return vccAvg;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Calculate current
	if (millis() > t_ic_update + dt_icUpdate) {
		ic_bit_in = analogRead(pin.BatIC);
		icNow = 36.7*(((float)ic_bit_in * (3.3 / 1024)) / 3.3) - 18.3;
		t_ic_update = millis();
	}

	// Calculate voltage
	vcc_bit_in = analogRead(pin.BatVcc);
	vccNow = (float)vcc_bit_in * bit2volt;
	vcc_sum = 0;

	// Shift array and compute average
	for (int i = 0; i < vccMaxSamp - 1; i++) {
		vccArr[i] = vccArr[i + 1];
		vcc_sum += vccArr[i];
	}
	vccArr[vccMaxSamp - 1] = vccNow;
	vcc_avg = vcc_sum / vccMaxSamp;

	// Return 0 till array full
	cnt_samples = cnt_samples < vccMaxSamp ? cnt_samples + 1 : vccMaxSamp;
	if (cnt_samples < vccMaxSamp) {
		return 0;
	}

	// Store new voltage level
	vcc_last = vccAvg;
	vccAvg = vcc_avg;

	// Store time
	t_vcc_update = millis();

	// Keep a list of averages to check for shutdown
	for (int i = 0; i < 10 - 1; i++) {
		vcc_shutdown_arr[i] = vcc_shutdown_arr[i + 1];
	}
	vcc_shutdown_arr[9] = vccAvg;

	// Send if min dt ellapsed
	if (
		fc.doSendVCC &&
		millis() > t_vcc_send + dt_vccSend) {

		// Send
		QueuePacket(&r2c, 'J', vccAvg, 0, 0, 0, false);

		// Store time
		t_vcc_send = millis();
	}

	// Log/print voltage and current
	if (millis() > t_vcc_print + dt_vccPrint) {

		// Log/print voltage and current
		sprintf(str, "[GetBattVolt] VCC Change: vcc=%0.2fV ic=%0.2fA dt_chk=%d",
			vccAvg, icNow, millis() - t_vcc_update);
		DebugFlow(str);

		// Print to lcd
		sprintf(vcc_str, "VCC=%0.2fV", vccAvg);
		sprintf(ic_str, "IC=%0.2fA", icNow);
		PrintLCD(false, vcc_str, ic_str);

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
			// Run error hold then shutdown after 5 min
			sprintf(str, "BATT LOW %0.2fV", vccAvg);
			RunErrorHold(str, 60000);
		}
	}

	// Return battery voltage
	return vccAvg;
}

// TURN LCD LIGHT ON/OFF
void ChangeLCDlight(uint32_t duty)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Check if new duty given
	if (duty == 256) {
		fc.isLitLCD = !fc.isLitLCD;
		duty = fc.isLitLCD ? 50 : 0;
	}
	else {
		fc.isLitLCD = duty > 0;
	}

	// Log/print
	sprintf(str, "[ChangeLCDlight] Set LCD Light: is_lit=%s duty=%d",
		fc.isLitLCD ? "1" : "0", duty);
	DebugFlow(str);

	// Set LCD duty
	analogWrite(pin.Disp_LED, duty);
}

// QUIT AND RESTART ARDUINO
void QuitSession()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Stop all movement
	HardStop("QuitSession");
	Pid.PID_Stop("QuitSession");
	Bull.BullOff("QuitSession");

	// Log/print anything left in queue
	DoAll("WriteLog");
	DoAll("PrintDebug");

	// Wait 1 sec for any backround stuff to finish
	delay(1000);

	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

#pragma endregion


#pragma region --------DEBUGGING---------

// DO HARDWARE TEST
void HardwareTest()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

#if DO_HARDWARE_TEST

	// Local vars
	static char str[200] = { 0 };
	static char str_vcc[200] = { 0 };
	static char str_lcd[200] = { 0 };
	static char str_print[200] = { 0 };
	static char str_log[200] = { 0 };
	static char str_pixy[200] = { 0 };
	static char str_ping[2][200] = { { 0 } };
	const int dt_test = 10000;
	uint32_t t_test_str = 0;

	// Stress test
	double speed_range[2] = { 5,65 };
	const byte n_stress_samp = 10;
	byte speed_step = (speed_range[1] - speed_range[0]) / (n_stress_samp / 2);
	byte cnt_stress = 0;
	uint32_t t_stress_run = 0;
	uint32_t dt_stress_run = dt_test / n_stress_samp;
	uint32_t dt_close_sol = 100;
	double run_speed[n_stress_samp] = { 0 };
	bool is_stressin = false;
	float vcc_baseline = 0;
	float vcc_arr[n_stress_samp] = { 0 };
	float vcc_sum = 0;
	float vcc_avg = 0;
	byte s_now = speed_range[0];
	for (int i = 0; i < n_stress_samp; i++) {
		run_speed[i] = i % 2 == 0 ? s_now += speed_step : s_now;
	}
	uint32_t t_lcd_str = 0;
	uint32_t t_print_str = 0;
	uint32_t t_log_str = 0;
	uint32_t lcd_arr[n_stress_samp] = { 0 };
	uint32_t lcd_sum = 0;
	double lcd_avg = 0;
	uint32_t print_arr[n_stress_samp] = { 0 };
	uint32_t print_sum = 0;
	double print_avg = 0;
	uint32_t log_arr[n_stress_samp] = { 0 };
	uint32_t log_sum = 0;
	double log_avg = 0;
	bool is_stress_test_done = false;

	// Pixy test
	const byte n_pixy_samp = 10;
	uint32_t t_pixy_check = 0;
	uint32_t dt_pixy_check = dt_test / n_pixy_samp / 4;
	byte cnt_pixy = 0;
	bool is_pixy_led_on = false;
	double pixy_pos_arr[n_pixy_samp] = { 0 };
	double pixy_pos_sum = 0;
	double pixy_pos_avg = 0;
	bool is_pixy_test_done = false;

	// Ping test
	uint32_t dt_ping_mat[2][n_pings] = { { 0 } };
	uint32_t dt_ping_sum = 0;
	uint16_t cnt_ping[2] = { 0,0 };
	byte r2i = 0;
	bool do_send_ping[2] = { true, true };
	R2 *r2;
	R4 *r4;
	bool is_ping_test_done = false;

	// Log/print
	DebugFlow("[HardwareTest] RUNNING HARDWARE TEST...");

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

	// Print remaining queue
	DoAll("PrintDebug");
	// Store remaining logs
	DoAll("WriteLog");

	// Store start time
	t_test_str = millis();

	// Run Test
	while (
		(!is_stress_test_done ||
			!is_pixy_test_done ||
			!is_ping_test_done
			) &&
		millis() < t_test_str + dt_test * 2) {

		// Do stress test
		if (!is_stress_test_done) {

			// Block printing and logging
			fc.doBlockLogQueue = true;
			fc.doBlockPrintQueue = true;

			// Extend retract feed arm
			if (!Reward.doExtendArm &&
				!Reward.doRetractArm) {
				Reward.isArmExtended ? Reward.RetractFeedArm() : Reward.ExtendFeedArm();
			}
			Reward.CheckFeedArm();

			// Do next stage
			if (cnt_stress < n_stress_samp &&
				millis() > t_stress_run + dt_stress_run) {

				// Flip state
				is_stressin = !is_stressin;

				// Run motors
				HardStop("HardwareTest");
				RunMotor(runDirNow == 'f' ? 'r' : 'f', run_speed[cnt_stress], "Override");

				// Open Solenoids
				digitalWrite(pin.Rel_Rew, HIGH);
				digitalWrite(pin.Rel_EtOH, HIGH);

				// Change LEDS
				analogWrite(pin.Disp_LED, is_stressin ? 255 : 0);
				analogWrite(pin.TrackLED, 255);
				if (!is_pixy_led_on) {
					analogWrite(pin.RewLED_C, is_stressin ? 255 : 0);
					analogWrite(pin.RewLED_R, is_stressin ? 255 : 0);
				}

				// Print to LCD
				t_lcd_str = micros();
				sprintf(str, "C=%d S=%0.0f V=%0.0f", cnt_stress + 1, run_speed[cnt_stress], vccNow);
				PrintLCD(false, str);
				lcd_arr[cnt_stress] = micros() - t_lcd_str;

				// Print to console
				fc.doBlockPrintQueue = false;
				t_print_str = micros();
				sprintf(str, "[HardwareTest] Stress Test %d: Speed=%0.2f VCC=%0.2f", cnt_stress + 1, run_speed[cnt_stress], vccNow);
				DebugFlow(str);
				fc.doBlockPrintQueue = true;
				DoAll("PrintDebug");
				print_arr[cnt_stress] = micros() - t_print_str;

				// Store log
				fc.doBlockLogQueue = false;
				t_log_str = micros();
				sprintf(str, "[HardwareTest] Stress Test %d: Speed=%0.2f VCC=%0.2f", cnt_stress + 1, run_speed[cnt_stress], vccNow);
				DebugFlow(str);
				fc.doBlockLogQueue = true;
				DoAll("WriteLog");
				log_arr[cnt_stress] = micros() - t_log_str;

				// Store time
				t_stress_run = millis();

				// Store battery voltage
				CheckBattery(true);
				vcc_arr[cnt_stress] = vccNow;

				// Itterate count
				cnt_stress++;
			}

			else if (cnt_stress == n_stress_samp &&
				millis() > t_stress_run + dt_stress_run) {

				// Stop motors
				HardStop("HardwareTest");

				// Turn all off
				analogWrite(pin.Disp_LED, 0);
				analogWrite(pin.RewLED_C, 0);
				analogWrite(pin.RewLED_R, 0);
				analogWrite(pin.TrackLED, 0);
				digitalWrite(pin.Rel_Rew, LOW);
				digitalWrite(pin.Rel_EtOH, LOW);

				// Get averages
				for (int i = 0; i < n_stress_samp; i++) {

					// LCD
					lcd_sum += lcd_arr[i];
					sprintf(str, "%0.2f|", (double)lcd_arr[i] / 1000);
					strcat(str_lcd, str);

					// Print
					print_sum += print_arr[i];
					sprintf(str, "%0.2f|", (double)print_arr[i] / 1000);
					strcat(str_print, str);

					// Log
					log_sum += log_arr[i];
					sprintf(str, "%0.2f|", (double)log_arr[i] / 1000);
					strcat(str_log, str);

					// VCC
					vcc_sum += vcc_arr[i];
					sprintf(str, "%0.2f|", vcc_arr[i]);
					strcat(str_vcc, str);
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
				if (digitalRead(pin.Rel_Rew) == HIGH) {
					digitalWrite(pin.Rel_Rew, LOW);
				}

				// Close reward sol
				if (digitalRead(pin.Rel_EtOH) == HIGH) {
					digitalWrite(pin.Rel_EtOH, LOW);
				}
			}

			// Unblock printing and logging
			fc.doBlockLogQueue = false;
			fc.doBlockPrintQueue = false;

			// Check motor status
			AD_CheckOC();
		}

		// Test Pixy
		if (!is_pixy_test_done) {

			// Get new sample
			if (cnt_pixy < n_pixy_samp &&
				millis() > t_pixy_check + dt_pixy_check) {

				// Flip led state
				is_pixy_led_on = !is_pixy_led_on;

				// Turn on/off LED
				analogWrite(pin.RewLED_R, is_pixy_led_on ? 15 : 0);
				analogWrite(pin.RewLED_C, 0);
				t_pixy_check = millis();

				// Check pixy and store pos
				if (!is_pixy_led_on) {

					// Store value
					pixy_pos_arr[cnt_pixy] = CheckPixy(true);

					// Itterate count
					cnt_pixy++;
				}
			}

			// Get final average
			else if (cnt_pixy == n_pixy_samp) {

				// Loop samples
				for (int i = 0; i < n_pixy_samp; i++) {

					pixy_pos_sum += pixy_pos_arr[i];
					sprintf(str, "%0.2f|", pixy_pos_arr[i]);
					strcat(str_pixy, str);
				}

				// Compute average
				pixy_pos_avg = pixy_pos_sum / (n_pixy_samp);

				// Set flag
				is_pixy_test_done = true;
			}

		}

		// Send pings
		if (!is_ping_test_done) {

			// Do send receive check
			if (cnt_ping[0] <= n_pings ||
				cnt_ping[1] <= n_pings) {

				// Ping CS and CheetahDue
				r2 = r2i == 0 ? &r2c : &r2a;
				r4 = r2i == 0 ? &c2r : &a2r;

				// Check for reply
				GetSerial(r4);

				// Store round trip time
				if (r4->isNew &&
					r4->dat[0] == cnt_ping[r2i]) {

					// Store dt send
					dt_ping_mat[r2i][cnt_ping[r2i]] = r4->t_rcvd - r2->t_sent;

					// Itterate count
					cnt_ping[r2i]++;

					// Set flag to send next
					do_send_ping[r2i] = cnt_ping[r2i] <= n_pings;

					// Flip destination
					r2i = r2i == 0 ? 1 : 0;

					// Next loop
					continue;
				}

				// Send next r2 ping
				if (do_send_ping[r2i]) {

					// Send pack
					float dat1 = cnt_ping[r2i];
					float dat2 = cnt_ping[0] > 0 ? dt_ping_mat[0][cnt_ping[0] - 1] : 0;
					float dat3 = cnt_ping[1] > 0 ? dt_ping_mat[1][cnt_ping[1] - 1] : 0;
					QueuePacket(r2, 't', dat1, dat2, dat3, 0, true);

					// Reset flag
					do_send_ping[r2i] = false;
				}

			}

			// Get final ping times
			else {

				// Compute r2c ping average
				for (int i = 0; i < 2; i++) {

					// Reset sum
					dt_ping_sum = 0;

					// Loop pings
					for (int j = 0; j < n_pings; j++) {

						dt_ping_sum += dt_ping_mat[i][j];
						sprintf(str, "%d|", dt_ping_mat[i][j]);
						strcat(str_ping[i], str);
					}

					// Compute average
					dt_pingRoundTrip[i] = (float)dt_ping_sum / (n_pings);
				}

				// Flag done
				is_ping_test_done = true;
			}

		}

		// Send any packets
		SendPacket(&r2c);
		SendPacket(&r2a);

		// Log/print all
		DoAll("PrintDebug");
		DoAll("WriteLog");
	}

	// Log/print
	DebugFlow("[HardwareTest] FINISHED HARDWARE TEST");

	// Log/print vcc
	sprintf(str, "[HardwareTest] VCC: baseline=%0.2f avg=%0.2f all=|%s",
		vcc_baseline, vcc_avg, str_vcc);
	DebugFlow(str);

	// Log/print LCD and Print and log times
	sprintf(str, "[HardwareTest] LCD PRINT TIME: avg=%0.2f all=|%s", lcd_avg, str_lcd);
	DebugFlow(str);
	sprintf(str, "[HardwareTest] CONSOLE PRINT TIME: avg=%0.2f all=|%s", print_avg, str_print);
	DebugFlow(str);
	sprintf(str, "[HardwareTest] LOG WRITE TIME: avg=%0.2f all=|%s", log_avg, str_log);
	DebugFlow(str);

	// Log/print pixy pos
	sprintf(str, "[HardwareTest] PIXY POS: avg=%0.2f all=|%s", pixy_pos_avg, str_pixy);
	DebugFlow(str);

	// Log/print ping time
	sprintf(str, "[HardwareTest] PING TIMES r2c: avg=%0.2f all=|%s", dt_pingRoundTrip[0], str_ping[0]);
	DebugFlow(str);
	sprintf(str, "[HardwareTest] PING TIMES r2a: avg=%0.2f all=|%s", dt_pingRoundTrip[1], str_ping[1]);
	DebugFlow(str);

#endif
}

// CHECK LOOP TIME AND MEMORY
void CheckLoop()
{

	// Local static vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char msg[50] = { 0 }; msg[0] = '\0';
	static uint32_t t_loop_last = millis();
	static int dt_loop = 0;
	static int dt_loop_last = 0;
	static int c_rx_last = 0;
	static int c_tx_last = 0;
	static int a_rx_last = 0;
	static int a_tx_last = 0;
	static uint32_t t_led = 0;
	static bool is_led_high = false;
	bool is_dt_change = false;
	bool is_buff_flooding = false;
	int c_rx = 0;
	int c_tx = 0;
	int a_rx = 0;
	int a_tx = 0;

	// Keep short count of loops
	cnt_loop_short = cnt_loop_short < 999 ? cnt_loop_short + 1 : 1;

	// Bail till ses started
	if (!fc.isSesStarted) {
		return;
	}

	// Flicker led
	if (!StatusBlink()) {
		if (millis() > t_led) {
			analogWrite(pin.TrackLED, is_led_high ? trackLEDdutyMin : trackLEDdutyMax);
			is_led_high = !is_led_high;
			t_led = millis() + 100;
		}
	}

	// Bail if this is a test run
	if (db.is_runTest) {
		return;
	}

	// Get total data left in buffers
	c_rx = c2r.port.available();
	c_tx = SERIAL_BUFFER_SIZE - 1 - c2r.port.availableForWrite();
	a_rx = a2r.port.available();
	a_tx = SERIAL_BUFFER_SIZE - 1 - a2r.port.availableForWrite();

	// Track total loops
	cnt_loop_tot++;

	// Compute current loop dt
	dt_loop = millis() - t_loop_last;
	t_loop_last = millis();

	// Check long loop time
	is_dt_change = dt_loop > 60;

	// Check if either buffer more than half full
	is_buff_flooding =
		c_rx >= 96 ||
		c_tx >= 96 ||
		a_rx >= 96 ||
		a_tx >= 96;


	if (
		is_dt_change ||
		is_buff_flooding
		)
	{

		// Skip first 1k runs
		if (cnt_loop_tot >= 1000) {

			// Get message id
			sprintf(msg, "**CHANGE DETECTED** [CheckLoop] |%s%s%s:",
				is_dt_change ? "Loop DT Change|" : "",
				is_buff_flooding ? "Buffer Flooding|" : "");

			// Log/print message
			sprintf(str, "%s cnt_loop:%d|%d dt_loop=%d|%d c_rx=%d|%d c_tx=%d|%d a_rx=%d|%d a_tx=%d|%d",
				msg, cnt_loop_short, cnt_loop_tot, dt_loop, dt_loop_last, c_rx, c_rx_last, c_tx, c_tx_last, a_rx, a_rx_last, a_tx, a_tx_last);
			DebugFlow(str);
		}
	}

	// Store vars
	dt_loop_last = dt_loop;
	c_rx_last = c_rx;
	c_tx_last = c_tx;
	a_rx_last = a_rx;
	a_tx_last = a_tx;
}

// PRINT ALL FUNCTION ENTRY AND EXITS: str_where = ["start", "end"]
void DebugAllFun(const char *fun, int line, int mem)
{
	// Local vars
	static char str[100] = { 0 }; str[0] = '\0';
	static char fun_copy[100] = { 0 };
	static int line_copy = 0;
	static float mem_copy = 0;
	static float t_s = 0;
	static uint32_t t_check = 0; // ()us
	static uint32_t cnt_chk = 0;
	static uint32_t chk_last = 0;
	int chk_diff = 0;
	
	// Bail if not ready
	if (!fc.isSetup) {
		return;
	}

	// Itterate count
	cnt_chk++;

	// Handle rollover
	if (cnt_chk == UINT32_MAX) {
		cnt_chk = cnt_chk - chk_last;
		chk_last = 0;
	}

	//// Bail if less than x us sinse last
	//if (micros() < t_check + 500) {
	//	return;
	//}

	// Bail if message repeat
	if (strcmp(fun, fun_copy) == 0) {
		return;
	}

	// Store time
	t_check = micros();
	t_s = (float)(millis() - t_sync) / 1000.0f;

	// Copy string
	strcpy(fun_copy, fun);

	// Correct line number
	line_copy = line - 21;

	// Get skipped checks
	chk_diff = cnt_chk - chk_last;

	// Get memory
	mem_copy = (float)mem / 1000;
	

	// FORMAT AND PRINT STRING

	// Print all
	//sprintf(str, "%0.0f|%d|%0.0f %s\n", t_s, cnt_loop_short, mem_copy, fun_copy);

	// Print line
	sprintf(str, "%0.0f %d|%0.2f\n", t_s, line_copy, mem_copy);

	// Print string
	SerialUSB.print(str);

	// Store cnt
	chk_last = cnt_chk;

}

// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	bool do_print = db.print_flow && (db.CONSOLE || db.LCD);
	bool do_log = db.log_flow && DO_LOG;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg, t);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(msg, t);
	}

}

// LOG/PRINT ERRORS
void DebugError(char msg[], bool is_error, uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	bool do_print = db.print_errors && (db.CONSOLE || db.LCD);
	bool do_log = db.log_errors && DO_LOG;

	// Set error flag
	db.isErrLoop = true;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg, t);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(msg, t);
	}

	// Store error info
	if (is_error) {
		err_line[cnt_err < 100 ? cnt_err++ : 99] = Log.cnt_logsStored;
	}
	else {
		warn_line[cnt_warn < 100 ? cnt_warn++ : 99] = Log.cnt_logsStored;
	}
}

// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	bool do_print = db.print_motorControl && (db.CONSOLE || db.LCD);
	bool do_log = db.log_motorControl && DO_LOG;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format message
	sprintf(str, "%s[DebugMotorControl] Change %s: set_in=%s set_out=%s [%s]",
		!pass ? "**WARNING** " : "", pass ? "Succeeded" : "Failed", set_to, fc.motorControl, called_from);

	// Add to print queue
	if (do_print) {
		QueueDebug(str, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(str, millis());
	}

}

// LOG/PRINT MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(char msg[], char called_from[], uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	bool do_print = db.print_motorControl && (db.CONSOLE || db.LCD);
	bool do_log = db.log_motorControl && DO_LOG;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format message
	sprintf(str, "[DebugMotorBocking] %s %lu ms [%s]", msg, t, called_from);

	// Add to print queue
	if (do_print) {
		QueueDebug(str, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(str, millis());
	}

}

// LOG/PRINT MOTOR SPEED CHANGE
void DebugRunSpeed(char agent[], double speed_last, double speed_now)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	bool do_print = db.print_runSpeed && (db.CONSOLE || db.LCD);
	bool do_log = db.log_runSpeed && DO_LOG;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format message
	sprintf(str, "[RunMotor] Changed Motor Speed: agent=%s speed_last=%0.2f speed_new=%0.2f",
		agent, speed_last, speed_now);

	// Add to print queue
	if (do_print) {
		QueueDebug(str, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(str, millis());
	}

}

// LOG/PRINT RECIEVED PACKET DEBUG STRING
void DebugRcvd(R4 *r4, char msg[], bool is_repeat)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	static char msg_out[maxStoreStrLng + 50] = { 0 }; msg_out[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print status
	do_print =
		((r4->instID == "c2r" && ((db.print_c2r && r4->idNow != 'P') || (db.print_rcvdVT && r4->idNow == 'P'))) ||
		(r4->instID == "a2r" && db.print_a2r)) &&
			(db.CONSOLE || db.LCD);
	do_log =
		((r4->instID == "c2r" && db.log_c2r && r4->idNow != 'P') ||
		(r4->instID == "a2r" && db.log_a2r)) &&
		DO_LOG;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Check if this is a repeat
	if (!is_repeat) {
		sprintf(msg_out, "   [RCVD] %s: %s", r4->instID, msg);
	}
	else {
		sprintf(msg_out, "   [*RE-RCVD*] %s: cnt=%d %s", r4->instID, r4->cnt_repeat, msg);
	}

	// Add samp dt for pos data
	if (r4->idNow == 'P') {
		U.f = r4->dat[2];
		sprintf(str, " ts_int=%d dt_samp=%d",
			U.i32, millis() - Pos[cmd.vtEnt].t_msNow);
		strcat(msg_out, str);
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg_out, r4->t_rcvd);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(msg_out, r4->t_rcvd);
	}

}

// LOG/PRINT SENT PACKET DEBUG STRING
void DebugSent(R2 *r2, char msg[], bool is_repeat)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	char msg_out[maxStoreStrLng + 50] = { 0 };
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Set pointer to struct
	if (r2->instID == "r2c") {
		r2 = &r2c;
	}
	else if (r2->instID == "r2a") {
		r2 = &r2a;
	}

	// Get print status
	do_print = ((db.print_r2c && r2->instID == "r2c") || (db.print_r2a && r2->instID == "r2a")) &&
		(db.CONSOLE || db.LCD);
	do_log = ((db.log_r2c && r2->instID == "r2c") || (db.log_r2a && r2->instID == "r2a")) &&
		DO_LOG;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Check if this is a repeat
	if (!is_repeat) {
		sprintf(msg_out, "   [SENT] %s: %s", r2->instID, msg);
	}
	// Add to counters
	else {
		r2->cnt_repeat++;
		sprintf(msg_out, "   [*RE-SENT*] %s: cnt=%d %s", r2->instID, r2->cnt_repeat, msg);
	}

	// Store
	if (do_print) {
		QueueDebug(msg_out, r2->t_sent);
	}

	if (do_log) {
		Log.QueueLog(msg_out, r2->t_sent);
	}

}

// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

#if DO_DEBUG

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static char msg_copy[maxStoreStrLng + 50] = { 0 }; msg_copy[0] = '\0';
	static char queue_state[printQueueSize + 1]; queue_state[0] = '\0';
	static char spc[50] = { 0 }; spc[0] = '\0';
	static char arg[50] = { 0 }; arg[0] = '\0';
	bool is_queue_overflowed = false;
	bool is_mem_overflowed = false;
	uint32_t t_m = 0;
	float t_s = 0;

	// Bail if queue store blocked
	if (fc.doBlockPrintQueue) {
		return;
	}

	// Update printQueue ind
	printQueueIndStore++;

	// Check if ind should roll over 
	if (printQueueIndStore == printQueueSize) {

		// Reset queueIndWrite
		printQueueIndStore = 0;
	}

	// Get message length
	is_mem_overflowed = strlen(msg) >= maxMsgStrLng;

	// Check for overflow
	is_queue_overflowed = printQueue[printQueueIndStore][0] != '\0';

	// Check if queue overflowed or message too long
	if (is_queue_overflowed || is_mem_overflowed) {

		// Handle overflow queue
		if (is_queue_overflowed) {

			// Get list of empty entries
			for (int i = 0; i < printQueueSize; i++) {
				queue_state[i] = printQueue[i][0] == '\0' ? '0' : '1';
			}
			queue_state[printQueueSize] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [QueueDebug] PRINT QUEUE OVERFLOWED: queue_s=%d queue_r=%d queue_state=|%s|",
				printQueueIndStore, printQueueIndRead, queue_state);

			// Set queue back so overflow will write over last print
			printQueueIndStore = printQueueIndStore - 1 >= 0 ? printQueueIndStore - 1 : printQueueSize - 1;
		}

		// Handle overflow char array
		else if (is_mem_overflowed) {

			// Store part of message
			for (int i = 0; i < 100; i++) {
				str[i] = msg[i];
			}
			str[100] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [QueueDebug] MESSAGE TOO LONG: msg_lng=%d max_lng=%d \"%s%s\"",
				strlen(msg), maxMsgStrLng, str, "...");
		}

		// Log error
		if (db.log_errors && DO_LOG) {
			Log.QueueLog(msg_copy, t);
		}

	}

	// Copy message
	else {
		for (int i = 0; i < strlen(msg); i++) {
			msg_copy[i] = msg[i];
		}
		msg_copy[strlen(msg)] = '\0';
	}

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Make time string
	sprintf(str, "[%0.3f][%d]", t_s, cnt_loop_short);

	// Add space after time
	sprintf(arg, "%%%ds", 20 - strlen(str));
	sprintf(spc, arg, '_');

	// Put it all together
	sprintf(printQueue[printQueueIndStore], "%s%s%s\n", str, spc, msg_copy);

	// Check if should print now
	if (db.FASTPRINT) {
		DoAll("PrintDebug");
	}

#endif
}

// PRINT DB INFO
bool PrintDebug()
{

#if DO_DEBUG

	// Bail if nothing in queue
	if (printQueueIndRead == printQueueIndStore &&
		printQueue[printQueueIndStore][0] == '\0') {
		return false;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Itterate send ind
	printQueueIndRead++;

	// Check if ind should roll over 
	if (printQueueIndRead == printQueueSize) {
		printQueueIndRead = 0;
	}

	// Print
	SerialUSB.print(printQueue[printQueueIndRead]);

	// Set entry to null
	printQueue[printQueueIndRead][0] = '\0';

	// Return success
	return true;

#else
	return false;

#endif
}

// FOR PRINTING TO LCD
void PrintLCD(bool do_block, char msg_1[], char msg_2[], char f_siz)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Check if printing blocked
	if (do_block) {
		fc.doBlockWriteLCD = false;
	}
	else if (fc.doBlockWriteLCD) {
		return;
	}

	// Reset
	LCD.clrScr();
	LCD.invert(true);
	LCD.setFont(SmallFont);

	// Print
	if (msg_2[0] != '\0')
	{
		LCD.print(msg_1, 5, 24 - 4);
		LCD.print(msg_2, 5, 24 + 4);
	}
	else {
		LCD.print(msg_1, 5, 24);
	}


	// Prevent overwrite till cleared
	if (do_block) {
		fc.doBlockWriteLCD = true;
	}
}

// CLEAR LCD
void ClearLCD()
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Clear
	LCD.clrScr();

	// Stop blocking LCD log
	fc.doBlockWriteLCD = false;
}

// PRINT SPECIAL CHARICTERS
char* PrintSpecialChars(char chr, bool do_show_byte)
{
#if DO_DEBUG_XXX
	DB_INF();
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
	static char str[10] = { 0 }; str[0] = '\0';
	byte b = chr;

	for (int i = 0; i < 10; i++) {
		str[i] = '\0';
	}

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

// GET AUTODRIVER BOARD STATUS
int GetAD_Status(uint16_t stat_reg, char stat_str[])
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	const static char status_list[16][25] =
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
		if (strcmp(stat_str, status_list[i]) == 0)
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

// LOG TRACKING
void LogTrackingData()
{

	// Bail if not doing any pos logging
	if (!db.log_pos) {
		return;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	static const byte n_samps = 40;
	static int16_t pos_hist[10][n_samps] = { { 0 } };
	static const char id_str[10][n_samps] = { { "Rat VT Pos: |" } ,{ "Rat Px Pos: |" } ,{ "Rob VT Pos: |" } ,{ "Rat EKF Pos: |" } ,{ "Rob EKF Pos: |" },
	{ "Rat VT Vel: |" } ,{ "Rat Px Vel: |" } ,{ "Rob VT Vel: |" } ,{ "Rat EKF Vel: |" } ,{ "Rob EKF Vel: |" } };
	static bool do_log[10] = { db.log_pos_rat_vt, db.log_pos_rat_pixy, db.log_pos_rob_vt, db.log_pos_rat_ekf, db.log_pos_rob_ekf,
		db.log_vel_rat_vt, db.log_vel_rat_pixy, db.log_vel_rob_vt, db.log_vel_rat_ekf, db.log_vel_rob_ekf };
	static int hist_ind = 0;
	static int cnt_last = 0;

	// Bail in ekf not new or not logging
	if (kal.cnt == cnt_last ||
		(!do_log[0] && !do_log[1] && !do_log[2] && !do_log[3])) {
		return;
	}

	// Initilazie store time
	static uint32_t t_last_log = kal.t_update;

	// Store pos values
	if (do_log[0]) {
		pos_hist[0][hist_ind] = Pos[0].posNow;
	}
	if (do_log[1]) {
		pos_hist[1][hist_ind] = Pos[2].posNow;
	}
	if (do_log[2]) {
		pos_hist[2][hist_ind] = Pos[1].posNow;
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
	cnt_last = kal.cnt;

	// Check if hist filled
	if (millis() >= t_last_log + 1000 || hist_ind == n_samps) {

		// Store values in respective string
		for (int i = 0; i < 10; i++) {

			if (!do_log[i]) {
				continue;
			}

			// Add identfiyer string
			sprintf(str, "%s", id_str[i]);

			// Store each value in string
			char s[10];
			for (int j = 0; j < hist_ind; j++) {

				sprintf(s, "%d|", pos_hist[i][j]);
				strcat(str, s);
			}

			// Log
			Log.QueueLog(str, kal.t_update);
		}

		// Reset vals
		hist_ind = 0;
		t_last_log = kal.t_update;
	}
}

// SEND TEST PACKET
void TestSendPack(R2 *r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// EXAMPLE:
	/*
	static uint32_t t_s = 0;
	static int send_cnt = 0;
	static uint16_t pack = 0;
	if (send_cnt == 0 && millis()>t_s + 30) {
	pack++;
	TestSendPack(&r2c, 'Z', 0, 0, 0, 1, true);
	t_s = millis();
	send_cnt++;
	}
	*/

	//// Only send once
	//if (cnt_loop_short > 0 || cnt_loop_tot > 0) {
	//	return;
	//}

	// Queue packet
	QueuePacket(r2, id, dat1, dat2, dat3, pack, do_conf);

	// Fuck with packet
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer, [9]targ
	*/
	//r2->sendQueue[sendQueueInd + 1][8] = 'i';

	// Send packet
	SendPacket(r2);

	// Block resend
	R4 *r4;
	if (r2->instID == "r2c") {
		r4 = &c2r;
	}
	else if (r2->instID == "r2a") {
		r4 = &a2r;
	}
	int r2_ind = CharInd<R2>(id, r2);
	if (r2_ind != -1) {
		r2c.doRcvCheck[r2_ind] = false;
	}

	// Print everything
	DoAll("PrintDebug");
}

// HOLD FOR CRITICAL ERRORS
void RunErrorHold(char msg[], uint32_t t_kill)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static uint32_t t_shutdown = t_kill > 0 ? millis() + t_kill : 0;
	int duty[2] = { 255, 0 };
	bool do_led_on = true;
	int dt = 250;
	float t_s = 0;

	// Print anything left in print queue
	DoAll("PrintDebug");

	// Turn on LCD LED
	ChangeLCDlight(100);

	// Get time seconds
	t_s = (float)(millis() - t_sync) / 1000.0f;

	// Print error
	sprintf(str, "ERROR %0.2fs", t_s);
	PrintLCD(true, msg, str, 't');

	// Loop indefinaitely
	while (true)
	{
		// Flicker lights
		analogWrite(pin.RewLED_R, duty[(int)do_led_on]);
		analogWrite(pin.TrackLED, duty[(int)do_led_on]);
		delay(dt);
		do_led_on = !do_led_on;

		// Check if should shutdown
		if (t_shutdown > 0 && millis() > t_shutdown) {

			// Log/print
			sprintf(str, "!!ERROR!! [RunErrorHold] SHUTTING DOWN BECAUSE: %s", msg);
			DebugError(str, true);
			delay(1000);

			// Set kill switch high
			digitalWrite(pin.PWR_OFF, HIGH);

			// Quit if still powered by USB
			QuitSession();
		}

	}
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// GET ID INDEX
template <typename T> int CharInd(char id, T *r24)
{
#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// Return -1 if not found
	int ind = -1;
	for (int i = 0; i < r24->lng; i++) {

		if (id == r24->id[i]) {
			ind = i;
		}
	}

	// Print warning if not found
	if (ind == -1) {
		sprintf(str, "**WARNING** [CharInd] ID \'%c\' Not Found in %s", id, r24->instID);
		DebugError(str);
	}

	return ind;

}

// BLINK LEDS AT RESTART/UPLOAD
bool StatusBlink(bool do_set, byte n_blinks, uint16_t dt_blink, bool rat_in_blink)
{

	static uint32_t t_blink_last = 0;
	static byte n_cycles = 0;
	static uint16_t dt_cycle = 0;
	static bool do_led_on = true;
	static bool do_blink = true;
	static int cnt_blink = 0;
	static int is_rat_blink = false;
	int duty[2] = { 100, 0 };

	// Set values
	if (do_set) {
		n_cycles = n_blinks;
		dt_cycle = dt_blink;
		do_blink = true;
		is_rat_blink = rat_in_blink;
	}

	// Bail if not running
	else if (!do_blink) {
		return false;
	}

#if DO_DEBUG_XXX
	DB_INF();
#endif

	// Flash sequentially
	if (cnt_blink <= n_cycles) {
		if (millis() > t_blink_last + dt_cycle) {

			// Set LEDs
			analogWrite(pin.TrackLED, duty[(int)do_led_on]);
			if (!is_rat_blink) {
				analogWrite(pin.Disp_LED, duty[(int)do_led_on]);
				analogWrite(pin.RewLED_C, duty[(int)do_led_on]);
			}
			else {
				analogWrite(pin.RewLED_R, duty[(int)do_led_on]);
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
		analogWrite(pin.Disp_LED, 0);
		analogWrite(pin.RewLED_C, rewLEDmin);
		analogWrite(pin.TrackLED, trackLEDdutyMax);
		do_led_on = true;
		cnt_blink = 0;
		do_blink = false;
		is_rat_blink = false;
		return false;
	}
}

// PRINT ALL IN QUEUE: fun_id = ["PrintDebug", "WriteLog"]
bool DoAll(char fun_id[])
{

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	static bool do_skip_next_run = false;
	static bool do_skip_next_print = false;
	uint32_t dt_timeout = 500;
	uint32_t t_timeout = millis() + dt_timeout;
	bool do_loop = true;

	// Bail if skipping
	if (do_skip_next_run) {
		do_skip_next_run = false;
		return false;
	}

	// Loop till done or timeout reached
	while (do_loop && millis() < t_timeout) {
		if (fun_id == "PrintDebug") {
			do_loop = PrintDebug();
		}
		else if (fun_id == "WriteLog") {
			do_loop = Log.WriteLog();
		}

	}

	// Check if timedout
	if (do_loop) {

		// Print error
		if (!do_skip_next_print) {
			sprintf(str, "**WARNING** [DoAll] \"%s\" Timedout after %dms", fun_id, dt_timeout);
			DebugError(str);

			// Set flag
			do_skip_next_print = true;
		}

		// Skip next run
		do_skip_next_run = true;
		return false;
	}

	// Reset flags
	do_skip_next_print = false;
	do_skip_next_run = false;
	return true;

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
		digitalRead(pin.FeedSwitch) == LOW) {

		// Set flag
		is_done = true;
	}

	// Count exeded byte max
	else if (v_cnt_steps >= 255) {

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

	// Itterate count
	v_cnt_steps += v_stepState ? 1 : 0;

	// Set state
	v_stepState = !v_stepState;

	// Step motor
	digitalWrite(pin.ED_STP, v_stepState);
}

// POWER OFF
void Interupt_Power()
{
	// NOT USING
	return;

	// Turn off power
	digitalWrite(pin.PWR_OFF, HIGH);

	// Disable regulators
	digitalWrite(pin.REG_24V_ENBLE, LOW);
	digitalWrite(pin.REG_12V_ENBLE, LOW);
	digitalWrite(pin.REG_5V_ENBLE, LOW);

	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt() {

	// Local vars
	static uint32_t t_debounce = 0; 

	// Exit if < 250 ms has not passed
	if (t_debounce > millis()) {
		return;
	}

	// Run stop in main loop
	v_doIRhardStop = true;

	// Update debounce
	t_debounce = millis() + 250;
}

// DETECT IR SYNC EVENT
void Interupt_IR_Detect()
{
	// Local vars
	static uint32_t t_debounce = 0;

	// Exit if < 25 ms has not passed
	if (millis() < t_debounce) {
		return;
	}

	// Store time
	v_dt_ir = millis() - v_t_irSyncLast;
	v_t_irSyncLast += v_dt_ir;
	v_cnt_ir++;

	// Set flag
	if (t_sync != 0) {
		v_doLogIR = true;
	}

	// Update debounce
	t_debounce = millis() + 50;
}

//// BLOCK/UNBLOCK ALL INTERUPTS do_what=["block", "unblock"]
//inline void ChangeIRQState(char do_what[])
//{
//	/* 
//	NOTE: 
//		Taken from: https://forum.arduino.cc/index.php?topic=421181.0
//	*/
//
//	// Local vars
//	static uint32_t pmask = __get_PRIMASK() & 1;
//
//	// Block interupts
//	if (do_what = "block") {
//
//		// Save global interupt mask
//		pmask = __get_PRIMASK() & 1;
//
//		// Set so all blocked
//		__set_PRIMASK(1);
//	}
//
//	// Unblock interupts
//	else if (do_what = "unblock") {
//		__set_PRIMASK(pmask);
//	}
//}

#pragma endregion

#pragma endregion


void setup() {

	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// XBee 1a (to/from CS)
	r2c.port.begin(57600);

	// XBee 1b (to/from CheetahDue)
	r2a.port.begin(57600);

	// Wait for SerialUSB if debugging
	uint32_t t_check = millis() + 100;
	if (db.CONSOLE) {
		while (!SerialUSB && millis() < t_check);
	}

	// SETUP PINS
	SetupPins();

	// WAIT FOR POWER SWITCH RELEASE
	while (digitalRead(pin.PWR_Swtch) == LOW) {
		delay(10);
	}

	// WAIT FOR POWER SWITCH IF NOT DEBUGGING
	digitalWrite(pin.PWR_OFF, HIGH);
	while (!DO_DEBUG &&
		!DO_DEBUG_XXX &&
		digitalRead(pin.PWR_Swtch) == HIGH);

	// Pause before powering on if in debug mode
	if (DO_DEBUG || DO_DEBUG_XXX) {
		delay(1000);
	}

	// TURN ON POWER
	digitalWrite(pin.PWR_OFF, LOW);
	delayMicroseconds(100);
	digitalWrite(pin.PWR_ON, HIGH);
	delayMicroseconds(100);
	digitalWrite(pin.PWR_ON, LOW);
	delayMicroseconds(100);

	// WAIT FOR POWER SWITCH RELEASE
	while (digitalRead(pin.PWR_Swtch) == LOW) {
		delay(10);
	}

	// ENABLE VOLTGAGE REGULATORS
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_12V_ENBLE, HIGH);
	digitalWrite(pin.REG_5V_ENBLE, HIGH);

	// SHOW RESTART BLINK
	delayMicroseconds(100);
	StatusBlink(true, 1, 100);
	while (StatusBlink());

	// INITIALIZE LCD
	LCD.InitLCD();

	// LOG/PRINT SETUP RUNNING

	// Print to LCD
	ChangeLCDlight(50);
	PrintLCD(true, "SETUP", "MAIN");

	// Log and print to console
	DebugFlow("[setup] RUNNING: Setup...");
	DoAll("PrintDebug");

	// SETUP AUTODRIVER

	// Configure SPI
	PrintLCD(true, "RUN SETUP", "AutoDriver");
	AD_R.SPIConfig();
	delayMicroseconds(100);
	AD_F.SPIConfig();
	delayMicroseconds(100);
	// Reset boards
	AD_Reset();
	PrintLCD(true, "DONE SETUP", "AutoDriver");

	// Make sure motor is stopped and in high impedance
	AD_R.hardHiZ();
	AD_F.hardHiZ();

	// SETUP BIG EASY DRIVER

	// Start BigEasyDriver in sleep
	PrintLCD(true, "RUN SETUP", "Big Easy");
	digitalWrite(pin.ED_RST, HIGH);
	digitalWrite(pin.ED_SLP, LOW);
	PrintLCD(true, "DONE SETUP", "Big Easy");

	// INITIALIZE PIXY
	PrintLCD(true, "RUN SETUP", "Pixy");
	Pixy.init();
	Wire.begin();
	PrintLCD(true, "DONE SETUP", "Pixy");

	// DUMP BUFFER
	PrintLCD(true, "RUN SETUP", "Dump Serial");
	while (c2r.port.available() > 0) {
		c2r.port.read();
	}
	while (a2r.port.available() > 0) {
		a2r.port.read();
	}
	PrintLCD(true, "DONE SETUP", "Dump Serial");

	// RESET VOLITILES AND RELAYS
	v_t_irSyncLast = 0; // (ms)
	v_dt_ir = 0;
	v_cnt_ir = 0;
	v_doIRhardStop = false;
	v_doLogIR = false;
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);

	// CHECK THAT POWER ON
	uint32_t t_check_vcc = millis() + 1000;

	// Loop till vcc or timeout
	PrintLCD(true, "RUN SETUP", "Battery Check");
	while (CheckBattery(true) == 0 && millis() < t_check_vcc);

	// Exit with error if power not on
	if (CheckBattery(true) == 0) {
		// Hold for error
		DebugError("!!ERROR!! [setup] ABORTED: Power Off", true);
		DoAll("PrintDebug");
		RunErrorHold("POWER OFF");
	}
	PrintLCD(true, "DONE SETUP", "Battery Check");

	// SETUP OPENLOG
	PrintLCD(true, "RUN SETUP", "OpenLog");
	DebugFlow("[setup] RUNNING: OpenLog Setup...");
	DoAll("PrintDebug");

	// Setup OpenLog
	if (Log.Setup())
	{
		// Log/print setup finished
		PrintLCD(true, "DONE SETUP", "OpenLog");
		DebugFlow("[setup] SUCCEEDED: OpenLog Setup");
		DoAll("PrintDebug");
	}
	else {
		// Hold for error
		PrintLCD(true, "FAILED SETUP", "OpenLog");
		DebugError("!!ERROR!! [setup] ABORTED: OpenLog Setup", true);
		DoAll("PrintDebug");
		RunErrorHold("OPENLOG SETUP");
	}

	// Create new log file
	PrintLCD(true, "RUN SETUP", "Log File");
	DebugFlow("[setup] RUNNING: Make New Log...");
	if (Log.OpenNewLog() == 0) {
		// Hold for error
		PrintLCD(true, "FAILED SETUP", "Log File");
		DebugError("!!ERROR!! [setup] ABORTED: Setup", true);
		DoAll("PrintDebug");
		RunErrorHold("OPEN LOG FILE");
	}
	else {
		PrintLCD(true, "DONE SETUP", "Log File");
		sprintf(str, "[setup] SUCCEEDED: Make New Log: file_name=%s", Log.logFile);
		DebugFlow(str);
		DoAll("PrintDebug");
	}

	// DEFINE EXTERNAL INTERUPTS
	PrintLCD(true, "RUN SETUP", "Interrupts");
	PrintLCD(true, "SETUP", "Interrupts");
	uint32_t t_check_ir = millis() + 1000;
	bool is_ir_low = false;

	// Check if IR detector already high
	while (!is_ir_low && millis() < t_check_ir) {
		is_ir_low = digitalRead(pin.IRdetect) == LOW;
	}

	// Do not enable if ir detector pin already high
	if (is_ir_low) {
		// IR detector
		attachInterrupt(digitalPinToInterrupt(pin.IRdetect), Interupt_IR_Detect, HIGH);
	}
	else {
		// Skip ir sync setup
		t_sync = 1;
		DebugError("!!ERROR!! [setup] IR SENSOR DISABLED", true);
		DoAll("PrintDebug");
	}

	// Power off
	//attachInterrupt(digitalPinToInterrupt(pin.PWR_Swtch), Interupt_Power, FALLING);

	// IR prox right
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Rt), Interupt_IRprox_Halt, FALLING);

	// IR prox left
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Lft), Interupt_IRprox_Halt, FALLING);

	// Start Feed Arm timer
	FeederArmTimer.attachInterrupt(Interupt_TimerHandler);
	PrintLCD(true, "DONE SETUP", "Interrupts");

	// RESET FEEDER ARM
	DebugFlow("[setup] RUNNING: Reset Feeder Arm...");
	DoAll("PrintDebug");
	PrintLCD(true, "RUN SETUP", "Retract Arm");
	Reward.RetractFeedArm();
	while (Reward.doRetractArm) {
		Reward.CheckFeedArm();
	}
	DebugFlow("[setup] FINISHED: Reset Feeder Arm");
	DoAll("PrintDebug");

	// CLEAR LCD
	ChangeLCDlight(0);
	ClearLCD();

	// SET DEFAULTS
	fc.isManualSes = true;
	fc.doAllowRevMove = true;

	// PRINT AVAILABLE MEMORY
	sprintf(str, "[setup] AVAILABLE MEMORY: %0.2fKB",
		(float)freeMemory() / 1000);
	DebugFlow(str);
	DoAll("PrintDebug");

	// PRINT SERIAL RING BUFFER SIZE
	sprintf(str, "[setup] RING BUFFER SIZE: %dB", SERIAL_BUFFER_SIZE);
	DebugFlow(str);
	DoAll("PrintDebug");

	// PRINT DEBUG STATUS
	sprintf(str, "[setup] RUNNING IN %s MODE: |%s%s%s%s%s",
		DO_DEBUG ? "DEBUG" : "RELEASE", db.CONSOLE ? "PRINT TO CONSOLE|" : "",
		db.LCD ? "PRINT TO LCD|" : "", DO_LOG ? "LOGGING TO OPENLOG|" : "",
		db.FASTPRINT ? "FAST PRINTING|" : "", db.FASTLOG ? "FAST LOGGING|" : "");
	DebugFlow(str);

	// PRINT SETUP FINISHED
	DebugFlow("[setup] FINISHED: Setup");
	DoAll("PrintDebug");

	// Flag setup done
	fc.isSetup = true;

	// TEMP
	//Log.TestLoad(0, "LOG00035.CSV");
	//Log.TestLoad(2500);

}


void loop() {

#pragma region //--- ONGOING OPPERATIONS ---

	// Local vars
	static char horeStr[maxStoreStrLng] = { 0 }; horeStr[0] = '\0';

	// CHECK LOOP TIME AND MEMORY
	CheckLoop();

	// PARSE CHEETAHDUE SERIAL INPUT
	GetSerial(&a2r);

	// SEND CHEETAHDUE DATA
	SendPacket(&r2a);

	// PARSE CS SERIAL INPUT
	GetSerial(&c2r);

	// SEND CS DATA
	SendPacket(&r2c);

	// RESEND CHEETAHDUE DATA
	CheckResend(&r2a);

	// RESEND CS DATA
	CheckResend(&r2c);

	// PRINT QUEUED DB
	PrintDebug();

	// STORE QUEUED LOGS
	Log.WriteLog();

	// SEND SERIAL DATA
	if (fc.doLogSend) {
		// Send log
		Log.StreamLogs();

		// Print
		if (!fc.doLogSend) {
			DebugFlow("[loop] FINISHED SENDING LOGS");
		}
	}

	// GET BUTTON INPUT
	if (GetButtonInput())
	{

		// Button triggered reward
		if (fc.doBtnRew) {
			// Bail if already rewarding
			if (Reward.isRewarding) {
				DebugError("**WARNING** [loop] ABORTED: Button Reward Triggered When Already Running Reward");
			}
			else {
				// Set mode
				Reward.SetRewMode("Button");
				Reward.StartRew();
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

	// CHECK FOR IR TRIGGERED HALT
	if (v_doIRhardStop)
	{
		IRprox_Halt();
		v_doIRhardStop = false;
	}

	// LOG NEW IR EVENT
	if (v_doLogIR)
	{
		// Log event if streaming started
		sprintf(horeStr, "[loop] IR Sync Event: cnt=%d dt=%dms", v_cnt_ir, v_dt_ir);
		DebugFlow(horeStr, v_t_irSyncLast);

		// Reset flag
		v_doLogIR = false;
	}

	// END ONGOING REWARD
	if (Reward.EndRew()) {

		// Tell CS what zone was rewarded and get confirmation
		if (strcmp(Reward.modeReward, "Now") != 0 &&
			strcmp(Reward.modeReward, "Button") != 0) {

			QueuePacket(&r2c, 'Z', Reward.zoneInd + 1, 0, 0, 0, true);
		}
	}

	// RETRACT FEEDER ARM
	Reward.CheckFeedArm();

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

#pragma endregion

#pragma region //--- HOLD FOR HANDSHAKE ---
	if (!fc.isHandShook)
	{

		// PULSE TRACKER LED
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

		// CHECK FOR SYNC IR HANDSHAKE PULSE
		if (t_sync == 0)
		{

			// Check for setup ir pulse
			if (abs(75 - v_dt_ir) < 10) {

				// Set sync time
				t_sync = v_t_irSyncLast;

				// Log/print sync time
				sprintf(horeStr, "SET SYNC TIME: %dms", t_sync);
				DebugFlow(horeStr);

				// Send handshake 
				QueuePacket(&r2c, 'h', n_pings, 0, 0, 0, true);
				SendPacket(&r2c);
			}
			// Restart loop
			else {
				return;
			}
		}

		// RESET LCD
		if (!fc.isManualSes &&
			fc.isLitLCD) {

			fc.doChangeLCDstate = true;
		}
		ClearLCD();

		// RESET SOLONOIDS
		if (digitalRead(pin.Rel_EtOH) == HIGH) {
			OpenCloseEtOHSolenoid();
		}
		if (digitalRead(pin.Rel_Rew) == HIGH) {
			OpenCloseRewSolenoid();
		}

		// PRINT AD BOARD STATUS
		sprintf(horeStr, "[loop] BOARD R STATUS: %04X", AD_R.getStatus());
		DebugFlow(horeStr);
		sprintf(horeStr, "[loop] BOARD F STATUS: %04X", AD_F.getStatus());
		DebugFlow(horeStr);

		// PRINT AVAILABLE MEMORY
		sprintf(horeStr, "[loop] AVAILABLE MEMORY: %0.2fKB",
			(float)freeMemory() / 1000);
		DebugFlow(horeStr);

		// SET FLAG
		fc.isHandShook = true;
		DebugFlow("[loop] READY TO ROCK!");

	}

#pragma endregion

#pragma region //--- (T) SYSTEM TESTS ---

	if (c2r.idNow == 'T' && c2r.isNew)
	{
		// Store message data
		cmd.testCond = (byte)c2r.dat[0];
		cmd.testDat = (byte)c2r.dat[1];

		// Set testing flag
		db.is_runTest = true;

		// Set run pid calibration flag
		if (cmd.testCond == 1)
		{
			// Log/rint settings
			sprintf(horeStr, "[loop] RUN PID CALIBRATION = kC=%0.2f", kC);
			DebugFlow(horeStr);

			// Set flag
			db.do_pidCalibration = true;
		}

		// Update Halt Error test run speed
		else if (cmd.testCond == 2)
		{
			// Store new speed
			double new_speed = double(cmd.testDat);

			// Print speed
			sprintf(horeStr, "[loop] HALT ERROR SPEED = %0.0f cm/sec", new_speed);
			DebugFlow(horeStr);

			if (new_speed > 0) {
				// Run motor
				RunMotor('f', new_speed, "Override");
			}
			else {
				// Halt robot
				AD_R.hardStop();
				AD_F.hardStop();
			}
		}

		// Symulated rat test
		else if (cmd.testCond == 3)
		{
			db.do_simRatTest = true;
		}
	}

	// Run position debugging
	if (db.do_posDebug)
	{
		static double pos_last[3] = { 0 };
		bool is_new = false;

		// Check if position values changed
		for (int i = 0; i < 3; i++) {
			if (Pos[i].posNow != pos_last[i]) {
				is_new = true;
			}
			pos_last[i] = Pos[i].posNow;
		}

		// Plot new data
		if (is_new)
		{
			// Get setpoint pos
			double setpoint_vt = Pos[1].posNow + Pid.setPoint;
			double setpoint_ekf = kal.RobPos + Pid.setPoint;

			// Plot pos
			if (db.do_posPlot) {
				/*
				{@Plot.Pos.Pos[0].Blue Pos[0].posNow}{@Plot.Pos.Pos[2].Green Pos[2].posNow}{@Plot.Pos.Pos[1].Orange Pos[1].posNow}{@Plot.Pos.setpoint_vt.Orange setpoint_vt}{@Plot.Pos.ratEKF.Black kal.RatPos}{@Plot.Pos.robEKF.Red kal.RobPos}{@Plot.Pos.setpoint_ekf.Red kal.setpoint_ekf}
				*/
				millis();
			}

			// Turn on rew led when near setpoint
			if (Pid.error > -0.5 && Pid.error < 0.5 &&
				Pos[0].is_streamStarted &&
				Pos[1].is_streamStarted) {

				analogWrite(pin.RewLED_C, 10);
			}
			else {
				analogWrite(pin.RewLED_C, rewLEDmin);
			}

			// Print pos data
			char str1[50];
			char str2[50];
			sprintf(str1, "Rat Dst = %0.2fcm", kal.RatPos - kal.RobPos);
			sprintf(str2, "Pid Err = %0.2fcm", Pid.error);
			PrintLCD(false, str1, str2, 't');

		}
	}

	// Run Pid calibration
	if (db.do_pidCalibration)
	{
		double new_speed = Pid.RunCalibration();

		// Run motors
		if (Pid.cal_isPidUpdated)
		{
			if (new_speed >= 0) {
				RunMotor('f', new_speed, "Override");
			}
			// Print values
			/*
			millis()%50 == 0
			{Pid.cal_isCalFinished}{"SPEED"}{Pid.cal_ratVel}{"ERROR"}{Pid.cal_errNow}{Pid.cal_errArr[0]}{Pid.cal_errArr[1]}{Pid.cal_errArr[2]}{Pid.cal_errArr[3]}{"PERIOD"}{Pid.cal_PcNow}{Pid.cal_cntPcArr[0]}{Pid.cal_PcArr[0]}{Pid.cal_cntPcArr[1]}{Pid.cal_PcArr[1]}{Pid.cal_cntPcArr[2]}{Pid.cal_PcArr[2]}{Pid.cal_cntPcArr[3]}{Pid.cal_PcArr[3]}{Pid.cal_PcAll}
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

#pragma region //--- (h) PING TEST ---
	if (c2r.idNow == 'h' && c2r.isNew)
	{
		// Run hardware test
		HardwareTest();

		// Send final ping times
		QueuePacket(&r2c, 't', n_pings + 1, dt_pingRoundTrip[0], dt_pingRoundTrip[1], 0, true);

		// Set flag
		fc.isSesStarted = true;

		// Indicate completion
		StatusBlink(true, 10, 100);

	}
#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (c2r.idNow == 'S' && c2r.isNew)
	{
		// Store message data
		cmd.sesCond = (byte)c2r.dat[0];
		cmd.soundCond = (byte)c2r.dat[1];
		millis();
		// Set condition
		if (cmd.sesCond == 1) {
			fc.isManualSes = false;
			DebugFlow("[loop] DO TRACKING");
		}
		else {
			fc.isManualSes = true;
			DebugFlow("[loop] DONT DO TRACKING");
		}
		// Set reward tone
		if (cmd.soundCond == 0) {
			// No sound
			QueuePacket(&r2a, 's', 0);
			DebugFlow("[loop] NO SOUND");
		}
		else if (cmd.soundCond == 1) {
			// Use white noise only
			QueuePacket(&r2a, 's', 0);
			DebugFlow("[loop] DONT DO TONE");
		}
		else {
			// Use white and reward noise
			QueuePacket(&r2a, 's', 2);
			DebugFlow("[loop] DO TONE");
		}

	}
#pragma endregion

#pragma region //--- (Q) DO QUIT ---
	if (c2r.idNow == 'Q' && c2r.isNew) {

		// Log/print event
		DebugFlow("[loop] DO QUIT");

		// Set flags and delay time
		fc.doQuit = true;

		// Tell CheetahDue to quit 
		QueuePacket(&r2a, 'q', 0, 0, 0, 0, true);

		// Block all motor control
		SetMotorControl("Halt", "Quit");

	}

	// Check if time to quit
	if (fc.doQuit) {

		// Wait for any unfinished opperations
		if (
			!SendPacket(&r2a) &&
			!SendPacket(&r2c) &&
			!CheckResend(&r2a) &&
			!CheckResend(&r2c) &&
			!Log.WriteLog() &&
			!PrintDebug()) {

			// Tell CS quit is done
			if (!fc.isQuitConfirmed) {
				QueuePacket(&r2c, 'D', 0, 0, 0, c2r.pack[CharInd<R4>('Q', &c2r)], true);
				fc.isQuitConfirmed = true;
			}

			// Quit after 100 ms
			if (t_quit == 0) {
				t_quit = millis() + 100;
			}

			// Quit
			else if (fc.doQuit && millis() > t_quit)
			{
				// Quit session
				DebugFlow("[loop] QUITING...");
				QuitSession();
			}
		}
	}


#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (c2r.idNow == 'M' && c2r.isNew) {

		// Store move pos
		cmd.moveToTarg = c2r.dat[0];

		// Align target pos to feeder
		cmd.moveToTarg = cmd.moveToTarg - feedDist;
		cmd.moveToTarg = cmd.moveToTarg < 0 ? cmd.moveToTarg + (140 * PI) : cmd.moveToTarg;

		// Set flags
		fc.doMove = true;

		sprintf(horeStr, "[loop] DO MOVE: targ=%0.2fcm", cmd.moveToTarg);
		DebugFlow(horeStr);
	}

	// Perform movement
	if (fc.doMove) {

		// Compute move target
		if (!Move.isTargSet) {

			// If succesfull
			if (Move.CompTarg(kal.RobPos, cmd.moveToTarg)) {

				// Start running
				if (SetMotorControl("MoveTo", "MoveTo")) {

					if (RunMotor(Move.moveDir, moveToSpeed, "MoveTo")
						) {

						// Print message
						sprintf(horeStr, "[loop] RUNNING: MoveTo: targ_dist=%0.2fcm move_dir=\'%c\'...",
							Move.targDist, Move.moveDir);
						DebugFlow(horeStr);

						// Set flag
						Move.isMoveStarted = true;
					}

					// Failed to run motor
					else {
						// Reset control
						SetMotorControl("None", "MoveTo");

						// Set flags
						Move.doAbortMove = true;
					}
				}

				// Failed to take motor control
				else {
					// Set flags
					Move.doAbortMove = true;
				}

			}
		}

		// Check if robot is ready to be stopped
		else if (!Move.isTargReached && !Move.doAbortMove) {

			// Do deceleration
			double new_speed = Move.DecelToTarg(kal.RobPos, kal.RobVel, 40, 10);

			// Change speed if > 0
			if (new_speed > 0 && new_speed != runSpeedNow) {

				RunMotor(Move.moveDir, new_speed, "MoveTo");
			}

		}

		// Check if target reached or move aborted
		else {

			// Hard stop
			HardStop("MoveTo");

			// Set motor cotrol to None
			SetMotorControl("None", "MoveTo");

			// Log/print final move status
			if (Move.isTargReached) {

				// Print success message
				sprintf(horeStr, "[loop] SUCCEEDED: MoveTo: targ_dist=%0.2fcm dist_error=%0.2fcm move_dir=\'%c\'",
					Move.targDist, Move.GetMoveError(kal.RobPos), Move.moveDir);
				DebugFlow(horeStr);

				// Tell CS movement is done
				QueuePacket(&r2c, 'D', 0, 0, 0, c2r.pack[CharInd<R4>('M', &c2r)], true);
			}

			// Log/print error
			else if (Move.doAbortMove) {

				// Print failure message
				sprintf(horeStr, "!!ERROR!! [loop] ABORTED: MoveTo: targ_set=%s ekf_ready=%s move_started=%s targ_dist=%0.2fcm dist_error=%0.2fcm move_dir=\'%c\'",
					Move.isTargSet ? "1" : "0", fc.isEKFReady ? "1" : "0", Move.isMoveStarted ? "1" : "0", Move.targDist, Move.GetMoveError(kal.RobPos), Move.moveDir);
				DebugError(horeStr, true);
			}

			// Reset flags
			fc.doMove = false;
			Move.MoveToReset();
		}
	}
#pragma endregion

#pragma region //--- (R) RUN REWARD ---
	if (c2r.idNow == 'R' && c2r.isNew) {

		// Store message data
		cmd.rewPos = c2r.dat[0];
		cmd.rewCond = (byte)c2r.dat[1];
		cmd.rewZoneInd = (byte)c2r.dat[2] - 1;
		cmd.rewDelay = (byte)c2r.dat[2];


		// Bail if already rewarding
		if (Reward.isRewarding) {
			DebugError("**WARNING** [loop] ABORTED: \'R\' Reward Triggered When Already Running Reward");
			return;
		}

		// Imediate reward
		else if (cmd.rewCond == 1) {

			// Log/print
			DebugFlow("[loop] DO NOW REWARD");

			// Set mode
			Reward.SetRewMode("Now", cmd.rewZoneInd);

			// Start reward
			Reward.StartRew();

		}

		// Cued reward
		else if (cmd.rewCond == 2) {

			// Log/print
			DebugFlow("[loop] DO CUED REWARD");

			// Set mode
			Reward.SetRewMode("Cue", cmd.rewZoneInd);

			// Set flag
			fc.doRew = true;
		}

		// Free reward
		else if (cmd.rewCond == 3) {

			// Log/print
			DebugFlow("[loop] DO FREE REWARD");

			// Set mode
			Reward.SetRewMode("Free", cmd.rewDelay);

			// Set flag
			fc.doRew = true;
		}

	}

	// CHECK REWARD BOUNDS
	if (fc.doRew) {

		// If not rewarding 
		if (!Reward.isRewarding) {

			// Zone bounds not set yet
			if (!Reward.isBoundsSet) {

				// Compute bounds
				Reward.CompZoneBounds(kal.RatPos, cmd.rewPos);

				// Print message
				sprintf(horeStr, "[loop] SET REWARD ZONE: center=%0.2fcm from=%0.2fcm to=%0.2fcm",
					Reward.rewCenterRel, Reward.boundMin, Reward.boundMax);
				DebugFlow(horeStr);
			}

			// Zone not triggered yet
			else if (!Reward.isZoneTriggered) {

				// Check each zone
				if (Reward.CheckZoneBounds(kal.RatPos)) {

					// Start reward
					Reward.StartRew();

					// Reset flag
					fc.doRew = false;

					// Print message
					sprintf(horeStr, "[loop] REWARDED ZONE: occ=%dms zone=%d from=%0.2fcm to=%0.2fcm",
						Reward.occRewarded, Reward.zoneRewarded, Reward.boundsRewarded[0], Reward.boundsRewarded[1]);
					DebugFlow(horeStr);
				}
			}

			// Check if rat passed all bounds
			if (Reward.isAllZonePassed &&
				!Reward.isZoneTriggered) {

				// Print reward missed
				sprintf(horeStr, "[loop] REWARD MISSED: rat=%0.2fcm bound_max=%0.2fcm",
					kal.RatPos, Reward.boundMax);
				DebugFlow(horeStr);

				// Reset flags
				Reward.RewardReset();
				fc.doRew = false;
			}
		}
	}

#pragma endregion

#pragma region //--- (H) HALT ROBOT STATUS ---
	if (c2r.idNow == 'H' && c2r.isNew) {

		// Store message data
		fc.doHalt = c2r.dat[0] != 0 ? true : false;

		if (fc.doHalt) {

			// Log/print
			DebugFlow("[loop] HALT STARTED");

			// Stop pid and set to manual
			HardStop("Halt");

			// Block motor control
			SetMotorControl("Halt", "Halt");

		}
		else {

			// Log/print
			DebugFlow("[loop] HALT FINISHED");

			// Open motor control
			SetMotorControl("Open", "Halt");

		}
	}
#pragma endregion

#pragma region //--- (B) BULLDOZE RAT STATUS ---
	if (c2r.idNow == 'B' && c2r.isNew) {

		// Store message data
		cmd.bullDel = (byte)c2r.dat[0];
		cmd.bullSpeed = (byte)c2r.dat[1];

		// Local vars
		bool is_mode_changed = false;

		// Reinitialize bulldoze
		Bull.BullReinitialize(cmd.bullDel, cmd.bullSpeed, "loop \'B\'");

		// Check if mode should be changedchanged
		if (cmd.bullSpeed > 0) {

			// Mode changed
			if (!fc.doBulldoze) {

				// Log/print event
				DebugFlow("[loop] SET BULLDOZE ON");

				// Set flags
				is_mode_changed = true;
				fc.doBulldoze = true;

			}
			// Only settings changed
			else {
				is_mode_changed = false;
			}
		}
		else {
			// Mode changed
			if (fc.doBulldoze)
			{
				// Log/print event
				DebugFlow("[loop] SET BULLDOZE OFF");

				// Set flags
				is_mode_changed = true;
				fc.doBulldoze = false;

			}
			// Only settings changed
			else {
				is_mode_changed = false;
			}
		}

		// Don't exicute until rat is in and mode is changed
		if (fc.isTrackingEnabled &&
			is_mode_changed) {

			if (fc.doBulldoze) {

				// Log/print event
				DebugFlow("[loop] BULLDOZE ON");

				// Turn bulldoze on
				Bull.BullOn("loop \'B\'");
			}
			else {

				// Log/print event
				DebugFlow("[loop] BULLDOZE OFF");

				// Turn bulldoze off
				Bull.BullOff("loop \'B\'");
			}
		}

	}
#pragma endregion

#pragma region //--- (I) START/STOP PID ---
	if (c2r.idNow == 'I' && c2r.isNew)
	{
		// Store message data
		fc.isRatIn = c2r.dat[0] != 0 ? true : false;

		if (fc.isRatIn) {

			// Pid started by InitializeTracking()
			DebugFlow("[loop] RAT IN");

			// Reset rat pos data
			Pos[0].PosReset();
			Pos[2].PosReset();
		}
		else {

			// Log/print event
			DebugFlow("[loop] RAT OUT");

			// Turn off bulldoze
			Bull.BullOff("loop \'I\'");
			fc.doBulldoze = false;

			// Turn off pid
			Pid.PID_Stop("loop \'I\'");

			// Set to stop tracking
			fc.isTrackingEnabled = false;
		}
	}
#pragma endregion

#pragma region //--- (V) GET STREAM STATUS ---
	if (c2r.idNow == 'V' && c2r.isNew)
	{
		fc.doStreamCheck = true;
	}

	// Check for streaming
	if (fc.doStreamCheck && Pos[1].is_streamStarted)
	{
		// Log/print event

		DebugFlow("[loop] STREAMING CONFIRMED");
		// Send streaming confirmation
		QueuePacket(&r2c, 'D', 0, 0, 0, c2r.pack[CharInd<R4>('V', &c2r)], true);

		// Reset flag
		fc.doStreamCheck = false;

		// Set flag to begin sending vcc
		fc.doSendVCC = true;
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (c2r.idNow == 'P' && c2r.isNew)
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

			// Set rat vt and pixy to setpoint if rat not in 
			if (!fc.isRatIn && !db.do_posDebug) {
				Pos[0].SwapPos(Pos[1].posAbs + Pid.setPoint, Pos[1].t_msNow);
				Pos[2].SwapPos(Pos[1].posAbs + Pid.setPoint, Pos[1].t_msNow);
			}

			// Log/print first sample
			if (!Pos[1].is_streamStarted) {

				// Log/print
				sprintf(horeStr, "[loop] FIRST ROBOT VT RECORD: pos_abs=%0.2f pos_rel=%0.2f n_laps=%d",
					Pos[1].posAbs, Pos[1].posNow, Pos[1].nLaps);
				DebugFlow(horeStr);

				// Set flag
				Pos[1].is_streamStarted = true;
			}
		}

		// Handle rat vt data
		else if (cmd.vtEnt == 0) {

			// Update only after rat in
			if (fc.isRatIn || db.do_posDebug) {

				// Update rat VT
				Pos[cmd.vtEnt].UpdatePos(cmd.vtCM[cmd.vtEnt], cmd.vtTS[cmd.vtEnt]);

				// Use rat vt for pixy if running simulated rat test
				if (cmd.vtEnt == 0 && db.do_simRatTest) {
					Pos[2].SwapPos(Pos[0].posAbs, Pos[0].t_msNow);
				}

				// Log/print first sample
				if (!Pos[0].is_streamStarted) {

					// Log/print
					sprintf(horeStr, "[loop] FIRST RAT VT RECORD: pos_abs=%0.2f pos_rel=%0.2f n_laps=%d",
						Pos[0].posAbs, Pos[0].posNow, Pos[0].nLaps);
					DebugFlow(horeStr);

					// Set flag
					Pos[0].is_streamStarted = true;
				}
			}

		}

	}
#pragma endregion

#pragma region //--- (L) SEND LOG ---
	if (c2r.idNow == 'L' && c2r.isNew) {

		// Flag to begin sending
		fc.doLogSend = c2r.dat[0] == 1 ? true : false;

		// Check for 2 way confirmation before sending log
		if (!fc.doLogSend) {

			// Log/print
			sprintf(horeStr, "[loop] SENDING LOG: logs_stored=~%d b_stored=~%d",
				Log.cnt_logsStored, Log.cnt_logBytesStored);
			DebugFlow(horeStr);

			// Send number of log bytes being sent
			QueuePacket(&r2c, 'U', Log.cnt_logBytesStored, 0, 0, 0, true);

			// Block sending vcc updates
			fc.doSendVCC = false;
		}

		// Begin sending log
		else {

			// Log/print
			DebugFlow("[loop] DO SEND LOG");

			// Set send time
			Log.t_beginSend = millis() + Log.dt_beginSend;

			// Store remaining logs
			DoAll("WriteLog");
		}

	}

#pragma endregion

#pragma region //--- UPDATE TRACKING ---

	// UPDATE PIXY
	CheckPixy();

	// CHECK IF RAT POS FRAMES DROPPING
	CheckSampDT();

	// INITIALIZE RAT AHEAD
	InitializeTracking();

	// UPDATE EKF
	UpdateEKF();

	// UPDATE PID AND SPEED
	double new_speed = Pid.UpdatePID();

	if (new_speed == 0) {
		HardStop("Pid");
	}
	else if (new_speed > 0) {
		RunMotor('f', new_speed, "Pid");
	}

	// UPDATE BULLDOZER
	Bull.UpdateBull();

	// LOG TRACKING DATA
	LogTrackingData();

#pragma endregion

}
