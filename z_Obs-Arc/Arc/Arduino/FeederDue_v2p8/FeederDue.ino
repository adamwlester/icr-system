
//-------FEEDERDUE-------

// XBee DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size) 
// XBee DO (to UART rx) buffer = 202 bytes
// DUE SERIAL_BUFFER_SIZE = 128
// SerialUSB receive buffer size is now 512 (ARDUINO 1.5.2 BETA - 2013.02.06)

#pragma region ---------LIBRARIES & PACKAGES---------


//-------SOFTWARE RESET----------
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

//----------LIBRARIES------------

// General
#include <string.h>

// AutoDriver

#include <SPI.h>

#include "AutoDriver_Due.h"

// Pixy

#include <Wire.h>

#include <PixyI2C.h>

// LCD
#include <LCD5110_Graph.h>

// TinyEKF
#define N 4     // States
#define M 6     // Measurements
#include <TinyEKF.h>

#pragma endregion 


#pragma region ---------VARIABLE DECLARATION---------

//----------DEFINE PINS---------

// Autodriver
const int pin_AD_CSP = 5;
//const int pin_AD_Busy = 4;
const int pin_AD_Reset = 3;

// Display
const int pin_Disp_Power = A0;
const int pin_Disp_SCK = 8;
const int pin_Disp_MOSI = 9;
const int pin_Disp_DC = 10;
const int pin_Disp_RST = 11;
const int pin_Disp_CS = 12;
const int pin_Disp_LED = 13;

// LEDs
const int pin_TrackLED = 6;
const int pin_RewLED = 7;

// Relays
const int pin_Rel_1 = 44;
const int pin_Rel_2 = 45;

// Note: pins bellow are all used for external interupts 
// and must all be members of the same port (PortA)
// IR Senosors
const int pin_IR_Rt = 42;
const int pin_IR_Lft = 43;

// Buttons
const int pin_Btn[4] = { A4, A3, A2, A1 };
volatile uint32_t t_debounce[3] = { millis(), millis(), millis() };

//----------VARIABLE SETUP----------

// Debug print
const bool doPrint_flow = true;
const bool doPrint_pid = false;
const bool doPrint_rcvd = false;
const bool doPrint_sent = false;
const bool doPrint_resent = false;
const bool doPrint_dropped = false;
String printQueue[10];
static int printQueue_lng =
sizeof(printQueue) / sizeof(printQueue[0]);
bool doPrint = false;
// debug tracking
uint32_t t_loopMain = millis();
uint32_t t_loopRead = millis();
int cnt_droppedPacks;
int cnt_overflowEvt = 0;
int cnt_timeoutEvt = 0;
int cnt_packResend = 0;

// Bool flow control
bool fc_isFirstPass = true;
bool fc_doStreamCheck = false;
bool fc_doRunPID = false;
bool fc_doTrackRat = false;
bool fc_doRewTone = true;
bool fc_doQuit = false;
bool fc_isStreaming = false;
bool fc_doMove = false;
bool fc_isMoveTargSet = false;
bool fc_doCue = false;
bool fc_isCueTargSet = false;
volatile bool fc_doRew = false;
bool fc_isRewarding = false;
bool fc_doHalt = false;
bool fc_isHalted = false;
volatile bool fc_isIRtripped = false;
bool fc_doCheckDoneRcvd = false;

// Start/Quit
byte setupCmd[2];
uint32_t t_quitCmd;

// SERIAL COM
uint32_t t_sync = 0;

// Serial from CS
const char c2r_head[2] = { '_', '<' };
const char c2r_foot = '>';
const char c2r_id[10] = {
	'S', // start session
	'Q', // quit session
	'M', // move to position
	'R', // dispense reward
	'H', // halt movement
	'B', // bulldoze rat
	'I', // start/end pid
	'P', // position data
	'C', // request stream status
	'Y', // confirm done recieved
};
char msg_id = ' ';
bool msg_pass = false;
uint16_t t_rsvd = millis(); // (ms)
uint16_t t_rsvdLast; // (ms)

// Serial to CS
const char r2c_head = '{';
const char r2c_foot = '}';
const char r2c_id[9] = {
	'S', // start session
	'Q', // quit session
	'M', // move to position
	'R', // dispense reward
	'H', // halt movement
	'B', // bulldoze rat
	'I', // start/end pid
	'D', // execution done
	'C', // connected and streaming
};
uint32_t t_resendDone = millis();
//uint32_t resendDone_cnt = 0;
//uint32_t resendDone_max = 5;

// Serial to other ard
const char r2a_head = '[';
const char r2a_foot = ']';
short r2a_packCnt = 0;
const char r2a_id[5] = {
	'r', // reward tone
	'w', // white noise on
	'o', // white noise off
	't', // set sync time
	'q', // quit/reset
};
uint16_t t_sent = millis();
uint16_t t_sentLast; // (ms)

// Outgoing data
byte r2_queue[10][5];
const int r2_lngR = 10;
const int r2_lngC = 5;
bool doSend = false;

// Serial packet tracking
uint16_t packNow;
uint16_t packLast = 0;
int packTot = 0;
// last packet
const int c2r_idLng = sizeof(c2r_id) / sizeof(c2r_id[0]);
const int r2c_idLng = sizeof(r2c_id) / sizeof(r2c_id[0]);
const int r2a_idLng = sizeof(r2a_id) / sizeof(r2a_id[0]);
uint16_t c2r_packLast[c2r_idLng];
uint16_t r2c_packLast[r2c_idLng];
uint16_t c2r_packRepeat[c2r_idLng];
int c2r_cntRepeat[c2r_idLng];
// packet history
uint16_t c2r_packHist[10];
uint16_t r2c_packHist[10];
char c2r_idHist[10];
char r2c_idHist[10];
const int c2r_hLng = sizeof(c2r_packHist) / sizeof(c2r_packHist[0]);
const int r2c_hLng = sizeof(r2c_packHist) / sizeof(r2c_packHist[0]);

// Serial VT
byte vtEnt;
float vtCM[2];
uint32_t vtTS[2];

// Pixy
const float pixyTarg = 16; // (cm)
const double pixyCoeff[5] = {
	0.000000064751850,
	-0.000029178819914,
	0.005627883140337,
	-0.690663759936117,
	38.949828090278942
};

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 80; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)

// Kalman model measures
float ekf_ratPos = 0;
float ekf_robPos = 0;
float ekf_ratVel = 0;
float ekf_robVel = 0;

// PID Settings
bool do_includeTerm[2] = { true, true };
const uint32_t loopTim = 60; // pid sample rate (ms)
const float kC = 5; // critical gain kC = 5 * (1 / 0.6)
const float pC = 3; // oscillation period          

// Movement
float moveSpeed = maxSpeed;
float movePos;
float moveSum;
float moveDist;
float distLeft;
float distLast;
char moveDir;
float moveStart;
byte bullSec;

// Reward
const uint32_t solDir = 2000; // (ms) sol open duration
const uint32_t ledDir = 1000; // (ms) light on duration
volatile byte solState = HIGH;
uint32_t t_rewSolClose;
uint32_t t_rewLEDOff;
float rewPos;

// LEDs
const int rewLEDDuty = 50; // value between 0 and 255
const int trackLEDDuty = 75; // value between 0 and 255

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
volatile bool lndLedOn = false;
uint32_t t_printNext = millis();

//----------INITILIZE OBJECTS----------
// AutoDriver
AutoDriver_Due board(pin_AD_CSP, pin_AD_Reset);
// Pixy
PixyI2C pixy(0x54);
// LCD
LCD5110 myGLCD(pin_Disp_CS, pin_Disp_RST, pin_Disp_DC, pin_Disp_MOSI, pin_Disp_SCK);

#pragma endregion 


#pragma region ---------CONSTRUCT CLASSES---------

//----------CLASS: PosDat----------
class PosDat
{
private:

	char objID;
	int nSamp;
	float posArr[6];
	uint32_t tsArr[6];
	int t_skip = 0;

public:

	float velNow = 0.0f;
	float posNow = 0.0f;
	float nLaps = 0;
	int sampCnt = 0;
	bool datReady = false;
	bool newData = false;

	// constructor
	PosDat(char id, int l)
	{
		this->objID = id;
		this->nSamp = l;
		for (int i = 0; i < l; i++) this->posArr[i] = 0.0f;
	}

	void UpdatePosVel(float posNew, uint32_t tsNew)
	{
		// Update itteration count
		this->sampCnt++;

		// Shift and add data
		for (int i = 0; i < this->nSamp - 1; i++)
		{
			this->posArr[i] = this->posArr[i + 1];
			this->tsArr[i] = this->tsArr[i + 1];
		}
		// Add new variables
		this->posArr[nSamp - 1] = posNew;
		this->tsArr[nSamp - 1] = tsNew;

		// Do not use early samples
		if (this->sampCnt < this->nSamp * 2)
		{
			this->posNow = posNew;
			this->velNow = 0.0f;
			newData = false;
			datReady = false;
		}
		else
		{
			newData = true;
			datReady = true;
			// COMPUTE TOTAL DISTANCE RAN

			// Get dif of current pos and last pos
			float posDiff = posNew - this->posArr[this->nSamp - 2];
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
			this->posNow = posNew + this->nLaps*(140 * PI);

			// COMPUTE VELOCITY

			float dist;
			float distSum = 0;
			int dt;
			float dtSum = 0;
			float vel;
			for (int i = 0; i < this->nSamp - 1; i++)
			{
				// compute distance
				dist = this->posArr[i + 1] - this->posArr[i];
				distSum += dist;
				// compute dt
				dt = this->tsArr[i + 1] - this->tsArr[i];
				dtSum += (float)dt;
			}
			// compute vel
			vel = distSum / (dtSum / 1000.0f);

			// ignore outlyer values unless too many frames discarted
			if (this->t_skip > 500 || abs(this->velNow - vel) < 150)
			{
				this->velNow = vel;
				this->t_skip = 0;
			}
			// add to skip time
			else this->t_skip += dt;
		}
	}

	float GetPos()
	{
		newData = false;
		return posNow;
	}

	float GetVel()
	{
		return velNow;
	}

	void ResetDat(float n_laps)
	{
		velNow = 0.0f;
		posNow = 0.0f;
		sampCnt = 0;
		datReady = false;
		newData = false;
		nLaps = n_laps;
	}

};
// Initialize and define
PosDat pos_ratVT('A', 4);
PosDat pos_robVT('C', 4);
PosDat pos_ratPixy('B', 6);

//----------CLASS: PID----------
class PID
{

private:
	// loop time
	uint32_t t_lastLoop = 0;
	float dtLoop;
	bool wasLoopRan = false;
	// dynamic vars
	float p_term;
	float i_term;
	float d_term;
	bool firstRun = true;

public:
	// dynamic vars
	float error = 0;
	float errorLast = 0;
	float integral = 0;
	float derivative = 0;
	float setPos;
	float velUpdate;
	float runSpeed = 0;
	// pid constants
	float dT;
	float kP;
	float kI;
	float kD;
	float setPoint = 0.81 * ((140 * PI) / (2 * PI));
	// state vars
	String modeNow = "Manual";
	String modeLast = "Manual";
	bool isHoldingForSetpointCrossing = false;
	// ekf
	uint32_t t_ekfStr = 0;
	bool ekfReady = false;
	bool ekfNew = false;
	// rew
	uint32_t t_holdForRewTill;
	bool isHoldingForRew = false;

	// constructor
	PID(const float kC, const float pC)
	{
		this->kP = 0.6 * kC; // proportional constant
		this->kI = 2 * kP / pC; // integral constant
		this->kD = kP*pC / 8; // derivative constant
	}

	void UpdatePID(float rat_pos, float rob_pos, float rat_vel, float rob_vel)
	{

		// Compute new error 
		setPos = rob_pos + setPoint;
		error = rat_pos - setPos;

		// Check for holding
		CheckRewHold();

		// Check if in auto mode
		if (modeNow == "Automatic")
		{

			// Check if rat stopped behind setpoint
			if (rat_vel < 1 && error < 0 && !isHoldingForSetpointCrossing)
			{
				// halt running
				HardStop("---pid: hard stop [UpdatePID]", fc_doTrackRat);
			}

			// Check for setpoint crossing
			CheckSetpointCrossing();

			// Update PID and speed
			if (
				!isHoldingForSetpointCrossing && // not waiting for setpoint pass
				ekfNew && // New EKF data 
				ekfReady // ekf is usable
				)
			{

				// Compute new integral
				if (do_includeTerm[0])
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
				if (do_includeTerm[1])
				{
					derivative = error - errorLast;
				}
				else derivative = 0;

				// Compute new terms
				p_term = kP*error;
				i_term = kI*dtLoop*integral;
				d_term = kD / dtLoop*derivative;
				//if (error < 0) d_term = d_term * 2.0f;

				// Compute updated vel
				velUpdate = p_term + i_term + d_term;

				// Get new run speed
				runSpeed = rob_vel + velUpdate;

				// Keep speed in range [0, maxSpeed]
				if (runSpeed > maxSpeed) runSpeed = maxSpeed;
				else if (runSpeed < 0) runSpeed = 0;

				errorLast = error;
				wasLoopRan = true;
				ekfNew = false;

				// Update run speed
				UpdateSpeed();
			}

		}

	}

	void UpdateSpeed()
	{
		RunMotor('f', runSpeed);
		if (firstRun)
		{
			PrintPID("---pid: first run");
			firstRun = false;
		}
	}

	void HardStop(String called_from, bool track_rat)
	{
		// Normal hard stop
		if (track_rat)
		{
			board.hardStop();
		}
		// Put motor in high impedance
		else
		{
			board.hardHiZ();
		}
		PrintPID(called_from);
		ResetPID();
	}

	void ResetPID()
	{
		integral = 0;
		t_lastLoop = millis();
		isHoldingForSetpointCrossing = true;
	}

	void Manual(String called_from)
	{
		ResetPID();
		modeLast = modeNow;
		modeNow = "Manual";
		PrintPID(called_from);
	}

	void Automatic(String called_from, String last_mode = "Automatic")
	{
		ResetPID();
		if (last_mode == "Automatic")
		{
			modeLast = modeNow;
			modeNow = "Automatic";
			PrintPID(called_from);
		}
	}

	void SetLoopTime(uint32_t now_ms)
	{
		ekfNew = true;
		if (wasLoopRan)
		{
			dtLoop = (float)(now_ms - t_lastLoop) / 1000.0f;
			wasLoopRan = false;
			t_lastLoop = now_ms;
		}
	}

	void SetHoldTim(uint32_t dt)
	{
		if (modeNow == "Automatic")
		{
			t_holdForRewTill = millis() + dt;
			isHoldingForRew = true;
			Manual("---pid: manual [SetHoldTim]");
		}
	}

	void CheckRewHold()
	{
		if (isHoldingForRew && (millis() > t_holdForRewTill ||
			error > 50)) // rat moved 10 cm past setpoint
		{
			isHoldingForRew = false;
			Automatic("---pid: automatic [CheckRewHold]");
		}
	}

	void CheckSetpointCrossing()
	{
		// Check if rat has moved back in front of setpoint
		if (isHoldingForSetpointCrossing && (error > 0))
		{
			isHoldingForSetpointCrossing = false;
			PrintPID("---pid: crossed setpoint");
		}
	}

	void CheckEKF(uint32_t now_ms)
	{
		if (!ekfReady)
		{
			if ((now_ms - t_ekfStr) > 2000)
			{
				ekfReady = true;
			}
		}
	}

	void ResetEKF()
	{
		ekfReady = false;
		t_ekfStr = millis();
		PrintPID("---pid: reset ekf");
	}

	void PrintPID(String msg)
	{
		if (doPrint_pid)
		{
			char s[50];
			String str;
			float t = (float)(millis() / 1000.0f);
			sprintf(s, " (%0.2f sec)\n", t);
			str = msg + s;
			StoreStr(str);
		}
	}

	void RunCalibration()
	{
		float PcCount = 0;  // oscillation count
		uint32_t t_Pc[2] = { 0, 0 }; // oscillation period sum
		uint32_t PcSum = 0; // oscillation period sum
		float PcAvg;
		float PcNow;
		float errCount = 0;
		float errSum = 0;
		float errAvg;
		float errMax = 0;
		float errMin = 0;
		if (errorLast > 0 && error < 0)
		{
			t_Pc[0] = t_Pc[1];
			t_Pc[1] = millis() / 1000;
			if (t_Pc[0] > 0 && t_Pc[1] > 0)
			{
				PcCount++;
				PcNow = t_Pc[1] - t_Pc[0];
				PcSum += PcNow;
				PcAvg = (float)(PcSum) / PcCount;
			}
		}
		errCount++;
		errSum += abs(error);
		errAvg = errSum / errCount;
		errMax = max(errMax, error);
		errMin = max(errMin, error);

		// If robot runs over rat
		if (error < setPoint*-1);
	}

};
// Initialize and define
PID pid(kC, pC);

//----------CLASS: Fuser----------
class Fuser : public TinyEKF
{

public:

	Fuser()
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
// TinyEKF
Fuser ekf;


//----------CLASS: union----------
union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (int16) 2 byte
	uint32_t i32; // (int32) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
} u;

#pragma endregion 


//---------SETUP---------
void setup() {

	delayMicroseconds(100);
	//while (!SerialUSB);
	SerialUSB.begin(0);
	DebugState("SETUP");

	// XBee
	Serial1.begin(57600);

	// Set button pins enable internal pullup
	for (int i = 0; i <= 3; i++) {
		pinMode(pin_Btn[i], INPUT_PULLUP);
	}

	// Set output/power pins
	pinMode(pin_Rel_1, OUTPUT);
	pinMode(pin_Rel_2, OUTPUT);
	digitalWrite(pin_Rel_1, LOW);
	digitalWrite(pin_Rel_2, LOW);
	pinMode(pin_Disp_Power, OUTPUT);
	digitalWrite(pin_Disp_Power, HIGH);

	// Define external interrupt
	attachInterrupt(digitalPinToInterrupt(pin_Btn[0]), Interupt_Btn1, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_Btn[1]), Interupt_Btn2, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_Btn[2]), Interupt_Btn3, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_IR_Rt), Interupt_Halt, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_IR_Lft), Interupt_Halt, FALLING);

	// Initialize AutoDriver

	// Configure SPI
	board.SPIConfig();
	delayMicroseconds(100);
	// Reset each axis
	board.resetDev();
	delayMicroseconds(100);
	// Configure each axis
	dSPINConfig_board();
	delayMicroseconds(100);
	// Get the status to clear the UVLO Flag
	board.getStatus();

	// Initailize Pixy
	pixy.init();
	Wire.begin();

	// Initialize LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);

	// Initialize print queue
	for (int i = 0; i < printQueue_lng; i++)
	{
		printQueue[i] = " ";
	}

	// Initialize send msg matrix
	for (int i = 0; i < r2_lngR; i++)
	{
		for (int j = 0; j < r2_lngC; j++)
		{
			r2_queue[i][j] = 0;
		}
	}

	// Initilize packet history
	for (int i = 0; i < c2r_hLng; i++)
	{
		c2r_packHist[i] = 0;
		c2r_idHist[i] = ' ';
	}
	for (int i = 0; i < r2c_hLng; i++)
	{
		r2c_packHist[i] = 0;
		r2c_idHist[i] = ' ';
	}

	// Initialize last packet and repeat packet array
	for (int i = 0; i < c2r_idLng; i++)
	{
		c2r_packLast[i] = 0;
		c2r_packRepeat[i] = 0;
		c2r_cntRepeat[i] = 0;
	}
	for (int i = 0; i < r2c_idLng; i++)
	{
		r2c_packLast[i] = 0;
	}

	// Make sure motor is stopped and in high impedance
	board.hardHiZ();
	RunMotor('f', 0);

}


// ---------MAIN LOOP---------
void loop() {

	// Store loop time
	t_loopMain = millis();

#pragma region //--- FIRST PASS SETUP ---
	if (fc_isFirstPass)
	{
		// Blink to show setup done
		SetupBlink();

		// Make sure Xbee buffer empty
		while (Serial1.available() > 0) Serial1.read();

		fc_isFirstPass = false;
		DebugState("MAIN LOOP");

	}
#pragma endregion

#pragma region //--- PRINT DEBUG ---
	if (doPrint)
	{
		PrintStr();
	}
#pragma endregion

#pragma region //--- PARSE SERIAL ---
	msg_pass = false; // reset before next loop
	if (Serial1.available() > 0)
	{
		msg_pass = ParseSerial();
	}
#pragma endregion

#pragma region //--- SEND SERIAL DATA ---
	if (doSend)
	{
		SendData();
	}
#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (msg_id == 'S' && msg_pass)
	{
		// Set condition
		if (setupCmd[0] == 1)
		{
			fc_doTrackRat = true;
			DebugState("DO TRACKING");
		}
		else
		{
			fc_doTrackRat = false;
			DebugState("NO TRACKING");
		}
		// Set reward tone
		if (setupCmd[1] == 1)
		{
			fc_doRewTone = true;
			// Start white noise
			Store4_Ard('w');
			DebugState("DO TONE");
		}
		else
		{
			fc_doRewTone = false;
			// Stop white noise
			Store4_Ard('o');
			DebugState("NO TONE");
		}
	}
#pragma endregion

#pragma region //--- (Q) DO QUIT ---
	if (msg_id == 'Q' && msg_pass)
	{
		fc_doQuit = true;
		uint32_t t_quitCmd = millis() + 5000;
		// Tell ard to quit
		Store4_Ard('q');
		DebugState("DO QUIT");
		// Stop PID
		pid.Manual("---pid: manual [Msg Q]");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (fc_doQuit && millis() > t_quitCmd && !doSend)
	{
		// Retell ard to quit
		Store4_Ard('q');
		DebugState("QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (msg_id == 'M' && msg_pass)
	{
		fc_doMove = true;
		DebugState("DO MOVE");
	}
	// Perform movement
	if (fc_doMove)
	{
		// Check if robot is ready to be moved
		if (!fc_isMoveTargSet)
		{
			bool is_targ_set = CompTarg(movePos, "MoveTo");
			if (is_targ_set)
			{
				fc_isMoveTargSet = true;
				DebugState("MOVE STARTED");
			}
		}
		// Check if robot is ready to be stopped
		if (fc_isMoveTargSet)
		{
			bool is_targ_reached = DoDecelerate(40, "MoveTo");
			if (is_targ_reached)
			{
				fc_doMove = false;
				fc_isMoveTargSet = false;
				// Tell CS movement is done
				Store4_CS('D', c2r_packLast[CharInd('M', "c2r")]);
				DebugState("MOVE FINISHED");
			}
		}
	}
#pragma endregion

#pragma region //--- (R) DO REWARD ---
	if (msg_id == 'R' && msg_pass)
	{
		if (rewPos == 0)
		{
			fc_doRew = true;
			DebugState("REWARD NOW");
		}
		else
		{
			fc_doCue = true;
			DebugState("REWARD CUE");
		}
	}

	// Reward now
	if (fc_doRew)
	{
		StartRew();
		fc_doRew = false;
		DebugState("REWARDIND...");
	}

	// Cue reward
	if (fc_doCue)
	{
		// Compute reward target pos
		if (!fc_isCueTargSet)
		{
			bool is_targ_set = CompTarg(rewPos, "Cue");
			if (is_targ_set)
			{
				fc_isCueTargSet = true;
				DebugState("CUEING REWARD...");
			}
		}
		// Check if robot is ready to be stopped
		if (fc_isCueTargSet)
		{
			bool is_targ_reached = DoDecelerate(30, "Cue");
			if (is_targ_reached)
			{
				fc_doCue = false;
				fc_isCueTargSet = false;
				// Trigger reward
				StartRew();
				DebugState("REWARDIND...");
			}
		}
	}

	// End ongoing reward
	if (fc_isRewarding)
	{
		EndRew();
		if (!fc_isRewarding)
		{
			DebugState("REWARD FINISHED");
		}
	}

#pragma endregion

#pragma region //--- (H) HALT ROBOT STATUS ---
	if (msg_id == 'H' && msg_pass)
	{
		if (fc_doHalt)
		{
			DebugState("HALT STARTED");
			// Stop pid and set to manual
			pid.HardStop("---pid: hard stop [Msg H]", fc_doTrackRat);
			pid.Manual("---pid: manual [Msg H]");
			fc_isHalted = true;
			fc_doHalt = false;
		}
		else
		{
			DebugState("HALT FINISHED");
			// Set pid back to auto if was in auto before
			pid.Automatic("---pid: automatic [Msg H]", pid.modeLast);
			fc_isHalted = false;
		}
	}
#pragma endregion

#pragma region //--- (B) BULLDOZE RAT STATUS ---
	if (msg_id == 'B' && msg_pass)
	{
		if (bullSec > 0)
		{
			DebugState("DO BULLDOZE");
		}
		else
		{
			DebugState("NO BULLDOZE");
		}
	}
#pragma endregion

#pragma region //--- (I) START/STOP PID ---
	if (msg_id == 'I' && msg_pass)
	{
		if (fc_doRunPID)
		{
			DebugState("PID STARTED");
			// Set rat ahead of robot
			SetRatAhead();
			// Blink lcd display
			RatInBlink();
			// Reset EKF
			pid.ResetEKF();
			// Start PID
			pid.Automatic("---pid: automatic [Msg I]");
		}
		else
		{
			DebugState("PID STOPPED");
			// Stop PID
			pid.Manual("---pid: manual [Msg I]");
		}
	}
#pragma endregion

#pragma region //--- (C) GET STREAM STATUS ---
	if (msg_id == 'C' && msg_pass)
	{
		fc_doStreamCheck = true;
	}

	// Check for streaming
	if (fc_doStreamCheck && fc_isStreaming)
	{
		Store4_CS('D', c2r_packLast[CharInd('C', "c2r")]);
		fc_doStreamCheck = false;
		DebugState("STREAMING CONFIRMED");
	}
#pragma endregion

#pragma region //--- (Y) DONE RECIEVED ---
	if (msg_id == 'Y' && msg_pass)
	{
		fc_doCheckDoneRcvd = false;
		DebugState("CS RESIEVED 'DONE'");
	}
	// Request done recieved confirmation
	if (fc_doCheckDoneRcvd && millis() > t_resendDone)
	{
		// Resend done confirmation
		Store4_CS('D', r2c_packLast[CharInd('D', "r2c")]);
		// Send again after 1 sec
		t_resendDone = millis() + 1000;
		DebugState("RESENT 'D'");
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (msg_id == 'P' && msg_pass)
	{
		// Update vt pos data
		if (vtEnt == 0)
		{
			pos_ratVT.UpdatePosVel(vtCM[vtEnt], vtTS[vtEnt]);
		}
		else
		{
			pos_robVT.UpdatePosVel(vtCM[vtEnt], vtTS[vtEnt]);
		}
	}
#pragma endregion

#pragma region //--- RUN TRACKING ---
	// UPDATE PIXY
	UpdatePixyPos();

	// UPDATE EKF
	UpdateEKF();

	// UPDATE PID AND SPEED
	if (fc_doTrackRat)
	{
		pid.UpdatePID(ekf_ratPos, ekf_robPos, ekf_ratVel, ekf_robVel);
	}
#pragma endregion

}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{
	byte buff;
	char head[2];
	char foot;
	bool pass;
	bool loop;
	msg_id == ' ';

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(c2r_head[0]);
	if (buff == 255)
	{
		return pass = false;
	}

	// get header
	u.f = 0.0f;
	u.b[0] = buff;
	u.b[1] = WaitBuffRead(0);
	head[0] = u.c[0];
	head[1] = u.c[1];

	// get packet num
	u.f = 0.0f;
	u.b[0] = WaitBuffRead(0);
	u.b[1] = WaitBuffRead(0);
	packNow = u.i16[0];
	// get id
	u.f = 0.0f;
	u.b[0] = WaitBuffRead(0);
	msg_id = u.c[0];

	// Check for second header
	if (head[1] != c2r_head[1])
	{
		// mesage will be dumped
		return pass = false;
	}

	// Get setup data
	if (msg_id == 'S')
	{

		// Get session comand
		setupCmd[0] = WaitBuffRead(0);

		// Get tone condition
		setupCmd[1] = WaitBuffRead(0);

	}

	// Get MoveTo data
	if (msg_id == 'M')
	{

		// Reset buffer
		u.f = 0.0f;

		// Get move posm
		u.b[0] = WaitBuffRead(0);
		u.b[1] = WaitBuffRead(0);
		u.b[2] = WaitBuffRead(0);
		u.b[3] = WaitBuffRead(0);
		movePos = u.f;

	}

	// Get Reward data
	if (msg_id == 'R')
	{
		// Reset buffer
		u.f = 0.0f;

		// Get move posm
		u.b[0] = WaitBuffRead(0);
		u.b[1] = WaitBuffRead(0);
		u.b[2] = WaitBuffRead(0);
		u.b[3] = WaitBuffRead(0);
		rewPos = u.f;
	}

	// Get halt robot data
	if (msg_id == 'H')
	{
		// Get halt bool
		byte do_halt = WaitBuffRead(0);
		fc_doHalt = do_halt == 1 ? true : false;
	}

	// Get bulldoze rat data
	if (msg_id == 'B')
	{
		// Get delay in sec
		byte bullSec = WaitBuffRead(0);
	}

	// Get start/end pid data
	if (msg_id == 'I')
	{
		// Get session comand
		byte run_pid = WaitBuffRead(0);
		fc_doRunPID = run_pid == 1 ? true : false;
	}

	// Get VT data
	else if (msg_id == 'P')
	{
		// Get Ent
		vtEnt = WaitBuffRead(0);
		// Get TS
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = WaitBuffRead(0);
		}
		vtTS[vtEnt] = u.i32;
		// Get pos cm
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = WaitBuffRead(0);
		}
		vtCM[vtEnt] = u.f;
	}

	// Check for footer
	u.f = 0.0f;
	u.b[0] = WaitBuffRead(0);
	foot = u.c[0];

	// Footer missing
	if (foot != c2r_foot) {
		// mesage will be dumped
		return pass = false;
	}
	// Process complete mesage
	else
	{
		// Update recive time
		t_rsvdLast = t_rsvd;
		t_rsvd = millis();

		// CHECK FOR STREAMING STARTED
		if (msg_id == 'P' && !fc_isStreaming)
		{
			// Signal streaming started
			fc_isStreaming = true;

			// Get sync time
			t_sync = millis();
			// send sync time cmd to ard
			Store4_Ard('t');
		}

		// Process packet number
		pass = CheckPack(msg_id, packNow);

		// Return final status
		return pass;

	}

}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(byte match)
{
	// Local vars
	static uint32_t time_out = 100;
	uint32_t t_out = millis() + time_out;
	byte buff;
	bool pass = false;

	// Check for overflow
	if (Serial1.available() < SERIAL_BUFFER_SIZE - 1)
	{

		// Wait for at least 1 byte
		while (Serial1.available() < 1 &&
			millis() < t_out);

		// Store next byte
		if (match == 0)
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
			} while (buff != match &&
				millis() < t_out &&
				Serial1.available() < SERIAL_BUFFER_SIZE - 1);
			// check match was found
			if (buff == match)
			{
				pass = true;
			}
		}
	}

	// Failed
	if (!pass)
	{

		// Buffer flooded
		if (Serial1.available() >= SERIAL_BUFFER_SIZE - 1)
		{
			// DUMP IT ALL
			while (Serial1.available() > 0)
			{
				Serial1.read();
			}
			cnt_overflowEvt++;
		}

		// Timed out
		if (millis() > t_out)
		{
			cnt_timeoutEvt++;
		}
		
		// Set buff to 255 ((byte)-1) if !pass
		buff = 255;
	}

	// Store time
	t_loopRead = millis() - (t_out - time_out);
	// Return buffer
	return buff;
}

// CHECK AND PROCESS RSVD PACKET NUMBER
bool CheckPack(char id, uint16_t pack)
{
	// Local vars
	bool pass;
	int pack_diff;
	int dropped_packs;
	bool use_old_pack;
	int id_ind;

	// SEND AND PRINT PACKET CONFIRMATON
	if (id != 'P')
	{
		// Print revieved pack details
		DebugRsvd(id, pack);

		// Send back packet confirmation
		if (id != 'Y')
		{
			// Send revieved pack
			Store4_CS(id, pack);
		}
	}

	// UPDATE PACKET HISTORY
	c2r_packLast[CharInd(id, "c2r")] = pack;

	// Shift packet history
	for (int i = 0; i < c2r_hLng - 1; i++)
	{
		c2r_packHist[i] = c2r_packHist[i + 1];
		c2r_idHist[i] = c2r_idHist[i + 1];
	}
	c2r_packHist[c2r_hLng - 1] = pack;
	c2r_idHist[c2r_hLng - 1] = id;

	// CHECK FOR MISSING PACKETS
	pack_diff = (int)(pack - packLast);

	// Is a new packet
	if (pack_diff > 0)
	{
		// Save packet and set to process
		packLast = pack;
		use_old_pack = true;

		// Check for dropped packet
		packTot += pack_diff;
		dropped_packs = pack_diff - 1;
		// print missed packs
		if (dropped_packs > 0)
		{
			cnt_droppedPacks += dropped_packs;
			DebugDropped(dropped_packs, cnt_droppedPacks, packTot); 
		}
	}
	// PROCESS RESENT PACKETS
	else
	{
		use_old_pack = true;

		// Loop through pack history
		for (int i = 0; i < c2r_hLng - 1; i++)
		{
			if (pack == c2r_packHist[i])
			{

				// Make sure packet history corresponds to current id
				if (c2r_idHist[i] == id)
				{

					// Get list index
					id_ind = CharInd(c2r_idHist[i], c2r_id);

					// Check caught pack is what we have stored
					if (c2r_packRepeat[id_ind] != pack)
					{
						// reset count
						c2r_cntRepeat[id_ind] = 1;
						// update stored packet
						c2r_packRepeat[id_ind] = c2r_packHist[i];
					}
					// itterate count
					else
					{
						c2r_cntRepeat[id_ind]++;
						cnt_packResend++;
					}

					if (c2r_cntRepeat[id_ind] > 0)
					{
						DebugResent(id, c2r_packRepeat[id_ind], c2r_cntRepeat[id_ind]);
						//Serial1.end();
						//Serial1.begin(57600);
					}

					// Is a resent packet so dont process
					use_old_pack = false;

				}
			}
		}
	}

	// Packet not found in hist so process
	return use_old_pack;
}

// STORE BYTE SERIAL DATA FOR CS
void Store4_CS(char id, uint16_t pack)
{
	// Local vars
	int queue_ind;

	// Find current queue ind
	for (int i = r2_lngR - 1; i >= 0; i--)
	{
		if (r2_queue[i][0] == 0);
		{
			queue_ind = i;
			break;
		}
	}

	// Store header
	u.f = 0.0f;
	u.c[0] = r2c_head;
	r2_queue[queue_ind][0] = u.b[0];
	// Store mesage id
	u.f = 0.0f;
	u.c[0] = id;
	r2_queue[queue_ind][1] = u.b[0];
	// Store packet number
	u.f = 0.0f;
	u.i16[0] = pack;
	r2_queue[queue_ind][2] = u.b[0];
	r2_queue[queue_ind][3] = u.b[1];
	// Store footer
	u.f = 0.0f;
	u.c[0] = r2c_foot;
	r2_queue[queue_ind][4] = u.b[0];

	// Set to send
	doSend = true;

	// Update last packet array
	r2c_packLast[CharInd(id, "r2c")] = pack;

	// Update packet history 
	for (int i = 0; i < r2c_hLng - 1; i++)
	{
		r2c_packHist[i] = r2c_packHist[i + 1];
		r2c_idHist[i] = r2c_idHist[i + 1];
	}
	r2c_packHist[r2c_hLng - 1] = pack;
	r2c_idHist[r2c_hLng - 1] = id;

	// Check for done id
	if (id == 'D')
	{
		// Set to check for done recieved
		fc_doCheckDoneRcvd = true;
		// Send done recieved request after 1 sec
		t_resendDone = millis() + 1000;
	}
}

// STORE BYTE SERIAL DATA FOR ARD
void Store4_Ard(char id)
{
	// itterate packet
	r2a_packCnt++;

	// Shift data back so ard msg is first in queue
	for (int i = 0; i < r2_lngR - 1; i++)
	{
		for (int j = 0; j < r2_lngC; j++)
		{
			r2_queue[i][j] = r2_queue[i + 1][j];
		}
	}

	// Store header
	u.f = 0.0f;
	u.c[0] = r2a_head;
	r2_queue[r2_lngR - 1][0] = u.b[0];
	// Store mesage id
	u.f = 0.0f;
	u.c[0] = id;
	r2_queue[r2_lngR - 1][1] = u.b[0];
	// Store packet number
	u.f = 0.0f;
	u.i16[0] = r2a_packCnt;
	r2_queue[r2_lngR - 1][2] = u.b[0];
	r2_queue[r2_lngR - 1][3] = u.b[1];
	// Store footer
	u.f = 0.0f;
	u.c[0] = r2a_foot;
	r2_queue[r2_lngR - 1][4] = u.b[0];

	// Set to send
	doSend = true;

}

// SEND SERIAL DATA
void SendData()
{
	// Local vars
	static byte msg[r2_lngC];
	bool do_send = false;
	int buff_tx;

	// Get total data in quiue;
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial.availableForWrite();

	// Send r2a sync time or tone immediately
	if ((r2_queue[r2_lngR - 1][1] == 'r' ||
		r2_queue[r2_lngR - 1][1] == 't'))
	{
		do_send = true;
	}
	// Avoid overlap between sent or rcvd events
	else if (millis() > t_sent + 10 &&
		millis() > t_rsvd + 5)
	{
		do_send = true;
	}

	// Send if conditions met
	if (do_send && Serial.availableForWrite() >= 10)
	{

		// Move to temp msg array
		for (int i = 0; i < r2_lngC; i++)
		{
			msg[i] = r2_queue[r2_lngR - 1][i];
		}

		// Send
		Serial1.write(msg, 5);

		// Update send time 
		t_sentLast = t_sent;
		t_sent = millis();

		//// Print
		if (doPrint_sent)
		{
			char str[50];
			char id;
			uint16_t pack;
			uint16_t ts;
			uint16_t dt;
			float t;

			// Convert back to native types
			// Store mesage id
			u.f = 0.0f;
			u.b[0] = r2_queue[r2_lngR - 1][1];
			id = u.c[0];
			// Store packet number
			u.f = 0.0f;
			u.b[0] = r2_queue[r2_lngR - 1][2];
			u.b[1] = r2_queue[r2_lngR - 1][3];
			pack = u.i16[0];

			// Print
			// set sync time
			if (t_sync == 0) ts = millis();
			else ts = t_sync;
			// compute times
			t = (float)((millis() - ts) / 1000.0f);
			dt = t_sent - t_sentLast;
			// print
			sprintf(str, "---Sent: [id:%c pack:%d tx_buff:%d (dt:%dms tot:%0.3fsec)]\n", id, pack, buff_tx, dt, t);
			StoreStr(str);
			Serial1.clearWriteError();
		}

		// Remove sent msg from front of queue
		for (int i = 0; i < r2_lngR - 1; i++)
		{
			for (int j = 0; j < r2_lngC; j++)
			{
				r2_queue[i + 1][j] = r2_queue[i][j];
			}
		}
		// Set first entry to all zeros
		for (int j = 0; j < r2_lngC; j++)
		{
			r2_queue[0][j] = 0;
		}

		// Set to not send again if all sent
		if (r2_queue[r2_lngR - 1][0] == 0)
		{
			doSend = false;
		}

	}
}

#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------

// RUN AUTODRIVER
void RunMotor(char dir, float speed)
{
	// Local vars
	float speed_steps = speed*cm2stp;

	if (!fc_isHalted)
	{
		if (dir == 'f')
		{
			board.run(FWD, speed_steps);
		}
		else if (dir == 'r')
		{
			board.run(REV, speed_steps);
		}
	}
}

// COMPUTE TARGET VARS
bool CompTarg(float move_targ, String move_type)
{

	// Check there is enough rob vt data
	if (pid.ekfReady)
	{
		char move_dir;
		float set_pos;
		float rob_tot_pos;
		float rob_abs_pos;
		float move_dif;

		// Use EKF estimate 
		rob_tot_pos = ekf_robPos;

		// Compute target distance and direction
		set_pos = move_targ - pid.setPoint;
		if (set_pos < 0)
		{
			set_pos = set_pos + (140 * PI);
		}

		// Robot current pos
		moveStart = rob_tot_pos;
		rob_abs_pos = moveStart - pos_robVT.nLaps*(140 * PI);

		// Diff and absolute distance
		move_dif = set_pos - rob_abs_pos;
		moveDist =
			min((140 * PI) - abs(move_dif), abs(move_dif));

		// initialize for later
		distLeft = moveDist;
		distLast = 140 * PI;

		// Run robot if not cueing
		if (move_type == "MoveTo")
		{
			// Set pid to manual
			pid.Manual("---pid: manual [CompTarg]");
			// Compute direction to move
			if ((move_dif > 0 && abs(move_dif) == moveDist) ||
				(move_dif < 0 && abs(move_dif) != moveDist))
			{
				moveDir = 'f';
			}
			else
			{
				moveDir = 'r';
			}
			// Start movement
			RunMotor(moveDir, moveSpeed);
		}

		// target set
		return true;
	}
	else return false;
}

// DECELERATE TO POS
bool DoDecelerate(float dec_pos, String move_type)
{

	static float base_speed = 0;
	static uint32_t t_updateNext = millis();
	static uint32_t t_updateDT = 50;
	static float min_speed = 2.5;
	float new_speed;

	// Compute remaining distance
	distLeft = moveDist -
		min((140 * PI) - abs(ekf_robPos - moveStart), abs(ekf_robPos - moveStart));

	// Check if rob is dec_pos cm from target
	if (distLeft <= dec_pos)
	{
		// Get base speed to decelerate from
		if (base_speed == 0)
		{
			// Use EKF estimate
			base_speed = abs(ekf_robVel);

			// If cueing stop PID now
			if (move_type == "Cue")
			{
				// Set pid to manual
				pid.Manual("---pid: manual [DoDecelerate]");
			}
		}

		// Update decel speed
		else if (millis() > t_updateNext)
			//else if ((distLast - distLeft) >= 5)
		{
			distLast = distLeft;
			// Compute new speed
			new_speed = (distLeft / dec_pos) * base_speed;
			// Maintain at min speed
			if (new_speed < min_speed) new_speed = min_speed;

			// Change speed and If cueing always run FWD
			if (move_type == "Cue")
			{
				RunMotor('f', new_speed);
			}
			else
			{
				RunMotor(moveDir, new_speed);
			}

			// Update loop time
			t_updateNext = millis() + t_updateDT;

			//// Print for csv cut-and-paist
			//char msg[50];
			//sprintf(msg, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d\n",
			//	new_speed, pos_robVT.GetVel(), ekf_robVel, pos_robVT.GetPos(), ekf_robPos, millis());
			//SerialUSB.print(msg);
		}

	}

	// Rat has reached +- 5 cm of target
	if (distLeft < 2.5 && distLeft > -2.5)
	{
		// Hard stop
		pid.HardStop("---pid: hard stop [DoDecelerate]", fc_doTrackRat);

		// Set pid back to auto if was in auto before
		pid.Automatic("---pid: automatic [DoDecelerate]", pid.modeLast);

		// Reset values
		base_speed = 0;
		fc_isMoveTargSet = false;
		return true;
	}
	else return false;
}

// PROCESS PIXY STREAM
void UpdatePixyPos() {

	uint16_t blocks = pixy.getBlocks();
	static float pxRel;
	static double pxAbs;
	static uint32_t pxTS;

	// Check for new data
	if (blocks)
	{
		pxTS = millis();

		// Get Y pos from last block and convert to CM
		double pixyPosY = pixy.blocks[blocks - 1].y;
		pxRel =
			pixyCoeff[0] * (pixyPosY * pixyPosY * pixyPosY * pixyPosY) +
			pixyCoeff[1] * (pixyPosY * pixyPosY * pixyPosY) +
			pixyCoeff[2] * (pixyPosY * pixyPosY) +
			pixyCoeff[3] * pixyPosY +
			pixyCoeff[4];
	}

	// Update pos with new rob vt data
	if (pos_robVT.newData)
	{
		// Compute position error
		pxAbs = pxRel - pixyTarg;
		pxAbs = pxAbs + pid.setPoint;
		pxAbs = pos_robVT.posNow + pxAbs;
		if (pxAbs > (140 * PI))
		{
			pxAbs = pxAbs - (140 * PI);
		}
		// Update pixy pos and vel
		pos_ratPixy.UpdatePosVel((float)pxAbs, pxTS);
	}

}

// UPDATE EKF
void UpdateEKF()
{
	// Check for new data
	if ((pos_ratVT.newData &&
		pos_ratPixy.newData &&
		pos_robVT.newData) ||
		(pos_robVT.newData && !fc_doRunPID))
	{

		// Check EKF progress
		pid.CheckEKF(millis());

		// Update PID loop time
		if (fc_doRunPID)
		{
			// Update pid next loop time
			pid.SetLoopTime(millis());
		}
		// Reset rat pos data
		else
		{
			pos_ratVT.ResetDat(0);
			pos_ratPixy.ResetDat(0);
		}

		//----------UPDATE EKF---------
		double z[M] = {
			pos_ratVT.GetPos(),
			pos_ratPixy.GetPos(),
			pos_robVT.GetPos(),
			pos_ratVT.GetVel(),
			pos_ratPixy.GetVel(),
			pos_robVT.GetVel(),
		};

		// Run EKF
		ekf.step(z);

		// Update error estimate
		ekf_ratPos = ekf.getX(0);
		ekf_robPos = ekf.getX(1);
		ekf_ratVel = ekf.getX(2);
		ekf_robVel = ekf.getX(3);

	}
}

// CHECK RAT POS AHEAD OF ROBOT AT START 
void SetRatAhead()
{
	// Check if rat pos < robot
	if (pos_ratVT.posNow < pos_robVT.posNow)
	{
		pos_ratVT.ResetDat(1);
		pos_ratPixy.ResetDat(1);
	}
}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// START REWARD
void StartRew()
{
	// Stop robot
	if (fc_doTrackRat)
	{
		// Halt robot 
		pid.HardStop("---pid: hard stop [StartRew]", fc_doTrackRat);
		// Set hold time
		pid.SetHoldTim(5000);
	}

	// Trigger reward tone on
	if (fc_doRewTone)
	{
		Store4_Ard('r');
		DebugState("SENT TONE COMMAND");
	}

	// Turn on reward LED
	analogWrite(pin_RewLED, rewLEDDuty);

	// Open solenoid
	digitalWrite(pin_Rel_1, HIGH);

	// Compute Times
	t_rewLEDOff = millis() + ledDir;
	t_rewSolClose = millis() + solDir;

	// indicate reward in progress
	fc_isRewarding = true;
}

// END REWARD
void EndRew()
{
	// Turn off reward LED
	if (millis() > t_rewLEDOff)
	{
		analogWrite(pin_RewLED, 0);
	}

	// Close solenoid
	if (millis() > t_rewSolClose)
	{
		digitalWrite(pin_Rel_1, LOW);
		// indicate reward finished
		fc_isRewarding = false;
	}
}

// OPEN/CLOSE SOLONOID
void OpenCloseSolonoid()
{
	// change state
	solState = !solState;
	// open/close solenoid
	digitalWrite(pin_Rel_1, solState);

	char str[50];
	sprintf(str, "Solenoid %s", digitalRead(pin_Rel_1) == HIGH ? "open" : "close");
	myGLCD.print(str, CENTER, 20);
	myGLCD.update();
	if (digitalRead(pin_Rel_1) == LOW) {
		myGLCD.clrScr();
		myGLCD.update();
	}
}

#pragma endregion


#pragma region --------DEBUGGING---------

void StoreStr(String str)
{
	int queue_ind;

	// Find current queue ind
	for (int i = printQueue_lng - 1; i >= 0; i--)
	{
		if (printQueue[i] == " ");
		{
			queue_ind = i;
			break;
		}
	}

	// Add to queue
	printQueue[queue_ind] = str;
	doPrint = true;
}

void PrintStr()
{

	// Local vars
	String msg;

	// Get current string
	msg = printQueue[printQueue_lng - 1];

	// Send

	SerialUSB.flush();
	SerialUSB.print(msg);

	// Remove sent msg from front of queue
	for (int i = 0; i < printQueue_lng - 1; i++)
	{
		printQueue[i + 1] = printQueue[i];
	}
	// Set first entry to " "
	printQueue[0] = " ";

	// Set to not print again if all sent
	if (printQueue[printQueue_lng - 1] == " ")
	{
		doPrint = false;
	}

}

void DebugDropped(int missed, int missed_total, int total)
{
	if (doPrint_dropped)
	{
		// Local vars
		char str[50];
		uint32_t ts;
		float t;

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);

		// Store
		sprintf(str, "!!dropped %d packs [total:%d/%d] (%0.2f sec)]!!\n", missed, missed_total, total, t);
		StoreStr(str);
	}
}

void DebugResent(char id, uint16_t pack, int total)
{
	if (doPrint_resent)
	{
		// Local vars
		char str[50];
		uint32_t ts;
		float t;

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);

		// Store
		sprintf(str, "!!resent id:%c pack:%d %d times (%0.2f sec)!!\n", id, pack, total, t);
		StoreStr(str);
	}
}

void DebugRsvd(char id, uint16_t pack)
{
	//// Print
	if (doPrint_rcvd)
	{
		char s1[50];
		char s2[50];
		String str;
		uint32_t ts;
		uint32_t dt;
		float t;
		int buff_rx;

		// Get total data in quiue;
		buff_rx = Serial1.available();

		// Print

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);
		dt = t_rsvd - t_rsvdLast;

		// Print specific pack contents
		if (id == 'S')
		{
			sprintf(s1, "---Rcvd: [id:%c d1:%d d2:%d pack:%d", id, setupCmd[0], setupCmd[1], pack);
		}
		else if (id == 'M')
		{
			sprintf(s1, "---Rcvd: [id:%c d1:%0.2f pack:%d", id, movePos, pack);
		}
		else if (id == 'R')
		{
			sprintf(s1, "---Rcvd: [id:%c d1:%0.2f pack:%d", id, rewPos, pack);
		}
		else if (id == 'H')
		{
			sprintf(s1, "---Rcvd: [id:%c d1:%d pack:%d", id, fc_doHalt, pack);
		}
		else if (id == 'B')
		{
			sprintf(s1, "---Rcvd: [id:%c d1:%d pack:%d", id, bullSec, pack);
		}
		else if (id == 'I')
		{
			sprintf(s1, "---Rcvd: [id:%c d1:%d pack:%d", id, fc_doRunPID, pack);
		}
		else sprintf(s1, "---Rcvd: [id:%c pack:%d", id, pack);

		// add other
		sprintf(s2, " rx_buff:%d (dt:%dms tot:%0.3fsec)]\n", buff_rx, dt, t);
		str = (String)s1 + (String)s2;
		StoreStr(str);

	}
}

void DebugState(String msg)
{
	if (doPrint_flow)
	{
		char s[50];
		String str;
		uint32_t ts;
		float t;

		// set sync time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;

		// print
		t = (float)((millis() - ts) / 1000.0f);
		sprintf(s, " (%0.3fsec)\n\n", t);
		str = "\n" + msg + s;
		StoreStr(str);
	}
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

void QuitSession()
{
	// Stop motor and PID
	pid.HardStop("---pid: hard stop [QuitSession]", fc_doTrackRat);
	pid.Manual("---pid: manual [QuitSession]");
	delayMicroseconds(100);
	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

void SetupBlink()
{
	int duty[2] = { 100, 0 };
	bool isOn = false;
	int del = 100;
	// Flash sequentially
	for (int i = 0; i < 8; i++)
	{
		analogWrite(pin_Disp_LED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin_TrackLED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin_RewLED, duty[(int)isOn]);
		delay(del);
		isOn = !isOn;
	}
	// Reset LEDs
	analogWrite(pin_Disp_LED, 0);
	analogWrite(pin_TrackLED, trackLEDDuty);
	analogWrite(pin_RewLED, 0);
}

void RatInBlink()
{
	int duty[2] = { 75, 0 };
	bool isOn = false;
	int del = 50;
	// Flash 
	for (int i = 0; i < 10; i++)
	{
		analogWrite(pin_Disp_LED, duty[(int)isOn]);
		delay(del);
		isOn = !isOn;
	}
	// Reset LED
	analogWrite(pin_Disp_LED, 0);
}

int CharInd(char id, String a_lab)
{

	int ind = -1;
	if (a_lab == "c2r")
	{
		for (int i = 0; i < c2r_idLng; i++)
		{
			if (id == c2r_id[i]) ind = i;
		}
	}
	else if (a_lab == "r2c")
	{
		for (int i = 0; i < r2c_idLng; i++)
		{
			if (id == r2c_id[i]) ind = i;
		}
	}
	else if (a_lab == "r2a")
	{
		for (int i = 0; i < r2a_idLng; i++)
		{
			if (id == r2a_id[i]) ind = i;
		}
	}
	return ind;

}

#pragma endregion


#pragma region ---------INTERUPTS---------

// Halt run on IR trigger
void Interupt_Halt() {
	pid.HardStop("---pid: hard stop [Interupt_Halt]", fc_doTrackRat);
}

// Open/close solonoid
void Interupt_Btn1() {

	// exit if < 250 ms has not passed
	if (t_debounce[0] > millis()) return;

	// Run open close function
	OpenCloseSolonoid();

	t_debounce[0] = millis() + 250;
}

// Trigger reward
void Interupt_Btn2() {

	// exit if < 250 ms has not passed
	if (t_debounce[1] > millis()) return;

	// Set to start reward function
	fc_doRew = true;

	t_debounce[1] = millis() + 1000;

}

// Turn on/off LCD LED
void Interupt_Btn3() {

	// exit if < 250 ms has not passed
	if (t_debounce[2] > millis()) return;

	if (!lndLedOn) {
		analogWrite(pin_Disp_LED, 50);
		lndLedOn = true;
	}
	else {
		analogWrite(pin_Disp_LED, 0);
		lndLedOn = false;
	}

	t_debounce[2] = millis() + 250;

}
#pragma endregion

