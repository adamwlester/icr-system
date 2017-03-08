
//-------FeederDue-------

#pragma region ---------VARIABLE DECLARATION---------


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

// Setup
bool doPrintPack = false;
bool doPrintFlow = true;
byte setupCmd[2];
bool firstPass = true;
bool doTrackRun = false;
bool doRewTone = false;
bool doStreamCheck = false;
bool doQuit = false;
uint32_t quitTim;

// Serial from CS
char c2r_head[2] = { '<', '_' };
char c2r_foot = '>';
char c2r_id[8] = {
	'S', // start session
	'Q', // quit session
	'M', // move to position
	'R', // dispense reward
	'H', // halt movement
	'P', // position data
	'C', // request stream status
	'Z', // request done resend
};
uint16_t pack_now;
uint16_t pack_last;
int missed_packs;
uint32_t pack_tot = 0;
uint16_t c2r_packHist[10];
uint16_t c2r_packLast[sizeof(c2r_id)];
char msg_id = ' ';
bool msg_pass = false;
bool streamStarted = false;

// Serial to CS
const char r2c_id[7] = {
	'S', // setup command
	'Q', // quit session
	'M', // move to position
	'R', // dispense reward
	'H', // halt movement
	'D', // execution done
	'C', // request stream status
};
uint16_t c2r_doneHist[10];

// Serial to other ard
char r2a_head = '[';
char r2a_foot = ']';
byte r2a_msg[3];
char r2a_idChar[3] = {
	't', // reward tone
	'w', // white noise on
	'o', // white noise off
};
byte r2a_id[3];

// Serial VT
uint16_t vtRec;
byte vtEnt;
float vtCM[2];
uint32_t vtTS[2];

// Pixy
const float pixyTarg = 15; // (cm)
const double pixyCoeff[5] = {
	0.000000064751850,
	-0.000029178819914,
	0.005627883140337,
	-0.690663759936117,
	38.949828090278942
};
float pixyPosRel;
float pixyCM;
uint32_t  pixyTS;

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 80; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)

// Kalman model measures
float estRatPos;
float estrob_pos;
float estRatVel;
float estRobVel;
bool posUpdate[3] = { false, false, false };
uint32_t t_ekfStr = 0;
uint32_t ekfSettleDel = 60000;

// PID Calibration
const bool do_simulateRat = true;
bool do_includeTerm[2] = { true, true };
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
const float loopTim = 40; // pid sample rate (ms)
const float Kc = 5; // critical gain
//const float Kc = 5 * (1 / 0.6); // critical gain
const float Pc = 3; // oscillation period

// PID controller
const float dT = (loopTim / 1000); // pid sample rate (sec)
const float Kp = 0.6 * Kc; // proportional constant
const float Ki = 2 * Kp*dT / Pc; // integral constant
const float Kd = Kp*Pc / (8 * dT); // derivative constant
uint32_t t_nextLoop = millis();
float runSpeed;
float dtLoop;
float setPos;
float velUpdate;
const float setPoint = 0.81 * ((140 * PI) / (2 * PI)); // (cm)                  
float error;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pTerm;
float iTerm;
float dTerm;
volatile bool haltRun = true;
volatile bool zeroIntegral = true;

// MoveTo 
float moveSpeed = maxSpeed;
float movePos;
float moveSum;
float moveDist;
char moveDir;
float moveStart;
float baseSpeed;
bool doMove = false;
bool isMoving = false;

// Reward
const uint32_t solDir = 2000; // (ms) on duration
volatile byte solState = HIGH;
volatile bool doRew = false;
volatile byte doingRew = false;
uint32_t t_rewStop = millis();
float t_rewStr;
float rewPos;
uint32_t t_holdTill = millis();
uint32_t t_runHold = 5000; //(ms)

// LEDs
const int rewLEDDuty = 50; // value between 0 and 255
const int trackLEDDuty = 75; // value between 0 and 255

// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
volatile byte lndLedOn = false;
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
	bool ready = false;

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

		// Check for lap crossing
		// skip till array filled
		if (this->sampCnt < this->nSamp * 2)
		{
			this->posNow = posNew;
			this->velNow = 0.0f;
		}
		else
		{
			if (!ready) ready = true;

			// Compute cumulative pos
			// get dif of current pos and last pos
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

			// Compute vel
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

			// ignore outlyer values unness too many frames discarted
			if (this->t_skip > 500 || abs(this->velNow - vel) < 150)
			{
				this->velNow = vel;
				this->t_skip = 0;
			}
			// add to skip time
			else this->t_skip += dt;
		}
	}
};
PosDat vtRatHist('A', 4);
PosDat pixyRatHist('B', 6);
PosDat vtRobHist('C', 4);

//----------CLASS: Fuser----------
class Fuser : public TinyEKF
{

public:

	Fuser()
	{
		// We approximate the process noise using a small constant
		this->setQ(0, 0, .0001);
		this->setQ(1, 1, .0001);
		this->setQ(2, 2, .0001);
		this->setQ(3, 3, .0001);

		// Same for measurement noise
		this->setR(0, 0, .01);
		this->setR(1, 1, .01);
		this->setR(2, 2, .01);
		this->setR(3, 3, .01);
		this->setR(4, 4, .01);
		this->setR(5, 5, .01);
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

	//while (!SerialUSB);
	PrintState("SETUP");

	// XBee
	Serial1.begin(115490);

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

	// Set LEDs
	analogWrite(pin_RewLED, 0);
	analogWrite(pin_TrackLED, trackLEDDuty);

	// Get rob to ard byte serial vars
	u.f = 0.0f;
	u.c[0] = r2a_head;
	u.c[1] = r2a_foot;
	r2a_msg[0] = u.b[0];
	r2a_msg[1] = 0;
	r2a_msg[2] = u.b[1];
	u.f = 0.0f;
	for (int i = 0; i < 3; i++)
	{
		u.c[0] = r2a_idChar[i];
		r2a_id[i] = u.b[0];
	}

	// Initilize packet history
	for (int i = 0; i < sizeof(c2r_packHist) / 2; i++)
	{
		c2r_packHist[i] = 0;
	}

	// Initialize done history
	for (int i = 0; i < sizeof(c2r_doneHist) / 2; i++)
	{
		c2r_doneHist[i] = 0;
	}

	// Make sure motor is stopped and in high impedance
	board.hardHiZ();
	//board.run(FWD, 0);

	// PID calibration setup
	if (do_simulateRat)
	{
		posUpdate[2] = true;
	}

}


// ---------MAIN LOOP---------
void loop() {

	//while (true)
	//{
	//	Send2_Ard(r2a_id[0]);
	//	delay(1000);
	//	Send2_Ard(r2a_id[1]);
	//	delay(1000);
	//	Send2_Ard(r2a_id[2]);
	//	delay(1000);
	//}

	if (doRew)
	{
		StartRew();
		doRew = false;
	}
	else
	{
		// Run on first pass
		if (firstPass)
		{
			// Make sure Xbee buffer empty
			while (Serial1.available() > 0) Serial1.read();

			// Start white noise
			Send2_Ard(r2a_id[1]);

			PrintState("MAIN LOOP");
			firstPass = false;
		}

		// Check XBee for new input
		msg_pass = false; // reset before next loop
		if (Serial1.available() > 0)
		{
			msg_pass = ParseSerial();
		}

		// Check for (S) setup mesage
		if (msg_id == c2r_id[0] && msg_pass)
		{
			// Run tracking
			if (setupCmd[0] == 1)
			{
				doTrackRun = true;
				PrintState("DO TRACKING");
			}
			else
			{
				// put motor in high impedance
				board.hardHiZ();
				PrintState("NO TRACKING");
			}
			// Use reward tone
			if (setupCmd[1] == 1)
			{
				doRewTone = true;
				PrintState("DO TONE");
				// Start white noise
				Send2_Ard(r2a_id[1]);
			}
			else PrintState("NO TONE");

		}

		// Check for (Q) quit session mesage
		else if (msg_id == c2r_id[1] && msg_pass)
		{
			doQuit = true;
			PrintState("QUITING");
			uint32_t quitTim = millis() + 1000;
		}

		if (doQuit && millis() > quitTim)
		{
			QuitSession();
		}

		// Check for (M) move command
		else if (msg_id == c2r_id[2] && msg_pass)
		{
			doMove = true;
			PrintState("DO MOVE");
		}

		// Check for (R) reward command
		else if (msg_id == c2r_id[3] && msg_pass)
		{
			if (rewPos == 0)
			{
				PrintState("REWARD NOW");
				doRew = true;
			}
			else 
			{
				PrintState("REWARD CUE");
				//movePos = rewPos;
			}
		}

		// Check for (C) request stream status
		else if (msg_id == c2r_id[6] && msg_pass)
		{
			doStreamCheck = true;
		}

		// Check for (Z) request done resend
		else if (msg_id == c2r_id[7] && msg_pass)
		{
			// Resend last done pack
			Send2_CS(r2c_id[5], c2r_doneHist[sizeof(c2r_doneHist) / 2 - 1]);
		}

		// Check for (P) new vt data
		else if (msg_id == c2r_id[5] && msg_pass)
		{
			// Signal streaming started
			if (!streamStarted)
			{
				PrintState("STREAMING STARTED");
				streamStarted = true;
			}
			posUpdate[vtEnt] = true;
		}

		// Check if robot is ready to be moved
		if (doMove && !isMoving)
		{
			isMoving = DoMove();
			if (isMoving)
			{
				PrintState("MOVE STARTED");
			}
		}

		// Check if robot is ready to be stopped
		if (isMoving)
		{
			bool move_done = CheckMove();
			if (move_done)
			{
				PrintState("MOVE FINISHED");
				// Tell CS movement is done
				Send2_CS(r2c_id[5], c2r_packLast[2]);
				doMove = false;
				isMoving = false;
			}
		}

		// Check for streaming
		if (doStreamCheck && streamStarted)
		{
			PrintState("STREAMING CONFIRMED");
			Send2_CS(r2c_id[5], c2r_packLast[6]);
			doStreamCheck = false;
		}

		// Check for new Pixy data
		CheckPixy();

		// Wait for new pos data input and update pos
		if ((posUpdate[0] && posUpdate[2]) || posUpdate[1])
		{
			// Update pos values
			UpdatePos();
		}

		
		if (doTrackRun && !isMoving)
		{
			//board.run(FWD, 100);
			//doTrackRun = false;
		}

		// Wait to start running
		if (doTrackRun && millis() > t_holdTill)
		{

			// Update EKF
			UpdateEKF();

			// Update speed
			// Check if ekf has reached stable level
			if ((millis() - t_ekfStr) > ekfSettleDel)
			{
				// Check if sample interval has ellapsed
				if (millis() >= t_nextLoop)
				{
					UpdateSpeed();
				}
			}

		}

		// End ongoing reward
		if (doingRew) {
			if (t_rewStop < millis())
				EndRew();
		}
	}
}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{
	char head[2];
	char foot;
	bool pass;
	int pack_diff;
	bool use_old_pack;
	msg_id == ' ';

	// Dump data till msg header byte is reached
	while (Serial1.available() > 0)
	{

		while (Serial1.peek() != c2r_head[0])
		{
			Serial1.read(); // dump
		}

		// Wait for complete packet
		while (Serial1.available() < 6);

		// get header
		u.f = 0.0f;
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		head[0] = u.c[0];
		head[1] = u.c[1];
		// get packet num
		u.f = 0.0f;
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		pack_last = pack_now;
		pack_now = u.i16[0];
		// get id
		u.f = 0.0f;
		u.b[0] = Serial1.read();
		msg_id = u.c[0];

		// Check for second header
		if (head[1] != c2r_head[1])
		{
			// mesage will be dumped
			return pass = false;
		}
		else break;
	}

	// Get setup data
	if (msg_id == c2r_id[0])
	{
		// Wait for complete packet
		while (Serial1.available() < 3);

		// Get session comand
		setupCmd[0] = Serial1.read();

		// Get tone condition
		setupCmd[1] = Serial1.read();

	}

	// Get MoveTo data
	if (msg_id == c2r_id[2])
	{
		// Wait for complete packet
		while (Serial1.available() < 5);

		// Reset buffer
		u.f = 0.0f;

		// Get move posm
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		u.b[2] = Serial1.read();
		u.b[3] = Serial1.read();
		movePos = u.f;

	}

	// Get Reward data
	if (msg_id == c2r_id[3])
	{
		// Wait for complete packet
		while (Serial1.available() < 5);

		// Reset buffer
		u.f = 0.0f;

		// Get move posm
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		u.b[2] = Serial1.read();
		u.b[3] = Serial1.read();
		rewPos = u.f;

	}

	// Get VT data
	else if (msg_id == c2r_id[5])
	{

		// Wait for complete packet
		while (Serial1.available() < 12);

		// Get record number
		u.f = 0.0f;
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		vtRec = u.i16[0];
		// Get Ent
		vtEnt = Serial1.read();
		// Get TS
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = Serial1.read();
		}
		vtTS[vtEnt] = u.i32;
		// Get pos cm
		u.f = 0.0f;
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = Serial1.read();
		}
		vtCM[vtEnt] = u.f;
	}

	// Check for footer
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	foot = u.c[0];

	// Footer missing
	if (foot != c2r_foot) {
		// mesage will be dumped
		return pass = false;
	}
	else
	{
		// Save packet
		for (int i = 0; i < sizeof(r2c_id); i++) {
			if (r2c_id[i] == msg_id) {
				c2r_packLast[i] = pack_now;
				break;
			}
		}

		// Save packet number Send back recieved id for all but pos and done resend data
		if (msg_id != c2r_id[5] && msg_id != c2r_id[7])
		{

			// Send
			Send2_CS(msg_id, pack_now);

			// Shift packet history
			for (int i = 0; i < sizeof(c2r_packHist) / 2 - 1; i++)
			{
				c2r_packHist[i] = c2r_packHist[i + 1];
			}
			c2r_packHist[sizeof(c2r_packHist) / 2 - 1] = pack_now;
		}

		// Check for missing packets
		pack_diff = pack_now - pack_last;
		// Count sent packets
		if (pack_diff > 0)
		{
			pack_tot += pack_diff;
			missed_packs += pack_diff - 1;
			// print missed packs
			if (pack_diff - 1 > 0)
			{
				PrintMissedPack(msg_id, pack_now, pack_diff - 1);
			}
			return pass = true;
		}

		// Check if resent packet was already recieved
		else
		{
			use_old_pack = true;
			for (int i = 0; i < sizeof(c2r_packHist) / 2 - 1; i++)
			{
				if (pack_now == c2r_packHist[i])
				{
					use_old_pack = false;
				}
			}
			// If old packet not used then use
			if (use_old_pack)
			{
				return pass = true;
			}
			else return pass = false;

		}
	}

}

// PRINT RECIEVED PACKET
void PrintMissedPack(char id, uint16_t pack, int missed)
{
	if (doPrintPack)
	{
		char str[50];
		float t = (float)(millis() / 1000.0f);
		sprintf(str, "__missed:%d pack:%d (%0.2f sec)]\n", missed, pack, t);
		SerialUSB.print(str);
	}
}

// SEND SERIAL TO CS
void Send2_CS(char id, uint16_t pack)
{
	// Initialize
	byte msg[3];

	//// Print
	if (doPrintPack)
	{
		char str[50];
		float t = (float)(millis() / 1000.0f);
		sprintf(str, " pack:%d (%0.2f sec)]\n", pack, t);
		SerialUSB.print("Sent: ");
		SerialUSB.print("[id:");
		SerialUSB.print(id);
		SerialUSB.print(str);
	}

	// Store mesage id
	u.f = 0.0f;
	u.c[0] = id;
	msg[0] = u.b[0];
	// Store packet number
	u.f = 0.0f;
	u.i16[0] = pack;
	msg[1] = u.b[0];
	msg[2] = u.b[1];

	// Send
	Serial1.write(msg, 3);

	// Update done cmd history
	if (id == r2c_id[5])
	{
		for (int i = 0; i < (sizeof(c2r_doneHist) / 2) - 1; i++)
		{
			c2r_doneHist[i] = c2r_doneHist[i + 1];
		}
		c2r_doneHist[(sizeof(c2r_doneHist) / 2) - 1] = pack;
	}
}

// SEND SERIAL TO ARD
void Send2_Ard(byte id)
{
	r2a_msg[1] = id;
	Serial1.write(r2a_msg, 3);
}


#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------
// DO MOVE
bool DoMove() {

	// Check there is enough rob vt data
	if (vtRobHist.ready)
	{
		char move_dir;
		float set_pos;
		float rob_pos;
		float move_dif;

		// Compute target distance and direction
		set_pos = movePos - setPoint;
		if (set_pos < 0)
		{
			set_pos = set_pos + (140 * PI);
		}

		// Robot current pos
		moveStart = vtRobHist.posNow;
		rob_pos = moveStart - vtRobHist.nLaps*(140 * PI);

		// Diff and absolute distance
		move_dif = set_pos - rob_pos;
		moveDist =
			min((140 * PI) - abs(move_dif), abs(move_dif));

		// Compute direction to move
		if ((move_dif > 0 && abs(move_dif) == moveDist) ||
			(move_dif < 0 && abs(move_dif) != moveDist))
		{
			moveDir = 'f';
			board.run(FWD, moveSpeed * cm2stp);
		}
		else
		{
			moveDir = 'r';
			board.run(REV, moveSpeed * cm2stp);
		}
		baseSpeed = moveSpeed;
		return true;
	}
	else
	{
		return false;
	}

}

// CHECK MOVEMENT
bool CheckMove() {

	float decPos = 30;
	float dist_gone;
	float dist_last;
	float dist_left;
	float new_speed;

	// Add total distance
	dist_last = dist_gone;
	dist_gone =
		min((140 * PI) - abs(vtRobHist.posNow - moveStart), abs(vtRobHist.posNow - moveStart));

	// Get remaining distance
	dist_left = moveDist - dist_gone;

	// Only run for new data
	if (dist_gone != dist_last)
	{
		// Check if rat is decPos cm from target
		if (dist_left <= decPos && dist_left > 0.1)
		{
			// Get base speed to decelerate from
			if (baseSpeed == 1000)
			{
				if ((millis() - t_ekfStr) > ekfSettleDel)
				{
					baseSpeed = abs(estRobVel);
				}
				else baseSpeed = abs(vtRobHist.velNow);				
			}
			// Update decel speed
			else
			{
				new_speed = (dist_left / decPos) * baseSpeed;
				if (new_speed < 2.5) new_speed = 2.5;
				// Change speed
				if (moveDir == 'f') board.run(FWD, new_speed);
				else if (moveDir == 'r') board.run(REV, new_speed);
				// SerialUSB.println(new_speed);
			}
		}
	}
	// Rat has reached target
	if (dist_left <= 0)
	{
		// Change speed
		if (moveDir == 'f') board.run(FWD, 0);
		else if (moveDir == 'r') board.run(REV, 0);
		baseSpeed == 1000;
		return true;
	}
	return false;
}

// PROCESS PIXY STREAM
void CheckPixy() {

	uint16_t blocks = pixy.getBlocks();
	if (blocks)
	{
		pixyTS = millis();

		// Get Y pos from last block and convert to CM
		double pixyPosY = pixy.blocks[blocks - 1].y;
		pixyPosRel =
			pixyCoeff[0] * (pixyPosY * pixyPosY * pixyPosY * pixyPosY) +
			pixyCoeff[1] * (pixyPosY * pixyPosY * pixyPosY) +
			pixyCoeff[2] * (pixyPosY * pixyPosY) +
			pixyCoeff[3] * pixyPosY +
			pixyCoeff[4];

		posUpdate[2] = true;
	}
}

// CALCULATE PIXY COORD IN ABSOLUTE SPACE
float GetPixyAbsPos(float pxRel, float rbPos)
{
	// Compute position error
	double pxAbs = pxRel - pixyTarg;
	pxAbs = pxAbs + setPoint;
	pxAbs = rbPos + pxAbs;
	if (pxAbs > (140 * PI))
	{
		pxAbs = pxAbs - (140 * PI);
	}
	return pxAbs;
}

// UPDATE POS DATA 
void UpdatePos()
{

	// Update loop time
	dtLoop = (float)(millis() - (t_nextLoop - loopTim));
	t_nextLoop = millis() + loopTim;

	//----------UPDATE SIM DATA---------
	
	// UPDATE ROBOT
	if (posUpdate[1])
	{
		// Update robot first
		vtRobHist.UpdatePosVel(vtCM[1], vtTS[1]);

		// Check for new data next loop
		posUpdate[1] = false;
	}

	// UPDATE RAT
	if (posUpdate[0] && posUpdate[2])
	{
		if (do_simulateRat)
		{
			// Add noise for pixy input
			pixyCM = vtCM[0] + (float)random(0, 10) / 100;
			if (pixyCM > (140 * PI)) pixyCM = pixyCM - (140 * PI);
			pixyTS = vtTS[0];

		}
		// Update pos and vel from real data
		else
		{
			// Convert pixy cm to absolute space
			pixyCM = GetPixyAbsPos(pixyPosRel, vtCM[1]);
		}

		// Make sure rat pos > rob pos at start
		if (vtRatHist.sampCnt == 1)
		{
			// Check if rat pos < robot
			if (vtRatHist.nLaps == 0 &&
				vtCM[0] < vtRobHist.posNow)
			{
				vtRatHist.nLaps++;
				pixyRatHist.nLaps++;
			}
		}
		// Update rat pos and vel
		vtRatHist.UpdatePosVel(vtCM[0], vtTS[0]);
		pixyRatHist.UpdatePosVel(pixyCM, pixyTS);

		// Check for new data next loop
		posUpdate[0] = false;
		posUpdate[2] = false;
		if (do_simulateRat)
		{
			posUpdate[2] = true;
		}
	}

}

// UPDATE EKF VEL AND POS
void UpdateEKF()
{

	//----------UPDATE EKF---------
	if (t_ekfStr == 0) t_ekfStr = millis();

	double z[M] = {
		vtRatHist.posNow,
		pixyRatHist.posNow,
		vtRobHist.posNow,
		vtRatHist.velNow,
		pixyRatHist.velNow,
		vtRobHist.velNow,
	};

	// Run EKF
	ekf.step(z);

	// Update error estimate
	estRatPos = ekf.getX(0);
	estrob_pos = ekf.getX(1);
	estRatVel = ekf.getX(2);
	estRobVel = ekf.getX(3);


}

// RUN PID AND UPDATE ROBOT SPEED
void UpdateSpeed()
{
	//----------UPDATE PID---------

		// Compute error and other terms
	setPos = estrob_pos + setPoint;
	error = estRatPos - setPos;
	if (do_includeTerm[0])
	{
		// Catch zero crossing
		if ((abs(error) + abs(lastError)) > abs(error + lastError))
			integral = 0;
		//else if (error < 0) integral = integral + error * 5;
		else integral = integral + error;
	}
	else integral = 0;
	if (do_includeTerm[1])
	{
		derivative = error - lastError;
	}

	float pTerm = Kp*error;
	float iTerm = Ki*integral;
	float dTerm = Kd*derivative;
	//if (error < 0) dTerm = dTerm * 2.0f;

	// Compute updated vel
	velUpdate = pTerm + iTerm + dTerm;
	runSpeed = estRobVel + velUpdate;

	// Keep speed in range [0, maxSpeed]
	if (runSpeed > maxSpeed) runSpeed = maxSpeed;
	else if (runSpeed < 0) runSpeed = 0;

	// If rat stopped behind target halt running
	if (estRatVel < 1 && error < 0 && !haltRun)
	{
		Interupt_Halt();
	}

	// Update AutoDriver
	if (!haltRun || (error > 0))
	{
		//board.run(FWD, runSpeed*cm2stp);
		haltRun = false;
		zeroIntegral = false;

	}


	// Compute occilation period
	if (do_simulateRat)
	{
		if (lastError > 0 && error < 0)
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

	// Updatew last error
	lastError = error;

}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// START REWARD
void StartRew()
{
	// Start reward
	// chack if sol already open
	if (digitalRead(pin_Rel_1) == LOW) {
		if (doTrackRun)
		{
			// Halt robot 
			Interupt_Halt();
			// Set hold time
			t_holdTill = millis() + t_runHold;
		}
		// trigger reward tone on
		Send2_Ard(r2a_id[0]);
		PrintState("SEND TONE ON");
		doingRew = true;
		// turn on reward LED
		analogWrite(pin_RewLED, rewLEDDuty);
		// open solenoid
		digitalWrite(pin_Rel_1, HIGH);
		// compute stop time
		t_rewStop = millis() + solDir;
		char str[50];
		t_rewStr = millis() / 1000;
		sprintf(str, "Reward (%0.0fs)", t_rewStr);
		myGLCD.print(str, CENTER, 20);
		myGLCD.update();
	}
}

// END REWARD
void EndRew()
{
	// turn off reward LED
	analogWrite(pin_RewLED, 0);
	// close solenoid
	digitalWrite(pin_Rel_1, LOW);
	// reset
	doingRew = false;
	myGLCD.clrScr();
	myGLCD.update();
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


#pragma region --------MINOR FUNCTIONS---------

void QuitSession()
{
	// Stop motor
	Interupt_Halt();
	delay(100);
	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

// PRINT STATUS
void PrintState(String str)
{
	if (doPrintFlow)
	{
		char msg[50];
		float t = (float)(millis() / 1000.0f);
		sprintf(msg, " (%0.2f sec)\n", t);
		SerialUSB.print('\n');
		SerialUSB.print(str);
		SerialUSB.print(msg);
	}
}


#pragma endregion


#pragma region ---------INTERUPTS---------

// Halt run on IR trigger
void Interupt_Halt() {
	board.hardStop();
	haltRun = true;
	zeroIntegral = true;
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
	doRew = true;

	t_debounce[1] = millis() + 250;

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

