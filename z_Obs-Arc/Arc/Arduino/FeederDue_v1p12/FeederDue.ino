
#pragma region =========FeederDue=========

#pragma region ---------VARIABLE DECLARATION---------

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

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 80; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)

// Kalman model measures
float estRatPos;
float estRobPos;
float estRatVel;
float estRobVel;
bool posUpdate[3] = { false, false, false };
uint32_t t_ekfStr = 0;
uint32_t ekfSettleDel = 3000;

// PID Calibration
const bool do_simulateRat = true;
bool do_includeTerm[2] = { true, true };
float simNoise;
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
const float setpoint = 0.78 * ((140 * PI) / (2 * PI)); // (cm)                  
float error;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pTerm;
float iTerm;
float dTerm;
volatile bool haltRun = true;
volatile bool zeroIntegral = true;

// Serial parsing
int colonInd;
int cammaInd[2];
char msg_head[2] = { '<', '_' };
char msg_foot = '>';
char msg_idArr[4] = { 'P', 'S', 'H', 'R' };
char msg_id;
int16_t msg_rec;
int cnt = 0;

// Serial VT
int vtDt;
byte vtEnt;
float vtRad[2];
int vtTS[2];
float vtPosCM[2];
const float vtTarg = setpoint; // (cm)

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
float pixyPosCM;
uint32_t  pixyPosTS;

// Reward
const long solDir = 1000; // (ms) on duration
volatile byte solState = HIGH;
volatile byte rewNow = false;
uint32_t t_rewStop = millis();
float t_rewStr;

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


#pragma region ---------CONSTRUC CLASSES---------

//----------CLASS: PosDat----------
class PosDat
{
private:

	char objID;
	int nSamp;
	float posArr[6];
	int tsArr[6];
	int iteration = 0;
	int t_skip = 0;

public:

	float velNow = 0.0f;
	float posNow = 0.0f;
	float nLaps = 0;
	bool doEKF = false;

	// constructor
	PosDat(char id, int l)
	{
		this->objID = id;
		this->nSamp = l;
		for (int i = 0; i < l; i++) this->posArr[i] = 0.0f;
	}

	void UpdatePosVel(float posNew, int tsNew)
	{
		// Update itteration count
		this->iteration++;

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
		if (this->iteration < this->nSamp)
		{
			this->posNow = posNew;
			this->velNow = 0.0f;
		}
		else
		{
			if (!doEKF) doEKF = true;

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
	int16_t i16[2]; // (int16) 2 byte
	int i32; // (int32) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
} u;

#pragma endregion 


#pragma region  ---------SETUP---------

void setup() {

	// Wait for connection
	while (!Serial);
	// USB/Serial_Monitor
	SerialUSB.begin(0);
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

	// Set LEDs
	analogWrite(pin_RewLED, 0);
	analogWrite(pin_TrackLED, trackLEDDuty);

	// Start white noise

	//Serial1.write('0');
	board.softStop();
	//board.run(FWD, 0);

	// PID calibration setup
	if (do_simulateRat)
	{
		posUpdate[2] = true;
	}

}

#pragma endregion


#pragma region ---------MAIN LOOP---------

void loop() {

	// Check XBee for new input
	if (Serial1.available() > 0)
	{
		bool pass = ParseSerial();
		if (pass)
		{
			vtPosCM[vtEnt] = Rad2cm(vtRad[vtEnt]);
			posUpdate[vtEnt] = true;
		}
	}

	// Get new Pixy data
	uint16_t blocks = pixy.getBlocks();
	if (blocks)
	{
		pixyPosTS = millis();
		ProcPixy(blocks);
		posUpdate[2] = true;
	}

	// Combine measurements, run EKF and update speed with PID
	if (posUpdate[0] && posUpdate[1] && posUpdate[2])
	{
		if (millis() >= t_nextLoop)
		{
			UpdateSpeed();
		}
	}

	// Trigger reward
	if (rewNow) {
		TrigRew();
	}

}

#pragma endregion


#pragma region ---------SUPPORT FUN---------

// PARSE SERIAL INPUT
bool ParseSerial()
{
	char head[2];
	char foot;
	bool pass;

	// Dump data till msgHeader byte is reached
	while (Serial1.available() > 0)
	{
		while (Serial1.peek() != msg_head[0])
		{
			Serial1.read(); // dump
		}
		// Quit do not have complete block left
		if (Serial1.available() < 15)
		{
			return pass = false;
		}
		// Save headers and id
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		u.b[2] = Serial1.read();
		head[0] = u.c[0];
		head[1] = u.c[1];
		msg_id = u.c[2];
		// Check for second header char right after
		if (head[1] != msg_head[1])
		{
			// quit
			return pass = false;
		}
		else break;
	}

	// Get VT data
	if (msg_id == msg_idArr[0])
	{
		// Get record number
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		msg_rec = u.i16[0];
		// Get Ent
		vtEnt = Serial1.read();
		// Get TS
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = Serial1.read();
		}
		vtTS[vtEnt] = u.l;
		// Get Rad
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = Serial1.read();
		}
		vtRad[vtEnt] = u.f;
	}
	// Check for footer
	u.b[0] = Serial1.read();
	foot = u.c[0];
	if (foot == msg_foot) {
		cnt++;
		pass = true;
	}
	else pass = false;
	return pass;

}

// UPDATE ROBOT SPEED WITH PID 
void UpdateSpeed()
{

	// Update loop time
	dtLoop = (float)(millis() - (t_nextLoop - loopTim));
	t_nextLoop = millis() + loopTim;

	//----------UPDATE SIM DATA---------
	if (do_simulateRat)
	{

		// Add noise
		simNoise = (float)random(0, 10) / 100;

		// Update pos and vel from sim data
		vtRatHist.UpdatePosVel(vtPosCM[0], vtTS[0]);
		pixyRatHist.UpdatePosVel(vtPosCM[0] + simNoise, vtTS[0]);
		vtRobHist.UpdatePosVel(vtPosCM[1], vtTS[1]);

	}

	// Update pos and vel from real data
	else
	{
		// Convert pixy cm to absolute space
		pixyPosCM = GetPixyAbsPos(pixyPosRel, vtPosCM[1]);

		// Update pos and vel
		vtRatHist.UpdatePosVel(vtPosCM[0], vtTS[0]);
		pixyRatHist.UpdatePosVel(pixyPosCM, pixyPosTS);
		vtRobHist.UpdatePosVel(vtPosCM[1], vtTS[1]);

	}

	// Wait till enough position data has been sampled at startup
	if (!vtRatHist.doEKF || !pixyRatHist.doEKF || !vtRobHist.doEKF)
	{
		// Check if rat pos < robot
		if (vtRatHist.nLaps == 0 &&
			vtRatHist.posNow < vtRobHist.posNow)
		{
			vtRatHist.nLaps++;
			pixyRatHist.nLaps++;
		}
	}
	else
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
		estRobPos = ekf.getX(1);
		estRatVel = ekf.getX(2);
		estRobVel = ekf.getX(3);

		//----------UPDATE PID---------
		// Wait for ekf to settle to stable value
		if ((millis() - t_ekfStr) > ekfSettleDel)
		{

			// Compute error and other terms
			setPos = estRobPos + setpoint;
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
				board.run(FWD, runSpeed*cm2stp);
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
				if (error < setpoint*-1);
			}



			// Updatew last error
			lastError = error;

		}

	}

	// Check for new data next loop
	posUpdate[0] = false;
	posUpdate[1] = false;
	posUpdate[2] = false;
	if (do_simulateRat)
	{
		posUpdate[2] = true;
	}

}

// PROCESS PIXY STREAM
void ProcPixy(int16_t blocks) {

	// Get Y pos from last block and convert to CM
	double pixyPosY = pixy.blocks[blocks - 1].y;
	pixyPosRel =
		pixyCoeff[0] * (pixyPosY * pixyPosY * pixyPosY * pixyPosY) +
		pixyCoeff[1] * (pixyPosY * pixyPosY * pixyPosY) +
		pixyCoeff[2] * (pixyPosY * pixyPosY) +
		pixyCoeff[3] * pixyPosY +
		pixyCoeff[4];
}

// CONVERT RAD TO CM
float Rad2cm(float rad)
{
	rad = abs(rad - (2 * PI)); // flip
	return rad*((140 * PI) / (2 * PI)); // convert
}

// CALCULATE PIXY COORD IN ABSOLUTE SPACE
float GetPixyAbsPos(float pxRel, float rbPos)
{
	// Compute position error
	double pxAbs = pxRel - pixyTarg;
	pxAbs = pxAbs + vtTarg;
	pxAbs = rbPos + pxAbs;
	if (pxAbs > (140 * PI))
	{
		pxAbs = pxAbs - (140 * PI);
	}
	return pxAbs;
}

// TRIGGER REWARD
void TrigRew()
{
	// chack if sol already open
	if (digitalRead(pin_Rel_1) == LOW) {
		// turn on reward LED
		analogWrite(pin_RewLED, rewLEDDuty);
		// trigger reward tone on
		Serial1.write('1');
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
	else if (t_rewStop < millis()) {
		// turn off reward LED
		analogWrite(pin_RewLED, 0);
		// trigger reward tone off
		Serial1.write('0');
		// close solenoid
		digitalWrite(pin_Rel_1, LOW);
		// reset
		rewNow = false;
		myGLCD.clrScr();
		myGLCD.update();
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

	t_debounce[0] = millis() + 250;
}

// Trigger reward
void Interupt_Btn2() {

	// exit if < 250 ms has not passed
	if (t_debounce[1] > millis()) return;

	if (~rewNow) {
		rewNow = true;
	}

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

#pragma endregion