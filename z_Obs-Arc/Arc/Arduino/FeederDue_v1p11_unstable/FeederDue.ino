// LIBRARIES
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
// Arduino-PID-Library
#include <PID_v1.h>

// DEFINE PINS
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

// VARIABLE SETUP
// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 60; // (cm)
const float maxAcc = 30; // (cm)
const float maxDec = 60; // (cm)
float runSpeed;
bool haltRun = true;
// Kalman model measures
float estRatPos;
float estRobPos;
float estRatVel;
float estRobVel;
bool posUpdate[3] = { false, false, false };
// PID parameters
double inPID, outPID;
double setpoint = 0.78 * ((140 * PI) / (2 * PI)); // (cm)
// aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;
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

// CONSTRUCT OBJECTS
// AutoDriver
AutoDriver_Due board(pin_AD_CSP, pin_AD_Reset);
// Pixy
PixyI2C pixy(0x54);
// PID links and initial tuning parameters
PID myPID(&inPID, &outPID, &setpoint, consKp, consKi, consKd, DIRECT);
// LCD
LCD5110 myGLCD(pin_Disp_CS, pin_Disp_RST, pin_Disp_DC, pin_Disp_MOSI, pin_Disp_SCK);


// Create PosArr class
class PosArr
{
private:

	char objID;
	int nSamp;
	float posArr[6];
	int tsArr[6];
	int iteration = 0;
	int t_skip = 0;

public:

	float velLast = 0.0f;

	// constructor
	PosArr(char id, int l)
	{
		objID = id;
		nSamp = l;
		for (int i = 0; i < l; i++) posArr[i] = 0.0f;
	}

	float CompVelAcc(float posNew, int tsNew)
	{
		// shift and add data
		for (int i = 0; i < nSamp - 1; i++)
		{
			posArr[i] = posArr[i + 1];
			tsArr[i] = tsArr[i + 1];
		}
		posArr[nSamp - 1] = posNew;
		tsArr[nSamp - 1] = tsNew;
		// update itteration count
		if (iteration < nSamp) iteration++;
		// skip till array filled
		if (iteration >= nSamp)
		{
			// compute vel
			float dist;
			float distSum;
			int dt;
			float dtSum;
			float vel;
			for (int i = 0; i < nSamp - 1; i++)
			{
				// compute distance
				dist = min((140 * PI) - abs(posArr[i + 1] - posArr[i]), abs(posArr[i + 1] - posArr[i]));
				if (abs(posArr[i + 1] - posArr[i]) != posArr[i + 1] - posArr[i])
				{
					dist = dist*-1;
				}
				distSum += dist;
				// compute dt
				dt = tsArr[i + 1] - tsArr[i];
				dtSum += (float)dt;
			}
			// compute vel
			vel = distSum / (dtSum / 1000.0f);

			// ignore outlyer values unness too many frames discarted
			if (t_skip > 500 || abs(velLast - vel) < 15)
			{
				velLast = vel;
				t_skip = 0;
			}
			// add to skip time
			else t_skip += dt;
		}
		else
		{
			velLast = 0.0f;
		}

		return velLast;
	}
};
PosArr vtRatHist('A', 4);
PosArr pixyRatHist('B', 6);
PosArr vtRobHist('C', 4);

// Create Fuser class
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


// Union
union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	int16_t i[2]; // (int) 2 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
} u;

void setup() {

	// Wait for connection
	while (!Serial);
	// USB/Serial_Monitor
	SerialUSB.begin(0); // does not matter
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

	// Setup PID
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(0, maxSpeed);

	// Setup LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);

	// Set LEDs
	analogWrite(pin_RewLED, 0);
	analogWrite(pin_TrackLED, trackLEDDuty);

	// Start white noise
	//Serial1.write('0');
	board.softStop();
	//board.run(FWD, 0);

}

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

	// Combine measurements, run EKF and update speed
	if (posUpdate[0] && posUpdate[1] && posUpdate[2])
	{
		UpdateEKF();
		UpdateSpeed();
	}

	// Check for robot speed input
	if (SerialUSB.available() > 0)
	{
		runSpeed = (float)SerialUSB.parseInt();
		SerialUSB.println(runSpeed);
		// Run robot
		board.run(FWD, runSpeed*cm2stp);
		return;
	}

	// Trigger reward
	if (rewNow) {
		TrigRew();
	}

}

void UpdateEKF()
{
	// Convert pixy cm to absolute space
	pixyPosCM = GetPixyAbsPos(pixyPosRel, vtPosCM[1]);
	double z[M] = {
		vtPosCM[0],
		pixyPosCM,
		vtPosCM[1],
		vtRatHist.CompVelAcc(vtPosCM[0], vtTS[0]),
		pixyRatHist.CompVelAcc(pixyPosCM, pixyPosTS),
		vtRobHist.CompVelAcc(vtPosCM[1], vtTS[0]),
	};

	// Run EKF
	ekf.step(z);

	// Update error estimate
	estRatPos = ekf.getX(0);
	estRobPos = ekf.getX(1);
	estRatVel = ekf.getX(2);
	estRobVel = ekf.getX(3);

	posUpdate[0] = false;
	posUpdate[1] = false;
	posUpdate[2] = false;
}

void UpdateSpeed()
{

	//const float targBnd[2] = { 2.5, 5 }; // (cm)

 //   // Calculate diference in velocity
	//float velDif = estRatVel - estRobVel;
	//
	//// Calculate target error for scaling
	//float posDif =
	//	min((140 * PI) - abs(estRobPos - estRatPos), abs(estRobPos - estRatPos));
	//float posErr = posDif - vtTarg;
	//// calculate scaling factor
	//float scale;
	//if (posErr < 0.0f) scale = posErr / targBnd[0];
	//else scale = posErr / targBnd[1];
	//if (scale > 1) scale = scale / abs(scale);
	//// increase scaling factor for deceleration
	//float dt = (float)(millis() - t_lastRunUpdate);
	//if (velDif < 1) scale = scale * (dt*maxDec);
	//else scale = scale * (dt*maxAcc);

	float velDif = estRatVel - estRobVel;
    inPID = 
		min((140 * PI) - abs(estRobPos - estRatPos), abs(estRobPos - estRatPos));
	float posErr = inPID - setpoint;
	myPID.SetSampleTime((int)(millis() - myPID.GetLastTime()));
	myPID.SetTunings(consKp, consKi, consKd);
	myPID.Compute();
	runSpeed = outPID;

    // If rat stopped behind target halt running
	if (estRatVel < 1 && posErr < 0 && !haltRun)
	{
		Interupt_Halt();
	}

	//if (t_nextUpdate < millis())
	//{
	if (!haltRun || (posErr > 0.0f))
	{
		//board.softStop();
		//while (board.busyCheck());
		//delayMicroseconds(100);
		//board.run(FWD, runSpeed*cm2stp);
		haltRun = false;

	}
	//}
}

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
		msg_rec = u.i[0];
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
	else return pass = false;

}

float Rad2cm(float rad)
{
	rad = abs(rad - (2 * PI)); // flip
	return rad*((140 * PI) / (2 * PI)); // convert
}

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

//========================== INTERUPTS ==========================

// Halt run on IR trigger
void Interupt_Halt() {
	board.hardStop();
	haltRun = true;
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
