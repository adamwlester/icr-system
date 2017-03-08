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
#define N 1     // State value
#define M 2     // Measurements
#include <TinyEKF.h>

// DEFINE PINS
// Autodriver
const int pin_AD_CSP = 5;
const int pin_AD_Busy = 4;
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
volatile long intDebounce[3] = { millis(), millis(), millis() };

// VARIABLE SETUP
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
uint32_t vtLast = millis();
int vtDt;
bool vtUpdated[2] = { false, false };
byte vtEnt;
float vtRad[2];
int vtTS[2];
const float vtTarg = 0.78; // (rad)
float vtPosError; // (cm)
// Reward
const long solDir = 1000; // (ms) on duration
volatile byte solState = HIGH;
volatile byte rewNow = false;
long rewStopTim = millis();
// LEDs
const int rewLEDDuty = 50; // value between 0 and 255
const int trackLEDDuty = 75; // value between 0 and 255
// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
volatile byte lndLedOn = false;
long printNextTim = millis();
// Pixy
const float pixyTarg = 15;
const float pixyCoeff[5] = {
  0.000000064751850,
  -0.000029178819914,
  0.005627883140337,
  -0.690663759936117,
  38.949828090278942
};
float pixyLastTS;
float pixyDeltaTS;
float pixyPosY;
float pixyPosCM;
float pixyPosError;
// Other
float posErrorEstimate;
float timSec = 0;

// INITILIZE OBJECTS
// AutoDriver
AutoDriver_Due board(pin_AD_CSP, pin_AD_Reset, pin_AD_Busy);
// Pixy
PixyI2C pixy(0x54);
// LCD
LCD5110 myGLCD(pin_Disp_CS, pin_Disp_RST, pin_Disp_DC, pin_Disp_MOSI, pin_Disp_SCK);


// Create Fuser class
class Fuser : public TinyEKF {

public:

	Fuser()
	{
		// We approximate the process noise using a small constant
		this->setQ(0, 0, .0001);

		// Same for measurement noise
		this->setR(0, 0, .0001);
		this->setR(1, 1, .0001);
	}

protected:

	void model(double fx[N], double F[N][N], double hx[M], double H[M][N])
	{
		// Process model is f(x) = x
		fx[0] = this->x[0];

		// So process model Jacobian is identity matrix
		F[0][0] = 1;

		// Measurement function
		hx[0] = this->x[0]; // Barometric pressure from previous state
		hx[1] = this->x[0]; // Baro temperature from previous state

		// Jacobian of measurement function
		H[0][0] = 1;        // Barometric pressure from previous state
		H[1][0] = 1;       // Baro temperature from previous state
	}
};

// TinyEKF
Fuser ekf;

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

	// Initialize LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);

	// Set LEDs
	analogWrite(pin_RewLED, 0);
	analogWrite(pin_TrackLED, trackLEDDuty);

	// Start white noise
	//Serial1.write('0');

	board.run(FWD, 0);

}

void loop() {

	// Check XBee for new input
	if (Serial1.available() > 0)
	{
		bool pass = ParseSerial();
		if (pass)
		{
			vtUpdated[vtEnt] = true;
		}
	}

	// Get new Pixy data
	uint16_t blocks = pixy.getBlocks();
	if (blocks)
	{
		ProcPixy();
	}

	// Combine measurements and run EKF
	if (vtUpdated[0] && vtUpdated[0])
	{
		vtDt = (int)(millis() - vtLast);
		vtLast = millis();

		// Compute error
		vtPosError = CompVT_Err();
		pixyPosError = pixyPosCM - pixyTarg;

		double z[M] = {
			vtPosError,
			pixyPosError
		};
		// Run EKF
		ekf.step(z);

		// Update pos error estimate
		posErrorEstimate = ekf.getX(0);

		vtUpdated[0] = false;
		vtUpdated[1] = false;
	}

	//// Check for robot speed input
	//if (SerialUSB.available() > 0)
	//{
	//	int runSpeed = SerialUSB.parseInt();
	//	SerialUSB.println(runSpeed);
	//	// Run robot
	//	board.run(FWD, runSpeed);
	//}

	// Trigger reward
	if (rewNow) {
		TrigRew();
	}

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
		//char msg[50];
		//sprintf(msg, "%s%s,%s,%d,%d,%0.2f,%s)", head, msg_id, vtEnt, vtTS[vtEnt], vtRad[vtEnt], foot);
		//SerialUSB.print(msg);
	}
	else return pass = false;

}

float CompVT_Err()
{
	// Compute position error
	float err = min((2 * PI) - abs(vtRad[0] - vtRad[1]), abs(vtRad[0] - vtRad[1]));
	err = err - vtTarg;
	err = err * ((140 * PI) / (2 * PI));
	return err;
}

void ProcPixy() {
	// Save block time and delta
	pixyDeltaTS = (float)(millis() - pixyLastTS) / 1000;
	pixyLastTS = millis();

	// Get Y pos and convert to CM
	pixyPosY = pixy.blocks[0].y;
	pixyPosCM =
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
		rewStopTim = millis() + solDir;
		char str[50];
		timSec = millis() / 1000;
		sprintf(str, "Reward (%0.0fs)", timSec);
		myGLCD.print(str, CENTER, 20);
		myGLCD.update();
	}
	else if (rewStopTim < millis()) {
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
}

// Open/close solonoid
void Interupt_Btn1() {

	// exit if < 250 ms has not passed
	if (intDebounce[0] > millis()) return;

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

	intDebounce[0] = millis() + 250;
}

// Trigger reward
void Interupt_Btn2() {

	// exit if < 250 ms has not passed
	if (intDebounce[1] > millis()) return;

	if (~rewNow) {
		rewNow = true;
	}

	intDebounce[1] = millis() + 250;

}

// Turn on/off LCD LED
void Interupt_Btn3() {

	// exit if < 250 ms has not passed
	if (intDebounce[2] > millis()) return;

	if (!lndLedOn) {
		analogWrite(pin_Disp_LED, 50);
		lndLedOn = true;
	}
	else {
		analogWrite(pin_Disp_LED, 0);
		lndLedOn = false;
	}

	intDebounce[2] = millis() + 250;

}
