#pragma region ---------DEBUG SETTINGS---------

// POT Test 
/*
Connect pot and use to test motor function
*/
const bool do_POT_Test = false;

// POT Test 
/*
Test voltage measurment
*/
const bool do_VoltTest = false;

// XBee Test 
/*
Send keyboard entries from XBee on XCTU
*/
const bool do_XBeeTest = false;

// Solenoid Test 
/*
Open Ensure w/ button1 and Ethonal w/ button 2
*/
const bool do_SolTest = false;

// IR Detector test 
/*
Upload code to both CheetahDue and FeederDue)
Connect 34 on FeederDue to IR relay pin on CheetahDue
Make sure pin A8 is shorted to A9 on CheetahDue
*/
const bool do_IR_ComTest = true;
const uint32_t t_IR_Del = 1000; // (ms)
const uint32_t t_IR_Dur = 5; // (ms)
const uint32_t t_LED_Dur = 250; // (ms)

#pragma endregion


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


#pragma region ---------PIN DECLARATION---------

// Autodriver
const int pin_AD_CSP_R = 5;
const int pin_AD_CSP_F = 6;
const int pin_AD_RST = 7;

// Display
const int pin_Disp_SCK = 8;
const int pin_Disp_MOSI = 9;
const int pin_Disp_DC = 10;
const int pin_Disp_RST = 11;
const int pin_Disp_CS = 12;
const int pin_Disp_LED = 13;

// LEDs
const int pin_RewLED_R = 4;
const int pin_RewLED_C = 3;
const int pin_TrackLED = 2;

// Relays
const int pin_Rel_Ens = 22;
const int pin_Rel_Eth = 23;

// BigEasyDriver
const int pin_ED_RST = 47;
const int pin_ED_SLP = 49;
const int pin_ED_DIR = 51;
const int pin_ED_STP = 53;
const int pin_ED_ENBL = 35;
const int pin_ED_MS1 = 37;
const int pin_ED_MS2 = 39;
const int pin_ED_MS3 = 41;

// Feeder switch
const int pin_FeedSwitch_Gnd = 14;
const int pin_FeedSwitch = 15;

// Power off
const int pin_PwrOff = 45;

// Voltage monitor
const int pin_BatVolt = A11;

/*
Note: pins bellow are all used for external interupts
and must all be members of the same port (PortA)
*/

// IR proximity sensors
const int pin_IRprox_Rt = 42;
const int pin_IRprox_Lft = 43;

// IR detector
const int pin_IRdetect = 17;

// Buttons
const int pin_Btn[3] = { A3, A2, A1 };

// POT switch test
int pin_POT_Gnd = A10;
int pin_POT_In = A9;
int pin_POT_Vcc = A8;

// IR detector test
int pin_IRdetect_Relay = 9;
int pin_IRdetect_PlsIn = 24;
int pin_CheetahDueID_Gnd = A8;
int pin_CheetahDueID_IP = A9;

#pragma endregion


#pragma region ---------VARIABLE SETUP---------

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 100; // (cm)
const float maxAcc = 80; // (cm)
const float maxDec = 80; // (cm)
const byte kAcc = 60 * 2;
const byte kDec = 60 * 2;
const byte kRun = 60;
const byte kHold = 60 / 2;

// POT Variables
const float Pi = 3.141593;
int rewLEDDuty = 50; // value between 0 and 255
int trackLEDDuty = 75; // value between 0 and 255
volatile boolean ir_1_on = false;
volatile boolean ir_2_on = false;
float vccMax = 1024;
float vccNow;
float vccNorm;
float velNow;
int runSpeed = 0;
int pastSpeed = 99;
boolean switched = false;
long switchLst = millis();

// Button variables
volatile uint32_t t_debounce[3] = { millis(), millis(), millis() };

// Voltage test variables
float bit2volt = 0.0164;
float bitVolt;
float batVoltArr[100];
float batVoltNow;
float batVoltAvg;
bool firstPass = true;

// IR Com test variables
volatile int cnt_IR_Sent = 0;
volatile int cnt_IR_Rcvd = 0;
volatile int cnt_Pls_Rcvd = 0;
volatile int cnt_IR_Trig = 0;
volatile int cnt_Pls_Trig = 0;
uint32_t t_IR_Sent = 0; // (ms)
volatile uint32_t t_IR_Rcvd = 0; // (us)
volatile uint32_t t_Pls_Rcvd = 0; // (us)
uint32_t t_IR_Lat = 0; // (us)
volatile bool is_IR_On = false;
bool is_FeederDue = false;
volatile bool is_IR_Rcvd = false;
volatile bool is_Pls_Rcvd = false;

//----------INITILIZE OBJECTS----------

// AutoDriver
AutoDriver_Due ad_R(pin_AD_CSP_R, pin_AD_RST);
AutoDriver_Due ad_F(pin_AD_CSP_F, pin_AD_RST);

// LCD
LCD5110 myGLCD(pin_Disp_CS, pin_Disp_RST, pin_Disp_DC, pin_Disp_MOSI, pin_Disp_SCK);
extern unsigned char SmallFont[];

#pragma endregion


#pragma region ---------CLASS SETUP---------

//----------CLASS: union----------
union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (int16) 2 byte
	uint32_t i32; // (int32) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
}
// Initialize object
u;

#pragma endregion

void setup()
{

	delayMicroseconds(100);

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// XBee
	Serial1.begin(57600);

	// SETUP OUTPUT POWER AND GROUND PINS

	// Set output pins
	pinMode(pin_Rel_Ens, OUTPUT);
	pinMode(pin_Rel_Eth, OUTPUT);
	pinMode(pin_ED_STP, OUTPUT);
	pinMode(pin_ED_DIR, OUTPUT);
	pinMode(pin_ED_SLP, OUTPUT);
	pinMode(pin_ED_MS1, OUTPUT);
	pinMode(pin_ED_MS2, OUTPUT);
	pinMode(pin_ED_ENBL, OUTPUT);
	pinMode(pin_PwrOff, OUTPUT);
	pinMode(pin_FeedSwitch_Gnd, OUTPUT);

	// Set power/ground pins
	digitalWrite(pin_FeedSwitch_Gnd, LOW);

	// Set button pins enable internal pullup
	for (int i = 0; i <= 2; i++) {
		pinMode(pin_Btn[i], INPUT_PULLUP);
	}
	pinMode(pin_FeedSwitch, INPUT_PULLUP);

	// Make sure relay pins low
	digitalWrite(pin_Rel_Ens, LOW);
	digitalWrite(pin_Rel_Eth, LOW);

	// SETUP AUTODRIVER

	// Configure SPI
	ad_R.SPIConfig();
	delayMicroseconds(100);
	ad_F.SPIConfig();
	delayMicroseconds(100);
	// Reset each axis
	ad_R.resetDev();
	delayMicroseconds(100);
	ad_F.resetDev();
	delayMicroseconds(100);
	// Configure each axis
	dSPINConfig_board();
	delayMicroseconds(100);
	// Get the status to clear the UVLO Flag
	ad_R.getStatus();
	delayMicroseconds(100);
	ad_F.getStatus();

	// Make sure motor is stopped and in high impedance
	ad_R.hardHiZ();
	ad_F.hardHiZ();

	// Print ad board status
	SerialUSB.println(' ');
	SerialUSB.print("BOARD R STATUS: ");
	SerialUSB.println(ad_R.getStatus(), HEX);
	SerialUSB.print("BOARD F STATUS: ");
	SerialUSB.println(ad_F.getStatus(), HEX);

	// SETUP BIG EASY DRIVER

	// Set to 1/2 step mode
	digitalWrite(pin_ED_MS1, HIGH);
	digitalWrite(pin_ED_MS2, LOW);
	digitalWrite(pin_ED_MS3, LOW);

	// Start BigEasyDriver in sleep
	digitalWrite(pin_ED_SLP, LOW);

	// DEFINE EXTERNAL INTERUPTS
	attachInterrupt(digitalPinToInterrupt(pin_Btn[0]), Interupt_Btn1, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_Btn[1]), Interupt_Btn2, FALLING);
	attachInterrupt(digitalPinToInterrupt(pin_Btn[2]), Interupt_Btn3, FALLING);
	if (do_POT_Test)
	{
		attachInterrupt(digitalPinToInterrupt(pin_IRprox_Rt), Interupt_IRprox_Halt, FALLING);
		attachInterrupt(digitalPinToInterrupt(pin_IRprox_Lft), Interupt_IRprox_Halt, FALLING);
	}

	// INITIALIZE LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);
	myGLCD.invert(true);

	// TEST STUFF SETUP

	// POT vcc and Gnd
	pinMode(pin_POT_Gnd, OUTPUT);
	pinMode(pin_POT_Vcc, OUTPUT);
	digitalWrite(pin_POT_Vcc, HIGH);
	digitalWrite(pin_POT_Gnd, LOW);

	// Initialize bat volt array
	for (int i = 0; i <= 100; i++) {
		batVoltArr[i] = 0;
	}


	if (do_IR_ComTest)
	{
		pinMode(pin_CheetahDueID_IP, INPUT_PULLUP);
		pinMode(pin_CheetahDueID_Gnd, OUTPUT);
		digitalWrite(pin_CheetahDueID_Gnd, LOW);
		delay(10);
		// Detect if code is running on FeederDue
		if (digitalRead(pin_CheetahDueID_IP) == HIGH)
		{
			is_FeederDue = true;
			attachInterrupt(digitalPinToInterrupt(pin_IRdetect), Interupt_IR_Detect, HIGH);
			attachInterrupt(digitalPinToInterrupt(pin_IRdetect_PlsIn), Interupt_Pulse_Detect, HIGH);
			SerialUSB.println("Running on FeederDue");
		}
		else
		{
			// Set all relays low on CheetahDue
			pinMode(pin_IRdetect_Relay, OUTPUT);
			pinMode(11, OUTPUT);
			pinMode(12, OUTPUT);
			digitalWrite(pin_IRdetect_Relay, LOW);
			digitalWrite(11, LOW);
			digitalWrite(12, LOW);
			SerialUSB.println("Running on CheetahDue");
		}
		pinMode(pin_CheetahDueID_IP, INPUT);
		pinMode(pin_CheetahDueID_Gnd, INPUT);
	}

}

void loop()
{
	// POT Test
	if (do_POT_Test)
	{
		POT_Run();
	}

	// Voltage Test
	if (do_VoltTest)
	{
		Volt_Read();
	}

	// XBee Test
	if (do_XBeeTest)
	{
		XBee_Read();
	}

	// IR Com Test
	if (do_IR_ComTest)
	{
		IR_Send();
		IR_Lat_Comp();
	}


}

void POT_Run() {

	vccNow = analogRead(pin_POT_In);
	vccNorm = vccNow / vccMax;
	velNow = round(vccNorm * maxSpeed * 100) / 100;
	runSpeed = velNow * cm2stp;
	millis();

	if (!switched && runSpeed != pastSpeed) {
		switchLst = millis();
		switched = true;
	}
	if (switched && (millis() - switchLst >= 250)) {
		SerialUSB.print("New Vcc: ");
		SerialUSB.print(vccNow);
		SerialUSB.print("    New Speed: ");
		SerialUSB.println(runSpeed / cm2stp);
		pastSpeed = runSpeed;
		switched = false;
		if (velNow <= 1)
		{
			ad_R.hardStop();
			ad_F.hardStop();
			analogWrite(13, 0);
		}
		else
		{
			ad_R.run(FWD, runSpeed);
			ad_F.run(FWD, runSpeed);
			analogWrite(13, 120);
		}
	}
}

void Volt_Read()
{
	delay(10);
	if (firstPass)
	{
		digitalWrite(pin_Rel_Eth, HIGH);
		firstPass = false;
	}
	bitVolt = analogRead(pin_BatVolt);
	batVoltNow = bitVolt * bit2volt;
	// Shift array and compute average
	float voltSum = 0;
	for (int i = 99; i > 0; i--) {
		batVoltArr[i] = batVoltArr[i - 1];
		voltSum += batVoltArr[i];
	}
	batVoltArr[0] = batVoltNow;
	batVoltAvg = voltSum / 99;
	SerialUSB.println(batVoltAvg);
}

void XBee_Read()
{
	byte buff_b;
	char buff_c[2] = { '\0' ,'\r' };
	if (Serial1.available() > 0) {
		buff_b = Serial1.read();
		u.b[0] = buff_b;
		buff_c[0] = u.c[0];
		SerialUSB.print(buff_c);
	}
}

void IR_Send()
{
	if (!is_FeederDue)
	{
		// Set IR relay high
		if (
			!is_IR_On &&
			millis() > t_IR_Sent + t_IR_Dur + t_IR_Del
			)
		{
			digitalWrite(pin_IRdetect_Relay, HIGH);
			t_IR_Sent = millis();
			cnt_IR_Sent++;
			is_IR_On = true;
			SerialUSB.println("IR On");
		}
		// Set IR relay low
		else if (
			is_IR_On &&
			millis() > t_IR_Sent + t_IR_Dur
			)
		{
			digitalWrite(pin_IRdetect_Relay, LOW);
			is_IR_On = false;
			SerialUSB.println("IR Off");
		}
	}
}

void Interupt_IR_Detect()
{
	// Turn on LED and store count
	if (is_FeederDue && !is_IR_On && !is_IR_Rcvd)
	{
		// Save IR time
		t_IR_Rcvd = micros();
		cnt_IR_Rcvd++;
		is_IR_On = true;
		is_IR_Rcvd = true;
		//char str[50];
		//sprintf(str, "__IR Detected:    %d (ms)", t_IR_Rcvd);
		//SerialUSB.println(str);
	}
	//cnt_IR_Trig++;
}

void Interupt_Pulse_Detect()
{
	// Store count
	if (is_FeederDue && !is_IR_On && !is_Pls_Rcvd)
	{
		// Save pulse time
		t_Pls_Rcvd = micros();
		cnt_Pls_Rcvd++;
		is_Pls_Rcvd = true;
		//char str[50];
		//sprintf(str, "__Pulse Detected: %d (ms)", t_Pls_Rcvd);
		//SerialUSB.println(str);
	}
	//cnt_Pls_Trig++;
}

void IR_Lat_Comp()
{
	if (is_FeederDue)
	{
		if (is_IR_Rcvd && is_Pls_Rcvd)
		{
			// Turn on tracker LED
			analogWrite(pin_TrackLED, 100);
			// Compute delay between pulse and IR
			t_IR_Lat = t_IR_Rcvd - t_Pls_Rcvd;
			// Print info
			char str[50];
			sprintf(str, "IR Lat: %d (us) [Cnt:%d|%d]", t_IR_Lat, cnt_Pls_Rcvd, cnt_IR_Rcvd);
			SerialUSB.println(str);
			// Print to LCD
			//sprintf(str, "IR Lat: %d (us)", t_IR_Lat);
			//myGLCD.clrScr();
			//myGLCD.print(str, LEFT, 20);
			//myGLCD.update();

			// Print number of times interupt triggered
			//sprintf(str, "Trigger Count: %d|%d", cnt_Pls_Trig, cnt_IR_Trig);
			//SerialUSB.println(str);

			// Reset counters and flags
			cnt_Pls_Trig = 0;
			cnt_IR_Trig = 0;
			is_IR_Rcvd = false;
			is_Pls_Rcvd = false;
		}
		// Turn off LED on FeederDue
		else if (
			is_IR_On &&
			micros() > t_IR_Rcvd + t_LED_Dur*1000
			)
		{
			// Turn off tracker LED
			analogWrite(pin_TrackLED, 0);
			is_IR_On = false;
		}
	}
}

void Feed_Switch() {
	if (digitalRead(pin_FeedSwitch) == LOW)
	{

	}
}

void Interupt_Btn1() {

	if (t_debounce[0] > millis()) return;
	t_debounce[0] = millis() + 250;

	if (do_SolTest)
	{
		if (digitalRead(pin_Rel_Ens) == LOW)
			digitalWrite(pin_Rel_Ens, HIGH);
		else digitalWrite(pin_Rel_Ens, LOW);
	}

	SerialUSB.println("Button 1");
}

void Interupt_Btn2() {

	if (t_debounce[1] > millis()) return;
	t_debounce[1] = millis() + 250;

	if (do_SolTest)
	{
		if (digitalRead(pin_Rel_Eth) == LOW)
			digitalWrite(pin_Rel_Eth, HIGH);
		else digitalWrite(pin_Rel_Eth, LOW);
	}

	SerialUSB.println("Button 2");
}

void Interupt_Btn3() {

	if (t_debounce[2] > millis()) return;
	t_debounce[2] = millis() + 250;

	if (do_IR_ComTest)
	{
		static bool is_LCD_On = false;
		if (is_LCD_On) { analogWrite(pin_Disp_LED, 25); }
		else { analogWrite(pin_Disp_LED, 0); }
		is_LCD_On = !is_LCD_On;
	}

	SerialUSB.println("Button 3");
}

void Interupt_IRprox_Halt() {
	ad_R.hardStop();
	ad_F.hardStop();
	SerialUSB.println("Run Halted");
}

