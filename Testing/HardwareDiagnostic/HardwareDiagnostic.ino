#pragma region ---------DEBUG SETTINGS---------

// POT Test 
/*
Connect pot and use to test motor function
*/
const bool do_POT_Test = false;
const bool do_POT_PrintSpeed = false;
const bool do_POT_PrintStat = false;

// Feed Arm Test
/*
Retract arm w/ button 1 and extend arm w/ button 2
*/
const bool do_FeedArmTest = false;
bool is_armExtended = false;

// Volt Test 
/*
Test voltage measurment
*/
const bool do_VoltTest = false;

// XBee Test 
/*
Send keyboard entries from XBee on XCTU
*/
const bool do_XBeeTest = true;

// Solenoid Test 
/*
Open Rew sol w/ button 1 and EtOH w/ button 2
*/
const bool do_SolTest = false;

// IR Detector Timing test 
/*
Upload code to both CheetahDue and FeederDue)
Connect 34 on FeederDue to IR relay pin on CheetahDue
Make sure pin A8 is shorted to A9 on CheetahDue
*/
const bool do_IR_ComTest = false;
const uint32_t t_IR_Del = 5000; // [1000, 5000] (ms)
const uint32_t t_IR_Dur = 5000; // [5, 5000] (ms)
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
	const int PwrOff = 45;

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

	// POT switch test
	int POT_Gnd = A10;
	int POT_In = A9;
	int POT_Vcc = A8;

	// IR detector test
	int IRdetect_Relay = 51;
	int IRdetect_PlsIn = 24;
	int CheetahDueID_Gnd = A8;
	int CheetahDueID_IP = A9;
}
// Initialize
pin;



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

// LEDs
const int trackLEDduty = 75; // value between 0 and 255
const int rewLEDduty = 15; // value between 0 and 255
const int rewLEDmin = 0; // value between 0 and 255

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
uint16_t ad_r_stat;
uint16_t ad_f_stat;

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
AutoDriver_Due ad_R(pin.AD_CSP_R, pin.AD_RST);
AutoDriver_Due ad_F(pin.AD_CSP_F, pin.AD_RST);

// LCD
LCD5110 myGLCD(pin.Disp_CS, pin.Disp_RST, pin.Disp_DC, pin.Disp_MOSI, pin.Disp_SCK);
extern unsigned char SmallFont[];

#pragma endregion


#pragma region ---------CLASS/STRUCT/UNION SETUP---------

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


#pragma region ---------FUNCTION DECLARATION---------

int CheckAD_Status(uint16_t stat_reg, String stat_id);
void AD_Reset();

#pragma endregion


void setup()
{

	delayMicroseconds(100);

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// XBee
	Serial1.begin(57600);

	// SETUP OUTPUT PINS

	// Autodriver
	pinMode(pin.AD_CSP_R, OUTPUT);
	pinMode(pin.AD_CSP_F, OUTPUT);
	pinMode(pin.AD_RST, OUTPUT);
	// Display
	pinMode(pin.Disp_SCK, OUTPUT);
	pinMode(pin.Disp_MOSI, OUTPUT);
	pinMode(pin.Disp_DC, OUTPUT);
	pinMode(pin.Disp_RST, OUTPUT);
	pinMode(pin.Disp_CS, OUTPUT);
	pinMode(pin.Disp_LED, OUTPUT);
	// LEDs
	pinMode(pin.RewLED_R, OUTPUT);
	pinMode(pin.RewLED_C, OUTPUT);
	pinMode(pin.TrackLED, OUTPUT);
	// Relays
	pinMode(pin.Rel_Rew, OUTPUT);
	pinMode(pin.Rel_EtOH, OUTPUT);
	// BigEasyDriver
	pinMode(pin.ED_RST, OUTPUT);
	pinMode(pin.ED_SLP, OUTPUT);
	pinMode(pin.ED_DIR, OUTPUT);
	pinMode(pin.ED_STP, OUTPUT);
	pinMode(pin.ED_ENBL, OUTPUT);
	pinMode(pin.ED_MS1, OUTPUT);
	pinMode(pin.ED_MS2, OUTPUT);
	pinMode(pin.ED_MS3, OUTPUT);
	// OpenLog
	pinMode(pin.OL_RST, OUTPUT);
	// Feeder switch
	pinMode(pin.FeedSwitch_Gnd, OUTPUT);
	// Power off
	pinMode(pin.PwrOff, OUTPUT);
	delayMicroseconds(100);

	// Autodriver
	digitalWrite(pin.AD_CSP_R, LOW);
	digitalWrite(pin.AD_CSP_F, LOW);
	digitalWrite(pin.AD_RST, LOW);
	// Display
	digitalWrite(pin.Disp_SCK, LOW);
	digitalWrite(pin.Disp_MOSI, LOW);
	digitalWrite(pin.Disp_DC, LOW);
	digitalWrite(pin.Disp_RST, LOW);
	digitalWrite(pin.Disp_CS, LOW);
	digitalWrite(pin.Disp_LED, LOW);
	// LEDs
	digitalWrite(pin.RewLED_R, LOW);
	digitalWrite(pin.RewLED_C, LOW);
	digitalWrite(pin.TrackLED, LOW);
	// Relays
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);
	// OpenLog
	digitalWrite(pin.OL_RST, LOW);
	// Feeder switch
	digitalWrite(pin.FeedSwitch_Gnd, LOW);
	// Power off
	digitalWrite(pin.PwrOff, LOW);
	delayMicroseconds(100);

	// SET INPUT PINS

	// Voltage monitor
	pinMode(pin.BatVolt, INPUT);
	// IR proximity sensors
	pinMode(pin.IRprox_Rt, INPUT);
	pinMode(pin.IRprox_Lft, INPUT);
	// IR detector
	pinMode(pin.IRdetect, INPUT);

	// Set button pins enable internal pullup
	for (int i = 0; i <= 2; i++) {
		pinMode(pin.Btn[i], INPUT_PULLUP);
	}
	pinMode(pin.FeedSwitch, INPUT_PULLUP);
	delayMicroseconds(100);

	// SETUP AUTODRIVER

	// Configure SPI
	ad_R.SPIConfig();
	delayMicroseconds(100);
	ad_F.SPIConfig();
	delayMicroseconds(100);
	// Reset 
	AD_Reset();

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
	digitalWrite(pin.ED_MS1, HIGH);
	digitalWrite(pin.ED_MS2, LOW);
	digitalWrite(pin.ED_MS3, LOW);

	// Start BigEasyDriver in sleep
	digitalWrite(pin.ED_RST, HIGH);
	digitalWrite(pin.ED_SLP, LOW);
	digitalWrite(pin.ED_DIR, LOW);
	digitalWrite(pin.ED_STP, LOW);
	digitalWrite(pin.ED_ENBL, LOW);

	// DEFINE EXTERNAL INTERUPTS
	if (do_POT_Test)
	{
		attachInterrupt(digitalPinToInterrupt(pin.IRprox_Rt), InteruptIRproxHalt, FALLING);
		attachInterrupt(digitalPinToInterrupt(pin.IRprox_Lft), InteruptIRproxHalt, FALLING);
	}

	// INITIALIZE LCD
	myGLCD.InitLCD();
	myGLCD.setFont(SmallFont);
	myGLCD.invert(true);

	// TEST STUFF SETUP

	// POT vcc and Gnd
	pinMode(pin.POT_Gnd, OUTPUT);
	pinMode(pin.POT_Vcc, OUTPUT);
	digitalWrite(pin.POT_Vcc, HIGH);
	digitalWrite(pin.POT_Gnd, LOW);

	// Initialize bat volt array
	for (int i = 0; i < 100; i++) {
		batVoltArr[i] = 0;
	}


	if (do_IR_ComTest)
	{
		pinMode(pin.CheetahDueID_IP, INPUT_PULLUP);
		pinMode(pin.CheetahDueID_Gnd, OUTPUT);
		digitalWrite(pin.CheetahDueID_Gnd, LOW);
		delay(10);
		// Detect if code is running on FeederDue
		if (digitalRead(pin.CheetahDueID_IP) == HIGH)
		{
			is_FeederDue = true;
			attachInterrupt(digitalPinToInterrupt(pin.IRdetect), Interupt_IR_Detect, HIGH);
			attachInterrupt(digitalPinToInterrupt(pin.IRdetect_PlsIn), Interupt_Pulse_Detect, HIGH);
			SerialUSB.println("Running on FeederDue");
		}
		else
		{
			// Set all relays low on CheetahDue
			pinMode(pin.IRdetect_Relay, OUTPUT);
			pinMode(11, OUTPUT);
			pinMode(12, OUTPUT);
			digitalWrite(pin.IRdetect_Relay, LOW);
			digitalWrite(11, LOW);
			digitalWrite(12, LOW);
			SerialUSB.println("Running on CheetahDue");
		}
		pinMode(pin.CheetahDueID_IP, INPUT);
		pinMode(pin.CheetahDueID_Gnd, INPUT);
	}

}

void loop()
{
	static bool first_pass = true;
	if (first_pass)
	{
		// Blink to show setup done
		SetupBlink();
		first_pass = false;
	}

	// POT Test
	if (do_POT_Test)
	{
		POT_Run();

		// Check for errors
		static uint32_t t_check = millis() + 100;
		if (millis() > t_check)
		{
			char msg[50];
			// Get rear board status
			ad_r_stat = ad_R.getStatus();
			int ocd_r = CheckAD_Status(ad_r_stat, "OCD");
			int mot_r = CheckAD_Status(ad_r_stat, "MOT_STATUS");
			// Get front board status
			ad_f_stat = ad_F.getStatus();
			int ocd_f = CheckAD_Status(ad_f_stat, "OCD");
			int mot_f = CheckAD_Status(ad_f_stat, "MOT_STATUS");
			// Print OCD vals
			if (do_POT_PrintStat)
			{
				sprintf(msg, "AD: R_MOT=%d F_MOT=%d R_OCD=%d F_OCD=%d", mot_r, mot_f, ocd_r, ocd_f);
				SerialUSB.println(msg);
			}
			// Check for errors
			if (ocd_r == 0 || ocd_f == 0)
			{
				SerialUSB.println("!!ERROR!!");
				pastSpeed = 0;
				//AD_Reset();
			}
			t_check = millis() + 100;
		}
	}

	// Voltage Test
	if (do_VoltTest)
	{
		CheckBattery();
	}

	// XBee Test
	if (do_XBeeTest)
	{
		XBeeRead();
		ConsoleRead();
	}

	// IR Com Test
	if (do_IR_ComTest)
	{
		IR_Send();
		IR_LatComp();
	}

	// Check buttons for input
	CheckButtons();

}

bool POT_Run() {

	vccNow = analogRead(pin.POT_In);
	vccNorm = vccNow / vccMax;
	velNow = round(vccNorm * maxSpeed * 100) / 100;
	runSpeed = velNow * cm2stp;
	millis();
	bool is_speed_changed = false;

	if (!switched && runSpeed != pastSpeed) {
		switchLst = millis();
		switched = true;
	}
	if (switched && (millis() - switchLst >= 250)) {
		if (do_POT_PrintSpeed)
		{
			char msg[50];
			sprintf(msg, "AD: New Speed = %0.2fcm/sec", runSpeed / cm2stp);
			SerialUSB.println(msg);
		}
		pastSpeed = runSpeed;
		switched = false;
		is_speed_changed = true;
		if (velNow <= 1)
		{
			ad_R.hardStop();
			ad_F.hardStop();
			analogWrite(13, 0);
		}
		else
		{
			ad_R.run(FWD, runSpeed);
			ad_F.run(FWD, runSpeed*1.0375);
			analogWrite(13, 120);
		}
	}
	return is_speed_changed;
}

void ExtendFeedArm()
{
	bool armStpOn = false;
	int stepCnt = 0;


	if (!is_armExtended)
	{
		SerialUSB.println("Extending Feed Arm");

		// Wake motor
		digitalWrite(pin.ED_SLP, HIGH);

		// Set arm direction
		digitalWrite(pin.ED_DIR, LOW); // extend

		while (stepCnt < 200)
		{
			if (!armStpOn)
			{
				delay(1);
				digitalWrite(pin.ED_STP, HIGH);
				stepCnt++;
			}
			else
			{
				delay(1);
				digitalWrite(pin.ED_STP, LOW);
			}
			armStpOn = !armStpOn;
		}

		// Unstep motor
		digitalWrite(pin.ED_STP, LOW);

		// Sleep motor
		digitalWrite(pin.ED_SLP, LOW);

		is_armExtended = true;
	}
}

void RetractFeedArm()
{
	bool armStpOn = false;

	if (is_armExtended)
	{
		SerialUSB.println("Extending Feed Arm");

		// Wake motor
		digitalWrite(pin.ED_SLP, HIGH);

		// Set arm direction
		digitalWrite(pin.ED_DIR, HIGH); // retract

		while (digitalRead(pin.FeedSwitch) == HIGH)
		{
			if (!armStpOn)
			{
				delay(1);
				digitalWrite(pin.ED_STP, HIGH);
			}
			else
			{
				delay(1);
				digitalWrite(pin.ED_STP, LOW);
			}
			armStpOn = !armStpOn;
		}

		// Unstep motor
		digitalWrite(pin.ED_STP, LOW);

		// Sleep motor
		digitalWrite(pin.ED_SLP, LOW);

		is_armExtended = false;
	}
}

void CheckBattery()
{
	// Local vars
	static uint32_t t_lastUpdate = millis();
	static uint32_t t_delUpdate = 500;
	float bit_in;
	float volt_in;
	float volt_sum;
	float volt_avg;
	byte bit_out;

	// Make sure relay is open
	if (digitalRead(pin.Rel_EtOH) == LOW)
	{
		digitalWrite(pin.Rel_EtOH, HIGH);
	}

	bit_in = analogRead(pin.BatVolt);
	volt_in = bit_in * bit2volt;
	volt_sum = 0;
	// Shift array and compute average
	for (int i = 99; i > 0; i--) {
		batVoltArr[i] = batVoltArr[i - 1];
		volt_sum += batVoltArr[i];
	}
	batVoltArr[0] = volt_in;
	volt_avg = volt_sum / 99;

	// Convert float to byte
	bit_out = byte(round(volt_avg * 10));

	if (millis() > t_lastUpdate + t_delUpdate)
	{
		char str[50];
		sprintf(str, "\r\nVOLTAGE: Float = %0.2fV Byte = %d", volt_avg, bit_out);
		SerialUSB.print(str);

		t_lastUpdate = millis();
	}
}

void XBeeRead()
{
	byte buff_b;
	char buff_c;
	while (Serial1.available() > 0) {
		buff_b = Serial1.read();
		buff_c = buff_b;
		SerialUSB.print(buff_c);
	}
}

void ConsoleRead()
{
	byte buff_b;
	while (SerialUSB.available() > 0) {
		buff_b = SerialUSB.read();
		Serial1.write(buff_b);
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
			digitalWrite(pin.IRdetect_Relay, HIGH);
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
			digitalWrite(pin.IRdetect_Relay, LOW);
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

void IR_LatComp()
{
	if (is_FeederDue)
	{
		if (is_IR_Rcvd && is_Pls_Rcvd)
		{
			// Turn on tracker LED
			analogWrite(pin.TrackLED, 100);
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
			micros() > t_IR_Rcvd + t_LED_Dur * 1000
			)
		{
			// Turn off tracker LED
			analogWrite(pin.TrackLED, 0);
			is_IR_On = false;
		}
	}
}

void FeedSwitch() {
	if (digitalRead(pin.FeedSwitch) == LOW)
	{

	}
}

void CheckButtons()
{
	// RUN BUTTON 1 OPPERATIONS
	if (digitalRead(pin.Btn[0]) == LOW)
	{
		// Check debounce time
		if (t_debounce[0] > millis()) return;
		t_debounce[0] = millis() + 250;

		// Feed arm test
		if (do_FeedArmTest)
		{
			RetractFeedArm();
		}

		// Solenoid Test
		if (do_SolTest)
		{
			if (digitalRead(pin.Rel_Rew) == LOW)
				digitalWrite(pin.Rel_Rew, HIGH);
			else digitalWrite(pin.Rel_Rew, LOW);
		}

		SerialUSB.println("Button 1");
	}
	// RUN BUTTON 2 OPPERATIONS
	else if (digitalRead(pin.Btn[1]) == LOW)
	{
		// Check debounce time
		if (t_debounce[1] > millis()) return;
		t_debounce[1] = millis() + 250;

		// Feed arm test
		if (do_FeedArmTest)
		{
			ExtendFeedArm();
		}

		// Solenoid Test
		if (do_SolTest)
		{
			if (digitalRead(pin.Rel_EtOH) == LOW)
				digitalWrite(pin.Rel_EtOH, HIGH);
			else digitalWrite(pin.Rel_EtOH, LOW);
		}

		SerialUSB.println("Button 2");
	}
	// RUN BUTTON 3 OPPERATIONS
	else if (digitalRead(pin.Btn[2]) == LOW)
	{
		// Check debounce time
		if (t_debounce[2] > millis()) return;
		t_debounce[2] = millis() + 250;

		// Run function
		if (do_IR_ComTest)
		{
			static bool is_LCD_On = false;
			if (is_LCD_On) { analogWrite(pin.Disp_LED, 25); }
			else { analogWrite(pin.Disp_LED, 0); }
			is_LCD_On = !is_LCD_On;
		}

		SerialUSB.println("Button 3");
	}
	// Exit
	else return;
}

void InteruptIRproxHalt() {
	ad_R.hardStop();
	ad_F.hardStop();
	SerialUSB.println("Run Halted");
}

void SetupBlink()
{
	int duty[2] = { 100, 0 };
	bool isOn = false;
	int del = 100;
	// Flash sequentially
	for (int i = 0; i < 8; i++)
	{
		analogWrite(pin.Disp_LED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin.TrackLED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin.RewLED_R, duty[(int)isOn]);
		delay(del);
		isOn = !isOn;
	}
	// Reset LEDs
	analogWrite(pin.Disp_LED, 0);
	analogWrite(pin.TrackLED, trackLEDduty);
	analogWrite(pin.RewLED_R, rewLEDmin);
}

int CheckAD_Status(uint16_t stat_reg, String stat_id)
{
	// Local vars
	String status_list[16] =
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
	bool bit_set[2] = { false, false };
	uint16_t bit_val = 0x0;

	// Get id ind
	for (int i = 0; i < 16; i++)
	{
		if (stat_id == status_list[i])
		{
			bit_ind[bit_set[0] ? 1 : 0] = i;
			bit_set[bit_set[0] ? 1 : 0] = true;
		}
	}

	// Get bit value
	int n_loop = bit_set[1] ? 2 : 1;
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

void PrintBits(uint16_t stat_reg)
{
	String status_list[16] =
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
	char c[50];
	SerialUSB.println("\r\r");

	for (int i = 0; i < 16; i++)
	{
		int k = i;
		uint16_t mask = 1 << k;
		uint16_t masked_n = stat_reg & mask;
		uint16_t thebit = masked_n >> k;

		sprintf(c, "[%d] ", thebit);
		SerialUSB.println(c + status_list[i]);
	}
	SerialUSB.println("\r");
}

void AD_Reset()
{
	// Reset each axis
	ad_R.resetDev();
	delayMicroseconds(100);
	ad_F.resetDev();
	delayMicroseconds(100);
	// Configure each axis
	dSPINConfig_board();
	delayMicroseconds(100);
	ad_R.getStatus();
	delayMicroseconds(100);
	ad_F.getStatus();
	delayMicroseconds(100);
}
