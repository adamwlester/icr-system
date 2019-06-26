#pragma region ---------DEBUG SETTINGS---------

// POT Test 
/*
Connect pot and use to test motor function
*/
const bool do_POT_Test = false;
const bool do_POT_PrintSpeed = false;
const bool do_POT_PrintStat = false;

// Stepper Rotation Test
const bool do_Step_Rot_Test = false;
const bool do_scale_speed_correction = false;
const bool do_lin_speed_correction = false;
const double deltaSpeed = 5;
const double speedRange[2] = { 5,90 };
const int stepsPerSpeed = 3;
const int accRots = 1;

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
USARTClass &c2rPort = Serial2;
USARTClass &a2rPort = Serial3;

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
#include "FeederDue_PinMap.h"
//
#include <string.h>

// AutoDriver

#include <SPI.h>
//
#include "AutoDriver_Due.h"

// Pixy

#include <Wire.h>
//
#include <PixyI2C.h>

// LCD
#include <LCD5110_Graph.h>

// TinyEKF
#define N 4     // States
#define M 6     // Measurements
#include <TinyEKF.h>

#pragma endregion 


#pragma region ---------PIN DECLARATION---------

// HardwareDiagnostic pins
struct PINS
{
	// POT switch test
	int POT_GRN = A10;
	int POT_IN = A9;
	int POT_VCC = A8;

	//// Stepper Rotation Test
	int SWITCH_WHEEL_R = A8;
	int SWITCH_WHEEL_F = A10;
	int SWITCH_WHEEL_GRN_R = A9;
	int SWITCH_WHEEL_GRN_F = A11;
	//// Stepper Rotation Test
	//int SWITCH_WHEEL_R = A7;
	//int SWITCH_WHEEL_F = A5;
	//int SWITCH_WHEEL_GRN_R = A6;
	//int SWITCH_WHEEL_GRN_F = A4;

	// IR detector test
	int IR_DETECT_Relay = 51;
	int IR_DETECT_PlsIn = 24;
	int CheetahDueID_GRN = A8;
	int CheetahDueID_IP = A9;
}
// Initialize
pins;

#pragma endregion


#pragma region ---------VARIABLE SETUP---------

// AutoDriver
const float cm2stp = 200 / (9 * PI);
const float stp2cm = (9 * PI) / 200;
const float maxSpeed = 100; // (cm) 
const float maxAcc = 80; // (cm) 
const float maxDec = 160; // (cm)
const double scaleSpeedRear = 0.955556;
const double scaleSpeedFront = 0.997604;
const double regSpeedRear[2] = { 0.953122147782673, 0.080943539773743 };
const double regSpeedFront[2] = { 0.998759337302555, 0.010702522395395 };
const double rearMotCoeff[5] = {
	0.000000044120830,
	-0.000007753772088,
	0.000418793299060,
	0.945620729817402,
	0.047445535533065,
};
const double frontMotCoeff[5] = {
	0.000000032971731,
	-0.000006928907732,
	0.000457085441358,
	0.972947848949920,
	0.021557249590414,
};

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

// Stepper Rotation Variables
volatile int v_cnt_Step_R = 0;
volatile int v_cnt_Step_F = 0;
volatile uint32_t t_debounce_R = 0;
volatile uint32_t t_debounce_F = 0;
volatile bool v_do_step_interupt_R = false;
volatile bool v_do_step_interupt_F = false;
volatile int v_stepMax = 0;
volatile uint32_t v_t_start_R = 0;
volatile uint32_t v_t_start_F = 0;
volatile uint32_t v_t_end_R = 0;
volatile uint32_t v_t_end_F = 0;
volatile uint32_t v_dt_debounce = 0;

// Button variables
volatile uint32_t t_debounce[3] = { millis(), millis(), millis() };

// Voltage test variables
float bit2volt = 0.01545; // 0.0164
float bit2ic = 0.00479;
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
AutoDriver_Due AD_R(pin.AD_CSP_R, pin.AD_RST);
AutoDriver_Due AD_F(pin.AD_CSP_F, pin.AD_RST);

// LCD
LCD5110 LCD(pin.LCD_CS, pin.LCD_RST, pin.LCD_DC, pin.LCD_MOSI, pin.LCD_SCK);
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

#pragma endregion


void setup()
{

	delayMicroseconds(100);

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// XBee 1a (to/from CS)
	c2rPort.begin(57600);

	// XBee 1b (to/from CheetahDue)
	a2rPort.begin(57600);

	// SETUP PINS
	SetupPins();

	// TURN ON POWER
	digitalWrite(pin.PWR_OFF, LOW);
	delayMicroseconds(100);
	digitalWrite(pin.PWR_ON, HIGH);
	delayMicroseconds(100);
	digitalWrite(pin.PWR_ON, LOW);
	delayMicroseconds(100);

	// ENABLE VOLTGAGE REGULATORS
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_12V2_ENBLE, HIGH);
	digitalWrite(pin.REG_5V1_ENBLE, HIGH);

	// INITIALIZE LCD
	LCD.InitLCD();
	LCD.setFont(SmallFont);
	LCD.invert(true);

	// SETUP AUTODRIVER

	// Configure SPI
	AD_R.SPIConfig();
	delayMicroseconds(100);
	AD_F.SPIConfig();
	delayMicroseconds(100);
	// Reset 
	AD_Reset(maxSpeed, maxAcc, maxDec);

	// Make sure motor is stopped and in high impedance
	AD_R.hardHiZ();
	AD_F.hardHiZ();

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

	// TEST STUFF SETUP

	// POT vcc and Gnd
	if (do_POT_Test) {
		pinMode(pins.POT_GRN, OUTPUT);
		pinMode(pins.POT_VCC, OUTPUT);
		digitalWrite(pins.POT_VCC, HIGH);
		digitalWrite(pins.POT_GRN, LOW);
		attachInterrupt(digitalPinToInterrupt(pin.IRPROX_R), InteruptIRproxHalt, FALLING);
		attachInterrupt(digitalPinToInterrupt(pin.IRPROX_L), InteruptIRproxHalt, FALLING);
	}

	// Stepper Rotation Test
	if (do_Step_Rot_Test) {

		// Setup pins
		pinMode(pins.SWITCH_WHEEL_R, INPUT_PULLUP);
		pinMode(pins.SWITCH_WHEEL_F, INPUT_PULLUP);
		pinMode(pins.SWITCH_WHEEL_GRN_R, OUTPUT);
		pinMode(pins.SWITCH_WHEEL_GRN_F, OUTPUT);
		digitalWrite(pins.SWITCH_WHEEL_GRN_R, LOW);
		digitalWrite(pins.SWITCH_WHEEL_GRN_F, LOW);

		// Setup interupts
		attachInterrupt(digitalPinToInterrupt(pins.SWITCH_WHEEL_R), InteruptSWITCH_WHEEL_R, FALLING);
		attachInterrupt(digitalPinToInterrupt(pins.SWITCH_WHEEL_F), InteruptSWITCH_WHEEL_F, FALLING);
	}

	// Initialize bat volt array
	for (int i = 0; i < 100; i++) {
		batVoltArr[i] = 0;
	}


	if (do_IR_ComTest)
	{
		pinMode(pins.CheetahDueID_IP, INPUT_PULLUP);
		pinMode(pins.CheetahDueID_GRN, OUTPUT);
		digitalWrite(pins.CheetahDueID_GRN, LOW);
		delay(10);
		// Detect if code is running on FeederDue
		if (digitalRead(pins.CheetahDueID_IP) == HIGH)
		{
			is_FeederDue = true;
			attachInterrupt(digitalPinToInterrupt(pin.INTERUPT_IR_DETECT), Interupt_IR_Detect, HIGH);
			attachInterrupt(digitalPinToInterrupt(pins.IR_DETECT_PlsIn), Interupt_Pulse_Detect, HIGH);
			SerialUSB.println("Running on FeederDue");
		}
		else
		{
			// Set all relays low on CheetahDue
			pinMode(pins.IR_DETECT_Relay, OUTPUT);
			pinMode(11, OUTPUT);
			pinMode(12, OUTPUT);
			digitalWrite(pins.IR_DETECT_Relay, LOW);
			digitalWrite(11, LOW);
			digitalWrite(12, LOW);
			SerialUSB.println("Running on CheetahDue");
		}
		pinMode(pins.CheetahDueID_IP, INPUT);
		pinMode(pins.CheetahDueID_GRN, INPUT);
	}

	// PRINT TEST

	// POT Run Test
	if (do_POT_Test) {
		SerialUSB.println("RUNNING \"POT Run Test\"");
	}

	// Stepper Rotation Test
	if (do_Step_Rot_Test) {
		SerialUSB.println("RUNNING \"Stepper Rotation Test\"");
	}

	// Feed Arm Test
	if (do_FeedArmTest) {
		SerialUSB.println("RUNNING \"Feed Arm Test\"");
	}

	// Volt Test 
	if (do_VoltTest) {
		SerialUSB.println("RUNNING \"Volt Test\"");
	}

	// XBee Test 
	if (do_XBeeTest) {
		SerialUSB.println("RUNNING \"XBee Test\"");
		SerialUSB.println("NOTE: Precede commands with \'c\'\\\'a\' for r2c\\r2a coms");
	}

	// Solenoid Test 
	if (do_SolTest) {
		SerialUSB.println("RUNNING \"Solenoid Test\"");
	}

	// IR Detector Timing Test 
	if (do_IR_ComTest) {
		SerialUSB.println("RUNNING \"IR Detector Timing Test\"");
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
			ad_r_stat = AD_R.getStatus();
			int ocd_r = CheckAD_Status(ad_r_stat, "OCD");
			int mot_r = CheckAD_Status(ad_r_stat, "MOT_STATUS");
			// Get front board status
			ad_f_stat = AD_F.getStatus();
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
				sprintf(msg, "!!ERROR!! AD: R_MOT=%d F_MOT=%d R_OCD=%d F_OCD=%d", mot_r, mot_f, ocd_r, ocd_f);
				SerialUSB.println(msg);
				pastSpeed = 0;
				//AD_Reset();
			}
			t_check = millis() + 100;
		}
	}

	// Stepper Rotation Test
	if (do_Step_Rot_Test)
	{
		StepRun();
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

	vccNow = analogRead(pins.POT_IN);
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
			AD_R.hardStop();
			AD_F.hardStop();
			analogWrite(13, 0);
		}
		else
		{
			//AD_R.run(FWD, runSpeed*scaleSpeedRear);
			//AD_F.run(FWD, runSpeed*scaleSpeedFront);
			//AD_R.run(FWD, runSpeed*scaleSpeedRear);
			//AD_F.run(FWD, runSpeed*scaleSpeedFront);
			double speed_R =
				rearMotCoeff[0] * (velNow * velNow * velNow * velNow) +
				rearMotCoeff[1] * (velNow * velNow * velNow) +
				rearMotCoeff[2] * (velNow * velNow) +
				rearMotCoeff[3] * velNow +
				rearMotCoeff[4];
			double speed_F =
				frontMotCoeff[0] * (velNow * velNow * velNow * velNow) +
				frontMotCoeff[1] * (velNow * velNow * velNow) +
				frontMotCoeff[2] * (velNow * velNow) +
				frontMotCoeff[3] * velNow +
				frontMotCoeff[4];
			double runSpeed_R = speed_R * cm2stp;
			double runSpeed_F = speed_F * cm2stp;
			AD_R.run(FWD, runSpeed_R);
			AD_F.run(FWD, runSpeed_F);

			// Turn on lcd
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

		while (digitalRead(pin.SWITCH_DISH) == HIGH)
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

void StepRun()
{
	// Local vars
	char str[200] = { 0 };
	int cnt_step_R = 0;
	int cnt_step_F = 0;
	int n_speed_steps = 0;
	double speed_R = 0;
	double speed_F = 0;
	double speed_step_arr[100] = { 0 };
	double speed_arr_R[100] = { 0 };
	double speed_arr_F[100] = { 0 };
	double ratio_arr_R[100] = { 0 };
	double ratio_arr_F[100] = { 0 };
	double ratio_sum = 0;
	double ratio_avg_R = 0;
	double ratio_avg_F = 0;
	double runSpeed = 0;
	double runSteps = 0;
	int step_last_R = 0;
	int step_last_F = 0;
	bool is_stopped_R = false;
	bool is_stopped_F = false;
	uint32_t t_step_r = 0;
	uint32_t t_step_f = 0;

	// Get speed steps
	n_speed_steps =
		(speedRange[1] - speedRange[0]) / deltaSpeed + 1;
	double s = speedRange[0];
	for (int i = 0; i < n_speed_steps; i++)
	{
		speed_step_arr[i] = s;
		s += deltaSpeed;
	}

	// Print speed steps
	SerialUSB.println("\r\nvel = [ ...");
	for (int i = 0; i < n_speed_steps; i++)
	{
		sprintf(str, "%0.2f, ...", speed_step_arr[i]);
		SerialUSB.println(str);
	}
	SerialUSB.println("];");

	// Loop through speed steps
	for (int i = 0; i < n_speed_steps; i++)
	{
		// Reset counters
		v_cnt_Step_R = 0;
		v_cnt_Step_F = 0;

		// Align wheels
		SerialUSB.println("\r\nResseting Position");
		runSteps = 10 * cm2stp;
		AD_R.run(FWD, runSteps);
		while (digitalRead(pins.SWITCH_WHEEL_R) == HIGH);
		while (digitalRead(pins.SWITCH_WHEEL_R) == LOW);
		delay(100);
		AD_R.hardStop();
		AD_F.run(FWD, runSteps);
		while (digitalRead(pins.SWITCH_WHEEL_F) == HIGH);
		while (digitalRead(pins.SWITCH_WHEEL_F) == LOW);
		delay(100);
		AD_F.hardStop();

		// Get settings
		runSpeed = speed_step_arr[i];
		runSteps = runSpeed * cm2stp;
		v_stepMax = stepsPerSpeed + accRots;
		v_dt_debounce = ((1 / (runSpeed / (9 * PI))) / 8) * 1000;

		// Print new speed
		sprintf(str, "[StepRun] Run Settings: Speed=%0.2fcm/sec Steps=%d DT_Debounce=%d", runSpeed, stepsPerSpeed, v_dt_debounce);
		SerialUSB.println(str);
		delay(500);

		// Reset timers and flags
		v_t_start_R = 0;
		v_t_start_F = 0;
		v_t_end_R = 0;
		v_t_end_F = 0;
		step_last_R = 0;
		step_last_F = 0;
		is_stopped_R = false;
		is_stopped_F = false;

		// Run motor
		if (do_scale_speed_correction) {
			AD_R.run(FWD, runSteps*scaleSpeedRear);
			AD_F.run(FWD, runSteps*scaleSpeedFront);
		}
		else if (do_lin_speed_correction) {
			AD_R.run(FWD, runSteps*regSpeedRear[0] + regSpeedRear[1]);
			AD_F.run(FWD, runSteps*regSpeedFront[0] + regSpeedFront[1]);
		}
		else {
			AD_R.run(FWD, runSteps);
			AD_F.run(FWD, runSteps);
		}
		t_step_r = millis();
		t_step_f = millis();


		// Enable interupts
		delay(10);
		v_do_step_interupt_R = true;
		v_do_step_interupt_F = true;

		// Wait for time to ellapse
		while (!is_stopped_R || !is_stopped_F) {

			////TEMP
			//static uint32_t t_print = millis() + 100;
			//if (millis() > t_print) {
			//	sprintf(str, "Rear=%s Front=%s", digitalRead(pins.SWITCH_WHEEL_R) ? "HIGH" : "LOW", digitalRead(pins.SWITCH_WHEEL_F) ? "HIGH" : "LOW");
			//	SerialUSB.println(str);
			//	t_print = millis() + 100;
			//}

			// Track steps
			if (step_last_R != v_cnt_Step_R) {
				step_last_R = v_cnt_Step_R;
				sprintf(str, "   rear step %d dt=%d", step_last_R, millis()- t_step_r);
				SerialUSB.println(str);
				t_step_r = millis();
			}
			if (step_last_F != v_cnt_Step_F) {
				step_last_F = v_cnt_Step_F;
				sprintf(str, "   front step %d dt=%d", step_last_F, millis() - t_step_f);
				SerialUSB.println(str);
				t_step_f = millis();
			}

			// Stop when done
			if (!is_stopped_R && !v_do_step_interupt_R) {
				AD_R.hardStop();
				is_stopped_R = true;
				SerialUSB.println("Hard Stop Rear");
			}
			if (!is_stopped_F && !v_do_step_interupt_F) {
				AD_F.hardStop();
				is_stopped_F = true;
				SerialUSB.println("Hard Stop Front");
			}
		}
		delay(100);

		// Remove extra step
		cnt_step_R = v_cnt_Step_R - accRots;
		cnt_step_F = v_cnt_Step_F - accRots;

		// Get speed
		speed_R = (cnt_step_R * (9 * PI)) / ((double)(v_t_end_R - v_t_start_R) / 1000);
		speed_F = (cnt_step_F * (9 * PI)) / ((double)(v_t_end_F - v_t_start_F) / 1000);
		speed_arr_R[i] = speed_R;
		speed_arr_F[i] = speed_F;

		// Print rotations
		ratio_arr_R[i] = (runSteps / cm2stp) / speed_R;
		ratio_arr_F[i] = (runSteps / cm2stp) / speed_F;
		sprintf(str, "[StepRun] Steps: Rear=%d Front=%d", cnt_step_R, cnt_step_F);
		SerialUSB.println(str);
		sprintf(str, "[StepRun] Speed: Rear=%0.4fcm/sec Front=%0.4fcm/sec", speed_R, speed_F);
		SerialUSB.println(str);
		sprintf(str, "[StepRun] Ratio: Rear=%0.4fcm/sec Front=%0.4fcm/sec", ratio_arr_R[i], ratio_arr_F[i]);
		SerialUSB.println(str);

	}

	// Print speed settings
	SerialUSB.println("\r\nvel = [ ...");
	for (int i = 0; i < n_speed_steps; i++)
	{
		sprintf(str, "%0.2f, ...", speed_step_arr[i]);
		SerialUSB.println(str);
	}
	SerialUSB.println("];");

	// Print rear speeds 
	SerialUSB.println("\r\nr_vel = [ ...");
	for (int i = 0; i < n_speed_steps; i++)
	{
		sprintf(str, "%0.6f, ...", speed_arr_R[i]);
		SerialUSB.println(str);
	}
	SerialUSB.println("];");

	// Print front speeds 
	SerialUSB.println("\r\nf_vel = [ ...");
	for (int i = 0; i < n_speed_steps; i++)
	{
		sprintf(str, "%0.6f, ...", speed_arr_F[i]);
		SerialUSB.println(str);
	}
	SerialUSB.println("];");

	// Get rear ratio average 
	SerialUSB.println("\r\nr_ratio = [ ...");
	ratio_sum = 0;
	for (int i = 0; i < n_speed_steps; i++)
	{
		ratio_sum += ratio_arr_R[i];
		sprintf(str, "%0.6f, ...", ratio_arr_R[i]);
		SerialUSB.println(str);
	}
	SerialUSB.println("];");
	ratio_avg_R = ratio_sum / n_speed_steps;

	// Get front ratio average 
	SerialUSB.println("\r\nf_ratio = [ ...");
	ratio_sum = 0;
	for (int i = 0; i < n_speed_steps; i++)
	{
		ratio_sum += ratio_arr_F[i];
		sprintf(str, "%0.6f, ...", ratio_arr_F[i]);
		SerialUSB.println(str);
	}
	SerialUSB.println("];");
	ratio_avg_F = ratio_sum / n_speed_steps;

	// Print average
	sprintf(str, "\r\n[StepRun] Speed Ratios Average: Rear=%0.6f Front=%0.6f", ratio_avg_R, ratio_avg_F);
	SerialUSB.println(str);

	// Hold here
	while (true);
}

void CheckBattery()
{
	// Local vars
	static uint32_t t_lastUpdate = millis();
	static int dt_update = 500;
	static uint32_t t_relOpen = 0;
	static int dt_relOpen = 5000;
	uint32_t vcc_bit_in;
	uint32_t ic_bit_in;
	float volt_in;
	float volt_sum;
	float volt_avg;
	byte vcc_bit_out;

	// Make sure relay is open
	//if (digitalRead(pin.REL_VCC) == LOW) {
	//	digitalWrite(pin.REL_VCC, HIGH);
	//}

	if (digitalRead(pin.REL_VCC) == LOW &&
		millis() > t_relOpen + dt_relOpen * 2)
	{
		digitalWrite(pin.REL_VCC, HIGH);
		t_relOpen = millis();
		char str[200];
		sprintf(str, "\r\nVOLTAGE RELAY OPEN", volt_avg, vcc_bit_out);
		SerialUSB.print(str);
	}
	else if (digitalRead(pin.REL_VCC) == HIGH &&
		millis() > t_relOpen + dt_relOpen)
	{
		digitalWrite(pin.REL_VCC, LOW);
		char str[200];
		sprintf(str, "\r\nVOLTAGE RELAY CLOSED", volt_avg, vcc_bit_out);
		SerialUSB.print(str);
	}

	// Calculate voltage
	vcc_bit_in = analogRead(pin.BAT_VCC);
	volt_in = (float)vcc_bit_in * bit2volt;

	// Shift array and compute average
	volt_sum = 0;
	for (int i = 99; i > 0; i--) {
		batVoltArr[i] = batVoltArr[i - 1];
		volt_sum += batVoltArr[i];
	}
	batVoltArr[0] = volt_in;
	volt_avg = volt_sum / 99;

	// Convert float to byte
	vcc_bit_out = byte(round(volt_avg * 10));

	if (millis() > t_lastUpdate + dt_update)
	{
		char str[200];
		sprintf(str, "\r\nVOLTAGE & CURRENT: Vcc=%0.2fV Vcc_Bit_In=%d Vcc_Bit_Out=%d",
			volt_avg, vcc_bit_in, vcc_bit_out);
		SerialUSB.print(str);

		t_lastUpdate = millis();
	}
}

void XBeeRead()
{
	byte buff_b;
	char buff_c;
	while (c2rPort.available() > 0) {
		buff_b = c2rPort.read();
		buff_c = buff_b;
		SerialUSB.print(buff_c);
	}
	while (a2rPort.available() > 0) {
		buff_b = a2rPort.read();
		buff_c = buff_b;
		SerialUSB.print(buff_c);
	}
}

void ConsoleRead()
{
	byte buff_b;
	char targ = ' ';
	while (SerialUSB.available() > 0) {
		buff_b = SerialUSB.read();
		if (targ == ' ') {
			targ = buff_b;
		}
		SerialUSB.write(buff_b);
		if (targ == 'c') {
			c2rPort.write(buff_b);
		}
		else if (targ == 'a') {
			a2rPort.write(buff_b);
		}
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
			digitalWrite(pins.IR_DETECT_Relay, HIGH);
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
			digitalWrite(pins.IR_DETECT_Relay, LOW);
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
		//char str[200];
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
		//char str[200];
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
			analogWrite(pin.LED_TRACKER, 100);
			// Compute delay between pulse and IR
			t_IR_Lat = t_IR_Rcvd - t_Pls_Rcvd;
			// Print info
			char str[200];
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
			analogWrite(pin.LED_TRACKER, 0);
			is_IR_On = false;
		}
	}
}

void SWITCH_DISH() {
	if (digitalRead(pin.SWITCH_DISH) == LOW)
	{

	}
}

void CheckButtons()
{
	// RUN BUTTON 1 OPPERATIONS
	if (digitalRead(pin.BTN_1) == LOW)
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
			if (digitalRead(pin.REL_FOOD) == LOW)
				digitalWrite(pin.REL_FOOD, HIGH);
			else digitalWrite(pin.REL_FOOD, LOW);
		}

		SerialUSB.println("Button 1");
	}
	// RUN BUTTON 2 OPPERATIONS
	else if (digitalRead(pin.BTN_2) == LOW)
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
			if (digitalRead(pin.REL_ETOH) == LOW)
				digitalWrite(pin.REL_ETOH, HIGH);
			else digitalWrite(pin.REL_ETOH, LOW);
		}

		SerialUSB.println("Button 2");
	}
	// RUN BUTTON 3 OPPERATIONS
	else if (digitalRead(pin.BTN_3) == LOW)
	{
		// Check debounce time
		if (t_debounce[2] > millis()) return;
		t_debounce[2] = millis() + 250;

		// Run function
		if (do_IR_ComTest)
		{
			static bool is_LCD_On = false;
			if (is_LCD_On) { analogWrite(pin.LCD_LED, 25); }
			else { analogWrite(pin.LCD_LED, 0); }
			is_LCD_On = !is_LCD_On;
		}

		SerialUSB.println("Button 3");
	}
	// Exit
	else return;
}

void InteruptSWITCH_WHEEL_R()
{
	// Bail if blocking
	if (!v_do_step_interupt_R) {
		return;
	}

	// Exit if < debounce time has not passed
	if (t_debounce_R + v_dt_debounce > millis()) {
		
		// Reset debounce time
		t_debounce_R = millis();
		return;
	}

	// Reset debounce time
	t_debounce_R = millis();

	// Add to count
	v_cnt_Step_R++;

	// Check if one rotation completed
	if (v_cnt_Step_R == accRots && v_t_start_R == 0) {
		v_t_start_R = millis();
	}

	// Check if steps reached 
	if (v_cnt_Step_R == v_stepMax) {
		v_t_end_R = millis();
		v_do_step_interupt_R = false;
	}
}

void InteruptSWITCH_WHEEL_F()
{
	// Bail if blocking
	if (!v_do_step_interupt_F) {
		return;
	}

	// Exit if < debounce time has not passed
	if (t_debounce_F + v_dt_debounce > millis()) {

		// Reset debounce time
		t_debounce_F = millis();
		return;
	}

	// Reset debounce time
	t_debounce_F = millis();

	// Add to count
	v_cnt_Step_F++;

	// Check if one rotation completed
	if (v_cnt_Step_F == accRots && v_t_start_F == 0) {
		v_t_start_F = millis();
	}

	// Check if steps reached 
	if (v_cnt_Step_F == v_stepMax) {
		v_t_end_F = millis();
		v_do_step_interupt_F = false;
	}
}

void InteruptIRproxHalt() {
	AD_R.hardStop();
	AD_F.hardStop();
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
		analogWrite(pin.LCD_LED, duty[(int)isOn]);
		delay(del);
		analogWrite(pin.LED_TRACKER, duty[(int)isOn]);
		delay(del);
		analogWrite(pin.LED_REW_R, duty[(int)isOn]);
		delay(del);
		isOn = !isOn;
	}
	// Reset LEDs
	analogWrite(pin.LCD_LED, 0);
	analogWrite(pin.LED_TRACKER, trackLEDduty);
	analogWrite(pin.LED_REW_R, rewLEDmin);
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

void AD_Config(float max_speed, float max_acc, float max_dec)
{
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

void AD_Reset(float max_speed, float max_acc, float max_dec)
{
	// Reset each axis
	AD_R.resetDev();
	AD_F.resetDev();
	delayMicroseconds(100);

	// Configure each axis
	AD_Config(max_speed, max_acc, max_dec);
	delayMicroseconds(100);
	AD_R.getStatus();
	AD_F.getStatus();
	delayMicroseconds(100);
}
