//-------CheetahDue-------

// SOFTWARE RESET
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#pragma region ---------PIN SETUP---------

// DEFINE PINS
const int pin_LED = 13;
const int pin_relWhiteNoise = 12;
const int pin_relRewTone = 11;

// NLX TTL

// PT
const int pin_ttlNorthOn = A7;
const int pin_ttlWestOn = A6;
const int pin_ttlSouthOn = A5;
const int pin_ttlEastOn = A4;
const int pin_ptNorthOn = A3;
const int pin_ptWestOn = A2;
const int pin_ptSouthOn = A1;
const int pin_ptEastOn = A0;

// Noise/rew
const int pin_ttlWhiteNoise = 25;
const int pin_ttlRewTone = 26;
const int pin_ttlRewOn = 27;
const int pin_ttlRewOff = 28;

// PID
const int pin_ttlPidRun = 24;
const int pin_ttlPidStop = 31;

// Bulldozer
const int pin_ttlBullRun = 29;
const int pin_ttlBullStop = 30;


#pragma endregion

#pragma region ---------VARIABLE DECLARATION---------

// PHOTO TRANSDUCER VARS

// north vars
volatile uint32_t t_inLastNorth = millis();
volatile uint32_t t_outLastNorth = millis();
volatile bool isOnNorth = false;
// west vars
volatile uint32_t t_inLastWest = millis();
volatile uint32_t t_outLastWest = millis();
volatile bool isOnWest = false;
// south 
volatile uint32_t t_inLastSouth = millis();
volatile uint32_t t_outLastSouth = millis();
volatile bool isOnSouth = false;
// east vars
volatile uint32_t t_inLastEast = millis();
volatile uint32_t t_outLastEast = millis();
volatile bool isOnEast = false;

// TTL timers
uint32_t t_debounce = 10; // (ms)
uint32_t t_pulseWdth = 50; // (ms)

// Flow control
bool doPrintFlow = false;
bool doPrintRcvdPack = false;
bool fc_isSesStarted = false;
bool fc_doWhiteNoise = false;
bool fc_doRewTone = false;
bool fc_isRewarding = false;

// Serial
uint32_t t_sync = 0;
char msg_id = ' ';
byte msg_dat;
uint16_t packNow;
const char r2a_id[10] = {
	't', // set sync time
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
};
char rob2ard_head = '[';
char rob2ard_foot = ']';
bool msg_pass = false;

// Reward
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;

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


// ---------SETUP---------
void setup()
{

	//while (!SerialUSB);
	PrintState("SETUP");

	// XBee.
	Serial1.begin(57600);

	// Set pin direction
	pinMode(pin_ptNorthOn, INPUT);
	pinMode(pin_ptWestOn, INPUT);
	pinMode(pin_ptSouthOn, INPUT);
	pinMode(pin_ptEastOn, INPUT);
	pinMode(pin_ttlNorthOn, OUTPUT);
	pinMode(pin_ttlWestOn, OUTPUT);
	pinMode(pin_ttlSouthOn, OUTPUT);
	pinMode(pin_ttlEastOn, OUTPUT);

	// Set initial output state
	digitalWrite(pin_ttlNorthOn, LOW);
	digitalWrite(pin_ttlWestOn, LOW);
	digitalWrite(pin_ttlSouthOn, LOW);
	digitalWrite(pin_ttlEastOn, LOW);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(pin_ptNorthOn), NorthInterrupt, RISING); // north
	attachInterrupt(digitalPinToInterrupt(pin_ptWestOn), WestInterrupt, RISING); // west
	attachInterrupt(digitalPinToInterrupt(pin_ptSouthOn), SouthInterrupt, RISING); // south
	attachInterrupt(digitalPinToInterrupt(pin_ptEastOn), EastInterrupt, RISING); // east

	// set other output pins
	pinMode(pin_LED, OUTPUT);
	pinMode(pin_relWhiteNoise, OUTPUT);
	pinMode(pin_ttlWhiteNoise, OUTPUT);
	pinMode(pin_relRewTone, OUTPUT);
	pinMode(pin_ttlRewTone, OUTPUT);
	pinMode(pin_ttlRewOn, OUTPUT);
	pinMode(pin_ttlRewOff, OUTPUT);
	pinMode(pin_ttlBullRun, OUTPUT);
	pinMode(pin_ttlBullStop, OUTPUT);
	pinMode(pin_ttlPidRun, OUTPUT);
	pinMode(pin_ttlPidStop, OUTPUT);

	// set noise off
	digitalWrite(pin_relRewTone, LOW);
	digitalWrite(pin_relWhiteNoise, HIGH);
}


// ---------MAIN LOOP---------
void loop()
{
	// Check for XBee input
	msg_pass = false;
	if (Serial1.available() > 0) {
		msg_pass = ParseSerial();
	}

	// Check for sync time indicating session starting
	if (!fc_isSesStarted)
	{
		if (msg_pass && msg_id == 't')
		{
			// Set sync time and session start
			t_sync = millis();
			fc_isSesStarted = true;
			PrintState("START SESSION");
		}
	}
	else
	{
		// Exicute input
		if (msg_pass)
		{

			// Run reward tone
			if (msg_id == 'r') {

				// Get reward duration and convert to ms
				rewDur = (uint32_t)msg_dat * 10;

				// Run tone
				StartRew();
			}

			// Get noise settings
			else if (msg_id == 's') {
				// No noise
				if (msg_dat == 0)
				{
					fc_doWhiteNoise = false;
					fc_doRewTone = false;
					PrintState("WHITE OFF");
				}
				// White noise only
				else if (msg_dat == 1)
				{
					// set white noise pins
					digitalWrite(pin_relWhiteNoise, HIGH);
					digitalWrite(pin_ttlWhiteNoise, HIGH);
					fc_doWhiteNoise = true;
					fc_doRewTone = false;
					PrintState("WHITE ON");
				}
				// White and reward sound
				else if (msg_dat == 2)
				{
					// set white noise pins
					digitalWrite(pin_relWhiteNoise, HIGH);
					digitalWrite(pin_ttlWhiteNoise, HIGH);
					fc_doWhiteNoise = true;
					fc_doRewTone = true;
					PrintState("WHITE AND TONE ON");
				}
			}

			// Signal PID mode
			else if (msg_id == 'p') {
				// Signal PID stopped
				if (msg_dat == 0)
				{
					digitalWrite(pin_ttlPidRun, LOW);
					digitalWrite(pin_ttlPidStop, HIGH);
					PrintState("PID STOPPED");
				}
				// Signal PID running
				else if (msg_dat == 1)
				{
					digitalWrite(pin_ttlPidStop, LOW);
					digitalWrite(pin_ttlPidRun, HIGH);
					PrintState("PID RUNNING");
				}
			}

			// Signal bull running
			else if (msg_id == 'b') {
				// Signal Bull stopped
				if (msg_dat == 0)
				{
					digitalWrite(pin_ttlBullRun, LOW);
					digitalWrite(pin_ttlBullStop, HIGH);
					PrintState("BULL STOPPED");
				}
				// Signal Bull running
				else if (msg_dat == 1)
				{
					digitalWrite(pin_ttlBullStop, LOW);
					digitalWrite(pin_ttlBullRun, HIGH);
					PrintState("BULL RUNNING");
				}
			}

			// Quite and reset
			else if (msg_id == 'q') {
				PrintState("QUITING");
				// Run bleep bleep
				QuitBleep();
				fc_isSesStarted = false;
				// Restart Arduino
				REQUEST_EXTERNAL_RESET;
			}

		}

		// Check for rew end
		if (fc_isRewarding && millis() > t_rewEnd)
		{
			EndRew();
		}

		// Set output pins back to low
		ResetTTL();

	}
}


#pragma region --------COMMUNICATION---------
// PARSE SERIAL INPUT
bool ParseSerial()
{

	static char head;
	static char foot;
	static bool pass;

	// Dump data till header byte is reached
	while (Serial1.peek() != rob2ard_head && Serial1.available() > 0)
	{
		Serial1.read(); // dump
	}

	// Save header
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	head = u.c[0];

	// Check header
	if (head != rob2ard_head) {
		// mesage will be dumped
		return pass = false;
	}

	// Wait for id packet
	while (Serial1.available() < 1);
	// get id
	u.b[1] = Serial1.read();
	msg_id = u.c[1];

	// Wait for data packet
	while (Serial1.available() < 1);
	// get id
	msg_dat = Serial1.read();

	// Wait for pack number packet
	while (Serial1.available() < 2);
	// get packet num
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	u.b[1] = Serial1.read();
	packNow = u.i16[0];

	// Wait for foot packet
	while (Serial1.available() < 1);

	// Check for footer
	u.b[2] = Serial1.read();
	foot = u.c[2];

	// Footer missing
	if (foot != rob2ard_foot) {
		// mesage will be dumped
		return pass = false;
	}
	else
	{
		PrintRcvdPack(msg_id, packNow);
		return pass = true;
	}

}

#pragma endregion


#pragma region --------REWARD---------

// START REWARD
void StartRew()
{
	// Tell NLX reward on
	digitalWrite(pin_ttlRewOff, LOW);
	digitalWrite(pin_ttlRewOn, HIGH);

	if (fc_doRewTone)
	{
		// set rew pins
		digitalWrite(pin_relRewTone, HIGH);
		digitalWrite(pin_ttlRewTone, HIGH);
		// set white noise pins
		digitalWrite(pin_relWhiteNoise, LOW);
		digitalWrite(pin_ttlWhiteNoise, LOW);
	}
	// Set rew off time
	t_rewEnd = millis() + rewDur;
	fc_isRewarding = true;
	char str[50];
	sprintf(str, "REWARDING(%dms)...", rewDur);
	PrintState(str);
}

// END REWARD
void EndRew()
{
	// Tell NLX reward off
	digitalWrite(pin_ttlRewOn, LOW);
	digitalWrite(pin_ttlRewOff, HIGH);

	if (fc_doRewTone)
	{
		// set white noise pins
		digitalWrite(pin_relWhiteNoise, HIGH);
		digitalWrite(pin_ttlWhiteNoise, HIGH);
		// set rew pins
		digitalWrite(pin_relRewTone, LOW);
		digitalWrite(pin_ttlRewTone, LOW);
	}
	fc_isRewarding = false;
	PrintState("REWARD OFF");
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// PRINT STATUS
void PrintState(String str)
{
	if (doPrintFlow)
	{
		char msg[50];
		uint32_t ts;
		float t;

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);

		// print
		sprintf(msg, " (%0.3fsec)\n\n", t);
		SerialUSB.print('\n');
		SerialUSB.print(str);
		SerialUSB.print(msg);
	}
}

// PRINT RECIEVED PACKET
void PrintRcvdPack(char id, uint16_t pack)
{

	//// Print
	if (doPrintRcvdPack)
	{
		char str[50];
		static uint32_t t1 = millis();
		uint32_t ts;
		uint32_t dt;
		float t;

		// Print

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);
		dt = millis() - t1;

		// Print specific pack contents
		sprintf(str, "---Rcvd: [id:%c pack:%d (dt:%dms tot:%0.3fsec)]\n", id, pack, dt, t);
		SerialUSB.print(str);
		t1 = millis();
	}
}

// PLAY SOUND WHEN QUITING
void QuitBleep()
{
	bool tone = true;
	for (int i = 0; i < 7; i++)
	{
		if (tone)
		{
			digitalWrite(pin_relRewTone, HIGH);
			digitalWrite(pin_relWhiteNoise, LOW);
		}
		else
		{
			digitalWrite(pin_relWhiteNoise, HIGH);
			digitalWrite(pin_relRewTone, LOW);
		}
		tone = !tone;
		delay(250);
	}
	digitalWrite(pin_relWhiteNoise, HIGH);
	digitalWrite(pin_relRewTone, LOW);
}

#pragma endregion


#pragma region --------PHOTO TRANSDUCERS ---------

// Reset PT pins
void ResetTTL()
{
	// north
	if (isOnNorth && millis() - t_outLastNorth > t_pulseWdth) {
		digitalWrite(pin_ttlNorthOn, LOW); // set back to LOW
		isOnNorth = false;
	}
	// west
	if (isOnWest && millis() - t_outLastWest > t_pulseWdth) {
		digitalWrite(pin_ttlWestOn, LOW); // set back to LOW
		isOnWest = false;
	}
	// south
	if (isOnSouth && millis() - t_outLastSouth > t_pulseWdth) {
		digitalWrite(pin_ttlSouthOn, LOW); // set back to LOW
		isOnSouth = false;
	}
	// east
	if (isOnEast && millis() - t_outLastEast > t_pulseWdth) {
		digitalWrite(pin_ttlEastOn, LOW); // set back to LOW
		isOnEast = false;
	}
}

// North
void NorthInterrupt()
{
	NorthFun();
}
void NorthFun()
{
	if (fc_isSesStarted)
	{
		if (millis() - t_inLastNorth > t_debounce) {
			digitalWrite(pin_ttlNorthOn, HIGH);
			t_outLastNorth = millis();
			isOnNorth = true;
		}
		t_inLastNorth = millis();
		if (doPrintFlow)
		{
			SerialUSB.print("\nNorth On\n");
		}
	}
}

// West
void WestInterrupt()
{
	WestFun();
}
void WestFun()
{
	if (fc_isSesStarted)
	{
		if (millis() - t_inLastWest > t_debounce) {
			digitalWrite(pin_ttlWestOn, HIGH);
			t_outLastWest = millis();
			isOnWest = true;
		}
		t_inLastWest = millis();
		if (doPrintFlow)
		{
			SerialUSB.print("\nWest On\n");
		}
	}
}

// South
void SouthInterrupt()
{
	SouthFun();
}
void SouthFun()
{
	if (fc_isSesStarted)
	{
		if (millis() - t_inLastSouth > t_debounce) {
			digitalWrite(pin_ttlSouthOn, HIGH);
			t_outLastSouth = millis();
			isOnSouth = true;
		}
		t_inLastSouth = millis();
		if (doPrintFlow)
		{
			SerialUSB.print("\nSouth On\n");
		}
	}
}

// East
void EastInterrupt()
{
	EastFun();
}
void EastFun()
{
	if (fc_isSesStarted)
	{
		if (millis() - t_inLastEast > t_debounce) {
			digitalWrite(pin_ttlEastOn, HIGH);
			t_outLastEast = millis();
			isOnEast = true;
		}
		t_inLastEast = millis();
		if (doPrintFlow)
		{
			SerialUSB.print("\nEast On\n");
		}
	}
}

#pragma endregion

