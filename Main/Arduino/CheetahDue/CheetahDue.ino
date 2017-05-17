//-------CheetahDue-------

#pragma region ---------DEBUG SETTINGS---------

bool doPrintFlow = false;
bool doPrintRcvdPack = false;
bool doPrintSentPack = false;
bool doPrintResent = false;
bool doTestPinMapping = false;

#pragma endregion 

#pragma region ---------LIBRARIES & PACKAGES---------

// SOFTWARE RESET
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#pragma endregion 

#pragma region ---------PIN SETUP---------

// DEFINE PINS

// Relays
const int pin_relIR = 51;
const int pin_relRewTone = 47;
const int pin_relWhiteNoise = 45;
// ttl
const int pin_ttlIR = 50;
const int pin_ttlRewTone = 46;
const int pin_ttlWhiteNoise = 44;

// Rew
const int pin_ttlRewOn = 34;
const int pin_ttlRewOff = 36;

// SAM3X pin
const int sam_relIR = 12;
const int sam_relRewTone = 16;
const int sam_relWhiteNoise = 18;
const int sam_ttlIR = 13;
const int sam_ttlRewTone = 17;
const int sam_ttlWhiteNoise = 19;
const int sam_ttlRewOn = 2;
const int sam_ttlRewOff = 4;

// PID
const int pin_ttlPidRun = 26;
const int pin_ttlPidStop = 28;

// Bulldozer
const int pin_ttlBullRun = 30;
const int pin_ttlBullStop = 32;

// PT
const int pin_ttlNorthOn = A7;
const int pin_ttlWestOn = A6;
const int pin_ttlSouthOn = A5;
const int pin_ttlEastOn = A4;
const int pin_ptNorthOn = A3;
const int pin_ptWestOn = A2;
const int pin_ptSouthOn = A1;
const int pin_ptEastOn = A0;




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
bool fc_doQuit = false;
bool fc_isSesStarted = false;
bool fc_doWhiteNoise = false;
bool fc_doRewTone = false;
bool fc_isRewarding = false;

// Serial
char msg_id = ' ';
byte msg_dat;
char r2a_id[6] = {
	't', // set sync time
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
};
const int r2a_idLng = sizeof(r2a_id) / sizeof(r2a_id[0]);
uint16_t r2a_packLast[r2a_idLng];
char r2a_head = '{';
char r2a_foot = '}';
bool r2a_isNew = false;
bool doPackSend = false;
const char a2r_head = '{';
const char a2r_foot = '}';

// Reward
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;
uint32_t word_rewOn;
uint32_t word_rewOff;

// IR time sync LED
const uint32_t syncDur = 5; // (ms)
const uint32_t syncDel = 10000; // (ms)
uint32_t t_sync = 0;
uint32_t t_syncLast;
uint32_t word_irOn;

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
	// XBee.
	Serial1.begin(57600);

	// Set PT pin direction
	pinMode(pin_ptNorthOn, INPUT);
	pinMode(pin_ptWestOn, INPUT);
	pinMode(pin_ptSouthOn, INPUT);
	pinMode(pin_ptEastOn, INPUT);
	pinMode(pin_ttlNorthOn, OUTPUT);
	pinMode(pin_ttlWestOn, OUTPUT);
	pinMode(pin_ttlSouthOn, OUTPUT);
	pinMode(pin_ttlEastOn, OUTPUT);

	// Set PT pins low
	digitalWrite(pin_ttlNorthOn, LOW);
	digitalWrite(pin_ttlWestOn, LOW);
	digitalWrite(pin_ttlSouthOn, LOW);
	digitalWrite(pin_ttlEastOn, LOW);

	// Set relay/ttl pin to output
	pinMode(pin_relIR, OUTPUT);
	pinMode(pin_relWhiteNoise, OUTPUT);
	pinMode(pin_relRewTone, OUTPUT);
	pinMode(pin_ttlIR, OUTPUT);
	pinMode(pin_ttlRewTone, OUTPUT);
	pinMode(pin_ttlWhiteNoise, OUTPUT);

	// set relay/ttl pins low
	digitalWrite(pin_relIR, LOW);
	digitalWrite(pin_relRewTone, LOW);
	digitalWrite(pin_relWhiteNoise, LOW);
	digitalWrite(pin_ttlIR, LOW);
	digitalWrite(pin_ttlRewTone, LOW);
	digitalWrite(pin_ttlWhiteNoise, LOW);

	// set other output pins
	pinMode(pin_ttlRewOn, OUTPUT);
	pinMode(pin_ttlRewOff, OUTPUT);
	pinMode(pin_ttlBullRun, OUTPUT);
	pinMode(pin_ttlBullStop, OUTPUT);
	pinMode(pin_ttlPidRun, OUTPUT);
	pinMode(pin_ttlPidStop, OUTPUT);

	// Setup reward ttl stuff on port C
	REG_PIOC_OWER = 0xFFFFFFFF;     // enable PORT C
	REG_PIOC_OER = 0xFFFFFFFF;     // set PORT C as output port

	// Get ir word
	int sam_ir_pins[2] = { sam_ttlIR, sam_relIR };
	word_irOn = GetPortWord(0x0, sam_ir_pins, 2);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(pin_ptNorthOn), NorthInterrupt, RISING); // north
	attachInterrupt(digitalPinToInterrupt(pin_ptWestOn), WestInterrupt, RISING); // west
	attachInterrupt(digitalPinToInterrupt(pin_ptSouthOn), SouthInterrupt, RISING); // south
	attachInterrupt(digitalPinToInterrupt(pin_ptEastOn), EastInterrupt, RISING); // east

	PrintState("RESTART");

	// Test pin mapping
	/*
	Note: make sure Cheetah aquiring
	*/
	if (doTestPinMapping) {
		delay(5000);
		bool print_flow = doPrintFlow;
		doPrintFlow = true;
		digitalWrite(pin_ttlNorthOn, HIGH); PrintState("North TTL"); delay(1000); digitalWrite(pin_ttlNorthOn, LOW);
		digitalWrite(pin_ttlWestOn, HIGH); PrintState("West TTL"); delay(1000); digitalWrite(pin_ttlWestOn, LOW);
		digitalWrite(pin_ttlSouthOn, HIGH); PrintState("South TTL"); delay(1000); digitalWrite(pin_ttlSouthOn, LOW);
		digitalWrite(pin_ttlEastOn, HIGH); PrintState("East TTL"); delay(1000); digitalWrite(pin_ttlEastOn, LOW);
		digitalWrite(pin_ttlIR, HIGH); PrintState("IR Sync TTL"); delay(1000); digitalWrite(pin_ttlIR, LOW);
		digitalWrite(pin_ttlWhiteNoise, HIGH); PrintState("White Noise TTL"); delay(1000); digitalWrite(pin_ttlWhiteNoise, LOW);
		digitalWrite(pin_ttlRewTone, HIGH); PrintState("Reward Tone TTL"); delay(1000); digitalWrite(pin_ttlRewTone, LOW);
		digitalWrite(pin_ttlRewOn, HIGH); PrintState("Reward On TTL"); delay(1000); digitalWrite(pin_ttlRewOn, LOW);
		digitalWrite(pin_ttlRewOff, HIGH); PrintState("Reward Off TTL"); delay(1000); digitalWrite(pin_ttlRewOff, LOW);
		digitalWrite(pin_ttlPidRun, HIGH); PrintState("PID Run TTL"); delay(1000); digitalWrite(pin_ttlPidRun, LOW);
		digitalWrite(pin_ttlPidStop, HIGH); PrintState("PID Stop TTL"); delay(1000); digitalWrite(pin_ttlPidStop, LOW);
		digitalWrite(pin_ttlBullRun, HIGH); PrintState("Bull Run TTL"); delay(1000); digitalWrite(pin_ttlBullRun, LOW);
		digitalWrite(pin_ttlBullStop, HIGH); PrintState("Bull Stop TTL"); delay(1000); digitalWrite(pin_ttlBullStop, LOW);
		digitalWrite(pin_relIR, HIGH); PrintState("IR Sync Relay"); delay(1000); digitalWrite(pin_relIR, LOW);
		digitalWrite(pin_relRewTone, HIGH); PrintState("Reward Tone Relay"); delay(1000); digitalWrite(pin_relRewTone, LOW);
		digitalWrite(pin_relWhiteNoise, HIGH); PrintState("White Noise Relay"); delay(1000); digitalWrite(pin_relWhiteNoise, LOW);
		doPrintFlow = print_flow;
		delay(5000);
	}
}


// ---------MAIN LOOP---------
void loop()
{

	// Check for XBee input
	r2a_isNew = false;
	if (Serial1.available() > 0) {
		r2a_isNew = ParseSerial();
	}

	// Check for sync time indicating session starting
	if (!fc_isSesStarted)
	{
		if (r2a_isNew && msg_id == 't')
		{
			// Set sync time and pulse IR
			t_sync = millis();
			CheckIRPulse();

			// Set flag
			fc_isSesStarted = true;
			PrintState("START SESSION");
		}
	}
	else
	{
		// Exicute input
		if (r2a_isNew)
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
				}
				// White noise only
				else if (msg_dat == 1)
				{
					// set white noise pins
					fc_doWhiteNoise = true;
					fc_doRewTone = false;
				}
				// White and reward sound
				else if (msg_dat == 2)
				{
					// set white noise pins
					fc_doWhiteNoise = true;
					fc_doRewTone = true;
				}

				// Get reward on port word
				if (fc_doWhiteNoise && fc_doRewTone) {
					// white and tone pins
					int sam_on_pins[3] = { sam_relRewTone, sam_ttlRewTone, sam_ttlRewOn };
					word_rewOn = GetPortWord(0x0, sam_on_pins, 3);
					int sam_off_pins[3] = { sam_relWhiteNoise, sam_ttlWhiteNoise, sam_ttlRewOff };
					word_rewOff = GetPortWord(0x0, sam_off_pins, 3);
				}
				else {
					// rew event pins only
					int sam_on_pins[1] = { sam_ttlRewOn };
					word_rewOn = GetPortWord(0x0, sam_on_pins, 1);
					int sam_off_pins[1] = { sam_ttlRewOff };
					word_rewOff = GetPortWord(0x0, sam_off_pins, 1);
				}
				// Turn on white noise
				if (fc_doWhiteNoise) {
					// turn white on
					int sam_white_pins[2] = { sam_relWhiteNoise, sam_ttlWhiteNoise };
					uint32_t word_white = GetPortWord(0x0, sam_white_pins, 2);
					SetPort(word_white, 0x0);
				}

			}

			// Signal PID mode
			else if (msg_id == 'p') {
				// Signal PID stopped
				if (msg_dat == 0)
				{
					digitalWrite(pin_ttlPidStop, HIGH);
					digitalWrite(pin_ttlPidRun, LOW);
					PrintState("PID STOPPED");
				}
				// Signal PID running
				else if (msg_dat == 1)
				{
					digitalWrite(pin_ttlPidRun, HIGH);
					digitalWrite(pin_ttlPidStop, LOW);
					PrintState("PID RUNNING");
				}
			}

			// Signal bull running
			else if (msg_id == 'b') {
				// Signal Bull stopped
				if (msg_dat == 0)
				{
					digitalWrite(pin_ttlBullStop, HIGH);
					digitalWrite(pin_ttlBullRun, LOW);
					PrintState("BULL STOPPED");
				}
				// Signal Bull running
				else if (msg_dat == 1)
				{
					digitalWrite(pin_ttlBullRun, HIGH);
					digitalWrite(pin_ttlBullStop, LOW);
					PrintState("BULL RUNNING");
				}
			}

			// Quite and reset
			else if (msg_id == 'q') {
				fc_doQuit = true;
				PrintState("QUITING");
			}

		}

		// Check for rew end
		if (fc_isRewarding)
		{
			EndRew();
		}

		// Set output pins back to low
		ResetTTL();

		// Check if IR should be pulsed 
		CheckIRPulse();

		// Check if ready to quit
		if (fc_doQuit && !doPackSend)
		{
			// Run bleep bleep
			QuitBleep();
			// Restart Arduino
			REQUEST_EXTERNAL_RESET;
		}

	}

	// Send message confirmation
	if (doPackSend)
	{
		SendSerial(msg_id, r2a_packLast[CharInd(msg_id, r2a_id, r2a_idLng)]);
	}

}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{

	static char head = ' ';
	static char foot = ' ';
	uint16_t pack = 0;
	static bool pass = false;
	doPackSend = false;

	// Dump data till header byte is reached
	while (Serial1.peek() != r2a_head && Serial1.available() > 0)
	{
		Serial1.read(); // dump
	}

	// Save header
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	head = u.c[0];

	// Check header
	if (head != r2a_head) {
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
	// get data
	msg_dat = Serial1.read();

	// Wait for pack number packet
	while (Serial1.available() < 2);
	// get packet num
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	u.b[1] = Serial1.read();
	pack = u.i16[0];

	// Wait for foot packet
	while (Serial1.available() < 1);

	// Check for footer
	u.b[2] = Serial1.read();
	foot = u.c[2];

	// Footer missing
	if (foot != r2a_foot) {
		// mesage will be dumped
		pass = false;
	}
	else
	{
		// Send back confirmation
		doPackSend = true;
		// Print rcvd pack
		PrintRcvdPack(msg_id, msg_dat, pack);
		pass = true;
	}

	// Check if packet is new
	if (pass)
	{
		if (r2a_packLast[CharInd(msg_id, r2a_id, r2a_idLng)] != pack)
		{
			// Update last packet
			r2a_packLast[CharInd(msg_id, r2a_id, r2a_idLng)] = pack;
		}
		else
		{
			// Print resent packet
			PrintResent(msg_id, msg_dat, pack);
			pass = false;
		}
	}

	return pass;

}

// SEND SERIAL DATA
void SendSerial(char id, uint16_t pack)
{
	// Local vars
	const int msg_size = 5;
	byte msg[msg_size];
	bool pass = false;

	// Confirm message really intended for here

	for (int i = 0; i < r2a_idLng; i++)
	{
		if (id == r2a_id[i])
			pass = true;
	}

	if (pass)
	{
		// Store header
		u.f = 0.0f;
		u.c[0] = a2r_head;
		msg[0] = u.b[0];
		// Store mesage id
		u.f = 0.0f;
		u.c[0] = id;
		msg[1] = u.b[0];
		// Store packet number
		u.f = 0.0f;
		u.i16[0] = pack;
		msg[2] = u.b[0];
		msg[3] = u.b[1];
		// Store footer
		u.f = 0.0f;
		u.c[0] = a2r_foot;
		msg[4] = u.b[0];

		// Send
		Serial1.write(msg, msg_size);

		// Print sent
		PrintSentPack(id, pack);

		// Reset flag
		doPackSend = false;
	}
}

#pragma endregion


#pragma region --------REWARD---------

// START REWARD
void StartRew()
{
	// Set rew on pins
	SetPort(word_rewOn, word_rewOff);

	// Set rew off time
	t_rewEnd = millis() + rewDur;

	// Signal PID stopped
	digitalWrite(pin_ttlPidRun, LOW);
	digitalWrite(pin_ttlPidStop, HIGH);
	PrintState("PID STOPPED");

	// Set flag
	fc_isRewarding = true;

	// Print
	char chr[50];
	sprintf(chr, "REWARDING(%dms)...", rewDur);
	String str = chr;
	PrintState(str);
}

// END REWARD
void EndRew()
{
	if (millis() > t_rewEnd)
	{

		// Set reward off pins
		SetPort(word_rewOff, word_rewOn);

		fc_isRewarding = false;
		PrintState("REWARD OFF");

	}
}

#pragma endregion


#pragma region --------DEBUGGING---------

// PRINT STATUS
void PrintState(String msg)
{
	if (doPrintFlow)
	{
		PrintStr(msg, millis());
	}
}

// PRINT RECIEVED PACKET
void PrintRcvdPack(char id, byte dat, uint16_t pack)
{

	//// Print
	if (doPrintRcvdPack)
	{
		char str[50];
		sprintf(str, "rcvd: id=%c dat=%d pack=%d", id, dat, pack);
		PrintStr(str, millis());
	}
}

// PRINT SENT PACKET
void PrintSentPack(char id, uint16_t pack)
{

	//// Print
	if (doPrintSentPack)
	{
		char str[50];
		sprintf(str, "sent: id=%c pack=%d", id, pack);
		PrintStr(str, millis());
	}
}

// PRINT RESENT PACKET
void PrintResent(char id, byte dat, uint16_t pack)
{
	static int resent_cnt = 0;
	resent_cnt++;

	//// Print
	if (doPrintResent)
	{
		char str[50];
		sprintf(str, "!!RESENT PACK: tot=%d id=%c dat=%d pack=%d!!", resent_cnt, id, dat, pack);
		PrintStr(str, millis());
	}
}

// PRINT DB INFO
void PrintStr(String msg, uint32_t ts)
{
	static uint32_t t1 = millis();
	uint32_t ts_norm = 0;
	float t_c = 0;
	float t;

	// Time now
	ts_norm = t_sync == 0 ? ts : ts - t_sync;

	// Total time
	t_c = (float)(ts_norm) / 1000.0f;

	// Pad string
	char chr[50];
	char chr_ts[50];
	char arg[50];
	sprintf(chr_ts, "[%0.2fs]", t_c);
	String str_ts = chr_ts;
	sprintf(arg, "%%%ds", 20 - str_ts.length());
	sprintf(chr, arg, '_');

	// Append string
	String str = str_ts + chr + msg + "\n";

	// Print
	SerialUSB.print(str);
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// GET ID INDEX
int CharInd(char id, char id_arr[], int arr_size)
{

	int ind = -1;
	for (int i = 0; i < arr_size; i++)
	{
		if (id == id_arr[i])
			ind = i;
	}

	return ind;

}

// PULSE IR
void CheckIRPulse()
{
	static bool is_ir_on = false;
	// Set high
	if (
		!is_ir_on &&
		millis() > t_syncLast + syncDel
		)
	{
		// Set ir pins on
		SetPort(word_irOn, 0x0);
		t_syncLast = millis();
		PrintState("IR SYNC ON");
		is_ir_on = true;
	}
	// Set low
	else if (
		is_ir_on &&
		millis() > t_syncLast + syncDur
		)
	{
		SetPort(0x0, word_irOn);
		PrintState("IR SYNC OFF");
		is_ir_on = false;
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

// GET 32 BIT WORD FOR PORT
uint32_t GetPortWord(uint32_t word, int pin_arr[], int arr_size)
{

	// Get 32 bit state
	for (int i = 0; i < arr_size; i++) {
		word = word | 0x01 << pin_arr[i];
	}

	return word;
}

// SET PORT
void SetPort(uint32_t word_on, uint32_t word_off)
{
	uint32_t word_new = REG_PIOC_ODSR;
	word_new = word_new & ~word_off;
	word_new |= word_on;
	REG_PIOC_ODSR = word_new;
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
			// Print
			if (doPrintFlow)
				PrintStr("NORTH ON", t_outLastNorth);
		}
		t_inLastNorth = millis();
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
			// Print
			if (doPrintFlow)
				PrintStr("WEST ON", t_outLastWest);
		}
		t_inLastWest = millis();
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
			// Print
			if (doPrintFlow)
				PrintStr("SOUTH ON", t_outLastSouth);
		}
		t_inLastSouth = millis();
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
			// Print
			if (doPrintFlow)
				PrintStr("EAST ON", t_outLastEast);
		}
		t_inLastEast = millis();
	}
}

#pragma endregion

