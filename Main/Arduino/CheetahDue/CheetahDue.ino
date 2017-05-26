//-------CheetahDue-------

#pragma region ---------DEBUG SETTINGS---------

struct DB
{
	// Do log
	bool Log = true;
	// What to print
	bool log_flow = true;
	bool log_r2a = true;
	bool log_a2r = true;
	bool log_resent = true;

	// Print to console
	bool Console = false;
	// What to print
	bool print_flow = true;
	bool print_r2a = true;
	bool print_a2r = true;
	bool print_resent = true;
	bool print_log = true;
}
// Initialize
db;

// TESTING 

// Print pin mapping
const bool doTestPinMapping = false;

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

// Flow/state control
struct FC
{
	bool doQuit = false;
	bool isSesStarted = false;
	bool doWhiteNoise = false;
	bool doRewTone = false;
	bool isRewarding = false;
}
// Initialize
fc;

// PT north vars
volatile uint32_t t_inLastNorth = millis();
volatile uint32_t t_outLastNorth = millis();
volatile bool isOnNorth = false;
// PT west vars
volatile uint32_t t_inLastWest = millis();
volatile uint32_t t_outLastWest = millis();
volatile bool isOnWest = false;
// PT south 
volatile uint32_t t_inLastSouth = millis();
volatile uint32_t t_outLastSouth = millis();
volatile bool isOnSouth = false;
// PT east vars
volatile uint32_t t_inLastEast = millis();
volatile uint32_t t_outLastEast = millis();
volatile bool isOnEast = false;

// TTL timers
uint32_t t_debounce = 10; // (ms)
uint32_t t_pulseWdth = 50; // (ms)

// Serial from rob
struct R2A
{
	char idList[6] = {
		't', // set sync time
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
	};
	char head = '{';
	char foot = '}';
	char idNew = ' ';
	byte dat = 0;
	const static int idLng = sizeof(idList) / sizeof(idList[0]);
	uint16_t packLast[idLng];
	bool isNew = false;
}
// Initialize
r2a;

// Serial to rob
struct A2R
{
	const char head = '{';
	const char foot = '}';
	byte dat[1] = { 255 };
}
// Initialize
a2r;

// Serial to CS
struct A2C
{
	const char head = '<';
	const char foot = '>';
}
// Initialize
a2c;

// Serial tracking
bool doPackSend = false;
uint32_t t_sent = millis(); // (ms)
uint32_t t_rcvd = millis(); // (ms)

// Log debugging
String logNow = " ";
String logLast = " ";
int cnt_logSend = 0;

// Reward
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;
uint32_t word_rewOn;
uint32_t word_rewOff;

// IR time sync LED
const uint32_t syncDur = 5; // (ms)
const uint32_t syncDel = 60000; // (ms)
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
	// XBee
	Serial1.begin(57600);

	// CS through programming port
	Serial.begin(57600);

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

	DebugFlow("RESTART");

	// Test pin mapping
	/*
	Note: make sure Cheetah aquiring
	*/
	if (doTestPinMapping) {
		delay(5000);
		bool p_flow = db.print_flow;
		db.print_flow = true;
		digitalWrite(pin_ttlNorthOn, HIGH); DebugFlow("North TTL"); delay(1000); digitalWrite(pin_ttlNorthOn, LOW);
		digitalWrite(pin_ttlWestOn, HIGH); DebugFlow("West TTL"); delay(1000); digitalWrite(pin_ttlWestOn, LOW);
		digitalWrite(pin_ttlSouthOn, HIGH); DebugFlow("South TTL"); delay(1000); digitalWrite(pin_ttlSouthOn, LOW);
		digitalWrite(pin_ttlEastOn, HIGH); DebugFlow("East TTL"); delay(1000); digitalWrite(pin_ttlEastOn, LOW);
		digitalWrite(pin_ttlIR, HIGH); DebugFlow("IR Sync TTL"); delay(1000); digitalWrite(pin_ttlIR, LOW);
		digitalWrite(pin_ttlWhiteNoise, HIGH); DebugFlow("White Noise TTL"); delay(1000); digitalWrite(pin_ttlWhiteNoise, LOW);
		digitalWrite(pin_ttlRewTone, HIGH); DebugFlow("Reward Tone TTL"); delay(1000); digitalWrite(pin_ttlRewTone, LOW);
		digitalWrite(pin_ttlRewOn, HIGH); DebugFlow("Reward On TTL"); delay(1000); digitalWrite(pin_ttlRewOn, LOW);
		digitalWrite(pin_ttlRewOff, HIGH); DebugFlow("Reward Off TTL"); delay(1000); digitalWrite(pin_ttlRewOff, LOW);
		digitalWrite(pin_ttlPidRun, HIGH); DebugFlow("PID Run TTL"); delay(1000); digitalWrite(pin_ttlPidRun, LOW);
		digitalWrite(pin_ttlPidStop, HIGH); DebugFlow("PID Stop TTL"); delay(1000); digitalWrite(pin_ttlPidStop, LOW);
		digitalWrite(pin_ttlBullRun, HIGH); DebugFlow("Bull Run TTL"); delay(1000); digitalWrite(pin_ttlBullRun, LOW);
		digitalWrite(pin_ttlBullStop, HIGH); DebugFlow("Bull Stop TTL"); delay(1000); digitalWrite(pin_ttlBullStop, LOW);
		digitalWrite(pin_relIR, HIGH); DebugFlow("IR Sync Relay"); delay(1000); digitalWrite(pin_relIR, LOW);
		digitalWrite(pin_relRewTone, HIGH); DebugFlow("Reward Tone Relay"); delay(1000); digitalWrite(pin_relRewTone, LOW);
		digitalWrite(pin_relWhiteNoise, HIGH); DebugFlow("White Noise Relay"); delay(1000); digitalWrite(pin_relWhiteNoise, LOW);
		db.print_flow = p_flow;
		delay(5000);
	}
}


// ---------MAIN LOOP---------
void loop()
{

	// Check for XBee input
	r2a.isNew = false;
	if (Serial1.available() > 0) {
		r2a.isNew = ParseSerial();
	}

	// Check for sync time indicating session starting
	if (!fc.isSesStarted)
	{
		if (r2a.isNew && r2a.idNew == 't')
		{
			// Set sync time
			t_sync = millis();
			DebugFlow("SET SYNC TIME", t_sync);

			// Set ir pulse
			CheckIRPulse();

			// Set flag
			fc.isSesStarted = true;
			DebugFlow("START SESSION");
		}
	}
	else
	{
		// Exicute input
		if (r2a.isNew)
		{

			// Run reward tone
			if (r2a.idNew == 'r') {

				// Get reward duration and convert to ms
				rewDur = (uint32_t)r2a.dat * 10;

				// Run tone
				StartRew();
			}

			// Get noise settings
			else if (r2a.idNew == 's') {

				// No noise
				if (r2a.dat == 0)
				{
					fc.doWhiteNoise = false;
					fc.doRewTone = false;
				}
				// White noise only
				else if (r2a.dat == 1)
				{
					// set white noise pins
					fc.doWhiteNoise = true;
					fc.doRewTone = false;
				}
				// White and reward sound
				else if (r2a.dat == 2)
				{
					// set white noise pins
					fc.doWhiteNoise = true;
					fc.doRewTone = true;
				}

				// Get reward on port word
				if (fc.doWhiteNoise && fc.doRewTone) {
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
				if (fc.doWhiteNoise) {
					// turn white on
					int sam_white_pins[2] = { sam_relWhiteNoise, sam_ttlWhiteNoise };
					uint32_t word_white = GetPortWord(0x0, sam_white_pins, 2);
					SetPort(word_white, 0x0);
				}

			}

			// Signal PID mode
			else if (r2a.idNew == 'p') {
				// Signal PID stopped
				if (r2a.dat == 0)
				{
					digitalWrite(pin_ttlPidStop, HIGH);
					digitalWrite(pin_ttlPidRun, LOW);
					DebugFlow("PID STOPPED");
				}
				// Signal PID running
				else if (r2a.dat == 1)
				{
					digitalWrite(pin_ttlPidRun, HIGH);
					digitalWrite(pin_ttlPidStop, LOW);
					DebugFlow("PID RUNNING");
				}
			}

			// Signal bull running
			else if (r2a.idNew == 'b') {
				// Signal Bull stopped
				if (r2a.dat == 0)
				{
					digitalWrite(pin_ttlBullStop, HIGH);
					digitalWrite(pin_ttlBullRun, LOW);
					DebugFlow("BULL STOPPED");
				}
				// Signal Bull running
				else if (r2a.dat == 1)
				{
					digitalWrite(pin_ttlBullRun, HIGH);
					digitalWrite(pin_ttlBullStop, LOW);
					DebugFlow("BULL RUNNING");
				}
			}

			// Quite and reset
			else if (r2a.idNew == 'q') {
				fc.doQuit = true;
				DebugFlow("QUITING");
			}

		}

		// Check for rew end
		if (fc.isRewarding)
		{
			EndRew();
		}

		// Set output pins back to low
		ResetTTL();

		// Check if IR should be pulsed 
		CheckIRPulse();

		// Check if ready to quit
		if (fc.doQuit && !doPackSend)
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
		SendPacketData(r2a.idNew, 255, r2a.packLast[CharInd(r2a.idNew, r2a.idList, r2a.idLng)], false);
	}

}


#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
bool ParseSerial()
{

	static char head = ' ';
	static char foot = ' ';
	static bool conf = false;
	uint16_t pack = 0;
	static bool pass = false;
	doPackSend = false;

	// Dump data till header byte is reached
	while (Serial1.peek() != r2a.head && Serial1.available() > 0)
	{
		Serial1.read(); // dump
	}

	// Save header
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	head = u.c[0];

	// Check header
	if (head != r2a.head) {
		// mesage will be dumped
		return pass = false;
	}

	// Get id
	while (Serial1.available() < 1);
	u.b[1] = Serial1.read();
	r2a.idNew = u.c[1];

	// Get data
	while (Serial1.available() < 1);
	r2a.dat = Serial1.read();

	// Get pack number
	while (Serial1.available() < 2);
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	u.b[1] = Serial1.read();
	pack = u.i16[0];

	// Get send confirm request
	while (Serial1.available() < 1);
	conf = Serial1.read() == 1 ? true : false;

	// Get footer
	while (Serial1.available() < 1);
	u.b[2] = Serial1.read();
	foot = u.c[2];

	// Check for missing footer
	if (foot != r2a.foot) {
		// mesage will be dumped
		pass = false;
	}
	else
	{
		// Send back confirmation
		doPackSend = true;

		// Print rcvd pack
		DebugRcvd(r2a.idNew, r2a.dat, pack, conf);
		pass = true;

		// Update recive time
		t_rcvd = millis();
	}

	// Check if packet is new
	if (pass)
	{
		if (r2a.packLast[CharInd(r2a.idNew, r2a.idList, r2a.idLng)] != pack)
		{
			// Update last packet
			r2a.packLast[CharInd(r2a.idNew, r2a.idList, r2a.idLng)] = pack;
		}
		else
		{
			// Print resent packet
			PrintResent(r2a.idNew, r2a.dat, pack);
			pass = false;
		}
	}

	return pass;

}

// SEND SERIAL PACKET DATA
void SendPacketData(char id, byte d1, uint16_t pack, bool do_conf)
{
	// Local vars
	const int msg_size = 7;
	byte msg[msg_size];
	bool pass = false;

	// Confirm message really intended for here

	for (int i = 0; i < r2a.idLng; i++)
	{
		if (id == r2a.idList[i])
			pass = true;
	}

	if (pass)
	{
		// Store header
		u.f = 0.0f;
		u.c[0] = a2r.head;
		msg[0] = u.b[0];
		// Store mesage id
		u.f = 0.0f;
		u.c[0] = id;
		msg[1] = u.b[0];
		// Store mesage data 
		msg[2] = d1;
		// Store packet number
		u.f = 0.0f;
		u.i16[0] = pack;
		msg[3] = u.b[0];
		msg[4] = u.b[1];
		// Store get_confirm request
		msg[5] = do_conf ? 1 : 0;
		// Store footer
		u.f = 0.0f;
		u.c[0] = a2r.foot;
		msg[6] = u.b[0];

		// Send
		Serial1.write(msg, msg_size);
		t_sent = millis();

		// Print sent
		DebugSent(id, d1, pack, do_conf);

		// Reset flag
		doPackSend = false;
	}
}

// SEND SERIAL LOG DATA
void SendLogData(String str, uint32_t t)
{
	// Local vars
	char str_c[200];
	char msg_c[200];
	byte chksum = 0;
	byte msg_size = 0;
	String msg_s = " ";
	uint32_t t_m = 0;
	byte msg[200];

	// Itterate log entry count
	cnt_logSend++;

	// Time now
	t_m = t_sync == 0 || t_sync == t ? t : t - t_sync;

	// Concatinate ts with message
	str.toCharArray(str_c, 200);
	sprintf(msg_c, "[%d],%lu,%s", cnt_logSend, t_m, str_c);
	msg_s = msg_c;

	// Get message size
	chksum = msg_s.length();

	// Load byte array

	// head
	msg[0] = a2c.head;
	// checksum
	msg[1] = chksum;
	// msg
	for (int i = 0; i < chksum; i++)
		msg[i + 2] = msg_c[i];
	// foot
	msg[chksum + 2] = a2c.foot;

	// Compute message size
	msg_size = chksum + 3;

	// Send
	Serial.write(msg, msg_size);

	// Store message
	logLast = logNow;
	logNow = String((char *)msg);

	// Print
	if (db.print_log && db.Console)
	{
		char chr[100];
		String str;

		// Store
		sprintf(chr, "sent log: sent=%d chksum=%d: ", cnt_logSend, chksum);
		str = chr;
		PrintDB(str + "\"" + msg_s + "\"", millis());
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
	DebugFlow("PID STOPPED");

	// Set flag
	fc.isRewarding = true;

	// Print
	char chr[50];
	sprintf(chr, "REWARDING(%dms)...", rewDur);
	String str = chr;
	DebugFlow(str);
}

// END REWARD
void EndRew()
{
	if (millis() > t_rewEnd)
	{

		// Set reward off pins
		SetPort(word_rewOff, word_rewOn);

		fc.isRewarding = false;
		DebugFlow("REWARD OFF");

	}
}

#pragma endregion

#pragma region --------DEBUGGING---------

// LOG/PRING MAIN EVENT
void DebugFlow(String msg)
{
	DebugFlow(msg, millis());
}
void DebugFlow(String msg, uint32_t t)
{
	// Local vars
	bool do_print = db.Console && db.print_flow;
	bool do_log = db.Log && db.log_flow;

	if (do_print)
		PrintDB(msg, millis());
	if (do_log)
		SendLogData(msg, millis());
}

// PRINT RECIEVED PACKET
void DebugRcvd(char id, byte dat, uint16_t pack, bool conf)
{
	// Local vars
	bool do_print = db.Console && db.print_r2a;
	bool do_log = db.Log && db.log_r2a;

	// Print/Log
	if (do_print || do_log)
	{
		char str[50];
		sprintf(str, "rcvd_r2a: id=%c dat=%d pack=%d conf=%s", id, dat, pack, conf ? "true" : "false");
		if (do_print)
			PrintDB(str, t_rcvd);
		if (do_log)
			SendLogData(str, millis());
	}
}

// LOG/PRING SENT PACKET DEBUG STRING
void DebugSent(char id, byte d1, uint16_t pack, bool do_conf)
{
	// Local vars
	bool do_print = db.Console && db.print_a2r;
	bool do_log = db.Log && db.log_a2r;

	// Print/Log
	if (do_print || do_log)
	{
		char str[50];
		sprintf(str, "sent_a2r: id=%c dat1=%d pack=%d do_conf=%s", id, d1, pack, do_conf ? "true" : "false");
		if (do_print)
			PrintDB(str, t_sent);
		if (do_log)
			SendLogData(str, millis());
	}
}

// PRINT RESENT PACKET
void PrintResent(char id, byte dat, uint16_t pack)
{
	// Local vars
	bool do_print = db.Console && db.print_resent;
	bool do_log = db.Log && db.log_resent;
	static int resent_cnt = 0;

	// Print/Log
	if (do_print || do_log)
	{
		// Itterate count
		resent_cnt++;

		char str[50];
		sprintf(str, "!!RESENT PACK: tot=%d id=%c dat=%d pack=%d!!", resent_cnt, id, dat, pack);
		if (do_print)
			PrintDB(str, millis());
		if (do_log)
			SendLogData(str, millis());

	}
}

// PRINT DB INFO
void PrintDB(String msg, uint32_t t)
{
	static uint32_t t1 = millis();
	uint32_t t_m = 0;
	float t_s = 0;

	// Time now
	t_m = t_sync == 0 || t_sync == t ? t : t - t_sync;

	// Total time
	t_s = (float)(t_m) / 1000.0f;

	// Pad string
	char chr[50];
	char chr_ts[50];
	char arg[50];
	sprintf(chr_ts, "[%0.2fs]", t_s);
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
		DebugFlow("IR SYNC ON");
		is_ir_on = true;
	}
	// Set low
	else if (
		is_ir_on &&
		millis() > t_syncLast + syncDur
		)
	{
		SetPort(0x0, word_irOn);
		DebugFlow("IR SYNC OFF");
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
	if (fc.isSesStarted)
	{
		if (millis() - t_inLastNorth > t_debounce) {
			digitalWrite(pin_ttlNorthOn, HIGH);
			t_outLastNorth = millis();
			isOnNorth = true;
			// Print
			if (db.print_flow)
				PrintDB("NORTH ON", t_outLastNorth);
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
	if (fc.isSesStarted)
	{
		if (millis() - t_inLastWest > t_debounce) {
			digitalWrite(pin_ttlWestOn, HIGH);
			t_outLastWest = millis();
			isOnWest = true;
			// Print
			if (db.print_flow)
				PrintDB("WEST ON", t_outLastWest);
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
	if (fc.isSesStarted)
	{
		if (millis() - t_inLastSouth > t_debounce) {
			digitalWrite(pin_ttlSouthOn, HIGH);
			t_outLastSouth = millis();
			isOnSouth = true;
			// Print
			if (db.print_flow)
				PrintDB("SOUTH ON", t_outLastSouth);
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
	if (fc.isSesStarted)
	{
		if (millis() - t_inLastEast > t_debounce) {
			digitalWrite(pin_ttlEastOn, HIGH);
			t_outLastEast = millis();
			isOnEast = true;
			// Print
			if (db.print_flow)
				PrintDB("EAST ON", t_outLastEast);
		}
		t_inLastEast = millis();
	}
}

#pragma endregion

