//######################################

//============ CHEETAHDUE ==============

//######################################


#pragma region ========= LIBRARIES & EXT DEFS =========

// SOFTWARE RESET
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#pragma endregion 


#pragma region ========= DEBUG SETTINGS =========

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
	bool Console = true;
	// What to print
	bool print_flow = true;
	bool print_r2a = false;
	bool print_a2r = false;
	bool print_resent = false;
	bool print_log = false;
}
// Initialize
db;

// TESTING 

// Print pin mapping
const bool doTestPinMapping = false;

#pragma endregion 


#pragma region ========= VARIABLE SETUP =========

// Pin mapping
struct PIN
{
	// Relays
	const int relIR = 51;
	const int relRewTone = 47;
	const int relWhiteNoise = 45;
	// ttl
	const int ttlIR = 50;
	const int ttlRewTone = 46;
	const int ttlWhiteNoise = 44;

	// Rew
	const int ttlRewOn = 34;
	const int ttlRewOff = 36;

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
	const int ttlPidRun = 26;
	const int ttlPidStop = 28;

	// Bulldozer
	const int ttlBullRun = 30;
	const int ttlBullStop = 32;

	// PT
	const int ttlNorthOn = A7;
	const int ttlWestOn = A6;
	const int ttlSouthOn = A5;
	const int ttlEastOn = A4;
	const int ptNorthOn = A3;
	const int ptWestOn = A2;
	const int ptSouthOn = A1;
	const int ptEastOn = A0;
}
// Initialize
pin;

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
uint32_t dt_ttlPulse = 50; // (ms)

// Serial from rob
struct R2A
{
	char idList[5] = {
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
	uint16_t packLast[idLng] = { 0 };
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
int cnt_logSend = 0;

// Reward
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;
uint32_t word_rewOn;
uint32_t word_rewOff;

// IR time sync LED
const uint32_t dt_syncPulse = 10; // (ms)
uint32_t del_syncPulse = 60000; // (ms)
uint32_t t_sync = 0;
bool is_irOn = false;
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
} U;

#pragma endregion 


#pragma region ========= FUNCTION DECLARATIONS =========

#pragma region --------COMMUNICATION---------

// CHECK FOR START COMMAND
bool AwaitStart()
{
	if (!fc.isSesStarted)
	{
		// Local vars
		byte in_byte[1] = { 0 };
		byte out_byte[1] = { 0 };
		bool is_rcvd = false;
		int send_cnt = 0;
		char str[200] = { 0 };

		// Get id byte
		out_byte[0] = '+';

		// Loop till message found
		while (!fc.isSesStarted)
		{
			// Check if IR should be pulsed 
			PulseIR(1000, dt_syncPulse);

			// Get new data
			if (Serial.available() > 0)
			{
				in_byte[0] = Serial.read();
				millis();
				// Check for match
				if (in_byte[0] == out_byte[0])
				{
					// Store time
					t_sync = millis();

					// Send confirmation
					Serial.write(out_byte, 1);
					send_cnt++;

					// Pulse ir
					if (!is_irOn) PulseIR(); // turn on
					delay(10 - (millis() - t_syncLast));
					PulseIR();  // turn off
					delay(75);
					PulseIR(); // turn on
					delay(10);
					PulseIR();  // turn off

					// Dump buffer
					while (Serial.available())
						Serial.read();

					// Wait and make sure message recieved
					while (!is_rcvd)
					{
						delay(1000);
						if (Serial.available() > 0)
						{
							in_byte[0] == Serial.read();
							if (in_byte[0] == out_byte[0])
							{
								// Resend confirmation
								Serial.write(out_byte, 1);
								send_cnt++;
								sprintf(str, "!!ERROR!! HANDSHAKE FAILED: tot=%d", send_cnt);
							}
						}
						else
						{
							// Log any failures
							if (str[0] != '\0')
								DebugFlow(str);

							// Log success
							DebugFlow("HANDSHAKE SUCCEEDED");

							// Log sync time
							sprintf(str, "SET SYNC TIME: %dms", t_sync);
							DebugFlow(str, t_sync);

							// Set flags
							is_rcvd = true;
							fc.isSesStarted = true;
							DebugFlow("START SESSION");
						}

					}

				}
			}

		}
	}
}

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
	head = Serial1.read();

	// Check header
	if (head != r2a.head) {
		// mesage will be dumped
		return pass = false;
	}

	// Get id
	while (Serial1.available() < 1);
	r2a.idNew = Serial1.read();

	// Get data
	while (Serial1.available() < 1);
	r2a.dat = Serial1.read();

	// Get pack number
	while (Serial1.available() < 2);
	U.f = 0.0f;
	U.b[0] = Serial1.read();
	U.b[1] = Serial1.read();
	pack = U.i16[0];

	// Get send confirm request
	while (Serial1.available() < 1);
	conf = Serial1.read() == 1 ? true : false;

	// Get footer
	while (Serial1.available() < 1);
	foot = Serial1.read();

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
		msg[0] = a2r.head;
		// Store mesage id
		msg[1] = id;
		// Store mesage data 
		msg[2] = d1;
		// Store packet number
		U.f = 0.0f;
		U.i16[0] = pack;
		msg[3] = U.b[0];
		msg[4] = U.b[1];
		// Store get_confirm request
		msg[5] = do_conf ? 1 : 0;
		// Store footer
		msg[6] = a2r.foot;

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
void SendLogData(char msg[], uint32_t t)
{
	// Local vars
	static char msg_out[200] = { 0 };
	char str[200] = { 0 };
	byte chksum = 0;
	byte msg_lng = 0;
	uint32_t t_m = 0;

	// Itterate log entry count
	cnt_logSend++;

	// Get sync correction
	t_m = t - t_sync;

	// Concatinate ts with message
	sprintf(str, "[%d],%lu,%s", cnt_logSend, t_m, msg);

	// Get message size
	chksum = strlen(str);

	// Load byte array

	// head
	msg_out[msg_lng++] = a2c.head;
	// checksum
	msg_out[msg_lng++] = chksum;
	// msg
	for (int i = 0; i < chksum; i++)
		msg_out[msg_lng++] = str[i];
	// foot
	msg_out[msg_lng++] = a2c.foot;
	// null
	msg_out[msg_lng + 1] = '\0';

	// Send
	Serial.write(msg_out, msg_lng);

	// Print
	if (db.print_log && db.Console)
	{
		// Store
		sprintf(str, "sent log: \"%s\"", msg_out);
		PrintDB(str, millis());
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
	digitalWrite(pin.ttlPidRun, LOW);
	digitalWrite(pin.ttlPidStop, HIGH);
	DebugFlow("PID STOPPED");

	// Set flag
	fc.isRewarding = true;

	// Print
	char str[100];
	sprintf(str, "REWARDING(%dms)...", rewDur);
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

// TEST PIN MAPPING
void DebugPinMap()
{
	delay(5000);
	bool p_flow = db.print_flow;
	db.print_flow = true;
	digitalWrite(pin.ttlNorthOn, HIGH); DebugFlow("North TTL"); delay(1000); digitalWrite(pin.ttlNorthOn, LOW);
	digitalWrite(pin.ttlWestOn, HIGH); DebugFlow("West TTL"); delay(1000); digitalWrite(pin.ttlWestOn, LOW);
	digitalWrite(pin.ttlSouthOn, HIGH); DebugFlow("South TTL"); delay(1000); digitalWrite(pin.ttlSouthOn, LOW);
	digitalWrite(pin.ttlEastOn, HIGH); DebugFlow("East TTL"); delay(1000); digitalWrite(pin.ttlEastOn, LOW);
	digitalWrite(pin.ttlIR, HIGH); DebugFlow("IR Sync TTL"); delay(1000); digitalWrite(pin.ttlIR, LOW);
	digitalWrite(pin.ttlWhiteNoise, HIGH); DebugFlow("White Noise TTL"); delay(1000); digitalWrite(pin.ttlWhiteNoise, LOW);
	digitalWrite(pin.ttlRewTone, HIGH); DebugFlow("Reward Tone TTL"); delay(1000); digitalWrite(pin.ttlRewTone, LOW);
	digitalWrite(pin.ttlRewOn, HIGH); DebugFlow("Reward On TTL"); delay(1000); digitalWrite(pin.ttlRewOn, LOW);
	digitalWrite(pin.ttlRewOff, HIGH); DebugFlow("Reward Off TTL"); delay(1000); digitalWrite(pin.ttlRewOff, LOW);
	digitalWrite(pin.ttlPidRun, HIGH); DebugFlow("PID Run TTL"); delay(1000); digitalWrite(pin.ttlPidRun, LOW);
	digitalWrite(pin.ttlPidStop, HIGH); DebugFlow("PID Stop TTL"); delay(1000); digitalWrite(pin.ttlPidStop, LOW);
	digitalWrite(pin.ttlBullRun, HIGH); DebugFlow("Bull Run TTL"); delay(1000); digitalWrite(pin.ttlBullRun, LOW);
	digitalWrite(pin.ttlBullStop, HIGH); DebugFlow("Bull Stop TTL"); delay(1000); digitalWrite(pin.ttlBullStop, LOW);
	digitalWrite(pin.relIR, HIGH); DebugFlow("IR Sync Relay"); delay(1000); digitalWrite(pin.relIR, LOW);
	digitalWrite(pin.relRewTone, HIGH); DebugFlow("Reward Tone Relay"); delay(1000); digitalWrite(pin.relRewTone, LOW);
	digitalWrite(pin.relWhiteNoise, HIGH); DebugFlow("White Noise Relay"); delay(1000); digitalWrite(pin.relWhiteNoise, LOW);
	db.print_flow = p_flow;
	delay(5000);
}

// LOG/PRING MAIN EVENT
void DebugFlow(char msg[])
{
	DebugFlow(msg, millis());
}
void DebugFlow(char msg[], uint32_t t)
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
		char str[100];
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
		char str[100];
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

		char str[100];
		sprintf(str, "!!RESENT PACK: tot=%d id=%c dat=%d pack=%d!!", resent_cnt, id, dat, pack);
		if (do_print)
			PrintDB(str, millis());
		if (do_log)
			SendLogData(str, millis());

	}
}

// PRINT DB INFO
void PrintDB(char msg[], uint32_t t)
{
	static uint32_t t1 = millis();
	uint32_t t_m = 0;
	float t_s = 0;

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Get string with time
	char str[200] = { 0 };
	char str_tim[50] = { 0 };
	char spc[50] = { 0 };
	char arg[50] = { 0 };
	sprintf(str_tim, "[%0.2fs]", t_s);
	sprintf(arg, "%%%ds", 20 - strlen(str_tim));
	sprintf(spc, arg, '_');

	// Put it all together
	sprintf(str, "%s%s%s\n", str_tim, spc, msg);

	// Print
	SerialUSB.print(str);
}

#pragma endregion

#pragma region --------MINOR FUNCTIONS---------

// BLINK LEDS AT RESTART/UPLOAD
void ResetBlink()
{
	bool is_on = false;
	int dt = 100;
	// Flash 
	for (int i = 0; i < 10; i++)
	{
		digitalWrite(pin.relIR, is_on ? LOW : HIGH);
		delay(dt);
		is_on = !is_on;
	}
	// Reset LED
	digitalWrite(pin.relIR, LOW);
}

// PLAY SOUND WHEN QUITING
void QuitBleep()
{
	bool tone = true;
	for (int i = 0; i < 7; i++)
	{
		if (tone)
		{
			digitalWrite(pin.relRewTone, HIGH);
			digitalWrite(pin.relWhiteNoise, LOW);
		}
		else
		{
			digitalWrite(pin.relWhiteNoise, HIGH);
			digitalWrite(pin.relRewTone, LOW);
		}
		tone = !tone;
		delay(250);
	}
	digitalWrite(pin.relWhiteNoise, HIGH);
	digitalWrite(pin.relRewTone, LOW);
}

// PULSE IR
bool PulseIR()
{
	return PulseIR(-1, -1);
}
bool PulseIR(int del_sync, int dt_sync)
{
	// Local vars
	bool is_changed = false;

	// Set high
	if (
		!is_irOn &&
		millis() > t_syncLast + del_sync
		)
	{
		// Set ir pins on
		SetPort(word_irOn, 0x0);
		t_syncLast = millis();
		is_irOn = true;
		is_changed = true;
	}
	// Set low
	else if (
		is_irOn &&
		millis() > t_syncLast + dt_sync
		)
	{
		SetPort(0x0, word_irOn);
		is_irOn = false;
		is_changed = true;
	}
	return is_changed;
}

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
	if (isOnNorth && millis() - t_outLastNorth > dt_ttlPulse) {
		digitalWrite(pin.ttlNorthOn, LOW); // set back to LOW
		isOnNorth = false;
	}
	// west
	if (isOnWest && millis() - t_outLastWest > dt_ttlPulse) {
		digitalWrite(pin.ttlWestOn, LOW); // set back to LOW
		isOnWest = false;
	}
	// south
	if (isOnSouth && millis() - t_outLastSouth > dt_ttlPulse) {
		digitalWrite(pin.ttlSouthOn, LOW); // set back to LOW
		isOnSouth = false;
	}
	// east
	if (isOnEast && millis() - t_outLastEast > dt_ttlPulse) {
		digitalWrite(pin.ttlEastOn, LOW); // set back to LOW
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
			digitalWrite(pin.ttlNorthOn, HIGH);
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
			digitalWrite(pin.ttlWestOn, HIGH);
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
			digitalWrite(pin.ttlSouthOn, HIGH);
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
			digitalWrite(pin.ttlEastOn, HIGH);
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

#pragma endregion


void setup()
{
	// XBee
	Serial1.begin(57600);

	// CS through programming port
	Serial.begin(57600);

	// Set PT pin direction
	pinMode(pin.ptNorthOn, INPUT);
	pinMode(pin.ptWestOn, INPUT);
	pinMode(pin.ptSouthOn, INPUT);
	pinMode(pin.ptEastOn, INPUT);
	pinMode(pin.ttlNorthOn, OUTPUT);
	pinMode(pin.ttlWestOn, OUTPUT);
	pinMode(pin.ttlSouthOn, OUTPUT);
	pinMode(pin.ttlEastOn, OUTPUT);

	// Set PT pins low
	digitalWrite(pin.ttlNorthOn, LOW);
	digitalWrite(pin.ttlWestOn, LOW);
	digitalWrite(pin.ttlSouthOn, LOW);
	digitalWrite(pin.ttlEastOn, LOW);

	// Set relay/ttl pin to output
	pinMode(pin.relIR, OUTPUT);
	pinMode(pin.relWhiteNoise, OUTPUT);
	pinMode(pin.relRewTone, OUTPUT);
	pinMode(pin.ttlIR, OUTPUT);
	pinMode(pin.ttlRewTone, OUTPUT);
	pinMode(pin.ttlWhiteNoise, OUTPUT);

	// set relay/ttl pins low
	digitalWrite(pin.relIR, LOW);
	digitalWrite(pin.relRewTone, LOW);
	digitalWrite(pin.relWhiteNoise, LOW);
	digitalWrite(pin.ttlIR, LOW);
	digitalWrite(pin.ttlRewTone, LOW);
	digitalWrite(pin.ttlWhiteNoise, LOW);

	// set other output pins
	pinMode(pin.ttlRewOn, OUTPUT);
	pinMode(pin.ttlRewOff, OUTPUT);
	pinMode(pin.ttlBullRun, OUTPUT);
	pinMode(pin.ttlBullStop, OUTPUT);
	pinMode(pin.ttlPidRun, OUTPUT);
	pinMode(pin.ttlPidStop, OUTPUT);

	// Setup reward ttl stuff on port C
	REG_PIOC_OWER = 0xFFFFFFFF;     // enable PORT C
	REG_PIOC_OER = 0xFFFFFFFF;     // set PORT C as output port

								   // Get ir word
	int sam_ir_pins[2] = { pin.sam_ttlIR, pin.sam_relIR };
	word_irOn = GetPortWord(0x0, sam_ir_pins, 2);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(pin.ptNorthOn), NorthInterrupt, RISING); // north
	attachInterrupt(digitalPinToInterrupt(pin.ptWestOn), WestInterrupt, RISING); // west
	attachInterrupt(digitalPinToInterrupt(pin.ptSouthOn), SouthInterrupt, RISING); // south
	attachInterrupt(digitalPinToInterrupt(pin.ptEastOn), EastInterrupt, RISING); // east

	// Test pin mapping
	/*
	Note: make sure Cheetah aquiring
	*/
	if (doTestPinMapping)
		DebugPinMap();

	// Dump buffers
	while (Serial.available())
		Serial.read();
	while (Serial1.available())
		Serial1.read();

	// Blink LEDs at setup
	ResetBlink();

}


void loop()
{

	// Keep checking for start command
	AwaitStart();

	// Check for XBee input
	r2a.isNew = false;
	if (Serial1.available() > 0) {
		r2a.isNew = ParseSerial();
	}


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
				int sam_on_pins[3] = { pin.sam_relRewTone, pin.sam_ttlRewTone, pin.sam_ttlRewOn };
				word_rewOn = GetPortWord(0x0, sam_on_pins, 3);
				int sam_off_pins[3] = { pin.sam_relWhiteNoise, pin.sam_ttlWhiteNoise, pin.sam_ttlRewOff };
				word_rewOff = GetPortWord(0x0, sam_off_pins, 3);
			}
			else {
				// rew event pins only
				int sam_on_pins[1] = { pin.sam_ttlRewOn };
				word_rewOn = GetPortWord(0x0, sam_on_pins, 1);
				int sam_off_pins[1] = { pin.sam_ttlRewOff };
				word_rewOff = GetPortWord(0x0, sam_off_pins, 1);
			}
			// Turn on white noise
			if (fc.doWhiteNoise) {
				// turn white on
				int sam_white_pins[2] = { pin.sam_relWhiteNoise, pin.sam_ttlWhiteNoise };
				uint32_t word_white = GetPortWord(0x0, sam_white_pins, 2);
				SetPort(word_white, 0x0);
			}

		}

		// Signal PID mode
		else if (r2a.idNew == 'p') {
			// Signal PID stopped
			if (r2a.dat == 0)
			{
				digitalWrite(pin.ttlPidStop, HIGH);
				digitalWrite(pin.ttlPidRun, LOW);
				DebugFlow("PID STOPPED");
			}
			// Signal PID running
			else if (r2a.dat == 1)
			{
				digitalWrite(pin.ttlPidRun, HIGH);
				digitalWrite(pin.ttlPidStop, LOW);
				DebugFlow("PID RUNNING");
			}
		}

		// Signal bull running
		else if (r2a.idNew == 'b') {
			// Signal Bull stopped
			if (r2a.dat == 0)
			{
				digitalWrite(pin.ttlBullStop, HIGH);
				digitalWrite(pin.ttlBullRun, LOW);
				DebugFlow("BULL STOPPED");
			}
			// Signal Bull running
			else if (r2a.dat == 1)
			{
				digitalWrite(pin.ttlBullRun, HIGH);
				digitalWrite(pin.ttlBullStop, LOW);
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
	if (PulseIR(del_syncPulse, dt_syncPulse))
	{
		if (is_irOn)
			DebugFlow("IR SYNC ON");
		else
			DebugFlow("IR SYNC OFF");
	}

	// Check if ready to quit
	if (fc.doQuit && !doPackSend)
	{
		// Run bleep bleep
		QuitBleep();
		// Restart Arduino
		REQUEST_EXTERNAL_RESET;
	}

	// Send message confirmation
	if (doPackSend)
	{
		SendPacketData(r2a.idNew, 255, r2a.packLast[CharInd(r2a.idNew, r2a.idList, r2a.idLng)], false);
	}

}
