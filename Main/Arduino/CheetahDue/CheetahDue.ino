//######################################

//============ CHEETAHDUE ==============

//######################################


#pragma region ========= LIBRARIES & EXT DEFS ==========

//-------SOFTWARE RESET----------
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

//----------LIBRARIES------------

// General
#include <string.h>
//
#include <MemoryFree.h>

#pragma endregion 


#pragma region ============ DEBUG SETTINGS =============

struct DB
{
	// Do log
	bool Log = true;
	// What to print
	bool log_flow = true;
	bool log_errors = true;
	bool log_r2a = true;
	bool log_a2r = true;
	bool log_resent = true;

	// Print to console
	bool Console = false;
	// What to print
	bool print_flow = true;
	bool print_errors = true;
	bool print_r2a = true;
	bool print_a2r = true;
	bool print_resent = true;
	bool print_log = false;
}
// Initialize
db;

// TESTING 

// Print pin mapping
const bool doTestPinMapping = false;

#pragma endregion 


#pragma region ============ VARIABLE SETUP =============

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

// Debugging general
uint32_t cnt_loop_tot = 0;
uint16_t cnt_loop_short = 0;

// Log debugging
const int logQueueSize = 15;
char logQueue[logQueueSize][300] = { { 0 } };
int logQueueIndStore = 0;
int logQueueIndRead = 0;
int cnt_logsStored = 0;

// Print debugging
const int printQueueSize = 15;
char printQueue[printQueueSize][300] = { { 0 } };
int printQueueIndStore = 0;
int printQueueIndRead = 0;

// Serial tracking
const int sendQueueSize = 10;
const int sendQueueBytes = 9;
byte sendQueue[sendQueueSize][sendQueueBytes] = { { 0 } };
int sendQueueIndStore = 0;
int sendQueueIndRead = 0;
const int dt_sendSent = 1; // (ms) 
const int dt_sendRcvd = 1; // (ms) 
uint32_t t_sent = millis(); // (ms)
uint32_t t_rcvd = millis(); // (ms)
int cnt_packBytesRead = 0;
int cnt_packBytesSent = 0;
int cnt_packBytesDiscarded = 0;
int cnt_logBytesSent = 0;

// Serial from rob
struct R2A
{
	const char id[5] = {
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
	};
	const char head = '{';
	const char foot = '}';
	char idNow = ' ';
	bool isNew = false;
	byte dat[3] = { 0 };
	const static int lng = sizeof(id) / sizeof(id[0]);
	uint16_t pack[lng] = { 0 };
	uint16_t packLast[lng] = { 0 };
	int cnt_repeat = 0;
	int cnt_dropped = 0;
}
// Initialize
r2a;

// Serial to CS
struct R2C
{
	const char head = '<';
	const char foot = '>';
	const char id[16] = {
		'h', // Setup handshake
		'T', // system test command
		'S', // start session
		'Q', // quit session
		'M', // move to position
		'R', // run reward
		'H', // halt movement
		'B', // bulldoze rat
		'I', // rat in/out
		'V', // connected and streaming
		'L', // request log conf/send
		'J', // battery voltage
		'Z', // reward zone
		'U', // log size
		'D', // execution done
		'P', // position data
	};
}
// Initialize
r2c;

// Serial to rob
struct A2R
{
	const char id[5] = {
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
	};
	const char head = '{';
	const char foot = '}';
	byte dat[3] = { 0 };
	const static int lng = sizeof(id) / sizeof(id[0]);
	uint16_t pack[lng] = { 0 };
	uint16_t packLast[lng] = { 0 };
	int cnt_repeat = 0;
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

// Start/Quit
uint32_t t_quit = 0;

// Reward
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;
uint32_t word_rewOn;
uint32_t word_rewOff;

// IR time sync LED
const uint32_t dt_irSyncPulse = 10; // (ms)
uint32_t del_irSyncPulse = 60000; // (ms)
uint32_t t_sync = 0;
bool is_irOn = false;
int cnt_ir = 0;
uint32_t t_irSyncLast;
uint32_t word_irOn;

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
// Any pin
volatile bool isOnAny = false;

// TTL timers
uint32_t t_debounce = 10; // (ms)
uint32_t dt_ttlPulse = 50; // (ms)

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

// CHECK FOR START COMMAND
bool CheckForStart();
// PARSE SERIAL INPUT
void GetSerial();
// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char mtch = '\0');
// STORE PACKET DATA TO BE SENT
void QueuePacket(char id, byte dat1, byte dat2, byte dat3, uint16_t pack, bool do_conf);
// SEND SERIAL PACKET DATA
bool SendPacket();
// STORE LOG STRING
void QueueLog(char msg[], uint32_t t);
// SEND LOG DATA OVER SERIAL
bool SendLog();
// START REWARD
void StartRew();
// END REWARD
void EndRew();
// TEST PIN MAPPING
void DebugPinMap();
// LOG/PRING MAIN EVENT
void DebugFlow(char msg[], uint32_t t = millis());
// LOG/PRINT ERRORS
void DebugError(char msg[], uint32_t t = millis());
// PRINT RECIEVED PACKET
void DebugRcvd(char id, byte dat[], uint16_t pack, bool do_conf, int buff_rx, int buff_tx, bool is_repeat = false);
// LOG/PRING SENT PACKET DEBUG STRING
void DebugSent(char id, byte dat[], uint16_t pack, bool do_conf, int buff_tx, int buff_rx, bool is_repeat = false);
// PRINT RESENT PACKET
void DebugResent(char id, byte dat[], uint16_t pack);
// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t);
// PRINT DB INFO
bool PrintDebug();
// SEND TEST PACKET
void TestSendPack(char id, byte dat1, byte dat2, byte dat3, uint16_t pack, bool do_conf);
// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink();
// PLAY SOUND WHEN QUITING
void QuitBeep();
// PULSE IR
bool PulseIR(int del_sync = 1, int dt_sync = 1);
// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size);
// GET 32 BIT WORD FOR PORT
uint32_t GetPortWord(uint32_t word, int pin_arr[], int arr_size);
// SET PORT
void SetPort(uint32_t word_on, uint32_t word_off);
// Reset PT pins
void ResetTTL();
// North
void NorthInterrupt();
void NorthFun();
// West
void WestInterrupt();
void WestFun();
// South
void SouthInterrupt();
void SouthFun();
// East
void EastInterrupt();
void EastFun();

#pragma endregion


#pragma region ========== FUNCTION DEFINITIONS =========

#pragma region --------COMMUNICATION---------

// CHECK FOR START COMMAND
bool CheckForStart()
{
	// Local vars
	bool is_rcvd = false;
	char str[200] = { 0 };
	byte in_byte[1] = { 0 };
	byte out_byte[1] = { 'h' };

	// Bail if session started
	if (fc.isSesStarted) {
		return true;
	}

	// Check if IR should be pulsed 
	PulseIR(1000, dt_irSyncPulse);

	// Bail if no new data
	if (Serial.available() < 1) {
		return false;
	}

	// Get new data
	in_byte[0] = Serial.read();

	// Bail if not a match
	if (in_byte[0] != out_byte[0]) {
		return false;
	}

	// Store time
	t_sync = millis();

	// Pulse ir
	if (!is_irOn) {
		PulseIR(); // turn on
	}
	delay(10 - (millis() - t_irSyncLast));
	PulseIR();  // turn off
	delay(75);
	PulseIR(); // turn on
	delay(10);
	PulseIR();  // turn off

	// Dump buffer
	while (Serial.available()) {
		Serial.read();
	}

	// Log success
	DebugFlow("[CheckForStart] HANDSHAKE SUCCEEDED");

	// Log sync time
	sprintf(str, "SET SYNC TIME: %dms", t_sync);
	DebugFlow(str, t_sync);

	// Set flags
	is_rcvd = true;
	fc.isSesStarted = true;
	DebugFlow("[CheckForStart] START SESSION");

	// Dump Xbee buffer
	while (Serial1.available() > 0) {
		Serial1.read();
	}

	// Return success
	return true;
}

// PARSE SERIAL INPUT
void GetSerial()
{
	/*
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	uint32_t t_str = millis();
	char str[200] = { 0 };
	char dat_str[200] = { 0 };
	int buff_tx = 0;
	int buff_rx = 0;
	byte buff = 0;
	char head = ' ';
	char id = ' ';
	byte dat[3] = { 0 };
	char foot = ' ';
	bool do_conf = false;
	uint16_t pack = 0;

	// Reset vars
	cnt_packBytesRead = 0;
	cnt_packBytesDiscarded = 0;
	r2a.isNew = false;
	r2a.idNow = ' ';

	// Bail if no new input
	if (Serial1.available() == 0) {
		return;
	}

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(r2a.head);
	if (buff == 0) {
		return;
	}

	// Store header
	head = buff;

	// Get id
	id = WaitBuffRead();

	// Get data
	dat[0] = WaitBuffRead();
	dat[1] = WaitBuffRead();
	dat[2] = WaitBuffRead();

	// Get packet num
	U.f = 0.0f;
	U.b[0] = WaitBuffRead();
	U.b[1] = WaitBuffRead();
	pack = U.i16[0];

	// Get recieved confirmation
	U.f = 0.0f;
	U.b[0] = WaitBuffRead();
	do_conf = U.b[0] != 0 ? true : false;

	// Get footer
	foot = WaitBuffRead();

	// Get total data in buffers
	buff_rx = Serial1.available();
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();

	// Store data string
	sprintf(dat_str, "head=%c id=\'%c\' dat=|%d|%d|%d pack=%d foot=%c bytes_read=%d bytes_discarded=%d rx=%d tx=%d dt_parse=%d",
		head, id, dat[0], dat[1], dat[2], pack, foot, cnt_packBytesRead, cnt_packBytesDiscarded, buff_rx, buff_tx, millis() - t_str);

	// Check for missing footer
	if (foot != r2a.foot) {

		// Itterate dropped count
		r2a.cnt_dropped++;

		// Log/print dropped packet info
		sprintf(str, "**WARNING** [GetSerial] Dropped r2a Packet: cnt=%d ", r2a.cnt_dropped);
		strcat(str, dat_str);
		DebugError(str);
		return;

	}
	else
	{
		// Update recive time
		t_rcvd = millis();

		// Update last packet
		int id_ind = CharInd(id, r2a.id, r2a.lng);
		r2a.packLast[id_ind] = r2a.pack[id_ind];
		r2a.pack[id_ind] = pack;

		// Check if packet is new
		if (r2a.packLast[id_ind] != pack)
		{
			// Print received packet
			DebugRcvd(id, dat, pack, do_conf, buff_rx, buff_tx);

			// Update struct vars
			r2a.isNew = true;
			r2a.idNow = id;
			r2a.dat[0] = dat[0];
			r2a.dat[1] = dat[1];
			r2a.dat[2] = dat[2];
		}
		else
		{
			// Itterate count
			r2a.cnt_repeat++;

			// Print resent packet
			DebugRcvd(id, dat, pack, do_conf, buff_rx, buff_tx, true);
		}

		// Send confirmation
		if (do_conf) {
			QueuePacket(id, dat[0], dat[1], dat[2], pack, false);
		}

	}

	// Check if data was discarded
	if (cnt_packBytesDiscarded > 0) {

		// Log/print discarded data
		sprintf(str, "**WARNING** [GetSerial] Discarded Bytes: %s", dat_str);
		DebugError(str);
	}

	// Check if parsing took unusually long
	if (millis() - t_str > 30) {
		sprintf(str, "**WARNING** [GetSerial] Parser Hanging: %s", dat_str);
		DebugError(str);
	}

}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char mtch)
{
	// Local vars
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	static int cnt_overflow = 0;
	static int cnt_timeout = 0;
	static bool is_fail_last = false;
	bool is_bytes_discarded = false;
	bool is_overflowed = false;
	bool is_r2c_pack = false;
	byte buff = 0;

	// Get total data in buffers now
	int buff_rx_start = Serial1.available();

	// Check for overflow
	is_overflowed = buff_rx_start >= SERIAL_BUFFER_SIZE - 1;

	// Wait for at least 1 byte
	while (Serial1.available() < 1 &&
		millis() < t_timeout);

	// Get any byte
	if (!is_overflowed &&
		mtch == '\0') {

		if (Serial1.available() > 0) {

			buff = Serial1.read();
			cnt_packBytesRead++;
			is_fail_last = false;
			return buff;
		}
	}

	// Find specific byte
	while (
		buff != mtch  &&
		millis() < t_timeout &&
		!is_overflowed &&
		!is_r2c_pack) {

		// Check new data
		if (Serial1.available() > 0) {

			buff = Serial1.read();
			cnt_packBytesRead++;

			// check match was found
			if (buff == mtch) {
				is_fail_last = false;
				return buff;
			}

			// Otherwise add to discard count
			else {
				cnt_packBytesDiscarded++;
				is_bytes_discarded = true;
			}

			// Check for r2c packet
			if (buff == r2c.head) {
				is_r2c_pack = true;
				break;
			}

			// Check for overflow
			is_overflowed =
				!is_overflowed ? Serial1.available() >= SERIAL_BUFFER_SIZE - 1 : is_overflowed;
		}

	}

	// Print issue
	char msg_str[200];
	char dat_str[200];

	// Check if buffer flooded
	if (is_overflowed) {

		// DUMP IT ALL
		while (Serial1.available() > 0) {
			if (Serial1.available() > 0) {
				Serial1.read();
				cnt_packBytesRead++;
				cnt_packBytesDiscarded++;
			}
		}
	}

	// Get buffer 
	int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	int buff_rx = Serial1.available();

	// Handl r2c packet
	if (is_r2c_pack) {

		// Dump data till footer found
		while (millis() < t_timeout &&
			buff != r2c.foot &&
			buff != mtch) {

			if (Serial1.available() > 0) {
				buff = Serial1.read();
				cnt_packBytesRead++;
				cnt_packBytesDiscarded++;
			}
		}
	}

	// Store current info
	char buff_print = buff == 10 ? 'n' : buff == 13 ? 'r' : buff;
	sprintf(dat_str, " buff=%c bytes_read=%d bytes_discarded=%d rx_start=%d rx_now=%d tx_now=%d dt_check=%d",
		buff_print, cnt_packBytesRead, cnt_packBytesDiscarded, buff_rx_start, buff_rx, buff_tx, (millis() - t_timeout) + timeout);

	// Received r2c packet
	if (is_r2c_pack) {
		// Log/print discarded data
		sprintf(msg_str, "[WaitBuffRead] Received r2c Packet: %s", dat_str);
		DebugFlow(msg_str);

		// Check if match still found
		is_fail_last = true;
		if (buff == mtch)
			return buff;
		else
			return 0;
	}

	// Buffer flooded
	if (is_overflowed) {
		cnt_overflow++;
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Buffer Overflowed: cnt=%d", cnt_overflow);
	}
	// Timed out
	else if (millis() > t_timeout) {
		cnt_timeout++;
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Timedout: cnt=%d", cnt_timeout);
	}
	// Byte not found
	else if (mtch != '\0' && !is_fail_last) {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Char %c Not Found:", mtch);
	}
	// Getting r2c log data
	else if (mtch != '\0' && is_fail_last) {
		// Only print once
		static bool is_print_once = false;
		if (!is_print_once) {
			sprintf(msg_str, "[WaitBuffRead] Likely Dumping Robot Log Data:");
			is_print_once = true;
		}
		else {
			is_fail_last = true;
			return 0;
		}
	}
	// Failed for unknown reason
	else {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Failed for Unknown Reason:");
	}

	// Compbine strings
	strcat(msg_str, dat_str);

	// Log/print error
	DebugError(msg_str);

	// Set buff to '\0' ((byte)-1) if !pass
	is_fail_last = true;
	return 0;

}

// STORE PACKET DATA TO BE SENT
void QueuePacket(char id, byte dat1, byte dat2, byte dat3, uint16_t pack, bool do_conf)
{
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	char str[200] = { 0 };
	int id_ind = 0;

	// Update sendQueue ind
	sendQueueIndStore++;

	// Check if ind should roll over 
	if (sendQueueIndStore == sendQueueSize) {

		// Reset queueIndWrite
		sendQueueIndStore = 0;
	}

	// Check if overfloweed
	if (sendQueue[sendQueueIndStore][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[sendQueueSize + 1];
		for (int i = 0; i < sendQueueSize; i++) {
			queue_state[i] = sendQueue[i][0] == '\0' ? '0' : '1';
		}

		// Get buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		int buff_rx = Serial1.available();

		// Store overflow error instead
		sprintf(str, "!!ERRROR!! [QueuePacket] SEND QUEUE OVERFLOWED: sendQueueIndStore=%d sendQueueIndRead=%d queue_state=|%s| dt_sent=%d dt_rcvd=%d buff_tx=%d buff_rx=%d",
			sendQueueIndStore, sendQueueIndRead, queue_state, millis() - t_sent, millis() - t_rcvd, buff_tx, buff_rx);

		// Log/print error
		DebugError(str);

		// Set queue back 
		sendQueueIndStore = sendQueueIndStore - 1 >= 0 ? sendQueueIndStore - 1 : sendQueueSize - 1;

		// Bail
		return;

	}

	// Create byte packet
	int b_ind = 0;
	// Store header
	sendQueue[sendQueueIndStore][b_ind++] = a2r.head;
	// Store mesage id
	sendQueue[sendQueueIndStore][b_ind++] = id;
	// Store mesage data 
	sendQueue[sendQueueIndStore][b_ind++] = dat1;
	sendQueue[sendQueueIndStore][b_ind++] = dat2;
	sendQueue[sendQueueIndStore][b_ind++] = dat3;
	// Store packet number
	U.f = 0.0f;
	U.i16[0] = pack;
	sendQueue[sendQueueIndStore][b_ind++] = U.b[0];
	sendQueue[sendQueueIndStore][b_ind++] = U.b[1];
	// Store get_confirm request
	sendQueue[sendQueueIndStore][b_ind++] = do_conf ? 1 : 0;
	// Store footer
	sendQueue[sendQueueIndStore][b_ind++] = a2r.foot;

}

// SEND SERIAL PACKET DATA
bool SendPacket()
{
	/*
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	const int msg_lng = sendQueueBytes;
	byte msg[msg_lng];
	cnt_packBytesSent = 0;
	bool is_repeat = false;
	char id = '\0';
	byte dat[3] = { 0 };
	bool do_conf = 0;
	uint16_t pack = 0;

	// Bail if nothing in queue
	if (sendQueueIndRead == sendQueueIndStore &&
		sendQueue[sendQueueIndStore][0] == '\0') {
		return false;
	}

	// Bail if buffer or time inadequate
	if (Serial1.availableForWrite() < sendQueueBytes + 10 ||
		Serial1.available() > 0 ||
		millis() < t_sent + dt_sendSent ||
		millis() < t_rcvd + dt_sendRcvd) {

		// Indicate still packs to send
		return true;
	}

	// Itterate send ind
	sendQueueIndRead++;

	// Check if ind should roll over 
	if (sendQueueIndRead == sendQueueSize) {
		sendQueueIndRead = 0;
	}

	// Send
	Serial1.write(sendQueue[sendQueueIndRead], msg_lng);
	t_sent = millis();
	cnt_packBytesSent = msg_lng;

	// Get buffers
	int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	int buff_rx = Serial1.available();

	// pull out packet data
	int b_ind = 1;
	// id
	id = sendQueue[sendQueueIndRead][b_ind++];
	// dat
	dat[0] = sendQueue[sendQueueIndRead][b_ind++];
	dat[1] = sendQueue[sendQueueIndRead][b_ind++];
	dat[2] = sendQueue[sendQueueIndRead][b_ind++];
	// pack
	U.f = 0.0f;
	U.b[0] = sendQueue[sendQueueIndRead][b_ind++];
	U.b[1] = sendQueue[sendQueueIndRead][b_ind++];
	pack = U.i16[0];
	// do_conf 
	do_conf = sendQueue[sendQueueIndRead][b_ind++] == 1 ? true : false;

	// Check if resending
	is_repeat = pack == r2a.packLast[CharInd(id, r2a.id, r2a.lng)];

	// Print sent
	DebugSent(id, dat, pack, do_conf, buff_tx, buff_rx, is_repeat);

	// Set entry to null
	sendQueue[sendQueueIndRead][0] = '\0';

	// Return success
	return true;

}

// STORE LOG STRING
void QueueLog(char msg[], uint32_t t)
{
	/*
	STORE LOG DATA FOR CS
	FORMAT: head,chksum,"[log_cnt],loop,ts_ms,message",foot
	*/

	// Local vars
	uint32_t t_m = 0;
	char str[200] = { 0 };
	char msg_out[200] = { 0 };
	byte chksum = 0;

	// Update logQueue ind
	logQueueIndStore++;

	// Check if ind should roll over 
	if (logQueueIndStore == logQueueSize) {

		// Reset queueIndWrite
		logQueueIndStore = 0;
	}

	// Check if overfloweed
	if (logQueue[logQueueIndStore][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[logQueueSize + 1];
		for (int i = 0; i < logQueueSize; i++) {
			queue_state[i] = logQueue[i][0] == '\0' ? '0' : '1';
		}

		// Store overflow error instead
		sprintf(msg, "**WARNING** [QueueLog] LOG QUEUE OVERFLOWED: logQueueIndStore=%d logQueueIndRead=%d queue_state=|%s|",
			logQueueIndStore, logQueueIndRead, queue_state);

		// Set queue back so overflow will write over last log
		logQueueIndStore = logQueueIndStore - 1 >= 0 ? logQueueIndStore - 1 : logQueueSize - 1;

		// Print error
		if (db.print_errors && db.Console) {
			QueueDebug(msg, t);
		}

	}
	// Update log count
	else {
		cnt_logsStored++;
	}

	// Get sync correction
	t_m = t - t_sync;

	// Put it all together
	sprintf(str, "[%d],%lu,%d,%s", cnt_logsStored, t_m, cnt_loop_short, msg);

	// Get message size
	chksum = strlen(str);

	// Add header, chksum and footer
	int b_ind = 0;
	// head
	msg_out[b_ind++] = a2c.head;
	// checksum
	msg_out[b_ind++] = chksum;
	// msg
	for (int i = 0; i < chksum; i++) {
		msg_out[b_ind++] = str[i];
	}
	// foot
	msg_out[b_ind++] = a2c.foot;
	// null
	msg_out[b_ind] = '\0';

	// Store log
	sprintf(logQueue[logQueueIndStore], "%s", msg_out);

}

// SEND LOG DATA OVER SERIAL
bool SendLog()
{
	/*
	STORE LOG DATA FOR CS
	FORMAT: head,chksum,"[log_cnt],loop,ts_ms,message",foot
	*/
	// Local vars
	int msg_lng = 0;
	char str[200] = { 0 };
	cnt_logBytesSent = 0;

	// Bail if serial not established or no logs to store
	if (!fc.isSesStarted ||
		logQueueIndRead == logQueueIndStore &&
		logQueue[logQueueIndStore][0] == '\0') {
		return false;
	}

	// Bail if buffer or time inadequate
	if (Serial1.available() > 0 ||
		millis() < t_sent + dt_sendSent ||
		millis() < t_rcvd + dt_sendRcvd) {

		// Indicate still logs to send
		return true;
	}

	// Itterate send ind
	logQueueIndRead++;

	// Check if ind should roll over 
	if (logQueueIndRead == logQueueSize) {
		logQueueIndRead = 0;
	}

	// Get message size
	msg_lng = strlen(logQueue[logQueueIndRead]);

	// Send
	Serial.write(logQueue[logQueueIndRead], msg_lng);
	t_sent = millis();
	cnt_logBytesSent += msg_lng;

	// Print
	if (db.print_log && db.Console)
	{
		// Get data in buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial.availableForWrite();
		int buff_rx = Serial.available();

		// Print stored log
		char str[300];
		sprintf(str, "   [LOG] a2c: log_cnt=%d bytes_sent=%d msg=\"%s\"",
			cnt_logsStored, cnt_logBytesSent, logQueue[logQueueIndRead]);
		QueueDebug(str, millis());
	}

	// Set entry to null
	logQueue[logQueueIndRead][0] = '\0';

	// Return success
	return true;
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
	DebugFlow("[StartRew] PID STOPPED");

	// Set flag
	fc.isRewarding = true;

	// Print
	char str[200];
	sprintf(str, "[StartRew] REWARDING(%dms)...", rewDur);
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
		DebugFlow("[EndRew] REWARD OFF");

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
	digitalWrite(pin.ttlNorthOn, HIGH); DebugFlow("[DebugPinMap] North TTL"); delay(1000); digitalWrite(pin.ttlNorthOn, LOW);
	digitalWrite(pin.ttlWestOn, HIGH); DebugFlow("[DebugPinMap] West TTL"); delay(1000); digitalWrite(pin.ttlWestOn, LOW);
	digitalWrite(pin.ttlSouthOn, HIGH); DebugFlow("[DebugPinMap] South TTL"); delay(1000); digitalWrite(pin.ttlSouthOn, LOW);
	digitalWrite(pin.ttlEastOn, HIGH); DebugFlow("[DebugPinMap] East TTL"); delay(1000); digitalWrite(pin.ttlEastOn, LOW);
	digitalWrite(pin.ttlIR, HIGH); DebugFlow("[DebugPinMap] IR Sync TTL"); delay(1000); digitalWrite(pin.ttlIR, LOW);
	digitalWrite(pin.ttlWhiteNoise, HIGH); DebugFlow("[DebugPinMap] White Noise TTL"); delay(1000); digitalWrite(pin.ttlWhiteNoise, LOW);
	digitalWrite(pin.ttlRewTone, HIGH); DebugFlow("[DebugPinMap] Reward Tone TTL"); delay(1000); digitalWrite(pin.ttlRewTone, LOW);
	digitalWrite(pin.ttlRewOn, HIGH); DebugFlow("[DebugPinMap] Reward On TTL"); delay(1000); digitalWrite(pin.ttlRewOn, LOW);
	digitalWrite(pin.ttlRewOff, HIGH); DebugFlow("[DebugPinMap] Reward Off TTL"); delay(1000); digitalWrite(pin.ttlRewOff, LOW);
	digitalWrite(pin.ttlPidRun, HIGH); DebugFlow("[DebugPinMap] PID Run TTL"); delay(1000); digitalWrite(pin.ttlPidRun, LOW);
	digitalWrite(pin.ttlPidStop, HIGH); DebugFlow("[DebugPinMap] PID Stop TTL"); delay(1000); digitalWrite(pin.ttlPidStop, LOW);
	digitalWrite(pin.ttlBullRun, HIGH); DebugFlow("[DebugPinMap] Bull Run TTL"); delay(1000); digitalWrite(pin.ttlBullRun, LOW);
	digitalWrite(pin.ttlBullStop, HIGH); DebugFlow("[DebugPinMap] Bull Stop TTL"); delay(1000); digitalWrite(pin.ttlBullStop, LOW);
	digitalWrite(pin.relIR, HIGH); DebugFlow("[DebugPinMap] IR Sync Relay"); delay(1000); digitalWrite(pin.relIR, LOW);
	digitalWrite(pin.relRewTone, HIGH); DebugFlow("[DebugPinMap] Reward Tone Relay"); delay(1000); digitalWrite(pin.relRewTone, LOW);
	digitalWrite(pin.relWhiteNoise, HIGH); DebugFlow("[DebugPinMap] White Noise Relay"); delay(1000); digitalWrite(pin.relWhiteNoise, LOW);
	db.print_flow = p_flow;
	delay(5000);
}

// LOG/PRING MAIN EVENT
void DebugFlow(char msg[], uint32_t t)
{
	// Local vars
	bool do_print = db.Console && db.print_flow;
	bool do_log = db.Log && db.log_flow;

	if (do_print) {
		QueueDebug(msg, millis());
	}

	if (do_log) {
		QueueLog(msg, millis());
	}

}

// LOG/PRINT ERRORS
void DebugError(char msg[], uint32_t t)
{
	// Local vars
	bool do_print = db.print_errors && db.Console;
	bool do_log = db.log_errors && db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg, t);
	}

	// Add to log queue
	if (do_log) {
		QueueLog(msg, t);
	}

}

// PRINT RECIEVED PACKET
void DebugRcvd(char id, byte dat[], uint16_t pack, bool do_conf, int buff_rx, int buff_tx, bool is_repeat)
{
	// Local vars
	bool do_print = db.Console && db.print_r2a;
	bool do_log = db.Log && db.log_r2a;

	// Print/Log
	if (!(do_print || do_log)) {
		return;
	}

	// Check if this is a repeat
	char msg[100];
	if (!is_repeat) {
		sprintf(msg, "   [RCVD] r2a: ");
	}
	else {
		sprintf(msg, "   [*RE-RCVD*] r2a: cnt=%d ", r2a.cnt_repeat);
	}

	char str[200];
	sprintf(str, "id=\'%c\' dat=|%d|%d|%d| pack=%d do_conf=%s bytes_read=%d rx=%d tx=%d",
		id, dat[0], dat[1], dat[2], pack, do_conf ? "true" : "false", cnt_packBytesRead, buff_rx, buff_tx);

	// Concatinate strings
	strcat(msg, str);

	if (do_print) {
		QueueDebug(msg, t_rcvd);
	}

	if (do_log) {
		QueueLog(msg, t_rcvd);
	}

}

// LOG/PRING SENT PACKET DEBUG STRING
void DebugSent(char id, byte dat[], uint16_t pack, bool do_conf, int buff_tx, int buff_rx, bool is_repeat)
{
	// Local vars
	bool do_print = db.Console && db.print_a2r;
	bool do_log = db.Log && db.log_a2r;
	int cnt_queued = sendQueueSize - sendQueueIndStore - 1;

	// Print/Log
	if (!(do_print || do_log)) {
		return;
	}

	// Check if this is a repeat
	char msg[100];
	if (!is_repeat) {
		sprintf(msg, "   [SENT] a2r: ");
	}
	else {
		sprintf(msg, "   [*RE-SENT*] a2r: cnt=%d ", a2r.cnt_repeat);
	}

	// Make string
	char str[200];
	sprintf(str, "id=\'%c\' dat=|%d|%d|%d| pack=%d do_conf=%s bytes_sent=%d tx=%d rx=%d queued=%d",
		id, dat[0], dat[1], dat[2], pack, do_conf ? "true" : "false", cnt_packBytesSent, buff_tx, buff_rx, cnt_queued);

	// Concatinate strings
	strcat(msg, str);

	if (do_print) {
		QueueDebug(msg, t_sent);
	}

	if (do_log) {
		QueueLog(msg, t_sent);
	}

}

// PRINT RESENT PACKET
void DebugResent(char id, byte dat[], uint16_t pack)
{
	// Local vars
	bool do_print = db.Console && db.print_resent;
	bool do_log = db.Log && db.log_resent;
	static int cnt_repeat = 0;

	// Print/Log
	if (!(do_print || do_log)) {
		return;
	}

	// Itterate count
	cnt_repeat++;

	char str[200];
	sprintf(str, "   [*RE-RCVD*] r2a: tot=%d id=\'%c\' dat=|%d|%d|%d| pack=%d!!", cnt_repeat, id, dat[0], dat[1], dat[2], pack);

	if (do_print) {
		QueueDebug(str, millis());
	}

	if (do_log) {
		QueueLog(str, millis());
	}
}

// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t)
{
	// Local vars
	uint32_t t_m = 0;
	float t_s = 0;
	char str[200] = { 0 };
	char str_tim[100] = { 0 };

	// Update printQueue ind
	printQueueIndStore++;

	// Check if ind should roll over 
	if (printQueueIndStore == printQueueSize) {

		// Reset queueIndWrite
		printQueueIndStore = 0;
	}

	// Check if overfloweed
	if (printQueue[printQueueIndStore][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[printQueueSize + 1];
		for (int i = 0; i < printQueueSize; i++) {
			queue_state[i] = printQueue[i][0] == '\0' ? '0' : '1';
		}

		// Store overflow error instead
		sprintf(msg, "**WARNING** [QueueDebug] PRINT QUEUE OVERFLOWED: printQueueIndStore=%d printQueueIndRead=%d queue_state=|%s|",
			printQueueIndStore, printQueueIndRead, queue_state);

		// Set queue back so overflow will write over last print
		printQueueIndStore = printQueueIndStore - 1 >= 0 ? printQueueIndStore - 1 : printQueueSize - 1;

		// Log error
		if (db.log_errors && db.Log) {
			QueueLog(msg, t);
		}

	}

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Make time string
	sprintf(str_tim, "[%0.3f][%d]", t_s, cnt_loop_short);

	// Add space after time
	char spc[50] = { 0 };
	char arg[50] = { 0 };
	sprintf(arg, "%%%ds", 20 - strlen(str_tim));
	sprintf(spc, arg, '_');

	// Put it all together
	sprintf(printQueue[printQueueIndStore], "%s%s%s\n", str_tim, spc, msg);

}

// PRINT DB INFO
bool PrintDebug()
{

	// Bail if nothing in queue
	if (printQueueIndRead == printQueueIndStore &&
		printQueue[printQueueIndStore][0] == '\0') {
		return false;
	}

	// Itterate send ind
	printQueueIndRead++;

	// Check if ind should roll over 
	if (printQueueIndRead == printQueueSize) {
		printQueueIndRead = 0;
	}

	// Print
	SerialUSB.print(printQueue[printQueueIndRead]);

	// Set entry to null
	printQueue[printQueueIndRead][0] = '\0';

	// Return success
	return true;
}

// SEND TEST PACKET
void TestSendPack(char id, byte dat1, byte dat2, byte dat3, uint16_t pack, bool do_conf)
{
	// TestSendPack('r', 0, 0, 0, 1, true);

	//// Only send once
	//if (cnt_loop_short > 0 || cnt_loop_tot > 0) {
	//	return;
	//}

	// Queue packet
	QueuePacket(id, dat1, dat2, dat3, pack, do_conf);

	// Fuck with packet: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer

	// Send packet
	SendPacket();

	// Print everything
	while (PrintDebug());
}

#pragma endregion

#pragma region --------MINOR FUNCTIONS---------

// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink()
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
void QuitBeep()
{
	bool tone = true;
	int cnt_beep = 0;
	while (cnt_beep <= 3)
	{
		if (tone)
		{
			digitalWrite(pin.relRewTone, HIGH);
			digitalWrite(pin.relWhiteNoise, LOW);
			delay(100);
		}
		else
		{
			digitalWrite(pin.relWhiteNoise, HIGH);
			digitalWrite(pin.relRewTone, LOW);
			delay(50);
			cnt_beep++;
		}
		tone = !tone;
	}
	digitalWrite(pin.relWhiteNoise, HIGH);
	digitalWrite(pin.relRewTone, LOW);
}

// PULSE IR
bool PulseIR(int del_sync, int dt_sync)
{
	// Local vars
	bool is_changed = false;

	// Set high
	if (
		!is_irOn &&
		millis() > t_irSyncLast + del_sync
		)
	{
		// Set ir pins on
		SetPort(word_irOn, 0x0);
		t_irSyncLast = millis();
		is_irOn = true;
		is_changed = true;
	}
	// Set low
	else if (
		is_irOn &&
		millis() > t_irSyncLast + dt_sync
		)
	{
		SetPort(0x0, word_irOn);
		is_irOn = false;
		is_changed = true;
	}
	return is_changed;
}

// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size)
{

	// Return -1 if not found
	int ind = -1;
	for (int i = 0; i < arr_size; i++)
	{
		if (id == id_arr[i]) {
			ind = i;
		}
	}

	// Print error if not found
	if (ind == -1) {

		char str[200];
		sprintf(str, "!!ERROR!! [CharInd] ID \'%c\' Not Found", id);
		DebugError(str);
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

	// Bail if nothing on
	if (!isOnAny) {
		return;
	}

	// north
	if (isOnNorth && millis() - t_outLastNorth > dt_ttlPulse) {

		digitalWrite(pin.ttlNorthOn, LOW); // set back to LOW
		isOnNorth = false;
		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] NORTH OFF");
		}
	}

	// west
	if (isOnWest && millis() - t_outLastWest > dt_ttlPulse) {

		digitalWrite(pin.ttlWestOn, LOW); // set back to LOW
		isOnWest = false;

		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] WEST OFF");
		}
	}

	// south
	if (isOnSouth && millis() - t_outLastSouth > dt_ttlPulse) {

		digitalWrite(pin.ttlSouthOn, LOW); // set back to LOW
		isOnSouth = false;

		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] SOUTH OFF");
		}
	}

	// east
	if (isOnEast && millis() - t_outLastEast > dt_ttlPulse) {

		digitalWrite(pin.ttlEastOn, LOW); // set back to LOW
		isOnEast = false;

		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] EAST OFF");
		}
	}
	isOnAny = isOnNorth || isOnWest || isOnSouth || isOnEast;
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
			isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[NorthFun] NORTH ON", t_outLastNorth);
			}
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
			isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[WestFun] WEST ON", t_outLastWest);
			}
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
			isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[SouthFun] SOUTH ON", t_outLastSouth);
			}
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
			isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[EastFun] EAST ON", t_outLastEast);
			}
		}
		t_inLastEast = millis();
	}
}

#pragma endregion

#pragma endregion


void setup()
{
	// SETUP PINS

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

	// SET UP SERIAL STUFF

	// XBee
	Serial1.begin(57600);

	// CS through programming port
	Serial.begin(57600);

	// Serial monitor
	SerialUSB.begin(0);

	// Wait for SerialUSB if debugging
	uint32_t t_check = millis() + 100;
	if (db.Console) {
		while (!SerialUSB && millis() < t_check);
	}

	// Dump CS buffers
	while (Serial.available() > 0) {
		Serial.read();
	}

	// SETUP TTL PORTS

	// Setup reward ttl stuff on port C
	REG_PIOC_OWER = 0xFFFFFFFF;     // enable PORT C
	REG_PIOC_OER = 0xFFFFFFFF;     // set PORT C as output port

	// Get ir word
	int sam_ir_pins[2] = { pin.sam_ttlIR, pin.sam_relIR };
	word_irOn = GetPortWord(0x0, sam_ir_pins, 2);

	// SETUP INTERUPTS

	// North
	attachInterrupt(digitalPinToInterrupt(pin.ptNorthOn), NorthInterrupt, RISING);
	// West
	attachInterrupt(digitalPinToInterrupt(pin.ptWestOn), WestInterrupt, RISING);
	// South
	attachInterrupt(digitalPinToInterrupt(pin.ptSouthOn), SouthInterrupt, RISING);
	// East
	attachInterrupt(digitalPinToInterrupt(pin.ptEastOn), EastInterrupt, RISING);

	// PRINT PIN MAPPING
	/*
	Note: make sure Cheetah aquiring
	*/
	if (doTestPinMapping) {
		DebugPinMap();
	}

	// PRINT SETUP FINISHED
	char str[200] = { 0 };
	sprintf(str, "[setup] FINISHED: Setup: free_ram=%dKB", freeMemory());
	DebugFlow(str);
	while (PrintDebug());

	// SHOW RESTART BLINK
	delayMicroseconds(100);
	StatusBlink();

}


void loop()
{
	// TRACK LOOPS
	cnt_loop_tot = 0;
	cnt_loop_short = cnt_loop_short < 999 ? cnt_loop_short + 1 : 1;

	// SEND DATA
	if (SendPacket());

	// PRINT QUEUED DB
	else if (PrintDebug());

	// SEND QUEUED LOG
	else(SendLog());

	// RESET TTL PINS
	ResetTTL();

	// WAIT FOR HANDSHAKE
	if (!CheckForStart()) {
		return;
	}

	// GET SERIAL INPUT
	GetSerial();

	// HANDLE SERIAL INPUT
	if (r2a.isNew)
	{

		// Run reward tone
		if (r2a.idNow == 'r') {

			// Get reward duration and convert to ms
			rewDur = (uint32_t)r2a.dat[0] * 10;

			// Run tone
			StartRew();
		}

		// Get noise settings
		else if (r2a.idNow == 's') {

			// No noise
			if (r2a.dat[0] == 0)
			{
				fc.doWhiteNoise = false;
				fc.doRewTone = false;
			}
			// White noise only
			else if (r2a.dat[0] == 1)
			{
				// set white noise pins
				fc.doWhiteNoise = true;
				fc.doRewTone = false;
			}
			// White and reward sound
			else if (r2a.dat[0] == 2)
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
		else if (r2a.idNow == 'p') {
			// Signal PID stopped
			if (r2a.dat[0] == 0)
			{
				digitalWrite(pin.ttlPidStop, HIGH);
				digitalWrite(pin.ttlPidRun, LOW);
				DebugFlow("[loop] PId Stopped");
			}
			// Signal PID running
			else if (r2a.dat[0] == 1)
			{
				digitalWrite(pin.ttlPidRun, HIGH);
				digitalWrite(pin.ttlPidStop, LOW);
				DebugFlow("[loop] Pid Started");
			}
		}

		// Signal bull running
		else if (r2a.idNow == 'b') {
			// Signal Bull stopped
			if (r2a.dat[0] == 0)
			{
				digitalWrite(pin.ttlBullStop, HIGH);
				digitalWrite(pin.ttlBullRun, LOW);
				DebugFlow("[loop] Bull Stopped");
			}
			// Signal Bull running
			else if (r2a.dat[0] == 1)
			{
				digitalWrite(pin.ttlBullRun, HIGH);
				digitalWrite(pin.ttlBullStop, LOW);
				DebugFlow("[loop] Bull Started");
			}
		}

		// Quite and reset
		else if (r2a.idNow == 'q') {
			fc.doQuit = true;
			t_quit = millis() + 1000;
			DebugFlow("[loop] QUITING...");
		}

	}

	// CHECK FOR REWARD END
	if (fc.isRewarding) {
		EndRew();
	}

	// PULSE IR
	if (PulseIR(del_irSyncPulse, dt_irSyncPulse)) {

		// Itterate count
		if (is_irOn) {
			cnt_ir++;
		}

		// Log ir event
		char str[200];
		sprintf(str, "[loop] IR Sync %s: cnt=%d dt=%dms",
			is_irOn ? "Start" : "End", cnt_ir, millis() - t_irSyncLast);
		DebugFlow(str);
	}

	// CHECK FOR QUIT
	if (fc.doQuit) {

		// Make sure queues empty
		if (SendPacket() ||
			SendLog() ||
			PrintDebug()) {
			return;
		}

		// Make sure minimum time ellapsed
		if (millis() > t_sent + 100 &&
			millis() > t_rcvd + 100 &&
			millis() > t_quit) {

			// Run bleep bleep
			QuitBeep();

			// Restart Arduino
			REQUEST_EXTERNAL_RESET;
		}
	}

}
