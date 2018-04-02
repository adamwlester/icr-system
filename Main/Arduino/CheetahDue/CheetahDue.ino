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

// Logging
#define DO_LOG 1

// Console
#define DO_PRINT_DEBUG 0

// Main debug flag
#if DO_PRINT_DEBUG 
#define DO_DEBUG 1
#else
#define DO_DEBUG 0
#endif

struct DB
{
	// Logging
	bool log_flow = true;
	bool log_errors = true;
	bool log_r2a = true;
	bool log_a2r = true;
	bool log_resent = true;

	// Printing
	bool print_flow = true;
	bool print_errors = true;
	bool print_r2a = true;
	bool print_a2r = true;
	bool print_resent = true;
	bool print_log = false;

	// Testing
	const bool doPrintPimMapTest = false; // I set
	const bool doHandshakeBypass = false; // I set
	bool do_irSyncCalibration = false; // set by system

}
// Initialize
db;

#pragma endregion 


#pragma region ============== PIN MAPPING ==============

struct PIN
{
	// Relays
	const int relIR = 51;				// port=PC due=51 sam=12
	const int relRewTone = 47;			// port=PC due=47 sam=16
	const int relWhiteNoise = 45;		// port=PC due=45 sam=18

	// TTL
	const int ttlIR = 50;				// port=PC due=50 sam=13 nlx=1,7
	const int ttlRewTone = 46;			// port=PC due=46 sam=17 nlx=1,4
	const int ttlWhiteNoise = 44;		// port=PC due=44 sam=19 nlx=1,5
	const int ttlRewOn = 34;			// port=PC due=34 sam=2  nlx=1,0
	const int ttlRewOff = 36;			// port=PC due=36 sam=4  nlx=1,1

	// SAM3X pin
	const int sam_relIR = 12;			// port=PC due=51 sam=12
	const int sam_relRewTone = 16;		// port=PC due=47 sam=16
	const int sam_relWhiteNoise = 18;	// port=PC due=45 sam=18
	const int sam_ttlIR = 13;			// port=PC due=50 sam=13 nlx=1,7
	const int sam_ttlRewTone = 17;		// port=PC due=46 sam=17 nlx=1,4
	const int sam_ttlWhiteNoise = 19;	// port=PC due=44 sam=19 nlx=1,5
	const int sam_ttlRewOn = 2;			// port=PC due=34 sam=2  nlx=1,0
	const int sam_ttlRewOff = 4;		// port=PC due=36 sam=4  nlx=1,1

	// PID
	const int ttlPidRun = 26;
	const int ttlPidStop = 28;

	// Bulldozer
	const int ttlBullRun = 30;
	const int ttlBullStop = 32;

	// IR blink switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int BlinkSwitch_Gnd = 11;
	const int BlinkSwitch = 12;

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

#pragma endregion


#pragma region ============ VARIABLE SETUP =============

// Flow/state control
struct FC
{
	bool doQuit = false;
	bool isSesStarted = false;
	bool doWhiteNoise = false;
	bool doRewTone = false;
	bool isRewarding = false;
	bool isRobLogging = false;
	bool doBlockIRPulse = false;
}
// Initialize
fc;

// Debugging general
uint32_t cnt_loop_tot = 0;
uint16_t cnt_loop_short = 0;
uint16_t cnt_warn = 0;
uint16_t cnt_err = 0;
uint16_t warn_line[100] = { 0 };
uint16_t err_line[100] = { 0 };
const uint16_t maxStoreStrLng = 300;
const uint16_t maxMsgStrLng = maxStoreStrLng - 50;

// Log debugging
const int logQueueSize = 40;
char logQueue[logQueueSize][maxStoreStrLng] = { { 0 } };
int logQueueIndStore = 0;
int logQueueIndRead = 0;
int cnt_logsStored = 0;

// Print debugging
const int printQueueSize = 15;
char printQueue[printQueueSize][maxStoreStrLng] = { { 0 } };
int printQueueIndStore = 0;
int printQueueIndRead = 0;

// Serial tracking
const int sendQueueSize = 10;
const int sendQueueBytes = 18;
byte sendQueue[sendQueueSize][sendQueueBytes] = { { 0 } };
int sendQueueIndStore = 0;
int sendQueueIndRead = 0;
const int dt_sendSent = 5; // (ms) 
const int dt_sendRcvd = 0; // (ms) 
int cnt_packBytesRead = 0;
int cnt_packBytesSent = 0;
int cnt_packBytesDiscarded = 0;
int cnt_logBytesSent = 0;

// Serial from rob
struct R2A
{
	const char id[6] = {
		't', // system test
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
	float dat[3] = { 0 };
	const static int lng = sizeof(id) / sizeof(id[0]);
	uint16_t pack[lng] = { 0 };
	uint16_t packLast[lng] = { 0 };
	int cnt_repeat = 0;
	int cnt_dropped = 0;
	int32_t t_rcvd = 0; // (ms)
	int dt_rcvd = 0; // (ms)
}
// Initialize
r2a;

// Serial to rob
struct A2R
{
	const char id[6] = {
		't', // system test
		'q', // quit/reset
		'r', // reward
		's', // sound cond [0, 1, 2]
		'p', // pid mode [0, 1]
		'b', // bull mode [0, 1]
	};
	const char head = '{';
	const char foot = '}';
	float dat[3] = { 0 };
	const static int lng = sizeof(id) / sizeof(id[0]);
	uint16_t pack[lng] = { 0 };
	uint16_t packLast[lng] = { 0 };
	int cnt_repeat = 0;
	int32_t t_sent = 0; // (ms)
	int dt_sent = 0; // (ms)
}
// Initialize
a2r;

// Serial to CS
struct A2C
{
	const char head = '<';
	const char foot = '>';
	int32_t t_sent = 0; // (ms)
	int dt_sent = 0; // (ms)
}
// Initialize
a2c;

// Reward
int cnt_rew;
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;
uint32_t word_rewStr;
uint32_t word_rewEnd;

// IR time sync LED
const int dt_irSyncPulse = 500; // (ms) 
const int dt_irSyncPulseOn = 10; // (ms)
uint32_t del_irSyncPulse = 60000; // (ms)
uint32_t t_sync = 0;
bool is_irOn = false;
int cnt_ir = 0;
uint32_t t_irSyncLast;
uint32_t word_irOn;

// Volitiles
volatile bool v_doPTInterupt = false;
// PT north vars
volatile uint32_t v_t_inLastNorth = millis();
volatile uint32_t v_t_outLastNorth = millis();
volatile bool v_isOnNorth = false;
// PT west vars
volatile uint32_t v_t_inLastWest = millis();
volatile uint32_t v_t_outLastWest = millis();
volatile bool v_isOnWest = false;
// PT south 
volatile uint32_t v_t_inLastSouth = millis();
volatile uint32_t v_t_outLastSouth = millis();
volatile bool v_isOnSouth = false;
// PT east vars
volatile uint32_t v_t_inLastEast = millis();
volatile uint32_t v_t_outLastEast = millis();
volatile bool v_isOnEast = false;
// Any pin
volatile bool v_isOnAny = false;

// TTL timers
uint32_t t_debounce = 10; // (ms)
uint32_t dt_ttlPulse = 50; // (ms)

						   // union
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

// CHECK FOR HANDSHAKE
bool CheckForStart();
// PARSE SERIAL INPUT
void GetSerial();
// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char mtch = '\0');
// STORE PACKET DATA TO BE SENT
void QueuePacket(char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf);
// SEND SERIAL PACKET DATA
bool SendPacket();
// STORE LOG STRING
void QueueLog(char msg[], uint32_t t);
// SEND LOG DATA OVER SERIAL
bool SendLog();
// GET CURRENT NUMBER OF ENTRIES IN DB QUEUE: queue_str=["Log", "Print"]
int GetQueueAvailable(char queue_str[]);
// START REWARD
void StartRew();
// END REWARD
void EndRew();
// TEST PIN MAPPING
void DebugPinMap();
// LOG/PRING MAIN EVENT
void DebugFlow(char msg[], uint32_t t = millis());
// LOG/PRINT ERRORS
void DebugError(char msg[], bool is_error = false, uint32_t t = millis());
// PRINT RECIEVED PACKET
void DebugRcvd(char id, char msg[], bool is_repeat = false);
// LOG/PRING SENT PACKET DEBUG STRING
void DebugSent(char msg[], bool is_repeat);
// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t);
// PRINT DB INFO
bool PrintDebug();
// SEND TEST PACKET
void TestSendPack(char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf);
// HARDWARE TEST
void HardwareTest(int test_num);
// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink();
// PLAY SOUND WHEN QUITING
void QuitBeep();
// PULSE IR: force_state=[0=off, 1=on, 2=auto]
bool PulseIR(int dt_off, int dt_on, byte force_state = 2, bool do_ttl = true);
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

// CHECK FOR HANDSHAKE
bool CheckForStart()
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	bool is_rcvd = false;
	byte in_byte[1] = { 0 };
	byte handshake_byte[1] = { 'i' };

	// Bail if session started
	if (fc.isSesStarted) {
		return true;
	}

	// Bypass handshake
	if (db.doHandshakeBypass) {
		in_byte[0] = handshake_byte[0];
	}

	// Check if IR should be pulsed 
	if (digitalRead(pin.BlinkSwitch) == LOW || is_irOn)
	{
		// Turn on without triggering ttl
		PulseIR(dt_irSyncPulse, dt_irSyncPulseOn, 2, false);
	}

	// Get new data
	if (Serial.available() > 0) {
		in_byte[0] = Serial.read();
	}

	// Bail if not a match
	if (in_byte[0] != handshake_byte[0]) {
		return false;
	}

	// Turn off ir
	PulseIR(0, 0, 0);
	delay(1000);

	// Turn ir on for 10 ms
	PulseIR(0, 0, 1);
	delayMicroseconds(10 * 1000);

	// Turn ir off for 65 ms
	PulseIR(0, 0, 0);
	delayMicroseconds(65 * 1000);

	// Turn ir back on
	PulseIR(0, 0, 1);

	// Store time
	t_sync = millis();

	// Turn ir on for 10 ms
	PulseIR(0, 0, 1);
	delayMicroseconds(10 * 1000);

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
	v_doPTInterupt = true;
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
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char dat_str_1[200] = { 0 }; dat_str_1[0] = '\0';
	static char dat_str_2[200] = { 0 }; dat_str_2[0] = '\0';
	uint32_t t_str = millis();
	int dt_parse = 0;
	int buff_tx = 0;
	int buff_rx = 0;
	byte buff = 0;
	char head = ' ';
	char id = ' ';
	float dat[3] = { 0 };
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
	for (int i = 0; i < 3; i++)
	{
		U.f = 0.0f;
		U.b[0] = WaitBuffRead();
		U.b[1] = WaitBuffRead();
		U.b[2] = WaitBuffRead();
		U.b[3] = WaitBuffRead();
		dat[i] = U.f;
	}

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

	// Strore parse time
	dt_parse = millis() - t_str;

	// Get total data in buffers
	buff_rx = Serial1.available();
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();

	// Store data strings
	sprintf(dat_str_1, "head=%c id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d foot=%c do_conf=%d b_read=%d b_dump=%d",
		head, id, dat[0], dat[1], dat[2], pack, foot, do_conf, cnt_packBytesRead, cnt_packBytesDiscarded);


	// Check for missing footer
	if (foot != r2a.foot) {

		// Store data strings
		sprintf(dat_str_2, "rx=%d tx=%d dt_prs=%d dt_snd=%d dt_rcv=%d",
			buff_rx, buff_tx, dt_parse, a2r.t_sent > 0 ? millis() - a2r.t_sent : 0, r2a.dt_rcvd);

		// Itterate dropped count
		r2a.cnt_dropped++;

		// Log/print dropped packet info
		sprintf(str, "**WARNING** [GetSerial] Dropped r2a Packs: cnt=%d %s %s",
			r2a.cnt_dropped, dat_str_1, dat_str_2);
		DebugError(str);

	}
	else
	{
		// Update recive time
		r2a.dt_rcvd = r2a.t_rcvd > 0 ? millis() - r2a.t_rcvd : 0;
		r2a.t_rcvd = millis();

		// Store data strings
		sprintf(dat_str_2, "rx=%d tx=%d dt_prs=%d dt_snd=%d dt_rcv=%d",
			buff_rx, buff_tx, dt_parse, a2r.t_sent > 0 ? millis() - a2r.t_sent : 0, r2a.dt_rcvd);

		// Update last packet
		int id_ind = CharInd(id, r2a.id, r2a.lng);
		r2a.packLast[id_ind] = r2a.pack[id_ind];
		r2a.pack[id_ind] = pack;

		// Combine data strings
		sprintf(str, "%s %s", dat_str_1, dat_str_2);

		// Check if packet is new
		if (r2a.packLast[id_ind] != pack)
		{
			// Print received packet
			DebugRcvd(id, str);

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
			DebugRcvd(id, str, true);
		}

		// Send confirmation
		if (do_conf) {
			QueuePacket(id, dat[0], dat[1], dat[2], pack, false);
		}

	}

	// Check if data was discarded
	if (cnt_packBytesDiscarded > 0) {

		// Log/print discarded data
		sprintf(str, "**WARNING** [GetSerial] Dumped Bytes: %s %s", dat_str_1, dat_str_2);
		DebugError(str);
	}

	// Check if parsing took unusually long
	if (dt_parse > 30) {
		sprintf(str, "**WARNING** [GetSerial] Parser Hanging: %s %s", dat_str_1, dat_str_2);
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
	bool is_bytes_discarded = false;
	bool is_overflowed = false;
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
			return buff;
		}
	}

	// Find specific byte
	while (
		buff != mtch  &&
		millis() < t_timeout &&
		!is_overflowed) {

		// Check new data
		if (Serial1.available() > 0) {

			buff = Serial1.read();
			cnt_packBytesRead++;

			// check match was found
			if (buff == mtch) {
				return buff;
			}

			// Otherwise add to discard count
			else {
				cnt_packBytesDiscarded++;
				is_bytes_discarded = true;
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

	// Store current info
	char buff_print = buff == 10 ? 'n' : buff == 13 ? 'r' : buff;
	sprintf(dat_str, " buff=%c b_read=%d b_dump=%d rx_str=%d rx_now=%d tx_now=%d dt_chk=%d",
		buff_print, cnt_packBytesRead, cnt_packBytesDiscarded, buff_rx_start, buff_rx, buff_tx, (millis() - t_timeout) + timeout);


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
	else if (mtch != '\0') {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Char %c Not Found:", mtch);
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
	return 0;

}

// STORE PACKET DATA TO BE SENT
void QueuePacket(char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf)
{
	/*
	STORE DATA FOR ROBOT
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	int id_ind = 0;
	float dat[3] = { dat1 , dat2 , dat3 };

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
		queue_state[sendQueueSize] = '\0';

		// Get buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		int buff_rx = Serial1.available();

		// Store overflow error instead
		sprintf(str, "!!ERRROR!! [QueuePacket] SEND QUEUE OVERFLOWED: sendQueueIndStore=%d sendQueueIndRead=%d queue_state=|%s| dt_snd=%d dt_rcv=%d tx=%d rx=%d",
			sendQueueIndStore, sendQueueIndRead, queue_state, millis() - a2r.t_sent, millis() - r2a.t_rcvd, buff_tx, buff_rx);

		// Log/print error
		DebugError(str, true);

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
	for (int i = 0; i < 3; i++)
	{
		U.f = dat[i];
		sendQueue[sendQueueIndStore][b_ind++] = U.b[0];
		sendQueue[sendQueueIndStore][b_ind++] = U.b[1];
		sendQueue[sendQueueIndStore][b_ind++] = U.b[2];
		sendQueue[sendQueueIndStore][b_ind++] = U.b[3];
	}
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
	static char dat_str[200] = { 0 }; dat_str[0] = '\0';
	byte msg[msg_lng];
	uint32_t t_queue = millis();
	bool is_resend = false;
	char id = '\0';
	float dat[3] = { 0 };
	bool do_conf = 0;
	uint16_t pack = 0;
	int buff_tx;
	int buff_rx;

	// Reset bytes sent
	cnt_packBytesSent = 0;

	// Bail if nothing in queue
	if (sendQueueIndRead == sendQueueIndStore &&
		sendQueue[sendQueueIndStore][0] == '\0') {
		return false;
	}

	// Get buffer 
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	buff_rx = Serial1.available();

	// Bail if buffer or time inadequate
	if (buff_tx > 0 ||
		buff_rx > 0 ||
		millis() < a2r.t_sent + dt_sendSent) {

		// Indicate still packs to send
		return true;
	}

	// Add small delay if just recieved
	else if (millis() < r2a.t_rcvd + dt_sendRcvd) {
		delayMicroseconds(500);
	}

	// Itterate send ind
	sendQueueIndRead++;

	// Check if ind should roll over 
	if (sendQueueIndRead == sendQueueSize) {
		sendQueueIndRead = 0;
	}

	// Send
	Serial1.write(sendQueue[sendQueueIndRead], msg_lng);
	a2r.dt_sent = a2r.t_sent > 0 ? millis() - a2r.t_sent : 0;
	a2r.t_sent = millis();
	cnt_packBytesSent = msg_lng;

	// Get buffers
	buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	buff_rx = Serial1.available();

	// pull out packet data
	int b_ind = 1;
	// id
	id = sendQueue[sendQueueIndRead][b_ind++];
	// dat
	for (int i = 0; i < 3; i++)
	{
		U.f = 0;
		U.b[0] = sendQueue[sendQueueIndRead][b_ind++];
		U.b[1] = sendQueue[sendQueueIndRead][b_ind++];
		U.b[2] = sendQueue[sendQueueIndRead][b_ind++];
		U.b[3] = sendQueue[sendQueueIndRead][b_ind++];
		dat[i] = U.f;
	}
	// pack
	U.f = 0.0f;
	U.b[0] = sendQueue[sendQueueIndRead][b_ind++];
	U.b[1] = sendQueue[sendQueueIndRead][b_ind++];
	pack = U.i16[0];
	// do_conf 
	do_conf = sendQueue[sendQueueIndRead][b_ind++] == 1 ? true : false;

	// Set entry to null
	sendQueue[sendQueueIndRead][0] = '\0';

	// Check if resending
	is_resend = pack == r2a.packLast[CharInd(id, r2a.id, r2a.lng)];

	// Make log/print string
	sprintf(dat_str, "id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d do_conf=%d b_sent=%d tx=%d rx=%d dt_snd=%d dt_rcv=%d dt_q=%d",
		id, dat[0], dat[1], dat[2], pack, do_conf, cnt_packBytesSent, buff_tx, buff_rx, a2r.dt_sent, r2a.t_rcvd > 0 ? millis() - r2a.t_rcvd : 0, millis() - t_queue);

	// Log/print
	DebugSent(dat_str, is_resend);

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
	static char str[200] = { 0 }; str[0] = '\0';
	static char msg_temp[maxStoreStrLng] = { 0 }; msg_temp[0] = '\0';
	static char msg_copy[maxStoreStrLng] = { 0 }; msg_copy[0] = '\0';
	static char msg_out[maxStoreStrLng] = { 0 }; msg_out[0] = '\0';
	static char queue_state[logQueueSize + 1] = { 0 }; queue_state[0] = '\0';
	bool is_queue_overflowing = false;
	bool is_mem_overflowing = false;
	uint32_t t_m = 0;
	byte chksum = 0;

	// Update logQueue ind
	logQueueIndStore++;

	// Check if ind should roll over 
	if (logQueueIndStore == logQueueSize) {

		// Reset queueIndWrite
		logQueueIndStore = 0;
	}

	// Get message length
	is_mem_overflowing = strlen(msg) >= maxMsgStrLng;

	// Check for overflow
	is_queue_overflowing = logQueue[logQueueIndStore][0] != '\0';

	// Check if queue overflowed or message too long
	if (is_queue_overflowing || is_mem_overflowing) {

		// Handle overflow queue
		if (is_queue_overflowing) {

			// Get list of empty entries
			for (int i = 0; i < logQueueSize; i++) {
				queue_state[i] = logQueue[i][0] == '\0' ? '0' : '1';
			}
			queue_state[logQueueSize] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [QueueLog] LOG QUEUE OVERFLOWED: queue_s=%d queue_r=%d queue_state=|%s|",
				logQueueIndStore, logQueueIndRead, queue_state);

			// Set queue back so overflow will write over last log
			logQueueIndStore = logQueueIndStore - 1 >= 0 ? logQueueIndStore - 1 : logQueueSize - 1;
		}

		// Handle overflow char array
		else if (is_mem_overflowing) {

			// Store part of message
			for (int i = 0; i < 100; i++) {
				str[i] = msg[i];
			}
			str[100] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [QueueLog] MESSAGE TOO LONG: msg_lng=%d max_lng=%d \"%s%s\"",
				strlen(msg), maxMsgStrLng, str, "...");
		}

		// Print error if print queue not also overflowed
		if (db.print_errors &&
			DO_PRINT_DEBUG &&
			GetQueueAvailable("Print") > 1) {

			// Print
			QueueDebug(msg_copy, t);
		}
	}

	// Copy message and Update log count
	else {
		for (int i = 0; i < strlen(msg); i++) {
			msg_copy[i] = msg[i];
		}
		msg_copy[strlen(msg)] = '\0';

		// Itterate count
		cnt_logsStored++;
	}

	// Get sync correction
	t_m = t - t_sync;

	// Put it all together
	sprintf(msg_temp, "[%d],%lu,%d,%s", cnt_logsStored, t_m, cnt_loop_short, msg_copy);

	// Get message size
	chksum = strlen(msg_temp);

	// Add header, chksum and footer
	int b_ind = 0;
	// head
	msg_out[b_ind++] = a2c.head;
	// checksum
	msg_out[b_ind++] = chksum;
	// msg
	for (int i = 0; i < chksum; i++) {
		msg_out[b_ind++] = msg_temp[i];
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
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	int msg_lng = 0;
	cnt_logBytesSent = 0;
	int xbee_buff_tx;
	int xbee_buff_rx;
	int cs_buff_tx;
	int cs_buff_rx;

	// Bail if serial not established or no logs to store
	if (!fc.isSesStarted ||
		logQueueIndRead == logQueueIndStore &&
		logQueue[logQueueIndStore][0] == '\0') {
		return false;
	}

	// Get buffers 
	xbee_buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
	xbee_buff_rx = Serial1.available();
	cs_buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial.availableForWrite();
	cs_buff_rx = Serial.available();

	// Bail if buffer or time inadequate
	if (!(xbee_buff_tx == 0 &&
		xbee_buff_rx == 0 &&
		cs_buff_tx == 0 &&
		cs_buff_rx == 0 &&
		millis() > a2c.dt_sent + dt_sendSent &&
		millis() > a2r.t_sent + dt_sendSent &&
		millis() > r2a.t_rcvd + dt_sendRcvd)) {

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
	a2c.dt_sent = a2c.t_sent > 0 ? millis() - a2c.t_sent : 0;
	a2c.t_sent = millis();
	cnt_logBytesSent += msg_lng;

	// Print
	if (db.print_log && DO_PRINT_DEBUG)
	{
		// Get data in buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial.availableForWrite();
		int buff_rx = Serial.available();

		// Get buffers 
		xbee_buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial1.availableForWrite();
		xbee_buff_rx = Serial1.available();
		cs_buff_tx = SERIAL_BUFFER_SIZE - 1 - Serial.availableForWrite();
		cs_buff_rx = Serial.available();

		// Print stored log
		sprintf(str, "   [LOG] a2c: log_cnt=%d b_sent=%d xbee_rx=%d xbee_tx=%d cs_rx=%d cs_tx=%d dt_snd=%d msg=\"%s\"",
			cnt_logsStored, cnt_logBytesSent, xbee_buff_tx, xbee_buff_rx, cs_buff_tx, cs_buff_rx, a2c.dt_sent, logQueue[logQueueIndRead]);
		QueueDebug(str, a2c.t_sent);
	}

	// Set entry to null
	logQueue[logQueueIndRead][0] = '\0';

	// Return success
	return true;
}

// GET CURRENT NUMBER OF ENTRIES IN DB QUEUE: queue_str=["Log", "Print"]
int GetQueueAvailable(char queue_str[]) {

	// Local vars
	int n_entries = 0;

	// Check log queue
	if (strcmp(queue_str, "Log") == 0) {
		for (int i = 0; i < logQueueSize; i++) {
			n_entries += logQueue[i][0] != '\0' ? 1 : 0;
		}

		// Get total available
		return logQueueSize - n_entries;
	}

	// Check print queue
	else if (strcmp(queue_str, "Print") == 0) {
		for (int i = 0; i < printQueueSize; i++) {
			n_entries += printQueue[i][0] != '\0' ? 1 : 0;
		}

		// Get total available
		return printQueueSize - n_entries;
	}

	// Bad entry
	else {
		return 0;
	}
}

#pragma endregion

#pragma region --------REWARD---------

// START REWARD
void StartRew()
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

	// Set rew on pins
	SetPort(word_rewStr, word_rewEnd);

	// Add to count
	cnt_rew++;

	// Set rew off time
	t_rewEnd = millis() + rewDur;

	// Signal PID stopped
	digitalWrite(pin.ttlPidRun, LOW);
	digitalWrite(pin.ttlPidStop, HIGH);
	DebugFlow("[StartRew] PID STOPPED");

	// Set flag
	fc.isRewarding = true;

	// Print
	sprintf(str, "[StartRew] RUNNING: Reward: cnt_rew=%d dt_rew=%dms", cnt_rew, rewDur);
	DebugFlow(str);
}

// END REWARD
void EndRew()
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

	if (millis() > t_rewEnd)
	{

		// Set reward off pins
		SetPort(word_rewEnd, word_rewStr);

		fc.isRewarding = false;
		sprintf(str, "[StartRew] FINISHED: Reward: cnt_rew=%d dt_rew=%dms", cnt_rew, rewDur);
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
	bool do_print = DO_PRINT_DEBUG && db.print_flow;
	bool do_log = DO_LOG && db.log_flow;

	if (do_print) {
		QueueDebug(msg, millis());
	}

	if (do_log) {
		QueueLog(msg, millis());
	}

}

// LOG/PRINT ERRORS
void DebugError(char msg[], bool is_error, uint32_t t)
{
	// Local vars
	bool do_print = db.print_errors && DO_PRINT_DEBUG;
	bool do_log = db.log_errors && DO_LOG;

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

	// Store error info
	if (is_error) {
		err_line[cnt_err < 100 ? cnt_err++ : 99] = cnt_logsStored;
	}
	else {
		warn_line[cnt_warn < 100 ? cnt_warn++ : 99] = cnt_logsStored;
	}
}

// PRINT RECIEVED PACKET
void DebugRcvd(char id, char msg[], bool is_repeat)
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	char msg_out[maxStoreStrLng + 50] = { 0 };
	bool do_print = DO_PRINT_DEBUG && db.print_r2a;
	bool do_log = DO_LOG && db.log_r2a;

	// Print/Log
	if (!(do_print || do_log)) {
		return;
	}

	// Check if this is a repeat
	if (!is_repeat) {
		sprintf(msg_out, "   [RCVD] r2a: %s", msg);
	}
	else {
		sprintf(msg_out, "   [*RE-RCVD*] r2a: cnt=%d %s", r2a.cnt_repeat, msg);
	}

	if (do_print) {
		QueueDebug(msg_out, r2a.t_rcvd);
	}

	if (do_log) {
		QueueLog(msg_out, r2a.t_rcvd);
	}

}

// LOG/PRING SENT PACKET DEBUG STRING
void DebugSent(char msg[], bool is_repeat)
{
	// Local vars
	char msg_out[maxStoreStrLng + 50] = { 0 };
	static char str[maxStoreStrLng + 50] = { 0 }; str[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print status
	do_print = DO_PRINT_DEBUG && db.print_a2r;
	do_log = DO_LOG && db.log_a2r;

	// Bail if neither set
	if (!(do_print || do_log)) {
		return;
	}

	// Check if this is a repeat
	if (!is_repeat) {
		sprintf(msg_out, "   [SENT] a2r: %s", msg);
	}
	// Add to counters
	else {
		a2r.cnt_repeat++;
		sprintf(msg_out, "   [*RE-SENT*] a2r: cnt=%d %s", a2r.cnt_repeat, msg);
	}

	if (do_print) {
		QueueDebug(msg_out, a2r.t_sent);
	}

	if (do_log) {
		QueueLog(msg_out, a2r.t_sent);
	}

}

// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t)
{
	// Local vars
	static char str[200] = { 0 }; str[0] = '\0';
	char msg_copy[maxStoreStrLng] = { 0 };
	char str_time[100] = { 0 };
	char queue_state[printQueueSize + 1];
	bool is_queue_overflowing = false;
	bool is_mem_overflowing = false;
	uint32_t t_m = 0;
	float t_s = 0;

	// Update printQueue ind
	printQueueIndStore++;

	// Check if ind should roll over 
	if (printQueueIndStore == printQueueSize) {

		// Reset queueIndWrite
		printQueueIndStore = 0;
	}

	// Get message length
	is_mem_overflowing = strlen(msg) >= maxMsgStrLng;

	// Check for overflow
	is_queue_overflowing = printQueue[printQueueIndStore][0] != '\0';

	// Check if queue overflowed or message too long
	if (is_queue_overflowing || is_mem_overflowing) {

		// Handle overflow queue
		if (is_queue_overflowing) {

			// Get list of empty entries
			for (int i = 0; i < printQueueSize; i++) {
				queue_state[i] = printQueue[i][0] == '\0' ? '0' : '1';
			}
			queue_state[printQueueSize] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [QueueDebug] PRINT QUEUE OVERFLOWED: queueIndS=%d queueIndR=%d queue_state=|%s|",
				printQueueIndStore, printQueueIndRead, queue_state);

			// Set queue back so overflow will write over last print
			printQueueIndStore = printQueueIndStore - 1 >= 0 ? printQueueIndStore - 1 : printQueueSize - 1;
		}

		// Handle overflow char array
		else if (is_mem_overflowing) {

			// Store part of message
			for (int i = 0; i < 100; i++) {
				str[i] = msg[i];
			}
			str[100] = '\0';

			// Store overflow error instead
			sprintf(msg_copy, "**WARNING** [QueueDebug] MESSAGE TOO LONG: msg_lng=%d max_lng=%d \"%s%s\"",
				strlen(msg), maxMsgStrLng, str, "...");
		}

		// Log error
		if (db.log_errors &&
			DO_LOG &&
			GetQueueAvailable("Log") > 1) {

			// Log
			QueueLog(msg_copy, t);
		}

	}

	// Copy message
	else {
		for (int i = 0; i < strlen(msg); i++) {
			msg_copy[i] = msg[i];
		}
		msg_copy[strlen(msg)] = '\0';
	}

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Make time string
	sprintf(str_time, "[%0.3f][%d]", t_s, cnt_loop_short);

	// Add space after time
	char spc[50] = { 0 };
	char arg[50] = { 0 };
	sprintf(arg, "%%%ds", 20 - strlen(str_time));
	sprintf(spc, arg, '_');

	// Put it all together
	sprintf(printQueue[printQueueIndStore], "%s%s%s\n", str_time, spc, msg_copy);

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
void TestSendPack(char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf)
{
	// EXAMPLE:
	/*
	static uint32_t t_s = 0;
	static int send_cnt = 0;
	static uint16_t pack = 0;
	if (send_cnt == 0 && millis()>t_s + 30) {
	pack++;
	TestSendPack('r', 0, 0, 0, 1, true);
	t_s = millis();
	send_cnt++;
	}
	*/

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

// HARDWARE TEST
void HardwareTest(int test_num)
{
	// Local vars
	int dt_on = 500;

	switch (test_num)
	{

	// Test arduino pin reward tone
	case 1:
	{
		// Write pins high then low
		DebugFlow("[HardwareTest] TEST DUE PIN: Reward Tone");
		digitalWrite(pin.relWhiteNoise, LOW);
		digitalWrite(pin.ttlWhiteNoise, LOW);
		digitalWrite(pin.relRewTone, HIGH);
		digitalWrite(pin.ttlRewTone, HIGH);
		delay(dt_on);
		digitalWrite(pin.relRewTone, LOW);
		digitalWrite(pin.ttlRewTone, LOW);
	}
	break;

	// Test arduino pin white noise
	case 2:
	{
		DebugFlow("[HardwareTest] TEST DUE PIN: White Noise");
		digitalWrite(pin.relWhiteNoise, HIGH);
		digitalWrite(pin.ttlWhiteNoise, HIGH);
		delay(dt_on);
		digitalWrite(pin.relWhiteNoise, LOW);
		digitalWrite(pin.ttlWhiteNoise, LOW);
	}
	break;

	// Test port set reward only
	case 3:
	{
		// Create port word for rew event pins only
		DebugFlow("[HardwareTest] TEST PORT WORD: Reward Event Only");
		int sam_on_pins[1] = { pin.sam_ttlRewOn };
		word_rewStr = GetPortWord(0x0, sam_on_pins, 1);
		int sam_off_pins[1] = { pin.sam_ttlRewOff };
		word_rewEnd = GetPortWord(0x0, sam_off_pins, 1);

		// Write word on then off
		SetPort(word_rewStr, word_rewEnd);
		delay(dt_on);
		SetPort(word_rewEnd, word_rewStr);
	}
	break;

	// Test port set reward with sound
	case 4:
	{
		// Create port word for white and tone pins
		DebugFlow("[HardwareTest] TEST PORT WORD: Reward With Sound");
		int sam_on_pins[3] = { pin.sam_relRewTone, pin.sam_ttlRewTone, pin.sam_ttlRewOn };
		word_rewStr = GetPortWord(0x0, sam_on_pins, 3);
		int sam_off_pins[3] = { pin.sam_relWhiteNoise, pin.sam_ttlWhiteNoise, pin.sam_ttlRewOff };
		word_rewEnd = GetPortWord(0x0, sam_off_pins, 3);

		// Write word on then off
		SetPort(word_rewStr, word_rewEnd);
		delay(dt_on);
		SetPort(word_rewEnd, word_rewStr);
	}
	break;

	// Test arduino pin IR
	case 5:
	{
		// Write pins high then low
		DebugFlow("[HardwareTest] TEST DUE PIN: IR");
		digitalWrite(pin.relIR, HIGH);
		digitalWrite(pin.ttlIR, HIGH);
		delay(dt_on);
		digitalWrite(pin.relIR, LOW);
		digitalWrite(pin.ttlIR, LOW);
	}
	break;

	// Test arduino pin IR
	case 6:
	{
		// Create port word for white and tone pins
		DebugFlow("[HardwareTest] TEST PORT WORD: IR");
		int sam_ir_pins[2] = { pin.sam_ttlIR, pin.sam_relIR };
		word_irOn = GetPortWord(0x0, sam_ir_pins, 2);

		// Write word on then off
		SetPort(word_irOn, 0x0);
		delay(dt_on);
		SetPort(0x0, word_irOn);

	}
	break;

	// Test arduino pin PID
	case 7:
	{
		// Write pins high then low
		DebugFlow("[HardwareTest] TEST DUE PIN: PID");
		digitalWrite(pin.ttlPidRun, HIGH);
		digitalWrite(pin.ttlPidStop, LOW);
		delay(dt_on);
		digitalWrite(pin.ttlPidRun, LOW);
		digitalWrite(pin.ttlPidStop, HIGH);
	}
	break;


	// Test arduino pin Bull
	case 8:
	{
		// Write pins high then low
		DebugFlow("[HardwareTest] TEST DUE PIN: BULL");
		digitalWrite(pin.ttlBullRun, HIGH);
		digitalWrite(pin.ttlBullStop, LOW);
		delay(dt_on);
		digitalWrite(pin.ttlBullRun, LOW);
		digitalWrite(pin.ttlBullStop, HIGH);
	}
	break;

	// Test arduino pin PT
	case 9:
	{
		// Write pins high then low
		DebugFlow("[HardwareTest] TEST DUE PIN: PT");
		digitalWrite(pin.ttlNorthOn, HIGH);
		delay(10);
		digitalWrite(pin.ttlWestOn, HIGH);
		delay(10);
		digitalWrite(pin.ttlSouthOn, HIGH);
		delay(10);
		digitalWrite(pin.ttlEastOn, HIGH);
		delay(dt_on);
		digitalWrite(pin.ttlNorthOn, LOW);
		digitalWrite(pin.ttlWestOn, LOW);
		digitalWrite(pin.ttlSouthOn, LOW);
		digitalWrite(pin.ttlEastOn, LOW);
	}
	break;

	default:
		break;
	}
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

// PULSE IR: force_state=[0,1,2]
bool PulseIR(int dt_pulse, int dt_on, byte force_state, bool do_ttl)
{
	// Local vars
	bool is_changed = false;

	// Bail if blocking
	if (fc.doBlockIRPulse) {
		return false;
	}

	// Set high
	if (
		force_state == 1 ||
		(
			force_state == 2 &&
			!is_irOn &&
			millis() > t_irSyncLast + dt_pulse)
		)
	{
		// Set ir relay and ttl pins high
		if (do_ttl) {
			SetPort(word_irOn, 0x0);
		}
		else {
			digitalWrite(pin.relIR, HIGH);
		}
		t_irSyncLast = millis();
		is_irOn = true;
		is_changed = true;
	}
	// Set low
	else if (
		force_state == 0 ||
		(
			force_state == 2 &&
			is_irOn &&
			millis() > t_irSyncLast + dt_on)
		)
	{
		// Set ir relay and ttl pins low
		if (do_ttl) {
			SetPort(0x0, word_irOn);
		}
		else {
			digitalWrite(pin.relIR, LOW);
		}
		is_irOn = false;
		is_changed = true;
	}
	return is_changed;
}

// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size)
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

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
	if (!v_isOnAny) {
		return;
	}

	// north
	if (v_isOnNorth && millis() - v_t_outLastNorth > dt_ttlPulse) {

		digitalWrite(pin.ttlNorthOn, LOW); // set back to LOW
		v_isOnNorth = false;
		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] NORTH OFF");
		}
	}

	// west
	if (v_isOnWest && millis() - v_t_outLastWest > dt_ttlPulse) {

		digitalWrite(pin.ttlWestOn, LOW); // set back to LOW
		v_isOnWest = false;

		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] WEST OFF");
		}
	}

	// south
	if (v_isOnSouth && millis() - v_t_outLastSouth > dt_ttlPulse) {

		digitalWrite(pin.ttlSouthOn, LOW); // set back to LOW
		v_isOnSouth = false;

		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] SOUTH OFF");
		}
	}

	// east
	if (v_isOnEast && millis() - v_t_outLastEast > dt_ttlPulse) {

		digitalWrite(pin.ttlEastOn, LOW); // set back to LOW
		v_isOnEast = false;

		// Print
		if (db.print_flow) {
			DebugFlow("[ResetTTL] EAST OFF");
		}
	}
	v_isOnAny = v_isOnNorth || v_isOnWest || v_isOnSouth || v_isOnEast;
}

// North
void NorthInterrupt()
{
	NorthFun();
}
void NorthFun()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastNorth > t_debounce) {

			digitalWrite(pin.ttlNorthOn, HIGH);
			v_t_outLastNorth = millis();
			v_isOnNorth = true;
			v_isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[NorthFun] NORTH ON", v_t_outLastNorth);
			}
		}
		v_t_inLastNorth = millis();
	}
}

// West
void WestInterrupt()
{
	WestFun();
}
void WestFun()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastWest > t_debounce) {
			digitalWrite(pin.ttlWestOn, HIGH);
			v_t_outLastWest = millis();
			v_isOnWest = true;
			v_isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[WestFun] WEST ON", v_t_outLastWest);
			}
		}
		v_t_inLastWest = millis();
	}
}

// South
void SouthInterrupt()
{
	SouthFun();
}
void SouthFun()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastSouth > t_debounce) {

			digitalWrite(pin.ttlSouthOn, HIGH);
			v_t_outLastSouth = millis();
			v_isOnSouth = true;
			v_isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[SouthFun] SOUTH ON", v_t_outLastSouth);
			}
		}
		v_t_inLastSouth = millis();
	}
}

// East
void EastInterrupt()
{
	EastFun();
}
void EastFun()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastEast > t_debounce) {

			digitalWrite(pin.ttlEastOn, HIGH);
			v_t_outLastEast = millis();
			v_isOnEast = true;
			v_isOnAny = true;

			// Print
			if (db.print_flow) {
				DebugFlow("[EastFun] EAST ON", v_t_outLastEast);
			}
		}
		v_t_inLastEast = millis();
	}
}

#pragma endregion

#pragma endregion


void setup()
{
	// Local varss
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

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

	// Set relay/ttl pin direction
	pinMode(pin.relIR, OUTPUT);
	pinMode(pin.relWhiteNoise, OUTPUT);
	pinMode(pin.relRewTone, OUTPUT);
	pinMode(pin.ttlIR, OUTPUT);
	pinMode(pin.ttlRewTone, OUTPUT);
	pinMode(pin.ttlWhiteNoise, OUTPUT);

	// Set relay/ttl pins low
	digitalWrite(pin.relIR, LOW);
	digitalWrite(pin.relRewTone, LOW);
	digitalWrite(pin.relWhiteNoise, LOW);
	digitalWrite(pin.ttlIR, LOW);
	digitalWrite(pin.ttlRewTone, LOW);
	digitalWrite(pin.ttlWhiteNoise, LOW);

	// Set other ttl pins
	pinMode(pin.ttlRewOn, OUTPUT);
	pinMode(pin.ttlRewOff, OUTPUT);
	pinMode(pin.ttlBullRun, OUTPUT);
	pinMode(pin.ttlBullStop, OUTPUT);
	pinMode(pin.ttlPidRun, OUTPUT);
	pinMode(pin.ttlPidStop, OUTPUT);

	// Set ir blink switch
	pinMode(pin.BlinkSwitch_Gnd, OUTPUT);
	digitalWrite(pin.BlinkSwitch_Gnd, LOW);
	pinMode(pin.BlinkSwitch, INPUT_PULLUP);

	// SET UP SERIAL STUFF

	// XBee
	Serial1.begin(57600);

	// CS through programming port
	Serial.begin(57600);

	// Serial monitor
	SerialUSB.begin(0);

	// Wait for SerialUSB if debugging
	uint32_t t_check = millis() + 100;
	if (DO_PRINT_DEBUG) {
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
	if (db.doPrintPimMapTest) {
		DebugPinMap();
	}

	// PRINT DEBUG STATUS

	// Print run mode
	if (DO_DEBUG) {
		DebugFlow("RUN MODE = DEBUG");
	}
	else {
		DebugFlow("RUN MODE = RELEASE");
	}

	// Print settings
	sprintf(str, "[setup] RUNNING IN %s MODE: |%s%s",
		DO_DEBUG ? "DEBUG" : "RELEASE",
		DO_LOG ? "LOGGING ENABLED|" : "",
		DO_PRINT_DEBUG ? "PRINTING ENABLED|" : "");
	DebugFlow(str);

	// PRINT SETUP FINISHED
	DebugFlow("[setup] FINISHED: Setup");

	// PRINT AVAILABLE MEMORY
	sprintf(str, "[setup] AVAILABLE MEMORY: %0.2fKB",
		(float)freeMemory() / 1000);
	DebugFlow(str);
	while (PrintDebug());

	// SET WHITE NOISE RELAY HIGH
	digitalWrite(pin.relWhiteNoise, HIGH);

	// SHOW RESTART BLINK
	delayMicroseconds(100);
	StatusBlink();

}


void loop()
{
	// Local vars
	char str[maxStoreStrLng];

	// TRACK LOOPS
	cnt_loop_tot = 0;
	cnt_loop_short = cnt_loop_short < 999 ? cnt_loop_short + 1 : 1;

	// RESET TTL PINS
	ResetTTL();

	// GET SERIAL INPUT
	GetSerial();

	// SEND DATA
	if (SendPacket());

	// PRINT QUEUED DB
	else if (PrintDebug());

	// SEND QUEUED LOG
	else(SendLog());

	// WAIT FOR HANDSHAKE
	if (!CheckForStart()) {
		return;
	}

	// HANDLE SERIAL INPUT
	if (r2a.isNew)
	{

		// (t) SESTEM TEST
		if (r2a.idNow == 't') {

			// Wall IR timing test
			if (r2a.dat[0] == 5)
			{

				// Setup
				if (r2a.dat[1] == 0)
				{
					// Log/rint test
					DebugFlow("DO TEST: WALL IR TIMING TEST");

					// Make sure IR off
					PulseIR(0, 0, 0);

					// Set to block IR pulse
					fc.doBlockIRPulse = true;
				}

				// Test finished
				if (r2a.dat[1] == 2)
				{
					// Unblock IR
					fc.doBlockIRPulse = false;

					// Unset flag
					fc.doBlockIRPulse = false;
				}

			}

			// Sync IR timing test
			if (r2a.dat[0] == 6)
			{
				// Setup
				if (r2a.dat[1] == 0)
				{
					// Log/rint test
					DebugFlow("DO TEST: SYNC IR TIMING TEST");

					// Set flag
					db.do_irSyncCalibration = true;

					// Make sure IR off
					PulseIR(0, 0, 0);

					// Set to block IR pulse
					fc.doBlockIRPulse = true;
				}

				// Pulse IR
				if (r2a.dat[1] == 1)
				{
					// Unblock IR
					fc.doBlockIRPulse = false;

					// Turn on IR
					PulseIR(0, 0, 1);
					delayMicroseconds(10 * 1000);

					// Turn off IR
					PulseIR(0, 0, 0);

					// Turn block back on
					fc.doBlockIRPulse = true;

				}

				// Test finished
				if (r2a.dat[1] == 2)
				{
					// Unblock IR
					fc.doBlockIRPulse = false;

					// Unset flag
					fc.doBlockIRPulse = false;
				}

			}

			// Hardware test
			if (r2a.dat[0] == 7)
			{
				// Setup
				if (r2a.dat[1] == 0)
				{
					// Log/rint test
					DebugFlow("DO TEST: HARDWARE");

					// Set to block IR pulse
					fc.doBlockIRPulse = true;
				}

				// Run
				if (r2a.dat[1] == 1)
				{
					HardwareTest(r2a.dat[2]);
				}

				// Test finished
				if (r2a.dat[1] == 2)
				{
					// Unblock IR
					fc.doBlockIRPulse = false;
				}


			}
		}

		// (r) RUN REWARD TONE
		if (r2a.idNow == 'r') {

			// Get reward duration and convert to ms
			rewDur = (uint32_t)r2a.dat[0];

			// Run tone
			StartRew();
		}

		// (s) SESSION SETUP
		else if (r2a.idNow == 's') {

			// No noise
			if (r2a.dat[0] == 0)
			{
				fc.doWhiteNoise = false;
				fc.doRewTone = false;
				DebugFlow("[loop] NO SOUND");
			}

			// White noise only
			else if (r2a.dat[0] == 1)
			{
				
				// Set flags
				fc.doWhiteNoise = true;
				fc.doRewTone = false;
				DebugFlow("[loop] DONT DO TONE");

			}

			// White and reward sound
			else if (r2a.dat[0] == 2)
			{

				// Set flags
				fc.doWhiteNoise = true;
				fc.doRewTone = true;
				DebugFlow("[loop] DO TONE");

			}

			// Create white, tone and reward word
			if (fc.doWhiteNoise && fc.doRewTone) {

				// Create word
				int sam_on_pins[3] = { pin.sam_relRewTone, pin.sam_ttlRewTone, pin.sam_ttlRewOn };
				word_rewStr = GetPortWord(0x0, sam_on_pins, 3);
				int sam_off_pins[3] = { pin.sam_relWhiteNoise, pin.sam_ttlWhiteNoise, pin.sam_ttlRewOff };
				word_rewEnd = GetPortWord(0x0, sam_off_pins, 3);
			
			}

			// Create reward event only word
			else {

				// Create word
				int sam_on_pins[1] = { pin.sam_ttlRewOn };
				word_rewStr = GetPortWord(0x0, sam_on_pins, 1);
				int sam_off_pins[1] = { pin.sam_ttlRewOff };
				word_rewEnd = GetPortWord(0x0, sam_off_pins, 1);
			}

			// Turn on white noise
			if (fc.doWhiteNoise) {

				// Set relay back to low first
				digitalWrite(pin.relWhiteNoise, LOW);
				
				// Set port 
				int sam_white_pins[2] = { pin.sam_relWhiteNoise, pin.sam_ttlWhiteNoise };
				uint32_t word_white = GetPortWord(0x0, sam_white_pins, 2);
				SetPort(word_white, 0x0);
			}

		}

		// (p) SIGNAL PID MODE
		else if (r2a.idNow == 'p') {
			// Signal PID stopped
			if (r2a.dat[0] == 0)
			{
				digitalWrite(pin.ttlPidStop, HIGH);
				digitalWrite(pin.ttlPidRun, LOW);
				DebugFlow("[loop] PID Stopped");
			}
			// Signal PID running
			else if (r2a.dat[0] == 1)
			{
				digitalWrite(pin.ttlPidRun, HIGH);
				digitalWrite(pin.ttlPidStop, LOW);
				DebugFlow("[loop] PID Started");
			}
		}

		// (b) SIGNAL BULLDOZE MODE
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

		// (q) DO QUIT
		else if (r2a.idNow == 'q') {

			// Log warnings summary
			char warn_lines[200] = "ON LINES |";
			for (int i = 0; i < cnt_warn; i++) {
				char str_lin[10];
				sprintf(str_lin, "%d|", warn_line[i]);
				strcat(warn_lines, str_lin);
			}
			sprintf(str, "TOTAL WARNINGS: %d %s", cnt_warn, cnt_warn > 0 ? warn_lines : "");
			DebugFlow(str);

			// Log errors summary
			char err_lines[200] = "ON LINES |";
			for (int i = 0; i < cnt_err; i++) {
				char str_lin[10];
				sprintf(str_lin, "%d|", err_line[i]);
				strcat(err_lines, str_lin);
			}
			sprintf(str, "TOTAL ERRORS:  %d %s", cnt_err, cnt_err > 0 ? warn_lines : "");
			DebugFlow(str);

			// Set flags
			fc.doQuit = true;
			DebugFlow("[loop] QUITING...");
		}

	}

	// CHECK FOR REWARD END
	if (fc.isRewarding) {
		EndRew();
	}

	// PULSE IR
	if (PulseIR(del_irSyncPulse, dt_irSyncPulseOn)) {

		// Itterate count
		if (is_irOn) {
			cnt_ir++;
		}

		// Log ir event
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

		// Make sure we wont recieve a resend request
		if (millis() < r2a.t_rcvd + 500) {
			return;
		}

		// Run bleep bleep
		QuitBeep();

		// Restart Arduino
		REQUEST_EXTERNAL_RESET;
	}

}
