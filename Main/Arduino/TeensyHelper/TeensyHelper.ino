
//NOTES
/*
	Changd "C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3\serial3"
	SERIAL2_TX_BUFFER_SIZE 40 to 128
	SERIAL2_RX_BUFFER_SIZE 64 to 2048
	Cant 'see' these directives from script for some reason
*/

// Set to print debug info to console
#define DO_PRINT_DEBUG 0
#define DO_LOG_DEBUG 1
#define DO_PRINT_LOGS 0

#include <SPI.h>

#include "SdFat.h"

#pragma region ============ VARIABLE SETUP =============

// PIN MAPPING
struct PIN
{
	// Status led
	int StatLED = 13;

	// Due coms
	int Teensy_Unused = 14;
	int Teensy_SendLogs = 15;
	int Teensy_Resetting = 16;
}
// Initialize
pin;

// SERIAL COMS

// Lower packet range
const uint16_t lower_pack_range[2] = { 1, UINT16_MAX / 2 };

// Uper packet range
const uint16_t upper_pack_range[2] = { (UINT16_MAX / 2) + 1, UINT16_MAX };

const char tnsy_id_list[1] =
{
	'\0'
};

struct R42T
{
	HardwareSerial2 &port;
	const char *objID;
	const uint16_t packRange[2];
	const int lng;
	const char head;
	const char foot;
	const char *id;
	uint16_t packInd;
	uint32_t packSentAll;
	uint32_t packRcvdAll;
	int cnt_dropped;
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
};

// Initialize C2R
R42T r42t
{
	// serial
	Serial2,
	// objID
	"t2r",
	// packRange
	{ lower_pack_range[0], lower_pack_range[1] },
	// lng
	0,
	// head
	'{',
	// foot
	'}',
	// id
	tnsy_id_list,
	// packInd
	0,
	// packSentAll
	0,
	// packRcvdAll
	0,
	// cnt_dropped
	0,
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

//UNION: UTAG TEENSY
union UTAG_TEENSY {
	byte b[16]; // (byte) 1 byte
	char c[16]; // (char) 1 byte
	uint16_t i16[8]; // (uint16_t) 2 byte
	uint32_t i32[4]; // (uint32_t) 4 byte
	uint64_t i64[2]; // (uint64_t) 8 byte
};
UTAG_TEENSY Utnsy;

// Status stuff
uint32_t t_blink = 0;
uint32_t dt_blink = 100; // (ms)
bool is_ledOn = false;

// Debugging general
const uint16_t maxStoreStrLng = 300;
const uint16_t logSize = 40;
uint16_t cnt_logsStored = 0;
uint16_t cnt_logsSkipped = 0;
byte logInd = 0;
byte logIndArr[logSize] = { 0 };
char Log[logSize][maxStoreStrLng] = { { 0 } };
uint16_t cnt_reset = 0;
bool isLoggingReady = false;

// Serial com general
uint32_t t_sync = 0;

// SD Logging
char logDir[10] = "LOGS";
char logCountFi[20] = "LOGCNT.bin";
char logFiStr[100] = { 0 };
uint32_t cnt_logsFiles = 0;
uint32_t dt_flush = 500; // (ms)
uint32_t t_flush = 0;
SdFatSdioEX sdEx;
File logFileSD;

// union
union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint8_t i8[4]; // (uint8_t) 1 byte
	uint16_t i16[2]; // (int16) 2 byte
	uint32_t i32; // (int32) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
} U;

#pragma endregion


#pragma region ========= FUNCTION DECLARATIONS =========

// RESET VARIABLES
void RunReset();

// CHANGE TO LOG DIRECTORY
void SetLogDir();

// OPEN NEW LOG FOR WRITING
void OpenNewLog(bool is_log_named);

// GET LOG NUMBER
uint32_t GetFileCount();

// PARSE SERIAL INPUT
void GetSerial();

// STORE MESSAGE
void StoreMessage(char msg[]);

// SEND LOGS
void SendLastLogs();

// FORMAT AND PRINT MESSAGE
void DebugTeensy(char msg[], uint32_t t = millis());

// RUN STATUS BLINK
void StatusBlink(bool do_force = true);

#pragma endregion 


#pragma region ========== FUNCTION DEFINITIONS =========

// RESET VARIABLES
void RunReset() {

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	uint32_t t_flicker = 0;

	// Itterate count
	cnt_reset++;

	// Reset variables
	isLoggingReady = false;
	cnt_logsStored = 0;
	cnt_logsSkipped = 0;
	logInd = 0;
	r42t.packInd = 0;
	r42t.packSentAll = 0;
	r42t.packRcvdAll = 0;
	r42t.cnt_dropped = 0;
	r42t.t_rcvd = 0;
	r42t.dt_rcvd = 0;

	// Reinitialize arrays
	for (size_t i = 0; i < logSize; i++) {
		logIndArr[i] = i;
		Log[i][0] = '\0';
	}

	// Set log entry to null
	sprintf(logFiStr, "NULL");

	// Dump buffer
	while (r42t.port.available() > 0) {
		r42t.port.read();
	}

	// Set reset indicator pin high
	sprintf(str, "[RunReset] Setting Reset Pin High");
	DebugTeensy(str);
	digitalWrite(pin.Teensy_Resetting, HIGH);

	// Flicker LED
	t_flicker = millis() + 500;
	while (millis() < t_flicker) {
		StatusBlink();
		delay(25);
	}

	// Set reset indicator pin back to low
	digitalWrite(pin.Teensy_Resetting, LOW);
	sprintf(str, "[RunReset] Setting Reset Pin Low");
	DebugTeensy(str);

	// Check RX buffer
	sprintf(str, "[RunReset] RX BUFFER CONTAINS %dB", r42t.port.available());
	DebugTeensy(str);

	// Check TX buffer
	if (r42t.port.availableForWrite() + 1 == 512) {
		sprintf(str, "[RunReset] TX BUFFER SIZE: ACTUAL=%dB EXPECTED=512B", r42t.port.availableForWrite() + 1);
	}
	// Buffer size is wrong
	else {
		sprintf(str, "!!ERROR!! [RunReset] TX BUFFER SIZE: ACTUAL=%dB EXPECTED=512B", r42t.port.availableForWrite() + 1);
	}
	DebugTeensy(str);

	// Initialize SD card
	if (sdEx.begin()) {
		DebugTeensy("[RunReset] FINISHED: INITIALIZE SD");
	}
	else {
		DebugTeensy("!!ERROR!! [RunReset] FAILED: INITIALIZE SD");
	}

	// Start in root directory
	if (sdEx.chdir()) {
		DebugTeensy("[RunReset] FINISHED: SET SD TO ROOT DIRECTORY");
	}
	else {
		DebugTeensy("!!ERROR!! [RunReset] FAILED: SET SD TO ROOT DIRECTORY");
	}

	// Print status
	sprintf(str, "[RunReset] FINISHED RESET %d", cnt_reset);
	DebugTeensy(str);

}

// OPEN NEW LOG FOR WRITING
void OpenNewLog(bool is_log_named) {

	// Local vars
	static char str[100] = { 0 }; str[0] = '\0';
	static char file_str[50] = { 0 };

	// Go to log directory
	SetLogDir();

	// Get next log number
	cnt_logsFiles = GetFileCount();

	// Reset logs if > 50 logs
	if (cnt_logsFiles > 50) {

		// Move back to root directory
		if (sdEx.chdir()) {
			DebugTeensy("[OpenNewLog] FINISHED: SET SD BACK TO ROOT DIRECTORY");
		}
		else {
			DebugTeensy("!!ERROR!! [OpenNewLog] FAILED: SET SD TO ROOT DIRECTORY");
		}

		// Wipe all data
		if (sdEx.vwd()->rmRfStar()) {
			sprintf(str, "[OpenNewLog] FINISHED: WIPE ALL %d FILES FROM SD CARD", cnt_logsFiles);
		}
		else {
			sprintf(str, "!!ERROR!! [OpenNewLog] FAILED: WIPE ALL %d FILES FROM SD CARD", cnt_logsFiles);
		}
		DebugTeensy(str);

		// Reinitialize SD card
		sdEx.begin();

		// Create/change to log directory
		SetLogDir();

		// Get new log number
		cnt_logsFiles = GetFileCount();

	}

	// Check if log file name already set
	if (!is_log_named) {
		sprintf(logFiStr, "NULL_LOG%05u", cnt_logsFiles);
	}

	// Format log file name
	sprintf(file_str, "%s.CSV", logFiStr);

	// Attempt to open/create count file
	if (logFileSD.open(file_str, O_RDWR | O_CREAT)) {
		sprintf(str, "[OpenNewLog] FINISHED: CREATE & OPEN \"%s\"", file_str);
	}
	else {
		sprintf(str, "!!ERROR!! [OpenNewLog] FAILED: CREATE & OPEN \"%s\"", file_str);
	}
	DebugTeensy(str);

}

// CHANGE TO LOG DIRECTORY
void SetLogDir() {

	// Local vars
	static char str[100] = { 0 }; str[0] = '\0';

	// Check for main directory
	if (!sdEx.exists(logDir)) {

		// Make main directory
		if (sdEx.mkdir(logDir)) {
			sprintf(str, "[GetFileCount] CREATED \"%s\" DIR", logDir);
		}
		else {
			sprintf(str, "!!ERROR!! [GetFileCount] FAILED: CREATE \"%s\" DIR", logDir);
		}
		DebugTeensy(str);

	}

	// Change directory
	if (sdEx.exists(logDir))
		if (sdEx.chdir(logDir)) {
			sprintf(str, "[GetFileCount] CD TO \"%s\" DIR", logDir);
		}
		else {
			sprintf(str, "!!ERROR!! [GetFileCount] FAILED: CD TO \"%s\" DIR", logDir);
		}
	else {
		sprintf(str, "**WARNING** [GetFileCount] SKIPPED: CD TO \"%s\" DIR", logDir);
	}
	DebugTeensy(str);

}

// GET LOG NUMBER
uint32_t GetFileCount() {

	// Local vars
	static char str[100] = { 0 }; str[0] = '\0';
	File log_num_file_sd;

	// Attempt to open/create count file
	if (log_num_file_sd.open(logCountFi, O_RDWR | O_CREAT)) {
		sprintf(str, "[GetFileCount] OPENED \"%s\" FILE", logCountFi);
	}
	else {
		sprintf(str, "!!ERROR!! [GetFileCount] FAILED: OPENING \"%s\" FILE", logCountFi);
	}
	DebugTeensy(str);

	// Read in log number 
	U.i32 = 0;
	log_num_file_sd.read(U.i8, 4);
	sprintf(str, "[GetFileCount] CURRENTLY %lu LOGS STORED", U.i32);
	DebugTeensy(str);

	// Set current position to zero
	log_num_file_sd.rewind();

	// Itterate and store count
	U.i32++;
	log_num_file_sd.write(U.i8, 4);

	// Close log num count
	log_num_file_sd.close();

	// Return number
	return U.i32;

}

// PARSE SERIAL INPUT
void GetSerial()
{
	// Notes
	/*
		// Coms Union (20B):
			1B/1B: (char)head: c[0]
			1B/2B: (char)id: c[1]
			2B/4B: (uint16_t)pack: i16[1]
			4B/8B: (uint32_t)time_ms: i32[1]
			2B/10B: (uint16_t)line_number: i16[4]
			2B/12B: (char)function_abreviation: c[10-11]
			1B/13B: (byte)loop_count: b[12]
			1B/14B: (byte)skip_count: b[13]
			1B/15B: (byte)memory_GB: b[14]
			1B/16B: (char)foot: c[15]
	*/

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char msg_in[maxStoreStrLng] = { 0 }; msg_in[0] = '\0';
	static char msg_store[maxStoreStrLng] = { 0 }; msg_store[0] = '\0';
	uint32_t t_timeout = millis() + 100;
	bool is_timedout = false;
	int bytes_read = 0;
	char head = 0;
	char foot = 0;
	char id = 0;
	uint16_t pack = 0;
	uint32_t t_m = 0;
	uint16_t line_num = 0;
	char fun_abr[3] = { 0 };
	byte cnt_loop = 0;
	byte cnt_skip = 0;
	byte mem_gb = 0;
	static byte msg_bytes = sizeof(Utnsy.b);

	// Bail if old logs not sent yet
	if (!isLoggingReady ||
		digitalRead(pin.Teensy_SendLogs) == HIGH) {
		return;
	}

	// Check for new data
	if (r42t.port.available() == 0) {

		// Flush sd queue
		if (logFileSD.isOpen() &&
			millis() - t_flush > dt_flush &&
			millis() - r42t.t_rcvd > dt_flush) {

			// Flush and store time
			logFileSD.flush();
			t_flush = millis();
		}

		// Bail
		return;
	}

	// Clear union entries
	Utnsy.i64[0] = 0;
	Utnsy.i64[1] = 0;

	// Read in complete message
	while (bytes_read < msg_bytes) {

		// Check for new data
		if (r42t.port.available() > 0) {

			// Read next byte
			byte b = r42t.port.read();

			// Add to union if header found
			if (b == r42t.head || Utnsy.b[0] == r42t.head) {

				// Add to union
				Utnsy.b[bytes_read++] = b;

			}

		}

		// Bail if timedout
		if (millis() > t_timeout) {
			is_timedout = true;
			break;
		}
	}

	// Flicker status light
	StatusBlink();

	// Store header
	head = Utnsy.c[0];
	// Store message id
	id = Utnsy.c[1];
	// Store packet number
	pack = Utnsy.i16[1];

	// Check for log file message
	if (id == 'L') {

		// Itterate through message
		int b_ind = 4;
		for (int i = 0; i < msg_bytes; i++) {

			// Check for end
			if (Utnsy.c[b_ind] != '\0') {
				msg_in[i] = Utnsy.c[b_ind];
				b_ind++;
			}
			else {
				msg_in[i] = '\0';
				break;
			}
		}

	}

	// Store call function details
	else {

		// Store time
		t_m = Utnsy.i32[1];
		// Store line number
		line_num = Utnsy.i16[4];
		// Store function abbreviation
		fun_abr[0] = Utnsy.c[10];
		fun_abr[1] = Utnsy.c[11];
		// Store loop count
		cnt_loop = Utnsy.b[12];
		// Store skip count
		cnt_skip = Utnsy.b[13];
		// Store memory
		mem_gb = Utnsy.b[14];

		// Format recieved message
		sprintf(msg_in, "%lu,%d,[%s:%d] id=%c pack=%d mem=%dGB cnt_skip=%d",
			t_m, cnt_loop, fun_abr, line_num, id, pack, mem_gb, cnt_skip);

	}

	// Store footer
	foot = Utnsy.c[15];

	// Check for missing header or footer
	if (head != r42t.head || foot != r42t.foot) {

		// Log/print incomplete message
#if DO_PRINT_DEBUG || DO_LOG_DEBUG
		if (r42t.packRcvdAll > 0) {
			sprintf(str, "**WARNING** [GetSerial] MISSING |%s%s: t_out=%s b_read=%d rx_buff=%d head=%c foot=%c \"%s\"",
				head != r42t.head ? "HEAD|" : "", foot != r42t.foot ? "FOOT|" : "",
				is_timedout ? "true" : "false", bytes_read, r42t.port.available(), head, foot, msg_in);
			DebugTeensy(str);
		}
#endif

		// Bail
		return;
	}

	// Create log file
	if (strcmp(logFiStr, "NULL") == 0 || !logFileSD.isOpen()) {

		// Check msg contains "LOG"
		if (id == 'L') {

			// Name and open log file
			sprintf(logFiStr, "TEENSY_LOG%s", msg_in);
			OpenNewLog(true);
		}

		// Create generic log file
		else {
			// Open log file
			OpenNewLog(false);
			sprintf(str, "!!ERROR!! [StoreMessage] MISSING LOG NAME MESSAGE: msg=\"%s\"", msg_in);
			DebugTeensy(str);
		}

	}

	// Update skipped logs
	cnt_logsSkipped += cnt_skip;

	// Get pack diff accounting for packet rollover
	int pack_diff =
		max((pack - r42t.packInd), -(pack - r42t.packInd)) < (r42t.packRange[1] - r42t.packRange[0]) ?
		pack - r42t.packInd :
		pack - (r42t.packRange[0] - 1);

	// Update dropped packets
	int cnt_dropped = (pack - r42t.packInd) - 1;
	r42t.cnt_dropped += pack_diff - 1;

	// Log/print dropped packs
#if DO_PRINT_DEBUG || DO_LOG_DEBUG
	if (cnt_dropped > 0) {
		sprintf(str, "**WARNING** [GetSerial] DROPPED PACKETS: cnt=|%d|%d|", cnt_dropped, r42t.cnt_dropped);
		DebugTeensy(str);
	}
#endif

	// Update packet ind 
	r42t.packInd = pack;

	// Update packets sent
	r42t.packSentAll += pack_diff;

	// Itterate packets recieved
	r42t.packRcvdAll++;

	// Store recieve time
	r42t.t_rcvd = millis();

	// Format message to store
	sprintf(msg_store, "FEEDERDUE,%lu,[%lu],%s",
		r42t.packRcvdAll, r42t.packSentAll, msg_in);

	// Store message
	StoreMessage(msg_store);

}

// STORE MESSAGE
void StoreMessage(char msg[])
{
	// Store in memory
	sprintf(Log[logInd], "%c%s%c", r42t.head, msg, r42t.foot);

	// Shift ind array
	for (size_t i = 0; i < logSize - 1; i++) {
		logIndArr[i] = logIndArr[i + 1];
	}
	logIndArr[logSize - 1] = logInd;

	// Update log ind and count
	cnt_logsStored += cnt_logsStored < logSize ? 1 : 0;
	logInd++;

	// Check if ind should roll over 
	if (logInd == logSize) {

		// Reset queueIndWrite
		logInd = 0;
	}

	// Write to SD log
	logFileSD.print(msg);
	logFileSD.print('\r');
	logFileSD.print('\n');

	// Print message
#if DO_PRINT_LOGS
	DebugTeensy(msg);
#endif

}

// SEND LAST LOGS
void SendLastLogs()
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char msg_str[maxStoreStrLng] = { 0 }; msg_str[0] = '\0';
	uint32_t t_check = millis() + 100;
	char c_arr[4] = { 0 };

	// Check for high pin
	if (digitalRead(pin.Teensy_SendLogs) != HIGH) {
		return;
	}

	// Make sure its HIGH for at least 100 ms
	while (millis() < t_check) {

		// Check for high pin
		if (digitalRead(pin.Teensy_SendLogs) != HIGH) {
			return;
		}

		// Check for start string
		if (r42t.port.available() > 0) {

			// Get next byte
			c_arr[0] = c_arr[1];
			c_arr[1] = c_arr[2];
			c_arr[2] = r42t.port.read();
		}

		// Break out if start string found
		if (strcmp(c_arr, "<<<") == 0) {
			DebugTeensy("[SendLastLogs] Recieved Start String \"<<<\"");
			break;
		}
	}

	// Bail out if start string not found
	if (strcmp(c_arr, "<<<") != 0) {
		return;
	}

	// Log/print
	sprintf(str, "[SendLastLogs] Begin Sending %d of %lu Logs...",
		cnt_logsStored, r42t.packRcvdAll);
	DebugTeensy(str);

	// Wait 100 ms
	delay(100);

	// Check if logs to send
	if (cnt_logsStored > 0) {

		// Send logs
		for (size_t i = 0; i < logSize; i++) {
			delay(10);
			if (Log[logIndArr[i]][0] == '\0') {
				continue;
			}

			// Send log and blink status
			r42t.port.write(Log[logIndArr[i]]);
			StatusBlink();

#if DO_PRINT_LOGS
			DebugTeensy(Log[logIndArr[i]]);
#endif
			}

		// Format summary
		sprintf(str, "TEENSY SUMMARY \"%s\": SENT=%d RCVD=%lu SKIPPED=%d DROPPED=%d FILES=%d",
			logFiStr, r42t.packSentAll, r42t.packRcvdAll, cnt_logsSkipped, r42t.cnt_dropped, cnt_logsFiles);
		sprintf(msg_str, "%c%s%c", r42t.head, str, r42t.foot);
		
		// Send summary
		r42t.port.write(msg_str);

		// Log/print summary
		DebugTeensy(str);

		}

	// No logs to send
	else {
		r42t.port.write("TEENSY SUMMARY: NO LOGS STORED");
		DebugTeensy(str);
	}

	// End reached send ">>>"
	r42t.port.write(">>>");
	delay(100);

	// Log/print
	sprintf(str, "[SendLastLogs] Finished Sending %d of %lu Logs",
		cnt_logsStored, r42t.packRcvdAll);
	DebugTeensy(str);

	// Close log file
	if (logFileSD.isOpen()) {
		logFileSD.close();
	}

	// Reset Stuff
	RunReset();

	// Set flag
	isLoggingReady = true;

	// Print logging ready
	DebugTeensy("[SendLastLogs] READY TO RECIEVE LOGS");

	}

// FORMAT AND LOG/PRINT MESSAGE
void DebugTeensy(char msg[], uint32_t t)
{

#if DO_PRINT_DEBUG || DO_LOG_DEBUG

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	static char log_str[maxStoreStrLng] = { 0 }; log_str[0] = '\0';
	char str_time[100] = { 0 };
	uint32_t t_m = 0;
	float t_s = 0;

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Make time string
	sprintf(str_time, "[%0.3f]", t_s);

	// Add space after time
	char spc[50] = { 0 };
	char arg[50] = { 0 };
	sprintf(arg, "%%%ds", 15 - strlen(str_time));
	sprintf(spc, arg, '_');

	// Put it all together
	sprintf(str, "%s%s%s\r\n", str_time, spc, msg);

#if DO_LOG_DEBUG

	// Write to SD log
	if (logFileSD.isOpen()) {

		// Format log string
		sprintf(log_str, "TEENSY,,,,%lu,%s\r\n",
			t_m, msg);

		// Write to SD card
		logFileSD.print(log_str);
	}

#endif

#if DO_PRINT_DEBUG

	// Print it
	SerialUSB.print(str);

#endif

#endif

}

// RUN STATUS BLINK
void StatusBlink(bool do_force) {

	// Do status blink
	if (do_force ||
		millis() > t_blink + dt_blink) {

		// Flip state and run
		digitalWrite(pin.StatLED, is_ledOn ? LOW : HIGH);
		is_ledOn = !is_ledOn;
		t_blink = millis();
	}

}

#pragma endregion


void setup()
{

	// SET UP SERIAL STUFF

	// Serial monitor
#if DO_PRINT_DEBUG
	SerialUSB.begin(0);
#endif 

	// FeederDue Serial [57600, 115200, 256000]
	r42t.port.begin(256000);

	// SETUP PINS

	// Set direction
	pinMode(pin.Teensy_SendLogs, INPUT);
	pinMode(pin.Teensy_Resetting, OUTPUT);
	pinMode(pin.StatLED, OUTPUT);

	// Set initial state
	digitalWrite(pin.StatLED, LOW);
	digitalWrite(pin.Teensy_Resetting, LOW);

	// Show setup blink
	uint32_t t_flicker = millis() + 500;
	while (millis() < t_flicker) {
		StatusBlink();
		delay(100);
	}

	// RUN RESET
	RunReset();
}


void loop()
{

	// Do status blink
	StatusBlink(false);

	// Get new data
	GetSerial();

	// Check if ready to send logs
	SendLastLogs();

}

