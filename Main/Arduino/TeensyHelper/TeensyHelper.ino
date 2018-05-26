/*
NOTES
	Changd "C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3\serial3"
	SERIAL2_TX_BUFFER_SIZE 40 to 128
	SERIAL2_RX_BUFFER_SIZE 64 to 512
	Cant 'see' these directives from script for some reason  
*/

// Set to print debug info to console
#define DO_PRINT_DEBUG 0
#define DO_LOG_DEBUG 1
#define DO_PRINT_LOGS 0

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

const char tnsy_id_list[1] =
{
	'\0'
};

struct R42T
{
	HardwareSerial2 &port;
	const char *instID;
	const int lng;
	const char head;
	const char foot;
	const char *id;
	uint16_t packTot;
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
};

// Initialize C2R
R42T r42t
{
	// serial
	//Serial2,
	Serial2,
	// instID
	"t2r",
	// lng
	0,
	// head
	'(',
	// foot
	')',
	// id
	tnsy_id_list,
	// packTot
	0,
	// t_rcvd
	0,
	// dt_rcvd
	0,
};

// Status stuff
uint32_t t_blink = 0;
uint32_t dt_blink = 100; // (ms)
bool is_ledOn = false;

// Debugging general
const uint16_t maxStoreStrLng = 300;
const uint16_t logSize = 40;
bool is_startConfirmed = false;
uint32_t cnt_logsRcvd = 0;
uint16_t cnt_logsStored = 0;
byte logInd = 0;
byte logIndArr[logSize] = { 0 };
char Log[logSize][maxStoreStrLng] = { { 0 } };
uint16_t cnt_reset = 0;

// Serial com general
uint32_t t_sync = 0;
uint16_t buffInd = 0;
char readBuff[maxStoreStrLng] = { 0 };
uint32_t packTotBytesRead = 0;
uint32_t packTotBytesDiscarded = 0;

// SD Logging
char logFiStr[100] = { 0 };
uint32_t dt_flush = 500; // (ms)
uint32_t t_flush = 0;
SdFatSdioEX sdEx;
File logFileSD;
float freeMemGB = 0;

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

// OPEN NEW LOG FOR WRITING
void OpenNewLog();

// GET LOG NUMBER
uint32_t GetLogNumber();

// PARSE SERIAL INPUT
void GetSerial();

// WAIT FOR BUFFER TO FILL
char WaitBuffRead(int timeout, char mtch = '\0');

// STORE MESSAGE
void StoreMessage(char msg[], uint16_t pack);

// SEND LOGS
void SendLogs();

// FORMAT AND PRINT MESSAGE
void PrintDebug(char msg[], uint32_t t = millis());

// RUN STATUS BLINK
void StatusBlink(bool do_force = true);

#pragma endregion 


#pragma region ========== FUNCTION DEFINITIONS =========

// RESET VARIABLES
void RunReset() {

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

	// Itterate count
	cnt_reset++;

	// Reset variables
	is_startConfirmed = false;
	cnt_logsRcvd = 0;
	cnt_logsStored = 0;
	logInd = 0;
	buffInd = 0;
	readBuff[0] = '\0';
	packTotBytesRead = 0;
	packTotBytesDiscarded = 0;

	// Reinitialize arrays
	for (size_t i = 0; i < logSize; i++) {
		logIndArr[i] = i;
		Log[i][0] = '\0';
	}

	// Set log entry to null
	sprintf(logFiStr, "%cNULL%c", r42t.head, r42t.foot);

	// Dump buffer
	while (r42t.port.available() > 0) {
		r42t.port.read();
	}

	// Set reset indicator pin high
	sprintf(str, "[RunReset] Setting Reset Pin High");
	PrintDebug(str);
	digitalWrite(pin.Teensy_Resetting, HIGH);

	// Flicker LED
	for (size_t i = 0; i < 40; i++)
	{
		StatusBlink();
		delay(25);
	}

	// Set reset indicator pin back to low
	digitalWrite(pin.Teensy_Resetting, LOW);
	sprintf(str, "[RunReset] Setting Reset Pin Low");
	PrintDebug(str);

	// Check RX buffer
	sprintf(str, "[RunReset] RX BUFFER CONTAINS %dB", r42t.port.available());
	PrintDebug(str);

	// Check TX buffer
	if (r42t.port.availableForWrite() + 1 == 128) {
		sprintf(str, "[RunReset] TX BUFFER SIZE: ACTUAL=%dB EXPECTED=128B", r42t.port.availableForWrite() + 1);
	}
		// Buffer size is wrong
	else {
		sprintf(str, "!!ERROR!! [RunReset] TX BUFFER SIZE: ACTUAL=%dB EXPECTED=128B", r42t.port.availableForWrite() + 1);
	}
	PrintDebug(str);

	// Print status
	sprintf(str, "[RunReset] FINISHED RESET %d", cnt_reset);
	PrintDebug(str);

}

// OPEN NEW LOG FOR WRITING
void OpenNewLog() {

	// Local vars
	static char str[100] = { 0 }; str[0] = '\0';
	static char file_str[50] = { 0 };
	uint32_t log_num = 0;
	uint32_t free_kb = 0;
	delay(1000);
	// Initialize SD card
	sdEx.begin();
	sdEx.chvol();

	// Get curent log number
	log_num = GetLogNumber();

	// Get free space on sd
	free_kb = sdEx.vol()->freeClusterCount();
	free_kb *= sdEx.vol()->blocksPerCluster() / 2;
	freeMemGB = (float)free_kb / 1000000;

	// Reset logs if < 2GB left or > 50 logs
	if (freeMemGB < 2 || log_num > 50) {

		// Move to root directory
		sdEx.chdir();

		// Wipe all data
		if (sdEx.vwd()->rmRfStar()) {
			PrintDebug("[OpenNewLog] WIPED SD CARD");
		}
		else {
			PrintDebug("!!ERROR!! [OpenNewLog] FAILED: WIPE SD CARD");
		}

		// Reinitialize SD card
		sdEx.begin();
		sdEx.chvol();

		// Get new log number
		log_num = GetLogNumber();

	}

	// Format log file name
	sprintf(file_str, "%s.CSV", logFiStr);
	PrintDebug(file_str);

	// Attempt to open/create count file
	if (logFileSD.open(file_str, O_RDWR | O_CREAT)) {
		sprintf(str, "[OpenNewLog] OPENED \"%s\" FILE", file_str);
	}
	else {
		sprintf(str, "!!ERROR!! [OpenNewLog] FAILED: OPENING \"%s\" FILE", file_str);
	}
	PrintDebug(str);

	// Print available space
	sprintf(str, "[OpenNewLog] %0.2fGB ABAILABLE SPACE ON 8GB SD", freeMemGB);
	PrintDebug(str);

}

// GET LOG NUMBER
uint32_t GetLogNumber() {

	// Local vars
	static char str[100] = { 0 }; str[0] = '\0';
	bool is_dir_exit = false;
	File log_num_file_sd;

	// Specify file and dir
	char log_dir[10] = "LOGS";
	char fi_num_dir[20] = "LOGCNT.bin";

	// Check for main directory
	if (!sdEx.exists(log_dir)) {

		// Make main directory
		if (sdEx.mkdir(log_dir)) {
			sprintf(str, "[GetLogNumber] CREATED \"%s\" DIR", log_dir);
		}
		else {
			sprintf(str, "!!ERROR!! [GetLogNumber] FAILED: CREATE \"%s\" DIR", log_dir);
		}
		PrintDebug(str);

	}

	// Change directory
	if (sdEx.chdir(log_dir)) {
		sprintf(str, "[GetLogNumber] \CD TO \"%s\" DIR", log_dir);
	}
	else {
		sprintf(str, "!!ERROR!! [GetLogNumber] FAILED: CD TO \"%s\" DIR", log_dir);
	}
	PrintDebug(str);

	// Attempt to open/create count file
	if (log_num_file_sd.open(fi_num_dir, O_RDWR | O_CREAT)) {
		sprintf(str, "[GetLogNumber] OPENED \"%s\" FILE", fi_num_dir);
	}
	else {
		sprintf(str, "!!ERROR!! [GetLogNumber] FAILED: OPENING \"%s\" FILE", fi_num_dir);
	}
	PrintDebug(str);

	// Read in log number 
	U.i32 = 0;
	log_num_file_sd.read(U.i8, 4);
	sprintf(str, "[GetLogNumber] CURRENTLY %lu LOGS", U.i32);
	PrintDebug(str);

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
	/*
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	uint16_t pack = 0;

	// Reset vars
	packTotBytesRead = 0;
	packTotBytesDiscarded = 0;

	// Check for new data
	if (r42t.port.available() == 0 ||
		digitalRead(pin.Teensy_SendLogs) == LOW) {

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

	// Check send log command
	if (!is_startConfirmed &&
		digitalRead(pin.Teensy_SendLogs) == LOW) {
		PrintDebug("[GetSerial] RECIEVED SEND LOG SIGNAL");
		is_startConfirmed = true;
	}

	// Set status LED high
	StatusBlink();

	// Dump data till msg header byte is reached
	WaitBuffRead(100, r42t.head);
	if (readBuff[buffInd] == 0) {

		// Indicate incomplete message
		if (cnt_logsRcvd > 0) {
#if DO_PRINT_DEBUG || DO_LOG_DEBUG
			sprintf(str, "**WARNING** [GetSerial] Missing Head: b_read=%lu b_dump=%lu",
				packTotBytesRead, packTotBytesDiscarded);
			PrintDebug(str);
#endif
		}

		// Bail
		return;
	}

	// Get packet num
	U.f = 0;
	U.b[0] = WaitBuffRead(100);
	U.b[1] = WaitBuffRead(100);
	pack = U.i16[0];

	// Read till foot found
	WaitBuffRead(200, r42t.foot);

	// Note if footer not found
	if (readBuff[buffInd] != r42t.foot) {

		// Indicate incomplete message
		if (cnt_logsRcvd > 0) {
#if DO_PRINT_DEBUG || DO_LOG_DEBUG
				sprintf(str, "**WARNING** [GetSerial] Missing Foot: b_read=%lu b_dump=%lu",
					packTotBytesRead, packTotBytesDiscarded);
				PrintDebug(str);
#endif
		}

		// Bail on first read
		else {
			return;
		}

	}

	// Update pack info
	r42t.packTot = pack;

	// Terminate string
	readBuff[buffInd] = '\0';

	// Update counts
	cnt_logsRcvd++;
	cnt_logsStored += cnt_logsStored < logSize ? 1 : 0;

	// Store message
	StoreMessage(readBuff, pack);

	// Store recieve time
	r42t.t_rcvd = millis();

	// Print every 100 log
	if (cnt_logsRcvd == 1 || cnt_logsRcvd % 100 == 0) {
		sprintf(str, "[GetSerial] Recieved %lu Logs", cnt_logsRcvd);
		PrintDebug(str);
	}


}

// WAIT FOR BUFFER TO FILL
char WaitBuffRead(int timeout, char mtch)
{
	// Local vars
	uint32_t t_timeout = millis() + timeout;

	// Reset buff ind
	buffInd = 0;

	// Wait for at least 1 byte
	while (r42t.port.available() < 1 &&
		millis() < t_timeout);

	// Get any byte
	if (mtch == '\0') {

		if (r42t.port.available() > 0) {

			// Get first byte
			readBuff[0] = r42t.port.read();

			// Itterate count
			packTotBytesRead++;

			// Add termination and bail
			readBuff[buffInd + 1] = '\0';
			return readBuff[buffInd];
		}
	}

	// Find specific byte
	while (
		readBuff[0] != mtch  &&
		millis() < t_timeout) {

		// Check new data
		if (r42t.port.available() > 0) {

			// Get next byte
			readBuff[buffInd] = r42t.port.read();

			// Itterate count
			packTotBytesRead++;

			// check match was found
			if (readBuff[buffInd] == mtch) {

				// Add termination and bail
				readBuff[buffInd + 1] = '\0';
				return readBuff[buffInd];
			}

			// Otherwise add to discard count
			else {

				// Itterate counts
				packTotBytesDiscarded++;
				buffInd++;
			}
		}

	}

	// Set readBuff[0] to '\0' ((byte)-1) if !pass
	readBuff[0] = '\0';
	return readBuff[0];
}

// STORE MESSAGE
void StoreMessage(char msg[], uint16_t pack)
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

	// Format string
	sprintf(str, "TEENSY %lu|%d: %s", cnt_logsRcvd, pack, msg);

	// Format log
	sprintf(Log[logInd], "%c%s%c", r42t.head, str, r42t.foot);

	// Create log file
	if (pack == 1) {

		// Check msg contains "LOG"
		if (msg[0] == 'L' && msg[1] == 'O' && msg[2] == 'G') {
			sprintf(logFiStr, "TEENSY%s", msg);
		}
		else {
			sprintf(logFiStr, "TEENSYLOG00000");
		}

		// Create new log file
		OpenNewLog();
	}

	// Shift ind array
	for (size_t i = 0; i < logSize - 1; i++) {
		logIndArr[i] = logIndArr[i + 1];
	}
	logIndArr[logSize - 1] = logInd;

	// Update log ind and count
	logInd++;

	// Check if ind should roll over 
	if (logInd == logSize) {

		// Reset queueIndWrite
		logInd = 0;
	}

	// Write to SD log
	logFileSD.print(str);
	logFileSD.print('\r');
	logFileSD.print('\n');

	// Print message
	if (DO_PRINT_LOGS) {
		PrintDebug(str);
	}
}

// SEND LOGS
void SendLogs()
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	uint32_t t_check = millis() + 100;
	char c_arr[4] = { 0 };

	// Check for low pin
	if (digitalRead(pin.Teensy_SendLogs) != LOW) {
		return;
	}

	// Make sure its low for at least 100 ms
	while (millis() < t_check) {

		// Check for high pin
		if (digitalRead(pin.Teensy_SendLogs) != LOW) {
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
			break;
		}
	}

	// Bail out if start string not found
	if (strcmp(c_arr, "<<<") != 0) {
		return;
	}

	// Log/print
	sprintf(str, "[SendLogs] Begin Sending %d of %lu Logs...",
		cnt_logsStored, cnt_logsRcvd);
	PrintDebug(str);

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
			r42t.port.write(Log[logIndArr[i]]);
			PrintDebug(Log[logIndArr[i]]);
			StatusBlink();
		}

		// Get number of dropped/missed packets
		int cnt_dropped = r42t.packTot - cnt_logsRcvd;

		// Send summary
		sprintf(str, "%cTEENSY SUMMARY \"%s\": sent=%d rcvd=%lu dropped=%d free_mem=%0.2fGB%c",
			r42t.head, logFiStr, cnt_logsStored, cnt_logsRcvd, cnt_dropped, freeMemGB, r42t.foot);
		r42t.port.write(str);
		PrintDebug(str);

	}

	// No logs to send
	else {
		r42t.port.write("TEENSY SUMMARY: NO LOGS STORED");
		PrintDebug(str);
	}

	// End reached send ">>>"
	r42t.port.write(">>>");
	delay(100);

	// Log/print
	sprintf(str, "[SendLogs] Finished Sending %d of %lu Logs",
		cnt_logsStored, cnt_logsRcvd);
	PrintDebug(str);

	// Close log file
	if (logFileSD.isOpen()) {
		logFileSD.close();
	}

	// Reset Stuff
	RunReset();
}

// FORMAT AND PRINT MESSAGE
void PrintDebug(char msg[], uint32_t t)
{

#if DO_PRINT_DEBUG || DO_LOG_DEBUG

	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	char msg_copy[maxStoreStrLng] = { 0 }; msg_copy[0] = '\0';
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
		logFileSD.print(str);
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
	SerialUSB.begin(57600);
#endif 

	// FeederDue Serial
	//r42t.port.begin(57600);
	//r42t.port.begin(115200);
	r42t.port.begin(256000);

	// SETUP PINS

	// Set direction
	pinMode(pin.Teensy_SendLogs, INPUT);
	pinMode(pin.Teensy_Resetting, OUTPUT);
	pinMode(pin.StatLED, OUTPUT);

	// Set initial state
	digitalWrite(pin.StatLED, LOW);
	digitalWrite(pin.Teensy_Resetting, LOW);

	// USE AS DELAY 
	for (size_t i = 0; i < 120; i++)
	{
		StatusBlink();
		delay(25);
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
	SendLogs();

}

