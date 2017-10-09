/*
NOTES
	Changd "C:\Program Files (x86)\Arduino\hardware\teensy\avr\cores\teensy3\serial3"
	SERIAL2_TX_BUFFER_SIZE 40 to 64
	SERIAL2_RX_BUFFER_SIZE 64 to 128
	Have to run from Arduino IDE for changes to take effect
*/

#define DO_DEBUG 0


#pragma region ============ VARIABLE SETUP =============


// Pin mapping
struct PIN
{
	// Status led
	int StatLED = 13;

	// Send log
	int Teensy_SendStart = 4;
	int Teensy_Resetting = 5;
	int Teensy_Unused = 6;
}
// Initialize
pin;


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
	uint16_t cnt_pack;
	int cnt_dropped;
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
};

// Initialize C2R
R42T r42t
{
	// serial
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
	// cnt_pack
	0,
	// cnt_dropped
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
const uint16_t logSize = 20;
uint32_t cnt_logsRcvd = 0;
uint16_t cnt_logsStored = 0;
byte logInd = 0;
byte logIndArr[logSize] = { 0 };
char Log[logSize][maxStoreStrLng] = { { 0 } };
uint16_t cnt_reset = 0;
char logNumStr[100] = { 0 };

// Serial com general
uint32_t t_sync = 0;
uint16_t buffInd = 0;
char readBuff[maxStoreStrLng] = { 0 };
uint32_t cnt_packBytesRead = 0;
uint32_t cnt_packBytesDiscarded = 0;

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

// RESET VARIABLES
void RunReset();

// PARSE SERIAL INPUT
void GetSerial();

// WAIT FOR BUFFER TO FILL
char WaitBuffRead(int timeout, char mtch = '\0');

// STORE MESSAGE
void StoreMessage(char msg[], uint16_t cnt_pack);

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

	// Reset counters
	cnt_logsRcvd = 0;
	cnt_logsStored = 0;
	logInd = 0;
	buffInd = 0;
	readBuff[0] = '\0';
	cnt_packBytesRead = 0;
	cnt_packBytesDiscarded = 0;

	// Reinitialize arrays
	for (size_t i = 0; i < logSize; i++) {
		logIndArr[i] = i;
		Log[i][0] = '\0';
	}

	// Set log entry to null
	sprintf(logNumStr, "%cNULL%c", r42t.head, r42t.foot);

	// Dump buffer
	while (r42t.port.available() > 0) {
		r42t.port.read();
	}

	// Set reset indicator pin high
	sprintf(str, "[RunReset] Setting Reset Pin High");
	PrintDebug(str);
	digitalWrite(pin.StatLED, HIGH);
	digitalWrite(pin.Teensy_Resetting, HIGH);
	// Flicker LED
	for (size_t i = 0; i < 40; i++)
	{
		is_ledOn = !is_ledOn;
		digitalWrite(pin.StatLED, is_ledOn ? HIGH : LOW);
		delay(25);
	}
	// Print finished
	digitalWrite(pin.Teensy_Resetting, LOW);
	sprintf(str, "[RunReset] Setting Reset Pin Low");
	PrintDebug(str);

	// CHECK TX BUFFER SIZE
	PrintDebug("[RunReset] TX BUFFER SHOULD BE 64 Bytes");
	sprintf(str, "[RunReset] TX BUFFER IS %d Bytes", r42t.port.availableForWrite()+1);
	PrintDebug(str);

	// Print status
	sprintf(str, "[RunReset] FINISHED RESET %d", cnt_reset);
	PrintDebug(str);

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
	cnt_packBytesRead = 0;
	cnt_packBytesDiscarded = 0;

	// Bail if no new input
	if (r42t.port.available() == 0 ||
		digitalRead(pin.Teensy_SendStart) == LOW) {
		return;
	}

	// Set status LED high
	StatusBlink();

	// Dump data till msg header byte is reached
	WaitBuffRead(100, r42t.head);
	if (readBuff[buffInd] == 0) {

		// Indicate incomplete message
		if (cnt_logsRcvd > 0) {
			if (DO_DEBUG) {
				sprintf(str, "**WARNING** [GetSerial] Missing Head: b_read=%lu b_dump=%lu",
					cnt_packBytesRead, cnt_packBytesDiscarded);
				PrintDebug(str);
			}
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
			if (DO_DEBUG) {
				sprintf(str, "**WARNING** [GetSerial] Missing Foot: b_read=%lu b_dump=%lu",
					cnt_packBytesRead, cnt_packBytesDiscarded);
				PrintDebug(str);
			}
		}

		// Bail on first read
		else {
			return;
		}

	}

	// Check for dropped packs
	else {

		// Check for missed packets
		int pack_diff = (pack - r42t.cnt_pack);

		// Get number of dropped/missed packets
		int cnt_dropped = pack_diff - 1;

		// Add to total
		if (cnt_dropped > 0 && cnt_logsRcvd > 0) {
			r42t.cnt_dropped += cnt_dropped;
		}

		// Update packs
		r42t.cnt_pack = pack;
	}

	// Terminate string
	readBuff[buffInd] = '\0';

	// Store message
	StoreMessage(readBuff, r42t.cnt_pack);

	// Update counts
	cnt_logsRcvd++;
	cnt_logsStored += cnt_logsStored < logSize ? 1 : 0;

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
			cnt_packBytesRead++;

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
			cnt_packBytesRead++;

			// check match was found
			if (readBuff[buffInd] == mtch) {

				// Add termination and bail
				readBuff[buffInd + 1] = '\0';
				return readBuff[buffInd];
			}

			// Otherwise add to discard count
			else {

				// Itterate counts
				cnt_packBytesDiscarded++;
				buffInd++;
			}
		}

	}

	// Set readBuff[0] to '\0' ((byte)-1) if !pass
	readBuff[0] = '\0';
	return readBuff[0];
}

// STORE MESSAGE
void StoreMessage(char msg[], uint16_t cnt_pack)
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';

	// Format string
	sprintf(str, "TeensyLog %lu: %s", cnt_pack, msg);

	// Store log
	sprintf(Log[logInd], "%c%s%c", r42t.head, str, r42t.foot);

	// Store first log which has log file
	if (cnt_pack == 1) {
		sprintf(logNumStr, Log[logInd]);
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

	// Print message
	PrintDebug(str);
}

// SEND LOGS
void SendLogs()
{
	// Local vars
	static char str[maxStoreStrLng] = { 0 }; str[0] = '\0';
	uint32_t t_check = millis() + 100;
	char c_arr[4] = { 0 };

	// Check for low pin
	if (digitalRead(pin.Teensy_SendStart) != LOW) {
		return;
	}

	// Make sure its low for at least 100 ms
	while (millis() < t_check) {

		// Check for high pin
		if (digitalRead(pin.Teensy_SendStart) != LOW) {
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

	// Send log number first
	r42t.port.write(logNumStr);
	PrintDebug(logNumStr);

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

	// Send summary
	sprintf(str, "%cTEENSY LOG SUMMARY: sent=%d rcvd=%lu dropped=%d%c",
		r42t.head, cnt_logsStored, cnt_logsRcvd, r42t.cnt_dropped, r42t.foot);
	r42t.port.write(str);
	PrintDebug(str);

	// End reached send ">>>"
	r42t.port.write(">>>");
	delay(100);


	// Log/print
	sprintf(str, "[SendLogs] Finished Sending %d of %lu Logs",
		cnt_logsStored, cnt_logsRcvd);
	PrintDebug(str);

	// Reset Stuff
	RunReset();
}

// FORMAT AND PRINT MESSAGE
void PrintDebug(char msg[], uint32_t t)
{
#if DO_DEBUG

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
	sprintf(str, "%s%s%s\n", str_time, spc, msg);

	// Print it
	Serial.print(str);

	// Store message
	//StoreMessage(str, 0);

#endif
}

// RUN STATUS BLINK
void StatusBlink(bool do_force) {

	// Do status blink
	if (do_force ||
		millis() > t_blink + dt_blink) {

		// Flip state and run
		is_ledOn = !is_ledOn;
		digitalWrite(pin.StatLED, is_ledOn ? HIGH : LOW);
		t_blink = millis();
	}

}

#pragma endregion


void setup()
{

	// SET UP SERIAL STUFF

	// Serial monitor
#if DO_DEBUG
	Serial.begin(57600);
#endif 

	// FeederDue Serial
	//r42t.port.begin(57600);
	//r42t.port.begin(115200);
	r42t.port.begin(256000);
	// SETUP PINS

	// Set direction
	pinMode(pin.Teensy_SendStart, INPUT);
	pinMode(pin.StatLED, OUTPUT);
	pinMode(pin.Teensy_Resetting, OUTPUT);

	// Set initial state
	digitalWrite(pin.StatLED, LOW);
	digitalWrite(pin.Teensy_Resetting, LOW);

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

