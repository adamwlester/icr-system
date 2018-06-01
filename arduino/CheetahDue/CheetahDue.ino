//######################################

//========== CheetahDue.ino ============

//######################################


//============= INCLUDE ================

// LOCAL
#include "CheetahDue.h"
//
#include "CheetahDue_PinMap.h"

// GENERAL
#include <stdarg.h>

// MEMORY
#include <MemoryFree.h>

#pragma region ========== CLASS DECLARATIONS ===========


// CLASS: DB
class DEBUG
{
public:

	// VARIABLES
	DB_FLAG flag;
	uint16_t cnt_warn = 0;
	uint16_t cnt_err = 0;
	uint16_t err_cap = 100;
	VEC<uint16_t> warn_line;
	VEC<uint16_t> err_line;
	char buff_lrg_sprintfErr[buffLrg] = { 0 };
	char cnt_sprintfErr = 0;
	char buff_lrg_strcatErr[buffLrg] = { 0 };
	char cnt_strcatErr = 0;

	// METHODS
	DEBUG();
	// CHECK EVERY LOOP
	void CheckLoop();
	// LOG/PRING MAIN EVENT
	void DB_General(const char *p_fun, int line, char *p_msg, uint32_t ts = millis());
	// LOG/PRINT WARNINGS
	void DB_Warning(const char *p_fun, int line, char *p_msg, uint32_t ts = millis());
	// LOG/PRINT ERRORS
	void DB_Error(const char *p_fun, int line, char *p_msg, uint32_t ts = millis());
	// LOG/PRINT RCVD PACKET
	void DB_Rcvd(char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte);
	// LOG/PRINT QUEUED SEND PACKET
	void DB_SendQueued(char *p_msg, uint32_t ts);
	// LOG/PRINT SENT PACKET
	void DB_Sent(char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte);
	// PRINT LOG SEND
	void DB_LogSend(char *p_msg);
	// TEST PIN MAPPING
	void DB_PinMap();
	// STORE STRING FOR PRINTING
	void Queue(char *p_msg, uint32_t ts = millis(), char *p_type = "NOTICE", const char *p_fun = "", int line = 0);
	// PRINT DB INFO
	bool Print();
	// PRINT EVERYTHING IN QUEUE
	bool PrintAll(uint32_t timeout);
	// LOG/PRINT SESSION SUMMARY
	void DoSummary();
	// GET CURRENT NUMBER OF ENTRIES IN PRINT QUEUE
	int GetPrintQueueAvailable();
	// GET CURRENT NUMBER OF ENTRIES IN LOG QUEUE
	int GetLogQueueAvailable();
	// FORMAT INT AS BINARY
	char* FormatBinary(unsigned int int_in);
	// FORMAT TIME STAMP STRING
	char* FormatTimestamp(uint32_t ts);
	// SAFE VERSION OF SPRINTF
	void sprintf_safe(uint16_t buff_cap, char *p_buff, char *p_fmt, ...);
	// SAFE VERSION OF STRCAT
	void strcat_safe(uint16_t buff_cap, uint16_t buff_lng_1, char *p_buff_1, uint16_t buff_lng_2, char *p_buff_2);
	// HOLD FOR CRITICAL ERRORS
	void RunErrorHold(char *p_msg_print, uint32_t dt_shutdown_sec);

};

//INITILIZE OBJECTS

// Initialize DEBUG class instance
DEBUG Debug;

#pragma endregion


#pragma region ========= FUNCTION DECLARATIONS =========

// CHECK FOR HANDSHAKE
bool CheckForHandshake();

// PARSE SERIAL INPUT
void GetSerial();

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char mtch = '\0');

// STORE PACKET DATA TO BE SENT
void QueuePacket(char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf, bool is_conf, bool is_done);

// SEND SERIAL PACKET DATA
bool SendPacket();

// STORE LOG STRING
void QueueLog(char *p_msg, uint32_t ts = millis(), char *p_type = "NOTICE", const char *p_fun = "", int line = 0);

// SEND LOG DATA OVER SERIAL
bool SendLog();

// START REWARD
void StartRew();

// END REWARD
void EndRew();

// HARDWARE TEST
void HardwareTest(int test_num);

// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink();

// PULSE SYNC IR
bool IR_SyncPulse();

// SET IR
bool SetIR(int dt_off, int dt_on, SETIRSTATE force_state = FREE, bool do_ttl = true);

// GET ID INDEX
template <typename A24> int ID_Ind(char id, A24 *p_r24);

// GET/SET BYTE BIT VALUE
bool GetSetByteBit(byte * b_set, int bit, bool do_set);

// GET 32 BIT WORD FOR PORT
uint32_t GetPortWord(uint32_t word, int *p_pin_arr, int arr_size);

// SET PORT
void SetPort(uint32_t word_on, uint32_t word_off);

// RESET PT PINS
void ResetTTL();

// QUIT AND RESTART ARDUINO
void QuitSession();

// NORTH PT
void Interrupt_North();

// WEST PT
void Interrupt_West();

// SOUTH PT
void Interrupt_South();

// EAST PT
void Interrupt_East();

#pragma endregion


#pragma region =========== CLASS DEFINITIONS ===========

#pragma region ----------CLASS: DEBUG----------

DEBUG::DEBUG() :
	warn_line(err_cap, __LINE__),
	err_line(err_cap, __LINE__)
{}

void DEBUG::CheckLoop()
{

	// Local static vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Keep short count of loops
	cnt_loopShort++;

	// Track total loops
	cnt_loopTot++;

	// Check for VEC errors
#if DO_VEC_DEBUG
	// Log each error
	if (VEC_CNT_ERR > 0) {
		for (int i = 0; i < min(VEC_CNT_ERR, VEC_MAX_ERR); i++) {
			Debug.DB_Error(__FUNCTION__, __LINE__, VEC_STR_LIST_ERR[i]);
		}
	}
	// Reset counter
	VEC_CNT_ERR = 0;
#endif

	// Check for SPRINTF_SAFE error
	if (cnt_sprintfErr > 0) {
		// Use standard sprintf()
		sprintf(buff_lrg, "**SPRINTF_SAFE**: cnt=%d buff=\"%s\"",
			cnt_sprintfErr, buff_lrg_sprintfErr);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Reset counter and buff
	buff_lrg_sprintfErr[0] = '\0';
	cnt_sprintfErr = 0;

	// Check for STRCAT_SAFE error
	if (cnt_strcatErr > 0) {
		// Use standard sprintf()
		sprintf(buff_lrg, "**STRCAT_SAFE**: cnt=%d buff=\"%s\"",
			cnt_strcatErr, buff_lrg_strcatErr);
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Reset counter and buff
	buff_lrg_strcatErr[0] = '\0';
	cnt_strcatErr = 0;

}

void DEBUG::DB_General(const char *p_fun, int line, char *p_msg, uint32_t ts)
{

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_general;
	do_log = flag.log_general;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, ts, "NOTICE", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		QueueLog(p_msg, ts, "NOTICE", p_fun, line - 11);
	}

}

void DEBUG::DB_Warning(const char *p_fun, int line, char *p_msg, uint32_t ts)
{

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_errors;
	do_log = flag.log_errors;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, ts, "**WARNING**", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		QueueLog(p_msg, ts, "**WARNING**", p_fun, line - 11);
	}

	// Store warning info
	warn_line[cnt_warn < 100 ? cnt_warn++ : 99] = cnt_logsStored;

}

void DEBUG::DB_Error(const char *p_fun, int line, char *p_msg, uint32_t ts)
{

	// Local vars
	bool do_print = false;
	bool do_log = false;

	// Get print and log flags
	do_print = flag.print_errors;
	do_log = flag.log_errors;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		Queue(p_msg, ts, "!!ERROR!!", p_fun, line - 11);
	}

	// Add to log queue
	if (do_log) {
		QueueLog(p_msg, ts, "!!ERROR!!", p_fun, line - 11);
	}

	// Store error info
	err_line[cnt_err < 100 ? cnt_err++ : 99] = cnt_logsStored;

}

void DEBUG::DB_Rcvd(char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte)
{

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	bool do_print = false;
	bool do_log = false;
	bool is_resend = false;
	bool is_conf = false;

	// Get print status
	do_print = Debug.flag.print_r2a;
	do_log = Debug.flag.log_r2a;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Parse flag
	is_conf = GetSetByteBit(&flag_byte, 1, false);
	is_resend = GetSetByteBit(&flag_byte, 3, false);

	// Format message
	Debug.sprintf_safe(buffMax, buff_max, "   [%sRCVD%s:%s] %s %s",
		is_resend ? "RSND-" : is_repeat ? "RPT-" : "",
		is_conf ? "-CONF" : "",
		"r2c", p_msg_1, p_msg_2);

	// Add to print queue
	if (do_print) {
		Debug.Queue(buff_max, r2a.t_rcvd, "COM");
	}

	// Add to log queue
	if (do_log) {
		QueueLog(buff_max, r2a.t_rcvd, "COM");
	}

}

void DEBUG::DB_SendQueued(char *p_msg, uint32_t ts)
{

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	bool do_print = false;
	bool do_log = false;

	// Get print status
	do_print = Debug.flag.print_a2rQueued;
	do_log = Debug.flag.log_a2rQueued;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Format message
	Debug.sprintf_safe(buffMax, buff_max, "   [SEND-QUEUED:%s] %s", "a2r", p_msg);

	if (do_print) {
		Debug.Queue(buff_max, ts, "COM");
	}

	if (do_log) {
		QueueLog(buff_max, ts, "COM");
	}

}

void DEBUG::DB_Sent(char *p_msg_1, char *p_msg_2, bool is_repeat, byte flag_byte)
{

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	bool do_print = false;
	bool do_log = false;
	bool is_conf = false;

	// Get print status
	do_print = Debug.flag.print_a2r;
	do_log = Debug.flag.log_a2r;

	// Get print status

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Parse flag
	is_conf = GetSetByteBit(&flag_byte, 1, false);

	// Format message
	Debug.sprintf_safe(buffMax, buff_max, "   [%sSENT%s:%s] %s %s",
		is_repeat ? "RPT-" : "",
		is_conf ? "-CONF" : "",
		"a2r", p_msg_1, p_msg_2);

	// Store
	if (do_print) {
		Debug.Queue(buff_max, a2r.t_sent, "COM");
	}

	if (do_log) {
		QueueLog(buff_max, a2r.t_sent, "COM");
	}

}

void DEBUG::DB_LogSend(char *p_msg)
{

	// Local vars
	bool do_print = false;

	// Get print and log flags
	do_print = flag.print_logSend;

	// Bail if not set
	if (!do_print) {
		return;
	}

	// Store
	if (do_print) {
		Debug.Queue(p_msg, millis());
	}
}

void DEBUG::DB_PinMap()
{
	// Bail if flag not set
	if (!Debug.flag.do_printPimMapTest)
		return;

	// Pause
	delay(5000);

	// Print Pin Mapping
	bool p_flow = flag.print_general;
	flag.print_general = true;
	digitalWrite(pin.TTL_NORTH, HIGH); DB_General(__FUNCTION__, __LINE__, "North TTL"); delay(1000); digitalWrite(pin.TTL_NORTH, LOW);
	digitalWrite(pin.TTL_WEST, HIGH); DB_General(__FUNCTION__, __LINE__, "West TTL"); delay(1000); digitalWrite(pin.TTL_WEST, LOW);
	digitalWrite(pin.TTL_SOUTH, HIGH); DB_General(__FUNCTION__, __LINE__, "South TTL"); delay(1000); digitalWrite(pin.TTL_SOUTH, LOW);
	digitalWrite(pin.TTL_EAST, HIGH); DB_General(__FUNCTION__, __LINE__, "East TTL"); delay(1000); digitalWrite(pin.TTL_EAST, LOW);
	digitalWrite(pin.TTL_IR, HIGH); DB_General(__FUNCTION__, __LINE__, "IR Sync TTL"); delay(1000); digitalWrite(pin.TTL_IR, LOW);
	digitalWrite(pin.TTL_WHITE, HIGH); DB_General(__FUNCTION__, __LINE__, "White Noise TTL"); delay(1000); digitalWrite(pin.TTL_WHITE, LOW);
	digitalWrite(pin.TTL_TONE, HIGH); DB_General(__FUNCTION__, __LINE__, "Reward Tone TTL"); delay(1000); digitalWrite(pin.TTL_TONE, LOW);
	digitalWrite(pin.TTL_REW_ON, HIGH); DB_General(__FUNCTION__, __LINE__, "Reward On TTL"); delay(1000); digitalWrite(pin.TTL_REW_ON, LOW);
	digitalWrite(pin.TTL_REW_OFF, HIGH); DB_General(__FUNCTION__, __LINE__, "Reward Off TTL"); delay(1000); digitalWrite(pin.TTL_REW_OFF, LOW);
	digitalWrite(pin.TTL_PID_RUN, HIGH); DB_General(__FUNCTION__, __LINE__, "PID Run TTL"); delay(1000); digitalWrite(pin.TTL_PID_RUN, LOW);
	digitalWrite(pin.TTL_PID_STOP, HIGH); DB_General(__FUNCTION__, __LINE__, "PID Stop TTL"); delay(1000); digitalWrite(pin.TTL_PID_STOP, LOW);
	digitalWrite(pin.TTL_BULL_RUN, HIGH); DB_General(__FUNCTION__, __LINE__, "Bull Run TTL"); delay(1000); digitalWrite(pin.TTL_BULL_RUN, LOW);
	digitalWrite(pin.TTL_BULL_STOP, HIGH); DB_General(__FUNCTION__, __LINE__, "Bull Stop TTL"); delay(1000); digitalWrite(pin.TTL_BULL_STOP, LOW);
	digitalWrite(pin.REL_IR, HIGH); DB_General(__FUNCTION__, __LINE__, "IR Sync Relay"); delay(1000); digitalWrite(pin.REL_IR, LOW);
	digitalWrite(pin.REL_TONE, HIGH); DB_General(__FUNCTION__, __LINE__, "Reward Tone Relay"); delay(1000); digitalWrite(pin.REL_TONE, LOW);
	digitalWrite(pin.REL_WHITE, HIGH); DB_General(__FUNCTION__, __LINE__, "White Noise Relay"); delay(1000); digitalWrite(pin.REL_WHITE, LOW);
	flag.print_general = p_flow;

	// Pause
	delay(5000);

}

void DEBUG::Queue(char *p_msg, uint32_t ts, char *p_type, const char *p_fun, int line)
{
	// Local vars
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_2[buffMed] = { 0 }; buff_med_2[0] = '\0';
	static char buff_med_3[buffMed] = { 0 }; buff_med_3[0] = '\0';
	static char buff_med_4[buffMed] = { 0 }; buff_med_4[0] = '\0';
	static char buff_med_5[buffMed] = { 0 }; buff_med_5[0] = '\0';
	static char buff_med_save[buffMed] = { 0 };
	static uint16_t overflow_cnt = 0;
	uint32_t t_m = 0;

	// Update queue ind
	PQ_StoreInd++;

	// Check if ind should roll over 
	if (PQ_StoreInd == PQ_Capacity) {
		PQ_StoreInd = 0;
	}

	// Handle overflow
	if (PQ_Queue[PQ_StoreInd][0] != '\0') {

		// Add to count 
		overflow_cnt += overflow_cnt < 1000 ? 1 : 0;

		// Update string
		Debug.sprintf_safe(buffMed, buff_med_save, "[*DB_Q-DROP:%d*] ", overflow_cnt);

		// Set queue back
		PQ_StoreInd = PQ_StoreInd - 1 >= 0 ? PQ_StoreInd - 1 : PQ_Capacity - 1;

		// Bail
		return;
	}

	// Get sync correction
	t_m = ts - t_sync;

	// Make time string
	Debug.sprintf_safe(buffMed, buff_med_1, "[%s][%d]", FormatTimestamp(t_m), cnt_loopShort);

	// Add space after time
	Debug.sprintf_safe(buffMed, buff_med_2, "%%%ds", 20 - strlen(buff_med_1));
	Debug.sprintf_safe(buffMed, buff_med_3, buff_med_2, '_');

	// Add type
	if (strcmp(p_type, "!!ERROR!!") == 0 || strcmp(p_type, "**WARNING**") == 0) {
		Debug.sprintf_safe(buffMed, buff_med_4, "%s ", p_type);
	}

	// Add function and line number
	if (strcmp(p_fun, "") != 0) {
		Debug.sprintf_safe(buffMed, buff_med_5, "[%s:%d] ", p_fun, line);
	}

	// Put it all together
	Debug.sprintf_safe(buffMax, PQ_Queue[PQ_StoreInd], "%s%s%s%s%s%s\n", buff_med_1, buff_med_3, buff_med_save, buff_med_4, buff_med_5, p_msg);

	// Reset overlow stuff
	overflow_cnt = 0;
	buff_med_save[0] = '\0';

}

bool DEBUG::Print()
{

	// Bail if nothing in queue
	if (PQ_ReadInd == PQ_StoreInd &&
		PQ_Queue[PQ_StoreInd][0] == '\0') {
		return false;
	}

	// Increment send ind
	PQ_ReadInd++;

	// Check if ind should roll over 
	if (PQ_ReadInd == PQ_Capacity) {
		PQ_ReadInd = 0;
	}

	// Print
	SerialUSB.print(PQ_Queue[PQ_ReadInd]);

	// Set entry to null
	PQ_Queue[PQ_ReadInd][0] = '\0';

	// Return success
	return true;
}

bool DEBUG::PrintAll(uint32_t timeout)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	uint16_t cnt_print = 0;
	int queue_size = 0;
	uint32_t t_timeout = millis() + timeout;
	bool is_timedout = false;

	// Loop till done or timeout reached
	while (Debug.Print()) {

		// Incriment counter
		cnt_print++;

		// Check for timeout
		if (millis() > t_timeout) {
			is_timedout = true;
			break;
		}

	}

	// Get queue left
	queue_size = PQ_Capacity - Debug.GetPrintQueueAvailable();

	// Check for timeout
	if (is_timedout) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: cnt_print%d queued=%d dt_run=%d",
			cnt_print, queue_size, timeout);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Return all printed flag
	return queue_size == 0;

}

void DEBUG::DoSummary()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	uint16_t xbee_tx_size = 0;
	uint16_t xbee_rx_size = 0;
	uint16_t log_tx_size = 0;
	uint16_t log_rx_size = 0;

	// Update buffers 
	xbee_tx_size = SERIAL_BUFFER_SIZE - 1 - r2a.hwSerial.availableForWrite();
	xbee_rx_size = r2a.hwSerial.available();
	log_tx_size = SERIAL_BUFFER_SIZE - 1 - a2c.hwSerial.availableForWrite();
	log_rx_size = a2c.hwSerial.available();

	// Com summary info Genaeral
	Debug.sprintf_safe(buffLrg, buff_lrg, "COM SUMMARY GENERAL: RX=|flood=%lu|tout=%lu|xbeebuff=%d|logbuff=%d| TX=|xbeebuff=%d|logbuff=%d|",
		cnt_overflowRX, cnt_timeoutRX, xbee_rx_size, log_rx_size, xbee_tx_size, log_tx_size);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Com summary info FeederDue
	Debug.sprintf_safe(buffLrg, buff_lrg, "COM SUMMARY FEEDERDUE: A2R=|pind=%d|psent=%lu|resnd=%lu| R2A=|pind=%d|psent=%d|prcvd=%lu|rercv=%lu|drop=%lu|",
		a2r.packInd, a2r.packSentAll, a2r.cnt_repeat,
		r2a.packInd, r2a.packSentAll, r2a.packRcvdAll, r2a.cnt_repeat, r2a.cnt_dropped);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Warnings summary
	Debug.sprintf_safe(buffLrg, buff_lrg, "ON LINES |");
	for (int i = 0; i < cnt_warn; i++) {
		Debug.sprintf_safe(buffMed, buff_med, "%d|", warn_line[i]);
		Debug.strcat_safe(buffLrg, strlen(buff_lrg), buff_lrg, strlen(buff_med), buff_med);
	}
	Debug.sprintf_safe(buffLrg, buff_lrg, "TOTAL WARNINGS: %d %s", cnt_warn, cnt_warn > 0 ? buff_lrg : "");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Errors summary
	Debug.sprintf_safe(buffLrg, buff_lrg, "ON LINES |");
	for (int i = 0; i < cnt_err; i++) {
		Debug.sprintf_safe(buffMed, buff_med, "%d|", err_line[i]);
		Debug.strcat_safe(buffLrg, strlen(buff_lrg), buff_lrg, strlen(buff_med), buff_med);
	}
	Debug.sprintf_safe(buffLrg, buff_lrg, "TOTAL ERRORS:  %d %s", cnt_err, cnt_err > 0 ? buff_lrg : "");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

}

int DEBUG::GetPrintQueueAvailable() {

	// Local vars
	int n_entries = 0;

	// Check print queue
	for (int i = 0; i < PQ_Capacity; i++) {
		n_entries += PQ_Queue[i][0] != '\0' ? 1 : 0;
	}

	// Get total available
	return PQ_Capacity - n_entries;
}

int DEBUG::GetLogQueueAvailable() {

	// Local vars
	int n_entries = 0;

	// Check log queue
	for (int i = 0; i < LQ_Capacity; i++) {
		n_entries += LQ_Queue[i][0] != '\0' ? 1 : 0;
	}

	// Get total available
	return LQ_Capacity - n_entries;

}

char* DEBUG::FormatBinary(unsigned int int_in)
{
	static char bit_str[100]; bit_str[0] = '\0';
	UNION_SERIAL U;
	byte bit_ind = 0;

	U.i32 = int_in;

	bool do_write = false;
	for (int i = 3; i >= 0; i--)
	{
		do_write = do_write || U.b[i] > 0;
		if (!do_write) {
			continue;
		}

		for (int j = 7; j >= 0; j--) {
			bit_str[bit_ind++] = ((U.b[i] >> j) & 0x01) == 1 ? '1' : '0';
		}

		if (i > 0) {
			bit_str[bit_ind++] = ' ';
		}
	}
	bit_str[bit_ind++] = '\0';
	//SerialUSB.println(bit_str);
	//SerialUSB.print('\n');

	return bit_str;
}

char* DEBUG::FormatTimestamp(uint32_t ts)
{

	// Local vars
	static char buff_med[buffMed] = { 0 }; buff_med[0] = '\0';
	int ts_m = 0;
	int s = 0;
	int ts_s = 0;
	int ts_ms = 0;

	// Get minutes
	ts_m = (ts - (ts % (60 * 1000))) / (60 * 1000);

	// Get seconds
	s = ts - (ts_m * 60 * 1000);
	ts_s = (s - (s % 1000)) / 1000;

	// Get milliseconds
	ts_ms = ts - (ts_m * 60 * 1000) - (ts_s * 1000);

	// Format string
	Debug.sprintf_safe(buffMed, buff_med, "%02u:%02u::%03u", ts_m, ts_s, ts_ms);
	return buff_med;

}

void DEBUG::sprintf_safe(uint16_t buff_cap, char *p_buff, char *p_fmt, ...) {

	// Local vars
	static const uint16_t buff_size = buffMax * 2;
	static char buff[buff_size]; buff[0] = '\0';
	const char str_prfx_med[buffMed] = "*T*";
	int cut_ind = 0;

	// Reset output buffer
	p_buff[0] = '\0';

	// Hide Due specific intellisense errors
#ifndef __INTELLISENSE__

	// Get variable length input arguments
	va_list args;

	// Begin retrieving additional arguments
	va_start(args, p_fmt);

	// Use vsnprintf to format string from argument list
	vsnprintf(buff, buff_size, p_fmt, args);

	// End retrval
	va_end(args);

#endif

	// Formated string too long
	if (strlen(buff) + 1 > buff_cap) {

		// Store part of buff for error
		int err_str_len = min(50, buff_cap);
		for (int i = 0; i < err_str_len; i++)
		{
			buff_lrg_sprintfErr[i] = buff[i];
		}
		buff_lrg_sprintfErr[err_str_len] = '\0';
		cnt_sprintfErr++;

		// Get cut ind and truncate buff
		cut_ind = buff_cap - strlen(str_prfx_med);

		// Check for negative cut ind
		if (cut_ind > 0) {
			// Cut buff
			buff[cut_ind] = '\0';
		}
		else {
			// Clear buff
			buff[0] = '\0';
		}

		// Append error prefix
		strcat(p_buff, str_prfx_med);


	}

	// Copy/concat buff to p_buff
	strcat(p_buff, buff);

}

void DEBUG::strcat_safe(uint16_t buff_cap, uint16_t buff_lng_1, char *p_buff_1, uint16_t buff_lng_2, char *p_buff_2)
{
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	const char str_prfx_med[buffMed] = "*T*";
	int cut_ind = 0;

	// Make sure buffer large enough
	if (buff_cap > buff_lng_1 + buff_lng_2) {

		// Concatinate strings
		strcat(p_buff_1, p_buff_2);
	}

	// String too long
	else {

		// Store part of buff for error
		int err_str_len = min(50, buff_lng_1);
		for (int i = 0; i < err_str_len; i++)
		{
			buff_lrg_strcatErr[i] = p_buff_1[i];
		}
		buff_lrg_strcatErr[err_str_len] = '\0';
		cnt_strcatErr++;

		// Insert prefix at front of buffer 1
		for (int i = 0; i < strlen(str_prfx_med); i++)
		{
			p_buff_1[i] = str_prfx_med[i];
		}

	}

}

void DEBUG::RunErrorHold(char *p_msg_print, uint32_t dt_shutdown_sec)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static uint32_t t_shutdown = dt_shutdown_sec > 0 ? millis() + (dt_shutdown_sec * 1000) : 0;
	bool do_led_on = true;
	int dt_cycle = 100;

	// Log error message
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUN ERROR HOLD FOR %d sec: %s", dt_shutdown_sec / 1000, p_msg_print);
	DB_Error(__FUNCTION__, __LINE__, buff_lrg);

	// Loop till shutdown
	while (true)
	{
		// Log anything in queue
		SendLog();
		Debug.Print();

		// Flicker lights
		if (do_led_on) {
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_ON, false);
		}
		else {
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_OFF, false);
		}
		do_led_on = !do_led_on;

		// Pause
		delay(dt_cycle);


		// Check if should shutdown
		if (t_shutdown > 0 && millis() > t_shutdown) {

			// Quit if still powered by USB
			QuitSession();
		}

	}

}

#pragma endregion

#pragma endregion


#pragma region ========== FUNCTION DEFINITIONS =========

#pragma region --------COMMUNICATION---------

// CHECK FOR HANDSHAKE
bool CheckForHandshake()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static uint32_t t_timeout = 0;
	VEC<byte> in_byte(1, __LINE__);
	uint32_t ir_start = 0;

	// Bail if session started
	if (FC.is_SesStarted) {
		return true;
	}

	// Bypass handshake
	if (Debug.flag.do_handshakeBypass) {
		in_byte[0] = c2a.id[0];
	}

	// Check for CS handshake
	if (!FC.is_CSHandshakeDone) {

		// Check if IR should be pulsed 
		if (digitalRead(pin.SWITCH_BLINK) == LOW || is_irOn)
		{
			// Update without triggering ttl
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FREE, false);
		}

		// Get new data
		if (c2a.hwSerial.available() > 0) {
			in_byte[0] = c2a.hwSerial.read();
		}

		// Bail if not a match
		if (in_byte[0] != c2a.id[0]) {
			return false;
		}

		// Turn off ir
		SetIR(0, 0, FORCE_OFF);
		delay(1000);

		// Pulse IR on
		ir_start = millis();
		SetIR(0, 0, FORCE_ON);
		delayMicroseconds(dt_irSyncPulseOn * 1000);

		// Turn IR off
		SetIR(0, 0, FORCE_OFF);
		delayMicroseconds((dt_irHandshakePulse - dt_irSyncPulseOn) * 1000);

		// Turn IR back on
		SetIR(0, 0, FORCE_ON);

		// Store most resent pulse time
		t_sync = t_irSyncLast;

		// Log pulse time
		Debug.sprintf_safe(buffLrg, buff_lrg, "%dms IR SYNC PULSE SENT", t_sync - ir_start);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		// Turn IR back off
		delayMicroseconds(dt_irSyncPulseOn * 1000);
		SetIR(0, 0, FORCE_OFF);

		// Dump CS buffer
		while (c2a.hwSerial.available()) {
			c2a.hwSerial.read();
		}

		// Set flag
		FC.is_CSHandshakeDone = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "CS Handshake Confirmed");

		// Set abort time
		t_timeout = millis() + dt_timeoutHandshake;

		// Send handshake message that can be used in place of IR pulse
		if (DO_SKIP_IR_SYNC)
		{
			QueuePacket('h', 1, 0, 0, 0, true, false, false);
		}

	}

	// Check for handshake confirmation from FeederDue
	if (!FC.is_FeederDueHandshakeDone && r2a.idNew == 'h' && r2a.is_new)
	{
		// Set flag
		FC.is_FeederDueHandshakeDone = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "FeederDue Handshake Confirmed");

	}

	// Check if handshake complete
	if (FC.is_CSHandshakeDone && FC.is_FeederDueHandshakeDone) {

		// Log sync time
		Debug.sprintf_safe(buffLrg, buff_lrg, "SET SYNC TIME: %dms", t_sync);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg, t_sync);

		// Enable PT interupts
		v_doPTInterupt = true;

		// Dump Xbee buffer
		while (r2a.hwSerial.available() > 0) {
			r2a.hwSerial.read();
		}

		// Set flag
		FC.is_SesStarted = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "HANDSHAKE COMPLETE");
	}

	// Check for handshake timeout
	else if (t_timeout > 0 && millis() > t_timeout) {

		// Format message
		Debug.sprintf_safe(buffLrg, buff_lrg, "HANDSHAKE TIMEDOUT AFTER %d ms: |%s%s",
			dt_timeoutHandshake,
			!FC.is_CSHandshakeDone ? "NO CS HANDSHAKE|" : "",
			!FC.is_FeederDueHandshakeDone ? "NO FEEDERDUE HANDSHAKE|" : "");

		// Run error hold and restart
		Debug.RunErrorHold(buff_lrg, 15);
	}

	// Handshake not complete
	else {
		return false;
	}

}

// PARSE SERIAL INPUT
void GetSerial()
{
	/*
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static char buff_lrg_3[buffLrg] = { 0 }; buff_lrg_3[0] = '\0';
	static char buff_lrg_4[buffLrg] = { 0 }; buff_lrg_4[0] = '\0';
	uint32_t t_start = millis();
	int dt_parse = 0;
	int dt_sent = 0;
	uint16_t tx_size = 0;
	uint16_t rx_size = 0;
	byte b = 0;
	char head = ' ';
	char id = ' ';
	VEC<float> dat(3, __LINE__);
	char foot = ' ';
	bool do_conf = false;
	bool is_conf = false;
	bool is_resend = false;
	bool is_repeat = false;
	bool is_missed = false;
	bool is_dropped = false;
	int n_missed = 0;
	byte flag_byte = 0;
	uint16_t pack = 0;
	int id_ind = ' ';

	// Reset vars
	cnt_bytesRead = 0;
	cnt_bytesDiscarded = 0;
	r2a.is_new = false;
	r2a.idNew = ' ';

	// Bail if no new input
	if (r2a.hwSerial.available() == 0) {
		return;
	}

	// Dump data till p_msg header byte is reached
	b = WaitBuffRead(r2a.head);
	if (b == 0) {
		return;
	}

	// Store header
	head = b;

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

	// Get confirmation flag
	U.f = 0.0f;
	U.b[0] = WaitBuffRead();
	flag_byte = U.b[0];
	do_conf = GetSetByteBit(&flag_byte, 0, false);
	is_conf = GetSetByteBit(&flag_byte, 1, false);
	is_resend = GetSetByteBit(&flag_byte, 3, false);

	// Get footer
	foot = WaitBuffRead();

	// Get total data in buffers
	rx_size = r2a.hwSerial.available();
	tx_size = SERIAL_BUFFER_SIZE - 1 - r2a.hwSerial.availableForWrite();

	// Compute parce and sent dt
	dt_parse = millis() - t_start;
	dt_sent = a2r.t_sent > 0 ? millis() - a2r.t_sent : 0;

	// Check for missing footer
	if (foot != r2a.foot) {

		// Increment dropped count
		r2a.cnt_dropped++;

		// Set flag
		is_dropped = true;

	}
	else
	{

		// Send confirmation
		if (do_conf) {
			QueuePacket(id, dat[0], dat[1], dat[2], pack, false, true, false);
		}

		// Get id ind
		id_ind = ID_Ind<A4_COM<USARTClass>>(id, &r2a);

		// Update times
		r2a.dt_rcvd = r2a.t_rcvd > 0 ? millis() - r2a.t_rcvd : 0;
		r2a.t_rcvd = millis();

		// Get last pack
		uint16_t pack_last;
		if (!is_conf)
			pack_last = r2a.packArr[id_ind];
		else
			pack_last = r2a.packConfArr[id_ind];

		// Flag resent pack
		is_repeat = pack == pack_last;

		// Incriment repeat count
		r2a.cnt_repeat  += is_repeat || is_resend ? 1 : 0;

		// Update packet history
		if (!is_conf)
			r2a.packArr[id_ind] = pack;
		else
			r2a.packConfArr[id_ind] = pack;

		// Update data
		r2a.dat[0] = dat[0];
		r2a.dat[1] = dat[1];
		r2a.dat[2] = dat[2];

		// Update for new packets
		if (!is_conf && !is_repeat) {

			// Get pack diff accounting for packet rollover
			int pack_diff =
				abs(pack - r2a.packInd) < (r2a.packRange[1] - r2a.packRange[0]) ?
				pack - r2a.packInd :
				pack - (r2a.packRange[0] - 1);

			// Update dropped packets
			n_missed = (pack - r2a.packInd) - 1;
			r2a.cnt_dropped += n_missed > 0 ? n_missed : 0;

			// flag missed packets
			if (n_missed > 0) {
				is_missed = true;
			}

			// Update packets sent
			r2a.packSentAll += pack_diff;

			// Update packet ind 
			r2a.packInd = pack;

			// Increment packets recieved
			r2a.packRcvdAll++;

			// Update new message info
			r2a.idNew = id;
			r2a.is_new = true;

		}

	}

	// Store data strings
	Debug.sprintf_safe(buffLrg, buff_lrg_2, "\'%c\': dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s",
		id, dat[0], dat[1], dat[2], pack, Debug.FormatBinary(flag_byte));
	Debug.sprintf_safe(buffLrg, buff_lrg_3, "b_read=%d b_dump=%d rx=%d tx=%d dt(snd|rcv|prs)=|%d|%d|%d|",
		cnt_bytesRead, cnt_bytesDiscarded, rx_size, tx_size, dt_sent, r2a.dt_rcvd, dt_parse);

	// Log recieved
	if (!is_dropped) {

		// Log as warning if re-revieving packet
		if (is_repeat || is_resend) {
			Debug.sprintf_safe(buffLrg, buff_lrg_4, "Recieved %s r2a Packet: %s %s",
				is_repeat ? "Duplicate" : is_resend ? "Resent" : "", buff_lrg_2, buff_lrg_3);
			Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg_4, r2a.t_rcvd);
		}

		// Log recieved
		Debug.DB_Rcvd(buff_lrg_2, buff_lrg_3, is_repeat, flag_byte);
	}
	// Log dropped packs warning
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Dropped r2a Packs: cnt=%lu %s %s",
			r2a.cnt_dropped, buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log missed packs warning
	if (is_missed) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Missed r2a Packs: (cns|tot)=|%d|%lu| p_last=%d %s %s",
			n_missed, r2a.cnt_dropped, r2a.packInd, buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log discarded bytes warning
	if (cnt_bytesDiscarded > 0) {

		// Log discarded data
		Debug.sprintf_safe(buffLrg, buff_lrg, "Dumped Bytes: %s %s", buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	// Log parse hanging warning
	if (dt_parse > 30) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "Parser Hanging: %s %s", buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(char mtch)
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	bool is_bytes_discarded = false;
	bool is_overflowed = false;
	byte b = 0;

	// Get total data in buffers now
	uint16_t rx_size_start = r2a.hwSerial.available();

	// Check for overflow
	is_overflowed = rx_size_start >= SERIAL_BUFFER_SIZE - 1;

	// Wait for at least 1 byte
	while (r2a.hwSerial.available() < 1 &&
		millis() < t_timeout);

	// Get any byte
	if (!is_overflowed &&
		mtch == '\0') {

		if (r2a.hwSerial.available() > 0) {

			b = r2a.hwSerial.read();
			cnt_bytesRead++;
			return b;
		}
	}

	// Find specific byte
	while (
		b != mtch  &&
		millis() < t_timeout &&
		!is_overflowed) {

		// Check new data
		if (r2a.hwSerial.available() > 0) {

			b = r2a.hwSerial.read();
			cnt_bytesRead++;

			// check match was found
			if (b == mtch) {
				return b;
			}

			// Otherwise add to discard count
			else {
				cnt_bytesDiscarded++;
				is_bytes_discarded = true;
			}

			// Check for overflow
			is_overflowed =
				!is_overflowed ? r2a.hwSerial.available() >= SERIAL_BUFFER_SIZE - 1 : is_overflowed;
		}

	}

	// Check if buffer flooded
	if (is_overflowed) {

		// DUMP IT ALL
		while (r2a.hwSerial.available() > 0) {
			if (r2a.hwSerial.available() > 0) {
				r2a.hwSerial.read();
				cnt_bytesRead++;
				cnt_bytesDiscarded++;
			}
		}
	}

	// Get buffer 
	uint16_t tx_size = SERIAL_BUFFER_SIZE - 1 - r2a.hwSerial.availableForWrite();
	uint16_t rx_size = r2a.hwSerial.available();

	// Store current info
	char buff_print = b == 10 ? 'n' : b == 13 ? 'r' : b;
	Debug.sprintf_safe(buffLrg, buff_lrg_2, "buff_lrg=%c b_read=%d b_dump=%d rx_start=%d rx_now=%d tx_now=%d dt_chk=%d",
		buff_print, cnt_bytesRead, cnt_bytesDiscarded, rx_size_start, rx_size, tx_size, (millis() - t_timeout) + timeout);


	// Buffer flooded
	if (is_overflowed) {
		cnt_overflowRX++;
		Debug.sprintf_safe(buffLrg, buff_lrg, "BUFFER OVERFLOWED: cnt=%d %s", cnt_overflowRX, buff_lrg_2);
	}
	// Timed out
	else if (millis() > t_timeout) {
		cnt_timeoutRX++;
		Debug.sprintf_safe(buffLrg, buff_lrg, "TIMEDOUT: cnt=%d %s", cnt_timeoutRX, buff_lrg_2);
	}
	// Byte not found
	else if (mtch != '\0') {
		Debug.sprintf_safe(buffLrg, buff_lrg, "CHAR \'%c\' NOT FOUND: %s", mtch, buff_lrg_2);
	}
	// Failed for unknown reason
	else {
		Debug.sprintf_safe(buffLrg, buff_lrg, "FAILED FOR UNKNOWN REASON: %s", buff_lrg_2);
	}

	// Log error
	Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

	// Set buff_lrg to '\0' ((byte)-1) if !pass
	return 0;

}

// STORE PACKET DATA TO BE SENT
void QueuePacket(char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf, bool is_conf, bool is_done)
{
	/*
	STORE DATA FOR ROBOT
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	float _dat[3] = { dat1 , dat2 , dat3 };
	VEC<float> dat(3, __LINE__, _dat);
	byte flag_byte = 0;
	int id_ind = 0;
	uint16_t tx_size = 0;
	uint16_t rx_size = 0;
	
	// Set flag_byte
	GetSetByteBit(&flag_byte, 0, do_conf);
	GetSetByteBit(&flag_byte, 1, is_conf);
	GetSetByteBit(&flag_byte, 2, is_done);

	// Get buffers
	tx_size = SERIAL_BUFFER_SIZE - 1 - a2r.hwSerial.availableForWrite();
	rx_size = a2r.hwSerial.available();

	// Update a2r.QUEUE ind
	a2r.SQ_StoreInd++;

	// Check if ind should roll over 
	if (a2r.SQ_StoreInd == SQ_Capacity) {
		a2r.SQ_StoreInd = 0;
	}

	// Check if overfloweed
	if (a2r.SQ_Queue[a2r.SQ_StoreInd][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[SQ_Capacity + 1];
		for (int i = 0; i < SQ_Capacity; i++) {
			queue_state[i] = a2r.SQ_Queue[i][0] == '\0' ? '0' : '1';
		}
		queue_state[SQ_Capacity] = '\0';

		// Format queue overflow error string
		Debug.sprintf_safe(buffLrg, buff_lrg, "[*%s_Q-DROP*]: queue_s=%d queue_r=%d queue_state=|%s|",
			"a2r", a2r.SQ_StoreInd, a2r.SQ_ReadInd, queue_state);

		// Log error
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);

		// Set queue back 
		a2r.SQ_StoreInd = a2r.SQ_StoreInd - 1 >= 0 ? a2r.SQ_StoreInd - 1 : SQ_Capacity - 1;

		// Bail
		return;

	}

	// Increment packet ind if originating cheetah due
	if (pack == 0) {

		// Increment packet
		a2r.packInd++;

		// Reset packet if out of range
		if (a2r.packInd > a2r.packRange[1])
		{
			Debug.sprintf_safe(buffLrg, buff_lrg, "RESETTING A2R PACKET INDEX FROM %d TO %d", a2r.packInd, a2r.packRange[0]);
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Set to lowest range value
			a2r.packInd = a2r.packRange[0];
		}

		// Copy packet number
		pack = a2r.packInd;
	}

	// Create byte packet
	int b_ind = 0;
	// Store header
	a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = a2r.head;
	// Store mesage id
	a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = id;
	// Store mesage data 
	for (int i = 0; i < 3; i++)
	{
		U.f = dat[i];
		a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = U.b[0];
		a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = U.b[1];
		a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = U.b[2];
		a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = U.b[3];
	}
	// Store packet number
	U.f = 0.0f;
	U.i16[0] = pack;
	a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = U.b[0];
	a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = U.b[1];
	// Store confirm type flag
	a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = flag_byte;
	// Store footer
	a2r.SQ_Queue[a2r.SQ_StoreInd][b_ind++] = a2r.foot;

	// Store time
	id_ind = ID_Ind<A2_COM<USARTClass>>(id, &a2r);
	a2r.t_queuedArr[id_ind] = millis();

	// Format data string
	Debug.sprintf_safe(buffLrg, buff_lrg, "\'%c\': dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s",
		id, dat[0], dat[1], dat[2], pack, Debug.FormatBinary(flag_byte));

	// Log sent
	Debug.DB_SendQueued(buff_lrg, a2r.t_queuedArr[id_ind]);

}

// SEND SERIAL PACKET DATA
bool SendPacket()
{
	/*
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]flag_byte, [8]footer
	*/

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_lrg_2[buffLrg] = { 0 }; buff_lrg_2[0] = '\0';
	static char buff_lrg_3[buffLrg] = { 0 }; buff_lrg_3[0] = '\0';
	static char buff_lrg_4[buffLrg] = { 0 }; buff_lrg_4[0] = '\0';
	int dt_rcvd = 0;
	int dt_queue = 0;
	char id = '\0';
	VEC<float> dat(3, __LINE__);
	bool do_conf = false;
	bool is_conf = false;
	bool is_repeat = false;
	byte flag_byte = 0;
	uint16_t pack = 0;
	uint16_t pack_last = 0;
	uint16_t tx_size = 0;
	uint16_t rx_size = 0;
	int id_ind = 0;

	// Bail if nothing in queue
	if (a2r.SQ_ReadInd == a2r.SQ_StoreInd &&
		a2r.SQ_Queue[a2r.SQ_StoreInd][0] == '\0') {
		return false;
	}

	// Get buffer 
	tx_size = SERIAL_BUFFER_SIZE - 1 - a2r.hwSerial.availableForWrite();
	rx_size = a2r.hwSerial.available();

	// Bail if buffer or time inadequate
	if (tx_size > 0 ||
		rx_size > 0 ||
		millis() < a2r.t_sent + a2r.dt_minSentRcvd) {

		// Indicate still packs to send
		return true;
	}

	// Add small delay if just recieved
	else if (millis() < r2a.t_rcvd + r2a.dt_minSentRcvd) {
		delayMicroseconds(500);
	}

	// Increment send ind
	a2r.SQ_ReadInd++;

	// Check if ind should roll over 
	if (a2r.SQ_ReadInd == SQ_Capacity) {
		a2r.SQ_ReadInd = 0;
	}

	// Send
	a2r.hwSerial.write(a2r.SQ_Queue[a2r.SQ_ReadInd], SQ_MsgBytes);

	// Update dt stuff
	a2r.dt_sent = a2r.t_sent > 0 ? millis() - a2r.t_sent : 0;
	a2r.t_sent = millis();
	dt_rcvd = r2a.t_rcvd > 0 ? millis() - r2a.t_rcvd : 0;
	dt_queue = millis() - a2r.t_queuedArr[id_ind];

	// Get buffers
	tx_size = SERIAL_BUFFER_SIZE - 1 - a2r.hwSerial.availableForWrite();
	rx_size = a2r.hwSerial.available();

	// pull out packet data
	int b_ind = 1;
	// id
	id = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
	// dat
	for (int i = 0; i < 3; i++)
	{
		U.f = 0;
		U.b[0] = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
		U.b[1] = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
		U.b[2] = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
		U.b[3] = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
		dat[i] = U.f;
	}
	// pack
	U.f = 0.0f;
	U.b[0] = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
	U.b[1] = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
	pack = U.i16[0];
	// conf flag
	flag_byte = a2r.SQ_Queue[a2r.SQ_ReadInd][b_ind++];
	do_conf = GetSetByteBit(&flag_byte, 0, false);
	is_conf = GetSetByteBit(&flag_byte, 1, false);

	// Set entry to null
	a2r.SQ_Queue[a2r.SQ_ReadInd][0] = '\0';

	// Get id ind
	id_ind = ID_Ind<A2_COM<USARTClass>>(id, &a2r);

	// Get last pack
	pack_last = is_conf ? r2a.packConfArr[id_ind] : r2a.packArr[id_ind];

	// Flag resent pack
	is_repeat = pack == pack_last;

	// Incriment repeat count
	r2a.cnt_repeat += is_repeat ? 1: 0;

	// Incriment sent packet count
	a2r.packSentAll++;

	// Update packet history
	if (!is_conf)
		r2a.packArr[id_ind] = pack;
	else
		r2a.packConfArr[id_ind] = pack;

	// Format data string
	Debug.sprintf_safe(buffLrg, buff_lrg_2, "\'%c\': dat=|%0.2f|%0.2f|%0.2f| pack=%d flag_byte=%s",
		id, dat[0], dat[1], dat[2], pack, Debug.FormatBinary(flag_byte));
	Debug.sprintf_safe(buffLrg, buff_lrg_3, "b_sent=%d tx=%d rx=%d dt(snd|rcv|q)=|%d|%d|%d|",
		SQ_MsgBytes, tx_size, rx_size, a2r.dt_sent, dt_rcvd, dt_queue);

	// Log warning
	if (is_repeat) {
		Debug.sprintf_safe(buffLrg, buff_lrg_4, "Sent Duplicate a2r Packet: %s %s", buff_lrg_2, buff_lrg_3);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg_4, a2r.t_sent);
	}

	// Log sent
	Debug.DB_Sent(buff_lrg_2, buff_lrg_3, is_repeat, flag_byte);

	// Return success
	return true;

}

// STORE LOG STRING
void QueueLog(char *p_msg, uint32_t ts, char *p_type, const char *p_fun, int line)
{

	// Local vars
	static char buff_max[buffMax] = { 0 }; buff_max[0] = '\0';
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';
	static char buff_med_1[buffMed] = { 0 }; buff_med_1[0] = '\0';
	static char buff_med_save[buffMed] = { 0 };
	static uint16_t overflow_cnt = 0;
	uint32_t t_m = 0;
	byte chksum = 0;

	// Update queue ind
	LQ_StoreInd++;

	// Check if ind should roll over 
	if (LQ_StoreInd == LQ_Capacity) {
		LQ_StoreInd = 0;
	}

	// Handle overflow
	if (LQ_Queue[LQ_StoreInd][0] != '\0') {

		// Add to count 
		overflow_cnt += overflow_cnt < 1000 ? 1 : 0;

		// Update string
		Debug.sprintf_safe(buffMed, buff_med_save, "[*LOG_Q-DROP:%d*] ", overflow_cnt);

		// Set queue back
		LQ_StoreInd = LQ_StoreInd - 1 >= 0 ? LQ_StoreInd - 1 : LQ_Capacity - 1;

		// Bail
		return;
	}

	// Incriment count
	cnt_logsStored++;

	// Get sync correction
	t_m = ts - t_sync;

	// Add function and line number
	if (strcmp(p_fun, "") != 0) {
		Debug.sprintf_safe(buffMed, buff_med_1, "[%s:%d] ", p_fun, line);
	}

	// Store log
	Debug.sprintf_safe(buffMax, buff_max, "%d,%lu,=\"%s\",%d,%s,%s%s%s",
		cnt_logsStored, t_m, Debug.FormatTimestamp(t_m), cnt_loopShort, p_type, buff_med_save, buff_med_1, p_msg);

	// Get message size
	chksum = strlen(buff_max);

	// Add header, chksum and footer
	int b_ind = 0;
	// head
	buff_lrg[b_ind++] = a2c.head;
	// checksum
	buff_lrg[b_ind++] = chksum;
	// p_msg
	for (int i = 0; i < chksum; i++) {
		buff_lrg[b_ind++] = buff_max[i];
	}
	// foot
	buff_lrg[b_ind++] = a2c.foot;
	// null
	buff_lrg[b_ind] = '\0';

	// Store log
	Debug.sprintf_safe(buffMax, LQ_Queue[LQ_StoreInd], "%s", buff_lrg);

	// Reset overlow stuff
	overflow_cnt = 0;
	buff_med_save[0] = '\0';

	// Reset overlow stuff
	overflow_cnt = 0;
	buff_med_save[0] = '\0';

}

// SEND LOG DATA OVER SERIAL
bool SendLog()
{
	/*
	STORE LOG DATA FOR CS
	FORMAT: head,chksum,"[log_cnt],loop,ts_ms,message",foot
	*/
	// Local vars
	static char buff_lrg[buffMax + 100] = { 0 }; buff_lrg[0] = '\0';
	int msg_lng = 0;
	cnt_logBytesSent = 0;
	int xbee_tx_size;
	int xbee_rx_size;
	int log_tx_size;
	int log_rx_size;

	// Bail if serial not established or no logs to store
	if (!FC.is_SesStarted ||
		LQ_ReadInd == LQ_StoreInd &&
		LQ_Queue[LQ_StoreInd][0] == '\0')
	{

		return false;
	}

	// Get buffers 
	xbee_tx_size = SERIAL_BUFFER_SIZE - 1 - r2a.hwSerial.availableForWrite();
	xbee_rx_size = r2a.hwSerial.available();
	log_tx_size = SERIAL_BUFFER_SIZE - 1 - a2c.hwSerial.availableForWrite();
	log_rx_size = a2c.hwSerial.available();

	// Bail if buffer or time inadequate
	if (!(xbee_tx_size == 0 &&
		xbee_rx_size == 0 &&
		log_tx_size == 0 &&
		log_rx_size == 0 &&
		millis() > a2c.dt_sent + a2c.dt_minSentRcvd &&
		millis() > a2r.t_sent + a2r.dt_minSentRcvd &&
		millis() > r2a.t_rcvd + r2a.dt_minSentRcvd)) {

		// Indicate still logs to send
		return true;
	}

	// Increment send ind
	LQ_ReadInd++;

	// Check if ind should roll over 
	if (LQ_ReadInd == LQ_Capacity) {
		LQ_ReadInd = 0;
	}

	// Get message size
	msg_lng = strlen(LQ_Queue[LQ_ReadInd]);

	// Send
	a2c.hwSerial.write(LQ_Queue[LQ_ReadInd], msg_lng);
	a2c.dt_sent = a2c.t_sent > 0 ? millis() - a2c.t_sent : 0;
	a2c.t_sent = millis();
	cnt_logBytesSent += msg_lng;

	// Print
	if (Debug.flag.print_logSend)
	{

		// Update log buffer
		log_tx_size = SERIAL_BUFFER_SIZE - 1 - a2c.hwSerial.availableForWrite();
		log_rx_size = a2c.hwSerial.available();

		// Format stored log message
		Debug.sprintf_safe(buffLrg, buff_lrg, "   [LOG:%d] a2c: b_snt=%d l_rx=%d l_rx=%d dt=%d \"%s\"",
			cnt_logsStored, cnt_logBytesSent, log_tx_size, log_rx_size, a2c.dt_sent, LQ_Queue[LQ_ReadInd]);

		// Truncate message
		msg_lng = strlen(buff_lrg);
		if (msg_lng > buffLrg) {

			// Terminate message early
			buff_lrg[buffLrg - 4] = '.';
			buff_lrg[buffLrg - 3] = '.';
			buff_lrg[buffLrg - 2] = '.';
			buff_lrg[buffLrg - 1] = '\0';
		}

		// Store
		Debug.DB_LogSend(buff_lrg);
	}

	// Set entry to null
	LQ_Queue[LQ_ReadInd][0] = '\0';

	// Return success
	return true;
}

#pragma endregion

#pragma region --------REWARD---------

// START REWARD
void StartRew()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Set rew on pins
	SetPort(word_rewStr, word_rewEnd);

	// Add to count
	cnt_rew++;

	// Set rew off time
	t_rewEnd = millis() + rewDur;

	// Signal PID stopped
	digitalWrite(pin.TTL_PID_RUN, LOW);
	digitalWrite(pin.TTL_PID_STOP, HIGH);
	Debug.DB_General(__FUNCTION__, __LINE__, "PID STOPPED");

	// Set flag
	FC.is_Rewarding = true;

	// Print
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING: Reward: cnt_rew=%d dt_rew=%dms", cnt_rew, rewDur);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
}

// END REWARD
void EndRew()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	if (millis() > t_rewEnd)
	{

		// Set reward off pins
		SetPort(word_rewEnd, word_rewStr);

		FC.is_Rewarding = false;
		Debug.sprintf_safe(buffLrg, buff_lrg, "FINISHED: Reward: cnt_rew=%d dt_rew=%dms", cnt_rew, rewDur);
		Debug.DB_General(__FUNCTION__, __LINE__, "REWARD OFF");

	}
}

#pragma endregion

#pragma region --------TESTING---------

// HARDWARE TEST
void HardwareTest(int test_num)
{
	// Local vars
	int dt_on = 50;

	switch (test_num)
	{

		// Test arduino pin reward tone
	case 1:
	{
		// Write pins high then low
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST DUE PIN: Reward Tone");
		digitalWrite(pin.REL_WHITE, LOW);
		digitalWrite(pin.TTL_WHITE, LOW);
		digitalWrite(pin.REL_TONE, HIGH);
		digitalWrite(pin.TTL_TONE, HIGH);
		delay(dt_on);
		digitalWrite(pin.REL_TONE, LOW);
		digitalWrite(pin.TTL_TONE, LOW);
	}
	break;

	// Test arduino pin white noise
	case 2:
	{
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST DUE PIN: White Noise");
		digitalWrite(pin.REL_WHITE, HIGH);
		digitalWrite(pin.TTL_WHITE, HIGH);
		delay(dt_on);
		digitalWrite(pin.REL_WHITE, LOW);
		digitalWrite(pin.TTL_WHITE, LOW);
	}
	break;

	// Test port set reward only
	case 3:
	{
		// Create port word for rew event pins only
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST PORT WORD: Reward Event Only");
		int SAM_on_pins[1] = { pin.SAM_TTL_REW_ON };
		word_rewStr = GetPortWord(0x0, SAM_on_pins, 1);
		int SAM_off_pins[1] = { pin.SAM_TTL_REW_OFF };
		word_rewEnd = GetPortWord(0x0, SAM_off_pins, 1);

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
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST PORT WORD: Reward With Sound");
		int SAM_on_pins[3] = { pin.SAM_REL_TONE, pin.SAM_TTL_TONE, pin.SAM_TTL_REW_ON };
		word_rewStr = GetPortWord(0x0, SAM_on_pins, 3);
		int SAM_off_pins[3] = { pin.SAM_REL_WHITE, pin.SAM_TTL_WHITE, pin.SAM_TTL_REW_OFF };
		word_rewEnd = GetPortWord(0x0, SAM_off_pins, 3);

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
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST DUE PIN: IR");
		digitalWrite(pin.REL_IR, HIGH);
		digitalWrite(pin.TTL_IR, HIGH);
		delay(dt_on);
		digitalWrite(pin.REL_IR, LOW);
		digitalWrite(pin.TTL_IR, LOW);
	}
	break;

	// Test arduino pin IR
	case 6:
	{
		// Create port word for white and tone pins
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST PORT WORD: IR");

		// Write word on then off
		SetPort(word_irAll, 0x0);
		delay(dt_on);
		SetPort(0x0, word_irAll);

	}
	break;

	// Test arduino pin PID
	case 7:
	{
		// Write pins high then low
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST DUE PIN: PID");
		digitalWrite(pin.TTL_PID_RUN, HIGH);
		digitalWrite(pin.TTL_PID_STOP, LOW);
		delay(dt_on);
		digitalWrite(pin.TTL_PID_RUN, LOW);
		digitalWrite(pin.TTL_PID_STOP, HIGH);
	}
	break;


	// Test arduino pin Bull
	case 8:
	{
		// Write pins high then low
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST DUE PIN: BULL");
		digitalWrite(pin.TTL_BULL_RUN, HIGH);
		digitalWrite(pin.TTL_BULL_STOP, LOW);
		delay(dt_on);
		digitalWrite(pin.TTL_BULL_RUN, LOW);
		digitalWrite(pin.TTL_BULL_STOP, HIGH);
	}
	break;

	// Test arduino pin PT
	case 9:
	{
		// Write pins high then low
		Debug.DB_General(__FUNCTION__, __LINE__, "TEST DUE PIN: PT");
		digitalWrite(pin.TTL_NORTH, HIGH);
		delay(10);
		digitalWrite(pin.TTL_WEST, HIGH);
		delay(10);
		digitalWrite(pin.TTL_SOUTH, HIGH);
		delay(10);
		digitalWrite(pin.TTL_EAST, HIGH);
		delay(dt_on);
		digitalWrite(pin.TTL_NORTH, LOW);
		digitalWrite(pin.TTL_WEST, LOW);
		digitalWrite(pin.TTL_SOUTH, LOW);
		digitalWrite(pin.TTL_EAST, LOW);
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
	bool do_led_on = false;
	int dt_cycle = 100;

	// Flash 
	for (int i = 0; i < 10; i++)
	{
		do_led_on = !do_led_on;
		if (do_led_on) {
			// Pulse IR on without ttl
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_ON, false);
		}
		else {
			// Pulse IR off without ttl
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_OFF, false);
		}
		delay(dt_cycle);

	}

	// Reset LED
	SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_OFF, false);
}

// PULSE SYNC IR
bool IR_SyncPulse()
{
	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Attempt pulse
	if (SetIR(del_irSyncPulse, dt_irSyncPulseOn)) {

		// Increment count
		if (is_irOn) {
			cnt_ir++;
		}

		// Log ir event
		Debug.sprintf_safe(buffLrg, buff_lrg, "IR Sync %s: cnt=%d dt=%dms",
			is_irOn ? "Start" : "End", cnt_ir, millis() - t_irSyncLast);
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}

}

// SET IR
bool SetIR(int dt_pulse, int dt_on, SETIRSTATE force_state, bool do_ttl)
{
	// Local vars
	bool is_changed = false;

	// Bail if blocking
	if (FC.do_BlockIRPulse) {
		return false;
	}

	// Set high
	if (
		force_state == FORCE_ON ||
		(
			force_state == FREE &&
			!is_irOn &&
			millis() > t_irSyncLast + dt_pulse)
		)
	{
		// Set ir port on
		if (do_ttl) {
			SetPort(word_irAll, 0x0);
		}
		else {
			SetPort(word_irRel, 0x0);
		}

		// Store time
		t_irSyncLast = millis();

		// Update flags
		is_irOn = true;
		is_changed = true;
	}
	// Set low
	else if (
		force_state == FORCE_OFF ||
		(
			force_state == FREE &&
			is_irOn &&
			millis() > t_irSyncLast + dt_on)
		)
	{
		// Set ir port off
		SetPort(0x0, word_irAll);

		// Update flags
		is_irOn = false;
		is_changed = true;
	}
	return is_changed;
}

// GET ID INDEX
template <typename A24> int ID_Ind(char id, A24 *p_r24)
{

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// Return -1 if not found
	int ind = -1;
	for (int i = 0; i < p_r24->lng; i++) {

		if (id == p_r24->id[i]) {
			ind = i;
		}
	}

	// Print warning if not found
	if (ind == -1) {
		Debug.sprintf_safe(buffLrg, buff_lrg, "ID \'%c\' Not Found in %s", id, COM::str_list_id[p_r24->comID]);
		Debug.DB_Warning(__FUNCTION__, __LINE__, buff_lrg);
	}

	return ind;

}

// GET/SET BYTE BIT VALUE
bool GetSetByteBit(byte * b_set, int bit, bool do_set)
{
	// Set bit
	if (do_set) {
		*b_set = *b_set | 0x01 << bit;
	}

	// Return state
	return ((*b_set >> bit) & 0x01) == 1;
}

// GET 32 BIT WORD FOR PORT
uint32_t GetPortWord(uint32_t word, int *p_pin_arr, int arr_size)
{

	// Set 32 bit word bit state
	for (int i = 0; i < arr_size; i++) {
		word = word | 0x01 << p_pin_arr[i];
	}

	return word;
}

// SET PORT
void SetPort(uint32_t word_on, uint32_t word_off)
{

	// Copy current registry state
	uint32_t word_new = REG_PIOC_ODSR;

	// Set off entries
	word_new = word_new & ~word_off;

	// Set on entries
	word_new |= word_on;

	// Update port registry
	REG_PIOC_ODSR = word_new;

}

// RESET PT PINS
void ResetTTL()
{

	// Bail if nothing on
	if (!v_isOnAny) {
		return;
	}

	// North
	if (v_isOnNorth) {

		// Print on time
		if (Debug.flag.print_general && v_doPrintNorth) {
			Debug.DB_General(__FUNCTION__, __LINE__, "NORTH ON", v_t_outLastNorth);
			v_doPrintNorth = false;
		}

		// Set back to LOW
		if (millis() - v_t_outLastNorth > dt_ttlPulse) {

			digitalWrite(pin.TTL_NORTH, LOW);
			v_isOnNorth = false;
			// Print
			if (Debug.flag.print_general) {
				Debug.DB_General(__FUNCTION__, __LINE__, "NORTH OFF");
			}
		}
	}

	// West
	if (v_isOnWest) {

		// Print on time
		if (Debug.flag.print_general && v_doPrintWest) {
			Debug.DB_General(__FUNCTION__, __LINE__, "WEST ON", v_t_outLastWest);
			v_doPrintWest = false;
		}

		// Set back to LOW
		if (millis() - v_t_outLastWest > dt_ttlPulse) {

			digitalWrite(pin.TTL_WEST, LOW);
			v_isOnWest = false;

			// Print
			if (Debug.flag.print_general) {
				Debug.DB_General(__FUNCTION__, __LINE__, "WEST OFF");
			}
		}
	}

	// South
	if (v_isOnSouth) {

		// Print on time
		if (Debug.flag.print_general && v_doPrintSouth) {
			Debug.DB_General(__FUNCTION__, __LINE__, "SOUTH ON", v_t_outLastSouth);
			v_doPrintSouth = false;
		}

		// Set back to LOW
		if (millis() - v_t_outLastSouth > dt_ttlPulse) {

			digitalWrite(pin.TTL_SOUTH, LOW);
			v_isOnSouth = false;

			// Print
			if (Debug.flag.print_general) {
				Debug.DB_General(__FUNCTION__, __LINE__, "SOUTH OFF");
			}
		}
	}

	// East
	if (v_isOnEast) {

		// Print on time
		if (Debug.flag.print_general && v_doPrintEast) {
			Debug.DB_General(__FUNCTION__, __LINE__, "EAST ON", v_t_outLastEast);
			v_doPrintEast = false;
		}

		// Set back to LOW
		if (millis() - v_t_outLastEast > dt_ttlPulse) {

			digitalWrite(pin.TTL_EAST, LOW);
			v_isOnEast = false;

			// Print
			if (Debug.flag.print_general) {
				Debug.DB_General(__FUNCTION__, __LINE__, "EAST OFF");
			}
		}
	}

	// Update on flag
	v_isOnAny = v_isOnNorth || v_isOnWest || v_isOnSouth || v_isOnEast;
}

// QUIT AND RESTART ARDUINO
void QuitSession()
{
	// Local vars
	bool do_led_on = true;
	int cnt_flash = 0;

	// Bail if anything in queues
	if (SendPacket() ||
		SendLog() ||
		Debug.Print()) {
		return;
	}

	// Make sure we wont recieve a resend request
	if (millis() < r2a.t_rcvd + 500) {
		return;
	}

	// Flash and beep
	while (cnt_flash <= 3)
	{
		if (do_led_on)
		{
			// Set tone on
			digitalWrite(pin.REL_TONE, HIGH);
			digitalWrite(pin.REL_WHITE, LOW);

			// Pulse IR on without ttl
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_ON, false);
			delay(100);
		}
		else
		{
			// Set tone off
			digitalWrite(pin.REL_WHITE, HIGH);
			digitalWrite(pin.REL_TONE, LOW);

			// Pulse IR off without ttl
			SetIR(dt_irSyncPulse, dt_irSyncPulseOn, FORCE_ON, false);

			delay(50);
			cnt_flash++;
		}
		do_led_on = !do_led_on;
	}
	digitalWrite(pin.REL_WHITE, HIGH);
	digitalWrite(pin.REL_TONE, LOW);

	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

#pragma endregion

#pragma region --------INTERUPTS---------

// NORTH PT
void Interrupt_North()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastNorth > t_debounce) {

			digitalWrite(pin.TTL_NORTH, HIGH);
			v_t_outLastNorth = millis();
			v_doPrintNorth = true;
			v_isOnNorth = true;
			v_isOnAny = true;
		}
		v_t_inLastNorth = millis();
	}
}

// WSST PT
void Interrupt_West()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastWest > t_debounce) {
			digitalWrite(pin.TTL_WEST, HIGH);
			v_t_outLastWest = millis();
			v_doPrintWest = true;
			v_isOnWest = true;
			v_isOnAny = true;
		}
		v_t_inLastWest = millis();
	}
}

// SOUTH PT
void Interrupt_South()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastSouth > t_debounce) {

			digitalWrite(pin.TTL_SOUTH, HIGH);
			v_t_outLastSouth = millis();
			v_doPrintSouth = true;
			v_isOnSouth = true;
			v_isOnAny = true;
		}
		v_t_inLastSouth = millis();
	}
}

// EAST PT
void Interrupt_East()
{
	if (v_doPTInterupt)
	{
		if (millis() - v_t_inLastEast > t_debounce) {

			digitalWrite(pin.TTL_EAST, HIGH);
			v_t_outLastEast = millis();
			v_doPrintEast = true;
			v_isOnEast = true;
			v_isOnAny = true;
		}
		v_t_inLastEast = millis();
	}
}

#pragma endregion

#pragma endregion


void setup()
{
	// Local varss
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// Wait for SerialUSB if debugging
	uint32_t t_check = millis() + 5000;
	if (DO_PRINT_DEBUG) {
		while (!SerialUSB && millis() < t_check);
	}

	// XBee
	a2r.hwSerial.begin(57600);

	// CS through programming port
	a2c.hwSerial.begin(57600);

	// Dump CS buffers
	while (a2c.hwSerial.available() > 0) {
		a2c.hwSerial.read();
	}

	// SETUP PINS
	SetupPins();

	// LOG/PRINT SETUP RUNNING

	// Print run mode
	if (DO_DEBUG) {
		Debug.DB_General(__FUNCTION__, __LINE__, "RUN MODE = DEBUG");
	}
	else {
		Debug.DB_General(__FUNCTION__, __LINE__, "RUN MODE = RELEASE");
	}

	// Log compile time
	Debug.sprintf_safe(buffLrg, buff_lrg, "BUILD DATE: %s %s", __DATE__, __TIME__);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// Log and print to console
	Debug.DB_General(__FUNCTION__, __LINE__, "RUNNING: Setup...");
	Debug.PrintAll(1000);

	// SETUP TTL PORTS

	// Setup reward ttl stuff on port C
	REG_PIOC_OWER = 0xFFFFFFFF;     // enable PORT C
	REG_PIOC_OER = 0xFFFFFFFF;     // set PORT C as output port

	// Get ir all word
	int SAM_ir_all_pins[2] = { pin.SAM_TTL_IR, pin.SAM_REL_IR };
	word_irAll = GetPortWord(0x0, SAM_ir_all_pins, 2);
	Debug.sprintf_safe(buffLrg, buff_lrg,
		"Created Port Word \"%s\": pins=|%d|%d| word=%s",
		"word_irAll", SAM_ir_all_pins[0], SAM_ir_all_pins[1], Debug.FormatBinary(word_irAll));
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	Debug.PrintAll(1000);

	// Get ir relay word
	int SAM_ir_rel_pins[1] = { pin.SAM_REL_IR };
	word_irRel = GetPortWord(0x0, SAM_ir_rel_pins, 1);
	Debug.sprintf_safe(buffLrg, buff_lrg,
		"Created Port Word \"%s\": pins=|%d| word=%s",
		"word_irRel", SAM_ir_rel_pins[0], Debug.FormatBinary(word_irRel));
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	Debug.PrintAll(1000);

	// SETUP INTERUPTS

	// North
	attachInterrupt(digitalPinToInterrupt(pin.PT_NORTH), Interrupt_North, RISING);
	// West
	attachInterrupt(digitalPinToInterrupt(pin.PT_WEST), Interrupt_West, RISING);
	// South
	attachInterrupt(digitalPinToInterrupt(pin.PT_SOUTH), Interrupt_South, RISING);
	// East
	attachInterrupt(digitalPinToInterrupt(pin.PT_EAST), Interrupt_East, RISING);

	// PRINT PIN MAPPING
	/*
	Note: make sure Cheetah aquiring
	*/
	Debug.DB_PinMap();

	// PRINT DEBUG STATUS

	// Print settings
	Debug.sprintf_safe(buffLrg, buff_lrg, "RUNNING IN %s MODE: |%s%s",
		DO_DEBUG ? "DEBUG" : "RELEASE",
		DO_LOG ? "LOGGING ENABLED|" : "",
		DO_PRINT_DEBUG ? "PRINTING ENABLED|" : "");
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

	// PRINT AVAILABLE MEMORY
	Debug.sprintf_safe(buffLrg, buff_lrg, "AVAILABLE MEMORY: %0.2fKB",
		(float)freeMemory() / 1000);
	Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	while (Debug.Print());

	// CHECK SERIAL BUFFER SIZE
	Debug.sprintf_safe(buffLrg, buff_lrg, "SERIAL BUFFER SIZE: ACTUAL=%dB EXPECTED=%dB", SERIAL_BUFFER_SIZE, expectedSerialBufferSize);
	if (SERIAL_BUFFER_SIZE == expectedSerialBufferSize) {
		Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
	}
	// Buffer size is wrong
	else {
		Debug.DB_Error(__FUNCTION__, __LINE__, buff_lrg);
	}

	// SET WHITE NOISE RELAY HIGH
	digitalWrite(pin.REL_WHITE, HIGH);

	// SHOW RESTART BLINK
	delayMicroseconds(100);
	StatusBlink();

	// PRINT SETUP FINISHED
	Debug.DB_General(__FUNCTION__, __LINE__, "FINISHED: Setup");
	Debug.PrintAll(500);

}


void loop()
{

#pragma region //--- ONGOING OPPERATIONS ---

	// Local vars
	static char buff_lrg[buffLrg] = { 0 }; buff_lrg[0] = '\0';

	// DO LOOP CHECK
	Debug.CheckLoop();

	// RESET TTL PINS
	ResetTTL();

	// GET SERIAL INPUT
	GetSerial();

	// SEND DATA
	if (SendPacket());

	// PRINT QUEUED DB
	else if (Debug.Print());

	// SEND QUEUED LOG
	else(SendLog());

	// WAIT FOR HANDSHAKE
	if (!CheckForHandshake()) {
		return;
	}

	// CHECK FOR REWARD END
	if (FC.is_Rewarding) {
		EndRew();
	}
	// PULSE IR
	IR_SyncPulse();

	// CHECK FOR QUIT
	if (FC.do_Quit) {
		QuitSession();
	}

#pragma endregion

#pragma region //--- PROCESS NEW MESSAGES ---

	// CONTINUE LOOP IF NO NEW MESSAGE
	if (!r2a.is_new) {
		return;
	}

	// (t) SESTEM TEST
	if (r2a.idNew == 't') {

		// Wall IR timing test
		if (r2a.dat[0] == 5)
		{

			// Setup
			if (r2a.dat[1] == 0)
			{
				// Log/rint test
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TEST: WALL IR TIMING TEST");

				// Make sure IR off
				SetIR(0, 0, FORCE_OFF);

				// Set to block IR pulse
				FC.do_BlockIRPulse = true;
			}

			// Test finished
			if (r2a.dat[1] == 2)
			{
				// Unblock IR
				FC.do_BlockIRPulse = false;

				// Unset flag
				FC.do_BlockIRPulse = false;
			}

		}

		// Sync IR timing test
		if (r2a.dat[0] == 6)
		{
			// Setup
			if (r2a.dat[1] == 0)
			{
				// Log/rint test
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TEST: SYNC IR TIMING TEST");

				// Set flag
				Debug.flag.do_irSyncCalibration = true;

				// Make sure IR off
				SetIR(0, 0, FORCE_OFF);

				// Set to block IR pulse
				FC.do_BlockIRPulse = true;
			}

			// Pulse IR
			if (r2a.dat[1] == 1)
			{
				// Unblock IR
				FC.do_BlockIRPulse = false;

				// Turn on IR
				SetIR(0, 0, FORCE_ON);
				delayMicroseconds(10 * 1000);

				// Turn off IR
				SetIR(0, 0, FORCE_OFF);

				// Turn block back on
				FC.do_BlockIRPulse = true;

			}

			// Test finished
			if (r2a.dat[1] == 2)
			{
				// Unblock IR
				FC.do_BlockIRPulse = false;

				// Unset flag
				FC.do_BlockIRPulse = false;
			}

		}

		// Hardware test
		if (r2a.dat[0] == 7)
		{
			// Setup
			if (r2a.dat[1] == 0)
			{
				// Log/rint test
				Debug.DB_General(__FUNCTION__, __LINE__, "DO TEST: HARDWARE");

				// Set to block IR pulse
				FC.do_BlockIRPulse = true;
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
				FC.do_BlockIRPulse = false;
			}


		}
	}

	// (r) RUN REWARD TONE
	if (r2a.idNew == 'r') {

		// Get reward duration and convert to ms
		rewDur = (uint32_t)r2a.dat[0];

		// Run tone
		StartRew();
	}

	// (s) SESSION SETUP
	if (r2a.idNew == 's') {

		// No noise
		if (r2a.dat[0] == 0)
		{
			FC.do_WhiteNoise = false;
			FC.do_RewTone = false;
			Debug.DB_General(__FUNCTION__, __LINE__, "NO SOUND");
		}

		// White noise only
		else if (r2a.dat[0] == 1)
		{

			// Set flags
			FC.do_WhiteNoise = true;
			FC.do_RewTone = false;
			Debug.DB_General(__FUNCTION__, __LINE__, "DONT DO TONE");

		}

		// White and reward sound
		else if (r2a.dat[0] == 2)
		{

			// Set flags
			FC.do_WhiteNoise = true;
			FC.do_RewTone = true;
			Debug.DB_General(__FUNCTION__, __LINE__, "DO TONE");

		}

		// Create white, tone and reward word
		if (FC.do_WhiteNoise && FC.do_RewTone) {

			// Create word
			int SAM_on_pins[3] = { pin.SAM_REL_TONE, pin.SAM_TTL_TONE, pin.SAM_TTL_REW_ON };
			word_rewStr = GetPortWord(0x0, SAM_on_pins, 3);
			Debug.sprintf_safe(buffLrg, buff_lrg,
				"Created Port Word \"%s\": pins=|%d|%d|%d| word=%s",
				"word_rewStr", SAM_on_pins[0], SAM_on_pins[1], SAM_on_pins[2], Debug.FormatBinary(word_rewStr));
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			int SAM_off_pins[3] = { pin.SAM_REL_WHITE, pin.SAM_TTL_WHITE, pin.SAM_TTL_REW_OFF };
			word_rewEnd = GetPortWord(0x0, SAM_off_pins, 3);
			Debug.sprintf_safe(buffLrg, buff_lrg,
				"Created Port Word \"%s\": pins=|%d|%d|%d| word=%s",
				"word_rewEnd", SAM_off_pins[0], SAM_off_pins[1], SAM_off_pins[2], Debug.FormatBinary(word_rewEnd));
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

		}

		// Create reward event only word
		else {

			// Create word
			int SAM_on_pins[1] = { pin.SAM_TTL_REW_ON };
			word_rewStr = GetPortWord(0x0, SAM_on_pins, 1);
			Debug.sprintf_safe(buffLrg, buff_lrg,
				"Created Port Word \"%s\": pins=|%d| word=%s",
				"word_rewStr", SAM_on_pins[0], Debug.FormatBinary(word_rewStr));
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			int SAM_off_pins[1] = { pin.SAM_TTL_REW_OFF };
			word_rewEnd = GetPortWord(0x0, SAM_off_pins, 1);
			Debug.sprintf_safe(buffLrg, buff_lrg,
				"Created Port Word \"%s\": pins=|%d| word=%s",
				"word_rewEnd", SAM_off_pins[0], Debug.FormatBinary(word_rewEnd));
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);
		}

		// Turn on white noise
		if (FC.do_WhiteNoise) {

			// Set relay back to low first
			digitalWrite(pin.REL_WHITE, LOW);

			// Create word
			int SAM_white_pins[2] = { pin.SAM_REL_WHITE, pin.SAM_TTL_WHITE };
			uint32_t word_white = GetPortWord(0x0, SAM_white_pins, 2);
			Debug.sprintf_safe(buffLrg, buff_lrg,
				"Created Port Word \"%s\": pins=|%d|%d| word=%s",
				"word_white", SAM_white_pins[0], SAM_white_pins[1], Debug.FormatBinary(word_white));
			Debug.DB_General(__FUNCTION__, __LINE__, buff_lrg);

			// Set port
			SetPort(word_white, 0x0);

		}

	}

	// (p) SIGNAL PID MODE
	if (r2a.idNew == 'p') {
		// Signal PID stopped
		if (r2a.dat[0] == 0)
		{
			digitalWrite(pin.TTL_PID_STOP, HIGH);
			digitalWrite(pin.TTL_PID_RUN, LOW);
			Debug.DB_General(__FUNCTION__, __LINE__, "PID Stopped");
		}
		// Signal PID running
		else if (r2a.dat[0] == 1)
		{
			digitalWrite(pin.TTL_PID_RUN, HIGH);
			digitalWrite(pin.TTL_PID_STOP, LOW);
			Debug.DB_General(__FUNCTION__, __LINE__, "PID Started");
		}
	}

	// (b) SIGNAL BULLDOZE MODE
	if (r2a.idNew == 'b') {
		// Signal Bull stopped
		if (r2a.dat[0] == 0)
		{
			digitalWrite(pin.TTL_BULL_STOP, HIGH);
			digitalWrite(pin.TTL_BULL_RUN, LOW);
			Debug.DB_General(__FUNCTION__, __LINE__, "Bull Stopped");
		}
		// Signal Bull running
		else if (r2a.dat[0] == 1)
		{
			digitalWrite(pin.TTL_BULL_RUN, HIGH);
			digitalWrite(pin.TTL_BULL_STOP, LOW);
			Debug.DB_General(__FUNCTION__, __LINE__, "Bull Started");
		}
	}

	// (q) DO QUIT
	if (r2a.idNew == 'q') {

		// Log suammry
		Debug.DoSummary();

		// Set flags
		FC.do_Quit = true;
		Debug.DB_General(__FUNCTION__, __LINE__, "QUITING...");
	}

#pragma endregion

}
