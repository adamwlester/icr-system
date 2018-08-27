//######################################

//=========== CheetahDue.h =============

//######################################

#ifndef CHEETAHDUE_H
#define CHEETAHDUE_H

//========== DEBUG EXT DEFS ============

// CONSOLE
#define DO_PRINT_DEBUG 0 // 0

// SERIAL LOGGING
#define DO_LOG 1 // 1

// DEBUG SAFEVECTOR ERRORS TO CONSOLE
#define DO_VEC_DEBUG 1 // 0

//============== INCLUDE ===============

// LOCAL
#include "CheetahDue_PinMap.h"
//
#include "SafeVector.h"

//========== EXT DEFS OTHER ============

// SOFTWARE RESET
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#pragma region ============ DEBUG SETTINGS ============

// DEBUGGING FLAGS STRUCT
struct DB_FLAG
{

	// PRINTING

	bool print_general = DO_PRINT_DEBUG && true; // true
	bool print_errors = DO_PRINT_DEBUG && true; // true
	bool print_r2a = DO_PRINT_DEBUG && true; // true
	bool print_a2r = DO_PRINT_DEBUG && true; // true
	bool print_a2rQueued = DO_PRINT_DEBUG && true; // true
	bool print_logSend = DO_PRINT_DEBUG && false; // false

	// LOGGING

	bool log_general = DO_LOG && true; // true
	bool log_errors = DO_LOG && true; // true
	bool log_r2a = DO_LOG && true; // true
	bool log_a2r = DO_LOG && true; // true
	bool log_a2rQueued = DO_LOG && true; // true

	// TESTING

	// Manually set
	const bool do_printPimMapTest = false;
	const bool do_handshakeBypass = false;

	// Set by system
	bool do_irSyncCalibration = false;

};

// MAIN DEBUG FLAG
#if DO_PRINT_DEBUG 
#define DO_DEBUG 1
#else
#define DO_DEBUG 0
#endif

#pragma endregion 


#pragma region ============ VARIABLE SETUP ============

// Flow/state control
struct FC
{
	bool do_Quit = false;
	bool is_CSHandshakeDone = false;
	bool is_FeederDueHandshakeDone = false;
	bool is_SesStarted = false;
	bool do_WhiteNoise = false;
	bool do_RewTone = false;
	bool is_Rewarding = false;
	bool do_BlockIRPulse = false;
}
// Initialize
fc;

// DEBUGGING BUFFER PARAMETERS
const uint16_t buffMed = 50;
const uint16_t buffLrg = 250;
const uint16_t buffMax = buffLrg + 100;

// DEBUGGING GENERAL
uint32_t dt_timeoutHandshake = 5000; // (ms)
uint32_t cnt_loopTot = 0;
byte cnt_loopShort = 0;
uint32_t cnt_overflowRX = 0;
uint32_t cnt_timeoutRX = 0;

// LOGGING GENERAL
int cnt_logsStored = 0;

// LOG QUEUE
const int LQ_Capacity = 40;
char LQ_Queue[LQ_Capacity][buffMax] = { { 0 } };
int LQ_StoreInd = 0;
int LQ_ReadInd = 0;

// PRINT QUEUE
const int PQ_Capacity = 30;
char PQ_Queue[PQ_Capacity][buffMax] = { { 0 } };
int PQ_StoreInd = 0;
int PQ_ReadInd = 0;

// SERIAL COM GENERAL
const uint16_t expectedSerialBufferSize = 1028;
int cnt_bytesRead = 0;
int cnt_bytesDiscarded = 0;
int cnt_logBytesSent = 0;

//UNION FOR SERIAL COMS
union UNION_SERIAL {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (uint16_t) 2 byte
	uint32_t i32; // (uint32_t) 4 byte
	float f; // (float) 4 byte
};
UNION_SERIAL U;

// REWARD
int cnt_rew;
uint32_t rewDur; // (ms) 
uint32_t t_rewEnd;
uint32_t word_rewStr;
uint32_t word_rewEnd;

// IR SYNC
const int dt_irHandshakePulse = 75; // (ms) 75
const int dt_irSyncPulse = 500; // (ms) 500 
const int dt_irSyncPulseOn = 10; // (ms) 10 
uint32_t del_irSyncPulse = 60000; // (ms) 
uint32_t t_sync = 0;
bool is_irOn = false;
int cnt_ir = 0;
uint32_t t_irSyncLast;
uint32_t word_irAll;
uint32_t word_irRel;
enum SETIRSTATE
{
	FREE,
	FORCE_ON,
	FORCE_OFF
};

// INTERUPTS/VOLATILES

// IR flicker flag
volatile bool v_is_irHigh = false;
// IR flicker flag
volatile bool v_do_irTTL = false;
// PT interupt flag
volatile bool v_doPTInterupt = false;
// PT north vars
volatile uint32_t v_t_inLastNorth = millis();
volatile uint32_t v_t_outLastNorth = millis();
volatile bool v_isOnNorth = false;
volatile bool v_doPrintNorth = false;
// PT west vars
volatile uint32_t v_t_inLastWest = millis();
volatile uint32_t v_t_outLastWest = millis();
volatile bool v_isOnWest = false;
volatile bool v_doPrintWest = false;
// PT south 
volatile uint32_t v_t_inLastSouth = millis();
volatile uint32_t v_t_outLastSouth = millis();
volatile bool v_isOnSouth = false;
volatile bool v_doPrintSouth = false;
// PT east vars
volatile uint32_t v_t_inLastEast = millis();
volatile uint32_t v_t_outLastEast = millis();
volatile bool v_isOnEast = false;
volatile bool v_doPrintEast = false;
// Any pin
volatile bool v_isOnAny = false;

// TTL TIMERS
uint32_t t_debounce = 10; // (ms)
uint32_t dt_ttlPulse = 50; // (ms)


#pragma endregion 


#pragma region ============== COM SETUP ===============

// SERIAL QUEUE
const int SQ_Capacity = 10;
const int SQ_MsgBytes = 18;

// Packet range
const uint16_t _pack_range[2] = { 1, UINT16_MAX-1 };

// COM INSTANCE ID
namespace COM
{
	enum ID {
		a2r,
		r2a,
		a2c,
		c2a
	};
	const char str_list_id[4][buffMed] =
	{ { "a2r" },{ "r2a" },{ "a2c" },{ "c2a" } };
}


// FEEDERDUE COM IDs
const char _rob_id_list[9] =
{
	'h', // setup handshake
	'n', // ping test packets
	't', // hardware test
	'q', // quit/reset
	'r', // reward
	's', // sound cond [0, 1, 2]
	'p', // pid mode [0, 1]
	'b', // bull mode [0, 1]
	'\0'
};

// FEEDERDUE COM IDs
const char _cs_id_list[2] =
{
	'h', // setup handshake
	'\0'
};

// CHEETAHDUE OUTGOING SERIAL
template <typename HW>
struct A2_COM
{
	HW &hwSerial;
	const COM::ID comID;
	const int lng;
	const char head;
	const char foot;
	VEC<char> id;
	VEC<uint16_t> packRange;
	VEC<uint16_t> packArr;
	VEC<uint16_t> packConfArr;
	uint16_t packInd;
	uint32_t packSentAll;
	uint32_t packRcvdAll;
	VEC<float> dat1;
	VEC<float> dat2;
	VEC<float> dat3;
	VEC<uint32_t> t_sentArr;
	VEC<uint32_t> t_queuedArr;
	VEC<bool> do_rcvCheckArr;
	VEC<int> cnt_repeatArr;
	uint32_t cnt_repeat;
	uint32_t t_sent; // (ms)
	int dt_sent; // (ms) mmo
	int dt_minSentRcvd; // (ms) 
	byte SQ_Queue[SQ_Capacity][SQ_MsgBytes];
	int SQ_StoreInd;
	int SQ_ReadInd;
	A2_COM(HW &_hwSerial, const COM::ID _comID, const char _head, const char _foot, const char *_id, const uint16_t *_packRange, int _dt_minSentRcvd = 0) :
		hwSerial(_hwSerial),
		comID(_comID),
		lng(strlen(_id)),
		head(_head),
		foot(_foot),
		id(lng, __LINE__, _id),
		packRange(2, __LINE__, _packRange),
		packArr(lng, __LINE__),
		packConfArr(lng, __LINE__),
		packInd(_packRange[0] - 1),
		packSentAll(0),
		packRcvdAll(0),
		dat1(lng, __LINE__),
		dat2(lng, __LINE__),
		dat3(lng, __LINE__),
		t_sentArr(lng, __LINE__),
		t_queuedArr(lng, __LINE__),
		do_rcvCheckArr(lng, __LINE__),
		cnt_repeatArr(lng, __LINE__),
		cnt_repeat(0),
		t_sent(0),
		dt_sent(0),
		dt_minSentRcvd(_dt_minSentRcvd),
		SQ_Queue(),
		SQ_StoreInd(0),
		SQ_ReadInd(0)
	{}
};
A2_COM<USARTClass> a2r(Serial1, COM::ID::a2r, '{', '}', _rob_id_list, _pack_range, 5);
A2_COM<UARTClass> a2c(Serial, COM::ID::a2c, '<', '>', _cs_id_list, _pack_range, 5);

// FEEDERDUE INCOMING SERIAL
template <typename HW>
struct A4_COM
{
	HW &hwSerial;
	const COM::ID comID;
	const int lng;
	const char head;
	const char foot;
	VEC<char> id;
	VEC<uint16_t> packRange;
	VEC<uint16_t> packArr;
	VEC<uint16_t> packConfArr;
	uint16_t packInd;
	uint32_t packSentAll;
	uint32_t packRcvdAll;
	VEC<float> dat;
	uint32_t cnt_repeat;
	uint32_t cnt_dropped;
	char idNew;
	bool is_new;
	uint32_t t_rcvd; // (ms)
	int dt_rcvd; // (ms)
	int dt_minSentRcvd; // (ms) 
	A4_COM(HW &_hwSerial, const COM::ID _comID, const char _head, const char _foot, const char *_id, const uint16_t *_packRange, int _dt_minSentRcvd = 0) :
		hwSerial(_hwSerial),
		comID(_comID),
		lng(strlen(_id)),
		head(_head),
		foot(_foot),
		id(lng, __LINE__, _id),
		packRange(2, __LINE__, _packRange),
		packArr(lng, __LINE__),
		packConfArr(lng, __LINE__),
		packInd(_packRange[0] - 1),
		packSentAll(0),
		packRcvdAll(0),
		dat(3, __LINE__),
		cnt_repeat(0),
		cnt_dropped(0),
		idNew('\0'),
		is_new(false),
		t_rcvd(0),
		dt_rcvd(0),
		dt_minSentRcvd(_dt_minSentRcvd)
	{}
};
A4_COM<USARTClass> r2a(Serial1, COM::ID::r2a, '{', '}', _rob_id_list, _pack_range, 5);
A4_COM<UARTClass> c2a(Serial, COM::ID::c2a, '<', '>', _cs_id_list, _pack_range, 5);

#pragma endregion 

#endif