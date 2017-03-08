//-------CheetahDue-------

// SOFTWARE RESET
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

// DEFINE PINS
const int pin_LED = 13;
const int pin_relRewTone = 51;
const int pin_relWhiteNoise = 53;

// NLX TTL
const int pin_ttlRewTone = 36;
const int pin_ttlWhiteNoise = 37;
const int pin_ttlNorthOn = A7;
const int pin_ttlWestOn = A6;
const int pin_ttlSouthOn = A5;
const int pin_ttlEastOn = A4;
const int pin_ptNorthOn = A3;
const int pin_ptWestOn = A2;
const int pin_ptSouthOn = A1;
const int pin_ptEastOn = A0;

// PHOTO TRANSDUCER VARS

// Timer variables
long startTim = millis();
int trigDel = 10;
int pulseWdth = 50;

// north vars
volatile long northInLastTim = millis();
volatile long northInNowTim = millis();
volatile long northOutLastTim = millis();
volatile long northOutNowTim = millis();
// west vars
volatile long westInLastTim = millis();
volatile long westInNowTim = millis();
volatile long westOutLastTim = millis();
volatile long westOutNowTim = millis();
// south 
volatile long southInLastTim = millis();
volatile long southInNowTim = millis();
volatile long southOutLastTim = millis();
volatile long southOutNowTim = millis();
// east vars
volatile long eastInLastTim = millis();
volatile long eastInNowTim = millis();
volatile long eastOutLastTim = millis();
volatile long eastOutNowTim = millis();

// OTHER VARS

// Flow control
bool doPrintFlow = true;
bool doPrintRcvdPack = true;

// Serial
uint32_t t_sync = 0;
char msg_id = ' ';
uint16_t packNow;
char rob2ard_id[5] = {
	'r', // reward tone
	'w', // white noise on
	'o', // white noise off
	't', // set sync time
	'q', // quit/reset
};
char rob2ard_head = '[';
char rob2ard_foot = ']';
bool msg_pass = false;

// Reward tone
bool isToneOn = false;
const long toneDir = 500; // (ms) on duration
uint32_t t_toneOff = millis();

//----------CLASS: union----------
union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	uint16_t i16[2]; // (int16) 2 byte
	uint32_t i32; // (int32) 4 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
} u;

void setup()
{

	//while (!SerialUSB);
	PrintState("SETUP");

	// XBee.
	Serial1.begin(57600);

	// Set pin direction
	pinMode(pin_ptNorthOn, INPUT);
	pinMode(pin_ptWestOn, INPUT);
	pinMode(pin_ptSouthOn, INPUT);
	pinMode(pin_ptEastOn, INPUT);
	pinMode(pin_ttlNorthOn, OUTPUT);
	pinMode(pin_ttlWestOn, OUTPUT);
	pinMode(pin_ttlSouthOn, OUTPUT);
	pinMode(pin_ttlEastOn, OUTPUT);

	// Set initial output state
	digitalWrite(pin_ttlNorthOn, LOW);
	digitalWrite(pin_ttlWestOn, LOW);
	digitalWrite(pin_ttlSouthOn, LOW);
	digitalWrite(pin_ttlEastOn, LOW);

	// External interrupt
	attachInterrupt(digitalPinToInterrupt(pin_ptNorthOn), NorthInterrupt, RISING); // north
	attachInterrupt(digitalPinToInterrupt(pin_ptWestOn), WestInterrupt, RISING); // west
	attachInterrupt(digitalPinToInterrupt(pin_ptSouthOn), SouthInterrupt, RISING); // south
	attachInterrupt(digitalPinToInterrupt(pin_ptEastOn), EastInterrupt, RISING); // east

	// set other output pins
	pinMode(pin_LED, OUTPUT);
	pinMode(pin_relRewTone, OUTPUT);
	pinMode(pin_relWhiteNoise, OUTPUT);

	// set noise off
	digitalWrite(pin_relWhiteNoise, LOW);
	digitalWrite(pin_relRewTone, LOW);
}

void loop()
{
    // Check for XBee input
	msg_pass = false;
	if (Serial1.available() > 0) {
		msg_pass = ParseSerial();
	}

	// Run reward tone
	if (msg_pass && msg_id == 'r') {
		t_toneOff = millis() + toneDir;
		digitalWrite(pin_relRewTone, HIGH);
		digitalWrite(pin_relWhiteNoise, LOW);
		digitalWrite(pin_LED, HIGH);
		digitalWrite(pin_ttlRewTone, HIGH);
		PrintState("TONE ON");
		isToneOn = true;
	}

	// Turn on white noise
	else if (msg_pass && msg_id == 'w') {
		digitalWrite(pin_relWhiteNoise, HIGH);
		PrintState("WHITE ON");
	}

	// Turn off white noise
	else if (msg_pass && msg_id == 'o') {
		digitalWrite(pin_relWhiteNoise, LOW);
		PrintState("WHITE OFF");
	}

	// Set sync time
	else if (msg_pass && msg_id == 't') {
		t_sync = millis();
		PrintState("SET SYNC TIME");
	}

	// Quite and reset
	else if (msg_pass && msg_id == 'q') {
		PrintState("QUITING");
		// Restart Arduino
		REQUEST_EXTERNAL_RESET;
	}

	// Check for tone end time ellapsed
	if (isToneOn && millis() > t_toneOff)
	{
		digitalWrite(pin_relWhiteNoise, HIGH);
		digitalWrite(pin_relRewTone, LOW);
		digitalWrite(pin_LED, LOW);
		digitalWrite(pin_ttlRewTone, LOW);
		PrintState("TONE OFF");
		isToneOn = false;
	}

	// Set output pins back to low

	// north
	if (northOutNowTim - northOutLastTim > pulseWdth) {
		digitalWrite(pin_ttlNorthOn, LOW); // set back to LOW
	}
	// west
	if (westOutNowTim - westOutLastTim > pulseWdth) {
		digitalWrite(pin_ttlWestOn, LOW); // set back to LOW
	}
	// south
	if (southOutNowTim - southOutLastTim > pulseWdth) {
		digitalWrite(pin_ttlSouthOn, LOW); // set back to LOW
	}
	// east
	if (eastOutNowTim - eastOutLastTim > pulseWdth) {
		digitalWrite(pin_ttlEastOn, LOW); // set back to LOW
	}

}

// PARSE SERIAL INPUT
bool ParseSerial()
{

	static char head;
	static char foot;
	static bool pass;

	// Dump data till header byte is reached
	while (Serial1.peek() != rob2ard_head && Serial1.available() > 0)
	{
		Serial1.read(); // dump
	}

	// Save header
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	head = u.c[0];

	// Check header
	if (head != rob2ard_head) {
		// mesage will be dumped
		return pass = false;
	}

	// Wait for id packet
	while (Serial1.available() < 1);

	// get id
	u.b[1] = Serial1.read();
	msg_id = u.c[1];

	// Wait for pack number packet
	while (Serial1.available() < 2);
	// get packet num
	u.f = 0.0f;
	u.b[0] = Serial1.read();
	u.b[1] = Serial1.read();
	packNow = u.i16[0];

	// Wait for foot packet
	while (Serial1.available() < 1);

	// Check for footer
	u.b[2] = Serial1.read();
	foot = u.c[2];

	// Footer missing
	if (foot != rob2ard_foot) {
		// mesage will be dumped
		return pass = false;
	}
	else
	{
		PrintRcvdPack(msg_id, packNow);
		return pass = true;
	}

}

// PRINT STATUS
void PrintState(String str)
{
	if (doPrintFlow)
	{
		char msg[50];
		uint32_t ts;
		float t;

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);

		// print
		sprintf(msg, " (%0.3fsec)\n", t);
		SerialUSB.print('\n');
		SerialUSB.print(str);
		SerialUSB.print(msg);
	}
}

// PRINT RECIEVED PACKET
void PrintRcvdPack(char id, uint16_t pack)
{

	//// Print
	if (doPrintRcvdPack)
	{
		char str[50];
		static uint32_t t1 = millis();
		uint32_t ts;
		uint32_t dt;
		float t;

		// Print

		// compute time
		if (t_sync == 0) ts = millis();
		else ts = t_sync;
		t = (float)((millis() - ts) / 1000.0f);
		dt = millis() - t1;

		// Print specific pack contents
		sprintf(str, "---Rcvd: [id:%c pack:%d (dt:%dms tot:%0.3fsec)]\n", id, pack, dt, t);
		SerialUSB.print(str);
		t1 = millis();
	}
}

// Monitor Phototransducer Inputs

// North
void NorthInterrupt()
{
	NorthFun();
}
void NorthFun()
{
	if (millis() - northInLastTim > trigDel) {
		digitalWrite(pin_ttlNorthOn, HIGH); // set out high
		Serial.println(millis());
		northInNowTim = millis(); // save current time
		northOutNowTim = millis(); // save current time
	}
	northInLastTim = millis();
	PrintState("North On");
}

// West
void WestInterrupt()
{
	WestFun();
}
void WestFun()
{
	if (millis() - westInLastTim > trigDel) {
		digitalWrite(pin_ttlWestOn, HIGH); // set out high
		Serial.println(millis());
		westInNowTim = millis(); // save current time
		westOutNowTim = millis(); // save current time
	}
	westInLastTim = millis();
	PrintState("West On");
}

// South
void SouthInterrupt()
{
	SouthFun();
}
void SouthFun()
{
	if (millis() - southInLastTim > trigDel) {
		digitalWrite(pin_ttlSouthOn, HIGH); // set out high
		Serial.println(millis());
		southInNowTim = millis(); // save current time
		southOutNowTim = millis(); // save current time
	}
	southInLastTim = millis();
	PrintState("South On");
}

// East
void EastInterrupt()
{
	EastFun();
}
void EastFun()
{
	if (millis() - eastInLastTim > trigDel) {
		digitalWrite(pin_ttlEastOn, HIGH); // set out high
		Serial.println(millis());
		eastInNowTim = millis(); // save current time
		eastOutNowTim = millis(); // save current time
	}
	eastInLastTim = millis();
	PrintState("East On");
}



