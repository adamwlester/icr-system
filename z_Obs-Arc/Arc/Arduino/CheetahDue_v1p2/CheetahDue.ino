//-------CheetahDue-------

// DEFINE PINS
const int pin_LED = 13;
const int pin_RewTone = 51;
const int pin_WhiteNoise = 53;

// OTHER VARS
const long toneDir = 500; // (ms) on duration
char msg_id = ' ';
char rob2ard_idChar[3] = {
	't', // reward tone
	'w', // white noise on
	'o', // white noise off
};
char rob2ard_head = '[';
char rob2ard_foot = ']';
bool msg_pass = false;

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

	while (!SerialUSB);
	PrintState("RESET");

	// XBee.
	Serial1.begin(115200);

	// set output pins
	pinMode(pin_LED, OUTPUT);
	pinMode(pin_RewTone, OUTPUT);
	pinMode(pin_WhiteNoise, OUTPUT);

	// turn on white noise
	digitalWrite(pin_WhiteNoise, LOW);
	digitalWrite(pin_RewTone, LOW);


}

void loop()
{

	//digitalWrite(pin_WhiteNoise, LOW);
	//digitalWrite(pin_RewTone, HIGH);
	//delay(1000);
	//digitalWrite(pin_WhiteNoise, HIGH);
	//digitalWrite(pin_RewTone, LOW);
	//delay(1000);
	//digitalWrite(pin_WhiteNoise, LOW);
	//digitalWrite(pin_RewTone, LOW);

	// Check for XBee input
	if (Serial1.available() > 0) {
		msg_pass = ParseSerial();

	}

	// Run reward tone
	if (msg_pass && msg_id == 't') {
		PrintState("TONE ON");
		digitalWrite(pin_RewTone, HIGH);
		digitalWrite(pin_WhiteNoise, LOW);
		digitalWrite(pin_LED, HIGH);
		delay(toneDir);
		PrintState("TONE OFF");
		digitalWrite(pin_WhiteNoise, HIGH);
		digitalWrite(pin_RewTone, LOW);
		digitalWrite(pin_LED, LOW);
		msg_pass = false;
	}
	// Turn on white noise
	else if (msg_pass && msg_id == 'w') {
		PrintState("WHITE ON");
		digitalWrite(pin_WhiteNoise, HIGH);
		msg_pass = false;
	}
	// Turn off white noise
	else if (msg_pass && msg_id == 'o') {
		PrintState("WHITE OFF");
		digitalWrite(pin_WhiteNoise, LOW);
		msg_pass = false;
	}


}

// PARSE SERIAL INPUT
bool ParseSerial()
{

	static char head;
	static char foot;
	static bool pass;

	u.f = 0.0f;
	// Dump data till header byte is reached
	
	while (!(Serial1.peek() == rob2ard_idChar[0] ||
		Serial1.peek() == rob2ard_idChar[1] ||
		Serial1.peek() == rob2ard_idChar[2]) &&
		Serial1.available() > 0)
	{
		u.b[0] = Serial1.read(); // dump/store
		head = u.c[0];

	}

	// Check header
	if (head != rob2ard_head) {
		// mesage will be dumped
		return pass = false;
	}

	// Wait for complete packet
	while (Serial1.available() < 2);

	// get id
	u.b[1] = Serial1.read();
	msg_id = u.c[1];

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
		return pass = true;
	}

}

// PRINT STATUS
void PrintState(String str)
{
	char msg[50];
	float t = (float)(millis() / 1000.0f);
	sprintf(msg, " (%0.2f sec)\n", t);
	SerialUSB.print('\n');
	SerialUSB.print(str);
	SerialUSB.print(msg);
}



