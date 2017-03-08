char msg_head[2] = { '<', '_' };
char msg_foot = '>';
char msg_idArr[4] = { 'P', 'S', 'H', 'R' };
char msg_id;
int16_t msg_rec;
byte msg_ent;
long msg_ts;
float msg_rad;
int cnt = 0;

void setup()
{

	/* add setup code here */
	Serial1.begin(57600);
	SerialUSB.begin(57600);
	analogWrite(6, 255);
}

union u_tag {
	byte b[4]; // (byte) 1 byte
	char c[4]; // (char) 1 byte
	int16_t i[2]; // (int) 2 byte
	long l; // (long) 4 byte
	float f; // (float) 4 byte
} u;

void loop()
{

	// Check for new data
	if (Serial1.available() > 0)
	{
		bool pass = parseSerial();
	}

}

bool parseSerial()
{
	char head[2];
	char foot;
	bool pass;

	// Dump data till msgHeader byte is reached
	while (Serial1.available() > 0)
	{
		while (Serial1.peek() != msg_head[0])
		{
			Serial1.read(); // dump
		}
		// Quit do not have complete block left
		if (Serial1.available() < 15)
		{
			return pass = false;
		}
		// Save headers and id
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		u.b[2] = Serial1.read();
		head[0] = u.c[0];
		head[1] = u.c[1];
		msg_id = u.c[2];
		// Check for second header char right after
		if (head[1] != msg_head[1])
		{
			// quit
			return pass = false;
		}
		else break;
	}

	// Get VT data
	if (msg_id == msg_idArr[0])
	{
		// Get record number
		u.b[0] = Serial1.read();
		u.b[1] = Serial1.read();
		msg_rec = u.i[0];
		// Get Ent
		msg_ent = Serial1.read();
		// Get TS
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = Serial1.read();
		}
		msg_ts = u.l;
		// Get Rad
		for (int i = 0; i < 4; i++)
		{
			u.b[i] = Serial1.read();
		}
		msg_rad = u.f;
	}
	// Check for footer
	u.b[0] = Serial1.read();
	foot = u.c[0];
	if (foot == msg_foot) {
		cnt++;
		pass = true;
		//char msg[50];
		//sprintf(msg, "%s%s,%s,%d,%d,%0.2f,%s)", head, msg_id, msg_ent, msg_ts, msg_rad, foot);
		//SerialUSB.print(msg);
	}
	else return pass = false;

}
