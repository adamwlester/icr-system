//######################################

//======== FeederDue_PinMap.h ==========

//######################################


#ifndef CHEETAHDUE_PINMAP_H
#define CHEETAHDUE_PINMAP_H


#pragma region ============== DEFINE PINS ==============

struct PIN
{
	// Relays
	const int REL_IR = 51;				// port=PC due=51 sam=12
	const int REL_TONE = 47;			// port=PC due=47 sam=16
	const int REL_WHITE = 45;		// port=PC due=45 sam=18

									// TTL
	const int TTL_IR = 50;				// port=PC due=50 sam=13 nlx=1,7
	const int TTL_TONE = 46;			// port=PC due=46 sam=17 nlx=1,4
	const int TTL_WHITE = 44;		// port=PC due=44 sam=19 nlx=1,5
	const int TTL_REW_ON = 34;			// port=PC due=34 sam=2  nlx=1,0
	const int TTL_REW_OFF = 36;			// port=PC due=36 sam=4  nlx=1,1

										// SAM3X pin
	const int SAM_REL_IR = 12;			// port=PC due=51 sam=12
	const int SAM_REL_TONE = 16;		// port=PC due=47 sam=16
	const int SAM_REL_WHITE = 18;	// port=PC due=45 sam=18
	const int SAM_TTL_IR = 13;			// port=PC due=50 sam=13 nlx=1,7
	const int SAM_TTL_TONE = 17;		// port=PC due=46 sam=17 nlx=1,4
	const int SAM_TTL_WHITE = 19;	// port=PC due=44 sam=19 nlx=1,5
	const int SAM_TTL_REW_ON = 2;			// port=PC due=34 sam=2  nlx=1,0
	const int SAM_TTL_REW_OFF = 4;		// port=PC due=36 sam=4  nlx=1,1

	// PID
	const int TTL_PID_RUN = 26;
	const int TTL_PID_STOP = 28;

	// Bulldozer
	const int TTL_BULL_RUN = 30;
	const int TTL_BULL_STOP = 32;

	// IR blink switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int SWITCH_BLINK_GRN = 11;
	const int SWITCH_BLINK = 12;

	// PT
	const int TTL_NORTH = A7;
	const int TTL_WEST = A6;
	const int TTL_SOUTH = A5;
	const int TTL_EAST = A4;
	const int PT_NORTH = A3;
	const int PT_WEST = A2;
	const int PT_SOUTH = A1;
	const int PT_EAST = A0;
}
// Initialize
pin;

#pragma endregion 


#pragma region ============== SETUP PINS ===============

void SetupPins() {

	// Set PT pin direction
	pinMode(pin.PT_NORTH, INPUT);
	pinMode(pin.PT_WEST, INPUT);
	pinMode(pin.PT_SOUTH, INPUT);
	pinMode(pin.PT_EAST, INPUT);
	pinMode(pin.TTL_NORTH, OUTPUT);
	pinMode(pin.TTL_WEST, OUTPUT);
	pinMode(pin.TTL_SOUTH, OUTPUT);
	pinMode(pin.TTL_EAST, OUTPUT);

	// Set PT pins low
	digitalWrite(pin.TTL_NORTH, LOW);
	digitalWrite(pin.TTL_WEST, LOW);
	digitalWrite(pin.TTL_SOUTH, LOW);
	digitalWrite(pin.TTL_EAST, LOW);

	// Set relay/ttl pin direction
	pinMode(pin.REL_IR, OUTPUT);
	pinMode(pin.REL_WHITE, OUTPUT);
	pinMode(pin.REL_TONE, OUTPUT);
	pinMode(pin.TTL_IR, OUTPUT);
	pinMode(pin.TTL_TONE, OUTPUT);
	pinMode(pin.TTL_WHITE, OUTPUT);

	// Set relay/ttl pins low
	digitalWrite(pin.REL_IR, LOW);
	digitalWrite(pin.REL_TONE, LOW);
	digitalWrite(pin.REL_WHITE, LOW);
	digitalWrite(pin.TTL_IR, LOW);
	digitalWrite(pin.TTL_TONE, LOW);
	digitalWrite(pin.TTL_WHITE, LOW);

	// Set other ttl pins
	pinMode(pin.TTL_REW_ON, OUTPUT);
	pinMode(pin.TTL_REW_OFF, OUTPUT);
	pinMode(pin.TTL_BULL_RUN, OUTPUT);
	pinMode(pin.TTL_BULL_STOP, OUTPUT);
	pinMode(pin.TTL_PID_RUN, OUTPUT);
	pinMode(pin.TTL_PID_STOP, OUTPUT);

	// Set ir blink switch
	pinMode(pin.SWITCH_BLINK_GRN, OUTPUT);
	digitalWrite(pin.SWITCH_BLINK_GRN, LOW);
	pinMode(pin.SWITCH_BLINK, INPUT_PULLUP);

}

#pragma endregion 


#endif