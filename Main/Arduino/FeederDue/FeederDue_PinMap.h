#ifndef FeederDue_PinMap_h
#define FeederDue_PinMap_h
#include "Arduino.h"

// Pin mapping
struct PIN
{
	// Power off
	const int PWR_OFF = 45;
	const int PWR_ON = 44;
	const int PWR_Swtch = 24;
	const int PWR_Swtch_Grn = 25;

	// Autodriver
	const int AD_CSP_R = 5;
	const int AD_CSP_F = 6;
	const int AD_RST = 7;

	// XBees
	const int X1a_CTS = 29;
	const int X1b_CTS = 27;
	const int X1a_UNDEF = 28;
	const int X1b_UNDEF = 26;

	// Teensy
	const int Teensy_SendStart = 36;
	const int Teensy_Resetting = 38;
	const int Teensy_Unused = 40;

	// Display
	const int Disp_SCK = 8;
	const int Disp_MOSI = 9;
	const int Disp_DC = 10;
	const int Disp_RST = 11;
	const int Disp_CS = 12;
	const int Disp_LED = 13;

	// LEDs
	const int RewLED_R = 4;
	const int RewLED_C = 3;
	const int TrackLED = 2;

	// Relays
	const int Rel_EtOH = 23;
	const int Rel_Rew = 22;
	const int Rel_Vcc = A5;

	// Voltage Regulators
	const int REG_24V_ENBLE = 34;
	const int REG_12V_ENBLE = 46;
	const int REG_5V_ENBLE = 48;

	// BigEasyDriver
	const int ED_RST = 47;
	const int ED_SLP = 49;
	const int ED_DIR = 51;
	const int ED_STP = 53;
	const int ED_ENBL = 35;
	const int ED_MS1 = 37;
	const int ED_MS2 = 39;
	const int ED_MS3 = 41;

	// OpenLog
	const int OL_RST = 30;

	// Feeder switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int FeedSwitch_Gnd = 33;
	const int FeedSwitch = 32;

	// Voltage monitor
	const int BatVcc = A6;
	const int BatIC = A7;

	// Buttons
	const int Btn[3] = { A2, A1, A0 };

	// Testing
	int Test_Signal = A8;

	/*
	Note: pins bellow are all used for external interupts
	and must all be members of the same port (PortA)
	*/

	// IR proximity sensors
	const int IRprox_Rt = 42;
	const int IRprox_Lft = 43;

	// IR detector
	const int IRdetect = 31;
}
// Initialize
pin;


// SETUP PINS
void SetupPins() {
	
	// SETUP OUTPUT PINS

	// Power
	pinMode(pin.PWR_OFF, OUTPUT);
	pinMode(pin.PWR_ON, OUTPUT);
	pinMode(pin.PWR_Swtch_Grn, OUTPUT);
	// Voltage Regulators
	pinMode(pin.REG_24V_ENBLE, OUTPUT);
	pinMode(pin.REG_12V_ENBLE, OUTPUT);
	pinMode(pin.REG_5V_ENBLE, OUTPUT);
	// Autodriver
	pinMode(pin.AD_CSP_R, OUTPUT);
	pinMode(pin.AD_CSP_F, OUTPUT);
	pinMode(pin.AD_RST, OUTPUT);
	// Teensy
	pinMode(pin.Teensy_SendStart, OUTPUT);
	// Display
	pinMode(pin.Disp_SCK, OUTPUT);
	pinMode(pin.Disp_MOSI, OUTPUT);
	pinMode(pin.Disp_DC, OUTPUT);
	pinMode(pin.Disp_RST, OUTPUT);
	pinMode(pin.Disp_CS, OUTPUT);
	pinMode(pin.Disp_LED, OUTPUT);
	// LEDs
	pinMode(pin.RewLED_R, OUTPUT);
	pinMode(pin.RewLED_C, OUTPUT);
	pinMode(pin.TrackLED, OUTPUT);
	// Relays
	pinMode(pin.Rel_Rew, OUTPUT);
	pinMode(pin.Rel_EtOH, OUTPUT);
	pinMode(pin.Rel_Vcc, OUTPUT);
	// BigEasyDriver
	pinMode(pin.ED_RST, OUTPUT);
	pinMode(pin.ED_SLP, OUTPUT);
	pinMode(pin.ED_DIR, OUTPUT);
	pinMode(pin.ED_STP, OUTPUT);
	pinMode(pin.ED_ENBL, OUTPUT);
	pinMode(pin.ED_MS1, OUTPUT);
	pinMode(pin.ED_MS2, OUTPUT);
	pinMode(pin.ED_MS3, OUTPUT);
	// OpenLog
	pinMode(pin.OL_RST, OUTPUT);
	// Feeder switch
	pinMode(pin.FeedSwitch_Gnd, OUTPUT);
	delayMicroseconds(100);
	// Test Pins
	pinMode(pin.Test_Signal, OUTPUT);

	// Power
	digitalWrite(pin.PWR_OFF, LOW);
	digitalWrite(pin.PWR_ON, LOW);
	digitalWrite(pin.PWR_Swtch_Grn, LOW);
	// Voltage Regulators (start high)
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_12V_ENBLE, HIGH);
	digitalWrite(pin.REG_5V_ENBLE, HIGH);
	// Autodriver
	digitalWrite(pin.AD_CSP_R, LOW);
	digitalWrite(pin.AD_CSP_F, LOW);
	digitalWrite(pin.AD_RST, LOW);
	// Teensy (start high)
	digitalWrite(pin.Teensy_SendStart, HIGH);
	// Display
	digitalWrite(pin.Disp_SCK, LOW);
	digitalWrite(pin.Disp_MOSI, LOW);
	digitalWrite(pin.Disp_DC, LOW);
	digitalWrite(pin.Disp_RST, LOW);
	digitalWrite(pin.Disp_CS, LOW);
	digitalWrite(pin.Disp_LED, LOW);
	// LEDs
	digitalWrite(pin.RewLED_R, LOW);
	digitalWrite(pin.RewLED_C, LOW);
	digitalWrite(pin.TrackLED, LOW);
	// Relays
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);
	digitalWrite(pin.Rel_Vcc, LOW);
	// Big Easy Driver
	digitalWrite(pin.ED_MS1, LOW);
	digitalWrite(pin.ED_MS2, LOW);
	digitalWrite(pin.ED_MS3, LOW);
	digitalWrite(pin.ED_RST, LOW);
	digitalWrite(pin.ED_SLP, LOW);
	digitalWrite(pin.ED_DIR, LOW);
	digitalWrite(pin.ED_STP, LOW);
	digitalWrite(pin.ED_ENBL, LOW);
	// OpenLog
	digitalWrite(pin.OL_RST, LOW);
	// Feeder switch
	digitalWrite(pin.FeedSwitch_Gnd, LOW);
	delayMicroseconds(100);
	// Test Pins
	digitalWrite(pin.Test_Signal, LOW);

	// SET INPUT PINS

	// Power
	pinMode(pin.PWR_Swtch, INPUT);
	// XBees
	pinMode(pin.X1a_CTS, INPUT);
	pinMode(pin.X1b_CTS, INPUT);
	// Teensy
	pinMode(pin.Teensy_Resetting, INPUT);
	// Battery monitor
	pinMode(pin.BatVcc, INPUT);
	pinMode(pin.BatIC, INPUT);
	// IR proximity sensors
	pinMode(pin.IRprox_Rt, INPUT);
	pinMode(pin.IRprox_Lft, INPUT);
	// IR detector
	pinMode(pin.IRdetect, INPUT);

	// Set power, button and switch internal pullup
	pinMode(pin.PWR_Swtch, INPUT_PULLUP);
	for (int i = 0; i <= 2; i++) {
		pinMode(pin.Btn[i], INPUT_PULLUP);
	}
	pinMode(pin.FeedSwitch, INPUT_PULLUP);
	delayMicroseconds(100);
}

#endif
