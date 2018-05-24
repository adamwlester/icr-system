
#ifndef FeederDue_PinMap_h
#define FeederDue_PinMap_h

// Pin mapping
struct PIN
{
	// Power off
	const int PWR_OFF = 48; // (gray)
	const int PWR_ON = 46; // (white)
	const int PWR_Swtch = 44; // (red)
	const int PWR_Swtch_Grn = 45; // (black)

	// Autodriver
	const int AD_CSP_R = 5; // (yellow)
	const int AD_CSP_F = 6; // (red)
	const int AD_RST = 7; // (brown)

	// XBees
	const int X1a_CTS = 28; // (blue)
	const int X1b_CTS = 26; // (blue)
	const int X1a_UNDEF = 29; // (yellow)
	const int X1b_UNDEF = 27; // (yellow)

	// Teensy
	const int Teensy_Unused = 36; // (green)
	const int Teensy_SendLogs = 38; // (yellow)
	const int Teensy_Resetting = 40; // (orange)

	// Display
	const int Disp_CS = 8; // (white)
	const int Disp_RST = 9; // (gray)
	const int Disp_DC = 10; // (purple)
	const int Disp_MOSI = 11; // (blue)
	const int Disp_SCK = 12; // (green)
	const int Disp_LED = 13; // (yellow)

	// LEDs
	const int RewLED_C = 2; // (white)
	const int RewLED_R = 3; // (yellow)
	const int TrackLED = 4; // (red)

	// Relays
	const int Rel_Rew = 22; // (green)
	const int Rel_EtOH = 23; // (blue)
	const int Rel_Vcc = A5; // (blue)

	// Voltage Regulators
	const int REG_24V_ENBLE = 33; // (green)
	const int REG_12V_ENBLE = 50; // (orange) 
	const int REG_5V_ENBLE = 52; // (yellow) 
	const int REG_5V_PIXY_ENBLE = 32; // (blue)

	// BigEasyDriver
	const int ED_RST = 47; // (green)
	const int ED_SLP = 49; // (blue)
	const int ED_DIR = 51; // (gray)
	const int ED_STP = 53; // (white)
	const int ED_ENBL = 35; // (brown)
	const int ED_MS1 = 37; // (red)
	const int ED_MS2 = 39; // (orange)
	const int ED_MS3 = 41; // (yellow)

	// OpenLog
	const int OL_RST = 34; // (purple)

	// Feeder switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int FeedSwitch = 24; // (red)
	const int FeedSwitch_Gnd = 25; // (black)

	// Voltage monitor
	const int BatVcc = A6; // (purple)
	const int BatIC = A7; // (blue)

	// Buttons
	const int Btn[3] = { A2, A1, A0 }; // (blue, purple, white)

	// Testing
	int Test_Signal = A8;

	/*
	Note: pins bellow are all used for external interupts
	and must all be members of the same port (PortA)
	*/

	// IR proximity sensors
	const int IRprox_Rt = 42; // (green)
	const int IRprox_Lft = 43; // (blue)

	// IR detector
	const int IRdetect = 31; // (white)
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
	pinMode(pin.REG_5V_PIXY_ENBLE, OUTPUT);
	// Autodriver
	pinMode(pin.AD_CSP_R, OUTPUT);
	pinMode(pin.AD_CSP_F, OUTPUT);
	pinMode(pin.AD_RST, OUTPUT);
	// Teensy
	pinMode(pin.Teensy_SendLogs, OUTPUT);
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
	digitalWrite(pin.REG_12V_ENBLE, HIGH);
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_5V_ENBLE, HIGH);
	digitalWrite(pin.REG_5V_PIXY_ENBLE, HIGH);
	// Autodriver
	digitalWrite(pin.AD_CSP_R, LOW);
	digitalWrite(pin.AD_CSP_F, LOW);
	digitalWrite(pin.AD_RST, LOW);
	// Teensy (start high)
	digitalWrite(pin.Teensy_SendLogs, HIGH);
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
