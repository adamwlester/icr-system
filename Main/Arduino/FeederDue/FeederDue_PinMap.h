//######################################

//======== FeederDue_PinMap.h ==========

//######################################


#ifndef FEEDERDUE_PINMAP_H
#define FEEDERDUE_PINMAP_H


//============== NOTES =================
/*
PIN MAP:
0,RX_R24T,blue
1,TX_R24T,purple
2,LED_REW_C,white
3,LED_REW_R,yellow
4,LED_TRACKER,red
5,AD_CSP_R,yellow
6,AD_CSP_F,red
7,AD_RST,brown
8,LCD_CS,white
9,LCD_RST,gray
10,LCD_DC,purple
11,LCD_MOSI,blue
12,LCD_SCK,green
13,LCD_LED,yellow
14,TX_R24C,orange
15,RX_R24C,purple
16,TX_R24A,orange
17,RX_R24A,purple
18,TX_OL,blue
19,RX_OL,green
20,SLA_PIXY,white
21,SCL_PIXY,green
22,REL_FOOD,green
23,REL_ETOH,blue
24,PWR_SWITCH,white
25,PWR_SWITCH_GRN,black
26,XB_CTS_F,blue
27,XB_UNDEF_F,yellow
28,XB_CTS_R,blue
29,XB_UNDEF_R,yellow
30,OL_RST,purple
31,INTERUPT_IR_DETECT,white
32,TEENSY_UNUSED,green
33,ED_ENBL,yellow
34,TEENSY_SEND,yellow
35,ED_MS1,orange
36,TEENSY_RESET,orange
37,ED_MS2,red
38,UNUSED,NA
39,ED_MS3,brown
40,SWITCH_DISH,red
41,SWITCH_DISH_GRN,black
42,INTERUPT_IRPROX_R,green
43,INTERUPT_IRPROX_L,blue
44,PWR_OFF,gray
45,PWR_ON,white
46,REG_5V2_ENBLE,blue
47,ED_RST,green
48,REG_5V1_ENBLE,green
49,ED_SLP,blue
50,REG_12V2_ENBLE,yellow
51,ED_DIR,gray
52,UNUSED,NA
53,ED_STP,white
54,BTN_3,white
55,BTN_2,purple
56,BTN_1,blue
57,REL_VCC,orange
58,REG_24V_ENBLE,red
59,BAT_VCC,brown
60,UNUSED,NA
61,UNUSED,NA
62,TEST_SIGNAL,NA
63,UNUSED,NA
64,UNUSED,NA
65,UNUSED,NA

*/

#pragma region ============== DEFINE PINS ==============

struct PIN
{
	// Power button
	const int PWR_OFF = 44; // (gray)
	const int PWR_ON = 45; // (white)

	// Autodriver
	const int AD_CSP_R = 5; // (yellow)
	const int AD_CSP_F = 6; // (red)
	const int AD_RST = 7; // (brown)

	// XBees
	const int XB_CTS_F = 26; // (blue)
	const int XB_CTS_R = 28; // (blue)
	const int XB_UNDEF_F = 27; // (yellow)
	const int XB_UNDEF_R = 29; // (yellow)

	// Teensy
	const int TEENSY_UNUSED = 32; // (green)
	const int TEENSY_SEND = 34; // (yellow)
	const int TEENSY_RESET = 36; // (orange)

	// Display
	const int LCD_CS = 8; // (white)
	const int LCD_RST = 9; // (gray)
	const int LCD_DC = 10; // (purple)
	const int LCD_MOSI = 11; // (blue)
	const int LCD_SCK = 12; // (green)
	const int LCD_LED = 13; // (yellow)

	// LEDs
	const int LED_REW_C = 2; // (white)
	const int LED_REW_R = 3; // (yellow)
	const int LED_TRACKER = 4; // (red)

	// Relays
	const int REL_FOOD = 22; // (green)
	const int REL_ETOH = 23; // (blue)
	const int REL_VCC = A3; // (orange)

	// Voltage Regulators
	const int REG_24V_ENBLE = A4; // (red)
	const int REG_12V2_ENBLE = 50; // (yellow) 
	const int REG_5V1_ENBLE = 48; // (green) 
	const int REG_5V2_ENBLE = 46; // (blue)

	// BigEasyDriver
	const int ED_RST = 47; // (green)
	const int ED_SLP = 49; // (blue)
	const int ED_DIR = 51; // (gray)
	const int ED_STP = 53; // (white)
	const int ED_ENBL = 33; // (yellow)
	const int ED_MS1 = 35; // (orange)
	const int ED_MS2 = 37; // (red)
	const int ED_MS3 = 39; // (brown)

	// OpenLog
	const int OL_RST = 30; // (purple)

	// Feeder switch
	/*
	Note: Do not use real ground pin as this will cause
	an upload error if switch is shorted when writing sketch
	*/
	const int SWITCH_DISH = 40; // (red)
	const int SWITCH_DISH_GRN = 41; // (black)

	// Voltage monitor
	const int BAT_VCC = A5; // (brown)

	// Buttons
	const int BTN_1 = A2; // (blue)
	const int BTN_2 = A1; // (purple)
	const int BTN_3 = A0; // (white)

	// Testing
	int TEST_SIGNAL = A8;

	// IR proximity sensors
	const int IRPROX_R = 42; // (green)
	const int IRPROX_L = 43; // (blue)

	/*
	Note: pins bellow are all used for external interupts
	and must all be members of the same hwSerial (PortA)
	*/

	// Power button
	const int PWR_SWITCH = 24; // (white)
	const int PWR_SWITCH_GRN = 25; // (black)

	// IR detector
	const int INTERUPT_IR_DETECT = 31; // (white)
}
// Initialize
pin;

#pragma endregion 


#pragma region ============== SETUP PINS ===============

void SetupPins() {

	// SETUP OUTPUT PINS

	// Power
	pinMode(pin.PWR_OFF, OUTPUT);
	pinMode(pin.PWR_ON, OUTPUT);
	pinMode(pin.PWR_SWITCH_GRN, OUTPUT);
	// Voltage Regulators
	pinMode(pin.REG_24V_ENBLE, OUTPUT);
	pinMode(pin.REG_12V2_ENBLE, OUTPUT);
	pinMode(pin.REG_5V1_ENBLE, OUTPUT);
	pinMode(pin.REG_5V2_ENBLE, OUTPUT);
	// Autodriver
	pinMode(pin.AD_CSP_R, OUTPUT);
	pinMode(pin.AD_CSP_F, OUTPUT);
	pinMode(pin.AD_RST, OUTPUT);
	// Teensy
	pinMode(pin.TEENSY_SEND, OUTPUT);
	// Display
	pinMode(pin.LCD_SCK, OUTPUT);
	pinMode(pin.LCD_MOSI, OUTPUT);
	pinMode(pin.LCD_DC, OUTPUT);
	pinMode(pin.LCD_RST, OUTPUT);
	pinMode(pin.LCD_CS, OUTPUT);
	pinMode(pin.LCD_LED, OUTPUT);
	// LEDs
	pinMode(pin.LED_REW_R, OUTPUT);
	pinMode(pin.LED_REW_C, OUTPUT);
	pinMode(pin.LED_TRACKER, OUTPUT);
	// Relays
	pinMode(pin.REL_FOOD, OUTPUT);
	pinMode(pin.REL_ETOH, OUTPUT);
	pinMode(pin.REL_VCC, OUTPUT);
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
	pinMode(pin.SWITCH_DISH_GRN, OUTPUT);
	delayMicroseconds(100);
	// Test Pins
	pinMode(pin.TEST_SIGNAL, OUTPUT);

	// Power
	digitalWrite(pin.PWR_OFF, LOW);
	digitalWrite(pin.PWR_ON, LOW);
	digitalWrite(pin.PWR_SWITCH_GRN, LOW);
	// Voltage Regulators (start high)
	digitalWrite(pin.REG_12V2_ENBLE, HIGH);
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_5V1_ENBLE, HIGH);
	digitalWrite(pin.REG_5V2_ENBLE, HIGH);
	// Autodriver
	digitalWrite(pin.AD_CSP_R, LOW);
	digitalWrite(pin.AD_CSP_F, LOW);
	digitalWrite(pin.AD_RST, LOW);
	// Teensy
	digitalWrite(pin.TEENSY_SEND, LOW);
	// Display
	digitalWrite(pin.LCD_SCK, LOW);
	digitalWrite(pin.LCD_MOSI, LOW);
	digitalWrite(pin.LCD_DC, LOW);
	digitalWrite(pin.LCD_RST, LOW);
	digitalWrite(pin.LCD_CS, LOW);
	digitalWrite(pin.LCD_LED, LOW);
	// LEDs
	digitalWrite(pin.LED_REW_R, LOW);
	digitalWrite(pin.LED_REW_C, LOW);
	digitalWrite(pin.LED_TRACKER, LOW);
	// Relays
	digitalWrite(pin.REL_FOOD, LOW);
	digitalWrite(pin.REL_ETOH, LOW);
	digitalWrite(pin.REL_VCC, LOW);
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
	digitalWrite(pin.SWITCH_DISH_GRN, LOW);
	delayMicroseconds(100);
	// Test Pins
	digitalWrite(pin.TEST_SIGNAL, LOW);

	// SET INPUT PINS

	// Power
	pinMode(pin.PWR_SWITCH, INPUT);
	// XBees
	pinMode(pin.XB_CTS_F, INPUT);
	pinMode(pin.XB_CTS_R, INPUT);
	// Teensy
	pinMode(pin.TEENSY_RESET, INPUT);
	// Battery monitor
	pinMode(pin.BAT_VCC, INPUT);
	// IR proximity sensors
	pinMode(pin.IRPROX_R, INPUT);
	pinMode(pin.IRPROX_L, INPUT);
	// IR detector
	pinMode(pin.INTERUPT_IR_DETECT, INPUT);

	// Set power, button and switch internal pullup
	pinMode(pin.PWR_SWITCH, INPUT_PULLUP);
	pinMode(pin.BTN_1, INPUT_PULLUP);
	pinMode(pin.BTN_2, INPUT_PULLUP);
	pinMode(pin.BTN_3, INPUT_PULLUP);
	pinMode(pin.SWITCH_DISH, INPUT_PULLUP);
	delayMicroseconds(100);
}

#pragma endregion 


#endif
