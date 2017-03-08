// LIBRARIES
#include <string.h>
// AutoDriver
#include <SPI.h>
#include "AutoDriver_Due.h"
// Pixy
#include <Wire.h>
#include <PixyI2C.h>
// LCD
#include <LCD5110_Graph.h>
// TinyEKF
#define N 1     // State value
#define M 2     // Measurements
#include <TinyEKF.h>

// DEFINE PINS
// Autodriver
const int pin_AD_CSP = 5;
const int pin_AD_Busy = 4;
const int pin_AD_Reset = 3;
// Display
const int pin_Disp_Power = A0;
const int pin_Disp_SCK = 8;
const int pin_Disp_MOSI = 9;
const int pin_Disp_DC = 10;
const int pin_Disp_RST = 11;
const int pin_Disp_CS = 12;
const int pin_Disp_LED = 13;
// LEDs
const int pin_TrackLED = 6;
const int pin_RewLED = 7;
// Relays
const int pin_Rel_1 = 24;
const int pin_Rel_2 = 25;
// IR Senosors
const int pin_IR_Rt = 22;
const int pin_IR_Lft = 23;
// Buttons
const int pin_Btn[4] = { A4, A3, A2, A1 };
volatile long intDebounce[4] = { millis(), millis(), millis(), millis() };

// VARIABLE SETUP
// Serial parsing
String xbeeIn; // serial string from C#
int colonInd;
int cammaInd[2];
// Serial VT
int vtID;
float vtRad[2];
int vtTS[2];
const float vtTarg = 0.78; // (rad)
float vtPosError; // (cm)
// Reward
const long solDir = 1000; // (ms) on duration
volatile byte solState = HIGH;
volatile byte rewNow = false;
long rewStopTim = millis();
// LEDs
const int rewLEDDuty = 50; // value between 0 and 255
const int trackLEDDuty = 75; // value between 0 and 255
// LCD
extern unsigned char SmallFont[];
extern unsigned char TinyFont[];
volatile byte lndLedOn = false;
long printNextTim = millis();
// Pixy
const float pixyTarg = 18;
const float pixyCoeff[5] = {
  0.000000064751850,
  -0.000029178819914,
  0.005627883140337,
  -0.690663759936117,
  38.949828090278942
};
float pixyLastTS;
float pixyDeltaTS;
float pixyPosY;
float pixyPosCM;
float pixyPosError;
// Other
float posErrorEstimate;

// INITILIZE OBJECTS
// AutoDriver
AutoDriver_Due board(pin_AD_CSP, pin_AD_Reset, pin_AD_Busy);
// Pixy

// LCD

void setup() {
	
	// Wait for connection
	while (!Serial);
	// USB/Serial_Monitor
	SerialUSB.begin(0); // does not matter
						// XBee
	Serial1.begin(57600);

	

}

void loop() {

	digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);              // wait for a second
	digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);              // wait for a second
}

