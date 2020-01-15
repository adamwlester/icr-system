//
// Calibrate pixy by hand using stick on cm template
// Sample from 0 to 28 cm mark on cm template
//

#include <Wire.h>
#include <PixyI2C.h>
#include "FeederDue_PinMap.h"

PixyI2C pixy(0x54); // You can set the I2C address through PixyI2C object

// Set what to print
bool printY = false;
bool printCM = true;
bool printSpeed = false;

// Declare vars

// Coeff calculated using CalibrateCurveFit.m
const double coeff[5] = {
	0.000000015540370,
	-0.000012238336108,
	0.003550674880560,
	-0.695456876361152,
	90.475818467619405,
};

int yNow = NULL;
int cmNow = NULL;
int cmLast = NULL;
const int ySampN = 10;
float spArr[ySampN];
float spDevArr[ySampN];
float spSum = 0;
float spMu = 0;
float spDevSum = 0;
float spDevMu = 0;
float spFltSum = 0;
float spFltMu = 0;
int cutSD = 3;
int cmArr[ySampN];
float cmFltSum = 0;
float cmFltMu = 0;
int yArr[ySampN];
float yFltSum = 0;
float yFltMu = 0;

void setup()
{
	// SETUP PRINTING
	SerialUSB.begin(0);
	SerialUSB.print("Starting...\n");

	// SETUP PINS
	SetupPins();

	// TURN ON POWER AND RELAYES

	// Voltage Regulators
	pinMode(pin.REG_24V_ENBLE, OUTPUT);
	pinMode(pin.REG_12V2_ENBLE, OUTPUT);
	pinMode(pin.REG_5V1_ENBLE, OUTPUT);
	
	// Voltage Regulators
	digitalWrite(pin.REG_24V_ENBLE, LOW);
	digitalWrite(pin.REG_12V2_ENBLE, HIGH);
	digitalWrite(pin.REG_5V1_ENBLE, HIGH);
	
	// INIT PIXY STUFF
	pixy.init();
	Wire.begin();
	memset(spArr, NULL, sizeof(spArr));
}


void loop()
{
	delay(100);

	uint16_t blocks;
	static int i = 0;
	static int k = 0;

	// get new data
	blocks = pixy.getBlocks();

	if (blocks)
	{

		cmLast = cmNow;
		yNow = pixy.blocks[blocks - 1].y;

		// Transfor Y pxl to cm
		cmNow =
			coeff[0] * (yNow * yNow * yNow * yNow) +
			coeff[1] * (yNow * yNow * yNow) +
			coeff[2] * (yNow * yNow) +
			coeff[3] * (yNow) +
			coeff[4];

		// Fill array
		if (cmLast != NULL && i < ySampN) {
			spArr[i] = (cmNow - cmLast) * 50;
			cmArr[i] = cmNow;
			yArr[i] = yNow;
			i++;
		}

		// Start shifting array once full
		else {
			// shift old data
			for (int j = 0; j <= (ySampN - 1) - 1; j++) {
				spArr[j] = spArr[j + 1];
				cmArr[j] = cmArr[j + 1];
				yArr[j] = yArr[j + 1];
			}
			// convert to speed and add new data
			spArr[ySampN - 1] = (cmNow - cmLast) * 50;
			cmArr[ySampN - 1] = cmNow;
			yArr[ySampN - 1] = yNow;

			// Compute mean
			for (int j = 0; j <= (ySampN - 1); j++) {
				spSum = spSum + spArr[j];
			}
			spMu = spSum / ySampN;
			spSum = 0;
			// Compute deviation
			for (int j = 0; j <= (ySampN - 1); j++) {
				spDevArr[j] = (spArr[j] - spMu) * (spArr[j] - spMu);
			}
			// Compute mean deviation
			for (int j = 0; j <= (ySampN - 1); j++) {
				spDevSum = spDevSum + spDevArr[j];
			}
			spDevMu = sqrt(spDevSum / ySampN);
			spDevSum = 0;

			// Remove outlyers and compute mean speed
			int k = 0;
			for (int j = 0; j <= (ySampN - 1); j++) {
				if (abs(spArr[j]) <= cutSD * abs(spDevMu)) {
					spFltSum = spFltSum + spArr[j];
					cmFltSum = cmFltSum + cmArr[j];
					yFltSum = yFltSum + yArr[j];
					k++;
				}
			}
			spFltMu = spFltSum / k;
			cmFltMu = cmFltSum / k;
			yFltMu = yFltSum / k;
			spFltSum = 0;
			cmFltSum = 0;
			yFltSum = 0;

			delay(50);

			// Print speed
			char msg[50];
			if (printSpeed)
			{
				sprintf(msg, "Speed: %0.2f\n", spFltMu);
				SerialUSB.print(msg);
			}
			// Print pos in CM
			else if (printCM)
			{
				sprintf(msg, "CM: %0.2f\n", cmFltMu);
				SerialUSB.print(msg);
			}
			// Print pos in Y
			else if (printY)
			{
				float yRound = round(yFltMu * 10.0) / 10.0;
				sprintf(msg, "Y: %0.1f\n", yRound);
				SerialUSB.print(msg);
			}
		}
	}
}

