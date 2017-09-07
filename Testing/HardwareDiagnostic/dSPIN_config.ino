void dSPINConfig_board()
{

#pragma region ---------BOARD SETTINGS---------

	// Set busy pin as BUSY_PIN or SYNC_PIN;
	AD_R.configSyncPin(BUSY_PIN, 0);
	AD_F.configSyncPin(BUSY_PIN, 0);
	//  SYNC_FS_2 - two pulses on sync pin per full step of motor        
	//  SYNC_FS - one pulse per full step
	//  SYNC_XFS - where X can be 2, 4, 8, 16, 32, or 64, and X indicates the number of full steps between pulses on the sync pin 

	// Microsteps per step
	AD_R.setParam(STEP_MODE, STEP_FS_128);
	AD_F.setParam(STEP_MODE, STEP_FS_128);
	// STEP_FS - Full-step mode; microstepping disabled   
	// STEP_FS_X - Enable microstepping with X microsteps per full step. X can be 2, 4, 8, 16, 32, 64, or 128.

	// PWM freq
	AD_R.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq
	AD_F.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq		
												//  PWM_DIV_X, where X can be any value 1-7.
												//  PWM_MUL_X, where X can be 0_625 (for 0.625), 0_75 (for 0.75), 0_875, 1, 1_25, 1_5, 1_75, or 2.

												// Overcurent enable
	AD_R.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC
	AD_F.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC

												// Motor V compensation
	AD_R.setVoltageComp(VS_COMP_ENABLE);
	AD_F.setVoltageComp(VS_COMP_ENABLE);
	// VS_COMP_ENABLE, VS_COMP_DISABLE

	// Switch pin mode
	AD_R.setSwitchMode(SW_USER);				// Switch is not hard stop
	AD_F.setSwitchMode(SW_USER);				// Switch is not hard stop

												// Slew rate
	AD_R.setSlewRate(SR_530V_us);
	AD_F.setSlewRate(SR_530V_us);
	// Upping the edge speed increases torque
	// SR_180V_us, SR_290V_us, SR_530V_us

	// Overcurrent threshold
	AD_R.setOCThreshold(OC_4875mA);
	AD_F.setOCThreshold(OC_3750mA);
	// Peak Amp for 1.2 A stepper = 1.2*1.41 = 1690 mA
	// Peak Amp for 1.7 A stepper = 1.7*1.41 = 2397 mA
	// Peak Amp for 2.82 A stepper = 2.82*1.41 = 3.97 mA
	// 375, 750, 1125, 1500, 1875, 2250, 2625, 3000, 
	// 3375, 3750, 4125, 4500, 4875, 5250, 5625, or 6000

	// Low speed compensation
	AD_R.setLoSpdOpt(true);
	AD_F.setLoSpdOpt(true);
	// Enabled low speed compensation. If enabled, MinSpeed is upper threshold at which this compensation is employed.

#pragma endregion 

#pragma region ---------SPEED SETTTINGS---------

	// Steps/s max
	AD_R.setMaxSpeed(maxSpeed * cm2stp);
	AD_F.setMaxSpeed(maxSpeed * cm2stp);

	// Minimum speed
	AD_R.setMinSpeed(10 * cm2stp);
	AD_F.setMinSpeed(10 * cm2stp);

	// Full speed
	AD_R.setFullSpeed(maxSpeed * cm2stp);
	AD_F.setFullSpeed(maxSpeed * cm2stp);

	// Acceleration
	AD_R.setAcc(maxAcc * cm2stp);
	AD_F.setAcc(maxAcc * cm2stp);
	// Accelerate at maximum steps/s/s; 0xFFF = infinite

	// Deceleration
	AD_R.setDec(maxDec * cm2stp);
	AD_F.setDec(maxDec * cm2stp);
	// Deccelerate at maximum steps/s/s; 0xFFF = infinite

#pragma endregion 

#pragma region ---------KVAL SETTTINGS---------
	// K Val settings
	// KVAL = [(KVAL_X + BEMF_COMP) * VSCOMP * K_THERM] * microstep
	// KVAL = Rm * Iph / Vs = %
	// Pololu item #: 1200: 1.2A, 4V, 3.3Ohm, 2.8mH = 84.15
	// Pololu item #: 2267: 1.68A, 2.8V, 1.65Ohm, 3.2mH = 58.9
	// AA item #: 23Y108D-LW8: 2.82A, 2.82V, 1.65Ohm, 3.2mH = 58.9

	// NIMA 23 24V MIN KVALS
	AD_R.setAccKVAL(50);				        // This controls the acceleration current
	AD_R.setDecKVAL(50);				        // This controls the deceleration current
	AD_R.setRunKVAL(50);					    // This controls the run current
	AD_R.setHoldKVAL(20);				        // This controls the holding current keep it low

												// NIMA 17 24V
	AD_F.setAccKVAL(50);				        // This controls the acceleration current
	AD_F.setDecKVAL(50);				        // This controls the deceleration current
	AD_F.setRunKVAL(50);					    // This controls the run current
	AD_F.setHoldKVAL(20);				        // This controls the holding current keep it low

												/*
												// NIMA 17 12V
												AD_F.setAccKVAL(100);				        // This controls the acceleration current
												AD_F.setDecKVAL(100);				        // This controls the deceleration current
												AD_F.setRunKVAL(120);					    // This controls the run current
												AD_F.setHoldKVAL(35);				        // This controls the holding current keep it low
												*/


#pragma endregion

}