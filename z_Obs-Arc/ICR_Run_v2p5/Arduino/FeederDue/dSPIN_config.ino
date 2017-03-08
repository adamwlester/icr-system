void dSPINConfig_board()
{

	// AutoDriver Settings
	board.configSyncPin(BUSY_PIN, 0);			// Set busy pin as BUSY_PIN or SYNC_PIN;
													//  SYNC_FS_2 - two pulses on sync pin per full step of motor        
													//  SYNC_FS - one pulse per full step
													//  SYNC_XFS - where X can be 2, 4, 8, 16, 32, or 64, and X indicates the number of full steps between pulses on the sync pin 
	board.setParam(STEP_MODE, STEP_FS_128);		// Microsteps per step
												// STEP_FS - Full-step mode; microstepping disabled   
												// STEP_FS_X - Enable microstepping with X microsteps per full step. X can be 2, 4, 8, 16, 32, 64, or 128.
	board.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq
												    //  PWM_DIV_X, where X can be any value 1-7.
													//  PWM_MUL_X, where X can be 0_625 (for 0.625), 0_75 (for 0.75), 0_875, 1, 1_25, 1_5, 1_75, or 2.
	board.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC (ENABLE CAUSES MOTOR TO STALL SOMETIMES)
	board.setVoltageComp(VS_COMP_ENABLE);		// Motor V compensation
												    // VS_COMP_ENABLE, VS_COMP_DISABLE
	board.setSwitchMode(SW_USER);				// Switch is not hard stop
	board.setSlewRate(SR_530V_us);				// Upping the edge speed increases torque.
												    // SR_180V_us, SR_290V_us, SR_530V_us
	board.setOCThreshold(OC_3000mA);			// Overcurrent threshold
													// Peak Amp for 1.2 A stepper = 1.2*1.41 = 1690 mA
													// Peak Amp for 1.7 A stepper = 1.7*1.41 = 2397 mA
													// 375, 750, 1125, 1500, 1875, 2250, 2625, 3000, 
													// 3375, 3750, 4125, 4500, 4875, 5250, 5625, or 6000
	board.setLoSpdOpt(true);					// Enabled low speed compensation. If enabled, MinSpeed is upper threshold at which this compensation is employed.
	// Speed settings
	board.setMaxSpeed(maxSpeed*cm2stp);			// Steps/s max
	board.setMinSpeed(10*cm2stp);				// No minimum speed
	board.setFullSpeed(maxSpeed*cm2stp);		// Microstep below 500 steps/s
	board.setAcc(maxAcc*cm2stp);				// Accelerate at maximum steps/s/s; 0xFFF = infinite
	board.setDec(maxDec*cm2stp);				// Deccelerate at maximum steps/s/s; 0xFFF = infinite
	// K Val settings
		// KVAL = [(KVAL_X + BEMF_COMP) * VSCOMP * K_THERM] * microstep
		// KVAL = Rm * Iph / Vs = %
		// Pololu item #: 1200: 1.2A, 4V, 3.3Ohm, 2.8mH = 84.15
		// Pololu item #: 2267: 1.68A, 2.8V, 1.65Ohm, 3.2mH = 58.9
	board.setAccKVAL(100);				       // This controls the acceleration current
	board.setDecKVAL(100);				       // This controls the deceleration current
	board.setRunKVAL(100);					   // This controls the run current
	board.setHoldKVAL(35);				   // This controls the holding current keep it low
}