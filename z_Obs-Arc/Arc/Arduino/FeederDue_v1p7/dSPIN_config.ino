void dSPINConfig_board()
{

  // AutoDriver Settings
  board.configSyncPin(BUSY_PIN, 0); // Set busy pin as BUSY_PIN or SYNC_PIN;
                                    //  SYNC_FS_2 - two pulses on sync pin per full step of motor        
                                    //  SYNC_FS - one pulse per full step
                                    //  SYNC_XFS - where X can be 2, 4, 8, 16, 32, or 64, and X indicates the number of full steps between pulses on the sync pin 
  board.setSlewRate(SR_530V_us); // SR_180V_us, SR_290V_us, SR_530V_us
                                   // The slew rate is the slope of the voltage change coming out of the driver. 
                                   // Higher slew rates increase the torque at higher speeds                                             

  // Settings inluced with library                                  
  board.setParam(STEP_MODE,STEP_FS_128);  // Microsteps per step
                                          // STEP_FS - Full-step mode; microstepping disabled   
                                          // STEP_FS_X - Enable microstepping with X microsteps per full step. X can be 2, 4, 8, 16, 32, 64, or 128.
  board.setSlewRate(SR_530V_us);          // Upping the edge speed increases torque.
                                          // SR_180V_us, SR_290V_us, SR_530V_us
  board.setOCThreshold(OC_6000mA);        // Overcurrent threshold
                                          // 375, 750, 1125, 1500, 1875, 2250, 2625, 3000, 
                                          // 3375, 3750, 4125, 4500, 4875, 5250, 5625, or 6000
  board.setMaxSpeed(400);                 // 500 steps/s max
  board.setMinSpeed(0);                   // No minimum speed
  board.setFullSpeed(400);                // Microstep below 500 steps/s
  board.setAcc(50);                      // Accelerate at maximum steps/s/s; 0xFFF = infinite
  board.setDec(50);                      // Deccelerate at maximum steps/s/s; 0xFFF = infinite
  board.setPWMFreq(PWM_DIV_2, PWM_MUL_2); // 31.25kHz PWM freq
  board.setOCThreshold(OC_3000mA);        // 375, 750, 1125, 1500, 1875, 2250, 2625, 3000, 3375, 3750, 4125, 4500, 4875, 5250, 5625, 6000
  board.setOCShutdown(OC_SD_ENABLE);     // shutdown on OC (ENABLE CAUSES MOTOR TO STALL)
  board.setVoltageComp(VS_COMP_ENABLE);   // Motor V compensation
  board.setSwitchMode(SW_USER);           // Switch is not hard stop
  board.setOscMode(INT_16MHZ_OSCOUT_16MHZ); // Clock settings
  board.setAccKVAL(100);                  // This controls the acceleration current
  board.setDecKVAL(100);                  // This controls the deceleration current
  board.setRunKVAL(100);                  // This controls the run current
  board.setHoldKVAL(35);                  // This controls the holding current; keep it low.
  board.setLoSpdOpt(false);               // Enabled low speed compensation. If enabled, MinSpeed is upper threshold at which this compensation is employed.
}
