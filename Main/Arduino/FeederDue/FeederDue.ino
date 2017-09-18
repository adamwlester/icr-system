// ######################################

// ============= FEEDERDUE ==============

// ######################################

// NOTES:
/*

* XBee DI (from UART tx) buffer = 202 bytes or 100 bytes (maximum packet size)

* XBee DO (to UART rx) buffer = 202 bytes

* ARDUINO SERIAL_BUFFER_SIZE CHANGED FROM 128 TO 512
	Path: "C:\Users\lester\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.8\cores\arduino\RingBuffer.h"

* SerialUSB receive buffer size is now 512 (ARDUINO 1.5.2 BETA - 2013.02.06)

* DATA TYPES:
	byte = 1 byte
	char = 1 byte
	int = 4 byte
	long = 4 byte
	float = 4 byte
	double = 8 byte

* Step down resistor for vcc monitoring:
	To ground = 2.2k Ohm
	To vcc = 8.2k Ohm

* config.txt settings:
	57600,26,3,2,0,0,0
	baud,escape,esc#,mode,verb,echo,ignoreRX

*/


#pragma region ========== LIBRARIES & EXT DEFS =========

//----------LIBRARIES------------

// General
#include <string.h>


// Memory
#include <MemoryFree.h>

// Timers
#include <DueTimer.h>

// AutoDriver

#include <SPI.h>
//
#include "AutoDriver_Due.h"

// Pixy

#include <Wire.h> 
//
#include <PixyI2C.h>

// LCD
#include <LCD5110_Basic.h>

// TinyEKF
#define N 4     // States
#define M 6     // Measurements
#include <TinyEKF.h>

// FeederDue
#include "FeederDue.h"

//-------SOFTWARE RESET----------
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

#pragma endregion 


#pragma region =========== CLASS DEFINITIONS ===========

#pragma region ----------CLASS: POSTRACK----------

POSTRACK::POSTRACK(uint32_t t, char obj_id[], int n_samp)
{
	this->t_init = t;
	strcpy(this->instID, obj_id);
	this->nSamp = n_samp;

	for (int i = 0; i < n_samp; i++) {
		this->posArr[i] = 0.0f;
	}
}

void POSTRACK::UpdatePos(double pos_new, uint32_t ts_new)
{
	// Local vars
	char str[300] = { 0 };
	double pos_diff = 0;
	double dist = 0;
	double dist_sum = 0;
	int dt = 0;
	int dt_sum = 0;
	double dt_sec = 0;
	double vel_diff = 0;
	double vel = 0;

	// Store input
	this->t_msNow = millis();
	this->posAbs = pos_new;
	this->t_tsNow = ts_new;

	// Update itteration count
	this->sampCnt++;

	// Shift and add data
	for (int i = 0; i < this->nSamp - 1; i++)
	{
		this->posArr[i] = this->posArr[i + 1];
		this->t_tsArr[i] = this->t_tsArr[i + 1];
	}
	// Add new variables
	this->posArr[this->nSamp - 1] = pos_new;
	this->t_tsArr[this->nSamp - 1] = ts_new;

	// Do not process early samples
	if (this->sampCnt < this->nSamp + 1) {

		this->posNow = this->posNow;
		this->velNow = 0;

		// Set flag
		this->isNew = false;

		// Bail 
		return;
	}
	// Ready to use
	else {
		this->isNew = true;
	}

	// COMPUTE TOTAL DISTANCE RAN

	// Check for zero crossing
	pos_diff = pos_new - this->posArr[this->nSamp - 2];
	if (abs(pos_diff) > 140 * PI * 0.75) {

		// Crossed over
		if (pos_diff < 0) {
			this->nLaps++;
		}

		// Crossed back
		else {
			this->nLaps--;
		}
	}

	// Store cumulative position in cm
	this->posNow = pos_new + this->nLaps*(140 * PI);

	// COMPUTE VELOCITY
	for (int i = 0; i < this->nSamp - 1; i++)
	{
		// Compute sample distance
		dist = this->posArr[i + 1] - this->posArr[i];

		// Correct for zero crossing
		if (abs(dist) > 140 * PI * 0.75) {
			dist = ((140 * PI) - abs(dist)) * abs(dist) == dist ? -1 : 1;
		}

		// Add to total
		dist_sum += dist;

		// Compute total time
		dt = this->t_tsArr[i + 1] - this->t_tsArr[i];
		dt_sum += dt;
	}

	// Convert dt to sec 
	dt_sec = (double)dt_sum / 1000;

	// Compute vel
	vel = dist_sum / dt_sec;

	// Compute change in vel
	vel_diff = abs(this->velNow - vel);

	// Check for problem vals
	bool do_skip_vel =
		dt_sec == 0 ||
		(this->dt_skip < 500 && vel_diff > 300);

	// Ignore outlyer values unless too many frames discarted
	if (!do_skip_vel) {
		this->velLast = this->velNow;
		this->velNow = vel;
		this->dt_skip = 0;
	}
	// Add to skip time
	else {
		this->dt_skip += dt;
	}

	// Check for errors
	bool is_error =
		dt_sec == 0 ||
		vel_diff > 300;

	// Log/print error
	if (is_error) {

		// Add to count
		this->cnt_error++;

		// Log/print first and every 10 errors
		if (this->cnt_error == 1 ||
			this->cnt_error % 10 == 0) {

			// Log/print error
			sprintf(str, "**WARNING** [POSTRACK::UpdatePos] Bad Values |%s%s: obj=\"%s\" cnt_err=%d pos_new=%0.2f pos_last=%0.2f dist_sum=%0.2f dt_sec=%0.2f vel_new=%0.2f vel_last=%0.2f",
				vel_diff > 300 ? "Vel|" : "", dt_sec == 0 ? "DT|" : "", this->instID, this->cnt_error, pos_new, this->posArr[this->nSamp - 2], dist_sum, dt_sec, vel, this->velLast);
			DebugError(str);
		}
	}
}

double POSTRACK::GetPos()
{
	this->isNew = false;
	return this->posNow;
}

double POSTRACK::GetVel()
{
	return this->velNow;
}

void POSTRACK::SwapPos(double set_pos, uint32_t t)
{
	// Make sure pos val range [0, 140*PI]
	set_pos = set_pos < 0 ? set_pos + (140 * PI) : set_pos;
	set_pos = set_pos > (140 * PI) ? set_pos - (140 * PI) : set_pos;

	// Compute ts
	uint32_t ts = this->t_tsNow + (t - this->t_msNow);

	// Update pos
	UpdatePos(set_pos, ts);
}

void POSTRACK::Reset()
{
	this->sampCnt = 0;
	this->isNew = false;
}

#pragma endregion 

#pragma region ----------CLASS: PID----------

PID::PID(uint32_t t, const float kC, const float pC, const float set_point)
{
	this->t_init = t;
	this->kP = 0.6 * kC; // proportional constant
	this->kI = 2 * kP / pC; // integral constant
	this->kD = kP*pC / 8; // derivative constant
	this->setPoint = set_point;
}

double PID::UpdatePID()
{

	// Wait till ekf ready
	if (!fc.isEKFReady) {
		return -1;
	}

	// Compute error 
	error = kal.RatPos - (kal.RobPos + setPoint);
	errorFeeder = kal.RatPos - (kal.RobPos + feedDist);

	// Check if motor is open
	CheckMotorControl();

	// Check throttling 
	CheckThrottle();

	// Check if in auto mode
	if (mode != "Automatic") {
		return -1;
	}

	// Check if rat stopped behind setpoint
	if (kal.RatVel < 1 && error < -15 && !isHolding4cross) {
		// halt running
		return runSpeed = 0;
	}

	// Check if throttling
	SetThrottle();

	// Check for setpoint crossing
	CheckSetpointCrossing();

	// Update Pid and speed
	if (
		!is_ekfNew || // New EKF data 
		isHolding4cross // wait for setpoint pass
		) {
		return -1;
	}

	// Re-compute error 
	error = kal.RatPos - (kal.RobPos + setPoint);

	// Compute new integral
	if (doIncludeTerm[0]) {

		// Catch setpoint (i.e. error == 0) crossing
		if ((abs(error) + abs(errorLast)) > abs(error + errorLast)) {

			integral = 0;
		}

		//else if (error < 0) integral = integral + error * 5;
		else {
			integral = integral + error;
		}
	}
	else {
		integral = 0;
	}

	// Compute new derivative
	if (doIncludeTerm[1]) {
		derivative = error - errorLast;
	}
	else {
		derivative = 0;
	}

	// Compute new terms
	p_term = kP*error;
	i_term = kI*dt_update*integral;
	d_term = kD / dt_update*derivative;
	//if (error < 0) d_term = d_term * 2.0f;

	// Compute updated vel
	velUpdate = p_term + i_term + d_term;

	// Get new run speed
	runSpeed = kal.RobVel + velUpdate;

	// Keep speed in range [0, speedMax]
	if (runSpeed > speedMax) {
		runSpeed = speedMax;
	}
	else if (runSpeed < 0) {
		runSpeed = 0;
	}

	errorLast = error;
	isPidUpdated = true;
	is_ekfNew = false;

	if (isFirstRun) {
		PrintPID("[PID::UpdatePID] First Run");
		isFirstRun = false;
	}

	// Return new run speed
	return runSpeed;

}

void PID::Run(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Take motor control
	SetMotorControl("Pid", "PID::Run");

	// Reset
	Reset();
	mode = "Automatic";

	// Tell ard pid is running
	QueuePacket(&r2a, 'p', 1);

	// Log/print event
	sprintf(str, "[PID::Run] Run [%s]", called_from);
	PrintPID(str);
}

void PID::Stop(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	if (fc.motorControl == "Pid")
	{
		// Stop movement
		RunMotor('f', 0, "Pid");

		// Set run speed
		runSpeed = 0;

		// Give over control
		SetMotorControl("Open", "PID::Stop");
	}

	// Tell ard pid is stopped if not rewarding
	if (!Reward.isRewarding) {
		QueuePacket(&r2a, 'p', 0);
	}

	// Set mode
	mode = "Manual";

	// Log/print event
	sprintf(str, "[PID::Stop] Stop [%s]", called_from);
	PrintPID(str);
}

void PID::Hold(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Call Stop
	Stop("Pid.Hold");

	// But set mode to "Hold"
	mode = "Hold";

	// Log/print event
	sprintf(str, "[PID::Hold] Hold [%s]", called_from);
	PrintPID(str);
}

void PID::Reset()
{
	integral = 0;
	t_updateLast = millis();
	isHolding4cross = true;
	doThrottle = true;
}

void PID::SetThrottle()
{
	// Local vars
	char str[200] = { 0 };

	if (!doThrottle) {
		return;
	}

	// Only run if rat ahead of setpoint
	if (
		error > 5 &&
		millis() > t_lastThrottle + 5000
		) {

		// Change acc to rat pos
		AD_Reset(maxSpeed, throttleAcc, maxDec);

		// Set time to throttle till
		t_throttleTill = millis() + dt_throttle;

		// Set flags
		isThrottled = true;
		doThrottle = false;

		// Log/print
		sprintf(str, "[PID::SetThrottle] Throttle ACC to %0.2fcm/sec", throttleAcc);
		PrintPID(str);
	}
	else {
		doThrottle = false;
	}

}

void PID::CheckThrottle()
{
	if (isThrottled) {

		// Check for criteria
		if (
			millis() >= t_throttleTill ||
			kal.RatVel > throttleSpeedStop ||
			error < 0
			) {

			// Set acc back to normal
			AD_Reset(maxSpeed, maxAcc, maxDec);

			// Store time
			t_lastThrottle = millis();

			// Reset flag
			isThrottled = false;
			PrintPID("[PID::CheckThrottle] Finished Throttle");
		}
	}
}

void PID::CheckMotorControl()
{
	// Check if motor control available
	if ((fc.motorControl == "Pid" || fc.motorControl == "Open") &&
		mode == "Hold") {

		// Print taking conrol
		PrintPID("[PID::CheckMotorControl] Take Motor Control [PID::CheckMotorControl]");

		// Run pid
		Run("PID::CheckMotorControl");
	}
	else if ((fc.motorControl != "Pid" && fc.motorControl != "Open") &&
		mode == "Automatic") {

		// Print taking conrol
		PrintPID("[PID::CheckMotorControl] Surender Motor Control [PID::CheckMotorControl]");

		// Hold pid
		Hold("PID::CheckMotorControl");
	}
}

void PID::CheckSetpointCrossing()
{
	// Check if rat has moved in front of setpoint
	if (isHolding4cross && error > 0)
	{
		isHolding4cross = false;
		PrintPID("[PID::CheckSetpointCrossing] Crossed Setpoint");
	}
}

void PID::CheckEKF(uint32_t t)
{
	if (!fc.isEKFReady)
	{
		if ((t - t_ekfReady) > dt_ekfSettle)
		{
			fc.isEKFReady = true;
			// Log/print
			PrintPID("[PID::CheckEKF] EKF Ready");
		}
	}
}

void PID::ResetEKF(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Set flag and time
	fc.isEKFReady = false;
	t_ekfReady = millis();

	// Log/print
	sprintf(str, "[PID::ResetEKF] Reset EKF [%s]", called_from);
	PrintPID(str);
}

void PID::SetUpdateTime(uint32_t t)
{
	is_ekfNew = true;
	if (isPidUpdated)
	{
		dt_update = (double)(t - t_updateLast) / 1000;
		isPidUpdated = false;
		t_updateLast = t;
	}
}

void PID::PrintPID(char msg[])
{
	// Add to print queue
	if (db.print_pid && (db.Console || db.LCD)) {
		QueueDebug(msg, millis());
	}
	// Add to log queue
	if (db.log_pid && db.Log) {
		Log.QueueLog(msg, millis());
	}
}

double PID::RunPidCalibration()
{

	/*
	Calibration based on the Ziegler–Nichols method:
	http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
	Equations:
	kP = 0.6*Kc
	Kd = 2*Kp * dT/Pc
	Ki = Kp*Pc / (8*dT)
	*/

	// Local vars
	char str[200] = { 0 };
	float pc_sum = 0;

	if (cal_isCalFinished) {
		return -1;
	}

	// End of calibration 
	if (
		cal_cntPcArr[3] == cal_nMeasPerSteps &&
		cal_PcArr[3] > 0
		) {
		// Compute overal average
		for (int i = 0; i < 4; i++)
		{
			pc_sum += cal_PcArr[i];
		}
		cal_PcAll = pc_sum / 4;

		// Set flags
		cal_isPidUpdated = true;
		cal_isCalFinished = true;

		// Log/print
		DebugFlow("[PID::RunPidCalibration] Finished PID Calibration");
		return 0;
	}

	// Bail if pos data not ready
	if (!is_ekfNew) {
		return -1;
	}

	// Check if ready to get new pc val
	if (millis() < t_updateLast + cal_dt_min) {
		return -1;
	}

	// Setup stuff
	if (cal_ratPos == 0) {
		cal_ratPos = kal.RobPos + setPoint;
		t_updateLast = millis();
		return -1;
	}

	// Check if speed should be incrimented
	if (cal_cntPcArr[cal_stepNow] == cal_nMeasPerSteps) {

		// Store values
		cal_PcArr[cal_stepNow] = cal_PcAvg;
		cal_PcSum = 0;
		cal_PcCnt = 0;
		cal_errArr[cal_stepNow] = cal_errAvg;
		cal_errSum = 0;
		cal_errCnt = 0;

		// Incriment step or bail if max reached
		if (cal_stepNow < 3) {
			cal_stepNow++;
		}
		else {
			return -1;
		}


		// Log/print
		sprintf(str, "[PID::RunPidCalibration] Set Speed to %0.2fcm/sec", cal_speedSteps[cal_stepNow]);
		DebugFlow(str);
	}

	// Get update dt
	cal_dt_update = (float)(millis() - t_updateLast) / 1000.0f;
	t_updateLast = millis();

	// Compute Pid
	cal_ratVel = cal_speedSteps[cal_stepNow];
	cal_ratPos += cal_ratVel * (cal_dt_update / 1);

	// Compute error 
	error = cal_ratPos - (kal.RobPos + setPoint);
	cal_errNow = error;

	// Compute new terms
	p_term = kC*error;

	// Get new run speed
	runSpeed = kal.RobVel + p_term;

	// Keep speed in range [0, speedMax]
	if (runSpeed > speedMax) {
		runSpeed = speedMax;
	}
	else if (runSpeed < 0) {
		runSpeed = 0;
	}

	// Catch occilation edge
	if (cal_errLast > 0 && error < 0) {

		// Update values
		cal_t_PcLast = cal_t_PcNow;
		cal_t_PcNow = millis();

		// Skip first period
		if (cal_t_PcLast > 0) {
			cal_PcCnt++;
			cal_cntPcArr[cal_stepNow]++;
			cal_PcNow = float(cal_t_PcNow - cal_t_PcLast) / 1000;
			cal_PcSum += cal_PcNow;
			cal_PcAvg = cal_PcSum / cal_PcCnt;
		}
	}
	cal_errCnt++;
	cal_errSum += abs(error);
	cal_errAvg = cal_errSum / cal_errCnt;
	cal_errMax = max(cal_errMax, error);
	cal_errMin = max(cal_errMin, error);

	// Update vars
	cal_errLast = error;
	cal_isPidUpdated = true;
	isPidUpdated = true;
	is_ekfNew = false;

	return runSpeed;

}

#pragma endregion 

#pragma region ----------CLASS: BULLDOZE----------

BULLDOZE::BULLDOZE(uint32_t t)
{
	this->t_init = t;
}

void BULLDOZE::UpdateBull()
{
	// Check who has motor control
	CheckMotorControl();

	// Bulldoze is on
	if (state == "On")
	{

		// Update when ready
		if (
			fc.isEKFReady &&
			millis() > t_updateNext)
		{

			// Update rat pos
			posNow = kal.RatPos;
			guardPos = kal.RobPos + guardDist;

			// Get distance traveled
			distMoved = posNow - posCheck;

			// Check for movement
			isMoved = distMoved >= moveMin ? true : false;

			// Check if rat passed reset
			double error = kal.RatPos - (kal.RobPos + pidSetPoint);
			isPassedReset = error > 1 ? true : false;

			// Check time
			isTimeUp = millis() > t_bullNext ? true : false;

			// Check if has not moved in time
			if (!isMoved) {

				// Bulldoze him!
				if (isTimeUp &&
					mode == "Inactive") {
					Run("BULLDOZE::UpdateBull");
				}
			}

			// Has moved minimal distance
			else {
				// Reset check pos
				posCheck = posNow;

				// Reset bull next
				t_bullNext = millis() + bDelay;

				// Stop bulldoze if rat ahead of set point and not 0 delay
				if (isPassedReset &&
					mode == "Active" &&
					bDelay != 0) {

					Stop("BULLDOZE::UpdateBull");
				}
			}

			// Set next update time
			t_updateNext = millis() + dt_update;
		}
	}
}

void BULLDOZE::Reinitialize(byte del, byte spd, char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Update vars
	bSpeed = (int)spd;
	bDelay = (int)del * 1000;
	t_bullNext = millis() + bDelay;
	posCheck = kal.RatPos;

	// Log/print event
	sprintf(str, "[BULLDOZE::Reinitialize] Reinitialize Bull [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Run(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Take control
	SetMotorControl("Bull", "BULLDOZE::Run");

	// Start bulldozer
	RunMotor('f', bSpeed, "Bull");

	// Tell ard bull is running
	QueuePacket(&r2a, 'b', 1);

	// Set mode
	mode = "Active";

	// Log/print event
	sprintf(str, "[BULLDOZE::Run] Run [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Stop(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Stop movement
	RunMotor('f', 0, "Bull");

	// Give over control
	if (fc.motorControl == "Bull") {

		// Stop movement
		RunMotor('f', 0, "Bull");

		// Give over control
		SetMotorControl("Open", "BULLDOZE::Stop");;
	}

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Tell ard bull is stopped
	QueuePacket(&r2a, 'b', 0);

	// Set mode
	mode = "Inactive";

	// Log/print event
	sprintf(str, "[BULLDOZE::Stop] Stop [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::TurnOn(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Change state
	state = "On";

	// Reset 
	Reset();

	// Log/print event
	sprintf(str, "[BULLDOZE::TurnOn] Turn On [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::TurnOff(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Change state
	state = "Off";

	// Stop bulldozer if running
	if (mode == "Active") {
		// Stop bull
		Stop("BULLDOZE::TurnOff");
	}

	// Log/print event
	sprintf(str, "[BULLDOZE::TurnOff] Turn Off [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Hold(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Change state
	state = "Hold";

	// Stop running
	if (mode == "Active") {

		// Run stop bulldozer
		Stop("BULLDOZE::Hold");

		// Set mode back to active for later
		mode = "Active";
	}

	// Log/print event
	sprintf(str, "bull: hold [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Resume(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Set state back to "On"
	state = "On";

	// Reset 
	Reset();

	// Log/print event
	sprintf(str, "[BULLDOZE::Resume] Resume [%s]", called_from);
	PrintBull(str);
}

void BULLDOZE::Reset()
{
	// Set mode
	mode = "Inactive";

	// Reset bull next
	t_bullNext = millis() + bDelay;

	// Reset check pos
	posCheck = kal.RatPos;
}

void BULLDOZE::CheckMotorControl()
{
	if ((fc.motorControl == "Bull" || fc.motorControl == "Pid" || fc.motorControl == "Open") &&
		state == "Hold") {

		// Turn bull on
		Resume("BULLDOZE::CheckMotorControl");
	}
	else if ((fc.motorControl != "Bull" && fc.motorControl != "Pid" && fc.motorControl != "Open") &&
		state == "On") {

		// Turn bull off
		Hold("BULLDOZE::CheckMotorControl");
	}
}

void BULLDOZE::PrintBull(char msg[])
{
	// Add to print queue
	if (db.print_bull && (db.Console || db.LCD)) {
		QueueDebug(msg, millis());
	}
	// Add to log queue
	if (db.log_bull && db.Log) {
		Log.QueueLog(msg, millis());
	}
}

#pragma endregion 

#pragma region ----------CLASS: MOVETO----------

MOVETO::MOVETO(uint32_t t)
{
	this->t_init = t;
}

bool MOVETO::CompTarg(double now_pos, double targ_pos)
{
	// Run only if targ not set
	if (isTargSet) {
		return isTargSet;
	}

	// Local vars
	char str[300] = { 0 };
	double move_diff = 0;
	int circ = 0;
	int pos = 0;

	// Get abort timeout
	if (t_tryTargSetTill == 0) {
		t_tryTargSetTill = millis() + targSetTimeout;
	}

	// Check if time out reached
	if (millis() > t_tryTargSetTill) {
		doAbortMove = true;
		// Log/print error
		sprintf(str, "!!ERROR!! [MOVETO::CompTarg] Timedout after %dms", targSetTimeout);
		DebugError(str, true);
		return isTargSet;
	}

	// Bail if ekf pos data not ready
	if (!fc.isEKFReady) {
		return isTargSet;
	}

	// Current absolute pos on track
	circ = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posAbs = (double)(pos % circ) / 100;

	// Diff and absolute distance
	move_diff = targ_pos - posAbs;
	move_diff = move_diff < 0 ? move_diff + (140 * PI) : move_diff;

	// Allow for reverse move
	if (fc.doAllowRevMove) {

		// Get minimum distance to target
		targDist = min((140 * PI) - abs(move_diff), abs(move_diff));

		// Set to negative for reverse move
		if ((move_diff > 0 && abs(move_diff) == targDist) ||
			(move_diff < 0 && abs(move_diff) != targDist)) {
			moveDir = 'f';
		}
		else {
			moveDir = 'r';
		}
	}

	// Only move forward
	else {
		targDist = move_diff;
		moveDir = 'f';
	}

	// Set vars for later
	t_updateNext = millis();
	baseSpeed = 0;

	// Copy to public vars
	targPos = targ_pos;
	posCumStart = now_pos;
	posAbsStart = posAbs;

	// Set flag true
	isTargSet = true;

	// Log/print
	sprintf(str, "[MOVETO::CompTarg] FINISHED: Set Target: start_cum=%0.2fcm start_abs=%0.2fcm targ=%0.2fcm dist_move=%0.2fcm move_dir=\'%c\'", posCumStart, posAbsStart, targPos, targDist, moveDir);
	DebugFlow(str);

	// Retern flag
	return isTargSet;
}

double MOVETO::DecelToTarg(double now_pos, double now_vel, double dist_decelerate, double speed_min)
{
	// Local vars
	char str[300] = { 0 };
	double new_speed = 0;

	// Run if targ not reached
	if (isTargReached) {
		return 0;
	}

	// Get abort timeout
	if (t_tryMoveTill == 0) {
		t_tryMoveTill = millis() + moveTimeout;
	}

	// Check if time out reached
	if (millis() > t_tryMoveTill) {
		doAbortMove = true;
		// Log/print error
		sprintf(str, "!!ERROR!! [MOVETO::DecelToTarg] Timedout after %dms", moveTimeout);
		DebugError(str, true);
		return 0;
	}

	// Compute remaining distance
	distLeft = targDist - abs(now_pos - posCumStart);

	// Check if rob is dec_pos cm from target
	if (distLeft <= dist_decelerate) {

		// Get base speed to decelerate from
		if (baseSpeed == 0) {
			baseSpeed = abs(now_vel);
		}

		// Update decel speed
		else if (millis() > t_updateNext) {
			// Compute new speed so range constrained to [min_speed, base_speed]
			new_speed = (((baseSpeed - speed_min) * distLeft) / dist_decelerate) + speed_min;

			// Get next update time
			t_updateNext = millis() + dt_update;
		}

	}

	// Target reached
	if (distLeft < 1)
	{
		// Set flag true
		isTargReached = true;

		// Stop movement
		new_speed = 0;

		// Log/print
		sprintf(str, "[MOVETO::DecelToTarg] FINISHED: MoveTo: start_abs=%0.2fcm now_abs=%0.2f targ=%0.2fcm dist_move=%0.2fcm dist_left=%0.2fcm",
			posAbsStart, now_pos, targPos, targDist, distLeft);
		DebugFlow(str);
	}

	// Return new speed
	return new_speed;
}

double MOVETO::GetError(double now_pos)
{
	// Local vars
	int diam = 0;
	int pos = 0;

	// Current relative pos on track
	diam = (int)(140 * PI * 100);
	pos = (int)(now_pos * 100);
	posAbs = (double)(pos % diam) / 100;

	// Target error
	return targPos - posAbs;
}

void MOVETO::Reset()
{
	isTargSet = false;
	isMoveStarted = false;
	isTargReached = false;
	doAbortMove = false;
	t_tryTargSetTill = 0;
	t_tryMoveTill = 0;
}

#pragma endregion 

#pragma region ----------CLASS: REWARD----------

REWARD::REWARD(uint32_t t)
{
	this->t_init = t;
	this->duration = durationDefault;
	this->durationByte = (byte)(duration / 10);
	Reset();
}

void REWARD::StartRew()
{
	// Local vars
	char str[200] = { 0 };

	// Set to extend feeder arm 
	ExtendFeedArm();

	// Hard stop
	HardStop("StartRew");

	// Set hold time
	BlockMotorTill(dt_rewBlock, "REWARD::StartRew");

	// Store and send packet imediately if coms setup
	if (fc.isSesStarted) {
		QueuePacket(&r2a, 'r', duration);
		SendPacket(&r2a);
	}

	// Turn on reward LED
	analogWrite(pin.RewLED_R, round(rewLEDduty*0.75));
	analogWrite(pin.RewLED_C, rewLEDduty);

	// Open solenoid
	digitalWrite(pin.Rel_Rew, HIGH);

	// Compute reward end time
	t_rew_str = millis();
	t_closeSol = t_rew_str + duration;

	// Compute retract arm time
	if (mode != "Button") {
		t_retractArm = t_rew_str + dt_rewBlock;
		doTimedRetract = true;
	}
	else {
		doTimedRetract = false;
	}


	// Print to LCD for manual rewards
	if (mode == "Button") {
		PrintLCD(true, "REWARDING...");
	}

	// Log/print 
	sprintf(str, "[REWARD::StartRew] RUNNING: \"%s\" Reward: dt_rew=%dms dt_retract=%d...",
		mode_str, duration, doTimedRetract ? t_retractArm - t_rew_str : 0);
	DebugFlow(str, t_rew_str);

	// Set flags
	isRewarding = true;

}

bool REWARD::EndRew()
{
	// Local vars
	char str[200] = { 0 };

	// Bail if not rewarding
	if (!isRewarding) {
		return false;
	}

	// Bail if time not up
	if (millis() < t_closeSol) {
		return false;
	}

	// Close solenoid
	digitalWrite(pin.Rel_Rew, LOW);

	// Turn off reward LED
	analogWrite(pin.RewLED_R, rewLEDmin);
	analogWrite(pin.RewLED_C, rewLEDmin);

	// Store time
	t_rew_end = millis();

	// Clear LCD
	if (mode == "Button") {
		ClearLCD();
	}

	// Log/print
	sprintf(str, "[REWARD::EndRew] FINISHED: \"%s\" Reward: dt_rew=%dms dt_retract=%d",
		mode_str, t_rew_end - t_rew_str, doTimedRetract ? t_retractArm - t_rew_str : 0);
	DebugFlow(str, t_rew_end);

	// Reset flags etc
	Reset();

	// Return end reward status
	return true;

}

void REWARD::SetRewDur(int zone_ind)
{
	// Local vars
	char str[200] = { 0 };

	// Set duration
	duration = zoneRewDurs[zone_ind];

	// Save zone ind
	zoneInd = zone_ind;

	// Log/print
	sprintf(str, "[REWARD::SetRewDur] Set Reward Diration: zone_ind=%d duration=%d",
		zone_ind, duration);
	DebugFlow(str);
}

void REWARD::SetRewMode(char mode_now[], int arg2)
{
	// NOTE: arg2 = reward delay or zone ind or reward duration

	// Local vars
	char str[200] = { 0 };
	char dat_str[100] = { 0 };

	// Store mode
	sprintf(mode_str, "%s", mode_now);
	mode = mode_now;

	// Store info
	if (mode == "Button") {

		// Set duration to default
		duration = durationDefault;
		sprintf(dat_str, "mode=\"Button\" duration=%d", duration);
	}
	else if (mode == "Now") {

		// Set duration
		SetRewDur(arg2);
		sprintf(dat_str, "mode=\"Now\" duration=%d", duration);

		// Change duration default
		durationDefault = duration;
	}
	else if (mode == "Free") {

		// Include all zones
		zoneMin = 0;
		zoneMax = zoneLng - 1;

		// Store threshold
		occThresh = arg2 * 1000;
		sprintf(dat_str, "mode=\"Free\" occ_thresh=%d", occThresh);
	}
	else if (mode == "Cue") {

		// Include one zone
		zoneMin = arg2;
		zoneMax = arg2;

		// Store zone ind
		occThresh = 0;
		sprintf(dat_str, "mode=\"Cue\" zone_ind=%d", arg2);
	}

	// Log/print
	sprintf(str, "[REWARD::SetRewMode] Set Reward Mode: %s", dat_str);
	DebugFlow(str);
}

bool REWARD::CompZoneBounds(double now_pos, double rew_pos)
{
	// Run only if bounds are not set
	if (!isBoundsSet)
	{
		// Local vars
		int diam = 0;
		int pos_int = 0;
		double pos_rel = 0;
		double dist_center_cm = 0;
		double dist_start_cm = 0;
		double dist_end_cm = 0;

		// Compute laps
		diam = (int)(140 * PI * 100);
		pos_int = (int)(now_pos * 100);
		lapN = round(now_pos / (140 * PI) - (float)(pos_int % diam) / diam);
		// Check if rat 'ahead' of rew pos
		pos_rel = (double)(pos_int % diam) / 100;
		// add lap
		lapN = pos_rel > rew_pos ? lapN + 1 : lapN;

		// Compute reward center
		rewCenterRel = rew_pos + lapN*(140 * PI);

		// Compute bounds for each zone
		for (int i = zoneMin; i <= zoneMax; i++)
		{
			// Compute 5 deg bounds
			dist_center_cm = -1 * zoneLocs[i] * ((140 * PI) / 360);
			dist_start_cm = dist_center_cm - (2.5 * ((140 * PI) / 360));
			dist_end_cm = dist_center_cm + (2.5 * ((140 * PI) / 360));
			// Store in array
			zoneBounds[i][0] = rewCenterRel + dist_start_cm;
			zoneBounds[i][1] = rewCenterRel + dist_end_cm;
		}
		// Save bound min/max
		boundMin = zoneBounds[zoneMin][0];
		boundMax = zoneBounds[zoneMax][0];
		// Set flag
		isBoundsSet = true;
	}
	return isBoundsSet;
}

bool REWARD::CheckZoneBounds(double now_pos)
{
	// Run only if reward not already triggered
	if (isZoneTriggered) {
		return isZoneTriggered;
	}

	// Bail if pos data not new
	if (!is_ekfNew) {
		return isZoneTriggered;
	}

	// Reset flag
	is_ekfNew = false;

	// Check if all bounds passed
	if (now_pos > boundMax + 5) {
		isAllZonePassed = true;
		return isZoneTriggered;
	}

	// Bail if first bound not reached
	if (now_pos < boundMin) {
		return isZoneTriggered;
	}

	// Check if rat in any bounds
	for (int i = zoneMin; i <= zoneMax; i++)
	{
		if (
			now_pos > zoneBounds[i][0] &&
			now_pos < zoneBounds[i][1]
			) {

			// Update timers
			t_lastZoneCheck = t_lastZoneCheck == 0 ? millis() : t_lastZoneCheck;
			t_nowZoneCheck = millis();

			// Store occupancy time
			zoneOccTim[i] += t_nowZoneCheck - t_lastZoneCheck;
			zoneOccCnt[i]++;
			t_lastZoneCheck = t_nowZoneCheck;

			// Check if occ thresh passed
			if (zoneOccTim[i] >= occThresh)
			{

				// REWARD at this pos
				SetRewDur(i);

				// Store reward info for debugging
				zoneRewarded = zoneLocs[i] * -1;
				boundsRewarded[0] = zoneBounds[i][0];
				boundsRewarded[1] = zoneBounds[i][1];
				occRewarded = zoneOccTim[i];

				// Set flag
				isZoneTriggered = true;
			}
		}
	}

	return isZoneTriggered;
}

void REWARD::ExtendFeedArm()
{
	// Step mode:
	/*
	MS1	MS2	MS3	Microstep Resolution	Excitation Mode
	L	L	L	Full Step	2 Phase
	H	L	L	Half Step	1 - 2 Phase
	L	H	L	Quarter Step	W1 - 2 Phase
	H	H	L	Eigth Step	2W1 - 2 Phase
	H	H	H	Sixteenth Step	4W1 - 2 Phase
	*/

	// Bail if arm already extended
	if (isArmExtended) {
		return;
	}

	// Block handler
	v_stepTimerActive = false;
	delayMicroseconds(500);

	// Set targ and flag
	v_stepTarg = armExtStps;
	doExtendArm = true;
	v_isArmMoveDone = false;

	// Wake motor
	digitalWrite(pin.ED_SLP, HIGH);

	// Make sure step low
	digitalWrite(pin.ED_STP, LOW);

	// Set to half step
	digitalWrite(pin.ED_MS1, HIGH);
	digitalWrite(pin.ED_MS2, LOW);
	digitalWrite(pin.ED_MS3, LOW);

	// Set direction to extend
	digitalWrite(pin.ED_DIR, LOW);
	v_stepDir = 'e';

	// Store start time
	t_moveArmStr = millis();

	// Set intterupt flag
	delayMicroseconds(500);
	v_stepTimerActive = true;

	// Log/print
	DebugFlow("[REWARD::ExtendFeedArm] Set Extend Feed Arm");

}

void REWARD::RetractFeedArm()
{
	// Step mode:
	/*
	MS1	MS2	MS3	Microstep Resolution	Excitation Mode
	L	L	L	Full Step	2 Phase
	H	L	L	Half Step	1 - 2 Phase
	L	H	L	Quarter Step	W1 - 2 Phase
	H	H	L	Eigth Step	2W1 - 2 Phase
	H	H	H	Sixteenth Step	4W1 - 2 Phase
	*/

	// Bail if arm not extended
	if (!isArmExtended) {
		return;
	}

	// Block handler
	v_stepTimerActive = false;
	delayMicroseconds(500);

	// Set targ and flag
	v_stepTarg = 0;
	doRetractArm = true;
	v_isArmMoveDone = false;

	// Wake motor
	digitalWrite(pin.ED_SLP, HIGH);

	// Make sure step low
	digitalWrite(pin.ED_STP, LOW);

	// Set to quarter step
	digitalWrite(pin.ED_MS1, LOW);
	digitalWrite(pin.ED_MS2, HIGH);
	digitalWrite(pin.ED_MS3, LOW);

	// Set direction to retract
	digitalWrite(pin.ED_DIR, HIGH);
	v_stepDir = 'r';

	// Store start time
	t_moveArmStr = millis();

	// Set intterupt flag
	delayMicroseconds(500);
	v_stepTimerActive = true;

	// Log/print
	DebugFlow("[REWARD::RetractFeedArm] Set Retract Feed Arm");

}

void REWARD::CheckFeedArm()
{
	// Local vars
	char str[200] = { 0 };
	bool is_move_done = false;
	bool is_timedout = false;

	// Do timed retract
	if (doTimedRetract) {

		// Check if its time to retract arm
		if (millis() > t_retractArm) {

			// Log/print
			sprintf(str, "[REWARD::CheckFeedArm] Time to Retract Feeder Arm: dt_rew=%d",
				millis() - t_rew_str);
			DebugFlow(str);

			// Set to retract
			RetractFeedArm();

			// Reset flag
			doTimedRetract = false;
		}
	}

	// Bail if nothing to do
	if (!doExtendArm &&
		!doRetractArm)
	{
		// Make sure motor asleep
		if (digitalRead(pin.ED_SLP) == HIGH) {
			// Sleep motor
			digitalWrite(pin.ED_SLP, LOW);
		}
		return;
	}

	// Check if done extending/retracting
	if ((doExtendArm || doRetractArm) &&
		v_isArmMoveDone) {

		is_move_done = true;
	}

	// Check if timedout
	else if (millis() > t_moveArmStr + armMoveTimeout) {

		is_timedout = true;
	}

	// Target reached
	if (is_move_done || is_timedout) {

		// Set intterupt flag
		v_stepTimerActive = false;

		// Unstep motor
		if (digitalRead(pin.ED_STP) == HIGH) {
			digitalWrite(pin.ED_STP, LOW);
		}

		// Sleep motor
		digitalWrite(pin.ED_SLP, LOW);

		// Set arm state flags
		if (doExtendArm) {
			isArmExtended = true;
		}
		else if (doRetractArm) {
			isArmExtended = false;
		}

		// Log/print status
		if (!is_timedout) {
			sprintf(str, "[REWARD::CheckFeedArm] SUCCEEDED: Arm %s: cnt_steps=%d step_targ=%d dt_move=%d",
				isArmExtended ? "Extend" : "Retract", v_cnt_steps, v_stepTarg, millis() - t_moveArmStr);
			DebugFlow(str);
		}
		else {
			sprintf(str, "!!ERROR!! [REWARD::CheckFeedArm] TIMEDOUT: Arm %s: cnt_steps=%d step_targ=%d dt_move=%d",
				isArmExtended ? "Extend" : "Retract", v_cnt_steps, v_stepTarg, millis() - t_moveArmStr);
			DebugError(str, true);
		}

		// Reset pos time and flags
		v_cnt_steps = 0;
		v_stepTarg = 0;
		t_moveArmStr = 0;
		isArmStpOn = false;
		doExtendArm = false;
		doRetractArm = false;
		v_isArmMoveDone = true;
	}
}

void REWARD::Reset()
{
	// Log zone info
	if (mode == "Free" || mode == "Cue")
	{
		char str1[200] = "[REWARD::Reset] ZONE OCC:";
		char str2[200] = "[REWARD::Reset] ZONE CNT:";
		for (int i = zoneMin; i <= zoneMax; i++)
		{
			char ss1[50];
			sprintf(ss1, " z%d=%dms", i + 1, zoneOccTim[i]);
			strcat(str1, ss1);
			char ss2[50];
			sprintf(ss2, " z%d=%d", i + 1, zoneOccCnt[i]);
			strcat(str2, ss2);
		}
		DebugFlow(str1);
		DebugFlow(str2);
	}

	// Reset flags etc
	sprintf(mode_str, "None");
	isRewarding = false;
	isBoundsSet = false;
	isZoneTriggered = false;
	isAllZonePassed = false;
	is_ekfNew = false;

	// Reset occ time
	for (int i = 0; i < zoneLng; i++) {
		zoneOccTim[i] = 0;
		zoneOccCnt[i] = 0;
	}
	t_nowZoneCheck = 0;
	t_lastZoneCheck = 0;
}

#pragma endregion 

#pragma region ----------CLASS: LOGGER----------

LOGGER::LOGGER(uint32_t t)
{
	this->t_init = t;
}

bool LOGGER::Setup()
{
	/*
	NOTE:
	config.txt settings:
	57600,26,3,2,0,0,0
	baud,escape,esc#,mode,verb,echo,ignoreRX
	*/

	// Local vars
	byte match = '\0';

	// Start serial
	port.begin(57600);

	// Reset OpenLog
	t_sent = millis();
	digitalWrite(pin.OL_RST, HIGH);
	delay(100);
	digitalWrite(pin.OL_RST, LOW);
	delay(100);
	match = GetReply(5000);

	// Bail if setup failed
	if (match != '>' && match != '<') {
		DebugError("!!ERROR!! [LOGGER::Setup] ABORTING: Did Not Get Initial \'>\' or \'<\'", true);
		return false;
	}

	// Set to command mode;
	if (match == '<') {
		if (!SetToCmdMode()) {
			return false;
		}
	}

	// Turn off verbose mode and echo mode
	SendCommand("verbose off\r");
	SendCommand("echo off\r");

	// Get settings
	if (SendCommand("get\r") == '!') {
		return false;
	}
	else {
		strcpy(openLgSettings, rcvdArr);
	}

	return true;
}

int LOGGER::OpenNewLog()
{
	// Local vars
	char str[200] = { 0 };
	byte match = 0;
	char new_file[50] = { 0 };

	// Make sure in command mode
	if (!SetToCmdMode())
		return 0;

	// Check/cd to log directory
	if (SendCommand("cd LOGS\r") == '!')
	{
		// Make new log dir
		if (SendCommand("md LOGS\r") == '!') {
			return 0;
		}
		if (SendCommand("cd LOGS\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGS\" Directory");

		// Make new log count file
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGCNT.TXT\" File");
	}

	// Get log count
	else if (SendCommand("read LOGCNT.TXT\r") != '!') {
		// Store count
		logNum = atoi(logCntStr) + 1;
	}
	else {
		return 0;
	}


	// Check if more than 100 logs saved
	if (logNum > 100)
	{
		// Step out of directory
		if (SendCommand("cd ..\r") == '!') {
			return 0;
		}

		// Delete directory
		if (SendCommand("rm -rf LOGS\r") == '!') {
			return 0;
		}

		// Print 
		sprintf(str, "[OpenNewLog] Deleted Log Directory: log_count=%d", logNum);
		DebugFlow(str);

		// Make new log dir
		if (SendCommand("md LOGS\r") == '!') {
			return 0;
		}
		if (SendCommand("cd LOGS\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGS\" Directory");

		// Make new log count file
		logNum = 1;
		if (SendCommand("new LOGCNT.TXT\r") == '!') {
			return 0;
		}
		DebugFlow("[OpenNewLog] Made \"LOGCNT.TXT\" File");
	}

	// Update count
	if (SendCommand("write LOGCNT.TXT\r") == '!') {
		return 0;
	}
	// write int string
	sprintf(str, "{{%d}}\r", logNum);
	SendCommand(str);
	// exit with empty line
	str[0] = '\r';
	str[1] = '\0';
	if (SendCommand(str) == '!') {
		return 0;
	}

	// Create new log file
	sprintf(new_file, "LOG%05u.CSV", logNum);

	// Begin logging to this file
	if (!SetToWriteMode(new_file)) {
		return 0;
	}

	// Write first log entry
	if (db.Log) {
		sprintf(str, "[OpenNewLog] Begin Logging to \"%s\"", logFile);
		QueueLog(str);
	}

	// Return log number
	return logNum;

}

bool LOGGER::SetToCmdMode()
{
	// Local vars
	char str[200] = { 0 };
	bool pass = false;
	char rstArr[3] = { 26,26,26 };

	// Check if already in cmd mode
	if (mode == '>') {
		return true;
	}

	// Add delay if command just sent
	int del = 100 - (millis() - t_sent);
	if (del > 0) {
		delay(del);
	}

	// Add delay if log just sent
	del = dt_write - (micros() - t_write);
	if (del > 0) {
		delayMicroseconds(del);
	}

	//Send "$$$" command mode
	if (SendCommand(rstArr, true, 500) == '>')
	{
		// Set flag
		pass = true;
		// Pause to let OpenLog get its shit together
		delay(100);
		// Print status
		sprintf(str, "[LOGGER::SetToCmdMode] OpenLog Set to Cmd Mode: mode = %c", mode);
		DebugFlow(str);
	}
	// Log/print error
	else {
		pass = false;
		sprintf(str, "**WARNING** [LOGGER::SetToCmdMode] ABORTED: mode=%c", mode);
		DebugError(str);
	}

	return pass;
}

void LOGGER::GetCommand()
{
	// Local vars
	char str[300] = { 0 };
	static int byte_ind_out = 0;

	// Get terminal command
	if (SerialUSB.available() > 0)
	{
		byte_ind_out = 0;
		while (SerialUSB.available() > 0)
		{
			str[byte_ind_out] = SerialUSB.read();
			byte_ind_out++;
			delayMicroseconds(100);
		}

		// Print command
		str[byte_ind_out] = '\0';
		SendCommand(str);

	}
}

char LOGGER::SendCommand(char msg[], bool do_conf, uint32_t timeout)
{
	// Local vars
	char str[200] = { 0 };
	char reply = '\0';
	char msg_copy[300] = { 0 };

	// Add min delay
	int del = 15 - (millis() - t_sent);
	if (del > 0) {
		delay(del);
	}

	// Copy message
	for (int i = 0; i < strlen(msg); i++)
	{
		msg_copy[i] = msg[i];
	}
	msg_copy[strlen(msg)] = '\0';

	// Send
	port.write(msg_copy);
	t_sent = millis();

	// Print sent
	if (db.print_a2o)
	{
		PrintLOGGER("SENT[=======================", true);
		for (int i = 0; i < strlen(msg_copy) + 1; i++) {
			PrintLOGGER(PrintSpecialChars(msg_copy[i]));
		}
		PrintLOGGER("\n=======================]SENT\n\n");
	}

	// Get confirmation
	if (do_conf) {
		reply = GetReply(timeout);
		// Check for error
		if (reply == '!') {
			// Remove '\r'
			msg_copy[strlen(msg_copy) - 1] = msg_copy[strlen(msg_copy) - 1] == '\r' ? '\0' : msg_copy[strlen(msg_copy) - 1];
			sprintf(str, "**WARNING** [LOGGER::SendCommand] Command %s Failed", msg_copy);
			PrintLOGGER(str, true);
		}
	}
	else {
		reply = mode;
	}

	// Return mode
	return reply;
}

char LOGGER::GetReply(uint32_t timeout)
{
	// Local vars
	char str[300] = { 0 };
	uint32_t t_start = millis();
	uint32_t t_timeout = millis() + timeout;
	int dat_ind[2] = { 0,0 };
	int arr_ind = -1;
	char cmd_reply = ' ';
	bool pass = false;

	// Wait for new data
	while (
		port.available() == 0 &&
		millis() < t_timeout
		);
	t_rcvd = millis();

	// Check for match byte
	if (db.print_o2a) {
		PrintLOGGER("[LOGGER::GetReply] RCVD_FORMATED[==========", true);
	}
	while (
		!pass &&
		millis() < t_timeout
		)
	{
		// Get new data
		if (port.available() > 0)
		{
			// Get next byte
			char c = port.read();

			// Print new byte
			if (db.print_o2a) {
				SerialUSB.print(c);
			}

			// Itterate byte ind
			if (
				arr_ind < maxBytesStore - 2 ||
				rcvdArr[arr_ind] == '!' ||
				rcvdArr[arr_ind] == '<' ||
				rcvdArr[arr_ind] == '>'
				) {
				arr_ind++;
			}

			// Store data
			rcvdArr[arr_ind] = c;
		}
		// Make sure message finished
		else if (arr_ind >= 0) {
			bool p_arr[3] = { false, false, false };
			for (int i = arr_ind; i >= 0; i--)
			{
				// Check that certain specific comnination of chars recieved
				p_arr[0] = rcvdArr[i] == '\r' || rcvdArr[i] == '1' || rcvdArr[i] == '~' ? true : p_arr[0];
				p_arr[1] = rcvdArr[i] == '\n' || rcvdArr[i] == '2' || rcvdArr[i] == '~' ? true : p_arr[1];
				p_arr[2] = rcvdArr[i] == '>' || rcvdArr[i] == '<' ? true : p_arr[2];
			}
			if (p_arr[0] && p_arr[1] && p_arr[2]) {
				pass = true;
			}
		}
	}
	// Set null terminator
	rcvdArr[arr_ind + 1] = '\0';

	if (db.print_o2a) {
		PrintLOGGER("\n============================================]RCVD_FORMATED\n");
	}

	// Print formated string
	if (db.print_o2aRaw) {
		PrintLOGGER("[LOGGER::GetReply] RCVD_RAW[==========", true);
		for (int i = 0; i <= arr_ind + 1; i++)
		{
			//sprintf(str, "[%d]\'%s\'", rcvdArr[i], PrintSpecialChars(rcvdArr[i]));
			sprintf(str, "\'%s\'", PrintSpecialChars(rcvdArr[i]));
			PrintLOGGER(str);
		}
		PrintLOGGER("\n============================================]RCVD_RAW\n");
	}

	// Save values
	for (int i = 0; i <= arr_ind; i++)
	{
		// Save cmd 
		if (
			rcvdArr[i] == '!' ||
			rcvdArr[i] == '<' ||
			rcvdArr[i] == '>'
			) {
			cmd_reply = cmd_reply != '!' ? rcvdArr[i] : cmd_reply;
		}

		// Check for data start indeces
		dat_ind[0] =
			dat_ind[0] == 0 && rcvdArr[i - 1] == '{' && rcvdArr[i] == '{' ?
			i + 1 : dat_ind[0];
		// Check for end indeces
		if (
			dat_ind[0] > 0 &&
			rcvdArr[i] == '}' &&
			rcvdArr[i + 1] == '}'
			) {
			dat_ind[1] = i - 1;
		}
	}

	// Get data
	if (
		dat_ind[0] != 0 &&
		dat_ind[1] != 0
		) {
		int ii = 0;
		for (int i = dat_ind[0]; i <= dat_ind[1]; i++)
		{
			logCntStr[ii] = rcvdArr[i];
			ii++;
		}
		logCntStr[ii + 1] = '\0';
	}

	// Get current mode
	mode = cmd_reply == '>' || cmd_reply == '<' ? cmd_reply : mode;

	// Print mode and round trip time
	if (db.print_logMode || db.print_o2a || db.print_o2aRaw) {
		sprintf(str, "[LOGGER::GetReply] mode=\'%c\' reply=\'%c\' dt=%dms bytes=%d", mode, cmd_reply, t_rcvd - t_sent, arr_ind + 1);
		PrintLOGGER(str, true);
	}

	// Log/print error
	if (!pass) {
		sprintf(str, "**WARNING** [LOGGER::GetReply] Timedout: dt=%d bytes=%d", timeout, arr_ind + 1);
		PrintLOGGER(str, true);
	}

	// Return cmd 
	return cmd_reply;
}

bool LOGGER::SetToWriteMode(char log_file[])
{
	// Local vars
	char str[200] = { 0 };
	bool pass = false;

	// Check if already in log mode
	if (mode == '<') {
		return true;
	}

	// Send append file command
	sprintf(str, "append %s\r", log_file);
	if (SendCommand(str) != '!') {
		pass = true;
	}

	// Store new log file
	if (pass) {
		strcpy(logFile, log_file);
		delay(100);
		sprintf(str, "[LOGGER::SetToWriteMode] OpenLog Set to Write Mode: file_name=%s mode = %c",
			log_file, mode);
		DebugFlow(str);
	}

	// Log/print error
	else {
		sprintf(str, "!!ERROR!! [LOGGER::SetToWriteMode] ABORTED: file_name=%s mode = %c",
			log_file, mode);
		DebugError(str, true);
	}

	return pass;
}

void LOGGER::QueueLog(char msg[], uint32_t t)
{
	/*
	STORE LOG DATA FOR CS
	FORMAT: "[log_cnt],ts_ms,message,\r\n"
	*/

	// Local vars
	uint32_t t_m = 0;
	char msg_copy[300] = { 0 };

	// Update queue ind
	queueIndStore++;

	// Check if ind should roll over 
	if (queueIndStore == logQueueSize) {

		// Reset queueIndWrite
		queueIndStore = 0;
	}

	// Check if overfloweed
	if (logQueue[queueIndStore][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[logQueueSize + 1];
		for (int i = 0; i < logQueueSize; i++) {
			queue_state[i] = logQueue[i][0] == '\0' ? '0' : '1';
		}
		queue_state[logQueueSize] = '\0';

		// Store overflow error instead
		sprintf(msg_copy, "**WARNING** [LOGGER::QueueLog] LOG QUEUE OVERFLOWED: queueIndStore=%d queueIndRead=%d queue_state=|%s|",
			queueIndStore, queueIndRead, queue_state);

		// Set queue back so overflow will write over last log
		queueIndStore = queueIndStore - 1 >= 0 ? queueIndStore - 1 : logQueueSize - 1;

	}
	// Update log count
	else {
		// Copy message
		for (int i = 0; i < strlen(msg); i++)
		{
			msg_copy[i] = msg[i];
		}
		msg_copy[strlen(msg)] = '\0';

		// Itterate count
		cnt_logsStored++;
	}

	// Get sync correction
	t_m = t - t_sync;

	// Store log
	sprintf(logQueue[queueIndStore], "[%d],%lu,%d,%s\r\n", cnt_logsStored, t_m, cnt_loop_short, msg_copy);

}

bool LOGGER::WriteLog(bool do_send)
{
	/*
	STORE LOG DATA FOR CS
	FORMAT: "[log_cnt],ts_ms,message,\r\n"
	*/

	// Local vars
	char str[300] = { 0 };

	// Bail if not in write mode, no new logs or write blocked
	if (mode != '<' ||
		(queueIndRead == queueIndStore &&
			logQueue[queueIndStore][0] == '\0')) {

		// Indicate no logs to write
		return false;
	}

	// Bail if writing blocked
	if (fc.doBlockLogWrite) {

		return false;
	}

	// Bail if less than 50ms sinse last write
	if (micros() < t_write + dt_write) {
		// Indicate still logs to store
		return true;
	}

	// Itterate send ind
	queueIndRead++;

	// Check if ind should roll over 
	if (queueIndRead == logQueueSize) {
		queueIndRead = 0;
	}

	// Write to SD
	port.write(logQueue[queueIndRead]);
	t_write = micros();

	// Send now
	if (do_send) {
		r2c.port.write(logQueue[queueIndRead]);
		cnt_logBytesSent++;
		delay(10);
	}

	// Update bytes stored
	cnt_logBytesStored += strlen(logQueue[queueIndRead]);

	// Print stored log
	if (db.print_logStore) {
		sprintf(str, "   [LOG] r2c: log_cnt=%d bytes_sent=%d/%d queueIndStore=%d, queueIndRead=%d, msg=\"%s\"",
			cnt_logsStored, strlen(logQueue[queueIndRead]), cnt_logBytesStored, queueIndStore, queueIndRead, logQueue[queueIndRead]);
		QueueDebug(str, millis());
	}

	// Set entry to null
	logQueue[queueIndRead][0] = '\0';

	// Return success
	return true;
}

void LOGGER::StreamLogs()
{
	// Local vars
	char str[300] = { 0 };
	const int timeout = 1200000;
	static int cnt_err_set_cmd_mode = 0;
	static int cnt_err_read_cmd = 0;
	static int cnt_err_read_timeout = 0;
	uint32_t t_start = millis(); // (ms)
	uint32_t t_last_read = millis(); // (ms)
	int read_ind = 0;
	bool head_passed = false;
	bool send_done = false;
	bool do_abort = false;
	bool is_timedout = false;
	char err_str[200] = "|";
	char c_arr[3] = { 0 };

	// Bail if not ready to send
	if (millis() < t_beginSend) {
		return;
	}

	// Store remaining logs
	while (WriteLog());
	delay(100);

	// Stop writing logs
	fc.doBlockLogWrite = true;

	// Make sure in command mode
	if (!SetToCmdMode()) {

		// Add to error counter
		cnt_err_set_cmd_mode++;

		// Leave function
		if (cnt_err_set_cmd_mode < 3) {
			return;
		}

		// Abort after 3 failures
		else {
			do_abort = true;
		}

		// Store error
		if (strlen(err_str) < 200) {
			sprintf(str, "\"%c%c%c\" Failed: cnt=%d|",
				26, 26, 26, cnt_err_set_cmd_mode);
			strcat(err_str, str);
		}
	}

	// Print anything left in queue
	while (PrintDebug());

	// Start timers
	t_start = millis();

	// Begin streaming data
	while (millis() < (t_start + timeout)) {

		// Dump anything in openlog buffer
		uint32_t t_out = millis() + 10;
		while (millis() < t_out || port.available() > 0) {
			if (port.available() > 0) {
				port.read();
			}
		}

		// Send new "read" command
		if (!send_done &&
			!do_abort) {
			sprintf(str, "read %s %d %d\r", logFile, 0, (uint32_t)-1);
			SendCommand(str, false, 5000);
		}

		// Finished
		else {
			break;
		}

		// Reset vars
		head_passed = false;
		read_ind = 0;
		c_arr[0] = 0;
		c_arr[1] = 0;
		c_arr[2] = 0;
		t_last_read = millis();

		// Read one byte at a time
		while (millis() < (t_start + timeout)) {

			// Check for new data
			if (port.available() == 0)
			{
				// Wait a max of 1 sec for new data
				if (millis() - t_last_read < 1000 ||
					cnt_logBytesSent == 0) {
					continue;
				}

				// Handle read timeout
				else {

					// Add to error counter
					cnt_err_read_timeout++;

					// Try re-requesting data at least once
					if (cnt_err_read_timeout > 1) {
						do_abort = true;
					}

					// Store error
					if (strlen(err_str) < 200) {
						sprintf(str, "Read Timedout: cnt=%d read_ind=%d dt_read=%d|",
							cnt_err_read_timeout, read_ind, millis() - t_last_read);
						strcat(err_str, str);
					}

					// Break
					break;
				}
			}

			// Get next bytes
			c_arr[0] = c_arr[1];
			c_arr[1] = c_arr[2];
			c_arr[2] = port.read();
			t_last_read = millis();
			read_ind++;

			// Check for leading "\r\n"
			if (!head_passed) {

				// Header found
				if (c_arr[2] != '\r' &&
					c_arr[2] != '\n') {

					head_passed = true;
				}

				// Continue till header found
				else {
					continue;
				}
			}

			// Check for error
			if (c_arr[2] == '!') {

				if ((c_arr[0] == '\r' && c_arr[1] == '\n') ||
					read_ind < 3) {

					// Add to error counter
					cnt_err_read_cmd++;

					// Retry "read" command only once
					if (cnt_err_read_cmd > 1) {
						do_abort = true;
					}

					// Store error
					if (strlen(err_str) < 200) {
						sprintf(str, "\"read\" Failed: cnt=%d|", cnt_err_read_cmd);
						strcat(err_str, str);
					}

					// Break
					break;
				}
			}

			// Send byte
			r2c.port.write(c_arr[2]);
			cnt_logBytesSent++;

			// Check if all bytes sent
			if (cnt_logBytesSent == cnt_logBytesStored) {

				send_done = true;
				break;
			}

		}
	}

	// Check for unflagged timeout
	if (!do_abort &&
		millis() >= t_start + timeout) {

		do_abort = true;
		sprintf(str, "Run Timedout|");
		if (strlen(err_str) < 200) {
			strcat(err_str, str);
		}
	}

	// Resume logging
	fc.doBlockLogWrite = false;
	SetToWriteMode(logFile);

	// Log warnings summary
	char warn_lines[200] = "ON LINES |";
	for (int i = 0; i < cnt_warn; i++) {
		char str_lin[10];
		sprintf(str_lin, "%d|", warn_line[i]);
		strcat(warn_lines, str_lin);
	}
	sprintf(str, "TOTAL WARNINGS: %d %s", cnt_warn, cnt_warn > 0 ? warn_lines : "");
	DebugFlow(str);

	// Log errors summary
	char err_lines[200] = "ON LINES |";
	for (int i = 0; i < cnt_err; i++) {
		char str_lin[10];
		sprintf(str_lin, "%d|", err_line[i]);
		strcat(err_lines, str_lin);
	}
	sprintf(str, "TOTAL ERRORS:  %d %s", cnt_err, cnt_err > 0 ? warn_lines : "");
	DebugFlow(str);

	// Send any new logs
	while (WriteLog(true));

	// Get total data left in buffers
	int xbee_buff_tx = SERIAL_BUFFER_SIZE - 1 - r2c.port.availableForWrite();
	int xbee_buff_rx = r2c.port.available();
	int ol_buff_tx = SERIAL_BUFFER_SIZE - 1 - port.availableForWrite();
	int ol_buff_rx = port.available();

	// Print log time info
	float dt_s = (float)(millis() - t_start) / 1000.0f;
	sprintf(str, "[LOGGER::StreamLogs] Run Info: dt_run=%0.2fs bytes_sent=%d bytes_stored=%d cnt_err_set_cmd_mode=%d cnt_err_read_cmd=%d cnt_err_read_timeout=%d openlog_tx=%d openlog_rx=%d xbee_tx=%d xbee_rx=%d",
		dt_s, cnt_logBytesSent, cnt_logBytesStored, cnt_err_set_cmd_mode, cnt_err_read_cmd, cnt_err_read_timeout, ol_buff_tx, ol_buff_rx, xbee_buff_tx, xbee_buff_rx);
	DebugFlow(str);
	// Send imediately
	while (WriteLog(true));

	// Print final status then send as log
	if (!do_abort) {
		sprintf(str, "[LOGGER::StreamLogs] SUCCEEDED: Sent %d Logs", cnt_logsStored + 1);
		DebugFlow(str);
	}
	else {
		sprintf(str, "!!ERROR!! [LOGGER::StreamLogs] ABORTED: Sending %d Logs: errors=%s", cnt_logsStored + 1, err_str);
		DebugError(str, true);
	}
	// Send imediately
	while (WriteLog(true));

	// End reached send ">>>"
	if (!do_abort) {
		byte msg_end[3] = { '>','>','>' };
		r2c.port.write(msg_end, 3);
		delay(100);
	}
	// Aborted send ">>!"
	else {
		byte msg_end[3] = { '>','>','!' };
		r2c.port.write(msg_end, 3);
		delay(100);
	}

	// Print anything left in queue
	while (PrintDebug());

	// Reset flag
	fc.doLogSend = false;
	fc.doBlockVccSend = false;
}

void LOGGER::TestLoad(int n_entry, char log_file[])
{
	// Local vars
	char str[200] = { 0 };
	bool pass = false;

	// Prevent any new info from logging
	while (WriteLog());
	db.Log = false;

	// Load existing log file
	if (log_file != '\0') {
		sprintf(str, "[LOGGER::TestLoad] RUNNING: Load Log: file_name=%s...", log_file);
		DebugFlow(str, millis());
		while (PrintDebug());

		if (SetToCmdMode()) {

			// Get bytes
			if (SendCommand("ls\r") != '!') {
				cnt_logBytesStored = GetFileSize(log_file);
				pass = true;
			}

			// Start writing to file
			if (pass)
				if (!SetToWriteMode(log_file))
					pass = false;
		}
		if (pass) {
			sprintf(str, "[LOGGER::TestLoad] SUCCEEDED: Load Log: file_name=%s size=%dB",
				logFile, cnt_logBytesStored);
			DebugFlow(str, millis());
			while (PrintDebug());
		}
		else {
			DebugError("!!ERROR!! [LOGGER::TestLoad] ABORTED: Load Log File", true);
			while (PrintDebug());
		}
	}

	// Write n_entry entries to log
	else if (n_entry != 0) {
		sprintf(str, "[LOGGER::TestLoad] RUNNING: Write %d Logs...", n_entry);
		DebugFlow(str, millis());
		while (PrintDebug());
		delayMicroseconds(100);
		while (WriteLog());

		// Add n new logs
		randomSeed(analogRead(A0));
		int milestone_incriment = n_entry / 10;
		for (int i = 0; i < n_entry - 1; i++)
		{
			// Print status
			if (i%milestone_incriment == 0) {
				sprintf(str, "[LOGGER::TestLoad] Log Write %d%% Complete", i / milestone_incriment * 10);
				DebugFlow(str, millis());
				while (PrintDebug());
				while (WriteLog());
			}

			// Store a 120 charicter string
			char msg[200] = "AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFFGGGGGGGGGGHHHHHHHHHHIIIIIIIIIIJJJJJJJJJJKKKKKKKKKKLLLLLLLLLL";

			//// Make random sized strings
			//char num_str[15] = { 0 };
			//char msg[300] = { 0 };
			//for (int i = 0; i < 18; i++)
			//{
			//	int num = random(999999999);
			//	sprintf(num_str, "%d", num);
			//	num_str[random(10) + 1] = '\0';
			//	strcat(msg, num_str);
			//}

			// Store log
			QueueLog(msg, millis());
			t_write = micros() - dt_write;
			while (WriteLog());
		}
		sprintf(str, "[LOGGER::TestLoad] FINISHED: Write %d Logs", n_entry);
		DebugFlow(str, millis());
		while (PrintDebug());
		while (WriteLog());
	}
}

int LOGGER::GetFileSize(char log_file[])
{
	// Local vars
	int ind = -1;
	int fi_size = 0;
	char num_str[20] = { 0 };

	// Find file index in array
	for (int i = strlen(rcvdArr) - strlen(log_file); i >= 0; i--)
	{
		int ii = 0;
		for (ii = strlen(log_file) - 1; ii >= 0; ii--) {

			if (log_file[ii] != rcvdArr[i + ii]) {
				break;
			}
			else if (ii == 0) {
				ind = ii + i;
				break;
			}
		}
		if (ind != -1) {
			break;
		}
	}

	// Get file size
	if (ind != -1) {

		int i = ind + strlen(log_file) + 3;
		int ii = 0;

		while (i < strlen(rcvdArr) && rcvdArr[i] != '\r')
		{
			num_str[ii] = rcvdArr[i];
			i++;
			ii++;
		}

		num_str[ii] = '\0';
		fi_size = atoi(num_str);
	}

	return fi_size;
}

void LOGGER::PrintLOGGER(char msg[], bool start_entry)
{
	// Local vars
	uint32_t t_m = 0;
	float t_s = 0;

	// Bail if logging should not be printed
	if (!db.print_logging) {
		return;
	}

	// Print like normal entry
	if (start_entry) {
		DebugFlow(msg, millis());
		// Print right away
		while (PrintDebug());
	}

	// Print directly 
	else {
		SerialUSB.print(msg);
	}
}

#pragma endregion 

#pragma endregion 


#pragma region ========== FUNCTION DEFINITIONS =========

#pragma region --------COMMUNICATION---------

// PARSE SERIAL INPUT
void GetSerial(R4 *r4)
{
	/*
	PARSE DATA FROM CS
	FORMAT: [0]head, [1]id, [2:5]dat[0], [6:9]dat[1], [10:13]dat[1], [14:15]pack, [16]do_conf, [17]footer, [18]targ
	*/

	// Local vars
	char str[300] = { 0 };
	uint32_t t_str = millis();
	char dat_str[200] = { 0 };
	int buff_tx = 0;
	int buff_rx = 0;
	byte buff = 0;
	char head = ' ';
	char id = ' ';
	float dat[3] = { 0 };
	int id_ind = 0;
	uint16_t pack = 0;
	char foot = ' ';
	bool do_conf;
	R2 *r2;

	// Set pointer to R2 struct
	if (r4->instID == 'c') {
		r2 = &r2c;
	}
	else if (r4->instID == 'a') {
		r2 = &r2a;
	}

	// Reset vars
	cnt_packBytesRead = 0;
	cnt_packBytesDiscarded = 0;
	r4->isNew = false;
	r4->idNow = ' ';

	// Bail if no new input
	if (r4->port.available() == 0) {
		return;
	}

	// Dump data till msg header byte is reached
	buff = WaitBuffRead(r4, r4->head);
	if (buff == 0) {
		return;
	}

	// Store header
	head = buff;

	// Get id
	id = WaitBuffRead(r4);

	// Parse data
	for (int i = 0; i < 3; i++)
	{
		U.f = 0.0f;
		U.b[0] = WaitBuffRead(r4);
		U.b[1] = WaitBuffRead(r4);
		U.b[2] = WaitBuffRead(r4);
		U.b[3] = WaitBuffRead(r4);
		dat[i] = U.f;
	}

	// Get packet num
	U.f = 0.0f;
	U.b[0] = WaitBuffRead(r4);
	U.b[1] = WaitBuffRead(r4);
	pack = U.i16[0];

	// Get recieved confirmation
	U.f = 0.0f;
	U.b[0] = WaitBuffRead(r4);
	do_conf = U.b[0] != 0 ? true : false;

	// Get footer
	foot = WaitBuffRead(r4);

	// Get total data in buffers
	buff_rx = r4->port.available();
	buff_tx = SERIAL_BUFFER_SIZE - 1 - r4->port.availableForWrite();

	// Store data string
	sprintf(dat_str, " head=%c id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d foot=%c do_conf=%s bytes_read=%d bytes_dumped=%d rx=%d tx=%d dt_parse=%d dt_send=%d dt_rcv=%d",
		head, id, dat[0], dat[1], dat[2], pack, foot, do_conf ? "true" : "false", cnt_packBytesRead, cnt_packBytesDiscarded, buff_rx, buff_tx, millis() - t_str, millis() - r2->t_sent, r4->dt_rcvd);

	// Check for matching footer
	if (foot == r4->foot) {

		// Update recive time
		r4->dt_rcvd = r4->t_rcvd > 0 ? millis() - r4->t_rcvd : 0;
		r4->t_rcvd = millis();

		// Get id ind
		id_ind = CharInd(id, r4->id, r4->lng);

		// Send confirmation
		if (do_conf) {
			QueuePacket(r2, id, dat[0], dat[1], dat[2], pack, false);
		}

		// Set coms started flag
		if (r4->instID == 'c' && !fc.isComsStarted) {
			fc.isComsStarted = true;
		}
	}

	// Packet dropped
	else {

		// Itterate dropped count
		r4->cnt_dropped++;

		// Log/print dropped packet info
		sprintf(str, "**WARNING** [GetSerial] Dropped %c2r Packet: cnt=%d",
			r4->instID, r4->cnt_dropped);
		strcat(str, dat_str);
		DebugError(str);
		return;
	}

	// Reset check
	r2->doRcvCheck[id_ind] = false;
	r2->cnt_resend[id_ind] = 0;

	// Check for missed packets
	int pack_diff = (pack - r4->packTot);

	// Get number of dropped/missed packets
	int cnt_dropped = pack_diff - 1;

	if (cnt_dropped > 0)
	{
		// Add to count and get last pack
		int cnt_dropped_tot = 0;
		uint16_t pack_tot_last = 0;

		// Store dropped data
		r4->cnt_dropped += cnt_dropped;
		cnt_dropped_tot = r4->cnt_dropped;
		pack_tot_last = r4->packTot;

		// Log/print skipped packet info
		sprintf(str, "**WARNING** [GetSerial] Missed %c2r Packets: cnt=%d|%d pack_last=%d",
			r4->instID, cnt_dropped, cnt_dropped_tot, pack_tot_last);
		strcat(str, dat_str);
		DebugError(str);
	}

	// Update packet history
	r4->packLast[id_ind] = r4->pack[id_ind];
	r4->pack[id_ind] = pack;
	r4->packTot = pack > r4->packTot ? pack : r4->packTot;
	uint16_t pack_last = r4->packLast[id_ind];

	// Update id
	r4->idNow = id;

	// New pack
	if (pack != pack_last)
	{
		// Log/print received
		DebugRcvd(r4, dat_str);

		// Store data and flag
		r4->isNew = true;
		r4->dat[0] = dat[0];
		r4->dat[1] = dat[1];
		r4->dat[2] = dat[2];
	}

	// Resent packet
	else {
		// Add to counters
		r4->cnt_repeat++;

		// Log/print received
		DebugRcvd(r4, dat_str, true);
	}

	// Check if data was discarded
	if (cnt_packBytesDiscarded > 0) {

		// Log/print discarded data
		sprintf(str, "**WARNING** [GetSerial] Discarded Bytes:%s", dat_str);
		DebugError(str);
	}

	// Check if parsing took unusually long
	if (millis() - t_str > 30) {
		sprintf(str, "**WARNING** [GetSerial] Parser Hanging:%s", dat_str);
		DebugError(str);
	}

}

// WAIT FOR BUFFER TO FILL
byte WaitBuffRead(R4 *r4, char mtch)
{
	// Local vars
	static int timeout = 100;
	uint32_t t_timeout = millis() + timeout;
	static int cnt_overflow = 0;
	static int cnt_timeout = 0;
	bool is_overflowed = false;
	byte buff = 0;

	// Get total data in buffers now
	int buff_rx_start = r4->port.available();

	// Check for overflow
	is_overflowed = buff_rx_start >= SERIAL_BUFFER_SIZE - 1;

	// Wait for at least 1 byte
	while (r4->port.available() < 1 &&
		millis() < t_timeout);

	// Get any byte
	if (!is_overflowed &&
		mtch == '\0') {

		if (r4->port.available() > 0) {

			buff = r4->port.read();
			cnt_packBytesRead++;
			return buff;
		}
	}

	// Find specific byte
	while (
		buff != mtch  &&
		millis() < t_timeout &&
		!is_overflowed) {

		// Check new data
		if (r4->port.available() > 0) {

			buff = r4->port.read();
			cnt_packBytesRead++;

			// check match was found
			if (buff == mtch) {
				return buff;
			}

			// Otherwise add to discard count
			else {
				cnt_packBytesDiscarded++;
			}

			// Check for overflow
			is_overflowed =
				!is_overflowed ? r4->port.available() >= SERIAL_BUFFER_SIZE - 1 : is_overflowed;
		}

	}

	// Print issue
	char msg_str[200];
	char dat_str[200];

	// Check if buffer flooded
	if (is_overflowed) {

		// DUMP IT ALL
		while (r4->port.available() > 0) {
			if (r4->port.available() > 0) {
				r4->port.read();
				cnt_packBytesRead++;
			}
		}
	}

	// Get buffer 
	int buff_tx = SERIAL_BUFFER_SIZE - 1 - r4->port.availableForWrite();
	int buff_rx = r4->port.available();

	// Store current info
	sprintf(dat_str, " from=%c buff=\'%s\' bytes_read=%d bytes_dumped=%d rx_start=%d rx_now=%d tx_now=%d dt_check=%d",
		r4->instID, PrintSpecialChars(buff), cnt_packBytesRead, cnt_packBytesDiscarded, buff_rx_start, buff_rx, buff_tx, (millis() - t_timeout) + timeout);

	// Buffer flooded
	if (is_overflowed) {
		cnt_overflow++;
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Buffer Overflowed: cnt=%d", cnt_overflow);
	}
	// Timed out
	else if (millis() > t_timeout) {
		cnt_timeout++;
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Timedout: cnt=%d", cnt_timeout);
	}
	// Byte not found
	else if (mtch != '\0') {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Char \'%c\' Not Found:",
			mtch);
	}
	// Failed for unknown reason
	else {
		sprintf(msg_str, "**WARNING** [WaitBuffRead] Failed for Unknown Reason:");
	}

	// Compbine strings
	strcat(msg_str, dat_str);

	// Log/print error
	DebugError(msg_str);

	// Return 0
	return 0;

}

// STORE PACKET DATA TO BE SENT
void QueuePacket(R2 *r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf)
{
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	char str[300] = { 0 };
	int id_ind = 0;
	float dat[3] = { dat1 , dat2 , dat3 };
	R4 *r4;

	// Set pointer to R4 struct
	if (r2->instID == 'c') {
		r4 = &c2r;
	}
	else if (r2->instID == 'a') {
		r4 = &a2r;
	}

	// Update sendQueue ind
	r2->sendQueueIndStore++;

	// Check if ind should roll over 
	if (r2->sendQueueIndStore == sendQueueSize) {

		// Reset queueIndWrite
		r2->sendQueueIndStore = 0;
	}

	// Check if overfloweed
	if (r2->sendQueue[r2->sendQueueIndStore][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[sendQueueSize + 1];
		for (int i = 0; i < sendQueueSize; i++) {
			queue_state[i] = r2->sendQueue[i][0] == '\0' ? '0' : '1';
		}
		queue_state[sendQueueSize] = '\0';

		// Get buffers
		int buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
		int buff_rx = r2->port.available();

		// Store overflow error instead
		sprintf(str, "!!ERRROR!! [QueuePacket] r2%c SEND QUEUE OVERFLOWED: sendQueueIndStore=%d sendQueueIndRead=%d queue_state=|%s| dt_send=%d dt_rcv=%d tx=%d rx=%d",
			r2->instID, r2->sendQueueIndStore, r2->sendQueueIndRead, queue_state, millis() - r2->t_sent, millis() - r4->t_rcvd, buff_tx, buff_rx);

		// Log/print error
		DebugError(str, true);

		// Set queue back 
		r2->sendQueueIndStore = r2->sendQueueIndStore - 1 >= 0 ? r2->sendQueueIndStore - 1 : sendQueueSize - 1;

		// Bail
		return;

	}

	// Itterate packet number
	if (pack == 0) {
		pack = ++r2->cnt_pack;
	}

	// Create byte packet
	int b_ind = 0;
	// Store header
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = r2->head;
	// Store mesage id
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = id;
	// Store mesage data 
	for (int i = 0; i < 3; i++)
	{
		U.f = dat[i];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[0];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[1];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[2];
		r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[3];
	}
	// Store packet number
	U.f = 0.0f;
	U.i16[0] = pack;
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[0];
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = U.b[1];
	// Store get_confirm request
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = do_conf ? 1 : 0;
	// Store footer
	r2->sendQueue[r2->sendQueueIndStore][b_ind++] = r2->foot;

}

// SEND SERIAL PACKET DATA
bool SendPacket(R2 *r2)
{
	/*
	STORE DATA TO SEND
	FORMAT IN:  [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer, [9]targ
	FORMAT OUT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer
	*/

	// Local vars
	const int msg_lng = sendQueueBytes;
	byte msg[msg_lng];
	cnt_packBytesSent = 0;
	bool is_resend = false;
	char id = '\0';
	float dat[3] = { 0 };
	bool do_conf = 0;
	uint16_t pack = 0;
	int buff_tx;
	int buff_rx;
	int id_ind = 0;
	char dat_str[200] = { 0 };
	uint32_t t_queue = millis();
	R4 *r4;

	// Set pointer to R4 struct
	if (r2->instID == 'c') {
		r4 = &c2r;
	}
	else if (r2->instID == 'a') {
		r4 = &a2r;
	}

	// Bail if nothing in queue
	if (r2->sendQueueIndRead == r2->sendQueueIndStore &&
		r2->sendQueue[r2->sendQueueIndStore][0] == '\0') {
		return false;
	}

	// Get buffer 
	buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
	buff_rx = r2->port.available();

	// Bail if buffer or time inadequate
	if (!(buff_tx == 0 &&
		buff_rx == 0 &&
		millis() > r2->t_sent + dt_sendSent &&
		millis() > r4->t_rcvd + dt_sendRcvd)) {

		// Indicate still packs to send
		return true;
	}

	// Itterate send ind
	r2->sendQueueIndRead++;

	// Check if ind should roll over 
	if (r2->sendQueueIndRead == sendQueueSize) {
		r2->sendQueueIndRead = 0;
	}

	// Send
	r2->port.write(r2->sendQueue[r2->sendQueueIndRead], msg_lng);
	r2->dt_sent = r2->t_sent > 0 ? millis() - r2->t_sent : 0;
	r2->t_sent = millis();
	cnt_packBytesSent = msg_lng;

	// Get buffers
	buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
	buff_rx = r2->port.available();

	// pull out packet data
	int b_ind = 1;
	// id
	id = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
	// dat
	for (int i = 0; i < 3; i++)
	{
		U.f = 0;
		U.b[0] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		U.b[1] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		U.b[2] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		U.b[3] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
		dat[i] = U.f;
	}
	// pack
	U.f = 0.0f;
	U.b[0] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
	U.b[1] = r2->sendQueue[r2->sendQueueIndRead][b_ind++];
	pack = U.i16[0];
	// do_conf 
	do_conf = r2->sendQueue[r2->sendQueueIndRead][b_ind++] == 1 ? true : false;

	// Set entry to null
	r2->sendQueue[r2->sendQueueIndRead][0] = '\0';

	// Get id ind
	id_ind = CharInd(id, r2->id, r2->lng);

	// Check if resending
	is_resend = pack == r2a.packLast[id_ind];

	// Set flags for recieve confirmation
	if (do_conf) {
		r2->doRcvCheck[id_ind] = true;
	}

	// Update struct info
	r2->datList[id_ind][0] = dat[0];
	r2->datList[id_ind][1] = dat[1];
	r2->datList[id_ind][2] = dat[2];
	r2->packLast[id_ind] = r2->pack[id_ind];
	r2->pack[id_ind] = pack;
	r2->t_sentList[id_ind] = r2->t_sent;

	// Check if resending
	is_resend = pack == r2->packLast[id_ind];

	// Make log/print string
	sprintf(dat_str, "id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d do_conf=%s bytes_sent=%d tx=%d rx=%d cts=%s dt_send=%d dt_rcv=%d dt_queue=%d",
		id, dat[0], dat[1], dat[2], pack, do_conf ? "true" : "false", cnt_packBytesSent, buff_tx, buff_rx, r2->stateCTS ? "high" : "low", r2->dt_sent, millis() - r4->t_rcvd, millis() - t_queue);

	// Is first send
	if (!is_resend) {
		DebugSent(r2, dat_str);
	}

	// Is resend
	else {
		// Add to counters
		r2->cnt_repeat++;

		// Log/print resend and count
		DebugSent(r2, dat_str, true);
	}

	// Return success
	return true;
}

// CHECK IF ROB TO ARD PACKET SHOULD BE RESENT
bool CheckResend(R2 *r2)
{
	// Local vars
	char str[300] = { 0 };
	char dat_str[200] = { 0 };
	bool do_pack_resend = false;
	int dt_sent = 0;

	// Loop and check ard flags
	for (int i = 0; i < r2->lng; i++)
	{
		// Get dt sent
		dt_sent = millis() - r2->t_sentList[i];

		// Check if should resend
		if (
			r2->doRcvCheck[i] &&
			dt_sent > dt_resend
			) {

			// Get total data left in buffers
			int buff_tx = SERIAL_BUFFER_SIZE - 1 - r2->port.availableForWrite();
			int buff_rx = r2->port.available();

			// Get dat string
			sprintf(dat_str, "id=\'%c\' dat=|%0.2f|%0.2f|%0.2f| pack=%d dt_sent=%dms tx=%d rx=%d",
				 r2->id[i], r2->datList[i][0], r2->datList[i][1], r2->datList[i][2], r2->pack[i], dt_sent, buff_tx, buff_rx);

			if (r2->cnt_resend[i] < resendMax) {

				// Resend data
				QueuePacket(r2, r2->id[i], r2->datList[i][0], r2->datList[i][1], r2->datList[i][2], r2->pack[i], true);

				// Update count
				r2->cnt_resend[i]++;

				// Print resent packet
				sprintf(str, "**WARNING** [CheckResend] Resending r2%c Packet: cnt=%d %s", 
					r2->instID, r2->cnt_resend[i], dat_str);
				DebugError(str);

				// Set flags
				do_pack_resend = true;
				r2->doRcvCheck[i] = false;
			}

			// Coms failed
			else {

				// Log/print error
				sprintf(str, "!!ERROR!! [CheckResend] ABORTED: Resending r2%c Packet: cnt=%d %s", 
					r2->instID, r2->cnt_resend[i], dat_str);
				DebugError(str, true);

				// Reset flag
				r2->doRcvCheck[i] = false;
			}
		}
	}

	// Return
	return do_pack_resend;
}

#pragma endregion


#pragma region --------MOVEMENT AND TRACKING---------

// CONFIGURE AUTODRIVER BOARDS
void AD_Config(float max_speed, float max_acc, float max_dec)
{
	// Set busy pin as BUSY_PIN or SYNC_PIN;
	/*
	SYNC_FS_2 - two pulses on sync pin per full step of motor
	SYNC_FS - one pulse per full step
	SYNC_XFS - where X can be 2, 4, 8, 16, 32, or 64, and X indicates the number of full steps between pulses on the sync pin
	*/
	AD_R.configSyncPin(BUSY_PIN, 0);
	AD_F.configSyncPin(BUSY_PIN, 0);

	// Microsteps per step
	/*
	STEP_FS - Full-step mode; microstepping disabled
	STEP_FS_X - Enable microstepping with X microsteps per full step. X can be 2, 4, 8, 16, 32, 64, or 128.
	*/
	AD_R.setParam(STEP_MODE, STEP_FS_128);
	AD_F.setParam(STEP_MODE, STEP_FS_128);

	// PWM freq
	/*
	PWM_DIV_X, where X can be any value 1-7.
	PWM_MUL_X, where X can be 0_625 (for 0.625), 0_75 (for 0.75), 0_875, 1, 1_25, 1_5, 1_75, or 2.
	*/
	AD_R.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq
	AD_F.setPWMFreq(PWM_DIV_2, PWM_MUL_2);		// 31.25kHz PWM freq		

												// Overcurent enable
	AD_R.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC
	AD_F.setOCShutdown(OC_SD_ENABLE);			// shutdown on OC

												// Motor V compensation
												/*
												VS_COMP_ENABLE, VS_COMP_DISABLE
												*/
	AD_R.setVoltageComp(VS_COMP_ENABLE);
	AD_F.setVoltageComp(VS_COMP_ENABLE);

	// Switch pin mode
	AD_R.setSwitchMode(SW_USER);				// Switch is not hard stop
	AD_F.setSwitchMode(SW_USER);				// Switch is not hard stop

												// Slew rate
												/*
												Upping the edge speed increases torque
												SR_180V_us, SR_290V_us, SR_530V_us
												*/
	AD_R.setSlewRate(SR_530V_us);
	AD_F.setSlewRate(SR_530V_us);


	// Overcurrent threshold
	/*
	375, 750, 1125, 1500, 1875, 2250, 2625, 3000,
	3375, 3750, 4125, 4500, 4875, 5250, 5625, or 6000
	Peak Amp for 1.2 A stepper = 1.2*1.41 = 1690 mA
	Peak Amp for 1.7 A stepper = 1.7*1.41 = 2397 mA
	Peak Amp for 2.82 A stepper = 2.82*1.41 = 3.97 mA
	*/
	AD_R.setOCThreshold(OC_4875mA);
	AD_F.setOCThreshold(OC_3750mA);

	// Low speed compensation
	/*
	Enabled low speed compensation. If enabled, MinSpeed is upper threshold at which this compensation is employed.
	*/
	AD_R.setLoSpdOpt(true);
	AD_F.setLoSpdOpt(true);

	// ---------SPEED SETTTINGS---------

	// Steps/s max
	AD_R.setMaxSpeed(max_speed * cm2stp);
	AD_F.setMaxSpeed(max_speed * cm2stp);

	// Minimum speed
	AD_R.setMinSpeed(10 * cm2stp);
	AD_F.setMinSpeed(10 * cm2stp);

	// Full speed
	AD_R.setFullSpeed(max_speed * cm2stp);
	AD_F.setFullSpeed(max_speed * cm2stp);

	// Acceleration
	/*
	Accelerate at maximum steps/s/s; 0xFFF = infinite
	*/
	AD_R.setAcc(max_acc * cm2stp);
	AD_F.setAcc(max_acc * cm2stp);

	// Deceleration
	/*
	Deccelerate at maximum steps/s/s; 0xFFF = infinite
	*/
	AD_R.setDec(max_dec * cm2stp);
	AD_F.setDec(max_dec * cm2stp);

	// ---------KVAL SETTTINGS---------
	/*
	K Val settings
	KVAL = [(KVAL_X + BEMF_COMP) * VSCOMP * K_THERM] * microstep
	KVAL = Rm * Iph / Vs = %
	Pololu item #: 1200: 1.2A, 4V, 3.3Ohm, 2.8mH = 84.15
	Pololu item #: 2267: 1.68A, 2.8V, 1.65Ohm, 3.2mH = 58.9
	AA item #: 23Y108D-LW8: 2.82A, 2.82V, 1.65Ohm, 3.2mH = 58.9
	*/

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
}

// RESET AUTODRIVER BOARDS
void AD_Reset(float max_speed, float max_acc, float max_dec)
{
	// Reset each axis
	AD_R.resetDev();
	AD_F.resetDev();
	delayMicroseconds(100);

	// Configure each axis
	AD_Config(max_speed, max_acc, max_dec);
	delayMicroseconds(100);
	AD_CheckOC(true);
	delayMicroseconds(100);

	// Run motor at last speed
	RunMotor(runDirNow, runSpeedNow, "Override");
}

// CHECK AUTODRIVER STATUS
void AD_CheckOC(bool force_check)
{
	// Local vars
	char str[300] = { 0 };
	static bool dp_disable = false;
	static int cnt_errors = 0;
	static int ocd_last_r = 1;
	static int ocd_last_f = 1;
	int ocd_r;
	int ocd_f;

	// Bail if check disabled
	if (dp_disable) {
		return;
	}

	// Bail if not time for next check
	if (!force_check &&
		millis() < t_checkAD) {

		return;
	}

	// Get/check 16 bit status flag
	adR_stat = AD_R.getStatus();
	ocd_r = CheckAD_Status(adR_stat, "OCD");
	adF_stat = AD_F.getStatus();
	ocd_f = CheckAD_Status(adF_stat, "OCD");

	// Check for overcurrent shut down
	if (ocd_r == 0 || ocd_f == 0) {

		// Track events
		cnt_errors++;

		// Reset motors
		AD_Reset();

		// Log/print
		sprintf(str, "!!ERROR!! [AD_CheckOC] Overcurrent Detected & Motor Reset: now_ocd_R|F=%d|%d last_ocd_R|F=%d|%d", ocd_r, ocd_f, ocd_last_r, ocd_last_f);
		DebugError(str, true);
	}

	// Store status
	ocd_last_r = ocd_r;
	ocd_last_f = ocd_f;

	// Set next check
	t_checkAD = millis() + dt_checkAD;

	// Disable checking after 5 errors
	if (cnt_errors >= 5) {
		sprintf(str, "!!ERROR!! [AD_CheckOC] Disabled AD Check After %d Errors", cnt_errors);
		DebugError(str, true);
		dp_disable = true;
	}

}

// HARD STOP
void HardStop(char called_from[])
{
	// Local vars
	char str[200] = { 0 };

	// Normal hard stop
	AD_R.hardStop();
	AD_F.hardStop();

	// Reset speed
	runSpeedNow = 0;

	// Reset pid
	Pid.Reset();

	// Set to high impedance so robot can be moved
	if (fc.isManualSes) {

		AD_R.hardHiZ();
		AD_F.hardHiZ();
	}

	// Log/print event
	sprintf(str, "[HardStop] Hard Stop [%s]", called_from);
	DebugFlow(str);
}

// IR TRIGGERED HARD STOP
void IRprox_Halt()
{
	if (!(Bull.mode == "Active" && Bull.state == "On")) {

		HardStop("IRprox_Halt");
	}
}

// RUN AUTODRIVER
bool RunMotor(char dir, double new_speed, String agent)
{
	// Local vars
	double speed_steps = new_speed*cm2stp;

	// Check that caller has control
	if (agent == fc.motorControl ||
		agent == "Override") {

		// Run forward
		if (dir == 'f') {
			AD_R.run(FWD, speed_steps);
			AD_F.run(FWD, speed_steps*scaleFrontAD);
		}

		// Run reverse
		else if (dir == 'r') {
			AD_R.run(REV, speed_steps);
			AD_F.run(REV, speed_steps*scaleFrontAD);
		}

		// Log/print speed change
		DebugRunSpeed(agent, runSpeedNow, new_speed);

		// Update run speed and dir
		runSpeedNow = new_speed;
		runDirNow = dir;

		return true;
	}
	else {
		return false;
	}
}

// RUN MOTOR MANUALLY
bool ManualRun(char dir)
{
	// Local vars
	char speed_str[100] = { 0 };
	char vcc_str[100] = { 0 };
	int inc_speed = 10; // (cm/sec)
	double new_speed;

	// Check for first run
	if (dir != runDirNow ||
		runSpeedNow == 0) {

		// Set to start speed
		new_speed = 5;
	}

	// Incriment speed
	else {
		new_speed = runSpeedNow <= maxSpeed - inc_speed ?
			runSpeedNow + inc_speed : runSpeedNow;
	}

	// Run motor
	RunMotor(dir, new_speed, "Override");

	// Print voltage and speed to LCD
	sprintf(vcc_str, "VCC=%0.2fV", vccNow);
	sprintf(speed_str, "VEL=%s%dcm/s", runDirNow == 'f' ? "->" : "<-", (int)runSpeedNow);
	PrintLCD(false, vcc_str, speed_str);
}

// SET WHATS CONTROLLING THE MOTOR
bool SetMotorControl(char set_to[], char called_from[])
{
	// VALUES:  ["None", "Halt", "Open", "MoveTo", "Bull", "Pid"]
	bool pass = false;

	// Set to/from "Halt"
	if (set_to == "Halt" || fc.motorControl == "Halt") {

		// Only "Halt" and "Quit" can set/unset "Halt"
		if (called_from == "Halt" || called_from == "Quit") {

			fc.motorControl = set_to;
		}
	}

	// If not in "Hault" mode
	else {

		// Can always set to "None"
		if (set_to == "None") {
			fc.motorControl = "None";
		}

		// Cannot unset "None" unless certain conditions met
		if (fc.motorControl == "None") {

			// Can still move robot if rat not in
			if (
				set_to == "MoveTo" &&
				!fc.isRatIn
				) {

				fc.motorControl = set_to;
			}

			// Can set to "Open" under these conditions
			else if (set_to == "Open") {

				// InitializeTracking can always unset "None"
				if (called_from == "InitializeTracking") {

					fc.motorControl = set_to;
				}

				// CheckBlockTimElapsed can unblock if tracking setup
				if (fc.isTrackingEnabled &&
					(called_from == "CheckBlockTimElapsed")
					) {

					fc.motorControl = set_to;
				}

			}

		}

		// "MoveTo" can only be set to "Open" or "None"
		else if (fc.motorControl == "MoveTo") {

			if (set_to == "Open") {

				fc.motorControl = set_to;
			}
		}

		// "Bull" can only be set to "MoveTo" or "Open"
		else if (fc.motorControl == "Bull") {

			if (set_to == "MoveTo" || set_to == "Open") {

				fc.motorControl = set_to;
			}
		}

		// Otherwise can set to anything
		else if (fc.motorControl == "Open" || fc.motorControl == "Pid") {

			fc.motorControl = set_to;
		}

	}

	// Return true if set to input
	if ((String)set_to == fc.motorControl) {
		pass = true;
	}

	// Store current controller
	DebugMotorControl(pass, set_to, called_from);

	return pass;
}

// BLOCK MOTOR TILL TIME ELLAPESED
void BlockMotorTill(int dt, char called_from[])
{
	// Set blocking and time
	fc.isBlockingTill = true;

	// Update time to hold till
	t_rewBlockMove = millis() + dt;

	// Remove all motor controll
	SetMotorControl("None", "BlockMotorTill");

	// Print blocking finished
	DebugMotorBocking("Blocking Motor for ", called_from, dt);
}

// CHECK IF TIME ELLAPESED
void CheckBlockTimElapsed()
{
	// Bail if not checking
	if (!fc.isBlockingTill) {
		return;
	}

	// Check that all 3 measures say rat has passed
	bool is_passed_feeder =
		fc.isTrackingEnabled &&
		kal.RatPos - (kal.RobPos + feedDist) > 0 &&
		Pos[0].posNow - (kal.RobPos + feedDist) > 0 &&
		Pos[2].posNow - (kal.RobPos + feedDist) > 0;

	// Check for time elapsed or rat moved at least 3cm past feeder
	if (millis() > t_rewBlockMove || is_passed_feeder)
	{
		// Print blocking finished
		DebugMotorBocking("Finished Blocking Motor at ", "CheckBlockTimElapsed");

		// Set flag to stop checking
		fc.isBlockingTill = false;

		// Retract arm early if rat ahead
		if (is_passed_feeder) {

			// Log/print
			DebugFlow("[CheckBlockTimElapsed] Unblocking Early because Rat Passed Feeder");

			// Retract feeder arm
			Reward.RetractFeedArm();
		}

		// Open up control
		SetMotorControl("Open", "CheckBlockTimElapsed");
	}

}

// DO SETUP TO BEGIN TRACKING
void InitializeTracking()
{
	// Local vars
	char str[200] = { 0 };
	static bool run_blink = false;
	static bool do_led_on = true;
	static int cnt_blink = 0;
	static uint32_t t_blink = 0;
	int n_laps = 0;
	double cm_diff = 0;
	double cm_dist = 0;

	// Bail if finished
	if (fc.isTrackingEnabled &&
		fc.isRatIn &&
		!run_blink) {
		return;
	}

	// Wait for new data
	if (fc.isRatIn &&
		!fc.isTrackingEnabled &&
		Pos[0].isNew &&
		Pos[2].isNew &&
		Pos[1].isNew)
	{

		// Log/Print
		DebugFlow("[InitializeTracking] RUNNING: Initialize Rat Tracking...");

		// Log/print rat and robot starting pos
		sprintf(str, "[InitializeTracking] Starting Positions: rat_vt=%0.2f rat_pixy=%0.2f rob_vt=%0.2f",
			Pos[0].posNow, Pos[2].posNow, Pos[1].posNow);
		DebugFlow(str);

		// Check that pos values make sense
		cm_diff = Pos[0].posNow - Pos[1].posNow;
		cm_dist = min((140 * PI) - abs(cm_diff), abs(cm_diff));

		// Rat should be ahead of robot and by no more than 90 deg
		if (cm_diff < 0 ||
			cm_dist >((140 * PI) / 4))
		{
			// Will have to run again with new samples
			Pos[0].Reset();
			Pos[2].Reset();
			Pos[1].Reset();
			DebugError("**WARNING** [InitializeTracking] Had to Reset Position Data");
			return;
		}

		// Set flag
		fc.isTrackingEnabled = true;

		// Reset ekf
		Pid.ResetEKF("InitializeTracking");

		// Don't start pid for manual sessions
		if (!fc.isManualSes)
		{
			// Open up motor control
			SetMotorControl("Open", "InitializeTracking");

			// Run Pid
			Pid.Run("InitializeTracking");
			DebugFlow("[InitializeTracking] PID STARTED");
		}

		// Initialize bulldoze
		if (fc.doBulldoze)
		{
			// Run from initial blocked mode
			Bull.TurnOn("InitializeTracking");
			DebugFlow("[InitializeTracking] BULLDOZE INITIALIZED");
		}

		// Log/Print
		DebugFlow("[InitializeTracking] FINISHED: Initialize Rat Tracking");

		// Set to run rat in blink
		run_blink = true;
	}

	// Perform rat in blink
	if (run_blink)
	{
		// Change led state
		if (millis() > t_blink + 100)
		{
			analogWrite(pin.RewLED_R, do_led_on ? 250 : 0);
			analogWrite(pin.TrackLED, do_led_on ? 250 : 0);

			// Update vars
			t_blink = millis();
			do_led_on = !do_led_on;
			cnt_blink = !do_led_on ? cnt_blink + 1 : cnt_blink;
		}

		// Reset LED
		if (cnt_blink >= 3)
		{
			analogWrite(pin.RewLED_R, rewLEDmin);
			analogWrite(pin.TrackLED, trackLEDduty);
			run_blink = false;
		}
	}
}

// CHECK IF RAT VT OR PIXY DATA IS NOT UPDATING
void CheckSampDT() {

	// Local vars
	char str[200] = { 0 };
	static int cnt_swap_vt = 0;
	static int cnt_swap_pixy = 0;
	bool do_swap_vt = false;
	bool do_swap_pixy = false;
	int dt_max_vt = 150;
	int dt_max_pixy = 100;
	int dt_vt = 0;
	int dt_pixy = 0;

	// Bail if rat not in yet
	if (!fc.isRatIn) {
		return;
	}

	// Compute dt
	dt_vt = millis() - Pos[0].t_msNow;
	dt_pixy = millis() - Pos[2].t_msNow;

	// Check VT 
	do_swap_vt =
		!Pos[0].isNew &&
		dt_vt > dt_max_vt &&
		dt_pixy < dt_vt;

	// Check pixy
	do_swap_pixy =
		!Pos[2].isNew &&
		dt_pixy > dt_max_pixy &&
		dt_vt < dt_pixy;

	// Bail if all good
	if (!do_swap_vt && !do_swap_pixy) {
		return;
	}

	// Use Pixy for VT data
	if (do_swap_vt && !do_swap_pixy) {
		Pos[0].SwapPos(Pos[2].posAbs, Pos[2].t_msNow);
		cnt_swap_vt++;
	}

	// Use VT for Pixy data
	else if (do_swap_pixy && !do_swap_vt) {
		Pos[2].SwapPos(Pos[0].posAbs, Pos[0].t_msNow);
		cnt_swap_pixy++;
	}

	// Log/print first and every 10 errors
	if (cnt_swap_vt + cnt_swap_pixy == 1 ||
		cnt_swap_vt + cnt_swap_pixy % 10 == 0) {

		// Swapped one for the other
		if (!(do_swap_vt && do_swap_pixy)) {
			sprintf(str, "**WARNING** [CheckSampDT] Swapped %s with %s Data: cnt_vt=%d cnt_pixy=%d dt_vt=%d dt_pixy=%d",
				do_swap_vt ? "VT" : "Pixy", do_swap_vt ? "Pixy" : "VT", cnt_swap_vt, cnt_swap_pixy, dt_vt, dt_pixy);
		}

		// Both sources dt too long
		else {
			sprintf(str, "**WARNING** [CheckSampDT] All Rat Tracking Hanging: cnt_vt=%d cnt_pixy=%d dt_vt=%d dt_pixy=%d",
				cnt_swap_vt, cnt_swap_pixy, dt_vt, dt_pixy);
		}

		// Log/print
		DebugError(str);
	}

}

// PROCESS PIXY STREAM
void CheckPixy() {

	// Local vars
	double px_rel = 0;
	double px_abs = 0;
	uint32_t t_px_ts = 0;
	double pixy_pos_y = 0;

	// Get new blocks
	uint16_t blocks = Pixy.getBlocks();

	// Bail in no new data
	if (!blocks) {
		return;
	}

	// Bail if rat not in or doing sym test
	if ((!fc.isRatIn || db.do_simRatTest) &&
		!db.do_posDebug) {
		return;
	}

	// Bail if robot not streaming yet
	if (!Pos[1].is_streamStarted) {
		return;
	}

	// Save time stamp
	t_px_ts = millis();

	// Get Y pos from last block and convert to CM
	pixy_pos_y = Pixy.blocks[blocks - 1].y;
	px_rel =
		pixyCoeff[0] * (pixy_pos_y * pixy_pos_y * pixy_pos_y * pixy_pos_y) +
		pixyCoeff[1] * (pixy_pos_y * pixy_pos_y * pixy_pos_y) +
		pixyCoeff[2] * (pixy_pos_y * pixy_pos_y) +
		pixyCoeff[3] * pixy_pos_y +
		pixyCoeff[4];

	// Scale to abs space with rob vt data
	px_abs = px_rel + Pos[1].posAbs;
	if (px_abs > (140 * PI)) {
		px_abs = px_abs - (140 * PI);
	}

	// Update pixy pos and vel
	Pos[2].UpdatePos(px_abs, t_px_ts);

	// Log/print first sample
	if (!Pos[2].is_streamStarted) {
		DebugFlow("[NetComCallbackVT] FIRST RAT PIXY RECORD");
		Pos[2].is_streamStarted = true;
	}

}

// UPDATE EKF
void UpdateEKF()
{
	// Local vars
	char str[200] = { 0 };
	static int cnt_error = 0;
	static bool is_nans_last = false;
	double rat_pos = 0;
	double rob_pos = 0;
	double rat_vel = 0;
	double rob_vel = 0;

	// Bail if all data not new
	if (!(Pos[0].isNew && Pos[2].isNew && Pos[1].isNew)) {
		return;
	}

	// Check EKF progress
	Pid.CheckEKF(millis());

	// Set pid update time
	Pid.SetUpdateTime(millis());

	// Set flag for reward
	if (fc.isTrackingEnabled) {
		Reward.is_ekfNew = true;
	}

	//----------UPDATE EKF---------
	double z[M] = {
		Pos[0].GetPos(),
		Pos[2].GetPos(),
		Pos[1].GetPos(),
		Pos[0].GetVel(),
		Pos[2].GetVel(),
		Pos[1].GetVel(),
	};

	// Run EKF
	ekf.step(z);

	// Update error estimate
	rat_pos = ekf.getX(0);
	rob_pos = ekf.getX(1);
	rat_vel = ekf.getX(2);
	rob_vel = ekf.getX(3);

	// Copy over values
	kal.RatPos = !isnan(rat_pos) ? rat_pos : kal.RatPos;
	kal.RobPos = !isnan(rob_pos) ? rob_pos : kal.RobPos;
	kal.RatVel = !isnan(rat_vel) ? rat_vel : kal.RatVel;
	kal.RobVel = !isnan(rob_vel) ? rob_vel : kal.RobVel;

	// Update count and time
	kal.t_update = millis();
	kal.cnt++;

	// Check for nan values
	if (isnan(rat_pos) || isnan(rob_pos) || isnan(rat_vel) || isnan(rob_vel)) {

		// Do not print consecutively
		if (!is_nans_last)
		{
			sprintf(str, "!!ERROR!! [UpdateEKF] \"nan\" EKF Output: Pos[0]=%0.2f|%0.2f Pos[2]=%0.2f|%0.2f Pos[1]=%0.2f|%0.2f",
				Pos[0].posNow, Pos[0].velNow, Pos[2].posNow, Pos[2].velNow, Pos[1].posNow, Pos[0].velNow);
			DebugError(str, true);

			// Set flag
			is_nans_last = true;
		}
	}
	else {
		is_nans_last = false;
	}

	// Check if too much time elapsed between updates
	if (millis() > kal.t_update + 250 &&
		kal.t_update != 0) {

		// Add to count
		cnt_error++;

		// Log/print first and every 10 errors
		if (cnt_error == 1 ||
			cnt_error % 10 == 0) {

			// Log/print warning
			sprintf(str, "**WARNING** [UpdateEKF] EKF Hanging: cnt_err=%d dt_update=%d", cnt_error, millis() - kal.t_update);
			DebugError(str);
		}
	}

}

#pragma endregion


#pragma region --------HARDWARE CONTROL---------

// CHECK FOR BUTTON INPUT
bool GetButtonInput()
{
	// Notes
	/*
	BUTTON 1:
	Short Hold: Trigger reward or retract feeder arm
	Long Hold:	Move robot forward
	BUTTON 2:
	Short Hold: open/close Reward solonoid
	Long Hold:	Move robot backward
	BUTTON 3:
	Short Hold: open/close EtOH solonoid
	Long Hold:	Turn LCD LED on/off
	BUTTON 4: Shorted to reset
	*/

	// Local vars
	char str[200] = { 0 };
	bool is_new_input = false;
	static bool is_pressed[3] = { false, false, false };
	static bool is_running[3] = { false, false, false };
	int do_flag_fun[3][2] = { { false,false },{ false,false },{ false,false } };
	static uint32_t t_debounce[3] = { millis() + 1000, millis() + 1000, millis() + 1000 };
	static uint32_t t_hold_min[3] = { 0, 0, 0 };
	static uint32_t t_long_hold[3] = { 0, 0, 0 };
	int dt_debounce[3] = { 100, 100, 100 };
	int dt_hold_min = 50;
	int dt_long_hold = 500;
	int btn_ind = 0;

	// Loop through and check each button
	for (int i = 0; i < 3; i++)
	{
		btn_ind = i;

		// Detect press
		if (
			digitalRead(pin.Btn[btn_ind]) == LOW &&
			!is_pressed[btn_ind])
		{
			// Exit if < debounce time has not passed
			if (t_debounce[btn_ind] > millis()) {
				return false;
			}

			// Get long hold time
			t_long_hold[btn_ind] = millis() + dt_long_hold - dt_hold_min;

			// Set flag
			is_pressed[btn_ind] = true;

			// Log/print
			sprintf(str, "[GetButtonInput] Pressed button %d", i);
			DebugFlow(str);
		}

		// Check released
		else if (
			is_pressed[btn_ind] &&
			!is_running[btn_ind])
		{

			// Bail if not dt hold min
			t_hold_min[btn_ind] = t_hold_min[btn_ind] == 0 ? millis() + dt_hold_min : t_hold_min[btn_ind];
			if (millis() < t_hold_min[btn_ind]) {
				return false;
			}

			// Check for short hold
			bool is_short_hold =
				digitalRead(pin.Btn[btn_ind]) == HIGH &&
				millis() < t_long_hold[btn_ind];

			// Check for long hold
			bool is_long_hold = millis() > t_long_hold[btn_ind];

			// Set flag for either condition
			if (is_short_hold || is_long_hold) {

				// Run short hold function
				if (is_short_hold) {
					do_flag_fun[btn_ind][0] = true;
				}

				// Run long hold function
				if (is_long_hold) {
					do_flag_fun[btn_ind][1] = true;
				}

				// Make tracker LED brighter
				analogWrite(pin.TrackLED, 255);

				// Set running flag
				is_running[btn_ind] = true;

				// Flag input rcvd
				is_new_input = true;

				// Log/print
				sprintf(str, "[GetButtonInput] Triggered button %d", i);
				DebugFlow(str);
			}
		}

		// Check if needs to be reset
		else if (digitalRead(pin.Btn[btn_ind]) == HIGH &&
			is_pressed[btn_ind]) {

			if (is_running[btn_ind] ||
				millis() > t_long_hold[btn_ind]) {

				// Reset flags etc
				t_debounce[btn_ind] = millis() + dt_debounce[btn_ind];
				analogWrite(pin.TrackLED, trackLEDduty);
				is_running[btn_ind] = false;
				t_hold_min[btn_ind] = 0;
				t_long_hold[btn_ind] = 0;
				is_pressed[btn_ind] = false;

				// Log/print
				sprintf(str, "[GetButtonInput] Reset button %d", i);
				DebugFlow(str);
			}
		}

	}

	// Bail if no new flags
	if (!is_new_input) {
		return false;
	}

	// Set button 1 function flag
	if (do_flag_fun[0][0]) {

		// Reward or retract feeder arm
		if (!Reward.isArmExtended) {
			fc.doBtnRew = true;
			DebugFlow("[GetButtonInput] Button 0 \"fc.doBtnRew\" Triggered");
		}
		else {
			Reward.RetractFeedArm();
			DebugFlow("[GetButtonInput] Button 0 \"Reward.RetractFeedArm()\" Triggered");
		}
	}

	else if (do_flag_fun[0][1]) {
		fc.doMoveRobFwd = true;
		DebugFlow("[GetButtonInput] Button 0 \"fc.doMoveRobFwd\" Triggered");
	}
	// Set button 2 function flag
	else if (do_flag_fun[1][0]) {
		fc.doRewSolStateChange = true;
		DebugFlow("[GetButtonInput] Button 1 \"fc.doRewSolStateChange\" Triggered");
	}
	else if (do_flag_fun[1][1]) {
		fc.doMoveRobRev = true;
		DebugFlow("[GetButtonInput] Button 1 \"fc.doMoveRobRev\" Triggered");
	}
	// Set button 3 function flag
	else if (do_flag_fun[2][0]) {
		fc.doEtOHSolStateChange = true;
		DebugFlow("[GetButtonInput] Button 2 \"fc.doEtOHSolStateChange\" Triggered");
	}
	else if (do_flag_fun[2][1]) {
		fc.doChangeLCDstate = true;
		DebugFlow("[GetButtonInput] Button 2 \"fc.doChangeLCDstate\" Triggered");
	}

	// Reset flag
	for (int i = 0; i < 3; i++) {

		// Turn on LCD if any fucntion flagged durring manual mode
		if (fc.isManualSes &&
			!fc.doChangeLCDstate &&
			(do_flag_fun[i][0] || do_flag_fun[i][1])) {

			ChangeLCDlight(25);
		}

		// Reset flag
		do_flag_fun[i][0] = false;
		do_flag_fun[i][1] = false;
	}

	// Return flag
	return true;
}

// OPEN/CLOSE REWARD SOLENOID
void OpenCloseRewSolenoid()
{
	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_Rew);
	char etoh_str[100] = { 0 };
	char rew_str[100] = { 0 };

	// Change state
	is_sol_open = !is_sol_open;

	// Store open time
	if (is_sol_open) {
		t_solOpen = millis();
	}
	else {
		t_solClose = millis();
	}

	// Open/close solenoid
	digitalWrite(pin.Rel_Rew, is_sol_open);

	// Print etoh and rew sol state to LCD
	sprintf(rew_str, "Food   %s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN  " : "CLOSED");
	sprintf(etoh_str, "EtOH   %s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN  " : "CLOSED");
	PrintLCD(true, rew_str, etoh_str, 's');

}

// OPEN/CLOSE EtOH SOLENOID
void OpenCloseEtOHSolenoid()
{
	// Local vars
	byte is_sol_open = digitalRead(pin.Rel_EtOH);
	char etoh_str[100] = { 0 };
	char rew_str[100] = { 0 };

	// Change state
	is_sol_open = !is_sol_open;

	// Open/close solenoid
	digitalWrite(pin.Rel_EtOH, is_sol_open);

	// Store time and make sure periodic drip does not run
	if (is_sol_open) {
		t_solOpen = millis();
		fc.doEtOHRun = false;
	}
	else {
		t_solClose = millis();
		fc.doEtOHRun = true;
	}

	// Print etoh and rew sol state to LCD
	sprintf(rew_str, "Food   %s", digitalRead(pin.Rel_Rew) == HIGH ? "OPEN  " : "CLOSED");
	sprintf(etoh_str, "EtOH   %s", digitalRead(pin.Rel_EtOH) == HIGH ? "OPEN  " : "CLOSED");
	PrintLCD(true, rew_str, etoh_str, 's');

}

// CHECK FOR ETOH UPDATE
void CheckEtOH()
{
	// Local vars
	char str[200] = { 0 };
	byte is_sol_open = digitalRead(pin.Rel_EtOH);
	static float etoh_dist_start = 0; // (cm)
	static float etoh_dist_diff = 0; // (cm)
	bool do_open = false;
	bool do_close = false;

	// Bail if etoh should not be run
	if (!fc.doEtOHRun) {
		return;
	}

	// Get distance traveled
	etoh_dist_diff = abs(kal.RobPos - etoh_dist_start);

	if (!is_sol_open) {

		// Open if dt run has ellapsed and robot has moved
		do_open = millis() > (t_solOpen + dt_delEtOH[fc.isSesStarted ? 0 : 1]);

		// Open if robot has covered half the trak
		do_open = do_open ||
			etoh_dist_diff > (140 * PI) / 2;

		// Open only if motor not running
		do_open =
			do_open &&
			(CheckAD_Status(adR_stat, "MOT_STATUS") == 0 &&
				CheckAD_Status(adF_stat, "MOT_STATUS") == 0);
	}

	// Close if open and dt close has ellapsed
	else {
		do_close = millis() > (t_solOpen + dt_durEtOH[fc.isSesStarted ? 0 : 1]);
	}

	// Check if sol should be opened
	if (do_open) {
		// Open solenoid
		digitalWrite(pin.Rel_EtOH, HIGH);


		// Store current time and pos
		t_solOpen = millis();
		etoh_dist_start = kal.RobPos;
	}

	// Check if sol should be closed
	else if (do_close) {
		// Close solenoid
		digitalWrite(pin.Rel_EtOH, LOW);

		// Print to debug
		sprintf(str, "[CheckEtOH] Ran EtOH: dt_close=%d dt_open=%d", t_solOpen - t_solClose, millis() - t_solOpen);
		DebugFlow(str);

		// Store current time and pos
		t_solClose = millis();
	}
}

// CHECK BATTERY VALUES
float CheckBattery(bool force_check)
{
	// Local vars
	char str[200] = { 0 };
	static uint32_t t_vcc_update = 0;
	static uint32_t t_vcc_send = 0;
	static uint32_t t_vcc_print = 0;
	static uint32_t t_check_str = millis();
	static uint32_t t_check_end = millis() + dt_vccCheck;
	static uint32_t t_ic_update = 0;
	static float vcc_arr[10] = { 0 };
	static float vcc_avg = 0;
	static float vcc_last = 0;
	static int n_samples = 0;
	uint32_t vcc_bit_in = 0;
	uint32_t ic_bit_in = 0;
	float vcc_in = 0;
	float vcc_sum = 0;
	bool is_mot_off = false;
	byte do_shutdown = false;

	// Calculate current
	if (millis() > t_ic_update + dt_icUpdate) {
		ic_bit_in = analogRead(pin.BatIC);
		icNow = 36.7*(((float)ic_bit_in * (3.3 / 1024)) / 3.3) - 18.3;
		t_ic_update = millis();
	}

	// Not time to check
	if (millis() < t_check_str &&
		!force_check) {

		// Turn off relay
		digitalWrite(pin.Rel_Vcc, LOW);
	}

	// Done checking
	else if (millis() > t_check_end &&
		!force_check) {

		// Reset timers
		t_check_str = millis() + dt_vccUpdate;
		t_check_end = t_check_str + dt_vccCheck;

		// Turn off relay
		digitalWrite(pin.Rel_Vcc, LOW);
	}

	// Only sample if motor off
	else if ((CheckAD_Status(adR_stat, "MOT_STATUS") != 0 ||
		CheckAD_Status(adF_stat, "MOT_STATUS") != 0)) {

		// Turn off relay
		digitalWrite(pin.Rel_Vcc, LOW);
	}

	// Time to check
	else {

		// Turn on relay and skip this run to alow switch to open
		if (digitalRead(pin.Rel_Vcc) == LOW) {

			// Turn on switch
			digitalWrite(pin.Rel_Vcc, HIGH);
			
			// Bail
			return vccNow;
		}

		// Calculate voltage
		vcc_bit_in = analogRead(pin.BatVcc);
		vcc_in = (float)vcc_bit_in * bit2volt;
		vcc_sum = 0;
		// Shift array and compute average
		for (int i = 99; i > 0; i--) {
			batVoltArr[i] = batVoltArr[i - 1];
			vcc_sum += batVoltArr[i];
		}
		batVoltArr[0] = vcc_in;
		vcc_avg = vcc_sum / 99;

		// Return 0 till array full
		n_samples = n_samples < 100 ? n_samples + 1 : n_samples;
		if (n_samples < 100) {
			return 0;
		}

		// Store new voltage level
		vcc_last = vccNow;
		vccNow = vcc_avg;

		// Store time
		t_vcc_update = millis();

		// Add to array
		for (int i = 0; i < 10 - 1; i++) {
			vcc_arr[i] = vcc_arr[i + 1];
		}
		vcc_arr[9] = vccNow;

	}

	// Send and print if min dt ellapsed
	if (fc.doSendVCC &&
		!fc.doBlockVccSend &&
		millis() > t_vcc_send + dt_vccSend) {

		// Send
		QueuePacket(&r2c, 'J', vccNow, 0, 0, 0, false);

		// Store time
		t_vcc_send = millis();
	}

	// Log/print voltage and current
	if (millis() > t_vcc_print + dt_vccPrint) {

		// Log/print voltage and current
		sprintf(str, "[GetBattVolt] VCC Change: vcc=%0.2fV ic=%0.2fA dt_check=%d",
			vccNow, icNow, millis()- t_vcc_update);
		DebugFlow(str);

		// Store time
		t_vcc_print = millis();
	}

	// Print voltage and current to LCD if voltage changed
	if (round(vccNow * 100) != round(vcc_last * 100)) {
		char vcc_str[100];
		char speed_str[100];
		sprintf(vcc_str, "VCC=%0.2fV", vccNow);
		sprintf(speed_str, "IC=%0.2fA", icNow);
		PrintLCD(false, vcc_str, speed_str);
	}

	// Check if voltage critically low
	do_shutdown = true;
	for (int i = 0; i < 10 - 1; i++) {
		do_shutdown = do_shutdown && vcc_arr[i] < vccCutoff && vcc_arr[i] > 0 ?
			true : false;
	}

	// Perform shutdown
	if (do_shutdown) {
		// Run error hold then shutdown after 5 min
		sprintf(str, "BATT LOW %0.2fV", vccNow);
		RunErrorHold(str, 60000);
	}

	// Return battery voltage
	return vccNow;
}

// TURN LCD LIGHT ON/OFF
void ChangeLCDlight(uint32_t duty)
{
	// Local vars
	char str[200] = { 0 };

	// Check if new duty given
	if (duty == 256) {
		fc.isLitLCD = !fc.isLitLCD;
		duty = fc.isLitLCD ? 50 : 0;
	}
	else {
		fc.isLitLCD = duty > 0;
	}

	// Set LCD duty
	analogWrite(pin.Disp_LED, duty);

	// Log/print
	sprintf(str, "[ChangeLCDlight] Set LCD Light: is_lit=%s duty=%d",
		fc.isLitLCD ? "true" : "false", duty);
	DebugFlow(str);
}

// QUIT AND RESTART ARDUINO
void QuitSession()
{
	// Stop all movement
	HardStop("QuitSession");
	Pid.Stop("QuitSession");
	Bull.TurnOff("QuitSession");
	delayMicroseconds(100);
	// Restart Arduino
	REQUEST_EXTERNAL_RESET;
}

#pragma endregion


#pragma region --------DEBUGGING---------

// CHECK LOOP TIME AND MEMORY
void CheckLoop()
{
	// Local static vars
	char str[300] = { 0 };
	static bool first_log = true;
	static uint32_t t_loop_last = millis();
	static int dt_loop = 0;
	static int dt_loop_last = 0;
	static int c_buff_rx_last = 0;
	static int c_buff_tx_last = 0;
	static int a_buff_rx_last = 0;
	static int a_buff_tx_last = 0;
	bool is_dt_change = false;
	bool is_loop_error = false;
	bool is_buff_flooding = false;
	int c_buff_rx = 0;
	int c_buff_tx = 0;
	int a_buff_rx = 0;
	int a_buff_tx = 0;

	// Keep short count of loops
	cnt_loop_short = cnt_loop_short < 999 ? cnt_loop_short + 1 : 1;

	// Bail till ses started
	if (!fc.isSesStarted) {
		return;
	}

	// Get total data left in buffers
	c_buff_rx = c2r.port.available();
	c_buff_tx = SERIAL_BUFFER_SIZE - 1 - c2r.port.availableForWrite();
	a_buff_rx = a2r.port.available();
	a_buff_tx = SERIAL_BUFFER_SIZE - 1 - a2r.port.availableForWrite();

	// Track total loops
	cnt_loop_tot++;

	// Compute current loop dt
	dt_loop = millis() - t_loop_last;
	t_loop_last = millis();

	// Check for errors durring this or last loop
	is_loop_error = db.isErrLoop;
	db.isErrLoop = false;

	// Check long loop time
	is_dt_change = dt_loop > 60;

	// Check if either buffer more than half full
	is_buff_flooding =
		c_buff_rx >= 96 ||
		c_buff_tx >= 96 ||
		a_buff_rx >= 96 ||
		a_buff_tx >= 96;


	if (
		first_log ||
		is_loop_error ||
		is_dt_change ||
		is_buff_flooding
		)
	{

		// Skip first 1k runs
		if (cnt_loop_tot >= 1000) {

			// Get message id
			char msg[50];
			if (first_log) {
				sprintf(msg, "[CheckLoop] BASELINE:");
			}
			else {
				sprintf(msg, "**CHANGE DETECTED** [CheckLoop] |%s%s%s:",
					is_dt_change ? "Loop DT Change|" : "",
					is_loop_error ? "Error Loop|" : "",
					is_buff_flooding ? "Buffer Flooding|" : "");
			}

			// Log/print message
			sprintf(str, "%s cnt_loop:%d|%d dt_loop=%d|%d c_rx=%d|%d c_tx=%d|%d a_rx=%d|%d a_tx=%d|%d",
				msg, cnt_loop_short, cnt_loop_tot, dt_loop, dt_loop_last, c_buff_rx, c_buff_rx_last, c_buff_tx, c_buff_tx_last, a_buff_rx, a_buff_rx_last, a_buff_tx, a_buff_tx_last);
			DebugFlow(str);

			// Set flag
			first_log = false;
		}
	}

	// Store vars
	dt_loop_last = dt_loop;
	c_buff_rx_last = c_buff_rx;
	c_buff_tx_last = c_buff_tx;
	a_buff_rx_last = a_buff_rx;
	a_buff_tx_last = a_buff_tx;
}

// LOG/PRINT MAIN EVENT
void DebugFlow(char msg[], uint32_t t)
{
	// Local vars
	bool do_print = db.print_flow && (db.Console || db.LCD);
	bool do_log = db.log_flow && db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg, t);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(msg, t);
	}

}

// LOG/PRINT ERRORS
void DebugError(char msg[], bool is_error, uint32_t t)
{
	// Local vars
	bool do_print = db.print_errors && (db.Console || db.LCD);
	bool do_log = db.log_errors && db.Log;

	// Set error flag
	db.isErrLoop = true;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg, t);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(msg, t);
	}

	// Store error info
	if (is_error) {
		err_line[cnt_err < 100 ? cnt_err : 99] = Log.cnt_logsStored;
		cnt_err++;
	}
	else {
		warn_line[cnt_warn < 100 ? cnt_warn : 99] = Log.cnt_logsStored;
		cnt_warn++;
	}
}

// LOG/PRINT MOTOR CONTROL DEBUG STRING
void DebugMotorControl(bool pass, char set_to[], char called_from[])
{
	// Local vars
	char str[200] = { 0 };
	bool do_print = db.print_motorControl && (db.Console || db.LCD);
	bool do_log = db.log_motorControl && db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	sprintf(str, "%s[DebugMotorControl] Change %s: set_in=%s set_out=%s [%s]",
		!pass ? "**WARNING** " : "", pass ? "Succeeded" : "Failed", set_to, fc.motorControl.c_str(), called_from);

	// Add to print queue
	if (do_print) {
		QueueDebug(str, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(str, millis());
	}

}

// LOG/PRINT MOTOR BLOCKING DEBUG STRING
void DebugMotorBocking(char msg[], char called_from[], uint32_t t)
{
	// Local vars
	char str[200] = { 0 };
	bool do_print = db.print_motorControl && (db.Console || db.LCD);
	bool do_log = db.log_motorControl && db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Log/print
	sprintf(str, "[DebugMotorBocking] %s %lu ms [%s]", msg, t, called_from);

	// Add to print queue
	if (do_print) {
		QueueDebug(str, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(str, millis());
	}

}

// LOG/PRINT MOTOR SPEED CHANGE
void DebugRunSpeed(String agent, double speed_last, double speed_now)
{
	// Local vars
	char str[200] = { 0 };
	bool do_print = db.print_runSpeed && (db.Console || db.LCD);
	bool do_log = db.log_runSpeed && db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Log/print
	char agent_str[50];
	agent.toCharArray(agent_str, 50);
	sprintf(str, "[RunMotor] Changed Motor Speed: agent=%s speed_last=%0.2f speed_new=%0.2f",
		agent_str, speed_last, speed_now);

	// Add to print queue
	if (do_print) {
		QueueDebug(str, millis());
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(str, millis());
	}

}

// LOG/PRINT RECIEVED PACKET DEBUG STRING
void DebugRcvd(R4 *r4, char msg[], bool is_repeat)
{
	// Local vars
	char msg_out[300] = { 0 };
	char str[300] = { 0 };
	bool do_print = false;
	bool do_log = false;

	// Get print status
	do_print =
		((r4->instID == 'c' && ((db.print_c2r && r4->idNow != 'P') || (db.print_rcvdVT && r4->idNow == 'P'))) ||
		(r4->instID == 'a' && db.print_a2r)) &&
			(db.Console || db.LCD);
	do_log =
		((r4->instID == 'c' && db.log_c2r && r4->idNow != 'P') ||
		(r4->instID == 'a' && db.log_a2r)) &&
		db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Check if this is a repeat
	if (!is_repeat) {
		sprintf(msg_out, "   [RCVD] %c2r: %s", r4->instID, msg);
	}
	else {
		sprintf(msg_out, "   [*RE-RCVD*] %c2r: cnt=%d %s", r4->instID, r4->cnt_repeat, msg);
	}

	// Add samp dt for pos data
	if (r4->idNow == 'P') {
		U.f = r4->dat[2];
		sprintf(str, " ts_int=%d dt_samp=%d",
			U.i32, millis() - Pos[cmd.vtEnt].t_msNow);
		strcat(msg_out, str);
	}

	// Add to print queue
	if (do_print) {
		QueueDebug(msg_out, r4->t_rcvd);
	}

	// Add to log queue
	if (do_log) {
		Log.QueueLog(msg_out, r4->t_rcvd);
	}

}

// LOG/PRINT SENT PACKET DEBUG STRING
void DebugSent(R2 *r2, char msg[], bool is_repeat)
{
	// Local vars
	char msg_out[300] = { 0 };
	char str[300] = { 0 };
	bool is_resend = false;
	bool do_print = false;
	bool do_log = false;

	// Set pointer to struct
	if (r2->instID == 'c') {
		r2 = &r2c;
	}
	else if (r2->instID == 'a') {
		r2 = &r2a;
	}

	// Get print status
	do_print = ((db.print_r2c && r2->instID == 'c') || (db.print_r2a && r2->instID == 'a')) &&
		(db.Console || db.LCD);
	do_log = ((db.log_r2c && r2->instID == 'c') || (db.log_r2a && r2->instID == 'a')) &&
		db.Log;

	// Bail if neither set
	if (!do_print && !do_log) {
		return;
	}

	// Check if this is a repeat
	if (!is_repeat) {
		sprintf(msg_out, "   [SENT] r2%c: %s", r2->instID, msg);
	}
	else {
		sprintf(msg_out, "   [*RE-SENT*] r2%c: cnt=%d %s", r2->instID, r2->cnt_repeat, msg);
	}

	// Concatinate strings
	strcat(msg, str);

	// Store
	if (do_print) {
		QueueDebug(msg_out, r2->t_sent);
	}

	if (do_log) {
		Log.QueueLog(msg_out, r2->t_sent);
	}

}

// STORE STRING FOR PRINTING
void QueueDebug(char msg[], uint32_t t)
{
	// Local vars
	uint32_t t_m = 0;
	float t_s = 0;
	char msg_copy[300] = { 0 };
	char str_tim[100] = { 0 };

	// Update printQueue ind
	printQueueIndStore++;

	// Check if ind should roll over 
	if (printQueueIndStore == printQueueSize) {

		// Reset queueIndWrite
		printQueueIndStore = 0;
	}

	// Check if overfloweed
	if (printQueue[printQueueIndStore][0] != '\0')
	{

		// Get list of empty entries
		char queue_state[printQueueSize + 1];
		for (int i = 0; i < printQueueSize; i++) {
			queue_state[i] = printQueue[i][0] == '\0' ? '0' : '1';
		}
		queue_state[printQueueSize] = '\0';

		// Store overflow error instead
		sprintf(msg_copy, "**WARNING** [QueueDebug] PRINT QUEUE OVERFLOWED: printQueueIndStore=%d printQueueIndRead=%d queue_state=|%s|",
			printQueueIndStore, printQueueIndRead, queue_state);

		// Set queue back so overflow will write over last print
		printQueueIndStore = printQueueIndStore - 1 >= 0 ? printQueueIndStore - 1 : printQueueSize - 1;

		// Log error
		if (db.log_errors && db.Log) {
			Log.QueueLog(msg_copy, t);
		}

	}
	else {
		// Copy message
		for (int i = 0; i < strlen(msg); i++)
		{
			msg_copy[i] = msg[i];
		}
		msg_copy[strlen(msg)] = '\0';
	}

	// Get sync correction
	t_m = t - t_sync;

	// Convert to seconds
	t_s = (float)(t_m) / 1000.0f;

	// Make time string
	sprintf(str_tim, "[%0.3f][%d]", t_s, cnt_loop_short);

	// Add space after time
	char spc[50] = { 0 };
	char arg[50] = { 0 };
	sprintf(arg, "%%%ds", 20 - strlen(str_tim));
	sprintf(spc, arg, '_');

	// Put it all together
	sprintf(printQueue[printQueueIndStore], "%s%s%s\n", str_tim, spc, msg_copy);

}

// PRINT DB INFO
bool PrintDebug()
{

	// Bail if nothing in queue
	if (printQueueIndRead == printQueueIndStore &&
		printQueue[printQueueIndStore][0] == '\0') {
		return false;
	}

	// Itterate send ind
	printQueueIndRead++;

	// Check if ind should roll over 
	if (printQueueIndRead == printQueueSize) {
		printQueueIndRead = 0;
	}

	// Print
	SerialUSB.print(printQueue[printQueueIndRead]);

	// Set entry to null
	printQueue[printQueueIndRead][0] = '\0';

	// Return success
	return true;
}

// FOR PRINTING TO LCD
void PrintLCD(bool do_block, char msg_1[], char msg_2[], char f_siz)
{

	// Check if printing blocked
	if (do_block) {
		fc.doBlockWriteLCD = false;
	}
	else if (fc.doBlockWriteLCD) {
		return;
	}

	// Reset
	LCD.clrScr();
	LCD.invert(true);
	LCD.setFont(SmallFont);

	// Print
	if (msg_2[0] != '\0')
	{
		LCD.print(msg_1, 5, 24-4);
		LCD.print(msg_2, 5, 24+4);
	}
	else {
		LCD.print(msg_1, 5, 24);
	}


	// Prevent overwrite till cleared
	if (do_block) {
		fc.doBlockWriteLCD = true;
	}
}

// CLEAR LCD
void ClearLCD()
{
	// Clear
	LCD.clrScr();

	// Stop blocking LCD log
	fc.doBlockWriteLCD = false;
}

// PRINT SPECIAL CHARICTERS
char* PrintSpecialChars(char chr, bool do_show_byte)
{
	/*
	Character					ASCII Representation	ASCII Value		Escape Sequence
	---------                   --------------------    -----------     ---------------
	Newline						NL(LF)					10				\n
	Horizontal tab				HT						9				\t
	Vertical tab				VT						11				\v
	Backspace					BS						8				\b
	Carriage return				CR						13				\r
	Formfeed					FF						12				\f
	Alert						BEL						7				\a
	Backslash					\						92				\\
	Question mark				?						63				\?
	Single quotation mark		'						39				\'
	Double quotation mark		"						34				\"
	Null character				NUL						0				\0
	*/

	// Local vars
	static char str[10] = { 0 };
	byte b = chr;

	for (int i = 0; i < 10; i++) {
		str[i] = '\0';
	}

	if (!do_show_byte)
	{
		// Get normal and special chars in quots
		switch (b) {
		case 10: sprintf(str, "\\n\n"); break;
		case 9: sprintf(str, "\\t\t"); break;
		case 11: sprintf(str, "\\v\v"); break;
		case 8: sprintf(str, "\\b\b"); break;
		case 13: sprintf(str, "\\r\r"); break;
		case 12: sprintf(str, "\\f\f"); break;
		case 7: sprintf(str, "\\a\a"); break;
		case 92: sprintf(str, "\\\\"); break;
		case 63: sprintf(str, "\\?\?"); break;
		case 39: sprintf(str, "\\'\""); break;
		case 34: sprintf(str, "\\\"\""); break;
		case 0: sprintf(str, "\\0\0"); break;
		default: sprintf(str, "%c", b); break;
		}
	}
	else
	{
		switch (b) {
		case 10: sprintf(str, "[10]\'\\n\'\n"); break;
		case 9: sprintf(str, "[9]\'\\t\'\t"); break;
		case 11: sprintf(str, "[11]\'\\v\'\v"); break;
		case 8: sprintf(str, "[8]\'\\b\'\b"); break;
		case 13: sprintf(str, "[13]\'\\r\'\r"); break;
		case 12: sprintf(str, "[12]\'\\f\'\f"); break;
		case 7: sprintf(str, "[7]\'\\a\'\a"); break;
		case 92: sprintf(str, "[92]\'\\\'\\"); break;
		case 63: sprintf(str, "[63]\'\\?\'\?"); break;
		case 39: sprintf(str, "[39]\'\\'\'\""); break;
		case 34: sprintf(str, "[34]\'\\\"\'\""); break;
		case 0: sprintf(str, "[0]\'\\0\'\0"); break;
		default: sprintf(str, "[%d]\'%c\'", b, b); break;
		}
	}

	return str;
}

// CHECK AUTODRIVER BOARD STATUS
int CheckAD_Status(uint16_t stat_reg, String stat_str)
{
	// Local vars
	String status_list[16] =
	{
		"HiZ",
		"BUSY",
		"SW_F",
		"SW_EVN",
		"DIR",
		"MOT_STATUS",
		"MOT_STATUS",
		"NOTPERF_CMD",
		"WRONG_CMD",
		"UVLO",
		"TH_WRN",
		"TH_SD",
		"OCD",
		"STEP_LOSS_A",
		"STEP_LOSS_B",
		"SCK_MOD"
	};
	byte bit_ind[2] = { 0, 0 };
	bool is_bit_set[2] = { false, false };
	uint16_t bit_val = 0x0;

	// Get id ind
	for (int i = 0; i < 16; i++)
	{
		if (stat_str == status_list[i])
		{
			bit_ind[is_bit_set[0] ? 1 : 0] = i;
			is_bit_set[is_bit_set[0] ? 1 : 0] = true;
		}
	}

	// Get bit value
	int n_loop = is_bit_set[1] ? 2 : 1;
	for (int i = 0; i < n_loop; i++)
	{
		uint16_t mask = 1 << bit_ind[i];
		uint16_t masked_n = stat_reg & mask;
		uint16_t k = masked_n >> bit_ind[i];
		bit_val |= bit_val & ~(1 << i) | (k << i);
	}

	// return bit value
	return (int)bit_val;
}

// LOG TRACKING
void LogTrackingData()
{
	// Bail if not doing any pos logging
	if (!db.log_pos) {
		return;
	}

	// Local vars
	char str[300] = { 0 };
	static int cnt_last = 0;
	static int16_t pos_hist[10][40] = { { 0 } };
	static int hist_ind = 0;
	static char id_str[10][50] = { { "Rat VT Pos: |" } ,{ "Rat Px Pos: |" } ,{ "Rob VT Pos: |" } ,{ "Rat EKF Pos: |" } ,{ "Rob EKF Pos: |" },
	{ "Rat VT Vel: |" } ,{ "Rat Px Vel: |" } ,{ "Rob VT Vel: |" } ,{ "Rat EKF Vel: |" } ,{ "Rob EKF Vel: |" } };
	static bool do_log[10] = { db.log_pos_rat_vt, db.log_pos_rat_pixy, db.log_pos_rob_vt, db.log_pos_rat_ekf, db.log_pos_rob_ekf,
		db.log_vel_rat_vt, db.log_vel_rat_pixy, db.log_vel_rob_vt, db.log_vel_rat_ekf, db.log_vel_rob_ekf };

	// Bail in ekf not new or not logging
	if (kal.cnt == cnt_last ||
		(!do_log[0] && !do_log[1] && !do_log[2] && !do_log[3])) {
		return;
	}

	// Initilazie store time
	static uint32_t t_last_log = kal.t_update;

	// Store pos values
	if (do_log[0]) {
		pos_hist[0][hist_ind] = Pos[0].posNow;
	}
	if (do_log[1]) {
		pos_hist[1][hist_ind] = Pos[2].posNow;
	}
	if (do_log[2]) {
		pos_hist[2][hist_ind] = Pos[1].posNow;
	}
	if (do_log[3]) {
		pos_hist[3][hist_ind] = kal.RatPos;
	}
	if (do_log[4]) {
		pos_hist[4][hist_ind] = kal.RobPos;
	}

	// Store vel values
	if (do_log[5]) {
		pos_hist[5][hist_ind] = Pos[0].velNow;
	}
	if (do_log[6]) {
		pos_hist[6][hist_ind] = Pos[2].velNow;
	}
	if (do_log[7]) {
		pos_hist[7][hist_ind] = Pos[1].velNow;
	}
	if (do_log[8]) {
		pos_hist[8][hist_ind] = kal.RatVel;
	}
	if (do_log[9]) {
		pos_hist[9][hist_ind] = kal.RobVel;
	}

	// Store counts
	hist_ind++;
	cnt_last = kal.cnt;

	// Check if hist filled
	if (millis() >= t_last_log + 1000 || hist_ind == 40) {

		// Store values in respective string
		for (int i = 0; i < 10; i++) {

			if (!do_log[i]) {
				continue;
			}

			// Add identfiyer string
			sprintf(str, "%s", id_str[i]);

			// Store each value in string
			char s[10];
			for (int j = 0; j < hist_ind; j++) {

				sprintf(s, "%d|", pos_hist[i][j]);
				strcat(str, s);
			}

			// Log
			Log.QueueLog(str, kal.t_update);
		}

		// Reset vals
		hist_ind = 0;
		t_last_log = kal.t_update;
	}
}

// SEND TEST PACKET
void TestSendPack(R2 *r2, char id, float dat1, float dat2, float dat3, uint16_t pack, bool do_conf)
{
	// EXAMPLE:
	/*
	// TEMP
	static uint32_t t_s = 0;
	static int send_cnt = 0;
	static uint16_t pack = 0;
	if (send_cnt == 0 && millis()>t_s + 30) {
		pack++;
		TestSendPack(&r2c, 'Z', 0, 0, 0, 1, true);
		t_s = millis();
		send_cnt++;
	}
	*/

	//// Only send once
	//if (cnt_loop_short > 0 || cnt_loop_tot > 0) {
	//	return;
	//}

	// Queue packet
	QueuePacket(r2, id, dat1, dat2, dat3, pack, do_conf);

	// Fuck with packet
	/*
	STORE DATA FOR CHEETAH DUE
	FORMAT: [0]head, [1]id, [2:4]dat, [5:6]pack, [7]do_conf, [8]footer, [9]targ
	*/
	//r2->sendQueue[sendQueueInd + 1][8] = 'i';

	// Send packet
	SendPacket(r2);

	// Block resend
	R4 *r4;
	if (r2->instID == 'c') {
		r4 = &c2r;
	}
	else if (r2->instID == 'a') {
		r4 = &a2r;
	}
	r2c.doRcvCheck[CharInd(id, r4->id, r4->lng)] = false;

	// Print everything
	while (PrintDebug());
}

// HOLD FOR CRITICAL ERRORS
void RunErrorHold(char msg[], uint32_t t_kill)
{
	// Local vars
	char str[200] = { 0 };
	static uint32_t t_shutdown = t_kill > 0 ? millis() + t_kill : 0;
	int duty[2] = { 255, 0 };
	bool do_led_on = true;
	int dt = 250;
	float t_s = 0;

	// Print anything left in print queue
	while (PrintDebug());

	// Turn on LCD LED
	ChangeLCDlight(100);

	// Get time seconds
	t_s = (float)(millis() - t_sync) / 1000.0f;

	// Print error
	sprintf(str, "ERROR %0.2fs", t_s);
	PrintLCD(true, msg, str, 't');

	// Loop indefinaitely
	while (true)
	{
		// Flicker lights
		analogWrite(pin.RewLED_R, duty[(int)do_led_on]);
		analogWrite(pin.TrackLED, duty[(int)do_led_on]);
		delay(dt);
		do_led_on = !do_led_on;

		// Check if should shutdown
		if (t_shutdown > 0 && millis() > t_shutdown) {
			// Log/print
			sprintf(str, "!!ERROR!! [RunErrorHold] SHUTTING DOWN BECAUSE: %s", msg);
			DebugError(str, true);
			delay(1000);

			// Set kill switch high
			digitalWrite(pin.KillSwitch, HIGH);

			// Quit if still powered by USB
			QuitSession();
		}

	}
}

#pragma endregion


#pragma region --------MINOR FUNCTIONS---------

// GET ID INDEX
int CharInd(char id, const char id_arr[], int arr_size)
{
	// Local vars
	char str[200] = { 0 };

	// Return -1 if not found
	int ind = -1;
	for (int i = 0; i < arr_size; i++) {

		if (id == id_arr[i]) {
			ind = i;
		}
	}

	// Print error if not found
	if (ind == -1) {
		sprintf(str, "**WARNING** [CharInd] ID \'%c\' Not Found", id);
		DebugError(str);
	}

	return ind;

}

// BLINK LEDS AT RESTART/UPLOAD
void StatusBlink(int n_blinks, int dt_led)
{
	int duty[2] = { 100, 0 };
	bool do_led_on = true;
	int cnt_blink = 0;

	// Flash sequentially
	while (cnt_blink <= n_blinks)
	{
		analogWrite(pin.Disp_LED, duty[(int)do_led_on]);
		delay(dt_led);
		analogWrite(pin.TrackLED, duty[(int)do_led_on]);
		delay(dt_led);
		analogWrite(pin.RewLED_C, duty[(int)do_led_on]);
		delay(100);
		do_led_on = !do_led_on;
		cnt_blink = !do_led_on ? cnt_blink + 1 : cnt_blink;
	}
	// Reset LEDs
	analogWrite(pin.Disp_LED, 0);
	analogWrite(pin.RewLED_C, rewLEDmin);
	analogWrite(pin.TrackLED, trackLEDduty);
}

#pragma endregion


#pragma region --------INTERUPTS---------

// TIMER INTERUPT/HANDLER
void Interupt_TimerHandler()
{
	// Bail if not active now
	if (!v_stepTimerActive) {
		return;
	}

	// Bail when extend target reached
	else if (v_stepDir == 'e' &&
		v_cnt_steps >= v_stepTarg) {

		// Make sure step off
		v_stepState = false;
		digitalWrite(pin.ED_STP, v_stepState);

		// Block handler
		v_stepTimerActive = false;

		// Set done flag
		v_isArmMoveDone = true;

		// Bail
		return;
	}

	// Release swtich when switch triggered on retract
	else if (v_stepDir == 'r' &&
		digitalRead(pin.FeedSwitch) == LOW) {

		// Make sure step off
		v_stepState = false;
		digitalWrite(pin.ED_STP, v_stepState);

		// Set direction to extend
		digitalWrite(pin.ED_DIR, LOW);

		// Move arm x steps
		for (int i = 0; i < 20; i++)
		{
			digitalWrite(pin.ED_STP, HIGH);
			delayMicroseconds(500);
			digitalWrite(pin.ED_STP, LOW);
			delayMicroseconds(500);
			v_cnt_steps--;
		}

		// Block handler
		v_stepTimerActive = false;

		// Set done flag
		v_isArmMoveDone = true;

		// Bail
		return;

	}

	// Itterate count
	v_cnt_steps += v_stepState ? 1 : 0;

	// Set state
	v_stepState = !v_stepState;

	// Step motor
	digitalWrite(pin.ED_STP, v_stepState);
}

// HALT RUN ON IR TRIGGER
void Interupt_IRprox_Halt() {

	// Exit if < 250 ms has not passed
	if (v_t_irProxDebounce > millis()) {
		return;
	}

	// Run stop in main loop
	v_doIRhardStop = true;

	// Update debounce
	v_t_irProxDebounce = millis() + 250;
}

// DETECT IR SYNC EVENT
void Interupt_IR_Detect()
{
	// Exit if < 25 ms has not passed
	if (millis() < v_t_irDetectDebounce) {
		return;
	}

	// Store time
	v_dt_ir = millis() - v_t_irSyncLast;
	v_t_irSyncLast += v_dt_ir;
	v_cnt_ir++;

	// Set flag
	if (t_sync != 0) {
		v_doLogIR = true;
	}

	// Update debounce
	v_t_irDetectDebounce = millis() + 50;
}

#pragma endregion

#pragma endregion


void setup() {

	// Local vars
	char str[200] = { 0 };

	// SET UP SERIAL STUFF

	// Serial monitor
	SerialUSB.begin(0);

	// XBee 1a (to/from CS)
	r2c.port.begin(57600);

	// XBee 1b (to/from CheetahDue)
	r2a.port.begin(57600);

	// Wait for SerialUSB if debugging
	uint32_t t_check = millis() + 100;
	if (db.Console) {
		while (!SerialUSB && millis() < t_check);
	}

	// SETUP OUTPUT PINS

	// Autodriver
	pinMode(pin.AD_CSP_R, OUTPUT);
	pinMode(pin.AD_CSP_F, OUTPUT);
	pinMode(pin.AD_RST, OUTPUT);
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
	// Voltage Regulators
	pinMode(pin.REG_24V_ENBLE, OUTPUT);
	pinMode(pin.REG_12V_ENBLE, OUTPUT);
	pinMode(pin.REG_5V_ENBLE, OUTPUT);
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
	// Power off
	pinMode(pin.KillSwitch, OUTPUT);
	delayMicroseconds(100);

	// Autodriver
	digitalWrite(pin.AD_CSP_R, LOW);
	digitalWrite(pin.AD_CSP_F, LOW);
	digitalWrite(pin.AD_RST, LOW);
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
	// Voltage Regulators
	digitalWrite(pin.REG_24V_ENBLE, LOW);
	digitalWrite(pin.REG_12V_ENBLE, LOW);
	digitalWrite(pin.REG_5V_ENBLE, LOW);
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
	// Power off
	digitalWrite(pin.KillSwitch, LOW);
	delayMicroseconds(100);

	// SET INPUT PINS

	// XBees
	pinMode(pin.X1a_CTS, INPUT);
	pinMode(pin.X1b_CTS, INPUT);
	// Battery monitor
	pinMode(pin.BatVcc, INPUT);
	pinMode(pin.BatIC, INPUT);
	// IR proximity sensors
	pinMode(pin.IRprox_Rt, INPUT);
	pinMode(pin.IRprox_Lft, INPUT);
	// IR detector
	pinMode(pin.IRdetect, INPUT);

	// Set button and switch pins enable internal pullup
	for (int i = 0; i <= 2; i++) {
		pinMode(pin.Btn[i], INPUT_PULLUP);
	}
	pinMode(pin.FeedSwitch, INPUT_PULLUP);
	delayMicroseconds(100);

	// ENABLE VOLTGAGE REGULATORS
	delay(250);
	digitalWrite(pin.REG_24V_ENBLE, HIGH);
	digitalWrite(pin.REG_12V_ENBLE, HIGH);
	digitalWrite(pin.REG_5V_ENBLE, HIGH);

	// SHOW RESTART BLINK
	delayMicroseconds(100);
	StatusBlink(1, 0);

	// INITIALIZE LCD
	LCD.InitLCD();

	// LOG/PRINT SETUP RUNNING

	// Print to LCD
	ChangeLCDlight(50);
	PrintLCD(true, "SETUP", "MAIN");

	// Log and print to console
	DebugFlow("[setup] RUNNING: Setup...");
	while (PrintDebug());

	// SETUP AUTODRIVER

	// Configure SPI
	PrintLCD(true, "SETUP", "AutoDriver");
	AD_R.SPIConfig();
	delayMicroseconds(100);
	AD_F.SPIConfig();
	delayMicroseconds(100);
	// Reset boards
	AD_Reset();

	// Make sure motor is stopped and in high impedance
	AD_R.hardHiZ();
	AD_F.hardHiZ();

	// SETUP BIG EASY DRIVER

	// Start BigEasyDriver in sleep
	PrintLCD(true, "SETUP", "Big Easy");
	digitalWrite(pin.ED_RST, HIGH);
	digitalWrite(pin.ED_SLP, LOW);

	// INITIALIZE PIXY
	PrintLCD(true, "SETUP", "Pixy");
	Pixy.init();
	Wire.begin();

	// DUMP BUFFER
	PrintLCD(true, "SETUP", "Dump Serial");
	while (c2r.port.available() > 0) {
		c2r.port.read();
	}
	while (a2r.port.available() > 0) {
		a2r.port.read();
	}

	// RESET VOLITILES AND RELAYS
	v_t_irProxDebounce = millis(); // (ms)
	v_t_irDetectDebounce = millis(); // (ms)
	v_t_irSyncLast = 0; // (ms)
	t_sync = 0; // (ms)
	v_dt_ir = 0;
	v_cnt_ir = 0;
	v_doIRhardStop = false;
	v_doLogIR = false;
	digitalWrite(pin.Rel_Rew, LOW);
	digitalWrite(pin.Rel_EtOH, LOW);

	// CHECK THAT POWER ON
	uint32_t t_check_vcc = millis() + 1000;

	// Loop till vcc or timeout
	PrintLCD(true, "SETUP", "Battery Check");
	while (CheckBattery(true) == 0 && millis() < t_check_vcc);

	// Exit with error if power not on
	if (CheckBattery(true) == 0) {
		// Hold for error
		DebugError("!!ERROR!! [setup] ABORTED: Power Off", true);
		while (PrintDebug());
		digitalWrite(pin.Rel_EtOH, LOW);
		RunErrorHold("POWER OFF");
	}

	// SETUP OPENLOG
	PrintLCD(true, "SETUP", "OpenLog");
	DebugFlow("[setup] RUNNING: OpenLog Setup...");
	while (PrintDebug());

	// Setup OpenLog
	if (Log.Setup())
	{
		// Log/print setup finished
		DebugFlow("[setup] SUCCEEDED: OpenLog Setup");
		while (PrintDebug());
	}
	else {
		// Hold for error
		DebugError("!!ERROR!! [setup] ABORTED: OpenLog Setup", true);
		while (PrintDebug());
		RunErrorHold("OPENLOG SETUP");
	}

	// Create new log file
	PrintLCD(true, "SETUP", "Log File");
	DebugFlow("[setup] RUNNING: Make New Log...");
	if (Log.OpenNewLog() == 0) {
		// Hold for error
		DebugError("!!ERROR!! [setup] ABORTED: Setup", true);
		while (PrintDebug());
		RunErrorHold("OPEN LOG FILE");
	}
	else {
		sprintf(str, "[setup] SUCCEEDED: Make New Log: file_name=%s", Log.logFile);
		DebugFlow(str);
		while (PrintDebug());
	}

	// DEFINE EXTERNAL INTERUPTS
	PrintLCD(true, "SETUP", "Interrupts");
	uint32_t t_check_ir = millis() + 1000;
	bool is_ir_low = false;

	// Check if IR detector already high
	while (!is_ir_low && millis() < t_check_ir) {
		is_ir_low = digitalRead(pin.IRdetect) == LOW;
	}

	// Do not enable if ir detector pin already high
	if (is_ir_low) {
		// IR detector
		attachInterrupt(digitalPinToInterrupt(pin.IRdetect), Interupt_IR_Detect, HIGH);
	}
	else {
		// Skip ir sync setup
		t_sync = 1;
		DebugError("!!ERROR!! [setup] IR SENSOR DISABLED", true);
		while (PrintDebug());
	}

	// IR prox right
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Rt), Interupt_IRprox_Halt, FALLING);

	// IR prox left
	attachInterrupt(digitalPinToInterrupt(pin.IRprox_Lft), Interupt_IRprox_Halt, FALLING);

	// Start Feed Arm timer
	Timer1.attachInterrupt(Interupt_TimerHandler).start(1000);

	// CLEAR LCD
	ChangeLCDlight(0);
	ClearLCD();

	// SET DEFAULTS
	fc.isManualSes = true;
	fc.doAllowRevMove = true;

	// PRINT SETUP FINISHED
	DebugFlow("[setup] FINISHED: Setup");
	while (PrintDebug());

	// PRINT AVAILABLE MEMORY
	sprintf(str, "[setup] AVAILABLE MEMORY: %0.2fKB",
		(float)freeMemory() / 1000);
	DebugFlow(str);
	while (PrintDebug());

	// TEMP
	//Log.TestLoad(0, "LOG00027.CSV");
	//Log.TestLoad(2500);

	// RESET FEEDER ARM
	Reward.RetractFeedArm();

}


void loop() {

#pragma region //--- ONGOING OPPERATIONS ---

	// Local vars
	static char horeStr[200] = { 0 };

	// CHECK LOOP TIME AND MEMORY
	CheckLoop();

	// PARSE CS SERIAL INPUT
	GetSerial(&c2r);

	// PARSE CHEETAHDUE SERIAL INPUT
	GetSerial(&a2r);

	// SEND CHEETAHDUE DATA
	SendPacket(&r2a);

	// SEND CS DATA
	SendPacket(&r2c);

	// RESEND CHEETAHDUE DATA
	CheckResend(&r2a);

	// RESEND CS DATA
	CheckResend(&r2c);

	// PRINT QUEUED DB
	PrintDebug();

	// STORE QUEUED LOGS
	Log.WriteLog();

	// SEND SERIAL DATA
	if (fc.doLogSend) {
		// Send log
		Log.StreamLogs();

		// Print
		if (!fc.doLogSend) {
			DebugFlow("[loop] FINISHED SENDING LOGS");
		}
	}

	// GET BUTTON INPUT
	if (GetButtonInput())
	{

		// Button triggered reward
		if (fc.doBtnRew) {
			// Bail if already rewarding
			if (Reward.isRewarding) {
				DebugError("**WARNING** [loop] ABORTED: Button Reward Triggered When Already Running Reward");
			}
			else {
				// Set mode
				Reward.SetRewMode("Button");
				Reward.StartRew();
			}
			fc.doBtnRew = false;
		}

		// Open/close Rew sol
		if (fc.doRewSolStateChange) {
			OpenCloseRewSolenoid();
			fc.doRewSolStateChange = false;
		}

		// Open/close EtOH sol
		if (fc.doEtOHSolStateChange) {
			OpenCloseEtOHSolenoid();
			fc.doEtOHSolStateChange = false;
		}

		// Turn LCD on/off
		if (fc.doChangeLCDstate) {
			ChangeLCDlight();
			fc.doChangeLCDstate = false;
		}

		// Move robot foward/backward
		if (fc.doMoveRobFwd) {
			ManualRun('f');
			fc.doMoveRobFwd = false;
		}
		else if (fc.doMoveRobRev) {
			ManualRun('r');
			fc.doMoveRobRev = false;
		}

	}

	// CHECK FOR IR TRIGGERED HALT
	if (v_doIRhardStop)
	{
		IRprox_Halt();
		v_doIRhardStop = false;
	}

	// LOG NEW IR EVENT
	if (v_doLogIR)
	{
		// Log event if streaming started
		sprintf(horeStr, "[loop] IR Sync Event: cnt=%d dt=%dms", v_cnt_ir, v_dt_ir);
		DebugFlow(horeStr, v_t_irSyncLast);

		// Reset flag
		v_doLogIR = false;
	}

	// END ONGOING REWARD
	if (Reward.EndRew()) {

		// Tell CS what zone was rewarded and get confirmation
		if (Reward.mode != "Now" &&
			Reward.mode != "Button") {

			QueuePacket(&r2c, 'Z', Reward.zoneInd + 1, 0, 0, 0, true);
		}
	}

	// RETRACT FEEDER ARM
	Reward.CheckFeedArm();

	// CHECK IF STILL BLOCKING
	CheckBlockTimElapsed();

	// GET AD STATUS
	AD_CheckOC();

	// RUN ETOH SOLONOID
	CheckEtOH();

	// GET VCC
	CheckBattery();

#pragma endregion

#pragma region //--- FIRST PASS SETUP ---
	if (!fc.isSesStarted)
	{

		// PULSE TRACKER LED
		static bool is_on = false;
		static uint32_t t_pulse_last = millis() - 1001;
		if (
			(is_on && millis() >= t_pulse_last + 10) ||
			(!is_on && millis() >= t_pulse_last + 1000)
			) {

			analogWrite(pin.TrackLED, is_on ? 0 : 100);
			is_on = !is_on;
			t_pulse_last = is_on ? millis() : t_pulse_last;
		}

		// CHECK FOR SYNC IR HANDSHAKE PULSE
		if (t_sync == 0)
		{

			// Check for setup ir pulse
			if (abs(75 - v_dt_ir) < 10) {

				// Set sync time
				t_sync = v_t_irSyncLast;

				// Log/print sync time
				sprintf(horeStr, "SET SYNC TIME: %dms", t_sync);
				DebugFlow(horeStr);

				// Send handshake 
				QueuePacket(&r2c, 'h', 1, 0, 0, 0, true);
				SendPacket(&r2c);
			}
			// Restart loop
			else {
				return;
			}
		}

		// DO SETUP BLINK
		StatusBlink(5);

		// RESET LCD
		if (!fc.isManualSes &&
			fc.isLitLCD) {

			fc.doChangeLCDstate = true;
		}
		ClearLCD();

		// RESET SOLONOIDS
		if (digitalRead(pin.Rel_EtOH) == HIGH) {
			OpenCloseEtOHSolenoid();
		}
		if (digitalRead(pin.Rel_Rew) == HIGH) {
			OpenCloseRewSolenoid();
		}

		// PRINT AD BOARD STATUS
		sprintf(horeStr, "[loop] BOARD R STATUS: %04X", AD_R.getStatus());
		DebugFlow(horeStr);
		sprintf(horeStr, "[loop] BOARD F STATUS: %04X", AD_F.getStatus());
		DebugFlow(horeStr);

		// SET FLAG
		fc.isSesStarted = true;
		DebugFlow("[loop] READY TO ROCK!");

	}

#pragma endregion

#pragma region //--- (T) SYSTEM TESTS ---

	if (c2r.idNow == 'T' && c2r.isNew)
	{
		// Store message data
		cmd.testCond = (byte)c2r.dat[0];
		cmd.testDat = (byte)c2r.dat[1];

		// Set run pid calibration flag
		if (cmd.testCond == 1)
		{
			db.do_pidCalibration = true;

			// Print settings
			sprintf(horeStr, "[loop] RUN PID CALIBRATION = kC=%0.2f", kC);
			DebugFlow(horeStr);
		}

		// Update Halt Error test run speed
		else if (cmd.testCond == 2)
		{
			double new_speed = double(cmd.testDat);
			double speed_steps = new_speed*cm2stp;


			if (new_speed > 0)
			{
				// Run motor
				AD_R.run(FWD, speed_steps);
				AD_F.run(FWD, speed_steps*scaleFrontAD);
			}
			else
			{
				// Halt robot
				AD_R.hardStop();
				AD_F.hardStop();
			}

			// Print speed
			sprintf(horeStr, "[loop] HALT ERROR SPEED = %0.0f cm/sec", new_speed);
			DebugFlow(horeStr);
		}

		// Symulated rat test
		else if (cmd.testCond == 3)
		{
			db.do_simRatTest = true;
		}
	}

	// Run position debugging
	if (db.do_posDebug)
	{
		static double pos_last[3] = { 0 };
		bool is_new = false;

		// Check if position values changed
		for (int i = 0; i < 3; i++) {
			if (Pos[i].posNow != pos_last[i]) {
				is_new = true;
			}
			pos_last[i] = Pos[i].posNow;
		}

		// Plot new data
		if (is_new)
		{
			// Get setpoint pos
			double setpoint_vt = Pos[1].posNow + Pid.setPoint;
			double setpoint_ekf = kal.RobPos + Pid.setPoint;

			// Plot pos
			if (db.do_posPlot) {
				/*
				{@Plot.Pos.Pos[0].Blue Pos[0].posNow}{@Plot.Pos.Pos[2].Green Pos[2].posNow}{@Plot.Pos.Pos[1].Orange Pos[1].posNow}{@Plot.Pos.setpoint_vt.Orange setpoint_vt}{@Plot.Pos.ratEKF.Black kal.RatPos}{@Plot.Pos.robEKF.Red kal.RobPos}{@Plot.Pos.setpoint_ekf.Red kal.setpoint_ekf}
				*/
				millis();
			}

			// Turn on rew led when near setpoint
			if (Pid.error > -0.5 && Pid.error < 0.5 &&
				Pos[0].is_streamStarted &&
				Pos[1].is_streamStarted) {

				analogWrite(pin.RewLED_C, 10);
			}
			else {
				analogWrite(pin.RewLED_C, rewLEDmin);
			}

			// Print pos data
			char str1[50];
			char str2[50];
			sprintf(str1, "Rat Dst = %0.2fcm", kal.RatPos - kal.RobPos);
			sprintf(str2, "Pid Err = %0.2fcm", Pid.error);
			PrintLCD(false, str1, str2, 't');

		}
	}

	// Run Pid calibration
	if (db.do_pidCalibration)
	{
		double new_speed = Pid.RunPidCalibration();

		// Run motors
		if (Pid.cal_isPidUpdated)
		{
			if (new_speed >= 0) {
				RunMotor('f', new_speed, "Override");
			}
			// Print values
			/*
			millis()%50 == 0
			{Pid.cal_isCalFinished}{"ERROR"}{Pid.cal_errNow}{Pid.cal_errArr[0]}{Pid.cal_errArr[1]}{Pid.cal_errArr[2]}{Pid.cal_errArr[3]}{"PERIOD"}{Pid.cal_PcNow}{Pid.cal_cntPcArr[0]}{Pid.cal_PcArr[0]}{Pid.cal_cntPcArr[1]}{Pid.cal_PcArr[1]}{Pid.cal_cntPcArr[2]}{Pid.cal_PcArr[2]}{Pid.cal_cntPcArr[3]}{Pid.cal_PcArr[3]}{Pid.cal_PcAll}
			*/
			millis();
			// Plot error
			/*
			{@Plot.Vel.Error.Red Pid.cal_errNow} {@Plot.Vel.Setpoint.Black 0}
			*/
			millis();
			// Reset flag
			Pid.cal_isPidUpdated = false;
		}
	}

#pragma endregion

#pragma region //--- (S) DO SETUP ---
	if (c2r.idNow == 'S' && c2r.isNew)
	{
		// Store message data
		cmd.sesCond = (byte)c2r.dat[0];
		cmd.soundCond = (byte)c2r.dat[1];
		millis();
		// Set condition
		if (cmd.sesCond == 1) {
			fc.isManualSes = false;
			DebugFlow("[loop] DO TRACKING");
		}
		else {
			fc.isManualSes = true;
			DebugFlow("[loop] DONT DO TRACKING");
		}
		// Set reward tone
		if (cmd.soundCond == 0) {
			// No sound
			QueuePacket(&r2a, 's', 0);
			DebugFlow("[loop] NO SOUND");
		}
		else if (cmd.soundCond == 1) {
			// Use white noise only
			QueuePacket(&r2a, 's', 0);
			DebugFlow("[loop] DONT DO TONE");
		}
		else {
			// Use white and reward noise
			QueuePacket(&r2a, 's', 2);
			DebugFlow("[loop] DO TONE");
		}

	}
#pragma endregion

#pragma region //--- (Q) DO QUIT ---
	if (c2r.idNow == 'Q' && c2r.isNew) {

		// Set flags and delay time
		fc.doQuit = true;
		t_quit = millis() + 1000;
		DebugFlow("[loop] DO QUIT");

		// Tell CheetahDue to quit 
		QueuePacket(&r2a, 'q', 0, 0, 0, 0, true);

		// Block all motor control
		SetMotorControl("Halt", "Quit");

	}

	// Wait for queue buffer to empty and quit delay to pass 
	if (
		fc.doQuit && millis() > t_quit &&
		!CheckResend(&r2a) &&
		!CheckResend(&r2c) &&
		!SendPacket(&r2a) &&
		!SendPacket(&r2c)) {

		// Quit session
		DebugFlow("[loop] QUITING...");
		QuitSession();
	}
#pragma endregion

#pragma region //--- (M) DO MOVE ---
	if (c2r.idNow == 'M' && c2r.isNew) {

		// Store move pos
		cmd.moveToTarg = c2r.dat[0];

		// Align target pos to feeder
		cmd.moveToTarg = cmd.moveToTarg - feedDist;
		cmd.moveToTarg = cmd.moveToTarg < 0 ? cmd.moveToTarg + (140 * PI) : cmd.moveToTarg;

		// Set flags
		fc.doMove = true;

		sprintf(horeStr, "[loop] DO MOVE: targ=%0.2fcm", cmd.moveToTarg);
		DebugFlow(horeStr);
	}

	// Perform movement
	if (fc.doMove) {

		// Compute move target
		if (!Move.isTargSet) {

			// If succesfull
			if (Move.CompTarg(kal.RobPos, cmd.moveToTarg)) {

				// Start running
				if (SetMotorControl("MoveTo", "MoveTo")) {

					if (RunMotor(Move.moveDir, moveToSpeed, "MoveTo")
						) {

						// Print message
						sprintf(horeStr, "[loop] RUNNING: MoveTo: targ_dist=%0.2fcm move_dir=\'%c\'...",
							Move.targDist, Move.moveDir);
						DebugFlow(horeStr);

						// Set flag
						Move.isMoveStarted = true;
					}

					// Failed to run motor
					else {
						// Reset control
						SetMotorControl("None", "MoveTo");

						// Set flags
						Move.doAbortMove = true;
					}
				}

				// Failed to take motor control
				else {
					// Set flags
					Move.doAbortMove = true;
				}

			}
		}

		// Check if robot is ready to be stopped
		else if (!Move.isTargReached && !Move.doAbortMove) {

			// Do deceleration
			double new_speed = Move.DecelToTarg(kal.RobPos, kal.RobVel, 40, 10);

			// Change speed if > 0
			if (new_speed > 0 && new_speed != runSpeedNow) {

				RunMotor(Move.moveDir, new_speed, "MoveTo");
			}

		}

		// Check if target reached or move aborted
		else {

			// Hard stop
			HardStop("MoveTo");

			// Set motor cotrol to None
			SetMotorControl("None", "MoveTo");

			// Log/print final move status
			if (Move.isTargReached) {

				// Tell CS movement is done
				QueuePacket(&r2c, 'D', 0, 0, 0, c2r.pack[CharInd('M', c2r.id, c2r.lng)], true);

				// Print success message
				sprintf(horeStr, "[loop] SUCCEEDED: MoveTo: targ_dist=%0.2fcm dist_error=%0.2fcm move_dir=\'%c\'",
					Move.targDist, Move.GetError(kal.RobPos), Move.moveDir);
				DebugFlow(horeStr);
			}

			// Log/print error
			else if (Move.doAbortMove) {
				// Print failure message
				sprintf(horeStr, "!!ERROR!! [loop] ABORTED: MoveTo: targ_set=%s ekf_ready=%s move_started=%s targ_dist=%0.2fcm dist_error=%0.2fcm move_dir=\'%c\'",
					Move.isTargSet ? "true" : "false", fc.isEKFReady ? "true" : "false", Move.isMoveStarted ? "true" : "false", Move.targDist, Move.GetError(kal.RobPos), Move.moveDir);
				DebugError(horeStr, true);
			}

			// Reset flags
			fc.doMove = false;
			Move.Reset();
		}
	}
#pragma endregion

#pragma region //--- (R) RUN REWARD ---
	if (c2r.idNow == 'R' && c2r.isNew) {

		// Store message data
		cmd.rewPos = c2r.dat[0];
		cmd.rewCond = (byte)c2r.dat[1];
		cmd.rewZoneInd = (byte)c2r.dat[2] - 1;
		cmd.rewDelay = (byte)c2r.dat[2];


		// Bail if already rewarding
		if (Reward.isRewarding) {
			DebugError("**WARNING** [loop] ABORTED: \'R\' Reward Triggered When Already Running Reward");
			return;
		}

		// Imediate reward
		else if (cmd.rewCond == 1) {

			// Log/print
			DebugFlow("[loop] DO NOW REWARD");

			// Set mode
			Reward.SetRewMode("Now", cmd.rewZoneInd);

			// Start reward
			Reward.StartRew();

		}

		// Cued reward
		else if (cmd.rewCond == 2) {

			// Log/print
			DebugFlow("[loop] DO CUED REWARD");

			// Set mode
			Reward.SetRewMode("Cue", cmd.rewZoneInd);

			// Set flag
			fc.doRew = true;
		}

		// Free reward
		else if (cmd.rewCond == 3) {

			// Log/print
			DebugFlow("[loop] DO FREE REWARD");

			// Set mode
			Reward.SetRewMode("Free", cmd.rewDelay);

			// Set flag
			fc.doRew = true;
		}

	}

	// CHECK REWARD BOUNDS
	if (fc.doRew) {

		// If not rewarding 
		if (!Reward.isRewarding) {

			// Zone bounds not set yet
			if (!Reward.isBoundsSet) {

				// Compute bounds
				Reward.CompZoneBounds(kal.RatPos, cmd.rewPos);

				// Print message
				sprintf(horeStr, "[loop] SET REWARD ZONE: center=%0.2fcm from=%0.2fcm to=%0.2fcm",
					Reward.rewCenterRel, Reward.boundMin, Reward.boundMax);
				DebugFlow(horeStr);
			}

			// Zone not triggered yet
			else if (!Reward.isZoneTriggered) {

				// Check each zone
				if (Reward.CheckZoneBounds(kal.RatPos)) {

					// Start reward
					Reward.StartRew();

					// Reset flag
					fc.doRew = false;

					// Print message
					sprintf(horeStr, "[loop] REWARDED ZONE: occ=%dms zone=%d from=%0.2fcm to=%0.2fcm",
						Reward.occRewarded, Reward.zoneRewarded, Reward.boundsRewarded[0], Reward.boundsRewarded[1]);
					DebugFlow(horeStr);
				}
			}

			// Check if rat passed all bounds
			if (Reward.isAllZonePassed &&
				!Reward.isZoneTriggered) {

				// Print reward missed
				sprintf(horeStr, "[loop] REWARD MISSED: rat=%0.2fcm bound_max=%0.2fcm",
					kal.RatPos, Reward.boundMax);
				DebugFlow(horeStr);

				// Reset flags
				Reward.Reset();
				fc.doRew = false;
			}
		}
	}

#pragma endregion

#pragma region //--- (H) HALT ROBOT STATUS ---
	if (c2r.idNow == 'H' && c2r.isNew) {

		// Store message data
		fc.doHalt = c2r.dat[0] != 0 ? true : false;

		if (fc.doHalt) {

			// Log/print
			DebugFlow("[loop] HALT STARTED");

			// Stop pid and set to manual
			HardStop("Halt");

			// Block motor control
			SetMotorControl("Halt", "Halt");

		}
		else {

			// Log/print
			DebugFlow("[loop] HALT FINISHED");

			// Open motor control
			SetMotorControl("Open", "Halt");

		}
	}
#pragma endregion

#pragma region //--- (B) BULLDOZE RAT STATUS ---
	if (c2r.idNow == 'B' && c2r.isNew) {

		// Store message data
		cmd.bullDel = (byte)c2r.dat[0];
		cmd.bullSpeed = (byte)c2r.dat[1];

		// Local vars
		bool is_mode_changed = false;

		// Reinitialize bulldoze
		Bull.Reinitialize(cmd.bullDel, cmd.bullSpeed, "loop \'B\'");

		// Check if mode should be changedchanged
		if (cmd.bullSpeed > 0) {

			// Mode changed
			if (!fc.doBulldoze) {

				is_mode_changed = true;
				fc.doBulldoze = true;
				DebugFlow("[loop] SET BULLDOZE ON");
			}
			// Only settings changed
			else {
				is_mode_changed = false;
			}
		}
		else {
			// Mode changed
			if (fc.doBulldoze)
			{

				is_mode_changed = true;
				fc.doBulldoze = false;
				DebugFlow("[loop] SET BULLDOZE OFF");
			}
			// Only settings changed
			else {
				is_mode_changed = false;
			}
		}

		// Don't exicute until rat is in and mode is changed
		if (fc.isTrackingEnabled &&
			is_mode_changed) {

			if (fc.doBulldoze) {

				// Turn bulldoze on
				Bull.TurnOn("loop \'B\'");
				DebugFlow("[loop] BULLDOZE ON");
			}
			else {

				// Turn bulldoze off
				Bull.TurnOff("loop \'B\'");
				DebugFlow("[loop] BULLDOZE OFF");
			}
		}

	}
#pragma endregion

#pragma region //--- (I) START/STOP PID ---
	if (c2r.idNow == 'I' && c2r.isNew)
	{
		// Store message data
		fc.isRatIn = c2r.dat[0] != 0 ? true : false;

		if (fc.isRatIn) {

			// Reset rat pos data
			Pos[0].Reset();
			Pos[2].Reset();

			// Pid started by InitializeTracking()
			DebugFlow("[loop] RAT IN");
		}
		else {

			// Turn off bulldoze
			Bull.TurnOff("loop \'I\'");
			fc.doBulldoze = false;

			// Turn off pid
			Pid.Stop("loop \'I\'");

			// Set to stop tracking
			fc.isTrackingEnabled = false;

			DebugFlow("[loop] RAT OUT");
		}
	}
#pragma endregion

#pragma region //--- (V) GET STREAM STATUS ---
	if (c2r.idNow == 'V' && c2r.isNew)
	{
		fc.doStreamCheck = true;
	}

	// Check for streaming
	if (fc.doStreamCheck && Pos[1].is_streamStarted)
	{
		// Send streaming confirmation
		QueuePacket(&r2c, 'D', 0, 0, 0, c2r.pack[CharInd('V', c2r.id, c2r.lng)], true);
		fc.doStreamCheck = false;
		DebugFlow("[loop] STREAMING CONFIRMED");

		// Set flag to begin sending vcc
		fc.doSendVCC = true;
	}
#pragma endregion

#pragma region //--- (P) VT DATA RECIEVED ---
	if (c2r.idNow == 'P' && c2r.isNew)
	{
		// Store message data
		cmd.vtEnt = (byte)c2r.dat[0];
		cmd.vtCM[cmd.vtEnt] = c2r.dat[1];
		U.f = c2r.dat[2];
		cmd.vtTS[cmd.vtEnt] = U.i32;

		// Handle rob vt data
		if (cmd.vtEnt == 1) {

			// Update VT
			Pos[cmd.vtEnt].UpdatePos(cmd.vtCM[cmd.vtEnt], cmd.vtTS[cmd.vtEnt]);

			// Set rat vt and pixy to setpoint if rat not in 
			if (!fc.isRatIn && !db.do_posDebug) {
				Pos[0].SwapPos(Pos[1].posAbs + Pid.setPoint, Pos[1].t_msNow);
				Pos[2].SwapPos(Pos[1].posAbs + Pid.setPoint, Pos[1].t_msNow);
			}

			// Log/print first sample
			if (!Pos[1].is_streamStarted) {
				DebugFlow("[NetComCallbackVT] FIRST ROBOT VT RECORD");
				Pos[1].is_streamStarted = true;
			}
		}

		// Handle rat vt data
		else if (cmd.vtEnt == 0) {

			// Update only after rat in
			if (fc.isRatIn || db.do_posDebug) {

				// Update rat VT
				Pos[cmd.vtEnt].UpdatePos(cmd.vtCM[cmd.vtEnt], cmd.vtTS[cmd.vtEnt]);

				// Use rat vt for pixy if running simulated rat test
				if (cmd.vtEnt == 0 && db.do_simRatTest) {
					Pos[2].SwapPos(Pos[0].posAbs, Pos[0].t_msNow);
				}

				// Log/print first sample
				if (!Pos[0].is_streamStarted) {
					DebugFlow("[NetComCallbackVT] FIRST RAT VT RECORD");
					Pos[0].is_streamStarted = true;
				}
			}

		}

	}
#pragma endregion

#pragma region //--- (L) SEND LOG ---
	if (c2r.idNow == 'L' && c2r.isNew) {

		// Flag to begin sending
		fc.doLogSend = c2r.dat[0] == 1 ? true : false;

		// Check for 2 way confirmation before sending log
		if (!fc.doLogSend) {

			// Send number of log bytes being sent
			QueuePacket(&r2c, 'U', Log.cnt_logBytesStored, 0, 0, 0, true);

			// Block sending vcc updates
			fc.doBlockVccSend = true;

			// Log/print
			sprintf(horeStr, "[loop] SENDING LOG: logs_stored=~%d bytes_stored=~%d",
				Log.cnt_logsStored, Log.cnt_logBytesStored);
			DebugFlow(horeStr);
		}

		// Begin sending log
		else {

			// Set send time
			Log.t_beginSend = millis() + Log.dt_beginSend;

			// Log/print
			DebugFlow("[loop] DO SEND LOG");

			// Store remaining logs
			while (Log.WriteLog()) {
				delay(100);
			}
		}

	}

#pragma endregion

#pragma region //--- UPDATE TRACKING ---

	// UPDATE PIXY
	CheckPixy();

	// CHECK IF RAT POS FRAMES DROPPING
	CheckSampDT();

	// INITIALIZE RAT AHEAD
	InitializeTracking();

	// UPDATE EKF
	UpdateEKF();

	// UPDATE PID AND SPEED
	double new_speed = Pid.UpdatePID();

	if (new_speed == 0) {
		HardStop("Pid");
	}
	else if (new_speed > 0) {
		RunMotor('f', new_speed, "Pid");
	}

	// UPDATE BULLDOZER
	Bull.UpdateBull();

	// LOG TRACKING DATA
	LogTrackingData();

#pragma endregion

}
