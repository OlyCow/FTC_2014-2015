#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     sensor_IR,      sensorI2CCustom)
#pragma config(Sensor, S3,     sensor_color,   sensorCOLORFULL)
#pragma config(Sensor, S4,     sensor_gyro,    sensorAnalogInactive)
#pragma config(Motor,  motorA,          motor_assist,  tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_clamp_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          motor_clamp_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_L,       tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_pickup,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_lift_A,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift_B,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motor_R_A,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motor_R_B,     tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Servo,  srvo_S1_C1_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S1_C1_2,    servo_auton,          tServoStandard)
#pragma config(Servo,  srvo_S1_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)

#include "includes.h"
#include "proto.h"

// This autonomous program

task Gyro();
task PID();
task Display();

void DumpBall();
void DumpAuton();

bool DriveForward(int encoder_count);
bool DriveBackward(int encoder_count);
bool TurnLeft(int degrees);
bool TurnRight(int degrees);

bool Drive(int encoder_count);
bool Turn(int degrees);

float heading = 0.0;
int lift_target = 0;
int power_lift = 0;
int power_lift_temp = 0;
bool is_lift_manual = false;
bool isDown = true;
int light_intensity = 0;
int IR_A = 0;
int IR_B = 0;
int IR_C = 0;
int IR_D = 0;
int IR_E = 0;

float term_P_lift = 0.0;
float term_I_lift = 0.0;
float term_D_lift = 0.0;

int target_dist_disp = 0;
int pos_dist_disp_L = 0;
int pos_dist_disp_R = 0;
float error_dist_disp = 0.0;
float error_sum_dist_disp = 0.0;
float power_dist_disp = 0.0;
float term_P_dist = 0.0;
float term_I_dist = 0.0;

int target_angle_disp = 0;
int curr_angle_disp = 0;
float error_angle_disp = 0.0;
float error_sum_angle_disp = 0.0;
float power_angle_disp = 0.0;
float term_P_angle = 0.0;
float term_I_angle = 0.0;

typedef enum CenterGoalPos {
	CENTER_POS_UNKNOWN	= -1,
	CENTER_POS_1		= 0,        //Goal is facing the parking zone
	CENTER_POS_2		= 1,        //Goal faces the corner where your ramp is
	CENTER_POS_3		= 2,        //Goal is perpendicular to pos_1 (facing the side of the arena your ramp is on)
	CENTER_POS_NUM
} CenterGoalPos;

CenterGoalPos centerGoalPos = CENTER_POS_UNKNOWN;

task main()
{
	bool isRed = false;
	bool isBlue = false;
	bool isUnknown = true;
	int color_threshold = 0;

	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);
	Motor_ResetEncoder(encoder_lift);

	HTIRS2setDSPMode(sensor_IR, DSP_1200);

	Task_Spawn(Display);
	Task_Spawn(Gyro);
	Task_Spawn(PID);

	Servo_SetPosition(servo_dump, pos_dump_closed);

	Joystick_WaitForStart();
	Time_Wait(500);

	// Determine color of ramp:
	// Currently unbelievably complicated. Could probably be alleviated by using
	// an enum. Whatever. Still need to figure out what happens if data conflicts.
	SensorType[sensor_color] = sensorCOLORRED;
	if (SensorValue[sensor_color]>=detected_red) {
		isRed = true;
	}
	SensorType[sensor_color] = sensorCOLORBLUE;
	if (SensorValue[sensor_color]>=detected_blue) {
		isBlue = true;
	}
	if (isRed==isBlue) {
		isUnknown = true;
	}
	if (!isUnknown) {
		if (isRed) {
			color_threshold = detected_red;
		} else {
			color_threshold = detected_blue;
		}
	}

	// Drive off ramp (backwards):
	// This portion is mostly wizardry (heuristic). First we drive off the ramp at
	// full speed to minimize the amount of time spent in an unbalanced position,
	// then slow down for the ramped portion of the ramp, and stop moving forward
	// when the color sensor determines that we are just off of the ramp.
	int ramp_power = -100;
	Motor_SetPower(ramp_power + 45, motor_L);
	Motor_SetPower(ramp_power, motor_R_B);
	Motor_SetPower(ramp_power, motor_R_A);
	Time_Wait(2000);
	//ramp_power = -30;
	//Motor_SetPower(ramp_power, motor_L);
	//Motor_SetPower(ramp_power, motor_R_B);
	//Motor_SetPower(ramp_power, motor_R_A);
	//Time_Wait(1800);
	// TODO: WHAT HAPPENS HERE??? DETECT COLOR CHANGE OR SOMETHING?
	Motor_SetPower(0, motor_L);
	Motor_SetPower(0, motor_R_B);
	Motor_SetPower(0, motor_R_A);
	Time_Wait(500);

	// Correct the heading of the robot:
	// After coming off the ramp the robot will (probably) be at an angle. As
	// as the robot isn't stranded on the ramp, the displacement should be small
	// enough such that we can correct it and still be reasonably close to being
	// on the correct path.
	// We correct the heading by turning the robot until the gyro reads that our
	// angle is 0 again. This may not work 100% because the gyro might mess up
	// the angle readings when pitch and roll aren't 0.
	int correction_turn = heading;
	TurnLeft(correction_turn);

	// Back up to bottom of ramp:
	// Apply just enough power to move backwards but not enough to drive the
	// robot back up the ramp. This makes doubly sure that the robot is in a
	// determined location (since we're aligning the robot up against the ramp).
	Motor_SetPower(10, motor_L);
	Motor_SetPower(10, motor_R_B);
	Motor_SetPower(10, motor_R_A);
	Time_Wait(1100);
	Motor_SetPower(0, motor_L);
	Motor_SetPower(0, motor_R_B);
	Motor_SetPower(0, motor_R_A);

	// Sense IR, determine position of center goal:
	// Due to the way the IR beacons are placed, if the signals from the two (R/B)
	// beacons add linearly, we can determine the position of the center goal
	// solely by aggregating the readings from each region.
	Time_Wait(500);
	HTIRS2readAllACStrength(sensor_IR, IR_A, IR_B, IR_C, IR_D, IR_E);
	if (((IR_B+IR_C)>35) && (IR_B>15) && (IR_C>15)) {
		centerGoalPos = CENTER_POS_3;
	} else if (IR_B>25) {
		centerGoalPos = CENTER_POS_2;
	} else if ((IR_A + IR_B + IR_C)<10) {
		centerGoalPos = CENTER_POS_1;
	} // else the center position stays unknown.

	lift_target = LIFT_HIGH;

	// Move into position to pick up rolling goal:
	// The first drive command moves us about flush with the tile just before the
	// medium rolling goal (going off memory here; I may be completely off).
	// The second drive command moves us back far enough so that we line up with
	// the tall rolling goal when we go to retrieve it. We then drive backwards.
	// We make sure to leave some space for PID to overshoot and still not bump
	// the goal (such that moves it farther from us).
	DriveBackward(800);
	TurnLeft(30);
	DriveBackward(5700);
	TurnRight(75);
	DriveBackward(2000);

	// Pick up goal:
	// Start both the clamp motors, drive backward a bit, wait a bit to make sure
	// we're clamped on solidly, then turn off the clamp motors.
	Motor_SetPower(100, motor_clamp_L);
	Motor_SetPower(100, motor_clamp_R);
	DriveBackward(1400);
	Time_Wait(800);

	// Tow the goal back to the parking zone:
	// The first drive statement determines how close the robot drives to the ramp and
	// the center goal. In one of the positions the robot drives very close to the
	// center goal (when the kickstands face the parking zones) because there isn't a
	// lot of clearance. However, it is more dangerous to drive close to the ramp,
	// since there is the risk of turning and having the goal knocked out of the clamp.
	// Then we drive back far enough to line ourselves up with the parking zone,
	// then turn and enter the zone with the rolling goal in the back. (It only needs
	// to be partially inside the zone.) Moving too far does have the risk of burning
	// out motors, since the watchdog kicks in after a sizeable delay.
	DriveForward(3300);

	// Raise lift and dump balls:
	// We want to be safe and keep our center of gravity low for as long as possible,
	// so only raise lift now. We're giving it a delay to get to the top, then giving
	// the dumping servo plenty of time to dump the balls *and* close so that it
	// doesn't get caught on the ball tube as the lift lowers.
	DumpBall();

	// Lower lift.
	lift_target = LIFT_CENTER;

	//TurnLeft(45);
	//DriveForward(7000);
	//TurnRight(45);
	//DriveForward(4000);

	// Take different routes depending on center goal position:
	// (Currently this does not work "magic number"-wise.)
	// There's probably a better way to do the whole lift thing. Not to mention, using
	// the servo on the main hopper is prolly gonna change depending on how it is decided
	// to put the center goal ball in. Not to mention, at the start of this switch statement
	// the robot is facing the arena side at a 45 degree angle and towing a rolling goal
	// which is staying on through teleop. Dropping it and picking it back up isn't too
	// practical in terms of feasibility. Something may have to be done with the above code
	// in order to avoid spinning and pushing against the side of the arena with the rolling goal.

    // All 3 cases mostly do the same thing. Drive up to the center goal, dump the ball and
    // drive back. Then align with the side of the arena and drive forwards a bit to make sure
    // that the rolling goal is inside the parking zone. Depending on what happens, we might need
    // to implement a small turn in position 1 to not bump into the ramp as we back up.
    // An unknkown goal position means we just center the rolling goal, or maybe just guess that it's
    // position 3, no crashing happens if it isn't.

	switch (centerGoalPos) {		//these are all conservative guesses so when we test we most likely wont crash
		default :					//intentional fall-through
		case CENTER_POS_UNKNOWN :	//guesses that it is pos_1 since we won't crash into anything
			lift_target = LIFT_BOTTOM;
			TurnLeft(50);
			DriveForward(12000);
            break;

		//case CENTER_POS_1 :			//goal faces the parking zone
  //          TurnLeft(45);
		//    DriveForward(2000);
		//    TurnRight(90);
		//    DriveForward(1600);
		//    TurnLeft(90);
		//    DriveBackward(600);

  //          DumpAuton();
  //          lift_target = LIFT_BOTTOM;

  //          DriveForward(1600);
  //          TurnRight(1350);
  //          DriveBackward(500);
  //          break;

		//case CENTER_POS_2 :				//goal faces corner
		//	TurnLeft(45);
		//    DriveForward(800);
		//    TurnRight(45);
  //          DriveForward(3000);
  //          TurnRight(45);
  //          DriveForward(1000);
  //          TurnLeft(45);
  //          DriveBackward(800);

  //          DumpAuton();
		//	lift_target = LIFT_BOTTOM;

  //          TurnRight(90);
  //          DriveBackward(2400);
  //      	break;

		//case CENTER_POS_3 :				//goal faces the side of the arena
		//    DriveBackward(4000);
		//    TurnRight(135);
		//    DriveBackward(100);

  //          DumpAuton();
  //          lift_target = LIFT_BOTTOM;

  //          TurnRight(45);
  //          DriveBackward(3000);
  //          TurnRight(45);
  //          DriveBackward(800);
  //          break;
	}

	Motor_SetPower(0, motor_clamp_L);
	Motor_SetPower(0, motor_clamp_R);

	// Lower lift:
	// We need to be extra sure that the lift lowers completely. Do NOT get rid
	// of the delay at the end!
	lift_target = LIFT_BOTTOM;
	while (true) {
		PlaySound(soundUpwardTones);
		Time_Wait(1000);
	}
}

void DumpBall()
{
	Servo_SetPosition(servo_dump, pos_dump_open);
	Time_Wait(1600);
	Servo_SetPosition(servo_dump, pos_dump_closed);
	Time_Wait(600);
}

void DumpAuton()
{
	Servo_SetPosition(servo_auton, pos_auton_open);
	Time_Wait(1600);
	Servo_SetPosition(servo_auton, pos_auton_closed);
	Time_Wait(800);
}

bool Drive(int encoder_count)
{
	target_dist_disp = encoder_count;

	bool isSuccess = false;

	int timer_watchdog = 0;
	Time_ClearTimer(timer_watchdog);
	const float watchdog_encoder_rate = 1.736;
	const float watchdog_base = 2000.0;
	int time_limit = (int)round((float)abs(encoder_count*watchdog_encoder_rate)+watchdog_base);
	const int acceptable_error = 200;

	const int finish_limit = 750; // msec
	bool isFinishing = false;
	int timer_finish = 0;
	Time_ClearTimer(timer_finish);

	const float kP = 0.0133;
	const float kI = 0.0047;
	const float I_term_decay_rate = 0.91;

	int count_init_L = Motor_GetEncoder(encoder_L);
	int count_init_R = Motor_GetEncoder(encoder_R);
	int pos_L = Motor_GetEncoder(encoder_L) - count_init_L;
	int pos_R = Motor_GetEncoder(encoder_R) - count_init_R;
	int error_L = 0;
	int error_R = 0;
	float error_sum_L = 0.0;
	float error_sum_R = 0.0;
	float power_L = 0.0;
	float power_R = 0.0;
	float power_L_prev = 0.0;
	float power_R_prev = 0.0;

	const float kP_turn_correct = 5.1;
	float heading_init = heading;
	float heading_curr = heading_init;
	float heading_error = 0.0;
	float power_turn = 0.0;

	while (true) {
		heading_curr = heading;
		heading_error = heading_init - heading_curr;
		power_turn = kP_turn_correct * heading_error;

		power_L_prev = power_L;
		power_R_prev = power_R;
		pos_L = Motor_GetEncoder(encoder_L) - count_init_L;
		pos_L *= -1;
		pos_R = Motor_GetEncoder(encoder_R) - count_init_R;
		error_L = pos_L - encoder_count;
		error_R = pos_R - encoder_count;
		error_sum_L *= I_term_decay_rate;
		error_sum_R *= I_term_decay_rate;
		error_sum_L += error_L;
		error_sum_R += error_R;
		power_L = kP*error_L + kI*error_sum_L;
		power_R = kP*error_R + kI*error_sum_R;
		power_L = Math_Limit(power_L, 70);
		power_R = Math_Limit(power_R, 70);
		int power_final = (int)round((power_L+power_R)/2.0);
		power_turn = Math_Limit(power_turn, power_final*0.4);
		if (encoder_count<0) {
			power_turn *= -1;
		}
		power_L -= power_turn;
		power_R += power_turn;
		//power_L = power_final;
		//power_R = power_final;

		pos_dist_disp_L = pos_L;
		pos_dist_disp_R = pos_R;
		error_dist_disp = (float)(error_L+error_R)/2.0;
		error_sum_dist_disp = (error_sum_L+error_sum_R)/2.0;
		power_dist_disp = (float)power_final;

		Motor_SetPower(power_L, motor_L);
		Motor_SetPower(power_R, motor_R_A);
		Motor_SetPower(power_R, motor_R_B);

		if ((abs(error_L)<acceptable_error) && (abs(error_R)<acceptable_error)) {
			if (isFinishing == false) {
				Time_ClearTimer(timer_finish);
			}
			isFinishing = true;
		} else {
			isFinishing = false;
		}

		if ((isFinishing == true)&&(Time_GetTime(timer_finish)>finish_limit)) {
			isSuccess = true;
			break;
		}
		if ((isFinishing == false)&&(Time_GetTime(timer_watchdog)>time_limit)) {
			isSuccess = false;
			break;
		}

		Time_Wait(2);
	}

	Motor_SetPower(0, motor_L);
	Motor_SetPower(0, motor_R_A);
	Motor_SetPower(0, motor_R_B);

	return isSuccess;
}

bool Turn(int degrees)
{
	target_angle_disp = degrees;

	bool isSuccess = false;

	int timer_watchdog = 0;
	Time_ClearTimer(timer_watchdog);
	const float watchdog_degree_rate = 0.0139;
	const float watchdog_base = 1800.0;
	int time_limit = (int)round((float)abs(degrees)*watchdog_degree_rate+watchdog_base);
	const float acceptable_error = 1.0;

	const int finish_limit = 1250; // msec
	bool isFinishing = false;
	int timer_finish = 0;
	Time_ClearTimer(timer_finish);

	const float kP = 7.2;
	const float kI = 0.14;
	const float I_term_decay_rate = 0.94;

	float heading_init = heading;
	float heading_curr = heading;
	float error = 0.0;
	float error_sum = 0.0;
	float power = 0.0;
	float power_prev = 0.0;

	while (true) {
		power_prev = power;
		heading_curr = heading - heading_init;
		error = heading_curr - (float)degrees;
		error_sum *= I_term_decay_rate;
		error_sum += error;
		float kP_var = kP;
		float kI_var = kI;
		power = kP_var*error + kI_var*error_sum;
		power = Math_Limit(power, 65.0);

		int power_L = (int)round(power);
		power_L *= -1;
		int power_R = (int)round(power);
		power_R *= 1.6;

		curr_angle_disp = heading_curr;
		error_angle_disp = error;
		error_sum_angle_disp = error_sum;
		power_angle_disp = power_R;

		Motor_SetPower(power_L, motor_L);
		Motor_SetPower(power_R, motor_R_A);
		Motor_SetPower(power_R, motor_R_B);

		if (abs(error)<acceptable_error) {
			if (isFinishing == false) {
				Time_ClearTimer(timer_finish);
			}
			isFinishing = true;
		} else {
			isFinishing = false;
		}

		if ((isFinishing == true)&&(Time_GetTime(timer_finish)>finish_limit)) {
			isSuccess = true;
			break;
		}
		if ((isFinishing == false)&&(Time_GetTime(timer_watchdog)>time_limit)) {
			isSuccess = false;
			break;
		}

		Time_Wait(2);
	}

	Motor_SetPower(0, motor_L);
	Motor_SetPower(0, motor_R_A);
	Motor_SetPower(0, motor_R_B);

	return isSuccess;
}

bool DriveForward(int encoder_count)
{
	int temp_count = encoder_count * -1;
	return Drive(temp_count);
}
bool DriveBackward(int encoder_count)
{
	return Drive(encoder_count);
}
bool TurnLeft(int degrees)
{
	int temp_degrees = degrees * -1;
	return Turn(temp_degrees);
}
bool TurnRight(int degrees)
{
	return Turn(degrees);
}

task Gyro()
{
	HTGYROstartCal(sensor_gyro);
	float vel_curr = 0.0;
	float vel_prev = 0.0;
	float dt = 0.0;
	int timer_gyro = 0;
	Time_ClearTimer(timer_gyro);

	while (true) {
		vel_prev = vel_curr;
		dt = (float)Time_GetTime(timer_gyro)/(float)1000.0; // msec to sec
		Time_ClearTimer(timer_gyro);
		vel_curr = (float)HTGYROreadRot(sensor_gyro);
		heading += ((float)vel_prev+(float)vel_curr)*(float)0.5*(float)dt;
		Time_Wait(1);
	}
}

task PID()
{
	const float kP_up = 0.083;
	const float kI_up = 0.011;
	const float kD_up = 0.0;
	const float kP_down = 0.079;
	const float kI_down = 0.008;
	const float kD_down = 0.0;
	const float I_term_decay_rate = 0.8;

	int timer_loop = 0;
	Time_ClearTimer(timer_loop);
	int dt = Time_GetTime(timer_loop);

	int lift_pos = Motor_GetEncoder(encoder_lift);

	float error = 0.0;
	float error_prev = 0.0;
	float error_sum = 0.0;
	float error_rate = 0.0;

	Joystick_WaitForStart();
	Time_ClearTimer(timer_loop);

	while (true) {
		dt = Time_GetTime(timer_loop);
		Time_ClearTimer(timer_loop);
		error_prev = error;
		lift_pos = Motor_GetEncoder(encoder_lift);


		if (lift_target<LIFT_BOTTOM) {
			lift_target = LIFT_BOTTOM;
		}
		if (lift_target>LIFT_TOP) {
			lift_target = LIFT_TOP;
		}
		error = lift_target - lift_pos;
		if (error > 0) {
			isDown = false;
		} else {
			isDown = true;
		}
		error_sum *= I_term_decay_rate;
		error_sum += error * (int)dt;
		error_rate = (error - error_prev) / (int)dt;

		term_P_lift = error;
		term_I_lift = error_sum;
		term_D_lift = error_rate;
		switch (isDown) {
			case true :
				term_P_lift *= kP_down;
				term_I_lift *= kI_down;
				term_D_lift *= kD_down;
				break;
			case false :
				term_P_lift *= kP_up;
				term_I_lift *= kI_up;
				term_D_lift *= kD_up;
				break;
		}
		power_lift = term_P_lift + term_I_lift + term_D_lift;

		if (abs(power_lift)<10) {
			power_lift = 0;
		}

		Motor_SetPower(-power_lift, motor_lift_A);
		Motor_SetPower(power_lift, motor_lift_B);

		if (Motor_GetPower(motor_lift_B)<-10) { // motor_lift_A is opposite encoders
			Motor_SetPower(-100, motor_assist);
		} else {
			Motor_SetPower(0, motor_assist);
		}

		Time_Wait(2);
	}
}

task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_ENCODERS,			// Raw encoder values.
		DISP_PID_LIFT,
		DISP_PID_ENCODERS,
		DISP_PID_ANGLE,
		DISP_SENSORS,			// Might need to split this into two screens.
		DISP_NUM
	};

	Task_Spawn(displayDiagnostics); // Explicit here: this is only spawned when buttons are pressed.
	DisplayMode isMode = DISP_FCS;

	// We don't need to wait for start. ;)

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_ENCODERS :
				nxtDisplayTextLine(0, "Lift:  %+6i", Motor_GetEncoder(encoder_lift));
				nxtDisplayTextLine(1, "  Tgt: %+6i", lift_target);
				nxtDisplayTextLine(2, "  Pwr: %+6i", power_lift);
				nxtDisplayTextLine(3, "Mtr L: %+6i", Motor_GetEncoder(encoder_L));
				nxtDisplayTextLine(4, "Mtr R: %+6i", Motor_GetEncoder(encoder_R));
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "Angle: %3d", heading);
				nxtDisplayTextLine(1, "IR A:  %3d", IR_A);
				nxtDisplayTextLine(2, "IR B:  %3d", IR_B);
				nxtDisplayTextLine(3, "IR C:  %3d", IR_C);
				nxtDisplayTextLine(4, "IR D:  %3d", IR_D);
				nxtDisplayTextLine(5, "IR E:  %3d", IR_E);
				nxtDisplayTextLine(6, "Light: %3d", light_intensity);
				switch (centerGoalPos) {
					case CENTER_POS_1 :
						nxtDisplayTextLine(7, "pos 1");
						break;
					case CENTER_POS_2 :
						nxtDisplayTextLine(7, "pos 2");
						break;
					case CENTER_POS_3 :
						nxtDisplayTextLine(7, "pos 3");
						break;
					case CENTER_POS_UNKNOWN :
						nxtDisplayTextLine(7, "IDK wutttt");
						break;
				}
				break;
			case DISP_PID_LIFT :
				nxtDisplayTextLine(0, "P: %+7d", term_P_lift);
				nxtDisplayTextLine(1, "I: %+7d", term_I_lift);
				nxtDisplayTextLine(2, "D: %+7d", term_D_lift);
				if (is_lift_manual) {
					nxtDisplayTextLine(3, "MANUAL");
				} else {
					nxtDisplayTextLine(3, "PID");
				}
				if (isDown) {
					nxtDisplayTextLine(4, "DOWN");
				} else {
					nxtDisplayTextLine(4, "UP");
				}
				break;
			case DISP_PID_ENCODERS :
				nxtDisplayTextLine(0, "trgt : %+6i", target_dist_disp);
				nxtDisplayTextLine(1, "pos L: %+6i", pos_dist_disp_L);
				nxtDisplayTextLine(2, "pos R: %+6i", pos_dist_disp_R);
				nxtDisplayTextLine(3, "error: %+6i", error_dist_disp);
				nxtDisplayTextLine(4, "e_sum: %+6i", error_sum_dist_disp);
				nxtDisplayTextLine(5, "power: %+6i", power_dist_disp);
				nxtDisplayTextLine(6, "P    : %+6i", term_P_dist);
				nxtDisplayTextLine(7, "I    : %+6i", term_I_dist);
				break;
			case DISP_PID_ANGLE :
				nxtDisplayTextLine(0, "trgt : %+6i", target_angle_disp);
				nxtDisplayTextLine(1, "angle: %+6i", curr_angle_disp);
				nxtDisplayTextLine(3, "error: %+6i", error_angle_disp);
				nxtDisplayTextLine(4, "e_sum: %+6i", error_sum_angle_disp);
				nxtDisplayTextLine(5, "power: %+6i", power_angle_disp);
				nxtDisplayTextLine(6, "P    : %+6i", term_P_angle);
				nxtDisplayTextLine(7, "I    : %+6i", term_I_angle);
				break;
			default :
				nxtDisplayCenteredTextLine(3, "Doesn't work...");
				nxtDisplayCenteredTextLine(4, "Yet. >:(");
				break;
		}

		if (Buttons_Released(NXT_BUTTON_L)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM-1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				Task_Spawn(displayDiagnostics);
			} else {
				Task_Kill(displayDiagnostics);
			}
		}
		if (Buttons_Released(NXT_BUTTON_R)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM+1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				Task_Spawn(displayDiagnostics);
			} else {
				Task_Kill(displayDiagnostics);
			}
		}
		Time_Wait(50); // MAGIC_NUM: Prevents the LCD from updating itself to death.
	}
}
