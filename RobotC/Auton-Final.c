#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTServo,  none)
#pragma config(Sensor, S3,     sensor_gyro,    sensorAnalogInactive)
#pragma config(Motor,  motorA,          motor_clamp_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_clamp_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motor_RB,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     motor_RT,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motor_LT,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_LB,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_lift_A,  tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift_B,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motor_pickup,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motor_lift_C,  tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S2_C1_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_turntable,      tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo12,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_hopper_T,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_hopper_B,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C3_1,    servo_pickup_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C3_2,    servo_pickup_R,       tServoStandard)
#pragma config(Servo,  srvo_S2_C3_3,    servo15,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_5,    servo17,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_6,    servo18,              tServoNone)

#include "includes.h"
#include "final.h"

task Gyro();
task PID();
task Display();

bool DriveForward(int encoder_count);
bool DriveBackward(int encoder_count);
bool TurnLeft(int degrees);
bool TurnRight(int degrees);

bool Drive(int encoder_count);
bool Turn(int degrees);

float heading = 0.0;
int lift_pos = 0;
int lift_target = 0;
bool is_lift_manual = false;
bool isDown = false;

float term_P_lift = 0.0;
float term_I_lift = 0.0;
float term_D_lift = 0.0;
float power_lift = 0.0;
float power_lift_temp = 0.0;

int target_dist_disp = 0;
int pos_dist_disp = 0;
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

task main()
{
	initializeGlobalVariables();

	Motor_ResetEncoder(encoder_dist);
	Motor_ResetEncoder(encoder_lift);

	// All servos must be set to default position before "waitForStart"
	// to alleviate servo twitching. This also counts as the initialization
	// routine for the servos (only one command sent to each servo).
	// NOTE: If the refs call us out for having the servos move to two
	// different positions, tell them it's because the servos respond to
	// a garbage value in the controllers before actually responding to
	// our commands (and that there's nothing we can do about it). You
	// can also show them our code and prove that we only send one command
	// to each servo before the matches start.
	Servo_SetPosition(servo_dump, pos_servo_dump_closed);
	Servo_SetPosition(servo_hopper_T, 128 + pos_servo_hopper_down);
	Servo_SetPosition(servo_hopper_B, 128 - pos_servo_hopper_down);


	Task_Spawn(Gyro);
	Task_Spawn(PID);
	Task_Spawn(Display);
	Joystick_WaitForStart();
	Time_Wait(500);

	// Move down the ramp at full power (time-based dead reckoning).
	// NOTE: The commented out section would have broken the "move
	// down ramp" sequence into two parts: a fast part (to get over
	// the bump) and a slow part (to stay accurate). Currently the
	// times are NOT calibrated (i.e. a wild guess). Don't use the
	// commented out section unless our robot gets off by a lot.
	int ramp_power = -100;
	Motor_SetPower(ramp_power, motor_LT);
	Motor_SetPower(ramp_power, motor_LB);
	Motor_SetPower(ramp_power, motor_RT);
	Motor_SetPower(ramp_power, motor_RB);
	Time_Wait(1600);
	//ramp_power = -20;
	//Motor_SetPower(ramp_power, motor_LT);
	//Motor_SetPower(ramp_power, motor_LB);
	//Motor_SetPower(ramp_power, motor_RT);
	//Motor_SetPower(ramp_power, motor_RB);
	//Time_Wait(800);
	// TODO: If we were to detect color changes, we'd do it here.
	// Unfortunately it seems like we're just going with dead reckoning.
	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);
	Time_Wait(500);

	// Quick correction turn. The turn is set to our current heading
	// and in the opposite direction to counteract any drift we gained
	// from driving on the ramp.

	//int correction_turn = heading;
	//TurnLeft(correction_turn);

	// Drive backward slowly. This power should be slow enough that the
	// robot will not drive up the ramp if it hits it, but not so slow
	// such that the robot won't even drive.
	// NOTE: You can increase the time if the robot doesn't back up far
	// enough. This is still time-based dead reckoning.
	Motor_SetPower(15, motor_LT);
	Motor_SetPower(15, motor_LB);
	Motor_SetPower(15, motor_RT);
	Motor_SetPower(15, motor_RB);
	Time_Wait(700);
	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);

	// Start raising lift early.
	lift_target = pos_lift_high;

	DriveBackward(1500);
	TurnLeft(45);


	Servo_SetPosition(servo_pickup_L, 127 + pos_servo_pickup_large);
	Servo_SetPosition(servo_pickup_R, 128 - pos_servo_pickup_large);

	DriveBackward(2000);
	TurnRight(90);
	DriveBackward(1500);

	for (int i=0; i<10; i++) {
		Servo_SetPosition(servo_hopper_T, 128 + pos_servo_hopper_goal);
		Servo_SetPosition(servo_hopper_B, 128 - pos_servo_hopper_goal);
	}

	Motor_SetPower(100, motor_clamp_L);
	Motor_SetPower(100, motor_clamp_R);
	DriveBackward(1300);
	Time_Wait(600);
	DriveForward(3300);

	Servo_SetPosition(servo_dump, pos_servo_dump_open_dump);
	Time_Wait(1000);
	Servo_SetPosition(servo_dump, pos_servo_dump_closed);

	for (int i=0; i<10; i++) {
		Servo_SetPosition(servo_hopper_T, 128 + pos_servo_hopper_down);
		Servo_SetPosition(servo_hopper_B, 128 - pos_servo_hopper_down);
	}
	Time_Wait(3000);

	lift_target = pos_lift_bottom;

	/*

	TurnLeft(45);
	DriveForward(5000);
	TurnLeft(120);
	DriveForward(100);

	*/

	// Lower lift:
	// We need to be extra sure that the lift lowers completely. Do NOT get rid
	// of the delay at the end!
	Motor_SetPower(0, motor_clamp_L);
	Motor_SetPower(0, motor_clamp_R);
	lift_target = pos_lift_bottom;
//	while (true) {
//		PlaySound(soundUpwardTones);
//		Time_Wait(1000);	}
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

	int count_init = Motor_GetEncoder(encoder_dist);
	int pos_dist = Motor_GetEncoder(encoder_dist) - count_init;
	int error = 0;
	float error_sum = 0.0;
	float power = 0.0;
	float power_prev = 0.0;

	while (true) {
		power_prev = power;
		pos_dist = Motor_GetEncoder(encoder_dist) - count_init;
		error = pos_dist - encoder_count;
		error_sum *= I_term_decay_rate;
		error_sum += error;
		power = kP*error + kI*error_sum;
		power = Math_Limit(power, 85);

		pos_dist_disp = pos_dist;
		error_dist_disp = (float)(error);
		error_sum_dist_disp = error_sum;
		power_dist_disp = (float)power;

		Motor_SetPower(power, motor_LT);
		Motor_SetPower(power, motor_LB);
		Motor_SetPower(power, motor_RT);
		Motor_SetPower(power, motor_RB);

		if (abs(error) < acceptable_error) {
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

	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);

	return isSuccess;
}

bool Turn(int degrees)
{
	target_angle_disp = degrees;

	bool isSuccess = false;

	int timer_watchdog = 0;
	Time_ClearTimer(timer_watchdog);
	const float watchdog_degree_rate = 0.0279;
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
		power = Math_Limit(power, 80.0);

		int power_L = (int)round(power);
		power_L *= -1;
		int power_R = (int)round(power);
		power_R *= 1;

		curr_angle_disp = heading_curr;
		error_angle_disp = error;
		error_sum_angle_disp = error_sum;
		power_angle_disp = power;

		Motor_SetPower(power_L, motor_LT);
		Motor_SetPower(power_L, motor_LB);
		Motor_SetPower(power_R, motor_RT);
		Motor_SetPower(power_R, motor_RB);

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

	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);

	return isSuccess;
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
	const float kP_up = 0.067;
	const float kI_up = 0.013;
	const float kD_up = 0.0;
	const float kP_down = 0.01;
	const float kI_down = 0.004;
	const float kD_down = 0.0;
	const float I_term_decay_rate = 0.85;

	int timer_loop = 0;
	Time_ClearTimer(timer_loop);
	int dt = Time_GetTime(timer_loop);

	lift_pos = Motor_GetEncoder(encoder_lift);

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

		if (is_lift_manual == false) {
			if (lift_target < pos_lift_bottom) {
				lift_target = pos_lift_bottom;
			}
			if (lift_target > pos_lift_top) {
				lift_target = pos_lift_top;
			}
			error = lift_target - lift_pos;
			if (error > 0) {
				isDown = false;
			} else {
				isDown = true;
			}
			error_sum *= I_term_decay_rate;
			error_sum += error * (float)dt;
			error_rate = (error - error_prev) / (float)dt;

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
		} else {
			lift_target = lift_pos;
			power_lift = power_lift_temp;
		}
		if (abs(power_lift)<10) {
			power_lift = 0;
		}

		if (power_lift>0 && lift_pos>pos_lift_top) {
			power_lift = 0;
		} else if (power_lift<0 && lift_pos<pos_lift_bottom) {
			power_lift = 0;
		}

		Motor_SetPower(power_lift, motor_lift_A);
		Motor_SetPower(power_lift, motor_lift_B);
		Motor_SetPower(power_lift, motor_lift_C);

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
				nxtDisplayTextLine(3, "Dist:  %+6i", Motor_GetEncoder(encoder_dist));
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "Angle: %3d", heading);
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
				nxtDisplayTextLine(1, "pos L: %+6i", pos_dist_disp);
				nxtDisplayTextLine(2, "error: %+6i", error_dist_disp);
				nxtDisplayTextLine(3, "e_sum: %+6i", error_sum_dist_disp);
				nxtDisplayTextLine(4, "power: %+6i", power_dist_disp);
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
