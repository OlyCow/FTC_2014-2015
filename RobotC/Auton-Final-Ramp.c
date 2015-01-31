#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTServo,  none)
#pragma config(Sensor, S3,     sensor_gyro,    sensorAnalogInactive)
#pragma config(Sensor, S4,     sensor_IR,      sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_clamp_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_clamp_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motor_RB,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_RT,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motor_LT,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     motor_LB,      tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_lift_A,  tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift_B,  tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     motor_pickup,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motor_lift_C,  tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S2_C1_1,    servo_hopper_A,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_hopper_B,       tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo_pickup_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_3,    servo_pickup_R,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_1,    servo_turntable,      tServoStandard)
#pragma config(Servo,  srvo_S2_C3_2,    servo14,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_3,    servo15,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_5,    servo17,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_6,    servo18,              tServoNone)

#include "includes.h"
#include "final.h"

task Gyro();
task PID();
task Display();

float heading = 0.0;
int lift_pos = 0;
int lift_target = 0;
bool is_lift_manual = false;
bool isDown = false;

#include "final-auton.h"

float term_P_lift = 0.0;
float term_I_lift = 0.0;
float term_D_lift = 0.0;
float power_lift = 0.0;
float power_lift_temp = 0.0;

task main()
{
	initializeGlobalVariables();
	initializeRobotVariables();

	const int delay_settle = 400; // msec

	// All servos must be set to default position before "waitForStart"
	// to alleviate servo twitching. This also counts as the initialization
	// routine for the servos (only one command sent to each servo).
	// NOTE: If the refs call us out for having the servos move to two
	// different positions, tell them it's because the servos respond to
	// a garbage value in the controllers before actually responding to
	// our commands (and that there's nothing we can do about it). You
	// can also show them our code and prove that we only send one command
	// to each servo before the matches start.

	Task_Spawn(Gyro);
	Task_Spawn(PID);
	Task_Spawn(Display);
	Joystick_WaitForStart();
	heading = 0;
	Time_Wait(500);

	// Move down the ramp at full power (time-based dead reckoning).
	// NOTE: The commented out section would have broken the "move
	// down ramp" sequence into two parts: a fast part (to get over
	// the bump) and a slow part (to stay accurate). Currently the
	// times are NOT calibrated (i.e. a wild guess). Don't use the
	// commented out section unless our robot gets off by a lot.
	int ramp_power = -90;
	Motor_SetPower(ramp_power, motor_LT);
	Motor_SetPower(ramp_power, motor_LB);
	Motor_SetPower(ramp_power, motor_RT);
	Motor_SetPower(ramp_power, motor_RB);
	Time_Wait(1700);
	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);
	Time_Wait(delay_settle);

	// Minor correction turn. The turn is equal to our current heading
	// (in the opposite direction) to counteract any error we gained
	// while driving down the ramp.
	int correction_turn = heading;
	TurnLeft(correction_turn);

	// Drive backward slowly. This power should be slow enough that the
	// robot will not drive up the ramp if it hits it, but not so slow
	// such that the robot won't even drive.
	// NOTE: You can increase the time if the robot doesn't back up far
	// enough. This is still time-based dead reckoning.
	Motor_SetPower(15, motor_LT);
	Motor_SetPower(15, motor_LB);
	Motor_SetPower(15, motor_RT);
	Motor_SetPower(15, motor_RB);
	Time_Wait(1200);
	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);

	// Start raising lift early.
	lift_target = pos_lift_high;

	DriveBackward(900);
	TurnLeft(30);

	Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_large);
	Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_large);

	DriveBackward(2450);
	TurnRight(75);
	DriveBackward(1750);

	for (int i=0; i<10; i++) {
		Servo_SetPosition(servo_hopper_A, pos_servo_hopper_goal);
		Servo_SetPosition(servo_hopper_B, pos_servo_hopper_goal);
	}

	Motor_SetPower(100, motor_clamp_L);
	Motor_SetPower(100, motor_clamp_R);
	DriveBackward(900);
	Time_Wait(900); //wait for arm to stop shaking

	Servo_SetPosition(servo_dump, pos_servo_dump_open_dump);
	Time_Wait(600);
	Servo_SetPosition(servo_dump, pos_servo_dump_closed);

	for (int i=0; i<10; i++) {
		Servo_SetPosition(servo_hopper_A, pos_servo_hopper_down);
		Servo_SetPosition(servo_hopper_B, pos_servo_hopper_down);
	}

	TurnLeft(15);
	DriveForward(4000);
	lift_target = pos_lift_bottom;
	DriveForward(5000);
	TurnLeft(155);

	// Lower lift:
	// We need to be extra sure that the lift lowers completely. Do NOT get rid
	// of the delay at the end!
	Motor_SetPower(0, motor_clamp_L);
	Motor_SetPower(0, motor_clamp_R);
	lift_target = pos_lift_bottom;

	while (true) {
		PlaySound(soundUpwardTones);
		Time_Wait(1000);
	}
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
				nxtDisplayTextLine(3, "Dist:  %+6i", Motor_GetEncoder(encoder_L));
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
