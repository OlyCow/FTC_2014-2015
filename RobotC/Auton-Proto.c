#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S2,     sensor_IR,      sensorI2CCustom)
#pragma config(Sensor, S3,     sensor_light,   sensorLightActive)
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
#pragma config(Servo,  srvo_S1_C1_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C1_6,    servo6,               tServoNone)

#include "includes.h"
#include "proto.h"

task Gyro();
task PID();
task Display();

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

task main()
{
	typedef enum CenterGoalPos {
		CENTER_POS_UNKNOWN	= -1,
		CENTER_POS_1		= 0,
		CENTER_POS_2		= 1,
		CENTER_POS_3		= 2,
		CENTER_POS_NUM
	} CenterGoalPos;
	
	CenterGoalPos centerGoalPos = CENTER_POS_UNKNOWN;
	
	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);
	Motor_ResetEncoder(encoder_lift);

	HTIRS2setDSPMode(sensor_IR, DSP_1200);

	Task_Spawn(Gyro);
	Task_Spawn(PID);
	Task_Spawn(Display);

	Joystick_WaitForStart();

	bool test = Turn(30);

	while (test) {
		PlaySound(soundBeepBeep);
		Time_Wait(1000);
	}
	while (!test) {
		PlaySound(soundLowBuzz);
		Time_Wait(1000);
	}

	// Drive off of ramp (backward)
	// Sense IR, determine position of center goal
	Time_Wait(500);
	HTIRS2readAllACStrength(sensor_IR, IR_A, IR_B, IR_C, IR_D, IR_E);
	if (((IR_B+IR_C)>35) && (IR_B>15) && (IR_C>15)) {
		centerGoalPos = CENTER_POS_3;
	} else if (IR_B>25) {
		centerGoalPos = CENTER_POS_2;
	} else if ((IR_A + IR_B + IR_C)<10) {
		centerGoalPos = CENTER_POS_1;
	} // else it stays unknown.
	
	// Drive backward, turn left, drive backward, turn right, drive backward
	// Raise lift (to tall goal height)
	lift_target = LIFT_HIGH;
	
	// Pick up goal
	Motor_SetPower(100, motor_clamp_L);
	Motor_SetPower(100, motor_clamp_R);
	// drive forward a bit
	Motor_SetPower(100, motor_clamp_L);
	Motor_SetPower(100, motor_clamp_R);
	// wait for lift to raise
	
	// Dump balls (or just small ball)
	// Lower lift
	
	// The following depends on center goal position!
	switch (centerGoalPos) {
		case CENTER_POS_UNKNOWN :
			break;
		case CENTER_POS_1 :
			break;
		case CENTER_POS_2 :
			break;
		case CENTER_POS_3 :
			break;
	}
	
	// 	Drive forward, turn left, drive forward
	// 	Turn left, drive backward, turn right, drive backward
	// Raise lift (to center goal height)
	// Dump large ball
	// Drive forward, turn around (180)
	
	// Lower lift!!!
	lift_target = LIFT_BOTTOM;
	while (true) {
		Time_Wait(1000);
	}
}

bool Drive(int encoder_count)
{
	target_dist_disp = encoder_count;

	bool isSuccess = false;

	int timer_watchdog = 0;
	Time_ClearTimer(timer_watchdog);
	const float watchdog_encoder_rate = 1.736;
	const float watchdog_base = 2000.0;
	int time_limit = (int)round((float)encoder_count*watchdog_encoder_rate+watchdog_base);
	const int acceptable_error = 40;

	const int finish_limit = 1250; // msec
	bool isFinishing = false;
	int timer_finish = 0;
	Time_ClearTimer(timer_finish);

	const float kP = 0.0048;
	const float kI = 0.0029;
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

	while (true) {
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
		int power_final = (power_L+power_R)/2.0;
		power_L = power_final;
		power_R = power_final;

		pos_dist_disp_L = pos_L;
		pos_dist_disp_R = pos_R;
		error_dist_disp = (float)(error_L+error_R)/2.0;
		error_sum_dist_disp = (error_sum_L+error_sum_R)/2.0;
		power_dist_disp = (float)power_final;

		Motor_SetPower((int)round(power_L), motor_L);
		Motor_SetPower((int)round(power_R), motor_R_A);
		Motor_SetPower((int)round(power_R), motor_R_B);

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
	int time_limit = (int)round((float)degrees*watchdog_degree_rate+watchdog_base);
	const float acceptable_error = 1.0;

	const int finish_limit = 1500; // msec
	bool isFinishing = false;
	int timer_finish = 0;
	Time_ClearTimer(timer_finish);

	const float kP = 6.5;
	const float kI = 0.0;
	const float I_term_decay_rate = 0.91;

	float heading_init = heading;
	float heading_curr = heading;
	float error = 0.0;
	float error_sum = 0.0;
	float power = 0.0;
	float power_prev = 0.0;

	while (true) {
		power_prev = power;
		heading_curr = heading - heading_init;
		error = (float)degrees - heading_curr;
		error_sum *= I_term_decay_rate;
		error_sum += error;
		float kP_var = kP;
		float kI_var = kI;
		power = kP_var*error + kI_var*error_sum;
		power = Math_Limit(power, 70.0);

		int power_L = -1 * (int)round(power);
		int power_R = (int)round(power);

		curr_angle_disp = heading_curr;
		error_angle_disp = error;
		error_sum_angle_disp = error_sum;
		power_angle_disp = power;

		Motor_SetPower(power_L, motor_L);
		Motor_SetPower(power_R, motor_R_A);
		Motor_SetPower(power_R, motor_R_B);

		if (abs(error)<acceptable_error) {
			if (isFinishing == false) {
				Time_ClearTimer(timer_finish);
			}
			isFinishing = true;
			nxtDisplayCenteredTextLine(2, "!!!!!");
		} else {
			isFinishing = false;
		}

		if ((isFinishing == true)&&(Time_GetTime(timer_finish)>finish_limit)) {
			isSuccess = true;
			break;
		}
		//if ((isFinishing == false)&&(Time_GetTime(timer_watchdog)>time_limit)) {
		//	isSuccess = false;
		//	break;
		//}

		Time_Wait(2);
	}

	Motor_SetPower(0, motor_L);
	Motor_SetPower(0, motor_R_A);
	Motor_SetPower(0, motor_R_B);

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

		if (is_lift_manual == false) {
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
		} else {
			lift_target = lift_pos;
			power_lift = power_lift_temp;
		}
		if (abs(power_lift)<10) {
			power_lift = 0;
		}

		Motor_SetPower(-power_lift, motor_lift_A);
		Motor_SetPower(power_lift, motor_lift_B);

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
