#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTServo,  none)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
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

task PID();
task Display();

int lift_pos = 0;
int lift_target = 0;
bool is_lift_manual = true;
bool isDown = false;

float term_P = 0.0;
float term_I = 0.0;
float term_D = 0.0;
float power_lift = 0.0;
float power_lift_temp = 0.0;

task main()
{
	typedef enum MotorDirection {
		DIRECTION_NONE = 0,
		DIRECTION_IN,
		DIRECTION_OUT
	};

	initializeGlobalVariables();

	MotorDirection pickup_direction = DIRECTION_NONE;
	MotorDirection pickup_direction_prev = DIRECTION_NONE;
	MotorDirection clamp_direction = DIRECTION_NONE;
	int servo_hopper_pos = pos_servo_hopper_down;
	int servo_dump_pos = pos_servo_dump_closed;

	float power_L		= 0.0;
	float power_R		= 0.0;
	float power_pickup	= 0.0;
	float power_clamp	= 0.0;

	Servo_SetPosition(servo_dump, pos_servo_dump_closed);
	Servo_SetPosition(servo_hopper_T, 128 + pos_servo_hopper_down);
	Servo_SetPosition(servo_hopper_B, 128 - pos_servo_hopper_down);
	Servo_SetPosition(servo_pickup_L, 127 + pos_servo_pickup_large);
	Servo_SetPosition(servo_pickup_R, 128 - pos_servo_pickup_large);

	Motor_ResetEncoder(encoder_lift);

	Task_Spawn(PID);
	Task_Spawn(Display);
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_lift_temp = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2);
		if (abs(power_lift_temp) > 5) {
			is_lift_manual = true;
		} else {
			is_lift_manual = false;
		}

		power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y);
		power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y);

		if (Joystick_ButtonPressed(BUTTON_A)) {
			switch (pickup_direction) {
				case DIRECTION_NONE :
				case DIRECTION_OUT :
					pickup_direction = DIRECTION_IN;
					break;
				case DIRECTION_IN :
					pickup_direction = DIRECTION_NONE;
					break;
			}
		}
		if (Joystick_ButtonPressed(BUTTON_Y)) {
			pickup_direction_prev = pickup_direction;
			pickup_direction = DIRECTION_OUT;
		}
		if (Joystick_ButtonReleased(BUTTON_Y)) {
			pickup_direction = pickup_direction_prev;
		}

		if (Joystick_ButtonPressed(BUTTON_X, CONTROLLER_2)) {
			Motor_SetPower(0, motor_LT);
			Motor_SetPower(0, motor_LB);
			Motor_SetPower(0, motor_RT);
			Motor_SetPower(0, motor_RB);
			Motor_SetPower(0, motor_lift_A);
			Motor_SetPower(0, motor_lift_B);
			Motor_SetPower(0, motor_lift_C);
			hogCPU();
			lift_target = pos_lift_bottom;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_down;
			Servo_SetPosition(servo_hopper_T, 128 + servo_hopper_pos);
			Servo_SetPosition(servo_hopper_B, 128 - servo_hopper_pos);
			int timer_temp = 0;
			Time_ClearTimer(timer_temp);
			while (true) {
				if (Time_GetTime(timer_temp) > 3000) {
					break;
				}
			}
			releaseCPU();
		} else if (Joystick_ButtonPressed(BUTTON_A, CONTROLLER_2)) {
			lift_target = pos_lift_low;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_goal;
		} else if (Joystick_ButtonPressed(BUTTON_B, CONTROLLER_2)) {
			lift_target = pos_lift_medium;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_goal;
		} else if (Joystick_ButtonPressed(BUTTON_Y, CONTROLLER_2)) {
			lift_target = pos_lift_high;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_goal;
		}

		if (Joystick_Direction(DIRECTION_B, CONTROLLER_2)) {
			clamp_direction = DIRECTION_IN;
		} else if (Joystick_Direction(DIRECTION_F, CONTROLLER_2)) {
			clamp_direction = DIRECTION_OUT;
		} else {
			clamp_direction = DIRECTION_NONE;
		}

		if (power_pickup > 50) {
			if (Joystick_Button(BUTTON_RT) || Joystick_Button(BUTTON_LT)) {
				servo_dump_pos = pos_servo_dump_closed;
			} else {
				servo_dump_pos = pos_servo_dump_open;
			}
		} else {
			if (Joystick_Button(BUTTON_RT) || Joystick_Button(BUTTON_LT)) {
				servo_dump_pos = pos_servo_dump_open;
			} else {
				servo_dump_pos = pos_servo_dump_closed;
			}
		}

		if (Joystick_Button(BUTTON_RT) || Joystick_Button(BUTTON_LT)) {
			if (lift_pos > pos_dump_safety) {
				servo_dump_pos = pos_servo_dump_open;
			} else {
				servo_dump_pos = pos_servo_dump_closed;
			}
		} else {
			if (lift_pos > pos_dump_safety) {
				servo_dump_pos = pos_servo_dump_closed;
			} else {
				if (power_pickup > 50) {
					servo_dump_pos = pos_servo_dump_open;
				} else {
					servo_dump_pos = pos_servo_dump_closed;
				}
			}
		}


		if (Joystick_ButtonPressed(BUTTON_X)) {
			switch (servo_hopper_pos) {
				case pos_servo_hopper_down :
					servo_hopper_pos = pos_servo_hopper_goal;
					break;
				default :
					servo_hopper_pos = pos_servo_hopper_down;
					break;
			}
		}
		if (Joystick_ButtonPressed(BUTTON_B)) {
			switch (servo_hopper_pos) {
				case pos_servo_hopper_down :
					servo_hopper_pos = pos_servo_hopper_center;
					break;
				default :
					servo_hopper_pos = pos_servo_hopper_down;
					break;
			}
		}

		switch (pickup_direction) {
			case DIRECTION_NONE :
				power_pickup = 0;
				break;
			case DIRECTION_IN :
				power_pickup = 100;
				break;
			case DIRECTION_OUT :
				power_pickup = -100;
				break;
		}
		switch (clamp_direction) {
			case DIRECTION_NONE :
				power_clamp = 0;
				break;
			case DIRECTION_IN :
				power_clamp = 100;
				break;
			case DIRECTION_OUT :
				power_clamp = -100;
				break;
		}

		Motor_SetPower(power_L, motor_LT);
		Motor_SetPower(power_L, motor_LB);
		Motor_SetPower(power_R, motor_RT);
		Motor_SetPower(power_R, motor_RB);
		Motor_SetPower(power_pickup, motor_pickup);
		Motor_SetPower(power_clamp, motor_clamp_L);
		Motor_SetPower(power_clamp, motor_clamp_R);

		Servo_SetPosition(servo_dump, servo_dump_pos);
		if (lift_pos<pos_hopper_safety) {
			Servo_SetPosition(servo_hopper_T, 128 + pos_servo_hopper_down);
			Servo_SetPosition(servo_hopper_B, 128 - pos_servo_hopper_down);
		} else {
			Servo_SetPosition(servo_hopper_T, 128 + servo_hopper_pos);
			Servo_SetPosition(servo_hopper_B, 128 - servo_hopper_pos);
		}
		Servo_SetPosition(servo_pickup_L, 128 + pos_servo_pickup_large);
		Servo_SetPosition(servo_pickup_R, 127 - pos_servo_pickup_large);

		Time_Wait(5);
	}
}

task PID()
{
	const float kP_up = 0.06;
	const float kI_up = 0.013;
	const float kD_up = 0.0;
	const float kP_down = 0.03;
	const float kI_down = 0.012;
	const float kD_down = 0.0;
	const float I_term_decay_rate = 0.8;

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

			term_P = error;
			term_I = error_sum;
			term_D = error_rate;
			switch (isDown) {
				case true :
					term_P *= kP_down;
					term_I *= kI_down;
					term_D *= kD_down;
					break;
				case false :
					term_P *= kP_up;
					term_I *= kI_up;
					term_D *= kD_up;
					break;
			}
			power_lift = term_P + term_I + term_D;
		} else {
			lift_target = lift_pos;
			power_lift = power_lift_temp;
		}
		if (abs(power_lift)<15) {
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
		DISP_PID,
		DISP_SENSORS,			// Might need to split this into two screens.
		DISP_JOYSTICKS,			// For convenience. TODO: Add buttons, D-pad, etc.?
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
				nxtDisplayTextLine(0, "Lift:  %+6i", lift_pos);
				nxtDisplayTextLine(1, "Tgt:   %+6i", lift_target);
				nxtDisplayTextLine(2, "Pwr:   %+6i", power_lift);
				//nxtDisplayTextLine(3, "Mtr L: %+6i", Motor_GetEncoder(encoder_L));
				//nxtDisplayTextLine(4, "Mtr R: %+6i", Motor_GetEncoder(encoder_R));
				break;
			//case DISP_SENSORS :
			//	nxtDisplayTextLine(0, "Angle: %3d", heading);
			//	nxtDisplayTextLine(1, "IR A:  %3d", IR_A);
			//	nxtDisplayTextLine(2, "IR B:  %3d", IR_B);
			//	nxtDisplayTextLine(3, "IR C:  %3d", IR_C);
			//	nxtDisplayTextLine(4, "IR D:  %3d", IR_D);
			//	nxtDisplayTextLine(5, "IR E:  %3d", IR_E);
			//	nxtDisplayTextLine(6, "Light: %3d", light_intensity);
			//	break;
			case DISP_PID :
				nxtDisplayTextLine(0, "P: %+7d", term_P);
				nxtDisplayTextLine(1, "I: %+7d", term_I);
				nxtDisplayTextLine(2, "D: %+7d", term_D);
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
			case DISP_JOYSTICKS :
				nxtDisplayCenteredTextLine(0, "--Driver I:--");
				nxtDisplayCenteredTextLine(1, "LX:%4d RX:%4d", joystick.joy1_x1, joystick.joy1_x2);
				nxtDisplayCenteredTextLine(2, "LY:%4d RY:%4d", joystick.joy1_y1, joystick.joy1_y2);
				nxtDisplayCenteredTextLine(4, "--Driver II:--");
				nxtDisplayCenteredTextLine(5, "LX:%4d RX:%4d", joystick.joy2_x1, joystick.joy2_x2);
				nxtDisplayCenteredTextLine(6, "LY:%4d RY:%4d", joystick.joy2_y1, joystick.joy2_y2);
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
