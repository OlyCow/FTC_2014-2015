#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Hubs,  S2, HTServo,  HTServo,  HTServo,  none)
#pragma config(Sensor, S3,     sensor_gyro,    sensorAnalogInactive)
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
#pragma config(Servo,  srvo_S2_C1_1,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    servo_turntable,      tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    servo12,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_hopper_A,       tServoStandard)
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
task Hopper();

typedef enum HopperPos {
	HOPPER_DOWN = 0,
	HOPPER_CENTER,
	HOPPER_GOAL
};

HopperPos hopper_pos = HOPPER_DOWN;
HopperPos hopper_target = HOPPER_DOWN;

float heading = 0.0;
int lift_pos = 0;
int lift_target = 0;
bool is_lift_manual = true;
bool isDown = false;
bool isReset = false;
bool isLiftFrozen = false;

float term_P_lift = 0.0;
float term_I_lift = 0.0;
float term_D_lift = 0.0;
float power_lift = 0.0;
float power_lift_temp = 0.0;

task main()
{
	typedef enum MotorDirection {
		DIRECTION_NONE = 0,
		DIRECTION_IN,
		DIRECTION_OUT
	};
	typedef enum PickupPos {
		PICKUP_UP = 0,
		PICKUP_RETRACT,
		PICKUP_LARGE,
		PICKUP_SMALL
	};

	initializeGlobalVariables();

	MotorDirection pickup_direction = DIRECTION_NONE;
	MotorDirection pickup_direction_prev = DIRECTION_NONE;
	MotorDirection clamp_direction = DIRECTION_NONE;
	PickupPos pickup_pos = PICKUP_LARGE;
	int servo_hopper_pos = pos_servo_hopper_down;
	int servo_dump_pos = pos_servo_dump_closed;

	float power_L		= 0.0;
	float power_R		= 0.0;
	float power_pickup	= 0.0;
	float power_clamp	= 0.0;

	Servo_SetPosition(servo_dump, pos_servo_dump_closed);
	Servo_SetPosition(servo_hopper_A, pos_servo_hopper_down);
	Servo_SetPosition(servo_hopper_B, pos_servo_hopper_down);
	Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_large);
	Servo_SetPosition(servo_pickup_R, 127 - pos_servo_pickup_large);

	Motor_ResetEncoder(encoder_lift);

	Task_Spawn(Gyro);
	Task_Spawn(PID);
	Task_Spawn(Display);
	Task_Spawn(Hopper);
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_lift_temp = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2);
		if (abs(power_lift_temp) > 5) {
			is_lift_manual = true;
		} else {
			is_lift_manual = false;
		}
		if (Joystick_Button(BUTTON_JOYL, CONTROLLER_2) == true) {
			isReset = true;
		} else {
			isReset = false;
		}

		power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y);
		power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y);

		if (Joystick_ButtonPressed(BUTTON_A)) {
			switch (pickup_direction) {
				case DIRECTION_NONE :
				case DIRECTION_OUT :
					if (lift_pos > pos_hopper_safety_up) {
						pickup_direction = DIRECTION_NONE;
					} else {
						pickup_direction = DIRECTION_IN;
					}
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

		if (Joystick_ButtonPressed(BUTTON_LB) || Joystick_ButtonPressed(BUTTON_RB)) {
			switch (pickup_pos) {
				case PICKUP_LARGE :
					pickup_pos = PICKUP_RETRACT;
					break;
				default :
					pickup_pos = PICKUP_LARGE;
					break;
			}
		}

		if (Joystick_ButtonPressed(BUTTON_X, CONTROLLER_2)) {
			lift_target = pos_lift_bottom;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_down;
			hopper_target = HOPPER_DOWN;
		} else if (Joystick_ButtonPressed(BUTTON_A, CONTROLLER_2)) {
			lift_target = pos_lift_low;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_goal;
			hopper_target = HOPPER_GOAL;
		} else if (Joystick_ButtonPressed(BUTTON_B, CONTROLLER_2)) {
			lift_target = pos_lift_medium;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_goal;
			hopper_target = HOPPER_GOAL;
		} else if (Joystick_ButtonPressed(BUTTON_Y, CONTROLLER_2)) {
			lift_target = pos_lift_high;
			is_lift_manual = false;
			servo_hopper_pos = pos_servo_hopper_goal;
			hopper_target = HOPPER_GOAL;
		}

		if (Joystick_Direction(DIRECTION_B, CONTROLLER_2)) {
			clamp_direction = DIRECTION_IN;
		} else if (Joystick_Direction(DIRECTION_F, CONTROLLER_2)) {
			clamp_direction = DIRECTION_OUT;
		} else {
			clamp_direction = DIRECTION_NONE;
		}

		if (Joystick_Button(BUTTON_RT) || Joystick_Button(BUTTON_LT)) {
			if (lift_pos > pos_dump_safety) {
				servo_dump_pos = pos_servo_dump_open_dump;
			} else {
				servo_dump_pos = pos_servo_dump_closed;
			}
		} else {
			if (lift_pos > pos_dump_safety) {
				servo_dump_pos = pos_servo_dump_closed;
			} else {
				if (power_pickup > 50) {
					servo_dump_pos = pos_servo_dump_open_feed;
				} else {
					servo_dump_pos = pos_servo_dump_closed;
				}
			}
		}

		if (Joystick_ButtonPressed(BUTTON_X)) {
			switch (servo_hopper_pos) {
				case pos_servo_hopper_down :
					servo_hopper_pos = pos_servo_hopper_goal;
					hopper_target = HOPPER_GOAL;
					break;
				default :
					servo_hopper_pos = pos_servo_hopper_down;
					hopper_target = HOPPER_DOWN;
					break;
			}
		}
		if (Joystick_ButtonPressed(BUTTON_B)) {
			switch (servo_hopper_pos) {
				// Intentional fall-through:
				case pos_servo_hopper_down :
				case pos_servo_hopper_goal :
					servo_hopper_pos = pos_servo_hopper_center;
					hopper_target = HOPPER_CENTER;
					break;
				default :
					servo_hopper_pos = pos_servo_hopper_down;
					hopper_target = HOPPER_DOWN;
					break;
			}
		}

		if (lift_pos > pos_hopper_safety_up) {
			if (pickup_direction==DIRECTION_IN) {
				pickup_direction = DIRECTION_OUT;
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
		// NOTE: These should be set in the "Hopper" task.
		//Servo_SetPosition(servo_hopper_T, 128 + servo_hopper_pos);
		//Servo_SetPosition(servo_hopper_B, 128 - servo_hopper_pos);
		switch (pickup_pos) {
			case PICKUP_UP :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_up);
				Servo_SetPosition(servo_pickup_R, 127 - pos_servo_pickup_up);
				break;
			case PICKUP_RETRACT :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_retract);
				Servo_SetPosition(servo_pickup_R, 127 - pos_servo_pickup_retract);
				break;
			case PICKUP_LARGE :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_large);
				Servo_SetPosition(servo_pickup_R, 127 - pos_servo_pickup_large);
				break;
			case PICKUP_SMALL :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_small);
				Servo_SetPosition(servo_pickup_R, 127 - pos_servo_pickup_small);
				break;
		}

		Time_Wait(5);
	}
}

task Hopper()
{
	Joystick_WaitForStart();
	while (true) {
		if (hopper_pos != hopper_target) {
			int timer_hopper = 0;
			int lift_target_prev = lift_target;
			switch (hopper_target) {
				case HOPPER_DOWN :
					if (lift_target_prev < pos_hopper_safety_down) {
						lift_target = pos_hopper_safety_above;
						is_lift_manual = false;
					}
					for (int i=0; i<10; i++) {
						Servo_SetPosition(servo_hopper_A, pos_servo_hopper_down);
						Servo_SetPosition(servo_hopper_B, pos_servo_hopper_down);
					}
					Time_ClearTimer(timer_hopper);
					while (Time_GetTime(timer_hopper)<2000) {
						if (lift_pos < pos_hopper_safety_down) {
							isLiftFrozen = true;
						} else {
							isLiftFrozen = false;
						}
						Time_Wait(10);
					}
					isLiftFrozen = false;
					lift_target = lift_target_prev;
					is_lift_manual = false;
					hopper_pos = HOPPER_DOWN;
					break;
				case HOPPER_CENTER :
					for (int i=0; i<10; i++) {
						Servo_SetPosition(servo_hopper_A, pos_servo_hopper_center);
						Servo_SetPosition(servo_hopper_B, pos_servo_hopper_center);
					}
					Time_ClearTimer(timer_hopper);
					while (Time_GetTime(timer_hopper)<1500) {
						Time_Wait(10);
					}
					isLiftFrozen = false;
					hopper_pos = HOPPER_CENTER;
					break;
				case HOPPER_GOAL :
					if (lift_target_prev < pos_hopper_safety_up) {
						lift_target = pos_hopper_safety_above;
						is_lift_manual = false;
					}
					while (lift_pos < pos_hopper_safety_up) {
						Time_Wait(10);
					}
					for (int i=0; i<10; i++) {
						Servo_SetPosition(servo_hopper_A, pos_servo_hopper_goal);
						Servo_SetPosition(servo_hopper_B, pos_servo_hopper_goal);
					}
					Time_ClearTimer(timer_hopper);
					while (Time_GetTime(timer_hopper)<3000) {
						if (lift_pos < pos_hopper_safety_up) {
							isLiftFrozen = true;
						} else {
							isLiftFrozen = false;
						}
						Time_Wait(10);
					}
					isLiftFrozen = false;
					lift_target = lift_target_prev;
					is_lift_manual = false;
					hopper_pos = HOPPER_GOAL;
					break;
			}
		}
		Time_Wait(5);
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
		vel_curr *= -1;
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

		if (isLiftFrozen) {
			power_lift = 0;
		}

		if (isReset == false) {
			if (power_lift>0 && lift_pos>pos_lift_top) {
				power_lift = 0;
			} else if (power_lift<0 && lift_pos<pos_lift_bottom) {
				power_lift = 0;
			}
		} else {
			Motor_ResetEncoder(encoder_lift);
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
				nxtDisplayTextLine(3, "Dist:  %+6i", Motor_GetEncoder(encoder_dist));
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "Angle: %3d", heading);
				break;
			case DISP_PID :
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
