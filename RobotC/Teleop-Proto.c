#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S2,     sensor_IR,      sensorI2CCustom)
#pragma config(Sensor, S3,     sensor_light,   sensorLightActive)
#pragma config(Sensor, S4,     sensor_gyro,    sensorI2CHiTechnicGyro)
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

task PID();
task Display();

int heading = 0;
int lift_target = 0;
int power_lift = 0;
int power_lift_temp = 0;
bool is_lift_manual = false;
bool isDown = true;
int IR_A = 0;
int IR_B = 0;
int IR_C = 0;
int IR_D = 0;
int IR_E = 0;

float term_P = 0.0;
float term_I = 0.0;
float term_D = 0.0;

task main()
{
	int power_L = 0;
	int power_R = 0;
	int power_pickup = 0;
	int power_clamp = 0;
	int dump_position = 0;

	bool isPickingUp = false;

	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);
	Motor_ResetEncoder(encoder_lift);

	Task_Spawn(PID);
	Task_Spawn(Display);

	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
		power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		power_lift_temp = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2);

		int current_pos = Motor_GetEncoder(encoder_lift);
		if (current_pos<LIFT_BOTTOM) {
			if (power_lift_temp<0) {
				power_lift_temp = 0;
			}
		} else if (current_pos>LIFT_TOP) {
			if (power_lift_temp>0) {
				power_lift_temp = 0;
			}
		}

		if (power_lift_temp != 0) {
			is_lift_manual = true;
		} else {
			is_lift_manual = false;
		}

		if (Joystick_ButtonReleased(BUTTON_X, CONTROLLER_1)) {
			isPickingUp = !isPickingUp;
		}
		if (isPickingUp == true) {
			power_pickup = 100;
		} else {
			power_pickup = 0;
		}

		if (Joystick_Button(BUTTON_Y)) {
			power_pickup = -100;
		}

		if (Joystick_Direction(DIRECTION_F, CONTROLLER_2)) {
			power_clamp = -100;
		} else if (Joystick_Direction(DIRECTION_B, CONTROLLER_2)) {
			power_clamp = 100;
		} else {
			power_clamp = 0;
		}

		if (Joystick_Button(BUTTON_LT, CONTROLLER_2)||Joystick_Button(BUTTON_RT, CONTROLLER_2)) {
			dump_position = pos_dump_open;
		} else {
			dump_position = pos_dump_closed;
		}

		if (Joystick_ButtonReleased(BUTTON_X, CONTROLLER_2)) {
			lift_target = LIFT_BOTTOM;
		}
		if (Joystick_ButtonReleased(BUTTON_A, CONTROLLER_2)) {
			lift_target = LIFT_LOW;
		}
		if (Joystick_ButtonReleased(BUTTON_B, CONTROLLER_2)) {
			lift_target = LIFT_MID;
		}
		if (Joystick_ButtonReleased(BUTTON_Y, CONTROLLER_2)) {
			lift_target = LIFT_HIGH;
		}

		Motor_SetPower(power_L, motor_L);
		Motor_SetPower(power_R, motor_R_A);
		Motor_SetPower(power_R, motor_R_B);
		Motor_SetPower(power_pickup, motor_pickup);
		Motor_SetPower(power_pickup, motor_assist);
		Motor_SetPower(power_clamp, motor_clamp_L);
		Motor_SetPower(power_clamp, motor_clamp_R);

		Servo_SetPosition(servo_dump, dump_position);
	}
}

task PID()
{
	const float kP_up = 0.12;
	const float kI_up = 0.06;
	const float kD_up = 0.0;
	const float kP_down = 0.10;
	const float kI_down = 0.07;
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
		if (abs(power_lift)<10) {
			power_lift = 0;
		}

		Motor_SetPower(-power_lift, motor_lift_A);
		Motor_SetPower(power_lift, motor_lift_B);
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
				nxtDisplayTextLine(0, "Lift:  %+6i", Motor_GetEncoder(encoder_lift));
				nxtDisplayTextLine(1, "Tgt:   %+6i", lift_target);
				nxtDisplayTextLine(2, "Pwr:   %+6i", power_lift);
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
				break;
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
