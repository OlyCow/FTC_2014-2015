#pragma config(Hubs,  S1, HTServo,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Motor,  motorA,          motor_assist,  tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_clamp_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,          motor_clamp_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     motor_L,       tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motor_pickup,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_lift_A,  tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motor_lift_B,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motor_R_A,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motor_R_B,     tmotorTetrix, openLoop, reversed)
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

task main()
{
	int power_L = 0;
	int power_R = 0;
	int power_pickup = 0;
	int power_clamp = 0;
	int dump_position = 0;

	bool isPickingUp = false;

	Motor_ResetEncoder(encoder_lift);
	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);

	bDisplayDiagnostics = false;

	Task_Spawn(PID);
	Task_Spawn(Display);

	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
		power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		power_lift = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2);
		if (Motor_GetEncoder(motor_lift_A)<LIFT_BOTTOM) {
			if (power_lift<0) {
				power_lift = 0;
			}
		} else if (Motor_GetEncoder(motor_lift_A)>LIFT_TOP) {
			if (power_lift>0) {
				power_lift = 0;
			}
		}
		if (Joystick_ButtonReleased(BUTTON_X, CONTROLLER_1)) {
			isPickingUp = !isPickingUp;
		}
		if (isPickingUp == true) {
			power_pickup = 100;
		} else {
			power_pickup = 0;
		}
		if (Joystick_Direction(DIRECTION_F, CONTROLLER_2)) {
			power_clamp = -100;
		} else if (Joystick_Direction(DIRECTION_B, CONTROLLER_2)) {
			power_clamp = 100;
		} else {
			power_clamp = 0;
		}

		if (Joystick_Button(BUTTON_Y, CONTROLLER_2) == true) {
			dump_position = pos_dump_open;
		} else {
			dump_position = pos_dump_closed;
		}

		Motor_SetPower(power_L, motor_L);
		Motor_SetPower(power_R, motor_R_A);
		Motor_SetPower(power_R, motor_R_B);
		Motor_SetPower(power_lift, motor_lift_A);
		Motor_SetPower(power_lift, motor_lift_B);
		Motor_SetPower(power_pickup, motor_pickup);
		Motor_SetPower(power_pickup, motor_assist);
		Motor_SetPower(power_clamp, motor_clamp_L);
		Motor_SetPower(power_clamp, motor_clamp_R);

		Servo_SetPosition(servo_dump, dump_position);

		nxtDisplayCenteredBigTextLine(2, "pwr: %d", power_lift);
		nxtDisplayCenteredBigTextLine(4, "pos: %d", Motor_GetEncoder(motor_lift_A));
	}
}

task PID()
{
	bool isDown = true;

	const float kP_up = 1.0;
	const float kI_up = 0.0;
	const float kD_up = 0.0;
	const float kP_down = 1.0;
	const float kI_down = 0.0;
	const float kD_down = 0.0;

	float term_P = 0.0;
	float term_I = 0.0;
	float term_D = 0.0;

	int timer_loop = 0;
	Time_ClearTimer(timer_loop);
	int dt = Time_GetTime(timer_loop);

	int lift_pos = Motor_GetEncoder(encoder_lift);

	float error = 0.0;
	float error_prev = 0.0;
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

		power_lift = term_P + term_I + term_D;
	}
}

task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_ENCODERS,			// Raw encoder values.
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
				nxtDisplayTextLine(1, "Pwr:   %+6i", power_lift);
				nxtDisplayTextLine(1, "Mtr L: %+6i", Motor_GetEncoder(encoder_L));
				nxtDisplayTextLine(2, "Mtr R: %+6i", Motor_GetEncoder(encoder_R));
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "Angle: %3d", heading);
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
