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

task main()
{
	const int LIFT_UPPER_LIMIT = 4000;

	const int dump_pos_open = 190;
	const int dump_pos_closed = 240;

	int power_L = 0;
	int power_R = 0;
	int power_lift = 0;
	int power_pickup = 0;
	int power_clamp = 0;
	int dump_position = 0;

	bool isPickingUp = false;

	Motor_ResetEncoder(motor_lift_A);

	bDisplayDiagnostics = false;

	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_1);
		power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_1);
		power_lift = Joystick_GenericInput(JOYSTICK_L, AXIS_Y, CONTROLLER_2);
		if (Motor_GetEncoder(motor_lift_A)>-500) {
			if (power_lift<0) {
				power_lift = 0;
			}
		} else if (Motor_GetEncoder(motor_lift_A)<-8200) {
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

		if (Joystick_Button(BUTTON_Y) == true) {
			dump_position = dump_pos_open;
		} else {
			dump_position = dump_pos_closed;
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
