#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Hubs,  S2, HTMotor,  HTServo,  HTServo,  HTServo)
#pragma config(Sensor, S3,     sensor_gyro,    sensorAnalogInactive)
#pragma config(Sensor, S4,     sensor_IR,      sensorI2CCustom9V)
#pragma config(Motor,  motorA,          motor_clamp_R, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          motor_clamp_L, tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     motor_lift_A,  tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     motor_lift_B,  tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     motor_pickup_O, tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     motor_pickup_I, tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     motor_LT,      tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_2,     motor_LB,      tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C1_1,     motor_RT,      tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S2_C1_2,     motor_RB,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C4_1,    servo1,       tServoNone)
#pragma config(Servo,  srvo_S1_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C4_6,    servo6,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_1,    servo_hopper_A,       tServoStandard)
#pragma config(Servo,  srvo_S2_C2_2,    servo8,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_3,    servo9,               tServoNone)
#pragma config(Servo,  srvo_S2_C2_4,    servo10,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_5,    servo11,              tServoNone)
#pragma config(Servo,  srvo_S2_C2_6,    servo12,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_1,    servo_pickup_L,       tServoStandard)
#pragma config(Servo,  srvo_S2_C3_2,    servo14,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_3,    servo15,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_4,    servo16,              tServoNone)
#pragma config(Servo,  srvo_S2_C3_5,    servo_dump,           tServoStandard)
#pragma config(Servo,  srvo_S2_C3_6,    servo_turntable,      tServoStandard)
#pragma config(Servo,  srvo_S2_C4_1,    servo_hopper_B,       tServoStandard)
#pragma config(Servo,  srvo_S2_C4_2,    servo_pickup_R,       tServoStandard)
#pragma config(Servo,  srvo_S2_C4_3,    servo21,              tServoNone)
#pragma config(Servo,  srvo_S2_C4_4,    servo22,              tServoNone)
#pragma config(Servo,  srvo_S2_C4_5,    servo23,              tServoNone)
#pragma config(Servo,  srvo_S2_C4_6,    servo24,              tServoNone)

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

float heading		= 0.0;

int lift_pos		= 0;
int lift_target		= 0;
bool is_lift_manual	= true;
bool isDown			= false;
bool isReset		= false;
bool isLiftFrozen	= false;

float term_P_lift	= 0.0;
float term_I_lift	= 0.0;
float term_D_lift	= 0.0;
float power_lift	= 0.0;
float power_lift_temp = 0.0;

float hopper_x_pos	= 0.0;
float hopper_y_pos	= 0.0;
float hopper_z_pos	= 0.0;
float hopper_x_target = 0.0;
float hopper_y_target = 0.0;
float hopper_z_target = 0.0;
float hopper_r		= 0.0;
float hopper_theta	= 0.0;
float hopper_phi	= 0.0;
float hopper_h		= 0.0;

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
		PICKUP_SMALL,
		PICKUP_KICK
	};

	initializeGlobalVariables();
	initializeRobotVariables();

	const short servo_updates_per_sec = 4;
	servoChangeRate[servo_hopper_A] = servo_updates_per_sec;
	servoChangeRate[servo_hopper_B] = servo_updates_per_sec;

	MotorDirection pickup_I_direction = DIRECTION_NONE;
	MotorDirection pickup_I_direction_prev = DIRECTION_NONE;
	MotorDirection pickup_O_direction = DIRECTION_NONE;
	MotorDirection pickup_O_direction_prev = DIRECTION_NONE;
	MotorDirection clamp_direction = DIRECTION_NONE;
	PickupPos pickup_pos = PICKUP_LARGE;
	int servo_hopper_pos = pos_servo_hopper_down;
	int servo_dump_pos = pos_servo_dump_closed;
	float servo_turntable_pos = pos_servo_turntable_front;

	float power_L			= 0.0;
	float power_R			= 0.0;
	float power_pickup_I	= 0.0;
	float power_pickup_O	= 0.0;
	float power_clamp		= 0.0;

	Task_Spawn(Gyro);
	Task_Spawn(PID);
	Task_Spawn(Display);
	Task_Spawn(Hopper);
	Joystick_WaitForStart();

	while (true) {
		Joystick_UpdateData();

		power_L = Joystick_GenericInput(JOYSTICK_L, AXIS_Y);
		power_R = Joystick_GenericInput(JOYSTICK_R, AXIS_Y);

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

		servo_turntable_pos += 0.01 * Joystick_GenericInput(JOYSTICK_R, AXIS_X, CONTROLLER_2);
		if (Joystick_Button(BUTTON_JOYR, CONTROLLER_2) || Joystick_ButtonReleased(BUTTON_JOYR, CONTROLLER_2)) {
			servo_turntable_pos = pos_servo_turntable_front;
			servo_hopper_pos = pos_servo_hopper_up;
		}
		servo_hopper_pos += 0.01 * Joystick_GenericInput(JOYSTICK_R, AXIS_Y, CONTROLLER_2);

		hopper_r = sqrt(hopper_x_target*hopper_x_target + hopper_y_target*hopper_y_target);
		hopper_theta = atan2(hopper_y_target, hopper_x_target);

		if (Joystick_Button(BUTTON_B)) {
			pickup_I_direction = DIRECTION_OUT;
		} else if (Joystick_Button(BUTTON_X)) {
			pickup_I_direction = DIRECTION_IN;
		} else {
			pickup_I_direction = DIRECTION_NONE;	// can be overridden later
		}

		if (Joystick_ButtonPressed(BUTTON_A)) {
			switch (pickup_O_direction) {
				case DIRECTION_NONE :
				case DIRECTION_OUT :
					pickup_O_direction = DIRECTION_IN;
					break;
				case DIRECTION_IN :
					pickup_O_direction = DIRECTION_NONE;
					break;
			}
		}
		if (Joystick_ButtonPressed(BUTTON_Y)) {
			pickup_O_direction_prev = pickup_O_direction;
			pickup_I_direction_prev = pickup_I_direction;
			pickup_O_direction = DIRECTION_OUT;
			pickup_I_direction = DIRECTION_OUT;
		}
		if (Joystick_ButtonReleased(BUTTON_Y)) {
			pickup_O_direction = pickup_O_direction_prev;
			pickup_I_direction = pickup_I_direction_prev;
		}

		if (Joystick_DirectionPressed(DIRECTION_R)) {
			pickup_pos = PICKUP_RETRACT;
		} else if (Joystick_DirectionPressed(DIRECTION_F)) {
			pickup_pos = PICKUP_KICK;
		} else if (Joystick_DirectionPressed(DIRECTION_L)) {
			pickup_pos = PICKUP_LARGE;
		} else if (Joystick_DirectionPressed(DIRECTION_B)) {
			pickup_pos = PICKUP_SMALL;
		}

		if (Joystick_Button(BUTTON_LB)) {
			clamp_direction = DIRECTION_OUT;
		} else if (Joystick_Button(BUTTON_LT)) {
			clamp_direction = DIRECTION_IN;
		} else {
			clamp_direction = DIRECTION_NONE;
		}

		if (Joystick_Button(BUTTON_RB) || Joystick_Button(BUTTON_RB, CONTROLLER_2)) {
			servo_dump_pos = pos_servo_dump_open_small;
		} else if (Joystick_Button(BUTTON_RT) || Joystick_Button(BUTTON_RT, CONTROLLER_2)) {
			servo_dump_pos = pos_servo_dump_open_large;
		} else {
			servo_dump_pos = pos_servo_dump_closed;
		}

		if (Joystick_ButtonPressed(BUTTON_LB, CONTROLLER_2) {
			servo_hopper_pos = pos_servo_hopper_goal;
		}
		if (Joystick_ButtonPressed(BUTTON_LT, CONTROLLER_2) {
			servo_hopper_pos = pos_servo_hopper_center;
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

		switch (pickup_I_direction) {
			case DIRECTION_NONE :
				power_pickup_I = 0;
				break;
			case DIRECTION_IN :
				power_pickup_I = 100;
				break;
			case DIRECTION_OUT :
				power_pickup_I = -100;
				break;
		}
		switch (pickup_O_direction) {
			case DIRECTION_NONE :
				power_pickup_O = 0;
				break;
			case DIRECTION_IN :
				power_pickup_O = 100;
				break;
			case DIRECTION_OUT :
				power_pickup_O = -100;
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
		Motor_SetPower(power_pickup_I, motor_pickup_I);
		Motor_SetPower(power_pickup_O, motor_pickup_O);
		Motor_SetPower(power_clamp, motor_clamp_L);
		Motor_SetPower(power_clamp, motor_clamp_R);

		Servo_SetPosition(servo_dump, servo_dump_pos);
		servo_turntable_pos = Math_Limit(servo_turntable_pos, pos_servo_turntable_back);
		Servo_SetPosition(servo_turntable, (int)round(servo_turntable_pos));
		// NOTE: Hopper servos should be set in the "Hopper" task.
		switch (pickup_pos) {
			case PICKUP_UP :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_up);
				Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_up);
				break;
			case PICKUP_RETRACT :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_retract);
				Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_retract);
				break;
			case PICKUP_LARGE :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_large);
				Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_large);
				break;
			case PICKUP_SMALL :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_small);
				Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_small);
				break;
			case PICKUP_KICK :
				Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_kick);
				Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_kick);
				break;
		}

		Time_Wait(5);
	}
}

task Hopper()
{
	Joystick_WaitForStart();
	while (true) {
		if (hopper_pos != hopper_target) { // if the hopper has to go somewhere
			int timer_hopper = 0; // timer = 0; this is set here rather than inside the case
			int lift_target_prev = lift_target; // prev target is where the lift was supposed to be going before the hopper thing was called
			switch (hopper_target) {
				case HOPPER_DOWN :
					if (lift_target_prev < pos_hopper_safety_down) {
						lift_target = pos_hopper_safety_above;
						is_lift_manual = false;
					}
					for (int i=0; i<10; i++) { // set the servos to the down position ten times (it doesn't always work with fewer tries)
						Servo_SetPosition(servo_hopper_A, pos_servo_hopper_down);
						Servo_SetPosition(servo_hopper_B, pos_servo_hopper_down);
					}
					Time_ClearTimer(timer_hopper);
					while (Time_GetTime(timer_hopper)<2000) { //stop lift moving for two seconds
						if (lift_pos < pos_hopper_safety_down) {
							isLiftFrozen = true;
						} else {
							isLiftFrozen = false;
						}
						Time_Wait(10);
					}
					isLiftFrozen = false;
					lift_target = lift_target_prev; // put the lift where it is supposed to be. this might actually be kinda screwy
					is_lift_manual = false;
					hopper_pos = HOPPER_DOWN;  // ends the conditional above; the hopper as arrived
					break;
				case HOPPER_CENTER :
					for (int i=0; i<10; i++) { // set servos to center position ten times
						//Servo_SetPosition(servo_hopper_A, pos_servo_hopper_up);
						//Servo_SetPosition(servo_hopper_B, pos_servo_hopper_up);
						Servo_SetPosition(servo_hopper_A, pos_servo_hopper_center);
						Servo_SetPosition(servo_hopper_B, pos_servo_hopper_center);
					}
					Time_ClearTimer(timer_hopper);
					while (Time_GetTime(timer_hopper)<1500) { // wait for 1.5 seconds
						Time_Wait(10);
					}
					isLiftFrozen = false; //unfreeze lift - were is it frozen?
					hopper_pos = HOPPER_CENTER; // it got there
					break;
				case HOPPER_GOAL :
					if (lift_target_prev < pos_hopper_safety_up) { // if the position the lift was supposed to be going to is below the safety position, set it to above the safety position. this shouldn't happen very often.
						lift_target = pos_hopper_safety_above;
						is_lift_manual = false;
					}
					while (lift_pos < pos_hopper_safety_up) { // wait for the lift to raise aboev the safety position before carrying on.
						Time_Wait(10);
					}
					for (int i=0; i<10; i++) { // signal ten times
						Servo_SetPosition(servo_hopper_A, pos_servo_hopper_goal);
						Servo_SetPosition(servo_hopper_B, pos_servo_hopper_goal);
					}
					Time_ClearTimer(timer_hopper);
					while (Time_GetTime(timer_hopper)<3000) { //don't let the lift be lowered back below the safety position for 3 seconds.
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
					hopper_pos = HOPPER_GOAL; // we made it!
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

		Time_Wait(2);
	}
}

task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,			// Default FCS screen.
		DISP_ENCODERS,		// Raw encoder values.
		DISP_LIFT,			// PID, status, mode
		DISP_HOPPER,		// Maths variables.
		DISP_SENSORS,		// Might need to split this into two screens.
		DISP_JOYSTICKS,		// For convenience. TODO: Add buttons, D-pad, etc.?
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
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "Angle: %3i", heading);
				break;
			case DISP_LIFT :
				string lift_manual_str;
				string lift_status_str;
				if (is_lift_manual) {
					lift_manual_str = "MANUAL";
				} else {
					lift_manual_str = "PID";
				}
				if (isDown) {
					lift_status_str = "DOWN";
				} else {
					lift_status_str = "UP";
				}
				nxtDisplayCenteredTextLine(0, "Lift-%s-%s", lift_manual_str, lift_status_str);
				nxtDisplayTextLine(1, "Pos: %+6i", lift_pos);
				nxtDisplayTextLine(2, "Tgt: %+6i", lift_target);
				nxtDisplayTextLine(3, "Pwr: %+3.3f", power_lift);
				nxtDisplayCenteredTextLine(4, "%+4i  %+4i  %+4i", term_P_lift, term_I_lift, term_D_lift);
				break;
			case DISP_HOPPER :
				nxtDisplayTextLine(0, "XYZ %+2.1f %+2.1f %3i", hopper_x_pos, hopper_y_pos, hopper_z_pos);
				nxtDisplayTextLine(1, "XYZ %+2.1f %+2.1f %3i", hopper_x_target, hopper_y_target, hopper_z_target);
				nxtDisplayTextLine(2, "k,i %+4i %+4i", hopper_theta, hopper_phi);
				nxtDisplayTextLine(3, "r,h %3.1f %3.1f", hopper_r, hopper_h);
				break;
			case DISP_JOYSTICKS :
				nxtDisplayCenteredTextLine(0, "--Driver I:--");
				nxtDisplayCenteredTextLine(1, "LX:%4i RX:%4i", joystick.joy1_x1, joystick.joy1_x2);
				nxtDisplayCenteredTextLine(2, "LY:%4i RY:%4i", joystick.joy1_y1, joystick.joy1_y2);
				nxtDisplayCenteredTextLine(4, "--Driver II:--");
				nxtDisplayCenteredTextLine(5, "LX:%4i RX:%4i", joystick.joy2_x1, joystick.joy2_x2);
				nxtDisplayCenteredTextLine(6, "LY:%4i RY:%4i", joystick.joy2_y1, joystick.joy2_y2);
				break;
			default :
				nxtDisplayCenteredTextLine(2, "Debug info");
				nxtDisplayCenteredTextLine(3, "for this screen");
				nxtDisplayCenteredTextLine(4, "is not currently");
				nxtDisplayCenteredTextLine(5, "available.");
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
