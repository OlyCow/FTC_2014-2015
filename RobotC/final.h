#ifndef FINAL_H
#define FINAL_H

void initializeRobotVariables();
void freezeRobot();

const tMotor encoder_lift	= motor_lift_A;
const tMotor encoder_L		= motor_LT;
const tMotor encoder_R		= motor_RT;

const int pos_lift_bottom			= 0;
const int pos_lift_low				= 288;
const int pos_lift_medium			= 288;
const int pos_lift_high				= 1755;
const int pos_lift_center			= 4356;
const int pos_lift_top				= 4356;
const int pos_hopper_safety_above	= 1089;
const int pos_hopper_safety_up		= 778;
const int pos_hopper_safety_down	= 1167;
const int pos_dump_safety			= 78;

const int pos_servo_dump_closed		= 201;
const int pos_servo_dump_open_feed	= 171;
const int pos_servo_dump_open_dump	= 158;
const int pos_servo_dump_open_small	= 160;
const int pos_servo_hopper_down		= 16;
const int pos_servo_hopper_center	= 120;
const int pos_servo_hopper_up		= 190;
const int pos_servo_hopper_goal		= 238;
const int pos_servo_pickup_up		= 72;	//servo_pickup_L (129+); servo_pickup_R (127-)
const int pos_servo_pickup_retract	= 52;
const int pos_servo_pickup_large	= 29;
const int pos_servo_pickup_small	= 10;
const int pos_servo_turntable_front	= 128;
const int pos_servo_turntable_side	= 65;	// TODO

void initializeRobotVariables()
{
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
	Servo_SetPosition(servo_hopper_A, pos_servo_hopper_down);
	Servo_SetPosition(servo_hopper_B, pos_servo_hopper_down);
	Servo_SetPosition(servo_pickup_L, 129 + pos_servo_pickup_up);
	Servo_SetPosition(servo_pickup_R, 120 - pos_servo_pickup_up);
	Servo_SetPosition(servo_turntable, pos_servo_turntable_front);

	HTIRS2setDSPMode(sensor_IR, DSP_1200);

	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);
	Motor_ResetEncoder(encoder_lift);
}

#endif // FINAL_H
