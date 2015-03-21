#ifndef FINAL_H
#define FINAL_H

void initializeRobotVariables();
void freezeRobot();

const tMotor encoder_L		= motor_LT;
const tMotor encoder_R		= motor_RT;
const tMotor encoder_lift	= motor_lift_A;
//const tMotor encoder_hopper	= ;

const int pos_lift_bottom			= 0;
const int pos_lift_low				= 288;
const int pos_lift_medium			= 328;
const int pos_lift_high				= 2355;
const int pos_lift_center			= 4356;
const int pos_lift_top				= 4356;
const int pos_hopper_safety_above	= 1089;
const int pos_hopper_safety_up		= 778;
const int pos_hopper_safety_down	= 1167;
const int pos_dump_safety			= 78;

const int pos_servo_dump_closed		= 201;
const int pos_servo_dump_open_feed	= 164;
const int pos_servo_dump_open_large	= 158;
const int pos_servo_dump_open_small	= 187;
const int pos_servo_hopper_down		= 16;
const int pos_servo_hopper_center	= 120;
const int pos_servo_hopper_up		= 190;
const int pos_servo_hopper_goal		= 238;
const int pos_servo_pickup_up		= 78;	// servo_pickup_L (129+); servo_pickup_R (127-)
const int pos_servo_pickup_retract	= 78;
const int pos_servo_pickup_large	= 30;
const int pos_servo_pickup_small	= 21;
const int pos_servo_pickup_kick		= 60;	// TODO
const int pos_servo_turntable_F		= 128;	// TODO
const int pos_servo_turntable_L		= 65;	// TODO
const int pos_servo_turntable_R		= 193;	// TODO
const int pos_servo_turntable_BL	= 20;	// TODO
const int pos_servo_turntable_BR	= 235;	// TODO

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
	Servo_SetPosition(servo_turntable, pos_servo_turntable_F);

	HTIRS2setDSPMode(sensor_IR, DSP_1200);

	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);
	Motor_ResetEncoder(encoder_lift);
}

void freezeRobot()
{
	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);
	Motor_SetPower(0, motor_lift_A);
	Motor_SetPower(0, motor_lift_B);
}

#endif // FINAL_H
