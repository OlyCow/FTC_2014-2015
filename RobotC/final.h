#ifndef FINAL_H
#define FINAL_H

void initializeRobotVariables();
void freezeRobot();
int encoderToHopper(int encoder);

const short servo_updates_per_sec = 2;

const tMotor encoder_L		= motor_LT;
const tMotor encoder_R		= motor_RT;
const tMotor encoder_lift	= motor_lift_A;
const tMotor encoder_hopper	= motor_LB;

const int pos_lift_bottom			= 0;
const int pos_lift_low				= 288;
const int pos_lift_medium			= 218;
//const int pos_lift_high				= 1700;
int pos_lift_high					= 1700;
const int pos_lift_center			= 4100;
const int pos_lift_top				= 4150;
const int pos_hopper_safety_above	= 700;
const int pos_hopper_safety_up		= 900;
const int pos_hopper_safety_down	= 1200;
const int pos_hopper_safety_side	= 50;	// TODO
const int pos_dump_safety			= 90;

const int pos_servo_dump_closed		= 201;
const int pos_servo_dump_open_feed	= 164;
const int pos_servo_dump_open_large	= 159;
const int pos_servo_dump_open_small	= 187;
const int pos_servo_hopper_down		= 20;
const int pos_servo_hopper_center	= 128;
const int pos_servo_hopper_up		= 193;
const int pos_servo_hopper_goal		= 242;
const int pos_servo_pickup_retract	= 78;
const int pos_servo_pickup_large	= 28;
const int pos_servo_pickup_small	= 18;
const int pos_servo_pickup_kick		= 60;
const int pos_servo_turntable_F		= 132;
const int pos_servo_turntable_L		= 161;
const int pos_servo_turntable_R		= 103;
const int pos_servo_turntable_BL	= 190;
const int pos_servo_turntable_BR	= 84;

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
	Servo_SetPosition(servo_pickup_L, 128 + pos_servo_pickup_retract);
	Servo_SetPosition(servo_pickup_R, 131 - pos_servo_pickup_retract);
	Servo_SetPosition(servo_turntable, pos_servo_turntable_F);

	servoChangeRate[servo_hopper_A] = servo_updates_per_sec;
	servoChangeRate[servo_hopper_B] = servo_updates_per_sec;

	HTIRS2setDSPMode(sensor_IR, DSP_1200);

	Motor_ResetEncoder(encoder_L);
	Motor_ResetEncoder(encoder_R);
	Motor_ResetEncoder(encoder_lift);
	Motor_ResetEncoder(encoder_hopper);
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

int encoderToHopper(int encoder)
{
	float conv = encoder * ((pos_servo_hopper_goal-pos_servo_hopper_down) / 803.0) + pos_servo_hopper_down;
	return (int)round(conv);
}

#endif // FINAL_H
