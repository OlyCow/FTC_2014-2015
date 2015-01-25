#ifndef FINAL_H
#define FINAL_H

const tMotor encoder_lift	= motor_LT;
const tMotor encoder_dist	= motor_lift_B;

const int pos_lift_bottom			= 0;
const int pos_lift_low				= 1200;
const int pos_lift_medium			= 1200;
const int pos_lift_high				= 2750;
const int pos_lift_center			= 5500;
const int pos_lift_top				= 5500;
const int pos_hopper_safety_above	= 1400;
const int pos_hopper_safety_up		= 1000;
const int pos_hopper_safety_down	= 1500;
const int pos_dump_safety			= 100;

const int pos_servo_dump_closed		= 202;
const int pos_servo_dump_open_feed	= 171;
const int pos_servo_dump_open_dump	= 158;
const int pos_servo_hopper_down		= 30;
const int pos_servo_hopper_center	= 100;
const int pos_servo_hopper_goal		= 247;
const int pos_servo_pickup_up		= 69;	//servo_pickup_L (127+); servo_pickup_R (128-)
const int pos_servo_pickup_retract	= 52;
const int pos_servo_pickup_large	= 15;
const int pos_servo_pickup_small	= 13;

#endif // FINAL_H
