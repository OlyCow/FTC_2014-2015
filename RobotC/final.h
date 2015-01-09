#ifndef FINAL_H
#define FINAL_H

const tMotor encoder_lift	= motor_LT;
const tMotor encoder_dist	= motor_lift_B; // TODO: NOT ACTUALLY PLUGGED IN THERE

const int pos_lift_bottom	= 0;
const int pos_lift_low		= 1000;
const int pos_lift_medium	= 1500;
const int pos_lift_high		= 2500;
const int pos_lift_center	= 5500;
const int pos_lift_top		= 5550;
const int pos_hopper_safety	= 300;
const int pos_dump_safety	= 100;

const int pos_servo_dump_closed		= 202;
const int pos_servo_dump_open		= 171;
const int pos_servo_hopper_down		= 104;	//servo_hopper_T (128+); servo_hopper_B (128-)
const int pos_servo_hopper_center	= 95;
const int pos_servo_hopper_goal		= 72;	// TODO
const int pos_servo_pickup_up		= 69;	//servo_pickup_L (127+); servo_pickup_R (128-)
const int pos_servo_pickup_large	= 15;
const int pos_servo_pickup_small	= 13;

#endif // FINAL_H
