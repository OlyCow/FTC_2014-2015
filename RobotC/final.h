#ifndef FINAL_H
#define FINAL_H

const tMotor encoder_lift	= motor_LT;
const tMotor encoder_dist	= mtr_S1_C3_1;

const int pos_lift_bottom	= 0;
const int pos_lift_low		= 0;
const int pos_lift_medium	= 0;
const int pos_lift_high		= 0;
const int pos_lift_center	= 0;
const int pos_lift_top		= 0;

const int pos_servo_dump_closed		= 140;
const int pos_servo_dump_open		= 120;
const int pos_servo_hopper_down		= 101;	//servo_hopper_T (128+); servo_hopper_B (128-)
const int pos_servo_hopper_center	= 95;
const int pos_servo_hopper_goal		= 72;
const int pos_servo_pickup_up		= 69;	//servo_pickup_L (127+); servo_pickup_R (128-)
const int pos_servo_pickup_large	= 15;
const int pos_servo_pickup_small	= 13;

#endif // FINAL_H
