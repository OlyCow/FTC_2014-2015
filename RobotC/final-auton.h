#ifndef FINAL_AUTON
#define FINAL_AUTON

bool DriveForward(int encoder_count);
bool DriveBackward(int encoder_count);
bool TurnLeft(int degrees);
bool TurnRight(int degrees);

bool Drive(int encoder_count);
bool Turn(int degrees);

int target_dist_disp = 0;
int pos_dist_disp = 0;
float error_dist_disp = 0.0;
float error_sum_dist_disp = 0.0;
float power_dist_disp = 0.0;
float term_P_dist = 0.0;
float term_I_dist = 0.0;

int target_angle_disp = 0;
int curr_angle_disp = 0;
float error_angle_disp = 0.0;
float error_sum_angle_disp = 0.0;
float power_angle_disp = 0.0;
float term_P_angle = 0.0;
float term_I_angle = 0.0;

bool DriveForward(int encoder_count)
{
	int temp_count = encoder_count * -1;
	return Drive(temp_count);
}
bool DriveBackward(int encoder_count)
{
	return Drive(encoder_count);
}
bool TurnLeft(int degrees)
{
	int temp_degrees = degrees * -1;
	return Turn(temp_degrees);
}
bool TurnRight(int degrees)
{
	return Turn(degrees);
}

bool Drive(int encoder_count)
{
	target_dist_disp = encoder_count;

	bool isSuccess = false;

	int timer_watchdog = 0;
	Time_ClearTimer(timer_watchdog);
	const float watchdog_encoder_rate = 2.3;
	const float watchdog_base = 1000.0;
	int time_limit = (int)round((float)abs(encoder_count*watchdog_encoder_rate)+watchdog_base);
	const int acceptable_error = 150;
	const int finish_limit = 750; // msec
	bool isFinishing = false;
	int timer_finish = 0;
	Time_ClearTimer(timer_finish);

	const float kP = 0.01178;
	const float kI = 0.00445;
	const float I_term_decay_rate = 0.87;

	int count_init = Motor_GetEncoder(encoder_R);
	int pos_dist = Motor_GetEncoder(encoder_R) - count_init;
	int error = 0;
	float error_sum = 0.0;
	float power = 0.0;
	float power_prev = 0.0;

	while (true) {
		power_prev = power;
		pos_dist = Motor_GetEncoder(encoder_R) - count_init;
		error = pos_dist - encoder_count;
		error_sum *= I_term_decay_rate;
		error_sum += error;
		power = kP*error + kI*error_sum;
		power = Math_Limit(power, 85.0);
		if (abs(power)<10.0) {
			power = 0.0;
		} else if (abs(power)<25.0) {
			power = sgn(power) * 25.0;
		}

		pos_dist_disp = pos_dist;
		error_dist_disp = (float)(error);
		error_sum_dist_disp = error_sum;
		power_dist_disp = (float)power;

		Motor_SetPower(power, motor_LT);
		Motor_SetPower(power, motor_LB);
		Motor_SetPower(power, motor_RT);
		Motor_SetPower(power, motor_RB);

		if (abs(error) < acceptable_error) {
			if (isFinishing == false) {
				Time_ClearTimer(timer_finish);
			}
			isFinishing = true;
		} else {
			isFinishing = false;
		}

		if ((isFinishing == true)&&(Time_GetTime(timer_finish)>finish_limit)) {
			isSuccess = true;
			break;
		}
		if ((isFinishing == false)&&(Time_GetTime(timer_watchdog)>time_limit)) {
			isSuccess = false;
			break;
		}

		Time_Wait(2);
	}

	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);

	return isSuccess;
}

bool Turn(int degrees)
{
	target_angle_disp = degrees;

	bool isSuccess = false;

	int timer_watchdog = 0;
	Time_ClearTimer(timer_watchdog);
	const float watchdog_degree_rate = 0.06;
	const float watchdog_base = 750.0;
	int time_limit = (int)round((float)abs(degrees)*watchdog_degree_rate+watchdog_base);
	const float acceptable_error = 0.5;

	const int finish_limit = 1000; // msec
	bool isFinishing = false;
	int timer_finish = 0;
	Time_ClearTimer(timer_finish);

	const float kP = 7.6;
	const float kI = 0.98;
	const float I_term_decay_rate = 0.97;

	float heading_init = heading;
	float heading_curr = heading;
	float error = 0.0;
	float error_sum = 0.0;
	float power = 0.0;
	float power_prev = 0.0;

	while (true) {
		power_prev = power;
		heading_curr = heading - heading_init;
		error = heading_curr - (float)degrees;
		error_sum *= I_term_decay_rate;
		if (error > 15) {
			error_sum += error;
		}
		float kP_var = kP;
		float kI_var = kI;
		power = kP_var*error + kI_var*error_sum;
		power = Math_Limit(power, 85.0);
		if (abs(power)<10.0) {
			power = 0.0;
		} else if (abs(power)<25.0) {
			power = sgn(power) * 25.0;
		}

		int power_L = (int)round(power);
		power_L *= -1;
		int power_R = (int)round(power);
		power_R *= 1;

		curr_angle_disp = heading_curr;
		error_angle_disp = error;
		error_sum_angle_disp = error_sum;
		power_angle_disp = power;

		Motor_SetPower(power_L, motor_LT);
		Motor_SetPower(power_L, motor_LB);
		Motor_SetPower(power_R, motor_RT);
		Motor_SetPower(power_R, motor_RB);

		if (abs(error)<acceptable_error) {
			if (isFinishing == false) {
				Time_ClearTimer(timer_finish);
			}
			isFinishing = true;
		} else {
			isFinishing = false;
		}

		if ((isFinishing == true)&&(Time_GetTime(timer_finish)>finish_limit)) {
			isSuccess = true;
			break;
		}
		if ((isFinishing == false)&&(Time_GetTime(timer_watchdog)>time_limit)) {
			isSuccess = false;
			break;
		}

		Time_Wait(2);
	}

	Motor_SetPower(0, motor_LT);
	Motor_SetPower(0, motor_LB);
	Motor_SetPower(0, motor_RT);
	Motor_SetPower(0, motor_RB);

	return isSuccess;
}

#endif // FINAL_AUTON
