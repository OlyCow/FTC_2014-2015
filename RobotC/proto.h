// Motor definitions (for encoders).
tMotor encoder_L	= motor_L;
tMotor encoder_R	= motor_R_B;
tMotor encoder_lift	= motor_lift_A;

// Lift position values.
const int LIFT_BOTTOM	= 0;
const int LIFT_LOW		= 1700;
const int LIFT_MID		= 4300;
const int LIFT_HIGH		= 6600;
const int LIFT_CENTER	= 8500;
const int LIFT_TOP		= 9200;

// Servo position values.
const int pos_dump_open 	= 190;
const int pos_dump_closed	= 240;
